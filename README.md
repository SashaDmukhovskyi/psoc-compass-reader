# PSoC Compass Reader

An embedded C learning project for the CY8CKIT-043. It reads a GY-271
magnetometer over I2C, filters its measurements and compares three methods of
calculating the XY angle: a double-precision baseline, an integer arctangent
series and the libfixmath library.

The tested module acknowledges the 7-bit address `0x0D`, returns `0xFF` from
register `0x0D` and operates with the QMC5883L register map. These observations
indicate a **QMC5883L-compatible device**, not unique proof of its manufacturer.
HMC5883L firmware is not interchangeable with this driver.

## Hardware and tools

| Item | Configuration |
|---|---|
| Development kit | CY8CKIT-043 |
| MCU | CY8C4247AZI-M485, PSoC 4200M |
| CPU | ARM Cortex-M0 |
| Nominal SYSCLK | 48 MHz |
| Environment | PSoC Creator |
| Tested compiler | ARM GCC 5.4-2016-q2-update |
| Benchmark build | Debug, `-Og`, LTO disabled |
| I2C | SCB master, 100 kbit/s, 7-bit address `0x0D` |
| UART | 115200 bit/s, 8 data bits, no parity, 1 stop bit |
| PC console | PowerShell; `COM3` on the tested computer |

## Wiring

| GY-271 signal | PSoC connection |
|---|---|
| VCC | VDDD / 3.3 V |
| GND | Common ground |
| SCL | P4[0] |
| SDA | P4[1] |
| DRDY | P2[0], according to the project wiring documentation |

Verify physical pin assignments in `Compass_reader.cydwr` before connecting
the module. The GPIO component is named `DRDY`, uses a rising-edge interrupt
and connects to the `isr_DRDY` interrupt component.

## Features

- Checked I2C register writes, single-register reads and six-byte data reads.
- Blocking I2C operation timeout of 100 ms.
- Hardware DRDY interrupt with a short ISR.
- Signed 16-bit X/Y/Z measurements and signed 32-bit IIR state.
- Continuous measurements configured for 200 Hz, independently of UART output.
- Three heading algorithms with accuracy and cycle comparisons.
- Predefined direction tests and a repeated timing test.
- Unsigned 32-/64-bit addition, subtraction and multiplication batch tests.
- Structured UART logs and a successful-read rate counter.
- Configurable smoothing, output period and startup tests through source settings.

## Project structure

| File | Responsibility |
|---|---|
| `Compass_reader.cydsn/main.c` | Startup, DRDY handler, reading/filtering loop and output scheduling |
| `qmc5883l.c`, `qmc5883l.h` | Sensor register access, initialization and raw data |
| `compass.c`, `compass.h` | IIR filter and three angle calculation functions |
| `benchmark.c`, `benchmark.h` | SysTick setup and elapsed-cycle calculation |
| `compass_tests.c`, `compass_tests.h` | Comparison output, direction tests and arithmetic tests |
| `fix16.c`, `fix16_trig.c`, `fix16_sqrt.c` | Imported libfixmath implementation |
| `fix16.h`, `int64.h` | Imported libfixmath headers |
| `Compass_reader.cyprj` | PSoC Creator project and build settings |
| `Compass_reader.cydwr` | Physical pin assignments |
| `TopDesign/TopDesign.cysch` | Peripheral schematic |
| `LICENSE-libfixmath.txt` | Third-party MIT license, stored in the repository root |

The source/header filenames without a directory prefix in this table are in
`Compass_reader.cydsn`. Generated source and build output are excluded from
version control.

## Sensor initialization and data reading

Initialization writes the following registers, checking each operation:

| Register | Value | Purpose |
|---|---|---|
| `0x0A` | `0x80` | Software reset, followed by a 10 ms delay |
| `0x0B` | `0x01` | Set/reset period |
| `0x0A` | `0x00` | Enable the physical data-ready output |
| `0x09` | `0x0D` | OSR 512, +/-2 G, 200 Hz, continuous mode |

The application reads back register `0x09` and reports whether it matches
`0x0D`. The I2C timeout is a maximum wait for an operation, not a sampling delay.

`DRDY_handler` clears the GPIO interrupt source and sets `dataReady`. The main
loop takes and clears that flag inside a short critical section, then reads and
filters the measurement outside the ISR.

A data read uses START in write mode, writes the register pointer `0x00`, sends
a repeated START in read mode and reads six bytes. The master sends ACK after
the first five bytes and NACK after the last byte, then sends STOP.

| Registers | Measurement |
|---|---|
| `0x00`, `0x01` | X low byte, X high byte |
| `0x02`, `0x03` | Y low byte, Y high byte |
| `0x04`, `0x05` | Z low byte, Z high byte |

Each little-endian pair is combined using a shift and bitwise OR and converted
to `int16`. Filtering is performed once per successful read.

## Settings

Edit the following values in `main.c`, rebuild and program the board:

```c
#define FILTER_DIVISOR   (8u)
#define OUTPUT_PERIOD_MS (1000u)
#define RUN_STARTUP_TESTS (1u)
```

- `FILTER_DIVISOR`: 1..65535. Larger values smooth more but respond more slowly.
- `OUTPUT_PERIOD_MS`: 1..1000 ms. This controls comparison output, not sensor ODR.
- `RUN_STARTUP_TESTS`: set to 1 to run direction, repeated timing and arithmetic
  tests at startup; set to 0 to skip them.

The sensor remains configured for 200 Hz. UART is blocking, so very frequent
output can reduce the number of samples the main loop services. With a 1000 ms
output period, observed steady-state rates were approximately 197..199
successful reads per second. This is not proof that every conversion was read.

The IIR update is:

```text
filtered += (measurement - filtered) / FILTER_DIVISOR
```

The first sample initializes the state. Integer division truncates toward zero,
so differences smaller than the divisor may produce no update. Filter response
in seconds also depends on the rate of successful updates.

## Angle algorithms

All methods use the same filtered X/Y values. Z is reported but is not used for
tilt compensation. The displayed angle is measured from the sensor's positive
X axis and normalized around a full circle.

| Method | Implementation | Result representation |
|---|---|---|
| Double | `Compass_CalculateHeadingDoubleRadians`, standard `atan2()` | Double radians |
| Series | `Compass_CalculateHeadingFixed`, own integer implementation | Q16.16 radians |
| Library | `Compass_CalculateHeadingLibrary`, libfixmath `fix16_atan2()` | Q16.16 radians |

Q16.16 uses an integer scale of 65536. The series implementation evaluates
`t - t^3/3 + t^5/5 - ...` with 12 terms, reduces the argument to approximately
`|t| <= 0.4143` and reconstructs the correct quadrant. A 64-bit intermediate
prevents overflow in bounded fixed-point products. This algorithm uses no
floating-point arithmetic.

libfixmath uses a cubic polynomial approximation for atan2, not the same
arctangent series. It is retained as a third-party comparison rather than a
replacement for the supervisor-requested series implementation.

For the zero vector, the integer wrappers return an arbitrary 0; direction is
undefined. Degree conversion and numerical-error reporting use double outside
the timed intervals. Thus the comparison firmware as a whole is not float-free.

## Timing methodology

SysTick is configured as a free-running 24-bit down-counter clocked by SYSCLK,
with its interrupt disabled. Elapsed cycles are calculated as:

```c
(start - end) & 0x00FFFFFFu
```

Each interval must be shorter than one counter period: approximately 349.5 ms
at nominal 48 MHz. Output scheduling accumulates short polling intervals to
support longer periods. Blocking longer than one counter period would make
that elapsed-time accounting incorrect.

Interrupts are disabled during algorithm measurements. UART formatting,
transmission, I2C reading and IIR updates are outside those timed intervals.
Reported cycles include wrapper calls and measurement overhead; the displayed
overhead is not subtracted. Observed overhead varied between 33 and 38 cycles
across builds. Code layout, compiler options and input values affect results.

At nominal 48 MHz, `time_us = cycles / 48`. Times are estimates based on the
configured clock, not independent measurements of oscillator frequency.

### Representative heading results

The following values are from one hardware run with the conditions above:

| Input X/Y | Double cycles | Series cycles | Library cycles | Series error | Library error |
|---|---:|---:|---:|---:|---:|
| 1000 / 0 | 477 | 2149 | 1182 | 0.000 deg | 0.002 deg |
| 1000 / 1000 | 6895 | 3082 | 731 | 0.000 deg | 0.001 deg |
| 1000 / 414 | 10635 | 3023 | 1091 | 0.000 deg | 0.000 deg |
| 1000 / 415 | 10635 | 3819 | 1091 | 0.000 deg | 0.002 deg |
| 1000 / -1 | 10866 | 2593 | 1181 | 0.001 deg | 0.012 deg |
| -793 / -97 | 11375 | 2757 | 1210 | 0.001 deg | 0.582 deg |

For X=1000, Y=414, 16 repetitions produced the same measured cycle counts.
On that input, the series was about 3.5x faster than the double baseline and
the library about 9.7x faster. Double was faster on axis-aligned special cases,
so integer arithmetic is not universally faster for every input.

Error is the shortest angular distance from the double baseline. It is rounded
to 0.001 degree; `0.000` does not mean exact equality. Angle display truncates
to 0.01 degree, which can show `179.99` for an angle extremely close to 180.
Observed series errors in supplied runs were approximately 0..0.002 degree;
library errors reached 0.582 degree. These are observations, not exhaustive
maximum-error guarantees or measurements of physical compass accuracy.

### Arithmetic results and assembly inspection

`RunArithmeticTests` measures 1000 iterations using volatile unsigned inputs
and results. Operands are 123456 and 37 for 32 bits, and 5000000000 and 37 for
64 bits; the tested results fit their types. These tests compare integer widths
on selected inputs, not signed arithmetic or equal-value operand sets.

| Operation | Cycles / 1000 iterations | Cycles / iteration |
|---|---:|---:|
| ADD32 | 14053 | 14.053 |
| ADD64 | 21052 | 21.052 |
| SUB32 | 15051 | 15.051 |
| SUB64 | 22057 | 22.057 |
| MUL32 | 14053 | 14.053 |
| MUL64 | 86053 | 86.053 |

The iteration costs include volatile loads/stores, loop control and measurement
overhead. They must not be presented as the latency of a single instruction.
In this test, the 64-bit multiplication batch was about 6.1x slower than the
32-bit batch.

Inspection of the generated `compass_tests.lst` confirmed:

| Operation | Generated arithmetic instructions |
|---|---|
| ADD32 | `adds` |
| ADD64 | `adds` followed by `adcs` |
| SUB32 | `subs` |
| SUB64 | `subs` followed by `sbcs` |
| MUL32 | `muls` |
| MUL64 | Call to `__aeabi_lmul` |

64-bit addition/subtraction handles two 32-bit halves and propagates carry or
borrow. Multiplication calls a compiler-runtime helper. Its internal body was
not present in this source-file listing. The listing also confirmed the loops
and memory accesses had not been optimized away.

## Building and programming

1. Open `Compass_reader.cywrk` in PSoC Creator.
2. Check the schematic and physical pin assignments.
3. Select the installed ARM GCC 5.4-2016-q2-update toolchain.
4. Ensure `FIXMATH_NO_CACHE` is set in Compiler Preprocessor Definitions for
   each build configuration used. An unconfigured ARM GCC Generic toolchain
   cannot build until a compiler location is supplied.
5. Save all files and build the project.
6. Connect the CY8CKIT-043 and program it through KitProg.
7. Open the UART port at 115200 baud. Reset the board while the console is open
   to capture the startup tests.

The imported files are compiled directly in PSoC Creator; no CMake or Python
is required. `FIXMATH_NO_CACHE` must apply to the library translation units,
not only be defined inside `main.c`. Its default caches exceed this MCU's
16 KB SRAM budget.

## Reading logs in PowerShell

Replace `COM3` if necessary. Stop the loop with Ctrl+C.

```powershell
$port = New-Object System.IO.Ports.SerialPort("COM3", 115200, "None", 8, "One")
$port.Open()

try {
    while ($port.IsOpen) {
        $data = $port.ReadExisting()
        if ($data.Length -gt 0) {
            Write-Host -NoNewline $data
        }
        Start-Sleep -Milliseconds 20
    }
}
finally {
    if ($port.IsOpen) {
        $port.Close()
    }
}
```

Close other serial monitors if the port reports access denied. Reads return
chunks of a byte stream and may begin or end in the middle of a log line.

### Example live output

```text
[INFO] QMC5883L-compatible chip ID: 0xFF
[INFO] Sensor initialized
[INFO] Configuration confirmed: 200 Hz
[RATE] Reads:198 | Window:1000 ms
[DATA] X:-793 Y:-97 Z:-2393 | Double:186.97 deg | Series:186.97 deg | Library:186.39 deg
[ERROR-DEG] Series:0.001 | Library:0.582
[TIME] Double:11375 | Series:2757 | Library:1210 | Overhead:33 cycles
```

`[DATA]` reports filtered axes. `[ERROR-DEG]` reports numerical disagreement,
not an I2C failure. `[RATE]` counts successful reads, not guaranteed distinct
sensor conversions. `[TEST]`, `[BENCH]` and `[ARITH]` are startup diagnostics.

## Limitations

- No hard-/soft-iron calibration, declination correction or tilt compensation.
- The XY angle is not a verified geographic-north navigation heading.
- Nearby metal, magnets and electrical currents can distort measurements.
- A one-bit DRDY flag can coalesce events; no sample queue is implemented.
- Blocking UART output and critical sections can delay servicing sensor events.
- Sensor overflow/overrun status bits are not monitored by the current driver.
- No runtime console settings, GUI or saved calibration; settings require a rebuild.
- Timing and numerical tests characterize selected inputs/builds, not all cases.

## Third-party library

[libfixmath](https://github.com/PetteriAimonen/libfixmath) supplies Q16.16
arithmetic and the library atan2 comparison. Imported source files retain their
original notices; the MIT license is in `LICENSE-libfixmath.txt`.

The project's own arctangent series remains separate from the library's cubic
approximation. Floating-point conversion helpers in the library are not used
by the integer angle wrapper.
