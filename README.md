# PSoC Compass Reader

An embedded C project for the CY8CKIT-043. It reads a GY-271 magnetometer
over I2C, filters its measurements and compares two XY angle calculations:

- A double-precision reference using standard `atan2()`.
- An integer arctangent series using Q16.16 fixed-point arithmetic.

The project does not require libfixmath.

The tested module acknowledges address `0x0D`, returns `0xFF` from register
`0x0D` and uses the QMC5883L register map. These observations indicate a
QMC5883L-compatible device; they do not uniquely identify its manufacturer.

## Hardware and tools

| Item | Configuration |
|---|---|
| Development kit | CY8CKIT-043 |
| MCU | CY8C4247AZI-M485, PSoC 4200M |
| CPU | ARM Cortex-M0 |
| Nominal SYSCLK | 48 MHz |
| Environment | PSoC Creator |
| Previously documented toolchain | ARM GCC 5.4-2016-q2-update |
| Previously documented benchmark settings | Debug, `-Og`, LTO disabled |
| I2C | SCB master, configured for 100 kbit/s |
| Sensor address | `0x0D`, 7-bit |
| UART | 115200 baud, 8N1 |

Verify the active toolchain and optimization settings in PSoC Creator when
reproducing timing results. Changes in compiler settings affect cycle counts.

## Wiring

| GY-271 signal | PSoC connection |
|---|---|
| VCC | VDDD / 3.3 V |
| GND | Common ground |
| SCL | P4[0] |
| SDA | P4[1] |
| DRDY | P2[0] |

Check `Compass_reader.cydwr` before connecting the module.
The `DRDY` GPIO uses a rising-edge interrupt connected to `isr_DRDY`.

## Project structure

Source and header files are located in `Compass_reader.cydsn`.

| File | Responsibility |
|---|---|
| `main.c` | Startup, DRDY handling, measurement processing and output scheduling |
| `qmc5883l.c/.h` | Sensor initialization and I2C register access |
| `compass.c/.h` | Integer IIR filter and two heading algorithms |
| `benchmark.c/.h` | SysTick configuration and elapsed-cycle measurement |
| `compass_tests.c/.h` | Direction tests, timing, error reporting and arithmetic tests |
| `Compass_reader.cyprj` | Project and build settings |
| `Compass_reader.cydwr` | Physical pin assignments |
| `TopDesign/TopDesign.cysch` | Peripheral schematic |

Open `Compass_reader.cywrk` from the repository root.

## Sensor initialization

Each register write is checked for success.

| Register | Value | Purpose |
|---|---|---|
| `0x0A` | `0x80` | Software reset, followed by a 10 ms delay |
| `0x0B` | `0x01` | Set/reset period |
| `0x0A` | `0x00` | Enable the physical data-ready output |
| `0x09` | `0x0D` | OSR 512, +/-2 G, 200 Hz, continuous measurement |

The application checks the chip ID before initialization and reads back
register `0x09` afterward. Configuration verification failures are reported,
but the main loop continues.

Driver functions return `1` on success and `0` on failure.
The underlying PSoC I2C API uses a status value of `0` for success.

The 100 ms timeout applies to individual blocking I2C operations, not the
complete transaction or the sensor sampling interval.

## Measurement processing

The DRDY interrupt handler clears the GPIO interrupt and sets `dataReady`.
The main loop takes and clears this flag inside a short critical section.

Sensor reading and filtering occur outside the interrupt handler.

A measurement read performs:

1. START in write mode.
2. Write the register pointer `0x00`.
3. Repeated START in read mode.
4. Read six bytes, acknowledging the first five.
5. Send NACK after the final byte and then STOP.

| Registers | Measurement |
|---|---|
| `0x00`, `0x01` | X low byte, X high byte |
| `0x02`, `0x03` | Y low byte, Y high byte |
| `0x04`, `0x05` | Z low byte, Z high byte |

Byte pairs are combined into signed 16-bit values. The output structure is
updated only after the complete read and STOP succeed.

## Settings and filtering

Edit these settings in `main.c`, rebuild and program the board:

```c
#define FILTER_DIVISOR   (8u)
#define OUTPUT_PERIOD_MS (1000u)
#define RUN_STARTUP_TESTS (1u)
```

| Setting | Meaning |
|---|---|
| `FILTER_DIVISOR` | 1..65535; larger values provide stronger smoothing |
| `OUTPUT_PERIOD_MS` | 1..1000 ms; comparison output period |
| `RUN_STARTUP_TESTS` | Enables direction, timing and arithmetic tests at startup |

The output period does not change the configured sensor rate of 200 Hz.

The integer IIR filter uses:

```c
filtered += (measurement - filtered) / filterDivisor;
```

The first measurement initializes the state directly. Integer division
truncates toward zero, so differences smaller than the divisor may produce
no update. Filter response depends on the successful update rate.

The supplied hardware run showed approximately 198..200 successful reads
per second after startup. The first window reported 202 reads.
These counts do not prove that every sensor conversion was read exactly once.

## Heading calculations

Both algorithms use the same filtered X/Y coordinates.
Z is reported but is not used for tilt compensation.

| Method | Function | Return value |
|---|---|---|
| Double | `Compass_CalculateHeadingDoubleRadians()` | Double radians |
| Series | `Compass_CalculateHeadingFixed()` | Q16.16 radians |

### Integer series

Q16.16 represents a value using an integer scale of 65536.

The implementation:

1. Divides the smaller coordinate magnitude by the larger, producing a
   fixed-point ratio in `[0, 1]`.
2. Reduces the arctangent argument to approximately `|t| <= 0.4143`.
3. Evaluates 12 terms of the series:

   ```text
   atan(t) = t - t^3/3 + t^5/5 - t^7/7 + ...
   ```

4. Reconstructs the quadrant using the original coordinate signs.

The algorithm uses integer arithmetic throughout. Fixed-point multiplication
and ratio calculations use 64-bit intermediates.

For the zero vector, direction is undefined; the integer function returns
zero by convention.

### Degree conversion and error

The series result is converted to integer millidegrees using:

```text
millidegrees = round(fixedRadians * 180000 / 205887)
```

This conversion uses integer arithmetic.

The double reference is converted once to integer millidegrees after
`atan2()` finishes. Subsequent error calculations and output formatting
use integers.

For example, `60587` represents `60.587 degrees`.

Error is the shortest angular distance between the two rounded angles,
accounting for the 0/360-degree boundary.

`0.000` means the rounded millidegree values agree. It does not imply exact
numerical equality. The comparison firmware still uses double arithmetic
for the reference calculation and its initial unit conversion.

## Timing methodology

SysTick is a free-running 24-bit down-counter clocked by SYSCLK.
Its periodic interrupt is disabled.

Elapsed cycles are calculated as:

```c
(start - end) & 0x00FFFFFFu
```

The interval must be shorter than `2^24` cycles: approximately 349.5 ms at
48 MHz. Longer output periods are tracked by accumulating short polling
intervals.

Interrupts are disabled during algorithm measurements.
I2C reading, filtering, degree conversion, error calculation, formatting
and UART transmission are outside the measured algorithm intervals.

Reported cycles include measurement overhead and function calls.
The displayed overhead is not subtracted.

At nominal 48 MHz:

```text
time_us = cycles / 48
```

These times depend on the configured clock and are not independent
measurements of oscillator frequency.

## Hardware results

The following results were supplied from the version without libfixmath.

| X | Y | Double cycles | Series cycles | Reported difference |
|---:|---:|---:|---:|---:|
| 1000 | 0 | 471 | 2177 | 0.000 deg |
| 1000 | 1000 | 6887 | 3012 | 0.000 deg |
| 0 | 1000 | 491 | 2178 | 0.000 deg |
| -1000 | 1000 | 7318 | 3015 | 0.000 deg |
| -1000 | 0 | 492 | 2180 | 0.000 deg |
| -1000 | -1000 | 7624 | 3016 | 0.001 deg |
| 0 | -1000 | 693 | 2183 | 0.000 deg |
| 1000 | -1000 | 7091 | 3017 | 0.001 deg |
| 1000 | 414 | 10627 | 3014 | 0.000 deg |
| 1000 | 415 | 10627 | 3755 | 0.001 deg |
| 1000 | -1 | 10858 | 2570 | 0.001 deg |

For `X=1000, Y=414`, all 16 timing repetitions produced:

| Method | Cycles | Approximate time at 48 MHz |
|---|---:|---:|
| Double | 10627 | 221.4 us |
| Series | 3014 | 62.8 us |

The series was approximately 3.53 times faster on this input.
The double implementation was faster on axis-aligned special cases.

The supplied live samples showed reported differences of 0..0.002 degrees.
These are observations on selected inputs, not an exhaustive error bound
or a measure of physical compass accuracy.

## 32-bit and 64-bit arithmetic

Each test measures 1000 iterations with volatile unsigned operands and results.

Operands are `123456` and `37` for 32 bits, and `5000000000` and `37` for
64 bits. Results fit their respective types.

| Operation | Cycles / 1000 iterations | Cycles / iteration |
|---|---:|---:|
| ADD32 | 15049 | 15.049 |
| ADD64 | 22055 | 22.055 |
| SUB32 | 14051 | 14.051 |
| SUB64 | 21050 | 21.050 |
| MUL32 | 15049 | 15.049 |
| MUL64 | 92052 | 92.052 |

The 64-bit multiplication batch was approximately 6.12 times slower than
the 32-bit batch.

Measurements include loop control, volatile loads/stores and timing overhead.
They do not represent individual instruction latency. These tests use
different operand magnitudes and do not measure signed arithmetic or division.

An earlier build's assembly listing showed `adds`/`adcs` for 64-bit addition,
`subs`/`sbcs` for subtraction, and a call to `__aeabi_lmul` for multiplication.
Inspect the current build's listing before attributing those exact instruction
sequences to this build.

## Building and programming

1. Open `Compass_reader.cywrk` in PSoC Creator.
2. Verify schematic, pin assignments, toolchain and build configuration.
3. Save all files and run Clean and Build.
4. Connect the CY8CKIT-043 and program it through KitProg.
5. Open UART at 115200 baud, 8N1.
6. Reset the board while the serial monitor is open to capture startup tests.

No libfixmath source files or `FIXMATH_NO_CACHE` definition are required.
The standard math library is still used for the double `atan2()` reference.

## Reading logs in PowerShell

Replace `COM3` with the actual port. List available ports with:

```powershell
[System.IO.Ports.SerialPort]::GetPortNames()
```

Run the monitor and stop it with Ctrl+C:

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

Close other serial monitors if the port reports access denied.

### Example live output

```text
[INFO] QMC5883L-compatible chip ID: 0xFF
[INFO] Sensor initialized
[INFO] Configuration confirmed: 200 Hz
[RATE] Reads:199 | Window:1000 ms
[DATA] X:2460 Y:4362 Z:-716 | Double:60.579 deg | Series:60.580 deg
[ERROR-DEG] Series:0.001
[TIME] Double:13468 | Series:3706 | Overhead:34 cycles
```

| Prefix | Meaning |
|---|---|
| `[INFO]` | Startup and configuration messages |
| `[TEST]` | Predefined direction checks |
| `[BENCH]` | Repeated timing case |
| `[ARITH]` | Integer arithmetic batch measurements |
| `[RATE]` | Successful reads and measurement window |
| `[DATA]` | Filtered coordinates and calculated angles |
| `[ERROR-DEG]` | Numerical angle difference |
| `[TIME]` | Algorithm cycle counts and timing overhead |
| `[ERROR]` | Sensor or communication error message |

## Limitations

- No hard-/soft-iron calibration, declination correction or tilt compensation.
- The XY angle is not a verified geographic-north heading.
- Nearby metal, magnets and electrical currents can distort readings.
- Multiple DRDY events can merge into one pending flag.
- Blocking UART output and critical sections can delay measurement processing.
- Sensor overflow/overrun status bits are not monitored.
- Long blocking intervals can invalidate SysTick elapsed-time accounting.
- Settings require rebuilding; there is no runtime configuration interface.
- Reported angle differences compare rounded millidegree values.
- Tests cover selected inputs and do not establish a global accuracy bound.
