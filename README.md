# PSoC Compass Reader

A learning-oriented embedded project that reads a GY-271 magnetometer with a
PSoC 4200M, filters the measurements, calculates a heading, and sends diagnostic
output to a UART debug console.

The module used in this project is sold as **GY-271 / HMC5883L**, but the device
installed on the tested board is actually a **QMC5883L**. It responds at the
7-bit I2C address `0x0D` and returns `0xFF` from its chip identification register.

## Features

- QMC5883L detection and chip ID verification
- I2C register read and write operations
- Continuous measurement mode at 50 Hz
- Hardware `DRDY` interrupt
- Raw signed 16-bit X, Y, and Z measurements
- First-order IIR low-pass filtering
- Heading calculation in degrees
- Structured UART diagnostic logs
- I2C error handling and operation timeouts

## Hardware

- **Development kit:** CY8CKIT-043
- **MCU:** PSoC 4200M, CY8C4247AZI-M485
- **Magnetometer module:** GY-271 with QMC5883L
- **Development environment:** PSoC Creator
- **Debug interface:** UART over the kit's USB connection

## Wiring

| GY-271 pin | PSoC connection | Purpose |
|---|---|---|
| `VCC` | `VDDD / 3.3 V` | Sensor power |
| `GND` | `GND` | Common ground |
| `SCL` | `P4[0]` | I2C clock |
| `SDA` | `P4[1]` | I2C data |
| `DRDY` | `P2[0]` | Data-ready interrupt |

## Peripheral configuration

### I2C

- Mode: Master
- Bus speed: 100 kbit/s
- Address format: 7-bit
- Sensor address: `0x0D`

### UART

- Baud rate: 115200 bit/s
- Data bits: 8
- Parity: None
- Stop bits: 1

### DRDY interrupt

- Pin: `P2[0]`
- Trigger: Rising edge
- ISR component: `isr_DRDY`

The interrupt handler only clears the GPIO interrupt source and sets a
`volatile` flag. I2C reads, filtering, calculations, and UART output are handled
by the main loop so that the ISR remains short.

## Sensor configuration

The QMC5883L is configured with the following sequence:

1. Perform a software reset.
2. Wait 10 ms for the sensor to restart.
3. Configure the recommended set/reset period.
4. Enable the physical `DRDY` output.
5. Start continuous measurements.

Control Register 1 is written with `0x05`, which selects:

- oversampling ratio: 512
- full-scale range: +/-2 G
- output data rate: 50 Hz
- continuous measurement mode

## How it works

1. The application starts the UART and I2C peripherals.
2. It probes address `0x0D` and checks for an I2C acknowledgement.
3. It reads register `0x0D` and expects the QMC5883L chip ID `0xFF`.
4. The sensor is reset and configured for continuous operation.
5. A rising edge on `DRDY` invokes the interrupt handler.
6. The handler sets `dataReady` and immediately returns.
7. The main loop reads six consecutive bytes from registers `0x00` through
   `0x05`.
8. Each low/high byte pair is combined into a signed 16-bit axis value.
9. The samples are filtered, converted to a heading, and printed over UART.

The QMC5883L stores each measurement in little-endian order:

| Registers | Measurement |
|---|---|
| `0x00`, `0x01` | X low byte, X high byte |
| `0x02`, `0x03` | Y low byte, Y high byte |
| `0x04`, `0x05` | Z low byte, Z high byte |

## IIR filter

A first-order IIR low-pass filter reduces short-term measurement noise:

```text
filtered += (measurement - filtered) / 8
```

This is equivalent to a smoothing coefficient of `1/8`. A smaller divisor
follows changes faster but removes less noise; a larger divisor produces a
smoother result with more delay.

The filter state uses signed 32-bit values so that negative magnetometer
measurements and intermediate subtraction results are handled correctly.

## Heading calculation

The horizontal heading is calculated from the filtered X and Y axes:

```text
heading = atan2(Y, X) * 180 / pi
```

Negative results are normalized to the `0...359` degree range. The project uses
the C math library for `atan2()`.

## Building and programming

1. Open `Compass_reader.cywrk` in PSoC Creator.
2. Check the pin assignments in `Compass_reader.cydwr`.
3. Select **Build -> Generate Application**.
4. Select **Build -> Build Compass_reader**.
5. Connect the CY8CKIT-043 through USB.
6. Select **Debug -> Program**.
7. Open the UART port at 115200 baud.

## Reading logs in PowerShell

Replace `COM3` if Windows assigned a different port:

```powershell
$uart = New-Object System.IO.Ports.SerialPort COM3,115200,None,8,One
$uart.Open()

try {
    while ($true) {
        if ($uart.BytesToRead -gt 0) {
            Write-Host -NoNewline $uart.ReadExisting()
        }

        Start-Sleep -Milliseconds 20
    }
}
finally {
    if ($uart.IsOpen) {
        $uart.Close()
    }
}
```

If Windows reports that access to the port is denied, close every other serial
monitor or PowerShell session that may already have the port open.

## Example output

```text
[INFO] Compass reader started
[INFO] Device found at 0x0D
[INFO] QMC5883L chip ID confirmed: 0xFF
[INFO] QMC5883L initialized
[DATA] RAW X:3310 Y:957 Z:-1485 | IIR X:3259 Y:964 Z:-1490 | Heading:16 deg
```

Raw values react immediately to measurement noise. IIR values change more
smoothly, and the heading is calculated from the filtered X and Y axes.

## HMC5883L and QMC5883L are not interchangeable

Despite similar module names, the two sensors use different addresses and
register maps:

| Device | Typical 7-bit address | Identification used here |
|---|---:|---|
| HMC5883L | `0x1E` | Did not acknowledge on the tested module |
| QMC5883L | `0x0D` | Register `0x0D` returned `0xFF` |

Code written for the HMC5883L should not be used unchanged with the QMC5883L.

## Project structure

```text
Compass_reader.cywrk
Compass_reader.cydsn/
|-- main.c
|-- Compass_reader.cyprj
|-- Compass_reader.cydwr
`-- TopDesign/
    `-- TopDesign.cysch
```

Generated source files and build output are excluded from version control.

## Learning objectives covered

- I2C electrical behavior and transaction structure
- Register-based sensor configuration
- Multi-byte I2C reads with ACK and final-byte NACK
- Repeated START and STOP conditions
- Hardware interrupts and ISR/main-loop communication
- C bitwise operations and configuration masks
- First-order IIR filtering
- UART debugging and structured logging

## Limitations

- The magnetometer is not hard-iron or soft-iron calibrated.
- Magnetic declination is not applied.
- There is no tilt compensation because no accelerometer is used.
- Nearby metal, magnets, current-carrying wires, and the development board can
  distort the measurements.
- The displayed heading is suitable for demonstrating the algorithm, but it
  should not be treated as an accurate navigation heading.

## Possible improvements

- Add hard-iron and soft-iron calibration.
- Save calibration parameters in flash or EEPROM.
- Add magnetic declination for geographic north.
- Add an accelerometer for tilt compensation.
- Split the sensor and filter logic into separate `.c` and `.h` modules.
- Throttle UART logs while continuing to sample the sensor at 50 Hz.
- Replace `sprintf()` with bounded formatting where supported.
