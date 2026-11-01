### ESP32 serial controller for ROS 2 robot hardware

The active firmware in `src/main.cpp` communicates with the host using serial
commands at 115200 baud, controls the ODrive over `Serial2`, and reads battery
voltage and current from an INA226 sensor with a 2 mΩ shunt.


The firmware creates the following FreeRTOS tasks after serial initialization
and an INA226 detection attempt:

| Task | Schedule | Responsibility |
| --- | --- | --- |
| Host (priority 2) | Poll serial once per RTOS tick | Reply from a snapshot; enqueue motor commands and acknowledge immediately |
| ODrive (priority 3) | Every 20 ms | Activate ODrive, write motor velocities, read both positions, publish a completed wheel pair |
| Battery (priority 1) | Every 200 ms | Read INA226 and publish voltage/current |
| IMU (priority 1) | Every 20 ms | Read Adafruit MPU6050 acceleration/gyro events; retry initialization after errors |

Only the ODrive task accesses `Serial2` or calls `ODriveArduino`, including during
activation. Snapshot mutexes cover memory copies/updates only. The one-item motor
queue overwrites older pending commands. A command is applied on the next ODrive
cycle; its CRLF acknowledgement means queued, not physically applied. After
500 ms without a valid motor command, the ODrive task writes zero to both axes.
Expiry uses the host receive time, even if queue consumption is delayed.

Commands remain CR-terminated (`e\r`, `m <left> <right>\r`, or `\r` for heartbeat);
LF is ignored. Malformed/nonfinite motor commands do not refresh the watchdog.
Overlong command lines are discarded through the next CR. The ready message
follows the original ODrive activation sequence and the first acquisition attempt.

The INA226 voltage retains a 50-sample moving average (a 10-second window at
normal acquisition). At startup it averages only collected samples. Current
uses the latest converted sample in signed mA, with no additional filter.
Both firmware builds use `include/ina226.h`, defaulting to address `0x40` and a
0.002 Ω shunt. It verifies chip identity and configuration/calibration readback,
with 16-sample hardware averaging and approximately 35.2 ms per conversion.
Override `INA226_ADDRESS` and `INA226_SHUNT_OHMS` using `build_flags` in the
appropriate PlatformIO environment if the hardware differs.
The battery voltage retains the existing `VBUS + shunt voltage` supply estimate;
this assumes VBUS is wired to IN−.

Failed I2C reads, incomplete conversions, math overflow, or lost configuration/
calibration leave the last valid battery snapshot and voltage filter unchanged.
Runtime battery reads do not print asynchronous text on the host serial port.
If INA226 initialization fails, the firmware prints a warning and continues with
the host, ODrive, and IMU tasks. The battery task is skipped and battery voltage
and current remain zero (unavailable). Restart the controller after connecting
the INA226; detection is attempted only at startup. Restart is also needed after
a sensor reset that clears its configuration/calibration.

The `e` reply is one space-separated line with these fields:

| Position (1-based) | Value | Unit |
| --- | --- | --- |
| 1–4 | Left position, right position, battery voltage, battery current | Existing ODrive position units, V, mA |
| 5–6 | Right position read, left position read | µs |
| 7 | ODrive task start-to-start period | µs |
| 8 | ODrive cycle work, including motor writes and position reads | µs |
| 9 | Both motor writes, including UART TX completion | µs |
| 10 | Age of the older wheel reading when the reply is formatted | µs |
| 11 | Completed wheel-pair sequence number | Counter |
| 12 | Cycles whose work exceeded 20 ms | Counter |
| 13 | Failed wheel-pair acquisition attempts | Counter |
| 14 | Interval between completed older-wheel readings | µs |
| 15 | Command watchdog active (zero target) | 0 or 1 |

The first six fields retain their existing order; parsers consuming the first
four values continue to work. No unsolicited timing logs are sent on the host
UART. The task period and sample interval are zero until a second observation.
Age is `4294967295` until the first valid pair, and age/interval saturate at that
value during a long outage rather than wrapping to a falsely fresh value.
A failed read leaves positions,
sample timestamp and sequence unchanged; read-duration fields describe the latest
attempt (zero when a read was skipped).

Each position read has a 12 ms timeout. Invalid, partial or missing replies do
not become fresh samples. Following a failure, the ODrive task continues writing
safe motor targets every cycle while draining late RX data and waiting for a
20 ms quiet interval before querying positions again. Missed deadlines cause a
one-tick yield and reschedule, rather than repeated catch-up cycles. This bounds
normal missing-reply waits; two slow successful reads can still exceed 20 ms and
must be diagnosed on hardware.

### MPU6050 wiring and acquisition

Connect the Adafruit MPU6050 **VIN to 3V3, GND to GND, SDA to GPIO21,
and SCL to GPIO22**. These are the same I2C lines used by INA226 (normally
address `0x40`). The MPU6050 defaults to `0x68`; for AD0 high, add
`build_flags = -DMPU6050_ADDRESS=0x69` to the PlatformIO environment.

Startup initializes `Adafruit_MPU6050`, then the IMU task owns it. Initialization sets
±2 g acceleration, ±250 degrees/s gyro range, a 21 Hz low-pass filter, and
a 50 Hz hardware sample rate. `getEvent()` supplies acceleration in m/s²
(including gravity) and angular velocity in rad/s. Samples remain in the sensor's
XYZ axes; install the board to match the robot's `imu_1` frame or adjust that
frame's fixed joint to match the physical mounting. No quaternion is estimated.

On every controller boot, **keep the robot stationary on level ground with the
sensor mounted flat, Z pointing upward**. Before creating tasks or activating
ODrive, firmware holds the ODrive reset pin low, allows one second for settling,
and collects 250 IMU samples with 20 ms between reads (about five seconds plus
I2C time). The sensor itself produces data at 50 Hz.
The existing ODrive activation sequence follows calibration, so the ready
message arrives about six seconds later than before.

Successful calibration subtracts average gyro bias and accelerometer offsets
relative to `(0, 0, 9.80665)` m/s². Gravity stays in the output; at rest expect
approximately zero gyro and `(0, 0, 9.81)` acceleration. Corrections are held in
RAM until restart and retained across runtime I2C reconnection. Reconnection
does not trigger another calibration while the robot may be moving.

The startup serial log reports `calibration complete` or `calibration skipped`
with a reason. Missing IMU, failed/nonfinite samples, excessive rotation,
unstable readings, implausible level poses, or a ten-second sampling timeout
skip correction and continue controller startup with uncalibrated readings
when available. Runtime IMU detection still retries normally. Per-axis sample
standard deviations must be at most 0.15 m/s² and 0.01 rad/s; absolute gyro
readings must stay within 0.15 rad/s. Acceleration magnitude must be within
0.75–1.25 g, X/Y means within ±2 m/s², and mean Z at least +0.75 g.

This is a single-position offset calibration, not sensitivity or mounting-angle
calibration. It cannot distinguish unknown tilt or scale error from offset,
nor reliably detect steady slow rotation. Always keep the robot still and
level throughout startup; the sensor's original 14% magnitude discrepancy
still warrants investigation. The standalone `sensor-debug` environment keeps
reporting uncorrected sensor measurements.

### Standalone INA226 and MPU6050 diagnostics

The `sensor-debug` sketch supports the replacement **INA226**, using a
**0.002 Ω (2 mΩ, 002/R002 marking)** shunt at I2C address `0x40`. SDA is GPIO21,
SCL is GPIO22, and serial output is 115200 baud. Build and upload it with:

```bash
pio run -e sensor-debug -t upload
pio device monitor -e sensor-debug
```

For a different shunt or address, add `build_flags` under `[env:sensor-debug]`
in `platformio.ini`, for example:

```ini
build_flags = -DINA226_SHUNT_OHMS=0.01f -DINA226_ADDRESS=0x41
```

Startup checks TI manufacturer ID `0x5449`, INA226 die ID `0x226x`, and
configuration/calibration register readback. Every 500 ms it prints bus voltage,
signed shunt voltage/current, hardware power, and current calculated as
`shunt/R`. It reports I2C failures, missing fresh conversions, lost configuration
or calibration, math overflow, and warnings near shunt saturation or above 36 V.
Send `r` to rescan and initialize both sensors again. The register scaling follows
the [TI INA226 datasheet](https://www.ti.com/lit/ds/symlink/ina226.pdf).

To check accuracy, compare bus voltage with a multimeter between VBUS and GND.
Apply a steady, known load through the shunt and compare reported current with
an independent current measurement. For the 2 mΩ shunt, **1 A produces 2 mV**;
the hardware current and `shunt/R` should agree at a steady load. Both depend on
the configured resistance, so their agreement alone cannot verify its value.
The ADC range is approximately ±40.96 A for this shunt; the board/shunt current
rating can be lower. INA226 bus voltage is the VBUS pin voltage; adding the shunt
drop estimates the supply voltage only when VBUS is connected to IN−.

The main controller and `sensor-debug` use the same INA226 driver and defaults.

### Main controller I2C and IMU protocol

Both sensors use an I2C mutex around complete library operations, with a 10 ms
timeout per bus transaction. The snapshot mutex is never held during I2C.
Missing or failed IMU initialization is retried after one second without stopping
host replies or motor control. Battery acquisition may wait during IMU
initialization (the library includes settling delays).

`CheckedWire` latches failed/short I2C transactions because Adafruit's
`getEvent()` does not propagate its internal read failures. Adafruit MPU6050 is
pinned to 2.2.9, and `scripts/patch_mpu6050.py` applies a small, idempotent patch
before compilation: reset stops on I2C failure or after 100 ms of polling, and
`begin()` propagates that failure. This prevents a disconnected sensor from
holding the shared bus indefinitely. The checked bus requires the virtual
TwoWire API in Arduino ESP32 3.x, supplied by the selected Espressif32 8.x platform.

The `e` reply appends these fields after the existing 15 values:

| Position (1-based) | Value | Unit |
| --- | --- | --- |
| 16–18 | Acceleration X, Y, Z | m/s² |
| 19–21 | Angular velocity X, Y, Z | rad/s |
| 22 | Age since the last successful IMU acquisition | µs; `4294967295` before the first sample |
| 23 | Successful IMU acquisition sequence | Counter |
| 24 | IMU valid and no more than 100 ms old | 0 or 1 |
| 25 | Failed IMU initialization/acquisition attempts | Counter |

On failure the last values, timestamp and sequence remain unchanged, while
validity becomes zero. Calibration text is printed only during startup, before
the host task; runtime IMU sampling writes no asynchronous text to the host UART.
