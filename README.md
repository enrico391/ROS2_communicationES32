# ESP32 Serial Controller for ROS 2

ESP32 firmware for a ROS 2 robot hardware interface. It receives motor commands
from a host over serial, controls an ODrive, and returns wheel positions, battery
measurements, and IMU data.

## Project Structure

```text
├── src/
│   ├── main.cpp                       # Main controller firmware
│   └── debug/
│       ├── main.cpp                   # Standalone sensor diagnostics
│       └── main_with_timing_info.cpp  # Saved controller with timing diagnostics
├── include/                           # Sensor drivers and controller helpers
├── lib/                               # Bundled libraries
├── scripts/                           # Build support scripts
├── test/native/                       # Native firmware tests and hardware fakes
└── platformio.ini                     # Board, dependencies, and build environments
```

## Features

- Independent FreeRTOS tasks for host communication, motor control, and sensors.
- ODrive velocity commands and wheel position feedback at 50 Hz.
- Battery voltage/current monitoring with an INA226 and motion sensing with an MPU6050.
- Motor watchdog that sets both velocity targets to zero after 500 ms without a valid command.
- A fixed 25-field serial response, with debug timing fields set to `0`.
- A separate sensor diagnostics build.

## Hardware

- ESP32 DOIT DevKit V1.
- ODrive configured for the connected motors and encoders.
- INA226 with a 2 mΩ shunt, default I2C address `0x40`.
- MPU6050, default I2C address `0x68`.

| Connection | ESP32 pin |
| --- | --- |
| ODrive UART RX → ESP32 TX | GPIO17 |
| ODrive UART TX → ESP32 RX | GPIO16 |
| ODrive reset control | GPIO15 |
| Shared sensor SDA | GPIO21 |
| Shared sensor SCL | GPIO22 |
| Status LED | GPIO2 |

Connect the sensor boards to 3.3 V and GND, and use a common ground with the
ODrive. Axis 0 is the right wheel; axis 1 is the left wheel.

## Getting Started

### Startup

Keep the robot stationary on level ground, with the MPU6050 mounted flat and
its Z axis pointing upward. Startup attempts an IMU offset calibration for
about six seconds, then activates the ODrive. Wait for
`ESP32 SBEM controller ready` before sending commands.

Calibration success or failure is reported during startup. IMU output includes
gravity and uses the sensor's XYZ axes. If the INA226 is unavailable at startup,
firmware continues without battery monitoring; restart after reconnecting it.
The IMU retries initialization automatically after communication failures.

## Usage

The host sends commands over USB serial at 115200 baud. Terminate each command
with carriage return (`\r`); line feed (`\n`) is ignored.

| Command | Purpose | Successful response |
| --- | --- | --- |
| `m <left> <right>\r` | Set wheel velocities in ODrive velocity units | `\r\n` |
| `e\r` | Read the latest sensor snapshot | 25 space-separated values followed by `\r\n` |
| `\r` | Heartbeat | `\r\n` |

For example, `m 1.0 1.0\r` requests the same velocity for both wheels;
`m 0 0\r` requests a stop. A motor acknowledgement means the command was queued.
The latest queued command is applied on the next motor cycle.

Send valid motor commands more frequently than every 500 ms to maintain motion.
The watchdog measures elapsed time from command reception; heartbeat and sensor
requests do not refresh it. Invalid motor commands do not refresh it either.

### Sensor Response

The `e` response preserves these **25 fields**, numbered from 1:

| Position | Value | Unit / format |
| --- | --- | --- |
| 1–2 | Left and right wheel positions | ODrive position units |
| 3 | Battery voltage | V |
| 4 | Battery current; positive when charging | mA |
| 5–10 | Debug read durations, task timings, and wheel sample age | Always `0` |
| 11 | Successful wheel-pair sequence | Counter |
| 12 | Debug overrun count | Always `0` |
| 13 | Failed wheel-pair acquisition attempts | Counter |
| 14 | Debug wheel sample interval | Always `0` |
| 15 | Motor watchdog active | `0` or `1` |
| 16–18 | Acceleration X, Y, Z | m/s² |
| 19–21 | Angular velocity X, Y, Z | rad/s |
| 22 | Debug IMU sample age | Always `0` |
| 23 | Successful IMU acquisition sequence | Counter |
| 24 | IMU valid and no more than 100 ms old | `0` or `1` |
| 25 | Failed IMU initialization/acquisition attempts | Counter |

Timing fields are placeholders; no debug timing measurements are collected.
Use field 24 to check IMU freshness. Failed acquisitions retain the last valid
sensor values, so repeated values alone do not indicate a successful new read.

## Debugging

Build and upload the standalone sensor diagnostics firmware:

```bash
pio run -e sensor-debug -t upload
pio device monitor -e sensor-debug
```

This build prints INA226 and MPU6050 readings every 500 ms. Send `r` to rescan
and initialize the sensors again. It does not run motor control or the host
command protocol.

The controller version with timing instrumentation is preserved separately in
`src/debug/main_with_timing_info.cpp`; it is excluded from the configured builds.
