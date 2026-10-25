### ESP32 serial controller for ROS 2 robot hardware

The active firmware in `src/main.cpp` communicates with the host using serial
commands at 115200 baud, controls the ODrive over `Serial2`, and reads battery
voltage and current from an INA219 sensor.


The firmware creates three FreeRTOS tasks after serial and INA219 initialization:

| Task | Schedule | Responsibility |
| --- | --- | --- |
| Host (priority 2) | Poll serial once per RTOS tick | Reply from a snapshot; enqueue motor commands and acknowledge immediately |
| ODrive (priority 3) | Every 20 ms | Activate ODrive, write motor velocities, read both positions, publish a completed wheel pair |
| Battery (priority 1) | Every 200 ms | Read INA219 and publish voltage/current |

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

The INA219 voltage retains a 50-sample moving average (now a 10-second window).
At startup it averages only collected samples. Current remains instantaneous mA,
as in the existing protocol.

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
