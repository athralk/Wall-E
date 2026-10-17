# MAZE SOLVING

Line follower for maze tracks (white line on black). It does everything [5_line_following](../5_line_following) does (PID line following, PID tuning from the web page over WiFi, Kp/Ki/Kd on the OLED) and adds handling for **90 degree corners** and **45 degree sharp turns**.

## How it works

The bot runs a small state machine every 10 ms. The current state is shown on the bar graph LEDs.

| State | LEDs | What the bot does | Leaves when |
|---|---|---|---|
| FOLLOW | 0001 | Normal PID line following | A corner, sharp turn or lost line is detected |
| APPROACH | 0010 | 90° corner seen (`11100` / `00111`): drives straight so the wheels sit over the corner | `APPROACH_MS` passes. If the line is still ahead it was only a curve, back to FOLLOW |
| PIVOT | 0100 | Spins in place toward the corner until the centre sensor finds the new line, then a short reverse brake | Centre sensor on the line, or timeout → SEARCH |
| SHARP | 0110 | 45° turn: line only under the outermost sensors (`10000` / `11000`). Outer wheel forward, inner wheel backward | Centre sensor on the line, or timeout → SEARCH |
| SEARCH | 1000 | Line lost after it slid to one side: spins toward that side, sweeping wider each time | Any middle sensor finds the line |
| UTURN | 1100 | Dead end (line ended while centred): spins 180° | Centre sensor back on the line after `UTURN_MIN_MS` |
| DONE | 1111 | End zone (whole bar still white after driving onto it): stopped | - |

## Nodes

APPROACH collects every exit at a junction: left / right from the side sensors while driving over it, straight from the middle sensors once the wheels are on it. A junction with 2+ exits is a **node** and is counted (`N:` on the OLED, `node N` in the serial log).

Exit choice: **straight first**, then `TURN_PREFERENCE`, then the other turn. Right after a dead end U-turn, straight leads back the way we came, so a turn is taken instead.

`NODE_PLAN` overrides the choice per node, one character per node in order: `A` auto, `S` straight, `L` left, `R` right. Do one practice run, read the `node N exits ...` lines in the serial monitor, and force the nodes where straight-first goes the wrong way (e.g. the branch to the pink drop zone).

Sensor patterns are read left to right, `1` = sensor on the white line.

## Files

| File | Purpose |
|---|---|
| `main/maze_solving.c` | `app_main`, the state machine and all turn tunables (`#define`s at the top) |
| `main/line_sensor.c` | Reads the LSA, decides which sensors see the white line, computes line position (-5..+5) |
| `main/motion.c` | `drive()` with signed duty (negative = backward), `pivot()`, `stop()` |
| `main/pid.c` | PID correction using the gains from the web page, `pid_reset()` after turns |
| `main/tuning_http_server.c`, `main/wifi_handler.c`, `frontend/` | PID tuning web page, same as 5_line_following |

## Usage

1. `idf.py menuconfig` → *Wall-E Configuration → WiFi Config*: set the SSID and password of the network the bot should join.
2. `idf.py build flash monitor`
3. Open the IP shown on the OLED (or `http://walle-tuning.local`) from a device on the same network and tune Kp, Ki, Kd.

## Tuning order

1. PID on a straight line and gentle curves (web page).
2. `APPROACH_MS` so the bot pivots with its wheels on the corner (too small: cuts the corner, too large: overshoots past it).
3. `TURN_DUTY` and `BRAKE_MS` so the pivot stops on the new line without overshoot.
4. `SHARP_OUTER_DUTY` / `SHARP_INNER_DUTY` for 45° turns.

Every state change is printed on the serial monitor with the sensor pattern and how long the previous state lasted.
