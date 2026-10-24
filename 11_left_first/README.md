# MAZE SOLVING

Line follower for maze tracks (white line on black). It does everything [5_line_following](../5_line_following) does (PID line following, PID tuning from the web page over WiFi, Kp/Ki/Kd on the OLED) and adds **90 degree corners**, **45 degree sharp turns**, nodes, dead end U-turns, the **colour blind** section, the **pink drop zone** and stopping at the end zone.

## How it works

The bot runs a small state machine every 10 ms. The current state is shown on the bar graph LEDs.

| State | LEDs | What the bot does | Leaves when |
|---|---|---|---|
| FOLLOW | 0001 | Normal PID line following | A corner, sharp turn or lost line is detected |
| APPROACH | 0010 | 90° corner seen (`11100` / `00111`): drives straight so the wheels sit over the corner | `APPROACH_MS` passes. If the line is still ahead it was only a curve, back to FOLLOW |
| TURN | 0100 | Every turn in place (90° pivot, 45° / acute left, U-turn): spins until the centre sensor has left the old line and lands on a line, then a short reverse brake. Speed `TURN_DUTY` | Centre sensor on a line, or timeout → SEARCH |
| SEARCH | 1000 | Line lost after it slid to one side: spins toward that side, sweeping wider each time | Any middle sensor finds the line |
| DROP | 1001 | Pink drop zone under the middle sensors: stops, backs off the object | Then a U-turn (TURN) |
| DONE | 1111 | End zone (whole bar still white after driving onto it): stopped | Put it back on a line for `RESUME_MS` and it carries on |

## Nodes

APPROACH collects every exit at a junction: left / right from the side sensors while driving over it, straight from the middle sensors once the wheels are on it. A junction with 2+ exits is a **node** and is counted (`N:` on the OLED, `node N` in the serial log).

Exit choice: **straight first**, then `TURN_PREFERENCE`, then the other turn. Every node is remembered (`node_memory.c`): if the exit taken leads to a dead end, the bot U-turns, comes back to that node and takes the next exit it hasn't tried. If every exit of a node was a dead end, it heads back to the node before and does the same there.

`NODE_PLAN` overrides the choice per node, one character per node in order: `A` auto, `S` straight, `L` left, `R` right. Do one practice run, read the `node N exits ...` lines in the serial monitor, and force the nodes where straight-first goes the wrong way (e.g. the branch to the pink drop zone).

Sensor patterns are read left to right, `1` = sensor on the white line.

## Colour blind section

A white block with a black line down the middle. When both outer sensors see the line and the centre doesn't (`11011` / `10001`), the polarity is wrong, so `colour_blind.c` flips it and black counts as the line. FOLLOW then steers toward the black line as usual (just averaging the white would steer *away* from it when the bot drifts). The same check flips it back once the white line returns, and an all-white bar while inverted (the block's end edge) also flips back. For `INVERT_HOLD_MS` after a flip the bot only PIDs straight, so the block's edges aren't taken as a junction or dead end. The OLED shows `INV` and the serial log prints `colour blind: ON / OFF`.

## Pink drop zone

The LSA only sees brightness, so pink is a band of readings between black and white. **Calibrate before running:** set `LOG_READINGS 1` in `config.h`, hold the bar over the pink square, and set `PINK_MIN` / `PINK_MAX` around the middle three values printed (the defaults are placeholders). The bot pushes the object along the line, stops when the middle sensors read pink for `PINK_CONFIRM` loops, backs off, U-turns and carries on. This only happens once per run.

## Files

Read them in this order:

| File | Purpose |
|---|---|
| `main/include/config.h` | **Every tunable number.** The only file you should edit while tuning |
| `main/include/maze.h` | List of states, the shared `maze_ctx_t` struct, which file handles which state |
| `main/maze_solving.c` | `app_main` and the loop: read sensors → colour blind check → run the state's handler |
| `main/state_follow.c` | FOLLOW: pink, junction, sharp turn and line lost checks, then PID |
| `main/state_junction.c` | APPROACH: end zone, node counting, picking the exit |
| `main/node_memory.c` | Remembers nodes and tried exits, picks the next exit after a dead end |
| `main/junction.c` | Pattern checks (`detect_branches`, `detect_sharp`), `choose_exit`, `NODE_PLAN` |
| `main/state_turns.c` | TURN (pivot / sharp / U-turn), SEARCH, DONE |
| `main/colour_blind.c` | Flips the sensor polarity for the black-on-white section |
| `main/drop_zone.c` | Pink detection and the DROP state |
| `main/display.c` | OLED: Kp / Ki / Kd, IP, node count, `INV` |
| `main/line_sensor.c` | Reads the LSA, decides which sensors see the line, computes line position (-5..+5) |
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
4. `TURN_DUTY` for every turn in place, `KICK_DUTY` / `KICK_MS` if the bot sits still after a turn.
5. `PINK_MIN` / `PINK_MAX` (see above), then `DROP_REVERSE_MS` so the U-turn clears the object.

Every state change is printed on the serial monitor with the sensor pattern and how long the previous state lasted.
