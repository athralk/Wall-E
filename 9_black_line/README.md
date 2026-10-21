## 9_black_line: left edge following

Wall-E rides the **left edge** of the white line, black on its left and white on its right,
instead of the middle of the line.

```
  A0 A1 A2 A3 A4
   0  0  1  1  1      <- normal: edge between A1 and A2
```

### Why
- **Left branches can't be confused with noise.** A0 and A1 sit on black the whole time,
  so white under both of them can only be a line coming in from the left.
- **No U-turn guessing.** Losing the white always means one thing: turn right until the
  edge is back. That covers a right corner, a dead end (the edge wraps round the end of
  the line, which is the U-turn) and drifting off to the left.
- **Right branches are passed by**, the left edge carries on straight past them.

Together that is the left hand rule: **left, else straight, else right, else U-turn**,
and coming back from a dead end the branches it skipped are now on its left.

### States (bar graph)
| State | LEDs | What it does |
|---|---|---|
| WAIT | 1000 | Motors off until the bar has sat on a line for 0.5 s (start, failed turn) |
| FOLLOW | 0001 | PID on the smooth edge position. White under A0 + A1 for 2 loops -> APPROACH (only once it has found the edge since the last turn). All white -> steers left onto the edge, 1.5 s of it = END zone. `00111` -> `00000` (2 loops) -> pivot right in place |
| APPROACH | 0010 | Drive onto the junction, then turn left. Still all white -> back to FOLLOW, which steers left onto the edge |
| TURN | 0100 | Pivot until the edge is back under A1/A2: kick 50, turn 45, ease 40, stop one sensor early, brake. Right turn: nothing within ~90 degrees = dead end, keeps going round (U-turn) |
| DONE | 1111 | End zone. Put it back on a line to carry on |

### Files
| File | What |
|---|---|
| `main/include/config.h` | Every tunable |
| `main/black_line.c` | Start up, main loop, state switching, OLED |
| `main/follow.c` | FOLLOW and APPROACH |
| `main/turns.c` | TURN and WAIT / DONE |
| `main/edge.c` | What the sensor bar says about the edge |
| the rest | Sensor bar, motors, PID, Wi-Fi tuner: same as `7_maze_solving` |

### Placing it
Put the bot down with the **left edge of the white line under the middle sensor**
(A0 A1 on black, A2 onwards on white). It starts by itself 0.5 s after it sees a line.

### Tuning order
1. `BASE_DUTY` / `MIN_DUTY` / `MAX_DUTY` and Kp / Kd (web page) until it rides the edge smoothly.
2. `APPROACH_MS` so left turns pivot with the wheels on the junction.
3. `TURN_DUTY`, `TURN_SLOW_DUTY`, `BRAKE_MS` until turns stop on the edge without overshoot.
4. `TURN_STOP_EARLY` (0 or 1) and `RIGHT_CHECK_PCT` once the turns land well.

Not in here yet: colour blind section, pink drop zone.
