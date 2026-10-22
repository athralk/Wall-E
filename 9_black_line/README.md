## 9_black_line: line following, centre or left edge

By default Wall-E follows the **centre** of the white line (normal line following).
Set `EDGE_AFTER_MS` in `main/include/config.h` and, that long after the start, it switches
to riding the **left edge** of the line: black under A0 A1, white from A2 on.

```
  A0 A1 A2 A3 A4
   0  1  1  1  0      <- centre mode
   0  0  1  1  1      <- edge mode: edge between A1 and A2
```

### Centre mode (default)
- **Straight wins:** at a cross, or a branch with the line carrying on, it keeps going.
- **Corners:** when the line vanishes, it rolls on 80 ms so the wheels reach the corner,
  then turns toward the side an outer sensor (A0 / A4) last saw the line.
- **T junction:** both sides seen at once, so it turns left.
- **Dead end:** neither side seen, so it keeps turning right, round the end of the line (U-turn).

### Edge mode (`EDGE_AFTER_MS` > 0)
- **Left branch:** white under A0 + A1 can only be a line from the left. It drives onto the
  junction and turns left.
- **Line gone:** it turns right. Edge back within ~90 degrees means a right turn; otherwise
  it keeps going round (U-turn).
- That's the left hand rule: left, else straight, else right, else U-turn.

### Turns
No stopping: the bot turns straight out of following and goes straight back into it.
1. A 60 ms kick at 50, then 48.
2. It eases to 45 once a sensor on the turn side sees the new line.
3. It hands over to the PID half a sensor early, and the momentum and PID finish the turn.

If nothing shows within 2.5 s it tries the other way once, then stops and waits to be put
back on a line.

### States (bar graph)
| State | LEDs | What it does |
|---|---|---|
| WAIT | 1000 | Motors off until the bar has sat on a line for 0.5 s |
| FOLLOW | 0001 | PID on the smooth line (or edge) position |
| APPROACH | 0010 | Edge mode: drive onto the junction, then turn left |
| TURN | 0100 | Pivot until the line is back under the middle |
| DONE | 1111 | End zone (all white 1.5 s). Put it back on a line to carry on |

### Files
| File | What |
|---|---|
| `main/include/config.h` | Every tunable |
| `main/black_line.c` | Start up, main loop, state switching, OLED |
| `main/follow.c` | FOLLOW and APPROACH |
| `main/turns.c` | TURN and WAIT / DONE |
| `main/edge.c` | Smooth line centre / edge position from the sensor bar |
| the rest | Sensor bar, motors, PID, Wi-Fi tuner: same as `7_maze_solving` |

### Tuning
1. `BASE_DUTY` / `MIN_DUTY` / `MAX_DUTY` and Kp / Kd (web page) until it follows smoothly.
2. `CREEP_MS` so turns start with the wheels on the corner.
3. `TURN_DUTY`, `TURN_SLOW_DUTY`, `TURN_STOP_EARLY`, `BRAKE_MS` until turns land without overshoot.

Not in here yet: colour blind section, pink drop zone.
