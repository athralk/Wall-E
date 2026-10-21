## IR sensor test

Checks the single side IR sensor (MH-B / FC-51 style module) reads white and black
before it gets used for turn detection in `7_maze_solving`.

### Wiring
| Module | SRA board |
| ------ | --------- |
| VCC | 3V3 (header pin 2, next to 36) |
| GND | GND (header pin 1) |
| OUT | GPIO 27 (same header, pin 12) |

GPIO 27 is free on the 2026 board: not used by the motors, LSA, OLED or bar graph, and
not a boot strapping pin.

### Run
```
idf.py build flash monitor
```

### What to check
1. Hold the sensor over **white**: the log should say `WHITE` and the small LED on the
   module should be on. Over **black**: `black`, LED off.
2. If it's the other way round, set `IR_WHITE_LEVEL` to 1 in `main/main.c`.
3. Mount it at its real height on the bot and turn the blue pot until it switches cleanly:
   white always reads WHITE, black always reads black, with no flicker on either.
4. Roll the bot by hand across a few white lines: `white patches so far` should go up by
   exactly one per line. If it jumps by more, the pot is right on the edge, turn it a bit.

If the log never changes at all, check OUT is really on GPIO 27 and the module's power LED is on.
