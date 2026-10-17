# Wall-E Line Following & Maze Solving

Line following and maze solving firmware for the SRA-VJTI **Wall-E** bot (ESP32 + SRA development board), trimmed down from [SRA-VJTI/Wall-E](https://github.com/SRA-VJTI/Wall-E) to the line following parts only.

## Table of Contents
- [Hardware](#hardware)
- [Installation](#installation)
- [Examples](#examples)
- [Running the Examples](#running-the-examples)
- [PID Tuning over WiFi](#pid-tuning-over-wifi)
- [License](#license)

## Hardware
- **ESP32** on the SRA development board
- **Line Sensor Array (LSA)**: 5 analog sensors, white line on black
- **Dual DC motors** driven by PWM through the board's motor driver
- **OLED** (optional): shows Kp / Ki / Kd, the bot's IP and the maze node count
- Two wheels plus a front ball caster (not self balancing)

## Installation
- Follow [these instructions](Installations.md), or the [official ESP-IDF guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/#installation-step-by-step).
- Clone with the board library submodule:
```bash
git clone --recursive <this repo>
# or, if already cloned
git submodule update --init components/sra-board-component
```

## Examples

| Example | Description |
|---|---|
| [2_LSA](2_LSA/) | Reads the line sensor array. Use it to check sensor order and white / black thresholds. |
| [5_line_following](5_line_following/) | PID line follower with PID tuning from a web page over WiFi. |
| [7_maze_solving](7_maze_solving/) | Everything in 5_line_following plus 90° corners, 45° sharp turns, dead end U-turns, junction (node) handling with straight-first priority, a node counter and stopping at the end zone. |

All examples use the SRA board library in [components/sra-board-component](components/).

## Running the Examples
```bash
cd 7_maze_solving        # or any other example
idf.py menuconfig        # set WiFi SSID / password under "Wall-E Configuration"
idf.py build flash monitor
```

## PID Tuning over WiFi
5_line_following and 7_maze_solving join the WiFi network set in `idf.py menuconfig` → *Wall-E Configuration → WiFi Config*. Once connected, the IP is shown on the OLED and in the serial monitor. Open it (or `http://walle-tuning.local`) from a device on the same network and change Kp, Ki and Kd while the bot runs.

## License
Distributed under the [MIT License](LICENSE). Based on [SRA-VJTI/Wall-E](https://github.com/SRA-VJTI/Wall-E).
