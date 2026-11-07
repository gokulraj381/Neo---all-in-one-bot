# Neo - all-in-one desk bot

A small desk companion for the **Waveshare ESP32-S3-LCD-1.28** (non-touch): a round 240x240 GC9A01 screen, a QMI8658 motion sensor and the BOOT button. It shows a face with blinking eyes that follow the way you tilt it, and has four apps: a tilt maze, a step counter, a flip-to-start kitchen timer and a slide-to-measure cm scale. Everything is drawn flicker-free into one full-screen sprite.

| App | What it does |
| --- | --- |
| **Eyes** (home) | Blinking orange eyes that look where you tilt the board. Any BOOT press opens the menu; the face returns after 10 s idle. |
| **MAZE** | Tilt the board to roll a ball through the rings of a round labyrinth to the hole in the middle. 4 levels, best times are saved. |
| **STEPS** | Step counter with a 10 000 step goal ring, cadence ring, a running figure, distance (km) and energy (kcal). Counts in the background and is saved to flash, so it survives power-off. |
| **TIMER** | Kitchen timer. Pick a preset (1, 3, 5, 10, 15, 30 min), then flip the board over and back to start; flip again to pause or resume. The picture rotates to stay upright. |
| **SCALE** | Slide the board along an object and read its length in cm. |

## Hardware

- Waveshare ESP32-S3-LCD-1.28 (non-touch version), 16 MB flash, 2 MB PSRAM
- GC9A01 display (SPI), QMI8658 IMU (I2C, SDA = GPIO6, SCL = GPIO7), BOOT button (GPIO0)
- A 3.7 V Li-ion battery can go on the MX1.25 2-pin connector (optional, the board charges it over USB)

## Controls

One button is all it takes.

| Where | Short press | Hold 1 s |
| --- | --- | --- |
| Menu | Next tile | Open the selected app |
| Any app | The action shown on the screen (for example `CLICK: STOP`) | Back to the menu |

Per app:

- **MAZE**: tilt to roll the ball. After a level, click for the next one.
- **STEPS**: click to reset the count (asks to confirm, click = yes, wait = no). A short notice on first open reminds you that steps are only counted while the board is worn or carried.
- **TIMER**: click changes the preset while idle or after it finished, and stops a running timer. Flip the board over and back to start, pause or resume.
- **SCALE**: click to start, put the board flat, wait for `KEEP STILL` to change to `SLIDE`, slide it along the object and stop. Keep it still at the end, or click to finish. The length is shown in cm. Click again to measure again.

## Remote control

Besides the button, the board can be driven from a phone.

**Bluetooth (BLE UART)** - device name `MenuBot`. Connect with a BLE terminal app (nRF Connect, Serial Bluetooth Terminal in BLE mode, LightBlue) and send text commands:

| Command | Action |
| --- | --- |
| `1` / `maze`, `2` / `steps`, `3` / `timer`, `4` / `scale` | Open that app |
| `0` / `menu` | Back to the menu |
| `click` | Same as a short BOOT press |
| `status`, `help` | Info |

The board replies on the same service (screen changes, the scale result, the step count).

**Wi-Fi** - the board creates its own network `MenuBot` (password `12345678`). Join it on the phone and open `http://192.168.4.1` for buttons: BALL, STEPS, TIMER, SCALE, MENU, CLICK.

Change the names and the password at the top of `src/main.cpp` (`BLE_NAME`, `WIFI_SSID`, `WIFI_PASS`).

## Build and flash

Install [PlatformIO](https://platformio.org/) (VS Code extension or CLI), then:

```bash
pio run -e neo -t upload
# or choose the port yourself
pio run -e neo -t upload --upload-port COM5
```

- The first build downloads the ESP32 platform and the `TFT_eSPI` library, so it takes a while.
- If the board does not show up as a serial port: install the CH34x driver, then hold **BOOT**, tap **RESET**, release **BOOT** to enter download mode.
- Serial monitor: `pio device monitor` (115200 baud).

## Project layout

```
platformio.ini                      build config (one environment: neo)
boards/                             board definition and 16 MB partition table
include/TFT_Setup_ESP32S3_LCD_128.h TFT_eSPI pin and driver setup for the GC9A01
src/main.cpp                        the whole firmware
```

## Notes and limits

- **SCALE is approximate.** The length comes from double-integrating the accelerometer, which drifts. Expect a few cm of error on short slides; use it for a rough measurement, not as a ruler. Slides that are too small, too short or too long (over 300 cm) are rejected as noise or drift.
- **STEPS** is a simple accelerometer step detector. It needs a few steps in a row before it starts counting and ignores shaking that is not walking. Keep the board in a pocket or on the body for the best result.
- If the eyes or the maze tilt the wrong way, flip the sign or swap the axis in `TILT_X_AXIS`, `TILT_X_SIGN`, `TILT_Y_AXIS` and `TILT_Y_SIGN` at the top of `src/main.cpp`.
- Uses `TFT_eSPI` 2.5.43 on `espressif32` 6.5.0 (Arduino core 2.x).
