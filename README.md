# flare-altimeter

Foot-mounted **laser altimeter** for parachute flare timing. It owns only the
last ~13 m AGL: a TF02-Pro rangefinder measures height, a BNO085 IMU corrects
for foot tilt, and the firmware emits escalating audio cues based on estimated
time-to-ground (height ÷ descent rate), not raw height.

This is a DIY **validation device**, not a life-critical instrument. It
cross-checks flare timing after the fact; nobody relies on it in the air.

## Where the documentation is

The project docs are **not in this repo**. They live in the Obsidian vault:

```
G:\My Drive\AI-Base\2-Projects\Personal-flare-altimeter\
```

| File | What it is |
|---|---|
| `Skydiving Flare Altimeter.md` | **The authoritative spec.** Requirements, BOM with decision records, power architecture, as-built GPIO map, firmware pipeline, alert design, roadmap, working conventions. Read this before any design change. |
| `Flare-altimeter.md` | Toolchain / connection troubleshooting: what the board really is, why `platformio.ini` overrides the board profile, COM-port gotchas. |
| `2026-08-21 TF02-Pro received - wiring plan.md` | Laser wiring and pin colours. |
| `2026-08-21 Bring-up session - laser, IMU, alert ladder.md` | Sensor bring-up findings: pin theft by peripherals, TF02 power, piezo drive, alert ladder. |
| `2026-08-22 Modular refactor, unit tests, replay harness.md` | Current code shape, the two bugs the refactor found, test scenarios, replay harness. |

Convention: findings from a work session go back into **every** affected vault
doc, not only a new dated log.

## Hardware

Two boards are supported, chosen by PlatformIO environment; the pin sets live
side by side in [`src/pins.h`](src/pins.h).

| Env | Board | Console | Status |
|---|---|---|---|
| `xiao` | Seeed XIAO ESP32S3 (S3R8: 8 MB flash, 8 MB PSRAM) | native USB CDC, COM7 | current board |
| `esp32s3` | ESP32-S3 N16R8 DevKitC-form clone + PinPulse shield | CH340K, COM4 | bench board |

Next board, ordered: Adafruit ESP32-S3 Feather #5477 — JST-PH battery socket
with charger and a MAX17048 fuel gauge, so the battery needs no soldering.

As wired on the XIAO:

- TF02-Pro laser: green (sensor TX) → D7 / GPIO44, 5 V from the MT3608 boost,
  common ground. White (sensor RX) unused; GPIO43 carries the ROM boot log.
- BNO085 IMU on I2C: D4 SDA / D5 SCL, RST on D3. Answers at 0x4B on this module.
- Buzzer: 3-pin passive, low-level-trigger module, I/O on D0, VCC from 3V3,
  idle level HIGH.
- Planned: 110 dB siren via NMOS on D1. SD logging is deprioritised.

The ground is assumed flat: height is the slant range times cos θ, which is the
height above ground only if the ground under the beam is level.

## Repo layout

```
lib/flare_math    attitude and geometry maths (tilt correction)     (host-testable)
lib/tf02          TF02-Pro 9-byte frame parser                    (host-testable)
lib/alert         alert engine: time-to-ground ladder, hysteresis (host-testable)
lib/flare_config  validation of the persisted runtime settings     (host-testable)
src/hw_*          hardware layer: IMU, lidar, buzzer, console, diag
src/main.cpp      wiring and the 100 Hz loop
src/sim.*         on-board descent simulator (synthetic height ramp into the alert engine)
test/test_*       Unity tests, one directory per lib module
tools/            host-side viewer, mock descent generator, replay/test scripts
```

The split is deliberate: everything that *decides* something lives in `lib/`
and does not include `Arduino.h`, so it compiles and runs on a desktop. Only
register writes and printf remain in `src/`.

## Build and upload

VSCode + PlatformIO, Arduino core 2.0.17 (channel-based LEDC API — check any
online LEDC snippet against this).

```powershell
pio run -e xiao -t upload         # XIAO on COM7, no button press needed
pio run -e esp32s3 -t upload      # bench board on COM4 (CH340K)
pio device monitor -e xiao        # 115200
```

On the bench board, do not remove the `qio_opi` / `default_16MB.csv` / 16 MB
overrides in [`platformio.ini`](platformio.ini): the stock `esp32-s3-devkitc-1`
profile is the N8 no-PSRAM variant and the board faults at boot without them.

A serial port allows one reader. If an upload fails with `Access is denied`,
close the viewer or any `pio device monitor` holding the port.

Live view of attitude and laser range (reconnects by itself after unplugging):

```powershell
.\tools\viewer.ps1                # XIAO on COM7
.\tools\viewer.ps1 --port COM4    # bench board
```

## Tests

Host-side unit tests, one Unity suite per `lib/` module:

```bash
pio test -e native            # intended entry point; needs a host g++
bash tools/run_native_tests.sh  # what actually works on this Windows box (run in WSL)
```

End-to-end replay — mock laser data through the real firmware, with the viewer
asserting the outcome each scenario declares in its CSV header:

```powershell
.\tools\replay.ps1                # all scenarios: perfect, accel, bad, noarm
.\tools\replay.ps1 perfect        # one
.\tools\replay.ps1 -Regenerate    # rebuild tools\mock\ first
```

Needs the Python venv at `%USERPROFILE%\.venvs\flare-imu` — see
[`tools/requirements.txt`](tools/requirements.txt).

## Conventions

- Code comments are bilingual: English first, then a fuller `// 中文说明：`
  block for anything non-obvious (hardware quirks, timing constants, why a
  workaround exists).
- One component per sketch, verify, then integrate.
- Contested design points: AI suggests, user decides.
