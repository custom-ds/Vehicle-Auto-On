# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Firmware for a vehicle power switch: an ATtiny404 drives a relay that powers accessories. The git repo root is two levels up (`../../`), which also holds `Case/` (FreeCAD/STL enclosure) and `PCB/` (KiCad project, Gerbers, versioned board revisions). This directory is the PlatformIO firmware project.

## Hardware constraints

- **The MCU runs at 3.3 V.** The schematic's regulator symbol reads `MCP1703Ax-250xxTT` (2.5 V), but the fitted part and the `+3V3` rail are 3.3 V.
- At 3.3 V the ATtiny404 is rated for at most 10 MHz (20 MHz needs ≥4.5 V). `platformio.ini` sets `board_build.f_cpu = 5000000L` (20 MHz osc ÷4). Don't remove it: the board default of 16 MHz leaves the prescaler off and the chip actually runs at 20 MHz, which caused intermittent lockups.
- Flash is 4 KB and nearly full. Avoid `float` (the soft-float library alone costs ~500 bytes) and watch the size after changes.

## Build / Upload

PlatformIO, single env `ATtiny404` (`atmelmegaavr` platform, Arduino framework). `pio` is not on PATH; use the full path:

```
~/.platformio/penv/Scripts/pio.exe run                 # build
~/.platformio/penv/Scripts/pio.exe run -t upload       # flash
```

Upload goes over UPDI via an Arduino running jtag2updi (Arduino pin 6 → ATtiny404 reset/UPDI), on the `upload_port` in `platformio.ini`. There are no tests or linter (`test/` is the PlatformIO placeholder).

`upload` does **not** write fuses for this board; `pio run -t fuses` does (oscillator, BOD level from `board_hardware.bod`, default 2.6 V, UPDI pin mode). The BOD level must stay below the 3.3 V supply.

## Code layout

- `src/main.cpp` — the entire firmware; this is the only file built.
- `poc/` — earlier standalone experiments (`proofofconcept.cpp`, `low-power-test.cpp`) kept for reference. Not compiled; each has its own `setup()`/`loop()`, so don't move them into `src/`.

## Firmware architecture

- **Tick-based loop.** `loop()` runs once per wake, then `goToSleep()`. The RTC PIT (internal 32 kHz ULP osc, `CYC8192`) fires every ~250 ms. In low-power mode the MCU enters `SLEEP_MODE_PWR_DOWN` and the PIT ISR wakes it; otherwise it just `delay(250)`. All timing constants (`SWITCH_POSITION_DELAY`, `CHECK_AUTO_INTERVAL`, `AUTO_SHUTDOWN_DELAY`, LED blink counts) are counted in these loop cycles, not milliseconds. State lives in `static` locals inside `loop()`.
- **3-position switch** (`SWITCH_AUTO` / `SWITCH_ON`, active-low with pullups) → `OFF` / `ON` / `AUTO`, debounced by `SWITCH_POSITION_DELAY` cycles. `ON` keeps the MCU awake; `OFF` and idle `AUTO` allow power-down sleep (measured ~100 µA).
- **AUTO mode logic:** every `CHECK_AUTO_INTERVAL` cycles, `isCharging()` compares battery voltage to a reference (analog front end gated by `ANALOG_ENABLE`). Charging starts/extends `autoKeepAliveCount`; on first turn-on it is set slightly above `AUTO_SHUTDOWN_DELAY` so the relay waits for the alternator to stabilize. If not charging but already on, motion from the LIS2HH12 accelerometer (I2C, addr `0x1D`, raw register access via `Wire`) extends the keep-alive — this handles smart alternators that let voltage drop while driving. Motion is only used to keep power on, never to turn it on.
- **Voltage conversion** in `isCharging()` uses empirically calibrated constants, including a different offset when the relay is on (drop across the resettable fuse). Treat these as hardware calibration, not arbitrary numbers.
- **LED** modes: solid = manual on, slow blink = charging, fast blink = motion keep-alive.
- **Debug output** is a bit-banged TX-only serial on `PIN_PB3` (`customSerial*`), guarded by `#define DEBUG_SERIAL`. It runs at 1200 baud and is very timing-sensitive to the clock. Serial printing is slow and adds awake time, so it affects power measurements.
- The watchdog (`wdt_enable` / `wdt_reset`) is currently commented out for power testing.
