# Micro SD Card Slot Addition

## Overview

Add a micro SD card slot to the NEMO PCB to enable onboard logging of CAN traffic dumps and sniffed NMEA2000 data. Currently all sniffed data streams over serial only — an SD card allows field use without a laptop.

## Pin Assignment

All four SPI0 pins on the Teensy 4.0 are **free** and available for the SD card.

| SPI Function | Teensy Pin | SD Card Pin |
|-------------|------------|-------------|
| CS (Chip Select) | 10 | CS |
| MOSI | 11 | DI |
| MISO | 12 | DO |
| SCK | 13 | CLK |
| — | 3.3V | VCC |
| — | GND | GND |

### Current Pin Usage (for reference — do not conflict with these)

| Pin | Function |
|-----|----------|
| 0 | CAN2_TX |
| 1 | CAN2_RX |
| 2 | Button Left/Back |
| 3 | Button Down |
| 4 | Button Right/Select |
| 5 | Button Up |
| 14 (A0) | Pot R3 |
| 15 (A1) | Pot R2 |
| 16 (A2) | Pot R1 |
| 18 | SDA (I2C to OLED) |
| 19 | SCL (I2C to OLED) |
| 22 | CAN1_TX |
| 23 | CAN1_RX |

## Hardware Changes (KiCad)

### KiCad Project Location

All PCB design files are in `PCB/` (KiCad 9.0 format):
- `PCB/NEMO.kicad_sch` — schematic
- `PCB/NEMO.kicad_pcb` — board layout
- `PCB/NEMO.kicad_pro` — project file
- `PCB/CAN_CHIP.pretty/` — custom footprints
- `PCB/fp-lib-table` — footprint library references
- `PCB/sym-lib-table` — symbol library references

### Schematic Changes

1. Add a micro SD card socket symbol (KiCad standard library has `Connector_Card:microSD_Card` or use a breakout-compatible footprint)
2. Wire SPI signals:
   - Teensy pin 13 (SCK) → SD CLK
   - Teensy pin 11 (MOSI) → SD DI (MOSI)
   - Teensy pin 12 (MISO) → SD DO (MISO)
   - Teensy pin 10 (CS) → SD CS
3. Power: 3.3V and GND from Teensy
4. Add 0.1µF decoupling capacitor between VCC and GND near the SD socket
5. Optional: 10kΩ pull-up on CS line (some modules include this)

### PCB Layout Considerations

- The SD socket should be placed on the edge of the board for easy card insertion/removal
- Keep SPI traces short and direct — they run at up to 25 MHz
- Route 3.3V from the Teensy's onboard regulator (adequate current for SD writes)
- The current board is a Teensy shield form factor — check that the SD socket fits without interfering with the OLED, buttons, or pots
- Consider whether the 3D-printed case (`Documentation/Case_Design.md`) needs a slot for card access

### Recommended Parts

**Option A — Bare micro SD socket (smallest footprint):**
- Molex 5031821852 or similar push-push micro SD connector
- Requires a KiCad footprint (many available in standard library)

**Option B — Breakout module (easiest for prototyping):**
- Adafruit #254 or generic "Micro SD SPI breakout"
- 6-pin header: VCC, GND, MISO, MOSI, SCK, CS
- Can mount via pin headers on the PCB

## Firmware Changes

### Library Addition

Add SdFat to `SRC/platformio.ini`:
```ini
lib_deps =
	olikraus/U8g2@^2.36.15
	adafruit/Adafruit SSD1306@^2.5.16
	adafruit/Adafruit GFX Library@^1.12.4
	gitlab-display/VEGA_SH1106@^1.0.0
	https://github.com/Soups71/NMEA2000.git
	https://github.com/Soups71/NMEA2000_Teensyx.git
	greiman/SdFat@^2.2.3
```

### Pin Definition

Add to `SRC/include/constants.h`:
```cpp
#define SD_CS_PIN 10
```

### Initialization

In `SRC/src/main.cpp` `setup()`:
```cpp
#include <SdFat.h>

SdFat sd;

void setup() {
    // ... existing setup code ...

    if (!sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(25)))) {
        // SD card not inserted or failed — continue without logging
        Serial.println("SD: not detected, logging disabled");
    } else {
        Serial.println("SD: ready");
    }
}
```

### Logging Integration

The existing CAN2 sniff handler in `main.cpp` already formats data in `candump` format for serial output. The SD logging would hook into the same path:

```cpp
// In the CAN2 message handler, after serial output:
if (sdReady) {
    logFile.print(timestamp);
    logFile.print(" ");
    logFile.println(candumpLine);
    // Flush periodically, not every write (performance)
}
```

### Menu Integration

Add an "SD Card" option to the existing OLED menu system to:
- Show SD card status (inserted/not detected/logging)
- Start/stop logging
- Show file size / free space

## Task Checklist

### PCB (KiCad)
- [ ] Add micro SD socket symbol to schematic
- [ ] Wire SPI0 pins (10, 11, 12, 13) from Teensy to SD socket
- [ ] Add decoupling capacitor
- [ ] Place SD socket on board edge in PCB layout
- [ ] Route traces
- [ ] Run DRC
- [ ] Update gerbers
- [ ] Check 3D model for case clearance

### Firmware
- [ ] Add SdFat library to platformio.ini
- [ ] Add SD_CS_PIN to constants.h
- [ ] Add SD initialization to setup()
- [ ] Add CAN traffic logging function
- [ ] Add periodic flush logic (every N seconds or N messages)
- [ ] Add SD card menu items to OLED UI
- [ ] Add start/stop logging toggle
- [ ] Test with and without SD card inserted (graceful fallback)

### Documentation
- [ ] Update PCB_Design.md with SD card in parts list
- [ ] Update Source_Code_Architecture.md with SD logging module
- [ ] Update Case_Design.md if case needs modification

## Notes

- The Teensy 4.0 does NOT have a built-in SD slot (the 4.1 does). We must use SPI.
- SPI throughput is ~3-4 MB/s, more than enough for CAN bus data rates (1 Mbit/s max).
- SD card should be optional — firmware must work fine without one inserted.
- Use SdFat v2 (not the basic SD.h) for better performance and reliability.
- Consider timestamping log entries using `millis()` since the Teensy 4.0 has no RTC.
