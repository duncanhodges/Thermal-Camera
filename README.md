# ESP32 MLX90640 Thermal Camera

A real-time thermal camera built around an ESP32, a Melexis MLX90640 32×24 IR
array, and a 240×320 SPI TFT. The sensor image is rendered as flat colour
blocks (or smoothly interpolated), with a manually-settable temperature scale
driven by a rotary encoder, a side colourbar legend, a centre crosshair with
live temperature readout, and markers tracking the hottest and coldest pixels.

## Features

- **32×24 → 320×240 render** as flat colour blocks (nearest-neighbour) or
  bilinear-interpolated, selectable at compile time.
- **Manual temperature scale** set with a rotary encoder — no auto-ranging, so
  the colours mean the same thing frame to frame.
- **Side colourbar legend** with live min/max labels; the active end is
  highlighted in yellow.
- **Centre crosshair** with a large centre-temperature readout (averaged over
  the four central pixels).
- **Hot / cold markers** that track the max and min pixels each frame.
- **Dead-pixel de-spike** that patches defective sensor pixels, including the
  NaN values the MLX90640 emits for some broken cells.
- **Dual-core pipeline**: the sensor is read on core 0 while the previous frame
  is rendered on core 1, and the image is streamed to the panel over SPI DMA.
  Throughput sits at the sensor's ~8 fps ceiling (16 Hz subpage rate).

## Hardware

- **Board:** NodeMCU-ESP32 (classic dual-core ESP32, e.g. WROOM-32).
  The dual core and robust fast I²C matter here — a single-core ESP32-C3 works
  but runs slower and is fussier on I²C.
- **Thermal sensor:** MLX90640 breakout (default I²C address `0x33`).
  Either FoV variant (55° or 110°) is fine.
- **Display:** 240×320 SPI TFT, **ILI9341** driver. ST7789 panels also work
  with a different driver `#define` (see below).
- **Input:** KY-040 (or any quadrature) rotary encoder with push switch.
- Everything runs at **3.3 V** — do not feed the logic 5 V.

## Wiring

All pins are the defaults in the sketch (I²C + encoder) and in TFT_eSPI's
`User_Setup.h` (display). Change them in one place if you re-pin.

### Display — ILI9341 (set in `User_Setup.h`)

| TFT pin      | ESP32 GPIO |
|--------------|------------|
| SCK / SCLK   | 18         |
| SDI / MOSI   | 23         |
| SDO / MISO   | 19 (optional) |
| CS           | 15         |
| DC / RS      | 2          |
| RST          | 4          |
| VCC          | 3V3        |
| LED / BLK    | 3V3        |
| GND          | GND        |

### MLX90640 — I²C (set in the sketch)

| Sensor pin | ESP32 GPIO |
|------------|------------|
| SDA        | 21         |
| SCL        | 22         |
| VIN        | 3V3        |
| GND        | GND        |

### Rotary encoder — KY-040 (set in the sketch)

| Encoder pin | ESP32 GPIO |
|-------------|------------|
| CLK         | 32         |
| DT          | 33         |
| SW          | 25         |
| +           | 3V3        |
| GND         | GND        |

The encoder uses the ESP32's internal pull-ups, so a bare encoder works without
the breakout's own resistors. The chosen GPIOs avoid the boot-strapping pins.

## Controls

| Action                    | Effect                                              |
|---------------------------|-----------------------------------------------------|
| Turn encoder              | Adjust the active scale end (min or max), 0.5 °C/detent |
| Short press               | Swap which end the knob adjusts (active end shown in yellow with `>`) |
| Long press (≥0.6 s)       | Snap both scale ends to the current scene, then fine-tune |

## Build & flash

### Libraries (Arduino Library Manager)

- **Adafruit MLX90640** (pulls in **Adafruit BusIO**)
- **TFT_eSPI** (Bodmer)

### Configure TFT_eSPI

TFT_eSPI reads its driver and pins from its own `User_Setup.h` at compile time —
**not** from the sketch. Edit `Documents/Arduino/libraries/TFT_eSPI/User_Setup.h`
(comment out the rest) so only this is active:

```cpp
#define ILI9341_DRIVER
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_MISO 19
#define TFT_CS   15
#define TFT_DC    2
#define TFT_RST   4
#define LOAD_GLCD
#define LOAD_FONT2
#define SPI_FREQUENCY 40000000
```

### Board settings (Arduino IDE)

- Board: **ESP32 Dev Module**
- Flash Size: **4MB**
- Partition Scheme: **Default**

Compile and upload `thermal_cam_esp32.ino`. Point it at something warm; the
centre readout tracks and the red marker snaps to the hottest spot.

## Configuration

Key `#define`s at the top of the sketch:

| Define         | Default | Purpose                                                   |
|----------------|---------|-----------------------------------------------------------|
| `SMOOTH`       | `0`     | `0` = flat colour blocks, `1` = bilinear interpolation    |
| `DESPIKE`      | `1`     | Patch defective sensor pixels (incl. NaN)                 |
| `SPIKE_THRESH` | `6.0`   | °C a finite pixel must exceed all neighbours by to be patched |
| `ENC_STEP`     | `0.5`   | °C per encoder detent                                     |
| `MIRROR_X`     | `false` | Flip image horizontally to match sensor mounting          |
| `FLIP_Y`       | `false` | Flip image vertically to match sensor mounting            |
| `tMin` / `tMax`| `20` / `35` | Startup temperature scale (°C)                        |

## Notes & gotchas

- **Colour byte order:** the sketch calls `tft.setSwapBytes(true)`. Without it,
  the palette renders with swapped bytes — blues come out green and the gradient
  breaks into bands. If your panel shows the opposite, remove that line.
- **ST7789 panels:** replace `ILI9341_DRIVER` with `ST7789_DRIVER`, add
  `#define TFT_WIDTH 240` and `#define TFT_HEIGHT 320`, and if colours look
  wrong add `#define TFT_INVERSION_ON` and/or `#define TFT_RGB_ORDER TFT_BGR`.
- **Encoder direction:** if turning clockwise lowers the value, swap the CLK/DT
  wires or flip the `++`/`--` in `encISR()`.
- **Frame rate:** ~8 fps is the MLX90640's realistic ceiling at a 16 Hz subpage
  rate, not a display limitation. `MLX90640_32_HZ` doubles it but is noisier.
- **Image mirrored/upside down:** set `MIRROR_X` / `FLIP_Y` to match how the
  sensor is physically mounted.

## Licence

MIT — do as you like.
