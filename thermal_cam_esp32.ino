/*
 * MLX90640 Thermal Camera -> 240x320 SPI TFT (ILI9341 / ST7789)
 * Board: NodeMCU-ESP32 (classic dual-core ESP32)
 *
 *  - 32x24 sensor shown as flat COLOUR BLOCKS (nearest-neighbour) by default,
 *    or smoothly interpolated - see the SMOOTH toggle below
 *  - MANUAL temperature scale set with a rotary encoder, shown as a side legend
 *  - Centre crosshair + large centre-temperature readout
 *  - Live HOT (red) and COLD (cyan) markers tracking the max/min pixels
 *
 * ROTARY ENCODER (KY-040 or similar)
 *  - Turn            : adjust the active scale end (min or max) in 0.5 C steps
 *  - Short press      : swap which end the knob adjusts (shown in yellow + ">")
 *  - Long press (0.6s): snap both ends to the current scene, then fine-tune
 *
 * PERFORMANCE DESIGN
 *  - Core 0 continuously reads the sensor (the ~125 ms/frame bottleneck)
 *  - Core 1 (loop) renders the *previous* frame while core 0 fetches the next
 *  - Image is streamed to the panel by SPI DMA using double line-buffers.
 *
 * LIBRARIES (Arduino Library Manager):
 *   - Adafruit MLX90640     (+ Adafruit BusIO dependency)
 *   - TFT_eSPI              (configure User_Setup.h for your ILI9341 pins)
 */

#include <Wire.h>
#include <string.h>
#include <math.h>
#include <Adafruit_MLX90640.h>
#include <TFT_eSPI.h>

// ---------------- Display mode ----------------
#define SMOOTH   1             // 0 = flat colour blocks (nearest-neighbour)
                               // 1 = smooth bilinear interpolation

// ---------------- Dead-pixel de-spike ----------------
#define DESPIKE      1         // 1 = patch stuck/defective sensor pixels
#define SPIKE_THRESH 6.0f      // a pixel beyond EVERY neighbour by this many
                               // degrees C is treated as dead and replaced

// ---------------- Pins / bus ----------------
#define I2C_SDA  21
#define I2C_SCL  22
#define I2C_HZ   800000UL      // 800 kHz is very reliable; 1000000 also fine on ESP32

// Rotary encoder
#define ENC_CLK  32
#define ENC_DT   33
#define ENC_SW   25
#define ENC_STEP 0.5f          // degrees C per detent
#define ENC_LONG_US 600000UL   // long-press threshold (microseconds)

// ---------------- Geometry ----------------
#define ROT      1             // 1 or 3 = landscape 320x240
#define TFT_W    320
#define TFT_H    240
#define IMG_W    256           // thermal image occupies x 0..255 (clean 8x10 blocks)
#define IMG_H    240
#define MLX_W    32
#define MLX_H    24
#define NPIX     (MLX_W * MLX_H)
#define BW       (IMG_W / MLX_W)   // block width  = 8 px
#define BH       (IMG_H / MLX_H)   // block height = 10 px

// ---------------- Legend (right-hand strip, x 256..319) ----------------
#define BAR_X0   268
#define BAR_X1   286           // bar occupies x 268..285
#define BAR_Y0   22
#define BAR_Y1   218           // bar occupies y 22..217 (hot at top)

// ---------------- Orientation (flip to match how you mount the sensor) ----
#define MIRROR_X true
#define FLIP_Y   false

TFT_eSPI          tft = TFT_eSPI();
Adafruit_MLX90640 mlx;

// Block maps: screen pixel -> sensor cell (mirror/flip baked in)
uint8_t blkX[IMG_W], blkY[IMG_H];

// Bilinear sample maps (only used when SMOOTH == 1)
uint8_t cX[IMG_W], cX1[IMG_W];  float cXf[IMG_W];
uint8_t rY[IMG_H], rY1[IMG_H];  float rYf[IMG_H];

uint16_t palette[256];
uint16_t lineBuf[2][TFT_W];     // double buffer for DMA streaming (full width)
uint16_t legRow[TFT_H];         // precomputed colourbar colour per screen row
uint16_t legBG;

// MANUAL scale (set by the encoder; sensible startup defaults)
float tMin = 20.0f, tMax = 35.0f;

// ---- Encoder state (ISR <-> loop) ----
volatile int32_t  encSteps  = 0;
volatile uint32_t encLastUs = 0;
volatile uint32_t btnDownUs = 0;
volatile bool     btnShort  = false, btnLong = false;
bool editMax = true;            // true = knob adjusts MAX, false = MIN

// ---- Cross-core frame hand-off ----
SemaphoreHandle_t frameMux;
volatile bool     haveNew   = false;
float             frameShared[NPIX];
float             frameSensor[NPIX];

// ---------------------------------------------------------------------------
void IRAM_ATTR encISR() {
  uint32_t now = micros();
  if (now - encLastUs < 1500) return;      // debounce
  encLastUs = now;
  if (digitalRead(ENC_DT)) encSteps++; else encSteps--;
}

void IRAM_ATTR btnISR() {
  uint32_t now = micros();
  if (digitalRead(ENC_SW) == LOW) {        // pressed (INPUT_PULLUP, active low)
    btnDownUs = now;
  } else {                                 // released
    uint32_t held = now - btnDownUs;
    if (held > 8000) { if (held > ENC_LONG_US) btnLong = true; else btnShort = true; }
  }
}

// ---------------------------------------------------------------------------
void buildPalette() {
  // ironbow-ish: black -> blue -> purple -> red -> orange -> pale yellow
  const int     N      = 6;
  const float   pos[N] = {0.00f, 0.20f, 0.40f, 0.60f, 0.80f, 1.00f};
  const uint8_t R[N]   = {0,   0,   120, 220, 255, 255};
  const uint8_t G[N]   = {0,   0,   0,   40,  160, 255};
  const uint8_t B[N]   = {0,   140, 160, 0,   0,   190};
  for (int i = 0; i < 256; i++) {
    float t = i / 255.0f;
    int s = 0;
    while (s < N - 2 && t > pos[s + 1]) s++;
    float f = (t - pos[s]) / (pos[s + 1] - pos[s]);
    uint8_t r = R[s] + f * (R[s + 1] - R[s]);
    uint8_t g = G[s] + f * (G[s + 1] - G[s]);
    uint8_t b = B[s] + f * (B[s + 1] - B[s]);
    palette[i] = tft.color565(r, g, b);
  }
}

void buildLegend() {
  legBG = tft.color565(8, 8, 8);
  for (int y = 0; y < TFT_H; y++) {
    if (y >= BAR_Y0 && y < BAR_Y1) {
      float t = (float)(BAR_Y1 - 1 - y) / (BAR_Y1 - 1 - BAR_Y0);   // 1 at top (hot)
      int idx = (int)(t * 255.0f);
      if (idx < 0) idx = 0; else if (idx > 255) idx = 255;
      legRow[y] = palette[idx];
    } else {
      legRow[y] = legBG;
    }
  }
}

void buildMaps() {
  for (int x = 0; x < IMG_W; x++) {
    int c = x * MLX_W / IMG_W;                 // 0..31
    blkX[x] = MIRROR_X ? (MLX_W - 1 - c) : c;
    float tx = x * (float)(MLX_W - 1) / (IMG_W - 1);
    if (MIRROR_X) tx = (MLX_W - 1) - tx;
    int xi = (int)tx;
    if (xi < 0) xi = 0; if (xi > MLX_W - 1) xi = MLX_W - 1;
    int xi1 = xi + 1; if (xi1 > MLX_W - 1) xi1 = MLX_W - 1;
    cX[x] = xi; cX1[x] = xi1; cXf[x] = tx - (int)tx;
  }
  for (int y = 0; y < IMG_H; y++) {
    int r = y * MLX_H / IMG_H;                 // 0..23
    blkY[y] = FLIP_Y ? (MLX_H - 1 - r) : r;
    float ty = y * (float)(MLX_H - 1) / (IMG_H - 1);
    if (FLIP_Y) ty = (MLX_H - 1) - ty;
    int yi = (int)ty;
    if (yi < 0) yi = 0; if (yi > MLX_H - 1) yi = MLX_H - 1;
    int yi1 = yi + 1; if (yi1 > MLX_H - 1) yi1 = MLX_H - 1;
    rY[y] = yi; rY1[y] = yi1; rYf[y] = ty - (int)ty;
  }
}

// Map a sensor cell to the CENTRE of its screen block, honouring MIRROR/FLIP
void sensorToScreen(int sx, int sy, int &X, int &Y) {
  int ux = MIRROR_X ? (MLX_W - 1 - sx) : sx;
  int uy = FLIP_Y   ? (MLX_H - 1 - sy) : sy;
  X = ux * BW + BW / 2;
  Y = uy * BH + BH / 2;
}

// ---------------------------------------------------------------------------
// Stream image (0..IMG_W-1) + legend colourbar (right strip) via SPI DMA.
void renderImage(const float *frame) {
  float span = tMax - tMin;
  if (span < 1.0f) span = 1.0f;
  float inv = 255.0f / span;

  tft.startWrite();
  tft.setAddrWindow(0, 0, TFT_W, TFT_H);
  int cur = 0;

#if SMOOTH
  for (int y = 0; y < TFT_H; y++) {
    int   y0 = rY[y]  * MLX_W;
    int   y1 = rY1[y] * MLX_W;
    float yf = rYf[y];
    uint16_t *buf = lineBuf[cur];
    for (int x = 0; x < IMG_W; x++) {
      float a = frame[y0 + cX[x]];
      float b = frame[y0 + cX1[x]];
      float c = frame[y1 + cX[x]];
      float d = frame[y1 + cX1[x]];
      float xf = cXf[x];
      float top = a + (b - a) * xf;
      float bot = c + (d - c) * xf;
      float v   = top + (bot - top) * yf;
      int idx = (int)((v - tMin) * inv);
      if (idx < 0) idx = 0; else if (idx > 255) idx = 255;
      buf[x] = palette[idx];
    }
    for (int x = IMG_W; x < TFT_W; x++)
      buf[x] = (x >= BAR_X0 && x < BAR_X1) ? legRow[y] : legBG;
    tft.pushPixelsDMA(buf, TFT_W);
    cur ^= 1;
  }
#else
  int      lastRow = -1;
  uint16_t rowCol[MLX_W];
  for (int y = 0; y < TFT_H; y++) {
    int sr = blkY[y];
    if (sr != lastRow) {
      int base = sr * MLX_W;
      for (int c = 0; c < MLX_W; c++) {
        int idx = (int)((frame[base + c] - tMin) * inv);
        if (idx < 0) idx = 0; else if (idx > 255) idx = 255;
        rowCol[c] = palette[idx];
      }
      lastRow = sr;
    }
    uint16_t *buf = lineBuf[cur];
    for (int x = 0; x < IMG_W; x++)   buf[x] = rowCol[blkX[x]];
    for (int x = IMG_W; x < TFT_W; x++)
      buf[x] = (x >= BAR_X0 && x < BAR_X1) ? legRow[y] : legBG;
    tft.pushPixelsDMA(buf, TFT_W);
    cur ^= 1;
  }
#endif

  tft.endWrite();
}

// Legend frame + min/max labels + active-end indicator (overlaid after stream)
void drawLegend() {
  tft.drawRect(BAR_X0 - 1, BAR_Y0 - 1, (BAR_X1 - BAR_X0) + 2, (BAR_Y1 - BAR_Y0) + 2, TFT_WHITE);
  tft.setTextSize(1);
  char b[8];

  tft.fillRect(BAR_X1 + 2, BAR_Y0 - 3, 32, 9, TFT_BLACK);
  snprintf(b, sizeof b, "%.1f", tMax);
  tft.setTextColor(editMax ? TFT_YELLOW : TFT_WHITE, TFT_BLACK);
  tft.setCursor(BAR_X1 + 3, BAR_Y0 - 2); tft.print(b);

  tft.fillRect(BAR_X1 + 2, BAR_Y1 - 6, 32, 9, TFT_BLACK);
  snprintf(b, sizeof b, "%.1f", tMin);
  tft.setTextColor(editMax ? TFT_WHITE : TFT_YELLOW, TFT_BLACK);
  tft.setCursor(BAR_X1 + 3, BAR_Y1 - 6); tft.print(b);

  // active-end arrow to the left of the bar
  tft.fillRect(BAR_X0 - 11, BAR_Y0 - 1, 9, (BAR_Y1 - BAR_Y0) + 2, TFT_BLACK);
  int ay = editMax ? BAR_Y0 + 1 : BAR_Y1 - 7;
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(BAR_X0 - 9, ay); tft.print(">");
}

// Replace dead pixels with the neighbour average. Catches two cases:
//  - NaN / Inf values (a common MLX90640 broken-pixel output); these must be
//    tested explicitly because every comparison against NaN is false.
//  - finite spikes: a cell beyond EVERY valid neighbour by SPIKE_THRESH.
// Genuine hot/cold regions are left untouched. NaN neighbours are ignored.
#define BAD(v) (isnan(v) || isinf(v))
void deSpike(float *f) {
  for (int y = 0; y < MLX_H; y++) {
    for (int x = 0; x < MLX_W; x++) {
      int i = y * MLX_W + x;
      float sum = 0, nmax = -1e9f, nmin = 1e9f; int n = 0;
      if (x > 0)         { float v = f[i - 1];     if (!BAD(v)) { sum += v; n++; if (v > nmax) nmax = v; if (v < nmin) nmin = v; } }
      if (x < MLX_W - 1) { float v = f[i + 1];     if (!BAD(v)) { sum += v; n++; if (v > nmax) nmax = v; if (v < nmin) nmin = v; } }
      if (y > 0)         { float v = f[i - MLX_W]; if (!BAD(v)) { sum += v; n++; if (v > nmax) nmax = v; if (v < nmin) nmin = v; } }
      if (y < MLX_H - 1) { float v = f[i + MLX_W]; if (!BAD(v)) { sum += v; n++; if (v > nmax) nmax = v; if (v < nmin) nmin = v; } }
      if (n == 0) continue;
      if (BAD(f[i]) || f[i] > nmax + SPIKE_THRESH || f[i] < nmin - SPIKE_THRESH)
        f[i] = sum / n;
    }
  }
}

float centreTemp(const float *frame) {
  int cx = MLX_W / 2, cy = MLX_H / 2;   // 16, 12
  return 0.25f * (frame[(cy - 1) * MLX_W + (cx - 1)]
                + frame[(cy - 1) * MLX_W +  cx     ]
                + frame[ cy      * MLX_W + (cx - 1)]
                + frame[ cy      * MLX_W +  cx     ]);
}

void drawMarker(int x, int y, uint16_t col, float temp) {
  if (x < 0) x = 0; if (x > IMG_W - 1) x = IMG_W - 1;
  if (y < 0) y = 0; if (y > IMG_H - 1) y = IMG_H - 1;
  tft.drawCircle(x, y, 5, TFT_BLACK);
  tft.drawCircle(x, y, 4, col);
  tft.drawPixel(x, y, col);

  char b[8];
  snprintf(b, sizeof b, "%.1f", temp);
  int tw = (int)strlen(b) * 6;
  int lx = x - tw / 2;
  if (lx < 1) lx = 1; if (lx + tw > IMG_W - 1) lx = IMG_W - 1 - tw;
  int ly = (y > 16) ? y - 15 : y + 8;
  tft.setTextSize(1);
  tft.setTextColor(col, TFT_BLACK);
  tft.fillRect(lx - 1, ly - 1, tw + 2, 9, TFT_BLACK);
  tft.setCursor(lx, ly);
  tft.print(b);
}

// ---------------------------------------------------------------------------
void sensorTask(void *pv) {
  for (;;) {
    if (mlx.getFrame(frameSensor) == 0) {
      xSemaphoreTake(frameMux, portMAX_DELAY);
      memcpy(frameShared, frameSensor, sizeof(frameShared));
      haveNew = true;
      xSemaphoreGive(frameMux);
    } else {
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT,  INPUT_PULLUP);
  pinMode(ENC_SW,  INPUT_PULLUP);

  tft.init();
  tft.setRotation(ROT);
  tft.initDMA();
  tft.setSwapBytes(true);        // buffers are filled with color565() values;
                                 // DMA pushes need byte-swap to match the panel
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(6, 6);
  tft.println("MLX90640 init...");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);

  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    tft.fillScreen(TFT_RED);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(6, 6);
    tft.println("MLX90640 not found");
    Serial.println("MLX90640 not found - check wiring / address");
    while (1) delay(10);
  }

  mlx.setMode(MLX90640_CHESS);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(MLX90640_16_HZ);   // subpage rate -> ~8 full fps
  Wire.setClock(I2C_HZ);                // re-assert after begin()

  buildPalette();
  buildLegend();
  buildMaps();
  tft.fillScreen(TFT_BLACK);

  attachInterrupt(digitalPinToInterrupt(ENC_CLK), encISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_SW),  btnISR, CHANGE);

  frameMux = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(sensorTask, "sensor", 8192, NULL, 1, NULL, 0); // core 0
}

void loop() {
  static float frame[NPIX];

  bool got = false;
  xSemaphoreTake(frameMux, portMAX_DELAY);
  if (haveNew) { memcpy(frame, frameShared, sizeof(frame)); haveNew = false; got = true; }
  xSemaphoreGive(frameMux);
  if (!got) { vTaskDelay(1); return; }

#if DESPIKE
  deSpike(frame);
#endif

  // Single scan: min/max values + their positions (for markers + snap)
  float fmin = 1e9f, fmax = -1e9f;
  int   iCold = 0, iHot = 0;
  for (int i = 0; i < NPIX; i++) {
    float v = frame[i];
    if (v < fmin) { fmin = v; iCold = i; }
    if (v > fmax) { fmax = v; iHot  = i; }
  }

  // Consume encoder / button events
  int32_t s; bool bs, bl;
  noInterrupts();
  s = encSteps; encSteps = 0;
  bs = btnShort; btnShort = false;
  bl = btnLong;  btnLong  = false;
  interrupts();

  if (bs) editMax = !editMax;                 // short press: swap active end
  if (bl) { tMin = fmin; tMax = fmax; }        // long press: snap to scene
  if (s)  { if (editMax) tMax += s * ENC_STEP; else tMin += s * ENC_STEP; }
  // clamp to sane bounds and keep max > min
  if (tMin < -40.0f) tMin = -40.0f;
  if (tMax > 300.0f) tMax = 300.0f;
  if (tMax < tMin + 1.0f) { if (editMax) tMax = tMin + 1.0f; else tMin = tMax - 1.0f; }

  renderImage(frame);
  drawLegend();

  int hx, hy, cx, cy;
  sensorToScreen(iHot  % MLX_W, iHot  / MLX_W, hx, hy);
  sensorToScreen(iCold % MLX_W, iCold / MLX_W, cx, cy);
  drawMarker(hx, hy, TFT_RED,  fmax);
  drawMarker(cx, cy, TFT_CYAN, fmin);

  // Centre crosshair (centre of the IMAGE area)
  int mx = IMG_W / 2, my = IMG_H / 2;
  tft.drawFastHLine(mx - 16, my, 33, TFT_BLACK);
  tft.drawFastVLine(mx, my - 16, 33, TFT_BLACK);
  tft.drawFastHLine(mx - 15, my, 31, TFT_WHITE);
  tft.drawFastVLine(mx, my - 15, 31, TFT_WHITE);
  tft.drawCircle(mx, my, 4, TFT_WHITE);

  // Centre temperature, top-left
  float ct = centreTemp(frame);
  tft.fillRect(0, 0, 96, 24, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(4, 4);
  tft.printf("%.1f", ct);
  tft.print((char)247);                 // degree symbol; delete if it looks odd
  tft.print("C");

  Serial.printf("centre %.2f  scale[%.1f..%.1f] %s  hot %.2f  cold %.2f\n",
                ct, tMin, tMax, editMax ? "MAX" : "MIN", fmax, fmin);
}
