#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string>
#include <WiFi.h>
#include <WebServer.h>

// Home menu with four apps for the NON-TOUCH Waveshare ESP32-S3-LCD-1.28
// (round 240x240 GC9A01 display, QMI8658 IMU, BOOT button on GPIO0).
//
//   MAZE   tilt the board to roll the ball through the rings of a round labyrinth to the middle (4 levels)
//   STEPS  step counter (counts in the background, saved to flash)
//   TIMER  flip-to-start kitchen timer (logic from examples/flip_timer.cpp)
//   SCALE  slide the board along a thing and read its length in cm (distance from the IMU, approx)
//
// Controls:
//   BOOT button  MENU: short press = next tile, hold 1 s = open the selected app
//                APPS: hold 1 s = back to the menu; a short press does the app action shown on the screen
//   Bluetooth    BLE UART (Nordic UART service), device name BLE_NAME. Connect with a BLE terminal app
//                (nRF Connect, Serial Bluetooth Terminal in BLE mode, LightBlue) and send a text command:
//                1 / maze, 2 / steps, 3 / timer, 4 / scale = open that app
//                0 / menu = back to the menu     click = same as a BOOT short press     status, help
//                The board replies on the same service (screen changes, the scale result, step count).
//   Wi-Fi        The board makes its own network WIFI_SSID (password WIFI_PASS). Join it on the phone and
//                open http://192.168.4.1 for buttons: BALL, STEPS, TIMER, SCALE, MENU, CLICK.
// Everything is drawn into a full-screen sprite that is pushed to the panel once per frame.

// ---------------- hardware ----------------
#define SCR 240
#define CX  120
#define CY  120

#define I2C_SDA    6
#define I2C_SCL    7
#define BOOT_PIN   0

#define IMU_SAMPLE_US  8000      // 125 Hz
#define ACC_LSB        8192.0f   // +/-4 g range
#define GYRO_LSB       64.0f     // +/-512 dps range
#define ACC_Y_OFFSET_G 0.10f     // the accelerometer Y axis reads about +0.1 g too high

// IMU axis -> screen axis for tilt (the face looks where the board is tilted). Screen X grows to the right, screen Y downwards.
// The eyes look towards +X when TILT_X_SIGN * accel[TILT_X_AXIS] is positive (same for Y).
// Flip a sign or swap the axis indices (0 = IMU X, 1 = IMU Y) if the eyes look the wrong way.
#define TILT_X_AXIS 0
#define TILT_X_SIGN (+1.0f)
#define TILT_Y_AXIS 1
#define TILT_Y_SIGN (-1.0f)

// BOOT button
#define BTN_LONG_MS     1000
#define BTN_DEBOUNCE_MS 30

// Menu
#define TILE_HALF    40          // tiles are 80x80
#define TILE_RADIUS  18

// Steps
#define STEP_GOAL        10000
#define STEP_M           0.7f    // metres per step
#define STEP_KCAL        0.04f   // kcal per step (rough)
#define STEP_LP_ALPHA    0.20f   // low-pass on |a| per 125 Hz sample (about 4 Hz)
#define STEP_DC_ALPHA    0.02f   // slow filter that follows gravity, removed from the signal
#define STEP_ENV_DECAY   0.995f  // decay per sample of the recent peak amplitude
#define STEP_MIN_AMP_G   0.10f   // smallest swing that can count as a step
#define STEP_THR_FRAC    0.5f    // threshold as a fraction of the recent peak amplitude
#define STEP_MIN_MS      280     // minimum interval between steps
#define STEP_MAX_MS      2000    // longer gap = the walk ended
#define STEP_CONFIRM_N   3       // steps in a row needed before they are counted
#define STEP_SAVE_MS     30000
#define STEP_SAVE_EVERY  100
#define STEP_CONFIRM_MS  4000    // the reset question goes away after this long without a click

// Flip timer (as in flip_timer.cpp)
#define FACE_THRESHOLD_G   0.5f  // gravity on the dominant axis must exceed this (g)
#define FACE_DEBOUNCE_MS   100
#define ROTATE_DEBOUNCE_MS 300   // the display follows the board orientation after this delay
#define ROT_SIDE_A 1             // display rotation for the two sideways orientations
#define ROT_SIDE_B 3
#define ROT_OFFSET 2             // the IMU is mounted 180 degrees to the screen

// Bluetooth (Nordic UART service)
#define BLE_NAME      "MenuBot"
#define NUS_SERVICE   "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX        "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // the phone writes commands here
#define NUS_TX        "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // the board notifies replies here

// Wi-Fi access point
#define WIFI_SSID     "MenuBot"
#define WIFI_PASS     "12345678"                               // at least 8 characters

// Scale
#define SLIDE_CALIB_N 75         // 0.6 s of stillness to measure the sensor bias
#define SLIDE_PRE_N   13         // samples kept from before the motion was detected
#define SLIDE_MAX_N   1250       // 10 s of recording
#define SLIDE_STILL_N 50         // 0.4 s at rest ends the measurement
#define SLIDE_START_G 0.04f      // the movement must exceed this (g, filtered) to count as a slide
#define SLIDE_MIN_PEAK_G 0.06f   // a slide that never reaches this is just noise
#define SLIDE_MIN_N   25         // and has to last at least 0.2 s
#define SLIDE_MAX_CM  300.0f     // anything longer is a drifting sensor, not a measurement
#define GRAV          9.80665f

// Colours
#define RGB565(r, g, b) ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
static const uint16_t C_DIM      = RGB565(110, 110, 110);
static const uint16_t C_TRACK    = RGB565(32, 32, 32);
static const uint16_t C_PANEL    = RGB565(24, 26, 34);
static const uint16_t C_TILE     = RGB565(34, 36, 44);
static const uint16_t C_TILE_HI  = RGB565(74, 78, 92);
static const uint16_t C_AMBER    = RGB565(255, 180, 40);
static const uint16_t C_TEAL     = RGB565(40, 210, 190);
static const uint16_t C_BLUE     = RGB565(70, 160, 255);
static const uint16_t C_PURPLE   = RGB565(190, 110, 255);
static const uint16_t C_GREEN    = RGB565(60, 220, 120);
static const uint16_t C_RED      = RGB565(255, 60, 60);
static const uint16_t C_DARKRED  = RGB565(110, 20, 20);
static const uint16_t C_CYAN     = RGB565(0, 200, 255);

TFT_eSPI tft;
TFT_eSprite spr(&tft);
Preferences prefs;

enum Screen { MENU = 0, APP_BALL, APP_STEPS, APP_TIMER, APP_SCALE };
static void switchTo(Screen s);

// ======================= QMI8658 IMU =======================
static uint8_t imuAddr = 0;
static float imuAcc[3] = {0, 0, 1};                  // latest accelerometer sample (g)

static bool imuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(imuAddr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool imuRead(uint8_t reg, uint8_t *buf, size_t len) {
    Wire.beginTransmission(imuAddr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)imuAddr, (int)len) != (int)len) return false;
    for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

static bool imuInit() {
    const uint8_t addrs[2] = {0x6B, 0x6A};
    for (uint8_t a : addrs) {
        imuAddr = a;
        uint8_t id = 0;
        if (imuRead(0x00, &id, 1) && id == 0x05) {   // WHO_AM_I
            imuWrite(0x02, 0x40);                    // CTRL1: address auto-increment, little-endian
            imuWrite(0x03, 0x16);                    // CTRL2: accel +/-4g, 125 Hz
            imuWrite(0x04, 0x56);                    // CTRL3: gyro +/-512 dps, 125 Hz
            imuWrite(0x08, 0x03);                    // CTRL7: enable accelerometer and gyroscope
            return true;
        }
    }
    imuAddr = 0;
    return false;
}

// a in g, g in deg/s
static bool imuReadAll(float a[3], float g[3]) {
    uint8_t b[12];
    if (!imuRead(0x35, b, 12)) return false;         // ACCEL_X_L .. GYRO_Z_H
    for (int i = 0; i < 3; i++) {
        a[i] = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8)) / ACC_LSB;
        g[i] = (int16_t)(b[6 + 2 * i] | (b[7 + 2 * i] << 8)) / GYRO_LSB;
    }
    a[1] -= ACC_Y_OFFSET_G;
    return true;
}

// ======================= BOOT button =======================
enum BtnEvent { BTN_NONE, BTN_SHORT, BTN_LONG };

static BtnEvent buttonPoll(uint32_t now) {
    static bool wasDown = false, longFired = false;
    static uint32_t downSince = 0;
    bool down = digitalRead(BOOT_PIN) == LOW;
    BtnEvent ev = BTN_NONE;
    if (down && !wasDown) {
        downSince = now;
        longFired = false;
    } else if (down && !longFired && now - downSince >= BTN_LONG_MS) {
        longFired = true;
        ev = BTN_LONG;
    } else if (!down && wasDown && !longFired && now - downSince > BTN_DEBOUNCE_MS) {
        ev = BTN_SHORT;
    }
    wasDown = down;
    return ev;
}

// ======================= shared drawing =======================
static void text(const char *s, int x, int y, const GFXfont *f, uint16_t col) {
    spr.setFreeFont(f);
    spr.setTextColor(col);                           // transparent background, so haze and panels show through
    spr.drawString(s, x, y);
}

// Ring segment that starts at 12 o'clock and runs clockwise. (TFT_eSPI arcs start at 6 o'clock.)
static void drawArcFrom12(int r, int ir, float frac, uint16_t color) {
    uint32_t a = (uint32_t)(constrain(frac, 0.0f, 1.0f) * 360.0f);
    if (a == 0) return;
    if (a <= 180) {
        spr.drawSmoothArc(CX, CY, r, ir, 180, 180 + a, color, TFT_BLACK, true);
    } else {
        spr.drawSmoothArc(CX, CY, r, ir, 180, 360, color, TFT_BLACK, true);
        spr.drawSmoothArc(CX, CY, r, ir, 0, a - 180, color, TFT_BLACK, true);
    }
}

// Dark rounded panel for overlays (level clear, confirmations).
static void drawPanel() {
    spr.fillSmoothRoundRect(36, 56, 168, 128, 18, C_PANEL);
}

// ======================= Bluetooth (BLE UART) =======================
static BLECharacteristic *bleTx = nullptr;
static volatile bool bleConnected = false;
static char bleCmd[24];                              // command written by the phone, handled in loop()
static volatile bool bleCmdReady = false;

class BleServerCb : public BLEServerCallbacks {
    void onConnect(BLEServer *) override { bleConnected = true; }
    void onDisconnect(BLEServer *s) override {
        bleConnected = false;
        s->startAdvertising();
    }
};

class BleRxCb : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *c) override {
        if (bleCmdReady) return;                     // the previous command is still being handled
        std::string v = c->getValue();
        size_t n = v.size() < sizeof(bleCmd) - 1 ? v.size() : sizeof(bleCmd) - 1;
        memcpy(bleCmd, v.data(), n);
        bleCmd[n] = 0;
        bleCmdReady = true;
    }
};

static char lastMsg[64] = "";                       // last message sent to the phone (shown on the web page)

// Message for the phone: a BLE notification, and kept for the web page.
static void remoteSend(const char *msg) {
    strlcpy(lastMsg, msg, sizeof(lastMsg));
    if (!bleConnected || !bleTx) return;
    String s = String(msg) + "\n";
    bleTx->setValue((uint8_t *)s.c_str(), s.length());
    bleTx->notify();
}

static void bleInit() {
    BLEDevice::init(BLE_NAME);
    BLEServer *server = BLEDevice::createServer();
    server->setCallbacks(new BleServerCb());
    BLEService *svc = server->createService(NUS_SERVICE);
    bleTx = svc->createCharacteristic(NUS_TX, BLECharacteristic::PROPERTY_NOTIFY);
    bleTx->addDescriptor(new BLE2902());
    BLECharacteristic *rx = svc->createCharacteristic(NUS_RX, BLECharacteristic::PROPERTY_WRITE |
                                                                BLECharacteristic::PROPERTY_WRITE_NR);
    rx->setCallbacks(new BleRxCb());
    svc->start();
    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(NUS_SERVICE);
    adv->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.println("BLE advertising as " BLE_NAME);
}

// ======================= step detector (runs in the background) =======================
static struct {
    uint32_t count = 0, savedCount = 0, lastSaveMs = 0, lastCandMs = 0;
    float lp = 0, dc = 0, env = 0;                   // filtered |a|, gravity estimate, peak envelope
    float cad = 0;                                   // cadence, steps per minute (smoothed)
    bool above = false;
    uint8_t pending = 0;                             // steps seen but not yet confirmed
} st;

static void stepsOnSample(const float a[3], uint32_t now) {
    float mag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (st.dc == 0) st.lp = st.dc = mag;
    st.lp += (mag - st.lp) * STEP_LP_ALPHA;
    st.dc += (st.lp - st.dc) * STEP_DC_ALPHA;
    float sig = st.lp - st.dc;                       // band-passed: tilt and slow motion fall out
    st.env = fmaxf(sig, st.env * STEP_ENV_DECAY);
    float thr = fmaxf(STEP_MIN_AMP_G, st.env * STEP_THR_FRAC);

    if (!st.above && sig > thr) {                    // rising edge through the threshold = peak
        st.above = true;
        if (now - st.lastCandMs > STEP_MAX_MS) st.pending = 0;
        if (now - st.lastCandMs >= STEP_MIN_MS) {
            uint32_t iv = now - st.lastCandMs;
            if (iv < STEP_MAX_MS) st.cad = st.cad > 0 ? st.cad * 0.7f + (60000.0f / iv) * 0.3f : 60000.0f / iv;
            else st.cad = 0;
            st.lastCandMs = now;
            if (st.pending < STEP_CONFIRM_N) {
                if (++st.pending == STEP_CONFIRM_N) st.count += STEP_CONFIRM_N;
            } else {
                st.count++;
            }
        }
    } else if (st.above && sig < thr * 0.3f) {
        st.above = false;
    }
}

static void stepsSave(uint32_t now, bool force) {
    if (st.count == st.savedCount) return;
    bool due = now - st.lastSaveMs >= STEP_SAVE_MS || st.count - st.savedCount >= STEP_SAVE_EVERY;
    if (!force && !due) return;
    prefs.putUInt("steps", st.count);
    st.savedCount = st.count;
    st.lastSaveMs = now;
}

// ======================= APP: MAZE (ball-in-a-maze puzzle) =======================
// A round labyrinth like the wooden toy: tilt the board to roll the ball from the outer groove, through the gap in
// each ring, to the middle. A short spoke in every groove blocks the short way round, so pick the right direction.
// Colours follow the case and the face: black screen, orange walls (the orange of the eyes), white ball.
// Angles are in degrees on the screen: 0 = right, 90 = down, 180 = left, 270 = up.
#define MZ_R          5.0f       // ball radius
#define MZ_RIM        116.0f     // inner edge of the outer wall
#define MZ_HALF_T     3.0f       // half thickness of the rings and spokes
#define MZ_GAP_PX     22.0f      // distance between the rounded ends of a ring at its gap (clear width = this - 2 * MZ_HALF_T)
#define MZ_ACCEL      1400.0f    // px/s^2 per g of tilt
#define MZ_FRICTION   1.2f       // velocity loss per second
#define MZ_VMAX       400.0f     // px/s
#define MZ_REST       0.3f       // bounce
#define MZ_DEADZONE_G 0.03f      // tilt below this is treated as flat
#define MZ_GOAL_R     16.0f      // ball centre this close to the middle = solved
#define MZ_N_RINGS    4
#define MZ_N_LEVELS   4
static const float MZ_RING_R[MZ_N_RINGS] = {97, 75, 53, 31};   // radius of the centre line of each ring wall

// Groove i is the channel just outside ring i (groove 0 is the outer one). The ball leaves groove i through the gap of
// ring i, so a level is solved by: reach gap[0], then gap[1], ... then the middle.
struct MzLevel {
    int16_t start;                                   // where the ball starts in the outer groove
    int16_t gap[MZ_N_RINGS];                         // gap angle of each ring
    int16_t spoke[MZ_N_RINGS];                       // spoke angle in each groove, -1 = none
};
static const MzLevel MZ_LEVELS[MZ_N_LEVELS] = {
    {  0, {180,   0, 180,   0}, { 90, 270,  90,  -1}},
    {270, { 90, 200, 340, 120}, {  0, 150, 270,  60}},
    {180, {330, 150, 270,  30}, {250,  60, 210, 330}},
    { 90, {270,  40, 220, 100}, {200, 350, 130, 160}},
};

static TFT_eSprite mzBg(&tft);                       // the maze is painted once per level into this, then copied every frame
static bool mzBgOk = false;

static struct {
    int level;
    float x, y, vx, vy;
    uint32_t startMs, clearMs;
    bool cleared, bestLoaded;
    float clearS;
    float best[MZ_N_LEVELS];                         // best time per level (s), 0 = none
    float spkAx[MZ_N_RINGS], spkAy[MZ_N_RINGS], spkBx[MZ_N_RINGS], spkBy[MZ_N_RINGS];   // spoke end points
} mz;

// Spark particles for pick-ups and the goal.
struct Particle { float x, y, vx, vy; int16_t life; uint16_t col; };
static Particle parts[40];

static void burst(float x, float y, int n, uint16_t c1, uint16_t c2) {
    for (int k = 0; k < n; k++) {
        for (Particle &p : parts) {
            if (p.life > 0) continue;
            float ang = random(0, 628) / 100.0f, sp = 40.0f + random(0, 110);
            p.x = x; p.y = y;
            p.vx = cosf(ang) * sp; p.vy = sinf(ang) * sp;
            p.life = 350 + random(0, 350);
            p.col = (k & 1) ? c1 : c2;
            break;
        }
    }
}

// Draws a clockwise arc from startDeg to endDeg (screen angles: 0 = right). TFT_eSPI wants 0 = 6 o'clock, max 360.
static void mzArcDeg(TFT_eSprite &t, int r, int ir, float startDeg, float endDeg, uint16_t col, uint16_t bg) {
    auto tf = [](float d) { int v = ((int)lroundf(d) - 90) % 360; return v < 0 ? v + 360 : v; };
    int s = tf(startDeg), e = tf(endDeg);
    if (s < e) {
        t.drawSmoothArc(CX, CY, r, ir, s, e, col, bg, true);
    } else {
        t.drawSmoothArc(CX, CY, r, ir, s, 360, col, bg, true);
        if (e > 0) t.drawSmoothArc(CX, CY, r, ir, 0, e, col, bg, true);
    }
}

// Paints the whole maze (floor, rim, spokes, rings). Slow because of the anti-aliased arcs, so it is done once per level.
static void mzPaint(TFT_eSprite &t) {
    const uint16_t FLOOR = RGB565(20, 9, 0), EDGE = RGB565(150, 70, 0), ORANGE = RGB565(255, 170, 0), HI = RGB565(255, 225, 140);
    const MzLevel &L = MZ_LEVELS[mz.level];
    t.fillSprite(TFT_BLACK);
    t.fillCircle(CX, CY, 118, FLOOR);
    t.fillCircle(CX, CY, 28, RGB565(4, 2, 0));
    for (int i = 0; i < MZ_N_RINGS; i++) {
        if (L.spoke[i] < 0) continue;
        t.drawWideLine(mz.spkAx[i], mz.spkAy[i], mz.spkBx[i], mz.spkBy[i], 6.0f, EDGE, FLOOR);
        t.drawWideLine(mz.spkAx[i], mz.spkAy[i], mz.spkBx[i], mz.spkBy[i], 4.0f, ORANGE, EDGE);
    }
    for (int i = 0; i < MZ_N_RINGS; i++) {
        int rm = (int)MZ_RING_R[i];
        float gapDeg = MZ_GAP_PX / MZ_RING_R[i] * RAD_TO_DEG;
        float s0 = L.gap[i] + gapDeg / 2, e0 = L.gap[i] + 360.0f - gapDeg / 2;
        mzArcDeg(t, rm + 3, rm - 3, s0, e0, EDGE, FLOOR);
        mzArcDeg(t, rm + 2, rm - 2, s0, e0, ORANGE, EDGE);
        mzArcDeg(t, rm + 2, rm + 1, s0, e0, HI, ORANGE);
    }
    t.drawSmoothArc(CX, CY, 120, 116, 0, 360, EDGE, TFT_BLACK);
    t.drawSmoothArc(CX, CY, 119, 117, 0, 360, ORANGE, EDGE);
    t.drawSmoothArc(CX, CY, 119, 118, 0, 360, HI, ORANGE);
}

static void mzStartLevel() {
    const MzLevel &L = MZ_LEVELS[mz.level];
    float a = L.start * DEG_TO_RAD, r = (MZ_RIM + MZ_RING_R[0] + MZ_HALF_T) / 2;   // middle of the outer groove
    mz.x = CX + r * cosf(a);
    mz.y = CY + r * sinf(a);
    mz.vx = mz.vy = 0;
    mz.cleared = false;
    mz.startMs = millis();
    for (int i = 0; i < MZ_N_RINGS; i++) {
        float sa = L.spoke[i] * DEG_TO_RAD;
        float ro = i == 0 ? 118.0f : MZ_RING_R[i - 1], ri = MZ_RING_R[i];
        mz.spkAx[i] = CX + ro * cosf(sa);
        mz.spkAy[i] = CY + ro * sinf(sa);
        mz.spkBx[i] = CX + ri * cosf(sa);
        mz.spkBy[i] = CY + ri * sinf(sa);
    }
    for (Particle &p : parts) p.life = 0;
    if (mzBgOk) mzPaint(mzBg);
}

static void mzInit() {
    if (!mz.bestLoaded) {
        for (int i = 0; i < MZ_N_LEVELS; i++) {
            char key[8];
            snprintf(key, sizeof(key), "mz%d", i);
            mz.best[i] = prefs.getUInt(key, 0) / 1000.0f;
        }
        mz.bestLoaded = true;
    }
    if (!mzBgOk) {
        mzBg.setColorDepth(16);
        mzBgOk = mzBg.createSprite(SCR, SCR) != nullptr;
        if (!mzBgOk) Serial.println("maze: no memory for the background sprite, drawing slowly");
    }
    mzStartLevel();
}

static void mzExit() {
    if (mzBgOk) {
        mzBg.deleteSprite();
        mzBgOk = false;
    }
}

// Moves the ball out of a wall: n = unit normal pointing away from the wall, pen = how deep it sank in.
static void mzPush(float nx, float ny, float pen) {
    mz.x += nx * pen;
    mz.y += ny * pen;
    float vn = mz.vx * nx + mz.vy * ny;
    if (vn < 0) {
        mz.vx -= (1.0f + MZ_REST) * vn * nx;
        mz.vy -= (1.0f + MZ_REST) * vn * ny;
    }
}

static void mzCollide() {
    const MzLevel &L = MZ_LEVELS[mz.level];
    const float need = MZ_HALF_T + MZ_R;
    float dx = mz.x - CX, dy = mz.y - CY, rr = hypotf(dx, dy);
    if (rr > MZ_RIM - MZ_R) mzPush(-dx / rr, -dy / rr, rr - (MZ_RIM - MZ_R));

    for (int i = 0; i < MZ_N_RINGS; i++) {           // ring walls: an arc with a gap, rounded at both ends
        dx = mz.x - CX;
        dy = mz.y - CY;
        rr = hypotf(dx, dy);
        if (rr < 1e-3f) break;
        float rm = MZ_RING_R[i], gapA = MZ_GAP_PX / rm;
        float a0 = L.gap[i] * DEG_TO_RAD + gapA / 2, sweep = TWO_PI - gapA;
        float phi = fmodf(atan2f(dy, dx) - a0, TWO_PI);
        if (phi < 0) phi += TWO_PI;
        if (phi <= sweep) {
            float d = rr - rm, ad = fabsf(d);
            if (ad < need) {
                float s = d >= 0 ? 1.0f : -1.0f;
                mzPush(s * dx / rr, s * dy / rr, need - ad);
            }
        } else {                                     // in the gap: only the two rounded ends can be hit
            float ea = (phi - sweep < TWO_PI - phi) ? a0 + sweep : a0;
            float ex = mz.x - (CX + rm * cosf(ea)), ey = mz.y - (CY + rm * sinf(ea)), d = hypotf(ex, ey);
            if (d < need && d > 1e-3f) mzPush(ex / d, ey / d, need - d);
        }
    }

    for (int i = 0; i < MZ_N_RINGS; i++) {           // spokes
        if (L.spoke[i] < 0) continue;
        float ax = mz.spkAx[i], ay = mz.spkAy[i], ex = mz.spkBx[i] - ax, ey = mz.spkBy[i] - ay;
        float t = constrain(((mz.x - ax) * ex + (mz.y - ay) * ey) / (ex * ex + ey * ey), 0.0f, 1.0f);
        float sx = mz.x - (ax + ex * t), sy = mz.y - (ay + ey * t), d = hypotf(sx, sy);
        if (d < need && d > 1e-3f) mzPush(sx / d, sy / d, need - d);
    }
}

// Called at the IMU rate: the tilt rolls the ball.
static void mzSample(const float a[3], const float g[3], float dt, uint32_t tUs) {
    (void)g;
    (void)tUs;
    if (mz.cleared) return;
    float tx = -TILT_X_SIGN * a[TILT_X_AXIS], ty = TILT_Y_SIGN * a[TILT_Y_AXIS];   // X is mirrored compared with the eyes
    if (fabsf(tx) < MZ_DEADZONE_G) tx = 0;
    if (fabsf(ty) < MZ_DEADZONE_G) ty = 0;
    const int N = 2;                                 // sub-steps keep the ball from skipping through thin walls
    float h = dt / N;
    for (int s = 0; s < N; s++) {
        float f = max(0.0f, 1.0f - MZ_FRICTION * h);
        mz.vx = (mz.vx + tx * MZ_ACCEL * h) * f;
        mz.vy = (mz.vy + ty * MZ_ACCEL * h) * f;
        float sp = hypotf(mz.vx, mz.vy);
        if (sp > MZ_VMAX) {
            mz.vx *= MZ_VMAX / sp;
            mz.vy *= MZ_VMAX / sp;
        }
        mz.x += mz.vx * h;
        mz.y += mz.vy * h;
        mzCollide();
    }
    if (hypotf(mz.x - CX, mz.y - CY) < MZ_GOAL_R) {  // dropped into the middle
        uint32_t now = millis();
        mz.cleared = true;
        mz.clearMs = now;
        mz.clearS = (now - mz.startMs) / 1000.0f;
        float &b = mz.best[mz.level];
        if (b <= 0 || mz.clearS < b) {
            b = mz.clearS;
            char key[8];
            snprintf(key, sizeof(key), "mz%d", mz.level);
            prefs.putUInt(key, (uint32_t)(b * 1000.0f));
        }
        burst(CX, CY, 24, RGB565(255, 170, 0), TFT_WHITE);
    }
}

// Click: restart the level, or go to the next one after a clear.
static void mzButton() {
    if (mz.cleared) mz.level = (mz.level + 1) % MZ_N_LEVELS;
    mzStartLevel();
}

// Scales an RGB565 colour: pct 100 = unchanged, 50 = half as bright, above 100 = brighter (clamped).
static uint16_t shade(uint16_t c, int pct) {
    int r = ((c >> 11) & 31) * pct / 100;
    int g = ((c >> 5) & 63) * pct / 100;
    int b = (c & 31) * pct / 100;
    if (r > 31) r = 31;
    if (g > 63) g = 63;
    if (b > 31) b = 31;
    return (uint16_t)((r << 11) | (g << 5) | b);
}

static void mzDraw(uint32_t now) {
    static uint32_t lastMs = 0;
    static float trail[6][2];
    uint32_t dtMs = lastMs ? min<uint32_t>(now - lastMs, 100) : 0;
    lastMs = now;
    char buf[24];
    const uint16_t ORANGE = RGB565(255, 170, 0);

    if (mzBgOk) mzBg.pushToSprite(&spr, 0, 0);
    else mzPaint(spr);

    // the goal in the middle: pulsing ring and one dot per level
    float pulse = 0.5f + 0.5f * sinf(now / 260.0f);
    spr.drawSmoothArc(CX, CY, 25, 22, 0, 360, shade(ORANGE, 55 + (int)(pulse * 45)), RGB565(4, 2, 0));
    for (int i = 0; i < MZ_N_LEVELS; i++) {
        spr.fillSmoothCircle(CX + (int)((i - (MZ_N_LEVELS - 1) / 2.0f) * 9), CY, 2, i == mz.level ? ORANGE : RGB565(80, 38, 0));
    }

    // orange comet trail and the white ball
    memmove(trail[1], trail[0], sizeof(trail) - sizeof(trail[0]));
    trail[0][0] = mz.x;
    trail[0][1] = mz.y;
    if (!mz.cleared) {
        for (int i = 5; i >= 1; i--) {
            if (hypotf(trail[i][0] - mz.x, trail[i][1] - mz.y) < 3.0f) continue;
            spr.fillSmoothCircle((int)trail[i][0], (int)trail[i][1], 4 - i / 3, shade(ORANGE, 12 + (6 - i) * 6));
        }
    }
    float br = MZ_R;
    if (mz.cleared) br = max(1.0f, MZ_R * (1.0f - 0.8f * min(1.0f, (now - mz.clearMs) / 400.0f)));   // sinks into the hole
    int bx = lroundf(mz.x), by = lroundf(mz.y), r0 = (int)lroundf(br);
    spr.fillSmoothCircle(bx, by, r0 + 1, RGB565(255, 190, 80));          // warm glow on the orange maze
    spr.fillSmoothCircle(bx, by, r0, RGB565(150, 152, 168));
    spr.fillSmoothCircle(bx - 1, by - 1, max(1, r0 - 1), RGB565(225, 227, 238));
    if (r0 > 2) spr.fillSmoothCircle(bx - 2, by - 2, 1, TFT_WHITE);

    float dts = dtMs / 1000.0f;                      // sparks
    for (Particle &p : parts) {
        if (p.life <= 0) continue;
        p.x += p.vx * dts;
        p.y += p.vy * dts;
        p.vx *= 0.97f;
        p.vy *= 0.97f;
        p.life -= dtMs;
        spr.fillCircle((int)p.x, (int)p.y, p.life > 200 ? 2 : 1, p.col);
    }

    if (mz.cleared && now - mz.clearMs > 600) {
        drawPanel();
        spr.drawSmoothRoundRect(36, 56, 18, 17, 168, 128, ORANGE, C_PANEL);
        text(mz.level == MZ_N_LEVELS - 1 ? "ALL DONE!" : "LEVEL CLEAR", CX, 78, &FreeSansBold12pt7b, ORANGE);
        snprintf(buf, sizeof(buf), "%.1f s", mz.clearS);
        text(buf, CX, 112, &FreeSansBold18pt7b, TFT_WHITE);
        snprintf(buf, sizeof(buf), "BEST %.1f s", mz.best[mz.level]);
        text(buf, CX, 142, &FreeSansBold9pt7b, C_DIM);
        text("CLICK: NEXT", CX, 168, &FreeSansBold9pt7b, ORANGE);
    }
    spr.pushSprite(0, 0);
}

// ======================= APP: STEP TRACKER =======================
static bool stepsNoticeDone = false;                 // the "worn or carried" notice, once per power-up
static bool stepsNotice = false, stepsConfirm = false;
static uint32_t stepsConfirmMs = 0;
static uint32_t stepsLastCount = 0, stepsFlashMs = 0;   // a new step makes the ring head flare for a moment
static float stepsDisp = 0, stepsCad = 0;            // progress and cadence as shown (they ease towards the real values)
static float stepsPhase = 0, stepsAmp = 0;           // walker gait phase, and how strongly it swings (0 = standing)
static uint32_t stepsLastDrawMs = 0;
static TFT_eSprite stepsBg(&tft);                    // dark haze and the ring tracks, painted once
static bool stepsBgOk = false;

static void stepsPaintBg() {
    stepsBg.fillSprite(TFT_BLACK);
    for (int y = 0; y < SCR; y++) {                  // soft teal haze, brightest in the middle
        for (int x = 0; x < SCR; x++) {
            int dx = x - CX, dy = y - CY;
            int d2 = dx * dx + dy * dy;
            if (d2 > 118 * 118) continue;
            float t = 1.0f - sqrtf((float)d2) / 118.0f;
            t *= t;
            stepsBg.drawPixel(x, y, RGB565((int)(2 + 12 * t), (int)(5 + 40 * t), (int)(9 + 50 * t)));
        }
    }
    stepsBg.drawSmoothArc(CX, CY, 119, 118, 0, 360, RGB565(30, 42, 48), TFT_BLACK);
    stepsBg.drawSmoothArc(CX, CY, 117, 105, 0, 360, RGB565(18, 30, 36), TFT_BLACK);   // outer track
    stepsBg.drawSmoothArc(CX, CY, 98, 91, 0, 360, RGB565(16, 26, 30), TFT_BLACK);     // cadence track
    for (int k = 1; k < 4; k++) {                    // quarter marks on the outer track
        float a = k * PI / 2;
        stepsBg.fillSmoothCircle(CX + (int)lroundf(sinf(a) * 111), CY - (int)lroundf(cosf(a) * 111), 1, RGB565(80, 96, 104));
    }
}

static void stepsInit() {
    stepsNotice = !stepsNoticeDone;
    stepsConfirm = false;
    stepsLastCount = st.count;
    stepsFlashMs = 0;
    stepsDisp = stepsCad = stepsAmp = stepsPhase = 0;
    stepsLastDrawMs = 0;
    if (!stepsBgOk) {
        stepsBg.setColorDepth(16);
        stepsBgOk = stepsBg.createSprite(SCR, SCR) != nullptr;
    }
    if (stepsBgOk) stepsPaintBg();
}

// Short press: dismiss the notice; otherwise ask to reset, and reset on a second press.
static void stepsButton() {
    if (stepsNotice) {
        stepsNotice = false;
        stepsNoticeDone = true;
    } else if (stepsConfirm) {
        st.count = 0;
        st.pending = 0;
        stepsSave(millis(), true);
        stepsConfirm = false;
    } else {
        stepsConfirm = true;
        stepsConfirmMs = millis();
    }
}

static void stepsUpdate(uint32_t now) {
    if (stepsConfirm && now - stepsConfirmMs >= STEP_CONFIRM_MS) stepsConfirm = false;
}

static uint16_t mix565(uint16_t a, uint16_t b, int t256) {   // blend two RGB565 colours, t256 0..256
    int r = (((a >> 11) & 31) * (256 - t256) + ((b >> 11) & 31) * t256) >> 8;
    int g = (((a >> 5) & 63) * (256 - t256) + ((b >> 5) & 63) * t256) >> 8;
    int bl = ((a & 31) * (256 - t256) + (b & 31) * t256) >> 8;
    return (uint16_t)((r << 11) | (g << 5) | bl);
}

// Colour along a ring (t = 0..256 round the circle): 0 aurora (violet, cyan, green), 1 gold (goal reached), 2 ember (cadence).
static uint16_t stepsGrad(int style, int t) {
    t = constrain(t, 0, 256);
    if (style == 1) return mix565(RGB565(255, 140, 0), RGB565(255, 235, 110), t);
    if (style == 2) return mix565(RGB565(255, 80, 0), RGB565(255, 205, 50), t);
    if (t < 128) return mix565(RGB565(150, 80, 255), RGB565(0, 200, 255), t * 2);
    return mix565(RGB565(0, 200, 255), RGB565(70, 255, 140), (t - 128) * 2);
}

// Arc from 12 o'clock, clockwise, built from short anti-aliased capsules: round ends and a colour that changes along the arc.
static void stepsArc(float rad, float hw, float frac, int style) {
    frac = constrain(frac, 0.0f, 1.0f);
    if (frac < 0.003f) return;
    const float STEP = 5.0f * PI / 180.0f;
    float end = frac * 2.0f * PI, th = 0;
    float px = CX, py = CY - rad;
    while (true) {
        float nt = fminf(th + STEP, end);
        float nx = CX + sinf(nt) * rad, ny = CY - cosf(nt) * rad;
        spr.drawWedgeLine(px, py, nx, ny, hw, hw, stepsGrad(style, (int)(nt / (2.0f * PI) * 256)));
        if (nt >= end) break;
        th = nt;
        px = nx;
        py = ny;
    }
}

// Soft round glow: the colour is blended over what is already there, strongest in the middle.
static void stepsGlow(float cx, float cy, int r, uint16_t col, int maxA) {
    int x0 = (int)lroundf(cx), y0 = (int)lroundf(cy);
    for (int y = -r; y <= r; y++) {
        for (int x = -r; x <= r; x++) {
            int d2 = x * x + y * y;
            if (d2 >= r * r) continue;
            int px = x0 + x, py = y0 + y;
            if (px < 0 || py < 0 || px >= SCR || py >= SCR) continue;
            float t = 1.0f - sqrtf((float)d2) / r;
            int a = (int)(maxA * t * t);
            if (a > 0) spr.drawPixel(px, py, mix565(spr.readPixel(px, py), col, a));
        }
    }
}

// Little runner seen from the side, facing right. ph = gait phase, amp 0 = standing still, 1 = full stride.
static void stepsWalker(float cx, float hipY, float ph, float amp, uint16_t col) {
    const float L1 = 11.0f, L2 = 11.0f, LT = 14.0f, LA = 8.0f;
    float lean = 0.10f + 0.06f * amp;
    float hy = hipY + amp * cosf(2.0f * ph);
    float sx = cx + sinf(lean) * LT, sy = hy - cosf(lean) * LT;      // shoulder
    for (int pass = 0; pass < 2; pass++) {           // far limbs first (dim), then the body and the near limbs
        float p = ph + (pass == 0 ? PI : 0.0f);
        uint16_t c = pass == 0 ? shade(col, 62) : col;
        float t = amp * 0.7f * sinf(p);              // thigh swing, positive = forward
        float bend = amp * fmaxf(0.0f, cosf(p)) + 0.05f;   // knee folds while the leg swings forward
        float kx = cx + sinf(t) * L1, ky = hy + cosf(t) * L1;
        float fx = kx + sinf(t - bend) * L2, fy = ky + cosf(t - bend) * L2;
        spr.drawWedgeLine(cx, hy, kx, ky, 2.8f, 2.4f, c);
        spr.drawWedgeLine(kx, ky, fx, fy, 2.4f, 2.0f, c);
        float a = -amp * 0.7f * sinf(p);             // the arm swings against the leg
        float ex = sx + sinf(a) * LA, ey = sy + cosf(a) * LA;
        float a2 = a + 0.5f + 0.3f * amp;
        spr.drawWedgeLine(sx, sy, ex, ey, 2.0f, 1.8f, c);
        spr.drawWedgeLine(ex, ey, ex + sinf(a2) * LA, ey + cosf(a2) * LA, 1.8f, 1.6f, c);
        if (pass == 0) {
            spr.drawWedgeLine(cx, hy, sx, sy, 3.4f, 3.0f, col);
            spr.fillSmoothCircle((int)lroundf(sx + sinf(lean) * 8.5f), (int)lroundf(sy - cosf(lean) * 8.5f), 5, col);
        }
    }
}

// One line of the bottom block: icon, value, unit - centred as a whole.
static void stepsStat(int y, bool flame, const char *val, const char *unit) {
    spr.setFreeFont(&FreeSansBold9pt7b);
    int wv = spr.textWidth(val), wu = spr.textWidth(unit);
    int x0 = CX - (16 + wv + 6 + wu) / 2;
    int ix = x0 + 6;
    if (flame) {
        spr.fillSmoothCircle(ix, y + 2, 4, RGB565(255, 130, 0));
        spr.fillTriangle(ix - 4, y + 1, ix + 4, y + 1, ix + 1, y - 8, RGB565(255, 130, 0));
        spr.fillSmoothCircle(ix, y + 3, 2, RGB565(255, 220, 80));
    } else {
        spr.fillSmoothCircle(ix, y - 2, 4, C_CYAN);
        spr.fillTriangle(ix - 3, y, ix + 3, y, ix, y + 6, C_CYAN);
        spr.fillSmoothCircle(ix, y - 2, 1, RGB565(8, 26, 32));
    }
    text(val, x0 + 16 + wv / 2, y, &FreeSansBold9pt7b, TFT_WHITE);
    text(unit, x0 + 16 + wv + 6 + wu / 2, y, &FreeSansBold9pt7b, C_DIM);
}

static void stepsDraw(uint32_t now) {
    char buf[32];
    uint32_t dtMs = stepsLastDrawMs ? min<uint32_t>(now - stepsLastDrawMs, 100) : 0;
    stepsLastDrawMs = now;
    float dt = dtMs / 1000.0f;

    if (stepsBgOk) stepsBg.pushToSprite(&spr, 0, 0);
    else spr.fillSprite(TFT_BLACK);

    if (st.count != stepsLastCount) {                // a step was counted
        stepsLastCount = st.count;
        stepsFlashMs = now;
    }
    bool walking = st.pending >= STEP_CONFIRM_N && st.lastCandMs && now - st.lastCandMs < 1800;
    float frac = (float)st.count / STEP_GOAL;
    bool done = frac >= 1.0f;
    int style = done ? 1 : 0;
    uint32_t age = now - stepsFlashMs;
    float fl = (stepsFlashMs && age < 600) ? 1.0f - age / 600.0f : 0.0f;
    uint16_t accent = done ? RGB565(255, 200, 40) : C_CYAN;

    stepsDisp += (fminf(frac, 1.0f) - stepsDisp) * (1.0f - expf(-dt * 4.0f));            // values ease towards the real ones
    stepsCad += ((walking ? st.cad : 0.0f) - stepsCad) * (1.0f - expf(-dt * 5.0f));
    stepsAmp += ((walking ? 1.0f : 0.0f) - stepsAmp) * (1.0f - expf(-dt * 6.0f));
    if (stepsAmp > 0.05f) stepsPhase += dt * PI * fmaxf(stepsCad, 90.0f) / 60.0f;       // one stride cycle = two steps

    // rings: progress to the goal (outer), live cadence (inner)
    stepsArc(111, 6.0f, stepsDisp, style);
    if (stepsDisp > 0.004f) {
        float th = stepsDisp * 2.0f * PI;
        float hx = CX + sinf(th) * 111, hy = CY - cosf(th) * 111;
        stepsGlow(hx, hy, 15 + (int)(6 * fl), stepsGrad(style, (int)(stepsDisp * 256)), 130 + (int)(90 * fl));
        spr.fillSmoothCircle((int)lroundf(hx), (int)lroundf(hy), 3, TFT_WHITE);
    }
    float cf = fminf(stepsCad / 160.0f, 1.0f);
    stepsArc(94.5f, 3.2f, cf, 2);
    if (cf > 0.02f) {
        float th = cf * 2.0f * PI;
        spr.fillSmoothCircle((int)lroundf(CX + sinf(th) * 94.5f), (int)lroundf(CY - cosf(th) * 94.5f), 2, RGB565(255, 235, 170));
    }
    if (done) {                                      // sparks circling the full gold ring
        for (int k = 0; k < 3; k++) {
            float a = now / 700.0f + k * 2.094f;
            float sx = CX + sinf(a) * 111, sy = CY - cosf(a) * 111;
            stepsGlow(sx, sy, 10, TFT_WHITE, 150);
            spr.fillSmoothCircle((int)lroundf(sx), (int)lroundf(sy), 2, TFT_WHITE);
        }
    }

    // runner with the ground scrolling under it
    float gy = 87;
    for (int i = 0; i < 4; i++) {
        float x = CX + 28 - fmodf(stepsPhase * 7.0f + i * 14.0f, 56.0f);
        float b = 1.0f - fabsf(x - CX) / 28.0f;
        if (b <= 0) continue;
        spr.drawWedgeLine(x, gy, x + 6, gy, 0.8f, 0.8f, mix565(RGB565(20, 34, 40), accent, (int)(b * (40 + 70 * stepsAmp))));
    }
    stepsWalker(CX - 1, 64, stepsPhase, stepsAmp, accent);

    // the count
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)st.count);
    text(buf, CX, 109, &FreeSansBold24pt7b, mix565(TFT_WHITE, accent, (int)(fl * 170)));
    text("STEPS", CX, 143, &FreeSansBold9pt7b, accent);

    // distance and energy
    snprintf(buf, sizeof(buf), "%.2f", st.count * STEP_M / 1000.0f);
    stepsStat(166, false, buf, "km");
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)(st.count * STEP_KCAL));
    stepsStat(188, true, buf, "kcal");

    if (stepsNotice) {
        drawPanel();
        spr.drawSmoothRoundRect(36, 56, 18, 17, 168, 128, C_TEAL, C_PANEL);
        text("STEPS", CX, 80, &FreeSansBold12pt7b, C_TEAL);
        text("counts only while", CX, 108, &FreeSansBold9pt7b, TFT_WHITE);
        text("worn or carried", CX, 128, &FreeSansBold9pt7b, TFT_WHITE);
        text("CLICK = OK", CX, 160, &FreeSansBold9pt7b, C_TEAL);
    } else if (stepsConfirm) {
        drawPanel();
        spr.drawSmoothRoundRect(36, 56, 18, 17, 168, 128, C_RED, C_PANEL);
        text("RESET?", CX, 84, &FreeSansBold12pt7b, TFT_WHITE);
        text("step count to 0", CX, 106, &FreeSansBold9pt7b, C_DIM);
        text("CLICK = YES", CX, 128, &FreeSansBold9pt7b, C_RED);
        text("WAIT = NO", CX, 152, &FreeSansBold9pt7b, C_DIM);
    }
    spr.pushSprite(0, 0);
}

static void stepsExit() {
    stepsConfirm = false;
    stepsSave(millis(), true);
    if (stepsBgOk) {
        stepsBg.deleteSprite();
        stepsBgOk = false;
    }
}

// ======================= APP: FLIP TIMER (logic from flip_timer.cpp) =======================
// mode 0: the eyes look the way the board is tilted. mode 1: relative to the usual pose (the timer is held upright, so gravity must not count).
static void faceDraw(uint32_t now, int mode = 0, const char *caption = nullptr);   // the eyes, defined with the menu
static const uint32_t PRESETS_S[] = {60, 180, 300, 600, 900, 1800};
static const int PRESET_COUNT = sizeof(PRESETS_S) / sizeof(PRESETS_S[0]);

enum Face { FACE_UP, FACE_DOWN };
enum TimerState { T_IDLE, T_RUNNING, T_PAUSED, T_DONE };

static struct {
    Face face = FACE_UP, faceCandidate = FACE_UP;
    uint32_t candidateSince = 0;
    float fax = 0, fay = 1, faz = 0;                 // low-pass filtered acceleration
    int rotCurrent = 0, rotCandidate = 0;
    uint32_t rotCandidateMs = 0;
    TimerState state = T_IDLE;
    int presetIdx = 2;                               // 5 minutes
    int32_t remainingMs = 0;
    uint32_t lastTickMs = 0, lastImuMs = 0, hintUntil = 0;   // hintUntil: the preset is shown under the eyes until then
} tm;

// Returns true once, when the board goes from flipped back to normal.
static bool timerUpdateFlip() {
    tm.fax += (imuAcc[0] - tm.fax) * 0.3f;
    tm.fay += (imuAcc[1] - tm.fay) * 0.3f;
    tm.faz += (imuAcc[2] - tm.faz) * 0.3f;

    // the axis carrying most of gravity (Y or Z) decides the orientation; sideways (X) is ignored
    float v = (fabsf(tm.fay) > fabsf(tm.faz)) ? tm.fay : tm.faz;
    Face seen = tm.face;                             // otherwise keep the current state
    if (fabsf(v) > FACE_THRESHOLD_G && fabsf(v) > fabsf(tm.fax)) seen = (v > 0) ? FACE_UP : FACE_DOWN;

    uint32_t now = millis();
    if (seen != tm.faceCandidate) {
        tm.faceCandidate = seen;
        tm.candidateSince = now;
    }
    if (tm.faceCandidate != tm.face && now - tm.candidateSince >= FACE_DEBOUNCE_MS) {
        tm.face = tm.faceCandidate;
        return tm.face == FACE_UP;
    }
    return false;
}

// Keeps the picture upright: rotates the display to follow the board (0, 90, 180, 270 degrees).
static void timerUpdateRotation() {
    float m = fmaxf(fabsf(tm.fax), fabsf(tm.fay));
    if (m < 0.6f || m < fabsf(tm.faz)) return;       // lying flat or tilted: keep the last rotation

    int wanted;
    if (fabsf(tm.fay) >= fabsf(tm.fax)) wanted = (tm.fay > 0) ? 0 : 2;
    else wanted = (tm.fax > 0) ? ROT_SIDE_A : ROT_SIDE_B;
    wanted = (wanted + ROT_OFFSET) % 4;

    uint32_t now = millis();
    if (wanted != tm.rotCandidate) {
        tm.rotCandidate = wanted;
        tm.rotCandidateMs = now;
    }
    if (tm.rotCandidate != tm.rotCurrent && now - tm.rotCandidateMs >= ROTATE_DEBOUNCE_MS) {
        tm.rotCurrent = tm.rotCandidate;
        tft.setRotation(tm.rotCurrent);
    }
}

static void timerOnFlip() {
    switch (tm.state) {
        case T_IDLE:
            tm.remainingMs = (int32_t)(PRESETS_S[tm.presetIdx] * 1000UL);
            tm.lastTickMs = millis();
            tm.state = T_RUNNING;
            break;
        case T_RUNNING:
            tm.state = T_PAUSED;
            break;
        case T_PAUSED:
            tm.lastTickMs = millis();
            tm.state = T_RUNNING;
            break;
        case T_DONE:
            tm.state = T_IDLE;
            break;
    }
}

static void timerTick() {
    if (tm.state != T_RUNNING) return;
    uint32_t now = millis();
    tm.remainingMs -= (int32_t)(now - tm.lastTickMs);
    tm.lastTickMs = now;
    if (tm.remainingMs <= 0) {
        tm.remainingMs = 0;
        tm.state = T_DONE;
    }
}

// Short press: next preset while idle or finished, stop the timer while it is running or paused.
static void timerButton() {
    if (tm.state == T_IDLE || tm.state == T_DONE) {
        tm.state = T_IDLE;
        tm.presetIdx = (tm.presetIdx + 1) % PRESET_COUNT;
    } else {
        tm.state = T_IDLE;
    }
    tm.hintUntil = millis() + 2500;
}

static void timerInit() {
    tm.state = T_IDLE;
    tm.face = tm.faceCandidate = FACE_UP;
    tm.rotCurrent = tm.rotCandidate = 0;
    tm.lastImuMs = 0;
    tm.hintUntil = millis() + 2500;
}

static void timerUpdate(uint32_t now) {
    if (now - tm.lastImuMs >= 20) {
        tm.lastImuMs = now;
        if (timerUpdateFlip()) timerOnFlip();
        timerUpdateRotation();
    }
    timerTick();
}

static void timerDraw(uint32_t now) {
    if (tm.state == T_IDLE) {                        // waiting: show the eyes until the board is flipped
        char cap[16];
        snprintf(cap, sizeof(cap), "%u MIN", (unsigned)(PRESETS_S[tm.presetIdx] / 60));
        faceDraw(now, 1, now < tm.hintUntil ? cap : nullptr);
        return;
    }
    uint32_t totalMs = PRESETS_S[tm.presetIdx] * 1000UL;
    uint32_t shownMs = (tm.state == T_IDLE) ? totalMs : (uint32_t)tm.remainingMs;
    float frac = (tm.state == T_IDLE || tm.state == T_DONE) ? 1.0f : (float)shownMs / (float)totalMs;

    bool flashOn = (now / 500) % 2 == 0;
    uint16_t accent = C_GREEN;                                        // running: green
    if (tm.state == T_PAUSED) accent = RGB565(255, 190, 40);          // paused: amber
    if (tm.state == T_IDLE) accent = C_BLUE;                          // idle: blue
    if (tm.state == T_DONE) accent = flashOn ? C_RED : C_DARKRED;

    spr.fillSprite(TFT_BLACK);
    spr.drawSmoothArc(CX, CY, 117, 105, 0, 360, C_TRACK, TFT_BLACK);   // track
    drawArcFrom12(117, 105, frac, accent);

    uint32_t secs = (shownMs + 999) / 1000;
    char buf[16];
    snprintf(buf, sizeof(buf), "%02u:%02u", (unsigned)(secs / 60), (unsigned)(secs % 60));

    char total[16];
    bool selectable = (tm.state == T_IDLE || tm.state == T_DONE);
    snprintf(total, sizeof(total), selectable ? "CLICK: %u MIN" : "%u MIN", (unsigned)(PRESETS_S[tm.presetIdx] / 60));

    text(total, CX, CY - 48, &FreeSansBold9pt7b, C_DIM);
    text(buf, CX, CY - 2, &FreeSansBold24pt7b, (tm.state == T_PAUSED && !flashOn) ? C_DIM : TFT_WHITE);

    const char *label = "";
    switch (tm.state) {
        case T_IDLE:    label = "FLIP TO START"; break;
        case T_RUNNING: label = "RUNNING";       break;
        case T_PAUSED:  label = "PAUSED";        break;
        case T_DONE:    label = "TIME UP";       break;
    }
    text(label, CX, CY + 46, &FreeSansBold9pt7b, accent);
    spr.pushSprite(0, 0);
}

static void timerExit() {
    tm.state = T_IDLE;
    tm.rotCurrent = tm.rotCandidate = 0;
    tft.setRotation(0);
}

// ======================= APP: CM SCALE =======================
enum SlideState { SL_IDLE, SL_CALIB, SL_ARMED, SL_MEASURING, SL_DONE };

static SlideState slState = SL_IDLE;
static float slAx[SLIDE_MAX_N], slAy[SLIDE_MAX_N];   // bias-corrected acceleration (g)
static uint32_t slT[SLIDE_MAX_N];                    // timestamps (us)
static int slN = 0, slCalibN = 0, slStillN = 0;
static float slBx = 0, slBy = 0, slThr = 0.02f;      // sensor bias and motion threshold (g)
static float slFm = 0, slResultCm = -1;              // filtered acceleration magnitude, result
static float slPeak = 0;                             // strongest filtered acceleration during the movement
static int slOverN = 0;                              // consecutive samples above the start threshold
static uint32_t slCalibFromUs = 0;                   // calibration ignores samples before this (the board is still settling)
static double slSx = 0, slSy = 0, slSxx = 0, slSyy = 0;

static void slidePushSample(float x, float y, uint32_t t) {
    if (slState == SL_ARMED && slN >= SLIDE_PRE_N) {   // keep only the last PRE_N samples while waiting
        memmove(slAx, slAx + 1, (slN - 1) * sizeof(float));
        memmove(slAy, slAy + 1, (slN - 1) * sizeof(float));
        memmove(slT,  slT + 1,  (slN - 1) * sizeof(uint32_t));
        slN = SLIDE_PRE_N - 1;
    }
    if (slN < SLIDE_MAX_N) {
        slAx[slN] = x;
        slAy[slN] = y;
        slT[slN] = t;
        slN++;
    }
}

static void slideStart() {
    slState = SL_CALIB;
    slCalibN = 0;
    slSx = slSy = slSxx = slSyy = 0;
    slN = 0;
    slFm = 0;
    slStillN = 0;
    slResultCm = -1;
    slPeak = 0;
    slOverN = 0;
    slCalibFromUs = micros() + 700000;               // let the board settle after the click before measuring its bias
}

// Not a real slide (noise, a bump, a drift): forget it and wait for the next one.
static void slideRearm() {
    slState = SL_ARMED;
    slN = 0;
    slFm = 0;
    slStillN = 0;
    slOverN = 0;
    slPeak = 0;
}

static void slideFinish(int trail) {
    int e = slN - trail;                             // end of the motion (trailing rest dropped)
    if (slPeak < SLIDE_MIN_PEAK_G || e < SLIDE_MIN_N) {
        slideRearm();
        return;
    }
    if (e < 10) {
        slResultCm = 0;
    } else {
        // pass 1: velocity at the end of the motion. It should be zero, so the error is drift.
        double vx = 0, vy = 0;
        for (int i = 1; i < e; i++) {
            double dt = (slT[i] - slT[i - 1]) * 1e-6;
            vx += slAx[i] * GRAV * dt;
            vy += slAy[i] * GRAV * dt;
        }
        double T = (slT[e - 1] - slT[0]) * 1e-6;
        double vex = vx, vey = vy;

        // pass 2: remove the drift linearly over time and integrate to position
        double px = 0, py = 0;
        vx = vy = 0;
        for (int i = 1; i < e; i++) {
            double dt = (slT[i] - slT[i - 1]) * 1e-6;
            double tt = (slT[i] - slT[0]) * 1e-6;
            vx += slAx[i] * GRAV * dt;
            vy += slAy[i] * GRAV * dt;
            px += (vx - vex * tt / T) * dt;
            py += (vy - vey * tt / T) * dt;
        }
        slResultCm = (float)(sqrt(px * px + py * py) * 100.0);
    }
    if (slResultCm > SLIDE_MAX_CM) {                 // drifting sensor, not a real slide
        slideRearm();
        return;
    }
    slState = SL_DONE;
    char msg[40];
    snprintf(msg, sizeof(msg), "LENGTH %.1f cm", slResultCm);
    remoteSend(msg);
}

static void slideStop() {
    if (slState == SL_MEASURING) slideFinish(slStillN);
    else if (slState == SL_CALIB || slState == SL_ARMED) slState = SL_IDLE;
}

static void scaleSample(const float a[3], const float g[3], float dt, uint32_t tUs) {
    (void)g; (void)dt;
    float ax = a[0], ay = a[1];

    if (slState == SL_CALIB) {
        if ((int32_t)(tUs - slCalibFromUs) < 0) return;   // still settling after the click
        slSx += ax; slSy += ay; slSxx += ax * ax; slSyy += ay * ay;
        if (++slCalibN >= SLIDE_CALIB_N) {
            float mx = slSx / slCalibN, my = slSy / slCalibN;
            float var = (slSxx / slCalibN - mx * mx) + (slSyy / slCalibN - my * my);
            float sigma = sqrtf(var > 0 ? var : 0);
            if (sigma > 0.02f) {                     // board was moving: start the calibration again
                slCalibN = 0;
                slSx = slSy = slSxx = slSyy = 0;
                return;
            }
            slBx = mx;
            slBy = my;
            slThr = fmaxf(0.015f, 4.0f * sigma);
            slN = 0;
            slFm = 0;
            slState = SL_ARMED;
            remoteSend("SLIDE NOW");
        }
        return;
    }
    if (slState != SL_ARMED && slState != SL_MEASURING) return;

    float dx = ax - slBx, dy = ay - slBy;
    slFm += (hypotf(dx, dy) - slFm) * 0.3f;
    slidePushSample(dx, dy, tUs);

    if (slState == SL_ARMED) {
        if (slFm > fmaxf(slThr * 1.5f, SLIDE_START_G)) slOverN++; else slOverN = 0;
        if (slOverN >= 3) {
            slState = SL_MEASURING;
            slStillN = 0;
            slPeak = 0;
        }
    } else {
        if (slFm > slPeak) slPeak = slFm;
        if (slFm < slThr) slStillN++; else slStillN = 0;
        if (slN >= SLIDE_MAX_N) slideRearm();        // never came to rest: drift, not a slide
        else if (slStillN >= SLIDE_STILL_N) slideFinish(slStillN);
    }
}

static void scaleInit() {
    slState = SL_IDLE;
    tm.hintUntil = millis() + 2500;
}

// Short press: start a measurement, stop one that is running, or measure again after a result.
static void scaleButton() {
    if (slState == SL_IDLE || slState == SL_DONE) slideStart();
    else slideStop();
}

static void drawSlide(uint32_t now) {
    const uint16_t ORANGE = RGB565(255, 170, 0);
    char buf[16];
    auto spin = [&](uint16_t col) {                  // ring with a bright segment turning round it
        int a = (now / 3) % 360, b = a + 100;
        spr.drawSmoothArc(CX, CY, 117, 109, 0, 360, C_TRACK, TFT_BLACK);
        if (b <= 360) {
            spr.drawSmoothArc(CX, CY, 117, 109, a, b, col, TFT_BLACK, true);
        } else {
            spr.drawSmoothArc(CX, CY, 117, 109, a, 360, col, TFT_BLACK, true);
            spr.drawSmoothArc(CX, CY, 117, 109, 0, b - 360, col, TFT_BLACK, true);
        }
    };
    auto ruler = [&](int y, int shift) {             // little ruler, its ticks scroll while measuring
        spr.fillSmoothRoundRect(CX - 62, y - 9, 124, 18, 4, ORANGE, TFT_BLACK);
        for (int k = 0; k < 8; k++) spr.drawFastVLine(CX - 56 + (k * 14 + shift) % 112, y - 9, 8, TFT_BLACK);
    };
    bool busy = (slState == SL_CALIB || slState == SL_ARMED || slState == SL_MEASURING);

    switch (slState) {
        case SL_CALIB:
            spin(ORANGE);
            text("KEEP", CX, CY - 22, &FreeSansBold18pt7b, TFT_WHITE);
            text("STILL", CX, CY + 14, &FreeSansBold18pt7b, TFT_WHITE);
            break;
        case SL_ARMED: {
            spr.drawSmoothArc(CX, CY, 117, 109, 0, 360, C_GREEN, TFT_BLACK);
            text("SLIDE", CX, CY - 16, &FreeSansBold18pt7b, C_GREEN);
            int off = (int)(6 * sinf(now / 180.0f));
            spr.fillTriangle(CX + 16 + off, CY + 20, CX + 16 + off, CY + 36, CX + 32 + off, CY + 28, ORANGE);
            spr.fillTriangle(CX - 16 - off, CY + 20, CX - 16 - off, CY + 36, CX - 32 - off, CY + 28, ORANGE);
            break;
        }
        case SL_MEASURING:
            spin(C_CYAN);
            text("MOVING", CX, CY - 16, &FreeSansBold18pt7b, C_CYAN);
            ruler(CY + 30, (int)(now / 6) % 112);
            break;
        default:                                     // SL_DONE (SL_IDLE shows the eyes)
            spr.drawSmoothArc(CX, CY, 117, 109, 0, 360, ORANGE, TFT_BLACK);
            snprintf(buf, sizeof(buf), "%.1f", slResultCm);
            text(buf, CX, CY - 18, &FreeSansBold24pt7b, TFT_WHITE);
            text("cm", CX, CY + 22, &FreeSansBold18pt7b, ORANGE);
            ruler(CY + 56, 0);
            break;
    }
    text(busy ? "CLICK: STOP" : "CLICK: AGAIN", CX, 210, &FreeSansBold9pt7b, C_DIM);
}

static void scaleDraw(uint32_t now) {
    if (slState == SL_IDLE) {                        // waiting: the eyes, until the button is pressed
        faceDraw(now, 0, now < tm.hintUntil ? "CLICK TO START" : nullptr);
        return;
    }
    spr.fillSprite(TFT_BLACK);
    drawSlide(now);
    spr.pushSprite(0, 0);
}

static void scaleExit() {
    slState = SL_IDLE;
}

// ======================= MENU =======================
struct Tile { int16_t cx, cy; const char *label; Screen app; uint16_t color; };
static const Tile TILES[4] = {
    { 76,  76, "MAZE",    APP_BALL,  C_AMBER},
    {164,  76, "STEPS", APP_STEPS, C_TEAL},
    { 76, 164, "TIMER", APP_TIMER, C_BLUE},
    {164, 164, "SCALE", APP_SCALE, C_PURPLE},
};
static int menuSel = 0;

// Home screen is a face with blinking eyes. Any BOOT press shows the tile menu, which returns to the face after 10 s idle.
static bool faceMode = true;
static uint32_t menuLastInputMs = 0;
#define MENU_IDLE_MS 10000

static void menuInit() {
    faceMode = true;
}

static void menuWake(uint32_t now) {
    faceMode = false;
    menuLastInputMs = now;
}

static void menuButton() {
    uint32_t now = millis();
    if (faceMode) menuWake(now);                     // first press only wakes the menu
    else {
        menuSel = (menuSel + 1) % 4;
        menuLastInputMs = now;
    }
}

static void menuUpdate(uint32_t now) {
    if (!faceMode && now - menuLastInputMs > MENU_IDLE_MS) faceMode = true;
}

// Half-ellipse eye with a flat top and a rounded bottom, anti-aliased on the left and right edges.
static void eyeDraw(int cx, int top, int rx, int ry, uint16_t col) {
    for (int dy = 0; dy < ry; dy++) {
        float t = (dy + 0.5f) / ry;
        float hw = rx * sqrtf(1.0f - t * t);
        int ix = (int)hw;
        spr.drawFastHLine(cx - ix, top + dy, 2 * ix + 1, col);
        uint16_t edge = spr.alphaBlend((uint8_t)((hw - ix) * 255), col, TFT_BLACK);
        spr.drawPixel(cx - ix - 1, top + dy, edge);
        spr.drawPixel(cx + ix + 1, top + dy, edge);
    }
}

static void faceDraw(uint32_t now, int mode, const char *caption) {
    static uint32_t lastCall = 0;
    static float baseX = 0, baseY = 0;
    static uint32_t blinkStart = 0, nextBlink = 2000, glanceUntil = 0, nextGlance = 3000;
    static bool blinking = false, followUp = false;
    static uint8_t extraBlink = 0;
    static float lookX = 0, lookY = 0, glanceX = 0, glanceY = 0;

    float open = 1.0f;                               // 1 = eyes fully open, 0 = closed
    if (!blinking && now >= nextBlink) {
        blinking = true;
        blinkStart = now;
        if (!followUp) extraBlink = (random(4) == 0) ? 1 : 0;   // sometimes blink twice
        followUp = false;
    }
    if (blinking) {
        uint32_t e = now - blinkStart;
        if (e < 70) open = 1.0f - e / 70.0f;
        else if (e < 110) open = 0.0f;
        else if (e < 190) open = (e - 110) / 80.0f;
        else {
            blinking = false;
            if (extraBlink) {
                extraBlink = 0;
                followUp = true;
                nextBlink = now + 130;
            } else {
                nextBlink = now + random(2200, 5500);
            }
        }
    }

    if (now >= nextGlance) {                         // now and then glance to a random side
        glanceX = (float)random(-12, 13);
        glanceY = (float)random(-5, 6);
        glanceUntil = now + 700;
        nextGlance = now + random(3000, 7000);
    }
    if (now > glanceUntil) glanceX = glanceY = 0;

    float tx = TILT_X_SIGN * imuAcc[TILT_X_AXIS];    // eyes look the way the ball would roll
    float ty = TILT_Y_SIGN * imuAcc[TILT_Y_AXIS];
    (void)mode;                                      // tilt is measured against the pose the board has been held in for a while,
    if (now - lastCall > 300) {                      // so it works flat on the desk and standing upright alike
        baseX = tx;
        baseY = ty;
    }
    baseX += (tx - baseX) * 0.01f;
    baseY += (ty - baseY) * 0.01f;
    tx = (tx - baseX) * 4.0f;
    ty = (ty - baseY) * 4.0f;
    lastCall = now;
    if (fabsf(tx) < 0.05f) tx = 0;
    if (fabsf(ty) < 0.05f) ty = 0;
    lookX += (constrain(tx, -1.0f, 1.0f) * 32.0f + glanceX - lookX) * 0.5f;
    lookY += (constrain(ty, -1.0f, 1.0f) * 14.0f + glanceY - lookY) * 0.5f;

    spr.fillSprite(TFT_BLACK);                       // the white printed case is the helmet, so the screen is all face

    const int RX = 36, RY = 32, GAP = 40;
    int ex = (int)lroundf(lookX), ey = CY + 2 + (int)lroundf(lookY);
    float sx = constrain(lookX / 32.0f, -1.0f, 1.0f), sy = constrain(lookY / 14.0f, -1.0f, 1.0f);
    uint16_t col = RGB565(255, 170, 0);
    for (int side = -1; side <= 1; side += 2) {      // the eye on the side being looked at gets bigger, the far one smaller;
        float f = 1.0f + 0.3f * sx * side;           // looking up opens the eyes wide, looking down makes them sleepy
        float v = (sy < 0 ? 1.0f - 0.25f * sy : 1.0f - 0.4f * sy);
        int ry = max(2, (int)lroundf(RY * open * f * v));
        int rx = (int)lroundf(RX * (1.0f + 0.15f * sx * side));
        eyeDraw(CX + side * GAP + ex, ey - ry / 2, rx, ry, col);
    }

    if (caption) text(caption, CX, 192, &FreeSansBold9pt7b, C_DIM);
    if (bleConnected) spr.fillSmoothCircle(CX, 214, 3, C_BLUE, TFT_BLACK);
    spr.pushSprite(0, 0);
}

static void menuDrawIcon(int i, int ix, int iy, uint16_t fill) {
    uint16_t c = TILES[i].color;
    switch (i) {
        case 0:                                      // maze: two rings with gaps, and the ball in the middle
            spr.drawSmoothArc(ix, iy, 19, 15, 40, 320, c, fill, true);
            spr.drawSmoothArc(ix, iy, 10, 6, 220, 360, c, fill, true);
            spr.drawSmoothArc(ix, iy, 10, 6, 0, 140, c, fill, true);
            spr.fillSmoothCircle(ix, iy, 3, TFT_WHITE, fill);
            break;
        case 1:                                      // two footprints
            spr.fillSmoothRoundRect(ix - 15, iy - 2, 11, 17, 5, c, fill);
            spr.fillSmoothCircle(ix - 10, iy - 9, 4, c, fill);
            spr.fillSmoothRoundRect(ix + 4, iy - 12, 11, 17, 5, c, fill);
            spr.fillSmoothCircle(ix + 9, iy - 19, 4, c, fill);
            break;
        case 2:                                      // clock
            spr.drawSmoothArc(ix, iy, 15, 11, 0, 360, c, fill, false);
            spr.drawWideLine(ix, iy, ix, iy - 8, 2.5f, TFT_WHITE, fill);
            spr.drawWideLine(ix, iy, ix + 6, iy, 2.5f, TFT_WHITE, fill);
            break;
        case 3:                                      // ruler
            spr.fillSmoothRoundRect(ix - 18, iy - 7, 36, 15, 3, c, fill);
            for (int k = -2; k <= 2; k++) spr.drawFastVLine(ix + k * 7, iy - 7, (k % 2) ? 5 : 8, fill);
            break;
    }
}

static void menuDraw(uint32_t now) {
    if (faceMode) {
        faceDraw(now);
        return;
    }
    spr.fillSprite(TFT_BLACK);
    for (int i = 0; i < 4; i++) {
        const Tile &t = TILES[i];
        uint16_t fill = (menuSel == i) ? C_TILE_HI : C_TILE;
        spr.fillSmoothRoundRect(t.cx - TILE_HALF, t.cy - TILE_HALF, 2 * TILE_HALF, 2 * TILE_HALF, TILE_RADIUS, fill, TFT_BLACK);
        if (menuSel == i) {
            spr.drawSmoothRoundRect(t.cx - TILE_HALF - 3, t.cy - TILE_HALF - 3, TILE_RADIUS + 3, TILE_RADIUS + 1,
                                    2 * TILE_HALF + 6, 2 * TILE_HALF + 6, t.color, TFT_BLACK);
        }
        menuDrawIcon(i, t.cx, t.cy - 10, fill);
        text(t.label, t.cx, t.cy + 27, &FreeSansBold9pt7b, TFT_WHITE);
    }
    text("CLICK=NEXT", CX, 17, &FreeSansBold9pt7b, C_DIM);
    text("HOLD=OPEN", CX, 223, &FreeSansBold9pt7b, C_DIM);
    if (!imuAddr) text("NO IMU", CX, CY, &FreeSansBold9pt7b, C_RED);
    if (bleConnected) spr.fillSmoothCircle(CX + 62, 17, 4, C_BLUE, TFT_BLACK);   // phone connected
    spr.pushSprite(0, 0);
}

// ======================= state machine =======================
struct AppVt {
    void (*init)();
    void (*sample)(const float a[3], const float g[3], float dt, uint32_t tUs);   // every IMU sample
    void (*button)();                                // BOOT short press (a long press always changes screen)
    void (*update)(uint32_t now);
    void (*draw)(uint32_t now);
    void (*exit)();
    uint16_t frameMs;
};

static const AppVt APPS[] = {
    {menuInit,  nullptr,     menuButton,  menuUpdate,   menuDraw,  nullptr,   25},
    {mzInit,    mzSample,    mzButton,    nullptr,      mzDraw,    mzExit,    25},
    {stepsInit, nullptr,     stepsButton, stepsUpdate,  stepsDraw, stepsExit, 33},
    {timerInit, nullptr,     timerButton, timerUpdate,  timerDraw, timerExit, 50},
    {scaleInit, scaleSample, scaleButton, nullptr,  scaleDraw, scaleExit, 33},
};

static Screen screen = MENU;
static const AppVt *cur = nullptr;
static uint32_t lastDrawMs = 0;

static const char *const SCREEN_NAMES[] = {"MENU", "MAZE", "STEPS", "TIMER", "SCALE"};

static void switchTo(Screen s) {
    if (cur && cur->exit) cur->exit();
    screen = s;
    cur = &APPS[s];
    cur->init();
    lastDrawMs = 0;
    char msg[24];
    snprintf(msg, sizeof(msg), "SCREEN %s", SCREEN_NAMES[s]);
    remoteSend(msg);
}

static void remoteHandle(char *cmd) {
    for (char *p = cmd; *p; p++) *p = tolower((unsigned char)*p);
    char *e = cmd + strlen(cmd);
    while (e > cmd && (e[-1] == '\n' || e[-1] == '\r' || e[-1] == ' ')) *--e = 0;
    while (*cmd == ' ') cmd++;

    Screen target = screen;
    if (!strcmp(cmd, "0") || !strcmp(cmd, "menu") || !strcmp(cmd, "back")) target = MENU;
    else if (!strcmp(cmd, "1") || !strcmp(cmd, "maze") || !strcmp(cmd, "ball")) target = APP_BALL;
    else if (!strcmp(cmd, "2") || !strcmp(cmd, "steps")) target = APP_STEPS;
    else if (!strcmp(cmd, "3") || !strcmp(cmd, "timer")) target = APP_TIMER;
    else if (!strcmp(cmd, "4") || !strcmp(cmd, "scale")) target = APP_SCALE;
    else if (!strcmp(cmd, "click") || !strcmp(cmd, "c")) {
        if (cur->button) cur->button();
        return;
    } else if (!strcmp(cmd, "status")) {
        char msg[48];
        snprintf(msg, sizeof(msg), "STEPS %lu", (unsigned long)st.count);
        remoteSend(msg);
        return;
    } else {
        remoteSend("1 maze, 2 steps, 3 timer, 4 scale, 0 menu, click, status");
        return;
    }
    if (target != screen) {
        for (int i = 0; i < 4; i++) if (TILES[i].app == target) menuSel = i;
        switchTo(target);
    }
}

// ======================= Wi-Fi web page =======================
static const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head>
<meta name=viewport content="width=device-width,initial-scale=1"><title>MenuBot</title>
<style>body{font-family:sans-serif;background:#111;color:#eee;text-align:center;margin:0;padding:12px}
h2{margin:8px}button{width:46%;margin:5px 1%;padding:24px 0;font-size:20px;border:0;border-radius:14px;color:#fff}
.a{background:#d98c00}.t{background:#17a896}.b{background:#2e7fe0}.p{background:#8a4fd8}.g{background:#555}
#s{margin:16px;font-size:20px;white-space:pre-line;color:#9f9}</style></head><body><h2>MenuBot</h2>
<button class=a onclick="c('1')">MAZE</button><button class=t onclick="c('2')">STEPS</button>
<button class=b onclick="c('3')">TIMER</button><button class=p onclick="c('4')">SCALE</button>
<button class=g onclick="c('0')">MENU</button><button class=g onclick="c('click')">CLICK</button>
<div id=s>...</div>
<script>function u(){fetch('/status').then(r=>r.text()).then(t=>{document.getElementById('s').textContent=t})}
function c(x){fetch('/cmd?c='+x).then(u)}setInterval(u,1000);u()</script></body></html>)HTML";

static WebServer web(80);

static void webInit() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    web.on("/", HTTP_GET, []() { web.send_P(200, "text/html", PAGE); });
    web.on("/cmd", HTTP_GET, []() {
        char c[24];
        strlcpy(c, web.arg("c").c_str(), sizeof(c));
        remoteHandle(c);
        web.send(200, "text/plain", "ok");
    });
    web.on("/status", HTTP_GET, []() {
        char m[128];
        snprintf(m, sizeof(m), "%s\nSTEPS %lu\n%s", SCREEN_NAMES[screen], (unsigned long)st.count, lastMsg);
        web.send(200, "text/plain", m);
    });
    web.begin();
    Serial.printf("Wi-Fi " WIFI_SSID " at http://%s\n", WiFi.softAPIP().toString().c_str());
}

void setup() {
    Serial.begin(115200);
    pinMode(BOOT_PIN, INPUT_PULLUP);

    tft.init();
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);
    spr.setColorDepth(16);
    if (!spr.createSprite(SCR, SCR)) {
        Serial.println("Sprite alloc failed");
        while (true) delay(1000);
    }
    spr.setTextDatum(MC_DATUM);

    Wire.begin(I2C_SDA, I2C_SCL, 400000);
    if (imuInit()) Serial.printf("QMI8658 found at 0x%02X\n", imuAddr);
    else Serial.println("QMI8658 not found");

    prefs.begin("menuapp", false);
    st.count = st.savedCount = prefs.getUInt("steps", 0);   // the count survives power-off
    Serial.printf("steps restored: %lu\n", (unsigned long)st.count);

    webInit();
    bleInit();
    switchTo(MENU);
}

void loop() {
    static uint32_t lastSampleUs = 0, lastImuRetryMs = 0;
    uint32_t now = millis();
    uint32_t nowUs = micros();

    if (!imuAddr) {                                  // retry until the sensor answers
        if (now - lastImuRetryMs >= 500) {
            lastImuRetryMs = now;
            if (imuInit()) Serial.println("QMI8658 ready");
        }
    } else if (nowUs - lastSampleUs >= IMU_SAMPLE_US) {
        float dt = (nowUs - lastSampleUs) * 1e-6f;
        lastSampleUs = nowUs;
        if (dt > 0.1f) dt = IMU_SAMPLE_US * 1e-6f;   // first call / long gap
        float a[3], g[3];
        if (imuReadAll(a, g)) {
            for (int i = 0; i < 3; i++) imuAcc[i] = a[i];
            stepsOnSample(a, now);
            if (cur->sample) cur->sample(a, g, dt, nowUs);
        }
    }

    BtnEvent b = buttonPoll(now);
    if (b == BTN_LONG) {
        if (screen != MENU) switchTo(MENU);
        else if (faceMode) menuWake(now);
        else switchTo(TILES[menuSel].app);
    }
    else if (b == BTN_SHORT && cur->button) cur->button();

    if (bleCmdReady) {
        char cmd[sizeof(bleCmd)];
        memcpy(cmd, bleCmd, sizeof(cmd));
        bleCmdReady = false;
        remoteHandle(cmd);
    }

    if (cur->update) cur->update(now);
    web.handleClient();

    if (now - lastDrawMs >= cur->frameMs) {
        lastDrawMs = now;
        cur->draw(now);
    }

    stepsSave(now, false);
}
