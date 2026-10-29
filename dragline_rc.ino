/*
 * RC Dragline Crane – Web control for 6 motors
 * ESP32 + 3× L298N motor drivers. Arduino IDE (ESP32 core 3.x).
 *
 * ═══════════════════════════════════════════════════════════════════════
 * ARDUINO IDE BUILD SETTINGS (required with PS3 Bluetooth)
 * ═══════════════════════════════════════════════════════════════════════
 *
 *   WiFi AP + Bluetooth Classic needs ~1.7 MB flash — larger than the
 *   default 1.2 MB app partition. In Tools menu set:
 *
 *     Board:            ESP32 Dev Module
 *     Flash Size:       4MB (32Mb)  — match your board
 *     Partition Scheme: Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)
 *                       — or — Huge APP (3MB No OTA/1MB SPIFFS)
 *     Core Debug Level: None  (optional, saves a little space)
 *
 *   If upload still fails, try Optimize: Size (-Os) under Tools.
 *
 * ═══════════════════════════════════════════════════════════════════════
 * MOTOR INDEX TABLE
 * ═══════════════════════════════════════════════════════════════════════
 *
 *   Index  Function           Direction Labels
 *   ─────  ─────────────────  ──────────────────
 *     0    Tilt Winch         Tilt (1) / Dump (-1)
 *     1    Bucket Winch       Up (1) / Down (-1)
 *     2    Boom Winch         Up (1) / Down (-1)
 *     3    Rotation           Right (1) / Left (-1)
 *     4    Left Track         Fwd (1) / Rev (-1)
 *     5    Right Track        Fwd (1) / Rev (-1)
 *
 * ═══════════════════════════════════════════════════════════════════════
 * WiFi CONFIGURATION
 * ═══════════════════════════════════════════════════════════════════════
 *
 *   Mode:      WiFi AP (Access Point, no external router needed)
 *   SSID:      "DraglineCrane"
 *   Password:  "crane1234"
 *   IP:        192.168.4.1
 *
 * ═══════════════════════════════════════════════════════════════════════
 * HTTP ENDPOINTS
 * ═══════════════════════════════════════════════════════════════════════
 *
 *   Method  Path      Parameters                       Response
 *   ──────  ────────  ───────────────────────────────  ────────────
 *   GET     /         (none)                           200, HTML page
 *   GET     /status   (none)                           200, JSON (ps3, motors, uptime, idle)
 *   GET     /motor    m=0..5, dir=-1/0/1, speed=0..255 200, "ok" / 403 if PS3 active
 *   GET     /allstop  (none)                           200, "ok"
 *   GET     /ps3release (none)                         200, "ok" — release PS3, unlock phone
 *   GET     * (other) (none)                           302 → "/"
 *
 *   /motor parameter notes:
 *     - m (required): motor index 0–5
 *     - dir (optional): direction -1, 0, or 1
 *     - speed (optional): PWM duty 0–255 (ignored for tracks)
 *     - If only speed is sent (no dir), speed updates and current direction is re-applied
 *     - Returns 403 "controller active" while a PS3 controller is connected
 *
 * ═══════════════════════════════════════════════════════════════════════
 * PS3 BLUETOOTH CONTROLLER (DualShock 3)
 * ═══════════════════════════════════════════════════════════════════════
 *
 *   Pairing (one-time):
 *     1. Upload sketch — OLED shows "Pair PS3 to:" + BT MAC for 10 s at boot
 *        (Do NOT hardcode ps3BtMac — it is filled automatically after BT init.)
 *     2. Plug PS3 into PC/Mac via USB
 *     3. Write the displayed ESP32 MAC into the controller (SixaxisPairTool / sixaxispairer)
 *     4. Unplug USB, press PS button — connects within ~10 s
 *
 *   "BT INIT FAIL" at boot means Bluetooth failed to start (not a pairing problem).
 *   On ESP32 core 3.x use the patched library in libraries/PS3_Controller_Host/.
 *
 *   Exclusive mode: when PS3 is connected, /motor web commands are blocked.
 *   Phone web page still loads; /allstop always works as emergency backup.
 *
 *   Button map (hold to run, release to stop):
 *     Left stick Y     Left track (4) fwd (up) / rev (down)
 *     Right stick Y    Right track (5) fwd (up) / rev (down)
 *                      (both up = straight, opposite = tank turn)
 *     D-pad Up/Down    Bucket winch (1) up/down
 *     D-pad Left/Right Boom winch (2) up/down
 *     Triangle/Cross   Tilt winch (0) tilt/dump
 *     L1 / R1          Rotation (3) left/right
 *     PS button        All-stop
 *
 *   PS3 winch speeds (Boom/Bucket/Tilt/Rotation) are set via PS3_MOTOR_SPEED
 *   constants in the sketch (default ~200). Tracks still ramp to full PWM.
 *   Web UI speed sliders are unchanged.
 *
 *   Idle sleep: 5 min with no deliberate PS3 input → allStop + BT disconnect.
 *   Press PS to reconnect. Short rumble pulse confirms link.
 *
 *   Library: PS3 Controller Host — patched copy in project libraries/ folder
 *             (ESP32 Arduino core 3.x). Remove duplicate from Library Manager.
 *
 * ═══════════════════════════════════════════════════════════════════════
 * PIN-MAPPING TABLE
 * ═══════════════════════════════════════════════════════════════════════
 *
 *   Motor            Board  Ch   EN   IN1  IN2
 *   ───────────────  ─────  ──   ───  ───  ───
 *   0: Tilt Winch      1     A   23   22   21
 *   1: Bucket Winch    1     B   19   18    5
 *   2: Boom Winch      2     A   17   16    4
 *   3: Rotation        2     B   25   26   27
 *   4: Left Track      3     A   14   12   13
 *   5: Right Track     3     B   33   32   15
 *
 *   Status / Display   Board   GPIO   Function
 *   ─────────────────  ──────  ─────  ───────────────────────────────
 *   Action LED         P0         0   Heartbeat when idle, solid when active
 *   OLED I2C SDA       TX         1   SSD1306 128×32 data
 *   Fault LED          P2         2   Fast blink if WiFi AP fails
 *   OLED I2C SCL       RX         3   SSD1306 128×32 clock
 *   OLED turns off after 10 min with no motors running and no user input.
 *
 *   Do NOT use SD0–SD3 (GPIO 7–10) — connected to internal flash.
 *   Do NOT use P34, P35, SVP (GPIO 34–36) — input-only, no LED/I2C.
 *
 *   Total motor GPIO used: 18 (all unique)
 *   Avoided: GPIO 6–11 (flash), GPIO 34–39 (input-only)
 *   Strapping pins used: GPIO 12, 15 (both safe as outputs)
 *   Note: GPIO 0 is a boot strapping pin — wire action LED active-LOW
 *         (3.3V → resistor → LED → GPIO 0) or ensure LED is off at boot
 *
 * ═══════════════════════════════════════════════════════════════════════
 * POWER DISTRIBUTION
 * ═══════════════════════════════════════════════════════════════════════
 *
 *   • Single 12V power source (battery or PSU)
 *   • 12V+ connects in parallel to all 3× L298N boards (shared 12V rail)
 *   • GND connects in parallel to all 3× L298N boards (shared GND bus)
 *   • ESP32 powered from L298N Board 1's onboard 5V regulator:
 *       - Board 1: 5V jumper ENABLED → 5V pin connects to ESP32 VIN
 *       - Board 2: 5V jumper DISABLED (no regulator output)
 *       - Board 3: 5V jumper DISABLED (no regulator output)
 *   • COMMON GROUND BUS: ESP32 GND, all 3× L298N GND terminals, and
 *     power supply GND MUST all be connected together
 *   • Tracks draw the most current (two motors). Soft-start: kick to PWM 120,
 *     then ramp to 255 over ~150 ms to limit inrush brownout resets.
 *   • If ESP still reboots when tracks run: use a sturdier 12 V supply (≥5 A
 *     recommended for two gearmotors), add 470–1000 µF on ESP 5 V/GND, and
 *     keep motor power and ESP grounds tied to the same bus.
 *
 * ═══════════════════════════════════════════════════════════════════════
 * WIRING SCHEMATIC
 * ═══════════════════════════════════════════════════════════════════════
 *
 *                          ┌─────────────────┐
 *                          │   12V BATTERY   │
 *                          │   (+)     (-)   │
 *                          └──┬─────────┬────┘
 *                             │         │
 *              ┌──────────────┼─────────┼──────────────────────┐
 *              │          12V RAIL       │     GND BUS          │
 *              │              │          │                      │
 *     ┌────┬──┼──────────────┼──────────┼───────────┐          │
 *     │    │  │              │          │           │          │
 *     │    ▼  ▼              ▼          ▼           │          │
 *     │ ┌──────────┐  ┌──────────┐  ┌──────────┐   │          │
 *     │ │ L298N #1 │  │ L298N #2 │  │ L298N #3 │   │          │
 *     │ │ 12V  GND │  │ 12V  GND │  │ 12V  GND │   │          │
 *     │ │ 5V=ON    │  │ 5V=OFF   │  │ 5V=OFF   │   │          │
 *     │ │          │  │          │  │          │   │          │
 *     │ │ ChA  ChB │  │ ChA  ChB │  │ ChA  ChB │   │          │
 *     │ │  │    │  │  │  │    │  │  │  │    │  │   │          │
 *     │ └──┼────┼──┘  └──┼────┼──┘  └──┼────┼──┘   │          │
 *     │    │    │         │    │         │    │      │          │
 *     │    ▼    ▼         ▼    ▼         ▼    ▼      │          │
 *     │  ┌──┐ ┌──┐     ┌──┐ ┌──┐     ┌──┐ ┌──┐     │          │
 *     │  │M0│ │M1│     │M2│ │M3│     │M4│ │M5│     │          │
 *     │  │Ti│ │Bk│     │B │ │Ro│     │LT│ │RT│     │          │
 *     │  └──┘ └──┘     └──┘ └──┘     └──┘ └──┘     │          │
 *     │  Tilt  Bucket   Boom  Rot     LTrk  RTrk    │          │
 *     │                                              │          │
 *     │          5V                                  │          │
 *     │    ┌─────┘ (from Board 1 regulator)          │          │
 *     │    │                                         │          │
 *     │    ▼                                         │          │
 *     │ ┌─────────────────────────────────┐          │          │
 *     │ │            ESP32                │          │          │
 *     │ │  VIN ←── 5V from L298N #1      │          │          │
 *     │ │  GND ←── Common GND Bus ───────┼──────────┼──────────┘
 *     │ │                                 │          │
 *     │ │  GPIO23 (EN0)  ──► L298N #1 ENA │          │
 *     │ │  GPIO22 (IN1_0)──► L298N #1 IN1 │          │
 *     │ │  GPIO21 (IN2_0)──► L298N #1 IN2 │          │
 *     │ │  GPIO19 (EN1)  ──► L298N #1 ENB │          │
 *     │ │  GPIO18 (IN1_1)──► L298N #1 IN3 │          │
 *     │ │  GPIO5  (IN2_1)──► L298N #1 IN4 │          │
 *     │ │                                 │          │
 *     │ │  GPIO17 (EN2)  ──► L298N #2 ENA │          │
 *     │ │  GPIO16 (IN1_2)──► L298N #2 IN1 │          │
 *     │ │  GPIO4  (IN2_2)──► L298N #2 IN2 │          │
 *     │ │  GPIO25 (EN3)  ──► L298N #2 ENB │          │
 *     │ │  GPIO26 (IN1_3)──► L298N #2 IN3 │          │
 *     │ │  GPIO27 (IN2_3)──► L298N #2 IN4 │          │
 *     │ │                                 │          │
 *     │ │  GPIO14 (EN4)  ──► L298N #3 ENA │          │
 *     │ │  GPIO12 (IN1_4)──► L298N #3 IN1 │          │
 *     │ │  GPIO13 (IN2_4)──► L298N #3 IN2 │          │
 *     │ │  GPIO33 (EN5)  ──► L298N #3 ENB │          │
 *     │ │  GPIO32 (IN1_5)──► L298N #3 IN3 │          │
 *     │ │  GPIO15 (IN2_5)──► L298N #3 IN4 │          │
 *     │ │                                 │          │
 *     │ │  P0  (GPIO0)  ──► Action LED      │          │
 *     │ │  TX  (GPIO1)  ──► OLED SDA        │          │
 *     │ │  P2  (GPIO2)  ──► Fault LED       │          │
 *     │ │  RX  (GPIO3)  ──► OLED SCL        │          │
 *     │ └─────────────────────────────────┘          │
 *     │                                              │
 *     └──────────────────────────────────────────────┘
 *
 *   Legend: M0=Tilt Winch, M1=Bucket Winch, M2=Boom Winch,
 *           M3=Rotation, M4=Left Track, M5=Right Track
 *           Ti=Tilt, Bk=Bucket, B=Boom, Ro=Rotation,
 *           LT=Left Track, RT=Right Track
 *
 * ═══════════════════════════════════════════════════════════════════════
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Ps3Controller.h>
#include <esp_system.h>
#if __has_include("esp32-hal-alloc-bt-classic-mem.h")
#include "esp32-hal-alloc-bt-classic-mem.h"
#endif

// ─── WiFi & Network Configuration ──────────────────────────────────────────

const char* ap_ssid     = "DraglineCrane";
const char* ap_password = "crane1234";

WebServer  server(80);
DNSServer  dnsServer;

// ─── Motor Pin Configuration ────────────────────────────────────────────────

struct MotorPins {
  uint8_t en;   // PWM enable pin (speed)
  uint8_t in1;  // Direction pin 1
  uint8_t in2;  // Direction pin 2
};

#define NUM_MOTORS 6

const MotorPins MOTORS[NUM_MOTORS] = {
  // { EN,  IN1, IN2 }       Motor           L298N Board/Channel
  {  23,  22,  21  },  // 0: Tilt Winch     – Board 1, Channel A
  {  19,  18,   5  },  // 1: Bucket Winch   – Board 1, Channel B
  {  17,  16,   4  },  // 2: Boom Winch     – Board 2, Channel A
  {  25,  26,  27  },  // 3: Rotation       – Board 2, Channel B
  {  14,  12,  13  },  // 4: Left Track     – Board 3, Channel A
  {  33,  32,  15  },  // 5: Right Track    – Board 3, Channel B
};

// ─── PWM Constants ──────────────────────────────────────────────────────────

#define PWM_FREQ       5000
#define PWM_RESOLUTION 8
#define PWM_MAX        255
#define TRACK_PWM_TARGET PWM_MAX  // tracks ramp up to full power (255)
#define TRACK_PWM_START       120  // immediate kick to overcome gear friction (10 RPM motors)
#define TRACK_RAMP_STEP        25  // ramp START→255 in ~150 ms after kick
#define TRACK_RAMP_INTERVAL_MS 20
#define TRACK_STOP_HOLD_MS     80  // ignore brief stick zero — prevents pulsing at deadzone
#define TRACK_DIR_DEBOUNCE_MS 100

// PS3-only PWM per motor (0-255). Web sliders are unaffected.
#define PS3_SPEED_TILT       210
#define PS3_SPEED_BUCKET     200
#define PS3_SPEED_BOOM       200
#define PS3_SPEED_ROTATION   255

const uint8_t PS3_MOTOR_SPEED[NUM_MOTORS] = {
  PS3_SPEED_TILT, PS3_SPEED_BUCKET, PS3_SPEED_BOOM, PS3_SPEED_ROTATION,
  TRACK_PWM_TARGET, TRACK_PWM_TARGET  // tracks unchanged — use ramp logic
};

// ─── Status LED & OLED ──────────────────────────────────────────────────────

#define PIN_LED_ACTION  0   // board silkscreen: P0
#define PIN_LED_FAULT   2   // board silkscreen: P2
#define OLED_SDA        1   // board silkscreen: TX
#define OLED_SCL        3   // board silkscreen: RX
#define OLED_WIDTH      128
#define OLED_HEIGHT     32
#define OLED_ADDR       0x3C
#define DISPLAY_REFRESH_MS 250
#define LINE1_ROTATE_MS    5000
#define LED_HEARTBEAT_MS   5000
#define LED_PULSE_MS       150
#define ICON_W             16
#define ICON_H             16
#define STICK_DEADZONE       50   // engage: stick must exceed this to start tracks
#define STICK_RELEASE_ZONE   28   // release: stick must fall below this to stop (hysteresis)
#define PS3_IDLE_STICK_ZONE  90   // stick drift below this does not reset idle timer
#define PS3_POLL_MS          20   // limit PS3 input polling rate (Hz) to reduce BT/CPU load
#define PS3_DISPLAY_MIN_MS  120   // min interval between OLED updates during PS3 control
#define PS3_IDLE_MS      (5UL * 60 * 1000)  // auto sleep after 5 min PS3 inactivity
#define PS3_IDLE_WARN_MS    (30UL * 1000)   // show idle countdown after 30 s quiet
#define PS3_STALE_MS        (30UL * 1000)   // no BT packets for 30 s → treat as disconnected
#define PS3_SLEEP_OLED_MS   (15UL * 1000)   // full-screen sleep message, then phone-style OLED
#define OLED_IDLE_MS     (10UL * 60 * 1000) // blank OLED after 10 min inactivity
#define BOOT_MAC_DISPLAY_MS 10000

#ifndef SSD1306_DISPLAYOFF
#define SSD1306_DISPLAYOFF 0xAE
#endif
#ifndef SSD1306_DISPLAYON
#define SSD1306_DISPLAYON 0xAF
#endif

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

bool     oledReady = false;
bool     oledPanelOn = true;
uint32_t lastOledActivityMs = 0;
uint32_t lastDisplayUpdate = 0;
bool     ps3Ready = false;
bool     ps3Connected = false;
bool     ps3WasConnected = false;
char     ps3BtMac[18] = "";
uint32_t bootMacDisplayEnd = 0;
uint32_t lastPs3PollMs = 0;
uint32_t lastPs3DisplayMs = 0;
bool     ps3TrackActive[2] = { false, false };  // per-stick hysteresis state (L, R)
bool     ps3SleepMode = false;
uint32_t ps3SleepSinceMs = 0;
uint32_t bootTimeMs = 0;
uint32_t lastPs3ActivityMs = 0;
uint32_t lastPs3PacketMs = 0;
uint8_t  trackPwmLevel[2] = { 0, 0 };
uint32_t lastTrackRampMs = 0;
uint32_t lastTrackStickMs[2] = { 0, 0 };
uint32_t trackStopRequestMs[2] = { 0, 0 };

const char* MOTOR_NAMES[NUM_MOTORS] = {
  "Tilt", "Bucket", "Boom", "Rot", "LTrk", "RTrk"
};

// ─── Runtime State ──────────────────────────────────────────────────────────

int8_t  motorDir[NUM_MOTORS]   = { 0, 0, 0, 0, 0, 0 };
uint8_t motorSpeed[NUM_MOTORS] = { 200, 200, 200, 200, PWM_MAX, PWM_MAX };

// ─── Status Helpers ─────────────────────────────────────────────────────────

enum IconDir { ICON_NONE, ICON_UP, ICON_DOWN, ICON_LEFT, ICON_RIGHT };

// 16×16 direction icons (2 bytes per row, MSB = left)
const uint8_t iconUp[] PROGMEM = {
  0x00,0x00, 0x01,0x80, 0x03,0xC0, 0x07,0xE0, 0x0F,0xF0, 0x1F,0xF8, 0x3F,0xFC,
  0x01,0x80, 0x01,0x80, 0x01,0x80, 0x01,0x80, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00
};
const uint8_t iconDown[] PROGMEM = {
  0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x01,0x80, 0x01,0x80, 0x01,0x80,
  0x01,0x80, 0x3F,0xFC, 0x1F,0xF8, 0x0F,0xF0, 0x07,0xE0, 0x03,0xC0, 0x01,0x80, 0x00,0x00
};
const uint8_t iconLeft[] PROGMEM = {
  0x00,0x00, 0x00,0x80, 0x01,0xC0, 0x03,0xE0, 0x07,0xF0, 0x0F,0xF8, 0x1F,0xFC,
  0x3F,0xFE, 0x1F,0xFC, 0x0F,0xF8, 0x07,0xF0, 0x03,0xE0, 0x01,0xC0, 0x00,0x80, 0x00,0x00
};
const uint8_t iconRight[] PROGMEM = {
  0x00,0x00, 0x01,0x00, 0x03,0x80, 0x07,0xC0, 0x0F,0xE0, 0x1F,0xF0, 0x3F,0xF8,
  0x7F,0xFC, 0x3F,0xF8, 0x1F,0xF0, 0x0F,0xE0, 0x07,0xC0, 0x03,0x80, 0x01,0x00, 0x00,0x00
};

const char* motorActionLabel(int motor, int8_t dir) {
  if (dir == 0) return nullptr;
  switch (motor) {
    case 0: return (dir > 0) ? "TILT" : "DUMP";
    case 1: return (dir > 0) ? "UP" : "DN";
    case 2: return (dir > 0) ? "UP" : "DN";
    case 3: return (dir > 0) ? "R" : "L";
    case 4:
    case 5: return (dir > 0) ? "FWD" : "REV";
    default: return "?";
  }
}

IconDir iconForMotor(int motor, int8_t dir) {
  if (dir == 0) return ICON_NONE;
  switch (motor) {
    case 0:
    case 1:
    case 2:
    case 4:
    case 5: return (dir > 0) ? ICON_UP : ICON_DOWN;
    case 3: return (dir > 0) ? ICON_RIGHT : ICON_LEFT;
    default: return ICON_NONE;
  }
}

void drawDirectionIcon(IconDir icon, int16_t x, int16_t y) {
  const uint8_t* bmp = nullptr;
  switch (icon) {
    case ICON_UP:    bmp = iconUp;    break;
    case ICON_DOWN:  bmp = iconDown;  break;
    case ICON_LEFT:  bmp = iconLeft;  break;
    case ICON_RIGHT: bmp = iconRight; break;
    default: return;
  }
  display.drawBitmap(x, y, bmp, ICON_W, ICON_H, SSD1306_WHITE);
}

bool anyMotorActive() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motorDir[i] != 0) return true;
  }
  return false;
}

bool anyWinchActive() {
  for (int i = 0; i <= 3; i++) {
    if (motorDir[i] != 0) return true;
  }
  return false;
}

int activeWinchSpeedPct() {
  if (ps3Connected && anyWinchActive()) {
    return 100;
  }
  for (int i = 0; i <= 3; i++) {
    if (motorDir[i] != 0) {
      return (motorSpeed[i] * 100) / PWM_MAX;
    }
  }
  return 0;
}

void getActiveMotorState(int& activeCount, int& lastActive) {
  activeCount = 0;
  lastActive  = -1;
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motorDir[i] != 0) {
      activeCount++;
      lastActive = i;
    }
  }
}

void tickActionLed() {
  if (anyMotorActive()) {
    digitalWrite(PIN_LED_ACTION, HIGH);
    return;
  }
  bool pulse = (millis() % LED_HEARTBEAT_MS) < LED_PULSE_MS;
  digitalWrite(PIN_LED_ACTION, pulse ? HIGH : LOW);
}

void printCenteredSize2(const char* text, int16_t y) {
  int16_t  x1, y1;
  uint16_t w, h;
  display.setTextSize(2);
  display.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  display.setCursor((OLED_WIDTH - (int16_t)w) / 2, y);
  display.print(text);
}

const char* ps3BatteryLabel() {
  if (!ps3Connected) return "";
  switch (Ps3.data.status.battery) {
    case ps3_status_battery_full:      return "FULL";
    case ps3_status_battery_high:      return "HIGH";
    case ps3_status_battery_low:       return "LOW";
    case ps3_status_battery_dying:      return "DYING";
    case ps3_status_battery_charging:  return "CHG";
    default:                           return "?";
  }
}

bool ps3BatteryLow() {
  if (!ps3Connected) return false;
  uint8_t bat = Ps3.data.status.battery;
  return bat == ps3_status_battery_low || bat == ps3_status_battery_dying;
}

void printUptimeShort() {
  uint32_t sec = (millis() - bootTimeMs) / 1000;
  uint32_t min = sec / 60;
  sec %= 60;
  display.print(F("Up "));
  if (min >= 60) {
    display.print(min / 60);
    display.print(F("h"));
    display.print((min % 60) / 10);
    display.print((min % 60) % 10);
  } else {
    if (min < 10) display.print(F("0"));
    display.print(min);
    display.print(F(":"));
    if (sec < 10) display.print(F("0"));
    display.print(sec);
  }
}

void printPs3IdleCountdown() {
  uint32_t idleMs = millis() - lastPs3ActivityMs;
  if (anyMotorActive() || idleMs < PS3_IDLE_WARN_MS) {
    display.print(F("WEB LOCKED"));
    return;
  }
  uint32_t remain = (PS3_IDLE_MS > idleMs) ? (PS3_IDLE_MS - idleMs) / 1000 : 0;
  uint32_t min = remain / 60;
  uint32_t sec = remain % 60;
  display.print(F("Idle "));
  display.print(min);
  display.print(F(":"));
  if (sec < 10) display.print(F("0"));
  display.print(sec);
}

void printWebLine1(int slot) {
  display.setCursor(0, 0);
  if (slot == 0) {
    display.print(F("ONLINE "));
    display.print(WiFi.softAPIP());
  } else if (slot == 1) {
    display.print(F("WiFi "));
    display.print(ap_ssid);
  } else if (slot == 2) {
    display.print(F("Clients: "));
    display.print(WiFi.softAPgetStationNum());
  } else if (slot == 3 && ps3SleepMode) {
    display.print(F("PS3 off Press PS"));
  } else if (slot == 3) {
    printUptimeShort();
  } else if (slot == 4 && anyWinchActive()) {
    display.print(F("Spd "));
    display.print(activeWinchSpeedPct());
    display.print(F("%"));
  }
}

void printPs3Line1(int slot) {
  display.setCursor(0, 0);
  if (slot == 0) {
    display.print(F("PS3 LINKED"));
  } else if (slot == 1) {
    if (ps3BatteryLow() && (millis() / 500) % 2 == 0) {
      display.print(F("BAT LOW!"));
    } else {
      display.print(F("Bat: "));
      display.print(ps3BatteryLabel());
      if (Ps3.data.status.charging) {
        display.print(F(" +CHG"));
      }
    }
  } else if (slot == 2) {
    printPs3IdleCountdown();
  } else if (slot == 3) {
    printUptimeShort();
  } else if (slot == 4 && anyWinchActive()) {
    display.print(F("Spd "));
    display.print(activeWinchSpeedPct());
    display.print(F("%"));
  }
}

void showBootBtMacScreen() {
  if (!oledReady) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  if (ps3Ready) {
    display.println(F("Pair PS3 to:"));
    display.println(ps3BtMac);
  } else {
    display.println(F("BT INIT FAIL"));
    display.println(F("Web only"));
  }
  display.display();
  lastDisplayUpdate = millis();
}

bool initDisplay() {
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
      return false;
    }
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.display();
  return true;
}

void oledSendCommand(uint8_t cmd) {
  Wire.beginTransmission(OLED_ADDR);
  Wire.write((uint8_t)0x00);
  Wire.write(cmd);
  Wire.endTransmission();
}

void oledSetPanelPower(bool on) {
  if (!oledReady || on == oledPanelOn) {
    return;
  }
  oledPanelOn = on;
  oledSendCommand(on ? SSD1306_DISPLAYON : SSD1306_DISPLAYOFF);
  if (on) {
    lastDisplayUpdate = 0;
    updateDisplay();
  }
}

void bumpOledActivity() {
  lastOledActivityMs = millis();
  if (!oledPanelOn) {
    oledSetPanelPower(true);
  }
}

void tickOledSleep() {
  if (!oledReady || !oledPanelOn) {
    return;
  }
  if (millis() < bootMacDisplayEnd && !ps3Connected) {
    return;
  }
  if (anyMotorActive()) {
    lastOledActivityMs = millis();
    return;
  }
  if (millis() - lastOledActivityMs >= OLED_IDLE_MS) {
    oledSetPanelPower(false);
  }
}

void updateDisplay() {
  if (!oledReady || !oledPanelOn) return;

  if (millis() < bootMacDisplayEnd && !ps3Connected) {
    showBootBtMacScreen();
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (ps3SleepMode && !ps3Connected &&
      (millis() - ps3SleepSinceMs < PS3_SLEEP_OLED_MS)) {
    display.setCursor(0, 0);
    display.println(F("PS3 SLEEP"));
    display.setCursor(0, 10);
    display.println(F("Press PS btn"));
    printCenteredSize2("READY", 14);
    display.display();
    lastDisplayUpdate = millis();
    return;
  }

  // Line 1: PS3 info when controller linked; otherwise WiFi/network rotation
  int line1Screens = anyWinchActive() ? 5 : 4;
  int line1Slot    = (int)((millis() / LINE1_ROTATE_MS) % line1Screens);

  if (ps3Connected) {
    printPs3Line1(line1Slot);
  } else {
    printWebLine1(line1Slot);
  }

  // Line 2: large status with direction icon
  int activeCount = 0;
  int lastActive  = -1;
  getActiveMotorState(activeCount, lastActive);

  display.setTextSize(2);
  const int16_t line2Y = 14;

  if (activeCount == 0) {
    printCenteredSize2("READY", line2Y);
  } else if (activeCount == 1) {
    char statusBuf[20];
    snprintf(statusBuf, sizeof(statusBuf), "%s %s",
             MOTOR_NAMES[lastActive],
             motorActionLabel(lastActive, motorDir[lastActive]));
    drawDirectionIcon(iconForMotor(lastActive, motorDir[lastActive]), 0, line2Y);
    display.setCursor(ICON_W + 2, line2Y);
    display.print(statusBuf);
  } else if (motorDir[4] != 0 && motorDir[5] != 0 &&
             motorDir[4] == motorDir[5] && activeCount == 2) {
    char statusBuf[16];
    snprintf(statusBuf, sizeof(statusBuf), "TRK %s",
             motorActionLabel(4, motorDir[4]));
    drawDirectionIcon(iconForMotor(4, motorDir[4]), 0, line2Y);
    display.setCursor(ICON_W + 2, line2Y);
    display.print(statusBuf);
  } else {
    char statusBuf[12];
    snprintf(statusBuf, sizeof(statusBuf), "MULTI (%d)", activeCount);
    printCenteredSize2(statusBuf, line2Y);
  }

  display.display();
  lastDisplayUpdate = millis();
}

void refreshStatus() {
  updateDisplay();
}

void refreshStatusThrottled() {
  uint32_t now = millis();
  if (now - lastPs3DisplayMs >= PS3_DISPLAY_MIN_MS) {
    lastPs3DisplayMs = now;
    updateDisplay();
  }
}

// ─── Motor Control ──────────────────────────────────────────────────────────

void setMotor(int motor, int8_t dir, uint8_t speed, bool ps3Drive = false) {
  if (motor < 0 || motor >= NUM_MOTORS) return;

  const MotorPins& p = MOTORS[motor];
  bool isTrack = (motor == 4 || motor == 5);
  int  trackIdx = isTrack ? (motor - 4) : 0;
  int8_t prevDir = motorDir[motor];

  if (isTrack && dir != prevDir) {
    if (dir == 0) {
      trackPwmLevel[trackIdx] = 0;
    } else if (prevDir != 0 && dir != prevDir) {
      // Direction reversal — soft-start again
      trackPwmLevel[trackIdx] = 0;
    } else if (prevDir == 0) {
      // Fresh engage — kick to START immediately so slow motors actually turn
      trackPwmLevel[trackIdx] = TRACK_PWM_START;
    }
  }

  motorDir[motor] = dir;

  if (dir != prevDir) {
    bumpOledActivity();
  }

  if (isTrack) {
    motorSpeed[motor] = TRACK_PWM_TARGET;
  } else if (ps3Drive) {
    motorSpeed[motor] = PS3_MOTOR_SPEED[motor];
  } else {
    motorSpeed[motor] = constrain(speed, 0, PWM_MAX);
  }

  if (dir == 1) {
    digitalWrite(p.in1, HIGH);
    digitalWrite(p.in2, LOW);
  } else if (dir == -1) {
    digitalWrite(p.in1, LOW);
    digitalWrite(p.in2, HIGH);
  } else {
    digitalWrite(p.in1, LOW);
    digitalWrite(p.in2, LOW);
    if (isTrack) {
      trackPwmLevel[trackIdx] = 0;
    }
    ledcWrite(p.en, 0);
    return;
  }

  if (isTrack) {
    ledcWrite(p.en, trackPwmLevel[trackIdx]);
    return;
  }

  uint8_t pwm = ps3Drive ? PS3_MOTOR_SPEED[motor] : motorSpeed[motor];
  ledcWrite(p.en, pwm);
}

void tickTrackRamp() {
  uint32_t now = millis();
  if (now - lastTrackRampMs < TRACK_RAMP_INTERVAL_MS) {
    return;
  }
  lastTrackRampMs = now;

  for (int i = 0; i < 2; i++) {
    int m = 4 + i;
    if (motorDir[m] == 0) {
      continue;
    }
    if (trackPwmLevel[i] >= TRACK_PWM_TARGET) {
      continue;
    }
    uint8_t next = trackPwmLevel[i] + TRACK_RAMP_STEP;
    if (next > TRACK_PWM_TARGET) {
      next = TRACK_PWM_TARGET;
    }
    trackPwmLevel[i] = next;
    ledcWrite(MOTORS[m].en, trackPwmLevel[i]);
  }
}

void allStop() {
  bumpOledActivity();
  for (int i = 0; i < NUM_MOTORS; i++) {
    setMotor(i, 0, motorSpeed[i]);
  }
  refreshStatus();
}

// ─── PS3 Bluetooth Controller ───────────────────────────────────────────────

int stickAxisFiltered(int raw, bool wasActive) {
  int limit = wasActive ? STICK_RELEASE_ZONE : STICK_DEADZONE;
  if (abs(raw) <= limit) return 0;
  return raw;
}

void resetPs3TrackState() {
  ps3TrackActive[0] = false;
  ps3TrackActive[1] = false;
}

// Left stick Y drives the left track, right stick Y drives the right track.
// Stick up (negative Y) = forward, stick down = reverse.
void computeTrackDirs(int rawLy, int rawRy, int8_t& leftDir, int8_t& rightDir) {
  const int raw[2] = { rawLy, rawRy };
  int8_t dir[2] = { 0, 0 };

  for (int i = 0; i < 2; i++) {
    int y = stickAxisFiltered(raw[i], ps3TrackActive[i]);
    ps3TrackActive[i] = (y != 0);
    if (y < 0)      dir[i] = 1;
    else if (y > 0) dir[i] = -1;
  }

  leftDir  = dir[0];
  rightDir = dir[1];
}

bool ps3HasMeaningfulInput() {
  if (Ps3.data.button.ps || Ps3.data.button.up || Ps3.data.button.down ||
      Ps3.data.button.left || Ps3.data.button.right ||
      Ps3.data.button.triangle || Ps3.data.button.cross ||
      Ps3.data.button.l1 || Ps3.data.button.r1) {
    return true;
  }
  int ly = Ps3.data.analog.stick.ly;
  int ry = Ps3.data.analog.stick.ry;
  return abs(ly) > STICK_DEADZONE || abs(ry) > STICK_DEADZONE;
}

bool ps3ResetsIdleTimer() {
  if (Ps3.data.button.ps || Ps3.data.button.up || Ps3.data.button.down ||
      Ps3.data.button.left || Ps3.data.button.right ||
      Ps3.data.button.triangle || Ps3.data.button.cross ||
      Ps3.data.button.l1 || Ps3.data.button.r1) {
    return true;
  }
  // Ignore small stick drift — only deliberate stick movement resets idle countdown
  int ly = Ps3.data.analog.stick.ly;
  int ry = Ps3.data.analog.stick.ry;
  return abs(ly) > PS3_IDLE_STICK_ZONE || abs(ry) > PS3_IDLE_STICK_ZONE;
}

bool isWebLocked() {
  return ps3Connected && !ps3SleepMode;
}

void pollPs3Input() {
  if (!ps3Ready || !Ps3.isConnected()) return;

  if (ps3SleepMode) {
    tryWakePs3FromSleep();
    return;
  }

  if (!ps3Connected) return;

  if (ps3ResetsIdleTimer()) {
    lastPs3ActivityMs = millis();
    bumpOledActivity();
  }

  if (Ps3.data.button.ps) {
    allStop();
    return;
  }

  int8_t dirs[NUM_MOTORS] = { 0, 0, 0, 0, 0, 0 };

  computeTrackDirs(Ps3.data.analog.stick.ly, Ps3.data.analog.stick.ry, dirs[4], dirs[5]);

  // Per-track filtering, so each stick is debounced independently
  uint32_t now = millis();
  for (int i = 0; i < 2; i++) {
    int m = 4 + i;
    bool stickChange = (dirs[m] != motorDir[m]);

    // Debounce direction changes (ignore brief stick wobble)
    if (stickChange && dirs[m] != 0) {
      if (now - lastTrackStickMs[i] < TRACK_DIR_DEBOUNCE_MS) {
        dirs[m] = motorDir[m];
      } else {
        lastTrackStickMs[i] = now;
      }
    }

    // Hold track running through brief stick-center gaps (prevents ramp reset pulsing)
    if (dirs[m] == 0 && motorDir[m] != 0) {
      if (trackStopRequestMs[i] == 0) {
        trackStopRequestMs[i] = now;
      }
      if (now - trackStopRequestMs[i] < TRACK_STOP_HOLD_MS) {
        dirs[m] = motorDir[m];
      }
    } else {
      trackStopRequestMs[i] = 0;
      if (stickChange && dirs[m] == 0) {
        lastTrackStickMs[i] = now;
      }
    }
  }

  if (Ps3.data.button.up)    dirs[1] = 1;
  if (Ps3.data.button.down)  dirs[1] = -1;
  if (Ps3.data.button.left)  dirs[2] = 1;
  if (Ps3.data.button.right) dirs[2] = -1;

  if (Ps3.data.button.triangle) dirs[0] = 1;
  if (Ps3.data.button.cross)    dirs[0] = -1;

  if (Ps3.data.button.l1) dirs[3] = -1;
  if (Ps3.data.button.r1) dirs[3] = 1;

  bool changed = false;
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motorDir[i] != dirs[i]) {
      setMotor(i, dirs[i], 0, true);
      changed = true;
    }
  }

  if (changed) {
    refreshStatusThrottled();
  }
}

void tickPs3Connection() {
  bool linkUp = ps3Ready && Ps3.isConnected();

  // ps3SleepMode keeps phone unlocked even if BT stack still reports connected
  if (ps3SleepMode) {
    ps3Connected = false;
  } else {
    ps3Connected = linkUp;
  }

  if (!linkUp && ps3WasConnected) {
    resetPs3TrackState();
    if (!ps3SleepMode) {
      WiFi.setTxPower(WIFI_POWER_19_5dBm);
      allStop();
    }
  }
  if (linkUp && ps3SleepMode) {
    ps3SleepMode = false;
    ps3Connected = true;
    lastPs3ActivityMs = millis();
    lastPs3PacketMs = millis();
    lastPs3PollMs = 0;
    lastPs3DisplayMs = 0;
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    refreshStatus();
    Serial.println(F("[DraglineCrane] PS3 reconnected."));
  } else if (linkUp && !ps3WasConnected && !ps3SleepMode) {
    ps3SleepMode = false;
    lastPs3ActivityMs = millis();
    lastPs3PollMs = 0;
    lastPs3DisplayMs = 0;
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (motorDir[i] != 0) {
        setMotor(i, motorDir[i], 0, true);
      }
    }
    refreshStatus();
  }
  ps3WasConnected = linkUp;
}

void onPs3Data() {
  lastPs3PacketMs = millis();
  // Do not auto-wake on every BT packet — wake is handled in pollPs3Input / onPs3Connect
}

void onPs3Connect() {
  ps3SleepMode = false;
  ps3Connected = true;
  bumpOledActivity();
  lastPs3ActivityMs = millis();
  lastPs3PacketMs = millis();
  Ps3.setPlayer(1);
  Ps3.setRumble(75.0f, 350);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  refreshStatus();
  Serial.println(F("[DraglineCrane] PS3 connected."));
}

void onPs3Disconnect() {
  resetPs3TrackState();
  // Keep ps3SleepMode — idle-sleep disconnect must not clear it
  ps3Connected = false;
  ps3WasConnected = false;
  lastPs3PacketMs = 0;
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  allStop();
  refreshStatus();
  Serial.println(F("[DraglineCrane] PS3 disconnected."));
}

void attachPs3Callbacks() {
  Ps3.attach(onPs3Data);
  Ps3.attachOnConnect(onPs3Connect);
  Ps3.attachOnDisconnect(onPs3Disconnect);
}

void clearPs3SleepForPhone() {
  if (!ps3SleepMode || ps3Connected) {
    return;
  }
  ps3SleepMode = false;
  refreshStatus();
}

void forcePs3Sleep() {
  allStop();
  resetPs3TrackState();
  if (ps3Ready && Ps3.isConnected()) {
    Ps3.setRumble(0.0f, 0);
    Ps3.setPlayer(0);
    delay(30);
    Ps3.disconnect();
  }
  ps3Connected = false;
  ps3WasConnected = false;
  ps3SleepMode = true;
  ps3SleepSinceMs = millis();
  lastPs3PacketMs = 0;
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  if (ps3Ready) {
    Ps3.requestRearm();
  }
  refreshStatus();
  Serial.println(F("[DraglineCrane] PS3 idle sleep — BT disconnected, phone unlocked."));
  bumpOledActivity();
}

bool tryWakePs3FromSleep() {
  if (!ps3SleepMode || !ps3Ready || !Ps3.isConnected()) {
    return false;
  }
  if (!ps3HasMeaningfulInput()) {
    return false;
  }
  ps3SleepMode = false;
  ps3Connected = true;
  lastPs3ActivityMs = millis();
  lastPs3PacketMs = millis();
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  refreshStatus();
  Serial.println(F("[DraglineCrane] PS3 wake (user input)."));
  return true;
}

void tickPs3StaleCheck() {
  if (!ps3Ready || ps3SleepMode) return;
  if (!Ps3.isConnected() || lastPs3PacketMs == 0) return;
  if (millis() - lastPs3PacketMs >= PS3_STALE_MS) {
    Serial.println(F("[DraglineCrane] PS3 link stale — releasing."));
    forcePs3Sleep();
  }
}

void tickPs3IdleSleep() {
  if (!ps3Connected || ps3SleepMode) return;
  uint32_t idleMs = millis() - lastPs3ActivityMs;
  if (idleMs >= PS3_IDLE_MS) {
    Serial.print(F("[DraglineCrane] Idle timeout ("));
    Serial.print(idleMs / 1000);
    Serial.println(F(" s) — sleeping PS3."));
    forcePs3Sleep();
  }
}

// ─── WiFi AP Initialization ─────────────────────────────────────────────────

void initWiFiAP() {
  WiFi.mode(WIFI_AP);

  bool apStarted = false;
  for (int attempt = 0; attempt < 3; attempt++) {
    if (WiFi.softAP(ap_ssid, ap_password)) {
      apStarted = true;
      break;
    }
    delay(1000);
  }

  if (!apStarted) {
    // Fault indicator: infinite blink on GPIO2
    pinMode(PIN_LED_FAULT, OUTPUT);
    while (true) {
      digitalWrite(PIN_LED_FAULT, HIGH);
      delay(200);
      digitalWrite(PIN_LED_FAULT, LOW);
      delay(200);
    }
  }

  // Required when WiFi AP and Bluetooth Classic run together (ESP32 core 3.x)
  WiFi.setSleep(true);

  // Start DNS server — wildcard redirects all queries to AP IP
  dnsServer.start(53, "*", IPAddress(192, 168, 4, 1));
}

// ─── PS3 Bluetooth init ─────────────────────────────────────────────────────

bool initPs3() {
  ps3Ready = Ps3.begin();
  if (ps3Ready) {
    attachPs3Callbacks();
    String mac = Ps3.getAddress();
    mac.toCharArray(ps3BtMac, sizeof(ps3BtMac));
    Serial.print(F("[DraglineCrane] BT MAC for pairing: "));
    Serial.println(ps3BtMac);
  } else {
    ps3BtMac[0] = '\0';
    Serial.println(F("[DraglineCrane] BT init failed — web control only."));
  }
  return ps3Ready;
}

// ─── HTML UI ────────────────────────────────────────────────────────────────

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>Dragline Crane</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
:root{--bg:#0a0e17;--card:#141c2b;--border:#2a3548;--text:#e8edf5;--muted:#8b9bb4;--up:#22c55e;--down:#ef4444;--stop:#64748b;--rot:#f59e0b;--trk:#3b82f6}
body{font-family:system-ui,-apple-system,sans-serif;background:var(--bg);color:var(--text);padding:12px;padding-bottom:24px;max-width:640px;margin:0 auto}
.hdr{display:flex;align-items:center;justify-content:space-between;margin-bottom:10px}
.hdr h1{font-size:1.15rem;font-weight:700;letter-spacing:.04em}
.pill{font-size:.72rem;padding:4px 10px;border-radius:999px;background:var(--card);border:1px solid var(--border);color:var(--muted)}
.pill.on{color:#4ade80;border-color:#166534}
.pill.ps3{color:#fbbf24;border-color:#92400e}
.estop{display:block;width:100%;min-height:52px;margin-bottom:10px;border:none;border-radius:14px;background:linear-gradient(180deg,#dc2626,#991b1b);color:#fff;font-size:1.05rem;font-weight:800;letter-spacing:.08em;cursor:pointer;box-shadow:0 4px 16px rgba(239,68,68,.35)}
.estop:active{transform:scale(.98)}
.release{display:none;width:100%;min-height:40px;margin-bottom:10px;border:1px solid var(--border);border-radius:12px;background:var(--card);color:var(--text);font-size:.82rem;font-weight:600;cursor:pointer}
.release.show{display:block}
.release:active{background:#1e293b}
.status{background:var(--card);border:1px solid var(--border);border-radius:12px;padding:10px 12px;margin-bottom:14px;font-size:.82rem;color:var(--muted);min-height:38px}
.status.warn{color:#fbbf24;border-color:#92400e;background:#1c1408}
.status.lock{color:#fca5a5;border-color:#7f1d1d;background:#1a0a0a}
.sec{font-size:.7rem;font-weight:700;color:var(--muted);letter-spacing:.12em;text-transform:uppercase;margin:16px 0 8px 4px}
.card{background:var(--card);border:1px solid var(--border);border-radius:16px;padding:14px;margin-bottom:10px}
.card h3{font-size:.88rem;font-weight:600;margin-bottom:10px}
.card h3 small{font-weight:400;color:var(--muted);font-size:.75rem}
.pad{display:flex;align-items:center;justify-content:center;gap:8px;flex-wrap:wrap}
.pad button{min-width:58px;min-height:48px;padding:10px 14px;border:none;border-radius:12px;font-size:.85rem;font-weight:700;cursor:pointer;transition:transform .12s,box-shadow .12s}
.pad .up{background:var(--up);color:#052e16}
.pad .dn{background:var(--down);color:#fff}
.pad .stp{background:var(--stop);color:#fff}
.pad .lf,.pad .rt{background:var(--rot);color:#1a1200}
.pad .fwd,.pad .rev{background:var(--trk);color:#fff}
.pad .fwd{color:#eff6ff}
.pad button:active{transform:scale(.96)}
.pad button.on{box-shadow:0 0 0 2px #fff,0 0 14px rgba(255,255,255,.35);transform:scale(1.03)}
.spd{display:flex;align-items:center;gap:8px;margin-top:10px;justify-content:center}
.spd label{font-size:.72rem;color:var(--muted);min-width:36px}
.spd input{width:150px;height:28px;accent-color:var(--trk)}
.locked .pad button:not(.stp),.locked .spd input{opacity:.35;pointer-events:none}
</style>
</head>
<body>
<div class="hdr">
  <h1>DRAGLINE CRANE</h1>
  <span class="pill on" id="pill">ONLINE</span>
</div>
<button class="estop" onclick="allStop()">ALL STOP</button>
<button class="release" id="releaseBtn" onclick="releasePs3()">Switch to Phone Control</button>
<div class="status" id="status">Connecting...</div>

<div class="sec">Winches</div>
<div class="card" id="m2">
  <h3>Boom</h3>
  <div class="pad"><button class="up" onclick="cmd(2,1)">Up</button><button class="stp" onclick="cmd(2,0)">Stop</button><button class="dn" onclick="cmd(2,-1)">Down</button></div>
  <div class="spd"><label>Spd</label><input type="range" min="0" max="255" value="200" oninput="speed(2,this.value)"></div>
</div>
<div class="card" id="m1">
  <h3>Bucket</h3>
  <div class="pad"><button class="up" onclick="cmd(1,1)">Up</button><button class="stp" onclick="cmd(1,0)">Stop</button><button class="dn" onclick="cmd(1,-1)">Down</button></div>
  <div class="spd"><label>Spd</label><input type="range" min="0" max="255" value="200" oninput="speed(1,this.value)"></div>
</div>
<div class="card" id="m0">
  <h3>Tilt</h3>
  <div class="pad"><button class="up" onclick="cmd(0,1)">Tilt</button><button class="stp" onclick="cmd(0,0)">Stop</button><button class="dn" onclick="cmd(0,-1)">Dump</button></div>
  <div class="spd"><label>Spd</label><input type="range" min="0" max="255" value="200" oninput="speed(0,this.value)"></div>
</div>

<div class="sec">Rotation</div>
<div class="card" id="m3">
  <h3>Superstructure</h3>
  <div class="pad"><button class="lf" onclick="cmd(3,-1)">Left</button><button class="stp" onclick="cmd(3,0)">Stop</button><button class="rt" onclick="cmd(3,1)">Right</button></div>
  <div class="spd"><label>Spd</label><input type="range" min="0" max="255" value="200" oninput="speed(3,this.value)"></div>
</div>

<div class="sec">Tracks</div>
<div class="card" id="tracks-both">
  <h3>Both Tracks <small>full speed</small></h3>
  <div class="pad"><button class="fwd" onclick="tracksBoth(1)">Fwd</button><button class="stp" onclick="tracksBoth(0)">Stop</button><button class="rev" onclick="tracksBoth(-1)">Rev</button></div>
</div>
<div class="card" id="m4">
  <h3>Left Track <small>full speed</small></h3>
  <div class="pad"><button class="fwd" onclick="cmd(4,1)">Fwd</button><button class="stp" onclick="cmd(4,0)">Stop</button><button class="rev" onclick="cmd(4,-1)">Rev</button></div>
</div>
<div class="card" id="m5">
  <h3>Right Track <small>full speed</small></h3>
  <div class="pad"><button class="fwd" onclick="cmd(5,1)">Fwd</button><button class="stp" onclick="cmd(5,0)">Stop</button><button class="rev" onclick="cmd(5,-1)">Rev</button></div>
</div>

<script>
var webLocked=false;
function hlBtn(c,d){
  var b=c.querySelectorAll('.pad>button');for(var i=0;i<b.length;i++)b[i].classList.remove('on');
  var t;if(d===1)t=c.querySelector('.up,.fwd,.rt');else if(d===-1)t=c.querySelector('.dn,.rev,.lf');else t=c.querySelector('.stp');
  if(t)t.classList.add('on');
}
function cmd(m,d){
  if(webLocked&&d!==0)return;
  var e=document.getElementById('m'+m);if(e)hlBtn(e,d);
  if(m===4||m===5){var both=document.getElementById('tracks-both');var bs=both.querySelectorAll('.pad>button');for(var i=0;i<bs.length;i++)bs[i].classList.remove('on');}
  fetch('/motor?m='+m+'&dir='+d).catch(function(){});
}
function speed(m,v){if(webLocked)return;fetch('/motor?m='+m+'&speed='+v).catch(function(){});}
function tracksBoth(d){
  if(webLocked&&d!==0)return;
  hlBtn(document.getElementById('tracks-both'),d);hlBtn(document.getElementById('m4'),d);hlBtn(document.getElementById('m5'),d);
  fetch('/motor?m=4&dir='+d).catch(function(){});fetch('/motor?m=5&dir='+d).catch(function(){});
}
function allStop(){fetch('/allstop').catch(function(){});for(var i=0;i<6;i++){var e=document.getElementById('m'+i);if(e)hlBtn(e,0);}hlBtn(document.getElementById('tracks-both'),0);}
function fmtUp(ms){var s=Math.floor(ms/1000),m=Math.floor(s/60);s%=60;return(m<10?'0':'')+m+':'+(s<10?'0':'')+s;}
function motorSummary(motors){
  var n=['Tilt','Bucket','Boom','Rot','LTrk','RTrk'],a=[];for(var i=0;i<motors.length;i++)if(motors[i].dir!==0)a.push(n[i]);return a.length?a.join(', '):'Idle';
}
function applyStatus(s){
  webLocked=!!s.webLocked;
  var st=document.getElementById('status'),pill=document.getElementById('pill'),body=document.body,rb=document.getElementById('releaseBtn');
  body.classList.toggle('locked',webLocked);
  rb.classList.toggle('show',webLocked);
  if(s.ps3Sleep){st.className='status warn';st.textContent='PS3 asleep — press PS on controller to reconnect, or use phone now';pill.className='pill';pill.textContent='PS3 SLEEP';}
  else if(s.ps3){st.className='status lock';var idle=s.ps3IdleSec||0,rem=Math.max(0,300-idle),rm=Math.floor(rem/60),rs=rem%60;
    st.textContent='PS3 active — phone locked'+(idle>=30?' · sleep in '+rm+':'+(rs<10?'0':'')+rs:'')+'. Tap below to use phone.';pill.className='pill ps3';pill.textContent='PS3'+(s.ps3Bat?' '+s.ps3Bat:'');}
  else{st.className='status';st.textContent='Phone control · '+motorSummary(s.motors)+' · up '+fmtUp(s.uptime);pill.className='pill on';pill.textContent='ONLINE';}
  for(var i=0;i<s.motors.length;i++){var el=document.getElementById('m'+i);if(el)hlBtn(el,s.motors[i].dir);}
  if(s.motors[4]&&s.motors[5]&&s.motors[4].dir===s.motors[5].dir)hlBtn(document.getElementById('tracks-both'),s.motors[4].dir);else hlBtn(document.getElementById('tracks-both'),0);
}
function releasePs3(){fetch('/ps3release').then(function(){pollStatus();}).catch(function(){});}
function pollStatus(){fetch('/status').then(function(r){return r.json();}).then(applyStatus).catch(function(){});}
document.addEventListener('DOMContentLoaded',function(){for(var i=0;i<6;i++){var e=document.getElementById('m'+i);if(e)hlBtn(e,0);}hlBtn(document.getElementById('tracks-both'),0);pollStatus();setInterval(pollStatus,500);});
</script>
</body>
</html>
)rawliteral";

// ─── Setup & Loop ───────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n[DraglineCrane] Setup start."));
  switch (esp_reset_reason()) {
    case ESP_RST_BROWNOUT:
      Serial.println(F("[DraglineCrane] WARNING: last reset was BROWNOUT (power dip)."));
      Serial.println(F("  Tracks draw high current — add 470uF cap on ESP 5V, or slow TRACK_RAMP_STEP."));
      break;
    case ESP_RST_PANIC:
      Serial.println(F("[DraglineCrane] Last reset: PANIC (software crash)."));
      break;
    case ESP_RST_TASK_WDT:
    case ESP_RST_INT_WDT:
    case ESP_RST_WDT:
      Serial.println(F("[DraglineCrane] Last reset: WATCHDOG."));
      break;
    default:
      break;
  }

  pinMode(PIN_LED_ACTION, OUTPUT);
  digitalWrite(PIN_LED_ACTION, LOW);

  oledReady = initDisplay();
  if (oledReady) {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(F("BOOT..."));
    display.display();
  }

  // Bluetooth must init before WiFi on ESP32 core 3.x (Classic BT memory + coexistence)
  initPs3();

  // Initialize WiFi Access Point
  initWiFiAP();
  Serial.print(F("[DraglineCrane] AP IP: "));
  Serial.println(WiFi.softAPIP());

  // Configure GPIO pins for all 6 motors
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(MOTORS[i].in1, OUTPUT);
    pinMode(MOTORS[i].in2, OUTPUT);
    digitalWrite(MOTORS[i].in1, LOW);
    digitalWrite(MOTORS[i].in2, LOW);
    ledcAttach(MOTORS[i].en, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(MOTORS[i].en, 0);
  }

  // Register HTTP routes
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/status", HTTP_GET, []() {
    char buf[512];
    int pos = 0;
    pos += snprintf(buf + pos, sizeof(buf) - pos, "{");
    pos += snprintf(buf + pos, sizeof(buf) - pos, "\"ps3\":%s,", ps3Connected ? "true" : "false");
    pos += snprintf(buf + pos, sizeof(buf) - pos, "\"webLocked\":%s,", isWebLocked() ? "true" : "false");
    pos += snprintf(buf + pos, sizeof(buf) - pos, "\"ps3Sleep\":%s,", ps3SleepMode ? "true" : "false");
    pos += snprintf(buf + pos, sizeof(buf) - pos, "\"uptime\":%lu,", millis());
    pos += snprintf(buf + pos, sizeof(buf) - pos, "\"clients\":%u,",
                    WiFi.softAPgetStationNum());
    pos += snprintf(buf + pos, sizeof(buf) - pos, "\"ps3IdleSec\":%lu,",
                    ps3Connected ? (millis() - lastPs3ActivityMs) / 1000UL : 0UL);
    pos += snprintf(buf + pos, sizeof(buf) - pos, "\"ps3Bat\":\"%s\",",
                    ps3Connected ? ps3BatteryLabel() : "");
    pos += snprintf(buf + pos, sizeof(buf) - pos, "\"motors\":[");
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (i > 0) {
        pos += snprintf(buf + pos, sizeof(buf) - pos, ",");
      }
      pos += snprintf(buf + pos, sizeof(buf) - pos,
                      "{\"dir\":%d,\"speed\":%u}", motorDir[i], motorSpeed[i]);
    }
    pos += snprintf(buf + pos, sizeof(buf) - pos, "]}");
    server.send(200, "application/json", buf);
  });

  server.on("/motor", HTTP_GET, []() {
    if (isWebLocked()) {
      server.send(403, "text/plain", "controller active");
      return;
    }

    int m = -1;
    if (server.hasArg("m")) m = server.arg("m").toInt();

    if (m >= 0 && m < NUM_MOTORS) {
      // Update speed if provided (ignored for tracks)
      if (server.hasArg("speed") && m != 4 && m != 5) {
        int speedVal = server.arg("speed").toInt();
        if (speedVal >= 0 && speedVal <= 255) {
          motorSpeed[m] = (uint8_t)speedVal;
        }
      }

      // Update direction if provided
      if (server.hasArg("dir")) {
        int dir = server.arg("dir").toInt();
        if (dir >= -1 && dir <= 1) {
          motorDir[m] = (int8_t)dir;
        }
      }

      // Apply motor state (handles speed-only updates by re-applying current direction)
      setMotor(m, motorDir[m], motorSpeed[m]);
      clearPs3SleepForPhone();
      bumpOledActivity();
      refreshStatus();
    }
    server.send(200, "text/plain", "ok");
  });

  server.on("/allstop", HTTP_GET, []() {
    allStop();
    clearPs3SleepForPhone();
    server.send(200, "text/plain", "ok");
  });

  server.on("/ps3release", HTTP_GET, []() {
    forcePs3Sleep();
    server.send(200, "text/plain", "ok");
  });

  // Captive portal: redirect unknown paths to "/"
  server.onNotFound([]() {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });

  server.begin();

  bootMacDisplayEnd = millis() + BOOT_MAC_DISPLAY_MS;
  if (oledReady) {
    showBootBtMacScreen();
  }

  refreshStatus();
  bootTimeMs = millis();
  lastPs3ActivityMs = bootTimeMs;
  lastOledActivityMs = bootTimeMs;
  Serial.println(F("[DraglineCrane] Web server started. Ready."));
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
  tickPs3Connection();
  tickPs3StaleCheck();
  if (ps3Ready) {
    Ps3.tickRearm();
  }
  tickTrackRamp();

  if (ps3Ready && (Ps3.isConnected() || ps3SleepMode)) {
    uint32_t now = millis();
    if (now - lastPs3PollMs >= PS3_POLL_MS) {
      lastPs3PollMs = now;
      pollPs3Input();
    }
    if (ps3Connected) {
      tickPs3IdleSleep();
    }
  }

  tickActionLed();
  tickOledSleep();

  uint32_t displayInterval = ps3Connected ? (DISPLAY_REFRESH_MS * 2) : DISPLAY_REFRESH_MS;
  if (oledReady && oledPanelOn && (millis() - lastDisplayUpdate >= displayInterval)) {
    updateDisplay();
  }
}
