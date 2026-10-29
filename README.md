# RC Dragline Crane – Web + PS3 Control for 6 Motors

An ESP32-based controller for an RC dragline crane. Three L298N motor drivers run six DC motors (three winches, a rotation motor and two tracks). The crane can be driven from a phone over its own WiFi access point, or with a Bluetooth PS3 (DualShock 3) controller.

- **Board:** ESP32 + 3× L298N motor drivers
- **IDE:** Arduino IDE (ESP32 core 3.x)
- **Control:** Phone web UI (WiFi AP, no router needed) and/or PS3 DualShock 3 over Bluetooth
- **Display/status:** SSD1306 128×32 OLED, action LED, fault LED

---

## Table of Contents

1. [Arduino IDE Build Settings](#arduino-ide-build-settings)
2. [Motor Index Table](#motor-index-table)
3. [WiFi Configuration](#wifi-configuration)
4. [HTTP Endpoints](#http-endpoints)
5. [PS3 Bluetooth Controller](#ps3-bluetooth-controller)
6. [Pin Mapping](#pin-mapping)
7. [Power Distribution](#power-distribution)
8. [Wiring Schematic](#wiring-schematic)

---

## Arduino IDE Build Settings

> **Required when using PS3 Bluetooth.**

WiFi AP + Bluetooth Classic needs about 1.7 MB of flash, which is larger than the default 1.2 MB app partition. In the **Tools** menu set:

| Setting | Value |
|---|---|
| Board | ESP32 Dev Module |
| Flash Size | 4MB (32Mb) — match your board |
| Partition Scheme | Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS) **or** Huge APP (3MB No OTA/1MB SPIFFS) |
| Core Debug Level | None (optional, saves a little space) |

If the upload still fails, try **Optimize: Size (-Os)** under Tools.

---

## Motor Index Table

| Index | Function | Direction Labels |
|:---:|---|---|
| 0 | Tilt Winch | Tilt (1) / Dump (-1) |
| 1 | Bucket Winch | Up (1) / Down (-1) |
| 2 | Boom Winch | Up (1) / Down (-1) |
| 3 | Rotation | Right (1) / Left (-1) |
| 4 | Left Track | Fwd (1) / Rev (-1) |
| 5 | Right Track | Fwd (1) / Rev (-1) |

---

## WiFi Configuration

The ESP32 creates its own access point, so no external router is needed.

| Item | Value |
|---|---|
| Mode | WiFi AP (Access Point) |
| SSID | `DraglineCrane` |
| Password | `crane1234` |
| IP address | `192.168.4.1` |

Connect your phone to the `DraglineCrane` network and open <http://192.168.4.1> in a browser.

---

## HTTP Endpoints

| Method | Path | Parameters | Response |
|---|---|---|---|
| GET | `/` | none | 200, HTML control page |
| GET | `/status` | none | 200, JSON (ps3, motors, uptime, idle) |
| GET | `/motor` | `m`=0..5, `dir`=-1/0/1, `speed`=0..255 | 200 `"ok"` / 403 if PS3 active |
| GET | `/allstop` | none | 200 `"ok"` |
| GET | `/ps3release` | none | 200 `"ok"` — releases PS3 and unlocks the phone |
| GET | *(anything else)* | none | 302 redirect to `/` |

### `/motor` parameter notes

- `m` (required): motor index 0–5
- `dir` (optional): direction `-1`, `0` or `1`
- `speed` (optional): PWM duty 0–255 (ignored for tracks)
- If only `speed` is sent (no `dir`), the speed is updated and the current direction is re-applied
- Returns `403 "controller active"` while a PS3 controller is connected

Example:

```
http://192.168.4.1/motor?m=2&dir=1&speed=200
http://192.168.4.1/allstop
```

---

## PS3 Bluetooth Controller

Supports the DualShock 3.

### Pairing (one-time)

1. Upload the sketch. The OLED shows **"Pair PS3 to:"** plus the ESP32 Bluetooth MAC for 10 s at boot.
   *(Do **not** hardcode `ps3BtMac` — it is filled in automatically after BT init.)*
2. Plug the PS3 controller into a PC/Mac via USB.
3. Write the displayed ESP32 MAC into the controller (SixaxisPairTool / sixaxispairer).
4. Unplug USB and press the PS button — it connects within ~10 s.

> **"BT INIT FAIL"** at boot means Bluetooth failed to start. It is not a pairing problem. On ESP32 core 3.x use the patched library in `libraries/PS3_Controller_Host/`.

### Exclusive mode

While a PS3 controller is connected, `/motor` web commands are blocked. The phone web page still loads, and `/allstop` always works as an emergency backup. Use `/ps3release` to release the controller and unlock the phone.

### Button map

Hold to run, release to stop.

| Input | Function |
|---|---|
| Left stick Y | Left track (4): fwd (up) / rev (down) |
| Right stick Y | Right track (5): fwd (up) / rev (down) |
| D-pad Up / Down | Bucket winch (1) up / down |
| D-pad Left / Right | Boom winch (2) up / down |
| Triangle / Cross | Tilt winch (0) tilt / dump |
| L1 / R1 | Rotation (3) left / right |
| PS button | All-stop |

Both sticks up = straight ahead; opposite directions = tank turn.

### Speeds

PS3 winch speeds (Boom / Bucket / Tilt / Rotation) are set by the `PS3_MOTOR_SPEED` constants in the sketch (default ~200). Tracks still ramp to full PWM. The web UI speed sliders are unchanged.

### Idle sleep

After 5 minutes with no deliberate PS3 input, the sketch runs all-stop and disconnects Bluetooth. Press PS to reconnect. A short rumble pulse confirms the link.

### Library

Uses **PS3 Controller Host**. A patched copy (for ESP32 Arduino core 3.x) is included in the project `libraries/` folder. Remove any duplicate installed from the Library Manager.

---

## Pin Mapping

### Motors

| Motor | Board | Ch | EN | IN1 | IN2 |
|---|:---:|:---:|:---:|:---:|:---:|
| 0: Tilt Winch | 1 | A | 23 | 22 | 21 |
| 1: Bucket Winch | 1 | B | 19 | 18 | 5 |
| 2: Boom Winch | 2 | A | 17 | 16 | 4 |
| 3: Rotation | 2 | B | 25 | 26 | 27 |
| 4: Left Track | 3 | A | 14 | 12 | 13 |
| 5: Right Track | 3 | B | 33 | 32 | 15 |

### Status and display

| Function | Board label | GPIO | Notes |
|---|---|:---:|---|
| Action LED | P0 | 0 | Heartbeat when idle, solid when active |
| OLED I2C SDA | TX | 1 | SSD1306 128×32 data |
| Fault LED | P2 | 2 | Fast blink if the WiFi AP fails |
| OLED I2C SCL | RX | 3 | SSD1306 128×32 clock |

The OLED turns off after 10 minutes with no motors running and no user input.

### Pin notes

- Do **not** use SD0–SD3 (GPIO 7–10) — connected to internal flash.
- Do **not** use P34, P35, SVP (GPIO 34–36) — input-only, no LED/I2C.
- Total motor GPIO used: 18 (all unique).
- Avoided: GPIO 6–11 (flash), GPIO 34–39 (input-only).
- Strapping pins used: GPIO 12 and 15 (both safe as outputs).
- GPIO 0 is a boot strapping pin — wire the action LED active-LOW (3.3V → resistor → LED → GPIO 0) or make sure the LED is off at boot.

---

## Power Distribution

- Single 12 V power source (battery or PSU).
- 12 V+ connects in parallel to all 3× L298N boards (shared 12 V rail).
- GND connects in parallel to all 3× L298N boards (shared GND bus).
- The ESP32 is powered from L298N Board 1's onboard 5 V regulator:
  - **Board 1:** 5V jumper **enabled** → 5V pin connects to ESP32 VIN
  - **Board 2:** 5V jumper **disabled**
  - **Board 3:** 5V jumper **disabled**
- **Common ground bus:** ESP32 GND, all 3× L298N GND terminals and the power supply GND **must** be connected together.
- Tracks draw the most current (two motors). Soft-start: kick to PWM 120, then ramp to 255 over ~150 ms to limit inrush brownout resets.
- If the ESP32 still reboots when the tracks run: use a sturdier 12 V supply (≥ 5 A recommended for two gearmotors), add 470–1000 µF across ESP 5 V/GND, and keep motor power and ESP grounds tied to the same bus.

---

## Wiring Schematic

```
                         ┌─────────────────┐
                         │   12V BATTERY   │
                         │   (+)     (-)   │
                         └──┬─────────┬────┘
                            │         │
             ┌──────────────┼─────────┼──────────────────────┐
             │          12V RAIL       │     GND BUS          │
             │              │          │                      │
    ┌────┬───┼──────────────┼──────────┼───────────┐          │
    │    │   │              │          │           │          │
    │    ▼   ▼              ▼          ▼           │          │
    │ ┌──────────┐  ┌──────────┐  ┌──────────┐     │          │
    │ │ L298N #1 │  │ L298N #2 │  │ L298N #3 │     │          │
    │ │ 12V  GND │  │ 12V  GND │  │ 12V  GND │     │          │
    │ │ 5V=ON    │  │ 5V=OFF   │  │ 5V=OFF   │     │          │
    │ │ ChA  ChB │  │ ChA  ChB │  │ ChA  ChB │     │          │
    │ └──┼────┼──┘  └──┼────┼──┘  └──┼────┼──┘     │          │
    │    ▼    ▼        ▼    ▼        ▼    ▼        │          │
    │  ┌──┐ ┌──┐     ┌──┐ ┌──┐     ┌──┐ ┌──┐      │          │
    │  │M0│ │M1│     │M2│ │M3│     │M4│ │M5│      │          │
    │  └──┘ └──┘     └──┘ └──┘     └──┘ └──┘      │          │
    │  Tilt  Bucket   Boom  Rot     LTrk  RTrk     │          │
    │                                              │          │
    │    ┌─────┘ 5V (from Board 1 regulator)       │          │
    │    ▼                                         │          │
    │ ┌─────────────────────────────────┐          │          │
    │ │            ESP32                │          │          │
    │ │  VIN ←── 5V from L298N #1       │          │          │
    │ │  GND ←── Common GND Bus ────────┼──────────┼──────────┘
    │ │                                 │          │
    │ │  GPIO23 (EN0)   ──► L298N #1 ENA│          │
    │ │  GPIO22 (IN1_0) ──► L298N #1 IN1│          │
    │ │  GPIO21 (IN2_0) ──► L298N #1 IN2│          │
    │ │  GPIO19 (EN1)   ──► L298N #1 ENB│          │
    │ │  GPIO18 (IN1_1) ──► L298N #1 IN3│          │
    │ │  GPIO5  (IN2_1) ──► L298N #1 IN4│          │
    │ │                                 │          │
    │ │  GPIO17 (EN2)   ──► L298N #2 ENA│          │
    │ │  GPIO16 (IN1_2) ──► L298N #2 IN1│          │
    │ │  GPIO4  (IN2_2) ──► L298N #2 IN2│          │
    │ │  GPIO25 (EN3)   ──► L298N #2 ENB│          │
    │ │  GPIO26 (IN1_3) ──► L298N #2 IN3│          │
    │ │  GPIO27 (IN2_3) ──► L298N #2 IN4│          │
    │ │                                 │          │
    │ │  GPIO14 (EN4)   ──► L298N #3 ENA│          │
    │ │  GPIO12 (IN1_4) ──► L298N #3 IN1│          │
    │ │  GPIO13 (IN2_4) ──► L298N #3 IN2│          │
    │ │  GPIO33 (EN5)   ──► L298N #3 ENB│          │
    │ │  GPIO32 (IN1_5) ──► L298N #3 IN3│          │
    │ │  GPIO15 (IN2_5) ──► L298N #3 IN4│          │
    │ │                                 │          │
    │ │  P0 (GPIO0) ──► Action LED      │          │
    │ │  TX (GPIO1) ──► OLED SDA        │          │
    │ │  P2 (GPIO2) ──► Fault LED       │          │
    │ │  RX (GPIO3) ──► OLED SCL        │          │
    │ └─────────────────────────────────┘          │
    │                                              │
    └──────────────────────────────────────────────┘
```

**Legend:** M0 = Tilt Winch, M1 = Bucket Winch, M2 = Boom Winch, M3 = Rotation, M4 = Left Track, M5 = Right Track.