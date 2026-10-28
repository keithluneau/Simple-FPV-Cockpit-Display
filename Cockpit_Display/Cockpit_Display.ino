// =====================================================================
//  Simple FPV Cockpit Display by Keith Luneau (aka Bonafidepirate)
//  MAVLink artificial horizon / attitude indicator for the LilyGo
//  T-Display-S3, for use with an ArduPilot flight controller.
//
//  Written with Claude (Anthropic). Released under the MIT License;
//  see the LICENSE file.
//
//  Shows roll and pitch, airspeed, altitude above home, arm state and
//  flight mode, plus a blinking pop-up warning for low battery voltage
//  or low altitude. It is meant to be filmed by an FPV camera, so the
//  markings are thick and outlined.
//
//  WIRING
//    Flight controller telemetry TX  ->  MAV_RX_PIN (GPIO 44)
//    Flight controller GND           ->  GND
//    Flight controller 5V            ->  5V pin on the display board
//    Set the telemetry port's baud rate to match MAV_BAUD.
//
//    WARNING: while the display board is wired to the flight
//    controller's 5V, do NOT also power it from USB. Two 5V sources
//    connected together risk feeding 5V back into the flight
//    controller. Unplug the 5V wire first before connecting USB to
//    program or bench-test the board.
//
//  FLIGHT CONTROLLER STREAMS
//  Set these stream rates for the MAVLink port the display is plugged
//  into (ArduPilot 4.7 or newer). In MAVn, n is the MAVLink port
//  number: MAV1 is the first serial port set to MAVLink, MAV2 the
//  second, and so on. Older firmware used SRn_ names for these
//  (SR0_EXTRA1 and so on, numbered from 0), so check the names for
//  your version.
//    Parameter      Rate  Message              Used for
//    MAVn_EXTRA1   10 Hz  ATTITUDE             horizon (20 Hz is smoother)
//    MAVn_EXTRA2    4 Hz  VFR_HUD              airspeed
//    MAVn_POSITION  4 Hz  GLOBAL_POSITION_INT  altitude above home
//    MAVn_EXT_STAT  2 Hz  SYS_STATUS           battery voltage
//    MAVn_EXTRA3    2 Hz  BATTERY_STATUS       battery voltage (this or
//                                              EXT_STAT; one is enough)
//    MAVn_RC_CHAN  10 Hz  RC_CHANNELS          only if brightness is
//                                              set by an R/C channel
//  The rates are suggested starting points. Keep every stream you use
//  at 1 Hz or faster, or its readout shows "---" (see DATA_TIMEOUT_MS).
//  The heartbeat (arm state, flight mode) is always sent.
//
//  IO14 BUTTON
//    Press to start or stop demo mode, a simulated flight that needs no
//    MAVLink connection.
//
//  Everything you're likely to want to change is in the USER SETTINGS
//  section below. The program code after it shouldn't need editing.
// =====================================================================

#define LILYGO_LGFX_USE_T_DISPLAY_S3
#include <LilyGo_LovyanGFX.h>
#include <MAVLink.h>
#include <math.h>


// =====================================================================
//  USER SETTINGS
// =====================================================================

// Each setting has its default value in a "default:" comment, so you can
// always get back to a known starting point.

// ---------------------------------------------------------------------
//  Hardware
// ---------------------------------------------------------------------

// Telemetry serial pins (T-Display-S3). RX receives the flight
// controller's TX. TX is set up but never used to send anything.
const int MAV_RX_PIN = 44;            // default: 44
const int MAV_TX_PIN = 43;            // default: 43
const uint32_t MAV_BAUD = 115200;     // default: 115200

// User button (IO14, active low) and its debounce time.
const int BUTTON_PIN = 14;                // default: 14
const uint32_t BUTTON_DEBOUNCE_MS = 40;   // default: 40

// ---------------------------------------------------------------------
//  Units
// ---------------------------------------------------------------------

// Speed: SPEED_KMH, SPEED_MS, SPEED_MPH or SPEED_KNOTS.
// Altitude: ALT_METERS or ALT_FEET.
// The altitude warning limit (WARN_ALT) uses the altitude unit.
enum SpeedUnit { SPEED_KMH, SPEED_MS, SPEED_MPH, SPEED_KNOTS };
enum AltUnit   { ALT_METERS, ALT_FEET };

const SpeedUnit SPEED_UNIT = SPEED_MPH;   // default: SPEED_MPH
const AltUnit   ALT_UNIT   = ALT_FEET;    // default: ALT_FEET

// ---------------------------------------------------------------------
//  Warning pop-up
// ---------------------------------------------------------------------

// Set a limit to 0 to disable that warning.

// Low battery, total pack voltage in volts. Set it to the number of
// cells times the voltage per cell you want to be warned at. The default
// is 3.3 V per cell on a 4S LiPo.
const float WARN_PACK_V = 13.20f;     // default: 13.2

// Low altitude above home, in the units chosen by ALT_UNIT (ft or m).
// Only active while armed, and only after the plane has first climbed
// above the limit, so it doesn't fire on the ground or during launch.
const float WARN_ALT = 50.0f;         // default: 50 (ft with the default unit)

// A warning clears once the value is this fraction above its limit, so
// it doesn't flicker on and off right at the threshold.
const float WARN_HYST = 0.05f;        // default: 0.05

// Which battery to monitor (MAVLink battery id, 0 = first/main).
const uint8_t BATT_ID = 0;            // default: 0

// Pop-up blinking: visible for ON ms, then hidden for OFF ms.
// Set WARN_BLINK_OFF_MS to 0 for a steady (non-blinking) pop-up.
const uint32_t WARN_BLINK_ON_MS  = 600;   // default: 600
const uint32_t WARN_BLINK_OFF_MS = 300;   // default: 300

// When the plane is armed, a red "WARNING / ARMED" pop-up shows for this
// long, in ms. It doesn't blink, and a low battery or low altitude
// warning takes priority over it. 0 = off.
const uint32_t ARMED_NOTICE_MS = 5000;    // default: 5000

// ---------------------------------------------------------------------
//  Screen brightness
// ---------------------------------------------------------------------

// R/C channel (1-18) that controls the brightness. 0 = disabled, the
// screen stays at BRIGHT_DEFAULT.
const int BRIGHT_RC_CHANNEL = 0;      // default: 0 (disabled)

// Channel values (microseconds) that map to the dimmest and brightest
// settings. Swap the two numbers to reverse the direction.
const int BRIGHT_RC_LOW_US  = 1000;   // default: 1000
const int BRIGHT_RC_HIGH_US = 2000;   // default: 2000

// Brightness range, 0-255. The backlight chip has 16 steps, so these are
// rounded to the nearest step. A minimum of 0 turns the backlight
// completely off. Use a higher minimum if you never want the screen to
// go fully dark.
const uint8_t BRIGHT_MIN = 0;         // default: 0
const uint8_t BRIGHT_MAX = 255;       // default: 255

// Brightness used at startup, when the R/C control is disabled, and
// until the first valid channel value arrives.
const uint8_t BRIGHT_DEFAULT = 255;   // default: 255

// Brightness curve. 1.0 is linear. Higher values spread the dim end of
// the range out over more of the channel's travel, where the eye is most
// sensitive, and crowd the bright end together. With a BRIGHT_MIN of 0,
// a higher value also widens the "off" zone at the bottom of the range.
const float BRIGHT_GAMMA = 1.5f;      // default: 1.5

// ---------------------------------------------------------------------
//  Telemetry and timing
// ---------------------------------------------------------------------

// A readout shows "---" if its data stops arriving for this long.
const uint32_t DATA_TIMEOUT_MS = 2000;        // default: 2000

// The link counts as lost if no autopilot heartbeat arrives for this
// long. The status line then shows "Waiting for MAVLINK".
const uint32_t HEARTBEAT_TIMEOUT_MS = 3000;   // default: 3000

// If no attitude data arrives for this long, the horizon levels itself.
const uint32_t ATTITUDE_TIMEOUT_MS = 1000;    // default: 1000

// Time between display updates (33 ms is about 30 frames per second).
const uint32_t FRAME_INTERVAL_MS = 33;        // default: 33

// ---------------------------------------------------------------------
//  Appearance
// ---------------------------------------------------------------------

// Colors (RGB565).
const uint16_t SKY      = 0x249F;   // default: 0x249F (blue)
const uint16_t GROUND   = 0x8200;   // default: 0x8200 (brown)
const uint16_t WHITE    = 0xFFFF;   // default: 0xFFFF
const uint16_t YELLOW   = 0xFFE0;   // default: 0xFFE0
const uint16_t CYAN     = 0x07FF;   // default: 0x07FF
const uint16_t BLACK    = 0x0000;   // default: 0x0000
const uint16_t RED      = 0xF800;   // default: 0xF800

// Line thicknesses in pixels. Tweak these to taste.
const float T_HORIZON = 5.0f;  // horizon line (default: 5.0)
const float T_LADDER  = 4.0f;  // pitch ladder rungs (default: 4.0)
const float T_BANK    = 4.0f;  // bank reference ticks (default: 4.0)
const float T_SYMBOL  = 5.0f;  // fixed yellow aircraft symbol (default: 5.0)

// Black outline added to each side of every marking, so the white and
// yellow lines stay visible against both the blue sky and brown ground.
// Set to 0 to disable.
const int OUTLINE = 3;         // default: 3

// Fixed yellow aircraft symbol: two bars, each with a short drop at its
// inner end. Distances are in pixels from the screen center. The bars
// sit on the horizon line when pitch and roll are level.
const int SYMBOL_OUTER = 52;   // center to outer end of each bar (default: 52)
const int SYMBOL_INNER = 16;   // center to inner end of each bar (default: 16)
const int SYMBOL_DROP  = 9;    // how far each inner end drops (default: 9)
const int SYMBOL_SLANT = 3;    // lean toward the center, 0 = straight down
                               // (default: 3)

// Pitch scale: how far the horizon moves per degree of pitch, in pixels.
const float PITCH_PIXELS_PER_DEGREE = 5.0f;   // default: 5.0

// Pitch ladder: a rung every LADDER_STEP_DEG degrees, up to
// LADDER_MAX_DEG above and below the horizon. LADDER_STEP_DEG must be
// greater than 0, and LADDER_MAX_DEG should be a multiple of it.
const int LADDER_STEP_DEG = 5;        // default: 5
const int LADDER_MAX_DEG  = 10;       // default: 10

// Readout boxes (left: airspeed, right: altitude above home).
const int BOX_W = 78;                 // default: 78
const int BOX_H = 34;                 // default: 34
const int BOX_MARGIN = 4;             // default: 4

// Text sizes (1 = smallest; each step adds about 6 x 8 pixels per
// character). Text that would run off the screen is shrunk to fit.
// Unit labels under the readout boxes ("mph", "ft").
const int LABEL_TEXT_SIZE = 3;        // default: 3

// Bottom status line. While disarmed it shows "DISARMED - <flight mode>"
// in the first size. While armed it shows just the flight mode, in the
// second (larger) size.
const int STATUS_SIZE_DISARMED = 2;   // default: 2
const int STATUS_SIZE_ARMED    = 3;   // default: 3

// Warning pop-up box layout.
const int WARN_W = 200;               // default: 200
const int WARN_HDR_H = 28;            // default: 28
const int WARN_BODY_H = 34;           // default: 34
const int WARN_Y = 4;                 // default: 4


// =====================================================================
//  PROGRAM CODE - you shouldn't need to change anything below here
// =====================================================================

LilyGo_T_Display_S3 tft;
LGFX_Sprite canvas(&tft);

// Screen size and center (T-Display-S3, landscape).
const int W = 320;
const int H = 170;
const int CX = W / 2;
const int CY = H / 2;

// Conversion tables for the unit settings above, indexed by SPEED_UNIT
// and ALT_UNIT. The speed factors convert from m/s, the altitude
// factors from metres.
const float SPEED_FACTORS[] = {3.6f, 1.0f, 2.23694f, 1.94384f};
const char* const SPEED_UNIT_NAMES[] = {"km/h", "m/s", "mph", "kt"};
const float ALT_FACTORS[] = {1.0f, 3.28084f};
const char* const ALT_UNIT_NAMES[] = {"m", "ft"};

// Backlight chip (AW9364): control pin and number of brightness steps.
const int BACKLIGHT_PIN = 38;
const int BACKLIGHT_STEPS = 16;

// Sanity checks on the settings.
static_assert(LADDER_STEP_DEG > 0, "LADDER_STEP_DEG must be greater than 0");
static_assert(BRIGHT_MIN <= BRIGHT_MAX, "BRIGHT_MIN must not exceed BRIGHT_MAX");


float rollDeg = 0.0f;
float pitchDeg = 0.0f;
float airspeedMs = 0.0f;   // estimated airspeed in m/s (VFR_HUD)
float homeAltM = 0.0f;     // altitude above home in m (GLOBAL_POSITION_INT)
uint32_t lastVfrMs = 0;
uint32_t lastPosMs = 0;

// Battery, arming and flight mode state.
float packV = 0.0f;        // total pack voltage in V
uint32_t lastPackMs = 0;
bool armed = false;
uint32_t flightMode = 0;   // ArduPilot custom_mode from the heartbeat
uint8_t vehicleType = 0;   // MAV_TYPE from the heartbeat
uint32_t lastHeartbeatMs = 0;
uint32_t armedNoticeUntilMs = 0;   // show the ARMED pop-up until this time

bool altWarnReady = false;   // true once above WARN_ALT while armed
bool lowBattActive = false;
bool lowAltActive = false;

// R/C brightness control.
uint16_t rcBrightUs = 0;   // last value of the brightness channel, in us
int currentLevel = (BRIGHT_DEFAULT * BACKLIGHT_STEPS + 127) / 255;  // 0-16
int blChipLevel = 0;       // where we think the backlight chip is (0 = off)

// Demo mode state.
bool demoMode = false;
uint32_t demoStartMs = 0;

uint32_t lastFrame = 0;
uint32_t lastAttitudeMs = 0;
uint32_t attitudeCount = 0;

mavlink_message_t mavMsg;
mavlink_status_t mavStatus;

void receiveMavlink() {
  while (Serial1.available() > 0) {
    uint8_t b = Serial1.read();

    if (mavlink_parse_char(
          MAVLINK_COMM_0, b, &mavMsg, &mavStatus)) {

      if (mavMsg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(&mavMsg, &attitude);

        // MAVLink ATTITUDE angles are in radians.
        rollDeg  = attitude.roll  * RAD_TO_DEG;
        pitchDeg = attitude.pitch * RAD_TO_DEG;

        lastAttitudeMs = millis();
        attitudeCount++;
      } else if (mavMsg.msgid == MAVLINK_MSG_ID_VFR_HUD) {
        mavlink_vfr_hud_t vfr;
        mavlink_msg_vfr_hud_decode(&mavMsg, &vfr);

        // Airspeed in m/s. With no airspeed sensor this is
        // ArduPilot's own estimate.
        airspeedMs = vfr.airspeed;
        lastVfrMs = millis();
      } else if (mavMsg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        mavlink_global_position_int_t pos;
        mavlink_msg_global_position_int_decode(&mavMsg, &pos);

        // relative_alt is millimetres above home.
        homeAltM = pos.relative_alt / 1000.0f;
        lastPosMs = millis();
      } else if (mavMsg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&mavMsg, &hb);

        // Only trust the autopilot's own heartbeat for arm state.
        if (mavMsg.compid == MAV_COMP_ID_AUTOPILOT1 &&
            hb.autopilot != MAV_AUTOPILOT_INVALID) {
          bool nowArmed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
          if (nowArmed && !armed) {
            armedNoticeUntilMs = millis() + ARMED_NOTICE_MS;
          }
          armed = nowArmed;
          flightMode = hb.custom_mode;
          vehicleType = hb.type;
          lastHeartbeatMs = millis();
        }
      } else if (mavMsg.msgid == MAVLINK_MSG_ID_SYS_STATUS) {
        mavlink_sys_status_t sys;
        mavlink_msg_sys_status_decode(&mavMsg, &sys);

        // Total pack voltage in mV (UINT16_MAX = unknown).
        if (sys.voltage_battery != UINT16_MAX) {
          packV = sys.voltage_battery / 1000.0f;
          lastPackMs = millis();
        }
      } else if (mavMsg.msgid == MAVLINK_MSG_ID_BATTERY_STATUS) {
        mavlink_battery_status_t bat;
        mavlink_msg_battery_status_decode(&mavMsg, &bat);

        if (bat.id == BATT_ID) {
          // Total pack voltage is the sum of the entries that are
          // present (UINT16_MAX = not provided). This works whether the
          // flight controller reports individual cells or one total.
          uint32_t sumMv = 0;
          int n = 0;
          for (int i = 0; i < 10; i++) {
            uint16_t mv = bat.voltages[i];
            if (mv != UINT16_MAX && mv > 0) {
              sumMv += mv;
              n++;
            }
          }

          if (n > 0) {
            packV = sumMv / 1000.0f;
            lastPackMs = millis();
          }
        }
      } else if (mavMsg.msgid == MAVLINK_MSG_ID_RC_CHANNELS) {
        mavlink_rc_channels_t rc;
        mavlink_msg_rc_channels_decode(&mavMsg, &rc);

        const uint16_t ch[18] = {
          rc.chan1_raw,  rc.chan2_raw,  rc.chan3_raw,
          rc.chan4_raw,  rc.chan5_raw,  rc.chan6_raw,
          rc.chan7_raw,  rc.chan8_raw,  rc.chan9_raw,
          rc.chan10_raw, rc.chan11_raw, rc.chan12_raw,
          rc.chan13_raw, rc.chan14_raw, rc.chan15_raw,
          rc.chan16_raw, rc.chan17_raw, rc.chan18_raw};

        if (BRIGHT_RC_CHANNEL >= 1 && BRIGHT_RC_CHANNEL <= 18) {
          rcBrightUs = ch[BRIGHT_RC_CHANNEL - 1];
        }
      }
    }
  }
}

// Draw a line of arbitrary thickness with round end caps.
// Built from two filled triangles plus a circle at each end, so there
// are no gaps on diagonals however thick it gets.
void thickLine(int x1, int y1, int x2, int y2,
               float thickness, uint16_t color) {
  float half = thickness * 0.5f;
  int r = (int)(half + 0.5f);
  float dx = (float)(x2 - x1);
  float dy = (float)(y2 - y1);
  float len = sqrtf(dx * dx + dy * dy);

  if (len >= 0.5f) {
    float nx = -dy / len * half;
    float ny =  dx / len * half;

    int ax = (int)lroundf(x1 + nx), ay = (int)lroundf(y1 + ny);
    int bx = (int)lroundf(x2 + nx), by = (int)lroundf(y2 + ny);
    int cx = (int)lroundf(x2 - nx), cy = (int)lroundf(y2 - ny);
    int ex = (int)lroundf(x1 - nx), ey = (int)lroundf(y1 - ny);

    canvas.fillTriangle(ax, ay, bx, by, cx, cy, color);
    canvas.fillTriangle(ax, ay, cx, cy, ex, ey, color);
  }

  if (r > 0) {
    canvas.fillCircle(x1, y1, r, color);
    canvas.fillCircle(x2, y2, r, color);
  }
}

// Thick line with a black outline underneath it.
void markLine(int x1, int y1, int x2, int y2,
              float thickness, uint16_t color) {
  if (OUTLINE > 0) {
    thickLine(x1, y1, x2, y2, thickness + 2 * OUTLINE, BLACK);
  }
  thickLine(x1, y1, x2, y2, thickness, color);
}

// Fixed yellow aircraft symbol. Drawn in two passes (all outlines first,
// then all yellow) so the outline of one segment never cuts into the
// yellow of the next.
void drawAircraftSymbol() {
  for (int pass = 0; pass < 2; pass++) {
    bool outlinePass = (pass == 0);
    if (outlinePass && OUTLINE <= 0) continue;

    uint16_t col = outlinePass ? BLACK : YELLOW;
    float t = T_SYMBOL + (outlinePass ? 2 * OUTLINE : 0);

    // Left half: bar, then the drop at its inner end.
    thickLine(CX - SYMBOL_OUTER, CY, CX - SYMBOL_INNER, CY, t, col);
    thickLine(CX - SYMBOL_INNER, CY,
              CX - SYMBOL_INNER + SYMBOL_SLANT, CY + SYMBOL_DROP, t, col);

    // Right half, mirrored.
    thickLine(CX + SYMBOL_OUTER, CY, CX + SYMBOL_INNER, CY, t, col);
    thickLine(CX + SYMBOL_INNER, CY,
              CX + SYMBOL_INNER - SYMBOL_SLANT, CY + SYMBOL_DROP, t, col);
  }
}

// One boxed numeric readout with a small label underneath.
// leftSide = true puts it at the left edge, false at the right edge.
void drawReadout(bool leftSide, bool valid,
                 float value, const char* label) {
  int bx = leftSide ? BOX_MARGIN : W - BOX_MARGIN - BOX_W;
  int by = CY - BOX_H / 2;

  // Black outer outline, white border, black fill.
  canvas.fillRect(bx - OUTLINE, by - OUTLINE,
                  BOX_W + 2 * OUTLINE, BOX_H + 2 * OUTLINE, BLACK);
  canvas.fillRect(bx, by, BOX_W, BOX_H, WHITE);
  canvas.fillRect(bx + 3, by + 3, BOX_W - 6, BOX_H - 6, BLACK);

  char buf[12];
  if (valid) {
    long v = lroundf(value);
    if (v > 9999) v = 9999;
    if (v < -999) v = -999;
    snprintf(buf, sizeof(buf), "%ld", v);
  } else {
    snprintf(buf, sizeof(buf), "---");
  }

  // Big number, drawn twice 1 px apart to make it bolder.
  canvas.setTextSize(3);
  canvas.setTextColor(valid ? WHITE : YELLOW);
  int tw = canvas.textWidth(buf);
  int tx = bx + (BOX_W - tw) / 2;
  int ty = by + (BOX_H - canvas.fontHeight()) / 2;
  canvas.setCursor(tx, ty);
  canvas.print(buf);
  canvas.setCursor(tx + 1, ty);
  canvas.print(buf);

  // Label under the box, kept fully on screen.
  canvas.setTextSize(LABEL_TEXT_SIZE);
  canvas.setTextColor(YELLOW, BLACK);
  int lw = canvas.textWidth(label);
  int lx = bx + (BOX_W - lw) / 2;
  if (lx < 0) lx = 0;
  if (lx + lw > W) lx = W - lw;
  canvas.setCursor(lx, by + BOX_H + OUTLINE + 2);
  canvas.print(label);
}

// Airspeed on the left, altitude above home on the right.
void drawReadouts() {
  uint32_t now = millis();
  bool speedOk = lastVfrMs != 0 && now - lastVfrMs < DATA_TIMEOUT_MS;
  bool altOk   = lastPosMs != 0 && now - lastPosMs < DATA_TIMEOUT_MS;

  drawReadout(true, speedOk,
              airspeedMs * SPEED_FACTORS[SPEED_UNIT],
              SPEED_UNIT_NAMES[SPEED_UNIT]);
  drawReadout(false, altOk,
              homeAltM * ALT_FACTORS[ALT_UNIT],
              ALT_UNIT_NAMES[ALT_UNIT]);
}

// Decide which warnings are active. Uses hysteresis so a warning
// doesn't flicker when a value hovers right at its limit.
void updateWarnings() {
  uint32_t now = millis();

  // --- Low battery (checked whether or not the plane is armed) ---
  bool packFresh = lastPackMs != 0 && now - lastPackMs < DATA_TIMEOUT_MS;

  if (WARN_PACK_V > 0.0f && packFresh) {
    if (packV < WARN_PACK_V) {
      lowBattActive = true;
    } else if (packV > WARN_PACK_V * (1.0f + WARN_HYST)) {
      lowBattActive = false;
    }
  } else {
    lowBattActive = false;
  }

  // --- Low altitude ---
  // Only active while armed, and only after the plane has first been
  // above the limit, so it doesn't fire on the ground or during launch.
  bool armedNow = armed && lastHeartbeatMs != 0 &&
                  now - lastHeartbeatMs < HEARTBEAT_TIMEOUT_MS;
  bool altFresh = lastPosMs != 0 && now - lastPosMs < DATA_TIMEOUT_MS;

  if (!armedNow) {
    altWarnReady = false;
    lowAltActive = false;
  } else if (altFresh && WARN_ALT > 0.0f) {
    float alt = homeAltM * ALT_FACTORS[ALT_UNIT];

    if (alt >= WARN_ALT) altWarnReady = true;

    if (altWarnReady) {
      if (alt < WARN_ALT) {
        lowAltActive = true;
      } else if (alt > WARN_ALT * (1.0f + WARN_HYST)) {
        lowAltActive = false;
      }
    }
  } else {
    lowAltActive = false;
  }
}

// Red WARNING pop-up, centered in the upper half of the screen.
void drawWarning() {
  bool showBatt = lowBattActive;
  bool showAlt = lowAltActive;
  bool showArmed = (int32_t)(armedNoticeUntilMs - millis()) > 0;
  if (!showBatt && !showAlt && !showArmed) return;

  char buf[24];

  if (showBatt || showAlt) {
    // A real warning takes priority over the ARMED notice.

    // One full blink cycle, in ms.
    uint32_t blinkPeriod = WARN_BLINK_ON_MS + WARN_BLINK_OFF_MS;
    if (blinkPeriod == 0) blinkPeriod = 1;

    // If both are active, alternate between them (two blinks each).
    if (showBatt && showAlt) {
      if ((millis() / (2 * blinkPeriod)) % 2 == 0) showAlt = false;
      else showBatt = false;
    }

    // Blink: hidden during the "off" part of each cycle.
    if (WARN_BLINK_OFF_MS > 0 &&
        millis() % blinkPeriod >= WARN_BLINK_ON_MS) {
      return;
    }

    if (showBatt) {
      snprintf(buf, sizeof(buf), "BATT %.1fV", packV);
    } else {
      snprintf(buf, sizeof(buf), "ALT %ld %s",
               lroundf(homeAltM * ALT_FACTORS[ALT_UNIT]),
               ALT_UNIT_NAMES[ALT_UNIT]);
    }
  } else {
    // Just armed: a steady (non-blinking) notice.
    snprintf(buf, sizeof(buf), "ARMED");
  }

  int bx = (W - WARN_W) / 2;
  int by = WARN_Y;
  int totalH = WARN_HDR_H + WARN_BODY_H;

  // Black outline, red box, then a black panel for the body text.
  canvas.fillRect(bx - OUTLINE, by - OUTLINE,
                  WARN_W + 2 * OUTLINE, totalH + 2 * OUTLINE, BLACK);
  canvas.fillRect(bx, by, WARN_W, totalH, RED);
  canvas.fillRect(bx + 3, by + WARN_HDR_H,
                  WARN_W - 6, WARN_BODY_H - 3, BLACK);

  // "WARNING" header: white on red, drawn twice 1 px apart for weight.
  canvas.setTextColor(WHITE);
  canvas.setTextSize(3);
  const char* hdr = "WARNING";
  int hw = canvas.textWidth(hdr);
  int hx = bx + (WARN_W - hw) / 2;
  int hy = by + (WARN_HDR_H - canvas.fontHeight()) / 2;
  canvas.setCursor(hx, hy);
  canvas.print(hdr);
  canvas.setCursor(hx + 1, hy);
  canvas.print(hdr);

  // Value text: white on black. Drops to a smaller size if it's too wide.
  canvas.setTextSize(3);
  if (canvas.textWidth(buf) > WARN_W - 12) canvas.setTextSize(2);
  int tw = canvas.textWidth(buf);
  int tx = bx + (WARN_W - tw) / 2;
  int ty = by + WARN_HDR_H + (WARN_BODY_H - 3 - canvas.fontHeight()) / 2;
  canvas.setCursor(tx, ty);
  canvas.print(buf);
  canvas.setCursor(tx + 1, ty);
  canvas.print(buf);
}

// ArduPlane flight mode name from the heartbeat's custom_mode.
// Other vehicle types or unknown modes show as "MODE n".
const char* flightModeName(uint32_t mode, uint8_t type,
                           char* buf, size_t n) {
  // MAV_TYPE 1 = fixed wing, 19-25 = the VTOL types ArduPlane can use.
  bool isPlane = (type == 1) || (type >= 19 && type <= 25);

  if (isPlane) {
    switch (mode) {
      case 0:  return "MANUAL";
      case 1:  return "CIRCLE";
      case 2:  return "STABILIZE";
      case 3:  return "TRAINING";
      case 4:  return "ACRO";
      case 5:  return "FBWA";
      case 6:  return "FBWB";
      case 7:  return "CRUISE";
      case 8:  return "AUTOTUNE";
      case 10: return "AUTO";
      case 11: return "RTL";
      case 12: return "LOITER";
      case 13: return "TAKEOFF";
      case 14: return "AVOID ADSB";
      case 15: return "GUIDED";
      case 17: return "QSTABILIZE";
      case 18: return "QHOVER";
      case 19: return "QLOITER";
      case 20: return "QLAND";
      case 21: return "QRTL";
      case 22: return "QAUTOTUNE";
      case 23: return "QACRO";
      case 24: return "THERMAL";
      case 25: return "LOITALTQLAND";
      case 26: return "AUTOLAND";
      default: break;
    }
  }

  snprintf(buf, n, "MODE %lu", (unsigned long)mode);
  return buf;
}

void drawHorizon(float roll, float pitch) {
  float a = -roll * DEG_TO_RAD;
  float s = sinf(a);
  float c = cosf(a);
  float horizonY = CY + pitch * PITCH_PIXELS_PER_DEGREE;

  // Fill each scanline on the correct side of the horizon.
  for (int y = 0; y < H; y++) {
    float dy = y - horizonY;

    if (fabsf(s) < 0.001f) {
      canvas.drawFastHLine(
        0, y, W, dy >= 0 ? GROUND : SKY);
    } else {
      float xCross = CX + dy * c / s;
      int split = (int)floorf(xCross);

      if (s > 0) {
        if (split <= 0) {
          canvas.drawFastHLine(0, y, W, SKY);
        } else if (split >= W) {
          canvas.drawFastHLine(0, y, W, GROUND);
        } else {
          canvas.drawFastHLine(0, y, split, GROUND);
          canvas.drawFastHLine(split, y, W - split, SKY);
        }
      } else {
        if (split <= 0) {
          canvas.drawFastHLine(0, y, W, GROUND);
        } else if (split >= W) {
          canvas.drawFastHLine(0, y, W, SKY);
        } else {
          canvas.drawFastHLine(0, y, split, SKY);
          canvas.drawFastHLine(split, y, W - split, GROUND);
        }
      }
    }
  }

  // Horizon line. Endpoints are run well past both screen edges along
  // the roll direction (the sprite clips them), which also avoids the
  // divide-by-cos(roll) problem at +/-90 degrees of bank.
  const float L = (float)W;
  markLine((int)(CX - L * c), (int)(horizonY - L * s),
           (int)(CX + L * c), (int)(horizonY + L * s),
           T_HORIZON, WHITE);

  // Pitch ladder.
  for (int p = -LADDER_MAX_DEG; p <= LADDER_MAX_DEG; p += LADDER_STEP_DEG) {
    if (p == 0) continue;

    float offset = -p * PITCH_PIXELS_PER_DEGREE;

    int x1 = (int)(CX - 25 * c - offset * s);
    int yy1 = (int)(horizonY - 25 * s + offset * c);
    int x2 = (int)(CX + 25 * c - offset * s);
    int yy2 = (int)(horizonY + 25 * s + offset * c);

    markLine(x1, yy1, x2, yy2, T_LADDER, WHITE);
  }

  // Fixed yellow aircraft symbol.
  drawAircraftSymbol();

  // Bank reference ticks.
  markLine(CX, 5, CX, 19, T_BANK, WHITE);
  markLine(CX - 42, 10, CX - 33, 22, T_BANK, WHITE);
  markLine(CX + 42, 10, CX + 33, 22, T_BANK, WHITE);
  markLine(CX - 70, 23, CX - 59, 31, T_BANK, WHITE);
  markLine(CX + 70, 23, CX + 59, 31, T_BANK, WHITE);

  // Airspeed (left) and altitude above home (right).
  drawReadouts();

  // Status line, centered at the bottom: the flight mode (with
  // "DISARMED" in front of it while disarmed), or a waiting message if
  // the autopilot heartbeat is lost. Black background so it stays
  // readable over both sky and ground.
  char status[40];
  int statusSize = STATUS_SIZE_DISARMED;
  if (lastHeartbeatMs != 0 &&
      millis() - lastHeartbeatMs < HEARTBEAT_TIMEOUT_MS) {
    char modeBuf[16];
    const char* mode = flightModeName(flightMode, vehicleType,
                                      modeBuf, sizeof(modeBuf));
    if (armed) {
      // Armed: just the flight mode, in the larger size.
      snprintf(status, sizeof(status), "%s", mode);
      statusSize = STATUS_SIZE_ARMED;
    } else {
      snprintf(status, sizeof(status), "DISARMED - %s", mode);
    }
  } else {
    snprintf(status, sizeof(status), "Waiting for MAVLINK");
  }

  canvas.setTextColor(YELLOW, BLACK);

  // Use the chosen size, but step down if the text wouldn't fit.
  canvas.setTextSize(statusSize);
  while (statusSize > 1 && canvas.textWidth(status) > W) {
    statusSize--;
    canvas.setTextSize(statusSize);
  }

  int sw = canvas.textWidth(status);
  canvas.setCursor((W - sw) / 2, H - canvas.fontHeight() - 4);
  canvas.print(status);

  // Small tag so it's obvious the data is simulated.
  if (demoMode) {
    canvas.setTextSize(2);
    canvas.setTextColor(YELLOW, BLACK);
    canvas.setCursor(4, 4);
    canvas.print("DEMO");
  }

  // Warning pop-up goes on top of everything else.
  updateWarnings();
  drawWarning();
}

// ---- Backlight chip ------------------------------------------------------
// The T-Display-S3's backlight chip (an AW9364) isn't a PWM dimmer. It
// has 16 brightness steps, set by pulses on one pin (GPIO38). Each pulse
// steps the brightness one level down, and after the lowest level it
// wraps back to full. Holding the pin low for a few milliseconds shuts
// the chip off, and it wakes up at full brightness.

// Take the backlight pin over from the library's PWM.
void backlightInit() {
  pinMatrixOutDetach(BACKLIGHT_PIN, false, false);
  pinMode(BACKLIGHT_PIN, OUTPUT);
  digitalWrite(BACKLIGHT_PIN, LOW);
  delay(5);                  // long enough to shut the chip down
  blChipLevel = 0;
}

// Set the backlight to a level from 0 (off) to 16 (full brightness).
void backlightSetLevel(int level) {
  if (level < 0) level = 0;
  if (level > BACKLIGHT_STEPS) level = BACKLIGHT_STEPS;
  if (level == blChipLevel) return;

  if (level == 0) {
    digitalWrite(BACKLIGHT_PIN, LOW);   // shut the chip down
    blChipLevel = 0;
    return;
  }

  // Waking from off, or going to full brightness: restart the chip so
  // it is certain to be at full brightness. This also corrects any
  // miscount, and costs only a few milliseconds of darkness.
  if (blChipLevel == 0 || level == BACKLIGHT_STEPS) {
    digitalWrite(BACKLIGHT_PIN, LOW);
    delay(5);
    digitalWrite(BACKLIGHT_PIN, HIGH);
    delayMicroseconds(100);
    blChipLevel = BACKLIGHT_STEPS;
  }

  // Each pulse steps one level down (wrapping from 1 back to full).
  int pulses = (blChipLevel - level + BACKLIGHT_STEPS) % BACKLIGHT_STEPS;
  noInterrupts();
  for (int i = 0; i < pulses; i++) {
    digitalWrite(BACKLIGHT_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(BACKLIGHT_PIN, HIGH);
    delayMicroseconds(2);
  }
  interrupts();
  blChipLevel = level;
}

// Map the R/C channel to screen brightness.
void updateBrightness() {
  if (BRIGHT_RC_CHANNEL < 1 || BRIGHT_RC_CHANNEL > 18) return;
  if (BRIGHT_RC_HIGH_US == BRIGHT_RC_LOW_US) return;

  // No data, an unused channel (65535) or a failsafe zero: hold the
  // current brightness.
  if (rcBrightUs < 800 || rcBrightUs > 2200) return;

  float t = (float)((int)rcBrightUs - BRIGHT_RC_LOW_US) /
            (float)(BRIGHT_RC_HIGH_US - BRIGHT_RC_LOW_US);
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;

  // Brightness curve, then scale into the BRIGHT_MIN..BRIGHT_MAX range.
  float shaped = powf(t, BRIGHT_GAMMA);
  float target = BRIGHT_MIN + shaped * (BRIGHT_MAX - BRIGHT_MIN);  // 0-255

  // The two ends of the channel always apply, so full range is reachable
  // even with a little noise on the channel value.
  bool atEnd = (t <= 0.01f || t >= 0.99f);

  // 16 steps. Hysteresis stops the level hopping back and forth when
  // the channel value hovers on the edge between two steps.
  const float LEVEL_HYST = 0.2f;
  float levelF = target * BACKLIGHT_STEPS / 255.0f;
  int level = (int)lroundf(levelF);

  if (level != currentLevel &&
      (atEnd || fabsf(levelF - currentLevel) > 0.5f + LEVEL_HYST)) {
    currentLevel = level;
    backlightSetLevel(currentLevel);
    Serial.printf("Backlight: R/C %u us -> level %d/%d\n",
                  (unsigned)rcBrightUs, currentLevel, BACKLIGHT_STEPS);
  }
}

// Start or stop the demo. Stopping also forgets the simulated data, so
// the display goes back to waiting for real telemetry.
void setDemoMode(bool on) {
  demoMode = on;
  demoStartMs = millis();

  altWarnReady = false;
  lowBattActive = false;
  lowAltActive = false;
  armedNoticeUntilMs = 0;

  if (!on) {
    rollDeg = 0.0f;
    pitchDeg = 0.0f;
    airspeedMs = 0.0f;
    homeAltM = 0.0f;
    packV = 0.0f;
    armed = false;
    flightMode = 0;
    vehicleType = 0;
    lastAttitudeMs = 0;
    lastVfrMs = 0;
    lastPosMs = 0;
    lastPackMs = 0;
    lastHeartbeatMs = 0;
  }

  Serial.println(on ? "Demo mode ON" : "Demo mode OFF");
}

// Fake flight: rolling and pitching, changing speed and altitude, flight
// modes cycling, and altitude and battery that dip below your warning
// limits so both pop-ups show up. Warning levels come from the settings
// at the top of the file.
void updateDemo(uint32_t now) {
  const float TAU = 6.2831853f;
  float t = (now - demoStartMs) / 1000.0f;

  rollDeg    = 40.0f * sinf(TAU * t / 9.0f);
  pitchDeg   = 12.0f * sinf(TAU * t / 6.0f + 1.0f);
  airspeedMs = 27.0f + 8.0f * sinf(TAU * t / 13.0f);

  // Altitude swings between 0.5x and 4.5x the low-altitude limit.
  float altLimitM = (WARN_ALT > 0.0f)
                      ? WARN_ALT / ALT_FACTORS[ALT_UNIT] : 50.0f;
  homeAltM = altLimitM * (2.5f + 2.0f * cosf(TAU * t / 30.0f));

  // Pack voltage swings between about 0.98x and 1.26x the limit.
  float battLimitV = (WARN_PACK_V > 0.0f) ? WARN_PACK_V : 20.0f;
  packV = battLimitV * (1.12f + 0.14f * cosf(TAU * t / 45.0f));

  // Disarmed for the first few seconds, then armed. The flight mode
  // changes every 8 seconds: FBWA, STABILIZE, MANUAL, AUTO.
  static const uint32_t demoModes[] = {5, 2, 0, 10};
  bool nowArmed = t > 3.0f;
  if (nowArmed && !armed) armedNoticeUntilMs = now + ARMED_NOTICE_MS;
  armed = nowArmed;
  vehicleType = 1;   // fixed wing
  flightMode = demoModes[((uint32_t)(t / 8.0f)) % 4];

  // Everything counts as freshly received.
  lastAttitudeMs = now;
  lastVfrMs = now;
  lastPosMs = now;
  lastPackMs = now;
  lastHeartbeatMs = now;
}

// Debounced IO14 button: each press toggles demo mode.
void pollButton() {
  static bool lastRaw = HIGH;
  static bool stable = HIGH;
  static uint32_t changedMs = 0;

  bool raw = digitalRead(BUTTON_PIN);
  uint32_t now = millis();

  if (raw != lastRaw) {
    lastRaw = raw;
    changedMs = now;
  }

  if (raw != stable && now - changedMs >= BUTTON_DEBOUNCE_MS) {
    stable = raw;
    if (stable == LOW) setDemoMode(!demoMode);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  tft.begin(1);
  backlightInit();
  backlightSetLevel(currentLevel);

  canvas.setColorDepth(16);

  if (!canvas.createSprite(W, H)) {
    Serial.println("Sprite allocation failed!");
    while (true) delay(1000);
  }

  // RX receives flight-controller TX.
  // TX is initialized but not used to send data.
  Serial1.begin(
    MAV_BAUD, SERIAL_8N1, MAV_RX_PIN, MAV_TX_PIN);

  Serial.println("MAVLink attitude display ready");
}

void loop() {
  pollButton();

  if (demoMode) {
    // Ignore real telemetry while the demo is running.
    while (Serial1.available() > 0) Serial1.read();
  } else {
    receiveMavlink();
  }

  uint32_t now = millis();

  if (now - lastFrame < FRAME_INTERVAL_MS) return;
  lastFrame = now;

  if (demoMode) updateDemo(now);

  // Hold the last attitude briefly, then level the horizon
  // if telemetry disappears. This is only a visual fallback.
  if (lastAttitudeMs == 0 ||
      now - lastAttitudeMs > ATTITUDE_TIMEOUT_MS) {
    rollDeg = 0;
    pitchDeg = 0;
  }

  updateBrightness();
  drawHorizon(rollDeg, pitchDeg);
  canvas.pushSprite(0, 0);

  // Debug information over USB Serial.
  static uint32_t lastDebug = 0;

  if (now - lastDebug >= 1000) {
    lastDebug = now;

    Serial.printf(
      "Attitude messages: %lu, roll: %.1f, pitch: %.1f\n",
      (unsigned long)attitudeCount,
      rollDeg, pitchDeg);
  }
}
