/**
 * Hardware:
 *   ESP32, AP33772S PD sink, INA226, SH1106 128x64 OLED,
 *   Rotary encoder (CLK/DT/SW), optional SW1, optional SW2, MOSFET
 *
 * ENCODER-ONLY OPERATION (buttons are optional shortcuts):
 *   Main screen:
 *     Rotate       → adjust selected field value
 *     Short press  → cycle field: PROFILE→VSET→ILIM→OUTPUT→MENU→…
 *     Long press   → PROFILE: apply profile
 *                    VSET/ILIM: apply PPS settings
 *                    OUTPUT: toggle ON/OFF
 *                    MENU: open menu
 *
 *   Menu / sub-screens:
 *     Rotate       → move cursor / scroll
 *     Short press  → enter item / toggle / advance sub-field
 *     Long press   → back / apply & return
 *
 * Optional shortcuts:
 *   SW1 short  → toggle VSET / ILIM on main
 *   SW1 long   → open menu
 *   SW2 short  → toggle output ON/OFF
 *   SW2 long   → jump to session screen
 */

// ============================================================
//  INCLUDES
// ============================================================
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "AP33772S.h"
#include <INA226.h>

// ============================================================
//  USER CONFIG — edit before flashing
// ============================================================
#define WIFI_SSID        "ESP"
#define WIFI_PASSWORD    "abcd1234"
#define AIO_SERVER       "io.adafruit.com"
#define AIO_PORT         1883
#define AIO_USERNAME     "Rau7han"
#define AIO_KEY          "aio_wxZc81qgBirIGZ7VoX7LTOo6VrA9"
#define AIO_FEED_PREFIX   AIO_USERNAME "/feeds/"

// ============================================================
//  PIN MAP
// ============================================================
static constexpr uint8_t PIN_SDA         = 21;
static constexpr uint8_t PIN_SCL         = 22;
static constexpr int8_t  PIN_INT         = 32;
static constexpr uint8_t PIN_LED         = 4;
static constexpr uint8_t PIN_MOSFET      = 25;
static constexpr uint8_t PIN_ENC_CLK     = 5;
static constexpr uint8_t PIN_ENC_DT      = 18;
static constexpr uint8_t PIN_ENC_SW      = 19;
static constexpr uint8_t PIN_SW1         = 17;   // optional
static constexpr uint8_t PIN_SW2         = 16;   // optional

static constexpr uint8_t ADDR_AP         = 0x52;
static constexpr uint8_t ADDR_INA        = 0x40;
static constexpr uint8_t ADDR_OLED       = 0x3C;

// ============================================================
//  CONSTANTS
// ============================================================
static constexpr uint16_t PPS_DEFAULT_MV  = 5000;
static constexpr float    INA_SHUNT_OHM   = 0.005f;
static constexpr float    INA_MAX_AMPS    = 6.0f;
static constexpr float    OVP_LIMIT_V     = 22.0f;
static constexpr float    OCP_LIMIT_A     = 5.5f;
static constexpr int8_t   OTP_LIMIT_C     = 80;

// Timing (ms)
static constexpr uint32_t T_DEBOUNCE      = 30;
static constexpr uint32_t T_LONGPRESS     = 650;
static constexpr uint32_t T_DISPLAY       = 60;
static constexpr uint32_t T_MEASURE       = 100;
static constexpr uint32_t T_HOTPLUG       = 800;
static constexpr uint32_t T_PPS_KEEPALIVE = 4500;
static constexpr uint32_t T_PDO_POLL      = 200;
static constexpr uint32_t T_PDO_TIMEOUT   = 6000;
static constexpr uint32_t T_NEGO_WAIT     = 1500;
static constexpr uint32_t T_WIFI_RETRY    = 15000;
static constexpr uint32_t T_MQTT_RETRY    = 5000;
static constexpr uint32_t T_CLOUD_DEF     = 15000;
static constexpr uint32_t T_ANALYZER_REFRESH = 2000;  // PD Analyzer refresh interval

// Display EMA smoothing
static constexpr float    EMA_K           = 6.0f;

// Step tables
static const uint16_t V_STEPS[]    = { 20, 100, 1000 };
static constexpr uint8_t V_STEP_N  = 3;
static const uint16_t I_STEPS[]    = { 50, 100, 500 };
static constexpr uint8_t I_STEP_N  = 3;
static const uint32_t CLOUD_RATES[]   = { 15000, 30000, 60000 };
static constexpr uint8_t CLOUD_RATE_N = 3;

// ============================================================
//  ENUMS
// ============================================================
enum class Screen : uint8_t {
    MAIN = 0, MENU,
    PPS_ADJUST, PDO_SELECT, PD_ANALYZER,
    SESSION, CLOUD, SAFETY, SYSTEM,
    FAULT, DISCONNECTED, DETECTING
};

// Fields cycled on MAIN by short-press
enum class MainField : uint8_t {
    FIELD_PROFILE = 0,
    FIELD_VSET,
    FIELD_ILIM,
    FIELD_OUTPUT,
    FIELD_MENU,
    FIELD_COUNT
};

enum class SysMode : uint8_t {
    DISCONNECTED, DETECTING, LEGACY, PD_FIXED, PPS, FAULT
};

// ============================================================
//  STATE STRUCTS
// ============================================================
struct SysState {
    SysMode  mode           = SysMode::DISCONNECTED;
    bool     ppsAvail       = false;

    uint8_t  fixedIdx[AP33772S_MAX_PDO] = {};
    uint8_t  fixedCount     = 0;
    uint8_t  fixedSel       = 0;

    uint8_t  ppsPdoIdx      = 0;
    uint16_t ppsSetV_mV     = PPS_DEFAULT_MV;
    uint16_t ppsSetI_mA     = 3000;
    uint16_t ppsMinV_mV     = 3300;
    uint16_t ppsMaxV_mV     = 21000;
    uint16_t ppsMaxI_mA     = 3000;
    uint16_t negoV_mV       = 0;
    uint16_t negoI_mA       = 0;

    // Raw values
    float    rawV           = 0.0f;
    float    rawI           = 0.0f;
    float    rawP           = 0.0f;

    // EMA-smoothed values for OLED
    float    dispV          = 0.0f;
    float    dispI          = 0.0f;
    float    dispP          = 0.0f;

    int8_t   temp_C         = 25;
    bool     outputReq      = false;
    bool     mosfetOn       = false;
    bool     fault          = false;
    char     faultMsg[32]   = "";
    uint8_t  faultCount     = 0;
    uint8_t  negoFails      = 0;
    bool     ovpEn          = true;
    bool     otpEn          = true;
} sys;

struct UIState {
    Screen    screen        = Screen::DETECTING;
    MainField mainField     = MainField::FIELD_PROFILE;

    // Profile cursor for home screen: 0..fixedCount-1 = fixed, fixedCount = PPS
    uint8_t   profileCursor = 0;

    uint8_t   cursor        = 0;
    uint8_t   scrollOff     = 0;

    // PPS Adjust: 0=VSET 1=ILIM 2=V STEP 3=I STEP
    uint8_t   ppsField      = 0;

    uint8_t   vStepIdx      = 1;   // 100mV default
    uint8_t   iStepIdx      = 1;   // 100mA default
    uint8_t   cloudRateIdx  = 1;   // 2s default

    // Safety cursor: 0=OVP 1=OTP
    uint8_t   safetyCursor  = 0;

    // PD Analyzer
    uint8_t   analyzerRow   = 0;
    bool      analyzerDetail = false;
} ui;

struct Session {
    uint32_t startMs       = 0;
    uint32_t lastSampleMs  = 0;
    double   wh            = 0.0;
    double   ah            = 0.0;
    float    peakW         = 0.0f;
    float    peakA         = 0.0f;
} sess;

struct CloudState {
    bool     wifiOk        = false;
    bool     mqttOk        = false;
    uint32_t lastTxMs      = 0;
    uint32_t interval      = T_CLOUD_DEF;
    char     ipStr[16]     = "0.0.0.0";
} cloud;

// Menu items — order must match MAP[] in handleMenu()
static const char* const MENU_ITEMS[] = {
    "PPS Adjust", "PDO Select", "PD Analyzer",
    "Session", "Cloud", "Safety", "System"
};
static constexpr uint8_t MENU_COUNT = 7;

// ============================================================
//  PERIPHERALS
// ============================================================
AP33772S pd(Wire, PIN_INT);
INA226   ina(ADDR_INA, &Wire);
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE);
Preferences prefs;
WiFiClient  wifiClient;
PubSubClient mqtt(wifiClient);

// ============================================================
//  BUTTON
// ============================================================
struct Button {
    uint8_t  pin;
    bool     lastRaw   = false;
    bool     state     = false;
    bool     longFired = false;
    uint32_t bounceMs  = 0;
    uint32_t pressMs   = 0;

    void begin(uint8_t p) { pin = p; pinMode(p, INPUT_PULLUP); }

    uint8_t poll() {
        bool raw = (digitalRead(pin) == LOW);
        uint32_t now = millis();
        if (raw != lastRaw) { lastRaw = raw; bounceMs = now; }
        if (now - bounceMs < T_DEBOUNCE) return 0;
        if (raw != state) {
            state = raw;
            if (state) { pressMs = now; longFired = false; }
            else if (!longFired) return 1;
        } else if (state && !longFired && now - pressMs >= T_LONGPRESS) {
            longFired = true; return 2;
        }
        return 0;
    }
};

Button encSw, sw1, sw2;

// ============================================================
//  ENCODER
// ============================================================
static const int8_t ENC_TABLE[] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
volatile uint8_t encRaw = 0;
volatile int16_t encAcc = 0;

void IRAM_ATTR isrEnc() {
    encRaw = (encRaw << 2) | ((digitalRead(PIN_ENC_CLK) << 1) | digitalRead(PIN_ENC_DT));
    encAcc += ENC_TABLE[encRaw & 0x0F];
}

int readEncoder() {
    noInterrupts(); int16_t acc = encAcc; interrupts();
    int det = acc / 4;
    if (!det) return 0;
    noInterrupts(); encAcc -= det * 4; interrupts();
    return det;
}

// ============================================================
//  SOFT TIMER
// ============================================================
struct SoftTimer {
    uint32_t last = 0;
    bool due(uint32_t p) {
        if (millis() - last >= p) { last = millis(); return true; }
        return false;
    }
};
SoftTimer tmDisplay, tmMeasure, tmHotplug, tmPPS, tmCloud, tmWifi, tmMqtt, tmAnalyzer;

// ============================================================
//  FORWARD DECLARATIONS
// ============================================================
void pdInit(); bool pdWaitPDOs(uint32_t t); void pdDetect(); void pdApply(bool smooth);
void mosfetUpdate(); void faultTrigger(const char* m); void faultClear();
void safetyCheck(); void sessionUpdate(); void nvLoad(); void nvSave();
void cloudConnect();
void cloudPublish();
void mqttCallback(char* topic, byte* payload, unsigned int length);


// Profile helpers
uint8_t  profileTotal();
bool     profileCursorIsPPS();
void     syncProfileCursorFromActiveMode();
void     applyProfileCursor();
void     formatProfileLabel(char* buf, size_t len, uint8_t cursor);
void     formatActiveProfileLabel(char* buf, size_t len);

// Render functions
void renderMain(); void renderMenu(); void renderPPSAdjust(); void renderPDOSelect();
void renderPDAnalyzer();
void renderSession(); void renderCloud(); void renderSafety(); void renderSystem();
void renderFault(); void renderDisconnected(); void renderDetecting();

// Input handlers
void handleMain(int, uint8_t, uint8_t, uint8_t);
void handleMenu(int, uint8_t, uint8_t, uint8_t);
void handlePPSAdjust(int, uint8_t, uint8_t, uint8_t);
void handlePDOSelect(int, uint8_t, uint8_t, uint8_t);
void handlePDAnalyzer(int, uint8_t, uint8_t, uint8_t);
void handleSession(int, uint8_t, uint8_t, uint8_t);
void handleCloud(int, uint8_t, uint8_t, uint8_t);
void handleSafety(int, uint8_t, uint8_t, uint8_t);
void handleSystem(int, uint8_t, uint8_t, uint8_t);

// Display helpers
void setFontS(); void setFontM(); void setFontL();
void dStr(uint8_t x, uint8_t y, const char* s);
void dStrC(uint8_t y, const char* s);
void dStrR(uint8_t y, const char* s);
void dHLine(uint8_t y);
void dHighlightRow(uint8_t y_top, uint8_t h);
void dHeader(const char* title, const char* right);
void dFooter(const char* msg);
void dBadge(uint8_t x, uint8_t y, uint8_t w, const char* label, bool filled);
void dRow(uint8_t yb, const char* left, const char* right, bool selected);
void dScrollBar(uint8_t total, uint8_t visible, uint8_t offset);
void dMeterBar(uint8_t x, uint8_t y, uint8_t w, uint8_t h, float fraction);
const char* modeLabelShort();
const char* mainFieldTitle();
float activeCurrentLimitA();

// ============================================================
//  PROFILE HELPERS
// ============================================================

uint8_t profileTotal() {
    return sys.fixedCount + (sys.ppsAvail ? 1 : 0);
}

bool profileCursorIsPPS() {
    return sys.ppsAvail && (ui.profileCursor == sys.fixedCount);
}

void syncProfileCursorFromActiveMode() {
    uint8_t total = profileTotal();

    if (total == 0) {
        ui.profileCursor = 0;
        return;
    }

    if (sys.mode == SysMode::PPS && sys.ppsAvail) {
        ui.profileCursor = sys.fixedCount;
        return;
    }

    if (sys.mode == SysMode::PD_FIXED && sys.fixedSel < sys.fixedCount) {
        ui.profileCursor = sys.fixedSel;
        return;
    }

    ui.profileCursor = 0;
}


void applyProfileCursor() {
    if (profileCursorIsPPS()) {
        sys.mode = SysMode::PPS;
        sys.ppsSetV_mV = constrain(sys.ppsSetV_mV, sys.ppsMinV_mV, sys.ppsMaxV_mV);
        sys.ppsSetI_mA = constrain(sys.ppsSetI_mA, (uint16_t)1000u, sys.ppsMaxI_mA);
        pdApply(false);
    } else if (ui.profileCursor < sys.fixedCount) {
        sys.fixedSel = ui.profileCursor;
        sys.mode = SysMode::PD_FIXED;
        pdApply(false);
    }
}

void formatProfileLabel(char* buf, size_t len, uint8_t cursor) {
    uint8_t total = profileTotal();
    if (total == 0) {
        snprintf(buf, len, "NO PD");
        return;
    }
    if (sys.ppsAvail && cursor == sys.fixedCount) {
        // Preview PPS
        snprintf(buf, len, "PPS %.2fV", sys.ppsSetV_mV / 1000.0f);
    } else if (cursor < sys.fixedCount) {
        uint8_t pi = sys.fixedIdx[cursor];
        const AP33772S_PDO &p = pd.getPDO(pi);
        float v = p.maxVoltage_mV / 1000.0f;
        // Use minimal decimal places: show .0 only if not whole
        if ((uint32_t)(v * 10) % 10 == 0) {
            snprintf(buf, len, "FIX %.0fV", v);
        } else {
            snprintf(buf, len, "FIX %.1fV", v);
        }
    } else {
        snprintf(buf, len, "---");
    }
}

void formatActiveProfileLabel(char* buf, size_t len) {
    if (sys.mode == SysMode::PPS) {
        snprintf(buf, len, "PPS %.2fV", sys.ppsSetV_mV / 1000.0f);
    } else if (sys.mode == SysMode::PD_FIXED && sys.fixedCount > 0) {
        uint8_t pi = sys.fixedIdx[sys.fixedSel];
        const AP33772S_PDO &p = pd.getPDO(pi);
        float v = p.maxVoltage_mV / 1000.0f;
        if ((uint32_t)(v * 10) % 10 == 0) {
            snprintf(buf, len, "FIX %.0fV", v);
        } else {
            snprintf(buf, len, "FIX %.1fV", v);
        }
    } else if (sys.mode == SysMode::LEGACY) {
        snprintf(buf, len, "LEG 5V");
    } else {
        snprintf(buf, len, "NO PD");
    }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(200);

    pinMode(PIN_LED,    OUTPUT); digitalWrite(PIN_LED, LOW);
    pinMode(PIN_MOSFET, OUTPUT); digitalWrite(PIN_MOSFET, LOW);

    encSw.begin(PIN_ENC_SW);
    sw1.begin(PIN_SW1);
    sw2.begin(PIN_SW2);

    pinMode(PIN_ENC_CLK, INPUT_PULLUP);
    pinMode(PIN_ENC_DT,  INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), isrEnc, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_DT),  isrEnc, CHANGE);

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000UL);

    // Boot splash
    u8g2.setI2CAddress(ADDR_OLED << 1);
    u8g2.begin();
    u8g2.setContrast(220);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_10x20_tf);
    int16_t bw = u8g2.getStrWidth("PowerPD");
    u8g2.drawStr((128 - bw) / 2, 26, "PowerPD");
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(18, 40, "PD/PPS Bench Supply");
    u8g2.drawStr(36, 52, "v6.0  ESP32");
    u8g2.sendBuffer();
    delay(700);

    // INA226
    if (ina.begin()) {
        ina.setMaxCurrentShunt(INA_MAX_AMPS, INA_SHUNT_OHM);
        ina.setAverage(4);
        ina.setMode(7);
    }

    // AP33772S presence check
    Wire.beginTransmission(ADDR_AP);
    if (Wire.endTransmission() != 0) {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(4, 32, "PD chip missing!");
        u8g2.sendBuffer();
        while (1) delay(100);
    }

    nvLoad();

    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    mqtt.setServer(AIO_SERVER, AIO_PORT);
mqtt.setKeepAlive(30);
mqtt.setCallback(mqttCallback);

    pdInit();
    uint8_t op = pd.getOpMode();
    if ((op & OPMODE_PDMOD) || (op & OPMODE_LGCYMOD)) {
        if (pdWaitPDOs(T_PDO_TIMEOUT) && sys.mode != SysMode::LEGACY)
            pdDetect();
    } else {
        sys.mode  = SysMode::DISCONNECTED;
        ui.screen = Screen::DISCONNECTED;
    }

    syncProfileCursorFromActiveMode();
    sess.startMs = sess.lastSampleMs = millis();
    Serial.println("PowerPD v6.0 ready");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
    pd.task();

    // PPS keepalive
    if (sys.mode == SysMode::PPS && sys.mosfetOn && tmPPS.due(T_PPS_KEEPALIVE))
        pd.setPPSPDO(sys.ppsPdoIdx, sys.ppsSetV_mV, sys.ppsSetI_mA);

    // Measurement
    if (tmMeasure.due(T_MEASURE)) {
        float rv = ina.getBusVoltage();
        float ri = ina.getCurrent();
        sys.rawV = (rv > 0) ? rv : 0.0f;
        sys.rawI = (ri > 0) ? ri : 0.0f;
        sys.rawP = sys.rawV * sys.rawI;
        sys.temp_C = pd.getTemperature_C();

        float a = 1.0f / EMA_K;
        sys.dispV = sys.dispV + a * (sys.rawV - sys.dispV);
        sys.dispI = sys.dispI + a * (sys.rawI - sys.dispI);
        sys.dispP = sys.dispV * sys.dispI;

        safetyCheck();
        sessionUpdate();
    }

    // Hotplug
    if (tmHotplug.due(T_HOTPLUG)) {
        bool charger = (pd.getOpMode() & (OPMODE_PDMOD | OPMODE_LGCYMOD)) != 0;
        bool active  = (sys.mode != SysMode::DISCONNECTED && sys.mode != SysMode::DETECTING);
        if (charger && !active) {
            sys.mode = SysMode::DETECTING; ui.screen = Screen::DETECTING;
            delay(150);
            if (pdWaitPDOs(T_PDO_TIMEOUT) && sys.mode != SysMode::LEGACY) pdDetect();
            syncProfileCursorFromActiveMode();
        } else if (!charger && active) {
            sys.mode = SysMode::DISCONNECTED;
            ui.screen = Screen::DISCONNECTED;

            sys.outputReq = false;
            sys.fault = false;

            sys.fixedCount = 0;
            sys.ppsAvail = false;
            sys.ppsPdoIdx = 0;
            sys.negoV_mV = 0;
            sys.negoI_mA = 0;
            ui.profileCursor = 0;

            mosfetUpdate();
        } else if (active && !sys.fault && pd.isFault()) {
            String f = pd.getFaultString();
            faultTrigger(f.c_str());
        }
    }

    // Input
    int     dir = readEncoder();
    uint8_t enc = encSw.poll();
    uint8_t b1  = sw1.poll();
    uint8_t b2  = sw2.poll();

    if (sys.fault) {
        if (enc || b1 || b2) faultClear();
    } else {
        switch (ui.screen) {
            case Screen::MAIN:        handleMain(dir, enc, b1, b2);       break;
            case Screen::MENU:        handleMenu(dir, enc, b1, b2);       break;
            case Screen::PPS_ADJUST:  handlePPSAdjust(dir, enc, b1, b2); break;
            case Screen::PDO_SELECT:  handlePDOSelect(dir, enc, b1, b2); break;
            case Screen::PD_ANALYZER: handlePDAnalyzer(dir, enc, b1, b2); break;
            case Screen::SESSION:     handleSession(dir, enc, b1, b2);   break;
            case Screen::CLOUD:       handleCloud(dir, enc, b1, b2);     break;
            case Screen::SAFETY:      handleSafety(dir, enc, b1, b2);    break;
            case Screen::SYSTEM:      handleSystem(dir, enc, b1, b2);    break;
            default: break;
        }
    }

    // Display
    if (tmDisplay.due(T_DISPLAY)) {
        u8g2.clearBuffer();
        setFontM();
        if (sys.fault)              renderFault();
        else switch (ui.screen) {
            case Screen::MAIN:         renderMain();         break;
            case Screen::MENU:         renderMenu();         break;
            case Screen::PPS_ADJUST:   renderPPSAdjust();   break;
            case Screen::PDO_SELECT:   renderPDOSelect();   break;
            case Screen::PD_ANALYZER:  renderPDAnalyzer();  break;
            case Screen::SESSION:      renderSession();      break;
            case Screen::CLOUD:        renderCloud();        break;
            case Screen::SAFETY:       renderSafety();       break;
            case Screen::SYSTEM:       renderSystem();       break;
            case Screen::DISCONNECTED: renderDisconnected(); break;
            case Screen::DETECTING:    renderDetecting();    break;
            default:                   renderMain();         break;
        }
        u8g2.sendBuffer();
    }

    // Cloud
    if (WiFi.status() == WL_CONNECTED) {
        cloud.wifiOk = true;
        WiFi.localIP().toString().toCharArray(cloud.ipStr, 16);
        if (!mqtt.connected() && tmMqtt.due(T_MQTT_RETRY)) cloudConnect();
        mqtt.loop();
        if (cloud.mqttOk && tmCloud.due(cloud.interval)) cloudPublish();
    } else {
        cloud.wifiOk = cloud.mqttOk = false;
        if (tmWifi.due(T_WIFI_RETRY)) WiFi.reconnect();
    }

    digitalWrite(PIN_LED, sys.mosfetOn ? HIGH : LOW);
}

// ============================================================
//  PD CONTROL
// ============================================================
void pdInit() {
    pd.begin(true, true);
    pd.setProtectionConfig(false, false, true, true, true);
    pd.setOTPThreshold(120);
    pd.setDeratingThreshold(85);
    pd.setOVPOffset_mV(2000);
    pd.setOCPThreshold_mA(0);
}

bool pdWaitPDOs(uint32_t timeout) {
    uint32_t start = millis();

    while (millis() - start < timeout) {
        pd.task();

        uint8_t op = pd.getOpMode();

        if (op & OPMODE_LGCYMOD) {
            sys.mode = SysMode::LEGACY;
            ui.screen = Screen::MAIN;

            sys.fixedCount = 0;
            sys.ppsAvail = false;
            sys.ppsPdoIdx = 0;
            sys.negoV_mV = 5000;
            sys.negoI_mA = 0;
            ui.profileCursor = 0;

            return true;
        }

        if ((op & OPMODE_PDMOD) && pd.readAllPDOs() > 0) {
            return true;
        }

        delay(T_PDO_POLL);
    }

    return false;
}

void pdDetect() {
    uint16_t savedPpsV = sys.ppsSetV_mV;
    uint16_t savedPpsI = sys.ppsSetI_mA;
    uint8_t  savedFixedSel = sys.fixedSel;
    if (pd.readAllPDOs() == 0) {
        sys.mode  = pd.isLegacyConnected() ? SysMode::LEGACY : SysMode::DISCONNECTED;
        ui.screen = (sys.mode == SysMode::DISCONNECTED) ? Screen::DISCONNECTED : Screen::MAIN;
        return;
    }
    sys.fixedCount = 0; sys.ppsAvail = false; sys.ppsMaxV_mV = 5000;
    for (uint8_t i = 1; i <= AP33772S_MAX_PDO; i++) {
        const AP33772S_PDO &p = pd.getPDO(i);
        if (!p.valid) continue;
        if (p.type == PDO_TYPE_FIXED) {
            sys.fixedIdx[sys.fixedCount++] = i;
        } else if (p.type == PDO_TYPE_PPS) {
            sys.ppsAvail = true;
            if (p.maxVoltage_mV >= sys.ppsMaxV_mV) {
                sys.ppsPdoIdx  = i;
                sys.ppsMinV_mV = p.minVoltage_mV;
                sys.ppsMaxV_mV = p.maxVoltage_mV;
                sys.ppsMaxI_mA = p.maxCurrent_mA;
            }
        }
    }

    // Validate saved fixedSel
    if (sys.fixedSel >= sys.fixedCount) sys.fixedSel = 0;

   if (sys.fixedCount > 0) {
    sys.fixedSel = (savedFixedSel < sys.fixedCount) ? savedFixedSel : 0;
}

if (sys.ppsAvail) {
    sys.mode = SysMode::PPS;

    sys.ppsSetV_mV = constrain(
        savedPpsV,
        sys.ppsMinV_mV,
        sys.ppsMaxV_mV
    );

    sys.ppsSetI_mA = constrain(
        savedPpsI,
        (uint16_t)1000u,
        sys.ppsMaxI_mA
    );

} else if (sys.fixedCount > 0) {
    sys.mode = SysMode::PD_FIXED;
} else {
    sys.mode = SysMode::DISCONNECTED;
    ui.screen = Screen::DISCONNECTED;
    return;
}
    sys.outputReq = false;
    ui.screen = Screen::MAIN;
    pdApply(false);
}

void pdApply(bool smooth) {
    bool bigJump = abs((int32_t)sys.ppsSetV_mV - (int32_t)sys.negoV_mV) > 2000;
    if (!smooth || bigJump || sys.mode != SysMode::PPS) {
        digitalWrite(PIN_MOSFET, LOW); sys.mosfetOn = false; delay(20);
    }

    // LEGACY: no negotiation needed, just allow MOSFET control
    if (sys.mode == SysMode::LEGACY) {
        sys.negoV_mV = 5000;
        sys.negoI_mA = 0;
        mosfetUpdate();
        return;
    }

    int8_t rc = AP_ERR_NO_PDO;
    if (sys.mode == SysMode::PPS) {
        sys.ppsSetV_mV = constrain(sys.ppsSetV_mV, sys.ppsMinV_mV, sys.ppsMaxV_mV);
        sys.ppsSetI_mA = constrain(sys.ppsSetI_mA, (uint16_t)1000u, sys.ppsMaxI_mA);
        sys.ppsSetV_mV = (sys.ppsSetV_mV / 20u) * 20u;
        rc = pd.setPPSPDO(sys.ppsPdoIdx, sys.ppsSetV_mV, sys.ppsSetI_mA);
        tmPPS.last = millis();
    } else if (sys.mode == SysMode::PD_FIXED && sys.fixedCount > 0) {
        uint8_t idx = sys.fixedIdx[sys.fixedSel];
        rc = pd.setFixPDO(idx, pd.getPDO(idx).maxCurrent_mA);
    }

    if (rc == AP_OK && pd.waitForNegotiation(T_NEGO_WAIT) == AP_OK) {
        sys.negoV_mV = pd.getRequestedVoltage_mV();
        sys.negoI_mA = pd.getRequestedCurrent_mA();
        mosfetUpdate();
        // NOTE: nvSave() is NOT called here. Caller must call it when user confirms.
    } else {
        sys.negoFails++;
        faultTrigger("Nego Failed");
    }
}

void mosfetUpdate() {
    if (sys.fault || sys.mode == SysMode::FAULT ||
        sys.mode == SysMode::DISCONNECTED || sys.mode == SysMode::DETECTING) {
        digitalWrite(PIN_MOSFET, LOW);
        sys.mosfetOn = false;
        return;
    }
    if (sys.outputReq) {
        uint32_t t = millis();
        while (ina.getBusVoltage() < 3.5f && millis() - t < 600) delay(10);
        digitalWrite(PIN_MOSFET, HIGH);
        sys.mosfetOn = true;
    } else {
        digitalWrite(PIN_MOSFET, LOW);
        sys.mosfetOn = false;
    }
}

void faultTrigger(const char* msg) {
    sys.fault = true;
    strncpy(sys.faultMsg, msg ? msg : "Unknown", 31);
    sys.faultMsg[31] = '\0';
    sys.faultCount++;
    sys.outputReq = false;
    mosfetUpdate();
    ui.screen = Screen::FAULT;
}

void faultClear() {
    sys.fault = false; sys.faultMsg[0] = '\0';
    ui.screen = Screen::MAIN;
    pdApply(false);
}

void safetyCheck() {
    if (sys.fault) return;

    if (sys.ovpEn && sys.rawV > OVP_LIMIT_V) {
        faultTrigger("OVP: Over Voltage");
        return;
    }

    if (sys.rawI > OCP_LIMIT_A) {
        faultTrigger("OCP: Over Current");
        return;
    }

    if (sys.otpEn && sys.temp_C > OTP_LIMIT_C) {
        faultTrigger("OTP: Over Temp");
        return;
    }
}

void sessionUpdate() {
    uint32_t now = millis();
    float dt = (now - sess.lastSampleMs) / 3600000.0f;
    sess.lastSampleMs = now;
    if (sys.mosfetOn) {
        sess.wh += sys.rawP * dt;
        sess.ah += sys.rawI * dt;
        if (sys.rawP > sess.peakW) sess.peakW = sys.rawP;
        if (sys.rawI > sess.peakA) sess.peakA = sys.rawI;
    }
}

// ============================================================
//  NVS
// ============================================================
void nvLoad() {
    prefs.begin("ppd", true);
    sys.ppsSetV_mV  = prefs.getUShort("ppsV",  PPS_DEFAULT_MV);
    sys.ppsSetI_mA  = prefs.getUShort("ppsI",  3000);
    sys.fixedSel    = prefs.getUChar ("fixSel", 0);
    ui.vStepIdx     = prefs.getUChar ("vstep",  1);
    ui.iStepIdx     = prefs.getUChar ("istep",  1);
    ui.cloudRateIdx = prefs.getUChar ("crate",  1);
    sys.ovpEn       = prefs.getBool  ("ovpEn",  true);
    sys.otpEn       = prefs.getBool  ("otpEn",  true);
    prefs.end();

    // Validate indexes to prevent out-of-bounds
    if (ui.vStepIdx     >= V_STEP_N)     ui.vStepIdx     = 1;
    if (ui.iStepIdx     >= I_STEP_N)     ui.iStepIdx     = 1;
    if (ui.cloudRateIdx >= CLOUD_RATE_N) ui.cloudRateIdx = 1;

    cloud.interval = CLOUD_RATES[ui.cloudRateIdx];
}

void nvSave() {
    prefs.begin("ppd", false);
    prefs.putUShort("ppsV",   sys.ppsSetV_mV);
    prefs.putUShort("ppsI",   sys.ppsSetI_mA);
    prefs.putUChar ("fixSel", sys.fixedSel);
    prefs.putUChar ("vstep",  ui.vStepIdx);
    prefs.putUChar ("istep",  ui.iStepIdx);
    prefs.putUChar ("crate",  ui.cloudRateIdx);
    prefs.putBool  ("ovpEn",  sys.ovpEn);
    prefs.putBool  ("otpEn",  sys.otpEn);
    prefs.end();
}

// ============================================================
//  CLOUD
// ============================================================

static void mqttPub(const char* feed, const char* val) {
    char topic[64];
    snprintf(topic, sizeof(topic), "%s%s", AIO_FEED_PREFIX, feed);
    mqtt.publish(topic, val);
}

void cloudConnect() {
    char id[24];
    snprintf(id, sizeof(id), "PPD_%08lX",
             (unsigned long)(ESP.getEfuseMac() & 0xFFFFFFFF));

    if (mqtt.connect(id, AIO_USERNAME, AIO_KEY)) {
        cloud.mqttOk = true;

        // Subscribe to Adafruit IO output feed for remote ON/OFF control
        char topic[64];
        snprintf(topic, sizeof(topic), "%soutput", AIO_FEED_PREFIX);
        mqtt.subscribe(topic);
    } else {
        cloud.mqttOk = false;
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    char msg[16];

    if (length >= sizeof(msg)) {
        length = sizeof(msg) - 1;
    }

    memcpy(msg, payload, length);
    msg[length] = '\0';

    String topicStr = String(topic);

    // Adafruit IO toggle feed: output
    // 1 = ON, 0 = OFF
    if (topicStr.endsWith("/output")) {
        if (strcmp(msg, "1") == 0 || strcmp(msg, "ON") == 0) {

            // Safety check before enabling output remotely
            if (!sys.fault &&
                sys.mode != SysMode::DISCONNECTED &&
                sys.mode != SysMode::DETECTING &&
                sys.mode != SysMode::FAULT) {
                sys.outputReq = true;
            }

        } else {
            sys.outputReq = false;
        }

        mosfetUpdate();
    }
}

void cloudPublish() {
    if (!mqtt.connected()) {
        cloud.mqttOk = false;
        return;
    }

    char b[16];

    // Feed: voltage
    snprintf(b, sizeof(b), "%.3f", sys.rawV);
    mqttPub("voltage", b);

    // Feed: current
    snprintf(b, sizeof(b), "%.3f", sys.rawI);
    mqttPub("current", b);

    // Feed: power
    snprintf(b, sizeof(b), "%.2f", sys.rawP);
    mqttPub("power", b);

    // Feed: temperature
    snprintf(b, sizeof(b), "%d", sys.temp_C);
    mqttPub("temperature", b);

    // Feed: output
    // 1 = ON, 0 = OFF
    snprintf(b, sizeof(b), "%d", sys.mosfetOn ? 1 : 0);
    mqttPub("output", b);

    cloud.lastTxMs = millis();
}

// ============================================================
//  DISPLAY HELPERS
// ============================================================
void setFontS() { u8g2.setFont(u8g2_font_5x7_tf);  }
void setFontM() { u8g2.setFont(u8g2_font_6x10_tf); }
void setFontL() { u8g2.setFont(u8g2_font_10x20_tf); }

void dStr(uint8_t x, uint8_t y, const char* s)  { u8g2.drawStr(x, y, s); }

void dStrC(uint8_t y, const char* s) {
    int16_t w = u8g2.getStrWidth(s);
    u8g2.drawStr((128 - w) / 2, y, s);
}

void dStrR(uint8_t y, const char* s) {
    int16_t w = u8g2.getStrWidth(s);
    u8g2.drawStr(127 - w, y, s);
}

void dHLine(uint8_t y) { u8g2.drawHLine(0, y, 128); }

// Inverted highlight bar — caller draws text on top, then must call
// u8g2.setDrawColor(1) after the text to restore normal drawing.
void dHighlightRow(uint8_t y_top, uint8_t h) {
    u8g2.setDrawColor(1);
    u8g2.drawBox(0, y_top, 128, h);
    u8g2.setDrawColor(0);
}


// Shared OLED UI style -------------------------------------------------------
// Same header, row height, scrollbar, and badge behavior across all screens.
void dHeader(const char* title, const char* right) {
    setFontM();
    dStr(2, 9, title);
    if (right && right[0]) {
        setFontS();
        dStrR(9, right);
        setFontM();
    }
    dHLine(11);
}

void dFooter(const char* msg) {
    dHLine(55);
    setFontS();
    dStrC(63, msg);
    setFontM();
}

void dBadge(uint8_t x, uint8_t y, uint8_t w, const char* label, bool filled) {
    setFontM();
    if (filled) {
        u8g2.drawBox(x, y, w, 11);
        u8g2.setDrawColor(0);
    } else {
        u8g2.drawFrame(x, y, w, 11);
    }

    int16_t tw = u8g2.getStrWidth(label);
    int16_t tx = (int16_t)x + ((int16_t)w - tw) / 2;
    if (tx < (int16_t)x + 1) tx = x + 1;
    u8g2.drawStr(tx, y + 9, label);
    u8g2.setDrawColor(1);
}

void dRow(uint8_t yb, const char* left, const char* right, bool selected) {
    if (selected) dHighlightRow(yb - 10, 12);
    setFontM();
    dStr(4, yb, left);
    if (right && right[0]) dStrR(yb, right);
    u8g2.setDrawColor(1);
}

void dScrollBar(uint8_t total, uint8_t visible, uint8_t offset) {
    if (total <= visible || visible == 0) return;
    static constexpr uint8_t BAR_Y = 12;
    static constexpr uint8_t BAR_H = 50;
    u8g2.drawFrame(124, BAR_Y, 3, BAR_H);

    uint8_t th = (uint8_t)(((uint16_t)BAR_H * visible) / total);
    if (th < 5) th = 5;
    if (th > BAR_H) th = BAR_H;

    uint8_t maxOff = total - visible;
    if (offset > maxOff) offset = maxOff;
    uint8_t ty = BAR_Y;
    if (maxOff > 0) {
        ty = BAR_Y + (uint8_t)(((uint16_t)offset * (BAR_H - th)) / maxOff);
    }
    u8g2.drawBox(124, ty, 3, th);
}

void dMeterBar(uint8_t x, uint8_t y, uint8_t w, uint8_t h, float fraction) {
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;
    u8g2.drawFrame(x, y, w, h);
    if (w <= 2 || h <= 2) return;
    uint8_t fill = (uint8_t)((w - 2) * fraction);
    if (fill) u8g2.drawBox(x + 1, y + 1, fill, h - 2);
}

const char* modeLabelShort() {
    if      (sys.mode == SysMode::PPS)          return "PPS";
    else if (sys.mode == SysMode::PD_FIXED)     return "FIX";
    else if (sys.mode == SysMode::LEGACY)       return "LEG";
    else if (sys.mode == SysMode::DETECTING)    return "DET";
    else if (sys.mode == SysMode::DISCONNECTED) return "NO PD";
    else if (sys.mode == SysMode::FAULT)        return "FAULT";
    return "---";
}

const char* mainFieldTitle() {
    switch (ui.mainField) {
        case MainField::FIELD_PROFILE: return "PROFILE";
        case MainField::FIELD_VSET:    return "SET VOLT";
        case MainField::FIELD_ILIM:    return "SET CURR";
        case MainField::FIELD_OUTPUT:  return "OUTPUT";
        case MainField::FIELD_MENU:    return "MENU";
        default:                       return "DASH";
    }
}

float activeCurrentLimitA() {
    if (sys.mode == SysMode::PPS) {
        return sys.ppsSetI_mA / 1000.0f;
    }
    if (sys.mode == SysMode::PD_FIXED || sys.mode == SysMode::LEGACY) {
        return sys.negoI_mA / 1000.0f;
    }
    return 0.0f;
}

// Approximate output regulation state for the main dashboard.
// Note: with USB-C PPS the charger/source performs the real limiting;
// this label is inferred from measured V/I versus the requested PPS limit.
const char* regLabel() {
    if (sys.fault) return "FLT";
    if (!sys.mosfetOn) return "OFF";
    if (sys.mode == SysMode::LEGACY) return "LEG";
    if (sys.mode == SysMode::PD_FIXED) return "FIX";
    if (sys.mode != SysMode::PPS) return "---";

    float vset = sys.ppsSetV_mV / 1000.0f;
    float ilim = sys.ppsSetI_mA / 1000.0f;
    float dv = sys.rawV - vset;
    if (dv < 0.0f) dv = -dv;

    bool nearCurrentLimit = (ilim > 0.0f) && (sys.rawI >= (ilim - 0.10f));
    bool voltageDroop     = sys.rawV < (vset - 0.25f);

    if (nearCurrentLimit && voltageDroop) return "CC";
    if (dv <= 0.20f) return "CV";
    return "PPS";
}

// ============================================================
//  RENDER — MAIN SCREEN
// ============================================================
/*
  128x64 layout (y = text baseline):

  y=9   [●●] PPS 20.00V       [ON]    <- profile label left, badge right
                                          wifi dots between them
  y=11  ─────────────────────────────
  y=31           20.05V               <- large voltage centered (10x20)
  y=51            1.234A              <- large current centered (10x20)
  y=53  ─────────────────────────────
  y=62  25.1W           LIM 3.00A    <- power left, limit right (6x10)

  Profile area:  x=0..100
  WiFi dots:     x=85,91  y=4  (between profile text and badge)
  ON/OFF badge:  x=104..127

  Field highlights:
    FIELD_PROFILE  → invert x=0..100  y=0..11
    FIELD_VSET     → invert voltage row (underline via box below)
    FIELD_ILIM     → invert LIM field at bottom right
    FIELD_OUTPUT   → invert ON/OFF badge
    FIELD_MENU     → replace profile text with "▸ MENU"
*/
void renderMain() {
    char buf[28];
    char title[18];

    // Header: same visual system as the menu, plus live status badges.
    if (ui.mainField == MainField::FIELD_PROFILE) {
        formatProfileLabel(title, sizeof(title), ui.profileCursor);
    } else {
        snprintf(title, sizeof(title), "%s", mainFieldTitle());
    }

    setFontM();
    dStr(2, 9, title);

    // Cloud status dots: hollow = disconnected, filled = OK.
    if (cloud.wifiOk) u8g2.drawDisc(62, 4, 2); else u8g2.drawCircle(62, 4, 2);
    if (cloud.mqttOk) u8g2.drawDisc(68, 4, 2); else u8g2.drawCircle(68, 4, 2);

    // Regulation + output badges.
    dBadge(74, 0, 28, regLabel(), false);
    dBadge(105, 0, 23, sys.mosfetOn ? "ON" : "OFF",
           sys.mosfetOn || ui.mainField == MainField::FIELD_OUTPUT);
    dHLine(11);

    bool editV = (ui.mainField == MainField::FIELD_VSET && sys.ppsAvail);
    bool editI = (ui.mainField == MainField::FIELD_ILIM && sys.ppsAvail);

    // Friendly focus frames. The selected editable value gets a clear box,
    // but the value remains normal black-on-white for readability.
    if (editV) u8g2.drawFrame(0, 13, 128, 20);
    if (editI) u8g2.drawFrame(0, 34, 128, 19);

    // Voltage block ---------------------------------------------------------
    setFontS();
    dStr(2, 21, editV ? "VSET" : "VOUT");
    setFontL();
    float shownV = editV ? (sys.ppsSetV_mV / 1000.0f) : sys.dispV;
    snprintf(buf, sizeof(buf), "%.2fV", shownV);
    int16_t w = u8g2.getStrWidth(buf);
    u8g2.drawStr((128 - w) / 2, 31, buf);

    // Current block ---------------------------------------------------------
    setFontS();
    dStr(2, 43, editI ? "ILIM" : "IOUT");
    setFontL();
    float shownI = editI ? (sys.ppsSetI_mA / 1000.0f) : sys.dispI;
    snprintf(buf, sizeof(buf), editI ? "%.2fA" : "%.3fA", shownI);
    w = u8g2.getStrWidth(buf);
    u8g2.drawStr((128 - w) / 2, 51, buf);

    dHLine(53);

    // Bottom action/status row --------------------------------------------
    setFontM();
    if (editV) {
        snprintf(buf, sizeof(buf), "OUT %.2fV", sys.dispV);
        dStr(0, 62, buf);
        snprintf(buf, sizeof(buf), "x%umV", V_STEPS[ui.vStepIdx]);
        dStrR(62, buf);
        return;
    }

    if (editI) {
        snprintf(buf, sizeof(buf), "OUT %.3fA", sys.dispI);
        dStr(0, 62, buf);
        snprintf(buf, sizeof(buf), "x%umA", I_STEPS[ui.iStepIdx]);
        dStrR(62, buf);
        return;
    }

    if (ui.mainField == MainField::FIELD_PROFILE) {
        setFontS();
        dStr(0, 62, "TURN choose");
        dStrR(62, "LONG apply");
        return;
    }

    if (ui.mainField == MainField::FIELD_OUTPUT) {
        setFontS();
        dStr(0, 62, "TURN on/off");
        dStrR(62, "LONG toggle");
        return;
    }

    if (ui.mainField == MainField::FIELD_MENU) {
        setFontS();
        dStr(0, 62, "LONG open menu");
        dStrR(62, "PRESS next");
        return;
    }

    float limA = activeCurrentLimitA();
    snprintf(buf, sizeof(buf), "P%.1fW", sys.dispP);
    dStr(0, 62, buf);

    if (limA > 0.0f) {
        float pct = sys.dispI / limA;
        dMeterBar(47, 56, 34, 7, pct);
        snprintf(buf, sizeof(buf), "LIM%.2fA", limA);
    } else {
        snprintf(buf, sizeof(buf), "LIM--");
    }
    dStrR(62, buf);
}


// ============================================================
//  RENDER — TEXT MENU
// ============================================================
/*
  y=9   MENU                  BACK
  y=11  ──────────────────────────
  y=23  PPS Adjust               (selected = inverted)
  y=36  PDO Select
  y=49  PD Analyzer
  y=62  Session
        [scrollbar right edge]
*/
void renderMenu() {
    dHeader("MENU", "LONG BACK");

    static constexpr uint8_t ROW_H   = 12;
    static constexpr uint8_t MAX_VIS = 4;
    static constexpr uint8_t Y0      = 23;

    if (ui.cursor < ui.scrollOff) ui.scrollOff = ui.cursor;
    if (ui.cursor >= ui.scrollOff + MAX_VIS) ui.scrollOff = ui.cursor - MAX_VIS + 1;

    for (uint8_t i = 0; i < MAX_VIS && (ui.scrollOff + i) < MENU_COUNT; i++) {
        uint8_t idx = ui.scrollOff + i;
        uint8_t yb  = Y0 + i * ROW_H;
        dRow(yb, MENU_ITEMS[idx], "", idx == ui.cursor);
    }

    dScrollBar(MENU_COUNT, MAX_VIS, ui.scrollOff);
}


// ============================================================
//  RENDER — PPS ADJUST (4 fields)
// ============================================================
/*
  y=9    PPS ADJUST
  y=11   ──────────────────────────
  y=23   VSET    20.00V          (field 0)
  y=37   ILIM     3.00A          (field 1)
  y=51   V STEP  100mV           (field 2)
  y=63   I STEP  100mA           (field 3)

  Selected row inverted.
  Footer range info omitted (no room with 4 rows).
*/
void renderPPSAdjust() {
    dHeader("PPS ADJUST", "LONG APPLY");

    if (!sys.ppsAvail) {
        setFontM();
        dStrC(32, "PPS not found");
        setFontS();
        dStrC(48, "Use PDO Select instead");
        return;
    }

    char right[24];
    static constexpr uint8_t Y0 = 23;
    static constexpr uint8_t ROW_H = 12;

    snprintf(right, sizeof(right), "%.2fV", sys.ppsSetV_mV / 1000.0f);
    dRow(Y0, "VOLT SET", right, ui.ppsField == 0);

    snprintf(right, sizeof(right), "%.2fA", sys.ppsSetI_mA / 1000.0f);
    dRow(Y0 + ROW_H, "CURR LIM", right, ui.ppsField == 1);

    snprintf(right, sizeof(right), "%umV", V_STEPS[ui.vStepIdx]);
    dRow(Y0 + 2 * ROW_H, "V STEP", right, ui.ppsField == 2);

    snprintf(right, sizeof(right), "%umA", I_STEPS[ui.iStepIdx]);
    dRow(Y0 + 3 * ROW_H, "I STEP", right, ui.ppsField == 3);
}


// ============================================================
//  RENDER — PDO SELECT
// ============================================================
void renderPDOSelect() {
    dHeader("PDO SELECT", "PRESS USE");

    uint8_t total = profileTotal();
    if (total == 0) {
        setFontM();
        dStrC(34, "No PD profiles");
        setFontS();
        dStrC(50, "Reconnect charger");
        return;
    }

    static constexpr uint8_t ROW_H = 12;
    static constexpr uint8_t MAX_V = 4;
    static constexpr uint8_t Y0    = 23;

    if (ui.cursor < ui.scrollOff) ui.scrollOff = ui.cursor;
    if (ui.cursor >= ui.scrollOff + MAX_V) ui.scrollOff = ui.cursor - MAX_V + 1;

    uint8_t activeIdx = 255;
    if (sys.mode == SysMode::PPS && sys.ppsAvail) {
        activeIdx = sys.fixedCount;
    } else if (sys.mode == SysMode::PD_FIXED && sys.fixedSel < sys.fixedCount) {
        activeIdx = sys.fixedSel;
    }

    for (uint8_t i = 0; i < MAX_V && (ui.scrollOff + i) < total; i++) {
        uint8_t idx = ui.scrollOff + i;
        uint8_t yb  = Y0 + i * ROW_H;
        bool act = (idx == activeIdx);

        char left[12];
        char right[24];
        if (idx < sys.fixedCount) {
            uint8_t pi = sys.fixedIdx[idx];
            const AP33772S_PDO &p = pd.getPDO(pi);
            snprintf(left, sizeof(left), "%cFIX%u", act ? '*' : ' ', (unsigned)pi);
            snprintf(right, sizeof(right), "%.1fV %.1fA",
                     p.maxVoltage_mV / 1000.0f,
                     p.maxCurrent_mA / 1000.0f);
        } else {
            snprintf(left, sizeof(left), "%cPPS", act ? '*' : ' ');
            snprintf(right, sizeof(right), "%.1f-%.1fV",
                     sys.ppsMinV_mV / 1000.0f,
                     sys.ppsMaxV_mV / 1000.0f);
        }
        dRow(yb, left, right, idx == ui.cursor);
    }

    dScrollBar(total, MAX_V, ui.scrollOff);
}



// ============================================================
//  RENDER — PD ANALYZER
// ============================================================
/*
  Overview rows + PDO rows shown as a scrollable list.
  All rows have the same height (10px + 1px gap).

  Row list (ui.analyzerRow is scroll offset):
    0:  Mode: PPS / Fixed / Legacy / No PD
    1:  Req:  20.00V 3.00A
    2:  Bus:  20.05V 1.23A
    3:  Temp: 35C   Fault: none
    4:  PDOs: 5   PPS: Yes
    ── separator ──
    5+: PDO detail rows (one per PDO)

  In detail mode (ui.analyzerDetail=true), PDO rows are expanded.
  In overview mode, PDO rows are compact single-line.
*/

// PDO row count — we show overview rows + one per PDO
static constexpr uint8_t ANA_OVW_ROWS = 5;  // overview rows before PDO list

static uint32_t gAnalyzerLastRead = 0;

static void analyzerEnsureRead() {
    // Refresh PDOs at most once per T_ANALYZER_REFRESH, non-blocking
    if (millis() - gAnalyzerLastRead >= T_ANALYZER_REFRESH) {
        gAnalyzerLastRead = millis();
        // Only attempt if we have a PD charger
        if (sys.mode != SysMode::DISCONNECTED &&
            sys.mode != SysMode::LEGACY &&
            sys.mode != SysMode::DETECTING) {
            pd.readAllPDOs();
        }
    }
}

void renderPDAnalyzer() {
    analyzerEnsureRead();

    dHeader("PD ANALYZER", ui.analyzerDetail ? "DETAIL" : "OVERVIEW");

    uint8_t pdoCount = 0;
    uint8_t ppsCount = 0;
    for (uint8_t i = 1; i <= AP33772S_MAX_PDO; i++) {
        const AP33772S_PDO &p = pd.getPDO(i);
        if (p.valid) {
            pdoCount++;
            if (p.type == PDO_TYPE_PPS) ppsCount++;
        }
    }

    uint8_t totalRows = ANA_OVW_ROWS + pdoCount;
    if (totalRows == 0) totalRows = ANA_OVW_ROWS;

    static constexpr uint8_t ROW_H = 12;
    static constexpr uint8_t MAX_V = 4;
    static constexpr uint8_t Y0    = 23;

    uint8_t maxScroll = (totalRows > MAX_V) ? (totalRows - MAX_V) : 0;
    if (ui.analyzerRow > maxScroll) ui.analyzerRow = maxScroll;

    for (uint8_t dispRow = 0; dispRow < MAX_V; dispRow++) {
        uint8_t row = ui.analyzerRow + dispRow;
        if (row >= totalRows) break;

        uint8_t yb = Y0 + dispRow * ROW_H;
        char left[14];
        char right[28];
        left[0] = '\0';
        right[0] = '\0';

        if (row < ANA_OVW_ROWS) {
            switch (row) {
                case 0:
                    snprintf(left, sizeof(left), "MODE");
                    snprintf(right, sizeof(right), "%s %s", modeLabelShort(), regLabel());
                    break;
                case 1:
                    snprintf(left, sizeof(left), "REQUEST");
                    snprintf(right, sizeof(right), "%.2fV %.2fA",
                             sys.negoV_mV / 1000.0f, sys.negoI_mA / 1000.0f);
                    break;
                case 2:
                    snprintf(left, sizeof(left), "BUS");
                    snprintf(right, sizeof(right), "%.2fV %.3fA", sys.rawV, sys.rawI);
                    break;
                case 3:
                    snprintf(left, sizeof(left), "POWER");
                    snprintf(right, sizeof(right), "%.1fW %dC", sys.rawP, sys.temp_C);
                    break;
                case 4:
                    snprintf(left, sizeof(left), "PDO");
                    snprintf(right, sizeof(right), "%u PDO %u PPS", pdoCount, ppsCount);
                    break;
                default:
                    break;
            }
            dRow(yb, left, right, false);
        } else {
            uint8_t pdoSlot = row - ANA_OVW_ROWS;
            uint8_t found = 0;
            uint8_t pi = 0;
            for (uint8_t i = 1; i <= AP33772S_MAX_PDO; i++) {
                if (pd.getPDO(i).valid) {
                    if (found == pdoSlot) { pi = i; break; }
                    found++;
                }
            }

            if (pi == 0) {
                dRow(yb, "PDO", "---", false);
            } else {
                const AP33772S_PDO &p = pd.getPDO(pi);
                bool active = false;
                if (sys.mode == SysMode::PPS && p.type == PDO_TYPE_PPS && pi == sys.ppsPdoIdx) active = true;
                if (sys.mode == SysMode::PD_FIXED && sys.fixedSel < sys.fixedCount && pi == sys.fixedIdx[sys.fixedSel]) active = true;

                if (p.type == PDO_TYPE_PPS) {
                    snprintf(left, sizeof(left), "%cPPS%u", active ? '*' : ' ', (unsigned)pi);
                    if (ui.analyzerDetail) {
                        snprintf(right, sizeof(right), "%.1f-%.1fV %.1fA",
                                 p.minVoltage_mV / 1000.0f,
                                 p.maxVoltage_mV / 1000.0f,
                                 p.maxCurrent_mA / 1000.0f);
                    } else {
                        snprintf(right, sizeof(right), "%.0f-%.0fV %.1fA",
                                 p.minVoltage_mV / 1000.0f,
                                 p.maxVoltage_mV / 1000.0f,
                                 p.maxCurrent_mA / 1000.0f);
                    }
                } else {
                    uint16_t watts = (uint16_t)((uint32_t)p.maxVoltage_mV *
                                                p.maxCurrent_mA / 1000000UL);
                    snprintf(left, sizeof(left), "%cFIX%u", active ? '*' : ' ', (unsigned)pi);
                    if (ui.analyzerDetail) {
                        snprintf(right, sizeof(right), "%.1fV %.1fA %uW",
                                 p.maxVoltage_mV / 1000.0f,
                                 p.maxCurrent_mA / 1000.0f,
                                 (unsigned)watts);
                    } else {
                        snprintf(right, sizeof(right), "%.1fV %.1fA",
                                 p.maxVoltage_mV / 1000.0f,
                                 p.maxCurrent_mA / 1000.0f);
                    }
                }
                dRow(yb, left, right, false);
            }
        }
    }

    dScrollBar(totalRows, MAX_V, ui.analyzerRow);
}


// ============================================================
//  RENDER — SESSION
// ============================================================
void renderSession() {
    dHeader("SESSION", "SW2 RESET");

    uint32_t s  = (millis() - sess.startMs) / 1000;
    uint32_t h  = s / 3600, m = (s % 3600) / 60, sc = s % 60;
    char right[28];

    snprintf(right, sizeof(right), "%.4fWh", sess.wh);
    dRow(23, "ENERGY", right, false);

    snprintf(right, sizeof(right), "%.4fAh", sess.ah);
    dRow(35, "CHARGE", right, false);

    snprintf(right, sizeof(right), "%02lu:%02lu:%02lu",
             (unsigned long)h, (unsigned long)m, (unsigned long)sc);
    dRow(47, "TIME", right, false);

    snprintf(right, sizeof(right), "%.1fW %.3fA", sess.peakW, sess.peakA);
    dRow(59, "PEAK", right, false);
}


// ============================================================
//  RENDER — CLOUD
// ============================================================
void renderCloud() {
    dHeader("CLOUD", "ROT RATE");

    char right[28];
    dRow(23, "WIFI", cloud.wifiOk ? "OK" : "OFF", false);
    dRow(35, "MQTT", cloud.mqttOk ? "OK" : "OFF", false);

    if (cloud.lastTxMs) {
        uint32_t ago = (millis() - cloud.lastTxMs) / 1000;
        snprintf(right, sizeof(right), "%lus ago", (unsigned long)ago);
    } else {
        snprintf(right, sizeof(right), "never");
    }
    dRow(47, "LAST TX", right, false);

    snprintf(right, sizeof(right), "%lus", (unsigned long)(CLOUD_RATES[ui.cloudRateIdx] / 1000));
    dRow(59, "SEND RATE", right, true);
}


// ============================================================
//  RENDER — SAFETY
// ============================================================
/*
  y=9    SAFETY
  y=11   ──────────────────
  y=23   OVP  ON           (cursor=0: inverted)
  y=35   OTP  OFF          (cursor=1: inverted)
  y=47   OCP  5.5A (hard)  read-only
  y=59   Faults 0 NegoF 0  read-only
*/
void renderSafety() {
    dHeader("SAFETY", "PRESS TOG");

    char right[28];
    dRow(23, "OVP", sys.ovpEn ? "ON" : "OFF", ui.safetyCursor == 0);
    dRow(35, "OTP", sys.otpEn ? "ON" : "OFF", ui.safetyCursor == 1);

    snprintf(right, sizeof(right), "%.1fA hard", OCP_LIMIT_A);
    dRow(47, "OCP", right, false);

    snprintf(right, sizeof(right), "%u F  %u N", sys.faultCount, sys.negoFails);
    dRow(59, "COUNTERS", right, false);
}


// ============================================================
//  RENDER — SYSTEM
// ============================================================
void renderSystem() {
    dHeader("SYSTEM", "LONG BACK");

    char right[28];
    dRow(23, "FIRMWARE", "6.0", false);

    snprintf(right, sizeof(right), "%dC", sys.temp_C);
    dRow(35, "TEMP", right, false);

    dRow(47, "MODE", modeLabelShort(), false);
    dRow(59, "PPS", sys.ppsAvail ? "AVAILABLE" : "NO", false);
}


// ============================================================
//  RENDER — FAULT
// ============================================================
void renderFault() {
    dHeader("FAULT", "PRESS CLR");

    char msg[15];
    strncpy(msg, sys.faultMsg, 14);
    msg[14] = '\0';

    dRow(23, "OUTPUT", "OFF", true);
    dRow(36, "REASON", msg, false);
    dRow(49, "ACTION", "Check load", false);

    setFontS();
    dStrC(63, "Fix issue, press encoder");
}


// ============================================================
//  RENDER — DISCONNECTED
// ============================================================
void renderDisconnected() {
    dHeader("NO SOURCE", "USB-C PD");
    setFontM();
    dStrC(28, "Connect charger");
    setFontS();
    dStrC(43, "Waiting for PD source");
    dStrC(57, "Output is safely off");
}


// ============================================================
//  RENDER — DETECTING
// ============================================================
void renderDetecting() {
    static uint8_t phase = 0;
    phase = (phase + 1) & 7;

    dHeader("DETECTING", "PLEASE WAIT");
    setFontM();
    dStrC(25, "Reading PDOs");

    u8g2.drawFrame(14, 34, 100, 8);
    uint8_t fill = (uint8_t)((uint16_t)phase * 100 / 8);
    if (fill) u8g2.drawBox(15, 35, fill, 6);

    setFontS();
    dStrC(57, "Keep USB-C connected");
}


// ============================================================
//  INPUT — MAIN SCREEN
// ============================================================
void handleMain(int dir, uint8_t enc, uint8_t b1, uint8_t b2) {

    // Short press → cycle field
    if (enc == 1) {
        uint8_t f = (uint8_t)ui.mainField + 1;
        if (f >= (uint8_t)MainField::FIELD_COUNT) f = 0;
        ui.mainField = (MainField)f;
        // When entering PROFILE field, sync cursor to active profile
        if (ui.mainField == MainField::FIELD_PROFILE) {
            syncProfileCursorFromActiveMode();
        }
        return;
    }

    // Long press → action for current field
    if (enc == 2) {
        switch (ui.mainField) {
            case MainField::FIELD_PROFILE:
                applyProfileCursor();
                nvSave();
                break;
            case MainField::FIELD_VSET:
            case MainField::FIELD_ILIM:
                if (sys.ppsAvail) {
                    sys.mode = SysMode::PPS;
                    ui.profileCursor = sys.fixedCount;
                    pdApply(false);
                    nvSave();
                }
                break;
            case MainField::FIELD_OUTPUT:
                sys.outputReq = !sys.outputReq;
                mosfetUpdate();
                break;
            case MainField::FIELD_MENU:
                ui.screen = Screen::MENU;
                ui.cursor = 0; ui.scrollOff = 0;
                break;
            default: break;
        }
        return;
    }

    // Rotate → adjust selected field
    if (dir != 0) {
        switch (ui.mainField) {
            case MainField::FIELD_PROFILE: {
                uint8_t total = profileTotal();
                if (total == 0) break;
                int16_t n = (int16_t)ui.profileCursor + dir;
                while (n < 0) n += total;
                while (n >= total) n -= total;
                ui.profileCursor = (uint8_t)n;
                break;
            }
            case MainField::FIELD_VSET:
                if (sys.ppsAvail) {
                    int32_t step = V_STEPS[ui.vStepIdx];
                    sys.ppsSetV_mV = (uint16_t)constrain(
                        (int32_t)sys.ppsSetV_mV + dir * step,
                        (int32_t)sys.ppsMinV_mV, (int32_t)sys.ppsMaxV_mV);
                    // Preview only — long press applies PPS and saves.
                }
                break;
            case MainField::FIELD_ILIM:
                if (sys.ppsAvail) {
                    int32_t step = I_STEPS[ui.iStepIdx];
                    sys.ppsSetI_mA = (uint16_t)constrain(
                        (int32_t)sys.ppsSetI_mA + dir * step,
                        1000L, (int32_t)sys.ppsMaxI_mA);
                    // Preview only — long press applies PPS and saves.
                }
                break;
            case MainField::FIELD_OUTPUT:
                // Rotation optionally toggles output
                sys.outputReq = (dir > 0) ? true : false;
                mosfetUpdate();
                break;
            case MainField::FIELD_MENU:
                // Rotation on MENU field does nothing — prevents accidental changes
                break;
            default: break;
        }
        return;
    }

    // Optional buttons
    if (b1 == 1) {
        ui.mainField = (ui.mainField == MainField::FIELD_VSET)
                       ? MainField::FIELD_ILIM : MainField::FIELD_VSET;
    }
    if (b1 == 2) {
        ui.screen = Screen::MENU; ui.cursor = 0; ui.scrollOff = 0;
    }
    if (b2 == 1) {
        sys.outputReq = !sys.outputReq; mosfetUpdate();
    }
    if (b2 == 2) {
        ui.screen = Screen::SESSION;
    }
}

// ============================================================
//  INPUT — MENU
// ============================================================
void handleMenu(int dir, uint8_t enc, uint8_t b1, uint8_t b2) {
    if (dir != 0) {
        int8_t n = (int8_t)ui.cursor + dir;
        ui.cursor = (uint8_t)constrain(n, 0, (int8_t)(MENU_COUNT - 1));
        return;
    }
    // Long press or SW1 short → back to main
    if (enc == 2 || b1 == 1) {
        ui.screen = Screen::MAIN; return;
    }
    // Short press or SW2 → enter selected screen
    if (enc == 1 || b2 == 1) {
        // Map cursor index to Screen
        // Matches MENU_ITEMS order:
        // 0=PPS Adjust, 1=PDO Select, 2=PD Analyzer,
        // 3=Session, 4=Cloud, 5=Safety, 6=System
        static const Screen MAP[] = {
            Screen::PPS_ADJUST, Screen::PDO_SELECT, Screen::PD_ANALYZER,
            Screen::SESSION,    Screen::CLOUD,      Screen::SAFETY, Screen::SYSTEM
        };
        if (ui.cursor < MENU_COUNT) {
            Screen dest = MAP[ui.cursor];
            ui.screen    = dest;
            ui.cursor    = 0;
            ui.scrollOff = 0;
            ui.ppsField  = 0;

            // When entering PD Analyzer, trigger immediate read
            if (dest == Screen::PD_ANALYZER) {
                ui.analyzerRow    = 0;
                ui.analyzerDetail = false;
                gAnalyzerLastRead = 0;  // force refresh
            }
        }
    }
}

// ============================================================
//  INPUT — PPS ADJUST (4 fields)
// ============================================================
void handlePPSAdjust(int dir, uint8_t enc, uint8_t b1, uint8_t b2) {
    // Long press → apply PPS settings & return to main
    if (enc == 2 || b1 == 2) {
        if (sys.ppsAvail) {
            sys.mode = SysMode::PPS;
            ui.profileCursor = sys.fixedCount;
            pdApply(false);
            nvSave();
        }
        ui.screen = Screen::MAIN;
        return;
    }
    // Short press → advance sub-field (4 fields: 0..3)
    if (enc == 1 || b1 == 1) {
        ui.ppsField = (ui.ppsField + 1) % 4;
        return;
    }
    if (!dir) return;

    switch (ui.ppsField) {
        case 0:  // VSET
            if (sys.ppsAvail) {
                int32_t step = V_STEPS[ui.vStepIdx];
                sys.ppsSetV_mV = (uint16_t)constrain(
                    (int32_t)sys.ppsSetV_mV + dir * step,
                    (int32_t)sys.ppsMinV_mV, (int32_t)sys.ppsMaxV_mV);
            }
            break;
        case 1:  // ILIM
            if (sys.ppsAvail) {
                int32_t step = I_STEPS[ui.iStepIdx];
                sys.ppsSetI_mA = (uint16_t)constrain(
                    (int32_t)sys.ppsSetI_mA + dir * step,
                    1000L, (int32_t)sys.ppsMaxI_mA);
            }
            break;
        case 2:  // V STEP
            {
                int8_t n = (int8_t)ui.vStepIdx + dir;
                ui.vStepIdx = (uint8_t)constrain(n, 0, (int8_t)(V_STEP_N - 1));
            }
            break;
        case 3:  // I STEP
            {
                int8_t n = (int8_t)ui.iStepIdx + dir;
                ui.iStepIdx = (uint8_t)constrain(n, 0, (int8_t)(I_STEP_N - 1));
            }
            break;
        default: break;
    }
}

// ============================================================
//  INPUT — PDO SELECT
// ============================================================
void handlePDOSelect(int dir, uint8_t enc, uint8_t b1, uint8_t b2) {
    uint8_t total = profileTotal();
    if (!total) { if (enc || b1) ui.screen = Screen::MENU; return; }
    // Long press → back to menu (no apply)
    if (enc == 2 || b1 == 2) { ui.screen = Screen::MENU; return; }
    if (dir != 0) {
        int8_t n = (int8_t)ui.cursor + dir;
        ui.cursor = (uint8_t)constrain(n, 0, (int8_t)(total - 1));
        return;
    }
    // Short press → apply selected profile
    if (enc == 1 || b1 == 1) {
        if (ui.cursor < sys.fixedCount) {
            sys.fixedSel = ui.cursor;
            sys.mode = SysMode::PD_FIXED;
            pdApply(false);
        } else if (sys.ppsAvail) {
            sys.mode = SysMode::PPS;
            pdApply(false);
        }
        nvSave();
        syncProfileCursorFromActiveMode();
        ui.screen = Screen::MAIN;
    }
}

// ============================================================
//  INPUT — PD ANALYZER
// ============================================================
void handlePDAnalyzer(int dir, uint8_t enc, uint8_t b1, uint8_t b2) {
    // Long press → back to menu
    if (enc == 2 || b1 == 2) {
        ui.screen = Screen::MENU; return;
    }
    // Short press → toggle overview / detail mode
    if (enc == 1 || b1 == 1) {
        ui.analyzerDetail = !ui.analyzerDetail;
        return;
    }
    // Rotate → scroll rows
    if (dir != 0) {
        int8_t n = (int8_t)ui.analyzerRow + dir;
        // Max scroll computed dynamically, clamp at render time
        ui.analyzerRow = (uint8_t)constrain(n, 0, 30);
    }
}

// ============================================================
//  INPUT — SESSION
// ============================================================
void handleSession(int dir, uint8_t enc, uint8_t b1, uint8_t b2) {
    if (enc == 2 || b1 == 1) { ui.screen = Screen::MENU; return; }
    if (b2 == 1) {
        sess.wh = 0; sess.ah = 0; sess.peakW = 0; sess.peakA = 0;
        sess.startMs = millis();
    }
}

// ============================================================
//  INPUT — CLOUD
// ============================================================
void handleCloud(int dir, uint8_t enc, uint8_t b1, uint8_t b2) {
    if (enc == 2 || b1 == 2) { ui.screen = Screen::MENU; return; }
    if (dir != 0) {
        int8_t n = (int8_t)ui.cloudRateIdx + dir;
        ui.cloudRateIdx = (uint8_t)constrain(n, 0, (int8_t)(CLOUD_RATE_N - 1));
        cloud.interval  = CLOUD_RATES[ui.cloudRateIdx];
        // Save cloud rate immediately (low-wear: user explicitly rotating)
        nvSave();
        return;
    }
    if (enc == 1 || b1 == 1 || b2 == 1) { ui.screen = Screen::MENU; }
}

// ============================================================
//  INPUT — SAFETY
// ============================================================
/*
  Rotate: move cursor between OVP (0) and OTP (1)
  Short press: toggle selected item
  Long press: back to menu
*/
void handleSafety(int dir, uint8_t enc, uint8_t b1, uint8_t b2) {
    // Long press → back to menu
    if (enc == 2 || b1 == 2) {
        ui.screen = Screen::MENU; return;
    }
    // Rotate → move cursor between OVP and OTP
    if (dir != 0) {
        ui.safetyCursor = (ui.safetyCursor == 0) ? 1 : 0;
        return;
    }
    // Short press → toggle selected safety item
    if (enc == 1 || b1 == 1) {
        if (ui.safetyCursor == 0) {
            sys.ovpEn = !sys.ovpEn;
        } else {
            sys.otpEn = !sys.otpEn;
        }
        nvSave();
        return;
    }
    // SW2 — same toggle shortcut
    if (b2 == 1) {
        if (ui.safetyCursor == 0) sys.ovpEn = !sys.ovpEn;
        else                      sys.otpEn = !sys.otpEn;
        nvSave();
    }
}

// ============================================================
//  INPUT — SYSTEM (read-only)
// ============================================================
void handleSystem(int dir, uint8_t enc, uint8_t b1, uint8_t b2) {
    if (enc || b1 || b2) ui.screen = Screen::MENU;
}
