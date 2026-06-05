/*
 * Supports:
 *   - USB PD 3.1 v1.6  (TID: 10062)
 *   - SPR Fixed PDOs   : 5V / 9V / 12V / 15V / 20V
 *   - SPR PPS APDOs    : 3.3V – 21V, 100 mV resolution
 *   - EPR Fixed PDOs   : 28V
 *   - EPR AVS APDOs    : 15V – 28V, 200 mV resolution
 *
 * I2C Address : 0x52
 * Protocol    : Wire.h (400 kHz max, little-endian multi-byte)
 * No blocking delay() in public API — non-blocking state machine.
 *
 * (c) 2024 – MIT License
 */

#pragma once
#ifndef AP33772S_LIB_H
#define AP33772S_LIB_H

#include <Arduino.h>
#include <Wire.h>

// ─────────────────────────────────────────────────────────────────────────────
//  DEVICE CONSTANTS
// ─────────────────────────────────────────────────────────────────────────────
#define AP33772S_I2C_ADDR       0x52   ///< Fixed I2C slave address
#define AP33772S_MAX_PDO        13     ///< 7 SPR + 6 EPR PDO slots
#define AP33772S_SPR_PDO_COUNT  7      ///< SPR PDO slots (index 0-6)
#define AP33772S_EPR_PDO_COUNT  6      ///< EPR PDO slots (index 7-12)
#define AP33772S_SRCPDO_BYTES   26     ///< 13 PDOs × 2 bytes
#define AP33772S_KEEPALIVE_MS   5000   ///< PPS/AVS keep-alive period (ms) – must be <10 s

// ─────────────────────────────────────────────────────────────────────────────
//  REGISTER MAP  (Datasheet Table 19 – AP33772S DS46176 Rev. 9-2)
// ─────────────────────────────────────────────────────────────────────────────
#define REG_STATUS      0x01   ///< Status (RC – reset on read)
#define REG_MASK        0x02   ///< Interrupt enable mask (RW)
#define REG_OPMODE      0x03   ///< Operating mode (RO)
#define REG_CONFIG      0x04   ///< Protection configuration (RW, pwr-on 0xF8)
#define REG_PDCONFIG    0x05   ///< PD mode configuration (RW, pwr-on 0x03)
#define REG_SYSTEM      0x06   ///< System control (RW)
#define REG_TR25        0x0C   ///< NTC resistance @ +25 °C, 16-bit LE (Ω)
#define REG_TR50        0x0D   ///< NTC resistance @ +50 °C, 16-bit LE (Ω)
#define REG_TR75        0x0E   ///< NTC resistance @ +75 °C, 16-bit LE (Ω)
#define REG_TR100       0x0F   ///< NTC resistance @ +100 °C, 16-bit LE (Ω)
#define REG_VOLTAGE     0x11   ///< VOUT voltage, 16-bit LE, LSB = 80 mV
#define REG_CURRENT     0x12   ///< VOUT current, 8-bit, LSB = 24 mA
#define REG_TEMP        0x13   ///< Temperature, 8-bit, unit °C (default 0x19 = +25 °C)
#define REG_VREQ        0x14   ///< Negotiated voltage, 16-bit LE, LSB = 50 mV
#define REG_IREQ        0x15   ///< Negotiated current, 16-bit LE, LSB = 10 mA
#define REG_VSELMIN     0x16   ///< HW voltage floor, 8-bit, LSB = 200 mV (default 0x19 = 5000 mV)
#define REG_UVPTHR      0x17   ///< UVP threshold % of VREQ (1=80%, 2=75%, 3=70%)
#define REG_OVPTHR      0x18   ///< OVP offset above VREQ, 8-bit, LSB = 80 mV (default 0x19 = 2000 mV)
#define REG_OCPTHR      0x19   ///< OCP threshold, 8-bit, LSB = 50 mA (0 = 110% of PDO Imax)
#define REG_OTPTHR      0x1A   ///< OTP threshold, unit °C (default 0x78 = +120 °C)
#define REG_DRTHR       0x1B   ///< De-rating threshold, unit °C (default 0x78 = +120 °C)
#define REG_SRCPDO      0x20   ///< Burst-read all 13 PDOs (26 bytes)
#define REG_SPR_PDO1    0x21   ///< SPR PDO 1 … PDO 7 = 0x21 … 0x27
#define REG_EPR_PDO8    0x28   ///< EPR PDO 8 … PDO 13 = 0x28 … 0x2D
#define REG_PD_REQMSG   0x31   ///< PD request message, 16-bit LE
#define REG_PD_CMDMSG   0x32   ///< PD command message (HRST=bit0, DRSWP=bit1)
#define REG_PD_MSGRLT   0x33   ///< PD message result (bit0 = success)
#define REG_GPIO        0x52   ///< GPIO control (FA02 variant only)

// ─────────────────────────────────────────────────────────────────────────────
//  STATUS REGISTER BIT MASKS  (REG_STATUS 0x01)
// ─────────────────────────────────────────────────────────────────────────────
#define STATUS_STARTED  (1 << 0)  ///< System started; configuration window open (100 ms)
#define STATUS_READY    (1 << 1)  ///< Ready to accept new I2C requests
#define STATUS_NEWPDO   (1 << 2)  ///< New source PDO list received
#define STATUS_UVP      (1 << 3)  ///< Under-voltage protection triggered
#define STATUS_OVP      (1 << 4)  ///< Over-voltage protection triggered
#define STATUS_OCP      (1 << 5)  ///< Over-current protection triggered
#define STATUS_OTP      (1 << 6)  ///< Over-temperature protection triggered
#define STATUS_FAULTS   (STATUS_UVP | STATUS_OVP | STATUS_OCP | STATUS_OTP)

// ─────────────────────────────────────────────────────────────────────────────
//  OPMODE REGISTER BIT MASKS  (REG_OPMODE 0x03)
// ─────────────────────────────────────────────────────────────────────────────
#define OPMODE_LGCYMOD  (1 << 0)  ///< Legacy (non-PD) source connected
#define OPMODE_PDMOD    (1 << 1)  ///< PD source connected
#define OPMODE_DATARL   (1 << 5)  ///< Current Data Role
#define OPMODE_DR       (1 << 6)  ///< De-rating active
#define OPMODE_CCFLIP   (1 << 7)  ///< CC2 is the CC wire (cable flipped)

// ─────────────────────────────────────────────────────────────────────────────
//  CONFIG REGISTER BIT MASKS  (REG_CONFIG 0x04)
// ─────────────────────────────────────────────────────────────────────────────
#define CONFIG_UVP_EN   (1 << 3)
#define CONFIG_OVP_EN   (1 << 4)
#define CONFIG_OCP_EN   (1 << 5)
#define CONFIG_OTP_EN   (1 << 6)
#define CONFIG_DR_EN    (1 << 7)
#define CONFIG_ALL_EN   (CONFIG_UVP_EN | CONFIG_OVP_EN | CONFIG_OCP_EN | CONFIG_OTP_EN | CONFIG_DR_EN)

// ─────────────────────────────────────────────────────────────────────────────
//  PDCONFIG REGISTER BIT MASKS  (REG_PDCONFIG 0x05)
// ─────────────────────────────────────────────────────────────────────────────
#define PDCFG_EPR_EN    (1 << 0)  ///< Enable EPR mode
#define PDCFG_PPS_EN    (1 << 1)  ///< Enable PPS/AVS capability
#define PDCFG_DRSWP_EN  (1 << 2)  ///< Accept Data Role Swap

// ─────────────────────────────────────────────────────────────────────────────
//  MASK REGISTER BIT MASKS  (REG_MASK 0x02)
// ─────────────────────────────────────────────────────────────────────────────
#define MASK_STARTED    (1 << 0)
#define MASK_READY      (1 << 1)
#define MASK_NEWPDO     (1 << 2)
#define MASK_UVP        (1 << 3)
#define MASK_OVP        (1 << 4)
#define MASK_OCP        (1 << 5)
#define MASK_OTP        (1 << 6)
#define MASK_ALL_FAULTS (MASK_STARTED | MASK_READY | MASK_NEWPDO | MASK_UVP | MASK_OVP | MASK_OCP | MASK_OTP)

// ─────────────────────────────────────────────────────────────────────────────
//  SYSTEM REGISTER  (REG_SYSTEM 0x06)
// ─────────────────────────────────────────────────────────────────────────────
#define SYSTEM_OUTPUT_ON  0x10
#define SYSTEM_OUTPUT_OFF 0x00

// ─────────────────────────────────────────────────────────────────────────────
//  PD_MSGRLT REGISTER
// ─────────────────────────────────────────────────────────────────────────────
#define MSGRLT_SUCCESS  (1 << 0)  ///< PD negotiation succeeded

// ─────────────────────────────────────────────────────────────────────────────
//  ERROR CODES
// ─────────────────────────────────────────────────────────────────────────────
#define AP_OK           (0)
#define AP_ERR_I2C      (-1)   ///< I2C communication failure
#define AP_ERR_TIMEOUT  (-2)   ///< Polling timed out
#define AP_ERR_RANGE    (-3)   ///< Argument out of valid range
#define AP_ERR_TYPE     (-4)   ///< PDO type mismatch
#define AP_ERR_NO_PDO   (-5)   ///< No suitable PDO found for request

// ─────────────────────────────────────────────────────────────────────────────
//  PDO TYPE ENUM
// ─────────────────────────────────────────────────────────────────────────────
typedef enum : uint8_t {
    PDO_TYPE_NONE  = 0,
    PDO_TYPE_FIXED = 1,
    PDO_TYPE_PPS   = 2,
    PDO_TYPE_AVS   = 3
} PdoType_t;

// ─────────────────────────────────────────────────────────────────────────────
//  FAULT FLAGS
// ─────────────────────────────────────────────────────────────────────────────
typedef enum : uint8_t {
    FAULT_NONE     = 0x00,
    FAULT_OTP      = 0x01,
    FAULT_OCP      = 0x02,
    FAULT_OVP      = 0x04,
    FAULT_UVP      = 0x08
} FaultFlags_t;

// ─────────────────────────────────────────────────────────────────────────────
//  SYSTEM STATUS STRUCT  (returned by getSystemStatus())
// ─────────────────────────────────────────────────────────────────────────────
struct AP33772S_Status {
    bool     started;         ///< Startup window active
    bool     ready;           ///< Ready for commands
    bool     newPDO;          ///< New PDO list available
    bool     pdConnected;     ///< PD source attached
    bool     legacyConnected; ///< Non-PD legacy source
    bool     cableFlipped;    ///< CC2 is CC wire
    bool     deratingActive;  ///< Thermal de-rating in effect
    uint8_t  faultMask;       ///< Bitmask of FaultFlags_t
    char     faultStr[32];    ///< Human-readable fault string
};

// ─────────────────────────────────────────────────────────────────────────────
//  TELEMETRY STRUCT
// ─────────────────────────────────────────────────────────────────────────────
struct AP33772S_Telemetry {
    uint16_t voltage_mV;        ///< Live VOUT voltage (80 mV/LSB)
    uint16_t current_mA;        ///< Live VOUT current (24 mA/LSB)
    uint32_t power_mW;          ///< Calculated power
    int8_t   temperature_C;     ///< NTC temperature (°C)
    uint16_t requestedVoltage_mV; ///< Negotiated voltage (50 mV/LSB)
    uint16_t requestedCurrent_mA; ///< Negotiated current (10 mA/LSB)
};

// ─────────────────────────────────────────────────────────────────────────────
//  PDO INFO STRUCT  (decoded from raw 16-bit PDO word)
// ─────────────────────────────────────────────────────────────────────────────
struct AP33772S_PDO {
    uint8_t  index;           ///< 1-based PDO index (1–13)
    PdoType_t type;           ///< FIXED / PPS / AVS / NONE
    bool     valid;           ///< DETECT bit set
    bool     isEPR;           ///< True for PDO 8–13
    uint16_t minVoltage_mV;   ///< Minimum voltage (APDO) or fixed voltage
    uint16_t maxVoltage_mV;   ///< Maximum voltage
    uint16_t maxCurrent_mA;   ///< Maximum current
    uint8_t  currentCode;     ///< Raw 4-bit CURRENT_MAX code
    uint16_t raw;             ///< Raw 16-bit PDO word
};

// ─────────────────────────────────────────────────────────────────────────────
//  RAW PDO BIT-FIELD UNION  (little-endian: byte0 = bits[7:0])
// ─────────────────────────────────────────────────────────────────────────────
typedef union {
    struct {                        // Fixed & APDO shared layout
        uint16_t voltage_max  : 8;  // bits[7:0]
        uint16_t voltage_min  : 2;  // bits[9:8]  (peak_cur for fixed)
        uint16_t current_max  : 4;  // bits[13:10]
        uint16_t type         : 1;  // bit[14]  0=Fixed 1=APDO
        uint16_t detect       : 1;  // bit[15]
    } fields;
    struct { uint8_t byte0; uint8_t byte1; };
    uint16_t word;
} PDO_RAW_T;

// ─────────────────────────────────────────────────────────────────────────────
//  RDO BIT-FIELD UNION  (PD_REQMSG 0x31, 16-bit LE)
// ─────────────────────────────────────────────────────────────────────────────
typedef union {
    struct {
        uint16_t VOLTAGE_SEL : 8;   // bits[7:0]
        uint16_t CURRENT_SEL : 4;   // bits[11:8]
        uint16_t PDO_INDEX   : 4;   // bits[15:12]
    } fields;
    struct { uint8_t byte0; uint8_t byte1; };
    uint16_t word;
} RDO_T;

// ─────────────────────────────────────────────────────────────────────────────
//  NTC CALIBRATION STRUCT (4-point piecewise-linear)
// ─────────────────────────────────────────────────────────────────────────────
struct AP33772S_NTC {
    uint16_t r25_ohm;   ///< Resistance @ +25 °C (default 10000 Ω)
    uint16_t r50_ohm;   ///< Resistance @ +50 °C (default 4161 Ω)
    uint16_t r75_ohm;   ///< Resistance @ +75 °C (default 1928 Ω)
    uint16_t r100_ohm;  ///< Resistance @ +100 °C (default 974 Ω)
};

// ─────────────────────────────────────────────────────────────────────────────
//  MAIN CLASS
// ─────────────────────────────────────────────────────────────────────────────
class AP33772S {
public:
    // ── Construction & Initialisation ────────────────────────────────────────

    /**
     * @brief Construct with Wire instance and optional INT pin.
     * @param wire    Reference to TwoWire bus (Wire, Wire1, …).
     * @param intPin  Arduino pin connected to AP33772S INT output.
     *                Pass -1 to disable interrupt support.
     */
    explicit AP33772S(TwoWire &wire = Wire, int8_t intPin = -1);

    /**
     * @brief Initialise the AP33772S.
     *
     * Call once after Wire.begin().  Configures MASK, CONFIG, and PDCONFIG
     * registers. Does NOT block.
     *
     * @param enableEPR  Enable EPR mode (requires EPR-capable charger).
     * @param enablePPS  Enable PPS/AVS capability.
     * @return AP_OK on success, AP_ERR_I2C if device not found.
     */
    int8_t begin(bool enableEPR = true, bool enablePPS = true);

    // ── Startup Sequencing ────────────────────────────────────────────────────

    /**
     * @brief Block until STATUS.STARTED is set (100 ms window opens).
     * @param timeoutMs Maximum wait time (ms). Default 2000.
     * @return AP_OK or AP_ERR_TIMEOUT.
     */
    int8_t waitForStartup(uint32_t timeoutMs = 2000);

    /**
     * @brief Block until STATUS.NEWPDO & STATUS.READY are both set.
     * @param timeoutMs Maximum wait time (ms). Default 5000.
     * @return AP_OK or AP_ERR_TIMEOUT.
     */
    int8_t waitForPDOs(uint32_t timeoutMs = 5000);

    // ── PDO Discovery ─────────────────────────────────────────────────────────

    /**
     * @brief Burst-read all 13 PDO slots from REG_SRCPDO (0x20).
     * @return Count of valid (DETECT=1) PDOs found.
     */
    uint8_t readAllPDOs();

    /**
     * @brief Read a single PDO by 1-based index.
     * @param index  1–13.
     * @param pdo    Destination struct.
     * @return true on success.
     */
    bool readPDO(uint8_t index, AP33772S_PDO &pdo);

    /** @brief Return cached decoded PDO by 1-based index. */
    const AP33772S_PDO& getPDO(uint8_t index) const;

    /** @brief Return number of valid PDOs found after last readAllPDOs(). */
    uint8_t validPDOCount() const { return _validPDOCount; }

    /** @brief Return 1-based index of first PPS PDO, or -1 if absent. */
    int8_t getPPSIndex() const;

    /** @brief Return 1-based index of first AVS PDO, or -1 if absent. */
    int8_t getAVSIndex() const;

    /**
     * @brief Pretty-print the PDO table to a Stream (Serial, etc.).
     */
    void printPDOs(Stream &s = Serial);

    // ── Universal Voltage Negotiation ─────────────────────────────────────────

    /**
     * @brief Request any voltage from 3.3 V to 28 V with automatic PDO selection.
     *
     * Decision tree:
     *  1. Exact-match Fixed PDO (preferred for standard rails).
     *  2. PPS APDO for voltages ≤ 21 V.
     *  3. AVS APDO for voltages > 21 V.
     *  4. Nearest Fixed PDO (ceil) as fallback.
     *
     * @param targetVoltage   Desired output voltage in Volts (e.g. 3.3, 9.0, 20.0).
     * @param maxCurrent      Maximum current in Amperes (e.g. 3.0).
     * @return 1-based PDO index used (1–13), or 0 on failure.
     */
    uint8_t setVoltage(float targetVoltage, float maxCurrent = 3.0f);

    // ── Direct PDO Selection ──────────────────────────────────────────────────

    /**
     * @brief Request a Fixed PDO by index.
     * @param pdoIndex    1-based PDO index.
     * @param maxCurrent_mA  Current limit in mA (capped at PDO max).
     * @return AP_OK, AP_ERR_RANGE, or AP_ERR_TYPE.
     */
    int8_t setFixPDO(uint8_t pdoIndex, uint16_t maxCurrent_mA);

    /**
     * @brief Request a PPS APDO with specific voltage.
     * @param pdoIndex    1-based PDO index.
     * @param voltage_mV  Target voltage in mV (100 mV resolution).
     * @param maxCurrent_mA  Current limit in mA.
     * @return AP_OK, AP_ERR_RANGE, or AP_ERR_TYPE.
     */
    int8_t setPPSPDO(uint8_t pdoIndex, uint16_t voltage_mV, uint16_t maxCurrent_mA);

    /**
     * @brief Request an AVS APDO with specific voltage.
     * @param pdoIndex    1-based PDO index.
     * @param voltage_mV  Target voltage in mV (200 mV resolution).
     * @param maxCurrent_mA  Current limit in mA.
     * @return AP_OK, AP_ERR_RANGE, or AP_ERR_TYPE.
     */
    int8_t setAVSPDO(uint8_t pdoIndex, uint16_t voltage_mV, uint16_t maxCurrent_mA);

    // ── Negotiation Status ────────────────────────────────────────────────────

    /**
     * @brief Poll PD_MSGRLT until success bit is set or timeout.
     * @param timeoutMs  Default 2000 ms.
     * @return AP_OK or AP_ERR_NEGO.
     */
    int8_t waitForNegotiation(uint32_t timeoutMs = 2000);

    // ── Output Enable / Hard Reset ────────────────────────────────────────────

    /** @brief Enable or disable the PWR_EN MOSFET output. */
    bool setOutput(bool on);

    /** @brief Issue a USB PD hard reset. Cancels keep-alive. */
    int8_t issueHardReset();

    // ── Keep-Alive Task ───────────────────────────────────────────────────────

    /**
     * @brief Non-blocking keep-alive for PPS/AVS (call in loop()).
     *
     * PPS/AVS RDOs must be refreshed at least once every 10 seconds or the
     * source will drop back to 5 V.  Call task() every loop iteration.
     */
    void task();

    // ── Safety Features ───────────────────────────────────────────────────────

    /**
     * @brief Set hardware voltage floor (REG_VSELMIN 0x16).
     *
     * If the negotiated voltage is below this value the AP33772S keeps
     * the PWR_EN MOSFET OFF, protecting the downstream circuit.
     *
     * @param minVoltage_mV  Floor in mV. Resolution 200 mV.
     *                       Set 0 to disable (allow any voltage).
     * @return true on success.
     */
    bool setVoltageFloor_mV(uint16_t minVoltage_mV);

    /**
     * @brief Configure all four NTC calibration points for temperature sensing.
     *
     * The AP33772S uses piecewise-linear interpolation between these four points.
     * Must be called BEFORE reading TEMP or enabling OTP/DR.
     *
     * Default (Murata NCP03XH103): r25=10000, r50=4161, r75=1928, r100=974 Ω
     *
     * @param ntc  Calibration struct with r25, r50, r75, r100 in Ohms.
     * @return true on success.
     */
    bool configureNTC(const AP33772S_NTC &ntc);

    /**
     * @brief Set OTP (over-temperature protection) threshold.
     * @param threshold_C  Temperature in °C. Default 120 °C.
     * @return true on success.
     */
    bool setOTPThreshold(uint8_t threshold_C = 120);

    /**
     * @brief Set thermal de-rating threshold.
     *
     * When temperature exceeds this level the AP33772S automatically sends
     * a new RDO reducing current by 50%.  Recovery when temp drops 10 °C below.
     *
     * @param threshold_C  Temperature in °C. Default 85 °C (safe operating limit).
     * @return true on success.
     */
    bool setDeratingThreshold(uint8_t threshold_C = 85);

    /**
     * @brief Set OVP offset above the negotiated voltage.
     * @param offset_mV  Offset in mV. Resolution 80 mV. Default 2000 mV.
     * @return true on success.
     */
    bool setOVPOffset_mV(uint16_t offset_mV = 2000);

    /**
     * @brief Set UVP threshold as a fraction of the requested voltage.
     * @param level  1 = 80%, 2 = 75%, 3 = 70%. Default 1.
     * @return true on success.
     */
    bool setUVPLevel(uint8_t level = 1);

    /**
     * @brief Set OCP threshold current.
     * @param threshold_mA  In mA. Resolution 50 mA. 0 = 110% of PDO Imax.
     * @return true on success.
     */
    bool setOCPThreshold_mA(uint16_t threshold_mA = 0);

    /**
     * @brief Enable/disable individual protection modules.
     * @param uvp  Under-voltage protection.
     * @param ovp  Over-voltage protection.
     * @param ocp  Over-current protection.
     * @param otp  Over-temperature protection.
     * @param dr   Thermal de-rating.
     * @return true on success.
     */
    bool setProtectionConfig(bool uvp = true, bool ovp = true,
                             bool ocp = true, bool otp = true, bool dr = true);

    // ── Interrupt Management ──────────────────────────────────────────────────

    /**
     * @brief Configure the MASK register to enable chosen interrupts.
     * @param mask  OR of MASK_* constants.  Default = all events.
     * @return true on success.
     */
    bool setInterruptMask(uint8_t mask = MASK_ALL_FAULTS);

    /**
     * @brief Read and clear the STATUS register (clears INT line).
     *
     * Call this from your hardware ISR or interrupt handler.  STATUS is RC
     * (reset-on-read) per datasheet Table 12.
     *
     * @return Raw STATUS byte captured at the time of the read.
     */
    uint8_t handleInterrupt();

    /**
     * @brief Attach an ISR callback to the INT pin (RISING edge).
     * @param cb  Function pointer (void fn()) to call on interrupt.
     */
    void attachInterruptCallback(void (*cb)());

    // ── Precision Telemetry ───────────────────────────────────────────────────

    /** @brief Populate a telemetry struct with live readings. */
    bool getTelemetry(AP33772S_Telemetry &t);

    uint16_t getVoltage_mV();           ///< VOUT voltage (80 mV/LSB)
    uint16_t getCurrent_mA();           ///< VOUT current (24 mA/LSB)
    uint32_t getPower_mW();             ///< Calculated V×I
    int8_t   getTemperature_C();        ///< NTC temperature in °C
    uint16_t getRequestedVoltage_mV();  ///< Negotiated voltage (50 mV/LSB)
    uint16_t getRequestedCurrent_mA();  ///< Negotiated current (10 mA/LSB)

    // ── Fault Decoding / System Status ────────────────────────────────────────

    /**
     * @brief Read and decode the STATUS and OPMODE registers.
     * @param out  Reference to result struct.
     * @return Raw STATUS byte.
     */
    uint8_t getSystemStatus(AP33772S_Status &out);

    uint8_t getRawStatus();   ///< Raw STATUS register byte
    uint8_t getOpMode();      ///< Raw OPMODE register byte
    uint8_t getMsgResult();   ///< Raw PD_MSGRLT register byte

    bool isPDConnected();       ///< PD source is attached
    bool isLegacyConnected();   ///< Legacy charger attached (non-PD)
    bool isCableFlipped();      ///< Cable is orientation-flipped
    bool isDerating();          ///< Thermal de-rating is active
    bool isFault();             ///< Any protection fault is active

    /** @brief Return human-readable fault string, or "" if none. */
    String getFaultString();

    // ── Advanced / Debug ──────────────────────────────────────────────────────

    /** @brief Dump all key registers to a Stream (Serial). */
    void dumpRegisters(Stream &s = Serial);

    // ── Low-Level Raw I2C Access ──────────────────────────────────────────────
    int16_t  readReg8(uint8_t reg);
    int32_t  readReg16(uint8_t reg);
    bool     writeReg8(uint8_t reg, uint8_t val);
    bool     writeReg16(uint8_t reg, uint16_t val);
    bool     readBytes(uint8_t reg, uint8_t *buf, uint8_t len);

private:
    TwoWire  &_wire;
    int8_t    _intPin;

    // PDO cache
    PDO_RAW_T    _pdoRaw[AP33772S_MAX_PDO];
    AP33772S_PDO _pdoDecoded[AP33772S_MAX_PDO];
    uint8_t      _validPDOCount;

    // Keep-alive state for PPS/AVS (must re-send RDO every <10 s)
    bool     _keepAliveActive;
    uint32_t _keepAliveTimer;
    uint8_t  _keepAlivePDOIndex;
    uint8_t  _keepAliveVoltageSel;
    uint8_t  _keepAliveCurrentSel;

    // Internal helpers
    void    _decodePDO(uint8_t idx);
    void    _sendRDO(uint8_t pdoIndex, uint8_t currentSel, uint8_t voltageSel);
    uint8_t _currentEncode(uint16_t mA);   // mA → 4-bit CURRENT_SEL code
    uint16_t _currentDecode(uint8_t code); // 4-bit code → approximate mA
};

#endif // AP33772S_LIB_H
