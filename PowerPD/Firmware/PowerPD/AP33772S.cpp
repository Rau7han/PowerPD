/**
 *
 * Key encoding facts verified from datasheet Table 2 (PD_REQMSG):
 *   Fixed PDO  : VOLTAGE_SEL = 0x00  (ignored by chip)
 *   PPS APDO   : VOLTAGE_SEL = target_mV / 100   (100 mV per unit, SPR bit15=0)
 *   AVS APDO   : VOLTAGE_SEL = target_mV / 200   (200 mV per unit, EPR bit15=1)
 *   CURRENT_SEL: 0x0=1.00A … 0xF=5.00A  (250 mA steps from code 1 upward)
 *
 * ADC scaling (Table 19):
 *   VOLTAGE  0x11 : 16-bit LE, 1 LSB = 80 mV
 *   CURRENT  0x12 :  8-bit,    1 LSB = 24 mA
 *   VREQ     0x14 : 16-bit LE, 1 LSB = 50 mV
 *   IREQ     0x15 : 16-bit LE, 1 LSB = 10 mA
 *   VSELMIN  0x16 :  8-bit,    1 LSB = 200 mV  (default 0x19 = 5000 mV)
 *   OVPTHR   0x18 :  8-bit,    1 LSB = 80 mV   (default 0x19 = 2000 mV)
 *   OCPTHR   0x19 :  8-bit,    1 LSB = 50 mA
 */

#include "AP33772S.h"

// ─────────────────────────────────────────────────────────────────────────────
//  CONSTRUCTION
// ─────────────────────────────────────────────────────────────────────────────
AP33772S::AP33772S(TwoWire &wire, int8_t intPin)
    : _wire(wire),
      _intPin(intPin),
      _validPDOCount(0),
      _keepAliveActive(false),
      _keepAliveTimer(0),
      _keepAlivePDOIndex(0),
      _keepAliveVoltageSel(0),
      _keepAliveCurrentSel(0)
{
    memset(_pdoRaw,     0, sizeof(_pdoRaw));
    memset(_pdoDecoded, 0, sizeof(_pdoDecoded));
}

// ─────────────────────────────────────────────────────────────────────────────
//  begin()
// ─────────────────────────────────────────────────────────────────────────────
int8_t AP33772S::begin(bool enableEPR, bool enablePPS)
{
    // Probe device
    _wire.beginTransmission(AP33772S_I2C_ADDR);
    if (_wire.endTransmission() != 0) return AP_ERR_I2C;

    // Configure INT pin
    if (_intPin >= 0) pinMode(_intPin, INPUT);

    // PDCONFIG (0x05): enable EPR and/or PPS as requested
    uint8_t pdcfg = 0;
    if (enableEPR) pdcfg |= PDCFG_EPR_EN;
    if (enablePPS) pdcfg |= PDCFG_PPS_EN;
    if (!writeReg8(REG_PDCONFIG, pdcfg))          return AP_ERR_I2C;

    // CONFIG (0x04): enable all protections by default
    if (!writeReg8(REG_CONFIG,   CONFIG_ALL_EN))   return AP_ERR_I2C;

    // MASK (0x02): unmask STARTED, READY, NEWPDO, and all faults
    if (!writeReg8(REG_MASK,     MASK_ALL_FAULTS)) return AP_ERR_I2C;

    return AP_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
//  waitForStartup()
// ─────────────────────────────────────────────────────────────────────────────
int8_t AP33772S::waitForStartup(uint32_t timeoutMs)
{
    uint32_t deadline = millis() + timeoutMs;
    while (millis() < deadline) {
        int16_t s = readReg8(REG_STATUS);
        if (s >= 0 && ((uint8_t)s & STATUS_STARTED)) return AP_OK;
        delay(10);
    }
    return AP_ERR_TIMEOUT;
}

// ─────────────────────────────────────────────────────────────────────────────
//  waitForPDOs()
// ─────────────────────────────────────────────────────────────────────────────
int8_t AP33772S::waitForPDOs(uint32_t timeoutMs)
{
    uint32_t deadline = millis() + timeoutMs;
    while (millis() < deadline) {
        int16_t s = readReg8(REG_STATUS);
        if (s >= 0) {
            uint8_t st = (uint8_t)s;
            if ((st & STATUS_NEWPDO) && (st & STATUS_READY)) return AP_OK;
        }
        delay(50);
    }
    return AP_ERR_TIMEOUT;
}

// ─────────────────────────────────────────────────────────────────────────────
//  _currentEncode()  —  mA → 4-bit CURRENT_SEL code
//
//  Per datasheet Table 2 / official example firmware:
//    code 0  → < 1.25 A (nominally 1.00 A)
//    code 1  → 1.25 A
//    code 2  → 1.50 A
//    …
//    code 15 → 5.00 A
//  Each step above code 0 is 250 mA.
// ─────────────────────────────────────────────────────────────────────────────
uint8_t AP33772S::_currentEncode(uint16_t mA)
{
    if (mA < 1250) return 0;
    if (mA >= 5000) return 15;
    int code = (((int)mA - 1250) / 250) + 1;
    return (code > 15) ? 15 : (uint8_t)code;
}

// ─────────────────────────────────────────────────────────────────────────────
//  _currentDecode()  —  4-bit code → approximate mA
// ─────────────────────────────────────────────────────────────────────────────
uint16_t AP33772S::_currentDecode(uint8_t code)
{
    if (code == 0)  return 1000;
    if (code == 15) return 5000;
    return (uint16_t)(1250 + (code - 1) * 250);
}

// ─────────────────────────────────────────────────────────────────────────────
//  _decodePDO()  —  parse raw 16-bit PDO word into AP33772S_PDO struct
//
//  PDO word layout (datasheet – SRC_SPRandEPR_PDO_Fields):
//    bits[7:0]   voltage_max
//    bits[9:8]   voltage_min (APDO) or peak_current (Fixed)
//    bits[13:10] current_max (4-bit range code)
//    bit[14]     type  (0 = Fixed, 1 = APDO)
//    bit[15]     detect / valid
//
//  Voltage scaling:
//    SPR Fixed  : voltage_max × 100 mV
//    EPR Fixed  : voltage_max × 200 mV
//    PPS        : voltage_max × 100 mV  (SPR slot)
//    AVS        : voltage_max × 200 mV  (EPR slot)
// ─────────────────────────────────────────────────────────────────────────────
void AP33772S::_decodePDO(uint8_t idx)
{
    // idx is 0-based (0–12 → PDO1–PDO13)
    const PDO_RAW_T  &raw = _pdoRaw[idx];
    AP33772S_PDO     &dec = _pdoDecoded[idx];

    dec.index = idx + 1;
    dec.raw   = raw.word;
    dec.valid = (raw.fields.detect == 1);
    dec.isEPR = (idx >= AP33772S_SPR_PDO_COUNT); // slots 7–12 = EPR

    if (!dec.valid) {
        dec.type = PDO_TYPE_NONE;
        dec.minVoltage_mV = dec.maxVoltage_mV = dec.maxCurrent_mA = 0;
        dec.currentCode   = 0;
        return;
    }

    dec.currentCode   = raw.fields.current_max;
    dec.maxCurrent_mA = _currentDecode(dec.currentCode);

    if (raw.fields.type == 0) {
        // ── Fixed PDO ────────────────────────────────────────────────────────
        dec.type = PDO_TYPE_FIXED;
        uint16_t scale    = dec.isEPR ? 200u : 100u;
        dec.maxVoltage_mV = (uint16_t)raw.fields.voltage_max * scale;
        dec.minVoltage_mV = dec.maxVoltage_mV;   // Fixed = single voltage point

    } else if (!dec.isEPR) {
        // ── PPS APDO (type=1, SPR slot, index 1–7) ───────────────────────────
        dec.type          = PDO_TYPE_PPS;
        dec.maxVoltage_mV = (uint16_t)raw.fields.voltage_max * 100u;

        // voltage_min field interpretation per datasheet:
        //   0 = reserved, 1 = ≥3300 mV, 2 = 3300 < min ≤ 5000 mV, 3 = other
        switch (raw.fields.voltage_min) {
            case 2:  dec.minVoltage_mV = 5000; break;
            default: dec.minVoltage_mV = 3300; break;  // conservative: 3.3 V floor
        }

    } else {
        // ── AVS APDO (type=1, EPR slot, index 8–13) ──────────────────────────
        dec.type          = PDO_TYPE_AVS;
        dec.maxVoltage_mV = (uint16_t)raw.fields.voltage_max * 200u;

        // voltage_min field interpretation:
        //   0 = reserved, 1 = ≥15000 mV, 2 = 15000 < min ≤ 20000 mV, 3 = other
        switch (raw.fields.voltage_min) {
            case 2:  dec.minVoltage_mV = 20000; break;
            default: dec.minVoltage_mV = 15000; break;  // conservative: 15 V floor
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  readAllPDOs()
// ─────────────────────────────────────────────────────────────────────────────
uint8_t AP33772S::readAllPDOs()
{
    // REG_SRCPDO (0x20) returns 26 bytes = 13 PDOs × 2 bytes, little-endian
    uint8_t buf[AP33772S_SRCPDO_BYTES];
    if (!readBytes(REG_SRCPDO, buf, AP33772S_SRCPDO_BYTES)) return 0;

    _validPDOCount = 0;
    for (uint8_t i = 0; i < AP33772S_MAX_PDO; i++) {
        _pdoRaw[i].byte0 = buf[i * 2];
        _pdoRaw[i].byte1 = buf[i * 2 + 1];
        _decodePDO(i);
        if (_pdoDecoded[i].valid) _validPDOCount++;
    }
    return _validPDOCount;
}

// ─────────────────────────────────────────────────────────────────────────────
//  readPDO()
// ─────────────────────────────────────────────────────────────────────────────
bool AP33772S::readPDO(uint8_t index, AP33772S_PDO &pdo)
{
    if (index < 1 || index > 13) return false;
    uint8_t reg = REG_SPR_PDO1 + (index - 1);   // 0x21 … 0x2D
    uint8_t buf[2];
    if (!readBytes(reg, buf, 2)) return false;

    uint8_t i = index - 1;
    _pdoRaw[i].byte0 = buf[0];
    _pdoRaw[i].byte1 = buf[1];
    _decodePDO(i);
    pdo = _pdoDecoded[i];
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  getPDO() / getPPSIndex() / getAVSIndex()
// ─────────────────────────────────────────────────────────────────────────────
const AP33772S_PDO& AP33772S::getPDO(uint8_t index) const
{
    if (index < 1 || index > 13) index = 13;
    return _pdoDecoded[index - 1];
}

int8_t AP33772S::getPPSIndex() const
{
    for (uint8_t i = 0; i < AP33772S_MAX_PDO; i++)
        if (_pdoDecoded[i].valid && _pdoDecoded[i].type == PDO_TYPE_PPS)
            return (int8_t)_pdoDecoded[i].index;
    return -1;
}

int8_t AP33772S::getAVSIndex() const
{
    for (uint8_t i = 0; i < AP33772S_MAX_PDO; i++)
        if (_pdoDecoded[i].valid && _pdoDecoded[i].type == PDO_TYPE_AVS)
            return (int8_t)_pdoDecoded[i].index;
    return -1;
}

// ─────────────────────────────────────────────────────────────────────────────
//  printPDOs()
// ─────────────────────────────────────────────────────────────────────────────
void AP33772S::printPDOs(Stream &s)
{
    s.println(F("┌─────┬──────────┬─────────┬────────────┬────────────┬────────────┬────────┐"));
    s.println(F("│ IDX │ TYPE     │  SLOT   │ MIN V (mV) │ MAX V (mV) │ MAX I (mA) │  RAW   │"));
    s.println(F("├─────┼──────────┼─────────┼────────────┼────────────┼────────────┼────────┤"));
    for (uint8_t i = 0; i < AP33772S_MAX_PDO; i++) {
        const AP33772S_PDO &p = _pdoDecoded[i];
        if (!p.valid) continue;
        const char *ts   = (p.type == PDO_TYPE_FIXED) ? "Fixed" :
                           (p.type == PDO_TYPE_PPS)   ? "PPS"   : "AVS";
        const char *slot = p.isEPR ? "EPR" : "SPR";
        char row[112];
        snprintf(row, sizeof(row),
                 "│ %-3d │ %-8s │ %-7s │ %-10u │ %-10u │ %-10u │ 0x%04X │",
                 p.index, ts, slot,
                 p.minVoltage_mV, p.maxVoltage_mV, p.maxCurrent_mA, p.raw);
        s.println(row);
    }
    s.println(F("└─────┴──────────┴─────────┴────────────┴────────────┴────────────┴────────┘"));
}

// ─────────────────────────────────────────────────────────────────────────────
//  _sendRDO()  —  write PD_REQMSG (0x31), 16-bit little-endian
//
//  RDO layout per Table 2:
//    bits[7:0]   = VOLTAGE_SEL
//    bits[11:8]  = CURRENT_SEL
//    bits[15:12] = PDO_INDEX
//
//  Little-endian wire order:
//    byte0 = bits[7:0]  = VOLTAGE_SEL
//    byte1 = bits[15:8] = (PDO_INDEX<<4) | CURRENT_SEL
// ─────────────────────────────────────────────────────────────────────────────
void AP33772S::_sendRDO(uint8_t pdoIndex, uint8_t currentSel, uint8_t voltageSel)
{
    RDO_T rdo;
    rdo.fields.PDO_INDEX   = pdoIndex  & 0x0F;
    rdo.fields.CURRENT_SEL = currentSel & 0x0F;
    rdo.fields.VOLTAGE_SEL = voltageSel;

    _wire.beginTransmission(AP33772S_I2C_ADDR);
    _wire.write(REG_PD_REQMSG);
    _wire.write(rdo.byte0);   // LE: low byte first
    _wire.write(rdo.byte1);
    _wire.endTransmission();
}

// ─────────────────────────────────────────────────────────────────────────────
//  setVoltage()  —  Universal voltage negotiation
// ─────────────────────────────────────────────────────────────────────────────
uint8_t AP33772S::setVoltage(float targetVoltage, float maxCurrent)
{
    // Refresh PDO cache if empty
    if (_validPDOCount == 0) readAllPDOs();
    if (_validPDOCount == 0) return 0;

    uint16_t target_mV   = (uint16_t)(targetVoltage * 1000.0f + 0.5f);
    uint16_t minCur_mA   = (uint16_t)(maxCurrent    * 1000.0f + 0.5f);

    // ── Pass 1: Exact-match Fixed PDO ────────────────────────────────────────
    for (uint8_t i = 0; i < AP33772S_MAX_PDO; i++) {
        const AP33772S_PDO &p = _pdoDecoded[i];
        if (!p.valid || p.type != PDO_TYPE_FIXED) continue;
        if (p.maxVoltage_mV != target_mV)          continue;
        if (p.maxCurrent_mA < minCur_mA)            continue;
        if (setFixPDO(p.index, minCur_mA) == AP_OK) return p.index;
    }

    // ── Pass 2: PPS (≤21 V) or AVS (>21 V) ──────────────────────────────────
    {
        uint8_t  bestIdx = 0;
        uint16_t bestI   = 0;

        // For voltages above 21 V, prefer AVS; otherwise prefer PPS
        bool wantAVS = (target_mV > 21000);

        for (uint8_t i = 0; i < AP33772S_MAX_PDO; i++) {
            const AP33772S_PDO &p = _pdoDecoded[i];
            if (!p.valid || p.type == PDO_TYPE_FIXED) continue;

            // Type filter: AVS for EPR range, PPS for SPR range
            if (wantAVS && p.type != PDO_TYPE_AVS) continue;
            if (!wantAVS && p.type != PDO_TYPE_PPS) continue;

            if (p.maxVoltage_mV < target_mV)  continue;
            if (p.minVoltage_mV > target_mV)  continue;
            if (p.maxCurrent_mA < minCur_mA)  continue;

            if (p.maxCurrent_mA > bestI) {
                bestI   = p.maxCurrent_mA;
                bestIdx = p.index;
            }
        }

        if (bestIdx) {
            const AP33772S_PDO &p = _pdoDecoded[bestIdx - 1];
            int8_t rc = (p.type == PDO_TYPE_PPS)
                ? setPPSPDO(bestIdx, target_mV, minCur_mA)
                : setAVSPDO(bestIdx, target_mV, minCur_mA);
            if (rc == AP_OK) return bestIdx;
        }

        // Fallback: if primary type unavailable, try the other APDO type
        for (uint8_t i = 0; i < AP33772S_MAX_PDO; i++) {
            const AP33772S_PDO &p = _pdoDecoded[i];
            if (!p.valid || p.type == PDO_TYPE_FIXED) continue;
            if (p.maxVoltage_mV < target_mV) continue;
            if (p.minVoltage_mV > target_mV) continue;
            if (p.maxCurrent_mA < minCur_mA) continue;
            if (p.maxCurrent_mA > bestI) {
                bestI   = p.maxCurrent_mA;
                bestIdx = p.index;
            }
        }

        if (bestIdx) {
            const AP33772S_PDO &p = _pdoDecoded[bestIdx - 1];
            int8_t rc = (p.type == PDO_TYPE_PPS)
                ? setPPSPDO(bestIdx, target_mV, minCur_mA)
                : setAVSPDO(bestIdx, target_mV, minCur_mA);
            if (rc == AP_OK) return bestIdx;
        }
    }

    // ── Pass 3: Nearest Fixed PDO (ceiling) as last resort ───────────────────
    {
        uint8_t  nearIdx  = 0;
        uint32_t nearDiff = UINT32_MAX;
        for (uint8_t i = 0; i < AP33772S_MAX_PDO; i++) {
            const AP33772S_PDO &p = _pdoDecoded[i];
            if (!p.valid || p.type != PDO_TYPE_FIXED)  continue;
            if (p.maxVoltage_mV < target_mV)            continue;
            if (p.maxCurrent_mA < minCur_mA)            continue;
            uint32_t diff = (uint32_t)p.maxVoltage_mV - target_mV;
            if (diff < nearDiff) { nearDiff = diff; nearIdx = p.index; }
        }
        if (nearIdx) {
            if (setFixPDO(nearIdx, minCur_mA) == AP_OK) return nearIdx;
        }
    }

    return 0;  // No suitable PDO found
}

// ─────────────────────────────────────────────────────────────────────────────
//  setFixPDO()
// ─────────────────────────────────────────────────────────────────────────────
int8_t AP33772S::setFixPDO(uint8_t pdoIndex, uint16_t maxCurrent_mA)
{
    if (pdoIndex < 1 || pdoIndex > 13)         return AP_ERR_RANGE;
    const AP33772S_PDO &p = _pdoDecoded[pdoIndex - 1];
    if (!p.valid)                               return AP_ERR_RANGE;
    if (p.type != PDO_TYPE_FIXED)              return AP_ERR_TYPE;

    uint8_t curSel = _currentEncode(maxCurrent_mA);
    if (curSel > p.currentCode) curSel = p.currentCode;  // cap at PDO max

    _keepAliveActive = false;  // Fixed PDOs do not need keep-alive
    _sendRDO(pdoIndex, curSel, 0x00);
    return AP_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
//  setPPSPDO()
// ─────────────────────────────────────────────────────────────────────────────
int8_t AP33772S::setPPSPDO(uint8_t pdoIndex, uint16_t voltage_mV, uint16_t maxCurrent_mA)
{
    if (pdoIndex < 1 || pdoIndex > 13)         return AP_ERR_RANGE;
    const AP33772S_PDO &p = _pdoDecoded[pdoIndex - 1];
    if (!p.valid)                               return AP_ERR_RANGE;
    if (p.type != PDO_TYPE_PPS)                return AP_ERR_TYPE;
    if (voltage_mV < p.minVoltage_mV ||
        voltage_mV > p.maxVoltage_mV)          return AP_ERR_RANGE;

    // Round to nearest 100 mV grid
    voltage_mV = (voltage_mV / 100) * 100;

    uint8_t curSel = _currentEncode(maxCurrent_mA);
    if (curSel > p.currentCode) curSel = p.currentCode;

    // PPS VOLTAGE_SEL: output voltage in 100 mV/unit (datasheet Table 2, SPR path)
    uint8_t volSel = (uint8_t)(voltage_mV / 100);

    // Arm keep-alive (PPS/AVS must be refreshed < every 10 s)
    _keepAliveActive     = true;
    _keepAliveTimer      = millis();
    _keepAlivePDOIndex   = pdoIndex;
    _keepAliveVoltageSel = volSel;
    _keepAliveCurrentSel = curSel;

    _sendRDO(pdoIndex, curSel, volSel);
    return AP_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
//  setAVSPDO()
// ─────────────────────────────────────────────────────────────────────────────
int8_t AP33772S::setAVSPDO(uint8_t pdoIndex, uint16_t voltage_mV, uint16_t maxCurrent_mA)
{
    if (pdoIndex < 1 || pdoIndex > 13)         return AP_ERR_RANGE;
    const AP33772S_PDO &p = _pdoDecoded[pdoIndex - 1];
    if (!p.valid)                               return AP_ERR_RANGE;
    if (p.type != PDO_TYPE_AVS)                return AP_ERR_TYPE;
    if (voltage_mV < p.minVoltage_mV ||
        voltage_mV > p.maxVoltage_mV)          return AP_ERR_RANGE;

    // Round to nearest 200 mV grid
    voltage_mV = (voltage_mV / 200) * 200;

    uint8_t curSel = _currentEncode(maxCurrent_mA);
    if (curSel > p.currentCode) curSel = p.currentCode;

    // AVS VOLTAGE_SEL: output voltage in 200 mV/unit (datasheet Table 2, EPR path)
    uint8_t volSel = (uint8_t)(voltage_mV / 200);

    _keepAliveActive     = true;
    _keepAliveTimer      = millis();
    _keepAlivePDOIndex   = pdoIndex;
    _keepAliveVoltageSel = volSel;
    _keepAliveCurrentSel = curSel;

    _sendRDO(pdoIndex, curSel, volSel);
    return AP_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
//  waitForNegotiation()
// ─────────────────────────────────────────────────────────────────────────────
int8_t AP33772S::waitForNegotiation(uint32_t timeoutMs)
{
    uint32_t deadline = millis() + timeoutMs;
    while (millis() < deadline) {
        int16_t r = readReg8(REG_PD_MSGRLT);
        if (r >= 0 && ((uint8_t)r & MSGRLT_SUCCESS)) return AP_OK;
        delay(50);
    }
    return AP_ERR_TIMEOUT;
}

// ─────────────────────────────────────────────────────────────────────────────
//  issueHardReset()
// ─────────────────────────────────────────────────────────────────────────────
int8_t AP33772S::issueHardReset()
{
    _keepAliveActive = false;
    _validPDOCount   = 0;
    // PD_CMDMSG bit0 = HRST
    return writeReg8(REG_PD_CMDMSG, 0x01) ? AP_OK : AP_ERR_I2C;
}

// ─────────────────────────────────────────────────────────────────────────────
//  task()  —  non-blocking keep-alive; call every loop()
// ─────────────────────────────────────────────────────────────────────────────
void AP33772S::task()
{
    if (!_keepAliveActive) return;
    if (millis() - _keepAliveTimer >= AP33772S_KEEPALIVE_MS) {
        _keepAliveTimer = millis();
        _sendRDO(_keepAlivePDOIndex, _keepAliveCurrentSel, _keepAliveVoltageSel);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  setOutput()
// ─────────────────────────────────────────────────────────────────────────────
bool AP33772S::setOutput(bool on)
{
    return writeReg8(REG_SYSTEM, on ? SYSTEM_OUTPUT_ON : SYSTEM_OUTPUT_OFF);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Safety / Protection Configuration
// ─────────────────────────────────────────────────────────────────────────────

bool AP33772S::setVoltageFloor_mV(uint16_t minVoltage_mV)
{
    // REG_VSELMIN (0x16): 8-bit, 1 LSB = 200 mV.  0 = disabled.
    uint8_t val = (minVoltage_mV == 0) ? 0 : (uint8_t)(minVoltage_mV / 200);
    return writeReg8(REG_VSELMIN, val);
}

bool AP33772S::configureNTC(const AP33772S_NTC &ntc)
{
    // Each TR register is 16-bit little-endian, value in Ohms
    return writeReg16(REG_TR25,  ntc.r25_ohm)  &&
           writeReg16(REG_TR50,  ntc.r50_ohm)  &&
           writeReg16(REG_TR75,  ntc.r75_ohm)  &&
           writeReg16(REG_TR100, ntc.r100_ohm);
}

bool AP33772S::setOTPThreshold(uint8_t threshold_C)
{
    return writeReg8(REG_OTPTHR, threshold_C);
}

bool AP33772S::setDeratingThreshold(uint8_t threshold_C)
{
    return writeReg8(REG_DRTHR, threshold_C);
}

bool AP33772S::setOVPOffset_mV(uint16_t offset_mV)
{
    // REG_OVPTHR (0x18): 8-bit, 1 LSB = 80 mV
    return writeReg8(REG_OVPTHR, (uint8_t)(offset_mV / 80));
}

bool AP33772S::setUVPLevel(uint8_t level)
{
    if (level < 1 || level > 3) return false;
    return writeReg8(REG_UVPTHR, level);
}

bool AP33772S::setOCPThreshold_mA(uint16_t threshold_mA)
{
    // REG_OCPTHR (0x19): 8-bit, 1 LSB = 50 mA.  0 = 110% of PDO Imax.
    return writeReg8(REG_OCPTHR, (uint8_t)(threshold_mA / 50));
}

bool AP33772S::setProtectionConfig(bool uvp, bool ovp, bool ocp, bool otp, bool dr)
{
    uint8_t cfg = 0;
    if (uvp) cfg |= CONFIG_UVP_EN;
    if (ovp) cfg |= CONFIG_OVP_EN;
    if (ocp) cfg |= CONFIG_OCP_EN;
    if (otp) cfg |= CONFIG_OTP_EN;
    if (dr)  cfg |= CONFIG_DR_EN;
    return writeReg8(REG_CONFIG, cfg);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Interrupt Management
// ─────────────────────────────────────────────────────────────────────────────

bool AP33772S::setInterruptMask(uint8_t mask)
{
    return writeReg8(REG_MASK, mask);
}

uint8_t AP33772S::handleInterrupt()
{
    // Reading STATUS (0x01) is RC (reset-on-read) — this clears the INT line
    int16_t s = readReg8(REG_STATUS);
    return (s < 0) ? 0 : (uint8_t)s;
}

void AP33772S::attachInterruptCallback(void (*cb)())
{
    if (_intPin >= 0 && cb)
        ::attachInterrupt(digitalPinToInterrupt(_intPin), cb, RISING);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Precision Telemetry
// ─────────────────────────────────────────────────────────────────────────────

bool AP33772S::getTelemetry(AP33772S_Telemetry &t)
{
    t.voltage_mV          = getVoltage_mV();
    t.current_mA          = getCurrent_mA();
    t.power_mW            = getPower_mW();
    t.temperature_C       = getTemperature_C();
    t.requestedVoltage_mV = getRequestedVoltage_mV();
    t.requestedCurrent_mA = getRequestedCurrent_mA();
    return true;
}

uint16_t AP33772S::getVoltage_mV()
{
    // 16-bit LE, 1 LSB = 80 mV
    int32_t r = readReg16(REG_VOLTAGE);
    return (r < 0) ? 0 : (uint16_t)((uint16_t)r * 80u);
}

uint16_t AP33772S::getCurrent_mA()
{
    // 8-bit, 1 LSB = 24 mA
    int16_t r = readReg8(REG_CURRENT);
    return (r < 0) ? 0 : (uint16_t)((uint8_t)r * 24u);
}

uint32_t AP33772S::getPower_mW()
{
    return (uint32_t)getVoltage_mV() * getCurrent_mA() / 1000UL;
}

int8_t AP33772S::getTemperature_C()
{
    // 8-bit signed, unit °C
    int16_t r = readReg8(REG_TEMP);
    return (r < 0) ? 25 : (int8_t)(uint8_t)r;
}

uint16_t AP33772S::getRequestedVoltage_mV()
{
    // 16-bit LE, 1 LSB = 50 mV
    int32_t r = readReg16(REG_VREQ);
    return (r < 0) ? 0 : (uint16_t)((uint16_t)r * 50u);
}

uint16_t AP33772S::getRequestedCurrent_mA()
{
    // 16-bit LE, 1 LSB = 10 mA
    int32_t r = readReg16(REG_IREQ);
    return (r < 0) ? 0 : (uint16_t)((uint16_t)r * 10u);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Fault Decoding / System Status
// ─────────────────────────────────────────────────────────────────────────────

uint8_t AP33772S::getSystemStatus(AP33772S_Status &out)
{
    uint8_t rawStatus = getRawStatus();
    uint8_t opmode    = getOpMode();

    out.started         = (rawStatus & STATUS_STARTED) != 0;
    out.ready           = (rawStatus & STATUS_READY)   != 0;
    out.newPDO          = (rawStatus & STATUS_NEWPDO)  != 0;
    out.pdConnected     = (opmode    & OPMODE_PDMOD)   != 0;
    out.legacyConnected = (opmode    & OPMODE_LGCYMOD) != 0;
    out.cableFlipped    = (opmode    & OPMODE_CCFLIP)  != 0;
    out.deratingActive  = (opmode    & OPMODE_DR)      != 0;

    // Build fault bitmask
    out.faultMask = 0;
    if (rawStatus & STATUS_OTP) out.faultMask |= (uint8_t)FAULT_OTP;
    if (rawStatus & STATUS_OCP) out.faultMask |= (uint8_t)FAULT_OCP;
    if (rawStatus & STATUS_OVP) out.faultMask |= (uint8_t)FAULT_OVP;
    if (rawStatus & STATUS_UVP) out.faultMask |= (uint8_t)FAULT_UVP;

    // Human-readable fault string
    out.faultStr[0] = '\0';
    if (out.faultMask == 0) {
        strncpy(out.faultStr, "NONE", sizeof(out.faultStr));
    } else {
        String s = "";
        if (rawStatus & STATUS_OTP) s += "OTP_FAULT ";
        if (rawStatus & STATUS_OCP) s += "OCP_FAULT ";
        if (rawStatus & STATUS_OVP) s += "OVP_FAULT ";
        if (rawStatus & STATUS_UVP) s += "UVP_FAULT ";
        s.trim();
        strncpy(out.faultStr, s.c_str(), sizeof(out.faultStr) - 1);
        out.faultStr[sizeof(out.faultStr) - 1] = '\0';
    }

    return rawStatus;
}

uint8_t AP33772S::getRawStatus()
{
    int16_t v = readReg8(REG_STATUS);
    return (v < 0) ? 0 : (uint8_t)v;
}

uint8_t AP33772S::getOpMode()
{
    int16_t v = readReg8(REG_OPMODE);
    return (v < 0) ? 0 : (uint8_t)v;
}

uint8_t AP33772S::getMsgResult()
{
    int16_t v = readReg8(REG_PD_MSGRLT);
    return (v < 0) ? 0 : (uint8_t)v;
}

bool AP33772S::isPDConnected()       { return (getOpMode() & OPMODE_PDMOD)   != 0; }
bool AP33772S::isLegacyConnected()   { return (getOpMode() & OPMODE_LGCYMOD) != 0; }
bool AP33772S::isCableFlipped()      { return (getOpMode() & OPMODE_CCFLIP)  != 0; }
bool AP33772S::isDerating()          { return (getOpMode() & OPMODE_DR)      != 0; }
bool AP33772S::isFault()             { return (getRawStatus() & STATUS_FAULTS) != 0; }

String AP33772S::getFaultString()
{
    uint8_t s = getRawStatus();
    if (!(s & STATUS_FAULTS)) return String("NONE");
    String out = "";
    if (s & STATUS_OTP) out += "OTP_FAULT ";
    if (s & STATUS_OCP) out += "OCP_FAULT ";
    if (s & STATUS_OVP) out += "OVP_FAULT ";
    if (s & STATUS_UVP) out += "UVP_FAULT ";
    out.trim();
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
//  dumpRegisters()  —  debug utility
// ─────────────────────────────────────────────────────────────────────────────
void AP33772S::dumpRegisters(Stream &s)
{
    struct { uint8_t addr; uint8_t width; const char *name; } regs[] = {
        {REG_STATUS,  1, "STATUS  "},
        {REG_MASK,    1, "MASK    "},
        {REG_OPMODE,  1, "OPMODE  "},
        {REG_CONFIG,  1, "CONFIG  "},
        {REG_PDCONFIG,1, "PDCONFIG"},
        {REG_SYSTEM,  1, "SYSTEM  "},
        {REG_TR25,    2, "TR25    "},
        {REG_TR50,    2, "TR50    "},
        {REG_TR75,    2, "TR75    "},
        {REG_TR100,   2, "TR100   "},
        {REG_VOLTAGE, 2, "VOLTAGE "},
        {REG_CURRENT, 1, "CURRENT "},
        {REG_TEMP,    1, "TEMP    "},
        {REG_VREQ,    2, "VREQ    "},
        {REG_IREQ,    2, "IREQ    "},
        {REG_VSELMIN, 1, "VSELMIN "},
        {REG_UVPTHR,  1, "UVPTHR  "},
        {REG_OVPTHR,  1, "OVPTHR  "},
        {REG_OCPTHR,  1, "OCPTHR  "},
        {REG_OTPTHR,  1, "OTPTHR  "},
        {REG_DRTHR,   1, "DRTHR   "},
        {REG_PD_MSGRLT,1,"MSGRLT  "},
    };

    s.println(F("──────────── AP33772S Register Dump ────────────"));
    char buf[56];
    for (auto &r : regs) {
        if (r.width == 1) {
            int16_t v = readReg8(r.addr);
            snprintf(buf, sizeof(buf), "  0x%02X  %s = 0x%02X  (%d)",
                     r.addr, r.name,
                     (unsigned)(v < 0 ? 0xFF : (uint8_t)v),
                     (int)(v < 0 ? 0 : (uint8_t)v));
        } else {
            int32_t v = readReg16(r.addr);
            snprintf(buf, sizeof(buf), "  0x%02X  %s = 0x%04X  (%d)",
                     r.addr, r.name,
                     (unsigned)(v < 0 ? 0xFFFF : (uint16_t)v),
                     (int)(v < 0 ? 0 : (uint16_t)v));
        }
        s.println(buf);
    }
    s.println(F("─────────────────────────────────────────────────"));
}

// ─────────────────────────────────────────────────────────────────────────────
//  Raw I2C primitives  (little-endian per datasheet Figure 5)
// ─────────────────────────────────────────────────────────────────────────────

int16_t AP33772S::readReg8(uint8_t reg)
{
    _wire.beginTransmission(AP33772S_I2C_ADDR);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0) return -1;
    if (_wire.requestFrom((uint8_t)AP33772S_I2C_ADDR, (uint8_t)1) < 1) return -1;
    return (int16_t)(uint8_t)_wire.read();
}

int32_t AP33772S::readReg16(uint8_t reg)
{
    _wire.beginTransmission(AP33772S_I2C_ADDR);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0) return -1;
    if (_wire.requestFrom((uint8_t)AP33772S_I2C_ADDR, (uint8_t)2) < 2) return -1;
    uint8_t lo = (uint8_t)_wire.read();
    uint8_t hi = (uint8_t)_wire.read();
    return (int32_t)(uint16_t)((uint16_t)lo | ((uint16_t)hi << 8));
}

bool AP33772S::writeReg8(uint8_t reg, uint8_t val)
{
    _wire.beginTransmission(AP33772S_I2C_ADDR);
    _wire.write(reg);
    _wire.write(val);
    return (_wire.endTransmission() == 0);
}

bool AP33772S::writeReg16(uint8_t reg, uint16_t val)
{
    _wire.beginTransmission(AP33772S_I2C_ADDR);
    _wire.write(reg);
    _wire.write((uint8_t)(val & 0xFF));   // LE: low byte first
    _wire.write((uint8_t)(val >> 8));
    return (_wire.endTransmission() == 0);
}

bool AP33772S::readBytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
    _wire.beginTransmission(AP33772S_I2C_ADDR);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0) return false;
    if (_wire.requestFrom((uint8_t)AP33772S_I2C_ADDR, len) < len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)_wire.read();
    return true;
}
