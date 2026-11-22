// VL53L0X sensor code :)
#pragma once
#include <Arduino.h>
#include <Wire.h>
class VL53L0X_Direct {
public:
  void setTimeout(uint16_t ms) { timeoutMs = ms; }
  bool timeoutOccurred() {
    bool t = didTimeout;
    didTimeout = false;
    return t;
  }
  bool init(TwoWire &bus = Wire, uint8_t address = 0x29) {
    i2c = &bus;
    addr = address;
    if (readReg(0xC0) != 0xEE) return false;     
    writeReg(0x89, readReg(0x89) | 0x01);        
    writeReg(0x88, 0x00);                         
    writeReg(0x80, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);
    stopVariable = readReg(0x91);               
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);
    writeReg(0x60, readReg(0x60) | 0x12);        
    writeReg16(0x44, (uint16_t)(0.25 * 128));    
    writeReg(0x01, 0xFF);                       
    uint8_t spadCount;
    bool spadIsAperture;
    if (!getSpadInfo(spadCount, spadIsAperture)) return false;
    uint8_t spadMap[6];
    readMulti(0xB0, spadMap, 6);
    writeReg(0xFF, 0x01);
    writeReg(0x4F, 0x00);
    writeReg(0x4E, 0x2C);
    writeReg(0xFF, 0x00);
    writeReg(0xB6, 0xB4);
    uint8_t first = spadIsAperture ? 12 : 0;
    uint8_t enabled = 0;
    for (uint8_t i = 0; i < 48; i++) {
      if (i < first || enabled == spadCount) {
        spadMap[i / 8] &= ~(1 << (i % 8));
      } else if ((spadMap[i / 8] >> (i % 8)) & 0x01) {
        enabled++;
      }
    }
    writeMulti(0xB0, spadMap, 6);
    loadTuning();
    writeReg(0x0A, 0x04);
    writeReg(0x84, readReg(0x84) & ~0x10);
    writeReg(0x0B, 0x01);
    writeReg(0x01, 0xE8);
    if (!setMeasurementTimingBudget(33000)) return false;
    writeReg(0x01, 0x01);
    if (!singleRefCalibration(0x40)) return false; 
    writeReg(0x01, 0x02);
    if (!singleRefCalibration(0x00)) return false;  
    writeReg(0x01, 0xE8);
    return true;
  }
  bool setMeasurementTimingBudget(uint32_t budgetUs) {
    const uint32_t StartOverhead = 1910, EndOverhead = 960;
    const uint32_t MsrcOverhead = 660, TccOverhead = 590, DssOverhead = 690;
    const uint32_t PreRangeOverhead = 660, FinalRangeOverhead = 550;
    if (budgetUs < 20000) return false;
    uint8_t seq = readReg(0x01);
    bool tcc = seq & 0x10, dss = seq & 0x08, msrc = seq & 0x04;
    bool preRange = seq & 0x40, finalRange = seq & 0x80;
    uint8_t prePclks = decodeVcselPeriod(readReg(0x50));
    uint8_t finalPclks = decodeVcselPeriod(readReg(0x70));
    uint32_t msrcUs = mclksToUs(readReg(0x46) + 1, prePclks);
    uint16_t preMclks = decodeTimeout(readReg16(0x51));
    uint32_t preUs = mclksToUs(preMclks, prePclks);
    uint32_t used = StartOverhead + EndOverhead;
    if (tcc) used += msrcUs + TccOverhead;
    if (dss) used += 2 * (msrcUs + DssOverhead);
    else if (msrc) used += msrcUs + MsrcOverhead;
    if (preRange) used += preUs + PreRangeOverhead;
    if (!finalRange) return true;
    used += FinalRangeOverhead;
    if (used > budgetUs) return false;
    uint32_t finalMclks = usToMclks(budgetUs - used, finalPclks);
    if (preRange) finalMclks += preMclks;
    writeReg16(0x71, encodeTimeout(finalMclks));
    return true;
  } 
  void startContinuous() {
    writeReg(0x80, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);
    writeReg(0x91, stopVariable);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);
    writeReg(0x00, 0x02);                         
  } 
  uint16_t readRangeContinuousMillimeters() {
    uint32_t start = millis();
    while ((readReg(0x13) & 0x07) == 0) {         
      if (timeoutMs && millis() - start > timeoutMs) {
        didTimeout = true;
        return 65535;
      }
    }
    uint16_t mm = readReg16(0x1E);                
    writeReg(0x0B, 0x01);                        
    return mm;
  }
private:
  TwoWire *i2c = &Wire;
  uint8_t addr = 0x29;
  uint8_t stopVariable = 0;
  uint16_t timeoutMs = 0;
  bool didTimeout = false;
  void writeReg(uint8_t reg, uint8_t val) {
    i2c->beginTransmission(addr);
    i2c->write(reg);
    i2c->write(val);
    i2c->endTransmission();
  }
  void writeReg16(uint8_t reg, uint16_t val) {
    i2c->beginTransmission(addr);
    i2c->write(reg);
    i2c->write((uint8_t)(val >> 8));
    i2c->write((uint8_t)(val & 0xFF));
    i2c->endTransmission();
  }
  void writeMulti(uint8_t reg, const uint8_t *src, uint8_t n) {
    i2c->beginTransmission(addr);
    i2c->write(reg);
    for (uint8_t i = 0; i < n; i++) i2c->write(src[i]);
    i2c->endTransmission();
  }
  uint8_t readReg(uint8_t reg) {
    i2c->beginTransmission(addr);
    i2c->write(reg);
    i2c->endTransmission();
    i2c->requestFrom(addr, (uint8_t)1);
    return (uint8_t)i2c->read();
  }
  uint16_t readReg16(uint8_t reg) {
    i2c->beginTransmission(addr);
    i2c->write(reg);
    i2c->endTransmission();
    i2c->requestFrom(addr, (uint8_t)2);
    uint16_t hi = (uint8_t)i2c->read();
    uint16_t lo = (uint8_t)i2c->read();
    return (hi << 8) | lo;
  }
  void readMulti(uint8_t reg, uint8_t *dst, uint8_t n) {
    i2c->beginTransmission(addr);
    i2c->write(reg);
    i2c->endTransmission();
    i2c->requestFrom(addr, n);
    for (uint8_t i = 0; i < n; i++) dst[i] = (uint8_t)i2c->read();
  }
  bool waitWhile(uint8_t reg, uint8_t mask, uint8_t value) {
    uint32_t start = millis();
    while ((readReg(reg) & mask) == value) {
      if (timeoutMs && millis() - start > timeoutMs) return false;
    }
    return true;
  }
  bool getSpadInfo(uint8_t &count, bool &isAperture) {
    writeReg(0x80, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);
    writeReg(0xFF, 0x06);
    writeReg(0x83, readReg(0x83) | 0x04);
    writeReg(0xFF, 0x07);
    writeReg(0x81, 0x01);
    writeReg(0x80, 0x01);
    writeReg(0x94, 0x6B);
    writeReg(0x83, 0x00);
    if (!waitWhile(0x83, 0xFF, 0x00)) return false;
    writeReg(0x83, 0x01);
    uint8_t tmp = readReg(0x92);
    count = tmp & 0x7F;
    isAperture = (tmp >> 7) & 0x01;
    writeReg(0x81, 0x00);
    writeReg(0xFF, 0x06);
    writeReg(0x83, readReg(0x83) & ~0x04);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);
    return true;
  }
  bool singleRefCalibration(uint8_t vhvInitByte) {
    writeReg(0x00, 0x01 | vhvInitByte);          
    if (!waitWhile(0x13, 0x07, 0x00)) return false;
    writeReg(0x0B, 0x01);
    writeReg(0x00, 0x00);
    return true;
  }
  void loadTuning() {    
    static const uint8_t tuning[][2] = {
      {0xFF, 0x01}, {0x00, 0x00},
      {0xFF, 0x00}, {0x09, 0x00}, {0x10, 0x00}, {0x11, 0x00},
      {0x24, 0x01}, {0x25, 0xFF}, {0x75, 0x00},
      {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00}, {0x30, 0x20},
      {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00}, {0x31, 0x04}, {0x32, 0x03},
      {0x40, 0x83}, {0x46, 0x25}, {0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06},
      {0x51, 0x00}, {0x52, 0x96}, {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00},
      {0x62, 0x00}, {0x64, 0x00}, {0x65, 0x00}, {0x66, 0xA0},
      {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF}, {0x4A, 0x00},
      {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00}, {0x78, 0x21},
      {0xFF, 0x01}, {0x23, 0x34}, {0x42, 0x00}, {0x44, 0xFF}, {0x45, 0x26},
      {0x46, 0x05}, {0x40, 0x40}, {0x0E, 0x06}, {0x20, 0x1A}, {0x43, 0x40},
      {0xFF, 0x00}, {0x34, 0x03}, {0x35, 0x44},
      {0xFF, 0x01}, {0x31, 0x04}, {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04},
      {0xFF, 0x00}, {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28},
      {0x67, 0x00}, {0x70, 0x04}, {0x71, 0x01}, {0x72, 0xFE}, {0x76, 0x00},
      {0x77, 0x00},
      {0xFF, 0x01}, {0x0D, 0x01},
      {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8},
      {0xFF, 0x01}, {0x8E, 0x01}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00},
    };
    for (const auto &t : tuning) writeReg(t[0], t[1]);
  }
  static uint8_t decodeVcselPeriod(uint8_t regVal) { return (regVal + 1) << 1; }
  static uint32_t macroPeriodNs(uint8_t vcselPclks) {
    return ((2304UL * vcselPclks * 1655UL) + 500) / 1000;
  }
  static uint32_t mclksToUs(uint16_t mclks, uint8_t vcselPclks) {
    uint32_t ns = macroPeriodNs(vcselPclks);
    return ((mclks * ns) + 500) / 1000;
  }
  static uint32_t usToMclks(uint32_t us, uint8_t vcselPclks) {
    uint32_t ns = macroPeriodNs(vcselPclks);
    return ((us * 1000) + (ns / 2)) / ns;
  } 
  static uint16_t decodeTimeout(uint16_t regVal) {
    return (uint16_t)((regVal & 0x00FF) << ((regVal & 0xFF00) >> 8)) + 1;
  }
  static uint16_t encodeTimeout(uint32_t mclks) {
    if (mclks == 0) return 0;
    uint32_t ls = mclks - 1;
    uint16_t ms = 0;
    while (ls & 0xFFFFFF00) {
      ls >>= 1;
      ms++;
    }
    return (ms << 8) | (ls & 0xFF);
  }
};
