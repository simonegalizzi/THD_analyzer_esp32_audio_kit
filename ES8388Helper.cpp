#include "ES8388Helper.h"

namespace ES8388Helper {

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegister(uint8_t reg, uint8_t &outValue) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)I2C_ADDR, (uint8_t)1) != 1 || !Wire.available()) return false;
  outValue = Wire.read();
  return true;
}

bool setPgaGain(uint8_t gainReg) {
  bool ok = writeRegister(0x09, gainReg);
  Serial.print("Scrittura guadagno PGA (reg 0x09=0x");
  Serial.print(gainReg, HEX);
  Serial.print("): ");
  Serial.println(ok ? "OK" : "ERRORE");
  return ok;
}

bool disableMicBias() {
  bool ok = writeRegister(0x03, 0x08);
  Serial.print("Spegnimento bias microfono (reg 0x03=0x08): ");
  Serial.println(ok ? "OK" : "ERRORE");

  uint8_t readback = 0xFF;
  if (readRegister(0x03, readback)) {
    Serial.print("Verifica reg 0x03 subito dopo: 0x");
    Serial.print(readback, HEX);
    Serial.println((readback == 0x08) ? " -> COINCIDE (in questo istante)" : " -> NON COINCIDE");
  } else {
    Serial.println("Verifica reg 0x03: ERRORE I2C in lettura");
  }
  return ok;
}

bool forceDoubleSpeed96k() {
  // Reg 13 (0x0D) - ADC Control 5: ADCFsMode=1 (double speed), ADCFsRatio=00000 (128)
  bool adc_ok = writeRegister(0x0D, 0x20);
  Serial.print("ADC double-speed 96kHz (reg 0x0D=0x20): ");
  Serial.println(adc_ok ? "OK" : "ERRORE");

  // Reg 24 (0x18) - DAC Control 2: DACFsMode=1, DACFsRatio=00000 (128)
  bool dac_ok = writeRegister(0x18, 0x20);
  Serial.print("DAC double-speed 96kHz (reg 0x18=0x20): ");
  Serial.println(dac_ok ? "OK" : "ERRORE");

  uint8_t readback = 0xFF;
  if (readRegister(0x0D, readback)) {
    Serial.print("Verifica reg 0x0D subito dopo: 0x");
    Serial.println(readback, HEX);
  }

  return adc_ok && dac_ok;
}

void printMasterModeDiag() {
  uint8_t reg8 = 0xFF;
  if (readRegister(0x08, reg8)) {
    Serial.print("Reg 0x08 (Master Mode Control): 0x");
    Serial.println(reg8, HEX);
    Serial.print("MCLKDIV2 bit: ");
    Serial.println((reg8 & 0x40) ? "1 (diviso)" : "0 (non diviso)");
  } else {
    Serial.println("Errore lettura reg 0x08");
  }
}

}
