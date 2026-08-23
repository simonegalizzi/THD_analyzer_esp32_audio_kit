#ifndef ES8388_HELPER_H
#define ES8388_HELPER_H

#include <Arduino.h>
#include <Wire.h>

// Tutte le scritture dirette I2C al codec ES8388 che bypassano la libreria
// arduino-audio-driver, raccolte qui invece di essere sparse in setup().
// Indirizzo I2C fisso del codec su questa board: 0x10, bus su pin 32/33.
namespace ES8388Helper {

  const uint8_t I2C_ADDR = 0x10;

  // Scrive un registro, ritorna true se l'operazione I2C e' andata a buon fine
  // (NON garantisce che il valore resti tale dopo eventuali riscritture
  // successive della libreria - vedi verifyRegister per un controllo puntuale).
  bool writeRegister(uint8_t reg, uint8_t value);

  // Legge un registro, ritorna true se la lettura e' andata a buon fine
  bool readRegister(uint8_t reg, uint8_t &outValue);

  // Imposta il guadagno PGA (registro 0x09 - ADCCONTROL1).
  // gainReg gia' pronto nel formato (L<<4 | R), stesso schema del menu segreto.
  bool setPgaGain(uint8_t gainReg);

  // Spegne il bias del microfono (registro 0x03 = 0x08), mantenendo
  // attivi ingresso analogico e ADC (necessari per LINE2).
  bool disableMicBias();

  // Forza la modalita' double-speed con ratio 128 (necessaria per 96kHz).
  // Scrive Reg 0x0D (ADC Control 5) e Reg 0x18 (DAC Control 2) = 0x20.
  bool forceDoubleSpeed96k();

  // Diagnostica: legge il registro 8 (Master Mode Control) e stampa
  // lo stato del bit MCLKDIV2 sul monitor seriale.
  void printMasterModeDiag();
}

#endif
