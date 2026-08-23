#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>
#include "THDAnalyzer.h"

// Incapsula display LCD 1602A (via PCF8574 I2C) e i due menu bloccanti
// (guadagno PGA, soglia esclusione rumore). Il bus I2C del display
// (IO21/IO22) e' indipendente da quello del codec ES8388 (32/33).
class UIManager {
  public:
    UIManager(uint8_t lcdAddr, uint8_t sdaPin, uint8_t sclPin,
               uint8_t pinStart, uint8_t pinReset, uint8_t pinKey3);

    void begin();

    // Da chiamare ad ogni ciclo di loop() per la lettura media di sessione
    void updateLcd(float thd_percent, float thdn_percent, float last_signal, bool stopped);

    // Menu bloccanti - ritornano solo all'uscita (combo pulsanti / KEY3)
    // gainRegOut: valore del registro PGA scelto, da scrivere via ES8388Helper
    // dopo il riavvio (la scrittura I2C al codec e' affidabile solo appena
    // dopo kit.begin(), mai a runtime - percio' si riavvia la scheda).
    void enterGainMenu(uint8_t &gainRegInOut);

    // noiseHzInOut: soglia esclusione rumore, si applica subito senza riavvio
    void enterNoiseMenu(float &noiseHzInOut);

    // Persistenza NVS (namespace "thdcfg")
    uint8_t loadGainReg();
    float   loadNoiseThresholdHz();
    void    saveGainReg(uint8_t reg);
    void    saveNoiseThresholdHz(float hz);

    LiquidCrystal_I2C& lcd() { return lcd_; }

  private:
    LiquidCrystal_I2C lcd_;
    Preferences prefs_;
    uint8_t pinStart_, pinReset_, pinKey3_;

    void showGainOnLcd(int gain_step, uint8_t gainReg);
    void showNoiseThresholdOnLcd(float hz);
};

#endif
