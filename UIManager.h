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
               uint8_t pinStart, uint8_t pinReset);

    void begin();

    // Da chiamare ad ogni ciclo di loop() per la lettura media di sessione
    void updateLcd(float thd_percent, float thdn_percent, float last_signal, bool stopped);

    // Display speciale (combo RESET+START tenuta 4s): riga0 = THD live,
    // riga1 = sequenza fissa "4  8  17  33  83" (16 caratteri esatti,
    // '4' in colonna 0, ultimo '3' di 83 in colonna 15).
    void showSpecialDisplay(float thd_percent, bool stopped);

    // Menu unificato impostazioni (GAIN / NOISE / AVG / FFT), navigabile con
    // solo due pulsanti - sostituisce i vecchi menu separati e KEY3.
    // Bloccante: ritorna solo all'uscita (tenuta combo ~1.5s).
    // gainRegInOut viene aggiornato E salvato SOLO se il parametro GAIN
    // e' stato effettivamente toccato durante la sessione di menu (in tal
    // caso la funzione riavvia la scheda prima di ritornare, per lo stesso
    // motivo di sempre: la scrittura I2C del guadagno e' affidabile solo
    // subito dopo kit.begin()). noiseHzInOut, numAvgInOut e fftSizeInOut si
    // applicano sempre subito, nessun riavvio necessario per questi tre.
    // fftSizeInOut viene solo CICLATO tra i valori ammessi (4096..65536):
    // sta al chiamante (.ino) applicarlo davvero all'analyzer dopo il ritorno,
    // dato che UIManager non conosce THDAnalyzer.
    void enterSettingsMenu(uint8_t &gainRegInOut, float &noiseHzInOut, int &numAvgInOut, int &fftSizeInOut);

    uint8_t loadGainReg();
    float   loadNoiseThresholdHz();
    int     loadNumAverages();
    void    saveGainReg(uint8_t reg);
    void    saveNoiseThresholdHz(float hz);
    void    saveNumAverages(int n);

    LiquidCrystal_I2C& lcd() { return lcd_; }

  private:
    LiquidCrystal_I2C lcd_;
    Preferences prefs_;
    uint8_t pinStart_, pinReset_;

    void showGainOnLcd(int gain_step, uint8_t gainReg);
    void showNoiseOnLcd(float hz);
    void showAvgOnLcd(int n);
    void showFftOnLcd(int fftSize);
};

#endif
