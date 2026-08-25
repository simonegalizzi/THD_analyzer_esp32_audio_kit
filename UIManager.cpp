#include "UIManager.h"

UIManager::UIManager(uint8_t lcdAddr, uint8_t sdaPin, uint8_t sclPin,
                       uint8_t pinStart, uint8_t pinReset)
  : lcd_(lcdAddr, 16, 2), pinStart_(pinStart), pinReset_(pinReset) {
  (void)sdaPin; (void)sclPin; // Wire.begin() va chiamato esternamente nel main
                              // sketch (ordine critico rispetto al codec, vedi setup())
}

void UIManager::begin() {
  lcd_.init();
  lcd_.backlight();
  lcd_.clear();
  lcd_.setCursor(0, 0);
  lcd_.print("THD Analyzer");
}

void UIManager::updateLcd(float thd_percent, float thdn_percent, float last_signal, bool stopped) {
  char line0[17];
  char line1[17];

  snprintf(line0, sizeof(line0), "THD:%.3f%%", thd_percent);
  if (stopped) {
    int len = strlen(line0);
    for (int i = len; i < 15; i++) line0[i] = ' ';
    line0[15] = 'X';
    line0[16] = '\0';
  }

  snprintf(line1, sizeof(line1), "N:%.2f%% A:%.2f", thdn_percent, last_signal);

  lcd_.setCursor(0, 0);
  lcd_.print("                ");
  lcd_.setCursor(0, 0);
  lcd_.print(line0);

  lcd_.setCursor(0, 1);
  lcd_.print("                ");
  lcd_.setCursor(0, 1);
  lcd_.print(line1);
}

void UIManager::showSpecialDisplay(float thd_percent, bool stopped) {
  char line0[17];
  snprintf(line0, sizeof(line0), "THD:%.3f%%", thd_percent);
  if (stopped) {
    int len = strlen(line0);
    for (int i = len; i < 15; i++) line0[i] = ' ';
    line0[15] = 'X';
    line0[16] = '\0';
  }

  lcd_.setCursor(0, 0);
  lcd_.print("                ");
  lcd_.setCursor(0, 0);
  lcd_.print(line0);

  // Sequenza fissa richiesta, esattamente 16 caratteri: "4  8  17  33  83"
  lcd_.setCursor(0, 1);
  lcd_.print("4  8  17  33  83");
}

void UIManager::showGainOnLcd(int gain_step, uint8_t gainReg) {
  char buf[17];
  int db = gain_step * 3;
  snprintf(buf, sizeof(buf), "+%2ddB (0x%02X)   ", db, gainReg);
  lcd_.setCursor(0, 1);
  lcd_.print(buf);
}

void UIManager::showNoiseOnLcd(float hz) {
  char buf[17];
  snprintf(buf, sizeof(buf), "%.0f Hz          ", hz);
  lcd_.setCursor(0, 1);
  lcd_.print(buf);
}

void UIManager::showAvgOnLcd(int n) {
  char buf[17];
  snprintf(buf, sizeof(buf), "%d letture      ", n);
  lcd_.setCursor(0, 1);
  lcd_.print(buf);
}

// ===================================================================
// MENU UNIFICATO IMPOSTAZIONI (GAIN / NOISE / AVG)
// ===================================================================
// Navigazione con solo due pulsanti:
//   - RESET singolo = aumenta valore del parametro corrente (risposta immediata)
//   - START singolo = diminuisce valore del parametro corrente (risposta immediata)
//   - tap veloce di entrambi insieme = passa al parametro successivo
//   - tenuta di entrambi ~1.5s = salva ed esce
enum SettingParam { PARAM_GAIN = 0, PARAM_NOISE = 1, PARAM_AVG = 2, PARAM_COUNT = 3 };

void UIManager::enterSettingsMenu(uint8_t &gainRegInOut, float &noiseHzInOut, int &numAvgInOut) {
  Serial.println("\n>>> MENU IMPOSTAZIONI <<<");
  Serial.println("RESET=+ START=- | tap combo=prossimo parametro | tenuta combo 1.5s=esci\n");

  while (digitalRead(pinStart_) == LOW || digitalRead(pinReset_) == LOW) delay(10);
  delay(100);

  int gain_step = gainRegInOut & 0x0F;
  if (gain_step > 8) gain_step = 8;
  bool gain_modified = false;

  int current_param = PARAM_GAIN;

  auto showCurrentParam = [&]() {
    lcd_.setCursor(0, 0);
    lcd_.print("                ");
    lcd_.setCursor(0, 0);
    switch (current_param) {
      case PARAM_GAIN:
        lcd_.print("GAIN:");
        showGainOnLcd(gain_step, gainRegInOut);
        break;
      case PARAM_NOISE:
        lcd_.print("NOISE excl.:");
        showNoiseOnLcd(noiseHzInOut);
        break;
      case PARAM_AVG:
        lcd_.print("MEDIE (AVG):");
        showAvgOnLcd(numAvgInOut);
        break;
    }
  };

  auto applyIncrement = [&]() {
    switch (current_param) {
      case PARAM_GAIN:
        if (gain_step < 8) gain_step++;
        gainRegInOut = (uint8_t)((gain_step << 4) | gain_step);
        gain_modified = true;
        showGainOnLcd(gain_step, gainRegInOut);
        break;
      case PARAM_NOISE:
        noiseHzInOut += 5.0f;
        if (noiseHzInOut > 2000.0f) noiseHzInOut = 2000.0f;
        showNoiseOnLcd(noiseHzInOut);
        break;
      case PARAM_AVG:
        if (numAvgInOut < 100) numAvgInOut++;
        showAvgOnLcd(numAvgInOut);
        break;
    }
  };

  auto applyDecrement = [&]() {
    switch (current_param) {
      case PARAM_GAIN:
        if (gain_step > 0) gain_step--;
        gainRegInOut = (uint8_t)((gain_step << 4) | gain_step);
        gain_modified = true;
        showGainOnLcd(gain_step, gainRegInOut);
        break;
      case PARAM_NOISE:
        noiseHzInOut -= 5.0f;
        if (noiseHzInOut < 0.0f) noiseHzInOut = 0.0f;
        showNoiseOnLcd(noiseHzInOut);
        break;
      case PARAM_AVG:
        if (numAvgInOut > 1) numAvgInOut--;
        showAvgOnLcd(numAvgInOut);
        break;
    }
  };

  lcd_.clear();
  showCurrentParam();

  const unsigned long REPEAT_MS = 200;
  const unsigned long EXIT_HOLD_MS = 1500;
  // Dopo che un combo (RESET+START) viene riconosciuto e risolto (cambio
  // parametro), i pulsanti vengono IGNORATI per questo tempo - non solo
  // "non eseguiti", ma il loro stato non viene nemmeno considerato per le
  // normali azioni singole. Serve a coprire il rilascio non perfettamente
  // simultaneo delle due dita: senza questa pausa, il dito che si alza per
  // ultimo puo' essere letto come una nuova pressione singola e cambiare
  // il valore subito dopo il cambio parametro.
  const unsigned long COMBO_LOCKOUT_MS = 2000;
  unsigned long lastReset = 0, lastIn = 0;
  bool combo_holding = false;
  unsigned long combo_start = 0;
  unsigned long lockout_until = 0;

  while (true) {
    unsigned long now = millis();
    bool locked_out = (now < lockout_until);

    // Durante il lockout, i pin non vengono nemmeno letti per le azioni
    // singole - solo per capire quando entrambi sono stati rilasciati,
    // cosi' un pulsante ancora premuto non riparte a contare da zero.
    bool inState = locked_out ? false : (digitalRead(pinStart_) == LOW);
    bool resetState = locked_out ? false : (digitalRead(pinReset_) == LOW);
    bool bothState = inState && resetState;

    if (locked_out) {
      // Non fare nulla con i pulsanti finche' il lockout non scade
    } else if (bothState) {
      if (!combo_holding) {
        combo_holding = true;
        combo_start = now;
      } else if ((now - combo_start) >= EXIT_HOLD_MS) {
        break; // tenuta lunga -> esci dal menu
      }
    } else {
      if (combo_holding) {
        // rilasciato prima della soglia di uscita -> tap veloce, prossimo parametro
        combo_holding = false;
        current_param = (current_param + 1) % PARAM_COUNT;
        showCurrentParam();
        // Blocca la lettura dei pulsanti per un po', per non confondere
        // il rilascio sfalsato delle due dita con una nuova pressione
        lockout_until = now + COMBO_LOCKOUT_MS;
      } else {
        if (resetState && (now - lastReset) > REPEAT_MS) {
          lastReset = now;
          applyIncrement();
        }
        if (inState && (now - lastIn) > REPEAT_MS) {
          lastIn = now;
          applyDecrement();
        }
      }
    }
    delay(15);
  }

  while (digitalRead(pinStart_) == LOW || digitalRead(pinReset_) == LOW) delay(10);

  Serial.print("Guadagno: +"); Serial.print(gain_step * 3); Serial.println("dB");
  Serial.print("Soglia rumore: "); Serial.print(noiseHzInOut, 1); Serial.println(" Hz");
  Serial.print("Numero medie: "); Serial.println(numAvgInOut);

  saveNoiseThresholdHz(noiseHzInOut);
  saveNumAverages(numAvgInOut);

  if (gain_modified) {
    saveGainReg(gainRegInOut);
    lcd_.clear();
    lcd_.setCursor(0, 0);
    lcd_.print("Salvo e riavvio");
    Serial.println("Guadagno modificato - riavvio per applicare...");
    delay(500);
    ESP.restart();
  } else {
    lcd_.clear();
    lcd_.setCursor(0, 0);
    lcd_.print("Impostazioni");
    lcd_.setCursor(0, 1);
    lcd_.print("salvate");
    delay(800);
    lcd_.clear();
    lcd_.setCursor(0, 0);
    lcd_.print("Pronto...");
  }
}

uint8_t UIManager::loadGainReg() {
  prefs_.begin("thdcfg", false);
  uint8_t v = prefs_.getUChar("reg", 0x00);
  prefs_.end();
  return v;
}

float UIManager::loadNoiseThresholdHz() {
  prefs_.begin("thdcfg", false);
  float v = prefs_.getFloat("noiseHz", 100.0f);
  prefs_.end();
  return v;
}

int UIManager::loadNumAverages() {
  prefs_.begin("thdcfg", false);
  int v = prefs_.getInt("numAvg", 4);
  prefs_.end();
  return v;
}

void UIManager::saveNumAverages(int n) {
  prefs_.begin("thdcfg", false);
  prefs_.putInt("numAvg", n);
  prefs_.end();
}

void UIManager::saveGainReg(uint8_t reg) {
  prefs_.begin("thdcfg", false);
  prefs_.putUChar("reg", reg);
  prefs_.end();
}

void UIManager::saveNoiseThresholdHz(float hz) {
  prefs_.begin("thdcfg", false);
  prefs_.putFloat("noiseHz", hz);
  prefs_.end();
}
