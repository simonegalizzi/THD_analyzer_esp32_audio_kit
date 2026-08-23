#include "UIManager.h"

UIManager::UIManager(uint8_t lcdAddr, uint8_t sdaPin, uint8_t sclPin,
                       uint8_t pinStart, uint8_t pinReset, uint8_t pinKey3)
  : lcd_(lcdAddr, 16, 2), pinStart_(pinStart), pinReset_(pinReset), pinKey3_(pinKey3) {
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

void UIManager::showGainOnLcd(int gain_step, uint8_t gainReg) {
  char buf[17];
  int db = gain_step * 3;
  snprintf(buf, sizeof(buf), "+%2ddB (0x%02X)", db, gainReg);
  lcd_.setCursor(0, 1);
  lcd_.print(buf);
  Serial.print("Guadagno selezionato: +");
  Serial.print(db);
  Serial.print("dB (reg 0x");
  Serial.print(gainReg, HEX);
  Serial.println(")");
}

void UIManager::enterGainMenu(uint8_t &gainRegInOut) {
  Serial.println("\n>>> MENU SEGRETO GUADAGNO PGA <<<");
  Serial.println("RESET = +3dB, START = -3dB, doppia pressione = esci e salva\n");

  int gain_step = gainRegInOut & 0x0F;
  if (gain_step > 8) gain_step = 8;

  lcd_.clear();
  lcd_.setCursor(0, 0);
  lcd_.print("Guadagno PGA:");
  showGainOnLcd(gain_step, gainRegInOut);

  while (digitalRead(pinStart_) == LOW || digitalRead(pinReset_) == LOW) delay(10);
  delay(100);

  const unsigned long REPEAT_MS = 220;
  unsigned long lastReset = 0, lastIn = 0;

  while (true) {
    bool inState = digitalRead(pinStart_) == LOW;
    bool resetState = digitalRead(pinReset_) == LOW;
    unsigned long now = millis();

    if (inState && resetState) {
      delay(50);
      if (digitalRead(pinStart_) == LOW && digitalRead(pinReset_) == LOW) break;
    } else {
      if (resetState && (now - lastReset) > REPEAT_MS) {
        lastReset = now;
        if (gain_step < 8) gain_step++;
        gainRegInOut = (uint8_t)((gain_step << 4) | gain_step);
        showGainOnLcd(gain_step, gainRegInOut);
      }
      if (inState && (now - lastIn) > REPEAT_MS) {
        lastIn = now;
        if (gain_step > 0) gain_step--;
        gainRegInOut = (uint8_t)((gain_step << 4) | gain_step);
        showGainOnLcd(gain_step, gainRegInOut);
      }
    }
    delay(15);
  }

  while (digitalRead(pinStart_) == LOW || digitalRead(pinReset_) == LOW) delay(10);

  Serial.print("\nGuadagno confermato: +");
  Serial.print(gain_step * 3);
  Serial.print("dB (reg 0x");
  Serial.print(gainRegInOut, HEX);
  Serial.println(")");

  lcd_.clear();
  lcd_.setCursor(0, 0);
  lcd_.print("Salvo e riavvio");

  saveGainReg(gainRegInOut);

  delay(500);
  ESP.restart();
}

void UIManager::showNoiseThresholdOnLcd(float hz) {
  char buf[17];
  snprintf(buf, sizeof(buf), "%.0f Hz       ", hz);
  lcd_.setCursor(0, 1);
  lcd_.print(buf);
  Serial.print("Soglia esclusione rumore: ");
  Serial.print(hz, 1);
  Serial.println(" Hz");
}

void UIManager::enterNoiseMenu(float &noiseHzInOut) {
  Serial.println("\n>>> MENU SOGLIA ESCLUSIONE RUMORE <<<");
  Serial.println("RESET = aumenta, START = diminuisci, KEY3 = esci e salva\n");

  lcd_.clear();
  lcd_.setCursor(0, 0);
  lcd_.print("Soglia rumore:");
  showNoiseThresholdOnLcd(noiseHzInOut);

  while (digitalRead(pinKey3_) == LOW) delay(10);
  delay(100);

  const unsigned long REPEAT_MS = 150;
  const float STEP_HZ = 5.0f;
  const float MIN_HZ = 0.0f;
  const float MAX_HZ = 2000.0f;
  unsigned long lastReset = 0, lastIn = 0;

  while (true) {
    bool inState = digitalRead(pinStart_) == LOW;
    bool resetState = digitalRead(pinReset_) == LOW;
    bool key3State = digitalRead(pinKey3_) == LOW;
    unsigned long now = millis();

    if (key3State) {
      delay(50);
      if (digitalRead(pinKey3_) == LOW) break;
    } else {
      if (resetState && (now - lastReset) > REPEAT_MS) {
        lastReset = now;
        noiseHzInOut += STEP_HZ;
        if (noiseHzInOut > MAX_HZ) noiseHzInOut = MAX_HZ;
        showNoiseThresholdOnLcd(noiseHzInOut);
      }
      if (inState && (now - lastIn) > REPEAT_MS) {
        lastIn = now;
        noiseHzInOut -= STEP_HZ;
        if (noiseHzInOut < MIN_HZ) noiseHzInOut = MIN_HZ;
        showNoiseThresholdOnLcd(noiseHzInOut);
      }
    }
    delay(15);
  }

  while (digitalRead(pinKey3_) == LOW) delay(10);

  Serial.print("\nSoglia confermata: ");
  Serial.print(noiseHzInOut, 1);
  Serial.println(" Hz - applicata immediatamente, nessun riavvio necessario");

  lcd_.clear();
  lcd_.setCursor(0, 0);
  lcd_.print("Soglia salvata");

  saveNoiseThresholdHz(noiseHzInOut);

  delay(800);
  lcd_.clear();
  lcd_.setCursor(0, 0);
  lcd_.print("Pronto...");
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
