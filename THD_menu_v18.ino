/*******************************************************
  THD+N ANALYZER v18 - ESP32 Audio Kit V2.2 (ES8388) @ 96kHz
  Refactor modulare della v17: stessa logica di analisi,
  suddivisa in classi (THDAnalyzer, ES8388Helper, UIManager)
  per rendere lo sketch principale leggibile.

  LIBRERIE RICHIESTE (Arduino IDE > Library Manager):
  - "arduino-audio-tools" di pschatzmann
  - "arduino-audio-driver" di pschatzmann
  - "LiquidCrystal I2C" di Frank de Brabander

  FILE DI QUESTO SKETCH (tutti nella stessa cartella):
  - THD_menu_v18.ino   <- questo file: setup()/loop()
  - THDAnalyzer.h/.cpp <- FFT + calcolo THD/THD+N/SNR
  - ES8388Helper.h/.cpp<- scritture dirette registri I2C codec
  - UIManager.h/.cpp   <- LCD + menu pulsanti

  PIN (gia' cablati sulla board Ai-Thinker ESP32-Audio-Kit V2.2):
  - I2S: BCK=27, WS=25, DATA_OUT=26, DATA_IN=35, MCLK=0
  - I2C codec (interno al modulo): SDA=32, SCL=33
  - I2C display LCD (header esterno): SDA=21, SCL=22
********************************************************/

#include "AudioTools.h"
#include "AudioTools/AudioLibs/I2SCodecStream.h"
using namespace audio_tools;

#include "THDAnalyzer.h"
#include "ES8388Helper.h"
#include "UIManager.h"

// ===== CONFIGURAZIONE =====
#define FFT_SIZE 65536
#define FS       96000

#define LCD_I2C_ADDR 0x27
const int PIN_START = 36;   // GPIO36 input-only, serve pull-up esterna 10k
const int PIN_RESET = 13;
const int PIN_KEY3  = 19;

#define NUM_AVERAGES 4
#define SERIAL_PER_READING 0

// ===== OGGETTI GLOBALI (uno per sottosistema) =====
THDAnalyzer analyzer(FFT_SIZE, FS);
UIManager   ui(LCD_I2C_ADDR, 21, 22, PIN_START, PIN_RESET, PIN_KEY3);
I2SCodecStream kit(AudioKitEs8388V1);

int16_t raw_block[512];

// ===== STATO SESSIONE (media mobile, clipping, pulsanti) =====
const int BUFFER_SIZE = 100;
float thd_buffer[BUFFER_SIZE];
float thdn_buffer[BUFFER_SIZE];
float freq_buffer[BUFFER_SIZE];
int buffer_index = 0;
int samples_count = 0;
bool buffer_full = false;

const float CLIPPING_THRESHOLD   = 0.95f;
const float LOW_SIGNAL_THRESHOLD = 0.12f;

bool  acquisition_active = false;
float session_max_signal = 0.0f;
int   session_clip_count = 0;
int   session_samples = 0;

float last_thd_percent = 0.0f;
float last_thdn_percent = 0.0f;
float last_campioni = 0.0f;
float frequenza = 0.0f;

uint8_t pga_reg_value = 0x00;

bool last_pinIn_state = HIGH, last_pinReset_state = HIGH, last_pinKey3_state = HIGH;
unsigned long last_button_time_start = 0, last_button_time_reset = 0, last_button_time_key3 = 0;
const unsigned long DEBOUNCE_MS = 800;

bool last_both_buttons_state = false;
unsigned long last_combo_time = 0;
const unsigned long COMBO_DEBOUNCE_MS = 600;

// ===================================================================
// FUNZIONE DI LETTURA CAMPIONI - passata come callback a THDAnalyzer
// ===================================================================
// Firma fissata da THDAnalyzer::ReadRawFn - legge un blocco dal codec,
// normalizza, applica DC offset, aggiorna maxAbs/clipCount.
int readAudioKitBlock(float32_t *dest, int start_idx, float &maxAbsOut, int &clipCountOut,
                       float dc_offset, bool dc_calibrated) {
  size_t bytes_read = kit.readBytes((uint8_t*)raw_block, sizeof(raw_block));
  int samples = bytes_read / sizeof(int16_t) / 2;

  for (int i = 0; i < samples; i++) {
    int16_t raw_orig = raw_block[i * 2 + 1]; // canale R (confermato funzionante con LINE2)
    float v = (float)raw_orig / 32768.0f;
    if (dc_calibrated) v -= dc_offset;

    dest[start_idx + i] = v;

    float av = fabsf(v);
    if (av > maxAbsOut) maxAbsOut = av;
    if (av > 0.95f) clipCountOut++;
  }
  return samples;
}

bool stopRequested() {
  return digitalRead(PIN_START) == LOW;
}

void resetBuffers() {
  buffer_index = 0;
  samples_count = 0;
  buffer_full = false;
  session_max_signal = 0.0f;
  session_clip_count = 0;
  session_samples = 0;
}

void evaluateSignalLevel() {
  if (session_samples == 0) return;
  Serial.println("\n--- Valutazione livello segnale ---");
  Serial.print("Max segnale sessione: ");
  Serial.println(session_max_signal, 4);
  Serial.print("Campioni con clipping: ");
  Serial.println(session_clip_count);

  if (session_clip_count > 5 || session_max_signal > CLIPPING_THRESHOLD) {
    Serial.println("CLIPPING rilevato! Riduci il livello del segnale in ingresso");
  } else if (session_max_signal < LOW_SIGNAL_THRESHOLD) {
    Serial.println("Segnale basso: aumenta il livello in ingresso");
  } else {
    Serial.println("Livello segnale OK");
  }

  session_max_signal = 0.0f;
  session_clip_count = 0;
  session_samples = 0;
}

void runOneMeasurement() {
  THDResult r;
  analyzer.acquireAndAnalyze(readAudioKitBlock, stopRequested, r);

  if (acquisition_active) {
    if (r.maxAbs > session_max_signal) session_max_signal = r.maxAbs;
    if (r.clipping) session_clip_count++;
    session_samples++;
  }

  thd_buffer[buffer_index] = r.thd_percent;
  thdn_buffer[buffer_index] = r.thdn_percent;
  freq_buffer[buffer_index] = r.frequenza;
  buffer_index++;
  if (buffer_index >= BUFFER_SIZE) { buffer_index = 0; buffer_full = true; }
  if (!buffer_full && samples_count < BUFFER_SIZE) samples_count++;

  frequenza = r.frequenza;
  last_campioni = r.maxAbs;
  last_thd_percent = r.thd_percent;
  last_thdn_percent = r.thdn_percent;

  // Diagnostica: divario sospetto THD/THD+N (probabile rumore a banda larga o rete)
  if (r.thdn_percent > r.thd_percent * 3.0f && r.thdn_percent > 1.0f) {
    Serial.print("  [DIAG] Energia rete (50/100/150Hz...): ");
    Serial.print(r.powerLine_pct, 3);
    Serial.print("% della fondamentale | Picco rumore piu' forte: ");
    Serial.print(r.worstNoise_hz, 1);
    Serial.print("Hz (");
    Serial.print(r.worstNoise_pct, 3);
    Serial.println("% della fondamentale)");
  }

#if SERIAL_PER_READING
  Serial.print("F:"); Serial.print(r.frequenza, 1);
  Serial.print("Hz THD:"); Serial.print(r.thd_percent, 4);
  Serial.print("% THD+N:"); Serial.print(r.thdn_percent, 4);
  Serial.print("% SNR:"); Serial.print(r.snr_db, 1);
  Serial.print("dB H:"); Serial.print(r.harmonics_found);
  Serial.print(" H_reali:"); Serial.print(r.harmonics_real);
  Serial.print(" H_max_din:"); Serial.print(r.harm_max_dynamic);
  Serial.print(r.clipping ? " [CLIP]" : "");
  Serial.print(" maxAbs:"); Serial.print(r.maxAbs, 5);
  Serial.println();
#endif
}

void updateMediaContinua() {
  if (samples_count < NUM_AVERAGES && !buffer_full) return;

  int num_samples = buffer_full ? BUFFER_SIZE : samples_count;
  int start_avg = max(0, num_samples - NUM_AVERAGES);

  double sum_thd = 0.0, sum_thdn = 0.0, sum_freq = 0.0;
  int navg = num_samples - start_avg;
  for (int i = start_avg; i < num_samples; i++) {
    sum_thd += thd_buffer[i];
    sum_thdn += thdn_buffer[i];
    sum_freq += freq_buffer[i];
  }
  float media_thd = sum_thd / navg;
  float media_thdn = sum_thdn / navg;
  float media_freq = sum_freq / navg;

  Serial.print("MEDIA("); Serial.print(navg); Serial.print(") F:");
  Serial.print(media_freq, 1); Serial.print("Hz THD:");
  Serial.print(media_thd, 4); Serial.print("% THD+N:");
  Serial.print(media_thdn, 4); Serial.println("%");

  last_thd_percent = media_thd;
  last_thdn_percent = media_thdn;
  ui.updateLcd(last_thd_percent, last_thdn_percent, last_campioni, !acquisition_active);
}

// ===================================================================
// SETUP
// ===================================================================
void setup() {
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_RESET, INPUT_PULLUP);
  pinMode(PIN_KEY3, INPUT_PULLUP);
  Serial.begin(115200);

  Serial.println("\n==============================================");
  Serial.println(" THD Analyzer v18 - ESP32 Audio Kit V2.2 (ES8388)");
  Serial.println("==============================================\n");

  if (!analyzer.begin()) {
    Serial.println("Riduci FFT_SIZE e ricarica.");
    while (1) delay(1000);
  }

  memset(thd_buffer, 0, sizeof(thd_buffer));
  memset(thdn_buffer, 0, sizeof(thdn_buffer));
  memset(freq_buffer, 0, sizeof(freq_buffer));

  pga_reg_value = ui.loadGainReg();
  analyzer.setNoiseExcludeBelowHz(ui.loadNoiseThresholdHz());
  Serial.print("Guadagno PGA caricato da NVS: 0x");
  Serial.println(pga_reg_value, HEX);
  Serial.print("Soglia esclusione rumore caricata da NVS: ");
  Serial.print(analyzer.getNoiseExcludeBelowHz(), 1);
  Serial.println(" Hz");

  // ===== INIZIALIZZAZIONE CODEC ES8388 =====
  auto cfg = kit.defaultConfig(RX_MODE);
  cfg.sample_rate = FS;
  cfg.buffer_size = 1024;   // decommenta se servono per stabilita' a 96kHz
  cfg.buffer_count = 8;     // (rimuovi se il compilatore segnala campo inesistente)
  cfg.bits_per_sample = 16;
  cfg.channels = 2;
  cfg.input_device = ADC_INPUT_LINE2;
  kit.begin(cfg);

  Serial.println("Codec ES8388 inizializzato (arduino-audio-driver)");

  // ===== SCRITTURE DIRETTE REGISTRI ES8388 (bus I2C 32/33) =====
  Wire.begin(32, 33);
  delay(50);
  ES8388Helper::setPgaGain(pga_reg_value);
  ES8388Helper::disableMicBias();
  ES8388Helper::forceDoubleSpeed96k();
  // ES8388Helper::printMasterModeDiag(); // decommenta per verifica MCLKDIV2
  Wire.end();
  delay(50);

  // ===== DISPLAY LCD (bus I2C indipendente 21/22, DOPO il codec) =====
  Wire.begin(21, 22);
  ui.begin();

  analyzer.calibrateDcOffset(readAudioKitBlock);

  Serial.println("\n>>> Sistema pronto <<<");
  Serial.println("Premi il pulsante (GPIO36) per avviare una sessione di misura\n");
  ui.lcd().setCursor(0, 1);
  ui.lcd().print("Pronto...");
}

// ===================================================================
// LOOP
// ===================================================================
void loop() {
  unsigned long now = millis();

  bool both_pressed = (digitalRead(PIN_START) == LOW && digitalRead(PIN_RESET) == LOW);
  if (both_pressed && !last_both_buttons_state && (now - last_combo_time) > COMBO_DEBOUNCE_MS) {
    last_combo_time = now;
    last_both_buttons_state = true;
    ui.enterGainMenu(pga_reg_value); // bloccante - la scheda si riavvia all'uscita
  }
  last_both_buttons_state = both_pressed;
  if (both_pressed) return;

  bool key3_state = digitalRead(PIN_KEY3) == LOW;
  if (key3_state && last_pinKey3_state == HIGH && (now - last_button_time_key3) > DEBOUNCE_MS) {
    last_button_time_key3 = now;
    float hz = analyzer.getNoiseExcludeBelowHz();
    ui.enterNoiseMenu(hz); // bloccante - si applica subito, nessun riavvio
    analyzer.setNoiseExcludeBelowHz(hz);
  }
  last_pinKey3_state = key3_state;

  bool pinIn_state = digitalRead(PIN_START);
  if (pinIn_state == LOW && last_pinIn_state == HIGH && (now - last_button_time_start) > DEBOUNCE_MS) {
    last_button_time_start = now;
    acquisition_active = !acquisition_active;

    if (acquisition_active) {
      Serial.println("\n>>> START acquisizione <<<");
      ui.lcd().clear();
      ui.lcd().setCursor(2, 0);
      ui.lcd().print("read buffer");
      resetBuffers();
      for (int flush = 0; flush < 8; flush++) {
        kit.readBytes((uint8_t*)raw_block, sizeof(raw_block));
      }
    } else {
      Serial.println(">>> STOP acquisizione <<<");
      evaluateSignalLevel();
      ui.updateLcd(last_thd_percent, last_thdn_percent, last_campioni, true);
    }
  }
  last_pinIn_state = pinIn_state;

  bool pinReset_state = digitalRead(PIN_RESET);
  if (pinReset_state == LOW && last_pinReset_state == HIGH && (now - last_button_time_reset) > DEBOUNCE_MS) {
    last_button_time_reset = now;
    Serial.println(">>> RESET campioni <<<");
    resetBuffers();
    ui.lcd().clear();
    ui.lcd().setCursor(0, 0);
    ui.lcd().print("Reset...");
    ui.lcd().setCursor(0, 1);
    char freq_disp[16];
    snprintf(freq_disp, sizeof(freq_disp), "Freq:%d", (int)frequenza);
    ui.lcd().print(freq_disp);
    delay(1000);
    if (!acquisition_active) ui.updateLcd(last_thd_percent, last_thdn_percent, last_campioni, true);
  }
  last_pinReset_state = pinReset_state;

  if (acquisition_active) {
    runOneMeasurement();
    if (acquisition_active) updateMediaContinua();
  }
}
