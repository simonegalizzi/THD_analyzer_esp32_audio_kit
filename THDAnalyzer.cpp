#include "THDAnalyzer.h"

THDAnalyzer::THDAnalyzer(int fft_size, int fs) : fft_size_(fft_size), fs_(fs) {}

THDAnalyzer::~THDAnalyzer() {
  // Nessun free esplicito: i buffer vivono per tutta la durata dello sketch,
  // stesso comportamento dell'originale (ps_malloc, mai liberati).
}

bool THDAnalyzer::begin() {
  bool psram_ok = psramFound();
  Serial.print("PSRAM rilevata: ");
  Serial.println(psram_ok ? "SI" : "NO");
  if (psram_ok) {
    Serial.print("PSRAM totale: ");
    Serial.print(ESP.getPsramSize() / 1024);
    Serial.println(" KB");
    Serial.print("PSRAM libera: ");
    Serial.print(ESP.getFreePsram() / 1024);
    Serial.println(" KB");
  }

  fft_input_ = (float32_t*) ps_malloc(sizeof(float32_t) * fft_size_);
  fft_re_    = (float32_t*) ps_malloc(sizeof(float32_t) * fft_size_);
  fft_im_    = (float32_t*) ps_malloc(sizeof(float32_t) * fft_size_);
  window_    = (float32_t*) ps_malloc(sizeof(float32_t) * fft_size_);
  powerSpec_ = (float32_t*) ps_malloc(sizeof(float32_t) * (fft_size_ / 2));

  if (!fft_input_ || !fft_re_ || !fft_im_ || !window_ || !powerSpec_) {
    Serial.println("ERRORE: allocazione buffer FFT fallita (memoria insufficiente)!");
    return false;
  }

  memset(fft_input_, 0, sizeof(float32_t) * fft_size_);
  memset(fft_re_, 0, sizeof(float32_t) * fft_size_);
  memset(fft_im_, 0, sizeof(float32_t) * fft_size_);
  memset(window_, 0, sizeof(float32_t) * fft_size_);
  memset(powerSpec_, 0, sizeof(float32_t) * (fft_size_ / 2));

  Serial.println("Buffer FFT allocati correttamente.");
  makeBlackmanHarris7();

  Serial.print("Risoluzione FFT: ");
  Serial.print(binHz(), 3);
  Serial.println(" Hz/bin");

  return true;
}

void THDAnalyzer::makeBlackmanHarris7() {
  const float a0 = 0.27105140069342f;
  const float a1 = 0.43329793923448f;
  const float a2 = 0.21812299954311f;
  const float a3 = 0.06592544638803f;
  const float a4 = 0.01081174209837f;
  const float a5 = 0.00077658482522f;
  const float a6 = 0.00001388721735f;
  const float N1 = fft_size_ - 1;

  Serial.println("Generazione finestra Blackman-Harris 7-term...");
  for (int n = 0; n < fft_size_; n++) {
    float x = 2.0f * PI * n / N1;
    window_[n] = a0
                - a1 * cosf(x)
                + a2 * cosf(2 * x)
                - a3 * cosf(3 * x)
                + a4 * cosf(4 * x)
                - a5 * cosf(5 * x)
                + a6 * cosf(6 * x);
  }
  Serial.println(" OK");
}

// Bit-reversal + butterfly, identica all'originale (nessuna dipendenza esterna,
// nessun limite di dimensione come esp-dsp).
void THDAnalyzer::fftRadix2(float32_t *re, float32_t *im, int n) {
  int j = 0;
  for (int i = 1; i < n; i++) {
    int bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      float32_t tr = re[i]; re[i] = re[j]; re[j] = tr;
      float32_t ti = im[i]; im[i] = im[j]; im[j] = ti;
    }
  }
  for (int len = 2; len <= n; len <<= 1) {
    float ang = -2.0f * PI / len;
    float wr_step = cosf(ang);
    float wi_step = sinf(ang);
    int half = len >> 1;
    for (int i = 0; i < n; i += len) {
      float cwr = 1.0f, cwi = 0.0f;
      for (int k = 0; k < half; k++) {
        int a = i + k;
        int b = a + half;
        float ur = re[a], ui = im[a];
        float vr = re[b] * cwr - im[b] * cwi;
        float vi = re[b] * cwi + im[b] * cwr;
        re[a] = ur + vr; im[a] = ui + vi;
        re[b] = ur - vr; im[b] = ui - vi;
        float nwr = cwr * wr_step - cwi * wi_step;
        float nwi = cwr * wi_step + cwi * wr_step;
        cwr = nwr; cwi = nwi;
      }
    }
  }
}

void THDAnalyzer::parabolicPeak(float ym1, float y0, float yp1, float &delta, float &yInterp) {
  float denom = (ym1 - 2.0f * y0 + yp1);
  if (fabsf(denom) < 1e-12f || y0 < ym1 || y0 < yp1) {
    delta = 0.0f;
    yInterp = y0;
    return;
  }
  delta = 0.5f * (ym1 - yp1) / denom;
  delta = constrain(delta, -0.5f, 0.5f);
  yInterp = y0 - 0.25f * (ym1 - yp1) * delta;
  if (yInterp < y0) yInterp = y0;
}

bool THDAnalyzer::isPowerLineHarmonic(int bin, float bin_hz) {
  float freq = bin * bin_hz;
  for (int h = 1; h <= 10; h++) {
    float power_freq = 50.0f * h;
    if (fabsf(freq - power_freq) < 2.0f) return true;
  }
  return false;
}

void THDAnalyzer::calibrateDcOffset(ReadRawFn readFn) {
  Serial.println("Calibrazione DC offset...");
  double sum = 0.0;
  int count = 0;
  int remaining = fft_size_;
  dc_calibrated_ = false;
  float dummyMax = 0.0f;
  int dummyClip = 0;

  while (remaining > 0) {
    int got = readFn(fft_input_, 0, dummyMax, dummyClip, 0.0f, false);
    if (got <= 0) break;
    int take = min(got, remaining);
    for (int i = 0; i < take; i++) {
      sum += fft_input_[i];
    }
    count += take;
    remaining -= take;
  }

  dc_offset_ = (count > 0) ? (float)(sum / count) : 0.0f;
  dc_calibrated_ = true;
  Serial.print(" DC offset rilevato: ");
  Serial.println(dc_offset_, 6);
  Serial.println(" Calibrazione OK!");
}

void THDAnalyzer::acquireAndAnalyze(ReadRawFn readFn, StopCheckFn stopCheck, THDResult &result) {
  float maxAbs = 0.0f;
  int clipCount = 0;

  int filled = 0;
  while (filled < fft_size_) {
    if (stopCheck && stopCheck()) break;
    int got = readFn(fft_input_, filled, maxAbs, clipCount, dc_offset_, dc_calibrated_);
    if (got <= 0) break;
    filled += got;
    if (filled > fft_size_) filled = fft_size_;
  }

  result.maxAbs = maxAbs;
  result.clipCount = clipCount;
  result.clipping = (maxAbs > 0.95f || clipCount > 10);

  for (int i = 0; i < fft_size_; i++) {
    fft_re_[i] = fft_input_[i] * window_[i];
    fft_im_[i] = 0.0f;
  }

  fftRadix2(fft_re_, fft_im_, fft_size_);

  const float scale = 1.0f / fft_size_;
  for (int i = 0; i < fft_size_ / 2; i++) {
    float re = fft_re_[i];
    float im = fft_im_[i];
    powerSpec_[i] = (re * re + im * im) * scale;
  }

  float bin_hz = binHz();
  int bin_ignore = ceilf(10.0f / bin_hz);

  int fund_bin = bin_ignore;
  float fund_pow = powerSpec_[fund_bin];
  for (int i = bin_ignore + 1; i < fft_size_ / 2; i++) {
    if (powerSpec_[i] > fund_pow) {
      fund_pow = powerSpec_[i];
      fund_bin = i;
    }
  }

  float ym1 = (fund_bin > 0) ? powerSpec_[fund_bin - 1] : 0.0f;
  float y0 = powerSpec_[fund_bin];
  float yp1 = (fund_bin < fft_size_ / 2 - 1) ? powerSpec_[fund_bin + 1] : 0.0f;
  float delta, interpAmp;
  parabolicPeak(ym1, y0, yp1, delta, interpAmp);
  float binFrac = fund_bin + delta;
  float f0 = binFrac * bin_hz;

  result.frequenza = f0;
  result.fund_bin = fund_bin;

  // H_MAX dinamico: dipende dalla fondamentale reale e dal sample rate,
  // con un margine di sicurezza di 1 armonica per il roll-off del filtro
  // anti-aliasing (non un muro perfetto a Nyquist esatto).
  int harm_max_dynamic = (f0 > 1.0f) ? (int)floorf((fs_ / 2.0f) / f0) - 1 : 2;
  if (harm_max_dynamic < 2) harm_max_dynamic = 2;
  if (harm_max_dynamic > HARM_MAX_CAP) harm_max_dynamic = HARM_MAX_CAP;
  result.harm_max_dynamic = harm_max_dynamic;

  double fundamentalPower = 0.0;
  for (int r = -BIN_SUM_RADIUS; r <= BIN_SUM_RADIUS; r++) {
    int b = fund_bin + r;
    if (b >= bin_ignore && b < fft_size_ / 2) fundamentalPower += powerSpec_[b];
  }

  // Stima rumore di fondo (esclude fondamentale, righe di rete, banda bassa)
  double noisePow = 0.0;
  int noiseBins = 0;
  double powerLinePow = 0.0;
  int worstNoiseBin = -1;
  double worstNoisePow = 0.0;
  for (int b = bin_ignore; b < fft_size_ / 2; b++) {
    bool skip = false;
    for (int h = 1; h <= harm_max_dynamic; h++) {
      int hb = (int)roundf(fund_bin * h);
      if (hb < bin_ignore || hb >= fft_size_ / 2) break;
      if (abs(b - hb) <= BIN_SUM_RADIUS + 2) { skip = true; break; }
    }
    bool isPowerLine = isPowerLineHarmonic(b, bin_hz);
    bool isLowFreq = (b * bin_hz) < noise_exclude_below_hz_;
    if (isPowerLine) powerLinePow += (double)powerSpec_[b];
    if (skip || isPowerLine || isLowFreq) continue;
    noisePow += (double)powerSpec_[b];
    noiseBins++;
    if (powerSpec_[b] > worstNoisePow) {
      worstNoisePow = powerSpec_[b];
      worstNoiseBin = b;
    }
  }
  double avgNoisePerBin = (noiseBins > 0) ? (noisePow / noiseBins) : 0.0;
  double expectedNoisePerHarmonic = avgNoisePerBin * (2 * BIN_SUM_RADIUS + 1);

  double harmonicPower = 0.0;
  int harmonics_found = 0;
  for (int h = 2; h <= harm_max_dynamic; h++) {
    int center = (int)roundf(binFrac * h);
    if (center >= fft_size_ / 2 - BIN_SUM_RADIUS) break;
    double hp = 0.0;
    for (int r = -BIN_SUM_RADIUS; r <= BIN_SUM_RADIUS; r++) {
      int b = center + r;
      if (b >= bin_ignore && b < fft_size_ / 2) hp += powerSpec_[b];
    }
    double hp_corrected = hp - expectedNoisePerHarmonic;
    if (hp_corrected > 0.0) harmonicPower += hp_corrected;
    harmonics_found++;
  }
  result.harmonics_found = harmonics_found;

  float thd = (fundamentalPower > 1e-12) ? sqrtf(harmonicPower / fundamentalPower) : 0.0f;
  result.thd_percent = thd * 100.0f;

  result.snr_db = (noisePow > 1e-18) ? 10.0f * log10f(fundamentalPower / noisePow) : 0.0f;

  double distortionAndNoise = harmonicPower + noisePow;
  float thdn = (fundamentalPower > 1e-12) ? sqrtf(distortionAndNoise / fundamentalPower) : 0.0f;
  result.thdn_percent = thdn * 100.0f;

  // Conteggio armoniche realmente sopra il pavimento di rumore (soglia 6dB)
  int harmonics_real = 0;
  if (noiseBins > 0) {
    double noiseThresholdPerHarmonic = expectedNoisePerHarmonic * 4.0;
    for (int h = 2; h <= harmonics_found + 1; h++) {
      int center = (int)roundf(binFrac * h);
      if (center >= fft_size_ / 2 - BIN_SUM_RADIUS) break;
      double hp = 0.0;
      for (int r = -BIN_SUM_RADIUS; r <= BIN_SUM_RADIUS; r++) {
        int b = center + r;
        if (b >= bin_ignore && b < fft_size_ / 2) hp += powerSpec_[b];
      }
      if (hp > noiseThresholdPerHarmonic) harmonics_real++;
    }
  }
  result.harmonics_real = harmonics_real;

  // Diagnostica energia di rete / rumore piu' forte (per stampe [DIAG] esterne)
  result.powerLine_pct = (fundamentalPower > 1e-12) ? sqrtf(powerLinePow / fundamentalPower) * 100.0f : 0.0f;
  result.worstNoise_hz = (worstNoiseBin >= 0) ? worstNoiseBin * bin_hz : -1.0f;
  result.worstNoise_pct = (fundamentalPower > 1e-12 && worstNoiseBin >= 0)
                              ? sqrtf(worstNoisePow / fundamentalPower) * 100.0f : 0.0f;
}
