#ifndef THD_ANALYZER_H
#define THD_ANALYZER_H

#include <Arduino.h>

typedef float float32_t;

// Risultato di una singola analisi THD - tutti i valori utili in un colpo solo,
// invece di leggere una decina di variabili globali sparse come nello sketch originale.
struct THDResult {
  float frequenza      = 0.0f;
  float thd_percent    = 0.0f;
  float thdn_percent   = 0.0f;
  float snr_db         = 0.0f;
  float maxAbs         = 0.0f;
  int   harmonics_found = 0;
  int   harmonics_real  = 0;
  int   harm_max_dynamic = 0;
  bool  clipping        = false;
  int   clipCount       = 0;
  int   fund_bin        = 0;
  // Diagnostica energia di rete / rumore piu' forte (usata per stampe [DIAG])
  float powerLine_pct   = 0.0f;
  float worstNoise_hz   = -1.0f;
  float worstNoise_pct  = 0.0f;
};

class THDAnalyzer {
  public:
    // fft_size: dimensione FFT (potenza di 2). fs: sample rate reale in Hz.
    THDAnalyzer(int fft_size, int fs);
    ~THDAnalyzer();

    // Alloca i buffer (in PSRAM se disponibile) e genera la finestra.
    // Ritorna false se l'allocazione fallisce (memoria insufficiente).
    bool begin();

    // Calibrazione DC offset: da chiamare una volta in setup(), DOPO begin().
    // readBlockFn deve riempire dest[0..n) con campioni normalizzati [-1..1]
    // gia' pronti (stesso identico formato usato poi in acquireAndAnalyze).
    typedef int (*ReadRawFn)(float32_t *dest, int start_idx, float &maxAbsOut, int &clipCountOut, float dc_offset, bool dc_calibrated);
    void calibrateDcOffset(ReadRawFn readFn);

    // Esegue un ciclo completo: riempie il buffer FFT chiamando readFn ripetutamente,
    // calcola FFT + THD/THD+N/SNR, ritorna il risultato via reference.
    // stopRequested: puntatore a funzione che ritorna true se l'acquisizione va interrotta
    // (permette di controllare il pulsante STOP senza bloccare l'intera acquisizione).
    typedef bool (*StopCheckFn)();
    void acquireAndAnalyze(ReadRawFn readFn, StopCheckFn stopCheck, THDResult &result);

    // Parametri regolabili a runtime (menu soglia rumore)
    void setNoiseExcludeBelowHz(float hz) { noise_exclude_below_hz_ = hz; }
    float getNoiseExcludeBelowHz() const { return noise_exclude_below_hz_; }

    int fftSize() const { return fft_size_; }
    int maxFftSize() const { return fft_size_max_; }
    int sampleRate() const { return fs_; }
    float binHz() const { return (float)fs_ / fft_size_; }

    // Cambia la dimensione FFT attiva senza riallocare memoria (i buffer
    // restano dimensionati al massimo usato in costruzione/begin()).
    // n deve essere una potenza di 2, compresa tra 256 e maxFftSize().
    // Rigenera la finestra Blackman-Harris per la nuova dimensione.
    // Ritorna false (e non applica nulla) se n non e' valido.
    bool setActiveFftSize(int n);

  private:
    int fft_size_;      // dimensione FFT ATTIVA (puo' cambiare a runtime, <= fft_size_max_)
    int fft_size_max_;  // dimensione con cui sono stati allocati i buffer (fissa, mai riallocata)
    int fs_;

    float32_t *fft_input_ = nullptr;
    float32_t *fft_re_    = nullptr;
    float32_t *fft_im_    = nullptr;
    float32_t *window_    = nullptr;
    float32_t *powerSpec_ = nullptr;

    float dc_offset_ = 0.0f;
    bool  dc_calibrated_ = false;

    float noise_exclude_below_hz_ = 100.0f;

    static const int HARM_MAX_CAP = 40;
    static const int BIN_SUM_RADIUS = 4;

    void makeBlackmanHarris7();
    static void fftRadix2(float32_t *re, float32_t *im, int n);
    static void parabolicPeak(float ym1, float y0, float yp1, float &delta, float &yInterp);
    static bool isPowerLineHarmonic(int bin, float bin_hz);
};

#endif
