#include "results_window.h"
#include "BinaryData.h"

#ifdef _WIN32
#include <windows.h>
#include <dwmapi.h>
#endif

#include <cmath>
#include <complex>
#include <numeric>
#include <algorithm>

// ── Popout window subclass with working close + wayverb title bar ───────────
class PopoutWindow : public DocumentWindow {
public:
    PopoutWindow(const String& name, Colour bg, int buttons)
        : DocumentWindow(name, bg, buttons) {}
    void closeButtonPressed() override { setVisible(false); }
};

// ═════════════════════════════════════════════════════════════════════════════
// Inferno colormap (outside anonymous namespace — used by SpectrogramDisplay)
// ═════════════════════════════════════════════════════════════════════════════

static Colour inferno(float t) {
    t = juce::jlimit(0.0f, 1.0f, t);
    struct Stop { float t; uint8 r, g, b; };
    static const Stop stops[] = {
        {0.00f,   0,   0,   4}, {0.13f,  40,  11,  84},
        {0.25f,  89,  13, 126}, {0.38f, 137,  21, 117},
        {0.50f, 188,  55,  84}, {0.63f, 227,  97,  49},
        {0.75f, 251, 150,  24}, {0.88f, 252, 206,  37},
        {1.00f, 252, 255, 164},
    };
    int i = 0;
    for (; i < 7; ++i) if (t <= stops[i + 1].t) break;
    const auto& a = stops[i];
    const auto& b = stops[i + 1];
    float f = (t - a.t) / (b.t - a.t);
    return Colour(uint8(a.r + f * (b.r - a.r)),
                  uint8(a.g + f * (b.g - a.g)),
                  uint8(a.b + f * (b.b - a.b)));
}

namespace {

using juce::Rectangle;  // disambiguate from wingdi.h Rectangle()

// ═════════════════════════════════════════════════════════════════════════════
// FFT (radix-2 Cooley-Tukey)
// ═════════════════════════════════════════════════════════════════════════════

void fft_inplace(std::vector<std::complex<float>>& x) {
    const size_t N = x.size();
    if (N <= 1) return;
    for (size_t i = 1, j = 0; i < N; ++i) {
        size_t bit = N >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(x[i], x[j]);
    }
    for (size_t len = 2; len <= N; len <<= 1) {
        float ang = -2.0f * float(M_PI) / float(len);
        std::complex<float> wlen(std::cos(ang), std::sin(ang));
        for (size_t i = 0; i < N; i += len) {
            std::complex<float> w(1.0f);
            for (size_t j = 0; j < len / 2; ++j) {
                auto u = x[i + j], v = x[i + j + len / 2] * w;
                x[i + j] = u + v;
                x[i + j + len / 2] = u - v;
                w *= wlen;
            }
        }
    }
}

size_t next_pow2(size_t n) {
    size_t p = 1; while (p < n) p <<= 1; return p;
}

// ═════════════════════════════════════════════════════════════════════════════
// FFT convolution — O(N log N)
// ═════════════════════════════════════════════════════════════════════════════

// Single-shot FFT convolution for short signals
std::vector<float> fft_convolve_short(const float* sig, size_t sig_len,
                                      const std::vector<std::complex<float>>& H_fft,
                                      size_t N) {
    std::vector<std::complex<float>> S(N, {0, 0});
    for (size_t i = 0; i < sig_len; ++i) S[i] = {sig[i], 0};
    fft_inplace(S);
    for (size_t i = 0; i < N; ++i) S[i] *= H_fft[i];
    for (auto& v : S) v = std::conj(v);
    fft_inplace(S);
    float inv = 1.0f / float(N);
    std::vector<float> out(N);
    for (size_t i = 0; i < N; ++i) out[i] = std::conj(S[i]).real() * inv;
    return out;
}

std::vector<float> fft_convolve(const std::vector<float>& signal,
                                const std::vector<float>& ir) {
    if (signal.empty() || ir.empty()) return {};

    // Cap IR length to 30 seconds worth at 96kHz (2.88M samples) — safety
    const size_t MAX_IR = 96000 * 30;
    size_t ir_len = std::min(ir.size(), MAX_IR);

    size_t out_len = signal.size() + ir_len - 1;

    // Use overlap-add if total FFT would exceed 2^22 (16M complex = 128MB)
    constexpr size_t MAX_SINGLE_FFT = 1 << 22;
    size_t single_N = next_pow2(signal.size() + ir_len - 1);

    if (single_N <= MAX_SINGLE_FFT) {
        // Small enough for single-shot FFT
        std::vector<std::complex<float>> S(single_N, {0, 0}), H(single_N, {0, 0});
        for (size_t i = 0; i < signal.size(); ++i) S[i] = {signal[i], 0};
        for (size_t i = 0; i < ir_len; ++i)        H[i] = {ir[i], 0};
        fft_inplace(S);
        fft_inplace(H);
        for (size_t i = 0; i < single_N; ++i) S[i] *= H[i];
        for (auto& v : S) v = std::conj(v);
        fft_inplace(S);
        float inv = 1.0f / float(single_N);
        for (auto& v : S) v = std::conj(v) * inv;
        std::vector<float> out(out_len);
        for (size_t i = 0; i < out_len; ++i) out[i] = S[i].real();
        float mx = 0;
        for (auto& s : out) mx = std::max(mx, std::abs(s));
        if (mx > 0) for (auto& s : out) s /= mx;
        return out;
    }

    // Overlap-add: process signal in blocks to keep FFT size manageable
    // Block size chosen so block + IR fits in MAX_SINGLE_FFT
    size_t block_size = MAX_SINGLE_FFT - ir_len;
    size_t N = next_pow2(block_size + ir_len - 1);  // FFT size per block

    // Pre-compute IR FFT once
    std::vector<std::complex<float>> H(N, {0, 0});
    for (size_t i = 0; i < ir_len; ++i) H[i] = {ir[i], 0};
    fft_inplace(H);

    std::vector<float> out(out_len, 0.0f);

    for (size_t offset = 0; offset < signal.size(); offset += block_size) {
        size_t this_block = std::min(block_size, signal.size() - offset);
        auto block_out = fft_convolve_short(signal.data() + offset, this_block, H, N);
        // Overlap-add into output
        size_t add_len = std::min(this_block + ir_len - 1, out_len - offset);
        for (size_t i = 0; i < add_len; ++i)
            out[offset + i] += block_out[i];
    }

    // Peak-normalize
    float mx = 0;
    for (auto& s : out) mx = std::max(mx, std::abs(s));
    if (mx > 0) for (auto& s : out) s /= mx;
    return out;
}

// ═════════════════════════════════════════════════════════════════════════════
// Octave-band filtering (ISO 3382 IEC 61260 Class 1)
// ═════════════════════════════════════════════════════════════════════════════

// Bandpass filter an IR into one octave band using FFT.
std::vector<float> octave_bandpass(const std::vector<float>& ir, double sr,
                                    double fc) {
    // Octave band edges: fc/√2 to fc×√2
    const double f_lo = fc / std::sqrt(2.0);
    const double f_hi = fc * std::sqrt(2.0);

    // FFT size: next power of 2 ≥ signal length
    size_t N = 1;
    while (N < ir.size()) N <<= 1;
    N <<= 1;  // 2× zero-padding for linear convolution

    std::vector<std::complex<float>> buf(N, {0.0f, 0.0f});
    for (size_t i = 0; i < ir.size(); ++i)
        buf[i] = {ir[i], 0.0f};

    fft_inplace(buf);

    // Apply bandpass with smooth Butterworth-like edges (4th order)
    const double df = sr / double(N);
    for (size_t k = 0; k <= N / 2; ++k) {
        double f = k * df;
        // 4th-order Butterworth magnitude: 1/sqrt(1 + (f/fc)^(2n))
        double g_hi = 1.0 / std::sqrt(1.0 + std::pow(f_lo / std::max(f, 1.0), 8.0));
        double g_lo = 1.0 / std::sqrt(1.0 + std::pow(f / f_hi, 8.0));
        float gain = static_cast<float>(g_hi * g_lo);
        buf[k] *= gain;
        if (k > 0 && k < N / 2)
            buf[N - k] *= gain;
    }

    // IFFT (conjugate trick)
    for (auto& c : buf) c = std::conj(c);
    fft_inplace(buf);
    const float scale = 1.0f / float(N);
    std::vector<float> out(ir.size());
    for (size_t i = 0; i < ir.size(); ++i)
        out[i] = buf[i].real() * scale;

    return out;
}

// ═════════════════════════════════════════════════════════════════════════════
// Acoustic metrics (ISO 3382)
// ═════════════════════════════════════════════════════════════════════════════

// Compute single-band metrics from an energy vector and its Schroeder EDC.
struct BandMetrics {
    double rt60 = 0, t30 = 0, edt = 0, c80 = 0, c50 = 0, d50 = 0, ts = 0;
};

BandMetrics computeBandMetrics(const std::vector<float>& sig, double sr) {
    BandMetrics bm;
    if (sig.empty()) return bm;

    const size_t N = sig.size();
    std::vector<double> energy(N);
    double total = 0;
    for (size_t i = 0; i < N; ++i) {
        energy[i] = double(sig[i]) * double(sig[i]);
        total += energy[i];
    }
    if (total <= 0) return bm;

    // Schroeder backward integration
    std::vector<double> edc(N);
    edc.back() = energy.back();
    for (int i = int(N) - 2; i >= 0; --i)
        edc[i] = edc[i + 1] + energy[i];
    double peak = edc[0];

    // RT60 (T20: -5 to -25 dB, ×3)
    int idx_5 = -1, idx_25 = -1, idx_35 = -1, idx_10 = -1;
    for (size_t i = 0; i < N; ++i) {
        double db = 10.0 * std::log10(std::max(edc[i] / peak, 1e-15));
        if (idx_5  < 0 && db <= -5.0)  idx_5  = int(i);
        if (idx_10 < 0 && db <= -10.0) idx_10 = int(i);
        if (idx_25 < 0 && db <= -25.0) idx_25 = int(i);
        if (idx_35 < 0 && db <= -35.0) { idx_35 = int(i); break; }
    }
    if (idx_5 >= 0 && idx_25 > idx_5)
        bm.rt60 = double(idx_25 - idx_5) / sr * 3.0;

    // T30 (-5 to -35 dB, ×2)
    if (idx_5 >= 0 && idx_35 > idx_5)
        bm.t30 = double(idx_35 - idx_5) / sr * 2.0;

    // EDT (0 to -10 dB, ×6)
    if (idx_10 > 0)
        bm.edt = double(idx_10) / sr * 6.0;

    // C80
    int n80 = int(0.080 * sr);
    if (n80 > 0 && n80 < int(N)) {
        double early = 0, late = 0;
        for (int i = 0; i < n80; ++i) early += energy[i];
        for (size_t i = n80; i < N; ++i) late += energy[i];
        if (late > 0) bm.c80 = 10.0 * std::log10(early / late);
    }

    // C50
    int n50 = int(0.050 * sr);
    if (n50 > 0 && n50 < int(N)) {
        double early = 0, late = 0;
        for (int i = 0; i < n50; ++i) early += energy[i];
        for (size_t i = n50; i < N; ++i) late += energy[i];
        if (late > 0) bm.c50 = 10.0 * std::log10(early / late);
    }

    // D50 (%)
    if (n50 > 0 && n50 < int(N)) {
        double early = 0;
        for (int i = 0; i < n50; ++i) early += energy[i];
        bm.d50 = early / total * 100.0;
    }

    // Ts (Centre Time, ms) = ∫t·h²(t)dt / ∫h²(t)dt
    {
        double num = 0;
        for (size_t i = 0; i < N; ++i)
            num += (double(i) / sr) * energy[i];
        bm.ts = (num / total) * 1000.0;  // convert to ms
    }

    return bm;
}

AcousticMetrics computeMetrics(const std::vector<float>& ir, double sr) {
    AcousticMetrics m;
    if (ir.empty()) return m;

    // Broadband metrics
    auto bb = computeBandMetrics(ir, sr);
    m.rt60 = bb.rt60;
    m.t30  = bb.t30;
    m.edt  = bb.edt;
    m.c80  = bb.c80;
    m.c50  = bb.c50;
    m.d50  = bb.d50;
    m.ts   = bb.ts;

    // Per-octave band metrics
    for (int b = 0; b < kNumBands; ++b) {
        auto filtered = octave_bandpass(ir, sr, kBandCentres[b]);
        auto bm = computeBandMetrics(filtered, sr);
        m.band_rt60[b] = bm.rt60;
        m.band_t30[b]  = bm.t30;
        m.band_edt[b]  = bm.edt;
        m.band_c80[b]  = bm.c80;
        m.band_c50[b]  = bm.c50;
        m.band_d50[b]  = bm.d50;
        m.band_ts[b]   = bm.ts;
    }

    // Bass Ratio: avg(RT125,RT250) / avg(RT500,RT1k)
    double rt_lo = (m.band_rt60[0] + m.band_rt60[1]) * 0.5;
    double rt_mid = (m.band_rt60[2] + m.band_rt60[3]) * 0.5;
    m.br = (rt_mid > 0.01) ? rt_lo / rt_mid : 0;

    return m;
}

// ═════════════════════════════════════════════════════════════════════════════
// Spectrogram computation (dynamic frequency cap + adaptive hop)
// ═════════════════════════════════════════════════════════════════════════════

struct SpectrogramResult {
    Image image;
    double min_freq;
    double max_freq;
};

SpectrogramResult compute_spectrogram(const std::vector<float>& data, double sr,
                                      float db_floor = -80.0f,
                                      bool log_freq = true) {
    const int fft_size = 1024;
    const int half = fft_size / 2;
    double nyquist = sr * 0.5;
    const double f_min = 20.0;  // lowest displayed frequency

    if ((int)data.size() < fft_size)
        return {Image(Image::RGB, 1, 1, true), f_min, nyquist};

    // Adaptive hop to limit max frames for performance
    const int max_frames = 2000;
    int hop = 256;
    if ((int(data.size()) - fft_size) / hop > max_frames)
        hop = std::max(256, (int(data.size()) - fft_size) / max_frames);
    const int n_frames = std::max(1, (int(data.size()) - fft_size) / hop + 1);

    // Hann window
    std::vector<float> win(fft_size);
    for (int i = 0; i < fft_size; ++i)
        win[i] = 0.5f * (1.0f - std::cos(2.0f * float(M_PI) * i / (fft_size - 1)));

    std::vector<std::vector<float>> mag_db(n_frames, std::vector<float>(half, db_floor));
    for (int f = 0; f < n_frames; ++f) {
        std::vector<std::complex<float>> buf(fft_size);
        int offset = f * hop;
        for (int i = 0; i < fft_size; ++i)
            buf[i] = {data[offset + i] * win[i], 0.0f};
        fft_inplace(buf);
        for (int i = 0; i < half; ++i) {
            float mag = std::abs(buf[i]) / float(fft_size);
            mag_db[f][i] = 20.0f * std::log10(std::max(mag, 1e-10f));
        }
    }

    // Find global max for normalization
    float global_max = db_floor;
    for (auto& fr : mag_db)
        for (auto v : fr) global_max = std::max(global_max, v);

    // Dynamic frequency cap: find highest bin with energy above noise floor
    float thresh = db_floor + 12.0f;
    int max_bin = half;
    for (int b = half - 1; b >= 0; --b) {
        for (int f = 0; f < n_frames; ++f) {
            if (mag_db[f][b] > thresh) { max_bin = b + 1; goto found; }
        }
    }
    found:
    // Round up to nice frequency
    double raw_max = double(max_bin) / half * nyquist;
    double nice[] = {4000, 6000, 8000, 10000, 12000, 16000, 20000};
    double eff_max = nyquist;
    for (double nf : nice) { if (nf >= raw_max) { eff_max = nf; break; } }
    eff_max = std::min(eff_max, nyquist);

    // Build frequency-mapped image (log or linear).
    const int n_rows = 256;
    Image img(Image::RGB, n_frames, n_rows, true);

    if (log_freq) {
        const float log_min = std::log10(float(f_min));
        const float log_max = std::log10(float(eff_max));
        for (int row = 0; row < n_rows; ++row) {
            float frac = float(row) / float(n_rows - 1);
            float freq = std::pow(10.0f, log_min + frac * (log_max - log_min));
            float lin_bin = float(freq / nyquist * half);
            int b0 = juce::jlimit(0, half - 1, int(lin_bin));
            int b1 = juce::jlimit(0, half - 1, b0 + 1);
            float t_interp = lin_bin - float(b0);
            for (int f = 0; f < n_frames; ++f) {
                float db = mag_db[f][b0] * (1.0f - t_interp)
                         + mag_db[f][b1] * t_interp;
                float t = (db - db_floor) / (global_max - db_floor);
                t = juce::jlimit(0.0f, 1.0f, t);
                img.setPixelAt(f, n_rows - 1 - row, inferno(t));
            }
        }
    } else {
        // Linear frequency mapping: row maps linearly from f_min to eff_max
        int bin_min = juce::jlimit(0, half - 1, int(f_min / nyquist * half));
        int bin_max = juce::jlimit(0, half - 1, int(eff_max / nyquist * half));
        for (int row = 0; row < n_rows; ++row) {
            float frac = float(row) / float(n_rows - 1);
            float lin_bin = float(bin_min) + frac * float(bin_max - bin_min);
            int b0 = juce::jlimit(0, half - 1, int(lin_bin));
            int b1 = juce::jlimit(0, half - 1, b0 + 1);
            float t_interp = lin_bin - float(b0);
            for (int f = 0; f < n_frames; ++f) {
                float db = mag_db[f][b0] * (1.0f - t_interp)
                         + mag_db[f][b1] * t_interp;
                float t = (db - db_floor) / (global_max - db_floor);
                t = juce::jlimit(0.0f, 1.0f, t);
                img.setPixelAt(f, n_rows - 1 - row, inferno(t));
            }
        }
    }
    return {img, f_min, eff_max};
}

// ═════════════════════════════════════════════════════════════════════════════
// Axis drawing helpers
// ═════════════════════════════════════════════════════════════════════════════

Rectangle<int> plotArea(const Rectangle<int>& bounds) {
    return {bounds.getX() + axis::left, bounds.getY() + axis::top,
            bounds.getWidth() - axis::left - axis::right,
            bounds.getHeight() - axis::top - axis::bottom};
}

double chooseTickSpacing(double duration) {
    if (duration <= 0) return 0.1;
    double nice[] = {0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 15, 30, 60};
    double target = duration / 10.0;
    for (double n : nice) if (n >= target) return n;
    return 60.0;
}

String formatTime(double s) {
    if (s < 60) return String(s, 1) + "s";
    int m = int(s) / 60;
    int sec = int(s) % 60;
    return String(m) + ":" + String(sec).paddedLeft('0', 2);
}

void drawTimeAxis(Graphics& g, const Rectangle<int>& plot, double dur) {
    if (dur <= 0) return;
    double tick = chooseTickSpacing(dur);
    g.setFont(9.0f);
    for (double t = 0; t <= dur + tick * 0.01; t += tick) {
        float x = plot.getX() + float(t / dur) * plot.getWidth();
        if (x > plot.getRight() + 1) break;
        g.setColour(wv_theme::text_dim);
        g.drawVerticalLine(int(x), float(plot.getBottom()), float(plot.getBottom() + 3));
        String label = (tick >= 1.0) ? String(int(t + 0.5)) + "s"
                                     : String(t, (tick < 0.1 ? 2 : 1)) + "s";
        g.drawText(label, int(x) - 18, plot.getBottom() + 3, 36, 13,
                   Justification::centred);
    }
}

void drawFreqAxisLogY(Graphics& g, const Rectangle<int>& plot,
                      double minFreq, double maxFreq) {
    if (minFreq <= 0 || maxFreq <= minFreq) return;
    float log_min = std::log10(float(minFreq));
    float log_max = std::log10(float(maxFreq));
    g.setFont(9.0f);
    struct FT { float f; const char* l; };
    FT ticks[] = {{20,"20"},{50,"50"},{100,"100"},{200,"200"},{500,"500"},
                  {1000,"1k"},{2000,"2k"},{5000,"5k"},{10000,"10k"},{20000,"20k"}};
    for (auto& ft : ticks) {
        if (ft.f <= minFreq || ft.f >= maxFreq) continue;
        float frac = (std::log10(ft.f) - log_min) / (log_max - log_min);
        int y = plot.getY() + int((1.0f - frac) * plot.getHeight());
        g.setColour(wv_theme::grid.withAlpha(0.25f));
        g.drawHorizontalLine(y, float(plot.getX()), float(plot.getRight()));
        g.setColour(wv_theme::text_dim);
        g.drawText(ft.l, plot.getX() - axis::left, y - 6, axis::left - 4, 12,
                   Justification::centredRight);
    }
}

void drawFreqAxisLinY(Graphics& g, const Rectangle<int>& plot,
                      double minFreq, double maxFreq) {
    if (maxFreq <= minFreq) return;
    g.setFont(9.0f);
    // Choose nice tick spacing for linear axis
    double range = maxFreq - minFreq;
    double target = range / 8.0;
    double nice[] = {50, 100, 200, 500, 1000, 2000, 5000, 10000};
    double tick = 1000;
    for (double n : nice) { if (n >= target) { tick = n; break; } }
    double first = std::ceil(minFreq / tick) * tick;
    for (double f = first; f < maxFreq; f += tick) {
        float frac = float((f - minFreq) / (maxFreq - minFreq));
        int y = plot.getY() + int((1.0f - frac) * plot.getHeight());
        g.setColour(wv_theme::grid.withAlpha(0.25f));
        g.drawHorizontalLine(y, float(plot.getX()), float(plot.getRight()));
        g.setColour(wv_theme::text_dim);
        String label = (f >= 1000) ? String(int(f / 1000)) + "k" : String(int(f));
        g.drawText(label, plot.getX() - axis::left, y - 6, axis::left - 4, 12,
                   Justification::centredRight);
    }
}

void drawFreqAxisLog(Graphics& g, const Rectangle<int>& plot,
                     float f_min, float f_max) {
    float log_min = std::log10(f_min), log_max = std::log10(f_max);
    g.setFont(9.0f);
    struct FT { float f; const char* l; };
    FT ticks[] = {{20,"20"},{50,"50"},{100,"100"},{200,"200"},{500,"500"},
                  {1000,"1k"},{2000,"2k"},{5000,"5k"},{10000,"10k"},{20000,"20k"}};
    for (auto& ft : ticks) {
        if (ft.f < f_min || ft.f > f_max) continue;
        float x = plot.getX() + (std::log10(ft.f) - log_min) / (log_max - log_min)
                  * plot.getWidth();
        g.setColour(wv_theme::grid.withAlpha(0.25f));
        g.drawVerticalLine(int(x), float(plot.getY()), float(plot.getBottom()));
        g.setColour(wv_theme::text_dim);
        g.drawVerticalLine(int(x), float(plot.getBottom()), float(plot.getBottom() + 3));
        g.drawText(ft.l, int(x) - 16, plot.getBottom() + 3, 32, 13,
                   Justification::centred);
    }
}

void drawDbAxis(Graphics& g, const Rectangle<int>& plot,
                float db_min, float db_max, float step = 10.0f) {
    g.setFont(9.0f);
    for (float db = db_min; db <= db_max + 0.1f; db += step) {
        float y = plot.getY() + (1.0f - (db - db_min) / (db_max - db_min))
                  * plot.getHeight();
        g.setColour(wv_theme::grid.withAlpha(0.25f));
        g.drawHorizontalLine(int(y), float(plot.getX()), float(plot.getRight()));
        g.setColour(wv_theme::text_dim);
        g.drawText(String(int(db)), plot.getX() - axis::left, int(y) - 6,
                   axis::left - 4, 12, Justification::centredRight);
    }
}

void drawAmpAxis(Graphics& g, const Rectangle<int>& plot) {
    g.setFont(9.0f);
    float mid = plot.getY() + plot.getHeight() * 0.5f;
    g.setColour(wv_theme::grid);
    g.drawHorizontalLine(int(mid), float(plot.getX()), float(plot.getRight()));
    g.setColour(wv_theme::text_dim);
    g.drawText("+1", plot.getX() - axis::left, plot.getY() - 6,
               axis::left - 4, 12, Justification::centredRight);
    g.drawText("0", plot.getX() - axis::left, int(mid) - 6,
               axis::left - 4, 12, Justification::centredRight);
    g.drawText("-1", plot.getX() - axis::left, plot.getBottom() - 6,
               axis::left - 4, 12, Justification::centredRight);
}

void styleBtn(TextButton& btn, Colour bg, Colour fg) {
    btn.setColour(TextButton::buttonColourId, bg);
    btn.setColour(TextButton::textColourOffId, fg);
}

// ═════════════════════════════════════════════════════════════════════════════
// Air absorption filter — gentle HF rolloff to reduce metallic artifacts
// ═════════════════════════════════════════════════════════════════════════════

// 2nd-order Butterworth low-pass applied to late portion of IR.
// This simulates air absorption and reduces harsh metallic ringing
// from coherent raytracer reflections.
void apply_air_absorption(std::vector<float>& ir, double sr) {
    if (ir.empty() || sr <= 0) return;

    // Cutoff at 10 kHz — high enough to preserve brightness,
    // low enough to tame metallic artifacts.
    double fc = 10000.0;
    if (fc >= sr * 0.49) fc = sr * 0.49;

    // Butterworth 2nd-order coefficients
    double w0 = 2.0 * M_PI * fc / sr;
    double cosw0 = std::cos(w0);
    double sinw0 = std::sin(w0);
    double alpha = sinw0 / std::sqrt(2.0);  // Q = 1/sqrt(2)

    double b0 = (1.0 - cosw0) / 2.0;
    double b1 = 1.0 - cosw0;
    double b2 = (1.0 - cosw0) / 2.0;
    double a0 = 1.0 + alpha;
    double a1 = -2.0 * cosw0;
    double a2 = 1.0 - alpha;

    // Normalize
    b0 /= a0; b1 /= a0; b2 /= a0;
    a1 /= a0; a2 /= a0;

    // Only filter the late part of the IR (after the direct sound).
    // The first 5ms stays untouched to preserve the attack transient.
    size_t onset = std::min(ir.size(), size_t(sr * 0.005));

    double x1 = 0, x2 = 0, y1 = 0, y2 = 0;

    // Prime the filter with the onset samples (don't modify them)
    for (size_t i = 0; i < onset && i < ir.size(); ++i) {
        double x0 = ir[i];
        double y0 = b0 * x0 + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1; x1 = x0;
        y2 = y1; y1 = y0;
    }

    // Apply filter to the late part with crossfade
    size_t fade_len = std::min(size_t(sr * 0.002), ir.size() - onset);
    for (size_t i = onset; i < ir.size(); ++i) {
        double x0 = ir[i];
        double y0 = b0 * x0 + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1; x1 = x0;
        y2 = y1; y1 = y0;

        if (i < onset + fade_len) {
            // Crossfade from dry to filtered over 2ms
            double t = double(i - onset) / fade_len;
            ir[i] = float(x0 * (1.0 - t) + y0 * t);
        } else {
            ir[i] = float(y0);
        }
    }
}

// Common zoom/pan mouse helpers for all plot components
void handlePlotWheel(Component& c, PlotZoom& z, const MouseEvent& e,
                     const MouseWheelDetails& w) {
    auto plot = plotArea(c.getLocalBounds());
    if (!plot.contains(e.getPosition())) return;

    // Horizontal scroll wheel → pan X
    if (std::abs(w.deltaX) > 0.001f) {
        z.panX(-double(w.deltaX) * 0.15 * (z.x1 - z.x0));
        c.repaint();
        return;
    }

    double cx = double(e.x - plot.getX()) / plot.getWidth();
    double cy = double(e.y - plot.getY()) / plot.getHeight();
    // Map cursor to zoom space
    cx = z.x0 + cx * (z.x1 - z.x0);
    cy = z.y0 + cy * (z.y1 - z.y0);
    double factor = (w.deltaY > 0) ? 0.8 : 1.25;
    if (e.mods.isShiftDown())
        z.zoomY(cy, factor);
    else
        z.zoomX(cx, factor);
    c.repaint();
}

void handlePlotDragStart(Point<float>& drag_start, const MouseEvent& e) {
    drag_start = e.position;
}

void handlePlotDrag(Component& c, PlotZoom& z, Point<float>& drag_start,
                    const MouseEvent& e) {
    auto plot = plotArea(c.getLocalBounds());
    float dx = e.position.x - drag_start.x;
    float dy = e.position.y - drag_start.y;
    drag_start = e.position;
    z.panX(-double(dx) / plot.getWidth() * (z.x1 - z.x0));
    z.panY(-double(dy) / plot.getHeight() * (z.y1 - z.y0));
    c.repaint();
}

void handlePlotDoubleClick(Component& c, PlotZoom& z) {
    z.reset();
    c.repaint();
}

// ═════════════════════════════════════════════════════════════════════════════
// LUFS measurement (simplified ITU-R BS.1770-4)
// K-weighting: high-shelf (+4 dB @ ~1.5 kHz) + high-pass (~38 Hz)
// ═════════════════════════════════════════════════════════════════════════════

struct Biquad {
    double b0, b1, b2, a1, a2;
    double z1 = 0, z2 = 0;
    float process(float x) {
        double y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return static_cast<float>(y);
    }
    void reset() { z1 = z2 = 0; }
};

double measure_lufs(const std::vector<float>& samples, double sr) {
    if (samples.empty() || sr <= 0) return -100.0;

    // K-weighting stage 1: high-shelf (head model)
    // Pre-computed for 48 kHz, adjusted via sample rate ratio
    Biquad shelf;
    {
        // Analog prototype: +4 dB shelf above ~1500 Hz
        const double fc = 1681.97;
        const double G = std::pow(10.0, 4.0 / 20.0);  // +4 dB
        const double K = std::tan(M_PI * fc / sr);
        const double Vh = G;
        const double Vb = std::sqrt(G);
        const double denom = 1.0 + std::sqrt(2.0) * K + K * K;
        shelf.b0 = (Vh + Vb * std::sqrt(2.0) * K + K * K) / denom;
        shelf.b1 = 2.0 * (K * K - Vh) / denom;
        shelf.b2 = (Vh - Vb * std::sqrt(2.0) * K + K * K) / denom;
        shelf.a1 = 2.0 * (K * K - 1.0) / denom;
        shelf.a2 = (1.0 - std::sqrt(2.0) * K + K * K) / denom;
    }

    // K-weighting stage 2: high-pass at ~38 Hz
    Biquad hp;
    {
        const double fc = 38.0;
        const double K = std::tan(M_PI * fc / sr);
        const double denom = 1.0 + std::sqrt(2.0) * K + K * K;
        hp.b0 = 1.0 / denom;
        hp.b1 = -2.0 / denom;
        hp.b2 = 1.0 / denom;
        hp.a1 = 2.0 * (K * K - 1.0) / denom;
        hp.a2 = (1.0 - std::sqrt(2.0) * K + K * K) / denom;
    }

    // Apply K-weighting and compute mean square
    double sum_sq = 0.0;
    for (size_t i = 0; i < samples.size(); ++i) {
        float s = shelf.process(samples[i]);
        s = hp.process(s);
        sum_sq += static_cast<double>(s) * s;
    }

    double mean_sq = sum_sq / static_cast<double>(samples.size());
    if (mean_sq < 1e-20) return -100.0;

    return -0.691 + 10.0 * std::log10(mean_sq);
}

}  // namespace

// ═════════════════════════════════════════════════════════════════════════════
// Metrics table display (ISO 3382)
// ═════════════════════════════════════════════════════════════════════════════

void MetricsTableDisplay::setMetrics(const AcousticMetrics& m) {
    metrics_ = m;
    repaint();
}

void MetricsTableDisplay::paint(Graphics& g) {
    auto area = getLocalBounds();
    g.fillAll(wv_theme::bg_panel);
    g.setColour(wv_theme::grid);
    g.drawRect(area);

    const float pad = 4.0f;
    const float rowH = 18.0f;
    const int nCols = kNumBands + 2;  // label + 6 bands + broadband

    // Column widths
    const float labelW = 52.0f;
    const float colW = float(area.getWidth() - labelW - pad * 2) / float(nCols - 1);

    float x0 = float(area.getX()) + pad;
    float y = float(area.getY()) + pad;

    // Title
    g.setFont(Font(13.0f).boldened());
    g.setColour(wv_theme::cyan);
    g.drawText("ISO 3382 Acoustic Parameters",
               juce::Rectangle<float>(x0, y, float(area.getWidth()) - pad * 2, rowH),
               Justification::centredLeft);
    y += rowH + 2;

    // Header row
    g.setFont(Font(11.0f).boldened());
    g.setColour(wv_theme::text);
    g.drawText("Param", juce::Rectangle<float>(x0, y, labelW, rowH),
               Justification::centredLeft);
    for (int b = 0; b < kNumBands; ++b) {
        String hdr;
        if (kBandCentres[b] >= 1000)
            hdr = String(int(kBandCentres[b] / 1000)) + "k";
        else
            hdr = String(int(kBandCentres[b]));
        g.drawText(hdr, juce::Rectangle<float>(x0 + labelW + b * colW, y, colW, rowH),
                   Justification::centred);
    }
    g.setColour(wv_theme::orange);
    g.drawText("Broad", juce::Rectangle<float>(x0 + labelW + kNumBands * colW, y, colW, rowH),
               Justification::centred);
    y += rowH;

    // Separator
    g.setColour(wv_theme::grid);
    g.drawLine(x0, y, x0 + float(area.getWidth()) - pad * 2, y, 1.0f);
    y += 2;

    // Data rows
    struct Row {
        const char* label;
        const double* band;
        double broad;
        const char* unit;
        int decimals;
    };
    const Row rows[] = {
        {"RT60",  metrics_.band_rt60, metrics_.rt60, "s",  2},
        {"T30",   metrics_.band_t30,  metrics_.t30,  "s",  2},
        {"EDT",   metrics_.band_edt,  metrics_.edt,  "s",  2},
        {"C80",   metrics_.band_c80,  metrics_.c80,  "dB", 1},
        {"C50",   metrics_.band_c50,  metrics_.c50,  "dB", 1},
        {"D50",   metrics_.band_d50,  metrics_.d50,  "%",  1},
        {"Ts",    metrics_.band_ts,   metrics_.ts,   "ms", 0},
    };

    g.setFont(Font(11.0f));
    for (const auto& row : rows) {
        // Label
        g.setColour(wv_theme::emphasis);
        g.drawText(String(row.label),
                   juce::Rectangle<float>(x0, y, labelW, rowH),
                   Justification::centredLeft);

        // Per-band values
        g.setColour(wv_theme::text);
        for (int b = 0; b < kNumBands; ++b) {
            String val = String(row.band[b], row.decimals);
            g.drawText(val,
                       juce::Rectangle<float>(x0 + labelW + b * colW, y, colW, rowH),
                       Justification::centred);
        }

        // Broadband
        g.setColour(wv_theme::orange);
        g.drawText(String(row.broad, row.decimals),
                   juce::Rectangle<float>(x0 + labelW + kNumBands * colW, y, colW, rowH),
                   Justification::centred);
        y += rowH;
    }

    // Bass Ratio row
    g.setColour(wv_theme::emphasis);
    g.drawText("BR",
               juce::Rectangle<float>(x0, y, labelW, rowH),
               Justification::centredLeft);
    g.setColour(wv_theme::orange);
    g.drawText(String(metrics_.br, 2),
               juce::Rectangle<float>(x0 + labelW + kNumBands * colW, y, colW, rowH),
               Justification::centred);
}

////////////////////////////////////////////////////////////////////////////////
// WaveformDisplay
////////////////////////////////////////////////////////////////////////////////

void WaveformDisplay::setData(const std::vector<float>& data, double sr,
                              const String& label) {
    data_ = data; sample_rate_ = sr; label_ = label; repaint();
}

void WaveformDisplay::paint(Graphics& g) {
    g.fillAll(wv_theme::bg_plot);
    auto plot = plotArea(getLocalBounds());
    g.setColour(wv_theme::text); g.setFont(11.0f);
    g.drawText(label_, plot.getX(), getLocalBounds().getY() + 2,
               plot.getWidth(), axis::top - 2, Justification::centredLeft);
    if (data_.empty()) {
        g.setColour(wv_theme::text_dim);
        g.drawText("No data", plot, Justification::centred);
        return;
    }
    // Compute visible time range from zoom state
    double total_dur = data_.size() / sample_rate_;
    double vis_t0 = zoom_.x0 * total_dur;
    double vis_t1 = zoom_.x1 * total_dur;
    drawAmpAxis(g, plot);
    drawTimeAxis(g, plot, vis_t1 - vis_t0);
    g.setColour(wv_theme::grid); g.drawRect(plot);

    // Visible sample range
    size_t N = data_.size();
    size_t s0 = size_t(zoom_.x0 * N);
    size_t s1 = std::min(size_t(zoom_.x1 * N), N);
    if (s0 >= s1) s1 = std::min(s0 + 1, N);
    size_t visN = s1 - s0;

    float peak = 0;
    for (size_t i = s0; i < s1; ++i) peak = std::max(peak, std::abs(data_[i]));
    if (peak == 0) peak = 1;
    float midY = plot.getY() + plot.getHeight() * 0.5f;
    float halfH = plot.getHeight() * 0.5f - 2.0f;

    int pw = plot.getWidth();

    // Min/max envelope rendering — fill between extremes per pixel column
    Path envTop, envBot;
    for (int px = 0; px < pw; ++px) {
        size_t i0 = s0 + size_t(double(px) / pw * visN);
        size_t i1 = s0 + size_t(double(px + 1) / pw * visN);
        i1 = std::min(i1, s1);
        if (i0 >= s1) break;
        if (i1 <= i0) i1 = i0 + 1;
        float lo = data_[i0], hi = data_[i0];
        for (size_t i = i0 + 1; i < i1; ++i) {
            lo = std::min(lo, data_[i]);
            hi = std::max(hi, data_[i]);
        }
        float yHi = midY - (hi / peak) * halfH;
        float yLo = midY - (lo / peak) * halfH;
        float x = float(plot.getX() + px);
        if (px == 0) { envTop.startNewSubPath(x, yHi); envBot.startNewSubPath(x, yLo); }
        else         { envTop.lineTo(x, yHi);           envBot.lineTo(x, yLo); }
    }

    // Draw filled envelope
    Path envelope(envTop);
    for (int px = pw - 1; px >= 0; --px) {
        size_t i0 = s0 + size_t(double(px) / pw * visN);
        size_t i1 = s0 + size_t(double(px + 1) / pw * visN);
        i1 = std::min(i1, s1);
        if (i0 >= s1) continue;
        if (i1 <= i0) i1 = i0 + 1;
        float lo = data_[i0];
        for (size_t i = i0 + 1; i < i1; ++i)
            lo = std::min(lo, data_[i]);
        float yLo = midY - (lo / peak) * halfH;
        envelope.lineTo(float(plot.getX() + px), yLo);
    }
    envelope.closeSubPath();

    g.setColour(wv_theme::cyan.withAlpha(0.25f));
    g.fillPath(envelope);
    g.setColour(wv_theme::cyan);
    g.strokePath(envTop, PathStrokeType(1.0f));
    g.strokePath(envBot, PathStrokeType(1.0f));
}

void WaveformDisplay::mouseWheelMove(const MouseEvent& e, const MouseWheelDetails& w) {
    handlePlotWheel(*this, zoom_, e, w);
}
void WaveformDisplay::mouseDown(const MouseEvent& e) {
    handlePlotDragStart(drag_start_, e);
}
void WaveformDisplay::mouseDrag(const MouseEvent& e) {
    handlePlotDrag(*this, zoom_, drag_start_, e);
}
void WaveformDisplay::mouseDoubleClick(const MouseEvent&) {
    handlePlotDoubleClick(*this, zoom_);
}

////////////////////////////////////////////////////////////////////////////////
// SpectrumDisplay (peak-normalized)
////////////////////////////////////////////////////////////////////////////////

void SpectrumDisplay::setData(const std::vector<float>& data, double sr) {
    sample_rate_ = sr;
    if (data.empty()) { magnitudes_db_.clear(); freq_axis_.clear(); repaint(); return; }
    size_t N = next_pow2(std::max(data.size(), size_t(4096)));
    std::vector<std::complex<float>> fd(N, {0, 0});
    for (size_t i = 0; i < data.size(); ++i) fd[i] = {data[i], 0};
    fft_inplace(fd);
    size_t half = N / 2;
    magnitudes_db_.resize(half);
    freq_axis_.resize(half);
    for (size_t i = 0; i < half; ++i) {
        float mag = std::abs(fd[i]) / float(N);
        magnitudes_db_[i] = 20.0f * std::log10(std::max(mag, 1e-10f));
        freq_axis_[i] = float(i) * float(sr) / float(N);
    }
    // Peak-normalize so shape is visible regardless of IR amplitude
    float maxDb = -999.0f;
    for (auto m : magnitudes_db_) maxDb = std::max(maxDb, m);
    for (auto& m : magnitudes_db_) m -= maxDb;
    repaint();
}

void SpectrumDisplay::paint(Graphics& g) {
    g.fillAll(wv_theme::bg_plot);
    auto plot = plotArea(getLocalBounds());
    g.setColour(wv_theme::text); g.setFont(11.0f);
    g.drawText("Frequency Response (dB)", plot.getX(), getLocalBounds().getY() + 2,
               plot.getWidth(), axis::top - 2, Justification::centredLeft);
    if (magnitudes_db_.empty()) {
        g.setColour(wv_theme::text_dim);
        g.drawText("No spectrum data", plot, Justification::centred); return;
    }
    float db_full_min = -80, db_full_max = 0;
    float f_full_min = 20, f_full_max = float(sample_rate_) * 0.5f;
    float log_full_min = std::log10(f_full_min), log_full_max = std::log10(f_full_max);

    // Apply zoom to freq and dB ranges
    float vis_log_min = log_full_min + float(zoom_.x0) * (log_full_max - log_full_min);
    float vis_log_max = log_full_min + float(zoom_.x1) * (log_full_max - log_full_min);
    float vis_f_min = std::pow(10.0f, vis_log_min);
    float vis_f_max = std::pow(10.0f, vis_log_max);
    float db_range = db_full_max - db_full_min;
    float vis_db_min = db_full_min + float(zoom_.y0) * db_range;
    float vis_db_max = db_full_min + float(zoom_.y1) * db_range;

    drawDbAxis(g, plot, vis_db_min, vis_db_max);
    drawFreqAxisLog(g, plot, vis_f_min, vis_f_max);
    g.setColour(wv_theme::grid); g.drawRect(plot);

    g.setColour(wv_theme::orange);
    g.saveState();
    g.reduceClipRegion(plot);
    Path path; bool started = false;
    for (size_t i = 1; i < magnitudes_db_.size(); ++i) {
        if (freq_axis_[i] < vis_f_min * 0.5f || freq_axis_[i] > vis_f_max * 2.0f) continue;
        float x = plot.getX() + (std::log10(freq_axis_[i]) - vis_log_min) /
                  (vis_log_max - vis_log_min) * plot.getWidth();
        float db = magnitudes_db_[i];
        float y = plot.getY() + (1.0f - (db - vis_db_min) / (vis_db_max - vis_db_min))
                  * plot.getHeight();
        if (!started) { path.startNewSubPath(x, y); started = true; }
        else path.lineTo(x, y);
    }
    g.strokePath(path, PathStrokeType(1.5f));
    g.restoreState();
}

void SpectrumDisplay::mouseWheelMove(const MouseEvent& e, const MouseWheelDetails& w) {
    handlePlotWheel(*this, zoom_, e, w);
}
void SpectrumDisplay::mouseDown(const MouseEvent& e) {
    handlePlotDragStart(drag_start_, e);
}
void SpectrumDisplay::mouseDrag(const MouseEvent& e) {
    handlePlotDrag(*this, zoom_, drag_start_, e);
}
void SpectrumDisplay::mouseDoubleClick(const MouseEvent&) {
    handlePlotDoubleClick(*this, zoom_);
}

////////////////////////////////////////////////////////////////////////////////
// SpectrogramDisplay (dynamic freq cap)
////////////////////////////////////////////////////////////////////////////////

SpectrogramDisplay::SpectrogramDisplay() {
    log_lin_btn_.setButtonText("Log");
    log_lin_btn_.setColour(TextButton::buttonColourId, wv_theme::bg_panel);
    log_lin_btn_.setColour(TextButton::textColourOffId, wv_theme::cyan);
    log_lin_btn_.addListener(this);
    addAndMakeVisible(log_lin_btn_);
}

void SpectrogramDisplay::resized() {
    // Place toggle button in top-right corner
    log_lin_btn_.setBounds(getWidth() - 42, 2, 38, 16);
}

void SpectrogramDisplay::buttonClicked(Button* b) {
    if (b == &log_lin_btn_) {
        setLogFreq(!log_freq_);
        log_lin_btn_.setButtonText(log_freq_ ? "Log" : "Lin");
    }
}

void SpectrogramDisplay::setData(const std::vector<float>& data, double sr) {
    raw_data_ = data;
    raw_sr_ = sr;
    duration_ = data.empty() ? 0.0 : data.size() / sr;
    db_floor_ = -80.0f;
    db_max_ = 0.0f;
    rebuildImage();
}

void SpectrogramDisplay::rebuildImage() {
    auto result = compute_spectrogram(raw_data_, raw_sr_, db_floor_, log_freq_);
    image_ = result.image;
    min_freq_ = result.min_freq;
    max_freq_ = result.max_freq;
    repaint();
}

void SpectrogramDisplay::setLogFreq(bool log) {
    if (log_freq_ == log) return;
    log_freq_ = log;
    log_lin_btn_.setButtonText(log_freq_ ? "Log" : "Lin");
    if (!raw_data_.empty()) rebuildImage();
}

void SpectrogramDisplay::paint(Graphics& g) {
    g.fillAll(wv_theme::bg_plot);
    constexpr int cbar_w = 24;  // colorbar width
    constexpr int cbar_gap = 4;
    auto bounds = getLocalBounds();
    // Reserve space for colorbar on the right
    auto plot = juce::Rectangle<int>(
            bounds.getX() + axis::left, bounds.getY() + axis::top,
            bounds.getWidth() - axis::left - axis::right - cbar_w - cbar_gap,
            bounds.getHeight() - axis::top - axis::bottom);

    g.setColour(wv_theme::text); g.setFont(11.0f);
    String title = log_freq_ ? "Spectrogram (log)" : "Spectrogram (linear)";
    g.drawText(title, plot.getX(), bounds.getY() + 2,
               plot.getWidth(), axis::top - 2, Justification::centredLeft);
    if (image_.isNull() || image_.getWidth() <= 1) {
        g.setColour(wv_theme::text_dim);
        g.drawText("No spectrogram data", plot, Justification::centred); return;
    }
    // Draw zoomed sub-region of spectrogram image
    int iw = image_.getWidth(), ih = image_.getHeight();
    int sx = int(zoom_.x0 * iw);
    int sy = int(zoom_.y0 * ih);
    int sw = std::max(1, int((zoom_.x1 - zoom_.x0) * iw));
    int sh = std::max(1, int((zoom_.y1 - zoom_.y0) * ih));
    g.drawImage(image_,
                plot.getX(), plot.getY(), plot.getWidth(), plot.getHeight(),
                sx, sy, sw, sh);
    g.setColour(wv_theme::grid); g.drawRect(plot);
    double vis_dur = (zoom_.x1 - zoom_.x0) * duration_;
    drawTimeAxis(g, plot, vis_dur);
    // Frequency axis labels
    if (log_freq_) {
        float log_min = std::log10(float(min_freq_));
        float log_max = std::log10(float(max_freq_));
        double vis_f_min = std::pow(10.0, log_min + zoom_.y0 * (log_max - log_min));
        double vis_f_max = std::pow(10.0, log_min + zoom_.y1 * (log_max - log_min));
        drawFreqAxisLogY(g, plot, vis_f_min, vis_f_max);
    } else {
        double vis_f_min = min_freq_ + zoom_.y0 * (max_freq_ - min_freq_);
        double vis_f_max = min_freq_ + zoom_.y1 * (max_freq_ - min_freq_);
        drawFreqAxisLinY(g, plot, vis_f_min, vis_f_max);
    }

    // ── Colorbar ──
    int cb_x = plot.getRight() + cbar_gap;
    int cb_y = plot.getY();
    int cb_h = plot.getHeight();
    // Draw gradient
    for (int row = 0; row < cb_h; ++row) {
        float t = 1.0f - float(row) / float(cb_h);  // top=1 (hot), bottom=0 (cold)
        g.setColour(inferno(t));
        g.fillRect(cb_x, cb_y + row, cbar_w - 14, 1);
    }
    g.setColour(wv_theme::grid);
    g.drawRect(cb_x, cb_y, cbar_w - 14, cb_h);
    // Labels
    g.setFont(8.0f);
    g.setColour(wv_theme::text_dim);
    g.drawText(String(int(db_max_)),
               cb_x + cbar_w - 14, cb_y - 4, 14, 10,
               Justification::centredLeft);
    g.drawText(String(int(db_floor_)),
               cb_x + cbar_w - 14, cb_y + cb_h - 6, 18, 10,
               Justification::centredLeft);
    float mid_db = (db_floor_ + db_max_) * 0.5f;
    g.drawText(String(int(mid_db)),
               cb_x + cbar_w - 14, cb_y + cb_h / 2 - 4, 18, 10,
               Justification::centredLeft);
    // "dB" label at top
    g.drawText("dB", cb_x, cb_y - 14, cbar_w, 12, Justification::centred);
}

void SpectrogramDisplay::mouseWheelMove(const MouseEvent& e, const MouseWheelDetails& w) {
    handlePlotWheel(*this, zoom_, e, w);
}
void SpectrogramDisplay::mouseDown(const MouseEvent& e) {
    handlePlotDragStart(drag_start_, e);
}
void SpectrogramDisplay::mouseDrag(const MouseEvent& e) {
    handlePlotDrag(*this, zoom_, drag_start_, e);
}
void SpectrogramDisplay::mouseDoubleClick(const MouseEvent&) {
    handlePlotDoubleClick(*this, zoom_);
}

////////////////////////////////////////////////////////////////////////////////
// SchroederDisplay
////////////////////////////////////////////////////////////////////////////////

void SchroederDisplay::setData(const std::vector<float>& ir, double sr) {
    sample_rate_ = sr;
    if (ir.empty()) { edc_db_.clear(); repaint(); return; }
    duration_ = ir.size() / sr;
    std::vector<double> energy(ir.size());
    for (size_t i = 0; i < ir.size(); ++i)
        energy[i] = double(ir[i]) * double(ir[i]);
    std::vector<double> edc(ir.size());
    edc.back() = energy.back();
    for (int i = int(ir.size()) - 2; i >= 0; --i)
        edc[i] = edc[i + 1] + energy[i];
    double peak = edc[0];
    edc_db_.resize(ir.size());
    for (size_t i = 0; i < ir.size(); ++i)
        edc_db_[i] = (edc[i] > 0 && peak > 0) ?
            10.0 * std::log10(edc[i] / peak) : -60.0;
    repaint();
}

void SchroederDisplay::paint(Graphics& g) {
    g.fillAll(wv_theme::bg_plot);
    auto plot = plotArea(getLocalBounds());
    g.setColour(wv_theme::text); g.setFont(11.0f);
    g.drawText("Energy Decay Curve", plot.getX(), getLocalBounds().getY() + 2,
               plot.getWidth(), axis::top - 2, Justification::centredLeft);
    if (edc_db_.empty()) {
        g.setColour(wv_theme::text_dim);
        g.drawText("No data", plot, Justification::centred); return;
    }
    float db_full_min = -60, db_full_max = 0;
    float db_range = db_full_max - db_full_min;
    float vis_db_min = db_full_min + float(zoom_.y0) * db_range;
    float vis_db_max = db_full_min + float(zoom_.y1) * db_range;
    double vis_t0 = zoom_.x0 * duration_;
    double vis_t1 = zoom_.x1 * duration_;
    drawDbAxis(g, plot, vis_db_min, vis_db_max);
    drawTimeAxis(g, plot, vis_t1 - vis_t0);
    g.setColour(wv_theme::grid); g.drawRect(plot);

    // Dashed reference lines at -5 and -25 dB (T20 regression bounds)
    g.saveState();
    g.reduceClipRegion(plot);
    for (float ref : {-5.0f, -25.0f}) {
        float y = plot.getY() + (1.0f - (ref - vis_db_min) / (vis_db_max - vis_db_min))
                  * plot.getHeight();
        g.setColour(wv_theme::emphasis.withAlpha(0.5f));
        float dashes[] = {4.0f, 4.0f};
        g.drawDashedLine(Line<float>(float(plot.getX()), y,
                                     float(plot.getRight()), y),
                         dashes, 2, 1.0f);
    }

    // EDC curve — visible sample range
    size_t N = edc_db_.size();
    size_t s0 = size_t(zoom_.x0 * N);
    size_t s1 = std::min(size_t(zoom_.x1 * N), N);
    if (s0 >= s1) s1 = std::min(s0 + 1, N);
    size_t visN = s1 - s0;

    g.setColour(wv_theme::green);
    Path path;
    int pw = plot.getWidth();
    for (int px = 0; px < pw; ++px) {
        size_t idx = s0 + size_t(double(px) / pw * visN);
        if (idx >= s1) break;
        float db = float(juce::jlimit(double(vis_db_min), double(vis_db_max), edc_db_[idx]));
        float y = plot.getY() + (1.0f - (db - vis_db_min) / (vis_db_max - vis_db_min))
                  * plot.getHeight();
        if (px == 0) path.startNewSubPath(float(plot.getX()), y);
        else path.lineTo(float(plot.getX() + px), y);
    }
    g.strokePath(path, PathStrokeType(1.5f));
    g.restoreState();
}

void SchroederDisplay::mouseWheelMove(const MouseEvent& e, const MouseWheelDetails& w) {
    handlePlotWheel(*this, zoom_, e, w);
}
void SchroederDisplay::mouseDown(const MouseEvent& e) {
    handlePlotDragStart(drag_start_, e);
}
void SchroederDisplay::mouseDrag(const MouseEvent& e) {
    handlePlotDrag(*this, zoom_, drag_start_, e);
}
void SchroederDisplay::mouseDoubleClick(const MouseEvent&) {
    handlePlotDoubleClick(*this, zoom_);
}

////////////////////////////////////////////////////////////////////////////////
// TransportBar
////////////////////////////////////////////////////////////////////////////////

TransportBar::TransportBar(AudioTransportSource& transport)
        : transport_(transport) {
    scrubber_.setSliderStyle(Slider::LinearHorizontal);
    scrubber_.setTextBoxStyle(Slider::NoTextBox, true, 0, 0);
    scrubber_.setRange(0, 1.0, 0.001);
    scrubber_.setColour(Slider::backgroundColourId, wv_theme::bg_panel);
    scrubber_.setColour(Slider::trackColourId, wv_theme::emphasis);
    scrubber_.setColour(Slider::thumbColourId, wv_theme::cyan);
    scrubber_.addListener(this);
    addAndMakeVisible(scrubber_);

    elapsed_.setFont(Font(12.0f));
    elapsed_.setColour(Label::textColourId, wv_theme::text);
    elapsed_.setText("0:00", dontSendNotification);
    addAndMakeVisible(elapsed_);

    total_.setFont(Font(12.0f));
    total_.setColour(Label::textColourId, wv_theme::text_dim);
    total_.setText("0:00", dontSendNotification);
    addAndMakeVisible(total_);

    startTimerHz(20);
}

TransportBar::~TransportBar() { stopTimer(); }

void TransportBar::setTotalLength(double seconds) {
    total_seconds_ = seconds;
    scrubber_.setRange(0, std::max(0.01, seconds), 0.01);
    total_.setText(formatTime(seconds), dontSendNotification);
}

void TransportBar::resized() {
    auto area = getLocalBounds();
    elapsed_.setBounds(area.removeFromLeft(50));
    total_.setBounds(area.removeFromRight(50));
    area.removeFromLeft(4);
    area.removeFromRight(4);
    scrubber_.setBounds(area);
}

void TransportBar::paint(Graphics& g) {
    g.fillAll(wv_theme::bg_dark);
}

void TransportBar::sliderValueChanged(Slider*) {
    if (scrubber_.isMouseButtonDown())
        transport_.setPosition(scrubber_.getValue());
}

void TransportBar::timerCallback() {
    if (!scrubber_.isMouseButtonDown()) {
        double pos = transport_.getCurrentPosition();
        scrubber_.setValue(pos, dontSendNotification);
    }
    elapsed_.setText(formatTime(transport_.getCurrentPosition()),
                     dontSendNotification);
}

////////////////////////////////////////////////////////////////////////////////
// ResultsContent
////////////////////////////////////////////////////////////////////////////////

ResultsContent::ResultsContent() {
    // IR analysis: L channel (always visible)
    addAndMakeVisible(ir_waveform_L_);
    addAndMakeVisible(schroeder_L_);
    addAndMakeVisible(ir_spectrogram_L_);
    addAndMakeVisible(ir_spectrum_L_);

    // IR analysis: R channel (hidden until stereo detected)
    addChildComponent(ir_waveform_R_);
    addChildComponent(schroeder_R_);
    addChildComponent(ir_spectrogram_R_);
    addChildComponent(ir_spectrum_R_);

    // Comparison components (initially hidden)
    addChildComponent(dry_waveform_);
    addChildComponent(dry_spectrogram_);
    addChildComponent(conv_waveform_L_);
    addChildComponent(conv_spectrogram_L_);
    addChildComponent(conv_waveform_R_);
    addChildComponent(conv_spectrogram_R_);

    // Title & metrics
    title_label_.setFont(Font(18.0f, Font::bold));
    title_label_.setColour(Label::textColourId, wv_theme::emphasis);
    title_label_.setText("", dontSendNotification);
    title_label_.setVisible(false);
    addAndMakeVisible(title_label_);

    metrics_label_.setFont(Font(12.0f));
    metrics_label_.setColour(Label::textColourId, wv_theme::cyan);
    addAndMakeVisible(metrics_label_);

    addAndMakeVisible(metrics_table_);

    // Buttons
    styleBtn(auralize_btn_, wv_theme::emphasis, Colours::white);
    styleBtn(play_dry_btn_, wv_theme::bg_panel, wv_theme::green);
    styleBtn(play_conv_btn_, wv_theme::bg_panel, wv_theme::green);
    styleBtn(stop_btn_, wv_theme::bg_panel, wv_theme::text);

    play_dry_btn_.setEnabled(false);
    play_dry_btn_.setAlpha(0.4f);
    play_conv_btn_.setEnabled(false);
    play_conv_btn_.setAlpha(0.4f);

    styleBtn(loudness_mode_btn_, wv_theme::bg_panel, wv_theme::orange);
    loudness_mode_btn_.setEnabled(false);
    loudness_mode_btn_.setAlpha(0.4f);

    lufs_label_.setFont(Font(11.0f));
    lufs_label_.setColour(Label::textColourId, wv_theme::text_dim);
    addAndMakeVisible(lufs_label_);

    styleBtn(save_all_btn_, wv_theme::bg_panel, wv_theme::cyan);

    auralize_btn_.addListener(this);
    play_dry_btn_.addListener(this);
    play_conv_btn_.addListener(this);
    stop_btn_.addListener(this);
    styleBtn(test_signal_btn_, wv_theme::bg_panel, wv_theme::green);

    loudness_mode_btn_.addListener(this);
    save_all_btn_.addListener(this);
    test_signal_btn_.addListener(this);
    addAndMakeVisible(auralize_btn_);
    addAndMakeVisible(play_dry_btn_);
    addAndMakeVisible(play_conv_btn_);
    addAndMakeVisible(stop_btn_);
    addAndMakeVisible(loudness_mode_btn_);
    addAndMakeVisible(save_all_btn_);
    addAndMakeVisible(test_signal_btn_);

    // ── Popout buttons (one per plot, overlaid on top-right corner) ──
    Component* plots[] = {
        &ir_waveform_L_, &schroeder_L_, &ir_spectrogram_L_, &ir_spectrum_L_,
        &ir_waveform_R_, &schroeder_R_, &ir_spectrogram_R_, &ir_spectrum_R_,
        &dry_waveform_, &dry_spectrogram_,
        &conv_waveform_L_, &conv_spectrogram_L_,
        &conv_waveform_R_, &conv_spectrogram_R_,
        &metrics_table_
    };
    for (auto* p : plots) {
        auto* btn = new TextButton("Pop");
        btn->setColour(TextButton::buttonColourId,
                        wv_theme::bg_panel.withAlpha(0.8f));
        btn->setColour(TextButton::textColourOffId, wv_theme::cyan);
        btn->setSize(32, 16);
        btn->addListener(this);
        addAndMakeVisible(btn);
        int idx = popout_btns_.size();
        popout_btns_.add(btn);
        popout_entries_.push_back({p, idx});
    }

    // Playback engine
    source_player_.setSource(&transport_);
    device_manager_.addAudioCallback(&source_player_);

    // Transport bar
    transport_bar_ = std::make_unique<TransportBar>(transport_);
    addAndMakeVisible(transport_bar_.get());
}

ResultsContent::~ResultsContent() {
    transport_.stop();
    transport_.setSource(nullptr);
    source_player_.setSource(nullptr);
    device_manager_.removeAudioCallback(&source_player_);
}

void ResultsContent::loadFiles(const std::vector<std::string>& paths,
                               double sampleRate) {
    sample_rate_ = sampleRate;
    ir_channels_.clear();

    DefaultAudioFormatManager fmt_mgr;
    for (const auto& path : paths) {
        File f(path);
        if (!f.existsAsFile()) continue;
        std::unique_ptr<AudioFormatReader> reader(fmt_mgr.createReaderFor(f));
        if (!reader) continue;
        int numSamples = int(reader->lengthInSamples);
        AudioBuffer<float> buf(1, numSamples);
        reader->read(&buf, 0, numSamples, 0, true, false);
        ChannelData cd;
        cd.file_name = f.getFileNameWithoutExtension().toStdString();
        cd.samples.resize(numSamples);
        std::memcpy(cd.samples.data(), buf.getReadPointer(0),
                    numSamples * sizeof(float));
        sample_rate_ = reader->sampleRate;
        ir_channels_.push_back(std::move(cd));
    }

    is_stereo_ = (ir_channels_.size() == 2);

    // ── Left (or mono) channel ──
    if (!ir_channels_.empty()) {
        auto& irL = ir_channels_[0].samples;
        metrics_ = computeMetrics(irL, sample_rate_);
        metrics_table_.setMetrics(metrics_);

        String mstr;
        mstr << "RT60: " << String(metrics_.rt60, 2) << "s";
        mstr << "   T30: " << String(metrics_.t30, 2) << "s";
        mstr << "   EDT: " << String(metrics_.edt, 2) << "s";
        mstr << "   Ts: " << String(metrics_.ts, 0) << "ms";
        mstr << "   C80: " << String(metrics_.c80, 1) << "dB";
        mstr << "   BR: " << String(metrics_.br, 2);
        if (is_stereo_) mstr << "   [Binaural L+R]";
        metrics_label_.setText(mstr, dontSendNotification);

        String lLabel = is_stereo_ ? ("IR Left (" + String(ir_channels_[0].file_name.c_str()) + ")")
                                   : "Impulse Response";
        ir_waveform_L_.setData(irL, sample_rate_, lLabel);
        schroeder_L_.setData(irL, sample_rate_);
        ir_spectrogram_L_.setData(irL, sample_rate_);
        ir_spectrum_L_.setData(irL, sample_rate_);
    }

    // ── Right channel (stereo only) ──
    if (is_stereo_) {
        auto& irR = ir_channels_[1].samples;
        String rLabel = "IR Right (" + String(ir_channels_[1].file_name.c_str()) + ")";
        ir_waveform_R_.setData(irR, sample_rate_, rLabel);
        schroeder_R_.setData(irR, sample_rate_);
        ir_spectrogram_R_.setData(irR, sample_rate_);
        ir_spectrum_R_.setData(irR, sample_rate_);

        ir_waveform_R_.setVisible(true);
        schroeder_R_.setVisible(true);
        ir_spectrogram_R_.setVisible(true);
        ir_spectrum_R_.setVisible(true);
    } else {
        ir_waveform_R_.setVisible(false);
        schroeder_R_.setVisible(false);
        ir_spectrogram_R_.setVisible(false);
        ir_spectrum_R_.setVisible(false);
    }

    resized();
}

void ResultsContent::startPlayback(const AudioBuffer<float>& buffer, double sr) {
    transport_.stop();
    transport_.setSource(nullptr);
    buffer_source_.reset();
    buffer_source_ = std::make_unique<BufferAudioSource>(buffer, sr);
    transport_.setSource(buffer_source_.get(), 0, nullptr, sr,
                         buffer.getNumChannels());
    transport_.setPosition(0.0);
    transport_bar_->setTotalLength(buffer.getNumSamples() / sr);
    transport_.start();
}

void ResultsContent::loadDryAndConvolve() {
    if (ir_channels_.empty()) return;

    // Determine default folder
    File defaultDir = File::getSpecialLocation(File::currentExecutableFile)
                          .getParentDirectory();
    File testModels = defaultDir.getParentDirectory()
                          .getChildFile("demo").getChildFile("assets");
    if (!testModels.isDirectory()) testModels = File();

    FileChooser chooser("Select dry audio file...", testModels,
                        "*.wav;*.aif;*.aiff;*.flac;*.mp3");
    if (!chooser.browseForFileToOpen()) return;

    convolveWithFile(chooser.getResult());
}

void ResultsContent::convolveWithFile(const File& file) {
    if (ir_channels_.empty() || !file.existsAsFile()) return;

    DefaultAudioFormatManager fmt_mgr;
    std::unique_ptr<AudioFormatReader> reader(fmt_mgr.createReaderFor(file));
    if (!reader) {
        AlertWindow::showMessageBoxAsync(AlertWindow::WarningIcon,
                                         "Error", "Could not read audio file.");
        return;
    }

    // Read entire file (up to 5 minutes at native rate), preserving all channels.
    const double dry_native_rate = reader->sampleRate;
    dry_rate_ = dry_native_rate;
    const int srcChannels = int(reader->numChannels);
    int maxSamples = std::min(int(reader->lengthInSamples),
                               int(dry_native_rate * 300));

    // Read all channels from the file.
    AudioBuffer<float> tmp(srcChannels, maxSamples);
    reader->read(&tmp, 0, maxSamples, 0, true, srcChannels > 1);

    // Store left channel for convolution (mono IR convolution uses left).
    dry_samples_.resize(maxSamples);
    std::memcpy(dry_samples_.data(), tmp.getReadPointer(0), maxSamples * sizeof(float));

    // Build a STEREO playback buffer so audio plays in both ears.
    // If source is mono, duplicate to both channels.
    const int playCh = std::max(srcChannels, 2);
    dry_buffer_.setSize(playCh, maxSamples);
    dry_buffer_.copyFrom(0, 0, tmp.getReadPointer(0), maxSamples);
    if (srcChannels >= 2) {
        dry_buffer_.copyFrom(1, 0, tmp.getReadPointer(1), maxSamples);
    } else {
        // Mono source → duplicate left to right for balanced playback.
        dry_buffer_.copyFrom(1, 0, tmp.getReadPointer(0), maxSamples);
    }

    // Resample each IR channel to the dry audio's native rate so that
    // convolution happens at the dry rate and no quality is lost.
    // Uses JUCE's LagrangeInterpolator (4-point) for high-quality resampling.
    const double conv_rate = dry_native_rate;  // convolve & play at dry rate
    std::vector<std::vector<float>> irs_at_dry_rate;
    if (std::abs(sample_rate_ - conv_rate) < 1.0) {
        // Rates match — no resampling needed
        for (auto& ch : ir_channels_)
            irs_at_dry_rate.push_back(ch.samples);
    } else {
        const double speed_ratio = sample_rate_ / conv_rate;
        for (auto& ch : ir_channels_) {
            int out_len = int(ch.samples.size() / speed_ratio) + 16;
            std::vector<float> resampled(out_len);
            juce::LagrangeInterpolator interp;
            int produced = interp.process(speed_ratio,
                                          ch.samples.data(),
                                          resampled.data(),
                                          out_len);
            resampled.resize(produced);
            irs_at_dry_rate.push_back(std::move(resampled));
        }
    }

    // FFT-convolve dry audio with each resampled IR channel (all at dry rate)
    std::vector<std::vector<float>> conv_per_ch;
    for (auto& ir : irs_at_dry_rate)
        conv_per_ch.push_back(fft_convolve(dry_samples_, ir));

    conv_samples_L_ = conv_per_ch.empty() ? std::vector<float>() : conv_per_ch[0];
    conv_samples_R_ = (conv_per_ch.size() >= 2) ? conv_per_ch[1] : std::vector<float>();

    // Build convolved playback buffer — always at least stereo so both ears play.
    int n_ir_ch = int(conv_per_ch.size());
    int conv_len = conv_per_ch.empty() ? 0 : int(conv_per_ch[0].size());
    int n_play_ch = std::max(n_ir_ch, 2);
    conv_buffer_.setSize(n_play_ch, conv_len);
    if (n_ir_ch >= 2) {
        // Binaural: L and R from separate IR channels
        for (int c = 0; c < n_ir_ch; ++c)
            conv_buffer_.copyFrom(c, 0, conv_per_ch[c].data(), conv_len);
    } else if (n_ir_ch == 1) {
        // Mono IR convolution → duplicate to both channels
        conv_buffer_.copyFrom(0, 0, conv_per_ch[0].data(), conv_len);
        conv_buffer_.copyFrom(1, 0, conv_per_ch[0].data(), conv_len);
    }

    has_dry_ = true;

    // ── LUFS measurement ──
    dry_lufs_ = measure_lufs(dry_samples_, dry_rate_);

    // For convolved, measure the loudest channel
    double conv_lufs_L = measure_lufs(conv_samples_L_, dry_rate_);
    double conv_lufs_R = conv_samples_R_.empty()
            ? -100.0 : measure_lufs(conv_samples_R_, dry_rate_);
    conv_lufs_ = std::max(conv_lufs_L, conv_lufs_R);

    fprintf(stderr, "[results] LUFS: dry=%.1f conv=%.1f room_gain=%.1f dB\n",
            dry_lufs_, conv_lufs_, conv_lufs_ - dry_lufs_);
    fflush(stderr);

    // Apply loudness matching if enabled
    if (match_dry_loudness_ && dry_lufs_ > -90.0 && conv_lufs_ > -90.0) {
        float gain = static_cast<float>(
                std::pow(10.0, (dry_lufs_ - conv_lufs_) / 20.0));
        for (auto& s : conv_samples_L_) s *= gain;
        for (auto& s : conv_samples_R_) s *= gain;
        // Update playback buffer too
        for (int c = 0; c < conv_buffer_.getNumChannels(); ++c)
            conv_buffer_.applyGain(c, 0, conv_buffer_.getNumSamples(), gain);
        conv_lufs_ = dry_lufs_;  // they now match
        fprintf(stderr, "[results] matched dry loudness: gain=%.3f (%.1f dB)\n",
                gain, 20.0 * std::log10(gain));
        fflush(stderr);
    }

    // Update LUFS display
    String lufs_text;
    lufs_text << "Dry: " << String(dry_lufs_, 1) << " LUFS";
    if (conv_lufs_ > -90.0) {
        lufs_text << "    Conv: " << String(conv_lufs_, 1) << " LUFS";
        lufs_text << "    Room gain: " << String(conv_lufs_ - dry_lufs_, 1) << " dB";
    }
    lufs_label_.setText(lufs_text, dontSendNotification);

    loudness_mode_btn_.setEnabled(true);
    loudness_mode_btn_.setAlpha(1.0f);

    // Populate comparison displays (all at dry audio's native rate now)
    dry_waveform_.setData(dry_samples_, dry_rate_, "Dry Audio");
    dry_spectrogram_.setData(dry_samples_, dry_rate_);

    conv_waveform_L_.setData(conv_samples_L_, dry_rate_,
                             is_stereo_ ? "Convolved (Left Ear)" : "Convolved");
    conv_spectrogram_L_.setData(conv_samples_L_, dry_rate_);

    dry_waveform_.setVisible(true);
    dry_spectrogram_.setVisible(true);
    conv_waveform_L_.setVisible(true);
    conv_spectrogram_L_.setVisible(true);

    if (is_stereo_ && !conv_samples_R_.empty()) {
        conv_waveform_R_.setData(conv_samples_R_, dry_rate_, "Convolved (Right Ear)");
        conv_spectrogram_R_.setData(conv_samples_R_, dry_rate_);
        conv_waveform_R_.setVisible(true);
        conv_spectrogram_R_.setVisible(true);
    } else {
        conv_waveform_R_.setVisible(false);
        conv_spectrogram_R_.setVisible(false);
    }

    play_dry_btn_.setEnabled(true);
    play_dry_btn_.setAlpha(1.0f);
    play_conv_btn_.setEnabled(true);
    play_conv_btn_.setAlpha(1.0f);

    resized();
    repaint();
}

void ResultsContent::buttonClicked(Button* b) {
    if (b == &auralize_btn_) {
        loadDryAndConvolve();
    } else if (b == &play_dry_btn_ && dry_buffer_.getNumSamples() > 0) {
        startPlayback(dry_buffer_, dry_rate_);
    } else if (b == &play_conv_btn_ && conv_buffer_.getNumSamples() > 0) {
        startPlayback(conv_buffer_, dry_rate_);
    } else if (b == &stop_btn_) {
        transport_.stop();
    } else if (b == &save_all_btn_) {
        saveAllToFolder();
    } else if (b == &test_signal_btn_) {
        // Find test_signals folder relative to executable
        File exeDir = File::getSpecialLocation(File::currentExecutableFile)
                          .getParentDirectory();
        File tsDir = exeDir.getChildFile("test_signals");

        PopupMenu menu;
        menu.addItem(1, "Linear Sweep (20 Hz - 20 kHz)");
        menu.addItem(2, "Log Sweep (20 Hz - 20 kHz)");
        int choice = menu.show();

        File target;
        if (choice == 1)
            target = tsDir.getChildFile("linear_sweep.wav");
        else if (choice == 2)
            target = tsDir.getChildFile("log_sweep.wav");

        if (target.existsAsFile()) {
            convolveWithFile(target);
        } else if (choice > 0) {
            AlertWindow::showMessageBoxAsync(
                    AlertWindow::WarningIcon, "Not Found",
                    "Test signal not found at:\n" + target.getFullPathName()
                    + "\n\nPlace linear_sweep.wav / log_sweep.wav in test_signals/ next to wayverb.exe");
        }
    } else if (b == &loudness_mode_btn_) {
        match_dry_loudness_ = !match_dry_loudness_;
        loudness_mode_btn_.setButtonText(
                match_dry_loudness_ ? "Match Dry Loudness" : "Physical Loudness");

        // Apply or remove loudness matching without re-opening file chooser
        if (has_dry_ && dry_lufs_ > -90.0 && conv_lufs_ > -90.0) {
            // Re-convolve from stored IR data (no file chooser)
            std::vector<std::vector<float>> conv_per_ch;
            for (auto& ch : ir_channels_)
                conv_per_ch.push_back(fft_convolve(dry_samples_, ch.samples));

            conv_samples_L_ = conv_per_ch.empty()
                    ? std::vector<float>() : conv_per_ch[0];
            conv_samples_R_ = (conv_per_ch.size() >= 2)
                    ? conv_per_ch[1] : std::vector<float>();

            // Rebuild playback buffer
            int n_ch = std::max(int(conv_per_ch.size()), 2);
            int conv_len = conv_samples_L_.empty() ? 0 : int(conv_samples_L_.size());
            conv_buffer_.setSize(n_ch, conv_len);
            if (conv_per_ch.size() >= 2) {
                for (int c = 0; c < int(conv_per_ch.size()); ++c)
                    conv_buffer_.copyFrom(c, 0, conv_per_ch[c].data(), conv_len);
            } else if (!conv_per_ch.empty()) {
                conv_buffer_.copyFrom(0, 0, conv_per_ch[0].data(), conv_len);
                conv_buffer_.copyFrom(1, 0, conv_per_ch[0].data(), conv_len);
            }

            // Measure fresh LUFS
            conv_lufs_ = measure_lufs(conv_samples_L_, dry_rate_);

            if (match_dry_loudness_) {
                float gain = static_cast<float>(
                        std::pow(10.0, (dry_lufs_ - conv_lufs_) / 20.0));
                for (auto& s : conv_samples_L_) s *= gain;
                for (auto& s : conv_samples_R_) s *= gain;
                for (int c = 0; c < conv_buffer_.getNumChannels(); ++c)
                    conv_buffer_.applyGain(c, 0, conv_buffer_.getNumSamples(), gain);
                conv_lufs_ = dry_lufs_;
            }

            // Update displays
            String lufs_text;
            lufs_text << "Dry: " << String(dry_lufs_, 1) << " LUFS";
            lufs_text << "    Conv: " << String(conv_lufs_, 1) << " LUFS";
            lufs_text << "    Room gain: " << String(conv_lufs_ - dry_lufs_, 1) << " dB";
            lufs_label_.setText(lufs_text, dontSendNotification);

            conv_waveform_L_.setData(conv_samples_L_, dry_rate_,
                    is_stereo_ ? "Convolved (Left Ear)" : "Convolved");
            conv_spectrogram_L_.setData(conv_samples_L_, dry_rate_);
            if (is_stereo_ && !conv_samples_R_.empty()) {
                conv_waveform_R_.setData(conv_samples_R_, dry_rate_, "Convolved (Right Ear)");
                conv_spectrogram_R_.setData(conv_samples_R_, dry_rate_);
            }
            repaint();
        }
    } else {
        // Check popout buttons
        for (auto& pe : popout_entries_) {
            if (b == popout_btns_[pe.btn_idx]) {
                openPopout(pe.plot);
                return;
            }
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// Popout plot window
// ═════════════════════════════════════════════════════════════════════════════

void ResultsContent::openPopout(Component* source) {
    String title;
    Component* newPlot = nullptr;

    // Helpers to create each plot type with data
    auto makeWave = [](const std::vector<float>& d, double sr,
                       const String& lbl) -> Component* {
        auto* p = new WaveformDisplay();
        p->setData(d, sr, lbl);
        return p;
    };
    auto makeEDC = [](const std::vector<float>& d, double sr) -> Component* {
        auto* p = new SchroederDisplay();
        p->setData(d, sr);
        return p;
    };
    auto makeSpectro = [](const std::vector<float>& d, double sr) -> Component* {
        auto* p = new SpectrogramDisplay();
        p->setData(d, sr);
        return p;
    };
    auto makeSpectrum = [](const std::vector<float>& d, double sr) -> Component* {
        auto* p = new SpectrumDisplay();
        p->setData(d, sr);
        return p;
    };

    // IR Left / Mono
    if (source == &ir_waveform_L_ && !ir_channels_.empty()) {
        title = is_stereo_ ? "IR Waveform (Left)" : "IR Waveform";
        newPlot = makeWave(ir_channels_[0].samples, sample_rate_, title);
    } else if (source == &schroeder_L_ && !ir_channels_.empty()) {
        title = is_stereo_ ? "Energy Decay (Left)" : "Energy Decay";
        newPlot = makeEDC(ir_channels_[0].samples, sample_rate_);
    } else if (source == &ir_spectrogram_L_ && !ir_channels_.empty()) {
        title = is_stereo_ ? "IR Spectrogram (Left)" : "IR Spectrogram";
        newPlot = makeSpectro(ir_channels_[0].samples, sample_rate_);
    } else if (source == &ir_spectrum_L_ && !ir_channels_.empty()) {
        title = is_stereo_ ? "Frequency Response (Left)" : "Frequency Response";
        newPlot = makeSpectrum(ir_channels_[0].samples, sample_rate_);
    }
    // IR Right
    else if (source == &ir_waveform_R_ && ir_channels_.size() >= 2) {
        title = "IR Waveform (Right)";
        newPlot = makeWave(ir_channels_[1].samples, sample_rate_, title);
    } else if (source == &schroeder_R_ && ir_channels_.size() >= 2) {
        title = "Energy Decay (Right)";
        newPlot = makeEDC(ir_channels_[1].samples, sample_rate_);
    } else if (source == &ir_spectrogram_R_ && ir_channels_.size() >= 2) {
        title = "IR Spectrogram (Right)";
        newPlot = makeSpectro(ir_channels_[1].samples, sample_rate_);
    } else if (source == &ir_spectrum_R_ && ir_channels_.size() >= 2) {
        title = "Frequency Response (Right)";
        newPlot = makeSpectrum(ir_channels_[1].samples, sample_rate_);
    }
    // Dry
    else if (source == &dry_waveform_ && has_dry_) {
        title = "Dry Audio";
        newPlot = makeWave(dry_samples_, dry_rate_, title);
    } else if (source == &dry_spectrogram_ && has_dry_) {
        title = "Dry Spectrogram";
        newPlot = makeSpectro(dry_samples_, dry_rate_);
    }
    // Convolved L
    else if (source == &conv_waveform_L_ && has_dry_) {
        title = is_stereo_ ? "Convolved (Left)" : "Convolved";
        newPlot = makeWave(conv_samples_L_, dry_rate_, title);
    } else if (source == &conv_spectrogram_L_ && has_dry_) {
        title = is_stereo_ ? "Convolved Spectrogram (Left)" : "Convolved Spectrogram";
        newPlot = makeSpectro(conv_samples_L_, dry_rate_);
    }
    // Convolved R
    else if (source == &conv_waveform_R_ && has_dry_ && !conv_samples_R_.empty()) {
        title = "Convolved (Right)";
        newPlot = makeWave(conv_samples_R_, dry_rate_, title);
    } else if (source == &conv_spectrogram_R_ && has_dry_ && !conv_samples_R_.empty()) {
        title = "Convolved Spectrogram (Right)";
        newPlot = makeSpectro(conv_samples_R_, dry_rate_);
    }
    // Metrics table
    else if (source == &metrics_table_) {
        title = "ISO 3382 Metrics";
        auto* p = new MetricsTableDisplay();
        p->setMetrics(metrics_);
        newPlot = p;
    }

    if (!newPlot) return;

    auto* win = new PopoutWindow(title, wv_theme::bg_dark,
                                 DocumentWindow::allButtons);
    win->setContentOwned(newPlot, false);
    newPlot->setSize(700, 400);
    win->centreWithSize(700, 400);
    win->setResizable(true, true);
    win->setResizeLimits(300, 150, 4000, 4000);
    win->setUsingNativeTitleBar(true);
    win->setVisible(true);

    // Wayverb icon (must be after setVisible so the native peer exists)
    if (auto* peer = win->getPeer()) {
        auto icon = ImageCache::getFromMemory(BinaryData::wayverb_png,
                                              BinaryData::wayverb_pngSize);
        if (icon.isValid())
            peer->setIcon(icon);

#ifdef _WIN32
        // Purple title bar via DWM
        HWND hwnd = (HWND)peer->getNativeHandle();
        constexpr DWORD DWMWA_CAPTION_COLOR_VAL = 35;
        COLORREF purple = RGB(0x8a, 0x2b, 0xe2);
        DwmSetWindowAttribute(hwnd, DWMWA_CAPTION_COLOR_VAL,
                              &purple, sizeof(purple));
#endif
    }
    win->toFront(true);

    popout_windows_.add(win);
}

void ResultsContent::saveAllToFolder() {
    FileChooser chooser("Choose export folder...",
                        File::getSpecialLocation(File::userDesktopDirectory));
    if (!chooser.browseForDirectory()) return;

    File folder = chooser.getResult();

    // ── Helper: save a component as 2x PNG ──
    auto savePNG = [&](Component& comp, const String& name) {
        if (!comp.isVisible() || comp.getWidth() <= 0) return;
        auto img = comp.createComponentSnapshot(comp.getLocalBounds(), true, 2.0f);
        File f = folder.getChildFile(name + ".png");
        FileOutputStream fos(f);
        if (fos.openedOk()) {
            PNGImageFormat png;
            png.writeImageToStream(img, fos);
        }
    };

    // ── Helper: save float samples as 24-bit WAV ──
    auto saveWAV = [&](const std::vector<float>& samples, double sr,
                       const String& name) {
        if (samples.empty()) return;
        File f = folder.getChildFile(name + ".wav");
        f.deleteFile();
        auto* fos = new FileOutputStream(f);
        if (!fos->openedOk()) { delete fos; return; }
        WavAudioFormat wav;
        std::unique_ptr<AudioFormatWriter> writer(
                wav.createWriterFor(fos, sr, 1, 24, {}, 0));
        if (writer) {
            AudioBuffer<float> buf(1, int(samples.size()));
            buf.copyFrom(0, 0, samples.data(), int(samples.size()));
            writer->writeFromAudioSampleBuffer(buf, 0, buf.getNumSamples());
        }
    };

    // ── Helper: save AudioBuffer as WAV ──
    auto saveBufferWAV = [&](const AudioBuffer<float>& buf, double sr,
                             const String& name) {
        if (buf.getNumSamples() <= 0) return;
        File f = folder.getChildFile(name + ".wav");
        f.deleteFile();
        auto* fos = new FileOutputStream(f);
        if (!fos->openedOk()) { delete fos; return; }
        WavAudioFormat wav;
        std::unique_ptr<AudioFormatWriter> writer(
                wav.createWriterFor(fos, sr, buf.getNumChannels(), 24, {}, 0));
        if (writer)
            writer->writeFromAudioSampleBuffer(buf, 0, buf.getNumSamples());
    };

    // ── Save plot PNGs ──
    savePNG(metrics_table_, "metrics_table");
    savePNG(ir_waveform_L_, "ir_waveform_L");
    savePNG(schroeder_L_, "edc_L");
    savePNG(ir_spectrogram_L_, "ir_spectrogram_L");
    savePNG(ir_spectrum_L_, "ir_spectrum_L");

    if (is_stereo_) {
        savePNG(ir_waveform_R_, "ir_waveform_R");
        savePNG(schroeder_R_, "edc_R");
        savePNG(ir_spectrogram_R_, "ir_spectrogram_R");
        savePNG(ir_spectrum_R_, "ir_spectrum_R");
    }

    if (has_dry_) {
        savePNG(dry_waveform_, "dry_waveform");
        savePNG(dry_spectrogram_, "dry_spectrogram");
        savePNG(conv_waveform_L_, "conv_waveform_L");
        savePNG(conv_spectrogram_L_, "conv_spectrogram_L");
        if (is_stereo_) {
            savePNG(conv_waveform_R_, "conv_waveform_R");
            savePNG(conv_spectrogram_R_, "conv_spectrogram_R");
        }
    }

    // ── Save IR WAV files ──
    if (!ir_channels_.empty())
        saveWAV(ir_channels_[0].samples, sample_rate_, "ir_L");
    if (is_stereo_ && ir_channels_.size() >= 2)
        saveWAV(ir_channels_[1].samples, sample_rate_, "ir_R");

    // ── Save dry + convolved WAV ──
    if (has_dry_) {
        saveBufferWAV(dry_buffer_, dry_rate_, "dry_audio");
        saveBufferWAV(conv_buffer_, dry_rate_, "convolved");
    }

    // ── Save ISO 3382 metrics CSV ──
    {
        File f = folder.getChildFile("metrics_iso3382.csv");
        FileOutputStream fos(f);
        if (fos.openedOk()) {
            fos.writeText("Parameter,125Hz,250Hz,500Hz,1kHz,2kHz,4kHz,Broadband\n",
                          false, false);
            auto row = [&](const char* name, const double* bands, double broad,
                           int dec) {
                String line;
                line << name;
                for (int b = 0; b < kNumBands; ++b)
                    line << "," << String(bands[b], dec);
                line << "," << String(broad, dec) << "\n";
                fos.writeText(line, false, false);
            };
            row("RT60 (s)", metrics_.band_rt60, metrics_.rt60, 3);
            row("T30 (s)", metrics_.band_t30, metrics_.t30, 3);
            row("EDT (s)", metrics_.band_edt, metrics_.edt, 3);
            row("C80 (dB)", metrics_.band_c80, metrics_.c80, 1);
            row("C50 (dB)", metrics_.band_c50, metrics_.c50, 1);
            row("D50 (%)", metrics_.band_d50, metrics_.d50, 1);
            row("Ts (ms)", metrics_.band_ts, metrics_.ts, 0);
            fos.writeText("Bass Ratio,,,,,,," + String(metrics_.br, 3) + "\n",
                          false, false);
            if (dry_lufs_ > -90.0) {
                fos.writeText("\nDry LUFS," + String(dry_lufs_, 1) + "\n",
                              false, false);
                fos.writeText("Conv LUFS," + String(conv_lufs_, 1) + "\n",
                              false, false);
            }
        }
    }

    AlertWindow::showMessageBoxAsync(
            AlertWindow::InfoIcon, "Export Complete",
            "All results saved to:\n" + folder.getFullPathName());
}

void ResultsContent::paint(Graphics& g) {
    g.fillAll(wv_theme::bg_dark);

    if (!has_dry_) {
        auto area = getLocalBounds();
        int hintY = area.getHeight() - 60;
        g.setColour(wv_theme::text_dim);
        g.setFont(13.0f);
        g.drawText("Click \"Auralize...\" to load dry audio and compare",
                   0, hintY, area.getWidth(), 20, Justification::centred);
    }
}

void ResultsContent::resized() {
    const int W = getWidth();
    const int pad = 10;
    const int usableW = W - 2 * pad;
    const int plotH = 210;
    const int metricsH = 200;
    const int ctrlH = 28;
    const int gap = 6;
    const int colGap = 6;  // gap between L/R columns

    // ── Calculate total content height ──
    int totalH = pad;
    totalH += 22 + gap;       // title row
    totalH += ctrlH + gap;    // buttons
    totalH += 16 + gap;       // LUFS
    totalH += metricsH + gap; // metrics table

    if (is_stereo_) {
        // Stereo: L|R side-by-side → 4 rows (waveform, EDC, spectrogram, spectrum)
        totalH += 4 * (plotH + gap);
    } else {
        // Mono: 4 full-width rows
        totalH += 4 * (plotH + gap);
    }

    if (has_dry_) {
        totalH += plotH + gap;  // dry waveform
        totalH += plotH + gap;  // dry spectrogram
        if (is_stereo_) {
            // Conv L|R side-by-side → 2 rows (waveform, spectrogram)
            totalH += 2 * (plotH + gap);
        } else {
            totalH += 2 * (plotH + gap);  // conv waveform + spectrogram
        }
    }

    totalH += 32 + gap + pad; // transport bar

    // Anti-recursion: adjust height for Viewport scrolling
    if (getHeight() != totalH) {
        setSize(W, totalH);
        return;
    }

    int y = pad;

    // ── Title + metrics summary ──
    title_label_.setBounds(pad, y, 170, 22);
    metrics_label_.setBounds(pad + 178, y, usableW - 178, 22);
    y += 22 + gap;

    // ── Button row ──
    int bx = pad;
    auralize_btn_.setBounds(bx, y, 90, ctrlH); bx += 94;
    test_signal_btn_.setBounds(bx, y, 90, ctrlH); bx += 94;
    play_dry_btn_.setBounds(bx, y, 70, ctrlH); bx += 74;
    play_conv_btn_.setBounds(bx, y, 100, ctrlH); bx += 104;
    stop_btn_.setBounds(bx, y, 44, ctrlH); bx += 48;
    loudness_mode_btn_.setBounds(bx, y, 130, ctrlH); bx += 134;
    save_all_btn_.setBounds(bx, y, 80, ctrlH);
    y += ctrlH + gap;

    // ── LUFS ──
    lufs_label_.setBounds(pad, y, usableW, 16);
    y += 16 + gap;

    // ── ISO 3382 metrics table ──
    metrics_table_.setBounds(pad, y, usableW, metricsH);
    y += metricsH + gap;

    // ── Helpers ──
    // Full-width plot
    auto placeFull = [&](Component& c) {
        c.setBounds(pad, y, usableW, plotH);
        y += plotH + gap;
    };

    // Side-by-side pair (L on left, R on right)
    int halfW = (usableW - colGap) / 2;
    auto placePair = [&](Component& L, Component& R) {
        L.setBounds(pad, y, halfW, plotH);
        R.setBounds(pad + halfW + colGap, y, halfW, plotH);
        y += plotH + gap;
    };

    // ── IR Analysis ──
    schroeder_L_.setVisible(true);
    ir_spectrum_L_.setVisible(true);

    if (is_stereo_) {
        schroeder_R_.setVisible(true);
        ir_spectrum_R_.setVisible(true);

        // L | R side-by-side for each plot type
        placePair(ir_waveform_L_, ir_waveform_R_);
        placePair(schroeder_L_, schroeder_R_);
        placePair(ir_spectrogram_L_, ir_spectrogram_R_);
        placePair(ir_spectrum_L_, ir_spectrum_R_);
    } else {
        // Mono: full-width
        placeFull(ir_waveform_L_);
        placeFull(schroeder_L_);
        placeFull(ir_spectrogram_L_);
        placeFull(ir_spectrum_L_);
    }

    // ── Comparison section (after auralization) ──
    if (has_dry_) {
        // Dry audio is always full-width (single signal)
        placeFull(dry_waveform_);
        placeFull(dry_spectrogram_);

        if (is_stereo_) {
            // Convolved L | R side-by-side
            placePair(conv_waveform_L_, conv_waveform_R_);
            placePair(conv_spectrogram_L_, conv_spectrogram_R_);
        } else {
            placeFull(conv_waveform_L_);
            placeFull(conv_spectrogram_L_);
        }
    }

    // ── Transport bar ──
    transport_bar_->setBounds(pad, y, usableW, 32);
    y += 32 + gap;

    // ── Position popout buttons on each visible plot ──
    for (auto& pe : popout_entries_) {
        auto* btn = popout_btns_[pe.btn_idx];
        if (pe.plot->isVisible() && pe.plot->getWidth() > 0) {
            auto b = pe.plot->getBounds();
            // Top-right corner, inset slightly
            btn->setBounds(b.getRight() - 36, b.getY() + 2, 32, 16);
            btn->setVisible(true);
            btn->toFront(false);
        } else {
            btn->setVisible(false);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// ResultsWindow
////////////////////////////////////////////////////////////////////////////////

ResultsWindow::ResultsWindow(const std::vector<std::string>& paths,
                             double sampleRate,
                             const std::string& title)
        : DocumentWindow(String(title),
                          wv_theme::bg_dark,
                          DocumentWindow::allButtons) {
    // Set up scrollable viewport
    viewport_.setViewedComponent(&content_, false);  // don't take ownership
    viewport_.setScrollBarsShown(true, false);       // vertical only
    viewport_.setScrollBarThickness(10);

    setContentNonOwned(&viewport_, false);
    centreWithSize(1100, 980);
    setResizable(true, true);
    setResizeLimits(800, 600, 10000, 10000);
    setUsingNativeTitleBar(true);
    setVisible(true);
    toFront(true);

    // Wayverb icon
    {
        auto icon = ImageCache::getFromMemory(
                BinaryData::wayverb_png, BinaryData::wayverb_pngSize);
        if (icon.isValid())
            if (auto* peer = getPeer()) peer->setIcon(icon);
    }

#ifdef _WIN32
    // Purple title bar (Windows 11 DWM)
    if (auto* peer = getPeer()) {
        HWND hwnd = static_cast<HWND>(peer->getNativeHandle());
        COLORREF purple = RGB(92, 0, 163);
        constexpr DWORD DWMWA_CAPTION_COLOR_VAL = 35;
        DwmSetWindowAttribute(hwnd, DWMWA_CAPTION_COLOR_VAL, &purple, sizeof(purple));
    }
#endif

    content_.loadFiles(paths, sampleRate);
}

void ResultsWindow::resized() {
    DocumentWindow::resized();
    if (auto* cc = getContentComponent()) {
        auto area = cc->getLocalBounds();
        viewport_.setBounds(area);
        int contentW = area.getWidth() - viewport_.getScrollBarThickness();
        if (contentW > 0 && content_.getWidth() != contentW)
            content_.setSize(contentW, content_.getHeight());
    }
}

void ResultsWindow::closeButtonPressed() {
    setVisible(false);
}
