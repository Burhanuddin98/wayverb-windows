#pragma once

#include "../JuceLibraryCode/JuceHeader.h"
#include "../UtilityComponents/DefaultAudio.h"
#include <vector>
#include <string>

// ── Wayverb theme colours ───────────────────────────────────────────────────
namespace wv_theme {
static const Colour bg_dark{0xff1e1e2f};
static const Colour bg_panel{0xff2a2a40};
static const Colour bg_plot{0xff12121e};
static const Colour emphasis{0xff8a2be2};   // blueviolet
static const Colour cyan{0xff00d4ff};
static const Colour orange{0xffff6600};
static const Colour green{0xff00e676};
static const Colour text{0xffc0c0c0};
static const Colour text_dim{0xff808090};
static const Colour grid{0xff333355};
}  // namespace wv_theme

// ── Axis margin constants ───────────────────────────────────────────────────
namespace axis {
constexpr int left   = 46;
constexpr int bottom = 22;
constexpr int top    = 20;
constexpr int right  = 10;
}  // namespace axis

// ── Acoustic metrics (ISO 3382) ─────────────────────────────────────────────
//  Octave band centre frequencies: 125, 250, 500, 1k, 2k, 4k Hz
constexpr int kNumBands = 6;
constexpr double kBandCentres[kNumBands] = {125, 250, 500, 1000, 2000, 4000};

struct AcousticMetrics {
    // Broadband
    double rt60 = 0;   // Reverberation time (T20 × 3)
    double t30  = 0;   // T30 (-5 to -35 dB, × 2)
    double edt  = 0;   // Early Decay Time
    double c80  = 0;   // Clarity (80 ms)
    double c50  = 0;   // Clarity (50 ms)
    double d50  = 0;   // Definition (%, 50 ms)
    double ts   = 0;   // Centre Time (ms)
    double br   = 0;   // Bass Ratio: avg(RT125,RT250) / avg(RT500,RT1k)

    // Per-octave band (125, 250, 500, 1k, 2k, 4k)
    double band_rt60[kNumBands] = {};
    double band_t30[kNumBands]  = {};
    double band_edt[kNumBands]  = {};
    double band_c80[kNumBands]  = {};
    double band_c50[kNumBands]  = {};
    double band_d50[kNumBands]  = {};
    double band_ts[kNumBands]   = {};
};

// ── BufferAudioSource ───────────────────────────────────────────────────────
class BufferAudioSource : public PositionableAudioSource {
public:
    BufferAudioSource(const AudioBuffer<float>& buf, double sr)
            : buffer_(buf), sample_rate_(sr) {}

    void prepareToPlay(int, double) override { position_ = 0; }
    void releaseResources() override {}

    void getNextAudioBlock(const AudioSourceChannelInfo& info) override {
        // Guard against degenerate buffer
        if (buffer_.getNumChannels() <= 0 || buffer_.getNumSamples() <= 0) {
            info.buffer->clear(info.startSample, info.numSamples);
            return;
        }
        const int remaining = jmax(0, buffer_.getNumSamples() - (int)position_);
        const int toCopy = jmin(info.numSamples, remaining);
        if (toCopy > 0) {
            const int numSrcCh = buffer_.getNumChannels();
            for (int ch = 0; ch < info.buffer->getNumChannels(); ++ch) {
                const int srcCh = ch < numSrcCh ? ch : numSrcCh - 1;
                info.buffer->copyFrom(ch, info.startSample,
                                      buffer_, srcCh, (int)position_, toCopy);
            }
        }
        if (toCopy < info.numSamples)
            info.buffer->clear(info.startSample + toCopy,
                               info.numSamples - toCopy);
        position_ += toCopy;
    }

    int64 getTotalLength() const override { return buffer_.getNumSamples(); }
    int64 getNextReadPosition() const override { return position_; }
    void setNextReadPosition(int64 p) override {
        position_ = (int64)jlimit((int64)0, (int64)buffer_.getNumSamples(), p);
    }
    bool isLooping() const override { return false; }

private:
    AudioBuffer<float> buffer_;
    double sample_rate_;
    int64 position_ = 0;
};

// ── Zoom/pan state for interactive plots ────────────────────────────────────
struct PlotZoom {
    // Visible range in normalized [0,1] coordinates
    double x0 = 0.0, x1 = 1.0;  // horizontal
    double y0 = 0.0, y1 = 1.0;  // vertical

    void reset() { x0 = 0; x1 = 1; y0 = 0; y1 = 1; }

    // Zoom around a normalized cursor position (0-1)
    void zoomX(double centre, double factor) {
        double w = x1 - x0;
        double new_w = juce::jlimit(0.005, 1.0, w * factor);
        double new_x0 = centre - (centre - x0) * (new_w / w);
        x0 = juce::jlimit(0.0, 1.0 - new_w, new_x0);
        x1 = x0 + new_w;
    }
    void zoomY(double centre, double factor) {
        double h = y1 - y0;
        double new_h = juce::jlimit(0.005, 1.0, h * factor);
        double new_y0 = centre - (centre - y0) * (new_h / h);
        y0 = juce::jlimit(0.0, 1.0 - new_h, new_y0);
        y1 = y0 + new_h;
    }
    void panX(double delta) {
        double w = x1 - x0;
        x0 = juce::jlimit(0.0, 1.0 - w, x0 + delta);
        x1 = x0 + w;
    }
    void panY(double delta) {
        double h = y1 - y0;
        y0 = juce::jlimit(0.0, 1.0 - h, y0 + delta);
        y1 = y0 + h;
    }
};

// ── Waveform display ────────────────────────────────────────────────────────
class WaveformDisplay : public Component {
public:
    void setData(const std::vector<float>& data, double sampleRate,
                 const String& label = "");
    void paint(Graphics& g) override;
    void mouseWheelMove(const MouseEvent& e, const MouseWheelDetails& w) override;
    void mouseDown(const MouseEvent& e) override;
    void mouseDrag(const MouseEvent& e) override;
    void mouseDoubleClick(const MouseEvent& e) override;
    PlotZoom zoom_;
private:
    std::vector<float> data_;
    double sample_rate_ = 44100.0;
    String label_;
    Point<float> drag_start_;
};

// ── Frequency response (log-freq, dB, peak-normalized) ─────────────────────
class SpectrumDisplay : public Component {
public:
    void setData(const std::vector<float>& data, double sampleRate);
    void paint(Graphics& g) override;
    void mouseWheelMove(const MouseEvent& e, const MouseWheelDetails& w) override;
    void mouseDown(const MouseEvent& e) override;
    void mouseDrag(const MouseEvent& e) override;
    void mouseDoubleClick(const MouseEvent& e) override;
    PlotZoom zoom_;
private:
    std::vector<float> magnitudes_db_;
    std::vector<float> freq_axis_;
    double sample_rate_ = 44100.0;
    Point<float> drag_start_;
};

// ── Spectrogram (STFT, inferno, dynamic freq cap) ──────────────────────────
class SpectrogramDisplay : public Component,
                           public Button::Listener {
public:
    SpectrogramDisplay();
    void setData(const std::vector<float>& data, double sampleRate);
    void paint(Graphics& g) override;
    void resized() override;
    void mouseWheelMove(const MouseEvent& e, const MouseWheelDetails& w) override;
    void mouseDown(const MouseEvent& e) override;
    void mouseDrag(const MouseEvent& e) override;
    void mouseDoubleClick(const MouseEvent& e) override;
    void buttonClicked(Button* b) override;
    void setLogFreq(bool log);
    bool isLogFreq() const { return log_freq_; }
    PlotZoom zoom_;
private:
    void rebuildImage();
    TextButton log_lin_btn_{"Log"};
    Image image_;
    double duration_ = 0.0;
    double min_freq_ = 20.0;
    double max_freq_ = 22050.0;
    float db_floor_ = -80.0f;
    float db_max_ = 0.0f;
    bool log_freq_ = true;
    std::vector<float> raw_data_;
    double raw_sr_ = 44100.0;
    Point<float> drag_start_;
};

// ── ISO 3382 metrics table ─────────────────────────────────────────────────
class MetricsTableDisplay : public Component {
public:
    void setMetrics(const AcousticMetrics& m);
    void paint(Graphics& g) override;
private:
    AcousticMetrics metrics_;
};

// ── Schroeder energy decay curve ────────────────────────────────────────────
class SchroederDisplay : public Component {
public:
    void setData(const std::vector<float>& ir, double sampleRate);
    void paint(Graphics& g) override;
    void mouseWheelMove(const MouseEvent& e, const MouseWheelDetails& w) override;
    void mouseDown(const MouseEvent& e) override;
    void mouseDrag(const MouseEvent& e) override;
    void mouseDoubleClick(const MouseEvent& e) override;
    PlotZoom zoom_;
private:
    std::vector<double> edc_db_;
    double sample_rate_ = 44100.0;
    double duration_ = 0.0;
    Point<float> drag_start_;
};

// ── Transport bar (scrubber slider + elapsed/total time) ────────────────────
class TransportBar : public Component,
                     public Slider::Listener,
                     public Timer {
public:
    TransportBar(AudioTransportSource& transport);
    ~TransportBar() override;
    void setTotalLength(double seconds);
    void resized() override;
    void paint(Graphics& g) override;
    void sliderValueChanged(Slider* s) override;
    void timerCallback() override;
private:
    AudioTransportSource& transport_;
    Slider scrubber_;
    Label elapsed_;
    Label total_;
    double total_seconds_ = 0.0;
};

// ── Main results content ────────────────────────────────────────────────────
class ResultsContent : public Component,
                       public Button::Listener {
public:
    ResultsContent();
    ~ResultsContent() override;

    void loadFiles(const std::vector<std::string>& paths, double sampleRate);

    void resized() override;
    void paint(Graphics& g) override;
    void buttonClicked(Button* b) override;

private:
    void loadDryAndConvolve();
    void startPlayback(const AudioBuffer<float>& buffer, double sr);

    double sample_rate_ = 44100.0;
    double dry_rate_ = 44100.0;  // native sample rate of loaded dry audio
    bool has_dry_ = false;
    bool is_stereo_ = false;
    AcousticMetrics metrics_;

    struct ChannelData {
        std::string file_name;
        std::vector<float> samples;
    };
    std::vector<ChannelData> ir_channels_;

    // Dry + convolved
    std::vector<float> dry_samples_;
    std::vector<float> conv_samples_L_;
    std::vector<float> conv_samples_R_;
    AudioBuffer<float> dry_buffer_;
    AudioBuffer<float> conv_buffer_;

    // ── ISO 3382 metrics table ──
    MetricsTableDisplay metrics_table_;

    // ── IR analysis: Left channel ──
    WaveformDisplay ir_waveform_L_;
    SchroederDisplay schroeder_L_;
    SpectrogramDisplay ir_spectrogram_L_;
    SpectrumDisplay ir_spectrum_L_;

    // ── IR analysis: Right channel (stereo only) ──
    WaveformDisplay ir_waveform_R_;
    SchroederDisplay schroeder_R_;
    SpectrogramDisplay ir_spectrogram_R_;
    SpectrumDisplay ir_spectrum_R_;

    // ── Comparison section ──
    WaveformDisplay dry_waveform_;
    SpectrogramDisplay dry_spectrogram_;
    WaveformDisplay conv_waveform_L_;
    SpectrogramDisplay conv_spectrogram_L_;
    WaveformDisplay conv_waveform_R_;
    SpectrogramDisplay conv_spectrogram_R_;

    // ── Labels ──
    Label title_label_;
    Label metrics_label_;

    // ── Loudness ──
    double dry_lufs_ = -100.0;
    double conv_lufs_ = -100.0;
    bool match_dry_loudness_ = false;
    Label lufs_label_;

    // ── Buttons ──
    TextButton auralize_btn_{"Auralize..."};
    TextButton play_dry_btn_{"Play Dry"};
    TextButton play_conv_btn_{"Play Convolved"};
    TextButton stop_btn_{"Stop"};
    TextButton loudness_mode_btn_{"Physical Loudness"};
    TextButton save_all_btn_{"Save All..."};
    TextButton test_signal_btn_{"Test Signal"};

    void saveAllToFolder();
    void convolveWithFile(const File& audioFile);
    void openPopout(Component* source);

    // ── Popout plot buttons (overlaid on each plot's corner) ──
    OwnedArray<TextButton> popout_btns_;
    struct PopEntry { Component* plot; int btn_idx; };
    std::vector<PopEntry> popout_entries_;
    OwnedArray<DocumentWindow> popout_windows_;

    // ── Playback ──
    DefaultAudioDeviceManager device_manager_;
    AudioSourcePlayer source_player_;
    AudioTransportSource transport_;
    std::unique_ptr<BufferAudioSource> buffer_source_;
    std::unique_ptr<TransportBar> transport_bar_;
};

// ── Top-level results window ────────────────────────────────────────────────
class ResultsWindow : public DocumentWindow {
public:
    ResultsWindow(const std::vector<std::string>& paths, double sampleRate,
                  const std::string& title = "Results");
    void closeButtonPressed() override;
    void resized() override;
private:
    Viewport viewport_;
    ResultsContent content_;
};
