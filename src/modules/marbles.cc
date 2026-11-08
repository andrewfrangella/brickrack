// BrickRack - GPL-3.0-or-later.
//
// Random Sampler: wraps the Marbles t/X/Y generators (marbles/random/*) and
// drives them the way marbles.cc, cv_reader.cc and ui.cc do on the hardware:
// 32 kHz, 5-sample blocks, the same CV conditioning (CvReaderChannel), the
// same clock-source selection (including the self-patching detector) and the
// same button logic (short/long presses, deja vu lock, scale selection and
// scale recording).
//
// Rate conversion: every input and output of this module is a gate or a
// stepped CV, and the firmware's clock handling counts samples (ramp
// extractor, self-patching detector), so the 32 kHz core is clocked with a
// sample-and-hold converter instead of BlockAdapter's windowed sinc: the sinc
// filter would ring on every step and add latency the self-patching detector
// cannot tolerate. One native sample is taken whenever the 32 kHz clock
// ticks; outputs hold the latest native sample.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>

#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"
#include "marbles/clock_self_patching_detector.h"
#include "marbles/cv_reader_channel.h"
#include "marbles/drivers/adc.h"
#include "marbles/io_buffer.h"
#include "marbles/note_filter.h"
#include "marbles/random/random_generator.h"
#include "marbles/random/random_stream.h"
#include "marbles/random/t_generator.h"
#include "marbles/random/x_y_generator.h"
#include "marbles/scale_recorder.h"
#include "stmlib/dsp/hysteresis_quantizer.h"
#include "stmlib/utils/gate_flags.h"
#include "ui/canvas.h"

namespace br {
namespace {

using marbles::kBlockSize;
using stmlib::GateFlags;

constexpr float kNativeRate = 32000.f;
constexpr int kGateDelay = 2;
constexpr int kNumScales = 6;
constexpr int32_t kLongPressMs = 2000;

// --- Data copied from the firmware (marbles.cc / settings.cc / cv_reader.cc).
// settings.cc and cv_reader.cc cannot be compiled (they include the STM32
// drivers), so their static tables are reproduced here verbatim.

const marbles::Scale kPresetScales[kNumScales] = {
    // C major
    {1.0f, 12, {{0.0000f, 255}, {0.0833f, 16}, {0.1667f, 96}, {0.2500f, 24},
                {0.3333f, 128}, {0.4167f, 64}, {0.5000f, 8}, {0.5833f, 192},
                {0.6667f, 16}, {0.7500f, 96}, {0.8333f, 24}, {0.9167f, 128}}},
    // C minor
    {1.0f, 12, {{0.0000f, 255}, {0.0833f, 16}, {0.1667f, 96}, {0.2500f, 128},
                {0.3333f, 8}, {0.4167f, 64}, {0.5000f, 4}, {0.5833f, 192},
                {0.6667f, 96}, {0.7500f, 16}, {0.8333f, 128}, {0.9167f, 16}}},
    // Pentatonic
    {1.0f, 12, {{0.0000f, 255}, {0.0833f, 4}, {0.1667f, 96}, {0.2500f, 4},
                {0.3333f, 4}, {0.4167f, 140}, {0.5000f, 4}, {0.5833f, 192},
                {0.6667f, 4}, {0.7500f, 96}, {0.8333f, 4}, {0.9167f, 4}}},
    // Pelog
    {1.0f, 7, {{0.0000f, 255}, {0.1275f, 128}, {0.2625f, 32}, {0.4600f, 8},
               {0.5883f, 192}, {0.7067f, 64}, {0.8817f, 16}}},
    // Raag Bhairav That
    {1.0f, 12, {{0.0000f, 255}, {0.0752f, 128}, {0.1699f, 4}, {0.2630f, 4},
                {0.3219f, 128}, {0.4150f, 64}, {0.4918f, 4}, {0.5850f, 192},
                {0.6601f, 64}, {0.7549f, 4}, {0.8479f, 4}, {0.9069f, 64}}},
    // Raag Shri
    {1.0f, 12, {{0.0000f, 255}, {0.0752f, 4}, {0.1699f, 128}, {0.2630f, 64},
                {0.3219f, 4}, {0.4150f, 128}, {0.4918f, 4}, {0.5850f, 192},
                {0.6601f, 4}, {0.7549f, 64}, {0.8479f, 128}, {0.9069f, 4}}},
};

const marbles::Ratio kYDividerRatios[12] = {
    {1, 64}, {1, 48}, {1, 32}, {1, 24}, {1, 16}, {1, 12},
    {1, 8},  {1, 6},  {1, 4},  {1, 3},  {1, 2},  {1, 1},
};

const int kLoopLength[] = {
    1,
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
    3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    5, 5, 5, 5,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    7, 7,
    8, 8, 8, 8, 8, 8, 8, 8, 8,
    10, 10, 10,
    12, 12, 12, 12, 12, 12, 12,
    14, 14,
    16,
};

// cv_lp | pot_scale | pot_offset | pot_lp | min | max | hysteresis
const marbles::CvReaderChannel::Settings kChannelSettings[marbles::ADC_CHANNEL_LAST] = {
    {0.05f, 1.0f, 0.0f, 0.01f, 0.0f, 1.0f, 0.00f},              // DEJA_VU_AMOUNT
    {0.05f, 1.0f, 0.0f, 0.01f, 0.0f, 1.0f, 0.00f},              // DEJA_VU_LENGTH
    {0.2f, 120.0f, -60.0f, 0.01f, -120.0f, 120.0f, 0.001f},     // T_RATE
    {0.05f, 1.05f, -0.025f, 0.01f, 0.0f, 1.0f, 0.00f},          // T_BIAS
    {0.05f, 1.0f, 0.0f, 0.01f, 0.0f, 1.0f, 0.00f},              // T_JITTER
    {0.1f, 1.0f, 0.0f, 0.01f, 0.0f, 1.0f, 0.01f},               // X_SPREAD
    {0.1f, 1.0f, 0.0f, 0.01f, 0.0f, 1.0f, 0.02f},               // X_BIAS
    {0.05f, 1.0f, 0.0f, 0.01f, 0.0f, 1.0f, 0.02f},              // X_STEPS
};

const char* kTModelNames[6] = {"Complementary Bernoulli", "Clusters", "Drums",
                               "Independent Bernoulli", "Divider", "Three states"};
const char* kTModelShort[6] = {"COIN TOSS", "CLUSTERS", "DRUMS", "INDEP. COIN", "DIVIDER",
                               "3 STATES"};
const char* kScaleNames[kNumScales] = {"Major",  "Minor",  "Pentatonic", "Pelog",
                                       "Raag Bhairav That", "Raag Shri"};
const char* kXModeShort[3] = {"IDENTICAL", "BUMP", "TILT"};

enum DejaVuState { DEJA_VU_OFF, DEJA_VU_ON, DEJA_VU_LOCKED };
enum UiMode { UI_MODE_NORMAL, UI_MODE_SELECT_SCALE, UI_MODE_RECORD_SCALE };

// LED colours of the hardware's tricolour LEDs, as (red, green).
struct Rg {
  float r, g;
};
const Rg kGreen = {0.f, 1.f}, kYellow = {1.f, 1.f}, kRed = {1.f, 0.f}, kOff = {0.f, 0.f};
const Rg kPalette[4] = {kGreen, kYellow, kRed, kOff};

class RandomSampler : public Module {
 public:
  enum ParamIds {
    DEJA_VU_PARAM,
    LENGTH_PARAM,
    T_RATE_PARAM,
    T_BIAS_PARAM,
    T_JITTER_PARAM,
    X_SPREAD_PARAM,
    X_BIAS_PARAM,
    X_STEPS_PARAM,
    T_DEJA_VU_BUTTON,
    X_DEJA_VU_BUTTON,
    T_MODE_BUTTON,
    T_RANGE_BUTTON,
    X_MODE_BUTTON,
    X_RANGE_BUTTON,
    X_EXT_BUTTON,
    // Hidden: the firmware's persistent State (settings.h), editable from the menu.
    T_DEJA_VU_STATE,
    X_DEJA_VU_STATE,
    T_MODEL_STATE,
    T_RANGE_STATE,
    X_MODE_STATE,
    X_RANGE_STATE,
    X_EXT_STATE,  // x_register_mode
    SCALE_STATE,
    T_PW_MEAN_PARAM,
    T_PW_STD_PARAM,
    Y_SPREAD_PARAM,
    Y_BIAS_PARAM,
    Y_STEPS_PARAM,
    Y_DIVIDER_PARAM,
    Y_RANGE_PARAM,
    EXPLICIT_RESET_PARAM,
    NUM_PARAMS
  };
  enum InputIds {
    T_BIAS_INPUT,
    X_BIAS_INPUT,
    T_CLOCK_INPUT,
    T_RATE_INPUT,
    T_JITTER_INPUT,
    DEJA_VU_INPUT,
    X_STEPS_INPUT,
    X_SPREAD_INPUT,
    X_CLOCK_INPUT,
    NUM_INPUTS
  };
  enum OutputIds { T1_OUTPUT, T2_OUTPUT, T3_OUTPUT, Y_OUTPUT, X1_OUTPUT, X2_OUTPUT, X3_OUTPUT,
                   NUM_OUTPUTS };
  enum LightIds {
    T_MODEL_LIGHT = 0,   // R, G, B
    T_RANGE_LIGHT = 3,
    X_MODE_LIGHT = 6,
    X_RANGE_LIGHT = 9,
    T_DEJA_VU_LIGHT = 12,
    X_DEJA_VU_LIGHT,
    X_EXT_LIGHT,
    T1_LIGHT,
    T2_LIGHT,
    T3_LIGHT,
    Y_LIGHT,             // green, red (+/-)
    X1_LIGHT = Y_LIGHT + 2,
    X2_LIGHT = X1_LIGHT + 2,
    X3_LIGHT = X2_LIGHT + 2,
    NUM_LIGHTS = X3_LIGHT + 2
  };

  RandomSampler() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(DEJA_VU_PARAM, 0.f, 1.f, 0.5f, "Deja vu", "%", 100.f);
    configParam(LENGTH_PARAM, 0.f, 1.f, 0.5f, "Loop length", "%", 100.f);
    configParam(T_RATE_PARAM, 0.f, 1.f, 0.5f, "t rate", "semitones", 120.f, -60.f);
    configParam(T_BIAS_PARAM, 0.f, 1.f, 0.5f, "t bias (probability)", "%", 100.f);
    configParam(T_JITTER_PARAM, 0.f, 1.f, 0.f, "t jitter", "%", 100.f);
    configParam(X_SPREAD_PARAM, 0.f, 1.f, 0.5f, "X spread", "%", 100.f);
    configParam(X_BIAS_PARAM, 0.f, 1.f, 0.5f, "X bias", "%", 100.f);
    configParam(X_STEPS_PARAM, 0.f, 1.f, 0.5f, "X steps", "%", 100.f);
    configButton(T_DEJA_VU_BUTTON, "t deja vu (hold 2 s: lock)");
    configButton(X_DEJA_VU_BUTTON, "X deja vu (hold 2 s: lock)");
    configButton(T_MODE_BUTTON, "t mode (hold 2 s: alternate modes)");
    configButton(T_RANGE_BUTTON, "t range");
    configButton(X_MODE_BUTTON, "X mode");
    configButton(X_RANGE_BUTTON, "X range (hold 2 s: scale select)");
    configButton(X_EXT_BUTTON, "X external processing (hold 2 s: record scale)");

    const std::vector<std::string> dejaVu = {"Off", "On", "Locked"};
    const std::vector<std::string> ranges = {"Narrow (0..+2 V)", "Positive (0..+5 V)",
                                             "Full (-5..+5 V)"};
    configSwitch(T_DEJA_VU_STATE, 0.f, 2.f, 0.f, "t deja vu", dejaVu).hidden = true;
    configSwitch(X_DEJA_VU_STATE, 0.f, 2.f, 0.f, "X deja vu", dejaVu).hidden = true;
    configSwitch(T_MODEL_STATE, 0.f, 5.f, 0.f, "t mode",
                 std::vector<std::string>(kTModelNames, kTModelNames + 6)).hidden = true;
    configSwitch(T_RANGE_STATE, 0.f, 2.f, 1.f, "t range", {"1/4x", "1x", "4x"}).hidden = true;
    configSwitch(X_MODE_STATE, 0.f, 2.f, 0.f, "X mode",
                 {"Identical", "Bump", "Tilt"}).hidden = true;
    configSwitch(X_RANGE_STATE, 0.f, 2.f, 2.f, "X range", ranges).hidden = true;
    configSwitch(X_EXT_STATE, 0.f, 1.f, 0.f, "X external processing (EXT)",
                 {"Off", "On"}).hidden = true;
    configSwitch(SCALE_STATE, 0.f, 5.f, 0.f, "Scale",
                 std::vector<std::string>(kScaleNames, kScaleNames + kNumScales)).hidden = true;
    configParam(T_PW_MEAN_PARAM, 0.f, 1.f, 0.5f, "t gate length", "%", 100.f).hidden = true;
    configParam(T_PW_STD_PARAM, 0.f, 1.f, 0.f, "t gate length randomness", "%", 100.f)
        .hidden = true;
    configParam(Y_SPREAD_PARAM, 0.f, 1.f, 0.5f, "Y spread", "%", 100.f).hidden = true;
    configParam(Y_BIAS_PARAM, 0.f, 1.f, 0.5f, "Y bias", "%", 100.f).hidden = true;
    configParam(Y_STEPS_PARAM, 0.f, 1.f, 0.f, "Y steps", "%", 100.f).hidden = true;
    configSwitch(Y_DIVIDER_PARAM, 0.f, 11.f, 6.f, "Y clock divider",
                 {"1/64", "1/48", "1/32", "1/24", "1/16", "1/12", "1/8", "1/6", "1/4", "1/3",
                  "1/2", "1/1"}).hidden = true;
    configSwitch(Y_RANGE_PARAM, 0.f, 2.f, 2.f, "Y range", ranges).hidden = true;
    configSwitch(EXPLICIT_RESET_PARAM, 0.f, 1.f, 0.f,
                 "Jitter/Steps CV inputs reset t/X", {"Off", "On"}).hidden = true;

    configInput(T_BIAS_INPUT, "t bias");
    configInput(X_BIAS_INPUT, "X bias");
    configInput(T_CLOCK_INPUT, "t clock");
    configInput(T_RATE_INPUT, "t rate");
    configInput(T_JITTER_INPUT, "t jitter");
    configInput(DEJA_VU_INPUT, "Deja vu");
    configInput(X_STEPS_INPUT, "X steps");
    configInput(X_SPREAD_INPUT, "X spread");
    configInput(X_CLOCK_INPUT, "X clock");
    configOutput(T1_OUTPUT, "t1");
    configOutput(T2_OUTPUT, "t2");
    configOutput(T3_OUTPUT, "t3");
    configOutput(Y_OUTPUT, "Y");
    configOutput(X1_OUTPUT, "X1");
    configOutput(X2_OUTPUT, "X2");
    configOutput(X3_OUTPUT, "X3");

    // Same initialisation order as marbles.cc Init().
    // The firmware's DSP objects are zero-initialised globals: do the same
    // (Init() doesn't set every member).
    std::memset(static_cast<void*>(&randomGenerator_), 0, sizeof(randomGenerator_));
    std::memset(static_cast<void*>(&randomStream_), 0, sizeof(randomStream_));
    std::memset(static_cast<void*>(&tGenerator_), 0, sizeof(tGenerator_));
    std::memset(static_cast<void*>(&xyGenerator_), 0, sizeof(xyGenerator_));
    std::memset(static_cast<void*>(&noteFilter_), 0, sizeof(noteFilter_));
    std::memset(static_cast<void*>(&scaleRecorder_), 0, sizeof(scaleRecorder_));
    std::memset(static_cast<void*>(detector_), 0, sizeof(detector_));
    std::memset(static_cast<void*>(&dejaVuLengthQuantizer_), 0, sizeof(dejaVuLengthQuantizer_));
    std::memset(static_cast<void*>(channel_), 0, sizeof(channel_));
    std::random_device rd;
    rngState_ = rd() | 1u;
    noteFilter_.Init();
    dejaVuLengthQuantizer_.Init(sizeof(kLoopLength) / sizeof(int), 0.25f, false);
    for (int i = 0; i < marbles::ADC_CHANNEL_LAST; ++i) {
      // Calibration in volts: channels see V / 5 (T rate: semitones), exactly
      // the values the factory calibration produces from the ADC codes.
      cvScale_[i] = i == marbles::ADC_CHANNEL_T_RATE ? 12.f : 0.2f;
      cvOffset_[i] = 0.f;
      channel_[i].Init(&cvScale_[i], &cvOffset_[i], kChannelSettings[i]);
      attenuverter_[i] = 1.01f;  // cv_reader.cc: a little over 100%
    }
    attenuverter_[marbles::ADC_CHANNEL_DEJA_VU_LENGTH] = 0.5f;  // no CV for length
    scaleRecorder_.Init();
    randomGenerator_.Init(1);
    randomStream_.Init(&randomGenerator_);
    tGenerator_.Init(&randomStream_, kNativeRate);
    xyGenerator_.Init(&randomStream_, kNativeRate);
    for (int i = 0; i < kNumScales; ++i) {
      scales_[i] = kPresetScales[i];
      xyGenerator_.LoadScale(i, scales_[i]);
    }
    // Firmware bug: LoadScale() only reaches the X channels, so Y keeps the
    // placeholder from OutputChannel::Init() (one degree, weight 0). With Y
    // STEPS above 50% its quantizer then reads voltage_[0xff] (level_[n].first
    // stays 0xff for every threshold above 0). Give Y the same single degree
    // with weight 255: every level then quantizes to whole volts, which is
    // what the placeholder already does at the lowest level.
    {
      marbles::Scale octaves;
      octaves.Init();
      octaves.degree[0].weight = 255;
      for (int i = 0; i < kNumScales; ++i) {
        xyGenerator_.LoadScale(marbles::kNumXChannels, i, octaves);
      }
    }
    for (size_t i = 0; i < marbles::kNumGateOutputs; ++i) detector_[i].Init(i);
    std::memset(&block_, 0, sizeof(block_));
    std::memset(cvOut_, 0, sizeof(cvOut_));
    std::memset(gateDelayTail_, 0, sizeof(gateDelayTail_));
    onSampleRateChange(48000.f);
  }

  void onSampleRateChange(float sampleRate) override {
    step_ = kNativeRate / sampleRate;
  }

  void onReset() override {
    for (int i = 0; i < kNumScales; ++i) pendingScales_[i] = kPresetScales[i];
    dirtyScales_.store((1 << kNumScales) - 1, std::memory_order_release);
  }

  void process(const ProcessArgs& args) override {
    // The firmware's UI runs from a 1 kHz timer.
    uiPhase_ += args.sampleTime * 1000.f;
    while (uiPhase_ >= 1.f) {
      uiPhase_ -= 1.f;
      pollUi();
    }

    phase_ += step_;
    while (phase_ >= 1.f) {
      phase_ -= 1.f;
      nativeSample();
    }
    for (int i = 0; i < 3; ++i) {
      outputs[static_cast<size_t>(T1_OUTPUT + i)].setVoltage(gateOut_[i] ? 10.f : 0.f);
    }
    outputs[X1_OUTPUT].setVoltage(voltageOut_[0]);
    outputs[X2_OUTPUT].setVoltage(voltageOut_[1]);
    outputs[X3_OUTPUT].setVoltage(voltageOut_[2]);
    outputs[Y_OUTPUT].setVoltage(voltageOut_[3]);
  }

  // BetaDistributionSample() reads table rows bias * 8 and bias * 8 + 1 of a
  // 5-row table (it mirrors biases above 0.5): at exactly 0.5, the centre of
  // the knob, the second one is row 5, one past the end (with zero weight).
  // Nudge that point just above 0.5, where the mirrored lookup is in range;
  // the change (1e-5) is inaudible.
  static float betaSafeBias(float bias) {
    return bias > 0.49999f && bias <= 0.5f ? 0.50001f : bias;
  }

  // What marbles::Quantizer::Init() / Process() assume about a scale: 1 to
  // kMaxDegrees degrees, voltages finite, sorted and within one octave, and
  // at least one degree with weight 255. (ScaleRecorder can store a degree
  // just below 0: notes within 1/36 V under an octave are folded onto it.)
  static bool validScale(const marbles::Scale& sc) {
    if (sc.num_degrees < 1 || sc.num_degrees > marbles::kMaxDegrees) return false;
    if (!(sc.base_interval == 1.f)) return false;
    bool hasRoot = false;
    float previous = -0.03f;
    for (int d = 0; d < sc.num_degrees; ++d) {
      float v = sc.degree[d].voltage;
      if (!std::isfinite(v) || v < previous || v >= 1.f) return false;
      previous = v;
      hasRoot = hasRoot || sc.degree[d].weight == 255;
    }
    return hasRoot;
  }

  // Recorded scales are persistent data on the hardware: save them here.
  void saveState(StateMap& s) const override {
    for (int i = 0; i < kNumScales; ++i) {
      const marbles::Scale& sc = scales_[i];
      bool preset = sc.num_degrees == kPresetScales[i].num_degrees;
      for (int d = 0; preset && d < sc.num_degrees; ++d) {
        preset = sc.degree[d].voltage == kPresetScales[i].degree[d].voltage &&
                 sc.degree[d].weight == kPresetScales[i].degree[d].weight;
      }
      if (preset) continue;
      std::string text;
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%d", sc.num_degrees);
      text = buf;
      for (int d = 0; d < sc.num_degrees; ++d) {
        std::snprintf(buf, sizeof(buf), " %.5f:%d", static_cast<double>(sc.degree[d].voltage),
                      sc.degree[d].weight);
        text += buf;
      }
      s["scale" + std::to_string(i)] = text;
    }
  }

  void loadState(const StateMap& s) override {
    for (int i = 0; i < kNumScales; ++i) {
      marbles::Scale sc = kPresetScales[i];
      auto it = s.find("scale" + std::to_string(i));
      if (it != s.end()) {
        const char* p = it->second.c_str();
        int n = 0, used = 0, parsed = 0;
        if (std::sscanf(p, "%d%n", &n, &used) == 1 && n >= 2 && n <= marbles::kMaxDegrees) {
          p += used;
          sc.base_interval = 1.f;
          sc.num_degrees = n;
          for (int d = 0; d < n; ++d) {
            float v = 0.f;
            int w = 0;
            if (std::sscanf(p, " %f:%d%n", &v, &w, &used) != 2) break;
            p += used;
            sc.degree[d].voltage = v;
            sc.degree[d].weight = static_cast<uint8_t>(std::min(255, std::max(1, w)));
            ++parsed;
          }
        }
        // Anything the scale recorder can't produce falls back to the preset:
        // e.g. without a weight-255 degree marbles::Quantizer reads
        // voltage_[0xff] (level_[6].first stays 0xff).
        if (parsed != n || !validScale(sc)) sc = kPresetScales[i];
      }
      pendingScales_[i] = sc;
    }
    dirtyScales_.store((1 << kNumScales) - 1, std::memory_order_release);
  }

  void appendMenu(std::vector<MenuAction>& items) override {
    items.push_back({"Reset current scale to preset", nullptr, [this](int dir) {
                       if (dir != 0) return;
                       int i = static_cast<int>(params[SCALE_STATE].value + 0.5f);
                       pendingScales_[i] = kPresetScales[i];
                       dirtyScales_.fetch_or(1 << i, std::memory_order_release);
                     }});
  }

  // For the panel readout.
  int tModel() const { return static_cast<int>(params[T_MODEL_STATE].value + 0.5f); }
  int xMode() const { return static_cast<int>(params[X_MODE_STATE].value + 0.5f); }
  int scale() const { return static_cast<int>(params[SCALE_STATE].value + 0.5f); }
  UiMode uiMode() const { return mode_; }

 private:
  int state(int id) const { return static_cast<int>(params[static_cast<size_t>(id)].value + 0.5f); }
  // Hidden 8-bit settings (y_spread etc.) as the firmware stores them.
  uint8_t state8(int id) const {
    return static_cast<uint8_t>(clampf(params[static_cast<size_t>(id)].value, 0.f, 1.f) * 255.f);
  }
  void setState(int id, int v) { params[static_cast<size_t>(id)].set(static_cast<float>(v)); }

  uint32_t nextRandom() {  // stands in for the STM32 hardware RNG
    rngState_ ^= rngState_ << 13;
    rngState_ ^= rngState_ >> 17;
    rngState_ ^= rngState_ << 5;
    return rngState_;
  }

  // One 32 kHz sample: FillBuffer() of the firmware.
  void nativeSample() {
    for (int i = 0; i < 2; ++i) {
      const Port& in = inputs[i == 0 ? T_CLOCK_INPUT : X_CLOCK_INPUT];
      clockTrigger_[i].process(in.getVoltage());
      clockFlags_[i] = stmlib::ExtractGateFlags(clockFlags_[i], clockTrigger_[i].isHigh());
      block_.input[i][pos_] = clockFlags_[i];
    }
    randomStream_.Write(nextRandom());
    for (int i = 0; i < 3; ++i) gateOut_[i] = block_.gate_output[i][pos_];
    for (int i = 0; i < 4; ++i) voltageOut_[i] = cvOut_[i][pos_];
    if (++pos_ == kBlockSize) {
      pos_ = 0;
      // The normalisation probe is replaced by the cable state.
      block_.input_patched[0] = inputs[T_CLOCK_INPUT].isConnected();
      block_.input_patched[1] = inputs[X_CLOCK_INPUT].isConnected();
      renderBlock();
    }
  }

  // cv_reader.cc CvReader::Process(), fed from the panel and the CV inputs.
  void readCv(bool explicitReset, float* parameters, GateFlags* gates) {
    using namespace marbles;
    attenuverter_[ADC_CHANNEL_T_JITTER] = explicitReset ? 0.5f : 1.01f;
    attenuverter_[ADC_CHANNEL_X_STEPS] = explicitReset ? 0.5f : 1.01f;
    auto cv = [&](int input) { return clampf(inputs[static_cast<size_t>(input)].getVoltage(),
                                             -5.f, 5.f); };
    const float pot[ADC_CHANNEL_LAST] = {
        params[DEJA_VU_PARAM].value, params[LENGTH_PARAM].value,
        params[T_RATE_PARAM].value,  params[T_BIAS_PARAM].value,
        params[T_JITTER_PARAM].value, params[X_SPREAD_PARAM].value,
        params[X_BIAS_PARAM].value,  params[X_STEPS_PARAM].value,
    };
    // The length pot shares its ADC channel with a second reading of the
    // spread CV (ADC_CHANNEL_X_SPREAD_2).
    const float volts[ADC_CHANNEL_LAST] = {
        cv(DEJA_VU_INPUT), cv(X_SPREAD_INPUT), cv(T_RATE_INPUT), cv(T_BIAS_INPUT),
        cv(T_JITTER_INPUT), cv(X_SPREAD_INPUT), cv(X_BIAS_INPUT), cv(X_STEPS_INPUT),
    };
    for (int i = 0; i < ADC_CHANNEL_LAST; ++i) {
      parameters[i] = channel_[i].Process(pot[i], volts[i], attenuverter_[i]);
      gates[i] = channel_[i].gate_flags();
    }
  }

  // marbles.cc Process().
  void renderBlock() {
    using namespace marbles;
    const size_t size = kBlockSize;
    const bool explicitReset = state(EXPLICIT_RESET_PARAM) != 0;

    int dirty = dirtyScales_.exchange(0, std::memory_order_acquire);
    for (int i = 0; dirty && i < kNumScales; ++i) {
      if (dirty & (1 << i)) {
        scales_[i] = pendingScales_[i];
        xyGenerator_.LoadScale(i, scales_[i]);
      }
    }

    float parameters[ADC_CHANNEL_LAST];
    GateFlags hiddenGates[ADC_CHANNEL_LAST];
    readCv(explicitReset, parameters, hiddenGates);

    float dejaVu = parameters[ADC_CHANNEL_DEJA_VU_AMOUNT];
    // Deadband near 12 o'clock for the deja vu parameter.
    const float d = std::fabs(dejaVu - 0.5f);
    if (d > 0.03f) {
      dejaVuLock_ = false;
    } else if (d < 0.02f) {
      dejaVuLock_ = true;
    }
    if (dejaVu < 0.47f) {
      dejaVu *= 1.06382978723f;
    } else if (dejaVu > 0.53f) {
      dejaVu = 0.5f + (dejaVu - 0.53f) * 1.06382978723f;
    } else {
      dejaVu = 0.5f;
    }

    GateFlags* tClock = block_.input[0];
    GateFlags* xyClock = block_.input[1];

    // Clock source for the XY section, including self-patching detection.
    ClockSource xyClockSource = CLOCK_SOURCE_INTERNAL_T1_T2_T3;
    if (block_.input_patched[1]) {
      xyClockSource = CLOCK_SOURCE_EXTERNAL;
      size_t bestScore = 8;
      for (size_t i = 0; i < kNumGateOutputs; ++i) {
        size_t score = detector_[i].Process(&block_, size);
        if (score >= bestScore) {
          xyClockSource = ClockSource(CLOCK_SOURCE_INTERNAL_T1 + i);
          bestScore = score;
        }
      }
    }
    xyClockSource_ = xyClockSource;

    Ramps ramps;
    ramps.master = &rampBuffer_[0];
    ramps.external = &rampBuffer_[kBlockSize];
    ramps.slave[0] = &rampBuffer_[kBlockSize * 2];
    ramps.slave[1] = &rampBuffer_[kBlockSize * 3];

    const int tDejaVu = state(T_DEJA_VU_STATE);
    const int xDejaVu = state(X_DEJA_VU_STATE);
    int dejaVuLength =
        dejaVuLengthQuantizer_.Lookup(kLoopLength, parameters[ADC_CHANNEL_DEJA_VU_LENGTH]);

    bool tSectionReset = explicitReset && (hiddenGates[ADC_CHANNEL_T_JITTER] & stmlib::GATE_FLAG_RISING);

    tGenerator_.set_model(TGeneratorModel(state(T_MODEL_STATE)));
    tGenerator_.set_range(TGeneratorRange(state(T_RANGE_STATE)));
    tGenerator_.set_rate(parameters[ADC_CHANNEL_T_RATE]);
    tGenerator_.set_bias(parameters[ADC_CHANNEL_T_BIAS]);
    tGenerator_.set_jitter(parameters[ADC_CHANNEL_T_JITTER]);
    tGenerator_.set_deja_vu(tDejaVu == DEJA_VU_LOCKED ? 0.5f
                                                      : (tDejaVu == DEJA_VU_ON ? dejaVu : 0.0f));
    tGenerator_.set_length(dejaVuLength);
    tGenerator_.set_pulse_width_mean(float(state8(T_PW_MEAN_PARAM)) / 256.0f);
    tGenerator_.set_pulse_width_std(float(state8(T_PW_STD_PARAM)) / 256.0f);
    tGenerator_.Process(block_.input_patched[0], &tSectionReset, tClock, ramps, gates_, size);
    // TGenerator holds its slave ramps at exactly 1.0 at the end of a cycle,
    // and LagProcessor::Process() then reads lut_raised_cosine[257] (257
    // entries) through Interpolate(..., phase, 256). Keep the ramps it wrote
    // just below 1 (no audible difference; the ordering of samples, which the
    // output channels use to detect a new step, is unchanged).
    for (float* r : {ramps.master, ramps.slave[0], ramps.slave[1]}) {
      for (size_t i = 0; i < size; ++i) r[i] = std::min(r[i], 0.99999994f);
    }

    // X section.
    float noteCv1 = channel_[ADC_CHANNEL_X_SPREAD].scaled_raw_cv();
    float noteCv2 = channel_[ADC_CHANNEL_X_SPREAD_2].scaled_raw_cv();
    float noteCv = 0.5f * (noteCv1 + noteCv2);
    float u = noteFilter_.Process(0.5f * (noteCv + 1.0f));

    if (mode_ == UI_MODE_RECORD_SCALE) {
      float voltage = (u - 0.5f) * 10.0f;
      for (size_t i = 0; i < size; ++i) {
        GateFlags gate = block_.input_patched[1] ? block_.input[1][i] : stmlib::GATE_FLAG_LOW;
        if (gate & stmlib::GATE_FLAG_RISING) scaleRecorder_.NewNote(voltage);
        if (gate & stmlib::GATE_FLAG_HIGH) scaleRecorder_.UpdateVoltage(voltage);
        if (gate & stmlib::GATE_FLAG_FALLING) scaleRecorder_.AcceptNote();
      }
      std::fill(&voltages_[0], &voltages_[4 * size], voltage);
    } else {
      const bool registerMode = state(X_EXT_STATE) != 0;
      x_.control_mode = ControlMode(state(X_MODE_STATE));
      x_.voltage_range = VoltageRange(state(X_RANGE_STATE) % 3);
      x_.register_mode = registerMode;
      // Just below 1: RandomSequence::GetReplayValue() converts
      // value * 2^32 to uint32 (out of range at exactly 1).
      x_.register_value = clampf(u, 0.f, 0.99999994f);
      attenuverter_[ADC_CHANNEL_X_SPREAD] = registerMode ? 0.5f : 1.0f;

      x_.spread = parameters[ADC_CHANNEL_X_SPREAD];
      x_.bias = betaSafeBias(parameters[ADC_CHANNEL_X_BIAS]);
      x_.steps = parameters[ADC_CHANNEL_X_STEPS];
      x_.deja_vu = xDejaVu == DEJA_VU_LOCKED ? 0.5f : (xDejaVu == DEJA_VU_ON ? dejaVu : 0.0f);
      x_.length = dejaVuLength;
      x_.ratio.p = 1;
      x_.ratio.q = 1;

      y_.control_mode = CONTROL_MODE_IDENTICAL;
      y_.voltage_range = VoltageRange(state(Y_RANGE_PARAM));
      y_.register_mode = false;
      y_.register_value = 0.0f;
      y_.spread = float(state8(Y_SPREAD_PARAM)) / 256.0f;
      y_.bias = betaSafeBias(float(state8(Y_BIAS_PARAM)) / 256.0f);
      y_.steps = float(state8(Y_STEPS_PARAM)) / 256.0f;
      y_.deja_vu = 0.0f;
      y_.length = 1;
      y_.ratio = kYDividerRatios[std::min(11, std::max(0, state(Y_DIVIDER_PARAM)))];

      y_.scale_index = x_.scale_index = state(SCALE_STATE);

      bool xSectionReset =
          explicitReset && (hiddenGates[ADC_CHANNEL_X_STEPS] & stmlib::GATE_FLAG_RISING);
      if (xyClockSource != CLOCK_SOURCE_EXTERNAL) xSectionReset |= tSectionReset;

      xyGenerator_.Process(xyClockSource, x_, y_, &xSectionReset, xyClock, ramps, voltages_,
                           size);
    }

    const float* v = voltages_;
    const bool* g = gates_;
    for (size_t i = 0; i < size; ++i) {
      // CONSTRAIN in DacCode(): the DAC swings +/-5 V.
      for (int c = 0; c < 4; ++c) cvOut_[c][i] = clampf(*v++, -5.f, 5.f);
      block_.gate_output[0][i + kGateDelay] = *g++;
      block_.gate_output[1][i + kGateDelay] = ramps.master[i] < 0.5f;
      block_.gate_output[2][i + kGateDelay] = *g++;
    }
    for (size_t i = 0; i < kNumGateOutputs; ++i) {
      for (size_t j = 0; j < kGateDelay; ++j) {
        block_.gate_output[i][j] = gateDelayTail_[i][j];
        gateDelayTail_[i][j] = block_.gate_output[i][size + j];
      }
    }
  }

  // --- ui.cc, polled at 1 kHz ----------------------------------------------

  void pollUi() {
    ++ms_;
    for (int b = 0; b < kNumButtons; ++b) {
      bool pressed = params[static_cast<size_t>(T_DEJA_VU_BUTTON + b)].value > 0.5f;
      if (pressed && !wasPressed_[b]) {
        pressTime_[b] = ms_;
        ignoreRelease_[b] = false;
        lastEventMs_ = ms_;
      }
      if (pressed && !ignoreRelease_[b] && ms_ - pressTime_[b] > kLongPressMs) {
        onSwitchReleased(b, ms_ - pressTime_[b]);
        ignoreRelease_[b] = true;
      }
      if (!pressed && wasPressed_[b] && !ignoreRelease_[b]) {
        onSwitchReleased(b, ms_ - pressTime_[b] + 1);
        ignoreRelease_[b] = true;
      }
      wasPressed_[b] = pressed;
    }
    if (mode_ == UI_MODE_SELECT_SCALE && ms_ - lastEventMs_ > 4000) mode_ = UI_MODE_NORMAL;
    updateLeds();
  }

  void onSwitchReleased(int b, int32_t duration) {
    lastEventMs_ = ms_;
    const bool longPress = duration >= kLongPressMs;
    switch (T_DEJA_VU_BUTTON + b) {
      case T_DEJA_VU_BUTTON:
      case X_DEJA_VU_BUTTON: {
        int id = b == 0 ? T_DEJA_VU_STATE : X_DEJA_VU_STATE;
        int s = state(id);
        if (s == DEJA_VU_OFF) {
          s = longPress ? DEJA_VU_LOCKED : DEJA_VU_ON;
        } else if (s == DEJA_VU_LOCKED) {
          s = DEJA_VU_ON;
        } else {
          s = longPress ? DEJA_VU_LOCKED : DEJA_VU_OFF;
        }
        setState(id, s);
        break;
      }
      case T_MODE_BUTTON: {
        int model = state(T_MODEL_STATE);
        int bank = model / 3;
        if (longPress) {
          if (!bank) model += 3;
        } else {
          model = bank ? model - 3 : (model + 1) % 3;
        }
        setState(T_MODEL_STATE, model);
        break;
      }
      case T_RANGE_BUTTON:
        setState(T_RANGE_STATE, (state(T_RANGE_STATE) + 1) % 3);
        break;
      case X_MODE_BUTTON:
        setState(X_MODE_STATE, (state(X_MODE_STATE) + 1) % 3);
        break;
      case X_EXT_BUTTON:
        if (mode_ == UI_MODE_RECORD_SCALE) {
          int index = state(SCALE_STATE);
          bool success = true;
          if (longPress) {
            scales_[index] = kPresetScales[index];
          } else {
            success = scaleRecorder_.ExtractScale(&scales_[index]);
            // Same checks as for patch-loaded scales (see loadState()).
            if (success && !validScale(scales_[index])) scales_[index] = kPresetScales[index];
          }
          if (success) xyGenerator_.LoadScale(index, scales_[index]);
          mode_ = UI_MODE_NORMAL;
        } else if (longPress) {
          mode_ = UI_MODE_RECORD_SCALE;
          scaleRecorder_.Clear();
        } else {
          setState(X_EXT_STATE, !state(X_EXT_STATE));
        }
        break;
      case X_RANGE_BUTTON:
        if (longPress) {
          if (mode_ == UI_MODE_NORMAL) mode_ = UI_MODE_SELECT_SCALE;
        } else if (mode_ == UI_MODE_SELECT_SCALE) {
          setState(SCALE_STATE, (state(SCALE_STATE) + 1) % kNumScales);
        } else if (!state(X_EXT_STATE)) {
          setState(X_RANGE_STATE, (state(X_RANGE_STATE) + 1) % 3);
        }
        break;
    }
  }

  Rg makeColor(int value) const {
    bool slowBlink = (ms_ & 255) > 128;
    int bank = value >= 3 ? 1 : 0;
    Rg color = kPalette[value - bank * 3];
    return slowBlink || !bank ? color : kOff;
  }

  // The firmware PWMs the LED with a triangle; render that as brightness.
  float dejaVuBrightness(int s) const {
    if (s == DEJA_VU_OFF) return 0.f;
    if (s == DEJA_VU_ON) {
      if (!dejaVuLock_) return 1.f;
      int tri = static_cast<int>(ms_ & 1023) >> 5;
      tri = tri >= 16 ? 31 - tri : tri;
      return static_cast<float>(tri) / 15.f;
    }
    int tri = static_cast<int>(ms_ & 511) >> 4;
    tri = tri >= 16 ? 31 - tri : tri;
    return static_cast<float>(tri) / 15.f;
  }

  void setRg(int light, Rg c) {
    lights[static_cast<size_t>(light)] = c.r;
    lights[static_cast<size_t>(light + 1)] = c.g;
    lights[static_cast<size_t>(light + 2)] = 0.f;
  }

  void updateLeds() {
    bool slowBlink = (ms_ & 255) > 128;
    bool fastBlink = (ms_ & 63) > 32;
    int scale = state(SCALE_STATE);
    Rg scaleColor = scale < 3 ? (slowBlink ? kPalette[scale] : kOff)
                              : (fastBlink ? kPalette[scale - 3] : kOff);
    bool registerMode = state(X_EXT_STATE) != 0;
    Rg tModel = kOff, tRange = kOff, xMode = kOff, xRange = kOff;
    float tDv = 0.f, xDv = 0.f, ext = 0.f;
    if (mode_ == UI_MODE_SELECT_SCALE) {
      xRange = scaleColor;
    } else {
      tModel = makeColor(state(T_MODEL_STATE));
      tRange = makeColor(state(T_RANGE_STATE));
      xMode = makeColor(state(X_MODE_STATE));
      tDv = dejaVuBrightness(state(T_DEJA_VU_STATE));
      xDv = dejaVuBrightness(state(X_DEJA_VU_STATE));
      if (mode_ == UI_MODE_NORMAL) {
        xRange = registerMode ? kOff : makeColor(state(X_RANGE_STATE));
        ext = registerMode ? 1.f : 0.f;
      } else {
        xRange = scaleColor;
        ext = 1.f;
      }
    }
    setRg(T_MODEL_LIGHT, tModel);
    setRg(T_RANGE_LIGHT, tRange);
    setRg(X_MODE_LIGHT, xMode);
    setRg(X_RANGE_LIGHT, xRange);
    lights[T_DEJA_VU_LIGHT] = tDv;
    lights[X_DEJA_VU_LIGHT] = xDv;
    lights[X_EXT_LIGHT] = ext;
    // Output LEDs follow the jacks (hardware, not firmware).
    for (int i = 0; i < 3; ++i) lights[static_cast<size_t>(T1_LIGHT + i)] = gateOut_[i] ? 1.f : 0.f;
    const int cvLights[4] = {X1_LIGHT, X2_LIGHT, X3_LIGHT, Y_LIGHT};
    for (int i = 0; i < 4; ++i) {
      float v = voltageOut_[i] / 5.f;
      lights[static_cast<size_t>(cvLights[i])] = std::max(0.f, v);
      lights[static_cast<size_t>(cvLights[i] + 1)] = std::max(0.f, -v);
    }
  }

  static constexpr int kNumButtons = X_EXT_BUTTON - T_DEJA_VU_BUTTON + 1;

  // DSP (marbles.cc globals).
  marbles::RandomGenerator randomGenerator_;
  marbles::RandomStream randomStream_;
  marbles::TGenerator tGenerator_;
  marbles::XYGenerator xyGenerator_;
  marbles::NoteFilter noteFilter_;
  marbles::ScaleRecorder scaleRecorder_;
  marbles::ClockSelfPatchingDetector detector_[marbles::kNumGateOutputs];
  stmlib::HysteresisQuantizer2 dejaVuLengthQuantizer_;
  marbles::CvReaderChannel channel_[marbles::ADC_CHANNEL_LAST];
  float cvScale_[marbles::ADC_CHANNEL_LAST];
  float cvOffset_[marbles::ADC_CHANNEL_LAST];
  float attenuverter_[marbles::ADC_CHANNEL_LAST];
  marbles::GroupSettings x_, y_;
  marbles::Scale scales_[kNumScales];
  marbles::Scale pendingScales_[kNumScales];
  std::atomic<int> dirtyScales_{0};

  marbles::IOBuffer::Block block_;
  float cvOut_[4][kBlockSize];
  bool gateDelayTail_[marbles::kNumGateOutputs][kGateDelay];
  float rampBuffer_[kBlockSize * 4];
  bool gates_[kBlockSize * 2];
  float voltages_[kBlockSize * 4];
  marbles::ClockSource xyClockSource_ = marbles::CLOCK_SOURCE_INTERNAL_T1_T2_T3;
  bool dejaVuLock_ = false;

  // Native clock and I/O.
  float step_ = 2.f / 3.f;
  float phase_ = 0.f;
  size_t pos_ = 0;
  uint32_t rngState_ = 1;
  SchmittTrigger clockTrigger_[2];
  GateFlags clockFlags_[2] = {0, 0};
  bool gateOut_[3] = {false, false, false};
  float voltageOut_[4] = {0.f, 0.f, 0.f, 0.f};

  // UI.
  float uiPhase_ = 0.f;
  uint32_t ms_ = 0;
  uint32_t lastEventMs_ = 0;
  uint32_t pressTime_[kNumButtons] = {};
  bool wasPressed_[kNumButtons] = {};
  bool ignoreRelease_[kNumButtons] = {};
  UiMode mode_ = UI_MODE_NORMAL;
};

void buildPanel(PanelSpec& p) {
  using M = RandomSampler;
  p.hp = 18;
  p.bg = 0xFFF3F2EE;
  p.ink = 0xFF1E1E20;
  const uint32_t dark = 0xFF5E5E60;
  const uint32_t white = 0xFFFFFFFF;
  const uint32_t teal = 0xFF168A8C, ochre = 0xFFD39F2B, red = 0xFFC8414B;
  const uint32_t dejaVuGreen = 0xFF35E06A;

  p.label(6.2f, 16.6f, "t", 6.5f);
  p.label(85.2f, 16.6f, "X", 6.5f);

  // Deja vu: buttons either side of the big knob (lit green when active).
  p.button(M::T_DEJA_VU_BUTTON, 16.3f, 17.3f, 7.f, "", 0xFFD8D9DB);
  p.light(M::T_DEJA_VU_LIGHT, 16.3f, 17.3f, dejaVuGreen, 3.6f);
  p.button(M::X_DEJA_VU_BUTTON, 74.9f, 17.3f, 7.f, "", 0xFFD8D9DB);
  p.light(M::X_DEJA_VU_LIGHT, 74.9f, 17.3f, dejaVuGreen, 3.6f);
  p.line(20.6f, 17.3f, 38.4f, 20.2f, 0.3f);
  p.line(52.8f, 20.2f, 70.6f, 17.3f, 0.3f);
  p.knob(M::DEJA_VU_PARAM, 45.6f, 21.2f, 13.f);
  p.label(45.6f, 33.3f, "DEJA VU", 3.3f);

  // t / X mode buttons with their tricolour LEDs.
  p.label(6.7f, 25.6f, "MODE", 2.6f);
  p.rgbLight(M::T_MODEL_LIGHT, M::T_MODEL_LIGHT + 1, M::T_MODEL_LIGHT + 2, 6.7f, 29.8f, 2.6f);
  p.button(M::T_MODE_BUTTON, 6.7f, 38.3f, 6.f);
  p.label(84.4f, 25.6f, "MODE", 2.6f);
  p.rgbLight(M::X_MODE_LIGHT, M::X_MODE_LIGHT + 1, M::X_MODE_LIGHT + 2, 84.4f, 29.8f, 2.6f);
  p.button(M::X_MODE_BUTTON, 84.4f, 38.3f, 6.f);

  p.knob(M::T_RATE_PARAM, 23.4f, 35.0f, 13.f);
  p.label(23.4f, 48.3f, "RATE", 3.3f);
  p.knob(M::X_SPREAD_PARAM, 67.8f, 35.0f, 13.f);
  p.label(67.8f, 48.3f, "SPREAD", 3.3f);

  // Range buttons.
  p.rgbLight(M::T_RANGE_LIGHT, M::T_RANGE_LIGHT + 1, M::T_RANGE_LIGHT + 2, 26.6f, 53.7f, 2.6f);
  p.button(M::T_RANGE_BUTTON, 26.6f, 59.4f, 6.f);
  p.label(31.9f, 53.7f, "/4", 2.5f, kAlignCenter, teal);
  p.label(31.9f, 57.3f, "x1", 2.5f, kAlignCenter, ochre);
  p.label(31.9f, 60.9f, "x4", 2.5f, kAlignCenter, red);
  p.rgbLight(M::X_RANGE_LIGHT, M::X_RANGE_LIGHT + 1, M::X_RANGE_LIGHT + 2, 64.6f, 53.7f, 2.6f);
  p.button(M::X_RANGE_BUTTON, 64.6f, 59.4f, 6.f);
  p.label(59.3f, 53.7f, "+2", 2.5f, kAlignCenter, teal);
  p.label(59.3f, 57.3f, "+5", 2.5f, kAlignCenter, ochre);
  p.label(59.3f, 60.9f, "+-5", 2.5f, kAlignCenter, red);

  // Loop length with its scale.
  p.knob(M::LENGTH_PARAM, 45.6f, 50.4f, 9.f);
  p.label(45.6f, 61.6f, "LENGTH", 3.0f);
  const struct { float x, y; const char* t; } marks[] = {
      {40.3f, 57.6f, "1"}, {37.6f, 53.2f, "2"}, {37.6f, 47.6f, "3"}, {41.0f, 43.3f, "4"},
      {50.2f, 43.3f, "6"}, {53.6f, 47.6f, "8"}, {53.8f, 53.2f, "12"}, {51.0f, 57.6f, "16"}};
  for (const auto& m : marks) p.label(m.x, m.y, m.t, 2.2f);

  p.knob(M::T_BIAS_PARAM, 9.4f, 58.2f, 9.f);
  p.label(9.4f, 69.4f, "BIAS", 3.0f);
  p.knob(M::X_BIAS_PARAM, 81.9f, 58.2f, 9.f);
  p.label(81.9f, 69.4f, "BIAS", 3.0f);

  // External processing button and LED.
  p.button(M::X_EXT_BUTTON, 45.6f, 67.4f, 5.f);
  p.light(M::X_EXT_LIGHT, 45.6f, 73.6f, dejaVuGreen, 2.4f);
  p.label(45.6f, 77.3f, "EXT", 2.5f);

  p.knob(M::T_JITTER_PARAM, 31.5f, 72.6f, 9.f);
  p.label(31.5f, 83.4f, "JITTER", 3.0f);
  p.knob(M::X_STEPS_PARAM, 59.8f, 72.6f, 9.f);
  p.label(59.8f, 83.4f, "STEPS", 3.0f);

  // CV/clock inputs.
  p.input(M::T_BIAS_INPUT, 9.4f, 81.6f);
  p.input(M::X_BIAS_INPUT, 81.9f, 81.6f);
  p.line(9.4f, 71.4f, 9.4f, 77.6f, 0.3f);
  p.line(81.9f, 71.4f, 81.9f, 77.6f, 0.3f);
  p.label(9.4f, 90.2f, "CLOCK", 3.0f);
  p.label(45.6f, 90.2f, "DEJA VU", 3.0f);
  p.label(81.9f, 90.2f, "CLOCK", 3.0f);
  const float jy = 96.4f;
  p.input(M::T_CLOCK_INPUT, 9.4f, jy);
  p.input(M::T_RATE_INPUT, 21.3f, jy);
  p.input(M::T_JITTER_INPUT, 33.4f, jy);
  p.input(M::DEJA_VU_INPUT, 45.6f, jy);
  p.input(M::X_STEPS_INPUT, 57.8f, jy);
  p.input(M::X_SPREAD_INPUT, 69.9f, jy);
  p.input(M::X_CLOCK_INPUT, 81.9f, jy);
  p.line(22.6f, 50.2f, 21.3f, 92.4f, 0.3f);
  p.line(68.6f, 50.2f, 69.9f, 92.4f, 0.3f);
  p.line(32.2f, 85.2f, 33.4f, 92.4f, 0.3f);
  p.line(59.0f, 85.2f, 57.8f, 92.4f, 0.3f);

  // Outputs.
  p.box(3.9f, 102.4f, 83.6f, 14.8f, dark, 1.4f);
  const struct { int out, light; bool bicolor; const char* name; } outs[] = {
      {M::T1_OUTPUT, M::T1_LIGHT, false, "t1"}, {M::T2_OUTPUT, M::T2_LIGHT, false, "t2"},
      {M::T3_OUTPUT, M::T3_LIGHT, false, "t3"}, {M::Y_OUTPUT, M::Y_LIGHT, true, "Y"},
      {M::X1_OUTPUT, M::X1_LIGHT, true, "X1"},  {M::X2_OUTPUT, M::X2_LIGHT, true, "X2"},
      {M::X3_OUTPUT, M::X3_LIGHT, true, "X3"}};
  const float ox[7] = {9.4f, 21.3f, 33.4f, 45.6f, 57.8f, 69.9f, 81.9f};
  for (int i = 0; i < 7; ++i) {
    p.output(outs[i].out, ox[i], 111.2f);
    if (outs[i].bicolor) {
      p.bicolorLight(outs[i].light, outs[i].light + 1, ox[i] - 3.0f, 105.4f, 0xFF35E06A,
                     0xFFFF3B30, 2.0f);
    } else {
      p.light(outs[i].light, ox[i] - 3.0f, 105.4f, 0xFF35E06A, 2.0f);
    }
    p.label(ox[i] + 1.0f, 105.4f, outs[i].name, 2.9f, kAlignCenter, white);
  }

  // Readout of the current t mode, X mode / scale (handy without the icons).
  p.display(3.f, 9.8f, 85.4f, 3.6f, [](Canvas& c, float x, float y, float w, float h, float s,
                                       const Module* m) {
    const RandomSampler* r = static_cast<const RandomSampler*>(m);
    float size = std::max(12.f, 2.5f * s);
    const uint32_t col = 0xFF168A8C;
    c.text(x + 3.0f * s, y + h * 0.5f, kTModelShort[r->tModel()], size, col, kAlignLeft);
    std::string right = r->uiMode() == UI_MODE_RECORD_SCALE
                            ? std::string("REC SCALE ") + std::to_string(r->scale() + 1)
                            : std::string(kXModeShort[r->xMode()]) + " / " +
                                  kScaleNames[r->scale()];
    if (r->uiMode() == UI_MODE_SELECT_SCALE) right = std::string("SCALE: ") + kScaleNames[r->scale()];
    c.text(x + w - 3.0f * s, y + h * 0.5f, right, size, col, kAlignRight);
  });
}

Model model = [] {
  Model m;
  m.slug = "random-sampler";
  m.name = "Random Sampler";
  m.shortName = "RANDOM SAMPLER";
  m.basedOn = "Marbles";
  m.category = Category::Modulator;
  m.description = "Random gates (t) and quantized random voltages (X, Y) with deja vu loops.";
  m.create = [] { return new RandomSampler(); };
  m.buildPanel = buildPanel;
  return m;
}();

BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
