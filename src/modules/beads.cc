// BrickRack - GPL-3.0-or-later.
//
// Texture Synthesizer 2: wraps the Beads granular processor and its
// wavetable synth (beads/dsp/processor.cc, beads/dsp/synth.cc) as beads.cc
// drives them:
//  * the codec rate follows the MEDIA setting (processor.sample_rate():
//    24, 48, 32 or 48 kHz) with blocks of rate / 4000 samples, and the
//    module switches to the 48 kHz wavetable synth after 10 seconds without
//    anything patched in the audio inputs. One BlockAdapter per rate.
//  * CVs and pots are read once per block through copies of CvReader /
//    PotChannel (same laws, filters and calibration scalings).
//  * GainControl (the input AGC) is reproduced here, since the firmware's
//    version is tied to the codec driver's frame type.
//  * The buttons and LEDs follow beads/ui.cc. Button combos (e.g. holding
//    MEDIA while turning FEEDBACK to set a manual gain) cannot be performed
//    on a handheld; those hidden settings are in the module menu instead.

#include <algorithm>
#include <cmath>
#include <cstring>

#include "beads/dsp/processor.h"
#include "beads/dsp/synth.h"
#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"
#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/hysteresis_filter.h"
#include "stmlib/dsp/parameter_interpolator.h"
#include "stmlib/utils/gate_flags.h"

namespace br {
namespace {

using beads::FloatFrame;

const int kBlockRate = 4000;            // beads.cc
const int kMaxBlockSize = 12;           // 48 kHz / 4000
const float kSynthModeDelayMs = 10000.f;

const char* kMediaNames[4] = {"Dirty tape (24 kHz, 8-bit mu-law)", "Clean tape (12-bit)",
                              "Vintage digital (32 kHz, 12-bit)", "Digital (48 kHz, 16-bit)"};
const char* kAssignNames[4] = {"Feedback", "Dry/wet", "Reverb", "All (custom amounts)"};

// beads/gain_control.cc, operating on floats instead of codec frames.
class GainControl {
 public:
  enum SoftClippingMode { SOFT_CLIPPING_MODE_OFF, SOFT_CLIPPING_MODE_ON, SOFT_CLIPPING_MODE_DYNAMIC };

  void Init(float sampleRate, SoftClippingMode mode, bool reset) {
    if (reset) {
      stableAgcPeak_ = agcPeak_ = peak_ = 0.5f;
      gain_ = 1.0f;
      softClippingAmount_ = 0.0f;
    }
    softClippingMode_ = mode;
    ignoreTime_ = static_cast<size_t>(sampleRate * kIgnoreTime);
    bool noReadjust = !reset && time_ && time_ >= activeTime_;
    activeTime_ = static_cast<size_t>(sampleRate * (kIgnoreTime + kActiveTime));
    time_ = noReadjust ? activeTime_ : 0;
    oneHertz_ = 1.0f / sampleRate;
  }

  void Restart() { time_ = ignoreTime_; }

  void Process(bool active, uint8_t gainSetting, bool leftNormalized, bool rightNormalized,
               const float* in, FloatFrame* out, size_t size) {
    if (leftNormalized && rightNormalized) {
      for (size_t i = 0; i < size; ++i) out[i].l = out[i].r = 0.0f;
      peak_ = 0.0f;
      agcPeak_ = stableAgcPeak_ = 0.5f;
      gain_ = 1.0f;
      time_ = 0;
      return;
    }
    const float hz = static_cast<float>(size) * oneHertz_;
    if (time_ < ignoreTime_) {
      agcPeak_ = 0.5f;
    } else if (time_ < activeTime_) {
      SLOPE(agcPeak_, peak_, 10.0f * hz, 2.0f * hz);
      stableAgcPeak_ = agcPeak_;
    } else {
      SLOPE(agcPeak_, peak_, 5.0f * hz, (peak_ > stableAgcPeak_ ? 0.5f : 0.01f) * hz);
    }
    if (active) {
      if (time_ > ignoreTime_ || peak_ > 0.001f) time_ += size;
      if (time_ >= activeTime_) time_ = activeTime_;
    }

    autoGain_ = gainSetting == 0;
    float manualGain = static_cast<float>(gainSetting) / 255.0f;
    manualGain *= manualGain;
    manualGain *= manualGain;
    manualGain = kMinGain + (kMaxGain - kMinGain) * manualGain;
    ONE_POLE(manualGain_, manualGain, 0.01f);

    const float optimalGain = std::min(0.9f / std::min(agcPeak_, 0.9f), kMaxGain);
    const float gain = autoGain_ ? optimalGain : manualGain_;
    float softClippingAmount;
    if (softClippingMode_ == SOFT_CLIPPING_MODE_OFF) {
      softClippingAmount = 0.0f;
    } else if (softClippingMode_ == SOFT_CLIPPING_MODE_ON) {
      softClippingAmount = 1.0f;
    } else {
      softClippingAmount = clampf(2.0f * (manualGain_ / optimalGain - 0.75f), 0.f, 1.f);
    }

    stmlib::ParameterInterpolator gainModulation(&gain_, gain, size);
    stmlib::ParameterInterpolator softClippingModulation(&softClippingAmount_, softClippingAmount, size);
    for (size_t i = 0; i < size; ++i) {
      float l = leftNormalized ? in[2 * i + 1] : in[2 * i];
      float r = rightNormalized ? in[2 * i] : in[2 * i + 1];
      if (active) {
        const float peak = std::max(std::fabs(l), std::fabs(r));
        SLOPE(peak_, peak, 0.1f, 2.0f * oneHertz_);
      }
      const float g = gainModulation.Next();
      l *= g;
      r *= g;
      if (!autoGain_) {
        if (softClippingMode_ == SOFT_CLIPPING_MODE_OFF) {
          l = clampf(l, -1.f, 1.f);
          r = clampf(r, -1.f, 1.f);
        } else {
          const float amount = softClippingModulation.Next();
          const float bias = 0.05f;
          const float softL = (1.0f - bias) * stmlib::SoftClip(l + bias) - bias;
          const float softR = (1.0f - bias) * stmlib::SoftClip(r + bias) - bias;
          l = stmlib::Crossfade(clampf(l, -1.f, 1.f), softL, amount);
          r = stmlib::Crossfade(clampf(r, -1.f, 1.f), softR, amount);
        }
      }
      out[i].l = l;
      out[i].r = r;
    }
  }

  uint8_t vuMeter() const {
    float intensity = peak_ * gain_;
    return intensity > 1.0f ? 255 : static_cast<uint8_t>(192.0f * intensity * intensity);
  }
  bool adjusting() const { return time_ < activeTime_ && time_ > 0 && autoGain_; }

 private:
  static constexpr float kIgnoreTime = 1.0f, kActiveTime = 5.0f;
  static constexpr float kMinGain = 1.0f, kMaxGain = 40.0f;
  float peak_ = 0.5f, agcPeak_ = 0.5f, stableAgcPeak_ = 0.5f, gain_ = 1.f;
  float softClippingAmount_ = 0.f;
  SoftClippingMode softClippingMode_ = SOFT_CLIPPING_MODE_OFF;
  bool autoGain_ = true;
  float manualGain_ = 1.f;
  float oneHertz_ = 1.f / 48000.f;
  size_t time_ = 0, ignoreTime_ = 0, activeTime_ = 0;
};

// beads/cv_reader.h PotChannel.
class PotChannel {
 public:
  enum Law { LAW_NORMAL, LAW_ATTENUVERTER, LAW_DETUNE };
  void Init(Law law, float lp, float hysteresis) {
    law_ = law;
    lpCoefficient_ = lp;
    lp_ = 0.f;
    hysteresis_.Init(hysteresis);
  }
  void Process(float v) {  // v: 0..1 pot position
    if (law_ == LAW_ATTENUVERTER) {
      v -= 0.5f;
      v = (2.0f / 0.47f) * v * std::max(std::fabs(v) - 0.03f, 0.0f);
    } else if (law_ == LAW_DETUNE) {
      float detune = 2.0f * v - 1.0f;
      float sign = detune < 0.0f ? -1.0f : 1.0f;
      detune = detune * sign * 6.0f;
      MAKE_INTEGRAL_FRACTIONAL(detune);
      const float intervals[8] = {0.0f, 1.0f, 3.0f, 4.0f, 7.0f, 12.0f, 24.0f, 24.0f};
      detune_fractional = std::max((detune_fractional - 0.33333333f) * 1.5f, 0.0f);
      float a = intervals[detune_integral];
      float b = intervals[detune_integral + 1];
      v = stmlib::Crossfade(a, b, stmlib::SmoothStep(detune_fractional)) * sign;
    }
    ONE_POLE(lp_, v, lpCoefficient_);
    hysteresis_.Process(lp_);
  }
  float value() const { return hysteresis_.value(); }

 private:
  Law law_ = LAW_NORMAL;
  float lp_ = 0.f, lpCoefficient_ = 0.05f;
  stmlib::HysteresisFilter hysteresis_;
};

// beads/cv_reader.h CvChannel, fed with the already calibrated value.
struct CvChannel {
  float lp = 0.f, coefficient = 1.f, lo = -1.f, hi = 1.f, value = 0.f;
  void Init(float c, float l, float h) { coefficient = c; lo = l; hi = h; lp = value = 0.f; }
  void Process(float v) {
    ONE_POLE(lp, v, coefficient);
    value = clampf(lp, lo, hi);
  }
};

class TextureSynth2 : public Module {
 public:
  enum ParamIds {
    DENSITY_PARAM,
    TIME_PARAM,
    PITCH_PARAM,
    SIZE_PARAM,
    SHAPE_PARAM,
    FEEDBACK_PARAM,
    DRY_WET_PARAM,
    REVERB_PARAM,
    TIME_ATT_PARAM,
    SIZE_ATT_PARAM,
    SHAPE_ATT_PARAM,
    PITCH_ATT_PARAM,
    FREEZE_BUTTON_PARAM,
    SEED_BUTTON_PARAM,
    MEDIA_BUTTON_PARAM,
    ASSIGN_BUTTON_PARAM,
    // Hidden: the firmware's persistent state (settings.h) and seed lock.
    MEDIA_PARAM,
    ASSIGN_DEST_PARAM,
    ASSIGN_AMOUNT1_PARAM,
    ASSIGN_AMOUNT2_PARAM,
    ASSIGN_AMOUNT3_PARAM,
    GAIN_PARAM,
    GATE_OUTPUT_PARAM,
    SEED_LOCK_PARAM,
    NUM_PARAMS
  };
  enum InputIds {
    ASSIGN_INPUT,
    TIME_INPUT,
    SIZE_INPUT,
    SHAPE_INPUT,
    PITCH_INPUT,
    DENSITY_INPUT,
    IN_L_INPUT,
    IN_R_INPUT,
    FREEZE_INPUT,
    SEED_INPUT,
    NUM_INPUTS
  };
  enum OutputIds { OUT_L_OUTPUT, OUT_R_OUTPUT, NUM_OUTPUTS };
  enum LightIds {
    MEDIA_R_LIGHT,
    MEDIA_G_LIGHT,
    MEDIA_B_LIGHT,
    FREEZE_LIGHT,
    SEED_LIGHT,
    VU_LIGHT,
    ASSIGN1_LIGHT,
    ASSIGN2_LIGHT,
    ASSIGN3_LIGHT,
    NUM_LIGHTS
  };

  TextureSynth2() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(DENSITY_PARAM, 0.f, 1.f, 0.5f, "Density", "%", 100.f);
    configParam(TIME_PARAM, 0.f, 1.f, 0.5f, "Time", "%", 100.f);
    configParam(PITCH_PARAM, 0.f, 1.f, 0.5f, "Pitch (detune)", "%", 100.f);
    configParam(SIZE_PARAM, 0.f, 1.f, 0.5f, "Size", "%", 100.f);
    configParam(SHAPE_PARAM, 0.f, 1.f, 0.5f, "Shape", "%", 100.f);
    configParam(FEEDBACK_PARAM, 0.f, 1.f, 0.f, "Feedback", "%", 100.f);
    configParam(DRY_WET_PARAM, 0.f, 1.f, 0.5f, "Dry/wet", "%", 100.f);
    configParam(REVERB_PARAM, 0.f, 1.f, 0.f, "Reverb", "%", 100.f);
    configParam(TIME_ATT_PARAM, -1.f, 1.f, 0.f, "Time CV / random amount", "%", 100.f);
    configParam(SIZE_ATT_PARAM, -1.f, 1.f, 0.f, "Size CV / random amount", "%", 100.f);
    configParam(SHAPE_ATT_PARAM, -1.f, 1.f, 0.f, "Shape CV / random amount", "%", 100.f);
    configParam(PITCH_ATT_PARAM, -1.f, 1.f, 0.f, "Pitch CV / random amount", "%", 100.f);
    configButton(FREEZE_BUTTON_PARAM, "Freeze");
    configButton(SEED_BUTTON_PARAM, "Seed (hold 4 s: free-running grains)");
    configButton(MEDIA_BUTTON_PARAM, "Quality / media (hold 1.5 s: auto gain)");
    configButton(ASSIGN_BUTTON_PARAM, "Assign CV destination");
    // Defaults from beads/settings.cc.
    configSwitch(MEDIA_PARAM, 0.f, 3.f, 3.f, "Quality / media",
                 {kMediaNames[0], kMediaNames[1], kMediaNames[2], kMediaNames[3]})
        .hidden = true;
    configSwitch(ASSIGN_DEST_PARAM, 0.f, 3.f, 0.f, "Assign CV destination",
                 {kAssignNames[0], kAssignNames[1], kAssignNames[2], kAssignNames[3]})
        .hidden = true;
    configParam(ASSIGN_AMOUNT1_PARAM, 0.f, 1.f, 255.f / 256.f, "Assign amount: feedback", "%", 100.f)
        .hidden = true;
    configParam(ASSIGN_AMOUNT2_PARAM, 0.f, 1.f, 0.f, "Assign amount: dry/wet", "%", 100.f).hidden = true;
    configParam(ASSIGN_AMOUNT3_PARAM, 0.f, 1.f, 0.5f, "Assign amount: reverb", "%", 100.f).hidden = true;
    {
      Param& g = configParam(GAIN_PARAM, 0.f, 255.f, 0.f, "Input gain (0 = automatic)");
      g.snap = true;
      g.hidden = true;
    }
    configSwitch(GATE_OUTPUT_PARAM, 0.f, 1.f, 0.f, "Right output = grain gates", {"Off", "On"})
        .hidden = true;
    configSwitch(SEED_LOCK_PARAM, 0.f, 1.f, 1.f, "Seed locked (free-running grains)", {"Off", "On"})
        .hidden = true;

    configInput(ASSIGN_INPUT, "Assignable CV");
    configInput(TIME_INPUT, "Time");
    configInput(SIZE_INPUT, "Size");
    configInput(SHAPE_INPUT, "Shape");
    configInput(PITCH_INPUT, "V/Oct");
    configInput(DENSITY_INPUT, "Density");
    configInput(IN_L_INPUT, "In L");
    configInput(IN_R_INPUT, "In R");
    configInput(FREEZE_INPUT, "Freeze");
    configInput(SEED_INPUT, "Seed");
    configOutput(OUT_L_OUTPUT, "Out L");
    configOutput(OUT_R_OUTPUT, "Out R");

    // Same memory layout as on the hardware.
    std::memset(delayBuffer1_, 0, sizeof(delayBuffer1_));
    std::memset(delayBuffer2_, 0, sizeof(delayBuffer2_));
    std::memset(reverbBuffer_, 0, sizeof(reverbBuffer_));
    uint8_t* buffers[3] = {delayBuffer1_, delayBuffer1_ + 256 * 1024, delayBuffer2_};
    // The firmware's DSP objects are zero-initialised globals: do the same
    // (Init() doesn't set every member).
    std::memset(static_cast<void*>(&processor_), 0, sizeof(processor_));
    std::memset(static_cast<void*>(&synth_), 0, sizeof(synth_));
    processor_.Init(buffers, reverbBuffer_);
    synth_.Init(reverbBuffer_);

    // CvReader::Init.
    for (int i = 0; i < kNumPots; ++i) {
      PotChannel::Law law = PotChannel::LAW_NORMAL;
      if (i >= POT_TIME_AMOUNT && i <= POT_PITCH_AMOUNT) law = PotChannel::LAW_ATTENUVERTER;
      else if (i == POT_PITCH) law = PotChannel::LAW_DETUNE;
      pots_[i].Init(law, 0.05f, i == POT_TIME ? 0.001f : 0.0f);
    }
    cvs_[CV_ASSIGN].Init(0.1f, -1.f, 1.f);
    cvs_[CV_DENSITY].Init(1.0f, -1.f, 1.f);
    cvs_[CV_PITCH].Init(1.0f, -60.f, 60.f);
    cvs_[CV_TIME].Init(0.5f, -1.6f, 1.6f);
    cvs_[CV_SIZE].Init(0.5f, -1.6f, 1.6f);
    cvs_[CV_SHAPE].Init(0.5f, -1.6f, 1.6f);

    gainControl_.Init(48000.f, GainControl::SOFT_CLIPPING_MODE_OFF, true);
    initAdapters(48000.f);
    // At power-on the firmware starts in processor mode (beads.cc Init +
    // MonitorStateChange) and falls back to the synth after 10 s of silence.
    synthMode_ = false;
    configureProcessor(true);
  }

  void onSampleRateChange(float sr) override {
    initAdapters(sr);
    startCodec(true);
  }

  void onReset() override {
    freezeButton_ = false;
    seedButton_ = false;
  }

  void process(const ProcessArgs& args) override {
    nowMs_ += args.sampleTime * 1000.0;
    handleButtons();

    float in[4] = {inputs[IN_L_INPUT].getVoltage() / 5.f, inputs[IN_R_INPUT].getVoltage() / 5.f,
                   inputs[FREEZE_INPUT].getVoltage(), inputs[SEED_INPUT].getVoltage()};
    float out[2];
    adapter_[adapterIndex_].process(in, out, [&](const float* nin, float* nout, int n) {
      renderBlock(nin, nout, n);
    });
    outputs[OUT_L_OUTPUT].setVoltage(out[0] * 5.f);
    outputs[OUT_R_OUTPUT].setVoltage(out[1] * 5.f);

    monitorStateChange(args.sampleTime * 1000.f);
    if ((args.frame & 63) == 0) updateLights();
  }

 private:
  enum Pot {
    POT_FEEDBACK, POT_DRY_WET, POT_REVERB, POT_TIME_AMOUNT, POT_SIZE_AMOUNT, POT_SHAPE_AMOUNT,
    POT_PITCH_AMOUNT, POT_TIME, POT_DENSITY, POT_PITCH, POT_SIZE, POT_SHAPE, kNumPots
  };
  enum Cv { CV_ASSIGN, CV_DENSITY, CV_PITCH, CV_TIME, CV_SIZE, CV_SHAPE, kNumCvs };
  enum Switch { SWITCH_FREEZE, SWITCH_SEED, SWITCH_MEDIA, SWITCH_ASSIGN, kNumSwitches };

  int intParam(int id) const { return static_cast<int>(params[static_cast<size_t>(id)].value + 0.5f); }

  // Adapters for the three codec rates: 24 kHz / 6, 32 kHz / 8, 48 kHz / 12.
  static int adapterFor(float sr) { return sr < 28000.f ? 0 : (sr < 40000.f ? 1 : 2); }
  void initAdapters(float engineRate) {
    engineRate_ = engineRate;
    const float rates[3] = {24000.f, 32000.f, 48000.f};
    for (int i = 0; i < 3; ++i)
      adapter_[i].init(engineRate, rates[i], static_cast<int>(rates[i]) / kBlockRate);
  }

  // ConfigureAndStartCodec(). Re-initialising an adapter at an unchanged size
  // does not allocate (vector::assign reuses its storage).
  void startCodec(bool resetGainControl) {
    float sr = synthMode_ ? 48000.f : processor_.sample_rate();
    beads::Processor::Media media = processor_.media();
    gainControl_.Init(sr,
                      media == beads::Processor::MEDIA_DIRTY_TAPE
                          ? GainControl::SOFT_CLIPPING_MODE_ON
                          : (media == beads::Processor::MEDIA_CLEAN_TAPE
                                 ? GainControl::SOFT_CLIPPING_MODE_DYNAMIC
                                 : GainControl::SOFT_CLIPPING_MODE_OFF),
                      resetGainControl);
    eventDelayPos_ = 0;
    std::memset(eventDelay_, 0, sizeof(eventDelay_));
    transitionFrames_ = kBlockRate / 4;
    synthModeStartMs_ = 0.0;
    adapterIndex_ = adapterFor(sr);
    adapter_[adapterIndex_].init(engineRate_, sr, static_cast<int>(sr) / kBlockRate);
  }

  void configureProcessor(bool resetGainControl) {
    auto media = static_cast<beads::Processor::Media>(intParam(MEDIA_PARAM) & 3);
    bool mono = !inputs[IN_L_INPUT].isConnected() || !inputs[IN_R_INPUT].isConnected();
    processor_.Configure(media, mono);
    startCodec(resetGainControl);
  }

  // MonitorStateChange() from beads.cc.
  void monitorStateChange(float ms) {
    if (freezeLed_ || transitionFrames_) {
      synthModeStartMs_ = 0.0;
      return;
    }
    bool noInput = !inputs[IN_L_INPUT].isConnected() && !inputs[IN_R_INPUT].isConnected();
    if (noInput) {
      if (!synthMode_) {
        synthModeStartMs_ += ms;
        if (synthModeStartMs_ >= kSynthModeDelayMs) {
          synthMode_ = true;
          startCodec(true);
        }
      }
    } else {
      auto media = static_cast<beads::Processor::Media>(intParam(MEDIA_PARAM) & 3);
      bool mono = !inputs[IN_L_INPUT].isConnected() || !inputs[IN_R_INPUT].isConnected();
      bool monoChanged = mono != processor_.mono();
      if (synthMode_ || monoChanged || media != processor_.media()) {
        bool reset = monoChanged || synthMode_;
        synthMode_ = false;
        configureProcessor(reset);
      }
      synthModeStartMs_ = 0.0;
    }
  }

  // Ui::Poll / OnSwitchPressed / OnSwitchReleased for single presses.
  void handleButtons() {
    static const float kLongPress[kNumSwitches] = {4000.f, 4000.f, 1500.f, 1e30f};
    for (int s = 0; s < kNumSwitches; ++s) {
      bool pressed = params[static_cast<size_t>(FREEZE_BUTTON_PARAM + s)].value > 0.5f;
      if (pressed && !down_[s]) {
        pressStart_[s] = nowMs_;
        ignoreRelease_[s] = false;
        onPressed(s);
      }
      double held = nowMs_ - pressStart_[s];
      if (pressed && !ignoreRelease_[s] && held >= kLongPress[s]) {
        onReleased(s, held);
        ignoreRelease_[s] = true;
      }
      if (!pressed && down_[s] && !ignoreRelease_[s]) {
        onReleased(s, held + 1.0);
        ignoreRelease_[s] = true;
      }
      down_[s] = pressed;
    }
  }

  void onPressed(int s) {
    if (s == SWITCH_FREEZE) {
      freezeButton_ = !freezeButton_;
    } else if (s == SWITCH_SEED) {
      params[SEED_LOCK_PARAM].set(0.f);
      seedButton_ = true;
    }
  }

  void onReleased(int s, double duration) {
    switch (s) {
      case SWITCH_SEED:
        if (duration >= 4000.0) params[SEED_LOCK_PARAM].set(1.f);
        seedButton_ = false;
        break;
      case SWITCH_MEDIA:
        if (duration >= 1500.0) {
          params[GAIN_PARAM].set(0.f);
          gainControl_.Restart();
        } else {
          params[MEDIA_PARAM].set(static_cast<float>((intParam(MEDIA_PARAM) + 3) % 4));
        }
        break;
      case SWITCH_ASSIGN: {
        int d = intParam(ASSIGN_DEST_PARAM);
        params[ASSIGN_DEST_PARAM].set(static_cast<float>(d == 3 ? 0 : (d + 1) % 3));
        break;
      }
      default:
        break;
    }
  }

  // CvReader::Read(). Calibrated CV scalings (settings.cc): PITCH 12
  // semitones/V; the other CVs 0.2/V (+/-5 V for DENSITY and ASSIGN, +/-8 V
  // for TIME/SIZE/SHAPE, which clamp at +/-1.6).
  void readCv() {
    auto pot = [&](int id) { return params[static_cast<size_t>(id)].value; };
    auto att = [&](int id) { return (params[static_cast<size_t>(id)].value + 1.f) * 0.5f; };
    const float raw[kNumPots] = {pot(FEEDBACK_PARAM), pot(DRY_WET_PARAM), pot(REVERB_PARAM),
                                 att(TIME_ATT_PARAM), att(SIZE_ATT_PARAM), att(SHAPE_ATT_PARAM),
                                 att(PITCH_ATT_PARAM), pot(TIME_PARAM), pot(DENSITY_PARAM),
                                 pot(PITCH_PARAM), pot(SIZE_PARAM), pot(SHAPE_PARAM)};
    for (int i = 0; i < kNumPots; ++i) pots_[i].Process(raw[i]);
    auto v = [&](int id) { return inputs[static_cast<size_t>(id)].getVoltage(); };
    cvs_[CV_ASSIGN].Process(v(ASSIGN_INPUT) * 0.2f);
    cvs_[CV_DENSITY].Process(v(DENSITY_INPUT) * 0.2f);
    cvs_[CV_PITCH].Process(v(PITCH_INPUT) * 12.f);
    cvs_[CV_TIME].Process(v(TIME_INPUT) * 0.2f);
    cvs_[CV_SIZE].Process(v(SIZE_INPUT) * 0.2f);
    cvs_[CV_SHAPE].Process(v(SHAPE_INPUT) * 0.2f);
  }

  // Process() from beads.cc, for one codec block.
  void renderBlock(const float* nin, float* nout, int n) {
    const size_t size = static_cast<size_t>(n);
    const bool leftNormalized = !inputs[IN_L_INPUT].isConnected();
    const bool rightNormalized = !inputs[IN_R_INPUT].isConnected();

    readCv();

    bool freeze[kMaxBlockSize];
    bool freezeBlock = false;
    for (size_t i = 0; i < size; ++i) {
      freezeGate_.process(nin[i * 4 + 2]);
      freeze[i] = freezeButton_ || freezeGate_.isHigh();
      freezeBlock |= freeze[i];
    }
    freezeLed_ = freezeBlock;

    stmlib::GateFlags seed[kMaxBlockSize];
    stmlib::GateFlags* seedInput = nullptr;
    bool seedBlock = false;
    if (inputs[SEED_INPUT].isConnected()) {
      for (size_t i = 0; i < size; ++i) {
        seedGate_.process(nin[i * 4 + 3]);
        // Delay the seed input by 2.5 ms (10 blocks), as the firmware does.
        eventDelay_[eventDelayPos_] = seedGate_.isHigh();
        // stmlib::DelayLine::Read(size * 10) right after Write().
        bool hasSeed = eventDelay_[(eventDelayPos_ + kEventDelaySize - (size * 10 - 1)) % kEventDelaySize];
        eventDelayPos_ = (eventDelayPos_ + 1) % kEventDelaySize;
        previousSeed_ = seed[i] = stmlib::ExtractGateFlags(previousSeed_, hasSeed);
        seedBlock |= hasSeed;
      }
      seedInput = seed;
    } else {
      previousSeed_ = stmlib::GATE_FLAG_LOW;
    }
    seedLed_ = seedBlock && !transitionFrames_;
    overwriteReserve_ = synthMode_ ? 1.0f : processor_.overwrite_reserve();

    FloatFrame floatIn[kMaxBlockSize], floatOut[kMaxBlockSize];
    float audioIn[2 * kMaxBlockSize];
    for (size_t i = 0; i < size; ++i) {  // codec clipping
      audioIn[2 * i] = clampf(nin[i * 4], -1.f, 1.f);
      audioIn[2 * i + 1] = clampf(nin[i * 4 + 1], -1.f, 1.f);
    }
    gainControl_.Process(!transitionFrames_, static_cast<uint8_t>(intParam(GAIN_PARAM)),
                         leftNormalized, rightNormalized, audioIn, floatIn, size);
    if (transitionFrames_) {
      --transitionFrames_;
      for (size_t i = 0; i < 2 * size; ++i) nout[i] = 0.f;
      return;
    }

    beads::Parameters p;
    p.time.base = pots_[POT_TIME].value();
    p.time.attenuverter = pots_[POT_TIME_AMOUNT].value();
    p.time.has_cv = inputs[TIME_INPUT].isConnected();
    p.time.cv = cvs_[CV_TIME].value;
    p.size.base = pots_[POT_SIZE].value();
    p.size.attenuverter = pots_[POT_SIZE_AMOUNT].value();
    p.size.has_cv = inputs[SIZE_INPUT].isConnected();
    p.size.cv = cvs_[CV_SIZE].value;
    p.shape.base = pots_[POT_SHAPE].value();
    p.shape.attenuverter = pots_[POT_SHAPE_AMOUNT].value();
    p.shape.has_cv = inputs[SHAPE_INPUT].isConnected();
    p.shape.cv = cvs_[CV_SHAPE].value;
    p.pitch.base = pots_[POT_PITCH].value();
    p.pitch.attenuverter = pots_[POT_PITCH_AMOUNT].value();
    p.pitch.has_cv = inputs[PITCH_INPUT].isConnected();
    p.pitch.cv = cvs_[CV_PITCH].value;
    p.monophonic_mode = false;  // ENABLE_ADVANCED_FEATURES is off in the firmware

    beads::ModulatedParameter d;
    d.base = pots_[POT_DENSITY].value();
    d.cv = inputs[DENSITY_INPUT].isConnected() ? cvs_[CV_DENSITY].value : 0.0f;

    const int destination = intParam(ASSIGN_DEST_PARAM) & 3;
    float mix[3];
    for (int i = 0; i < 3; ++i) {
      float base = pots_[POT_FEEDBACK + i].value() * 1.005f;
      float amount = destination == 3
                         ? std::min(params[static_cast<size_t>(ASSIGN_AMOUNT1_PARAM + i)].value, 255.f / 256.f)
                         : (destination == i ? 1.0f : 0.0f);
      mix[i] = clampf(base + (cvs_[CV_ASSIGN].value - 0.002f) * amount, 0.f, 1.f);
    }
    beads::Processor::MixParameters m;
    m.feedback = mix[0];
    m.dry_wet = mix[1];
    m.reverb_amount = mix[2];
    m.output_gate = intParam(GATE_OUTPUT_PARAM) != 0;

    beads::SeedButtonState seedState = intParam(SEED_LOCK_PARAM)
                                           ? beads::SEED_BUTTON_STATE_LOCKED
                                           : (seedButton_ ? beads::SEED_BUTTON_STATE_ON
                                                          : beads::SEED_BUTTON_STATE_OFF);
    if (synthMode_) {
      synth_.Render(seedState, seedInput, freeze,
                    static_cast<beads::Synth::Resolution>(intParam(MEDIA_PARAM) & 3), d, p, m,
                    floatOut, size);
    } else {
      processor_.Process(seedState, seedInput, freeze, d, p, m, floatIn, floatOut, size);
    }

    for (size_t i = 0; i < size; ++i) {
      float l = clampf(floatOut[i].l, -1.0f, 0.999999f);
      float r = clampf(floatOut[i].r, -1.0f, 0.999999f);
      if (std::fabs(l) > 0.01f || std::fabs(r) > 0.01f) synthModeStartMs_ = 0.0;
      nout[2 * i] = l;
      nout[2 * i + 1] = r;
    }
  }

  // Ui::UpdateLeds() (normal mode).
  void updateLights() {
    uint32_t ms = static_cast<uint32_t>(nowMs_);
    int triangle = (ms & 2047) >> 2;
    triangle = triangle >= 256 ? 511 - triangle : triangle;

    static const uint8_t kPalette[4][3] = {{128, 0, 48}, {255, 144, 0}, {0, 255, 192}, {192, 192, 208}};
    const uint8_t* c = kPalette[intParam(MEDIA_PARAM) & 3];
    float a = overwriteReserve_;
    lights[MEDIA_R_LIGHT] = c[0] / 255.f * a * a;
    lights[MEDIA_G_LIGHT] = c[1] / 255.f * a;
    lights[MEDIA_B_LIGHT] = c[2] / 255.f * a;

    lights[FREEZE_LIGHT] = freezeLed_ ? 1.f : 0.f;
    if (intParam(SEED_LOCK_PARAM)) {
      lights[SEED_LIGHT] = ((triangle * 3 >> 2) + 64) / 255.f;
    } else {
      lights[SEED_LIGHT] = (seedButton_ || seedLed_) ? 1.f : 0.f;
    }
    int destination = intParam(ASSIGN_DEST_PARAM) & 3;
    for (int i = 0; i < 3; ++i) {
      float v = destination == 3
                    ? params[static_cast<size_t>(ASSIGN_AMOUNT1_PARAM + i)].value * 0.5f
                    : (destination == i ? 0.5f : 0.f);
      lights[static_cast<size_t>(ASSIGN1_LIGHT + i)] = v;
    }
    if (gainControl_.adjusting()) {
      lights[VU_LIGHT] = (ms & 128) ? 0.5f : 0.f;
    } else {
      lights[VU_LIGHT] = gainControl_.vuMeter() / 255.f;
    }
  }

  static const size_t kEventDelaySize = 256;

  // Same sizes as in beads.cc: 512 KB + 256 KB delay memory, 64 KB reverb.
  uint8_t delayBuffer1_[512 * 1024];
  uint8_t delayBuffer2_[256 * 1024];
  uint16_t reverbBuffer_[32768];
  beads::Processor processor_;
  beads::Synth synth_;
  GainControl gainControl_;
  PotChannel pots_[kNumPots];
  CvChannel cvs_[kNumCvs];

  BlockAdapter<4, 2> adapter_[3];
  int adapterIndex_ = 2;
  float engineRate_ = 48000.f;
  bool synthMode_ = false;
  double synthModeStartMs_ = 0.0;
  int transitionFrames_ = 0;

  SchmittTrigger freezeGate_, seedGate_;
  bool eventDelay_[kEventDelaySize] = {};
  size_t eventDelayPos_ = 0;
  stmlib::GateFlags previousSeed_ = stmlib::GATE_FLAG_LOW;

  bool freezeButton_ = false, seedButton_ = false;
  bool freezeLed_ = false, seedLed_ = false;
  float overwriteReserve_ = 1.f;

  double nowMs_ = 0.0;
  double pressStart_[kNumSwitches] = {};
  bool down_[kNumSwitches] = {}, ignoreRelease_[kNumSwitches] = {};
};

void buildPanel(PanelSpec& p) {
  // 14 HP. Positions measured on the original panel (mm = px * 0.1285).
  p.hp = 14;
  p.bg = 0xFFF3F1EE;
  p.ink = 0xFF242426;
  p.title = "TEXTURE SYNTH 2";
  const uint32_t teal = 0xFF14958C, ochre = 0xFFD9A030, red = 0xFFC0404B;
  const uint32_t white = 0xFFFFFFFF, grey = 0xFFB4B4B4, dark = 0xFF5E5E60;
  using M = TextureSynth2;

  p.button(M::FREEZE_BUTTON_PARAM, 8.7f, 17.7f, 7.f, "", 0xFFD8D9DB, M::FREEZE_LIGHT);
  p.label(8.7f, 24.0f, "FREEZE", 3.0f);
  p.button(M::SEED_BUTTON_PARAM, 62.1f, 17.7f, 7.f, "", 0xFFD8D9DB, M::SEED_LIGHT);
  p.label(62.1f, 24.0f, "SEED", 3.0f);

  p.knob(M::DENSITY_PARAM, 35.3f, 23.8f, 13.f);
  p.label(35.3f, 37.1f, "DENSITY", 3.2f);
  p.knob(M::TIME_PARAM, 11.6f, 37.9f, 13.f);
  p.label(11.6f, 51.1f, "TIME", 3.2f);
  p.knob(M::PITCH_PARAM, 59.1f, 37.9f, 13.f);
  p.label(59.1f, 51.1f, "PITCH", 3.2f);

  // Quality (media) button, its RGB LED and the four-colour legend.
  p.rgbLight(M::MEDIA_R_LIGHT, M::MEDIA_G_LIGHT, M::MEDIA_B_LIGHT, 29.2f, 45.0f, 2.8f);
  p.button(M::MEDIA_BUTTON_PARAM, 35.3f, 45.0f, 5.2f, "", 0xFFD8D9DB);
  // Top to bottom: digital, vintage digital, clean tape, dirty tape.
  const uint32_t legend[4] = {0xFFB8B8C8, 0xFF00C8A0, 0xFFFF9000, 0xFF800030};
  const float barW[4] = {1.4f, 2.6f, 4.0f, 7.0f};
  for (int i = 0; i < 4; ++i) p.box(39.6f, 42.6f + i * 1.25f, barW[i], 0.9f, legend[i], 0.2f);

  p.knob(M::SIZE_PARAM, 24.4f, 60.4f, 10.f);
  p.label(24.4f, 71.1f, "SIZE", 3.2f);
  p.knob(M::SHAPE_PARAM, 46.5f, 60.4f, 10.f);
  p.label(46.5f, 71.1f, "SHAPE", 3.2f);

  // Mix knobs in their coloured pills, with the assign LEDs.
  p.box(4.4f, 56.6f, 5.4f, 16.0f, teal, 2.7f);
  p.knob(M::FEEDBACK_PARAM, 7.1f, 60.4f, 7.f);
  p.light(M::ASSIGN1_LIGHT, 7.1f, 69.4f, 0xFFFFFFFF, 2.4f);
  p.box(61.6f, 56.6f, 5.4f, 16.0f, ochre, 2.7f);
  p.knob(M::DRY_WET_PARAM, 64.3f, 60.4f, 7.f);
  p.light(M::ASSIGN2_LIGHT, 64.3f, 69.4f, 0xFFFFFFFF, 2.4f);
  p.box(61.6f, 74.4f, 5.4f, 13.4f, red, 2.7f);
  p.knob(M::REVERB_PARAM, 64.3f, 77.6f, 7.f);
  p.light(M::ASSIGN3_LIGHT, 64.3f, 84.4f, 0xFFFFFFFF, 2.4f);

  p.button(M::ASSIGN_BUTTON_PARAM, 7.1f, 77.6f, 5.2f, "", 0xFFD8D9DB);
  p.label(7.1f, 83.4f, "ASSIGN", 2.4f);
  p.box(1.8f, 86.4f, 2.6f, 2.6f, teal, 1.3f);
  p.box(5.8f, 86.4f, 2.6f, 2.6f, ochre, 1.3f);
  p.box(9.8f, 86.4f, 2.6f, 2.6f, red, 1.3f);

  // Attenuverters and CV inputs.
  const float colX[4] = {18.2f, 29.8f, 41.1f, 52.7f};
  const int attIds[4] = {M::TIME_ATT_PARAM, M::SIZE_ATT_PARAM, M::SHAPE_ATT_PARAM, M::PITCH_ATT_PARAM};
  for (int i = 0; i < 4; ++i) {
    p.knob(attIds[i], colX[i], 80.3f, 6.5f);
    p.line(colX[i], 76.2f, colX[i], 74.0f, 0.4f, p.ink);
    p.line(colX[i], 84.0f, colX[i], 92.0f, 0.3f, 0xFF8A8A8A);
  }
  p.line(11.6f, 53.6f, colX[0], 76.4f, 0.3f, 0xFF8A8A8A);  // TIME -> attenuverter
  p.line(59.1f, 53.6f, colX[3], 76.4f, 0.3f, 0xFF8A8A8A);  // PITCH -> attenuverter
  p.input(M::ASSIGN_INPUT, 7.1f, 95.7f);
  p.input(M::TIME_INPUT, colX[0], 95.7f);
  p.input(M::SIZE_INPUT, colX[1], 95.7f);
  p.input(M::SHAPE_INPUT, colX[2], 95.7f);
  p.input(M::PITCH_INPUT, colX[3], 95.7f);
  p.input(M::DENSITY_INPUT, 64.3f, 95.7f);
  p.label(64.3f, 89.6f, "DENSITY", 2.7f);

  // Audio and gate jacks.
  p.input(M::IN_L_INPUT, 7.1f, 110.9f, "IN L");
  p.input(M::IN_R_INPUT, 18.2f, 110.9f, "IN R");
  p.line(10.7f, 110.9f, 14.6f, 110.9f, 0.3f, p.ink);  // R normalled to L
  p.light(M::VU_LIGHT, 12.65f, 104.4f, 0xFF35E06A, 2.2f);
  p.input(M::FREEZE_INPUT, 29.8f, 110.9f, "FREEZE");
  p.input(M::SEED_INPUT, 41.1f, 110.9f, "SEED");
  p.box(47.8f, 102.4f, 20.3f, 14.5f, dark, 1.2f);
  p.output(M::OUT_L_OUTPUT, 52.7f, 111.4f);
  p.output(M::OUT_R_OUTPUT, 64.3f, 111.4f);
  p.label(52.7f, 105.0f, "L", 2.9f, kAlignCenter, white);
  p.label(64.3f, 105.0f, "R", 2.9f, kAlignCenter, white);

  p.line(3.5f, 119.2f, 67.6f, 119.2f, 0.6f, grey);
}

Model model = [] {
  Model m;
  m.slug = "texture-synth-2";
  m.name = "Texture Synthesizer 2";
  m.shortName = "TEXTURE SYNTH 2";
  m.basedOn = "Beads";
  m.category = Category::Effect;
  m.description = "Granular processor with tape/digital media, delay mode and a wavetable synth.";
  m.create = [] { return new TextureSynth2(); };
  m.buildPanel = buildPanel;
  return m;
}();

BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
