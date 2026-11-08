// BrickRack - GPL-3.0-or-later.
//
// Texture Synthesizer: wraps the Clouds granular processor
// (clouds/dsp/granular_processor.cc) exactly as the firmware drives it:
// 32 kHz, 32-sample blocks, Parameters filled once per block by a copy of the
// firmware's CvScaler::Read() (same smoothing coefficients, same blend-knob
// mechanics), Prepare() called after every block as in the firmware's desktop
// test. The MODE button and the 4 bicolour LEDs follow clouds/ui.cc.

#include <algorithm>
#include <cmath>
#include <cstring>

#include "clouds/dsp/granular_processor.h"
#include "clouds/resources.h"
#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"
#include "stmlib/dsp/dsp.h"
#include "ui/canvas.h"

namespace br {
namespace {

const char* kPlaybackNames[4] = {"Granular", "Pitch shifter / time stretcher",
                                 "Looping delay", "Spectral madness"};
const char* kPlaybackShort[4] = {"GRANULAR", "STRETCH", "LOOPING DELAY", "SPECTRAL"};
const char* kQualityNames[4] = {"16-bit stereo", "16-bit mono", "8-bit u-law stereo",
                                "8-bit u-law mono"};
const char* kBlendNames[4] = {"Dry/wet", "Stereo spread", "Feedback", "Reverb"};

const float kNativeRate = 32000.f;
const int kBlockSize = 32;  // codec.Start(32, ...) in clouds.cc
const float kLongPressMs = 1000.f;
const float kVeryLongPressMs = 4000.f;

class TextureSynth : public Module {
 public:
  enum ParamIds {
    POSITION_PARAM,
    SIZE_PARAM,
    PITCH_PARAM,
    IN_GAIN_PARAM,
    DENSITY_PARAM,
    TEXTURE_PARAM,
    BLEND_PARAM,
    FREEZE_PARAM,
    MODE_PARAM,
    // Hidden: the firmware's persistent state (settings.cc).
    BLEND_SELECT_PARAM,
    QUALITY_PARAM,
    PLAYBACK_PARAM,
    DRY_WET_PARAM,
    SPREAD_PARAM,
    FEEDBACK_PARAM,
    REVERB_PARAM,
    NUM_PARAMS
  };
  enum InputIds {
    FREEZE_INPUT,
    TRIG_INPUT,
    POSITION_INPUT,
    SIZE_INPUT,
    PITCH_INPUT,
    BLEND_INPUT,
    IN_L_INPUT,
    IN_R_INPUT,
    DENSITY_INPUT,
    TEXTURE_INPUT,
    NUM_INPUTS
  };
  enum OutputIds { OUT_L_OUTPUT, OUT_R_OUTPUT, NUM_OUTPUTS };
  // LED i: green = 2*i, red = 2*i + 1.
  enum LightIds { FREEZE_LIGHT = 8, ZERO_LIGHT, NUM_LIGHTS };  // ZERO_LIGHT: unused blue channel

  // Firmware UI modes we reproduce (ui.h). Calibration/load/save are not needed.
  enum UiMode { UI_VU_METER, UI_BLEND_METER, UI_QUALITY, UI_BLENDING, UI_PLAYBACK };

  TextureSynth() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(POSITION_PARAM, 0.f, 1.f, 0.5f, "Position", "%", 100.f);
    configParam(SIZE_PARAM, 0.f, 1.f, 0.5f, "Size", "%", 100.f);
    // The pot goes through lut_quantized_pitch (-24..+24 semitones with notches).
    configParam(PITCH_PARAM, 0.f, 1.f, 0.5f, "Pitch", "semitones", 48.f, -24.f)
        .displayPrecision = 1;
    configParam(IN_GAIN_PARAM, 0.f, 1.f, 0.5f, "Input gain", "x", 2.f);
    configParam(DENSITY_PARAM, 0.f, 1.f, 0.5f, "Density", "%", 100.f);
    configParam(TEXTURE_PARAM, 0.f, 1.f, 0.5f, "Texture", "%", 100.f);
    configParam(BLEND_PARAM, 0.f, 1.f, 1.f, "Blend (selected parameter)", "%", 100.f);
    configButton(FREEZE_PARAM, "Freeze");
    configButton(MODE_PARAM, "Blend select (hold 1 s: quality, 4 s: playback mode)");
    configSwitch(BLEND_SELECT_PARAM, 0.f, 3.f, 0.f, "Blend knob controls",
                 {kBlendNames[0], kBlendNames[1], kBlendNames[2], kBlendNames[3]})
        .hidden = true;
    configSwitch(QUALITY_PARAM, 0.f, 3.f, 0.f, "Quality",
                 {kQualityNames[0], kQualityNames[1], kQualityNames[2], kQualityNames[3]})
        .hidden = true;
    configSwitch(PLAYBACK_PARAM, 0.f, 3.f, 0.f, "Playback mode",
                 {kPlaybackNames[0], kPlaybackNames[1], kPlaybackNames[2], kPlaybackNames[3]})
        .hidden = true;
    // Defaults from clouds/settings.cc (blend_value = 255, 128, 0, 0).
    configParam(DRY_WET_PARAM, 0.f, 1.f, 1.f, "Dry/wet", "%", 100.f).hidden = true;
    configParam(SPREAD_PARAM, 0.f, 1.f, 128.f / 255.f, "Stereo spread", "%", 100.f).hidden = true;
    configParam(FEEDBACK_PARAM, 0.f, 1.f, 0.f, "Feedback", "%", 100.f).hidden = true;
    configParam(REVERB_PARAM, 0.f, 1.f, 0.f, "Reverb", "%", 100.f).hidden = true;

    configInput(FREEZE_INPUT, "Freeze");
    configInput(TRIG_INPUT, "Trigger");
    configInput(POSITION_INPUT, "Position");
    configInput(SIZE_INPUT, "Size");
    configInput(PITCH_INPUT, "V/Oct");
    configInput(BLEND_INPUT, "Blend");
    configInput(IN_L_INPUT, "In L");
    configInput(IN_R_INPUT, "In R");
    configInput(DENSITY_INPUT, "Density");
    configInput(TEXTURE_INPUT, "Texture");
    configOutput(OUT_L_OUTPUT, "Out L");
    configOutput(OUT_R_OUTPUT, "Out R");

    std::memset(blockMem_, 0, sizeof(blockMem_));
    std::memset(blockCcm_, 0, sizeof(blockCcm_));
    // The firmware's DSP objects are zero-initialised globals: do the same
    // (Init() doesn't set every member).
    std::memset(static_cast<void*>(&processor_), 0, sizeof(processor_));
    processor_.Init(blockMem_, sizeof(blockMem_), blockCcm_, sizeof(blockCcm_));
    std::memset(processor_.mutable_parameters(), 0, sizeof(clouds::Parameters));
    applySettings();
    processor_.Prepare();
    adapter_.init(48000.f, kNativeRate, kBlockSize);
  }

  void onSampleRateChange(float sr) override { adapter_.init(sr, kNativeRate, kBlockSize); }

  void onReset() override {
    processor_.set_freeze(false);
    blendSync_ = true;
    uiMode_ = UI_VU_METER;
  }
  void loadState(const StateMap&) override { blendSync_ = true; }

  void process(const ProcessArgs& args) override {
    const float ms = args.sampleTime * 1000.f;
    nowMs_ += ms;

    if (freezeButton_.process(params[FREEZE_PARAM].value)) processor_.ToggleFreeze();
    processModeButton(params[MODE_PARAM].value > 0.5f);

    // Gate inputs (gate_input.cc): freeze follows the gate edges, triggers are
    // latched until the next block reads them.
    if (inputs[FREEZE_INPUT].isConnected()) {
      bool was = freezeGate_.isHigh();
      freezeGate_.process(inputs[FREEZE_INPUT].getVoltage());
      if (freezeGate_.isHigh() != was) processor_.set_freeze(freezeGate_.isHigh());
    }
    if (trigIn_.process(inputs[TRIG_INPUT].getVoltage())) trigPending_ = true;

    // IN R is normalled to IN L on the panel.
    const float gain = params[IN_GAIN_PARAM].value * 2.f;
    float l = inputs[IN_L_INPUT].getVoltage();
    float r = inputs[IN_R_INPUT].isConnected() ? inputs[IN_R_INPUT].getVoltage() : l;
    float in[2] = {l * gain / 5.f, r * gain / 5.f};
    float out[2];
    adapter_.process(in, out, [&](const float* nin, float* nout, int n) {
      renderBlock(nin, nout, n);
    });
    outputs[OUT_L_OUTPUT].setVoltage(out[0] * 5.f);
    outputs[OUT_R_OUTPUT].setVoltage(out[1] * 5.f);

    if ((args.frame & 63) == 0) paintLeds();
  }

  const clouds::GranularProcessor& processor() const { return processor_; }

 private:
  int intParam(int id) const { return static_cast<int>(params[static_cast<size_t>(id)].value + 0.5f); }

  // Pushes the hidden "settings" params into the processor (Ui::Init / SaveState).
  void applySettings() {
    int quality = intParam(QUALITY_PARAM) & 3;
    int mode = intParam(PLAYBACK_PARAM) & 3;
    if (quality != processor_.quality()) processor_.set_quality(quality);
    if (mode != processor_.playback_mode())
      processor_.set_playback_mode(static_cast<clouds::PlaybackMode>(mode));
  }

  // Ui::Poll() / OnSwitchReleased() for the MODE switch.
  void processModeButton(bool pressed) {
    if (pressed && !modePressed_) {
      pressStart_ = nowMs_;
      pressArmed_ = longArmed_ = true;
    }
    if (pressed) {
      float held = nowMs_ - pressStart_;
      if (pressArmed_ && held > kLongPressMs) {
        onModeReleased(held);
        pressArmed_ = false;
      }
      if (longArmed_ && held > kVeryLongPressMs) {
        onModeReleased(held);
        longArmed_ = false;
      }
    } else if (modePressed_ && pressArmed_) {
      onModeReleased(nowMs_ - pressStart_ + 1.f);
      pressArmed_ = false;
    }
    modePressed_ = pressed;
  }

  void onModeReleased(float data) {
    lastEventMs_ = nowMs_;
    if (data >= kVeryLongPressMs) {
      uiMode_ = UI_PLAYBACK;
    } else if (data >= kLongPressMs) {
      uiMode_ = uiMode_ == UI_QUALITY ? UI_VU_METER : UI_QUALITY;
    } else if (uiMode_ == UI_VU_METER || uiMode_ == UI_BLEND_METER) {
      uiMode_ = UI_BLENDING;
    } else if (uiMode_ == UI_BLENDING) {
      params[BLEND_SELECT_PARAM].set(static_cast<float>((intParam(BLEND_SELECT_PARAM) + 1) & 3));
    } else if (uiMode_ == UI_QUALITY) {
      params[QUALITY_PARAM].set(static_cast<float>((intParam(QUALITY_PARAM) + 1) & 3));
    } else if (uiMode_ == UI_PLAYBACK) {
      params[PLAYBACK_PARAM].set(static_cast<float>((intParam(PLAYBACK_PARAM) + 1) & 3));
    } else {
      uiMode_ = UI_VU_METER;
    }
  }

  // The panel's BLEND knob edits whichever of the 4 hidden blend values is
  // selected. On hardware the knob "catches" the stored value with a skewed
  // response (CvScaler::UpdateBlendParameters); a virtual knob can simply be
  // moved to the stored value instead, like a motorised pot.
  void updateBlendKnob() {
    int sel = intParam(BLEND_SELECT_PARAM) & 3;
    Param& knob = params[BLEND_PARAM];
    Param& target = params[static_cast<size_t>(DRY_WET_PARAM + sel)];
    if (sel != blendSelected_) {
      blendSelected_ = sel;
      blendSync_ = true;
    }
    if (blendSync_) {
      knob.set(target.value);
      blendSync_ = false;
    } else if (knob.value != lastBlendKnob_) {
      target.set(knob.value);  // knob moved
    } else if (target.value != knob.value) {
      knob.set(target.value);  // edited from the module menu
    }
    lastBlendKnob_ = knob.value;

    // blend_knob_touched_ (0.02 hysteresis) switches the LEDs to the blend meter.
    if (blendQuantized_ < 0.f) blendQuantized_ = knob.value;
    if (std::fabs(knob.value - blendQuantized_) > 0.02f) {
      blendQuantized_ = knob.value;
      if (uiMode_ == UI_VU_METER || uiMode_ == UI_BLEND_METER || uiMode_ == UI_BLENDING) {
        uiMode_ = UI_BLEND_METER;
        lastEventMs_ = nowMs_;
      }
    }
  }

  // Equivalent of CvScaler::Read(): ADC values become knob + CV / 5 V (the
  // range of the Clouds ADCs), smoothed with the firmware's coefficients.
  void readCv(clouds::Parameters* p) {
    auto cv = [&](int id) { return inputs[static_cast<size_t>(id)].getVoltage() / 5.f; };
    const float raw[kNumChannels] = {
        clampf(params[POSITION_PARAM].value + cv(POSITION_INPUT), 0.f, 1.f),
        clampf(params[DENSITY_PARAM].value + cv(DENSITY_INPUT), 0.f, 1.f),
        params[SIZE_PARAM].value,
        cv(SIZE_INPUT),
        params[PITCH_PARAM].value,
        inputs[PITCH_INPUT].getVoltage(),
        cv(BLEND_INPUT),
        params[TEXTURE_PARAM].value,
        cv(TEXTURE_INPUT),
    };
    static const float kCoefficient[kNumChannels] = {0.05f, 0.01f, 0.01f, 0.1f, 0.01f,
                                                     1.0f,  0.2f,  0.01f, 0.01f};
    for (int i = 0; i < kNumChannels; ++i) smoothed_[i] += kCoefficient[i] * (raw[i] - smoothed_[i]);

    p->position = smoothed_[kPosition];
    p->texture = clampf(smoothed_[kTexturePot] + smoothed_[kTextureCv], 0.f, 1.f);
    p->density = clampf(smoothed_[kDensity], 0.f, 1.f);
    // Just below 1 here and for dry/wet and the pitch pot: the firmware's
    // Interpolate(lut, x, N) on an N + 1 entry table reads entry N + 1 at x = 1
    // (lut_grain_size, lut_xfade_in/out, lut_quantized_pitch).
    p->size = clampf(smoothed_[kSizePot] + smoothed_[kSizeCv], 0.f, 0.9999f);

    // Blend values and their CV modulation (UpdateBlendParameters).
    int sel = intParam(BLEND_SELECT_PARAM) & 3;
    for (int i = 0; i < 4; ++i) {
      float v = params[static_cast<size_t>(DRY_WET_PARAM + i)].value;
      blend_[i] += 0.05f * (v - blend_[i]);
      float target = i == sel ? smoothed_[kBlendCv] : 0.f;
      blendMod_[i] += (i == sel ? 0.1f : 0.002f) * (target - blendMod_[i]);
    }
    float dryWet = (blend_[0] + blendMod_[0]) * 1.05f - 0.025f;
    p->dry_wet = clampf(dryWet, 0.f, 0.9999f);
    p->stereo_spread = clampf(blend_[1] + blendMod_[1], 0.f, 1.f);
    p->feedback = clampf(blend_[2] + blendMod_[2], 0.f, 1.f);
    p->reverb = clampf(blend_[3] + blendMod_[3], 0.f, 1.f);

    p->pitch = stmlib::Interpolate(clouds::lut_quantized_pitch,
                                   clampf(smoothed_[kPitchPot], 0.f, 0.9999f), 1024.f);
    float note = smoothed_[kVoct] * 12.f;
    if (std::fabs(note - note_) > 0.5f) {
      note_ = note;
    } else {
      note_ += 0.2f * (note - note_);
    }
    p->pitch = clampf(p->pitch + note_, -48.f, 48.f);

    // The firmware delays triggers by 5 blocks to line them up with the ADC
    // pipeline latency; our CVs have no such latency, so no delay is needed.
    p->trigger = trigPending_;
    p->gate = trigIn_.isHigh();
    trigPending_ = false;
  }

  void renderBlock(const float* nin, float* nout, int n) {
    updateBlendKnob();
    applySettings();
    readCv(processor_.mutable_parameters());

    clouds::ShortFrame input[kBlockSize], output[kBlockSize];
    for (int i = 0; i < n; ++i) {
      input[i].l = static_cast<short>(clampf(nin[2 * i] * 32768.f, -32768.f, 32767.f));
      input[i].r = static_cast<short>(clampf(nin[2 * i + 1] * 32768.f, -32768.f, 32767.f));
    }
    processor_.Process(input, output, static_cast<size_t>(n));
    processor_.Prepare();  // the firmware runs this in its main loop
    meter(processor_.parameters().freeze ? output : input, n);
    for (int i = 0; i < n; ++i) {
      nout[2 * i] = output[i].l / 32768.f;
      nout[2 * i + 1] = output[i].r / 32768.f;
    }
  }

  // clouds/meter.h at 32 kHz.
  void meter(const clouds::ShortFrame* f, int n) {
    const int32_t attack = 32768 * 100 / 32000, release = 32768 * 4 / 32000;
    for (int i = 0; i < n; ++i) {
      int32_t s[2] = {std::abs(static_cast<int32_t>(f[i].l)), std::abs(static_cast<int32_t>(f[i].r))};
      for (int c = 0; c < 2; ++c) {
        int32_t error = s[c] - peak_[c];
        peak_[c] += error * (error > 0 ? attack : release) >> 15;
      }
    }
  }

  void setStatus(int i, int red, int green) {
    lights[static_cast<size_t>(2 * i)] = green / 255.f;
    lights[static_cast<size_t>(2 * i + 1)] = red / 255.f;
  }
  void setIntensity(int i, int value) {  // Leds::set_intensity
    int red = 0, green = 0;
    if (value < 128) {
      green = value << 1;
    } else if (value < 192) {
      green = 255;
      red = (value - 128) << 2;
    } else {
      green = 255 - ((value - 192) << 2);
      red = 255;
    }
    setStatus(i, red, green);
  }
  void paintBar(int32_t db) {  // Leds::PaintBar
    if (db < 0) return;
    db = std::min(db, 32767) << 1;
    if (db >= 49152) {
      setStatus(3, (db - 49152) >> 6, 0);
      setStatus(2, 255, 255);
      setStatus(1, 0, 255);
      setStatus(0, 0, 255);
    } else if (db >= 32768) {
      setStatus(2, (db - 32768) >> 6, (db - 32768) >> 6);
      setStatus(1, 0, 255);
      setStatus(0, 0, 255);
    } else if (db >= 16384) {
      setStatus(1, 0, (db - 16384) >> 6);
      setStatus(0, 0, 255);
    } else {
      setStatus(0, 0, db >> 6);
    }
  }

  // Ui::PaintLeds() and the 3-second timeout of Ui::DoEvents().
  void paintLeds() {
    if (nowMs_ - lastEventMs_ > 3000.f) {
      lastEventMs_ = nowMs_;
      if (uiMode_ != UI_VU_METER && !modePressed_) uiMode_ = UI_VU_METER;
    }
    for (int i = 0; i < 4; ++i) setStatus(i, 0, 0);
    uint32_t t = static_cast<uint32_t>(nowMs_);
    int fade = (t >> 1) & 255;
    fade = fade <= 127 ? (fade << 1) : 255 - (fade << 1);
    fade = (fade & 255) * (fade & 255) >> 8;
    switch (uiMode_) {
      case UI_VU_METER: {
        int32_t peak = std::max(peak_[0], peak_[1]);
        paintBar(clouds::lut_db[std::min(std::max(peak, 0), 32767) >> 7]);
        break;
      }
      case UI_BLEND_METER:
        for (int i = 0; i < 4; ++i)
          setIntensity(i, static_cast<int>(params[static_cast<size_t>(DRY_WET_PARAM + i)].value * 255.f));
        break;
      case UI_QUALITY:
        setStatus(intParam(QUALITY_PARAM) & 3, 255, 0);
        break;
      case UI_BLENDING:
        setStatus(intParam(BLEND_SELECT_PARAM) & 3, 0, 255);
        break;
      case UI_PLAYBACK:
        setStatus(intParam(PLAYBACK_PARAM) & 3, 128 + (fade >> 1), 255 - (fade >> 1));
        break;
    }
    lights[FREEZE_LIGHT] = processor_.frozen() ? 1.f : 0.f;
  }

  enum { kPosition, kDensity, kSizePot, kSizeCv, kPitchPot, kVoct, kBlendCv, kTexturePot,
         kTextureCv, kNumChannels };

  // Same sizes as block_mem / block_ccm in clouds.cc.
  uint8_t blockMem_[118784];
  uint8_t blockCcm_[65536 - 128];
  clouds::GranularProcessor processor_;
  BlockAdapter<2, 2> adapter_;

  float smoothed_[kNumChannels] = {};
  float blend_[4] = {1.f, 0.5f, 0.f, 0.f};
  float blendMod_[4] = {};
  float note_ = 0.f;
  float lastBlendKnob_ = -1.f;
  float blendQuantized_ = -1.f;
  int blendSelected_ = -1;
  bool blendSync_ = true;

  SchmittTrigger freezeGate_, trigIn_;
  bool trigPending_ = false;
  ButtonEdge freezeButton_;

  UiMode uiMode_ = UI_VU_METER;
  double nowMs_ = 0.0;
  double pressStart_ = 0.0;
  double lastEventMs_ = 0.0;
  bool modePressed_ = false, pressArmed_ = false, longArmed_ = false;
  int32_t peak_[2] = {0, 0};
};

void buildPanel(PanelSpec& p) {
  // 18 HP. Positions measured on the original panel (mm = px * 0.1285).
  p.hp = 18;
  p.bg = 0xFFF3F1EE;
  p.ink = 0xFF242426;
  p.title = "TEXTURE SYNTH";
  const uint32_t red = 0xFFB9414F;
  const uint32_t teal = 0xFF14958C;
  const uint32_t white = 0xFFFFFFFF;
  const uint32_t grey = 0xFFB4B4B4;

  // Coloured label tags, as printed on the hardware.
  auto tag = [&](float x, float y, float w, const char* text, uint32_t col, float size) {
    p.box(x - w * 0.5f, y - 2.2f, w, 4.4f, col, 1.0f);
    p.label(x, y, text, size, kAlignCenter, white);
  };

  // Button / LED row.
  p.button(TextureSynth::FREEZE_PARAM, 8.7f, 19.5f, 7.f, "", 0xFFD8D9DB,
           TextureSynth::FREEZE_LIGHT);
  p.label(20.4f, 19.5f, "FREEZE", 3.2f);
  const float ledX[4] = {29.6f, 40.1f, 50.8f, 61.3f};
  const char* qualityText[4] = {"16 ST", "16 MONO", "8 ST", "8 MONO"};
  const char* blendText[4] = {"MIX", "SPREAD", "FDBK", "VERB"};
  for (int i = 0; i < 4; ++i) {
    // Red/green LEDs, drawn as RGB lights so that both on mix to yellow/orange.
    p.rgbLight(2 * i + 1, 2 * i, TextureSynth::ZERO_LIGHT, ledX[i], 19.5f, 3.2f);
    p.label(ledX[i], 14.6f, qualityText[i], 2.4f, kAlignCenter, red);
    p.label(ledX[i], 24.4f, blendText[i], 2.4f, kAlignCenter, teal);
  }
  p.button(TextureSynth::MODE_PARAM, 76.5f, 19.5f, 6.f, "", 0xFFD8D9DB);
  p.label(76.5f, 14.6f, "MODE", 2.6f);

  // Big knobs.
  p.knob(TextureSynth::POSITION_PARAM, 18.0f, 40.2f, 14.f);
  p.knob(TextureSynth::SIZE_PARAM, 45.7f, 40.2f, 14.f);
  p.knob(TextureSynth::PITCH_PARAM, 73.3f, 40.2f, 14.f);
  tag(18.0f, 53.7f, 15.f, "POSITION", red, 3.0f);
  tag(45.7f, 53.7f, 10.f, "SIZE", teal, 3.0f);
  p.label(73.3f, 53.7f, "PITCH", 3.2f);

  // Small knobs.
  p.knob(TextureSynth::IN_GAIN_PARAM, 12.2f, 68.4f, 9.f);
  p.knob(TextureSynth::DENSITY_PARAM, 34.4f, 68.4f, 9.f);
  p.knob(TextureSynth::TEXTURE_PARAM, 56.5f, 68.4f, 9.f);
  p.knob(TextureSynth::BLEND_PARAM, 78.8f, 68.4f, 9.f);
  tag(12.2f, 79.2f, 13.f, "IN GAIN", red, 2.8f);
  tag(34.4f, 79.2f, 15.f, "DENSITY", red, 2.8f);
  tag(56.5f, 79.2f, 15.f, "TEXTURE", teal, 2.8f);
  p.label(78.8f, 79.2f, "BLEND", 3.0f);

  p.line(3.5f, 85.5f, 87.9f, 85.5f, 0.6f, grey);

  // Jacks.
  const float col[6] = {9.0f, 23.8f, 38.3f, 52.8f, 67.5f, 82.0f};
  const float row1 = 96.4f, row2 = 111.2f;
  p.input(TextureSynth::FREEZE_INPUT, col[0], row1, "FREEZE");
  p.input(TextureSynth::TRIG_INPUT, col[1], row1, "TRIG");
  p.input(TextureSynth::POSITION_INPUT, col[2], row1);
  tag(col[2], row1 - 6.6f, 8.f, "POS", red, 2.7f);
  p.input(TextureSynth::SIZE_INPUT, col[3], row1);
  tag(col[3], row1 - 6.6f, 8.f, "SIZE", teal, 2.7f);
  p.input(TextureSynth::PITCH_INPUT, col[4], row1, "V/OCT");
  p.input(TextureSynth::BLEND_INPUT, col[5], row1, "BLEND");

  p.box(62.6f, 102.4f, 24.8f, 13.9f, 0xFF5E5E60, 1.2f);
  p.input(TextureSynth::IN_L_INPUT, col[0], row2, "IN L");
  p.input(TextureSynth::IN_R_INPUT, col[1], row2, "IN R");
  p.line(col[0] + 3.6f, row2, col[1] - 3.6f, row2, 0.3f, p.ink);  // R normalled to L
  p.input(TextureSynth::DENSITY_INPUT, col[2], row2);
  tag(col[2], row2 - 6.6f, 9.f, "DENS", red, 2.7f);
  p.input(TextureSynth::TEXTURE_INPUT, col[3], row2);
  tag(col[3], row2 - 6.6f, 9.f, "TEXT", teal, 2.7f);
  p.output(TextureSynth::OUT_L_OUTPUT, col[4], row2 + 0.8f);
  p.output(TextureSynth::OUT_R_OUTPUT, col[5], row2 + 0.8f);
  p.label(col[4], 105.4f, "OUT L", 2.7f, kAlignCenter, white);
  p.label(col[5], 105.4f, "OUT R", 2.7f, kAlignCenter, white);

  p.line(3.5f, 119.2f, 87.9f, 119.2f, 0.6f, grey);

  // Playback mode / quality readout (handy on a small screen).
  p.display(3.f, 120.2f, 85.44f, 4.6f, [](Canvas& c, float x, float y, float w, float h, float s,
                                           const Module* m) {
    int mode = static_cast<int>(m->params[TextureSynth::PLAYBACK_PARAM].value + 0.5f) & 3;
    int q = static_cast<int>(m->params[TextureSynth::QUALITY_PARAM].value + 0.5f) & 3;
    std::string text = std::string(kPlaybackShort[mode]) + "  /  " + kQualityNames[q];
    c.text(x + w * 0.5f, y + h * 0.5f, text, std::max(12.f, 2.7f * s), 0xFF14958C, kAlignCenter);
  });
}

Model model = [] {
  Model m;
  m.slug = "texture-synth";
  m.name = "Texture Synthesizer";
  m.shortName = "TEXTURE SYNTH";
  m.basedOn = "Clouds";
  m.category = Category::Effect;
  m.description = "Granular audio processor: granular, stretch, looping delay, spectral modes.";
  m.create = [] { return new TextureSynth(); };
  m.buildPanel = buildPanel;
  return m;
}();

BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
