// BrickRack - GPL-3.0-or-later.
//
// Modal Synthesizer: wraps the Elements Part (elements/dsp/part.cc) at its
// native 32 kHz in 16-sample blocks, as elements.cc's FillBuffer() runs it:
// cv_scaler.cc's pot laws, low-pass filters and BIND() mappings, the coarse
// tune quantizer, the input noise gates, SoftConvert() on the outputs, and
// the PLAY button from ui.cc including its 3-second gestures (resonator
// model, "Ominous" easter egg).

#include <algorithm>
#include <cmath>
#include <cstring>

#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"
#include "elements/dsp/part.h"
#include "elements/resources.h"
#include "stmlib/dsp/dsp.h"

namespace br {
namespace {

const char* kResonatorModels[3] = {"Modal", "String", "Strings"};

// The freshly-baked calibration (pitch_scale = -84.26 semitones per ADC unit)
// puts 7.02 V across the CV ADC range; 0 V reads 0.505.
constexpr float kCvScale = 1.f / 7.02f;

inline float quadraticBipolar(float x) { return 3.3f * x * std::fabs(x); }
inline float quarticBipolar(float x) {
  float x2 = x * x;
  return 3.3f * (x < 0.f ? -x2 * x2 : x2 * x2);
}

class ModalSynth : public Module {
 public:
  enum ParamIds {
    CONTOUR_PARAM,
    BOW_PARAM,
    BLOW_PARAM,
    STRIKE_PARAM,
    COARSE_PARAM,
    FINE_PARAM,
    FM_PARAM,
    FLOW_PARAM,
    MALLET_PARAM,
    GEOMETRY_PARAM,
    BRIGHTNESS_PARAM,
    BOW_TIMBRE_PARAM,
    BLOW_TIMBRE_PARAM,
    STRIKE_TIMBRE_PARAM,
    DAMPING_PARAM,
    POSITION_PARAM,
    SPACE_PARAM,
    BOW_TIMBRE_MOD_PARAM,
    FLOW_MOD_PARAM,
    BLOW_TIMBRE_MOD_PARAM,
    MALLET_MOD_PARAM,
    STRIKE_TIMBRE_MOD_PARAM,
    DAMPING_MOD_PARAM,
    GEOMETRY_MOD_PARAM,
    POSITION_MOD_PARAM,
    BRIGHTNESS_MOD_PARAM,
    SPACE_MOD_PARAM,
    PLAY_PARAM,
    MODEL_PARAM,  // hidden: resonator model
    NUM_PARAMS
  };
  enum InputIds {
    NOTE_INPUT,
    FM_INPUT,
    GATE_INPUT,
    STRENGTH_INPUT,
    BLOW_INPUT,
    STRIKE_INPUT,
    BOW_TIMBRE_MOD_INPUT,
    FLOW_MOD_INPUT,
    BLOW_TIMBRE_MOD_INPUT,
    MALLET_MOD_INPUT,
    STRIKE_TIMBRE_MOD_INPUT,
    DAMPING_MOD_INPUT,
    GEOMETRY_MOD_INPUT,
    POSITION_MOD_INPUT,
    BRIGHTNESS_MOD_INPUT,
    SPACE_MOD_INPUT,
    NUM_INPUTS
  };
  enum OutputIds { AUX_OUTPUT, MAIN_OUTPUT, NUM_OUTPUTS };
  enum LightIds { GATE_LIGHT, EXCITER_LIGHT, RESONATOR_LIGHT, NUM_LIGHTS };

  ModalSynth() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(CONTOUR_PARAM, 0.f, 1.f, 1.f, "Envelope contour", "%", 100.f);
    configParam(BOW_PARAM, 0.f, 1.f, 0.f, "Bow level", "%", 100.f);
    configParam(BLOW_PARAM, 0.f, 1.f, 0.f, "Blow level", "%", 100.f);
    configParam(STRIKE_PARAM, 0.f, 1.f, 0.5f, "Strike level", "%", 100.f);
    configParam(COARSE_PARAM, -30.f, 30.f, 0.f, "Coarse frequency", "semitones");
    configParam(FINE_PARAM, -1.f, 1.f, 0.f, "Fine frequency");
    configParam(FM_PARAM, -1.f, 1.f, 0.f, "FM amount", "%", 100.f);
    configParam(FLOW_PARAM, 0.f, 1.f, 0.5f, "Flow (air turbulence)", "%", 100.f);
    configParam(MALLET_PARAM, 0.f, 1.f, 0.5f, "Mallet type", "%", 100.f);
    configParam(GEOMETRY_PARAM, 0.f, 1.f, 0.5f, "Geometry", "%", 100.f);
    configParam(BRIGHTNESS_PARAM, 0.f, 1.f, 0.5f, "Brightness", "%", 100.f);
    configParam(BOW_TIMBRE_PARAM, 0.f, 1.f, 0.5f, "Bow timbre", "%", 100.f);
    configParam(BLOW_TIMBRE_PARAM, 0.f, 1.f, 0.5f, "Blow timbre", "%", 100.f);
    configParam(STRIKE_TIMBRE_PARAM, 0.f, 1.f, 0.5f, "Strike timbre", "%", 100.f);
    configParam(DAMPING_PARAM, 0.f, 1.f, 0.5f, "Damping", "%", 100.f);
    configParam(POSITION_PARAM, 0.f, 1.f, 0.5f, "Position", "%", 100.f);
    configParam(SPACE_PARAM, 0.f, 1.f, 0.25f, "Space", "%", 100.f);
    configParam(BOW_TIMBRE_MOD_PARAM, -1.f, 1.f, 0.f, "Bow timbre CV amount", "%", 100.f);
    configParam(FLOW_MOD_PARAM, -1.f, 1.f, 0.f, "Flow CV amount", "%", 100.f);
    configParam(BLOW_TIMBRE_MOD_PARAM, -1.f, 1.f, 0.f, "Blow timbre CV amount", "%", 100.f);
    configParam(MALLET_MOD_PARAM, -1.f, 1.f, 0.f, "Mallet CV amount", "%", 100.f);
    configParam(STRIKE_TIMBRE_MOD_PARAM, -1.f, 1.f, 0.f, "Strike timbre CV amount", "%", 100.f);
    configParam(DAMPING_MOD_PARAM, -1.f, 1.f, 0.f, "Damping CV amount", "%", 100.f);
    configParam(GEOMETRY_MOD_PARAM, -1.f, 1.f, 0.f, "Geometry CV amount", "%", 100.f);
    configParam(POSITION_MOD_PARAM, -1.f, 1.f, 0.f, "Position CV amount", "%", 100.f);
    configParam(BRIGHTNESS_MOD_PARAM, -1.f, 1.f, 0.f, "Brightness CV amount", "%", 100.f);
    configParam(SPACE_MOD_PARAM, -1.f, 1.f, 0.f, "Space CV amount", "%", 100.f);
    configButton(PLAY_PARAM, "Play (hold 3 s for the secret gestures)");
    configSwitch(MODEL_PARAM, 0.f, 2.f, 0.f, "Resonator model",
                 std::vector<std::string>(kResonatorModels, kResonatorModels + 3))
        .hidden = true;

    configInput(NOTE_INPUT, "V/Oct");
    configInput(FM_INPUT, "FM");
    configInput(GATE_INPUT, "Gate");
    configInput(STRENGTH_INPUT, "Strength");
    configInput(BLOW_INPUT, "Blow audio in");
    configInput(STRIKE_INPUT, "Strike audio in");
    configInput(BOW_TIMBRE_MOD_INPUT, "Bow timbre CV");
    configInput(FLOW_MOD_INPUT, "Flow CV");
    configInput(BLOW_TIMBRE_MOD_INPUT, "Blow timbre CV");
    configInput(MALLET_MOD_INPUT, "Mallet CV");
    configInput(STRIKE_TIMBRE_MOD_INPUT, "Strike timbre CV");
    configInput(DAMPING_MOD_INPUT, "Damping CV");
    configInput(GEOMETRY_MOD_INPUT, "Geometry CV");
    configInput(POSITION_MOD_INPUT, "Position CV");
    configInput(BRIGHTNESS_MOD_INPUT, "Brightness CV");
    configInput(SPACE_MOD_INPUT, "Space CV");
    configOutput(AUX_OUTPUT, "Out L");
    configOutput(MAIN_OUTPUT, "Out R");

    std::memset(reverbBuffer_, 0, sizeof(reverbBuffer_));
    std::memset(static_cast<void*>(&part_), 0, sizeof(part_));
    part_.Init(reverbBuffer_);
    // The hardware seeds its per-unit "signature" from the MCU serial number.
    uint32_t seed[3] = {0x42726963u, 0x6b526163u, 0x6b000001u};
    part_.Seed(seed, 3);
    adapter_.init(48000.f, elements::kSampleRate, kBlockSize);
  }

  void onSampleRateChange(float sampleRate) override {
    adapter_.init(sampleRate, elements::kSampleRate, kBlockSize);
  }

  void process(const ProcessArgs& args) override {
    handlePlayButton(params[PLAY_PARAM].value > 0.5f, args.sampleTime);
    gateTrigger_.process(inputs[GATE_INPUT].getVoltage());

    float in[2] = {inputs[BLOW_INPUT].getVoltage() / 5.f, inputs[STRIKE_INPUT].getVoltage() / 5.f};
    float out[2];
    adapter_.process(in, out, [&](const float* nin, float* nout, int n) {
      renderBlock(nin, nout, n);
    });
    // SoftConvert(): soft-limited int16 codec output.
    outputs[AUX_OUTPUT].setVoltage(stmlib::SoftLimit(out[0] * 0.5f) * 5.f);
    outputs[MAIN_OUTPUT].setVoltage(stmlib::SoftLimit(out[1] * 0.5f) * 5.f);

    if ((args.frame & 63) == 0) updateLights(args.frame * 1000.0 / args.sampleRate);
  }

  void saveState(StateMap& s) const override { s["ominous"] = easterEgg_ ? "1" : "0"; }
  void loadState(const StateMap& s) override {
    auto it = s.find("ominous");
    if (it != s.end()) easterEgg_ = it->second == "1";
  }
  void onReset() override { easterEgg_ = false; }
  void appendMenu(std::vector<MenuAction>& items) override {
    items.push_back({"Ominous voice (easter egg)",
                     [this] { return std::string(easterEgg_ ? "ON" : "OFF"); },
                     [this](int) { easterEgg_ = !easterEgg_; }});
  }

 private:
  static constexpr int kBlockSize = 16;

  // Firmware pot indices (elements/drivers/pots_adc.h).
  enum {
    POT_EXCITER_ENVELOPE_SHAPE, POT_EXCITER_BOW_LEVEL, POT_EXCITER_BOW_TIMBRE,
    POT_EXCITER_BOW_TIMBRE_ATTENUVERTER, POT_EXCITER_BLOW_LEVEL, POT_EXCITER_BLOW_META,
    POT_EXCITER_BLOW_META_ATTENUVERTER, POT_EXCITER_BLOW_TIMBRE,
    POT_EXCITER_BLOW_TIMBRE_ATTENUVERTER, POT_EXCITER_STRIKE_LEVEL, POT_EXCITER_STRIKE_META,
    POT_EXCITER_STRIKE_META_ATTENUVERTER, POT_EXCITER_STRIKE_TIMBRE,
    POT_EXCITER_STRIKE_TIMBRE_ATTENUVERTER, POT_RESONATOR_COARSE, POT_RESONATOR_FINE,
    POT_RESONATOR_FM_ATTENUVERTER, POT_RESONATOR_GEOMETRY, POT_RESONATOR_GEOMETRY_ATTENUVERTER,
    POT_RESONATOR_BRIGHTNESS, POT_RESONATOR_BRIGHTNESS_ATTENUVERTER, POT_RESONATOR_DAMPING,
    POT_RESONATOR_DAMPING_ATTENUVERTER, POT_RESONATOR_POSITION,
    POT_RESONATOR_POSITION_ATTENUVERTER, POT_SPACE, POT_SPACE_ATTENUVERTER, POT_LAST
  };

  // CvScaler::ReadPanelPots(): pot laws, then a one-pole low-pass on all pots.
  void readPots() {
    auto lin = [&](int id) { return params[id].value; };
    auto quad = [&](int id) { return quadraticBipolar(params[id].value); };
    float raw[POT_LAST];
    raw[POT_EXCITER_ENVELOPE_SHAPE] = lin(CONTOUR_PARAM);
    raw[POT_EXCITER_BOW_LEVEL] = lin(BOW_PARAM);
    raw[POT_EXCITER_BOW_TIMBRE] = lin(BOW_TIMBRE_PARAM);
    raw[POT_EXCITER_BOW_TIMBRE_ATTENUVERTER] = quad(BOW_TIMBRE_MOD_PARAM);
    raw[POT_EXCITER_BLOW_LEVEL] = lin(BLOW_PARAM);
    raw[POT_EXCITER_BLOW_META] = lin(FLOW_PARAM);
    raw[POT_EXCITER_BLOW_META_ATTENUVERTER] = quad(FLOW_MOD_PARAM);
    raw[POT_EXCITER_BLOW_TIMBRE] = lin(BLOW_TIMBRE_PARAM);
    raw[POT_EXCITER_BLOW_TIMBRE_ATTENUVERTER] = quad(BLOW_TIMBRE_MOD_PARAM);
    raw[POT_EXCITER_STRIKE_LEVEL] = lin(STRIKE_PARAM);
    raw[POT_EXCITER_STRIKE_META] = lin(MALLET_PARAM);
    raw[POT_EXCITER_STRIKE_META_ATTENUVERTER] = quad(MALLET_MOD_PARAM);
    raw[POT_EXCITER_STRIKE_TIMBRE] = lin(STRIKE_TIMBRE_PARAM);
    raw[POT_EXCITER_STRIKE_TIMBRE_ATTENUVERTER] = quad(STRIKE_TIMBRE_MOD_PARAM);
    raw[POT_RESONATOR_FINE] = quad(FINE_PARAM);
    raw[POT_RESONATOR_FM_ATTENUVERTER] = quarticBipolar(params[FM_PARAM].value);
    raw[POT_RESONATOR_GEOMETRY] = lin(GEOMETRY_PARAM);
    raw[POT_RESONATOR_GEOMETRY_ATTENUVERTER] = quad(GEOMETRY_MOD_PARAM);
    raw[POT_RESONATOR_BRIGHTNESS] = lin(BRIGHTNESS_PARAM);
    raw[POT_RESONATOR_BRIGHTNESS_ATTENUVERTER] = quad(BRIGHTNESS_MOD_PARAM);
    raw[POT_RESONATOR_DAMPING] = lin(DAMPING_PARAM);
    raw[POT_RESONATOR_DAMPING_ATTENUVERTER] = quad(DAMPING_MOD_PARAM);
    raw[POT_RESONATOR_POSITION] = lin(POSITION_PARAM);
    raw[POT_RESONATOR_POSITION_ATTENUVERTER] = quad(POSITION_MOD_PARAM);
    raw[POT_SPACE] = lin(SPACE_PARAM);
    raw[POT_SPACE_ATTENUVERTER] = quad(SPACE_MOD_PARAM);

    // LAW_QUANTIZED_NOTE: 5 octaves, smoothed then quantized with hysteresis.
    float note = params[COARSE_PARAM].value + 30.f;
    coarseRaw_ += 0.5f * (note - coarseRaw_);
    float hysteresis = coarseRaw_ - coarseQuantized_ > 0.f ? -0.3f : +0.3f;
    coarseQuantized_ = static_cast<float>(static_cast<int>(coarseRaw_ + hysteresis + 0.5f));
    raw[POT_RESONATOR_COARSE] = coarseRaw_;

    for (int i = 0; i < POT_LAST; ++i) potLp_[i] += 0.01f * (raw[i] - potLp_[i]);
  }

  // CvScaler::Read().
  void readCv(elements::Patch* patch, elements::PerformanceState* state) {
    readPots();
    patch->exciter_envelope_shape = potLp_[POT_EXCITER_ENVELOPE_SHAPE];
    patch->exciter_bow_level = potLp_[POT_EXCITER_BOW_LEVEL];
    patch->exciter_blow_level = potLp_[POT_EXCITER_BLOW_LEVEL];
    patch->exciter_strike_level = potLp_[POT_EXCITER_STRIKE_LEVEL];

    // BIND(): pot + attenuverter * CV, clamped, optionally low-passed.
    auto bind = [&](float* dest, int pot, int input, float lp, float hi) {
      float cv = inputs[input].getVoltage() * kCvScale;
      float value = clampf(potLp_[pot] + potLp_[pot + 1] * cv, 0.f, hi);
      *dest = lp != 1.f ? *dest + (value - *dest) * lp : value;
    };
    bind(&patch->exciter_bow_timbre, POT_EXCITER_BOW_TIMBRE, BOW_TIMBRE_MOD_INPUT, 1.f, 0.9995f);
    bind(&patch->exciter_blow_meta, POT_EXCITER_BLOW_META, FLOW_MOD_INPUT, 0.05f, 0.9995f);
    bind(&patch->exciter_blow_timbre, POT_EXCITER_BLOW_TIMBRE, BLOW_TIMBRE_MOD_INPUT, 1.f, 0.9995f);
    bind(&patch->exciter_strike_meta, POT_EXCITER_STRIKE_META, MALLET_MOD_INPUT, 0.05f, 0.9995f);
    bind(&patch->exciter_strike_timbre, POT_EXCITER_STRIKE_TIMBRE, STRIKE_TIMBRE_MOD_INPUT, 1.f, 0.995f);
    bind(&patch->resonator_geometry, POT_RESONATOR_GEOMETRY, GEOMETRY_MOD_INPUT, 0.05f, 0.9995f);
    bind(&patch->resonator_brightness, POT_RESONATOR_BRIGHTNESS, BRIGHTNESS_MOD_INPUT, 1.f, 0.9995f);
    bind(&patch->resonator_damping, POT_RESONATOR_DAMPING, DAMPING_MOD_INPUT, 1.f, 0.9995f);
    bind(&patch->resonator_position, POT_RESONATOR_POSITION, POSITION_MOD_INPUT, 0.01f, 0.9995f);
    bind(&patch->space, POT_SPACE, SPACE_MOD_INPUT, 0.01f, 2.f);

    // V/Oct with the firmware's quarter-tone slew limiter.
    float note = inputs[NOTE_INPUT].getVoltage() * 12.f;
    float interval = note - note_;
    if (interval < -0.4f || interval > 0.4f) {
      note_ = note;
    } else {
      note_ += 0.1f * interval;
    }
    float modulation =
        potLp_[POT_RESONATOR_FM_ATTENUVERTER] * 49.5f * inputs[FM_INPUT].getVoltage() * kCvScale;
    modulation_ += 0.5f * (modulation - modulation_);
    state->modulation = clampf(modulation_, -60.f, 60.f);

    state->note = note_ + coarseQuantized_ + 19.f + potLp_[POT_RESONATOR_FINE] * (2.f / 3.3f);
    // 1 - ADC reading: 0 V (and an unpatched jack) is 0.495, about unity gain.
    state->strength = clampf(0.495f + inputs[STRENGTH_INPUT].getVoltage() * kCvScale, 0.f, 1.f);

    // The gate is read one block late, after the other CVs.
    state->gate = previousGate_;
    previousGate_ = gate_;
    gate_ = gateTrigger_.isHigh();
  }

  // elements.cc: FillBuffer().
  void renderBlock(const float* nin, float* nout, int n) {
    part_.set_easter_egg(easterEgg_);
    part_.set_resonator_model(
        static_cast<elements::ResonatorModel>(static_cast<int>(params[MODEL_PARAM].value + 0.5f)));

    elements::PerformanceState s;
    readCv(part_.mutable_patch(), &s);
    s.gate |= buttonGate_;

    for (int i = 0; i < n; ++i) {
      float blow = nin[2 * i], strike = nin[2 * i + 1];
      float error = strike * strike - strikeLevel_;
      strikeLevel_ += error * (error > 0.f ? 0.1f : 0.0001f);
      float gain = strikeLevel_ <= kNoiseGateThreshold ? (1.f / kNoiseGateThreshold) * strikeLevel_ : 1.f;
      strikeIn_[i] = gain * strike;
      error = blow * blow - blowLevel_;
      blowLevel_ += error * (error > 0.f ? 0.1f : 0.0001f);
      gain = blowLevel_ <= kNoiseGateThreshold ? (1.f / kNoiseGateThreshold) * blowLevel_ : 1.f;
      blowIn_[i] = gain * blow;
    }
    part_.Process(s, blowIn_, strikeIn_, out_, aux_, static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
      nout[2 * i] = aux_[i];
      nout[2 * i + 1] = out_[i];
    }
  }

  // ui.cc: the PLAY button gates while held; a 3 s hold with the
  // "resonator high" knob gesture cycles the resonator model, or with the
  // "exciter low" gesture as well, toggles the Ominous voice.
  void handlePlayButton(bool pressed, float dt) {
    if (pressed) {
      if (!held_) {
        held_ = true;
        holdTime_ = 0.f;
        longFired_ = false;
        buttonGate_ = true;
      } else if (!longFired_) {
        holdTime_ += dt;
        if (holdTime_ >= 3.f && resonatorHigh()) {
          longFired_ = true;
          buttonGate_ = false;
          if (exciterLow()) {
            easterEgg_ = !easterEgg_;
          } else {
            int model = (static_cast<int>(params[MODEL_PARAM].value + 0.5f) + 1) % 3;
            params[MODEL_PARAM].set(static_cast<float>(model));
            showModel_ = true;
          }
        }
      }
    } else if (held_) {
      held_ = false;
      buttonGate_ = false;
    }
  }

  bool exciterLow() const {
    return potLp_[POT_EXCITER_BOW_TIMBRE_ATTENUVERTER] < -3.f &&
           potLp_[POT_EXCITER_BLOW_META_ATTENUVERTER] < -3.f &&
           potLp_[POT_EXCITER_BLOW_TIMBRE_ATTENUVERTER] < -3.f &&
           potLp_[POT_EXCITER_STRIKE_META_ATTENUVERTER] < -3.f &&
           potLp_[POT_EXCITER_STRIKE_TIMBRE_ATTENUVERTER] < -3.f &&
           potLp_[POT_EXCITER_BLOW_TIMBRE] < 0.2f && potLp_[POT_EXCITER_BOW_TIMBRE] < 0.2f &&
           potLp_[POT_EXCITER_STRIKE_TIMBRE] < 0.2f;
  }
  bool resonatorHigh() const {
    return potLp_[POT_RESONATOR_GEOMETRY_ATTENUVERTER] > 3.f &&
           potLp_[POT_RESONATOR_BRIGHTNESS_ATTENUVERTER] > 3.f &&
           potLp_[POT_RESONATOR_DAMPING_ATTENUVERTER] > 3.f &&
           potLp_[POT_RESONATOR_POSITION_ATTENUVERTER] > 3.f &&
           potLp_[POT_SPACE_ATTENUVERTER] > 3.f && potLp_[POT_RESONATOR_DAMPING] > 0.8f &&
           potLp_[POT_RESONATOR_POSITION] > 0.8f && potLp_[POT_SPACE] > 0.8f;
  }

  // Ui::Poll(): gate LED, dB-scaled exciter / resonator meters, model display.
  void updateLights(double ms) {
    if (showModel_) {
      showModel_ = false;
      modelShownUntil_ = ms + 4000.0;
    }
    if (ms < modelShownUntil_) {
      int64_t t = static_cast<int64_t>(ms);
      bool blink = (t & 255) > 128;
      int count = static_cast<int>((t >> 8) & 3);
      bool pulse = count <= static_cast<int>(params[MODEL_PARAM].value + 0.5f) && blink;
      lights[GATE_LIGHT] = lights[EXCITER_LIGHT] = lights[RESONATOR_LIGHT] = pulse ? 1.f : 0.f;
      return;
    }
    auto meter = [](float level) {
      int i = std::min(std::max(static_cast<int>(level * 512.f), 0), 512);
      return clampf(elements::lut_db_led_brightness[i] / 256.f, 0.f, 1.f);
    };
    lights[GATE_LIGHT] = part_.gate() ? 1.f : 0.f;
    lights[EXCITER_LIGHT] = meter(part_.exciter_level());
    lights[RESONATOR_LIGHT] = meter(part_.resonator_level());
  }

  static constexpr float kNoiseGateThreshold = 0.0001f;

  uint16_t reverbBuffer_[32768];
  elements::Part part_;
  BlockAdapter<2, 2> adapter_;
  float blowIn_[kBlockSize], strikeIn_[kBlockSize], out_[kBlockSize], aux_[kBlockSize];
  float potLp_[POT_LAST] = {};
  float coarseRaw_ = 30.f, coarseQuantized_ = 30.f;
  float note_ = 0.f, modulation_ = 0.f;
  float strikeLevel_ = 0.f, blowLevel_ = 0.f;
  bool gate_ = false, previousGate_ = false;
  SchmittTrigger gateTrigger_;
  bool easterEgg_ = false;

  bool held_ = false, longFired_ = false, buttonGate_ = false, showModel_ = false;
  float holdTime_ = 0.f;
  double modelShownUntil_ = 0.0;
};

void buildPanel(PanelSpec& p) {
  p.hp = 34;
  p.bg = 0xFFF2F1EC;
  p.ink = 0xFF1E1E20;
  p.title = "MODAL SYNTH";
  const uint32_t red = 0xFFC8414B;
  const uint32_t teal = 0xFF168A8C;
  const uint32_t white = 0xFFFFFFFF;

  // Divider between the exciter and resonator sections.
  p.line(99.8f, 13.f, 99.8f, 116.f, 0.4f, 0xFFC4C4C4);

  // Top row: contour, bow / blow / strike levels, coarse / fine / FM.
  const float y1 = 21.0f, yl1 = 31.6f;
  p.knob(ModalSynth::CONTOUR_PARAM, 16.7f, y1, 9.f);
  p.label(16.7f, yl1, "CONTOUR", 3.0f);
  p.knob(ModalSynth::BOW_PARAM, 40.5f, y1, 9.f);
  p.label(40.5f, yl1, "BOW", 3.0f);
  p.knob(ModalSynth::BLOW_PARAM, 64.2f, y1, 9.f);
  p.box(58.6f, yl1 - 2.0f, 11.2f, 4.0f, red, 1.0f);
  p.label(64.2f, yl1, "BLOW", 2.9f, kAlignCenter, white);
  p.knob(ModalSynth::STRIKE_PARAM, 88.1f, y1, 9.f);
  p.box(81.4f, yl1 - 2.0f, 13.4f, 4.0f, teal, 1.0f);
  p.label(88.1f, yl1, "STRIKE", 2.9f, kAlignCenter, white);
  p.knob(ModalSynth::COARSE_PARAM, 111.9f, y1, 9.f);
  p.label(111.9f, yl1, "COARSE", 3.0f);
  p.knob(ModalSynth::FINE_PARAM, 135.7f, y1, 9.f);
  p.label(135.7f, yl1, "FINE", 3.0f);
  p.knob(ModalSynth::FM_PARAM, 159.5f, y1, 9.f);
  p.label(159.5f, yl1, "FM", 3.0f);
  p.label(152.0f, 27.6f, "-", 3.2f);
  p.label(167.0f, 27.6f, "+", 3.2f);
  p.line(21.2f, y1, 59.7f, y1, 0.3f, red);  // contour shapes the bow/blow/strike envelope

  // Play button with gate LED.
  p.button(ModalSynth::PLAY_PARAM, 16.7f, 43.6f, 8.f, "", 0xFFD8D9DB, ModalSynth::GATE_LIGHT);
  p.label(27.6f, 43.6f, "PLAY", 3.0f);

  // Big knobs and level LEDs.
  const float y2 = 48.3f, yl2 = 60.9f;
  p.knob(ModalSynth::FLOW_PARAM, 44.0f, y2, 13.f);
  p.box(37.4f, yl2 - 2.0f, 13.2f, 4.0f, red, 1.0f);
  p.label(44.0f, yl2, "FLOW", 2.9f, kAlignCenter, white);
  p.knob(ModalSynth::MALLET_PARAM, 77.1f, y2, 13.f);
  p.box(69.6f, yl2 - 2.0f, 15.0f, 4.0f, teal, 1.0f);
  p.label(77.1f, yl2, "MALLET", 2.9f, kAlignCenter, white);
  p.knob(ModalSynth::GEOMETRY_PARAM, 119.2f, y2, 13.f);
  p.label(119.2f, yl2, "GEOMETRY", 3.0f);
  p.knob(ModalSynth::BRIGHTNESS_PARAM, 152.0f, y2, 13.f);
  p.label(152.0f, yl2, "BRIGHTNESS", 3.0f);
  p.light(ModalSynth::EXCITER_LIGHT, 64.3f, 57.4f, 0xFF35E06A, 3.f);
  p.light(ModalSynth::RESONATOR_LIGHT, 135.7f, 57.4f, 0xFF35E06A, 3.f);

  // Second row of knobs.
  const float y3 = 75.4f, yl3 = 85.9f;
  p.knob(ModalSynth::BOW_TIMBRE_PARAM, 40.5f, y3, 9.f);
  p.label(40.5f, yl3, "TIMBRE", 3.0f);
  p.knob(ModalSynth::BLOW_TIMBRE_PARAM, 64.2f, y3, 9.f);
  p.box(56.8f, yl3 - 2.0f, 14.8f, 4.0f, red, 1.0f);
  p.label(64.2f, yl3, "TIMBRE", 2.9f, kAlignCenter, white);
  p.knob(ModalSynth::STRIKE_TIMBRE_PARAM, 88.1f, y3, 9.f);
  p.box(80.7f, yl3 - 2.0f, 14.8f, 4.0f, teal, 1.0f);
  p.label(88.1f, yl3, "TIMBRE", 2.9f, kAlignCenter, white);
  p.knob(ModalSynth::DAMPING_PARAM, 111.9f, y3, 9.f);
  p.label(111.9f, yl3, "DAMPING", 3.0f);
  p.knob(ModalSynth::POSITION_PARAM, 135.7f, y3, 9.f);
  p.label(135.7f, yl3, "POSITION", 3.0f);
  p.knob(ModalSynth::SPACE_PARAM, 159.5f, y3, 9.f);
  p.label(159.5f, yl3, "SPACE", 3.0f);

  // Attenuverters and their CV inputs.
  const float cols[10] = {37.6f, 51.2f, 64.3f, 77.1f, 89.4f, 109.5f, 122.6f, 135.7f, 148.3f, 161.6f};
  const int atts[10] = {ModalSynth::BOW_TIMBRE_MOD_PARAM,    ModalSynth::FLOW_MOD_PARAM,
                        ModalSynth::BLOW_TIMBRE_MOD_PARAM,   ModalSynth::MALLET_MOD_PARAM,
                        ModalSynth::STRIKE_TIMBRE_MOD_PARAM, ModalSynth::DAMPING_MOD_PARAM,
                        ModalSynth::GEOMETRY_MOD_PARAM,      ModalSynth::POSITION_MOD_PARAM,
                        ModalSynth::BRIGHTNESS_MOD_PARAM,    ModalSynth::SPACE_MOD_PARAM};
  const int cvs[10] = {ModalSynth::BOW_TIMBRE_MOD_INPUT,    ModalSynth::FLOW_MOD_INPUT,
                       ModalSynth::BLOW_TIMBRE_MOD_INPUT,   ModalSynth::MALLET_MOD_INPUT,
                       ModalSynth::STRIKE_TIMBRE_MOD_INPUT, ModalSynth::DAMPING_MOD_INPUT,
                       ModalSynth::GEOMETRY_MOD_INPUT,      ModalSynth::POSITION_MOD_INPUT,
                       ModalSynth::BRIGHTNESS_MOD_INPUT,    ModalSynth::SPACE_MOD_INPUT};
  for (int i = 0; i < 10; ++i) {
    p.label(cols[i] - 3.0f, 90.6f, "-", 2.8f);
    p.label(cols[i] + 3.0f, 90.6f, "+", 2.8f);
    p.knob(atts[i], cols[i], 95.2f, 6.f);
    p.line(cols[i], 98.8f, cols[i], 107.6f, 0.3f, red);
    p.input(cvs[i], cols[i], 111.3f);
  }
  // The big knobs feed the 2nd, 4th, 7th and 9th CV columns.
  p.line(49.0f, 63.0f, 51.2f, 91.0f, 0.3f, red);
  p.line(79.0f, 63.0f, 77.1f, 91.0f, 0.3f, red);
  p.line(120.6f, 63.0f, 122.6f, 91.0f, 0.3f, red);
  p.line(150.4f, 63.0f, 148.3f, 91.0f, 0.3f, red);

  // Left column: pitch, gate, audio inputs, outputs.
  p.input(ModalSynth::NOTE_INPUT, 10.5f, 64.3f, "V/OCT", -6.9f);
  p.input(ModalSynth::FM_INPUT, 22.6f, 64.3f, "FM", -6.9f);
  p.input(ModalSynth::GATE_INPUT, 10.5f, 79.9f, "GATE", -6.8f);
  p.input(ModalSynth::STRENGTH_INPUT, 22.6f, 79.9f, "STRENGTH", -6.8f);
  p.box(5.0f, 86.8f, 11.0f, 4.0f, red, 1.0f);
  p.label(10.5f, 88.8f, "EXT IN", 2.6f, kAlignCenter, white);
  p.box(17.1f, 86.8f, 11.0f, 4.0f, teal, 1.0f);
  p.label(22.6f, 88.8f, "EXT IN", 2.6f, kAlignCenter, white);
  p.input(ModalSynth::BLOW_INPUT, 10.5f, 95.4f);
  p.input(ModalSynth::STRIKE_INPUT, 22.6f, 95.4f);
  p.box(5.0f, 102.4f, 11.2f, 14.8f, 0xFF5E5E60, 1.2f);
  p.box(17.0f, 102.4f, 11.2f, 14.8f, 0xFF5E5E60, 1.2f);
  p.label(10.6f, 105.0f, "OUT L", 2.6f, kAlignCenter, white);
  p.label(22.6f, 105.0f, "OUT R", 2.6f, kAlignCenter, white);
  p.output(ModalSynth::AUX_OUTPUT, 10.6f, 111.3f);
  p.output(ModalSynth::MAIN_OUTPUT, 22.6f, 111.3f);
}

Model model = [] {
  Model m;
  m.slug = "modal-synth";
  m.name = "Modal Synthesizer";
  m.shortName = "MODAL SYNTH";
  m.basedOn = "Elements";
  m.category = Category::Oscillator;
  m.description = "Bowed, blown and struck exciters into a modal / string resonator with reverb.";
  m.create = [] { return new ModalSynth(); };
  m.buildPanel = buildPanel;
  return m;
}();

BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
