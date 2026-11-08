// BrickRack - GPL-3.0-or-later.
//
// Resonator: wraps the Rings Part / StringSynthPart (rings/dsp) exactly as the
// firmware's FillBuffer() drives them: 24-sample blocks at 48 kHz, Patch and
// PerformanceState filled the way cv_scaler.cc does (same pot laws, same
// low-pass coefficients, same chord / transpose hysteresis), the input noise
// gate from rings.cc, and the two-button UI from ui.cc (short / 3 s long
// presses, bonus models, "Disastrous Peace" string-synth easter egg).

#include <algorithm>
#include <cmath>
#include <cstring>

#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"
#include "rings/dsp/part.h"
#include "rings/dsp/string_synth_part.h"
#include "rings/dsp/strummer.h"

namespace br {
namespace {

const char* kResonatorModels[6] = {
    "Modal resonator",      "Sympathetic strings", "Modulated/inharmonic string",
    "FM voice",             "Quantized sympathetic strings", "String and reverb",
};
const char* kEggFx[6] = {"Formant", "Chorus", "Reverb", "Formant 2", "Ensemble", "Reverb 2"};

// Pot laws from rings/cv_scaler.cc, expressed for a -1..1 attenuverter knob.
inline float quadraticBipolar(float x) { return 3.3f * x * std::fabs(x); }
inline float quarticBipolar(float x) {
  float x2 = x * x;
  return 3.3f * (x < 0.f ? -x2 * x2 : x2 * x2);
}

class Resonator : public Module {
 public:
  enum ParamIds {
    POLYPHONY_BUTTON_PARAM,
    RESONATOR_BUTTON_PARAM,
    FREQUENCY_PARAM,
    STRUCTURE_PARAM,
    BRIGHTNESS_PARAM,
    DAMPING_PARAM,
    POSITION_PARAM,
    BRIGHTNESS_MOD_PARAM,
    FREQUENCY_MOD_PARAM,
    DAMPING_MOD_PARAM,
    STRUCTURE_MOD_PARAM,
    POSITION_MOD_PARAM,
    POLYPHONY_PARAM,  // hidden: 1..4 voices
    MODEL_PARAM,      // hidden: resonator model 0..5 (FX type in easter-egg mode)
    NUM_PARAMS
  };
  enum InputIds {
    BRIGHTNESS_MOD_INPUT,
    FREQUENCY_MOD_INPUT,
    DAMPING_MOD_INPUT,
    STRUCTURE_MOD_INPUT,
    POSITION_MOD_INPUT,
    STRUM_INPUT,
    PITCH_INPUT,
    IN_INPUT,
    NUM_INPUTS
  };
  enum OutputIds { ODD_OUTPUT, EVEN_OUTPUT, NUM_OUTPUTS };
  enum LightIds {
    POLY_RED_LIGHT,
    POLY_GREEN_LIGHT,
    MODEL_RED_LIGHT,
    MODEL_GREEN_LIGHT,
    OFF_LIGHT,  // always 0: blue channel of the red/green LEDs
    NUM_LIGHTS
  };

  // Indices into lp_[], mirroring rings' ADC channel order.
  enum {
    CV_FREQUENCY, CV_STRUCTURE, CV_BRIGHTNESS, CV_DAMPING, CV_POSITION, CV_V_OCT,
    POT_FREQUENCY, POT_STRUCTURE, POT_BRIGHTNESS, POT_DAMPING, POT_POSITION,
    ATT_FREQUENCY, ATT_STRUCTURE, ATT_BRIGHTNESS, ATT_DAMPING, ATT_POSITION,
    NUM_CHANNELS
  };

  Resonator() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configButton(POLYPHONY_BUTTON_PARAM, "Polyphony (hold 3 s: 3 voices)");
    configButton(RESONATOR_BUTTON_PARAM, "Resonator model (hold 3 s: bonus models)");
    configParam(FREQUENCY_PARAM, 0.f, 60.f, 30.f, "Frequency", "semitones");
    configParam(STRUCTURE_PARAM, 0.f, 1.f, 0.5f, "Structure", "%", 100.f);
    configParam(BRIGHTNESS_PARAM, 0.f, 1.f, 0.5f, "Brightness", "%", 100.f);
    configParam(DAMPING_PARAM, 0.f, 1.f, 0.5f, "Damping", "%", 100.f);
    configParam(POSITION_PARAM, 0.f, 1.f, 0.5f, "Position", "%", 100.f);
    configParam(BRIGHTNESS_MOD_PARAM, -1.f, 1.f, 0.f, "Brightness CV amount", "%", 100.f);
    configParam(FREQUENCY_MOD_PARAM, -1.f, 1.f, 0.f, "Frequency CV amount", "%", 100.f);
    configParam(DAMPING_MOD_PARAM, -1.f, 1.f, 0.f, "Damping CV amount", "%", 100.f);
    configParam(STRUCTURE_MOD_PARAM, -1.f, 1.f, 0.f, "Structure CV amount", "%", 100.f);
    configParam(POSITION_MOD_PARAM, -1.f, 1.f, 0.f, "Position CV amount", "%", 100.f);
    configSwitch(POLYPHONY_PARAM, 1.f, 4.f, 1.f, "Polyphony", {"1", "2", "3", "4"}).hidden = true;
    configSwitch(MODEL_PARAM, 0.f, 5.f, 0.f, "Resonator model",
                 std::vector<std::string>(kResonatorModels, kResonatorModels + 6))
        .hidden = true;

    configInput(BRIGHTNESS_MOD_INPUT, "Brightness CV");
    configInput(FREQUENCY_MOD_INPUT, "Frequency CV");
    configInput(DAMPING_MOD_INPUT, "Damping CV");
    configInput(STRUCTURE_MOD_INPUT, "Structure CV");
    configInput(POSITION_MOD_INPUT, "Position CV");
    configInput(STRUM_INPUT, "Strum");
    configInput(PITCH_INPUT, "V/Oct");
    configInput(IN_INPUT, "Audio in");
    configOutput(ODD_OUTPUT, "Odd");
    configOutput(EVEN_OUTPUT, "Even");

    initDsp();
    std::fill(lp_, lp_ + NUM_CHANNELS, 0.f);
    adapter_.init(48000.f, rings::kSampleRate, static_cast<int>(rings::kMaxBlockSize));
  }

  void onSampleRateChange(float sampleRate) override {
    adapter_.init(sampleRate, rings::kSampleRate, static_cast<int>(rings::kMaxBlockSize));
  }

  // The firmware's DSP objects are zero-initialised globals: do the same.
  void initDsp() {
    std::memset(reverbBuffer_, 0, sizeof(reverbBuffer_));
    std::memset(static_cast<void*>(&part_), 0, sizeof(part_));
    std::memset(static_cast<void*>(&stringSynth_), 0, sizeof(stringSynth_));
    std::memset(static_cast<void*>(&strummer_), 0, sizeof(strummer_));
    strummer_.Init(0.01f, rings::kSampleRate / rings::kMaxBlockSize);
    part_.Init(reverbBuffer_);
    stringSynth_.Init(reverbBuffer_);
    stringPolyphony_ = 0;  // re-applied by the next renderBlock()
  }

  void process(const ProcessArgs& args) override {
    handleButton(0, params[POLYPHONY_BUTTON_PARAM].value > 0.5f, args.sampleTime);
    handleButton(1, params[RESONATOR_BUTTON_PARAM].value > 0.5f, args.sampleTime);

    // The trigger input is read once per block by the firmware; latch edges.
    if (strumTrigger_.process(inputs[STRUM_INPUT].getVoltage())) strumPending_ = true;

    float in[1] = {inputs[IN_INPUT].getVoltage() / 5.f};
    float out[2];
    adapter_.process(in, out, [&](const float* nin, float* nout, int n) {
      renderBlock(nin, nout, n);
    });

    // Hardware normalisation: with a single jack patched, both signals are mixed.
    if (outputs[ODD_OUTPUT].isConnected() && outputs[EVEN_OUTPUT].isConnected()) {
      outputs[ODD_OUTPUT].setVoltage(clampf(out[0], -1.f, 1.f) * 5.f);
      outputs[EVEN_OUTPUT].setVoltage(clampf(out[1], -1.f, 1.f) * 5.f);
    } else {
      float v = clampf(out[0] + out[1], -1.f, 1.f) * 5.f;
      outputs[ODD_OUTPUT].setVoltage(v);
      outputs[EVEN_OUTPUT].setVoltage(v);
    }

    if ((args.frame & 63) == 0) updateLights(args.frame * 1000.0 / args.sampleRate);
  }

  void saveState(StateMap& s) const override { s["easterEgg"] = easterEgg_ ? "1" : "0"; }
  void loadState(const StateMap& s) override {
    auto it = s.find("easterEgg");
    if (it != s.end()) easterEgg_ = it->second == "1";
  }
  void onReset() override { easterEgg_ = false; }
  void appendMenu(std::vector<MenuAction>& items) override {
    items.push_back({"Disastrous Peace (string synth easter egg)",
                     [this] { return std::string(easterEgg_ ? "ON" : "OFF"); },
                     [this](int) { toggleEasterEgg(); }});
    items.push_back({"Current resonator / FX", [this] {
                       int m = static_cast<int>(params[MODEL_PARAM].value + 0.5f);
                       return std::string(easterEgg_ ? kEggFx[m] : kResonatorModels[m]);
                     },
                     [](int) {}});
  }

 private:
  // ui.cc: OnSwitchReleased(), with long presses firing after 3 s of holding.
  void handleButton(int i, bool pressed, float dt) {
    if (pressed) {
      if (!held_[i]) {
        held_[i] = true;
        holdTime_[i] = 0.f;
        longFired_[i] = false;
      } else if (!longFired_[i]) {
        holdTime_[i] += dt;
        if (holdTime_[i] >= 3.f) {
          longFired_[i] = true;
          onSwitch(i, true);
        }
      }
    } else if (held_[i]) {
      held_[i] = false;
      if (!longFired_[i]) onSwitch(i, false);
    }
  }

  void onSwitch(int i, bool longPress) {
    int polyphony = static_cast<int>(params[POLYPHONY_PARAM].value + 0.5f);
    int model = static_cast<int>(params[MODEL_PARAM].value + 0.5f);
    if (longPress && easterEggGesture()) {
      toggleEasterEgg();
      return;
    }
    if (i == 0) {
      if (longPress) {
        polyphony = 3;
      } else {
        if (polyphony == 3) polyphony = 2;
        polyphony <<= 1;
        if (polyphony > 4) polyphony = 1;
      }
      params[POLYPHONY_PARAM].set(static_cast<float>(polyphony));
    } else {
      if (longPress) {
        model = model >= 3 ? model - 3 : model + 3;
      } else {
        model = model >= 3 ? model - 3 : (model + 1) % 3;
      }
      params[MODEL_PARAM].set(static_cast<float>(model));
    }
  }

  void toggleEasterEgg() {
    easterEgg_ = !easterEgg_;
    eggAnimStart_ = -1.0;  // started by updateLights()
  }

  // CvScaler::easter_egg(): the secret knob positions checked on a long press.
  bool easterEggGesture() const {
    return lp_[POT_FREQUENCY] < 0.1f && lp_[POT_STRUCTURE] > 0.9f &&
           lp_[POT_BRIGHTNESS] < 0.1f && lp_[POT_POSITION] > 0.9f &&
           lp_[POT_DAMPING] > 0.4f && lp_[POT_DAMPING] < 0.6f &&
           lp_[ATT_BRIGHTNESS] < -1.f && lp_[ATT_FREQUENCY] > 1.f &&
           lp_[ATT_DAMPING] < -1.f && lp_[ATT_STRUCTURE] > 1.f && lp_[ATT_POSITION] < -1.f;
  }

  // CvScaler::Read(), with voltages in place of ADC codes (CV / 5 V, as in
  // the VCV Rack port).
  void readCv(rings::Patch* patch, rings::PerformanceState* ps) {
    float raw[NUM_CHANNELS];
    raw[CV_FREQUENCY] = inputs[FREQUENCY_MOD_INPUT].getVoltage() / 5.f;
    raw[CV_STRUCTURE] = inputs[STRUCTURE_MOD_INPUT].getVoltage() / 5.f;
    raw[CV_BRIGHTNESS] = inputs[BRIGHTNESS_MOD_INPUT].getVoltage() / 5.f;
    raw[CV_DAMPING] = inputs[DAMPING_MOD_INPUT].getVoltage() / 5.f;
    raw[CV_POSITION] = inputs[POSITION_MOD_INPUT].getVoltage() / 5.f;
    // The V/Oct ADC spans 7 octaves: with the default calibration
    // (cv_scaler.cc, settings.cc) note = 12 * V covers -17.6..66.7, i.e.
    // about -1.5..+5.5 V. Clamp to that like the hardware does.
    raw[CV_V_OCT] = clampf(inputs[PITCH_INPUT].getVoltage(), -1.5f, 5.5f);
    raw[POT_FREQUENCY] = params[FREQUENCY_PARAM].value / 60.f;
    raw[POT_STRUCTURE] = params[STRUCTURE_PARAM].value;
    raw[POT_BRIGHTNESS] = params[BRIGHTNESS_PARAM].value;
    raw[POT_DAMPING] = params[DAMPING_PARAM].value;
    raw[POT_POSITION] = params[POSITION_PARAM].value;
    raw[ATT_FREQUENCY] = quarticBipolar(params[FREQUENCY_MOD_PARAM].value);
    raw[ATT_STRUCTURE] = quadraticBipolar(params[STRUCTURE_MOD_PARAM].value);
    raw[ATT_BRIGHTNESS] = quadraticBipolar(params[BRIGHTNESS_MOD_PARAM].value);
    raw[ATT_DAMPING] = quadraticBipolar(params[DAMPING_MOD_PARAM].value);
    raw[ATT_POSITION] = quadraticBipolar(params[POSITION_MOD_PARAM].value);
    static const float kLp[NUM_CHANNELS] = {1.f,   0.1f,  0.1f,  0.05f, 0.01f, 1.f,
                                            0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.005f,
                                            0.005f, 0.005f, 0.005f, 0.005f};
    for (int i = 0; i < NUM_CHANNELS; ++i) lp_[i] += kLp[i] * (raw[i] - lp_[i]);

    auto attenuvert = [&](int cv, int att, int pot, float hi) {
      return clampf(lp_[cv] * lp_[att] + lp_[pot], 0.f, hi);
    };
    patch->structure = attenuvert(CV_STRUCTURE, ATT_STRUCTURE, POT_STRUCTURE, 0.9995f);
    patch->brightness = attenuvert(CV_BRIGHTNESS, ATT_BRIGHTNESS, POT_BRIGHTNESS, 1.f);
    // Just below 1: Resonator::ComputeFilters() reads lut_4_decades[257]
    // (257 entries) through Interpolate(..., damping, 256) at damping = 1.
    patch->damping = attenuvert(CV_DAMPING, ATT_DAMPING, POT_DAMPING, 0.9999f);
    patch->position = attenuvert(CV_POSITION, ATT_POSITION, POT_POSITION, 1.f);

    float fm = lp_[CV_FREQUENCY] * 48.f;
    float error = fm - fmCv_;
    if (std::fabs(error) >= 0.8f) {
      fmCv_ = fm;
    } else {
      fmCv_ += 0.02f * error;
    }
    ps->fm = clampf(fmCv_ * lp_[ATT_FREQUENCY], -48.f, 48.f);

    float transpose = 60.f * lp_[POT_FREQUENCY];
    float hysteresis = transpose - transpose_ > 0.f ? -0.3f : +0.3f;
    transpose_ = static_cast<float>(static_cast<int>(transpose + hysteresis + 0.5f));

    ps->note = lp_[CV_V_OCT] * 12.f;
    ps->tonic = 12.f + transpose_;

    // Normalisation probes -> jack detection.
    ps->internal_exciter = !inputs[IN_INPUT].isConnected();
    ps->internal_strum = !inputs[STRUM_INPUT].isConnected();
    ps->internal_note = !inputs[PITCH_INPUT].isConnected();
    ps->strum = strumPending_;
    strumPending_ = false;
    if (ps->internal_note) {
      // Remove quantization when nothing is plugged in the V/OCT input.
      ps->note = 0.f;
      ps->tonic = 12.f + transpose;
    }
    // The firmware computes SemitonesToRatio(note + tonic + fm + offset - 69)
    // with chord/sympathetic-string offsets of up to +-24 semitones, and
    // lut_pitch_ratio_high only covers -128..127. The filtered note stays in
    // -18..69 (V/Oct clamp above; NoteFilter starts at 69), so keep
    // tonic + fm within -16..102 and the sum can't leave the table. Only the
    // extreme corners of FREQUENCY + FM + V/Oct are affected; the hardware
    // reads past the table there.
    ps->fm = clampf(ps->fm, -16.f - ps->tonic, 102.f - ps->tonic);

    // Hysteresis on chord (from the unfiltered structure CV).
    float chord = raw[CV_STRUCTURE] * lp_[ATT_STRUCTURE] + lp_[POT_STRUCTURE];
    chord *= static_cast<float>(rings::kNumChords - 1);
    hysteresis = chord - chord_ > 0 ? -0.1f : +0.1f;
    chord_ = std::min(std::max(static_cast<int>(chord + hysteresis + 0.5f), 0),
                      static_cast<int>(rings::kNumChords - 1));
    ps->chord = chord_;
  }

  // rings.cc: FillBuffer().
  void renderBlock(const float* nin, float* nout, int n) {
    int polyphony = static_cast<int>(params[POLYPHONY_PARAM].value + 0.5f);
    int model = static_cast<int>(params[MODEL_PARAM].value + 0.5f);
    if (polyphony != part_.polyphony()) part_.set_polyphony(polyphony);
    if (polyphony != stringPolyphony_) {
      stringSynth_.set_polyphony(polyphony);
      stringPolyphony_ = polyphony;
    }
    part_.set_model(static_cast<rings::ResonatorModel>(model));
    stringSynth_.set_fx(static_cast<rings::FxType>(model));

    rings::Patch patch;
    rings::PerformanceState ps;
    readCv(&patch, &ps);

    const size_t size = static_cast<size_t>(n);
    if (easterEgg_) {
      for (int i = 0; i < n; ++i) in_[i] = nin[i];
      strummer_.Process(nullptr, size, &ps);
      stringSynth_.Process(ps, patch, in_, out_, aux_, size);
    } else {
      // Noise gate.
      for (int i = 0; i < n; ++i) {
        float s = nin[i];
        float error = s * s - inLevel_;
        inLevel_ += error * (error > 0.f ? 0.1f : 0.0001f);
        float gain = inLevel_ <= kNoiseGateThreshold ? (1.f / kNoiseGateThreshold) * inLevel_ : 1.f;
        in_[i] = gain * s;
      }
      strummer_.Process(in_, size, &ps);
      part_.Process(ps, patch, in_, out_, aux_, size);
    }
    // Safety net: a non-finite sample would latch in the resonators/reverb
    // forever. Re-initialise the DSP (cheap, and should never happen).
    bool finite = true;
    for (int i = 0; i < n; ++i) finite = finite && std::isfinite(out_[i]) && std::isfinite(aux_[i]);
    if (!finite) {
      initDsp();
      std::fill(out_, out_ + n, 0.f);
      std::fill(aux_, aux_ + n, 0.f);
    }
    for (int i = 0; i < n; ++i) {
      nout[2 * i] = out_[i];
      nout[2 * i + 1] = aux_[i];
    }
    if (ps.strum) strumFlag_ = true;
  }

  // ui.cc: Poll(), normal (non colour-blind) palette. PWM duty -> brightness.
  void updateLights(double ms) {
    int polyphony = static_cast<int>(params[POLYPHONY_PARAM].value + 0.5f);
    int model = static_cast<int>(params[MODEL_PARAM].value + 0.5f);
    int64_t t = static_cast<int64_t>(ms);
    float pr, pg, mr, mg;

    if (eggAnimStart_ < 0.0) eggAnimStart_ = ms;
    if (eggAnimStart_ > 0.0 && ms - eggAnimStart_ < 3000.0) {
      // Easter-egg intro / outro animation.
      auto tri = [](int64_t x) {
        int v = static_cast<int>(x & 31);
        return (v < 16 ? v : 31 - v) / 16.f;
      };
      if (easterEgg_) {
        float a = tri(t / 7), b = tri(t / 17);
        pr = a; pg = b; mr = b; mg = a;
      } else {
        float a = tri(t / 9), b = tri(t / 13);
        pr = a < 0.44f ? 1.f : 0.f; pg = 1.f - pr;
        mg = b < 0.44f ? 1.f : 0.f; mr = 1.f - mg;
      }
    } else {
      eggAnimStart_ = 0.0;
      int tr = static_cast<int>((t >> 5) & 31);
      float triangle = (tr < 16 ? tr : 31 - tr) / 16.f;
      pr = polyphony >= 2 ? 1.f : 0.f;
      pg = polyphony <= 2 ? 1.f : 0.f;
      mr = model >= 1 ? 1.f : 0.f;
      mg = model <= 1 ? 1.f : 0.f;
      if (polyphony == 3) {
        pr = 1.f;
        pg = triangle;
      }
      if (model >= 3) {
        mr = model >= 4 ? triangle : 0.f;
        mg = model <= 4 ? triangle : 0.f;
      }
      // Strumming flag: the polyphony LED blinks off on each strum.
      if (strumFlag_) {
        strumFlag_ = false;
        strumBlankUntil_ = ms + std::min(50.0, (ms - lastStrumMs_) * 0.25);
        lastStrumMs_ = ms;
      }
      if (ms < strumBlankUntil_) pr = pg = 0.f;
    }
    lights[POLY_RED_LIGHT] = pr;
    lights[POLY_GREEN_LIGHT] = pg;
    lights[MODEL_RED_LIGHT] = mr;
    lights[MODEL_GREEN_LIGHT] = mg;
    lights[OFF_LIGHT] = 0.f;
  }

  static constexpr float kNoiseGateThreshold = 0.00003f;

  uint16_t reverbBuffer_[32768];
  rings::Part part_;
  rings::StringSynthPart stringSynth_;
  rings::Strummer strummer_;
  BlockAdapter<1, 2> adapter_;
  float in_[rings::kMaxBlockSize];
  float out_[rings::kMaxBlockSize];
  float aux_[rings::kMaxBlockSize];
  float lp_[NUM_CHANNELS];
  float inLevel_ = 0.f;
  float fmCv_ = 0.f;
  float transpose_ = 0.f;
  int chord_ = 0;
  int stringPolyphony_ = 0;
  bool easterEgg_ = false;

  SchmittTrigger strumTrigger_;
  bool strumPending_ = false;
  bool strumFlag_ = false;
  double strumBlankUntil_ = 0.0, lastStrumMs_ = 0.0;
  double eggAnimStart_ = 0.0;

  bool held_[2] = {false, false};
  bool longFired_[2] = {false, false};
  float holdTime_[2] = {0.f, 0.f};
};

void buildPanel(PanelSpec& p) {
  p.hp = 14;
  p.bg = 0xFFF2F1EC;
  p.ink = 0xFF1E1E20;
  p.title = "RESONATOR";
  const uint32_t red = 0xFFC8414B;
  const uint32_t teal = 0xFF168A8C;
  const uint32_t ochre = 0xFFD9A43A;

  // Polyphony / resonator buttons with their LEDs and colour legends.
  p.button(Resonator::POLYPHONY_BUTTON_PARAM, 7.3f, 16.3f, 5.f, "", 0xFFD8D9DB);
  p.rgbLight(Resonator::POLY_RED_LIGHT, Resonator::POLY_GREEN_LIGHT, Resonator::OFF_LIGHT,
             13.7f, 16.3f, 3.f);
  p.button(Resonator::RESONATOR_BUTTON_PARAM, 63.4f, 16.3f, 5.f, "", 0xFFD8D9DB);
  p.rgbLight(Resonator::MODEL_RED_LIGHT, Resonator::MODEL_GREEN_LIGHT, Resonator::OFF_LIGHT,
             57.0f, 16.3f, 3.f);
  const uint32_t legend[3] = {teal, ochre, red};
  for (int i = 0; i < 3; ++i) {
    p.box(17.0f + i * 3.6f, 14.9f, 2.8f, 2.8f, legend[i], 1.4f);
    p.box(43.2f + i * 3.6f, 14.9f, 2.8f, 2.8f, legend[i], 1.4f);
  }
  p.label(21.0f, 20.6f, "1  2  4", 2.4f);
  p.label(47.2f, 20.6f, "MOD SYM STR", 2.2f);

  p.knob(Resonator::FREQUENCY_PARAM, 18.9f, 33.4f, 14.f);
  p.label(18.9f, 45.9f, "FREQUENCY", 3.3f);
  p.knob(Resonator::STRUCTURE_PARAM, 51.8f, 33.4f, 14.f);
  p.label(51.8f, 45.9f, "STRUCTURE", 3.3f);

  p.knob(Resonator::BRIGHTNESS_PARAM, 11.6f, 60.4f, 9.f);
  p.label(11.6f, 70.6f, "BRIGHTNESS", 3.0f);
  p.knob(Resonator::DAMPING_PARAM, 35.5f, 60.4f, 9.f);
  p.label(35.5f, 70.6f, "DAMPING", 3.0f);
  p.knob(Resonator::POSITION_PARAM, 59.2f, 60.4f, 9.f);
  p.label(59.2f, 70.6f, "POSITION", 3.0f);

  const float cols[5] = {9.4f, 22.4f, 35.5f, 48.4f, 61.4f};
  const int atts[5] = {Resonator::BRIGHTNESS_MOD_PARAM, Resonator::FREQUENCY_MOD_PARAM,
                       Resonator::DAMPING_MOD_PARAM, Resonator::STRUCTURE_MOD_PARAM,
                       Resonator::POSITION_MOD_PARAM};
  const int cvs[5] = {Resonator::BRIGHTNESS_MOD_INPUT, Resonator::FREQUENCY_MOD_INPUT,
                      Resonator::DAMPING_MOD_INPUT, Resonator::STRUCTURE_MOD_INPUT,
                      Resonator::POSITION_MOD_INPUT};
  for (int i = 0; i < 5; ++i) {
    p.label(cols[i] - 3.2f, 75.6f, "-", 3.0f);
    p.label(cols[i] + 3.2f, 75.6f, "+", 3.0f);
    p.knob(atts[i], cols[i], 80.7f, 6.5f);
    p.line(cols[i], 84.6f, cols[i], 92.4f, 0.3f, red);
    p.input(cvs[i], cols[i], 96.4f);
  }
  // Frequency / structure knobs feed the 2nd and 4th CV columns.
  p.line(20.4f, 49.5f, 22.4f, 76.0f, 0.3f, red);
  p.line(49.8f, 49.5f, 48.4f, 76.0f, 0.3f, red);

  p.box(42.6f, 102.2f, 24.6f, 15.0f, 0xFF5E5E60, 1.2f);
  p.input(Resonator::STRUM_INPUT, 9.4f, 111.2f, "STRUM");
  p.input(Resonator::PITCH_INPUT, 22.4f, 111.2f, "V/OCT");
  p.input(Resonator::IN_INPUT, 35.5f, 111.2f, "IN");
  p.output(Resonator::ODD_OUTPUT, 48.6f, 111.2f);
  p.output(Resonator::EVEN_OUTPUT, 61.2f, 111.2f);
  p.label(48.6f, 105.2f, "ODD", 2.7f, kAlignCenter, 0xFFFFFFFF);
  p.label(61.2f, 105.2f, "EVEN", 2.7f, kAlignCenter, 0xFFFFFFFF);
}

Model model = [] {
  Model m;
  m.slug = "resonator";
  m.name = "Resonator";
  m.shortName = "RESONATOR";
  m.basedOn = "Rings";
  m.category = Category::Resonator;
  m.description = "Modal, sympathetic-string and string resonator with internal exciter.";
  m.create = [] { return new Resonator(); };
  m.buildPanel = buildPanel;
  return m;
}();

BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
