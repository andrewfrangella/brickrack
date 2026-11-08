// BrickRack - GPL-3.0-or-later.
//
// BrickRack's own modules, which connect the rack to the handheld:
//   AUDIO OUT     - headphones / speaker
//   BRICK CONTROL - left stick X/Y and the L2/R2/L3/R3 buttons as CV/gates
//   MIC IN        - the built-in microphone, with envelope follower and gate
//   STEP SEQ      - 8-step pitch/gate sequencer with internal clock

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "core/engine.h"
#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"
#include "ui/canvas.h"

namespace br {
namespace {

const uint32_t kIoPanel = 0xFF2C2F35;
const uint32_t kIoInk = 0xFFE7E8EA;
const uint32_t kIoAccent = 0xFFFFC83D;

// ---------------------------------------------------------------------------
class AudioOut : public Module {
 public:
  enum { LEVEL_PARAM, NUM_PARAMS };
  enum { LEFT_INPUT, RIGHT_INPUT, NUM_INPUTS };
  enum { NUM_LIGHTS = 10 };  // 5 per channel

  AudioOut() {
    config(NUM_PARAMS, NUM_INPUTS, 0, NUM_LIGHTS);
    configParam(LEVEL_PARAM, 0.f, 1.f, 0.8f, "Level", "%", 100.f);
    configInput(LEFT_INPUT, "Left");
    configInput(RIGHT_INPUT, "Right");
  }

  void process(const ProcessArgs& args) override {
    float l = inputs[LEFT_INPUT].getVoltage();
    float r = inputs[RIGHT_INPUT].getNormalVoltage(l);
    if (!inputs[LEFT_INPUT].isConnected()) l = r;
    float g = params[LEVEL_PARAM].value * params[LEVEL_PARAM].value / 5.f;  // 5 V -> full scale
    l *= g;
    r *= g;
    args.engine->addToMaster(l, r);
    peak_[0] = std::max(std::fabs(l), peak_[0] * 0.9997f);
    peak_[1] = std::max(std::fabs(r), peak_[1] * 0.9997f);
    if ((args.frame & 127) == 0) {
      const float th[5] = {0.06f, 0.18f, 0.4f, 0.7f, 0.95f};
      for (int ch = 0; ch < 2; ++ch)
        for (int i = 0; i < 5; ++i)
          lights[static_cast<size_t>(ch * 5 + i)] = peak_[ch] > th[i] ? 1.f : 0.f;
    }
  }

 private:
  float peak_[2] = {0.f, 0.f};
};

void audioOutPanel(PanelSpec& p) {
  p.hp = 6;
  p.bg = kIoPanel;
  p.ink = kIoInk;
  p.title = "AUDIO OUT";
  p.label(15.24f, 14.f, "HEADPHONES / SPEAKER", 2.4f, kAlignCenter, 0xFF9DA1A8);
  p.knob(AudioOut::LEVEL_PARAM, 15.24f, 32.f, 13.f, "LEVEL", -9.5f, 0xFF4A4E57);
  const uint32_t cols[5] = {0xFF35E06A, 0xFF35E06A, 0xFF35E06A, 0xFFFFC83D, 0xFFFF3B30};
  for (int i = 0; i < 5; ++i) {
    float y = 70.f - i * 5.f;
    p.light(i, 10.f, y, cols[i], 2.6f);
    p.light(5 + i, 20.48f, y, cols[i], 2.6f);
  }
  p.label(10.f, 76.f, "L", 2.8f);
  p.label(20.48f, 76.f, "R", 2.8f);
  p.box(3.f, 88.f, 24.48f, 32.f, 0xFF1C1E22, 1.5f);
  p.input(AudioOut::LEFT_INPUT, 15.24f, 98.f, "LEFT", -6.0f);
  p.input(AudioOut::RIGHT_INPUT, 15.24f, 113.f, "RIGHT", -6.0f);
}

Model audioOutModel = [] {
  Model m;
  m.slug = "audio-out";
  m.name = "Audio Out";
  m.shortName = "AUDIO OUT";
  m.category = Category::IO;
  m.description = "Sends audio to the Brick's headphones/speaker. Right is normalled to left.";
  m.create = [] { return new AudioOut(); };
  m.buildPanel = audioOutPanel;
  return m;
}();
BR_REGISTER_MODEL(audioOutModel);

// ---------------------------------------------------------------------------
class BrickControl : public Module {
 public:
  enum { X_RANGE_PARAM, Y_RANGE_PARAM, SLEW_PARAM, NUM_PARAMS };
  enum { X_OUTPUT, Y_OUTPUT, L2_OUTPUT, R2_OUTPUT, L3_OUTPUT, R3_OUTPUT, NUM_OUTPUTS };
  enum { L2_LIGHT, R2_LIGHT, L3_LIGHT, R3_LIGHT, NUM_LIGHTS };

  BrickControl() {
    config(NUM_PARAMS, 0, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(X_RANGE_PARAM, -1.f, 1.f, 1.f, "Stick X range", "V", 5.f);
    configParam(Y_RANGE_PARAM, -1.f, 1.f, 1.f, "Stick Y range", "V", 5.f);
    configParam(SLEW_PARAM, 0.f, 1.f, 0.1f, "Stick smoothing", "%", 100.f);
    configOutput(X_OUTPUT, "Left stick X");
    configOutput(Y_OUTPUT, "Left stick Y");
    configOutput(L2_OUTPUT, "L2 gate");
    configOutput(R2_OUTPUT, "R2 gate");
    configOutput(L3_OUTPUT, "L3 gate");
    configOutput(R3_OUTPUT, "R3 gate");
  }

  void process(const ProcessArgs& args) override {
    auto& ctl = args.engine->controller();
    float tx = ctl.axis[0].load(std::memory_order_relaxed);
    float ty = -ctl.axis[1].load(std::memory_order_relaxed);  // up = positive
    float slew = params[SLEW_PARAM].value;
    float tau = 0.001f + slew * slew * 0.5f;
    float k = std::min(1.f, args.sampleTime / tau);
    x_ += (tx - x_) * k;
    y_ += (ty - y_) * k;
    outputs[X_OUTPUT].setVoltage(x_ * params[X_RANGE_PARAM].value * 5.f);
    outputs[Y_OUTPUT].setVoltage(y_ * params[Y_RANGE_PARAM].value * 5.f);
    for (int i = 0; i < 4; ++i) {
      bool g = ctl.gate[i].load(std::memory_order_relaxed);
      outputs[static_cast<size_t>(L2_OUTPUT + i)].setVoltage(g ? 10.f : 0.f);
      lights[static_cast<size_t>(i)] = g ? 1.f : 0.f;
    }
  }

  float x_ = 0.f, y_ = 0.f;
};

void brickControlPanel(PanelSpec& p) {
  p.hp = 8;
  p.bg = kIoPanel;
  p.ink = kIoInk;
  p.title = "BRICK CONTROL";
  p.display(5.f, 12.f, 30.64f, 30.64f, [](Canvas& c, float x, float y, float w, float h, float s,
                                           const Module* m) {
    auto* bc = static_cast<const BrickControl*>(m);
    c.fillRoundRect(x, y, w, h, 1.5f * s, 0xFF15171A);
    c.line(x + w * 0.5f, y + 4, x + w * 0.5f, y + h - 4, 1.f, 0xFF3A3E45);
    c.line(x + 4, y + h * 0.5f, x + w - 4, y + h * 0.5f, 1.f, 0xFF3A3E45);
    float px = x + w * 0.5f + bc->x_ * (w * 0.5f - 6.f);
    float py = y + h * 0.5f - bc->y_ * (h * 0.5f - 6.f);
    c.fillCircle(px, py, 2.0f * s, withAlpha(kIoAccent, 0.3f));
    c.fillCircle(px, py, 1.1f * s, kIoAccent);
    c.text(x + 3, y + 8, "LEFT STICK", std::max(11.f, 2.2f * s), 0xFF7D828A, kAlignLeft);
  });
  p.knob(BrickControl::X_RANGE_PARAM, 9.f, 53.f, 7.f, "X RANGE", -6.2f);
  p.knob(BrickControl::Y_RANGE_PARAM, 20.32f, 53.f, 7.f, "Y RANGE", -6.2f);
  p.knob(BrickControl::SLEW_PARAM, 31.64f, 53.f, 7.f, "SMOOTH", -6.2f);
  p.box(3.f, 64.f, 34.64f, 57.f, 0xFF1C1E22, 1.5f);
  p.output(BrickControl::X_OUTPUT, 12.f, 73.f, "X", -5.4f);
  p.output(BrickControl::Y_OUTPUT, 28.64f, 73.f, "Y", -5.4f);
  p.output(BrickControl::L2_OUTPUT, 12.f, 92.f, "L2", -5.4f);
  p.output(BrickControl::R2_OUTPUT, 28.64f, 92.f, "R2", -5.4f);
  p.output(BrickControl::L3_OUTPUT, 12.f, 111.f, "L3", -5.4f);
  p.output(BrickControl::R3_OUTPUT, 28.64f, 111.f, "R3", -5.4f);
  p.light(BrickControl::L2_LIGHT, 18.f, 86.6f, kIoAccent, 2.f);
  p.light(BrickControl::R2_LIGHT, 34.6f, 86.6f, kIoAccent, 2.f);
  p.light(BrickControl::L3_LIGHT, 18.f, 105.6f, kIoAccent, 2.f);
  p.light(BrickControl::R3_LIGHT, 34.6f, 105.6f, kIoAccent, 2.f);
}

Model brickControlModel = [] {
  Model m;
  m.slug = "brick-control";
  m.name = "Brick Control";
  m.shortName = "BRICK CONTROL";
  m.category = Category::IO;
  m.description = "Left stick as X/Y voltages; L2, R2, L3, R3 as 10 V gates.";
  m.create = [] { return new BrickControl(); };
  m.buildPanel = brickControlPanel;
  return m;
}();
BR_REGISTER_MODEL(brickControlModel);

// ---------------------------------------------------------------------------
class MicIn : public Module {
 public:
  enum { GAIN_PARAM, THRESHOLD_PARAM, NUM_PARAMS };
  enum { AUDIO_OUTPUT, ENV_OUTPUT, GATE_OUTPUT, NUM_OUTPUTS };
  enum { GATE_LIGHT, LEVEL_LIGHT, NUM_LIGHTS };

  MicIn() {
    config(NUM_PARAMS, 0, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(GAIN_PARAM, 0.f, 1.f, 0.5f, "Gain", "dB", 40.f);
    configParam(THRESHOLD_PARAM, 0.f, 1.f, 0.3f, "Gate threshold", "%", 100.f);
    configOutput(AUDIO_OUTPUT, "Audio");
    configOutput(ENV_OUTPUT, "Envelope");
    configOutput(GATE_OUTPUT, "Gate");
  }

  void process(const ProcessArgs& args) override {
    float in = args.engine->deviceInput(0);
    float gain = std::pow(10.f, params[GAIN_PARAM].value * 40.f / 20.f);
    float a = in * gain * 5.f;
    a = 10.f * std::tanh(a / 10.f);
    outputs[AUDIO_OUTPUT].setVoltage(a);
    float rect = std::fabs(a);
    float coef = rect > env_ ? 0.01f : 0.0004f;
    env_ += (rect - env_) * coef;
    outputs[ENV_OUTPUT].setVoltage(std::min(10.f, env_ * 2.f));
    float th = 0.05f + params[THRESHOLD_PARAM].value * 4.f;
    if (gate_) {
      if (env_ < th * 0.6f) gate_ = false;
    } else if (env_ > th) {
      gate_ = true;
    }
    outputs[GATE_OUTPUT].setVoltage(gate_ ? 10.f : 0.f);
    lights[GATE_LIGHT] = gate_ ? 1.f : 0.f;
    lights[LEVEL_LIGHT] = std::min(1.f, env_ / 5.f);
    available_ = args.engine->deviceInputAvailable();
  }

  float env_ = 0.f;
  bool gate_ = false;
  bool available_ = false;
};

void micInPanel(PanelSpec& p) {
  p.hp = 6;
  p.bg = kIoPanel;
  p.ink = kIoInk;
  p.title = "MIC IN";
  p.knob(MicIn::GAIN_PARAM, 15.24f, 30.f, 13.f, "GAIN", -9.5f, 0xFF4A4E57);
  p.knob(MicIn::THRESHOLD_PARAM, 15.24f, 56.f, 8.f, "THRESHOLD", -6.8f);
  p.light(MicIn::LEVEL_LIGHT, 9.f, 68.f, 0xFF35E06A, 2.6f);
  p.light(MicIn::GATE_LIGHT, 21.48f, 68.f, kIoAccent, 2.6f);
  p.label(9.f, 72.6f, "LEVEL", 2.4f);
  p.label(21.48f, 72.6f, "GATE", 2.4f);
  p.display(3.f, 76.f, 24.48f, 6.f, [](Canvas& c, float x, float y, float w, float h, float s,
                                       const Module* m) {
    bool ok = static_cast<const MicIn*>(m)->available_;
    c.text(x + w * 0.5f, y + h * 0.5f, ok ? "MIC ACTIVE" : "NO MIC INPUT",
           std::max(11.f, 2.3f * s), ok ? 0xFF35E06A : 0xFFFF8A65, kAlignCenter);
  });
  p.box(3.f, 84.f, 24.48f, 37.f, 0xFF1C1E22, 1.5f);
  p.output(MicIn::AUDIO_OUTPUT, 15.24f, 93.f, "AUDIO", -5.4f);
  p.output(MicIn::ENV_OUTPUT, 9.f, 112.f, "ENV", -5.4f);
  p.output(MicIn::GATE_OUTPUT, 21.48f, 112.f, "GATE", -5.4f);
}

Model micInModel = [] {
  Model m;
  m.slug = "mic-in";
  m.name = "Mic In";
  m.shortName = "MIC IN";
  m.category = Category::IO;
  m.description = "Built-in microphone with gain, envelope follower and gate (Ears-style).";
  m.create = [] { return new MicIn(); };
  m.buildPanel = micInPanel;
  return m;
}();
BR_REGISTER_MODEL(micInModel);

// ---------------------------------------------------------------------------
class StepSeq : public Module {
 public:
  enum {
    TEMPO_PARAM,
    LENGTH_PARAM,
    GATE_LEN_PARAM,
    SCALE_PARAM,
    RUN_PARAM,
    PITCH_PARAM,               // 8 params
    GATE_PARAM = PITCH_PARAM + 8,  // 8 toggles
    NUM_PARAMS = GATE_PARAM + 8
  };
  enum { CLOCK_INPUT, RESET_INPUT, TRANSPOSE_INPUT, NUM_INPUTS };
  enum { PITCH_OUTPUT, GATE_OUTPUT, CLOCK_OUTPUT, NUM_OUTPUTS };
  enum { STEP_LIGHT = 0, GATE_LIGHT = 8, RUN_LIGHT = 16, NUM_LIGHTS };

  StepSeq() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(TEMPO_PARAM, 30.f, 300.f, 110.f, "Tempo", "BPM").displayPrecision = 0;
    configSwitch(LENGTH_PARAM, 1.f, 8.f, 8.f, "Steps");
    configParam(GATE_LEN_PARAM, 0.05f, 0.95f, 0.5f, "Gate length", "%", 100.f).displayPrecision = 0;
    configSwitch(SCALE_PARAM, 0.f, 4.f, 2.f, "Quantizer",
                 {"Off", "Chromatic", "Major", "Minor", "Pentatonic"});
    configToggle(RUN_PARAM, "Run", true);
    const float defaults[8] = {0.f, 0.25f, 0.58f, 0.25f, 0.83f, 0.58f, 0.25f, 1.0f};
    for (int i = 0; i < 8; ++i) {
      char name[32];
      std::snprintf(name, sizeof(name), "Step %d pitch", i + 1);
      configParam(PITCH_PARAM + i, 0.f, 2.f, defaults[i], name, "V");
      std::snprintf(name, sizeof(name), "Step %d gate", i + 1);
      configToggle(GATE_PARAM + i, name, i != 3);
    }
    configInput(CLOCK_INPUT, "Clock");
    configInput(RESET_INPUT, "Reset");
    configInput(TRANSPOSE_INPUT, "Transpose");
    configOutput(PITCH_OUTPUT, "Pitch");
    configOutput(GATE_OUTPUT, "Gate");
    configOutput(CLOCK_OUTPUT, "Clock");
  }

  static float quantize(float v, int scale) {
    if (scale == 0) return v;
    static const int major[7] = {0, 2, 4, 5, 7, 9, 11};
    static const int minor[7] = {0, 2, 3, 5, 7, 8, 10};
    static const int penta[5] = {0, 2, 4, 7, 9};
    float semis = v * 12.f;
    if (scale == 1) return std::round(semis) / 12.f;
    const int* tbl = scale == 2 ? major : (scale == 3 ? minor : penta);
    int n = scale == 4 ? 5 : 7;
    float oct = std::floor(semis / 12.f);
    float within = semis - oct * 12.f;
    float best = 0.f, bestD = 99.f;
    for (int i = 0; i <= n; ++i) {
      float cand = i == n ? 12.f : static_cast<float>(tbl[i]);
      float d = std::fabs(cand - within);
      if (d < bestD) {
        bestD = d;
        best = cand;
      }
    }
    return (oct * 12.f + best) / 12.f;
  }

  void process(const ProcessArgs& args) override {
    bool running = params[RUN_PARAM].value > 0.5f;
    int length = static_cast<int>(params[LENGTH_PARAM].value + 0.5f);
    bool tick = false;
    if (inputs[CLOCK_INPUT].isConnected()) {
      tick = clockIn_.process(inputs[CLOCK_INPUT].getVoltage());
      if (tick) {
        periodEst_ = std::max(0.01f, samplesSinceTick_ * args.sampleTime);
        samplesSinceTick_ = 0;
      }
      ++samplesSinceTick_;
    } else if (running) {
      phase_ += params[TEMPO_PARAM].value / 60.f * 4.f * args.sampleTime;  // 16ths
      if (phase_ >= 1.f) {
        phase_ -= 1.f;
        tick = true;
      }
      periodEst_ = 60.f / (params[TEMPO_PARAM].value * 4.f);
    }
    if (resetIn_.process(inputs[RESET_INPUT].getVoltage())) {
      step_ = -1;
      phase_ = 0.f;
    }
    if (tick && running) {
      step_ = (step_ + 1) % std::max(1, length);
      gateTimer_ = 0.f;
      clockTimer_ = 0.f;
      int scale = static_cast<int>(params[SCALE_PARAM].value + 0.5f);
      pitch_ = quantize(params[PITCH_PARAM + step_].value, scale);
      stepOn_ = params[GATE_PARAM + step_].value > 0.5f;
    }
    gateTimer_ += args.sampleTime;
    clockTimer_ += args.sampleTime;
    bool gate = step_ >= 0 && stepOn_ && running &&
                gateTimer_ < periodEst_ * params[GATE_LEN_PARAM].value;
    outputs[GATE_OUTPUT].setVoltage(gate ? 10.f : 0.f);
    outputs[PITCH_OUTPUT].setVoltage(pitch_ + inputs[TRANSPOSE_INPUT].getVoltage());
    outputs[CLOCK_OUTPUT].setVoltage(clockTimer_ < 0.005f && running ? 10.f : 0.f);
    if ((args.frame & 63) == 0) {
      for (int i = 0; i < 8; ++i) {
        lights[static_cast<size_t>(STEP_LIGHT + i)] = (i == step_) ? 1.f : (i < length ? 0.08f : 0.f);
        lights[static_cast<size_t>(GATE_LIGHT + i)] = params[GATE_PARAM + i].value > 0.5f ? 0.9f : 0.f;
      }
      lights[RUN_LIGHT] = running ? 1.f : 0.f;
    }
  }

  void onReset() override {
    step_ = -1;
    phase_ = 0.f;
  }

 private:
  SchmittTrigger clockIn_, resetIn_;
  float phase_ = 0.f;
  int step_ = -1;
  float gateTimer_ = 0.f, clockTimer_ = 1.f;
  float periodEst_ = 0.13f;
  int samplesSinceTick_ = 0;
  float pitch_ = 0.f;
  bool stepOn_ = false;
};

void stepSeqPanel(PanelSpec& p) {
  p.hp = 14;
  p.bg = kIoPanel;
  p.ink = kIoInk;
  p.title = "STEP SEQ";
  const float w = 14 * kHpMm;
  p.knob(StepSeq::TEMPO_PARAM, 12.f, 24.f, 11.f, "TEMPO", -8.2f, 0xFF4A4E57);
  p.knob(StepSeq::GATE_LEN_PARAM, 30.f, 24.f, 7.5f, "GATE", -6.4f);
  p.knob(StepSeq::LENGTH_PARAM, 44.f, 24.f, 7.5f, "STEPS", -6.4f);
  p.knob(StepSeq::SCALE_PARAM, 58.f, 24.f, 7.5f, "SCALE", -6.4f);
  p.button(StepSeq::RUN_PARAM, 58.f, 37.5f, 4.5f, "", kIoAccent, StepSeq::RUN_LIGHT);
  p.label(50.f, 37.5f, "RUN", 2.6f);
  for (int i = 0; i < 8; ++i) {
    int row = i / 4, col = i % 4;
    float x = 10.5f + col * 16.f;
    float y = 52.f + row * 27.f;
    char n[4];
    std::snprintf(n, sizeof(n), "%d", i + 1);
    p.light(StepSeq::STEP_LIGHT + i, x - 5.5f, y - 7.2f, kIoAccent, 2.0f);
    p.label(x + 0.5f, y - 7.2f, n, 2.8f);
    p.knob(StepSeq::PITCH_PARAM + i, x, y, 9.f);
    p.button(StepSeq::GATE_PARAM + i, x, y + 10.f, 4.2f, "", 0xFF8F949C, StepSeq::GATE_LIGHT + i);
  }
  p.box(3.f, 98.f, w - 6.f, 24.f, 0xFF1C1E22, 1.5f);
  p.input(StepSeq::CLOCK_INPUT, 9.f, 113.f, "CLOCK", -5.6f);
  p.input(StepSeq::RESET_INPUT, 20.f, 113.f, "RESET", -5.6f);
  p.input(StepSeq::TRANSPOSE_INPUT, 31.f, 113.f, "TRANSP", -5.6f);
  p.output(StepSeq::CLOCK_OUTPUT, 42.f, 113.f, "CLK", -5.6f);
  p.output(StepSeq::GATE_OUTPUT, 52.5f, 113.f, "GATE", -5.6f);
  p.output(StepSeq::PITCH_OUTPUT, 63.f, 113.f, "PITCH", -5.6f);
}

Model stepSeqModel = [] {
  Model m;
  m.slug = "step-sequencer";
  m.name = "Step Sequencer";
  m.shortName = "STEP SEQ";
  m.category = Category::Sequencer;
  m.description = "8 steps of pitch and gate with internal clock, quantizer and reset.";
  m.create = [] { return new StepSeq(); };
  m.buildPanel = stepSeqPanel;
  return m;
}();
BR_REGISTER_MODEL(stepSeqModel);

}  // namespace
}  // namespace br
