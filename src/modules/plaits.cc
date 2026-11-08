// BrickRack - GPL-3.0-or-later.
//
// Macro Oscillator 2: wraps the Plaits voice (plaits/dsp/voice.cc) exactly as
// the firmware drives it: 12-sample blocks at 48 kHz, Patch/Modulations
// structures filled from the panel and the CV inputs.

#include <algorithm>
#include <cstring>

#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"
#include "plaits/dsp/voice.h"
#include "stmlib/utils/buffer_allocator.h"
#include "ui/canvas.h"

namespace br {
namespace {

const char* kEngineNames[24] = {
    "VA + filter",     "Phase distortion", "6-op FM I",      "6-op FM II",
    "6-op FM III",     "Wave terrain",     "String machine", "Chiptune",
    "Virtual analog",  "Waveshaping",      "2-op FM",        "Granular formant",
    "Harmonic",        "Wavetable",        "Chords",         "Speech",
    "Granular cloud",  "Filtered noise",   "Particle noise", "Inharmonic string",
    "Modal resonator", "Analog kick",      "Analog snare",   "Analog hi-hat",
};

class MacroOsc2 : public Module {
 public:
  enum ParamIds {
    FREQ_PARAM,
    HARMONICS_PARAM,
    TIMBRE_PARAM,
    MORPH_PARAM,
    TIMBRE_CV_PARAM,
    FREQ_CV_PARAM,
    MORPH_CV_PARAM,
    MODEL1_PARAM,
    MODEL2_PARAM,
    ENGINE_PARAM,   // hidden: selected engine 0..23
    LPG_COLOUR_PARAM,  // hidden
    DECAY_PARAM,       // hidden
    NUM_PARAMS
  };
  enum InputIds {
    ENGINE_INPUT,
    TIMBRE_INPUT,
    FREQ_INPUT,
    MORPH_INPUT,
    HARMONICS_INPUT,
    TRIGGER_INPUT,
    LEVEL_INPUT,
    NOTE_INPUT,
    NUM_INPUTS
  };
  enum OutputIds { OUT_OUTPUT, AUX_OUTPUT, NUM_OUTPUTS };
  enum LightIds { ENUMS_LIGHT_GREEN = 0, NUM_LIGHTS = 16 };  // [row*2 + {green, red}]

  MacroOsc2() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(FREQ_PARAM, -4.f, 4.f, 0.f, "Frequency", "semitones", 12.f);
    configParam(HARMONICS_PARAM, 0.f, 1.f, 0.5f, "Harmonics", "%", 100.f);
    configParam(TIMBRE_PARAM, 0.f, 1.f, 0.5f, "Timbre", "%", 100.f);
    configParam(MORPH_PARAM, 0.f, 1.f, 0.5f, "Morph", "%", 100.f);
    configParam(TIMBRE_CV_PARAM, -1.f, 1.f, 0.f, "Timbre CV amount", "%", 100.f);
    configParam(FREQ_CV_PARAM, -1.f, 1.f, 0.f, "FM amount", "%", 100.f);
    configParam(MORPH_CV_PARAM, -1.f, 1.f, 0.f, "Morph CV amount", "%", 100.f);
    configButton(MODEL1_PARAM, "Model: green bank (next)");
    configButton(MODEL2_PARAM, "Model: red bank (next)");
    {
      std::vector<std::string> names(kEngineNames, kEngineNames + 24);
      Param& p = configSwitch(ENGINE_PARAM, 0.f, 23.f, 8.f, "Model", names);
      p.hidden = true;
    }
    configParam(LPG_COLOUR_PARAM, 0.f, 1.f, 0.5f, "LPG colour", "%", 100.f).hidden = true;
    configParam(DECAY_PARAM, 0.f, 1.f, 0.5f, "LPG decay", "%", 100.f).hidden = true;

    configInput(ENGINE_INPUT, "Model");
    configInput(TIMBRE_INPUT, "Timbre");
    configInput(FREQ_INPUT, "FM");
    configInput(MORPH_INPUT, "Morph");
    configInput(HARMONICS_INPUT, "Harmonics");
    configInput(TRIGGER_INPUT, "Trigger");
    configInput(LEVEL_INPUT, "Level");
    configInput(NOTE_INPUT, "V/Oct");
    configOutput(OUT_OUTPUT, "Out");
    configOutput(AUX_OUTPUT, "Aux");

    std::memset(sharedBuffer_, 0, sizeof(sharedBuffer_));
    stmlib::BufferAllocator allocator(sharedBuffer_, sizeof(sharedBuffer_));
    voice_.Init(&allocator);
    std::memset(&patch_, 0, sizeof(patch_));
    std::memset(&modulations_, 0, sizeof(modulations_));
  }

  void process(const ProcessArgs& args) override {
    if (button1_.process(params[MODEL1_PARAM].value)) navigate(0);
    if (button2_.process(params[MODEL2_PARAM].value)) navigate(1);

    if (pos_ >= plaits::kBlockSize) {
      pos_ = 0;
      renderBlock();
    }
    outputs[OUT_OUTPUT].setVoltage(frames_[pos_].out / 32768.f * 5.f);
    outputs[AUX_OUTPUT].setVoltage(frames_[pos_].aux / 32768.f * 5.f);
    ++pos_;

    // LEDs: active engine steady, selected engine blinking (like the hardware).
    if ((args.frame & 255) == 0) updateLights(args.frame);
  }

  void saveState(StateMap& s) const override { s["altNavigation"] = altNavigation_ ? "1" : "0"; }
  void loadState(const StateMap& s) override {
    auto it = s.find("altNavigation");
    bool saved = it != s.end() && it->second == "1";
    // ui.cc LoadState(): the yellow bank (engines 0-7) can only be reached
    // with the alternate navigation, so it is enabled whenever the stored
    // engine is in it. (Params are loaded before the state.)
    altNavigation_ = saved || selectedEngine() < 8;
  }
  void appendMenu(std::vector<MenuAction>& items) override {
    items.push_back({"Buttons step through all 24 models",
                     [this] { return std::string(altNavigation_ ? "ON" : "OFF"); },
                     [this](int) { altNavigation_ = !altNavigation_; }});
  }
  void onReset() override { altNavigation_ = false; }

  int activeEngine() const { return activeEngine_; }

 private:
  int selectedEngine() const { return static_cast<int>(params[ENGINE_PARAM].value + 0.5f); }

  void navigate(int button) {
    int engine = selectedEngine();
    // The engine can also be picked from the menu: from the yellow bank, use
    // the alternate navigation like the firmware would after its next boot
    // (see loadState()); otherwise the buttons could never get back to it.
    if (engine < 8) altNavigation_ = true;
    if (altNavigation_) {
      engine = (engine + (button == 0 ? 23 : 1)) % 24;
    } else {
      int newBank = button + 1;
      int bank = engine / 8;
      int row = engine % 8;
      int cycle = newBank == bank ? 1 : 0;
      engine = newBank * 8 + (row + cycle) % 8;
    }
    params[ENGINE_PARAM].set(static_cast<float>(engine));
  }

  void renderBlock() {
    patch_.engine = static_cast<int>(params[ENGINE_PARAM].value + 0.5f);
    patch_.note = 60.f + params[FREQ_PARAM].value * 12.f;
    patch_.harmonics = params[HARMONICS_PARAM].value;
    patch_.timbre = params[TIMBRE_PARAM].value;
    patch_.morph = params[MORPH_PARAM].value;
    patch_.lpg_colour = params[LPG_COLOUR_PARAM].value;
    patch_.decay = params[DECAY_PARAM].value;
    patch_.frequency_modulation_amount = params[FREQ_CV_PARAM].value;
    patch_.timbre_modulation_amount = params[TIMBRE_CV_PARAM].value;
    patch_.morph_modulation_amount = params[MORPH_CV_PARAM].value;

    // Same voltage scaling the firmware's calibration produces.
    modulations_.engine = inputs[ENGINE_INPUT].getVoltage() / 5.f;
    modulations_.note = inputs[NOTE_INPUT].getVoltage() * 12.f;
    modulations_.frequency = inputs[FREQ_INPUT].getVoltage() * 6.f;
    modulations_.harmonics = inputs[HARMONICS_INPUT].getVoltage() / 5.f;
    modulations_.timbre = inputs[TIMBRE_INPUT].getVoltage() / 8.f;
    modulations_.morph = inputs[MORPH_INPUT].getVoltage() / 8.f;
    modulations_.trigger = inputs[TRIGGER_INPUT].getVoltage() / 3.f;
    // Just below full scale: at level 1 the compressed level (accent) is
    // exactly 1 and the 6-op engines' NormalizeVelocity() reads
    // lut_cube_root[17] (17 entries).
    modulations_.level = std::min(inputs[LEVEL_INPUT].getVoltage() / 8.f, 0.999f);
    modulations_.frequency_patched = inputs[FREQ_INPUT].isConnected();
    modulations_.timbre_patched = inputs[TIMBRE_INPUT].isConnected();
    modulations_.morph_patched = inputs[MORPH_INPUT].isConnected();
    modulations_.trigger_patched = inputs[TRIGGER_INPUT].isConnected();
    modulations_.level_patched = inputs[LEVEL_INPUT].isConnected();

    voice_.Render(patch_, modulations_, frames_, plaits::kBlockSize);
    activeEngine_ = voice_.active_engine();
  }

  void updateLights(int64_t frame) {
    int selected = static_cast<int>(params[ENGINE_PARAM].value + 0.5f);
    int active = activeEngine_ < 0 ? selected : activeEngine_;
    float blink = ((frame / 9600) & 1) ? 1.f : 0.25f;
    for (int i = 0; i < 16; ++i) lights[static_cast<size_t>(i)] = 0.f;
    auto setRow = [&](int engine, float b) {
      int row = engine % 8, bank = engine / 8;
      // bank 0 = yellow (green + red), bank 1 = green, bank 2 = red
      float g = (bank == 0 || bank == 1) ? b : 0.f;
      float r = (bank == 0 || bank == 2) ? b : 0.f;
      lights[static_cast<size_t>(row * 2)] = std::max(lights[static_cast<size_t>(row * 2)], g);
      lights[static_cast<size_t>(row * 2 + 1)] = std::max(lights[static_cast<size_t>(row * 2 + 1)], r);
    };
    setRow(active, 1.f);
    if (selected != active) setRow(selected, blink * 0.6f);
  }

  char sharedBuffer_[16384];
  plaits::Voice voice_;
  plaits::Patch patch_;
  plaits::Modulations modulations_;
  plaits::Voice::Frame frames_[plaits::kBlockSize];
  size_t pos_ = plaits::kBlockSize;
  int activeEngine_ = -1;
  bool altNavigation_ = false;
  ButtonEdge button1_, button2_;
};

void buildPanel(PanelSpec& p) {
  p.hp = 12;
  p.bg = 0xFFF2F1EC;
  p.ink = 0xFF1E1E20;
  const uint32_t red = 0xFFC8414B;
  const uint32_t teal = 0xFF168A8C;
  p.title = "MACRO OSC 2";

  // Model buttons and LED ladder.
  p.button(MacroOsc2::MODEL1_PARAM, 25.7f, 17.1f, 5.6f, "", 0xFFD8D9DB);
  p.button(MacroOsc2::MODEL2_PARAM, 34.6f, 17.1f, 5.6f, "", 0xFFD8D9DB);
  const float rows[8] = {24.0f, 29.4f, 34.8f, 40.2f, 45.6f, 51.0f, 56.4f, 61.8f};
  for (int i = 0; i < 8; ++i) {
    p.box(23.6f, rows[i] - 1.5f, 3.0f, 3.0f, teal, 1.5f);
    p.box(35.6f, rows[i] - 1.5f, 3.0f, 3.0f, red, 1.5f);
    p.bicolorLight(i * 2, i * 2 + 1, 30.15f, rows[i], 0xFF35E06A, 0xFFFF3B30, 3.0f);
  }

  p.knob(MacroOsc2::FREQ_PARAM, 12.3f, 29.3f, 13.f);
  p.label(12.3f, 42.1f, "FREQUENCY", 3.3f);
  p.knob(MacroOsc2::HARMONICS_PARAM, 48.3f, 29.3f, 13.f);
  p.label(48.3f, 42.1f, "HARMONICS", 3.3f);
  p.knob(MacroOsc2::TIMBRE_PARAM, 11.1f, 55.0f, 10.f);
  p.label(11.1f, 66.9f, "TIMBRE", 3.3f);
  p.knob(MacroOsc2::MORPH_PARAM, 49.7f, 55.0f, 10.f);
  p.label(49.7f, 66.9f, "MORPH", 3.3f);

  p.label(30.3f, 72.6f, "FM", 3.3f);
  p.knob(MacroOsc2::TIMBRE_CV_PARAM, 11.1f, 80.4f, 6.5f);
  p.knob(MacroOsc2::FREQ_CV_PARAM, 30.3f, 80.4f, 6.5f);
  p.knob(MacroOsc2::MORPH_CV_PARAM, 49.7f, 80.4f, 6.5f);
  for (float x : {11.1f, 30.3f, 49.7f}) {
    p.label(x - 4.6f, 76.0f, "-", 3.0f);
    p.label(x + 4.6f, 76.0f, "+", 3.0f);
  }
  p.line(11.1f, 84.f, 18.6f, 92.6f, 0.3f, red);
  p.line(30.3f, 84.f, 30.3f, 92.6f, 0.3f, red);
  p.line(49.7f, 84.f, 42.3f, 92.6f, 0.3f, red);

  p.label(7.4f, 90.2f, "MODEL", 3.0f);
  p.label(53.1f, 90.2f, "HARMO", 3.0f);
  p.input(MacroOsc2::ENGINE_INPUT, 7.4f, 96.7f);
  p.input(MacroOsc2::TIMBRE_INPUT, 18.6f, 96.7f);
  p.input(MacroOsc2::FREQ_INPUT, 30.3f, 96.7f);
  p.input(MacroOsc2::MORPH_INPUT, 42.3f, 96.7f);
  p.input(MacroOsc2::HARMONICS_INPUT, 53.1f, 96.7f);

  p.box(36.9f, 102.4f, 21.6f, 15.2f, 0xFF5E5E60, 1.2f);
  p.input(MacroOsc2::TRIGGER_INPUT, 7.4f, 111.1f, "TRIG");
  p.input(MacroOsc2::LEVEL_INPUT, 18.6f, 111.1f, "LEVEL");
  p.input(MacroOsc2::NOTE_INPUT, 30.3f, 111.1f, "V/OCT");
  p.output(MacroOsc2::OUT_OUTPUT, 42.3f, 111.1f);
  p.output(MacroOsc2::AUX_OUTPUT, 53.1f, 111.1f);
  p.label(42.3f, 105.0f, "OUT", 3.0f, kAlignCenter, 0xFFFFFFFF);
  p.label(53.1f, 105.0f, "AUX", 3.0f, kAlignCenter, 0xFFFFFFFF);

  // Engine name readout under the title (handy on a small screen).
  p.display(3.f, 9.6f, 54.96f, 4.4f, [](Canvas& c, float x, float y, float w, float h, float s,
                                        const Module* m) {
    int e = static_cast<const MacroOsc2*>(m)->activeEngine();
    if (e < 0) e = static_cast<int>(m->params[MacroOsc2::ENGINE_PARAM].value + 0.5f);
    c.text(x + w * 0.5f, y + h * 0.5f, kEngineNames[e], std::max(12.f, 2.9f * s), 0xFF168A8C,
           kAlignCenter);
  });
}

Model model = [] {
  Model m;
  m.slug = "macro-osc-2";
  m.name = "Macro Oscillator 2";
  m.shortName = "MACRO OSC 2";
  m.basedOn = "Plaits";
  m.category = Category::Oscillator;
  m.description = "24 synthesis models: VA, FM, wavetable, chords, speech, strings, drums.";
  m.create = [] { return new MacroOsc2(); };
  m.buildPanel = buildPanel;
  return m;
}();

BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
