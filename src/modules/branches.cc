// BrickRack - GPL-3.0-or-later.
//
// Dual Bernoulli Gate, after Mutable Instruments Branches.
//
// Branches is an 8-bit AVR (ATmega88) firmware written as a single
// branches/branches.cc whose main loop reads GPIO pins, the ADC and Timer1
// directly; it cannot be compiled on desktop. Its main loop is reproduced here
// line by line, once per engine sample (the AVR loop runs at a comparable rate):
//   * the same 256-entry probability table (linear_table, copied verbatim
//     because it is a local of branches.cc and cannot be #included alone),
//   * the same 32-bit Galois LFSR (stepped once per loop) and the same
//     comparison `random >= threshold && threshold != 65535`,
//   * the same toggle mode (outcome XOR previous outcome), latch mode, button
//     timing (short press >= 64 Timer1 ticks toggles "toggle mode", a press of
//     6250 ticks = 0.8 s toggles "latch mode"; Timer1 = 8 MHz / 1024),
//   * the same LED logic (green = A, red = B, held 0x1ff loop iterations after
//     the gate falls).
// The toggle/latch settings, kept in EEPROM on the hardware, are hidden params
// here (editable from the module menu, saved with the patch).
//
// Voltages: outputs are 0/+5 V like the hardware. The p CV adds to the knob,
// +/-5 V sweeping the full range (same scaling as the VCV Rack port). Knob
// fully CCW sends every trigger to A, fully CW to B; as on the hardware the
// knob/CV voltage reaches the ADC inverted.

#include <algorithm>
#include <cstdint>

#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"
#include "ui/canvas.h"

namespace br {
namespace {

// branches/branches.cc: linear_table.
const uint16_t kLinearTable[256] = {
      0,     0,   259,   518,   777,  1036,  1295,  1554,
   1813,  2072,  2331,  2590,  2849,  3108,  3367,  3626,
   3885,  4145,  4404,  4663,  4922,  5181,  5440,  5699,
   5958,  6217,  6476,  6735,  6994,  7253,  7512,  7771,
   8030,  8289,  8548,  8807,  9066,  9325,  9584,  9843,
  10102, 10361, 10620, 10879, 11138, 11397, 11656, 11915,
  12174, 12434, 12693, 12952, 13211, 13470, 13729, 13988,
  14247, 14506, 14765, 15024, 15283, 15542, 15801, 16060,
  16319, 16578, 16837, 17096, 17355, 17614, 17873, 18132,
  18391, 18650, 18909, 19168, 19427, 19686, 19945, 20204,
  20463, 20723, 20982, 21241, 21500, 21759, 22018, 22277,
  22536, 22795, 23054, 23313, 23572, 23831, 24090, 24349,
  24608, 24867, 25126, 25385, 25644, 25903, 26162, 26421,
  26680, 26939, 27198, 27457, 27716, 27975, 28234, 28493,
  28753, 29012, 29271, 29530, 29789, 30048, 30307, 30566,
  30825, 31084, 31343, 31602, 31861, 32120, 32379, 32638,
  32897, 33156, 33415, 33674, 33933, 34192, 34451, 34710,
  34969, 35228, 35487, 35746, 36005, 36264, 36523, 36782,
  37042, 37301, 37560, 37819, 38078, 38337, 38596, 38855,
  39114, 39373, 39632, 39891, 40150, 40409, 40668, 40927,
  41186, 41445, 41704, 41963, 42222, 42481, 42740, 42999,
  43258, 43517, 43776, 44035, 44294, 44553, 44812, 45072,
  45331, 45590, 45849, 46108, 46367, 46626, 46885, 47144,
  47403, 47662, 47921, 48180, 48439, 48698, 48957, 49216,
  49475, 49734, 49993, 50252, 50511, 50770, 51029, 51288,
  51547, 51806, 52065, 52324, 52583, 52842, 53101, 53361,
  53620, 53879, 54138, 54397, 54656, 54915, 55174, 55433,
  55692, 55951, 56210, 56469, 56728, 56987, 57246, 57505,
  57764, 58023, 58282, 58541, 58800, 59059, 59318, 59577,
  59836, 60095, 60354, 60613, 60872, 61131, 61390, 61650,
  61909, 62168, 62427, 62686, 62945, 63204, 63463, 63722,
  63981, 64240, 64499, 64758, 65017, 65276, 65535, 65535,
};

const uint16_t kLongPressTime = 6250;  // 800 * 8000 / 1024
const uint16_t kLedGateDelay = 0x1ff;
const double kTimer1Rate = 8000000.0 / 1024.0;

enum LedState { LED_STATE_OFF, LED_STATE_RED, LED_STATE_GREEN };

class BernoulliGate : public Module {
 public:
  enum ParamIds {
    P1_PARAM,
    P2_PARAM,
    MODE1_PARAM,
    MODE2_PARAM,
    TOGGLE1_PARAM,  // hidden: EEPROM settings
    TOGGLE2_PARAM,
    LATCH1_PARAM,
    LATCH2_PARAM,
    NUM_PARAMS
  };
  enum InputIds { IN1_INPUT, P1_INPUT, IN2_INPUT, P2_INPUT, NUM_INPUTS };
  enum OutputIds { OUT1A_OUTPUT, OUT1B_OUTPUT, OUT2A_OUTPUT, OUT2B_OUTPUT, NUM_OUTPUTS };
  enum LightIds { LED1_GREEN, LED1_RED, LED2_GREEN, LED2_RED, NUM_LIGHTS };

  BernoulliGate() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(P1_PARAM, 0.f, 1.f, 0.5f, "Probability 1 (A..B)", "%", 100.f);
    configParam(P2_PARAM, 0.f, 1.f, 0.5f, "Probability 2 (A..B)", "%", 100.f);
    configButton(MODE1_PARAM, "Mode 1 (press: toggle, hold 0.8 s: latch)");
    configButton(MODE2_PARAM, "Mode 2 (press: toggle, hold 0.8 s: latch)");
    configSwitch(TOGGLE1_PARAM, 0.f, 1.f, 0.f, "Channel 1 toggle mode", {"Off", "On"}).hidden = true;
    configSwitch(TOGGLE2_PARAM, 0.f, 1.f, 0.f, "Channel 2 toggle mode", {"Off", "On"}).hidden = true;
    configSwitch(LATCH1_PARAM, 0.f, 1.f, 0.f, "Channel 1 latch mode", {"Off", "On"}).hidden = true;
    configSwitch(LATCH2_PARAM, 0.f, 1.f, 0.f, "Channel 2 latch mode", {"Off", "On"}).hidden = true;
    configInput(IN1_INPUT, "Gate 1");
    configInput(P1_INPUT, "Probability 1 CV");
    configInput(IN2_INPUT, "Gate 2 (normalled to gate 1)");
    configInput(P2_INPUT, "Probability 2 CV");
    configOutput(OUT1A_OUTPUT, "1 A");
    configOutput(OUT1B_OUTPUT, "1 B");
    configOutput(OUT2A_OUTPUT, "2 A");
    configOutput(OUT2B_OUTPUT, "2 B");
    // The firmware seeds with 1; give each instance its own sequence so two
    // modules fed the same triggers don't make identical decisions.
    static uint32_t seed = 0x2545F491u;
    seed = seed * 1664525u + 1013904223u;
    rng_state_ = seed | 1u;
  }

  void onReset() override {
    for (int i = 0; i < 2; ++i) {
      input_state_[i] = previous_state_[i] = false;
      gate_[i] = 0;
      led_state_[i] = 0;
      led_gate_delay_[i] = 0;
    }
  }

  void process(const ProcessArgs& args) override {
    timer1_ += kTimer1Rate * args.sampleTime;
    if (timer1_ >= 65536.0) timer1_ -= 65536.0;
    const uint16_t tcnt1 = static_cast<uint16_t>(timer1_);

    for (int i = 0; i < 2; ++i) {
      // ADC: knob + CV, inverted at the ADC input (see header).
      float v = params[static_cast<size_t>(P1_PARAM + i)].value +
                inputs[static_cast<size_t>(P1_INPUT + 2 * i)].getVoltage() / 10.f;
      uint8_t adc = static_cast<uint8_t>(255 - static_cast<int>(clampf(v, 0.f, 1.f) * 255.f + 0.5f));
      p_[i] = kLinearTable[adc];
    }

    // Scan switches.
    for (int i = 0; i < 2; ++i) {
      bool new_input_state = params[static_cast<size_t>(MODE1_PARAM + i)].value > 0.5f;
      if (!switch_state_[i] && new_input_state) {
        press_time_[i] = tcnt1;
        inhibit_switch_[i] = false;
      }
      if (switch_state_[i] && !inhibit_switch_[i]) {
        uint16_t this_press_time = static_cast<uint16_t>(tcnt1 - press_time_[i]);
        if (this_press_time >= kLongPressTime) {
          inhibit_switch_[i] = true;
          flip(LATCH1_PARAM + i);
        } else if (this_press_time >= 64 && !new_input_state) {
          flip(TOGGLE1_PARAM + i);
        }
      }
      switch_state_[i] = new_input_state;
    }

    // Scan inputs. Input 2 is normalled to input 1.
    trigger_[0].process(inputs[IN1_INPUT].getVoltage());
    trigger_[1].process(inputs[IN2_INPUT].getNormalVoltage(inputs[IN1_INPUT].getVoltage()));
    uint32_t random_words = rng_state_;
    for (int i = 0; i < 2; ++i) {
      bool new_input_state = trigger_[i].isHigh();
      bool toggle_mode = params[static_cast<size_t>(TOGGLE1_PARAM + i)].value > 0.5f;
      bool latch_mode = params[static_cast<size_t>(LATCH1_PARAM + i)].value > 0.5f;

      if (new_input_state || latch_mode) led_gate_delay_[i] = kLedGateDelay;

      if (new_input_state && !input_state_[i]) {  // Rising edge
        uint16_t random = static_cast<uint16_t>(random_words & 0xffff);
        uint16_t threshold = p_[i];
        bool outcome = random >= threshold && threshold != 65535;
        if (toggle_mode) outcome = outcome ^ previous_state_[i];
        previous_state_[i] = outcome;
        gate_[i] = outcome ? 2 : 1;  // 1 = A high, 2 = B high
        led_state_[i] = outcome ? LED_STATE_RED : LED_STATE_GREEN;
      } else if (!new_input_state && input_state_[i] && !latch_mode) {
        gate_[i] = 0;
      }
      input_state_[i] = new_input_state;
      random_words >>= 16;

      if (led_gate_delay_[i]) {
        --led_gate_delay_[i];
        if (!led_gate_delay_[i]) led_state_[i] = LED_STATE_OFF;
      }
      outputs[static_cast<size_t>(OUT1A_OUTPUT + 2 * i)].setVoltage(gate_[i] == 1 ? 5.f : 0.f);
      outputs[static_cast<size_t>(OUT1B_OUTPUT + 2 * i)].setVoltage(gate_[i] == 2 ? 5.f : 0.f);
      lights[static_cast<size_t>(LED1_GREEN + 2 * i)] = led_state_[i] == LED_STATE_GREEN ? 1.f : 0.f;
      lights[static_cast<size_t>(LED1_RED + 2 * i)] = led_state_[i] == LED_STATE_RED ? 1.f : 0.f;
    }

    rng_state_ = (rng_state_ >> 1) ^ (-(rng_state_ & 1u) & 0xD0000001u);
  }

  bool toggleMode(int i) const { return params[static_cast<size_t>(TOGGLE1_PARAM + i)].value > 0.5f; }
  bool latchMode(int i) const { return params[static_cast<size_t>(LATCH1_PARAM + i)].value > 0.5f; }

 private:
  void flip(int param) {
    Param& p = params[static_cast<size_t>(param)];
    p.set(p.value > 0.5f ? 0.f : 1.f);
  }

  uint16_t p_[2] = {0, 0};
  bool input_state_[2] = {false, false};
  bool previous_state_[2] = {false, false};
  bool switch_state_[2] = {false, false};
  bool inhibit_switch_[2] = {false, false};
  uint16_t press_time_[2] = {0, 0};
  uint8_t led_state_[2] = {0, 0};
  uint16_t led_gate_delay_[2] = {0, 0};
  uint8_t gate_[2] = {0, 0};
  uint32_t rng_state_ = 1;
  double timer1_ = 0.0;
  SchmittTrigger trigger_[2];
};

void buildPanel(PanelSpec& p) {
  p.hp = 6;
  p.bg = 0xFFF2F1EC;
  p.ink = 0xFF1E1E20;
  p.title = "BERNOULLI GATE";
  using M = BernoulliGate;
  const uint32_t badge[2] = {0xFFC8414B, 0xFF168A8C};
  const char* nums[2] = {"1", "2"};

  // Positions measured on the original panel (0.1902 mm/px); channel 2 is
  // 52.7 mm below channel 1.
  for (int i = 0; i < 2; ++i) {
    const float dy = 52.7f * static_cast<float>(i);
    p.display(1.6f, 20.0f + dy, 4.2f, 4.2f, [i, badge, nums](Canvas& c, float x, float y, float w,
                                                              float h, float s, const Module*) {
      c.fillCircle(x + w * 0.5f, y + h * 0.5f, w * 0.5f, badge[i]);
      c.text(x + w * 0.5f, y + h * 0.5f, nums[i], std::max(12.f, 2.9f * s), 0xFFFFFFFF,
             kAlignCenter);
    });
    p.button(M::MODE1_PARAM + i, 25.7f, 22.1f + dy, 5.0f);
    p.knob(M::P1_PARAM + i, 15.0f, 28.3f + dy, 9.5f);
    // Mode readout (the hardware has none; handy since the modes are hidden).
    p.display(20.6f, 26.2f + dy, 9.4f, 5.6f, [i](Canvas& c, float x, float y, float w, float h,
                                                 float s, const Module* m) {
      auto* b = static_cast<const BernoulliGate*>(m);
      float fs = std::max(13.f, 2.2f * s);
      c.text(x + w * 0.5f, y + h * 0.27f, "TOGGLE", fs,
             b->toggleMode(i) ? 0xFF168A8C : 0x40000000, kAlignCenter);
      c.text(x + w * 0.5f, y + h * 0.75f, "LATCH", fs,
             b->latchMode(i) ? 0xFF168A8C : 0x40000000, kAlignCenter);
    });
    // Dotted guide from the knob to its CV jack.
    for (float y = 34.0f + dy; y < 44.6f + dy; y += 1.5f)
      p.line(15.0f, y, 15.0f, y + 0.7f, 0.3f, p.ink);
    for (float x = 15.0f; x < 19.2f; x += 1.5f)
      p.line(x, 45.5f + dy, x + 0.7f, 45.5f + dy, 0.3f, p.ink);

    p.label(6.9f, 39.4f + dy, "IN", 2.9f);
    p.label(22.8f, 39.4f + dy, "p", 3.4f);
    p.input(M::IN1_INPUT + 2 * i, 6.9f, 45.5f + dy);
    p.input(M::P1_INPUT + 2 * i, 22.8f, 45.5f + dy);

    p.box(1.4f, 50.4f + dy, 27.7f, 12.6f, 0xFF5E5E60, 1.2f);
    p.label(6.9f, 52.6f + dy, "OUT A", 2.6f, kAlignCenter, 0xFFFFFFFF);
    p.label(22.8f, 52.6f + dy, "OUT B", 2.6f, kAlignCenter, 0xFFFFFFFF);
    p.output(M::OUT1A_OUTPUT + 2 * i, 6.9f, 58.4f + dy);
    p.output(M::OUT1B_OUTPUT + 2 * i, 22.8f, 58.4f + dy);
    p.bicolorLight(M::LED1_GREEN + 2 * i, M::LED1_RED + 2 * i, 14.85f, 58.4f + dy, 0xFF35E06A,
                   0xFFFF3B30, 2.6f);
  }
  p.line(1.4f, 68.9f, 29.1f, 68.9f, 0.6f, 0xFFB8B8B8);
  p.line(1.4f, 121.6f, 29.1f, 121.6f, 0.6f, 0xFFB8B8B8);
}

Model model = [] {
  Model m;
  m.slug = "bernoulli-gate";
  m.name = "Dual Bernoulli Gate";
  m.shortName = "BERNOULLI GATE";
  m.basedOn = "Branches";
  m.category = Category::Utility;
  m.description = "Two coin tosses: each trigger goes to output A or B with probability p.";
  m.create = [] { return new BernoulliGate(); };
  m.buildPanel = buildPanel;
  return m;
}();

BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
