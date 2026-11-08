// BrickRack - GPL-3.0-or-later.
//
// The engine owns the rack (an ordered list of modules) and the cables.
// Threading model:
//   * The UI thread is the only thread that mutates structure (modules,
//     cables). It does so while holding `mutex()`.
//   * The audio thread holds `mutex()` while rendering a buffer.
//   * Because only the UI thread mutates, the UI thread may *read* the
//     structure without locking.
//   * Param values are plain floats written by the UI and read by the audio
//     thread (same approach as VCV Rack; aligned 32-bit stores are atomic on
//     every target we build for).

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "core/module.h"

namespace br {

struct Cable {
  int id = -1;
  int outModule = -1;
  int outPort = -1;
  int inModule = -1;
  int inPort = -1;
  uint32_t color = 0xFFE0B030;
};

class Engine {
 public:
  explicit Engine(float sampleRate = 48000.f);
  ~Engine();

  float sampleRate() const { return sampleRate_; }

  // ---- Structure (UI thread) ----
  // Takes ownership. position = index in the rack, -1 appends. forcedId is
  // honoured when valid and unused (patch loading); otherwise a fresh id is
  // assigned. initRate=false skips onSampleRateChange (caller already did it).
  int addModule(Module* module, int position = -1, int forcedId = -1, bool initRate = true);
  static constexpr int kMaxModuleId = 1 << 20;
  void removeModule(int moduleId);
  void moveModule(int moduleId, int newIndex);
  void resetModule(int moduleId);
  void clear();

  // Connects an output to an input. Any cable already on that input is
  // replaced. Returns the cable id, or -1 on failure.
  int addCable(int outModule, int outPort, int inModule, int inPort, uint32_t color,
               int forcedId = -1);
  void removeCable(int cableId);
  void removeCablesOnPort(int moduleId, bool isOutput, int port);

  const std::vector<Module*>& modules() const { return modules_; }
  const std::vector<Cable>& cables() const { return cables_; }
  Module* findModule(int id) const;
  int indexOf(int moduleId) const;
  const Cable* cableOnInput(int moduleId, int port) const;
  std::vector<const Cable*> cablesOnOutput(int moduleId, int port) const;

  std::mutex& mutex() { return mutex_; }

  // ---- Audio (audio thread) ----
  // Renders `frames` interleaved stereo float frames. `in` (may be null) holds
  // `inChannels` interleaved channels of device input (microphone).
  void process(float* out, const float* in, int inChannels, int frames);

  // Called by I/O modules during process().
  void addToMaster(float left, float right) {
    masterL_ += left;
    masterR_ += right;
  }
  float deviceInput(int channel) const { return channel == 0 ? inL_ : inR_; }
  bool deviceInputAvailable() const { return inputAvailable_; }

  // Controller state published by the UI thread, read by the controller module.
  struct ControllerState {
    std::atomic<float> axis[4];     // LX, LY, RX, RY in [-1, 1]
    std::atomic<bool> gate[4];      // L2, R2, L3, R3
    ControllerState() {
      for (auto& a : axis) a.store(0.f);
      for (auto& g : gate) g.store(false);
    }
  };
  ControllerState& controller() { return controller_; }

  std::atomic<float> masterVolume{0.8f};
  float cpuLoad() const { return cpuLoad_.load(); }
  float peakLevel(int ch) const { return peak_[ch].load(); }
  int64_t frameCount() const { return frame_; }
  // Number of master-bus frames that contained NaN/Inf (a module bug).
  int64_t nonFiniteFrames() const { return nonFinite_.load(); }

  // ---- Patch persistence (UI thread) ----
  bool savePatch(const std::string& path) const;
  bool loadPatch(const std::string& path, std::string* error = nullptr);

  int nextCableColorIndex() { return cableColorIndex_++; }

 private:
  void rebuildRouting();  // must hold mutex

  float sampleRate_;
  std::vector<Module*> modules_;
  std::vector<Cable> cables_;
  int nextModuleId_ = 1;
  int nextCableId_ = 1;
  int cableColorIndex_ = 0;

  struct Route {
    const Port* from;
    Port* to;
  };
  std::vector<Route> routes_;

  std::mutex mutex_;
  int64_t frame_ = 0;
  float masterL_ = 0.f, masterR_ = 0.f;
  float inL_ = 0.f, inR_ = 0.f;
  bool inputAvailable_ = false;
  std::atomic<float> cpuLoad_{0.f};
  std::atomic<int64_t> nonFinite_{0};
  std::atomic<float> peak_[2];
  float dcL_ = 0.f, dcR_ = 0.f;
  ControllerState controller_;
};

}  // namespace br
