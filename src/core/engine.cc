// BrickRack - GPL-3.0-or-later.

#include "core/engine.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <map>
#include <fcntl.h>
#include <unistd.h>

#include "core/model.h"

#if defined(__aarch64__)
#include <cstdint>
#endif

namespace br {

namespace {

// Flush denormals to zero on the audio thread (the original firmware runs on a
// Cortex-M4 with FTZ enabled; several filters rely on this for speed).
void enableFlushToZero() {
#if defined(__aarch64__)
  uint64_t fpcr;
  __asm__ __volatile__("mrs %0, fpcr" : "=r"(fpcr));
  fpcr |= (1ULL << 24);  // FZ
  __asm__ __volatile__("msr fpcr, %0" : : "r"(fpcr));
#elif defined(__SSE__) || defined(__x86_64__)
  unsigned int csr;
  __asm__ __volatile__("stmxcsr %0" : "=m"(csr));
  csr |= 0x8040;  // FTZ | DAZ
  __asm__ __volatile__("ldmxcsr %0" : : "m"(csr));
#endif
}

inline float softClip(float x) {
  // Smooth saturation that is transparent below ~0.8.
  if (x > 1.5f) return 1.f;
  if (x < -1.5f) return -1.f;
  return x - (4.f / 27.f) * x * x * x;
}

}  // namespace

Engine::Engine(float sampleRate) : sampleRate_(sampleRate) {
  peak_[0].store(0.f);
  peak_[1].store(0.f);
}

Engine::~Engine() { clear(); }

int Engine::addModule(Module* module, int position, int forcedId, bool initRate) {
  // The module isn't visible to the audio thread yet, so do the (possibly
  // allocating) rate setup before taking the lock.
  if (initRate) module->onSampleRateChange(sampleRate_);
  std::lock_guard<std::mutex> lock(mutex_);
  if (forcedId > 0 && forcedId < kMaxModuleId && !findModule(forcedId)) {
    module->id = forcedId;
    nextModuleId_ = std::max(nextModuleId_, forcedId + 1);
  } else {
    module->id = nextModuleId_++;
  }
  if (position < 0 || position > static_cast<int>(modules_.size())) {
    modules_.push_back(module);
  } else {
    modules_.insert(modules_.begin() + position, module);
  }
  rebuildRouting();
  return module->id;
}

void Engine::removeModule(int moduleId) {
  Module* victim = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = std::find_if(modules_.begin(), modules_.end(),
                           [&](Module* m) { return m->id == moduleId; });
    if (it == modules_.end()) return;
    victim = *it;
    modules_.erase(it);
    cables_.erase(std::remove_if(cables_.begin(), cables_.end(),
                                 [&](const Cable& c) {
                                   return c.inModule == moduleId || c.outModule == moduleId;
                                 }),
                  cables_.end());
    rebuildRouting();
  }
  delete victim;  // outside the lock: destructors may free large buffers
}

void Engine::moveModule(int moduleId, int newIndex) {
  std::lock_guard<std::mutex> lock(mutex_);
  int idx = -1;
  for (size_t i = 0; i < modules_.size(); ++i) {
    if (modules_[i]->id == moduleId) idx = static_cast<int>(i);
  }
  if (idx < 0) return;
  newIndex = std::max(0, std::min(newIndex, static_cast<int>(modules_.size()) - 1));
  Module* m = modules_[static_cast<size_t>(idx)];
  modules_.erase(modules_.begin() + idx);
  modules_.insert(modules_.begin() + newIndex, m);
  rebuildRouting();
}

void Engine::resetModule(int moduleId) {
  std::lock_guard<std::mutex> lock(mutex_);
  Module* m = findModule(moduleId);
  if (!m) return;
  m->resetParams();
  m->onReset();
}

void Engine::clear() {
  std::vector<Module*> victims;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    victims.swap(modules_);
    cables_.clear();
    routes_.clear();
    nextModuleId_ = 1;
    nextCableId_ = 1;
  }
  for (Module* m : victims) delete m;
}

int Engine::addCable(int outModule, int outPort, int inModule, int inPort, uint32_t color,
                     int forcedId) {
  std::lock_guard<std::mutex> lock(mutex_);
  Module* om = findModule(outModule);
  Module* im = findModule(inModule);
  if (!om || !im) return -1;
  if (outPort < 0 || outPort >= static_cast<int>(om->outputs.size())) return -1;
  if (inPort < 0 || inPort >= static_cast<int>(im->inputs.size())) return -1;
  cables_.erase(std::remove_if(cables_.begin(), cables_.end(),
                               [&](const Cable& c) {
                                 return c.inModule == inModule && c.inPort == inPort;
                               }),
                cables_.end());
  Cable c;
  bool idFree = std::none_of(cables_.begin(), cables_.end(),
                             [&](const Cable& x) { return x.id == forcedId; });
  if (forcedId > 0 && forcedId < kMaxModuleId && idFree) {
    c.id = forcedId;
    nextCableId_ = std::max(nextCableId_, forcedId + 1);
  } else {
    c.id = nextCableId_++;
  }
  c.outModule = outModule;
  c.outPort = outPort;
  c.inModule = inModule;
  c.inPort = inPort;
  c.color = color;
  cables_.push_back(c);
  rebuildRouting();
  return c.id;
}

void Engine::removeCable(int cableId) {
  std::lock_guard<std::mutex> lock(mutex_);
  cables_.erase(std::remove_if(cables_.begin(), cables_.end(),
                               [&](const Cable& c) { return c.id == cableId; }),
                cables_.end());
  rebuildRouting();
}

void Engine::removeCablesOnPort(int moduleId, bool isOutput, int port) {
  std::lock_guard<std::mutex> lock(mutex_);
  cables_.erase(std::remove_if(cables_.begin(), cables_.end(),
                               [&](const Cable& c) {
                                 return isOutput ? (c.outModule == moduleId && c.outPort == port)
                                                 : (c.inModule == moduleId && c.inPort == port);
                               }),
                cables_.end());
  rebuildRouting();
}

Module* Engine::findModule(int id) const {
  for (Module* m : modules_) {
    if (m->id == id) return m;
  }
  return nullptr;
}

int Engine::indexOf(int moduleId) const {
  for (size_t i = 0; i < modules_.size(); ++i) {
    if (modules_[i]->id == moduleId) return static_cast<int>(i);
  }
  return -1;
}

const Cable* Engine::cableOnInput(int moduleId, int port) const {
  for (const Cable& c : cables_) {
    if (c.inModule == moduleId && c.inPort == port) return &c;
  }
  return nullptr;
}

std::vector<const Cable*> Engine::cablesOnOutput(int moduleId, int port) const {
  std::vector<const Cable*> result;
  for (const Cable& c : cables_) {
    if (c.outModule == moduleId && c.outPort == port) result.push_back(&c);
  }
  return result;
}

void Engine::rebuildRouting() {
  for (Module* m : modules_) {
    for (Port& p : m->inputs) p.connected = false;
    for (Port& p : m->outputs) p.connected = false;
  }
  routes_.clear();
  for (const Cable& c : cables_) {
    Module* om = findModule(c.outModule);
    Module* im = findModule(c.inModule);
    if (!om || !im) continue;
    Port* from = &om->outputs[static_cast<size_t>(c.outPort)];
    Port* to = &im->inputs[static_cast<size_t>(c.inPort)];
    from->connected = true;
    to->connected = true;
    routes_.push_back({from, to});
  }
  // Unpatched inputs read 0 V.
  for (Module* m : modules_) {
    for (Port& p : m->inputs) {
      if (!p.connected) p.voltage = 0.f;
    }
  }
}

void Engine::process(float* out, const float* in, int inChannels, int frames) {
  static thread_local bool ftzSet = false;
  if (!ftzSet) {
    enableFlushToZero();
    ftzSet = true;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  auto t0 = std::chrono::steady_clock::now();  // after the lock: measure DSP only

  ProcessArgs args;
  args.sampleRate = sampleRate_;
  args.sampleTime = 1.f / sampleRate_;
  args.engine = this;

  inputAvailable_ = in != nullptr && inChannels > 0;
  const float vol = masterVolume.load(std::memory_order_relaxed);
  float pk0 = 0.f, pk1 = 0.f;
  const float dcCoef = 1.f - (2.f * 3.14159265f * 8.f / sampleRate_);

  for (int f = 0; f < frames; ++f) {
    if (inputAvailable_) {
      inL_ = in[f * inChannels];
      inR_ = inChannels > 1 ? in[f * inChannels + 1] : inL_;
    } else {
      inL_ = inR_ = 0.f;
    }
    // Cables carry what real patch cables can: anything beyond the ±12 V
    // Eurorack rails is clipped, and NaN/Inf from a misbehaving module becomes
    // 0 V instead of spreading through the rack.
    for (const Route& r : routes_) {
      float v = r.from->voltage;
      if (!(v >= -12.f && v <= 12.f)) v = v > 12.f ? 12.f : (v < -12.f ? -12.f : 0.f);
      r.to->voltage = v;
    }
    masterL_ = masterR_ = 0.f;
    args.frame = frame_;
    for (Module* m : modules_) m->process(args);
    ++frame_;

    // DC blocker + master volume + soft clip. Non-finite input is dropped
    // *before* it can poison the DC-blocker state (which would mute forever).
    float l = masterL_, r = masterR_;
    if (!std::isfinite(l) || !std::isfinite(r)) {
      nonFinite_.fetch_add(1, std::memory_order_relaxed);
      if (!std::isfinite(l)) l = 0.f;
      if (!std::isfinite(r)) r = 0.f;
    }
    dcL_ = dcCoef * dcL_ + (1.f - dcCoef) * l;
    dcR_ = dcCoef * dcR_ + (1.f - dcCoef) * r;
    l = softClip((l - dcL_) * vol);
    r = softClip((r - dcR_) * vol);
    out[2 * f] = l;
    out[2 * f + 1] = r;
    pk0 = std::max(pk0, std::fabs(l));
    pk1 = std::max(pk1, std::fabs(r));
  }

  peak_[0].store(std::max(pk0, peak_[0].load() * 0.85f));
  peak_[1].store(std::max(pk1, peak_[1].load() * 0.85f));

  auto t1 = std::chrono::steady_clock::now();
  float elapsed = std::chrono::duration<float>(t1 - t0).count();
  float budget = static_cast<float>(frames) / sampleRate_;
  float load = elapsed / budget;
  cpuLoad_.store(cpuLoad_.load() * 0.9f + load * 0.1f);
}

// ---------------------------------------------------------------------------
// Patch files: a small line-oriented text format, easy to diff and hand-edit.
//
//   brickrack-patch 1
//   volume 0.8
//   module <id> <slug>
//   param <index> <value>
//   state <key> <value to end of line>
//   cable <id> <outModule> <outPort> <inModule> <inPort> <color hex>

namespace {

std::string escape(const std::string& s) {
  std::string r;
  for (char c : s) {
    if (c == '\n') r += "\\n";
    else if (c == '\\') r += "\\\\";
    else r += c;
  }
  return r;
}

std::string unescape(const std::string& s) {
  std::string r;
  for (size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '\\' && i + 1 < s.size()) {
      ++i;
      r += s[i] == 'n' ? '\n' : s[i];
    } else {
      r += s[i];
    }
  }
  return r;
}

}  // namespace

bool Engine::savePatch(const std::string& path) const {
  std::string tmp = path + ".tmp";
  {
    std::ofstream f(tmp);
    if (!f) return false;
    f << std::setprecision(9);  // round-trips every float exactly
    f << "brickrack-patch 1\n";
    f << "volume " << masterVolume.load() << "\n";
    for (const Module* m : modules_) {
      f << "module " << m->id << " " << m->model->slug << "\n";
      for (size_t i = 0; i < m->params.size(); ++i) {
        f << "param " << i << " " << m->params[i].value << "\n";
      }
      StateMap state;
      m->saveState(state);
      for (const auto& kv : state) {
        f << "state " << kv.first << " " << escape(kv.second) << "\n";
      }
    }
    for (const Cable& c : cables_) {
      char col[16];
      std::snprintf(col, sizeof(col), "%08X", c.color);
      f << "cable " << c.id << " " << c.outModule << " " << c.outPort << " " << c.inModule
        << " " << c.inPort << " " << col << "\n";
    }
    f.flush();
    if (!f.good()) return false;
  }
  // Make sure the data is on the SD card before replacing the old file.
  int fd = ::open(tmp.c_str(), O_RDONLY);
  if (fd >= 0) {
    ::fsync(fd);
    ::close(fd);
  }
  return std::rename(tmp.c_str(), path.c_str()) == 0;
}

bool Engine::loadPatch(const std::string& path, std::string* error) {
  std::ifstream f(path);
  if (!f) {
    if (error) *error = "cannot open " + path;
    return false;
  }
  std::string line;
  std::getline(f, line);
  if (line.rfind("brickrack-patch", 0) != 0) {
    if (error) *error = "not a BrickRack patch";
    return false;
  }

  struct PendingModule {
    int id;
    Module* module;
    StateMap state;
  };
  std::vector<PendingModule> pending;
  std::vector<Cable> pendingCables;
  float volume = masterVolume.load();

  while (std::getline(f, line)) {
    std::istringstream ss(line);
    std::string kw;
    ss >> kw;
    if (kw == "volume") {
      float v;
      if (ss >> v && std::isfinite(v)) volume = std::min(std::max(v, 0.f), 1.5f);
    } else if (kw == "module") {
      int id = 0;
      std::string slug;
      ss >> id >> slug;
      if (!ss) id = 0;
      const Model* model = findModel(slug);
      if (!model) {
        std::fprintf(stderr, "patch: unknown module '%s' skipped\n", slug.c_str());
        pending.push_back({id, nullptr, {}});
        continue;
      }
      Module* m = model->create();
      m->model = model;
      pending.push_back({id, m, {}});
    } else if (kw == "param") {
      if (pending.empty() || !pending.back().module) continue;
      size_t idx;
      float v;
      if (!(ss >> idx >> v) || !std::isfinite(v)) continue;
      Module* m = pending.back().module;
      if (idx < m->params.size()) m->params[idx].set(v);
    } else if (kw == "state") {
      if (pending.empty() || !pending.back().module) continue;
      std::string key, rest;
      ss >> key;
      std::getline(ss, rest);
      if (!rest.empty() && rest[0] == ' ') rest.erase(0, 1);
      pending.back().state[key] = unescape(rest);
    } else if (kw == "cable") {
      Cable c;
      std::string col;
      if (!(ss >> c.id >> c.outModule >> c.outPort >> c.inModule >> c.inPort)) continue;
      ss >> col;
      c.color = static_cast<uint32_t>(std::strtoul(col.c_str(), nullptr, 16));
      if ((c.color >> 24) == 0) c.color |= 0xFF000000;  // RRGGBB or missing alpha
      pendingCables.push_back(c);
    }
  }

  clear();
  masterVolume.store(volume);
  // Module ids from the file may be invalid or duplicated: addModule assigns a
  // fresh id in that case. Cables follow the first module that claimed an id.
  std::map<int, int> idMap;
  for (PendingModule& p : pending) {
    if (!p.module) continue;
    // Rate setup first, so state loaded afterwards is not reset by it.
    p.module->onSampleRateChange(sampleRate_);
    p.module->loadState(p.state);
    int newId = addModule(p.module, -1, p.id, false);
    if (idMap.find(p.id) == idMap.end()) idMap[p.id] = newId;
  }
  for (const Cable& c : pendingCables) {
    auto o = idMap.find(c.outModule), i = idMap.find(c.inModule);
    if (o == idMap.end() || i == idMap.end()) continue;
    addCable(o->second, c.outPort, i->second, c.inPort, c.color,
             c.id > 0 && c.id < kMaxModuleId ? c.id : -1);
  }
  return true;
}

}  // namespace br
