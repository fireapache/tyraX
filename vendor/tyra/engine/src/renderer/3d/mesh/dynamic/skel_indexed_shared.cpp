// The per-model-part-level half of the indexed skinned stream (skel_indexed_shared.hpp).

#include "renderer/3d/mesh/dynamic/skel_indexed_shared.hpp"

#include "renderer/3d/mesh/dynamic/skel_room.hpp"

#include <algorithm>
#include <cstring>
#include <new>

#if defined(_EE)
#include "renderer/core/paths/path1/vif1_queue.hpp"
#endif
#if TYRA_MEMORY_MANAGER
#include "memory/manager.hpp"
#endif

namespace Tyra {
namespace SkelSkin {
namespace Indexed {

namespace {

u8* alignedNew(u32 bytes, u8** raw) {
  *raw = new (std::nothrow) u8[(size_t)bytes + 15];
  if (*raw == nullptr) return nullptr;
  return reinterpret_cast<u8*>((reinterpret_cast<uintptr_t>(*raw) + 15) &
                               ~static_cast<uintptr_t>(15));
}

u32 gMade = 0, gBytes = 0, gInUse = 0, gPeak = 0;

bool registerSpan(const u8* data, u32 qwords) {
#if defined(_EE)
  return Vif1Queue::registerImmutableSpan(data, qwords);
#else
  (void)data;
  (void)qwords;
  return false;
#endif
}

}  // namespace

void poolTotals(u32* made, u32* bytes, u32* inUse, u32* peakInUse) {
  *made = gMade;
  *bytes = gBytes;
  *inUse = gInUse;
  *peakInUse = gPeak;
}

Shared::Shared() {}

Shared::~Shared() { clear(); }

void Shared::clear() {
#if defined(_EE)
  // unregister waits for every chain that names the memory before it is freed
  if (staticRegistered && staticData != nullptr) Vif1Queue::unregisterImmutableSpan(staticData);
  for (u32 i = 0; i < all.size(); ++i)
    if (all[i]->registered) Vif1Queue::unregisterImmutableSpan(all[i]->data);
#endif
  for (u32 i = 0; i < all.size(); ++i) {
    PoseBuf* b = all[i];
    gMade -= 1;
    gBytes -= b->qwords * 16;
    delete[] b->raw;
    delete b;
  }
  gInUse -= inUse < gInUse ? inUse : gInUse;
  all.clear();
  retiring.clear();
  std::vector<u32>().swap(rngOff);
  std::vector<u16>().swap(rng);
  std::vector<u32>().swap(poseLayout.block);
  std::vector<u32>().swap(stat.block);
  lvl = Level();
  delete[] staticRaw;
  staticRaw = staticData = nullptr;
  staticRegistered = false;
  isBuilt = false;
  memoryRefused = false;
  made = inUse = peakInUse = 0;
}

bool Shared::build(const u32* skinSource, u32 corners, u32 packageSize, const s16* stq,
                   const StaPipCompactLayout& layout) {
  clear();
#if TYRA_MEMORY_MANAGER
  Memory::MemScope memoryScope(Memory::Budget::Skinning);
#endif
  bool noRoom = false;
  if (!Indexed::build(skinSource, corners, packageSize, lvl, &noRoom)) {
    lvl = Level();
    memoryRefused = noRoom;
    return false;
  }
  // The tables below are std::vectors: each stage asks for its exact size first (a refusal is
  // an answer, a throwing request over a full Skinning budget is a halt).
  const u32 uniques = lvl.uniques;
  if (!room(2 * vectorBytes(lvl.packages + 1, sizeof(u32)) + vectorBytes(uniques + 1, sizeof(u32)) +
            vectorBytes(uniques, sizeof(u32)))) {
    clear();
    memoryRefused = true;
    return false;
  }
  layoutPose(lvl, poseLayout);
  layoutStatic(lvl, stat);
  // The stream parts each level-unique falls in (packageSize / 3 corners a part), as the
  // plain scatter's range lists: corners ascend, so a unique's parts ascend too.
  parts = (corners + packageSize / 3 - 1) / (packageSize / 3);
  {
    const u32 partSize = packageSize / 3;
    std::vector<u32> last(lvl.uniques, 0xFFFFFFFFu);
    rngOff.assign(lvl.uniques + 1, 0);
    for (u32 v = 0; v < corners; ++v) {
      const u32 u = lvl.uid[v], r = v / partSize;
      if (last[u] != r) {
        last[u] = r;
        ++rngOff[u + 1];
      }
    }
    for (u32 u = 0; u < lvl.uniques; ++u) rngOff[u + 1] += rngOff[u];
    if (!room(vectorBytes(rngOff[lvl.uniques], sizeof(u16)) +
              vectorBytes(lvl.uniques + 1, sizeof(u32)))) {
      std::vector<u32>().swap(last);
      clear();
      memoryRefused = true;
      return false;
    }
    rng.assign(rngOff[lvl.uniques], 0);
    std::vector<u32> fill(rngOff.begin(), rngOff.end() - 1);
    std::fill(last.begin(), last.end(), 0xFFFFFFFFu);
    for (u32 v = 0; v < corners; ++v) {
      const u32 u = lvl.uid[v], r = v / partSize;
      if (last[u] != r) {
        last[u] = r;
        rng[fill[u]++] = (u16)r;
      }
    }
  }
  staticData = alignedNew(stat.bytes(), &staticRaw);
  if (staticData == nullptr) {
    clear();
    memoryRefused = true;
    return false;
  }
  buildStatic(lvl, stat, stq, nullptr, nullptr, staticData);
  // The bind-time tables are dead now: the pose gathers by `src`, the corner table carries the
  // local indices.
  std::vector<u32>().swap(lvl.uid);
  std::vector<u16>().swap(lvl.local);
  lay = layout;
  indexedView = StaPipIndexedLayout();
  indexedView.packageSize = packageSize;
  indexedView.packages = lvl.packages;
  indexedView.uniqueFirst = lvl.first.data();
  indexedView.poseBlock = poseLayout.block.data();
  indexedView.staticBlock = stat.block.data();
  indexedView.staticData = staticData;
  indexedView.poseQw = poseLayout.bytes() / 16;
  indexedView.staticQw = stat.bytes() / 16;
  staticRegistered = registerSpan(staticData, indexedView.staticQw);
  isBuilt = true;
  return true;
}

PoseBuf* Shared::acquire() {
  if (!isBuilt) return nullptr;
  PoseBuf* b = nullptr;
#if defined(_EE)
  // None free: the oldest retired buffer no recorded chain reads any more is taken - the scan
  // stops at the first, retirement being in age order. A pose buffer stays a registered span for
  // its whole life: nothing names a retired one in a new draw, so the span needs neither the
  // retire flag nor an unregister / register round trip per pose (each shifts the span table).
  for (u32 i = 0; b == nullptr && i < retiring.size(); ++i) {
    PoseBuf* r = retiring[i];
    if (!r->registered || Vif1Queue::immutableSpanReady(r->data)) {
      retiring.eraseAt(i);
      b = r;
    }
  }
#else
  if (b == nullptr && !retiring.empty()) {
    b = retiring[0];
    retiring.eraseAt(0);
  }
#endif
  if (b == nullptr) {
#if TYRA_MEMORY_MANAGER
    Memory::MemScope memoryScope(Memory::Budget::Skinning);
#endif
    b = new (std::nothrow) PoseBuf();
    if (b == nullptr) return nullptr;
    b->data = alignedNew(poseLayout.bytes(), &b->raw);
    // Room to list the buffer, and to retire it later (retire cannot fail): both lists can hold
    // every buffer made.
    if (b->data == nullptr || !all.reserve(made + 1) || !retiring.reserve(made + 1)) {
      delete[] b->raw;
      delete b;
      return nullptr;
    }
    initPose(lvl, poseLayout, b->data);
    b->qwords = poseLayout.bytes() / 16;
    all.push(b);
    ++made;
    ++gMade;
    gBytes += b->qwords * 16;
    // Registered once, when made: the chains of the frames to come name it in place.
    b->registered = registerSpan(b->data, b->qwords);
  }
  ++inUse;
  if (inUse > peakInUse) peakInUse = inUse;
  if (++gInUse > gPeak) gPeak = gInUse;
  return b;
}

void Shared::initBuffer(u8* data) const {
  if (isBuilt && data != nullptr) initPose(lvl, poseLayout, data);
}

void Shared::retire(PoseBuf* b) {
  if (b == nullptr) return;
  retiring.push(b);
  if (inUse > 0) --inUse;
  if (gInUse > 0) --gInUse;
}

}  // namespace Indexed
}  // namespace SkelSkin
}  // namespace Tyra
