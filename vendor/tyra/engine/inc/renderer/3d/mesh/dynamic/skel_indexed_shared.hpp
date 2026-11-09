#pragma once

// The per-model-part-level half of the indexed skinned stream (skel_indexed.hpp,
// docs/animated-models.md "Skinned streams with indexed corners"): the packaging, the static
// blocks every body REFs, the gather list, and the pool of pose buffers the bodies write.
//
// One Shared is built for each part level the first time a body of the model needs it
// (SkelBindLod::idx) and lives as long as the model's bind data. A body holds one pose buffer
// per part level at a time; a pose takes a free buffer from the pool, writes it, and retires
// the buffer it replaces. A retired buffer is reused once no DMA chain can still read it
// (the immutable-span retire rule the plain streams follow), so the pool settles at the
// bodies plus the poses a few frames in flight need.

#include <tamtypes.h>

#include <new>
#include <vector>

#include "renderer/3d/mesh/dynamic/skel_indexed.hpp"

namespace Tyra {
namespace SkelSkin {
namespace Indexed {

/** One pose buffer: the unique vertices of every package of a part level for one pose. */
struct PoseBuf {
  u8* raw = nullptr;
  u8* data = nullptr;  // 16-byte aligned, `qwords` long
  u32 qwords = 0;
  bool registered = false;  // an immutable span: DMA chains name it in place
};

/** A growable list of pose buffers that grows nothrow: `reserve` answers false when the budget
 * has no room, and `push` / `eraseAt` never allocate. (A std::vector's growth is a throwing
 * request, which a full Skinning budget answers by stopping the game.) */
class PoseList {
 public:
  PoseList() = default;
  ~PoseList() { delete[] items; }
  PoseList(const PoseList&) = delete;
  PoseList& operator=(const PoseList&) = delete;

  /** Room for `n` entries; false, with the list unchanged, when it cannot be had. */
  bool reserve(u32 n) {
    if (n <= cap) return true;
    PoseBuf** grown = new (std::nothrow) PoseBuf*[n];
    if (grown == nullptr) return false;
    for (u32 i = 0; i < count; ++i) grown[i] = items[i];
    delete[] items;
    items = grown;
    cap = n;
    return true;
  }
  u32 size() const { return count; }
  bool empty() const { return count == 0; }
  PoseBuf* operator[](u32 i) const { return items[i]; }
  /** Appends; the caller reserved room (size() < capacity). */
  void push(PoseBuf* b) { items[count++] = b; }
  /** Removes entry `i`, keeping the order of the rest. */
  void eraseAt(u32 i) {
    for (u32 k = i + 1; k < count; ++k) items[k - 1] = items[k];
    --count;
  }
  void clear() { count = 0; }

 private:
  PoseBuf** items = nullptr;
  u32 count = 0, cap = 0;
};

/** Pose buffers over every Shared: made, their bytes, away from their pool now, and the most
 * that were away at once (the pool's measured size, for the log). */
void poolTotals(u32* made, u32* bytes, u32* inUse, u32* peakInUse);

class Shared {
 public:
  Shared();
  ~Shared();
  Shared(const Shared&) = delete;
  Shared& operator=(const Shared&) = delete;

  /** Packages a part level and builds its static blocks: `skinSource` the corners' first
   * identical corner, `stq` [corners][3] the level's ST payload as the plain stream stores it
   * (Compact::packRun of the St attribute, per package), `layout` the stream layout the
   * blocks are for ({Position, St, Normal} at `packageSize`). False, with nothing kept, when
   * the package size does not fit or memory is refused (refused() tells which): every table is
   * asked for at its exact size first, so a full budget refuses and never halts. */
  bool build(const u32* skinSource, u32 corners, u32 packageSize, const s16* stq,
             const StaPipCompactLayout& layout);
  bool built() const { return isBuilt; }
  /** True when build() failed for want of memory (a later attempt may succeed), false when
   * the part cannot be packaged at all. */
  bool refused() const { return memoryRefused; }

  const StaPipCompactLayout& streamLayout() const { return lay; }
  const StaPipIndexedLayout& view() const { return indexedView; }
  const Level& level() const { return lvl; }
  const PoseLayout& pose() const { return poseLayout; }
  /** Per level-unique, the stream parts (packageSize / 3 corners) its corners fall in: the
   * unique `u` updates the boxes of parts rng[rngOff[u] .. rngOff[u + 1]). */
  const u32* rangeOffsets() const { return rngOff.data(); }
  const u16* ranges() const { return rng.data(); }
  u32 partCount() const { return parts; }

  /** A pose buffer to write, its run headers set: a collectable one, else a new one (nothrow, in
   * the Skinning budget, with the room to retire it). Null when memory is refused. */
  PoseBuf* acquire();
  /** A buffer the caller replaced (or never used): no new chain may name it; it becomes free
   * once the chains that do have run. */
  void retire(PoseBuf* b);

  u32 staticBytes() const { return stat.bytes(); }
  u32 poseBytes() const { return poseLayout.bytes(); }
  u32 gatherBytes() const { return (u32)(lvl.src.size() * sizeof(u32) + lvl.first.size() * sizeof(u32)); }
  u32 rangeBytes() const { return (u32)(rngOff.size() * sizeof(u32) + rng.size() * sizeof(u16)); }
  /** Sets the run headers of a pose buffer that somebody else owns (a render slot's side):
   * `data` is 16-byte aligned and poseBytes() long. What acquire() does to a new buffer. */
  void initBuffer(u8* data) const;
  /** Pose buffers made so far, and the most that were ever away from the pool at once. */
  u32 buffersMade() const { return made; }
  u32 buffersInUse() const { return inUse; }
  u32 buffersPeak() const { return peakInUse; }

 private:
  void clear();

  bool isBuilt = false;
  bool memoryRefused = false;
  Level lvl;
  PoseLayout poseLayout;
  StaticLayout stat;
  StaPipCompactLayout lay{};
  StaPipIndexedLayout indexedView;
  u32 parts = 0;
  std::vector<u32> rngOff;
  std::vector<u16> rng;
  u8* staticRaw = nullptr;
  u8* staticData = nullptr;
  bool staticRegistered = false;
  // Both lists have room for every buffer made (acquire reserves it before it makes one), so
  // retire() - which cannot fail - never allocates.
  PoseList retiring;  // waiting for their chains, oldest first
  PoseList all;       // every buffer made (owned)
  u32 made = 0, inUse = 0, peakInUse = 0;
};

}  // namespace Indexed
}  // namespace SkelSkin
}  // namespace Tyra
