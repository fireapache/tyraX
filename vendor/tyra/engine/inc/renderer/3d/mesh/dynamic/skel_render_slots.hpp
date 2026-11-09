#pragma once

// Render slots: skinned vertex memory reserved per LOD tier at level load, and held only by the
// bodies a view sees (docs/render-slots.md). A body that no view sees owns no skinned memory at
// all; a body that is drawn takes a slot of the tier its distance asks for, keeps it while it is
// drawn and for kSlotHoldFrames frames after, and gives it back for the next body to take.
//
// Two parts:
//  - SkelSlotTable, the logic: which body holds which slot, the hold, the fall back to the next
//    cheaper tier, the counts. Plain C++, run by the host verifier.
//  - SkelRenderSlots, the memory: the slots carved from the budget at layout(), two sides each (a
//    pass writes the side no frame reads, then flips), registered as immutable spans so draws
//    borrow them in place. What a slot's bytes hold is the payload of the body's parts
//    (SkelInstance::bindSlot places it): today's plain compact stream pair, or the indexed pose
//    buffer when TYRA_SKIN_INDEXED is on - the table and the carving do not know which.

#include <tamtypes.h>

#include <vector>

#include "renderer/3d/pipeline/static/core/compact/stapip_compact_stream.hpp"

namespace Tyra {

struct SkelModel;

constexpr u8 kMaxSlotTiers = 4;
constexpr u32 kSlotHoldFrames = 30;
/** The most parts a body can have and use slots: a model with more draws from per-body streams. */
constexpr u32 kMaxSlotParts = 8;

class SkelSlotTable {
 public:
  /** Whether a free slot may be given to a new owner now: no frame still reads its memory. */
  using ReadyFn = bool (*)(u32 slot, void* context);

  struct Stats {
    u32 inUse[kMaxSlotTiers] = {0, 0, 0, 0};
    u32 peak[kMaxSlotTiers] = {0, 0, 0, 0};
    u32 fellBack = 0;  // slots given at a cheaper tier than the body asked for
    u32 refused = 0;   // acquires that found no slot at any tier from the one asked for down
  };

  /** `slotsPerTier[t]` slots of tier t (0 = the most expensive), for the first `tiers` tiers.
   * Slot numbers run through the tiers in order. Every slot is free afterwards. */
  void configure(const u32 slotsPerTier[kMaxSlotTiers], u8 tiers);

  /** The slot `body` draws from at `frame`, or -1: the one it holds when that serves it, else a
   * free one of `wantTier`, else of the next cheaper tier that has one (fellBack counts it), and
   * -1 - refused counts it - only past the cheapest tier. A body that holds a slot is never left
   * without one: it keeps a slot cheaper than it asks for when no better tier is free, and one
   * better than it asks for until it has not needed that quality for kSlotHoldFrames (a body one
   * view of a split screen sees near and the other far keeps the near tier). `fresh`, when given,
   * says the slot is newly the body's: whatever it holds is not this body's. */
  s32 acquire(u32 body, u8 wantTier, u32 frame, bool* fresh = nullptr, ReadyFn ready = nullptr,
              void* context = nullptr);
  /** The body is drawn at `frame` without asking again (a follower of its pose). */
  void touch(u32 body, u32 frame);
  /** Gives back the slots no acquire or touch has reached for kSlotHoldFrames frames. */
  void frameEnd(u32 frame);
  /** Gives back the body's slot now (its object is gone). */
  void release(u32 body);
  /** The slot a body holds, or -1. */
  s32 slotOf(u32 body) const;
  u8 tierOf(s32 slot) const;
  u32 slotCount() const { return static_cast<u32>(slots.size()); }
  u32 tierCount() const { return tiers; }
  u32 slotsInTier(u8 tier) const { return tier < tiers ? tierEnd[tier] - tierBegin[tier] : 0; }
  /** The body holding a slot (kNoBody: free). */
  static constexpr u32 kNoBody = 0xFFFFFFFFu;
  u32 ownerOf(s32 slot) const;
  Stats stats() const { return counts; }
  /** Zeroes the peaks and the two counters (a gym measures a window). */
  void resetStats();

 private:
  struct Slot {
    u32 owner = kNoBody;
    u32 lastTouch = 0;
    u32 qualityFrame = 0;  // the last frame the owner asked for this tier or a cheaper one
    u8 tier = 0;
  };
  std::vector<Slot> slots;
  u32 tierBegin[kMaxSlotTiers] = {0, 0, 0, 0};
  u32 tierEnd[kMaxSlotTiers] = {0, 0, 0, 0};
  u8 tiers = 0;
  Stats counts;

  s32 freeSlot(u8 tier, ReadyFn ready, void* context) const;
  void assign(s32 slot, u32 body, u32 frame, u8 wantTier);
  void free(s32 slot);
};

/** One slot: its two sides' memory, and the streams the body's parts draw from, placed in it.
 * A slot is made for bodies of up to `partCapacity` parts - the most any model of the level has
 * - and holds that many streams a side, not the engine's ceiling (kMaxSlotParts). The object,
 * the parts' offsets and the streams are one block (make / destroy). */
struct SkelSlot {
  static constexpr u32 kNever = 0xFFFFFFFFu;

  /** A slot for bodies of up to `partCapacity` parts (1..kMaxSlotParts), or null when the heap
   * has no room for it. */
  static SkelSlot* make(u32 partCapacity);
  static void destroy(SkelSlot* slot);

  SkelSlot(const SkelSlot&) = delete;
  SkelSlot& operator=(const SkelSlot&) = delete;

  u8 tier = 0;
  u32 index = 0;
  u8* memory[2] = {nullptr, nullptr};  // the two sides, 16-byte aligned
  u32 sideBytes = 0;
  u8 drawn = 0;       // the side the body draws from (valid once skinnedFrame != kNever)
  u32 skinnedFrame = kNever;  // the frame the drawn side was written
  /** What the streams below are placed for: the serial of a SkelModel (0 = none). A body of
   * another model re-places them. Not the model's address: a model freed and another loaded at
   * the same address would be taken for the same one. */
  u32 boundModel = 0;
  u32 partCapacity = 0;  // the parts a body may have to use this slot
  u32 partCount = 0;
  u32* partOffset = nullptr;  // [partCapacity] a part's place in a side
  StaPipCompactStream* streams = nullptr;  // [2][partCapacity]: side 0's parts, then side 1's

  /** The stream of a side's part. */
  StaPipCompactStream& stream(u32 side, size_t part) { return streams[side * partCapacity + part]; }
  const StaPipCompactStream& stream(u32 side, size_t part) const {
    return streams[side * partCapacity + part];
  }
  /** Whether `s` is one of this slot's streams. */
  bool owns(const StaPipCompactStream* s) const {
    return s >= streams && s < streams + 2 * static_cast<size_t>(partCapacity);
  }
  /** The stream a part draws from now. */
  StaPipCompactStream* current(size_t part) { return &stream(drawn, part); }
  /** The stream a pass writes (the side no frame reads). */
  StaPipCompactStream* spare(size_t part) { return &stream(static_cast<u32>(1 - drawn), part); }
  u8 spareSide() const { return static_cast<u8>(1 - drawn); }
  /** The written side becomes the drawn one. */
  void flip() { drawn = static_cast<u8>(1 - drawn); }
  bool written() const { return skinnedFrame != kNever; }

 private:
  SkelSlot() = default;
};

class SkelRenderSlots {
 public:
  /** What one slot of `tier` needs for `model`, in bytes of ONE side (16-aligned); 0 when the
   * model cannot use slots at that tier. SkelInstance::slotSideBytes is the engine's. */
  using SizeFn = u32 (*)(const SkelModel& model, u8 tier, u32 packageSize);
  /** Memory for a slot's two sides: `bytes`, or null. Defaults to a nothrow new[] under the
   * Skinning budget; the host verifier hands in a limited one. */
  using AllocFn = u8* (*)(u32 bytes);
  /** Gives back what an AllocFn returned (the pointer exactly as it returned it). The default
   * allocator's is delete[]; an allocator that does not come from new[] says how to free its
   * blocks here. */
  using FreeFn = void (*)(u8* block);

  SkelRenderSlots() = default;
  ~SkelRenderSlots();
  SkelRenderSlots(const SkelRenderSlots&) = delete;
  SkelRenderSlots& operator=(const SkelRenderSlots&) = delete;

  /** Level load: carves `tierBytes[t]` bytes of each tier into floor(tierBytes[t] / slotBytes[t])
   * slots, slotBytes[t] = two sides of the largest of the `count` models at that tier. A refused
   * allocation ends a tier's carve there (fewer slots). Frees what an earlier layout held.
   * `partCapacity` is the most parts a body of the level has (the slots hold that many streams a
   * side); a body of more parts cannot use a slot. Everything the layout allocates - the sides,
   * the slot objects, the tables - is charged to the Skinning budget and asked for nothrow
   * (the lists are probed first), so a full budget gives fewer slots, never a halt. `release`
   * frees what `alloc` returned (default: delete[], which the default allocator pairs with). */
  void layout(const SkelModel* const* models, u32 count, const u32 tierBytes[kMaxSlotTiers],
              u8 tiers, u32 packageSize, SizeFn size, AllocFn alloc = nullptr,
              u32 partCapacity = kMaxSlotParts, FreeFn release = nullptr);
  /** Frees every slot (waits for the frames that read them) and forgets the layout. */
  void reset();
  bool active() const { return !slotList.empty(); }

  /** The body's slot for this frame (see SkelSlotTable::acquire); null when refused. `fresh` is
   * set when the slot's content is not this body's pose. */
  SkelSlot* acquire(u32 body, u8 wantTier, u32 frame, bool* fresh = nullptr);
  void touch(u32 body, u32 frame) { table.touch(body, frame); }
  void release(u32 body) { table.release(body); }
  void frameEnd(u32 frame) { table.frameEnd(frame); }
  SkelSlot* slotOf(u32 body);
  /** Whether `stream` is one of a slot some body holds now, written and drawable: a bag that
   * kept a slot's stream after the slot went back is told apart from one that still draws it. */
  bool holds(const StaPipCompactStream* stream) const;
  SkelSlotTable::Stats stats() const { return table.stats(); }
  void resetStats() { table.resetStats(); }
  const SkelSlotTable& slotTable() const { return table; }

  /** Per tier: slots carved, and the bytes of one slot (two sides) - the layout's result. */
  u32 slotsCarved(u8 tier) const { return tier < kMaxSlotTiers ? carved[tier] : 0; }
  u32 slotBytes(u8 tier) const { return tier < kMaxSlotTiers ? slotSize[tier] : 0; }
  /** Whether no frame still reads the side (it may be written): always true off the EE. */
  static bool sideReady(const SkelSlot& slot, u8 side);
  /** Bytes held by every slot's memory now. */
  u32 reservedBytes() const;

 private:
  SkelSlotTable table;
  std::vector<SkelSlot*> slotList;  // by slot number
  u32 carved[kMaxSlotTiers] = {0, 0, 0, 0};
  u32 slotSize[kMaxSlotTiers] = {0, 0, 0, 0};
  std::vector<u8*> raws;  // the allocations, by slot number
  AllocFn allocator = nullptr;
  FreeFn releaser = nullptr;
};

/** The one set of slots a game has (the template and the instance code share it). */
SkelRenderSlots& skelRenderSlots();

}  // namespace Tyra
