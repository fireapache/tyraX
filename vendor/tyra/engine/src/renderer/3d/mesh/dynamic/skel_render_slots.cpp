// Render slots (renderer/3d/mesh/dynamic/skel_render_slots.hpp, docs/render-slots.md).

#include "renderer/3d/mesh/dynamic/skel_render_slots.hpp"

#include <cstring>
#include <new>

#if defined(_EE)
#include "renderer/core/paths/path1/vif1_queue.hpp"
#endif
#if TYRA_MEMORY_MANAGER
#include "memory/manager.hpp"
#endif

namespace Tyra {

// ---------------------------------------------------------------------------
// SkelSlotTable

void SkelSlotTable::configure(const u32 slotsPerTier[kMaxSlotTiers], u8 tierCount) {
  tiers = tierCount > kMaxSlotTiers ? kMaxSlotTiers : tierCount;
  counts = Stats();
  slots.clear();
  u32 n = 0;
  for (u32 t = 0; t < kMaxSlotTiers; ++t) {
    tierBegin[t] = tierEnd[t] = n;
    if (t < tiers) {
      n += slotsPerTier[t];
      tierEnd[t] = n;
    }
  }
  slots.resize(n);
  for (u32 t = 0; t < tiers; ++t)
    for (u32 s = tierBegin[t]; s < tierEnd[t]; ++s) slots[s].tier = static_cast<u8>(t);
}

s32 SkelSlotTable::slotOf(u32 body) const {
  for (size_t s = 0; s < slots.size(); ++s)
    if (slots[s].owner == body) return static_cast<s32>(s);
  return -1;
}

u8 SkelSlotTable::tierOf(s32 slot) const {
  return slot >= 0 && static_cast<size_t>(slot) < slots.size() ? slots[slot].tier : 0;
}

u32 SkelSlotTable::ownerOf(s32 slot) const {
  return slot >= 0 && static_cast<size_t>(slot) < slots.size() ? slots[slot].owner : kNoBody;
}

s32 SkelSlotTable::freeSlot(u8 tier, ReadyFn ready, void* context) const {
  if (tier >= tiers) return -1;
  for (u32 s = tierBegin[tier]; s < tierEnd[tier]; ++s)
    if (slots[s].owner == kNoBody && (ready == nullptr || ready(s, context)))
      return static_cast<s32>(s);
  return -1;
}

void SkelSlotTable::assign(s32 slot, u32 body, u32 frame, u8 wantTier) {
  Slot& s = slots[slot];
  s.owner = body;
  s.lastTouch = frame;
  s.qualityFrame = wantTier <= s.tier ? frame : 0;
  if (++counts.inUse[s.tier] > counts.peak[s.tier]) counts.peak[s.tier] = counts.inUse[s.tier];
}

void SkelSlotTable::free(s32 slot) {
  Slot& s = slots[slot];
  if (s.owner == kNoBody) return;
  s.owner = kNoBody;
  if (counts.inUse[s.tier] > 0) --counts.inUse[s.tier];
}

s32 SkelSlotTable::acquire(u32 body, u8 wantTier, u32 frame, bool* fresh, ReadyFn ready,
                           void* context) {
  if (fresh != nullptr) *fresh = false;
  if (tiers == 0 || slots.empty()) {
    ++counts.refused;
    return -1;
  }
  if (wantTier >= tiers) wantTier = static_cast<u8>(tiers - 1);
  const s32 held = slotOf(body);
  if (held >= 0) {
    Slot& h = slots[held];
    h.lastTouch = frame;
    if (h.tier == wantTier) {
      h.qualityFrame = frame;
      return held;
    }
    if (h.tier < wantTier) {
      // Better than asked for: kept until the body has not needed this quality for a hold.
      if (frame - h.qualityFrame < kSlotHoldFrames) return held;
      const s32 f = freeSlot(wantTier, ready, context);
      if (f < 0) return held;
      free(held);
      assign(f, body, frame, wantTier);
      if (fresh != nullptr) *fresh = true;
      return f;
    }
    // Cheaper than asked for (a fall back, or a better tier that was full): take a better tier
    // the moment one is free.
    h.qualityFrame = frame;
    for (u8 t = wantTier; t < h.tier; ++t) {
      const s32 f = freeSlot(t, ready, context);
      if (f < 0) continue;
      free(held);
      assign(f, body, frame, wantTier);
      if (t > wantTier) ++counts.fellBack;
      if (fresh != nullptr) *fresh = true;
      return f;
    }
    return held;
  }
  for (u8 t = wantTier; t < tiers; ++t) {
    const s32 f = freeSlot(t, ready, context);
    if (f < 0) continue;
    assign(f, body, frame, wantTier);
    if (t > wantTier) ++counts.fellBack;
    if (fresh != nullptr) *fresh = true;
    return f;
  }
  ++counts.refused;
  return -1;
}

void SkelSlotTable::touch(u32 body, u32 frame) {
  const s32 s = slotOf(body);
  if (s >= 0) slots[s].lastTouch = frame;
}

void SkelSlotTable::release(u32 body) {
  const s32 s = slotOf(body);
  if (s >= 0) free(s);
}

void SkelSlotTable::frameEnd(u32 frame) {
  for (size_t s = 0; s < slots.size(); ++s)
    if (slots[s].owner != kNoBody && frame - slots[s].lastTouch >= kSlotHoldFrames &&
        frame >= slots[s].lastTouch)
      free(static_cast<s32>(s));
}

void SkelSlotTable::resetStats() {
  for (u32 t = 0; t < kMaxSlotTiers; ++t) counts.peak[t] = counts.inUse[t];
  counts.fellBack = counts.refused = 0;
}

// ---------------------------------------------------------------------------
// SkelRenderSlots

namespace {

u8* defaultAlloc(u32 bytes) {
#if TYRA_MEMORY_MANAGER
  Memory::MemScope memoryScope(Memory::Budget::Skinning);
#endif
  return new (std::nothrow) u8[bytes];
}

// A slot's sides are registered spans: the frames that draw them borrow them in place, and a
// side is written only once no frame names it. A side whose span the table could not take is
// copied by the draws instead, and is ready whenever its pass is.
bool registerSide(const u8* side, u32 bytes) {
#if defined(_EE)
  return Vif1Queue::registerImmutableSpan(side, bytes / 16);
#else
  (void)side;
  (void)bytes;
  return false;
#endif
}

}  // namespace

SkelSlot* SkelSlot::make(u32 partCapacity) {
  if (partCapacity == 0 || partCapacity > kMaxSlotParts) return nullptr;
  const size_t head = (sizeof(SkelSlot) + 15) & ~static_cast<size_t>(15);
  const size_t offsets = (static_cast<size_t>(partCapacity) * sizeof(u32) + 15) &
                         ~static_cast<size_t>(15);
  const size_t streamBytes = 2 * static_cast<size_t>(partCapacity) * sizeof(StaPipCompactStream);
  u8* block = static_cast<u8*>(::operator new(head + offsets + streamBytes, std::nothrow));
  if (block == nullptr) return nullptr;
  SkelSlot* slot = new (block) SkelSlot();
  slot->partCapacity = partCapacity;
  slot->partOffset = new (block + head) u32[partCapacity]();
  slot->streams = reinterpret_cast<StaPipCompactStream*>(block + head + offsets);
  for (u32 i = 0; i < 2 * partCapacity; ++i) new (&slot->streams[i]) StaPipCompactStream();
  return slot;
}

void SkelSlot::destroy(SkelSlot* slot) {
  if (slot == nullptr) return;
  for (u32 i = 0; i < 2 * slot->partCapacity; ++i) slot->streams[i].~StaPipCompactStream();
  slot->~SkelSlot();
  ::operator delete(slot);
}

SkelRenderSlots::~SkelRenderSlots() { reset(); }

void SkelRenderSlots::reset() {
  for (size_t i = 0; i < slotList.size(); ++i) {
    SkelSlot* s = slotList[i];
    for (u32 side = 0; side < 2; ++side) {
      for (u32 p = 0; p < s->partCapacity; ++p) s->stream(side, p).release();
#if defined(_EE)
      // Unregistering waits for every frame that reads the span.
      if (s->memory[side] != nullptr) Vif1Queue::unregisterImmutableSpan(s->memory[side]);
#endif
    }
    SkelSlot::destroy(s);
    delete[] raws[i];
  }
  slotList.clear();
  raws.clear();
  for (u32 t = 0; t < kMaxSlotTiers; ++t) carved[t] = slotSize[t] = 0;
  const u32 none[kMaxSlotTiers] = {0, 0, 0, 0};
  table.configure(none, 0);
}

void SkelRenderSlots::layout(const SkelModel* const* models, u32 count,
                             const u32 tierBytes[kMaxSlotTiers], u8 tiers, u32 packageSize,
                             SizeFn size, AllocFn alloc, u32 partCapacity) {
  reset();
  if (partCapacity == 0 || partCapacity > kMaxSlotParts) partCapacity = kMaxSlotParts;
  allocator = alloc != nullptr ? alloc : defaultAlloc;
  if (tiers > kMaxSlotTiers) tiers = kMaxSlotTiers;
  for (u8 t = 0; t < tiers; ++t) {
    // One side of the largest model's parts at this tier; a slot is two sides.
    u32 side = 0;
    for (u32 m = 0; m < count; ++m) {
      if (models[m] == nullptr) continue;
      const u32 b = size(*models[m], t, packageSize);
      if (b > side) side = b;
    }
    side = (side + 15) & ~15u;
    if (side == 0 || tierBytes[t] < 2 * side) continue;
    const u32 want = tierBytes[t] / (2 * side);
    slotSize[t] = 2 * side;
    slotList.reserve(slotList.size() + want);  // exact growth: no doubling slack
    raws.reserve(raws.size() + want);
    for (u32 i = 0; i < want; ++i) {
      u8* raw = allocator(2 * side + 15);
      if (raw == nullptr) break;  // refused: the tier gets the slots it has
      SkelSlot* s = SkelSlot::make(partCapacity);
      if (s == nullptr) {
        delete[] raw;
        break;
      }
      s->tier = t;
      s->sideBytes = side;
      s->memory[0] = reinterpret_cast<u8*>((reinterpret_cast<uintptr_t>(raw) + 15) &
                                           ~static_cast<uintptr_t>(15));
      s->memory[1] = s->memory[0] + side;
      registerSide(s->memory[0], side);
      registerSide(s->memory[1], side);
      slotList.push_back(s);
      raws.push_back(raw);
      ++carved[t];
    }
  }
  // The slot numbers run through the tiers in order, as the carve did.
  table.configure(carved, tiers);
  u32 n = 0;
  for (SkelSlot* s : slotList) s->index = n++;
}

bool SkelRenderSlots::sideReady(const SkelSlot& slot, u8 side) {
#if defined(_EE)
  // A side that never became a span is copied by its draws and is never named in place.
  return Vif1Queue::immutableSpanReady(slot.memory[side]);
#else
  (void)slot;
  (void)side;
  return true;
#endif
}

namespace {
struct ReadyContext {
  const std::vector<SkelSlot*>* slots;
};

bool slotReady(u32 slot, void* context) {
  const ReadyContext* c = static_cast<const ReadyContext*>(context);
  if (slot >= c->slots->size()) return false;
  const SkelSlot& s = *(*c->slots)[slot];
  return SkelRenderSlots::sideReady(s, 0) && SkelRenderSlots::sideReady(s, 1);
}
}  // namespace

SkelSlot* SkelRenderSlots::acquire(u32 body, u8 wantTier, u32 frame, bool* fresh) {
  ReadyContext ctx{&slotList};
  bool isFresh = false;
  const s32 s = table.acquire(body, wantTier, frame, &isFresh, slotReady, &ctx);
  if (fresh != nullptr) *fresh = isFresh;
  if (s < 0) return nullptr;
  SkelSlot* slot = slotList[s];
  if (isFresh) {
    // Whatever the slot held is another body's: nothing is drawable until this body skins.
    slot->skinnedFrame = SkelSlot::kNever;
    slot->drawn = 1;  // the first pass writes side 0, then flips
  }
  return slot;
}

SkelSlot* SkelRenderSlots::slotOf(u32 body) {
  const s32 s = table.slotOf(body);
  return s >= 0 ? slotList[s] : nullptr;
}

bool SkelRenderSlots::holds(const StaPipCompactStream* stream) const {
  for (const SkelSlot* s : slotList) {
    if (!s->owns(stream)) continue;
    return table.ownerOf(static_cast<s32>(s->index)) != SkelSlotTable::kNoBody && s->written();
  }
  return false;
}

u32 SkelRenderSlots::reservedBytes() const {
  u32 total = 0;
  for (const SkelSlot* s : slotList) total += 2 * s->sideBytes;
  return total;
}

SkelRenderSlots& skelRenderSlots() {
  static SkelRenderSlots slots;
  return slots;
}

}  // namespace Tyra
