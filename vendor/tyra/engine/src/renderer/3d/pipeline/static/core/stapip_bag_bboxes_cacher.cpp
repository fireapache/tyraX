/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Sandro Sobczyński <sandro.sobczynski@gmail.com>
*/

// Modified by TyraX: bounded count variants prevent a full body and its
// reflective prefix from rebuilding one shared cache entry every frame.
#include "renderer/3d/pipeline/static/core/stapip_bag_bboxes_cacher.hpp"
#include <algorithm>
#include <new>
#if TYRA_MEMORY_MANAGER
#include <cstdio>
#include "memory/manager.hpp"
#endif

namespace Tyra {

#if TYRA_MEMORY_MANAGER
// Added by TyraX: a bag left undrawn for want of its boxes says so - once a
// second per mesh (its vertex count), as the baked and retained caches do
// (docs/memory-manager.md, "When a budget is full").
static void noteBBoxFallback(u32 vertices) {
  char asset[32];
  snprintf(asset, sizeof asset, "bag-%lu-vertices",
           static_cast<unsigned long>(vertices));
  static const char what[] = "bbox cache: bag not drawn";
  const uintptr_t key = reinterpret_cast<uintptr_t>(what) ^
                        (static_cast<uintptr_t>(vertices) << 12);
  Memory::Manager::instance().noteFallback(
      what, asset, -1, reinterpret_cast<const void*>(key));
}
#define TYRA_BBOX_SCOPE() \
  Memory::MemScope memoryScope(Memory::Budget::Renderer)
#define TYRA_BBOX_FALLBACK(vertices) noteBBoxFallback(vertices)
#else
#define TYRA_BBOX_SCOPE() ((void)0)
#define TYRA_BBOX_FALLBACK(vertices) ((void)0)
#endif

#if TYRA_STAPIP_ATTRIB
// Added by TyraX: the attribution pass' own clock, identical to the one
// StaPipCore uses. Compiled out with the counters it feeds.
static inline u32 readCacherAttribTicks() {
  u32 ticks;
  asm volatile("mfc0 %0, $9" : "=r"(ticks));
  return ticks;
}
#define TYRA_CACHER_INC(field) (++stats.field)
#else
#define TYRA_CACHER_INC(field) ((void)0)
#endif

StapipBagBBoxesCacher::StapipBagBBoxesCacher() {
  std::fill(indexBuckets, indexBuckets + indexBucketCount, -1);
}

StapipBagBBoxesCacher::~StapipBagBBoxesCacher() {}

void StapipBagBBoxesCacher::onFrameEnd() {
#if TYRA_STAPIP_ATTRIB
  const u32 attribStart = readCacherAttribTicks();
#endif
  for (auto& item : storage) {
    if (item.framesLeftToDestroy > 0) {
      item.framesLeftToDestroy--;
    }
  }

  const auto newEnd =
      std::remove_if(storage.begin(), storage.end(),
                     [](const StapipBagBBoxesCacheItem& item) {
                       return item.framesLeftToDestroy <= 0;
                     });
  if (newEnd != storage.end()) {
    storage.erase(newEnd, storage.end());
    rebuildIndex();
  }
#if TYRA_STAPIP_ATTRIB
  // This scan is per FRAME and lives outside StaPipCore::render, so it is
  // outside `boundsTicks` and every other shipped bracket - which is exactly
  // why nothing had ever measured it.
  stats.entries = static_cast<u32>(storage.size());
  stats.frameEndTicks += readCacherAttribTicks() - attribStart;
#endif
}

StaPipBagPackagesBBox* StapipBagBBoxesCacher::getBBoxes(
    const Vec4* vertices, const u32& count, const u32& id, const u32& version,
    const u32& maxVertCount) {
  auto* cache = getCache(maxVertCount, id, count);

  if (cache) {
    cache->framesLeftToDestroy = cacheFramesCount * cacheSecondsCount;
    // Modified by TyraX: same buffer, new content - recompute in
    // place; a changed vertex count needs a fresh part split. The version
    // check alone is not enough: games that free and reallocate vertex
    // buffers (layer streaming) can present a recycled heap address whose
    // version happens to equal the dead buffer's cached one - reusing those
    // boxes misclassifies packages and indexes past the cached part count.
    // A count mismatch exposes that case here; equal-count aliasing is
    // prevented by the process-unique version stamps generated games use.
    if (cache->version != version ||
        cache->bboxes->getVertexCount() != count) {
#if TYRA_STAPIP_ATTRIB
      const u32 recalcStart = readCacherAttribTicks();
#endif
      if (cache->bboxes->getVertexCount() == count) {
        TYRA_CACHER_INC(recalcs);
        cache->bboxes->recalculate(vertices, maxVertCount);
      } else {
        // Modified by TyraX: a new split for a new count - none, and the old
        // entry kept as it was, when the heap has no room for it.
        TYRA_BBOX_SCOPE();  // Added by TyraX: the boxes are Renderer's
        if (!roomForEntry(count, maxVertCount, false)) {
          TYRA_BBOX_FALLBACK(count);
          return nullptr;
        }
        TYRA_CACHER_INC(fresh);
        cache->bboxes = std::make_unique<StaPipBagPackagesBBox>(vertices, count,
                                                                maxVertCount);
      }
#if TYRA_STAPIP_ATTRIB
      // Only the RECOMPUTE, so `bdCacheTicks` minus this is the pure lookup.
      stats.recalcTicks += readCacherAttribTicks() - recalcStart;
#endif
      cache->version = version;
    } else {
      TYRA_CACHER_INC(hits);
    }
    return cache->bboxes.get();
  }
  // Modified by TyraX: a new entry - none, with nothing changed, when the heap
  // has no room for its boxes and the storage's growth.
  TYRA_BBOX_SCOPE();  // Added by TyraX: the boxes are Renderer's
  if (!roomForEntry(count, maxVertCount,
                    storage.size() == storage.capacity())) {
    TYRA_BBOX_FALLBACK(count);
    return nullptr;
  }
  TYRA_CACHER_INC(fresh);

  auto bboxes =
      std::make_unique<StaPipBagPackagesBBox>(vertices, count, maxVertCount);

  const u32 bucket = getBucket(maxVertCount, id);
  storage.push_back(
      StapipBagBBoxesCacheItem{maxVertCount, id, version, std::move(bboxes),
                               cacheFramesCount * cacheSecondsCount,
                               indexBuckets[bucket]});

  indexBuckets[bucket] = static_cast<int>(storage.size() - 1);

  return storage.back().bboxes.get();
}

StapipBagBBoxesCacheItem* StapipBagBBoxesCacher::getCache(
    const u32& maxVertCount, const u32& id, const u32& count) {
  // Keep two common count variants (base + coat), but never one entry for
  // every changing particle count. Missing third counts reuse the least
  // recently touched variant; getBBoxes replaces its split and version.
  StapipBagBBoxesCacheItem* oldest = nullptr;
  u32 variants = 0;
  int itemIndex = indexBuckets[getBucket(maxVertCount, id)];
  while (itemIndex >= 0) {
    auto& item = storage[itemIndex];
    TYRA_CACHER_INC(probes);
    if (item.vu1MaxVertCount == maxVertCount && item.id == id) {
      if (item.bboxes->getVertexCount() == count) return &item;
      ++variants;
      if (!oldest || item.framesLeftToDestroy < oldest->framesLeftToDestroy)
        oldest = &item;
    }
    itemIndex = item.nextInBucket;
  }

  return variants >= 2 ? oldest : nullptr;
}

u32 StapipBagBBoxesCacher::getBucket(const u32& maxVertCount,
                                     const u32& id) const {
  // Mix both parts of the cache key. indexBucketCount is a power of two, so
  // the mask is cheaper than division on the EE.
  u32 hash = id * 0x9E3779B1U;
  hash ^= maxVertCount + 0x85EBCA6BU + (hash << 6) + (hash >> 2);
  hash ^= hash >> 16;
  return hash & (indexBucketCount - 1);
}

bool StapipBagBBoxesCacher::roomForEntry(const u32& count,
                                         const u32& maxVertCount,
                                         bool grows) const {
  size_t bytes = StaPipBagPackagesBBox::bytesFor(count, maxVertCount);
  if (grows)  // the vector doubles (one entry when empty)
    bytes += (storage.empty() ? 1 : 2 * storage.capacity()) *
             sizeof(StapipBagBBoxesCacheItem);
  std::unique_ptr<u8[]> room(new (std::nothrow) u8[bytes + bytes / 4 + 4096]);
  return room != nullptr;
}

void StapipBagBBoxesCacher::rebuildIndex() {
  std::fill(indexBuckets, indexBuckets + indexBucketCount, -1);
  for (u32 i = 0; i < storage.size(); ++i) {
    auto& item = storage[i];
    const u32 bucket = getBucket(item.vu1MaxVertCount, item.id);
    item.nextInBucket = indexBuckets[bucket];
    indexBuckets[bucket] = i;
  }
}

}  // namespace Tyra
