/*
# Modified by TyraX - persistent qbuffer pools.
# The original allocated and freed up to four Vec4 arrays PER FILL CALL,
# per subpackage, per frame. Arrays are now allocated once per buffer at
# maxVertCount capacity and reused. The fillByCopy* paths copy whole arrays
# with memcpy instead of a per-vertex Vec4::copy call (plus four branch
# checks) per element - these copies run for every clip-classified
# subpackage, every frame. Based on the original by
# Sandro Sobczynski (h4570/tyra), Apache License 2.0.
#
# Modified by TyraX - the compact vertex format (docs/compact-vertex-format.md).
# A buffer is a set of compact RUNS - a header quadword and its payload per
# attribute - read in place out of the bag's compact stream, or packed into
# the slot's pool block from the stream's elements or the bag's float arrays.
# The float arrays remain for the EE clipper and the as_is programs only.
*/

#include "renderer/3d/pipeline/static/core/stapip_qbuffer.hpp"
#include "renderer/3d/pipeline/static/core/compact/stapip_compact_tyra.hpp"
#include <string.h>
#include <new>
#include <sstream>
#include <iomanip>

#include "debug/hardware_trace.hpp"
#include "renderer/core/paths/path1/vif1_queue.hpp"
#if TYRA_MEMORY_MANAGER
#include "memory/manager.hpp"
#endif

namespace Tyra {

namespace {

struct QBufferPool {
  const void* owner = nullptr;
  // The float streams: the EE clipper's input and output (as_is programs).
  Vec4* vertices = nullptr;
  Vec4* sts = nullptr;
  Vec4* colors = nullptr;
  Vec4* normals = nullptr;
  u16 capacity = 0;
  // Modified by TyraX: the compact runs a packing fill writes, back to back.
  u8* compactRaw = nullptr;
  u8* compact = nullptr;  // compactRaw aligned to 16
  u32 compactQw = 0;
};

constexpr int kMaxPools = 32;

// Modified by TyraX: TWO pools per slot, used alternately. A slot's arrays are
// what the packet's REF tags point at, and the renderer hands the slot to the
// next bag as soon as its packet is SENT - the DMA is still reading the arrays
// while the next bag's copy lands in them. The side flips together with the
// renderer's packet double buffer (StaPipQBufferRenderer::sendPacket flips
// both), and that send waits for the previous DMA first, so the side being
// written is always the one whose transfer has finished. Same guarantee the
// packet buffers already had, extended to the data they reference. Measured
// on a real PS2: without it the last small bag of a run (a lamp's corona) came
// out as a sliver to the screen corner in 4-19 of 30 frames, with an EE-side
// wait per bag it cost 4 FPS; this costs nothing (docs/vu1-clipping.md).
// Modified by TyraX: one side per packet buffer. With the VIF1 queue
// (vif1_queue.hpp) up to kDepth packets are in flight, so the side being
// written must be the one belonging to the packet buffer the renderer is about
// to reuse - the renderer waits for THAT packet's chain before flipping here.
constexpr int kPoolSides = TYRA_VIF1_QUEUE ? static_cast<int>(Vif1Queue::kDepth) : 2;
QBufferPool pools[kPoolSides][kMaxPools];
int g_poolSide = 0;
bool g_poolTouched = false;  // see StaPipQBuffer::poolTouched
// Modified by TyraX: the bag every fill reads (StaPipQBuffer::setSource).
StaPipQBufferSource g_source;
u32 g_packedPackages = 0;
// Added by TyraX: indexed packages sent as they are / expanded to plain corners.
u32 g_indexedSent = 0;
u32 g_indexedExpanded = 0;

QBufferPool* poolFor(const void* owner, int side) {
  QBufferPool* set = pools[side];
  for (int i = 0; i < kMaxPools; i++)
    if (set[i].owner == owner) return &set[i];
  for (int i = 0; i < kMaxPools; i++)
    if (set[i].owner == nullptr) {
      set[i].owner = owner;
      return &set[i];
    }
  return nullptr;  // more buffers than pools - fall back to plain new[]
}

QBufferPool* poolFor(const void* owner) { return poolFor(owner, g_poolSide); }

// The most quadwords `n` vertices of any layout take as runs: three runs, the
// widest element 8 bytes (billboard weights).
u32 worstBodyQwords(u32 n) { return 3 * (1 + (n * 8 + 15) / 16); }

}  // namespace

StaPipQBuffer::StaPipQBuffer() {
  size = 0;
  // Modified by TyraX: was left uninitialised until the renderer's first
  // setMaxVertCount fan-out. That fan-out now skips a no-op propagation
  // (StaPipQBufferRenderer::setMaxVertCount), so a defined starting value is
  // part of the invariant rather than tidiness.
  maxVertCount = 0;
  clipPlaneMask = 0;
  stripped = false;
  retainIndex = -1;  // Modified by TyraX: see the field's comment.
  bakeIndex = -1;    // Modified by TyraX: see the field's comment.
  streamPackage = -1;
  bag = nullptr;
  indexedStream = nullptr;
  clearRuns();
#if TYRA_STAPIP_PROBE_UNCACHED_CHAIN
  probeCopyFilled = false;  // Probe B, see stapip_probes.hpp.
#endif
  _isDynamicallyAllocated = false;
  _stAllocated = false;
  _colorAllocated = false;
  _normalAllocated = false;

  vertices = nullptr;
  colors = nullptr;
  sts = nullptr;
  normals = nullptr;
}

StaPipQBuffer::~StaPipQBuffer() { deallocateDynamicData(); }

void StaPipQBuffer::flipPoolSide() {
  g_poolSide = (g_poolSide + 1) % kPoolSides;
  g_poolTouched = false;
}

bool StaPipQBuffer::poolTouched() { return g_poolTouched; }

void StaPipQBuffer::setSource(const StaPipQBufferSource& source) {
  g_source = source;
}

const StaPipQBufferSource& StaPipQBuffer::source() { return g_source; }

u32 StaPipQBuffer::takePackedPackages() {
  const u32 v = g_packedPackages;
  g_packedPackages = 0;
  return v;
}

// Modified by TyraX: is this address inside one of the copy pools?
//
// Diagnostic only, and it exists for one question. The acceptance gate's
// geometry hash is not reproducible between two boots of ONE ELF at a frozen
// pose, while the VIFcodes, the uniforms and the picture all are
// (docs/baked-stream-acceptance-gate.md). The suspect is these pools: a slot
// keeps its arrays between bags and only the first `size` vertices of each are
// rewritten, so anything transferred past that is whatever the pool held last.
// This lets the hash fold pool-sourced payload separately from bag-sourced
// payload and settle it by measurement instead of by reading the writers.
bool StaPipQBuffer::isPoolAddress(const void* addr) {
  if (addr == nullptr) return false;
  const u8* a = static_cast<const u8*>(addr);
  for (int side = 0; side < kPoolSides; ++side) {
    for (int i = 0; i < kMaxPools; ++i) {
      const QBufferPool& p = pools[side][i];
      if (p.compact != nullptr && a >= p.compact &&
          a < p.compact + p.compactQw * 16)
        return true;
      if (p.capacity == 0) continue;
      const u32 bytes = static_cast<u32>(p.capacity) * sizeof(Vec4);
      const Vec4* arrays[4] = {p.vertices, p.sts, p.colors, p.normals};
      for (int k = 0; k < 4; ++k) {
        if (arrays[k] == nullptr) continue;
        const u8* base = reinterpret_cast<const u8*>(arrays[k]);
        if (a >= base && a < base + bytes) return true;
      }
    }
  }
  return false;
}

void StaPipQBuffer::setMaxVertCount(const u32& count) { maxVertCount = count; }

void StaPipQBuffer::clearRuns() {
  for (int k = 0; k < 3; ++k) {
    run[k] = nullptr;
    runQwords[k] = 0;
  }
  bodyQwords = 0;
  indexedStream = nullptr;
}

void StaPipQBuffer::noteIndexedSent(u32 n) { g_indexedSent += n; }

void StaPipQBuffer::takeIndexedCounts(u32* sent, u32* expanded) {
  *sent = g_indexedSent;
  *expanded = g_indexedExpanded;
  g_indexedSent = 0;
  g_indexedExpanded = 0;
}

void StaPipQBuffer::fillByIndexed(const StaPipCompactStream& stream, u32 p) {
  deallocateDynamicData();
  vertices = sts = colors = normals = nullptr;
  const StaPipIndexedLayout* il = stream.indexedLayout();
  const u32 n = stream.packageVertices(p);
  size = n;
  TYRA_ASSERT(il != nullptr && size <= maxVertCount, "VU1 buffer supports only ", maxVertCount,
              " verts. Provided: ", size);
  clearRuns();
  run[0] = static_cast<const u8*>(stream.data()) + il->poseBlock[p];
  runQwords[0] = (il->poseBlock[p + 1] - il->poseBlock[p]) / 16;
  run[1] = il->staticData + il->cornerHeader(p);
  runQwords[1] = 1 + StaPipIndexedLayout::cornerQw(n);
  bodyQwords = runQwords[0] + runQwords[1];
  indexedStream = &stream;
  stripped = false;
  clipPlaneMask = 0;
  streamPackage = static_cast<int>(p);
  bag = const_cast<StaPipBag*>(g_source.bag);
}

void StaPipQBuffer::expandIndexed() {
  const StaPipCompactStream* stream = indexedStream;
  if (stream == nullptr) return;
  StaPipBag* keepBag = bag;
  const u8 mask = clipPlaneMask;
  const u32 first = static_cast<u32>(streamPackage) * stream->layout().packageSize;
  const u32 n = size;
  if (!pack(nullptr, first, n)) {
    // The copy pool could not grow (a refusing Renderer budget): the package is empty and draws
    // nothing this frame; the next one asks again. Say so - once a second per stream.
#if TYRA_MEMORY_MANAGER
    Memory::Manager::instance().noteFallback("indexed package expand: no pool block",
                                             "indexed stream", 0, stream);
#endif
  }
  bag = keepBag;
  clipPlaneMask = mask;
  stripped = false;
}

void StaPipQBuffer::fillByStream(const StaPipCompactStream& stream, u32 p) {
  deallocateDynamicData();
  vertices = sts = colors = normals = nullptr;
  const StaPipCompactLayout& l = stream.layout();
  size = stream.packageVertices(p);
  TYRA_ASSERT(size <= maxVertCount, "VU1 buffer supports only ", maxVertCount,
              " verts. Provided: ", size);
  clearRuns();
  for (u8 a = 0; a < l.attrCount && a < 3; ++a) {
    run[a] = stream.runHeader(p, a);
    runQwords[a] = stream.runQwords(p, a);
    bodyQwords += 1 + runQwords[a];
  }
  stripped = l.stripped;
  clipPlaneMask = 0;
  streamPackage = static_cast<int>(p);
  bag = const_cast<StaPipBag*>(g_source.bag);
}

// Pack n vertices of the source bag into this slot's pool block as the
// source layout's runs, back to back.
bool StaPipQBuffer::pack(const u32* index, u32 first, u32 n) {
  HardwareTrace::Scope trace("QBuffer_pack");
#if TYRA_STAPIP_PROBE_UNCACHED_CHAIN
  // Probe B: this buffer's REF tags will name the copy pool, whose cached
  // stores only FlushCache writes back. See stapip_probes.hpp.
  probeCopyFilled = true;
#endif
  deallocateDynamicData();
  vertices = sts = colors = normals = nullptr;
  clearRuns();
  streamPackage = -1;
  size = 0;
  const StaPipQBufferSource& src = g_source;
  const StaPipCompactLayout& l = src.layout;
  TYRA_ASSERT(n <= maxVertCount && n <= StaPipCompactWriter::kMaxPackage,
              "VU1 buffer supports only ", maxVertCount, " verts. Provided: ",
              n);
  if (n == 0 || l.attrCount == 0) return true;

  u32 qw = 0;
  for (u8 a = 0; a < l.attrCount && a < 3; ++a)
    qw += 1 + Compact::runQwords(l.attrs[a], n);

  QBufferPool* pool = poolFor(this);
  if (pool == nullptr) return false;  // more buffers than pools: never here
  if (pool->compactQw < qw) {
    // Grown once to what the slot's largest package can take, nothrow: a
    // refused block draws nothing this frame and asks again the next.
    const u32 want = qw > worstBodyQwords(maxVertCount)
                         ? qw
                         : worstBodyQwords(maxVertCount);
#if TYRA_MEMORY_MANAGER
    // Renderer memory, as the float pool it replaces was (a refusing budget).
    Memory::MemScope memoryScope(Memory::Budget::Renderer);
#endif
    u8* raw = new (std::nothrow) u8[want * 16 + 15];
    if (raw == nullptr) return false;
    delete[] pool->compactRaw;
    pool->compactRaw = raw;
    pool->compact = reinterpret_cast<u8*>(
        (reinterpret_cast<uintptr_t>(raw) + 15) & ~static_cast<uintptr_t>(15));
    pool->compactQw = want;
  }
  g_poolTouched = true;
  ++g_packedPackages;

  // From the bag's stream when it has one (the integers copied), else its
  // float arrays (quantized exactly as a stream is packed).
  const StaPipCompactStream* stream = src.stream;
  bool fromStream = stream != nullptr && stream->allocated() &&
                    stream->layout().attrCount == l.attrCount;
  if (fromStream && stream->isIndexed()) ++g_indexedExpanded;
  for (u8 a = 0; fromStream && a < l.attrCount; ++a)
    fromStream = stream->layout().attrs[a] == l.attrs[a];
  StaPipCompactSource floats = {};
  if (!fromStream && src.bag != nullptr) floats = Compact::sourceOf(*src.bag);

  u8* out = pool->compact;
  for (u8 a = 0; a < l.attrCount && a < 3; ++a) {
    const Compact::Attr attr = l.attrs[a];
    u32 header[4];
    Compact::runHeader(attr, n, l.dest(a, n), header);
    memcpy(out, header, sizeof(header));
    if (fromStream)
      Compact::copyRun(*stream, a, index, first, n, out + 16);
    else
      Compact::packRun(attr, floats, index, first, n, src.box, src.weightScale,
                       out + 16, l.stFixed);
    run[a] = out;
    runQwords[a] = Compact::runQwords(attr, n);
    bodyQwords += 1 + runQwords[a];
    out += (1 + runQwords[a]) * 16;
  }
  size = n;
  return true;
}

bool StaPipQBuffer::fillByPack(const StaPipBagPackage& pkg) {
  const bool ok = pack(nullptr, pkg.first, pkg.size);
  clipPlaneMask = pkg.clipPlaneMask;
  stripped = pkg.bag->stripped;
  bag = pkg.bag;
  return ok;
}

void StaPipQBuffer::fillByPointer(const StaPipBagPackage& pkg) {
  TYRA_ASSERT(pkg.size <= maxVertCount, "VU1 buffer supports only ",
              maxVertCount, " verts. Provided: ", pkg.size);
  const StaPipCompactStream* stream = g_source.stream;
  if (stream != nullptr && pkg.bag == g_source.bag && stream->allocated()) {
    const u32 ps = stream->layout().packageSize;
    const u32 p = pkg.first / ps;
    if (pkg.first % ps == 0 && pkg.size == stream->packageVertices(p)) {
      if (!stream->isIndexed()) {
        fillByStream(*stream, p);
        clipPlaneMask = pkg.clipPlaneMask;
        bag = pkg.bag;
        return;
      }
      if (g_source.indexedOk) {
        // An indexed package goes out as it is; a clip route expands it (expandIndexed).
        fillByIndexed(*stream, p);
        clipPlaneMask = pkg.clipPlaneMask;
        bag = pkg.bag;
        return;
      }
    }
  }
  fillByPack(pkg);
}

void StaPipQBuffer::fillByPointer(StaPipBag* source, u32 offset, u32 count) {
  StaPipBagPackage pkg;
  pkg.bag = source;
  pkg.first = offset;
  pkg.size = static_cast<u16>(count);
  pkg.clipPlaneMask = 0;
  fillByPointer(pkg);
  clipPlaneMask = 0;
}

void StaPipQBuffer::fillByCopyMax(const StaPipBagPackage& pkg1,
                                  const StaPipBagPackage& pkg2,
                                  const StaPipBagPackage& pkg3) {
  HardwareTrace::Scope trace("QBuffer_copy");
  TYRA_ASSERT(pkg1.size <= maxVertCount / 3,
              "Wrong package size (1). Provided: ", pkg1.size);
  TYRA_ASSERT(pkg2.size <= maxVertCount / 3,
              "Wrong package size (2). Provided: ", pkg2.size);
  TYRA_ASSERT(pkg3.size <= maxVertCount / 3,
              "Wrong package size (3). Provided: ", pkg3.size);
  u32 index[StaPipCompactWriter::kMaxPackage];
  u32 n = 0;
  const StaPipBagPackage* pkgs[3] = {&pkg1, &pkg2, &pkg3};
  for (const auto* pkg : pkgs)
    for (u32 i = 0; i < pkg->size && n < StaPipCompactWriter::kMaxPackage; ++i)
      index[n++] = pkg->first + i;
  pack(index, 0, n);
  clipPlaneMask = pkg1.clipPlaneMask | pkg2.clipPlaneMask | pkg3.clipPlaneMask;
  stripped = false;
  bag = pkg1.bag;
}

void StaPipQBuffer::fillByCopy1By2(const StaPipBagPackage& pkg1,
                                   const StaPipBagPackage& pkg2) {
  HardwareTrace::Scope trace("QBuffer_copy");
  TYRA_ASSERT(pkg1.size <= maxVertCount / 3,
              "Wrong package size (1). Provided: ", pkg1.size);
  TYRA_ASSERT(pkg2.size <= maxVertCount / 3,
              "Wrong package size (2). Provided: ", pkg2.size);
  u32 index[StaPipCompactWriter::kMaxPackage];
  u32 n = 0;
  const StaPipBagPackage* pkgs[2] = {&pkg1, &pkg2};
  for (const auto* pkg : pkgs)
    for (u32 i = 0; i < pkg->size && n < StaPipCompactWriter::kMaxPackage; ++i)
      index[n++] = pkg->first + i;
  pack(index, 0, n);
  clipPlaneMask = pkg1.clipPlaneMask | pkg2.clipPlaneMask;
  stripped = false;
  bag = pkg1.bag;
}

void StaPipQBuffer::fillByCopy1By3(const StaPipBagPackage& pkg) {
  HardwareTrace::Scope trace("QBuffer_copy");
  TYRA_ASSERT(pkg.size <= maxVertCount / 3,
              "Wrong package size (1). Provided: ", pkg.size);
  pack(nullptr, pkg.first, pkg.size);
  clipPlaneMask = pkg.clipPlaneMask;
  stripped = false;
  bag = pkg.bag;
}

// Modified by TyraX: strip -> list expansion for the clip route. See the
// header for why it exists and what the caller owes.
void StaPipQBuffer::fillByStripExpand(const StaPipBagPackage& pkg,
                                      u32 firstTri, u32 triCount) {
  HardwareTrace::Scope trace("QBuffer_copy");
  TYRA_ASSERT(triCount * 3 <= maxVertCount,
              "Strip expansion does not fit the VU1 buffer. Triangles: ",
              triCount);
  TYRA_ASSERT(firstTri + triCount + 2 <= pkg.size,
              "Strip expansion runs past the package. First: ", firstTri,
              " count: ", triCount, " size: ", pkg.size);
  u32 index[StaPipCompactWriter::kMaxPackage];
  u32 out = 0;
  for (u32 t = 0; t < triCount && out + 3 <= StaPipCompactWriter::kMaxPackage;
       t++) {
    const u32 base = firstTri + t;
    // A triangle strip flips its effective winding on every primitive. Once
    // expanded to PRIM_TRIANGLE, odd triangles must carry that flip explicitly.
    // Otherwise the clipper receives an inside-out polygon and can grow it
    // into a giant textured wedge at the near plane.
    for (u32 k = 0; k < 3; k++, out++) {
      const u32 local = ((base & 1U) && k < 2U) ? 1U - k : k;
      index[out] = pkg.first + base + local;
    }
  }
  pack(index, 0, out);
  clipPlaneMask = pkg.clipPlaneMask;
  stripped = false;
  bag = pkg.bag;
}

bool StaPipQBuffer::decodeToFloats() {
  if (bodyQwords == 0) return vertices != nullptr;  // already floats
  const StaPipQBufferSource& src = g_source;
  const StaPipCompactLayout& l = src.layout;
  const void* runs[3] = {run[0], run[1], run[2]};
  const u32 n = size;
  deallocateDynamicData();
  allocateDynamicData(static_cast<u16>(n), bag);
  if (vertices == nullptr) return false;
  for (u8 a = 0; a < l.attrCount && a < 3; ++a) {
    const u8* payload = static_cast<const u8*>(runs[a]) + 16;
    Vec4* target = nullptr;
    switch (l.attrs[a]) {
      case Compact::Attr::Position:
      case Compact::Attr::BbCenter:
        target = vertices;
        break;
      case Compact::Attr::St:
      case Compact::Attr::EnvNormal:
      case Compact::Attr::BbParams:
        target = sts;
        break;
      case Compact::Attr::Normal:
        target = normals;
        break;
      case Compact::Attr::Color:
      case Compact::Attr::BbColor:
        target = colors;
        break;
    }
    // The as_is programs' convention (Compact::decodeRunToFloats).
    if (target != nullptr)
      Compact::decodeRunToFloats(l.attrs[a], payload, n, src.box, target->xyzw);
  }
  clearRuns();
  return true;
}

void StaPipQBuffer::reallocateManually(const u16& t_size) {
  deallocateDynamicData();
  clearRuns();
  streamPackage = -1;
  allocateDynamicData(t_size, bag);
  // Modified by TyraX: a refused allocation leaves an empty buffer (skipped).
  size = vertices != nullptr ? t_size : 0;
  clipPlaneMask = 0;
  stripped = false;
}

void StaPipQBuffer::deallocateDynamicData() {
  if (!_isDynamicallyAllocated) return;

  // Pooled arrays are kept for reuse - only true heap fallbacks are freed.
  // The arrays may belong to ANY side: the side has usually moved on since
  // this slot was filled. Modified by TyraX: this used to test sides 0 and 1
  // only, which was the whole pool while there were two sides. With the VIF1
  // queue's four, a slot filled from side 2 or 3 read as "not pooled", its
  // arrays were delete[]d while the pool still owned them, and the next copy
  // into that pool landed in whatever the heap had handed out since - the
  // retained command cache, on the Motor District, so VIF1 met vertex floats
  // where DMA tags belonged ("Unknown VifCmd 3f"). Every loop over sides runs
  // to kPoolSides; there is no "the other side" any more.
  bool pooled = false;
  for (int side = 0; side < kPoolSides && !pooled; ++side) {
    const QBufferPool* pool = poolFor(this, side);
    pooled = pool && vertices != nullptr && vertices == pool->vertices;
  }

  if (!pooled) {
    delete[] vertices;

    if (_stAllocated) delete[] sts;
    if (_colorAllocated) delete[] colors;
    if (_normalAllocated) delete[] normals;
  }

  vertices = sts = colors = normals = nullptr;
  _stAllocated = false;
  _colorAllocated = false;
  _normalAllocated = false;
  _isDynamicallyAllocated = false;
}

void StaPipQBuffer::allocateDynamicData(u16 size, StaPipBag* bag) {
  TYRA_ASSERT(size <= maxVertCount, "Wrong size. Max buffer size in VU1 is ",
              maxVertCount, ". Provided: ", size);
  TYRA_ASSERT(!_isDynamicallyAllocated, "Buffer is already allocated");

  if (size == 0) return;
  g_poolTouched = true;  // pool or heap: either is a copy the packet references

  // Modified by TyraX: nothrow - the float arrays are the EE clipper's input
  // (decodeToFloats) and its output (StaPipClipper::writeChunk); a refusal
  // leaves the buffer unallocated (vertices null), and the clip of that
  // package is skipped this frame instead of halting the game.
  QBufferPool* pool = poolFor(this);
  if (pool) {
    const u16 needed = maxVertCount > size ? (u16)maxVertCount : size;
    if (pool->capacity < needed) {
#if TYRA_MEMORY_MANAGER
      // Renderer memory, a refusing budget: these grow at frame time, and a
      // full region must skip the clip, not halt the game in System's scope.
      Memory::MemScope memoryScope(Memory::Budget::Renderer);
#endif
      delete[] pool->vertices;
      delete[] pool->sts;
      delete[] pool->colors;
      delete[] pool->normals;
      pool->vertices = new (std::nothrow) Vec4[needed];
      pool->sts = new (std::nothrow) Vec4[needed];
      pool->colors = new (std::nothrow) Vec4[needed];
      pool->normals = new (std::nothrow) Vec4[needed];
      pool->capacity = needed;
      if (!pool->vertices || !pool->sts || !pool->colors || !pool->normals) {
        delete[] pool->vertices;
        delete[] pool->sts;
        delete[] pool->colors;
        delete[] pool->normals;
        pool->vertices = pool->sts = pool->colors = pool->normals = nullptr;
        pool->capacity = 0;
        return;
      }
    }
    vertices = pool->vertices;
    if (bag->texture != nullptr) {
      sts = pool->sts;
      _stAllocated = true;
    }
    if (bag->color->many != nullptr) {
      colors = pool->colors;
      _colorAllocated = true;
    }
    if (bag->lighting != nullptr) {
      normals = pool->normals;
      _normalAllocated = true;
    }
    _isDynamicallyAllocated = true;
    return;
  }

  // Fallback: original behavior (nothrow, as above)
#if TYRA_MEMORY_MANAGER
  Memory::MemScope memoryScope(Memory::Budget::Renderer);  // as above
#endif
  vertices = new (std::nothrow) Vec4[size];
  bool refused = vertices == nullptr;

  if (bag->texture != nullptr) {
    sts = new (std::nothrow) Vec4[size];
    _stAllocated = true;
    refused = refused || sts == nullptr;
  }

  if (bag->color->many != nullptr) {
    colors = new (std::nothrow) Vec4[size];
    _colorAllocated = true;
    refused = refused || colors == nullptr;
  }

  if (bag->lighting != nullptr) {
    normals = new (std::nothrow) Vec4[size];
    _normalAllocated = true;
    refused = refused || normals == nullptr;
  }

  _isDynamicallyAllocated = true;
  if (refused) deallocateDynamicData();  // all or nothing: vertices null
}

bool StaPipQBuffer::any() const { return size > 0; }

void StaPipQBuffer::print() const {
  auto text = getPrint(nullptr);
  printf("%s\n", text.c_str());
}

void StaPipQBuffer::print(const char* name) const {
  auto text = getPrint(name);
  printf("%s\n", text.c_str());
}

std::string StaPipQBuffer::getPrint(const char* name) const {
  std::stringstream res;
  if (name) {
    res << name << "(";
  } else {
    res << "StaPipQBuffer(";
  }
  res << std::fixed << std::setprecision(2);
  res << std::endl;
  res << "Size: " << static_cast<int>(size) << std::endl;
  if (bodyQwords != 0) {
    res << "Compact runs: " << bodyQwords << " qwords";
    for (int k = 0; k < 3; ++k)
      if (run[k] != nullptr) res << " [" << runQwords[k] << "]";
    res << std::endl << ")";
    return res.str();
  }
  if (vertices == nullptr) {
    res << ")";
    return res.str();
  }

  res << "Vertices: " << std::endl;
  for (u32 i = 0; i < size; i++)
    res << i << ": " << vertices[i].getPrint() << std::endl;

  if (bag->texture != nullptr && sts != nullptr) {
    res << "STs: " << std::endl;
    for (u32 i = 0; i < size; i++)
      res << i << ": " << sts[i].getPrint() << std::endl;
  }

  if (bag->color->many != nullptr && colors != nullptr) {
    res << "Colors: " << std::endl;
    for (u32 i = 0; i < size; i++)
      res << i << ": " << colors[i].getPrint() << std::endl;
  }

  if (bag->lighting != nullptr && normals != nullptr) {
    res << "Normals: " << std::endl;
    for (u32 i = 0; i < size; i++) {
      res << i << ": " << normals[i].getPrint();
      if (i < size - 1) {
        res << std::endl;
      }
    }
  }

  res << ")";

  return res.str();
}

}  // namespace Tyra
