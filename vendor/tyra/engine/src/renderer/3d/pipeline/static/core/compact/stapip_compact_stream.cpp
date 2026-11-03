/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# TyraX
#-----------------------------------------------------------------------
*/

// Added by TyraX: the compact stream (stapip_compact_stream.hpp,
// docs/compact-vertex-format.md "The stream"). The core is portable C++ - the
// host verifier (tools/verify-stapip-compact.cpp) runs exactly this code; the
// StaPipBag side is built for the EE only, at the end of the file.

#include "renderer/3d/pipeline/static/core/compact/stapip_compact_stream.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <new>
#include "renderer/3d/pipeline/static/core/programs/stapip_vu1_shared_defines.h"

#if defined(_EE)
#include <cstdio>
#include "renderer/3d/pipeline/static/core/bag/stapip_bag.hpp"
#include "renderer/3d/pipeline/static/core/compact/stapip_compact_tyra.hpp"
#include "renderer/core/paths/path1/vif1_queue.hpp"
#endif

namespace Tyra {

using Compact::Attr;

namespace {

constexpr u32 kEntryQw = 3;  // count / scale, GIF tag, their UNPACK
constexpr u32 kKickQw = 1;   // FLUSH, MSCAL
// A stream's weight scale until it is set (StaPipCompactStream::weightScale).
constexpr float kDefaultWeightScale = 4096.0f;
// Keeps every size below in u32 arithmetic; no bag comes near it.
constexpr u32 kMaxVertices = 1u << 22;
// Every grid index must be an exact float: a box's origin index too.
constexpr float kExactIndex = 16777216.0f;  // 2^24

float gSceneStep = 1.0f / 512;

bool isPowerOfTwo(float f) {
  if (!(f > 0.0f && f < 1e30f)) return false;
  int e = 0;
  return std::frexp(f, &e) == 0.5f;
}

// What worldBox and modelBox make: a power-of-two step, the origin a whole
// number of steps whose index is an exact float. Only such a box decodes to
// the same float on the EE, on a host and on the VU.
bool validBox(const Compact::Box& box) {
  if (!isPowerOfTwo(box.step)) return false;
  for (u32 a = 0; a < 3; ++a) {
    const float k = box.origin[a] / box.step;  // exact: a power-of-two step
    if (!(std::fabs(k) < kExactIndex) || k != std::trunc(k)) return false;
  }
  return true;
}

// A package of `n` vertices: its quadwords, and where each run header sits.
u32 packageLayout(const StaPipCompactLayout& layout, u32 n, u16 runAt[3]) {
  u32 at = kEntryQw;
  for (u32 a = 0; a < 3; ++a) {
    runAt[a] = 0;
    if (a >= layout.attrCount) continue;
    runAt[a] = static_cast<u16>(at);
    at += 1 + Compact::runQwords(layout.attrs[a], n);
  }
  return at + kKickQw;
}

s32 slotOf(const StaPipCompactLayout& l, Attr a, Attr b) {
  const s32 i = l.indexOf(a);
  return i >= 0 ? i : l.indexOf(b);
}

// One element of each attribute, as the writer and packArrays both store it.

void putPosition(u8* dst, const float* p, const Compact::Box& box) {
  const s16 q[3] = {Compact::quantize(p[0], box.origin[0], box.step),
                    Compact::quantize(p[1], box.origin[1], box.step),
                    Compact::quantize(p[2], box.origin[2], box.step)};
  std::memcpy(dst, q, sizeof(q));
}

void putColor(u8* dst, const float* rgba) {
  for (u32 c = 0; c < 4; ++c) dst[c] = Compact::quantizeColor(rgba[c]);
}

void putNormal(u8* dst, const float* n) {
  const s8 q[4] = {Compact::quantizeNormal(n[0]), Compact::quantizeNormal(n[1]),
                   Compact::quantizeNormal(n[2]), 0};
  std::memcpy(dst, q, sizeof(q));
}

// The env programs take the normal as a direction: written at unit length.
void putEnvNormal(u8* dst, const float* n) {
  const float len2 = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
  const float scale = len2 > 0.0f && len2 < 1e30f ? 1.0f / std::sqrt(len2) : 1.0f;
  const float unit[3] = {n[0] * scale, n[1] * scale, n[2] * scale};
  putNormal(dst, unit);
}

void putWeights(u8* dst, const float* m, float scale) {
  const s16 q[4] = {
      Compact::quantizeWeight(m[0], scale), Compact::quantizeWeight(m[1], scale),
      Compact::quantizeWeight(m[2], scale), Compact::quantizeWeight(m[3], scale)};
  std::memcpy(dst, q, sizeof(q));
}

// What quantizeSt saturates: |x x 4096| rounds past 32767.
bool stSaturates(float x) {
  const float s = x * 4096.0f;
  return s == s && !(s < 32767.5f && s > -32767.5f);
}

// The worst ST scale any package was stored at so far (Compact::stScaleBits),
// for the log line that says so once per new worst.
u32 gStWorstBits = 0;

void noteStScale(u32 bits) {
  if (bits <= gStWorstBits) return;
  gStWorstBits = bits;
#if defined(_EE)
  std::printf("[COMPACT] ST scale 2^-%u: a package's S, T and Q keep %u fewer "
              "bits of fraction\n",
              static_cast<unsigned>(bits), static_cast<unsigned>(bits));
#endif
}

// A texture coordinate of a projective STQ: S / Q (exactly S when Q is 1);
// a Q that is not positive has no repeats to count, so S itself.
float texCoordOf(float s, float q) {
  if (q == 1.0f || !(q > 0.0f)) return s;
  return s / q;
}

// One package's STQ - S, T, Q at stq[0..2], vertices `stride` floats apart -
// as a run holds it (docs/compact-vertex-format.md, "ST"): each texture
// coordinate rebased by whole repeats (S - n x Q), except the ones `stFixed`
// names (StaPipCompactLayout::stFixed: not a REPEAT wrap); then S, T and Q
// all x the package's power-of-two scale (Compact::stScaleFor), which the GS's
// S/Q cancels. Returns the scale's bits (Compact::stScaleBits); `saturated` is
// set when a value still did not fit (only infinities and NaN-free absurdities
// past 2^27 repeats).
u32 putStqs(u8* run, const float* stq, u32 n, u32 stride, u8 stFixed,
            bool* saturated) {
  *saturated = false;
  if (n == 0) return 0;
  float lo[2] = {0.0f, 0.0f}, hi[2] = {0.0f, 0.0f};
  for (u32 i = 0; i < n; ++i) {
    const float* e = stq + i * stride;
    for (u32 c = 0; c < 2; ++c) {
      const float t = texCoordOf(e[c], e[2]);
      if (i == 0 || t < lo[c]) lo[c] = t;
      if (i == 0 || t > hi[c]) hi[c] = t;
    }
  }
  const float shift[2] = {
      (stFixed & 1) ? 0.0f : Compact::stRebaseRange(lo[0], hi[0]),
      (stFixed & 2) ? 0.0f : Compact::stRebaseRange(lo[1], hi[1])};
  float largest = 0.0f;
  for (u32 i = 0; i < n; ++i) {
    const float* e = stq + i * stride;
    const float v[3] = {e[0] - shift[0] * e[2], e[1] - shift[1] * e[2], e[2]};
    for (u32 c = 0; c < 3; ++c)
      if (std::fabs(v[c]) > largest) largest = std::fabs(v[c]);
  }
  const float k = Compact::stScaleFor(largest);
  for (u32 i = 0; i < n; ++i) {
    const float* e = stq + i * stride;
    const float v[3] = {(e[0] - shift[0] * e[2]) * k,
                        (e[1] - shift[1] * e[2]) * k, e[2] * k};
    s16 q[3];
    for (u32 c = 0; c < 3; ++c) {
      *saturated = *saturated || stSaturates(v[c]);
      q[c] = Compact::quantizeSt(v[c]);
    }
    std::memcpy(run + i * 6, q, sizeof(q));
  }
  const u32 bits = Compact::stScaleBits(k);
  noteStScale(bits);
  return bits;
}

// The union of `n` boxes, six floats each (min x, y, z, max x, y, z), into
// `out`; at least one box.
void unionOf(const float* boxes, u32 n, float out[6]) {
  for (u32 a = 0; a < 6; ++a) out[a] = boxes[a];
  for (u32 i = 1; i < n; ++i)
    for (u32 a = 0; a < 3; ++a) {
      const float lo = boxes[i * 6 + a], hi = boxes[i * 6 + 3 + a];
      if (lo < out[a]) out[a] = lo;
      if (hi > out[3 + a]) out[3 + a] = hi;
    }
}

// Bounds of `count` positions 4 floats apart; NaN is skipped, none reads 0.
void boundsOf(const float* positions, u32 count, float mn[3], float mx[3]) {
  for (u32 a = 0; a < 3; ++a) {
    mn[a] = 1e30f;
    mx[a] = -1e30f;
  }
  for (u32 i = 0; positions != nullptr && i < count; ++i)
    for (u32 a = 0; a < 3; ++a) {
      const float v = positions[i * 4 + a];
      if (v < mn[a]) mn[a] = v;
      if (v > mx[a]) mx[a] = v;
    }
  for (u32 a = 0; a < 3; ++a)
    if (mn[a] > mx[a]) mn[a] = mx[a] = 0.0f;
}

}  // namespace

// ---------------------------------------------------------------------------
// StaPipCompactLayout

StaPipCompactLayout StaPipCompactLayout::forClass(bool texture, bool env,
                                                  bool lit, bool singleColor,
                                                  u32 packageSize,
                                                  bool stripped) {
  StaPipCompactLayout l;
  l.packageSize = static_cast<u16>(packageSize > 0xFFFFu ? 0xFFFFu : packageSize);
  l.stripped = stripped;
  l.billboard = false;
  l.stFixed = 0;
  l.attrCount = 0;
  l.attrs[l.attrCount++] = Attr::Position;
  if (texture) l.attrs[l.attrCount++] = env ? Attr::EnvNormal : Attr::St;
  // The last block is normals for a lit program and colours otherwise; the
  // env program is never lit (StaPipQBufferRenderer::getDrawProgramTypeByBag).
  if (lit && !(texture && env))
    l.attrs[l.attrCount++] = Attr::Normal;
  else if (!singleColor)
    l.attrs[l.attrCount++] = Attr::Color;
  for (u32 a = l.attrCount; a < 3; ++a) l.attrs[a] = Attr::Position;
  return l;
}

StaPipCompactLayout StaPipCompactLayout::forBillboards(u32 packageSize) {
  StaPipCompactLayout l;
  l.packageSize = static_cast<u16>(packageSize > 0xFFFFu ? 0xFFFFu : packageSize);
  l.stripped = false;
  l.billboard = true;
  l.stFixed = 0;
  l.attrCount = 3;
  l.attrs[0] = Attr::BbCenter;
  l.attrs[1] = Attr::BbParams;
  l.attrs[2] = Attr::BbColor;
  return l;
}

u32 StaPipCompactLayout::dest(u8 attrIndex, u32 n) const {
  return VU1_STAPIP_VERT_DATA_ADDR + attrIndex * n;
}

s32 StaPipCompactLayout::indexOf(Attr a) const {
  for (u32 i = 0; i < attrCount && i < 3; ++i)
    if (attrs[i] == a) return static_cast<s32>(i);
  return -1;
}

bool StaPipCompactLayout::operator==(const StaPipCompactLayout& o) const {
  if (packageSize != o.packageSize || attrCount != o.attrCount ||
      stripped != o.stripped || billboard != o.billboard ||
      stFixed != o.stFixed)
    return false;
  for (u32 a = 0; a < attrCount && a < 3; ++a)
    if (attrs[a] != o.attrs[a]) return false;
  return true;
}

// ---------------------------------------------------------------------------
// StaPipCompactStream

StaPipCompactStream::StaPipCompactStream()
    : raw(nullptr),
      block(nullptr),
      partBox_(nullptr),
      coarseBox_(nullptr),
      wholeBox_(nullptr),
      lay(),
      bx(),
      wscale(kDefaultWeightScale),
      count(0),
      packages(0),
      parts(0),
      coarse(0),
      total(0),
      fullQw(0),
      lastQw(0),
      fullRun{0, 0, 0},
      lastRun{0, 0, 0},
      version(0),
      ekey{0, 0, 0, 0, 0, 0},
      spanRegistered(false) {}

StaPipCompactStream::~StaPipCompactStream() { release(); }

StaPipCompactStream::StaPipCompactStream(StaPipCompactStream&& o) noexcept
    : StaPipCompactStream() {
  moveFrom(o);
}

StaPipCompactStream& StaPipCompactStream::operator=(
    StaPipCompactStream&& o) noexcept {
  if (this != &o) {
    release();
    moveFrom(o);
  }
  return *this;
}

void StaPipCompactStream::moveFrom(StaPipCompactStream& o) {
  raw = o.raw;
  block = o.block;
  partBox_ = o.partBox_;
  coarseBox_ = o.coarseBox_;
  wholeBox_ = o.wholeBox_;
  lay = o.lay;
  bx = o.bx;
  wscale = o.wscale;
  count = o.count;
  packages = o.packages;
  parts = o.parts;
  coarse = o.coarse;
  total = o.total;
  fullQw = o.fullQw;
  lastQw = o.lastQw;
  for (u32 a = 0; a < 3; ++a) {
    fullRun[a] = o.fullRun[a];
    lastRun[a] = o.lastRun[a];
  }
  version = o.version;
  for (u32 i = 0; i < kEntryKeyWords; ++i) ekey[i] = o.ekey[i];
  spanRegistered = o.spanRegistered;
  o.spanRegistered = false;  // the span moved with the block
  o.raw = o.block = nullptr;
  o.partBox_ = o.coarseBox_ = o.wholeBox_ = nullptr;
  o.release();
}

bool StaPipCompactStream::allocate(const StaPipCompactLayout& layout,
                                   u32 vertexCount, const Compact::Box& box) {
  release();
  const u32 size = layout.packageSize;
  if (vertexCount == 0 || vertexCount > kMaxVertices || size < 3 ||
      size > StaPipCompactWriter::kMaxPackage || layout.attrCount == 0 ||
      layout.attrCount > 3 || !validBox(box))
    return false;
  if (layout.attrs[0] != Attr::Position && layout.attrs[0] != Attr::BbCenter)
    return false;

  const u32 pk = (vertexCount + size - 1) / size;
  const u32 lastN = vertexCount - (pk - 1) * size;
  u16 fr[3], lr[3];
  const u32 fq = packageLayout(layout, size, fr);
  const u32 lq = packageLayout(layout, lastN, lr);
  const u32 tot = (pk - 1) * fq + lq;
  const u32 partSize = size / 3;
  const u32 np = (vertexCount + partSize - 1) / partSize;
  const u32 ng = (np + kCoarseParts - 1) / kCoarseParts;
  // The boxes: a part's, a coarse group's, the whole stream's.
  const u32 boxFloats = (np + ng + 1) * 6;
  const u32 bytes = tot * 16 + boxFloats * static_cast<u32>(sizeof(float));

  raw = new (std::nothrow) u8[bytes + 15];
  if (raw == nullptr) return false;
  block = reinterpret_cast<u8*>((reinterpret_cast<uintptr_t>(raw) + 15) &
                                ~static_cast<uintptr_t>(15));
  partBox_ = reinterpret_cast<float*>(block + tot * 16);
  coarseBox_ = partBox_ + np * 6;
  wholeBox_ = coarseBox_ + ng * 6;
  std::memset(block, 0, bytes);
  lay = layout;
  bx = box;
  wscale = kDefaultWeightScale;
  count = vertexCount;
  packages = pk;
  parts = np;
  coarse = ng;
  total = tot;
  fullQw = fq;
  lastQw = lq;
  for (u32 a = 0; a < 3; ++a) {
    fullRun[a] = fr[a];
    lastRun[a] = lr[a];
  }

  for (u32 p = 0; p < packages; ++p) {
    const u32 n = packageVertices(p);
    for (u8 a = 0; a < lay.attrCount; ++a) {
      u32 header[4];
      Compact::runHeader(lay.attrs[a], n, lay.dest(a, n), header);
      std::memcpy(block + runOffset(p, a) * 16, header, sizeof(header));
    }
  }
  ++version;
  return true;
}

void StaPipCompactStream::setEntryKey(const u32 key[kEntryKeyWords]) {
  for (u32 i = 0; i < kEntryKeyWords; ++i) ekey[i] = key[i];
}

void StaPipCompactStream::setPackageWords(u32 p, const u32 entryCodes[2],
                                          const void* entryData,
                                          const u32 kickCodes[2]) {
  if (block == nullptr || p >= packages) return;
  constexpr u32 kStmodNormal = 0x05000000u;  // STMOD 0
  u32 words[4] = {kStmodNormal, 0u, entryCodes[0], entryCodes[1]};
  std::memcpy(entry(p), words, sizeof(words));
  std::memcpy(entry(p) + 16, entryData, 32);
  words[2] = kickCodes[0];
  words[3] = kickCodes[1];
  std::memcpy(kick(p), words, sizeof(words));
}

void StaPipCompactStream::clearPackageWords() {
  for (u32 p = 0; p < packages; ++p) {
    std::memset(entry(p), 0, kEntryQw * 16);
    std::memset(kick(p), 0, kKickQw * 16);
  }
  for (u32 i = 0; i < kEntryKeyWords; ++i) ekey[i] = 0;
}

void StaPipCompactStream::release() {
#if defined(_EE)
  // A block the renderer registered may be in a chain or a recorded frame:
  // unregistering waits for every reader before the memory goes.
  if (spanRegistered && block != nullptr)
    Vif1Queue::unregisterImmutableSpan(block);
#endif
  spanRegistered = false;
  for (u32 i = 0; i < kEntryKeyWords; ++i) ekey[i] = 0;
  delete[] raw;
  raw = block = nullptr;
  partBox_ = coarseBox_ = wholeBox_ = nullptr;
  lay = StaPipCompactLayout();
  bx = Compact::Box();
  wscale = kDefaultWeightScale;
  count = packages = parts = coarse = total = fullQw = lastQw = 0;
  for (u32 a = 0; a < 3; ++a) fullRun[a] = lastRun[a] = 0;
}

bool StaPipCompactStream::matches(const StaPipCompactLayout& layout,
                                  u32 vertexCount,
                                  const Compact::Box& box) const {
  return allocated() && count == vertexCount && lay == layout &&
         std::memcmp(&bx, &box, sizeof(bx)) == 0;
}

u32 StaPipCompactStream::packageVertices(u32 p) const {
  if (p >= packages) return 0;
  return p + 1 < packages ? lay.packageSize : count - p * lay.packageSize;
}

u32 StaPipCompactStream::packageQwords(u32 p) const {
  return p + 1 < packages ? fullQw : lastQw;
}

u32 StaPipCompactStream::runOffset(u32 p, u8 a) const {
  return p * fullQw + (p + 1 < packages ? fullRun[a] : lastRun[a]);
}

u8* StaPipCompactStream::run(u32 p, u8 a) {
  return block + (runOffset(p, a) + 1) * 16;
}

const u8* StaPipCompactStream::run(u32 p, u8 a) const {
  return block + (runOffset(p, a) + 1) * 16;
}

const u8* StaPipCompactStream::runHeader(u32 p, u8 a) const {
  return block + runOffset(p, a) * 16;
}

u32 StaPipCompactStream::runQwords(u32 p, u8 a) const {
  return Compact::runQwords(lay.attrs[a], packageVertices(p));
}

u8* StaPipCompactStream::entry(u32 p) { return block + p * fullQw * 16; }

u8* StaPipCompactStream::kick(u32 p) {
  return block + (p * fullQw + packageQwords(p) - kKickQw) * 16;
}

void StaPipCompactStream::finish() {
  if (block == nullptr) return;
  // Each part's box from the stored integers: the smallest box that holds
  // the positions as they decode (step x the grid index, exact).
  const u32 size = lay.packageSize, partSize = size / 3;
  const u32 element = Compact::elementBytes(lay.attrs[0]);
  u32 row[4];
  Compact::rowFor(bx, row);
  for (u32 part = 0; part < parts; ++part) {
    const u32 first = part * partSize;
    const u32 end = first + partSize < count ? first + partSize : count;
    s32 lo[3] = {32767, 32767, 32767}, hi[3] = {-32768, -32768, -32768};
    for (u32 i = first; i < end; ++i) {
      s16 q[3];
      std::memcpy(q, run(i / size, 0) + (i % size) * element, sizeof(q));
      for (u32 a = 0; a < 3; ++a) {
        if (q[a] < lo[a]) lo[a] = q[a];
        if (q[a] > hi[a]) hi[a] = q[a];
      }
    }
    float* box = partBox_ + part * 6;
    for (u32 a = 0; a < 3; ++a) {
      box[a] = bx.step * static_cast<float>(static_cast<s32>(row[a]) + lo[a]);
      box[3 + a] =
          bx.step * static_cast<float>(static_cast<s32>(row[a]) + hi[a]);
    }
  }
  // The coarse groups and the whole stream: unions of the part boxes (exact -
  // min and max pick one of the values), so a classifier needs no per-frame
  // union of its own.
  for (u32 g = 0; g < coarse; ++g) {
    const u32 first = g * kCoarseParts;
    const u32 end = first + kCoarseParts < parts ? first + kCoarseParts : parts;
    unionOf(partBox_ + first * 6, end - first, coarseBox_ + g * 6);
  }
  unionOf(coarseBox_, coarse, wholeBox_);
  ++version;
}

void StaPipCompactStream::partBox(u32 part, float mn[3], float mx[3]) const {
  for (u32 a = 0; a < 3; ++a) {
    mn[a] = part < parts ? partBox_[part * 6 + a] : 0.0f;
    mx[a] = part < parts ? partBox_[part * 6 + 3 + a] : 0.0f;
  }
}

// ---------------------------------------------------------------------------
// StaPipCompactWriter

StaPipCompactWriter::StaPipCompactWriter(StaPipCompactStream& stream)
    : s(&stream),
      package(0),
      k(0),
      n(0),
      runs{nullptr, nullptr, nullptr},
      posSlot(-1),
      stSlot(-1),
      colorSlot(-1),
      normalSlot(-1),
      paramsSlot(-1),
      envNormals(false),
      posBytes(0),
      clamped(0),
      scaleBits(0) {
  if (!stream.allocated()) return;
  const StaPipCompactLayout& l = stream.layout();
  posSlot = 0;
  posBytes = Compact::elementBytes(l.attrs[0]);
  stSlot = l.indexOf(Attr::St);
  colorSlot = slotOf(l, Attr::Color, Attr::BbColor);
  normalSlot = l.indexOf(Attr::Normal);
  if (normalSlot < 0) {
    normalSlot = l.indexOf(Attr::EnvNormal);
    envNormals = normalSlot >= 0;
  }
  paramsSlot = l.indexOf(Attr::BbParams);
  openPackage(0);
}

void StaPipCompactWriter::openPackage(u32 p) {
  package = p;
  k = 0;
  n = s->packageVertices(p);
  for (u8 a = 0; a < 3; ++a)
    runs[a] = n > 0 && a < s->layout().attrCount ? s->run(p, a) : nullptr;
  if (stSlot >= 0 && n > 0) {
    // S, T and Q a vertex: Q = 1 until stq() says otherwise.
    for (u32 i = 0; i < n; ++i) {
      stBuf[i * 3] = stBuf[i * 3 + 1] = 0.0f;
      stBuf[i * 3 + 2] = 1.0f;
    }
  }
}

void StaPipCompactWriter::closePackage() {
  if (stSlot < 0 || k == 0) return;
  bool saturated = false;
  const u32 bits =
      putStqs(runs[stSlot], stBuf, k, 3, s->layout().stFixed, &saturated);
  if (saturated) ++clamped;
  if (bits > scaleBits) scaleBits = bits;
}

void StaPipCompactWriter::position(const float p[3]) {
  if (k < n && posSlot >= 0) putPosition(runs[posSlot] + k * posBytes, p, s->box());
}

void StaPipCompactWriter::st(float u, float v) { stq(u, v, 1.0f); }

void StaPipCompactWriter::stq(float sCoord, float tCoord, float qCoord) {
  if (k >= n || stSlot < 0) return;
  stBuf[k * 3] = sCoord;
  stBuf[k * 3 + 1] = tCoord;
  stBuf[k * 3 + 2] = qCoord;
}

void StaPipCompactWriter::color(const float rgba[4]) {
  if (k < n && colorSlot >= 0) putColor(runs[colorSlot] + k * 4, rgba);
}

void StaPipCompactWriter::normal(const float nrm[3]) {
  if (k >= n || normalSlot < 0) return;
  if (envNormals)
    putEnvNormal(runs[normalSlot] + k * 4, nrm);
  else
    putNormal(runs[normalSlot] + k * 4, nrm);
}

void StaPipCompactWriter::bbParams(const float m[4]) {
  if (k < n && paramsSlot >= 0)
    putWeights(runs[paramsSlot] + k * 8, m, s->weightScale());
}

void StaPipCompactWriter::next() {
  if (n == 0) return;
  if (++k == n) {
    closePackage();
    openPackage(package + 1);
  }
}

void StaPipCompactWriter::end() {
  if (n > 0 && k > 0) closePackage();
  n = 0;
  k = 0;
  package = s->packageCount();
  s->finish();
}

// ---------------------------------------------------------------------------
// StaPipCompactView

StaPipCompactView::StaPipCompactView(const StaPipCompactStream& stream)
    : s(&stream),
      posSlot(-1),
      stSlot(-1),
      colorSlot(-1),
      normalSlot(-1),
      paramsSlot(-1),
      posBytes(0),
      row{0, 0, 0},
      step(stream.box().step) {
  if (!stream.allocated()) return;
  const StaPipCompactLayout& l = stream.layout();
  posSlot = 0;
  posBytes = Compact::elementBytes(l.attrs[0]);
  stSlot = l.indexOf(Attr::St);
  colorSlot = slotOf(l, Attr::Color, Attr::BbColor);
  normalSlot = slotOf(l, Attr::Normal, Attr::EnvNormal);
  paramsSlot = l.indexOf(Attr::BbParams);
  u32 words[4];
  Compact::rowFor(stream.box(), words);
  for (u32 a = 0; a < 3; ++a) row[a] = static_cast<s32>(words[a]);
}

u32 StaPipCompactView::count() const { return s->vertexCount(); }

const u8* StaPipCompactView::element(u32 i, s32 slot) const {
  if (slot < 0 || i >= s->vertexCount()) return nullptr;
  const StaPipCompactLayout& l = s->layout();
  const u32 size = l.packageSize;
  return s->run(i / size, static_cast<u8>(slot)) +
         (i % size) * Compact::elementBytes(l.attrs[slot]);
}

void StaPipCompactView::position(u32 i, float out[4]) const {
  out[0] = out[1] = out[2] = 0.0f;
  out[3] = 1.0f;
  const u8* e = element(i, posSlot);
  if (e == nullptr) return;
  // decodePositions' per-vertex arithmetic: the grid index times the step.
  s16 q[3];
  std::memcpy(q, e, sizeof(q));
  for (u32 a = 0; a < 3; ++a) out[a] = step * static_cast<float>(row[a] + q[a]);
}

void StaPipCompactView::st(u32 i, float out[2]) const {
  out[0] = out[1] = 0.0f;
  const u8* e = element(i, stSlot);
  if (e == nullptr) return;
  s16 q[3];
  std::memcpy(q, e, sizeof(q));
  // S/Q and T/Q. An ordinary package stores Q as its scale x 4096, a power of
  // two, so this is exact - the rebased u, v to 1/4096 at scale 1.
  const float den = q[2] != 0 ? static_cast<float>(q[2]) : 4096.0f;
  for (u32 c = 0; c < 2; ++c) out[c] = static_cast<float>(q[c]) / den;
}

void StaPipCompactView::stq(u32 i, float out[3]) const {
  out[0] = out[1] = out[2] = 0.0f;
  const u8* e = element(i, stSlot);
  if (e == nullptr) return;
  s16 q[3];
  std::memcpy(q, e, sizeof(q));
  for (u32 c = 0; c < 3; ++c) out[c] = static_cast<float>(q[c]) * (1.0f / 4096);
}

void StaPipCompactView::color(u32 i, float out[4]) const {
  out[0] = out[1] = out[2] = out[3] = 0.0f;
  const u8* e = element(i, colorSlot);
  if (e == nullptr) return;
  for (u32 c = 0; c < 4; ++c) out[c] = static_cast<float>(e[c]);
}

void StaPipCompactView::normal(u32 i, float out[3]) const {
  out[0] = out[1] = out[2] = 0.0f;
  const u8* e = element(i, normalSlot);
  if (e == nullptr) return;
  s8 q[3];
  std::memcpy(q, e, sizeof(q));
  for (u32 c = 0; c < 3; ++c) out[c] = static_cast<float>(q[c]) / 127.0f;
}

void StaPipCompactView::bbParams(u32 i, float out[4]) const {
  out[0] = out[1] = out[2] = out[3] = 0.0f;
  const u8* e = element(i, paramsSlot);
  if (e == nullptr) return;
  s16 q[4];
  std::memcpy(q, e, sizeof(q));
  for (u32 c = 0; c < 4; ++c) out[c] = static_cast<float>(q[c]) / s->weightScale();
}

void StaPipCompactView::positions(u32 first, u32 n, float* out4) const {
  if (posSlot < 0 || first >= s->vertexCount()) return;
  if (n > s->vertexCount() - first) n = s->vertexCount() - first;
  const u32 size = s->layout().packageSize;
  while (n > 0) {
    const u32 p = first / size, k = first % size;
    const u32 avail = s->packageVertices(p) - k;
    const u32 m = n < avail ? n : avail;
    Compact::decodePositions(s->run(p, 0) + k * posBytes, posBytes, m, s->box(),
                             out4);
    out4 += m * 4;
    first += m;
    n -= m;
  }
}

// ---------------------------------------------------------------------------
// Packing, boxes, the scene step

bool Compact::packArrays(const StaPipCompactLayout& layout,
                         const StaPipCompactSource& source, const Box& box,
                         StaPipCompactStream& out, u32* stClamped,
                         u32* stScaleBits) {
  if (!out.matches(layout, source.count, box) &&
      !out.allocate(layout, source.count, box))
    return false;
  if (layout.indexOf(Attr::BbParams) >= 0 && source.sts != nullptr) {
    float largest = 0.0f;
    for (u32 i = 0; i < source.count * 4; ++i) {
      const float w = std::fabs(source.sts[i]);
      if (w > largest) largest = w;
    }
    out.setWeightScale(weightScaleFor(largest));
  }
  u32 clamped = 0, worstBits = 0;
  const u32 size = layout.packageSize;
  for (u32 p = 0; p < out.packageCount(); ++p) {
    const u32 first = p * size, n = out.packageVertices(p);
    for (u8 a = 0; a < layout.attrCount; ++a) {
      u8* run = out.run(p, a);
      const Attr attr = layout.attrs[a];
      switch (attr) {
        case Attr::Position:
        case Attr::BbCenter:
          if (source.positions != nullptr)
            quantizePositions(source.positions + first * 4, n, box, run,
                              elementBytes(attr));
          break;
        case Attr::St:
          if (source.sts != nullptr) {
            bool saturated = false;
            const u32 bits = putStqs(run, source.sts + first * 4, n, 4,
                                     layout.stFixed, &saturated);
            if (saturated) ++clamped;
            if (bits > worstBits) worstBits = bits;
          }
          break;
        case Attr::EnvNormal:
          for (u32 i = 0; source.sts != nullptr && i < n; ++i)
            putEnvNormal(run + i * 4, source.sts + (first + i) * 4);
          break;
        case Attr::Normal:
          for (u32 i = 0; source.normals != nullptr && i < n; ++i)
            putNormal(run + i * 4, source.normals + (first + i) * 4);
          break;
        case Attr::Color:
        case Attr::BbColor:
          for (u32 i = 0; source.colors != nullptr && i < n; ++i)
            putColor(run + i * 4, source.colors + (first + i) * 4);
          break;
        case Attr::BbParams:
          for (u32 i = 0; source.sts != nullptr && i < n; ++i)
            putWeights(run + i * 8, source.sts + (first + i) * 4,
                       out.weightScale());
          break;
      }
    }
  }
  out.finish();
  if (stClamped != nullptr) *stClamped = clamped;
  if (stScaleBits != nullptr) *stScaleBits = worstBits;
  return true;
}

bool Compact::packRun(Attr a, const StaPipCompactSource& source,
                      const u32* index, u32 first, u32 n, const Box& box,
                      float weightScale, u8* run, u8 stFixed) {
  const u32 bytes = elementBytes(a);
  std::memset(run, 0, runQwords(a, n) * 16);
  bool clamped = false;
  auto at = [&](u32 i) { return index != nullptr ? index[i] : first + i; };
  switch (a) {
    case Attr::Position:
    case Attr::BbCenter:
      if (source.positions == nullptr) break;
      if (index == nullptr) {
        // Contiguous: the batch path, as packArrays (bit-identical to the
        // per-element one - Compact::selfTest).
        quantizePositions(source.positions + first * 4, n, box, run, bytes);
      } else {
        for (u32 i = 0; i < n; ++i)
          putPosition(run + i * bytes, source.positions + at(i) * 4, box);
      }
      break;
    case Attr::St: {
      if (source.sts == nullptr) break;
      if (index == nullptr) {
        putStqs(run, source.sts + first * 4, n, 4, stFixed, &clamped);
        break;
      }
      float stq[3 * StaPipCompactWriter::kMaxPackage];
      if (n > StaPipCompactWriter::kMaxPackage) n = StaPipCompactWriter::kMaxPackage;
      for (u32 i = 0; i < n; ++i)
        for (u32 c = 0; c < 3; ++c) stq[i * 3 + c] = source.sts[at(i) * 4 + c];
      putStqs(run, stq, n, 3, stFixed, &clamped);
      break;
    }
    case Attr::EnvNormal:
      for (u32 i = 0; source.sts != nullptr && i < n; ++i)
        putEnvNormal(run + i * 4, source.sts + at(i) * 4);
      break;
    case Attr::Normal:
      for (u32 i = 0; source.normals != nullptr && i < n; ++i)
        putNormal(run + i * 4, source.normals + at(i) * 4);
      break;
    case Attr::Color:
    case Attr::BbColor:
      for (u32 i = 0; source.colors != nullptr && i < n; ++i)
        putColor(run + i * 4, source.colors + at(i) * 4);
      break;
    case Attr::BbParams:
      for (u32 i = 0; source.sts != nullptr && i < n; ++i)
        putWeights(run + i * 8, source.sts + at(i) * 4, weightScale);
      break;
  }
  return clamped;
}

void Compact::copyRun(const StaPipCompactStream& stream, u8 a,
                      const u32* index, u32 first, u32 n, u8* run) {
  const StaPipCompactLayout& l = stream.layout();
  const u32 bytes = elementBytes(l.attrs[a]);
  const u32 size = l.packageSize;
  std::memset(run, 0, runQwords(l.attrs[a], n) * 16);
  if (!stream.allocated() || a >= l.attrCount) return;
  for (u32 i = 0; i < n; ++i) {
    const u32 v = index != nullptr ? index[i] : first + i;
    if (v >= stream.vertexCount()) continue;
    std::memcpy(run + i * bytes, stream.run(v / size, a) + (v % size) * bytes,
                bytes);
  }
}

Compact::Box Compact::boxFor(const float* positions, u32 count,
                             bool worldSpace, bool* coarse) {
  float mn[3], mx[3];
  boundsOf(positions, count, mn, mx);
  if (worldSpace) return worldBox(mn, mx, sceneStep(), coarse);
  if (coarse != nullptr) *coarse = false;
  return modelBox(mn, mx);
}

void Compact::setSceneStep(float step) {
  if (!(step > 0.0f && step < 1e30f)) return;
  float s = 1.0f / 1024;
  while (s < step) s *= 2.0f;
  gSceneStep = s;
}

float Compact::sceneStep() { return gSceneStep; }

// ---------------------------------------------------------------------------
// The EE half: StaPipBag and the classifier's boxes.

#if defined(_EE)

StaPipCompactLayout StaPipCompactLayout::forBag(const StaPipBag& bag,
                                                u32 packageSize) {
  if (bag.billboard != nullptr) return forBillboards(packageSize);
  const bool texture = bag.texture != nullptr;
  const bool env = texture && bag.texture->coordinatesAreNormals;
  const bool lit = bag.lighting != nullptr;
  const bool single = bag.color != nullptr && bag.color->single != nullptr;
  StaPipCompactLayout l =
      forClass(texture, env, lit, single, packageSize, bag.stripped);
  // A texture that does not REPEAT on an axis keeps that coordinate as it is
  // (StaPipCompactLayout::stFixed) - a lamp's pool, a projected shadow.
  if (texture && !env && bag.texture->texture != nullptr) {
    const texwrap_t* w = bag.texture->texture->getWrapSettings();
    if (w != nullptr) {
      if (w->horizontal != WRAP_REPEAT) l.stFixed |= 1;
      if (w->vertical != WRAP_REPEAT) l.stFixed |= 2;
    }
  }
  return l;
}

bool Compact::packBag(const StaPipBag& bag, u32 packageSize, const Box& box,
                      StaPipCompactStream& out) {
  return packArrays(StaPipCompactLayout::forBag(bag, packageSize),
                    sourceOf(bag), box, out);
}

Compact::Box Compact::boxForBag(const StaPipBag& bag) {
  bool world = true;
  if (bag.info != nullptr && bag.info->model != nullptr)
    for (u32 i = 0; i < 16; ++i)
      if (bag.info->model->data[i] != (i % 5 == 0 ? 1.0f : 0.0f)) world = false;
  const float* positions = bag.vertices != nullptr ? bag.vertices->xyzw : nullptr;
  bool coarse = false;
  const Box box = boxFor(positions, bag.count, world, &coarse);
  if (coarse) {
    // Once a bag, for the first few: a scene whose cells outgrow the step says
    // so without a line every frame.
    static const void* logged[16];
    static u32 loggedCount = 0;
    bool seen = false;
    for (u32 i = 0; i < loggedCount; ++i) seen = seen || logged[i] == &bag;
    if (!seen && loggedCount < 16) {
      logged[loggedCount++] = &bag;
      float mn[3], mx[3], extent = 0.0f;
      boundsOf(positions, bag.count, mn, mx);
      for (u32 a = 0; a < 3; ++a)
        if (mx[a] - mn[a] > extent) extent = mx[a] - mn[a];
      std::printf("[COMPACT] coarse box bag=%p extent=%.3f\n",
                  static_cast<const void*>(&bag), static_cast<double>(extent));
    }
  }
  return box;
}

#endif  // _EE

}  // namespace Tyra
