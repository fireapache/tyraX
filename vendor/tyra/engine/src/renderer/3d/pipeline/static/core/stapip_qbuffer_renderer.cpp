#include "renderer/core/paths/path1/frame_submission.hpp"
#include "renderer/3d/pipeline/static/core/stapip_spot_bounds.hpp"
/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Sandro Sobczyński <sandro.sobczynski@gmail.com>
*/

#include "math/math.hpp"
#include "math/mmi.hpp"
#include "renderer/core/paths/path1/vif1_queue.hpp"
#include <kernel.h>  // Modified by TyraX: FlushCache for the VIF1 queue

// Modified by TyraX: PipelineZTest_TestOnly branch in sendObjectData;
// per-mesh object-space spot light (flashlight) upload for the color VU1
// programs + EE clipper; alpha-test AFAIL fixed to ATEST_KEEP_ALL (see
// sendObjectData) so cutout textures stop stamping the z buffer.

#include <math.h>
#include <string.h>
#include <dma_tags.h>
#include <algorithm>
#include <new>  // Modified by TyraX: std::nothrow
#include <stdio.h>  // Modified by TyraX: snprintf, the cache fallback line
#include "debug/hardware_trace.hpp"
#include "renderer/3d/pipeline/static/core/stapip_qbuffer_renderer.hpp"
#include "renderer/3d/pipeline/static/core/stapip_baked_body.hpp"
#include "renderer/3d/pipeline/static/core/programs/stapip_vu1_shared_defines.h"
#include "renderer/core/gs/renderer_core_depth.hpp"
#include "packet2/packet2_tyra_utils.hpp"
#include "renderer/3d/pipeline/static/core/stapip_vu_tap.hpp"
#include "renderer/3d/pipeline/static/core/stapip_vu1_experiments.hpp"
#include "renderer/3d/pipeline/static/core/compact/stapip_compact_tyra.hpp"
#if TYRA_MEMORY_MANAGER
#include "memory/manager.hpp"
#endif

// #define TYRA_QBUFF_RENDERER_VERBOSE_LOG 1

#ifdef TYRA_QBUFF_RENDERER_VERBOSE_LOG
#define Verbose(...) TyraDebug::writeLines("VRB: ", ##__VA_ARGS__, "\n")
#else
#define Verbose(...) ((void)0)
#endif

namespace Tyra {

#if TYRA_MEMORY_MANAGER
// Modified by TyraX: a static-pipeline cache that did without - the heap, or a
// refusing budget, had no room - says so in the log, once a second per kind
// of fallback and per mesh (Memory::Manager::noteFallback; nothing is
// allocated). The bags carry no name, so a mesh is its vertex count: every
// fighter's body is one asset, not one per fighter.
static void noteCacheFallback(const char* what, u32 vertices) {
  char asset[32];
  snprintf(asset, sizeof asset, "bag-%lu-vertices",
           static_cast<unsigned long>(vertices));
  const uintptr_t key = reinterpret_cast<uintptr_t>(what) ^
                        (static_cast<uintptr_t>(vertices) << 12);
  Memory::Manager::instance().noteFallback(
      what, asset, -1, reinterpret_cast<const void*>(key));
}
#define TYRA_CACHE_FALLBACK(what, vertices) noteCacheFallback(what, vertices)
#else
#define TYRA_CACHE_FALLBACK(what, vertices) ((void)0)
#endif

static inline u32 readTelemetryTicks() {
  u32 result;
  asm volatile("mfc0 %0, $9" : "=r"(result));
  return result;
}

/**
 * VU1 = 1000 vert
 *
 * Quadbuffering:
 * 2 main buffers = 1000 / 2 = 500 vert
 * 2 kick buffers =  500 / 2 = 250 vert
 *
 * Vert data:
 * Pos + Normal + ST + Color = 4
 * = 4 * 48 = 192
 *
 * Other data:
 * mvp matrix, light matrix, tags = 14
 * 20 light vectors, light intesities = 25
 * = 14 + 25 = 39
 *
 * All data:
 * = 192 + 39 = 231
 *
 */

const u16 StaPipQBufferRenderer::buffersCount = 32;
namespace {
// Modified by TyraX: 57 packet qwords before the transient REF payloads were
// made inline, plus 4 MVP, 3 light-matrix, 3 direction and 1 colour qwords.
// Modified by TyraX: + 3 - the compact vertex format's bag setup.
constexpr u16 kObjectDataPacketSize = 71;
constexpr u16 kSubmissionBatchSize = 4;
constexpr u16 kQBufferCommandCapacity = 32;  // keep with buffersCount
constexpr u16 kWorstBagPacketSize =
    4 * kQBufferCommandCapacity + kObjectDataPacketSize;

#if TYRA_STAPIP_PACKET_PROFILE
inline u8 telemetryProducer(const StaPipTelemetry* telemetry) {
  return telemetry != nullptr && telemetry->producer < StaPipProducerCount
             ? telemetry->producer
             : static_cast<u8>(StaPipProducerMixed);
}

bool isTexturedProgram(const StaPipProgramName name) {
  return name == StaPipCullTextureDirLights ||
         name == StaPipAsIsTextureDirLights ||
         name == StaPipCullTextureColor || name == StaPipAsIsTextureColor ||
         name == StaPipClipTextureDirLights ||
         name == StaPipClipTextureColor || name == StaPipCullTextureEnv ||
         name == StaPipAsIsTextureEnv || name == StaPipClipTextureEnv ||
         name == StaPipBillboardTexture;
}

void recordGsPacket(StaPipTelemetry* telemetry, StaPipVU1Program* program,
                    const u32 inputVertices, const bool emitState) {
  if (telemetry == nullptr || program == nullptr) return;
  StaPipPacketCounters& out = telemetry->packet[telemetryProducer(telemetry)];
  const bool textured = isTexturedProgram(program->getName());
  const bool stateReuse =
      (program->getName() == StaPipCullTextureColor ||
       program->getName() == StaPipCullTextureDirLights ||
       program->getName() == StaPipAsIsTextureColor ||
       program->getName() == StaPipAsIsTextureDirLights) &&
      !emitState;
  const u32 gsVertices = program->getGsVertexCount(inputVertices);
  const u32 registers = program->getReglistCount();
  // StoreTyraGifTags*Alpha emits three A+D state pairs for an untextured
  // package and four for a textured one, followed by the primitive giftag.
  out.gifTags += stateReuse ? 1 : (textured ? 5 : 4);
  out.adWrites += stateReuse ? 0 : (textured ? 4 : 3);
  out.gsPayloadQwords += (stateReuse ? 1 : (textured ? 9 : 7)) +
                         (gsVertices * registers + 1) / 2;
  if (stateReuse) ++out.gsStateReuses;
  ++out.xgkicks;
}

struct VifCounterState {
  u32 pending = 0;
  bool malformed = false;
};

void countVifWord(StaPipPacketCounters& out, VifCounterState& state,
                  const u32 word) {
  ++out.vifWords;
  if (state.pending != 0) {
    --state.pending;
    return;
  }
  const u32 cmd = (word >> 24) & 0xFFu;
  const u32 num = (word >> 16) & 0xFFu;
  if (cmd == 0x00) {
    ++out.vifNops;
    return;
  }
  if ((cmd & 0x60u) == 0x60u) {
    const u32 vn = (cmd >> 2) & 3u;
    const u32 vl = cmd & 3u;
    const u32 count = num == 0 ? 256u : num;
    state.pending = (count * (vn + 1u) * (32u >> vl) + 31u) / 32u;
    ++out.vifUnpack;
    return;
  }
  switch (cmd) {
    case 0x01: ++out.vifStcycl; return;
    case 0x05:  // STMOD
    case 0x07:  // MARK
      return;
    case 0x10: ++out.vifFlushe; return;
    case 0x11: ++out.vifFlush; return;
    case 0x13: ++out.vifFlusha; return;
    case 0x14: ++out.vifMscal; return;
    case 0x15: ++out.vifMscalf; return;
    case 0x17: ++out.vifMscnt; return;
    case 0x20: state.pending = 1; return;  // STMASK
    case 0x30: state.pending = 4; ++out.vifStrow; return;
    case 0x31: state.pending = 4; ++out.vifStcol; return;
    default: state.malformed = true; return;
  }
}

/** Count the exact source chain handed to VIF1. The decoder deliberately
 * understands the same closed VIF command set as StaPipVifHash: an unknown
 * command latches malformed instead of letting every later payload word pose
 * as a command. */
void recordVifChain(StaPipTelemetry* telemetry, const u8 producer,
                    const void* base, const u32 qwc) {
  if (telemetry == nullptr || base == nullptr || qwc == 0) return;
  StaPipPacketCounters& out = telemetry->packet[
      producer < StaPipProducerCount ? producer : StaPipProducerMixed];
  out.chainQwords += qwc;
  const qword_t* src = reinterpret_cast<const qword_t*>(base);
  VifCounterState state;
  u32 i = 0;
  while (i < qwc && !state.malformed) {
    const dma_tag_t* tag = reinterpret_cast<const dma_tag_t*>(&src[i]);
    const u32 tagQwc = static_cast<u32>(tag->QWC);
    const u32 id = static_cast<u32>(tag->ID);
    if (id < 8) ++out.dmaTags[id];
    const u32* tte = reinterpret_cast<const u32*>(&src[i]) + 2;
    countVifWord(out, state, tte[0]);
    countVifWord(out, state, tte[1]);

    const qword_t* payload = nullptr;
    bool stop = false;
    if (id == 1 || id == 7) {  // CNT / END
      if (i + 1 + tagQwc > qwc) {
        state.malformed = true;
        break;
      }
      payload = &src[i + 1];
      out.inlinePayloadQwords += tagQwc;
      i += 1 + tagQwc;
      stop = id == 7;
    } else if (id == 0 || id == 3 || id == 4) {  // REFE / REF / REFS
      const u32 addr = static_cast<u32>(tag->ADDR);
      payload = reinterpret_cast<const qword_t*>(addr);
      out.refPayloadQwords += tagQwc;
      if ((addr & 127u) == 0) ++out.refsAligned128;
      else ++out.refsUnaligned128;
      ++i;
      stop = id == 0;
    } else {
      state.malformed = true;  // NEXT/CALL/RET are not emitted here.
      break;
    }
    for (u32 q = 0; q < tagQwc && !state.malformed; ++q) {
      const u32* words = reinterpret_cast<const u32*>(&payload[q]);
      for (u32 w = 0; w < 4; ++w) countVifWord(out, state, words[w]);
    }
    if (stop) break;
  }
  if (state.malformed || state.pending != 0) ++out.malformedChains;
}
#endif

// Modified by TyraX: append quadwords that were already built as part of a DMA
// chain.
//
// Every tag this pipeline writes is POSITION-INDEPENDENT: a CNT tag counts the
// quadwords that follow it, a REF tag names an absolute address OUTSIDE the
// packet, and with TTE the VIF codes ride in the tag's own upper half. So a
// finished, quadword-aligned run of them means exactly the same thing at any
// quadword-aligned position in any packet, and copying one is indistinguishable
// from having built it there. Nothing here may be used for a tag that refers to
// its own position (NEXT, CALL) - the pipeline writes none, and
// docs/vu1-and-dma-cache-cost.md records why it must not start.
inline void appendChainQwords(packet2_t* packet, const qword_t* src,
                              u32 qwCount) {
  memcpy(packet->next, src, qwCount * 16);
  packet2_advance_next(packet, qwCount * 16);
}

// Modified by TyraX: the object-data fast path (sendObjectData).
//
// Measured on a physical PS2 (Motor District garage day, 75 bags a frame):
// sendObjectData cost ~14 us a bag, and most of it was not the data. A
// section holding ONE qword - the FLUSHE tag, the ALPHA unpack - took ~1.4 us,
// because every packet2 open/close runs the DMA-tag bitfield writes, a
// back-patched VIFcode and a handful of asserts, and every packet2_add_float
// reloads packet->next through memory. Cache misses on the packet lines were
// ~20% (16 qwords: 1.0 us into cold lines, 0.17 us into hot ones).
//
// So each uniform block is now its header qword plus whole-qword copies. The
// header - CNT tag, STCYCL, UNPACK V4_32 - is produced ONCE per (address,
// length) by packet2 itself into a scratch packet and remembered, which makes
// the fast path's bytes identical to the old path's by construction; the
// VIF-hash gate (docs/baked-stream-acceptance-gate.md) compared the two.
struct UnpackHeader {
  u32 key;  // VU address << 8 | qwords; 0 = unused
  qword_t header;
};
constexpr int kUnpackHeaders = 32;
UnpackHeader g_unpackHeaders[kUnpackHeaders];
qword_t g_flusheHeader;
bool g_flusheHeaderReady = false;

packet2_t* headerScratch() {
  static packet2_t* scratch =
      packet2_create(32, P2_TYPE_NORMAL, P2_MODE_CHAIN, true);
  packet2_reset(scratch, false);
  return scratch;
}

const qword_t& unpackHeader(u32 addr, u32 qwords) {
  const u32 key = (addr << 8) | qwords;
  for (int i = 0; i < kUnpackHeaders; ++i) {
    if (g_unpackHeaders[i].key == key) return g_unpackHeaders[i].header;
    if (g_unpackHeaders[i].key == 0) {
      packet2_t* s = headerScratch();
      packet2_utils_vu_open_unpack(s, addr, false);
      for (u32 q = 0; q < qwords; ++q) packet2_add_2x_s64(s, 0, 0);
      packet2_utils_vu_close_unpack(s);
      g_unpackHeaders[i].header = s->base[0];
      g_unpackHeaders[i].key = key;
      return g_unpackHeaders[i].header;
    }
  }
  TYRA_ASSERT(false, "Object-data unpack header cache is full: raise "
                     "kUnpackHeaders");
  return g_unpackHeaders[0].header;
}

// The leading CNT tag of every uniform upload - a bag's object data, the EE
// clipper's other uniform forms, an instanced copy's MVP: FLUSHE + STMOD 0,
// no payload (Compact::kUniformHeadCodes). Modified by TyraX: the STMOD 0 was
// a NOP; a freshly built positions-only package ends with the SDK's kick and
// leaves offset MODE standing, which would add ROW to the uniform's bits
// (docs/compact-vertex-format.md, "The bag setup and the run header").
const qword_t& flusheHeader() {
  if (!g_flusheHeaderReady) {
    packet2_t* s = headerScratch();
    packet2_chain_open_cnt(s, 0, 0, 0);
    packet2_vif_flushe(s, 0);
    packet2_vif_stmod(s, 0, 0);
    packet2_chain_close_tag(s);
    g_flusheHeader = s->base[0];
    TYRA_ASSERT(g_flusheHeader.sw[2] == Compact::kUniformHeadCodes[0] &&
                    g_flusheHeader.sw[3] == Compact::kUniformHeadCodes[1],
                "The uniform head must be FLUSHE + STMOD 0");
    g_flusheHeaderReady = true;
  }
  return g_flusheHeader;
}

// Modified by TyraX: the compact vertex format (docs/compact-vertex-format.md).
// No compact chain leaves VIF1 in the offset MODE of a position run (ruling
// P12): a stream's entry and kick quadwords carry STMOD 0
// (StaPipCompactStream::setPackageWords), so does a program's count / GIF-tag
// CNT, and so does every packet's END tag - this one: END with STMOD 0 and a
// NOP as its VIFcodes (packet2_utils_vu_add_end_tag with the STMOD in place of
// a NOP).
inline void addEndTag(packet2_t* packet) {
  packet2_chain_open_end(packet, 0, 0);
  packet2_vif_stmod(packet, 0, 0);
  packet2_vif_nop(packet, 0);
  packet2_chain_close_tag(packet);
}

// The attributes, the strip flag and the fixed ST axes of a layout, as one
// word (a cache key): a texture whose wrap changes packs again.
u32 layoutKeyOf(const StaPipCompactLayout& l) {
  u32 k = (l.attrCount & 3u) | (l.stripped ? 4u : 0u) | (l.billboard ? 8u : 0u);
  for (u32 a = 0; a < l.attrCount && a < 3; ++a)
    k |= (static_cast<u32>(l.attrs[a]) + 1u) << (4 + a * 4);
  return k | (static_cast<u32>(l.stFixed & 3u) << 16);
}

// A bag in world space: its model matrix is exactly the identity.
bool identityModel(const StaPipBag* bag) {
  if (bag->info == nullptr || bag->info->model == nullptr) return true;
  const float* m = bag->info->model->data;
  for (u32 i = 0; i < 16; ++i)
    if (m[i] != (i % 5 == 0 ? 1.0F : 0.0F)) return false;
  return true;
}

// An optional content stamp: null contributes nothing to a key.
inline u32 stampWord(const u32* p) { return p != nullptr ? *p : 0u; }

u32 floatBitsOf(float f) {
  u32 b;
  memcpy(&b, &f, sizeof(b));
  return b;
}

// The single colour as four integer lanes (ruling P8): RGB clamped to 0..255
// and truncated, alpha one higher - what the float programs' add of vf00 and
// FixColor gave the GS.
void singleColorLanes(const Color& c, u32 out[4]) {
  for (u32 k = 0; k < 4; ++k) {
    float v = c.rgba[k] + (k == 3 ? 1.0F : 0.0F);
    v = v < 0.0F ? 0.0F : (v > 255.0F ? 255.0F : v);
    out[k] = static_cast<u32>(v);
  }
}

// The env (matcap) basis the TCE programs read: transposed, the ST scale
// folded in; the compact form takes the 1/127 of the int8 normal in its x and
// y lanes and 127 in X.w - the renormalising paths' rsqrt numerator.
void envBasis(const Vec4& r, const Vec4& u, bool compact, float e[12]) {
  const float k = compact ? 1.0F / 127.0F : 1.0F;
  const float in[12] = {r.x * 0.5F, u.x * -0.5F, 0.0F, compact ? 127.0F : 0.0F,
                        r.y * 0.5F, u.y * -0.5F, 0.0F, 0.0F,
                        r.z * 0.5F, u.z * -0.5F, 1.0F, 0.5F};
  for (u32 i = 0; i < 12; ++i) e[i] = in[i];
  for (u32 row = 0; row < 3; ++row) {
    e[row * 4] *= k;
    e[row * 4 + 1] *= k;
  }
}

// The spot light's VU constants in GRID units (ruling P9): the compact
// programs see positions as grid indices (object units / step), so the
// position divides by the step and the two terms that multiply a squared
// distance by step^2 - exact with a power-of-two step.
void spotGridConstants(const StaPipClipperSpot& spot, float step, float s[12]) {
  const float inv = 1.0F / step, step2 = step * step;
  const float out[12] = {spot.position.x * inv,  spot.position.y * inv,
                         spot.position.z * inv,  spot.invRange2 * step2,
                         spot.direction.x,       spot.direction.y,
                         spot.direction.z,       spot.cosCut2,
                         spot.enabled ? spot.color[0] : 0.0F,
                         spot.enabled ? spot.color[1] : 0.0F,
                         spot.enabled ? spot.color[2] : 0.0F,
                         spot.invSoft * step2};
  for (u32 i = 0; i < 12; ++i) s[i] = out[i];
}

// MVP x D(step, 0): the step on the diagonal, no translation - the first three
// columns scaled, exactly (a power of two).
void mvpDequant(const M4x4& mvp, float step, M4x4* out) {
  *out = mvp;
  for (u32 i = 0; i < 12; ++i) out->data[i] *= step;
}

bool isAsIsProgram(const StaPipProgramName name) {
  return name == StaPipAsIsColor || name == StaPipAsIsDirLights ||
         name == StaPipAsIsTextureDirLights || name == StaPipAsIsTextureColor ||
         name == StaPipAsIsTextureEnv;
}

inline void emitQword(packet2_t* packet, const qword_t& q) {
  *packet->next = q;
  packet->next += 1;
}

// A DMA REF to `qwords` of VIF stream at `data` (16-byte aligned, stable
// until every chain naming it has run). Its VIFcodes are two NOPs, so the
// stream VIF1 sees is exactly the referenced data.
inline void emitRef(packet2_t* packet, const void* data, u32 qwords) {
  qword_t* q = packet->next;
  q->dw[0] = static_cast<u64>(qwords & 0xFFFFU) | (static_cast<u64>(3) << 28) |
             (static_cast<u64>(reinterpret_cast<u32>(data) & 0x0FFFFFFFU)
              << 32);
  q->dw[1] = 0;
  packet->next = q + 1;
}

// A chain fragment of CNT tags + inline data, rewritten as the VIF stream it
// delivers: each tag's DMA half becomes two VIF NOPs, its VIFcodes and data
// stay. Returns the quadwords written (the same count), 0 if it is not a
// plain CNT run.
u32 chainToVifStream(const qword_t* src, u32 qw, qword_t* dst) {
  u32 i = 0;
  while (i < qw) {
    const u64 tag = src[i].dw[0];
    const u32 id = static_cast<u32>((tag >> 28) & 7U);
    const u32 n = static_cast<u32>(tag & 0xFFFFU);
    if (id != 1 || i + 1 + n > qw) return 0;  // only CNT, fully inside
    dst[i].dw[0] = 0;
    dst[i].dw[1] = src[i].dw[1];
    for (u32 k = 1; k <= n; ++k) dst[i + k] = src[i + k];
    i += 1 + n;
  }
  return qw;
}

// One uniform unpack to the absolute VU1 region: header + `qwords` copied
// whole from `src` (16-byte aligned).
inline void emitUnpack(packet2_t* packet, u32 addr, const void* src,
                       u32 qwords) {
  qword_t* q = packet->next;
  q[0] = unpackHeader(addr, qwords);
  const qword_t* s = static_cast<const qword_t*>(src);
  for (u32 i = 0; i < qwords; ++i) q[1 + i] = s[i];
  packet->next = q + 1 + qwords;
}
}  // namespace

#if TYRA_STAPIP_RETAINED_COMMANDS

StaPipRetainedCommands::StaPipRetainedCommands() {
  std::fill(indexBuckets, indexBuckets + kBucketCount, -1);
}

u32 StaPipRetainedCommands::getBucket(const void* vertices,
                                      u32 maxVertCount) const {
  u32 hash = static_cast<u32>(reinterpret_cast<u32>(vertices)) * 0x9E3779B1U;
  hash ^= maxVertCount + 0x85EBCA6BU + (hash << 6) + (hash >> 2);
  hash ^= hash >> 16;
  return hash & (kBucketCount - 1);
}

bool StaPipRetainedCommands::keyMatches(const StaPipRetainedEntry& a,
                                        const StaPipRetainedEntry& b) {
  return a.vertices == b.vertices && a.sts == b.sts && a.colors == b.colors &&
         a.normals == b.normals && a.program == b.program &&
         a.count == b.count && a.maxVertCount == b.maxVertCount &&
         a.bboxVersion == b.bboxVersion && a.primKey == b.primKey &&
         a.depthScaleBits == b.depthScaleBits &&
         a.singleColor == b.singleColor && a.stripped == b.stripped;
}

void StaPipRetainedCommands::rebuildIndex() {
  std::fill(indexBuckets, indexBuckets + kBucketCount, -1);
  for (u32 i = 0; i < storage.size(); ++i) {
    auto& item = storage[i];
    const u32 bucket = getBucket(item.vertices, item.maxVertCount);
    item.nextInBucket = indexBuckets[bucket];
    indexBuckets[bucket] = static_cast<int>(i);
  }
}

void StaPipRetainedCommands::onFrameEnd() {
  evictionsThisFrame = 0;
  bool expired = false;
  for (auto& item : storage) {
    if (item.framesLeftToDestroy > 0) {
      item.framesLeftToDestroy--;
      if (item.framesLeftToDestroy <= 0) expired = true;
    }
  }
  if (!expired) return;

  u32 freed = 0;
  const auto newEnd = std::remove_if(
      storage.begin(), storage.end(), [&](const StaPipRetainedEntry& item) {
        if (item.framesLeftToDestroy > 0) return false;
        freed += static_cast<u32>(item.packages) * kMaxBlockQw;
        return true;
      });
  storage.erase(newEnd, storage.end());
  usedQwords -= freed < usedQwords ? freed : usedQwords;
  rebuildIndex();
}

void StaPipRetainedCommands::clear() {
  storage.clear();
  std::fill(indexBuckets, indexBuckets + kBucketCount, -1);
  usedQwords = 0;
  evictionsThisFrame = 0;
}

// Modified by TyraX: the cap is a hard EE-RAM budget, so it has to be spent on
// the bags that are actually being drawn. Without this, the first bags to ask
// filled it and every later one was refused for the whole 250-frame lifetime -
// measured on the Motor District garage as ~25-40% of packages rebuilding every
// frame while the cache sat pinned at the cap holding geometry from a pose the
// camera had left. An entry this frame already touched is never a victim (its
// countdown is still at kLifetimeFrames), and the per-frame bound is what stops
// a scene whose working set genuinely does not fit from evicting and
// re-capturing the same bags forever - that would be strictly worse than not
// retaining them at all.
bool StaPipRetainedCommands::evictFor(u32 wanted) {
  while (usedQwords + wanted > kMaxQwords) {
    if (evictionsThisFrame >= kMaxEvictionsPerFrame) return false;
    int victim = -1;
    int oldest = kLifetimeFrames;  // strictly older than "touched this frame"
    for (u32 i = 0; i < storage.size(); ++i) {
      if (storage[i].framesLeftToDestroy < oldest) {
        oldest = storage[i].framesLeftToDestroy;
        victim = static_cast<int>(i);
      }
    }
    if (victim < 0) return false;
    const u32 freed =
        static_cast<u32>(storage[victim].packages) * kMaxBlockQw;
    usedQwords -= freed < usedQwords ? freed : usedQwords;
    storage.erase(storage.begin() + victim);
    rebuildIndex();
    ++evictionsThisFrame;
  }
  return true;
}

StaPipRetainedEntry* StaPipRetainedCommands::acquire(
    const StaPipRetainedEntry& key) {
  const u32 bucket = getBucket(key.vertices, key.maxVertCount);
  int index = indexBuckets[bucket];
  while (index >= 0) {
    auto& item = storage[index];
    if (item.vertices == key.vertices &&
        item.maxVertCount == key.maxVertCount) {
      item.framesLeftToDestroy = kLifetimeFrames;
      if (keyMatches(item, key)) return &item;
      // Something the block encodes moved - an LOD tier, a material, a
      // reallocated stream, a pipeline switch. Only the captured blocks are
      // thrown away; the storage is reused, or re-sized when the package count
      // moved with it.
      if (item.packages != key.packages) {
        const u32 had = static_cast<u32>(item.packages) * kMaxBlockQw;
        const u32 wants = static_cast<u32>(key.packages) * kMaxBlockQw;
        // A resize that does not fit is simply refused: the entry keeps its
        // old storage with every block invalidated, so this bag rebuilds until
        // room appears. Evicting here would have to consider evicting the very
        // entry being resized, and this case - a bag whose PACKAGE COUNT moved
        // while the cache is already full - is rare enough not to be worth it.
        if (key.packages == 0 || usedQwords - had + wants > kMaxQwords)
          return nullptr;
        // Modified by TyraX: and so is one the heap has no memory for (nothrow:
        // a full heap is not a reason to stop the game) - both blocks are
        // taken before the old ones are let go.
        std::unique_ptr<qword_t[]> data;
        std::unique_ptr<u8[]> ready;
        {
#if TYRA_MEMORY_MANAGER
          // Modified by TyraX: a resized entry's blocks are the Renderer
          // budget's, as a new entry's are - unscoped they would be System's,
          // which halts instead of answering null (docs/memory-manager.md).
          Memory::MemScope memoryScope(Memory::Budget::Renderer);
#endif
          data.reset(new (std::nothrow) qword_t[wants]);
          ready.reset(new (std::nothrow) u8[key.packages]);
        }
        if (!data || !ready) {
          TYRA_CACHE_FALLBACK("retained resize: ordinary route", key.count);
          return nullptr;
        }
        item.data = std::move(data);
        item.ready = std::move(ready);
        item.packages = key.packages;
        usedQwords = usedQwords - had + wants;
      }
      const StaPipRetainedEntry& k = key;
      item.sts = k.sts;
      item.colors = k.colors;
      item.normals = k.normals;
      item.program = k.program;
      item.count = k.count;
      item.bboxVersion = k.bboxVersion;
      item.primKey = k.primKey;
      item.depthScaleBits = k.depthScaleBits;
      item.singleColor = k.singleColor;
      item.stripped = k.stripped;
      item.blockQw = 0;
      memset(item.ready.get(), 0, item.packages);
      return &item;
    }
    index = item.nextInBucket;
  }

  const u32 wanted = static_cast<u32>(key.packages) * kMaxBlockQw;
  if (key.packages == 0 || wanted > kMaxQwords) return nullptr;
  if (usedQwords + wanted > kMaxQwords && !evictFor(wanted)) return nullptr;

  // Modified by TyraX: the table's room and the entry's blocks are taken
  // nothrow, before anything is counted - memory short, the bag draws the
  // ordinary way this frame and asks again the next (docs/retained-static-
  // commands.md, "When memory is short").
  if (storage.size() == storage.capacity())
    storage.reserve(storage.size() < 16 ? 16 : storage.size() * 2);
  std::unique_ptr<qword_t[]> data;
  std::unique_ptr<u8[]> ready;
  {
#if TYRA_MEMORY_MANAGER
    // Modified by TyraX: the retained entries are render data, the Renderer
    // budget's - which refuses when full (docs/memory-manager.md).
    Memory::MemScope memoryScope(Memory::Budget::Renderer);
#endif
    data.reset(new (std::nothrow) qword_t[wanted]);
    ready.reset(new (std::nothrow) u8[key.packages]);
  }
  if (!data || !ready) {
    TYRA_CACHE_FALLBACK("retained entry: ordinary route", key.count);
    return nullptr;
  }

  StaPipRetainedEntry entry;
  entry.vertices = key.vertices;
  entry.sts = key.sts;
  entry.colors = key.colors;
  entry.normals = key.normals;
  entry.program = key.program;
  entry.count = key.count;
  entry.maxVertCount = key.maxVertCount;
  entry.bboxVersion = key.bboxVersion;
  entry.primKey = key.primKey;
  entry.depthScaleBits = key.depthScaleBits;
  entry.singleColor = key.singleColor;
  entry.stripped = key.stripped;
  entry.packages = key.packages;
  entry.blockQw = 0;
  entry.data = std::move(data);
  entry.ready = std::move(ready);
  memset(entry.ready.get(), 0, key.packages);
  entry.framesLeftToDestroy = kLifetimeFrames;
  entry.nextInBucket = indexBuckets[bucket];

  storage.push_back(std::move(entry));
  indexBuckets[bucket] = static_cast<int>(storage.size() - 1);
  usedQwords += wanted;
  return &storage.back();
}

#endif  // TYRA_STAPIP_RETAINED_COMMANDS

// Modified by TyraX: the compact stream cache - docs/baked-vif-stream.md,
// docs/compact-vertex-format.md ("Routes").
#if TYRA_STAPIP_BAKED_STREAM

StaPipBakedStreams::StaPipBakedStreams() {
  std::fill(indexBuckets, indexBuckets + kBucketCount, -1);
}

StaPipBakedStreams::~StaPipBakedStreams() {
  // Teardown is a real compatibility fence; no unsubmitted reader may survive.
  // The streams are freed by their own destructors once their spans are
  // collected (retired by clear(), and every one in the graveyard).
  Vif1Queue::drain();
  clear();
  for (auto& slot : graveyard)
    for (auto& item : slot) Vif1Queue::collectRetiredImmutableSpan(item.data());
}

u32 StaPipBakedStreams::getBucket(const void* vertices,
                                  u32 maxVertCount) const {
  const u32 v = reinterpret_cast<u32>(vertices);
  return ((v >> 4) ^ (v >> 12) ^ maxVertCount) & (kBucketCount - 1);
}

bool StaPipBakedStreams::keyMatches(const StaPipBakedEntry& a,
                                    const StaPipBakedEntry& b) {
  return a.vertices == b.vertices && a.sts == b.sts && a.colors == b.colors &&
         a.normals == b.normals && a.count == b.count &&
         a.maxVertCount == b.maxVertCount && a.bboxVersion == b.bboxVersion &&
         a.contentVersion == b.contentVersion && a.layoutKey == b.layoutKey;
}

#if TYRA_STAPIP_BAKED_SAMPLE_VERIFY
void StaPipBakedStreams::pickVerifyTarget() {
  if (storage.empty()) {
    verifyTarget = nullptr;
    return;
  }
  verifyTarget = storage[verifyCursor % storage.size()].get();
  ++verifyCursor;
}
#endif

void StaPipBakedStreams::rebuildIndex() {
  std::fill(indexBuckets, indexBuckets + kBucketCount, -1);
  for (u32 i = 0; i < storage.size(); ++i) {
    auto& item = *storage[i];
    const u32 bucket = getBucket(item.vertices, item.maxVertCount);
    item.nextInBucket = indexBuckets[bucket];
    indexBuckets[bucket] = static_cast<int>(i);
  }
}

// A stream is READ BY DMA, so freeing one is not a local decision: a packet the
// pipeline submitted moments ago, or a recorded frame, may still name it. Its
// span is RETIRED (no new REF may name it, the leases it has last until their
// frames complete) and the memory waits in the graveyard until
// collectRetiredImmutableSpan says no bank reads it.
void StaPipBakedStreams::bury(StaPipCompactStream& stream) {
  if (!stream.allocated()) return;
  const u32 qw = stream.qwords();
  Vif1Queue::retireImmutableSpan(stream.data());
#if TYRA_STAPIP_BAKED_POISON
  // The adversarial mode: overwrite a retired stream NOW instead of letting
  // the graveyard hide it. Anything still naming it tears visibly.
  Vif1Queue::unregisterImmutableSpan(stream.data());
  memset(const_cast<void*>(stream.data()), 0xDE, qw * 16);
#endif
  graveyard[graveyardWrite].push_back(std::move(stream));
  graveyardQwords += qw;
}

void StaPipBakedStreams::retire(StaPipBakedEntry& item) {
  const u32 qw = item.stream.qwords();
  usedQwords -= qw < usedQwords ? qw : usedQwords;
  bury(item.stream);
  item.complete = 0;
  retireSpare(item);
}

void StaPipBakedStreams::retireSpare(StaPipBakedEntry& item) {
  if (!item.spare.allocated()) return;
  const u32 qw = item.spare.qwords();
  usedQwords -= qw < usedQwords ? qw : usedQwords;
  bury(item.spare);
}

void StaPipBakedStreams::onFrameEnd() {
  evictionsThisFrame = 0;
  ++frameNow;
  graveyardWrite ^= 1;
  collectGraveyard(graveyardWrite);

  bool expired = false;
  for (auto& item : storage) {
#if TYRA_STAPIP_PREPARED_STATIC
    if (item->scenePinned) continue;
#endif
    if (item->framesLeftToDestroy > 0) {
      item->framesLeftToDestroy--;
      if (item->framesLeftToDestroy <= 0) expired = true;
    }
  }
  if (expired) {
    for (auto& item : storage)
      if (item->framesLeftToDestroy <= 0) {
        retire(*item);
        noteDestroyed(item.get());  // stamps
      }
    const auto newEnd =
        std::remove_if(storage.begin(), storage.end(),
                       [](const std::unique_ptr<StaPipBakedEntry>& item) {
                         return item->framesLeftToDestroy <= 0;
                       });
    storage.erase(newEnd, storage.end());
    rebuildIndex();
  }

  // A budget lowered below what the cache holds is walked down here, least
  // recently used first, bounded like any eviction.
  while (usedQwords > budgetQwords &&
         evictionsThisFrame < kMaxEvictionsPerFrame && evictOldest())
    ++evictionsThisFrame;

#if TYRA_STAPIP_BAKED_SAMPLE_VERIFY
  pickVerifyTarget();
#endif
}

void StaPipBakedStreams::clear() {
  ++eraseGen;  // stamps
  for (auto& item : storage) retire(*item);
  storage.clear();
  std::fill(indexBuckets, indexBuckets + kBucketCount, -1);
  usedQwords = 0;
  evictionsThisFrame = 0;
#if TYRA_STAPIP_BAKED_SAMPLE_VERIFY
  verifyTarget = nullptr;
#endif
}

void StaPipBakedStreams::collectGraveyard(u8 slot) {
  auto& retired = graveyard[slot];
  retired.erase(std::remove_if(retired.begin(), retired.end(),
      [this](const StaPipCompactStream& item) {
        if (!Vif1Queue::collectRetiredImmutableSpan(item.data())) return false;
        const u32 qw = item.qwords();
        graveyardQwords -= qw < graveyardQwords ? qw : graveyardQwords;
        return true;  // freed by the erase
      }), retired.end());
}

void StaPipBakedStreams::releaseAll() {
  clear();
  collectGraveyard(0);
  collectGraveyard(1);
}

bool StaPipBakedStreams::evictOldest() {
  int victim = -1;
  int oldest = kLifetimeFrames;  // never an entry this frame has touched
  for (u32 i = 0; i < storage.size(); ++i) {
#if TYRA_STAPIP_PREPARED_STATIC
    if (storage[i]->scenePinned) continue;
#endif
    if (storage[i]->framesLeftToDestroy < oldest) {
      oldest = storage[i]->framesLeftToDestroy;
      victim = static_cast<int>(i);
    }
  }
  if (victim < 0) return false;
  noteDestroyed(storage[victim].get());  // stamps
  retire(*storage[victim]);
#if TYRA_STAPIP_BAKED_SAMPLE_VERIFY
  if (verifyTarget == storage[victim].get()) verifyTarget = nullptr;
#endif
  storage.erase(storage.begin() + victim);
  rebuildIndex();
  return true;
}

bool StaPipBakedStreams::evictFor(u32 wanted) {
  while (usedQwords + wanted > budgetQwords) {
    if (evictionsThisFrame >= kMaxEvictionsPerFrame) return false;
    if (!evictOldest()) return false;
    ++evictionsThisFrame;
  }
  return true;
}

bool StaPipBakedStreams::reserve(u32 qwords) {
  if (qwords > budgetQwords) return false;
  if (usedQwords + qwords > budgetQwords && !evictFor(qwords)) return false;
  usedQwords += qwords;
  return true;
}

// Pack into the entry - its first stream, or (a payload that moved) its spare,
// which then swaps with the current one. Ping-pong, the policy of
// stapip_arena_pair.hpp: the spare is written in place only when a pack of the
// same shape fits it AND no bank still reads it; otherwise a fresh stream, and
// the busy spare to the graveyard.
bool StaPipBakedStreams::pack(StaPipBakedEntry& item,
                              const StaPipCompactLayout& layout,
                              const StaPipCompactSource& source,
                              const Compact::Box& box, u8 keepSlots) {
  const bool hasCurrent = item.stream.allocated();
  StaPipCompactStream* target = &item.stream;
  StaPipCompactStream fresh;
  if (hasCurrent) {
    // A repack: never the current stream - a chain may be reading it.
    // Modified by TyraX: the spare is reused for the same SHAPE - its block
    // takes a moved pose's new box in place (StaPipCompactStream::rebox), so
    // a body re-posed every frame allocates nothing.
    StaPipCompactStream& spare = item.spare;
    const bool hasSpare = spare.allocated();
    const bool shapeFits = hasSpare && spare.fits(layout, source.count);
    const bool collected =
        shapeFits && Vif1Queue::collectRetiredImmutableSpan(spare.data());
    const StaPipSpareUse use = chooseSpareUse(
        hasSpare, hasSpare ? spare.qwords() : 0,
        shapeFits ? spare.qwords() : spare.qwords() + 1, collected);
    if (use == StaPipSpareUse::Reuse) {
      target = &spare;
      ++spareReuses;
    } else {
      if (use == StaPipSpareUse::Replace && hasSpare) {
        if (shapeFits) ++spareBusy;
        retireSpare(item);  // gives its qwords back; freed once unread
      }
      target = &fresh;
      ++freshArenas;
    }
  }

  // Charge the budget for a block this pack allocates; a reused spare is
  // charged already.
  const bool allocates = !target->fits(layout, source.count);
  if (allocates) {
    // allocate() is the only sizing authority: allocate, then charge.
    bool allocated;
    {
#if TYRA_MEMORY_MANAGER
      // A stream is the BakedStreams budget's (docs/memory-manager.md) - and
      // ONLY the stream: that budget refuses instead of halting, so nothing
      // that allocates with a throwing new may run in its scope (reserve's
      // evictions bury streams in the graveyard, a growing vector).
      Memory::MemScope memoryScope(Memory::Budget::BakedStreams);
#endif
      allocated = target->allocate(layout, source.count, box);
    }
    if (!allocated) {
      TYRA_CACHE_FALLBACK("compact stream: packed into the copy pool",
                          source.count);
      return false;
    }
    if (!reserve(target->qwords())) {
      target->release();
      return false;
    }
  }
  if (!Compact::packArrays(layout, source, box, *target, nullptr, nullptr,
                           hasCurrent ? &item.stream : nullptr,
                           hasCurrent ? keepSlots : 0)) {
    if (allocates) {
      unreserve(target->qwords());
      target->release();
    }
    return false;
  }
  // An immutable span: a recorded frame borrows it instead of copying it. A
  // registry with no room (or a build without the frame recorder) answers
  // false, and the stream is copied like any other REF target - correct,
  // slower; its memory still goes through the graveyard.
  Vif1Queue::registerImmutableSpan(target->data(), target->qwords());
  if (hasCurrent) {
    // The swap: the current stream may still be named by a packet in flight
    // or a recorded frame, so it is RETIRED in the span registry and kept as
    // the spare, still charged, for the next repack to write once unread.
#if TYRA_STAPIP_BAKED_POISON
    Vif1Queue::unregisterImmutableSpan(item.stream.data());
    memset(const_cast<void*>(item.stream.data()), 0xDE,
           item.stream.qwords() * 16);
#else
    Vif1Queue::retireImmutableSpan(item.stream.data());
#endif
    StaPipCompactStream previous = std::move(item.stream);
    // `target` - the reused spare or a fresh stream - becomes current, and
    // the old current the spare.
    item.stream = std::move(*target);
    item.spare = std::move(previous);
    ++patchCount;
  }
  item.complete = 1;
  item.serial = ++serialNext;  // stamps: the REF target moved
  ++packedBags;
  countBuild();
  return true;
}

StaPipBakedEntry* StaPipBakedStreams::acquire(const StaPipBakedEntry& key) {
  const u32 bucket = getBucket(key.vertices, key.maxVertCount);
  // A SECOND PASS over one vertex array (a terrain chunk's layers, the AO and
  // emissive passes - their own ST and colour arrays over one position array)
  // gets its own entry, up to kVariantsPerArray of them; told apart by its
  // streams and layout.
  const u32 kVariantsPerArray = 6;
  u32 variants = 0;
  int index = indexBuckets[bucket];
  while (index >= 0) {
    auto& item = *storage[index];
    if (item.vertices == key.vertices &&
        item.maxVertCount == key.maxVertCount) {
      ++variants;
      const bool samePass = item.layoutKey == key.layoutKey &&
                            item.sts == key.sts && item.colors == key.colors &&
                            item.normals == key.normals;
      if (!samePass && variants < kVariantsPerArray) {
        index = item.nextInBucket;
        continue;
      }
      item.framesLeftToDestroy = kLifetimeFrames;
      if (keyMatches(item, key)) return &item;
      const bool payloadMoved = item.bboxVersion != key.bboxVersion ||
                                item.contentVersion != key.contentVersion;
      // Churn damping: only a PAYLOAD move counts.
      if (item.churning) {
        if (key.bboxVersion == item.seenBBoxVersion &&
            key.contentVersion == item.seenContentVersion) {
          if (++item.settled >= kSettleFrames) item.churning = 0;
        } else {
          item.seenBBoxVersion = key.bboxVersion;
          item.seenContentVersion = key.contentVersion;
          item.settled = 0;
        }
      } else if (payloadMoved && item.complete &&
                 (item.count != key.count || item.layoutKey != key.layoutKey) &&
                 frameNow - item.lastRebuildFrame <= kChurnGapFrames) {
        // Only a shape that keeps moving churns (Added by TyraX): a payload
        // move of the same shape is one repack into the spare per pose.
        item.churning = 1;
        item.seenBBoxVersion = key.bboxVersion;
        item.seenContentVersion = key.contentVersion;
        item.settled = 0;
      }
      if (item.churning) {
        ++churnSkips;
        return nullptr;  // packed into the copy pool as it is drawn
      }
      if (payloadMoved) item.lastRebuildFrame = frameNow;
      MissReason reason = MissCountOrSize;
      if (item.bboxVersion != key.bboxVersion)
        reason = MissBBoxVersion;
      else if (item.contentVersion != key.contentVersion)
        reason = MissContentVersion;
      else if (item.sts != key.sts || item.colors != key.colors ||
               item.normals != key.normals)
        reason = MissStreams;
      else if (item.layoutKey != key.layoutKey)
        reason = MissProgram;
      countMiss(reason);
      noteLoud(reason, key.count, key.packages);
      // A payload that moved on a complete entry of the same shape is
      // repacked into the spare (ping-pong); anything else drops both streams.
      const bool sameShape = item.complete && item.count == key.count &&
                             item.layoutKey == key.layoutKey;
      if (!sameShape) {
        item.serial = ++serialNext;
        retire(item);
      }
      item.pendingPatch = sameShape ? 1 : 0;
      item.complete = sameShape ? item.complete : 0;
      item.vertices = key.vertices;
      item.sts = key.sts;
      item.colors = key.colors;
      item.normals = key.normals;
      item.count = key.count;
      item.bboxVersion = key.bboxVersion;
      item.contentVersion = key.contentVersion;
      item.layoutKey = key.layoutKey;
      item.packages = key.packages;
      return key.packages == 0 ? nullptr : &item;
    }
    index = item.nextInBucket;
  }

  if (key.packages == 0) return nullptr;
  countMiss(MissNewEntry);
  noteLoud(MissNewEntry, key.count, key.packages);

  if (storage.size() == storage.capacity())
    storage.reserve(storage.size() < 32 ? 32 : storage.size() * 2);
#if TYRA_MEMORY_MANAGER
  Memory::MemScope memoryScope(Memory::Budget::BakedStreams);
#endif
  std::unique_ptr<StaPipBakedEntry> entry(new (std::nothrow) StaPipBakedEntry());
  if (!entry) {
    TYRA_CACHE_FALLBACK("baked entry: packed into the copy pool", key.count);
    return nullptr;
  }
  entry->vertices = key.vertices;
  entry->sts = key.sts;
  entry->colors = key.colors;
  entry->normals = key.normals;
  entry->count = key.count;
  entry->maxVertCount = key.maxVertCount;
  entry->bboxVersion = key.bboxVersion;
  entry->contentVersion = key.contentVersion;
  entry->layoutKey = key.layoutKey;
  entry->packages = key.packages;
  entry->complete = 0;
  entry->framesLeftToDestroy = kLifetimeFrames;
  entry->nextInBucket = indexBuckets[bucket];
  entry->serial = ++serialNext;

  storage.push_back(std::move(entry));
  indexBuckets[bucket] = static_cast<int>(storage.size() - 1);
  return storage.back().get();
}

#endif  // TYRA_STAPIP_BAKED_STREAM

StaPipQBufferRenderer::StaPipQBufferRenderer() {
  currentBufferIndex = 0;
  nextBufferIndex = 0;
  context = 0;
  lastProgramName = StaPipUndefinedProgram;

  // Modified by TyraX: four bounded small direct bags can share one native
  // packet. Each retains its own worst-case uniforms and up to 15 command
  // groups; the extra headroom keeps packet construction fail-closed.
  packetSize = kSubmissionBatchSize * kWorstBagPacketSize;
  programsPacket = nullptr;
  billboardProgramsPacket = nullptr;
}

void StaPipQBufferRenderer::allocateOnUse() {
  rendererCore->texture.setMutationBarrier(textureMutationBarrier, this);
  staticDataPacket = packet2_create(3, P2_TYPE_NORMAL, P2_MODE_CHAIN, true);
  // Modified by TyraX: +6 qwords for the spot light unpack, +16 for the
  // VU1 clipping constants + plane table.
  // Modified by TyraX: 42 -> 48 - the per-mesh ALPHA qword and the env
  // (matcap) camera basis added two unpack blocks to sendObjectData.
  // Modified by TyraX: 48 -> 52 - the billboard basis unpack (2 qwords
  // + headers).
  // Four inline lighting qwords replace the former REF payload.
  packets = new packet2_t*[kPacketCount];
#if TYRA_STAPIP_PROBE_UNCACHED_CHAIN
  // Probe B (stapip_probes.hpp). 1 = P2_TYPE_UNCACHED, 2 = UNCACHED_ACCL.
  // packet2_create asserts qwords % 4 == 0 for either uncached type, so say so
  // here rather than discovering it as a trap on the console.
  static_assert((4 * (4 * 32 + 68)) % 4 == 0,
                "Probe B: an uncached packet2 needs a qword count that is a "
                "multiple of 4 (ps2sdk packet2_create). Keep packetSize so.");
  const enum Packet2Type probeType = TYRA_STAPIP_PROBE_UNCACHED_CHAIN == 2
                                         ? P2_TYPE_UNCACHED_ACCL
                                         : P2_TYPE_UNCACHED;
  for (u16 i = 0; i < kPacketCount; i++)
    packets[i] = packet2_create(packetSize, probeType, P2_MODE_CHAIN, true);
  probePacketUsesPool = false;
  // VERIFY THE ALLOCATION TOOK EFFECT BEFORE TRUSTING A SINGLE NUMBER FROM
  // THIS ARM. A packet that quietly came back in the normal segment would
  // measure "no flush needed" while the DMA read stale cache lines - the
  // failure mode is intermittent wrong geometry, not a crash. The segment is
  // the top nibble of the base pointer: 0x3 accelerated, 0x2 uncached.
  for (u16 i = 0; i < kPacketCount; i++) {
    TYRA_ASSERT(packets[i] != nullptr, "Probe B: packet2_create refused an "
                "uncached packet. Check packetSize % 4.");
    const u32 seg = reinterpret_cast<u32>(packets[i]->base) >> 28;
    TYRA_ASSERT(seg == (TYRA_STAPIP_PROBE_UNCACHED_CHAIN == 2 ? 0x3U : 0x2U),
                "Probe B: packet ", i, " is NOT in the uncached segment. "
                "base nibble: ", seg);
    TYRA_LOG("PROBEB: packet ", i, " base segment 0x", seg,
             " (1=uncached, 2=uncached-accelerated arm)");
  }
#else
  for (u16 i = 0; i < kPacketCount; i++)
    packets[i] =
        packet2_create(packetSize, P2_TYPE_NORMAL, P2_MODE_CHAIN, true);
#endif
  for (u16 i = 0; i < kPacketCount; i++) packetOrigin[i] = packets[i]->base;

  buffers = new StaPipQBuffer*[buffersCount];
  for (u16 i = 0; i < buffersCount; i++) {
    buffers[i] = new StaPipQBuffer();
  }
  // Modified by TyraX: these buffers hold no package size yet, so the cached
  // value that setMaxVertCount early-outs on must not claim they do. 0 is not
  // a legal size, so the next bag always propagates. See setMaxVertCount.
  maxVertCount = 0;

  dBufferPrograms = new StaPipVU1Program*[buffersCount];

  sendStaticData();
}

void StaPipQBufferRenderer::deallocateOnUse() {
  beforeTextureMutation();
  frameWaitVif();
  rendererCore->texture.clearMutationBarrier(this);
  submissionBatchScope = false;
  submissionBatchCandidate = false;
#if TYRA_STAPIP_RETAINED_COMMANDS
  // Modified by TyraX: a scene teardown frees every stream the retained REF
  // tags name. Nothing may survive it - the pointer compare would only catch a
  // reallocation that came back at a DIFFERENT address.
  retainedCurrent = nullptr;
  retained.clear();
  clipBlockQw[0] = clipBlockQw[1] = 0;
  clipBlockVifQw[0] = clipBlockVifQw[1] = 0;
#endif
#if TYRA_STAPIP_BAKED_STREAM
  // Same reasoning, and one more: a stream is named by DMA REF tags, so
  // nothing may outlive the pipeline that submitted them.
  bakedStream = nullptr;
  sourceEntry = nullptr;
  baked.clear();
  if (captureScratch) packet2_free(captureScratch);
  captureScratch = nullptr;
#endif
  packet2_free(staticDataPacket);
  for (u16 i = 0; i < kPacketCount; i++) {
    packets[i]->base = packets[i]->next = packetOrigin[i];  // the own buffer
    packet2_free(packets[i]);
  }
  delete[] packets;

  for (u16 i = 0; i < buffersCount; i++) delete buffers[i];
  delete[] buffers;

  delete[] dBufferPrograms;
}

// Added by TyraX: a scene load's release - see the header. The pipeline stays
// in use: only what it caches for the old scene goes.
void StaPipQBufferRenderer::releaseCaches() {
  // Nothing of the old scene stays queued: a pending packet names its arrays
  // and its baked arenas.
  flushPendingPacket();
#if TYRA_STAPIP_RETAINED_COMMANDS
  // The retained REF tags name the old scene's arrays, which the load frees -
  // the same reasoning as deallocateOnUse().
  retainedCurrent = nullptr;
  retained.clear();
  clipBlockQw[0] = clipBlockQw[1] = 0;
  clipBlockVifQw[0] = clipBlockVifQw[1] = 0;
#endif
#if TYRA_STAPIP_BAKED_STREAM
  bakedStream = nullptr;
  sourceEntry = nullptr;
  if (baked.holdsArenas()) {
    // A real fence first: every submitted packet and recorded frame that may
    // name a baked arena completes, and the frame recorder gives its bank
    // leases back - so the graveyard frees every arena now, before the next
    // level allocates, instead of two frames into it.
    Vif1Queue::drain();
    baked.releaseAll();
  }
#endif
}

StaPipQBufferRenderer::~StaPipQBufferRenderer() {
  if (programsPacket) packet2_free(programsPacket);
  if (billboardProgramsPacket) packet2_free(billboardProgramsPacket);
}

void StaPipQBufferRenderer::init(RendererCore* t_core, prim_t* t_prim,
                                 lod_t* t_lod) {
  path1 = t_core->getPath1();
  clipper.init(t_core->getSettings());
  rendererCore = t_core;
  prim = t_prim;
  lod = t_lod;

  // Modified by TyraX: the clip-space planes the VU1 clip programs cut
  // against - the same ones the EE clipper uses (see PlanesClipAlgorithm::init;
  // clipMargin must be set before setRenderer, like the EE path requires).
  clipNearZ =
      t_core->getSettings().getNear() - (-PlanesClipAlgorithm::clipMargin);
  clipFarZ = -t_core->getSettings().getFar();
#if TYRA_STAPIP_RETAINED_COMMANDS
  clipBlockQw[0] = clipBlockQw[1] = 0;
  // Modified by TyraX: their inputs were just written.
  clipBlockVifQw[0] = clipBlockVifQw[1] = 0;
#endif

  dma_channel_initialize(DMA_CHANNEL_VIF1, nullptr, 0);
#if TYRA_VIF1_QUEUE
  Vif1Queue::init();  // Modified by TyraX: see vif1_queue.hpp
#endif

  setProgramsCache();

  reinitVU1();

  TYRA_LOG("StaPipQBufferRenderer initialized");
}

void StaPipQBufferRenderer::reinitVU1() {
  flushPendingPacket();
  uploadPrograms();
  setDoubleBuffer();
}

namespace {
// Inverse of an affine model matrix (rotation * scale + translation).
// M4x4 is column-major: data[0..3] = X basis, [4..7] = Y, [8..11] = Z,
// [12..14] = translation. inv3x3 comes out row-major.
void invertAffine(const float* m, float* inv3x3, float* invT) {
  const float a = m[0], b = m[4], c = m[8];
  const float d = m[1], e = m[5], f = m[9];
  const float g = m[2], h = m[6], i = m[10];
  float det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  if (det > -1e-12F && det < 1e-12F) det = 1e-12F;
  const float id = 1.0F / det;
  inv3x3[0] = (e * i - f * h) * id;
  inv3x3[1] = (c * h - b * i) * id;
  inv3x3[2] = (b * f - c * e) * id;
  inv3x3[3] = (f * g - d * i) * id;
  inv3x3[4] = (a * i - c * g) * id;
  inv3x3[5] = (c * d - a * f) * id;
  inv3x3[6] = (d * h - e * g) * id;
  inv3x3[7] = (b * g - a * h) * id;
  inv3x3[8] = (a * e - b * d) * id;
  invT[0] = -(inv3x3[0] * m[12] + inv3x3[1] * m[13] + inv3x3[2] * m[14]);
  invT[1] = -(inv3x3[3] * m[12] + inv3x3[4] * m[13] + inv3x3[5] * m[14]);
  invT[2] = -(inv3x3[6] * m[12] + inv3x3[7] * m[13] + inv3x3[8] * m[14]);
}

// Builds the object-space spot light for this mesh: transforms the world
// light through the inverse model matrix, expresses the range in object
// units via the (assumed near-uniform) mesh scale, and precomputes the
// constants the cull VU1 programs and the EE clipper both consume.
StaPipClipperSpot buildSpotForBagInverse(const RendererCoreSpotLight& spot,
                                         const float* inv, const float* invT);

StaPipClipperSpot buildSpotForBag(const RendererCoreSpotLight& spot,
                                  const M4x4* model) {
  if (!spot.enabled) {
    StaPipClipperSpot out;
    out.enabled = false;
    return out;
  }
  float inv[9], invT[3];
  invertAffine(model->data, inv, invT);
  return buildSpotForBagInverse(spot, inv, invT);
}

// The same, with the model's inverse already known (a stamp keeps it).
StaPipClipperSpot buildSpotForBagInverse(const RendererCoreSpotLight& spot,
                                         const float* inv, const float* invT) {
  StaPipClipperSpot out;
  out.enabled = spot.enabled;
  if (!spot.enabled) return out;

  out.position.x = inv[0] * spot.position.x + inv[1] * spot.position.y +
                   inv[2] * spot.position.z + invT[0];
  out.position.y = inv[3] * spot.position.x + inv[4] * spot.position.y +
                   inv[5] * spot.position.z + invT[1];
  out.position.z = inv[6] * spot.position.x + inv[7] * spot.position.y +
                   inv[8] * spot.position.z + invT[2];
  out.position.w = 1.0F;

  Vec4 dir;
  dir.x = inv[0] * spot.direction.x + inv[1] * spot.direction.y +
          inv[2] * spot.direction.z;
  dir.y = inv[3] * spot.direction.x + inv[4] * spot.direction.y +
          inv[5] * spot.direction.z;
  dir.z = inv[6] * spot.direction.x + inv[7] * spot.direction.y +
          inv[8] * spot.direction.z;
  float dirLen2 = dir.x * dir.x + dir.y * dir.y + dir.z * dir.z;
  if (dirLen2 < 1e-10F) dirLen2 = 1.0F;

  // A world direction through the inverse scales by 1/s (uniform scale s),
  // so |dir|^2 = 1/s^2 - reuse it to express the range in object units.
  const float objRange2 = spot.range * spot.range * dirLen2;

  const float invDirLen = 1.0F / Math::sqrtNonNegative(dirLen2);
  out.direction.x = dir.x * invDirLen;
  out.direction.y = dir.y * invDirLen;
  out.direction.z = dir.z * invDirLen;
  out.direction.w = 0.0F;

  // Modified by TyraX: clamped at 0 (ruling P13) - the compact spot-lit loop
  // keeps only FixColor's ceiling. The stamp replay builds through here too.
  out.color[0] = spot.color.r > 0.0F ? spot.color.r : 0.0F;
  out.color[1] = spot.color.g > 0.0F ? spot.color.g : 0.0F;
  out.color[2] = spot.color.b > 0.0F ? spot.color.b : 0.0F;
  out.invRange2 = 1.0F / objRange2;

  // Point (omni) light through the SAME spot constants: zero direction makes
  // the axial term t = max(0, d.dir) collapse to 0, so the cone factor
  // becomes (0 - cosCut2*dist2)*invSoft = dist2 * invSoft with cosCut2 = -1.
  // invSoft is sized to saturate that to 1 within ~1% of the range - the
  // radial falloff alone shapes the light. No VU1 change, no micro memory.
  if (spot.point) {
    out.direction.x = 0.0F;
    out.direction.y = 0.0F;
    out.direction.z = 0.0F;
    out.cosCut2 = -1.0F;
    out.invSoft = 1.0e4F / objRange2;
    return out;
  }

  out.cosCut2 = spot.cosCutoff * spot.cosCutoff;
  // The VU1/EE cone term is clamp01((t^2 - cosCut2*dist2) * invSoft). Its
  // SIGN is the exact angular cutoff and is distance-independent, but its
  // MAGNITUDE scales with dist2 - so sizing invSoft off the full range
  // (objRange2 * (1 - cosCut2)) made the beam ramp up across the whole
  // range: dim on everything close to the lamp, fully bright only near its
  // far end. On the camera flashlight that reads as "it doesn't light what
  // I'm looking at". Saturate at a fraction of the range instead; the
  // cutoff ANGLE is unchanged, the edge just gets crisper.
  constexpr float kFullBrightAt = 0.18F;  // of the range, on the axis
  const float coneBase =
      objRange2 * kFullBrightAt * kFullBrightAt * (1.0F - out.cosCut2);
  out.invSoft = coneBase > 1e-10F ? spot.softness / coneBase : 0.0F;
  return out;
}
}  // namespace

// Modified by TyraX: the VU1 clipping uniform chain - one quadword of
// constants for the per-triangle crossing test plus the six clip planes as
// (A,B,C,D)+(E,0,0,0) pairs. Lifted out of sendObjectData unchanged so the
// retained-command path can capture exactly these bytes and replay them (see
// the call site); nothing in it depends on the mesh.
void StaPipQBufferRenderer::addClipChain(packet2_t* objectDataPacket) const {
  // Modified by TyraX: the X/Y band follows the bag (setClipBandTight).
  const float band = clipTight ? VU1_CLIP_XY_BAND_TIGHT : VU1_CLIP_XY_BAND;
  packet2_utils_vu_open_unpack(objectDataPacket, VU1_CLIP_CONSTS_ADDR, false);
  {
    packet2_add_float(objectDataPacket, clipNearZ - VU1_CLIP_GUARD);
    packet2_add_float(objectDataPacket, -clipFarZ - VU1_CLIP_GUARD);
    packet2_add_float(objectDataPacket, 0.0F);
    packet2_add_float(objectDataPacket, VU1_CLIP_GUARD);
  }
  packet2_utils_vu_close_unpack(objectDataPacket);

  const float planes[6][8] = {
      // near: z <= clipNearZ (exact, matches the EE clipper)
      {0.0F, 0.0F, -1.0F, 0.0F, clipNearZ, 0.0F, 0.0F, 0.0F},
      // far: z >= clipFarZ (exact, matches the EE clipper)
      {0.0F, 0.0F, 1.0F, 0.0F, -clipFarZ, 0.0F, 0.0F, 0.0F},
      // guard band X/Y at +/-band * w - strictly inside the GS raster
      // window; the scissor trims the rest of the way
      {-1.0F, 0.0F, 0.0F, band, 0.0F, 0.0F, 0.0F, 0.0F},
      {1.0F, 0.0F, 0.0F, band, 0.0F, 0.0F, 0.0F, 0.0F},
      {0.0F, -1.0F, 0.0F, band, 0.0F, 0.0F, 0.0F, 0.0F},
      {0.0F, 1.0F, 0.0F, band, 0.0F, 0.0F, 0.0F, 0.0F},
  };
  packet2_utils_vu_open_unpack(objectDataPacket, VU1_CLIP_PLANES_ADDR, false);
  {
    for (u32 i = 0; i < 6; i++)
      for (u32 j = 0; j < 8; j++)
        packet2_add_float(objectDataPacket, planes[i][j]);
  }
  packet2_utils_vu_close_unpack(objectDataPacket);
}

// Modified by TyraX: with the VIF1 queue a chain runs well after it was
// built, so a REF to the renderer's MVP / light / single-colour storage would
// read whatever the NEXT bag wrote there. Copy them into the packet instead -
// the same inline form bounded submission batching already uses.
constexpr bool kInlineUniforms = TYRA_VIF1_QUEUE != 0;

void StaPipQBufferRenderer::sendObjectData(
    StaPipBag* bag, M4x4* mvp, RendererCoreTextureBuffers* texBuffers) {
#if TYRA_VU1_EXP_PAINT
  const bool paintVu = bag->texture && bag->texture->paintVu;
  TYRA_ASSERT(!paintVu || supportsPaintVu(bag),
              "VU paint uniforms require a supported built-in paint program!");
#endif
  // Modified by TyraX: build uniforms at the head of the same packet that the
  // first geometry flush will append to. This removes one VIF1 DMA launch and
  // its intervening EE wait per visible bag without relying on a NEXT/CALL
  // jump between separately allocated chains (which is not stable on hardware).
  // The other packet was completed before sendPacket flipped back to this
  // context, so it is safe to reuse without adding a preparation-side wait.
  auto* objectDataPacket = packets[context];
  if (!objectDataPending) {
    startPacket(objectDataPacket);
    packetWrapSet = false;  // a fresh packet carries no wrap write yet
#if TYRA_STAPIP_PACKET_PROFILE
    packetTelemetryProducer = telemetryProducer(telemetry);
  } else if (packetTelemetryProducer != telemetryProducer(telemetry)) {
    // Submission batching may deliberately keep one VIF chain open across
    // producer scopes. Do not lie by charging that whole chain to whichever
    // scope happened to flush it.
    packetTelemetryProducer = StaPipProducerMixed;
#endif
  }
  // Modified by TyraX: everything in this chain lands at an ABSOLUTE VU1
  // address (MVP 0..3, OPTIONS 8, CLIP_CONSTS 20, ALPHA 21, the clip planes at
  // 944..955) and the microprograms read those inside their loops. The DMA
  // wait before this packet only proves the previous CHAIN was consumed - its
  // final MSCAL merely started the last batch, and that program may still be
  // running (parked on an XGKICK for as long as the GS takes to fill a big
  // translucent triangle). The VIF performs absolute unpacks immediately and
  // stalls only at the next MSCAL, so without a barrier the next mesh's matrix
  // and plane table land mid-draw: one vertex through the wrong MVP is a wedge
  // to the screen corner, a swapped plane table cuts or drops the polygon.
  // PCSX2 never overlaps a VIF unpack with a running microprogram, so it
  // cannot show this; a real PS2 does (docs/vu1-clipping.md). FLUSHE = wait
  // for the end of the microprogram; the halves the GIF may still be reading
  // are not touched by this chain, so no PATH1 drain is needed here.
  // Modified by TyraX: fast path - and the compact vertex format's bag setup
  // right behind the FLUSHE, in front of everything else the bag unpacks, its
  // object data included (docs/compact-vertex-format.md): STCYCL 1/1, STMOD
  // 0, the box's origin index in ROW, 1.0 in COL. Four quadwords.
  qword_t* const bagHead = objectDataPacket->next;
  {
    qword_t tag = flusheHeader();
    tag.dw[0] = (tag.dw[0] & ~static_cast<u64>(0xFFFF)) | 3u;  // CNT, 3 qw
    bagHead[0] = tag;
    u32 setup[Compact::kBagSetupWords];
    Compact::bagSetup(compactSource.box, setup);
    memcpy(&bagHead[1], setup, sizeof(setup));
    objectDataPacket->next = bagHead + 4;
  }
  asIsFormLoaded = false;  // the compact forms of the uniforms below
  asIsSwapNeeded = false;
  asIsHasSingle = asIsHasEnv = false;
  const float step = compactSource.box.step;
#if TYRA_STAPIP_STAMP
  // Modified by TyraX: stamp capture starts at this bag's FLUSHE; every
  // block below that a stamp cannot re-create clears stampOk.
  stampHead = bagHead;
  stampOk = stampArmed && kInlineUniforms;
  stampHasTex0 = 0;
  stampFogEligible = 0;
  stampWrapBlock = 0;
  stampSingleAt = 0;
  stampEnvAt = 0;
#endif

  // Modified by TyraX: the bag's texture wrap, in the chain (setBagWrap).
  // FLUSH rather than the FLUSHE above: the previous bag's last XGKICK may
  // still be on PATH1, and CLAMP_1 must reach the GS after those primitives,
  // not between them. DIRECT then hands the A+D pair to the GIF over PATH2.
  if (bagWrap != nullptr) {
    const texwrap_t& inForce =
        packetWrapSet ? packetWrap : rendererCore->gs.currentTextureWrap();
    if (!RendererCoreGS::wrapEquals(*bagWrap, inForce)) {
#if TYRA_STAPIP_STAMP
      // State-dependent: a stamp keeps it only in its deterministic form,
      // the first non-REPEAT bag after REPEAT was in force.
      if (RendererCoreGS::wrapEquals(inForce, RendererCoreGS::repeatWrap())) {
        stampWrapBlock = 1;
        stampWrap = *bagWrap;
      } else {
        stampOk = false;
      }
#endif
      packet2_chain_open_cnt(objectDataPacket, 0, 0, 0);
      packet2_vif_flush(objectDataPacket, 0);
      packet2_vif_open_direct(objectDataPacket, 0);
      packet2_add_2x_s64(objectDataPacket,
                         GIF_SET_TAG(1, 1, 0, 0, GIF_FLG_PACKED, 1),
                         GIF_REG_AD);
      packet2_add_2x_s64(
          objectDataPacket,
          GS_SET_CLAMP(bagWrap->horizontal, bagWrap->vertical, bagWrap->minu,
                       bagWrap->maxu, bagWrap->minv, bagWrap->maxv),
          GS_REG_CLAMP_1);
      packet2_vif_close_direct_auto(objectDataPacket);
      packet2_chain_close_tag(objectDataPacket);
      packetWrap = *bagWrap;
      packetWrapSet = true;
    }
    bagWrap = nullptr;
  }
  {
    // Modified by TyraX: MVP x D(step, 0) - the compact programs see grid
    // indices (docs/compact-vertex-format.md). Always inline: a REF would
    // name a matrix that lives only for this call.
#if TYRA_STAPIP_STAMP
    stampMvpAt = static_cast<u16>(objectDataPacket->next - stampHead);
#endif
    alignas(16) M4x4 mvpD;
    mvpDequant(*mvp, step, &mvpD);
    emitUnpack(objectDataPacket, VU1_MVP_MATRIX_ADDR, mvpD.data, 4);
  }

  // Modified by TyraX: experiment (b), TYRA_VU1_EXP_EE_LIGHT_FOLD - the
  // *_fold images read the light directions pre-multiplied by the light
  // matrix and never read the matrix, so the EE folds D * M here (nine
  // multiplies a bag, against three VU1 instructions a vertex) and skips the
  // matrix upload. A project override of a lit program keeps the unfolded
  // pair: its generator still does the multiply on VU1.
  bool foldLights = false;
#if TYRA_STAPIP_STAMP
  if (bag->lighting) stampOk = false;  // light uniforms are live state
#endif
#if TYRA_VU1_EXP_EE_LIGHT_FOLD
  foldLights = bag->lighting != nullptr && !litProgramOverridden();
#endif
  if (bag->lighting) {
  if (foldLights) {
    // VU1: n_w = M0*n.x + M1*n.y + M2*n.z (Mi = the matrix's qword i), then
    // c = D0*n_w.x + D1*n_w.y + D2*n_w.z; so Fi = D0*Mi.x + D1*Mi.y + D2*Mi.z.
    const float* m = bag->lighting->lightMatrix->data;
    const Vec4* d = bag->lighting->dirLights->getLightDirections();
    alignas(16) Vec4 f[3];
    // Modified by TyraX: x 1/127 - the normal arrives as its int8 value
    // (docs/compact-vertex-format.md, "The VU side").
    constexpr float k = 1.0F / 127.0F;
    for (int i = 0; i < 3; i++) {
      const float* mi = m + i * 4;
      f[i].x = (d[0].x * mi[0] + d[1].x * mi[1] + d[2].x * mi[2]) * k;
      f[i].y = (d[0].y * mi[0] + d[1].y * mi[1] + d[2].y * mi[2]) * k;
      f[i].z = (d[0].z * mi[0] + d[1].z * mi[1] + d[2].z * mi[2]) * k;
      f[i].w = 0.0F;
    }
    emitUnpack(objectDataPacket, VU1_LIGHTS_DIRS_ADDR, f, 3);
    // One line per boot, so an A/B arm can prove the fold path RAN - a scene
    // with no lit mesh never takes it and measures nothing.
    static bool foldLogged = false;
    if (!foldLogged) {
      foldLogged = true;
      TYRA_LOG("VU1 experiment (b): first folded light upload");
    }
  } else {
      // Modified by TyraX: the light matrix multiplies the normal: x 1/127
      // (docs/compact-vertex-format.md). Inline - the scaled copy is ours.
      alignas(16) float lm[12];
      const float* src = bag->lighting->lightMatrix->data;
      for (int i = 0; i < 12; ++i) lm[i] = src[i] * (1.0F / 127.0F);
      emitUnpack(objectDataPacket, VU1_LIGHTS_MATRIX_ADDR, lm, 3);
      emitUnpack(objectDataPacket, VU1_LIGHTS_DIRS_ADDR,
                 bag->lighting->dirLights->getLightDirections(), 3);
  }
    // The colours and ambient go up in BOTH cases - a first cut of the
    // fold skipped them with the matrix and every lit mesh drew black
    // (caught by the PCSX2 pixel A/B; --vu-check stages memory itself and
    // cannot see an EE upload that is missing).
    // add_unpack_data emits a DMA REF, not a copy. The mode-adjusted
    // colors must live in the packet, never in a temporary stack array.
    const Vec4* colors = bag->lighting->dirLights->getLightColors();
    {
      alignas(16) Vec4 c[4] = {colors[0], colors[1], colors[2], colors[3]};
      c[3].w = bag->lighting->dirLights->signedSH ? -1.0F : 0.0F;
      emitUnpack(objectDataPacket, VU1_LIGHTS_COLORS_ADDR, c, 4);
    }
  }

  // Modified by TyraX: dynamic light for the color programs - the per-bag
  // pick from StaPipCore (flashlight or the strongest scene point light),
  // falling back to the global flashlight state when no pick was made.
  // The dir-lights addresses are free when the bag has no lighting - the
  // C/TC programs read the three spot quads from there. The same numbers go
  // to the EE clipper for the as_is path.
  //
  // Modified by TyraX: the VU1 ARITHMETIC is gated - `spotActive` below rides
  // in VU1_OPTIONS_ADDR.y and
  // the cull/clip colour programs branch over CalculateTyraSpotLight when it
  // is clear. `enabled` is exactly the predicate the EE clipper's
  // addSpotToColor already used, so the two halves of the formula are gated
  // by one fact and a skipped mesh renders bit-identically. An inactive
  // spot's uniform quads are also omitted unless a custom program may read them.
  bool spotActive = false;
  if (!bag->lighting) {
    const auto& light = bagLight ? *bagLight : rendererCore->spot;
    auto meshSpot = buildSpotForBag(light, bag->info->model);
    // Modified by TyraX: selection is unchanged. Skip the stock lighting
    // loop only when the selected light contributes zero to every vertex.
    // Custom VU stages retain their light/uniform contract even outside it.
    if (meshSpot.enabled && bagLightBounds && !repository.hasAnyOverride() &&
        stapipSpotHasNoInfluence(meshSpot, *bagLightBounds))
      meshSpot.enabled = false;
    clipper.setSpot(meshSpot);
    spotActive = meshSpot.enabled;

    // Modified by TyraX: the VU1 colour programs never read an inactive
    // spot's three quads. An override may, so preserve the upload for one.
#if TYRA_STAPIP_STAMP
    // The spot quads are re-derived per replay; record where they sit and
    // whether the block exists at all (its presence is part of the layout).
    stampSpotActive = spotActive ? 1 : 0;
    stampSpotAt = 0;
    if (repository.hasAnyOverride()) stampOk = false;
#endif
    if (meshSpot.enabled || repository.hasAnyOverride()) {
#if TYRA_STAPIP_STAMP
      stampSpotAt = static_cast<u16>(objectDataPacket->next - stampHead);
#endif
      // Modified by TyraX: in grid units (ruling P9) - the EE clipper above
      // keeps the object-space ones, it reads decoded positions.
      alignas(16) float s[12];
      spotGridConstants(meshSpot, step, s);
      emitUnpack(objectDataPacket, VU1_LIGHTS_DIRS_ADDR, s, 3);
    }

    // Modified by TyraX: the two quadwords a project's own microprogram reads
    // (docs/vu-authoring.md). Inside the `if (!bag->lighting)` on purpose -
    // they occupy the directional-lights COLOUR block, which a lit bag needs.
    if (vuCustomEnabled) {
#if TYRA_STAPIP_STAMP
      stampOk = false;
#endif
      {
        alignas(16) const float c[8] = {vuParams[0], vuParams[1], vuParams[2],
                                        vuParams[3], vuTime[0],   vuTime[1],
                                        vuTime[2],   vuTime[3]};
        emitUnpack(objectDataPacket, VU1_CUSTOM_PARAMS_ADDR, c, 2);
      }
    }
  }

  // Modified by TyraX: VU1 clipping data. One quad of constants for the
  // per-triangle crossing test (see stapip_vu1_shared_defines.h) and the six
  // clip planes as (A,B,C,D)+(E,0,0,0) pairs; inside = dot4(v,ABCD) + E >= 0.
  // Uploaded per mesh: other pipelines may reuse this VU1 memory in between.
  const bool mayClip = bagMayClip;
  bagMayClip = true;
  if (vu1Clipping && mayClip) {
#if TYRA_STAPIP_STAMP
    stampOk = false;  // only partial bags clip; stamps are whole bags
#endif
#if TYRA_STAPIP_RETAINED_COMMANDS
    // Modified by TyraX: retained command data. These fifteen quadwords are
    // the same for every mesh in the run - their only inputs are the
    // renderer's near/far pair and the guard-band constant - so the chain is
    // captured once out of the packet addClipChain just wrote it into and
    // replayed with a memcpy afterwards. 52 float stores per bag became one
    // copy. init() and setVU1Clipping() are the only places those inputs can
    // move and both drop the capture.
    const int cb = clipTight ? 1 : 0;  // which band's capture
    if (clipBlockVifQw[cb] != 0) {
      // Modified by TyraX: one REF to the shared VIF-stream copy instead of
      // fifteen quadwords per bag (193 us of a 75-bag frame on a PS2).
      emitRef(objectDataPacket, clipBlockVif[cb], clipBlockVifQw[cb]);
    } else if (clipBlockQw[cb] != 0) {
      appendChainQwords(objectDataPacket, clipBlock[cb], clipBlockQw[cb]);
    } else {
      const u32 clipStart = packet2_get_qw_count(objectDataPacket);
      addClipChain(objectDataPacket);
      const u32 qw = packet2_get_qw_count(objectDataPacket) - clipStart;
      if (qw > 0 && qw <= (sizeof(clipBlock[cb]) / sizeof(clipBlock[cb][0]))) {
        memcpy(clipBlock[cb],
               reinterpret_cast<const u8*>(objectDataPacket->base) +
                   clipStart * 16,
               qw * 16);
        clipBlockQw[cb] = static_cast<u16>(qw);
        // The REF copy: rewritten only here, after every chain that could
        // still read the old one has run.
        frameWaitVif();
        clipBlockVifQw[cb] = static_cast<u16>(chainToVifStream(
            clipBlock[cb], clipBlockQw[cb], clipBlockVif[cb]));
      }
    }
#else
    addClipChain(objectDataPacket);
#endif
  }

  u8 singleColorEnabled = bag->color->single != nullptr;

  if (singleColorEnabled) {  // Color is placed in 4th slot of
                            // VU1_LIGHTS_MATRIX_ADDR
    // Modified by TyraX: four integer lanes, alpha + 1 (ruling P8) - the
    // compact programs copy it. The as_is programs (the EE clipper's) read
    // the float colour and add vf00 themselves: swapped in for their buffers.
    u32 lanes[4];
    singleColorLanes(*bag->color->single, lanes);
    memcpy(compactSingle, lanes, sizeof(lanes));
    memcpy(asIsSingle, bag->color->single->rgba, sizeof(asIsSingle));
    asIsHasSingle = true;
#if TYRA_STAPIP_STAMP
    stampSingleAt = static_cast<u16>(objectDataPacket->next - stampHead);
#endif
    emitUnpack(objectDataPacket, VU1_SINGLE_COLOR_ADDR, compactSingle, 1);
  }

  // Modified by TyraX: the options block through the fast path too (see
  // emitUnpack) - built as whole qwords, the same GS_SET_* words packet2's
  // helpers would have written, then one cached header + copy.
  {
    alignas(16) qword_t opt[4];
    u32 optQw = 0;
    const u32 sharedClipVariant =
        bag->lighting != nullptr ||
                (bag->texture != nullptr && bag->texture->coordinatesAreNormals)
            ? 1
            : 0;
    // Static-pipeline-only use of the old dynpip lerp lane: C/D and TC/TCE
    // share one clip image per ABI-compatible pair. Other programs ignore it.
    //
    // Modified by TyraX: the lane is THREE-STATE now. Every reader that
    // predates this tested only `> 0` versus `<= 0` (clip_c's three `iblez`,
    // clip_tc's `ibgtz`), so the negative half was free to carry a second
    // fact: the colour programs run CalculateTyraSpotLight only when it is
    // negative. The two cannot collide - the peer variant is selected by a
    // lighting bag or matcap coordinates, and neither of those classes has
    // the spot macro at all - so the variant wins when both are true.
    const s32 variantLane =
#if TYRA_VU1_EXP_PAINT
        paintVu ? 2 :
#endif
        (sharedClipVariant ? 1 : (spotActive ? -1 : 0));
    {
      qword_t& q = opt[optQw++];
      // Single color enabled - or, Modified by TyraX, -1 for a TD bag: the
      // TC clip image carries TD's lighting path and picks it on .x < 0
      // (every reader of the lane tests `> 0` only; a TD bag is never drawn
      // single-colour). Gated on the class being resident so a TD bag walked
      // down to the TC program (residentFallback) is still drawn as TC.
      const bool textureDirLightsBag =
          bag->lighting != nullptr && bag->texture != nullptr &&
          !bag->texture->coordinatesAreNormals && bag->billboard == nullptr &&
          (residentClasses & StaPipClassTextureDirLights) != 0;
      q.sw[0] = textureDirLightsBag ? static_cast<u32>(-1)
                                    : static_cast<u32>(singleColorEnabled);
      q.sw[1] = static_cast<u32>(variantLane);
      // Modified by TyraX: GS hardware fog params (see RendererCoreFog)
      float fogScale = rendererCore->fog.scale;
      float fogOffset = rendererCore->fog.offset;
      // Modified by TyraX: fog scale 0 is the "F is the constant 255" signal
      // cull_tc branches on (setFogConstant), read as the LOW 16 BITS of the
      // float. A real, non-zero scale whose low half happens to be zero gets
      // its lowest mantissa bit set: one ulp, invisible, never mistaken.
      // Decided only where it can pay: a textured, unlit, non-env bag with no
      // spot light reaching it is the one cull_tc runs through its fog-free
      // loop. The box test is the linear maximum of clip w over the box: w
      // at the centre plus |d w / d axis| times the half-extent, three
      // multiplies instead of eight matrix transforms (the transforms cost
      // the EE more than VU1 saved in a spot-lit night scene).
      bool fogConstant = fogOffForBag;
#if TYRA_STAPIP_STAMP
      // The fog words are re-derived per stamp replay from the live MVP and
      // fog parameters; record only the bag-static half of the predicate.
      stampFogEligible = !fogOffForBag && fogBox != nullptr && !spotActive &&
                         bag->lighting == nullptr && bag->texture != nullptr &&
                         !bag->texture->coordinatesAreNormals &&
                         bag->billboard == nullptr;
      stampFogOff = fogOffForBag ? 1 : 0;
#endif
      if (!fogConstant && fogBox != nullptr && fogScale < 0.0F && !spotActive &&
          bag->lighting == nullptr && bag->texture != nullptr &&
          !bag->texture->coordinatesAreNormals && bag->billboard == nullptr) {
        const Vec4& lo = (*fogBox)[0];
        const Vec4& hi = (*fogBox)[7];
        const float* m = mvp->data;
        const float cx = 0.5F * (lo.x + hi.x), hx = 0.5F * (hi.x - lo.x);
        const float cy = 0.5F * (lo.y + hi.y), hy = 0.5F * (hi.y - lo.y);
        const float cz = 0.5F * (lo.z + hi.z), hz = 0.5F * (hi.z - lo.z);
        const float maxW = m[3] * cx + m[7] * cy + m[11] * cz + m[15] +
                           fabsf(m[3]) * hx + fabsf(m[7]) * hy +
                           fabsf(m[11]) * hz;
        fogConstant = maxW * fogScale + fogOffset >= 255.0F;
      }
      if (fogConstant) {
        fogScale = 0.0F;
        fogOffset = 255.0F;
      } else {
        u32 bits;
        memcpy(&bits, &fogScale, 4);
        if ((bits & 0xFFFFU) == 0U && bits != 0U) {
          bits |= 1U;
          memcpy(&fogScale, &bits, 4);
        }
      }
      memcpy(&q.sw[2], &fogScale, 4);
      memcpy(&q.sw[3], &fogOffset, 4);
    }
    {
      qword_t& q = opt[optQw++];
      q.dw[0] = GS_SET_TEX1(lod->calculation, lod->max_level, lod->mag_filter,
                            lod->min_filter, lod->mipmap_select, lod->l,
                            (int)(lod->k * 16.0F));
      q.dw[1] = GS_REG_TEX1;
    }

    // Modified by TyraX: the destination-alpha gate (PipelineInfoBag::
    // dateLit) rides the same in-band TEST qword every mesh already emits -
    // DATE = 1 draws this bag's pixels only where the framebuffer alpha's
    // MSB is 0, which is how the flashlight's shadow volumes mask its light.
    const int date = bag->info->dateLit ? 1 : 0;
    {
      qword_t& q = opt[optQw++];
      if (bag->info->zTestType == PipelineZTest_AllPass) {
        q.dw[0] = GS_SET_TEST(0, 0, 0, 0, date, 0, 0, ZTEST_METHOD_ALLPASS);
      } else if (bag->info->zTestType == PipelineZTest_TestOnly) {
        // Depth-tested, no z write: alpha test fails every pixel and AFAIL
        // keeps the z buffer (GS FB_ONLY - color still written). The ZBUF
        // register (and thus the VU1 options layout) stays untouched.
        q.dw[0] = GS_SET_TEST(DRAW_ENABLE, ATEST_METHOD_ALLFAIL, 0x00,
                              ATEST_KEEP_ZBUFFER, date, DRAW_DISABLE,
                              DRAW_ENABLE, rendererCore->gs.zBuffer.method);
      } else {
        // Cutout alpha: texels with alpha 0 fail the test and must write
        // NOTHING. Upstream passed ATEST_KEEP_FRAMEBUFFER, whose ps2sdk name
        // reads backwards - it is AFAIL=ZB_ONLY (2), "keep the framebuffer,
        // update z". So every transparent texel stamped the z buffer while
        // drawing no colour, and the invisible part of an alpha-cutout card
        // (foliage, decals, grates) occluded whatever was drawn behind it
        // later. ATEST_KEEP_ALL (0) leaves both buffers alone, which is what a
        // cutout means. Opaque geometry carries alpha 0x80 and never fails the
        // test, so nothing else changes.
        q.dw[0] = GS_SET_TEST(DRAW_ENABLE, ATEST_METHOD_NOTEQUAL, 0x00,
                              ATEST_KEEP_ALL, date, DRAW_DISABLE, DRAW_ENABLE,
                              rendererCore->gs.zBuffer.method);
      }
      q.dw[1] = GS_REG_TEST;
    }

    if (texBuffers != nullptr) {
      rendererCore->texture.updateClutBuffer(texBuffers->clut);

      // Added by TyraX: per-bag GS texture function (StaPipTextureBag - the
      // NFS paint pass draws HIGHLIGHT2). TEX0 is emitted per bag right
      // here, so writing the shared texbuffer_t immediately before the emit
      // gives every bag its own TFX - the next bag on the same texture
      // overwrites it again, which is exactly why the mutation is safe.
      texBuffers->core->info.function = bag->texture->textureFunction;
      const texbuffer_t* tb = texBuffers->core;
      const clutbuffer_t* cl = &rendererCore->texture.clut;
      qword_t& q = opt[optQw++];
      q.dw[0] = GS_SET_TEX0(tb->address >> 6, tb->width >> 6, tb->psm,
                            tb->info.width, tb->info.height,
                            tb->info.components, tb->info.function,
                            cl->address >> 6, cl->psm, cl->storage_mode,
                            cl->start, cl->load_method);
      q.dw[1] = GS_REG_TEX0;
    }
#if TYRA_STAPIP_STAMP
    stampOptAt = static_cast<u16>(objectDataPacket->next - stampHead);
    stampHasTex0 = texBuffers != nullptr ? 1 : 0;
    stampTex0At = static_cast<u16>(stampOptAt + optQw);  // TEX0 is last
#endif
    emitUnpack(objectDataPacket, VU1_OPTIONS_ADDR, opt, optQw);
  }

  // Modified by TyraX: particle billboard camera basis (right, up - world
  // space). Reuses the lights-matrix area like the env basis; billboard
  // bags never carry lighting (asserted in StaPipCore::render). Swapping
  // this basis and re-rendering the same bag draws the same centers for
  // another view (e.g. a portal's virtual camera).
  if (bag->billboard != nullptr) {
#if TYRA_STAPIP_STAMP
    stampOk = false;
#endif
    {
      // Modified by TyraX: divided by the step (the centre is a grid index)
      // and by the weights' scale (stored x it) - docs/compact-vertex-format.md.
      const float k = 1.0F / (step * compactSource.weightScale);
      alignas(16) const float bb[8] = {
          bag->billboard->right.x * k, bag->billboard->right.y * k,
          bag->billboard->right.z * k, 0.0F,
          bag->billboard->up.x * k,    bag->billboard->up.y * k,
          bag->billboard->up.z * k,    0.0F};
      emitUnpack(objectDataPacket, VU1_BILLBOARD_BASIS_ADDR, bb, 2);
    }
  }

  // Modified by TyraX: env (matcap) camera basis for the TCE programs.
  // Transpose it and fold in the ST scale here so CalculateTyraEnvStq can
  // evaluate both scaled dot products in one VU1 accumulator chain. Reuses
  // the lights-matrix area; env bags never carry lighting.
  if (bag->texture != nullptr && bag->texture->coordinatesAreNormals) {
#if TYRA_STAPIP_STAMP
    // Camera-dependent, so a stamp re-writes it on replay (envAt) - it used
    // to keep every env pass off the stamp route, ~2.9 ms of EE a frame on
    // Showcase (docs/stapip-stamps.md, "Env bags"). The VU1 paint arm uploads
    // live paint vectors too and stays excluded.
#if TYRA_VU1_EXP_PAINT
    if (paintVu) stampOk = false;
#endif
    stampEnvAt = static_cast<u16>(objectDataPacket->next - stampHead);
#endif
    const Vec4& r = bag->texture->envRight;
    const Vec4& u = bag->texture->envUp;
    {
      // Modified by TyraX: x 1/127 in the x and y lanes, 127 in X.w (the
      // renormalising paths' rsqrt numerator) - docs/compact-vertex-format.md.
      // The as_is env program (the EE clipper's) renormalises to 1: its form
      // is swapped in for its buffers.
      envBasis(r, u, true, compactEnv);
      envBasis(r, u, false, asIsEnv);
      asIsHasEnv = true;
      emitUnpack(objectDataPacket, VU1_ENV_BASIS_ADDR, compactEnv, 3);
#if TYRA_VU1_EXP_PAINT
      if (paintVu) {
        const Vec4& f = bag->texture->paintForward;
        const Vec4& h = bag->texture->paintHalf;
        constexpr float k = 1.0F / 127.0F;  // the paint basis meets a normal
        alignas(16) const float p[8] = {f.x * k, f.y * k, f.z * k, 0.0F,
                                      h.x * k, h.y * k, h.z * k, 0.0F};
        emitUnpack(objectDataPacket, VU1_LIGHTS_DIRS_ADDR, p, 2);
      }
#endif
    }
  }

  // Modified by TyraX: per-mesh GS blend equation, emitted in-band with the
  // other tags by every program (StoreTyraGifTags*Alpha) - no FINISH barrier
  // to switch it. Default alpha-over; the reflective-material env pass sets
  // additiveBlendFix for Cv = Cs*FIX/128 + Cd.
  {
    const u8 fix = bag->info->additiveBlendFix;
    const u8 sub = bag->info->subtractiveBlendFix;
    // Subtractive wins over additive when both are set: (0 - Cs)*FIX + Cd,
    // clamped at 0 - the shadow volumes' count-down pass.
    qword_t a;
    a.dw[0] = sub != 0   ? GS_SET_ALPHA(2, 0, 2, 1, sub)
              : fix != 0 ? GS_SET_ALPHA(0, 2, 2, 1, fix)
                         : GS_SET_ALPHA(0, 1, 0, 1, 0);
    a.dw[1] = GS_REG_ALPHA;
    emitUnpack(objectDataPacket, VU1_ALPHA_ADDR, &a, 1);
  }

  // Modified by TyraX: with the EE clipper, the as_is programs read a float
  // single colour and an env basis without the 1/127 - addBuffersDataToPacket
  // swaps them in around those buffers (emitUniformForm).
  asIsSwapNeeded = !vu1Clipping && (asIsHasSingle || asIsHasEnv);

  // Do not terminate the DMA chain here: addBuffersDataToPacket appends the
  // first geometry commands and adds the single END tag for the whole packet.
  // A wholly culled mesh simply replaces this unsent packet on the next call.
  objectDataPending = true;
}

// Modified by TyraX: the uniforms whose form differs between the compact
// programs and the as_is ones, re-uploaded - after a FLUSHE, because a
// running program reads them (docs/compact-vertex-format.md, "The VU side").
void StaPipQBufferRenderer::emitUniformForm(packet2_t* packet, bool asIs) {
  emitQword(packet, flusheHeader());
  if (asIsHasSingle)
    emitUnpack(packet, VU1_SINGLE_COLOR_ADDR, asIs ? asIsSingle : compactSingle,
               1);
  if (asIsHasEnv)
    emitUnpack(packet, VU1_ENV_BASIS_ADDR, asIs ? asIsEnv : compactEnv, 3);
}

// Modified by TyraX: the compact vertex format's routes
// (docs/compact-vertex-format.md, "Routes") - where a bag's draw reads its
// vertices.
const StaPipCompactStream* StaPipQBufferRenderer::acquireStream(
    StaPipBag* bag, u32 packageSize) {
  sourceEntry = nullptr;
  refusedStream = false;
  if (bag->compact != nullptr) {
    // The bag's own stream: drawn in place, registered as an immutable span
    // the first time (the frame recorder borrows it instead of copying it;
    // its release unregisters it) - when it is the layout the bag's program
    // reads at this package size. Every route's fallback (the classifier's
    // boxes, a package packed into the pool) reads float arrays a native bag
    // need not have, so a stream that is not drawable is refused unless the
    // bag has them.
    auto* stream = const_cast<StaPipCompactStream*>(bag->compact);
    const bool drawable =
        stream->allocated() && packageSize >= 3 &&
        packageSize <= StaPipCompactWriter::kMaxPackage &&
        stream->layout() == StaPipCompactLayout::forBag(*bag, packageSize);
    if (drawable) {
      if (!stream->isSpanRegistered() &&
          Vif1Queue::registerImmutableSpan(stream->data(), stream->qwords()))
        stream->setSpanRegistered(true);
      return stream;
    }
    if (bag->vertices == nullptr || bag->count == 0) {
      refusedStream = true;
      static const void* logged[16];
      static u32 loggedCount = 0;
      bool seen = false;
      for (u32 i = 0; i < loggedCount; ++i) seen = seen || logged[i] == bag;
      if (!seen && loggedCount < 16) {
        logged[loggedCount++] = bag;
        printf("[COMPACT] refused bag=%p: its stream is not the layout its "
               "program reads (allocated %d, package size %u, the program's "
               "%u) and it has no float arrays - not drawn\n",
               static_cast<const void*>(bag), stream->allocated() ? 1 : 0,
               static_cast<unsigned>(stream->layout().packageSize),
               static_cast<unsigned>(packageSize));
      }
      return nullptr;
    }
    // Float arrays beside it: drawn from those, like any float bag.
  }
#if TYRA_STAPIP_BAKED_STREAM
  // A float bag: the stream this renderer packs for it. Billboards are
  // rewritten every frame by the EE (no stream would ever replay).
  if (bag->billboard != nullptr || bag->vertices == nullptr || bag->count == 0 ||
      packageSize < 3 || packageSize > StaPipCompactWriter::kMaxPackage)
    return nullptr;
  const StaPipCompactLayout layout =
      StaPipCompactLayout::forBag(*bag, packageSize);
  StaPipBakedEntry key;
  key.vertices = bag->vertices;
  key.sts = bag->texture != nullptr ? bag->texture->coordinates : nullptr;
  key.colors = bag->color->many;
  key.normals = bag->lighting != nullptr ? bag->lighting->normals : nullptr;
  key.count = bag->count;
  key.maxVertCount = packageSize;
  key.bboxVersion = bag->bboxVersion;
  // The four per-stream content stamps, folded (the mixing constants keep two
  // streams swapping equal stamps from being a no-op).
  key.contentVersion =
      stampWord(bag->contentVersion) * 0x9E3779B1U +
      stampWord(bag->color->contentVersion) * 0x85EBCA6BU +
      stampWord(bag->texture != nullptr ? bag->texture->contentVersion
                                      : nullptr) *
          0xC2B2AE35U +
      stampWord(bag->lighting != nullptr ? bag->lighting->contentVersion
                                       : nullptr) *
          0x27D4EB2FU;
  key.layoutKey = layoutKeyOf(layout);
  key.packages = static_cast<u16>((bag->count + packageSize - 1) / packageSize);
  StaPipBakedEntry* entry = baked.acquire(key);
  if (entry == nullptr) return nullptr;
  const bool world = identityModel(bag);
  const u32 stepBits = floatBitsOf(Compact::sceneStep());
  // A world stream packed at another scene step would break the seam rule.
  if (entry->complete && entry->world && entry->sceneStepBits != stepBits)
    entry->pendingPatch = 1;
  if (!entry->complete || entry->pendingPatch) {
    // Packed now: the bag's box (its positions' bounds) and its arrays,
    // through the batch quantizer. A repack goes to the spare (ping-pong).
    const bool repack = entry->pendingPatch != 0;
    // Modified by TyraX: what this pack reads, stamp by stamp. A repack
    // keeps the runs whose attribute did not move: positions and normals
    // move together (a skinned body rewrites both and bumps bboxVersion),
    // ST and colours only with their own stamp - a body's ST never changes
    // between poses. Anything the vertices' content stamp says moved is
    // packed again (it is the owner's word for the whole bag).
    const u32 stamps[5] = {
        bag->bboxVersion, stampWord(bag->contentVersion),
        stampWord(bag->color->contentVersion),
        stampWord(bag->texture != nullptr ? bag->texture->contentVersion
                                          : nullptr),
        stampWord(bag->lighting != nullptr ? bag->lighting->contentVersion
                                           : nullptr)};
    u8 keep = 0;
    const void* sts = bag->texture != nullptr ? bag->texture->coordinates : nullptr;
    const void* normals = bag->lighting != nullptr ? bag->lighting->normals : nullptr;
    if (repack && entry->stream.allocated() && entry->packedSts == sts &&
        entry->packedColors == bag->color->many &&
        entry->packedNormals == normals) {
      const u32* was = entry->packedStamps;
      const bool sameVertices = stamps[1] == was[1];
      const bool samePositions = sameVertices && stamps[0] == was[0];
      for (u8 a = 0; a < layout.attrCount; ++a) {
        bool same = false;
        switch (layout.attrs[a]) {
          case Compact::Attr::Position:
            same = samePositions && entry->world == (world ? 1 : 0) &&
                   entry->sceneStepBits == stepBits;
            break;
          case Compact::Attr::Normal:
          case Compact::Attr::EnvNormal:
            same = samePositions && stamps[4] == was[4] && stamps[3] == was[3];
            break;
          case Compact::Attr::St:
            same = sameVertices && stamps[3] == was[3];
            break;
          case Compact::Attr::Color:
            same = sameVertices && stamps[2] == was[2];
            break;
          default:
            break;
        }
        if (same) keep |= static_cast<u8>(1u << a);
      }
    }
    if (!baked.pack(*entry, layout, Compact::sourceOf(*bag),
                    Compact::boxForBag(*bag), keep)) {
      entry->complete = 0;
      if (repack) baked.countRefusedPatch();
      return nullptr;  // this frame: packed into the copy pool as drawn
    }
    for (u32 s = 0; s < 5; ++s) entry->packedStamps[s] = stamps[s];
    entry->packedSts = sts;
    entry->packedColors = bag->color->many;
    entry->packedNormals = normals;
    entry->pendingPatch = 0;
    entry->world = world ? 1 : 0;
    entry->sceneStepBits = stepBits;
  }
#if TYRA_STAPIP_BAKED_ANY_VERIFY
  // The stream against a fresh pack of the bag as it is now: a difference is
  // a missing invalidation (the stream would have drawn stale data).
  if (baked.verifying(entry)) {
    StaPipCompactStream fresh;
    ++baked.verifyChecked;
    if (Compact::packArrays(layout, Compact::sourceOf(*bag),
                            entry->stream.box(), fresh)) {
      bool same = fresh.qwords() == entry->stream.qwords();
      for (u32 p = 0; same && p < fresh.packageCount(); ++p)
        for (u8 a = 0; same && a < layout.attrCount; ++a)
          same = memcmp(fresh.run(p, a), entry->stream.run(p, a),
                        fresh.runQwords(p, a) * 16) == 0;
      if (!same) {
        ++baked.verifyFailed;
        TYRA_LOG("STAPIPVERIFY MISMATCH count=", entry->count,
                 " packages=", entry->packages, " bboxV=", entry->bboxVersion,
                 " contentV=", entry->contentVersion);
      }
    }
  }
#endif
  baked.noteDrawn(*entry);
  sourceEntry = entry;
  return &entry->stream;
#else
  (void)packageSize;
  return nullptr;
#endif
}

void StaPipQBufferRenderer::setCompactSource(StaPipBag* bag,
                                             const StaPipCompactStream* stream,
                                             u32 packageSize,
                                             const float* bounds) {
  StaPipQBufferSource& src = compactSource;
  src.bag = bag;
  src.stream = stream;
  src.layout = StaPipCompactLayout::forBag(*bag, packageSize);
  if (stream != nullptr) {
    src.box = stream->box();
    src.weightScale = stream->weightScale();
  } else {
    // Packed into the copy pool as drawn: a box from the bag's bounds (the
    // classifier's, when it has them), world space on the scene grid when
    // its model matrix is the identity - the same box a stream would take.
    if (bounds != nullptr) {
      bool coarse = false;
      src.box = identityModel(bag)
                    ? Compact::worldBox(bounds, bounds + 3,
                                        Compact::sceneStep(), &coarse)
                    : Compact::modelBox(bounds, bounds + 3);
    } else {
      src.box = Compact::boxForBag(*bag);
    }
    src.weightScale = 4096.0F;
    if (bag->billboard != nullptr && bag->texture != nullptr &&
        bag->texture->coordinates != nullptr) {
      // The weights' scale: the largest that keeps every weight in int16.
      float largest = 0.0F;
      const float* w = bag->texture->coordinates->xyzw;
      for (u32 i = 0; i < bag->count * 4; ++i) {
        const float a = fabsf(w[i]);
        if (a > largest) largest = a;
      }
      src.weightScale = Compact::weightScaleFor(largest);
    }
  }
  StaPipQBuffer::setSource(src);
}

void StaPipQBufferRenderer::setInfo(PipelineInfoBag* bag) {
  prim->antialiasing = bag->antiAliasingEnabled;
  prim->blending = bag->blendingEnabled;
  prim->shading = bag->shadingType;

  if (bag->textureMappingType == TyraLinear) {
    lod->mag_filter = LOD_MAG_LINEAR;
    lod->min_filter = LOD_MIN_LINEAR;
  } else {
    lod->mag_filter = LOD_MAG_NEAREST;
    lod->min_filter = LOD_MIN_NEAREST;
  }
}

void StaPipQBufferRenderer::sendStaticData() const {
  packet2_reset(staticDataPacket, false);
  packet2_utils_vu_open_unpack(staticDataPacket, VU1_SET_GIFTAG_ADDR, false);
  { packet2_utils_gif_add_set(staticDataPacket, 1); }
  packet2_utils_vu_close_unpack(staticDataPacket);

  packet2_utils_vu_add_end_tag(staticDataPacket);
  { HardwareTrace::Scope trace("VIF1_DMA_wait");
    HardwareTrace::state("VIF1_WAIT_START");
    frameWaitVif();
    HardwareTrace::state("VIF1_WAIT_END"); }
  frameSendPacket(staticDataPacket, DMA_CHANNEL_VIF1, true);
}

// Modified by TyraX: micro-memory words a program cache of these programs
// occupies - Path1::createProgramsCache's own arithmetic (back to back, each
// rounded to an even count, an image shared by two wrappers counted once).
static u32 programSetWords(VU1Program* const* programs, const u32& count) {
  u32 words = 0;
  for (u32 i = 0; i < count; i++) {
    bool shared = false;
    for (u32 j = 0; j < i && !shared; j++)
      shared = programs[i]->getStart() == programs[j]->getStart() &&
               programs[i]->getEnd() == programs[j]->getEnd();
    if (!shared) words += programs[i]->getProgramSize();
  }
  return words;
}

void StaPipQBufferRenderer::setProgramsCache() {
  // Modified by TyraX: in VU1 clipping mode the clip programs replace
  // the as_is family (both plus cull would overflow VU1 micro memory, and
  // as_is is only ever fed by the retired EE clipper path).
  // The env (matcap) variants ride along in both sets: cull_tce +
  // as_is_tce with the EE clipper, cull_tce + clip_tce in VU1-clipping
  // mode. The clip family uses three resident images: C/D share the C image,
  // TC/TCE share TC, and TD stays specialised. Their input/scratch/output ABIs
  // match within each pair; VU1_OPTIONS_ADDR.y selects only the per-corner
  // shading path. Path1 aliases identical source ranges to one destination.
  // Check the exact micro-memory budget with nm after touching any program -
  // the createProgramsCache overflow assert is compiled out in release.
  // Modified by TyraX: the set is DATA-DRIVEN now (setResidentClasses). Each
  // material class contributes a pair - the cull program plus its clipped or
  // as_is twin - and a class the project never draws can be dropped to buy
  // micro memory for a program the user wrote. Every class is kept unless a
  // game says otherwise, so this is the old hardcoded ten by default.
  VU1Program* programs[12];  // ten class programs + the two billboards
  u32 count = 0;
  const struct {
    u32 bit;
    StaPipProgramName cull, clipped, asIs;
  } classes[] = {
      {StaPipClassColor, StaPipCullColor, StaPipClipColor, StaPipAsIsColor},
      {StaPipClassDirLights, StaPipCullDirLights, StaPipClipDirLights,
       StaPipAsIsDirLights},
      {StaPipClassTextureDirLights, StaPipCullTextureDirLights,
       StaPipClipTextureDirLights, StaPipAsIsTextureDirLights},
      {StaPipClassTextureColor, StaPipCullTextureColor, StaPipClipTextureColor,
       StaPipAsIsTextureColor},
      {StaPipClassTextureEnv, StaPipCullTextureEnv, StaPipClipTextureEnv,
       StaPipAsIsTextureEnv},
  };
  for (const auto& c : classes) {
    if ((residentClasses & c.bit) == 0) continue;
    programs[count++] = repository.getProgram(c.cull);
    programs[count++] =
        repository.getProgram(vu1Clipping ? c.clipped : c.asIs);
  }
  // Modified by TyraX: the billboard family joins the resident set whenever it
  // fits under the draw-finish helper - with clip TD sharing the TC image the
  // all-class VU1-clipping set is ~1740 of 2042 words and the pair is 206, so
  // that is every built-in configuration. Resident, ensureProgramSet never
  // swaps: a swap was two VIF1 drains plus a ~15.5 KB MPG upload per
  // billboard/non-billboard transition, several per frame in a scene with
  // particles. Only a set that a project's own programs have grown past the
  // ceiling falls back to the old separate packet, swapped in on demand.
  // Decided on every rebuild, because overrides and setResidentClasses change
  // the set's size at run time.
  VU1Program* billboardPrograms[2] = {
      repository.getProgram(StaPipBillboardColor),
      repository.getProgram(StaPipBillboardTexture)};
  billboardsResident = programSetWords(programs, count) +
                           programSetWords(billboardPrograms, 2) <=
                       path1->getDrawFinishAddr();
  if (billboardsResident) {
    programs[count++] = billboardPrograms[0];
    programs[count++] = billboardPrograms[1];
  }
  programsPacket = path1->createProgramsCache(programs, count, 0);

  // createProgramsCache assigns each program's micro-memory address, so the
  // fallback packet is rebuilt AFTER the main one: its programs must point at
  // address 0 (where the swap puts them), not at wherever an earlier resident
  // build had parked them.
  if (billboardProgramsPacket != nullptr) {
    packet2_free(billboardProgramsPacket);
    billboardProgramsPacket = nullptr;
  }
  if (!billboardsResident)
    billboardProgramsPacket =
        path1->createProgramsCache(billboardPrograms, 2, 0);
  TYRA_LOG("StaPip VU1 program set: ", count, " programs, billboards ",
           billboardsResident ? "resident" : "swapped on demand");
}

// TyraX addition: see the header. Safe to call at run time - a level that stops
// needing a material class can hand its micro memory to something else - but it
// is a full pipeline drain plus an upload, so it belongs at a zone or level
// boundary, never per bag.
void StaPipQBufferRenderer::setResidentClasses(const u32& mask) {
  const u32 wanted = mask | StaPipClassColor;  // colour is the fallback floor
  if (wanted == residentClasses) return;
  flushPendingPacket();
  residentClasses = wanted;
  if (programsPacket == nullptr) return;  // init() will build the right set

  packet2_free(programsPacket);
  setProgramsCache();
  uploadPrograms();
  clearLastProgramName();
}

// TyraX addition: experiment (b) folds the light matrix on the EE only for the
// engine's own lit programs - see sendObjectData.
bool StaPipQBufferRenderer::litProgramOverridden() const {
  return repository.hasOverride(StaPipCullDirLights) ||
         repository.hasOverride(StaPipCullTextureDirLights) ||
         repository.hasOverride(StaPipClipDirLights) ||
         repository.hasOverride(StaPipClipTextureDirLights) ||
         repository.hasOverride(StaPipAsIsDirLights) ||
         repository.hasOverride(StaPipAsIsTextureDirLights);
}

// TyraX addition: which material class a program name belongs to, so residency
// can be tested before anything is substituted.
static u32 classOfProgram(const StaPipProgramName& name) {
  switch (name) {
    case StaPipCullColor:
    case StaPipClipColor:
    case StaPipAsIsColor:
      return StaPipQBufferRenderer::StaPipClassColor;
    case StaPipCullDirLights:
    case StaPipClipDirLights:
    case StaPipAsIsDirLights:
      return StaPipQBufferRenderer::StaPipClassDirLights;
    case StaPipCullTextureDirLights:
    case StaPipClipTextureDirLights:
    case StaPipAsIsTextureDirLights:
      return StaPipQBufferRenderer::StaPipClassTextureDirLights;
    case StaPipCullTextureColor:
    case StaPipClipTextureColor:
    case StaPipAsIsTextureColor:
      return StaPipQBufferRenderer::StaPipClassTextureColor;
    case StaPipCullTextureEnv:
    case StaPipClipTextureEnv:
    case StaPipAsIsTextureEnv:
      return StaPipQBufferRenderer::StaPipClassTextureEnv;
    default:
      return 0;  // billboards and anything else: not class-managed
  }
}

// TyraX addition: see the header.
StaPipProgramName StaPipQBufferRenderer::residentFallback(
    const StaPipProgramName& name) const {
  const u32 cls = classOfProgram(name);
  // Not class-managed, or its class is resident: nothing to substitute.
  if (cls == 0 || (residentClasses & cls) != 0) return name;

  switch (name) {
    case StaPipCullTextureDirLights:
      return (residentClasses & StaPipClassTextureColor)
                 ? StaPipCullTextureColor
                 : StaPipCullColor;
    case StaPipClipTextureDirLights:
      return (residentClasses & StaPipClassTextureColor)
                 ? StaPipClipTextureColor
                 : StaPipClipColor;
    case StaPipAsIsTextureDirLights:
      return (residentClasses & StaPipClassTextureColor)
                 ? StaPipAsIsTextureColor
                 : StaPipAsIsColor;
    case StaPipCullTextureEnv:
      return (residentClasses & StaPipClassTextureColor)
                 ? StaPipCullTextureColor
                 : StaPipCullColor;
    case StaPipClipTextureEnv:
      return (residentClasses & StaPipClassTextureColor)
                 ? StaPipClipTextureColor
                 : StaPipClipColor;
    case StaPipAsIsTextureEnv:
      return (residentClasses & StaPipClassTextureColor)
                 ? StaPipAsIsTextureColor
                 : StaPipAsIsColor;
    case StaPipCullDirLights:
    case StaPipCullTextureColor:
      return StaPipCullColor;
    case StaPipClipDirLights:
    case StaPipClipTextureColor:
      return StaPipClipColor;
    case StaPipAsIsDirLights:
    case StaPipAsIsTextureColor:
      return StaPipAsIsColor;
    default:
      return name;
  }
}

// TyraX addition: see the header. sinf/cosf on the EE once per call is nothing
// next to what the same series costs three times per vertex on VU1.
void StaPipQBufferRenderer::setVuTime(const float& seconds) {
  vuTime[0] = seconds;
  vuTime[1] = sinf(seconds);
  vuTime[2] = cosf(seconds);
  vuTime[3] = 1.0F;
}

void StaPipQBufferRenderer::setProgramOverride(const StaPipProgramName& name,
                                               StaPipVU1Program* program) {
  flushPendingPacket();
  repository.setOverride(name, program);
  if (programsPacket == nullptr) return;  // init() will pick it up

  packet2_free(programsPacket);
  setProgramsCache();
  uploadPrograms();
  clearLastProgramName();
}

// TyraX addition: see the header. Same work as setProgramOverride, once.
void StaPipQBufferRenderer::setProgramOverrides(
    const StaPipProgramName* names, StaPipVU1Program* const* programs,
    u32 count) {
  flushPendingPacket();
  for (u32 i = 0; i < count; i++) repository.setOverride(names[i], programs[i]);
  if (programsPacket == nullptr) return;  // init() will pick them up

  packet2_free(programsPacket);
  setProgramsCache();
  uploadPrograms();
  clearLastProgramName();
}

void StaPipQBufferRenderer::setVU1Clipping(const bool& enabled) {
  if (vu1Clipping == enabled) return;
  flushPendingPacket();
  vu1Clipping = enabled;
#if TYRA_STAPIP_RETAINED_COMMANDS
  // Modified by TyraX: the clip chain is either present or absent per mode,
  // and every bag's cull program changes with it.
  clipBlockQw[0] = clipBlockQw[1] = 0;
  clipBlockVifQw[0] = clipBlockVifQw[1] = 0;
  retained.clear();
#endif
#if TYRA_STAPIP_BAKED_STREAM
  // The clip chain is present or absent per mode and every bag's cull program
  // changes with it - and a stream's captured entries carry the program's
  // MSCAL: the streams are packed and captured again.
  bakedStream = nullptr;
  sourceEntry = nullptr;
  baked.clear();
#endif

  if (programsPacket == nullptr) return;  // init() will build the right set

  packet2_free(programsPacket);
  setProgramsCache();
  uploadPrograms();
  clearLastProgramName();
}

void StaPipQBufferRenderer::uploadPrograms() {
  { HardwareTrace::Scope trace("VIF1_DMA_wait");
    HardwareTrace::state("VIF1_WAIT_START");
    frameWaitVif();
    HardwareTrace::state("VIF1_WAIT_END"); }
  frameSendPacket(programsPacket, DMA_CHANNEL_VIF1, true);
  { HardwareTrace::Scope trace("VIF1_DMA_wait");
    HardwareTrace::state("VIF1_WAIT_START");
    frameWaitVif();
    HardwareTrace::state("VIF1_WAIT_END"); }
  billboardSetActive = false;  // Modified by TyraX: main set is resident now
}

// Modified by TyraX: swap between the resident program set and the
// billboard set (both packets are prebuilt - this is one VIF1 MPG upload,
// the VIF stalls it until VU1 halts, so it is safe mid-frame).
void StaPipQBufferRenderer::ensureProgramSet(const bool& billboard) {
  // Modified by TyraX: resident billboards (see setProgramsCache) never swap.
  if (billboardsResident || billboardSetActive == billboard) return;
  flushPendingPacket();
  billboardSetActive = billboard;

  const u32 waitStart = telemetry != nullptr ? readTelemetryTicks() : 0;

  { HardwareTrace::Scope trace("VIF1_DMA_wait");
    HardwareTrace::state("VIF1_WAIT_START");
    frameWaitVif();
    HardwareTrace::state("VIF1_WAIT_END"); }
  frameSendPacket(
      billboard ? billboardProgramsPacket : programsPacket, DMA_CHANNEL_VIF1,
      true);
  { HardwareTrace::Scope trace("VIF1_DMA_wait");
    HardwareTrace::state("VIF1_WAIT_START");
    frameWaitVif();
    HardwareTrace::state("VIF1_WAIT_END"); }
  if (telemetry != nullptr) {
    ++telemetry->programSetSwaps;
    telemetry->programSetWaitTicks += readTelemetryTicks() - waitStart;
  }
  clearLastProgramName();
}

void StaPipQBufferRenderer::setDoubleBuffer() {
  u16 startingAddr = VU1_STAPIP_LAST_ITEM_ADDR + 1;
  // Modified by TyraX: the double buffer stops below the VU1 clipping
  // scratch area (plane table + Sutherland-Hodgman polygons) at the top of
  // VU1 data memory - see stapip_vu1_shared_defines.h.
  const u16 bufferMaxSize = VU1_STAPIP_DBUFFER_END;
  bufferSize = (bufferMaxSize - startingAddr) / 2;

  path1->setDoubleBuffer(startingAddr, bufferSize);

  bufferSize -= 1;  // Because we don't want to upload anything from first
                    // buffer, to first addr of second buffer
}

StaPipQBuffer* StaPipQBufferRenderer::getBuffer() {
  currentBufferIndex = nextBufferIndex++;
  Verbose("Proposing buffer: ", currentBufferIndex);
  auto* result = buffers[currentBufferIndex];
  // Modified by TyraX: one gate every buffer passes through before any fill,
  // so a slot recycled out of a retained bag cannot carry a stale package
  // index into a copied or clipped one. StaPipCore sets it back after the fill
  // for the routes that may be retained.
  result->retainIndex = -1;
  result->bakeIndex = -1;  // Modified by TyraX: same gate, same reason
#if TYRA_STAPIP_PROBE_UNCACHED_CHAIN
  // Probe B: same gate, same reason - a recycled slot must not carry a stale
  // "my streams are in the copy pool" claim into a fillByPointer buffer.
  result->probeCopyFilled = false;
#endif
  if (nextBufferIndex >= buffersCount) {
    Verbose("Rollup - clearing buffer indices. currentBufferIndex: ",
            currentBufferIndex, " (before)nextBufferIndex: ", nextBufferIndex);
    nextBufferIndex = 0;
  }
  return result;
}

u16 StaPipQBufferRenderer::getQBufferIndex(StaPipQBuffer* buffer) {
  // Modified by TyraX: render submits the buffer just acquired by getBuffer.
  if (buffers[currentBufferIndex] == buffer) return currentBufferIndex;
  for (u16 i = 0; i < buffersCount; i++) {
    if (buffers[i] == buffer) return i;
  }
  TYRA_TRAP("Buffer not found!");
  return 0;
}

bool StaPipQBufferRenderer::is1stDBufferFlushTime() {
  return nextBufferIndex == buffersCount / 2;
}

bool StaPipQBufferRenderer::is2ndDBufferFlushTime() {
  return nextBufferIndex == 0;
}

void StaPipQBufferRenderer::flushBuffers() {
  auto is1stDBuffer = is1stDBufferFlushTime();
  auto is2ndDBuffer = is2ndDBufferFlushTime();

  if (!is1stDBuffer && !is2ndDBuffer) {
    auto offset = currentBufferIndex >= buffersCount / 2 ? buffersCount / 2 : 0;
    auto size = (currentBufferIndex + 1) - offset;
    Verbose("-- End flush. from: ", offset, " to: ", offset + size);
    // Modified by TyraX: a batched PARTIAL bag (StaPipCore::setPartialBatch)
    // may not stay pending in two cases. (1) It copied into the slot pools
    // (strip expansion, merged subpackages): the next bag reuses those slots
    // on the same pool side, which only a send flips. (2) A half flush sent
    // the packet mid-bag: the uniforms are gone with it, so objectDataPending
    // is false and nothing would ever send what is appended now. Either way
    // the packet goes out at the end of this bag, as an unbatched one does.
    const bool keepPending =
        submissionBatchCandidate && objectDataPending &&
        !StaPipQBuffer::poolTouched();
    addBuffersDataToPacket(offset, offset + size, !keepPending);
    if (!keepPending) {
      sendPacket();
      submissionBatchBags = 0;
    } else if (++submissionBatchBags >= kSubmissionBatchSize) {
      flushPendingPacket();
    }
  }

  currentBufferIndex = 0;
  nextBufferIndex = 0;

  // Modified by TyraX: resetting the indices hands the slots to the next bag
  // while the packet just sent is still being read by the DMA, and the slots'
  // own arrays are what its REF tags point at whenever a package was COPIED
  // (fillByCopyMax / fillByCopy1By2 merge small in-frustum packages in both
  // clipping modes; fillByCopy1By3 / writeChunk are the EE clipper's). That is
  // safe only because those arrays live in the pool side the last send moved
  // away from - StaPipQBuffer::flipPoolSide in sendPacket. Do not add a DMA
  // wait here instead: measured at -4 FPS on the console (docs/vu1-clipping.md).

  Verbose("End flush - zeroing buffer indices.");
}

void StaPipQBufferRenderer::cull(StaPipQBuffer* buffer) {
  if (buffer->size == 0) {
    return;
  }

  dBufferPrograms[getQBufferIndex(buffer)] = getCullProgramByBag(buffer->bag);

  Verbose("Add cull[", getQBufferIndex(buffer), "]: ", buffer->size);

  auto is1stDBuffer = is1stDBufferFlushTime();
  auto is2ndDBuffer = is2ndDBufferFlushTime();

  if (is1stDBuffer || is2ndDBuffer) {
    auto from = is1stDBuffer ? 0 : buffersCount / 2;
    auto to = from + buffersCount / 2;

    Verbose("-- Half flush at ", getQBufferIndex(buffer), ". from: ", from,
            " to: ", to);

    addBuffersDataToPacket(from, to);
    sendPacket();
  }
}

void StaPipQBufferRenderer::clip(StaPipQBuffer* buffer) {
  if (buffer->size == 0) {
    return;
  }

  // Modified by TyraX: VU1 clipping - clip-classified packages go to
  // the clip VU1 programs with raw object-space vertices (exactly like the
  // cull path); the EE clipper is bypassed entirely. The package occupancy
  // cap (StaPipCore::getClipPackageDivisor) bounds the on-VU1 fan-out.
  if (vu1Clipping) {
    dBufferPrograms[getQBufferIndex(buffer)] = getClipProgramByBag(buffer->bag);

    Verbose("Add vu1-clip[", getQBufferIndex(buffer), "]: ", buffer->size);

    auto is1stDBuffer = is1stDBufferFlushTime();
    auto is2ndDBuffer = is2ndDBufferFlushTime();

    if (is1stDBuffer || is2ndDBuffer) {
      auto from = is1stDBuffer ? 0 : buffersCount / 2;
      auto to = from + buffersCount / 2;

      addBuffersDataToPacket(from, to);
      sendPacket();
    }
    return;
  }

  // Modified by TyraX: a subpackage clipped against the frustum can fan
  // out into more verts than one VU1 buffer holds (maxVertCount). Drain the
  // clip result across as many buffer slots / VU1 draws as needed instead of
  // overflowing a single buffer (that tripped the "Max buffer size in VU1"
  // assert in stapip_qbuffer.cpp while walking around a scene).
  StaPipBag* bag = buffer->bag;
  // Modified by TyraX: the EE clipper reads floats - the buffer's compact runs
  // decoded (positions the grid points the cull programs draw).
  if (!buffer->decodeToFloats()) return;
  const u32 total = clipper.clipToPool(buffer);

  // chunk is a multiple of 3 so a triangle is never split across two draws.
  const u32 chunk = (maxVertCount / 3) * 3;

  u32 emitted = 0;
  StaPipQBuffer* target = buffer;  // reuse the slot the caller already acquired
  do {
    const u32 remaining = total - emitted;
    const u32 n = remaining < chunk ? remaining : chunk;

    target->bag = bag;
    clipper.writeChunk(target, emitted, n);  // n == 0 -> empty slot, skipped
    dBufferPrograms[getQBufferIndex(target)] = getAsIsProgramByBag(bag);

    Verbose("Add clip[", getQBufferIndex(target), "]: ", target->size);

    auto is1stDBuffer = is1stDBufferFlushTime();
    auto is2ndDBuffer = is2ndDBufferFlushTime();

    if (is1stDBuffer || is2ndDBuffer) {
      auto from = is1stDBuffer ? 0 : buffersCount / 2;
      auto to = from + buffersCount / 2;

      Verbose("-- Half flush at ", getQBufferIndex(target), ". from: ", from,
              " to: ", to);

      addBuffersDataToPacket(from, to);
      sendPacket();
    }

    emitted += n;
    if (emitted >= total) break;

    target = getBuffer();  // next slot for the next chunk
  } while (true);
}

void StaPipQBufferRenderer::clearLastProgramName() {
  lastProgramName = StaPipUndefinedProgram;
}

// Modified by TyraX: retained command data - see StaPipRetainedCommands.
#if TYRA_STAPIP_RETAINED_COMMANDS

bool StaPipQBufferRenderer::beginRetainedBag(StaPipBag* bag,
                                             const u32& packageSize) {
  retainedCurrent = nullptr;
  if (bag == nullptr || packageSize == 0) return false;
  // A billboard bag expands centres on VU1 from a basis that changes every
  // frame, and it swaps the whole program set; a game-supplied writer has no
  // packet-size ABI and may not be position-independent. Both are excluded for
  // the same reason the submission-batch scope excludes them.
  if (bag->billboard != nullptr || repository.hasAnyOverride()) return false;

  // Modified by TyraX: the blocks' REFs name the bag's compact stream, so the
  // stream is the key - its block and its content version (a repack moves
  // one or the other). A bag packed into the copy pool has nothing stable to
  // name and is never retained.
  const StaPipCompactStream* stream = compactSource.stream;
  if (stream == nullptr || stream->layout().packageSize != packageSize)
    return false;
  StaPipRetainedEntry key;
  key.vertices = stream->data();
  key.sts = nullptr;
  key.colors = nullptr;
  key.normals = nullptr;
  key.program = getCullProgramByBag(bag);
  key.count = bag->count;
  key.maxVertCount = packageSize;
  key.bboxVersion = stream->contentVersion();
  // The prim state reaches the block only through the GIFtag, so pack exactly
  // the fields that GIFtag reads. `mapping` is derived the way
  // addStandardBufferDataToPacket derives it, because prim->mapping still
  // holds the PREVIOUS bag's answer at this point.
  const u32 mapping =
      (bag->texture != nullptr && bag->texture->texture != nullptr) ? 1u : 0u;
  key.primKey = static_cast<u32>(prim->type) |
                (static_cast<u32>(prim->shading) << 4) | (mapping << 6) |
                (static_cast<u32>(prim->fogging) << 7) |
                (static_cast<u32>(prim->blending) << 8) |
                (static_cast<u32>(prim->antialiasing) << 9) |
                (static_cast<u32>(prim->mapping_type) << 10) |
                (static_cast<u32>(prim->colorfix) << 11);
  // The Z scale lands in the block's first data quadword and follows the
  // framebuffer's colour depth, which a display-mode switch moves.
  const float depthScale = RendererCoreDepth::scale;
  memcpy(&key.depthScaleBits, &depthScale, sizeof(u32));
  key.singleColor = bag->color->single != nullptr ? 1 : 0;
  key.stripped = bag->stripped ? 1 : 0;
  key.packages =
      static_cast<u16>((bag->count + packageSize - 1) / packageSize);

  retainedCurrent = retained.acquire(key);
  return retainedCurrent != nullptr;
}

void StaPipQBufferRenderer::endRetainedBag() { retainedCurrent = nullptr; }

u32 StaPipQBufferRenderer::takeRetainedHits() { return retained.takeHits(); }
u32 StaPipQBufferRenderer::takeRetainedBuilds() {
  return retained.takeBuilds();
}
u32 StaPipQBufferRenderer::getRetainedBytes() const {
  return retained.getBytes();
}

#else

bool StaPipQBufferRenderer::beginRetainedBag(StaPipBag*, const u32&) {
  return false;
}
void StaPipQBufferRenderer::endRetainedBag() {}
u32 StaPipQBufferRenderer::takeRetainedHits() { return 0; }
u32 StaPipQBufferRenderer::takeRetainedBuilds() { return 0; }
u32 StaPipQBufferRenderer::getRetainedBytes() const { return 0; }

#endif  // TYRA_STAPIP_RETAINED_COMMANDS

// Modified by TyraX: the BAKED VIF STREAM spike - docs/baked-vif-stream.md.
#if TYRA_STAPIP_BAKED_STREAM

/** Read an optional content stamp. A null pointer means the owner does not
 * track its array's contents, and contributes nothing to the key - which is
 * how every caller that has not opted in keeps its previous behaviour. */
static inline u32 stampOf(const u32* p) { return p != nullptr ? *p : 0u; }

// Modified by TyraX: the capture key of a program's entry and kick words - what
// they encode besides the package: the program and its micro-memory address
// (MSCAL), the prim state the GIF tag carries, and the count / scale
// quadword's three scales.
void StaPipQBufferRenderer::entryKeyFor(
    const StaPipVU1Program& program, const StaPipBag* bag,
    u32 out[StaPipCompactStream::kEntryKeyWords]) const {
  const u32 mapping =
      (bag->texture != nullptr && bag->texture->texture != nullptr) ? 1u : 0u;
  out[0] = reinterpret_cast<u32>(&program);
  out[1] = const_cast<StaPipVU1Program&>(program).getDestinationAddress();
  out[2] = static_cast<u32>(prim->type) |
           (static_cast<u32>(prim->shading) << 4) | (mapping << 6) |
           (static_cast<u32>(prim->fogging) << 7) |
           (static_cast<u32>(prim->blending) << 8) |
           (static_cast<u32>(prim->antialiasing) << 9) |
           (static_cast<u32>(prim->mapping_type) << 10) |
           (static_cast<u32>(prim->colorfix) << 11);
  float scales[3] = {RendererCoreDepth::scale, RendererCoreDepth::screenScaleX,
                     RendererCoreDepth::screenScaleY};
  memcpy(&out[3], scales, sizeof(scales));
}

bool StaPipQBufferRenderer::captureEntries(StaPipCompactStream& stream,
                                           const StaPipVU1Program& constProgram,
                                           StaPipBag* bag) {
  if (!stream.allocated()) return false;
  if (captureScratch == nullptr) {
    captureScratch = packet2_create(16, P2_TYPE_NORMAL, P2_MODE_CHAIN, true);
    if (captureScratch == nullptr) return false;
  }
  auto& program = const_cast<StaPipVU1Program&>(constProgram);
  u32 key[StaPipCompactStream::kEntryKeyWords];
  entryKeyFor(program, bag, key);
  StaPipQBuffer scratch;
  scratch.setMaxVertCount(StaPipCompactWriter::kMaxPackage);
  const prim_t savedPrim = *prim;
  bool ok = true;
  for (u32 p = 0; ok && p < stream.packageCount(); ++p) {
    scratch.fillByStream(stream, p);
    scratch.bag = bag;
    packet2_reset(captureScratch, false);
    // The program's own writers - package 0 the bag's first: GS state, MSCAL.
    program.addStandardBufferDataToPacket(captureScratch, &scratch, prim, p == 0);
    if (p == 0)
      packet2_utils_vu_add_start_program(captureScratch,
                                         program.getDestinationAddress());
    else
      packet2_utils_vu_add_continue_program(captureScratch);
    // Exactly a CNT of two quadwords (its VIFcodes STMOD 0 and the UNPACK)
    // and a CNT of none (FLUSH, MSCAL / MSCNT); anything else refuses.
    const qword_t* q = captureScratch->base;
    const u64 entryTag = q[0].dw[0], kickTag = q[3].dw[0];
    ok = packet2_get_qw_count(captureScratch) == 4 &&
         ((entryTag >> 28) & 7) == 1 && (entryTag & 0xFFFF) == 2 &&
         ((kickTag >> 28) & 7) == 1 && (kickTag & 0xFFFF) == 0;
    if (!ok) break;
    // Their words in, each quadword's first slot STMOD 0 (P12).
    const u32 entryCodes[2] = {static_cast<u32>(q[0].sw[2]),
                               static_cast<u32>(q[0].sw[3])};
    const u32 kickCodes[2] = {static_cast<u32>(q[3].sw[2]),
                              static_cast<u32>(q[3].sw[3])};
    stream.setPackageWords(p, entryCodes, &q[1], kickCodes);
  }
  *prim = savedPrim;
  if (!ok) {
    stream.clearPackageWords();  // back to NOPs: never half captured
    return false;
  }
  stream.setEntryKey(key);
  return true;
}

bool StaPipQBufferRenderer::emitFreshEntry(packet2_t* packet,
                                           const StaPipCompactStream& stream,
                                           u32 p, StaPipVU1Program* program,
                                           bool emitState, int clipMask) {
  StaPipQBuffer scratch;
  scratch.setMaxVertCount(StaPipCompactWriter::kMaxPackage);
  scratch.fillByStream(stream, p);
  scratch.bag = const_cast<StaPipBag*>(compactSource.bag);
  scratch.clipPlaneMask = clipMask >= 0 ? static_cast<u8>(clipMask) : 0;
  program->addStandardBufferDataToPacket(packet, &scratch, prim, emitState);
  emitRef(packet, scratch.run[0], scratch.bodyQwords);
  if (emitState)
    packet2_utils_vu_add_start_program(packet, program->getDestinationAddress());
  else
    packet2_utils_vu_add_continue_program(packet);
  return true;
}

// Modified by TyraX: the strip-expansion cache - see the header. The source
// package is one of the bag's stream's; its expansion is a list stream of
// `chunk`-triangle packages, made once per (stream block, version, package).
bool StaPipQBufferRenderer::fillCachedStrip(StaPipQBuffer* buffer,
                                            const StaPipBagPackage& pkg,
                                            u32 first, u32 triangles,
                                            u32 chunk) {
  const StaPipCompactStream* source = compactSource.stream;
  auto* bag = pkg.bag;
  if (!vu1Clipping || source == nullptr || !source->allocated() ||
      triangles == 0 || chunk == 0 || first % chunk != 0 || pkg.size < 3 ||
      bag->billboard != nullptr || repository.hasAnyOverride())
    return false;
  const u32 ps = source->layout().packageSize;
  if (pkg.first % ps != 0 || pkg.size != source->packageVertices(pkg.first / ps))
    return false;
  const u32 allTriangles = pkg.size - 2u;
  const u32 c = first / chunk;
  const u32 inChunk = std::min(chunk, allTriangles - first);
  if (triangles != inChunk) return false;  // a merged range: the pool

  StaPipBakedEntry key;
  key.vertices = source->runHeader(pkg.first / ps, 0);  // one per package
  key.count = allTriangles * 3;
  key.maxVertCount = StaPipBakedStreams::kExpansionKey | (chunk * 3);
  key.contentVersion = source->contentVersion();
  StaPipCompactLayout layout = source->layout();
  layout.stripped = false;
  layout.packageSize = static_cast<u16>(chunk * 3);
  key.layoutKey = layoutKeyOf(layout);
  key.packages = static_cast<u16>((allTriangles + chunk - 1) / chunk);
  StaPipBakedEntry* entry = baked.acquire(key);
  if (entry == nullptr) return false;
  if (entry->pendingPatch) {
    // The source's block was reused for new content (its spare): a fresh
    // expansion - never this one rewritten, a chain may read it.
    baked.dropStreams(*entry);
    entry->pendingPatch = 0;
  }
  if (!entry->complete) {
    StaPipCompactStream& out = entry->stream;
    // An incomplete entry holds no stream (a fresh one, or one whose streams
    // went to the graveyard above): allocate() must never free a block a
    // chain may read. Should one be there, it goes the graveyard's way too.
    TYRA_ASSERT(!out.allocated(), "An incomplete expansion entry holds a stream");
    if (out.allocated()) baked.dropStreams(*entry);
    bool allocated;
    {
#if TYRA_MEMORY_MANAGER
      // Only the allocation under the refusing budget (see pack()).
      Memory::MemScope memoryScope(Memory::Budget::BakedStreams);
#endif
      allocated = out.allocate(layout, allTriangles * 3, source->box());
    }
    if (!allocated) {
      TYRA_CACHE_FALLBACK("expansion: packed into the copy pool", pkg.size);
      return false;
    }
    if (!baked.reserve(out.qwords())) {
      out.release();
      return false;
    }
    // The expanded list, copied element by element out of the source - odd
    // triangles swap their first two vertices, the strip's winding.
    u32 index[StaPipCompactWriter::kMaxPackage];
    for (u32 p = 0; p < out.packageCount(); ++p) {
      const u32 t0 = p * chunk, nt = out.packageVertices(p) / 3;
      for (u32 t = 0; t < nt; ++t)
        for (u32 k = 0; k < 3; ++k) {
          const u32 base = t0 + t;
          const u32 local = ((base & 1U) && k < 2U) ? 1U - k : k;
          index[t * 3 + k] = pkg.first + base + local;
        }
      for (u8 a = 0; a < layout.attrCount; ++a)
        Compact::copyRun(*source, a, index, 0, nt * 3, out.run(p, a));
    }
    out.finish();
    Vif1Queue::registerImmutableSpan(out.data(), out.qwords());  // see pack()
    entry->complete = 1;
#if TYRA_STAPIP_PREPARED_STATIC
    if (preparingStatic) entry->scenePinned = true;
#endif
  }
  buffer->fillByStream(entry->stream, c);
  buffer->bag = bag;
  buffer->clipPlaneMask = pkg.clipPlaneMask;
  buffer->stripped = false;
  // Not the bag's stream: its packages carry no bake index.
  buffer->streamPackage = -1;
  return true;
}

#if TYRA_STAPIP_PREPARED_STATIC
bool StaPipQBufferRenderer::prepareStaticGeometry(StaPipBag* bag,
    u32 packageSize, u32 clipSize) {
  if (!programsPacket || repository.hasAnyOverride() || !clipSize ||
      Vif1Queue::recordingFrame()) return false;
  const StaPipCompactStream* stream = acquireStream(bag, packageSize);
  if (stream == nullptr || sourceEntry == nullptr) return false;
  sourceEntry->scenePinned = true;
  setCompactSource(bag, stream, packageSize, nullptr);
  if (!bag->stripped || !vu1Clipping) return true;
  StaPipQBuffer scratch;
  scratch.setMaxVertCount(packageSize);
  preparingStatic = true;
  const u32 chunk = std::max(1u, clipSize / 3);
  for (u32 offset = 0; offset < bag->count; offset += packageSize) {
    StaPipBagPackage pkg;
    pkg.bag = bag;
    pkg.first = offset;
    pkg.size = static_cast<u16>(std::min(packageSize, bag->count - offset));
    if (pkg.size < 3) continue;
    pkg.clipPlaneMask = 0x3f;
    // This creates no draw. Runtime classification supplies its fresh mask.
    fillCachedStrip(&scratch, pkg, 0, std::min(chunk, u32(pkg.size - 2)), chunk);
  }
  preparingStatic = false;
  return true;
}
#endif

bool StaPipQBufferRenderer::beginBakedBag(StaPipBag* bag,
                                          const u32& packageSize, bool partial) {
  bakedStream = nullptr;
  bakedPartial = partial;
  partialClipProgram = nullptr;
  const StaPipCompactStream* stream = compactSource.stream;
  if (bag == nullptr || stream == nullptr || !stream->allocated() ||
      stream->layout().packageSize != packageSize)
    return false;
  // The exclusions the retained cache and the submission-batch scope make:
  // a billboard expands on VU1 from a per-frame basis, and a game-supplied
  // writer has no packet-size ABI.
  if (bag->billboard != nullptr || repository.hasAnyOverride()) return false;
#if TYRA_STAPIP_BAKED_ANY_VERIFY
  // A VERIFIED bag is drawn the ordinary way: fresh entries, every package.
  if (baked.verifying(sourceEntry)) return false;
#endif
  StaPipVU1Program* program = getCullProgramByBag(bag);
  if (program == nullptr) return false;
  u32 key[StaPipCompactStream::kEntryKeyWords];
  entryKeyFor(*program, bag, key);
  if (!stream->entriesCaptured()) {
    // Never captured, so never REF'd as entries: nothing can be reading them.
    if (!captureEntries(*const_cast<StaPipCompactStream*>(stream), *program,
                        bag))
      return false;
  } else if (memcmp(stream->entryKey(), key, sizeof(key)) != 0) {
    // Captured for another program or state: this draw sends fresh entries.
    return false;
  }
  bakedStream = stream;
  return true;
}

// Modified by TyraX (R5): see the header. The crossing bag's cull packages
// replay the stream's captured entries; its clip packages take the clip
// program's fresh entry and one REF of their runs.
bool StaPipQBufferRenderer::beginPartialReplay(StaPipBag* bag, u32 size) {
  partialClipProgram = nullptr;
  const StaPipCompactStream* stream = bakedStream;
  if (!vu1Clipping || stream == nullptr || !bakedPartial ||
      bag->billboard != nullptr || repository.hasAnyOverride() || size == 0 ||
      stream->qwords() > 0xFFFFu)
    return false;
  // A strip package under three vertices draws nothing and the routed loop
  // skips it; the clip program must never be handed one.
  if (bag->stripped) {
    const u32 tail = bag->count % size;
    if (tail == 1 || tail == 2) return false;
  }
  partialClipProgram = getClipProgramByBag(bag);
  return partialClipProgram != nullptr;
}

// The bag's uniforms are in the packet (sendObjectData), the qbuffer ring is
// empty; this is replayWholeBakedBag's tail for a bag that took no qbuffer.
void StaPipQBufferRenderer::finishPartialReplay() {
  objectDataPending = true;
  ++submissionBatchBags;
  if (!submissionBatchCandidate ||
      submissionBatchBags >= kSubmissionBatchSize)
    flushPendingPacket();
}

bool StaPipQBufferRenderer::emitPartialRuns(const u8* routes, u32 count,
                                            const u8* clipMasks) {
  const StaPipCompactStream* stream = bakedStream;
  StaPipVU1Program* clipProgram = partialClipProgram;
  if (stream == nullptr || clipProgram == nullptr || count == 0 ||
      count > stream->packageCount())
    return false;
  auto* cullProgram = reinterpret_cast<StaPipVU1Program*>(
      static_cast<uintptr_t>(stream->entryKey()[0]));
  // Price it first: a package with a fresh entry costs 5 quadwords (entry,
  // runs REF, kick), a run of captured ones one REF.
  auto* packet = packets[context];
  u32 need = 0;
  {
    u8 last = 0;
    for (u32 i = 0; i < count;) {
      const u8 r = routes[i];
      if (r == 0) { ++i; continue; }
      u32 j = i + 1;
      while (j < count && routes[j] == r) ++j;
      if (r == 2) need += 5 * (j - i);
      else need += (r != last && i > 0) ? 6 : 1;
      last = r;
      i = j;
    }
  }
  if (packet2_get_qw_count(packet) + need + 4 > packetSize) return false;
  auto* mutableStream = const_cast<StaPipCompactStream*>(stream);
  StaPipProgramName lastName = lastProgramName;
  for (u32 i = 0; i < count;) {
    const u8 r = routes[i];
    if (r == 0) { ++i; continue; }
    u32 j = i + 1;
    while (j < count && routes[j] == r) ++j;
    StaPipVU1Program* program = r == 1 ? cullProgram : clipProgram;
    const bool switched = program->getName() != lastName;
    if (r == 2) {
      for (u32 k = i; k < j; ++k)
        emitFreshEntry(packet, *stream, k, program, switched && k == i,
                       clipMasks != nullptr ? clipMasks[k] : 0x3F);
    } else {
      u32 from = i;  // the first package REF'd with its captured entry
      if (switched && i > 0) {
        // Captured as a continuation (no GS-state flag, MSCNT); this run
        // starts after another program, so its first package needs both.
        emitFreshEntry(packet, *stream, i, program, true, -1);
        from = i + 1;
      }
      if (from < j) {
        const u8* at = mutableStream->entry(from);
        const u8* end = mutableStream->kick(j - 1) + 16;
        emitRef(packet, at, static_cast<u32>(end - at) / 16);
      }
    }
    lastName = program->getName();
    i = j;
  }
  lastProgramName = lastName;
  baked.countHit();
#if TYRA_STAPIP_STAMP
  ++partialRunHits;  // takeReplayStats - a stamp-build counter
#endif
  finishPartialReplay();
  return true;
}

bool StaPipQBufferRenderer::replayWholeBakedBag() {
  const StaPipCompactStream* stream = bakedStream;
  if (stream == nullptr || bakedPartial || !stream->entriesCaptured()) return false;
  // A DMA tag's QWC is 16 bits: a larger stream keeps the per-package route
  // (consecutive packages are still one REF each run).
  if (stream->qwords() > 0xFFFFu) return false;
  TYRA_ASSERT((reinterpret_cast<u32>(stream->data()) & 0xFu) == 0,
              "Compact stream is not quadword aligned");

  auto* currentPacket = packets[context];
  // DO NOT FLUSH HERE: this bag's uniforms are already in this packet
  // (sendObjectData), and setSubmissionBatchCandidate reserved this bag's
  // worst case before any of its bytes were appended.
  TYRA_ASSERT(packet2_get_qw_count(currentPacket) + 4 <= packetSize,
              "No room for a baked bag's REF: the per-bag reservation in "
              "setSubmissionBatchCandidate is no longer conservative");

  emitRef(currentPacket, stream->data(), stream->qwords());
  wholeArena = stream->data();  // instancing
  wholeArenaQw = stream->qwords();
  wholeStep = stream->box().step;
#if TYRA_STAPIP_STAMP
  // The head + this REF are now contiguous in the packet, before any flush
  // can send them - the one moment a stamp can be copied.
  if (stampArmed && stampOk) captureStamp();
  stampArmed = false;
#endif

  // The stream's own kicks started the program for every package in it.
  auto* program = reinterpret_cast<StaPipVU1Program*>(
      static_cast<uintptr_t>(stream->entryKey()[0]));
  lastProgramName = program->getName();
#if TYRA_STAPIP_PACKET_PROFILE
  bool emitState = true;
  for (u32 p = 0; p < stream->packageCount(); ++p) {
    recordGsPacket(telemetry, program, stream->packageVertices(p), emitState);
    emitState = false;
  }
#endif

  baked.countHit();
  objectDataPending = true;
  // MANDATORY: flushPendingPacket() resets the packet when submissionBatchBags
  // is 0, which would throw this REF away.
  ++submissionBatchBags;
  if (!submissionBatchCandidate ||
      submissionBatchBags >= kSubmissionBatchSize) {
    flushPendingPacket();
  }
  return true;
}

void StaPipQBufferRenderer::endBakedBag() {
  bakedStream = nullptr;
  bakedPartial = false;
  partialClipProgram = nullptr;
}

#if TYRA_STAPIP_STAMP
// Modified by TyraX: bag stamps (docs/stapip-stamps.md).
//
// The PS2 way to submit a static, wholly visible world bag is to append the
// chain it produced last frame, not to re-derive it: the ordinary path spends
// ~10 000 EE instructions and ~600 cache misses a bag rebuilding a head that
// differs from last frame's in at most the MVP, the fog words and TEX0. A
// stamp keeps those bytes, the facts they were derived from, and re-emits
// them; anything it cannot prove unchanged sends the bag down the ordinary
// path, which re-captures.

StaPipQBufferRenderer::StampEntry* StaPipQBufferRenderer::findStamp(
    const StaPipBag* bag, bool create) {
  if (stamps == nullptr) {
    if (!create) return nullptr;
    stamps = new StampEntry[kStampSlots];
    stampKeys = new const StaPipBag*[kStampSlots]();
    stampBakedOf = new const StaPipBakedEntry*[kStampSlots]();
  }
  // Modified by TyraX: the slot the bag remembers - one compare against the
  // entry the hit reads anyway, no hash and no key line (item "stamp slot").
  if (stampSlotHint && bag->stampSlot >= 0 &&
      stamps[bag->stampSlot].bag == bag)
    return &stamps[bag->stampSlot];
  u32 h = reinterpret_cast<u32>(bag) * 0x9E3779B1U;
  h = (h ^ (h >> 15)) & (kStampSlots - 1);
  // Modified by TyraX: probe the packed keys, touch one entry.
  for (u32 probe = 0; probe < 8; ++probe) {
    const u32 at = (h + probe) & (kStampSlots - 1);
    if (stampKeys[at] == bag) return &stamps[at];
  }
  if (!create) return nullptr;
  StampEntry* oldest = nullptr;
  for (u32 probe = 0; probe < 8; ++probe) {
    const u32 at = (h + probe) & (kStampSlots - 1);
    StampEntry& s = stamps[at];
    if (stampKeys[at] == nullptr) return &s;
    if (oldest == nullptr || s.frame < oldest->frame) oldest = &s;
  }
  return oldest;  // create: recycle the stalest of the eight probes
}

int StaPipQBufferRenderer::stampSlot(const StaPipBag* bag) const {
  if (stampKeys == nullptr) return -1;
  if (stampSlotHint && bag->stampSlot >= 0 &&
      stamps[bag->stampSlot].bag == bag)
    return bag->stampSlot;
  u32 h = reinterpret_cast<u32>(bag) * 0x9E3779B1U;
  h = (h ^ (h >> 15)) & (kStampSlots - 1);
  for (u32 probe = 0; probe < 8; ++probe) {
    const u32 at = (h + probe) & (kStampSlots - 1);
    if (stampKeys[at] == bag) return static_cast<int>(at);
  }
  return -1;
}

bool StaPipQBufferRenderer::hasStamp(const StaPipBag* bag) const {
  return stampEnabled && stampSlot(bag) >= 0;
}

namespace {
inline void prefetchLine(const void* p) {
#ifdef _EE
  asm volatile("pref 0, 0(%0)" : : "r"(p));
#else
  (void)p;
#endif
}
}  // namespace

void StaPipQBufferRenderer::prefetchStamp(const StaPipBag* bag) const {
  const int at = stampSlot(bag);
  if (at < 0) return;
  const u8* e = reinterpret_cast<const u8*>(&stamps[at]);
  // Facts, boxes, matrix and the info copy: the first ~6 lines of the entry.
  for (u32 off = 0; off < 384; off += 64) prefetchLine(e + off);
  prefetchLine(bag);
  prefetchLine(bag->info);
  prefetchLine(bag->info->model);
}

// Drop every stamp whose baked entry was destroyed since the last sweep, before
// anything can read through its raw entry pointer. Rare: entries die only on
// expiry (unused for kLifetimeFrames), eviction and clear().
void StaPipQBufferRenderer::sweepStamps() {
  const u32 gen = baked.eraseGeneration();
  if (gen != stampTableGen) {
    for (u32 i = 0; i < kStampSlots; ++i) {
      stamps[i].bag = nullptr;
      stampKeys[i] = nullptr;
    }
    stampTableGen = gen;
    baked.clearDestroyed();
    return;
  }
  const u32 n = baked.destroyedPending();
  if (n == 0) return;
  for (u32 i = 0; i < kStampSlots; ++i) {
    if (stampKeys[i] == nullptr) continue;
    for (u32 k = 0; k < n; ++k)
      if (stampBakedOf[i] == baked.destroyedAt(k)) {
        stamps[i].bag = nullptr;
        stampKeys[i] = nullptr;
        break;
      }
  }
  baked.clearDestroyed();
}

u32 StaPipQBufferRenderer::stampPrimKey(const StaPipBag* bag) const {
  const u32 mapping =
      (bag->texture != nullptr && bag->texture->texture != nullptr) ? 1u : 0u;
  return static_cast<u32>(prim->type) |
         (static_cast<u32>(prim->shading) << 4) | (mapping << 6) |
         (static_cast<u32>(prim->fogging) << 7) |
         (static_cast<u32>(prim->blending) << 8) |
         (static_cast<u32>(prim->antialiasing) << 9) |
         (static_cast<u32>(prim->mapping_type) << 10) |
         (static_cast<u32>(prim->colorfix) << 11);
}

// World AABB of the transformed object-space box: a superset of the oriented
// box, so "world AABB wholly inside" implies the exact test StaPipCore makes.
void StaPipQBufferRenderer::stampWorldBox(StampEntry& s) {
  const float* m = s.model.data;
  float mn[3] = {1e30F, 1e30F, 1e30F}, mx[3] = {-1e30F, -1e30F, -1e30F};
  for (int c = 0; c < 8; ++c) {
    const float x = (c & 1) ? s.hi.x : s.lo.x;
    const float y = (c & 2) ? s.hi.y : s.lo.y;
    const float z = (c & 4) ? s.hi.z : s.lo.z;
    const float w[3] = {m[0] * x + m[4] * y + m[8] * z + m[12],
                        m[1] * x + m[5] * y + m[9] * z + m[13],
                        m[2] * x + m[6] * y + m[10] * z + m[14]};
    for (int k = 0; k < 3; ++k) {
      if (w[k] < mn[k]) mn[k] = w[k];
      if (w[k] > mx[k]) mx[k] = w[k];
    }
  }
  s.wlo = Vec4(mn[0], mn[1], mn[2], 1.0F);
  s.whi = Vec4(mx[0], mx[1], mx[2], 1.0F);
  // The light pick's sphere, exactly as tryStamp derived it per frame.
  const Vec4 mid(0.5F * (s.lo.x + s.hi.x), 0.5F * (s.lo.y + s.hi.y),
                 0.5F * (s.lo.z + s.hi.z), 1.0F);
  s.worldCenter = s.model * mid;
  const float scale = Math::sqrtNonNegative(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
  const float ex = s.hi.x - s.lo.x, ey = s.hi.y - s.lo.y, ez = s.hi.z - s.lo.z;
  s.worldRadius = 0.5F * Math::sqrtNonNegative(ex * ex + ey * ey + ez * ez) * scale;
  invertAffine(m, s.inv, s.invT);
}

static inline u32 stampContent(const StaPipBag* bag) {
  return stampOf(bag->contentVersion) * 0x9E3779B1U +
         stampOf(bag->color->contentVersion) * 0x85EBCA6BU +
         stampOf(bag->texture != nullptr ? bag->texture->contentVersion
                                         : nullptr) *
             0xC2B2AE35U;
}

void StaPipQBufferRenderer::armStampCapture(StaPipBag* bag, const Vec4& lo,
                                            const Vec4& hi, bool batchSized) {
  stampArmed = stampEnabled;
  stampBag = bag;
  stampBatchSized = batchSized;
  stampLo = lo;
  stampHi = hi;
}

void StaPipQBufferRenderer::captureStamp() {
  // Modified by TyraX: a stamp stands for a bag whose stream is a cache entry
  // (the entry's serial says when the stream moved) or the bag's own stream
  // (StaPipBag::compact - its contentVersion says when it was repacked).
  StaPipBakedEntry* entry = sourceEntry;
  StaPipBag* bag = stampBag;
  if (bag == nullptr || stampHead == nullptr || bakedStream == nullptr) return;
  if (entry != nullptr ? bakedStream != &entry->stream
                       : bakedStream != bag->compact)
    return;
  const StaPipCompactStream* stream = bakedStream;
  const u32 qw = static_cast<u32>(packets[context]->next - stampHead);
  if (qw == 0 || qw > kStampMaxQw || stampMvpAt == 0 || stampOptAt == 0 ||
      (stampSpotActive && stampSpotAt == 0))
    return;
  StampEntry* s = findStamp(bag, true);
  if (s == nullptr) return;
  s->bag = bag;
  stampKeys[s - stamps] = bag;
  stampBakedOf[s - stamps] = entry;
  s->lo = stampLo;
  s->hi = stampHi;
  s->model = *bag->info->model;
  s->modelPtr = bag->info->model;
  stampWorldBox(*s);
  s->vertices = bag->vertices;
  s->texture = bag->texture != nullptr ? bag->texture->texture : nullptr;
  s->info = bag->info;
  memcpy(s->infoBytes, bag->info, sizeof(StaPipInfoBag));
  s->colorSingle = bag->color->single;
  s->sts = bag->texture != nullptr ? bag->texture->coordinates : nullptr;
  s->colors = bag->color->many;
  s->stripped = bag->stripped ? 1 : 0;
  s->compact = bag->compact;
  s->compactVersion =
      bag->compact != nullptr ? bag->compact->contentVersion() : 0;
  s->entry = entry;
  s->arena = stream->data();
  s->step = stream->box().step;
  s->weightScale = stream->weightScale();
  s->count = static_cast<u32>(bag->count);
  s->bboxVersion = bag->bboxVersion;
  s->contentStamp = stampContent(bag);
  s->eraseGen = baked.eraseGeneration();
  s->serial = entry != nullptr ? entry->serial : 0;
  const float depthScale = RendererCoreDepth::scale;
  memcpy(&s->depthScaleBits, &depthScale, sizeof(u32));
  s->residentClasses = residentClasses;
  s->primKey = stampPrimKey(bag);
  s->qw = static_cast<u16>(qw);
  s->mvpAt = stampMvpAt;
  s->optAt = stampOptAt;
  s->tex0At = stampTex0At;
  s->hasTex0 = stampHasTex0;
  s->fogEligible = stampFogEligible;
  s->spotActive = stampSpotActive;
  s->fogOff = stampFogOff;
  s->wrapBlock = stampWrapBlock;
  s->wrap = stampWrap;
  s->singleAt = stampSingleAt;
  s->envAt = stampEnvAt;
  s->batchSized = stampBatchSized ? 1 : 0;
  s->spotAt = stampSpotAt;
  s->program = reinterpret_cast<const StaPipVU1Program*>(
                   static_cast<uintptr_t>(stream->entryKey()[0]))
                   ->getName();
  for (u32 i = 0; i < qw; ++i) s->bytes[i] = stampHead[i];
  s->frame = stampHits + stampCaptures;
  bag->stampSlot = static_cast<s16>(s - stamps);
  ++stampCaptures;
}


bool StaPipQBufferRenderer::tryStamp(StaPipBag* bag, const M4x4& viewProj,
                                     const Plane* planes,
                                     const Plane* guardBandPlanes,
                                     bool dynamicLightReceive,
                                     const RendererCoreSpotLight* noSpot) {
#if TYRA_STAPIP_STAMP_DIAG
  static u32 why[12];
  static u32 tries = 0;
  if (stampEnabled && ++tries % 4000 == 0) {
    printf("STAMPMISS noScope=%u none=%u producer=%u renderer=%u prim=%u "
           "view=%u tex=%u spot=%u wrap=%u\n",
           why[0], why[1], why[2], why[3], why[4], why[5], why[6], why[7],
           why[8]);
    fflush(stdout);
  }
#define STAMP_REJECT(n) do { ++why[n]; return false; } while (0)
#else
#define STAMP_REJECT(n) return false
#endif
  if (!stampEnabled || stamps == nullptr) return false;
  sweepStamps();
  StampEntry* s = findStamp(bag, false);
  if (s == nullptr) STAMP_REJECT(1);
  // Producer-owned facts: the same bag, the same streams, the same versions.
  if (s->vertices != bag->vertices || s->count != static_cast<u32>(bag->count) ||
      s->bboxVersion != bag->bboxVersion || s->info != bag->info ||
      s->modelPtr != bag->info->model || bag->billboard != nullptr ||
      bag->lighting != nullptr ||
      (bag->color->single != nullptr) != (s->singleAt != 0) ||
      (bag->texture != nullptr && bag->texture->coordinatesAreNormals) !=
          (s->envAt != 0) ||
      s->texture != (bag->texture != nullptr ? bag->texture->texture : nullptr) ||
      s->contentStamp != stampContent(bag) ||
      s->sts != (bag->texture != nullptr ? bag->texture->coordinates : nullptr) ||
      s->colors != bag->color->many || s->stripped != (bag->stripped ? 1 : 0) ||
      memcmp(s->infoBytes, bag->info, sizeof(StaPipInfoBag)) != 0)
    STAMP_REJECT(2);
  // Modified by TyraX: the bag's own stream, and its content: a native bag
  // has no float arrays to compare (vertices is null), and a stream repacked
  // under the same pointers - or one that became drawable after the bag was
  // stamped from the cache - moves the version.
  if (s->compact != bag->compact ||
      (bag->compact != nullptr &&
       bag->compact->contentVersion() != s->compactVersion))
    STAMP_REJECT(2);
  // A moved model is not a different submission: MVP, fog, light and spot
  // are re-derived from the live matrix below; only the world box used for
  // visibility has to follow it.
  if (!mmi::equal64(s->model.data, bag->info->model->data)) {  // Modified by TyraX: MMI compare (docs/vu0-macro-mode.md)
    s->model = *bag->info->model;
    stampWorldBox(*s);
  }
  // Renderer-owned facts. eraseGeneration guards the raw entry pointer (an
  // entry was destroyed somewhere); serial/arena say THIS entry was rebuilt.
  float depthScale = RendererCoreDepth::scale;
  u32 depthBits;
  memcpy(&depthBits, &depthScale, sizeof(u32));
  if (depthBits != s->depthScaleBits || residentClasses != s->residentClasses ||
      repository.hasAnyOverride() || vuCustomEnabled)
    STAMP_REJECT(3);
  if (s->entry != nullptr
          ? (s->entry->serial != s->serial ||
             s->entry->stream.data() != s->arena || !s->entry->complete ||
             s->entry->pendingPatch)
          : (bag->compact == nullptr || bag->compact->data() != s->arena ||
             !bag->compact->entriesCaptured()))
    STAMP_REJECT(3);
  // Same prim state as the capture (the baked arena's GIF tags encode it).
  prim->fogging = rendererCore->fog.enabled && !bag->info->fogDisabled
                      ? DRAW_ENABLE
                      : DRAW_DISABLE;
  if ((prim->fogging != DRAW_ENABLE ? 1 : 0) != s->fogOff) STAMP_REJECT(4);
  setInfo(bag->info);
  if (stampPrimKey(bag) != s->primKey) STAMP_REJECT(4);
  // Visibility: the stamp only ever stands for the wholly-inside route - the
  // view frustum, or (StaPipCore's guard-band rule) a box that straddles only
  // the screen edge but lies inside all eight VU1 clip planes.
  {
    const CoreBBoxFrustum f = CoreBBox::frustumCheckAABB(planes, s->wlo, s->whi);
    if (f == OUTSIDE_FRUSTUM) STAMP_REJECT(5);
    if (f != IN_FRUSTUM &&
        (guardBandPlanes == nullptr ||
         CoreBBox::activePlaneMaskAABB(guardBandPlanes, s->wlo, s->whi, 8) != 0))
      STAMP_REJECT(5);
  }
  // Texture: resident (a stamp never triggers an upload), and the bag's
  // wanted wrap chosen exactly as StaPipCore chooses it.
  const Texture* tex = static_cast<const Texture*>(s->texture);
  if (tex != nullptr && !rendererCore->texture.isResident(tex)) STAMP_REJECT(6);
  const texwrap_t& wanted =
      (tex != nullptr && tex->getWrapSettings()->horizontal != WRAP_REPEAT)
          ? *tex->getWrapSettings()
          : RendererCoreGS::repeatWrap();
  if (s->wrapBlock ? !RendererCoreGS::wrapEquals(wanted, s->wrap)
                   : !RendererCoreGS::wrapEquals(wanted,
                                                 RendererCoreGS::repeatWrap()))
    STAMP_REJECT(6);
  // The dynamic light: the same pick StaPipCore makes (world sphere of the
  // box - identity model) and the same zero-influence test sendObjectData
  // makes. The spot block's PRESENCE is layout, so it must match the capture;
  // its numbers are rewritten below from the live light.
  const RendererCoreSpotLight* light = nullptr;
  const bool fastLight = RendererCore::fastLightPick;
  if (dynamicLightReceive && bag->info->dynLightPick) {
    if (fastLight) {
      // Modified by TyraX: the sphere is the stamp's, derived with its world
      // box (stampWorldBox) - the same numbers, not re-derived per frame.
      light = rendererCore->pickDynLight(
          s->worldCenter, s->worldRadius, bag->info->dynLightSkipSlot,
          bag->info->dynLightSkipBaked ? rendererCore->dynBakedMask : 0u,
          bag->info->spotLit, bag->info->torchPerPixel);
    } else {
      const M4x4& m = s->model;
      const Vec4 mid(0.5F * (s->lo.x + s->hi.x), 0.5F * (s->lo.y + s->hi.y),
                     0.5F * (s->lo.z + s->hi.z), 1.0F);
      const Vec4 worldCenter = m * mid;
      const float scale = Math::sqrtNonNegative(
          m.data[0] * m.data[0] + m.data[1] * m.data[1] + m.data[2] * m.data[2]);
      const float ex = s->hi.x - s->lo.x, ey = s->hi.y - s->lo.y,
                  ez = s->hi.z - s->lo.z;
      light = rendererCore->pickDynLight(
          worldCenter,
          0.5F * Math::sqrtNonNegative(ex * ex + ey * ey + ez * ez) * scale,
          bag->info->dynLightSkipSlot,
          bag->info->dynLightSkipBaked ? rendererCore->dynBakedMask : 0u,
          bag->info->spotLit, bag->info->torchPerPixel);
    }
  }
  // Modified by TyraX: as StaPipCore - spotLit = false drops the torch from
  // the pick, not the scene's lights.
  if (!dynamicLightReceive ||
      (!bag->info->spotLit && (!bag->info->dynLightPick || light == nullptr)))
    light = noSpot;
  auto meshSpot =
      fastLight ? buildSpotForBagInverse(light ? *light : rendererCore->spot,
                                         s->inv, s->invT)
                : buildSpotForBag(light ? *light : rendererCore->spot,
                                  bag->info->model);
  if (meshSpot.enabled && stapipSpotHasNoInfluence(meshSpot, s->lo, s->hi))
    meshSpot.enabled = false;
  if ((meshSpot.enabled ? 1 : 0) != s->spotActive) STAMP_REJECT(7);

  // Batch membership exactly as StaPipCore decides it for this bag. This is
  // also the ordinary path's first mutation (it may flush the pending packet),
  // so a rejection below still leaves the ordinary path a valid start.
  // Modified by TyraX: the size cap in batchSized bounds an ORDINARY bag's
  // packet (one command group per package). A stamp appends ~20 qwords and a
  // REF to an immutable arena whatever the bag's size, so only the REPEAT half
  // of the rule still applies: large road and terrain strips used to send one
  // packet per stamp (docs/stapip-stamps.md, "Batching").
  {
    const bool repeatNow =
        tex == nullptr ||
        (tex->getWrapSettings()->horizontal == WRAP_REPEAT &&
         tex->getWrapSettings()->vertical == WRAP_REPEAT);
    setSubmissionBatchCandidate(
        s->batchSized != 0 || (stampBatchAnySize && repeatNow), tex != nullptr);
  }
  // The wrap write is state-dependent: both captured forms assume REPEAT in
  // force, the state the wrap block (or its absence) was recorded under.
  {
    const texwrap_t& inForce =
        packetWrapSet ? packetWrap : rendererCore->gs.currentTextureWrap();
    if (!RendererCoreGS::wrapEquals(RendererCoreGS::repeatWrap(), inForce))
      STAMP_REJECT(8);
  }
#undef STAMP_REJECT

  // ---- committed: from here on this IS the bag's submission ----
  clipper.setSpot(meshSpot);
  ensureProgramSet(false);
  texbuffer_t* tb = nullptr;
  if (tex != nullptr) {
    auto temp = rendererCore->texture.useTexture(tex);
    rendererCore->texture.updateClutBuffer(temp.clut);
    temp.core->info.function = bag->texture->textureFunction;
    tb = temp.core;
  }
  auto* packet = packets[context];
  if (!objectDataPending) {
    startPacket(packet);
    packetWrapSet = false;
  }
  qword_t* dst = packet->next;
  for (u32 i = 0; i < s->qw; ++i) dst[i] = s->bytes[i];
  // MVP: identity model was proven above by the matrix compare at capture.
  // Modified by TyraX: uploaded x D(step) (docs/compact-vertex-format.md).
  const M4x4 mvp = viewProj * s->model;
  {
    alignas(16) M4x4 mvpD;
    mvpDequant(mvp, s->step, &mvpD);
    memcpy(&dst[s->mvpAt + 1], mvpD.data, sizeof(mvpD.data));
  }
  // Fog words (options qword 0), re-derived exactly as sendObjectData does.
  {
    qword_t& q = dst[s->optAt + 1];
    float fogScale = rendererCore->fog.scale;
    float fogOffset = rendererCore->fog.offset;
    bool fogConstant = s->fogOff != 0;  // as sendObjectData: fog off => 255
    if (!fogConstant && s->fogEligible && fogScale < 0.0F) {
      const Vec4& lo = s->lo;
      const Vec4& hi = s->hi;
      const float* m = mvp.data;
      const float cx = 0.5F * (lo.x + hi.x), hx = 0.5F * (hi.x - lo.x);
      const float cy = 0.5F * (lo.y + hi.y), hy = 0.5F * (hi.y - lo.y);
      const float cz = 0.5F * (lo.z + hi.z), hz = 0.5F * (hi.z - lo.z);
      const float maxW = m[3] * cx + m[7] * cy + m[11] * cz + m[15] +
                         fabsf(m[3]) * hx + fabsf(m[7]) * hy +
                         fabsf(m[11]) * hz;
      fogConstant = maxW * fogScale + fogOffset >= 255.0F;
    }
    if (fogConstant) {
      fogScale = 0.0F;
      fogOffset = 255.0F;
    } else {
      u32 bits;
      memcpy(&bits, &fogScale, 4);
      if ((bits & 0xFFFFU) == 0U && bits != 0U) {
        bits |= 1U;
        memcpy(&fogScale, &bits, 4);
      }
    }
    memcpy(&q.sw[2], &fogScale, 4);
    memcpy(&q.sw[3], &fogOffset, 4);
  }
  if (s->hasTex0 && tb != nullptr) {
    const clutbuffer_t* cl = &rendererCore->texture.clut;
    dst[s->tex0At].dw[0] = GS_SET_TEX0(
        tb->address >> 6, tb->width >> 6, tb->psm, tb->info.width,
        tb->info.height, tb->info.components, tb->info.function,
        cl->address >> 6, cl->psm, cl->storage_mode, cl->start,
        cl->load_method);
  }
  if (s->spotActive) {
    // Modified by TyraX: grid units, as sendObjectData (ruling P9).
    alignas(16) float sp[12];
    spotGridConstants(meshSpot, s->step, sp);
    memcpy(&dst[s->spotAt + 1], sp, sizeof(sp));
  }
  if (s->singleAt != 0) {  // as sendObjectData: integer lanes (ruling P8)
    u32 lanes[4];
    singleColorLanes(*bag->color->single, lanes);
    memcpy(&dst[s->singleAt + 1], lanes, 16);
  }
  if (s->envAt != 0) {  // as sendObjectData: the compact env basis
    alignas(16) float e[12];
    envBasis(bag->texture->envRight, bag->texture->envUp, true, e);
    memcpy(&dst[s->envAt + 1], e, sizeof(e));
  }
  packet->next = dst + s->qw;
  if (s->wrapBlock) {  // the re-emitted CLAMP write is now the packet's state
    packetWrap = wanted;
    packetWrapSet = true;
  }

  // The same bookkeeping replayWholeBakedBag does after its REF.
  if (s->entry != nullptr)
    s->entry->framesLeftToDestroy = StaPipBakedStreams::kLifetimeFrames;
  s->frame = stampHits + stampCaptures;  // recency for slot recycling
  lastProgramName = s->program;
  baked.countHit();
  objectDataPending = true;
  ++submissionBatchBags;
  // A stamp is ~20 qwords, not the worst-case 196 the 4-bag cap was sized
  // for: keep packing stamps until the packet itself is nearly full.
  // A later ordinary bag still reserves its own worst case before appending
  // (setSubmissionBatchCandidate), so only the next stamp needs room here.
  if (!submissionBatchCandidate ||
      packet2_get_qw_count(packet) + kStampMaxQw > packetSize)
    flushPendingPacket();
  ++stampHits;
  return true;
}
#endif  // TYRA_STAPIP_STAMP

u32 StaPipQBufferRenderer::takeBakedHits() { return baked.takeHits(); }
u32 StaPipQBufferRenderer::takePackedBags() { return baked.takePackedBags(); }
u32 StaPipQBufferRenderer::takeBakedBuilds() { return baked.takeBuilds(); }
u32 StaPipQBufferRenderer::getBakedBytes() const { return baked.getBytes(); }
u32 StaPipQBufferRenderer::getBakedBudgetBytes() const {
  return baked.getBudgetQwords() * 16;
}
void StaPipQBufferRenderer::setBakedBudgetBytes(u32 bytes) {
  baked.setBudgetQwords(bytes / 16);
}
u32 StaPipQBufferRenderer::takeBakedPatches() { return baked.takePatches(); }
u32 StaPipQBufferRenderer::takeBakedSpareReuses() {
  return baked.takeSpareReuses();
}
u32 StaPipQBufferRenderer::takeBakedFreshArenas() {
  return baked.takeFreshArenas();
}
u32 StaPipQBufferRenderer::takeBakedSpareBusy() {
  return baked.takeSpareBusy();
}
u32 StaPipQBufferRenderer::takeBakedRefusedPatches() {
  return baked.takeRefusedPatches();
}
u32 StaPipQBufferRenderer::takeBakedStaleFrames() {
  return baked.takeStaleFrames();
}
u32 StaPipQBufferRenderer::getBakedLongestStaleRun() const {
  return baked.getLongestStaleRun();
}
u32 StaPipQBufferRenderer::getBakedGraveyardBytes() const {
  return baked.getGraveyardBytes();
}
const u32* StaPipQBufferRenderer::getBakedMisses() const {
  return baked.getMisses();
}
u32 StaPipQBufferRenderer::getBakedLoudCount() const {
  return baked.getLoudCount();
}
u32 StaPipQBufferRenderer::getBakedLoudPackages() const {
  return baked.getLoudPackages();
}
u32 StaPipQBufferRenderer::getBakedLoudReason() const {
  return baked.getLoudReason();
}
void StaPipQBufferRenderer::clearBakedMisses() { baked.clearMisses(); }
#if TYRA_STAPIP_BAKED_ANY_VERIFY
u32 StaPipQBufferRenderer::getVerifyChecked() const {
  return baked.verifyChecked;
}
u32 StaPipQBufferRenderer::getVerifyFailed() const {
  return baked.verifyFailed;
}
#else
u32 StaPipQBufferRenderer::getVerifyChecked() const { return 0; }
u32 StaPipQBufferRenderer::getVerifyFailed() const { return 0; }
#endif

#else

bool StaPipQBufferRenderer::fillCachedStrip(StaPipQBuffer*,
    const StaPipBagPackage&, u32, u32, u32) { return false; }
bool StaPipQBufferRenderer::captureEntries(StaPipCompactStream&,
    const StaPipVU1Program&, StaPipBag*) { return false; }

bool StaPipQBufferRenderer::beginBakedBag(StaPipBag*, const u32&, bool) {
  return false;
}
bool StaPipQBufferRenderer::replayWholeBakedBag() { return false; }
bool StaPipQBufferRenderer::beginPartialReplay(StaPipBag*, u32) { return false; }
bool StaPipQBufferRenderer::emitPartialRuns(const u8*, u32, const u8*) { return false; }
void StaPipQBufferRenderer::endBakedBag() {}
u32 StaPipQBufferRenderer::takeBakedHits() { return 0; }
u32 StaPipQBufferRenderer::takePackedBags() { return 0; }
u32 StaPipQBufferRenderer::takeBakedBuilds() { return 0; }
u32 StaPipQBufferRenderer::getBakedBytes() const { return 0; }
u32 StaPipQBufferRenderer::getBakedBudgetBytes() const { return 0; }
void StaPipQBufferRenderer::setBakedBudgetBytes(u32) {}
u32 StaPipQBufferRenderer::takeBakedPatches() { return 0; }
u32 StaPipQBufferRenderer::takeBakedSpareReuses() { return 0; }
u32 StaPipQBufferRenderer::takeBakedFreshArenas() { return 0; }
u32 StaPipQBufferRenderer::takeBakedSpareBusy() { return 0; }
u32 StaPipQBufferRenderer::takeBakedRefusedPatches() { return 0; }
u32 StaPipQBufferRenderer::takeBakedStaleFrames() { return 0; }
u32 StaPipQBufferRenderer::getBakedLongestStaleRun() const { return 0; }
u32 StaPipQBufferRenderer::getBakedGraveyardBytes() const { return 0; }
const u32* StaPipQBufferRenderer::getBakedMisses() const { return nullptr; }
u32 StaPipQBufferRenderer::getBakedLoudCount() const { return 0; }
u32 StaPipQBufferRenderer::getBakedLoudPackages() const { return 0; }
u32 StaPipQBufferRenderer::getBakedLoudReason() const { return 0; }
void StaPipQBufferRenderer::clearBakedMisses() {}
u32 StaPipQBufferRenderer::getVerifyChecked() const { return 0; }
u32 StaPipQBufferRenderer::getVerifyFailed() const { return 0; }

#endif  // TYRA_STAPIP_BAKED_STREAM

// Modified by TyraX: one buffer's commands - the program's entry (count /
// scale, GIF tag) and its compact runs. The runs of a buffer lie back to back
// (StaPipQBuffer::bodyQwords), so they go as ONE REF; a float buffer (the
// EE clipper's output) and a game-supplied program use the program's writers.
void StaPipQBufferRenderer::addBufferCommands(packet2_t* packet,
                                              StaPipQBuffer* buffer,
                                              StaPipVU1Program* program,
                                              bool emitState) {
  if (buffer->bodyQwords != 0 && !repository.hasAnyOverride()) {
    program->addStandardBufferDataToPacket(packet, buffer, prim, emitState);
    emitRef(packet, buffer->run[0], buffer->bodyQwords);
  } else {
    program->addBufferDataToPacket(packet, buffer, prim, emitState);
  }
}

void StaPipQBufferRenderer::addBuffersDataToPacket(const u32& from,
                                                   const u32& to,
                                                   const bool& finalize) {
  HardwareTrace::Scope trace("Packet_build");
  const u32 buildStart = telemetry ? readTelemetryTicks() : 0;
  auto* currentPacket = packets[context];
  if (!objectDataPending) startPacket(currentPacket);

  for (u32 i = from; i < to; i++) {
    if (!buffers[i]->any()) continue;

    auto* program = dBufferPrograms[i];
    const bool emitState = lastProgramName != program->getName();

    // Modified by TyraX: every vertex that actually reaches a VU1 buffer,
    // counted where it is packetised rather than where it was classified -
    // the number the EE's per-package bill scales with. See StaPipTelemetry.
    if (telemetry) telemetry->verticesSubmitted += buffers[i]->size;
#if TYRA_STAPIP_PACKET_PROFILE
    if (telemetry) {
      recordGsPacket(telemetry, program, buffers[i]->size, emitState);
    }
#endif

    // Modified by TyraX: with the EE clipper, the as_is programs read other
    // forms of two uniforms than the compact programs (emitUniformForm).
    if (asIsSwapNeeded) {
      const bool asIs = isAsIsProgram(program->getName());
      if (asIs != asIsFormLoaded) {
        emitUniformForm(currentPacket, asIs);
        asIsFormLoaded = asIs;
      }
    }

#if TYRA_STAPIP_BAKED_STREAM
    // Modified by TyraX: the compact stream's captured entries. A run of
    // consecutive packages of the bag's stream is ONE DMA REF - entry to
    // kick, the stream holds them in package order - carrying the unpacks,
    // the runs AND the MSCAL/MSCNT of every package in it. The run stops at
    // the flush boundary `to`. A VERIFIED bag never gets here (beginBakedBag
    // refused it a stream to replay).
    {
      const StaPipCompactStream* stream = bakedStream;
      const int bakeIdx = buffers[i]->bakeIndex;
      if (!bakedPartial && stream != nullptr && bakeIdx >= 0 &&
          buffers[i]->streamPackage == bakeIdx &&
          static_cast<u32>(bakeIdx) < stream->packageCount()) {
        u32 next = i + 1;
        u32 last = static_cast<u32>(bakeIdx);
        while (next < to && last + 1 < stream->packageCount() &&
               buffers[next]->any() &&
               buffers[next]->bakeIndex == static_cast<int>(last + 1) &&
               buffers[next]->streamPackage == static_cast<int>(last + 1)) {
          if (telemetry) telemetry->verticesSubmitted += buffers[next]->size;
#if TYRA_STAPIP_PACKET_PROFILE
          if (telemetry) {
            recordGsPacket(telemetry, dBufferPrograms[next],
                           buffers[next]->size, false);
          }
#endif
          ++last;
          ++next;
        }
        auto* mutableStream = const_cast<StaPipCompactStream*>(stream);
        const u8* at = mutableStream->entry(static_cast<u32>(bakeIdx));
        const u8* end = mutableStream->kick(last) + 16;
        emitRef(currentPacket, at, static_cast<u32>(end - at) / 16);
        // The stream's kicks already started the program.
        lastProgramName = program->getName();
        baked.countHit();
        i = next - 1;  // the for() increments
        continue;
      }
    }
#endif

#if TYRA_STAPIP_PROBE_UNCACHED_CHAIN
    // Probe B: one buffer whose streams live in the copy pool is enough to
    // make this whole chain need the write-back. See stapip_probes.hpp.
    if (buffers[i]->probeCopyFilled) probePacketUsesPool = true;
#endif

#if TYRA_STAPIP_RETAINED_COMMANDS
    // Modified by TyraX: retained command data. The block is the bag's
    // geometry commands for THIS package - identical every frame while the key
    // holds - so a hit is a memcpy where the miss below runs the scale
    // quadword, the prim GIFtag and one DMA REF per stream. It is captured out
    // of the packet the miss just wrote, which is what makes the two paths
    // byte-identical by construction rather than by inspection.
    StaPipRetainedEntry* entry = retainedCurrent;
    const int retainIdx = buffers[i]->retainIndex;
    // Modified by TyraX: only a package filled in place out of the bag's
    // stream - the block's REF names it.
    const bool retainable = entry != nullptr && retainIdx >= 0 &&
                            retainIdx < entry->packages &&
                            buffers[i]->streamPackage == retainIdx;
    if (retainable && entry->ready[retainIdx]) {
      appendChainQwords(currentPacket,
                        &entry->data[static_cast<u32>(retainIdx) *
                                     StaPipRetainedCommands::kMaxBlockQw],
                        entry->blockQw);
      // Modified by TyraX: re-flag the replayed header for THIS position in
      // the packet. Its packed count carries VU1_STAPIP_EMIT_STATE_FLAG -
      // "(re)send the GS state" - which is set only when the buffer's program
      // differs from the previous buffer's, so it is a property of where the
      // block lands, not of the block. Replaying a block captured mid-run
      // (flag clear) right after a different program - a stripped bag's
      // partially visible package goes to the clip program as an expanded
      // LIST - told VU1 to keep the GS state that list left behind, and the
      // next STRIP was drawn as a triangle list: road packages full of holes
      // and slivers, stable per camera, in PCSX2 and on hardware alike
      // (docs/roads.md, "Holes in the road").
      {
        u32* packed =
            reinterpret_cast<u32*>(currentPacket->next - entry->blockQw) +
            StaPipVU1Program::kPackedCountWord;
        if (program->emitsStateFlag(emitState))
          *packed |= VU1_STAPIP_EMIT_STATE_FLAG;
        else
          *packed &= ~static_cast<u32>(VU1_STAPIP_EMIT_STATE_FLAG);
      }
      retained.countHit();
    } else {
      const u32 blockStart = packet2_get_qw_count(currentPacket);
      addBufferCommands(currentPacket, buffers[i], program, emitState);
      retained.countBuild();
      if (retainable) {
        const u32 blockQw = packet2_get_qw_count(currentPacket) - blockStart;
        // A block that does not fit, or that disagrees with the length this
        // bag's other packages produced, is simply never retained - the entry
        // keeps rebuilding, which is slower and always right.
        if (blockQw > 0 && blockQw <= StaPipRetainedCommands::kMaxBlockQw &&
            (entry->blockQw == 0 || entry->blockQw == blockQw)) {
          entry->blockQw = static_cast<u16>(blockQw);
          memcpy(&entry->data[static_cast<u32>(retainIdx) *
                              StaPipRetainedCommands::kMaxBlockQw],
                 reinterpret_cast<const u8*>(currentPacket->base) +
                     blockStart * 16,
                 blockQw * 16);
          TYRA_ASSERT(
              (reinterpret_cast<const u32*>(currentPacket->base +
                                            blockStart)[StaPipVU1Program::
                                                            kPackedCountWord] &
               VU1_STAPIP_COUNT_MASK) == buffers[i]->size,
              "Retained block header moved: the replay re-flags the packed "
              "count at StaPipVU1Program::kPackedCountWord");
          entry->ready[retainIdx] = 1;
        }
      }
    }
#else
    addBufferCommands(currentPacket, buffers[i], program, emitState);
#endif

    Verbose("Send ", program->getStringName(), "[", i, "]: ", buffers[i]->size);

    if (lastProgramName != program->getName()) {
      packet2_utils_vu_add_start_program(currentPacket,
                                         program->getDestinationAddress());
      lastProgramName = program->getName();
    } else {
      packet2_utils_vu_add_continue_program(currentPacket);
    }

  }

  if (finalize) addEndTag(currentPacket);
  if (telemetry) telemetry->packetBuildTicks += readTelemetryTicks()-buildStart;
}

void StaPipQBufferRenderer::beginSubmissionBatch() {
  TYRA_ASSERT(!submissionBatchScope, "Submission batch is already open");
  submissionBatchScope = true;
}

void StaPipQBufferRenderer::endSubmissionBatch() {
  TYRA_ASSERT(submissionBatchScope, "Submission batch is not open");
  flushPendingPacket();
  submissionBatchScope = false;
  submissionBatchCandidate = false;
}

void StaPipQBufferRenderer::onFrameEnd() {
  // Modified by TyraX: fail safe for a mismatched caller scope. Never carry
  // DMA references into the next frame, where retained source arrays may be
  // updated before VIF1 consumes them.
  flushPendingPacket();
  TYRA_ASSERT(!submissionBatchScope,
              "Submission batch must end before the frame ends");
  submissionBatchScope = false;
  submissionBatchCandidate = false;
#if TYRA_STAPIP_RETAINED_COMMANDS
  retainedCurrent = nullptr;
  retained.onFrameEnd();
#endif
#if TYRA_STAPIP_BAKED_STREAM
  bakedStream = nullptr;
  sourceEntry = nullptr;
  baked.onFrameEnd();
#endif
}

void StaPipQBufferRenderer::setSubmissionBatchCandidate(const bool& candidate,
                                                        const bool& textured) {
  const bool enabled = submissionBatchScope && candidate;
  if (!enabled && objectDataPending) flushPendingPacket();
  // Modified by TyraX: reserve a complete old-style 32-group packet before
  // appending any bytes. The candidate currently caps at 15 groups, but this
  // conservative bound remains valid if that policy changes. Checking after
  // construction is too late: packet2 would already have written past base.
  if (enabled && objectDataPending &&
      packet2_get_qw_count(packets[context]) + kWorstBagPacketSize >
          packetSize)
    flushPendingPacket();
  submissionBatchCandidate = enabled;
  if (enabled && textured) submissionPacketHasTexture = true;
  if (enabled && telemetry) ++telemetry->submissionBatchEligibleBags;
}

void StaPipQBufferRenderer::flushPendingPacket() {
  if (!objectDataPending) return;
  if (submissionBatchBags == 0) {
    packet2_reset(packets[context], false);
    packetWrapSet = false;  // Modified by TyraX: its wrap write goes too
    objectDataPending = false;
    submissionPacketHasTexture = false;
    return;
  }
  if (HardwareTrace::active && HardwareTrace::detailed && submissionBatchBags > 0) {
    const u32 now = HardwareTrace::ticks();
    HardwareTrace::record("Submission_batch", now, now, submissionBatchBags);
  }
  if (telemetry && submissionBatchBags > 0) {
    telemetry->submissionBatchedBags += submissionBatchBags;
    ++telemetry->submissionBatchPackets;
  }
  addEndTag(packets[context]);
  sendPacket();
  submissionBatchBags = 0;
}

// Modified by TyraX: instancing - see the header. Bounded by the packet, not by
// kSubmissionBatchSize: each copy is seven quadwords whatever the mesh holds,
// so ~110 copies share one chain and one VIF1 kick.
void StaPipQBufferRenderer::appendBakedInstance(const M4x4& mvp,
                                                bool textured) {
  constexpr u32 kInstanceQw = 7;  // FLUSHE + MVP header + 4 MVP + REF
  TYRA_ASSERT(wholeArena != nullptr, "appendBakedInstance with no arena");
  if (objectDataPending &&
      packet2_get_qw_count(packets[context]) + kInstanceQw + 1 > packetSize)
    flushPendingPacket();
  auto* packet = packets[context];
  if (!objectDataPending) {
    startPacket(packet);
    packetWrapSet = false;
    objectDataPending = true;
  }
  alignas(16) M4x4 m;  // emitUnpack copies whole qwords
  mvpDequant(mvp, wholeStep, &m);  // Modified by TyraX: x D(step)
  emitQword(packet, flusheHeader());
  emitUnpack(packet, VU1_MVP_MATRIX_ADDR, m.data, 4);
  emitRef(packet, wholeArena, wholeArenaQw);
  if (textured) submissionPacketHasTexture = true;
  // flushPendingPacket throws an unsent packet away when this is 0.
  if (submissionBatchBags < 255) ++submissionBatchBags;
}

void StaPipQBufferRenderer::finishInstances() { flushPendingPacket(); }

void StaPipQBufferRenderer::textureMutationBarrier(void* context) {
  static_cast<StaPipQBufferRenderer*>(context)->beforeTextureMutation();
}

void StaPipQBufferRenderer::beforeTextureMutation() {
#if TYRA_STAPIP_VIFHASH
  // Modified by TyraX: the gate's second input. This barrier is the ONLY thing
  // ordering a texture upload (GIF channel) against the draws that read it, and
  // it works by flushing the pending packet - so a redesign that moves the
  // flush cadence can move an upload between a different pair of draws. Folding
  // a marker here in sequence makes the hash model this pipeline's whole
  // contribution to GS state as an ordered sequence of draws and texture
  // changes. Folded BEFORE the flush, so it lands between the draws it
  // separates. docs/baked-stream-acceptance-gate.md.
  vifHash.foldTextureMutation(
      submissionTextureReadersOutstanding ? 1u : 0u);
#endif
  flushPendingPacket();
  if (!submissionTextureReadersOutstanding) return;
  rendererCore->sync.align3D();
  submissionTextureReadersOutstanding = false;
}

// Modified by TyraX (R3): an empty packet is about to be written. While a
// frame is being recorded it is written straight into the frame bank (see
// packetOrigin); otherwise, or when the bank refuses, into its own buffer.
// Only ever re-pointed while EMPTY - everything that remembers a position in
// the packet (stampHead, the retained/baked captures) is set after this.
void StaPipQBufferRenderer::startPacket(packet2_t* packet) {
  packet2_reset(packet, false);
#if TYRA_VIF1_QUEUE && TYRA_VIF1_OWNED_PACKETS && !TYRA_STAPIP_PROBE_UNCACHED_CHAIN
  void* owned = Vif1Queue::reservePacket(static_cast<u32>(packetSize) * 16,
                                         packet->base);
  qword_t* base = owned ? static_cast<qword_t*>(owned) : packetOrigin[context];
  packet->base = packet->next = base;
#endif
}

void StaPipQBufferRenderer::sendPacket() {
  auto* currentPacket = packets[context];

  TYRA_ASSERT(packet2_get_qw_count(currentPacket) <= packetSize,
              "Packet is too big. Internal error");

#if TYRA_VIF1_QUEUE
  // Modified by TyraX: no wait for the previous chain here - it may still be
  // running, and the next one may be queued behind it (vif1_queue.hpp). The
  // wait that protects buffer reuse happens after the context flip below, for
  // the one buffer about to be rewritten.
  if (telemetry != nullptr) ++telemetry->packetFlushes;
#else
  const u32 waitStart = telemetry != nullptr ? readTelemetryTicks() : 0;
  { HardwareTrace::Scope trace("VIF1_DMA_wait");
    HardwareTrace::state("VIF1_WAIT_START");
    frameWaitVif();
    HardwareTrace::state("VIF1_WAIT_END"); }
  if (telemetry != nullptr) {
    ++telemetry->packetFlushes;
    telemetry->vu1WaitTicks += readTelemetryTicks() - waitStart;
  }
#endif
  // Added by TyraX: this wait was inside no bracket at all, so it was
  // indistinguishable from `dispatch`'s package creation and classification.
  // Opt-in only - see stapip_attrib.hpp.
#if TYRA_STAPIP_ATTRIB
  const u32 gifWaitStart = telemetry != nullptr ? readTelemetryTicks() : 0;
#endif
  { HardwareTrace::Scope trace("GIF_DMA_wait");
    frameWaitGif(DMA_CHANNEL_GIF, 0); }  // Wait for texture. Issue #182.
#if TYRA_STAPIP_ATTRIB
  if (telemetry != nullptr)
    telemetry->attrib.gifWaitTicks += readTelemetryTicks() - gifWaitStart;
#endif

  // TyraX: the VU1 packet tap (docs/devkit.md). Null in any build whose devkit
  // layer does not exist, so this is one load + branch per bag flush and the
  // capture code is not linked at all.
  if (g_vuPacketHook)
    g_vuPacketHook(currentPacket->base, packet2_get_qw_count(currentPacket),
                   "");  // the program is in the packet's MSCAL address

#if TYRA_STAPIP_VIFHASH
  // Modified by TyraX: the acceptance gate, leg 1. Same seam as the devkit tap
  // and for the same reason - the chain is finished and the bytes here are the
  // bytes the DMAC is about to fetch - but it decodes and folds ON THE CONSOLE
  // instead of shipping a capture to the host, because a capture is one flush
  // of the ~120 a frame and its buffers truncate silently.
  // docs/baked-stream-acceptance-gate.md.
  vifHash.foldChain(currentPacket->base, packet2_get_qw_count(currentPacket));
#endif

#if TYRA_STAPIP_PACKET_PROFILE
  recordVifChain(telemetry, packetTelemetryProducer, currentPacket->base,
                 packet2_get_qw_count(currentPacket));
#endif

  // dma_wait_fast(); // This have no impact on performance

  const u32 submitStart = telemetry != nullptr ? readTelemetryTicks() : 0;
  { HardwareTrace::Scope trace("VIF1_submit");
    if (HardwareTrace::active && HardwareTrace::detailed) { const u32 t=HardwareTrace::ticks(); HardwareTrace::record("Packet_qwords", t, t, packet2_get_qw_count(currentPacket)); }
    // Modified by TyraX: the DMA chain quadwords this pipeline hands VIF1 -
    // the number the baked VIF stream exists to move. Compiled into both arms.
    if (telemetry) chainQwords += packet2_get_qw_count(currentPacket);
#if TYRA_STAPIP_PROBE_UNCACHED_CHAIN
    // Probe B (stapip_probes.hpp): the packet itself is uncached, so nothing
    // of IT needs writing back - but a chain whose REF tags name the qbuffer
    // copy pool still does, and that half is bounded rather than assumed.
    const bool probeFlush = probePacketUsesPool || TYRA_STAPIP_PROBE_FORCE_FLUSH;
    if (telemetry != nullptr) {
      if (probeFlush) ++telemetry->probeFlushedSends;
      else ++telemetry->probeUnflushedSends;
    }
#if TYRA_STAPIP_PROBE_LOG_SENDS
    {
      // Scene property, read in PCSX2. Never enable this for a console timing
      // arm - see TYRA_STAPIP_PROBE_LOG_SENDS in stapip_probes.hpp.
      static u32 probeKept = 0, probeDropped = 0;
      if (probeFlush) ++probeKept; else ++probeDropped;
      if ((probeKept + probeDropped) % 3000 == 0)
        TYRA_LOG("PROBEB: sends kept(flush)=", probeKept,
                 " dropped(no flush)=", probeDropped);
    }
#endif
    frameSendPacket(currentPacket, DMA_CHANNEL_VIF1, probeFlush);
    probePacketUsesPool = false;
#elif TYRA_VIF1_QUEUE
    // The same write-back dma_channel_send_packet2 performs, then the chain
    // goes to the queue instead of straight to the channel.
#if !TYRA_VIF1_QUEUE_LAZY_FLUSH
    FlushCache(0);
#endif
    bool sourceCopied = false;
#if TYRA_VIF1_OWNED_PACKETS
    // Modified by TyraX (R3): a packet built in the frame bank is handed over
    // where it lies; its own buffer was never touched, so nothing to wait for.
    const u32 sequence =
        currentPacket->base != packetOrigin[context]
            ? Vif1Queue::submitOwned(currentPacket->base,
                                     packet2_get_qw_count(currentPacket),
                                     &sourceCopied)
            : Vif1Queue::submit(currentPacket->base,
                                packet2_get_qw_count(currentPacket),
                                &sourceCopied);
#else
    const u32 sequence = Vif1Queue::submit(
        currentPacket->base, packet2_get_qw_count(currentPacket), &sourceCopied);
#endif
    packetSequence[context] = sourceCopied ? 0 : sequence;
#else
    frameSendPacket(currentPacket, DMA_CHANNEL_VIF1, true);
#endif
  }
  // Modified by TyraX: the packet's wrap write is on its way (setBagWrap).
  if (packetWrapSet) {
    rendererCore->gs.noteTextureWrap(packetWrap);
    packetWrapSet = false;
  }
  if (submissionPacketHasTexture)
    submissionTextureReadersOutstanding = true;
  submissionPacketHasTexture = false;
  if (telemetry) telemetry->dmaSubmitTicks += readTelemetryTicks()-submitStart;
  objectDataPending = false;

  // TyraX: with a VU1 memory hook installed (a devkit capture is in flight),
  // wait for the transfer AND for VU1 to finish its microprogram, then hand the
  // whole of VU1 data memory over. This stalls the pipeline on purpose - it runs
  // for the one frame a capture was armed for, never otherwise.
  if (g_vuMemHook) {
    { HardwareTrace::Scope trace("VIF1_DMA_wait");
    HardwareTrace::state("VIF1_WAIT_START");
    Vif1Queue::drain();
    HardwareTrace::state("VIF1_WAIT_END"); }
    // VIF1_STAT: VPS (bits 0-1) = VIF status, VEW (bit 2) = waiting for VU1.
    volatile u32* const vif1Stat = (volatile u32*)0x10003c00;
    for (int spin = 0; spin < 2000000 && (*vif1Stat & 0x7) != 0; ++spin) {
    }
    g_vuMemHook((const void*)0x1100c000, 1024 * 16);
  }

  // Switch packet, so we can proceed during DMA transfer - and switch the
  // slots' copy pools with it, they are referenced by the packet just sent.
#if TYRA_VIF1_QUEUE
  // Modified by TyraX: rotate through kPacketCount buffers. The buffer (and the
  // copy-pool side that goes with it) is rewritten from here on, so wait for
  // the chain that last read it - usually long finished, since kPacketCount - 1
  // newer chains were queued behind it. This is the queue's only EE wait, and
  // it is what vu1WaitTicks reports in this arm.
  context = static_cast<u8>((context + 1) % kPacketCount);
  {
    const u32 waitStart = telemetry != nullptr ? readTelemetryTicks() : 0;
    HardwareTrace::Scope trace("VIF1_DMA_wait");
    HardwareTrace::state("VIF1_WAIT_START");
    if (packetSequence[context]) Vif1Queue::waitFor(packetSequence[context]);
    HardwareTrace::state("VIF1_WAIT_END");
    if (telemetry != nullptr)
      telemetry->vu1WaitTicks += readTelemetryTicks() - waitStart;
  }
#else
  context = !context;
#endif
  StaPipQBuffer::flipPoolSide();
}

// Modified by TyraX: propagate only when the value actually MOVES.
//
// StaPipCore calls this once per bag, inside the `bounds` bracket, and it fans
// the same u32 out to all 32 qbuffers plus the clipper - 33 out-of-line stores
// per bag, 7 474 per garage-day frame, to write the number that was already
// there. Measured by the attribution pass: the pin-and-store half of `bdSize`
// is 0.448 ms of a 2.016 ms `bounds` (docs/render-submission-attribution.md),
// while resolving the program is 0.072 and the integer divisions in
// getMaxVertCount are 0.057 (measured when there were three of them; the
// rounding step is one division now, so this is an upper bound). The package
// size is a property of the PROGRAM
// CLASS, so consecutive bags of one class - which is most of a frame - ask for
// the size that is already set.
//
// Every leaf setter here is a pure store (StaPipQBuffer, StaPipClipper), so
// skipping a no-op propagation is exact rather than approximate.
//
// THE INVARIANT THIS DEPENDS ON: `maxVertCount` must equal what the buffers
// hold. A fresh `allocateOnUse()` builds new StaPipQBuffers whose own
// maxVertCount is not yet meaningful, so that function resets this field to 0
// - never a legal package size, every derived one being a multiple of 3 and at
// least 3 - which forces the next bag to propagate. Without that reset a scene
// reload whose first bag happened to want the previous scene's size would skip
// the propagation and leave 32 buffers holding a stale number.
void StaPipQBufferRenderer::setMaxVertCount(const u32& count) {
  if (maxVertCount == count) return;
  maxVertCount = count;
  for (u32 i = 0; i < buffersCount; i++) {
    buffers[i]->setMaxVertCount(count);
  }
  clipper.setMaxVertCount(count);
}

StaPipVU1Program* StaPipQBufferRenderer::getAsIsProgramByBag(
    const StaPipBag* bag) {
  auto programType = getDrawProgramTypeByBag(bag);

  // Modified by TyraX: billboard bags always use the billboard programs
  // (per-quad culling happens on VU1; there is no as_is/clip variant).
  if (programType == StaPipVU1BillboardTexture)
    return getProgramByName(StaPipBillboardTexture);
  else if (programType == StaPipVU1BillboardColor)
    return getProgramByName(StaPipBillboardColor);

  if (programType == StaPipVU1TextureDirLights)
    return getProgramByName(StaPipAsIsTextureDirLights);
  else if (programType == StaPipVU1DirLights)
    return getProgramByName(StaPipAsIsDirLights);
  else if (programType == StaPipVU1TextureEnvColor)  // TyraX: env (matcap)
    return getProgramByName(StaPipAsIsTextureEnv);
  else if (programType == StaPipVU1TextureColor)
    return getProgramByName(StaPipAsIsTextureColor);
  else
    return getProgramByName(StaPipAsIsColor);
}

// Modified by TyraX: VU1 clipping.
StaPipVU1Program* StaPipQBufferRenderer::getClipProgramByBag(
    const StaPipBag* bag) {
  auto programType = getDrawProgramTypeByBag(bag);

  // Modified by TyraX: billboard bags always use the billboard programs.
  if (programType == StaPipVU1BillboardTexture)
    return getProgramByName(StaPipBillboardTexture);
  else if (programType == StaPipVU1BillboardColor)
    return getProgramByName(StaPipBillboardColor);

  if (programType == StaPipVU1TextureDirLights)
    return getProgramByName(StaPipClipTextureDirLights);
  else if (programType == StaPipVU1DirLights)
    return getProgramByName(StaPipClipDirLights);
  else if (programType == StaPipVU1TextureEnvColor)  // TyraX: env (matcap)
    return getProgramByName(StaPipClipTextureEnv);
  else if (programType == StaPipVU1TextureColor)
    return getProgramByName(StaPipClipTextureColor);
  else
    return getProgramByName(StaPipClipColor);
}

StaPipVU1Program* StaPipQBufferRenderer::getCullProgramByBag(
    const StaPipBag* bag) {
  auto programType = getDrawProgramTypeByBag(bag);
  return getCullProgramByType(programType);
}

StaPipVU1Program* StaPipQBufferRenderer::getProgramByName(
    const StaPipProgramName& name) {
  // Modified by TyraX: a class the project dropped has no program on VU1, and
  // MSCAL-ing to an address nothing was uploaded to draws garbage. Walk down to
  // a resident relative instead - the mesh loses a feature, not the frame.
  StaPipProgramName resolved = name;
  for (int guard = 0; guard < 4; ++guard) {
    const StaPipProgramName next = residentFallback(resolved);
    if (next == resolved) break;
    resolved = next;
  }
  return repository.getProgram(resolved);
}

StaPipVU1Program* StaPipQBufferRenderer::getCullProgramByParams(
    const bool& isLightingEnabled, const bool& isTextureEnabled) {
  auto type = getDrawProgramTypeByParams(isLightingEnabled, isTextureEnabled);
  return getCullProgramByType(type);
}

StaPipVU1Program* StaPipQBufferRenderer::getCullProgramByType(
    const StaPipProgramType& programType) {
  // Modified by TyraX: billboard bags always use the billboard programs.
  if (programType == StaPipVU1BillboardTexture)
    return getProgramByName(StaPipBillboardTexture);
  else if (programType == StaPipVU1BillboardColor)
    return getProgramByName(StaPipBillboardColor);

  if (programType == StaPipVU1TextureDirLights)
    return getProgramByName(StaPipCullTextureDirLights);
  else if (programType == StaPipVU1DirLights)
    return getProgramByName(StaPipCullDirLights);
  else if (programType == StaPipVU1TextureEnvColor)  // TyraX: env (matcap)
    return getProgramByName(StaPipCullTextureEnv);
  else if (programType == StaPipVU1TextureColor)
    return getProgramByName(StaPipCullTextureColor);
  else
    return getProgramByName(StaPipCullColor);
}

StaPipProgramType StaPipQBufferRenderer::getDrawProgramTypeByBag(
    const StaPipBag* bag) const {
  // Modified by TyraX: particle billboards - the vertex slot carries
  // centers, the ST slot the per-particle basis weights. The texture bag is
  // always present (params channel); a real image selects the T variant.
  if (bag->billboard != nullptr)
    return bag->texture->texture != nullptr ? StaPipVU1BillboardTexture
                                            : StaPipVU1BillboardColor;
  // Modified by TyraX: env (matcap) - the ST slot carries normals, the
  // texture ST is computed on VU1. Lighting is unsupported with env.
  if (bag->texture != nullptr && bag->texture->coordinatesAreNormals)
    return StaPipVU1TextureEnvColor;
  auto isLightingEnabled = bag->lighting != nullptr;
  auto isTextureEnabled = bag->texture != nullptr;
  return getDrawProgramTypeByParams(isLightingEnabled, isTextureEnabled);
}

StaPipProgramType StaPipQBufferRenderer::getDrawProgramTypeByParams(
    const bool& isLightingEnabled, const bool& isTextureEnabled) const {
  if (isLightingEnabled && isTextureEnabled)
    return StaPipVU1TextureDirLights;
  else if (isLightingEnabled)
    return StaPipVU1DirLights;
  else if (isTextureEnabled)
    return StaPipVU1TextureColor;
  else
    return StaPipVU1Color;
}

}  // namespace Tyra
