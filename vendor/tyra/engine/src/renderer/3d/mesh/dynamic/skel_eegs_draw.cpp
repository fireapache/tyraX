// Skinning straight to the GS, the draw half (renderer/3d/mesh/dynamic/skel_eegs_draw.hpp).
// Modified by TyraX: new file.

#include "renderer/3d/mesh/dynamic/skel_eegs_draw.hpp"

#include <dma.h>
#include <dma_tags.h>
#include <draw.h>
#include <packet2_utils.h>

#include <cstring>

#include "debug/debug.hpp"
#include "renderer/core/gs/renderer_core_depth.hpp"
#include "renderer/core/paths/path1/frame_submission.hpp"
#include "renderer/core/paths/path3/path3_fence.hpp"
#include "renderer/core/renderer_core.hpp"
#include "renderer/3d/pipeline/static/core/programs/stapip_vu1_shared_defines.h"

namespace Tyra {
namespace SkelSkin {
namespace EeGs {

namespace {

// ST, RGBAQ and XYZF2 a loop, 64 bits each: 24 bytes a corner (the PACKED form is 48).
constexpr u64 kRegList = ((u64)GIF_REG_ST) << 0 | ((u64)GIF_REG_RGBAQ) << 4 |
                         ((u64)GIF_REG_XYZF2) << 8;
// State A+D pairs a part writes: TEST, TEX1, TEX0, ALPHA.
constexpr u32 kStateSets = 4;
constexpr u32 kPrimTagQwords = 1;
constexpr u32 kMaxNloop = 32767;

inline u32 floatBits(float f) {
  u32 b;
  std::memcpy(&b, &f, 4);
  return b;
}

u64 testWord(const StaPipBag& bag, const RendererCore& core) {
  const int date = bag.info->dateLit ? 1 : 0;
  // As StaPipQBufferRenderer::sendObjectData writes it.
  if (bag.info->zTestType == PipelineZTest_AllPass)
    return GS_SET_TEST(0, 0, 0, 0, date, 0, 0, ZTEST_METHOD_ALLPASS);
  if (bag.info->zTestType == PipelineZTest_TestOnly)
    return GS_SET_TEST(DRAW_ENABLE, ATEST_METHOD_ALLFAIL, 0x00, ATEST_KEEP_ZBUFFER, date,
                       DRAW_DISABLE, DRAW_ENABLE, core.gs.zBuffer.method);
  return GS_SET_TEST(DRAW_ENABLE, ATEST_METHOD_NOTEQUAL, 0x00, ATEST_KEEP_ALL, date,
                     DRAW_DISABLE, DRAW_ENABLE, core.gs.zBuffer.method);
}

u64 alphaWord(const StaPipBag& bag) {
  const u8 fix = bag.info->additiveBlendFix;
  const u8 sub = bag.info->subtractiveBlendFix;
  return sub != 0   ? GS_SET_ALPHA(2, 0, 2, 1, sub)
         : fix != 0 ? GS_SET_ALPHA(0, 2, 2, 1, fix)
                    : GS_SET_ALPHA(0, 1, 0, 1, 0);
}

// Hand the packet to the frame chain the way the batched world lines are (Gunner's
// line_batch): recorded behind the 3D already in the chain, or on the legacy renderer a
// plain PATH3 send after the queued 3D has drained.
void submit(RendererCore& core, packet2_t* p) {
  packet2_update(p, frameDrawFinish(p->next));
  packet2_chain_close_tag(p);
  if (!Vif1Queue::recordingFrame()) {
    if (!core.drained3DForDebug && core.getPath1()->isVU1Configured()) {
      core.sync.align3D();
      core.drained3DForDebug = true;
    }
  }
  frameWaitGif(DMA_CHANNEL_GIF, 0);
  path3Fence();
  frameSendPacket(p, DMA_CHANNEL_GIF, true);
  frameWaitGif(DMA_CHANNEL_GIF, 0);
}

}  // namespace

Drawer& drawer() {
  static Drawer d;
  return d;
}

bool Drawer::ensurePacket(u32 qwords) {
  if (packet != nullptr && packetQwords >= qwords) return true;
  if (packet != nullptr) {
    packet2_free(packet);
    packet = nullptr;
    packetQwords = 0;
  }
  // Room for a few parts' corners at once; the packet is made once and reused.
  u32 want = qwords + 512;
  if (want < 2048) want = 2048;
  packet = packet2_create(want, P2_TYPE_NORMAL, P2_MODE_CHAIN, false);
  if (packet == nullptr) return false;
  packetQwords = want;
  return true;
}

bool Drawer::draw(RendererCore& core, const StaPipBag& bag, const Result& res, const Bind& bind) {
  // The compact TD program's class: a lit, textured bag with its own model matrix.
  if (!bag.lighting || !bag.texture || !bag.texture->texture ||
      bag.texture->coordinatesAreNormals || bag.billboard != nullptr ||
      bag.info->transformationType != TyraMVP || bag.info->model == nullptr || !bind.ready ||
      bind.count == 0 || res.count == 0 || bind.cornerUnique.size() != bind.count) {
    ++stats.refused;
    return false;
  }
  Texture* const tex = bag.texture->texture;
  const texwrap_t* wrap = tex->getWrapSettings();
  if (wrap->horizontal != WRAP_REPEAT || wrap->vertical != WRAP_REPEAT ||
      !core.texture.isResident(tex)) {
    ++stats.refused;
    return false;
  }
  const M4x4 mvp = core.renderer3D.getViewProj() * *bag.info->model;
  switch (classifyBox(mvp.data, res.step, res.oi, res.lo, res.hi, VU1_CLIP_XY_BAND)) {
    case Box::Outside:
      ++stats.culled;
      return true;
    case Box::Crossing:
      ++stats.crossing;
      return false;
    default:
      break;
  }
  Screen scr;
  scr.sx = RendererCoreDepth::screenScaleX;
  scr.sy = RendererCoreDepth::screenScaleY;
  scr.sz = RendererCoreDepth::scale;
  scr.maxZ = (float)RendererCoreDepth::maxZ;
  scr.fog = core.fog.enabled && !bag.info->fogDisabled;
  scr.fogScale = core.fog.scale;
  scr.fogOffset = core.fog.offset;
  Xform xf;
  if (!buildXform(mvp.data, res.step, res.oi, res.lo, res.hi, scr, xf)) {
    ++stats.refused;
    return false;
  }
  Light lt;
  buildLight(bag.lighting->lightMatrix->data,
             reinterpret_cast<const float*>(bag.lighting->dirLights->getLightDirections()),
             reinterpret_cast<const float*>(bag.lighting->dirLights->getLightColors()),
             bag.lighting->dirLights->signedSH, lt);

  // Per unique: the GS position and depth, Q, the light.
  if (outs.size() < res.count) outs.resize(res.count);
  for (u32 u = 0; u < res.count; ++u) {
    Out& o = outs[u];
    if (!project(xf, res.pos + 4 * (size_t)u, o)) {
      ++stats.refused;
      return false;
    }
    o.rgba = light(lt, res.nrm + 4 * (size_t)u);
  }

  const u32 corners = bind.count;
  const u32 loops = corners > kMaxNloop ? kMaxNloop : corners;
  const u32 need = 1 + kStateSets + 1 + kPrimTagQwords + (loops * 3 + 1) / 2 + 4;
  if (open && (openTexture != tex || packet == nullptr ||
               (u32)(packet->next - packet->base) + need + 2 > packetQwords))
    flush(core);
  if (!ensurePacket(need + 2)) {
    ++stats.refused;
    return false;
  }

  // The texture: resident (checked), so this uploads nothing and evicts nothing.
  auto temp = core.texture.useTexture(tex);
  RendererCoreTextureBuffers tb = {temp.id, temp.core, temp.clut};
  core.texture.updateClutBuffer(tb.clut);
  tb.core->info.function = bag.texture->textureFunction;
  const texbuffer_t* tbuf = tb.core;
  const clutbuffer_t* cl = &core.texture.clut;
  const bool linear = bag.info->textureMappingType == TyraLinear;

  if (!open) {
    packet2_reset(packet, false);
    packet2_chain_open_end(packet, 0, 0);
    open = true;
    openTexture = tex;
  }
  packet2_utils_gif_add_set(packet, kStateSets);
  packet2_add_2x_s64(packet, testWord(bag, core), GS_REG_TEST);
  packet2_add_2x_s64(packet,
                     GS_SET_TEX1(LOD_USE_K, 0, linear ? LOD_MAG_LINEAR : LOD_MAG_NEAREST,
                                 linear ? LOD_MIN_LINEAR : LOD_MIN_NEAREST, LOD_MIPMAP_REGISTER, 0,
                                 0),
                     GS_REG_TEX1);
  packet2_add_2x_s64(packet,
                     GS_SET_TEX0(tbuf->address >> 6, tbuf->width >> 6, tbuf->psm,
                                 tbuf->info.width, tbuf->info.height, tbuf->info.components,
                                 tbuf->info.function, cl->address >> 6, cl->psm, cl->storage_mode,
                                 cl->start, cl->load_method),
                     GS_REG_TEX0);
  packet2_add_2x_s64(packet, alphaWord(bag), GS_REG_ALPHA);

  const u64 prim = GS_SET_PRIM(PRIM_TRIANGLE, PRIM_SHADE_GOURAUD, DRAW_ENABLE,
                               scr.fog ? DRAW_ENABLE : DRAW_DISABLE, DRAW_ENABLE, DRAW_DISABLE,
                               PRIM_MAP_ST, 0, PRIM_UNFIXED);
  qword_t* tag = packet->next;
  tag->dw[0] = GIF_SET_TAG(loops, 0, 1, prim, GIF_FLG_REGLIST, 3);
  tag->dw[1] = kRegList;
  u64* d = reinterpret_cast<u64*>(tag + 1);
  const u16* cu = bind.cornerUnique.data();
  const float* st = bind.st.data();
  for (u32 c = 0; c < loops; ++c) {
    const Out& o = outs[cu[c]];
    d[0] = (u64)floatBits(st[2 * c] * o.q) | ((u64)floatBits(st[2 * c + 1] * o.q) << 32);
    d[1] = (u64)o.rgba | ((u64)floatBits(o.q) << 32);
    d[2] = (u64)o.xy | ((u64)o.zf << 32);
    d += 3;
  }
  // XYZF2 packs X, Y in the low word and Z with F above it: x (16), y (16) at bits 0-31,
  // then z (24) and f (8) - the layout project() built.
  qword_t* end = reinterpret_cast<qword_t*>((reinterpret_cast<uintptr_t>(d) + 15) & ~(uintptr_t)15);
  packet2_update(packet, end);
  ++stats.drawn;
  stats.corners += loops;
  return true;
}

void Drawer::flush(RendererCore& core) {
  if (!open) return;
  open = false;
  openTexture = nullptr;
  submit(core, packet);
  ++stats.packets;
}

}  // namespace EeGs
}  // namespace SkelSkin
}  // namespace Tyra
