#pragma once

// Skinning straight to the GS, the draw half (EE only; docs/animated-models.md, "Skinning
// straight to the GS (EE only)"). A skinned part whose unique vertices a pose pass left
// (SkelInstance::skinIntoUniques) is mapped to the GS in fixed point (skel_eegs.hpp) and
// written as one GIF packet - TEST, TEX1, TEX0, ALPHA, then the corners as PACKED ST (with Q),
// RGBAQ and XYZF2, as the VU1 programs send them - that goes into the frame's ordered chain
// behind the VU1 work already recorded. VU1 draws none of it.
//
// A part is drawn this way when it is lit and textured like the compact TD program draws it
// (REPEAT wrap, a texture already in VRAM, no environment map) and its pose box is wholly in
// front of the near plane and inside the guard band. A part wholly outside the view is
// culled; a part that crosses the view's limits, or fails any other condition, is refused
// (`draw` returns false) and the caller draws it through its stream, as before.

#include <packet2.h>
#include <tamtypes.h>

#include "renderer/3d/mesh/dynamic/skel_eegs.hpp"
#include "renderer/3d/pipeline/static/core/bag/stapip_bag.hpp"

namespace Tyra {

class RendererCore;
struct SkelBindLod;

namespace SkelSkin {
namespace EeGs {

/** Counters of the parts a view's draw pass saw (reset by takeStats). */
struct DrawStats {
  u32 drawn = 0;     // parts sent as GS packets
  u32 culled = 0;    // parts wholly outside the view
  u32 crossing = 0;  // parts refused because they cross the view's limits
  u32 refused = 0;   // parts refused for any other reason (state, texture, packet room)
  u32 corners = 0;   // corners sent
  u32 packets = 0;   // packets submitted
};

class Drawer {
 public:
  /** Draws a part of a bag whose model matrix and texture are the bag's. True: the part is
   * drawn or culled (nothing else to do); false: draw it through its stream. */
  bool draw(RendererCore& core, const StaPipBag& bag, const Result& res, const Bind& bind);
  /** True while parts are queued: call flush at the end of the pass, and before anything
   * else is drawn (a texture upload, another bag). */
  bool pending() const { return open; }
  /** Sends the queued parts to the frame chain. */
  void flush(RendererCore& core);
  DrawStats takeStats() {
    DrawStats s = stats;
    stats = DrawStats();
    return s;
  }

 private:
  packet2_t* packet = nullptr;
  u32 packetQwords = 0;
  bool open = false;
  const void* openTexture = nullptr;
  std::vector<Out> outs;
  DrawStats stats;
  bool ensurePacket(u32 qwords);
};

/** The one drawer of the process. */
Drawer& drawer();

}  // namespace EeGs
}  // namespace SkelSkin
}  // namespace Tyra
