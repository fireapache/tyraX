#pragma once

// The pose cadence of animated bodies: which far bodies are re-posed (their pose evaluated
// and their vertices re-skinned) on a given frame. A body re-posed on frame t holds that pose
// and its stream until its next turn; the held pose is exact for the clip time it was
// evaluated at, so a stale body is the same picture a frame or three later, only its
// motion is held. Close bodies are re-posed every frame.
//
// The bands are the mesh LOD's: beyond the animation-LOD distance d a body is re-posed every
// kFarPeriod frames, beyond 2 d every kFarthestPeriod frames (the periods are powers of two,
// so the stagger is a mask). Bodies are staggered by their index, so the re-poses of a
// frame are spread over the periods instead of landing on one frame.
//
// A body is always re-posed when it has no held pose yet (its first frame, or the first
// frame after it re-enters the view) and when its mesh LOD changes (the other tier's
// buffers hold an older pose). The rule is pure so the host verifier
// (tools/verify-skel-cadence.cpp) checks it.

#include "renderer/3d/mesh/dynamic/skel_skin.hpp"

namespace Tyra {
namespace SkelSkin {
namespace Cadence {

/** Re-pose period (frames) of a body at squared distance dist2 with the animation-LOD
 * distance squared lodDist2 (> 0). Within the distance: 1 (every frame). */
constexpr u32 kFarPeriod = 2;
constexpr u32 kFarthestPeriod = 4;

inline u32 periodFor(float dist2, float lodDist2) {
  if (!(lodDist2 > 0.0F)) return 1;
  if (dist2 > lodDist2 * 4.0F) return kFarthestPeriod;
  if (dist2 > lodDist2) return kFarPeriod;
  return 1;
}

/** Whether a body with stagger index `index` is due on frame `tick` for `period` (1, 2 or
 * 4). Bodies with different indices fall on different frames within each period. */
inline bool dueOnFrame(u32 tick, u32 index, u32 period) {
  return period <= 1 || ((tick + index) & (period - 1)) == 0;
}

/** Whether a body is re-posed this frame: it is due, or it has no held pose (first frame,
 * or it was not updated for more than `maxGapTicks` frames: it re-entered the view), or its
 * mesh LOD changed. */
inline bool repostThisFrame(u32 tick, u32 index, u32 period, bool held, u32 lastTick,
                            u32 maxGapTicks, bool lodChanged) {
  if (lodChanged || !held) return true;
  if (tick - lastTick > maxGapTicks) return true;
  return dueOnFrame(tick, index, period);
}

}  // namespace Cadence
}  // namespace SkelSkin
}  // namespace Tyra
