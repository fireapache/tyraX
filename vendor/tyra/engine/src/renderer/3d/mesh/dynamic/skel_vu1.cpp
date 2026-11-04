// Skinning on VU1, EE side (skel_vu1.hpp). Modified by TyraX: new in the skinning work.

#include "renderer/3d/mesh/dynamic/skel_vu1.hpp"

namespace Tyra {
namespace SkelVu1 {

bool buildBind(const SkelMesh& mesh, u32 slots, BindCorner* out) {
  if (slots == 0 || slots > kMaxSlots) return false;
  for (u32 v = 0; v < mesh.vertexCount; ++v) {
    BindCorner& c = out[v];
    c.pos[0] = mesh.pos[3 * v];
    c.pos[1] = mesh.pos[3 * v + 1];
    c.pos[2] = mesh.pos[3 * v + 2];
    c.pad0 = 0;
    c.nrm[0] = mesh.nrm[4 * v];
    c.nrm[1] = mesh.nrm[4 * v + 1];
    c.nrm[2] = mesh.nrm[4 * v + 2];
    c.nrm[3] = 0;
    unsigned sum = 0;
    for (u32 k = 0; k < 4; ++k) {
      const u32 slot = mesh.joints[4 * v + k];
      if (slot >= slots) return false;
      c.joint[k] = (u8)(kPaletteBaseQword + kQwordsPerJoint * slot);
      c.weight[k] = mesh.weights[4 * v + k];
      sum += c.weight[k];
    }
    if (sum != 255) return false;
  }
  return true;
}

void buildPalette(const float* pose, u32 slots, const Compact::Box& bind,
                  const Compact::Box& out, float* palette) {
  // One pose's folds at a time: the EE runs one thread and the fold is consumed here.
  static SkelSkin::Folded folded;
  SkelSkin::foldPalette(pose, slots, bind, out, folded);
  const float inv255 = 1.0F / 255.0F;
  for (u32 s = 0; s < slots; ++s)
    for (u32 i = 0; i < 16; ++i) palette[16 * s + i] = folded.f[s][i] * inv255;
}

}  // namespace SkelVu1
}  // namespace Tyra
