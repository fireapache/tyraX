/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Added by TyraX: the .tskl parse (versions 1-3). Host-compilable (no EE
# header), so tools/verify-tskl-compact.cpp reads the same bytes the console
# does. The layout is the editor's src/glbparser.cpp (writeTskl).
*/

#include <math.h>
#include <string.h>

#include <string>
#include <vector>

#include "loaders/3d/tmdl_loader/tmdl_compact.hpp"
#include "loaders/3d/tskl_loader/tskl_loader.hpp"
#include "renderer/3d/mesh/dynamic/blend_space_weights.hpp"

namespace Tyra {

namespace {

// Corruption guards - a bad count must not turn into a huge allocation.
constexpr u32 kMaxParts = 256;
constexpr u32 kMaxVertices = 1u << 21;
constexpr u32 kMaxLods = 4;
// The newest layout this parse reads (the editor's writeTskl).
constexpr u32 kMaxVersion = 3;

/** Bounds-checked sequential reader over the in-memory file. */
struct Reader {
  const u8* p;
  size_t left;

  bool bytes(void* dst, size_t n) {
    if (n > left) return false;
    if (n) memcpy(dst, p, n);
    p += n;
    left -= n;
    return true;
  }
  bool u32le(u32* dst) { return bytes(dst, 4); }
  bool f32le(float* dst) { return bytes(dst, 4); }
  bool floats(std::vector<float>* dst, size_t count) {
    if (count > left / sizeof(float)) return false;
    dst->resize(count);
    return count == 0 || bytes(dst->data(), count * sizeof(float));
  }
  bool fixedString(std::string* dst, size_t size) {
    if (size > left) return false;
    size_t len = 0;
    while (len < size && p[len]) ++len;
    dst->assign(reinterpret_cast<const char*>(p), len);
    p += size;
    left -= size;
    return true;
  }
};

/** A version 1-2 mesh as the floats it was written with. */
struct FloatMesh {
  u32 count = 0;
  std::vector<float> pos, nrm, uv;
  std::vector<u8> joints, weights;
};

struct FloatPart {
  std::string name, texture;
  float color[4] = {1.0F, 1.0F, 1.0F, 1.0F};
  FloatMesh base;
  std::vector<FloatMesh> lods;
};

/** The version 1-2 float mesh (count, pos, nrm, [uv], joints, weights). */
bool readFloatMesh(Reader& in, bool textured, u32 paletteCount, FloatMesh& m) {
  if (!in.u32le(&m.count) || m.count == 0 || m.count > kMaxVertices)
    return false;
  const size_t n = m.count;
  if (textured && !in.floats(&m.uv, n * 2)) return false;
  if (!in.floats(&m.pos, n * 3) || !in.floats(&m.nrm, n * 3)) return false;
  if (n * 8 > in.left) return false;  // joints and weights, 8 bytes a vertex
  m.joints.resize(n * 4);
  m.weights.resize(n * 4);
  if (!in.bytes(m.joints.data(), n * 4) || !in.bytes(m.weights.data(), n * 4))
    return false;
  for (u8 joint : m.joints)
    if (joint >= paletteCount) return false;
  return true;
}

/**
 * The version 3 compact mesh: count, then (when textured) the UV grid and
 * its UVs, then positions, normals (x, y, z, 0), joints and weights.
 */
bool readCompactMesh(Reader& in, bool textured, u32 paletteCount,
                     SkelMesh& m) {
  u32 n = 0;
  if (!in.u32le(&n) || n == 0 || n > kMaxVertices) return false;
  // The payload is 22 bytes a vertex (18 untextured): a count the file cannot
  // hold is refused before anything is sized by it.
  if ((size_t)n * (textured ? 22 : 18) > in.left) return false;
  m.vertexCount = n;
  m.uv.clear();
  m.uvFracBits = 12;
  m.uvOrigin[0] = m.uvOrigin[1] = 0;
  if (textured) {
    s8 frac = 0;
    if (!in.bytes(&frac, 1) || frac < 4 || frac > 12) return false;
    m.uvFracBits = frac;
    if (!in.bytes(m.uvOrigin, sizeof(m.uvOrigin))) return false;
    m.uv.resize((size_t)n * 2);
    if (!in.bytes(m.uv.data(), (size_t)n * 2 * sizeof(s16))) return false;
  }
  m.pos.resize((size_t)n * 3);
  if (!in.bytes(m.pos.data(), (size_t)n * 3 * sizeof(s16))) return false;
  m.nrm.resize((size_t)n * 4);
  if (!in.bytes(m.nrm.data(), (size_t)n * 4)) return false;
  m.joints.resize((size_t)n * 4);
  m.weights.resize((size_t)n * 4);
  if (!in.bytes(m.joints.data(), (size_t)n * 4) ||
      !in.bytes(m.weights.data(), (size_t)n * 4))
    return false;
  for (u8 joint : m.joints)
    if (joint >= paletteCount) return false;
  return true;
}

/**
 * A version 1-2 float mesh in the compact form. The grid is the editor's own
 * rule (TmdlCompact::fromFloats), so a v2 file and its v3 twin convert to the
 * same bits.
 */
SkelMesh toCompact(const FloatMesh& f, bool textured, const Compact::Box& box) {
  std::vector<float> verts((size_t)f.count * 8, 0.0F);
  for (u32 v = 0; v < f.count; ++v) {
    for (int c = 0; c < 3; ++c) {
      verts[(size_t)v * 8 + c] = f.pos[(size_t)v * 3 + c];
      verts[(size_t)v * 8 + 3 + c] = f.nrm[(size_t)v * 3 + c];
    }
    if (textured) {
      verts[(size_t)v * 8 + 6] = f.uv[(size_t)v * 2];
      verts[(size_t)v * 8 + 7] = f.uv[(size_t)v * 2 + 1];
    }
  }
  const CompactMesh cm = TmdlCompact::fromFloats(verts.data(), f.count, box);
  SkelMesh m;
  m.vertexCount = f.count;
  m.pos = cm.pos;
  m.nrm = cm.nrm;
  m.joints = f.joints;
  m.weights = f.weights;
  if (textured) {
    m.uv = cm.uv;
    m.uvFracBits = cm.uvFracBits;
    m.uvOrigin[0] = cm.uvOrigin[0];
    m.uvOrigin[1] = cm.uvOrigin[1];
  }
  return m;
}

/** A mesh's compact UVs, as floats. */
void uvFloats(const SkelMesh& m, std::vector<float>& out) {
  const float step = 1.0F / (float)(1 << m.uvFracBits);
  out.resize((size_t)m.vertexCount * 2);
  for (size_t i = 0; i < out.size(); ++i)
    out[i] = Compact::dequantize(m.uv[i], (float)m.uvOrigin[i & 1], step);
}

/**
 * Two meshes of one material, one after the other. Positions, normals and
 * joints concatenate as they are (one box per model). UVs concatenate when
 * the two grids agree; when they do not, both sides are decoded and the
 * union is quantized on its own grid (the same rule as a fresh mesh).
 */
SkelMesh mergeMeshes(const SkelMesh& a, const SkelMesh& b, bool textured) {
  SkelMesh m;
  m.vertexCount = a.vertexCount + b.vertexCount;
  m.pos = a.pos;
  m.pos.insert(m.pos.end(), b.pos.begin(), b.pos.end());
  m.nrm = a.nrm;
  m.nrm.insert(m.nrm.end(), b.nrm.begin(), b.nrm.end());
  m.joints = a.joints;
  m.joints.insert(m.joints.end(), b.joints.begin(), b.joints.end());
  m.weights = a.weights;
  m.weights.insert(m.weights.end(), b.weights.begin(), b.weights.end());
  if (!textured) return m;

  if (a.uvFracBits == b.uvFracBits && a.uvOrigin[0] == b.uvOrigin[0] &&
      a.uvOrigin[1] == b.uvOrigin[1]) {
    m.uvFracBits = a.uvFracBits;
    m.uvOrigin[0] = a.uvOrigin[0];
    m.uvOrigin[1] = a.uvOrigin[1];
    m.uv = a.uv;
    m.uv.insert(m.uv.end(), b.uv.begin(), b.uv.end());
    return m;
  }
  std::vector<float> fa, fb;
  uvFloats(a, fa);
  uvFloats(b, fb);
  fa.insert(fa.end(), fb.begin(), fb.end());
  std::vector<float> verts((size_t)m.vertexCount * 8, 0.0F);
  for (u32 v = 0; v < m.vertexCount; ++v) {
    verts[(size_t)v * 8 + 6] = fa[(size_t)v * 2];
    verts[(size_t)v * 8 + 7] = fa[(size_t)v * 2 + 1];
  }
  TmdlCompact::uvGrid(verts.data(), m.vertexCount, &m.uvFracBits, m.uvOrigin);
  const float step = 1.0F / (float)(1 << m.uvFracBits);
  m.uv.resize(fa.size());
  for (size_t i = 0; i < fa.size(); ++i)
    m.uv[i] = Compact::quantize(fa[i], (float)m.uvOrigin[i & 1], step);
  return m;
}

/** A part's base mesh, or its LOD `level`. The base stands in for a missing
 * level, so a small undecimated part never vanishes at distance. */
const SkelMesh& meshAt(const SkelPart& part, size_t level) {
  if (level < part.lods.size()) return part.lods[level];
  return part;
}

/** "BLSP" (see the trailer comment in the editor's writeTskl). A sample naming
 * a clip the model does not have is dropped with a warning; a truncated block
 * keeps the spaces read so far. False = stop reading. */
bool readBlendSpaces(Reader& in, u32 count, SkelModel& model) {
  for (u32 b = 0; b < count; b++) {
    SkelBlendSpace space;
    u32 samples = 0;
    if (!in.fixedString(&space.name, 32) || !in.f32le(&space.stride) ||
        !in.f32le(&space.smoothing) || !in.f32le(&space.lodFade) ||
        !in.u32le(&samples) || samples > (u32)blendspace::kMaxSamples)
      return false;
    for (u32 k = 0; k < samples; k++) {
      u32 clip = 0, flags = 0;
      blendspace::Sample sm{};
      if (!in.u32le(&clip) || !in.f32le(&sm.dirDeg) || !in.f32le(&sm.speed) ||
          !in.u32le(&flags))
        return false;
      if (clip >= model.clips.size()) continue;  // names a missing clip
      sm.clip = (int)clip;
      sm.ownTime = (flags & 1U) != 0;
      space.samples.push_back(sm);
    }
    model.blendSpaces.push_back(std::move(space));
  }
  return true;
}

/** "BLGR": the blend spaces laid out as a 2D GRID (docs/blend-spaces.md) - per
 * grid u32 blendSpace (its BLSP index), u32 columns, rows, f32 dirMin, dirMax,
 * speedMin, speedMax. The rest stay radial. */
bool readBlendGrids(Reader& in, u32 count, SkelModel& model) {
  for (u32 k = 0; k < count; k++) {
    u32 space = 0, columns = 0, rows = 0;
    blendspace::Grid g;
    if (!in.u32le(&space) || !in.u32le(&columns) || !in.u32le(&rows) ||
        !in.f32le(&g.dirMin) || !in.f32le(&g.dirMax) || !in.f32le(&g.speedMin) ||
        !in.f32le(&g.speedMax))
      return false;
    if (space >= model.blendSpaces.size() || columns < 2 || rows < 2) continue;
    g.columns = (int)columns;
    g.rows = (int)rows;
    model.blendSpaces[space].grid = true;
    model.blendSpaces[space].lines = g;
  }
  return true;
}

}  // namespace

void decodeMesh(const Compact::Box& box, const SkelMesh& mesh, float* pos,
                float* nrm, float* uv) {
  const bool hasUv = uv != nullptr && !mesh.uv.empty();
  const float uvStep = 1.0F / (float)(1 << mesh.uvFracBits);
  for (u32 v = 0; v < mesh.vertexCount; ++v) {
    if (pos)
      for (int c = 0; c < 3; ++c)
        pos[(size_t)v * 3 + c] =
            Compact::dequantize(mesh.pos[(size_t)v * 3 + c], box.origin[c], box.step);
    if (nrm) {
      float n[3] = {(float)mesh.nrm[(size_t)v * 4] / 127.0F,
                    (float)mesh.nrm[(size_t)v * 4 + 1] / 127.0F,
                    (float)mesh.nrm[(size_t)v * 4 + 2] / 127.0F};
      const float len = sqrtf(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
      if (len > 0.0F)
        for (float& x : n) x /= len;
      for (int c = 0; c < 3; ++c) nrm[(size_t)v * 3 + c] = n[c];
    }
    if (hasUv)
      for (int a = 0; a < 2; ++a)
        uv[(size_t)v * 2 + a] = Compact::dequantize(
            mesh.uv[(size_t)v * 2 + a], (float)mesh.uvOrigin[a], uvStep);
  }
}

bool TsklLoader::parse(const u8* data, size_t size, SkelModel& out) {
  out = SkelModel();
  Reader in{data, size};
  char magic[4];
  u32 version = 0, nodeCount = 0, paletteCount = 0, partCount = 0,
      clipCount = 0;
  if (!in.bytes(magic, 4) || memcmp(magic, "TSKL", 4) != 0 ||
      !in.u32le(&version) || version < 1 || version > kMaxVersion ||
      !in.u32le(&nodeCount) || !in.u32le(&paletteCount) ||
      !in.u32le(&partCount) || !in.u32le(&clipCount))
    return false;
  for (int c = 0; c < 3; c++)
    if (!in.f32le(&out.min[c])) return false;
  for (int c = 0; c < 3; c++)
    if (!in.f32le(&out.max[c])) return false;
  if (version >= 3) {
    for (int c = 0; c < 3; c++)
      if (!in.f32le(&out.box.origin[c])) return false;
    if (!in.f32le(&out.box.step) || !(out.box.step > 0.0F) ||
        !(out.box.step < 1e30F))
      return false;
    // The grid is exact only on a power-of-two step (stapip_compact.hpp), and
    // the origin must be a finite number of steps.
    u32 stepBits = 0;
    memcpy(&stepBits, &out.box.step, 4);
    if ((stepBits & 0x7fffffu) != 0) return false;
    for (int c = 0; c < 3; c++) {
      const float o = out.box.origin[c];
      if (!(o - o == 0.0F)) return false;  // NaN or infinite
    }
  }
  if (nodeCount == 0 || paletteCount == 0 || paletteCount > 256 ||
      partCount == 0 || partCount > kMaxParts || clipCount == 0)
    return false;

  // A node record is 112 bytes, a palette slot 68: a count the file cannot
  // hold is refused before anything is sized by it.
  if ((size_t)nodeCount * 112 > in.left) return false;
  out.nodes.resize(nodeCount);
  for (u32 i = 0; i < nodeCount; i++) {
    SkelNode& node = out.nodes[i];
    u32 parent = 0, flags = 0;
    if (!in.u32le(&parent) || !in.u32le(&flags) ||
        !in.bytes(node.t, sizeof(node.t)) ||
        !in.bytes(node.r, sizeof(node.r)) ||
        !in.bytes(node.s, sizeof(node.s)) ||
        !in.bytes(node.matrix, sizeof(node.matrix)))
      return false;
    node.parent = (s32)parent;
    node.hasMatrix = flags & 1 ? 1 : 0;
    if (node.parent >= (s32)nodeCount) return false;
  }

  // Parents-first order for the pose walk: iterative DFS from the roots.
  // A malformed hierarchy (cycle / unreachable node) fails the load.
  {
    std::vector<std::vector<u32>> children(nodeCount);
    for (u32 i = 0; i < nodeCount; i++)
      if (out.nodes[i].parent >= 0) children[out.nodes[i].parent].push_back(i);
    std::vector<u32> stack;
    for (u32 i = nodeCount; i-- > 0;)
      if (out.nodes[i].parent < 0) stack.push_back(i);
    out.order.reserve(nodeCount);
    while (!stack.empty()) {
      const u32 i = stack.back();
      stack.pop_back();
      out.order.push_back(i);
      for (u32 k = children[i].size(); k-- > 0;) stack.push_back(children[i][k]);
    }
    if (out.order.size() != nodeCount) return false;
  }

  if ((size_t)paletteCount * 68 > in.left) return false;
  out.palette.resize(paletteCount);
  for (u32 i = 0; i < paletteCount; i++) {
    SkelJoint& joint = out.palette[i];
    if (!in.u32le(&joint.node) || !in.bytes(joint.ibm, sizeof(joint.ibm)))
      return false;
    if (joint.node >= nodeCount) return false;
  }

  for (u32 c = 0; c < clipCount; c++) {
    SkelClip clip;
    u32 channelCount = 0;
    if (!in.fixedString(&clip.name, 32) || !in.f32le(&clip.duration) ||
        !in.u32le(&channelCount))
      return false;
    for (u32 ch = 0; ch < channelCount; ch++) {
      SkelChannel channel;
      u8 head[4];
      u32 keyCount = 0;
      if (!in.u32le(&channel.node) || !in.bytes(head, 4) ||
          !in.u32le(&keyCount) || channel.node >= nodeCount || keyCount == 0)
        return false;
      channel.path = head[0];
      channel.step = head[1];
      if (channel.path > 2) return false;
      if (!in.floats(&channel.times, keyCount)) return false;
      if (channel.path == 1) {
        channel.quat.resize((size_t)keyCount * 4);
        if (!in.bytes(channel.quat.data(), channel.quat.size() * 2)) return false;
      } else {
        if (!in.floats(&channel.vec, (size_t)keyCount * 3)) return false;
      }
      clip.channels.push_back(std::move(channel));
    }
    out.clips.push_back(std::move(clip));
  }

  if (version >= 3) {
    for (u32 mi = 0; mi < partCount; mi++) {
      SkelPart part;
      u32 lodCount = 0;
      if (!in.fixedString(&part.name, 32) ||
          !in.fixedString(&part.texturePath, 64) ||
          !in.bytes(part.color, sizeof(part.color)))
        return false;
      if (part.name.empty()) part.name = "mat";
      const bool textured = !part.texturePath.empty();
      if (!readCompactMesh(in, textured, paletteCount, part)) return false;
      if (!in.u32le(&lodCount) || lodCount > kMaxLods) return false;
      for (u32 l = 0; l < lodCount; l++) {
        SkelLod lod;
        if (!readCompactMesh(in, textured, paletteCount, lod)) return false;
        part.lods.push_back(std::move(lod));
      }
      out.parts.push_back(std::move(part));
    }
  } else {
    // Version 1-2: floats. They are read whole first - the box is the bounds
    // of every mesh, base and LODs, so it is known only after the last part.
    std::vector<FloatPart> floats(partCount);
    for (u32 mi = 0; mi < partCount; mi++) {
      FloatPart& fp = floats[mi];
      u32 lodCount = 0;
      if (!in.fixedString(&fp.name, 32) || !in.fixedString(&fp.texture, 64) ||
          !in.bytes(fp.color, sizeof(fp.color)))
        return false;
      if (fp.name.empty()) fp.name = "mat";
      const bool textured = !fp.texture.empty();
      if (!readFloatMesh(in, textured, paletteCount, fp.base)) return false;
      if (version >= 2) {
        if (!in.u32le(&lodCount) || lodCount > kMaxLods) return false;
        for (u32 l = 0; l < lodCount; l++) {
          FloatMesh lod;
          if (!readFloatMesh(in, textured, paletteCount, lod)) return false;
          fp.lods.push_back(std::move(lod));
        }
      }
    }
    TmdlCompact::Bounds bounds;
    for (const FloatPart& fp : floats) {
      bounds.add(fp.base.pos.data(), fp.base.count, 3);
      for (const FloatMesh& lod : fp.lods) bounds.add(lod.pos.data(), lod.count, 3);
    }
    out.box = bounds.box();
    for (const FloatPart& fp : floats) {
      SkelPart part;
      part.name = fp.name;
      part.texturePath = fp.texture;
      for (int c = 0; c < 4; ++c) part.color[c] = fp.color[c];
      static_cast<SkelMesh&>(part) =
          toCompact(fp.base, !fp.texture.empty(), out.box);
      for (const FloatMesh& lod : fp.lods) {
        SkelLod converted;
        static_cast<SkelMesh&>(converted) =
            toCompact(lod, !fp.texture.empty(), out.box);
        part.lods.push_back(std::move(converted));
      }
      out.parts.push_back(std::move(part));
    }
  }

  // Optional trailers, read in order: "NAME" (node names, only when the count
  // is this model's), "LTEX" (per part, per LOD, an authored LOD's own texture;
  // "" = the part's), "BLSP" (blend spaces), "BLGR" (grids). A file that ends
  // on a tag boundary is fine; one cut inside a tag it has begun is corrupt. An
  // unknown tag, or a count that is not this model's, stops the reading (an
  // older or newer writer), as it always did.
  for (bool more = true; more && in.left > 0;) {
    if (in.left < 8) return false;  // a cut tag
    char tag[4];
    u32 count = 0;
    if (!in.bytes(tag, 4) || !in.u32le(&count)) return false;
    if (memcmp(tag, "NAME", 4) == 0) {
      if (count != nodeCount) break;
      for (u32 i = 0; i < nodeCount; i++)
        if (!in.fixedString(&out.nodes[i].name, 32)) return false;
    } else if (memcmp(tag, "BLSP", 4) == 0) {
      if (!readBlendSpaces(in, count, out)) return false;
    } else if (memcmp(tag, "BLGR", 4) == 0) {
      if (!readBlendGrids(in, count, out)) return false;
    } else if (memcmp(tag, "LTEX", 4) == 0) {
      if (count != out.parts.size()) break;
      for (SkelPart& part : out.parts) {
        u32 lods = 0;
        if (!in.u32le(&lods)) return false;
        if (lods != part.lods.size()) {
          for (SkelPart& p : out.parts)
            for (SkelLod& lod : p.lods) lod.texturePath.clear();
          more = false;
          break;
        }
        for (SkelLod& lod : part.lods)
          if (!in.fixedString(&lod.texturePath, 64)) return false;
      }
    } else {
      break;
    }
  }

  // Merge parts that share texture and color: every part becomes a draw bag
  // per instance per frame (object-data DMA + packager run each), and models
  // commonly split one material into several tiny parts. Merging is safe -
  // vertices carry their own joints/weights and draw order among equal
  // materials is irrelevant under z-testing.
  for (size_t a = 0; a + 1 < out.parts.size(); a++) {
    SkelPart& dst = out.parts[a];
    for (size_t b = a + 1; b < out.parts.size();) {
      const SkelPart& src = out.parts[b];
      // tiers with their own textures merge only level for level: a missing
      // level's stand-in (the base arrays) would draw with the wrong texture
      bool sameLodTextures = true;
      {
        bool anyOwn = false;
        const SkelPart* sides[2] = {&dst, &src};
        for (const SkelPart* side : sides)
          for (const SkelLod& lod : side->lods) anyOwn |= !lod.texturePath.empty();
        if (anyOwn) {
          sameLodTextures = dst.lods.size() == src.lods.size();
          for (size_t l = 0; sameLodTextures && l < dst.lods.size(); l++)
            sameLodTextures = dst.lods[l].texturePath == src.lods[l].texturePath;
        }
      }
      const bool same = dst.texturePath == src.texturePath && sameLodTextures &&
                        memcmp(dst.color, src.color, sizeof(dst.color)) == 0;
      if (!same) {
        ++b;
        continue;
      }
      const bool textured = !dst.texturePath.empty();
      // the LOD chains first (the base stands in for a missing level)
      const size_t lodLevels =
          dst.lods.size() > src.lods.size() ? dst.lods.size() : src.lods.size();
      std::vector<SkelLod> merged(lodLevels);
      for (size_t l = 0; l < lodLevels; l++) {
        static_cast<SkelMesh&>(merged[l]) =
            mergeMeshes(meshAt(dst, l), meshAt(src, l), textured);
        if (l < dst.lods.size()) merged[l].texturePath = dst.lods[l].texturePath;
      }
      static_cast<SkelMesh&>(dst) = mergeMeshes(dst, src, textured);
      dst.lods = std::move(merged);
      out.parts.erase(out.parts.begin() + b);
    }
  }

  return true;
}

}  // namespace Tyra
