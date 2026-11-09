/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Added by TyraX: per-object skeletal playback; pose evaluation on
# the EE, vertex skinning on VU0 in macro mode (COP2 inline asm).
# The sampling/pose math mirrors the editor's src/glbparser.cpp (the
# viewport preview and the stage-1 baker) - keep the formulas in sync so
# what the editor shows is what the console computes. Two parts do NOT keep
# bit-parity, both for VU0 rounding (~1 ulp against the EE FPU): the vertex
# loop, verified by screenshot parity against the EE loop, and (TyraX fork)
# the pose's matrix products (mulM4Vu0), verified by the golden-vertex gyms.
*/

#include "renderer/3d/mesh/dynamic/skel_instance.hpp"

#include <algorithm>
#include <float.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <unordered_map>
#include <string>
#include <new>
#include "debug/debug.hpp"
#include "loaders/3d/tskl_loader/skel_clip_codec.hpp"
#include "renderer/3d/mesh/dynamic/skel_room.hpp"
#include "renderer/3d/mesh/dynamic/skel_room.hpp"
#include "renderer/3d/mesh/dynamic/skel_skin.hpp"
#include "renderer/3d/mesh/dynamic/skel_unique.hpp"
#include "renderer/core/paths/path1/vif1_queue.hpp"

#if TYRA_SKEL_PROFILE
static unsigned skelTicks() { unsigned t; asm volatile("mfc0 %0, $9" : "=r"(t)); return t; }
#endif

#include "loaders/3d/builder/mesh_builder_data.hpp"
#if TYRA_MEMORY_MANAGER
#include "memory/manager.hpp"
#endif

namespace Tyra {

namespace {

/** Column-major r = a * b on VU0 in macro mode, for 16-byte aligned operands
 *  (r may alias a or b - every load lands before a store). A pose does 78 of
 *  these (41 nodes + 37 palette slots on the combatent): as a scalar triple
 *  loop on the EE they were ~60% of evalPose, about 2 us each - 0.18 of a
 *  0.29 ms pose (docs/animated-models.md, "What a pose costs"). The sequence
 *  is M4x4::cross's, which is private. */
void mulM4Vu0(float* r, const float* a, const float* b) {
  asm volatile(
      "lqc2         $vf1, 0x00(%1) \n\t"
      "lqc2         $vf2, 0x10(%1) \n\t"
      "lqc2         $vf3, 0x20(%1) \n\t"
      "lqc2         $vf4, 0x30(%1) \n\t"
      "lqc2         $vf5, 0x00(%2) \n\t"
      "lqc2         $vf6, 0x10(%2) \n\t"
      "lqc2         $vf7, 0x20(%2) \n\t"
      "lqc2         $vf8, 0x30(%2) \n\t"
      "vmulax.xyzw  $ACC, $vf5, $vf1 \n\t"
      "vmadday.xyzw $ACC, $vf6, $vf1 \n\t"
      "vmaddaz.xyzw $ACC, $vf7, $vf1 \n\t"
      "vmaddw.xyzw  $vf1, $vf8, $vf1 \n\t"
      "vmulax.xyzw  $ACC, $vf5, $vf2 \n\t"
      "vmadday.xyzw $ACC, $vf6, $vf2 \n\t"
      "vmaddaz.xyzw $ACC, $vf7, $vf2 \n\t"
      "vmaddw.xyzw  $vf2, $vf8, $vf2 \n\t"
      "vmulax.xyzw  $ACC, $vf5, $vf3 \n\t"
      "vmadday.xyzw $ACC, $vf6, $vf3 \n\t"
      "vmaddaz.xyzw $ACC, $vf7, $vf3 \n\t"
      "vmaddw.xyzw  $vf3, $vf8, $vf3 \n\t"
      "vmulax.xyzw  $ACC, $vf5, $vf4 \n\t"
      "vmadday.xyzw $ACC, $vf6, $vf4 \n\t"
      "vmaddaz.xyzw $ACC, $vf7, $vf4 \n\t"
      "vmaddw.xyzw  $vf4, $vf8, $vf4 \n\t"
      "sqc2         $vf1, 0x00(%0) \n\t"
      "sqc2         $vf2, 0x10(%0) \n\t"
      "sqc2         $vf3, 0x20(%0) \n\t"
      "sqc2         $vf4, 0x30(%0) \n\t"
      :
      : "r"(r), "r"(b), "r"(a)
      : "memory");
}

/** T * R * S from translation / quaternion / scale (glTF conventions). */
void fromTrs(float* m, const float* t, const float* q, const float* s) {
  const float x = q[0], y = q[1], z = q[2], w = q[3];
  const float x2 = x + x, y2 = y + y, z2 = z + z;
  const float xx = x * x2, xy = x * y2, xz = x * z2;
  const float yy = y * y2, yz = y * z2, zz = z * z2;
  const float wx = w * x2, wy = w * y2, wz = w * z2;
  m[0] = (1.0F - (yy + zz)) * s[0];
  m[1] = (xy + wz) * s[0];
  m[2] = (xz - wy) * s[0];
  m[3] = 0.0F;
  m[4] = (xy - wz) * s[1];
  m[5] = (1.0F - (xx + zz)) * s[1];
  m[6] = (yz + wx) * s[1];
  m[7] = 0.0F;
  m[8] = (xz + wy) * s[2];
  m[9] = (yz - wx) * s[2];
  m[10] = (1.0F - (xx + yy)) * s[2];
  m[11] = 0.0F;
  m[12] = t[0];
  m[13] = t[1];
  m[14] = t[2];
  m[15] = 1.0F;
}

}  // namespace

namespace {

/** One corner's bind data as the VU0 loop reads it (see the header comment on SkelBindLod),
 * decoded from the compact mesh: the position and normal, the weights normalized to sum 1 and
 * the joints sorted by descending weight, with the count of nonzero weights for the dispatch. */
struct BindCorner {
  float pos[3], nrm[3];
  float weights[4];
  u8 joints[4];
  u8 influences;
};

void bindCorner(const Compact::Box& box, const SkelMesh& m, u32 v, BindCorner& c) {
  decodeVertex(box, m, v, c.pos, c.nrm, nullptr);
  const u8* jj = &m.joints[(size_t)v * 4];
  const u8* ww = &m.weights[(size_t)v * 4];
  u8 idx[4] = {0, 1, 2, 3};  // slot order by descending weight
  for (int a = 1; a < 4; a++)
    for (int b = a; b > 0 && ww[idx[b]] > ww[idx[b - 1]]; b--) {
      const u8 t = idx[b];
      idx[b] = idx[b - 1];
      idx[b - 1] = t;
    }
  const u32 wsum = (u32)ww[0] + ww[1] + ww[2] + ww[3];
  const float inv = wsum > 0 ? 1.0F / (float)wsum : 0.0F;
  u8 n = 0;
  for (int k = 0; k < 4; k++) {
    c.weights[k] = (float)ww[idx[k]] * inv;
    c.joints[k] = jj[idx[k]];
    if (ww[idx[k]] > 0) n++;
  }
  c.influences = n;
}

/** What the table of first corners (an unordered_map keyed by a 32-byte string) takes while it is
 * built, `n` vertices: per vertex a hash node (the next pointer, the key string, the index, the
 * cached hash) and the key's own 33-byte buffer, two blocks each with a 16-byte heap header and
 * rounded to 16, and a bucket - 132 bytes a vertex on the EE (a 36-byte node and a 33-byte string
 * served as 64 each, a 4-byte bucket), 152 on a 64-bit host, as measured. */
size_t firstCornerTableBytes(size_t n) {
  auto served = [](size_t b) { return (b + 16 + 15) / 16 * 16; };
  const size_t node = sizeof(void*) + sizeof(std::pair<const std::string, u32>) + sizeof(size_t);
  return n * (served(node) + served(32 + 1) + sizeof(void*));
}

/** The skin source of a mesh's corners: the first corner with the same bind attributes. Exact
 * bind attributes only: UV seams can share skinning, hard normals and different bone weights
 * cannot. The temporary table dies with the call. With `probe`, false - nothing made - when the
 * budget has no room for the source and the table (a throwing request over a full Skinning
 * budget halts the game). */
bool bindSkinSource(const Compact::Box& box, const SkelMesh& m, std::vector<u32>& source,
                    bool probe) {
  const u32 count = m.vertexCount;
  if (probe &&
      !SkelSkin::room(SkelSkin::vectorBytes(count, sizeof(u32)) + firstCornerTableBytes(count)))
    return false;
  source.assign(count, 0);
  std::unordered_map<std::string, u32> firstCorner;
  firstCorner.reserve(count);
  for (u32 v = 0; v < count; v++) {
    float pos[3], nrm[3];
    decodeVertex(box, m, v, pos, nrm, nullptr);
    char key[32];
    memcpy(key, pos, 12);
    memcpy(key + 12, nrm, 12);
    memcpy(key + 24, &m.joints[(size_t)v * 4], 4);
    memcpy(key + 28, &m.weights[(size_t)v * 4], 4);
    const auto inserted = firstCorner.emplace(std::string(key, sizeof(key)), v);
    source[v] = inserted.first->second;
  }
  return true;
}

/** Fills `out`'s float rows (positions, normals, weights, sorted joints, influence counts) from
 * the compact mesh: every corner, or only the unique ones (skinSource[v] == v) in corner order.
 * Every array is made at its exact size. */
void fillBindRows(const Compact::Box& box, const SkelMesh& m, const std::vector<u32>& skinSource,
                  bool uniquesOnly, SkelBindLod& out) {
  const u32 n = m.vertexCount;
  u32 rows = n;
  if (uniquesOnly) {
    rows = 0;
    for (u32 v = 0; v < n; v++)
      if (skinSource[v] == v) rows++;
  }
  out.bindPositions.reserve(rows);
  out.bindNormals.reserve(rows);
  out.skinWeights.reserve(rows);
  out.sortedJoints.reserve((size_t)rows * 4);
  out.influences.reserve(rows);
  for (u32 v = 0; v < n; v++) {
    if (uniquesOnly && skinSource[v] != v) continue;
    BindCorner c;
    bindCorner(box, m, v, c);
    Vec4 p, q, w;
    p.set(c.pos[0], c.pos[1], c.pos[2], 1.0F);
    q.set(c.nrm[0], c.nrm[1], c.nrm[2], 0.0F);
    w.set(c.weights[0], c.weights[1], c.weights[2], c.weights[3]);
    out.bindPositions.push_back(p);
    out.bindNormals.push_back(q);
    out.skinWeights.push_back(w);
    out.sortedJoints.insert(out.sortedJoints.end(), c.joints, c.joints + 4);
    out.influences.push_back(c.influences);
  }
  out.count = rows;
}

/** A level's unique corners as a level of their own (SkelBindLod::uniq): each corner whose
 * skinSource is itself, in corner order, with its bind data decoded. The level has no skin
 * source (every row is its own: skinBlendRange reads an empty one as the identity) and no corner
 * list until a plain-stream pass asks for one (uniqueCorners). */
bool buildUniqueLod(const Compact::Box& box, const SkelMesh& m, SkelBindLod& bl, bool probe) {
  if (probe) {
    // The unique level at its real size (the skin source is known): the object, three Vec4
    // rows, the joints and the influence counts.
    size_t uniques = 0;
    for (u32 v = 0; v < m.vertexCount; v++)
      if (bl.skinSource[v] == v) uniques++;
    if (!SkelSkin::room(sizeof(SkelBindLod) + 4 * SkelSkin::kBlockOverhead +
                        3 * SkelSkin::vectorBytes(uniques, sizeof(Vec4)) +
                        SkelSkin::vectorBytes(uniques * 4, 1) + SkelSkin::vectorBytes(uniques, 1)))
      return false;
  }
  auto u = std::make_shared<SkelBindLod>();
  fillBindRows(box, m, bl.skinSource, true, *u);
  bl.uniq = std::move(u);
  return true;
}

/** Permutes a mesh's triangles into the locality order of the indexed stream (TYRA_SKIN_INDEXED,
 * Indexed::orderTriangles): the corners move triangle by triangle in every per-corner array.
 * The draw order inside an opaque part is not visible, and everything built afterwards (the bind
 * data, the streams, the stream check) sees the reordered mesh. The corners that repeat an
 * earlier corner's bind data are found on the compact bytes (position, normal, joints, weights),
 * the same partition repackBind finds on the decoded floats. */
bool reorderMesh(SkelMesh& m, u32 packageSize, bool probe) {
  if (m.ordered) return true;
  const u32 n = m.vertexCount;
  if (n < 3 || n % 3 != 0 || m.pos.size() < (size_t)n * 3 || m.nrm.size() < (size_t)n * 4 ||
      m.joints.size() < (size_t)n * 4 || m.weights.size() < (size_t)n * 4) {
    m.ordered = true;
    return true;
  }
  if (probe &&
      !SkelSkin::room(SkelSkin::vectorBytes(n, sizeof(u32)) + firstCornerTableBytes(n)))
    return false;
  std::vector<u32> source(n);
  {
    std::unordered_map<std::string, u32> first;
    first.reserve(n);
    for (u32 v = 0; v < n; v++) {
      char key[20];
      memcpy(key, &m.pos[(size_t)v * 3], 6);
      memcpy(key + 6, &m.nrm[(size_t)v * 4], 3);
      memcpy(key + 9, &m.joints[(size_t)v * 4], 4);
      memcpy(key + 13, &m.weights[(size_t)v * 4], 4);
      source[v] = first.emplace(std::string(key, 17), v).first->second;
    }
  }
  const bool textured = m.uv.size() >= (size_t)n * 2;
  if (probe) {
    // The rest, at its real size: the order, orderTriangles' tables (the unique ids, the
    // adjacency, the candidate list at its doubled worst), and the permuted copies of the arrays.
    size_t uniques = 0;
    for (u32 v = 0; v < n; v++)
      if (source[v] == v) uniques++;
    const size_t tris = n / 3;
    if (!SkelSkin::room(SkelSkin::vectorBytes(tris, sizeof(u32)) +
                        SkelSkin::vectorBytes(n, sizeof(u32)) +
                        SkelSkin::vectorBytes(uniques + 1, sizeof(u32)) +
                        SkelSkin::vectorBytes(n, sizeof(u32)) +
                        SkelSkin::vectorBytes(uniques, sizeof(u32)) +
                        SkelSkin::vectorBytes(tris, 1) +
                        SkelSkin::vectorBytes(uniques, sizeof(u32)) +
                        SkelSkin::vectorBytes(3 * (size_t)n, sizeof(u32)) +
                        SkelSkin::vectorBytes(m.pos.size(), sizeof(s16)) +
                        SkelSkin::vectorBytes(textured ? m.uv.size() : 0, sizeof(s16)) +
                        SkelSkin::vectorBytes(m.nrm.size(), 1) +
                        SkelSkin::vectorBytes(m.joints.size(), 1) +
                        SkelSkin::vectorBytes(m.weights.size(), 1)))
      return false;
  }
  std::vector<u32> order;
  SkelSkin::Indexed::orderTriangles(source.data(), n, packageSize, order);
  if (order.size() != n / 3) {
    m.ordered = true;
    return true;
  }
  std::vector<s16> pos(m.pos.size()), uv(textured ? m.uv.size() : 0);
  std::vector<s8> nrm(m.nrm.size());
  std::vector<u8> joints(m.joints.size()), weights(m.weights.size());
  for (u32 t = 0; t < n / 3; t++)
    for (u32 c = 0; c < 3; c++) {
      const size_t d = (size_t)t * 3 + c, o = (size_t)order[t] * 3 + c;
      memcpy(&pos[d * 3], &m.pos[o * 3], 6);
      memcpy(&nrm[d * 4], &m.nrm[o * 4], 4);
      memcpy(&joints[d * 4], &m.joints[o * 4], 4);
      memcpy(&weights[d * 4], &m.weights[o * 4], 4);
      if (textured) memcpy(&uv[d * 2], &m.uv[o * 2], 4);
    }
  // Back into the arrays the loader made (the model's own budget), not swapped for these
  // temporaries: the permuted copies are charged to the Skinning scope this runs in.
  std::copy(pos.begin(), pos.end(), m.pos.begin());
  std::copy(nrm.begin(), nrm.end(), m.nrm.begin());
  std::copy(joints.begin(), joints.end(), m.joints.begin());
  std::copy(weights.begin(), weights.end(), m.weights.begin());
  if (textured) std::copy(uv.begin(), uv.end(), m.uv.begin());
  m.ordered = true;
  return true;
}

/** Every part's chain, built once per model: level 0 = the full mesh, then
 * the baked (generated or authored) LOD variants. A level keeps its skin source, its extent and
 * its unique set; the per-corner float rows are made on demand (SkelInstance::ensureFloatLevel).
 *
 * Built under the refusing Skinning scope out of throwing containers, so with `probe` each stage
 * first asks the budget for its exact size from the nothrow operator new (the sizes follow from
 * the corner counts and, once the skin sources are known, the unique counts - a flat-shaded model
 * has as many uniques as corners): null, with the model's meshes in a state a later call
 * continues from (`SkelMesh::ordered`), when any stage is refused. Without `probe` it builds as it
 * always did (a caller that has no way to refuse). */
std::shared_ptr<const SkelBindCache> buildBindCache(const SkelModel& model, bool probe = true) {
  using SkelSkin::room;
  using SkelSkin::vectorBytes;
  if (probe) {
    // the cache, its parts and one chain a part
    size_t skeleton = 4 * SkelSkin::kBlockOverhead + sizeof(SkelBindCache) +
                      vectorBytes(model.parts.size(), sizeof(std::vector<SkelBindLod>));
    for (const SkelPart& part : model.parts)
      skeleton += vectorBytes(1 + part.lods.size(), sizeof(SkelBindLod));
    if (!room(skeleton)) return nullptr;
  }
  auto cache = std::make_shared<SkelBindCache>();
  if (SkelSkin::gUseIndexed) {
    // The indexed stream's locality order, before anything reads the meshes: the bind data
    // below, the streams and the checks all use the corner order they are in now.
    for (SkelPart& part : const_cast<SkelModel&>(model).parts) {
      if (!reorderMesh(part, SkelSkin::Indexed::kStreamPackage, probe)) return nullptr;
      for (SkelLod& lod : part.lods)
        if (!reorderMesh(lod, SkelSkin::Indexed::kStreamPackage, probe)) return nullptr;
    }
  }
  cache->parts.resize(model.parts.size());
  auto level = [&model, probe](const SkelMesh& mesh, SkelBindLod& bl) {
    bl.count = mesh.vertexCount;
    if (!bindSkinSource(model.box, mesh, bl.skinSource, probe)) return false;
    SkelSkin::bindExtent(mesh.pos.data(), mesh.joints.data(), mesh.weights.data(),
                         mesh.vertexCount, bl.extent);
    return buildUniqueLod(model.box, mesh, bl, probe);
  };
  for (size_t pi = 0; pi < model.parts.size(); pi++) {
    const SkelPart& part = model.parts[pi];
    auto& chain = cache->parts[pi];
    chain.resize(1 + part.lods.size());
    if (!level(part, chain[0])) return nullptr;
    for (size_t l = 0; l < part.lods.size(); l++)
      if (!level(part.lods[l], chain[1 + l])) return nullptr;
  }
  return cache;
}

/** A level > 0 gets its skin output the first time it is skinned - an
 * instance never seen at that distance never pays for the buffers. They are
 * taken with new (std::nothrow): false, with nothing changed, when the heap
 * has no room for them - or the level's float rows (`rows`) are not held - the
 * caller then draws a level the instance holds (heldLevel) and asks again the
 * next time it skins. */
bool ensureOutput(SkelInstance::PartLod& pl, const SkelBindLod* rows) {
  if (pl.outV) return true;
  if (rows == nullptr) return false;
#if TYRA_MEMORY_MANAGER
  // Added by TyraX: a level's skin output is the Skinning budget's, as the
  // buffers the constructor makes are (docs/memory-manager.md). Skinning
  // refuses when full: the buffers are null and the part draws a level it
  // holds.
  Memory::MemScope memoryScope(Memory::Budget::Skinning);
#endif
  std::unique_ptr<Vec4[]> vertices(new (std::nothrow) Vec4[pl.count]);
  std::unique_ptr<Vec4[]> normals(new (std::nothrow) Vec4[pl.count]);
  if (!vertices || !normals) return false;
  for (u32 v = 0; v < pl.count; v++) {  // bind pose until the first skin
    vertices[v] = rows->bindPositions[v];
    normals[v] = rows->bindNormals[v];
  }
  pl.ownVertices = std::move(vertices);
  pl.ownNormals = std::move(normals);
  pl.outV = pl.ownVertices.get();
  pl.outN = pl.ownNormals.get();
  return true;
}

/** The level a part draws for a wanted `lod` (clamped to its chain): that
 * level when it holds a skin output - made now when `make` (the body's float rows of the part,
 * indexed by level) is given - else the cheapest level it holds one for. Level 0's output is the
 * mesh's own, so there always is one. Skinning (make) and reading (not) answer the same level,
 * since only skinning makes outputs. */
u8 heldLevel(std::vector<SkelInstance::PartLod>& chain, u8 lod,
             const std::vector<std::shared_ptr<const SkelBindLod>>* make) {
  if (lod >= chain.size()) lod = (u8)(chain.size() - 1);
  if (chain[lod].outV != nullptr ||
      (make != nullptr && lod < make->size() && ensureOutput(chain[lod], (*make)[lod].get())))
    return lod;
  for (u8 l = (u8)(chain.size() - 1); l > 0; l--)
    if (chain[l].outV != nullptr) return l;
  return 0;
}

}  // namespace

SkelInstance::SkelInstance(const SkelModel* t_model, bool floatMesh, const SkelLook* look)
    : model(t_model), streamOnly(!floatMesh) {
#if TYRA_MEMORY_MANAGER
  // Added by TyraX: what an instance allocates here - its mesh, its skin
  // output and every LOD's buffers - is the Skinning budget's
  // (docs/memory-manager.md).
  Memory::MemScope memoryScope(Memory::Budget::Skinning);
#endif
  // Single-frame builder data in bind pose; the DynamicMesh takes ownership
  // of the arrays, we keep the pointers and overwrite them every update.
  MeshBuilderData data;
  data.loadNormals = true;
  data.loadLightmap = false;
  // (create() builds it first, refusably; a caller that constructs directly builds it here)
  if (!model->bindCache) model->bindCache = buildBindCache(*model, false);
  bindHold = model->bindCache;
  const SkelBindCache& cache = *bindHold;
  partLods.resize(model->parts.size());
  partLevel.assign(model->parts.size(), 0);
  for (size_t pi = 0; pi < model->parts.size(); pi++) {
    const SkelPart& part = model->parts[pi];
    // The texture this body's look puts on the part ("" = untextured).
    const std::string& texturePath =
        look != nullptr && pi < look->parts.size() ? look->parts[pi].texturePath : part.texturePath;
    auto* material = new MeshBuilderMaterialData();
    if (data.materials.capacity() == 0) data.materials.reserve(model->parts.size());
    data.materials.push_back(material);
    material->name = part.name;
    if (!texturePath.empty()) material->texturePath = texturePath;
    // the single color the VU1 programs modulate with (128 = 1.0)
    material->ambient.set(part.color[0] * 128.0F, part.color[1] * 128.0F,
                          part.color[2] * 128.0F, 128.0F);

    auto* frame = new MeshBuilderMaterialFrameData();
    material->frames.push_back(frame);
    if (streamOnly) {
      // A body that draws render slots only: no float arrays. The mesh needs a frame to make
      // its bounding box from, so the frame is the two corners of the bind pose's box (the box
      // is the same: the frame's box is the min and max of its vertices) and the arrays the
      // mesh insists on are two vertices long.
      Vec4 lo(FLT_MAX, FLT_MAX, FLT_MAX, 1.0F), hi(-FLT_MAX, -FLT_MAX, -FLT_MAX, 1.0F);
      for (u32 v = 0; v < part.vertexCount; v++) {
        float q[3];  // the bind position, decoded from the compact mesh (no copy is kept)
        decodeVertex(model->box, part, v, q, nullptr, nullptr);
        if (q[0] < lo.x) lo.x = q[0];
        if (q[1] < lo.y) lo.y = q[1];
        if (q[2] < lo.z) lo.z = q[2];
        if (q[0] > hi.x) hi.x = q[0];
        if (q[1] > hi.y) hi.y = q[1];
        if (q[2] > hi.z) hi.z = q[2];
      }
      frame->count = 2;
      frame->vertices = new Vec4[2];
      frame->normals = new Vec4[2];
      frame->vertices[0] = lo;
      frame->vertices[1] = hi;
      frame->normals[0].set(0.0F, 1.0F, 0.0F, 1.0F);
      frame->normals[1].set(0.0F, 1.0F, 0.0F, 1.0F);
      if (!texturePath.empty()) {
        frame->textureCoords = new Vec4[2];
        frame->textureCoords[0].set(0.0F, 0.0F, 1.0F, 0.0F);
        frame->textureCoords[1].set(0.0F, 0.0F, 1.0F, 0.0F);
      }
    } else {
    frame->count = part.vertexCount;
    frame->vertices = new Vec4[part.vertexCount];
    frame->normals = new Vec4[part.vertexCount];
    // the bind pose, decoded from the compact mesh (the model's box)
    std::vector<float> pos((size_t)part.vertexCount * 3);
    std::vector<float> nrm((size_t)part.vertexCount * 3);
    std::vector<float> uv(texturePath.empty() ? 0 : (size_t)part.vertexCount * 2);
    decodeMesh(model->box, part, pos.data(), nrm.data(),
               uv.empty() ? nullptr : uv.data());
    for (u32 v = 0; v < part.vertexCount; v++) {
      frame->vertices[v].set(pos[(size_t)v * 3], pos[(size_t)v * 3 + 1],
                             pos[(size_t)v * 3 + 2], 1.0F);
      frame->normals[v].set(nrm[(size_t)v * 3], nrm[(size_t)v * 3 + 1],
                            nrm[(size_t)v * 3 + 2], 1.0F);
    }
    if (!texturePath.empty()) {
      frame->textureCoords = new Vec4[part.vertexCount];
      for (u32 v = 0; v < part.vertexCount; v++)
        frame->textureCoords[v].set(uv[(size_t)v * 2], uv[(size_t)v * 2 + 1],
                                    1.0F, 0.0F);
    }
    }

    // level 0 = the full mesh, skinning straight into the frame arrays
    auto& chain = partLods[pi];
    const auto& shared = cache.parts[pi];
    chain.resize(shared.size());
    for (size_t l = 0; l < shared.size(); l++) {
      chain[l].bind = &shared[l];
      chain[l].count = shared[l].count;
    }
    if (!streamOnly) {
      chain[0].outV = frame->vertices;
      chain[0].outN = frame->normals;
      chain[0].uvPtr = frame->textureCoords;  // nullptr when untextured
    }

    // deeper levels: baked decimated variants - their own skin buffers and packed UVs
    // (ensureOutput, ensureFloatLevel: on first use)
    if (chain.size() > maxLodLevels) maxLodLevels = (u8)chain.size();
  }
  mesh = std::make_unique<DynamicMesh>(&data);
  // The two-vertex stand-ins are the mesh's to keep (its frames own them); nothing reads them.
  floatMeshReleased = streamOnly;

  localsCur.resize(model->nodes.size() * 10);
  localsPrev.resize(model->nodes.size() * 10);
  animatedCur.resize(model->nodes.size());
  animatedPrev.resize(model->nodes.size());
  globals.resize(model->nodes.size());
  palette.resize(model->palette.size());
  // Blend-space sample layers switch clips at runtime: their cursor arrays
  // are reserved for the longest clip now, so a switch never allocates.
  size_t maxChannels = 0, maxSamples = 0;
  for (const SkelClip& c : model->clips)
    if (c.channels.size() > maxChannels) maxChannels = c.channels.size();
  for (const SkelBlendSpace& b : model->blendSpaces)
    if (b.samples.size() > maxSamples) maxSamples = b.samples.size();
  if (maxSamples > 0) {
    for (Layer& l : bs) l.cursors.reserve(maxChannels);
    // the frozen pose a mode or blend-LOD switch fades from (~1.7 KB for 41
    // nodes): sized now, so no switch allocates mid-game
    localsFrom.resize(localsCur.size());
    animatedFrom.resize(animatedCur.size());
  }
  bsOwnClock.assign(maxSamples, 0.0F);

  play(0, true, 0.0F);
}

SkelInstance::~SkelInstance() {
  // The pose buffers go back to their level's pool; the pool outlives the instance.
  for (auto& chain : partLods)
    for (PartLod& pl : chain) dropIndexed(pl);
}

size_t SkelInstance::bytesFor(const SkelModel& model, bool floatMesh) {
  // Per allocation, a heap header and alignment slack; per part, the builder
  // material and frame, the mesh's material, frame and boxes, the names.
  const size_t kPerPart = 1024;
  const size_t kFixed = 8 * 1024;  // the mesh, its frame, a log line, vectors
  size_t bytes = kFixed;
  size_t largestLevel = 0;  // the most vertices one level of one part has
  for (const SkelPart& part : model.parts) {
    const size_t arrays = part.texturePath.empty() ? 2 : 3;
    bytes += (floatMesh ? (size_t)part.vertexCount : (size_t)2) * sizeof(Vec4) * arrays + kPerPart;
    if (!model.bindCache) {
      // The bind data of every level (SkelBindLod): the skin source index a corner and the
      // unique level - at most a quarter of the corners, each with its position, normal and
      // weights as Vec4s, joints, influences, the identity skin source and its corner index.
      size_t vertices = part.vertexCount;
      for (const SkelLod& lod : part.lods) vertices += lod.vertexCount;
      bytes += vertices * (4 + (3 * sizeof(Vec4) + 4 + 1 + 4 + 4 + 3) / 4) +
               (part.lods.size() + 1) * kPerPart;
      if (part.vertexCount > largestLevel) largestLevel = part.vertexCount;
      for (const SkelLod& lod : part.lods)
        if (lod.vertexCount > largestLevel) largestLevel = lod.vertexCount;
    }
  }
  if (!model.bindCache) {
    // Added by TyraX: and while it is built, the temporary table of first
    // corners - one level at a time, so the largest level's.
    bytes += firstCornerTableBytes(largestLevel);
  }
  const size_t nodes = model.nodes.size();
  // locals cur / prev (10 floats each), animated cur / prev, globals.
  bytes += nodes * (2 * 10 * sizeof(float) + 2 + sizeof(M4x4));
  bytes += model.palette.size() * sizeof(M4x4);
  size_t channels = 0;
  for (const SkelClip& clip : model.clips)
    if (clip.channels.size() > channels) channels = clip.channels.size();
  bytes += channels * sizeof(u32) * 2;  // the playing and the fading layer
  // Added by TyraX (blend spaces): a model with blend spaces also reserves
  // its sample layers' cursors, the frozen pose a switch fades from and a
  // clock per sample.
  size_t samples = 0;
  for (const SkelBlendSpace& b : model.blendSpaces)
    if (b.samples.size() > samples) samples = b.samples.size();
  if (samples > 0)
    bytes += 3 * channels * sizeof(u32) + nodes * (10 * sizeof(float) + 1) +
             samples * sizeof(float) + 4 * kPerPart;
  return bytes;
}

std::unique_ptr<SkelInstance> SkelInstance::create(const SkelModel* model, bool floatMesh,
                                                   const SkelLook* look) {
  if (model == nullptr) return nullptr;
  if (!model->bindCache) {
#if TYRA_MEMORY_MANAGER
    // Added by TyraX: the model's bind data is the first body's to build, out of throwing
    // containers in the Skinning budget: each stage asks for its exact size first
    // (buildBindCache), so a full budget refuses the body - the caller asks again later - and
    // never halts. The sizes depend on the model's unique corners (a flat-shaded model has as
    // many as it has corners), which no estimate made before the cache exists can know.
    Memory::MemScope memoryScope(Memory::Budget::Skinning);
#endif
    model->bindCache = buildBindCache(*model);
    if (!model->bindCache) {
#if TYRA_MEMORY_MANAGER
      Memory::Manager::instance().noteFallback(
          "skinned instance: no bind data yet",
          model->parts.empty() ? nullptr : model->parts[0].name.c_str(), 0, model);
#endif
      return nullptr;
    }
  }
  const size_t bytes = bytesFor(*model, floatMesh);
  {
#if TYRA_MEMORY_MANAGER
    // Added by TyraX: the probe asks the budget the constructor's allocations
    // are charged to (Skinning, which refuses when full), so a probe that
    // passes means the constructor's own requests are not refused either.
    Memory::MemScope memoryScope(Memory::Budget::Skinning);
#endif
    // The room the constructor will take, a quarter and 16 KB over: refused,
    // nothing is made. A call to operator new, not a new-expression: a
    // compiler may drop a new-expression whose block is never used - and
    // fold its null check to "not null" - so that probe would always pass.
    void* room = ::operator new[](bytes + bytes / 4 + 16 * 1024, std::nothrow);
    if (room == nullptr) {
#if TYRA_MEMORY_MANAGER
      // Added by TyraX: say so - once a second per model (the key).
      Memory::Manager::instance().noteFallback(
          "skinned instance: no body yet",
          model->parts.empty() ? nullptr : model->parts[0].name.c_str(), 0,
          model);
#endif
      return nullptr;
    }
    ::operator delete[](room);
  }
  return std::unique_ptr<SkelInstance>(new SkelInstance(model, floatMesh, look));
}

void SkelInstance::play(u32 clip, bool loop, float fadeSeconds) {
  if (clip >= model->clips.size()) clip = 0;
  if (model->clips.empty()) return;

  if (bsSpace >= 0) {
    // Leaving blend-space mode: the samples are not one layer prev could
    // keep, so the fade starts from the frozen blended pose instead.
    if (fadeSeconds > 0.0F) freezeFrom(fadeSeconds);
    bsSpace = -1;
    bsCount = 0;
    prev.clip = -1;
    fadeT = 1.0F;
  } else if (fadeSeconds > 0.0F && cur.clip >= 0) {
    prev = cur;  // keeps its own time/loop/cursors alive during the fade
    fadeDuration = fadeSeconds;
    fadeT = 0.0F;
  } else {
    prev.clip = -1;
    fadeT = 1.0F;
  }
  cur.clip = (s32)clip;
  cur.time = 0.0F;
  cur.loop = loop;
  cur.held = false;
  cur.cursors.assign(model->clips[clip].channels.size(), 0);
  oneShotDone = false;
  poseDirty = globalsStale = clipStale = true;
}

void SkelInstance::seek(float seconds) {
  if (cur.clip < 0) return;
  const SkelClip& clip = model->clips[cur.clip];
  if (seconds < 0.0F) seconds = 0.0F;
  if (seconds > clip.duration) seconds = clip.duration;
  cur.held = true;  // a held clip keeps its place while it fades out, too
  if (seconds == cur.time) return;
  cur.time = seconds;  // findKey rewinds its cursors on a backward jump
  poseDirty = globalsStale = clipStale = true;
}

void SkelInstance::setBlend(s32 clip, float seconds, float weight) {
  if (clip < 0 || clip >= (s32)model->clips.size() || weight <= 0.0F) {
    if (aux.clip >= 0) poseDirty = globalsStale = clipStale = true;  // the layer just switched off
    aux.clip = -1;
    auxWeight = 0.0F;
    return;
  }
  const SkelClip& c = model->clips[clip];
  if (seconds < 0.0F) seconds = 0.0F;
  if (seconds > c.duration) seconds = c.duration;
  if (weight > 1.0F) weight = 1.0F;
  if (aux.clip != clip) {
    aux.clip = clip;
    aux.loop = false;
    aux.cursors.assign(c.channels.size(), 0);
    poseDirty = globalsStale = clipStale = true;
  }
  if (aux.time != seconds || auxWeight != weight) poseDirty = globalsStale = clipStale = true;
  aux.time = seconds;
  auxWeight = weight;
}

void SkelInstance::advanceLayer(Layer& layer, float dt) {
  if (layer.clip < 0) return;
  const SkelClip& clip = model->clips[layer.clip];
  if (clip.duration <= 0.0F) return;
  layer.time += dt;
  if (layer.time >= clip.duration) {
    if (layer.loop) {
      layer.time = fmodf(layer.time, clip.duration);
      for (u32& c : layer.cursors) c = 0;
      if (&layer == &cur) oneShotDone = false;  // fresh end if loop turns off
    } else {
      layer.time = clip.duration;
    }
  }
}

void SkelInstance::advanceFade(float dt) {
  advanceFrom(dt);
  if (dt <= 0.0F || fadeT >= 1.0F) return;
  if (!prev.held) advanceLayer(prev, dt);  // the fading-out clip keeps moving
  fadeT += fadeDuration > 0.0F ? dt / fadeDuration : 1.0F;
  if (fadeT >= 1.0F) {
    fadeT = 1.0F;
    prev.clip = -1;
  }
  poseDirty = globalsStale = clipStale = true;
}

bool SkelInstance::advance(float dt) {
  bool finished = false;
  if (cur.clip >= 0 && dt > 0.0F) {
    const SkelClip& clip = model->clips[cur.clip];
    if (clip.duration > 0.0F) {
      // the animFinished contract: once for one-shots, every wrap for loops
      const float next = cur.time + dt;
      if (next >= clip.duration)
        finished = cur.loop ? true : !oneShotDone;
      if (!cur.loop && next >= clip.duration) oneShotDone = true;
      advanceLayer(cur, dt);
      poseDirty = globalsStale = clipStale = true;
    }
    advanceFade(dt);
  }
  return finished;
}

bool SkelInstance::ensurePose(u8 lod) {
  if (streamOnly) return false;  // no float arrays to skin into: render slots only
  if (lod >= maxLodLevels) lod = maxLodLevels - 1;
  // An overridden pose changes only when the game writes a new one - the
  // clips' bookkeeping (advance/seek) does not dirty it.
  const bool dirty = overridden ? overrideDirty : poseDirty;
  if (!dirty && lod == lastSkinnedLod) return false;
  // a pure LOD switch reuses the current palette - only the skin reruns
#if TYRA_SKEL_PROFILE
  const u32 t0 = skelTicks();
#endif
  if (dirty) evalPose();
#if TYRA_SKEL_PROFILE
  const u32 t1 = skelTicks();
#endif
  skinParts(lod);
#if TYRA_SKEL_PROFILE
  const u32 t2 = skelTicks();
  profilePose += t1 - t0;
  profileSkin += t2 - t1;
  if (++profileCount == 100) {
    char msg[192];
    snprintf(msg, sizeof(msg), "SKELTIME instance=%p nodes=%u lod=%u pose=%.3f skin=%.3f ms",
             this, (unsigned)model->nodes.size(), lod,
             profilePose / 29491200.0, profileSkin / 29491200.0);
    TYRA_LOG(msg);
    profileCount = profilePose = profileSkin = 0;
  }
#endif
  poseDirty = false;
  overrideDirty = false;
  lastSkinnedLod = lod;
  return true;
}

bool SkelInstance::update(float dt) {
  const bool finished = advance(dt);
  ensurePose(lastSkinnedLod);
  return finished;
}

s32 SkelInstance::findNode(const char* name) const {
  if (!name || !name[0]) return -1;
  for (size_t i = 0; i < model->nodes.size(); i++)
    if (model->nodes[i].name == name) return (s32)i;
  return -1;
}

size_t SkelInstance::nodeCount() const { return model ? model->nodes.size() : 0; }

void SkelInstance::adoptPose(const SkelInstance& owner) {
  if (&owner == this || owner.model != model || !owner.posedOnce ||
      owner.globals.size() != globals.size())
    return;
  globals = owner.globals;
  posedOnce = true;
  globalsStale = false;  // these ARE the pose it is drawn with
  adoptedBehind = owner.globalsStale || owner.adoptedBehind;  // ...and as old as theirs
}

bool SkelInstance::setPoseOverride(const M4x4* nodeGlobals, size_t count) {
  if (!nodeGlobals || !model || count != model->nodes.size()) return false;
  if (!overridden) {
    overrideGlobals.resize(count);
    overridden = true;
    overrideDirty = true;
  } else if (!overrideDirty &&
             memcmp(overrideGlobals.data(), nodeGlobals, count * sizeof(M4x4)) != 0) {
    overrideDirty = true;
  }
  memcpy(static_cast<void*>(overrideGlobals.data()), nodeGlobals,
         count * sizeof(M4x4));
  if (overrideDirty) globalsStale = true;
  return true;
}

void SkelInstance::clearPoseOverride() {
  if (!overridden) return;
  overridden = false;
  overrideDirty = false;
  poseDirty = globalsStale = clipStale = true;  // back to the clips' pose
}

bool SkelInstance::nodeGlobal(s32 node, M4x4* out) const {
  if (!posedOnce || node < 0 || (size_t)node >= globals.size()) return false;
  *out = globals[node];
  return true;
}

bool SkelInstance::currentNodeGlobal(s32 node, M4x4* out) {
  if (node < 0 || (size_t)node >= globals.size()) return false;
  if (!posedOnce || (following && globalsStale)) evalPose();
  return nodeGlobal(node, out);
}

bool SkelInstance::vertexModel(const float bindPos[3], float out[3],
                               float maxDist) {
  // The scan decodes each bind position from the compact mesh as it goes: no decoded copy of the
  // bind pose is kept for this reader, and nothing is allocated.
  // nearest bind vertex (duplicated corners of one vertex skin identically)
  const SkelPart* bestPart = nullptr;
  u32 best = 0;
  float bestD2 = maxDist * maxDist;
  for (size_t pi = 0; pi < model->parts.size(); pi++) {
    const SkelPart& part = model->parts[pi];
    for (u32 v = 0; v < part.vertexCount; v++) {
      float q[3];
      decodeVertex(model->box, part, v, q, nullptr, nullptr);
      const float dx = q[0] - bindPos[0], dy = q[1] - bindPos[1],
                  dz = q[2] - bindPos[2];
      const float d2 = dx * dx + dy * dy + dz * dz;
      if (d2 <= bestD2) bestD2 = d2, bestPart = &part, best = v;
    }
  }
  if (!bestPart) return false;
  // The current pose; ensurePose re-evaluates it (poseDirty stays as is), so
  // the renderer's skinning is unaffected.
  if (poseDirty || !posedOnce) evalPose();
  float p[3];
  decodeVertex(model->box, *bestPart, best, p, nullptr, nullptr);
  const u8* j = &bestPart->joints[(size_t)best * 4];
  const u8* w = &bestPart->weights[(size_t)best * 4];
  float sum = 0.0F;
  out[0] = out[1] = out[2] = 0.0F;
  for (int k = 0; k < 4; k++) {
    if (!w[k]) continue;
    const float* m = palette[j[k]].data;  // column-major
    const float wk = (float)w[k];
    for (int r = 0; r < 3; r++)
      out[r] += wk * (m[r] * p[0] + m[4 + r] * p[1] + m[8 + r] * p[2] + m[12 + r]);
    sum += wk;
  }
  if (sum <= 0.0F) return false;
  for (int r = 0; r < 3; r++) out[r] /= sum;
  return true;
}

SkelInstance::LodArrays SkelInstance::lodArrays(size_t part, u8 lod) {
  auto& chain = partLods[part];
  // The level skinParts skinned for this request - the wanted one, or the
  // cheapest held when the heap had no room for the wanted one's output.
  lod = heldLevel(chain, lod, nullptr);
  PartLod& pl = chain[lod];
  return {pl.outV, pl.outN, pl.uvPtr, pl.count, lod};
}

void SkelInstance::evalLocals(Layer& layer, std::vector<float>& locals,
                              std::vector<u8>& animated) {
  const size_t nodeCount = model->nodes.size();
  for (size_t i = 0; i < nodeCount; i++) {
    const SkelNode& node = model->nodes[i];
    float* l = &locals[i * 10];
    memcpy(l, node.t, 3 * sizeof(float));
    memcpy(l + 3, node.r, 4 * sizeof(float));
    memcpy(l + 7, node.s, 3 * sizeof(float));
    animated[i] = 0;
  }
  if (layer.clip < 0) return;
  const SkelClip& clip = model->clips[layer.clip];
  for (size_t c = 0; c < clip.channels.size(); c++) {
    const SkelChannel& ch = clip.channels[c];
    float* l = &locals[(size_t)ch.node * 10];
    float* dst = ch.path == 0 ? l : (ch.path == 1 ? l + 3 : l + 7);
    SkelClipCodec::sample(clip, ch, layer.time, &layer.cursors[c], dst);
    animated[ch.node] = 1;
  }
}

void SkelInstance::blendLocals(const std::vector<float>& from,
                               const std::vector<u8>& fromAnimated, float w) {
  // Nodes only one clip animates blend against the other pose's bind
  // values, which is what a full-pose blend means.
  const size_t nodeCount = model->nodes.size();
  for (size_t i = 0; i < nodeCount; i++) {
    float* a = &localsCur[i * 10];    // blend target (in place), weight w
    const float* b = &from[i * 10];   // the other pose, weight 1 - w
    for (int c = 0; c < 3; ++c) {
      a[c] = b[c] + (a[c] - b[c]) * w;          // translation
      a[7 + c] = b[7 + c] + (a[7 + c] - b[7 + c]) * w;  // scale
    }
    float dot = 0.0F;
    for (int c = 0; c < 4; ++c) dot += a[3 + c] * b[3 + c];
    const float sign = dot < 0.0F ? -1.0F : 1.0F;  // short arc
    float len = 0.0F;
    for (int c = 0; c < 4; ++c) {
      a[3 + c] = sign * b[3 + c] + (a[3 + c] - sign * b[3 + c]) * w;
      len += a[3 + c] * a[3 + c];
    }
    len = sqrtf(len);
    if (len > 1e-6F)
      for (int c = 0; c < 4; ++c) a[3 + c] /= len;
    else
      a[6] = 1.0F;  // w component
    animatedCur[i] |= fromAnimated[i];
  }
}

void SkelInstance::evalGlobals() {
  posedOnce = true;  // the globals below are a real pose from here on
  globalsStale = false;
  adoptedBehind = false;
  if (overridden) {
    // The game's pose (setPoseOverride): its model-space node matrices ARE
    // the globals.
    memcpy(static_cast<void*>(globals.data()), overrideGlobals.data(),
           globals.size() * sizeof(M4x4));
    return;
  }
  evalClipGlobals(globals);
}

void SkelInstance::evalPose() {
  evalGlobals();
  for (size_t j = 0; j < model->palette.size(); j++)
    mulM4Vu0(palette[j].data, globals[model->palette[j].node].data,
             model->palette[j].ibm);
}

bool SkelInstance::refreshBones() {
  if (posedOnce && !globalsStale && !adoptedBehind) return false;
  // Bones only: poseDirty stays as it is, so the skin's own turn still
  // evaluates the palette (ensurePose / skinInto / vertexModel test it).
  evalGlobals();
  return true;
}

bool SkelInstance::clipNodeGlobal(s32 node, M4x4* out) {
  if (node < 0 || (size_t)node >= globals.size()) return false;
  if (!overridden) return currentNodeGlobal(node, out);
  if (clipGlobals.size() != globals.size()) {
    clipGlobals.resize(globals.size());
    clipStale = true;
  }
  if (clipStale) {
    evalClipGlobals(clipGlobals);
    clipStale = false;
  }
  *out = clipGlobals[node];
  return true;
}

void SkelInstance::evalBaseLocals() {
  if (bsSpace >= 0 && bsCount > 0) {
    // Blend-space mode: the shown samples folded one by one - after k of
    // them localsCur is their normalised blend, so the next one enters with
    // its weight over the running total (the same nlerp as the crossfade).
    evalLocals(bs[0], localsCur, animatedCur);
    float total = bsWeight[0];
    for (int k = 1; k < bsCount; k++) {
      evalLocals(bs[k], localsPrev, animatedPrev);
      const float next = total + bsWeight[k];
      blendLocals(localsPrev, animatedPrev, next > 0.0F ? total / next : 1.0F);
      total = next;
    }
  } else {
    evalLocals(cur, localsCur, animatedCur);
    if (prev.clip >= 0 && fadeT < 1.0F) {
      // Crossfade: blend the two poses' local transforms (nlerp rotations),
      // then walk the hierarchy once.
      evalLocals(prev, localsPrev, animatedPrev);
      blendLocals(localsPrev, animatedPrev, fadeT);  // 0 = all prev, 1 = all cur
    }
  }
  if (fromT < 1.0F && localsFrom.size() == localsCur.size())
    blendLocals(localsFrom, animatedFrom, fromT);  // 0 = all the frozen pose
}

void SkelInstance::evalClipGlobals(std::vector<M4x4>& out) {
  evalBaseLocals();
  if (aux.clip >= 0 && auxWeight > 0.0F) {
    // The held layer rides on top of the (cross-faded) playing pose. The
    // prev scratch is free again by now, so it samples into that.
    evalLocals(aux, localsPrev, animatedPrev);
    blendLocals(localsPrev, animatedPrev, 1.0F - auxWeight);
  }

  // globals, parents-first; matrix nodes are never animated (glTF spec),
  // an animated flag overrides just in case a file breaks that rule
  for (u32 i : model->order) {
    const SkelNode& node = model->nodes[i];
    alignas(16) float local[16];  // mulM4Vu0 loads it with lqc2
    const float* localPtr = local;
    if (node.hasMatrix && !animatedCur[i]) {
      localPtr = node.matrix;
    } else {
      const float* l = &localsCur[(size_t)i * 10];
      fromTrs(local, l, l + 3, l + 7);
    }
    if (node.parent >= 0)
      mulM4Vu0(out[i].data, out[node.parent].data, localPtr);
    else
      memcpy(out[i].data, localPtr, 16 * sizeof(float));
  }
}

// The weighted palette matrix of one corner, left in $vf5-$vf8 for the tail that transforms with
// it (the dispatch of skinBlendCorners, shared by the extent and the fused passes). `j` the
// corner's four palette slots, `n` its influence count, `wq` its weights (a Vec4). Inlined: the
// VU0 registers carry the matrix into the asm that follows.
static inline __attribute__((always_inline)) void blendMatrix(const M4x4* pal, const Vec4* wq,
                                                              const u8* j, u8 n) {
  if (n == 2) {
    // two influences - the common case for smooth skinning
    const float* p0 = pal[j[0]].data;
    const float* p1 = pal[j[1]].data;
    asm volatile(
        "lqc2         $vf9, 0x00(%[wgt])       \n\t"
        "lqc2         $vf1, 0x00(%[p0])        \n\t"
        "lqc2         $vf2, 0x00(%[p1])        \n\t"
        "vmulax.xyzw  $ACC, $vf1, $vf9         \n\t"
        "vmaddy.xyzw  $vf5, $vf2, $vf9         \n\t"
        "lqc2         $vf1, 0x10(%[p0])        \n\t"
        "lqc2         $vf2, 0x10(%[p1])        \n\t"
        "vmulax.xyzw  $ACC, $vf1, $vf9         \n\t"
        "vmaddy.xyzw  $vf6, $vf2, $vf9         \n\t"
        "lqc2         $vf1, 0x20(%[p0])        \n\t"
        "lqc2         $vf2, 0x20(%[p1])        \n\t"
        "vmulax.xyzw  $ACC, $vf1, $vf9         \n\t"
        "vmaddy.xyzw  $vf7, $vf2, $vf9         \n\t"
        "lqc2         $vf1, 0x30(%[p0])        \n\t"
        "lqc2         $vf2, 0x30(%[p1])        \n\t"
        "vmulax.xyzw  $ACC, $vf1, $vf9         \n\t"
        "vmaddy.xyzw  $vf8, $vf2, $vf9         \n\t"
        :
        : [wgt] "r"(wq), [p0] "r"(p0), [p1] "r"(p1)
        : "memory");
  } else if (n == 1) {
    // single influence - weight is exactly 1, use the matrix as-is
    const float* p0 = pal[j[0]].data;
    asm volatile(
        "lqc2         $vf5, 0x00(%[p0])        \n\t"
        "lqc2         $vf6, 0x10(%[p0])        \n\t"
        "lqc2         $vf7, 0x20(%[p0])        \n\t"
        "lqc2         $vf8, 0x30(%[p0])        \n\t"
        :
        : [p0] "r"(p0)
        : "memory");
  } else if (n >= 3) {
    // 3 or 4 influences - blend all four slots (a 0-weight 4th slot
    // contributes exactly 0; the loader validated every index)
    const float* p0 = pal[j[0]].data;
    const float* p1 = pal[j[1]].data;
    const float* p2 = pal[j[2]].data;
    const float* p3 = pal[j[3]].data;
    asm volatile(
        "lqc2         $vf9, 0x00(%[wgt])       \n\t"
        "lqc2         $vf1, 0x00(%[p0])        \n\t"
        "lqc2         $vf2, 0x00(%[p1])        \n\t"
        "lqc2         $vf3, 0x00(%[p2])        \n\t"
        "lqc2         $vf4, 0x00(%[p3])        \n\t"
        "vmulax.xyzw  $ACC, $vf1, $vf9         \n\t"
        "vmadday.xyzw $ACC, $vf2, $vf9         \n\t"
        "vmaddaz.xyzw $ACC, $vf3, $vf9         \n\t"
        "vmaddw.xyzw  $vf5, $vf4, $vf9         \n\t"
        "lqc2         $vf1, 0x10(%[p0])        \n\t"
        "lqc2         $vf2, 0x10(%[p1])        \n\t"
        "lqc2         $vf3, 0x10(%[p2])        \n\t"
        "lqc2         $vf4, 0x10(%[p3])        \n\t"
        "vmulax.xyzw  $ACC, $vf1, $vf9         \n\t"
        "vmadday.xyzw $ACC, $vf2, $vf9         \n\t"
        "vmaddaz.xyzw $ACC, $vf3, $vf9         \n\t"
        "vmaddw.xyzw  $vf6, $vf4, $vf9         \n\t"
        "lqc2         $vf1, 0x20(%[p0])        \n\t"
        "lqc2         $vf2, 0x20(%[p1])        \n\t"
        "lqc2         $vf3, 0x20(%[p2])        \n\t"
        "lqc2         $vf4, 0x20(%[p3])        \n\t"
        "vmulax.xyzw  $ACC, $vf1, $vf9         \n\t"
        "vmadday.xyzw $ACC, $vf2, $vf9         \n\t"
        "vmaddaz.xyzw $ACC, $vf3, $vf9         \n\t"
        "vmaddw.xyzw  $vf7, $vf4, $vf9         \n\t"
        "lqc2         $vf1, 0x30(%[p0])        \n\t"
        "lqc2         $vf2, 0x30(%[p1])        \n\t"
        "lqc2         $vf3, 0x30(%[p2])        \n\t"
        "lqc2         $vf4, 0x30(%[p3])        \n\t"
        "vmulax.xyzw  $ACC, $vf1, $vf9         \n\t"
        "vmadday.xyzw $ACC, $vf2, $vf9         \n\t"
        "vmaddaz.xyzw $ACC, $vf3, $vf9         \n\t"
        "vmaddw.xyzw  $vf8, $vf4, $vf9         \n\t"
        :
        : [wgt] "r"(wq), [p0] "r"(p0), [p1] "r"(p1), [p2] "r"(p2),
          [p3] "r"(p3)
        : "memory");
  } else {
    // all-zero weights - zero matrix collapses the vertex to the
    // origin, the same degenerate result the stage-1 baker produced
    asm volatile(
        "vsub.xyzw    $vf5, $vf0, $vf0         \n\t"
        "vsub.xyzw    $vf6, $vf0, $vf0         \n\t"
        "vsub.xyzw    $vf7, $vf0, $vf0         \n\t"
        "vsub.xyzw    $vf8, $vf0, $vf0         \n\t" ::);
  }
}

// The VU0 blend of one part's corners: the body of skinParts' corner loop,
// shared with the compact stream path (SkelInstance::skinInto), which blends
// into float scratch and quantizes it. $vf20/$vf21 carry the running AABB
// across the loop: the caller sets them (skinParts its box; the stream path any
// value, nothing reads them). Model-space outputs, as skinParts writes them.
//
// skinBlendRange blends the corners [first, first + count) into outV / outN [0, count): the whole
// part when first is 0, a window for the fused pass's self-check. A repeat corner copies its
// source's output, so a window holds the sources of its repeats (a unique level has none).
static void skinBlendRange(const M4x4* pal, const SkelBindLod& bind, u32 first, u32 count,
                           Vec4* outV, Vec4* outN) {
  const Vec4* srcP = bind.bindPositions.data();
  const Vec4* srcN = bind.bindNormals.data();
  const Vec4* wq = bind.skinWeights.data();
  const u8* joints = bind.sortedJoints.data();
  const u8* infl = bind.influences.data();
  const u32* skinSource = bind.skinSource.empty() ? nullptr : bind.skinSource.data();
  outV -= first;
  outN -= first;

  for (u32 v = first; v < first + count; v++) {
    const u32 source = skinSource ? skinSource[v] : v;
    if (source != v) {
      // No Vec4 helper here: VU0's running AABB must survive the copy.
      asm volatile(
          "lqc2 $vf1, 0(%[srcv]) \n\t"
          "lqc2 $vf2, 0(%[srcn]) \n\t"
          "sqc2 $vf1, 0(%[dstv]) \n\t"
          "sqc2 $vf2, 0(%[dstn]) \n\t"
          : : [srcv] "r"(outV + source), [srcn] "r"(outN + source),
              [dstv] "r"(outV + v), [dstn] "r"(outN + v) : "memory");
      continue;
    }
    const u8* j = &joints[(size_t)v * 4];
    const u8 n = infl[v];

    blendMatrix(pal, wq + v, j, n);

    // shared tail: transform by $vf5-$vf8, normalize, fold the AABB
    asm volatile(
        // position (x, y, z, 1) -> $vf11, force w = 1
        "lqc2         $vf10, 0x00(%[pos])      \n\t"
        "vmulax.xyzw  $ACC, $vf5, $vf10        \n\t"
        "vmadday.xyzw $ACC, $vf6, $vf10        \n\t"
        "vmaddaz.xyzw $ACC, $vf7, $vf10        \n\t"
        "vmaddw.xyzw  $vf11, $vf8, $vf10       \n\t"
        "vmulw.w      $vf11, $vf0, $vf0        \n\t"
        // normal (nx, ny, nz, 0) -> $vf12 (w = 0 drops the translation)
        "lqc2         $vf12, 0x00(%[nrm])      \n\t"
        "vmulax.xyzw  $ACC, $vf5, $vf12        \n\t"
        "vmadday.xyzw $ACC, $vf6, $vf12        \n\t"
        "vmaddaz.xyzw $ACC, $vf7, $vf12        \n\t"
        "vmaddw.xyzw  $vf12, $vf8, $vf12       \n\t"
        // 1 / sqrt(len^2 + eps); AABB fold hides the vrsqrt latency
        "vmul.xyz     $vf13, $vf12, $vf12      \n\t"
        "vaddy.x      $vf13, $vf13, $vf13      \n\t"
        "vaddz.x      $vf13, $vf13, $vf13      \n\t"
        "vaddw.x      $vf13, $vf13, $vf20      \n\t"
        "vrsqrt       $Q, $vf0w, $vf13x        \n\t"
        "vmini.xyz    $vf20, $vf20, $vf11      \n\t"
        "vmax.xyz     $vf21, $vf21, $vf11      \n\t"
        "sqc2         $vf11, 0x00(%[outv])     \n\t"
        "vwaitq                                \n\t"
        "vmulq.xyz    $vf12, $vf12, $Q         \n\t"
        "vmulw.w      $vf12, $vf0, $vf0        \n\t"
        "sqc2         $vf12, 0x00(%[outn])     \n\t"
        :
        : [outv] "r"(outV + v), [outn] "r"(outN + v), [pos] "r"(srcP + v),
          [nrm] "r"(srcN + v)
        : "memory");
  }
}

static void skinBlendCorners(const M4x4* pal, const SkelBindLod& bind, u32 count,
                             Vec4* outV, Vec4* outN) {
  skinBlendRange(pal, bind, 0, count, outV, outN);
}

void SkelInstance::skinParts(u8 lod) {
  // The whole per-vertex job runs on VU0 in macro mode (the era-correct
  // split: animation on VU0, 3D on VU1, game code on EE): build the blended
  // palette matrix in $vf5-$vf8 - dispatching on the vertex's influence
  // count so 1- and 2-bone vertices (the vast majority) pay for exactly
  // that many matrix loads - then transform position and normal, normalize
  // the normal with vrsqrt, and fold the skinned AABB with vmini/vmax.
  //
  // VU0 register state deliberately spans the asm blocks below: $vf5-$vf8
  // carry the matrix from the blend block into the shared tail, and
  // $vf20/$vf21 hold the running AABB across the whole loop ($vf20.w
  // carries the epsilon added to squared normal lengths, so a degenerate
  // zero normal divides by sqrt(eps) and comes out ~0 instead of blowing
  // up - the EE loop's `len > 1e-6` guard). That is safe for the same
  // reason every M4x4/Vec4 helper is: GCC never emits COP2 code of its
  // own, and nothing else in the engine runs VU0 concurrently. Don't call
  // anything between these blocks.
  // (first-use output buffers are allocated here, before the VU0 state
  // below is set up; a part whose wanted level's buffers the heap could not
  // supply skins - and draws - the cheapest level it holds instead)
  for (size_t pi = 0; pi < partLods.size(); pi++) {
    // The wanted level's float rows first: its skin output is made from them, and a refusal
    // (no room for the rows) is the same as a refusal of the output - the part draws a level it
    // holds. Level 0's output is the mesh's own, so its rows are held when it is the one drawn.
    ensureFloatLevel(pi, lod < partLods[pi].size() ? lod : (u8)(partLods[pi].size() - 1));
    partLevel[pi] = heldLevel(partLods[pi], lod, &floatBinds[pi]);
    if (!floatBinds[pi][partLevel[pi]]) ensureFloatLevel(pi, partLevel[pi]);
#if TYRA_MEMORY_MANAGER
    // Added by TyraX: the wanted level's output was refused - say which level
    // was drawn instead (Memory::Manager::noteFallback, once a second per
    // model, part and level: the key is their shared bind data).
    const u8 wanted =
        lod < partLods[pi].size() ? lod : (u8)(partLods[pi].size() - 1);
    if (partLevel[pi] != wanted) {
      char what[48];
      snprintf(what, sizeof what, "skin output: drew lod %u",
               static_cast<unsigned>(partLevel[pi]));
      Memory::Manager::instance().noteFallback(
          what, model->parts[pi].name.c_str(), wanted,
          partLods[pi][wanted].bind);
    }
#endif
  }
  float bmin[4] alignas(16) = {FLT_MAX, FLT_MAX, FLT_MAX, 1e-12F};
  float bmax[4] alignas(16) = {-FLT_MAX, -FLT_MAX, -FLT_MAX, 0.0F};
  asm volatile(
      "lqc2         $vf20, 0x00(%[bmin])     \n\t"
      "lqc2         $vf21, 0x00(%[bmax])     \n\t"
      :
      : [bmin] "r"(bmin), [bmax] "r"(bmax)
      : "memory");  // reads the two initializers above (TyraX)

  const M4x4* pal = palette.data();
  for (size_t pi = 0; pi < partLods.size(); pi++) {
    const auto& chain = partLods[pi];
    const PartLod& plod = chain[partLevel[pi]];
    const SkelBindLod* rows = floatBinds[pi][partLevel[pi]].get();
    if (rows == nullptr) continue;  // no room for the rows: the part keeps the pose it holds
    skinBlendCorners(pal, *rows, plod.count, plod.outV, plod.outN);
  }

  asm volatile(
      "sqc2         $vf20, 0x00(%[bmin])     \n\t"
      "sqc2         $vf21, 0x00(%[bmax])     \n\t"
      :
      : [bmin] "r"(bmin), [bmax] "r"(bmax)
      : "memory");

  // Refresh the frame bboxes from the skinned result - the bbox cache is
  // keyed by frame data and would otherwise stay at the bind pose (matters
  // for anything that culls by mesh bbox).
  Vec4 corners[2];
  corners[0].set(bmin[0], bmin[1], bmin[2], 1.0F);
  corners[1].set(bmax[0], bmax[1], bmax[2], 1.0F);
  const BBox box(corners, 2);
  if (!mesh->frames.empty() && mesh->frames[0]->bbox)
    *mesh->frames[0]->bbox = box;
  for (auto* material : mesh->materials)
    if (!material->frames.empty() && material->frames[0]->bbox)
      *material->frames[0]->bbox = box;
}

// --- blend spaces (docs/blend-spaces.md) ------------------------------------

namespace {
constexpr float kBlendLodHysteresis = 0.1F;  // weight a rival needs to take over
}

s32 SkelInstance::findBlendSpace(const char* name) const {
  if (!name || !name[0]) return -1;
  for (size_t i = 0; i < model->blendSpaces.size(); i++)
    if (model->blendSpaces[i].name == name) return (s32)i;
  return -1;
}

void SkelInstance::setBlendSpace(s32 space, float fadeSeconds) {
  if (space >= (s32)model->blendSpaces.size()) space = -1;
  if (space == bsSpace) return;
  if (fadeSeconds > 0.0F && (bsSpace >= 0 || cur.clip >= 0)) freezeFrom(fadeSeconds);
  bsSpace = space;
  bsCount = 0;
  bsPointSet = false;  // the first input is taken as is: nothing to smooth from
  bsLodSample = -1;
  bsWasSingle = false;
  prev.clip = -1;  // the frozen pose replaces any layer crossfade
  fadeT = 1.0F;
  poseDirty = globalsStale = clipStale = true;
}

void SkelInstance::setBlendInput(float dirDeg, float speed, float phase) {
  bsInDir = dirDeg;
  bsInSpeed = speed > 0.0F ? speed : 0.0F;
  bsHeldPhase = phase < 0.0F ? -1.0F : (phase > 1.0F ? 1.0F : phase);
}

void SkelInstance::freezeFrom(float seconds) {
  if (seconds <= 0.0F) {
    fromT = 1.0F;
    return;
  }
  if (localsFrom.size() != localsCur.size()) {  // a model without blend spaces: sized once
    localsFrom.resize(localsCur.size());
    animatedFrom.resize(animatedCur.size());
  }
  evalBaseLocals();  // the pose showing now (an in-flight fade included)
  memcpy(localsFrom.data(), localsCur.data(), localsCur.size() * sizeof(float));
  memcpy(animatedFrom.data(), animatedCur.data(), animatedCur.size());
  fromT = 0.0F;
  fromDuration = seconds;
  poseDirty = globalsStale = clipStale = true;
}

void SkelInstance::advanceFrom(float dt) {
  if (dt <= 0.0F || fromT >= 1.0F) return;
  fromT += fromDuration > 0.0F ? dt / fromDuration : 1.0F;
  if (fromT > 1.0F) fromT = 1.0F;
  poseDirty = globalsStale = clipStale = true;
}

void SkelInstance::advanceBlend(float dt) {
  if (bsSpace < 0) {
    advanceFrom(dt);
    return;
  }
  const SkelBlendSpace& sp = model->blendSpaces[bsSpace];
  const int n = (int)sp.samples.size();
  // the smoothing chases the input clamped to the fastest ring (see
  // blendspace::inputPoint: unclamped, a run's start would pop)
  const blendspace::Point target =
      blendspace::inputPoint(sp.samples.data(), n, bsInDir, bsInSpeed);
  if (!bsPointSet) {
    bsPoint = target;
    bsPointSet = true;
  } else {
    bsPoint = blendspace::smooth(bsPoint, target, dt, sp.smoothing);
  }
  float dir = 0.0F, speed = 0.0F;
  blendspace::fromPoint(bsPoint, &dir, &speed);
  blendspace::Weights w =
      blendspace::weightsFor(sp.samples.data(), n, sp.grid ? &sp.lines : nullptr, dir, speed);

  // The sync phase: held by the game, or played - by distance with a
  // stride, else at the shown sync samples' weighted rate.
  if (bsHeldPhase >= 0.0F) {
    bsPhase = bsHeldPhase;
  } else if (dt > 0.0F) {
    float rate = 0.0F;
    if (sp.stride > 0.0F) {
      rate = bsInSpeed / sp.stride;  // the real speed: the feet cover real ground
    } else {
      float wSum = 0.0F;
      for (int k = 0; k < w.count; k++) {
        const blendspace::Sample& sm = sp.samples[w.sample[k]];
        const float d = model->clips[sm.clip].duration;
        if (sm.ownTime || d <= 0.0F) continue;
        rate += w.weight[k] / d;
        wSum += w.weight[k];
      }
      rate = wSum > 0.0F ? rate / wSum : 0.0F;
    }
    bsPhase += dt * rate;
    bsPhase -= floorf(bsPhase);
  }
  if (dt > 0.0F)
    for (int k = 0; k < n && k < (int)bsOwnClock.size(); k++) {
      const blendspace::Sample& sm = sp.samples[k];
      const float d = model->clips[sm.clip].duration;
      if (!sm.ownTime || d <= 0.0F) continue;
      bsOwnClock[k] = fmodf(bsOwnClock[k] + dt, d);
    }

  // The blend LOD: the best-fitting sample alone; every change of what is
  // shown (into or out of the LOD, or another sample) crossfades.
  if (bsSingle && w.count > 0) {
    const int pick = blendspace::heaviest(w, bsLodSample, kBlendLodHysteresis);
    if ((bsWasSingle && pick != bsLodSample) || (!bsWasSingle && bsCount > 0))
      freezeFrom(sp.lodFade);
    bsLodSample = pick;
    w.count = 1;
    w.sample[0] = pick;
    w.weight[0] = 1.0F;
  } else {
    if (bsWasSingle && bsCount > 0) freezeFrom(sp.lodFade);
    bsLodSample = -1;
  }
  bsWasSingle = bsSingle;

  bool changed = w.count != bsCount;
  for (int k = 0; k < w.count; k++) {
    const blendspace::Sample& sm = sp.samples[w.sample[k]];
    const SkelClip& clip = model->clips[sm.clip];
    const float t = sm.ownTime && w.sample[k] < (int)bsOwnClock.size()
                        ? bsOwnClock[w.sample[k]]
                        : bsPhase * clip.duration;
    Layer& l = bs[k];
    if (l.clip != sm.clip) {
      l.clip = sm.clip;
      l.loop = true;
      l.held = true;
      l.cursors.assign(clip.channels.size(), 0);  // reserved: no allocation
      changed = true;
    }
    if (l.time != t || bsWeight[k] != w.weight[k]) changed = true;
    l.time = t;
    bsWeight[k] = w.weight[k];
  }
  bsCount = w.count;
  if (changed) poseDirty = globalsStale = clipStale = true;
  advanceFrom(dt);
}

int SkelInstance::shownSamples(s32 clips[3], float weights[3]) const {
  if (bsSpace < 0) return 0;
  const float share = fromT < 1.0F ? fromT : 1.0F;  // the rest: the pose faded from
  for (int k = 0; k < 3; k++) {
    clips[k] = k < bsCount ? bs[k].clip : -1;
    weights[k] = k < bsCount ? bsWeight[k] * share : 0.0F;
  }
  return bsCount;
}

int SkelInstance::baseKey(s32 clips[3], float times[3], float weights[3]) const {
  if (fromT < 1.0F) return -1;
  if (bsSpace >= 0 && bsCount > 0) {
    for (int k = 0; k < bsCount; k++)
      clips[k] = bs[k].clip, times[k] = bs[k].time, weights[k] = bsWeight[k];
    return bsCount;
  }
  if (fadeT < 1.0F) return -1;
  clips[0] = cur.clip, times[0] = cur.time, weights[0] = 1.0F;
  return 1;
}

// ---------------------------------------------------------------------------
// Compact streams (skinInto): a part's corners skinned by the scalar core
// straight into the compact stream the renderer draws (renderer/3d/mesh/dynamic/
// skel_skin.hpp, docs/animated-models.md "Skinning into compact streams").

namespace {

// Three 16-corner parts a package: the stream's package size.
constexpr u32 kSkinPackage = 48;

u8 levelOf(size_t levels, u8 lod) {
  return lod < levels ? lod : (u8)(levels - 1);
}

// The chain's mesh for a level: level 0 the part, level k the part's lods[k - 1].
const SkelMesh& meshAt(const SkelModel& model, size_t part, u8 level) {
  if (level == 0) return model.parts[part];
  return model.parts[part].lods[level - 1];
}

// The stream check (SkelSkin::gStreamCheck): the written stream's corner v against
// the float reference - the palette on the bind position decoded in metres (the
// same decode the bind data uses, independent of the skin core's strides).
void checkStream(const SkelModel& model, size_t part, u8 level, const float* palette,
                 u32 slots, const StaPipCompactStream& written) {
  const SkelMesh& mesh = meshAt(model, part, level);
  const u32 n = mesh.vertexCount;
  if (n == 0 || written.vertexCount() != n) return;
  std::vector<float> pos((size_t)n * 3);
  std::vector<float> nrm((size_t)n * 3);
  decodeMesh(model.box, mesh, pos.data(), nrm.data(), nullptr);
  const StaPipCompactView view(written);
  const float step = written.box().step;
  float worst = 0.0F;
  for (u32 v = 0; v < n; v++) {
    const float* p = &pos[(size_t)v * 3];
    float sum = 0.0F;
    for (int k = 0; k < 4; k++) sum += (float)mesh.weights[(size_t)v * 4 + k];
    float ref[3] = {0.0F, 0.0F, 0.0F};
    if (sum > 0.0F)
      for (int k = 0; k < 4; k++) {
        const u32 j = mesh.joints[(size_t)v * 4 + k];
        if (j >= slots) continue;
        const float wk = (float)mesh.weights[(size_t)v * 4 + k] / sum;
        const float* m = palette + 16 * j;
        for (int i = 0; i < 3; i++)
          ref[i] += wk * (m[0 * 4 + i] * p[0] + m[1 * 4 + i] * p[1] + m[2 * 4 + i] * p[2] +
                          m[3 * 4 + i]);
      }
    float got[4];
    view.position(v, got);
    for (int i = 0; i < 3; i++) {
      const float d = fabsf(got[i] - ref[i]) / step;
      if (d > worst) worst = d;
    }
  }
  SkelSkin::StreamCheck& c = SkelSkin::gStreamCheck;
  c.passes++;
  c.corners += n;
  if (worst > c.worstSteps) c.worstSteps = worst;
}

/** The full float rows of a part level (every corner), decoded from the compact mesh by the
 * arithmetic the unique level uses; the levels past the full mesh also get their packed UVs. */
std::shared_ptr<SkelBindLod> buildFloatLevel(const SkelModel& model, size_t pi, u8 level,
                                             const SkelBindLod& shared) {
  const SkelMesh& mesh = meshAt(model, pi, level);
  auto rows = std::make_shared<SkelBindLod>();
  fillBindRows(model.box, mesh, shared.skinSource, false, *rows);
  rows->skinSource = shared.skinSource;
  if (level > 0 && !model.parts[pi].texturePath.empty()) {
    rows->uvs.resize(mesh.vertexCount);
    for (u32 v = 0; v < mesh.vertexCount; v++) {
      float uv[2] = {0.0F, 0.0F};
      decodeVertex(model.box, mesh, v, nullptr, nullptr, uv);
      rows->uvs[v].set(uv[0], uv[1], 1.0F, 0.0F);
    }
  }
  return rows;
}

}  // namespace

bool SkelInstance::ensureFloatLevel(size_t pi, u8 level) {
  if (floatBinds.size() != partLods.size()) floatBinds.resize(partLods.size());
  std::vector<std::shared_ptr<const SkelBindLod>>& held = floatBinds[pi];
  if (held.size() != partLods[pi].size()) held.resize(partLods[pi].size());
  if (held[level]) return true;
  PartLod& pl = partLods[pi][level];
  if (!pl.bind) return false;
  std::shared_ptr<const SkelBindLod> rows = pl.bind->floatLevel.lock();
  if (!rows) {
#if TYRA_MEMORY_MANAGER
    Memory::MemScope memoryScope(Memory::Budget::Skinning);
#endif
    // The room the rows will take, a quarter and 4 KB over, asked in one block and given back
    // (as SkelInstance::create does): refused, nothing is made and the body keeps the level it
    // holds.
    const size_t n = pl.bind->count;
    const size_t bytes = n * (3 * sizeof(Vec4) + 4 + 1 + 4) + (level > 0 ? n * sizeof(Vec4) : 0);
    void* room = ::operator new[](bytes + bytes / 4 + 4096, std::nothrow);
    if (room == nullptr) {
#if TYRA_MEMORY_MANAGER
      Memory::Manager::instance().noteFallback("float bind rows: no room",
                                               model->parts[pi].name.c_str(), level, pl.bind);
#endif
      return false;
    }
    ::operator delete[](room);
    rows = buildFloatLevel(*model, pi, level, *pl.bind);
    pl.bind->floatLevel = rows;
  }
  held[level] = rows;
  if (level > 0 && !rows->uvs.empty()) pl.uvPtr = const_cast<Vec4*>(rows->uvs.data());
  return true;
}

bool SkelInstance::ensureStreams(size_t pi, u8 level) {
  PartLod& pl = partLods[pi][level];
  if (pl.bothStreams()) return true;
  if (!pl.bind) return false;
#if TYRA_MEMORY_MANAGER
  Memory::MemScope memoryScope(Memory::Budget::Skinning);
#endif
  if (!pl.makeStreams()) return false;
  // The layout the part's bag reads (set by the owner), else the default: a lit
  // part with the texture its mesh carries.
  if (!skinBoxReady) {
    skinBox = skinOutputBox(*model);
    skinBoxReady = true;
  }
  const SkelMesh& mesh = meshAt(*model, pi, level);
  const u32 n = mesh.vertexCount;
  if (n == 0 || pl.bind->count != n) return false;
  const bool textured = !mesh.uv.empty();
  const StaPipCompactLayout layout =
      pi < streamLayouts.size() && streamLayouts[pi].packageSize != 0
          ? streamLayouts[pi]
          : StaPipCompactLayout::forClass(textured, false, true, false, kSkinPackage, false);

  // The packer's source: four floats a corner for position, normal and ST.
  // Decoded once here and dropped after both sides are packed (transient).
  float* decoded = new (std::nothrow) float[(size_t)n * 8];
  float* source = new (std::nothrow) float[(size_t)n * 12];
  if (decoded == nullptr || source == nullptr) {
    delete[] decoded;
    delete[] source;
    return false;
  }
  float* pos3 = decoded;
  float* nrm3 = decoded + 3 * (size_t)n;
  float* uv2 = decoded + 6 * (size_t)n;
  decodeMesh(model->box, mesh, pos3, nrm3, textured ? uv2 : nullptr);
  float* spos = source;
  float* snrm = source + 4 * (size_t)n;
  float* sst = source + 8 * (size_t)n;
  for (u32 v = 0; v < n; v++) {
    for (int c = 0; c < 3; c++) {
      spos[v * 4 + c] = pos3[v * 3 + c];
      snrm[v * 4 + c] = nrm3[v * 3 + c];
    }
    spos[v * 4 + 3] = 1.0F;
    snrm[v * 4 + 3] = 0.0F;
    sst[v * 4 + 0] = textured ? uv2[v * 2] : 0.0F;
    sst[v * 4 + 1] = textured ? uv2[v * 2 + 1] : 0.0F;
    sst[v * 4 + 2] = 1.0F;
    sst[v * 4 + 3] = 0.0F;
  }
  const StaPipCompactSource packed{spos, sst, nullptr, snrm, n};
  for (int k = 0; k < 2; k++)
    if (!pl.streams[k].allocated())
      Compact::packArrays(layout, packed, skinBox, pl.streams[k]);
  delete[] decoded;
  delete[] source;
  return pl.bothStreams();
}

namespace {

// The ST payload of a part level's static blocks: the plain stream's ST run, package by package
// (rebased and scaled per package), as [corners][3] int16. False when memory is refused.
bool buildStq(const Compact::Box& box, const SkelMesh& mesh, u32 packageSize, u8 stFixed,
              std::vector<s16>& stq) {
  const u32 n = mesh.vertexCount;
  // (`stq` is a std::vector: asked for first, so a full budget refuses it)
  if (!SkelSkin::room(SkelSkin::vectorBytes((size_t)n * 3, sizeof(s16)))) return false;
  float* uv = new (std::nothrow) float[(size_t)n * 2];
  float* sts = new (std::nothrow) float[(size_t)n * 4];
  u8* run = new (std::nothrow) u8[(size_t)Compact::runQwords(Compact::Attr::St, packageSize) * 16];
  if (uv == nullptr || sts == nullptr || run == nullptr) {
    delete[] uv;
    delete[] sts;
    delete[] run;
    return false;
  }
  decodeMesh(box, mesh, nullptr, nullptr, uv);
  for (u32 v = 0; v < n; v++) {
    sts[v * 4 + 0] = uv[v * 2];
    sts[v * 4 + 1] = uv[v * 2 + 1];
    sts[v * 4 + 2] = 1.0F;
    sts[v * 4 + 3] = 0.0F;
  }
  const StaPipCompactSource source{nullptr, sts, nullptr, nullptr, n};
  stq.assign((size_t)n * 3, 0);
  for (u32 first = 0; first < n; first += packageSize) {
    const u32 count = n - first < packageSize ? n - first : packageSize;
    Compact::packRun(Compact::Attr::St, source, nullptr, first, count, box, 4096.0F, run, stFixed);
    memcpy(&stq[(size_t)first * 3], run, (size_t)count * 6);
  }
  delete[] uv;
  delete[] sts;
  delete[] run;
  return true;
}

bool sameIndexedLayout(const StaPipCompactLayout& a, const StaPipCompactLayout& b) {
  StaPipCompactLayout x = a;
  x.packageSize = b.packageSize;
  return x == b;
}

/** The shared half of an indexed part level (SkelBindLod::idx): the packaging and the static
 * blocks, built when no body has built them. False with the level unbuilt when memory is refused
 * (a later call may succeed) or the level cannot be packaged (idxFailed: never again). The
 * Skinning scope must be open. */
bool buildIndexedShared(const SkelModel& model, size_t pi, u8 level, const SkelBindLod* bl,
                        const StaPipCompactLayout& want) {
  if (bl->idx) return true;
  const SkelMesh& mesh = meshAt(model, pi, level);
  const u32 n = mesh.vertexCount;
  // The level's packaging: 75-corner packages (the program's ceiling), else 48. (make_shared is a
  // throwing request: the object's room is asked for first.)
  if (!SkelSkin::room(sizeof(SkelSkin::Indexed::Shared) + 4 * SkelSkin::kBlockOverhead))
    return false;
  auto shared = std::make_shared<SkelSkin::Indexed::Shared>();
  bool memory = false;
  const u32 sizes[2] = {SkelSkin::Indexed::kStreamPackage, SkelSkin::Indexed::kFallbackPackage};
  for (u32 k = 0; k < 2 && !shared->built() && !memory; k++) {
    StaPipCompactLayout layout = want;
    layout.packageSize = (u16)sizes[k];
    std::vector<s16> stq;
    if (!buildStq(model.box, mesh, sizes[k], layout.stFixed, stq)) {
      memory = true;
      break;
    }
    if (!shared->build(bl->skinSource.data(), n, sizes[k], stq.data(), layout)) {
      memory = shared->refused();
      if (!memory && k == 0) {
        char msg[128];
        snprintf(msg, sizeof msg,
                 "[SKIN] indexed: part %u level %u does not fit %u-corner packages, trying %u",
                 (unsigned)pi, (unsigned)level, (unsigned)sizes[0], (unsigned)sizes[1]);
        TYRA_LOG(msg);
      }
    }
  }
  if (!shared->built()) {
    if (!memory) bl->idxFailed = true;  // structural: do not try again
    return false;
  }
  bl->idx = shared;
  char msg[192];
  snprintf(msg, sizeof msg,
           "[SKIN] indexed: part %u level %u packages %u of %u corners, uniques %u (%.2f per "
           "unique), static %u B, pose %u B, gather+ranges %u B",
           (unsigned)pi, (unsigned)level, (unsigned)shared->level().packages,
           (unsigned)shared->level().packageSize, (unsigned)shared->level().uniques,
           shared->level().duplication(), (unsigned)shared->staticBytes(),
           (unsigned)shared->poseBytes(),
           (unsigned)(shared->gatherBytes() + shared->rangeBytes()));
  TYRA_LOG(msg);
  return true;
}

}  // namespace

bool SkelInstance::ensureIndexed(size_t pi, u8 level) {
  if (!SkelSkin::gUseIndexed) return false;
  PartLod& pl = partLods[pi][level];
  if (pl.indexed) return true;
  const SkelBindLod* bl = pl.bind;
  if (bl == nullptr || bl->idxFailed || !bl->uniq) return false;
  if (pl.anyStream()) return false;  // plain streams in use
  // The layout the part's bag reads (set by the owner): the indexed program's class, a textured
  // and lit triangle list at the stream package size.
  if (pi >= streamLayouts.size() || streamLayouts[pi].packageSize == 0) return false;
  const StaPipCompactLayout& want = streamLayouts[pi];
  if (want.attrCount != 3 || want.attrs[0] != Compact::Attr::Position ||
      want.attrs[1] != Compact::Attr::St || want.attrs[2] != Compact::Attr::Normal ||
      want.stripped || want.billboard || want.packageSize != SkelSkin::Indexed::kStreamPackage)
    return false;
  const SkelMesh& mesh = meshAt(*model, pi, level);
  const u32 n = mesh.vertexCount;
  if (n == 0 || bl->count != n || mesh.uv.empty()) return false;
#if TYRA_MEMORY_MANAGER
  Memory::MemScope memoryScope(Memory::Budget::Skinning);
#endif
  if (!skinBoxReady) {
    skinBox = skinOutputBox(*model);
    skinBoxReady = true;
  }
  if (!buildIndexedShared(*model, pi, level, bl, want)) return false;
  SkelSkin::Indexed::Shared& sh = *bl->idx;
  if (!sameIndexedLayout(want, sh.streamLayout())) return false;
  SkelSkin::Indexed::PoseBuf* buf = sh.acquire();
  if (buf == nullptr) return false;
  if (!pl.makeStreams() ||
      !pl.streams[0].allocateIndexed(sh.streamLayout(), n, skinBox, &sh.view(), buf->data)) {
    sh.retire(buf);
    return false;
  }
  pl.pose = buf;
  pl.indexed = true;
  pl.side = 0;
  pl.streamed = false;
  return true;
}

void SkelInstance::dropIndexed(PartLod& pl) {
  if (!pl.indexed) return;
  pl.streams[0].release();
  if (pl.pose != nullptr && pl.bind != nullptr && pl.bind->idx) pl.bind->idx->retire(pl.pose);
  pl.pose = nullptr;
  pl.indexed = false;
  pl.streamed = false;
  pl.side = 0;
}

SkelInstance::LodStream SkelInstance::lodStream(size_t part, u8 lod) {
  if (part >= partLods.size() || partLods[part].empty()) return LodStream{nullptr, 0};
  auto& chain = partLods[part];
  const u8 want = levelOf(chain.size(), lod);
  if (chain[want].streamed)
    return LodStream{&chain[want].streams[chain[want].side], want};
  // The requested level has no stream yet (or was refused): the cheapest level
  // that has one is drawn instead.
  for (size_t l = chain.size(); l-- > 0;)
    if (chain[l].streamed) return LodStream{&chain[l].streams[chain[l].side], (u8)l};
  return LodStream{nullptr, want};
}

StaPipCompactStream* SkelInstance::spareStream(size_t part, u8 lod) {
  if (part >= partLods.size() || partLods[part].empty()) return nullptr;
  const u8 level = levelOf(partLods[part].size(), lod);
  // An indexed level has the one stream; its pose buffer is taken from the pool by the pass.
  if (ensureIndexed(part, level)) return &partLods[part][level].streams[0];
  if (!ensureStreams(part, level)) return nullptr;
  PartLod& pl = partLods[part][level];
  StaPipCompactStream* spare = &pl.streams[pl.streamed ? 1 - pl.side : 0];
  if (spare->isSpanRegistered()) {
    // A frame may still read the span it was drawn from: wait until none does.
    if (!Vif1Queue::collectRetiredImmutableSpan(spare->data())) return nullptr;
    spare->setSpanRegistered(false);
  }
  return spare;
}

void SkelInstance::setStreamLayout(size_t part, const StaPipCompactLayout& layout) {
  if (!model || part >= partLods.size()) return;
  for (const PartLod& pl : partLods[part])
    if (pl.anyStream()) return;
  if (streamLayouts.size() < partLods.size())
    streamLayouts.resize(partLods.size());
  streamLayouts[part] = layout;
}

#if TYRA_SKIN_PROBE
#define SKININTO_EARLY(v) (SkelSkin::gProbe.early++, (v))
#else
#define SKININTO_EARLY(v) (v)
#endif

// Scratch of the float passes that keep a blended-float array: the blended floats of the part
// being streamed (the vector path, the unique pass of a plain stream). One pair serves every part,
// since a pass consumes it before the next part starts. It grows on demand under the Skinning
// budget; a refused grow means the scalar core. The indexed pass (the shipped path) does not use
// it: its blend is quantized as it is made and nothing is stored as floats, so a game that only
// draws indexed streams never allocates this pair.
struct SkinScratch {
  Vec4* pos = nullptr;
  Vec4* nrm = nullptr;
  u32 cap = 0;
};
SkinScratch gSkinScratch;

bool ensureSkinScratch(u32 count) {
  if (count <= gSkinScratch.cap) return true;
  delete[] gSkinScratch.pos;
  delete[] gSkinScratch.nrm;
  gSkinScratch = SkinScratch();
#if TYRA_MEMORY_MANAGER
  Memory::MemScope memoryScope(Memory::Budget::Skinning);
#endif
  Vec4* pos = new (std::nothrow) Vec4[count];
  Vec4* nrm = pos ? new (std::nothrow) Vec4[count] : nullptr;
  if (!pos || !nrm) {
    delete[] pos;
    delete[] nrm;
    return false;
  }
  gSkinScratch.pos = pos;
  gSkinScratch.nrm = nrm;
  gSkinScratch.cap = count;
  return true;
}

// The indexed pass's quantized uniques: positions s16 [cap][4] and normals s8 [cap][4], the
// arrays the pose gather reads. 16-byte aligned so the kernels store them with one doubleword /
// word each.
struct alignas(16) Lanes8 {
  s16 h[8];
};
struct alignas(16) Lanes16 {
  s8 b[16];
};
struct QuantScratch {
  Lanes8* pos = nullptr;
  Lanes16* nrm = nullptr;
  u32 cap = 0;
  s16* positions() const { return pos ? pos->h : nullptr; }
  s8* normals() const { return nrm ? nrm->b : nullptr; }
};
QuantScratch gQuant;

bool ensureQuantScratch(u32 count) {
  if (count <= gQuant.cap) return true;
  delete[] gQuant.pos;
  delete[] gQuant.nrm;
  gQuant = QuantScratch();
#if TYRA_MEMORY_MANAGER
  Memory::MemScope memoryScope(Memory::Budget::Skinning);
#endif
  Lanes8* pos = new (std::nothrow) Lanes8[(count + 1) / 2];
  Lanes16* nrm = pos ? new (std::nothrow) Lanes16[(count + 3) / 4] : nullptr;
  if (!pos || !nrm) {
    delete[] pos;
    delete[] nrm;
    return false;
  }
  gQuant.pos = pos;
  gQuant.nrm = nrm;
  gQuant.cap = count;
  return true;
}

// One part's stream pass (SkelInstance::skinInto). The vector path blends the part
// with the VU0 corner loop into scratch and quantizes it; the first vector pass of
// the process is checked against the scalar core, which switches the vector path
// off when the streams differ. The scalar core is the fallback everywhere else.
u32 skinPartStream(const SkelSkin::Folded& folded, const SkelSkin::SkelSkinPart& part,
                   const SkelBindLod& bind, const M4x4* pal, StaPipCompactStream& target) {
#if defined(_EE)
  if (SkelSkin::gUseVector && ensureSkinScratch(part.count)) {
    s32 oi[3];  // the pose box's origin in steps (the scalar core's; unused here)
    if (!SkelSkin::posePlace(folded, part, target, oi)) return SkelSkin::kSkinRefused;
    // $vf20/$vf21: the AABB registers skinBlendCorners folds into. The stream path
    // reads no box from them, so they only start from the same values skinParts uses.
    float bmin[4] alignas(16) = {FLT_MAX, FLT_MAX, FLT_MAX, 1e-12F};
    float bmax[4] alignas(16) = {-FLT_MAX, -FLT_MAX, -FLT_MAX, 0.0F};
    asm volatile(
        "lqc2         $vf20, 0x00(%[bmin])     \n\t"
        "lqc2         $vf21, 0x00(%[bmax])     \n\t"
        :
        : [bmin] "r"(bmin), [bmax] "r"(bmax)
        : "memory");
    skinBlendCorners(pal, bind, part.count, gSkinScratch.pos, gSkinScratch.nrm);
    const u32 clamped = SkelSkin::quantizeIntoStream(
        reinterpret_cast<const float*>(gSkinScratch.pos),
        reinterpret_cast<const float*>(gSkinScratch.nrm), part, target);
    if (SkelSkin::gVectorChecked) return clamped;
    SkelSkin::gVectorChecked = true;
    const SkelSkin::VectorCheck check = SkelSkin::checkVectorStream(
        folded, part, target, reinterpret_cast<float*>(gSkinScratch.pos),
        reinterpret_cast<float*>(gSkinScratch.nrm));
    SkelSkin::gUseVector = check.exact == 0 && check.worst <= 1 && check.boxMismatch == 0;
    char msg[176];
    snprintf(msg, sizeof msg,
             "SKINVEC self-check: corners=%u exact-mismatch=%u worst=%u LSB box-mismatch=%u -> %s",
             (unsigned)part.count, (unsigned)check.exact, (unsigned)check.worst,
             (unsigned)check.boxMismatch,
             SkelSkin::gUseVector ? "vector path on" : "vector path OFF (scalar core)");
    TYRA_LOG(msg);
    return check.clamped;
  }
#else
  (void)bind;
  (void)pal;
#endif
  return SkelSkin::skinIntoStream(folded, part, target);
}

// The unique-vertex pass (SkelSkin::gUseUnique): the VU0 corner loop blends the part's unique
// corners (bl.uniq) into float scratch, and writeUniqueStream boxes, quantizes and scatters
// them to every destination. Only the uniques are floats in the pass: the stream is int16.
// Refused when the scratch cannot be had (the caller skips the body this pass).
static u32 skinIntoUniqueVu0(const M4x4* pal, const SkelSkin::SkelSkinPart& part,
                             const SkelBindLod& bl, StaPipCompactStream& target) {
#if defined(_EE)
  SkelBindLod& u = *bl.uniq;
  if (!ensureSkinScratch(u.count)) return SkelSkin::kSkinRefused;
  if (u.uniqCorner.empty() && u.count > 0) {  // the corner of each unique, made on first use
    u.uniqCorner.reserve(u.count);
    for (u32 v = 0; v < (u32)bl.skinSource.size(); v++)
      if (bl.skinSource[v] == v) u.uniqCorner.push_back(v);
  }
  // $vf20 is the normal's epsilon source (skinBlendCorners' vaddw.x) and the AABB fold
  // registers: start them the way skinPartStream does, so the normals do not depend on
  // what the previous pass left in them.
  float bmin[4] alignas(16) = {FLT_MAX, FLT_MAX, FLT_MAX, 1e-12F};
  float bmax[4] alignas(16) = {-FLT_MAX, -FLT_MAX, -FLT_MAX, 0.0F};
  asm volatile(
      "lqc2         $vf20, 0x00(%[bmin])     \n\t"
      "lqc2         $vf21, 0x00(%[bmax])     \n\t"
      :
      : [bmin] "r"(bmin), [bmax] "r"(bmax)
      : "memory");
  skinBlendCorners(pal, u, u.count, gSkinScratch.pos, gSkinScratch.nrm);
  return SkelSkin::writeUniqueStream(reinterpret_cast<const float*>(gSkinScratch.pos),
                                     reinterpret_cast<const float*>(gSkinScratch.nrm),
                                     u.uniqCorner.data(), u.count, part, target);
#else
  (void)pal;
  (void)part;
  (void)bl;
  (void)target;
  return SkelSkin::kSkinRefused;
#endif
}

#if TYRA_SKIN_PROBE
static inline unsigned idxTicks() {
  unsigned t;
  asm volatile("mfc0 %0, $9" : "=r"(t));
  return t;
}
#define IDX_PROBE_DECL() unsigned idxT = idxTicks()
#define IDX_PROBE_LAP(field)                              \
  do {                                                    \
    const unsigned now_ = idxTicks();                     \
    SkelSkin::gProbe.field += now_ - idxT;                \
    idxT = now_;                                          \
  } while (0)
#else
#define IDX_PROBE_DECL() ((void)0)
#define IDX_PROBE_LAP(field) ((void)0)
#endif

#if defined(_EE)
// The fused indexed pass blends each unique twice and stores no float: the first run folds the
// extent of the blended positions (the pose box's origin needs every unique first), the second
// takes each blend straight to the pose's int16 position and int8 normal lanes. Both runs use
// the VU0 corner blend (blendMatrix), so they see the same floats the stored scratch held.

// Constants of the fused kernel's EE tail, in words: the lane mask of the tie test (x, y, z), then
// +127 and -127 in four lanes each (the normal's clamp).
alignas(16) static const s32 kFusedInts[12] = {1, 1, 1, 0, 127, 127, 127, 127, -127, -127, -127, -127};

// Pass one: the min and max of the blended positions over the level's uniques ($vf20 / $vf21 fold
// them across the loop, as skinBlendCorners always did).
static void skinExtentUniques(const M4x4* pal, const SkelBindLod& u, float mn[3], float mx[3]) {
  float bmin[4] alignas(16) = {FLT_MAX, FLT_MAX, FLT_MAX, 1e-12F};
  float bmax[4] alignas(16) = {-FLT_MAX, -FLT_MAX, -FLT_MAX, 0.0F};
  asm volatile(
      "lqc2         $vf20, 0x00(%[bmin])     \n\t"
      "lqc2         $vf21, 0x00(%[bmax])     \n\t"
      :
      : [bmin] "r"(bmin), [bmax] "r"(bmax)
      : "memory");
  const Vec4* srcP = u.bindPositions.data();
  const Vec4* wq = u.skinWeights.data();
  const u8* joints = u.sortedJoints.data();
  const u8* infl = u.influences.data();
  for (u32 v = 0; v < u.count; v++) {
    blendMatrix(pal, wq + v, &joints[(size_t)v * 4], infl[v]);
    asm volatile(
        "lqc2         $vf10, 0x00(%[pos])      \n\t"
        "vmulax.xyzw  $ACC, $vf5, $vf10        \n\t"
        "vmadday.xyzw $ACC, $vf6, $vf10        \n\t"
        "vmaddaz.xyzw $ACC, $vf7, $vf10        \n\t"
        "vmaddw.xyzw  $vf11, $vf8, $vf10       \n\t"
        "vmini.xyz    $vf20, $vf20, $vf11      \n\t"
        "vmax.xyz     $vf21, $vf21, $vf11      \n\t"
        :
        : [pos] "r"(srcP + v)
        : "memory");
  }
  asm volatile(
      "sqc2         $vf20, 0x00(%[bmin])     \n\t"
      "sqc2         $vf21, 0x00(%[bmax])     \n\t"
      :
      : [bmin] "r"(bmin), [bmax] "r"(bmax)
      : "memory");
  for (int i = 0; i < 3; ++i) {
    mn[i] = bmin[i];
    mx[i] = bmax[i];
  }
}

// Pass two: blend, then quantize in the same loop.
//
// The reference is Unique::quantizeOne on the stored blend: x = p * invStep - origin, rounded
// half away from zero as trunc(x +- 0.5), in the EE's float arithmetic. The EE's float add and
// subtract round to nearest where the VU0's truncate, so the same x can differ by one ulp, and
// x + 0.5 can round up across an integer when x is a hair below a tie. The kernel therefore
// rounds with the exact trunc(2x) - trunc(x) and tests, per lane, whether a tie lies just above
// |2x| (within about 4 ulp: some odd integer between |2x| and |2x| (1 + 2^-20) + 2^-20, found as
// the low bits of the two truncations); a unique with such a lane, or a position past +-32766
// steps (the clamp and its count), is quantized again by quantizeOne from the VU0 registers.
// Everything else is the reference's bytes, tie for tie. Returns the clamped uniques;
// `fallbacks` counts the uniques that took quantizeOne.
static u32 skinEmitUniques(const M4x4* pal, const SkelBindLod& u, u32 first, u32 count,
                           float invStep, const s32 oi[3], s16* qp, s8* qn, u32* fallbacks) {
  // $vf22: 1 / step, 2^-20, -, 32766 (the position's clip limit); $vf23: the origin in steps;
  // $vf24: 127, 2^-20; $vf20.w: the normal's epsilon.
  float scale[4] alignas(16) = {invStep, 1.0F / 1048576.0F, 0.0F, 32766.0F};
  float origin[4] alignas(16) = {(float)oi[0], (float)oi[1], (float)oi[2], 0.0F};
  float nscale[4] alignas(16) = {127.0F, 1.0F / 1048576.0F, 0.0F, 0.0F};
  float eps[4] alignas(16) = {0.0F, 0.0F, 0.0F, 1e-12F};
  asm volatile(
      "lqc2         $vf22, 0x00(%[scale])    \n\t"
      "lqc2         $vf23, 0x00(%[origin])   \n\t"
      "lqc2         $vf24, 0x00(%[nscale])   \n\t"
      "lqc2         $vf20, 0x00(%[eps])      \n\t"
      :
      : [scale] "r"(scale), [origin] "r"(origin), [nscale] "r"(nscale), [eps] "r"(eps)
      : "memory");
  const Vec4* srcP = u.bindPositions.data();
  const Vec4* srcN = u.bindNormals.data();
  const Vec4* wq = u.skinWeights.data();
  const u8* joints = u.sortedJoints.data();
  const u8* infl = u.influences.data();
  u32 clip = 0, slow = 0;
  qp -= 4 * (size_t)first;
  qn -= 4 * (size_t)first;
  for (u32 v = first; v < first + count; v++) {
    blendMatrix(pal, wq + v, &joints[(size_t)v * 4], infl[v]);
    unsigned long long danger;
    asm volatile(
        "lqc2         $vf10, 0x00(%[pos])      \n\t"
        "lqc2         $vf12, 0x00(%[nrm])      \n\t"
        "vmulax.xyzw  $ACC, $vf5, $vf10        \n\t"
        "vmadday.xyzw $ACC, $vf6, $vf10        \n\t"
        "vmaddaz.xyzw $ACC, $vf7, $vf10        \n\t"
        "vmaddw.xyzw  $vf11, $vf8, $vf10       \n\t"
        "vmulax.xyzw  $ACC, $vf5, $vf12        \n\t"
        "vmadday.xyzw $ACC, $vf6, $vf12        \n\t"
        "vmaddaz.xyzw $ACC, $vf7, $vf12        \n\t"
        "vmaddw.xyzw  $vf12, $vf8, $vf12       \n\t"
        // 1 / sqrt(len^2 + eps), as skinBlendCorners computes it; the position hides the latency
        "vmul.xyz     $vf13, $vf12, $vf12      \n\t"
        "vaddy.x      $vf13, $vf13, $vf13      \n\t"
        "vaddz.x      $vf13, $vf13, $vf13      \n\t"
        "vaddw.x      $vf13, $vf13, $vf20      \n\t"
        "vrsqrt       $Q, $vf0w, $vf13x        \n\t"
        // position: x = p / step - origin ($vf14), flag |x| > 32766, z = 2x ($vf15), the tie
        // test's limit s = |z| (1 + 2^-20) + 2^-20 ($vf19), then the three conversions
        "vmulx.xyz    $vf14, $vf11, $vf22      \n\t"
        "vsub.xyz     $vf14, $vf14, $vf23      \n\t"
        "vclipw.xyz   $vf14, $vf22             \n\t"
        "vadd.xyz     $vf15, $vf14, $vf14      \n\t"
        "vabs.xyz     $vf18, $vf15             \n\t"
        "vmuly.xyz    $vf19, $vf18, $vf22      \n\t"
        "vaddy.xyz    $vf19, $vf19, $vf22      \n\t"
        "vadd.xyz     $vf19, $vf19, $vf18      \n\t"
        "vsub.w       $vf14, $vf0, $vf0        \n\t"
        "vsub.w       $vf15, $vf0, $vf0        \n\t"
        "vftoi0       $vf14, $vf14             \n\t"
        "vftoi0       $vf15, $vf15             \n\t"
        "vftoi0       $vf19, $vf19             \n\t"
        // normal: unit * 127 ($vf25), twice that ($vf26), the same tie test ($vf28)
        "vwaitq                                \n\t"
        "vmulq.xyz    $vf16, $vf12, $Q         \n\t"
        "vmulx.xyz    $vf25, $vf16, $vf24      \n\t"
        "vadd.xyz     $vf26, $vf25, $vf25      \n\t"
        "vabs.xyz     $vf27, $vf26             \n\t"
        "vmuly.xyz    $vf28, $vf27, $vf24      \n\t"
        "vaddy.xyz    $vf28, $vf28, $vf24      \n\t"
        "vadd.xyz     $vf28, $vf28, $vf27      \n\t"
        "vsub.w       $vf25, $vf0, $vf0        \n\t"
        "vsub.w       $vf26, $vf0, $vf0        \n\t"
        "vftoi0       $vf25, $vf25             \n\t"
        "vftoi0       $vf26, $vf26             \n\t"
        "vftoi0       $vf28, $vf28             \n\t"
        // EE, position: trunc(2x) - trunc(x), packed to four halfwords, one store
        "cfc2         $13, $vi28               \n\t"
        "lq           $8, 0x00(%[k])           \n\t"
        "qmfc2        $11, $vf15               \n\t"
        "qmfc2        $12, $vf14               \n\t"
        "qmfc2        $9, $vf19                \n\t"
        "andi         $13, $13, 0x3f           \n\t"
        "psubw        $10, $11, $12            \n\t"
        "pnor         $11, $11, $0             \n\t"
        "ppach        $10, $0, $10             \n\t"
        "pand         $11, $11, $9             \n\t"
        "sd           $10, 0x00(%[qp])         \n\t"
        "pand         $11, $11, $8             \n\t"
        "pcpyud       $12, $11, $11            \n\t"
        "or           $11, $11, $12            \n\t"
        "or           $13, $13, $11            \n\t"
        // EE, normal: the same, the integer clamp to +-127, packed to four bytes
        "lq           $14, 0x10(%[k])          \n\t"
        "lq           $15, 0x20(%[k])          \n\t"
        "qmfc2        $11, $vf26               \n\t"
        "qmfc2        $12, $vf25               \n\t"
        "qmfc2        $9, $vf28                \n\t"
        "psubw        $10, $11, $12            \n\t"
        "pnor         $11, $11, $0             \n\t"
        "pminw        $10, $10, $14            \n\t"
        "pand         $11, $11, $9             \n\t"
        "pmaxw        $10, $10, $15            \n\t"
        "pand         $11, $11, $8             \n\t"
        "ppach        $10, $0, $10             \n\t"
        "pcpyud       $12, $11, $11            \n\t"
        "ppacb        $10, $0, $10             \n\t"
        "or           $11, $11, $12            \n\t"
        "sw           $10, 0x00(%[qn])         \n\t"
        "or           %[danger], $13, $11      \n\t"
        : [danger] "=&r"(danger)
        : [pos] "r"(srcP + v), [nrm] "r"(srcN + v), [qp] "r"(qp + 4 * (size_t)v),
          [qn] "r"(qn + 4 * (size_t)v), [k] "r"(kFusedInts)
        : "$8", "$9", "$10", "$11", "$12", "$13", "$14", "$15", "memory");
    if (__builtin_expect(danger != 0, 0)) {
      // A tie close by or a clamp: the reference arithmetic on the registers' values.
      float tp[4] alignas(16);
      float tn[4] alignas(16);
      asm volatile(
          "sqc2         $vf11, 0x00(%[tp])       \n\t"
          "sqc2         $vf16, 0x00(%[tn])       \n\t"
          :
          : [tp] "r"(tp), [tn] "r"(tn)
          : "memory");
      clip += SkelSkin::Unique::quantizeOne(tp, tn, invStep, oi, qp + 4 * (size_t)v,
                                            qn + 4 * (size_t)v);
      ++slow;
    }
  }
  if (fallbacks != nullptr) *fallbacks += slow;
  return clip;
}

// The earlier route, kept as the reference and as the fallback when the self-check fails: the
// blend into float scratch, the origin by scanning it, quantizeOne per unique.
static bool quantizeUniquesScratch(const M4x4* pal, const SkelBindLod& u, StaPipCompactStream& out,
                                   float step, s16* qp, s8* qn, s32 oi[3], u32& clamped) {
  if (!ensureSkinScratch(u.count)) return false;
  IDX_PROBE_DECL();
  float bmin[4] alignas(16) = {FLT_MAX, FLT_MAX, FLT_MAX, 1e-12F};
  float bmax[4] alignas(16) = {-FLT_MAX, -FLT_MAX, -FLT_MAX, 0.0F};
  asm volatile(
      "lqc2         $vf20, 0x00(%[bmin])     \n\t"
      "lqc2         $vf21, 0x00(%[bmax])     \n\t"
      :
      : [bmin] "r"(bmin), [bmax] "r"(bmax)
      : "memory");
  skinBlendCorners(pal, u, u.count, gSkinScratch.pos, gSkinScratch.nrm);
  const float* pos4 = reinterpret_cast<const float*>(gSkinScratch.pos);
  const float* nrm4 = reinterpret_cast<const float*>(gSkinScratch.nrm);
  SkelSkin::Unique::poseOrigin(pos4, u.count, step, oi);
  Compact::Box box;
  box.step = step;
  for (int i = 0; i < 3; ++i) box.origin[i] = (float)oi[i] * step;
  if (!out.rebox(box)) return false;
  IDX_PROBE_LAP(idxPre);
  const float invStep = 1.0F / step;
  clamped = 0;
  for (u32 k = 0; k < u.count; ++k)
    clamped += SkelSkin::Unique::quantizeOne(pos4 + 4 * (size_t)k, nrm4 + 4 * (size_t)k, invStep,
                                             oi, qp + 4 * (size_t)k, qn + 4 * (size_t)k);
  IDX_PROBE_LAP(idxQuant);
  return true;
}

// The fused route: the extent run, the origin, the box, the quantizing run.
static u32 gFusedPasses = 0, gFusedFallbacks = 0;

static bool quantizeUniquesFused(const M4x4* pal, const SkelBindLod& u, StaPipCompactStream& out,
                                 float step, s16* qp, s8* qn, s32 oi[3], u32& clamped) {
  IDX_PROBE_DECL();
  oi[0] = oi[1] = oi[2] = 0;
  if (u.count > 0) {
    float mn[3], mx[3];
    skinExtentUniques(pal, u, mn, mx);
    SkelSkin::Unique::originFromExtent(mn, mx, step, oi);
  }
  Compact::Box box;
  box.step = step;
  for (int i = 0; i < 3; ++i) box.origin[i] = (float)oi[i] * step;
  if (!out.rebox(box)) return false;
  IDX_PROBE_LAP(idxPre);
  clamped = skinEmitUniques(pal, u, 0, u.count, 1.0F / step, oi, qp, qn, &gFusedFallbacks);
  IDX_PROBE_LAP(idxQuant);
  if ((++gFusedPasses & 4095u) == 0) {
    char msg[96];
    snprintf(msg, sizeof msg, "SKINFUSED stats: %u passes, %u uniques re-quantized by quantizeOne",
             (unsigned)gFusedPasses, (unsigned)gFusedFallbacks);
    TYRA_LOG(msg);
  }
  return true;
}

// The fused route against the earlier one, on the pose just made: the blend is redone in windows of
// 32 uniques (stack floats, no heap), the origin taken from their extent and every unique put
// through quantizeOne. True when the origin, every lane and the clamp count are equal.
static bool fusedMatchesScratch(const M4x4* pal, const SkelBindLod& u, float step, const s32 oi[3],
                                const s16* qp, const s8* qn, u32 clamped, u32 mis[4]) {
  constexpr u32 kWindow = 32;
  Vec4 bv[kWindow], bn[kWindow];
  float eps[4] alignas(16) = {0.0F, 0.0F, 0.0F, 1e-12F};
  asm volatile("lqc2 $vf20, 0x00(%[eps]) \n\t" : : [eps] "r"(eps) : "memory");
  mis[0] = mis[1] = mis[2] = mis[3] = 0;
  float mn[3] = {FLT_MAX, FLT_MAX, FLT_MAX}, mx[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
  for (u32 first = 0; first < u.count; first += kWindow) {
    const u32 n = u.count - first < kWindow ? u.count - first : kWindow;
    skinBlendRange(pal, u, first, n, bv, bn);
    for (u32 k = 0; k < n; ++k)
      for (int i = 0; i < 3; ++i) {
        const float p = bv[k].xyzw[i];
        if (p < mn[i]) mn[i] = p;
        if (p > mx[i]) mx[i] = p;
      }
  }
  s32 ref[3] = {0, 0, 0};
  if (u.count > 0) SkelSkin::Unique::originFromExtent(mn, mx, step, ref);
  for (int i = 0; i < 3; ++i)
    if (ref[i] != oi[i]) ++mis[3];
  const float invStep = 1.0F / step;
  u32 refClamped = 0;
  for (u32 first = 0; first < u.count; first += kWindow) {
    const u32 n = u.count - first < kWindow ? u.count - first : kWindow;
    skinBlendRange(pal, u, first, n, bv, bn);
    for (u32 k = 0; k < n; ++k) {
      s16 rp[4];
      s8 rn[4];
      refClamped += SkelSkin::Unique::quantizeOne(bv[k].xyzw, bn[k].xyzw, invStep, ref, rp, rn);
      if (memcmp(rp, qp + 4 * (size_t)(first + k), 8) != 0) ++mis[0];
      if (memcmp(rn, qn + 4 * (size_t)(first + k), 4) != 0) ++mis[1];
    }
  }
  if (refClamped != clamped) mis[2] = 1;
  return mis[0] == 0 && mis[1] == 0 && mis[2] == 0 && mis[3] == 0;
}
#endif  // _EE

#if defined(_EE)
// The part boxes of the indexed pass: for every stream part (range) the min and max of the
// quantized positions of the uniques that fall in it, per axis. `lo` / `hi` are [ranges][3] s32.
// The scalar loop is the reference; the MMI form keeps a range's three axes (and a zero pad lane)
// as four halfwords in one 64-bit value and folds a unique into it with PMINH / PMAXH, one
// load, two lane ops and two stores a (unique, range) pair instead of six compares and branches.
static void partBoundsScalar(const u32* rngOff, const u16* rng, u32 count, const s16* qp,
                             u32 ranges, s32* lo, s32* hi) {
  for (u32 r = 0; r < 3 * ranges; ++r) {
    lo[r] = 32767;
    hi[r] = -32768;
  }
  for (u32 k = 0; k < count; ++k) {
    const s16* q = qp + 4 * (size_t)k;
    for (u32 j = rngOff[k]; j < rngOff[k + 1]; ++j) {
      s32* const l = lo + 3 * (size_t)rng[j];
      s32* const h = hi + 3 * (size_t)rng[j];
      for (int i = 0; i < 3; ++i) {
        if (q[i] < l[i]) l[i] = q[i];
        if (q[i] > h[i]) h[i] = q[i];
      }
    }
  }
}

typedef u64 __attribute__((may_alias, aligned(8))) Lane64;

static void partBoundsMmi(const u32* rngOff, const u16* rng, u32 count, const s16* qp, u32 ranges,
                          s32* lo, s32* hi) {
  static std::vector<u64> lanesLo, lanesHi;  // [ranges] x, y, z, pad as halfwords
  if (lanesLo.size() < ranges) {
    lanesLo.resize(ranges);
    lanesHi.resize(ranges);
  }
  u64* const l64 = lanesLo.data();
  u64* const h64 = lanesHi.data();
  for (u32 r = 0; r < ranges; ++r) {
    l64[r] = 0x00007FFF7FFF7FFFULL;  // 32767 on x, y, z; the pad lane stays 0
    h64[r] = 0x0000800080008000ULL;  // -32768
  }
  const Lane64* const q64 = static_cast<const Lane64*>(__builtin_assume_aligned(qp, 8));
  for (u32 k = 0; k < count; ++k) {
    const u64 q = q64[k];
    for (u32 j = rngOff[k]; j < rngOff[k + 1]; ++j) {
      u64 a = l64[rng[j]], b = h64[rng[j]];
      asm("pminh %0, %0, %2 \n\t"
          "pmaxh %1, %1, %2 \n\t"
          : "+r"(a), "+r"(b)
          : "r"(q));
      l64[rng[j]] = a;
      h64[rng[j]] = b;
    }
  }
  for (u32 r = 0; r < ranges; ++r) {
    s16 a[4], b[4];
    memcpy(a, &l64[r], 8);
    memcpy(b, &h64[r], 8);
    for (int i = 0; i < 3; ++i) {
      lo[3 * (size_t)r + i] = a[i];
      hi[3 * (size_t)r + i] = b[i];
    }
  }
}

// The MMI bounds against the scalar loop on the first pass of the process; a difference switches
// them off for good (the scalar loop runs from then on) and is logged.
static bool gMmiBounds = TYRA_SKIN_MMI_BOUNDS != 0;
static bool gMmiBoundsChecked = false;

static void partBounds(const u32* rngOff, const u16* rng, u32 count, const s16* qp, u32 ranges,
                       s32* lo, s32* hi) {
  if (!gMmiBounds) {
    partBoundsScalar(rngOff, rng, count, qp, ranges, lo, hi);
    return;
  }
  partBoundsMmi(rngOff, rng, count, qp, ranges, lo, hi);
  if (gMmiBoundsChecked) return;
  gMmiBoundsChecked = true;
  std::vector<s32> rl(3 * (size_t)ranges), rh(3 * (size_t)ranges);
  partBoundsScalar(rngOff, rng, count, qp, ranges, rl.data(), rh.data());
  u32 mis = 0;
  for (u32 r = 0; r < 3 * ranges; ++r)
    if (rl[r] != lo[r] || rh[r] != hi[r]) ++mis;
  gMmiBounds = mis == 0;
  char msg[120];
  snprintf(msg, sizeof msg, "SKINBOUNDS self-check: ranges=%u uniques=%u mismatches=%u -> %s",
           (unsigned)ranges, (unsigned)count, (unsigned)mis,
           gMmiBounds ? "MMI bounds on" : "MMI bounds OFF (scalar loop)");
  TYRA_LOG(msg);
  if (!gMmiBounds) {
    memcpy(lo, rl.data(), rl.size() * sizeof(s32));
    memcpy(hi, rh.data(), rh.size() * sizeof(s32));
  }
}
#endif

// The indexed pass (TYRA_SKIN_INDEXED): the unique corners are blended with the VU0 corner loop and
// quantized once each into the level's quantized lanes, gathered into a pose buffer from the level's
// pool (a unique goes into every package that uses it) and the stream takes the buffer; the part
// boxes come from the uniques' own ranges. The pose the body drew before is retired to the pool, so
// no chain still reading it is ever written. Refused (nothing changed) when the lanes or a buffer
// cannot be had. Returns the clamped uniques.
//
// The blend is fused with the quantize (TYRA_SKIN_FUSED): no float result is stored, see
// skinEmitUniques. The first fused pass of the process is checked against the earlier route (float
// scratch, poseOrigin, quantizeOne) and the route is switched off, for good, when they differ.
//
// skinIndexedPose is the pass into `out` and the pose buffer `poseData` (a pool buffer's bytes, or
// one side of a render slot): the caller owns the buffer and decides what retires.
static u32 skinIndexedPose(const M4x4* pal, const SkelBindLod& bl, StaPipCompactStream& out,
                           u8* poseData) {
#if defined(_EE)
  SkelSkin::Indexed::Shared& sh = *bl.idx;
  const SkelBindLod& u = *bl.uniq;
  if (u.count != sh.level().uniques || !(out.box().step > 0.0F)) return SkelSkin::kSkinRefused;
  if (!ensureQuantScratch(u.count)) return SkelSkin::kSkinRefused;
#if TYRA_SKIN_PROBE
  const unsigned idxStart = idxTicks();
#endif
  s16* const qp = gQuant.positions();
  s8* const qn = gQuant.normals();
  const float step = out.box().step;
  s32 oi[3];
  u32 clamped = 0;
  bool done = false;
  if (SkelSkin::gUseFused) {
    if (!quantizeUniquesFused(pal, u, out, step, qp, qn, oi, clamped))
      return SkelSkin::kSkinRefused;
    done = true;
    if (!SkelSkin::gFusedChecked) {
      SkelSkin::gFusedChecked = true;
      u32 mis[4];
      const bool same = fusedMatchesScratch(pal, u, step, oi, qp, qn, clamped, mis);
      SkelSkin::gUseFused = same;
      char msg[176];
      snprintf(msg, sizeof msg,
               "SKINFUSED self-check: uniques=%u position-lanes=%u normal-lanes=%u clamp=%u "
               "origin=%u mismatches, %u re-quantized -> %s",
               (unsigned)u.count, (unsigned)mis[0], (unsigned)mis[1], (unsigned)mis[2],
               (unsigned)mis[3], (unsigned)gFusedFallbacks,
               same ? "fused quantize on" : "fused quantize OFF (float scratch)");
      TYRA_LOG(msg);
      done = same;
    }
  }
  if (!done && !quantizeUniquesScratch(pal, u, out, step, qp, qn, oi, clamped))
    return SkelSkin::kSkinRefused;
  IDX_PROBE_DECL();
  const u32 ranges = sh.partCount();
  static std::vector<s32> lo, hi;  // [ranges][3]
  if (lo.size() < 3 * (size_t)ranges) {
    lo.resize(3 * (size_t)ranges);
    hi.resize(3 * (size_t)ranges);
  }
  partBounds(sh.rangeOffsets(), sh.ranges(), u.count, qp, ranges, lo.data(), hi.data());
  IDX_PROBE_LAP(idxBounds);
  SkelSkin::Indexed::writePose(sh.level(), sh.pose(), qp, qn, poseData);
  IDX_PROBE_LAP(idxGather);
  u32 row[4];
  Compact::rowFor(out.box(), row);
  for (u32 r = 0; r < ranges; ++r) {
    s32 glo[3], ghi[3];
    for (int i = 0; i < 3; ++i) {
      glo[i] = lo[3 * (size_t)r + i] + (s32)row[i];
      ghi[i] = hi[3 * (size_t)r + i] + (s32)row[i];
    }
    out.setPartBounds(r, glo, ghi);
  }
  out.setPose(poseData);
  out.finishBounded();
  IDX_PROBE_LAP(idxFinish);
#if TYRA_SKIN_PROBE
  SkelSkin::gProbe.idxParts++;
  SkelSkin::gProbe.idxUniques += u.count;
  SkelSkin::gProbe.idxTotal += idxTicks() - idxStart;
#endif
  return clamped;
#else
  (void)pal;
  (void)bl;
  (void)out;
  (void)poseData;
  return SkelSkin::kSkinRefused;
#endif
}

// The indexed pass of a body that owns its stream (no slot): a pool buffer is taken, written, swapped
// in, and the pose it replaces is retired to the pool.
static u32 skinIntoIndexedVu0(const M4x4* pal, const SkelBindLod& bl, SkelInstance::PartLod& pl) {
#if defined(_EE)
  SkelSkin::Indexed::Shared& sh = *bl.idx;
  SkelSkin::Indexed::PoseBuf* nb = sh.acquire();
  if (nb == nullptr) return SkelSkin::kSkinRefused;
  const u32 r = skinIndexedPose(pal, bl, pl.streams[0], nb->data);
  if (r == SkelSkin::kSkinRefused) {
    sh.retire(nb);
    return r;
  }
  sh.retire(pl.pose);
  pl.pose = nb;
  return r;
#else
  (void)pal;
  (void)bl;
  (void)pl;
  return SkelSkin::kSkinRefused;
#endif
}

// Passes a level may go unused before its stream pair is freed. A body that flickers
// between two levels keeps both; a body that has left a level frees it.
static constexpr u8 kStreamIdlePasses = 8;

void SkelInstance::releaseIdleStreams(u8 lod) {
  for (auto& chain : partLods) {
    const u8 keep = levelOf(chain.size(), lod);
    // A refused level (no stream written for it) keeps the levels it holds as the
    // body's fallback: nothing is freed until the level in use has a drawn stream.
    if (!chain[keep].streamed) continue;
    for (size_t l = 0; l < chain.size(); ++l) {
      PartLod& pl = chain[l];
      if (!pl.anyStream()) continue;
      if (pl.indexed && l != keep) {
        if (pl.idlePasses < kStreamIdlePasses) {
          ++pl.idlePasses;
          continue;
        }
        dropIndexed(pl);  // the pose buffer waits in the pool for its chains, then is reused
        pl.idlePasses = 0;
        continue;
      }
      if (l == keep) {
        pl.idlePasses = 0;
        continue;
      }
      if (pl.idlePasses < kStreamIdlePasses) {
        ++pl.idlePasses;
        continue;
      }
      // release() waits until no frame reads either side (the immutable span lease).
      pl.streams[0].release();
      pl.streams[1].release();
      pl.side = 0;
      pl.streamed = false;
      pl.idlePasses = 0;
    }
  }
}

void SkelInstance::releaseOwnStreams() {
  if (!ownStreamsMade) return;
  for (auto& chain : partLods)
    for (PartLod& pl : chain) {
      if (!pl.anyStream()) continue;
      if (pl.indexed) {
        dropIndexed(pl);
        continue;
      }
      // release() waits until no frame reads either side (the immutable span lease).
      pl.streams[0].release();
      pl.streams[1].release();
      pl.side = 0;
      pl.streamed = false;
      pl.idlePasses = 0;
    }
  compactValid = false;  // a later pass of its own starts from a pose it makes itself
  ownStreamsMade = false;
}

void SkelInstance::releaseFloatMesh() {
  if (!mesh || floatMeshReleased) return;
  for (const auto& chain : partLods) {
    bool drawn = false;
    for (const PartLod& pl : chain) drawn = drawn || pl.streamed;
    if (!drawn) return;  // a part with no stream still reads the float arrays
  }
  for (size_t pi = 0; pi < mesh->materials.size(); pi++) {
    MeshMaterial* material = mesh->materials[pi];
    if (material == nullptr || material->frames.empty()) continue;
    MeshMaterialFrame* f = material->frames[0];
    delete[] f->vertices;
    f->vertices = nullptr;
    if (f->normals) delete[] f->normals;
    f->normals = nullptr;
    if (f->textureCoords) delete[] f->textureCoords;
    f->textureCoords = nullptr;
  }
  for (auto& chain : partLods) {
    if (chain.empty()) continue;
    chain[0].outV = chain[0].outN = chain[0].uvPtr = nullptr;
  }
  floatMeshReleased = true;
}

// One pose's folds. Static: the EE runs one thread and the fold is consumed before the pass
// that made it returns.
static SkelSkin::Folded gFolded;

u32 SkelInstance::skinPlainPart(size_t pi, u8 level, StaPipCompactStream& target,
                                bool& foldedReady) {
  PartLod& pl = partLods[pi][level];
  const SkelMesh& mesh = meshAt(*model, pi, level);
  const SkelSkin::SkelSkinPart part{mesh.pos.data(),         mesh.nrm.data(),
                                    mesh.joints.data(),      mesh.weights.data(),
                                    pl.bind->skinSource.data(), pl.bind->count,
                                    &pl.bind->extent,
                                    nullptr,  // the corners' influences are made as a pass needs them
                                    &pl.bind->scatter,
                                    SkelSkin::gUseBatch ? &pl.bind->batch : nullptr};
  SkelSkin::Folded& folded = gFolded;
  u32 r;
  if (SkelSkin::gUseBatch)
    r = SkelSkin::skinIntoFixed(folded, reinterpret_cast<const float*>(palette.data()),
                                (u32)palette.size(), model->box, part, target);
  else if (SkelSkin::gUseUnique && pl.bind->uniq)
    r = skinIntoUniqueVu0(palette.data(), part, *pl.bind, target);
  else {
    // The vector path (TYRA_SKIN_FIXED 0) blends from per-corner float rows.
    const SkelBindLod* vectorRows = pl.bind;
    if (!SkelSkin::gUseFixed) {
      if (!ensureFloatLevel(pi, level)) return SkelSkin::kSkinRefused;
      vectorRows = floatBinds[pi][level].get();
    }
    if (!foldedReady) {
      SkelSkin::foldPalette(reinterpret_cast<const float*>(palette.data()),
                            (u32)palette.size(), model->box, skinBox, folded);
      foldedReady = true;
    }
    r = SkelSkin::gUseFixed
            ? SkelSkin::skinIntoFixed(folded, reinterpret_cast<const float*>(palette.data()),
                                      (u32)palette.size(), model->box, part, target)
            : skinPartStream(folded, part, *vectorRows, palette.data(), target);
  }
  return r;
}

u32 SkelInstance::skinInto(u8 lod, StaPipCompactStream* const* partStreams) {
#if TYRA_SKIN_PROBE
  SkelSkin::gProbe.calls++;
  SkelSkin::gProbe.paletteSize = (u32)palette.size();
  SkelSkin::probeReport();
#endif
  if (!model || partLods.empty() || partStreams == nullptr) return SKININTO_EARLY(0);
  if (palette.size() > SkelSkin::kMaxSlots) return SKININTO_EARLY(0);
  // A pass is due when the pose moved or the level changed; otherwise the drawn
  // sides already hold this pose and nothing is written (a split frame's second
  // half, a follower's owner).
  if (!streamPassDue(lod)) return 0;
  const bool dirty = overridden ? overrideDirty : poseDirty;
  if (!compactValid || dirty) {
    evalPose();
    poseDirty = false;
    overrideDirty = false;
  }
  if (!skinBoxReady) {
    skinBox = skinOutputBox(*model);
    skinBoxReady = true;
  }
  // One pose's folds (gFolded), shared by every part. The unique pass reads the palette
  // matrices directly, so the fold is made only when a part takes a fixed-point or stream path.
  bool folded_ready = false;
  u32 outside = 0;
  for (size_t pi = 0; pi < partLods.size(); pi++) {
    StaPipCompactStream* target = partStreams[pi];
    if (target == nullptr) {
#if TYRA_SKIN_PROBE
      SkelSkin::gProbe.noTarget++;
#endif
      continue;
    }
    const u8 level = levelOf(partLods[pi].size(), lod);
    PartLod& pl = partLods[pi][level];
    if (!pl.streams || (target != &pl.streams[0] && target != &pl.streams[1])) continue;
    if (pl.indexed) {
      // The one stream; the pass writes a pool buffer and swaps it in.
      if (target != &pl.streams[0] || !pl.bind || !pl.bind->idx) continue;
      const u32 r = skinIntoIndexedVu0(palette.data(), *pl.bind, pl);
      if (r == SkelSkin::kSkinRefused) continue;
      if (SkelSkin::gStreamCheck.enabled)
        checkStream(*model, pi, level, reinterpret_cast<const float*>(palette.data()),
                    (u32)palette.size(), *target);
      pl.side = 0;
      pl.streamed = true;
      if (r > 0) {
        outside += r;
#if TYRA_MEMORY_MANAGER
        char what[64];
        snprintf(what, sizeof what, "skin out of box count=%u", (unsigned)r);
        Memory::Manager::instance().noteFallback(what, model->parts[pi].name.c_str(), level,
                                                 model);
#endif
      }
      continue;
    }
    // The drawn side is never rewritten: a pass writes the spare only.
    if (pl.streamed && target == &pl.streams[pl.side]) continue;
    if (!target->allocated() || !pl.bind) {
#if TYRA_SKIN_PROBE
      SkelSkin::gProbe.noTarget++;
#endif
      continue;
    }
    const u32 r = skinPlainPart(pi, level, *target, folded_ready);
    if (r == SkelSkin::kSkinRefused) continue;
    if (SkelSkin::gStreamCheck.enabled)
      checkStream(*model, pi, level, reinterpret_cast<const float*>(palette.data()),
                  (u32)palette.size(), *target);
    const u8 side = target == &pl.streams[1] ? 1 : 0;
    if (pl.streamed && side != pl.side) {
      // The side it replaces is no longer drawn: retire its span (no new
      // reference may name it; the frames that do finish first).
      StaPipCompactStream& old = pl.streams[pl.side];
      if (old.allocated() && old.isSpanRegistered())
        Vif1Queue::retireImmutableSpan(old.data());
    }
    pl.side = side;
    pl.streamed = true;
    if (r > 0) {
      outside += r;
#if TYRA_MEMORY_MANAGER
      // Once a second per model (the key), like the other fallbacks.
      char what[64];
      snprintf(what, sizeof what, "skin out of box count=%u", (unsigned)r);
      Memory::Manager::instance().noteFallback(what, model->parts[pi].name.c_str(), level,
                                               model);
#endif
    }
  }
  releaseIdleStreams(lod);
  releaseFloatMesh();
  ownStreamsMade = true;
  compactValid = true;
  compactLod = lod;
  ++compactPasses;
#if TYRA_SKIN_PROBE
  SkelSkin::probeReport();
#endif
  return outside;
}

// --- render slots (docs/render-slots.md) --------------------------------------------------------

namespace {

// Whether a layout is the indexed program's class: a textured, lit triangle list at the indexed
// stream's package size.
bool indexedClass(const StaPipCompactLayout& want) {
  return want.attrCount == 3 && want.attrs[0] == Compact::Attr::Position &&
         want.attrs[1] == Compact::Attr::St && want.attrs[2] == Compact::Attr::Normal &&
         !want.stripped && !want.billboard &&
         want.packageSize == SkelSkin::Indexed::kStreamPackage;
}

// How one part of a body is placed in one side of a slot at a tier: the level it draws, the
// format (the indexed pose buffer, else the plain stream) and the bytes it takes.
struct SlotPartPlan {
  bool indexed = false;
  u8 level = 0;
  u32 vertices = 0;
  StaPipCompactLayout layout{};
  u32 poseBytes = 0;  // indexed: the pose buffer's share of `bytes`
  u32 bytes = 0;      // 16-aligned
  const SkelBindLod* bind = nullptr;
};

// `owner` is the layout the part's bag reads when the template set it, else the default of a
// textured, lit triangle list. The Skinning scope must be open (the indexed level's shared
// packaging is built on first need). `wait`, when given, is set - and false answered - when the
// packaging was refused for want of memory: the part could be placed later, so a body that asks
// does not size it as a plain stream (the layout, which passes none, does).
bool planSlotPart(const SkelModel& model, size_t pi, u8 tier, u32 packageSize,
                  const StaPipCompactLayout* owner, SlotPartPlan* out, bool* wait = nullptr) {
  const SkelBindCache* cache = model.bindCache.get();
  if (cache == nullptr || pi >= cache->parts.size()) return false;
  const auto& chain = cache->parts[pi];
  const u8 level = levelOf(chain.size(), tier);
  const SkelBindLod& bl = chain[level];
  const SkelMesh& mesh = meshAt(model, pi, level);
  const u32 n = mesh.vertexCount;
  if (n == 0 || bl.count != n) return false;
  const bool textured = !mesh.uv.empty();
  const StaPipCompactLayout lay = owner != nullptr && owner->packageSize != 0
                                      ? *owner
                                      : StaPipCompactLayout::forClass(textured, false, true, false,
                                                                      packageSize, false);
  SlotPartPlan plan;
  plan.level = level;
  plan.vertices = n;
  plan.bind = &bl;
  const bool wantsIndexed =
      SkelSkin::gUseIndexed && indexedClass(lay) && bl.uniq && !bl.idxFailed && textured;
  const bool packaged = wantsIndexed && buildIndexedShared(model, pi, level, &bl, lay);
  if (wantsIndexed && !packaged && !bl.idxFailed && wait != nullptr) {
    *wait = true;
    return false;
  }
  if (packaged && bl.idx && sameIndexedLayout(lay, bl.idx->streamLayout())) {
    plan.indexed = true;
    plan.layout = bl.idx->streamLayout();
    plan.poseBytes = (bl.idx->poseBytes() + 15) & ~15u;
    const u32 boxes = StaPipCompactStream::indexedPlacedBytes(plan.layout, n);
    if (boxes == 0) return false;
    plan.bytes = plan.poseBytes + boxes;
    *out = plan;
    return true;
  }
  plan.layout = lay;
  plan.bytes = StaPipCompactStream::placedBytes(lay, n);
  if (plan.bytes == 0) return false;
  *out = plan;
  return true;
}

}  // namespace

void SkelInstance::prepareModel(const SkelModel& model) {
  if (model.bindCache) return;
#if TYRA_MEMORY_MANAGER
  Memory::MemScope memoryScope(Memory::Budget::Skinning);
#endif
  // (null when the budget refuses: the sizes below then answer 0 and a body keeps its own streams)
  model.bindCache = buildBindCache(model);
}

u32 SkelInstance::slotSideBytes(const SkelModel& model, u8 tier, u32 packageSize) {
  if (model.parts.empty() || model.parts.size() > kMaxSlotParts) return 0;
  prepareModel(model);
#if TYRA_MEMORY_MANAGER
  Memory::MemScope memoryScope(Memory::Budget::Skinning);
#endif
  u32 total = 0;
  for (size_t pi = 0; pi < model.parts.size(); pi++) {
    SlotPartPlan plan;
    if (!planSlotPart(model, pi, tier, packageSize, nullptr, &plan)) return 0;
    total += plan.bytes;
  }
  return total;
}

u8 SkelInstance::slotLevel(size_t part, u8 tier) const {
  if (part >= partLods.size() || partLods[part].empty()) return 0;
  return levelOf(partLods[part].size(), tier);
}

bool SkelInstance::bindSlot(SkelSlot& slot, u32 packageSize) {
  if (model && slot.boundModel == model->serial.value) return true;
  if (!model || partLods.empty()) return false;
  // A body of more parts than the slots hold: slots are never an option for it.
  if (partLods.size() > kMaxSlotParts || partLods.size() > slot.partCapacity) {
    slotRefusedForGood = true;
    return false;
  }
  // Re-placing rewrites the slot's memory: no frame may still read it.
  if (!SkelRenderSlots::sideReady(slot, 0) || !SkelRenderSlots::sideReady(slot, 1)) return false;
#if TYRA_MEMORY_MANAGER
  Memory::MemScope memoryScope(Memory::Budget::Skinning);
#endif
  if (!skinBoxReady) {
    skinBox = skinOutputBox(*model);
    skinBoxReady = true;
  }
  for (u32 side = 0; side < 2; ++side)
    for (u32 p = 0; p < slot.partCapacity; ++p) slot.stream(side, p).release();
  slot.boundModel = 0;
  slot.partCount = 0;
  SlotPartPlan plans[kMaxSlotParts];
  u32 offset = 0;
  for (size_t pi = 0; pi < partLods.size(); pi++) {
    const StaPipCompactLayout* owner =
        pi < streamLayouts.size() && streamLayouts[pi].packageSize != 0 ? &streamLayouts[pi]
                                                                        : nullptr;
    bool wait = false;
    if (!planSlotPart(*model, pi, slot.tier, packageSize, owner, &plans[pi], &wait)) {
      if (!wait) slotRefusedForGood = true;  // (a refused packaging is tried again next pass)
      return false;
    }
    slot.partOffset[pi] = offset;
    offset += plans[pi].bytes;
  }
  if (offset > slot.sideBytes) {
#if TYRA_MEMORY_MANAGER
    Memory::Manager::instance().noteFallback("render slot: the body does not fit",
                                             model->parts[0].name.c_str(), slot.tier, model);
#endif
    // Another model than the layout was sized for (loaded after it, or a bigger one, or a bag
    // layout the shared packaging does not match): this body draws from streams of its own.
    slotRefusedForGood = true;
    return false;
  }
  for (u32 side = 0; side < 2; ++side) {
    for (size_t pi = 0; pi < partLods.size(); pi++) {
      const SlotPartPlan& plan = plans[pi];
      u8* base = slot.memory[side] + slot.partOffset[pi];
      StaPipCompactStream& stream = slot.stream(side, pi);
      bool ok;
      if (plan.indexed) {
        SkelSkin::Indexed::Shared& sh = *plan.bind->idx;
        sh.initBuffer(base);
        ok = stream.allocateIndexedIn(plan.layout, plan.vertices, skinBox, &sh.view(), base,
                                      base + plan.poseBytes, plan.bytes - plan.poseBytes);
      } else {
        ok = stream.allocateIn(plan.layout, plan.vertices, skinBox, base, plan.bytes);
        // The ST run never changes between poses: written now, from the model's coordinates.
        if (ok && plan.layout.indexOf(Compact::Attr::St) >= 0) {
          const SkelMesh& mesh = meshAt(*model, pi, plan.level);
          const u32 n = plan.vertices;
          float* uv = new (std::nothrow) float[(size_t)n * 2];
          float* sts = new (std::nothrow) float[(size_t)n * 4];
          if (uv != nullptr && sts != nullptr) {
            decodeMesh(model->box, mesh, nullptr, nullptr, uv);
            for (u32 v = 0; v < n; v++) {
              sts[v * 4 + 0] = uv[v * 2];
              sts[v * 4 + 1] = uv[v * 2 + 1];
              sts[v * 4 + 2] = 1.0F;
              sts[v * 4 + 3] = 0.0F;
            }
            const StaPipCompactSource source{nullptr, sts, nullptr, nullptr, n};
            ok = Compact::packArrays(plan.layout, source, skinBox, stream);
          } else {
            ok = false;
          }
          delete[] uv;
          delete[] sts;
        }
      }
      if (!ok) {
        for (u32 s2 = 0; s2 < 2; ++s2)
          for (u32 p = 0; p < slot.partCapacity; ++p) slot.stream(s2, p).release();
        return false;  // (memory for the ST payload, most likely: tried again next pass)
      }
    }
  }
  slot.partCount = (u32)partLods.size();
  slot.boundModel = model->serial.value;
  return true;
}

bool SkelInstance::skinIntoSlot(SkelSlot& slot, u32 frame, u32 packageSize, u32* outside) {
#if TYRA_SKIN_PROBE
  SkelSkin::gProbe.calls++;
  SkelSkin::probeReport();
#endif
  if (outside != nullptr) *outside = 0;
  if (!model || partLods.empty() || palette.size() > SkelSkin::kMaxSlots) return false;
  if (!bindSlot(slot, packageSize)) return false;
  const u8 side = slot.spareSide();
  // The side no frame reads: the pass that drew it two frames ago may still be in a chain.
  if (!SkelRenderSlots::sideReady(slot, side)) return false;
  const bool dirty = overridden ? overrideDirty : poseDirty;
  if (!posedOnce || dirty) evalPose();
  bool foldedReady = false;
  u32 clamped = 0;
  for (size_t pi = 0; pi < partLods.size(); pi++) {
    const u8 level = levelOf(partLods[pi].size(), slot.tier);
    PartLod& pl = partLods[pi][level];
    StaPipCompactStream& target = slot.stream(side, pi);
    if (!pl.bind || !target.allocated()) return false;
    const u32 r = target.isIndexed()
                      ? skinIndexedPose(palette.data(), *pl.bind, target,
                                        slot.memory[side] + slot.partOffset[pi])
                      : skinPlainPart(pi, level, target, foldedReady);
    if (r == SkelSkin::kSkinRefused) return false;
    if (SkelSkin::gStreamCheck.enabled)
      checkStream(*model, pi, level, reinterpret_cast<const float*>(palette.data()),
                  (u32)palette.size(), target);
    clamped += r;
  }
  poseDirty = false;
  overrideDirty = false;
  slot.flip();
  slot.skinnedFrame = frame;
  ++compactPasses;
  if (clamped > 0) {
#if TYRA_MEMORY_MANAGER
    char what[64];
    snprintf(what, sizeof what, "skin out of box count=%u", (unsigned)clamped);
    Memory::Manager::instance().noteFallback(what, model->parts[0].name.c_str(), slot.tier, model);
#endif
  }
  if (outside != nullptr) *outside = clamped;
  return true;
}

u32 SkelInstance::skinnedBytes() const {
  u32 bytes = 0;
  auto boxes = [](const StaPipCompactStream& st) {
    return (st.partCount() + st.coarseCount() + 1) * 6 * (u32)sizeof(float);
  };
  for (const auto& chain : partLods)
    for (const PartLod& pl : chain) {
      for (int k = 0; k < 2 && pl.streams; k++) {
        const StaPipCompactStream& st = pl.streams[k];
        if (!st.allocated() || st.isPlaced()) continue;
        bytes += st.isIndexed() ? boxes(st) : st.qwords() * 16 + boxes(st);
      }
      if (pl.pose != nullptr) bytes += pl.pose->qwords * 16;
      if (pl.ownVertices) bytes += pl.count * (u32)sizeof(Vec4);
      if (pl.ownNormals) bytes += pl.count * (u32)sizeof(Vec4);
    }
  if (mesh && !streamOnly && !floatMeshReleased)
    for (const MeshMaterial* material : mesh->materials) {
      if (material == nullptr || material->frames.empty()) continue;
      const MeshMaterialFrame* f = material->frames[0];
      if (f->vertices) bytes += f->count * (u32)sizeof(Vec4);
      if (f->normals) bytes += f->count * (u32)sizeof(Vec4);
      if (f->textureCoords) bytes += f->count * (u32)sizeof(Vec4);
    }
  return bytes;
}

}  // namespace Tyra

