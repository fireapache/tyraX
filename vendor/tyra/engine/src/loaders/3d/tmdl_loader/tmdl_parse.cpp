/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Added by TyraX: the .tmdl parse (versions 1-5). Host-compilable (no EE
# header), so the host verifier (tools/verify-tmdl.cpp) reads the same bytes
# the console does. The layout is the editor's src/tmdl.hpp.
*/

#include <math.h>
#include <string.h>

#include <string>

#include "loaders/3d/tmdl_loader/tmdl_compact.hpp"
#include "loaders/3d/tmdl_loader/tmdl_loader.hpp"

namespace Tyra {

namespace {

// Corruption guards - a bad count must not turn into a huge allocation.
constexpr u32 kMaxParts = 256;
constexpr u32 kMaxVertices = 1u << 21;
constexpr u32 kMaxLods = 4;
// The newest layout this parse reads (tmdl.hpp's kVersion).
constexpr u32 kMaxVersion = 5;

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

/** Version 1-4: vertexCount + interleaved floats + the optional AO table. */
bool readFloatMesh(Reader& in, std::vector<float>* verts, std::vector<u8>* ao) {
  u32 count = 0;
  if (!in.u32le(&count)) return false;
  if (count == 0 || count % 3 != 0 || count > kMaxVertices) return false;
  verts->resize((size_t)count * 8);
  if (!in.bytes(verts->data(), (size_t)count * 8 * sizeof(float))) return false;
  u32 aoCount = 0;
  if (!in.u32le(&aoCount)) return false;
  if (aoCount != 0 && aoCount != count) return false;  // partial = malformed
  ao->resize(aoCount);
  return aoCount == 0 || in.bytes(ao->data(), aoCount);
}

/**
 * Version 4's strip twin of readFloatMesh. An EMPTY mesh is legal (a part that
 * did not strip smaller than its list writes 0). Every run but the last is
 * exactly stripRun vertices and the last is a multiple of 3 - checked here,
 * because the failure mode of a bad run length is geometry spliced across a
 * package boundary rather than a crash.
 */
bool readFloatStrip(Reader& in, u32 stripRun, std::vector<float>* verts,
                    std::vector<u8>* ao) {
  u32 count = 0;
  if (!in.u32le(&count)) return false;
  verts->clear();
  ao->clear();
  if (count == 0) {
    u32 aoCount = 0;
    return in.u32le(&aoCount) && aoCount == 0;
  }
  if (count < 3 || count > kMaxVertices) return false;
  if (stripRun == 0 || stripRun % 3 != 0) return false;
  if (count % 3 != 0) return false;
  verts->resize((size_t)count * 8);
  if (!in.bytes(verts->data(), (size_t)count * 8 * sizeof(float))) return false;
  u32 aoCount = 0;
  if (!in.u32le(&aoCount)) return false;
  if (aoCount != 0 && aoCount != count) return false;
  ao->resize(aoCount);
  return aoCount == 0 || in.bytes(ao->data(), aoCount);
}

/**
 * Version 5: one compact record (the editor's appendCompact). `strip` allows
 * an empty mesh and checks the run length the way readFloatStrip does.
 */
bool readCompact(Reader& in, bool strip, u32 stripRun, CompactMesh* cm,
                 std::vector<u8>* ao) {
  u32 n = 0;
  if (!in.u32le(&n)) return false;
  if (n > kMaxVertices) return false;
  if (strip && n != 0) {
    if (n < 3 || n % 3 != 0 || stripRun == 0 || stripRun % 3 != 0) return false;
  } else if (!strip) {
    if (n == 0 || n % 3 != 0) return false;
  }
  cm->vertexCount = n;
  cm->pos.resize((size_t)n * 3);
  cm->nrm.resize((size_t)n * 4);
  cm->uv.resize((size_t)n * 2);
  if (!in.bytes(cm->pos.data(), (size_t)n * 3 * sizeof(s16))) return false;
  if (!in.bytes(cm->nrm.data(), (size_t)n * 4)) return false;
  if (!in.bytes(cm->uv.data(), (size_t)n * 2 * sizeof(s16))) return false;
  s8 frac = 0;
  if (!in.bytes(&frac, 1)) return false;
  if (frac < 4 || frac > 12) return false;
  cm->uvFracBits = frac;
  if (!in.bytes(cm->uvOrigin, sizeof(cm->uvOrigin))) return false;
  u32 aoCount = 0;
  if (!in.u32le(&aoCount)) return false;
  if (aoCount != 0 && aoCount != n) return false;
  ao->resize(aoCount);
  return aoCount == 0 || in.bytes(ao->data(), aoCount);
}

}  // namespace

bool TmdlLoader::parse(const u8* data, size_t size, LeanObjMesh& out) {
  out = LeanObjMesh();
  Reader in{data, size};
  char magic[4];
  u32 version = 0, partCount = 0;
  if (!in.bytes(magic, 4) || memcmp(magic, "TMDL", 4) != 0 ||
      !in.u32le(&version) || version < 1 || version > kMaxVersion) {
    return false;
  }
  for (int c = 0; c < 3; c++)
    if (!in.f32le(&out.min[c])) return false;
  for (int c = 0; c < 3; c++)
    if (!in.f32le(&out.max[c])) return false;
  const bool compact = version >= 5;
  if (compact) {
    // The model's box: every compact position below is in it.
    for (int c = 0; c < 3; c++)
      if (!in.f32le(&out.box.origin[c])) return false;
    if (!in.f32le(&out.box.step) || !(out.box.step > 0.0f) ||
        !(out.box.step < 1e30f))
      return false;
  }
  if (!in.u32le(&partCount) || partCount == 0 || partCount > kMaxParts)
    return false;

  out.materials.resize(partCount);
  for (u32 i = 0; i < partCount; i++) {
    LeanObjMaterial& mat = out.materials[i];
    u32 flags = 0, lodCount = 0;
    if (!in.fixedString(&mat.name, 32) ||
        !in.fixedString(&mat.textureName, 64) ||
        !in.fixedString(&mat.reflTextureName, 64) ||
        !in.bytes(mat.kd, sizeof(mat.kd))) {
      return false;
    }
    // Ke (emissive materials) arrived with version 2; a v1 file has no such
    // field and its parts stay matte, which is exactly what they were.
    if (version >= 2 && !in.bytes(mat.ke, sizeof(mat.ke))) return false;
    if (!in.f32le(&mat.reflStrength) || !in.u32le(&flags)) return false;
    mat.reflRounded = (flags & 1) != 0;

    // The base mesh. Version 5 decodes its floats from the compact record now;
    // older files keep their floats and get the compact copies at the end.
    if (compact) {
      std::vector<u8> ao;
      if (!readCompact(in, false, 0, &mat.compact, &ao)) return false;
      mat.vertexAo = std::move(ao);
      Tyra::TmdlCompact::toFloats(mat.compact, out.box, mat.vertices);
    } else if (!readFloatMesh(in, &mat.vertices, &mat.vertexAo)) {
      return false;
    }

    if (!in.u32le(&lodCount) || lodCount > kMaxLods) return false;
    // uvRect stays {0,0,1,1}: an atlas rect is already folded into the baked
    // UVs, so nothing downstream may multiply through it a second time.
    mat.lods.resize(lodCount);
    if (compact) mat.compactLods.resize(lodCount);
    for (u32 l = 0; l < lodCount; l++) {
      if (compact) {
        std::vector<u8> ao;
        if (!readCompact(in, false, 0, &mat.compactLods[l], &ao)) return false;
        mat.lods[l].vertexAo = std::move(ao);
        Tyra::TmdlCompact::toFloats(mat.compactLods[l], out.box,
                                    mat.lods[l].vertices);
      } else if (!readFloatMesh(in, &mat.lods[l].vertices,
                                &mat.lods[l].vertexAo)) {
        return false;
      }
    }

    // Triangle strips (version 4). A v3 file has none and every part renders
    // as the triangle list it always was.
    if (version >= 4) {
      if (!in.u32le(&mat.stripRun)) return false;
      if (compact) {
        std::vector<u8> ao;
        if (!readCompact(in, true, mat.stripRun, &mat.compactStrip, &ao))
          return false;
        mat.stripVertexAo = std::move(ao);
        Tyra::TmdlCompact::toFloats(mat.compactStrip, out.box,
                                    mat.stripVertices);
        mat.compactLodStrips.resize(lodCount);
      } else if (!readFloatStrip(in, mat.stripRun, &mat.stripVertices,
                                 &mat.stripVertexAo)) {
        return false;
      }
      for (u32 l = 0; l < lodCount; l++) {
        if (compact) {
          std::vector<u8> ao;
          if (!readCompact(in, true, mat.stripRun, &mat.compactLodStrips[l], &ao))
            return false;
          mat.lods[l].stripVertexAo = std::move(ao);
          Tyra::TmdlCompact::toFloats(mat.compactLodStrips[l], out.box,
                                      mat.lods[l].stripVertices);
        } else if (!readFloatStrip(in, mat.stripRun, &mat.lods[l].stripVertices,
                                   &mat.lods[l].stripVertexAo)) {
          return false;
        }
      }
      // A run length with nothing to run over is meaningless; normalise it so
      // a consumer can test either field.
      if (mat.stripVertices.empty()) mat.stripRun = 0;
    }
  }

  // Shadow proxy (version 3). A v2 file ends here and casts from its full
  // mesh or boxes, as it always did.
  if (version >= 3) {
    u32 shadowCorners = 0;
    if (!in.u32le(&shadowCorners) || shadowCorners % 3 != 0 ||
        shadowCorners > kMaxVertices) {
      return false;
    }
    const size_t floats = (size_t)shadowCorners * 3;
    if (compact) {
      out.shadowPos.resize(floats);
      if (!in.bytes(out.shadowPos.data(), floats * sizeof(s16))) return false;
      out.shadowVertices.resize(floats);
      for (size_t i = 0; i < floats; ++i)
        out.shadowVertices[i] = Compact::dequantize(
            out.shadowPos[i], out.box.origin[i % 3], out.box.step);
    } else {
      out.shadowVertices.resize(floats);
      if (floats && !in.bytes(out.shadowVertices.data(), floats * sizeof(float)))
        return false;
    }
  }

  // Older files: the compact copies and the box come from the floats, by the
  // rules the editor's writer uses (plan 52 Task 7).
  if (!compact) Tyra::TmdlCompact::compactModel(out);
  return true;
}

}  // namespace Tyra
