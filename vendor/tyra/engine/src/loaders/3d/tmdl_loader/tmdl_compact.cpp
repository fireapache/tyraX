/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Added by TyraX: the compact model rules (see tmdl_compact.hpp).
*/

#include "loaders/3d/tmdl_loader/tmdl_compact.hpp"

#include <cmath>

namespace Tyra {
namespace TmdlCompact {

namespace {

constexpr float kMaxInt16 = 32767.0f;
constexpr int kMinUvFracBits = 4;
constexpr int kMaxUvFracBits = 12;

}  // namespace

void Bounds::add(const float* xyz, u32 count, u32 stride) {
  for (u32 i = 0; i < count; ++i) {
    const float* p = xyz + (size_t)i * stride;
    for (int c = 0; c < 3; ++c) {
      if (!any) {
        mn[c] = mx[c] = p[c];
      } else {
        if (p[c] < mn[c]) mn[c] = p[c];
        if (p[c] > mx[c]) mx[c] = p[c];
      }
    }
    any = true;
  }
}

Compact::Box Bounds::box() const {
  static const float kZero[3] = {0.0f, 0.0f, 0.0f};
  return Compact::modelBox(any ? mn : kZero, any ? mx : kZero);
}

void uvGrid(const float* verts, u32 count, s8* fracBits, s32 origin[2]) {
  float lo[2] = {0.0f, 0.0f};
  float hi[2] = {0.0f, 0.0f};
  for (u32 i = 0; i < count; ++i) {
    for (int a = 0; a < 2; ++a) {
      const float v = verts[(size_t)i * 8 + 6 + a];
      if (i == 0 || v < lo[a]) lo[a] = v;
      if (i == 0 || v > hi[a]) hi[a] = v;
    }
  }
  float range = 0.0f;
  for (int a = 0; a < 2; ++a) {
    origin[a] = (s32)std::floor(lo[a]);
    const float r = hi[a] - (float)origin[a];
    if (r > range) range = r;
  }
  int frac = kMaxUvFracBits;
  while (frac > kMinUvFracBits && range * (float)(1 << frac) > kMaxInt16)
    --frac;
  *fracBits = (s8)frac;
}

CompactMesh fromFloats(const float* verts, u32 count, const Compact::Box& box) {
  CompactMesh m;
  m.vertexCount = count;
  uvGrid(verts, count, &m.uvFracBits, m.uvOrigin);
  m.pos.resize((size_t)count * 3);
  m.nrm.resize((size_t)count * 4);
  m.uv.resize((size_t)count * 2);
  const float uvStep = 1.0f / (float)(1 << m.uvFracBits);
  for (u32 i = 0; i < count; ++i) {
    const float* p = verts + (size_t)i * 8;
    for (int c = 0; c < 3; ++c) {
      m.pos[i * 3 + c] = Compact::quantize(p[c], box.origin[c], box.step);
      m.nrm[i * 4 + c] = Compact::quantizeNormal(p[3 + c]);
    }
    m.nrm[i * 4 + 3] = 0;
    for (int a = 0; a < 2; ++a)
      m.uv[i * 2 + a] =
          Compact::quantize(p[6 + a], (float)m.uvOrigin[a], uvStep);
  }
  return m;
}

void toFloats(const CompactMesh& m, const Compact::Box& box,
              std::vector<float>& out) {
  out.resize((size_t)m.vertexCount * 8);
  const float uvStep = 1.0f / (float)(1 << m.uvFracBits);
  for (u32 i = 0; i < m.vertexCount; ++i) {
    float* o = &out[(size_t)i * 8];
    for (int c = 0; c < 3; ++c)
      o[c] = Compact::dequantize(m.pos[i * 3 + c], box.origin[c], box.step);
    float n[3] = {(float)m.nrm[i * 4] / 127.0f,
                  (float)m.nrm[i * 4 + 1] / 127.0f,
                  (float)m.nrm[i * 4 + 2] / 127.0f};
    const float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    if (len > 0.0f)
      for (float& x : n) x /= len;
    o[3] = n[0];
    o[4] = n[1];
    o[5] = n[2];
    for (int a = 0; a < 2; ++a)
      o[6 + a] = Compact::dequantize(m.uv[i * 2 + a], (float)m.uvOrigin[a], uvStep);
  }
}

void compactModel(LeanObjMesh& m) {
  Bounds bounds;
  for (const LeanObjMaterial& mat : m.materials) {
    bounds.add(mat.vertices.data(), (u32)(mat.vertices.size() / 8), 8);
    bounds.add(mat.stripVertices.data(), (u32)(mat.stripVertices.size() / 8), 8);
    for (const LeanObjLod& lod : mat.lods) {
      bounds.add(lod.vertices.data(), (u32)(lod.vertices.size() / 8), 8);
      bounds.add(lod.stripVertices.data(), (u32)(lod.stripVertices.size() / 8),
                 8);
    }
  }
  bounds.add(m.shadowVertices.data(), (u32)(m.shadowVertices.size() / 3), 3);
  m.box = bounds.box();

  for (LeanObjMaterial& mat : m.materials) {
    mat.compact = fromFloats(mat.vertices.data(), (u32)(mat.vertices.size() / 8), m.box);
    mat.compactStrip =
        fromFloats(mat.stripVertices.data(), (u32)(mat.stripVertices.size() / 8), m.box);
    mat.compactLods.clear();
    mat.compactLodStrips.clear();
    for (const LeanObjLod& lod : mat.lods) {
      mat.compactLods.push_back(
          fromFloats(lod.vertices.data(), (u32)(lod.vertices.size() / 8), m.box));
      mat.compactLodStrips.push_back(fromFloats(
          lod.stripVertices.data(), (u32)(lod.stripVertices.size() / 8), m.box));
    }
  }

  const u32 shadowCorners = (u32)(m.shadowVertices.size() / 3);
  m.shadowPos.resize((size_t)shadowCorners * 3);
  for (u32 i = 0; i < shadowCorners; ++i)
    for (int c = 0; c < 3; ++c)
      m.shadowPos[i * 3 + c] = Compact::quantize(
          m.shadowVertices[i * 3 + c], m.box.origin[c], m.box.step);
}

}  // namespace TmdlCompact
}  // namespace Tyra
