// Host harness for .tmdl 5 (docs/model-pipeline.md, "The compact
// `.tmdl` (version 5)"): the editor's writer (src/tmdl.cpp), the engine's parse
// (TmdlLoader::parse) and the conversion the two share (tmdl_compact.hpp). It
// checks a v5 image against its float input, a v4 image (written here, by this
// file's own minimal v4 writer) against the v5 parse, the truncated and unknown
// version rejects, and the bake warning for a triangle spanning more than 16
// repeats. Build and run from the TyraX root, after one editor configure (it
// writes the tamtypes.h host shim to build/host-shim):
//   g++ -std=c++20 -O1 -Wall -Wextra -Isrc -Ibuild/host-shim -Ivendor/tyra/engine/inc
//       -o build/verify-tmdl tools/verify-tmdl.cpp src/tmdl.cpp
//       vendor/tyra/engine/src/loaders/3d/tmdl_loader/tmdl_parse.cpp
//       vendor/tyra/engine/src/loaders/3d/tmdl_loader/tmdl_compact.cpp
//       vendor/tyra/engine/src/renderer/3d/pipeline/static/core/compact/stapip_compact.cpp
//   build/verify-tmdl        (prints "verify-tmdl: <n> checks, 0 failed")

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "loaders/3d/tmdl_loader/tmdl_loader.hpp"
#include "tmdl.hpp"

using namespace Tyra;

namespace {

int g_checks = 0;
int g_fails = 0;

void check(bool ok, const char* what) {
    ++g_checks;
    if (!ok) {
        ++g_fails;
        std::printf("FAIL: %s\n", what);
    }
}

// One flat-shaded triangle: 3 vertices of 8 floats (pos, flat normal, uv).
void addTri(std::vector<float>& v, const float* a, const float* b,
            const float* c, float ua, float va, float ub, float vb, float uc,
            float vc) {
    const float e1[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    const float e2[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
    float n[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2],
                  e1[0] * e2[1] - e1[1] * e2[0]};
    const float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    if (len > 0.0f)
        for (float& x : n) x /= len;
    const float* p[3] = {a, b, c};
    const float uv[3][2] = {{ua, va}, {ub, vb}, {uc, vc}};
    for (int k = 0; k < 3; ++k) {
        v.push_back(p[k][0]);
        v.push_back(p[k][1]);
        v.push_back(p[k][2]);
        v.push_back(n[0]);
        v.push_back(n[1]);
        v.push_back(n[2]);
        v.push_back(uv[k][0]);
        v.push_back(uv[k][1]);
    }
}

// A unit cube: 12 triangles, UVs in 0..1 (a uvFracBits of 12).
std::vector<float> cubeVerts() {
    const float c[8][3] = {{-.5f, -.5f, -.5f}, {.5f, -.5f, -.5f},
                           {.5f, .5f, -.5f},   {-.5f, .5f, -.5f},
                           {-.5f, -.5f, .5f},  {.5f, -.5f, .5f},
                           {.5f, .5f, .5f},    {-.5f, .5f, .5f}};
    const int t[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
                          {0, 1, 5}, {0, 5, 4}, {2, 3, 7}, {2, 7, 6},
                          {1, 2, 6}, {1, 6, 5}, {0, 4, 7}, {0, 7, 3}};
    std::vector<float> v;
    for (const auto& tri : t)
        addTri(v, c[tri[0]], c[tri[1]], c[tri[2]], 0, 0, 1, 0, 1, 1);
    return v;
}

// A flat 40 x 40 plane whose UVs run 0..40 (a uvFracBits of 9).
std::vector<float> planeVerts() {
    std::vector<float> v;
    for (int z = 0; z < 40; ++z) {
        for (int x = 0; x < 40; ++x) {
            const float p00[3] = {(float)x, 0.0f, (float)z};
            const float p10[3] = {(float)x + 1, 0.0f, (float)z};
            const float p01[3] = {(float)x, 0.0f, (float)z + 1};
            const float p11[3] = {(float)x + 1, 0.0f, (float)z + 1};
            addTri(v, p00, p01, p11, (float)x, (float)z, (float)x,
                   (float)z + 1, (float)x + 1, (float)z + 1);
            addTri(v, p00, p11, p10, (float)x, (float)z, (float)x + 1,
                   (float)z + 1, (float)x + 1, (float)z);
        }
    }
    return v;
}

// The cube with everything a part can carry: AO on the base and the LOD, a
// LOD tier (its first two triangles), a strip twin (stripRun 6) and a shadow
// proxy (the first twelve corners).
tmdl::Model cubeModel() {
    tmdl::Model m;
    for (int i = 0; i < 3; ++i) {
        m.min[i] = -0.5f;
        m.max[i] = 0.5f;
    }
    tmdl::Part part;
    part.name = "cube";
    part.texture = "models/cube.png";
    part.kd[0] = 0.8f;
    part.verts = cubeVerts();
    part.ao.assign(36, 200);
    tmdl::Lod lod;
    lod.verts.assign(part.verts.begin(), part.verts.begin() + 48);
    lod.ao.assign(6, 90);
    part.lods.push_back(lod);
    part.stripVerts = part.verts;
    part.stripRun = 6;
    m.parts.push_back(part);
    for (int i = 0; i < 12; ++i)
        for (int c = 0; c < 3; ++c) m.shadowVerts.push_back(part.verts[i * 8 + c]);
    return m;
}

tmdl::Model planeModel() {
    tmdl::Model m;
    m.min[0] = m.min[2] = 0.0f;
    m.max[0] = m.max[2] = 40.0f;
    tmdl::Part part;
    part.name = "ground";
    part.verts = planeVerts();
    m.parts.push_back(part);
    return m;
}

// --- The version-4 writer of this harness: the layout tmdl.cpp wrote before
// version 5, kept here so an old file exists to convert. ---

void app(std::string& o, const void* p, size_t n) {
    o.append(reinterpret_cast<const char*>(p), n);
}
void u32(std::string& o, uint32_t v) { app(o, &v, 4); }
void f32(std::string& o, float v) { app(o, &v, 4); }
void fixedStr(std::string& o, const std::string& s, size_t n) {
    std::string t = s.substr(0, n);
    t.resize(n, '\0');
    o += t;
}
void mesh4(std::string& o, const std::vector<float>& v,
           const std::vector<unsigned char>& ao) {
    const uint32_t n = (uint32_t)(v.size() / 8);
    u32(o, n);
    app(o, v.data(), v.size() * 4);
    const bool hasAo = ao.size() == n && n > 0;
    u32(o, hasAo ? n : 0u);
    if (hasAo) app(o, ao.data(), n);
}
void strip4(std::string& o, const std::vector<float>& v,
            const std::vector<unsigned char>& ao) {
    const uint32_t n = (uint32_t)(v.size() / 8);
    u32(o, n);
    if (n == 0) {
        u32(o, 0u);
        return;
    }
    app(o, v.data(), v.size() * 4);
    const bool hasAo = ao.size() == n;
    u32(o, hasAo ? n : 0u);
    if (hasAo) app(o, ao.data(), n);
}

std::string writeV4(const tmdl::Model& m) {
    std::string o = "TMDL";
    u32(o, 4);
    for (int i = 0; i < 3; ++i) f32(o, m.min[i]);
    for (int i = 0; i < 3; ++i) f32(o, m.max[i]);
    u32(o, (uint32_t)m.parts.size());
    for (const tmdl::Part& p : m.parts) {
        fixedStr(o, p.name, 32);
        fixedStr(o, p.texture, 64);
        fixedStr(o, p.reflTexture, 64);
        for (int i = 0; i < 3; ++i) f32(o, p.kd[i]);
        for (int i = 0; i < 3; ++i) f32(o, p.ke[i]);
        f32(o, p.reflStrength);
        u32(o, p.reflRounded ? 1u : 0u);
        mesh4(o, p.verts, p.ao);
        u32(o, (uint32_t)p.lods.size());
        for (const tmdl::Lod& l : p.lods) mesh4(o, l.verts, l.ao);
        u32(o, p.stripRun);
        strip4(o, p.stripVerts, p.stripAo);
        for (const tmdl::Lod& l : p.lods) strip4(o, l.stripVerts, l.stripAo);
    }
    const uint32_t corners = (uint32_t)(m.shadowVerts.size() / 9 * 3);
    u32(o, corners);
    app(o, m.shadowVerts.data(), (size_t)corners * 3 * 4);
    return o;
}

bool sameMesh(const CompactMesh& a, const CompactMesh& b) {
    return a.vertexCount == b.vertexCount && a.pos == b.pos && a.nrm == b.nrm &&
           a.uv == b.uv && a.uvFracBits == b.uvFracBits &&
           a.uvOrigin[0] == b.uvOrigin[0] && a.uvOrigin[1] == b.uvOrigin[1];
}

bool sameMaterialCompact(const LeanObjMaterial& a, const LeanObjMaterial& b) {
    if (!sameMesh(a.compact, b.compact) || !sameMesh(a.compactStrip, b.compactStrip))
        return false;
    if (a.compactLods.size() != b.compactLods.size() ||
        a.compactLodStrips.size() != b.compactLodStrips.size())
        return false;
    for (size_t i = 0; i < a.compactLods.size(); ++i)
        if (!sameMesh(a.compactLods[i], b.compactLods[i])) return false;
    for (size_t i = 0; i < a.compactLodStrips.size(); ++i)
        if (!sameMesh(a.compactLodStrips[i], b.compactLodStrips[i])) return false;
    return a.vertices == b.vertices && a.stripVertices == b.stripVertices &&
           a.stripRun == b.stripRun;
}

// Every decoded position within half a grid step of its input.
bool positionsWithinHalfStep(const std::vector<float>& out,
                             const std::vector<float>& in, float step) {
    if (out.size() != in.size()) return false;
    for (size_t v = 0; v < in.size() / 8; ++v)
        for (int c = 0; c < 3; ++c)
            if (std::fabs(out[v * 8 + c] - in[v * 8 + c]) > step * 0.5f + 1e-6f)
                return false;
    return true;
}

bool uvWithin(const std::vector<float>& out, const std::vector<float>& in,
              float tol) {
    if (out.size() != in.size()) return false;
    for (size_t v = 0; v < in.size() / 8; ++v)
        for (int c = 6; c < 8; ++c)
            if (std::fabs(out[v * 8 + c] - in[v * 8 + c]) > tol) return false;
    return true;
}

bool normalsWithin(const std::vector<float>& out, const std::vector<float>& in,
                   float tol) {
    if (out.size() != in.size()) return false;
    for (size_t v = 0; v < in.size() / 8; ++v)
        for (int c = 3; c < 6; ++c)
            if (std::fabs(out[v * 8 + c] - in[v * 8 + c]) > tol) return false;
    return true;
}

}  // namespace

int main() {
    // --- The cube: counts, AO, the strip twin, the LOD tier, the shadow proxy.
    const tmdl::Model cube = cubeModel();
    const std::string cubeV5 = tmdl::write(cube);
    LeanObjMesh cubeA;
    check(TmdlLoader::parse((const u8*)cubeV5.data(), cubeV5.size(), cubeA),
          "cube v5 parses");
    check(cubeA.materials.size() == 1, "cube: one part");
    if (cubeA.materials.size() == 1) {
        const LeanObjMaterial& pa = cubeA.materials[0];
        const float step = cubeA.box.step;
        check(pa.vertices.size() == cube.parts[0].verts.size(),
              "cube: base vertex count");
        check(pa.compact.vertexCount == 36, "cube: compact base count");
        check(pa.lods.size() == 1 && pa.lods[0].vertices.size() == 48,
              "cube: LOD tier count");
        check(pa.compactLods.size() == 1 && pa.compactLods[0].vertexCount == 6,
              "cube: compact LOD count");
        check(pa.stripRun == 6 && pa.stripVertices.size() == 36 * 8,
              "cube: strip twin");
        check(pa.compactStrip.vertexCount == 36, "cube: compact strip count");
        check(pa.compactLodStrips.size() == 1 &&
                  pa.compactLodStrips[0].vertexCount == 0,
              "cube: LOD without a strip twin");
        check(pa.vertexAo == cube.parts[0].ao, "cube: base AO survives");
        check(pa.lods.size() == 1 && pa.lods[0].vertexAo == cube.parts[0].lods[0].ao,
              "cube: LOD AO survives");
        check(positionsWithinHalfStep(pa.vertices, cube.parts[0].verts, step),
              "cube: positions within step/2");
        check(normalsWithin(pa.vertices, cube.parts[0].verts, 1.0f / 127.0f),
              "cube: normals within 1/127");
        check(pa.compact.uvFracBits == 12, "cube: uvFracBits == 12");
        check(uvWithin(pa.vertices, cube.parts[0].verts, 1.0f / 4096.0f),
              "cube: UVs within 2^-12");
        check(cubeA.shadowVertices.size() == 36 && cubeA.shadowPos.size() == 36,
              "cube: shadow proxy counts");
        check(cubeA.box.step > 0.0f, "cube: box step positive");
    }

    // --- The plane: 40 x 40 tiled, UVs 0..40.
    const tmdl::Model plane = planeModel();
    const std::string planeV5 = tmdl::write(plane);
    LeanObjMesh planeA;
    check(TmdlLoader::parse((const u8*)planeV5.data(), planeV5.size(), planeA),
          "plane v5 parses");
    if (planeA.materials.size() == 1) {
        const LeanObjMaterial& pa = planeA.materials[0];
        check(pa.compact.vertexCount == 9600, "plane: 3200 triangles");
        check(pa.compact.uvFracBits == 9, "plane: uvFracBits == 9");
        check(pa.compact.uvOrigin[0] == 0 && pa.compact.uvOrigin[1] == 0,
              "plane: UV origin 0");
        check(uvWithin(pa.vertices, plane.parts[0].verts, 1.0f / 1024.0f),
              "plane: UVs within 2^-10");
        check(positionsWithinHalfStep(pa.vertices, plane.parts[0].verts,
                                      planeA.box.step),
              "plane: positions within step/2");
    }

    // --- A v4 image of each model parses to the same compact data and floats.
    const struct {
        const tmdl::Model* model;
        const char* tag;
        const LeanObjMesh* v5;
    } pairs[] = {{&cube, "cube", &cubeA}, {&plane, "plane", &planeA}};
    for (const auto& pr : pairs) {
        const std::string v4 = writeV4(*pr.model);
        LeanObjMesh b;
        const bool ok = TmdlLoader::parse((const u8*)v4.data(), v4.size(), b);
        check(ok, pr.tag);
        if (!ok) continue;
        check(b.box.step == pr.v5->box.step && b.box.origin[0] == pr.v5->box.origin[0] &&
                  b.box.origin[1] == pr.v5->box.origin[1] &&
                  b.box.origin[2] == pr.v5->box.origin[2],
              "v4 image: box equals the v5 box");
        check(b.materials.size() == pr.v5->materials.size(),
              "v4 image: part count equal");
        if (b.materials.size() == pr.v5->materials.size())
            for (size_t i = 0; i < b.materials.size(); ++i)
                check(sameMaterialCompact(b.materials[i], pr.v5->materials[i]),
                      "v4 image: compact data equal to the v5 parse");
        check(b.shadowPos == pr.v5->shadowPos, "v4 image: shadow positions equal");
    }

    // --- Rejects: truncated and unknown versions.
    check(!TmdlLoader::parse((const u8*)cubeV5.data(), cubeV5.size() - 7, cubeA),
          "truncated v5 rejected");
    check(!TmdlLoader::parse((const u8*)cubeV5.data(), 3, cubeA),
          "3-byte file rejected");
    for (uint32_t bad : {0u, 6u, 9u}) {
        std::string copy = cubeV5;
        std::memcpy(&copy[4], &bad, 4);
        check(!TmdlLoader::parse((const u8*)copy.data(), copy.size(), cubeA),
              "unknown version rejected");
    }

    // --- The bake warning: a triangle spanning 20 repeats names mesh and part.
    tmdl::Model wide;
    tmdl::Part brick;
    brick.name = "brick";
    const float p0[3] = {0, 0, 0}, p1[3] = {1, 0, 0}, p2[3] = {0, 1, 0};
    addTri(brick.verts, p0, p1, p2, 0, 0, 20, 0, 0, 1);
    wide.parts.push_back(brick);
    const auto warnings = tmdl::uvSpanWarnings(wide, "wall-mesh");
    check(!warnings.empty(), "20-repeat triangle warns");
    if (!warnings.empty()) {
        check(warnings[0].find("wall-mesh") != std::string::npos,
              "warning names the mesh");
        check(warnings[0].find("brick") != std::string::npos,
              "warning names the part");
    }
    tmdl::Model narrow;
    tmdl::Part tame;
    tame.name = "tame";
    addTri(tame.verts, p0, p1, p2, 0, 0, 10, 0, 0, 1);
    narrow.parts.push_back(tame);
    check(tmdl::uvSpanWarnings(narrow, "wall-mesh").empty(),
          "10-repeat triangle does not warn");

    // --- Off-grid input: the compact indices round (they do not truncate),
    // and so do a tilted normal and a UV off the 12-bit grid. The input is off
    // the grid on purpose; the self-checks prove a truncation would differ.
    {
        tmdl::Model off;
        tmdl::Part part;
        part.name = "offgrid";
        const float a[3] = {0.3337f, -0.7771f, 0.1113f};
        const float b[3] = {0.3337f, -0.7771f, 1.1113f};
        const float c[3] = {1.1337f, -1.3771f, 0.1113f};
        // A normal of (0.6, 0.8, 0): x127 = (76.2, 101.6) rounds to (76, 102).
        addTri(part.verts, a, b, c, 0.3337f, 0.0f, 1.0f, 0.5f, 0.2f, 0.9f);
        off.parts.push_back(part);
        const std::string bytes = tmdl::write(off);
        LeanObjMesh got;
        check(TmdlLoader::parse((const u8*)bytes.data(), bytes.size(), got),
              "off-grid v5 parses");
        if (got.materials.size() == 1) {
            const LeanObjMaterial& mat = got.materials[0];
            const double step = got.box.step;
            bool allMatch = true;
            int truncDiffers = 0;
            for (unsigned v = 0; v < 3; ++v)
                for (int k = 0; k < 3; ++k) {
                    const double scaled = (double)part.verts[v * 8 + k] / step;
                    const double origin = (double)got.box.origin[k] / step;
                    const s32 want = (s32)std::round(scaled) - (s32)std::round(origin);
                    if (mat.compact.pos[v * 3 + k] != want) allMatch = false;
                    if (std::trunc(scaled) != std::round(scaled)) ++truncDiffers;
                }
            check(allMatch, "positions are round(v / step) less round(origin / step)");
            check(truncDiffers > 0, "self-check: the off-grid input truncates differently");
            check(mat.compact.nrm[0] == 76 && mat.compact.nrm[1] == 102 &&
                      mat.compact.nrm[2] == 0,
                  "tilted normal rounds to (76, 102, 0)");
            check(mat.compact.uvFracBits == 12, "off-grid UVs: fraction 12");
            check(mat.compact.uv[0] == 1367, "UV 0.3337 rounds to 1367 on the 12-bit grid");
        }
    }

    // --- Saturation: a UV range of 3000 repeats cannot be stored on the finest
    // 4-bit grid (2047 repeats). The bake names it, and the grid clamps at 4 bits.
    {
        tmdl::Model ribbon;
        tmdl::Part r;
        r.name = "ribbon";
        for (int q = 0; q < 300; ++q) {
            const float x0 = (float)q, x1 = (float)q + 1.0f;
            const float u0 = 10.0f * (float)q, u1 = 10.0f * (float)(q + 1);
            const float p00[3] = {x0, 0.0f, 0.0f}, p10[3] = {x1, 0.0f, 0.0f};
            const float p01[3] = {x0, 0.0f, 1.0f}, p11[3] = {x1, 0.0f, 1.0f};
            addTri(r.verts, p00, p01, p11, u0, 0.0f, u0, 1.0f, u1, 1.0f);
            addTri(r.verts, p00, p11, p10, u0, 0.0f, u1, 1.0f, u1, 0.0f);
        }
        ribbon.parts.push_back(r);
        const auto saturated = tmdl::uvSpanWarnings(ribbon, "rope-mesh");
        check(saturated.size() == 1, "a 3000-repeat range warns once");
        if (saturated.size() == 1) {
            check(saturated[0].find("rope-mesh") != std::string::npos &&
                      saturated[0].find("ribbon") != std::string::npos,
                  "saturation warning names mesh and part");
            check(saturated[0].find("saturate") != std::string::npos,
                  "saturation warning says the UVs saturate");
        }
        const std::string bytes = tmdl::write(ribbon);
        LeanObjMesh got;
        check(TmdlLoader::parse((const u8*)bytes.data(), bytes.size(), got) &&
                  got.materials.size() == 1 &&
                  got.materials[0].compact.uvFracBits == 4,
              "saturating UVs clamp at a 4-bit fraction");
    }

    // --- Every prefix of a v5 file is rejected, and its version word is 5.
    {
        size_t accepted = 0;
        for (size_t len = 0; len < cubeV5.size(); ++len) {
            LeanObjMesh t;
            if (TmdlLoader::parse((const u8*)cubeV5.data(), len, t)) ++accepted;
        }
        check(accepted == 0, "every prefix of a v5 file is rejected");
        uint32_t version = 0;
        std::memcpy(&version, &cubeV5[4], 4);
        check(version == 5, "v5 image: the version word is 5");
    }

    // --- readInfo rejects a file cut anywhere in its shadow-proxy tail (the
    // tail is seeked past, so the cut must be caught by the seek, not a read).
    {
        tmdl::Info whole;
        check(tmdl::readInfo(cubeV5, whole), "readInfo: the whole v5 file reads");
        check(!tmdl::readInfo(cubeV5.substr(0, cubeV5.size() - 7), whole),
              "readInfo: a cut inside the shadow proxy is rejected");
        check(!tmdl::readInfo(cubeV5.substr(0, cubeV5.size() - 1), whole),
              "readInfo: a cut one byte short of the end is rejected");
        check(!tmdl::readInfo(cubeV5.substr(0, cubeV5.size() - 72 - 4), whole),
              "readInfo: a cut before the shadow count is rejected");
    }

    std::printf("verify-tmdl: %d checks, %d failed\n", g_checks, g_fails);
    return g_fails == 0 ? 0 : 1;
}
