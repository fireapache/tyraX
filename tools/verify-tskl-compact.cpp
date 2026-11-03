// Host harness for .tskl 3 (docs/animated-models.md, ".tskl version 3"): the
// editor's writer (glbparser::writeTskl), the engine's parse (TsklLoader::parse)
// and the compact rules the two share (tmdl_compact.hpp). It checks a v3 image
// against its float input (counts, positions within half a grid step, normals
// within one int8 step after renormalising, UVs within half a UV step, joints
// and weights exact, merged parts as before), a v2 image written here by this
// file's own minimal v2 writer parses to the SAME compact data as the v3 image,
// a truncated v3 image is rejected, and - when a .glb is given - the combatent's
// box step and per-mesh error.
// Build and run from the TyraX root, after one editor configure (it writes the
// tamtypes.h host shim to build/host-shim):
//   g++ -std=c++20 -O1 -Wall -Wextra -Isrc -Ibuild/host-shim -Ivendor/tyra/engine/inc
//       -Ivendor/stb -Ivendor/glfw/deps
//       -o build/verify-tskl-compact tools/verify-tskl-compact.cpp src/glbparser.cpp
//       src/meshlod.cpp src/json.cpp
//       vendor/tyra/engine/src/loaders/3d/tskl_loader/tskl_parse.cpp
//       vendor/tyra/engine/src/loaders/3d/tmdl_loader/tmdl_compact.cpp
//       vendor/tyra/engine/src/renderer/3d/pipeline/static/core/compact/stapip_compact.cpp
//   build/verify-tskl-compact [res/models/combatent.glb]
//       (prints "verify-tskl-compact: <n> checks, 0 failed")

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "glbparser.hpp"
#include "loaders/3d/tskl_loader/tskl_loader.hpp"
#include "renderer/3d/pipeline/static/core/compact/stapip_compact.hpp"

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

// Deterministic pseudo-random numbers in [0, 1): the same bytes on every run.
struct Lcg {
    uint32_t s;
    float next() {
        s = s * 1664525u + 1013904223u;
        return (float)(s >> 8) / 16777216.0f;
    }
};

// One textured part's worth of geometry: `tris` triangles, positions in a
// `span` box, unit normals, UVs in 0..uvRange, two-bone weights.
void fillMesh(glbparser::SkelPart& p, Lcg& rng, int tris, float span,
              float uvRange, const char* what) {
    (void)what;
    p.vertexCount = tris * 3;
    p.positions.clear();
    p.normals.clear();
    p.uvs.clear();
    p.joints.clear();
    p.weights.clear();
    for (int v = 0; v < p.vertexCount; ++v) {
        for (int c = 0; c < 3; ++c)
            p.positions.push_back((rng.next() - 0.5f) * span + (c == 1 ? 1.0f : 0.0f));
        float n[3] = {rng.next() - 0.5f, rng.next() - 0.5f, rng.next() + 0.1f};
        const float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        for (float x : n) p.normals.push_back(x / len);
        p.uvs.push_back(rng.next() * uvRange);
        p.uvs.push_back(rng.next() * uvRange);
        const u8 a = (u8)(rng.next() * 2.0f);  // joint 0 or 1
        const u8 w = (u8)(64 + (int)(rng.next() * 128.0f));
        p.joints.insert(p.joints.end(), {a, (u8)(1 - a), 0, 0});
        p.weights.insert(p.weights.end(), {w, (u8)(255 - w), 0, 0});
    }
}

// The LOD of a part: the first `count` vertices of its base (a stand-in for a
// decimated mesh - the layout is what matters here, not the geometry).
glbparser::SkelLod lodOf(const glbparser::SkelPart& p, int count) {
    glbparser::SkelLod l;
    l.vertexCount = count;
    l.positions.assign(p.positions.begin(), p.positions.begin() + count * 3);
    l.normals.assign(p.normals.begin(), p.normals.begin() + count * 3);
    if (!p.uvs.empty()) l.uvs.assign(p.uvs.begin(), p.uvs.begin() + count * 2);
    l.joints.assign(p.joints.begin(), p.joints.begin() + count * 4);
    l.weights.assign(p.weights.begin(), p.weights.begin() + count * 4);
    return l;
}

// A two-bone strip: a root and a child, a two-slot palette, one clip with a
// rotation track, a textured skin part with a LOD, a second textured part of
// the SAME material whose UVs span a different grid (it must merge), and an
// untextured trim in its own colour (it must not merge).
glbparser::Skel syntheticSkel() {
    glbparser::Skel s;
    s.nodes.resize(2);
    s.nodes[0].parent = -1;
    s.nodes[1].parent = 0;
    s.nodes[1].t[1] = 1.0f;
    for (int i = 0; i < 2; ++i) {
        for (int c = 0; c < 16; ++c) s.nodes[i].matrix[c] = 0.0f;
        s.nodes[i].matrix[0] = s.nodes[i].matrix[5] = s.nodes[i].matrix[10] =
            s.nodes[i].matrix[15] = 1.0f;
    }
    s.palette.resize(2);
    for (int i = 0; i < 2; ++i) {
        s.palette[i].node = i;
        for (int c = 0; c < 16; ++c) s.palette[i].ibm[c] = 0.0f;
        s.palette[i].ibm[0] = s.palette[i].ibm[5] = s.palette[i].ibm[10] =
            s.palette[i].ibm[15] = 1.0f;
    }
    glbparser::SkelClip clip;
    clip.name = "wave";
    clip.duration = 1.0f;
    glbparser::SkelChannel ch;
    ch.node = 1;
    ch.path = 1;
    ch.times = {0.0f, 1.0f};
    ch.values = {0, 0, 0, 1, 0, 0.5f, 0, 0.866f};
    clip.channels.push_back(ch);
    s.clips.push_back(clip);

    Lcg rng{12345u};
    glbparser::SkelPart skin;
    skin.material = "skin";
    skin.image = 0;
    skin.baseColor[0] = skin.baseColor[1] = skin.baseColor[2] = 1.0f;
    fillMesh(skin, rng, 10, 1.0f, 1.0f, "skin");
    skin.lods.push_back(lodOf(skin, 15));
    s.parts.push_back(skin);

    glbparser::SkelPart more;
    more.material = "skin";  // same texture and colour: merges into skin
    more.image = 0;
    more.baseColor[0] = more.baseColor[1] = more.baseColor[2] = 1.0f;
    fillMesh(more, rng, 6, 1.0f, 40.0f, "skin2");  // UVs on another grid
    s.parts.push_back(more);

    glbparser::SkelPart trim;
    trim.material = "trim";
    trim.image = -1;
    trim.baseColor[0] = 0.2f;
    trim.baseColor[1] = 0.4f;
    trim.baseColor[2] = 0.6f;
    trim.baseColor[3] = 1.0f;
    fillMesh(trim, rng, 4, 0.5f, 1.0f, "trim");
    trim.uvs.clear();  // untextured: no UV stream
    s.parts.push_back(trim);

    float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
    for (const glbparser::SkelPart& p : s.parts)
        for (size_t i = 0; i < p.positions.size(); ++i) {
            lo[i % 3] = std::fmin(lo[i % 3], p.positions[i]);
            hi[i % 3] = std::fmax(hi[i % 3], p.positions[i]);
        }
    for (int c = 0; c < 3; ++c) {
        s.min[c] = lo[c];
        s.max[c] = hi[c];
    }
    return s;
}

// --- This file's own v2 writer: the layout .tskl 2 had before the compact
// format, written from the same float structures, so a v2 image of the same
// data exists to be converted. ---

void app(std::string& o, const void* p, size_t n) {
    o.append(reinterpret_cast<const char*>(p), n);
}
void appU32(std::string& o, uint32_t v) { app(o, &v, 4); }
void appF32(std::string& o, float v) { app(o, &v, 4); }
void appFixed(std::string& o, const std::string& s, size_t n) {
    std::string t = s.substr(0, n);
    t.resize(n, '\0');
    o += t;
}

std::string writeV2(const glbparser::Skel& s,
                    const std::vector<std::string>& names) {
    std::string o = "TSKL";
    appU32(o, 2);
    appU32(o, (uint32_t)s.nodes.size());
    appU32(o, (uint32_t)s.palette.size());
    appU32(o, (uint32_t)s.parts.size());
    appU32(o, (uint32_t)s.clips.size());
    for (int c = 0; c < 3; ++c) appF32(o, s.min[c]);
    for (int c = 0; c < 3; ++c) appF32(o, s.max[c]);
    for (const glbparser::SkelNode& n : s.nodes) {
        appU32(o, (uint32_t)n.parent);
        appU32(o, n.hasMatrix ? 1u : 0u);
        for (float f : n.t) appF32(o, f);
        for (float f : n.r) appF32(o, f);
        for (float f : n.s) appF32(o, f);
        for (float f : n.matrix) appF32(o, f);
    }
    for (const glbparser::SkelJoint& j : s.palette) {
        appU32(o, (uint32_t)j.node);
        for (float f : j.ibm) appF32(o, f);
    }
    for (const glbparser::SkelClip& c : s.clips) {
        appFixed(o, c.name, 32);
        appF32(o, c.duration);
        appU32(o, (uint32_t)c.channels.size());
        for (const glbparser::SkelChannel& ch : c.channels) {
            appU32(o, (uint32_t)ch.node);
            o.push_back((char)ch.path);
            o.push_back((char)ch.step);
            o.push_back(0);
            o.push_back(0);
            appU32(o, (uint32_t)ch.times.size());
            for (float t : ch.times) appF32(o, t);
            if (ch.path == 1)
                for (float v : ch.values) {
                    const int16_t q = (int16_t)std::lround(v * 32767.0f);
                    app(o, &q, 2);
                }
            else
                for (float v : ch.values) appF32(o, v);
        }
    }
    for (const glbparser::SkelPart& p : s.parts) {
        appFixed(o, p.material.empty() ? "mat" : p.material, 32);
        const std::string tex =
            p.image >= 0 && p.image < (int)names.size() ? names[p.image] : "";
        appFixed(o, tex, 64);
        for (float f : p.baseColor) appF32(o, f);
        appU32(o, (uint32_t)p.vertexCount);
        if (!tex.empty()) app(o, p.uvs.data(), p.uvs.size() * 4);
        app(o, p.positions.data(), p.positions.size() * 4);
        app(o, p.normals.data(), p.normals.size() * 4);
        app(o, p.joints.data(), p.joints.size());
        app(o, p.weights.data(), p.weights.size());
        appU32(o, (uint32_t)p.lods.size());
        for (const glbparser::SkelLod& l : p.lods) {
            appU32(o, (uint32_t)l.vertexCount);
            if (!tex.empty()) app(o, l.uvs.data(), l.uvs.size() * 4);
            app(o, l.positions.data(), l.positions.size() * 4);
            app(o, l.normals.data(), l.normals.size() * 4);
            app(o, l.joints.data(), l.joints.size());
            app(o, l.weights.data(), l.weights.size());
        }
    }
    return o;
}

// The loaded model's compact data equal in every field (the v2 conversion
// against the v3 file).
bool sameCompact(const Tyra::SkelModel& a, const Tyra::SkelModel& b) {
    if (a.parts.size() != b.parts.size()) return false;
    if (std::memcmp(&a.box, &b.box, sizeof(a.box)) != 0) return false;
    for (size_t i = 0; i < a.parts.size(); ++i) {
        const Tyra::SkelPart& x = a.parts[i];
        const Tyra::SkelPart& y = b.parts[i];
        if (x.vertexCount != y.vertexCount || x.pos != y.pos || x.nrm != y.nrm ||
            x.uv != y.uv || x.uvFracBits != y.uvFracBits ||
            std::memcmp(x.uvOrigin, y.uvOrigin, sizeof(x.uvOrigin)) != 0 ||
            x.joints != y.joints || x.weights != y.weights ||
            x.lods.size() != y.lods.size())
            return false;
        for (size_t l = 0; l < x.lods.size(); ++l) {
            const Tyra::SkelLod& p = x.lods[l];
            const Tyra::SkelLod& q = y.lods[l];
            if (p.vertexCount != q.vertexCount || p.pos != q.pos ||
                p.nrm != q.nrm || p.uv != q.uv ||
                p.uvFracBits != q.uvFracBits ||
                std::memcmp(p.uvOrigin, q.uvOrigin, sizeof(p.uvOrigin)) != 0 ||
                p.joints != q.joints || p.weights != q.weights)
                return false;
        }
    }
    return true;
}

// The grid step of a compact UV set (2^-fraction).
float uvStepOf(int frac) { return 1.0f / (float)(1 << frac); }

// Checks one loaded mesh's compact data against its float input: positions
// within half a grid step, normals within the int8 grid, UVs within half a UV
// step (the merged grid), joints and weights exact.
// `uvSlack` widens the UV tolerance by the finest source grid's half step: a
// merged part whose UVs were re-gridded is rounded twice (once on its own
// grid, once on the merged one).
void checkMesh(const Tyra::SkelModel& m, const Tyra::SkelPart& got,
               const glbparser::SkelPart& in, const char* tag,
               float uvSlack = 0.0f) {
    (void)tag;
    const float half = m.box.step * 0.5f + 1e-7f;
    bool posOk = true, nrmOk = true, uvOk = true;
    float worstNrm = 0.0f;
    for (u32 v = 0; v < got.vertexCount; ++v) {
        for (int c = 0; c < 3; ++c) {
            const float dec = Tyra::Compact::dequantize(got.pos[v * 3 + c],
                                                        m.box.origin[c], m.box.step);
            if (std::fabs(dec - in.positions[v * 3 + c]) > half) posOk = false;
        }
        float n[3];
        for (int c = 0; c < 3; ++c) n[c] = (float)got.nrm[v * 4 + c] / 127.0f;
        const float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        for (float& x : n) x /= len;
        for (int c = 0; c < 3; ++c) {
            const float e = std::fabs(n[c] - in.normals[v * 3 + c]);
            if (e > worstNrm) worstNrm = e;
        }
        if (!got.uv.empty()) {
            const float step = uvStepOf(got.uvFracBits);
            for (int a = 0; a < 2; ++a) {
                const float dec = Tyra::Compact::dequantize(
                    got.uv[v * 2 + a], (float)got.uvOrigin[a], step);
                if (std::fabs(dec - in.uvs[v * 2 + a]) > step * 0.5f + uvSlack + 1e-7f)
                    uvOk = false;
            }
        }
    }
    // The decoded normal is renormalised, so its error is the int8 grid's
    // (half a step a component) plus the length correction: 2 grid halves.
    if (worstNrm > 2.0f * 0.5f / 127.0f + 1e-6f) nrmOk = false;
    std::printf("  %s: %u verts, worst normal error %.5f (one int8 step %.5f)\n",
                tag, got.vertexCount, (double)worstNrm, 1.0 / 127.0);
    check(posOk, "positions within half a grid step of the input");
    check(nrmOk, "normals within the int8 grid of the input");
    check(uvOk, "UVs within half a UV step of the input (their own grid)");
    check(got.joints == in.joints, "joints equal");
    check(got.weights == in.weights, "weights equal");
}

// Shared checks of one loaded skin model against its synthetic float input.
void checkSynthetic(const Tyra::SkelModel& m, const glbparser::Skel& in,
                    const char* label) {
    std::printf("%s: %zu parts, box step %g\n", label, m.parts.size(),
                m.box.step);
    // Parts: `skin` and `more` merge (same texture and colour), `trim` stays.
    check(m.parts.size() == 2, "merged parts: skin+more and trim");
    if (m.parts.size() != 2) return;
    const Tyra::SkelPart& skin = m.parts[0];
    const Tyra::SkelPart& trim = m.parts[1];
    check(skin.vertexCount == (u32)(in.parts[0].vertexCount + in.parts[1].vertexCount),
          "merged skin count");
    check(trim.vertexCount == (u32)in.parts[2].vertexCount, "trim count");
    check(trim.uv.empty(), "untextured trim carries no UV stream");

    // The merged skin: base corners of both parts, in order.
    glbparser::SkelPart both;
    both.positions = in.parts[0].positions;
    both.positions.insert(both.positions.end(), in.parts[1].positions.begin(),
                          in.parts[1].positions.end());
    both.normals = in.parts[0].normals;
    both.normals.insert(both.normals.end(), in.parts[1].normals.begin(),
                        in.parts[1].normals.end());
    both.uvs = in.parts[0].uvs;
    both.uvs.insert(both.uvs.end(), in.parts[1].uvs.begin(), in.parts[1].uvs.end());
    both.joints = in.parts[0].joints;
    both.joints.insert(both.joints.end(), in.parts[1].joints.begin(),
                       in.parts[1].joints.end());
    both.weights = in.parts[0].weights;
    both.weights.insert(both.weights.end(), in.parts[1].weights.begin(),
                        in.parts[1].weights.end());
    // The skin's UVs were on the 12-bit grid and are re-gridded onto 9 bits.
    checkMesh(m, skin, both, "skin", 0.5f / 4096.0f);
    checkMesh(m, trim, in.parts[2], "trim");

    // The LOD of the skin: its own 15 corners, then the stand-in base of
    // `more` (no LOD on it: its base stands in, as the loader merges it).
    check(skin.lods.size() == 1, "skin LOD tier kept");
    if (!skin.lods.empty()) {
        check(skin.lods[0].vertexCount == 15 + 18,
              "merged LOD = the LOD of skin + the base of more");
    }
}

// The combatent's box step (<= 3.06e-5 m) and its error on the real
// skin: parse the real .glb with the editor's own parser, write it, load it.
bool checkCombatent(const char* glbPath) {
    glbparser::Skel skel;
    std::string error;
    if (!glbparser::parseSkel(glbPath, skel, error)) {
        std::printf("FAIL: combatent parse: %s\n", error.c_str());
        return false;
    }
    std::vector<std::string> names;
    for (size_t i = 0; i < skel.images.size(); ++i)
        names.push_back("tex" + std::to_string(i) + ".png");
    const std::string bytes = glbparser::writeTskl(skel, names);
    Tyra::SkelModel m;
    const bool ok = Tyra::TsklLoader::parse((const u8*)bytes.data(), bytes.size(), m);
    check(ok, "combatent v3 image parses");
    if (!ok) return false;
    std::printf("combatent: %zu authored parts, %zu loaded, box step %g m\n",
                skel.parts.size(), m.parts.size(), (double)m.box.step);
    check(m.box.step <= 3.06e-5f, "combatent box step <= 3.06e-5 m");

    // The loader merges parts of one texture and colour, in first-use order
    // (the authored LOD materials are not adopted by parseSkel, so they are
    // parts of their own here). Group the authored parts the same way.
    struct Group {
        int image;
        float color[4];
        glbparser::SkelPart merged;
        int members;
    };
    std::vector<Group> groups;
    for (const glbparser::SkelPart& p : skel.parts) {
        Group* g = nullptr;
        for (Group& candidate : groups)
            if (candidate.image == p.image &&
                std::memcmp(candidate.color, p.baseColor, sizeof(candidate.color)) == 0)
                g = &candidate;
        if (!g) {
            groups.push_back(Group{p.image, {0, 0, 0, 0}, glbparser::SkelPart(), 0});
            g = &groups.back();
            std::memcpy(g->color, p.baseColor, sizeof(g->color));
            g->merged.image = p.image;
        }
        glbparser::SkelPart& d = g->merged;
        d.vertexCount += p.vertexCount;
        d.positions.insert(d.positions.end(), p.positions.begin(), p.positions.end());
        d.normals.insert(d.normals.end(), p.normals.begin(), p.normals.end());
        d.uvs.insert(d.uvs.end(), p.uvs.begin(), p.uvs.end());
        d.joints.insert(d.joints.end(), p.joints.begin(), p.joints.end());
        d.weights.insert(d.weights.end(), p.weights.begin(), p.weights.end());
        g->members++;
    }
    check(m.parts.size() == groups.size(),
          "combatent: one loaded part per texture and colour");
    if (m.parts.size() != groups.size()) return true;
    for (size_t gi = 0; gi < groups.size(); ++gi) {
        // A group of several parts may re-grid its UVs: one more rounding.
        const float slack = groups[gi].members > 1 ? 0.5f / 4096.0f : 0.0f;
        checkMesh(m, m.parts[gi], groups[gi].merged, "combatent", slack);
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    const glbparser::Skel in = syntheticSkel();
    const std::vector<std::string> names = {"tex0.png"};

    // 1. v3 written by the editor, parsed by the engine.
    const std::string v3 = glbparser::writeTskl(in, names);
    Tyra::SkelModel m3;
    const bool ok3 = Tyra::TsklLoader::parse((const u8*)v3.data(), v3.size(), m3);
    check(ok3, "v3 image parses");
    if (ok3) checkSynthetic(m3, in, "v3");

    // 2. v2 written here, parsed: the same compact data as the v3 parse.
    const std::string v2 = writeV2(in, names);
    Tyra::SkelModel m2;
    const bool ok2 = Tyra::TsklLoader::parse((const u8*)v2.data(), v2.size(), m2);
    check(ok2, "v2 image parses");
    if (ok2 && ok3) {
        check(sameCompact(m3, m2), "v2 conversion equals the v3 compact data");
        if (!sameCompact(m3, m2)) std::printf("  (v2 vs v3 differ)\n");
    }

    // 3. A truncated v3 image is rejected, cut anywhere past the header.
    for (size_t cut : {v3.size() - 1, v3.size() / 2, (size_t)40}) {
        Tyra::SkelModel t;
        check(!Tyra::TsklLoader::parse((const u8*)v3.data(), cut, t),
              "a truncated v3 image is rejected");
    }

    // 4. Optional: the real combatent.
    if (argc > 1) checkCombatent(argv[1]);

    std::printf("verify-tskl-compact: %d checks, %d failed\n", g_checks, g_fails);
    return g_fails == 0 ? 0 : 1;
}
