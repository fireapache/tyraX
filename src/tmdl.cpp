#include "tmdl.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <utility>

#include "../vendor/tyra/engine/inc/loaders/3d/tmdl_loader/tmdl_compact.hpp"

namespace tmdl {

namespace {

void appendBytes(std::string& out, const void* p, size_t n) {
    out.append(reinterpret_cast<const char*>(p), n);
}

void appendU32(std::string& out, uint32_t v) { appendBytes(out, &v, 4); }
void appendF32(std::string& out, float v) { appendBytes(out, &v, 4); }

// Fixed-size NUL-padded field, truncating (the .tskl convention).
void appendFixedString(std::string& out, const std::string& s, size_t size) {
    char buf[128] = {};
    std::snprintf(buf, std::min(size, sizeof(buf)), "%s", s.c_str());
    out.append(buf, size);
}

// A vector's bytes, skipping the empty case (data() may be null then).
template <typename T>
void appendVector(std::string& out, const std::vector<T>& v) {
    if (!v.empty()) appendBytes(out, v.data(), v.size() * sizeof(T));
}

// The version-5 compact record: the mesh quantized in the model's box, with
// its AO (written only when it covers every vertex - a partial table would
// make the loader guess). An empty mesh is a legal record of zero vertices.
void appendCompact(std::string& out, const Tyra::CompactMesh& cm,
                   const std::vector<unsigned char>& ao) {
    appendU32(out, cm.vertexCount);
    appendVector(out, cm.pos);
    appendVector(out, cm.nrm);
    appendVector(out, cm.uv);
    appendBytes(out, &cm.uvFracBits, 1);
    appendBytes(out, cm.uvOrigin, 8);
    const bool hasAo = ao.size() == cm.vertexCount && cm.vertexCount > 0;
    appendU32(out, hasAo ? cm.vertexCount : 0u);
    if (hasAo) appendBytes(out, ao.data(), cm.vertexCount);
}

// Every mesh of the model, in the order the file writes them, quantized in
// `box`. The box is the bounds of all of them, so it is computed first.
Tyra::Compact::Box boxOf(const Model& m, uint32_t shadowCorners) {
    Tyra::TmdlCompact::Bounds bounds;
    for (const Part& p : m.parts) {
        bounds.add(p.verts.data(), (uint32_t)(p.verts.size() / 8), 8);
        bounds.add(p.stripVerts.data(), (uint32_t)(p.stripVerts.size() / 8), 8);
        for (const Lod& l : p.lods) {
            bounds.add(l.verts.data(), (uint32_t)(l.verts.size() / 8), 8);
            bounds.add(l.stripVerts.data(), (uint32_t)(l.stripVerts.size() / 8), 8);
        }
    }
    bounds.add(m.shadowVerts.data(), shadowCorners, 3);
    return bounds.box();
}

}  // namespace

std::string write(const Model& m) {
    // The shadow proxy's corner count, and the float count it carries: a whole
    // number of triangles, the same count the bounds and the file both use.
    const uint32_t shadowCorners = (uint32_t)(m.shadowVerts.size() / 9 * 3);
    const Tyra::Compact::Box box = boxOf(m, shadowCorners);

    std::string out;
    out.reserve(4096);
    out += "TMDL";
    appendU32(out, kVersion);
    for (int i = 0; i < 3; ++i) appendF32(out, m.min[i]);
    for (int i = 0; i < 3; ++i) appendF32(out, m.max[i]);
    for (int i = 0; i < 3; ++i) appendF32(out, box.origin[i]);
    appendF32(out, box.step);
    appendU32(out, (uint32_t)m.parts.size());
    for (const Part& part : m.parts) {
        appendFixedString(out, part.name, 32);
        appendFixedString(out, part.texture, 64);
        appendFixedString(out, part.reflTexture, 64);
        for (int i = 0; i < 3; ++i) appendF32(out, part.kd[i]);
        for (int i = 0; i < 3; ++i) appendF32(out, part.ke[i]);
        appendF32(out, part.reflStrength);
        appendU32(out, part.reflRounded ? 1u : 0u);

        auto mesh = [&](const std::vector<float>& verts,
                        const std::vector<unsigned char>& ao) {
            const uint32_t count = (uint32_t)(verts.size() / 8);
            appendCompact(out,
                          Tyra::TmdlCompact::fromFloats(verts.data(), count, box),
                          ao);
        };
        mesh(part.verts, part.ao);
        appendU32(out, (uint32_t)part.lods.size());
        for (const Lod& lod : part.lods) mesh(lod.verts, lod.ao);
        // The strip twins follow the lists, the base first and then each tier
        // in order, so a reader of the older layout that stops here still sees
        // a complete part.
        appendU32(out, part.stripRun);
        mesh(part.stripVerts, part.stripAo);
        for (const Lod& lod : part.lods) mesh(lod.stripVerts, lod.stripAo);
    }
    appendU32(out, shadowCorners);
    for (uint32_t i = 0; i < shadowCorners; ++i)
        for (int c = 0; c < 3; ++c) {
            const int16_t q = Tyra::Compact::quantize(
                m.shadowVerts[(size_t)i * 3 + c], box.origin[c], box.step);
            appendBytes(out, &q, 2);
        }
    return out;
}

std::vector<std::string> uvSpanWarnings(const Model& m,
                                        const std::string& meshName) {
    constexpr float kMaxRepeats = 16.0f;
    // The UV grid's finest fraction is 4 bits and its int16 holds 2047 repeats
    // of range at that fraction (TmdlCompact::uvGrid): a mesh spanning more
    // saturates, and nothing below can store it.
    constexpr float kMaxStorableRepeats = 32767.0f / 16.0f;
    std::vector<std::string> out;
    for (const Part& p : m.parts) {
        {
            // The same range uvGrid picks its origin and fraction from: the
            // least corner's floor, and the widest axis from there.
            const size_t count = p.verts.size() / 8;
            float lo[2] = {0.0f, 0.0f}, hi[2] = {0.0f, 0.0f};
            for (size_t v = 0; v < count; ++v)
                for (int a = 0; a < 2; ++a) {
                    const float uv = p.verts[v * 8 + 6 + a];
                    if (v == 0 || uv < lo[a]) lo[a] = uv;
                    if (v == 0 || uv > hi[a]) hi[a] = uv;
                }
            float range = 0.0f;
            for (int a = 0; a < 2; ++a) {
                const float r = hi[a] - std::floor(lo[a]);
                if (r > range) range = r;
            }
            if (range > kMaxStorableRepeats) {
                char buf[512];
                std::snprintf(buf, sizeof(buf),
                              "mesh %s part %s: UVs span %.0f repeats, more "
                              "than a 4-bit UV grid holds (%.0f); they "
                              "saturate and the part is drawn wrong",
                              meshName.c_str(), p.name.c_str(), range,
                              kMaxStorableRepeats);
                out.push_back(buf);
            }
        }
        const size_t tris = p.verts.size() / 24;
        float worst = 0.0f;
        size_t over = 0;
        for (size_t t = 0; t < tris; ++t) {
            float lo[2] = {0.0f, 0.0f};
            float hi[2] = {0.0f, 0.0f};
            for (int k = 0; k < 3; ++k) {
                const float* v = &p.verts[(t * 3 + k) * 8 + 6];
                for (int a = 0; a < 2; ++a) {
                    if (k == 0 || v[a] < lo[a]) lo[a] = v[a];
                    if (k == 0 || v[a] > hi[a]) hi[a] = v[a];
                }
            }
            const float span = std::max(hi[0] - lo[0], hi[1] - lo[1]);
            if (span > worst) worst = span;
            if (span > kMaxRepeats) ++over;
        }
        if (over == 0) continue;
        char buf[512];
        std::snprintf(buf, sizeof(buf),
                      "mesh %s part %s: %zu triangle(s) span more than %.0f "
                      "repeats (the largest %.1f); they are drawn at a coarser "
                      "texture scale (lower precision)",
                      meshName.c_str(), p.name.c_str(), over, kMaxRepeats,
                      worst);
        out.push_back(buf);
    }
    return out;
}

namespace {

// A bounds-checked cursor. Every read goes through it, so a truncated or
// hand-corrupted file fails the whole parse instead of walking off the end -
// the reader's one job beyond the layout itself.
struct Cursor {
    const std::string& s;
    size_t at = 0;
    bool bad = false;

    bool take(void* dst, size_t n) {
        if (bad || at + n > s.size()) return !(bad = true);
        if (dst) std::memcpy(dst, s.data() + at, n);
        at += n;
        return true;
    }
    bool skip(size_t n) { return take(nullptr, n); }
    uint32_t u32() {
        uint32_t v = 0;
        take(&v, 4);
        return v;
    }
    float f32() {
        float v = 0.0f;
        take(&v, 4);
        return v;
    }
    // A fixed NUL-padded field back to a std::string.
    std::string fixed(size_t size) {
        char buf[128] = {};
        if (size > sizeof(buf) - 1 || !take(buf, size)) return std::string();
        return std::string(buf);
    }
    // vertexCount + verts + optional AO, the version 1-4 float mesh. The
    // vertices themselves are skipped - the caller wants the COUNT, which is
    // what a package count is derived from - and the count is reported back
    // through `count`. The multiply is done in 64 bits: a corrupt count times
    // 32 overflows a 32-bit size and could wrap back INSIDE the buffer.
    bool skipMesh(uint32_t* count = nullptr) {
        const uint32_t n = u32();
        if (count) *count = n;
        if (!skip((size_t)((uint64_t)n * 8u * sizeof(float)))) return false;
        const uint64_t ao = u32();
        return skip((size_t)ao);
    }
    // The version-5 compact record: 14 bytes a vertex (pos 6, nrm 4, uv 4),
    // then the grid's 9 bytes and the AO table.
    bool skipCompact(uint32_t* count = nullptr) {
        const uint32_t n = u32();
        if (count) *count = n;
        if (!skip((size_t)((uint64_t)n * 14u))) return false;
        if (!skip(1 + 8)) return false;
        const uint64_t ao = u32();
        return skip((size_t)ao);
    }
};

}  // namespace

bool readInfo(const std::string& bytes, Info& out) {
    Cursor c{bytes};
    char magic[4] = {};
    if (!c.take(magic, 4) || std::memcmp(magic, "TMDL", 4) != 0) return false;
    const uint32_t version = c.u32();
    // The loader accepts a RANGE (tmdl.hpp), and so does this: the fields it
    // reads have been in the layout since v1 except stripRun, which is
    // defaulted to 0 below for anything older - "no strip", the honest answer
    // for a file baked before strips existed.
    if (version < 1 || version > kVersion) return false;

    Info info;
    for (int i = 0; i < 3; ++i) info.min[i] = c.f32();
    for (int i = 0; i < 3; ++i) info.max[i] = c.f32();
    // The version-5 box is not part of the grouping read (the info carries
    // what a batch layout needs, not positions), so it is seeked past.
    if (version >= 5) c.skip(4 * sizeof(float));
    const uint32_t partCount = c.u32();
    if (c.bad) return false;
    // A part is at least its fixed fields, so a count that could not fit in
    // what is left is rejected before it reserves anything on its word.
    if ((uint64_t)partCount * 160u > bytes.size()) return false;

    // The mesh skipper of this version: the compact record from v5, the float
    // one before.
    auto skipAny = [&](uint32_t* count) {
        return version >= 5 ? c.skipCompact(count) : c.skipMesh(count);
    };

    info.parts.reserve(partCount);
    for (uint32_t p = 0; p < partCount && !c.bad; ++p) {
        PartInfo part;
        part.name = c.fixed(32);
        part.texture = c.fixed(64);
        part.reflTexture = c.fixed(64);
        c.skip(3 * sizeof(float));  // kd
        if (version >= 2) c.skip(3 * sizeof(float));  // ke
        c.skip(sizeof(float));  // reflStrength
        c.skip(sizeof(uint32_t));  // flags
        if (!skipAny(&part.vertexCount)) break;
        const uint32_t lodCount = c.u32();
        if (c.bad || lodCount > 8) return false;
        for (uint32_t l = 0; l < lodCount; ++l)
            if (!skipAny(nullptr)) break;
        if (version >= 4) {
            part.stripRun = c.u32();
            // The strip twin of the base mesh and of every tier, in order.
            // Tier 0's count is the one a package count is derived from; a
            // zero here is the legal "this part did not strip smaller than
            // its list" and means the game renders it as a list.
            if (!skipAny(&part.stripVertexCount)) break;
            for (uint32_t l = 0; l < lodCount; ++l)
                if (!skipAny(nullptr)) break;
        }
        if (c.bad) break;
        info.parts.push_back(std::move(part));
    }
    if (c.bad || info.parts.size() != partCount) return false;
    out = std::move(info);
    return true;
}

}  // namespace tmdl
