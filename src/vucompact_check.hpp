// The compact staging, held to the engine (docs/vu-framework.md, "Compact input").
//
// vugen's compact equivalence stages a compact program's input itself, which
// means it restates the compact vertex format - what the VIF lands in VU
// memory - because vugen is also compiled into projects on its own and cannot
// include the engine (vugen.hpp, CompactTrial). This holds that restatement to
// the engine: each staged trial is re-made from the engine's own words
// (Tyra::Compact::bagSetup, runHeader, the runs packed the way the engine
// stores them), decoded by vucap's VIF model, and every attribute block must
// come out word for word what the stager wrote. The reference side is held to
// Tyra::Compact too: its floats are the engine's decode of the stored integers,
// and the engine's quantizers give those integers back.
//
// Header-only and editor-only (the engine's format links in through the
// tyrax-compact library): `--vu-check` and tools/verify-vu-compact.cpp run it.
#pragma once

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../vendor/tyra/engine/inc/renderer/3d/pipeline/static/core/compact/stapip_compact.hpp"
#include "vucap.hpp"
#include "vugen.hpp"

namespace vucompact {

/** VU1 VERT_DATA: where the first attribute block starts, relative to TOPS -
 * vugen's kVertDataAddr. The blocks follow it one vertex count apart. */
constexpr int kVertData = 2;

/** The payload of one run: `verts` elements of `stored` (four integers a
 * vertex, CompactTrial's layout) packed as the engine's format for `attr`
 * stores them, padded with zero words to whole quadwords. */
inline std::vector<uint32_t> packRun(Tyra::Compact::Attr attr,
                                     const std::vector<int32_t>& stored,
                                     int verts) {
    using namespace Tyra::Compact;
    std::vector<unsigned char> bytes((size_t)runQwords(attr, (u32)verts) * 16, 0);
    const Format fmt = formatOf(attr);
    const int lanes = fmt == Format::V2_16 ? 2 : fmt == Format::V3_16 ? 3 : 4;
    const int size = fmt == Format::V4_8 ? 1 : 2;
    for (int v = 0; v < verts; ++v)
        for (int a = 0; a < lanes; ++a) {
            const uint32_t q = (uint32_t)stored[(size_t)v * 4 + a];
            const size_t at = ((size_t)v * lanes + a) * size;
            bytes[at] = (unsigned char)(q & 0xFF);
            if (size == 2) bytes[at + 1] = (unsigned char)((q >> 8) & 0xFF);
        }
    std::vector<uint32_t> words(bytes.size() / 4);
    std::memcpy(words.data(), bytes.data(), bytes.size());
    return words;
}

/** Empty when `ct` is what the engine's words make the VIF write and what the
 * engine's decode gives the float program; else the first difference. */
inline std::string checkTrial(const vugen::CompactTrial& ct) {
    using namespace Tyra::Compact;
    char buf[200];
    Box box;
    box.step = ct.step;
    for (int a = 0; a < 3; ++a) box.origin[a] = ct.origin[a];
    std::vector<uint32_t> words(kBagSetupWords);
    bagSetup(box, words.data());
    for (size_t b = 0; b < ct.attr.size(); ++b) {
        const Attr attr = static_cast<Attr>(ct.attr[b]);
        u32 header[4];
        runHeader(attr, (u32)ct.verts, (u32)(kVertData + (int)b * ct.verts), header);
        words.insert(words.end(), header, header + 4);
        const std::vector<uint32_t> run = packRun(attr, ct.stored[b], ct.verts);
        words.insert(words.end(), run.begin(), run.end());
    }
    vucap::VifRegs regs;
    std::string err;
    const std::vector<vucap::Unpack> unpacks = vucap::decodeVif(words, regs, &err);
    if (!err.empty()) return "vucap: " + err;
    if (unpacks.size() != ct.attr.size()) {
        std::snprintf(buf, sizeof buf, "%d unpacks decoded for %d blocks",
                      (int)unpacks.size(), (int)ct.attr.size());
        return buf;
    }
    for (size_t b = 0; b < unpacks.size(); ++b) {
        const vucap::Unpack& u = unpacks[b];
        if (!u.useTops || (int)u.vuAddr != kVertData + (int)b * ct.verts ||
            (int)u.words.size() != ct.verts * 4)
            return "block " + std::to_string(b) + ": the run lands elsewhere";
        for (size_t i = 0; i < u.words.size(); ++i) {
            const size_t at = (size_t)(ct.top + (int)u.vuAddr) * 4 + i;
            if (u.words[i] != ct.compact[at]) {
                std::snprintf(buf, sizeof buf,
                              "block %d (attr %d) word %d: the VIF writes 0x%08X, "
                              "vugen staged 0x%08X",
                              (int)b, ct.attr[b], (int)i, u.words[i], ct.compact[at]);
                return buf;
            }
        }
        // The float program's side of the same integers.
        const Attr attr = static_cast<Attr>(ct.attr[b]);
        for (int v = 0; v < ct.verts; ++v)
            for (int a = 0; a < 4; ++a) {
                const size_t at = (size_t)(ct.top + (int)u.vuAddr + v) * 4 + (size_t)a;
                float f;
                std::memcpy(&f, &ct.ref[at], 4);
                const int32_t q = ct.stored[b][(size_t)v * 4 + a];
                bool ok = true;
                if (attr == Attr::Position && a < 3) {
                    const float want = dequantize((s16)q, box.origin[a], box.step);
                    ok = std::memcmp(&f, &want, 4) == 0 &&
                         quantize(f, box.origin[a], box.step) == q;
                } else if (attr == Attr::St && a < 3)
                    // S, T, Q: the float side reads them unscaled, the
                    // package's power-of-two scale is the compact side's
                    ok = quantizeSt(f * ct.stScale) == q;
                else if (attr == Attr::Color)
                    ok = quantizeColor(f) == q;
                else if ((attr == Attr::Normal || attr == Attr::EnvNormal) && a < 3)
                    ok = quantizeNormal(f) == q;
                if (!ok) {
                    std::snprintf(buf, sizeof buf,
                                  "block %d (attr %d) vertex %d lane %d: the float "
                                  "program reads %.9g for the stored %d",
                                  (int)b, ct.attr[b], v, a, (double)f, (int)q);
                    return buf;
                }
            }
    }
    return std::string();
}

/** Every cull and clip description, `trials` trials each: the count of trials
 * the engine's words disagree with, and the first difference in `detail`. */
inline int checkStaging(int trials, uint32_t seed, std::string& detail,
                        int* checked = nullptr) {
    int bad = 0, n = 0;
    for (const vugen::Desc& d : vugen::allDescs()) {
        if (!d.cull && !d.clip) continue;
        for (int t = 0; t < trials; ++t) {
            const vugen::CompactTrial ct = vugen::stageCompact(d, t, seed);
            ++n;
            const std::string e = ct.error.empty() ? checkTrial(ct) : ct.error;
            if (e.empty()) continue;
            if (bad++ == 0) detail = d.vclName + " trial " + std::to_string(t) + ": " + e;
        }
    }
    if (checked) *checked = n;
    return bad;
}

}  // namespace vucompact
