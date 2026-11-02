// Host harness for the VU framework's compact-input tooling (docs/vu-framework.md,
// "Compact input"; docs/compact-vertex-format.md): the itof4 / itof12 / itof15
// conversions in the IR, the assembler and the simulator; vucap's VIF model (every
// unpack format as it lands in VU memory, STMASK / STROW / STCOL / STMOD / STCYCL,
// an unpack's data running across words and tags, captures with compact runs); the
// compact staging held to the engine's own words; and compactEquivalence itself,
// which must pass a correct compact twin of every cull and clip program and catch a
// wrong one. Build and run from the TyraX root, after one editor configure (it
// writes the tamtypes.h host shim to build/host-shim):
//   g++ -std=c++20 -O1 -Wall -Wextra -Isrc -Ibuild/host-shim -Ivendor/tyra/engine/inc
//       -o build/verify-vu-compact tools/verify-vu-compact.cpp src/vuir.cpp
//       src/vuasm.cpp src/vusim.cpp src/vucap.cpp src/vugen.cpp
//       vendor/tyra/engine/src/renderer/3d/pipeline/static/core/compact/stapip_compact.cpp
//   build/verify-vu-compact        (prints "verify-vu-compact: <n> checks, 0 failed")
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../src/vuasm.hpp"
#include "../src/vucap.hpp"
#include "../src/vucompact_check.hpp"
#include "../src/vugen.hpp"
#include "../src/vuir.hpp"
#include "../src/vusim.hpp"

namespace {

int g_checks = 0, g_failed = 0;

#define CHECK(x)                                                         \
    do {                                                                 \
        ++g_checks;                                                      \
        if (!(x)) {                                                      \
            ++g_failed;                                                  \
            std::printf("FAIL line %d: %s\n", __LINE__, #x);             \
        }                                                                \
    } while (0)

using u32 = uint32_t;

float simItof(int fracBits, u32 bits) { return vusim::simItof(fracBits, bits); }

bool parseVcl(const std::string& text, vuir::Program& out, std::string& error) {
    return vuasm::parseText(text, "verify-vu-compact", vuasm::Options{}, out, error);
}

bool parses(const char* line) {
    vuir::Program p;
    std::string err;
    return parseVcl(std::string("#vuprog Check\n") + line + "\n#endvuprog\n", p, err) &&
           p.code.size() == 1;
}

u32 bits(float f) {
    u32 b;
    std::memcpy(&b, &f, 4);
    return b;
}

// VIFcodes, by hand.
u32 vifStrow() { return 0x30000000u; }
u32 vifStcol() { return 0x31000000u; }
u32 vifStmask() { return 0x20000000u; }
u32 vifStmod(u32 m) { return 0x05000000u | m; }
u32 vifStcycl(u32 cl, u32 wl) { return 0x01000000u | (wl << 8) | cl; }
u32 vifMscal(u32 a) { return 0x14000000u | a; }
// UNPACK: cmd 0x60 | m << 4 | vn << 2 | vl; imm = addr | USN << 14 | FLG << 15.
u32 vifUnpack(u32 vn, u32 vl, bool masked, u32 num, u32 addr, bool usn, bool tops) {
    const u32 cmd = 0x60u | (masked ? 0x10u : 0u) | (vn << 2) | vl;
    return (cmd << 24) | ((num & 0xFFu) << 16) | (addr & 0x3FFu) | (usn ? 1u << 14 : 0u) |
           (tops ? 1u << 15 : 0u);
}

bool wordsAre(const vucap::Unpack& u, size_t element, std::vector<u32> want) {
    if (u.words.size() < element * 4 + 4) return false;
    for (size_t f = 0; f < 4; ++f)
        if (u.words[element * 4 + f] != want[f]) return false;
    return true;
}

// ---------------------------------------------------------------------------
// The ops: itof4 / itof12 / itof15 in the IR, the assembler, the simulator
// ---------------------------------------------------------------------------

void ops() {
    std::printf("-- simulator ops --\n");
    CHECK(simItof(12, 0x00001000) == 1.0f && simItof(12, 0xFFFF8000) == -8.0f);
    CHECK(simItof(4, 16) == 1.0f && simItof(15, 0x8000) == 1.0f && simItof(0, (u32)-5) == -5.0f);
    CHECK(parses("itof12.xy vf1, vf2"));
    CHECK(std::isnan(simItof(7, 1)));  // no such instruction
    int failed = 0;
    const std::vector<vusim::OpCheck> all = vusim::checkOps(&parseVcl);
    for (const vusim::OpCheck& k : all) {
        CHECK(k.ok);
        if (!k.ok) {
            ++failed;
            std::printf("  FAILED %s: %s\n", k.name.c_str(), k.detail.c_str());
        }
    }
    CHECK(all.size() >= 30);  // 16 known answers, each parsed back, + 4 spellings
    std::printf("  %d op checks, %d failed\n", (int)all.size(), failed);

    // The builder: Vu::itof emits the instruction, names it, and a bad fracBits says so.
    vuir::Program p;
    vugen::Vu b(p);
    const vugen::Val in = b.lqConst(0);
    const vugen::Val out = b.itof(in, 12, (uint8_t)(vuir::MX | vuir::MY));
    b.sq(out, b.izero(), 1);
    b.finish();
    CHECK(p.code.size() == 3 && p.code[1].op == vuir::Op::Itof12);
    CHECK(vuir::disassemble(p, p.code[1]).rfind("itof12.xy", 0) == 0);
    std::vector<u32> mem(8, 0u);
    mem[0] = 0x2000u;            // 2.0 in 4.12
    mem[1] = 0xFFFFF000u;        // -1.0
    mem[2] = mem[3] = 12345u;    // masked off: the register's own (0) stays
    const vusim::Result r = vusim::run(p, mem, {});
    CHECK(r.ok && r.mem[4] == bits(2.0f) && r.mem[5] == bits(-1.0f) && r.mem[6] == 0u && r.mem[7] == 0u);
    vuir::Program q;
    vugen::Vu bq(q);
    bq.itof(bq.lqConst(0), 7);
    CHECK(q.code.back().op == vuir::Op::Itof0 && !q.notes.empty());
    vuir::Program pressure;
    vugen::Vu bp(pressure);
    bp.sq(bp.itof(bp.lqConst(0), 15), bp.izero(), 1);
    CHECK(vugen::vfPressure(pressure).peak == 2);  // itof reads and writes VF registers
}

// ---------------------------------------------------------------------------
// vucap: what lands in VU memory
// ---------------------------------------------------------------------------

void vif() {
    std::printf("-- vucap: unpacks as they land --\n");
    std::string err;
    {
        // STROW(0, 0, 1.0f, 1.0f) STMASK(z, w from ROW) STCYCL UNPACK V2-16 masked signed
        // num=3 {(-1, 4096), (32767, -32768), (0, 0)}
        vucap::VifRegs regs;
        const std::vector<u32> w = {
            vifStrow(), 0u, 0u, 0x3F800000u, 0x3F800000u, vifStmask(), 0x50505050u,
            vifStcycl(1, 1), vifUnpack(1, 1, true, 3, 77, false, true),
            0x1000FFFFu, 0x80007FFFu, 0x00000000u};
        const auto u = vucap::decodeVif(w, regs, &err);
        CHECK(err.empty() && u.size() == 1);
        CHECK(u.size() == 1 && u[0].count == 3 && u[0].vuAddr == 77 && u[0].useTops && u[0].masked && !u[0].usn);
        CHECK(u.size() == 1 && wordsAre(u[0], 0, {0xFFFFFFFFu, 0x1000u, 0x3F800000u, 0x3F800000u}));
        CHECK(u.size() == 1 && wordsAre(u[0], 1, {0x7FFFu, 0xFFFF8000u, 0x3F800000u, 0x3F800000u}));
        CHECK(u.size() == 1 && wordsAre(u[0], 2, {0u, 0u, 0x3F800000u, 0x3F800000u}));
        CHECK(u.size() == 1 && u[0].floats[1] == 4096.0f && u[0].floats[2] == 1.0f);  // data itof0, ROW as float
    }
    {
        // The engine's way: the bag setup's COL, an ST run header - the same lanes.
        Tyra::Compact::Box box = {{0.0f, 0.0f, 0.0f}, 1.0f / 512};
        std::vector<u32> w(Tyra::Compact::kBagSetupWords);
        Tyra::Compact::bagSetup(box, w.data());
        u32 h[4];
        Tyra::Compact::runHeader(Tyra::Compact::Attr::St, 3, 77, h);
        w.insert(w.end(), h, h + 4);
        w.insert(w.end(), {0x1000FFFFu, 0x80007FFFu, 0x00000000u, 0u});  // + a NOP pad
        vucap::VifRegs regs;
        const auto u = vucap::decodeVif(w, regs, &err);
        CHECK(err.empty() && u.size() == 1 && wordsAre(u[0], 0, {0xFFFFFFFFu, 0x1000u, 0x3F800000u, 0x3F800000u}));
        CHECK(regs.cl == 1 && regs.wl == 1 && regs.mode == 0 && regs.mask == 0xA0A0A0A0u && regs.col[0] == 0x3F800000u);
    }
    {
        // V3-16 num=5 consumes 8 words; the next VIFcode is read from word 9.
        vucap::VifRegs regs;
        std::vector<u32> w = {vifUnpack(2, 1, false, 5, 0, false, false)};
        for (u32 i = 0; i < 8; ++i) w.push_back(0x00010000u * (2 * i + 2) + (2 * i + 1));  // halfwords 1..16
        w.push_back(vifUnpack(3, 2, false, 1, 9, true, false));  // word 9: V4-8 unsigned
        w.push_back(0x04030201u);
        const auto u = vucap::decodeVif(w, regs, &err);
        CHECK(err.empty() && u.size() == 2);
        if (u.size() == 2) {
            CHECK(u[0].words.size() == 20);
            CHECK(wordsAre(u[0], 0, {1u, 2u, 3u, 4u}));    // V3: w = the next element's x
            CHECK(wordsAre(u[0], 4, {13u, 14u, 15u, 16u}));
            CHECK(u[1].vuAddr == 9 && wordsAre(u[1], 0, {1u, 2u, 3u, 4u}));
        }
        // One word short: the second UNPACK is still data, and the stream says so.
        w.resize(w.size() - 1);
        vucap::VifRegs again;
        vucap::decodeVif(w, again, &err);
        CHECK(!err.empty());
    }
    {
        // V4-8 0xFF: unsigned 255, signed 0xFFFFFFFF; V2-32 writes x, y again in z, w.
        vucap::VifRegs regs;
        const std::vector<u32> w = {vifUnpack(3, 2, false, 1, 0, true, false), 0xFFFFFFFFu,
                                    vifUnpack(3, 2, false, 1, 1, false, false), 0x7F80FF01u,
                                    vifUnpack(1, 0, false, 1, 2, false, false), 5u, 6u};
        const auto u = vucap::decodeVif(w, regs, &err);
        CHECK(err.empty() && u.size() == 3);
        if (u.size() == 3) {
            CHECK(wordsAre(u[0], 0, {255u, 255u, 255u, 255u}));
            CHECK(wordsAre(u[1], 0, {1u, 0xFFFFFFFFu, 0xFFFFFF80u, 0x7Fu}));
            CHECK(wordsAre(u[2], 0, {5u, 6u, 5u, 6u}));
            CHECK(u[1].floats[1] == -1.0f && u[1].floats[2] == -128.0f);
        }
    }
    {
        // STMOD offset: ROW added to the data lanes as integers, not to the COL one -
        // the compact position: a box-local int16 plus the origin index.
        vucap::VifRegs regs;
        const std::vector<u32> w = {
            vifStrow(), 100000u, (u32)-5, 7u, 999u, vifStcol(), 0x3F800000u, 0u, 0u, 0u,
            vifStmask(), 0x80808080u, vifStmod(1), vifUnpack(2, 1, true, 1, 3, false, true),
            0x00020001u, 0x0000FFFDu,
            // difference: ROW += data, and the lane is the sum
            vifStrow(), 10u, 10u, 10u, 10u, vifStmod(2), vifUnpack(3, 0, false, 2, 5, false, false),
            1u, 1u, 1u, 1u, 2u, 2u, 2u, 2u,
            // normal again: no ROW
            vifStmod(0), vifUnpack(3, 0, false, 1, 7, false, false), 4u, 4u, 4u, 4u};
        const auto u = vucap::decodeVif(w, regs, &err);
        CHECK(err.empty() && u.size() == 3);
        if (u.size() == 3) {
            CHECK(u[0].mode == 1 && wordsAre(u[0], 0, {100001u, (u32)-3, 4u, 0x3F800000u}));
            CHECK(u[0].floats[0] == 100001.0f && u[0].floats[3] == 1.0f);
            CHECK(u[1].mode == 2 && wordsAre(u[1], 0, {11u, 11u, 11u, 11u}) && wordsAre(u[1], 1, {13u, 13u, 13u, 13u}));
            CHECK(regs.row[0] == 13u && regs.mode == 0);
            CHECK(wordsAre(u[2], 0, {4u, 4u, 4u, 4u}));
        }
    }
    {
        // Write-protect (mask 3) reads 0; COL follows the write cycle (STCYCL 2/2).
        vucap::VifRegs regs;
        const std::vector<u32> w = {vifStcol(), 11u, 22u, 33u, 44u, vifStcycl(2, 2), vifStmask(),
                                    0x0000C0A0u,
                                    vifUnpack(3, 0, true, 2, 0, false, false), 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u};
        // 0xA0 = cycle 0: x, y data, z and w COL (10); 0xC0 = cycle 1: w protected (11)
        const auto u = vucap::decodeVif(w, regs, &err);
        CHECK(err.empty() && u.size() == 1);
        if (u.size() == 1) {
            CHECK(wordsAre(u[0], 0, {1u, 2u, 11u, 11u}));
            CHECK(wordsAre(u[0], 1, {5u, 6u, 7u, 0u}));
        }
    }
}

// A capture file: header, chain, then the REF'd blocks (docs/devkit.md).
std::vector<unsigned char> captureBytes(const std::vector<u32>& chain,
                                        const std::vector<std::pair<int, std::vector<u32>>>& blocks) {
    std::vector<unsigned char> b;
    auto put = [&](u32 v) {
        for (int i = 0; i < 4; ++i) b.push_back((unsigned char)(v >> (8 * i)));
    };
    b.insert(b.end(), {'T', 'X', 'V', 'U'});
    put(2);
    put(7);
    put((u32)(chain.size() / 4) | ((u32)blocks.size() << 16));
    for (u32 w : chain) put(w);
    for (const auto& bl : blocks) put((u32)bl.first | ((u32)(bl.second.size() / 4) << 16));
    for (const auto& bl : blocks)
        for (u32 w : bl.second) put(w);
    return b;
}

void captures() {
    std::printf("-- vucap: a capture with compact runs --\n");
    using namespace Tyra::Compact;
    const Box box = {{-512.0f, 0.0f, 1.0f}, 1.0f / 512};  // origin index (-262144, 0, 512)
    std::vector<u32> chain;
    auto tag = [&](u32 id, u32 qwc, u32 vif0, u32 vif1) {
        chain.insert(chain.end(), {(id << 28) | qwc, 0x00100000u, vif0, vif1});
    };
    // qw 0: cnt, inline: the bag setup (3 qw), a position run header and its run of 3
    // vertices (18 bytes: 2 qw) - VIFcodes inside an inline payload.
    tag(1, 6, 0u, 0u);
    std::vector<u32> setup(kBagSetupWords);
    bagSetup(box, setup.data());
    chain.insert(chain.end(), setup.begin(), setup.end());
    u32 h[4];
    runHeader(Attr::Position, 3, 2, h);
    chain.insert(chain.end(), h, h + 4);
    const int16_t q[9] = {-1, 0, 1, 100, -100, 2, 32767, -32767, 0};
    std::vector<u32> pos(8, 0u);
    std::memcpy(pos.data(), q, sizeof q);
    chain.insert(chain.end(), pos.begin(), pos.end());
    // qw 7: ref, captured below: an ST run header and its payload (one quadword).
    tag(3, 2, 0u, 0u);
    // qw 8: ref, NOT captured, whose UNPACK sits in the tag's own slot.
    tag(3, 1, 0u, vifUnpack(3, 0, false, 1, 100, false, true));
    // qw 9: end, MSCAL in its slot - stamps the three unpacks.
    tag(7, 0, 0u, vifMscal(42));
    std::vector<u32> stBlock(4);
    runHeader(Attr::St, 3, 5, stBlock.data());
    stBlock.insert(stBlock.end(), {0x1000FFFFu, 0x80007FFFu, 0u, 0u});
    vucap::Capture cap;
    CHECK(vucap::loadBytes(captureBytes(chain, {{7, stBlock}}), cap));
    CHECK(cap.loaded && cap.unpacks.size() == 3);
    if (cap.unpacks.size() == 3) {
        const vucap::Unpack& p = cap.unpacks[0];
        CHECK(p.format == "V3_16 (masked)" && p.mode == 1 && p.useTops && p.vuAddr == 2 && p.captured);
        CHECK(wordsAre(p, 0, {(u32)(-1 - 262144), 0u, 513u, 0x3F800000u}));
        CHECK(wordsAre(p, 2, {(u32)(32767 - 262144), (u32)-32767, 512u, 0x3F800000u}));
        CHECK(p.program == 42 && p.posNote.empty());
        const vucap::Unpack& st = cap.unpacks[1];
        CHECK(st.format == "V2_16 (masked)" && st.mode == 0 && st.captured);
        CHECK(wordsAre(st, 0, {0xFFFFFFFFu, 0x1000u, 0x3F800000u, 0x3F800000u}));
        CHECK(!st.posNote.empty());  // an ST is never a mesh
        CHECK(!cap.unpacks[2].captured && cap.unpacks[2].words.empty());
    }
    CHECK(cap.meshes.size() == 1 && cap.meshes[0].gridUnits && cap.meshes[0].verts == 3);
    if (cap.meshes.size() == 1) {
        CHECK(cap.meshes[0].lo[0] == -262145.0f && cap.meshes[0].hi[2] == 514.0f);
    }
    bool sawRow = false, sawCol = false, sawMod = false, sawGap = false;
    for (const vucap::Step& s : cap.steps) {
        sawRow = sawRow || s.text.find("STROW") != std::string::npos;
        sawCol = sawCol || s.text.find("STCOL") != std::string::npos;
        sawMod = sawMod || s.text.find("STMOD      offset") != std::string::npos;
        sawGap = sawGap || s.text.find("[payload not captured]") != std::string::npos;
    }
    CHECK(sawRow && sawCol && sawMod && sawGap);
    // An old-style chain (UNPACK in the tag's slot, V4_32 data inline) still decodes.
    std::vector<u32> old;
    old.insert(old.end(), {(1u << 28) | 3u, 0u, 0x01000101u, vifUnpack(3, 0, false, 3, 2, true, true)});
    for (int v = 0; v < 3; ++v) old.insert(old.end(), {bits((float)v), bits(2.0f), bits(3.0f), bits(1.0f)});
    old.insert(old.end(), {7u << 28, 0u, 0u, vifMscal(8)});
    vucap::Capture oc;
    CHECK(vucap::loadBytes(captureBytes(old, {}), oc) && oc.unpacks.size() == 1 && oc.meshes.size() == 1);
    CHECK(oc.meshes.size() == 1 && !oc.meshes[0].gridUnits && oc.meshes[0].hi[0] == 2.0f);
}

// ---------------------------------------------------------------------------
// compactEquivalence
// ---------------------------------------------------------------------------

enum class Flaw { None, StAsItof4, NoAlphaFix, ForgetEnvBasis };

/** A compact twin of a float program, for this harness only: a prologue turns the
 * staged compact input back into what the float program reads - every attribute
 * block converted in place (itof0 / itof12), the single colour converted and its
 * extra alpha taken off, the env basis's 1/127 undone for a program that
 * renormalises - and then the float program runs unchanged. A real compact program
 * (plan Task 3) converts inline after each load instead; this one exists so
 * compactEquivalence can be shown to pass a correct twin and to catch wrong ones. */
vuir::Program compactTwin(const vugen::Desc& d, const vuir::Program& floatProg,
                          bool renormalisesEnv, Flaw flaw) {
    using namespace vugen;
    const int top = 22;
    vuir::Program twin = floatProg;
    twin.code.clear();
    Vu b(twin);
    const IVal packed = b.ilw(b.izero(), top, 3, "twPacked");
    const IVal mask = b.iaddiu(b.izero(), 0x1FF, "twMask");
    const IVal n = b.iand(packed, mask, "twN");
    const IVal p = b.iaddiu(b.izero(), top + 2, "twP");
    const int blocks = d.texture ? 3 : 2;
    for (int blk = 0; blk < blocks; ++blk) {
        int frac = 0;
        uint8_t m = vuir::MXYZ;
        if (blk == 1 && d.texture && !d.env) {
            frac = flaw == Flaw::StAsItof4 ? 4 : 12;
            m = (uint8_t)(vuir::MX | vuir::MY);
        } else if (blk == blocks - 1 && !d.dirLights) {
            m = vuir::MALL;  // colours
        }
        const IVal i = b.iaddiu(n, 0, "twI");
        const Lbl loop = b.label(("twLoop" + std::to_string(blk)).c_str());
        b.bind(loop);
        const Val v = b.lq(p, 0, vuir::MALL, "twV");
        b.itofInto(v, v, frac, m);
        b.sq(v, p, 0);
        b.iaddiuInto(p, p, 1);
        b.iaddiuInto(i, i, -1);
        b.branchIfGtz(i, loop);
    }
    if (!d.dirLights) {
        const Val c = b.lqConst(7, vuir::MALL, "twSingle");
        b.itofInto(c, c, 0);
        if (flaw != Flaw::NoAlphaFix) b.subInto(c, c, b.zero().broadcast(3), vuir::MW);
        b.sq(c, b.izero(), 7);
    }
    if (d.env && renormalisesEnv && flaw != Flaw::ForgetEnvBasis) {
        b.loadI(127.0f);
        for (int r = 0; r < 3; ++r) {
            const Val e = b.lqConst(4 + r, vuir::MALL, "twEnv");
            vuir::Instr mul;
            mul.op = vuir::Op::Mul;
            mul.dst = e.reg;
            mul.s1 = e.reg;
            mul.s2 = vuir::kI;
            mul.s2kind = vuir::Src::I;
            mul.mask = (uint8_t)(vuir::MX | vuir::MY);
            twin.code.push_back(mul);
            b.sq(e, b.izero(), 4 + r);
        }
    }
    b.finish();
    const int shift = (int)twin.code.size();
    for (vuir::Instr in : floatProg.code) {
        if (in.target >= 0) in.target += shift;
        twin.code.push_back(in);
    }
    return twin;
}

void report(const char* label, const vugen::Tolerance& t) {
    std::printf("  %-26s %-8s %2d trials  XY %d  Z %d  fog %d  colour %d  ST rel %.1e abs %.1e  "
                "%d of %d words differ\n",
                label, t.within ? "WITHIN" : "OUTSIDE", t.trials, t.maxXyDelta, t.maxZDelta,
                t.maxFogDelta, t.maxColorDelta, t.maxFloatRel, t.maxFloatAbs, t.wordsDiffering,
                t.words);
    if (!t.error.empty()) std::printf("      %s\n", t.error.c_str());
    if (!t.detail.empty()) std::printf("      %s\n", t.detail.c_str());
}

bool parseEngine(const char* dir, const char* stem, vuir::Program& out) {
    vuasm::Options opt;
    opt.includeRoot = "vendor/tyra/engine";
    std::string err;
    const std::string path = std::string("vendor/tyra/engine/src/renderer/3d/pipeline/static/core/programs/") +
                             dir + "/" + stem + ".vclpp";
    if (!vuasm::parseFile(path, opt, out, err)) {
        std::printf("  cannot parse %s: %s\n", path.c_str(), err.c_str());
        return false;
    }
    return true;
}

void equivalence() {
    std::printf("-- compactEquivalence: a correct twin passes, a wrong one does not --\n");
    using vugen::ExperimentInput;
    const uint32_t seed = 0xC0A1C7u;
    struct Row {
        const char* label;
        vugen::Desc d;
        bool renormalisesEnv;
    };
    const Row rows[] = {
        {"cull C", vugen::descCullColor(), false},
        {"cull TC", vugen::descCullTextureColor(), false},
        {"cull D", vugen::descCullDirLights(), false},
        {"cull TD", vugen::descCullTextureDirLights(), false},
        {"cull TCE", vugen::descCullTextureEnv(), true},
        {"clip C", vugen::descClipColor(), false},
        {"clip TC", vugen::descClipTextureColor(), false},
        {"clip D", vugen::descClipDirLights(), false},
        {"clip TD", vugen::descClipTextureDirLights(), false},
        {"clip TCE", vugen::descClipTextureEnv(), true},
    };
    for (const Row& r : rows) {
        const vugen::Built built = vugen::build(r.d);
        const vuir::Program twin = compactTwin(r.d, built.program, r.renormalisesEnv, Flaw::None);
        const vugen::Tolerance t = vugen::compactEquivalence(built.program, twin, r.d, 60, seed);
        report(r.label, t);
        CHECK(t.ran && t.within && t.trials == 60 && t.error.empty());
        CHECK(t.words > 0);
        // Fed compact input, the float program itself is caught.
        const vugen::Tolerance raw = vugen::compactEquivalence(built.program, built.program, r.d, 60, seed);
        CHECK(raw.ran && !raw.within);
    }
    // The flaws a compact program could ship with, each caught.
    {
        const vugen::Desc d = vugen::descCullTextureColor();
        const vuir::Program prog = vugen::build(d).program;
        const vugen::Tolerance st = vugen::compactEquivalence(prog, compactTwin(d, prog, false, Flaw::StAsItof4), d, 60, seed);
        report("cull TC, ST as itof4", st);
        CHECK(st.ran && !st.within);
        const vugen::Tolerance alpha = vugen::compactEquivalence(prog, compactTwin(d, prog, false, Flaw::NoAlphaFix), d, 60, seed);
        report("cull TC, alpha + 1 kept", alpha);
        CHECK(alpha.ran && !alpha.within && alpha.maxColorDelta == 1);
        const vugen::Desc e = vugen::descCullTextureEnv();
        const vuir::Program envProg = vugen::build(e).program;
        const vugen::Tolerance env = vugen::compactEquivalence(envProg, compactTwin(e, envProg, true, Flaw::ForgetEnvBasis), e, 60, seed);
        report("cull TCE, basis left / 127", env);
        CHECK(env.ran && !env.within);
    }
    // The derived images the engine runs by default: folded lights, unit env normals.
    {
        vuir::Program fold, envn;
        if (parseEngine("cull", "stapip_cull_d_fold_vu1", fold)) {
            const vugen::Desc d = vugen::descCullDirLights();
            const vugen::Tolerance t = vugen::compactEquivalence(
                fold, compactTwin(d, fold, false, Flaw::None), d, 60, seed, ExperimentInput::FoldedLights);
            report("cull D fold (folded rows)", t);
            CHECK(t.within);
            // The fold image staged as if it read the matrix: the 1/127 lands nowhere.
            const vugen::Tolerance wrong = vugen::compactEquivalence(
                fold, compactTwin(d, fold, false, Flaw::None), d, 60, seed, ExperimentInput::Same);
            CHECK(!wrong.within);
        } else {
            CHECK(false);
        }
        if (parseEngine("cull", "stapip_cull_tce_envn_vu1", envn)) {
            const vugen::Desc d = vugen::descCullTextureEnv();
            const vugen::Tolerance t = vugen::compactEquivalence(
                envn, compactTwin(d, envn, false, Flaw::None), d, 60, seed);
            report("cull TCE envn", t);
            CHECK(t.within);
        } else {
            CHECK(false);
        }
    }
    // as_is keeps float input.
    {
        const vugen::Desc d = vugen::descAsIsColor();
        const vuir::Program prog = vugen::build(d).program;
        const vugen::Tolerance t = vugen::compactEquivalence(prog, prog, d, 4, seed);
        CHECK(!t.ran && !t.error.empty());
    }
    // The generic experiment check reads the same staging.
    {
        const vugen::Desc d = vugen::descCullTextureColor();
        const vuir::Program prog = vugen::build(d).program;
        const vugen::Tolerance t = vugen::experimentEquivalence(
            prog, compactTwin(d, prog, false, Flaw::None), d, ExperimentInput::Compact, 60, seed, 0, 0.0);
        report("experiment(Compact) TC", t);
        CHECK(t.within);
    }
}

void staging() {
    std::printf("-- compact staging vs the engine's words --\n");
    std::string detail;
    int checked = 0;
    const int bad = vucompact::checkStaging(60, 0x5EED5u, detail, &checked);
    std::printf("  %d trials over every cull and clip description, %d differ%s%s\n", checked, bad,
                detail.empty() ? "" : ": ", detail.c_str());
    CHECK(bad == 0 && checked == 10 * 60);
    // Both box kinds and a far world box are in the mix.
    const vugen::CompactTrial world = vugen::stageCompact(vugen::descCullColor(), 0, 0x5EED5u);
    const vugen::CompactTrial model = vugen::stageCompact(vugen::descCullColor(), 1, 0x5EED5u);
    CHECK(world.worldBox && !model.worldBox);
    CHECK(world.step >= 1.0f / 1024 && (bits(world.step) & 0x7FFFFFu) == 0 && (bits(model.step) & 0x7FFFFFu) == 0);
    // The checker is not vacuous: a staged lane moved by one is caught.
    vugen::CompactTrial broken = vugen::stageCompact(vugen::descCullTextureColor(), 3, 0x5EED5u);
    broken.compact[(size_t)(broken.top + vucompact::kVertData + broken.verts) * 4] += 1;  // an ST lane
    CHECK(!vucompact::checkTrial(broken).empty());
    vugen::CompactTrial refBroken = vugen::stageCompact(vugen::descCullColor(), 2, 0x5EED5u);
    refBroken.ref[(size_t)(refBroken.top + vucompact::kVertData) * 4] ^= 1u;  // a position float
    CHECK(!vucompact::checkTrial(refBroken).empty());
}

}  // namespace

int main() {
    std::printf("verify-vu-compact\n");
    ops();
    vif();
    captures();
    staging();
    equivalence();
    std::printf("verify-vu-compact: %d checks, %d failed\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
