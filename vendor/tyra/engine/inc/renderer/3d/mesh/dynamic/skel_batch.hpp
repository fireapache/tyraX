// The batch skin kernel: its data layout, its integer definition (cornerBatchRef) and the
// lane model of its MMI sequence. Plain C++, shared by the host verifier; the EE kernel runs
// the same steps in inline asm.
//
// A part's UNIQUE corners (skinSource[v] == v) are blended through their INFLUENCE ENTRIES:
// one entry per (unique corner, nonzero weight slot). Entries are laid out joint-major, so
// one joint's entries are contiguous and padded to a multiple of 8 (a block of eight is one
// 128-bit load per array). Each entry carries its vertex's bind index, bind normal and its
// Q15 weight (the four weights of a corner sum to exactly 32768). A one-influence corner's
// weight 32768 does not fit int16: it is stored as 16384 and its sums are doubled (doubled[u]).
//
// Per pose, a joint's broadcast images (Image) hold the folded matrix columns, the
// translation and the rotation columns, each splatted into all eight halfword lanes. One block
// of eight entries of joint j, output row r:
//   acc = sum_c A[r][c] q_c                   (three PMADDH; lane e is entry e)
//   th  = round(shift(acc) + b[r])            (word arithmetic: the cornerNarrowRef steps)
//   P   = th x w                              (PMULTH: one 32-bit product per entry)
// and the same for the normals with R and n, with the x16 and the >> (15 - expR) before the
// weight. Lane e of the accumulator is word (e >> 1) of the even register (e even) or the odd
// register (e odd): PMFHL.LW gives the even words, PMFHL.UW the odd ones. So P is stored
// split, the even lanes to slots base+0..3 and the odd lanes to base+4..7 (pSlot), and the
// CSR of each unique gathers its slots. Integer sums are order-free, so the gather equals
// cornerBatchRef bit for bit.
#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include "renderer/3d/mesh/dynamic/skel_skin.hpp"
#include "renderer/3d/mesh/dynamic/skel_skin_mmi.hpp"

namespace Tyra {
namespace SkelSkin {
namespace Batch {

using Fixed::Fold;
using Lanes::Acc;
using Lanes::H8;
using Lanes::W4;

constexpr s32 kWeightOne = 32768;  // the Q15 weights of a corner sum to this
constexpr s32 kWeightStored = 16384;  // a one-influence corner's weight, stored (doubled later)
constexpr u32 kBlock = 8;          // entries per 128-bit block
constexpr u32 kJointsMax = 256;    // joint ids are u8
constexpr u32 kNone = 0xFFFFFFFFu;

struct Group {
  u32 joint = 0;
  u32 first = 0;  // first entry (a multiple of kBlock)
  u32 count = 0;  // entries, a multiple of kBlock (padding entries have weight 0)
};

// A part's bind-time batch data, shared by every body of the model.
struct Part {
  u32 corners = 0;
  u32 unique = 0;
  u32 entries = 0;
  u32 padding = 0;               // padding entries (weight 0)
  std::vector<u32> cornerUnique;  // [corners] the unique index of each corner
  std::vector<u32> uniqueCorner;  // [unique] the corner each unique is
  std::vector<s16> qx, qy, qz;    // [entries] bind index of the entry's vertex
  std::vector<s16> nx, ny, nz;    // [entries] bind normal (int8, sign-extended)
  std::vector<u16> w;             // [entries] Q15 weight (16384 for a doubled corner), 0 on padding
  std::vector<u8> doubled;        // [unique] 1 when the corner has one influence (sums doubled)
  std::vector<Group> groups;      // joint-major
  std::vector<u32> csrOff;        // [unique + 1]
  std::vector<u32> csr;           // the P slot of each entry of each unique
};

// The Q15 weights of a corner's four slots: they sum exactly to kWeightOne (the largest weight
// takes the rounding difference, as weightsQ15 does in the scalar blend).
inline void weightsQ15(const u8* weights, s32* w) {
  const u32 wsum = (u32)weights[0] + weights[1] + weights[2] + weights[3];
  for (int k = 0; k < 4; ++k) w[k] = 0;
  if (wsum == 0) return;
  s32 tot = 0;
  int big = 0;
  for (int k = 0; k < 4; ++k) {
    w[k] = (s32)((weights[k] * (u32)kWeightOne + wsum / 2) / wsum);
    tot += w[k];
    if (weights[k] > weights[big]) big = k;
  }
  w[big] += kWeightOne - tot;
}

// The P slot of entry i: its block base, then the even lanes (0, 2, 4, 6) in slots 0..3 and the
// odd lanes (1, 3, 5, 7) in slots 4..7.
inline u32 pSlot(u32 i) {
  const u32 lane = i & (kBlock - 1);
  return (i & ~(kBlock - 1)) + ((lane & 1) ? 4 : 0) + (lane >> 1);
}

// Bind-time build of a part from its bind arrays and skinSource (first identical corner).
inline void buildPart(const s16* pos, const s8* nrm, const u8* joints, const u8* weights,
                      const u32* skinSource, u32 corners, Part& p) {
  p = Part();
  p.corners = corners;
  p.cornerUnique.assign(corners, 0);
  for (u32 v = 0; v < corners; ++v) {
    if (skinSource[v] == v) {
      p.cornerUnique[v] = (u32)p.uniqueCorner.size();
      p.uniqueCorner.push_back(v);
    } else {
      p.cornerUnique[v] = p.cornerUnique[skinSource[v]];
    }
  }
  p.unique = (u32)p.uniqueCorner.size();

  struct Ent {
    u32 u;
    u16 w;
    s16 q[3];
    s16 n[3];
  };
  std::vector<std::vector<Ent>> byJoint(kJointsMax);
  p.doubled.assign(p.unique, 0);
  for (u32 u = 0; u < p.unique; ++u) {
    const u32 c = p.uniqueCorner[u];
    s32 w[4];
    weightsQ15(&weights[4 * c], w);
    for (int k = 0; k < 4; ++k) {
      if (w[k] == 0) continue;
      Ent e;
      e.u = u;
      if (w[k] == kWeightOne) {
        e.w = (u16)kWeightStored;
        p.doubled[u] = 1;
      } else {
        e.w = (u16)w[k];
      }
      for (int i = 0; i < 3; ++i) {
        e.q[i] = pos[3 * c + i];
        e.n[i] = (s16)nrm[4 * c + i];
      }
      byJoint[joints[4 * c + k]].push_back(e);
    }
  }

  // Joint-major storage, each joint padded to a block. owner[i] is the entry's unique (kNone on
  // padding); the CSR is built from it.
  std::vector<u32> owner;
  for (u32 j = 0; j < kJointsMax; ++j) {
    if (byJoint[j].empty()) continue;
    Group g;
    g.joint = j;
    g.first = (u32)p.w.size();
    for (const Ent& e : byJoint[j]) {
      p.qx.push_back(e.q[0]);
      p.qy.push_back(e.q[1]);
      p.qz.push_back(e.q[2]);
      p.nx.push_back(e.n[0]);
      p.ny.push_back(e.n[1]);
      p.nz.push_back(e.n[2]);
      p.w.push_back(e.w);
      owner.push_back(e.u);
    }
    while ((p.w.size() - g.first) % kBlock != 0) {
      p.qx.push_back(0);
      p.qy.push_back(0);
      p.qz.push_back(0);
      p.nx.push_back(0);
      p.ny.push_back(0);
      p.nz.push_back(0);
      p.w.push_back(0);
      owner.push_back(kNone);
      ++p.padding;
    }
    g.count = (u32)p.w.size() - g.first;
    p.groups.push_back(g);
  }
  p.entries = (u32)p.w.size();

  p.csrOff.assign(p.unique + 1, 0);
  for (u32 i = 0; i < p.entries; ++i)
    if (owner[i] != kNone) ++p.csrOff[owner[i] + 1];
  for (u32 u = 0; u < p.unique; ++u) p.csrOff[u + 1] += p.csrOff[u];
  std::vector<u32> fill(p.csrOff.begin(), p.csrOff.end() - 1);
  p.csr.assign(p.csrOff[p.unique], 0);
  for (u32 i = 0; i < p.entries; ++i)
    if (owner[i] != kNone) p.csr[fill[owner[i]]++] = pSlot(i);
}

// The per-pose shifts of a fold, as the blend applies them (cornerNarrowRef's).
struct Shifts {
  s32 shA;  // acc >> shA (negative: << -shA)
  s32 shT;  // round(t >> shT) (zero: none; negative: << -shT)
  s32 pe;   // the position sum's final shift
  s32 shN;  // the normal term's shift: (acc x 16) >> shN
};

inline Shifts shiftsOf(const Fold& f) {
  Shifts s;
  s.shA = 7 - f.expA;
  s.shT = 8 - f.expT;
  s.pe = 15 + f.expT;  // Q15 weights: the sums carry 2^15 (the blend's unit)
  s.shN = 15 - f.expR;
  return s;
}

// The Q4 x127 renormalisation of a blended normal (the scalar tail's arithmetic). Returns 0 when
// the normal is too short to trust (the caller takes the scalar path), 1 otherwise.
inline int renormalise(const s32 sumN[3], s8 out[3]) {
  s32 N[3];
  for (int r = 0; r < 3; ++r) N[r] = (sumN[r] + 16384) >> 15;
  const s32 n2 = N[0] * N[0] + N[1] * N[1] + N[2] * N[2];
  if (n2 < 16384) return 0;
  // 1 / sqrt(m) by the same table the scalar blend reads (rsqrtTable, built from its formula).
  const int lz = 31 - __builtin_clz((unsigned)n2);
  const int e2 = lz & ~1;
  const s32 mq = e2 >= 8 ? (n2 >> (e2 - 8)) : (n2 << (8 - e2));
  const double m = 1.0 + ((mq - 256) + 0.5) / 256.0;
  const s32 t = (s32)std::floor(65536.0 / std::sqrt(m) + 0.5);
  const int sh = 16 + e2 / 2;
  for (int r = 0; r < 3; ++r) {
    const s32 u = (N[r] * t + 64) >> 7;
    s32 v = (u * 127 + (1 << (sh - 8))) >> (sh - 7);
    if (v > 127) v = 127;
    if (v < -127) v = -127;
    out[r] = (s8)v;
  }
  out[3] = 0;
  return 1;
}

// The integer definition: one unique corner blended directly (the reference the lane model
// must equal). Weights are Q15, the per-term steps are cornerNarrowRef's. Returns the flags:
// bit 0 a position axis clamped (out of box), bit 1 the normal is too short (scalar fallback,
// outputs not written). `sat` counts joint terms that leave int16 (the noSat fold forbids them).
inline u32 cornerBatchRef(const Fold& f, const s16* q, const s8* n, const u8* joints,
                          const u8* weights, s16* outPos, s8* outNrm, u32* sat) {
  s32 w[4];
  weightsQ15(weights, w);
  const Shifts sh = shiftsOf(f);
  s32 sumP[3] = {0, 0, 0};
  s32 sumN[3] = {0, 0, 0};
  for (int k = 0; k < 4; ++k) {
    if (w[k] == 0) continue;
    const u32 j = joints[k];
    if (j >= f.slots) continue;
    for (int r = 0; r < 3; ++r) {
      const s32 acc = (s32)f.a[j][r] * q[0] + (s32)f.a[j][3 + r] * q[1] + (s32)f.a[j][6 + r] * q[2];
      const s32 t8 = (sh.shA >= 0 ? (acc >> sh.shA) : (acc << -sh.shA)) + f.b[j][r];
      s32 th = sh.shT > 0 ? (t8 + (1 << (sh.shT - 1))) >> sh.shT
                          : (sh.shT == 0 ? t8 : t8 << (-sh.shT));
      if (th > 32767 || th < -32768) {
        ++*sat;
        th = th > 32767 ? 32767 : -32768;
      }
      sumP[r] += w[k] * th;
    }
    for (int r = 0; r < 3; ++r) {
      const s32 acc = (s32)f.r[j][r] * n[0] + (s32)f.r[j][3 + r] * n[1] + (s32)f.r[j][6 + r] * n[2];
      sumN[r] += w[k] * ((acc * 16) >> sh.shN);
    }
  }
  u32 flags = 0;
  for (int r = 0; r < 3; ++r) {
    s32 v = (sumP[r] + (1 << (sh.pe - 1))) >> sh.pe;
    if (v > 32767 || v < -32767) {
      flags |= 1;
      v = v > 0 ? 32767 : -32767;
    }
    outPos[r] = (s16)v;
  }
  outPos[3] = 0;
  if (!renormalise(sumN, outNrm)) return flags | 2;
  return flags;
}

// The broadcast images of one joint: the folded A (a[c][r] = column c, row r, splatted), the
// translation b[r] and the normal rotation R (r[c][r] splatted). 21 quadwords per joint.
struct Image {
  H8 a[3][3];
  H8 r[3][3];
  s32 b[3];
};

inline H8 splatH(s16 v) {
  H8 h;
  for (int i = 0; i < 8; ++i) h.h[i] = v;
  return h;
}

// Fold-to-broadcast: the images of every joint the part's groups use (joints >= slots stay
// unset and their blocks are skipped by runPose).
inline void buildImages(const Fold& f, const Part& p, std::vector<Image>& img) {
  img.resize(p.groups.size());  // one image per group (joint-major), not per joint id
  for (size_t gi = 0; gi < p.groups.size(); ++gi) {
    const Group& g = p.groups[gi];
    if (g.joint >= f.slots) continue;
    Image& im = img[gi];
    for (int c = 0; c < 3; ++c)
      for (int r = 0; r < 3; ++r) {
        im.a[c][r] = splatH(f.a[g.joint][3 * c + r]);
        im.r[c][r] = splatH(f.r[g.joint][3 * c + r]);
      }
    for (int r = 0; r < 3; ++r) im.b[r] = f.b[g.joint][r];
  }
}

// Per-entry products of a pose, split as the kernel writes them: pp[r][slot], pn[r][slot].
struct Products {
  std::vector<s32> pp[3];
  std::vector<s32> pn[3];
  u64 truncations = 0;  // lane values that left int16 (the noSat / narrowN folds forbid them)
  void resize(u32 n) {
    for (int r = 0; r < 3; ++r) {
      pp[r].assign(n, 0);
      pn[r].assign(n, 0);
    }
  }
};

// Word helpers of the lane model (PSRAW / PSLLW / PADDW with a per-pose count).
inline W4 shiftW(const W4& x, s32 sa) {
  W4 r;
  for (int k = 0; k < 4; ++k)
    r.w[k] = sa >= 0 ? (x.w[k] >> sa) : (s32)((uint32_t)x.w[k] << (-sa));
  return r;
}

// (x + 2^(sh-1)) >> sh, rounding half up (sh > 0); sh == 0 keeps x; sh < 0 shifts left.
inline W4 roundShiftW(const W4& x, s32 sh) {
  W4 r;
  for (int k = 0; k < 4; ++k) {
    if (sh > 0)
      r.w[k] = (s32)((uint32_t)x.w[k] + (1u << (sh - 1))) >> sh;
    else
      r.w[k] = sh == 0 ? x.w[k] : (s32)((uint32_t)x.w[k] << (-sh));
  }
  return r;
}

// PMFHL.UW: (lo1, hi1, lo3, hi3), the odd lanes 1, 3, 5, 7.
inline W4 pmfhlUW(const Acc& a) { return W4{{a.lo[1], a.hi[1], a.lo[3], a.hi[3]}}; }

// Two word registers (lanes 0,2,4,6 and 1,3,5,7) to the eight halfwords in lane order: lane 2k
// is the low half of even word k, lane 2k+1 the low half of odd word k (the EE's PSLLW 16 of the
// odd register, PAND of the even one, POR).
inline H8 interleave(const W4& even, const W4& odd, u64& truncations) {
  H8 h;
  for (int k = 0; k < 4; ++k) {
    const s32 e = even.w[k];
    const s32 o = odd.w[k];
    if (e < -32768 || e > 32767 || o < -32768 || o > 32767) ++truncations;
    h.h[2 * k] = (int16_t)(e & 0xFFFF);
    h.h[2 * k + 1] = (int16_t)(o & 0xFFFF);
  }
  return h;
}

// The weight multiply (PMULTH) of a term register: lane e's product goes to the P slot of
// entry base + e (pSlot).
inline void storeProducts(const H8& th, const H8& wv, u32 base, std::vector<s32>& dst) {
  for (u32 e = 0; e < kBlock; ++e)
    dst[pSlot(base + e)] = (s32)th.h[e] * (s32)wv.h[e];
}

// The lane model of the block sweep: every group, every block of eight entries, every output
// row (positions and normals). Products of entries of joints >= slots are zero.
// The shifts are template constants: the kernel's PSRAW / PSLLW take immediates (the EE has
// no per-lane variable halfword shift; this scalar kernel takes them as values).

inline void runPoseT(const Fold& f, const Part& p, const std::vector<Image>& img, Products& out,
                     s32 SA, s32 ST, s32 SN) {
  out.resize(p.entries);
  for (size_t gi = 0; gi < p.groups.size(); ++gi) {
    const Group& g = p.groups[gi];
    if (g.joint >= f.slots) continue;  // products stay zero
    const Image& im = img[gi];
    for (u32 base = g.first; base < g.first + g.count; base += kBlock) {
      H8 qv[3], nv[3], wv;
      for (u32 e = 0; e < kBlock; ++e) {
        qv[0].h[e] = p.qx[base + e];
        qv[1].h[e] = p.qy[base + e];
        qv[2].h[e] = p.qz[base + e];
        nv[0].h[e] = p.nx[base + e];
        nv[1].h[e] = p.ny[base + e];
        nv[2].h[e] = p.nz[base + e];
        wv.h[e] = (int16_t)p.w[base + e];
      }
      for (int r = 0; r < 3; ++r) {
        // Positions: acc (the PMADDH sweep), split to even/odd words, the term, the weight.
        Acc acc = Acc();
        Lanes::pmaddh(acc, qv[0], im.a[0][r]);
        Lanes::pmaddh(acc, qv[1], im.a[1][r]);
        Lanes::pmaddh(acc, qv[2], im.a[2][r]);
        const W4 ev = Lanes::pmfhlLW(acc);
        const W4 od = pmfhlUW(acc);
        const W4 bv = {{im.b[r], im.b[r], im.b[r], im.b[r]}};
        const H8 th = interleave(roundShiftW(Lanes::paddW(shiftW(ev, SA), bv), ST),
                                 roundShiftW(Lanes::paddW(shiftW(od, SA), bv), ST),
                                 out.truncations);
        storeProducts(th, wv, base, out.pp[r]);

        // Normals: acc x 16 >> shN, the same split, the weight.
        Acc accn = Acc();
        Lanes::pmaddh(accn, nv[0], im.r[0][r]);
        Lanes::pmaddh(accn, nv[1], im.r[1][r]);
        Lanes::pmaddh(accn, nv[2], im.r[2][r]);
        const W4 en = Lanes::sllW(Lanes::pmfhlLW(accn), 4);
        const W4 on = Lanes::sllW(pmfhlUW(accn), 4);
        const H8 tn = interleave(shiftW(en, SN), shiftW(on, SN), out.truncations);
        storeProducts(tn, wv, base, out.pn[r]);
      }
    }
  }
}

// The block sweep for the given shifts (runtime values: the plain C++ kernel is scalar code on
// the EE, so a variable word shift costs the same as an immediate one). The inline-asm kernel
// will take the shifts as immediates through a template; see docs/animated-models.md.
inline bool runPoseFor(const Fold& f, const Part& p, const std::vector<Image>& img, Products& out) {
  const Shifts sh = shiftsOf(f);
  runPoseT(f, p, img, out, sh.shA, sh.shT, sh.shN);
  return true;
}

// The final pass: each unique corner's P slots are summed (the CSR), rounded and clamped
// (positions) and renormalised (normals). outPos [unique][4], outNrm [unique][4]; flags[u] has
// bit 0 for a clamped position axis and bit 1 for a short normal (scalar fallback; its output
// is not written). Returns the number of clamped uniques.
inline u32 finishPose(const Fold& f, const Part& p, const Products& P, s16* outPos, s8* outNrm,
                      u32* flags) {
  const Shifts sh = shiftsOf(f);
  u32 clamped = 0;
  for (u32 u = 0; u < p.unique; ++u) {
    s32 sumP[3] = {0, 0, 0};
    s32 sumN[3] = {0, 0, 0};
    for (u32 k = p.csrOff[u]; k < p.csrOff[u + 1]; ++k) {
      const u32 s = p.csr[k];
      for (int r = 0; r < 3; ++r) {
        sumP[r] += P.pp[r][s];
        sumN[r] += P.pn[r][s];
      }
    }
    if (p.doubled[u]) {  // the one-influence weight 32768 = 2 x the stored 16384
      for (int r = 0; r < 3; ++r) {
        sumP[r] *= 2;
        sumN[r] *= 2;
      }
    }
    u32 fl = 0;
    s16* oq = &outPos[4 * u];
    for (int r = 0; r < 3; ++r) {
      s32 v = (sumP[r] + (1 << (sh.pe - 1))) >> sh.pe;
      if (v > 32767 || v < -32767) {
        fl |= 1;
        v = v > 0 ? 32767 : -32767;
      }
      oq[r] = (s16)v;
    }
    oq[3] = 0;
    if (fl & 1) ++clamped;
    if (!renormalise(sumN, &outNrm[4 * u])) fl |= 2;
    flags[u] = fl;
  }
  return clamped;
}

}  // namespace Batch
}  // namespace SkelSkin
}  // namespace Tyra
