// The batch skin kernel: its data layout, its integer definition (cornerBatchRef), the lane
// model of its MMI sequence, and on the EE the inline-asm rows of that sequence.
//
// A part's UNIQUE corners (skinSource[v] == v) are blended through INFLUENCE ENTRIES: one entry
// per (unique corner, nonzero weight slot). Entries are laid out joint-major, each joint's
// entries padded to whole blocks of eight, so a block is one 128-bit load per array. A Block
// holds the bind index (x, y, z), the bind normal and the Q15 weight of its eight entries, each
// array 16-byte aligned. Blocks live in a 64-byte aligned arena owned by the part (shared by
// every body of the model).
//
// Per pose, a joint group's broadcast images (Image) hold the folded matrix columns and the
// translation, each splatted into all eight halfword lanes. For one block and output row r:
//   acc = sum_c A[r][c] q_c       (PMULTH, then PMADDH x2: lane e is entry e)
//   th  = round(acc >> SA + b) >> ST  (PSRAW immediates; the rounding is in the bias)
//   P   = th x w                  (PMULTH: one 32-bit product per entry)
// Normals are the same with R and n, with (acc << 4) >> SN before the weight.
// Lane e of an accumulator is word e/2 of the even register (e even) or odd (e odd):
// PMFHL.LW gives the even words, PMFHL.UW the odd ones. So each product block is stored as its
// even lanes (slots base+0..3) then its odd lanes (base+4..7): pSlot. The CSR of each unique
// gathers its slots; integer sums are order-free, so the gather equals cornerBatchRef.
//
// The EE shifts are immediates: the fold is forced to expA 0, expT 0, expR 1 (kForced), so
// SA = 7, ST = 8, SN = 14. A pose the forced exponents cannot hold is not noSat and takes the
// scalar blend (counted by the caller).
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

constexpr s32 kWeightOne = 32768;     // the Q15 weights of a corner sum to this
constexpr s32 kWeightStored = 16384;  // a one-influence corner's weight (sums doubled later)
constexpr u32 kBlock = 8;             // entries per block
constexpr u32 kJointsMax = 256;       // joint ids are u8
constexpr u32 kNone = 0xFFFFFFFFu;
// The forced fold exponents the kernel's immediates assume (expA, expT, expR).
constexpr s32 kForced[3] = {0, 0, 1};
constexpr s32 kShA = 7, kShT = 8, kShN = 14;  // 7 - expA, 8 - expT, 15 - expR

// The eight entries of a block: bind index per axis, bind normal per axis, Q15 weight.
struct alignas(16) Block {
  s16 q[3][8];
  s16 n[3][8];
  u16 w[8];
};
static_assert(sizeof(Block) == 112, "a block is seven quadwords");

struct Group {
  u32 joint = 0;
  u32 firstBlock = 0;  // the group's first block
  u32 blocks = 0;      // blocks (padding entries have weight 0)
};

// Storage aligned to 64 bytes inside a byte vector (the vector's own alignment is not enough for
// the 128-bit loads). Returns the aligned start; `raw` keeps the memory.
template <class T>
inline T* alignedIn(std::vector<u8>& raw, size_t count) {
  const size_t bytes = count * sizeof(T) + 64;
  if (raw.size() < bytes) raw.resize(bytes);
  uintptr_t p = (uintptr_t)raw.data();
  p = (p + 63) & ~(uintptr_t)63;
  return (T*)p;
}

// A part's bind-time batch data, shared by every body of the model.
struct Part {
  Part() = default;
  Part(const Part&) = delete;
  Part& operator=(const Part&) = delete;
  Part(Part&&) = default;
  Part& operator=(Part&&) = default;

  u32 corners = 0;
  u32 unique = 0;
  u32 blocks = 0;
  u32 padding = 0;                // padding entries (weight 0)
  std::vector<u32> cornerUnique;  // [corners] the unique index of each corner
  std::vector<u32> uniqueCorner;  // [unique] the corner each unique is
  std::vector<u8> arena;          // the aligned blocks' storage
  Block* blk = nullptr;           // [blocks], 16-byte aligned, inside arena
  std::vector<Group> groups;      // joint-major
  std::vector<u8> doubled;        // [unique] 1 when the corner has one influence
  std::vector<u32> csrOff;        // [unique + 1]
  std::vector<u32> csr;           // the P slot of each entry of each unique

  u32 entries() const { return blocks * kBlock; }
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
  {
    u32 uniques = 0;
    for (u32 v = 0; v < corners; ++v)
      if (skinSource[v] == v) ++uniques;
    p.uniqueCorner.reserve(uniques);  // exact: no doubling slack in the bind data
  }
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

  // Joint-major entries, each joint padded to whole blocks. owner[i] is the entry's unique
  // (kNone on padding).
  std::vector<Block> blocks;
  std::vector<u32> owner;
  for (u32 j = 0; j < kJointsMax; ++j) {
    if (byJoint[j].empty()) continue;
    Group g;
    g.joint = j;
    g.firstBlock = (u32)blocks.size();
    const size_t first = owner.size();
    for (const Ent& e : byJoint[j]) {
      owner.push_back(e.u);
    }
    while ((owner.size() - first) % kBlock != 0) {
      owner.push_back(kNone);
      ++p.padding;
    }
    g.blocks = (u32)((owner.size() - first) / kBlock);
    blocks.resize(blocks.size() + g.blocks);
    for (size_t i = first; i < owner.size(); ++i) {
      Block& B = blocks[i / kBlock];
      const u32 lane = (u32)(i & (kBlock - 1));
      const size_t k = i - first;
      if (k < byJoint[j].size()) {
        const Ent& e = byJoint[j][k];
        for (int c = 0; c < 3; ++c) {
          B.q[c][lane] = e.q[c];
          B.n[c][lane] = e.n[c];
        }
        B.w[lane] = e.w;
      } else {
        for (int c = 0; c < 3; ++c) {
          B.q[c][lane] = 0;
          B.n[c][lane] = 0;
        }
        B.w[lane] = 0;
      }
    }
    p.groups.push_back(g);
  }
  p.blocks = (u32)blocks.size();

  // The aligned arena copy of the blocks.
  p.blk = alignedIn<Block>(p.arena, blocks.size());
  for (size_t b = 0; b < blocks.size(); ++b) p.blk[b] = blocks[b];

  p.csrOff.assign(p.unique + 1, 0);
  for (size_t i = 0; i < owner.size(); ++i)
    if (owner[i] != kNone) ++p.csrOff[owner[i] + 1];
  for (u32 u = 0; u < p.unique; ++u) p.csrOff[u + 1] += p.csrOff[u];
  std::vector<u32> fill(p.csrOff.begin(), p.csrOff.end() - 1);
  p.csr.assign(p.csrOff[p.unique], 0);
  for (size_t i = 0; i < owner.size(); ++i)
    if (owner[i] != kNone) p.csr[fill[owner[i]]++] = pSlot((u32)i);
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

// 1 / sqrt(m) in Q16 for m = 1 + (i + 0.5) / 256, i in [0, 768): the table the scalar blend reads
// (the same formula as the scalar rsqrtTable). Built once per process: the EE does double math in
// software, so the per-normal sqrt is not computed per corner.
inline const s32* rsqrtTable() {
  struct Table {
    s32 v[768];
    Table() {
      for (int i = 0; i < 768; ++i) {
        const double m = 1.0 + (i + 0.5) / 256.0;
        v[i] = (s32)std::floor(65536.0 / std::sqrt(m) + 0.5);
      }
    }
  };
  static const Table table;
  return table.v;
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

  const s32 t = rsqrtTable()[mq - 256];
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

// The integer definition: one unique corner blended directly (the reference the lane model must
// equal). Weights are Q15, the per-term steps are cornerNarrowRef's. Returns the flags: bit 0 a
// position axis clamped (out of box), bit 1 the normal is too short (scalar fallback, outputs not
// written). `sat` counts joint terms that leave int16 (the noSat fold forbids them).
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

// The broadcast images of one joint group, 16-byte aligned: the folded A (a[c][r] = column c,
// row r, splatted), the normal rotation R (r[c][r], splatted), the translation b[r] as words and
// its rounding bias b[r] + 2^(kShT - 1) (the kernel's PADDW before PSRAW kShT).
struct alignas(16) Image {
  H8 a[3][3];
  H8 r[3][3];
  W4 bias[3];
  s32 b[3];
};
static_assert(sizeof(Image) == 352, "the EE kernel's image offsets");

inline H8 splatH(s16 v) {
  H8 h;
  for (int i = 0; i < 8; ++i) h.h[i] = v;
  return h;
}

// Fold-to-broadcast: the images of every group of the part, in group order. Groups whose joint
// is not folded (joint >= slots) keep zeros and are skipped by the sweep.
inline Image* buildImages(const Fold& f, const Part& p, std::vector<u8>& raw) {
  Image* img = alignedIn<Image>(raw, p.groups.size());
  for (size_t gi = 0; gi < p.groups.size(); ++gi) {
    const Group& g = p.groups[gi];
    Image& im = img[gi];
    if (g.joint >= f.slots) continue;
    for (int c = 0; c < 3; ++c)
      for (int r = 0; r < 3; ++r) {
        im.a[c][r] = splatH(f.a[g.joint][3 * c + r]);
        im.r[c][r] = splatH(f.r[g.joint][3 * c + r]);
      }
    for (int r = 0; r < 3; ++r) {
      im.b[r] = f.b[g.joint][r];
      const s32 bias = f.b[g.joint][r] + (1 << (kShT - 1));
      im.bias[r] = W4{{bias, bias, bias, bias}};
    }
  }
  return img;
}

// Per-entry products of a pose, split as the kernel writes them: pp[r][slot], pn[r][slot]. The
// three position rows and the three normal rows are contiguous 16-byte aligned runs.
struct Products {
  std::vector<u8> raw;
  s32* pp[3] = {nullptr, nullptr, nullptr};
  s32* pn[3] = {nullptr, nullptr, nullptr};
  u64 truncations = 0;  // lane values that left int16 (the noSat / narrowN folds forbid them)
  void resize(u32 entries) {
    s32* base = alignedIn<s32>(raw, 6 * (size_t)entries);
    for (int r = 0; r < 3; ++r) {
      pp[r] = base + (size_t)r * entries;
      pn[r] = base + (size_t)(3 + r) * entries;
    }
  }
};

// Word helpers of the lane model (PSRAW / PSLLW / PADDW with the pose's shifts).
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

// Two word registers (lanes 0,2,4,6 and 1,3,5,7) to the eight halfwords in lane order: lane 2k is
// the low half of even word k, lane 2k+1 the low half of odd word k.
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

// The weight multiply (PMULTH) of a term register: lane e's product goes to the P slot of entry
// base + e (pSlot).
inline void storeProducts(const H8& th, const H8& wv, u32 base, s32* dst) {
  for (u32 e = 0; e < kBlock; ++e) dst[pSlot(base + e)] = (s32)th.h[e] * (s32)wv.h[e];
}

#if defined(_EE)
// The MMI rows of a block (inline asm). kAsmRows selects the rows that run in asm, one bit each:
// bits 0-2 the position rows x, y, z; bits 3-5 the normal rows. The other rows are the lane model.
// The shifts are the forced immediates: positions PSRAW 7 then 8 (SA, ST), normals PSLLW 4 then
// PSRAW 14 (SN). Register map: $8-$11 the block's qx/qy/qz and w (or the normal axes and w), $12-$15
// the image's column and bias (or rotation), $16-$19 the intermediates; the stores go to dst.
#define BATCH_MMI_CLOBBERS                                                                       \
  "$8", "$9", "$10", "$11", "$12", "$13", "$14", "$15", "$16", "$17", "$18", "$19"
constexpr u32 kAsmRows = 63u;

template <int R>
inline void rowPosAsm(const Block* B, const Image* im, s32* dst) {
  constexpr int oa0 = (0 * 3 + R) * 16;  // a[c][R] at (c * 3 + R) * 16
  constexpr int oa1 = (1 * 3 + R) * 16;
  constexpr int oa2 = (2 * 3 + R) * 16;
  constexpr int ob = 288 + R * 16;  // bias[R]
  asm volatile(
      "lq       $8, 0(%[B])               \n\t"
      "lq       $9, 16(%[B])              \n\t"
      "lq       $10, 32(%[B])             \n\t"
      "lq       $11, 96(%[B])             \n\t"
      "lq       $12, %[a0](%[im])         \n\t"
      "lq       $13, %[a1](%[im])         \n\t"
      "lq       $14, %[a2](%[im])         \n\t"
      "lq       $15, %[bias](%[im])       \n\t"
      "pmulth   $0, $8, $12               \n\t"
      "pmaddh   $0, $9, $13               \n\t"
      "pmaddh   $0, $10, $14              \n\t"
      "pmfhl.lw $16                       \n\t"
      "pmfhl.uw $17                       \n\t"
      "psraw    $16, $16, 7               \n\t"
      "paddw    $16, $16, $15             \n\t"
      "psraw    $16, $16, 8               \n\t"
      "psraw    $17, $17, 7               \n\t"
      "paddw    $17, $17, $15             \n\t"
      "psraw    $17, $17, 8               \n\t"
      "psllw    $16, $16, 16              \n\t"
      "psrlw    $16, $16, 16              \n\t"
      "psllw    $17, $17, 16              \n\t"
      "por      $16, $16, $17             \n\t"
      "pmulth   $0, $16, $11              \n\t"
      "pmfhl.lw $18                       \n\t"
      "pmfhl.uw $19                       \n\t"
      "sq       $18, 0(%[dst])            \n\t"
      "sq       $19, 16(%[dst])           \n\t"
      :
      : [B] "r"(B), [im] "r"(im), [dst] "r"(dst), [a0] "i"(oa0), [a1] "i"(oa1), [a2] "i"(oa2),
        [bias] "i"(ob)
      : BATCH_MMI_CLOBBERS, "memory");
}

template <int R>
inline void rowNrmAsm(const Block* B, const Image* im, s32* dst) {
  constexpr int or0 = 144 + (0 * 3 + R) * 16;  // r[c][R] at 144 + (c * 3 + R) * 16
  constexpr int or1 = 144 + (1 * 3 + R) * 16;
  constexpr int or2 = 144 + (2 * 3 + R) * 16;
  asm volatile(
      "lq       $8, 48(%[B])              \n\t"  // nx
      "lq       $9, 64(%[B])              \n\t"  // ny
      "lq       $10, 80(%[B])             \n\t"  // nz
      "lq       $11, 96(%[B])             \n\t"  // w
      "lq       $12, %[r0](%[im])         \n\t"
      "lq       $13, %[r1](%[im])         \n\t"
      "lq       $14, %[r2](%[im])         \n\t"
      "pmulth   $0, $8, $12               \n\t"
      "pmaddh   $0, $9, $13               \n\t"
      "pmaddh   $0, $10, $14              \n\t"
      "pmfhl.lw $16                       \n\t"
      "pmfhl.uw $17                       \n\t"
      "psllw    $16, $16, 4               \n\t"  // acc x 16 >> 14
      "psraw    $16, $16, 14              \n\t"
      "psllw    $17, $17, 4               \n\t"
      "psraw    $17, $17, 14              \n\t"
      "psllw    $16, $16, 16              \n\t"
      "psrlw    $16, $16, 16              \n\t"
      "psllw    $17, $17, 16              \n\t"
      "por      $16, $16, $17             \n\t"
      "pmulth   $0, $16, $11              \n\t"
      "pmfhl.lw $18                       \n\t"
      "pmfhl.uw $19                       \n\t"
      "sq       $18, 0(%[dst])            \n\t"
      "sq       $19, 16(%[dst])           \n\t"
      :
      : [B] "r"(B), [im] "r"(im), [dst] "r"(dst), [r0] "i"(or0), [r1] "i"(or1), [r2] "i"(or2)
      : BATCH_MMI_CLOBBERS, "memory");
}
#endif

// The lane model of the block sweep: every group, every block of eight entries, every output row
// (positions and normals). Products of entries of joints >= slots are zero. On the EE the x row
// of the forced shifts runs the rows in kAsmRows as asm; everything else is the lane model.
inline void runPoseT(const Fold& f, const Part& p, const Image* img, Products& out, s32 SA,
                     s32 ST, s32 SN, bool asmX) {
  out.resize(p.entries());
  for (size_t gi = 0; gi < p.groups.size(); ++gi) {
    const Group& g = p.groups[gi];
    if (g.joint >= f.slots) {
      // products stay zero
      for (u32 bi = 0; bi < g.blocks; ++bi) {
        const u32 base = (g.firstBlock + bi) * kBlock;
        for (int r = 0; r < 3; ++r)
          for (u32 e = 0; e < kBlock; ++e) {
            out.pp[r][base + e] = 0;
            out.pn[r][base + e] = 0;
          }
      }
      continue;
    }
    const Image& im = img[gi];
    for (u32 bi = 0; bi < g.blocks; ++bi) {
      const u32 blk = g.firstBlock + bi;
      const Block& B = p.blk[blk];
      const u32 base = blk * kBlock;
      H8 qv[3], nv[3], wv;
      for (u32 e = 0; e < kBlock; ++e) {
        for (int c = 0; c < 3; ++c) {
          qv[c].h[e] = B.q[c][e];
          nv[c].h[e] = B.n[c][e];
        }
        wv.h[e] = (int16_t)B.w[e];
      }
#if defined(_EE)
      if (asmX) {
        if (kAsmRows & 1u) rowPosAsm<0>(&B, &im, out.pp[0] + base);
        if (kAsmRows & 2u) rowPosAsm<1>(&B, &im, out.pp[1] + base);
        if (kAsmRows & 4u) rowPosAsm<2>(&B, &im, out.pp[2] + base);
        if (kAsmRows & 8u) rowNrmAsm<0>(&B, &im, out.pn[0] + base);
        if (kAsmRows & 16u) rowNrmAsm<1>(&B, &im, out.pn[1] + base);
        if (kAsmRows & 32u) rowNrmAsm<2>(&B, &im, out.pn[2] + base);
      }
#endif
      for (int r = 0; r < 3; ++r) {
#if defined(_EE)
        if (!(asmX && (kAsmRows & (1u << r))))
#endif
        {
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
        }
#if defined(_EE)
        if (asmX && (kAsmRows & (8u << r))) continue;  // this normal row is in asm
#endif
        // Normals: acc x 16 >> SN, the same split, the weight.
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

// The kernel for a fold: the forced shifts take the EE's asm x row; any other shifts run the
// lane model for every row (the host's path, and the EE's for folds the forced exponents did not
// make). Returns false when the fold's shifts are not the forced ones and asm is needed.
inline bool runPoseFor(const Fold& f, const Part& p, const Image* img, Products& out) {
  const Shifts sh = shiftsOf(f);
  const bool forced = sh.shA == kShA && sh.shT == kShT && sh.shN == kShN;
  runPoseT(f, p, img, out, sh.shA, sh.shT, sh.shN, forced);
  return true;
}

// The final pass: each unique corner's P slots are summed (the CSR), rounded and clamped
// (positions) and renormalised (normals). outPos [unique][4], outNrm [unique][4]; flags[u] has
// bit 0 for a clamped position axis and bit 1 for a short normal (scalar fallback; its output is
// not written). Returns the number of clamped uniques.
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
