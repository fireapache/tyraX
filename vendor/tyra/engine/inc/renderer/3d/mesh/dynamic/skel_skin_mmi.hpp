// The lane model of the EE MMI instructions the fixed-point skin uses (skel_skin.cpp,
// Fixed::cornerMmi). Plain C++, the same semantics as the EE (PCSX2's MMI.cpp is the
// reference): the host verifier runs the kernel over these, the EE kernel runs the same
// steps in inline asm. Only what the kernel needs is modelled.
//
// Layouts the kernel relies on:
//  - a 128-bit register holds 8 halfwords h[0..7]; PMADDH multiplies the eight pairs and
//    accumulates lane i into one of the four LO words and four HI words of the
//    accumulator: lane i goes to word ((i >> 2) << 1 | (i & 1)), LO when (i >> 1) & 1 is 0,
//    HI otherwise (MMI.cpp PMADDH);
//  - the kernel keeps its operands in the even lanes 0, 2, 4 (the odd lanes are zero), so
//    lane 2k holds the k-th value;
//  - a register of four words (W4) read as halfwords has the low half of word k at lane 2k,
//    so a W4 whose words fit int16 is the halfword operand of the next PMADDH as it is
//    (no pack instruction): asHalves() is that view;
//  - PMFHL.LW reads (LO0, HI0, LO2, HI2) back as words; PMTHL.LW writes them.
//
// Every value is exact integer arithmetic, wrapping at 32 bits like the EE's words.
#pragma once

#include <cstdint>

namespace Tyra {
namespace SkelSkin {
namespace Lanes {

struct H8 {
  int16_t h[8];
};

struct W4 {
  int32_t w[4];
};

// The accumulator: the LO and HI registers as four words each.
struct Acc {
  int32_t lo[4];
  int32_t hi[4];
};

inline W4 zeroW() { return W4{{0, 0, 0, 0}}; }

inline W4 splatW(int32_t v) { return W4{{v, v, v, v}}; }

// The eight halfwords with the three values in lanes 0, 2 and 4, the rest zero.
inline H8 evenLanes(int32_t v0, int32_t v1, int32_t v2) {
  H8 r = {{(int16_t)v0, 0, (int16_t)v1, 0, (int16_t)v2, 0, 0, 0}};
  return r;
}

// The halfword view of a word register: the low half of word k is lane 2k, the high half
// lane 2k + 1 (both signed).
inline H8 asHalves(const W4& v) {
  H8 r;
  for (int k = 0; k < 4; ++k) {
    r.h[2 * k] = (int16_t)(v.w[k] & 0xFFFF);
    r.h[2 * k + 1] = (int16_t)((uint32_t)v.w[k] >> 16);
  }
  return r;
}

// PMADDH: the eight signed 16 x 16 products, each added (32-bit wrap) to its word.
inline void pmaddh(Acc& a, const H8& rs, const H8& rt) {
  for (int i = 0; i < 8; ++i) {
    const int32_t p = (int32_t)rs.h[i] * (int32_t)rt.h[i];
    const int word = ((i >> 2) << 1) | (i & 1);
    if (((i >> 1) & 1) == 0)
      a.lo[word] = (int32_t)((uint32_t)a.lo[word] + (uint32_t)p);
    else
      a.hi[word] = (int32_t)((uint32_t)a.hi[word] + (uint32_t)p);
  }
}

// PMFHL.LW: (LO0, HI0, LO2, HI2).
inline W4 pmfhlLW(const Acc& a) { return W4{{a.lo[0], a.hi[0], a.lo[2], a.hi[2]}}; }

// PMTHL.LW: sets LO0, HI0, LO2, HI2 from the words (the odd words are not written; the
// kernel never sets an odd lane, so they stay zero).
inline void pmthlLW(Acc& a, const W4& v) {
  a.lo[0] = v.w[0];
  a.hi[0] = v.w[1];
  a.lo[2] = v.w[2];
  a.hi[2] = v.w[3];
}

// PADDW: word-wise add (wrapping).
inline W4 paddW(const W4& x, const W4& y) {
  W4 r;
  for (int k = 0; k < 4; ++k) r.w[k] = (int32_t)((uint32_t)x.w[k] + (uint32_t)y.w[k]);
  return r;
}

// PSRAVW: arithmetic shift of each word by sa (0..31).
inline W4 sraW(const W4& x, int sa) {
  W4 r;
  for (int k = 0; k < 4; ++k) r.w[k] = x.w[k] >> sa;
  return r;
}

// PSLLW: logical shift left of each word by sa (0..31).
inline W4 sllW(const W4& x, int sa) {
  W4 r;
  for (int k = 0; k < 4; ++k) r.w[k] = (int32_t)((uint32_t)x.w[k] << sa);
  return r;
}

}  // namespace Lanes
}  // namespace SkelSkin
}  // namespace Tyra
