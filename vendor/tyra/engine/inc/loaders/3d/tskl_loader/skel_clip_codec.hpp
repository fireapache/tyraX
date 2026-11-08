/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Added by TyraX: the compact animation clip codec (.tskl 4). Header-only
# and free of any PS2 header, so the editor's writer, the loader's conversion
# of older files, the host verifier and SkelInstance on the console all run
# the same code (docs/animated-models.md, ".tskl version 4").
*/

#pragma once

#include <math.h>
#include <string.h>

#include <string>
#include <vector>

#include "tskl_loader.hpp"
#include "../../../renderer/3d/pipeline/static/core/compact/stapip_compact.hpp"

namespace Tyra {
namespace SkelClipCodec {

/** The tick rates a clip can use, tried in this order. */
constexpr float kTickRates[] = {24.0F, 25.0F, 30.0F, 48.0F,
                                50.0F, 60.0F, 120.0F, 1000.0F};
constexpr int kTickRateCount = 8;
/** A key lands within this many seconds of its tick, at the smallest rate that allows it. */
constexpr double kTickTolerance = 1e-4;
/** The largest tick a u16 holds. */
constexpr u32 kMaxTick = 65535;

// ---------------------------------------------------------------------------
// Sampling (SkelInstance's only decode site for clip data)
// ---------------------------------------------------------------------------

/** The seconds a tick stands for. A division, not a multiplication by the
 * reciprocal: a key authored as frame / fps comes back as the very float it was. */
inline float tickSeconds(const SkelClip& clip, u16 tick) {
  return (float)tick / clip.tickRate;
}

/** The keys a channel has: its time array's count, or 1. */
inline u32 keyCount(const SkelClip& clip, const SkelChannel& ch) {
  return ch.timeArray == kSkelConstantChannel
             ? 1U
             : (clip.timeArrays[ch.timeArray] & 0xFFFFU);
}

/** The int16 values a key holds: 4 for a rotation, 3 for a quantized translation or scale, 6
 * (the bits of three floats) for an exact constant. */
inline u32 valuesPerKey(const SkelChannel& ch) {
  return ch.path == 1 ? 4U : ((ch.step & kSkelFloatBit) ? 6U : 3U);
}

/** A translation or scale key value out of its box. */
inline float dequantValue(const Compact::Box& box, int axis, s16 q) {
  return box.origin[axis] + box.step * (float)q;
}

/** Spherical interpolation of two unit quaternions along the short arc. */
inline void slerp(const float* a, const float* bIn, float t, float* out) {
  float b[4] = {bIn[0], bIn[1], bIn[2], bIn[3]};
  float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  if (dot < 0.0F) {  // take the short arc
    dot = -dot;
    for (int i = 0; i < 4; ++i) b[i] = -b[i];
  }
  float wa, wb;
  if (dot > 0.9995F) {  // nearly parallel - lerp avoids a degenerate sin
    wa = 1.0F - t;
    wb = t;
  } else {
    const float theta = acosf(dot > 1.0F ? 1.0F : dot);
    const float sinTheta = sinf(theta);
    wa = sinf((1.0F - t) * theta) / sinTheta;
    wb = sinf(t * theta) / sinTheta;
  }
  for (int i = 0; i < 4; ++i) out[i] = wa * a[i] + wb * b[i];
  const float len = sqrtf(out[0] * out[0] + out[1] * out[1] +
                          out[2] * out[2] + out[3] * out[3]);
  if (len > 1e-6F)
    for (int i = 0; i < 4; ++i) out[i] /= len;
  else
    out[3] = 1.0F;
}

namespace detail {

/** The first key of `ticks[0..n)` at or after `seconds`, scanning from `cursor`. */
inline u32 locate(const SkelClip& clip, const u16* ticks, u32 n, float seconds,
                  u32 cursor) {
  u32 hi = cursor;
  if (hi > n) hi = n;
  while (hi > 0 && tickSeconds(clip, ticks[hi - 1]) >= seconds) --hi;  // rewind
  while (hi < n && tickSeconds(clip, ticks[hi]) < seconds) ++hi;
  return hi;
}

}  // namespace detail

/**
 * The first key of a channel whose time is at or after `seconds`: 0 before the first key, the
 * key count after the last. `cursor` is where the last call ended - playback moves forward, so
 * the scan is O(1) - and any value is safe (a restart rewinds). A one-key channel answers 0.
 */
inline u32 findKey(const SkelClip& clip, const SkelChannel& ch, float seconds,
                   u32 cursor) {
  if (ch.timeArray == kSkelConstantChannel) return 0;
  const u32 range = clip.timeArrays[ch.timeArray];
  return detail::locate(clip, &clip.ticks[range >> 16], range & 0xFFFFU, seconds,
                        cursor);
}

/**
 * Samples one channel at `seconds` into out[3|4] - the rules the float sampler had: clamp
 * outside the key range, STEP holds the left key, rotations slerp. `*cursor` is the channel's
 * scan position (findKey). The rotation keys are the int16 they always were and the key times
 * come back as the floats they were written from, so a rotation is the float sampler's, bit
 * for bit; a translation or scale that never changes is the authored floats; one that does is
 * off by at most half its box's step.
 */
inline void sample(const SkelClip& clip, const SkelChannel& ch, float seconds,
                   u32* cursor, float* out) {
  const bool rot = ch.path == 1;
  const Compact::Box& box = ch.path == 2 ? clip.sBox : clip.tBox;
  const s16* v = &clip.values[ch.valueAt];

  if (ch.timeArray == kSkelConstantChannel) {  // one key: that value at every time
    if (ch.step & kSkelFloatBit) {  // exact: the authored floats
      memcpy(out, v, 3 * sizeof(float));
      return;
    }
    if (rot) {
      for (int c = 0; c < 4; ++c) out[c] = v[c] * (1.0F / 32767.0F);
      const float len = sqrtf(out[0] * out[0] + out[1] * out[1] +
                              out[2] * out[2] + out[3] * out[3]);
      if (len > 1e-6F)
        for (int i = 0; i < 4; ++i) out[i] /= len;
    } else {
      for (int c = 0; c < 3; ++c) out[c] = dequantValue(box, c, v[c]);
    }
    return;
  }

  const u32 range = clip.timeArrays[ch.timeArray];
  const u16* ticks = &clip.ticks[range >> 16];
  const u32 n = range & 0xFFFFU;
  const u32 hi = detail::locate(clip, ticks, n, seconds, *cursor);
  *cursor = hi;

  if (rot) {
    float a[4], b[4];
    const u32 lo = hi == 0 ? 0 : hi - 1;
    const u32 hiC = hi >= n ? n - 1 : hi;
    for (int c = 0; c < 4; ++c) {
      a[c] = v[(size_t)lo * 4 + c] * (1.0F / 32767.0F);
      b[c] = v[(size_t)hiC * 4 + c] * (1.0F / 32767.0F);
    }
    if (hi == 0 || hi >= n || lo == hiC) {
      memcpy(out, hi == 0 ? a : b, 4 * sizeof(float));
      // still normalize the dequantized endpoint
      const float len = sqrtf(out[0] * out[0] + out[1] * out[1] +
                              out[2] * out[2] + out[3] * out[3]);
      if (len > 1e-6F)
        for (int i = 0; i < 4; ++i) out[i] /= len;
      return;
    }
    const float t0 = tickSeconds(clip, ticks[lo]);
    const float t1 = tickSeconds(clip, ticks[hi]);
    float f = t1 > t0 ? (seconds - t0) / (t1 - t0) : 0.0F;
    if (ch.step & kSkelStepBit) f = 0.0F;
    slerp(a, b, f, out);
    return;
  }

  if (ch.step & kSkelFloatBit) {  // float keys: the authored values
    const u32 k0 = hi == 0 ? 0 : (hi >= n ? n - 1 : hi - 1);
    const u32 k1 = hi == 0 ? 0 : (hi >= n ? n - 1 : hi);
    float fa[3], fb[3];
    memcpy(fa, v + (size_t)k0 * 6, 12);
    memcpy(fb, v + (size_t)k1 * 6, 12);
    if (hi == 0 || hi >= n) {
      memcpy(out, fa, 12);
      return;
    }
    const float t0 = tickSeconds(clip, ticks[k0]);
    const float t1 = tickSeconds(clip, ticks[k1]);
    float f = t1 > t0 ? (seconds - t0) / (t1 - t0) : 0.0F;
    if (ch.step & kSkelStepBit) f = 0.0F;
    for (int c = 0; c < 3; ++c) out[c] = fa[c] + (fb[c] - fa[c]) * f;
    return;
  }
  if (hi == 0) {
    for (int c = 0; c < 3; ++c) out[c] = dequantValue(box, c, v[c]);
    return;
  }
  if (hi >= n) {
    for (int c = 0; c < 3; ++c)
      out[c] = dequantValue(box, c, v[(size_t)(n - 1) * 3 + c]);
    return;
  }
  const u32 lo = hi - 1;
  const float t0 = tickSeconds(clip, ticks[lo]);
  const float t1 = tickSeconds(clip, ticks[hi]);
  float f = t1 > t0 ? (seconds - t0) / (t1 - t0) : 0.0F;
  if (ch.step & kSkelStepBit) f = 0.0F;
  for (int c = 0; c < 3; ++c) {
    const float a = dequantValue(box, c, v[(size_t)lo * 3 + c]);
    const float b = dequantValue(box, c, v[(size_t)hi * 3 + c]);
    out[c] = a + (b - a) * f;
  }
}

// ---------------------------------------------------------------------------
// Packing (the editor's writer, and the loader's conversion of .tskl 1-3)
// ---------------------------------------------------------------------------

/** A channel as the float formats hold it (.tskl 1-3, the editor's clips). */
struct FloatChannel {
  u32 node = 0;
  u8 path = 0;
  u8 step = 0;
  std::vector<float> times;  // seconds from clip start, ascending
  std::vector<float> vec;    // path 0 / 2: keyCount * 3
  std::vector<s16> quat;     // path 1: keyCount * 4, component * 32767
};

struct FloatClip {
  std::string name;
  float duration = 0.0F;
  std::vector<FloatChannel> channels;
};

/** What pack found worth telling: the largest distance of a key from its tick. */
struct PackReport {
  float tickRate = 24.0F;
  double worstTimeError = 0.0;  // seconds
  bool withinTolerance = true;  // false: no rate held every key within kTickTolerance
};

namespace detail {

inline bool finite(float v) { return v - v == 0.0F; }

/** Every key of a translation or scale channel is the same three floats, bit for bit. */
inline bool allKeysEqual(const FloatChannel& ch) {
  for (size_t k = 1; k < ch.times.size(); ++k)
    if (memcmp(&ch.vec[k * 3], &ch.vec[0], 3 * sizeof(float)) != 0) return false;
  return true;
}

/** A channel with key times: more than one key, and not a translation or scale that never changes. */
inline bool hasTimes(const FloatChannel& ch) {
  return ch.times.size() >= 2 && !(ch.path != 1 && allKeysEqual(ch));
}

struct Bounds {
  float mn[3] = {0.0F, 0.0F, 0.0F};
  float mx[3] = {0.0F, 0.0F, 0.0F};
  bool any = false;
  void add(const float* v) {
    for (int c = 0; c < 3; ++c) {
      if (!any || v[c] < mn[c]) mn[c] = v[c];
      if (!any || v[c] > mx[c]) mx[c] = v[c];
    }
    any = true;
  }
};

/** The key times of a multi-key channel as ticks of `rate`; false when one is outside u16. */
inline bool ticksAt(const FloatChannel& ch, float rate, std::vector<u16>& out,
                    double* worst) {
  out.resize(ch.times.size());
  for (size_t k = 0; k < ch.times.size(); ++k) {
    const double t = ch.times[k];
    const double tick = floor(t * (double)rate + 0.5);
    if (!(tick >= 0.0 && tick <= (double)kMaxTick)) return false;
    const double err = fabs(t - tick / (double)rate);
    if (err > *worst) *worst = err;
    out[k] = (u16)tick;
  }
  return true;
}

}  // namespace detail

/**
 * The compact clip of a float clip. The tick rate is the smallest of kTickRates that puts every
 * key (of the channels that have more than one) within kTickTolerance of a tick; when none does,
 * 1000 Hz, the finest, is taken and the report says so (a key then moves by at most half a
 * millisecond). Channels whose key ticks are equal share one time array, in the order the first
 * channel of each appears. A translation or scale whose keys are all equal is one exact
 * constant (three floats, no times). The other translations and scales keep their floats too
 * (6 int16 words a key holding the bits), so a pose is the float sampler's bit for bit; with
 * `quantize` they are int16 in a box over all of the clip's changing translation values and in
 * one over its changing scale values (Compact::modelBox: one step, a power of two, an origin an
 * axis), within half a step of the authored values and half the size. Rotations are the int16
 * they were. @return false, with a reason in `*error`, for a clip that cannot be held: a
 * malformed channel, a non-finite value, a key beyond 65.535 s at 1 000 Hz, more time-array
 * ticks than a u16 addresses.
 */
inline bool pack(const FloatClip& in, SkelClip& out, std::string* error = nullptr,
                 PackReport* report = nullptr, bool quantize = false) {
  auto fail = [&](const std::string& why) {
    if (error) *error = "clip \"" + in.name + "\": " + why;
    return false;
  };
  out = SkelClip();
  out.name = in.name;
  out.duration = in.duration;

  detail::Bounds tb, sb;
  for (const FloatChannel& ch : in.channels) {
    const size_t keys = ch.times.size();
    if (keys == 0 || ch.path > 2) return fail("a channel has no keys or a bad path");
    if (ch.path == 1 ? ch.quat.size() != keys * 4 : ch.vec.size() != keys * 3)
      return fail("a channel's values do not match its keys");
    for (size_t k = 0; k < keys; ++k)
      if (!detail::finite(ch.times[k])) return fail("a key time is not finite");
    if (ch.path != 1) {
      for (float v : ch.vec)
        if (!detail::finite(v)) return fail("a translation or scale value is not finite");
      if (quantize && !detail::allKeysEqual(ch))  // a constant is stored exact: it needs no box
        for (size_t k = 0; k < keys; ++k)
          (ch.path == 0 ? tb : sb).add(&ch.vec[k * 3]);
    }
  }
  if (in.channels.size() >= kSkelConstantChannel) return fail("too many channels");

  // The tick rate: the smallest that holds every multi-key channel's keys.
  std::vector<u16> ticks;
  PackReport rep;
  bool chosen = false;
  for (int r = 0; r < kTickRateCount && !chosen; ++r) {
    double worst = 0.0;
    bool ok = true;
    for (const FloatChannel& ch : in.channels) {
      if (!detail::hasTimes(ch)) continue;
      if (!detail::ticksAt(ch, kTickRates[r], ticks, &worst)) {
        ok = false;
        break;
      }
    }
    if (ok && worst <= kTickTolerance) {
      rep.tickRate = kTickRates[r];
      rep.worstTimeError = worst;
      chosen = true;
    }
  }
  if (!chosen) {  // none within tolerance: the finest rate, if the keys fit it at all
    double worst = 0.0;
    bool ok = true;
    for (const FloatChannel& ch : in.channels) {
      if (!detail::hasTimes(ch)) continue;
      if (!detail::ticksAt(ch, kTickRates[kTickRateCount - 1], ticks, &worst)) {
        ok = false;
        break;
      }
    }
    if (!ok) return fail("a key time is negative or beyond 65.535 s at 1000 Hz");
    rep.tickRate = kTickRates[kTickRateCount - 1];
    rep.worstTimeError = worst;
    rep.withinTolerance = false;
  }
  out.tickRate = rep.tickRate;

  // Time arrays: equal tick lists share one.
  std::vector<std::vector<u16>> arrays;
  std::vector<u16> channelArray(in.channels.size(), kSkelConstantChannel);
  for (size_t c = 0; c < in.channels.size(); ++c) {
    const FloatChannel& ch = in.channels[c];
    if (!detail::hasTimes(ch)) continue;
    double unused = 0.0;
    detail::ticksAt(ch, rep.tickRate, ticks, &unused);
    size_t found = arrays.size();
    for (size_t a = 0; a < arrays.size(); ++a)
      if (arrays[a] == ticks) {
        found = a;
        break;
      }
    if (found == arrays.size()) {
      if (ticks.size() > 0xFFFFU || arrays.size() + 1 >= kSkelConstantChannel)
        return fail("a time array is too long or there are too many");
      arrays.push_back(ticks);
    }
    channelArray[c] = (u16)found;
  }
  for (const std::vector<u16>& a : arrays) {
    if (out.ticks.size() > 0xFFFFU)
      return fail("more time-array ticks than a u16 addresses");
    out.timeArrays.push_back(((u32)out.ticks.size() << 16) | (u32)a.size());
    out.ticks.insert(out.ticks.end(), a.begin(), a.end());
  }

  // The value boxes, then the values.
  out.tBox = Compact::modelBox(tb.mn, tb.mx);
  out.sBox = Compact::modelBox(sb.mn, sb.mx);
  for (size_t c = 0; c < in.channels.size(); ++c) {
    const FloatChannel& ch = in.channels[c];
    SkelChannel sc;
    sc.node = ch.node;
    sc.path = ch.path;
    sc.step = ch.step ? kSkelStepBit : 0;
    sc.timeArray = channelArray[c];
    sc.valueAt = (u32)out.values.size();
    const bool exactConstant = ch.path != 1 && detail::allKeysEqual(ch);
    const bool floatKeys = ch.path != 1 && !quantize && !exactConstant && ch.times.size() >= 2;
    const size_t keys = ch.times.size() < 2 || exactConstant ? 1 : ch.times.size();
    if (floatKeys) {
      sc.step |= kSkelFloatBit;
      for (size_t i = 0; i < keys * 3; ++i) {
        s16 bits[2];
        memcpy(bits, &ch.vec[i], 4);
        out.values.push_back(bits[0]);
        out.values.push_back(bits[1]);
      }
    } else if (exactConstant) {  // three floats, as they are
      sc.step |= kSkelFloatBit;
      sc.timeArray = kSkelConstantChannel;
      s16 bits[6];
      memcpy(bits, ch.vec.data(), sizeof(bits));
      out.values.insert(out.values.end(), bits, bits + 6);
    } else if (ch.path == 1) {
      out.values.insert(out.values.end(), ch.quat.begin(),
                        ch.quat.begin() + (ptrdiff_t)(keys * 4));
    } else {
      const Compact::Box& box = ch.path == 0 ? out.tBox : out.sBox;
      for (size_t i = 0; i < keys * 3; ++i)
        out.values.push_back(Compact::quantize(ch.vec[i], box.origin[i % 3], box.step));
    }
    out.channels.push_back(sc);
  }
  if (report) *report = rep;
  return true;
}

}  // namespace SkelClipCodec
}  // namespace Tyra
