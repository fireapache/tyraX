// Host harness for the compact animation clips (docs/animated-models.md, ".tskl version 4"): the
// editor's writer (glbparser::writeTskl), the engine's parse (TsklLoader::parse) and the codec
// both share (loaders/3d/tskl_loader/skel_clip_codec.hpp). It checks, on a synthetic skeleton
// and - when a .glb is given - on every clip of the real model:
//   * every channel of every clip sampled at 240 Hz (past both ends, then backwards): a
//     translation or scale is within step / 2 + 1e-6 of the float sampler (the engine's sampler
//     before the compact clips, copied below), a rotation is EQUAL to it (bit for bit), a
//     one-key channel is the same value at every time and the grid point of its float value;
//   * the combatent's jog clip holds 1 time array, crouch / lean / death 2;
//   * a version 3 image's clips (floats) parse to the same compact clips as the version 4
//     image written from the same float clips, and so does the baked .tskl given as the second
//     argument (its float clips sampled against the converted ones);
//   * findKey agrees with a plain scan from any cursor; the tick rate is the smallest that holds
//     the keys; a clip no rate holds is refused or falls back to 1000 Hz as documented;
//   * a truncated or corrupt version 4 image is refused;
//   * the bytes of the compact pools against the float ones are printed.
// Build and run from the TyraX root, after one editor configure (it writes the tamtypes.h host
// shim to build/host-shim); tools/verify-tyrax2-host.py --only anim does all of it:
//   g++ -std=c++20 -O1 -Wall -Wextra -Isrc -Ibuild/host-shim -Ivendor/tyra/engine/inc
//       -Ivendor/stb -Ivendor/glfw/deps -Wl,--stack,16777216 -o build/verify-anim-compact
//       tools/verify-anim-compact.cpp src/glbparser.cpp src/meshlod.cpp src/json.cpp
//       vendor/tyra/engine/src/loaders/3d/tskl_loader/tskl_parse.cpp
//       vendor/tyra/engine/src/loaders/3d/tmdl_loader/tmdl_compact.cpp
//       vendor/tyra/engine/src/renderer/3d/pipeline/static/core/compact/stapip_compact.cpp
//   build/verify-anim-compact [model.glb [baked.tskl]]
//       (prints "verify-anim-compact: <n> checks, 0 failed")

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
#include "loaders/3d/tskl_loader/skel_clip_codec.hpp"
#include "loaders/3d/tskl_loader/tskl_loader.hpp"

namespace {

using Tyra::SkelClipCodec::FloatChannel;
using Tyra::SkelClipCodec::FloatClip;

int g_checks = 0;
int g_fails = 0;

void check(bool ok, const std::string& what) {
    ++g_checks;
    if (!ok) {
        ++g_fails;
        std::printf("FAIL: %s\n", what.c_str());
    }
}

// --- The float sampler: the engine's SkelInstance sampler before the compact clips ---

struct RefChannel {
    uint32_t node;
    uint8_t path, step;
    std::vector<float> times, vec;
    std::vector<int16_t> quat;
};
struct RefClip {
    std::string name;
    float duration;
    std::vector<RefChannel> channels;
};

uint32_t refFindKey(const std::vector<float>& times, float t, uint32_t* cursor) {
    const uint32_t n = (uint32_t)times.size();
    uint32_t hi = *cursor;
    if (hi > n) hi = n;
    while (hi > 0 && times[hi - 1] >= t) --hi;
    while (hi < n && times[hi] < t) ++hi;
    *cursor = hi;
    return hi;
}

void refSample(const RefChannel& ch, float t, uint32_t* cursor, float* out) {
    const uint32_t n = (uint32_t)ch.times.size();
    const uint32_t hi = refFindKey(ch.times, t, cursor);
    if (ch.path == 1) {
        float a[4], b[4];
        const uint32_t lo = hi == 0 ? 0 : hi - 1;
        const uint32_t hiC = hi >= n ? n - 1 : hi;
        for (int c = 0; c < 4; ++c) {
            a[c] = ch.quat[(size_t)lo * 4 + c] * (1.0F / 32767.0F);
            b[c] = ch.quat[(size_t)hiC * 4 + c] * (1.0F / 32767.0F);
        }
        if (hi == 0 || hi >= n || lo == hiC) {
            std::memcpy(out, hi == 0 ? a : b, 4 * sizeof(float));
            const float len = std::sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2] + out[3] * out[3]);
            if (len > 1e-6F)
                for (int i = 0; i < 4; ++i) out[i] /= len;
            return;
        }
        const float t0 = ch.times[lo], t1 = ch.times[hi];
        float f = t1 > t0 ? (t - t0) / (t1 - t0) : 0.0F;
        if (ch.step) f = 0.0F;
        Tyra::SkelClipCodec::slerp(a, b, f, out);
        return;
    }
    if (hi == 0) {
        std::memcpy(out, &ch.vec[0], 3 * sizeof(float));
        return;
    }
    if (hi >= n) {
        std::memcpy(out, &ch.vec[(size_t)(n - 1) * 3], 3 * sizeof(float));
        return;
    }
    const uint32_t lo = hi - 1;
    const float t0 = ch.times[lo], t1 = ch.times[hi];
    float f = t1 > t0 ? (t - t0) / (t1 - t0) : 0.0F;
    if (ch.step) f = 0.0F;
    const float* a = &ch.vec[(size_t)lo * 3];
    const float* b = &ch.vec[(size_t)hi * 3];
    for (int c = 0; c < 3; ++c) out[c] = a[c] + (b[c] - a[c]) * f;
}

int16_t quatToS16(float v) {
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    return (int16_t)std::lround(v * 32767.0f);
}

RefClip refOf(const glbparser::SkelClip& c) {
    RefClip r;
    r.name = c.name;
    r.duration = c.duration;
    for (const glbparser::SkelChannel& ch : c.channels) {
        RefChannel o;
        o.node = (uint32_t)ch.node;
        o.path = (uint8_t)ch.path;
        o.step = (uint8_t)ch.step;
        o.times = ch.times;
        if (ch.path == 1)
            for (float v : ch.values) o.quat.push_back(quatToS16(v));
        else
            o.vec = ch.values;
        r.channels.push_back(std::move(o));
    }
    return r;
}

FloatClip floatOf(const RefClip& r) {
    FloatClip f;
    f.name = r.name;
    f.duration = r.duration;
    for (const RefChannel& c : r.channels) {
        FloatChannel o;
        o.node = c.node;
        o.path = c.path;
        o.step = c.step;
        o.times = c.times;
        o.vec = c.vec;
        o.quat = c.quat;
        f.channels.push_back(std::move(o));
    }
    return f;
}

// --- byte helpers and the image layouts ---

void app(std::string& o, const void* p, size_t n) { o.append(reinterpret_cast<const char*>(p), n); }
void appU32(std::string& o, uint32_t v) { app(o, &v, 4); }
void appF32(std::string& o, float v) { app(o, &v, 4); }
uint32_t rdU32(const std::string& s, size_t at) {
    uint32_t v;
    std::memcpy(&v, s.data() + at, 4);
    return v;
}

// Where the clip section of an image starts (TSKL 3 and 4 share the header up to it).
size_t clipsStart(const std::string& img) {
    return 24 + 24 + 16 + (size_t)rdU32(img, 8) * 112 + (size_t)rdU32(img, 12) * 68;
}

struct ClipSpan {
    size_t begin = 0, channelTable = 0, end = 0;
    uint32_t channels = 0, ticks = 0, arrays = 0, values = 0;
};

// Walks a version 4 clip section; the spans of its clips, the section's end last.
std::vector<ClipSpan> walkV4(const std::string& img) {
    std::vector<ClipSpan> spans;
    size_t at = clipsStart(img);
    const uint32_t clips = rdU32(img, 20);
    for (uint32_t c = 0; c < clips; ++c) {
        ClipSpan s;
        s.begin = at;
        s.channels = rdU32(img, at + 36);
        s.ticks = rdU32(img, at + 40 + 4 + 32);
        s.arrays = rdU32(img, at + 40 + 4 + 32 + 4);
        s.values = rdU32(img, at + 40 + 4 + 32 + 8);
        size_t p = at + 40 + 4 + 32 + 12;
        p += ((size_t)s.ticks * 2 + 3) / 4 * 4;
        p += (size_t)s.arrays * 4;
        p += ((size_t)s.values * 2 + 3) / 4 * 4;
        s.channelTable = p;
        s.end = p + (size_t)s.channels * 12;
        spans.push_back(s);
        at = s.end;
    }
    return spans;
}

// The version 3 clip section: floats.
std::string v3ClipSection(const std::vector<RefClip>& clips) {
    std::string o;
    for (const RefClip& c : clips) {
        std::string name = c.name.substr(0, 32);
        name.resize(32, '\0');
        o += name;
        appF32(o, c.duration);
        appU32(o, (uint32_t)c.channels.size());
        for (const RefChannel& ch : c.channels) {
            appU32(o, ch.node);
            o.push_back((char)ch.path);
            o.push_back((char)ch.step);
            o.push_back(0);
            o.push_back(0);
            appU32(o, (uint32_t)ch.times.size());
            for (float t : ch.times) appF32(o, t);
            if (ch.path == 1)
                app(o, ch.quat.data(), ch.quat.size() * 2);
            else
                app(o, ch.vec.data(), ch.vec.size() * 4);
        }
    }
    return o;
}

// Reads the v3 clip section of a baked image into reference clips; false when it is not v3.
bool readV3Clips(const std::string& img, std::vector<RefClip>& out, size_t* end) {
    if (img.size() < 24 || std::memcmp(img.data(), "TSKL", 4) != 0 || rdU32(img, 4) != 3) return false;
    size_t at = clipsStart(img);
    const uint32_t clips = rdU32(img, 20);
    for (uint32_t c = 0; c < clips; ++c) {
        RefClip r;
        r.name.assign(img.data() + at, strnlen(img.data() + at, 32));
        std::memcpy(&r.duration, img.data() + at + 32, 4);
        const uint32_t chn = rdU32(img, at + 36);
        at += 40;
        for (uint32_t k = 0; k < chn; ++k) {
            RefChannel ch;
            ch.node = rdU32(img, at);
            ch.path = (uint8_t)img[at + 4];
            ch.step = (uint8_t)img[at + 5];
            const uint32_t keys = rdU32(img, at + 8);
            at += 12;
            ch.times.resize(keys);
            std::memcpy(ch.times.data(), img.data() + at, keys * 4);
            at += keys * 4;
            if (ch.path == 1) {
                ch.quat.resize((size_t)keys * 4);
                std::memcpy(ch.quat.data(), img.data() + at, (size_t)keys * 8);
                at += (size_t)keys * 8;
            } else {
                ch.vec.resize((size_t)keys * 3);
                std::memcpy(ch.vec.data(), img.data() + at, (size_t)keys * 12);
                at += (size_t)keys * 12;
            }
            r.channels.push_back(std::move(ch));
        }
        out.push_back(std::move(r));
    }
    *end = at;
    return true;
}

// A version 3 image with the clips of `clips`: the version 4 image's header and tail around them.
std::string makeV3(const std::string& v4, const std::vector<RefClip>& clips) {
    const std::vector<ClipSpan> spans = walkV4(v4);
    std::string o = v4.substr(0, clipsStart(v4));
    const uint32_t three = 3;
    std::memcpy(&o[4], &three, 4);
    o += v3ClipSection(clips);
    o += v4.substr(spans.empty() ? clipsStart(v4) : spans.back().end);
    return o;
}

// --- a synthetic skeleton ---

void identity(float* m) {
    for (int c = 0; c < 16; ++c) m[c] = 0.0f;
    m[0] = m[5] = m[10] = m[15] = 1.0f;
}

glbparser::Skel baseSkel() {
    glbparser::Skel s;
    s.nodes.resize(3);
    for (int i = 0; i < 3; ++i) {
        s.nodes[i].parent = i - 1;
        identity(s.nodes[i].matrix);
        s.nodes[i].t[1] = 0.5f;
    }
    s.palette.resize(1);
    s.palette[0].node = 0;
    identity(s.palette[0].ibm);
    glbparser::SkelPart p;
    p.material = "m";
    p.image = -1;
    p.vertexCount = 3;
    p.positions = {0, 0, 0, 1, 0, 0, 0, 1, 0};
    p.normals = {0, 0, 1, 0, 0, 1, 0, 0, 1};
    p.joints = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    p.weights = {255, 0, 0, 0, 255, 0, 0, 0, 255, 0, 0, 0};
    s.parts.push_back(p);
    for (int c = 0; c < 3; ++c) s.min[c] = 0, s.max[c] = 1;
    return s;
}

glbparser::SkelChannel chan(int node, int path, int step, std::vector<float> times, std::vector<float> values) {
    glbparser::SkelChannel c;
    c.node = node;
    c.path = path;
    c.step = step;
    c.times = std::move(times);
    c.values = std::move(values);
    return c;
}

std::vector<float> frames(int count, float fps, float first = 0.0f) {
    std::vector<float> t;
    for (int k = 0; k < count; ++k) t.push_back(first + (float)k / fps);
    return t;
}

std::vector<float> ramp(int keys, int per, float seed) {
    std::vector<float> v;
    for (int k = 0; k < keys; ++k)
        for (int c = 0; c < per; ++c) v.push_back(seed + 0.37f * std::sin(0.9f * k + c) + 0.01f * k);
    return v;
}

std::vector<float> rotKeys(int keys, float seed) {
    std::vector<float> v;
    for (int k = 0; k < keys; ++k) {
        const float a = seed + 0.35f * (float)k;
        const float q[4] = {std::sin(a) * 0.6f, std::cos(a * 1.3f) * 0.2f, std::sin(a * 0.7f) * 0.5f, 1.0f};
        const float len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        // every other key stored with the opposite sign: the same rotation, the short arc matters
        const float sgn = k % 2 ? -1.0f : 1.0f;
        for (float x : q) v.push_back(sgn * x / len);
    }
    return v;
}

glbparser::Skel syntheticSkel() {
    glbparser::Skel s = baseSkel();
    glbparser::SkelClip a;  // 24 Hz, one shared time array across 5 channels, one constant, one step
    a.name = "a24";
    a.duration = 12.0f / 24.0f;
    const std::vector<float> t = frames(13, 24.0f);
    a.channels.push_back(chan(0, 0, 0, t, ramp(13, 3, 0.2f)));
    a.channels.push_back(chan(1, 1, 0, t, rotKeys(13, 0.1f)));
    a.channels.push_back(chan(2, 1, 1, t, rotKeys(13, 0.8f)));  // STEP
    a.channels.push_back(chan(1, 2, 0, t, ramp(13, 3, 1.0f)));
    a.channels.push_back(chan(2, 0, 0, t, ramp(13, 3, -3.1f)));
    a.channels.push_back(chan(0, 2, 0, {0.0f}, {1.0f, 1.0f, 1.0f}));  // one key: a constant
    a.channels.push_back(chan(0, 1, 0, {0.25f}, {0.0f, 0.7071068f, 0.0f, 0.7071068f}));
    s.clips.push_back(a);
    glbparser::SkelClip b;  // 30 Hz, two time arrays (the second a subset of the frames)
    b.name = "b30";
    b.duration = 9.0f / 30.0f;
    b.channels.push_back(chan(0, 0, 0, frames(10, 30.0f), ramp(10, 3, 0.4f)));
    b.channels.push_back(chan(1, 1, 0, {0.0f, 4.0f / 30.0f, 9.0f / 30.0f}, rotKeys(3, 0.3f)));
    b.channels.push_back(chan(2, 1, 0, frames(10, 30.0f), rotKeys(10, 1.3f)));
    s.clips.push_back(b);
    glbparser::SkelClip c;  // 25 Hz
    c.name = "c25";
    c.duration = 1.0f;
    c.channels.push_back(chan(0, 1, 0, frames(26, 25.0f), rotKeys(26, 2.0f)));
    s.clips.push_back(c);
    glbparser::SkelClip d;  // keys that fit no rate within 1e-4 s: the 1000 Hz fallback
    d.name = "d_free";
    d.duration = 0.7f;
    d.channels.push_back(chan(0, 0, 0, {0.0f, 0.31417f, 0.70713f}, ramp(3, 3, 0.0f)));
    d.channels.push_back(chan(1, 1, 0, {0.0f, 0.31417f, 0.70713f}, rotKeys(3, 0.5f)));
    s.clips.push_back(d);
    glbparser::SkelClip e;  // a pose clip: every channel a constant, a translation far from the origin
    e.name = "e_pose";
    e.duration = 0.0f;
    e.channels.push_back(chan(0, 0, 0, {0.0f}, {123.456f, -0.001f, 7.0f}));
    e.channels.push_back(chan(1, 2, 0, {0.0f}, {1.0f, 2.0f, 0.5f}));
    s.clips.push_back(e);
    glbparser::SkelClip f;  // 60 Hz with a duplicated key time (a step edge)
    f.name = "f60";
    f.duration = 0.1f;
    f.channels.push_back(chan(0, 0, 1, {0.0f, 1.0f / 60.0f, 1.0f / 60.0f, 6.0f / 60.0f},
                              {0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2}));
    s.clips.push_back(f);
    return s;
}

// --- the sweep ---

struct SweepStats {
    double worstVec = 0.0;      // the worst translation / scale error in box steps
    uint64_t rotMismatch = 0;   // rotation samples that differ from the float sampler in any bit
    uint64_t constMismatch = 0;
    uint64_t samples = 0;
    uint64_t vecOverTol = 0;
};

// Samples every channel of `clip` against `ref` at 240 Hz, forwards from before the start to after
// the end and then backwards.
SweepStats sweep(const Tyra::SkelClip& clip, const RefClip& ref) {
    SweepStats st;
    if (clip.channels.size() != ref.channels.size()) {
        st.rotMismatch = 1ull << 40;
        return st;
    }
    const int n = (int)std::ceil((ref.duration + 0.2f) * 240.0f);
    for (size_t c = 0; c < ref.channels.size(); ++c) {
        const RefChannel& rc = ref.channels[c];
        const Tyra::SkelChannel& cc = clip.channels[c];
        const Tyra::Compact::Box& box = rc.path == 2 ? clip.sBox : clip.tBox;
        uint32_t rcur = 0, ccur = 0;
        std::vector<float> constFirst;
        for (int pass = 0; pass < 2; ++pass) {
            for (int i = 0; i <= n; ++i) {
                const int k = pass == 0 ? i : n - i;
                const float t = (float)(k - 12) / 240.0f;  // from 0.05 s before the start
                float want[4] = {}, got[4] = {};
                refSample(rc, t, &rcur, want);
                Tyra::SkelClipCodec::sample(clip, cc, t, &ccur, got);
                ++st.samples;
                if (rc.path == 1) {
                    if (std::memcmp(want, got, 16) != 0) ++st.rotMismatch;
                } else {
                    for (int a = 0; a < 3; ++a) {
                        const double err = std::fabs((double)want[a] - (double)got[a]);
                        if (err > box.step / 2.0 + 1e-6) ++st.vecOverTol;
                        if (err / box.step > st.worstVec) st.worstVec = err / box.step;
                    }
                }
                if (rc.times.size() == 1) {  // a constant: one value at every time
                    std::vector<float> v(got, got + (rc.path == 1 ? 4 : 3));
                    if (constFirst.empty())
                        constFirst = v;
                    else if (v != constFirst)
                        ++st.constMismatch;
                }
            }
        }
        if (rc.times.size() == 1 && rc.path != 1) {
            // and it is the grid point of the float value, exactly
            for (int a = 0; a < 3; ++a) {
                const int16_t q = Tyra::Compact::quantize(rc.vec[a], box.origin[a], box.step);
                if (constFirst.size() < 3 || constFirst[a] != Tyra::Compact::dequantize(q, box.origin[a], box.step))
                    ++st.constMismatch;
            }
        }
    }
    return st;
}

// findKey from any cursor agrees with a plain scan, on every channel.
void checkFindKey(const Tyra::SkelClip& clip, const RefClip& ref, const std::string& tag) {
    uint64_t bad = 0;
    const uint32_t cursors[] = {0, 1, 3, 7, 1000, 0xFFFFFFFFu};
    for (size_t c = 0; c < ref.channels.size(); ++c) {
        const std::vector<float>& times = ref.channels[c].times;
        for (int k = -6; k <= (int)std::ceil((ref.duration + 0.1f) * 240.0f); ++k) {
            const float t = (float)k / 240.0f;
            uint32_t want = 0;
            while (want < times.size() && times[want] < t) ++want;
            for (uint32_t cur : cursors) {
                const uint32_t got = Tyra::SkelClipCodec::findKey(clip, clip.channels[c], t, cur);
                // a one-key channel answers 0 whatever the time (it is never sampled by key)
                const uint32_t expect = times.size() == 1 ? 0 : want;
                if (got != expect) ++bad;
            }
        }
    }
    check(bad == 0, tag + ": findKey equals a plain scan from every cursor (" + std::to_string(bad) + " differ)");
}

void reportClip(const Tyra::SkelClip& clip, const RefClip& ref, const std::string& tag, bool verbose) {
    const SweepStats st = sweep(clip, ref);
    if (verbose)
        std::printf("  %-10s %2zu ch, %2zu time arrays, %.0f Hz, %llu samples, worst translation/scale %.3f steps\n",
                    ref.name.c_str(), ref.channels.size(), clip.timeArrays.size(), (double)clip.tickRate,
                    (unsigned long long)st.samples, st.worstVec);
    check(st.rotMismatch == 0, tag + " " + ref.name + ": rotations equal the float sampler (" +
                                   std::to_string(st.rotMismatch) + " samples differ)");
    check(st.vecOverTol == 0, tag + " " + ref.name + ": translations and scales within step / 2 + 1e-6 (" +
                                  std::to_string(st.vecOverTol) + " over)");
    check(st.constMismatch == 0, tag + " " + ref.name + ": one-key channels constant and on the grid (" +
                                     std::to_string(st.constMismatch) + " off)");
}

bool sameClip(const Tyra::SkelClip& a, const Tyra::SkelClip& b) {
    if (a.name != b.name || a.duration != b.duration || a.tickRate != b.tickRate ||
        std::memcmp(&a.tBox, &b.tBox, sizeof(a.tBox)) != 0 || std::memcmp(&a.sBox, &b.sBox, sizeof(a.sBox)) != 0 ||
        a.ticks != b.ticks || a.timeArrays != b.timeArrays || a.values != b.values ||
        a.channels.size() != b.channels.size())
        return false;
    for (size_t i = 0; i < a.channels.size(); ++i) {
        const Tyra::SkelChannel& x = a.channels[i];
        const Tyra::SkelChannel& y = b.channels[i];
        if (x.node != y.node || x.path != y.path || x.step != y.step || x.timeArray != y.timeArray ||
            x.valueAt != y.valueAt)
            return false;
    }
    return true;
}

struct Pools {
    size_t floatBytes = 0, compactBytes = 0;
};

Pools poolBytes(const std::vector<RefClip>& refs, const std::vector<Tyra::SkelClip>& clips) {
    Pools p;
    for (const RefClip& r : refs)
        for (const RefChannel& c : r.channels)
            p.floatBytes += c.times.size() * 4 + c.vec.size() * 4 + c.quat.size() * 2;
    for (const Tyra::SkelClip& c : clips)
        p.compactBytes += c.ticks.size() * 2 + c.timeArrays.size() * 4 + c.values.size() * 2;
    return p;
}

// Writes `skel`, parses it, and runs every check of the images; `refs` are the float clips.
bool checkSkel(const glbparser::Skel& skel, const std::string& tag, bool verbose) {
    std::vector<std::string> names;
    std::string error;
    std::vector<std::string> warnings;
    const std::string v4 = glbparser::writeTskl(skel, names, &error, &warnings);
    check(!v4.empty(), tag + ": writeTskl wrote an image (" + error + ")");
    if (v4.empty()) return false;
    check(rdU32(v4, 4) == 4, tag + ": the image is version 4");
    Tyra::SkelModel m;
    const bool ok = Tyra::TsklLoader::parse((const u8*)v4.data(), v4.size(), m);
    check(ok, tag + ": the version 4 image parses");
    if (!ok) return false;
    check(m.clips.size() == skel.clips.size(), tag + ": clip count");
    if (m.clips.size() != skel.clips.size()) return false;

    std::vector<RefClip> refs;
    for (const glbparser::SkelClip& c : skel.clips) refs.push_back(refOf(c));
    for (size_t i = 0; i < refs.size(); ++i) {
        // A clip no tick rate held within 0.1 ms is stored at 1000 Hz: its keys moved to the
        // millisecond (by at most half of one), and the float sampler is run on the moved keys.
        RefClip moved = refs[i];
        if (m.clips[i].tickRate == 1000.0f) {
            double worstMove = 0.0;
            for (RefChannel& ch : moved.channels)
                if (ch.times.size() > 1)
                    for (float& t : ch.times) {
                        const float snapped = (float)std::floor((double)t * 1000.0 + 0.5) / 1000.0f;
                        worstMove = std::fmax(worstMove, std::fabs((double)snapped - (double)t));
                        t = snapped;
                    }
            check(worstMove <= 5.0e-4 + 1e-9, tag + " " + refs[i].name + ": 1000 Hz keys move by at most half a millisecond");
        }
        reportClip(m.clips[i], moved, tag, verbose);
        checkFindKey(m.clips[i], moved, tag + " " + refs[i].name);
    }

    // the same clips through the packer directly equal the written ones (one rule, two callers)
    for (size_t i = 0; i < refs.size(); ++i) {
        Tyra::SkelClip packed;
        std::string e;
        const bool packOk = Tyra::SkelClipCodec::pack(floatOf(refs[i]), packed, &e);
        check(packOk && sameClip(packed, m.clips[i]), tag + " " + refs[i].name + ": pack equals the parsed clip");
    }

    // a version 3 image of the same float clips parses to the same compact clips
    const std::string v3 = makeV3(v4, refs);
    Tyra::SkelModel m3;
    const bool ok3 = Tyra::TsklLoader::parse((const u8*)v3.data(), v3.size(), m3);
    check(ok3, tag + ": the version 3 image parses");
    if (ok3) {
        bool same = m3.clips.size() == m.clips.size();
        for (size_t i = 0; same && i < m.clips.size(); ++i) same = sameClip(m3.clips[i], m.clips[i]);
        check(same, tag + ": a version 3 image's clips parse equal to the version 4 clips");
    }

    // a truncated image is refused: at every cut inside the clip section's header and tables
    {
        const std::vector<ClipSpan> spans = walkV4(v4);
        size_t refused = 0, tried = 0;
        for (const ClipSpan& s : spans) {
            const size_t cuts[] = {s.begin + 1, s.begin + 36, s.begin + 40, s.begin + 90,
                                   s.channelTable - 1, s.channelTable + 5, s.end - 1};
            for (size_t cut : cuts) {
                if (cut >= v4.size()) continue;
                Tyra::SkelModel t;
                ++tried;
                if (!Tyra::TsklLoader::parse((const u8*)v4.data(), cut, t)) ++refused;
            }
        }
        check(refused == tried, tag + ": truncated images refused (" + std::to_string(refused) + "/" +
                                    std::to_string(tried) + ")");
        // corrupt channel tables: a bad time array, a value offset beyond the values, a bad path,
        // a node beyond the skeleton, an array range beyond the ticks
        if (!spans.empty() && spans[0].channels > 0) {
            const ClipSpan& s = spans[0];
            auto corrupt = [&](size_t off, size_t len, uint32_t v) {
                std::string img = v4;
                std::memcpy(&img[off], &v, len);
                Tyra::SkelModel t;
                return !Tyra::TsklLoader::parse((const u8*)img.data(), img.size(), t);
            };
            check(corrupt(s.channelTable + 4 + 2, 2, 0x7000), tag + ": a time array index beyond the arrays is refused");
            check(corrupt(s.channelTable + 8, 4, 0x7FFFFFF0u), tag + ": a value offset beyond the values is refused");
            check(corrupt(s.channelTable + 4, 1, 3), tag + ": a path beyond 2 is refused");
            check(corrupt(s.channelTable, 4, 0xFFFFFF), tag + ": a node beyond the skeleton is refused");
            if (s.arrays > 0) {
                const size_t arraysAt = s.channelTable - ((size_t)s.values * 2 + 3) / 4 * 4 - (size_t)s.arrays * 4;
                check(corrupt(arraysAt, 4, (0xFFFFu << 16) | 5), tag + ": a time array beyond the ticks is refused");
                check(corrupt(arraysAt, 4, 1), tag + ": a time array of one key is refused");
            }
        }
    }

    const Pools p = poolBytes(refs, m.clips);
    std::printf("%s: clip pools, float %zu bytes, compact %zu bytes (%.1f%%), %zu clips\n", tag.c_str(),
                p.floatBytes, p.compactBytes, p.floatBytes ? 100.0 * p.compactBytes / p.floatBytes : 0.0,
                m.clips.size());
    for (const std::string& w : warnings) std::printf("  note: %s\n", w.c_str());
    return true;
}

void checkSyntheticRates() {
    glbparser::Skel s = syntheticSkel();
    std::vector<std::string> names;
    const std::string v4 = glbparser::writeTskl(s, names);
    Tyra::SkelModel m;
    if (!Tyra::TsklLoader::parse((const u8*)v4.data(), v4.size(), m) || m.clips.size() != 6) {
        check(false, "synthetic: parse for the rate checks");
        return;
    }
    check(m.clips[0].tickRate == 24.0f, "a 24 fps clip takes 24 Hz");
    check(m.clips[1].tickRate == 30.0f, "a 30 fps clip takes 30 Hz");
    check(m.clips[2].tickRate == 25.0f, "a 25 fps clip takes 25 Hz");
    check(m.clips[3].tickRate == 1000.0f, "keys no rate holds fall back to 1000 Hz");
    check(m.clips[5].tickRate == 60.0f, "a 60 fps clip takes 60 Hz");
    check(m.clips[0].timeArrays.size() == 1, "a24: the five multi-key channels share 1 time array");
    check(m.clips[1].timeArrays.size() == 2, "b30: 2 time arrays (10 frames, and the 3-key subset)");
    check(m.clips[4].timeArrays.empty() && m.clips[4].ticks.empty(), "e_pose: constants only, no times");
    check(m.clips[4].channels.size() == 2 && m.clips[4].channels[0].timeArray == Tyra::kSkelConstantChannel,
          "e_pose: constant channels carry the 0xFFFF array");
    check(m.clips[0].values.size() == 13 * 3 * 3 + 13 * 4 * 2 + 3 + 4,
          "a24: 13 keys of 3 x 4 vectors, 2 x 4 rotations, and the constants' one value each");
    // a rate-less clip says so
    std::string error;
    std::vector<std::string> warnings;
    glbparser::writeTskl(s, names, &error, &warnings);
    bool said = false;
    for (const std::string& w : warnings) said |= w.find("d_free") != std::string::npos;
    check(said, "the writer says which clip fell back to 1000 Hz");

    // a clip past 65.535 s at 1000 Hz cannot be held: the writer refuses with a reason
    glbparser::Skel big = baseSkel();
    glbparser::SkelClip c;
    c.name = "long";
    c.duration = 70.0f;
    c.channels.push_back(chan(0, 0, 0, {0.0f, 33.30517f, 70.0f}, ramp(3, 3, 0.0f)));
    big.clips.push_back(c);
    error.clear();
    const std::string none = glbparser::writeTskl(big, names, &error);
    check(none.empty() && error.find("long") != std::string::npos, "a clip beyond 65.535 s is refused with its name (" + error + ")");
    // ...unless its keys sit on a coarse grid that a u16 tick holds (70 s at 24 Hz is tick 1680)
    big.clips[0].channels[0].times = {0.0f, 35.0f, 70.0f};
    error.clear();
    check(!glbparser::writeTskl(big, names, &error).empty(), "a long clip on the 24 Hz grid is written");
}

}  // namespace

int main(int argc, char** argv) {
    std::printf("synthetic skeleton\n");
    checkSyntheticRates();
    checkSkel(syntheticSkel(), "synthetic", true);

    if (argc > 1) {
        glbparser::Skel skel;
        std::string error;
        if (!glbparser::parseSkel(argv[1], skel, error)) {
            check(false, "model parse: " + error);
        } else {
            std::printf("model %s: %zu clips\n", argv[1], skel.clips.size());
            checkSkel(skel, "model", true);
            // The jog clip holds one time array; crouch, lean and death hold two (the combatent).
            std::vector<std::string> names;
            for (size_t i = 0; i < skel.images.size(); ++i) names.push_back("tex" + std::to_string(i) + ".png");
            const std::string img = glbparser::writeTskl(skel, names);
            Tyra::SkelModel m;
            if (Tyra::TsklLoader::parse((const u8*)img.data(), img.size(), m)) {
                for (const Tyra::SkelClip& c : m.clips) {
                    int want = -1;
                    if (c.name == "jog") want = 1;
                    if (c.name == "crouch" || c.name == "lean_l" || c.name == "lean_r" || c.name == "death") want = 2;
                    if (want >= 0)
                        check((int)c.timeArrays.size() == want,
                              "model " + c.name + ": " + std::to_string(want) + " time array(s), has " +
                                  std::to_string(c.timeArrays.size()));
                }
            }
        }
    }

    if (argc > 2) {  // a baked version 3 file: its float clips against the converted ones
        FILE* f = std::fopen(argv[2], "rb");
        if (!f) {
            check(false, std::string("cannot open ") + argv[2]);
        } else {
            std::string img;
            char buf[65536];
            size_t n;
            while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) img.append(buf, n);
            std::fclose(f);
            std::vector<RefClip> refs;
            size_t end = 0;
            if (!readV3Clips(img, refs, &end)) {
                check(false, "the baked file is not a version 3 image (rebake or pass the .glb only)");
            } else {
                Tyra::SkelModel m;
                const bool ok = Tyra::TsklLoader::parse((const u8*)img.data(), img.size(), m);
                check(ok && m.clips.size() == refs.size(), "the baked version 3 file parses and converts");
                if (ok && m.clips.size() == refs.size()) {
                    std::printf("baked %s: %zu clips converted on load\n", argv[2], refs.size());
                    for (size_t i = 0; i < refs.size(); ++i) {
                        reportClip(m.clips[i], refs[i], "baked", true);
                        checkFindKey(m.clips[i], refs[i], "baked " + refs[i].name);
                    }
                    const Pools p = poolBytes(refs, m.clips);
                    std::printf("baked: clip pools, float %zu bytes, compact %zu bytes (%.1f%%)\n", p.floatBytes,
                                p.compactBytes, 100.0 * p.compactBytes / p.floatBytes);
                }
            }
        }
    }

    std::printf("verify-anim-compact: %d checks, %d failed\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
