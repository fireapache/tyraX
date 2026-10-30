// Host harness: the editor's pose sampler (animmerge::poseMatrices /
// poseMatricesBlend) interpolates rotation keys on the SHORT ARC, like the
// engine's SkelInstance and glbparser's bake do. Build from the repo root:
//   gcc -O1 -c -Ivendor/ufbx -o ufbx.o vendor/ufbx/ufbx.c
//   g++ -std=c++20 -O1 -Isrc -Ivendor/ufbx -Ivendor/stb -o verify-anim-sampling
//       tools/verify-anim-sampling.cpp src/animmerge.cpp src/glbparser.cpp src/fbxparser.cpp
//       src/json.cpp src/meshlod.cpp ufbx.o
//   ./verify-anim-sampling [model.glb]   (a model: every clip swept, no joint may jump)
// q and -q are one rotation, and an exporter may flip a key's sign mid-clip
// (Blender does on the combatent's thighs): a plain component lerp between the
// two passes near zero and the joint spins for a frame - the 2D Animation
// Blend Tool's leg "glitch" (docs/blend-spaces.md).
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "../src/animmerge.hpp"

// the editor keeps stb's implementation in app.cpp; this harness links without it
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

namespace {

int g_fail = 0;

void check(bool ok, const char* name, const std::string& detail = "") {
    std::printf("%s %s%s%s\n", ok ? "PASS" : "FAIL", name, detail.empty() ? "" : " - ", detail.c_str());
    if (!ok) ++g_fail;
}

// A one-node skeleton whose one clip turns about X from 10 to 12 degrees -
// the second key stored with the OPPOSITE sign (the same rotation).
glbparser::Skel flippedKeys() {
    glbparser::Skel s;
    s.nodes.resize(1);
    glbparser::SkelClip c;
    c.name = "flip";
    c.duration = 1.0f / 24.0f;
    glbparser::SkelChannel ch;
    ch.node = 0;
    ch.path = 1;
    ch.times = {0.0f, c.duration};
    const float a0 = 10.0f * 0.0087266463f, a1 = 12.0f * 0.0087266463f;  // half angles
    ch.values = {std::sin(a0), 0.0f, 0.0f, std::cos(a0), -std::sin(a1), 0.0f, 0.0f, -std::cos(a1)};
    c.channels.push_back(ch);
    s.clips.push_back(c);
    return s;
}

// The rotation about X a node global holds (column-major, m[5] = cos, m[6] = sin).
float angleX(const float* m) { return std::atan2(m[6], m[5]) * 57.29578f; }

void opposite_sign_keys_take_the_short_arc() {
    const glbparser::Skel s = flippedKeys();
    std::vector<float> g;
    float worst = 0.0f;
    for (int i = 0; i <= 200; ++i) {
        const float f = i / 200.0f;
        animmerge::poseMatrices(s, 0, f * s.clips[0].duration, g);
        worst = std::fmax(worst, std::fabs(angleX(g.data()) - (10.0f + 2.0f * f)));
    }
    check(worst < 0.5f, "opposite_sign_keys_take_the_short_arc", "worst " + std::to_string(worst) + " deg off 10 -> 12");
}

void blend_of_one_clip_takes_the_short_arc() {
    const glbparser::Skel s = flippedKeys();
    std::vector<float> g;
    float worst = 0.0f;
    const int clip = 0;
    const float weight = 1.0f;
    for (int i = 0; i <= 200; ++i) {
        const float f = i / 200.0f, t = f * s.clips[0].duration;
        animmerge::poseMatricesBlend(s, &clip, &t, &weight, 1, g);
        worst = std::fmax(worst, std::fabs(angleX(g.data()) - (10.0f + 2.0f * f)));
    }
    check(worst < 0.5f, "blend_of_one_clip_takes_the_short_arc", "worst " + std::to_string(worst) + " deg off 10 -> 12");
}

// The angle between two node globals' rotations (columns normalised: scale out).
float rotationStep(const float* a, const float* b) {
    float tr = 0.0f;
    for (int c = 0; c < 3; ++c) {
        float la = 0.0f, lb = 0.0f, d = 0.0f;
        for (int r = 0; r < 3; ++r) {
            la += a[c * 4 + r] * a[c * 4 + r];
            lb += b[c * 4 + r] * b[c * 4 + r];
            d += a[c * 4 + r] * b[c * 4 + r];
        }
        tr += la > 1e-12f && lb > 1e-12f ? d / std::sqrt(la * lb) : 1.0f;
    }
    const float x = std::fmax(-1.0f, std::fmin(1.0f, (tr - 1.0f) * 0.5f));
    return std::acos(x) * 57.29578f;
}

// Every clip of a real model, swept at 600 steps a second: no joint turns more
// than 15 deg between two steps (a sprinting leg turns ~1 deg a step).
void model_clips_have_no_joint_jumps(const char* path) {
    glbparser::Skel s;
    std::string err;
    if (!glbparser::parseSkel(path, s, err)) {
        check(false, "model_clips_have_no_joint_jumps", "cannot load " + std::string(path) + ": " + err);
        return;
    }
    for (size_t ci = 0; ci < s.clips.size(); ++ci) {
        const glbparser::SkelClip& c = s.clips[ci];
        std::vector<float> prev, cur;
        animmerge::poseMatrices(s, (int)ci, 0.0f, prev);
        float worst = 0.0f, worstT = 0.0f;
        size_t worstNode = 0;
        const int steps = (int)std::ceil(c.duration * 600.0f);
        for (int i = 1; i <= steps; ++i) {
            const float t = c.duration * (float)i / (float)steps;
            animmerge::poseMatrices(s, (int)ci, t, cur);
            for (size_t n = 0; n < s.nodes.size(); ++n) {
                const float d = rotationStep(&prev[n * 16], &cur[n * 16]);
                if (d > worst) worst = d, worstT = t, worstNode = n;
            }
            prev.swap(cur);
        }
        char buf[160];
        std::snprintf(buf, sizeof(buf), "%s: worst %.1f deg a step (node %s at %.3f s)", c.name.c_str(), worst,
                      worstNode < s.nodes.size() ? s.nodes[worstNode].name.c_str() : "?", worstT);
        check(worst < 15.0f, "model_clips_have_no_joint_jumps", buf);
    }
}

}  // namespace

int main(int argc, char** argv) {
    opposite_sign_keys_take_the_short_arc();
    blend_of_one_clip_takes_the_short_arc();
    if (argc > 1) model_clips_have_no_joint_jumps(argv[1]);
    if (g_fail) {
        std::printf("%d FAILED\n", g_fail);
        return 1;
    }
    std::printf("ALL CHECKS PASSED\n");
    return 0;
}
