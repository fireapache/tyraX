// Writes the small skinned model the host verifiers of the skinning (verify-skel-*, run by
// tools/verify-tyrax2-host.py) use when no model is passed: a 9-bone upright figure of three
// parts (a body tube, two arms, a head) with generated distance levels, vertices bound to one
// to four bones, texture coordinates and normals. No game art and no external file: the figure
// is built from its parameters, deterministically, and written through the editor's own .tskl
// writer, so the verifiers read exactly what a baked model is.
//
// Build and run from the TyraX root (verify-tyrax2-host.py does both):
//   g++ -std=c++20 -O1 -Isrc -Ibuild/host-shim -Ivendor/tyra/engine/inc -Ivendor/stb
//       -Ivendor/glfw/deps -Wl,--stack,16777216 -o build/make-skin-fixture
//       tools/make-skin-fixture.cpp src/glbparser.cpp src/meshlod.cpp src/json.cpp
//       vendor/tyra/engine/src/loaders/3d/tskl_loader/tskl_parse.cpp
//       vendor/tyra/engine/src/loaders/3d/tmdl_loader/tmdl_compact.cpp
//       vendor/tyra/engine/src/renderer/3d/pipeline/static/core/compact/stapip_compact.cpp
//   build/make-skin-fixture <out.tskl>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "glbparser.hpp"

namespace {

struct Joint {
    int parent;
    float x, y, z;  // world position in the bind pose
};

// The skeleton: root, hips, spine, chest, head, then a shoulder and an elbow a side.
const Joint kJoints[] = {
    {-1, 0.00f, 0.00f, 0.0f}, {0, 0.00f, 0.90f, 0.0f}, {1, 0.00f, 1.15f, 0.0f},
    {2, 0.00f, 1.40f, 0.0f},  {3, 0.00f, 1.65f, 0.0f}, {3, 0.25f, 1.50f, 0.0f},
    {5, 0.55f, 1.50f, 0.0f},  {3, -0.25f, 1.50f, 0.0f}, {7, -0.55f, 1.50f, 0.0f},
};
constexpr int kJointCount = 9;
const char* kJointNames[kJointCount] = {"Root",    "Hips",    "Spine",   "Chest", "Head",
                                        "L_Shoulder", "L_Elbow", "R_Shoulder", "R_Elbow"};

// A vertex of a tube before it is expanded to triangles.
struct Ring {
    float pos[3];
    float nrm[3];
    float uv[2];
    unsigned char joints[4];
    unsigned char weights[4];
};

// Bone weights from per-bone raw scores: the best four, scaled to sum exactly to 255.
void bind(Ring& v, const float (&score)[kJointCount]) {
    int order[kJointCount];
    for (int i = 0; i < kJointCount; ++i) order[i] = i;
    std::sort(order, order + kJointCount, [&](int a, int b) { return score[a] > score[b]; });
    float sum = 0.0f;
    for (int k = 0; k < 4; ++k) sum += score[order[k]];
    int used = 0;
    int given = 0;
    for (int k = 0; k < 4; ++k) {
        const float w = sum > 0.0f ? score[order[k]] / sum : 0.0f;
        int q = (int)std::floor(w * 255.0f + 0.5f);
        if (w < 0.02f) q = 0;
        v.joints[k] = (unsigned char)(q > 0 ? order[k] : 0);
        v.weights[k] = (unsigned char)q;
        given += q;
        if (q > 0) used = k + 1;
    }
    // Rounding: the largest weight takes the difference so the four sum to 255.
    v.weights[0] = (unsigned char)(v.weights[0] + (255 - given));
    (void)used;
}

float falloff(float distance, float reach) {
    const float t = 1.0f - std::fabs(distance) / reach;
    return t > 0.0f ? t * t : 0.0f;
}

// A tube along an axis from `a` to `b` (world positions), `rings` rings of `segments` points,
// radius `radius(t)`, expanded to a flat triangle list. `scoreOf(point)` gives the bone scores.
template <class Radius, class Score>
void tube(glbparser::SkelPart& part, const float a[3], const float b[3], int rings, int segments,
          Radius radius, Score scoreOf) {
    const float axis[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    const float length = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    const float d[3] = {axis[0] / length, axis[1] / length, axis[2] / length};
    // Two vectors orthogonal to the axis.
    float u[3] = {1.0f, 0.0f, 0.0f};
    if (std::fabs(d[0]) > 0.9f) {
        u[0] = 0.0f;
        u[1] = 0.0f;
        u[2] = 1.0f;
    }
    float w[3] = {d[1] * u[2] - d[2] * u[1], d[2] * u[0] - d[0] * u[2], d[0] * u[1] - d[1] * u[0]};
    float uu[3] = {w[1] * d[2] - w[2] * d[1], w[2] * d[0] - w[0] * d[2], w[0] * d[1] - w[1] * d[0]};
    std::vector<Ring> grid((size_t)rings * (size_t)segments);
    for (int i = 0; i < rings; ++i) {
        const float t = (float)i / (float)(rings - 1);
        const float r = radius(t);
        for (int j = 0; j < segments; ++j) {
            const float angle = 6.2831853f * (float)j / (float)segments;
            const float c = std::cos(angle), s = std::sin(angle);
            Ring& v = grid[(size_t)i * segments + j];
            for (int k = 0; k < 3; ++k) {
                const float radial = c * uu[k] + s * w[k];
                v.nrm[k] = radial;
                v.pos[k] = a[k] + axis[k] * t + radial * r;
            }
            v.uv[0] = 2.0f * (float)j / (float)segments;
            v.uv[1] = 2.0f * t;
            float score[kJointCount] = {};
            scoreOf(v.pos, t, score);
            bind(v, score);
        }
    }
    auto emit = [&](const Ring& v) {
        part.positions.insert(part.positions.end(), v.pos, v.pos + 3);
        part.normals.insert(part.normals.end(), v.nrm, v.nrm + 3);
        part.uvs.insert(part.uvs.end(), v.uv, v.uv + 2);
        part.joints.insert(part.joints.end(), v.joints, v.joints + 4);
        part.weights.insert(part.weights.end(), v.weights, v.weights + 4);
        part.vertexCount += 1;
    };
    for (int i = 0; i + 1 < rings; ++i)
        for (int j = 0; j < segments; ++j) {
            const int j1 = (j + 1) % segments;
            const Ring& p00 = grid[(size_t)i * segments + j];
            const Ring& p01 = grid[(size_t)i * segments + j1];
            const Ring& p10 = grid[(size_t)(i + 1) * segments + j];
            const Ring& p11 = grid[(size_t)(i + 1) * segments + j1];
            emit(p00);
            emit(p10);
            emit(p11);
            emit(p00);
            emit(p11);
            emit(p01);
        }
}

glbparser::SkelPart makePart(const char* material, float r, float g, float b) {
    glbparser::SkelPart p;
    p.material = material;
    p.image = 0;  // textured: the indexed stream is the textured lit one
    p.baseColor[0] = r;
    p.baseColor[1] = g;
    p.baseColor[2] = b;
    p.baseColor[3] = 1.0f;
    return p;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("usage: make-skin-fixture <out.tskl>\n");
        return 2;
    }
    glbparser::Skel skel;
    skel.nodes.resize(kJointCount);
    skel.palette.resize(kJointCount);
    for (int i = 0; i < kJointCount; ++i) {
        glbparser::SkelNode& n = skel.nodes[i];
        n.parent = kJoints[i].parent;
        n.name = kJointNames[i];
        const Joint* par = n.parent >= 0 ? &kJoints[n.parent] : nullptr;
        n.t[0] = kJoints[i].x - (par ? par->x : 0.0f);
        n.t[1] = kJoints[i].y - (par ? par->y : 0.0f);
        n.t[2] = kJoints[i].z - (par ? par->z : 0.0f);
        for (int c = 0; c < 16; ++c) skel.palette[i].ibm[c] = 0.0f;
        skel.palette[i].node = i;
        skel.palette[i].ibm[0] = skel.palette[i].ibm[5] = skel.palette[i].ibm[10] =
            skel.palette[i].ibm[15] = 1.0f;
        skel.palette[i].ibm[12] = -kJoints[i].x;  // the inverse of the bind translation
        skel.palette[i].ibm[13] = -kJoints[i].y;
        skel.palette[i].ibm[14] = -kJoints[i].z;
    }

    // The body: a tube from the feet to the shoulders, bound along the spine; the upper rings
    // also feel the arms' shoulder bones, so the figure has one to four influences a vertex.
    glbparser::SkelPart body = makePart("fixture_body", 0.8f, 0.7f, 0.6f);
    {
        const float a[3] = {0.0f, 0.0f, 0.0f}, b[3] = {0.0f, 1.55f, 0.0f};
        tube(body, a, b, 16, 14,
             [](float t) { return 0.20f + 0.08f * std::sin(t * 3.1415926f); },
             [](const float* p, float, float (&score)[kJointCount]) {
                 const float y = p[1];
                 score[0] = falloff(y - 0.00f, 0.70f);
                 score[1] = falloff(y - kJoints[1].y, 0.45f);
                 score[2] = falloff(y - kJoints[2].y, 0.35f);
                 score[3] = falloff(y - kJoints[3].y, 0.35f);
                 if (y > 1.30f) {
                     score[5] = 0.25f * falloff(p[0] - kJoints[5].x, 0.6f);
                     score[7] = 0.25f * falloff(p[0] - kJoints[7].x, 0.6f);
                 }
             });
    }
    skel.parts.push_back(body);

    // The arms: two tubes from the shoulders to the hands, bound shoulder -> elbow (and the
    // chest at the root), the elbow bone alone beyond the elbow.
    glbparser::SkelPart arms = makePart("fixture_arms", 0.6f, 0.5f, 0.4f);
    for (int side = 0; side < 2; ++side) {
        const float sign = side == 0 ? 1.0f : -1.0f;
        const float a[3] = {sign * 0.20f, 1.50f, 0.0f}, b[3] = {sign * 0.90f, 1.50f, 0.0f};
        const int shoulder = side == 0 ? 5 : 7, elbow = side == 0 ? 6 : 8;
        tube(arms, a, b, 8, 8, [](float t) { return 0.07f - 0.025f * t; },
             [&](const float* p, float, float (&score)[kJointCount]) {
                 const float x = std::fabs(p[0]);
                 score[3] = falloff(x - 0.20f, 0.30f);
                 score[shoulder] = falloff(x - kJoints[shoulder].x * sign, 0.35f);
                 score[elbow] = falloff(x - kJoints[elbow].x * sign, 0.45f);
             });
    }
    skel.parts.push_back(arms);

    // The head: a swelling tube above the neck, the head bone alone, the base blended with the
    // chest.
    glbparser::SkelPart head = makePart("fixture_head", 0.9f, 0.8f, 0.7f);
    {
        const float a[3] = {0.0f, 1.58f, 0.0f}, b[3] = {0.0f, 1.90f, 0.0f};
        tube(head, a, b, 8, 10, [](float t) { return 0.04f + 0.10f * std::sin(t * 3.1415926f); },
             [](const float* p, float, float (&score)[kJointCount]) {
                 score[4] = 1.0f;
                 if (p[1] < 1.66f) score[3] = (1.66f - p[1]) * 6.0f;
             });
    }
    skel.parts.push_back(head);

    // One clip, one rotation track on the chest, so the file has the clip block a model has.
    glbparser::SkelClip clip;
    clip.name = "idle";
    clip.duration = 1.0f;
    glbparser::SkelChannel ch;
    ch.node = 3;
    ch.path = 1;
    ch.times = {0.0f, 0.5f, 1.0f};
    ch.values = {0, 0, 0, 1, 0, 0.1305262f, 0, 0.9914449f, 0, 0, 0, 1};
    clip.channels.push_back(ch);
    skel.clips.push_back(clip);

    // The pose box: the bind box, widened by what a swung arm or a bend reaches.
    float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
    for (const glbparser::SkelPart& p : skel.parts)
        for (size_t i = 0; i < p.positions.size(); ++i) {
            lo[i % 3] = std::fmin(lo[i % 3], p.positions[i]);
            hi[i % 3] = std::fmax(hi[i % 3], p.positions[i]);
        }
    for (int c = 0; c < 3; ++c) {
        skel.min[c] = lo[c] - 0.25f;
        skel.max[c] = hi[c] + 0.25f;
    }

    glbparser::generateSkelLods(skel);  // the ~50 % and ~25 % levels of every part

    std::string error;
    const std::string image = glbparser::writeTskl(skel, std::vector<std::string>{"models/fixture.png"}, &error);
    if (image.empty()) {
        std::printf("make-skin-fixture: writeTskl failed: %s\n", error.c_str());
        return 1;
    }
    FILE* f = std::fopen(argv[1], "wb");
    if (f == nullptr) {
        std::printf("make-skin-fixture: cannot write %s\n", argv[1]);
        return 1;
    }
    std::fwrite(image.data(), 1, image.size(), f);
    std::fclose(f);
    int levels = 0;
    for (const glbparser::SkelPart& p : skel.parts) levels += 1 + (int)p.lods.size();
    std::printf("make-skin-fixture: %s, %zu bytes, %zu parts, %d levels, %d bones\n", argv[1],
                image.size(), skel.parts.size(), levels, kJointCount);
    return 0;
}
