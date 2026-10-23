// tyrax-editor --ragdoll <projectDir> <command> ... (docs/ragdoll-editor.md)
//
//   skeleton <model>              write <model>.skeleton (its skin's bones)
//   generate <model> [--force] [--min-bone f] [--mass kg]
//                                 write <model>.ragdoll, bodies generated from
//                                 the skeleton (never over one, unless --force)
//   header <asset.ragdoll> [--out path|-]
//                                 write the asset's C++ data header (default:
//                                 the path the asset names; - = stdout)
//   sim <asset.ragdoll> [--seconds s] [--push x,y,z] [--from clip[@t]]
//       [--clip name --profile name] [--stairs]
//                                 the preview's simulation, headless: falls
//                                 the body from the bind pose (or a clip's
//                                 pose), optionally motorized, and prints how
//                                 it went - where the pelvis ended, how far the
//                                 links stretched, how far a joint passed its
//                                 limit, whether it came to rest
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "animmerge.hpp"
#include "fbxparser.hpp"
#include "glbparser.hpp"
#include "ragdoll.hpp"
#include "ragdollsim.hpp"

namespace ragdoll {

namespace {

std::string absOf(const std::string& dir, const std::string& rel) {
    return (std::filesystem::path(dir) / rel).make_preferred().string();
}

int clipIndex(const glbparser::Skel& skel, const std::string& name) {
    for (size_t i = 0; i < skel.clips.size(); ++i)
        if (skel.clips[i].name == name) return (int)i;
    return -1;
}

// The skeleton bones' globals out of a full node-pose (poseMatrices order).
void boneGlobals(const glbparser::Skel& skel, const Skeleton& s, const std::vector<float>& nodes16,
                 std::vector<float>& out16) {
    out16.assign(s.bones.size() * 16, 0.0f);
    for (size_t i = 0; i < s.bones.size(); ++i) {
        int n = -1;
        for (size_t k = 0; k < skel.nodes.size(); ++k)
            if (skel.nodes[k].name == s.bones[i].name) n = (int)k;
        if (n >= 0 && (size_t)n * 16 + 16 <= nodes16.size())
            std::memcpy(&out16[i * 16], &nodes16[(size_t)n * 16], 16 * sizeof(float));
        else
            std::memcpy(&out16[i * 16], s.bones[i].bind, 16 * sizeof(float));
    }
}

int simCommand(const std::string& dir, const std::string& assetRel, int argc, char** argv, int at) {
    float seconds = 4.0f;
    ragdollsim::V3 push;
    std::string from, clip, profile;
    float fromT = 0.0f;
    bool stairs = false;
    for (int i = at; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
        if (a == "--seconds") seconds = (float)std::atof(next().c_str());
        else if (a == "--push") {
            const std::string v = next();
            std::sscanf(v.c_str(), "%f,%f,%f", &push.x, &push.y, &push.z);
        } else if (a == "--from") {
            from = next();
            const size_t k = from.find('@');
            if (k != std::string::npos) {
                fromT = (float)std::atof(from.c_str() + k + 1);
                from = from.substr(0, k);
            }
        } else if (a == "--clip") clip = next();
        else if (a == "--profile") profile = next();
        else if (a == "--stairs") stairs = true;
    }
    Asset asset;
    Skeleton skel;
    std::string err;
    if (!loadAsset(absOf(dir, assetRel), asset, err) ||
        !loadSkeleton(absOf(dir, asset.skeleton), skel, err)) {
        std::fprintf(stderr, "error: %s\n", err.c_str());
        return 1;
    }
    glbparser::Skel model;
    if (!animimport::parseSkel(absOf(dir, skel.model), model, err)) {
        std::fprintf(stderr, "error: model %s: %s\n", skel.model.c_str(), err.c_str());
        return 1;
    }
    Def def;
    std::vector<std::string> errors;
    if (!resolve(asset, skel, def, errors)) {
        for (const std::string& e : errors) std::fprintf(stderr, "error: %s\n", e.c_str());
        return 1;
    }
    ragdollsim::Sim sim;
    sim.build(def);
    std::vector<float> nodes, bones;
    animmerge::poseMatrices(model, from.empty() ? -1 : clipIndex(model, from), fromT, nodes);
    boneGlobals(model, skel, nodes, bones);
    // the floor under the lowest particle of the start pose
    float floorY = 1e9f;
    {
        ragdollsim::Sim probe;
        probe.build(def);
        probe.start(bones);
        for (size_t k = 0; k < probe.particles().size(); ++k)
            floorY = std::min(floorY, probe.particles()[k].y - def.particles[k].radius);
    }
    sim.setFloor(floorY);
    if (stairs) {  // the body on a landing, steps down toward +Z (the editor's)
        std::vector<ragdollsim::Box> boxes;
        sim.setFloor(ragdollsim::stairs(floorY, boxes));
        sim.setBoxes(boxes);
    }
    // the push lands on the chest: 70% up the start pose
    float lo = 1e9f, hi = -1e9f;
    {
        ragdollsim::Sim probe;
        probe.build(def);
        probe.start(bones);
        for (const ragdollsim::V3& p : probe.particles()) lo = std::min(lo, p.y), hi = std::max(hi, p.y);
    }
    sim.start(bones, ragdollsim::V3(), ragdollsim::V3(0.0f, lo + 0.7f * (hi - lo), 0.0f), push);
    int prof = -1;
    if (!profile.empty()) {
        for (size_t i = 0; i < def.profiles.size(); ++i)
            if (def.profiles[i].name == profile) prof = (int)i;
        if (prof < 0) {
            std::fprintf(stderr, "error: no profile '%s'\n", profile.c_str());
            return 1;
        }
    }
    sim.setProfile(prof);
    const int ci = clip.empty() ? -1 : clipIndex(model, clip);
    if (!clip.empty() && ci < 0) {
        std::fprintf(stderr, "error: no clip '%s'\n", clip.c_str());
        return 1;
    }
    const float dur = ci >= 0 ? std::max(model.clips[(size_t)ci].duration, 1e-3f) : 1.0f;
    const int steps = (int)std::lround(seconds / def.solver.fixedDt);
    float worstOver = 0.0f;
    std::vector<float> angles;
    const ragdollsim::V3 hips0 = sim.particles()[(size_t)def.bodies[0].a];
    for (int i = 0; i < steps; ++i) {
        if (prof >= 0) {
            const float t = ci >= 0 ? std::fmod((float)i * def.solver.fixedDt, dur) : 0.0f;
            animmerge::poseMatrices(model, ci, t, nodes);
            boneGlobals(model, skel, nodes, bones);
            sim.setTargets(bones);
        }
        sim.step();
        sim.jointAngles(angles);
        for (size_t b = 1; b < def.bodies.size(); ++b) {
            const Def::B& bd = def.bodies[b];
            const float p = angles[b * 3], y = angles[b * 3 + 1];
            worstOver = std::max(worstOver, std::max(bd.pitchMin - p, p - bd.pitchMax));
            worstOver = std::max(worstOver, std::max(bd.yawMin - y, y - bd.yawMax));
        }
        if ((i + 1) % (int)std::lround(0.5f / def.solver.fixedDt) == 0 || i + 1 == steps) {
            const ragdollsim::V3 h = sim.particles()[(size_t)def.bodies[0].a];
            float top = -1e9f;
            for (const ragdollsim::V3& p : sim.particles()) top = std::max(top, p.y);
            std::printf("t=%.2f pelvis (%.3f %.3f %.3f) top %.3f motors %.2f%s\n",
                        (double)((float)(i + 1) * def.solver.fixedDt), (double)h.x, (double)h.y,
                        (double)h.z, (double)top, (double)sim.motorScale(),
                        sim.asleep() ? " asleep" : "");
        }
    }
    const ragdollsim::V3 h = sim.particles()[(size_t)def.bodies[0].a];
    std::printf("[ragdoll] %s: %zu particles, %zu bodies, %zu links; pelvis moved %.3f m "
                "(%.3f down), worst joint past its limit %.1f deg, %s\n",
                assetRel.c_str(), def.particles.size(), def.bodies.size(), def.links.size(),
                (double)std::sqrt((h.x - hips0.x) * (h.x - hips0.x) + (h.z - hips0.z) * (h.z - hips0.z)),
                (double)(hips0.y - h.y), (double)worstOver,
                sim.asleep() ? "at rest" : "still moving");
    return 0;
}

}  // namespace

int cliMain(int argc, char** argv) {
    if (argc < 5) {
        std::fprintf(stderr,
                     "usage: tyrax-editor --ragdoll <projectDir> skeleton|generate|header|sim "
                     "<model or asset> [options]\n");
        return 2;
    }
    const std::string dir = argv[2], cmd = argv[3], target = argv[4];
    std::string err, note;
    if (cmd == "skeleton" || cmd == "generate") {
        glbparser::Skel model;
        if (!animimport::parseSkel(absOf(dir, target), model, err)) {
            std::fprintf(stderr, "error: %s\n", err.c_str());
            return 1;
        }
        if (!ensureSkeleton(dir, target, model, note)) {
            std::fprintf(stderr, "error: %s\n", note.c_str());
            return 1;
        }
        std::printf("[ragdoll] %s\n", note.c_str());
        if (cmd == "skeleton") return 0;
        GenOptions opt;
        bool force = false;
        for (int i = 5; i < argc; ++i) {
            const std::string a = argv[i];
            if (a == "--force") force = true;
            else if (a == "--min-bone" && i + 1 < argc) opt.minBoneSize = (float)std::atof(argv[++i]);
            else if (a == "--mass" && i + 1 < argc) opt.totalMass = (float)std::atof(argv[++i]);
        }
        if (force) {
            std::error_code ec;
            std::filesystem::remove(absOf(dir, ragdollPathFor(target)), ec);
        }
        if (!ensureRagdoll(dir, target, model, opt, note)) {
            std::fprintf(stderr, "error: %s\n", note.c_str());
            return 1;
        }
        std::printf("[ragdoll] %s\n", note.c_str());
        return 0;
    }
    if (cmd == "header") {
        std::string out;
        for (int i = 5; i < argc; ++i)
            if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) out = argv[++i];
        Asset a;
        Skeleton s;
        if (!loadAsset(absOf(dir, target), a, err) || !loadSkeleton(absOf(dir, a.skeleton), s, err)) {
            std::fprintf(stderr, "error: %s\n", err.c_str());
            return 1;
        }
        Def d;
        std::vector<std::string> errors;
        if (!resolve(a, s, d, errors)) {
            for (const std::string& e : errors) std::fprintf(stderr, "error: %s\n", e.c_str());
            return 1;
        }
        const std::string text = headerText(a, s, d, target);
        if (out == "-") {
            std::fwrite(text.data(), 1, text.size(), stdout);
            return 0;
        }
        if (out.empty()) out = a.header;
        if (out.empty()) {
            std::fprintf(stderr, "error: the asset names no header; pass --out\n");
            return 1;
        }
        if (!saveText(absOf(dir, out), text, err)) {
            std::fprintf(stderr, "error: %s\n", err.c_str());
            return 1;
        }
        std::printf("[ragdoll] %s -> %s (%zu particles, %zu bodies, %zu links, %zu profiles)\n",
                    target.c_str(), out.c_str(), d.particles.size(), d.bodies.size(),
                    d.links.size(), d.profiles.size());
        return 0;
    }
    if (cmd == "sim") return simCommand(dir, target, argc, argv, 5);
    std::fprintf(stderr, "error: unknown --ragdoll command '%s'\n", cmd.c_str());
    return 2;
}

}  // namespace ragdoll
