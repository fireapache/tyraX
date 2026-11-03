// tyrax-editor --blendspace <projectDir> <command> ... (docs/blend-spaces.md)
//
//   check <asset.blendspace>      the asset against its model's clips as the
//                                 GAME sees them (imports and clip edits
//                                 applied): prints its centre and rings, every
//                                 error and warning; exit 1 on an error
//   weights <asset.blendspace> --at <dirDeg>,<speed>
//                                 the samples the engine blends at that input
//                                 and their weights - the same function the
//                                 console and the tool's preview run
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "blendspace.hpp"
#include "glbparser.hpp"
#include "project.hpp"

namespace blendspace {

namespace {

namespace bs = Tyra::blendspace;

std::string absOf(const std::string& dir, const std::string& rel) {
    return (std::filesystem::path(dir) / rel).make_preferred().string();
}

int usage() {
    std::fprintf(stderr,
                 "usage: tyrax-editor --blendspace <projectDir> check <asset.blendspace>\n"
                 "       tyrax-editor --blendspace <projectDir> weights <asset.blendspace> "
                 "--at <dirDeg>,<speed>\n");
    return 2;
}

int checkCmd(const std::string& dir, const std::string& rel) {
    Asset a;
    std::string err;
    if (!load(absOf(dir, rel), a, err)) {
        std::fprintf(stderr, "%s: %s\n", rel.c_str(), err.c_str());
        return 1;
    }
    Project p;
    if (const std::string e = project::load(p, dir); !e.empty()) {
        std::fprintf(stderr, "%s: %s\n", dir.c_str(), e.c_str());
        return 1;
    }
    glbparser::Skel skel;
    if (!gameSkel(p, a.model, skel, err)) {
        std::fprintf(stderr, "%s: model %s: %s\n", rel.c_str(), a.model.c_str(), err.c_str());
        return 1;
    }
    std::vector<std::string> clips;
    for (const glbparser::SkelClip& c : skel.clips) clips.push_back(c.name);
    std::printf("%s: \"%s\" on %s, %zu samples\n", rel.c_str(), a.name.c_str(), a.model.c_str(),
                a.samples.size());
    if (a.grid) {  // a GRID: its lines, then each row's clips across
        const bs::Grid& g = a.lines;
        std::printf("  grid %d x %d: %g..%g deg across, %g..%g m/s up\n", g.columns, g.rows, (double)g.dirMin,
                    (double)g.dirMax, (double)g.speedMin, (double)g.speedMax);
        for (int r = 0; r < g.rows && g.rows >= 2 && g.columns >= 2; ++r) {
            const float sp = g.speedMin + (float)r * (g.speedMax - g.speedMin) / (float)(g.rows - 1);
            std::printf("  row %.2f m/s:", (double)sp);
            for (const AssetSample& s : a.samples)
                if (std::fabs(s.speed - sp) < 1e-3f)
                    std::printf(" %s %g%s", s.clip.c_str(), (double)s.dir, s.own ? " (own)" : "");
            std::printf("\n");
        }
    }
    for (const AssetSample& s : a.samples)
        if (!a.grid && s.speed < bs::kRingEpsilon)
            std::printf("  centre: %s%s\n", s.clip.c_str(), s.own ? " (own time)" : "");
    std::vector<float> rings;
    for (const AssetSample& s : a.samples) {
        if (a.grid || s.speed < bs::kRingEpsilon) continue;
        bool seen = false;
        for (float r : rings) seen |= std::fabs(r - s.speed) < bs::kRingEpsilon;
        if (!seen) rings.push_back(s.speed);
    }
    std::sort(rings.begin(), rings.end());
    for (float r : rings) {
        std::printf("  ring %.2f m/s:", (double)r);
        for (const AssetSample& s : a.samples)
            if (s.speed >= bs::kRingEpsilon && std::fabs(s.speed - r) < bs::kRingEpsilon)
                std::printf(" %s %g%s", s.clip.c_str(), (double)s.dir, s.own ? " (own time)" : "");
        std::printf("\n");
    }
    const Report rep = check(a, clips);
    for (const std::string& w : rep.warnings) std::printf("warning: %s\n", w.c_str());
    for (const std::string& e : rep.errors) std::printf("error: %s\n", e.c_str());
    std::printf(rep.errors.empty() ? "OK\n" : "%zu error(s)\n", rep.errors.size());
    return rep.errors.empty() ? 0 : 1;
}

int weightsCmd(const std::string& dir, const std::string& rel, const std::string& at) {
    Asset a;
    std::string err;
    if (!load(absOf(dir, rel), a, err)) {
        std::fprintf(stderr, "%s: %s\n", rel.c_str(), err.c_str());
        return 1;
    }
    float d = 0.0f, s = 0.0f;
    if (std::sscanf(at.c_str(), "%f,%f", &d, &s) != 2) return usage();
    const std::vector<bs::Sample> samples = weightSamples(a);
    const bs::Weights w = bs::weightsFor(samples.data(), (int)samples.size(), a.grid ? &a.lines : nullptr, d, s);
    for (int k = 0; k < w.count; ++k)
        std::printf("%s%s %.3f", k ? "  " : "", a.samples[(size_t)w.sample[k]].clip.c_str(),
                    (double)w.weight[k]);
    std::printf("\n");
    return w.count > 0 ? 0 : 1;
}

}  // namespace

int cliMain(int argc, char** argv) {
    if (argc < 5) return usage();
    const std::string dir = argv[2], cmd = argv[3], asset = argv[4];
    if (cmd == "check") return checkCmd(dir, asset);
    if (cmd == "weights") {
        for (int i = 5; i + 1 < argc; ++i)
            if (std::strcmp(argv[i], "--at") == 0) return weightsCmd(dir, asset, argv[i + 1]);
        return usage();
    }
    return usage();
}

}  // namespace blendspace
