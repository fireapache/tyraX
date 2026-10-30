#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "../vendor/tyra/engine/inc/renderer/3d/mesh/dynamic/blend_space_weights.hpp"

struct Project;
namespace glbparser {
struct Skel;
}

// 2D blend-space ASSETS (Tools > 2D Animation Blend Tool, docs/blend-spaces.md).
//
// A `<name>.blendspace` beside an animated model places some of the model's
// clips at (direction, speed) points of a compass: direction in degrees from
// the facing (+ = right, wraps at +-180), speed in m/s (0 = the centre). The
// game puts an object in blend-space mode (setAnimationMode) and feeds it a
// direction and a speed every frame; the engine blends up to 3 samples by the
// weights of the ONE shared function (blend_space_weights.hpp), which the
// tool's preview and the --blendspace CLI use too.
//
// The asset is editor-only: the build resolves its clip names against the
// model's clips as the GAME sees them (imports and clip edits applied) and
// writes it into the model's .tskl ("BLSP" trailer); the file never ships.
namespace blendspace {

struct AssetSample {
    std::string clip;      // a clip's name in game
    float dir = 0.0f;      // degrees from the facing, + = right
    float speed = 0.0f;    // m/s; 0 = the centre (every direction)
    bool own = false;      // own time (an idle) instead of the shared phase
};

struct Asset {
    std::string model;     // project-relative .glb/.fbx
    std::string name;      // the name a game asks for
    float stride = 0.0f;   // m per phase cycle when the engine plays it (0 = by time)
    float smoothing = 0.0f;  // s, the input point's time constant
    float lodFade = 0.15f;   // s, a blend-LOD switch's crossfade
    // the layout: the radial compass (false), or a 2D GRID - samples on the
    // intersections of `lines` (Tyra::blendspace::computeGrid)
    bool grid = false;
    Tyra::blendspace::Grid lines;
    std::vector<AssetSample> samples;
};

std::string serialize(const Asset& a);
bool parse(const std::string& text, Asset& out, std::string& error);
bool load(const std::string& path, Asset& out, std::string& error);
bool save(const std::string& path, const Asset& a, std::string& error);

// What is wrong with an asset against the model's clips (in-game names).
// Errors make it unusable (the build FAILS naming them, check exits 1): a
// clip the model lacks, two centre samples, two samples on one point, no
// samples, more samples than the engine takes, no name. Warnings: speed
// rings that do not share directions (they blend with a seam where a sample
// is crossed).
struct Report {
    std::vector<std::string> errors, warnings;
};
Report check(const Asset& a, const std::vector<std::string>& clipNames);

// The samples as the shared weight function takes them (Weights name ROWS):
// `clip` = the first row naming the same clip and time, so a grid's idle
// along its bottom row counts once.
inline std::vector<Tyra::blendspace::Sample> weightSamples(const Asset& a) {
    std::vector<Tyra::blendspace::Sample> out;
    for (size_t i = 0; i < a.samples.size(); ++i) {
        int same = (int)i;
        for (size_t k = 0; k < i; ++k)
            if (a.samples[k].clip == a.samples[i].clip && a.samples[k].own == a.samples[i].own) {
                same = (int)k;
                break;
            }
        out.push_back({same, a.samples[i].dir, a.samples[i].speed, a.samples[i].own});
    }
    return out;
}

// A radial asset's GRID version (the tool's Save as grid), laid out to blend
// EXACTLY like the compass: columns over -180..180 deg every step - the
// coarsest step every direction sits on - and rows from 0 to the fastest
// ring every step, likewise; every ring a full row, the 180 deg sample at
// -180 too, the centre's clip along the whole bottom row (once, at the column
// nearest 0 deg, when a full row would pass the engine's 32 samples). False,
// saying why, for a layout no grid reproduces: rings not evenly spaced from
// 0 m/s, a ring missing a direction, two samples on one intersection, more
// than 32 samples. Everything but the layout and samples is kept.
inline bool toGrid(const Asset& radial, Asset& out, std::string& error) {
    namespace bsw = Tyra::blendspace;
    const auto centre = [](const AssetSample& s) { return s.speed < bsw::kRingEpsilon; };
    int k = 0;  // columns - 1
    for (int c = 1; c <= 72 && !k; ++c) {
        const float step = 360.0f / (float)c;
        bool fits = true;
        for (const AssetSample& s : radial.samples) {
            const float x = (bsw::wrapDeg(s.dir) + 180.0f) / step;
            if (!centre(s) && std::fabs(x - std::round(x)) > 0.01f) fits = false;
        }
        if (fits) k = c;
    }
    if (!k) {
        error = "the directions sit on no even grid: place every sample on a multiple of one step "
                "(45 deg, 30 deg...)";
        return false;
    }
    float top = 0.0f;
    for (const AssetSample& s : radial.samples)
        if (!centre(s)) top = std::max(top, s.speed);
    if (top <= 0.0f) top = 0.5f;  // a centre alone
    int m = 0;  // rows - 1
    for (int r = 1; r <= 31 && !m; ++r) {
        const float step = top / (float)r;
        bool fits = true;
        for (const AssetSample& s : radial.samples) {
            const float y = s.speed / step;
            if (!centre(s) && std::fabs(y - std::round(y)) * step > bsw::kRingEpsilon * 0.5f) fits = false;
        }
        if (fits) m = r;
    }
    if (!m) {
        error = "the rings are not evenly spaced from 0 m/s: put every ring at a multiple of one speed step";
        return false;
    }
    Asset g = radial;
    g.grid = true;
    g.lines.columns = k + 1, g.lines.rows = m + 1;
    g.lines.dirMin = -180.0f, g.lines.dirMax = 180.0f;
    g.lines.speedMin = 0.0f, g.lines.speedMax = top;
    g.samples.clear();
    const float dx = 360.0f / (float)k, dy = top / (float)m;
    std::vector<int> taken((size_t)((k + 1) * (m + 1)), -1);
    const auto place = [&](const AssetSample& s, int c, int r) {
        int& at = taken[(size_t)(r * (k + 1) + c)];
        if (at >= 0) {
            error = s.clip + " and " + g.samples[(size_t)at].clip + " fall on one grid intersection: move one of them";
            return false;
        }
        AssetSample t = s;
        t.dir = -180.0f + (float)c * dx;
        t.speed = (float)r * dy;
        at = (int)g.samples.size();
        g.samples.push_back(t);
        return true;
    };
    for (const AssetSample& s : radial.samples) {
        if (centre(s)) continue;
        const int c = (int)std::lround((bsw::wrapDeg(s.dir) + 180.0f) / dx), r = (int)std::lround(s.speed / dy);
        if (!place(s, c, r) || (c == k && !place(s, 0, r))) return false;
    }
    for (int r = 1; r <= m; ++r)
        for (int c = 0; c <= k; ++c)
            if (taken[(size_t)(r * (k + 1) + c)] < 0) {
                char at[96];
                std::snprintf(at, sizeof(at), "no clip at %g deg, %g m/s", (double)(-180.0f + (float)c * dx),
                              (double)((float)r * dy));
                error = std::string(at) + ": a grid blends like the compass only when every ring has the "
                                          "same directions, evenly spaced from 0 m/s - give every ring a clip "
                                          "in each direction the others have";
                return false;
            }
    for (const AssetSample& s : radial.samples)
        if (centre(s)) {
            const bool fullRow = (int)g.samples.size() + k + 1 <= bsw::kMaxSamples;
            for (int c = 0; c <= k; ++c)
                if ((fullRow || c == k / 2) && !place(s, c, 0)) return false;
            break;
        }
    if ((int)g.samples.size() > bsw::kMaxSamples) {
        error = std::to_string(g.samples.size()) + " samples on the grid: the engine takes at most " +
                std::to_string(bsw::kMaxSamples) + " - remove some";
        return false;
    }
    // the file reads row by row, slowest first
    std::stable_sort(g.samples.begin(), g.samples.end(), [](const AssetSample& a, const AssetSample& b) {
        return a.speed != b.speed ? a.speed < b.speed : a.dir < b.dir;
    });
    out = g;
    return true;
}

// Whether a GRID asset goes back on the compass blending EXACTLY as it does
// (the tool greys the Layout switch's Radial out otherwise; toRadial then
// converts it). False, in `why` - a line per problem, each saying what to do
// - when the compass would lose a clip or blend differently: directions not
// all the way round, rows closer than it tells apart, more than one clip on
// the bottom (0 m/s) row, an empty spot on a moving row (the compass blends
// across it; the grid takes the nearest), different clips at -180 and 180
// deg, an empty row between two others.
inline bool canRadial(const Asset& grid, std::string& why) {
    namespace bsw = Tyra::blendspace;
    const bsw::Grid& g = grid.lines;
    if (g.columns < 2 || g.rows < 2 || !(g.dirMax > g.dirMin) || !(g.speedMax > g.speedMin)) {
        why = "The grid's lines are not set: at least 2 each way, and a direction and a speed range.";
        return false;
    }
    std::vector<std::string> issues;
    char line[320];
    const float dx = (g.dirMax - g.dirMin) / (float)(g.columns - 1);
    const float dy = (g.speedMax - g.speedMin) / (float)(g.rows - 1);
    const bool round = std::fabs(g.dirMin + 180.0f) < 1e-3f && std::fabs(g.dirMax - 180.0f) < 1e-3f;
    if (!round) {
        std::snprintf(line, sizeof(line),
                      "The directions run %g..%g deg, but the compass goes all the way round: set Directions to "
                      "-180..180.",
                      (double)g.dirMin, (double)g.dirMax);
        issues.push_back(line);
    }
    if (dy < bsw::kRingEpsilon)
        issues.push_back("The rows are closer than 0.05 m/s apart: the compass would merge them. Use fewer rows "
                         "or a wider speed range.");
    std::vector<int> filled((size_t)g.rows, 0);
    for (int r = 0; r < g.rows; ++r) {
        const float sp = g.speedMin + (float)r * dy;
        std::vector<const AssetSample*> at((size_t)g.columns, nullptr);
        for (const AssetSample& s : grid.samples) {
            const float cx = (s.dir - g.dirMin) / dx, cy = (s.speed - g.speedMin) / dy;
            const int c = (int)std::lround(cx), rr = (int)std::lround(cy);
            if (rr != r || c < 0 || c >= g.columns || std::fabs(cx - (float)c) >= 0.01f ||
                std::fabs(cy - (float)rr) >= 0.01f || at[(size_t)c])
                continue;
            at[(size_t)c] = &s;
            ++filled[(size_t)r];
        }
        if (!filled[(size_t)r]) continue;
        if (sp < bsw::kRingEpsilon) {  // the compass's centre: one clip in every direction
            std::vector<std::string> clips;
            for (const AssetSample* s : at)
                if (s && std::find(clips.begin(), clips.end(), s->clip) == clips.end()) clips.push_back(s->clip);
            if (clips.size() > 1) {
                std::string names;
                for (size_t k = 0; k < clips.size(); ++k) names += (k ? ", " : "") + clips[k];
                issues.push_back("The bottom row (0 m/s) holds " + std::to_string(clips.size()) +
                                 " different clips (" + names +
                                 "), but the compass's centre plays one clip in every direction: put one clip "
                                 "on the whole bottom row, or on one intersection and leave the rest empty.");
            }
            continue;
        }
        // a moving row is a ring: a clip in every direction, -180 = 180
        std::string gaps;
        int gapCount = 0;
        for (int c = 0; c < g.columns; ++c)
            if (!at[(size_t)c]) {
                if (gapCount < 4) {
                    std::snprintf(line, sizeof(line), "%s%g", gapCount ? ", " : "", (double)(g.dirMin + (float)c * dx));
                    gaps += line;
                }
                ++gapCount;
            }
        if (gapCount) {
            std::snprintf(line, sizeof(line),
                          "The %g m/s row has no clip at %s%s deg: the compass would blend across the gap "
                          "instead of taking the nearest clip on the row. Put a clip on every intersection of "
                          "the row.",
                          (double)sp, gaps.c_str(), gapCount > 4 ? "..." : "");
            issues.push_back(line);
        }
        const AssetSample* first = at.front();
        const AssetSample* last = at.back();
        if (round && first && last && first->clip != last->clip) {
            std::snprintf(line, sizeof(line),
                          "At %g m/s, -180 deg holds %s and 180 deg holds %s, but on the compass they are one "
                          "direction: put the same clip on both.",
                          (double)sp, first->clip.c_str(), last->clip.c_str());
            issues.push_back(line);
        }
    }
    for (int r = 1; r + 1 < g.rows; ++r) {
        if (filled[(size_t)r]) continue;
        bool below = false, above = false;
        for (int k = 0; k < r; ++k) below |= filled[(size_t)k] > 0;
        for (int k = r + 1; k < g.rows; ++k) above |= filled[(size_t)k] > 0;
        if (below && above) {
            std::snprintf(line, sizeof(line),
                          "The %g m/s row has no clip, but rows below and above it do: the compass would blend "
                          "across it instead of copying the nearest row. Fill it, or change Lines / Speeds.",
                          (double)(g.speedMin + (float)r * dy));
            issues.push_back(line);
        }
    }
    why.clear();
    for (size_t k = 0; k < issues.size(); ++k) why += (k ? "\n" : "") + issues[k];
    return issues.empty();
}

// A grid asset back on the compass (the tool's Layout: Radial): the bottom
// row's speed-0 samples become ONE centre (the first of them), the -180 deg
// copy of a 180 deg sample is dropped, the rest keep their place. Exactly
// undoes toGrid; any other grid keeps its samples where they are (check()
// then says what the compass makes of them).
inline Asset toRadial(const Asset& grid) {
    namespace bsw = Tyra::blendspace;
    Asset r = grid;
    r.grid = false;
    r.samples.clear();
    bool centre = false;
    for (const AssetSample& s : grid.samples) {
        if (s.speed < bsw::kRingEpsilon) {
            if (!centre) {
                AssetSample c = s;
                c.dir = 0.0f, c.speed = 0.0f;
                r.samples.push_back(c);
                centre = true;
            }
            continue;
        }
        bool copy = false;  // -180 deg beside the same clip at 180 deg
        if (s.dir <= -180.0f + 1e-3f)
            for (const AssetSample& o : grid.samples)
                copy |= o.clip == s.clip && o.dir >= 180.0f - 1e-3f && std::fabs(o.speed - s.speed) < 1e-3f;
        if (copy) continue;
        AssetSample t = s;
        t.dir = bsw::wrapDeg(s.dir);
        r.samples.push_back(t);
    }
    return r;
}

// The tool preview's MOVE speed (m/s: it plays the phase with a stride and
// scrolls the floor). The compass ends just past the fastest ring, but a body
// really moves at running speeds - and there the engine clamps the weights to
// the ring (inputPoint). So on or past the ring the preview moves at
// `moveSpeed`, the tool's Move speed (a faster point speed typed in wins), and
// inside it at the point's share of the ring of that: 87 % of the way to the
// jog = 87 % of the jog's speed, so dragging out never jumps the pace. A Move
// speed below the ring speed is the point's own speed.
inline float previewMoveSpeed(const Asset& a, float pointSpeed, float moveSpeed) {
    float ring = 0.0f;
    for (const AssetSample& s : a.samples) ring = std::max(ring, s.speed);
    if (ring < Tyra::blendspace::kRingEpsilon) return pointSpeed;
    const float atRing = std::max(moveSpeed, ring);
    return pointSpeed < ring ? pointSpeed / ring * atRing : std::max(pointSpeed, atRing);
}

// The speed sync clips were authored at: `stride` m over the mean of their
// `cycles` (s); 0 without a stride or a cycle. The Move speed's default.
inline float authoredSpeed(float stride, const float* cycles, int count) {
    float sum = 0.0f;
    int n = 0;
    for (int i = 0; i < count; ++i)
        if (cycles[i] > 0.0f) sum += cycles[i], ++n;
    return stride > 0.0f && n > 0 ? stride * (float)n / sum : 0.0f;
}

// Every .blendspace under <projectDir>/res for `modelRel`, by path, as
// (project-relative path, asset). Unreadable files become warnings.
std::vector<std::pair<std::string, Asset>> forModel(const std::string& projectDir,
                                                    const std::string& modelRel,
                                                    std::vector<std::string>& warnings);

// Every .blendspace under <projectDir>/res (the tool's Blend space list).
std::vector<std::string> listAssets(const std::string& projectDir);

// The model's skeleton with its clips as the game will see them: the same
// parse + imports + clip edits the build's anim bake applies.
bool gameSkel(const Project& p, const std::string& modelRel, glbparser::Skel& out,
              std::string& error);

// tyrax-editor --blendspace <projectDir> check|weights ... (blendspace_cli.cpp)
int cliMain(int argc, char** argv);

}  // namespace blendspace
