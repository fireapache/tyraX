#include "blendspace.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "animedit.hpp"
#include "animmerge.hpp"
#include "fbxparser.hpp"
#include "glbparser.hpp"
#include "json.hpp"
#include "project.hpp"

namespace blendspace {

namespace {

namespace bs = Tyra::blendspace;

std::string num(float v) {
    if (std::fabs(v) < 5e-7f) v = 0.0f;  // no "-0" or 1e-9 noise in the files
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.6g", (double)v);
    return buf;
}
std::string q(const std::string& s) { return "\"" + json::escape(s) + "\""; }
std::string str(const json::Value& o, const char* key, const std::string& fb = "") {
    const json::Value* v = o.find(key);
    return v ? v->stringOr(fb) : fb;
}
float flt(const json::Value& o, const char* key, float fb) {
    const json::Value* v = o.find(key);
    return v ? (float)v->numberOr(fb) : fb;
}
std::string slashes(std::string s) {
    std::replace(s.begin(), s.end(), '\\', '/');
    return s;
}
bool isCentre(const AssetSample& s) { return s.speed < bs::kRingEpsilon; }

}  // namespace

std::string serialize(const Asset& a) {
    std::string o = "{\n";
    o += "  \"asset\": \"tyrax.blendspace\",\n";
    o += "  \"model\": " + q(slashes(a.model)) + ",\n";
    o += "  \"name\": " + q(a.name) + ",\n";
    o += "  \"stride\": " + num(a.stride) + ",\n";
    o += "  \"smoothing\": " + num(a.smoothing) + ",\n";
    o += "  \"lodFade\": " + num(a.lodFade) + ",\n";
    if (a.grid)  // a radial asset's file is exactly what it always was
        o += "  \"layout\": \"grid\",\n  \"grid\": {\"columns\": " + std::to_string(a.lines.columns) +
             ", \"rows\": " + std::to_string(a.lines.rows) + ", \"direction\": [" + num(a.lines.dirMin) + ", " +
             num(a.lines.dirMax) + "], \"speed\": [" + num(a.lines.speedMin) + ", " + num(a.lines.speedMax) +
             "]},\n";
    o += "  \"samples\": [\n";
    for (size_t i = 0; i < a.samples.size(); ++i) {
        const AssetSample& s = a.samples[i];
        // a grid keeps -180 apart from 180: they are two columns
        o += "    {\"clip\": " + q(s.clip) + ", \"direction\": " + num(a.grid ? s.dir : bs::wrapDeg(s.dir)) +
             ", \"speed\": " + num(s.speed) + ", \"time\": " + (s.own ? "\"own\"" : "\"sync\"") +
             "}";
        o += i + 1 < a.samples.size() ? ",\n" : "\n";
    }
    o += "  ]\n}\n";
    return o;
}

bool parse(const std::string& text, Asset& out, std::string& error) {
    out = Asset();
    json::Value root;
    if (!json::parse(text, root) || root.type != json::Value::Type::Object) {
        error = "not a JSON object";
        return false;
    }
    if (str(root, "asset") != "tyrax.blendspace") {
        error = "not a tyrax.blendspace asset";
        return false;
    }
    out.model = slashes(str(root, "model"));
    out.name = str(root, "name");
    out.stride = flt(root, "stride", 0.0f);
    out.smoothing = flt(root, "smoothing", 0.0f);
    out.lodFade = flt(root, "lodFade", 0.15f);
    out.grid = str(root, "layout", "radial") == "grid";
    if (const json::Value* g = root.find("grid"); out.grid && g) {
        out.lines.columns = (int)flt(*g, "columns", 0.0f);
        out.lines.rows = (int)flt(*g, "rows", 0.0f);
        const auto range = [&](const char* key, float* lo, float* hi) {
            const json::Value* v = g->find(key);
            if (v && v->type == json::Value::Type::Array && v->arr.size() == 2)
                *lo = (float)v->arr[0].numberOr(*lo), *hi = (float)v->arr[1].numberOr(*hi);
        };
        range("direction", &out.lines.dirMin, &out.lines.dirMax);
        range("speed", &out.lines.speedMin, &out.lines.speedMax);
    }
    if (const json::Value* samples = root.find("samples");
        samples && samples->type == json::Value::Type::Array) {
        for (const json::Value& v : samples->arr) {
            AssetSample s;
            s.clip = str(v, "clip");
            const float dir = flt(v, "direction", 0.0f);
            s.dir = out.grid ? dir : bs::wrapDeg(dir);
            s.speed = std::max(0.0f, flt(v, "speed", 0.0f));
            s.own = str(v, "time", "sync") == "own";
            out.samples.push_back(s);
        }
    }
    return true;
}

bool load(const std::string& path, Asset& out, std::string& error) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        error = "cannot read " + path;
        return false;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    return parse(ss.str(), out, error);
}

bool save(const std::string& path, const Asset& a, std::string& error) {
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f) {
        error = "cannot write " + path;
        return false;
    }
    f << serialize(a);
    return (bool)f;
}

Report check(const Asset& a, const std::vector<std::string>& clipNames) {
    Report r;
    if (a.name.empty()) r.errors.push_back("no name in game");
    if (a.name.size() > 31) r.errors.push_back("the name in game is longer than 31 characters");
    if (a.samples.empty()) r.errors.push_back("no samples");
    if ((int)a.samples.size() > bs::kMaxSamples)
        r.errors.push_back(std::to_string(a.samples.size()) + " samples: the engine takes at most " +
                           std::to_string(bs::kMaxSamples) + " (a bigger block loads as none)");
    if (a.grid) {
        // a GRID: lines, every sample on an intersection, one per intersection
        const bs::Grid& g = a.lines;
        if (g.columns < 2 || g.rows < 2)
            r.errors.push_back("a grid needs at least 2 vertical and 2 horizontal lines");
        if (!(g.dirMax > g.dirMin)) r.errors.push_back("the grid's direction range is empty");
        if (!(g.speedMax > g.speedMin)) r.errors.push_back("the grid's speed range is empty");
        if (!r.errors.empty()) return r;
        const float dx = (g.dirMax - g.dirMin) / (float)(g.columns - 1);
        const float dy = (g.speedMax - g.speedMin) / (float)(g.rows - 1);
        std::vector<int> taken((size_t)(g.columns * g.rows), -1);
        std::vector<bool> rowUsed((size_t)g.rows, false);
        for (size_t i = 0; i < a.samples.size(); ++i) {
            const AssetSample& s = a.samples[i];
            const std::string at = "sample " + std::to_string(i + 1) + " (" + s.clip + ")";
            if (std::find(clipNames.begin(), clipNames.end(), s.clip) == clipNames.end())
                r.errors.push_back(at + ": '" + s.clip + "' is not a clip of " + a.model);
            const float cx = (s.dir - g.dirMin) / dx, cy = (s.speed - g.speedMin) / dy;
            const int c = (int)std::lround(cx), rw = (int)std::lround(cy);
            if (c < 0 || c >= g.columns || rw < 0 || rw >= g.rows || std::fabs(cx - (float)c) >= 0.01f ||
                std::fabs(cy - (float)rw) >= 0.01f) {
                r.errors.push_back(at + " is not on an intersection of the grid");
                continue;
            }
            int& t = taken[(size_t)(rw * g.columns + c)];
            if (t >= 0)
                r.errors.push_back(at + " sits on the same intersection as sample " + std::to_string(t + 1) + " (" +
                                   a.samples[(size_t)t].clip + ")");
            else
                t = (int)i;
            rowUsed[(size_t)rw] = true;
        }
        for (int rw = 0; rw < g.rows; ++rw)
            if (!rowUsed[(size_t)rw] && !a.samples.empty())
                r.warnings.push_back("the row at " + num(g.speedMin + (float)rw * dy) +
                                     " m/s has no clip: it takes the nearest row's");
        return r;
    }
    int centres = 0;
    for (size_t i = 0; i < a.samples.size(); ++i) {
        const AssetSample& s = a.samples[i];
        const std::string at = "sample " + std::to_string(i + 1) + " (" + s.clip + ")";
        if (std::find(clipNames.begin(), clipNames.end(), s.clip) == clipNames.end())
            r.errors.push_back(at + ": '" + s.clip + "' is not a clip of " + a.model);
        if (isCentre(s)) ++centres;
        for (size_t j = 0; j < i; ++j) {
            const AssetSample& o = a.samples[j];
            const bool samePoint =
                isCentre(s) && isCentre(o)
                    ? false  // counted as two centres below
                    : std::fabs(s.speed - o.speed) < bs::kRingEpsilon &&
                          std::fabs(bs::wrapDeg(s.dir - o.dir)) < 0.5f;
            if (samePoint)
                r.errors.push_back(at + " sits on the same point as sample " +
                                   std::to_string(j + 1) + " (" + o.clip + ")");
        }
    }
    if (centres > 1) r.errors.push_back("more than one sample at speed 0 (the centre)");
    // Rings that do not share their directions blend with a seam (the
    // diagonal split of a ring-to-ring cell is exact only for matching ones).
    std::vector<float> rings;
    for (const AssetSample& s : a.samples)
        if (!isCentre(s) && std::none_of(rings.begin(), rings.end(), [&](float r) {
                return std::fabs(r - s.speed) < bs::kRingEpsilon;
            }))
            rings.push_back(s.speed);
    if (rings.size() > 1) {
        const auto dirsOf = [&](float ring) {
            std::vector<int> d;
            for (const AssetSample& s : a.samples)
                if (!isCentre(s) && std::fabs(s.speed - ring) < bs::kRingEpsilon)
                    d.push_back((int)std::lround(bs::wrapDeg(s.dir)));
            std::sort(d.begin(), d.end());
            return d;
        };
        const std::vector<int> first = dirsOf(rings[0]);
        for (size_t k = 1; k < rings.size(); ++k)
            if (dirsOf(rings[k]) != first) {
                r.warnings.push_back("the speed rings do not use the same directions: they "
                                     "blend with a seam where a sample is crossed");
                break;
            }
    }
    return r;
}


std::vector<std::string> listAssets(const std::string& projectDir) {
    namespace fs = std::filesystem;
    std::vector<std::string> out;
    const fs::path res = fs::path(projectDir) / "res";
    std::error_code ec;
    if (!fs::exists(res, ec)) return out;
    for (fs::recursive_directory_iterator it(res, ec), end; it != end; it.increment(ec)) {
        if (ec) break;
        if (it->is_regular_file(ec) && it->path().extension() == ".blendspace")
            out.push_back(fs::relative(it->path(), projectDir, ec).generic_string());
    }
    std::sort(out.begin(), out.end());
    return out;
}

std::vector<std::pair<std::string, Asset>> forModel(const std::string& projectDir,
                                                    const std::string& modelRel,
                                                    std::vector<std::string>& warnings) {
    std::vector<std::pair<std::string, Asset>> out;
    const std::string model = slashes(modelRel);
    for (const std::string& rel : listAssets(projectDir)) {
        Asset a;
        std::string err;
        if (!load((std::filesystem::path(projectDir) / rel).string(), a, err)) {
            warnings.push_back(rel + ": " + err);
            continue;
        }
        if (a.model == model) out.push_back({rel, a});
    }
    return out;
}

bool gameSkel(const Project& p, const std::string& modelRel, glbparser::Skel& out,
              std::string& error) {
    out = glbparser::Skel();
    if (!animimport::parseSkel(p.filePath(modelRel), out, error)) return false;
    animmerge::applyImports(animedit::importsFor(p, modelRel), out, nullptr);
    animedit::applyClipEdits(p, modelRel, out);
    return true;
}

}  // namespace blendspace
