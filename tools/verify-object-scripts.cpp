// Host harness: object-script PARAMETERS, SCHEDULE and STORAGE (docs/object-scripts.md).
// Three layers, cheapest first:
//   1. the project file: how an object's "scripts" array reads and writes -
//      bare class names (the pre-parameter form, byte for byte) and
//      { "name", "params" } objects;
//   2. the code generator: the rows and parameter tables of
//      src/gen/object_scripts.gen.cpp, the storage table in scene_data.hpp;
//   3. the generated runtime itself: inc/scripts/script.hpp +
//      src/gen/object_scripts.gen.cpp compiled with a stub engine
//      (tools/verify-object-scripts/stub/tyra) next to user scripts and a frame
//      loop (tools/verify-object-scripts/scenario.cpp), then RUN: start order,
//      class order, start-only markers, inactive objects, onDestroy, storage
//      reuse, no allocation, where the driver sits among the global scripts.
//
// It links the editor's own objects (project.cpp pulls in most of them; the GUI
// code comes along unused). From the repo root, after building the editor into
// <build> (build/, build-dev/, ...):
//
//   Windows (Git Bash, MinGW):
//     B=build
//     OBJS=$(ls $B/CMakeFiles/tyrax-editor.dir/src/*.obj | grep -v /main.cpp.obj)
//     VOBJS=$(find $B/CMakeFiles/tyrax-editor.dir/vendor -name "*.obj")
//     g++ -std=c++20 -O1 -Isrc -I$B/generated -Ivendor/imgui -Ivendor/imguizmo/src
//         -Ivendor/imnodes -Ivendor/stb -Ivendor/ufbx -Ivendor/miniaudio
//         -o verify-object-scripts tools/verify-object-scripts.cpp $OBJS $VOBJS
//         $B/libimgui.a $B/vendor/glfw/src/libglfw3.a -static -lopengl32 -lgdi32
//         -lcomdlg32 -lshell32 -lole32 -luuid -lws2_32 -limm32 -ldwmapi
//     (one command line; it is wrapped here for reading)
//   Linux: the same with *.o and  -ldl -lrt -lm -lGLX -lOpenGL -lpthread -no-pie
//
//   ./verify-object-scripts [scratchDir] [--keep]   -> "ALL n CHECKS PASSED", exit 0
//
// Needs a host g++ on PATH for layer 3 (CXX overrides it) and is run from the
// repo root. A failed layer-3 compile or run prints the compiler / scenario output.
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "../src/json.hpp"
#include "../src/project.hpp"
#include "../src/templates.hpp"

namespace fs = std::filesystem;

static int g_checks = 0, g_failures = 0;
#define CHECK(cond, ...)                                 \
    do {                                                 \
        ++g_checks;                                      \
        if (!(cond)) {                                   \
            ++g_failures;                                \
            std::printf("FAIL %s:%d: ", __FILE__, __LINE__); \
            std::printf(__VA_ARGS__);                    \
            std::printf("\n");                           \
        }                                                \
    } while (0)

static std::string readText(const fs::path& p) {
    std::ifstream f(p, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

static void writeText(const fs::path& p, const std::string& text) {
    std::error_code ec;
    fs::create_directories(p.parent_path(), ec);
    std::ofstream f(p, std::ios::binary);
    f << text;
}

static bool has(const std::string& hay, const std::string& needle) {
    return hay.find(needle) != std::string::npos;
}

static ScriptParam pInt(const char* n, int v) {
    ScriptParam p;
    p.name = n;
    p.kind = ScriptParam::Kind::Int;
    p.i = v;
    return p;
}
static ScriptParam pFloat(const char* n, float v) {
    ScriptParam p;
    p.name = n;
    p.kind = ScriptParam::Kind::Float;
    p.f = v;
    return p;
}
static ScriptParam pBool(const char* n, bool v) {
    ScriptParam p;
    p.name = n;
    p.kind = ScriptParam::Kind::Bool;
    p.i = v ? 1 : 0;
    return p;
}
static ScriptParam pString(const char* n, const char* v) {
    ScriptParam p;
    p.name = n;
    p.kind = ScriptParam::Kind::String;
    p.s = v;
    return p;
}

static SceneObject makeObject(const char* name, std::vector<ScriptAttachment> scripts) {
    SceneObject o;
    o.name = name;
    o.type = PrimitiveType::Empty;
    o.scripts = std::move(scripts);
    return o;
}

static std::string fileOf(const std::vector<templates::File>& files, const std::string& rel) {
    for (const templates::File& f : files)
        if (f.relativePath == rel) return f.content;
    return std::string();
}

// A fresh one-scene project of the given name under `parent`.
static bool createProject(Project& p, const char* name, const fs::path& parent) {
    std::error_code ec;
    fs::remove_all(parent / name, ec);
    TerrainConfig t;
    t.enabled = false;
    t.width = 8;
    t.depth = 8;
    const std::string err = project::create(p, name, parent.string(), t, "empty", 1.0f);
    if (!err.empty()) {
        std::printf("FAIL project::create: %s\n", err.c_str());
        return false;
    }
    p.scenes[0].name = "main";
    p.scenes[0].objects.clear();
    return true;
}

// ---------------------------------------------------------------------------
// 1. The project file
// ---------------------------------------------------------------------------

static void serialization(const fs::path& scratch) {
    Project p;
    if (!createProject(p, "osfile", scratch)) {
        ++g_failures;
        return;
    }
    ScriptAttachment typed("Typed");
    typed.params = {pInt("team", 1),       pFloat("gain", 0.5f),    pFloat("scale", 2.0f),
                    pFloat("tenth", 0.1f), pBool("hq", true),       pBool("off", false),
                    pString("tag", "a\"b\\c")};
    p.scenes[0].objects.push_back(makeObject("a", {"Plain", typed, "Other"}));
    p.scenes[0].objects.push_back(makeObject("b", {"OnlyA", "OnlyB"}));
    project::ensureObjectIds(p);
    CHECK(project::save(p).empty(), "save");

    const std::string idA = p.scenes[0].objects[0].id, idB = p.scenes[0].objects[1].id;
    const std::string jsonA = readText(fs::path(p.dir) / "objects" / (idA + ".json"));
    const std::string jsonB = readText(fs::path(p.dir) / "objects" / (idB + ".json"));

    // The pre-parameter form, byte for byte.
    CHECK(has(jsonB, "\"scripts\": [\"OnlyA\", \"OnlyB\"]"), "bare names stay bare: %s", jsonB.c_str());
    CHECK(!has(jsonB, "params"), "no params key without parameters");
    // The object form: only for the attachment that has parameters, each token
    // carrying its kind (2.0 stays a float, 0.1 stays 0.1).
    CHECK(has(jsonA,
              "\"scripts\": [\"Plain\", { \"name\": \"Typed\", \"params\": { \"team\": 1, "
              "\"gain\": 0.5, \"scale\": 2.0, \"tenth\": 0.1, \"hq\": true, \"off\": false, "
              "\"tag\": \"a\\\"b\\\\c\" } }, \"Other\"]"),
          "object form: %s", jsonA.c_str());

    // Reads back equal, kinds and all.
    Project q;
    CHECK(project::load(q, p.dir).empty(), "load");
    CHECK(q.scenes[0].objects.size() == 2, "two objects, got %zu", q.scenes[0].objects.size());
    if (q.scenes[0].objects.size() == 2) {
        CHECK(q.scenes[0].objects[0].scripts == p.scenes[0].objects[0].scripts,
              "object a's attachments read back equal");
        CHECK(q.scenes[0].objects[1].scripts == p.scenes[0].objects[1].scripts,
              "object b's attachments read back equal");
        const ScriptAttachment& t = q.scenes[0].objects[0].scripts[1];
        CHECK(t.params.size() == 7 && t.params[2].kind == ScriptParam::Kind::Float &&
                  t.params[0].kind == ScriptParam::Kind::Int &&
                  t.params[4].kind == ScriptParam::Kind::Bool &&
                  t.params[6].kind == ScriptParam::Kind::String && t.params[6].s == "a\"b\\c",
              "kinds survive: 2.0 is a float, 1 an int");
    }
    // ...and writes the same bytes again (no churn on a second save).
    CHECK(project::save(q).empty(), "second save");
    CHECK(readText(fs::path(q.dir) / "objects" / (idA + ".json")) == jsonA,
          "a second save is byte-identical");

    // Hand-written input in every spelling, good and bad.
    std::string messy = jsonA;
    const size_t at = messy.find("\"scripts\": [");
    const size_t end = messy.find("]", messy.find("\"Other\"", at));
    messy.replace(at, end - at + 1,
                  "\"scripts\": [\"Plain\", 7, \"\", { \"params\": { \"x\": 1 } }, { \"name\": "
                  "\"\" }, { \"name\": \"Hand\", \"params\": { \"n\": 3, \"f\": 3.5, \"e\": 1e2, "
                  "\"big\": 4000000000, \"t\": true, \"s\": \"hi\", \"n\": 99, \"\": 5, "
                  "\"nul\": null, \"arr\": [1], \"obj\": {} } }, { \"name\": \"Bare\" }]");
    writeText(fs::path(p.dir) / "objects" / (idA + ".json"), messy);
    Project r;
    CHECK(project::load(r, p.dir).empty(), "load of hand-written scripts");
    if (!r.scenes[0].objects.empty()) {
        const auto& s = r.scenes[0].objects[0].scripts;
        CHECK(s.size() == 3, "kept Plain, Hand, Bare; dropped 7, \"\", nameless, empty-named: got %zu",
              s.size());
        if (s.size() == 3) {
            CHECK(s[0].name == "Plain" && s[0].params.empty(), "Plain");
            CHECK(s[2].name == "Bare" && s[2].params.empty(), "an object without params is plain");
            const auto& h = s[1].params;
            CHECK(s[1].name == "Hand" && h.size() == 6, "Hand has 6 usable params, got %zu", h.size());
            if (h.size() == 6) {
                CHECK(h[0].name == "n" && h[0].kind == ScriptParam::Kind::Int && h[0].i == 3,
                      "a whole token is an int and the first of a repeated name wins");
                CHECK(h[1].kind == ScriptParam::Kind::Float && h[1].f == 3.5f, "3.5 is a float");
                CHECK(h[2].kind == ScriptParam::Kind::Float && h[2].f == 100.0f, "1e2 is a float");
                CHECK(h[3].kind == ScriptParam::Kind::Float && h[3].f == 4000000000.0f,
                      "an integer past int32 is a float");
                CHECK(h[4].kind == ScriptParam::Kind::Bool && h[4].i == 1, "true is a bool");
                CHECK(h[5].kind == ScriptParam::Kind::String && h[5].s == "hi", "a string");
            }
        }
    }
    std::error_code ec;
    fs::remove_all(fs::path(p.dir), ec);
}

// ---------------------------------------------------------------------------
// 2 + 3. The generator, and the generated runtime
// ---------------------------------------------------------------------------

// The project scenario.cpp is written against (see its header).
static bool buildScenarioProject(Project& p, const fs::path& scratch) {
    if (!createProject(p, "osverify", scratch)) return false;
    ScriptAttachment marker("Marker");
    marker.params = {pInt("team", 1),   pFloat("gain", 0.5f),
                     pBool("hq", true), pString("tag", "north \"gate\""),
                     pInt("ratio", 3),  pFloat("scale", 2.0f),
                     pInt("team", 99),  // a repeat: the first one answers
                     pInt("", 5)};      // empty: ignored
    auto& mainScene = p.scenes[0].objects;
    mainScene.push_back(makeObject("a", {"Plain", "Late"}));
    mainScene.push_back(makeObject("b", {marker, "Early"}));
    mainScene.push_back(makeObject("c", {"Plain", "Ghost", "Quiet"}));
    mainScene.push_back(makeObject("d", {"Plain"}));
    SceneData second = p.scenes[0];
    second.name = "second";
    second.objects.clear();
    for (const char* n : {"e", "f", "g", "h"}) second.objects.push_back(makeObject(n, {"Plain"}));
    second.objects.push_back(makeObject("i", {"Early"}));
    p.scenes.push_back(second);
    project::ensureObjectIds(p);
    return true;
}

static void generator(const std::vector<templates::File>& files) {
    const std::string cpp = fileOf(files, "src\\gen\\object_scripts.gen.cpp");
    const std::string data = fileOf(files, "inc\\scene_data.hpp");
    const std::string hpp = fileOf(files, "inc\\scripts\\script.hpp");
    CHECK(!cpp.empty() && !data.empty() && !hpp.empty(), "the three files are generated");

    // A bare name is the row the pre-parameter generator wrote.
    CHECK(has(cpp, "    {0, 0, \"Plain\"},  // main / a\n"), "bare row: Plain");
    CHECK(has(cpp, "    {0, 0, \"Late\"},  // main / a\n"), "bare row: Late");
    CHECK(has(cpp, "    {0, 2, \"Ghost\"},  // main / c\n"), "bare row: Ghost");
    CHECK(has(cpp, "constexpr int OBJECT_SCRIPT_ATTACH_COUNT = 13;\n"), "13 attachments");
    CHECK(has(cpp, "constexpr int OBJECT_SCRIPT_MAX_ACTIVE = 8;\n"), "the busiest scene attaches 8");
    // A parameter table per attachment that has parameters, the repeat and the
    // empty name left out, each float spelled to read back as the same float.
    CHECK(has(cpp,
              "constexpr ObjectScriptParam kObjectScriptParams2[] = {\n"
              "    osInt(\"team\", 1),\n"
              "    osFloat(\"gain\", 0.5F),\n"
              "    osBool(\"hq\", true),\n"
              "    osString(\"tag\", \"north \\\"gate\\\"\"),\n"
              "    osInt(\"ratio\", 3),\n"
              "    osFloat(\"scale\", 2.0F),\n"
              "};\n"),
          "the Marker's parameter table");
    CHECK(has(cpp, "    {0, 1, \"Marker\", kObjectScriptParams2, 6},  // main / b\n"),
          "the Marker's row points at its table");
    CHECK(!has(cpp, "kObjectScriptParams0") && !has(cpp, "kObjectScriptParams1"),
          "attachments without parameters get no table");
    CHECK(!has(cpp, "99"), "the repeated name is not emitted");

    // Storage: per class, the most one scene attaches.
    CHECK(has(data, "inline constexpr int OBJECT_SCRIPT_CAP_COUNT = 6;\n"), "six classes");
    CHECK(has(data, "    {\"Plain\", 4},\n"), "Plain: 3 in scene 0, 4 in scene 1");
    CHECK(has(data, "    {\"Early\", 1},\n") && has(data, "    {\"Marker\", 1},\n") &&
              has(data, "    {\"Late\", 1},\n") && has(data, "    {\"Quiet\", 1},\n"),
          "one of the rest");

    // The API the docs promise.
    for (const char* api : {"int scriptOrder = 0;", "class ScriptList", "class MarkerScript",
                            "virtual void onDestroy(ScriptContext&)", "T param(const char* key",
                            "SCRIPT_ORDER_OBJECT_SCRIPTS = 1000", "struct ObjectScriptPool"})
        CHECK(has(hpp, api), "script.hpp has `%s`", api);
    CHECK(!has(hpp, "getObjectScriptFactories"), "the heap-backed factory vector is gone");

    // A project that attaches nothing still generates a valid driver and table.
    Project none;
    fs::path tmp = fs::temp_directory_path() / "tyra-editor-test";
    if (createProject(none, "osnone", tmp)) {
        const auto nf = templates::generate(none);
        const std::string nc = fileOf(nf, "src\\gen\\object_scripts.gen.cpp");
        CHECK(has(nc, "constexpr int OBJECT_SCRIPT_ATTACH_COUNT = 0;\n") &&
                  has(nc, "OBJECT_SCRIPT_ATTACHES[1] = {\n    {0, 0, \"\"},\n};\n"),
              "no attachments: a one-row placeholder table, as before");
        CHECK(has(fileOf(nf, "inc\\scene_data.hpp"), "OBJECT_SCRIPT_CAP_COUNT = 0;\n"),
              "no attachments: an empty storage table");
        std::error_code ec;
        fs::remove_all(fs::path(none.dir), ec);
    }
}

// Writes the generated files layer 3 needs, compiles the scenario against them
// and runs it.
static void runtime(const std::vector<templates::File>& files, const fs::path& scratch) {
    const fs::path rt = scratch / "rt";
    std::error_code ec;
    fs::remove_all(rt, ec);
    for (const char* rel : {"inc\\scene_data.hpp", "inc\\scripts\\script.hpp",
                            "src\\gen\\object_scripts.gen.cpp"}) {
        std::string path = rel;
        for (char& c : path)
            if (c == '\\') c = '/';
        writeText(rt / path, fileOf(files, rel));
    }

    const fs::path src = fs::path("tools") / "verify-object-scripts";
    if (!fs::exists(src / "scenario.cpp")) {
        std::printf("FAIL run from the repo root: %s not found\n", (src / "scenario.cpp").string().c_str());
        ++g_failures;
        return;
    }
    const char* cxxEnv = std::getenv("CXX");
    const std::string cxx = cxxEnv && *cxxEnv ? cxxEnv : "g++";
    const std::string exe = (rt / "scenario").string();
    const std::string log = (rt / "scenario.log").string();
    // The game builds with the compiler's default dialect (gnu++17) and -Wall.
    const std::string compile = cxx + " -std=gnu++17 -Wall -Wextra -Werror -I\"" +
                                (src / "stub").string() + "\" -I\"" + (rt / "inc").string() +
                                "\" -o \"" + exe + "\" \"" + (src / "scenario.cpp").string() +
                                "\" \"" + (rt / "src/gen/object_scripts.gen.cpp").string() +
                                "\" > \"" + log + "\" 2>&1";
    auto sh = [](const std::string& cmd) {
#ifdef _WIN32
        return std::system(("\"" + cmd + "\"").c_str());  // cmd /c strips one pair of quotes
#else
        return std::system(cmd.c_str());
#endif
    };
    const int cc = sh(compile);
    if (cc != 0) {
        ++g_failures;
        std::printf("FAIL the generated runtime does not compile (%s):\n%s\n", cxx.c_str(),
                    readText(log).c_str());
        return;
    }
    ++g_checks;
    const int run = sh("\"" + exe + "\" > \"" + log + "\" 2>&1");
    const std::string out = readText(log);
    ++g_checks;
    if (run != 0 || !has(out, "ALL ")) {
        ++g_failures;
        std::printf("FAIL the scenario (exit %d):\n%s\n", run, out.c_str());
    } else {
        std::printf("scenario: %s", out.c_str());
        // Its checks count here too: "ALL n CHECKS PASSED".
        g_checks += std::atoi(out.c_str() + out.find("ALL ") + 4);
    }
}

int main(int argc, char** argv) {
    // [scratchDir] [--keep]: --keep leaves <scratchDir>/rt (the generated files layer 3
    // compiled) behind to poke at; a failing run always does.
    fs::path scratch = fs::temp_directory_path() / "tyra-editor-test" / "osverify";
    bool keep = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--keep")
            keep = true;
        else
            scratch = argv[i];
    }
    std::error_code ec;
    fs::create_directories(scratch, ec);

    serialization(scratch);

    Project p;
    if (buildScenarioProject(p, scratch)) {
        CHECK(project::save(p).empty(), "save the scenario project");
        const std::vector<templates::File> files = templates::generate(p);
        generator(files);
        runtime(files, scratch);
        fs::remove_all(fs::path(p.dir), ec);
    } else {
        ++g_failures;
    }

    if (g_failures) {
        std::printf("%d of %d CHECKS FAILED\n", g_failures, g_checks);
        return 1;
    }
    std::printf("ALL %d CHECKS PASSED\n", g_checks);
    if (!keep) fs::remove_all(scratch / "rt", ec);
    fs::remove(scratch, ec);  // only when nothing else is in it
    return 0;
}
