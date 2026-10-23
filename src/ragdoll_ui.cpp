// Tools > Ragdoll Editor (docs/ragdoll-editor.md).
//
// The Unreal Engine Physics Asset Editor's counterpart for TyraX's particle
// ragdolls: the skeleton tree and the asset's bodies / particles / profiles on
// the left, a preview in the middle that SIMULATES - the animation alone, the
// ragdoll alone, or the ragdoll motorized toward a clip under a profile - with
// the bodies, particles, links, motor targets and the selected joint's limit
// drawn over it, and the selection's details on the right. The asset is a
// document of its own (Save / Revert / its own undo), not project data.
#include "app.hpp"
#include "app_internal.hpp"

#include <imgui_internal.h>  // SeparatorEx: the toolbar's vertical rules

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <functional>
#include <set>

#include "animedit.hpp"
#include "animmerge.hpp"
#include "fbxparser.hpp"
#include "glbparser.hpp"
#include "ragdoll.hpp"
#include "ragdollsim.hpp"

namespace fs = std::filesystem;
using ragdollsim::V3;

struct App::RagdollEd {
    std::string assetRel;
    ragdoll::Asset asset;
    std::string savedJson, lastJson;
    std::vector<std::string> undo, redo;
    bool loaded = false;
    std::string loadError;
    ragdoll::Skeleton skel;
    glbparser::Skel model;
    bool modelOk = false;
    std::string modelError;
    std::vector<int> boneNode;  // skeleton bone -> model node, -1
    ragdoll::Def def;
    std::vector<std::string> errors;
    bool defOk = false;
    ragdollsim::Sim sim;
    // simulation
    enum Mode { Animation = 0, Ragdoll = 1, Motorized = 2 };
    int mode = Animation;
    bool playing = true;
    float speed = 1.0f;
    float clipTime = 0.0f;
    float acc = 0.0f;
    int clip = -1;     // model clip index, -1 = the bind pose
    int profile = 0;   // index into asset.profiles (motorized)
    bool gravity = true;
    bool stairs = false;
    float pushSpeed = 4.5f;
    int pushDir = 0;   // 0 back, 1 forward, 2 left, 3 right, 4 up
    float floorY = 0.0f;
    float center[3] = {0, 1, 0};
    float radius = 1.0f;
    // view
    float yaw = 40.0f, pitch = 15.0f, zoom = 1.0f, panX = 0.0f, panY = 0.0f;
    bool showMesh = true, wire = false, showBodies = true, showParticles = true,
         showLinks = false, showBones = false, showLimits = true, showTargets = true,
         showAxes = true;
    float leftW = 250.0f, rightW = 330.0f;  // side panels, unscaled px (the splitters)
    float startClipTime = 0.0f;  // where the running simulation began in the clip
    // Ctrl+LMB on a body: the point pressed (particle a, or t of the way
    // a -> b) and its depth on screen (NDC z). Released before a drag it is a
    // PUNCH; dragged, a GRAB that pulls it in a plane facing the view.
    bool ctrlPress = false;  // pressed, neither a punch nor a grab yet
    bool restartReq = false;  // start the mode over next frame (a profile picked in the lists)
    bool grabbing = false;
    int grabA = -1, grabB = -1;
    float grabT = 0.0f, grabZ = 0.0f;
    std::string light;
    // selection
    enum Sel { None, Particle, Body, Bone, Profile };
    int selKind = None;
    int sel = -1;
    int leftTab = 0;
    std::string status;
    // generation
    std::string newModel;
    float genMinBone = 0.05f, genMass = 75.0f;
    // scratch
    std::vector<float> nodes, bones;
    std::vector<std::vector<float>> pos, nrm;
};

namespace {

bool inputName(const char* label, std::string& s) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "%s", s.c_str());
    if (ImGui::InputText(label, buf, sizeof(buf), ImGuiInputTextFlags_EnterReturnsTrue)) {
        std::string v = buf;
        v.erase(std::remove_if(v.begin(), v.end(), [](char c) { return c == '"' || c == '\\'; }), v.end());
        if (!v.empty() && v != s) {
            s = v;
            return true;
        }
    }
    return false;
}

// A combo over names; "" offered first when `allowNone`.
bool nameCombo(const char* label, std::string& cur, const std::vector<std::string>& names,
               bool allowNone) {
    bool changed = false;
    if (ImGui::BeginCombo(label, cur.empty() ? "(none)" : cur.c_str())) {
        if (allowNone && ImGui::Selectable("(none)", cur.empty())) cur.clear(), changed = true;
        for (const std::string& n : names)
            if (ImGui::Selectable(n.c_str(), n == cur) && n != cur) cur = n, changed = true;
        ImGui::EndCombo();
    }
    return changed;
}

void orthoM(const float* in, float* out) {
    std::memcpy(out, in, 16 * sizeof(float));
    for (int c = 0; c < 3; ++c) {
        const float l = std::sqrt(in[c * 4] * in[c * 4] + in[c * 4 + 1] * in[c * 4 + 1] +
                                  in[c * 4 + 2] * in[c * 4 + 2]);
        if (l > 1e-9f)
            for (int r = 0; r < 3; ++r) out[c * 4 + r] = in[c * 4 + r] / l;
    }
}
V3 xf(const float* m, V3 v) {
    return V3(m[0] * v.x + m[4] * v.y + m[8] * v.z + m[12], m[1] * v.x + m[5] * v.y + m[9] * v.z + m[13],
              m[2] * v.x + m[6] * v.y + m[10] * v.z + m[14]);
}
float dot3(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
float len3(V3 a) { return std::sqrt(dot3(a, a)); }
V3 colv(const float* r, int c) { return V3(r[c * 3], r[c * 3 + 1], r[c * 3 + 2]); }
V3 mul3(const float* r, V3 v) { return colv(r, 0) * v.x + colv(r, 1) * v.y + colv(r, 2) * v.z; }

// The joint's axes, in the preview and on the limits they name.
const ImVec4 kPitchCol(0.95f, 0.36f, 0.36f, 1.0f), kYawCol(0.42f, 0.88f, 0.42f, 1.0f),
    kRollCol(0.42f, 0.62f, 1.0f, 1.0f);

void vrule() {
    ImGui::SameLine();
    ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
    ImGui::SameLine();
}

// A range drag with its label coloured (the axis it limits) and on the right.
bool axisRange(const char* id, float* v, const char* label, const ImVec4& col) {
    const bool changed = ImGui::DragFloatRange2(id, &v[0], &v[1], 0.5f, -180.0f, 180.0f, "%.0f", "%.0f");
    ImGui::SameLine();
    ImGui::TextColored(col, "%s", label);
    return changed;
}

// The inverse of a 4x4 (column-major) - the preview's view-projection, to
// turn the mouse back into a point in the world.
bool invert4(const float* m, float* inv) {
    float t[16];
    t[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
    t[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
    t[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
    t[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
    t[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
    t[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
    t[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
    t[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
    t[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] + m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
    t[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] - m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
    t[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] + m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
    t[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] - m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];
    t[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] - m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
    t[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] + m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
    t[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] - m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
    t[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] + m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];
    const float det = m[0] * t[0] + m[1] * t[4] + m[2] * t[8] + m[3] * t[12];
    if (std::fabs(det) < 1e-12f) return false;
    for (int i = 0; i < 16; ++i) inv[i] = t[i] / det;
    return true;
}

const char* kPushNames[] = {"Backward (-Z)", "Forward (+Z)", "Left (+X)", "Right (-X)", "Up"};
V3 pushVector(int dir, float s) {
    switch (dir) {
        case 1: return V3(0, 0, s);
        case 2: return V3(s, 0, 0);
        case 3: return V3(-s, 0, 0);
        case 4: return V3(0, s, 0);
        default: return V3(0, 0, -s);
    }
}

}  // namespace

// ------------------------------------------------------------------ loading --

bool App::openRagdollEditor(const std::string& rel) {
    if (!rag_) rag_ = std::make_shared<RagdollEd>();
    showRagdollEditor_ = true;
    RagdollEd& r = *rag_;
    const std::string ext = fs::path(rel).extension().string();
    std::string assetRel = rel;
    if (ext == ".skeleton" || ext == ".glb" || ext == ".fbx") {
        assetRel = (fs::path(rel).parent_path() / (fs::path(rel).stem().string() + ".ragdoll")).generic_string();
        std::error_code ec;
        if (!fs::exists(project_.filePath(assetRel), ec)) {
            // no ragdoll yet: open the window on the generator for this model
            std::string model = rel;
            if (ext == ".skeleton") {
                ragdoll::Skeleton s;
                std::string err;
                if (ragdoll::loadSkeleton(project_.filePath(rel), s, err)) model = s.model;
            }
            r.newModel = model;
            r.status = "No ragdoll for " + model + " yet - Generate one below.";
            r.loaded = false;
            r.assetRel.clear();
            return true;
        }
    }
    return ragdollLoad(assetRel);
}

bool App::ragdollLoad(const std::string& assetRel) {
    RagdollEd& r = *rag_;
    r = RagdollEd();
    r.assetRel = assetRel;
    std::string err;
    if (!ragdoll::loadAsset(project_.filePath(assetRel), r.asset, err)) {
        r.loadError = err;
        r.status = "Cannot open " + assetRel + ": " + err;
        return false;
    }
    if (!ragdoll::loadSkeleton(project_.filePath(r.asset.skeleton), r.skel, err)) {
        r.loadError = "skeleton " + r.asset.skeleton + ": " + err;
        r.status = r.loadError;
        return false;
    }
    // The model the clips and the mesh come from - merged with its imported
    // clips and edited (trims, renames) exactly as the game sees it.
    std::string merr;
    r.modelOk = animmerge::mergedSkel(project_.filePath(r.skel.model),
                                      animedit::importsFor(project_, r.skel.model), r.model, merr,
                                      &skelCache_);
    if (r.modelOk) animedit::applyClipEdits(project_, r.skel.model, r.model);
    r.modelError = merr;
    r.boneNode.assign(r.skel.bones.size(), -1);
    for (size_t i = 0; i < r.skel.bones.size(); ++i)
        for (size_t n = 0; n < r.model.nodes.size(); ++n)
            if (r.model.nodes[n].name == r.skel.bones[i].name) r.boneNode[i] = (int)n;
    if (r.modelOk) {
        const float* mn = r.model.min;
        const float* mx = r.model.max;
        r.center[0] = 0.5f * (mn[0] + mx[0]);
        r.center[1] = 0.5f * (mn[1] + mx[1]);
        r.center[2] = 0.5f * (mn[2] + mx[2]);
        r.radius = 0.5f * std::sqrt((mx[0] - mn[0]) * (mx[0] - mn[0]) + (mx[1] - mn[1]) * (mx[1] - mn[1]) +
                                    (mx[2] - mn[2]) * (mx[2] - mn[2]));
    }
    r.savedJson = r.lastJson = ragdoll::assetJson(r.asset);
    r.loaded = true;
    ragdollResolve();
    // the floor under the bind pose's lowest particle
    r.floorY = 0.0f;
    if (r.defOk) {
        float lo = 1e9f;
        for (const ragdoll::Def::P& p : r.def.particles) lo = std::min(lo, p.bind[1] - p.radius);
        if (lo < 1e8f) r.floorY = lo;
    }
    // the idle when there is one: the pose a ragdoll is usually seen from
    for (size_t i = 0; i < r.model.clips.size(); ++i)
        if (r.clip < 0 || r.model.clips[i].name == "idle") r.clip = (int)i;
    r.status = "Opened " + assetRel;
    return true;
}

void App::ragdollResolve() {
    RagdollEd& r = *rag_;
    r.errors.clear();
    r.defOk = ragdoll::resolve(r.asset, r.skel, r.def, r.errors) && r.sim.build(r.def);
    if (r.mode != RagdollEd::Animation) ragdollStart();
}

// The skeleton bones' globals at `t` of clip `c` (-1 = bind), from the model.
void App::ragdollClipPose(int c, float t) {
    RagdollEd& r = *rag_;
    if (r.modelOk) {
        animmerge::poseMatrices(r.model, c, t, r.nodes);
    } else {
        r.nodes.clear();
    }
    r.bones.assign(r.skel.bones.size() * 16, 0.0f);
    for (size_t i = 0; i < r.skel.bones.size(); ++i) {
        const int n = r.boneNode[i];
        if (n >= 0 && (size_t)n * 16 + 16 <= r.nodes.size())
            std::memcpy(&r.bones[i * 16], &r.nodes[(size_t)n * 16], 16 * sizeof(float));
        else
            std::memcpy(&r.bones[i * 16], r.skel.bones[i].bind, 16 * sizeof(float));
    }
}

void App::ragdollStart() {
    RagdollEd& r = *rag_;
    if (!r.defOk) return;
    r.sim.build(r.def);
    ragdollClipPose(r.clip, r.clipTime);
    r.sim.setFloor(r.floorY);
    std::vector<ragdollsim::Box> boxes;
    if (r.stairs) r.sim.setFloor(ragdollsim::stairs(r.floorY, boxes));  // a landing, steps down
    r.sim.setBoxes(boxes);
    r.sim.setGravity(r.gravity);
    r.sim.start(r.bones);
    r.startClipTime = r.clipTime;
    r.grabbing = r.ctrlPress = false;
    r.sim.setProfile(r.mode == RagdollEd::Motorized ? r.profile : -1);
    if (r.mode == RagdollEd::Motorized) r.sim.setTargets(r.bones);
    r.acc = 0.0f;
}

// -------------------------------------------------------------- the window --

void App::drawRagdollEditorWindow() {
    if (!showRagdollEditor_ || !hasProject_) return;
    if (!rag_) rag_ = std::make_shared<RagdollEd>();
    RagdollEd& r = *rag_;
    ImGui::SetNextWindowSize(ImVec2(scaled(1180), scaled(720)), ImGuiCond_FirstUseEver);
    const bool dirty = r.loaded && r.lastJson != r.savedJson;
    char title[256];
    std::snprintf(title, sizeof(title), "Ragdoll Editor%s###RagdollEditor", dirty ? " *" : "");
    if (!ImGui::Begin(title, &showRagdollEditor_)) {
        ImGui::End();
        return;
    }

    // --- the document row -----------------------------------------------------
    std::vector<std::string> assets;
    {
        std::error_code ec;
        const fs::path res = project_.filePath("res");
        for (fs::recursive_directory_iterator it(res, ec), end; it != end; it.increment(ec)) {
            if (ec) break;
            if (it->is_regular_file(ec) && it->path().extension() == ".ragdoll")
                assets.push_back(fs::relative(it->path(), project_.dir, ec).generic_string());
        }
        std::sort(assets.begin(), assets.end());
    }
    ImGui::SetNextItemWidth(scaled(300));
    if (ImGui::BeginCombo("Asset", r.assetRel.empty() ? "(none)" : r.assetRel.c_str())) {
        for (const std::string& a : assets)
            if (ImGui::Selectable(a.c_str(), a == r.assetRel) && a != r.assetRel) ragdollLoad(a);
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!r.loaded);
    if (ImGui::Button("Save")) {
        std::string err;
        if (ragdoll::saveText(project_.filePath(r.assetRel), r.lastJson, err)) {
            r.savedJson = r.lastJson;
            r.status = "Saved " + r.assetRel + " (the build regenerates its header)";
        } else {
            r.status = err;
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Revert")) ragdollLoad(r.assetRel);
    ImGui::SameLine();
    ImGui::BeginDisabled(r.undo.empty());
    if (ImGui::Button("Undo")) {
        r.redo.push_back(r.lastJson);
        std::string err;
        ragdoll::parseAsset(r.undo.back(), r.asset, err);
        r.undo.pop_back();
        r.lastJson = ragdoll::assetJson(r.asset);
        ragdollResolve();
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(r.redo.empty());
    if (ImGui::Button("Redo")) {
        r.undo.push_back(r.lastJson);
        std::string err;
        ragdoll::parseAsset(r.redo.back(), r.asset, err);
        r.redo.pop_back();
        r.lastJson = ragdoll::assetJson(r.asset);
        ragdollResolve();
    }
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("New from model...")) ImGui::OpenPopup("New ragdoll");
    if (ImGui::BeginPopup("New ragdoll")) {
        ImGui::TextUnformatted("Generate a ragdoll from an animated model's skeleton");
        std::vector<std::string> models;
        for (const std::string& m : listAnimatedModelFiles()) models.push_back("res/models/" + m);
        ImGui::SetNextItemWidth(scaled(300));
        nameCombo("Model", r.newModel, models, false);
        ImGui::SetNextItemWidth(scaled(160));
        ImGui::SliderFloat("Min bone size", &r.genMinBone, 0.01f, 0.2f, "%.2f of height");
        ImGui::SetNextItemWidth(scaled(160));
        ImGui::DragFloat("Total mass", &r.genMass, 0.5f, 5.0f, 500.0f, "%.1f kg");
        const std::string target = r.newModel.empty() ? "" : ragdoll::ragdollPathFor(r.newModel);
        std::error_code ec;
        const bool exists = !target.empty() && fs::exists(project_.filePath(target), ec);
        if (exists) ImGui::TextColored(ImVec4(1, 0.7f, 0.3f, 1), "%s exists - Generate replaces it", target.c_str());
        ImGui::BeginDisabled(r.newModel.empty());
        if (ImGui::Button("Generate")) {
            glbparser::Skel model;
            std::string err, note;
            if (!animimport::parseSkel(project_.filePath(r.newModel), model, err)) {
                r.status = "Cannot read " + r.newModel + ": " + err;
            } else if (!ragdoll::ensureSkeleton(project_.dir, r.newModel, model, note)) {
                r.status = note;
            } else {
                if (exists) fs::remove(project_.filePath(target), ec);
                ragdoll::GenOptions opt;
                opt.minBoneSize = r.genMinBone;
                opt.totalMass = r.genMass;
                if (ragdoll::ensureRagdoll(project_.dir, r.newModel, model, opt, note)) {
                    ragdollLoad(target);
                    r.status = note;
                } else {
                    r.status = note;
                }
            }
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndDisabled();
        ImGui::EndPopup();
    }
    if (!r.status.empty()) {
        ImGui::SameLine();
        ImGui::TextDisabled("%s", r.status.c_str());
    }
    if (!r.loaded) {
        ImGui::Separator();
        ImGui::TextWrapped(
            "A ragdoll is made from an animated model's skeleton: pick one with 'New from "
            "model...' (a skinned import already wrote its .skeleton), or open a .ragdoll "
            "from the combo or the Asset Browser.");
        if (!r.loadError.empty()) ImGui::TextColored(ImVec4(1, 0.5f, 0.4f, 1), "%s", r.loadError.c_str());
        ImGui::End();
        return;
    }

    // --- the simulation rows --------------------------------------------------
    {
        bool restart = false;
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Simulate");
        ImGui::SameLine();
        // Clicking the mode that is already on starts it OVER (the clip from 0
        // for Animation; the simulation from the pose it began in for the
        // others); another mode starts from the pose shown now.
        auto modeRadio = [&](const char* label, int m) {
            if (ImGui::RadioButton(label, r.mode == m)) {
                if (r.mode == m) r.clipTime = m == RagdollEd::Animation ? 0.0f : r.startClipTime;
                r.mode = m;
                restart = true;
            }
            if (ImGui::IsItemHovered() && r.mode == m) ImGui::SetTooltip("Click again to start over.");
        };
        modeRadio("Animation", RagdollEd::Animation);
        ImGui::SameLine();
        modeRadio("Ragdoll", RagdollEd::Ragdoll);
        ImGui::SameLine();
        modeRadio("Motorized", RagdollEd::Motorized);
        vrule();
        ImGui::TextUnformatted("Clip");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(scaled(150));
        const char* clipName = r.clip >= 0 && r.clip < (int)r.model.clips.size()
                                   ? r.model.clips[(size_t)r.clip].name.c_str()
                                   : "(bind pose)";
        if (ImGui::BeginCombo("##Clip", clipName)) {
            if (ImGui::Selectable("(bind pose)", r.clip < 0)) r.clip = -1, r.clipTime = 0, restart = true;
            for (size_t i = 0; i < r.model.clips.size(); ++i)
                if (ImGui::Selectable(r.model.clips[i].name.c_str(), (int)i == r.clip)) {
                    r.clip = (int)i;
                    r.clipTime = 0.0f;
                    restart = true;
                }
            ImGui::EndCombo();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Animation: what plays. Ragdoll: the pose it starts from.\n"
                              "Motorized: the pose the motors pull toward, playing.");
        ImGui::SameLine();
        ImGui::BeginDisabled(r.mode != RagdollEd::Motorized);
        ImGui::TextUnformatted("Motor profile");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(scaled(130));
        const char* profName = r.profile >= 0 && r.profile < (int)r.asset.profiles.size()
                                   ? r.asset.profiles[(size_t)r.profile].name.c_str()
                                   : "(none)";
        if (ImGui::BeginCombo("##Motor profile", profName)) {
            for (size_t i = 0; i < r.asset.profiles.size(); ++i)
                if (ImGui::Selectable(r.asset.profiles[i].name.c_str(), (int)i == r.profile)) {
                    // a new profile starts the mode over, like clicking it again
                    r.profile = (int)i;
                    r.clipTime = r.startClipTime;
                    restart = true;
                }
            ImGui::EndCombo();
        }
        ImGui::EndDisabled();
        vrule();
        if (ImGui::Button(r.playing ? "Pause" : "Play", ImVec2(scaled(54), 0))) r.playing = !r.playing;
        ImGui::SameLine();
        if (ImGui::Button("Step")) {
            r.playing = false;
            r.acc += r.def.solver.fixedDt;
            r.clipTime += r.def.solver.fixedDt;
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset")) {
            r.clipTime = 0.0f;
            restart = true;
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(scaled(90));
        ImGui::SliderFloat("Speed", &r.speed, 0.05f, 2.0f, "%.2fx");

        // the second row: the push, the world, what the preview shows
        ImGui::BeginDisabled(r.mode == RagdollEd::Animation);
        ImGui::SetNextItemWidth(scaled(80));
        ImGui::DragFloat("##pushs", &r.pushSpeed, 0.1f, 0.0f, 30.0f, "%.1f m/s");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(scaled(120));
        ImGui::Combo("##pushd", &r.pushDir, kPushNames, 5);
        ImGui::SameLine();
        if (ImGui::Button("Shoot")) {
            // at the chest: 70% up the body as it stands now
            float lo = 1e9f, hi = -1e9f;
            for (const V3& p : r.sim.particles()) lo = std::min(lo, p.y), hi = std::max(hi, p.y);
            r.sim.applyPush(V3(0.0f, lo + 0.7f * (hi - lo), 0.0f) + V3(r.center[0], 0, r.center[2]),
                            pushVector(r.pushDir, r.pushSpeed));
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("A shot's push at the chest (the game's ragdollApplyPush).\n"
                              "In the preview, Ctrl+click a body to punch it there (this speed),\n"
                              "Ctrl+drag to grab it there and pull.");
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Checkbox("Gravity", &r.gravity)) r.sim.setGravity(r.gravity);
        ImGui::SameLine();
        if (ImGui::Checkbox("Stairs", &r.stairs)) restart = true;
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Stand the body on a landing above ten 15 cm steps.");
        vrule();
        ImGui::SetNextItemWidth(scaled(90));
        if (ImGui::BeginCombo("##Show", "Show", ImGuiComboFlags_HeightLargest)) {
            ImGui::Checkbox("Mesh", &r.showMesh);
            ImGui::Checkbox("Wireframe", &r.wire);
            ImGui::Checkbox("Bodies", &r.showBodies);
            ImGui::Checkbox("Particles", &r.showParticles);
            ImGui::Checkbox("Links", &r.showLinks);
            ImGui::Checkbox("Bones", &r.showBones);
            ImGui::Checkbox("Limits", &r.showLimits);
            ImGui::Checkbox("Angle axes", &r.showAxes);
            ImGui::Checkbox("Motor targets", &r.showTargets);
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(scaled(90));
        if (ImGui::BeginCombo("##View", "View")) {
            if (ImGui::Selectable("Reset view")) r.yaw = 40.0f, r.pitch = 15.0f, r.zoom = 1.0f, r.panX = r.panY = 0.0f;
            ImGui::TextUnformatted("Light");
            ImGui::SetNextItemWidth(scaled(180));
            if (previewLightCombo("##rag_light", r.light)) saveGlobalConfig();
            ImGui::EndCombo();
        }
        if (r.restartReq) {
            r.restartReq = false;
            r.clipTime = r.mode == RagdollEd::Animation ? 0.0f : r.startClipTime;
            restart = true;
        }
        if (restart && r.mode != RagdollEd::Animation) ragdollStart();
    }

    // --- advance time -----------------------------------------------------------
    const float clipDur = r.clip >= 0 && r.clip < (int)r.model.clips.size()
                              ? std::max(r.model.clips[(size_t)r.clip].duration, 1e-3f)
                              : 0.0f;
    if (r.playing) {
        const float dt = std::min(ImGui::GetIO().DeltaTime, 0.1f) * r.speed;
        r.clipTime += dt;
        if (r.mode != RagdollEd::Animation) r.acc += dt;
    }
    if (clipDur > 0.0f) r.clipTime = std::fmod(std::max(r.clipTime, 0.0f), clipDur);
    if (r.mode != RagdollEd::Animation && r.defOk) {
        if (!r.sim.started()) ragdollStart();
        int steps = 0;
        while (r.acc >= r.def.solver.fixedDt && steps < 8) {
            if (r.mode == RagdollEd::Motorized) {
                ragdollClipPose(r.clip, r.clipTime);
                r.sim.setTargets(r.bones);
            }
            r.sim.step();
            r.acc -= r.def.solver.fixedDt;
            ++steps;
        }
        if (steps == 8) r.acc = 0.0f;  // a slow frame: drop the backlog
    }

    // --- the pose drawn ---------------------------------------------------------
    ragdollClipPose(r.clip, r.clipTime);  // also every non-bone node
    std::vector<V3> parts;  // particle positions for the overlay
    if (r.mode != RagdollEd::Animation && r.defOk && r.sim.started()) {
        r.sim.pose(r.bones);
        parts = r.sim.particles();
    } else if (r.defOk) {
        for (const ragdoll::Def::P& p : r.def.particles) {
            float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
            if (p.bone >= 0) orthoM(&r.bones[(size_t)p.bone * 16], m);
            parts.push_back(xf(m, V3(p.offset[0], p.offset[1], p.offset[2])));
        }
    }
    if (r.modelOk) {
        for (size_t i = 0; i < r.skel.bones.size(); ++i) {
            const int n = r.boneNode[i];
            if (n >= 0 && (size_t)n * 16 + 16 <= r.nodes.size())
                std::memcpy(&r.nodes[(size_t)n * 16], &r.bones[i * 16], 16 * sizeof(float));
        }
        animmerge::skinParts(r.model, r.nodes, r.pos, r.nrm);
    }

    // --- three columns ------------------------------------------------------------
    // the side panels' widths follow the dividers between them and the preview
    {
        const float unit = std::max(scaled(1.0f), 1e-3f);
        const float splitW = scaled(6), minSide = scaled(150), minMid = scaled(220);
        const ImVec2 avail = ImGui::GetContentRegionAvail();
        r.leftW = std::clamp(r.leftW * unit, minSide, std::max(minSide, avail.x - minMid - minSide - 2 * splitW)) / unit;
        r.rightW = std::clamp(r.rightW * unit, minSide, std::max(minSide, avail.x - r.leftW * unit - minMid - 2 * splitW)) / unit;
        const float leftW = r.leftW * unit, rightW = r.rightW * unit;
        const float midW = std::max(minMid, avail.x - leftW - rightW - 2 * splitW);
        auto splitter = [&](const char* id, float& w, float dir) {
            ImGui::SameLine(0, 0);
            ImGui::InvisibleButton(id, ImVec2(splitW, avail.y));
            const bool hot = ImGui::IsItemHovered() || ImGui::IsItemActive();
            if (hot) ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
            if (ImGui::IsItemActive()) w += dir * ImGui::GetIO().MouseDelta.x / unit;
            const ImVec2 a = ImGui::GetItemRectMin(), b = ImGui::GetItemRectMax();
            const float x = 0.5f * (a.x + b.x);
            ImGui::GetWindowDrawList()->AddLine(ImVec2(x, a.y), ImVec2(x, b.y),
                                                ImGui::GetColorU32(hot ? ImGuiCol_SeparatorActive : ImGuiCol_Separator),
                                                hot ? 2.0f : 1.0f);
            ImGui::SameLine(0, 0);
        };
        ImGui::BeginChild("##rag_left", ImVec2(leftW, avail.y), ImGuiChildFlags_Borders);
        drawRagdollTree();
        ImGui::EndChild();
        splitter("##rag_split_left", r.leftW, 1.0f);
        ImGui::BeginChild("##rag_mid", ImVec2(midW, avail.y));
        drawRagdollPreview(parts);
        ImGui::EndChild();
        splitter("##rag_split_right", r.rightW, -1.0f);
        ImGui::BeginChild("##rag_right", ImVec2(0, avail.y), ImGuiChildFlags_Borders);
        drawRagdollDetails();
        ImGui::EndChild();
    }

    // --- the edit bookkeeping: re-resolve and an undo step per settled change ---
    const std::string now = ragdoll::assetJson(r.asset);
    if (now != r.lastJson) {
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left) || r.undo.empty()) {
            r.undo.push_back(r.lastJson);
            if (r.undo.size() > 100) r.undo.erase(r.undo.begin());
            r.redo.clear();
        }
        r.lastJson = now;
        ragdollResolve();
    }
    if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) &&
        ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S, false)) {
        std::string err;
        if (ragdoll::saveText(project_.filePath(r.assetRel), r.lastJson, err))
            r.savedJson = r.lastJson, r.status = "Saved " + r.assetRel;
    }
    ImGui::End();
}

// ---------------------------------------------------------------- the tree --

void App::drawRagdollTree() {
    RagdollEd& r = *rag_;
    ragdoll::Asset& a = r.asset;
    static const char* kSections[] = {"Skeleton", "Bodies", "Particles", "Profiles", "Solver"};
    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::Combo("##Section", &r.leftTab, kSections, 5);
    ImGui::Separator();
    ImGui::BeginChild("##rag_section");
    {
        if (r.leftTab == 0) {
            // bones, as a tree; a bone carrying a body shows it
            std::vector<std::vector<int>> kids(r.skel.bones.size());
            std::vector<int> roots;
            for (size_t i = 0; i < r.skel.bones.size(); ++i) {
                const int p = r.skel.bones[i].parent;
                (p >= 0 ? kids[(size_t)p] : roots).push_back((int)i);
            }
            std::vector<int> pivotBody(r.skel.bones.size(), -1);
            for (size_t b = 0; b < r.def.bodies.size(); ++b) {
                const int pa = r.def.bodies[b].a;
                if (pa >= 0) {
                    const int bone = r.def.particles[(size_t)pa].bone;
                    if (bone >= 0 && pivotBody[(size_t)bone] < 0) pivotBody[(size_t)bone] = (int)b;
                }
            }
            std::function<void(int)> node = [&](int i) {
                const ragdoll::Bone& bone = r.skel.bones[(size_t)i];
                const int body = pivotBody[(size_t)i];
                ImGuiTreeNodeFlags fl = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_DefaultOpen |
                                        ImGuiTreeNodeFlags_SpanAvailWidth;
                if (kids[(size_t)i].empty()) fl |= ImGuiTreeNodeFlags_Leaf;
                const bool selected = (r.selKind == RagdollEd::Bone && r.sel == i) ||
                                      (r.selKind == RagdollEd::Body && r.sel == body && body >= 0);
                if (selected) fl |= ImGuiTreeNodeFlags_Selected;
                char label[160];
                std::snprintf(label, sizeof(label), "%s%s%s##bone%d", bone.name.c_str(),
                              body >= 0 ? "  [" : "", body >= 0 ? (r.def.bodies[(size_t)body].name + "]").c_str() : "", i);
                const bool open = ImGui::TreeNodeEx(label, fl);
                if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) {
                    if (body >= 0) r.selKind = RagdollEd::Body, r.sel = body;
                    else r.selKind = RagdollEd::Bone, r.sel = i;
                }
                if (open) {
                    for (int k : kids[(size_t)i]) node(k);
                    ImGui::TreePop();
                }
            };
            for (int rt : roots) node(rt);
        }
        if (r.leftTab == 1) {
            for (size_t i = 0; i < a.bodies.size(); ++i) {
                char label[128];
                std::snprintf(label, sizeof(label), "%s%s##b%zu", a.bodies[i].parent.empty() ? "(root) " : "",
                              a.bodies[i].name.c_str(), i);
                if (ImGui::Selectable(label, r.selKind == RagdollEd::Body && r.sel == (int)i))
                    r.selKind = RagdollEd::Body, r.sel = (int)i;
            }
            ImGui::SeparatorText("Clusters (held rigid)");
            std::vector<std::string> pnames;
            for (const ragdoll::Particle& p : a.particles) pnames.push_back(p.name);
            for (size_t c = 0; c < a.clusters.size(); ++c) {
                ImGui::PushID((int)c);
                std::vector<std::string>& cl = a.clusters[c];
                for (size_t k = 0; k < cl.size(); ++k) {
                    if (k) ImGui::SameLine();
                    if (ImGui::GetContentRegionAvail().x < scaled(60)) ImGui::NewLine();
                    char chip[96];
                    std::snprintf(chip, sizeof(chip), "%s x##c%zu", cl[k].c_str(), k);
                    if (ImGui::SmallButton(chip)) {
                        cl.erase(cl.begin() + (long)k);
                        break;
                    }
                }
                std::string add;
                ImGui::SetNextItemWidth(scaled(120));
                if (nameCombo("##add", add, pnames, false) && !add.empty() &&
                    std::find(cl.begin(), cl.end(), add) == cl.end())
                    cl.push_back(add);
                ImGui::SameLine();
                if (ImGui::SmallButton("Delete cluster")) {
                    a.clusters.erase(a.clusters.begin() + (long)c);
                    ImGui::PopID();
                    break;
                }
                ImGui::Separator();
                ImGui::PopID();
            }
            if (ImGui::SmallButton("New cluster")) a.clusters.push_back({});
        }
        if (r.leftTab == 2) {
            for (size_t i = 0; i < a.particles.size(); ++i) {
                char label[128];
                std::snprintf(label, sizeof(label), "%s  (%s)##p%zu", a.particles[i].name.c_str(),
                              a.particles[i].bone.c_str(), i);
                if (ImGui::Selectable(label, r.selKind == RagdollEd::Particle && r.sel == (int)i))
                    r.selKind = RagdollEd::Particle, r.sel = (int)i;
            }
            if (ImGui::SmallButton("Add particle")) {
                ragdoll::Particle p;
                p.name = "particle";
                for (int k = 1; a.particle(p.name) >= 0; ++k) p.name = "particle_" + std::to_string(k);
                p.bone = r.skel.bones.empty() ? "" : r.skel.bones[0].name;
                if (r.selKind == RagdollEd::Bone && r.sel >= 0) p.bone = r.skel.bones[(size_t)r.sel].name;
                a.particles.push_back(p);
                r.selKind = RagdollEd::Particle, r.sel = (int)a.particles.size() - 1;
            }
        }
        if (r.leftTab == 3) {
            for (size_t i = 0; i < a.profiles.size(); ++i) {
                char label[128];
                std::snprintf(label, sizeof(label), "%s##pr%zu", a.profiles[i].name.c_str(), i);
                if (ImGui::Selectable(label, r.selKind == RagdollEd::Profile && r.sel == (int)i)) {
                    // the preview's profile too: the mode starts over with it
                    r.selKind = RagdollEd::Profile, r.sel = (int)i, r.profile = (int)i;
                    r.restartReq = true;
                }
            }
            if (ImGui::SmallButton("New profile")) {
                ragdoll::Profile p;
                p.name = "profile";
                for (int k = 1; a.profile(p.name); ++k) p.name = "profile_" + std::to_string(k);
                for (const ragdoll::Body& b : a.bodies) p.joints.push_back({b.name, false, 0.5f});
                a.profiles.push_back(p);
                r.selKind = RagdollEd::Profile, r.sel = (int)a.profiles.size() - 1;
            }
            ImGui::TextDisabled("A profile is what a MOTORIZED ragdoll does\nwith the clip: per body, absolute or\nrelative, and how strongly.");
        }
        if (r.leftTab == 4) {
            ragdoll::Solver& s = a.solver;
            auto f = [&](const char* label, float* v, float lo, float hi, const char* fmt, const char* tip) {
                ImGui::SetNextItemWidth(scaled(110));
                ImGui::DragFloat(label, v, (hi - lo) / 500.0f, lo, hi, fmt);
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tip);
            };
            ImGui::TextDisabled("The preview's simulation. The defaults\nare Gunner's tuning; a game keeps its\nown in its code.");
            ImGui::SetNextItemWidth(scaled(110));
            ImGui::SliderInt("Iterations", &s.iterations, 1, 20);
            f("Gravity", &s.gravity, 0, 30, "%.2f m/s2", "Downward acceleration.");
            f("Damping", &s.damping, 0.9f, 1.0f, "%.3f", "Share of its velocity a particle keeps a step.");
            f("Static friction", &s.staticFriction, 0, 2, "%.2f", "A contact grips while its slide this step stays under this x the ground's push back.");
            f("Sliding friction", &s.kineticFriction, 0, 2, "%.2f", "Above it, the slide is cut by this x the push back.");
            f("Rolling", &s.rolling, 0, 1, "%.2f", "Speed a grounded particle loses a step.");
            f("Limit stiffness", &s.limitStiffness, 0, 1, "%.2f", "Share of a joint-limit violation undone a pass.");
            f("Joint damping", &s.jointDamping, 0, 1, "%.2f", "Share of a joint's swing speed lost a step.");
            f("Motor gain", &s.motorGain, 0, 1, "%.2f", "Gap to the target a ratio-1 motor closes a step.");
            f("Motor velocity", &s.motorVelocity, 0, 1, "%.2f", "Share of a motor's correction kept as velocity.");
            f("Motor when down", &s.motorLying, 0, 1, "%.2f", "Strength left to groundFade bodies once the pelvis is down.");
            f("Motor fade time", &s.motorFadeTime, 0, 3, "%.2f s", "How long that fade takes.");
            f("Push radius", &s.pushRadius, 0, 2, "%.2f m", "A shot's push falls off over this.");
            f("Push topple", &s.pushTopple, 0, 1, "%.2f", "Share of the push at the top of the body (a turn about the feet).");
            f("Sleep speed", &s.sleepSpeed, 0, 1, "%.2f m/s", "Below it a particle counts as still.");
            f("Sleep time", &s.sleepTime, 0, 3, "%.2f s", "Still this long: asleep.");
            f("Max awake", &s.maxAwake, 0, 30, "%.1f s", "Asleep after this, regardless (a plain ragdoll).");
            if (ImGui::SmallButton("Defaults")) s = ragdoll::Solver();
        }
    }
    ImGui::EndChild();
}

// ----------------------------------------------------------------- details --

void App::drawRagdollDetails() {
    RagdollEd& r = *rag_;
    ragdoll::Asset& a = r.asset;
    if (!r.errors.empty()) {
        ImGui::TextColored(ImVec4(1, 0.5f, 0.4f, 1), "%zu problem%s - it cannot simulate:",
                           r.errors.size(), r.errors.size() == 1 ? "" : "s");
        for (const std::string& e : r.errors) ImGui::TextWrapped("- %s", e.c_str());
        ImGui::Separator();
    }
    std::vector<std::string> pnames, bnames, bones;
    for (const ragdoll::Particle& p : a.particles) pnames.push_back(p.name);
    for (const ragdoll::Body& b : a.bodies) bnames.push_back(b.name);
    for (const ragdoll::Bone& b : r.skel.bones) bones.push_back(b.name);
    ImGui::PushItemWidth(-scaled(110));

    if (r.selKind == RagdollEd::Body && r.sel >= 0 && r.sel < (int)a.bodies.size()) {
        ragdoll::Body& b = a.bodies[(size_t)r.sel];
        ImGui::SeparatorText("Body");
        std::string name = b.name;
        if (inputName("Name", name) && a.body(name) < 0) {
            for (ragdoll::Body& o : a.bodies)
                if (o.parent == b.name) o.parent = name;
            for (ragdoll::Profile& p : a.profiles)
                for (ragdoll::ProfileJoint& j : p.joints)
                    if (j.body == b.name) j.body = name;
            b.name = name;
        }
        nameCombo("Pivot", b.pivot, pnames, false);
        nameCombo("End", b.end, pnames, false);
        if (r.sel > 0) {
            std::vector<std::string> earlier(bnames.begin(), bnames.begin() + r.sel);
            nameCombo("Parent", b.parent, earlier, false);
        } else {
            ImGui::TextDisabled("The root body (no parent)");
        }
        nameCombo("Side A (+X)", b.sideA, pnames, true);
        nameCombo("Side B (-X)", b.sideB, pnames, true);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("A side pair gives the body its own frame (the pelvis' hips,\n"
                              "the chest's shoulders). The root needs one.");
        ImGui::SeparatorText("Limits (degrees from the bind pose)");
        ImGui::Checkbox("Hinge (pitch only)", &b.hinge);
        axisRange("##pitch", b.pitch, "Pitch", kPitchCol);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("+ swings the far end toward the pitch direction (red).");
        ImGui::BeginDisabled(b.hinge);
        axisRange("##yaw", b.yaw, "Yaw", kYawCol);
        ImGui::EndDisabled();
        axisRange("##roll", b.roll, "Roll", kRollCol);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Twist about its own axis (blue): acts on a side-pair body against\n"
                              "its parent and on a hinge's parent (its bend plane).");
        ImGui::DragFloat3("##pitchdir", b.pitchDir, 0.01f, -1.0f, 1.0f, "%.2f");
        ImGui::SameLine();
        ImGui::TextColored(kPitchCol, "Pitch dir");
        ImGui::DragFloat3("##yawdir", b.yawDir, 0.01f, -1.0f, 1.0f, "%.2f");
        ImGui::SameLine();
        ImGui::TextColored(kYawCol, "Yaw dir");
        if (ImGui::SmallButton("Default limits from the bone name")) {
            const int pi = a.particle(b.pivot), ei = a.particle(b.end);
            if (pi >= 0 && ei >= 0 && pi < (int)r.def.particles.size() && ei < (int)r.def.particles.size()) {
                const std::string boneName = a.particles[(size_t)pi].bone;
                ragdoll::defaultJoint(boneName, b.parent.empty(), r.def.particles[(size_t)pi].bind,
                                      r.def.particles[(size_t)ei].bind, b);
            }
        }
        ImGui::Checkbox("Motors fade once down", &b.groundFade);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("What carries the weight (legs, pelvis, chest): a motorized\n"
                              "body stops fighting the ground once its pelvis is down.");
        if (r.mode != RagdollEd::Animation && r.sim.started() && r.sel < (int)r.def.bodies.size()) {
            std::vector<float> ang;
            r.sim.jointAngles(ang);
            const float p = ang[(size_t)r.sel * 3], y = ang[(size_t)r.sel * 3 + 1], rl = ang[(size_t)r.sel * 3 + 2];
            const bool over = p < b.pitch[0] - 0.5f || p > b.pitch[1] + 0.5f ||
                              (!b.hinge && (y < b.yaw[0] - 0.5f || y > b.yaw[1] + 0.5f));
            ImGui::TextColored(over ? ImVec4(1, 0.6f, 0.3f, 1) : ImVec4(0.6f, 0.9f, 0.6f, 1),
                               "now: pitch %.0f  yaw %.0f  roll %.0f", (double)p, (double)y, (double)rl);
        }
        ImGui::Separator();
        if (ImGui::Button("Delete body")) {
            const std::string gone = b.name, up = b.parent;
            for (ragdoll::Body& o : a.bodies)
                if (o.parent == gone) o.parent = up;
            for (ragdoll::Profile& p : a.profiles)
                p.joints.erase(std::remove_if(p.joints.begin(), p.joints.end(),
                                              [&](const ragdoll::ProfileJoint& j) { return j.body == gone; }),
                               p.joints.end());
            a.bodies.erase(a.bodies.begin() + r.sel);
            r.selKind = RagdollEd::None, r.sel = -1;
        }
    } else if (r.selKind == RagdollEd::Particle && r.sel >= 0 && r.sel < (int)a.particles.size()) {
        ragdoll::Particle& p = a.particles[(size_t)r.sel];
        ImGui::SeparatorText("Particle");
        std::string name = p.name;
        if (inputName("Name", name) && a.particle(name) < 0) {
            for (ragdoll::Body& b : a.bodies)
                for (std::string* s : {&b.pivot, &b.end, &b.sideA, &b.sideB})
                    if (*s == p.name) *s = name;
            for (auto& c : a.clusters)
                for (std::string& n : c)
                    if (n == p.name) n = name;
            p.name = name;
        }
        nameCombo("Bone", p.bone, bones, false);
        ImGui::DragFloat3("Offset", p.offset, 0.002f, -2.0f, 2.0f, "%.3f");
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("In the bone's bind frame (m).");
        ImGui::DragFloat("Radius", &p.radius, 0.001f, 0.005f, 1.0f, "%.3f m");
        ImGui::DragFloat("Mass", &p.mass, 0.05f, 0.05f, 200.0f, "%.2f kg");
        ImGui::Checkbox("Collides with props (game)", &p.world);
        ImGui::Separator();
        bool used = false;
        for (const ragdoll::Body& b : a.bodies)
            used |= b.pivot == p.name || b.end == p.name || b.sideA == p.name || b.sideB == p.name;
        ImGui::BeginDisabled(used);
        if (ImGui::Button("Delete particle")) {
            const std::string gone = p.name;
            for (auto& c : a.clusters) c.erase(std::remove(c.begin(), c.end(), gone), c.end());
            a.particles.erase(a.particles.begin() + r.sel);
            r.selKind = RagdollEd::None, r.sel = -1;
        }
        ImGui::EndDisabled();
        if (used) ImGui::TextDisabled("A body uses it.");
    } else if (r.selKind == RagdollEd::Profile && r.sel >= 0 && r.sel < (int)a.profiles.size()) {
        ragdoll::Profile& pr = a.profiles[(size_t)r.sel];
        ImGui::SeparatorText("Motor profile");
        std::string name = pr.name;
        if (inputName("Name", name) && !a.profile(name)) pr.name = name;
        ImGui::TextWrapped("ABSOLUTE: the body is pulled to where the clip puts it in the world. "
                           "RELATIVE: to its pose against its parent body (it shapes a fall, never "
                           "lifts it). The ratio is how strongly, 0 = loose.");
        static float all = 0.5f;
        ImGui::SetNextItemWidth(scaled(90));
        ImGui::SliderFloat("##all", &all, 0.0f, 1.0f, "%.2f");
        ImGui::SameLine();
        if (ImGui::SmallButton("Set all")) {
            for (const ragdoll::Body& b : a.bodies) {
                bool found = false;
                for (ragdoll::ProfileJoint& j : pr.joints)
                    if (j.body == b.name) j.ratio = all, found = true;
                if (!found) pr.joints.push_back({b.name, false, all});
            }
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("All relative"))
            for (ragdoll::ProfileJoint& j : pr.joints) j.relative = true;
        ImGui::SameLine();
        if (ImGui::SmallButton("All absolute"))
            for (ragdoll::ProfileJoint& j : pr.joints) j.relative = false;
        if (ImGui::BeginTable("##prof", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("Body");
            ImGui::TableSetupColumn("Relative", ImGuiTableColumnFlags_WidthFixed, scaled(60));
            ImGui::TableSetupColumn("Ratio");
            ImGui::TableHeadersRow();
            for (const ragdoll::Body& b : a.bodies) {
                ragdoll::ProfileJoint* j = nullptr;
                for (ragdoll::ProfileJoint& x : pr.joints)
                    if (x.body == b.name) j = &x;
                if (!j) {
                    pr.joints.push_back({b.name, false, 0.0f});
                    j = &pr.joints.back();
                }
                ImGui::PushID(b.name.c_str());
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(b.name.c_str());
                ImGui::TableNextColumn();
                ImGui::Checkbox("##rel", &j->relative);
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-1);
                ImGui::SliderFloat("##ratio", &j->ratio, 0.0f, 1.0f, "%.2f");
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        ImGui::Separator();
        if (ImGui::Button("Duplicate")) {
            ragdoll::Profile c = pr;
            c.name += "_copy";
            a.profiles.push_back(c);
        }
        ImGui::SameLine();
        if (ImGui::Button("Delete profile")) {
            a.profiles.erase(a.profiles.begin() + r.sel);
            r.selKind = RagdollEd::None, r.sel = -1;
            r.profile = 0;
        }
    } else if (r.selKind == RagdollEd::Bone && r.sel >= 0 && r.sel < (int)r.skel.bones.size()) {
        const ragdoll::Bone& bone = r.skel.bones[(size_t)r.sel];
        ImGui::SeparatorText("Bone");
        ImGui::TextUnformatted(bone.name.c_str());
        const int follows = r.sel < (int)r.def.boneBody.size() ? r.def.boneBody[(size_t)r.sel] : -1;
        ImGui::TextDisabled("follows body: %s",
                            follows >= 0 ? r.def.bodies[(size_t)follows].name.c_str() : "(none)");
        ImGui::TextWrapped("No body is hung on this bone; it rides the nearest one above it.");
        if (ImGui::Button("Add a body on this bone")) {
            // pivot: a particle on the bone (made at its origin if none);
            // end: its child's origin, or along it; parent: the body it rode
            auto bonePos = [&](int i) {
                const float* m = r.skel.bones[(size_t)i].bind;
                return V3(m[12], m[13], m[14]);
            };
            int child = -1;
            for (size_t k = 0; k < r.skel.bones.size(); ++k)
                if (r.skel.bones[k].parent == r.sel) child = (int)k;
            const V3 p0 = bonePos(r.sel);
            V3 p1 = child >= 0 ? bonePos(child) : p0 + V3(0, 0.1f, 0);
            if (child < 0 && bone.parent >= 0) {
                V3 d = p0 - bonePos(bone.parent);
                const float l = len3(d);
                if (l > 1e-4f) p1 = p0 + d * (0.6f);
            }
            auto addParticle = [&](const std::string& base, int boneIdx, V3 w) {
                ragdoll::Particle p;
                p.name = base;
                for (int k = 1; a.particle(p.name) >= 0; ++k) p.name = base + "_" + std::to_string(k);
                p.bone = r.skel.bones[(size_t)boneIdx].name;
                float m[16];
                orthoM(r.skel.bones[(size_t)boneIdx].bind, m);
                const V3 d = w - V3(m[12], m[13], m[14]);
                p.offset[0] = dot3(V3(m[0], m[1], m[2]), d);
                p.offset[1] = dot3(V3(m[4], m[5], m[6]), d);
                p.offset[2] = dot3(V3(m[8], m[9], m[10]), d);
                p.radius = std::max(0.03f, 0.25f * len3(p1 - p0));
                p.mass = 2.0f;
                a.particles.push_back(p);
                return p.name;
            };
            std::string pivot;
            for (const ragdoll::Particle& p : a.particles)
                if (p.bone == bone.name && len3(V3(p.offset[0], p.offset[1], p.offset[2])) < 1e-4f) pivot = p.name;
            std::string lowerName = bone.name;
            for (char& c : lowerName) c = (char)std::tolower((unsigned char)c);
            if (pivot.empty()) pivot = addParticle(lowerName, r.sel, p0);
            const std::string end = addParticle(lowerName + "_end", child >= 0 ? child : r.sel, p1);
            ragdoll::Body b;
            b.name = lowerName;
            for (int k = 1; a.body(b.name) >= 0; ++k) b.name = lowerName + "_" + std::to_string(k);
            b.pivot = pivot;
            b.end = end;
            b.parent = follows >= 0 ? r.def.bodies[(size_t)follows].name : "";
            const float pv[3] = {p0.x, p0.y, p0.z}, ev[3] = {p1.x, p1.y, p1.z};
            ragdoll::defaultJoint(bone.name, b.parent.empty(), pv, ev, b);
            // the new pivot must be rigid with the parent: in its cluster
            if (follows >= 0) {
                const ragdoll::Body& par = a.bodies[(size_t)follows];
                if (par.end != pivot && par.pivot != pivot) {
                    std::vector<std::string>* cl = nullptr;
                    for (auto& c : a.clusters)
                        if (std::find(c.begin(), c.end(), par.pivot) != c.end() &&
                            std::find(c.begin(), c.end(), par.end) != c.end())
                            cl = &c;
                    if (!cl) {
                        a.clusters.push_back({par.pivot, par.end});
                        cl = &a.clusters.back();
                    }
                    cl->push_back(pivot);
                }
            }
            a.bodies.push_back(b);
            for (ragdoll::Profile& pr : a.profiles) pr.joints.push_back({b.name, false, 0.5f});
            r.selKind = RagdollEd::Body, r.sel = (int)a.bodies.size() - 1;
        }
    } else {
        ImGui::SeparatorText("Ragdoll");
        ImGui::TextDisabled("%s", r.assetRel.c_str());
        ImGui::TextDisabled("skeleton %s (%zu bones)", a.skeleton.c_str(), r.skel.bones.size());
        ImGui::TextDisabled("model %s%s", r.skel.model.c_str(), r.modelOk ? "" : "  (unreadable)");
        ImGui::Text("%zu bodies, %zu particles, %zu links, %zu profiles", r.def.bodies.size(),
                    r.def.particles.size(), r.def.links.size(), a.profiles.size());
        float mass = 0.0f;
        for (const ragdoll::Particle& p : a.particles) mass += p.mass;
        ImGui::Text("total mass %.1f kg", (double)mass);
        ImGui::SeparatorText("Generated header");
        ImGui::TextWrapped("A game consumes the ragdoll as a C++ data header, regenerated on every build.");
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%s", a.header.c_str());
        if (ImGui::InputText("Header", buf, sizeof(buf))) a.header = buf;
        std::snprintf(buf, sizeof(buf), "%s", a.nameSpace.c_str());
        if (ImGui::InputText("Namespace", buf, sizeof(buf))) a.nameSpace = buf;
        ImGui::BeginDisabled(a.header.empty() || !r.defOk);
        if (ImGui::Button("Write header now")) {
            std::string err;
            if (ragdoll::saveText(project_.filePath(a.header),
                                  ragdoll::headerText(a, r.skel, r.def, r.assetRel), err))
                r.status = "Wrote " + a.header;
            else
                r.status = err;
        }
        ImGui::EndDisabled();
        ImGui::SeparatorText("How to");
        ImGui::TextWrapped(
            "Click a body or particle in the preview or the lists. Skeleton: a bone without a "
            "body offers 'Add a body'. Simulate Ragdoll and Shoot (or Ctrl+click a body to punch "
            "it, Ctrl+drag to pinch and pull it) to see the limits hold; Motorized plays a clip "
            "through a profile. Click the active mode again, or pick a profile, to restart it.");
    }
    ImGui::PopItemWidth();
}

// ----------------------------------------------------------------- preview --

void App::drawRagdollPreview(const std::vector<V3>& parts) {
    RagdollEd& r = *rag_;
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const int pw = (int)std::max(scaled(200), avail.x);
    const int ph = (int)std::max(scaled(200), avail.y);
    Viewport::RagdollPreviewDesc d;
    d.key = r.skel.model;
    d.skel = r.modelOk ? &r.model : nullptr;
    d.positions = &r.pos;
    d.normals = &r.nrm;
    d.drawMesh = r.showMesh;
    d.wireframe = r.wire;
    std::memcpy(d.center, r.center, sizeof(d.center));
    d.radius = r.radius;
    d.floorY = r.floorY;
    if (r.stairs) {
        std::vector<ragdollsim::Box> boxes;
        d.floorY = ragdollsim::stairs(r.floorY, boxes);
        for (const ragdollsim::Box& b : boxes)
            d.boxes.push_back({b.mn.x, b.mn.y, b.mn.z, b.mx.x, b.mx.y, b.mx.z});
    }
    d.angleDeg = r.yaw, d.pitchDeg = r.pitch, d.zoom = r.zoom, d.panX = r.panX, d.panY = r.panY;
    d.light = previewLight(r.light);
    float vp[16];
    const uint32_t tex = viewport_.renderRagdollPreview(pw, ph, d, vp);
    if (!tex) return;
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    ImGui::Image((ImTextureID)(intptr_t)tex, ImVec2((float)pw, (float)ph), ImVec2(0, 1), ImVec2(1, 0));
    ImGui::SetCursorScreenPos(origin);
    ImGui::InvisibleButton("Ragdoll preview##rag_view", ImVec2((float)pw, (float)ph),
                           ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight |
                               ImGuiButtonFlags_MouseButtonMiddle);
    const bool hovered = ImGui::IsItemHovered();
    const bool active = ImGui::IsItemActive();
    const bool pressed = ImGui::IsItemActivated();
    ImGuiIO& io = ImGui::GetIO();
    if (hovered && io.MouseWheel != 0.0f) r.zoom = std::clamp(r.zoom * std::pow(1.15f, io.MouseWheel), 0.1f, 8.0f);
    if (active) {
        if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
            r.yaw += io.MouseDelta.x * 0.4f;
            r.pitch = std::clamp(r.pitch + io.MouseDelta.y * 0.3f, -30.0f, 85.0f);
        } else if (ImGui::IsMouseDown(ImGuiMouseButton_Middle)) {
            const float s = 0.0065f / std::max(r.zoom, 0.1f);
            r.panX = std::clamp(r.panX - io.MouseDelta.x * s, -5.0f, 5.0f);
            r.panY = std::clamp(r.panY + io.MouseDelta.y * s, -5.0f, 5.0f);
        }
    }

    // world -> screen through the preview's own matrix
    auto project = [&](V3 p, ImVec2& out) {
        const float x = vp[0] * p.x + vp[4] * p.y + vp[8] * p.z + vp[12];
        const float y = vp[1] * p.x + vp[5] * p.y + vp[9] * p.z + vp[13];
        const float w = vp[3] * p.x + vp[7] * p.y + vp[11] * p.z + vp[15];
        if (w <= 1e-4f) return false;
        out = ImVec2(origin.x + (x / w * 0.5f + 0.5f) * (float)pw, origin.y + (0.5f - y / w * 0.5f) * (float)ph);
        return true;
    };
    auto pxRadius = [&](V3 p, float rad) {
        ImVec2 a0, a1, a2;
        if (!project(p, a0) || !project(p + V3(rad, 0, 0), a1) || !project(p + V3(0, rad, 0), a2)) return 0.0f;
        return std::max(std::hypot(a1.x - a0.x, a1.y - a0.y), std::hypot(a2.x - a0.x, a2.y - a0.y));
    };
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->PushClipRect(origin, ImVec2(origin.x + (float)pw, origin.y + (float)ph), true);
    const ragdoll::Def& def = r.def;
    const bool have = r.defOk && parts.size() == def.particles.size();
    const ImU32 cBody = IM_COL32(80, 200, 255, 110), cBodyLine = IM_COL32(80, 200, 255, 220),
                cSel = IM_COL32(255, 170, 40, 230), cPart = IM_COL32(255, 255, 255, 200),
                cLink = IM_COL32(200, 200, 200, 120), cTarget = IM_COL32(90, 255, 120, 220),
                cLimit = IM_COL32(255, 230, 80, 220), cBone = IM_COL32(255, 255, 255, 140);
    if (r.showBones) {
        for (size_t i = 0; i < r.skel.bones.size(); ++i) {
            const int p = r.skel.bones[i].parent;
            if (p < 0 || r.bones.size() < (i + 1) * 16) continue;
            ImVec2 a0, a1;
            if (project(V3(r.bones[i * 16 + 12], r.bones[i * 16 + 13], r.bones[i * 16 + 14]), a0) &&
                project(V3(r.bones[(size_t)p * 16 + 12], r.bones[(size_t)p * 16 + 13], r.bones[(size_t)p * 16 + 14]), a1))
                dl->AddLine(a0, a1, cBone, 1.0f);
        }
    }
    if (have && r.showLinks)
        for (const auto& l : def.links) {
            ImVec2 a0, a1;
            if (project(parts[(size_t)l.first], a0) && project(parts[(size_t)l.second], a1))
                dl->AddLine(a0, a1, cLink, 1.0f);
        }
    if (have && r.showBodies)
        for (size_t b = 0; b < def.bodies.size(); ++b) {
            const ragdoll::Def::B& bd = def.bodies[b];
            if (bd.a < 0 || bd.b < 0) continue;
            ImVec2 a0, a1;
            if (!project(parts[(size_t)bd.a], a0) || !project(parts[(size_t)bd.b], a1)) continue;
            const float ra = pxRadius(parts[(size_t)bd.a], def.particles[(size_t)bd.a].radius);
            const float rb = pxRadius(parts[(size_t)bd.b], def.particles[(size_t)bd.b].radius);
            const bool sel = r.selKind == RagdollEd::Body && r.sel == (int)b;
            dl->AddLine(a0, a1, sel ? IM_COL32(255, 170, 40, 120) : cBody, std::max(2.0f, ra + rb));
            dl->AddLine(a0, a1, sel ? cSel : cBodyLine, sel ? 2.5f : 1.5f);
        }
    if (have && r.showParticles)
        for (size_t k = 0; k < parts.size(); ++k) {
            ImVec2 a0;
            if (!project(parts[k], a0)) continue;
            const float rp = pxRadius(parts[k], def.particles[k].radius);
            const bool sel = r.selKind == RagdollEd::Particle && r.sel == (int)k;
            dl->AddCircle(a0, std::max(2.0f, rp), sel ? cSel : cPart, 20, sel ? 2.0f : 1.0f);
            dl->AddCircleFilled(a0, 2.0f, sel ? cSel : cPart);
        }
    if (have && r.showTargets && r.mode == RagdollEd::Motorized && r.sim.hasTargets()) {
        const std::vector<V3>& t = r.sim.targets();
        for (size_t k = 0; k < t.size() && k < parts.size(); ++k) {
            ImVec2 a0, a1;
            if (!project(t[k], a0) || !project(parts[k], a1)) continue;
            dl->AddLine(ImVec2(a0.x - 3, a0.y), ImVec2(a0.x + 3, a0.y), cTarget, 1.5f);
            dl->AddLine(ImVec2(a0.x, a0.y - 3), ImVec2(a0.x, a0.y + 3), cTarget, 1.5f);
            dl->AddLine(a0, a1, IM_COL32(90, 255, 120, 70), 1.0f);
        }
    }
    // the selected joint: its swing limit (the allowed directions from the
    // pivot, in its parent's frame as it is now) and its axes - pitch (red),
    // yaw (green), roll (blue) - the colours its limits carry in the details
    if (have && (r.showLimits || r.showAxes) && r.selKind == RagdollEd::Body && r.sel >= 0 &&
        r.sel < (int)def.bodies.size() && r.sim.ready()) {
        const ragdoll::Def::B& bd = def.bodies[(size_t)r.sel];
        std::vector<float> fr;
        std::vector<V3> piv;
        if (!r.sim.started() || r.mode == RagdollEd::Animation) {
            ragdollsim::Sim probe;
            probe.build(def);
            probe.start(r.bones);
            probe.frames(fr, piv);
        } else {
            r.sim.frames(fr, piv);
        }
        const V3 pivot = parts[(size_t)bd.a];
        const float L = std::max(len3(parts[(size_t)bd.b] - pivot), 0.05f);
        auto arrow = [&](V3 dir, float length, ImU32 col, const char* label) {
            ImVec2 s0, s1;
            if (!project(pivot, s0) || !project(pivot + dir * length, s1)) return;
            dl->AddLine(s0, s1, col, 2.0f);
            const float dx = s1.x - s0.x, dy = s1.y - s0.y, l = std::max(std::hypot(dx, dy), 1e-3f);
            const float ux = dx / l, uy = dy / l, h = 7.0f;
            dl->AddTriangleFilled(s1, ImVec2(s1.x - ux * h - uy * h * 0.5f, s1.y - uy * h + ux * h * 0.5f),
                                  ImVec2(s1.x - ux * h + uy * h * 0.5f, s1.y - uy * h - ux * h * 0.5f), col);
            dl->AddText(ImVec2(s1.x + 4, s1.y - 7), col, label);
        };
        const ImU32 cPitch = ImGui::GetColorU32(kPitchCol), cYaw = ImGui::GetColorU32(kYawCol),
                    cRoll = ImGui::GetColorU32(kRollCol);
        if (bd.parent >= 0 && fr.size() >= (size_t)(bd.parent + 1) * 9) {
            const float* pf = &fr[(size_t)bd.parent * 9];
            V3 c, pa, ya;
            float hp, hy;
            r.sim.limitCone(r.sel, c, pa, ya, hp, hy);
            if (r.showLimits) {
                std::vector<ImVec2> ring;
                const int n = 48;
                for (int i = 0; i <= n; ++i) {
                    // walk the box [-hp,hp] x [-hy,hy] around its edge
                    const float t = (float)i / (float)n * 4.0f;
                    float u, v;
                    if (t < 1) u = -hp + 2 * hp * t, v = -hy;
                    else if (t < 2) u = hp, v = -hy + 2 * hy * (t - 1);
                    else if (t < 3) u = hp - 2 * hp * (t - 2), v = hy;
                    else u = -hp, v = hy - 2 * hy * (t - 3);
                    const V3 dir = mul3(pf, r.sim.swingDirection(r.sel, u, v));
                    ImVec2 sp;
                    if (project(pivot + dir * L, sp)) ring.push_back(sp);
                }
                ImVec2 sp;
                if (project(pivot, sp)) {
                    for (size_t i = 0; i + 1 < ring.size(); ++i) dl->AddLine(ring[i], ring[i + 1], cLimit, 1.5f);
                    for (size_t i = 0; i < ring.size(); i += ring.size() / 4 > 0 ? ring.size() / 4 : 1)
                        dl->AddLine(sp, ring[i], IM_COL32(255, 230, 80, 90), 1.0f);
                    ImVec2 cen;
                    if (project(pivot + mul3(pf, c) * L, cen)) dl->AddCircle(cen, 3.0f, cLimit);
                }
            }
            if (r.showAxes) {
                // + pitch / + yaw: where those limits swing the far end (at the
                // cone's centre); roll: the body's own axis, with a ring for the twist
                arrow(mul3(pf, pa), L * 0.6f, cPitch, "+pitch");
                if (!bd.hinge) arrow(mul3(pf, ya), L * 0.6f, cYaw, "+yaw");
                const V3 axis = (parts[(size_t)bd.b] - pivot) * (1.0f / L);
                arrow(axis, L * 1.15f, cRoll, "roll");
                const V3 u = mul3(pf, pa), w = V3(axis.y * u.z - axis.z * u.y, axis.z * u.x - axis.x * u.z,
                                                  axis.x * u.y - axis.y * u.x);
                ImVec2 prev;
                bool havePrev = false;
                for (int i = 0; i <= 24; ++i) {
                    const float ang = (float)i / 24.0f * 6.2831853f;
                    const V3 p = pivot + axis * (L * 0.5f) + (u * std::cos(ang) + w * std::sin(ang)) * (L * 0.18f);
                    ImVec2 sp;
                    if (!project(p, sp)) continue;
                    if (havePrev) dl->AddLine(prev, sp, cRoll, 1.5f);
                    prev = sp, havePrev = true;
                }
            }
        } else if (r.showAxes && fr.size() >= 9) {
            // the root: its own frame (x along it, the roll; y and z across)
            const float* f = &fr[(size_t)r.sel * 9];
            arrow(colv(f, 1), L * 0.6f, cPitch, "y");
            arrow(colv(f, 2), L * 0.6f, cYaw, "z");
            arrow(colv(f, 0), L * 1.15f, cRoll, "x (axis)");
        }
    }
    // the grab: the held point and the hand pulling it
    if (r.sim.grabbing()) {
        ImVec2 g0, g1;
        if (project(r.sim.grabPoint(), g0) && project(r.sim.grabTarget(), g1)) {
            dl->AddLine(g0, g1, IM_COL32(255, 80, 220, 230), 2.0f);
            dl->AddCircleFilled(g0, 4.0f, IM_COL32(255, 80, 220, 230));
            dl->AddCircle(g1, 6.0f, IM_COL32(255, 80, 220, 230), 16, 2.0f);
        }
    }
    dl->AddText(ImVec2(origin.x + scaled(8), origin.y + scaled(8)), ImGui::GetColorU32(ImGuiCol_TextDisabled),
                "LMB select  |  Ctrl+click punch  |  Ctrl+drag grab and pull  |  RMB rotate  |  MMB pan  |  "
                "wheel zoom");
    if (r.mode != RagdollEd::Animation) {
        char info[160];
        std::snprintf(info, sizeof(info), "t %.2f s  %s%s", (double)r.sim.age(),
                      r.sim.asleep() ? "asleep" : "awake",
                      r.mode == RagdollEd::Motorized ? (r.sim.motorScale() < 0.999f ? "  motors fading (down)" : "  motors on") : "");
        dl->AddText(ImVec2(origin.x + scaled(8), origin.y + (float)ph - scaled(22)),
                    ImGui::GetColorU32(ImGuiCol_Text), info);
    }
    dl->PopClipRect();

    // What is under the mouse: the nearest particle within its radius, else
    // the nearest body segment (and where along it).
    auto pick = [&](int& part, int& body, float& along) {
        const ImVec2 m = io.MousePos;
        part = body = -1;
        along = 0.0f;
        float bestD = 1e9f;
        for (size_t k = 0; k < parts.size(); ++k) {
            ImVec2 sp;
            if (!project(parts[k], sp)) continue;
            const float dd = std::hypot(sp.x - m.x, sp.y - m.y);
            const float reach = std::max(6.0f, pxRadius(parts[k], def.particles[k].radius));
            if (dd < reach && dd < bestD) bestD = dd, part = (int)k;
        }
        if (part >= 0) return;
        float bestBD = 1e9f;
        for (size_t bi = 0; bi < def.bodies.size(); ++bi) {
            const ragdoll::Def::B& bd = def.bodies[bi];
            ImVec2 a0, a1;
            if (bd.a < 0 || bd.b < 0 || !project(parts[(size_t)bd.a], a0) || !project(parts[(size_t)bd.b], a1))
                continue;
            const float vx = a1.x - a0.x, vy = a1.y - a0.y;
            const float l2 = std::max(vx * vx + vy * vy, 1e-6f);
            const float t = std::clamp(((m.x - a0.x) * vx + (m.y - a0.y) * vy) / l2, 0.0f, 1.0f);
            const float dd = std::hypot(a0.x + vx * t - m.x, a0.y + vy * t - m.y);
            const float ra = pxRadius(parts[(size_t)bd.a], def.particles[(size_t)bd.a].radius);
            const float rb = pxRadius(parts[(size_t)bd.b], def.particles[(size_t)bd.b].radius);
            const float reach = std::max(6.0f, ra + (rb - ra) * t);
            if (dd < reach && dd < bestBD) bestBD = dd, body = (int)bi, along = t;
        }
    };
    // The mouse on screen, back into the world at screen depth `ndcZ`.
    auto unproject = [&](float ndcZ, V3& out) {
        float inv[16];
        if (!invert4(vp, inv)) return false;
        const float x = (io.MousePos.x - origin.x) / (float)pw * 2.0f - 1.0f;
        const float y = 1.0f - (io.MousePos.y - origin.y) / (float)ph * 2.0f;
        const float wx = inv[0] * x + inv[4] * y + inv[8] * ndcZ + inv[12];
        const float wy = inv[1] * x + inv[5] * y + inv[9] * ndcZ + inv[13];
        const float wz = inv[2] * x + inv[6] * y + inv[10] * ndcZ + inv[14];
        const float ww = inv[3] * x + inv[7] * y + inv[11] * ndcZ + inv[15];
        if (std::fabs(ww) < 1e-8f) return false;
        out = V3(wx / ww, wy / ww, wz / ww);
        return true;
    };
    // Ctrl+press on the body remembers the point; what it becomes waits for the
    // mouse: released first, a PUNCH there along the mouse ray (the push speed);
    // dragged past ImGui's drag threshold, a GRAB - a pinch pulled with the
    // mouse, in a plane facing the view at the point's depth. In Animation
    // either one makes a ragdoll from the pose shown first.
    if (have && pressed && io.KeyCtrl && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        int part, body;
        float along;
        pick(part, body, along);
        if (part >= 0 || body >= 0) {
            r.grabA = part >= 0 ? part : def.bodies[(size_t)body].a;
            r.grabB = part >= 0 ? -1 : def.bodies[(size_t)body].b;
            r.grabT = part >= 0 ? 0.0f : along;
            r.ctrlPress = true;
        }
    }
    if (r.ctrlPress) {
        const bool dragged = ImGui::IsMouseDragging(ImGuiMouseButton_Left);
        const bool released = !ImGui::IsMouseDown(ImGuiMouseButton_Left);
        if (dragged || released) {
            r.ctrlPress = false;
            if (r.mode == RagdollEd::Animation) {
                r.mode = RagdollEd::Ragdoll;
                ragdollStart();
            }
            const std::vector<V3>& now = r.sim.particles();
            const V3 gp = r.grabB < 0 ? now[(size_t)r.grabA]
                                      : now[(size_t)r.grabA] * (1.0f - r.grabT) + now[(size_t)r.grabB] * r.grabT;
            const float cz = vp[2] * gp.x + vp[6] * gp.y + vp[10] * gp.z + vp[14];
            const float cw = vp[3] * gp.x + vp[7] * gp.y + vp[11] * gp.z + vp[15];
            const float z = cw > 1e-6f ? cz / cw : 0.5f;
            if (dragged) {
                r.grabZ = z;
                r.sim.setGrab(r.grabA, r.grabB, r.grabT, gp);
                r.grabbing = true;
            } else {
                // into the screen through the mouse: two depths on its ray
                V3 nearP, farP;
                if (unproject(z - 0.05f, nearP) && unproject(z, farP)) {
                    const V3 ray = farP - nearP;
                    const float len = std::sqrt(ray.x * ray.x + ray.y * ray.y + ray.z * ray.z);
                    if (len > 1e-6f) r.sim.applyPush(gp, ray * (r.pushSpeed / len));
                }
            }
        }
    }
    if (r.grabbing) {
        V3 target;
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && r.mode != RagdollEd::Animation) {
            if (unproject(r.grabZ, target)) r.sim.setGrab(r.grabA, r.grabB, r.grabT, target);
        } else {
            r.sim.releaseGrab();
            r.grabbing = false;
        }
    }

    // a plain click (no drag, no Ctrl) selects the nearest particle, else body
    if (have && hovered && !io.KeyCtrl && ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
        ImGui::GetMouseDragDelta(ImGuiMouseButton_Left).x * ImGui::GetMouseDragDelta(ImGuiMouseButton_Left).x +
                ImGui::GetMouseDragDelta(ImGuiMouseButton_Left).y * ImGui::GetMouseDragDelta(ImGuiMouseButton_Left).y <
            16.0f) {
        int part, body;
        float along;
        pick(part, body, along);
        if (part >= 0) r.selKind = RagdollEd::Particle, r.sel = part;
        else if (body >= 0) r.selKind = RagdollEd::Body, r.sel = body;
        else r.selKind = RagdollEd::None, r.sel = -1;
    }
}
