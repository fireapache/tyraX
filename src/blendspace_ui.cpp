// Tools > 2D Animation Blend Tool (docs/blend-spaces.md).
//
// The Unreal Engine Blend Space (2D) editor's counterpart for TyraX, drawn as
// a COMPASS seen from above: the model at the centre looking up, angle =
// the direction it moves in (+ = its right), distance from the centre = its
// speed. A .blendspace asset places some of an animated model's clips on it;
// the game blends the up-to-3 around its input (setAnimationMode /
// setBlendSpaceInput). The preview runs the SAME weight function as the
// console (blend_space_weights.hpp) and the engine's pose-blend rule
// (animmerge::poseMatricesBlend), so what blends here is what ships. The
// asset is a document of its own (Save / Revert / its own undo), not project
// data - the build bakes it into the model's .tskl.
#include "app.hpp"
#include "app_internal.hpp"

#include <imgui_internal.h>  // SeparatorEx: the toolbar's vertical rules

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "animedit.hpp"
#include "animmerge.hpp"
#include "blendspace.hpp"
#include "glbparser.hpp"
#include "theme.hpp"

namespace fs = std::filesystem;
namespace bsw = Tyra::blendspace;

struct App::BlendSpaceEd {
    std::string assetRel;
    blendspace::Asset asset;
    std::string savedJson, lastJson;
    std::vector<std::string> undo, redo;
    bool loaded = false;
    std::string modelSel;  // the toolbar's Model: whose blend spaces the list shows
    std::string status;
    // the model as the GAME sees it (imports and clip edits applied)
    glbparser::Skel model;
    bool modelOk = false;
    std::string modelFor, modelError;
    std::vector<std::string> clipNames;
    int sel = -1;  // the selected sample row, -1 = the blend space itself
    // the preview: the point the user put on the compass (the target), the
    // smoothed point the blend reads, the shared phase and own-time clocks
    float dir = 0.0f, speed = 0.0f;
    bsw::Point point{0.0f, 0.0f};
    bool pointSet = false;
    float phase = 0.0f;
    std::vector<float> ownClock;
    float travel[2] = {0.0f, 0.0f};  // the floor's scroll, metres
    // the speed the body moves at on or past the ring (blendspace::
    // previewMoveSpeed); < 0 = the clips' own (authoredSpeed)
    float moveSpeed = -1.0f;
    float authoredSpeed = 0.0f, moveAt = 0.0f;  // this frame's, for the details
    bool playing = true;
    float playSpeed = 1.0f;
    bsw::Weights shown;
    // view
    float yaw = 30.0f, pitch = 14.0f, zoom = 1.0f, panX = 0.0f, panY = 0.0f;
    bool showMesh = true, wire = false, showWeights = true, showFloor = true;
    float leftW = 260.0f, rightW = 300.0f;  // side panels, unscaled px
    float previewH = 0.55f;                 // the preview's share of the middle column
    std::string light;
    // the compass's drag state; its scale is frozen while a sample is dragged
    // (it follows the fastest sample - the dragged one included)
    int dragSample = -1;
    bool dragPoint = false;
    float dragMax = 0.0f;
    // New...
    std::string newModel, newName = "locomotion";
    // scratch
    std::vector<float> nodes;
    std::vector<std::vector<float>> pos, nrm;
    float center[3] = {0, 0.9f, 0};
    float radius = 1.0f;
};

namespace {

ImVec4 fadeTo(const ImVec4& c, float a) { return ImVec4(c.x, c.y, c.z, a); }
ImVec4 mixTo(const ImVec4& a, const ImVec4& b, float t) {
    return ImVec4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t);
}
ImU32 u32(const ImVec4& c) { return ImGui::GetColorU32(c); }
ImVec4 labelCol() {
    const theme::Semantics& sm = theme::semantics();
    return mixTo(sm.text, sm.textDim, 0.45f);
}
void vrule() {
    ImGui::SameLine();
    ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
    ImGui::SameLine();
}
void wrappedTooltip(const char* text) {
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 26.0f);
    ImGui::TextUnformatted(text);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
}
void tip(const char* text) {
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip | ImGuiHoveredFlags_AllowWhenDisabled))
        wrappedTooltip(text);
}
// A label's help: shown once the cursor RESTS on the label (still, then the
// normal hover delay) - the labels carry it instead of "(?)" markers.
void labelHelp(const char* text) {
    if (text && ImGui::IsItemHovered(ImGuiHoveredFlags_Stationary | ImGuiHoveredFlags_DelayNormal |
                                     ImGuiHoveredFlags_AllowWhenDisabled))
        wrappedTooltip(text);
}
void section(const char* title, const char* help = nullptr) {
    ImGui::Dummy(ImVec2(0.0f, ImGui::GetStyle().ItemSpacing.y * 0.6f));
    const float right = ImGui::GetCursorScreenPos().x + ImGui::GetContentRegionAvail().x;
    ImGui::TextUnformatted(title);
    labelHelp(help);
    const ImVec2 mn = ImGui::GetItemRectMin(), mx = ImGui::GetItemRectMax();
    const float y = std::floor(0.5f * (mn.y + mx.y)) + 0.5f;
    const float x0 = mx.x + ImGui::GetStyle().ItemSpacing.x;
    if (right > x0)
        ImGui::GetWindowDrawList()->AddLine(ImVec2(x0, y), ImVec2(right, y), u32(theme::semantics().border));
}
void prop(const char* label, float labelW, const char* help = nullptr) {
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(labelCol(), "%s", label);
    labelHelp(help);
    ImGui::SameLine();
    const float x = ImGui::GetCursorStartPos().x + labelW;
    if (ImGui::GetCursorPosX() < x) ImGui::SetCursorPosX(x);
    ImGui::SetNextItemWidth(-FLT_MIN);
}
bool tintButton(const char* label, const ImVec4& col, const ImVec2& size = ImVec2(0, 0)) {
    const theme::Semantics& sm = theme::semantics();
    ImGui::PushStyleColor(ImGuiCol_Button, fadeTo(col, 0.24f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, fadeTo(col, 0.38f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, fadeTo(col, 0.52f));
    ImGui::PushStyleColor(ImGuiCol_Border, fadeTo(col, 0.55f));
    ImGui::PushStyleColor(ImGuiCol_Text, sm.text);
    const bool clicked = ImGui::Button(label, size);
    ImGui::PopStyleColor(5);
    return clicked;
}
bool nameCombo(const char* label, std::string& cur, const std::vector<std::string>& names) {
    bool changed = false;
    if (ImGui::BeginCombo(label, cur.empty() ? "(none)" : cur.c_str())) {
        for (const std::string& n : names)
            if (ImGui::Selectable(n.c_str(), n == cur) && n != cur) cur = n, changed = true;
        ImGui::EndCombo();
    }
    return changed;
}

// The graph the samples sit on: the compass (up = where the model looks,
// + direction = its right, speed outward) or a GRID (direction across, speed
// up, the plot between lo and hi; points outside it are clamped to its edges).
struct Graph {
    bool grid = false;
    ImVec2 c;              // compass: its centre
    float r = 1.0f;        // compass: pixels at maxSpeed
    float maxSpeed = 1.0f;
    ImVec2 lo, hi;         // grid: the plot's corners
    bsw::Grid lines;
    ImVec2 at(float dirDeg, float speed) const {
        if (grid) {
            const float u = std::clamp((dirDeg - lines.dirMin) / (lines.dirMax - lines.dirMin), 0.0f, 1.0f);
            const float v = std::clamp((speed - lines.speedMin) / (lines.speedMax - lines.speedMin), 0.0f, 1.0f);
            return ImVec2(lo.x + (hi.x - lo.x) * u, hi.y - (hi.y - lo.y) * v);
        }
        const float a = dirDeg * 0.017453292f, k = speed / maxSpeed * r;
        return ImVec2(c.x + std::sin(a) * k, c.y - std::cos(a) * k);
    }
    void of(ImVec2 p, float* dirDeg, float* speed) const {
        if (grid) {
            const float u = std::clamp((p.x - lo.x) / std::max(1.0f, hi.x - lo.x), 0.0f, 1.0f);
            const float v = std::clamp((hi.y - p.y) / std::max(1.0f, hi.y - lo.y), 0.0f, 1.0f);
            *dirDeg = lines.dirMin + (lines.dirMax - lines.dirMin) * u;
            *speed = lines.speedMin + (lines.speedMax - lines.speedMin) * v;
            return;
        }
        const float dx = p.x - c.x, dy = c.y - p.y;
        *speed = std::sqrt(dx * dx + dy * dy) / r * maxSpeed;
        *dirDeg = *speed > 1e-6f ? std::atan2(dx, dy) * 57.29578f : 0.0f;
    }
    // grid: line k's direction / speed
    float column(int k) const { return lines.dirMin + (lines.dirMax - lines.dirMin) * (float)k / (float)(lines.columns - 1); }
    float row(int k) const { return lines.speedMin + (lines.speedMax - lines.speedMin) * (float)k / (float)(lines.rows - 1); }
};

float snapTo(float v, float step) { return std::round(v / step) * step; }

// The Layout switch: ui::Segmented's look (a field track, a surface pill) with
// the pill SLIDING to the option picked. An option that cannot be picked is
// greyed and hovering it shows `why` - what is in the way and what would make
// it available. Each option is a button named by its label (--ui-script).
// Returns true when *selected changed.
bool slideSwitch(const char* id, const char* const* labels, int count, int* selected, const bool* enabled,
                 const char* const* why) {
    const theme::Semantics& sm = theme::semantics();
    const ImGuiStyle& st = ImGui::GetStyle();
    const float h = ImGui::GetFrameHeight(), pad = 3.0f, r = st.FrameRounding;
    const float w = std::max(ImGui::GetContentRegionAvail().x, (float)count * h);
    ImGui::PushID(id);
    const ImVec2 base = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(base, ImVec2(base.x + w, base.y + h), u32(sm.field), r);
    dl->AddRect(base, ImVec2(base.x + w, base.y + h), u32(sm.border), r);
    const float sw = (w - pad * 2.0f) / (float)count;
    // the pill eases toward the picked option (it slides, not jumps)
    ImGuiStorage* store = ImGui::GetStateStorage();
    const ImGuiID key = ImGui::GetID("##pill");
    float at = store->GetFloat(key, (float)*selected);
    at += ((float)*selected - at) * std::min(1.0f, ImGui::GetIO().DeltaTime * 14.0f);
    if (std::fabs(at - (float)*selected) < 0.002f) at = (float)*selected;
    store->SetFloat(key, at);
    const float px = base.x + pad + at * sw;
    dl->AddRectFilled(ImVec2(px, base.y + pad), ImVec2(px + sw, base.y + h - pad), u32(sm.surface), r - pad * 0.5f);
    bool changed = false;
    for (int i = 0; i < count; ++i) {
        const float x = base.x + pad + sw * (float)i;
        ImGui::SetCursorScreenPos(ImVec2(x, base.y + pad));
        ImGui::BeginDisabled(!enabled[i]);
        if (ImGui::InvisibleButton(labels[i], ImVec2(sw, h - pad * 2.0f)) && *selected != i) {
            *selected = i;
            changed = true;
        }
        ImGui::EndDisabled();
        const bool on = *selected == i;
        const float t = theme::hoverAnim(ImGui::GetItemID(), enabled[i] && !on && ImGui::IsItemHovered(), 12.0f);
        if (t > 0.0f && !on)
            dl->AddRectFilled(ImVec2(x, base.y + pad), ImVec2(x + sw, base.y + h - pad), u32(fadeTo(sm.surface, 0.5f * t)),
                              r - pad * 0.5f);
        if (!enabled[i] && why && why[i] &&
            ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip | ImGuiHoveredFlags_AllowWhenDisabled))
            wrappedTooltip(why[i]);
        const ImVec2 ts = ImGui::CalcTextSize(labels[i]);
        const ImVec4 col = !enabled[i] ? fadeTo(sm.textDim, 0.45f) : (on ? sm.text : sm.textDim);
        dl->AddText(ImVec2(x + (sw - ts.x) * 0.5f, base.y + (h - ts.y) * 0.5f), u32(col), labels[i]);
    }
    ImGui::PopID();
    ImGui::SetCursorScreenPos(base);
    ImGui::Dummy(ImVec2(w, h));
    return changed;
}

}  // namespace

bool App::openBlendSpaceEditor(const std::string& rel) {
    if (!bs_) bs_ = std::make_shared<BlendSpaceEd>();
    showBlendSpaceEditor_ = true;
    const std::string ext = fs::path(rel).extension().string();
    if (ext == ".glb" || ext == ".fbx") {
        // an animated model: its first blend space, or the New... popup on it
        for (const std::string& a : blendspace::listAssets(project_.dir)) {
            blendspace::Asset as;
            std::string err;
            if (blendspace::load(project_.filePath(a), as, err) && as.model == rel) return blendSpaceLoad(a);
        }
        bs_->newModel = rel;
        bs_->status = "No blend space for " + rel + " yet: make one with New...";
        return true;
    }
    return blendSpaceLoad(rel);
}

void App::blendSpaceClose(const std::string& keepModel) {
    if (!bs_) bs_ = std::make_shared<BlendSpaceEd>();
    BlendSpaceEd& b = *bs_;
    const float yaw = b.yaw, pitch = b.pitch, zoom = b.zoom, leftW = b.leftW, rightW = b.rightW;
    b = BlendSpaceEd();
    b.yaw = yaw, b.pitch = pitch, b.zoom = zoom, b.leftW = leftW, b.rightW = rightW;
    b.modelSel = b.newModel = keepModel;
}

bool App::blendSpaceLoad(const std::string& assetRel) {
    if (!bs_) bs_ = std::make_shared<BlendSpaceEd>();
    BlendSpaceEd& b = *bs_;
    blendSpaceClose(b.modelSel);
    b.assetRel = assetRel;
    std::string err;
    if (!blendspace::load(project_.filePath(assetRel), b.asset, err)) {
        b.status = "Cannot open " + assetRel + ": " + err;
        return false;
    }
    b.modelSel = b.newModel = b.asset.model;
    b.savedJson = b.lastJson = blendspace::serialize(b.asset);
    b.loaded = true;
    b.status = "Opened " + assetRel;
    return true;
}

void App::blendSpaceSelectModel(const std::string& modelRel) {
    if (!bs_) bs_ = std::make_shared<BlendSpaceEd>();
    BlendSpaceEd& b = *bs_;
    if (b.loaded && b.asset.model == modelRel) return;  // already on one of its blend spaces
    std::vector<std::string> warnings;
    const auto spaces = blendspace::forModel(project_.dir, modelRel, warnings);
    if (!spaces.empty() && blendSpaceLoad(spaces.front().first)) return;
    blendSpaceClose(modelRel);
    b.status = "No blend space on " + modelRel + " yet - New... makes one.";
}

void App::drawBlendSpaceWindow() {
    if (!showBlendSpaceEditor_ || !hasProject_) return;
    if (!bs_) bs_ = std::make_shared<BlendSpaceEd>();
    BlendSpaceEd& b = *bs_;
    ImGui::SetNextWindowSize(ImVec2(scaled(1120), scaled(720)), ImGuiCond_FirstUseEver);
    const bool dirty = b.loaded && b.lastJson != b.savedJson;
    char title[256];
    std::snprintf(title, sizeof(title), "2D Animation Blend Tool%s###BlendSpaceTool", dirty ? " *" : "");
    if (!ImGui::Begin(title, &showBlendSpaceEditor_)) {
        ImGui::End();
        return;
    }
    const theme::Semantics& sm = theme::semantics();

    // The model the samples name, parsed as the game sees it (re-read when
    // the asset points at another one).
    if (b.loaded && b.modelFor != b.asset.model) {
        b.modelFor = b.asset.model;
        b.modelOk = blendspace::gameSkel(project_, b.asset.model, b.model, b.modelError);
        b.clipNames.clear();
        if (b.modelOk) {
            for (const glbparser::SkelClip& c : b.model.clips) b.clipNames.push_back(c.name);
            const float h = std::max(0.5f, b.model.max[1] - b.model.min[1]);
            b.center[0] = 0.5f * (b.model.min[0] + b.model.max[0]);
            b.center[1] = 0.5f * (b.model.min[1] + b.model.max[1]);
            b.center[2] = 0.5f * (b.model.min[2] + b.model.max[2]);
            b.radius = h * 0.75f;
        }
    }

    // --- the toolbar: the document, then the preview and the view -------------
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(labelCol(), "Model");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(scaled(190));
    const std::string modelsDir = "res/models/";
    const std::string modelShown =
        b.modelSel.rfind(modelsDir, 0) == 0 ? b.modelSel.substr(modelsDir.size()) : b.modelSel;
    if (ImGui::BeginCombo("##Model", b.modelSel.empty() ? "(pick one)" : modelShown.c_str())) {
        for (const std::string& m : listAnimatedModelFiles()) {
            const std::string rel = modelsDir + m;
            if (ImGui::Selectable(m.c_str(), rel == b.modelSel) && rel != b.modelSel) blendSpaceSelectModel(rel);
        }
        ImGui::EndCombo();
    }
    tip("The animated model whose blend spaces the list beside it shows. Dropping a model - or a "
        ".blendspace, which picks its model too - from the Asset Browser anywhere on this window "
        "picks it.");
    ImGui::SameLine();
    ImGui::TextColored(labelCol(), "Blend space");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(scaled(230));
    const std::string spaceShown = b.loaded             ? fs::path(b.assetRel).filename().string()
                                   : b.modelSel.empty() ? "(pick a model)"
                                                        : "(none)";
    if (ImGui::BeginCombo("##BlendSpace", spaceShown.c_str())) {
        std::vector<std::string> warnings;
        const auto spaces = blendspace::forModel(project_.dir, b.modelSel, warnings);
        for (const auto& [rel, a] : spaces) {
            ImGui::PushID(rel.c_str());
            if (ImGui::Selectable(fs::path(rel).filename().string().c_str(), rel == b.assetRel) &&
                rel != b.assetRel)
                blendSpaceLoad(rel);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s - \"%s\" in game", rel.c_str(), a.name.c_str());
            ImGui::PopID();
        }
        if (spaces.empty())
            ImGui::TextDisabled("%s", b.modelSel.empty() ? "pick a model first" : "none yet - New... makes one");
        ImGui::EndCombo();
    }
    tip("The model's blend spaces - every .blendspace under res/ that names it. The one picked is edited below.");
    ImGui::SameLine();
    ImGui::BeginDisabled(!b.loaded);
    const bool ctrlS = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) &&
                       ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S, false);
    if ((dirty ? tintButton("Save", sm.accent) : ImGui::Button("Save")) || (ctrlS && b.loaded)) {
        std::string err;
        if (blendspace::save(project_.filePath(b.assetRel), b.asset, err)) {
            b.savedJson = b.lastJson;
            b.status = "Saved " + b.assetRel + " - the next build bakes it into the model.";
        } else {
            b.status = err;
        }
    }
    tip("Writes the asset (Ctrl+S). The next build resolves its clips and bakes it into the model's .tskl.");
    ImGui::SameLine();
    if (ImGui::Button("Revert")) blendSpaceLoad(b.assetRel);
    tip("Throws away the unsaved edits and reloads the file.");
    vrule();
    ImGui::BeginDisabled(b.undo.empty());
    if (ImGui::Button("Undo")) {
        b.redo.push_back(b.lastJson);
        std::string err;
        blendspace::parse(b.undo.back(), b.asset, err);
        b.undo.pop_back();
        b.lastJson = blendspace::serialize(b.asset);
    }
    ImGui::EndDisabled();
    tip("Undoes the last edit to the asset.");
    ImGui::SameLine();
    ImGui::BeginDisabled(b.redo.empty());
    if (ImGui::Button("Redo")) {
        b.undo.push_back(b.lastJson);
        std::string err;
        blendspace::parse(b.redo.back(), b.asset, err);
        b.redo.pop_back();
        b.lastJson = blendspace::serialize(b.asset);
    }
    ImGui::EndDisabled();
    tip("Redoes the edit Undo took back.");
    ImGui::EndDisabled();
    vrule();
    if (ImGui::Button("New...")) {
        if (!b.modelSel.empty()) b.newModel = b.modelSel;
        ImGui::OpenPopup("New blend space");
    }
    tip("Makes an empty blend space for an animated model: drag its clips onto the compass.");
    ImGui::SameLine();
    ImGui::BeginDisabled(!b.loaded);
    if (ImGui::Button("Delete")) ImGui::OpenPopup("Delete blend space?");
    ImGui::EndDisabled();
    tip("Deletes this blend space's file - it asks first. The next build drops it from the model.");
    if (ImGui::BeginPopupModal("Delete blend space?", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("Delete %s?", b.assetRel.c_str());
        ImGui::TextDisabled("The next build drops \"%s\" from %s: a game's setAnimationMode on it\n"
                            "answers false.",
                            b.asset.name.c_str(), b.asset.model.c_str());
        ImGui::TextDisabled("The file is removed from res/. This cannot be undone.");
        ImGui::Separator();
        if (tintButton("Delete", sm.danger, ImVec2(scaled(120), 0))) {
            const std::string rel = b.assetRel, model = b.asset.model;
            deleteAssetFile(rel);
            assetsChanged();
            blendSpaceClose(model);
            blendSpaceSelectModel(model);  // the model's next blend space, if it has one
            b.status = "Deleted " + rel + (b.loaded ? " - opened " + b.assetRel : "");
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(scaled(120), 0))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    if (ImGui::BeginPopup("New blend space")) {
        ImGui::Dummy(ImVec2(scaled(360), 0.0f));
        section("New blend space",
                "An empty compass for an animated model, saved beside it as <model>_<name>.blendspace. "
                "Its name is what a game asks for (setAnimationMode).");
        const float lw = scaled(110);
        std::vector<std::string> models;
        for (const std::string& m : listAnimatedModelFiles()) models.push_back("res/models/" + m);
        prop("Model", lw, "An animated (skinned) model.");
        nameCombo("##NewModel", b.newModel, models);
        prop("Name in game", lw, "What setAnimationMode(..., \"name\") asks for. At most 31 characters.");
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%s", b.newName.c_str());
        if (ImGui::InputText("##NewName", buf, sizeof(buf))) b.newName = buf;
        ImGui::BeginDisabled(b.newModel.empty() || b.newName.empty());
        if (tintButton("Create", sm.accent)) {
            blendspace::Asset a;
            a.model = b.newModel;
            a.name = b.newName;
            a.smoothing = 0.15f;
            const fs::path m(b.newModel);
            const std::string rel =
                (m.parent_path() / (m.stem().string() + "_" + b.newName + ".blendspace")).generic_string();
            std::string err;
            std::error_code ec;
            if (fs::exists(project_.filePath(rel), ec)) {
                b.status = rel + " exists already - opened it.";
                blendSpaceLoad(rel);
            } else if (blendspace::save(project_.filePath(rel), a, err)) {
                blendSpaceLoad(rel);
                b.status = "Created " + rel + ": drag the model's clips onto the compass.";
            } else {
                b.status = err;
            }
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndDisabled();
        ImGui::EndPopup();
    }
    // the preview's controls: on the document's row while they fit (Model and
    // Blend space made it long), else a row of their own
    {
        const ImGuiStyle& st = ImGui::GetStyle();
        const float need = ImGui::CalcTextSize("Pause").x + ImGui::CalcTextSize("Show").x +
                           ImGui::CalcTextSize("Reset view").x + scaled(70) + st.FramePadding.x * 6.0f +
                           st.ItemSpacing.x * 5.0f;
        ImGui::SameLine();
        if (ImGui::GetContentRegionAvail().x < need) {
            ImGui::NewLine();
        } else {
            ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
            ImGui::SameLine();
        }
    }
    if (ImGui::Button(b.playing ? "Pause" : "Play")) b.playing = !b.playing;
    tip("Plays the preview: the phase advances by the distance the body covers (the asset's Stride) - "
        "at the Move speed on or past the ring, at the point's share of it inside - the own-time "
        "samples on their clocks.");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(scaled(70));
    if (ImGui::BeginCombo("##PlaySpeed", b.playSpeed == 0.5f ? "0.5x" : b.playSpeed == 2.0f ? "2x" : "1x")) {
        for (float s : {0.5f, 1.0f, 2.0f}) {
            char lbl[8];
            std::snprintf(lbl, sizeof(lbl), s == 0.5f ? "0.5x" : s == 2.0f ? "2x" : "1x");
            if (ImGui::Selectable(lbl, b.playSpeed == s)) b.playSpeed = s;
        }
        ImGui::EndCombo();
    }
    tip("The preview's speed.");
    ImGui::SameLine();
    if (ImGui::Button("Show")) ImGui::OpenPopup("##bs_show");
    tip("What the preview and the compass draw.");
    if (ImGui::BeginPopup("##bs_show")) {
        ImGui::Checkbox("Mesh", &b.showMesh);
        ImGui::Checkbox("Wireframe", &b.wire);
        ImGui::Checkbox("Moving floor", &b.showFloor);
        ImGui::Checkbox("Weights on the compass", &b.showWeights);
        ImGui::EndPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset view")) b.yaw = 30.0f, b.pitch = 14.0f, b.zoom = 1.0f, b.panX = b.panY = 0.0f;
    tip("The preview camera back to its start.");
    if (!b.status.empty()) {
        ImGui::SameLine();
        ImGui::TextDisabled("%s", b.status.c_str());
    }

    // The Asset Browser's models and .blendspace files drop anywhere on the
    // window: a model is picked (its first blend space opens), a blend space
    // opens with its model. Only those two light the window up.
    if (ImGui::BeginDragDropTargetCustom(ImGui::GetCurrentWindow()->Rect(), ImGui::GetID("##bs_drop"))) {
        if (const ImGuiPayload* pl = ImGui::AcceptDragDropPayload("TYRAX_ASSET_MODEL")) {
            const std::string rel((const char*)pl->Data);
            blendSpaceSelectModel(rel);
        }
        const ImGuiPayload* peek = ImGui::GetDragDropPayload();
        if (peek && peek->IsDataType("TYRAX_ASSET_FILE") &&
            fs::path((const char*)peek->Data).extension() == ".blendspace")
            if (const ImGuiPayload* pl = ImGui::AcceptDragDropPayload("TYRAX_ASSET_FILE")) {
                const std::string rel((const char*)pl->Data);
                blendSpaceLoad(rel);
            }
        ImGui::EndDragDropTarget();
    }

    if (!b.loaded) {
        ImGui::Spacing();
        ImGui::TextWrapped("Pick a model and one of its blend spaces above, drop a model or a .blendspace "
                           "here from the Asset Browser, or make one with New... - a .blendspace places an "
                           "animated model's clips on a compass of direction and speed.");
        ImGui::End();
        return;
    }

    // --- the blend at the preview point ----------------------------------------
    const std::vector<bsw::Sample> samples = blendspace::weightSamples(b.asset);
    // Pause freezes the clips (phase, own clocks, the floor) but not the
    // input: the preview point eases on real time, so a paused pose still
    // follows a dragged point - and it chases the input clamped to the
    // fastest ring, exactly as the engine does (blendspace::inputPoint).
    const float dt = b.playing ? ImGui::GetIO().DeltaTime * b.playSpeed : 0.0f;
    {
        const bsw::Point target = bsw::inputPoint(samples.data(), (int)samples.size(), b.dir, b.speed);
        b.point = b.pointSet ? bsw::smooth(b.point, target, ImGui::GetIO().DeltaTime, b.asset.smoothing)
                             : target;
        b.pointSet = true;
        float d = 0.0f, s = 0.0f;
        bsw::fromPoint(b.point, &d, &s);
        b.shown = bsw::weightsFor(samples.data(), (int)samples.size(), b.asset.grid ? &b.asset.lines : nullptr, d, s);
        // the phase: by distance with a stride, else at the clips' rate
        auto clipOf = [&](int row) {
            const auto it = std::find(b.clipNames.begin(), b.clipNames.end(), b.asset.samples[(size_t)row].clip);
            return it == b.clipNames.end() ? -1 : (int)(it - b.clipNames.begin());
        };
        // the speed the body really moves at (the phase, the floor): the
        // compass ends just past the ring, a running body is far past it
        std::vector<float> cycles;
        for (size_t i = 0; i < b.asset.samples.size(); ++i) {
            const int ci = clipOf((int)i);
            if (ci >= 0 && !b.asset.samples[i].own) cycles.push_back(b.model.clips[(size_t)ci].duration);
        }
        b.authoredSpeed = blendspace::authoredSpeed(b.asset.stride, cycles.data(), (int)cycles.size());
        const float moveAt = blendspace::previewMoveSpeed(
            b.asset, b.speed, b.moveSpeed >= 0.0f ? b.moveSpeed : b.authoredSpeed);
        b.moveAt = moveAt;
        if (dt > 0.0f && b.modelOk) {
            float rate = 0.0f;
            if (b.asset.stride > 0.0f) {
                rate = moveAt / b.asset.stride;  // by distance, as the engine
            } else {
                float wSum = 0.0f;
                for (int k = 0; k < b.shown.count; ++k) {
                    const int row = b.shown.sample[k], ci = clipOf(row);
                    if (ci < 0 || b.asset.samples[(size_t)row].own) continue;
                    const float dur = b.model.clips[(size_t)ci].duration;
                    if (dur > 0.0f) rate += b.shown.weight[k] / dur, wSum += b.shown.weight[k];
                }
                rate = wSum > 0.0f ? rate / wSum : 0.0f;
            }
            b.phase += dt * rate;
            b.phase -= std::floor(b.phase);
            b.ownClock.resize(b.asset.samples.size(), 0.0f);
            for (size_t i = 0; i < b.asset.samples.size(); ++i) {
                const int ci = clipOf((int)i);
                if (ci < 0 || !b.asset.samples[i].own) continue;
                const float dur = b.model.clips[(size_t)ci].duration;
                if (dur > 0.0f) b.ownClock[i] = std::fmod(b.ownClock[i] + dt, dur);
            }
            // the floor moves under the body, opposite the motion, in the
            // model's frame (glTF: +Z its front, +X its LEFT - the right is -X)
            const float a = b.dir * 0.017453292f;
            b.travel[0] += std::sin(a) * moveAt * dt;
            b.travel[1] -= std::cos(a) * moveAt * dt;
        }
        if (b.modelOk) {
            int clips[3];
            float times[3], weights[3];
            for (int k = 0; k < b.shown.count; ++k) {
                const int row = b.shown.sample[k], ci = clipOf(row);
                clips[k] = ci;
                weights[k] = ci < 0 ? 0.0f : b.shown.weight[k];
                const float dur = ci < 0 ? 0.0f : b.model.clips[(size_t)ci].duration;
                times[k] = b.asset.samples[(size_t)row].own && (size_t)row < b.ownClock.size()
                               ? b.ownClock[(size_t)row]
                               : b.phase * dur;
            }
            animmerge::poseMatricesBlend(b.model, clips, times, weights, b.shown.count, b.nodes);
            animmerge::skinParts(b.model, b.nodes, b.pos, b.nrm);
        }
    }

    // --- three columns: samples + clips | preview over compass | details -------
    const float unit = std::max(scaled(1.0f), 1e-3f);
    const float splitW = scaled(6), minSide = scaled(150), minMid = scaled(260);
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    b.leftW = std::clamp(b.leftW * unit, minSide, std::max(minSide, avail.x - minMid - minSide - 2 * splitW)) / unit;
    b.rightW = std::clamp(b.rightW * unit, minSide, std::max(minSide, avail.x - b.leftW * unit - minMid - 2 * splitW)) / unit;
    const float leftW = b.leftW * unit, rightW = b.rightW * unit;
    const float midW = std::max(minMid, avail.x - leftW - rightW - 2 * splitW);
    auto splitter = [&](const char* id, float& w, float dir) {
        ImGui::SameLine(0, 0);
        ImGui::InvisibleButton(id, ImVec2(splitW, avail.y));
        const bool hot = ImGui::IsItemHovered() || ImGui::IsItemActive();
        if (hot) ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
        if (ImGui::IsItemActive()) w += dir * ImGui::GetIO().MouseDelta.x / unit;
        const ImVec2 a = ImGui::GetItemRectMin(), c = ImGui::GetItemRectMax();
        const float x = 0.5f * (a.x + c.x);
        ImGui::GetWindowDrawList()->AddLine(ImVec2(x, a.y), ImVec2(x, c.y),
                                            ImGui::GetColorU32(hot ? ImGuiCol_SeparatorActive : ImGuiCol_Separator),
                                            hot ? 2.0f : 1.0f);
        ImGui::SameLine(0, 0);
    };

    // left: the samples, then the model's clips to drag onto the compass
    ImGui::BeginChild("##bs_left", ImVec2(leftW, avail.y), ImGuiChildFlags_Borders);
    section("Samples", "Every clip placed on the compass. Click one to edit it on the right; Delete removes it.");
    for (size_t i = 0; i < b.asset.samples.size(); ++i) {
        const blendspace::AssetSample& s = b.asset.samples[i];
        char row[128];
        if (s.speed < bsw::kRingEpsilon)
            std::snprintf(row, sizeof(row), "%s##bs_row%zu", s.clip.c_str(), i);
        else
            std::snprintf(row, sizeof(row), "%s##bs_row%zu", s.clip.c_str(), i);
        if (ImGui::Selectable(row, b.sel == (int)i)) b.sel = b.sel == (int)i ? -1 : (int)i;
        ImGui::SameLine();
        const bool centre = !b.asset.grid && s.speed < bsw::kRingEpsilon;
        ImGui::TextDisabled(centre ? "centre%s" : "%s", "");
        if (!centre) {
            ImGui::SameLine();
            ImGui::TextDisabled("%g deg  %.2f m/s%s", (double)s.dir, (double)s.speed, s.own ? "  own" : "");
        } else if (s.own) {
            ImGui::SameLine();
            ImGui::TextDisabled("own");
        }
    }
    if (b.asset.samples.empty()) ImGui::TextDisabled("None yet: drag a clip below onto the compass.");
    section("Clips", "The model's clips as the game names them (renames and imports applied). Drag one "
                     "onto the compass to place it; it snaps to 45 deg and 0.5 m/s (hold Shift for free).");
    if (!b.modelOk) ImGui::TextColored(sm.danger, "%s: %s", b.asset.model.c_str(), b.modelError.c_str());
    for (size_t i = 0; i < b.clipNames.size(); ++i) {
        const std::string& n = b.clipNames[i];
        if (ImGui::GetContentRegionAvail().x < ImGui::CalcTextSize(n.c_str()).x + scaled(24)) ImGui::NewLine();
        ImGui::SmallButton(n.c_str());
        if (ImGui::BeginDragDropSource()) {
            ImGui::SetDragDropPayload("BS_CLIP", n.c_str(), n.size() + 1);
            ImGui::Text("%s", n.c_str());
            ImGui::EndDragDropSource();
        }
        tip("Drag onto the compass to place this clip there.");
        ImGui::SameLine();
    }
    ImGui::NewLine();
    ImGui::EndChild();
    splitter("##bs_split_left", b.leftW, 1.0f);

    // middle: the preview over the compass
    ImGui::BeginChild("##bs_mid", ImVec2(midW, avail.y));
    {
        const ImVec2 mid = ImGui::GetContentRegionAvail();
        const int pw = (int)std::max(scaled(160), mid.x);
        const int ph = (int)std::max(scaled(140), mid.y * b.previewH);
        Viewport::RagdollPreviewDesc d;
        d.key = b.asset.model;
        d.skel = b.modelOk ? &b.model : nullptr;
        d.positions = &b.pos;
        d.normals = &b.nrm;
        d.drawMesh = b.showMesh && b.modelOk;
        d.wireframe = b.wire;
        std::memcpy(d.center, b.center, sizeof(d.center));
        d.radius = b.radius;
        d.floorY = 0.0f;
        d.angleDeg = b.yaw, d.pitchDeg = b.pitch, d.zoom = b.zoom, d.panX = b.panX, d.panY = b.panY;
        d.light = previewLight(b.light);
        // the moving floor: lines 0.5 m apart, scrolled by the distance the
        // body covered - a planted foot that slides drifts across them. Drawn
        // ON the floor by the render, so the body covers them.
        if (b.showFloor) {
            const float step = 0.5f, ext = 3.0f;
            for (int axis = 0; axis < 2; ++axis)
                for (int k = -6; k <= 6; ++k) {
                    const float v = (float)k * step + std::fmod(b.travel[axis], step);
                    d.floorLines.push_back(axis == 0 ? std::array<float, 6>{v, 0.0f, -ext, v, 0.0f, ext}
                                                     : std::array<float, 6>{-ext, 0.0f, v, ext, 0.0f, v});
                }
            const ImVec4 lc = mixTo(ImVec4(0.42f, 0.44f, 0.48f, 1.0f), sm.textDim, 0.55f);
            d.floorLineColor[0] = lc.x, d.floorLineColor[1] = lc.y, d.floorLineColor[2] = lc.z;
        }
        float vp[16];
        const uint32_t tex = viewport_.renderRagdollPreview(pw, ph, d, vp);
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        if (tex) {
            ImGui::Image((ImTextureID)(intptr_t)tex, ImVec2((float)pw, (float)ph), ImVec2(0, 1), ImVec2(1, 0));
            ImGui::SetCursorScreenPos(origin);
            ImGui::InvisibleButton("Blend preview##bs_view", ImVec2((float)pw, (float)ph),
                                   ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
            ImGuiIO& io = ImGui::GetIO();
            if (ImGui::IsItemHovered() && io.MouseWheel != 0.0f)
                b.zoom = std::clamp(b.zoom * std::pow(1.15f, io.MouseWheel), 0.1f, 8.0f);
            if (ImGui::IsItemActive() && ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
                b.yaw += io.MouseDelta.x * 0.4f;
                b.pitch = std::clamp(b.pitch + io.MouseDelta.y * 0.3f, -30.0f, 85.0f);
            }
            if (ImGui::IsItemActive() && ImGui::IsMouseDown(ImGuiMouseButton_Middle)) {
                const float s = 2.0f / std::max(1.0f, (float)ph) / b.zoom;
                b.panX = std::clamp(b.panX - io.MouseDelta.x * s, -5.0f, 5.0f);
                b.panY = std::clamp(b.panY + io.MouseDelta.y * s, -5.0f, 5.0f);
            }
            tip("Right-drag orbits, middle-drag pans, the wheel zooms. The lines are the floor moving "
                "under the body at the preview point's speed: a planted foot that slides drifts across them.");
        }

        // the compass
        const ImVec2 cpos = ImGui::GetCursorScreenPos();
        const ImVec2 csz(ImGui::GetContentRegionAvail().x, std::max(scaled(160), ImGui::GetContentRegionAvail().y));
        ImGui::InvisibleButton("Compass##bs_compass", csz, ImGuiButtonFlags_MouseButtonLeft);
        const bool hovered = ImGui::IsItemHovered(), pressed = ImGui::IsItemActivated();
        const bool active = ImGui::IsItemActive();
        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->AddRectFilled(cpos, ImVec2(cpos.x + csz.x, cpos.y + csz.y), u32(sm.field), ImGui::GetStyle().FrameRounding);
        Graph cp;
        const bsw::Grid& gl = b.asset.lines;
        cp.grid = b.asset.grid && gl.columns >= 2 && gl.rows >= 2 && gl.dirMax > gl.dirMin && gl.speedMax > gl.speedMin;
        cp.lines = gl;
        cp.c = ImVec2(cpos.x + csz.x * 0.5f, cpos.y + csz.y * 0.5f);
        cp.r = std::max(10.0f, std::min(csz.x, csz.y) * 0.5f - scaled(26));
        float ringMax = 0.0f;
        for (const blendspace::AssetSample& s : b.asset.samples) ringMax = std::max(ringMax, s.speed);
        cp.maxSpeed = cp.grid ? gl.speedMax
                              : (b.dragSample >= 0 && b.dragMax > 0.0f ? b.dragMax : std::max(1.0f, ringMax * 1.25f));
        const float fh = ImGui::GetFontSize();
        // the grid's labels: speeds left, directions under it, and two lines
        // above it for the top row's clip names
        cp.lo = ImVec2(cpos.x + ImGui::CalcTextSize("00.00").x + scaled(14), cpos.y + fh * 2.0f + scaled(14));
        cp.hi = ImVec2(cpos.x + csz.x - scaled(22), cpos.y + csz.y - fh * 2.0f - scaled(12));
        const ImU32 grid = u32(fadeTo(sm.textDim, 0.35f)), gridHi = u32(fadeTo(sm.textDim, 0.6f));
        if (cp.grid) {
            // the grid: its lines (the edges and 0 deg brighter), each one's
            // value, a dot on every empty intersection
            const float colW = (cp.hi.x - cp.lo.x) / (float)(gl.columns - 1);
            for (int k = 0; k < gl.columns; ++k) {
                const float d = cp.column(k);
                const ImVec2 a = cp.at(d, gl.speedMin), e = cp.at(d, gl.speedMax);
                dl->AddLine(a, e, k == 0 || k == gl.columns - 1 || std::fabs(d) < 1e-3f ? gridHi : grid, 1.0f);
                char t[16];
                std::snprintf(t, sizeof(t), "%g", (double)d);
                const float tw = ImGui::CalcTextSize(t).x;
                if (colW >= tw + scaled(6) || k % 2 == 0)
                    dl->AddText(ImVec2(a.x - tw * 0.5f, cp.hi.y + scaled(4)), u32(labelCol()), t);
            }
            for (int k = 0; k < gl.rows; ++k) {
                const float s = cp.row(k);
                const ImVec2 a = cp.at(gl.dirMin, s), e = cp.at(gl.dirMax, s);
                dl->AddLine(a, e, k == 0 || k == gl.rows - 1 ? gridHi : grid, 1.0f);
                char t[16];
                std::snprintf(t, sizeof(t), "%.2f", (double)s);
                dl->AddText(ImVec2(cp.lo.x - ImGui::CalcTextSize(t).x - scaled(6), a.y - fh * 0.5f), u32(labelCol()), t);
            }
            for (int r = 0; r < gl.rows; ++r)
                for (int k = 0; k < gl.columns; ++k) {
                    bool taken = false;
                    for (const blendspace::AssetSample& s : b.asset.samples)
                        taken |= std::fabs(s.dir - cp.column(k)) < 1e-3f && std::fabs(s.speed - cp.row(r)) < 1e-3f;
                    if (!taken) dl->AddCircleFilled(cp.at(cp.column(k), cp.row(r)), scaled(2.5f), gridHi);
                }
            const char* across = "direction (deg, + = right)";
            dl->AddText(ImVec2(cp.hi.x - ImGui::CalcTextSize(across).x, cp.hi.y + fh + scaled(6)), u32(labelCol()), across);
            dl->AddText(ImVec2(cpos.x + scaled(6), cpos.y + scaled(4)), u32(labelCol()), "m/s");
        } else {
            // the compass: a ring every 0.5 m/s, spokes every 45 deg
            for (float s = 0.5f; s <= cp.maxSpeed + 1e-3f; s += 0.5f)
                dl->AddCircle(cp.c, s / cp.maxSpeed * cp.r, grid, 64, 1.0f);
            for (int k = 0; k < 8; ++k) {
                const ImVec2 e = cp.at((float)k * 45.0f, cp.maxSpeed);
                dl->AddLine(cp.c, e, k % 2 == 0 ? gridHi : grid, 1.0f);
            }
            dl->AddText(ImVec2(cp.c.x - ImGui::CalcTextSize("forward").x * 0.5f, cp.c.y - cp.r - scaled(18)),
                        u32(labelCol()), "forward");
            dl->AddText(ImVec2(cp.c.x + cp.r + scaled(4), cp.c.y - ImGui::GetFontSize() * 0.5f), u32(labelCol()), "right");
            dl->AddText(ImVec2(cp.c.x - cp.r - ImGui::CalcTextSize("left").x - scaled(4), cp.c.y - ImGui::GetFontSize() * 0.5f),
                        u32(labelCol()), "left");
        }
        // the samples' edges (dashed: the interpolation's neighbours)
        auto dashed = [&](ImVec2 a, ImVec2 c, ImU32 col) {
            const float len = std::sqrt((c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y));
            const int n = std::max(1, (int)(len / scaled(6)));
            for (int i = 0; i < n; i += 2) {
                const float t0 = (float)i / n, t1 = std::min(1.0f, (float)(i + 1) / n);
                dl->AddLine(ImVec2(a.x + (c.x - a.x) * t0, a.y + (c.y - a.y) * t0),
                            ImVec2(a.x + (c.x - a.x) * t1, a.y + (c.y - a.y) * t1), col, 1.0f);
            }
        };
        const ImU32 edge = u32(fadeTo(sm.accent, 0.45f));
        int centre = -1;
        for (size_t i = 0; i < b.asset.samples.size(); ++i)
            if (b.asset.samples[i].speed < bsw::kRingEpsilon && centre < 0) centre = (int)i;
        for (size_t i = 0; i < b.asset.samples.size() && !cp.grid; ++i) {
            const blendspace::AssetSample& s = b.asset.samples[i];
            if (s.speed < bsw::kRingEpsilon) continue;
            if (centre >= 0) dashed(cp.c, cp.at(s.dir, s.speed), edge);
            // the next sample clockwise on the same ring
            int next = -1;
            float best = 361.0f;
            for (size_t j = 0; j < b.asset.samples.size(); ++j) {
                const blendspace::AssetSample& o = b.asset.samples[j];
                if (j == i || std::fabs(o.speed - s.speed) >= bsw::kRingEpsilon) continue;
                float a = std::fmod(o.dir - s.dir + 720.0f, 360.0f);
                if (a > 0.0f && a < best) best = a, next = (int)j;
            }
            if (next >= 0) {
                const blendspace::AssetSample& o = b.asset.samples[(size_t)next];
                // along the ring's arc, so the edge reads as "between these two"
                const int segs = std::max(2, (int)(best / 6.0f));
                for (int k = 0; k < segs; k += 2) {
                    const float a0 = s.dir + best * (float)k / segs, a1 = s.dir + best * (float)std::min(k + 1, segs) / segs;
                    dl->AddLine(cp.at(a0, s.speed), cp.at(a1, s.speed), edge, 1.0f);
                }
                (void)o;
            }
        }
        // the samples: dots, the selected one in the warning colour, weights -
        // a grid repeats a clip (the idle along its bottom row) and reports it
        // once, so its weight goes on the copy nearest the preview point
        int weightAt[3] = {-1, -1, -1};
        for (int k = 0; k < b.shown.count; ++k) {
            weightAt[k] = b.shown.sample[k];
            if (!cp.grid) continue;
            const ImVec2 pp = cp.at(b.dir, b.speed);
            float best = 1e30f;
            for (size_t i = 0; i < samples.size() && i < b.asset.samples.size(); ++i)
                if (samples[i].clip == samples[(size_t)b.shown.sample[k]].clip) {
                    const ImVec2 q = cp.at(b.asset.samples[i].dir, b.asset.samples[i].speed);
                    const float d = (q.x - pp.x) * (q.x - pp.x) + (q.y - pp.y) * (q.y - pp.y);
                    if (d < best) best = d, weightAt[k] = (int)i;
                }
        }
        const float dotR = scaled(6);
        int hoverSample = -1;
        const ImVec2 mouse = ImGui::GetIO().MousePos;
        for (size_t i = 0; i < b.asset.samples.size(); ++i) {
            const blendspace::AssetSample& s = b.asset.samples[i];
            const ImVec2 p = cp.at(s.dir, s.speed);
            if (hovered && (mouse.x - p.x) * (mouse.x - p.x) + (mouse.y - p.y) * (mouse.y - p.y) <= dotR * dotR * 2.5f)
                hoverSample = (int)i;
            const bool selected = b.sel == (int)i;
            dl->AddCircleFilled(p, dotR, u32(selected ? sm.warn : sm.surface));
            dl->AddCircle(p, dotR, u32(selected ? sm.warn : sm.accent), 0, 2.0f);
            if (cp.grid) {
                // centred above its dot, every other column a line higher so
                // neighbours do not run together; left out when even that is
                // too narrow (hovering the dot still names it)
                const float colW = (cp.hi.x - cp.lo.x) / (float)(cp.lines.columns - 1);
                const int k = (int)std::lround((s.dir - cp.lines.dirMin) / (cp.lines.dirMax - cp.lines.dirMin) *
                                               (float)(cp.lines.columns - 1));
                const float tw = ImGui::CalcTextSize(s.clip.c_str()).x;
                if (tw <= colW * 2.0f - scaled(6))
                    dl->AddText(ImVec2(p.x - tw * 0.5f, p.y - dotR - scaled(2) - ImGui::GetFontSize() * (k % 2 == 0 ? 1.0f : 2.0f)),
                                u32(sm.text), s.clip.c_str());
            } else {
                dl->AddText(ImVec2(p.x + dotR + scaled(3), p.y - ImGui::GetFontSize()), u32(sm.text), s.clip.c_str());
            }
            if (b.showWeights)
                for (int k = 0; k < b.shown.count; ++k)
                    if (weightAt[k] == (int)i) {
                        char w[16];
                        std::snprintf(w, sizeof(w), "%.0f%%", (double)(b.shown.weight[k] * 100.0f));
                        dl->AddText(ImVec2(p.x + dotR + scaled(3), cp.grid ? p.y - ImGui::GetFontSize() * 0.5f : p.y), u32(sm.ok), w);
                    }
        }
        // the preview point (green): where the user put it, and the smoothed one
        const ImVec2 pt = cp.at(b.dir, b.speed);
        float sd = 0.0f, ss = 0.0f;
        bsw::fromPoint(b.point, &sd, &ss);
        dl->AddCircle(cp.at(sd, ss), scaled(5), u32(fadeTo(sm.ok, 0.6f)), 0, 1.5f);
        dl->AddCircleFilled(pt, scaled(6), u32(sm.ok));
        dl->AddCircle(pt, scaled(6), u32(sm.surface), 0, 2.0f);
        // interaction: Ctrl+drag the point; drag a sample (snapped unless
        // Shift); click a sample to select it; drop a clip to add one
        ImGuiIO& io = ImGui::GetIO();
        float md = 0.0f, ms = 0.0f;
        cp.of(mouse, &md, &ms);
        const auto snapped = [&](float* d, float* s) {
            if (cp.grid) {  // a grid's sample sits on an intersection, always
                const int k = std::clamp((int)std::lround((*d - gl.dirMin) / (gl.dirMax - gl.dirMin) * (float)(gl.columns - 1)),
                                         0, gl.columns - 1);
                const int r = std::clamp((int)std::lround((*s - gl.speedMin) / (gl.speedMax - gl.speedMin) * (float)(gl.rows - 1)),
                                         0, gl.rows - 1);
                *d = cp.column(k), *s = cp.row(r);
                return;
            }
            if (io.KeyShift) return;
            *d = bsw::wrapDeg(snapTo(*d, 45.0f));
            *s = snapTo(*s, 0.5f);
            if (*s < bsw::kRingEpsilon) *d = 0.0f;
        };
        if (pressed) {
            if (io.KeyCtrl || hoverSample < 0) {
                b.dragPoint = true;
                b.dragSample = -1;
                if (hoverSample < 0 && !io.KeyCtrl) b.sel = -1;
            } else {
                b.dragSample = hoverSample;
                b.dragMax = cp.maxSpeed;  // the scale stays put while it is dragged
                b.sel = hoverSample;
            }
        }
        if (active && b.dragPoint) {
            b.dir = md;
            b.speed = std::min(ms, cp.maxSpeed);
        } else if (active && b.dragSample >= 0 && b.dragSample < (int)b.asset.samples.size() &&
                   ImGui::IsMouseDragging(ImGuiMouseButton_Left, 2.0f)) {
            float d = md, s = std::min(ms, cp.maxSpeed);
            snapped(&d, &s);
            blendspace::AssetSample& me = b.asset.samples[(size_t)b.dragSample];
            if (cp.grid)  // the intersection's sample takes this one's old place
                for (size_t j = 0; j < b.asset.samples.size(); ++j)
                    if ((int)j != b.dragSample && std::fabs(b.asset.samples[j].dir - d) < 1e-3f &&
                        std::fabs(b.asset.samples[j].speed - s) < 1e-3f)
                        b.asset.samples[j].dir = me.dir, b.asset.samples[j].speed = me.speed;
            me.dir = d;
            me.speed = s;
        }
        if (!active) b.dragPoint = false, b.dragSample = -1;
        if (ImGui::BeginDragDropTarget()) {
            if (const ImGuiPayload* pl = ImGui::AcceptDragDropPayload("BS_CLIP")) {
                blendspace::AssetSample s;
                s.clip = (const char*)pl->Data;
                s.dir = md;
                s.speed = std::min(ms, cp.maxSpeed);
                snapped(&s.dir, &s.speed);
                s.own = s.speed < bsw::kRingEpsilon;  // a centre / bottom-row sample keeps its own clock
                int taken = -1;
                for (size_t j = 0; j < b.asset.samples.size() && cp.grid; ++j)
                    if (std::fabs(b.asset.samples[j].dir - s.dir) < 1e-3f && std::fabs(b.asset.samples[j].speed - s.speed) < 1e-3f)
                        taken = (int)j;
                if (taken >= 0) {
                    b.asset.samples[(size_t)taken].clip = s.clip;
                    b.sel = taken;
                } else {
                    b.asset.samples.push_back(s);
                    b.sel = (int)b.asset.samples.size() - 1;
                }
            }
            ImGui::EndDragDropTarget();
        }
        if (hovered && b.sel >= 0 && ImGui::IsKeyPressed(ImGuiKey_Delete, false) &&
            b.sel < (int)b.asset.samples.size()) {
            b.asset.samples.erase(b.asset.samples.begin() + b.sel);
            b.sel = -1;
        }
        // just the values under the cursor - how to use the graph is in
        // docs/blend-spaces.md
        if (hovered) {
            if (hoverSample >= 0)
                ImGui::SetTooltip("%s - %g deg, %.2f m/s", b.asset.samples[(size_t)hoverSample].clip.c_str(),
                                  (double)b.asset.samples[(size_t)hoverSample].dir,
                                  (double)b.asset.samples[(size_t)hoverSample].speed);
            else
                ImGui::SetTooltip("%.0f deg, %.2f m/s", (double)md, (double)ms);
        }
    }
    ImGui::EndChild();
    splitter("##bs_split_right", b.rightW, -1.0f);

    // right: the selected sample, or the blend space, and the point's weights
    ImGui::BeginChild("##bs_right", ImVec2(0, avail.y), ImGuiChildFlags_Borders);
    {
        const float lw = scaled(96);
        if (b.sel >= 0 && b.sel < (int)b.asset.samples.size()) {
            blendspace::AssetSample& s = b.asset.samples[(size_t)b.sel];
            section("Sample", "One clip placed on the compass.");
            prop("Clip", lw, "Which of the model's clips this sample shows.");
            nameCombo("##SampleClip", s.clip, b.clipNames);
            prop("Direction", lw, "Degrees from where the model looks: 0 forward, +90 its right, -90 its "
                                  "left, 180 straight back.");
            if (ImGui::DragFloat("##SampleDir", &s.dir, 1.0f, -180.0f, 180.0f, "%.1f deg")) s.dir = bsw::wrapDeg(s.dir);
            prop("Speed", lw, "Metres per second. 0 = the centre, which counts for every direction "
                              "(one sample at most).");
            ImGui::DragFloat("##SampleSpeed", &s.speed, 0.01f, 0.0f, 20.0f, "%.2f m/s");
            s.speed = std::max(0.0f, s.speed);
            prop("Time", lw, "Sync: the clip shares the blend space's phase with the other sync samples "
                             "(the strides - their feet stay in step). Own time: it loops on its own clock "
                             "(an idle).");
            int t = s.own ? 1 : 0;
            if (ImGui::Combo("##SampleTime", &t, "Sync\0Own time\0")) s.own = t == 1;
            if (tintButton("Delete sample", sm.danger)) {
                b.asset.samples.erase(b.asset.samples.begin() + b.sel);
                b.sel = -1;
            }
        } else {
            section("Blend space", "The asset: the model whose clips it places and how the game uses it.");
            prop("Model", lw, "The animated model whose clips the samples name.");
            ImGui::TextUnformatted(b.asset.model.c_str());
            prop("Name in game", lw, "What a game asks for: setAnimationMode(ctx, object, "
                                     "ANIM_MODE_BLEND_SPACE, \"name\"). At most 31 characters.");
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%s", b.asset.name.c_str());
            if (ImGui::InputText("##BsName", buf, sizeof(buf))) b.asset.name = buf;
            prop("Layout", lw,
                 "How the samples are laid out and blended: the RADIAL compass (direction round, speed outward) "
                 "or a 2D GRID (direction across, speed up, clips on the intersections - an empty one takes the "
                 "nearest clip on its row). Switching converts the asset so it blends exactly as before; a side "
                 "it cannot switch to is greyed - rest the cursor on it to see why and what to change.");
            // Radial | Grid: an option the asset cannot become without losing a
            // clip or blending differently is greyed, its tooltip saying why
            blendspace::Asset asGrid;
            std::string gridWhy, radialWhy;
            const bool canGrid = b.asset.grid || blendspace::toGrid(b.asset, asGrid, gridWhy);
            const bool canRad = !b.asset.grid || blendspace::canRadial(b.asset, radialWhy);
            const std::string radialTip = "Radial is not available:\n" + radialWhy;
            const std::string gridTip = "Grid is not available: " + gridWhy + ".";
            const char* layouts[2] = {"Radial", "Grid"};
            const bool enabled[2] = {canRad, canGrid};
            const char* why[2] = {radialTip.c_str(), gridTip.c_str()};
            int layout = b.asset.grid ? 1 : 0;
            if (slideSwitch("##BsLayout", layouts, 2, &layout, enabled, why)) {
                if (layout == 1) {
                    b.asset = asGrid;
                    b.status = "Now a grid - it blends exactly like the compass did.";
                } else {
                    b.asset = blendspace::toRadial(b.asset);
                    b.status = "Back on the compass - it blends exactly like the grid did.";
                }
                b.sel = -1;
            }
            if (b.asset.grid) {
                bsw::Grid& g = b.asset.lines;
                prop("Lines", lw, "Vertical x horizontal lines, at least 2 each (the smallest grid is a square). "
                                  "Samples keep their place: one off the new intersections is flagged below.");
                int cr[2] = {g.columns, g.rows};
                if (ImGui::DragInt2("##BsLines", cr, 0.05f, 2, 33)) g.columns = std::max(2, cr[0]), g.rows = std::max(2, cr[1]);
                prop("Directions", lw, "Degrees at the first and the last vertical line (+ = right).");
                float dr[2] = {g.dirMin, g.dirMax};
                if (ImGui::DragFloat2("##BsDirs", dr, 1.0f, -360.0f, 360.0f, "%.0f deg")) g.dirMin = dr[0], g.dirMax = dr[1];
                prop("Speeds", lw, "m/s at the bottom and the top horizontal line.");
                float sr[2] = {g.speedMin, g.speedMax};
                if (ImGui::DragFloat2("##BsSpeeds", sr, 0.01f, 0.0f, 20.0f, "%.2f m/s"))
                    g.speedMin = std::max(0.0f, sr[0]), g.speedMax = std::max(0.0f, sr[1]);
            } else if (ImGui::Button("Save a grid version")) {
                // the same blend laid out on a grid, beside this asset: <stem>_grid
                blendspace::Asset g;
                std::string err;
                const fs::path p(b.assetRel);
                const std::string rel = (p.parent_path() / (p.stem().string() + "_grid.blendspace")).generic_string();
                std::error_code ec;
                if (!blendspace::toGrid(b.asset, g, err)) {
                    b.status = "No grid blends like this compass: " + err;
                } else if (fs::exists(project_.filePath(rel), ec)) {
                    b.status = rel + " exists already - open it from the Blend space list.";
                } else {
                    g.name = (b.asset.name + "_grid").substr(0, 31);
                    if (blendspace::save(project_.filePath(rel), g, err)) {
                        assetsChanged();
                        blendSpaceLoad(rel);
                        b.status = "Saved " + rel + " (\"" + g.name + "\" in game): the same blend on a grid.";
                    } else {
                        b.status = err;
                    }
                }
            }
            if (!b.asset.grid)
                tip("Writes <this asset>_grid.blendspace: the same samples laid out on the grid that blends exactly "
                    "like this compass (\"<name>_grid\" in game), and opens it. This asset stays as it is.");
            prop("Stride", lw, "Metres one phase cycle covers, when the ENGINE plays the phase (a game "
                               "passing its own phase ignores it): the phase advances by the distance "
                               "the speed covers, so planted feet stay planted. 0 = by the clips' time.");
            ImGui::DragFloat("##BsStride", &b.asset.stride, 0.01f, 0.0f, 20.0f, "%.3f m");
            prop("Smoothing", lw, "Seconds: the input point eases toward a new direction and speed over "
                                  "about this long, so starts, stops and turns blend instead of popping. "
                                  "0 = none.");
            ImGui::DragFloat("##BsSmooth", &b.asset.smoothing, 0.005f, 0.0f, 2.0f, "%.3f s");
            prop("LOD fade", lw, "Seconds of crossfade when the blend LOD changes what it shows (a body "
                                 "past its blendLod distance shows only the best-fitting sample).");
            ImGui::DragFloat("##BsLodFade", &b.asset.lodFade, 0.005f, 0.0f, 2.0f, "%.3f s");
            b.asset.stride = std::max(0.0f, b.asset.stride);
            b.asset.smoothing = std::max(0.0f, b.asset.smoothing);
            b.asset.lodFade = std::max(0.0f, b.asset.lodFade);
            const blendspace::Report rep = blendspace::check(b.asset, b.clipNames);
            for (const std::string& e : rep.errors) ImGui::TextColored(sm.danger, "%s", e.c_str());
            for (const std::string& w : rep.warnings) ImGui::TextColored(sm.warn, "%s", w.c_str());
        }
        section("Preview point", "Where the green point is: drag on the compass, or type it here. The "
                                 "bars are the weights the engine would blend with right now.");
        prop("Direction", lw);
        if (ImGui::DragFloat("##PtDir", &b.dir, 1.0f, -180.0f, 180.0f, "%.1f deg")) b.dir = bsw::wrapDeg(b.dir);
        prop("Speed", lw);
        ImGui::DragFloat("##PtSpeed", &b.speed, 0.01f, 0.0f, 20.0f, "%.2f m/s");
        b.speed = std::max(0.0f, b.speed);
        prop("Move speed", lw,
             "How fast the body really moves on or past the ring - the phase (by the Stride) and the floor. "
             "The compass ends just past the ring, but a game feeds running speeds there and the engine "
             "clamps the weights to the ring, so this is what a running body looks like. Inside the ring "
             "the body moves at the point's share of it (87 % of the way to a sample = 87 % of this), so "
             "dragging out never jumps the pace. Default: the speed the clips were authored at (the Stride "
             "over their cycle). Right-click resets it.");
        float mv = b.moveSpeed >= 0.0f ? b.moveSpeed : b.authoredSpeed;
        if (ImGui::DragFloat("##MoveSpeed", &mv, 0.05f, 0.0f, 20.0f,
                             b.moveSpeed >= 0.0f ? "%.2f m/s" : "%.2f m/s (authored)"))
            b.moveSpeed = std::max(0.0f, mv);
        if (ImGui::IsItemClicked(ImGuiMouseButton_Right)) b.moveSpeed = -1.0f;
        if (b.moveAt != b.speed) {
            ImGui::TextDisabled("moving at %.2f m/s", (double)b.moveAt);
        }
        for (int k = 0; k < b.shown.count; ++k) {
            const blendspace::AssetSample& s = b.asset.samples[(size_t)b.shown.sample[k]];
            ImGui::TextUnformatted(s.clip.c_str());
            ImGui::SameLine(lw);
            char ov[16];
            std::snprintf(ov, sizeof(ov), "%.0f%%", (double)(b.shown.weight[k] * 100.0f));
            ImGui::ProgressBar(b.shown.weight[k], ImVec2(-FLT_MIN, 0), ov);
        }
        if (b.shown.count == 0) ImGui::TextDisabled("No samples to blend.");
    }
    ImGui::EndChild();

    // --- the edit bookkeeping: an undo step per settled change -----------------
    const std::string now = blendspace::serialize(b.asset);
    if (now != b.lastJson && !ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        b.undo.push_back(b.lastJson);
        if (b.undo.size() > 100) b.undo.erase(b.undo.begin());
        b.redo.clear();
        b.lastJson = now;
    }
    ImGui::End();
}
