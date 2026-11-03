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
#include "theme.hpp"

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
    float leftW = 300.0f, rightW = 340.0f;  // side panels, unscaled px (the splitters)
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
    // the solver as last saved: what a solver row's undo goes back to
    ragdoll::Solver savedSolver;
    std::string savedSolverOf;  // the savedJson it was read from
    // A motor profile picked - in the toolbar or the Profiles list, the two
    // are one control: it is the one the preview drives, the one the list
    // marks, the one the details show, and the mode starts over with it.
    void pickProfile(int i) {
        profile = i;
        selKind = Profile;
        sel = i;
        restartReq = true;
    }
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

// The joint's axes, in the preview and on the limits they name: the usual
// red / green / blue, softened so a column of them does not glare.
const ImVec4 kPitchCol(0.94f, 0.47f, 0.47f, 1.0f), kYawCol(0.52f, 0.84f, 0.54f, 1.0f),
    kRollCol(0.50f, 0.68f, 1.0f, 1.0f);

ImVec4 fadeTo(const ImVec4& c, float a) { return ImVec4(c.x, c.y, c.z, a); }
ImVec4 mixTo(const ImVec4& a, const ImVec4& b, float t) {
    return ImVec4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t);
}
ImU32 u32(const ImVec4& c) { return ImGui::GetColorU32(c); }

// A label: a step quieter than the value beside it.
ImVec4 labelCol() {
    const theme::Semantics& sm = theme::semantics();
    return mixTo(sm.text, sm.textDim, 0.45f);
}

void vrule() {
    ImGui::SameLine();
    ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
    ImGui::SameLine();
}

// Help reads as a short paragraph, so every tooltip here wraps.
void wrappedTooltip(const char* text) {
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 26.0f);
    ImGui::TextUnformatted(text);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
}

// The last item's tooltip, over a disabled item too.
void tip(const char* text) {
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip | ImGuiHoveredFlags_AllowWhenDisabled))
        wrappedTooltip(text);
}

// The (?) every tool window puts after a label (atlas_ui.cpp keeps the same
// three lines); here its text wraps.
void helpMarker(const char* text) {
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayNone | ImGuiHoveredFlags_AllowWhenDisabled))
        wrappedTooltip(text);
}

// A section: its title, an optional (?), and a hairline out to the right edge.
void section(const char* title, const char* help = nullptr) {
    ImGui::Dummy(ImVec2(0.0f, ImGui::GetStyle().ItemSpacing.y * 0.6f));
    const float right = ImGui::GetCursorScreenPos().x + ImGui::GetContentRegionAvail().x;
    ImGui::TextUnformatted(title);
    if (help) helpMarker(help);
    const ImVec2 mn = ImGui::GetItemRectMin(), mx = ImGui::GetItemRectMax();
    const float y = std::floor(0.5f * (mn.y + mx.y)) + 0.5f;
    const float x0 = mx.x + ImGui::GetStyle().ItemSpacing.x;
    if (right > x0)
        ImGui::GetWindowDrawList()->AddLine(ImVec2(x0, y), ImVec2(right, y), u32(theme::semantics().border));
}

// An inspector row: the label on the LEFT in a fixed column and the widget
// filling the rest - not ImGui's label after the box. Not a table, so the
// widget keeps the window's ID scope (a ui-script reaches it by its id).
void prop(const char* label, float labelW, const char* help = nullptr, const ImVec4* col = nullptr) {
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(col ? *col : labelCol(), "%s", label);
    if (help) helpMarker(help);
    ImGui::SameLine();
    const float x = ImGui::GetCursorStartPos().x + labelW;
    if (ImGui::GetCursorPosX() < x) ImGui::SetCursorPosX(x);
    ImGui::SetNextItemWidth(-FLT_MIN);
}

// A slider drawn as a fill: how full the bar is IS the value, with no bright
// grab knob - a motor profile has a column of fifteen of them.
bool fillSlider(const char* id, float* v, float lo, float hi, const char* fmt) {
    const theme::Semantics& sm = theme::semantics();
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const float w = ImGui::CalcItemWidth(), h = ImGui::GetFrameHeight();
    const float t = std::clamp((*v - lo) / std::max(hi - lo, 1e-6f), 0.0f, 1.0f);
    const float rounding = ImGui::GetStyle().FrameRounding;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), u32(sm.surface), rounding);
    if (t > 0.0f)
        dl->AddRectFilled(p, ImVec2(p.x + w * t, p.y + h), u32(fadeTo(sm.accent, 0.30f)), rounding,
                          t > 0.98f ? ImDrawFlags_RoundCornersAll : ImDrawFlags_RoundCornersLeft);
    ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, fadeTo(sm.text, 0.05f));
    ImGui::PushStyleColor(ImGuiCol_FrameBgActive, fadeTo(sm.text, 0.09f));
    ImGui::PushStyleColor(ImGuiCol_SliderGrab, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, ImVec4(0, 0, 0, 0));
    const bool changed = ImGui::SliderFloat(id, v, lo, hi, fmt);
    ImGui::PopStyleColor(5);
    return changed;
}

// A square icon button the height of a frame, drawn rather than typed (the
// UI font has no icon glyphs): 0 = undo (a hooked arrow back), 1 = reset to
// the default (a circular arrow in the warn colour, as Unreal's "reset to
// default" is yellow).
bool iconButton(const char* id, int icon) {
    const theme::Semantics& sm = theme::semantics();
    const float h = ImGui::GetFrameHeight();
    const bool clicked = ImGui::InvisibleButton(id, ImVec2(h, h));
    const bool hot = ImGui::IsItemHovered() || ImGui::IsItemActive();
    const ImVec2 mn = ImGui::GetItemRectMin(), mx = ImGui::GetItemRectMax();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    if (hot) dl->AddRectFilled(mn, mx, u32(fadeTo(sm.text, 0.10f)), ImGui::GetStyle().FrameRounding);
    const ImVec4 base = icon == 0 ? labelCol() : sm.warn;
    const ImU32 col = u32(hot ? mixTo(base, sm.text, 0.4f) : base);
    const ImVec2 c(0.5f * (mn.x + mx.x), 0.5f * (mn.y + mx.y));
    const float r = h * 0.24f, th = std::max(1.0f, h * 0.075f), hs = h * 0.13f;
    // the arc runs from a0 to a1; its head sits at a1, pointing along the arc
    const float a0 = icon == 0 ? 0.45f : -0.35f, a1 = icon == 0 ? -3.14159f : -5.6f;
    dl->PathArcTo(c, r, a0, a1, 16);
    dl->PathStroke(col, ImDrawFlags_None, th);
    const ImVec2 p(c.x + r * std::cos(a1), c.y + r * std::sin(a1));
    const ImVec2 d(std::sin(a1), -std::cos(a1));  // the arc's way at a1 (angles decreasing)
    const ImVec2 n(-d.y, d.x);
    dl->AddTriangleFilled(ImVec2(p.x + d.x * hs, p.y + d.y * hs),
                          ImVec2(p.x - d.x * hs * 0.4f + n.x * hs, p.y - d.y * hs * 0.4f + n.y * hs),
                          ImVec2(p.x - d.x * hs * 0.4f - n.x * hs, p.y - d.y * hs * 0.4f - n.y * hs), col);
    return clicked;
}

// A value row's two slots at its far right: back to the SAVED value (only
// while it differs) and back to the DEFAULT (likewise). The slots keep their
// places whether shown or not, so the rows stay aligned. True when one was
// clicked, with the value it restores in `out`.
bool valueResets(const char* id, float cur, float saved, float def, const char* fmt, float& out) {
    auto differs = [](float a, float b) { return std::fabs(a - b) > 1e-4f * std::max(1.0f, std::fabs(b)); };
    const float h = ImGui::GetFrameHeight(), gap = ImGui::GetStyle().ItemSpacing.x * 0.4f;
    bool clicked = false;
    char val[48], tipText[96], undoId[64], defId[64];
    // ids at the row's own scope (no PushID), so a ui-script reaches them: <id>_undo, <id>_default
    std::snprintf(undoId, sizeof(undoId), "%s_undo", id);
    std::snprintf(defId, sizeof(defId), "%s_default", id);
    ImGui::SameLine(0.0f, gap);
    if (differs(cur, saved)) {
        if (iconButton(undoId, 0)) out = saved, clicked = true;
        std::snprintf(val, sizeof(val), fmt, (double)saved);
        std::snprintf(tipText, sizeof(tipText), "Back to the saved value: %s", val);
        tip(tipText);
    } else {
        ImGui::Dummy(ImVec2(h, h));
    }
    ImGui::SameLine(0.0f, gap);
    if (differs(cur, def)) {
        if (iconButton(defId, 1)) out = def, clicked = true;
        std::snprintf(val, sizeof(val), fmt, (double)def);
        std::snprintf(tipText, sizeof(tipText), "Back to the default: %s", val);
        tip(tipText);
    } else {
        ImGui::Dummy(ImVec2(h, h));
    }
    return clicked;
}
// What valueResets needs at the row's right: its widget's width is the rest.
float valueResetsWidth() {
    return 2.0f * (ImGui::GetFrameHeight() + ImGui::GetStyle().ItemSpacing.x * 0.4f);
}

// A button in a quiet wash of one colour: the active segment of a segmented
// control and a panel's main action (the accent), a delete (danger). `faint`
// is quieter still - the colour mostly in the text - for a button repeated
// down a column, where a solid wash of each would glare.
bool tintButton(const char* label, const ImVec4& col, const ImVec2& size = ImVec2(0, 0), bool faint = false) {
    const theme::Semantics& sm = theme::semantics();
    ImGui::PushStyleColor(ImGuiCol_Button, fadeTo(col, faint ? 0.07f : 0.24f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, fadeTo(col, faint ? 0.22f : 0.38f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, fadeTo(col, faint ? 0.34f : 0.52f));
    ImGui::PushStyleColor(ImGuiCol_Border, fadeTo(col, faint ? 0.28f : 0.55f));
    ImGui::PushStyleColor(ImGuiCol_Text, faint ? mixTo(col, sm.text, 0.30f) : sm.text);
    const bool clicked = ImGui::Button(label, size);
    ImGui::PopStyleColor(5);
    return clicked;
}
bool segment(const char* label, bool on, float w) {
    return on ? tintButton(label, theme::semantics().accent, ImVec2(w, 0)) : ImGui::Button(label, ImVec2(w, 0));
}
bool dangerButton(const char* label, const ImVec2& size = ImVec2(0, 0)) {
    return tintButton(label, theme::semantics().danger, size);
}

// A note at the right end of the last item (a list row, a tree node), when
// there is room beside its name; `pill` puts it on a tinted lozenge.
void noteRight(const char* note, const ImVec4& col, float nameW, bool pill = false) {
    if (!note || !*note) return;
    const ImVec2 mn = ImGui::GetItemRectMin(), mx = ImGui::GetItemRectMax();
    const ImVec2 ts = ImGui::CalcTextSize(note);
    const float pad = ImGui::GetStyle().FramePadding.x;
    const float x = mx.x - ts.x - pad;
    if (x < mn.x + nameW + pad * 2.0f) return;  // no room: the name wins
    const float y = mn.y + std::floor((mx.y - mn.y - ts.y) * 0.5f);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    if (pill)
        dl->AddRectFilled(ImVec2(x - pad * 0.7f, y), ImVec2(x + ts.x + pad * 0.7f, y + ts.y), u32(fadeTo(col, 0.16f)),
                          ts.y * 0.5f);
    dl->AddText(ImVec2(x, y), u32(col), note);
}

// A list row: its name (the Selectable's own label, so a script can click
// it) and a quieter note on the right.
bool listRow(const char* label, bool selected, const char* note, const ImVec4& noteCol) {
    const bool clicked = ImGui::Selectable(label, selected);
    noteRight(note, noteCol, ImGui::CalcTextSize(label, nullptr, true).x);
    return clicked;
}

// Text with a dark halo, legible over the mesh and the floor alike.
void haloText(ImDrawList* dl, ImVec2 p, ImU32 col, const char* text) {
    const ImU32 shade = IM_COL32(0, 0, 0, 170);
    dl->AddText(ImVec2(p.x + 1, p.y + 1), shade, text);
    dl->AddText(ImVec2(p.x - 1, p.y + 1), shade, text);
    dl->AddText(p, col, text);
}

const char* kControlsHelp =
    "Preview controls\n\n"
    "Click - select a body or a particle\n"
    "Ctrl+click - punch the body where you click\n"
    "Ctrl+drag - grab the body there and pull it\n"
    "Right-drag - orbit the camera\n"
    "Middle-drag - pan\n"
    "Wheel - zoom\n"
    "Ctrl+S - save the asset";

const char* kModeNames[] = {"Animation", "Ragdoll", "Motorized"};

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
            r.status = "No ragdoll for " + model + " yet: make one with New from model...";
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

    // --- the toolbar: the document, then the simulation, then the push and view
    const theme::Semantics& sm = theme::semantics();
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
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(labelCol(), "Asset");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(scaled(250));
    if (ImGui::BeginCombo("##Asset", r.assetRel.empty() ? "(none)" : r.assetRel.c_str())) {
        for (const std::string& a : assets)
            if (ImGui::Selectable(a.c_str(), a == r.assetRel) && a != r.assetRel) ragdollLoad(a);
        ImGui::EndCombo();
    }
    tip("The ragdoll being edited - every .ragdoll file under the project's res/ folder.");
    ImGui::SameLine();
    ImGui::BeginDisabled(!r.loaded);
    if (dirty ? tintButton("Save", sm.accent) : ImGui::Button("Save")) {
        std::string err;
        if (ragdoll::saveText(project_.filePath(r.assetRel), r.lastJson, err)) {
            r.savedJson = r.lastJson;
            r.status = "Saved " + r.assetRel;
        } else {
            r.status = err;
        }
    }
    tip("Writes the asset (Ctrl+S). The next build regenerates its C++ header.");
    ImGui::SameLine();
    if (ImGui::Button("Revert")) ragdollLoad(r.assetRel);
    tip("Throws away the unsaved edits and reloads the file.");
    vrule();
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
    tip("Undoes the last edit to the asset.");
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
    tip("Redoes the edit Undo took back.");
    ImGui::EndDisabled();
    vrule();
    if (ImGui::Button("New from model...")) ImGui::OpenPopup("New ragdoll");
    tip("Generates a ragdoll from an animated model's skeleton: its bodies, joint limits and two starter "
        "motor profiles.");
    if (ImGui::BeginPopup("New ragdoll")) {
        ImGui::Dummy(ImVec2(scaled(380), 0.0f));  // the popup's width: the rows fill it
        section("New ragdoll",
                "Puts a body on every bone long enough, sizes the particles' radii and masses from the "
                "skinned mesh, guesses hinges and limits from the bone names (knee, elbow, spine, neck...) "
                "and adds two starter motor profiles. It simulates as generated; tuning it is up to you.");
        const float lw = scaled(118);
        std::vector<std::string> models;
        for (const std::string& m : listAnimatedModelFiles()) models.push_back("res/models/" + m);
        prop("Model", lw, "An animated (skinned) model. Its import already wrote the skeleton.");
        nameCombo("##Model", r.newModel, models, false);
        prop("Min bone size", lw,
             "A bone shorter than this share of the model's height gets no body of its own: it rides its "
             "parent's.");
        ImGui::SliderFloat("##minbone", &r.genMinBone, 0.01f, 0.2f, "%.2f of the height");
        prop("Total mass", lw, "Shared out between the particles by their volume.");
        ImGui::DragFloat("##mass", &r.genMass, 0.5f, 5.0f, 500.0f, "%.1f kg");
        const std::string target = r.newModel.empty() ? "" : ragdoll::ragdollPathFor(r.newModel);
        std::error_code ec;
        const bool exists = !target.empty() && fs::exists(project_.filePath(target), ec);
        if (exists) {
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextColored(sm.warn, "%s exists - Generate replaces it.", target.c_str());
            ImGui::PopTextWrapPos();
        }
        ImGui::Spacing();
        ImGui::BeginDisabled(r.newModel.empty());
        if (tintButton("Generate", sm.accent)) {
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
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    if (!r.status.empty()) {
        // the last thing that happened, quietly, at the row's right end
        ImGui::SameLine();
        const float room = ImGui::GetContentRegionAvail().x;
        const float w = ImGui::CalcTextSize(r.status.c_str()).x;
        if (w < room) ImGui::SetCursorPosX(ImGui::GetCursorPosX() + room - w);
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("%s", r.status.c_str());
    }
    if (!r.loaded) {
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 40.0f);
        ImGui::TextDisabled(
            "Open a ragdoll from the Asset list or the Asset Browser, or make one with New from model... "
            "from an animated model (its import already wrote the skeleton).");
        if (!r.loadError.empty()) ImGui::TextColored(sm.danger, "%s", r.loadError.c_str());
        ImGui::PopTextWrapPos();
        ImGui::End();
        return;
    }

    // --- the simulation rows --------------------------------------------------
    {
        bool restart = false;
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(labelCol(), "Simulate");
        ImGui::SameLine();
        // Clicking the mode that is already on starts it OVER (the clip from 0
        // for Animation; the simulation from the pose it began in for the
        // others); another mode starts from the pose shown now.
        static const char* kModeTips[] = {
            "Plays the clip on the skeleton - no physics. Click it again to restart the clip.",
            "Takes the pose shown and lets go: the body falls, held together only by its joints and their "
            "limits. Push it with Shoot, Ctrl+click or Ctrl+drag. Click it again to start over.",
            "Animation and physics at once: motors pull every body toward the clip, as strongly as the "
            "motor profile says. Click it again to start over.",
        };
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(scaled(2), ImGui::GetStyle().ItemSpacing.y));
        for (int m = 0; m < 3; ++m) {
            if (m) ImGui::SameLine();
            if (segment(kModeNames[m], r.mode == m, scaled(86))) {
                if (r.mode == m) r.clipTime = m == RagdollEd::Animation ? 0.0f : r.startClipTime;
                r.mode = m;
                restart = true;
            }
            tip(kModeTips[m]);
        }
        ImGui::PopStyleVar();
        vrule();
        ImGui::TextColored(labelCol(), "Clip");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(scaled(140));
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
        tip("The model's animation. Animation plays it; Ragdoll starts from its current pose; Motorized "
            "pulls the bodies toward it as it plays.");
        ImGui::SameLine();
        ImGui::TextColored(labelCol(), "Motor profile");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(scaled(130));
        const char* profName = r.profile >= 0 && r.profile < (int)r.asset.profiles.size()
                                   ? r.asset.profiles[(size_t)r.profile].name.c_str()
                                   : "(none)";
        if (ImGui::BeginCombo("##Motor profile", profName)) {
            for (size_t i = 0; i < r.asset.profiles.size(); ++i)
                if (ImGui::Selectable(r.asset.profiles[i].name.c_str(), (int)i == r.profile)) r.pickProfile((int)i);
            ImGui::EndCombo();
        }
        tip("How the Motorized ragdoll follows the clip, body by body. The same choice as the Profiles list "
            "on the left: picking one here selects it there and shows it in the details, and the "
            "simulation starts over.");
        vrule();
        if (ImGui::Button(r.playing ? "Pause" : "Play", ImVec2(scaled(58), 0))) r.playing = !r.playing;
        tip("Pauses or resumes the preview.");
        ImGui::SameLine();
        if (ImGui::Button("Step")) {
            r.playing = false;
            r.acc += r.def.solver.fixedDt;
            r.clipTime += r.def.solver.fixedDt;
        }
        tip("Pauses, then advances one fixed simulation step.");
        ImGui::SameLine();
        if (ImGui::Button("Reset")) {
            r.clipTime = 0.0f;
            restart = true;
        }
        tip("Starts the clip and the simulation over from the beginning.");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(scaled(110));
        fillSlider("##Speed", &r.speed, 0.05f, 2.0f, "speed %.2fx");
        tip("How fast the preview runs: below 1 is slow motion.");

        // the second row: the push, the world, what the preview shows
        ImGui::BeginDisabled(r.mode == RagdollEd::Animation);
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(labelCol(), "Push");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(scaled(84));
        ImGui::DragFloat("##pushs", &r.pushSpeed, 0.1f, 0.0f, 30.0f, "%.1f m/s");
        tip("How hard Shoot and the Ctrl+click punch hit: the speed they give the body, in m/s.");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(scaled(124));
        ImGui::Combo("##pushd", &r.pushDir, kPushNames, 5);
        tip("The direction Shoot pushes the body in.");
        ImGui::SameLine();
        if (ImGui::Button("Shoot")) {
            // at the chest: 70% up the body as it stands now
            float lo = 1e9f, hi = -1e9f;
            for (const V3& p : r.sim.particles()) lo = std::min(lo, p.y), hi = std::max(hi, p.y);
            r.sim.applyPush(V3(0.0f, lo + 0.7f * (hi - lo), 0.0f) + V3(r.center[0], 0, r.center[2]),
                            pushVector(r.pushDir, r.pushSpeed));
        }
        tip("Hits the body at the chest in the chosen direction, like a bullet. To hit it somewhere else, "
            "Ctrl+click it in the preview.");
        ImGui::EndDisabled();
        vrule();
        if (ImGui::Checkbox("Gravity", &r.gravity)) r.sim.setGravity(r.gravity);
        tip("Turn it off to watch the joints and the motors alone.");
        ImGui::SameLine();
        if (ImGui::Checkbox("Stairs", &r.stairs)) restart = true;
        tip("Stands the body on a landing above a flight of ten 15 cm steps, to see how it tumbles down.");
        vrule();
        ImGui::SetNextItemWidth(scaled(96));
        if (ImGui::BeginCombo("##Show", "Show", ImGuiComboFlags_HeightLargest)) {
            auto show = [](const char* label, bool* v, const char* help) {
                ImGui::Checkbox(label, v);
                tip(help);
            };
            show("Mesh", &r.showMesh, "The skinned model, posed by the simulation.");
            show("Wireframe", &r.wire, "The mesh's triangles over it.");
            show("Bodies", &r.showBodies, "Every body as a capsule from its pivot to its end.");
            show("Particles", &r.showParticles,
                 "The particles as dots; the selection and the hover show their collision radius.");
            show("Links", &r.showLinks, "The distance constraints between particles.");
            show("Bones", &r.showBones, "The skeleton's bones.");
            show("Limits", &r.showLimits, "The selected body's swing limit: the cone of directions it may point in.");
            show("Angle axes", &r.showAxes, "The selected body's pitch (red), yaw (green) and roll (blue) axes.");
            show("Motor targets", &r.showTargets, "Motorized: where the clip wants each particle (green crosses).");
            ImGui::EndCombo();
        }
        tip("What the preview draws.");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(scaled(96));
        if (ImGui::BeginCombo("##View", "View")) {
            if (ImGui::Selectable("Reset view")) r.yaw = 40.0f, r.pitch = 15.0f, r.zoom = 1.0f, r.panX = r.panY = 0.0f;
            ImGui::TextColored(labelCol(), "Light");
            ImGui::SetNextItemWidth(scaled(180));
            if (previewLightCombo("##rag_light", r.light)) saveGlobalConfig();
            ImGui::EndCombo();
        }
        tip("The preview's camera and light.");
        helpMarker(kControlsHelp);
        if (r.restartReq) {
            r.restartReq = false;
            r.clipTime = r.mode == RagdollEd::Animation ? 0.0f : r.startClipTime;
            restart = true;
        }
        if (restart && r.mode != RagdollEd::Animation) ragdollStart();
    }
    ImGui::Dummy(ImVec2(0.0f, scaled(2)));

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
    const theme::Semantics& sm = theme::semantics();
    static const char* kSections[] = {"Skeleton", "Bodies", "Particles", "Profiles", "Solver"};
    static const char* kSectionHelp[] = {
        "The model's bones. A bone that carries a body shows the body's name on the right - click it to "
        "edit the body. A bone without one moves with the nearest body above it; select it to give it a "
        "body of its own.",
        "The simulated parts. Each body is a segment from its pivot particle to its end particle, hung on "
        "its parent body at the pivot; the first one is the root. Clusters hold groups of particles rigid "
        "together.",
        "The points the simulation moves. Each rides a bone and has a radius (what touches the ground) and "
        "a mass; the bodies are built between them.",
        "Motor profiles drive a Motorized ragdoll: per body, whether it follows the clip in the world "
        "(absolute) or against its parent (relative), and how strongly. Picking one here also previews "
        "it.",
        "The settings of the preview's simulation. They drive only this editor: a game that simulates the "
        "generated data keeps its own settings in its code.",
    };
    const float helpW = ImGui::CalcTextSize("(?)").x + ImGui::GetStyle().ItemSpacing.x;
    ImGui::SetNextItemWidth(-helpW);
    ImGui::Combo("##Section", &r.leftTab, kSections, 5);
    helpMarker(kSectionHelp[std::clamp(r.leftTab, 0, 4)]);
    ImGui::Spacing();
    ImGui::BeginChild("##rag_section");
    if (r.leftTab == 0) {
        // bones, as a tree; a bone carrying a body names it on the right
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
        const ImVec4 quiet = mixTo(sm.text, sm.textDim, 0.55f);
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
            std::snprintf(label, sizeof(label), "%s##bone%d", bone.name.c_str(), i);
            // the bones that carry a body read first; the rest step back
            if (body < 0) ImGui::PushStyleColor(ImGuiCol_Text, quiet);
            const bool open = ImGui::TreeNodeEx(label, fl);
            if (body < 0) ImGui::PopStyleColor();
            if (body >= 0)
                noteRight(r.def.bodies[(size_t)body].name.c_str(), sm.accent,
                          ImGui::GetTreeNodeToLabelSpacing() + ImGui::CalcTextSize(bone.name.c_str()).x, true);
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
            std::snprintf(label, sizeof(label), "%s##b%zu", a.bodies[i].name.c_str(), i);
            const char* note = a.bodies[i].parent.empty() ? "root" : a.bodies[i].hinge ? "hinge" : "";
            if (listRow(label, r.selKind == RagdollEd::Body && r.sel == (int)i, note, sm.textDim))
                r.selKind = RagdollEd::Body, r.sel = (int)i;
        }
        section("Clusters",
                "Groups of particles held rigid together - every pair linked - like a pelvis with its hips "
                "or a chest with its shoulders. A child body whose pivot is not its parent's end particle "
                "needs that pivot in its parent's cluster.");
        for (size_t c = 0; c < a.clusters.size(); ++c) {
            ImGui::PushID((int)c);
            std::vector<std::string>& cl = a.clusters[c];
            ImGui::AlignTextToFramePadding();
            ImGui::TextColored(labelCol(), "Cluster %zu", c + 1);
            ImGui::SameLine();
            const float delW = ImGui::CalcTextSize("Delete").x + ImGui::GetStyle().FramePadding.x * 2.0f;
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.0f, ImGui::GetContentRegionAvail().x - delW));
            if (tintButton("Delete", sm.danger, ImVec2(0, 0), true)) {
                a.clusters.erase(a.clusters.begin() + (long)c);
                ImGui::PopID();
                break;
            }
            tip("Deletes this cluster: its particles are no longer held together.");
            // its particles as chips; a chip's x takes it out
            ImGui::PushStyleColor(ImGuiCol_Button, fadeTo(sm.text, 0.07f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, fadeTo(sm.danger, 0.30f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, fadeTo(sm.danger, 0.45f));
            for (size_t k = 0; k < cl.size(); ++k) {
                char chip[96];
                std::snprintf(chip, sizeof(chip), "%s  \xc3\x97##c%zu", cl[k].c_str(), k);
                const float w = ImGui::CalcTextSize(chip, nullptr, true).x + ImGui::GetStyle().FramePadding.x * 2.0f;
                if (k) {
                    ImGui::SameLine(0.0f, ImGui::GetStyle().ItemSpacing.x * 0.5f);
                    if (ImGui::GetContentRegionAvail().x < w) ImGui::NewLine();
                }
                if (ImGui::SmallButton(chip)) {
                    cl.erase(cl.begin() + (long)k);
                    break;
                }
                tip("Takes the particle out of the cluster.");
            }
            ImGui::PopStyleColor(3);
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (ImGui::BeginCombo("##add", "Add a particle...")) {
                for (const ragdoll::Particle& p : a.particles)
                    if (std::find(cl.begin(), cl.end(), p.name) == cl.end() && ImGui::Selectable(p.name.c_str()))
                        cl.push_back(p.name);
                ImGui::EndCombo();
            }
            ImGui::Dummy(ImVec2(0.0f, ImGui::GetStyle().ItemSpacing.y));
            ImGui::PopID();
        }
        if (ImGui::Button("New cluster", ImVec2(-FLT_MIN, 0))) a.clusters.push_back({});
        tip("Adds an empty cluster; add particles to it from its list.");
    }
    if (r.leftTab == 2) {
        for (size_t i = 0; i < a.particles.size(); ++i) {
            char label[128];
            std::snprintf(label, sizeof(label), "%s##p%zu", a.particles[i].name.c_str(), i);
            if (listRow(label, r.selKind == RagdollEd::Particle && r.sel == (int)i, a.particles[i].bone.c_str(),
                        sm.textDim))
                r.selKind = RagdollEd::Particle, r.sel = (int)i;
        }
        ImGui::Spacing();
        if (ImGui::Button("Add particle", ImVec2(-FLT_MIN, 0))) {
            ragdoll::Particle p;
            p.name = "particle";
            for (int k = 1; a.particle(p.name) >= 0; ++k) p.name = "particle_" + std::to_string(k);
            p.bone = r.skel.bones.empty() ? "" : r.skel.bones[0].name;
            if (r.selKind == RagdollEd::Bone && r.sel >= 0) p.bone = r.skel.bones[(size_t)r.sel].name;
            a.particles.push_back(p);
            r.selKind = RagdollEd::Particle, r.sel = (int)a.particles.size() - 1;
        }
        tip("Adds a particle at the origin of the selected bone (else the first bone).");
    }
    if (r.leftTab == 3) {
        for (size_t i = 0; i < a.profiles.size(); ++i) {
            char label[128];
            std::snprintf(label, sizeof(label), "%s##pr%zu", a.profiles[i].name.c_str(), i);
            // the row marked is the toolbar's Motor profile: one choice, two places
            const bool previewing = r.mode == RagdollEd::Motorized && r.profile == (int)i;
            if (listRow(label, r.profile == (int)i, previewing ? "previewing" : "", sm.accent)) r.pickProfile((int)i);
        }
        ImGui::Spacing();
        if (ImGui::Button("New profile", ImVec2(-FLT_MIN, 0))) {
            ragdoll::Profile p;
            p.name = "profile";
            for (int k = 1; a.profile(p.name); ++k) p.name = "profile_" + std::to_string(k);
            for (const ragdoll::Body& b : a.bodies) p.joints.push_back({b.name, false, 0.5f});
            a.profiles.push_back(p);
            r.pickProfile((int)a.profiles.size() - 1);
        }
        tip("Adds a profile with every body absolute at half strength.");
    }
    if (r.leftTab == 4) {
        ragdoll::Solver& s = a.solver;
        const ragdoll::Solver defaults;
        // the saved solver, re-read only when the asset is saved again
        if (r.savedSolverOf != r.savedJson) {
            ragdoll::Asset saved;
            std::string err;
            if (ragdoll::parseAsset(r.savedJson, saved, err)) r.savedSolver = saved.solver;
            r.savedSolverOf = r.savedJson;
        }
        const float lw = scaled(120);
        // a row: the label, the value, and its undo / default slots at the far right
        auto f = [&](const char* label, const char* id, float ragdoll::Solver::*m, float lo, float hi,
                     const char* fmt, const char* help) {
            prop(label, lw, help);
            ImGui::SetNextItemWidth(-valueResetsWidth());
            ImGui::DragFloat(id, &(s.*m), (hi - lo) / 500.0f, lo, hi, fmt);
            float back = 0.0f;
            if (valueResets(id, s.*m, r.savedSolver.*m, defaults.*m, fmt, back)) s.*m = back;
        };
        section("Simulation",
                "The preview's simulation. A value you changed shows, at the end of its row, an arrow back "
                "to the value last saved and a yellow arrow back to the default.");
        prop("Iterations", lw, "Constraint passes a step: more holds the joints stiffer, and costs more.");
        ImGui::SetNextItemWidth(-valueResetsWidth());
        ImGui::DragInt("##iterations", &s.iterations, 0.1f, 1, 20);
        {
            float back = 0.0f;
            if (valueResets("##iterations", (float)s.iterations, (float)r.savedSolver.iterations,
                            (float)defaults.iterations, "%.0f", back))
                s.iterations = (int)std::lround(back);
        }
        f("Gravity", "##gravity", &ragdoll::Solver::gravity, 0, 30, "%.2f m/s\xc2\xb2", "The downward acceleration.");
        f("Damping", "##damping", &ragdoll::Solver::damping, 0.9f, 1.0f, "%.3f",
          "The share of its velocity a particle keeps each step: lower settles sooner.");
        section("Ground");
        f("Static friction", "##sfric", &ragdoll::Solver::staticFriction, 0, 2, "%.2f",
          "A contact grips the ground until its slide in a step is more than this times how hard the "
          "ground pushes back.");
        f("Sliding friction", "##kfric", &ragdoll::Solver::kineticFriction, 0, 2, "%.2f",
          "Once it slides, the slide is cut by this times the ground's push back.");
        f("Rolling", "##rolling", &ragdoll::Solver::rolling, 0, 1, "%.2f", "The share of its speed a grounded particle loses each step.");
        section("Joints");
        f("Limit stiffness", "##limstiff", &ragdoll::Solver::limitStiffness, 0, 1, "%.2f",
          "The share of a joint-limit overshoot undone each pass: 1 is a hard stop.");
        f("Joint damping", "##jdamp", &ragdoll::Solver::jointDamping, 0, 1, "%.2f", "The share of a joint's swing speed lost each step.");
        section("Motors");
        f("Motor gain", "##mgain", &ragdoll::Solver::motorGain, 0, 1, "%.2f",
          "How much of the gap to its target a full-strength motor closes each step.");
        f("Motor velocity", "##mvel", &ragdoll::Solver::motorVelocity, 0, 1, "%.2f",
          "The share of a motor's correction kept as velocity: higher swings through, lower just places.");
        f("When down", "##mlying", &ragdoll::Solver::motorLying, 0, 1, "%.2f",
          "The strength left to the bodies marked 'Fade once down' after the pelvis reaches the ground.");
        f("Fade time", "##mfade", &ragdoll::Solver::motorFadeTime, 0, 3, "%.2f s", "How long that fade takes.");
        section("Pushes");
        f("Push radius", "##pradius", &ragdoll::Solver::pushRadius, 0, 2, "%.2f m", "A push falls off over this distance from where it hits.");
        f("Push topple", "##ptopple", &ragdoll::Solver::pushTopple, 0, 1, "%.2f",
          "The share of a push applied at the top of the body, turning it about its feet.");
        section("Sleep");
        f("Sleep speed", "##sspeed", &ragdoll::Solver::sleepSpeed, 0, 1, "%.2f m/s", "Below this speed a particle counts as still.");
        f("Sleep time", "##stime", &ragdoll::Solver::sleepTime, 0, 3, "%.2f s",
          "Still for this long, the body falls asleep and stops simulating.");
        f("Max awake", "##sawake", &ragdoll::Solver::maxAwake, 0, 30, "%.1f s", "A plain ragdoll falls asleep after this long regardless.");
        ImGui::Spacing();
        if (ImGui::Button("Reset to defaults", ImVec2(-FLT_MIN, 0))) s = ragdoll::Solver();
        tip("Puts every setting above back to its default.");
    }
    ImGui::EndChild();
}

// ----------------------------------------------------------------- details --

void App::drawRagdollDetails() {
    RagdollEd& r = *rag_;
    ragdoll::Asset& a = r.asset;
    const theme::Semantics& sm = theme::semantics();
    const float lw = scaled(96);
    if (!r.errors.empty()) {
        ImGui::PushStyleColor(ImGuiCol_ChildBg, fadeTo(sm.danger, 0.10f));
        ImGui::PushStyleColor(ImGuiCol_Border, fadeTo(sm.danger, 0.45f));
        ImGui::BeginChild("##rag_errors", ImVec2(0, 0), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);
        ImGui::TextColored(sm.danger, "It cannot simulate - %zu problem%s", r.errors.size(),
                           r.errors.size() == 1 ? "" : "s");
        for (const std::string& e : r.errors) {
            ImGui::Bullet();
            ImGui::SameLine();
            ImGui::TextWrapped("%s", e.c_str());
        }
        ImGui::EndChild();
        ImGui::PopStyleColor(2);
    }
    std::vector<std::string> pnames, bnames, bones;
    for (const ragdoll::Particle& p : a.particles) pnames.push_back(p.name);
    for (const ragdoll::Body& b : a.bodies) bnames.push_back(b.name);
    for (const ragdoll::Bone& b : r.skel.bones) bones.push_back(b.name);
    auto footer = [] {
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
    };

    if (r.selKind == RagdollEd::Body && r.sel >= 0 && r.sel < (int)a.bodies.size()) {
        ragdoll::Body& b = a.bodies[(size_t)r.sel];
        section("Body", "A simulated part of the ragdoll: a segment from its pivot particle to its end particle, "
                        "hung on its parent body at the pivot.");
        std::string name = b.name;
        prop("Name", lw);
        if (inputName("##name", name) && a.body(name) < 0) {
            for (ragdoll::Body& o : a.bodies)
                if (o.parent == b.name) o.parent = name;
            for (ragdoll::Profile& p : a.profiles)
                for (ragdoll::ProfileJoint& j : p.joints)
                    if (j.body == b.name) j.body = name;
            b.name = name;
        }
        tip("Press Enter to rename. Profiles and child bodies follow the new name.");
        prop("Pivot", lw, "The particle the body turns about - where it hangs on its parent.");
        nameCombo("##pivot", b.pivot, pnames, false);
        prop("End", lw, "The particle at the body's far end. Pivot to end is the body's axis.");
        nameCombo("##end", b.end, pnames, false);
        prop("Parent", lw,
             "The body this one hangs from. Only bodies above it in the list can be its parent; the first "
             "body is the root.");
        if (r.sel > 0) {
            std::vector<std::string> earlier(bnames.begin(), bnames.begin() + r.sel);
            nameCombo("##parent", b.parent, earlier, false);
        } else {
            ImGui::TextDisabled("none - the root body");
        }
        prop("Side +X", lw,
             "Optional: a particle on the body's +X side. With the -X one it gives the body a frame of its "
             "own to twist about - a pelvis on its hips, a chest on its shoulders. The root needs a pair.");
        nameCombo("##sideA", b.sideA, pnames, true);
        prop("Side -X", lw, "The particle on the body's -X side; see Side +X.");
        nameCombo("##sideB", b.sideB, pnames, true);

        section("Joint limits",
                "How far the body may turn against its parent, in degrees from the bind pose. Pitch swings "
                "the far end toward the pitch direction, yaw toward the yaw direction, and roll twists the "
                "body about its own axis. The preview draws the selected body's axes in the same colours.");
        prop("Hinge", lw, "Bends in pitch only, like a knee or an elbow: yaw stays at 0.");
        ImGui::Checkbox("##hinge", &b.hinge);
        if (ImGui::BeginTable("##limits", 3, ImGuiTableFlags_None)) {
            ImGui::TableSetupColumn("axis", ImGuiTableColumnFlags_WidthFixed,
                                    lw - ImGui::GetStyle().CellPadding.x * 2.0f);
            ImGui::TableSetupColumn("min", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("max", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(1);
            ImGui::TextColored(labelCol(), "min");
            ImGui::TableSetColumnIndex(2);
            ImGui::TextColored(labelCol(), "max");
            auto limitRow = [&](const char* axis, float* v, const ImVec4& col, const char* help, bool off) {
                ImGui::PushID(axis);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::TextColored(col, "%s", axis);
                helpMarker(help);
                ImGui::BeginDisabled(off);
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-FLT_MIN);
                ImGui::DragFloat("##min", &v[0], 0.5f, -180.0f, v[1], "%.0f\xc2\xb0");
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-FLT_MIN);
                ImGui::DragFloat("##max", &v[1], 0.5f, v[0], 180.0f, "%.0f\xc2\xb0");
                ImGui::EndDisabled();
                ImGui::PopID();
            };
            limitRow("Pitch", b.pitch, kPitchCol, "+ swings the far end toward the pitch direction (red).", false);
            limitRow("Yaw", b.yaw, kYawCol, "+ swings the far end toward the yaw direction (green). A hinge has none.",
                     b.hinge);
            limitRow("Roll", b.roll, kRollCol,
                     "The twist about the body's own axis (blue). It acts on a body with a side pair against "
                     "its parent, and on a hinge's parent through the hinge's bend plane.",
                     false);
            ImGui::EndTable();
        }
        prop("Pitch dir", lw, "The direction + pitch swings the far end toward.", &kPitchCol);
        ImGui::DragFloat3("##pitchdir", b.pitchDir, 0.01f, -1.0f, 1.0f, "%.2f");
        ImGui::BeginDisabled(b.hinge);
        prop("Yaw dir", lw, "The direction + yaw swings the far end toward.", &kYawCol);
        ImGui::DragFloat3("##yawdir", b.yawDir, 0.01f, -1.0f, 1.0f, "%.2f");
        ImGui::EndDisabled();
        if (r.mode != RagdollEd::Animation && r.sim.started() && r.sel < (int)r.def.bodies.size()) {
            std::vector<float> ang;
            r.sim.jointAngles(ang);
            const float p = ang[(size_t)r.sel * 3], y = ang[(size_t)r.sel * 3 + 1], rl = ang[(size_t)r.sel * 3 + 2];
            const bool over = p < b.pitch[0] - 0.5f || p > b.pitch[1] + 0.5f ||
                              (!b.hinge && (y < b.yaw[0] - 0.5f || y > b.yaw[1] + 0.5f));
            prop("Now", lw, "The joint's angles at this moment of the simulation.");
            ImGui::TextColored(kPitchCol, "%.0f\xc2\xb0", (double)p);
            ImGui::SameLine();
            ImGui::TextColored(kYawCol, "%.0f\xc2\xb0", (double)y);
            ImGui::SameLine();
            ImGui::TextColored(kRollCol, "%.0f\xc2\xb0", (double)rl);
            if (over) {
                ImGui::SameLine();
                ImGui::TextColored(sm.warn, "past a limit");
            }
        }
        ImGui::Spacing();
        if (ImGui::Button("Reset from the bone name", ImVec2(-FLT_MIN, 0))) {
            const int pi = a.particle(b.pivot), ei = a.particle(b.end);
            if (pi >= 0 && ei >= 0 && pi < (int)r.def.particles.size() && ei < (int)r.def.particles.size()) {
                const std::string boneName = a.particles[(size_t)pi].bone;
                ragdoll::defaultJoint(boneName, b.parent.empty(), r.def.particles[(size_t)pi].bind,
                                      r.def.particles[(size_t)ei].bind, b);
            }
        }
        tip("Replaces the hinge, the limits and the directions with the defaults guessed from the pivot "
            "bone's name (knee, elbow, spine, neck...).");

        section("Motors");
        ImGui::Checkbox("Fade once down", &b.groundFade);
        helpMarker("For what carries the weight - legs, pelvis, chest. Once the pelvis is down, a Motorized "
                   "ragdoll's motors on this body weaken, so a fallen body stops fighting the ground.");
        footer();
        if (dangerButton("Delete body")) {
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
        tip("Removes the body. The bodies hanging on it hang on its parent instead.");
    } else if (r.selKind == RagdollEd::Particle && r.sel >= 0 && r.sel < (int)a.particles.size()) {
        ragdoll::Particle& p = a.particles[(size_t)r.sel];
        section("Particle", "A point the simulation moves. It rides a bone, and the bodies are built between particles.");
        std::string name = p.name;
        prop("Name", lw);
        if (inputName("##name", name) && a.particle(name) < 0) {
            for (ragdoll::Body& b : a.bodies)
                for (std::string* s : {&b.pivot, &b.end, &b.sideA, &b.sideB})
                    if (*s == p.name) *s = name;
            for (auto& c : a.clusters)
                for (std::string& n : c)
                    if (n == p.name) n = name;
            p.name = name;
        }
        tip("Press Enter to rename. Bodies and clusters follow the new name.");
        prop("Bone", lw, "The bone it rides: where it starts, and the bone it moves in the simulation.");
        nameCombo("##bone", p.bone, bones, false);
        prop("Offset", lw, "Where it sits in the bone's bind frame, in metres.");
        ImGui::DragFloat3("##offset", p.offset, 0.002f, -2.0f, 2.0f, "%.3f");
        prop("Radius", lw, "Its collision sphere: what touches the ground and the props.");
        ImGui::DragFloat("##radius", &p.radius, 0.001f, 0.005f, 1.0f, "%.3f m");
        prop("Mass", lw, "A heavier particle moves less when it is pushed or pulled.");
        ImGui::DragFloat("##mass", &p.mass, 0.05f, 0.05f, 200.0f, "%.2f kg");
        ImGui::Checkbox("Collides with props", &p.world);
        helpMarker("In a game, it also collides with the level's props, not only the ground.");
        footer();
        bool used = false;
        for (const ragdoll::Body& b : a.bodies)
            used |= b.pivot == p.name || b.end == p.name || b.sideA == p.name || b.sideB == p.name;
        ImGui::BeginDisabled(used);
        if (dangerButton("Delete particle")) {
            const std::string gone = p.name;
            for (auto& c : a.clusters) c.erase(std::remove(c.begin(), c.end(), gone), c.end());
            a.particles.erase(a.particles.begin() + r.sel);
            r.selKind = RagdollEd::None, r.sel = -1;
        }
        ImGui::EndDisabled();
        tip(used ? "A body is built on it: delete or change that body first." : "Removes the particle.");
    } else if (r.selKind == RagdollEd::Profile && r.sel >= 0 && r.sel < (int)a.profiles.size()) {
        ragdoll::Profile& pr = a.profiles[(size_t)r.sel];
        section("Motor profile",
                "How a Motorized ragdoll follows its clip, body by body.\n\n"
                "Absolute: the body is pulled toward where the clip puts it in the world - it can stand and "
                "walk.\n"
                "Relative: toward its pose against its parent body as it is now - it shapes a fall but never "
                "lifts the body.\n\n"
                "Strength is how hard, from 0 (loose) to 1.");
        std::string name = pr.name;
        prop("Name", lw);
        if (inputName("##name", name) && !a.profile(name)) pr.name = name;
        tip("Press Enter to rename.");
        static float all = 0.5f;
        prop("Every body", lw, "Gives every body the same strength.");
        const float applyW = ImGui::CalcTextSize("Apply").x + ImGui::GetStyle().FramePadding.x * 2.0f;
        ImGui::SetNextItemWidth(-(applyW + ImGui::GetStyle().ItemSpacing.x));
        fillSlider("##all", &all, 0.0f, 1.0f, "%.2f");
        ImGui::SameLine();
        if (ImGui::Button("Apply")) {
            for (const ragdoll::Body& b : a.bodies) {
                bool found = false;
                for (ragdoll::ProfileJoint& j : pr.joints)
                    if (j.body == b.name) j.ratio = all, found = true;
                if (!found) pr.joints.push_back({b.name, false, all});
            }
        }
        prop("Every mode", lw, "Switches every body to relative or to absolute.");
        const float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
        if (ImGui::Button("All relative", ImVec2(half, 0)))
            for (ragdoll::ProfileJoint& j : pr.joints) j.relative = true;
        ImGui::SameLine();
        if (ImGui::Button("All absolute", ImVec2(-FLT_MIN, 0)))
            for (ragdoll::ProfileJoint& j : pr.joints) j.relative = false;
        ImGui::Spacing();
        if (ImGui::BeginTable("##prof", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_PadOuterX)) {
            ImGui::TableSetupColumn("Body", ImGuiTableColumnFlags_WidthStretch, 1.0f);
            ImGui::TableSetupColumn("Mode", ImGuiTableColumnFlags_WidthFixed, scaled(76));
            ImGui::TableSetupColumn("Strength", ImGuiTableColumnFlags_WidthStretch, 1.1f);
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
                ImGui::AlignTextToFramePadding();
                ImGui::TextUnformatted(b.name.c_str());
                ImGui::TableNextColumn();
                // the mode as a tinted toggle: relative in the accent, absolute in warn
                if (tintButton(j->relative ? "Relative" : "Absolute", j->relative ? sm.accent : sm.warn,
                               ImVec2(-FLT_MIN, 0), true))
                    j->relative = !j->relative;
                tip("Click to switch. Absolute: toward the clip's pose in the world. Relative: toward its pose "
                    "against its parent.");
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-FLT_MIN);
                fillSlider("##ratio", &j->ratio, 0.0f, 1.0f, "%.2f");
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        footer();
        if (ImGui::Button("Duplicate")) {
            ragdoll::Profile c = pr;
            c.name += "_copy";
            for (int k = 2; a.profile(c.name); ++k) c.name = pr.name + "_copy" + std::to_string(k);
            a.profiles.push_back(c);
            r.pickProfile((int)a.profiles.size() - 1);
        }
        tip("Adds a copy of this profile and picks it.");
        ImGui::SameLine();
        if (dangerButton("Delete profile")) {
            a.profiles.erase(a.profiles.begin() + r.sel);
            const int next = std::min(r.sel, (int)a.profiles.size() - 1);  // -1 when none is left
            r.pickProfile(next);
            if (next < 0) r.selKind = RagdollEd::None;
        }
        tip("Removes the profile.");
    } else if (r.selKind == RagdollEd::Bone && r.sel >= 0 && r.sel < (int)r.skel.bones.size()) {
        const ragdoll::Bone& bone = r.skel.bones[(size_t)r.sel];
        section("Bone", "A bone of the skeleton. Without a body of its own it moves with the nearest body above it.");
        prop("Name", lw);
        ImGui::TextUnformatted(bone.name.c_str());
        const int follows = r.sel < (int)r.def.boneBody.size() ? r.def.boneBody[(size_t)r.sel] : -1;
        prop("Moves with", lw, "The body that carries this bone in the simulation.");
        if (follows >= 0)
            ImGui::TextColored(sm.accent, "%s", r.def.bodies[(size_t)follows].name.c_str());
        else
            ImGui::TextDisabled("no body");
        ImGui::Spacing();
        if (tintButton("Add a body on this bone", sm.accent, ImVec2(-FLT_MIN, 0))) {
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
        tip("Gives this bone a body of its own: particles at the bone and at its child, hung on the body it "
            "moves with now, with limits guessed from the bone's name.");
    } else {
        section("Ragdoll");
        prop("Asset", lw);
        ImGui::TextUnformatted(r.assetRel.c_str());
        prop("Skeleton", lw,
             "The bone hierarchy the ragdoll is built on, written by the model's import. Models that share "
             "a skeleton can share its ragdoll.");
        ImGui::TextUnformatted(a.skeleton.c_str());
        prop("Model", lw, "The animated model the preview shows and takes its clips from.");
        if (r.modelOk)
            ImGui::TextUnformatted(r.skel.model.c_str());
        else
            ImGui::TextColored(sm.warn, "%s (cannot read it)", r.skel.model.c_str());
        section("Contents");
        float mass = 0.0f;
        for (const ragdoll::Particle& p : a.particles) mass += p.mass;
        prop("Bones", lw);
        ImGui::Text("%zu", r.skel.bones.size());
        prop("Bodies", lw);
        ImGui::Text("%zu", r.def.bodies.size());
        prop("Particles", lw);
        ImGui::Text("%zu", r.def.particles.size());
        prop("Links", lw, "Distance constraints between particles: one along each body, and every pair in a cluster.");
        ImGui::Text("%zu", r.def.links.size());
        prop("Profiles", lw);
        ImGui::Text("%zu", a.profiles.size());
        prop("Total mass", lw);
        ImGui::Text("%.1f kg", (double)mass);
        section("Generated header",
                "Every build writes the ragdoll as a C++ data header a game can include: its particles, "
                "bodies, links, the bone each skin joint follows, and the motor profiles. With no path, no "
                "header is written.");
        char buf[256];
        prop("Header", lw, "Where the build writes it, relative to the project.");
        std::snprintf(buf, sizeof(buf), "%s", a.header.c_str());
        if (ImGui::InputText("##header", buf, sizeof(buf))) a.header = buf;
        prop("Namespace", lw, "The C++ namespace its tables are written in.");
        std::snprintf(buf, sizeof(buf), "%s", a.nameSpace.c_str());
        if (ImGui::InputText("##namespace", buf, sizeof(buf))) a.nameSpace = buf;
        ImGui::Spacing();
        ImGui::BeginDisabled(a.header.empty() || !r.defOk);
        if (ImGui::Button("Write header now", ImVec2(-FLT_MIN, 0))) {
            std::string err;
            if (ragdoll::saveText(project_.filePath(a.header),
                                  ragdoll::headerText(a, r.skel, r.def, r.assetRel), err))
                r.status = "Wrote " + a.header;
            else
                r.status = err;
        }
        ImGui::EndDisabled();
        tip("Writes the header right away instead of at the next build.");
        ImGui::Spacing();
        ImGui::Spacing();
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("Select a body, a particle or a bone - in the preview or the lists - to edit it.");
        ImGui::PopTextWrapPos();
    }
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
    const ragdoll::Def& def = r.def;
    const bool have = r.defOk && parts.size() == def.particles.size();
    const float px = scaled(1.0f);
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
            const float reach = std::max(6.0f * px, pxRadius(parts[k], def.particles[k].radius));
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
            const float reach = std::max(6.0f * px, ra + (rb - ra) * t);
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
    // the body or particle under the mouse is lit, so a click is never a guess
    int hovPart = -1, hovBody = -1;
    if (have && hovered && !active && !r.grabbing) {
        float along;
        pick(hovPart, hovBody, along);
    }

    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->PushClipRect(origin, ImVec2(origin.x + (float)pw, origin.y + (float)ph), true);
    // One strong colour at a time: the bodies are a quiet wash of the theme's
    // accent, the selection its warn colour, the hover its text colour.
    const theme::Semantics& sm = theme::semantics();
    const ImU32 cLink = u32(fadeTo(sm.text, 0.30f)), cTarget = u32(fadeTo(sm.ok, 0.90f)),
                cLimit = u32(fadeTo(sm.warn, 0.85f)), cBone = u32(fadeTo(sm.text, 0.50f)),
                cGrab = u32(fadeTo(sm.danger, 0.95f));
    // A capsule: the two end circles joined by their outer tangents, as ONE
    // convex outline - its fill never doubles up where the shapes overlap.
    std::vector<ImVec2> hull;
    auto capsule = [&](ImVec2 a0, float ra, ImVec2 a1, float rb, ImU32 fill, ImU32 line, float thick) {
        const float dx = a1.x - a0.x, dy = a1.y - a0.y, l = std::hypot(dx, dy);
        if (l <= std::fabs(ra - rb) + 0.5f) {  // one circle holds the other
            const ImVec2 c = ra > rb ? a0 : a1;
            const float rr = std::max(ra, rb);
            dl->AddCircleFilled(c, rr, fill, 32);
            dl->AddCircle(c, rr, line, 32, thick);
            return;
        }
        const float base = std::atan2(dy, dx), th = std::acos(std::clamp((ra - rb) / l, -1.0f, 1.0f));
        const int n = 14;
        hull.clear();
        for (int i = 0; i <= n; ++i) {
            const float t = base - th + 2.0f * th * (float)i / (float)n;
            hull.push_back(ImVec2(a1.x + rb * std::cos(t), a1.y + rb * std::sin(t)));
        }
        for (int i = 0; i <= n; ++i) {
            const float t = base + th + (6.2831853f - 2.0f * th) * (float)i / (float)n;
            hull.push_back(ImVec2(a0.x + ra * std::cos(t), a0.y + ra * std::sin(t)));
        }
        dl->AddConvexPolyFilled(hull.data(), (int)hull.size(), fill);
        dl->AddPolyline(hull.data(), (int)hull.size(), line, ImDrawFlags_Closed, thick);
    };
    if (r.showBones) {
        for (size_t i = 0; i < r.skel.bones.size(); ++i) {
            const int p = r.skel.bones[i].parent;
            if (p < 0 || r.bones.size() < (i + 1) * 16) continue;
            ImVec2 a0, a1;
            if (project(V3(r.bones[i * 16 + 12], r.bones[i * 16 + 13], r.bones[i * 16 + 14]), a0) &&
                project(V3(r.bones[(size_t)p * 16 + 12], r.bones[(size_t)p * 16 + 13], r.bones[(size_t)p * 16 + 14]), a1))
                dl->AddLine(a0, a1, cBone, px);
        }
    }
    if (have && r.showLinks)
        for (const auto& l : def.links) {
            ImVec2 a0, a1;
            if (project(parts[(size_t)l.first], a0) && project(parts[(size_t)l.second], a1))
                dl->AddLine(a0, a1, cLink, px);
        }
    const int selBody = r.selKind == RagdollEd::Body ? r.sel : -1;
    if (have && r.showBodies) {
        auto drawBody = [&](size_t b) {
            const ragdoll::Def::B& bd = def.bodies[b];
            if (bd.a < 0 || bd.b < 0) return;
            ImVec2 a0, a1;
            if (!project(parts[(size_t)bd.a], a0) || !project(parts[(size_t)bd.b], a1)) return;
            const float ra = std::max(2.0f * px, pxRadius(parts[(size_t)bd.a], def.particles[(size_t)bd.a].radius));
            const float rb = std::max(2.0f * px, pxRadius(parts[(size_t)bd.b], def.particles[(size_t)bd.b].radius));
            const bool sel = selBody == (int)b, hov = hovPart < 0 && hovBody == (int)b;
            const ImVec4 c = sel ? sm.warn : hov ? sm.text : sm.accent;
            capsule(a0, ra, a1, rb, u32(fadeTo(c, sel ? 0.20f : hov ? 0.16f : 0.09f)),
                    u32(fadeTo(c, sel ? 0.95f : hov ? 0.85f : 0.50f)), (sel ? 2.0f : 1.2f) * px);
            dl->AddLine(a0, a1, u32(fadeTo(c, sel ? 0.85f : 0.35f)), px);  // its axis
        };
        for (size_t b = 0; b < def.bodies.size(); ++b)
            if ((int)b != selBody) drawBody(b);
        if (selBody >= 0 && selBody < (int)def.bodies.size()) drawBody((size_t)selBody);  // on top
    }
    if (have && r.showParticles)
        for (size_t k = 0; k < parts.size(); ++k) {
            ImVec2 a0;
            if (!project(parts[k], a0)) continue;
            const bool sel = r.selKind == RagdollEd::Particle && r.sel == (int)k, hov = hovPart == (int)k;
            const bool ofSel = selBody >= 0 && selBody < (int)def.bodies.size() &&
                               (def.bodies[(size_t)selBody].a == (int)k || def.bodies[(size_t)selBody].b == (int)k);
            const ImVec4 c = sel ? sm.warn : hov ? sm.text : fadeTo(sm.text, 0.80f);
            // the collision radius only where it matters: the selection and the hover
            if (sel || hov || ofSel)
                dl->AddCircle(a0, std::max(2.0f * px, pxRadius(parts[k], def.particles[k].radius)),
                              u32(fadeTo(sel || ofSel ? sm.warn : sm.text, 0.75f)), 32, (sel ? 1.8f : 1.2f) * px);
            const float dot = (sel || hov ? 3.2f : 2.4f) * px;
            dl->AddCircleFilled(a0, dot + px, IM_COL32(0, 0, 0, 110));
            dl->AddCircleFilled(a0, dot, u32(c));
        }
    if (have && r.showTargets && r.mode == RagdollEd::Motorized && r.sim.hasTargets()) {
        const std::vector<V3>& t = r.sim.targets();
        const float h = 3.0f * px;
        for (size_t k = 0; k < t.size() && k < parts.size(); ++k) {
            ImVec2 a0, a1;
            if (!project(t[k], a0) || !project(parts[k], a1)) continue;
            dl->AddLine(ImVec2(a0.x - h, a0.y), ImVec2(a0.x + h, a0.y), cTarget, 1.5f * px);
            dl->AddLine(ImVec2(a0.x, a0.y - h), ImVec2(a0.x, a0.y + h), cTarget, 1.5f * px);
            dl->AddLine(a0, a1, u32(fadeTo(sm.ok, 0.12f)), px);  // a whisper: the crosses carry it
        }
    }
    // the selected joint: its swing limit (the allowed directions from the
    // pivot, in its parent's frame as it is now) and its axes - pitch (red),
    // yaw (green), roll (blue) - the colours its limits carry in the details
    if (have && (r.showLimits || r.showAxes) && selBody >= 0 && selBody < (int)def.bodies.size() && r.sim.ready()) {
        const ragdoll::Def::B& bd = def.bodies[(size_t)selBody];
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
            dl->AddLine(s0, s1, col, 2.0f * px);
            const float dx = s1.x - s0.x, dy = s1.y - s0.y, l = std::max(std::hypot(dx, dy), 1e-3f);
            const float ux = dx / l, uy = dy / l, h = 8.0f * px;
            dl->AddTriangleFilled(s1, ImVec2(s1.x - ux * h - uy * h * 0.5f, s1.y - uy * h + ux * h * 0.5f),
                                  ImVec2(s1.x - ux * h + uy * h * 0.5f, s1.y - uy * h - ux * h * 0.5f), col);
            haloText(dl, ImVec2(s1.x + 5.0f * px, s1.y - ImGui::GetFontSize() * 0.5f), col, label);
        };
        const ImU32 cPitch = u32(kPitchCol), cYaw = u32(kYawCol), cRoll = u32(kRollCol);
        if (bd.parent >= 0 && fr.size() >= (size_t)(bd.parent + 1) * 9) {
            const float* pf = &fr[(size_t)bd.parent * 9];
            V3 c, pa, ya;
            float hp, hy;
            r.sim.limitCone(selBody, c, pa, ya, hp, hy);
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
                    const V3 dir = mul3(pf, r.sim.swingDirection(selBody, u, v));
                    ImVec2 sp;
                    if (project(pivot + dir * L, sp)) ring.push_back(sp);
                }
                ImVec2 sp;
                if (project(pivot, sp)) {
                    for (size_t i = 0; i + 1 < ring.size(); ++i) dl->AddLine(ring[i], ring[i + 1], cLimit, 1.5f * px);
                    for (size_t i = 0; i < ring.size(); i += ring.size() / 4 > 0 ? ring.size() / 4 : 1)
                        dl->AddLine(sp, ring[i], u32(fadeTo(sm.warn, 0.30f)), px);
                    ImVec2 cen;
                    if (project(pivot + mul3(pf, c) * L, cen)) dl->AddCircle(cen, 3.0f * px, cLimit, 0, px);
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
                for (int i = 0; i <= 32; ++i) {
                    const float ang = (float)i / 32.0f * 6.2831853f;
                    const V3 p = pivot + axis * (L * 0.5f) + (u * std::cos(ang) + w * std::sin(ang)) * (L * 0.18f);
                    ImVec2 sp;
                    if (!project(p, sp)) continue;
                    if (havePrev) dl->AddLine(prev, sp, cRoll, 1.5f * px);
                    prev = sp, havePrev = true;
                }
            }
        } else if (r.showAxes && fr.size() >= 9) {
            // the root: its own frame (x along it, the roll; y and z across)
            const float* f = &fr[(size_t)selBody * 9];
            arrow(colv(f, 1), L * 0.6f, cPitch, "y");
            arrow(colv(f, 2), L * 0.6f, cYaw, "z");
            arrow(colv(f, 0), L * 1.15f, cRoll, "x (axis)");
        }
    }
    // the grab: the held point and the hand pulling it
    if (r.sim.grabbing()) {
        ImVec2 g0, g1;
        if (project(r.sim.grabPoint(), g0) && project(r.sim.grabTarget(), g1)) {
            dl->AddLine(g0, g1, cGrab, 2.0f * px);
            dl->AddCircleFilled(g0, 4.0f * px, cGrab);
            dl->AddCircle(g1, 6.0f * px, cGrab, 24, 2.0f * px);
        }
    }
    // the hovered part's name beside the cursor
    if (hovPart >= 0 || hovBody >= 0) {
        const std::string& nm = hovPart >= 0 ? r.asset.particles[(size_t)hovPart].name
                                             : r.asset.bodies[(size_t)hovBody].name;
        if ((hovPart < 0 || hovPart < (int)r.asset.particles.size()) &&
            (hovBody < 0 || hovBody < (int)r.asset.bodies.size()))
            haloText(dl, ImVec2(io.MousePos.x + 14.0f * px, io.MousePos.y + 8.0f * px), u32(sm.text), nm.c_str());
    }
    // what is running, in a pill at the bottom left
    {
        char info[200];
        const char* clipName = r.clip >= 0 && r.clip < (int)r.model.clips.size()
                                   ? r.model.clips[(size_t)r.clip].name.c_str()
                                   : "bind pose";
        if (r.mode == RagdollEd::Animation)
            std::snprintf(info, sizeof(info), "Animation  \xc2\xb7  %s  \xc2\xb7  %.2f s", clipName, (double)r.clipTime);
        else
            std::snprintf(info, sizeof(info), "%s  \xc2\xb7  %.2f s  \xc2\xb7  %s%s", kModeNames[r.mode],
                          (double)r.sim.age(), r.sim.asleep() ? "asleep" : "awake",
                          r.mode == RagdollEd::Motorized
                              ? (r.sim.motorScale() < 0.999f ? "  \xc2\xb7  motors fading" : "  \xc2\xb7  motors on")
                              : "");
        const ImVec2 ts = ImGui::CalcTextSize(info);
        const float padX = 9.0f * px, padY = 4.0f * px;
        const ImVec2 p0(origin.x + 10.0f * px, origin.y + (float)ph - 10.0f * px - ts.y - 2.0f * padY);
        dl->AddRectFilled(p0, ImVec2(p0.x + ts.x + 2.0f * padX, p0.y + ts.y + 2.0f * padY),
                          u32(fadeTo(sm.surface, 0.82f)), ts.y);
        dl->AddText(ImVec2(p0.x + padX, p0.y + padY), u32(sm.text), info);
    }
    // the controls, behind a ? at the top right
    {
        const float rad = 10.0f * px;
        const ImVec2 c(origin.x + (float)pw - 18.0f * px, origin.y + 18.0f * px);
        const bool over = hovered && std::hypot(io.MousePos.x - c.x, io.MousePos.y - c.y) <= rad + 3.0f * px;
        dl->AddCircleFilled(c, rad, u32(fadeTo(sm.surface, over ? 0.98f : 0.82f)), 24);
        dl->AddCircle(c, rad, u32(over ? sm.accent : sm.border), 24, px);
        const ImVec2 qs = ImGui::CalcTextSize("?");
        dl->AddText(ImVec2(c.x - qs.x * 0.5f, c.y - qs.y * 0.5f), u32(over ? sm.text : sm.textDim), "?");
        if (over) wrappedTooltip(kControlsHelp);
    }
    dl->PopClipRect();
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
