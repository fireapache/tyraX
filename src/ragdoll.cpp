#include "ragdoll.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <functional>
#include <map>
#include <set>

#include "animmerge.hpp"
#include "glbparser.hpp"
#include "json.hpp"

namespace ragdoll {

namespace {

// ------------------------------------------------------------------ helpers --
struct V3 {
    float x = 0, y = 0, z = 0;
};
V3 v3(float x, float y, float z) { return V3{x, y, z}; }
V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V3 operator*(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 cross(V3 a, V3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
float len(V3 a) { return std::sqrt(dot(a, a)); }
V3 unit(V3 a, V3 fb) {
    const float l = len(a);
    return l > 1e-8f ? a * (1.0f / l) : fb;
}

V3 xformPoint(const float* m, V3 p) {
    return {m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12],
            m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13],
            m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14]};
}
// p (model space) into the frame of rigid matrix m: R^T (p - t)
V3 untransformPoint(const float* m, V3 p) {
    const V3 d = p - v3(m[12], m[13], m[14]);
    return {m[0] * d.x + m[1] * d.y + m[2] * d.z, m[4] * d.x + m[5] * d.y + m[6] * d.z,
            m[8] * d.x + m[9] * d.y + m[10] * d.z};
}
// m with each rotation column normalised (a scaled armature node leaves its
// bones' globals scaled; an offset lives in the unscaled frame)
void orthoCols(const float* in, float* out) {
    std::memcpy(out, in, 16 * sizeof(float));
    for (int c = 0; c < 3; ++c) {
        const float l = std::sqrt(in[c * 4] * in[c * 4] + in[c * 4 + 1] * in[c * 4 + 1] +
                                  in[c * 4 + 2] * in[c * 4 + 2]);
        if (l > 1e-9f)
            for (int r = 0; r < 3; ++r) out[c * 4 + r] = in[c * 4 + r] / l;
    }
}

std::string num(float v) {
    if (std::fabs(v) < 5e-7f) v = 0.0f;  // no "-0" or 1e-9 noise in the files
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.6g", (double)v);
    return buf;
}
std::string arr3(const float* v) {
    return "[" + num(v[0]) + ", " + num(v[1]) + ", " + num(v[2]) + "]";
}
std::string arr2(const float* v) { return "[" + num(v[0]) + ", " + num(v[1]) + "]"; }
// A C++ float literal, short: 1 -> "1.0F", 0.125 -> "0.125F"
std::string flit(float v) {
    std::string t = num(v);
    if (t.find_first_of(".e") == std::string::npos) t += ".0";
    return t + "F";
}
std::string q(const std::string& s) { return "\"" + json::escape(s) + "\""; }

void readFloats(const json::Value* v, float* out, int n) {
    if (!v || v->type != json::Value::Type::Array) return;
    for (int i = 0; i < n && i < (int)v->arr.size(); ++i)
        out[i] = (float)v->arr[(size_t)i].numberOr(out[i]);
}
std::string str(const json::Value& o, const char* key, const std::string& fb = "") {
    const json::Value* v = o.find(key);
    return v ? v->stringOr(fb) : fb;
}
float flt(const json::Value& o, const char* key, float fb) {
    const json::Value* v = o.find(key);
    return v ? (float)v->numberOr(fb) : fb;
}
bool bln(const json::Value& o, const char* key, bool fb) {
    const json::Value* v = o.find(key);
    return v ? v->boolOr(fb) : fb;
}

std::string lower(std::string s) {
    for (char& c : s) c = (char)std::tolower((unsigned char)c);
    return s;
}
bool has(const std::string& lname, const char* token) {
    return lname.find(token) != std::string::npos;
}

// ------------------------------------------------------------- the solver --
// One table of name, getter, setter-by-pointer for the Solver's fields, so
// the JSON writer and reader can never disagree on a key.
struct SolverField {
    const char* key;
    float Solver::*f;
};
const SolverField kSolverFields[] = {
    {"gravity", &Solver::gravity},
    {"damping", &Solver::damping},
    {"rolling", &Solver::rolling},
    {"staticFriction", &Solver::staticFriction},
    {"kineticFriction", &Solver::kineticFriction},
    {"contactSlop", &Solver::contactSlop},
    {"limitStiffness", &Solver::limitStiffness},
    {"jointDamping", &Solver::jointDamping},
    {"rollBendMin", &Solver::rollBendMin},
    {"rollBendFull", &Solver::rollBendFull},
    {"sleepSpeed", &Solver::sleepSpeed},
    {"sleepTime", &Solver::sleepTime},
    {"maxAwake", &Solver::maxAwake},
    {"pushRadius", &Solver::pushRadius},
    {"pushTopple", &Solver::pushTopple},
    {"pushBody", &Solver::pushBody},
    {"motorGain", &Solver::motorGain},
    {"motorVelocity", &Solver::motorVelocity},
    {"motorLying", &Solver::motorLying},
    {"motorFadeTime", &Solver::motorFadeTime},
    {"lyingSlop", &Solver::lyingSlop},
    {"boxStepUp", &Solver::boxStepUp},
    {"fixedDt", &Solver::fixedDt},
};

}  // namespace

// ================================================================ skeleton ==

int Skeleton::find(const std::string& name) const {
    for (size_t i = 0; i < bones.size(); ++i)
        if (bones[i].name == name) return (int)i;
    return -1;
}

bool skeletonFromSkel(const glbparser::Skel& skel, const std::string& modelRel,
                      Skeleton& out) {
    out = Skeleton();
    out.model = modelRel;
    std::vector<float> g;
    animmerge::poseMatrices(skel, -1, 0.0f, g);
    std::vector<int> boneOfNode(skel.nodes.size(), -1);
    std::vector<int> nodes;
    for (const glbparser::SkelJoint& j : skel.palette) {
        if (j.node < 0 || j.node >= (int)skel.nodes.size()) continue;
        if (boneOfNode[(size_t)j.node] >= 0) continue;
        boneOfNode[(size_t)j.node] = (int)nodes.size();
        nodes.push_back(j.node);
    }
    // A rigid mesh's palette slot (identity ibm on its own node) is not a
    // bone of a skin; a model with no skin has nothing to ragdoll.
    if (nodes.empty()) return false;
    for (int n : nodes) {
        Bone b;
        b.name = skel.nodes[(size_t)n].name;
        for (int p = skel.nodes[(size_t)n].parent; p >= 0 && p < (int)skel.nodes.size();
             p = skel.nodes[(size_t)p].parent) {
            if (boneOfNode[(size_t)p] >= 0) {
                b.parent = boneOfNode[(size_t)p];
                break;
            }
            if (skel.nodes[(size_t)p].parent == p) break;
        }
        if ((size_t)n * 16 + 16 <= g.size()) std::memcpy(b.bind, &g[(size_t)n * 16], sizeof(b.bind));
        out.bones.push_back(b);
    }
    return true;
}

std::string skeletonJson(const Skeleton& s) {
    std::string o = "{\n  \"asset\": \"tyrax.skeleton\",\n  \"version\": 1,\n";
    o += "  \"model\": " + q(s.model) + ",\n  \"bones\": [\n";
    for (size_t i = 0; i < s.bones.size(); ++i) {
        const Bone& b = s.bones[i];
        o += "    {\"name\": " + q(b.name) + ", \"parent\": " +
             q(b.parent >= 0 ? s.bones[(size_t)b.parent].name : "") + ", \"bind\": [";
        for (int k = 0; k < 16; ++k) o += (k ? ", " : "") + num(b.bind[k]);
        o += "]}";
        o += i + 1 < s.bones.size() ? ",\n" : "\n";
    }
    o += "  ]\n}\n";
    return o;
}

bool parseSkeleton(const std::string& text, Skeleton& out, std::string& error) {
    out = Skeleton();
    json::Value root;
    if (!json::parse(text, root) || root.type != json::Value::Type::Object) {
        error = "not a JSON object";
        return false;
    }
    if (str(root, "asset") != "tyrax.skeleton") {
        error = "not a tyrax.skeleton asset";
        return false;
    }
    out.model = str(root, "model");
    const json::Value* bones = root.find("bones");
    if (!bones || bones->type != json::Value::Type::Array) {
        error = "no bones";
        return false;
    }
    std::vector<std::string> parents;
    for (const json::Value& b : bones->arr) {
        Bone bone;
        bone.name = str(b, "name");
        readFloats(b.find("bind"), bone.bind, 16);
        parents.push_back(str(b, "parent"));
        out.bones.push_back(bone);
    }
    for (size_t i = 0; i < out.bones.size(); ++i)
        out.bones[i].parent = parents[i].empty() ? -1 : out.find(parents[i]);
    return true;
}

// ================================================================= ragdoll ==

const ProfileJoint* Profile::find(const std::string& body) const {
    for (const ProfileJoint& j : joints)
        if (j.body == body) return &j;
    return nullptr;
}

int Asset::particle(const std::string& name) const {
    for (size_t i = 0; i < particles.size(); ++i)
        if (particles[i].name == name) return (int)i;
    return -1;
}
int Asset::body(const std::string& name) const {
    for (size_t i = 0; i < bodies.size(); ++i)
        if (bodies[i].name == name) return (int)i;
    return -1;
}
const Profile* Asset::profile(const std::string& name) const {
    for (const Profile& p : profiles)
        if (p.name == name) return &p;
    return nullptr;
}

std::string assetJson(const Asset& a) {
    std::string o = "{\n  \"asset\": \"tyrax.ragdoll\",\n  \"version\": 1,\n";
    o += "  \"skeleton\": " + q(a.skeleton) + ",\n";
    o += "  \"header\": " + q(a.header) + ",\n";
    o += "  \"namespace\": " + q(a.nameSpace) + ",\n";
    o += "  \"particles\": [\n";
    for (size_t i = 0; i < a.particles.size(); ++i) {
        const Particle& p = a.particles[i];
        o += "    {\"name\": " + q(p.name) + ", \"bone\": " + q(p.bone) +
             ", \"offset\": " + arr3(p.offset) + ", \"radius\": " + num(p.radius) +
             ", \"mass\": " + num(p.mass) + ", \"world\": " + (p.world ? "true" : "false") +
             "}";
        o += i + 1 < a.particles.size() ? ",\n" : "\n";
    }
    o += "  ],\n  \"bodies\": [\n";
    for (size_t i = 0; i < a.bodies.size(); ++i) {
        const Body& b = a.bodies[i];
        o += "    {\"name\": " + q(b.name) + ", \"pivot\": " + q(b.pivot) +
             ", \"end\": " + q(b.end) + ", \"parent\": " + q(b.parent) +
             ", \"side\": [" + q(b.sideA) + ", " + q(b.sideB) + "],\n     \"pitchDir\": " +
             arr3(b.pitchDir) + ", \"yawDir\": " + arr3(b.yawDir) +
             ", \"pitch\": " + arr2(b.pitch) + ", \"yaw\": " + arr2(b.yaw) +
             ", \"roll\": " + arr2(b.roll) + ", \"hinge\": " + (b.hinge ? "true" : "false") +
             ", \"groundFade\": " + (b.groundFade ? "true" : "false") + "}";
        o += i + 1 < a.bodies.size() ? ",\n" : "\n";
    }
    o += "  ],\n  \"clusters\": [\n";
    for (size_t i = 0; i < a.clusters.size(); ++i) {
        o += "    [";
        for (size_t k = 0; k < a.clusters[i].size(); ++k)
            o += (k ? ", " : "") + q(a.clusters[i][k]);
        o += i + 1 < a.clusters.size() ? "],\n" : "]\n";
    }
    o += "  ],\n  \"profiles\": [\n";
    for (size_t i = 0; i < a.profiles.size(); ++i) {
        const Profile& p = a.profiles[i];
        o += "    {\"name\": " + q(p.name) + ", \"joints\": [\n";
        for (size_t k = 0; k < p.joints.size(); ++k) {
            const ProfileJoint& j = p.joints[k];
            o += "      {\"body\": " + q(j.body) + ", \"relative\": " +
                 (j.relative ? "true" : "false") + ", \"ratio\": " + num(j.ratio) + "}";
            o += k + 1 < p.joints.size() ? ",\n" : "\n";
        }
        o += i + 1 < a.profiles.size() ? "    ]},\n" : "    ]}\n";
    }
    // Every field, default or not: the file says exactly what the preview runs.
    o += "  ],\n  \"solver\": {\"iterations\": " + std::to_string(a.solver.iterations);
    for (const SolverField& f : kSolverFields)
        o += std::string(", \"") + f.key + "\": " + num(a.solver.*(f.f));
    o += "}\n}\n";
    return o;
}

bool parseAsset(const std::string& text, Asset& out, std::string& error) {
    out = Asset();
    json::Value root;
    if (!json::parse(text, root) || root.type != json::Value::Type::Object) {
        error = "not a JSON object";
        return false;
    }
    if (str(root, "asset") != "tyrax.ragdoll") {
        error = "not a tyrax.ragdoll asset";
        return false;
    }
    out.skeleton = str(root, "skeleton");
    out.header = str(root, "header");
    out.nameSpace = str(root, "namespace");
    if (const json::Value* ps = root.find("particles"))
        for (const json::Value& v : ps->arr) {
            Particle p;
            p.name = str(v, "name");
            p.bone = str(v, "bone");
            readFloats(v.find("offset"), p.offset, 3);
            p.radius = flt(v, "radius", p.radius);
            p.mass = flt(v, "mass", p.mass);
            p.world = bln(v, "world", false);
            out.particles.push_back(p);
        }
    if (const json::Value* bs = root.find("bodies"))
        for (const json::Value& v : bs->arr) {
            Body b;
            b.name = str(v, "name");
            b.pivot = str(v, "pivot");
            b.end = str(v, "end");
            b.parent = str(v, "parent");
            if (const json::Value* s = v.find("side"); s && s->arr.size() == 2) {
                b.sideA = s->arr[0].stringOr("");
                b.sideB = s->arr[1].stringOr("");
            }
            readFloats(v.find("pitchDir"), b.pitchDir, 3);
            readFloats(v.find("yawDir"), b.yawDir, 3);
            readFloats(v.find("pitch"), b.pitch, 2);
            readFloats(v.find("yaw"), b.yaw, 2);
            readFloats(v.find("roll"), b.roll, 2);
            b.hinge = bln(v, "hinge", false);
            b.groundFade = bln(v, "groundFade", false);
            out.bodies.push_back(b);
        }
    if (const json::Value* cs = root.find("clusters"))
        for (const json::Value& c : cs->arr) {
            std::vector<std::string> names;
            for (const json::Value& n : c.arr) names.push_back(n.stringOr(""));
            out.clusters.push_back(names);
        }
    if (const json::Value* ps = root.find("profiles"))
        for (const json::Value& v : ps->arr) {
            Profile p;
            p.name = str(v, "name");
            if (const json::Value* js = v.find("joints"))
                for (const json::Value& jv : js->arr) {
                    ProfileJoint j;
                    j.body = str(jv, "body");
                    j.relative = bln(jv, "relative", false);
                    j.ratio = flt(jv, "ratio", 0.0f);
                    p.joints.push_back(j);
                }
            out.profiles.push_back(p);
        }
    if (const json::Value* s = root.find("solver")) {
        out.solver.iterations = (int)flt(*s, "iterations", (float)out.solver.iterations);
        for (const SolverField& f : kSolverFields)
            out.solver.*(f.f) = flt(*s, f.key, out.solver.*(f.f));
    }
    return true;
}

// ================================================================= resolve ==

bool resolve(const Asset& a, const Skeleton& s, Def& d, std::vector<std::string>& errors) {
    d = Def();
    d.solver = a.solver;
    const size_t before = errors.size();
    auto err = [&](const std::string& m) { errors.push_back(m); };

    for (const Particle& p : a.particles) {
        Def::P dp;
        dp.name = p.name;
        dp.bone = s.find(p.bone);
        if (dp.bone < 0) err("particle '" + p.name + "': no bone '" + p.bone + "' in the skeleton");
        std::memcpy(dp.offset, p.offset, sizeof(dp.offset));
        if (dp.bone >= 0) {
            float m[16];
            orthoCols(s.bones[(size_t)dp.bone].bind, m);
            const V3 w = xformPoint(m, v3(p.offset[0], p.offset[1], p.offset[2]));
            dp.bind[0] = w.x, dp.bind[1] = w.y, dp.bind[2] = w.z;
        }
        dp.radius = p.radius > 0.001f ? p.radius : 0.001f;
        dp.mass = p.mass > 0.001f ? p.mass : 0.001f;
        dp.world = p.world;
        d.particles.push_back(dp);
    }
    std::set<std::string> seen;
    for (const Particle& p : a.particles)
        if (!seen.insert(p.name).second) err("particle '" + p.name + "' is named twice");

    auto pidx = [&](const std::string& n, const std::string& body, const char* what,
                    bool optional) {
        if (n.empty()) {
            if (!optional) err("body '" + body + "': no " + what);
            return -1;
        }
        const int i = a.particle(n);
        if (i < 0) err("body '" + body + "': no particle '" + n + "' (" + what + ")");
        return i;
    };
    for (size_t bi = 0; bi < a.bodies.size(); ++bi) {
        const Body& b = a.bodies[bi];
        Def::B db;
        db.name = b.name;
        db.a = pidx(b.pivot, b.name, "pivot", false);
        db.b = pidx(b.end, b.name, "end", false);
        db.sideA = pidx(b.sideA, b.name, "side A", true);
        db.sideB = pidx(b.sideB, b.name, "side B", true);
        if ((db.sideA < 0) != (db.sideB < 0)) {
            err("body '" + b.name + "': a side pair needs both particles");
            db.sideA = db.sideB = -1;
        }
        if (db.a >= 0 && db.a == db.b) err("body '" + b.name + "': pivot and end are one particle");
        if (!b.parent.empty()) {
            db.parent = a.body(b.parent);
            if (db.parent < 0)
                err("body '" + b.name + "': no parent body '" + b.parent + "'");
            else if (db.parent >= (int)bi)
                err("body '" + b.name + "': its parent '" + b.parent +
                    "' must come before it in the list");
        } else if (bi != 0) {
            err("body '" + b.name + "': only the first body may be the root");
        }
        std::memcpy(db.pitchDir, b.pitchDir, sizeof(db.pitchDir));
        std::memcpy(db.yawDir, b.yawDir, sizeof(db.yawDir));
        db.pitchMin = b.pitch[0], db.pitchMax = b.pitch[1];
        db.yawMin = b.yaw[0], db.yawMax = b.yaw[1];
        db.rollMin = b.roll[0], db.rollMax = b.roll[1];
        if (db.pitchMin > db.pitchMax) std::swap(db.pitchMin, db.pitchMax);
        if (db.yawMin > db.yawMax) std::swap(db.yawMin, db.yawMax);
        if (db.rollMin > db.rollMax) std::swap(db.rollMin, db.rollMax);
        db.hinge = b.hinge;
        if (db.hinge) db.yawMin = db.yawMax = 0.0f;
        db.groundFade = b.groundFade;
        d.bodies.push_back(db);
    }
    if (d.bodies.empty()) err("no bodies");
    else if (d.bodies[0].parent >= 0 || !a.bodies[0].parent.empty())
        err("the first body must be the root (no parent)");
    else if (d.bodies[0].sideA < 0)
        err("the root body '" + d.bodies[0].name +
            "' needs a side pair (its frame is built from it)");
    seen.clear();
    for (const Body& b : a.bodies)
        if (!seen.insert(b.name).second) err("body '" + b.name + "' is named twice");

    // q: the parent's end farther from the child's pivot (bind pose)
    auto bindOf = [&](int k) {
        return k >= 0 && k < (int)d.particles.size()
                   ? v3(d.particles[(size_t)k].bind[0], d.particles[(size_t)k].bind[1],
                        d.particles[(size_t)k].bind[2])
                   : V3{};
    };
    for (Def::B& b : d.bodies) {
        if (b.parent < 0 || b.a < 0) continue;
        const Def::B& p = d.bodies[(size_t)b.parent];
        const float da = len(bindOf(p.a) - bindOf(b.a)), db = len(bindOf(p.b) - bindOf(b.a));
        b.q = da >= db ? p.a : p.b;
        if (b.q == b.a) b.q = b.q == p.a ? p.b : p.a;
    }

    // links: every cluster's pairs (in its order), then each body's segment
    std::set<std::pair<int, int>> have;
    auto link = [&](int x, int y) {
        if (x < 0 || y < 0 || x == y) return;
        const std::pair<int, int> key(std::min(x, y), std::max(x, y));
        if (!have.insert(key).second) return;
        d.links.push_back({x, y});
    };
    for (const std::vector<std::string>& c : a.clusters) {
        std::vector<int> ids;
        for (const std::string& n : c) {
            const int i = a.particle(n);
            if (i < 0) err("cluster: no particle '" + n + "'");
            else ids.push_back(i);
        }
        for (size_t i = 0; i < ids.size(); ++i)
            for (size_t k = i + 1; k < ids.size(); ++k) link(ids[i], ids[k]);
    }
    for (const Def::B& b : d.bodies) link(b.a, b.b);
    // A side pair must move with its body: linked to its pivot and end.
    for (const Def::B& b : d.bodies) {
        if (b.sideA < 0) continue;
        for (int sp : {b.sideA, b.sideB})
            for (int e : {b.a, b.b})
                if (!have.count({std::min(sp, e), std::max(sp, e)}))
                    err("body '" + b.name + "': side particle '" +
                        d.particles[(size_t)sp].name + "' is not in a cluster with its pivot and end");
    }

    // bone -> body: the nearest ancestor-or-self that is some body's pivot bone
    std::vector<int> pivotBody(s.bones.size(), -1);
    for (size_t bi = 0; bi < d.bodies.size(); ++bi) {
        const int a0 = d.bodies[bi].a;
        if (a0 < 0) continue;
        const int bone = d.particles[(size_t)a0].bone;
        if (bone >= 0 && pivotBody[(size_t)bone] < 0) pivotBody[(size_t)bone] = (int)bi;
    }
    d.boneBind.assign(s.bones.size() * 16, 0.0f);
    for (size_t i = 0; i < s.bones.size(); ++i) orthoCols(s.bones[i].bind, &d.boneBind[i * 16]);
    d.boneBody.assign(s.bones.size(), -1);
    for (size_t i = 0; i < s.bones.size(); ++i)
        for (int b = (int)i, guard = 0; b >= 0 && guard < 512;
             b = s.bones[(size_t)b].parent, ++guard)
            if (pivotBody[(size_t)b] >= 0) {
                d.boneBody[i] = pivotBody[(size_t)b];
                break;
            }

    for (const Profile& p : a.profiles) {
        Def::Prof dp;
        dp.name = p.name;
        dp.relative.assign(d.bodies.size(), false);
        dp.ratio.assign(d.bodies.size(), 0.0f);
        for (const ProfileJoint& j : p.joints) {
            const int b = a.body(j.body);
            if (b < 0) {
                err("profile '" + p.name + "': no body '" + j.body + "'");
                continue;
            }
            dp.relative[(size_t)b] = j.relative;
            dp.ratio[(size_t)b] = std::clamp(j.ratio, 0.0f, 1.0f);
        }
        d.profiles.push_back(dp);
    }
    return errors.size() == before;
}

// ================================================================== header ==

std::string headerText(const Asset& a, const Skeleton& s, const Def& d,
                       const std::string& assetRel) {
    char buf[512];
    std::string o;
    o += kHeaderMarker;
    o += assetRel + " (docs/ragdoll-editor.md): edit it in the\n";
    o += "// editor's Ragdoll Editor. Model space of " + s.model + ";\n";
    o += "// a particle is seeded from its node's model matrix times `offset`.\n";
    o += "#pragma once\n\n";
    std::vector<std::string> ns;
    {
        std::string n = a.nameSpace.empty() ? std::string("Ragdoll") : a.nameSpace;
        size_t at = 0;
        while (at <= n.size()) {
            const size_t c = n.find("::", at);
            ns.push_back(n.substr(at, c == std::string::npos ? std::string::npos : c - at));
            if (c == std::string::npos) break;
            at = c + 2;
        }
    }
    for (const std::string& n : ns) o += "namespace " + n + " {\n";
    o += R"(
struct Particle {
  const char* name;
  const char* node;  // the skeleton node it rides
  float offset[3];    // in that node's bind frame
  float bind[3];      // where it is in the bind pose (model space)
  float radius;       // m: what touches the ground
  float mass;         // kg
  bool world;         // also collides with props and walls
};
)";
    std::snprintf(buf, sizeof(buf), "inline constexpr int kParticleCount = %d;\n",
                  (int)d.particles.size());
    o += buf;
    o += "inline constexpr Particle kParticles[kParticleCount] = {\n";
    for (const Def::P& p : d.particles) {
        const std::string node = p.bone >= 0 ? s.bones[(size_t)p.bone].name : "";
        std::snprintf(buf, sizeof(buf),
                      "    {\"%s\", \"%s\", {%.5fF, %.5fF, %.5fF}, {%.5fF, %.5fF, %.5fF}, "
                      "%.3fF, %.2fF, %s},\n",
                      p.name.c_str(), node.c_str(), p.offset[0], p.offset[1], p.offset[2],
                      p.bind[0], p.bind[1], p.bind[2], p.radius, p.mass,
                      p.world ? "true" : "false");
        o += buf;
    }
    o += "};\n";
    o += R"(
// A body's joint is at `a`, against its parent. Its limits are in degrees from
// the BIND pose's direction in the parent's frame: + pitch swings the far end
// toward `pitchDir`, + yaw toward `yawDir`, + roll turns pitchDir toward yawDir
// about the body's own axis (model space, bind pose). A HINGE bends in pitch
// only; its bend plane is its parent's roll.
struct Body {
  const char* name;
  int a, b;        // its particles: the pivot, the far end
  int parent;      // -1: the root
  int q;           // the parent's end farther from `a` (a limit's reaction goes there)
  int sideA, sideB;  // its own side axis (sideA - sideB), or -1: its parent's frame
  float pitchDir[3], yawDir[3];
  float pitchMin, pitchMax, yawMin, yawMax, rollMin, rollMax;  // degrees
  bool hinge;
};
)";
    std::snprintf(buf, sizeof(buf), "inline constexpr int kBodyCount = %d;\n",
                  (int)d.bodies.size());
    o += buf;
    o += "inline constexpr Body kBodies[kBodyCount] = {\n";
    for (const Def::B& b : d.bodies) {
        std::snprintf(buf, sizeof(buf), "    {\"%s\", %d, %d, %d, %d, %d, %d, ", b.name.c_str(),
                      b.a, b.b, b.parent, b.q, b.sideA, b.sideB);
        o += buf;
        o += "{" + flit(b.pitchDir[0]) + ", " + flit(b.pitchDir[1]) + ", " + flit(b.pitchDir[2]) +
             "}, {" + flit(b.yawDir[0]) + ", " + flit(b.yawDir[1]) + ", " + flit(b.yawDir[2]) +
             "}, " + flit(b.pitchMin) + ", " + flit(b.pitchMax) + ", " + flit(b.yawMin) + ", " +
             flit(b.yawMax) + ", " + flit(b.rollMin) + ", " + flit(b.rollMax) + ", " +
             (b.hinge ? "true" : "false") + "},\n";
    }
    o += "};\n";
    o += "// What a motorized ragdoll's motors fade on once its pelvis is down.\n";
    o += "inline constexpr bool kBodyGroundFade[kBodyCount] = {";
    for (size_t i = 0; i < d.bodies.size(); ++i)
        o += std::string(i ? ", " : "") + (d.bodies[i].groundFade ? "true" : "false");
    o += "};\n";

    o += "\n// Distance links: every cluster's pairs, then every body's length.\n";
    std::snprintf(buf, sizeof(buf), "inline constexpr int kLinkCount = %d;\n",
                  (int)d.links.size());
    o += buf;
    o += "inline constexpr int kLinks[kLinkCount][2] = {";
    for (size_t i = 0; i < d.links.size(); ++i) {
        std::snprintf(buf, sizeof(buf), "%s{%d, %d}", i ? ", " : "", d.links[i].first,
                      d.links[i].second);
        o += buf;
    }
    o += "};\n";

    o += R"(
// Which body every skin joint follows (its nearest ancestor-or-self that drives one).
struct Bone {
  const char* node;
  int body;
  float bindModel[16];  // the node's bind (T-pose) model matrix, column-major
};
)";
    std::snprintf(buf, sizeof(buf), "inline constexpr int kBoneCount = %d;\n",
                  (int)s.bones.size());
    o += buf;
    o += "inline constexpr Bone kBones[kBoneCount] = {\n";
    for (size_t i = 0; i < s.bones.size(); ++i) {
        std::snprintf(buf, sizeof(buf), "    {\"%s\", %d, {", s.bones[i].name.c_str(),
                      d.boneBody[i]);
        o += buf;
        for (int k = 0; k < 16; ++k) {
            std::snprintf(buf, sizeof(buf), "%s%.5fF", k ? ", " : "", s.bones[i].bind[k]);
            o += buf;
        }
        o += "}},\n";
    }
    o += "};\n";

    o += R"(
// Motor profiles (a motorized ragdoll): per body, ABSOLUTE (toward the clip's
// pose in the world) or RELATIVE (toward its pose against the parent body),
// and how strongly, 0..1.
struct ProfileJoint {
  bool relative;
  float ratio;
};
struct Profile {
  const char* name;
  ProfileJoint joint[kBodyCount];
};
)";
    std::snprintf(buf, sizeof(buf), "inline constexpr int kProfileCount = %d;\n",
                  (int)d.profiles.size());
    o += buf;
    if (!d.profiles.empty()) {
        o += "inline constexpr Profile kProfiles[kProfileCount] = {\n";
        for (const Def::Prof& p : d.profiles) {
            o += "    {\"" + p.name + "\", {";
            for (size_t b = 0; b < d.bodies.size(); ++b) {
                std::snprintf(buf, sizeof(buf), "%s{%s, %.3fF}", b ? ", " : "",
                              p.relative[b] ? "true" : "false", p.ratio[b]);
                o += buf;
            }
            o += "}},\n";
        }
        o += "};\n";
    }
    o += "\n";
    for (auto it = ns.rbegin(); it != ns.rend(); ++it) o += "}  // namespace " + *it + "\n";
    return o;
}

// ============================================================== generation ==

namespace {

struct LimitPreset {
    float pitch[2], yaw[2], roll[2];
    bool hinge;
    int pitchKind;  // 0 forward, 1 backward, 2 down
    bool lower;     // part of the lower body (zombie profile, groundFade)
};

// By bone name - the conventions every rig in the wild shares (Mixamo,
// Blender's Rigify-lite, MakeHuman, Unreal's mannequin): what bends which way.
LimitPreset presetFor(const std::string& name, bool root) {
    const std::string n = lower(name);
    auto P = [](float p0, float p1, float y0, float y1, float r0, float r1, bool h, int k,
                bool lo) {
        LimitPreset l;
        l.pitch[0] = p0, l.pitch[1] = p1, l.yaw[0] = y0, l.yaw[1] = y1, l.roll[0] = r0,
        l.roll[1] = r1;
        l.hinge = h, l.pitchKind = k, l.lower = lo;
        return l;
    };
    if (root) return P(0, 0, 0, 0, 0, 0, false, 0, true);
    if (has(n, "toe")) return P(-30, 30, -10, 10, 0, 0, false, 2, true);
    if (has(n, "foot") || has(n, "ankle")) return P(-45, 65, -20, 20, 0, 0, false, 2, true);
    if (has(n, "upleg") || has(n, "thigh") || has(n, "up_leg") || has(n, "upperleg"))
        return P(-20, 100, -20, 40, -25, 30, false, 0, true);
    if (has(n, "knee") || has(n, "calf") || has(n, "shin") || has(n, "lowerleg") ||
        (has(n, "leg") && !has(n, "up")))
        return P(0, 110, 0, 0, 0, 0, true, 1, true);
    if (has(n, "forearm") || has(n, "lowerarm") || has(n, "elbow") || has(n, "fore_arm"))
        return P(0, 110, 0, 0, 0, 0, true, 0, false);
    if (has(n, "hand") || has(n, "wrist") || has(n, "finger") || has(n, "thumb"))
        return P(-50, 60, -30, 20, 0, 0, false, 2, false);
    if (has(n, "shoulder") || has(n, "clavicle") || has(n, "collar"))
        return P(-20, 20, -20, 20, -10, 10, false, 0, false);
    if (has(n, "arm")) return P(-40, 110, -80, 90, -60, 70, false, 0, false);
    if (has(n, "head")) return P(-50, 60, -40, 40, -20, 20, false, 0, false);
    if (has(n, "neck")) return P(-40, 50, -40, 40, -30, 30, false, 0, false);
    if (has(n, "spine") || has(n, "chest") || has(n, "torso") || has(n, "pelvis") ||
        has(n, "hip"))
        return P(-20, 40, -25, 25, -30, 30, false, 0, true);
    return P(-30, 30, -30, 30, -20, 20, false, 0, false);
}

}  // namespace

bool generate(const glbparser::Skel& skel, const Skeleton& s, const GenOptions& opt,
              Asset& out, std::string& report) {
    out = Asset();
    if (s.bones.empty()) {
        report = "the skeleton has no bones";
        return false;
    }
    const size_t nb = s.bones.size();
    std::vector<V3> pos(nb);
    for (size_t i = 0; i < nb; ++i) pos[i] = v3(s.bones[i].bind[12], s.bones[i].bind[13], s.bones[i].bind[14]);
    std::vector<std::vector<int>> kids(nb);
    for (size_t i = 0; i < nb; ++i)
        if (s.bones[i].parent >= 0) kids[(size_t)s.bones[i].parent].push_back((int)i);

    // Skinned vertices, each owned by its heaviest bone.
    std::map<int, int> boneOfNode;
    for (size_t i = 0; i < nb; ++i)
        for (size_t n = 0; n < skel.nodes.size(); ++n)
            if (skel.nodes[n].name == s.bones[i].name) boneOfNode[(int)n] = (int)i;
    std::vector<std::vector<V3>> verts(nb);
    float minY = 1e9f, maxY = -1e9f;
    for (const glbparser::SkelPart& part : skel.parts)
        for (int v = 0; v < part.vertexCount; ++v) {
            const V3 p = v3(part.positions[(size_t)v * 3], part.positions[(size_t)v * 3 + 1],
                            part.positions[(size_t)v * 3 + 2]);
            minY = std::min(minY, p.y), maxY = std::max(maxY, p.y);
            int best = -1, bw = 0;
            for (int k = 0; k < 4; ++k) {
                const size_t at = (size_t)v * 4 + (size_t)k;
                if (at >= part.weights.size()) break;
                if (part.weights[at] > bw) bw = part.weights[at], best = part.joints[at];
            }
            if (best < 0 || best >= (int)skel.palette.size()) continue;
            auto it = boneOfNode.find(skel.palette[(size_t)best].node);
            if (it != boneOfNode.end()) verts[(size_t)it->second].push_back(p);
        }
    const float height = maxY > minY ? maxY - minY : 1.7f;
    std::vector<int> subtree(nb, 0);
    std::function<int(int)> count = [&](int i) {
        int c = (int)verts[(size_t)i].size();
        for (int k : kids[(size_t)i]) c += count(k);
        subtree[(size_t)i] = c;
        return c;
    };
    int root = -1;
    for (size_t i = 0; i < nb; ++i)
        if (s.bones[i].parent < 0) {
            count((int)i);
            if (root < 0 || subtree[i] > subtree[(size_t)root]) root = (int)i;
        }
    auto mainChild = [&](int i) {
        int best = -1;
        for (int k : kids[(size_t)i])
            if (best < 0 || subtree[(size_t)k] > subtree[(size_t)best]) best = k;
        return best;
    };

    // Which bones carry a body: the root, and every bone with skin of its own
    // whose segment to its main child (or its own extent) is long enough.
    const float minLen = opt.minBoneSize * height;
    std::vector<char> isBody(nb, 0);
    std::function<void(int)> mark = [&](int i) {
        const int mc = mainChild(i);
        float segLen = mc >= 0 ? len(pos[(size_t)mc] - pos[(size_t)i]) : 0.0f;
        if (mc < 0) {  // a leaf: its own skin's reach
            for (const V3& v : verts[(size_t)i]) segLen = std::max(segLen, len(v - pos[(size_t)i]));
        }
        const bool skinned = subtree[(size_t)i] > 0;
        isBody[(size_t)i] = i == root || (skinned && segLen >= minLen && !verts[(size_t)i].empty());
        for (int k : kids[(size_t)i]) mark(k);
    };
    if (root >= 0) mark(root);

    // Bodies in hierarchy order (parents first), named after their bones.
    struct GB {
        int bone, parentBody = -1;
        int pivotP = -1, endP = -1;
        V3 endPos;
        float radius = 0.05f;
        std::vector<int> cluster;  // particle ids rigid with this body
        int sideA = -1, sideB = -1;
        size_t vertsN = 0;
    };
    std::vector<GB> gb;
    std::vector<int> bodyOfBone(nb, -1);
    std::function<void(int, int)> collect = [&](int i, int parentBody) {
        int mine = parentBody;
        if (isBody[(size_t)i]) {
            GB b;
            b.bone = i;
            b.parentBody = parentBody;
            gb.push_back(b);
            mine = (int)gb.size() - 1;
            bodyOfBone[(size_t)i] = mine;
        }
        for (int k : kids[(size_t)i]) collect(k, mine);
    };
    if (root >= 0) collect(root, -1);
    // the skin each body carries: its bone's and its non-body descendants'
    std::vector<int> ownerBody(nb, -1);
    for (size_t i = 0; i < nb; ++i)
        for (int b = (int)i; b >= 0; b = s.bones[(size_t)b].parent)
            if (bodyOfBone[(size_t)b] >= 0) {
                ownerBody[i] = bodyOfBone[(size_t)b];
                break;
            }

    // End of each body: the pivot of the nearest body down its main chain,
    // else its skin's reach along the bone.
    std::vector<int> endBone(gb.size(), -1);
    for (size_t bi = 0; bi < gb.size(); ++bi) {
        int c = mainChild(gb[bi].bone);
        while (c >= 0 && !isBody[(size_t)c]) c = mainChild(c);
        endBone[bi] = c;
        const V3 p0 = pos[(size_t)gb[bi].bone];
        if (c >= 0) {
            gb[bi].endPos = pos[(size_t)c];
        } else {
            const int par = s.bones[(size_t)gb[bi].bone].parent;
            V3 dir = par >= 0 ? unit(p0 - pos[(size_t)par], v3(0, 1, 0)) : v3(0, 1, 0);
            float reach = 0.0f;
            for (size_t i = 0; i < nb; ++i)
                if (ownerBody[i] == (int)bi)
                    for (const V3& v : verts[i]) reach = std::max(reach, dot(v - p0, dir));
            if (reach < minLen * 0.5f) reach = std::max(minLen * 0.5f, 0.02f * height);
            gb[bi].endPos = p0 + dir * reach;
        }
        // radius: the median distance of its skin from its segment
        std::vector<float> ds;
        const V3 ax = gb[bi].endPos - p0;
        const float axL2 = std::max(dot(ax, ax), 1e-8f);
        for (size_t i = 0; i < nb; ++i)
            if (ownerBody[i] == (int)bi)
                for (const V3& v : verts[i]) {
                    const float t = std::clamp(dot(v - p0, ax) / axL2, 0.0f, 1.0f);
                    ds.push_back(len(v - (p0 + ax * t)));
                }
        gb[bi].vertsN = ds.size();
        if (!ds.empty()) {
            std::nth_element(ds.begin(), ds.begin() + (long)(ds.size() / 2), ds.end());
            gb[bi].radius = ds[ds.size() / 2];
        }
        gb[bi].radius = std::clamp(gb[bi].radius, 0.015f * height, 0.1f * height);
    }

    // Particles: a body's pivot is its parent's END when that is the same
    // bone (a knee is the thigh's end and the shin's pivot), else a particle
    // of its own, held rigid in its parent's cluster (a hip in the pelvis).
    auto addP = [&](const std::string& name, int bone, V3 worldPos, float radius) {
        Particle p;
        std::string nm = name;
        for (int k = 1; out.particle(nm) >= 0; ++k) nm = name + "_" + std::to_string(k);
        p.name = nm;
        p.bone = s.bones[(size_t)bone].name;
        float m[16];
        orthoCols(s.bones[(size_t)bone].bind, m);
        const V3 off = untransformPoint(m, worldPos);
        p.offset[0] = off.x, p.offset[1] = off.y, p.offset[2] = off.z;
        p.radius = radius;
        p.mass = 0.0f;
        out.particles.push_back(p);
        return (int)out.particles.size() - 1;
    };
    auto baseName = [&](int bone) {
        std::string n;
        for (char c : s.bones[(size_t)bone].name)
            n += (char)std::tolower((unsigned char)c);
        return n;
    };
    for (size_t bi = 0; bi < gb.size(); ++bi) {
        GB& b = gb[bi];
        const V3 p0 = pos[(size_t)b.bone];
        if (b.parentBody >= 0 && endBone[(size_t)b.parentBody] == b.bone) {
            b.pivotP = gb[(size_t)b.parentBody].endP;
        } else {
            b.pivotP = addP(baseName(b.bone), b.bone, p0, b.radius * 0.8f);
            if (b.parentBody >= 0) gb[(size_t)b.parentBody].cluster.push_back(b.pivotP);
        }
        const int eb = endBone[bi];
        b.endP = addP(eb >= 0 ? baseName(eb) : baseName(b.bone) + "_end", eb >= 0 ? eb : b.bone,
                      b.endPos, b.radius * 0.8f);
    }

    // Side pairs: a body whose cluster holds two particles spread across the
    // model's X axis (the hips on the pelvis, the shoulders on the chest)
    // builds its own frame from them; the root must have one, so a root
    // without lateral children gets two bracing particles.
    for (size_t bi = 0; bi < gb.size(); ++bi) {
        GB& b = gb[bi];
        int lo = -1, hi = -1;
        float xlo = 0, xhi = 0;
        for (int p : b.cluster) {
            const Particle& pp = out.particles[(size_t)p];
            float m[16];
            orthoCols(s.bones[(size_t)s.find(pp.bone)].bind, m);
            const float x = xformPoint(m, v3(pp.offset[0], pp.offset[1], pp.offset[2])).x;
            if (lo < 0 || x < xlo) lo = p, xlo = x;
            if (hi < 0 || x > xhi) hi = p, xhi = x;
        }
        const float spread = xhi - xlo;
        if (lo >= 0 && hi != lo && spread > std::max(0.25f * b.radius, 0.02f * height)) {
            b.sideA = hi, b.sideB = lo;  // +X is the model's left
        } else if (bi == 0) {
            const V3 p0 = pos[(size_t)b.bone];
            const float r = std::max(b.radius, 0.05f * height);
            b.sideA = addP(baseName(b.bone) + "_side_l", b.bone, p0 + v3(r, 0, 0), b.radius * 0.5f);
            b.sideB = addP(baseName(b.bone) + "_side_r", b.bone, p0 + v3(-r, 0, 0), b.radius * 0.5f);
            b.cluster.push_back(b.sideA);
            b.cluster.push_back(b.sideB);
        }
        if (b.sideA >= 0) {
            // a bracing particle in front: four points in one plane can fold
            // through it (a mirrored pelvis) - a fifth off the plane cannot
            const V3 p0 = pos[(size_t)b.bone];
            const V3 mid = (p0 + b.endPos) * 0.5f;
            const int f = addP(baseName(b.bone) + "_front", b.bone, mid + v3(0, 0, b.radius * 0.9f),
                               b.radius * 0.35f);
            b.cluster.push_back(f);
        }
    }

    // Masses: the total over the bodies by their capsules' volumes (r^2 (L +
    // 4r/3)), 45% to each end; cluster particles share the rest.
    std::vector<float> vol(gb.size(), 0.0f);
    float allVol = 0.0f;
    for (size_t bi = 0; bi < gb.size(); ++bi) {
        const float r = gb[bi].radius;
        vol[bi] = r * r * (len(gb[bi].endPos - pos[(size_t)gb[bi].bone]) + 4.0f * r / 3.0f);
        allVol += vol[bi];
    }
    for (size_t bi = 0; bi < gb.size(); ++bi) {
        const GB& b = gb[bi];
        const float m = opt.totalMass * vol[bi] / std::max(allVol, 1e-9f);
        out.particles[(size_t)b.pivotP].mass += m * 0.45f;
        out.particles[(size_t)b.endP].mass += m * 0.45f;
        for (int p : b.cluster) out.particles[(size_t)p].mass += m * 0.1f / (float)b.cluster.size();
        if (b.cluster.empty()) out.particles[(size_t)b.endP].mass += m * 0.1f;
    }
    for (Particle& p : out.particles) p.mass = std::max(p.mass, 0.1f);

    // Bodies, limits and directions.
    for (size_t bi = 0; bi < gb.size(); ++bi) {
        const GB& g = gb[bi];
        Body b;
        b.name = baseName(g.bone);
        b.pivot = out.particles[(size_t)g.pivotP].name;
        b.end = out.particles[(size_t)g.endP].name;
        b.parent = g.parentBody >= 0 ? baseName(gb[(size_t)g.parentBody].bone) : "";
        if (g.sideA >= 0) {
            b.sideA = out.particles[(size_t)g.sideA].name;
            b.sideB = out.particles[(size_t)g.sideB].name;
        }
        const V3 p0 = pos[(size_t)g.bone];
        const float piv[3] = {p0.x, p0.y, p0.z}, end[3] = {g.endPos.x, g.endPos.y, g.endPos.z};
        defaultJoint(s.bones[(size_t)g.bone].name, bi == 0, piv, end, b);
        out.bodies.push_back(b);
    }
    for (const GB& g : gb) {
        if (g.cluster.empty()) continue;
        std::vector<std::string> c = {out.particles[(size_t)g.pivotP].name,
                                      out.particles[(size_t)g.endP].name};
        for (int p : g.cluster) c.push_back(out.particles[(size_t)p].name);
        out.clusters.push_back(c);
    }
    // The torso particles also sweep against props in a game (the
    // world flag): the root's and its chain's pivots.
    if (!gb.empty()) {
        out.particles[(size_t)gb[0].pivotP].world = true;
        out.particles[(size_t)gb[0].endP].world = true;
    }

    // Two starter profiles - the Gunner examples: a ZOMBIE (absolute, the
    // legs at 1, the rest at 0.5) and a DEATH TUMBLE (relative 0.75, root free).
    Profile zombie, tumble;
    zombie.name = "zombie";
    tumble.name = "death-tumble";
    for (size_t bi = 0; bi < out.bodies.size(); ++bi) {
        const bool lowerBody = presetFor(s.bones[(size_t)gb[bi].bone].name, bi == 0).lower &&
                               !has(lower(s.bones[(size_t)gb[bi].bone].name), "spine") &&
                               !has(lower(s.bones[(size_t)gb[bi].bone].name), "chest");
        zombie.joints.push_back({out.bodies[bi].name, false, lowerBody ? 1.0f : 0.5f});
        tumble.joints.push_back({out.bodies[bi].name, true, bi == 0 ? 0.0f : 0.75f});
    }
    out.profiles = {zombie, tumble};

    char buf[256];
    std::snprintf(buf, sizeof(buf), "%zu bodies, %zu particles, %zu clusters from %zu bones "
                  "(min bone %.3f m of a %.2f m model)",
                  out.bodies.size(), out.particles.size(), out.clusters.size(), nb,
                  (double)minLen, (double)height);
    report = buf;
    return !out.bodies.empty();
}

void defaultJoint(const std::string& boneName, bool root, const float pivot[3],
                  const float end[3], Body& b) {
    const LimitPreset pr = presetFor(boneName, root);
    std::memcpy(b.pitch, pr.pitch, sizeof(b.pitch));
    std::memcpy(b.yaw, pr.yaw, sizeof(b.yaw));
    std::memcpy(b.roll, pr.roll, sizeof(b.roll));
    b.hinge = pr.hinge;
    b.groundFade = pr.lower;
    const V3 forward = v3(0, 0, 1);
    const V3 p0 = v3(pivot[0], pivot[1], pivot[2]);
    const V3 axis = unit(v3(end[0], end[1], end[2]) - p0, v3(0, 1, 0));
    V3 pd = pr.pitchKind == 2 ? v3(0, -1, 0) : (pr.pitchKind == 1 ? forward * -1.0f : forward);
    if (std::fabs(dot(unit(pd, pd), axis)) > 0.9f) pd = pr.pitchKind == 2 ? forward : v3(0, -1, 0);
    pd = unit(pd - axis * dot(pd, axis), v3(0, 0, 1));
    V3 yd = unit(cross(pd, axis), v3(1, 0, 0));
    // + yaw outward on both sides (the right side mirrors the left)
    if ((p0.x < -1e-3f && yd.x > 0.0f) || (p0.x > 1e-3f && yd.x < 0.0f)) yd = yd * -1.0f;
    auto rnd = [](float v) { return std::fabs(v) < 1e-4f ? 0.0f : std::round(v * 1000.0f) / 1000.0f; };
    b.pitchDir[0] = rnd(pd.x), b.pitchDir[1] = rnd(pd.y), b.pitchDir[2] = rnd(pd.z);
    b.yawDir[0] = rnd(yd.x), b.yawDir[1] = rnd(yd.y), b.yawDir[2] = rnd(yd.z);
}

std::string skeletonPathFor(const std::string& modelRel) {
    std::filesystem::path p(modelRel);
    p.replace_extension(".skeleton");
    return p.generic_string();
}
std::string ragdollPathFor(const std::string& modelRel) {
    std::filesystem::path p(modelRel);
    p.replace_extension(".ragdoll");
    return p.generic_string();
}

// =================================================================== files ==

namespace {
bool readText(const std::string& absPath, std::string& out) {
    std::FILE* f = std::fopen(absPath.c_str(), "rb");
    if (!f) return false;
    out.clear();
    char buf[8192];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) out.append(buf, n);
    std::fclose(f);
    return true;
}
std::string absOf(const std::string& projectDir, const std::string& rel) {
    return (std::filesystem::path(projectDir) / rel).make_preferred().string();
}
}  // namespace

bool loadSkeleton(const std::string& absPath, Skeleton& out, std::string& error) {
    std::string text;
    if (!readText(absPath, text)) {
        error = "cannot read " + absPath;
        return false;
    }
    return parseSkeleton(text, out, error);
}

bool loadAsset(const std::string& absPath, Asset& out, std::string& error) {
    std::string text;
    if (!readText(absPath, text)) {
        error = "cannot read " + absPath;
        return false;
    }
    return parseAsset(text, out, error);
}

bool saveText(const std::string& absPath, const std::string& text, std::string& error) {
    std::string old;
    if (readText(absPath, old) && old == text) return true;
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(absPath).parent_path(), ec);
    std::FILE* f = std::fopen(absPath.c_str(), "wb");
    if (!f) {
        error = "cannot write " + absPath;
        return false;
    }
    const bool ok = std::fwrite(text.data(), 1, text.size(), f) == text.size();
    std::fclose(f);
    if (!ok) error = "short write to " + absPath;
    return ok;
}

bool ensureSkeleton(const std::string& projectDir, const std::string& modelRel,
                    const glbparser::Skel& skel, std::string& note) {
    Skeleton fresh;
    if (!skeletonFromSkel(skel, modelRel, fresh)) {
        note = "no skin - no skeleton asset";
        return false;
    }
    const std::string rel = skeletonPathFor(modelRel);
    const std::string abs = absOf(projectDir, rel);
    Skeleton old;
    std::string err;
    const bool had = loadSkeleton(abs, old, err);
    if (had) fresh.model = old.model.empty() ? modelRel : old.model;
    if (!saveText(abs, skeletonJson(fresh), err)) {
        note = err;
        return false;
    }
    char buf[256];
    std::snprintf(buf, sizeof(buf), "skeleton %s (%zu bones)%s", rel.c_str(), fresh.bones.size(),
                  had ? "" : " created");
    note = buf;
    return true;
}

bool ensureRagdoll(const std::string& projectDir, const std::string& modelRel,
                   const glbparser::Skel& skel, const GenOptions& opt, std::string& note) {
    const std::string rel = ragdollPathFor(modelRel);
    const std::string abs = absOf(projectDir, rel);
    std::error_code ec;
    if (std::filesystem::exists(abs, ec)) {
        note = "ragdoll " + rel + " kept (the Ragdoll Editor's)";
        return true;
    }
    Skeleton s;
    std::string err;
    if (!loadSkeleton(absOf(projectDir, skeletonPathFor(modelRel)), s, err) &&
        !skeletonFromSkel(skel, modelRel, s)) {
        note = "no skeleton - no ragdoll";
        return false;
    }
    Asset a;
    std::string report;
    if (!generate(skel, s, opt, a, report)) {
        note = "ragdoll not generated: " + report;
        return false;
    }
    a.skeleton = skeletonPathFor(modelRel);
    // A header by default, so a game can include it as soon as it exists:
    // inc/ragdoll/<stem>.gen.hpp in namespace Ragdoll::<Stem>.
    {
        const std::string stem = std::filesystem::path(modelRel).stem().string();
        std::string ns;
        bool up = true;
        for (char c : stem) {
            if (!std::isalnum((unsigned char)c)) {
                up = true;
                continue;
            }
            ns += up ? (char)std::toupper((unsigned char)c) : c;
            up = false;
        }
        if (ns.empty() || std::isdigit((unsigned char)ns[0])) ns = "R" + ns;
        a.header = "inc/ragdoll/" + stem + ".gen.hpp";
        a.nameSpace = "Ragdoll::" + ns;
    }
    if (!saveText(abs, assetJson(a), err)) {
        note = err;
        return false;
    }
    note = "ragdoll " + rel + " generated: " + report;
    return true;
}

std::vector<std::pair<std::string, std::string>> projectHeaders(
    const std::string& projectDir, std::vector<std::string>& warnings) {
    std::vector<std::pair<std::string, std::string>> out;
    namespace fs = std::filesystem;
    const fs::path res = fs::path(projectDir) / "res";
    std::error_code ec;
    if (!fs::exists(res, ec)) return out;
    std::vector<fs::path> files;
    for (fs::recursive_directory_iterator it(res, ec), end; it != end; it.increment(ec)) {
        if (ec) break;
        if (it->is_regular_file(ec) && it->path().extension() == ".ragdoll") files.push_back(it->path());
    }
    std::sort(files.begin(), files.end());
    for (const fs::path& f : files) {
        const std::string rel = fs::relative(f, projectDir, ec).generic_string();
        Asset a;
        std::string err;
        if (!loadAsset(f.string(), a, err)) {
            warnings.push_back(rel + ": " + err);
            continue;
        }
        if (a.header.empty()) continue;
        Skeleton s;
        if (!loadSkeleton(absOf(projectDir, a.skeleton), s, err)) {
            warnings.push_back(rel + ": skeleton " + a.skeleton + ": " + err);
            continue;
        }
        Def d;
        std::vector<std::string> errors;
        if (!resolve(a, s, d, errors)) {
            for (const std::string& e : errors) warnings.push_back(rel + ": " + e);
            continue;
        }
        std::string path = a.header;
        std::replace(path.begin(), path.end(), '/', '\\');
        out.push_back({path, headerText(a, s, d, rel)});
    }
    return out;
}

}  // namespace ragdoll
