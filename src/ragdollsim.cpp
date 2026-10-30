#include "ragdollsim.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

// The port keeps the game solver's structure and names (swungFrame,
// followHinge, limitSwing, ...) so the two can be read side by side; the
// comments that explain WHY live there (gunner/src/gunner/ragdoll.cpp).
namespace ragdollsim {

namespace {

constexpr float kPi = 3.14159265f;
constexpr float kDeg = kPi / 180.0f;

float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 cross(V3 a, V3 b) {
    return V3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
float length(V3 v) { return std::sqrt(dot(v, v)); }
V3 unit(V3 v, V3 fb) {
    const float l2 = dot(v, v);
    return l2 > 1e-12f ? v * (1.0f / std::sqrt(l2)) : fb;
}
V3 across(V3 v) {
    return unit(cross(v, std::fabs(v.x) < 0.9f ? V3(1, 0, 0) : V3(0, 1, 0)), V3(0, 0, 1));
}
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
float wrapAngle(float a) {
    while (a > kPi) a -= 2.0f * kPi;
    while (a <= -kPi) a += 2.0f * kPi;
    return a;
}

V3 col(const float* r, int c) { return V3(r[c * 3], r[c * 3 + 1], r[c * 3 + 2]); }
void setCols(float* r, V3 x, V3 y, V3 z) {
    r[0] = x.x, r[1] = x.y, r[2] = x.z;
    r[3] = y.x, r[4] = y.y, r[5] = y.z;
    r[6] = z.x, r[7] = z.y, r[8] = z.z;
}
V3 mul3(const float* r, V3 v) { return col(r, 0) * v.x + col(r, 1) * v.y + col(r, 2) * v.z; }
V3 mul3T(const float* r, V3 v) { return V3(dot(col(r, 0), v), dot(col(r, 1), v), dot(col(r, 2), v)); }
void mulABt(const float* a, const float* b, float* out) {
    for (int c = 0; c < 3; ++c)
        for (int r = 0; r < 3; ++r) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) s += a[k * 3 + r] * b[k * 3 + c];
            out[c * 3 + r] = s;
        }
}
V3 xform(const float* m, V3 v) {
    return V3(m[0] * v.x + m[4] * v.y + m[8] * v.z + m[12],
              m[1] * v.x + m[5] * v.y + m[9] * v.z + m[13],
              m[2] * v.x + m[6] * v.y + m[10] * v.z + m[14]);
}
void ortho(const float* in, float* out) {
    std::memcpy(out, in, 16 * sizeof(float));
    for (int c = 0; c < 3; ++c) {
        const float l = std::sqrt(in[c * 4] * in[c * 4] + in[c * 4 + 1] * in[c * 4 + 1] +
                                  in[c * 4 + 2] * in[c * 4 + 2]);
        if (l > 1e-9f)
            for (int r = 0; r < 3; ++r) out[c * 4 + r] = in[c * 4 + r] / l;
    }
}
// out = a * b, both affine (bottom row 0 0 0 1)
void mulAffine(const float* a, const float* b, float* out) {
    for (int c = 0; c < 4; ++c) {
        const float b0 = b[c * 4], b1 = b[c * 4 + 1], b2 = b[c * 4 + 2];
        const float w = c == 3 ? 1.0f : 0.0f;
        for (int r = 0; r < 3; ++r)
            out[c * 4 + r] = a[r] * b0 + a[4 + r] * b1 + a[8 + r] * b2 + a[12 + r] * w;
        out[c * 4 + 3] = w;
    }
}
void rigid(const float* r, V3 t, float* m) {
    for (int c = 0; c < 3; ++c) {
        for (int k = 0; k < 3; ++k) m[c * 4 + k] = r[c * 3 + k];
        m[c * 4 + 3] = 0.0f;
    }
    m[12] = t.x, m[13] = t.y, m[14] = t.z, m[15] = 1.0f;
}

struct Arc {
    V3 k, perp;
    float c, inv = 0.0f;
    bool flip;
    Arc(V3 from, V3 to) : k(cross(from, to)), c(dot(from, to)), flip(c < -0.9999f) {
        if (flip) perp = across(from);
        else inv = 1.0f / (1.0f + c);
    }
    V3 operator()(V3 u) const {
        if (flip) return perp * (2.0f * dot(perp, u)) - u;
        return u * c + cross(k, u) + k * (dot(k, u) * inv);
    }
};

void separate(V3& a, V3& b, float sa, float sb, float want) {
    const V3 d = b - a;
    const float d2 = dot(d, d);
    if (d2 < 1e-12f) return;
    const float k = 1.0f - want / std::sqrt(d2);
    a = a + d * (k * sa);
    b = b - d * (k * sb);
}

void sideFrame(const V3* p, const ragdoll::Def::B& bd, float* out) {
    const V3 x = unit(p[bd.b] - p[bd.a], V3(0, 1, 0));
    V3 z = cross(x, p[bd.sideA] - p[bd.sideB]);
    if (length(z) < 1e-5f) z = cross(x, V3(0, 0, 1));
    if (length(z) < 1e-5f) z = cross(x, V3(1, 0, 0));
    z = unit(z, V3(0, 0, 1));
    setCols(out, x, cross(z, x), z);
}

void rolled(const float* z, const float* cs, float* out) {
    const V3 y = col(z, 1), w = col(z, 2);
    setCols(out, col(z, 0), y * cs[0] + w * cs[1], w * cs[0] - y * cs[1]);
}

V3 nodeTurn(const float* now16, const float* bind16, V3 v) {
    const V3 l(bind16[0] * v.x + bind16[1] * v.y + bind16[2] * v.z,
               bind16[4] * v.x + bind16[5] * v.y + bind16[6] * v.z,
               bind16[8] * v.x + bind16[9] * v.y + bind16[10] * v.z);
    return V3(now16[0] * l.x + now16[4] * l.y + now16[8] * l.z,
              now16[1] * l.x + now16[5] * l.y + now16[9] * l.z,
              now16[2] * l.x + now16[6] * l.y + now16[10] * l.z);
}

V3 bindOf(const ragdoll::Def::P& p) { return V3(p.bind[0], p.bind[1], p.bind[2]); }

}  // namespace

float stairs(float topY, std::vector<Box>& out) {
    const float h = 0.15f, depth = 0.3f;
    const float bottom = topY - 10.0f * h;
    Box land;
    land.mn = V3(-1.0f, bottom, -1.0f);
    land.mx = V3(1.0f, topY, 0.45f);
    out.push_back(land);
    for (int s = 1; s < 10; ++s) {
        Box b;
        b.mn = V3(-1.0f, bottom, 0.45f + depth * (float)(s - 1));
        b.mx = V3(1.0f, topY - h * (float)s, 0.45f + depth * (float)s);
        out.push_back(b);
    }
    return bottom;
}

// ------------------------------------------------------------------- build --

bool Sim::build(const ragdoll::Def& def) {
    def_ = def;
    started_ = false;
    const int np = (int)def_.particles.size(), nb = (int)def_.bodies.size(),
              nl = (int)def_.links.size();
    if (nb == 0 || np == 0) return false;
    const ragdoll::Solver& sv = def_.solver;
    inv_.assign((size_t)np, 1.0f);
    for (int k = 0; k < np; ++k) inv_[(size_t)k] = 1.0f / def_.particles[(size_t)k].mass;
    auto shares = [&](int p, int q, float* out) {
        out[0] = inv_[(size_t)p] / (inv_[(size_t)p] + inv_[(size_t)q]);
        out[1] = inv_[(size_t)q] / (inv_[(size_t)p] + inv_[(size_t)q]);
    };
    linkShare_.assign((size_t)nl * 2, 0.5f);
    for (int i = 0; i < nl; ++i)
        shares(def_.links[(size_t)i].first, def_.links[(size_t)i].second, &linkShare_[(size_t)i * 2]);
    limitShare_.assign((size_t)nb * 2, 0.5f);
    for (int b = 0; b < nb; ++b)
        if (def_.bodies[(size_t)b].q >= 0)
            shares(def_.bodies[(size_t)b].q, def_.bodies[(size_t)b].b, &limitShare_[(size_t)b * 2]);

    // Joints (the game's Joints struct)
    std::vector<V3> bp((size_t)np);
    for (int k = 0; k < np; ++k) bp[(size_t)k] = bindOf(def_.particles[(size_t)k]);
    std::vector<char> linked((size_t)np * (size_t)np, 0);
    for (const auto& l : def_.links)
        linked[(size_t)l.first * (size_t)np + (size_t)l.second] =
            linked[(size_t)l.second * (size_t)np + (size_t)l.first] = 1;
    bendMin_ = std::sin(sv.rollBendMin * kDeg);
    bendFull_ = std::sin(sv.rollBendFull * kDeg);
    if (bendFull_ <= bendMin_ + 1e-4f) bendFull_ = bendMin_ + 1e-4f;
    joint_.assign((size_t)nb, Joint());
    bindFrame_.assign((size_t)nb * 9, 0.0f);
    for (int b = 0; b < nb; ++b) {
        const ragdoll::Def::B& bd = def_.bodies[(size_t)b];
        Joint& j = joint_[(size_t)b];
        j.bone = def_.particles[(size_t)bd.a].bone;
        const V3 x = unit(bp[(size_t)bd.b] - bp[(size_t)bd.a], V3(0, 1, 0));
        const V3 pd(bd.pitchDir[0], bd.pitchDir[1], bd.pitchDir[2]);
        const V3 yd(bd.yawDir[0], bd.yawDir[1], bd.yawDir[2]);
        int n = 0;
        if (bd.sideA >= 0) {
            sideFrame(bp.data(), bd, j.bind);
            for (int k = 0; k < np && n < 4; ++k)
                if (k != bd.a && k != bd.b && linked[(size_t)bd.a * (size_t)np + (size_t)k] &&
                    linked[(size_t)bd.b * (size_t)np + (size_t)k])
                    j.cluster[n++] = k;
        } else {
            const V3 y = unit(pd - x * dot(pd, x), across(x));
            setCols(j.bind, x, y, cross(x, y));
        }
        std::memcpy(&bindFrame_[(size_t)b * 9], j.bind, sizeof(j.bind));
        if (bd.parent < 0) continue;
        joint_[(size_t)bd.parent].isParent = true;
        if (bd.hinge) joint_[(size_t)bd.parent].rollChild = b;
        const float* pf = joint_[(size_t)bd.parent].bind;
        const V3 t = mul3T(pf, x);
        V3 pa = mul3T(pf, pd);
        pa = unit(pa - t * dot(pa, t), across(t));
        V3 ya = mul3T(pf, yd);
        ya = unit(ya - t * dot(ya, t) - pa * dot(ya, pa), cross(t, pa));
        j.rollSign = dot(cross(t, pa), ya) >= 0.0f ? 1.0f : -1.0f;
        j.midPitch = 0.5f * (bd.pitchMin + bd.pitchMax) * kDeg;
        j.midYaw = 0.5f * (bd.yawMin + bd.yawMax) * kDeg;
        j.halfPitch = std::sin(0.25f * (bd.pitchMax - bd.pitchMin) * kDeg);
        j.halfYaw = std::sin(0.25f * (bd.yawMax - bd.yawMin) * kDeg);
        const float m = std::sqrt(j.midPitch * j.midPitch + j.midYaw * j.midYaw);
        V3 c = t;
        if (m > 1e-6f) {
            const V3 h = (pa * j.midPitch + ya * j.midYaw) * (std::sin(0.5f * m) / m);
            const float hh = dot(h, h);
            c = t * (1.0f - 2.0f * hh) + h * (2.0f * std::sqrt(std::max(0.0f, 1.0f - hh)));
        }
        const Arc toCentre(t, c);
        j.centre = c;
        j.pitchAx = toCentre(pa);
        j.yawAx = toCentre(ya);
        for (int i = 0; i < 3; ++i) {
            const V3 v = toCentre(mul3T(pf, col(j.bind, i)));
            j.centreFrame[i * 3] = v.x, j.centreFrame[i * 3 + 1] = v.y, j.centreFrame[i * 3 + 2] = v.z;
        }
        const float lo = bd.rollMin * kDeg, hi = bd.rollMax * kDeg;
        j.rollLo = j.rollSign > 0.0f ? lo : -hi;
        j.rollHi = j.rollSign > 0.0f ? hi : -lo;
    }

    // Motor owners (the game's MotorMap)
    owner_.assign((size_t)np, -1);
    for (int b = 0; b < nb; ++b) {
        const ragdoll::Def::B& bd = def_.bodies[(size_t)b];
        auto own = [&](int k) {
            if (k >= 0 && owner_[(size_t)k] < 0) owner_[(size_t)k] = b;
        };
        if (bd.parent < 0) own(bd.a);
        own(bd.b);
        own(bd.sideA);
        own(bd.sideB);
    }
    for (int k = 0; k < np; ++k)
        if (owner_[(size_t)k] < 0) {
            const int bone = def_.particles[(size_t)k].bone;
            if (bone >= 0 && bone < (int)def_.boneBody.size()) owner_[(size_t)k] = def_.boneBody[(size_t)bone];
        }
    ownedMass_.assign((size_t)nb, 0.0f);
    for (int k = 0; k < np; ++k)
        if (owner_[(size_t)k] >= 0) ownedMass_[(size_t)owner_[(size_t)k]] += def_.particles[(size_t)k].mass;
    return true;
}

// ------------------------------------------------------------------ frames --

void Sim::swungFrame(const V3* p, int b, const std::vector<float>& frames, float* out) const {
    const Joint& j = joint_[(size_t)b];
    const ragdoll::Def::B& bd = def_.bodies[(size_t)b];
    const float* rp = &frames[(size_t)bd.parent * 9];
    const Arc swing(j.centre, unit(mul3T(rp, p[bd.b] - p[bd.a]), j.centre));
    for (int i = 0; i < 3; ++i) {
        const V3 v = mul3(rp, swing(col(j.centreFrame, i)));
        out[i * 3] = v.x, out[i * 3 + 1] = v.y, out[i * 3 + 2] = v.z;
    }
}

void Sim::setRoll(int b, float roll) {
    roll_[(size_t)b] = roll;
    rollCS_[(size_t)b * 2] = std::cos(roll);
    rollCS_[(size_t)b * 2 + 1] = std::sin(roll);
}

void Sim::framesOf(const V3* p, std::vector<float>& out, std::vector<float>* unrolled,
                   bool parentsOnly) const {
    const int nb = (int)def_.bodies.size();
    out.resize((size_t)nb * 9);
    if (unrolled) unrolled->resize((size_t)nb * 9);
    for (int b = 0; b < nb; ++b) {
        const ragdoll::Def::B& bd = def_.bodies[(size_t)b];
        if (parentsOnly && !joint_[(size_t)b].isParent) continue;
        float z[9];
        if (bd.sideA >= 0) {
            sideFrame(p, bd, &out[(size_t)b * 9]);
            std::memcpy(z, &out[(size_t)b * 9], sizeof(z));
        } else {
            swungFrame(p, b, out, z);
            rolled(z, &rollCS_[(size_t)b * 2], &out[(size_t)b * 9]);
        }
        if (unrolled) std::memcpy(&(*unrolled)[(size_t)b * 9], z, sizeof(z));
    }
}

float Sim::sideTwist(const V3* p, int b, const std::vector<float>& frames) const {
    float z[9];
    swungFrame(p, b, frames, z);
    const float* f = &frames[(size_t)b * 9];
    const V3 zy = col(z, 1), fy = col(f, 1);
    return std::atan2(dot(cross(zy, fy), col(f, 0)), dot(zy, fy));
}

void Sim::deflection(int b, V3 v, float* hp, float* hy) const {
    const Joint& j = joint_[(size_t)b];
    const float cv = dot(j.centre, v);
    const float s = 2.0f * (1.0f + cv);
    if (s < 1e-6f) {
        *hp = 1.0f, *hy = 0.0f;
        return;
    }
    const V3 e = v - j.centre * cv;
    const float inv = 1.0f / std::sqrt(s);
    *hp = dot(e, j.pitchAx) * inv;
    *hy = dot(e, j.yawAx) * inv;
}

V3 Sim::swingDirection(int b, float hp, float hy) const {
    const Joint& j = joint_[(size_t)b];
    float hh = hp * hp + hy * hy;
    if (hh > 1.0f) {
        const float k = 1.0f / std::sqrt(hh);
        hp *= k, hy *= k, hh = 1.0f;
    }
    return j.centre * (1.0f - 2.0f * hh) +
           (j.pitchAx * hp + j.yawAx * hy) * (2.0f * std::sqrt(std::max(0.0f, 1.0f - hh)));
}

void Sim::limitCone(int b, V3& centre, V3& pitchAx, V3& yawAx, float& halfPitch,
                    float& halfYaw) const {
    const Joint& j = joint_[(size_t)b];
    centre = j.centre, pitchAx = j.pitchAx, yawAx = j.yawAx;
    halfPitch = j.halfPitch, halfYaw = j.halfYaw;
}

void Sim::followHinge(int b, const float* z, bool limit) {
    const Joint& j = joint_[(size_t)b];
    const ragdoll::Def::B& hd = def_.bodies[(size_t)j.rollChild];
    const V3 u = unit(mul3T(z, p_[(size_t)hd.b] - p_[(size_t)hd.a]), V3(1, 0, 0));
    const float w = clampf((std::sqrt(u.y * u.y + u.z * u.z) - bendMin_) / (bendFull_ - bendMin_),
                           0.0f, 1.0f);
    if (w <= 0.0f) return;
    const V3 n = joint_[(size_t)j.rollChild].yawAx;
    const float a = u.y * n.y + u.z * n.z, s = u.z * n.y - u.y * n.z, c = u.x * n.x;
    const float len = std::sqrt(a * a + s * s);
    if (len < 1e-6f) return;
    const float base = std::atan2(s, a), off = std::acos(clampf(-c / len, -1.0f, 1.0f));
    const float d1 = wrapAngle(base + off - roll_[(size_t)b]),
                d2 = wrapAngle(base - off - roll_[(size_t)b]);
    float roll = roll_[(size_t)b] + w * (std::fabs(d1) < std::fabs(d2) ? d1 : d2);
    if (limit) roll = clampf(roll, rollLim_[(size_t)b * 2], rollLim_[(size_t)b * 2 + 1]);
    setRoll(b, roll);
}

void Sim::stepFrames(std::vector<float>& out, std::vector<float>& unrolled) {
    const int nb = (int)def_.bodies.size();
    out.resize((size_t)nb * 9);
    unrolled.resize((size_t)nb * 9);
    for (int b = 0; b < nb; ++b) {
        const ragdoll::Def::B& bd = def_.bodies[(size_t)b];
        if (bd.sideA >= 0) {
            sideFrame(p_.data(), bd, &out[(size_t)b * 9]);
            std::memcpy(&unrolled[(size_t)b * 9], &out[(size_t)b * 9], 9 * sizeof(float));
            continue;
        }
        swungFrame(p_.data(), b, out, &unrolled[(size_t)b * 9]);
        if (joint_[(size_t)b].rollChild >= 0) followHinge(b, &unrolled[(size_t)b * 9], true);
        rolled(&unrolled[(size_t)b * 9], &rollCS_[(size_t)b * 2], &out[(size_t)b * 9]);
    }
}

void Sim::turnCluster(int b, float angle) {
    const ragdoll::Def::B& bd = def_.bodies[(size_t)b];
    const V3 o = p_[(size_t)bd.a], x = unit(p_[(size_t)bd.b] - o, V3(0, 1, 0));
    const float c = std::cos(angle), s = std::sin(angle);
    const int* cl = joint_[(size_t)b].cluster;
    for (int i = 0; i < 4 && cl[i] >= 0; ++i) {
        const V3 v = p_[(size_t)cl[i]] - o;
        p_[(size_t)cl[i]] = o + v * c + cross(x, v) * s + x * (dot(x, v) * (1.0f - c));
    }
}

void Sim::limitTwist(int b, const std::vector<float>& frames) {
    const float t = sideTwist(p_.data(), b, frames);
    const float excess = t - clampf(t, rollLim_[(size_t)b * 2], rollLim_[(size_t)b * 2 + 1]);
    if (excess == 0.0f) return;
    const float half = 0.5f * excess * def_.solver.limitStiffness;
    turnCluster(b, -half);
    turnCluster(def_.bodies[(size_t)b].parent, half);
}

void Sim::limitSwing(int b, const float* rp) {
    const ragdoll::Def::B& bd = def_.bodies[(size_t)b];
    if (bd.q < 0) return;
    const V3 v = p_[(size_t)bd.b] - p_[(size_t)bd.a], u = p_[(size_t)bd.q] - p_[(size_t)bd.a];
    const float lv2 = dot(v, v), lu2 = dot(u, u);
    if (lv2 < 1e-12f || lu2 < 1e-12f) return;
    const float iv = 1.0f / std::sqrt(lv2), iu = 1.0f / std::sqrt(lu2);
    const V3 now = v * iv;
    float hp, hy;
    deflection(b, mul3T(rp, now), &hp, &hy);
    const float* lim = &swingLim_[(size_t)b * 4];
    const float cp = clampf(hp, lim[0], lim[1]), cy = clampf(hy, lim[2], lim[3]);
    if (cp == hp && cy == hy) return;
    const V3 want = mul3(rp, swingDirection(b, cp, cy));
    const V3 axis = cross(now, want);
    const float s2 = dot(axis, axis);
    if (s2 < 1e-12f) return;
    const V3 k = axis * (1.0f / std::sqrt(s2));
    const V3 gb = cross(k, now) * iv;
    const V3 gq = cross(k, u * iu) * -iu;
    const V3 ga = (gb + gq) * -1.0f;
    const float wb = inv_[(size_t)bd.b], wq = inv_[(size_t)bd.q], wa = inv_[(size_t)bd.a];
    const float denom = wb * dot(gb, gb) + wq * dot(gq, gq) + wa * dot(ga, ga);
    if (denom < 1e-12f) return;
    const float lambda = length(want - now) * def_.solver.limitStiffness / denom;
    p_[(size_t)bd.b] = p_[(size_t)bd.b] + gb * (lambda * wb);
    p_[(size_t)bd.q] = p_[(size_t)bd.q] + gq * (lambda * wq);
    p_[(size_t)bd.a] = p_[(size_t)bd.a] + ga * (lambda * wa);
}

void Sim::dampJoints(const std::vector<float>& unrolled) {
    const float k = def_.solver.jointDamping;
    for (int b = 0; b < (int)def_.bodies.size(); ++b) {
        const ragdoll::Def::B& bd = def_.bodies[(size_t)b];
        if (bd.parent < 0 || bd.q < 0) continue;
        const float* zp = &unrolled[(size_t)bd.parent * 9];
        const V3 d = p_[(size_t)bd.b] - p_[(size_t)bd.a];
        const V3 dir = unit(mul3T(zp, d), dirPrev_[(size_t)b]);
        const V3 turn = mul3(zp, cross(dirPrev_[(size_t)b], dir));
        dirPrev_[(size_t)b] = dir;
        prev_[(size_t)bd.b] = prev_[(size_t)bd.b] + cross(turn, d) * (k * limitShare_[(size_t)b * 2 + 1]);
        prev_[(size_t)bd.q] = prev_[(size_t)bd.q] -
                              cross(turn, p_[(size_t)bd.q] - p_[(size_t)bd.a]) * (k * limitShare_[(size_t)b * 2]);
    }
}

void Sim::motorFrames(const V3* p, std::vector<float>& out) const {
    const int nb = (int)def_.bodies.size();
    out.resize((size_t)nb * 9);
    for (int b = 0; b < nb; ++b) {
        const ragdoll::Def::B& bd = def_.bodies[(size_t)b];
        if (bd.sideA >= 0) {
            sideFrame(p, bd, &out[(size_t)b * 9]);
            continue;
        }
        const V3 x = unit(p[bd.b] - p[bd.a], V3(0, 1, 0));
        const V3 ref = bd.parent >= 0 ? col(&out[(size_t)bd.parent * 9], 2) : V3(0, 0, 1);
        V3 z = ref - x * dot(ref, x);
        if (dot(z, z) < 1e-8f) z = across(x);
        z = unit(z, across(x));
        setCols(&out[(size_t)b * 9], x, cross(z, x), z);
    }
}

// ------------------------------------------------------------------- grab --

void Sim::setGrab(int a, int b, float t, V3 target) {
    const int np = (int)p_.size();
    if (a < 0 || a >= np || b >= np) return;
    grabA_ = a, grabB_ = b, grabT_ = clampf(t, 0.0f, 1.0f), grabTarget_ = target;
    asleep_ = false, still_ = 0.0f, touched_ = age_;
}

V3 Sim::grabPoint() const {
    if (grabA_ < 0 || grabA_ >= (int)p_.size()) return V3();
    if (grabB_ < 0) return p_[(size_t)grabA_];
    return p_[(size_t)grabA_] * (1.0f - grabT_) + p_[(size_t)grabB_] * grabT_;
}

void Sim::applyGrab() {
    if (grabA_ < 0 || grabA_ >= (int)p_.size()) return;
    // Close half the gap a step (a firm hand, not a teleport), split over the
    // two particles by where the point sits between them; most of the move is
    // a position fix, so a held body does not wind up and fly off on release.
    constexpr float kStiffness = 0.5f, kKeep = 0.3f;
    const V3 d = (grabTarget_ - grabPoint()) * kStiffness;
    auto move = [&](int k, V3 m) {
        p_[(size_t)k] = p_[(size_t)k] + m;
        prev_[(size_t)k] = prev_[(size_t)k] + m * (1.0f - kKeep);
    };
    if (grabB_ < 0) {
        move(grabA_, d);
    } else {
        const float wa = 1.0f - grabT_, wb = grabT_;
        const float s = 1.0f / std::max(wa * wa + wb * wb, 1e-4f);  // the point moves by d
        move(grabA_, d * (wa * s));
        move(grabB_, d * (wb * s));
    }
    still_ = 0.0f;
}

// ------------------------------------------------------------------ motors --

float Sim::motorScale() const {
    const ragdoll::Solver& sv = def_.solver;
    float t = sv.motorFadeTime > 1e-4f ? lying_ / sv.motorFadeTime : 1.0f;
    t = clampf(t, 0.0f, 1.0f);
    t = t * t * (3.0f - 2.0f * t);
    return 1.0f + (sv.motorLying - 1.0f) * t;
}

void Sim::setProfile(int profile) {
    profile_ = profile >= 0 && profile < (int)def_.profiles.size() ? profile : -1;
    if (profile_ >= 0) asleep_ = false, still_ = 0.0f;
}

void Sim::setTargets(const std::vector<float>& g) {
    const int np = (int)def_.particles.size();
    target_.resize((size_t)np);
    for (int k = 0; k < np; ++k) {
        const ragdoll::Def::P& pt = def_.particles[(size_t)k];
        if (pt.bone < 0 || (size_t)pt.bone * 16 + 16 > g.size()) continue;
        float m[16];
        ortho(&g[(size_t)pt.bone * 16], m);
        target_[(size_t)k] = xform(m, V3(pt.offset[0], pt.offset[1], pt.offset[2]));
    }
    hasTarget_ = true;
}

void Sim::driveMotors() {
    if (profile_ < 0 || !hasTarget_) return;
    const ragdoll::Def::Prof& pf = def_.profiles[(size_t)profile_];
    const int np = (int)def_.particles.size(), nb = (int)def_.bodies.size();
    const float lying = motorScale();
    bool anyRelative = false;
    for (int b = 0; b < nb; ++b) anyRelative |= pf.relative[(size_t)b] && pf.ratio[(size_t)b] > 0.0f;
    std::vector<float> cur, tgt;
    V3 rootCur, rootTgt;
    if (anyRelative) {
        motorFrames(p_.data(), cur);
        motorFrames(target_.data(), tgt);
        int n = 0;
        for (int k = 0; k < np; ++k) {
            const int o = owner_[(size_t)k];
            if (o < 0 || def_.bodies[(size_t)o].parent >= 0) continue;
            rootCur = rootCur + p_[(size_t)k];
            rootTgt = rootTgt + target_[(size_t)k];
            ++n;
        }
        if (n > 0) {
            rootCur = rootCur * (1.0f / (float)n);
            rootTgt = rootTgt * (1.0f / (float)n);
        }
    }
    const float keep = def_.solver.motorVelocity;
    auto move = [&](int k, V3 d) {
        p_[(size_t)k] = p_[(size_t)k] + d;
        prev_[(size_t)k] = prev_[(size_t)k] + d * (1.0f - keep);
    };
    auto toLocal = [](const float* f, V3 v) { return V3(dot(col(f, 0), v), dot(col(f, 1), v), dot(col(f, 2), v)); };
    auto toWorld = [](const float* f, V3 l) { return col(f, 0) * l.x + col(f, 1) * l.y + col(f, 2) * l.z; };
    for (int k = 0; k < np; ++k) {
        const int b = owner_[(size_t)k];
        if (b < 0) continue;
        const float ratio = pf.ratio[(size_t)b];
        if (ratio <= 0.0f) continue;
        const ragdoll::Def::B& bd = def_.bodies[(size_t)b];
        const float g = std::min(ratio, 1.0f) * def_.solver.motorGain * (bd.groundFade ? lying : 1.0f);
        if (!pf.relative[(size_t)b]) {
            move(k, (target_[(size_t)k] - p_[(size_t)k]) * g);
            continue;
        }
        if (bd.parent < 0) {
            const V3 want = rootCur + toWorld(&cur[(size_t)b * 9], toLocal(&tgt[(size_t)b * 9], target_[(size_t)k] - rootTgt));
            move(k, (want - p_[(size_t)k]) * g);
            continue;
        }
        if (k == bd.a) continue;
        const float* pc = &cur[(size_t)bd.parent * 9];
        const float* pt = &tgt[(size_t)bd.parent * 9];
        const V3 want = p_[(size_t)bd.a] + toWorld(pc, toLocal(pt, target_[(size_t)k] - target_[(size_t)bd.a]));
        const V3 d = (want - p_[(size_t)k]) * g;
        const float mk = def_.particles[(size_t)k].mass;
        const float mp = ownedMass_[(size_t)bd.parent];
        move(k, d * (mp / (mk + mp)));
        const V3 back = d * (-mk / (mk + mp));
        for (int i = 0; i < np; ++i)
            if (owner_[(size_t)i] == bd.parent) move(i, back);
    }
}

// ------------------------------------------------------------------- start --

void Sim::start(const std::vector<float>& g, V3 velocity, V3 hitPoint, V3 push) {
    const int np = (int)def_.particles.size(), nb = (int)def_.bodies.size(),
              nl = (int)def_.links.size();
    const size_t bones = g.size() / 16;
    node0_ = g;
    p_.assign((size_t)np, V3());
    prev_.assign((size_t)np, V3());
    target_.assign((size_t)np, V3());
    ground_.assign((size_t)np, floorY_);
    rest_.assign((size_t)nl, 0.0f);
    roll_.assign((size_t)nb, 0.0f);
    rollCS_.assign((size_t)nb * 2, 0.0f);
    swingLim_.assign((size_t)nb * 4, 0.0f);
    rollLim_.assign((size_t)nb * 2, 0.0f);
    dirPrev_.assign((size_t)nb, V3(1, 0, 0));
    pivot0_.assign((size_t)nb, V3());
    frame0_.assign((size_t)nb * 9, 0.0f);
    asleep_ = hasTarget_ = false;
    age_ = still_ = lying_ = touched_ = 0.0f;
    grabA_ = -1;
    const float dt = def_.solver.fixedDt;
    for (int k = 0; k < np; ++k) {
        const ragdoll::Def::P& pt = def_.particles[(size_t)k];
        float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        if (pt.bone >= 0 && (size_t)pt.bone < bones) ortho(&g[(size_t)pt.bone * 16], m);
        p_[(size_t)k] = xform(m, V3(pt.offset[0], pt.offset[1], pt.offset[2]));
        prev_[(size_t)k] = p_[(size_t)k] - velocity * dt;
    }
    for (int i = 0; i < nl; ++i)
        rest_[(size_t)i] = length(p_[(size_t)def_.links[(size_t)i].second] - p_[(size_t)def_.links[(size_t)i].first]);
    std::vector<float> unrolled((size_t)nb * 9, 0.0f);
    for (int b = 0; b < nb; ++b) {
        const ragdoll::Def::B& bd = def_.bodies[(size_t)b];
        const Joint& j = joint_[(size_t)b];
        rollLim_[(size_t)b * 2] = j.rollLo;
        rollLim_[(size_t)b * 2 + 1] = j.rollHi;
        swingLim_[(size_t)b * 4] = -j.halfPitch;
        swingLim_[(size_t)b * 4 + 1] = j.halfPitch;
        swingLim_[(size_t)b * 4 + 2] = -j.halfYaw;
        swingLim_[(size_t)b * 4 + 3] = j.halfYaw;
        setRoll(b, 0.0f);
        if (bd.sideA >= 0) {
            sideFrame(p_.data(), bd, &frame0_[(size_t)b * 9]);
            std::memcpy(&unrolled[(size_t)b * 9], &frame0_[(size_t)b * 9], 9 * sizeof(float));
            continue;
        }
        swungFrame(p_.data(), b, frame0_, &unrolled[(size_t)b * 9]);
        if (j.bone >= 0 && (size_t)j.bone < bones) {
            const float* uz = &unrolled[(size_t)b * 9];
            const V3 x = col(uz, 0), zy = col(uz, 1);
            float nowO[16];
            ortho(&g[(size_t)j.bone * 16], nowO);
            V3 ny = nodeTurn(nowO, &def_.boneBind[(size_t)j.bone * 16], col(j.bind, 1));
            ny = ny - x * dot(ny, x);
            if (dot(ny, ny) > 1e-8f) setRoll(b, std::atan2(dot(cross(zy, ny), x), dot(zy, ny)));
        }
        if (j.rollChild >= 0) followHinge(b, &unrolled[(size_t)b * 9], false);
        rollLim_[(size_t)b * 2] = std::min(rollLim_[(size_t)b * 2], roll_[(size_t)b]);
        rollLim_[(size_t)b * 2 + 1] = std::max(rollLim_[(size_t)b * 2 + 1], roll_[(size_t)b]);
        rolled(&unrolled[(size_t)b * 9], &rollCS_[(size_t)b * 2], &frame0_[(size_t)b * 9]);
    }
    for (int b = 0; b < nb; ++b) {
        const ragdoll::Def::B& bd = def_.bodies[(size_t)b];
        pivot0_[(size_t)b] = p_[(size_t)bd.a];
        if (bd.parent < 0) continue;
        const V3 d = p_[(size_t)bd.b] - p_[(size_t)bd.a];
        float hp, hy;
        deflection(b, unit(mul3T(&frame0_[(size_t)bd.parent * 9], d), joint_[(size_t)b].centre), &hp, &hy);
        float* sl = &swingLim_[(size_t)b * 4];
        sl[0] = std::min(sl[0], hp), sl[1] = std::max(sl[1], hp);
        sl[2] = std::min(sl[2], hy), sl[3] = std::max(sl[3], hy);
        if (bd.sideA >= 0) {
            const float t = sideTwist(p_.data(), b, frame0_);
            rollLim_[(size_t)b * 2] = std::min(rollLim_[(size_t)b * 2], t);
            rollLim_[(size_t)b * 2 + 1] = std::max(rollLim_[(size_t)b * 2 + 1], t);
        }
        dirPrev_[(size_t)b] = unit(mul3T(&unrolled[(size_t)bd.parent * 9], d), V3(1, 0, 0));
    }
    started_ = true;
    applyPush(hitPoint, push);
}

void Sim::applyPush(V3 hitPoint, V3 push) {
    if (length(push) <= 0.0f) return;
    const ragdoll::Solver& sv = def_.solver;
    const int np = (int)p_.size();
    std::vector<float> w((size_t)np, 0.0f);
    float any = 0.0f, lo = p_[0].y, hi = p_[0].y;
    int nearest = 0;
    float nearestD = 1e9f;
    for (const V3& q : p_) lo = std::min(lo, q.y), hi = std::max(hi, q.y);
    const float span = hi - lo > 0.1f ? hi - lo : 0.1f;
    for (int k = 0; k < np; ++k) {
        const float d = length(p_[(size_t)k] - hitPoint);
        w[(size_t)k] = std::max(0.0f, 1.0f - d / std::max(sv.pushRadius, 1e-3f));
        any += w[(size_t)k];
        if (d < nearestD) nearestD = d, nearest = k;
    }
    if (any <= 0.0f) w[(size_t)nearest] = 1.0f;
    for (int k = 0; k < np; ++k)
        w[(size_t)k] = std::max(w[(size_t)k], sv.pushTopple * (p_[(size_t)k].y - lo) / span);
    for (int k = 0; k < np; ++k)
        prev_[(size_t)k] = prev_[(size_t)k] - push * ((w[(size_t)k] + sv.pushBody) * sv.fixedDt);
    asleep_ = false, still_ = 0.0f, touched_ = age_;
}

// -------------------------------------------------------------------- step --

float Sim::groundAt(const V3& p, float wasBottom) const {
    float g = floorY_;
    for (const Box& b : boxes_) {
        if (p.x < b.mn.x || p.x > b.mx.x || p.z < b.mn.z || p.z > b.mx.z) continue;
        if (b.mx.y > wasBottom + def_.solver.boxStepUp || b.mx.y <= g) continue;
        g = b.mx.y;
    }
    return g;
}

bool Sim::pushOutOfBoxes(V3& p, float radius) const {
    bool moved = false;
    for (const Box& b : boxes_) {
        if (p.y >= b.mx.y || p.y + radius <= b.mn.y) continue;
        const float cx = p.x < b.mn.x ? b.mn.x : (p.x > b.mx.x ? b.mx.x : p.x);
        const float cz = p.z < b.mn.z ? b.mn.z : (p.z > b.mx.z ? b.mx.z : p.z);
        const float dx = p.x - cx, dz = p.z - cz;
        const float d2 = dx * dx + dz * dz;
        if (d2 >= radius * radius) continue;
        if (d2 > 1e-10f) {
            const float d = std::sqrt(d2);
            const float s = (radius - d) / d;
            p.x += dx * s;
            p.z += dz * s;
            moved = true;
            continue;
        }
        const float up = b.mx.y - p.y + radius;
        const float px0 = p.x - b.mn.x + radius, px1 = b.mx.x - p.x + radius;
        const float pz0 = p.z - b.mn.z + radius, pz1 = b.mx.z - p.z + radius;
        float best = up;
        int axis = 0;
        if (px0 < best) best = px0, axis = 1;
        if (px1 < best) best = px1, axis = 2;
        if (pz0 < best) best = pz0, axis = 3;
        if (pz1 < best) best = pz1, axis = 4;
        switch (axis) {
            case 0: p.y += up; break;
            case 1: p.x -= px0; break;
            case 2: p.x += px1; break;
            case 3: p.z -= pz0; break;
            default: p.z += pz1; break;
        }
        moved = true;
    }
    return moved;
}

void Sim::step() {
    if (!started_ || asleep_) return;
    const ragdoll::Solver& sv = def_.solver;
    const float dt = sv.fixedDt;
    const int np = (int)def_.particles.size(), nb = (int)def_.bodies.size(),
              nl = (int)def_.links.size();
    const V3 gravity(0.0f, gravityOn_ ? -sv.gravity * dt * dt : 0.0f, 0.0f);
    std::vector<V3> start(p_);
    std::vector<float> load((size_t)np, 0.0f);
    for (int k = 0; k < np; ++k) {
        const V3 v = (p_[(size_t)k] - prev_[(size_t)k]) * sv.damping;
        prev_[(size_t)k] = p_[(size_t)k];
        p_[(size_t)k] = p_[(size_t)k] + v + gravity;
        ground_[(size_t)k] = groundAt(p_[(size_t)k], prev_[(size_t)k].y - def_.particles[(size_t)k].radius);
    }
    driveMotors();
    applyGrab();
    for (int k = 0; k < np; ++k) {
        const float floor = ground_[(size_t)k] + def_.particles[(size_t)k].radius;
        if (p_[(size_t)k].y >= floor) continue;
        load[(size_t)k] = floor - p_[(size_t)k].y;
        p_[(size_t)k].y = floor;
    }
    for (int k = 0; k < np; ++k) {  // Coulomb friction
        if (load[(size_t)k] <= 0.0f) continue;
        const float dx = p_[(size_t)k].x - start[(size_t)k].x, dz = p_[(size_t)k].z - start[(size_t)k].z;
        const float slide = std::sqrt(dx * dx + dz * dz);
        if (slide <= sv.staticFriction * load[(size_t)k]) {
            p_[(size_t)k].x = prev_[(size_t)k].x = start[(size_t)k].x;
            p_[(size_t)k].z = prev_[(size_t)k].z = start[(size_t)k].z;
        } else {
            const float cut = sv.kineticFriction * load[(size_t)k] / slide;
            p_[(size_t)k].x -= dx * cut;
            p_[(size_t)k].z -= dz * cut;
        }
    }
    std::vector<float> now, unrolled;
    stepFrames(now, unrolled);
    dampJoints(unrolled);
    for (int b = 0; b < nb; ++b)
        if (def_.bodies[(size_t)b].sideA >= 0 && def_.bodies[(size_t)b].parent >= 0) limitTwist(b, now);
    for (int it = 0; it < sv.iterations; ++it) {
        const bool back = (it & 1) != 0;
        for (int n = 0; n < nl; ++n) {
            const int i = back ? nl - 1 - n : n;
            const auto& l = def_.links[(size_t)i];
            separate(p_[(size_t)l.first], p_[(size_t)l.second], linkShare_[(size_t)i * 2],
                     linkShare_[(size_t)i * 2 + 1], rest_[(size_t)i]);
        }
        if (it > 0) framesOf(p_.data(), now, nullptr, true);
        for (int n = 0; n < nb; ++n) {
            const int b = back ? nb - 1 - n : n;
            if (def_.bodies[(size_t)b].parent >= 0)
                limitSwing(b, &now[(size_t)def_.bodies[(size_t)b].parent * 9]);
        }
        for (int k = 0; k < np; ++k) {
            const float r = def_.particles[(size_t)k].radius;
            const float floor = ground_[(size_t)k] + r;
            if (p_[(size_t)k].y < floor) p_[(size_t)k].y = floor;
            if (!boxes_.empty() && pushOutOfBoxes(p_[(size_t)k], r)) {
                const float f2 = ground_[(size_t)k] + r;
                if (p_[(size_t)k].y < f2) p_[(size_t)k].y = f2;
            }
        }
    }
    float moved = 0.0f;
    for (int k = 0; k < np; ++k) {
        const float floor = ground_[(size_t)k] + def_.particles[(size_t)k].radius;
        V3& p = p_[(size_t)k];
        V3& q = prev_[(size_t)k];
        if (p.y <= floor + sv.contactSlop) {
            q.x = p.x - (p.x - q.x) * (1.0f - sv.rolling);
            q.z = p.z - (p.z - q.z) * (1.0f - sv.rolling);
            q.y = p.y - (p.y - q.y) * (1.0f - sv.rolling);
            if (q.y > p.y) q.y = p.y;
        }
        moved = std::max(moved, length(p - q));
    }
    age_ += dt;
    const int root = 0;
    if (lying_ > 0.0f ||
        p_[(size_t)def_.bodies[root].a].y <=
            ground_[(size_t)def_.bodies[root].a] + def_.particles[(size_t)def_.bodies[root].a].radius + sv.lyingSlop)
        lying_ = std::min(lying_ + dt, sv.motorFadeTime);
    still_ = moved < sv.sleepSpeed * dt ? still_ + dt : 0.0f;
    // the cost cap counts from the last touch (a push, a grab), and a held
    // body never sleeps
    if (profile_ < 0 && grabA_ < 0 &&
        (still_ >= sv.sleepTime || age_ - touched_ >= sv.maxAwake))
        asleep_ = true;
}

// -------------------------------------------------------------------- pose --

void Sim::frames(std::vector<float>& frames9, std::vector<V3>& pivots) const {
    framesOf(p_.data(), frames9, nullptr, false);
    pivots.resize(def_.bodies.size());
    for (size_t b = 0; b < def_.bodies.size(); ++b) pivots[b] = p_[(size_t)def_.bodies[b].a];
}

void Sim::jointAngles(std::vector<float>& out) const {
    const int nb = (int)def_.bodies.size();
    out.assign((size_t)nb * 3, 0.0f);
    if (!started_) return;
    std::vector<float> f;
    framesOf(p_.data(), f, nullptr, false);
    for (int b = 0; b < nb; ++b) {
        const ragdoll::Def::B& bd = def_.bodies[(size_t)b];
        if (bd.parent < 0) continue;
        const Joint& j = joint_[(size_t)b];
        float hp, hy;
        deflection(b, unit(mul3T(&f[(size_t)bd.parent * 9], p_[(size_t)bd.b] - p_[(size_t)bd.a]), j.centre), &hp, &hy);
        out[(size_t)b * 3] = (j.midPitch + 2.0f * std::asin(clampf(hp, -1.0f, 1.0f))) / kDeg;
        out[(size_t)b * 3 + 1] = (j.midYaw + 2.0f * std::asin(clampf(hy, -1.0f, 1.0f))) / kDeg;
        const float roll = bd.sideA >= 0 ? sideTwist(p_.data(), b, f) : roll_[(size_t)b];
        out[(size_t)b * 3 + 2] = roll * j.rollSign / kDeg;
    }
}

void Sim::pose(std::vector<float>& out) const {
    out = node0_;
    if (!started_) return;
    std::vector<float> now;
    std::vector<V3> pivots;
    frames(now, pivots);
    const int nb = (int)def_.bodies.size();
    std::vector<float> move((size_t)nb * 16);
    for (int b = 0; b < nb; ++b) {
        float rot[9];
        mulABt(&now[(size_t)b * 9], &frame0_[(size_t)b * 9], rot);
        rigid(rot, pivots[(size_t)b] - mul3(rot, pivot0_[(size_t)b]), &move[(size_t)b * 16]);
    }
    const size_t bones = node0_.size() / 16;
    for (size_t n = 0; n < bones && n < def_.boneBody.size(); ++n) {
        const int b = def_.boneBody[n];
        if (b < 0) continue;
        mulAffine(&move[(size_t)b * 16], &node0_[n * 16], &out[n * 16]);
    }
}

}  // namespace ragdollsim
