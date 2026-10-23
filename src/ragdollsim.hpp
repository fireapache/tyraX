#pragma once

#include <vector>

#include "ragdoll.hpp"

// The Ragdoll Editor's SIMULATION (docs/ragdoll-editor.md): a host port of the
// Verlet particle solver the Gunner game runs (gunner/src/gunner/ragdoll.cpp,
// .plan/35-36-39 there), driven by a resolved ragdoll::Def instead of a
// compile-time header, so the preview falls, tumbles and follows its motors
// the way the game's bodies do. One step is the game's: integrate (gravity,
// damping), the motors, the ground contacts and their Coulomb friction, the
// joints' frames and damping, the torso twist limit, then `iterations` passes
// of links, swing limits and ground / box contacts, then rolling resistance
// and sleep.
//
// Everything is in MODEL space of the skeleton (the preview puts the model at
// the origin), y up; the floor is a plane at `floorY`.
namespace ragdollsim {

struct V3 {
    float x = 0, y = 0, z = 0;
    V3() = default;
    V3(float a, float b, float c) : x(a), y(b), z(c) {}
    V3 operator+(V3 o) const { return {x + o.x, y + o.y, z + o.z}; }
    V3 operator-(V3 o) const { return {x - o.x, y - o.y, z - o.z}; }
    V3 operator*(float s) const { return {x * s, y * s, z * s}; }
};

struct Box {
    V3 mn, mx;  // an axis-aligned prop the body collides with (a step)
};

// The preview's staircase: a landing under the body (top at `topY`, out to
// z = 0.45) and nine 15 cm steps down from it toward +Z, 30 cm deep, 2 m
// wide. Returns the floor height below them (the new ground).
float stairs(float topY, std::vector<Box>& out);

class Sim {
   public:
    // Copies the def and builds its fixed tables (masses, bind frames,
    // joints). False when the def has no bodies.
    bool build(const ragdoll::Def& def);
    bool ready() const { return !def_.bodies.empty(); }
    const ragdoll::Def& def() const { return def_; }

    // Starts the ragdoll from a pose: every skeleton bone's model-space
    // global (16 floats each, ragdoll::Skeleton order). `velocity` (m/s) on
    // every particle; `push` (m/s) on the particles near `hitPoint`, toppling
    // (Gunner's ragdollApplyPush).
    void start(const std::vector<float>& boneGlobals16, V3 velocity = V3(),
               V3 hitPoint = V3(), V3 push = V3());
    bool started() const { return started_; }
    void applyPush(V3 hitPoint, V3 push);

    // Motorized: a profile index into Def::profiles (-1 = a plain ragdoll),
    // and the animation's pose as the motors' targets (bone globals, as
    // start takes them). A motorized ragdoll never sleeps.
    void setProfile(int profile);
    int profile() const { return profile_; }
    void setTargets(const std::vector<float>& boneGlobals16);

    void setFloor(float y) { floorY_ = y; }
    void setBoxes(const std::vector<Box>& boxes) { boxes_ = boxes; }
    void setGravity(bool on) { gravityOn_ = on; }

    // A PINCH (the editor's Ctrl+drag): the point `t` of the way from
    // particle `a` to particle `b` (b = -1: particle a itself) is pulled
    // toward `target` every step, like a hand holding the body there - the
    // links and limits still answer it. releaseGrab lets go.
    void setGrab(int a, int b, float t, V3 target);
    void releaseGrab() { grabA_ = -1; }
    bool grabbing() const { return grabA_ >= 0; }
    V3 grabPoint() const;  // where the held point is now
    V3 grabTarget() const { return grabTarget_; }

    void step();  // one fixed step of Solver::fixedDt

    // The pose: every skeleton bone's global (16 floats each), from each
    // body's motion since the start (bones no body drives keep their start
    // pose).
    void pose(std::vector<float>& boneGlobals16) const;

    // Inspection (the overlay and the readouts).
    const std::vector<V3>& particles() const { return p_; }
    const std::vector<V3>& targets() const { return target_; }
    bool hasTargets() const { return hasTarget_; }
    bool asleep() const { return asleep_; }
    float age() const { return age_; }
    float motorScale() const;  // the ground fade, 1 standing .. motorLying down
    // Each body's joint now, degrees: pitch, yaw (from the bind direction) and
    // roll - what the limits are compared against.
    void jointAngles(std::vector<float>& pitchYawRoll) const;
    // Each body's frame (9 floats, column-major: x along it) and pivot.
    void frames(std::vector<float>& frames9, std::vector<V3>& pivots) const;
    // Every body's bind frame (9 floats) - the limits' reference.
    const std::vector<float>& bindFrames() const { return bindFrame_; }
    // The swing limit's cone centre and axes of body b, in its PARENT's frame
    // (unit vectors), and the half-angle sines of its pitch / yaw ranges -
    // what the overlay draws the limit from.
    void limitCone(int b, V3& centre, V3& pitchAx, V3& yawAx, float& halfPitch,
                   float& halfYaw) const;
    V3 swingDirection(int b, float hp, float hy) const;

   private:
    struct Joint {
        float bind[9] = {};
        V3 centre, pitchAx, yawAx;
        float centreFrame[9] = {};
        float midPitch = 0, midYaw = 0, halfPitch = 0, halfYaw = 0;
        float rollLo = 0, rollHi = 0, rollSign = 1;
        int rollChild = -1;
        bool isParent = false;
        int bone = -1;
        int cluster[4] = {-1, -1, -1, -1};
    };

    void swungFrame(const V3* p, int b, const std::vector<float>& frames, float* out) const;
    void framesOf(const V3* p, std::vector<float>& out, std::vector<float>* unrolled,
                  bool parentsOnly) const;
    float sideTwist(const V3* p, int b, const std::vector<float>& frames) const;
    void deflection(int b, V3 v, float* hp, float* hy) const;
    void followHinge(int b, const float* z, bool limit);
    void stepFrames(std::vector<float>& out, std::vector<float>& unrolled);
    void turnCluster(int b, float angle);
    void limitTwist(int b, const std::vector<float>& frames);
    void limitSwing(int b, const float* rp);
    void dampJoints(const std::vector<float>& unrolled);
    void motorFrames(const V3* p, std::vector<float>& out) const;
    void driveMotors();
    void setRoll(int b, float roll);
    float groundAt(const V3& p, float wasBottom) const;
    bool pushOutOfBoxes(V3& p, float radius) const;

    ragdoll::Def def_;
    // fixed tables
    std::vector<float> inv_;            // inverse masses
    std::vector<float> linkShare_;      // 2 a link
    std::vector<float> limitShare_;     // 2 a body (q, b)
    std::vector<Joint> joint_;
    std::vector<float> bindFrame_;      // 9 a body
    std::vector<int> owner_;            // motor owner body per particle
    std::vector<float> ownedMass_;      // per body
    float bendMin_ = 0, bendFull_ = 1;
    // state
    bool started_ = false, asleep_ = false, hasTarget_ = false, gravityOn_ = true;
    int profile_ = -1;
    float age_ = 0, still_ = 0, lying_ = 0, floorY_ = 0;
    float touched_ = 0;  // age at the last push / grab
    std::vector<V3> p_, prev_, target_;
    std::vector<float> ground_, rest_;
    std::vector<float> roll_, rollCS_, swingLim_, rollLim_;
    std::vector<V3> dirPrev_, pivot0_;
    std::vector<float> frame0_;
    std::vector<float> node0_;  // 16 a bone: the start pose
    std::vector<Box> boxes_;
    int grabA_ = -1, grabB_ = -1;
    float grabT_ = 0.0f;
    V3 grabTarget_;
    void applyGrab();
};

}  // namespace ragdollsim
