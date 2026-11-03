#pragma once

#include <string>
#include <vector>

namespace glbparser {
struct Skel;
}

// Skeleton and ragdoll ASSETS (docs/ragdoll-editor.md).
//
// A skinned model imported into a project gets a `<stem>.skeleton` beside it:
// its bone hierarchy and every bone's bind matrix, named so that other models
// of exactly the same structure can share it (and with it its animations and
// its ragdoll) later. A `<stem>.ragdoll` describes a Verlet PARTICLE ragdoll on
// that skeleton - the representation the Gunner game simulates, made generic:
//
//   particles  points riding a bone (an offset in its bind frame), each with a
//              radius (what touches the ground) and a mass
//   bodies     one per simulated joint: the segment pivot -> end, hung on its
//              parent body at the pivot, with a SWING limit (pitch / yaw, in
//              degrees from the bind direction, about pitchDir / yawDir) and a
//              ROLL limit (twist about its own axis); a HINGE bends in pitch
//              only. A body with a SIDE pair (the pelvis, the chest) builds
//              its own frame from it - the root needs one
//   clusters   particle groups held rigid (every pair linked)
//   profiles   MOTOR profiles for a motorized ragdoll: per body, ABSOLUTE
//              (pulled toward the animation's pose in the world) or RELATIVE
//              (toward its pose against the parent body), and a 0..1 ratio
//   solver     the preview's simulation settings
//
// The Unreal Engine counterpart is the Physics Asset (bodies + constraints)
// with its Physical Animation Profiles; the Ragdoll Editor (Tools) is the
// Physics Asset Editor's counterpart. Assets are JSON, editor-only (they never
// ship - texbake skips them); a ragdoll with a `header` path is turned into a
// plain C++ data header at every build (`headerText`), which is how a game
// consumes it.
namespace ragdoll {

// ---------------------------------------------------------------- skeleton --
struct Bone {
    std::string name;
    int parent = -1;  // index into Skeleton::bones, -1 = a root
    // Model-space bind (rest) matrix, column-major: the bone's global
    // transform in the pose the mesh was bound in.
    float bind[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
};

struct Skeleton {
    std::string model;  // project-relative source model (.glb/.fbx)
    std::vector<Bone> bones;
    int find(const std::string& name) const;
};

// The skeleton of a parsed model: its skin joints (the palette's nodes, in
// palette order, each once), parented to their nearest ancestor that is also a
// joint. False when the model has no skin.
bool skeletonFromSkel(const glbparser::Skel& skel, const std::string& modelRel,
                      Skeleton& out);
std::string skeletonJson(const Skeleton& s);
bool parseSkeleton(const std::string& text, Skeleton& out, std::string& error);

// ------------------------------------------------------------------ ragdoll --
struct Particle {
    std::string name;
    std::string bone;  // the skeleton bone it rides
    float offset[3] = {0, 0, 0};  // in that bone's bind frame
    float radius = 0.05f;  // m
    float mass = 1.0f;     // kg
    bool world = false;    // also collides with props and walls (game side)
};

struct Body {
    std::string name;
    std::string pivot, end;  // particle names: the joint, the far end
    std::string parent;      // body name, "" = the root
    std::string sideA, sideB;  // particle names, "" = no side pair
    // + pitch swings the far end toward pitchDir, + yaw toward yawDir (model
    // space, bind pose); + roll turns pitchDir toward yawDir about the axis.
    float pitchDir[3] = {0, 0, 1};
    float yawDir[3] = {1, 0, 0};
    float pitch[2] = {-45, 45};
    float yaw[2] = {-30, 30};
    float roll[2] = {-30, 30};
    bool hinge = false;       // bends in pitch only (an elbow, a knee)
    // A motorized body's motors fade once the ragdoll's pelvis is down on the
    // ground (what carries the weight: the legs, the pelvis, the chest).
    bool groundFade = false;
};

struct ProfileJoint {
    std::string body;
    bool relative = false;
    float ratio = 0.0f;
};

struct Profile {
    std::string name;
    std::vector<ProfileJoint> joints;  // bodies not listed: ratio 0 (loose)
    const ProfileJoint* find(const std::string& body) const;
};

// The simulation's settings - Gunner's tuning (inc/gunner/config.hpp) as the
// defaults, so the preview falls the way the game does.
struct Solver {
    float gravity = 9.81f;          // m/s^2
    float damping = 0.995f;         // velocity kept a step
    int iterations = 5;             // constraint passes a step
    float rolling = 0.35f;          // speed lost a step on the ground
    float staticFriction = 0.8f;    // Coulomb, against the ground's push back
    float kineticFriction = 0.6f;
    float contactSlop = 0.01f;      // m: "on the ground"
    float limitStiffness = 1.0f;
    float jointDamping = 0.1f;
    float rollBendMin = 10.0f;      // deg: a hinge's parent's roll follows...
    float rollBendFull = 25.0f;     // ...its bend plane from here, fully
    float sleepSpeed = 0.1f;        // m/s
    float sleepTime = 0.4f;         // s
    float maxAwake = 6.0f;          // s
    float pushRadius = 0.35f;       // m: a shot's push falls off over it
    float pushTopple = 0.5f;        // share at the top of the body
    float pushBody = 0.1f;          // share on every particle
    float motorGain = 0.2f;         // gap closed a step at ratio 1
    float motorVelocity = 0.6f;     // share of a correction kept as speed
    float motorLying = 0.25f;       // groundFade motors' strength once down
    float motorFadeTime = 0.5f;     // s
    float lyingSlop = 0.05f;        // m
    float boxStepUp = 0.05f;        // m
    float fixedDt = 1.0f / 60.0f;   // s
};

struct Asset {
    std::string skeleton;   // project-relative .skeleton
    std::string header;     // project-relative generated header ("" = none)
    std::string nameSpace;  // the header's namespace, "A::B" form
    std::vector<Particle> particles;
    std::vector<Body> bodies;  // parents before children
    std::vector<std::vector<std::string>> clusters;
    std::vector<Profile> profiles;
    Solver solver;

    int particle(const std::string& name) const;
    int body(const std::string& name) const;
    const Profile* profile(const std::string& name) const;
};

std::string assetJson(const Asset& a);
bool parseAsset(const std::string& text, Asset& out, std::string& error);

// ----------------------------------------------------- the resolved ragdoll --
// Index-based, validated, everything derived: what the solver steps and what
// the header is written from.
struct Def {
    struct P {
        std::string name;
        int bone = -1;
        float offset[3] = {0, 0, 0};
        float bind[3] = {0, 0, 0};  // model space, bind pose
        float radius = 0.05f, mass = 1.0f;
        bool world = false;
    };
    struct B {
        std::string name;
        int a = -1, b = -1, parent = -1;
        int q = -1;  // the parent's end farther from `a` (a limit's reaction)
        int sideA = -1, sideB = -1;
        float pitchDir[3] = {0, 0, 1}, yawDir[3] = {1, 0, 0};
        float pitchMin = 0, pitchMax = 0, yawMin = 0, yawMax = 0, rollMin = 0,
              rollMax = 0;
        bool hinge = false, groundFade = false;
    };
    struct Prof {
        std::string name;
        std::vector<bool> relative;  // per body
        std::vector<float> ratio;    // per body
    };
    std::vector<P> particles;
    std::vector<B> bodies;
    std::vector<std::pair<int, int>> links;  // clusters' pairs, then bodies
    std::vector<int> boneBody;  // per skeleton bone: the body it follows, -1
    std::vector<float> boneBind;  // 16 a skeleton bone: its bind, rotation unscaled
    std::vector<Prof> profiles;
    Solver solver;
};

// Resolves names, checks the structure (one root first, parents before
// children, the root has a side pair, a body's particles distinct, every bone
// known) and derives links, q and the bone -> body table. Problems go to
// `errors`; false when the ragdoll cannot be simulated.
bool resolve(const Asset& a, const Skeleton& s, Def& out,
             std::vector<std::string>& errors);

// The C++ data header a game consumes (the layout Gunner's
// combatent_ragdoll.hpp has: Particle / Body / Bone tables, plus the motor
// profiles and each body's groundFade).
std::string headerText(const Asset& a, const Skeleton& s, const Def& d,
                       const std::string& assetRel);
// Its first line - how the build knows the file is one to keep in sync.
inline constexpr const char* kHeaderMarker =
    "// Generated by TyraX. Do not edit - regenerated on every build.\n// The ragdoll ";

// -------------------------------------------------------------- generation --
// Unreal's "Generate All Bodies" for this representation: a body on every
// skin joint whose segment is at least `minBoneSize` of the model's height
// (shorter ones ride their parent's body), capsule radii and masses from the
// skinned vertices, side pairs and a bracing particle on the bodies with two
// lateral children, hinges and limits from the bone names (knees, elbows,
// spine, neck, ...), and two starter profiles. The result simulates; tuning it
// is the editor's job.
struct GenOptions {
    float minBoneSize = 0.05f;  // fraction of the model's height
    float totalMass = 75.0f;    // kg over the whole body
};
bool generate(const glbparser::Skel& skel, const Skeleton& s,
              const GenOptions& opt, Asset& out, std::string& report);
// One body's starting joint the way generate() sets it: limits, hinge and
// groundFade from the bone's name (knee, elbow, spine, ...), pitchDir /
// yawDir from its segment (pivot -> end, model space, bind pose).
void defaultJoint(const std::string& boneName, bool root, const float pivot[3],
                  const float end[3], Body& b);

// Project-relative sibling paths: "res/models/x.glb" -> "res/models/x.skeleton"
std::string skeletonPathFor(const std::string& modelRel);
std::string ragdollPathFor(const std::string& modelRel);

// ------------------------------------------------------------------- files --
bool loadSkeleton(const std::string& absPath, Skeleton& out, std::string& error);
bool loadAsset(const std::string& absPath, Asset& out, std::string& error);
// Writes text only when it differs (an unchanged asset keeps its mtime).
bool saveText(const std::string& absPath, const std::string& text, std::string& error);

// The skeleton asset of a parsed model, written beside it when missing - or
// rewritten when its bones changed (a re-exported model). False (and nothing
// written) for a model without a skin. `note` says what happened.
bool ensureSkeleton(const std::string& projectDir, const std::string& modelRel,
                    const glbparser::Skel& skel, std::string& note);
// ...and a generated ragdoll beside it when there is none (never over an
// existing one: once made, the ragdoll is the Ragdoll Editor's to tune).
bool ensureRagdoll(const std::string& projectDir, const std::string& modelRel,
                   const glbparser::Skel& skel, const GenOptions& opt, std::string& note);

// The generated headers of every ragdoll in the project that names one (a
// build's codegen): {project-relative '\\'-separated path, content}. A broken
// asset is reported in `warnings` and skipped.
std::vector<std::pair<std::string, std::string>> projectHeaders(
    const std::string& projectDir, std::vector<std::string>& warnings);

// `tyrax-editor --ragdoll ...` (docs/ragdoll-editor.md, "Command line").
int cliMain(int argc, char** argv);

}  // namespace ragdoll
