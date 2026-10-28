// The runtime half of tools/verify-object-scripts.cpp: user scripts and a frame
// loop compiled against the GENERATED inc/scripts/script.hpp and
// src/gen/object_scripts.gen.cpp of the harness's "osverify" project, with the
// stub engine in stub/tyra. The harness builds and runs it; it prints
// "ALL n CHECKS PASSED" and exits 0, or the failed expectations and exits 1.
//
// The project it expects (the harness writes it):
//   scene 0 "main"   o0 Plain, Late | o1 Marker{params}, Early | o2 Plain, Ghost, Quiet | o3 Plain
//   scene 1 "second" o0..o3 Plain | o4 Early
// Ghost is attached but never registered; Quiet is start-only by constant.
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>

#include "scripts/script.hpp"

// --- allocation counting: nothing in a frame or a scene switch may allocate ---
static unsigned long g_allocs = 0;
void* operator new(std::size_t n) {
  ++g_allocs;
  if (void* p = std::malloc(n ? n : 1)) return p;
  throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

// --- the event log: a fixed buffer, so logging itself never allocates ---
static char g_log[16384];
static std::size_t g_logLen = 0;
void tyraLogAppend(const char* text) {
  const std::size_t n = std::strlen(text);
  if (g_logLen + n + 1 >= sizeof(g_log)) return;
  std::memcpy(g_log + g_logLen, text, n + 1);
  g_logLen += n;
}
static void ev(const char* fmt, ...) {
  char line[160];
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  tyraLogAppend(line);
}
static void clearLog() {
  g_log[0] = 0;
  g_logLen = 0;
}

static int g_checks = 0;
static int g_failures = 0;
#define CHECK(cond, ...)                                       \
  do {                                                         \
    ++g_checks;                                                \
    if (!(cond)) {                                             \
      ++g_failures;                                            \
      std::printf("FAIL scenario.cpp:%d: ", __LINE__);         \
      std::printf(__VA_ARGS__);                                \
      std::printf("\n");                                       \
    }                                                          \
  } while (0)
static void expectLog(const char* want, const char* what) {
  ++g_checks;
  if (std::strcmp(g_log, want) != 0) {
    ++g_failures;
    std::printf("FAIL %s\n  want: %s\n  got:  %s\n", what, want, g_log);
  }
  clearLog();
}

namespace Osverify {

// Written the way scripts were written before parameters, order and start-only
// existed: only the hooks, `self` and the constructor/destructor.
class Traced : public ObjectScript {
 public:
  Traced() { ev("C "); }
  ~Traced() override { ev("D:%d ", selfIndex); }
  virtual const char* tag() const = 0;
  void onStart(ScriptContext&) override { ev("S:%s#%d ", tag(), selfIndex); }
  void onUpdate(ScriptContext&) override { ev("U:%s#%d ", tag(), selfIndex); }
  void onUsed(ScriptContext&) override { ev("Used:%s#%d ", tag(), selfIndex); }
  void onDestroy(ScriptContext&) override {
    ev("X:%s#%d(%s) ", tag(), selfIndex, self ? "self" : "null");
  }
};

static const ObjectScript* g_plainByScene[2] = {nullptr, nullptr};
static int g_plainScene = 0;

class Plain : public Traced {
 public:
  const char* tag() const override { return "Plain"; }
  void onStart(ScriptContext& ctx) override {
    Traced::onStart(ctx);
    if (selfIndex == 0) g_plainByScene[g_plainScene] = this;
    // A bare-name attachment (the pre-parameter form) has no parameters: every
    // read is its fallback.
    CHECK(param<int>("team", 9) == 9, "no parameters: the fallback");
    CHECK(!hasParam("team"), "no parameters: hasParam is false");
    CHECK(std::strcmp(param<const char*>("tag", nullptr), "") == 0,
          "a null string fallback reads as empty");
  }
};
TYRA_OBJECT_SCRIPT(Plain);

class Early : public Traced {
 public:
  static constexpr int kScriptOrder = -10;
  const char* tag() const override { return "Early"; }
};
TYRA_OBJECT_SCRIPT(Early);

class Late : public Traced {
 public:
  static constexpr int kScriptOrder = 10;
  const char* tag() const override { return "Late"; }
};
TYRA_OBJECT_SCRIPT(Late);

// Start-only by constant - and it overrides onUpdate and onUsed anyway, which a
// ticked class would have called.
class Quiet : public Traced {
 public:
  static constexpr bool kStartOnly = true;
  const char* tag() const override { return "Quiet"; }
};
TYRA_OBJECT_SCRIPT(Quiet);

// What the Marker read from its parameters.
struct Seen {
  int team = -1;
  float gain = -1.0F;
  double gainD = -1.0;
  bool hq = false;
  const char* tag = nullptr;
  float ratio = -1.0F;
  int fallback = 0;
  int gainAsInt = -1;
  bool teamAsBool = false;
  bool gainAsBool = false;
  int tagAsInt = -2;
  float fallbackF = 0.0F;
  const char* fallbackS = nullptr;
  bool hasTeam = false;
  bool hasNope = true;
  unsigned teamU = 0;
  bool startedOnce = false;
};
static Seen g_seen;

// A MarkerScript: start-only by its base, and it cannot override the tick hooks.
class Marker : public MarkerScript {
 public:
  Marker() { ev("C "); }
  ~Marker() override { ev("D:%d ", selfIndex); }
  void onStart(ScriptContext&) override {
    ev("S:Marker#%d ", selfIndex);
    g_seen.team = param<int>("team");
    g_seen.gain = param<float>("gain");
    g_seen.gainD = param<double>("gain");
    g_seen.hq = param<bool>("hq");
    g_seen.tag = param<const char*>("tag");
    g_seen.ratio = paramFloat("ratio");
    g_seen.fallback = param<int>("nope", 7);
    g_seen.gainAsInt = paramInt("gain", -1);
    g_seen.teamAsBool = paramBool("team");
    g_seen.gainAsBool = paramBool("gain");
    g_seen.tagAsInt = paramInt("tag", -1);
    g_seen.fallbackF = param<float>("nope", 2.5F);
    g_seen.fallbackS = param<const char*>("nope", "dflt");
    g_seen.hasTeam = hasParam("team");
    g_seen.hasNope = hasParam("nope");
    g_seen.teamU = param<unsigned>("team");
    g_seen.startedOnce = true;
  }
  void onDestroy(ScriptContext&) override { ev("X:Marker#%d(%s) ", selfIndex, self ? "self" : "null"); }
};
TYRA_OBJECT_SCRIPT(Marker);

// Registered and attached nowhere: it gets no storage and costs nothing.
class Unattached : public Traced {
 public:
  const char* tag() const override { return "Unattached"; }
};
TYRA_OBJECT_SCRIPT(Unattached);

// Global scripts: where each runs among the others (and the driver) is the
// subject of the order checks.
class Probe : public Script {
 public:
  Probe(const char* name, int order) : name_(name) { scriptOrder = order; }
  void update(ScriptContext&) override { ev("G:%s ", name_); }

 private:
  const char* name_;
};

// A script that is ALSO something else (Gunner's main-menu controller is an
// ObjectScript and a FrontEnd): the storage holds the whole object, and
// destroying it through the ObjectScript pointer runs the full destructor chain
// whichever base comes first.
static int g_ifaceDtors = 0;
static int g_multiDtors = 0;
struct Iface {
  virtual ~Iface() { ++g_ifaceDtors; }
  virtual int tag() const { return 7; }
  long long pad[3] = {0, 0, 0};
};
class MultiA : public ObjectScript, public Iface {
 public:
  ~MultiA() override { ++g_multiDtors; }
};
class MultiB : public Iface, public ObjectScript {
 public:
  ~MultiB() override { ++g_multiDtors; }
};

}  // namespace Osverify

using namespace Osverify;

int main() {
  // The capacity table is a compile-time fact: the busiest scene attaches four
  // Plain, one of everything else; a class nothing attaches gets none.
  static_assert(objectScriptCapacity("Plain") == 4, "Plain: 3 in scene 0, 4 in scene 1");
  static_assert(objectScriptCapacity("Early") == 1, "Early");
  static_assert(objectScriptCapacity("Marker") == 1, "Marker");
  static_assert(objectScriptCapacity("Unattached") == 0, "registered, attached nowhere");

  ScriptContext ctx;
  RuntimeObject objs[8];
  ctx.objects = objs;
  ctx.objectCount = 4;
  ctx.scene = 0;
  ctx.sceneGeneration = 1;

  // The driver registered at static-initialisation time. Register the others
  // now, out of order: the list sorts them.
  Probe mid("A", 0), high("B", 2000), low("C", -5);
  getScripts().push_back(&mid);
  getScripts().push_back(&high);
  getScripts().push_back(&low);
  CHECK(getScripts().size() == 4, "four global scripts, got %zu", getScripts().size());
  CHECK(getScripts()[0] == &low && getScripts()[1] == &mid && getScripts()[3] == &high,
        "global scripts sort by scriptOrder");

  auto frame = [&]() {
    for (Script* s : getScripts()) s->update(ctx);  // what the game loop does
  };

  // --- scene 0, first frame: start order, then the tick ---------------------
  frame();
  // Ghost is attached and not registered: skipped with a log line, before any start.
  expectLog("G:C G:A Object script not registered: Ghost\n"
            "C S:Early#1 C S:Plain#0 C S:Marker#1 C S:Plain#2 C S:Quiet#2 C S:Plain#3 C S:Late#0 "
            "U:Early#1 U:Plain#0 U:Plain#2 U:Plain#3 U:Late#0 G:B ",
            "first frame: globals by order, the driver after the default ones, starts by class "
            "order then attachment order, then one tick");
  const unsigned long allocsAfterFirst = g_allocs;

  // --- the parameters --------------------------------------------------------
  CHECK(g_seen.startedOnce, "the marker started");
  CHECK(g_seen.team == 1, "param<int> team: %d", g_seen.team);
  CHECK(g_seen.gain == 0.5F, "param<float> gain: %f", (double)g_seen.gain);
  CHECK(g_seen.gainD == 0.5, "param<double> gain: %f", g_seen.gainD);
  CHECK(g_seen.hq, "param<bool> hq");
  CHECK(g_seen.tag && std::strcmp(g_seen.tag, "north \"gate\"") == 0, "param<const char*> tag: %s",
        g_seen.tag ? g_seen.tag : "(null)");
  CHECK(g_seen.ratio == 3.0F, "an int param read as float: %f", (double)g_seen.ratio);
  CHECK(g_seen.fallback == 7, "a missing param is the fallback: %d", g_seen.fallback);
  CHECK(g_seen.fallbackF == 2.5F, "a missing float param is the fallback");
  CHECK(g_seen.fallbackS && std::strcmp(g_seen.fallbackS, "dflt") == 0, "a missing string is the fallback");
  CHECK(g_seen.gainAsInt == 0, "a float param read as int truncates: %d", g_seen.gainAsInt);
  CHECK(g_seen.teamAsBool, "a nonzero int reads as true");
  CHECK(g_seen.gainAsBool, "a nonzero float reads as true");
  CHECK(g_seen.tagAsInt == -1, "a string is not a number: %d", g_seen.tagAsInt);
  CHECK(g_seen.hasTeam && !g_seen.hasNope, "hasParam");
  CHECK(g_seen.teamU == 1U, "param<unsigned>");

  // --- an ordinary frame: class order, then attachment order ----------------------
  frame();
  expectLog("G:C G:A U:Early#1 U:Plain#0 U:Plain#2 U:Plain#3 U:Late#0 G:B ", "a plain frame");

  // --- an inactive object is not ticked --------------------------------------------
  objs[2].active = false;
  frame();
  expectLog("G:C G:A U:Early#1 U:Plain#0 U:Plain#3 U:Late#0 G:B ", "object 2 inactive: skipped");
  objs[2].active = true;
  frame();
  expectLog("G:C G:A U:Early#1 U:Plain#0 U:Plain#2 U:Plain#3 U:Late#0 G:B ", "object 2 active again");

  // --- USE reaches the ticked instances on that object only --------------------------
  ctx.usedObject = 3;
  frame();
  expectLog("G:C G:A U:Early#1 U:Plain#0 U:Plain#2 U:Plain#3 U:Late#0 Used:Plain#3 G:B ", "USE on 3");
  ctx.usedObject = 2;  // Plain and the start-only Quiet are on object 2
  frame();
  expectLog("G:C G:A U:Early#1 U:Plain#0 U:Plain#2 U:Plain#3 U:Late#0 Used:Plain#2 G:B ",
            "USE on 2: not the start-only Quiet");
  ctx.usedObject = 1;  // the Marker and Early are on object 1
  frame();
  expectLog("G:C G:A U:Early#1 U:Plain#0 U:Plain#2 U:Plain#3 U:Late#0 Used:Early#1 G:B ",
            "USE on 1: not the Marker");
  ctx.usedObject = -1;

  // --- a scene switch: onDestroy x all, destructors x all, then the new starts -----
  ctx.scene = 1;
  ctx.objectCount = 5;
  ctx.sceneGeneration = 2;
  g_plainScene = 1;
  frame();
  expectLog("G:C G:A "
            "X:Early#1(null) X:Plain#0(null) X:Marker#1(null) X:Plain#2(null) X:Quiet#2(null) "
            "X:Plain#3(null) X:Late#0(null) "
            "D:1 D:0 D:1 D:2 D:2 D:3 D:0 "
            "C S:Early#4 C S:Plain#0 C S:Plain#1 C S:Plain#2 C S:Plain#3 "
            "U:Early#4 U:Plain#0 U:Plain#1 U:Plain#2 U:Plain#3 G:B ",
            "scene switch");
  CHECK(g_plainByScene[0] && g_plainByScene[0] == g_plainByScene[1],
        "scene 1's first Plain lives where scene 0's did (static storage, reused)");

  // --- back to scene 0 -----------------------------------------------------------------
  ctx.scene = 0;
  ctx.objectCount = 4;
  ctx.sceneGeneration = 3;
  g_plainScene = 0;
  frame();
  expectLog("G:C G:A "
            "X:Early#4(null) X:Plain#0(null) X:Plain#1(null) X:Plain#2(null) X:Plain#3(null) "
            "D:4 D:0 D:1 D:2 D:3 "
            "Object script not registered: Ghost\n"
            "C S:Early#1 C S:Plain#0 C S:Marker#1 C S:Plain#2 C S:Quiet#2 C S:Plain#3 C S:Late#0 "
            "U:Early#1 U:Plain#0 U:Plain#2 U:Plain#3 U:Late#0 G:B ",
            "scene 0 again");
  CHECK(g_plainByScene[0] == g_plainByScene[1], "scene 0's Plain is where it was");

  // --- a frame and a scene reload allocate nothing -----------------------------------------
  CHECK(g_allocs == allocsAfterFirst, "allocations after the first frame: %lu",
        g_allocs - allocsAfterFirst);

  // --- the pool itself: Cap instances, then none, until released -------------------------
  clearLog();
  using Pool = ObjectScriptPool<Plain, 2>;
  ObjectScript* a = Pool::create();
  ObjectScript* b = Pool::create();
  CHECK(a && b && a != b, "a pool hands out its Cap instances");
  CHECK(Pool::create() == nullptr, "a full pool says so");
  a->~ObjectScript();
  b->~ObjectScript();
  Pool::release();
  ObjectScript* c = Pool::create();
  CHECK(c == a, "a released pool starts over at its first slot");
  c->~ObjectScript();
  Pool::release();
  clearLog();

  // --- multiple inheritance: placement-new + destruction through ObjectScript* ---------------
  {
    using PoolA = ObjectScriptPool<MultiA, 1>;
    using PoolB = ObjectScriptPool<MultiB, 1>;
    ObjectScript* ma = PoolA::create();
    ObjectScript* mb = PoolB::create();
    CHECK(ma && mb, "multiple-inheritance scripts construct in place");
    ma->~ObjectScript();
    mb->~ObjectScript();
    CHECK(g_multiDtors == 2 && g_ifaceDtors == 2, "full destructor chain: %d derived, %d base",
          g_multiDtors, g_ifaceDtors);
    PoolA::release();
    PoolB::release();
  }

  if (g_failures) {
    std::printf("%d of %d CHECKS FAILED\n", g_failures, g_checks);
    return 1;
  }
  std::printf("ALL %d CHECKS PASSED\n", g_checks);
  return 0;
}
