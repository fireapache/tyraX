// Added by TyraX: host verifier for the failure path (inc/memory/failure.hpp,
// src/memory/failure.cpp). Built and run by tools/verify-memory.py, once with
// TYRA_MEMORY_CHECKS=0 and once with =1.

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "verify-memory-check.hpp"

namespace {

// ---- hooks that record the order things happen in ------------------------

std::string trace;
std::vector<std::string> printed;
Failure handled = {"", "", 0, 0};
int handlerCalls = 0;

void tracePrint(const char* line) {
  trace += "P";
  printed.push_back(line);
}
void traceHandler(const Failure& f) {
  trace += "H";
  handled = f;
  ++handlerCalls;
}
struct Halted {};
void traceHalt() {
  trace += "X";
  throw Halted();
}

void reset() {
  trace.clear();
  printed.clear();
  handled = Failure{"", "", 0, 0};
  handlerCalls = 0;
  setFailureOutput(&tracePrint, &traceHalt);
  setFailureHandler(&traceHandler);
}

// Runs fail() and swallows the halt; true when the halt hook ran.
bool failAndCatch(const Failure& f) {
  try {
    fail(f);
  } catch (const Halted&) {
    return true;
  }
  return false;
}

void testLineFormat() {
  reset();
  CHECK(failAndCatch({"budget full", "Renderer", 12345, 0xBEEF}));
  CHECK(printed.size() == 1);
  CHECK(printed[0] == "[MEM] FAIL budget full name=Renderer size=12345 caller=0xbeef");

  reset();
  CHECK(failAndCatch({"scratch full", "frame", 0, 0}));
  CHECK(printed[0] == "[MEM] FAIL scratch full name=frame size=0 caller=0x0");

  reset();
  CHECK(failAndCatch({"pool exhausted", "bullets", SIZE_MAX, UINTPTR_MAX}));
  CHECK(printed[0] == "[MEM] FAIL pool exhausted name=bullets size=" +
                          std::to_string(SIZE_MAX) + " caller=0x" +
                          (sizeof(uintptr_t) == 8 ? "ffffffffffffffff" : "ffffffff"));

  // a missing text does not crash the writer
  reset();
  CHECK(failAndCatch({nullptr, nullptr, 7, 0x10}));
  CHECK(printed.size() == 1 && printed[0].compare(0, 11, "[MEM] FAIL ") == 0);
  CHECK(printed[0].find("size=7 caller=0x10") != std::string::npos);

  // an over-long text is cut, and the numbers still arrive
  reset();
  const std::string longWhat(1000, 'w'), longName(1000, 'n');
  CHECK(failAndCatch({longWhat.c_str(), longName.c_str(), 99, 0xABC}));
  CHECK(printed[0].size() < 256);
  CHECK(printed[0].compare(0, 11, "[MEM] FAIL ") == 0);
  CHECK(printed[0].find(" size=99 caller=0xabc") != std::string::npos);
  CHECK(printed[0].size() >= 11 + 80 + 6 + 48);  // texts kept up to their caps
}

void testOrder() {
  // print, then the game's handler, then halt
  reset();
  CHECK(failAndCatch({"ptr table full", "ptrs", 64, 0x1234}));
  CHECK(trace == "PHX");
  CHECK(handlerCalls == 1);
  CHECK(std::strcmp(handled.what, "ptr table full") == 0);
  CHECK(std::strcmp(handled.name, "ptrs") == 0);
  CHECK(handled.size == 64 && handled.caller == 0x1234);

  // no handler: still printed, still halted
  reset();
  setFailureHandler(nullptr);
  CHECK(failAndCatch({"registry does not fit", "", 1, 2}));
  CHECK(trace == "PX" && handlerCalls == 0);

  // the hooks stay installed for every later failure
  reset();
  CHECK(failAndCatch({"a", "b", 3, 4}));
  CHECK(failAndCatch({"c", "d", 5, 6}));
  CHECK(printed.size() == 2 && handlerCalls == 2);
}

// A failure while the failure is being handled (the handler allocates and the
// budget is full) must not recurse: the second goes to print and halt only.
void testReentry() {
  reset();
  static int depth = 0;
  depth = 0;
  setFailureHandler([](const Failure& f) {
    ++handlerCalls;
    handled = f;
    if (depth++ == 0) fail({"nested", "inner", 1, 2});
  });
  CHECK(failAndCatch({"outer", "first", 3, 4}));
  CHECK(handlerCalls == 1);  // the nested failure skipped the handler
  CHECK(printed.size() == 2);
  CHECK(printed[0].find("FAIL outer") != std::string::npos);
  CHECK(printed[1].find("FAIL nested") != std::string::npos);

  // and the guard is released again afterwards
  reset();
  CHECK(failAndCatch({"later", "ok", 5, 6}));
  CHECK(trace == "PHX");
}

void testThrowingHelpers() {
  installThrowingFailure();
  CHECK(throwsFailure([] { fail({"pool exhausted", "bullets", 24, 0x99}); }, "exhausted"));
  CHECK(!throwsFailure([] { fail({"pool exhausted", "bullets", 24, 0x99}); }, "full"));
  CHECK(!throwsFailure([] {}, "anything"));
  CHECK(capturedFailureLines().size() == 2);  // the third call never failed
  try {
    fail({"scratch full", "frame", 70000, 0x40});
  } catch (const ThrownFailure& t) {
    CHECK(std::strcmp(t.f.what, "scratch full") == 0);
    CHECK(std::strcmp(t.f.name, "frame") == 0);
    CHECK(t.f.size == 70000 && t.f.caller == 0x40);
  }
  CHECK(capturedFailureLines().back() ==
        "[MEM] FAIL scratch full name=frame size=70000 caller=0x40");
}

}  // namespace

int main() {
  testLineFormat();
  testOrder();
  testReentry();
  testThrowingHelpers();
  return finish("failure");
}
