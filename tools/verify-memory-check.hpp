// Added by TyraX: the shared pieces of the memory-manager host verifiers
// (tools/verify-memory-*.cpp, run by tools/verify-memory.py).
//
// CHECK counts and keeps going, so one run reports every broken expectation.
// finish() turns the counts into the exit code. installThrowingFailure() makes
// Memory::fail testable: the failure line is captured and the halt throws.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "memory/failure.hpp"

// The verifiers read as the engine code does.
using namespace Tyra::Memory;

namespace verify_detail {

inline int checks = 0;
inline int failures = 0;

}  // namespace verify_detail

#define CHECK(cond)                                                          \
  do {                                                                       \
    ++verify_detail::checks;                                                 \
    if (!(cond)) {                                                           \
      ++verify_detail::failures;                                             \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, \
                   #cond);                                                   \
    }                                                                        \
  } while (0)

// Prints "ALL <n> CHECKS PASSED", or how many of the n checks failed, and
// returns the process exit code (0 only when nothing failed).
inline int finish(const char* name) {
  if (verify_detail::failures == 0) {
    std::printf("%s: ALL %d CHECKS PASSED\n", name, verify_detail::checks);
    return 0;
  }
  std::printf("%s: %d of %d CHECKS FAILED\n", name, verify_detail::failures,
              verify_detail::checks);
  return 1;
}

// What the throwing halt throws: the failure fail() was called with.
struct ThrownFailure {
  Failure f;
};

namespace verify_detail {

inline std::vector<std::string>& lines() {
  static std::vector<std::string> v;
  return v;
}

inline Failure& lastFailure() {
  static Failure f = {"", "", 0, 0};
  return f;
}

inline void capturePrint(const char* line) { lines().push_back(line); }
inline void captureHandler(const Failure& f) { lastFailure() = f; }
inline void throwingHalt() { throw ThrownFailure{lastFailure()}; }

}  // namespace verify_detail

// The failure lines printed since the last installThrowingFailure().
inline std::vector<std::string>& capturedFailureLines() {
  return verify_detail::lines();
}

// Output goes to capturedFailureLines(); the halt throws ThrownFailure with the
// failure the handler last saw. Installs the failure handler to do that, so a
// verifier of its own handler sets it after this call.
inline void installThrowingFailure() {
  verify_detail::lines().clear();
  verify_detail::lastFailure() = Failure{"", "", 0, 0};
  setFailureOutput(&verify_detail::capturePrint, &verify_detail::throwingHalt);
  setFailureHandler(&verify_detail::captureHandler);
}

// True when f() fails through Memory::fail with a `what` containing the text.
template <class F>
bool throwsFailure(F f, const char* whatContains) {
  try {
    f();
  } catch (const ThrownFailure& t) {
    return t.f.what != nullptr && std::strstr(t.f.what, whatContains) != nullptr;
  }
  return false;
}

// True when all n bytes at p equal value.
inline bool allBytes(const void* p, size_t n, unsigned char value) {
  const unsigned char* b = static_cast<const unsigned char*>(p);
  for (size_t i = 0; i < n; ++i) {
    if (b[i] != value) return false;
  }
  return true;
}
