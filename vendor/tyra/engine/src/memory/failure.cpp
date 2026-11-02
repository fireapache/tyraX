/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# TyraX
#-----------------------------------------------------------------------
*/

// Added by TyraX: the memory manager's failure path (inc/memory/failure.hpp).
// Portable C++ - no PS2 header - so the host verifiers run it as it is.

#include "memory/failure.hpp"

#include <cstdio>
#include <cstdlib>

namespace Tyra {
namespace Memory {
namespace {

// The whole line lives on the stack: a failure is often an allocation that
// could not be served, so reporting it may not allocate. The texts are capped so
// the numbers at the end always fit.
constexpr size_t kLineBytes = 256;
constexpr size_t kMaxWhat = 80;
constexpr size_t kMaxName = 48;

void (*g_print)(const char*) = nullptr;
void (*g_halt)() = nullptr;
void (*g_handler)(const Failure&) = nullptr;
bool g_failing = false;

void defaultPrint(const char* line) {
  std::fputs(line, stderr);
  std::fputc('\n', stderr);
}

void defaultHalt() { std::abort(); }

class Line {
 public:
  void text(const char* s, size_t cap) {
    if (s == nullptr) s = "?";
    for (size_t i = 0; i < cap && s[i] != '\0'; ++i) {
      // One failure is one line, whatever the text holds.
      put(static_cast<unsigned char>(s[i]) < ' ' ? ' ' : s[i]);
    }
  }

  void decimal(uint64_t value) {
    char digits[20];
    size_t n = 0;
    do {
      digits[n++] = static_cast<char>('0' + value % 10);
      value /= 10;
    } while (value != 0);
    while (n > 0) put(digits[--n]);
  }

  void hex(uint64_t value) {
    char digits[16];
    size_t n = 0;
    do {
      digits[n++] = "0123456789abcdef"[value & 15];
      value >>= 4;
    } while (value != 0);
    while (n > 0) put(digits[--n]);
  }

  const char* c_str() {
    buffer_[length_] = '\0';
    return buffer_;
  }

 private:
  void put(char c) {
    if (length_ < kLineBytes - 1) buffer_[length_++] = c;
  }

  char buffer_[kLineBytes];
  size_t length_ = 0;
};

// Marks that a failure is being handled; a second one raised meanwhile sees it.
class Reentry {
 public:
  Reentry() : nested_(g_failing) { g_failing = true; }
  ~Reentry() { g_failing = nested_; }
  bool nested() const { return nested_; }

 private:
  bool nested_;
};

}  // namespace

void setFailureOutput(void (*print)(const char* line), void (*halt)()) {
  g_print = print;
  g_halt = halt;
}

void setFailureHandler(void (*handler)(const Failure&)) { g_handler = handler; }

void fail(const Failure& f) {
  Reentry reentry;

  Line line;
  line.text("[MEM] FAIL ", kLineBytes);
  line.text(f.what != nullptr ? f.what : "?", kMaxWhat);
  line.text(" name=", kLineBytes);
  line.text(f.name != nullptr ? f.name : "-", kMaxName);
  line.text(" size=", kLineBytes);
  line.decimal(static_cast<uint64_t>(f.size));
  line.text(" caller=0x", kLineBytes);
  line.hex(static_cast<uint64_t>(f.caller));

  (g_print != nullptr ? g_print : defaultPrint)(line.c_str());
  if (!reentry.nested() && g_handler != nullptr) g_handler(f);
  (g_halt != nullptr ? g_halt : defaultHalt)();
}

}  // namespace Memory
}  // namespace Tyra
