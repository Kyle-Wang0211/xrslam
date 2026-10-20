#pragma once
#include <cstdint>

// Lifecycle adaptation only. Both wrappers link this one implementation.
// A reservation excludes the other mode before any official Create call and
// remains held through replay/worker quiescence and the official Destroy call.
namespace pw_common_core {
enum class Mode : uint8_t { Live = 1, Bench = 2 };
using Token = uint64_t;
Token TryClaim(Mode mode);
bool Owns(Mode mode, Token token);
bool Release(Mode mode, Token token);
class Reservation final {
public:
  explicit Reservation(Mode mode) : mode_(mode), token_(TryClaim(mode)) {}
  ~Reservation() { if (token_) Release(mode_, token_); }
  Reservation(const Reservation&) = delete;
  Reservation& operator=(const Reservation&) = delete;
  Token token() const { return token_; }
  Token Detach() { const auto token = token_; token_ = 0; return token; }
private:
  Mode mode_;
  Token token_;
};
}
