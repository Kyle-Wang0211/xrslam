#include "CommonCoreOwner.h"
#include <limits>
#include <mutex>
namespace pw_common_core {
namespace {
std::mutex mutex;
Token generation = 0;
Token active = 0;
Mode active_mode = Mode::Live;
}
Token TryClaim(Mode mode) {
  std::lock_guard<std::mutex> lock(mutex);
  // Fail closed on epoch exhaustion rather than alias an ancient token.
  if (active || generation == std::numeric_limits<Token>::max()) return 0;
  active_mode = mode;
  active = ++generation;
  return active;
}
bool Owns(Mode mode, Token token) {
  std::lock_guard<std::mutex> lock(mutex);
  return token && active == token && active_mode == mode;
}
bool Release(Mode mode, Token token) {
  std::lock_guard<std::mutex> lock(mutex);
  if (!token || active != token || active_mode != mode) return false;
  active = 0;
  return true;
}
}
