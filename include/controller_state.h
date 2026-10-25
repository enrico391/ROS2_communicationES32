#pragma once

#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstdlib>

struct MotorCommand {
  float left = 0;
  float right = 0;
  uint32_t received_ms = 0;
};

// Assign only after validating the entire command. Bad input cannot feed the watchdog.
inline bool parseMotorCommand(const char *text, uint32_t received_ms,
                              MotorCommand &command) {
  if (text[0] != 'm' || text[1] != ' ') return false;
  char *end;
  const float left = std::strtof(text + 2, &end);
  if (end == text + 2 || !std::isfinite(left) ||
      !std::isspace(static_cast<unsigned char>(*end))) return false;
  const char *right_start = end;
  const float right = std::strtof(right_start, &end);
  if (end == right_start || !std::isfinite(right)) return false;
  while (std::isspace(static_cast<unsigned char>(*end))) ++end;
  if (*end != '\0') return false;
  command.left = left;
  command.right = right;
  command.received_ms = received_ms;
  return true;
}

class MotorCommandState {
public:
  void accept(const MotorCommand &command) {
    command_ = command;
    received_ = true;
  }

  bool expired(uint32_t now_ms) {
    // Latch expiry until accept(); a later full clock wrap must not revive motion.
    if (received_ && uint32_t(now_ms - command_.received_ms) >= 500) received_ = false;
    return !received_;
  }

  MotorCommand current(uint32_t now_ms) {
    return expired(now_ms) ? MotorCommand{} : command_;
  }

private:
  MotorCommand command_{};
  bool received_ = false;
};
