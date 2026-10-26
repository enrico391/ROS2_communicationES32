#pragma once

#include <cmath>

// Single-position offset calibration: stationary, level, sensor Z upward.
// This assumes unit sensitivity and does not estimate mounting tilt or gain.
class ImuCalibration {
public:
  static constexpr unsigned SAMPLE_COUNT = 250;
  static constexpr float GRAVITY = 9.80665f;

  bool add(const float acceleration[3], const float gyro[3]) {
    for (unsigned axis = 0; axis < 3; ++axis) {
      if (!std::isfinite(acceleration[axis]) || !std::isfinite(gyro[axis]) ||
          std::abs(gyro[axis]) > 0.15f) {
        rejected_ = true;
        return false;
      }
    }
    ++count_;
    for (unsigned n = 0; n < 6; ++n) {
      const double value = n < 3 ? acceleration[n] : gyro[n - 3];
      const double delta = value - mean_[n];
      mean_[n] += delta / count_;
      m2_[n] += delta * (value - mean_[n]);
    }
    return true;
  }

  bool finish() {
    if (rejected_ || count_ < SAMPLE_COUNT) return false;
    // Maximum per-axis standard deviation: 0.15 m/s² accel, 0.01 rad/s gyro.
    for (unsigned n = 0; n < 6; ++n) {
      const double limit = n < 3 ? 0.15 : 0.01;
      if (m2_[n] / (count_ - 1) > limit * limit) return false;
    }
    const double magnitude = std::sqrt(mean_[0] * mean_[0] + mean_[1] * mean_[1] +
                                       mean_[2] * mean_[2]);
    // Allow the reported 14% error, but reject clearly wrong poses/readings.
    if (magnitude < 0.75 * GRAVITY || magnitude > 1.25 * GRAVITY ||
        std::abs(mean_[0]) > 2.0 || std::abs(mean_[1]) > 2.0 ||
        mean_[2] < 0.75 * GRAVITY) return false;
    for (unsigned axis = 0; axis < 3; ++axis) {
      acceleration_offset_[axis] = float(mean_[axis]) - (axis == 2 ? GRAVITY : 0);
      gyro_offset_[axis] = float(mean_[axis + 3]);
    }
    calibrated_ = true;
    return true;
  }

  void apply(float acceleration[3], float gyro[3]) const {
    if (!calibrated_) return;
    for (unsigned axis = 0; axis < 3; ++axis) {
      acceleration[axis] -= acceleration_offset_[axis];
      gyro[axis] -= gyro_offset_[axis];
    }
  }

private:
  unsigned count_ = 0;
  double mean_[6] = {};
  double m2_[6] = {};
  float acceleration_offset_[3] = {};
  float gyro_offset_[3] = {};
  bool rejected_ = false;
  bool calibrated_ = false;
};
