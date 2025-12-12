#pragma once

#include <vector>
#include <cstdint>
#include "openvr_driver.h"

namespace psvr2_toolkit {

class AdaptiveGazeSmoother {
public:
  AdaptiveGazeSmoother();

  vr::HmdVector3_t Process(const vr::HmdVector3_t& rawDir, int64_t timestampUs);

  float GetSmoothingAmount() const { return m_smoothingAmount; }

private:
  float AngleBetween(const vr::HmdVector3_t& a, const vr::HmdVector3_t& b) const;
  vr::HmdVector3_t Normalize(const vr::HmdVector3_t& v) const;

  std::vector<float> m_recentAngles;
  size_t m_recentIndex;

  vr::HmdVector3_t m_lastSmoothed;
  vr::HmdVector3_t m_lastRaw;
  int64_t m_lastTimestampUs;

  float m_smoothingAmount;

  float m_jitterEma;

  int m_lowJitterCount;

  static constexpr size_t kWindowSize = 8;
  const float kHighThresholdDeg = 0.3f;
  const float kLowThresholdDeg = 0.08f;
  const float kMaxSmoothing = 0.92f;
  const float kMinSmoothing = 0.0f;
  const float kIncreasePerSecond = 0.6f;
  const float kDecreasePerSecond = 0.15f;
  const float kJitterEmaAlpha = 0.35f;
  const int kRequiredLowFrames = 20;
  const float kMaxDtSec = 0.05f;
  const float kDecreaseHysteresis = 0.5f;
};

} // namespace psvr2_toolkit
