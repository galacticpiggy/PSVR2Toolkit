#include "adaptive_gaze_smoother.h"
#include <cmath>
#include <algorithm>

namespace psvr2_toolkit {

static float Dot(const vr::HmdVector3_t& a, const vr::HmdVector3_t& b) {
  return a.v[0]*b.v[0] + a.v[1]*b.v[1] + a.v[2]*b.v[2];
}

static float Length(const vr::HmdVector3_t& a) {
  return std::sqrt(Dot(a,a));
}

template<typename T>
static T clamp_val(T v, T lo, T hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

AdaptiveGazeSmoother::AdaptiveGazeSmoother()
  : m_recentAngles(kWindowSize, 0.0f), m_recentIndex(0), m_lastSmoothed{0,0,1}, m_lastRaw{0,0,1}, m_lastTimestampUs(0), m_smoothingAmount(0.3f), m_jitterEma(0.0f), m_lowJitterCount(0)
{
}

vr::HmdVector3_t AdaptiveGazeSmoother::Normalize(const vr::HmdVector3_t& v) const {
  float len = Length(v);
  if (len <= 1e-6f) return vr::HmdVector3_t{0,0,1};
  return vr::HmdVector3_t{v.v[0]/len, v.v[1]/len, v.v[2]/len};
}

float AdaptiveGazeSmoother::AngleBetween(const vr::HmdVector3_t& a, const vr::HmdVector3_t& b) const {
  float da = Dot(a,b);
  da = clamp_val(da, -1.0f, 1.0f);
  return std::acos(da) * (180.0f / 3.14159265358979323846f);
}

vr::HmdVector3_t AdaptiveGazeSmoother::Process(const vr::HmdVector3_t& rawDir, int64_t timestampUs) {
  vr::HmdVector3_t r = Normalize(rawDir);

  if (m_lastTimestampUs == 0) {
    m_lastTimestampUs = timestampUs;
    m_lastRaw = r;
    m_lastSmoothed = r;
    return r;
  }

  int64_t dtUs = timestampUs - m_lastTimestampUs;
  if (dtUs <= 0) dtUs = 10000;
  float dtSec = static_cast<float>(dtUs) / 1e6f;
  dtSec = clamp_val(dtSec, 0.0f, kMaxDtSec);

  float angleDelta = AngleBetween(r, m_lastRaw);
  m_recentAngles[m_recentIndex] = angleDelta;
  m_recentIndex = (m_recentIndex + 1) % kWindowSize;

  std::vector<float> tmp = m_recentAngles;
  std::sort(tmp.begin(), tmp.end());
  float jitterMetric = 0.0f;
  if (tmp.size() > 2) {
    for (size_t i = 1; i + 1 < tmp.size(); ++i) jitterMetric += tmp[i];
    jitterMetric /= (tmp.size() - 2);
  } else if (!tmp.empty()) {
    float sum = 0.0f; for (auto v : tmp) sum += v; jitterMetric = sum / tmp.size();
  }

  m_jitterEma = (m_jitterEma == 0.0f) ? jitterMetric : (m_jitterEma * (1.0f - kJitterEmaAlpha) + jitterMetric * kJitterEmaAlpha);

  if (m_jitterEma < (kLowThresholdDeg * kDecreaseHysteresis)) {
    m_lowJitterCount++;
  } else {
    m_lowJitterCount = 0;
  }

  float maxIncrease = kIncreasePerSecond * dtSec;
  float maxDecrease = kDecreasePerSecond * dtSec;

  if (m_jitterEma > kHighThresholdDeg) {
    float factor = clamp_val(m_jitterEma / kHighThresholdDeg, 0.0f, 1.0f);
    m_smoothingAmount = std::min(kMaxSmoothing, m_smoothingAmount + maxIncrease * factor);
  } else if (m_lowJitterCount >= kRequiredLowFrames) {
    float decreaseFactor = clamp_val((kLowThresholdDeg - m_jitterEma) / kLowThresholdDeg, 0.0f, 1.0f);
    m_smoothingAmount = std::max(kMinSmoothing, m_smoothingAmount - maxDecrease * decreaseFactor);
    m_lowJitterCount = 0;
  }

  float alpha = 1.0f - std::pow(1.0f - m_smoothingAmount, dtSec * 60.0f);
  alpha = clamp_val(alpha, 0.0f, 1.0f);

  vr::HmdVector3_t smoothed;
  smoothed.v[0] = m_lastSmoothed.v[0] * (1.0f - alpha) + r.v[0] * alpha;
  smoothed.v[1] = m_lastSmoothed.v[1] * (1.0f - alpha) + r.v[1] * alpha;
  smoothed.v[2] = m_lastSmoothed.v[2] * (1.0f - alpha) + r.v[2] * alpha;
  smoothed = Normalize(smoothed);

  float stabilityAngle = AngleBetween(smoothed, r);
  if (stabilityAngle < (kLowThresholdDeg * 0.5f)) {
    float tinyBackoff = kDecreasePerSecond * dtSec * 0.25f;
    m_smoothingAmount = std::max(kMinSmoothing, m_smoothingAmount - tinyBackoff);
  }

  m_smoothingAmount = clamp_val(m_smoothingAmount, kMinSmoothing, kMaxSmoothing);

  m_lastRaw = r;
  m_lastSmoothed = smoothed;
  m_lastTimestampUs = timestampUs;

  return smoothed;
}

} // namespace psvr2_toolkit
