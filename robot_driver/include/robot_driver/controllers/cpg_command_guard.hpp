#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace cpg_guard {
// Sample-based bound on PD + feedforward. This changes transmitted gains/torque,
// not just telemetry. It is NOT a guarantee of physical torque between samples.
struct Motor { double q, dq, kp, kd, ff; };
inline double effort(const Motor& m, double q, double dq) {
  return m.kp*(m.q-q)+m.kd*(m.dq-dq)+m.ff;
}
inline bool finite(const Motor& m) {
  return std::isfinite(m.q)&&std::isfinite(m.dq)&&std::isfinite(m.kp)&&
         std::isfinite(m.kd)&&std::isfinite(m.ff)&&m.kp>=0&&m.kd>=0;
}
inline double limit(Motor& m, double q, double dq, double cap=18.) {
  if (!finite(m)||!std::isfinite(q)||!std::isfinite(dq)||!std::isfinite(cap)||cap<=0)
    throw std::invalid_argument("nonfinite CPG motor/state or invalid limit");
  const double raw=effort(m,q,dq);
  const double envelope=(raw*dq>=0 ? 20.2 : 23.4)*
      std::clamp((30.-std::abs(dq))/(30.-13.5),0.,1.);
  const double bound=std::min(cap,envelope);
  // Scale all gains and feedforward together to avoid cancellation artefacts.
  const double scale=std::abs(raw)>bound ? bound/std::abs(raw) : 1.;
  m.kp*=scale;m.kd*=scale;m.ff*=scale;
  return scale;
}
inline bool targetRange(double q,int joint) {
  // Conservative subset of Go2 URDF bounds; calf stays on negative-knee branch.
  constexpr double lo[3]={-.95,-1.32,-2.63}, hi[3]={.95,3.24,-.93};
  return std::isfinite(q)&&q>=lo[joint]&&q<=hi[joint];
}
inline bool measuredRange(double q,double dq,int joint) {
  constexpr double lo[3]={-1.20,-1.73,-2.88},hi[3]={1.20,3.65,-.68};
  constexpr double speed[3]={35.,35.,18.};
  return std::isfinite(q)&&std::isfinite(dq)&&q>=lo[joint]&&q<=hi[joint]&&std::abs(dq)<=speed[joint];
}
} // namespace cpg_guard
