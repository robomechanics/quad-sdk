//
// Copyright (c) 2022 INRIA
//

#ifndef PROXSUITE_PROXQP_TIMINGS_HPP
#define PROXSUITE_PROXQP_TIMINGS_HPP

#include <chrono>

namespace proxsuite {
namespace proxqp {

struct CPUTimes
{
  double wall;
  double user;
  double system;

  CPUTimes()
    : wall(0)
    , user(0)
    , system(0)
  {
  }

  void clear() { wall = user = system = 0; }
};

///
/// @brief This class mimics the way "boost/timer/timer.hpp" operates while
/// using the modern std::chrono library.
/// Importantly, this class will only have an effect for C++11 and more.
///
template<typename T>
struct Timer
{
  Timer()
    : m_is_stopped(true)
  {
    start();
  }

  CPUTimes elapsed() const
  {
    if (m_is_stopped)
      return m_times;

    CPUTimes current(m_times);
    std::chrono::time_point<std::chrono::steady_clock> current_clock =
      std::chrono::steady_clock::now();
    current.user +=
      static_cast<T>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                       current_clock - m_start)
                       .count()) *
      1e-3;

    return current;
  }

  void start()
  {
    if (m_is_stopped) {
      m_is_stopped = false;
      m_times.clear();
      m_start = std::chrono::steady_clock::now();
    }
  }

  void stop()
  {
    if (m_is_stopped)
      return;
    m_is_stopped = true;

    m_end = std::chrono::steady_clock::now();
    m_times.user +=
      static_cast<double>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(m_end - m_start)
          .count()) *
      1e-3;
  }

  void resume()
  {
    if (m_is_stopped)
      m_start = std::chrono::steady_clock::now();
  }

  bool is_stopped() const { return m_is_stopped; }

protected:
  CPUTimes m_times;
  bool m_is_stopped;

  std::chrono::time_point<std::chrono::steady_clock> m_start, m_end;
};

} // namespace proxqp
} // namespace proxsuite

#endif // ifndef PROXSUITE_PROXQP_TIMINGS_HPP
