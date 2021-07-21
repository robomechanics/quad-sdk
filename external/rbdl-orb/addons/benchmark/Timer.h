#ifndef _TIMER_H
#define _TIMER_H

#include <ctime>

struct TimerInfo {
  /// time stamp when timer_start() gets called
  clock_t clock_start_value;

  /// time stamp when the timer was stopped
  clock_t clock_end_value;

  /// duration between clock_start_value and clock_end_value in seconds
  double duration_sec;
};

inline void timer_start (TimerInfo *timer) {
  timer->clock_start_value = clock();
}

inline double timer_stop (TimerInfo *timer) {
  timer->clock_end_value = clock();

  timer->duration_sec = static_cast<double>(timer->clock_end_value - timer->clock_start_value) * 1 / CLOCKS_PER_SEC;

  return timer->duration_sec;
}

#endif
