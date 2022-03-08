#ifndef FUNCTION_TIMER_H
#define FUNCTION_TIMER_H

#include <chrono>
#include <iostream>

namespace quad_utils {

//! A lightweight class for measuring and reporting the duration of functions
//! calls
/*!
  FunctionTimer keeps track of the amount of time elapsed between start and stop
  calls, and reporting this along with the name of the function. For some reason
  the logic in this class takes about 1e-7 s to run so timing functions faster
  than that will yield inaccurate solutions compared to standard steady clock
  methods. For functions that take longer than 1e-6 s it should work.
*/
class FunctionTimer {
 public:
  /**
   * @brief Constructor for FunctionTimer Class
   * @return Constructed object of type FunctionTimer
   */
  FunctionTimer(const char* function_name) {
    function_name_ = const_cast<char*>(function_name);
    start_time_ = std::chrono::steady_clock::now();
  };

  /**
   * @brief Stop the timer and report the statistics
   * @return Time in seconds
   */
  double reportSilent() {
    stop_time_ = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed =
        std::chrono::duration_cast<std::chrono::duration<double>>(stop_time_ -
                                                                  start_time_);
    double current_time = elapsed.count();
    return current_time;
  };

  /**
   * @brief Stop the timer and report the statistics
   */
  double report() {
    double current_time = reportSilent();
    printf("Time spent in %s = %.2es\n", function_name_, current_time);
    return current_time;
  };

  /**
   * @brief Stop the timer and report the statistics
   */
  double report(int n) {
    double avg_time = reportSilent() / n;
    printf("Average time spent in %s = %.2es\n", function_name_, avg_time);
    return avg_time;
  };

  void reportAndRestart() {
    report();
    start_time_ = std::chrono::steady_clock::now();
  };

 private:
  /// The time at the start of the function call
  std::chrono::time_point<std::chrono::steady_clock> start_time_;

  /// The time at the which the report is queried
  std::chrono::time_point<std::chrono::steady_clock> stop_time_;

  /// Name of the function being timed
  char* function_name_;
};

}  // namespace quad_utils

#endif  // FUNCTION_TIMER_H
