#ifndef FUNCTION_TIMER_H
#define FUNCTION_TIMER_H

#include <iostream>
#include <chrono>

namespace spirit_utils {

//! A lightweight class for measuring and reporting the duration of functions calls
/*!
  FunctionTimer keeps track of the amount of time elapsed between start and stop calls,
  and reporting this along with the name of the function.
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
     */
    void report() {
      stop_time_ = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(stop_time_ - start_time_);
      double current_time = elapsed.count();

      printf("Time spent in %s = %.4es\n", function_name_, current_time);
    };

  private:
    /// The time at the start of the function call
    std::chrono::time_point<std::chrono::steady_clock> start_time_;

    /// The time at the which the report is queried
    std::chrono::time_point<std::chrono::steady_clock> stop_time_;

    /// Name of the function being timed
    char* function_name_;

};

}

#endif // FUNCTION_TIMER_H
