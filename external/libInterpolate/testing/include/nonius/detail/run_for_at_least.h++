// Nonius - C++ benchmarking tool
//
// Written in 2014 by Martinho Fernandes <martinho.fernandes@gmail.com>
//
// To the extent possible under law, the author(s) have dedicated all copyright and related
// and neighboring rights to this software to the public domain worldwide. This software is
// distributed without any warranty.
//
// You should have received a copy of the CC0 Public Domain Dedication along with this software.
// If not, see <http://creativecommons.org/publicdomain/zero/1.0/>

// Run a function for a minimum amount of time

#ifndef NONIUS_RUN_FOR_AT_LEAST_HPP
#define NONIUS_RUN_FOR_AT_LEAST_HPP

#include <nonius/clock.h++>
#include <nonius/timeout_error.h++>
#include <nonius/detail/measure.h++>
#include <nonius/detail/timing.h++>

#include <utility>

namespace nonius {
    namespace detail {
        template <typename Clock = default_clock, typename Fun>
        TimingOf<Clock, Fun(int)> run_for_at_least(Duration<Clock> how_long, int seed, Fun&& fun) {
            auto iters = seed;
            auto start = Clock::now();
            while(true) {
                auto now = Clock::now();
                if(now - start > how_long * 10) {
                    throw timeout_error(seed, iters);
                }
                auto r = detail::measure<Clock>(fun, iters);
                if(r.elapsed >= how_long) {
                    return { r.elapsed, std::move(r.result), iters };
                }
                iters *= 2;
            }
        }
    } // namespace detail
} // namespace nonius

#endif // NONIUS_RUN_FOR_AT_LEAST_HPP

