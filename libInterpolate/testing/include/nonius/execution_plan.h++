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

// Execution plan

#ifndef NONIUS_EXECUTION_PLAN_HPP
#define NONIUS_EXECUTION_PLAN_HPP

namespace nonius {
    template <typename Duration>
    struct execution_plan {
        int iterations_per_sample;
        Duration estimated_duration;

        template <typename Duration2>
        operator execution_plan<Duration2>() const {
            return { iterations_per_sample, estimated_duration };
        }
    };
} // namespace nonius

#endif // NONIUS_EXECUTION_PLAN_HPP

