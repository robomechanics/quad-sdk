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

// Statistics estimates

#ifndef NONIUS_ESTIMATE_HPP
#define NONIUS_ESTIMATE_HPP

namespace nonius {
    template <typename Duration>
    struct estimate {
        Duration point;
        Duration lower_bound;
        Duration upper_bound;
        double confidence_interval;

        template <typename Duration2>
        operator estimate<Duration2>() const {
            return { point, lower_bound, upper_bound, confidence_interval };
        }
    };
} // namespace nonius

#endif // NONIUS_ESTIMATE_HPP

