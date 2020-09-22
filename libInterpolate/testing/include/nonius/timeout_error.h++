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

// Exception to be thrown when a process takes too long to run

#ifndef NONIUS_TIMEOUT_ERROR_HPP
#define NONIUS_TIMEOUT_ERROR_HPP

#include <nonius/detail/noexcept.h++>

#include <sstream>
#include <string>
#include <exception>

namespace nonius {
    struct timeout_error : virtual std::exception {
    public:
        timeout_error(int seed, int iters) {
            std::ostringstream ss;
            ss << "took too long to run; seed: " << seed << ", iters: " << iters;
            message = ss.str();
        }

        char const* what() const NONIUS_NOEXCEPT override {
            return message.c_str();
        }

    private:
        std::string message;
    };
} // namespace nonius

#endif // NONIUS_TIMEOUT_ERROR_HPP
