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

// Dumb std::function implementation for consistent call overhead

#ifndef NONIUS_DETAIL_BENCHMARK_FUNCTION_HPP
#define NONIUS_DETAIL_BENCHMARK_FUNCTION_HPP

#include <nonius/chronometer.h++>
#include <nonius/detail/complete_invoke.h++>

#include <type_traits>
#include <utility>
#include <memory>

namespace nonius {
    namespace detail {
        template <typename T>
        using Decay = typename std::decay<T>::type;
        template <typename T, typename U>
        struct is_related
        : std::is_same<Decay<T>, Decay<U>> {};

        struct benchmark_function {
        private:
            struct concept {
                virtual void call(chronometer meter) const = 0;
                virtual concept* clone() const = 0;
                virtual ~concept() = default;
            };
            template <typename Fun>
            struct model : public concept {
                model(Fun&& fun) : fun(std::move(fun)) {}
                model(Fun const& fun) : fun(fun) {}

                model<Fun>* clone() const override { return new model<Fun>(*this); }

                void call(chronometer meter) const override {
                    call(meter, is_callable<Fun(chronometer)>());
                }
                void call(chronometer meter, std::true_type) const {
                    fun(meter);
                }
                void call(chronometer meter, std::false_type) const {
                    meter.measure(fun);
                }

                Fun fun;
            };
        public:
            template <typename Fun,
                      typename std::enable_if<!is_related<Fun, benchmark_function>::value, int>::type = 0>
            benchmark_function(Fun&& fun)
            : f(new model<typename std::decay<Fun>::type>(std::forward<Fun>(fun))) {}

            benchmark_function(benchmark_function&& that)
            : f(std::move(that.f)) {}

            benchmark_function(benchmark_function const& that)
            : f(that.f->clone()) {}

            benchmark_function& operator=(benchmark_function&& that) {
                f = std::move(that.f);
                return *this;
            }

            benchmark_function& operator=(benchmark_function const& that) {
                f.reset(that.f->clone());
                return *this;
            }

            void operator()(chronometer meter) const { f->call(meter); }
        private:
            std::unique_ptr<concept> f;
        };
    } // namespace detail
} // namespace nonius

#endif // NONIUS_DETAIL_BENCHMARK_FUNCTION_HPP

