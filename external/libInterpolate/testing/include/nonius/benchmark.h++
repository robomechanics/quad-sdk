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

// Benchmark

#ifndef NONIUS_BENCHMARK_HPP
#define NONIUS_BENCHMARK_HPP

#include <nonius/clock.h++>
#include <nonius/configuration.h++>
#include <nonius/environment.h++>
#include <nonius/execution_plan.h++>
#include <nonius/chronometer.h++>
#include <nonius/detail/measure.h++>
#include <nonius/detail/benchmark_function.h++>
#include <nonius/detail/repeat.h++>
#include <nonius/detail/run_for_at_least.h++>
#include <nonius/detail/unique_name.h++>

#include <algorithm>
#include <functional>
#include <string>
#include <vector>
#include <cmath>

namespace nonius {
    namespace detail {
        const auto warmup_iterations = 10000;
        const auto warmup_time = chrono::milliseconds(100);
        const auto minimum_ticks = 1000;
    } // namespace detail

    struct benchmark {
        benchmark(std::string name, detail::benchmark_function fun)
        : name(std::move(name)), fun(std::move(fun)) {}

        void operator()(chronometer meter) const {
            fun(meter);
        }

        template <typename Clock>
        execution_plan<FloatDuration<Clock>> prepare(configuration cfg, environment<FloatDuration<Clock>> env) const {
            auto min_time = env.clock_resolution.mean * detail::minimum_ticks;
            auto run_time = std::min(min_time, decltype(min_time)(detail::warmup_time));
            auto&& test = detail::run_for_at_least<Clock>(chrono::duration_cast<Duration<Clock>>(run_time), 1, [this](int k) {
                detail::chronometer_model<Clock> model;
                (*this)(chronometer(model, k));
            });
            int new_iters = static_cast<int>(std::ceil(min_time * test.iterations / test.elapsed));
            return { new_iters, test.elapsed / test.iterations * new_iters * cfg.samples };
        }

        template <typename Clock>
        std::vector<FloatDuration<Clock>> run(configuration cfg, environment<FloatDuration<Clock>> env, execution_plan<FloatDuration<Clock>> plan) const {
            // warmup a bit
            detail::run_for_at_least<Clock>(chrono::duration_cast<Duration<Clock>>(detail::warmup_time), detail::warmup_iterations, detail::repeat(now<Clock>{}));

            std::vector<FloatDuration<Clock>> times;
            times.reserve(cfg.samples);
            std::generate_n(std::back_inserter(times), cfg.samples, [this, env, plan]{
                    detail::chronometer_model<Clock> model;
                    (*this)(chronometer(model, plan.iterations_per_sample));
                    auto elapsed = model.finished - model.started;
                    auto sample_time = elapsed - env.clock_cost.mean;
                    if(sample_time < FloatDuration<Clock>::zero()) sample_time = FloatDuration<Clock>::zero();
                    return (sample_time / plan.iterations_per_sample);
            });
            return times;
        }

        std::string name;
        detail::benchmark_function fun;
    };

    using benchmark_registry = std::vector<benchmark>;

    inline benchmark_registry& global_benchmark_registry() {
        static benchmark_registry registry;
        return registry;
    }

    struct benchmark_registrar {
        template <typename Fun>
        benchmark_registrar(benchmark_registry& registry, std::string name, Fun&& registrant) {
            registry.emplace_back(std::move(name), std::forward<Fun>(registrant));
        }
    };
} // namespace nonius

#define NONIUS_BENCHMARK(name, ...) \
    namespace { static ::nonius::benchmark_registrar NONIUS_DETAIL_UNIQUE_NAME(benchmark_registrar) (::nonius::global_benchmark_registry(), name, __VA_ARGS__); }

#endif // NONIUS_BENCHMARK_HPP

