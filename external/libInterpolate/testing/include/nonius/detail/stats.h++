// Nonius - C++ benchmarking tool
//
// Written in 2014-2015 by Martinho Fernandes <martinho.fernandes@gmail.com>
//
// To the extent possible under law, the author(s) have dedicated all copyright and related
// and neighboring rights to this software to the public domain worldwide. This software is
// distributed without any warranty.
//
// You should have received a copy of the CC0 Public Domain Dedication along with this software.
// If not, see <http://creativecommons.org/publicdomain/zero/1.0/>

// Statistical analysis tools

#ifndef NONIUS_DETAIL_ANALYSIS_HPP
#define NONIUS_DETAIL_ANALYSIS_HPP

#include <nonius/clock.h++>
#include <nonius/estimate.h++>
#include <nonius/outlier_classification.h++>

#include <boost/math/distributions/normal.hpp>

#include <algorithm>
#include <functional>
#include <iterator>
#include <vector>
#include <array>
#include <random>
#include <numeric>
#include <tuple>
#include <cmath>
#include <utility>
#include <future>
#include <cstddef>

namespace nonius {
    namespace detail {
        using sample = std::vector<double>;

        template <typename Iterator>
        double weighted_average_quantile(int k, int q, Iterator first, Iterator last) {
            auto count = last - first;
            double idx = (count - 1) * k /static_cast<double>(q);
            int j = static_cast<int>(idx);
            double g = idx - j;
            std::nth_element(first, first+j, last);
            auto xj = first[j];
            if(g == 0) return xj;

            auto xj1 = *std::min_element(first+(j+1), last);
            return xj + g * (xj1 - xj);
        }

        template <typename Iterator>
        outlier_classification classify_outliers(Iterator first, Iterator last) {
            std::vector<double> copy(first, last);

            auto q1 = weighted_average_quantile(1, 4, copy.begin(), copy.end());
            auto q3 = weighted_average_quantile(3, 4, copy.begin(), copy.end());
            auto iqr = q3 - q1;
            auto los = q1 - (iqr * 3.);
            auto lom = q1 - (iqr * 1.5);
            auto him = q3 + (iqr * 1.5);
            auto his = q3 + (iqr * 3.);

            outlier_classification o;
            for(; first != last; ++first) {
                auto&& t = *first;
                if(t < los) ++o.low_severe;
                else if(t < lom) ++o.low_mild;
                else if(t > his) ++o.high_severe;
                else if(t > him) ++o.high_mild;
                ++o.samples_seen;
            }
            return o;
        }

        template <typename Iterator>
        double mean(Iterator first, Iterator last) {
            auto count = last - first;
            double sum = std::accumulate(first, last, 0.);
            return sum / count;
        }

        template <typename Iterator>
        double standard_deviation(Iterator first, Iterator last) {
            auto m = mean(first, last);
            double variance = std::accumulate(first, last, 0., [m](double a, double b) {
                        double diff = b - m;
                        return a + diff*diff;
                    }) / (last - first);
            return std::sqrt(variance);
        }

        template <typename URng, typename Iterator, typename Estimator>
        sample resample(URng& rng, int resamples, Iterator first, Iterator last, Estimator& estimator) {
            auto n = last - first;
            std::uniform_int_distribution<decltype(n)> dist(0, n-1);

            sample out;
            out.reserve(resamples);
            std::generate_n(std::back_inserter(out), resamples, [n, first, &estimator, &dist, &rng] {
                std::vector<double> resampled;
                resampled.reserve(n);
                std::generate_n(std::back_inserter(resampled), n, [first, &dist, &rng] { return first[dist(rng)]; });
                return estimator(resampled.begin(), resampled.end());
            });
            std::sort(out.begin(), out.end());
            return out;
        }

        template <typename Estimator, typename Iterator>
        sample jackknife(Estimator&& estimator, Iterator first, Iterator last) {
            auto n = last - first;
            auto second = std::next(first);
            sample results;
            results.reserve(n);

            for(auto it = first; it != last; ++it) {
                std::iter_swap(it, first);
                results.push_back(estimator(second, last));
            }

            return results;
        }

        template <typename Iterator, typename Estimator>
        estimate<double> bootstrap(double confidence_level, Iterator first, Iterator last, sample const& resample, Estimator&& estimator) {
            namespace bm = boost::math;

            auto n_samples = last - first;

            double point = estimator(first, last);
            // Degenerate case with a single sample
            if(n_samples == 1) return { point, point, point, confidence_level };

            sample jack = jackknife(estimator, first, last);
            double jack_mean = mean(jack.begin(), jack.end());
            double sum_squares, sum_cubes;
            std::tie(sum_squares, sum_cubes) = std::accumulate(jack.begin(), jack.end(), std::make_pair(0., 0.), [jack_mean](std::pair<double, double> sqcb, double x) -> std::pair<double, double> {
                        auto d = jack_mean - x;
                        auto d2 = d * d;
                        auto d3 = d2 * d;
                        return { sqcb.first + d2, sqcb.second + d3 };
                    });

            double accel = sum_cubes / (6 * std::pow(sum_squares, 1.5));
            int n = static_cast<int>(resample.size());
            double prob_n = std::count_if(resample.begin(), resample.end(), [point](double x) { return x < point; }) /(double) n;
            // degenerate case with uniform samples
            if(prob_n == 0) return { point, point, point, confidence_level };

            double bias = bm::quantile(bm::normal{}, prob_n);
            double z1 = bm::quantile(bm::normal{}, (1. - confidence_level) / 2.);

            auto cumn = [n](double x) -> int { return std::lround(bm::cdf(bm::normal{}, x) * n); };
            auto a = [bias, accel](double b) { return bias + b / (1. - accel * b); };
            double b1 = bias + z1;
            double b2 = bias - z1;
            double a1 = a(b1);
            double a2 = a(b2);
            auto lo = std::max(cumn(a1), 0);
            auto hi = std::min(cumn(a2), n - 1);

            return { point, resample[lo], resample[hi], confidence_level };
        }

        inline double outlier_variance(estimate<double> mean, estimate<double> stddev, int n) {
            double sb = stddev.point;
            double mn = mean.point / n;
            double mg_min = mn / 2.;
            double sg = std::min(mg_min / 4., sb / std::sqrt(n));
            double sg2 = sg * sg;
            double sb2 = sb * sb;

            auto c_max = [n, mn, sb2, sg2](double x) -> double {
                double k = mn - x;
                double d = k * k;
                double nd = n * d;
                double k0 = -n * nd;
                double k1 = sb2 - n * sg2 + nd;
                double det = k1 * k1 - 4 * sg2 * k0;
                return (int)(-2. * k0 / (k1 + std::sqrt(det)));
            };

            auto var_out = [n, sb2, sg2](double c) {
                double nc = n - c;
                return (nc / n) * (sb2 - nc * sg2);
            };

            return std::min(var_out(1), var_out(std::min(c_max(0.), c_max(mg_min)))) / sb2;
        }

        struct bootstrap_analysis {
            estimate<double> mean;
            estimate<double> standard_deviation;
            double outlier_variance;
        };

        template <typename Iterator>
        bootstrap_analysis analyse_samples(double confidence_level, int n_resamples, Iterator first, Iterator last) {
            static std::random_device entropy;

            auto n = static_cast<int>(last - first); // seriously, one can't use integral types without hell in C++

            auto mean = &detail::mean<Iterator>;
            auto stddev = &detail::standard_deviation<Iterator>;

            auto estimate = [=](double(*f)(Iterator, Iterator)) {
                auto seed = entropy();
                return std::async(std::launch::async, [=]{
                    std::mt19937 rng(seed);
                    auto resampled = resample(rng, n_resamples, first, last, f);
                    return bootstrap(confidence_level, first, last, resampled, f);
                });
            };

            auto mean_future = estimate(mean);
            auto stddev_future = estimate(stddev);

            auto mean_estimate = mean_future.get();
            auto stddev_estimate = stddev_future.get();

            double outlier_variance = detail::outlier_variance(mean_estimate, stddev_estimate, n);

            return { mean_estimate, stddev_estimate, outlier_variance };
        }
    } // namespace detail
} // namespace nonius

#endif // NONIUS_DETAIL_ANALYSIS_HPP

