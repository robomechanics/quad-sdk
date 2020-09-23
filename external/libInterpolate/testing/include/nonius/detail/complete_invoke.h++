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

// Invoke with a special case for void

#ifndef NONIUS_DETAIL_COMPLETE_INVOKE_HPP
#define NONIUS_DETAIL_COMPLETE_INVOKE_HPP

#include <type_traits>
#include <utility>

namespace nonius {
    namespace detail {
        template <typename T>
        struct complete_type { using type = T; };
        template <>
        struct complete_type<void> { struct type {}; };

        template <typename T>
        using CompleteType = typename complete_type<T>::type;

        template <typename Result>
        struct complete_invoker {
            template <typename Fun, typename... Args>
            static Result invoke(Fun&& fun, Args&&... args) {
                return std::forward<Fun>(fun)(std::forward<Args>(args)...);
            }
        };
        template <>
        struct complete_invoker<void> {
            template <typename Fun, typename... Args>
            static CompleteType<void> invoke(Fun&& fun, Args&&... args) {
                std::forward<Fun>(fun)(std::forward<Args>(args)...);
                return {};
            }
        };
        template <typename Sig>
        using ResultOf = typename std::result_of<Sig>::type;

        // invoke and not return void :(
        template <typename Fun, typename... Args>
        CompleteType<ResultOf<Fun(Args...)>> complete_invoke(Fun&& fun, Args&&... args) {
            return complete_invoker<ResultOf<Fun(Args...)>>::invoke(std::forward<Fun>(fun), std::forward<Args>(args)...);
        }

        template <typename> struct always_true : std::true_type {};
        struct is_callable_tester {
            template <typename Fun, typename... Args>
            always_true<decltype(std::declval<Fun>()(std::declval<Args>()...))> static test(int);
            template <typename...>
            std::false_type static test(...);
        };
        template <typename T>
        struct is_callable;
        template <typename Fun, typename... Args>
        struct is_callable<Fun(Args...)> : decltype(is_callable_tester::test<Fun, Args...>(0)) {};
    } // namespace detail
} // namespace nonius

#endif // NONIUS_DETAIL_COMPLETE_INVOKE_HPP

