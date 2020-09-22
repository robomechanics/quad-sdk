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

// Cross-compiler noexcept support

#ifndef NONIUS_DETAIL_NOEXCEPT_HPP
#define NONIUS_DETAIL_NOEXCEPT_HPP

#ifdef _MSC_VER
#define NONIUS_NOEXCEPT throw()
#else
#define NONIUS_NOEXCEPT noexcept
#endif // _MSC_VER

#endif // NONIUS_DETAIL_NOEXCEPT_HPP

