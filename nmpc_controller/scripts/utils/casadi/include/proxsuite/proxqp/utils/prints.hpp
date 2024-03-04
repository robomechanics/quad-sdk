//
// Copyright (c) 2022 INRIA
//
/** \file */

#ifndef PROXSUITE_PROXQP_UTILS_PRINTS_HPP
#define PROXSUITE_PROXQP_UTILS_PRINTS_HPP

#include <iostream>

namespace proxsuite {
namespace proxqp {

inline void
print_line()
{
  std::string the_line = "-----------------------------------------------------"
                         "--------------------------------------------\0";
  std::cout << the_line << "\n" << std::endl;
}

inline void
print_header()
{
  std::cout << "iter    objective    pri res    dua res    mu_in  \n"
            << std::endl;
}

inline void
print_preambule()
{
  print_line();
  std::cout
    << "                              ProxQP - Primal-Dual Proximal QP "
       "Solver\n"
    << "     (c) Antoine Bambade, Sarah El Kazdadi, Fabian Schramm, Adrien "
       "Taylor, and "
       "Justin Carpentier\n"
    << "                                         Inria Paris 2022        \n"
    << std::endl;
  print_line();
}

} // end namespace proxqp
} // end namespace proxsuite

#endif /* end of include guard PROXSUITE_PROXQP_UTILS_PRINTS_HPP */
