# Authors: Giulio Romualdi
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

add_osqpeigen_test(
  NAME SparseMatrix
  SOURCES SparseMatrixTest.cpp
  LINKS OsqpEigen::OsqpEigen)

add_osqpeigen_test(
  NAME QP
  SOURCES QPTest.cpp
  LINKS OsqpEigen::OsqpEigen)

add_osqpeigen_test(
  NAME UpdateMatrices
  SOURCES UpdateMatricesTest.cpp
  LINKS OsqpEigen::OsqpEigen)

add_osqpeigen_test(
  NAME MPC
  SOURCES MPCTest.cpp
  LINKS OsqpEigen::OsqpEigen
  COMPILE_DEFINITIONS _USE_MATH_DEFINES)

add_osqpeigen_test(
  NAME MPCUpdateMatrices
  SOURCES MPCUpdateMatricesTest.cpp
  LINKS OsqpEigen::OsqpEigen
  COMPILE_DEFINITIONS _USE_MATH_DEFINES)
