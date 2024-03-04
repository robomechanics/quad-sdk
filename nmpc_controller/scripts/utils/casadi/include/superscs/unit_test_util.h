/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Pantelis Sopasakis (https://alphaville.github.io),
 *                    Krina Menounou (https://www.linkedin.com/in/krinamenounou), 
 *                    Panagiotis Patrinos (http://homes.esat.kuleuven.be/~ppatrino)
 * Copyright (c) 2012 Brendan O'Donoghue (bodonoghue85@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 */

#ifndef UNITTESTS_H
#define UNITTESTS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "glbopts.h"
#include <math.h>
#include "linAlg.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef bool
#undef bool
#endif

#ifdef true
#undef true
#endif

#ifdef false
#undef false
#endif

    typedef int bool;
#define true 1
#define false 0


    int number_of_assertions;

#define TEST_SUCCESS 0 /**< test is successful */
#define TEST_FAILURE 1 /**< test fails */
#define MESSAGE_OK "OK" /**< a message returned when a test is successful */
#define TEST_PASS_FLAG "\x1B[92m[PASS]\x1B[39m " /**< flag for PASS */
#define TEST_FAIL_FLAG "\x1B[31m<FAIL>\x1B[39m " /**< flag for FAIL */   
#define TEST_MESSAGE_BUFF_SIZE 500
#define TEST_ERROR_MESSAGE_SIZE 100
    /**
     * Fails with a given message.
     */
#define FAIL_WITH_MESSAGE(str, message)\
        *str = (char*)(message);\
        return TEST_FAILURE


#define ASSERT_TRUE_OR_FAIL(p, str, message)\
    number_of_assertions++;\
    if (!(p)) { \
        FAIL_WITH_MESSAGE((str), (message));\
    }

    /**
     * Check whether two integers are equal, or fail with a given message.
     */
#define ASSERT_EQUAL_INT_OR_FAIL(val, expected, str, message)\
        number_of_assertions++;\
        if (!scs_assert_equals_int((val),(expected))) { \
          {\
           char buff[TEST_MESSAGE_BUFF_SIZE];\
           char error_msg[TEST_ERROR_MESSAGE_SIZE];\
           sprintf(error_msg, "\n\tExpected: %d, Actual %d", expected, val);\
           strncpy(buff, message, TEST_ERROR_MESSAGE_SIZE);\
           strncat(buff, error_msg, TEST_ERROR_MESSAGE_SIZE);\
           FAIL_WITH_MESSAGE((str), (buff)); \
          }\
        }

    /**
     * Check whether two integers are equal, or fail with a given message.
     */
#define ASSERT_EQUAL_FLOAT_OR_FAIL(val, expected, tol, str, message)\
        number_of_assertions++;\
        if (!scs_assert_equals_float((val), (expected), (tol))) {\
           char buff[TEST_MESSAGE_BUFF_SIZE];\
           char error_msg[TEST_ERROR_MESSAGE_SIZE];\
           sprintf(error_msg, "\n\tExpected: %g, Actual %g (tol=%g)", expected, val, tol);\
           strncpy(buff, message, TEST_ERROR_MESSAGE_SIZE);\
           strncat(buff, error_msg, TEST_ERROR_MESSAGE_SIZE);\
           FAIL_WITH_MESSAGE((str), (buff)); \
        }

    /**
     * Check whether two arrays are equal, or fail with a given message.
     */
#define ASSERT_EQUAL_ARRAY_OR_FAIL(val,expected,len,tol,str,message)\
    number_of_assertions++;\
    if (!scs_assert_equals_array((val),(expected),(len),(tol))){\
      FAIL_WITH_MESSAGE((str), (message));\
    }

    /**
     * Check whether two arrays are equal, or fail with a given message.
     */
#define ASSERT_EQUAL_ARRAY_INT_OR_FAIL(val,expected,len,str,message)\
    number_of_assertions++;\
    if (!scs_assert_equals_array_int((val),(expected),(len))){\
      FAIL_WITH_MESSAGE((str), (message));\
    }

    /**
     * Succeed
     */
#define SUCCEED(str)\
        *str = (char*) MESSAGE_OK;\
        return TEST_SUCCESS

    /**
     * Function template defining a unit test:
     * 
     *  int myTestFunction(char**);
     * 
     * This type is a pointer to such a function which takes as an input argument 
     * a pointer to a string (char**) and returns a status code (either #TEST_SUCCESS
     * or #TEST_FAILURE).
     */
    typedef bool (*unitTest_t)(char**);


    /**
     * Tester function.
     * @param ut Unit Test function handle
     * @param name Name of the test
     * @return #TEST_SUCCESS if the test succeeds and #TEST_FAILURE if it fails.
     */
    bool scs_test(const unitTest_t ut, const char* name);

    /**
     * Assert that two integers are equal.
     * @param a
     * @param b
     * @return 
     */
    bool scs_assert_equals_int(const scs_int a, const scs_int b);

    /**
     * Assert that two floats are equal up to a given tolerance.
     * @param a
     * @param b
     * @param tol tolerance
     * @return 
     */
    bool scs_assert_equals_float(const scs_float a, const scs_float b, const scs_float tol);

    /**
     * Checks whether two arrays of float are equal, element-wise, up to a certain
     * tolerance.
     * 
     * @param a first array
     * @param b second array
     * @param n length of array
     * @param tol tolerance
     * @return \c true is the two arrays are equal
     */
    bool scs_assert_equals_array(
            const scs_float * a,
            const scs_float * b,
            scs_int n,
            const scs_float tol);

    /**
     * Checks whether two arrays of float are equal, element-wise, up to a certain
     * tolerance.
     * 
     * @param a first array
     * @param b second array
     * @param n length of array
     * @return \c true is the two arrays are equal
     */
    bool scs_assert_equals_array_int(
            const scs_int * a,
            const scs_int * b,
            scs_int n);


#ifdef __cplusplus
}
#endif

#endif /* UNITTESTS_H */
