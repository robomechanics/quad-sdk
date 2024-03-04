/* $Id$ */
// Copyright (C) 2004, International Business Machines
// Corporation and others.  All Rights Reserved.
// This code is licensed under the terms of the Eclipse Public License (EPL).

#ifndef _CoinTypes_hpp
#define _CoinTypes_hpp

#include "CoinUtilsConfig.h"
/* On some systems, we require stdint.h to have the 64bit integer type defined. */
#ifdef COINUTILS_HAS_STDINT_H
#include <stdint.h>
#endif
#ifdef COINUTILS_HAS_CSTDINT
#include <cstdint>
#endif

#define CoinInt64 COIN_INT64_T
#define CoinUInt64 COIN_UINT64_T
#define CoinIntPtr COIN_INTPTR_T

//=============================================================================
#ifndef COIN_BIG_INDEX
#define COIN_BIG_INDEX 0
#endif

#if COIN_BIG_INDEX == 0
typedef int CoinBigIndex;
#elif COIN_BIG_INDEX == 1
typedef long CoinBigIndex;
#else
typedef long long CoinBigIndex;
#endif

//=============================================================================
#ifndef COIN_BIG_DOUBLE
#define COIN_BIG_DOUBLE 0
#endif

// See if we want the ability to have long double work arrays
#if COIN_BIG_DOUBLE == 2
#undef COIN_BIG_DOUBLE
#define COIN_BIG_DOUBLE 0
#define COIN_LONG_WORK 1
typedef long double CoinWorkDouble;
#elif COIN_BIG_DOUBLE == 3
#undef COIN_BIG_DOUBLE
#define COIN_BIG_DOUBLE 1
#define COIN_LONG_WORK 1
typedef long double CoinWorkDouble;
#else
#define COIN_LONG_WORK 0
typedef double CoinWorkDouble;
#endif

#if COIN_BIG_DOUBLE == 0
typedef double CoinFactorizationDouble;
#elif COIN_BIG_DOUBLE == 1
typedef long double CoinFactorizationDouble;
#else
typedef double CoinFactorizationDouble;
#endif

#endif

/* vi: softtabstop=2 shiftwidth=2 expandtab tabstop=2
*/
