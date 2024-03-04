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

/*
 * Interface for SCS signal handling.
 */

#ifndef CTRLC_H_GUARD
#define CTRLC_H_GUARD

#if !(defined _WIN32 || defined _WIN64 || defined _WINDLL || defined __APPLE__)
#ifdef _POSIX_C_SOURCE
#undef _POSIX_C_SOURCE
#endif
#define _POSIX_C_SOURCE 199309L
#endif

#if CTRLC > 0

#if defined MATLAB_MEX_FILE

/* No header file available here; define the prototypes ourselves */
extern int utIsInterruptPending();
extern int utSetInterruptEnabled(int);

#elif(defined _WIN32 || defined _WIN64 || defined _WINDLL)

/* Use Windows SetConsoleCtrlHandler for signal handling */
#include <windows.h>

#else

/* Use POSIX clocl_gettime() for timing on non-Windows machines */
#include <signal.h>

#endif

/* METHODS are the same for both */
void startInterruptListener(void);
void endInterruptListener(void);
int isInterrupted(void);

#else /* CTRLC = 0 */

/* No signal handling. */
#define startInterruptListener()
#define endInterruptListener()
#define isInterrupted() 0

#endif /* END IF CTRLC > 0 */

#endif /* END IFDEF __TIMER_H__ */
