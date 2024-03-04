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
#ifndef UTIL_H_GUARD
#define UTIL_H_GUARD

#ifdef __cplusplus
extern "C" {
#endif
    
#if !(defined _WIN32 || defined _WIN64 || defined _WINDLL || defined __APPLE__)   
#ifdef _POSIX_C_SOURCE
#undef _POSIX_C_SOURCE
#endif
#define _POSIX_C_SOURCE 200112L
#endif

#include <stdlib.h>
#include <stdarg.h>
#include "scs.h"

    /* timing code courtesy of A. Domahidi */
#if (defined NOTIMER)
    typedef void *ScsTimer;
#elif(defined _WIN32 || defined _WIN64 || defined _WINDLL)
    /* Use Windows QueryPerformanceCounter for timing */
#include <windows.h>

    struct scs_timer {
        LARGE_INTEGER tic;
        LARGE_INTEGER toc;
        LARGE_INTEGER freq;
    };

#elif(defined __APPLE__)
    /* Use MAC OSX mach_time for timing */
#include <mach/mach_time.h>

    struct scs_timer {
        uint64_t tic;
        uint64_t toc;
        mach_timebase_info_data_t tinfo;
    };

#else

#include <sys/time.h> /* Use POSIX clock_gettime() for timing on other machines */
#include <time.h>

    /** 
     * \brief SCS timer structure 
     */
    struct scs_timer {
        struct timespec tic;
        struct timespec toc;
    };


#endif

    /**
     * Typedef for the structure #scs_timer
     */
    typedef struct scs_timer ScsTimer;

    /* these all return milli-seconds */

    /**
     * \brief Starts the timer
     * @param timer pointer to timer structure
     */
    void scs_tic(ScsTimer *timer);
    /**
     * \brief Stops the timer 
     * 
     * This function prints a message of the form: 
     * 
     * ~~~~~
     * time: %8.4f milli-seconds.\n
     * ~~~~~
     * 
     * to the standard output using \c printf.
     * 
     * @param timer pointer to timer structure 
     * @return elapsed time in milliseconds
     */
    scs_float scs_toc(ScsTimer *timer);
    /**
     * \brief Stops the timer and prints a custom message
     * 
     * 
     * @param str string
     * @param timer pointer to timer structure 
     * @return elapsed time
     */
    scs_float scs_strtoc(char *str, ScsTimer *timer);
    /**
     * \brief Stops the timer 
     * 
     * \note In contrast to #scs_toc, this function does not print anything
     * 
     * @param timer pointer to timer structure
     * @return elapsed time in milliseconds
     * 
     * @sa #scs_toc
     */
    scs_float scs_toc_quiet(ScsTimer *timer);

    /**
     * \brief Prints the content of a Cone object
     * @param cone pointer to cone
     */
    void scs_print_cone_data(const ScsCone * RESTRICT cone);
    /**
     * \brief Prints the content of a Data object
     * @param data pointer to data
     */
    void scs_print_data(const ScsData *data);
    /**
     * \brief Prints the content of a Work object
     * @param work pointer to work
     */
    void scs_print_work(const ScsWork *work);

    /**
     * \brief Prints an array
     * 
     * @param arr pointer to array
     * @param n length of array
     * @param name name of the array
     */
    void scs_print_array(
            const scs_float * RESTRICT arr,
            scs_int n,
            const char *RESTRICT name);

    /**
     * \brief Sets the settings to certain default values
     * 
     * <table>
     * <tr><th>Parameter<th>Default value<th>Default value Macro
     * <tr><td>\ref ScsSettings#normalize "normalize"<td>1<td>::SCS_NORMALIZE_DEFAULT
     * <tr><td>\ref ScsSettings#scale "scale"<td>1.0<td>::SCS_SCALE_DEFAULT
     * <tr><td>\ref ScsSettings#rho_x "rho_x"<td>0.001<td>::SCS_RHO_X_DEFAULT
     * <tr><td>\ref ScsSettings#max_iters "max_iters"<td>10000<td>::SCS_MAX_ITERS_DEFAULT
     * <tr><td>\ref ScsSettings#max_time_milliseconds "max_time_milliseconds"<td>300000<td>::SCS_MAX_TIME_MILLISECONDS
     * <tr><td>\ref ScsSettings#previous_max_iters "previous_max_iters"<td>-1<td>::SCS_PMAXITER_DEFAULT
     * <tr><td>\ref ScsSettings#eps "eps"<td>1e-3<td>::SCS_EPS_DEFAULT
     * <tr><td>\ref ScsSettings#alpha "alpha"<td>1.5<td>::SCS_ALPHA_DEFAULT
     * <tr><td>\ref ScsSettings#verbose "verbose"<td>1<td>::SCS_VERBOSE_DEFAULT
     * <tr><td>\ref ScsSettings#warm_start "warm_start"<td>0<td>::SCS_WARM_START_DEFAULT
     * <tr><td>\ref ScsSettings#do_super_scs "do_super_scs"<td>1<td>::SCS_DO_SUPERSCS_DEFAULT
     * <tr><td>\ref ScsSettings#k0 "k0"<td>0<td>::SCS_K0_DEFAULT
     * <tr><td>\ref ScsSettings#k1 "k1"<td>1<td>::SCS_K1_DEFAULT
     * <tr><td>\ref ScsSettings#k2 "k2"<td>1<td>::SCS_K2_DEFAULT
     * <tr><td>\ref ScsSettings#c_bl "c_bl"<td>0.999<td>::SCS_C_BL_DEFAULT
     * <tr><td>\ref ScsSettings#c_bl "c1"<td>0.9999<td>::SCS_C1_DEFAULT
     * <tr><td>\ref ScsSettings#cg_rate "cg_rate"<td>2.0<td>::SCS_CG_RATE_DEFAULT
     * <tr><td>\ref ScsSettings#ls "ls"<td>10<td>::SCS_LS_DEFAULT
     * <tr><td>\ref ScsSettings#sse "sse"<td>0.999<td>::SCS_SSE_DEFAULT
     * <tr><td>\ref ScsSettings#beta "beta"<td>0.5<td>::SCS_BETA_DEFAULT
     * <tr><td>\ref ScsSettings#sigma "sigma"<td>0.01<td>::SCS_SIGMA_DEFAULT
     * <tr><td>\ref ScsSettings#direction "direction"<td>\ref anderson_acceleration "anderson_acceleration"
     *     <td>::SCS_DIRECTION_DEFAULT
     * <tr><td>\ref ScsSettings#thetabar "thetabar"<td>0.1<td>::SCS_THETABAR_DEFAULT
     * <tr><td>\ref ScsSettings#memory "memory"<td>5<td>::SCS_MEMORY_DEFAULT
     * <tr><td>\ref ScsSettings#broyden_init_scaling "broyden_init_scaling"<td>1<td>::SCS_BROYDEN_ISCS_SCALE_DEFAULT
     * <tr><td>\ref ScsSettings#do_record_progress "do_record_progress"<td>0<td>::SCS_DO_RECORD_PROGRESS_DEFAULT
     * <tr><td>\ref ScsSettings#do_override_streams "do_override_streams"<td>0<td>::SCS_OVERRIDE_STREAMS_DEFAULT
     * <tr><td>\ref ScsSettings#output_stream "output_stream"<td>\c stdout <td>::SCS_OUT_STREAM_DEFAULT
     * </table>
     * 
     * @param data Pointer to data
     * 
     * \warning If you want to increase the maximum number of iteration with respect
     * to the previous run and you have set \ref ScsSettings#do_record_progress "do_record_progress"
     * to \c 1, then you should not use this function. If you really want to use it, however,
     * you should set the parameter \ref ScsSettings#previous_max_iters "previous_max_iters"
     * to the maximum number of iterations you used in the previous run. This is in 
     * order to avoid memory management errors. 
     * 
     * \warning Alternatively, a simple solution is to invoke ::scs_free_info after you 
     * call ::scs and then again ::scs_init_info. Then it is safe to call this function and
     * run ::scs again.
     * 
     * \note If you have set \ref ScsSettings#do_record_progress "do_record_progress" to \c 0,
     * you may ignore this warning.
     * 
     * \sa #scs_set_restarted_broyden_settings
     * \sa #scs_set_anderson_settings
     * \sa #scs_set_tolerance
     * \sa \ref sec_superscs_config_factory "easy configuration in MATLAB CVX"
     */
    void scs_set_default_settings(ScsData * RESTRICT data);

    /**
     * Calls #scs_set_default_settings and sets the direction to #restarted_broyden
     * and the memory to a specified value.
     * 
     * \note Sets the memory to the maximum between \c 2 and the given memory.
     * \note Sets <code>k0</code> to <code>0</code>
     *  
     * @param data pointer to data
     * @param broyden_memory desired memory length
     * 
     * \sa #scs_set_anderson_settings
     * \sa #scs_set_tolerance
     * \sa #scs_set_default_settings
     */
    void scs_set_restarted_broyden_settings(ScsData * RESTRICT data, scs_int broyden_memory);

    /**
     * Calls #scs_set_default_settings and sets the direction to #anderson_acceleration
     * and the memory to a specified value.
     * 
     * \note Sets the memory to the maximum between \c 2 and the given memory.
     * \note Sets <code>k0</code> to <code>1</code>
     * 
     * @param data pointer to data
     * @param anderson_memory desired memory length
     * 
     * \sa #scs_set_restarted_broyden_settings
     * \sa #scs_set_tolerance
     * \sa #scs_set_default_settings
     */
    void scs_set_anderson_settings(ScsData * RESTRICT data, scs_int anderson_memory);

    /**
     * Sets the tolerance to a given value.
     * 
     * \note If the specified tolerance is lower than 2 times the machine accuracy,
     * the tolerance is set to that minimum value. That is, the minimum allowed 
     * tolerance is <code>10 * DBL_EPSILON</code>.
     * 
     * \warning If the user provides an illegal value such as negative or lower than
     * <code>10 * DBL_EPSILON</code>, the tolerance is set to <code>10 * DBL_EPSILON</code>.
     * 
     * \note Recommended range of tolerances: \c 1e-14 to \c 5e-2.
     * 
     * @param data pointer to data
     * @param tolerance desired tolerance
     */
    void scs_set_tolerance(ScsData * RESTRICT data, scs_float tolerance);
    
    /**
     * Sets the memory for limited-memory quasi-Newtonian direction methods.
     * 
     * \note If \c memory is below \c 2 the memory is set to \c 2. 
     * \note If \c memory is higher than the problem dimension, the memory is 
     *       saturated to exactly the problem dimension. This rule applies to the 
     *       Anderson acceleration method. It is not advisable to use too high 
     *       memory values.
     *  
     * @param data pointer to data
     * @param memory memory length
     * 
     * 
     */
    void scs_set_memory(ScsData * RESTRICT data, scs_int memory);

    /**
     * \brief Frees the memory allocated for a Sol object
     * 
     * @param sol pointer to allocated #ScsSolution structure
     * 
     * \sa scs_init_sol
     */
    void scs_free_sol(ScsSolution * RESTRICT sol);
    
    /**
     * Frees the memory associated with an #ScsData structure.
     * 
     * @param data pointer to data
     * 
     * \sa scs_init_data
     */
    void scs_free_data(ScsData *RESTRICT data);
    
    /**
     * Frees the memory associated with an #ScsCone structure
     * @param cone pointer to cone
     */
    void scs_free_cone(ScsCone *RESTRICT cone);
    /**
     * \brief Frees the memory allocate of a Data and a Cone object
     * @param d pointer to allocated #ScsData structure
     * @param k pointer to allocated #ScsCone structure
     * 
     * \sa scs_set_default_settings
     * \sa scs_init_data
     * \sa scs_free_data
     * \sa scs_free_cone
     */
    void scs_free_data_cone(ScsData * RESTRICT d, ScsCone * RESTRICT k);
    /**
     * \brief Frees the memory allocated for an Info object
     * @param info pointer to allocated #ScsInfo structure
     * 
     * \sa ::scs_init_info
     */
    void scs_free_info(ScsInfo * RESTRICT info);

    /**
     * \brief Custom print function for SCS.
     * 
     * This functions allows to print in different streams. The argument \c print_mode
     * specifies whether it is allowed to override the default stream and use a 
     * print function other than \c printf. 
     * 
     * For example, if SCS is interfaced via a MEX function, MATLAB expects to 
     * use \c printf exclusively which it then delegated to \c mexPrintf; a function
     * that prints the program's output to the console of MATLAB.
     * 
     * When SCS is called from other software, it is likely that \c print_mode 
     * has to be set to 0.
     * 
     * @param print_mode whether to override the default behavior (using \c printf)
     * @param __stream an output stream
     * @param __format string format 
     * @param ... arguments specifying data to print
     * @return return value of \c print or \c vfprintf
     */
    int scs_special_print(scs_int print_mode,
            FILE * RESTRICT __stream,
            const char *RESTRICT __format, ...);

#ifdef __cplusplus
}
#endif
#endif
