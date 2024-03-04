/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Pantelis Sopasakis (https://alphaville.github.io),
 *                    Krina Menounou (https://www.linkedin.com/in/krinamenounou), 
 *                    Panagiotis Patrinos (http://homes.esat.kuleuven.be/~ppatrino)
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
#ifndef SCS_DIRECTIONS_H
#define SCS_DIRECTIONS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "scs.h"

    /**
     * The cache has been incremented.
     */
#define SCS_DIRECTION_CACHE_INCREMENT 101
    /** 
     * The cursor of the cache has been reset to \c 0.
     */
#define SCS_DIRECTION_CACHE_RESET 100
    /**
     * The direction could not be computed due to an error.
     */
#define SCS_DIRECTION_ERROR -1
    /**
     * The direction was computed successfully.
     * 
     * All nonnegative status codes denote success.
     */
#define SCS_DIRECTION_SUCCESS 0

    /**
     * Resets the cache. This methods does not free the memory allocated by the 
     * cache, nor does it overwrite the previously cached data. It simply sets the 
     * parameters `mem_current` and `mem_idx` to 0.
     * 
     * @param cache the cache to be reset 
     * 
     * @return status code (returns #SCS_DIRECTION_CACHE_RESET)
     */
    scs_int scs_reset_direction_cache(ScsDirectionCache * cache);

    /**
     * Restarted Broyden (as it is reported in the paper).
     * 
     * @param work Work structure with all available information about the current
     * iteration (current FPR, values of \f$s_k\f$, \f$y_k\f$ etc).
     * 
     * @return status code of the method.
     * 
     * @see \ref sec-restarted-broyden "Restarted Broyden Algorithm"
     */
    scs_int scs_compute_dir_restarted_broyden(ScsWork *work);
    
    /**
     * Anderson's acceleration
     * 
     * @param work Work structure with all available information about the current
     * iteration (current FPR, values of \f$s_k\f$, \f$y_k\f$ etc).
     * 
     * @return status code of the method.
     */
    scs_int scs_compute_dir_anderson(ScsWork *work);

    /**
     * Full Broyden method.
     * 
     * @param work
     * @param i
     * 
     * @return status code of the method.
     * 
     * @see \ref sec-full-broyden "Full Broyden Algorithm"
     * 
     * \warning Not implemented yet
     */
    scs_int scs_compute_dir_full_broyden(ScsWork *work, scs_int i);

    /**
     * Frees memory allocated for the full Broyden method.
     */
    void scs_free_full_broyden(void);

    /**
     * Computes a direction according to the value of 
     * <code>work->stgs->direction</code>.
     * 
     * @param work workspace structure
     * @param i iteration count
     * @return status code; negative status corresponds to error. 
     * 
     * @see ScsDirectionType
     */
    scs_int scs_compute_direction(ScsWork *work, scs_int i);


#ifdef __cplusplus
}
#endif

#endif /* DIRECTIONS_H */

