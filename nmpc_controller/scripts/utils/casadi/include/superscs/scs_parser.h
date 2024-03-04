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
#ifndef SCS_PARSER_H_GUARD
#define SCS_PARSER_H_GUARD

#if !(defined _WIN32 || defined _WIN64 || defined _WINDLL || defined __APPLE__)
#ifdef _POSIX_C_SOURCE
#undef _POSIX_C_SOURCE
#endif
#define _POSIX_C_SOURCE 199309L
#endif

#include "glbopts.h"
#include <string.h>
#include "cones.h"
#include "linAlg.h"
#include "linSys.h"
#include "util.h"
#include "ctrlc.h"
#include "constants.h"
#include <math.h>
#include <stdio.h>
#include <time.h>
#include "scs.h"
#include "linsys/amatrix.h"

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * Length of character arrays in #ScsConicProblemMetadata.
     */
#define SCS_METADATA_TEXT_SIZE 1024

    /**
     * \brief Metadata for conic optimization problems.
     * 
     * All fields of this structure are character arrays of a fixed length, 
     * equal to #SCS_METADATA_TEXT_SIZE.
     * 
     * \sa scs_to_YAML
     * \sa scs_from_YAML
     */
    struct scs_conic_probem_metadata {
        /**
         * \brief Unique identifier of the conic problem.
         * 
         * This can be, for example, a URI.
         */
        char id[SCS_METADATA_TEXT_SIZE];
        /**
         * \brief Problem name.
         */
        char problemName[SCS_METADATA_TEXT_SIZE];
        /**
         * \brief License of the problem data.
         * 
         * If applicable, link (URL) to a license.
         */
        char license[SCS_METADATA_TEXT_SIZE];
        /**
         * \brief Creator of the problem.
         */
        char creator[SCS_METADATA_TEXT_SIZE];
        /**
         * \brief YAML version.
         */
        char yamlVersion[SCS_METADATA_TEXT_SIZE];
        /**
         * \brief Creation date.
         */
        char date[SCS_METADATA_TEXT_SIZE];
    };
    
    typedef struct scs_conic_probem_metadata ScsConicProblemMetadata;

    /**
     * \brief Initializes a #ScsConicProblemMetadata structure.
     * 
     * This function creates and initializes a #ScsConicProblemMetadata structure. 
     * It sets the problem name to a given value and initializes all other
     * fields with their default values as follows:
     * 
     * - \ref ScsConicProblemMetadata.id "id": URI of the problem which is <code>%http://superscs.org/problem/{problemName}</code>
     * - \ref ScsConicProblemMetadata.date "date": current date
     * - \ref ScsConicProblemMetadata.license "license": URL of <a href="https://github.com/kul-forbes/scs/blob/master/LICENSE.txt">SuperSCS's license</a>
     * - \ref ScsConicProblemMetadata.yamlVersion "yamlVersion": 1.2
     * - \ref ScsConicProblemMetadata.creator "creator": the \ref #scs_version "current SuperSCS version"
     * 
     * @param problemName problem name
     * @return New instance of #ScsConicProblemMetadata
     * 
     * \sa #scs_to_YAML
     */
    ScsConicProblemMetadata * scs_init_conic_problem_metadata(const char * problemName);


    /**
     * Parses a YAML file and constructs/initialises the corresponding #ScsData and #ScsCone
     * objects.
     * 
     * Example of use:
     * 
     * ~~~~~
     * ScsData * data;
     * ScsCone * cone;
     * const char * filepath = "matlab/scs-yaml/example.yml";
     * int status = scs_from_YAML(filepath, &data, &cone);
     * if (status != 0) { 
     *  // handle failure
     * }
     * // use `data` and `cone` ...
     * // at the end don't forget to call `scs_free_data_cone`
     * scs_free_data_cone(data, cone);
     * ~~~~~
     * 
     * @param filepath Absolute or relative path to YAML file
     * @param data pointer-to-pointer to a #ScsData object. This function will 
     * initialise `data` using the YAML file.
     * @param cone pointer-to-pointer to a #ScsCone object. This function will 
     * initialise `cone` using the YAML file.
     * @return status code; returns \c 0 if parsing has succeeded; a positive
     * error code otherwise. 
     * 
     * \sa #scs_to_YAML
     * \sa \ref page_save_load "Saving and loading problems" (detailed documentation)
     */
    scs_int scs_from_YAML(const char * filepath,
            ScsData ** data,
            ScsCone ** cone);

    /**
     * 
     * @param filepath relative or absolute path to a file which this function 
     * will create. 
     * 
     * The caller must have the necessary permissions to create the
     * file, otherwise the method returns the error code \c 101.
     * 
     * @param metadata problem metadata which can be created using #scs_init_conic_problem_metadata
     * @param data pointer to exisint non-null #ScsData object
     * @param cone pointer to exisint non-null #ScsCone object
     * @return this function returns \c 0 on success and a positive status code
     * otherwise.
     * 
     * \sa #scs_from_YAML
     * \sa ScsConicProblemMetadata
     * \sa \ref page_save_load "Saving and loading problems" (detailed documentation)
     */
    scs_int scs_to_YAML(
            const char * RESTRICT filepath,
            ScsConicProblemMetadata * metadata,
            const ScsData * RESTRICT data,
            const ScsCone * RESTRICT cone);



#ifdef __cplusplus
}
#endif
#endif
