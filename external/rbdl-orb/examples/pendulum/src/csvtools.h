/* -------------------------------------------------------------------------- *
 *                           csvtools.h                                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Matthew Millard                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <iostream>
#include <sstream>
#include <string>
#include <fstream> 
#include <vector>
#include <stdlib.h>

/**
This function will print cvs file of the matrix 
 data

@params data: A vector of state vectors
@params filename: The name of the file to print
*/
void printMatrixToFile( const std::vector<std::vector<double > >& dataMatrix, 
                        const std::string& header, 
                        const std::string& filename);
                        
/**
This function will read in a cvs file assuming that all entries are numbers - 
that is there is no header.

@params data: A matrix of data
@params filename: The name of the file to print
*/
std::vector<std::vector<double > > readMatrixFromFile(
                                    const std::string& filename);                        