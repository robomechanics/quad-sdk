/* -------------------------------------------------------------------------- *
 *                           csvtools.cpp                                     *
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
#include "csvtools.h"
 

std::vector<std::vector<double > > readMatrixFromFile(
                                    const std::string& filename,
                                    int startingRow)
{
    std::ifstream datafile;
	datafile.open(filename.c_str());       
    
    
    //SimTK::Matrix data;
    std::vector<std::vector<double > > dataMatrix;
    std::vector<double > rowVector;

    if(datafile.is_open())
    {
        std::string line;
        std::string entry;
        int row = 0;
        int col = 0;
        //int matrixRowNum = 0;
        int matrixColNum = 1;

        //1. Size the matrix


        getline(datafile,line); //get a line of text
       
        while(row < startingRow){
            getline(datafile,line);
            row++;
        }

        //parse it for all of the comma spots
        unsigned pos1 = 0;
        unsigned pos2 = 0;
        do{                    
            pos2 = line.find(",",pos1);
            //if this is the first time running this loop, count
            //the number of columns
            if(pos2 >= 0 && pos2 < line.length() && row == 0)
                matrixColNum++;

            pos1 = pos2+1;
        }while(pos2 >= 0 && pos2 < line.length());

        //Initial matrix sizing
        if(row == 0){
            //matrixRowNum = max(matrixColNum,20);
            rowVector.resize(matrixColNum);
            //dataMatrix.resize(matrixRowNum);
            //data.resizeKeep(matrixRowNum, matrixColNum);
        }


        while(datafile.good())
        {
            pos1 = 0;
            pos2 = 0;          
            for(int i=0; i < matrixColNum; i++){
                pos2 = line.find(",",pos1);
                if(pos2 < 0 && pos2 >= line.length())
                    pos2 = line.length();
                entry = line.substr(pos1,pos2-pos1);
                pos1 = pos2+1;
                //data(row,i) = atof(entry.c_str());
                rowVector[i] = atof(entry.c_str());
            }

            //Resize the matrix if its too small for the next line
            //if(row == matrixRowNum-1){
            //    matrixRowNum = matrixRowNum*2;
            //    data.resizeKeep(matrixRowNum,matrixColNum);
            //}
            dataMatrix.push_back(rowVector);

            row++;
            getline(datafile,line);
        }
            //data.resizeKeep(row,matrixColNum);
        
    }
	datafile.close();
    return dataMatrix;
}

void printMatrixToFile( 
    const std::vector<std::vector<double > >& dataMatrix, 
    const std::string& header, 
    const std::string& filename)
{

    std::ofstream datafile;
	datafile.open(filename.c_str());
    datafile << std::scientific;
    datafile.precision(16);
    if(header.length() > 1)
        datafile << header << "\n";

	for(int i = 0; i < dataMatrix.size(); i++){
		for(int j = 0; j < dataMatrix[0].size(); j++){
			if(j<dataMatrix[0].size()-1){
				datafile << dataMatrix[i][j] << ", ";
            }
			else{
                datafile << dataMatrix[i][j] << "\n";
            }            
		}	
	}
	datafile.close();
}

void printMatrixToFile(
    const std::vector<std::vector< int > >& dataMatrix,
    const std::string& header,
    const std::string& filename)
{

    std::ofstream datafile;
    datafile.open(filename.c_str());
    //datafile << std::scientific;
    //datafile.precision(16);
    if(header.length() > 1)
        datafile << header << "\n";

    for(int i = 0; i < dataMatrix.size(); i++){
        for(int j = 0; j < dataMatrix[0].size(); j++){
            if(j<dataMatrix[0].size()-1){
                datafile << dataMatrix[i][j] << ", ";
            }
            else{
                datafile << dataMatrix[i][j] << "\n";
            }
        }
    }
    datafile.close();
}
