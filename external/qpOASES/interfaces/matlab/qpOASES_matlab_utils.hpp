/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2017 by Hans Joachim Ferreau, Andreas Potschka,
 *	Christian Kirches et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file interfaces/matlab/qpOASES_matlab_utils.hpp
 *	\author Hans Joachim Ferreau, Alexander Buchner
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Collects utility functions for Interface to Matlab(R) that
 *	enables to call qpOASES as a MEX function.
 *
 */



/* Work-around for settings where mexErrMsgTxt causes unexpected behaviour. */
#ifdef __AVOID_MEXERRMSGTXT__
	#define myMexErrMsgTxt( TEXT ) mexPrintf( "%s\n\n",(TEXT) );
#else
	#define myMexErrMsgTxt mexErrMsgTxt
#endif


/* Workaround for problem on Matlab 2012b
 * see https://github.com/robotology/codyco-superbuild/issues/84
 * see http://stackoverflow.com/questions/22440523/mex-files-using-xcode-5-1-under-os-x-10-9-with-matlab-2012a/22705789#22705789 */
#ifdef __APPLE__
	#include <typeinfo>
#endif 

#include "mex.h"
#include "matrix.h"
#include "string.h"
#include <vector>


/*
 * QProblem instance class
 */
class QPInstance
{
	private:
		static int_t s_nexthandle;

	public:
		QPInstance(	uint_t _nV = 0,
					uint_t _nC = 0,
					HessianType _hessianType = HST_UNKNOWN,
					BooleanType _isSimplyBounded = BT_FALSE,
					BooleanType _sparseLA = BT_FALSE
					);

		~QPInstance( );
	
		returnValue deleteQPMatrices();
		
		int_t getNV() const;
		int_t getNC() const;

		int_t handle;

		SQProblem* sqp;
		QProblemB* qpb;
		BooleanType isSimplyBounded;
		BooleanType sparseLA;

		SymmetricMatrix* H;
		Matrix* A;
		sparse_int_t* Hir; 
		sparse_int_t* Hjc; 
		sparse_int_t* Air; 
		sparse_int_t* Ajc;
		real_t* Hv;
		real_t* Av;
};


/*
 *	end of file
 */
