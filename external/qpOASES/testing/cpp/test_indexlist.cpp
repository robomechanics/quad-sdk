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
 *	\file testing/cpp/test_indexlist.cpp
 *	\author Andreas Potschka, Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2010-2017
 *
 *	Unit test for Indexlist class.
 */



#include <qpOASES.hpp>
#include <qpOASES/UnitTesting.hpp>


/** Test Indexlist sorting */
int main()
{
	USING_NAMESPACE_QPOASES

	Indexlist il(10);
	int_t i, *numbers;

	il.addNumber(1);
	il.addNumber(3);
	il.addNumber(5);
	il.addNumber(2);
	il.addNumber(4);
	il.addNumber(0);
	il.addNumber(7);
	il.addNumber(6);
	il.addNumber(8);
	il.addNumber(9);

	il.getNumberArray(&numbers);
	fprintf(stdFile, "Unsorted numbers: ");
	for (i = 0; i < 10; i++)
		fprintf(stdFile, " %2d", (int)(numbers[i]) );
	fprintf(stdFile, "\n");

	fprintf(stdFile, "Unsorted index of number 0: %3d\n", (int)(il.getIndex(0)) );

	QPOASES_TEST_FOR_TRUE( il.getIndex(0) == 5 )

	il.removeNumber(5);
	fprintf(stdFile, "Unsorted index of (removed) number 5: %3d\n", (int)(il.getIndex(5)) );

	QPOASES_TEST_FOR_TRUE( il.getIndex(5) == -1 )

	il.getNumberArray(&numbers);
	fprintf(stdFile, "Unsorted numbers: ");
	for (i = 0; i < 9; i++)
		fprintf(stdFile, " %2d", (int)(numbers[i]) );
	fprintf(stdFile, "\n");

	il.swapNumbers(2, 7);

	il.getNumberArray(&numbers);
	fprintf(stdFile, "Unsorted numbers: ");
	for (i = 0; i < 9; i++)
		fprintf(stdFile, " %2d", (int)(numbers[i]) );
	fprintf(stdFile, "\n");

	QPOASES_TEST_FOR_TRUE( numbers[2] == 7 )

	return TEST_PASSED;
}


/*
 *	end of file
 */
