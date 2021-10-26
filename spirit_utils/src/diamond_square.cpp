#include "spirit_utils/diamond_square.h"

#include <stdlib.h>
#include <time.h>
#include <unistd.h>

DiamondSquare::DiamondSquare(double** array, int s, double r)
{
	map = array;
	size = s;
	range = r;
}

DiamondSquare::~DiamondSquare()
{
	for(int i = 0; i < size; i++)
	{
		delete map[i];
	}
	delete map;
}

double** DiamondSquare::process()
{
	_on_start();

	for(int sideLength = size-1; sideLength >= 2; sideLength /= 2, range /= 2)
	{
		int halfSide = sideLength/2;

		squareStep(sideLength, halfSide);
		diamondStep(sideLength, halfSide);
	}
	_on_end();

	return map;
}

/**
 * Performs the diamond step on the map.
 */
void DiamondSquare::diamondStep(int sideLength, int halfSide)
{
	for(int x = 0; x < size-1; x += halfSide)
	{
		for(int y = (x+halfSide)%sideLength; y < size -1; y +=sideLength)
		{
			double avg = map[(x-halfSide+size-1)%(size-1)][y] +
				map[(x+halfSide)%(size-1)][y] +
				map[x][(y+halfSide)%(size-1)] +
				map[x][(y-halfSide+size-1)%(size-1)];
			avg /= 4.0 + dRand(-range, range);
			map[x][y] = avg;

			if(x == 0) map[size-1][y] = avg;
			if(y == 0) map[x][size-1] = avg;
		}
	}
}

/**
 * Performs the square step on the map.
 */
void DiamondSquare::squareStep(int sideLength, int halfSide)
{
	for(int x = 0; x < size-1; x += sideLength)
	{
		for(int y = 0; y < size-1; y += sideLength)
		{
			double avg = map[x][y] + map[x+sideLength][y] + map[x][y+sideLength] + map[x+sideLength][y+sideLength];
			avg /= 4.0;
			map[x+halfSide][y+halfSide] = avg + dRand(-range, range);
		}
	}
}

void DiamondSquare::_on_start()
{
	// Defining the corners values :
	map[0][0] = map[0][size-1] = map[size-1][0] = map[size-1][size-1] = 0;
	// Initializing srand for random values :
	srand(time(NULL));
}

void DiamondSquare::_on_end()
{
}

double DiamondSquare::dRand(double dMin, double dMax)
{
	double d = (double)rand() / RAND_MAX;
	return dMin + d * (dMax - dMin);
}

