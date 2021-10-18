// (c) 2010 Geom e.U. Bernhard Kornberger, Graz/Austria.
//
// This file is part of the Fade2D library. You can use it for your
// personal non-commercial research. Licensees holding a commercial
// license may use this file in accordance with the Commercial
// License Agreement provided with the Software.
//
// This software is provided AS IS with NO WARRANTY OF ANY KIND,
// INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE.
//
// Contact: https://www.geom.at/contact/
//
// *THIS* example:       https://www.geom.at/fade-delaunay-triangulation/
// Fade2D-Documentation: https://www.geom.at/fade2d/html/
//
// When you use Fade free of charge, please put a link on your
// research homepage.

#include <Fade_2D.h>
#include <stdio.h>

using namespace GEOM_FADE2D;

int ex0_hello_main()
{
	std::cout<<"* Example0:";
	std::cout<<"* Triangulate 4 points\n";
	std::cout<<"* Visualize the result\n\n";

	// Create a triangulation
	Fade_2D dt;

	// Create and insert 4 points
	Point2 p0(0.0,0.0);
	Point2 p1(0.0,1.0);
	Point2 p3(2,0.5);
	Point2 p2(0.5,0.5);
	dt.insert(p0);
	dt.insert(p1);
	dt.insert(p2);
	dt.insert(p3);

	// Draw
	dt.show("example0.ps");
	return 0;
}

