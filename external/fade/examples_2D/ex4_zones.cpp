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
// *THIS* example:       https://www.geom.at/example4-zones-defined-areas-in-triangulations/
// Fade2D-Documentation: https://www.geom.at/fade2d/html/
//
// When you use Fade free of charge, please put a link on your
// research homepage.
#include <Fade_2D.h>
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>
#include "someTools.h"

using namespace GEOM_FADE2D;







int ex4_zones_main()
{
	std::cout<<"* Example4: Zones - Defines zones in different ways"<<std::endl;
	std::cout<<"  0) area inside a ConstraintGraph2"<<std::endl;
	std::cout<<"  1) area outside a ConstraintGraph2"<<std::endl;
	std::cout<<"  2) area grown from a seed point"<<std::endl;
	std::cout<<"  3) global (all triangles)"<<std::endl;
	std::cout<<"  4) area of arbitrary triangles\n"<<std::endl;

	// * 1 *   Insert 4 bounding box points
	Fade_2D dt;
	dt.insert(Point2(0,0));
	dt.insert(Point2(+100,0));
	dt.insert(Point2(100,50));
	dt.insert(Point2(0,50));

	// * 2 *   Create points on two circles
	std::vector<Point2> vCircle0;
	std::vector<Point2> vCircle1;
	int numPoints(8);
	double radius(22);
	//generateCircle( num,centerX,centerY,radiusX,radiusY,vOut);
	generateCircle(numPoints,25.0,25.0,radius,radius,vCircle0);
	generateCircle(numPoints,75.0,25.0,radius,radius,vCircle1);

	// * 3 *   Create segments
	std::vector<Segment2> vSegments0;
	std::vector<Segment2> vSegments1;
	for(int i=0;i<numPoints;++i)
	{
		Segment2 seg0(vCircle0[i],vCircle0[(i+1)%numPoints]);
		Segment2 seg1(vCircle1[i],vCircle1[(i+1)%numPoints]);
		vSegments0.push_back(seg0);
		vSegments1.push_back(seg1);
	}

	// * 4 *   Insert the segments as constraint graphs
	ConstraintGraph2* pCG0=dt.createConstraint(vSegments0,CIS_CONSTRAINED_DELAUNAY);
	ConstraintGraph2* pCG1=dt.createConstraint(vSegments1,CIS_CONSTRAINED_DELAUNAY);

	// * 5 *  Visualize
	dt.show("example4_constraints.ps",true);

	// Verify: pCG0 and pCG1 must be closed (GASSEX is defined in "someTools.h")
	GASSEX(pCG0->isPolygon());
	GASSEX(pCG1->isPolygon());

	// * 6 *   Create Zone2 objects in different ways, then retrieve the
	//    triangles and visualize the zones

	// + Zone inside pCG0:
	Zone2* pZoneInside(dt.createZone(pCG0,ZL_INSIDE));
	pZoneInside->show("example4_zoneInside.ps",true,true); // all triangles=true, constraints=true

	// + Zone outside pCG0:
	Zone2* pZoneOutside(dt.createZone(pCG0,ZL_OUTSIDE));
	pZoneOutside->show("example4_zoneOutside.ps",true,true);

	// + Zone grown from a seed-point, the growing stops at the
	//   edges of the specified ConstraintGraph2 objects
	std::vector<ConstraintGraph2*> vCG;
	vCG.push_back(pCG0);
	vCG.push_back(pCG1);
	Point2 seedPoint(5.0,5.0); // Point near the lower left corner
	Zone2* pZoneGrow(dt.createZone(vCG,ZL_GROW,seedPoint));
	pZoneGrow->show("example4_zoneGrow.ps",true,true);

	// + Global zone (all triangles):
	Zone2* pZoneGlobal(dt.createZone(NULL,ZL_GLOBAL));
	pZoneGlobal->show("example4_zoneGlobal.ps",true,true);

	// + Zone defined by specific triangles
	std::vector<Triangle2*> vT;
	dt.getTrianglePointers(vT);
	vT.resize(vT.size()/2);
	Zone2* pZoneRandomTriangles(dt.createZone(vT));
	pZoneRandomTriangles->show("example4_zoneFromTriangles.ps",true,true);
	return 0;
}


