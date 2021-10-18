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
// *THIS* example:       https://www.geom.at/boolean-operations/
// Fade2D-Documentation: https://www.geom.at/fade2d/html/
//
// When you use Fade free of charge, please put a link on your
// research homepage.

#include <Fade_2D.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <iomanip>
#include "someTools.h"
using namespace GEOM_FADE2D;

template<class T_InType>
std::string toString(const T_InType& in)
{
	std::ostringstream oss;
	oss << in;
	return oss.str();
}

int ex5_booleanOps_main()
{
	std::cout<<"* Example5: Zone operations - Boolean operations with zones (shapes)\n";
	std::cout<<"* Union\n";
	std::cout<<"* Intersection\n";
	std::cout<<"* Difference\n";
	std::cout<<"* Symmetric Difference\n\n";

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
	generateCircle(numPoints,45.0,25.0,radius,radius,vCircle0);
	generateCircle(numPoints,55.0,25.0,radius,radius,vCircle1);

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

	// * 4 *   Insert the segments as constraint graphs. Note that
	//         the two constraint graphs intersect and that the
	//         intersection point is inserted. The exact intersection
	//         point may not exist in double precision arithmetic, thus
	//         tiny rounding errors can occur.
	ConstraintGraph2* pCG0=dt.createConstraint(vSegments0,CIS_CONSTRAINED_DELAUNAY);
	ConstraintGraph2* pCG1=dt.createConstraint(vSegments1,CIS_CONSTRAINED_DELAUNAY);
	GASSEX(pCG0!=NULL);
	GASSEX(pCG1!=NULL);

	// * 5 *   Create two Zone2 objects using ZL_INSIDE:
	//         Note: Boolean operations require that the two zones
	//         belong to the same Fade_2D object!
	Zone2* pZone0(dt.createZone(pCG0,ZL_INSIDE));
	pZone0->show("example5_zone0.ps",true,true);

	Zone2* pZone1(dt.createZone(pCG1,ZL_INSIDE));
	pZone1->show("example5_zone1.ps",true,true);

	// **************************
	// * 6 *   BOOLEAN OPERATIONS
	// **************************

	//    a) Union operation
	Zone2* pZoneUnion(zoneUnion(pZone0,pZone1));
	pZoneUnion->show("example5_zoneUnion.ps",true,true);

	//    b) Intersection operation
	Zone2* pZoneIntersection(zoneIntersection(pZone0,pZone1));
	pZoneIntersection->show("example5_zoneIntersection.ps",true,true);

	//    c) Difference operation
	Zone2* pZoneDifference(zoneDifference(pZone0,pZone1));
	pZoneDifference->show("example5_zoneDifference.ps",true,true);

	//    d) Symmetric Difference operation
	Zone2* pZoneSymmetricDifference(zoneSymmetricDifference(pZone0,pZone1));
	pZoneSymmetricDifference->show("example5_zoneSymmetricDifference.ps",true,true);


	// * 7 *   Retrieve the (oriented but unordered) border edges of
	//         pZoneSymmetricDifference
	std::vector<Edge2> vBorders;
	pZoneSymmetricDifference->getBorderEdges(vBorders);
	Visualizer2 v("example5_symDiffBorders.ps");
	dt.show(&v,false); // show only the triangles
	v.addObject(vBorders,Color(CGREEN,0.5,true));
	v.writeFile();

	// * 8 *   Turn the border edges into polygons
	std::vector<Edge2> vRemainingEdges;
	std::vector<std::vector<Edge2> > vvEdges;
	edgesToPolygons(vBorders,vvEdges,vRemainingEdges);
	GASSEX(vRemainingEdges.empty()); // All edges belong to polygons

	// * 9 *   Draw
	Visualizer2 polyVis("example5_polygons.ps");
	pZoneSymmetricDifference->show(&polyVis,false,false); // show only the triangles
	for(size_t poly=0;poly<vvEdges.size();++poly)
	{
		const std::vector<Edge2>& vPolygon(vvEdges[poly]);
		Color c(Color::getNextColorName(),1.0,false); // Next color from a (repeating) sequence
		for(size_t j=0;j<vPolygon.size();++j)
		{
			const Edge2& edge(vPolygon[j]);
			polyVis.addObject(edge,c);

			Point2* pSourcePoint(edge.getSrc());
			Label lab(*pSourcePoint,toString(j).c_str(),true,16);
			polyVis.addObject(lab,c);
		}
	}
	polyVis.writeFile();


	return 0;
}
