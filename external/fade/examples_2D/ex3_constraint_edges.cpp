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
// *THIS* example:       https://www.geom.at/example3-constraint-edges/
// Fade2D-Documentation: https://www.geom.at/fade2d/html/
//
// When you use Fade free of charge, please put a link on your
// research homepage.

#include <Fade_2D.h>
#include <stdlib.h>
#include <stdio.h>
using namespace GEOM_FADE2D;
using namespace std;


int ex3_constraintEdges_main()
{
	std::cout<<"Example3: Constraints - Enforce constraint edges\n";

	// 1) Generate some input points
	std::vector<Point2> vInputPoints;
	vInputPoints.push_back(Point2(-100,-100));
	vInputPoints.push_back(Point2(+100,+100));
	vInputPoints.push_back(Point2(-50,-70));
	vInputPoints.push_back(Point2(-50,-30));
	vInputPoints.push_back(Point2(50,70));
	vInputPoints.push_back(Point2(50,30));

	// 2) Triangulate the points and show
	Fade_2D dt;
	dt.insert(vInputPoints);
	dt.show("example3_noConstraints.ps",true);

	// 3) Prepare a vector of one or more edges to be enforced
	std::vector<Segment2> vSegments;
	vSegments.push_back(Segment2(vInputPoints[0],vInputPoints[1]));

	// 4) Insert the Constraint Segments
	// Use always CIS_CONSTRAINED_DELAUNAY. This strategy does not
	// subdivide the constraint segments except when they intersect
	// an existing vertex or another constraint segment.
	// Other insertion strategies exist also but they are deprecated
	// and only kept for backwards compatibility. Their behavior is
	// perfectly replaced by fast and reliable methods like the
	// functions ConstraintGraph2::makeDelaunay() and Fade_2D::drape().
	ConstraintGraph2* pCG=dt.createConstraint(vSegments,CIS_CONSTRAINED_DELAUNAY);
	dt.show("example3_withConstraints.ps",true);

	// 5) makeDelaunay() - an optional function call to subdivide a
	// ConstraintGraph in order to achieve better shaped triangles.
	// Segments smaller than $minLen are not subdivided. This parameter
	// is thought to prevent excessive subdivision in narrow settings.
	double minLen(0.1);
	pCG->makeDelaunay(minLen);
	dt.show("example3_makeDelaunay.ps",true);


	return 0;
}



