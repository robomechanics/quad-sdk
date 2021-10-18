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
// *THIS* example:       https://www.geom.at/example2-traversing/
// Fade2D-Documentation: https://www.geom.at/fade2d/html/
//
// When you use Fade free of charge, please put a link on your
// research homepage.

#include <Fade_2D.h>
#include <stdio.h>
#include "someTools.h"
using namespace GEOM_FADE2D;
using namespace std;

// Prototypes
void manualDraw(Fade_2D& dt);
void moreDraw();


int ex2_accessDraw_main()
{
	std::cout<<"* Example2: Access elements of a triangulation\n";

	// * 1 *   Create points on a circle
	std::vector<Point2> vPoints;
	int numPoints(6);
	double centerX(5),centerY(5),radiusX(5),radiusY(5);
	generateCircle( numPoints,centerX,centerY,radiusX,radiusY,vPoints);
	vPoints.push_back(Point2(centerX,centerY)); // Add the center point

	// * 2 *   Optional step: You can add custom indices to relate
	//         the points to your own data structures. But you do
	//         not need to use this feature.
	int myStartIndex(77); // An arbitrary value
	for(size_t i=0;i<vPoints.size();++i)
	{
		vPoints[i].setCustomIndex(myStartIndex++);
	}

	// * 3 *   Insert the vertices
	Fade_2D dt; // The Delaunay triangulation
	std::vector<Point2*> vVertexHandles; // Pointers to the vertices inside Fade
	dt.insert(vPoints,vVertexHandles); // Fastest method: insert all points at once

	// * 4 *   Draw the triangulation using a ready-made function
	dt.show("example2_triangulation.ps");

	// * 5 *   Manual draw (to demonstrate access to the elements)
	manualDraw(dt);

	// * 6 *   More drawing examples using the Visualizer2 class
	moreDraw();

	return 0;
}





// Manual visualization of a triangulation. Note: This can also be
// done with the Fade_2D::show() function but the goal here is also
// to demonstrate how to access the elements of a triangulation.
void manualDraw(Fade_2D& dt)
{
	// * 1 *   Create a postscript visualizer and define some colors
	Visualizer2 vis("example2_manualDraw.ps");
	Color cBlack(CBLACK);
	Color cBlue(CBLUE);
	Color cRed(CRED);
	Color cPurple(CPURPLE);
	vis.addObject(Label(Point2(1.5,12.4),"BLACK: triangle edges",false,15),cBlack);
	vis.addObject(Label(Point2(1.5,11.6),"RED: neighbor-triangle labels",false,15),cRed);
	vis.addObject(Label(Point2(1.5,10.8),"BLUE: intra-triangle-indices",false,15),cBlue);
	vis.addObject(Label(Point2(1.5,10.0),"PURPLE: custom vertex indices",false,15),cPurple);


	// * 2 *   Get and draw the vertices with their custom index
	std::vector<Point2*> vAllPoints;
	dt.getVertexPointers(vAllPoints);
	std::cout<<"vAllPoints.size()="<<vAllPoints.size()<<std::endl;
	for(std::vector<Point2*>::iterator it(vAllPoints.begin());it!=vAllPoints.end();++it)
	{
		Point2* currentPoint(*it);
		int customIndex(currentPoint->getCustomIndex());
		std::string text(toString(customIndex));
		vis.addObject(Label(*currentPoint,text.c_str(),true,12),cPurple);
	}

	// * 3 *   Get and draw the triangles
	std::vector<Triangle2*> vAllDelaunayTriangles;
	dt.getTrianglePointers(vAllDelaunayTriangles);
	for(std::vector<Triangle2*>::iterator it=vAllDelaunayTriangles.begin();it!=vAllDelaunayTriangles.end();++it)
	{
		Triangle2* pT(*it);
		vis.addObject(*pT,cBlack);

		// An alternative (just to show how to access the vertices) would be:
		//Point2* p0=pT->getCorner(0);
		//Point2* p1=pT->getCorner(1);
		//Point2* p2=pT->getCorner(2);
		//vis.addObject(Segment2(*p0,*p1),cBlack);
		//vis.addObject(Segment2(*p1,*p2),cBlack);
		//vis.addObject(Segment2(*p2,*p0),cBlack);
	}

	// * 4 *   Choose one triangle and color it green
	Triangle2* pT(vAllDelaunayTriangles[0]);
	Color cGreenFill(CPALEGREEN,0.001f,true);
	vis.addObject(*pT,cGreenFill);

	// * 5 *   The corners of pT can be accessed through the so called intra-
	// triangle-indices 0,1,2. They are counterclockwise oriented (CCW).
	// Let's write intra-triangle-index labels beside the corners.
	for(int intraTriangleIndex=0;intraTriangleIndex<3;++intraTriangleIndex)
	{
		Point2* pCorner(pT->getCorner(intraTriangleIndex));
		std::string text("\nidx="+toString(intraTriangleIndex));
		vis.addObject(Label(*pCorner,text.c_str(),true,15),cBlue);
	}

	// * 6 *   Each triangle has three neighbor triangles (or NULL
	// pointers at border edges). They are accessed through the
	// intra-triangle-indices. The i'th opposite triangle of pT is
	// the one that is opposite to the i'th vertex. Let's draw that:
	Label label_pT(pT->getBarycenter()," pT",true,15);
	vis.addObject(label_pT,cBlue);
	for(int intraTriangleIndex=0;intraTriangleIndex<3;++intraTriangleIndex)
	{
		Triangle2* pNeigT(pT->getOppositeTriangle(intraTriangleIndex));
		if(pNeigT==NULL) continue; // No adjacent triangle at this edge

		// Compute the barycenter and write a label there
		Point2 barycenter(pNeigT->getBarycenter());
		std::string text(" =pT->getOppositeTriangle("+toString(intraTriangleIndex)+")");
		Label neigLabel(barycenter,text.c_str(),true,15);
		vis.addObject(neigLabel,cRed);
	}

	// Write the postscript file to disk
	vis.writeFile();
}

// More examples that demonstrate the use of the Visualizer2 class
void moreDraw()
{
	// * 1 *   Defining color
	// Define red by values (red,green,blue,linewidth,bFill)
	Color cRed(1,0,0,0.001,false);
	// Or simply use a color name and the defaults linewidth=0.001 and bFill=false
	Color cBlue(CBLUE);
	Color cGreen(CGREEN);
	Color cPurple(CPURPLE);
	// Specify a bolt green color (linewidth=1.0) and another one with
	// bFill=true to fill the area of an object which is drawn using
	// that color. In case that a segment is drawn with bFill=true,
	// marks for its endpoints are added.
	Color cGreenBolt(CGREEN,1.0,false);
	Color cGreenFill(CGREEN,.001f,true);

	// Postscript writer
	Visualizer2 vis("example2_moreDraw.ps");

	// Create a label and add it to the Visualizer
	Label headLabel(Point2(20,60),	// Position
					"This is the Fade2D\nPostscript Visualizer",
					false,			// Don't write an x-mark
					30);			// Font size
	vis.addObject(headLabel,cRed); 	// Add the label

	// Add a row of points
	for(int x=0;x<100;x+=5)
	{
		Point2 p(x,50);
		vis.addObject(p,cPurple); // Add the point
	}

	// Draw a segment with cGreen
	Point2 p0(0,0);
	Point2 p1(100,0);
	Segment2 seg0(p0,p1);
	vis.addObject(seg0,cGreen); // Add the segment

	// Draw a segment with cGreenFill (with marks at the endpoints)
	Point2 p2(0,10);
	Point2 p3(100,10);
	Segment2 seg1(p2,p3);
	vis.addObject(seg1,cGreenFill); // Add the segment

	// Draw a segment with cGreenBolt
	Point2 p4(0,20);
	Point2 p5(100,20);
	Segment2 seg2(p4,p5);
	vis.addObject(seg2,cGreenBolt); // Add the segment

	// Draw labels
	Label lab0(Point2(20,2),"Segment in cGreen",false,15);
	vis.addObject(lab0,cBlue); // Add the label

	Label lab1(Point2(20,12),"Segment in cGreenFill",false,15);
	vis.addObject(lab1,cBlue); // Add the label

	Label lab2(Point2(20,22),"Segment in cGreenBolt",false,15);
	vis.addObject(lab2,cBlue); // Add the label

	// Add three circles with radius=4 (squared radius 16)
	double sqRadius(4.0*4.0);
	Circle2 circ0(50,40,sqRadius);
	Circle2 circ1(60,40,sqRadius);
	Circle2 circ2(70,40,sqRadius);
	vis.addObject(circ0,cGreen);	// Add the circle
	vis.addObject(circ1,cGreenBolt);// Add the circle
	vis.addObject(circ2,cGreenFill);// Add the circle

	// The postscript file is only written when writeFile() is called.
	vis.writeFile();
}
