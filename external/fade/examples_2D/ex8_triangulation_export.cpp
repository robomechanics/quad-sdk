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
// *THIS* example:       https://www.geom.at/triangulation-export/
// Fade2D-Documentation: https://www.geom.at/fade2d/html/
//
// When you use Fade free of charge, please put a link on your
// research homepage.

#include <Fade_2D.h>
#include <stdio.h>
#include "someTools.h"

using namespace GEOM_FADE2D;

void triangulationExport(Fade_2D& dt);
void zoneExport(Zone2* pZone);
void checkData(FadeExport& fadeExport,const char* name);


// main()
int ex8_triangulationExport_main()
{
	std::cout<<"* Example 8: Export"<<std::endl;

	// * 1 *   Create some points
	std::vector<Point2> vPoints;
	generateCircle(6,25.0,25.0,20.0,20.0,vPoints);
	for(size_t i=0;i<vPoints.size();++i)
	{
		// Optional: Set an arbitrary custom index
		vPoints[i].setCustomIndex(int(i));
	}


	// * 2 *   Triangulate
	Fade_2D dt;
	dt.insert(vPoints);


	// * 3 *   Create a zone and export only its triangles
	std::cout<<"\n\n\nExporting a zone..."<<std::endl;
	std::vector<Triangle2*> vT;
	dt.getTrianglePointers(vT);
	vT.resize(2); // Keep only 2 triangles
	Zone2* pZone=dt.createZone(vT);
	zoneExport(pZone);


	// * 4 *   Export the whole triangulation (and delete data from dt)
	std::cout<<"\n\n\nExporting a triangulation..."<<std::endl;
	triangulationExport(dt);

	return 0;

}


void triangulationExport(Fade_2D& dt)
{
	// * 1 *   Export (clears the Fade object)
	FadeExport fadeExport;
	bool bCustomIndices(true); // To retrieve custom indices also
	bool bClear(true); // Clear memory in $dt
	dt.exportTriangulation(fadeExport,bCustomIndices,bClear);

	//////////////////////////////////////////////////////
	// The Fade object has been cleared to avoid memory //
	// peaks. Now demonstrate that we have all data...  //
	//////////////////////////////////////////////////////

	checkData(fadeExport,"example8_triangulationExport.ps");
}

void zoneExport(Zone2* pZone)
{
	FadeExport fadeExport;
	bool bWithCustomIndices(true);
	pZone->exportZone(fadeExport,bWithCustomIndices);

	checkData(fadeExport,"example8_zoneExport.ps");
}


void checkData(FadeExport& fadeExport,const char* name)
{
	// The below Steps 1,2,3 are only there to demonstrate how
	// to access the elements. Instead you can also use the much
	// simpler methods FadeExport::print() and FadeExport::writeObj().

	std::cout<<"\nThis is checkData("<<name<<"):"<<std::endl;

	// * 1 *   Vertex output
	Visualizer2 visExp(name);
	for(int vtxIdx=0;vtxIdx<fadeExport.numPoints;++vtxIdx)
	{
		double x,y;
		fadeExport.getCoordinates(vtxIdx,x,y);

		std::string s(" "+toString(vtxIdx));
		// Are there custom indices? Then add
		if(fadeExport.numCustomIndices==fadeExport.numPoints)
		{
			int customIdx(fadeExport.aCustomIndices[vtxIdx]);
			s.append("(customIdx="+toString(customIdx)+")");
		}
		std::cout<<"Vertex"<<s<<": "<<x<<" "<<y<<std::endl;
		Label label(Point2(x,y),s.c_str(),true,15);
		visExp.addObject(label,Color(CRED));
	}

	// * 2 *   Triangle output
	for(int triIdx=0;triIdx<fadeExport.numTriangles;++triIdx)
	{
		int vtxIdx0,vtxIdx1,vtxIdx2;
		fadeExport.getCornerIndices(triIdx,vtxIdx0,vtxIdx1,vtxIdx2);
		std::cout<<"Triangle "<<triIdx<<": "<<vtxIdx0<<" "<<vtxIdx1<<" "<<vtxIdx2<<std::endl;
		// Fetch also the coordinates and draw the edges
		double x0,y0;
		double x1,y1;
		double x2,y2;
		fadeExport.getCoordinates(vtxIdx0,x0,y0);
		fadeExport.getCoordinates(vtxIdx1,x1,y1);
		fadeExport.getCoordinates(vtxIdx2,x2,y2);
		Point2 p0(x0,y0);
		Point2 p1(x1,y1);
		Point2 p2(x2,y2);
		visExp.addObject(Segment2(p0,p1),Color(CBLACK));
		visExp.addObject(Segment2(p1,p2),Color(CBLACK));
		visExp.addObject(Segment2(p2,p0),Color(CBLACK));
		double midX((x0+x1+x2)/3.0);
		double midY((y0+y1+y2)/3.0);
		std::string text("T"+toString(triIdx));
		Label l(Point2(midX,midY),text.c_str(),true,15);
		visExp.addObject(l,Color(CBLUE));
	}

	// * 3 *   Neighbors output
	std::vector<std::pair<int,int> > vNeigs;
	fadeExport.extractTriangleNeighborships(vNeigs);
	for(size_t i=0;i<vNeigs.size();++i)
	{
		std::cout<<"Triangle "<<vNeigs[i].first<<" <-> Triangle "<<vNeigs[i].second<<std::endl;
	}
	visExp.writeFile(); // Write the postscript file
}

