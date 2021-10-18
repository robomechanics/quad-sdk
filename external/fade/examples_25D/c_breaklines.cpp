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
// *THIS* example:         https://www.geom.at/breakline-insertion/
// Fade2.5D-Documentation: https://www.geom.at/fade25d/html/
//
// When you use Fade free of charge, please put a link on your
// research homepage.

#include <Fade_2D.h>
#include <Visualizer3.h>
#include "someTools.h"



using namespace std;
using namespace GEOM_FADE25D;

// Prototypes for breaklines
int breaklines_main(); // Main function
void terrain_breaklines_noSubdiv_noDraping(); // 0...Breakline insertion (no subdivision, no draping)
void terrain_breaklines_noSubdiv_withDraping(); // 1...Breakline insertion (no subdivision, with draping)
void terrain_breaklines_withSubdiv_noDraping(); // 2...Breakline insertion (with subdivision, no draping)
void terrain_breaklines_withSubdiv_withDraping(); // 3...Breakline insertion (with subdivision, with draping)
void cookieCutter(); // 4...Cut out a piece of a triangulation
void isoContours(); // 5...ISO Contour computation
void heightQueries(); // 6...Get the height z at (x,y)
void createTriangulation(Fade_2D& dt); // Create a triangulation to test with it
void createCircularPolygon(vector<Segment2>& vPolygon); // Create segments

 // Defined in terrain_25d.cpp
void getMountain(std::vector<Point2>& vPointsOut);



// main() - control
int breaklines_main()
{
	while(true)
	{
		std::cout<<"\n* Fade2.5D Demo - Breaklines, Cookie-Cutter and Contours"<<std::endl;
		std::cout<<"--------------------------------------------------------\n"<<std::endl;

		// * 1 *   Let the user choose
		cout << "\t0...Breaklines - no subdivision, no draping" << endl;
		cout << "\t1...Breaklines - no subdivision, with draping" << endl;
		cout << "\t2...Breaklines - with subdivision, no draping" << endl;
		cout << "\t3...Breaklines - with subdivision, with draping" << endl;
		cout << "\t4...Cookie Cutter - cut out pieces of a triangulation and save/load" << endl;
		cout << "\t5...ISO contours - intersection with a horizontal plane"<<endl;
		cout << "\t6...Height queries - get the height z at (x,y)"<<endl;
		cout << "\tq...quit" << endl;
		cout << "\n\n\tChoose an example [0-6],q: ";
		char choice(0);
		cin>>choice;
		switch(choice)
		{
			case '0':
				cout<<"* 0: Breaklines no subdivision, no draping"<<std::endl;
				terrain_breaklines_noSubdiv_noDraping();
				break;
			case '1':
				cout<<"* 1: Breaklines no subdivision, with draping"<<std::endl;
				terrain_breaklines_noSubdiv_withDraping();
				break;
			case '2':
				cout<<"* 2: Breaklines with subdivision, no draping"<<std::endl;
				terrain_breaklines_withSubdiv_noDraping();
				break;
			case '3':
				cout<<"* 3: Breaklines with subdivision, with draping"<<std::endl;
				terrain_breaklines_withSubdiv_withDraping();
				break;
			case '4':
				cout<<"* 4: Cookie Cutter"<<std::endl;
				cookieCutter();
				break;
			case '5':
				cout<<"* 5: ISO Contours"<<std::endl;
				isoContours();
				break;
			case '6':
				cout<<"* 6: Height Queries"<<std::endl;
				heightQueries();
				break;
			case 'q':
			default:
				cout<<"Returning"<<endl;
				return 1;
		}
	}
}



// 0: Insert breaklines (no subdivision, no draping)
void terrain_breaklines_noSubdiv_noDraping()
{
	// * 1 *   Create a surface and some segments
	Fade_2D dt;
	createTriangulation(dt);
    vector<Segment2> vSegments;
    createCircularPolygon(vSegments);

	// * 2 *   Insert the segments ('breaklines')
	// This method does not subdivide the segments except when
	// they intersect an existing vertex or another constraint.
	bool bOrientedSegments(false); // Not required for this demo
	bool bUseHeightOfLatest(true); // When a constraint crosses an existing one, the split point gets z from the last inserted constraint if true
	dt.createConstraint(vSegments,CIS_CONSTRAINED_DELAUNAY,bOrientedSegments,bUseHeightOfLatest);

	// * 4 *   Visualize
	dt.writeObj("c0_terrain_noSubdiv_noDraping.obj"); // For web browsers
	Visualizer3 vis("c0_terrain_noSubdiv_noDraping.list"); // For the Geomview viewer (Linux)
	dt.showGeomview(&vis,Visualizer3::CWHEAT);
	vis.writeSegments(vSegments,"0 0 1 0.5",true);
}


// 1: Insert breaklines (no subdivision, with draping)
void terrain_breaklines_noSubdiv_withDraping()
{
		// * 1 *   Create a surface and some segments
	Fade_2D dt;
	createTriangulation(dt);
    vector<Segment2> vSegments;
    createCircularPolygon(vSegments);

	// * 2 *   Insert the segments ('breaklines')
	// This mode drapes the segments onto the existing surface
	// before insertion. If zTolerance < 0 then the segments
	// are subdivided at all intersections with triangulation
	// edges. Otherwise subdivision takes only place to maintain
	// the defined tolerance i.e., the error between segment and
	// the surface.
	double zTolerance(0.5);
	vector<Segment2> vDrapedSegments;
	dt.drape(vSegments,vDrapedSegments,zTolerance);
	dt.createConstraint(vDrapedSegments,CIS_CONSTRAINED_DELAUNAY);


	// * 4 *   Visualize
	Visualizer3 vis("c1_terrain_noSubdiv_withDraping.list"); // For the Geomview viewer (Linux)
	dt.showGeomview(&vis,Visualizer3::CWHEAT);
	vis.writeSegments(vSegments,"0 0 1 0.5",true);
	vis.writeSegments(vDrapedSegments,"1 0 0 0.5",true);
	dt.show("c1_terrain_noSubdiv_withDraping.ps"); // Postscript for gv, gsview or online ps-viewers


}

// 2: Insert breaklines (with subdivision, no draping)
void terrain_breaklines_withSubdiv_noDraping()
{
	// * 1 *   Create a surface and some segments
	Fade_2D dt;
	createTriangulation(dt);
    vector<Segment2> vSegments;
    createCircularPolygon(vSegments);

	// * 2 *   Insert the segments ('breaklines')
	bool bOrientedSegments(false); // Not required for this demo
	bool bUseHeightOfLatest(true); // When a constraint crosses an existing one, the split point gets z from the last inserted constraint if true
    ConstraintGraph2* pCG=dt.createConstraint(vSegments,CIS_CONSTRAINED_DELAUNAY,bOrientedSegments,bUseHeightOfLatest);
	double minLen(0.1);
	pCG->makeDelaunay(minLen); // Subdivision

	// * 3 *   Visualize
	Visualizer3 vis("c2_terrain_withSubdiv_noDraping.list"); // For the Geomview viewer (Linux)
	dt.showGeomview(&vis,Visualizer3::CWHEAT);
	vis.writeSegments(vSegments,"0 0 1 0.5",true);
	dt.show("c2_terrain_withSubdiv_noDraping.ps"); // Postscript for gv, gsview or online ps-viewers
}


// 3: Insert breaklines (with subdivision, with draping)
void terrain_breaklines_withSubdiv_withDraping()
{
	// * 1 *   Create a surface and some segments
	Fade_2D dt;
	createTriangulation(dt);
    vector<Segment2> vSegments;
    createCircularPolygon(vSegments);

	// * 2 *   Insert the segments ('breaklines')
	double zTolerance(.1);
	vector<Segment2> vDrapedSegments;
	dt.drape(vSegments,vDrapedSegments,zTolerance);
	ConstraintGraph2* pCG=dt.createConstraint(vDrapedSegments,CIS_CONSTRAINED_DELAUNAY);
	double minLen(0.1);
	pCG->makeDelaunay(minLen); // Subdivision

	// * 3 *   Visualize
	Visualizer3 vis("c3_terrain_withSubdiv_withDraping.list"); // For the Geomview viewer (Linux)
	dt.showGeomview(&vis,Visualizer3::CWHEAT);
	vis.writeSegments(vSegments,"0 0 1 0.5",true);
	vis.writeSegments(vDrapedSegments,"1 0 0 0.5",true);
	dt.show("c3_terrain_withSubdiv_withDraping.ps"); // Postscript for gv, gsview or online ps-viewers
}

// 4: Cookie Cutter
void cookieCutter()
{
	// * 1 *   Create a surface and a polygon
	Fade_2D dt;
	createTriangulation(dt);
    vector<Segment2> vPolygon;
    createCircularPolygon(vPolygon);

	// * 2 *   Show the triangulation and the polygon
	Visualizer3 vis("c4_surface.list");
	Zone2* pGlobalZone(dt.createZone(NULL,ZL_GLOBAL));
	pGlobalZone->showGeomview(&vis,Visualizer3::CWHEAT);
	vis.writeSegments(vPolygon,Visualizer3::CRED);

	// * 3 *   Cut out a piece with the cookieCutter
	Zone2* pCookie(dt.createZone_cookieCutter(vPolygon,true));
    pCookie->showGeomview("c4_cookie.list",Visualizer3::CGREEN);

	// * 4 *   Compute also the complementary piece
	Zone2* pNotCookie(zoneDifference(pGlobalZone,pCookie));
	pNotCookie->showGeomview("c4_notCookie.list",Visualizer3::CWHEAT);

	// * 5 *   Save the triangulation and the zones to a file
	vector<Zone2*> vSaveZones;
	vSaveZones.push_back(pCookie);
	vSaveZones.push_back(pNotCookie);

	bool bSaveOK(false);
	std::ofstream file("c4_triangulation.fade",std::ios::binary);
	if(file.is_open())
	{
		// Use a filename or std::ostream as first argument
		bSaveOK=dt.saveTriangulation(file,vSaveZones);
		file.close();
	}
	if(!bSaveOK)
	{
		cout<<"File not written"<<endl;
		return;
	}

	// * 6 *   Load the triangulation from the file
	Fade_2D dt2;
	vector<Zone2*> vLoadZones;
	bool bLoadOK=dt2.load("c4_triangulation.fade",vLoadZones); // istream works also
	if(!bLoadOK)
	{
		cout<<"File not loaded"<<endl;
		return;
	}
	for(size_t i=0;i<vLoadZones.size();++i)
	{
		Zone2* pZone(vLoadZones[i]);
		string name("c4_reloadZone_"+toString(i)+".ps");
		pZone->show(name.c_str(),true,false);
	}
}


// 5: Compute ISO lines (intersection with a horizontal plane)
void isoContours()
{
	// * 1 *   Create a surface and fetch the triangles
	Fade_2D dt;
	createTriangulation(dt);
	std::vector<Triangle2*> vTriangles;
	dt.getTrianglePointers(vTriangles);

    // * 2 *   Create IsoContours isoc
    IsoContours isoc(vTriangles);
    double minZ(isoc.getMinHeight());
    double maxZ(isoc.getMaxHeight());
    double height(minZ+0.45*(maxZ-minZ));

	// * 3 *   Compute the contours
	std::vector<std::vector<Segment2> > vvContours;
	isoc.getContours(height,vvContours,true,true);
	cout<<vvContours.size()<<"polylines at height "<<height<<endl;

	// For the Geomview viewer (Linux)
	Visualizer3 vis("c5_isoContours.list");
	dt.showGeomview(&vis,Visualizer3::CWHEAT);

	// Write the result
	for(size_t i=0;i<vvContours.size();++i)
	{
		std::vector<Segment2>& vContour(vvContours[i]);
		cout<<"\nContour no. "<<i<<" consists of "<<vContour.size()<<" segments"<<std::endl;
		vis.writeSegments(vContour,"1 0 0 0.5",true);
		for(size_t j=0;j<vContour.size();++j)
		{
			cout<<vContour[j]<<endl;
		}
    }
}

// 6: Find the height at arbitrary (x,y) coordinates
void heightQueries()
{
	// * 1 *   Create a surface and fetch the triangles
	Fade_2D dt;
	createTriangulation(dt);

	// * 2 *   Retrieve the height values at arbitrary (x,y) coordinates
	for(int i=0;i<10;++i)
	{
		double x((100.0*rand())/RAND_MAX);
		double y((100.0*rand())/RAND_MAX);
		double z;
		bool bHaveHeight(dt.getHeight(x,y,z));
		if(bHaveHeight)
		{
			cout<<"x="<<x<<", y="<<y<<", z="<<z<<endl;
		}
		else
		{
			cout<<"x="<<x<<", y="<<y<<" is out of bounds"<<endl;
		}
	}
}

// Returns a triangulation fo testing
void createTriangulation(Fade_2D& dt)
{
	std::vector<Point2> vInputPoints;
	getMountain(vInputPoints);
	CloudPrepare cloudPrep;
	cloudPrep.add(vInputPoints);
	dt.insert(&cloudPrep,true);
}


// Returns segments of a circular polygon for testing
void createCircularPolygon(vector<Segment2>& vPolygon)
{
    int numPoints(8);
    double centerX(50),centerY(50),centerZ(65),radius(30.5);
    vector<Point2> vCirclePoints;
    generateCircle(numPoints,centerX,centerY,centerZ,radius,radius,vCirclePoints);
	pointsToPolyline(vCirclePoints,true,vPolygon);
}
