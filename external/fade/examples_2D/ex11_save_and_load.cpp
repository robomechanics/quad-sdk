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
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include "someTools.h"

using namespace GEOM_FADE2D;

void createTestZones(Fade_2D& dt,Zone2*& pEarthZone,Zone2*& pMarsZone,Zone2*& pDiffZone);



int ex11_save_and_load_main()
{
	std::cout<<"* Example11: Save and Load - Store and reload triangulation data"<<std::endl;

	// * 1 *   Create 3 zones for this demo and visualize the
	//         triangulation with pDiffZone highlighted
	Fade_2D dt;
	Zone2* pEarthZone(NULL); // large circle
	Zone2* pMarsZone(NULL); // small circle
	Zone2* pDiffZone(NULL); // difference
	std::cout<<"\nCreating test zones for this demo"<<std::endl;
	createTestZones(dt,pEarthZone,pMarsZone,pDiffZone);
	std::cout<<" triangulation, numT="<<dt.numberOfTriangles()<<std::endl;
	std::cout<<" pEarthZone, numT="<<pEarthZone->getNumberOfTriangles()<<std::endl;
	std::cout<<" pMarsZone, numT="<<pMarsZone->getNumberOfTriangles()<<std::endl;
	std::cout<<" pDiffZone, numT="<<pDiffZone->getNumberOfTriangles()<<std::endl;
	std::vector<Zone2*> vSaveZones; // Below we will save these zones
	vSaveZones.push_back(pEarthZone);
	vSaveZones.push_back(pMarsZone);
	vSaveZones.push_back(pDiffZone);
	pEarthZone->show("ex11_orgTriangulation_and_pEarthZone.ps",true,true);
	pMarsZone->show("ex11_orgTriangulation_and_pMarsZone.ps",true,true);
	pDiffZone->show("ex11_orgTriangulation_and_pDiffZone.ps",true,true);

	// # The save/load commands accept a filename or any ostream #

	// * 2 *   Save all triangles and vSaveZones (filename provided)
	std::cout<<"saveTriangulation()..."<<std::endl;
	dt.saveTriangulation("ex11_saveTriangulation.fade",vSaveZones);

	// * 3 *   Save only vSaveZones (ofstream provided)
	std::cout<<"saveZones()..."<<std::endl;
	std::ofstream file("ex11_saveZones.fade",std::ios::binary);
	if(file.is_open())
	{
		bool bOK=dt.saveZones(file,vSaveZones);
		file.close();
		GASSEX(bOK); // End this demo if false
	}
	else
	{
		GASSEX(false); // End this demo
	}

	// * 4 *   Save an individual zone (stringstream provided)
	std::cout<<"Zone2::save()..."<<std::endl;
	std::stringstream sstr;
	pDiffZone->save(sstr);


	// * 5 *   Reload the full triangulation with all zones
	std::cout<<"\nReloading...\n"<<std::endl;
	std::cout<<"Loading the full triangulation and the 3 zones..."<<std::endl;
	std::cout<<"  ...first argument is the filename, \"ex11_saveTriangulation.fade\""<<std::endl;
	Fade_2D loadDt1;
	std::vector<Zone2*> vLoadZones1;
	bool bOK1=loadDt1.load("ex11_saveTriangulation.fade",vLoadZones1);
	GASSEX(bOK1); // End this demo if false
	std::cout<<" triangulation, numT="<<loadDt1.numberOfTriangles()<<std::endl;
	std::cout<<" pEarthZone, numT="<<vLoadZones1[0]->getNumberOfTriangles()<<std::endl;
	std::cout<<" pMarsZone, numT="<<vLoadZones1[1]->getNumberOfTriangles()<<std::endl;
	std::cout<<" pDiffZone, numT="<<vLoadZones1[2]->getNumberOfTriangles()<<std::endl;
	vLoadZones1[2]->show("ex11_allTriangles_reloaded_pDiffZone_highlighted.ps",true,true);

	// * 6 *   Reload only the zones
	std::cout<<std::endl<<"\nLoading just the 3 zones"<<std::endl;
	std::cout<<"  ...first argument is ifstream(\"ex11_saveZones.fade\")"<<std::endl;
	Fade_2D loadDt2;
	std::vector<Zone2*> vLoadZones2;
	std::ifstream inFile("ex11_saveZones.fade",std::ios::binary);
	bool bOK2(false);
	if(inFile.is_open())
	{
		bOK2=loadDt2.load(inFile,vLoadZones2);
		inFile.close();
	}
	GASSEX(bOK2); // End this demo if false

	std::cout<<" triangulation, numT="<<loadDt2.numberOfTriangles()<<std::endl;
	std::cout<<" pEarthZone, numT="<<vLoadZones2[0]->getNumberOfTriangles()<<std::endl;
	std::cout<<" pMarsZone, numT="<<vLoadZones2[1]->getNumberOfTriangles()<<std::endl;
	std::cout<<" pDiffZone, numT="<<vLoadZones2[2]->getNumberOfTriangles()<<std::endl;
	vLoadZones2[2]->show("ex11_zones_reloaded_pDiffZone_highlighted.ps",true,true);

	// * 7 *   Reload one zone from stringstream
	std::cout<<std::endl<<"\nLoading only one zone"<<std::endl;
	std::cout<<"  ...first argument is a stringstream"<<std::endl;
	Fade_2D loadDt3;
	std::vector<Zone2*> vLoadZones3;
	bool bOK3=loadDt3.load(sstr,vLoadZones3);
	GASSEX(bOK3); // End this demo if false
	std::cout<<" triangulation, numT="<<loadDt3.numberOfTriangles()<<std::endl;
	std::cout<<" pDiffZone, numT="<<vLoadZones3[0]->getNumberOfTriangles()<<std::endl;
	vLoadZones3[0]->show("ex11_one_zone_reloaded_and_highlighted.ps",true,true);

	return 0;
}


void createTestZones(Fade_2D& dt,Zone2*& pEarthZone,Zone2*& pMarsZone,Zone2*& pDiffZone)
{
	// Create circle-points for Earth and Mars
	int numPoints(20);
	std::vector<Point2> vEarthPoints;
	std::vector<Point2> vMarsPoints;
	double earthRadius(6371);
	double marsRadius(3389.5);
	generateCircle(numPoints,0,0,earthRadius,earthRadius,vEarthPoints);
	generateCircle(numPoints,5000,0,marsRadius,marsRadius,vMarsPoints);

	// Turn them into polygons and insert them as constraints
	std::vector<Segment2> vEarthSegments;
	std::vector<Segment2> vMarsSegments;
	pointsToPolyline(vEarthPoints,true,vEarthSegments);
	pointsToPolyline(vMarsPoints,true,vMarsSegments);
	ConstraintGraph2* pEarthCG(dt.createConstraint(vEarthSegments,CIS_CONSTRAINED_DELAUNAY));
	ConstraintGraph2* pMarsCG(dt.createConstraint(vMarsSegments,CIS_CONSTRAINED_DELAUNAY));
	GASSEX(pEarthCG!=NULL);
	GASSEX(pMarsCG!=NULL);

	// Create the zones pEarthZone and pMarsZone. Then create pDiffZone
	// as their difference
	pEarthZone=dt.createZone(pEarthCG,ZL_INSIDE);
	pMarsZone=dt.createZone(pMarsCG,ZL_INSIDE);
	GASSEX(pEarthZone!=NULL);
	GASSEX(pMarsZone!=NULL);
	pDiffZone=zoneDifference(pEarthZone,pMarsZone);

	// Add the 4 points of an extended bounding box
	Bbox2 box(dt.computeBoundingBox());
	std::vector<Point2> vBoxPoints;
	box.getOffsetCorners(500.0,vBoxPoints);
	dt.insert(vBoxPoints);
}
