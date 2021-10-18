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
// *THIS* example:         https://www.geom.at/mesh-improvements/
// Fade2.5D-Documentation: https://www.geom.at/fade25d/html/
//
// When you use Fade free of charge, please put a link on your
// research homepage.

#include <Fade_2D.h>
#include <Visualizer3.h>
#include "someTools.h"



using namespace std;
using namespace GEOM_FADE25D;

// Prototypes for meshImprovement
int meshImprovement_main(); // Main function
void smoothing(); // 0...Weighted Laplacian Smoothing and Valley/Ridge/River edge-flipping
void valleyRidge(); // 1...Weighted Laplacian Smoothing and Valley/Ridge/River edge-flipping
void removeBorderTriangles(); // 2...Remove unwanted triangles

// Defined in terrain25d.cpp
void getInputPoints(const std::string& filename,std::vector<Point2>& vPointsOut); // Creates input
void write(const std::string& id,const std::string& name,const Fade_2D& dt); // Writer (*.list and *.obj)
void getMountain(std::vector<Point2>& vPointsOut); // Returns a test terrain


// * main() - control
int meshImprovement_main()
{
	std::cout<<gRed("\n\n\n+++ +++ +++ +++ +++ +++ +++")<<std::endl;
	while(true)
	{
		std::cout<<"\n\n* Fade2.5D Demo - Mesh improvement\n"<<std::endl;

		// * 1 *   Let the user choose
		cout << "0...Smoothing" << endl;
		cout << "1...Valley/Ridge/River Optimization" << endl;
		cout << "2...Bad-shaped Border Triangle Removement" << endl;
		cout << "q...quit" << endl;
		cout << "\n\nChoose an example [0-2],q: ";
		char choice(0);
		cin>>choice;
		cout<<"------------------------------\n\n";
		switch(choice)
		{
			case '0':
				cout<<"* Smoothing"<<std::endl;
				smoothing();
				break;
			case '1':
				cout<<"* Valley/Ridge/River edge-flips"<<std::endl;
				valleyRidge();
				break;
			case '2':
				cout<<"* Remove border triangles"<<std::endl;
				removeBorderTriangles();
				break;
			case 'q':
			default:
				cout<<"Returning"<<endl;
				return 1;
		}
		cout<<"\n\nEND\n------------------------------"<<endl;
	}
}



// 0: Smoothing
void smoothing()
{
	// * 1 *   Get input points and add some noise
	std::vector<Point2> vInputPoints;
	getInputPoints("MOUNTAIN",vInputPoints);
	for(size_t i=0;i<vInputPoints.size();++i)
	{
		Point2& p(vInputPoints[i]);
		p.setHeight(p.z()-.1+(.2*rand())/RAND_MAX);// Noise
	}

	// * 2 *   Triangulate
	CloudPrepare cloudPrep;
	cloudPrep.add(vInputPoints);
	Fade_2D dt;
	dt.insert(&cloudPrep,true);

	// * 3 *   Show the noisy terrain
	write("b0","terrain_noisy",dt);

	// * 4 *   Smoothing and show
	Zone2* pGlobalZone(dt.createZone(NULL,ZL_GLOBAL));
	pGlobalZone->smoothing(2);
	write("b0","terrain_smooth",dt);
}

// 1: ValleyRidge
void valleyRidge()
{
	// * 1 *   Get input points
	std::vector<Point2> vInputPoints;
	getInputPoints("MOUNTAIN",vInputPoints);

	// * 2 *   Triangulate
	CloudPrepare cloudPrep;
	cloudPrep.add(vInputPoints);
	Fade_2D dt;
	dt.insert(&cloudPrep,true);

	// * 3 *   Show the original triangulation
	write("b1","original",dt);

	// * 4 *   Flip edges to optimize valleys/ridges/rivers
	Zone2* pGlobalZone(dt.createZone(NULL,ZL_GLOBAL));
	GASSEX(pGlobalZone!=NULL);
	pGlobalZone->slopeValleyRidgeOptimization(OPTMODE_BETTER);
	write("b1","flow",dt);
}


// 2: Custom Predicate derived from PeelPredicateTS
class PeelDecider:public PeelPredicateTS // (new since Fade v1.87)
{
public:
	bool operator()(const Triangle2* pT,std::set<Triangle2*>* pCurrentSet)
	{
		// Angle between face normal and (0,0,1)
		Vector2 nv(pT->getNormalVector());
		Vector2 up(0,0,1);
		double angle(-1);
		double cosPhi(nv*up);
		if(cosPhi>1.0) angle=0; // Robustness in case of numeric inaccuracy
			else if(cosPhi<-1.0) angle=180.0; // Robustness in case of numeric inaccuracy
				else angle=acos(cosPhi)*57.2958;
		if(angle>85.0)
		{
			// >85 degrees between face normal and v(0,0,1)
			return true; // Bad triangle
		}
		for (int i = 0; i < 3; ++i)
        {
			// Is the opposite edge of i a border edge of the zone?
			Triangle2* pNeigT(pT->getOppositeTriangle(i));
			if(	pNeigT==NULL ||
				pCurrentSet->find(pNeigT)==pCurrentSet->end() )
			{
				// Yes, border
				if (pT->getInteriorAngle25D(i) > 140)
				{
					return true;
				}
			}
        }
        return false;
	}
};

// 2: Remove border triangles
void removeBorderTriangles()
{
	// * 1 *   Get input points and simplify them without maintaining
	//         the convex hull because this is very likely to create
	//         bad border triangles - a setting that we want for this
	//         demo:
	std::vector<Point2> vInputPoints;
	getInputPoints("MOUNTAIN",vInputPoints);
	CloudPrepare cloudPrep;
	cloudPrep.add(vInputPoints);
	cloudPrep.adaptiveSimplify(1.0,SMS_MEDIAN,CHS_NOHULL);
	cout<<"CloudPrepare adaptiveSimplify, numPoints reduced="<<cloudPrep.getNumPoints()<<endl;
	Fade_2D dt;
	dt.insert(&cloudPrep,true);
	dt.showGeomview("b2_badBorder.list",Visualizer3::CORANGE);
	dt.writeObj("b2_badBorder.obj");

	// * 2 *   Create a global zone and a user precidate
	Zone2* pGlobalZone(dt.createZone(NULL,ZL_GLOBAL));
	cout<<"Original, number of triangles: "<<pGlobalZone->getNumberOfTriangles()<<endl;

	// * 3 *   Peel unwanted border triangles off
	cout<<"Getting rid of unwanted border triangles"<<endl;
	PeelDecider decider;
	Zone2* pResult=peelOffIf(pGlobalZone,true,&decider);
	if(pResult==NULL)
	{
		cout<<"NO RESULT ZONE"<<endl;
		return;
	}
	pResult->showGeomview("b2_goodBorder.list",Visualizer3::CORANGE);
	pResult->writeObj("b2_goodBorder.obj");
	cout<<"Cleaned, number of triangles: "<<pResult->getNumberOfTriangles()<<endl;
}

