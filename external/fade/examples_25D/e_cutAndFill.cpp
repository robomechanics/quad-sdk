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
// *THIS* example:         https://www.geom.at/cut-and-fill/
// Fade2.5D-Documentation: https://www.geom.at/fade25d/html/
//
// When you use Fade free of charge, please put a link on your
// research homepage.

#include <Fade_2D.h>

using namespace std;
using namespace GEOM_FADE25D;


// Custom Predicate derived from PeelPredicateTS (new since Fade v1.87)
class Decider:public PeelPredicateTS
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

// A terminal progress bar (optional)
class MyCAFProgBar:public GEOM_FADE25D::MsgBase
{
public:
	// Fade calls the update method with d={0.0,...,1.0}
	void update(MsgType ,const char* s,double d)
	{
		int numEq(int(d*20));
		std::string bar(numEq,'=');
		bar.append(20-numEq,' ');
		cout<<s<<" ["<<bar<<"] "<<d*100.0<<" %    \r"<<flush;
	}
};

// e: Cut-and-Fill
int cutAndFill_main()
{
	std::cout<<"\n* Fade2.5D Demo - Cut & Fill"<<std::endl;
	std::cout<<"----------------------------"<<std::endl<<std::endl;

	// * 1 *   Create two random surfaces, 2 x ~10 000 triangles.
	std::cout<<"\nPreparing two surfaces...\n\n"<<std::endl;
	vector<Point2> vRndSurfacePoints0;
	vector<Point2> vRndSurfacePoints1;
	generateRandomSurfacePoints(
		70, // numPointsX
		70, // numPointsY
		20, // numCenters
		0,0,-60,1000,1000,60, // Bounds xmin,ymin,zmin,xmax,ymax,zmax
		vRndSurfacePoints0,	// Output vector
		1	// Seed
		);
	generateRandomSurfacePoints(
		70, // numPointsX
		70, // numPointsY
		1, // numCenters
		150,-100,-30,1050,1050,30, // Bounds xmin,ymin,zmin,xmax,ymax,zmax
		vRndSurfacePoints1,	// Output vector
		2	// Seed
		);
	Fade_2D dt0;
	Fade_2D dt1;
	dt0.insert(vRndSurfacePoints0);
	dt1.insert(vRndSurfacePoints1);

	// * 2 *   Create Zones: One zone before the earthworks and one zone
	// after. The two zones do not need to match exactly, the algorithm
	// uses the overlapping area. Although the raw zones could be used
	// it is advised to peel off salient border triangles first whose
	// projection to the XY plane is almost zero. The behavior is
	// controlled by the user-specified PeelPredicate class.
	Zone2* pZoneBeforeRaw(dt0.createZone(NULL,ZL_GLOBAL,false));
	Zone2* pZoneAfterRaw(dt1.createZone(NULL,ZL_GLOBAL,false));
	Decider decider;
	Zone2* pZoneBefore=peelOffIf(pZoneBeforeRaw,true,&decider); // bAvoidSplit=true
	Zone2* pZoneAfter=peelOffIf(pZoneAfterRaw,true,&decider); // bAvoidSplit=true


	// * 3 *   Print & Draw
	cout<<"Surface before: "<<pZoneBefore->getNumberOfTriangles()<<" triangles"<<endl;
	cout<<"Surface after: "<<pZoneAfter->getNumberOfTriangles()<<" triangles"<<endl<<endl;
	pZoneBefore->showGeomview("e0_zoneBefore.list","1 0 0 0.5");
	pZoneAfter->showGeomview("e0_zoneAfter.list","0 1 0 0.5");
	dt0.writeObj("e0_zoneBefore.obj",pZoneBefore);
	dt1.writeObj("e0_zoneAfter.obj",pZoneAfter);


	// * 4 *   Create a CutAndFill object, register a progress bar
	//         (optional) and start the computations
	CutAndFill caf(pZoneBefore,pZoneAfter);
	MyCAFProgBar progBar=MyCAFProgBar();
	caf.subscribe(MSG_PROGRESS,&progBar);
	bool bOK=caf.go();
	if(!bOK)
    {
		cout<<"Cut&Fill computation (caf.go) failed: Check if the input surfaces do overlap!"<<endl;
		cout<<"- Can't continue"<<endl;
		return 1;
	}

	// * 5 *   Fetch connected components and report
	cout<<"\nReport:"<<endl;
	cout<<"-------"<<endl;
	cout<<"Number of components: "<<caf.getNumberOfComponents()<<endl;
	for(size_t i=0;i<caf.getNumberOfComponents();++i)
	{
		CAF_Component* pComponent(caf.getComponent(i));
		cout<<*pComponent<<endl;
	}

	// * 6 *   Draw the result in 2D
	Visualizer2 vis("e0_result.ps");
	caf.show(&vis);
	vis.writeFile();


	// * 7 *   Access the internal datastructures
	//
	// * mVtx2BeforeAfter is a map that contains for each vertex the
	// z-value in pZoneBefore and pZoneAfter
	// * pDiffZone defines the overlapping area of pZoneBefore and
	// pZoneAfter. The z-values of the points in pDiffZone correspond
	// to the height difference i.e. z-in-pZoneBefore minus z-in-
	// pZoneAfter.
	std::map< Point2 *, std::pair< double, double > > mVtx2BeforeAfter;
	Zone2* pDiffZone(NULL);
    bool bOK2=caf.getDiffZone(pDiffZone,mVtx2BeforeAfter);
	if(!bOK2)
    {
		cout<<"caf.getDiffZone() failed: Did not get pDiffZone. Check your input data."<<endl;
		return 1;
	}

	// * 8 *   Assign custom indices to the points in pDiffZone
	vector<Point2*> vZonePoints;
	pDiffZone->getVertices(vZonePoints);
	int counter(0);
    for(vector<Point2*>::iterator it(vZonePoints.begin());it!=vZonePoints.end();++it)
    {
        Point2* pVtx(*it);
        pVtx->setCustomIndex(counter++);
    }

	// * 9 *   Write a face-list file
	const char* filename("facelist.txt");
	std::ofstream file(filename);
	if(file.is_open())
	{
		cout<<"\nwriting "<<filename<<endl;
	}
	else
	{
		cout<<filename<<" can't be written"<<endl;
		return 1;
	}
	// * 10 *   Fetch the triangles
    vector<Triangle2*> vTriangles;
    pDiffZone->getTriangles(vTriangles);
	file<<"Facelist: numPoints="<<vZonePoints.size()<<", numTriangles: "<<vTriangles.size()<<endl;

	// * 11 *   Write the point list
	for(vector<Point2*>::iterator it(vZonePoints.begin());it!=vZonePoints.end();++it)
    {
        Point2* pVtx(*it);
        int idx(pVtx->getCustomIndex());
        std::map< Point2 *, std::pair< double, double > >::iterator q_it(mVtx2BeforeAfter.find(pVtx));
        assert(q_it!=mVtx2BeforeAfter.end());
        double zBefore(q_it->second.first);
        double zAfter(q_it->second.second);
		file<<idx<<"\tx="<<pVtx->x()<<" y="<<pVtx->y()<<"\tzdiff="<<pVtx->z()<<" zBefore="<<zBefore<<" zAfter="<<zAfter<<endl;
    }

	// * 12 *   Write the corner indices list
    for(vector<Triangle2*>::iterator it(vTriangles.begin());it!=vTriangles.end();++it)
    {
        Triangle2* pT(*it);
        file<<pT->getCorner(0)->getCustomIndex()<<" ";
        file<<pT->getCorner(1)->getCustomIndex()<<" ";
        file<<pT->getCorner(2)->getCustomIndex()<<"\n";
    }
	file.close();

	return 0;
}


