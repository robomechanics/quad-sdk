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
// *THIS* example:         https://www.geom.at/segment-checker/
// Fade2.5D-Documentation: https://www.geom.at/fade25d/html/
//
// When you use Fade free of charge, please put a link on your
// research homepage.

#include <Fade_2D.h>
#include <Visualizer3.h>
#include "someTools.h"
using namespace GEOM_FADE25D;
using namespace std;

// Prototypes for the SegmentChecker examples
int segmentChecker_main();



inline double rnd(double f)
{
	return (f*rand()/RAND_MAX);
}

// d0: segmentChecker
int segmentChecker_main()
{
	std::cout<<gRed("\n\n\n+++ +++ +++ +++ +++ +++ +++")<<std::endl;
	std::cout<<"\n\n* Fade2.5D Demo - Segment Checker\n"<<std::endl;

	// * 1 *   Create random segments:
	vector<Segment2*> vRndSeg;
	for(size_t i=0;i<500;++i)
	{
		Point2 p0(rnd(100),rnd(100),rnd(20));
		Point2 p1(rnd(100),rnd(100),rnd(20));
		vRndSeg.push_back(new Segment2(p0,p1));
	}

	// * 2 *   Find all intersecting segments
	SegmentChecker segChecker(vRndSeg);
	std::vector<Segment2*> vIntersectingSeg;
	segChecker.getIllegalSegments(false,vIntersectingSeg);
	cout<<"Intersecting segments: "<<vIntersectingSeg.size()<<endl;

	// * 3 *   Draw the intersections
	segChecker.showIllegalSegments(false,"d0_intersections.ps");
	if(vIntersectingSeg.empty()) return 0;

	// * 4 *   Demonstrate just for one intersecting segment
	//         how to analyze its intersections
	Segment2* pSeg(vIntersectingSeg[0]);
	cout<<"\nAnalyzing segment no. "<<segChecker.getIndex(pSeg)<<": "<<endl;

	// * 5 *   Get the intersectors of pSeg
	std::vector<std::pair< Segment2*,SegmentIntersectionType> > vIntersectors;
	segChecker.getIntersectors(pSeg,false,vIntersectors);

	// * 6 *   Iterate over the intersectors of pSeg:
	for(size_t j=0;j<vIntersectors.size();++j)
	{
		// a: The intersector and the intersection type
		Segment2* pOtherSeg(vIntersectors[j].first);
		SegmentIntersectionType sit(vIntersectors[j].second);
		cout<<"  Conflicting segment no. "<<segChecker.getIndex(pOtherSeg)<<"\t type="<<segChecker.getIntersectionTypeString(sit)<<endl;

		// b: Depending on the segment intersection type (sit):
		switch(sit)
		{
			case SIT_ENDPOINT:
			case SIT_POINT:
			{
				// Two segments can intersect at two different z values, thus two intersection points
				Point2 isp0,isp1;
				segChecker.getIntersectionPoint(sit,*pSeg,*pOtherSeg,isp0,isp1);
				cout<<"    intersection point on segment "<<segChecker.getIndex(pSeg)<<": "<<isp0<<endl;
				cout<<"    intersection point on segment "<<segChecker.getIndex(pOtherSeg)<<": "<<isp1<<endl;

				break;
			}
			case SIT_SEGMENT:
			{
				// Same for a collinear intersection, there may be two segments at different heights
				Segment2 iss0,iss1;
				segChecker.getIntersectionSegment(*pSeg,*pOtherSeg,iss0,iss1);
				cout<<"    intersection segment on segment "<<segChecker.getIndex(pSeg)<<": "<<iss0<<endl;
				cout<<"    intersection segment on segment "<<segChecker.getIndex(pOtherSeg)<<": "<<iss1<<endl;
				break;
			}
			case SIT_NONE: // Never reached
			{
				cout<<"    no intersection, impossible case"<<endl;
				break;
			}
			default: // Never reached
			{
				cout<<"    uninitialized, impossible case"<<endl;
			}
		}
	}


	cout<<"\n\nEND\n------------------------------"<<endl;
	return 0;
}

