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
// *THIS* example:       https://www.geom.at/automated-testing/
// Fade2D-Documentation: https://www.geom.at/fade2d/html/
//
// When you use Fade free of charge, please put a link on your
// research homepage.

#include <Fade_2D.h>
#include <stdio.h>
#include <map>
#include <string>
#include <sstream>
#include <iomanip>
using namespace GEOM_FADE2D;
using namespace std;

void randomNumbers()
{
	size_t num(5);
	double min(-100.0);
	double max(100.0);

	for(int seed=0;seed<3;++seed)
	{
		cout<<"seed "<<seed<<": "<<endl;

		vector<double> vRandomNumbers0;
		vector<double> vRandomNumbers1;
		generateRandomNumbers(num,min,max,vRandomNumbers0,seed);
		generateRandomNumbers(num,min,max,vRandomNumbers1,seed);


		for(size_t i=0;i<vRandomNumbers0.size();++i)
		{
			cout<<vRandomNumbers0[i]<<" ";
		}
		cout<<endl;
		for(size_t i=0;i<vRandomNumbers1.size();++i)
		{
			cout<<vRandomNumbers1[i]<<" ";
		}
		cout<<endl<<endl;
	}
}

void randomPoints()
{
	size_t num(1000);
	double min(-100.0);
	double max(100.0);
	int seed(0); // seed=0 for real randomness

	vector<Point2> vRandomPoints;
	generateRandomPoints(num,min,max,vRandomPoints,seed);
	Visualizer2 vis("example9_rndPoints.ps");
	vis.addObject(vRandomPoints,Color(CBLACK));
	vis.writeFile();
}

void randomPolygon()
{
	size_t num(50);
	double min(-100.0);
	double max(100.0);
	int seed(0); // seed=0 for real randomness

	vector<Segment2> vRandomPolygon;
	generateRandomPolygon(num,min,max,vRandomPolygon,seed);
	Visualizer2 vis("example9_rndPolygon.ps");
	vis.addObject(vRandomPolygon,Color(CBLACK));
	vis.writeFile();
}

void randomSegments()
{
	size_t num(50);
	double min(-100.0);
	double max(100.0);
	double maxLen(50);
	int seed(0);// seed=0 for real randomness

	vector<Segment2> vRandomSegments;
	generateRandomSegments(num,min,max,maxLen,vRandomSegments,seed);
	Visualizer2 vis("example9_rndSegments.ps");
	vis.addObject(vRandomSegments,Color(CBLACK));
	vis.writeFile();
}
void sineFunctions()
{
	int numSegments(50);
	int numPeriods(1);
	double xOffset(0);
	double yOffset(0);
	double xFactor(1.0);
	double yFactor(1.0);
	bool bSwapXY(false);

	vector<Segment2> vSineSegments0;
	vector<Segment2> vSineSegments1;
	vector<Segment2> vSineSegments2;
	vector<Segment2> vSineSegments3;
	generateSineSegments(numSegments,numPeriods,xOffset,yOffset,xFactor,yFactor,bSwapXY,vSineSegments0);
	generateSineSegments(numSegments,numPeriods,3.14159/2.0,yOffset,xFactor,yFactor,bSwapXY,vSineSegments1);
	generateSineSegments(numSegments,3,xOffset,yOffset,.33,yFactor,true,vSineSegments2);
	generateSineSegments(numSegments,3,xOffset,yOffset,.33,-yFactor,true,vSineSegments3);

	Visualizer2 vis("example9_sine.ps");
	vis.addObject(vSineSegments0,Color(CBLUE));
	vis.addObject(vSineSegments1,Color(CGREEN));
	vis.addObject(vSineSegments2,Color(CRED));
	vis.addObject(vSineSegments3,Color(CBLACK));
	vis.writeFile();

}
void circles()
{
	int numPoints(50);
	double centerX(0.0);
	double centerY(0.0);
	double radiusX(1.0);
	double radiusY(1.0);
	vector<Point2> vCirclePoints0;
	vector<Point2> vCirclePoints1;
	generateCircle(numPoints,centerX,centerY,radiusX,radiusY,vCirclePoints0);
	generateCircle(2*numPoints,centerX,centerY,2*radiusX,radiusY,vCirclePoints1);

	Visualizer2 vis("example9_circles.ps");
	vis.addObject(vCirclePoints0,Color(CBLUE));
	vis.addObject(vCirclePoints1,Color(CGREEN));
	vis.writeFile();
}

template<class T_InType>
std::string toString(const T_InType& in)
{
	std::ostringstream oss;
	oss << in;
	return oss.str();
}


int ex9_randomObjects_main()
{
	std::cout<<"* Example9: Random geometric objects\n";
	randomNumbers();
	randomPoints();
	randomPolygon();
	randomSegments();
	sineFunctions();
	circles();
	return 0;
}


