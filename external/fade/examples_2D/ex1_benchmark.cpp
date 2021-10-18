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
// *THIS* example:       https://www.geom.at/delaunay-triangulation-benchmark/
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

// Benchmark hints:
// * Make sure this example is linked against a release version of
//   the library and run it in a terminal, not in a VS debug window
// * Make sure you have enough memory to avoid usage of swap space,
//   you need roughly 21.3 GB per 100 mio points (on 64-bit systems)
// * Under Linux use the performance governor for more comparable
//   results.


int ex1_benchmark_main()
{
	std::cout<<"* Example1: Benchmark\n";
	std::cout<<"* Measures the (single-/multithreaded) triangulation time\n\n";

	// * 1 *   Create a Fade object.
	Fade_2D dt;
	int numUsedCPU=dt.setNumCPU(0); // 0 means: autodetect
	cout<<"Number of CPU cores: "<<numUsedCPU<<endl;

	// * 2 *   Set up how many points to test
	std::vector<std::pair<std::string,int> > vNumPoints;
	vNumPoints.push_back(make_pair("numPoints: 1k",1000));
	vNumPoints.push_back(make_pair("numPoints: 50k",50000));
	vNumPoints.push_back(make_pair("numPoints: 100k",100000));
	vNumPoints.push_back(make_pair("numPoints: 500k",500000));
	vNumPoints.push_back(make_pair("numPoints: 1 mio",1000000));
	vNumPoints.push_back(make_pair("numPoints: 10 mio",10000000));
	//vNumPoints.push_back(make_pair("numPoints: 50 mio (10.7 GB)",50000000));
	//vNumPoints.push_back(make_pair("numPoints: 100 mio (21.3 GB)",100000000));
	//vNumPoints.push_back(make_pair("numPoints: 250 mio (53 GB)",250000000));

	// * 3 *   Test
	for(size_t i=0;i<vNumPoints.size();++i)
	{
		std::string label(vNumPoints[i].first);
		int numPoints(vNumPoints[i].second);

		// Prepare a vector of random points
		vector<Point2> vInPoints;
		generateRandomPoints(numPoints,0,100,vInPoints,1);

		// Insert the points and erase afterwards. The total time for
		// triangulation (without destruction time) is measured.
		cout<<"\n"<<label<<": start"<<endl;
		double elapsed=dt.measureTriangulationTime(vInPoints);
		cout<<"Elapsed time: "<<elapsed<<endl<<endl;
	}

	return 0;
}
