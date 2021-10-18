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
// Fade2D-Examples       https://www.geom.at/category/fade2d-examples/
// Fade2D-Documentation: https://www.geom.at/fade2d/html/
//
// Fade2.5D-Examples:      https://www.geom.at/category/fade25d-examples/
// Fade2.5D-Documentation: https://www.geom.at/fade25d/html/
//
// When you use Fade free of charge, please put a link on your
// research homepage.

#include <stdio.h>
#include <iostream>
#include <Fade_2D.h>

using namespace std;

// These functions are defined in the corresponding cpp-files
int ex0_hello_main();
int ex1_benchmark_main();
int ex2_accessDraw_main();
int ex3_constraintEdges_main();
int ex4_zones_main();
int ex5_booleanOps_main();
int ex6_booleanOps2_main();
int ex7_qualityMeshing_main();
int ex8_triangulationExport_main();
int ex9_randomObjects_main();
int ex10_progressBar_main();
int ex11_save_and_load_main();
int ex12_voronoi_main();

void info()
{
	cout<<"\n\n\n\tWelcome to the Fade 2D examples"<<endl;
	cout<<"\t-------------------------------"<<endl<<endl;
	cout<<"\t0...HelloTriangulation - the most simple code with visualization"<<endl;
	cout<<"\t1...Benchmark - single- and multithreaded Delaunay triangulation times"<<endl;
	cout<<"\t2...Traversing - Retrieve geometric elements and draw them"<<endl;
	cout<<"\t3...Constrained Delaunay - Insert constraint edges"<<endl;
	cout<<"\t4...Zones - Defined areas in triangulations"<<endl;
	cout<<"\t5...Zones - Boolean operations with zones"<<endl;
	cout<<"\t6...Zones - Boolean operations, practical source code"<<endl;
	cout<<"\t7...Advanced Meshing - Quality meshing"<<endl;
	cout<<"\t8...Fade Export - Point triangle tndices (face list)"<<endl;
	cout<<"\t9...Random geometric objects - Create random test data"<<endl;
	cout<<"\ta...Progress Bar - A progress update mechanism"<<endl;
	cout<<"\tb...Save and load - Store and reload triangulation data"<<endl;
	cout<<"\tc...Voronoi - Point location in a Voronoi diagram"<<endl;
}

int main()
{
	GEOM_FADE2D::Fade_2D dt;
	dt.printLicense();

	while(true)
	{
		info();
		char choice(0);
		cout << "\n\n\tChoose an example [0-9 and a-c], q...quit ";
		cin>>choice;
		cout<<"\n\n\t-------------------------------------------------"<<endl<<endl;
		switch(choice)
		{
			case '0': ex0_hello_main();break;
			case '1': ex1_benchmark_main();break;
			case '2': ex2_accessDraw_main();break;
			case '3': ex3_constraintEdges_main();break;
			case '4': ex4_zones_main();break;
			case '5': ex5_booleanOps_main();break;
			case '6': ex6_booleanOps2_main();break;
			case '7': ex7_qualityMeshing_main();break;
			case '8': ex8_triangulationExport_main();break;
			case '9': ex9_randomObjects_main();break;
			case 'a': ex10_progressBar_main();break;
			case 'b': ex11_save_and_load_main();break;
			case 'c': ex12_voronoi_main();break;
			case 'q': return 0;
			default: break;
		}
		cout<<"\n\t-------------------------------------------------"<<endl<<endl;
	}
}

