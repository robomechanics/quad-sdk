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


// Functions defined in other *.cpp files (declaration avoids additional header files)
int terrain_main();
int breaklines_main();
int segmentChecker_main();
int cutAndFill_main();
int removeBorderTriangles_main();
int meshImprovement_main();

void info()
{
	cout<<"\n\n\n\tWelcome to the Fade 25D examples"<<endl;
	cout<<"\t-------------------------------"<<endl<<endl;
	cout<<"\ta...Terrain Triangulation, Point Cloud Simplification, Data Export"<<endl;
	cout<<"\tb...Mesh Improvement - Smoothing, Valley/Ridge, Border Trimming"<<endl;
	cout<<"\tc...Breaklines, Cookie Cutter, ISO Contours, Height Queries"<<endl;
	cout<<"\td...Segment Checker"<<endl;
	cout<<"\te...Cut & Fill"<<endl;
}

int main()
{
	GEOM_FADE25D::Fade_2D dt;
	dt.printLicense();

	// Choose an example
	while(true)
	{
		info();
		char choice(0);
		cout << "\n\n\tChoose an example [a-e],q: ";
		cin>>choice;
		cout<<"\n\n\n"<<endl<<endl;
		switch(choice)
		{
			case 'a': terrain_main();break;
			case 'b': meshImprovement_main();break;
			case 'c': breaklines_main();break;
			case 'd': segmentChecker_main();break;
			case 'e': cutAndFill_main();break;
			case 'q':
			default:
				return 0;
		}
	}
}



