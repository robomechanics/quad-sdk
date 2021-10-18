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
#include <stdio.h>
#include <map>
#include <string>
#include <sstream>
#include <iomanip>
using namespace GEOM_FADE2D;
using namespace std;

// Create a simple command line progress bar that derives from MsgBase
class MyProgressBar:public MsgBase
{
public:
        // Fade will later call the update method with d={0.0,...,1.0}
        void update(MsgType ,const char* s,double d)
        {
                cout<<s<<" [";
                for(size_t i=0;i<10;++i)
                {
                        if(i/10.0<d) cout<<"=";
                                else cout<<" ";
                }
                cout<<"] "<<d*100.0<<" %    \r"<<flush;
                if(d==1.0) cout<<endl<<endl;
        }
};


int ex10_progressBar_main()
{
	std::cout<<"\nExample10: Progress bar\n";

	// * 1 *   Create a Fade object dt
	Fade_2D dt;

	// * 2 *   Progress bar object: Subscribe to MSG_PROGRESS messages of dt
	MyProgressBar myPBar;
	dt.subscribe(MSG_PROGRESS,&myPBar);

	// * 3 *   Insert some random segments (and watch the progress bar output)
    std::vector<Segment2> vInputSegments;
	generateRandomSegments(10000,0,100,10,vInputSegments,0);
	dt.createConstraint(vInputSegments, CIS_CONSTRAINED_DELAUNAY);

	return 0;
}


