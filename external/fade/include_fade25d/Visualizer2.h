// Copyright (C) Geom Software e.U, Bernhard Kornberger, Graz/Austria
//
// This file is part of the Fade2D library. The student license is free
// of charge and covers personal non-commercial research. Licensees
// holding a commercial license may use this file in accordance with
// the Commercial License Agreement.
//
// This software is provided AS IS with NO WARRANTY OF ANY KIND,
// INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE.
//
// Please contact the author if any conditions of this licensing are
// not clear to you.
//
// Author: Bernhard Kornberger, bkorn (at) geom.at
// http://www.geom.at

/// @file Visualizer2.h
#pragma once


#include "Point2.h"
#include "Circle2.h"
#include "Segment2.h"
#include "Color.h"
#include "Label.h"
#include "Bbox2.h"
#include "Edge2.h"


#include "common.h"
#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif
class ConstraintSegment2; // FWD
class VoroCell2; // FWD
struct Dat; // FWD
class SegmentChecker; // FWD


/** \brief Visualizer2 is a general Postscript writer. It draws the
 * objects Point2, Segment2, Triangle2, Circle2 and Label.
 *
* \sa http://www.geom.at/example2-traversing/
* \image html visualizer.jpg "Example output of the Visualizer"
* \image latex visualizer.eps "Example output of the Visualizer" width=12cm
*
*

*/
class Visualizer2
{
public:


/** \brief Constructor
*
* @param filename_ is the name of the postscript file to be written
*/
	CLASS_DECLSPEC
	explicit Visualizer2(const char* filename_);

	CLASS_DECLSPEC
	~Visualizer2();
/** \brief Add a vector of Voronoi Cells to the visualization
*/
	CLASS_DECLSPEC
	void addObject(const std::vector<VoroCell2*>& vT,const Color& c);

/** \brief Add a Voronoi cell to the visualization
*/
	CLASS_DECLSPEC
	void addObject(VoroCell2* pVoroCell,const Color& c);

/** \brief Add a Segment2 object to the visualization
*/
	CLASS_DECLSPEC
	void addObject(const Segment2& seg,const Color& c);

/** \brief Add an Edge2 object to the visualization
*/
	CLASS_DECLSPEC
	void addObject(const Edge2& edge,const Color& c);

/** \brief Add a vector of Point2 objects to the visualization
*/
	CLASS_DECLSPEC
	void addObject(const std::vector<Point2>& vPoints,const Color& c);

/** \brief Add a vector of Point2* to the visualization
*/
	CLASS_DECLSPEC
	void addObject(const std::vector<Point2*>& vPoints,const Color& c);


/** \brief Add a vector of Segment2 objects to the visualization
*/
	CLASS_DECLSPEC
	void addObject(const std::vector<Segment2>& vSegments,const Color& c);

/** \brief Add a vector of ConstraintSegment2 objects to the visualization
*/
	CLASS_DECLSPEC
	void addObject(const std::vector<ConstraintSegment2*>& vConstraintSegments,const Color& c);

/** \brief Add a vector of Edge2 objects to the visualization
*/
	CLASS_DECLSPEC
	void addObject(const std::vector<Edge2>& vSegments,const Color& c);


/** \brief Add a vector of Triangle2 objects to the visualization
*/
	CLASS_DECLSPEC
	void addObject(const std::vector<Triangle2>& vT,const Color& c);

/** \brief Add a Circle2 object to the visualization
*/
	CLASS_DECLSPEC
	void addObject(const Circle2& circ,const Color& c);
/** \brief Add a Point2 object to the visualization
*/
	CLASS_DECLSPEC
	void addObject(const Point2& pnt,const Color& c);
/** \brief Add a Triangle2 object to the visualization
*/
	CLASS_DECLSPEC
	void addObject(const Triangle2& tri,const Color& c);

/** \brief Add a Triangle2* vector to the visualization
*/
	CLASS_DECLSPEC
	void addObject(const std::vector<Triangle2*>& vT,const Color& c);

/** \brief Add a Label object to the visualization
*/
	CLASS_DECLSPEC
	void addObject(const Label& lab,const Color& c);

/** \brief Add a header line to the visualization
*/
	CLASS_DECLSPEC
	void addHeaderLine(const char* s);


/** \brief Finish and write the postscript file
*
* @note This method \e must be called at the end when all the objects have been added.
*/
	CLASS_DECLSPEC
	void writeFile();

/** @private
 */
	CLASS_DECLSPEC
	void setLimit(const Bbox2& bbx);

	/** \brief Compute the range
	 *
	 * @param bWithVoronoi specifies if the Voronoi cells shall be
	 * incorporated.
	 *
	 * @return a bounding box of currently contained objects
	 */
	Bbox2 computeRange(bool bWithVoronoi);
protected:
	Dat* pDat;
	std::ofstream outFile;
	std::vector<std::pair<Segment2,Color> > vSegments;
	std::vector<std::pair<Circle2,Color> > vCircles;
	std::vector<std::pair<Point2,Color> > vPoints;
	std::vector<std::pair<Triangle2,Color> > vTriangles;
	std::vector<std::pair<Label,Color> > vLabels;
	std::vector<std::pair<VoroCell2*,Color> > vVoroCells;
	void writeHeaderLines();
	int updateCtr;
	Bbox2 bbox;
	bool bFill;
	Point2 scaledPoint(const Point2 &p);
	double scaledDouble(const double &d);
	void changeColor(float r,float g,float b,float linewidth,bool bFill);
	void changeColor(const Color& c);
	void writeHeader(const char* title);
	void writeFooter();
	void writeLabel(Label l);
	void writeLine(const Point2& pSource,const Point2& pTarget);
	void writeTriangle(const Point2& p0_,const Point2& p1_,const Point2& p2_,bool bFill,double width);
	void writeTriangle(const Triangle2* pT,bool bFill_,double width);
	void writeVoroCell(VoroCell2* pVoroCell,bool bFill,double width);
	void writePoint(Point2& p1_,float size);
	void writeMark(Point2& p1_,float size);
	void writeCircle(const Point2& p1_,double radius,bool bFill);
	void periodicStroke();

	Color lastColor;
	//const char* filename;
	//std::vector<const char*> vHeaderLines;
	bool bFileClosed;
};


} // (namespace)
