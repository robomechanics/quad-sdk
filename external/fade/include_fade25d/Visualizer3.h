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

/// @file Visualizer3.h
#pragma once



#include "common.h"
#if GEOM_PSEUDO3D==GEOM_TRUE

#include "Point2.h"
#include "Segment2.h"
#include "VertexPair2.h"
#include "Edge2.h"
namespace GEOM_FADE25D
{


/** \brief Visualizer3 is a 3D scene writer for the Geomview viewer
*/
class CLASS_DECLSPEC Visualizer3
{
public:
	static const char * const CLIGHTBLUE;
	static const char * const CDARKBLUE;
	static const char * const CYELLOW;
	static const char * const CPINK;
	static const char * const CBLACK;
	static const char * const CLIGHTBROWN;
	static const char * const CDARKBROWN;
	static const char * const CORANGE;
	static const char * const CPURPLE;
	static const char * const CGRAY;
	static const char * const CLIGHTGRAY;
	static const char * const CRED;
	static const char * const CGREEN;
	static const char * const CWHITE;
	static const char * const CRIMSON;
	static const char * const CDARKORANGE;
	static const char * const CGOLDENROD;
	static const char * const COLIVE;
	static const char * const CLAWNGREEN;
	static const char * const CGREENYELLOW;
	static const char * const CPALEGREEN;
	static const char * const CMEDSPRINGGREEN;
	static const char * const CLIGHTSEAGREAN;
	static const char * const CCYAN;
	static const char * const CSTEELBLUE;
	static const char * const MIDNIGHTBLUE;
	static const char * const CWHEAT;
	static const char * getColor(int i);
	static const char * getNextColor();
	static const char * getNextColorAndName(const char*&);
	explicit Visualizer3(const char* name);
	~Visualizer3();
	void closeFile();
	void writeNormals(const std::vector<Triangle2*>& vT,double scale);
	void writePoints(const std::vector<Point2*>& vPoints,unsigned linewidth,const char* color) ;
	void writePoints(const std::vector<Point2>& vPoints,unsigned linewidth,const char* color) ;

	void writePoint(const Point2& p,unsigned linewidth,const char* color);
	void writeSegment(const Point2& src,const Point2& trg,const char* color,bool bWithEndPoints=false);
	void writeSegments(const std::vector<Segment2>& vSegments,const char* color,bool bWithEndPoints=false);
	void writeSegments(const std::vector<Edge2>& vSegments,const char* color,bool bWithEndPoints=false);
	void writeVertexPairs(const std::vector<VertexPair2>& vVertexPairs,const char* color);
	void writeCubes(const std::vector<Point2>& vPoints,const char* color);
	void writeTriangles(const std::vector<Triangle2*>& vT,const char* color,bool bWithNormals=false);
	void writeTriangles(const std::vector<Point2>& vTriangleCorners,const char* color,bool bWithNNV);
	void writeTriangle(const Triangle2& t,const char* color);
	void writeTriangle(const Point2& p0,const Point2& p1,const Point2& p2,const char* color);
	void writeBall(Point2& p,double radius);
	void setBackfaces(bool bWithBackfaces_);
private:
	void startList(size_t numPoints,size_t numTriangles,bool bWithEdges);
	void endList();
	std::ofstream* pOutFile;
	static int nextColor;
	bool bWithBackfaces;
};

} // NAMESPACE FADE25D

#elif GEOM_PSEUDO3D==GEOM_FALSE
#else
#error GEOM_PSEUDO3D is not defined
#endif

