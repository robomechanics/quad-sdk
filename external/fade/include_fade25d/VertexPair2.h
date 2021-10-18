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
#pragma once

#include "common.h"
#include "Point2.h"
#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif

/** \brief VertexPair2
* A struct that holds two vertex pointers p0, p1 such that p0<=p1.
*/
struct CLASS_DECLSPEC VertexPair2
{
public:
	VertexPair2(Point2* p,Point2* q):p0(p),p1(q)
	{
		if(p0>p1) std::swap(p0,p1);
	}

	VertexPair2():p0(NULL),p1(NULL)
	{
	}

/** \brief Get the squared 2D length
*/
	double getSqLen2D() const
	{
		return sqDistance2D(*p0,*p1);
	}

#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Get the squared 2.5D length
*/

	double getSqLen25D() const
	{
		return sqDistance25D(*p0,*p1);
	}
#endif


/** \brief Equality operator
 *
 * Compares the two vertex pointers
*/

	bool operator==(const VertexPair2& other) const
	{
		if(p0==other.p0 && p1==other.p1) return true;
		return false;
	}

/** \brief Less than operator (pointers)
*
* Compares the two vertex pointers
*/

	bool operator<(const VertexPair2& other) const
	{
		// Known: p0<p1 in both pairs
		if(p0<other.p0) return true;
		if(p0>other.p0) return false;
		if(p1<other.p1) return true;
		return false;
	}


	// Data
	Point2* p0;
	Point2* p1;
};


} // (namespace)
