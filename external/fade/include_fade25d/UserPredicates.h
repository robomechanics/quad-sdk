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
/// @file UserPredicates.h

#pragma once
#include "common.h"
#include "Triangle2.h"

#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif


/** \brief User-defined predicate (deprecated)
 *
 * This class is deprecated in favor of PeelPredicateTS. It is kept
 * for backwards compatibility.
 *
*/
class CLASS_DECLSPEC UserPredicateT
{
public:
	UserPredicateT()
	{}

	virtual ~UserPredicateT()
	{
	}
	virtual bool operator()(const Triangle2*)=0;
};

/** \brief User-defined peel predicate
 *
 * This class is the successor of the deprecated (but still valid)
 * UserPredicateT. In contrast to UserPredicateT the operator()
 * receives also a set of current triangles to enable border-edge
 * tests.
 *
 * \sa https://www.geom.at/mesh-improvements/
*/
class CLASS_DECLSPEC PeelPredicateTS
{
public:
	PeelPredicateTS()
	{}

	virtual ~PeelPredicateTS()
	{
	}
	virtual bool operator()(const Triangle2*,std::set<Triangle2*>* pCurrentSet)=0;
};



} // NAMESPACE
