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

/// @file Label.h
#pragma once


#include "Point2.h"
#include "common.h"
#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif

struct LDat; // FWD

/**  \brief Label is a Text-Label for Visualization
*
* \see Visualizer2 where Label objects are used for visualizations
*/


class CLASS_DECLSPEC Label
{

public:
/** \brief Constructs a Text-%Label
*
* @param p_ is the point where the label appears
* @param s_ is the text to be shown
* @param bWithMark_ switches between text-only and text-with-mark
* @param fontSize_
*/
	Label(const Point2& p_,const char* s_,bool bWithMark_=true,int fontSize_=8);
	Label(const Label& other);
	Label& operator=(const Label& other);
	~Label();
	const char* getCS() const;
	// Data
	LDat* pDat;
	Point2 p;
	bool bWithMark;
	int fontSize;

};



} // (namespace)
