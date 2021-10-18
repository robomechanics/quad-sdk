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
/// \file Color.h


#pragma once

#include "common.h"
#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif

/** \brief Predefined colors for convenience
*/
enum Colorname
{
	CRED,
	CGREEN,
	CBLUE,
	CBLACK,
	CPINK,
	CGRAY,
	CORANGE,
	CLIGHTBLUE,
    CLIGHTBROWN,
    CDARKBROWN,
    CPURPLE,
    COLIVE,
    CLAWNGREEN,
    CPALEGREEN,
    CCYAN,
	CYELLOW,
	CWHITE
};

/** \brief Color for visualization
* \see Visualizer2
*
*/
class CLASS_DECLSPEC Color
{
public:
/**
*
* @param r_ red
* @param g_ green
* @param b_ blue
* @param width_ linewidth
* @param bFill_ fill (default: \e false)
*
* @note bFill_=true has two meanings: Objects that can be filled
* (Triangle2, Circle2) are filled with the rgb-color but line segments
* get x-marks at their endpoints.
*
*/

	Color(double r_,double g_,double b_,double width_,bool bFill_=false);

/**
 *
 * For convenience predefined colors can be used.
 *
 * @param c is a predefined color name
 * @param width_ linewidth (default: \e 0.001)
 * @param bFill_ fill (default: \e false)
*
* @note bFill_=true has two meanings: Objects that can be filled
* (Triangle2, Circle2) are filled with the rgb-color but line segments
* get x-marks at their endpoints.
 */
	Color(Colorname c,float width_=0.001,bool bFill_=false);
	Color();

	bool operator<(const Color& other) const;
	bool operator!=(const Color& other) const;
	bool operator==(const Color& other) const;
	float r; ///< Red
	float g; ///< Green
	float b; ///< Blue
	float width; ///< Linewidth
	bool bFill; ///< Fill the shape or not
	friend std::ostream &operator<<(std::ostream &stream, const Color& c);
	static Colorname getNextColorName();
	static size_t currentColorName;
};



inline std::ostream &operator<<(std::ostream &stream, const Color& c)
{
	stream<<"Color (r,g,b): "<<c.r<<","<<c.g<<","<<c.b<<", linewidth="<<c.width<<", fill="<<c.bFill;
	return stream;
}


} // (namespace)
