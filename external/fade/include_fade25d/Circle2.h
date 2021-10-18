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


/// @file Circle2.h
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

/** \brief Circle for visualization
* \see Visualizer2
*
*/

class Circle2
{
public:
/** \brief Constructor
*
* @param x is x-coordinate of the center
* @param y is y-coordinate of the center
* @param sqRadius_ is the squared radius of the circle
*
* @warning The method expects the \e squared radius
*/
	CLASS_DECLSPEC
	Circle2(double x,double y,double sqRadius_);
/** \brief Constructor
*
* @param center_ center of the circle
* @param sqRadius_ squared radius of the circle
*
* @warning The method expects the \e squared radius
*/
	CLASS_DECLSPEC
	Circle2(const Point2& center_,double sqRadius_);
	CLASS_DECLSPEC
	~Circle2();
/** \brief Get the radius of the circle
*
* \return the radius
*/
	CLASS_DECLSPEC
	double getRadius();
/** \brief Get the squared radius of the circle
*
* \return the squared radius
*/
	CLASS_DECLSPEC
	double getSqRadius();
/** \brief Get the center of the circle
*
* \return a Point2 which represents the center
*/
	CLASS_DECLSPEC
	Point2 getCenter();
	CLASS_DECLSPEC
	friend std::ostream &operator<<(std::ostream &stream, Circle2 b);
protected:
	Point2 center;
	double sqRadius;
};

} // (namespace)
