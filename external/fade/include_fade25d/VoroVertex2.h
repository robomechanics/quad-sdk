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


/** @file VoroVertex2.h
 *
 * Voronoi vertex
 */
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

class Voronoi2Impl; // FWD

/** @brief Voronoi vertex
 *
 * This class represents a vertex of the Voronoi diagram. A Voronoi
 * vertex is the circumcenter of a certain triangle of the dual
 * Delaunay triangulation.
 */
class CLASS_DECLSPEC VoroVertex2
{
public:
/// @private
	VoroVertex2(Voronoi2Impl* pImpl_,Triangle2* pT_);

/** @brief Is alive
 *
 * The Voronoi diagram changes dynamically when points are inserted
 * or removed from the dual Delaunay triangulation.
 *
 * @return whether the present Voronoi vertex is still valid
 */
	bool isAlive() const;
/** @brief Get Point
 *
 * @return the Voronoi vertex as a Point2
 */
	Point2 getPoint();
/** @brief Get dual triangle
 *
 * A Voronoi vertex is the circumcenter of a certain triangle in the
 * dual Delaunay triangulation.
 *
 * @return the corresponding Delaunay triangle
 */

	Triangle2* getDualTriangle() const;
protected:
/// @private
	bool isSaved() const;
/// @private
	Voronoi2Impl* pImpl;
/// @private
	CircumcenterQuality ccq;
/// @private
	void computeCC(bool bForceExact);
/// @private
	Point2 voroPoint;
/// @private
	Triangle2* pT;
/// @private
	Point2* aDelVtx[3];

};


} // (namespace)


