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

/** @file VoroCell2.h
 *
 * Voronoi cell
 */

#pragma once
#include "common.h"
#include "Point2.h"
#include "Bbox2.h"

#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif

class Voronoi2Impl; // FWD
class VoroVertex2; // FWD

/** @brief Voronoi cell
 *
 * This class represents a Voronoi cell. A Voronoi cell corresponds
 * to a certain site which is also a vertex of the dual Delaunay triangulation.
 */
class CLASS_DECLSPEC VoroCell2
{
public:
	/// @private
	VoroCell2(Point2* pSite_,Voronoi2Impl* pOwner_);

	/** Set custom cell-index
	 *
	 * Use this method to associate Voronoi cells with your own
	 * data structures or to assign labels for a visualization.
	 *
	 * @param customCellIndex_ is an arbitrary integer
	 *
	 * \see getCustomCellIndex()
	 */
	void setCustomCellIndex(int customCellIndex_);
	/** Get custom cell-index
	 *
	 * @return the custom cell index that has been set before or -1
	 * when no custom cell index has been set.
	 *
	 * \see setCustomCellIndex()
	 */
	int getCustomCellIndex() const;

	/** Get Voronoi vertices
	 *
	 * Used to retrieve the Voronoi vertices of the cell.
	 *
	 * @param [out] vVoroVertices contains VoroVertex2 objects in counterclockwise order.
	 *
	 * @return whether the cell is finite
	 *
	 * \see getBoundaryPoints()
	 */
	bool getVoronoiVertices(std::vector<VoroVertex2*>& vVoroVertices) const;

	/** Get boundary points
	 *
	 * Use this method to retrieve the Voronoi vertices of the
	 * present cell as Point2.
	 *
	 * @param [out] vPoints contains the boundary points in
	 * counterclockwise order.
	 * @param [out] pvInfiniteDirections can optionally be used to
	 * retrieve the directions of the infinite Voronoi edges: When a
	 * pointer to a vector is provided for \p pvInfiniteDirections and
	 * the cell is finite, then the first stored Vector2 describes the
	 * direction of the infinite edge at vPoints[0] and the second
	 * one the direction of the infinite edge at vPoints.back().
	 * @return whether the cell is finite
	 *
	 * \see getVoronoiVertices()
	 */
	bool getBoundaryPoints(
							std::vector<Point2>& vPoints,
							std::vector<Vector2>* pvInfiniteDirections=NULL
							) const;

	/** Get incident triangles
	 *
	 * The site of the present Voronoi cell is also a vertex of
	 * the dual Delaunay  triangulation.
	 *
	 * @param vIncTriangles is used to return the the Delaunay triangles
	 * incident to the site of the cell in counterclockwise order.
	 *
	 * @return whether the cell is finite.
	 *
	 */
	bool getIncidentTriangles(std::vector<Triangle2*>& vIncTriangles) const;
	/** Get neighbor sites
	 *
	 * @param vSites is used to return the sites of the adjacent cells
	 * in counterclockwise order.
	 *
	 * @return whether the cell is finite.
	 */
	bool getNeighborSites(std::vector<Point2*>& vSites) const;
	/** Get adjacent Voronoi cells
	 *
	 * @param vAdjacentCells is used to return the neighbor cells in
	 * counterclockwise order.
	 *
	 * @return whether the cell is finite.
	 */
	bool getAdjacentVCells(std::vector<VoroCell2*>& vAdjacentCells) const;
	/** Is finite cell
	 *
	 * @return whether the cell is finite
	 */
	bool isFinite() const;
	/** Get site
	 *
	 * @return the site of the cell, which is also a vertex of the
	 * dual Delaunay triangulation
	 */
	Point2* getSite() const;
	/** Get the centroid and area
	 *
	 * @param [out] centroid
	 *
	 * @return the area of the cell if it is finite OR -1.0 for an
	 * infinite cell.
	 */
	double getCentroid(Point2& centroid) const;
	/** Get the area
	 *
	 * @return the area of the cell if it is finite or -1.0 for an
	 * infinite cell.
	 */
	double getArea() const;

protected:
	/// @private
	Point2* pSite;
	/// @private
	Voronoi2Impl* pOwner;
	/// @private
	int customCellIndex;

};


} // (namespace)


