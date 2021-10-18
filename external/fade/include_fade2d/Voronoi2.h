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



/** @file Voronoi2.h
 *
 * Voronoi diagram
 */

#pragma once
#include "common.h"
#include "VoroVertex2.h"
#include "VoroCell2.h"


#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif


class Dt2; // FWD
class Voronoi2Impl; // FWD
class Visualizer2; // FWD

/** @brief Voronoi diagram
 *
 * This class represents a Voronoi diagram. A Voronoi diagram is the
 * dual graph of a Delaunay triangulation i.e.,
 *
 * - Each Voronoi cell contains the area closest to its site which is a Delaunay vertex
 * - Each Voronoi edge has a dual edge in the Delaunay triangulation at an angle of 90 degrees to it.
 * - Each Voronoi vertex is the circumcenter of a Delaunay triangle
 *
 * \see https://en.wikipedia.org/wiki/Voronoi_diagram
*/
class CLASS_DECLSPEC Voronoi2
{
public:
	/// @private
	explicit Voronoi2(Dt2* pDt);
	/// @private
	~Voronoi2();

	/** \brief Get the Voronoi cells adjacent to a Voronoi edge
	 *
	 * @param pVoroVtx0,pVoroVtx1 are the Voronoi vertices that define the Voronoi edge
	 * @param pCell0,pCell1 are used to return the adjacent Voronoi cells or NULL if
	 * the command fails.
	 *
	 * @return true when the command succeeds or false otherwise i.e.,
	 * when (pVoroVtx0,pVoroVtx1) is not an edge of the Voronoi diagram.
	 */
	bool getVCellsAtVEdge(VoroVertex2* pVoroVtx0,VoroVertex2* pVoroVtx1,VoroCell2*& pCell0,VoroCell2*& pCell1);

	/** \brief Is valid
	 *
	 * @return whether the Voronoi diagram is ready for use. This is
	 * the case as soon 3 sites exist which are not collinear.
	 */
	bool isValid() const;



	/** \brief Get all Voronoi cells
	 *
	 * Use this method to retrieve all finite and infinite Voronoi
	 * cells.
	 *
	 * @param [out] vVoronoiCells
	 */
	void getVoronoiCells(std::vector<VoroCell2*>& vVoronoiCells);

	/** \brief Get Voronoi cell
	 *
	 * Use this method to retrieve the Voronoi cell of a specific
	 * site.
	 *
	 * @param [in] pSite
	 *
	 * @return the VoroCell2 of \p pSite.
	 *
	 */
	VoroCell2* getVoronoiCell(Point2* pSite);

	/** \brief Draw the Voronoi diagram
	 *
	 * @param filename is the output *.ps filename
	 * @param bVoronoi draw the edges of the Voronoi diagram (default: true)
	 * @param bCellColors use background colors for the Voronoi cells (default: true)
	 * @param bSites draw the sites (default: true)
	 * @param bDelaunay draw the Delaunay triangles (default: true)
	 * @param bCellLabels show cell labels (or -1 if not assigned) (default: false)
	 *
	 * This method does automatically crop the viewport to twice the
	 * range of the sites. Thus very large and infinite cells appear
	 * clipped.
	 */
	void show(
				const char* filename,
				bool bVoronoi=true,
				bool bCellColors=true,
				bool bSites=true,
				bool bDelaunay=true,
				bool bCellLabels=false
				);

	/** \brief Draw the Voronoi diagram
	 *
	 * @param pVisualizer is the Visualizer2 object to be used
	 * @param bVoronoi draw the edges of the Voronoi diagram (default: true)
	 * @param bCellColors use background colors for the Voronoi cells (default: true)
	 * @param bSites draw the sites (default: true)
	 * @param bDelaunay draw the Delaunay triangles (default: true)
	 * @param bCellLabels show cell labels (or -1 if not assigned) (default: false)
	 *
	 * @note This method only clips infinite cells. But finite cells
	 * can also be very large. Call Visualizer2::setLimit() to specify
	 * the range of interest.
	 */
	void show(
				Visualizer2* pVisualizer,
				bool bVoronoi=true,
				bool bCellColors=true,
				bool bSites=true,
				bool bDelaunay=true,
				bool bCellLabels=false
				);

	/** \brief Locate a Voronoi Cell
	 *
	 * This is a high performance method to locate the Voronoi cell
	 * of an arbitrary \p queryPoint
	 *
	 * @param [in] queryPoint
	 *
	 * @return the Voronoi cell that contains \p queryPoint or NULL
	 * if the Voronoi diagram is invalid.
	 */
	VoroCell2* locateVoronoiCell(const Point2& queryPoint);

	/** \brief Get the Voronoi vertex of a triangle
	 *
	 * Get the Voronoi vertex of a certain dual Delaunay triangle \p pT
	 *
	 * @param [in] pT
	 *
	 * @return the Voronoi vertex that corresponds to \p pT
	 */
	VoroVertex2* getVoronoiVertex(Triangle2* pT) ;
protected:
	Voronoi2Impl* pImpl;
private:
	Voronoi2(Voronoi2&);
	Voronoi2& operator=(Voronoi2&);

};


} // (namespace)
