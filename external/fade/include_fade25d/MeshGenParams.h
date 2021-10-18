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

/// @file MeshGenParams.h
#pragma once
#include "common.h"
#include "Vector2.h"


#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif


class Triangle2; // Forward
class Zone2; // Forward
class Fade_2D; // Forward
class ConstraintSegment2; // Forward
/** @brief Unused parameter
 *
 * Empty template to avoid compiler warnings about unused function parameters
 */
template <typename T> inline void unusedParameter(const T&){}



/** \brief Parameters for the mesh generator
*
* This class serves as container for mesh generator parameters. Client
* code can provide a class which derives from MeshGenParams and which
* provides custom implementations of the getMaxTriangleArea(Triangle* pT)
* method or the getMaxEdgeLength(Triangle* pT) method in order to
* gain control over the local density of the generated mesh. When the
* meshing algorithm decides if a certain triangle T must be refined,
* then it calls these functions.
*
* \sa http://www.geom.at/advanced-mesh-generation/
*/
class CLASS_DECLSPEC MeshGenParams
{
public:
	explicit MeshGenParams(Zone2* pZone_):
#if GEOM_PSEUDO3D==GEOM_TRUE
		// *** These two parameters exist only in Fade2.5D ***
		pHeightGuideTriangulation(NULL),
		maxHeightError(DBL_MAX),
#endif
		pZone(pZone_),
		minAngleDegree(20.0),
		minEdgeLength(1e-3),
		maxEdgeLength(DBL_MAX),
		maxTriangleArea(DBL_MAX),
		bAllowConstraintSplitting(true),
		growFactor(DBL_MAX),
		growFactorMinArea(1e-3),
		capAspectLimit(10.0),
		gridLength(0.0),
		bKeepExistingSteinerPoints(true),
		command(0), // command is for development purposes
		psLockedConstraintSegments(NULL)
	{
		verify();
#if GEOM_PSEUDO3D==GEOM_TRUE
		gridVector=Vector2(1.0,0.0,0.0);
#else
		gridVector=Vector2(1.0,0.0);
#endif
	}

	virtual ~MeshGenParams();



/** \brief getMaxTriangleArea(Triangle2* pT)
 *
 * @param pT is a triangle for which the meshing algorithm checks if
 * it must be refined.
 *
 * The default implementation of the present class returns the value
 * maxTriangleArea (which is the default value DBL_MAX if not changed
 * by the user). This method can be overridden by the client software
 * in order to control the local mesh density.
 * \image html mesh-generator-customParameters.png "User Controlled Mesh Density, Area"
 * \image latex mesh-generator-customParameters.eps "User Controlled Mesh Density, Area" width=7cm
  */
	virtual double getMaxTriangleArea(Triangle2* pT)
	{
		unusedParameter(pT); // avoids useless compiler warnings
		return maxTriangleArea;
	}


/** \brief getMaxEdgeLength(Triangle2* pT)
 *
 * @param pT is a triangle for which the meshing algorithm checks if
 * it must be refined.
 *
 * The default implementation of the present class returns the value
 * maxEdgeLength (which is DBL_MAX if not changed by the user). This
 * method can be overridden by the client software in order to control
 * the local mesh density.
 * \image html mesh-generator-customParameters.png "User Controlled Mesh Density, Edge Length"
 * \image latex mesh-generator-customParameters.eps "User Controlled Mesh Density, Edge Length" width=7cm
*/
	virtual double getMaxEdgeLength(Triangle2* pT)
	{
		unusedParameter(pT); // avoids useless compiler warnings
		return maxEdgeLength;
	}


	// *** The following two parameters exist only in Fade2.5D ***
#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief pHeightGuideTriangulation
 *
 * When new vertices are created then their height (z-coordinate) is
 * usually computed from the existing triangles. In a situation where
 * an extra triangulation with more accurate heights exists this
 * extra triangulation cn be set als height guide triangulation. In
 * this case the z-coordinates are computed from the triangles of
 * the height guide triangulation.
 */
	Fade_2D* pHeightGuideTriangulation;

/** \brief maxHeightError
 *
 * If pHeightGuideTriangulation is set and the height error exceeds
 * locally maxHeightError then the triangulation is further refined.
 */
	double maxHeightError;
#endif




/** \brief Zone to be meshed.
*/
	Zone2* pZone;

/** \brief Minimum interior triangle angle
 *
 * Minimum interior angle: Default: 20.0, maximum: 30.0
*/
	double minAngleDegree;

/** \brief Minimum edge length
 *
 * Edges below the minimum length are not subdivided. This parameter
 * is useful to avoid tiny triangles. Default: 0.001
*/
	double minEdgeLength;

/** \brief Maximum edge length
 *
 * This value is returned by the default implementation of
 * getMaxEdgeLength(Triangle* pT). Larger edges are automatically
 * subdivided. If a custom implementation of getMaxEdgeLength(Triangle* pT)
 * is provided then this value is ignored. Default value: DBL_MAX.
 *
*/
	double maxEdgeLength;


/** \brief maxTriangleArea
 *
 * This value is returned by the default implementation of
 * getMaxTriangleArea(Triangle* pT). Larger triangles are
 * automatically subdivided. If a custom implementation of getMaxTriangleArea(Triangle* pT)
 * is provided then this value is ignored. Default value: DBL_MAX.
 */
	double maxTriangleArea;


/** \brief bAllowConstraintSplitting
 *
 * Defines if constraint segments can be splitted. Default: yes
*/
	bool bAllowConstraintSplitting;

/** \brief growFactor
 *
 * Limits the growth of adjacent triangles. The mesh is constructed such
 * that for any two adjacent triangles t0 and t1 (where t0 is the larger
 * one) area(t0)/area(t1) < growFactor. Recommendation: growFactor>5.0, Default: growFactor=DBL_MAX
*/
	double growFactor;

/** \brief growFactorMinArea
 *
 * The growFactor value is ignored for triangles with a smaller area
 * than growFactorMinArea. This value prevents generation of hundreds
 * of tiny triangles around one that is unusually small. Default: 0.001
 */
	double growFactorMinArea;

/** \brief capAspectLimit
 *
 * Limits the quotient edgeLength / height. Default value: 10.0
 */
	double capAspectLimit;


/** \brief gridVector
 *
 * When grid-meshing is used the grid is aligned to the \p gridVector.
 * By default \p gridVector is axis aligned.
 *
 * \if SECTION_FADE25D
 * \image html grid-mesh-angle.png "Grid Meshing along Vector2(1.0,0.3,0.0)"
 * \image latex grid-mesh-angle.eps "Grid Meshing along Vector2(1.0,0.3,0.0)" width=7cm
 * \else
 * \image html grid-mesh-angle.png "Grid Meshing along Vector2(1.0,0.3)"
 * \image latex grid-mesh-angle.eps "Grid Meshing along Vector2(1.0,0.3)" width=7cm
 * \endif
 */
	Vector2 gridVector;

/** \brief gridLength
 *
 * Set %gridLength > 0 to mesh large enough areas with grid points.
 * Border areas and narrow stripes where a grid does not fit are
 * automatically meshed using classic Delaunay methods. By default
 * %gridLength=0 (off).
 *
 * @note The length of the diagonals in the grid is sqrt(2)*gridLength
 * and the algorithm may automatically adapt the gridLength a bit such
 * that the grid fits better into the shape.
  * \image html grid-mesh.png "Grid Meshing axis aligned"
 * \image latex grid-mesh.eps "Grid Meshing axis aligned" width=7cm
 */
	double gridLength;

/** \brief Steiner points from previous refinements
 *
 * A previous call to refine() or refineAdvanced() may have created
 * Steiner points. These may be partially or entirely removed during
 * a later refinement call, even (!) if this later refinement takes
 * place in a different zone. It depends on your application if this
 * behavior is desired or not. Usually you want to preserve the points,
 * thus the default value of /p bKeepExistingSteinerPoints is true.
 */
	bool bKeepExistingSteinerPoints;

/** \brief Command
 *
 * A command for development, not for public use. Will vanish soon.
 */
	int command;

/** \brief Constraint Segments that shall not be splitted
 *
 * In case that some ConstraintSegment2 can be splitted and others
 * must not be splitted use \p bAllowConstraintSplitting=true and add
 * the ones that must not be splitted.
 */
	void addLockedConstraint(ConstraintSegment2* pConstraintSegment);



/// @private
	std::set<ConstraintSegment2*>* psLockedConstraintSegments;
/// @private
	void verify();
};


} // NAMESPACE
