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

/// @file IsoContours.h
#pragma once
#if GEOM_PSEUDO3D==GEOM_TRUE

#include "Segment2.h"
#include <map>

#include "common.h"
#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif


struct TriNode; // FWD
class Triangle2; // FWD
class GeomTest; // FWD
class Segment2; // FWD
class Func_Rot3D; // FWD

/// @private
typedef std::multimap<Point2,Segment2*> MMPS;
/// @private
typedef MMPS::iterator MMPSIt;

/** \brief IsoContours uses a fast datastructure to compute intersections of
 * horizontal planes with a given list of triangles.
*
* \sa http://www.geom.at/terrain-triangulation/
*/
class IsoContours
{

public:

	CLASS_DECLSPEC
	explicit IsoContours(std::vector<Triangle2*>& vTriangles);

/** Experimental feature
 *
 * IsoContours can be used to create profiles (slices).
 *
 * @param vCorners contains 3*n points to specify n triangles, i.e.
 * it is a corners-list where 3 subsequent points define a triangle.
 * @param dirVec specifies the slice direction to compute profiles
 */
	CLASS_DECLSPEC
	IsoContours(std::vector<Point2>& vCorners,const Vector2& dirVec);

	CLASS_DECLSPEC
	~IsoContours();
/** Get the minimum height
 *
 * Returns the smallest z coordinate
 */
	CLASS_DECLSPEC
	double getMinHeight();
/** The the maximum height
 *
 * Returns the largest z-coordinate
 */
	CLASS_DECLSPEC
	double getMaxHeight();
/** Get Contours
 *
 * Computes the intersection of a horizontal plane at a certain height
 * with all triangles and returns a vector of assembled polygons and
 * polylines. The method works only for \p height values that do not
 * occur as heights of any of the vertices. It returns false in such
 * as case except \p bAutoPerturbate=true. In this case a tiny offset
 * is automatically added to \p height.
 *
 * @note Polylines that end in the middle of a terrain can not exist
 * and if you see such lines in the output then these are not only
 * individual line segments but extremely narrow polygons - enforced
 * by your geometric setting. You can numerically inspect those lines
 * to verify that.
 */
	CLASS_DECLSPEC
	bool getContours(double height,std::vector<std::vector<Segment2> >& vvContours,bool bVerbose,bool bAutoPerturbate=true);

/** Get Profile
 *
 * This is a new method to compute profiles i.e., to produce slices
 * orthogonal to a direction specified in the constructor
 * IsoContours(std::vector<Point2>& vCorners,const Vector2& dirVec)
 *
 */
	CLASS_DECLSPEC
	void getProfile(const Point2& p,std::vector<Segment2>& vSegmentsOut);
protected:
/// @private
	void init();
/// @private
	CLASS_DECLSPEC
	bool getIntersectionSegments(double height,std::vector<Segment2*>& vIntersectionSegments);
/// @private
	void createContours(MMPS& mmPS,std::vector<std::vector<Segment2> >& vvContours);
/// @private
	void getIntersection(Triangle2* pT,double height,int i,std::vector<Point2>& vIntersectionPoints);
/// @private
	TriNode* pTree;
	std::vector<Triangle2*> vTriangles;
/// @private
	GeomTest* pGeomPredicates;
/// @private
	std::set<double> sForbiddenHeights;
/// @private
	Func_Rot3D* pFRot;
/// @private
	std::vector<Point2*> vVtx; // Rotation version

private:

};


} // NAMESPACE FADE2D


#endif


