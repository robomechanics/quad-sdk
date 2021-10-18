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

/// \file ConstraintSegment2.h
#pragma once
#include <set>

#include "common.h"
#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif

class Point2; // FWD
class ConstraintGraph2; // FWD
class ConstraintMgr; // FWD

/** \brief Constraint Insertion Strategy
* determines how a constraint edge shall be inserted:
*
* - CIS_CONSTRAINED_DELAUNAY inserts a segment without subdivision
* unless required (which is the case if existing vertices or constraint
* segments are crossed).
*
* All other constraint insertion strategies are deprecated and only
* kept for backwards compatibility. Their behavior can be achieved
* using ConstraintGraph2::makeDelaunay() and/or Fade_2D::drape().
* See also \p examples_25D/terrain.cpp.
*
* \note In former library versions the terms CIS_IGNORE_DELAUNAY
* and CIS_KEEP_DELAUNAY were used but these were misleading and
* are now deprecated. For backwards compatibility they are kept.
*/
enum ConstraintInsertionStrategy
{
	CIS_CONFORMING_DELAUNAY=0,
	CIS_CONSTRAINED_DELAUNAY=1,///< Deprecated
#if GEOM_PSEUDO3D==GEOM_TRUE
	CIS_CONFORMING_DELAUNAY_SEGMENT_LEVEL=2, ///< Deprecated
#endif
	CIS_KEEP_DELAUNAY=0, ///< Deprecated name
	CIS_IGNORE_DELAUNAY=1 ///< Deprecated
};

/** \brief A ConstraintSegment2 represents a Constraint Edge
 *
 * A ConstraintSegment2 can belong to more than one ConstraintGraph2
 * object, thus it is unoriented. But the ConstraintGraph knows the
 * orientation of its ConstraintSegment2's.
*/
class ConstraintSegment2
{
private:
	// ConstraintSegment2 objects are only made by ConstraintMgr/ConstraintGraph2
	ConstraintSegment2(Point2* p0_,Point2* p1_,ConstraintInsertionStrategy cis_);
	friend class ConstraintMgr;
	friend class ConstraintGraph2;
public:
	~ConstraintSegment2();
/** \brief Get the first endpoint
* \return the first vertex
*/
	CLASS_DECLSPEC
	Point2* getSrc() const;
/** \brief Get the second endpoint
* \return the second vertex
*/
	CLASS_DECLSPEC
	Point2* getTrg() const;
/// @private
	void kill();

/// @private
	bool dbg_hasDirection();

// Reanimate a constraint segment
/// @private
	void unkill();

/** \brief Check if the present ConstraintSegment2 is alive
 *
 * \return TRUE when the object is alive, FALSE otherwise
*/
	CLASS_DECLSPEC
	bool isAlive() const;

/** \brief Get the Constraint Insertion Strategy (CIS)
 *
* \return the constraint insertion strategy (CIS) of the present object
*/
	ConstraintInsertionStrategy getCIS() const;


/** \brief operator<(..)
* Compares the vertex pointers of the endpoints, not the length
*/
	CLASS_DECLSPEC
	bool operator<(const ConstraintSegment2& pOther) const;

/** \brief Split a constraint segment
*
* Splits the ConstraintSegment2 (which must be alive) at \p splitPoint.
*
* It may be impossible to represent a point on a certain line segment
* using floatingpoint arithmetic. Therefore it is highly recommended
* to split a ConstraintSegment2 object not just be inserting points
* into the triangulation but using the present method. It does not
* require that \p splitPoint is exactly on the segment.
*
* \note A splitted ConstraintSegment2 is dead and it has two child
* segments (which may also be dead and have children). The class is
* organized as a binary tree.
*/
	CLASS_DECLSPEC
	Point2* insertAndSplit(const Point2& splitPoint);

/** \brief Split a constraint segment
*
* internal use only (unless you do something very unusual)
*/
	CLASS_DECLSPEC
	bool split_combinatorialOnly(Point2* pSplit);
//*
//* internal use. Deprecated!
//*/
/// @private
	CLASS_DECLSPEC
	bool splitAndRemovePrev(const Point2& split);


//Sets a specific ConstraintGraph2 as owner of the current ConstraintSegment2.
/// @private
	void addOwnerRecursive(ConstraintGraph2* pOwner);

// Removes a specific ConstraintGraph2 as owner of the current ConstraintSegment2.
/// @private
	void removeOwner(ConstraintGraph2* pOwner);
	CLASS_DECLSPEC
/** \brief Get all children
* Recursively retrieve all children of the current ConstraintSegment2.
*/
	void getChildrenRec(std::vector<ConstraintSegment2*>& vChildConstraintSegments);
	CLASS_DECLSPEC
/** \brief Get the children and the split point
* Retrieve the two direct children of the current ConstraintSegment2 as well as the split point.
*/
	void getChildrenAndSplitPoint(ConstraintSegment2*& pCSeg0,ConstraintSegment2*& pCSeg1,Point2*& pSplitPoint);
	CLASS_DECLSPEC
	friend std::ostream &operator<<(std::ostream &stream, const ConstraintSegment2& cSeg);

/// @private
	CLASS_DECLSPEC
	size_t getNumberOfOwners() const;



//* A customer specific method, not thought for public use. If the
//* current constraint segment is a border segment, then the area of
//* the (non-existing) outside triangle can manually be deposited
//* here. This value is used by the extended meshing method when a
//* grow factor is given or ignored if not set.
/// @private
	CLASS_DECLSPEC
	void setAdjacentArea(double adjacentArea_);



//* A customer specific method, not thought for public use. Returns
//* a previously via setAdjacentArea(double adjacentArea_) deposited
//* value. If not explicitly set, DBL_MAX is returned.
/// @private
	CLASS_DECLSPEC
	double getAdjacentArea() const;


	int label;
protected:
/// @private
	std::set<ConstraintGraph2*> sOwners;

	Point2 *p0,*p1;
	ConstraintInsertionStrategy cis;
	bool bAlive;

	std::vector<ConstraintSegment2*> vChildren;
/// @private
	double adjacentArea;

	static int runningLabel;
};

} // NAMESPACE
