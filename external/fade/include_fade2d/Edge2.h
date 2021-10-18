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
/// @file Edge2.h
#pragma once

#include "common.h"
#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif

class Triangle2; // FWD
class Point2; // FWD
/**  \brief Edge2 is a directed edge
*/
class CLASS_DECLSPEC  Edge2
{
public:
	Edge2();
	Edge2(const Edge2& e_);
	/**  \brief Constructor
	 *
	 * @param pT is the triangle from which the edge is constructed
	 * @param oppIdx_ is intra-triangle-index of the opposite vertex (of the edge) in pT
	 *
	 * The orientation of the constructed Edge2 is counterclockwise (CCW)
	 * with respect to \p pT. Example: Edge2(pT,0) creates an edge from
	 * pT->getCorner(1) to pT->getCorner(2).
	 *
	*/
	Edge2(Triangle2* pT,int oppIdx_);
	Edge2& operator=(const Edge2& other);

	~Edge2();

	/**  \brief operator<()
	 *
	 * operator<() does NOT compare edge lengths but the associated
	 * triangle pointers and intra-triangle indices. This is useful
	 * when edges are used in STL containers.
	*/
	bool operator<(const Edge2& e) const
	{
		if(pT<e.pT) return true;
		if(pT>e.pT) return false;
		if(oppIdx<e.oppIdx) return true;
		return false;
	}

	/**  \brief operator==()
	 *
	 * operator==() compares oriented edges, i.e., it returns only true
	 * when the two edges have been made from the same triangle and the
	 * same intra-triangle-index i.e., an edge with two adjacent
	 * triangles has two Edge2 objects, one in each direction.
	*/
	bool operator==(const Edge2& e) const
	{
		return(pT==e.pT && oppIdx==e.oppIdx);
	}
	/**  \brief operator!=()
	 *
	 * operator!=() returns true if the compared edges are different.
	 * Be aware that edges are directed and therefore two adjacent
	 * triangles do not share the same Edge2.
	*/

	bool operator!=(const Edge2& e) const
	{
		return((pT!=e.pT || oppIdx!=e.oppIdx));
	}

	/** \brief Get the source point
	 *
	 * \return the source point of the edge, i.e. pT->getCorner((oppIdx+1)%3)
	 */
	Point2* getSrc() const;
	/** \brief Get the target point
	 *
	 * \return the target point of the edge, i.e. pT->getCorner((oppIdx+2)%3)
	 */
	Point2* getTrg() const;

	/** \brief Get the endpoints
	 *
	 * returns the source point of the edge as \p p1 and the target point as \p p2
	 */
	void getPoints(Point2*& p1,Point2*& p2) const;
#if GEOM_PSEUDO3D==GEOM_TRUE
	/** Get the 2D length
	 *
	 * \return the 2D length of the edge, the z-coordinate is ignored
	 */
#else
	/** Get the length
	 *
	 * \return the length of the edge
	 */
#endif
	double getLength2D() const;

#if GEOM_PSEUDO3D==GEOM_TRUE
	/** Get the 2.5D length
	 *
	 * \return the 2.5D length of the edge
	 */
	double getLength25D() const;
#endif

	/** Get the triangle
	 *
	 * \return the triangle whose directed edge the present edge is
	 */
	Triangle2* getTriangle() const;

	/** Get the opposite index
	 *
	 * \return the intra-triangle-index of the opposite vertex
	 */
	int getIndex() const;

	/** Get the triangles
	 *
	 * \return the two adjacent triangles of the present edge along with
	 * their intra-triangle-indices
	 *
	 * \param pT0 is used to return the triangle whose directed edge the present edge is
	 * \param idx0 is the opposite intra-triangle-index in pT0 of the present edge
	 * \param pT1 is the other adjacent triangle at the present edge (or NULL)
	 * \param idx1 is the intra-triangle index of the present edge in pT1 (or -1)
	 *
	 */
	void getTriangles(Triangle2*& pT0,Triangle2*& pT1,int& idx0,int& idx1) const;
	friend std::ostream &operator<<(std::ostream &stream, const Edge2& e);

protected:
	Triangle2* pT;
	int oppIdx;
};

/// @private
struct Func_ltUndirected
{
	bool operator()(const Edge2& eA,const Edge2& eB) const
	{
		Point2 *pA1,*pA2,*pB1,*pB2;
		eA.getPoints(pA1,pA2);
		eB.getPoints(pB1,pB2);
		if(pA1>pA2) std::swap(pA1,pA2);
		if(pB1>pB2) std::swap(pB1,pB2);

		if(pA1<pB1) return true;
		if(pA1>pB1) return false;
		return pA2<pB2;
	}
};

/** \brief Functor to sort edges by 2d length (ascending)
*/
struct Func_ltEdge2D
{
	bool operator()(const Edge2& e0,const Edge2& e1) const
	{
		if(e0.getLength2D()<e1.getLength2D()) return true;
			else return false;
	}
};
/** \brief Functor to sort edges by 2d length (descending)
*/
struct Func_gtEdge2D
{
	bool operator()(const Edge2& e0,const Edge2& e1) const
	{
		if(e0.getLength2D()>e1.getLength2D()) return true;
			else return false;
	}
};


#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Functor to sort edges by 2.5d length (ascending)
*/
struct Func_ltEdge25D
{
	bool operator()(const Edge2& e0,const Edge2& e1) const
	{
		if(e0.getLength25D()<e1.getLength25D()) return true;
			else return false;
	}
};
#endif




} // (namespace)
