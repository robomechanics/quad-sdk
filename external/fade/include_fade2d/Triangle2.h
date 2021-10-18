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

/// @file Triangle2.h
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

/** @brief CircumcenterQuality
 */
enum CircumcenterQuality
{
	CCQ_INIT, ///< Init value
	CCQ_INEXACT,  ///< Double precision computation, the result is accurate enough
	CCQ_EXACT, ///< Computation with multiple-precision arithmetic, the result is exact (apart from tiny quantization errors)
	CCQ_OUT_OF_BOUNDS ///< Computation with multiple-precision arithmetic, but the result is not representable with double precision coordinates
};

/** \brief Triangle
*
* Triangle2 is a triangle in the Fade_2D triangulation. It holds three Point2
* pointers to its corners. The corners are numbered in counterclockwise order.
* We refer to these indices as intra-triangle-indices.
*
* Each triangle has three neighbors which can be accessed through intra-triangle-indices:
* The i-th neighbor triangle of a certain triangle T is the one which shares an edge with T such
* that this edge does not include the i-th corner of T.
*
* \image html neig.jpg "Indices and neighborships, \e tb is the 0-th neighbor of \e ta and \e ta is the 2nd neighbor of \e tb."
* \image latex neig.eps "Indices and neighborships, \e tb is the 0-th neighbor of \e ta and \e ta is the 2nd neighbor of \e tb." width=6cm
*
* \see TriangleAroundVertexIterator to find out how to access all triangles incident
* to a certain vertex.
*/
class CLASS_DECLSPEC  Triangle2
{
public:
/** \brief Constructor
*
*/
	Triangle2()
	{
		aVertexPointer[0]=NULL;
		aVertexPointer[1]=NULL;
		aVertexPointer[2]=NULL;
		aOppTriangles[0]=NULL;
		aOppTriangles[1]=NULL;
		aOppTriangles[2]=NULL;

	} // Never used!




/** \brief Get the \e i-th corner of the triangle
*
* \return a pointer to the i-th corner point of the triangle.
*
* \image html corners.jpg "Intra triangle indices are ordered counterclockwise"
* \image latex corners.eps "Intra triangle indices are ordered counterclockwise" width=4cm
*
* @param ith is the intra-triangle-index, ith={0,1,2}.
*/
	Point2* getCorner(const int ith) const;

///< @private
/* Get the dual Voronoi vertex - DEPRECATED, but kept for
 * backwards compatibility - see the new method getCircumcenter()
*
* @return a std::pair<Point2,bool>, where the first component is the
* circumcenter of the triangle (i.e., the dual Voronoi vertex) and
* the second component is a boolean value which is normally true to
* indicate the computation is reliable.
*
* @param bForceExact can be used to enforce exact computation with
* multiple-precision arithmetic. By default the computationally
* more expensive multiple-precision arithmetic is only used for
* bad-shaped triangles.
*
*
* \if SECTION_FADE25D
* @note The z-coordinate of the returned point is always 0 but you
* can use Fade_2D::getHeight(..) to determine the height.
* \endif
*
* @attention Attention: The circumcenter of a nearly collinear triangle
* can have coordinates beyond the bounds of floating point arithmetic.
* Fade could compute the circumcenter with multiple-precision artihmetic
* but it could not represent the result as a point with floatingpoint
* coordinates. Thus the involved coordinates are DBL_MAX in such a case
* and the boolean return value is then false.
*
* @note Such extreme numeric cases can easily be avoided by insertion
* of four dummy vertices around the triangulation, e.g., at coordinates
* 10 times larger than the domain of the data points. This will
* automatically restrict the Voronoi diagram of the data points to
* this range.
*/
std::pair<Point2,bool> getDual(bool bForceExact=false) const; // DEPRECATED, see getCircumcenter()

/** \brief Get the circumcenter of the triangle
*
* @param [out] ccq holds the quality of the computed point and is one
* of CCQ_INEXACT, CCQ_EXACT and CCQ_OUT_OF_BOUNDS.
*
* @param [in] bForceExact forces exact computation with multiple-precision
* arithmetic. When bForceExact=false, then the faster double-precision
* arithmetic is used for good shaped triangles.
*
* \if SECTION_FADE25D
* @return the circumcenter of the triangle. The z-coordinate is 0.0.
* You can use Fade_2D::getHeight(..) to determine the height.
* \else
* @return the circumcenter of the triangle
* \endif
*
* @attention Attention: The circumcenter of a nearly collinear triangle
* can have extremely large coordinates. Fade computes the circumcenter
* with multiple-precision artihmetic in this case but the result might
* nevertheless not be exact because it too large for double-precision
* coordinates. In such cases a finite point is returned and \p ccq
* returns CCQ_OUT_OF_BOUNDS. You can avoid such extreme numeric cases
* easily: Just insert four dummy vertices around the triangulation
* at coordinates 10 times larger than the domain of the data points
* because this restricts the Voronoi cells of the data points to
* this range.
*/
Point2 getCircumcenter(CircumcenterQuality& ccq,bool bForceExact=false) const;



/** \brief Get the barycenter of a triangle
*
* @return the barycenter of the triangle.
*/
	Point2 getBarycenter() const;


#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Get the normal vector of a triangle
*
* @return the normalized normal vector
*/
	Vector2 getNormalVector() const;



#endif

/** \brief Get interior 2D angle
 *
 * \if SECTION_FADE25D
 * Note: The getInteriorAngle() method is deprecated and replaced by getInteriorAngle2D()
 * and getInteriorAngle25D()
 * \else
 * Note: The getArea() method is deprecated and replaced by getInteriorAngle2D()
 * to keep the names consistent.
 * \endif
 *
 * @return the interior 2D angle at the ith vertex
*/
double getInteriorAngle2D(int ith) const;

#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Get interior 2.5D angle
 *
 * @return the interior 2.5D angle at the ith vertex
*/
double getInteriorAngle25D(int ith) const;
#endif



/** \brief Get 2D Area
 *
 * Returns the 2D area of the triangle.
 *
 * \if SECTION_FADE25D
 * Note: The getArea() method is deprecated and replaced by getArea2D()
 * and getArea25D()
 * \else
 * Note: The getArea() method is deprecated and replaced by getArea2D()
 * to keep the names consistent.
 * \endif
 */
	double getArea2D() const;




#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Get 2.5D Area
 *
 * Returns the 2.5D area of the triangle.
 *
 * Note: The getArea() method is deprecated and replaced by getArea2D()
 * and getArea25D()
 */
	double getArea25D() const;
#endif


/** \brief Get the \e i-th neighbor triangle
*
* Returns the \e i-th neighbor triangle, i.e. the one opposite to the
* \e i-th corner.
* \image html getNeig.jpg "Neighbors of a triangle"
* \image latex getNeig.eps "Neighbors of a triangle" width=6cm
*
*
* @param ith is the intra-triangle-index of the opposite corner of @e *this
* @return the i-th neighbor triangle, i.e. the one opposite to the
* i-th vertex or NULL if no neighbor triangle exists which is the case
* at the convex hull edges of the triangulation.
*/
	Triangle2* getOppositeTriangle(const int ith) const;


/** \brief Get the index of \e p in the triangle
*
* \image html getITI.jpg "Intra triangle index of a vertex pointer" width=10cm
* \image latex getITI.eps "Intra triangle index of a vertex pointer" width=4cm
*
* @param p is a pointer to a vertex in @e *this
*
* @return the intra-triangle-index 0,1 or 2 of @e p in @e *this
*
*
*/
	int getIntraTriangleIndex(const Point2* p) const;
/** \brief Get the neighbor index of \e pTriangle
*
*
* \image html getITI_t.jpg "\e pTriangle is the 0-th neighbor of \e *this"
* \image latex getITI_t.eps "pTriangle is the 0-th neighbor of *this" width=8cm
*
*
* @param pTriangle is a neighbor triangle of *this.
*
* @return the intra-triangle-index of the vertex in \e *this which is opposite (i.e., does not
* touch the neighbor) \e pTriangle.
*/
	int getIntraTriangleIndex(const Triangle2* pTriangle) const;

/** \brief Get the index of \p (p0,p1)
*
* @return the index of the edge \p (p0,p1) in the triangle
*/

	int getIntraTriangleIndex(const Point2* p0,const Point2* p1) const;


///** \brief Method for internal use
 //*
 //*
 //*  Internal use
//*/
	//bool getState() const;

/** \brief Squared edge length
*
* Returns the squared length of the \e ith edge.
* \if SECTION_FADE25D
* This method ignores the z-coordinate.
* \endif
*/
	double getSquaredEdgeLength2D(int ith) const;

/** @private
 */
	double getSquaredEdgeLength(int ith) const; // Deprecated, use getSquaredEdgeLength2D()

#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Squared edge length
*
* Returns the squared length of the \e ith edge.
*/
	double getSquaredEdgeLength25D(int ith) const;
#endif
//** \brief Method for internal use
 //*
 //* Internal use
//*/
	//void setState(bool bState_);
/** \brief Set the \e i-th neighbor triangle
*
* \image html getITI_t.jpg "Make @e pTriangle the 0-th neighbor of @e *this"
* \image latex getITI_t.eps "Make pTriangle the 0-th neighbor of *this" width=8cm
*
*
* @param ith is the index of the corner of \e *this which does not touch \e pTriangle
* @param pTriangle is a pointer to the triangle which shares two corners with \e *this
*/
	void setOppTriangle(const int ith, Triangle2* pTriangle);
/** \brief Set all corners
*/
	void setProperties( Point2* pI, Point2* pJ, Point2* pK);

/** \brief Clear all corners and neighbor pointers
 */
	void clearProperties();

/** \brief Set all corners and neighbor triangles
*/
	void setPropertiesAndOppT(Point2* pI, Point2* pJ, Point2* pK,Triangle2* pNeig0,Triangle2* pNeig1,Triangle2* pNeig2);

/** \brief Set the \e i-th corner
*/
	void setVertexPointer(const int ith, Point2* pp);

/** \brief Has vertex
 *
 * \return if \p pVtx is a corner of the triangle
 */
	bool hasVertex(Point2* pVtx) const;

/** \brief Has vertex
 *
 * \return if \p vtx is a corner of the triangle
 */
	bool hasVertex(const Point2& vtx) const;

/** \brief Has point on edge
 *
 * \return if \p q is exactly on the i-th edge
 */
	bool hasOnEdge(int i,const Point2& q) const;

/** \brief Get the index of the largest edge
*/
	int getMaxIndex() const;

/** \brief Get the index of the smallest edge
*/
	int getMinIndex() const;

/** \brief Get the maximum squared 2D edge length
*/
	double getMaxSqEdgeLen2D() const;
	/// @private
	void getCommonOffset(double& x,double& y) const;


	CLASS_DECLSPEC
	friend std::ostream &operator<<(std::ostream &stream, const Triangle2& c);
	/// @private
	friend inline void registerTriangles(Triangle2* fromTriangle,int ith,Triangle2*  toTriangle,int jth);

protected:
	double computeArea(double l0,double l1,double l2) const;
	bool getCC_inexact(double avgOffX,double avgOffY,Point2& cc) const;


	Point2* aVertexPointer[3];
	Triangle2* aOppTriangles[3];
	//bool bState;
};

namespace{
inline bool checkRange(int ith)
{
	return (ith==0 || ith==1 || ith==2); // true if ith={0,1,2}
}

}
inline Triangle2* Triangle2::getOppositeTriangle(const int ith) const
{
	assert(checkRange(ith));
	return aOppTriangles[ith];
}

inline void Triangle2::setOppTriangle(const int ith, Triangle2* pNeig)
{
	//if(!checkRange(ith))
	//{
		//std::cout<<"ith="<<ith<<std::endl;
		//int* a(NULL);
		//*a+=1;
	//}

	assert(checkRange(ith));
	aOppTriangles[ith]=pNeig;
}


inline int Triangle2::getIntraTriangleIndex(const Point2* p0,const Point2* p1) const
{
	for(int i=0;i<3;++i)
	{
		int ici1((i+1)%3);
		int ici2((i+2)%3);

		if(	aVertexPointer[ici1]==p0 && aVertexPointer[ici2]==p1) return i;
		if(	aVertexPointer[ici1]==p1 && aVertexPointer[ici2]==p0) return i;
	}

	std::cout<<"BUG: Triangle2::getIntraTriangleIndex failed for"<<std::endl;// COUTOK
	std::cout<<*p0<<std::endl;// COUTOK
	std::cout<<*p1<<std::endl;// COUTOK
	std::cout<<*this<<std::endl;// COUTOK
	assert(false);
	return -1;
}

inline int Triangle2::getIntraTriangleIndex(const Point2* pVtx) const
{
#ifndef NDEBUG
	// Just debug code
	for(int i=0;i<3;++i)
	{
		if(aVertexPointer[i]==pVtx)
		{
			return i;
		}
	}
	assert(false);
#endif
	return ( (aVertexPointer[1]==pVtx) + 2*(aVertexPointer[2]==pVtx));
}


inline int Triangle2::getIntraTriangleIndex(const Triangle2* pTriangle) const
{
#ifndef NDEBUG
	// Just debug code
	for(int i=0;i<3;++i)
	{
		if(aOppTriangles[i]==pTriangle)
		{
			return i;
		}
	}
	assert(false);
#endif
	return ( (aOppTriangles[1]==pTriangle) + 2*(aOppTriangles[2]==pTriangle));
}

//inline int Triangle2::getIntraTriangleIndex(const Triangle2* pTriangle) const
//{

	//if(getOppositeTriangle(0)==pTriangle) return 0;
	//if(getOppositeTriangle(1)==pTriangle) return 1;

//#ifndef NDEBUG
	//if(getOppositeTriangle(2)!=pTriangle)
	//{
		//std::cout<<"Triangle2::getIntraTriangleIndex, pTriangle is not a neighbor of the current triangle"<<std::endl;// COUTOK
		//std::cout<<"Current triangle: "<<*this<<std::endl;// COUTOK
		//std::cout<<"pTriangle: "<<*pTriangle<<std::endl;// COUTOK


		//assert(false);
	//}
//#endif
	//return 2;
//}

inline Point2* Triangle2::getCorner(const int ith) const
{
	assert(checkRange(ith));
	return aVertexPointer[ith];
}

inline void Triangle2::setVertexPointer(const int ith, Point2* pp)
{
	aVertexPointer[ith]=pp;
}

inline void Triangle2::setProperties( Point2* pI, Point2* pJ, Point2* pK)
{
	assert((pI!=NULL && pJ!=NULL && pK!=NULL));
	aVertexPointer[0]=pI;
	aVertexPointer[1]=pJ;
	aVertexPointer[2]=pK;
	pI->setIncidentTriangle(this);
	pJ->setIncidentTriangle(this);
	pK->setIncidentTriangle(this);
	aOppTriangles[0]=NULL;
	aOppTriangles[1]=NULL;
	aOppTriangles[2]=NULL;
}

inline void Triangle2::clearProperties()
{
	for(int i=0;i<3;++i)
	{
		aVertexPointer[i]=NULL;
		aOppTriangles[i]=NULL;
	}
}

inline void Triangle2::setPropertiesAndOppT(
	Point2* pI, Point2* pJ, Point2* pK,
	Triangle2* pNeig0,Triangle2* pNeig1,Triangle2* pNeig2
	)
{
	assert((pI!=NULL && pJ!=NULL && pK!=NULL));
	aVertexPointer[0]=pI;
	aVertexPointer[1]=pJ;
	aVertexPointer[2]=pK;
	pI->setIncidentTriangle(this);
	pJ->setIncidentTriangle(this);
	pK->setIncidentTriangle(this);
	aOppTriangles[0]=pNeig0;
	aOppTriangles[1]=pNeig1;
	aOppTriangles[2]=pNeig2;
}

/// @private
inline void registerTriangles(Triangle2* pFromT,int ith,Triangle2* pToT,int jth)
{
	assert(checkRange(ith));
	assert(checkRange(jth));

	pFromT->aOppTriangles[ith]=pToT;
	pToT->aOppTriangles[jth]=pFromT;

}



} // (namespace)
