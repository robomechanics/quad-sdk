// (c) 2010 Geom e.U. Bernhard Kornberger, Graz/Austria. All rights reserved.
//
// This file is part of the Fade2D library. You can use it for your personal
// non-commercial research. Licensees holding a commercial license may use this
// file in accordance with the Commercial License Agreement provided
// with the Software.
//
// This software is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING
// THE WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Please contact the author if any conditions of this licensing are not clear
// to you.
//
// Author: Bernhard Kornberger, bkorn (at) geom.at
// http://www.geom.at
/// \file TriangleAroundVertexIterator.h

#pragma once
#include "common.h"
#include "Point2.h"
#include "Triangle2.h"
//#include "tools.h"

#include "common.h"
#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif

/// @private
inline
int inc1( int num)
{
	++num;
	if(num>2) return 0;
	return num;
}

/// @private
inline
int inc2( int num)
{
	--num;
	if(num<0) return 2;
	return num;
}
/** \brief Iterator for all triangles around a given vertex
*
* The TriangleAroundVertexIterator iterates over all triangles incident
* to a given vertex of a Fade_2D instance. The advantage is that the
* incident triangles can be visited in a certain order, namely
* counterclockwise with operator++() or clockwise using operator--().
* If the order is not important you can use
* Fade_2D::getIncidentTriangles() instead.
*
*
* \image html umbrella.jpg "Left: the iterator visits the triangles around a vertex in a circular manner. Right: Incrementing/decrementing the iterator at a border triangle makes it point to NULL and another increment/decrement sets it to the next border triangle"
* \image latex umbrella.eps "Left: the iterator visits the triangles around a vertex. Right: The iterator 'jumps' over the border edges of the triangulation" width=10cm
*
*/
class CLASS_DECLSPEC  TriangleAroundVertexIterator
{
public:
/** \brief Constructor
*
* @param pPnt_ is the vertex whose incident triangles can be visited with the iterator
*
* Initially the iterator points to an arbitrary triangle (not NULL)
*/
	explicit TriangleAroundVertexIterator(const Point2* pPnt_):pPnt(pPnt_),pTr(pPnt_->getIncidentTriangle()),pSavedTr(NULL)
	{
		if(pTr==NULL)
		{
			//flog("TriangleAroundVertexIterator::TriangleAroundVertexIterator(), created from an invalid point");
			FadeException fadeEx;
			throw fadeEx;
		}
		assert(pTr!=NULL);

	}
/** \brief Constructor
*
* @param pPnt_ is the vertex whose incident triangles can be visited with the iterator
* @param pTr_ is the triangle the iterator initially points to
*/
	TriangleAroundVertexIterator(Point2* pPnt_, Triangle2* pTr_):pPnt(pPnt_),pTr(pTr_),pSavedTr(NULL)
	{
		assert(pTr!=NULL);
	}
/** \brief Copy constructor
*
* Copies the iterator \p it
*/
	TriangleAroundVertexIterator(const TriangleAroundVertexIterator& it) : pPnt(it.pPnt),pTr(it.pTr),pSavedTr(NULL)
	{
		assert(pTr!=NULL);
	}

	TriangleAroundVertexIterator &operator=(const TriangleAroundVertexIterator& other)
	{
		pPnt=other.pPnt;
		pTr=other.pTr;
		pSavedTr=other.pSavedTr;
		return *this;
	}
/** \brief Proceed to the next triangle (the one in counterclockwise order)
*
* Moves the iterator to the next triangle in counterclockwise order.
*
* @warning At the border of a triangulation, two border edges exist which
* are incident to the center vertex. Consequently, the neighbor triangles
* are NULL there. If operator++() leads the iterator off the triangulation
* then the iterator will point to NULL. Another call to operator++() will
* set the iterator to the next triangle in counterclockwise order.
*/
	TriangleAroundVertexIterator& operator++()
	{
//std::cout<<"tavi++"<<std::endl;
		if(pTr==NULL)
		{
//std::cout<<"pTr==NULL, calling loop"<<std::endl;
			loop();
			return *this;
		}

		int ccwIdx=inc1(pTr->getIntraTriangleIndex(pPnt));
//std::cout<<"ccwIdx="<<ccwIdx<<"now swapping saved="<<pSavedTr<<", pTr="<<pTr<<std::endl;;
		std::swap(pSavedTr,pTr);
		pTr=pSavedTr->getOppositeTriangle(ccwIdx);
//std::cout<<"and pTr is now the opposite triangle of pSavedTr="<<*pSavedTr<<", namely "<<pTr<<std::endl;

		return *this;
	}

/** \brief Proceed to the previous triangle (the one in clockwise order)
*
* Moves the iterator to the next triangle in clockwise order.
*
* @warning At the border of a triangulation, two border edges are incident to the
* center vertex. Consequently, the neighbor triangles are NULL there. If
* operator--() leads the iterator off the triangulation then the iterator
* will point to NULL. Another call to operator--() will set the iterator to
* the next triangle in clockwise order.
*/
	TriangleAroundVertexIterator& operator--()
	{
		if(pTr==NULL)
		{
			loop();
			return *this;
		}
		int cwIdx=inc2(pTr->getIntraTriangleIndex(pPnt));
		std::swap(pSavedTr,pTr);
		pTr=pSavedTr->getOppositeTriangle(cwIdx);
		return *this;
	}
/** \brief operator==()
*
* Compares the center and the current triangle of *this and \p rhs
*
* @return true when they are identically or false otherwise
*
*/
	bool operator==(const TriangleAroundVertexIterator& rhs)
	{
		return (pPnt==rhs.pPnt && pTr==rhs.pTr);
	}
/** \brief operator!=()
*
* Compares the center and the current triangle of *this and \p rhs
*
* @return true when they are different, false otherwise
*/
	bool operator!=(const TriangleAroundVertexIterator& rhs)
	{
		return (pPnt!=rhs.pPnt || pTr!=rhs.pTr);
	}
/** \brief Returns a pointer to the current triangle (or NULL)
*
* Dereferencing the iterator yields a pointer to the triangle to which the iterator points.
* @warning This method might yield NULL at the border of a triangulation.
*/
	Triangle2* operator*()
	{
		return pTr;
	}
/** \brief Preview next triangle (CCW direction)
*
* @return the next triangle (the one in CCW direction) without changing
* the current position.
* @warning This method might yield NULL at the border of a triangulation.
*/
	Triangle2* previewNextTriangle()
	{
		TriangleAroundVertexIterator tmp(*this);
		++tmp;
		return *tmp;
	}
/** \brief Preview previous triangle (CW direction)
*
* @return the previous triangle (the one in CW direction) without
* changing the current position.
* @warning This method might yield NULL at the border of a triangulation.
*/
	Triangle2* previewPrevTriangle()
	{
		TriangleAroundVertexIterator tmp(*this);
		--tmp;
		return *tmp;
	}



protected:
	const Point2* pPnt;
	Triangle2 *pTr,*pSavedTr;

	void loop()
	{
		assert(pTr==NULL && pSavedTr!=NULL);

		enum DIRECTION{DIRECTION_NONE,DIRECTION_BACK,DIRECTION_FWD};
		DIRECTION direction(DIRECTION_NONE);

		int axisIndex=pSavedTr->getIntraTriangleIndex(pPnt);

		if(pSavedTr->getOppositeTriangle(inc2(axisIndex))!=NULL)
		{
			direction=DIRECTION_BACK;
		}
		if(pSavedTr->getOppositeTriangle(inc1(axisIndex))!=NULL)
		{
			assert(direction==DIRECTION_NONE);
			direction=DIRECTION_FWD;
		}
		pTr=pSavedTr;
		if(direction==DIRECTION_FWD) while(*operator++()!=NULL); // fast forward
		if(direction==DIRECTION_BACK) while(*operator--()!=NULL); // rewind
		pTr=pSavedTr;
	}
};


} // (namespace)
