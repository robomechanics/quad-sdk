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

/// @file Point2.h

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


class Triangle2; // FWD


/** \brief Point
*
* This class represents a point in 2D with x- and y-coordinates and
* an additional pointer to an associated triangle.
*/
class CLASS_DECLSPEC Point2
{
#ifdef GEOM_DBG
	int getSeqNum()
	{
		static int seqNum(0);
		return seqNum++;
	}
#endif

public:

#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Constructor
*
* @param x_ x-coordinate
* @param y_ y-coordinate
* @param z_ z-coordinate
*/
	Point2(const double x_,const double y_,const double z_):
		coordX(x_),
		coordY(y_),
		coordZ(z_),
		pAssociatedTriangle(NULL),
		customIndex(-1)
	{
#ifdef GEOM_DBG
		customIndex=getSeqNum();
#endif
	}


/** \brief Default constructor
*
*/
	Point2():coordX(-DBL_MAX),coordY(-DBL_MAX),coordZ(-DBL_MAX),pAssociatedTriangle(NULL),customIndex(-1)
	{
#ifdef GEOM_DBG
		customIndex=getSeqNum();
#endif
	}
/** \brief Copy constructor
*
* Create a point as a copy of p_. The associated triangle pointer is initialized to NULL
*/
	Point2(const Point2& p_):
		coordX(p_.x()),
		coordY(p_.y()),
		coordZ(p_.z()),
		pAssociatedTriangle(NULL),
		customIndex(p_.customIndex)
	{
	}

	Point2 &operator=(const Point2& other)
	{
		coordX=other.x();
		coordY=other.y();
		coordZ=other.z();
		pAssociatedTriangle=NULL;
		customIndex=other.customIndex;
		return *this;
	}


// Deprecated, use setHeight() instead. Kept for backward compatibility
/** @private
*/
	void setZ(double z)
	{
		setHeight(z);
	}


/** @private
*/
	void print()
	{
		std::cout<<coordX<<" "<<coordY<<" "<<coordZ<<std::endl;
	}



#else


/** \brief Constructor
*
* @param x_ x-coordinate
* @param y_ y-coordinate
*/
	Point2(const double x_,const double y_):
		coordX(x_),
		coordY(y_),
		pAssociatedTriangle(NULL),
		customIndex(-1)
	{
		//BC("Point2 Constructor default");
	}
/** \brief Default constructor
*
* The coordinates are initialized to -DBL_MAX
*/
	Point2():
		coordX(-DBL_MAX),
		coordY(-DBL_MAX),
		pAssociatedTriangle(NULL),
		customIndex(-1)
	{
	}
/** \brief Copy constructor
*
* Create a point as a copy of p_. The associated triangle pointer is initialized to NULL
*/
	Point2(const Point2& p_):
		coordX(p_.x()),
		coordY(p_.y()),
		pAssociatedTriangle(NULL),
		customIndex(p_.customIndex)
	{
	}

	Point2 &operator=(const Point2& other)
	{
		coordX=other.x();
		coordY=other.y();
		pAssociatedTriangle=NULL;
		customIndex=other.customIndex;
		return *this;
	}

/** @private
*/
	void print()
	{
		std::cout<<coordX<<" "<<coordY<<std::endl;
	}

#endif





	~Point2()
	{
	}

/** \brief Get the x-coordinate
*
* @return the x-coordinate
*/

	double x() const
	{
		return coordX;
	}
/** \brief Get the y-coordinate
*
* @return the y-coordinate
*/

	double y() const
	{
		return coordY;
	}

#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Get the z-coordinate.
*
* @return the z-coordinate
*/

	double z() const
	{
		return coordZ;
	}
#endif

#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Get the x-, y- and z-coordinate
*
* @param x_ x-coordinate
* @param y_ y-coordinate
* @param z_ z-coordinate
*
*/

	void xyz(double& x_,double& y_,double& z_) const
	{
		x_=coordX;
		y_=coordY;
		z_=coordZ;
	}
#endif

/** \brief Get the x- and y-coordinate
*
* @param x_ x-coordinate
* @param y_ y-coordinate
*
*/

	void xy(double& x_,double& y_) const
	{
		x_=coordX;
		y_=coordY;
	}


#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Set the z-coordinate.
*
* Allows to exchange the z-coordinate
*/

	void setHeight(double z)
	{
		coordZ=z;
	}
#endif




/** \brief Get max(abs(x),abs(y))
*
*/

	double getMaxAbs() const
	{
		double a(fabs(coordX));
		double b(fabs(coordY));
		return (std::max)(a,b);
	}






/** \brief Less than operator
*
* Compares the x and y coordinates
*
* @note Although a point has a z-coordinate in the 2.5D version only x and y a compared by this method
*/

	bool operator<(const Point2& p) const
	{
		if(coordX<p.coordX) return true;
		if(coordX>p.coordX) return false;
		if(coordY<p.coordY) return true;
		return false;
	}
/** \brief Greater than operator
*
* Compares the x and y coordinates
*
* @note Although a point has a z-coordinate in the 2.5D version only x and y a compared by this method
*/

	bool operator>(const Point2& p) const
	{
		if(coordX>p.coordX) return true;
		if(coordX<p.coordX) return false;
		if(coordY>p.coordY) return true;
		return false;
	}
/** \brief Equality operator
*
* Compares the x and y coordinates
*
* @note Although a point has a z-coordinate in the 2.5D version only x and y a compared by this method
*/

	bool operator==(const Point2& p) const
	{
		return (coordX==p.coordX && coordY==p.coordY);
	}
#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Equality operator
*
* Compares the x,y,z coordinates while operator==() compares only x,y
*
*/
	bool samePoint(const Point2& p) const
	{
		return(coordX==p.coordX && coordY==p.coordY && coordZ==p.coordZ);
	}
#endif
/** \brief Inequality operator
*
* Compares the x and y coordinates
*
* @note Although a point has a z-coordinate in the 2.5D version only x and y a compared by this method
*/
	bool operator!=(const Point2& p) const
	{
		if(coordX!=p.coordX || coordY!=p.coordY) return true;
		return false;
	}
/** \brief Get the associated triangle
*
* @return the associated triangle
*/
	Triangle2* getIncidentTriangle() const
	{
		return pAssociatedTriangle;
	}


#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Set the coordinates
*
* Internal method
*
* @param x_ x-coordinate
* @param y_ y-coordinate
* @param z_ z-coordinate
*
* @note Does not adapt customIndex and pAssociatedTriangle.
*/
	void setCoords(const double x_,const double y_,const double z_)
	{
		coordX=x_;
		coordY=y_;
		coordZ=z_;
	}
/** \brief Set the coordinates
*
* Internal method
*
* @param x_ x-coordinate
* @param y_ y-coordinate
* @param z_ z-coordinate
* @param customIndex_ Arbitrary index, use -1 if not required
*
*/
	void set(const double x_,const double y_,const double z_,int customIndex_)
	{
		coordX=x_;
		coordY=y_;
		coordZ=z_;
		pAssociatedTriangle=NULL;
		customIndex=customIndex_;
#ifdef GEOM_DBG
		customIndex=getSeqNum();
#endif
	}
#else
/** \brief Set the coordinates and customIndex
*
* Internal method
*
* @param x_ x-coordinate
* @param y_ y-coordinate
* @param customIndex_ Arbitrary index, use -1 if not required
*/
	void set(const double x_,const double y_,int customIndex_)
	{
		coordX=x_;
		coordY=y_;
		pAssociatedTriangle=NULL;
		customIndex=customIndex_;
#ifdef GEOM_DBG
		customIndex=getSeqNum();
#endif
	}
	void change(const double x_,const double y_)
	{
		coordX=x_;
		coordY=y_;
	}
#endif

/** \brief Set the coordiantes
*
* @param pnt is the point whose coordinates are assigned to the current point
*/
#if GEOM_PSEUDO3D==GEOM_TRUE
	void set(const Point2& pnt)
	{
		coordX=pnt.x();
		coordY=pnt.y();
		coordZ=pnt.z();
		pAssociatedTriangle=NULL;
		this->customIndex=pnt.customIndex;
	}
#else
	void set(const Point2& pnt)
	{
		coordX=pnt.x();
		coordY=pnt.y();
		pAssociatedTriangle=NULL;
		this->customIndex=pnt.customIndex;
	}
#endif



/** \brief Set a custom index
*
* An arbitrary index can be assigned to a point. Use getCustomIndex()
* to retrieve it later.
*
* @note This method is provided for the users' convenience. It has nothing
* to do with the internal data structures of Fade 2D and using this method
* is optional. By default this index is -1.
*
* \see int getCustomIndex()
*
* \see A best practices example that deals with indices: http://www.geom.at/runtime/
*
*/

	void setCustomIndex(int customIndex_)
	{
		customIndex=customIndex_;
	}
/** \brief Get the custom index
*
* @return the custom index.
*
* @note The custom index defaults to -1. It is not the index of the point
* in the triangulation (such an index does not exist) but an arbitrary
* value which can be set by the user.
*
* \see void setCustomIndex(int customIndex_)
*
* \see A best practices example that deals with indices: http://www.geom.at/runtime/
*/

	int getCustomIndex() const
	{
		return customIndex;
	}

/** \brief Associate a triangle with the point
*
* @param pT will be associated with the triangle
*/
	void setIncidentTriangle(Triangle2* pT)
	{
		pAssociatedTriangle=pT;
	}

/** \brief Returns a vector from other to *this
 */
Vector2 operator-(const Point2& other) const
{
#if GEOM_PSEUDO3D==GEOM_TRUE
	double xdiff(x()-other.x());
	double ydiff(y()-other.y());
	double zdiff(z()-other.z());
	return Vector2(xdiff,ydiff,zdiff);
#else
	double xdiff(x()-other.x());
	double ydiff(y()-other.y());
	return Vector2(xdiff,ydiff);
#endif


}

/** \brief Add vector and point
 */
Point2 operator+(const Vector2& vec) const
{
#if GEOM_PSEUDO3D==GEOM_TRUE
	return Point2(x()+vec.x(),y()+vec.y(),z()+vec.z());
#else
	return Point2(x()+vec.x(),y()+vec.y());
#endif
}

/** \brief Subtract vector from point
 */
Point2 operator-(const Vector2& vec) const
{
#if GEOM_PSEUDO3D==GEOM_TRUE
	return Point2(x()-vec.x(),y()-vec.y(),z()-vec.z());
#else
	return Point2(x()-vec.x(),y()-vec.y());
#endif
}


	friend std::ostream &operator<<(std::ostream &stream, const Point2& pnt);
	friend std::istream &operator>>(std::istream &stream, Point2& pnt);


protected:
friend class Dt2;
	double coordX;
	double coordY;
#if GEOM_PSEUDO3D==GEOM_TRUE
	double coordZ;
#endif
	Triangle2* pAssociatedTriangle;
	int customIndex;
}; // End of class

/// @brief Print to stream
inline std::ostream &operator<<(std::ostream &stream, const Point2& pnt)
{
#if GEOM_PSEUDO3D==GEOM_TRUE
	stream << "Point2 ("<<&pnt<<","<<pnt.customIndex<<"): "<<pnt.x()<<", "<<pnt.y()<<", "<<pnt.z();
	//#ifndef NDEBUG
		//stream<<", anyT="<<pnt.getIncidentTriangle();
	//#endif
#else
	stream << "Point2 ("<<&pnt<<","<<pnt.customIndex<<"): "<<pnt.x()<<", "<<pnt.y();
#endif


	return stream;
}

/// @brief Stream-to-Point
inline std::istream &operator>>(std::istream &stream, Point2& pnt)
{
#if GEOM_PSEUDO3D==GEOM_TRUE
	stream >> pnt.coordX >> pnt.coordY >> pnt.coordZ;
#else
	stream >> pnt.coordX >> pnt.coordY;
#endif
	return stream;
}

// Free functions

/** \brief Get the squared distance between two points in 2D
*
* @if SECTION_FADE25D
* @note This method does not use the z-coordinate
* @endif
*/
inline
double sqDistance2D(const Point2& p0,const Point2& p1)
{
	double deltaX=p1.x()-p0.x();
	double deltaY=p1.y()-p0.y();
	return (deltaX*deltaX+deltaY*deltaY);
}


/** \brief Get the squared distance between two points in 2D
*
* @if SECTION_FADE25D
* @note This method does not use the z-coordinate
* @endif
*/
inline
double sqDistance2D(const double x0,const double y0,const Point2& p1)
{
	double deltaX=p1.x()-x0;
	double deltaY=p1.y()-y0;
	return (deltaX*deltaX+deltaY*deltaY);
}

#if GEOM_PSEUDO3D==GEOM_TRUE

/** \brief Get the squared distance between two points
*
* @note In contrast to sqDistance2D this method uses also the z-coordinate of the points
*/
inline
double sqDistance25D(const Point2& p0,const Point2& p1)
{
	double deltaX=p1.x()-p0.x();
	double deltaY=p1.y()-p0.y();
	double deltaZ=p1.z()-p0.z();
	double result(deltaX*deltaX+deltaY*deltaY+deltaZ*deltaZ);

	if(result!=result)
	{
		std::cerr<<"warning, sqDistance25D, nan value, no distance"<<std::endl;
		std::cerr<<"p0="<<p0<<std::endl;
		std::cerr<<"p1="<<p1<<std::endl;
	}

	return result;
}


/** \brief Get the squared distance between two points in 2D
*
* @note In contrast to sqDistance2D this method uses also the z-coordinate of the points
*/
inline
double sqDistance25D(const double x0,const double y0,const double z0,const Point2& p1)
{
	double deltaX=p1.x()-x0;
	double deltaY=p1.y()-y0;
	double deltaZ=p1.z()-z0;
	double result(deltaX*deltaX+deltaY*deltaY+deltaZ*deltaZ);

	if(result!=result)
	{
		std::cerr<<"warning, sqDistance25D, nan value, no distance"<<std::endl;
		std::cerr<<"x0="<<x0<<", y0="<<y0<<", z0="<<z0<<std::endl;
		std::cerr<<"p1="<<p1<<std::endl;
	}

	return result;

}


#endif



/** \brief Compute the midpoint of p0 and p1
*
* @note: The exact midpoint of p0 and p1 may not exist in floating point
* numbers. Thus the returned point may not be collinear with p0 and p1.
*/

inline
Point2 center(const Point2& p0,const Point2& p1)
{
#if GEOM_PSEUDO3D==GEOM_TRUE
	Point2 center((p0.x()+p1.x())/2.0,(p0.y()+p1.y())/2.0,(p0.z()+p1.z())/2.0);

#else
	Point2 center((p0.x()+p1.x())/2.0,(p0.y()+p1.y())/2.0);
#endif
	return center;
}

/** \brief Compute the midpoint of p0 and p1 and adapt it
*
* Experimental new function that may change in the future. Thought
* for specific applications.
*
* This function works like center() but additionally it adapts
* the midpoint to the segment (p0,p1) such that it is 'as
* collinear as possible' with p0 and p1 in the x/y plane.
* Bounds for the shift are 0.01 and 1 % of the range in x-
* and y-direction.
*/
CLASS_DECLSPEC
Point2 centerWithShift(const Point2& p0,const Point2& p1);

#if GEOM_PSEUDO3D==GEOM_TRUE
/// \brief Functor to sort points lexicographically
struct CLASS_DECLSPEC Func_ltPointXYZ
{
	bool operator()(const Point2& p0,const Point2& p1) const
	{
		if(p0.x()<p1.x()) return true;
		if(p0.x()>p1.x()) return false;
		if(p0.y()<p1.y()) return true;
		if(p0.y()>p1.y()) return false;
		if(p0.z()<p1.z()) return true;
		return false;
	}
};

#endif

} // (namespace)
