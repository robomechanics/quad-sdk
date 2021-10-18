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

/// @file Vector2.h

#pragma once
#include "common.h"

#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif

class CLASS_DECLSPEC Vector2;

/** \brief Vector
*
* This class represents a vector in 2D
*/
class Vector2
{
public:

#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Constructor
*
*/
	Vector2(const double x_,const double y_,const double z_);

/** \brief Default constructor
*
* The vector is initialized to (0,0,0)
*/
	Vector2();

/** \brief Copy constructor
*
* Create a copy of vector v_
*/

	Vector2(const Vector2& v_);

#else

/** \brief Constructor
*
*/

	Vector2(const double x_,const double y_);
/** \brief Default constructor
*
* The vector is initialized to (0,0)
*/

	Vector2();
/** \brief Copy constructor
*
* Create a copy of vector v_
*/

	Vector2(const Vector2& v_);
#endif
	/// Assignment operator
	Vector2& operator=(const Vector2& other);


#if GEOM_PSEUDO3D==GEOM_TRUE
	/** \brief Get an orthogonal vector (CCW direction)
	 *
	 */
#endif

	Vector2 orthogonalVector() const;

	/** \brief isDegenerate
	 *
	 * @return true if the vector length is 0, false otherwise.
	*/
	bool isDegenerate() const;


	/** \brief Get the x-value
	*/
	double x() const;

	/** \brief Get the y-value
	*/
	double y() const;

#if GEOM_PSEUDO3D==GEOM_TRUE
	/** \brief Get the z-value.
	*/

	double z() const;
#endif

#if GEOM_PSEUDO3D==GEOM_TRUE
	/** \brief Set the values
	*/
	void set(const double x_,const double y_,const double z_);

#else
	/** \brief Set the values
	*/
	void set(const double x_,const double y_);
#endif

/** \brief Get the squared length of the vector
*/
	double sqLength() const;


#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Get max index
 *
 * @return the index of the largest component (0,1 or 2)
*/
	int getMaxIndex() const;
#endif

/** \brief Get the length of the vector
*/

	double length() const;

/** \brief Scalar product
 *
 * Scalar product
*/
#if GEOM_PSEUDO3D==GEOM_TRUE

	double operator*(const Vector2& other) const;
#else

	double operator*(const Vector2& other) const;
#endif


/** \brief Multiplication
 *
 * Multiply by a scalar value
*/
#if GEOM_PSEUDO3D==GEOM_TRUE

	Vector2 operator*(double val) const;
#else

	Vector2 operator*(double val) const;
#endif

/** \brief Division
 *
 * Divide by a scalar value
*/
#if GEOM_PSEUDO3D==GEOM_TRUE

	Vector2 operator/(double val) const;
#else

	Vector2 operator/(double val) const;
#endif



protected:
	double valX;
	double valY;
#if GEOM_PSEUDO3D==GEOM_TRUE
	double valZ;
#endif
};




// Free functions
/** \brief Print to stream
 *
 * Print to stream
*/
CLASS_DECLSPEC
inline std::ostream &operator<<(std::ostream &stream, const Vector2& vec)
{
#if GEOM_PSEUDO3D==GEOM_TRUE
	stream << "Vector2: "<<vec.x()<<", "<<vec.y()<<", "<<vec.z();
#else
	stream << "Vector2: "<<vec.x()<<", "<<vec.y();
#endif
	return stream;
}

#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Cross product
 *
 * Cross product of two vectors
*/
CLASS_DECLSPEC
inline Vector2 crossProduct(const Vector2& vec0,const Vector2& vec1)
{
	double x=vec0.y() * vec1.z() - vec0.z() * vec1.y();
	double y=vec0.z() * vec1.x() - vec0.x() * vec1.z();
	double z=vec0.x() * vec1.y() - vec0.y() * vec1.x();
	return Vector2(x,y,z);
}
#endif




/** \brief Normalize a vector
*/
CLASS_DECLSPEC
inline Vector2 normalize(const Vector2& other)
{
	double len(other.length());
#if GEOM_PSEUDO3D==GEOM_TRUE
	if(len>0)
	{
		return Vector2(other.x()/len,other.y()/len,other.z()/len);
	}
	else
	{

#ifndef NDEBUG
		std::cout<<"\n\n** warning: normalize(const Vector2& other), Null length vector!"<<std::endl;// COUTOK
		//// Deliberate segfault in debug mode!
		//std::cout<<"vec="<<other<<std::endl;
		//std::cout<<"** debug mode -> deliberate segfault, now:"<<std::endl;
		//int* i(NULL);
		//++*i;
#endif

		return Vector2(0,0,0);
	}
#else
	if(len>0)
	{

		return Vector2(other.x()/len,other.y()/len);
	}
	else
	{
		std::cout<<"warning: normalize(const Vector2& other), Null length vector!"<<std::endl;// COUTOK
		return Vector2(0,0);
	}
#endif
}
/** @brief Opposite vector
 *
 * @return a vector that points in the opposite direction
 */

CLASS_DECLSPEC
inline Vector2 operator-(const Vector2& in)
{
#if GEOM_PSEUDO3D==GEOM_TRUE
	return Vector2(-in.x(),-in.y(),-in.z());
#else
	return Vector2(-in.x(),-in.y());
#endif
}

/** @brief Multiplication with a scalar
 *
 * Multiplication with a scalar
 */

CLASS_DECLSPEC
inline Vector2 operator*(double d,const Vector2& vec)
{
#if GEOM_PSEUDO3D==GEOM_TRUE
	return Vector2(d*vec.x(),d*vec.y(),d*vec.z());
#else
	return Vector2(d*vec.x(),d*vec.y());
#endif
}

/** @brief Addition
 *
 * Addition
 */

CLASS_DECLSPEC
inline Vector2 operator+(const Vector2& vec0,const Vector2& vec1)
{
#if GEOM_PSEUDO3D==GEOM_TRUE
	return Vector2(vec0.x()+vec1.x(), vec0.y()+vec1.y() , vec0.z()+vec1.z());
#else
	return Vector2(vec0.x()+vec1.x(), vec0.y()+vec1.y() );
#endif
}

/** @brief Subtraction
 *
 * Subtraction
 */

CLASS_DECLSPEC
inline Vector2 operator-(const Vector2& vec0,const Vector2& vec1)
{
#if GEOM_PSEUDO3D==GEOM_TRUE
	return Vector2(vec0.x()-vec1.x(), vec0.y()-vec1.y() , vec0.z()-vec1.z());
#else
	return Vector2(vec0.x()-vec1.x(), vec0.y()-vec1.y() );
#endif
}


} // (namespace)
