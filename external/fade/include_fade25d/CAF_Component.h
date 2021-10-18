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

/// \file CAF_Component.h

#pragma once
#include <map>
#include "common.h"
#include "Segment2.h"



// Only defined for 2.5D
#if GEOM_PSEUDO3D==GEOM_TRUE
namespace GEOM_FADE25D {


/** \enum CAFTYP
*
* enumerates the three possible Cut-And-Fill types
*/
enum CAFTYP
{
	CT_NULL,///< the first surface corresponds to the second one
	CT_CUT,	///< the first surface is above the second one
	CT_FILL ///< the first surface is below the second one
};

class Triangle2;
class Point2;
class Zone2;



/** \brief CAF_Component stands for CUT AND FILL COMPONENT. It
 * represents a connected area of the surface.
 *
 * A CAF_Component object represents a connected part of the surface
 * such that:
 * - the first surface is below the second one (CAFTYP=CT_FILL) or
 * - the first surface is above the second one (CAFTYP=CT_CUT) or
 * - the first surface corresponds to the second one (CAFTYP=CT_NULL)
*/
class CLASS_DECLSPEC CAF_Component
{
public:
	CAF_Component(
		std::vector<Triangle2*>& vT_,
		std::map<Point2*,std::pair<double,double> >& mVtx2BeforeAfter_,
		int label_
		);

/** @brief Get Cut&Fill-Type
 *
 * @return CT_CUT, CT_FILL or CT_NULL
 *
 * - CT_CUT means that earth must be digged off to turn the first
 * surface into the second one,
 * - CT_FILL means that earth must
 * be added.
 * - CT_NULL is returned when the first surface corresponds
 * to the second one.
 */
	CAFTYP getCAFType() const;


/** @brief Get the volume
 *
 * @return the volume of the present component.
 *
 * @note The volume is an absolute value. Use getCAFType() to determine
 * if it is a CUT, FILL or ZERO volume.
 *
 * @warning The computations are unitless but you must make sure
 * that the x, y, and z-coordinate are given in the same unit.
 */
	double getVolume() const;

/** @brief Get label
 *
 * @return the component label
 *
 * Components are consecutively numbered.
 */
	int getLabel() const;

/** @brief Get Triangles
 *
 * \return the triangles of the present component. The z-coordinates
 * of their corners correspond to the height differences between the
 * two input surfaces.
 *
 * \param [out] vTrianglesOut is used to return the triangles
 *
 */
	void getTriangles(std::vector<Triangle2*>& vTrianglesOut) const;


/** @brief Get border
 *
 * @return border segments of the present component in no particular order
 */
	void getBorder(std::vector<Segment2>& vBorderSegments) const;

private:
	CAF_Component(const CAF_Component& e_);
protected:
	void init(std::map<Point2*,std::pair<double,double> >& mVtx2BeforeAfter);
	void showGeomview(const char* name,std::vector<Point2>* pvBeforeT, std::vector<Point2>* pvAfterT, std::vector<Point2>* pvWallT) const;

	std::vector<Triangle2*>* pVT;
	void setVolume(std::vector<Point2>* pvBeforeT, std::vector<Point2>* pvAfterT, std::vector<Point2>* pvWallT);
	CAFTYP caftype;
	double volume;
	int label;

};


/**
 *
 * Report
 */
inline std::ostream &operator<<(std::ostream &stream, const CAF_Component& c)
{
	stream<<"Component "<<c.getLabel();
	switch(c.getCAFType())
	{
		case CT_NULL: stream<<", Type: NULL";break;
		case CT_CUT:  stream<<", Type: CUT ";break;
		case CT_FILL: stream<<", Type: FILL";break;
	}
	stream<<", Volume: "<<c.getVolume();
	return stream;
}


} // (namespace)


#elif GEOM_PSEUDO3D==GEOM_FALSE
#else
	#error GEOM_PSEUDO3D is not defined
#endif
