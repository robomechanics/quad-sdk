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
/// @file CutAndFill.h

#pragma once
#include "common.h"
#include "MsgBase.h"

#include "CAF_Component.h"

#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {

class Zone2; // FWD;
class CutAndFillImpl; // FWD
class Visualizer2; // FWD
class Fade_2D; // FWD
/** \brief CutAndFill computes the volume(s) between two overlapping surfaces
*
* \image html cut_and_fill_surfaces.png "Overlapping input surfaces for Cut-And-Fill: RED=before, GREEN=after. The surfaces do not need to match exactly, the overlapping area is used"
* \image latex cut_and_fill_surfaces.eps "Overlapping input surfaces for Cut-And-Fill: RED=before, GREEN=after. The surfaces do not need to match exactly, the overlapping area is used" width=12cm
* Given two overlapping surfaces with different elevations, CutAndFill
* partitions the surfaces into connected components and computes the
* volume that must be removed or added to turn one surface into the
* other.
*
* \sa http://www.geom.at/cut-and-fill/
*/
class CLASS_DECLSPEC CutAndFill
{
public:
/** @brief Constructor
 *
 * @param pZoneBefore represents the surface before the earthworks
 * @param pZoneAfter is the surface afterwards
 * @param ignoreThreshold (default: 1e-3) can be used to ignore
 * insignificant height differences
 */

	CutAndFill(Zone2* pZoneBefore,Zone2* pZoneAfter,double ignoreThreshold=1e-3);
	CutAndFill();
	~CutAndFill();

/** @brief Get the difference zone
 *
 * This method gives access to the internal data structures, namely
 * to a Zone2 object whose vertices have z-values that correspond
 * to the height differences between the two input meshes (SurfaceBefore
 * minus SurfaceAfter). And a map is returned that contains for each
 * vertex the height in the first and in the second input mesh.
 *
 * @return true in case of success, false otherwise.
 *
 * @note This method may set pDiffZone=NULL and return false when the
 * two input surfaces do not share a common area. In this case the
 * previous call to CutAndFill::go() has already returned false.
 */
	bool getDiffZone(	Zone2*& pDiffZone,
						std::map<Point2*,std::pair<double,double> >& mVtx2BeforeAfter);

/** @brief Register a progress bar object
 *
 * A user defined message receiver object (for example your own
 * progress-bar class) can be registered to get progress updates.
 * This step is optional.
 *
 * @param msgType is the message type. For progress information
 * the type is always MSG_PROGRESS
 * @param pMsg is a user defined progress bar which derives from
 * Fade's MsgBase.
 *
 *
 */
	void subscribe(MsgType msgType,MsgBase* pMsg);

/** @brief Unregister a progress bar object
 *
 * @param msgType is the message type. For progress information
 * the type is always MSG_PROGRESS
 * @param pMsg is a user defined class which derives from Fade's MsgBase
*/
	void unsubscribe(MsgType msgType,MsgBase* pMsg);

/** @brief Get the number of components
 *
 * @return the number of components.
 *
 * A CAF_Component object represents a connected part of the surface
 * such that
 * - the first surface is below the second one (Type CT_FILL)
 * - the first surface is above the second one (Type CT_CUT)
 * - the first surface corresponds to the second one (Type CT_NULL)
 *
 */
	size_t getNumberOfComponents() const;

/** @brief Get component \p ith
 *
 * @return the \p ith CAF_Component.
 */
	CAF_Component* getComponent(size_t ith) const;

/** @brief Start the computation
 *
 * @return true in case of success, false otherwise.
 *
 * @note When an input zone is empty or when the two input zones
 * do not overlap then there is no common area on which the
 * algorithm could operate. In this case the present method
 * returns false.
 */
	bool go();

/** @brief Draw a postscript visualization
 *
 * For a quick overview a postscript visualization can be created.
 *
* \image html cut_and_fill_result.png "Cut&Fill-Result: YELLOW area: CUT, CYAN area: FILL"
* \image latex cut_and_fill_result.eps "Cut&Fill-Result: YELLOW area CUT, CYAN area: FILL" width=11cm
*/

	void show(Visualizer2* pVis) const;

protected:
/// @private
	CutAndFillImpl* pCAFI;
private:
	CutAndFill(const CutAndFill& ):pCAFI(NULL)
	{
		// Never reached
	}
	CutAndFill& operator=(const CutAndFill& )
	{
		pCAFI=NULL;
		return *this;
	}
};

} // Namespace Fade25D only

#elif GEOM_PSEUDO3D==GEOM_FALSE
#else
	#error GEOM_PSEUDO3D is not defined
#endif


