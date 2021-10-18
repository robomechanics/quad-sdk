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


#pragma once
#include <map>
#include "common.h"
#ifndef FADE2D_EXPORT
	#include "License.h"
#endif
#include "Segment2.h"
#include "MsgBase.h"
#include "Bbox2.h"


/// \file SegmentChecker.h

/** The Segment intersection type enumerates the way two line segments intersect each other
 *
 */
enum SegmentIntersectionType
{
	SIT_UNINITIALIZED, /**< Invalid value */
	SIT_NONE, /**< No intersection */
	SIT_SEGMENT, /**< The intersection is a non-degenerate segment (collinear intersection) */
	SIT_POINT, /**< The intersection is a single point differnt from the endpoints */
	SIT_ENDPOINT /**< The two segments share a common endpoint which is the only intersection */
};

enum ClipResult
{
	CR_INVALID, /**< Can't compute a result, call setLimit() with a valid Bbox2 before! */
	CR_EMPTY, /**< The result is empty (input completely outside the box) */
	CR_CLIPPED_DEGENERATE, /**< The result has been clipped and is degenerate */
	CR_CLIPPED_NONDEGENERATE, /**< The result has been clipped and is non-degenerate */
	CR_COMPLETE_DEGENERATE,   /**< The result is unclipped and degenerate (because the segment was already degenerate) */
	CR_COMPLETE_NONDEGENERATE /**< The result is unclipped and non-degenerate */
};

#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif

class SegmentCheckerData; // FWD, pImpl to ease DLL export


/** \brief SegmentChecker identifies intersecting line segments
 *
* SegmentChecker takes a bunch of line segments and fully automatically
* identifies illegal segment intersections. The intersection points can
* be computed in 2D and in 2.5D. Further this class offers visualization
* methods. Due to the underlying datastructure the search algorithm
* scales very well to large inputs.
*
* \image html identify_intersecting_segments.png "Polylines: Intersecting segments are automatically found"
* \image latex identify_intersecting_segments.eps "Polylines: Intersecting segments are automatically found" width=15cm
*
* \sa http://www.geom.at/segment-checker/
*
*/
class CLASS_DECLSPEC SegmentChecker
{
public:
	// *** CTOR and DTOR ***

/**
 *
 * Internally this constructor prepares a data structure from
 * \p vSegments that enables efficient spatial searches. The time
 * complexity is O(n*log(n)).
 *
 * @param vSegments_ contains the segments to be checked
 */
	explicit SegmentChecker(const std::vector<Segment2*>& vSegments_);
	SegmentChecker();
	~SegmentChecker();

/** Clip Segment
 *
 * Use this method to limit the length of a line-segment to its
 * intersection with a box. The result can be the whole segment,
 * a subsegment, a degenerate segment or the result can be empty.
 * In the last case the segment is not changed but the method
 * returns CR_EMPTY.
 *
 * @param [in,out] seg is the line segment to be clipped
 *
 * @return CR_INVALID,CR_EMPTY, CR_CLIPPED_DEGENERATE, CR_CLIPPED_NONDEGENERATE,
 * CR_COMPLETE_DEGENERATE, CR_COMPLETE_NONDEGENERATE or CR_INVALID.
 *
 * @note In case that you missed to call setLimit() with a valid
 * bounding box before, the method returns CR_INVALID
};

*/
	ClipResult clipSegment(Segment2& seg);


/** Set Limit
 *
 * Sets the bounding box \p bbx that is used by clipSegment() to limit
 * the length of a line segment
 */
	void setLimit(const Bbox2& bbx);

/** Get Limit
 *
 * @return the bounding box that has been set before using setLimit()
 */
	Bbox2 getLimit() const;

/**
 *
 * Returns the i-th segment
 *
 * @param i is the index of the segment to be returned
 */
	Segment2* getSegment(size_t i) const;
/** Returns the number of segments contained in this SegmentChecker object
 */
	size_t getNumberOfSegments() const;
/** Returns the index of a segment
 *
 * @param pSeg is the segment whose index is returned
 */
	int getIndex(Segment2* pSeg) const;
/** Register a progress bar object
 *
 * The SegmentChecker does its job typically in fractions of a second.
 * But inputs may contain a quadratic number of intersections and such
 * tasks take a while. Therefore a user defined message object (your
 * own progress-bar class) can be registered in order to get progress
 * updates. This step is optional.
 *
 * @param msgType is the message type. For progress information
 * the type is always MSG_PROGRESS
 * @param pMsg is a user defined progress bar which derives from
 * Fade's MsgBase.
 *
 *
 */
	void subscribe(MsgType msgType,MsgBase* pMsg);
/** Unregister a progress bar object
 *
 * @param msgType is the message type. For progress information
 * the type is always MSG_PROGRESS
 * @param pMsg is a user defined class which derives from Fade's MsgBase
*/
	void unsubscribe(MsgType msgType,MsgBase* pMsg);

	// *** Visualization and debug ***
/** Write all segments, with and without intersection, to a postscript file
 *
 * @param name is the output filename
 *
* \image html show_line_segments.png "Line segments written to a postscript file"
* \image latex show_line_segments.eps "Line segments written to a postscript file"  width=15cm
*/
	void showSegments(const char* name) const;


/** Write a postscript file, highlight illegal segments
 *
 * @param bAlsoEndPointIntersections specifies if intersections at endpoints are also illegal
 * @param name is the output filename
 *
 * \image html show_illegal_segments_count.png "Visualization of polyline intersections"
 * \image latex show_illegal_segments_count.eps "Visualization of polyline intersections"  width=15cm
*/
	void showIllegalSegments(bool bAlsoEndPointIntersections,const char* name) const;

	// *** Intersectors ***
/** Get illegal segments
 *
 * Returns segments which are involved in intersections. Intersections
 * at endpoints are only reported when \p bAlsoEndPointIntersections is
 * true. The asymptotic time consumption for the lookup per segment S
 * is O(log(n)+k) where k is the number of segments that intersect the
 * minimal bounding box of S. Thus, for n segments the method takes
 * O(n*(log(n)+k)) time.
 *
 * \image html segment_intersection_types.png "Segment intersections: (1) Non-collinear, (2) collinear, (3) duplicate segments, (4) endpoint intersection"
 *
 * @param bAlsoEndPointIntersections specifies if intersections at endpoints shall be detected
 * @param [out] vIllegalSegmentsOut is the output vector
 *
 *
*/
	void getIllegalSegments(bool bAlsoEndPointIntersections,std::vector<Segment2*>& vIllegalSegmentsOut) const;

/** Get the intersection type of two segments
 *
 * @param pSeg1,pSeg2 are the segments to be checked
 *
 * @return SIT_NONE (no intersection),\n SIT_SEGMENT (collinear intersection),\n SIT_POINT (intersection somewhere between the endpoints) or\n SIT_ENDPOINT (endpoint intersection)
 *
 *
 * @note \p pSeg1 and \p pSeg2 do not need to be from the set that has
 * been used to initialize the present object
*/
	SegmentIntersectionType getIntersectionType(const Segment2* pSeg1,const Segment2* pSeg2) const;


/** Return segments that intersect a certain segment along with their intersection type
 *
 * @param pTestSegment is the segment to be analyzed
 * @param bAlsoEndPointIntersections specifies if intersections of
 * type SIT_ENDPOINT shall also be reported.
 * @param [out] vIntersectorsOut is the output vector. Segments
 * intersecting \p pTestSegment are added to \p vIntersectorsOut
 * along with their intersection type.
 *
 * @note When vIntersectorsOut is non-empty, it is not cleared but
 * the intersected segments are added.
 *
 * The time complexity is O(log(n)+k) where n is the number of segments
 * and k is the number of intersections for \p pTestSegment.
*/
	void getIntersectors(	Segment2* pTestSegment,
							bool bAlsoEndPointIntersections,
							std::vector<std::pair<Segment2*,SegmentIntersectionType> >& vIntersectorsOut) const;

	// Intersections
#if GEOM_PSEUDO3D==GEOM_TRUE
/** Compute the intersection point(s) of two segments.
 *
 * Use \link getIntersectionType() \endlink to determine the segment
 * intersection type \p sit before. Call this function only when
 * the intersection type is SIT_POINT or SIT_ENDPOINT.
 *
 * @param sit is the segment intersection type (SIT_POINT or SIT_ENDPOINT for the present method)
 * @param seg0,seg1 are the intersecting segments
 * @param [out] ispOut0 output intersection point at \p seg0
 * @param [out] ispOut1 output intersection point at \p seg1
 *
 * The resulting two output intersection points \p ispOut0 and \p ispOut1
 * have always the same (x,y) coordinates but possibly different heights \p z.
 *
 * @note \p pSeg1 and \p pSeg2 do not need to be from the set of
 * segments that have been used as argument for the constructor of
 * the SegmentChecker. You can use any segments.
*/
	void getIntersectionPoint(	SegmentIntersectionType sit,
								const Segment2& seg0,
								const Segment2& seg1,
								Point2& ispOut0,
								Point2& ispOut1
								) const;
/** Compute the intersection segment(s) of two collinear intersecting
 * segments.
 *
 * Use \link getIntersectionType() \endlink to determine the segment
 * intersection type \p sit before. Call this function only when
 * the intersection type is SIT_SEGMENT.
 *
 * @param seg0,seg1 are intersecting segments such that their SegmentIntersectionType is SIT_SEGMENT
 * @param [out] issOut0 intersection segment at \p seg0
 * @param [out] issOut1 intersection segment at \p seg1
 *
 * The two output segments have always the same (x,y) coordinates but
 * possibly diffent heights \p z.
 *
 * @note \p pSeg1 and \p pSeg2 do not need to be from the set of
 * segments that have been used as argument for the constructor of
 * the SegmentChecker. You can use any segments.
 */

	void getIntersectionSegment(const Segment2& seg0,
								const Segment2& seg1,
								Segment2& issOut0,
								Segment2& issOut1
								) const;
#else
/** Compute the intersection point of two segments
 *
 * Use \link getIntersectionType() \endlink to determine the segment
 * intersection type \p sit.

 * @param typ is the intersection type (SIT_POINT or SIT_ENDPOINT for the present method)
 * @param seg0,seg1 are the intersecting segments
 * @param [out] ispOut is the output intersection point.
 *
 * @note \p pSeg1 and \p pSeg2 do not need to be from the set that has
 * been used to initialize the SegmentChecker.
 *
*/
	void getIntersectionPoint(	SegmentIntersectionType typ,
								const Segment2& seg0,
								const Segment2& seg1,
								Point2& ispOut
								) const;
/** Computes the intersection segment of two collinear intersecting segments
 *
 * @param seg0,seg1 are intersecting segments such that their SegmentIntersectionType is SIT_SEGMENT
 * @param [out] issOut is the computed intersection of \p seg0 and \p seg1
 *
 * @note \p pSeg1 and \p pSeg2 do not need to be from the set that has
 * been used to initialize the present object

 */

	void getIntersectionSegment(const Segment2& seg0,
								const Segment2& seg1,
								Segment2& issOut
								) const;
#endif

/** Return the intersection type as a human readable string. This is a convenience function
 *
 * @param sit is an intersection type to be converted to a string
 *
 */

	const char* getIntersectionTypeString(SegmentIntersectionType sit) const;



private:
	SegmentChecker(const SegmentChecker& );
/**
\cond HIDDEN_SYMBOLS
*/
	SegmentIntersectionType getIntersectionType_degeneratePart(
																const Segment2*& pSeg1,
																const Segment2*& pSeg2,
																Point2& p1,
																Point2& p2,
																Point2& p3,
																Point2& p4
																) const;
	void addSegments(const std::vector<Segment2*>& vSegments_);

	// Data
	SegmentCheckerData* pSCD;


/**
\endcond
*/

};

} // Namespace

