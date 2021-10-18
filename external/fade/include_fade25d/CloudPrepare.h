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

/// \file CloudPrepare.h

#pragma once

#include "common.h"
#if GEOM_PSEUDO3D==GEOM_TRUE

#include "Point2.h"


namespace GEOM_FADE25D {

/// SumStrategy for CloudPrepare
enum SumStrategy
{
	SMS_MINIMUM, ///< Assign the minimum height
	SMS_MAXIMUM, ///< Assign the maximum height
	SMS_MEDIAN, ///< Assign the median height
	SMS_AVERAGE ///< Assign the average height
};

/// ConvexHullStrategy for CloudPrepare
enum ConvexHullStrategy
{
	CHS_NOHULL, ///< No special treatment for convex hull points
	CHS_MAXHULL, ///< Use all points from the convex hull
	CHS_MINHULL ///< Use only convex points of the convex hull (no collinear ones)
};
class CloudPrepareImpl; // FWD





/** \brief CloudPrepare simplifies overdense point clouds and
 * helps to avoid memory-usage-peaks during data transfer.
 *
 * This class has two applications:
 * 1. <b>Simplification of over-dense point clouds</b> from areal and
 * photogrammetry surveys. The reduction is either grid-based or
 * z-adaptive with a tolerance threshold for the height error. Four
 * strategies can be selected for grouping similar points: MIN, MAX,
 * MEDIAN and AVG. MIN corresponds to simple ground filtering, since
 * it favors ground measurement points over those on vegetation.
 * MEDIAN stabilizes the point cloud because it removes outliers
 * while AVG is well suited to reduce noise in the scan.
 *
 * 2. <b>Avoding memory usage peaks</b> when triangulating a large
 * point cloud: Usually you have all vertices in the data structures
 * of your own software and when you then call Fade_2D::insert(),
 * triangles are created immediately and only after that you have
 * the possibility to remove the points from your own data structures.
 * This means for a short time the vertices are duplicated in memory,
 * and this creates an unnecessary memory peak. The solution is to
 * give the points to the CloudPrepare class in advance (one-by-one
 * or all at once) and to delete them from your own data structures
 * while not a single triangle exists yet. Only then call
 * Fade_2D::insert(&CloudPrepare).This avoids duplicating memory
 * usage for the vertices.
 *
 * Have a look at the <a href="https://www.geom.at/terrain-triangulation/">Examples</a>.
 *
 * @note This class replaces the EfficientModel class. It is much
 * more memory-efficient and it is extremely fast.
 *
 * \image html pointCloudReduction.png "Point Cloud Reduction: Left original, right reduced" width=95%
 * \image latex pointCloudReduction.eps "Point Cloud Reduction: Left original, right reduced" width=15cm
 */
class CLASS_DECLSPEC CloudPrepare
{
public:
	CloudPrepare();
	~CloudPrepare();

	// ADD
	/** \brief Add a point to the CloudPrepare object
	 *
	 * @param x,y,z [in] are the coordinates
	 * @param  [in] customIndex is an optional index that you can use
	 * to relate the point to your own data structures.
	 *
	 * @note If you call Point2::getCustomIndex() on this point at
	 * a later time, then exactly this index will be returned.
	 */
	void add(double x,double y,double z,int customIndex=-1);
	/** \brief Add points to the CloudPrepare object (vector-version)
	 *
	 * @param vPoints [in] are the input points
	 */
	void add(std::vector<Point2>& vPoints);
	/** \brief Add points to the CloudPrepare object (array-version)
	 *
	 * @param numPoints  [in] specifies the number of input points
	 * @param aCoordinates  [in] an array that holds 3*numPoints
	 * coordinates (x0,y0,z0,x1,y1,z1,...,xn,yn,zn)
	 */
	void add(size_t numPoints,double * aCoordinates);

	// REDUCE
	/** \brief Simplify the point cloud according to grid resolution
	 *
	 * This method uses a thought grid in the xy plane and combines
	 * the points of each cell into a single point.
	 *
	 * @param gridLength [in] determines the horizontal and vertical
	 * cell spacing in the grid.
	 * @param sms [in] is the SumStrategy used to combine similar
	 * points into one. Possible values are SMS_MINIMUM, SMS_MAXIMUM,
	 * SMS_MEDIAN and SMS_AVERAGE.
	 * @param chs [in] is the ConvexHullStrategy: Points of the convex
	 * hull can be kept unchanged. Use CHS_MAXHULL for this purpose. If
	 * only convex points but not collinear points of the convex hull
	 * are to be considered as convex hull points, then use CHS_MINHULL.
	 * If convex hull points should be treated like all other points,
	 * then use CHS_NOHULL.
	 * @param bDryRun is used to avoid point cloud simplification. This
	 * is used to determine the number of points that would result from
	 * simplification with certain parameters. By default bDryRun=false.
	 *
	 * @return the resulting number of points
	 */
	size_t uniformSimplifyGrid(double gridLength,SumStrategy sms,ConvexHullStrategy chs,bool bDryRun=false);
	/** \brief Simplify the Point Cloud to a specific target size
	 *
	 * This method uses a thought grid in the xy plane and combines
	 * the points of each cell into a single point. The resolution
	 * of the grid is automatically determined such that the point
	 * cloud is reduced to approximately the desired number of points.
	 *
	 * @param approxNumPoints [in] is the desired number of remaining
	 * points. The algorithm will reduce the point cloud to approximately
	 * that number of points.
	 * @param sms [in] is the SumStrategy used to combine similar
	 * points into one. Possible values are SMS_MINIMUM, SMS_MAXIMUM,
	 * SMS_MEDIAN and SMS_AVERAGE.
	 * @param chs [in] is the ConvexHullStrategy: Points of the convex
	 * hull can be kept unchanged. Use CHS_MAXHULL for this purpose. If
	 * only convex points but not collinear points of the convex hull
	 * are to be considered as convex hull points, then use CHS_MINHULL.
	 * If convex hull points should be treated like all other points,
	 * then use CHS_NOHULL.
	 *
	 * @return the resulting number of points
	 */
	size_t uniformSimplifyNum(int approxNumPoints,SumStrategy sms,ConvexHullStrategy chs);
	/** \brief Simplify the Point Cloud according to a tolerance z-value
	 *
	 * This method simplifies the point cloud height-adaptively. This
	 * means that adjoining points with similar z-values (within the
	 * given tolerance @p maxDiffZ) are combined into one.
	 *
	 * @param maxDiffZ [in] is the maximum height (z-coordinate)
	 * difference of points, so that these points are combined to one.
	 * @param sms [in] is the strategy used to summarize similar
	 * points into one. Possible values are SMS_MINIMUM, SMS_MAXIMUM,
	 * SMS_MEDIAN and SMS_AVERAGE.
	 * @param chs [in] is the ConvexHullStrategy: Use CHS_MAXHULL to
	 * keep all points of the convex hull unchanged. If only convex
	 * points but not collinear points of the convex hull are to be
	 * considered as convex hull points, then use CHS_MINHULL. If
	 * convex hull points should be treated like all other points,
	 * then use CHS_NOHULL.
	 * @param bDryRun is used to avoid point cloud simplification. This
	 * is used to determine the number of points that would result from
	 * simplification with certain parameters. By default bDryRun=false.
	 *
	 * @return the resulting number of points
	 */
	size_t adaptiveSimplify(double maxDiffZ,SumStrategy sms,ConvexHullStrategy chs,bool bDryRun=false);

	// BOUNDS
	///@return the x-Range
	double getRangeX();
	///@return the y-Range
	double getRangeY();
	///@return the z-Range
	double getRangeZ();
	///@brief Get the min/max bounds
	///@param minX,minY,minZ,maxX,maxY,maxZ [out]
	void getBounds(double& minX,double& minY,double& minZ,double& maxX,double& maxY,double& maxZ);

	// Convex Hull
	/** @brief Compute the 2.5D Convex Hull
	 *
	 * @param bAllPoints If this parameter is \a true, then all convex
	 * hull points are returned. Otherwise, those points are omitted
	 * which lie on the convex hull but whose absence does not shrink
	 * the convex hull
	 * @param vConvexHull [out] is used to return the convex hull points
	 */
	bool computeConvexHull(bool bAllPoints,std::vector<Point2>& vConvexHull);

	// Points
	/** @brief Get the number of points
	 *
	 * @return the number of points
	 */
	size_t getNumPoints() const;
	/** @brief Get the simplified point cloud
	 *
	 * @param vPointsOut [out] is used to return the points
	 *
	 * @note The points of the CloudPrepare object can be inserted
	 * directly with Fade_2D::insert(CloudPrepare). This is more
	 * memory efficient than getting the points out first only to
	 * pass them to insert().
	 *
	 * \sa Fade_2D::void insert(CloudPrepare* pCloudPrepare,bool bClear=true)
	*/
	void getPoints(std::vector<Point2>& vPointsOut) const;
	/// Clear all stored data
	void clear();
protected:

private:
	/// @private
	CloudPrepareImpl* pImpl;
private:
	CloudPrepare(const CloudPrepare&);
};

} // NAMESPACE

// up to here: if GEOM_PSEUDO3D==GEOM_TRUE
#elif GEOM_PSEUDO3D==GEOM_FALSE
	//namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif
