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


/** @file Fade_2D.h
 *
 * Fade_2D.h is the main API of the Fade library
 */

#pragma once

#include "common.h"
#include "freeFunctions.h"
#ifndef FADE2D_EXPORT
	#include "License.h"
#endif
#include "Point2.h"
#include "Triangle2.h"
#include "TriangleAroundVertexIterator.h"
#include "Visualizer2.h"
#include "Zone2.h"
#include "ConstraintGraph2.h"
#include "Performance.h"
#include "MeshGenParams.h"
#include "MsgBase.h"
#include "SegmentChecker.h"
#include "testDataGenerators.h"
#include "FadeExport.h"
#include "Voronoi2.h"

#if GEOM_PSEUDO3D==GEOM_TRUE
	#include "IsoContours.h"
	#include "EfficientModel.h"
	#include "CutAndFill.h"
	#include "CloudPrepare.h"
#endif



#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif



class Dt2; // Forward
class Visualizer3; // Forward


/** \brief Fade_2D is the Delaunay triangulation main class
*
* Fade_2D represents a Delaunay triangulation in 2D or 2.5D (depends on the used namespace)
*/
class CLASS_DECLSPEC Fade_2D
{
public:
/** \brief Constructor of the main triangulation class
*
* @param numExpectedVertices specifies the number of points that will
* be inserted. This is a default parameter that does not need to be
* specified.
*/
	explicit Fade_2D(unsigned numExpectedVertices=3)
	{
		initFade(numExpectedVertices);
	}
/// Destructor
	~Fade_2D();




/** \brief Save a triangulation
 *
 * The saveTriangulation() command saves all triangles of the present
 * triangulation to a binary file. Thereby it retains constraint edges
 * and custom vertex indices, if any. If Zone2* pointers are specified,
 * these zones will be saved also and their order will be retained.
 *
 * @param [in] filename is the name of the output file
 * @param [out] vSaveZones is used specify zones that shall additionally
 * be saved
 *
 * \sa If you just want to store zones, use Zone2::save() or
 * Fade_2D::saveTriangulation(). Use Fade_2D::load() to reload
 * data from such files.
 *
 * @return whether the operation was successful
 */
	bool saveTriangulation(const char* filename,std::vector<Zone2*>& vSaveZones);

/** \brief Save a triangulation
 *
 * The saveTriangulation() command saves all triangles of the present
 * triangulation to a stream. Thereby it retains constraint edges
 * and custom vertex indices, if any. If Zone2* pointers are specified,
 * these zones will be saved also and their order will be retained.
 *
 * @param stream is the output stream
 * @param [out] vSaveZones is used specify zones that shall additionally
 * be saved
 *
 * \sa If you just want to store zones, use Zone2::save() or
 * Fade_2D::saveTriangulation(). Use Fade_2D::load() to reload
 * data from such files.
 *
 * @return whether the operation was successful
 */
	bool saveTriangulation(std::ostream& stream,std::vector<Zone2*>& vSaveZones);


/** \brief Save zones
 *
 * The saveZones() command saves the triangles of the zones in
 * \p vSaveZones to a binary file. Thereby it keeps the order of
 * the zones and it retains any constraint edges and custom indices
 * in the domain.
 *
 * @note A Delaunay triangulation is convex without holes and this
 * may not hold for the zones to be saved. Thus extra triangles may
 * be saved to fill concavities. These extra-triangles will belong
 * to the Fade_2D instance but not to any Zone2 when reloaded later.
 *
 * @param [in] filename is the name of the output file
 * @param [out] vSaveZones (non-empty) specifies the zones to be saved
 *
 * @return whether the operation was successful
 *
 * \sa The saveTriangulation() command can be used to store all
 * triangles of a triangulation plus any specified zones. The
 * Zone2::save() command is used to store just one zone. Use
 * Fade_2D::load() to reload data from such files.
 */
	bool saveZones(const char* filename,std::vector<Zone2*>& vSaveZones);


/** \brief Save zones
 *
 * The saveZones() command saves the triangles of the zones in
 * \p vSaveZones to stream. Thereby it keeps the order of
 * the zones and it retains any constraint edges and custom indices
 * in the domain.
 *
 * @note A Delaunay triangulation is convex without holes and this
 * may not hold for the zones to be saved. Thus extra triangles may
 * be saved to fill concavities. These extra-triangles will belong
 * to the Fade_2D instance but not to any Zone2 when reloaded later.
 *
 * @param stream is the name of output stream
 * @param [out] vSaveZones (non-empty) specifies the zones to be saved
 *
 * @return whether the operation was successful
 *
 * \sa The saveTriangulation() command can be used to store all
 * triangles of a triangulation plus any specified zones. The
 * Zone2::save() command is used to store just one zone. Use
 * Fade_2D::load() to reload data from such files.
 */
	bool saveZones(std::ostream& stream,std::vector<Zone2*>& vSaveZones);


/** \brief Load a triangulation
 *
 * Loads a triangulation together with any custom indices,
 * constraint-edges and zones from a binary file
 *
 * @param [in] filename is the name of the input file
 * @param [out] vZones is used to return Zone2* pointers if any. The order
 * of the pointers is the same as at the time of storage
 *
 * @return whether the operation was successful
 */
	bool load(const char* filename,std::vector<Zone2*>& vZones);


/** \brief Load a triangulation
 *
 * Loads a triangulation together with any custom indices,
 * constraint-edges and zones from a stream
 *
 * @param stream is an input stream
 * @param [out] vZones is used to return Zone2* pointers if any. The order
 * of the pointers is the same as at the time of storage
 *
 * @return whether the operation was successful
 */
	bool load(std::istream& stream,std::vector<Zone2*>& vZones);


/** \brief Export triangulation data from Fade
 *
 * @param fadeExport is a struct that will hold the requested triangulation data
 * @param bWithCustomIndices determines whether the custom indices of the points are also stored
 * @param bClear determines whether the Fade instance is cleared <b>during</b> the export operation to save memory
 *
 * @note When bClear is true then all memory of the Fade object is
 * deleted i.e., all existing pointers to its objects become invalid.
 */
	void exportTriangulation(FadeExport& fadeExport,bool bWithCustomIndices,bool bClear);


/** \brief Checks if a triangulation is valid
*
* Checks the validity of the data structure.
*
* \param bCheckEmptyCircleProperty specifies if (slow!) multiprecision
* arithmetic shall be used to recheck the empty circle property
* \param msg is a debug string that will be shown in terminal output
* so that you know which checkValidity call currently runs.
*
* This method is thought for development purposes. Don't call it
* method unless you assume that something is wrong with the code.
*/
	bool checkValidity(bool bCheckEmptyCircleProperty,const char* msg) const;

/** \brief Set the number CPU cores for multithreading
 *
 * \param numCPU is the number of CPU cores to be used. The special
 * value \p numCPU=0 means: auto-detect and use the number of available
 * CPU cores.
 * \return the number of CPU cores that will be used (useful in case
 * of auto-detection)
 *
 *
 * Characteristics:
 * - This setting affects Fade_2D::measureTriangulationTime() and
 * Fade_2D::insert() which is by default single-threaded to avoid
 * undeliberate nested multithreading (an application may run Fade
 * in a thread).
 * - For technical reasons points should be inserted before any
 * constraint segments so that the algorithm can fully benefit from
 * multithreading.
 * - Fade continues support for very old compilers but multithreading
 * is not available for VS2010 and CentOS6.4 library versions.
 *
 */
	int setNumCPU(int numCPU);

/** \brief Get the Voronoi diagram
 *
 * @return a dual Voronoi diagram that changes dynamically when the
 * triangulation changes.
 */
	Voronoi2* getVoronoiDiagram();

/** \brief Set fast mode
 *
 * By default, numerically perfect calculations are performed to
 * compute a 100% perfect Delaunay triangulation. However, the
 * difference to using double precision arithmetic is hardly
 * noticeable. It is rather relevant in scientific applications
 * while practical applications may want to skip the computationally
 * expensive calculations. Depending on the position of the input
 * points, the effect of FastMode can be zero or a quite considerable
 * acceleration.
 *
 * @param bFast use true to avoid using multiple precision arithmetic
 * in favor of better performance.
 */
	void setFastMode(bool bFast);


/** \brief Statistics
 *
 * Prints mesh statistics to stdout.
 */
 	void statistics(const char*  s) const;

/** \brief Draws the triangulation as postscript file.
*
* show() is a convenience function for quick outputs with a default
* look. It is also possible to use the Visualizer2 class directly to
* draw arbitrary circles, line segments, vertices and labels with
* custom colors.
*
* @param postscriptFilename is the output name, i.e. "myFile.ps"
* @param bWithConstraints specifies if constraint segments shall be shown (default: true)
*/
 	void show(const char* postscriptFilename,bool bWithConstraints=true) const;


/** \brief Draws the triangulation as postscript file using an existing Visualizer2 object
*
* This overload of the show() method allows to add further geometric
* primitives to the Visualizer2 object before it is finally written.
*
* @param pVis is the pointer of a Visualizer2 object that may already contain geometric
* primitives or that may later be used to draw further elements
* @param bWithConstraints specifies if constraint segments shall be shown (default: true)
*
* @note The postscript file must be finalized with Visualizer2::writeFile().
*
*/
 	void show(Visualizer2* pVis,bool bWithConstraints=true) const;

#if GEOM_PSEUDO3D==GEOM_TRUE
	/** \brief Draws the triangulation in 3D.
	*
	* @param filename is the output filename
	* @param color is by default white (red:1,green:1,blue:1,alpha:0.5)
	*
	* @note The free viewer Geomview can be used to view such files
	*/
	void showGeomview(const  char* filename, const char* color="1 1 1 0.5") const;

	/** \brief Draws the triangulation in 3D.
	*
	* @param pVis points to a Visualizer3 object
	* @param color is by default white (red:1,green:1,blue:1,alpha:0.5)
	*
	* @note The free viewer Geomview can be used to view such files
	*/
	void showGeomview(Visualizer3* pVis, const char* color="1 1 1 0.5") const;


#endif

/** \brief Remove a single vertex
*
* @param pVertex shall be removed.
*
* \note \p pVertex must not be a vertex of a ConstraintGraph2 or
* ConstraintSegment2 object. If this is the case, the vertex is
* not removed and a warning is issued.
*/

	void remove(Point2* pVertex);


/** \brief Compute the convex hull
*
*
* @param bAllVertices determines if all convex hull points are returned
* or if collinear ones shall be removed.
* @param [out] vConvexHullPointsOut is used to return the convex hull
* vertices in counterclockwise order. The start vertex is the leftmost
* vertex. If more than one leftmost vertex exists, the bottommost of
* them is the start vertex.
*
*/

	void getConvexHull(bool bAllVertices,std::vector<Point2*>& vConvexHullPointsOut);

#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Insert points from a CloudPrepare object
 *
 * @param [in] pCloudPrepare is a CloudPrepare object that contains
 * a point cloud
 * @param [in] bClear determines whether pCloudPrepare shall be
 * cleared during the operation in order to save memory. Always use
 * bClear=true unless you plan to use pCloudPrepare also for other
 * triangulations.
 *
 * @note There's a difference in peak memory conversion compared to
 * the other insert() methods: You can pre-give the vertices to the
 * CloudPrepare object and delete them from your own software's data
 * structures while not a single triangle has been created yet. Only
 * after that you call this insert() function with the CloudPrepare
 * object and the vertices are triangulated.
 *
 */
	void insert(CloudPrepare* pCloudPrepare,bool bClear=true);
#endif

/** \brief Insert a single point
*
* @param p is the point to be inserted.
* @return a pointer to the point in the triangulation
*
* The triangulation keeps a copy of \e p. The return value is a pointer to this copy.
* If duplicate points are inserted, the triangulation does not create new copies but
* returns a pointer to the copy of the very first insertion. @n
* @n
* @note This method offers a very good performance but it is still faster if all
* points are passed at once, if possible.
*/

	Point2* insert(const Point2& p);

/** \brief Insert a vector of points.
*
* @param vInputPoints contains the points to be inserted.
*
* \note Use Fade_2D::setNumCPU() to activate multithreading
*
*/

	void insert(const std::vector<Point2>& vInputPoints);

/** \brief Insert points from a std::vector and store pointers in \e vHandles
*
* @param vInputPoints contains the points to be inserted.
* @param vHandles (empty) is used by Fade to return Point2 pointers
*
* Internally, the triangulation keeps copies of the inserted points which
* are returned in \e vHandles (in the same order). If duplicate points are
* contained in vInputPoints then only one copy will be made and a pointer
* to this unique copy will be stored in vHandles for every occurance.
*
* \note Use Fade_2D::setNumCPU() to activate multithreading
*/


	void insert(const std::vector<Point2>& vInputPoints,std::vector<Point2*>& vHandles);

#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Insert points from an array
*
* @param numPoints is the number of points to be inserted
* @param aCoordinates is an array of \e 3n double values, e.g. {x0,y0,z0,x1,y1,z1,...,xn,yn,zn}
* @param aHandles is an array with size \e n where pointers to the inserted points will be stored
*
* \note Use Fade_2D::setNumCPU() to activate multithreading
*/
#else
/** \brief Insert points from an array
*
* @param numPoints is the number of points to be inserted
* @param aCoordinates is an array of \e 2n double values, e.g. {x0,y0,x1,y1,...,xn,yn}
* @param aHandles is an empty array with size \e n where pointers to the inserted points will be stored by Fade
*
* \note Use Fade_2D::setNumCPU() to activate multithreading
*/
#endif
	void insert(int numPoints,double * aCoordinates,Point2 ** aHandles);


/** \brief Measure the Delaunay triangulation time
 *
 * This method evaluates the performance of single- and multithreaded
 * point insertion into a Delaunay triangulation.
 *
 * \param [in] vPoints are the points to be inserted
 * \return the total wall-time for point insertion in seconds
 *
 * \note The method cleans up the triangulation (objects, memory) on
 * exit. Thus the time measured outside this method may be slightly
 * larger than the returned time that is exactly the time needed to
 * triangulate the input points.
 *
 * \note Use Fade_2D::setNumCPU() to activate multithreading
 *
 */
	double measureTriangulationTime(std::vector<Point2>& vPoints);



/** \brief Locate a triangle which contains \e p
*
* \image html locate.jpg "Point location"
* \image latex locate.eps "Point location" width=7cm
*
* The Fade_2D class can be used as a data structure for point location.
* This method returns a pointer to a triangle which contains \e p.
*
* @param [in] p is the query point
* @return a pointer to a Triangle2 object (or NULL if hasArea()==false or if \e p is outside the triangulation)
*
*/


	Triangle2* locate(const Point2& p);


/** \brief Get nearest neighbor
*
* This method returns the closest vertex of \p p.
*
* @param [in] p is the query point
* @return a pointer to the closest vertex
*
*/
Point2* getNearestNeighbor(const Point2& p);

#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Compute the height of a certain point
*
* Computes the height (z) at the coordinates \p x and \p y, assigns it
* to heightOut and returns true if successful.
*
* \param [in] x,y are the input coordinates
* \param [out] heightOut is the computed height
* \param [in] pApproxT can be set to a nearby triangle. If unknown, use NULL.
* \param [in] tolerance is by default 0, see below
*
* \note pApproxT is an optional parameter to speed up the search in
* case that you know a nearby triangle. But point location if very
* fast anyway and if you are not sure, using NULL is probably faster.
*
* \note Due to rounding errors your query point may lie slightly outside
* the convex hull of the triangulation and in such a case the present
* method would correctly return false. But you can use the optional
* \p tolerance parameter (default: 0): If your query point is not
* farther outside the convex hull than \p tolerance then the height
* of the closest point of the convex hull is returned.
*
*
*
*/

	bool getHeight(double x,double y,double& heightOut,Triangle2* pApproxT=NULL,double tolerance=0) const;
#endif

/** \brief Delaunay refinement
 *
 * Creates a mesh inside the area given by a Zone2 object.
 *
 * @param pZone is the zone whose triangles are refined. Allowed zoneLocation values are ZL_INSIDE and ZL_BOUNDED.
 * @param minAngleDegree (up to 30) is the minimum interior triangle angle
 * @param minEdgeLength is a lower threshold on the edge length. Triangles with smaller edges are not refined.
 * @param maxEdgeLength is an upper threshold on the edge length. Triangles with larger edges are always refined.
 * @param bAllowConstraintSplitting specifies if constraint edges may be splitted
 *
 * @note The behavior of the present method had to be changed in Fade v1.39:
 * Only ZL_INSIDE and ZL_BOUNDED zones are accepted. But you can easily
 * convert other types of zones to ZL_BOUNDED using @ref convertToBoundedZone "Zone2::convertToBoundedZone()".
 */

	void refine(	Zone2* pZone,
					double minAngleDegree,
					double minEdgeLength,
					double maxEdgeLength,
					bool bAllowConstraintSplitting
					);

/** \brief Delaunay refinement and grid meshing
 *
 * This method calls an advanced Delaunay mesh generator and grid mesher.
 * The parameters are encapsulated in the MeshGenParams class. This class
 * provides default parameters that can be used as is. Alternatively
 * client code can derive from MeshGenParams and overwrite the methods
 * and parameters to gain full control over the mesh generation process.
 *
 *
 *
\htmlonly
<div class="sbs4image">
<a href="http://www.geom.at/wp-content/uploads/2021/01/delaunay_triangulation.jpg">
 <img style="width: 95%" src="http://localhost/geom_homepage_www/fade25d/html/mesh-generator-customParameters.png" alt="Control over the local mesh density in a Delaunay Mesh" /></a>
  <p>Control over the local mesh density</p>
</div>
<div class="sbs4image">
<a href="http://www.geom.at/wp-content/uploads/2021/01/constrained_delaunay.jpg">
 <img style="width: 95%" src="http://www.geom.at/wp-content/uploads/2021/01/mesh-generator-maxLength.png" alt="Delaunay meshing with maximum edge length" /></a>
  <p>Delaunay meshing with maximum edge length</p>
</div>
<div class="sbs4image">
<a href="http://www.geom.at/wp-content/uploads/2021/01/delaunay_zones.png">
 <img style="width: 95%" src="http://localhost/geom_homepage_www/fade25d/html/grid-mesh.png" alt="Axis-aligned grid mesh" /></a>
  <p>Grid mesh, axis-aligned</p>
</div>
<div class="sbs4image">
<a href="http://www.geom.at/wp-content/uploads/2021/01/delaunay-mesher-fade_l.png">
 <img style="width: 95%" src="http://localhost/geom_homepage_www/fade25d/html/grid-mesh-angle.png" alt="Grid mesh aligned to a certain direction" /></a>
  <p>Grid mesh, aligned to a certain direction</p>
</div>
<div class="floatStop"></div>
\endhtmlonly
 *
*/
	void refineAdvanced(MeshGenParams* pParameters);






/** \brief Number of points
 *
 * @return the number of points in the triangulation
 *
 * @note Due to possibly duplicate input points the number of points is
 * not stored somewhere but freshly computed in O(n) time. This is fast
 * but you are adviced to avoid calling this method over-frequently in
 * a loop. Duplicate point insertions count only once.
 */
	size_t numberOfPoints() const;

/** \brief Number of triangles
 *
 * @return the number of triangles in the triangulation (or 0 as long as hasArea() is false).
 *
 */

	size_t numberOfTriangles() const;


/** \brief Get pointers to all triangles
*
* This command fetches the existing triangles
*
* @param [out] vAllTriangles is used to return the triangles
*
*
* @note that the lifetime of data from the Fade2D datastructures
* does exceed the lifetime of the Fade2D object.
*/

	void getTrianglePointers(std::vector<Triangle2*>& vAllTriangles) const;

/** \brief Get pointers to all vertices
*
* @param vAllPoints is an empty vector of Point2 pointers.
*
* Stores pointers to all vertices of the triangulation in vAllPoints. The order in which
* the points are stored is \e not necessarily the insertion order. For geometrically identical
* points which have been inserted multiple times, only one pointer exists. Thus vAllPoints.size()
* can be smaller than the number of inserted points.
*
* @note that the lifetime of data from the Fade2D datastructures
* does exceed the lifetime of the Fade2D object.
*/

	void getVertexPointers(std::vector<Point2*>& vAllPoints) const;



/** \brief Get adjacent triangle
*
* @return the triangle that has the edge (p0,p1) or NULL if no such edge is present
*
* @note Recall the counter-clockwise enumeration of vertices in a
* triangle. If (p0,p1) is used, the unique triangle with the CCW
* oriented edge (p0,p1) is returned, using (p1,p0) one gets the
* other adjacent triangle.
*/
Triangle2* getAdjacentTriangle(Point2* p0,Point2* p1) const;

/** \brief Check if the triangulation contains triangles (which is the case if
* at least 3 non-collinear points exist in the triangulation
*
* As long as all inserted points are collinear the triangulation does not contain
* triangles. This is clearly the case as long as less than three input points are
* present but it may also be the case when 3 or more points have been inserted when
* all these points are collinear. These points are then in a pending state, i.e.
* they will be triangulated as soon as the first non-collinear point is inserted.
*
* \image html coll.jpg "Triangles are generated as soon as the first non-collinear point is inserted."
* \image latex coll.eps "Triangles are generated as soon as the first non-collinear point is inserted." width=7cm
*
* @return true if at least one triangle exists@n
* false otherwise
*
*/
	bool hasArea() const;

/** @private
 *  The function name is2D() was misleading. Thus it has been replaced
 *  by hasArea(). The deprecated name is2D() is kept for backwards
 *  compatibility
*/
	bool is2D() const;


/** \brief Add constraint edges (edges, polyline, polygon)
* \anchor createConstraint
* @param vSegments are segments which shall appear as edges of the
* triangulation. The segments may be automatically reordered and
* reoriented, see \p bOrientedSegments below.
* @param cis is the Constraint-Insertion-Strategy. Use always
* CIS_CONSTRAINED_DELAUNAY. This mode inserts the constraint segments
* \if SECTION_FADE25D
* at their original level (no projection onto the surface) and
* \endif
* without subdivision unless existing vertices or existing constraint
* segments are crossed. When subdivision (e.g., to achieve better triangle
* shapes) is desired then use ConstraintGraph2::makeDelaunay() after
* insertion.
* \if SECTION_FADE25D
* When the segments shall be adapted to the elevation of the existing
* surface then use Fade_2D::drape(). See the example code in
* \p examples_25D/terrain.cpp
* \endif
* @param bOrientedSegments specifies whether the segments in vSegments
* are oriented (<em>oriented, not ordered!</em>). If later a zone is to
* be made with the returned ConstraintGraph2 object this is only
* possible if the value is true (then it is assumed that all segments
* are counterclockwise oriented) or if the ConstraintGraph2 represents
* exactly one closed polygon. The value affects also the order of the
* returned vertices when later ConstraintGraph2::getPolygonVertices()
* is called. This is a default parameter and it defaults to false.
* \if SECTION_FADE25D
* @param bUseHeightOfLatest specifies that the height z of the last
* inserted segment is to be used. This is used in conflict case only,
* e.g. if the endpoint of an inserted segment already exists with a
* different height, or if an existing constraint segment is cut. By
* default, the height of the existing elements is used for the
* intersection point.
* \endif
*
* @return a pointer to the new ConstraintGraph2 object@n
*
*
* \htmlonly <div class="center"> \endhtmlonly
*
* \htmlonly <style>div.image img[src="bare-delaunay.png"]{width:40%;}</style> \endhtmlonly
* \image html bare-delaunay.png "Delaunay triangulation without constraints"
* \image latex bare-delaunay.eps "Delaunay triangulation without constraints" width=6cm

* \htmlonly <style>div.image img[src="constrained_delaunay.png"]{width:40%;}</style> \endhtmlonly
* \image html constrained_delaunay.png "Constrained Delaunay triangulation"
* \image latex constrained_delaunay.eps "Constraint Delaunay triangulation" width=6cm
*
* \htmlonly <style>div.image img[src="conforming-delaunay-makeDelaunay.png"]{width:40%;}</style> \endhtmlonly
* \image html conforming-delaunay-makeDelaunay.png "Conforming Delaunay triangulation through the ConstraintGraph::makeDelaunay() method"
* \image latex conforming-delaunay-makeDelaunay.png "Conforming Delaunay triangulation through the ConstraintGraph::makeDelaunay() method" width=6cm
* \htmlonly </div> \endhtmlonly
*
*/
#if GEOM_PSEUDO3D==GEOM_TRUE
	ConstraintGraph2* createConstraint(
		std::vector<Segment2>& vSegments,
		ConstraintInsertionStrategy cis,
		bool bOrientedSegments=false,
		bool bUseHeightOfLatest=false
		);
#else
	ConstraintGraph2* createConstraint(
		std::vector<Segment2>& vSegments,
		ConstraintInsertionStrategy cis,
		bool bOrientedSegments=false
		);
#endif





/** \brief Create a zone
*
* \anchor createZone
* A Zone2 object is an area of a triangulation, possibly bounded by
* a ConstraintGraph.
*
* @param zoneLoc is ZL_INSIDE, ZL_OUTSIDE or ZL_GLOBAL.
* @param pConstraintGraph points to a formerly created ConstraintGraph2
* object (which must be oriented and contain a \e simple polygon) or is
* NULL in case of zoneLoc==ZL_GLOBAL.
* @param bVerbose is by default true and causes a warning if NULL is returned.
*
* @return a pointer to the new Zone2 object or NULL if no triangles
* exist or pConstraintGraph->isOriented() returns false.@n
*
*
* \htmlonly <div class="center"> \endhtmlonly
* \image html bikeZones.jpg "Zones in a triangulation"
* \htmlonly </div> \endhtmlonly
*
* \image latex bikeZones.eps "Zones in a triangulation" width=6cm
*/
	Zone2* createZone(ConstraintGraph2* pConstraintGraph,ZoneLocation zoneLoc,bool bVerbose=true);


/** \brief Create a zone limited by multiple ConstraintGraph2 objects by growing from a start point
*
* A Zone2 object is an area of the traingulation, see \ref createZone
*
* @param vConstraintGraphs is a vector of ConstraintGraph objects
* @param zoneLoc must be ZL_GROW
* @param startPoint is the point from which the area is grown until
* the borders specified in vConstraintGraphs are reached
* @param bVerbose is by default true and causes a warning if NULL is returned.
* @return a pointer to the new Zone2 object (or NULL if zoneLoc!=ZL_GROW or no triangles exist)
*/
	Zone2* createZone(const std::vector<ConstraintGraph2*>& vConstraintGraphs,ZoneLocation zoneLoc,const Point2& startPoint,bool bVerbose=true);

/** \brief Create a zone limited by a ConstraintGraph by growing from a start point
*
* A Zone2 object is an area of the traingulation, see \ref createZone
*
* @param pConstraintGraph is a constraint whose edges specify the area's border
* @param zoneLoc must be ZL_GROW
* @param startPoint is the point from which the area is grown until the borders specified in pConstraint are reached
* @param bVerbose is by default true and causes a warning if NULL is returned.
* @return a pointer to the new Zone2 object (or NULL if zoneLoc!=ZL_GROW or no triangles exist)
*
*/

	Zone2* createZone(ConstraintGraph2* pConstraintGraph,ZoneLocation zoneLoc,const Point2& startPoint,bool bVerbose=true);

/** \brief Create a zone defined by a vector of triangles
*
* A Zone2 object is an area of the traingulation, see \ref createZone
*
* @param vTriangles
* @param bVerbose is by default true and causes a warning if NULL is returned.
* @return a pointer to the new Zone2 object (or NULL if vTriangles is empty)
*
*/

	Zone2* createZone( std::vector<Triangle2*>& vTriangles,bool bVerbose=true );



/** \brief Delete a Zone2 object
*
* Zone2 objects are automatically destroyed with their Fade_2D objects.
* In addition this method provides the possibility to eliminate Zone2
* objects earlier.
*
* \note Zones are designed transparently: When two zones Z1 and Z2 are
* combined to a new one Z3 (for example through a boolean operation)
* then Z1,Z2,Z3 form a tree such that changes in the leaf nodes Z1 and
* Z2 can propagate up to the root node Z3. For this reason Z1 and Z2
* must be alive as long as Z3 is used.
*/
	void deleteZone(Zone2* pZone);


/** \brief Apply conforming constraints and zones (deprecated!)
*
* This method establishes conforming constraint segments and zones
* which depend on them. For technical reasons conforming constraint
* segments are not immediately established but inserted at the end
* of the triangulation process. This step must be triggered manually
* i.e., it is up to the user to call applyConstraintsAndZones() before
* the resulting triangulation is used. If afterwards the triangulation
* is changed in any way, applyConstraintsAndZones() must be called again.
*
* \note The present function applyConstraintsAndZones() as well as the
* two constraint insertion strategies CIS_CONFORMING_DELAUNAY and
* CIS_CONFORMING_DELAUNAY_SEGMENT_LEVEL are deprecated. These are
* only kept for backwards compatibilty. The replacement is
* CIS_CONSTRAINED_DELAUNAY along with the methods Fade_2D::drape()
* and/or ConstraintGraph2::makeDelaunay(). See the example code in
* examples_25D/terrain.cpp
*/

	void applyConstraintsAndZones();


/** \brief Compute the axis-aligned bounding box of the points
*
* If no points have been inserted yet, then the returned Bbox2 object
* is invalid and its member function \e Bbox2::isValid() returns false.
*
*/
	Bbox2 computeBoundingBox() const;

/** \brief Check if an edge is a constraint edge
*
* Returns whether the edge in triangle \e pT which is opposite to the
* \e ith vertex is a constraint edge.
*
*/

	bool isConstraint(Triangle2* pT,int ith) const;

/** \brief Get active (alive) constraint segments
 */
	void getAliveConstraintSegments(std::vector<ConstraintSegment2*>& vAliveConstraintSegments) const;

/** \brief Get all (alive and dead) constraint segments
 */
	void getAliveAndDeadConstraintSegments(std::vector<ConstraintSegment2*>& vAllConstraintSegments) const;

/** \brief Retrieve a ConstraintSegment2
*
* @return a pointer to the ConstraintSegment2 between p0 and p1 or NULL
* if the segment is not a constraint edge (or dead because it has been
* splitted)
*
*/
	ConstraintSegment2* getConstraintSegment(Point2* p0,Point2* p1) const;


/** \brief Get incident triangles
*
* Stores pointers to all triangles around pVtx into vIncidentT
*/
	void getIncidentTriangles(Point2* pVtx,std::vector<Triangle2*>& vIncidentT) const;

/** \brief Get incident vertices
*
* Stores pointers to all vertices around pVtx into vIncidentVertices
*/
	void getIncidentVertices(Point2* pVtx,std::vector<Point2*>& vIncidentVertices) const;


/** \brief Write the current triangulation to an *.obj file
*
* Visualizes the current triangulation. The *.obj format represents a
* 3D scene.
*
*/

	void writeObj(const char*  filename) const;

/** \brief Write a zone to an *.obj file
*
* Visualizes a certain Zone2 object of the present triangulation. The
* *.obj format represents a 3D scene.
*
*/
	void writeObj(const char*  filename,Zone2* pZone) const;


/** \brief Write the current triangulation to an *.obj file
*
* Made for terrain visualizations in 2.5D but will work also for 2D.
*
*/
	void writeWebScene(const char* path) const;

/** \brief Write a zone to an *.obj file
*
* Made for terrain visualizations in 2.5D but will work also for 2D.
*
*/
	void writeWebScene(const char* path,Zone2* pZone) const;

/** \brief Register a message receiver
 *
 * @param msgType is the type of message the subscriber shall receive, e.g. MSG_PROGRESS or MSG_WARNING
 * @param pMsg is a pointer to a custom class derived from MsgBase
*/
void subscribe(MsgType msgType,MsgBase* pMsg);

/** \brief Unregister a message receiver
 *
 * @param msgType is the type of message the subscriber shall not receive anymore
 * @param pMsg is a pointer to a custom class derived from MsgBase
*/
void unsubscribe(MsgType msgType,MsgBase* pMsg);


/** \brief Check if an edge is a constraint edge
*
* Returns whether the edge (p0,p1) is a constraint edge.
*
*/

	bool isConstraint(Point2* p0,Point2* p1) const;

/** \brief Check if a vertex is a constraint vertex
*
* Returns whether the vertex \p pVtx belongs to a constraint edge.
*
*/
	bool isConstraint(Point2* pVtx) const;

	/// Prints license information
	void printLicense() const;

	/// @private
	bool checkZoneQuality(Zone2* pZone,double minAngle,const char*  name,const AcceptExperimentalFeature& accept);
	/// @private
	void setName(const char* s);
	/// @private
	const char* getName() const;


/** \brief Import triangles
*
* This method imports triangles into an empty Fade object. The triangles
* do not need to satisfy the empty circle property.
*
* \param vPoints contains the input vertices (3 subsequent ones per triangle)
* \param bReorientIfNeeded specifies if the orientations of the point triples
* shall be checked and corrected. If the point triples are certainly oriented
* in counterclockwise order then the orientation test can be skipped.
* \param bCreateExtendedBoundingBox can be used to insert 4 dummy points
* of an extended bounding box. This is convenient in some cases. Use
* false if you are unsure.
* \return a pointer to a Zone2 object or NULL if the input data is invalid
*
* \warning This method requires 100% correct input. A frequent source of
* trouble is when client software reads points from an ASCII file. The
* ASCII format is convenient but it can <strong>introduce rounding errors
* that cause intersections and flipped triangle orientations</strong>.
* Thus it is highly recommended to transfer point coordinates with binary
* files. See also readPointsBIN() and writePointsBIN().
*/
	Zone2* importTriangles(	std::vector<Point2>& vPoints,
							bool bReorientIfNeeded,
							bool bCreateExtendedBoundingBox
						);

/** \brief Compute the orientation of 3 points
*
* \return ORIENTATION2_COLLINEAR, ORIENTATION2_CW (clockwise) or ORENTATION2_CCW (counterclockwise)
*/
	Orientation2 getOrientation(const Point2& p0,const Point2& p1,const Point2& p2);

/** \brief Cut through a triangulation
*
* \param knifeStart is one point of the knife segment
* \param knifeEnd is the second point of the knife segment
* \param bTurnEdgesIntoConstraints turns all 3 edges of each intersected
* triangle into constraint segments.
*
* This method inserts a constraint edge \e knife(\e knifeStart,\e knifeEnd).
* If existing edges \e E are intersected by \e knife, then \e knife is
* subdivided at the intersection points \e P.
*
* In any case \e knife will exist (in a possibly subdivided form) in
* the result. But a consequence of the insertion of the points \e P
* is that the edges \e E and even edges which are not intersected by
* \e knife may be flipped. Use bTurnEdgesIntoConstraints=true to avoid
* that.
*
* \note The intersection point of two line segments may not be exactly
* representable in double precision floating point arithmetic and thus
* tiny rounding errors may occur. As a consequence two very close
* intersection points may be rounded to the same coordinates. \n
*
* \note When more than one knife segment is inserted then the method
* void cutTriangles(std::vector<Segment2>& vSegments,bool bTurnEdgesIntoConstraints)
* should be used. The reason is that each individual cut operation
* changes the triangulation and thus iterative calls to the present
* version of the method can lead to a different result. \n
*
*
*/
	void cutTriangles(	const Point2& knifeStart,
						const Point2& knifeEnd,
						bool bTurnEdgesIntoConstraints);

/** \brief Cut through a triangulation
*
* \param vSegments are the knife segments
* \param bTurnEdgesIntoConstraints specifies if intersected edges shall automatically be turned into constraints
*
* Same method as void cutTriangles(const Point2& knifeStart,const Point2& knifeEnd,bool bTurnEdgesIntoConstraints)
* but it takes a vector of segments instead of a single segment. This
* is the recommended method to cut through a triangulation when more
* than one knife segment exists.
*
*/

	void cutTriangles(	std::vector<Segment2>& vSegments,
						bool bTurnEdgesIntoConstraints);


/** \brief Cookie Cutter
 * The Cookie Cutter cuts out a part of a triangulation and returns it
 * as a Zone2 object.
 *
 * @param [in] vSegments specifies a simple polygon.
 * @param [in] bProtectEdges specifies if existing triangles
 * shall be protected with constraint segments.
 * @return a Zone2 object consisting of all triangles inside the polygon
 * or NULL when the operation has failed due to wrong preconditions.
 *
 * Properties: The input polygon ( \p vSegments) does not need to have
 * certain height values, the z-coordinates are computed automatically.
 * The input polygon is automatically trimmed when it is outside the
 * convex hull of the triangulation. Insertion of intersection points
 * may flip existing edges in the triangulation but this can be avoided
 * using bProtectEdges=true. In this case new constraint edges may
 * be created.
 *
 */
	Zone2* createZone_cookieCutter(std::vector<Segment2>& vSegments,bool bProtectEdges);


/** \brief Drape segments along a surface
 *
 * Projects the segments from \p vSegmentsIn onto the triangulation.
 * Thereby the segments are subdivided where they intersect edges of
 * the triangulation. Segment parts
 * outside the triangulation are cut off and ignored. Degenerate input
 * segments are also ignored.
 *
 * \if SECTION_FADE25D
 * The heights (z-values) of the result segments are adapted to the
 * surface.
 * @param [in] zTolerance is used to avoid excessive subdivision of
 * segments. Use some positive value to define the acceptable
 * geometric error or use \p zTolerance=-1.0 to split the segments
 * at all intersections with triangulation-edges.
 * \else
 * \endif
 *
 * @param [in] vSegmentsIn: Input segments
 * @param [out] vSegmentsOut: Output segments
 *
 * @return TRUE if all input segments are inside the convex hull
 * of the triangulation. Otherwise FALSE is returned and the result
 * is still valid but it contains only the segment parts inside the convex hull.
 *
 * @note The tiny rounding errors that occur when segment intersections
 * are computed are largely theoretical. But be aware that subdivided
 * segments are not always perfectly collinear. This can't be avoided
 * because the exact split point is sometimes not even representable
 * using floating point coordinates.
 *
 * \if SECTION_FADE25D
 * \htmlonly <style>div.image img[src="drape.png"]{width:100%;}</style> \endhtmlonly
 * * \image html drape.png "Drape: Input segments (blue) are draped (red) onto a TIN. Left with tolerance 1.0, right without tolerance"
 * \image latex drape.eps "Drape: Input segments (blue) are draped (red) onto a TIN. Left with tolerance 1.0, right without tolerance" width=10cm
 * \else
 * \htmlonly <style>div.image img[src="drape2d.png"]{width:70%;}</style> \endhtmlonly
 * * \image html drape2d.png "Drape: Input segments are draped (red) onto a TIN. They are subdivided (blue) at intersections with triangulation edges"
 * \image latex drape2d.eps "Drape: Input segments are draped (red) onto a TIN. They are subdivided (blue) at intersections with triangulation edges" width=6cm
 * \endif
 *
 * @note Draping segments onto a TIN does not insert them. Use
 * Fade_2D::createConstraint() for that purpose.
 */

#if GEOM_PSEUDO3D==GEOM_TRUE
	bool drape(	std::vector<Segment2>& vSegmentsIn,
				std::vector<Segment2>& vSegmentsOut,
				double zTolerance) const;
#else
	bool drape(	std::vector<Segment2>& vSegmentsIn,
				std::vector<Segment2>& vSegmentsOut) const;
#endif




/** \brief Get directed edges
* Edges are counterclockwise oriented around their triangle. The
* present method returns directed edges. That means each edge(a,b)
* is returend twice, as edge(a,b) and as edge(b,a).
*/
void getDirectedEdges(std::vector<Edge2>& vDirectedEdgesOut) const;

/** \brief Get undirected edges
 *
 * @param vUndirectedEdgesOut is used to return a unique set of
 * undirected edges.
*/
void getUndirectedEdges(std::vector<Edge2>& vUndirectedEdgesOut) const;



/// @private
	/* Deprecated: Use setNumCPU() instead. This method is kept for
	* compatibility with existing applications. Internally it calls
	* setNumCPU(0) to automatically determine and use the number of
	* available CPU cores.
	*/
	void enableMultithreading();
	// Development functions, not for public use
/// @private
	void internal(int au,int fu,const char*  s="");
/// @private
	Dt2* getImpl();
/// @private
	void setDev(const char* s,int ival,double dval);

protected:
/// @private
	void initFade(unsigned numExpectedVertices);
/// @private
	Fade_2D(const Fade_2D&); // No copy constructor
/// @private
	Fade_2D& operator=(const Fade_2D&); // No assignment allowed
/// @private
	Dt2* pImpl;
}; // end of class





} // (namespace)

