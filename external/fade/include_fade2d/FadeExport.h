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

/// @file FadeExport.h
#pragma once

#include <vector>
#include <algorithm>
#include "common.h"

#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif

/** \brief FadeExport is a simple struct to export triangulation data
 *
 * This data structure is there to get data out of Fade easily and
 * memory efficiently. <b>The source code of this class is deliberately
 * included in the header file</b> so that users can take over the code
 * to their individual project.
 *
 * Have a look at the <a href="https://www.geom.at/triangulation-export/">Examples</a>.
 */
struct CLASS_DECLSPEC FadeExport
{
	FadeExport():
		numCustomIndices(0),numTriangles(0),numPoints(0),
		aCoords(NULL),aCustomIndices(NULL),aTriangles(NULL)
	{
#if GEOM_PSEUDO3D==GEOM_TRUE
		dim=3;
#else
		dim=2;
#endif
	}
	~FadeExport();

	// Methods
	void lexiSort();

	/// Print data for demonstration purposes
	void print() const;

	/// Write an *.obj file (supported by virtually any 3D viewer)
	bool writeObj(const char* filename) const;

	/// Determine index-pairs of adjacent triangles
	void extractTriangleNeighborships(std::vector<std::pair<int,int> >& vNeigs) const;

	/** \brief Get the corner indices of a certain triangle
	 *
	 * \param triIdx [in] triangle index
	 * \param vtxIdx0,vtxIdx1,vtxIdx2 [out] corner indices
	 */
	void getCornerIndices(int triIdx,int& vtxIdx0,int& vtxIdx1,int& vtxIdx2) const;

#if GEOM_PSEUDO3D==GEOM_TRUE
	/** \brief Get the coorinates for a certain vertex index
	 *
	 * \param vtxIdx [in] vertex index
	 * \param x,y,z [out] coordinates
	 */
	void getCoordinates(int vtxIdx,double& x,double& y,double& z) const;
#else
	/** \brief Get the coorinates for a certain vertex index
	 *
	 * \param vtxIdx [in] vertex index
	 * \param x,y [out] coordinates
	 */
	void getCoordinates(int vtxIdx,double& x,double& y) const;
#endif

	// DATA
	int numCustomIndices; ///<number of custom indices (same as numPoints when exported, otherwise 0)
	int numTriangles;///< number of triangles
	int numPoints; ///< number of points
	double* aCoords; ///< Cartesian coordinates (dim*numPoints)
	int* aCustomIndices; ///< Custom indices of the points (only when exported)
	int* aTriangles; ///< 3 counterclockwise oriented vertex-indices per triangle (3*numTriangles)
	int dim; ///< Dimension
};

inline FadeExport::~FadeExport()
{
	if(aCoords!=NULL) delete [] aCoords;
	if(aCustomIndices!=NULL) delete [] aCustomIndices;
	if(aTriangles!=NULL) delete [] aTriangles;
	numCustomIndices=0;
	numTriangles=0;
	numPoints=0;
}



// For a triangle return the vertex indices
inline void FadeExport::getCornerIndices(int triIdx,int& vtxIdx0,int& vtxIdx1,int& vtxIdx2) const
{
	int base(3*triIdx);
	vtxIdx0=aTriangles[base];
	vtxIdx1=aTriangles[base+1];
	vtxIdx2=aTriangles[base+2];
}

// Print, just for demo purposes
inline void FadeExport::print() const
{
	for(int vtxIdx=0;vtxIdx<numPoints;++vtxIdx)
	{
		int customIndex(-1); // Optional custom index
		if(numCustomIndices>0) customIndex=aCustomIndices[vtxIdx];
		std::cout<<"\nVertex "<<vtxIdx<<" (customIndex="<<customIndex<<"):";
		for(int component=0;component<dim;++component) std::cout<<" "<<aCoords[dim*vtxIdx+component];
	}

	for(int triIdx=0;triIdx<numTriangles;++triIdx)
	{
		int v0,v1,v2;
		getCornerIndices(int(triIdx),v0,v1,v2);
		std::cout<<"\nTriangle "<<triIdx<<": "<<v0<<" "<<v1<<" "<<v2;
	}

	std::vector<std::pair<int,int> > vNeighbors;
	this->extractTriangleNeighborships(vNeighbors);
	for(size_t i=0;i<vNeighbors.size();++i)
	{
		std::cout<<"\nTriangle "<<vNeighbors[i].first<<" <-> Triangle "<<vNeighbors[i].second;
	}
	std::cout<<std::endl;
}

// Write an *.obj file
inline bool FadeExport::writeObj(const char* filename) const
{
	std::ofstream outFile(filename);
	if(!outFile.is_open())
	{
		std::cout<<"Can't write "<<filename<<std::endl;
		return false;
	}
	std::cout<<"writing "<<filename<<std::endl;

	outFile<<"# Written by Fade2D";
	for(int vtxIdx=0;vtxIdx<numPoints;++vtxIdx)
	{
		outFile<<"\nv";
		for(int component=0;component<dim;++component) outFile<<" "<<aCoords[dim*vtxIdx+component];
		if(dim==2) outFile<<" 0"; // *.obj needs always 3 components, so add z=0
	}
	for(int triIdx=0;triIdx<numTriangles;++triIdx)
	{
		outFile<<"\nf";
		for(int corner=0;corner<3;++corner)
		{
			outFile<<" "<<aTriangles[3*triIdx+corner]+1; // +1 because in *.obj format indices start at 1, not 0.
		}
	}
	outFile<<std::endl;
	outFile.close();
	return true;
}


inline void FadeExport::extractTriangleNeighborships(std::vector<std::pair<int,int> >& vNeigs) const
{
	vNeigs.reserve(numTriangles*3/2);
	std::vector<std::pair<std::pair<int,int>,int> > vVtxPair2Tri;
	vVtxPair2Tri.reserve(numTriangles*3);

	for(int tri=0;tri<numTriangles;++tri)
	{
		size_t vtxIdx(3*tri);
		int vtx0(aTriangles[vtxIdx]);
		int vtx1(aTriangles[vtxIdx+1]);
		int vtx2(aTriangles[vtxIdx+2]);
		if(vtx0>vtx1) std::swap(vtx0,vtx1);
		if(vtx1>vtx2)
		{
			std::swap(vtx1,vtx2);
			if(vtx0>vtx1) std::swap(vtx0,vtx1);
		}
		vVtxPair2Tri.push_back(std::make_pair(std::make_pair(vtx0,vtx1),tri));
		vVtxPair2Tri.push_back(std::make_pair(std::make_pair(vtx1,vtx2),tri));
		vVtxPair2Tri.push_back(std::make_pair(std::make_pair(vtx0,vtx2),tri));
	}
	std::sort(vVtxPair2Tri.begin(),vVtxPair2Tri.end());
	for(size_t i=0;i<vVtxPair2Tri.size();++i)
	{
		int vtx0(vVtxPair2Tri[i].first.first);
		int vtx1(vVtxPair2Tri[i].first.second);
		int tri(vVtxPair2Tri[i].second);
		if(	++i<vVtxPair2Tri.size() &&
			vVtxPair2Tri[i].first.first==vtx0 &&
			vVtxPair2Tri[i].first.second==vtx1)
		{
			vNeigs.push_back(std::pair<int,int>(tri,vVtxPair2Tri[i].second));
		}
		--i;
	}
}


#if GEOM_PSEUDO3D==GEOM_TRUE
inline void FadeExport::getCoordinates(int vtxIdx,double& x,double& y,double& z) const
{
	int base(dim*vtxIdx);
	x=aCoords[base];
	y=aCoords[base+1];
	z=aCoords[base+2];
}
#else
inline void FadeExport::getCoordinates(int vtxIdx,double& x,double& y) const
{
	int base(dim*vtxIdx);
	x=aCoords[base];
	y=aCoords[base+1];
}
#endif




} // (namespace)





