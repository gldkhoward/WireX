/*
* WireX  -  WireCenter
*
* Copyright (c) 2006-2019 Andreas Pott
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

/*! \file PathGenerator.h
 *
 *	\author   Andreas Pott
 *
 */

#pragma once

#include <IPAGL/GLShape.h>
#include <vector>

/*! \class CPathGenerator
 *  Class for generating axis parallel lines from a STL file. 
 *  These lines can be used to generate a robot program for welding.
 *  The functions in the class are specialized to hard-coded directions,
 *  e.g. the xy-plane, z-direction, and y-axis
 */
class CPathGenerator : public CGLShape
{
	std::vector<CVector3> vertices;		//!< list of normals and vertices
	std::vector<CVector3> path;			//!< vector of the path
	std::vector<CVector3> crosssection;	//!< vertices of the cross sections parallel to xy-plane
	std::vector<CVector3> vertices_normals;	//!< list of normals of the vertices
	std::vector<CVector3> path_normals;	 //!< list of normals of the path
	bool bExternalBB;					//!< enable the external bounding box (default: disabled)
	CVector3 BBmin_external;			//!< min limit of the bounding box defining the scope of PG
	CVector3 BBmax_external;			//!< max limit of the bounding box defining the scope of PG
	bool bMeanderShaped;		//!< the direction of the path in x-direction is alternating every z layer. so the path becomes meander shaped
public:
	GLenum drawmode;				//!< select the drawmode
	double eps_PathDist;			//!< distance between paths in x-direction
	double eps_LayerDist;			//!< distance between planes in z-direction
	int nErrors;					//!< a simple counter for the number of "special cases" occured during intersection
	
	
private:
	//! internal method for intersecting lines
	CVector3 intersection(const CVector3 &a, const CVector3 &b, const double& z);
	//! internal method for calulating the line (p1,p2) resulting from the intersection of a triangle (a,b,c) with an xy-plane for a given z value
	bool getIntersectionTriangleZPlane(const CVector3& a, const CVector3& b, const CVector3 &c, const double &z, CVector3 &p1, CVector3 &p2);

public:
	CPathGenerator(void);
	~CPathGenerator(void);

	//! load a geoemtry file from STL data
	void load(const std::string& filename);
	//! create the path (line segments parallel to y-axis)
	bool generatePath();
	//! save the path to a simple text file
	void savePath(const std::string& filename);
	//! draw the calculated path with openGL 
	void draw();
	//! enable the external bounding box for defining the scope of PG
	void setUseExternalBoundingBox(bool bExternalBB_) { bExternalBB=bExternalBB_;};
	//! set the values of the external bounding box
	void setExternalBoundingBox(const CVector3& BBmin_extern_, const CVector3& BBmax_extern_);
	//! set meander shaped path
	void setMeanderShapedPath(bool bMeanderShaped_) {bMeanderShaped=bMeanderShaped_;};
};
