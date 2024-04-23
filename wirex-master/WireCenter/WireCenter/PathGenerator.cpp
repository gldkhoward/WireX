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

#include "StdAfx.h"
#include "PathGenerator.h"
#include <iostream>
#include <fstream>

CPathGenerator::CPathGenerator(void)
{
	drawmode=GL_TRIANGLES;
	eps_PathDist = 1e-1;
	eps_LayerDist = 1e-1;
	nErrors=0;
	bExternalBB=false;
	bMeanderShaped=true;
}

CPathGenerator::~CPathGenerator(void)
{
}

/*! load a stl file */
void CPathGenerator::load(const std::string& filename)
{
	vertices.clear();
	// load the STL file
	try {
		std::ifstream file(filename.c_str(),ios::binary);
		char buf[80];
		file.read(buf,80);
		int anz=0;
		file.read((char*)&anz,4);
		CSTLRecord patch;
		while (anz-- && !file.eof())
		{
			file.read((char*)&patch,50);
			// cast the vertex and normal data from float (STL-patch) to double (GL-patches)
//			vertices.push_back(CVector3(patch.n.x,patch.n.y,patch.n.z)); at the moment the normals are discarded
			vertices.push_back(CVector3(patch.a.x,patch.a.y,patch.a.z));
			vertices.push_back(CVector3(patch.b.x,patch.b.y,patch.b.z));
			vertices.push_back(CVector3(patch.c.x,patch.c.y,patch.c.z));
		}
		printf("Triangles loaded: %i\n",vertices.size()/3);
	}
	catch (...)
	{ std::cout << "Error while Loading STL file: " << filename.c_str() << endl;	}
}


CVector3 CPathGenerator::intersection(const CVector3 &a, const CVector3 &b, const double& z)
{
	double lambda = ((z-a.z)/(b.z-a.z));
	return a + (b-a)*lambda;
}

/*! intersect a triangle (a,b,c) with a xy-plane of given z value. If the function returns true,
 *  the line segment is return in (p1,p2). Otherwise (p1,p2) is undefined.
 *  The algorithm handles most of special cases.
 */
bool CPathGenerator::getIntersectionTriangleZPlane(const CVector3& a, const CVector3& b, const CVector3 &c, const double& z, CVector3 &p1, CVector3& p2)
{
	// check if all vertices are on the same side of z
	// in this cases, no intersection occures
	if (a.z>z && b.z>z && c.z>z)
		return false;
	if (a.z<z && b.z<z && c.z<z)
		return false;

	// special cases
	if (a.z==z && b.z==z && c.z==z)
	{
		// triangle is fully in plane
		nErrors++;//printf("Error in triangle intersection; triangle is fully in plane; this is not yet handles!\n");
		return false;
	}

	// two vertices exectly in the plane
	if (a.z==z && b.z==z)
	{ p1=a; p2=b; return true; }
	if (a.z==z && c.z==z)
	{ p1=a; p2=c; return true; }
	if (b.z==z && c.z==z)
	{ p1=b; p2=c; return true; }

	if (a.z==z)
	{
		// the other vertices are on the same side; no intersection
		if ((b.z-z)*(c.z-z)>0)
			return false;
		p1=a;
		p2=intersection(b,c,z);
		return true;
	}

	if (b.z==z)
	{
		// the other vertices are on the same side; no intersection
		if ((a.z-z)*(c.z-z)>0)
			return false;
		p1=b;
		p2=intersection(a,c,z);
		return true;
	}

	if (c.z==z)
	{
		// the other vertices are on the same side; no intersection
		if ((a.z-z)*(b.z-z)>0)
			return false;
		p1=a;
		p2=intersection(b,c,z);
		return true;
	}

	// general cases a and b on same side
	if ((a.z-z)*(b.z-z)>0)
	{
		p1=intersection(a,c,z);
		p2=intersection(b,c,z);
		return true;
	}

	// general case b and c on same side
	if ((b.z-z)*(c.z-z)>0)
	{
		p1=intersection(b,a,z);
		p2=intersection(c,a,z);
		return true;
	}

	// general case a and c on same side
	if ((a.z-z)*(c.z-z)>0)
	{
		p1=intersection(b,c,z);
		p2=intersection(b,a,z);
		return true;
	}
	nErrors++;//	printf("An unhandles case occures in triangle plane intersection; this should never happen.\n");
	return false;
}

/*! calculate the path segments. The algorithm considered only shapes where
 *  exactly two intersecion between the path and the triangualted surface occur.
 *  Perhaps this is the most restrictive assumption in this implementation.
 */
bool CPathGenerator::generatePath()
{
	nErrors=0;				// count the number of errors during path generation
	time_t start=clock();	// and measure the time consumed
	
	if (vertices.size()==0)
		return false;

	// clean up internal memory for cross sections, path and normals
	path.clear();
	crosssection.clear();
	vertices_normals.clear();
	path_normals.clear();
	// calculate a bounding box according to the size of the STL-model (BBmin, BBmax)
	CVector3 BBmin=vertices[0];
	CVector3 BBmax=vertices[0];
	for (unsigned int i=0; i<vertices.size(); i++)
	{
		BBmin.x = min(BBmin.x,vertices[i].x); 
		BBmin.y = min(BBmin.y,vertices[i].y); 
		BBmin.z = min(BBmin.z,vertices[i].z); 
		BBmax.x = max(BBmax.x,vertices[i].x); 
		BBmax.y = max(BBmax.y,vertices[i].y); 
		BBmax.z = max(BBmax.z,vertices[i].z); 
	}

	if (bExternalBB) // apply external bounding box limits to define the scope of the path generator
	{
		BBmin.x = max(BBmin.x,BBmin_external.x); 
		BBmin.y = max(BBmin.y,BBmin_external.y); 
		BBmin.z = max(BBmin.z,BBmin_external.z); 
		BBmax.x = min(BBmax.x,BBmax_external.x); 
		BBmax.y = min(BBmax.y,BBmax_external.y); 
		BBmax.z = min(BBmax.z,BBmax_external.z); 
	}
	
	cout << "Bounding Box x-Range: " << BBmin.x << " " << BBmax.x << endl;
	cout << "Bounding Box y-Range: " << BBmin.y << " " << BBmax.y << endl;
	cout << "Bounding Box z-Range: " << BBmin.z << " " << BBmax.z << endl;

	// for each z-layer create cross sections and fill them with line parallel to y-axis
	cout << "Calculating cross sections\n";
	unsigned int cs_begin, cs_end;	// for path generation we only take into account the current cross section
	unsigned int n_begin, n_end;

	bool  bXDirection=true; // alternates between 1 and 0 if bMeanderShaped is true. 

	for (double z=BBmin.z; z<=BBmax.z; z+=eps_LayerDist)
	{
		cs_begin=crosssection.size();	
		n_begin=vertices_normals.size();
		// calculate the cross section
		CVector3 p1,p2;
		for (unsigned int i=0; i<vertices.size(); i+=3)
		{
			if (getIntersectionTriangleZPlane(vertices[i], vertices[i+1], vertices[i+2],z, p1, p2))
			{
				crosssection.push_back(p1);
				crosssection.push_back(p2);

				// vector orthogonal on the triangle
				CVector3 normal;
				cross(normal, vertices[i+1]- vertices[i],vertices[i+2]- vertices[i]); // establish the orthogonal vector with the cross product
				normal.normalize(); // normalize the orthogonal vector
				vertices_normals.push_back(normal); // save it in the list 
			}
		}
		cs_end=crosssection.size();
		n_end=vertices_normals.size();

		// create a list with parallel lines
		for (double x_=BBmin.x; x_<=BBmax.x; x_+=eps_PathDist)
		{
			double x;
			if (bXDirection)
				x=x_; // positive direction for x
			else
				x=BBmax.x-(x_-BBmin.x); // negative direction for x

			// initialize the range
			double miny=BBmin.y;
			double maxy=BBmax.y;
			vector<double> hitlist;
			hitlist.clear();
			unsigned int i_tmp=0; // position in the list of the selected cross section
			for (unsigned int i=cs_begin; i<cs_end; i+=2)
			{	// check each element in the cross section
				// intersection possible?
				if (crosssection[i].x > x && crosssection[i+1].x > x)
					continue;
				if (crosssection[i].x < x && crosssection[i+1].x < x)
					continue;
				// calculate the value for intersection
				double lambda = (x-crosssection[i].x)/(crosssection[i+1].x-crosssection[i].x);
				double y0 = crosssection[i].y + lambda*(crosssection[i+1].y-crosssection[i].y);

				hitlist.push_back(y0);
				i_tmp=i/2;
			}
			if (hitlist.size()<2)
				continue;	// no intersections here
			if (hitlist.size()==2)
			{	
				path.push_back(CVector3(x,hitlist[0],z));
				path.push_back(CVector3(x,hitlist[1],z));
				path_normals.push_back(vertices_normals[i_tmp]);
			}
			else
				nErrors++;//printf("There are multiple intersection; no inside/outside detection is implemented\n");			
		}

		if (bMeanderShaped)
			bXDirection=!bXDirection; // flipping the direction
	}

	// calculation finished; print some infos on the screen
	printf("Calculation Done. %i cross section line segment and %i path segments generated\n",crosssection.size(), path.size());
	if (nErrors>0)
		printf("%i conditions (errors) occured\n",nErrors);
	printf("Consumed time: %i [ms]\n",clock()-start);
	return true;
}

//! create a simple ASCII file with the path data
void CPathGenerator::savePath(const std::string& filename)
{
	std::ofstream file(filename.c_str());

	for (unsigned int i=0; i<path.size(); i+=2)
		file 
			<< path[i].x << " "
			<< path[i].y << " "
			<< path[i].z << " "
			<< path[i+1].x << " "
			<< path[i+1].y << " "
			<< path[i+1].z <<  " "
			<< path_normals[i/2].x << " "
			<< path_normals[i/2].y << " "
			<< path_normals[i/2].z << endl;
}

//! draw the cross sections and the path lines in openGL
void CPathGenerator::draw()
{
	// draw the cross sections
	glColor3d(0,1,0);
	glBegin(GL_LINES);
	for (unsigned int i = 0; i<crosssection.size(); i++)
		{
			glVertex3dv(&crosssection[i].x); i++;
			glVertex3dv(&crosssection[i].x);
		}
	glEnd();

	// draw the cross sections
	glColor3d(1,0,0);
	glBegin(GL_LINES);
	for (unsigned int i = 0; i<path.size(); i++)
		{
			glVertex3dv(&path[i].x); i++;
			glVertex3dv(&path[i].x);
		}
	glEnd();
}

//! set the values of the external bounding box
void CPathGenerator::setExternalBoundingBox(const CVector3& BBmin_external_, const CVector3& BBmax_external_)
{
	// check the range before assigning the values to verify that minBB < maxBB hold
	BBmin_external.x=min(BBmin_external_.x,BBmax_external_.x);
	BBmin_external.y=min(BBmin_external_.y,BBmax_external_.y);
	BBmin_external.z=min(BBmin_external_.z,BBmax_external_.z);
	BBmax_external.x=max(BBmin_external_.x,BBmax_external_.x);
	BBmax_external.y=max(BBmin_external_.y,BBmax_external_.y);
	BBmax_external.z=max(BBmin_external_.z,BBmax_external_.z);
}
