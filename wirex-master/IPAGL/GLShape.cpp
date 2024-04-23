/*
* WireX  -  IPAGL
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

/*! \file GLShape.cpp
 *
 *	\author   Andreas Pott
 */


#include "GLShape.h"
#include "GLScene.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include "Dib.h"

//********************************************************************
// Namespace IPAGL                                                   *
//********************************************************************

/*! Set number of facettes used for creating curved shapes
* /param facettes_ [in] Number of facettes used for curved shapes like circles, tubes, or cones.
*/
void IPAGL::setSceneAttributes(int facettes_)
{
	facettes = facettes_;
}


/*! Draw a box with an extent of (dx, dy, dz) centered around the origion
 *  of the coordinate system.
 */
void IPAGL::makeBox(const double& dx, const double& dy, const double& dz)
{
    double x=fabs(dx/2);
    double y=fabs(dy/2);
    double z=fabs(dz/2);
    glBegin(GL_QUADS);
        glNormal3d( 1, 0, 0);
        glVertex3d( x, y, z);
        glVertex3d( x,-y, z);
        glVertex3d( x,-y,-z);
        glVertex3d( x, y,-z);

        glNormal3d(-1, 0, 0);
        glVertex3d(-x, y, z);
        glVertex3d(-x, y,-z);
        glVertex3d(-x,-y,-z);
        glVertex3d(-x,-y, z);

        glNormal3d( 0, 1, 0);
        glVertex3d( x, y, z);
        glVertex3d( x, y,-z);
        glVertex3d(-x, y,-z);
        glVertex3d(-x, y, z);

        glNormal3d( 0,-1, 0);
        glVertex3d( x,-y, z);
        glVertex3d(-x,-y, z);
        glVertex3d(-x,-y,-z);
        glVertex3d( x,-y,-z);

        glNormal3d( 0, 0, 1);
        glVertex3d( x, y, z);
        glVertex3d(-x, y, z);
        glVertex3d(-x,-y, z);
        glVertex3d( x,-y, z);

        glNormal3d( 0, 0,-1);
        glVertex3d( x, y,-z);
        glVertex3d( x,-y,-z);
        glVertex3d(-x,-y,-z);
        glVertex3d(-x, y,-z);
    glEnd();
}


void IPAGL::makeFancyBox(const CVector3& a, const CVector3 &b, const double& width)
{
	// store the latest matrix
	glPushMatrix();
	// move the reference frame to point a
	glTranslated(a.x,a.y,a.z);
	// comput the respective matrix from a and b such that
	// the strut to be drawn is extruded in z-direction
	CVector3 d = b-a;
	double alpha=0,beta=0;
	if (d.x*d.x+d.y*d.y < 1e-6)	// almost pointing exactly upwards or downwards
	{
		alpha=0;
		if (d.z>0)
			beta=0;
		else
			beta=3.1415926;
	}
	else
	{
		alpha= atan2(d.y,d.x);
		beta = atan2(sqrt(d.x*d.x+d.y*d.y),d.z);
	}
	glRotated(180*alpha/3.1415926,0,0,1);
	glRotated(180*beta/3.1415926,0,1,0);

	double l=sqrt((a-b)*(a-b));	// the length of the box
	double w=width/2; // the half-width
	double W=width/6;
	double magic[16]={0};
/*	// the original version with QUAD_STRIP
	glBegin(GL_QUAD_STRIP);
		// we have to draw totally 22 faces, 5 for each side and the top and bottom
		glNormal3d(0,1,0);
		glVertex3d(-w, w, 0); glVertex3d(-w, w, l);
		glVertex3d(-W, w, 0); glVertex3d(-W, w, l);
		glNormal3d(1,0,0);
		glVertex3d(-W, W, 0); glVertex3d(-W, W, l);
		glNormal3d(0,1,0);
		glVertex3d( W, W, 0); glVertex3d( W, W, l);
		glNormal3d(-1,0,0);
		glVertex3d( W, w, 0); glVertex3d( W, w, l);
		glNormal3d(0,1,0);
		glVertex3d( w, w, 0); glVertex3d( w, w, l);

		// 2nd side
		glNormal3d(1,0,0);
		glVertex3d( w, W, 0); glVertex3d( w, W, l);
		glNormal3d(0,-1,0);
		glVertex3d( W, W, 0); glVertex3d( W, W, l);
		glNormal3d(1,0,0);
		glVertex3d( W,-W, 0); glVertex3d( W,-W, l);
		glNormal3d( 0,1,0);
		glVertex3d( w,-W, 0); glVertex3d( w,-W, l);
		glNormal3d(1,0,0);
		glVertex3d( w,-w, 0); glVertex3d( w,-w, l);

		// 3rd side
		glNormal3d(0,-1,0);
		glVertex3d( W, -w, 0); glVertex3d(W, -w, l);
		glNormal3d(-1,0,0);
		glVertex3d( W,-W, 0); glVertex3d( W, -W, l);
		glNormal3d(0,-1,0);
		glVertex3d(-W,-W, 0); glVertex3d(-W,-W, l);
		glNormal3d( 1,0,0);
		glVertex3d(-W,-w, 0); glVertex3d(-W,-w, l);
		glNormal3d(0,-1,0);
		glVertex3d(-w,-w, 0); glVertex3d(-w,-w, l);		

		// 4th side
		glNormal3d(-1,0,0);
		glVertex3d(-w,-W, 0); glVertex3d(-w,-W, l);
		glNormal3d(0, 1,0);
		glVertex3d(-W,-W, 0); glVertex3d(-W,-W, l);
		glNormal3d(-1,0,0);
		glVertex3d(-W, W, 0); glVertex3d(-W, W, l);
		glNormal3d( 0,-1,0);
		glVertex3d(-w, W, 0); glVertex3d(-w, W, l);
		glNormal3d(-1,0,0);
		glVertex3d(-w, w, 0); glVertex3d(-w, w, l);
	glEnd();


	glBegin(GL_QUADS);
		// draw the bottom
		glNormal3d(0,0,-1);
		glVertex3d(-w,-w,0);
		glVertex3d(-w, w,0);
		glVertex3d( w, w,0);
		glVertex3d( w,-w,0);

		glNormal3d(0,0, 1);
		glVertex3d(-w,-w,l);
		glVertex3d( w,-w,l);
		glVertex3d( w, w,l);
		glVertex3d(-w, w,l);
	glEnd();
*/
	// new implementation making the struct from five elementary boxes
	glPushMatrix(); glTranslated(         0,         0, l/2.0); makeBox(width/3.0, width/3.0, l); glPopMatrix();
	glPushMatrix(); glTranslated(-width/3.0,-width/3.0, l/2.0); makeBox(width/3.0, width/3.0, l); glPopMatrix();
	glPushMatrix(); glTranslated( width/3.0,-width/3.0, l/2.0); makeBox(width/3.0, width/3.0, l); glPopMatrix();
	glPushMatrix(); glTranslated( width/3.0, width/3.0, l/2.0); makeBox(width/3.0, width/3.0, l); glPopMatrix();
	glPushMatrix(); glTranslated(-width/3.0, width/3.0, l/2.0); makeBox(width/3.0, width/3.0, l); glPopMatrix();
	// restore the original matrix
	glPopMatrix();
}


/*! Draw a framed box with an extent of (dx, dy, dz) centered around the origion
 *  of the coordinate system.
 */
void IPAGL::makeWireBox(const double& dx, const double& dy, const double& dz)
{
    double x=fabs(dx/2);
    double y=fabs(dy/2);
    double z=fabs(dz/2);
    glBegin(GL_LINE_LOOP);
        glNormal3d( 1, 0, 0);
        glVertex3d( x, y, z);
        glVertex3d( x,-y, z);
        glVertex3d( x,-y,-z);
        glVertex3d( x, y,-z);
	glEnd();
    glBegin(GL_LINE_LOOP);
        glNormal3d(-1, 0, 0);
        glVertex3d(-x, y, z);
        glVertex3d(-x, y,-z);
        glVertex3d(-x,-y,-z);
        glVertex3d(-x,-y, z);
	glEnd();
    glBegin(GL_LINE_LOOP);
        glNormal3d( 0, 1, 0);
        glVertex3d( x, y, z);
        glVertex3d( x, y,-z);
        glVertex3d(-x, y,-z);
        glVertex3d(-x, y, z);
	glEnd();
    glBegin(GL_LINE_LOOP);
        glNormal3d( 0,-1, 0);
        glVertex3d( x,-y, z);
        glVertex3d(-x,-y, z);
        glVertex3d(-x,-y,-z);
        glVertex3d( x,-y,-z);
	glEnd();
    glBegin(GL_LINE_LOOP);
        glNormal3d( 0, 0, 1);
        glVertex3d( x, y, z);
        glVertex3d(-x, y, z);
        glVertex3d(-x,-y, z);
        glVertex3d( x,-y, z);
	glEnd();
    glBegin(GL_LINE_LOOP);
        glNormal3d( 0, 0,-1);
        glVertex3d( x, y,-z);
        glVertex3d( x,-y,-z);
        glVertex3d(-x,-y,-z);
        glVertex3d(-x, y,-z);
    glEnd();
}

/*! Draw a framed box with giving min_x, max_x, min_y, max_y, min_z, max_z coordinates
 *  of the coordinate system.
 */
void IPAGL::makeWireBox(const double& minx, const double& maxx, const double& miny, const double& maxy,const double& minz, const double& maxz)
{
	glBegin(GL_LINE_LOOP);
        glNormal3d( 1, 0, 0);
        glVertex3d( minx, maxy, maxz);
        glVertex3d( minx, miny, maxz);
        glVertex3d( minx, miny, minz);
        glVertex3d( minx, maxy, minz);
	glEnd();
    glBegin(GL_LINE_LOOP);
        glNormal3d(-1, 0, 0);
        glVertex3d( maxx, maxy, maxz);
        glVertex3d( maxx, maxy, minz);
        glVertex3d( maxx, miny, minz);
        glVertex3d( maxx, miny, maxz);
	glEnd();
    glBegin(GL_LINE_LOOP);
        glNormal3d( 0, 1, 0);
        glVertex3d( minx, maxy, maxz);
        glVertex3d( minx, maxy, minz);
        glVertex3d( maxx, maxy, minz);
        glVertex3d( maxx, maxy, maxz);
	glEnd();
    glBegin(GL_LINE_LOOP);
        glNormal3d( 0,-1, 0);
        glVertex3d( minx, miny, maxz);
        glVertex3d( maxx, miny, maxz);
        glVertex3d( maxx, miny, minz);
        glVertex3d( minx, miny, minz);
	glEnd();
    glBegin(GL_LINE_LOOP);
        glNormal3d( 0, 0, 1);
        glVertex3d( maxx, maxy, maxz);
        glVertex3d( minx, maxy, maxz);
        glVertex3d( minx, miny, maxz);
        glVertex3d( maxx, miny, maxz);
	glEnd();
    glBegin(GL_LINE_LOOP);
        glNormal3d( 0, 0,-1);
        glVertex3d( maxx, maxy, minz);
        glVertex3d( maxx, miny, minz);
        glVertex3d( minx, miny, minz);
        glVertex3d( minx, maxy, minz);
    glEnd();
}

/*! Draws a cylinder which axis is alligned with the y axis of the
 *  coordinate system with radius r. 
 *  \param r radius of the cylinder
 *  \param h legnth of the cylinder in y-direction. The length of the axis
 *  start from (1-h) and ends at 1.\n
 *  Note, that the strange convention for the length of the axis is compatible
 *  with the function auxSolidCylinder.\n
 *  This function implicitly depends on the global variable IPAGL::facettes with controlls
 *  the tessalation.
 */
void IPAGL::makeCylinder(const double& r, const double& h)
{
    glBegin(GL_QUAD_STRIP); 
        for (int i=0; i<=facettes; i++)
        {
            double s=sin(i*2*GL_PI/facettes);
            double c=cos(i*2*GL_PI/facettes);
            glNormal3d(c,0,s);
            glVertex3d(r*c,1-h,r*s);
            glVertex3d(r*c, 1,r*s);
        }
    glEnd();
}

/* Draws a closed cylinder along the y-axis that is symmetric to the x-axis
*/
void IPAGL::makeCylinder_Symmetric(const double& r, const double& h, const CVector3& color )
{
	// set appearance
	glEnable (GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

	glColor3d(color.x,color.y, color.z);

	// cylinder
    glBegin(GL_QUAD_STRIP); 
		for (int i=0; i<=facettes; i++)
		{
			double s=sin(i*2*GL_PI/facettes);
			double c=cos(i*2*GL_PI/facettes);
			glNormal3d(c,0,s);
			glVertex3d(r*c,-h/2,r*s);
			glVertex3d(r*c, h/2,r*s);
		}
	glEnd();

	// cap1
	glBegin(GL_TRIANGLE_FAN); 
		glVertex3d(0, h/2,0); // center point
		for (int i=0; i<=facettes; i++)
		{
			double s=sin(i*2*GL_PI/facettes);
			double c=cos(i*2*GL_PI/facettes);
			glNormal3d(0,1,0);
			glVertex3d(r*c, h/2,r*s);
		}
	glEnd();

	// cap2
	glBegin(GL_TRIANGLE_FAN); 
		glVertex3d(0, -h/2,0); // center point
		for (int i=0; i<=facettes; i++)
		{
			double s=sin(i*2*GL_PI/facettes);
			double c=cos(i*2*GL_PI/facettes);
			glNormal3d(0,-1,0);
			glVertex3d(r*c, -h/2,r*s);
		}
	glDisable(GL_BLEND);
	glEnd();

}


/*! Draws a triangular patch of a sphere by recursively subdivding the triangle
 *  The vectors v1, v2, v3 present the vertices and depth yields the number of
 *  remaining subdivisions.
 */
void IPAGL::makeSpherePatch(CVector3 &v1, CVector3 &v2, CVector3 &v3, int depth)
{
	double r=length(v1);
    if (depth==0)
    {
        glBegin(GL_TRIANGLES);
			v1.normalize();
            glNormal3dv(&v1.x);
			v1=v1*r;
            glVertex3dv(&v1.x);
			v2.normalize();
            glNormal3dv(&v2.x);
			v2=v2*r;
            glVertex3dv(&v2.x);
			v3.normalize();
            glNormal3dv(&v3.x);
			v3=v3*r;
            glVertex3dv(&v3.x);
        glEnd();
    }
    else
    {
        depth--;
        CVector3 v12=(v1+v2);v12.normalize();
		v12=v12*r;
        CVector3 v23=(v2+v3);v23.normalize();
		v23=r*v23;
        CVector3 v31=(v1+v3);v31.normalize();
		v31=r*v31;
        makeSpherePatch(v1,v12,v31,depth);
        makeSpherePatch(v2,v23,v12,depth);
        makeSpherePatch(v3,v31,v23,depth);
        makeSpherePatch(v12,v23,v31,depth);
    }
}

/*! Draws a sphere with radius r. The smoothness of the sphere is controlled through the
 *  global variable IPAGL::facettes. The number of recursive subdevisions is determined 
 *  through ld facettes. (with ld=log2)
 */
void IPAGL::makeSphere(const double& r)
{
    glPushMatrix();
        int depth = (int)(log((double)facettes)/log(2.0));
        if (depth>0) depth--;
		CVector3 v1( r,0,0), v2(0, r,0), v3(0,0,r),
			v4(-r,0,0), v5(0,-r,0), v6(0,0,-r);
		makeSpherePatch(v1,v2,v3,depth);
        makeSpherePatch(v1,v5,v6,depth);
        makeSpherePatch(v4,v5,v3,depth);
        makeSpherePatch(v4,v2,v6,depth);
        makeSpherePatch(v1,v3,v5,depth);
        makeSpherePatch(v1,v6,v2,depth);
        makeSpherePatch(v4,v3,v2,depth);
        makeSpherePatch(v4,v6,v5,depth);
    glPopMatrix();
}

/*! Draw a cone 
    \todo control the numer of surfaces on the circles through IPAGL::facettes
 */
void IPAGL::makeCone(double rate)
{
    GLdouble x,z,angle;  

	// Begin a triangle fan
	glBegin(GL_TRIANGLE_FAN);
		// Pinnacle of cone is shared vertex for fan, moved up Z axis
		// to produce a cone instead of a circle
		glNormal3d(0.0, 1, 0.0);
		glVertex3d(0.0, (rate*0.15), 0.0);
		
		// Loop around in a circle and specify even points along the circle
		// as the vertices of the triangle fan
		for(angle = 0.0f; angle <= (2.01*GL_PI); angle += (GL_PI/8.0))
		{
			// Calculate x and y position of the next vertex
			x = rate*0.06*sin(angle);
			z = rate*0.06*cos(angle);
			
			// Specify the next vertex for the triangle fan
			glNormal3d(x/(rate*0.06),0.0,z/(rate*0.06));
			glVertex3d(x,0.0,z);
		}
		// Done drawing fan for cone
	glEnd();

	// Begin a new triangle fan to cover the bottom
	glBegin(GL_TRIANGLE_FAN);
		// Center of fan is at the origin
		glNormal3d(0,-1,0);
		glVertex3d(0.0, 0.0,0.0);
		for(angle = 0.0; angle <= (2.01*GL_PI); angle += (GL_PI/8.0))
		{
			// Calculate x and y position of the next vertex
			x = rate*0.06*sin(angle);
			z = rate*0.06*cos(angle);
				
			// Specify the next vertex for the triangle fan
			glVertex3d(x,0.0,z);
		}
		// Done drawing the fan that covers the bottom
	glEnd();
}

/*! Draw a cone at apex along the axis with an aperture
 */
void IPAGL::makeCone(const CVector3& apex, const CVector3& axis, const double& aperture)
{
	// calculate an orthogonal base coordinate system from the given axis: [ e1 = axis/|axis|, e2, e3 ]
	CVector3 e1,e2,e3,ex(1,0,0),ey(0,1,0);
	e3 = axis; 
	e3.normalize();
	ex-=e3.x*e3;
	ey-=e3.y*e3;
	if (length(ex)>length(ey))
		e1=ex;
	else
		e1=ey;
	e2 = e3%e1;
	e1.normalize();
	e2.normalize();

	double phi;
	double r=length(axis)*tan(aperture);
	
	// draw the conic surface
	glBegin(GL_TRIANGLE_FAN);
		glVertex3dv(&apex.x);
		for (int i=0; i<=facettes; i++)
		{
			phi=double(i)*2*GL_PI/facettes;
			CVector3 n=r*cos(phi)*e1+r*sin(phi)*e2;
			glNormal3dv(&n.x);
			CVector3 p=apex+axis+n;
			glVertex3dv(&p.x);
		}
	glEnd();

	// draw the circle bottom
	CVector3 m=apex+axis;
	glNormal3dv(&axis.x);
	glBegin(GL_TRIANGLE_FAN);
		glVertex3dv(&m.x);
		for (int i=0; i<=facettes; i++)
		{
			phi=double(i)*2*GL_PI/facettes;
			CVector3 p=apex+axis+r*cos(phi)*e1+r*sin(phi)*e2;
			glVertex3dv(&p.x);
		}
	glEnd();	
}

/*! Draw a cone at apex along the axis with an aperture (user defined alpha settings)
 */
void IPAGL::makeCone(const CVector3& apex, const CVector3& axis, const double& aperture, const CVector3& color, const double& alpha)
{
	// calculate an orthogonal base coordinate system from the given axis: [ e1 = axis/|axis|, e2, e3 ]
	CVector3 e1,e2,e3,ex(1,0,0),ey(0,1,0);
	e3 = axis; 
	e3.normalize();
	ex-=e3.x*e3;
	ey-=e3.y*e3;
	if (length(ex)>length(ey))
		e1=ex;
	else
		e1=ey;
	e2 = e3%e1;
	e1.normalize();
	e2.normalize();

	double phi;
	double r=length(axis)*tan(aperture);
	
	// set appearance
	glEnable (GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glColor4d(color.x,color.y, color.z,alpha);

	// draw the conic surface
	glBegin(GL_TRIANGLE_FAN);
		glVertex3dv(&apex.x);
		for (int i=0; i<=facettes; i++)
		{
			phi=double(i)*2*GL_PI/facettes;
			CVector3 n=r*cos(phi)*e1+r*sin(phi)*e2;
			glNormal3dv(&n.x);
			CVector3 p=apex+axis+n;
			glVertex3dv(&p.x);
		}
	glEnd();

	// draw the circle bottom
	CVector3 m=apex+axis;
	glNormal3dv(&axis.x);
	glBegin(GL_TRIANGLE_FAN);
		glVertex3dv(&m.x);
		for (int i=0; i<=facettes; i++)
		{
			phi=double(i)*2*GL_PI/facettes;
			CVector3 p=apex+axis+r*cos(phi)*e1+r*sin(phi)*e2;
			glVertex3dv(&p.x);
		}
	glEnd();	
	glDisable(GL_BLEND);
}

/*! Draw an ellipsoid (user defined alpha settings)
 */
void IPAGL::makeEllipsoid(const CVector3& radius, const CVector3& color,const double& alpha)
{
	glScaled(radius.x, radius.y, radius.z);
	// set appearance
	glEnable (GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glColor4d(color.x, color.y, color.z,alpha);

    int depth = (int)(log((double)facettes)/log(2.0));
    if (depth>0) 
		depth--;
	
	CVector3 v1( 1,0,0), v2(0, 1,0), v3(0,0,1),
		v4(-1,0,0), v5(0,-1,0), v6(0,0,-1);
	
	makeSpherePatch(v1,v2,v3,depth);
    makeSpherePatch(v1,v5,v6,depth);
    makeSpherePatch(v4,v5,v3,depth);
    makeSpherePatch(v4,v2,v6,depth);
    makeSpherePatch(v1,v3,v5,depth);
    makeSpherePatch(v1,v6,v2,depth);
    makeSpherePatch(v4,v3,v2,depth);
    makeSpherePatch(v4,v6,v5,depth);
}


/*! draw a coordinate frame with color arrows for the axis around origin. If iHighlightedFrameDimension=0 (standard) nothing happens whereas values 1,2,3 
represents the x,y,z axis respectively and colors the corresponding axis and in addition a value of 4 sets the robot platform color to orange*/
void IPAGL::makeFrame(int iHighlightedFrameDimension)
{
	double FrameShrinkRate=1;
	glColor3d(0,0.8,0);
	
	glPushName(4);
	IPAGL::makeSphere(0.01*FrameShrinkRate);
	
    // y-axis
	glLoadName(2);
    glPushMatrix();
        glPushMatrix();
            glTranslated(0.0,(0.2*FrameShrinkRate-1),0.0);
			if (iHighlightedFrameDimension==2) glColor3d(1.0,0.5,0);
			else glColor3d(0,0.8,0);
            IPAGL::makeCylinder( 0.006*FrameShrinkRate,0.2*FrameShrinkRate );
        glPopMatrix();
        glTranslated(0.0,0.2*FrameShrinkRate,0.0);
        IPAGL::makeCone(0.2*FrameShrinkRate);
    glPopMatrix();
    // z-axis
	glLoadName(3);
    glPushMatrix();
        glRotated(90,1.0,0.0,0.0);
        glPushMatrix();
            glTranslated(0.0,(0.2*FrameShrinkRate-1),0.0);
			if (iHighlightedFrameDimension==3) glColor3d(1.0,0.5,0);
			else glColor3d(0,0,0.8);
            IPAGL::makeCylinder( 0.006*FrameShrinkRate,0.2*FrameShrinkRate );
        glPopMatrix();
        glTranslated(0.0,0.2*FrameShrinkRate,0.0);
        IPAGL::makeCone(0.2*FrameShrinkRate);
    glPopMatrix();
    // x-axis
	glLoadName(1);
	glPushMatrix();
        glRotated(-90,0.0,0.0,1.0);
        glPushMatrix();
            glTranslated(0.0,(0.2*FrameShrinkRate-1),0.0);
			if (iHighlightedFrameDimension==1) glColor3d(1.0,0.5,0);
			else glColor3d(0.8,0,0);
            IPAGL::makeCylinder( 0.006*FrameShrinkRate,0.2*FrameShrinkRate );
        glPopMatrix();
        glTranslated(0.0,0.2*FrameShrinkRate,0.0);
        IPAGL::makeCone(0.2*FrameShrinkRate);
    glPopMatrix();
	glPopName();
}

void IPAGL::makeRing(float fInnerRadius, float fOuterRadius)
{
	int elements = 40;
	glBegin(GL_TRIANGLE_STRIP); 
	glColor3d(1,1,1);
		for (int i=0; i<=elements; i++)
		{
			double s=sin(i*2*GL_PI/elements);
			double c=cos(i*2*GL_PI/elements);
			glNormal3d(0,0,1);
			glVertex3d(fInnerRadius*c,fInnerRadius*s,0);
			glVertex3d(fOuterRadius*c,fOuterRadius*s,0);			
		}
	glEnd();
};

void IPAGL::makeFrame(float fxAxisLength, float fyAxisLength, float fzAxisLength, bool bTransparent, bool bPaintRotationCircles, int nActiveAxis, bool bNamedAxes)
{
	glEnable(GL_CULL_FACE);
	float fScaleFactor = (fxAxisLength + fyAxisLength + fzAxisLength) / 3; //50
	float alpha = 0.2f;
	int elements = 40;
	float radius = (fxAxisLength + fyAxisLength + fzAxisLength) / 3;// 50.0f;
	fxAxisLength = max(fxAxisLength,radius+10);
	fyAxisLength = max(fyAxisLength,radius+10);
	fzAxisLength = max(fzAxisLength,radius+10);
	if (bTransparent) 
	{
		glEnable(GL_BLEND);	
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glColor4f(0,0.8f,0,alpha);
	}
	else glColor3d(0,0.8,0);
	
	if (bNamedAxes) glPushName(7);
	if (nActiveAxis == -1) IPAGL::makeSphere(0.05*fScaleFactor);
	
    // y-axis	
	if (bNamedAxes) glLoadName(2);
    glPushMatrix();
        glPushMatrix();
            glTranslated(0.0,/*(0.2*fScaleFactor-1)*/fyAxisLength,0.0);
			if (bTransparent) 
			{
				glEnable(GL_BLEND);	
				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				glColor4f(0,0.8f,0,alpha);
			}
			else glColor3d(0,0.8,0);
            if (nActiveAxis == -1 || nActiveAxis == 2) IPAGL::makeCylinder( 0.03*fScaleFactor,fyAxisLength );
        glPopMatrix();
        glTranslated(0.0,fyAxisLength,0.0);
        if (nActiveAxis == -1 || nActiveAxis == 2) IPAGL::makeCone(fScaleFactor);
    glPopMatrix();
	if (bPaintRotationCircles && (nActiveAxis == -1 || nActiveAxis == 5))
	{
		if (bNamedAxes) glLoadName(5);
		if (bNamedAxes) glPushName(1);
		glPushMatrix();
		glTranslated(0,0.5,0);	
		//glBegin(GL_TRIANGLE_FAN); 
		//glVertex3d(0, 0,0); // center point
		//for (int i=0; i<=elements; i++)
		//{
		//	double s=sin(i*2*GL_PI/elements);
		//	double c=cos(i*2*GL_PI/elements);
		//	glNormal3d(0,1,0);
		//	glVertex3d(radius*s,0,radius*c);
		//}
		glBegin(GL_TRIANGLE_STRIP); 		
		for (int i=0; i<=elements; i++)
		{
			double s=sin(i*2*GL_PI/elements);
			double c=cos(i*2*GL_PI/elements);
			glNormal3d(0,1,0);
			glVertex3d(radius*0.9*s,0,radius*0.9*c);
			glVertex3d(radius*s,0,radius*c);			
		}
		glEnd();
		glPopMatrix();
		if (bNamedAxes) glPopName();
		if (bNamedAxes) glPushName(2);
		glPushMatrix();
		glTranslated(0,-0.5,0);	
		//glBegin(GL_TRIANGLE_FAN); 
		//glVertex3d(0, 0,0); // center point
		//for (int i=0; i<=elements; i++)
		//{
		//	double s=sin(i*2*GL_PI/elements);
		//	double c=cos(i*2*GL_PI/elements);
		//	glNormal3d(0,-1,0);
		//	glVertex3d(radius*c,0,radius*s);
		//}
		glBegin(GL_TRIANGLE_STRIP); 		
		for (int i=0; i<=elements; i++)
		{
			double s=sin(i*2*GL_PI/elements);
			double c=cos(i*2*GL_PI/elements);
			glNormal3d(0,-1,0);
			glVertex3d(radius*0.9*c,0,radius*0.9*s);
			glVertex3d(radius*c,0,radius*s);			
		}
		glEnd();
		glPopMatrix();
		if (bNamedAxes) glPopName();
	}

    // z-axis
	if (bNamedAxes) glLoadName(3);
    glPushMatrix();
        glRotated(90,1.0,0.0,0.0);
        glPushMatrix();
            glTranslated(0.0,/*(0.2*fScaleFactor-1)*/fzAxisLength,0.0);
			if (bTransparent) 
			{
				glEnable(GL_BLEND);	
				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				glColor4f(0,0,0.8f,alpha);
			}
			else glColor3d(0,0,0.8);
            if (nActiveAxis == -1 || nActiveAxis == 3) IPAGL::makeCylinder( 0.03*fScaleFactor,fzAxisLength );
        glPopMatrix();
        glTranslated(0.0,fzAxisLength,0.0);
        if (nActiveAxis == -1 || nActiveAxis == 3) IPAGL::makeCone(fScaleFactor);
    glPopMatrix();
	if (bPaintRotationCircles && (nActiveAxis == -1 || nActiveAxis == 6))
	{
		if (bNamedAxes) glLoadName(6);
		if (bNamedAxes) glPushName(1);
		glPushMatrix();
		glTranslated(0,0,0.5);	
		//glBegin(GL_TRIANGLE_FAN); 
		//glVertex3d(0, 0,0); // center point
		//for (int i=0; i<=elements; i++)
		//{
		//	double s=sin(i*2*GL_PI/elements);
		//	double c=cos(i*2*GL_PI/elements);
		//	glNormal3d(0,0,1);
		//	glVertex3d(radius*c,radius*s,0);
		//}
		glBegin(GL_TRIANGLE_STRIP); 		
		for (int i=0; i<=elements; i++)
		{
			double s=sin(i*2*GL_PI/elements);
			double c=cos(i*2*GL_PI/elements);
			glNormal3d(0,0,1);
			glVertex3d(radius*0.9*c,radius*0.9*s,0);
			glVertex3d(radius*c,radius*s,0);			
		}
		glEnd();
		glPopMatrix();
		if (bNamedAxes) glPopName();
		if (bNamedAxes) glPushName(2);
		glPushMatrix();
		glTranslated(0,0,-0.5);	
		//glBegin(GL_TRIANGLE_FAN); 
		//glVertex3d(0, 0,0); // center point
		//for (int i=0; i<=elements; i++)
		//{
		//	double s=sin(i*2*GL_PI/elements);
		//	double c=cos(i*2*GL_PI/elements);
		//	glNormal3d(0,0,-1);
		//	glVertex3d(radius*s,radius*c,0);
		//}
		glBegin(GL_TRIANGLE_STRIP); 		
		for (int i=0; i<=elements; i++)
		{
			double s=sin(i*2*GL_PI/elements);
			double c=cos(i*2*GL_PI/elements);
			glNormal3d(0,0,-1);
			glVertex3d(radius*0.9*s,radius*0.9*c,0);
			glVertex3d(radius*s,radius*c,0);			
		}
		glEnd();
		glPopMatrix();
		if (bNamedAxes) glPopName();
	}

    // x-axis
	if (bNamedAxes) glLoadName(1);
	glPushMatrix();
        glRotated(-90,0.0,0.0,1.0);
        glPushMatrix();
            glTranslated(0.0,/*(0.2*fScaleFactor-1)*/fxAxisLength,0.0);
			if (bTransparent) 
			{
				glEnable(GL_BLEND);	
				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				glColor4f(0.8f,0,0,alpha);
			}
			else glColor3d(0.8,0,0);
            if (nActiveAxis == -1 || nActiveAxis == 1) IPAGL::makeCylinder( 0.03*fScaleFactor,fxAxisLength );
        glPopMatrix();
        glTranslated(0.0,fxAxisLength,0.0);
        if (nActiveAxis == -1 || nActiveAxis == 1) IPAGL::makeCone(fScaleFactor);
    glPopMatrix();
	if (bPaintRotationCircles && (nActiveAxis == -1 || nActiveAxis == 4))
	{
		if (bNamedAxes) glLoadName(4);
		if (bNamedAxes) glPushName(1);
		glPushMatrix();
		glTranslated(0.5,0,0);	
		//glBegin(GL_TRIANGLE_FAN); 
		//glVertex3d(0, 0,0); // center point
		//for (int i=0; i<=elements; i++)
		//{
		//	double s=sin(i*2*GL_PI/elements);
		//	double c=cos(i*2*GL_PI/elements);
		//	glNormal3d(1,0,0);
		//	glVertex3d(0,radius*c,radius*s);
		//}
		glBegin(GL_TRIANGLE_STRIP); 		
		for (int i=0; i<=elements; i++)
		{
			double s=sin(i*2*GL_PI/elements);
			double c=cos(i*2*GL_PI/elements);
			glNormal3d(1,0,0);
			glVertex3d(0,radius*0.9*c,radius*0.9*s);
			glVertex3d(0,radius*c,radius*s);			
		}
		glEnd();
		glPopMatrix();
		if (bNamedAxes) glPopName();
		if (bNamedAxes) glPushName(2);
		glPushMatrix();
		glTranslated(-0.5,0,0);	
		//glBegin(GL_TRIANGLE_FAN); 
		//glVertex3d(0, 0,0); // center point
		//for (int i=0; i<=elements; i++)
		//{
		//	double s=sin(i*2*GL_PI/elements);
		//	double c=cos(i*2*GL_PI/elements);
		//	glNormal3d(-1,0,0);
		//	glVertex3d(0,radius*s,radius*c);
		//}
		glBegin(GL_TRIANGLE_STRIP); 		
		for (int i=0; i<=elements; i++)
		{
			double s=sin(i*2*GL_PI/elements);
			double c=cos(i*2*GL_PI/elements);
			glNormal3d(-1,0,0);
			glVertex3d(0,radius*0.9*s,radius*0.9*c);
			glVertex3d(0,radius*s,radius*c);			
		}
		glEnd();
		glPopMatrix();
		if (bNamedAxes) glPopName();
	}
	if (bNamedAxes) glPopName();
	glDisable(GL_CULL_FACE);
}


// draw 3 circles around the coordinate axes
void IPAGL::makeRotationFrame(int iHighlightedFrameDimension)
{
	double FrameShrinkRate = 1;
	double radius = 0.1;
	int elements = 30;
	
	// x-Axis
	glPushName(1);	           
	
	glColor4d(0.8,0,0,0);
    glBegin(GL_TRIANGLE_FAN); 
	glVertex3d(0, 0,0); // center point
	for (int i=0; i<=elements; i++)
	{
		double s=sin(i*2*GL_PI/elements);
		double c=cos(i*2*GL_PI/elements);
		glNormal3d(1,0,0);
		glVertex3d(0,radius*c,radius*s);
	}
	glEnd();
	
    // y-axis
	glLoadName(2);                
	
	glColor4d(0,0.8,0,3000);
    glBegin(GL_TRIANGLE_FAN); 
	glVertex3d(0, 0,0); // center point
	for (int i=0; i<=elements; i++)
	{
		double s=sin(i*2*GL_PI/elements);
		double c=cos(i*2*GL_PI/elements);
		glNormal3d(0,1,0);
		glVertex3d(radius*c, 0,radius*s);
	}
	glEnd();
	
	// z-axis
	glLoadName(3);	          
	
	glColor4d(0,0,0.8,0.002);
    glBegin(GL_TRIANGLE_FAN); 
	glVertex3d(0, 0,0); // center point
	for (int i=0; i<=elements; i++)
	{
		double s=sin(i*2*GL_PI/elements);
		double c=cos(i*2*GL_PI/elements);
		glNormal3d(0,0,1);
		glVertex3d(radius*c, radius*s,0);
	}
	glEnd();
	glPopName();
}

/*! draw a 3d vector (arrow) defined by two points using a given radius, color and alpha value 
 *  \param v1 [in] start point of the arrow
 *  \param v2 [in] end point of the arrow
 *  \param radius [in] the radius of the shaft(?) of the arrow
 *  \param color [in] vector of r g b values of the arrow in [0,1] range
 *  \param alpha [in] alpha color value to control transparency of the arrow
 */
void IPAGL::makeVector(const CVector3 &v1, const CVector3 &v2, const double &radius, const CVector3& color, const double& alpha )
{
	CVector3 yAxis(0,1,0);
	CVector3 axis = v2-v1;

	// add small value, if vector points in y direction
	if (abs(axis.x) + abs(axis.z) < 0.000001)
	{
		axis.x = 0.0000011;
	}

	double axisLength = length(axis);
	double aperture = 0.3;
	double h_cone = 2.3* radius / tan(aperture);
	double h_cylinder = axisLength - h_cone;
	
	axis.normalize();
	CVector3 rotAxis = yAxis % axis;
	rotAxis.normalize();
	double rotAngle = 180 / GL_PI * acos(yAxis * axis);
	
	// set appearance
	glEnable (GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glColor4d(color.x,color.y, color.z,alpha);

	// draw the shaft of the arrow
	glPushMatrix();
		glTranslated(v1.x, v1.y, v1.z);
		glPushMatrix();
			glRotated(rotAngle,rotAxis.x,rotAxis.y,rotAxis.z);

			// make cylinder with y axis as center line
			glBegin(GL_QUAD_STRIP); 
			for (int i=0; i<=facettes; i++)
			{
				double s=sin(i*2*GL_PI/facettes);
				double c=cos(i*2*GL_PI/facettes);
				glNormal3d(c,0,s);
				glVertex3d(radius*c,0,radius*s);
				glVertex3d(radius*c, h_cylinder,radius*s);
			}
			glEnd();

		glPopMatrix();
	glPopMatrix();
	// draw the hat of the arrow
	axis.normalize();
	IPAGL::makeCone(v2, -axis * h_cone, aperture);

	glDisable(GL_BLEND);
}

//********************************************************************
// class CGLShapeShape
//********************************************************************

//! the next free frame ID; we start counting with 1 to reserve 0 for
//! special meaning
unsigned int CGLShape::nextID=1;

//********************************************************************
// class CGLShapeShape
//********************************************************************

/*! Load the position and orientation of the frame K as transformation 
 *  matrix onto the current openGL matrix stack. Further coordinates
 *  passed to OpenGL refer to this system. Make sure that the right
 *  matrix stack is selected before you call this function. On the other hand,
 *  the results may be unexpected when another frame was already
 *  loaded onto the matrix stack. For most applications you have to 
 *  save your current matrix through glPushMatrix before calling this function.
 *  \param K [in] a pointer to the reference frame that is loaded onto the matrix stack
 */
void CGLShape::MultFrame(CFrame* K)
{
	CVector3 r = K->r;
    double m[] ={K->R.e1.x, K->R.e1.y, K->R.e1.z, 0.0,
                 K->R.e2.x, K->R.e2.y, K->R.e2.z, 0.0,
                 K->R.e3.x, K->R.e3.y, K->R.e3.z, 0.0,
                 r.x,       r.y,       r.z,       1.0}; 
    glMultMatrixd(m);
}

//********************************************************************
// Klasse CGLShapeFont                                                *
//********************************************************************

int CGLShapeFont::ref_count=0;
int CGLShapeFont::start=32;          //!< the first letter
int CGLShapeFont::end=126;           //!< the last letter
int CGLShapeFont::ListStart=-1;
GLYPHMETRICSFLOAT CGLShapeFont::agmf[256]={0};

CGLShapeFont::CGLShapeFont()
{
    deviation = 0.01f;  
    extrusion = 0.1f;   
    format    = WGL_FONT_POLYGONS;          
    color[0]  = 1.0;    // color red
    color[1]  = 0.0;
    color[2]  = 0.0;

    ref_count++;
}

bool CGLShapeFont::InitFonts()
{
    HDC hdc = wglGetCurrentDC();
    CDC* pdc = CDC::FromHandle(hdc);

    // Set up the data for a font (Arial)
    LOGFONT m_logFontDefault;
    // Initialize Default Font Structure
    m_logFontDefault.lfHeight       = -18 ;
    m_logFontDefault.lfWidth        = 0 ;
    m_logFontDefault.lfEscapement   = 0 ;
    m_logFontDefault.lfOrientation  = 0 ;
    m_logFontDefault.lfWeight       = FW_NORMAL ;
    m_logFontDefault.lfItalic       = FALSE ;
    m_logFontDefault.lfUnderline    = FALSE ;
    m_logFontDefault.lfStrikeOut    = FALSE ;
    m_logFontDefault.lfCharSet      = ANSI_CHARSET ;
    m_logFontDefault.lfOutPrecision = OUT_TT_PRECIS ;
    m_logFontDefault.lfClipPrecision= CLIP_DEFAULT_PRECIS ;
    m_logFontDefault.lfQuality      = PROOF_QUALITY ;
    m_logFontDefault.lfPitchAndFamily = VARIABLE_PITCH | TMPF_TRUETYPE | FF_MODERN ;
    _tcsncpy(m_logFontDefault.lfFaceName, _T("Arial"),6) ;  // Escaping UNICODE
    // Create Default Font
    CFont m_pFontSelected;
    m_pFontSelected.CreateFontIndirect(&m_logFontDefault) ;
    CFont* pOldFont = (CFont*)pdc->SelectObject(&m_pFontSelected);

    // Generate a display list from the font
    ListStart=glGenLists(2*(end-start));
    wglUseFontOutlines(hdc,start,end,ListStart,deviation,extrusion,format,agmf);
    wglUseFontBitmaps(hdc,start,end,ListStart+end-start);
	return true;
}

CGLShapeFont::~CGLShapeFont()
{
    ref_count--;
    // Dispose the reserved display lists, if there are no objects
    // of type CGLShape3DText left
    if (ref_count==0 && ListStart!=-1)
        glDeleteLists(ListStart,2*(end-start));
}

//********************************************************************
// Klasse CGLShape3DText                                              *
//********************************************************************

void CGLShape3DText::draw()
{
	if (ListStart==-1)
		InitFonts();
	double LabelShrinkRate=1;
    // Draw the text with the display list
    glPushMatrix();
        MultFrame(baseFrame);
        glColor3dv(color);
        glListBase(ListStart-start);
        glScaled(size*LabelShrinkRate,size*LabelShrinkRate,size*LabelShrinkRate);
        glCallLists(text.length(),GL_UNSIGNED_BYTE,text.c_str());
    glPopMatrix();

    // reset the orientation of faces; this seems to be a bug in
    // the display lists generated by wglUseFontOutlines
    glFrontFace(GL_CCW);        // Counter clock-wise polygons face out
}

//********************************************************************
// Klasse CGLShapeLabel                                               *
//********************************************************************

void CGLShapeLabel::draw()
{
	if (ListStart==-1)
		InitFonts();
    // Draw the text with the display list
    glColor3dv(color);
    glListBase(ListStart-start+(end-start));
    CVector3 pos = baseFrame->R*baseFrame->r;
    glRasterPos3d(pos.x,pos.y,pos.z);
    glCallLists(text.length(),GL_UNSIGNED_BYTE,text.c_str()); 

    // reset the orientation of faces; this seems to be a bug in
    // the display lists generated by wglUseFontOutlines
}

//********************************************************************
// Klasse CGLShapeSTL												 *
//********************************************************************

CGLShapeSTL::CGLShapeSTL( CFrame& frame, const string& filename, const CVector3& trans, 
                             const CMatrix3& rot, const double& s, const double& r_, const double& g_, const double& b_, const double& alpha_)
							 : color(r_,g_,b_), alpha(alpha_)
{
    translate=trans;
    rotate=rot;
    scale=s;
    baseFrame=&frame;
	drawmode=GL_TRIANGLES;
	listIndex=0;

	// load the STL file
	try {
		ifstream file(filename.c_str(),ios::binary);

		if ( !file ) 
		{
			cout << "STL-File not found\n";
		}

		char buf[80];
		file.read(buf,80);
		int anz=0;
		file.read((char*)&anz,4);
		CSTLRecord patch;
		while (anz-- && !file.eof())
		{
			file.read((char*)&patch,50);
			// cast the vertex and normal data from float (STL-patch) to double (GL-patches)
			patches.push_back(CVector3(patch.n.x,patch.n.y,patch.n.z));
			patches.push_back(CVector3(patch.a.x,patch.a.y,patch.a.z));
			patches.push_back(CVector3(patch.b.x,patch.b.y,patch.b.z));
			patches.push_back(CVector3(patch.c.x,patch.c.y,patch.c.z));
		}
	}
	catch (...)
	{ std::cout << "Error while Loading STL file: " << filename.c_str() << endl;	}
}

CGLShapeSTL::~CGLShapeSTL()
{
	// free the index of the display lists when the object is destroyed
	if (listIndex)
		glDeleteLists(listIndex,1);
}

/*! draw the STL data
 *  \todo the implementation uses openGL display lists to speed up the drawing procedure. The current implementation
 *  evaluates the scale, offset, and rotation parameters only when the list is generated during the first call; 
 *  it might be more intuitive to evaluate the parameters during every call (but this would slow down the implementation a bit).
 *  In the current implementation all the cached parameters are private and there is no way to modify them. The display lists
 *  needs revision if these variables were made accessable. */
void CGLShapeSTL::draw()
{
#ifdef ASP_ENABLE_DISPLAY_LISTS
	if (listIndex==0)
	{
		// try to create the display list
		listIndex=glGenLists(1);
		// if it worked start the list otherwise just execute the drawing program
		if (listIndex!=0)
			glNewList(listIndex,GL_COMPILE_AND_EXECUTE);
#endif ASP_ENABLE_DISPLAY_LISTS
		// perform the normal drawing operations
		//glColor3dv(&color.R);

		glEnable (GL_BLEND); 
		glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glColor4d(color.R,color.G,color.B,alpha);
		glPushMatrix();
			MultFrame(baseFrame);
			// lokale Transformation for STL Objekt erzeugen
			GLdouble dm[] ={1.0,0.0,0.0,0.0,
						   0.0,1.0,0.0,0.0,
						   0.0,0.0,1.0,0.0,
						   0.0,0.0,0.0,scale};
			for ( int j = 0 ; j < 3 ; j++ )
			{
				for ( int k = 0 ; k < 3 ; k++ )
				{
					dm[k+4*j] = rotate(k+1,j+1) ;
				}   
				dm[j+12]  = translate(j+1);
			}
			glMultMatrixd(dm);
			glBegin(drawmode);
				for (list<CVector3>::iterator itor = patches.begin(); itor!=patches.end(); ++itor)
				{
					glNormal3dv(&(*itor).x); ++itor;
					glVertex3dv(&(*itor).x); ++itor;
					glVertex3dv(&(*itor).x); ++itor;
					glVertex3dv(&(*itor).x);
				}
			glEnd();
		glPopMatrix();
		// close the display list
		if (listIndex!=0)
			glEndList();
#ifdef ASP_ENABLE_DISPLAY_LISTS
	}
	else
		// if we already have a list, we just call it
		glCallList(listIndex);
#endif // ASP_ENABLE_DISPLAY_LISTS
}

void CGLShapeSTL::drawWireframe()
{
	drawmode=GL_LINE_LOOP;
	draw();
	drawmode=GL_TRIANGLES;
}

//********************************************************************
// Klasse CGLShapeCoordinatePlanes                                               *
//********************************************************************

CGLShapeCoordinatePlanes::CGLShapeCoordinatePlanes ( CFrame& K0 )
: color_planes(0.95,0.95,0.95), color_border(0,0,0)
{
	grid=3;
	alpha=0.25;
}

void CGLShapeCoordinatePlanes::draw()
{
	// draw some decorations: coordiante planes, axis of inertia frame, ...
	glEnable (GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glBegin(GL_QUADS);
		glColor4d(color_planes.R,color_planes.G,color_planes.B,alpha);
		glVertex3d(-grid,-grid,0);
		glVertex3d( grid,-grid,0);
		glVertex3d( grid, grid,0);
		glVertex3d(-grid, grid,0);

		glVertex3d(-grid,0,-grid);
		glVertex3d( grid,0,-grid);
		glVertex3d( grid,0, grid);
		glVertex3d(-grid,0, grid);

		glVertex3d(0,-grid,-grid);
		glVertex3d(0, grid,-grid);
		glVertex3d(0, grid, grid);
		glVertex3d(0,-grid, grid);
	glEnd();
	glLineWidth(3);
	glBegin(GL_LINE_LOOP);
		glColor4d(color_border.R,color_border.G,color_border.B,alpha*2);
		glVertex3d(-grid,-grid,0);
		glVertex3d( grid,-grid,0);
		glVertex3d( grid, grid,0);
		glVertex3d(-grid, grid,0);
	glEnd();
	glBegin(GL_LINE_LOOP);
		glVertex3d(-grid,0,-grid);
		glVertex3d( grid,0,-grid);
		glVertex3d( grid,0, grid);
		glVertex3d(-grid,0, grid);
	glEnd();
	glBegin(GL_LINE_LOOP);
		glVertex3d(0,-grid,-grid);
		glVertex3d(0, grid,-grid);
		glVertex3d(0, grid, grid);
		glVertex3d(0,-grid, grid);
	glEnd();
	glLineWidth(1);
	glDisable(GL_BLEND);
}


//********************************************************************
// Klasse CGLShapeGroundPlane                                               *
//********************************************************************

CGLShapeGroundPlane::CGLShapeGroundPlane ( CFrame& K0 )
: color_planes(0.2,0.2,0.2), color_border(0,0,0)
{
	grid=2.8;
	alpha=0.25;

	corner1[0]=-2.8;
	corner1[1]=-2.8;
	corner1[2]=0;
	corner2[0]=2.8;
	corner2[1]=2.8;
	corner2[2]=0;
}

void CGLShapeGroundPlane::setGroundSize(double *corner1, double *corner2)
{
	for (int i=0;i<3;i++)
	{
		this->corner1[i]=corner1[i];
		this->corner2[i]=corner2[i];
	}
}

void CGLShapeGroundPlane::draw()
{
// draw some decorations: coordinate planes, axis of inertia frame, ...
	glEnable (GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glBegin(GL_QUADS);
		glColor4d(color_planes.R,color_planes.G,color_planes.B,alpha);
		glVertex3d( corner1[0],corner1[1],corner1[2]);
		glVertex3d( corner2[0],corner1[1],corner1[2]);
		glVertex3d( corner2[0],corner2[1],corner1[2]);
		glVertex3d( corner1[0],corner2[1],corner1[2]);
	glEnd();
	glLineWidth(3);
	glBegin(GL_LINE_LOOP);
		glColor4d(color_border.R,color_border.G,color_border.B,alpha*2);
		glVertex3d(corner1[0],corner1[1],corner1[2]);
		glVertex3d( corner2[0],corner1[1],corner1[2]);
		glVertex3d(corner2[0],corner2[1],corner1[2]);
		glVertex3d( corner1[0],corner2[1],corner1[2]);
	glEnd();
	glLineWidth(1);
	glDisable(GL_BLEND);


return;

	// draw some decorations: coordinate planes, axis of inertia frame, ...
	glEnable (GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glBegin(GL_QUADS);
		glColor4d(color_planes.R,color_planes.G,color_planes.B,alpha);
		glVertex3d(-grid,-grid,0);
		glVertex3d( grid,-grid,0);
		glVertex3d( grid, grid,0);
		glVertex3d(-grid, grid,0);
	glEnd();
	glLineWidth(3);
	glBegin(GL_LINE_LOOP);
		glColor4d(color_border.R,color_border.G,color_border.B,alpha*2);
		glVertex3d(-grid,-grid,0);
		glVertex3d( grid,-grid,0);
		glVertex3d( grid, grid,0);
		glVertex3d(-grid, grid,0);
	glEnd();
	glLineWidth(1);
	glDisable(GL_BLEND);
}


//********************************************************************
// Klasse CGLShapeManip                                              *
//********************************************************************

void CGLShapeManip::draw() 
{ 
    preDraw(); 
    wrap->draw(); 
    postDraw(); 
}

//********************************************************************
// Klasse CGLShapeTransperency                                       *
//********************************************************************

void CGLShapeTransperency::preDraw()
{
    // save the state of the attribute blending
    glPushAttrib(GL_COLOR_BUFFER_BIT);
    // enable blending to mix the colors of the new object with the background
    glEnable(GL_BLEND);
    // set blending to one by one (50%)
    glBlendFunc(GL_ONE, GL_ONE);
}

void CGLShapeTransperency::postDraw()
{
    // disable the blending when the object if finished
    glDisable(GL_BLEND);
    // restore the attribute state
    glPopAttrib();
}

//********************************************************************
// Klasse CGLShapeTransformator                                      *
//********************************************************************

CGLShapeTransformator::CGLShapeTransformator(CGLShape& shape, CVector3 r) 
: CGLShapeManip(shape)
{
    dr=r;
//    dR=MoNullState;
}

CGLShapeTransformator::CGLShapeTransformator(CGLShape& shape, CMatrix3 R) 
: CGLShapeManip(shape) 
{
//    dr=MoNullState;
    dR=R;
}

CGLShapeTransformator::CGLShapeTransformator(CGLShape& shape, CVector3 r, CMatrix3 R) 
: CGLShapeManip(shape)
{
    dr=r;
    dR=R;
}

void CGLShapeTransformator::preDraw()
{
    glPushMatrix();
    GLdouble m[] ={dR.e1.x,dR.e1.y,dR.e1.z,0.0,
                   dR.e2.x,dR.e2.y,dR.e2.z,0.0,
                   dR.e3.x,dR.e3.y,dR.e3.z,0.0,
                   dr.x,dr.y,dr.z,1.0}; 
    glMultMatrixd(m);
}

void CGLShapeTransformator::postDraw()
{
    glPopMatrix();
}

//********************************************************************
// Klasse CGLShapeMobileFrame                                        *
//********************************************************************

CGLShapeMobileFrame::CGLShapeMobileFrame(CGLShape& shape, CFrame& frame)
: CGLShapeManip(shape), dr(&frame.r), dR(&frame.R)
{}

CGLShapeMobileFrame::CGLShapeMobileFrame(CGLShape& shape, CVector3&r)
: CGLShapeManip(shape), dr(&r), dR(0)
{}

CGLShapeMobileFrame::CGLShapeMobileFrame(CGLShape& shape, CVector3&r, CMatrix3& R)
: CGLShapeManip(shape), dr(&r), dR(&R)
{}

void CGLShapeMobileFrame::preDraw()
{
    glPushMatrix();
	if (dR && dr)
	{	
	GLdouble m[] ={dR->e1.x,dR->e1.y,dR->e1.z,0.0,
               dR->e2.x,dR->e2.y,dR->e2.z,0.0,
               dR->e3.x,dR->e3.y,dR->e3.z,0.0,
               dr->x,dr->y,dr->z,1.0}; 
		glMultMatrixd(m);
	}
	else if (dR)
	{
	GLdouble m[] ={dR->e1.x,dR->e1.y,dR->e1.z,0.0,
               dR->e2.x,dR->e2.y,dR->e2.z,0.0,
               dR->e3.x,dR->e3.y,dR->e3.z,0.0,
               0,0,0,1.0}; 
		glMultMatrixd(m);
	}
	else if (dr)
	{
	GLdouble m[] ={1,0,0,0.0,
				0,1,0,0.0,
				0,0,1,0.0,
				dr->x,dr->y,dr->z,1.0}; 
		glMultMatrixd(m);
	}
}

void CGLShapeMobileFrame::postDraw()
{
    glPopMatrix();
}

//********************************************************************
// Klasse CGLStdShape                                                 *
//********************************************************************
CGLShapeBox::CGLShapeBox(const CVector3& Min_, const CVector3& Max_, const CVector3& color_, const double& alpha_)
: Min(Min_), Max(Max_), color(color_), alpha(alpha_){m_wireframe=false;}
CGLShapeBox::CGLShapeBox(const CVector3& Min_, const CVector3& Max_, double r, double g, double b)
: Min(Min_), Max(Max_) {color.x=r;color.y=g;color.z=b;alpha =1;m_wireframe=false;}

CGLShapeBox::CGLShapeBox(const CVector3& Min_, const CVector3& Max_, double r, double g, double b, bool wireframe)
: Min(Min_), Max(Max_) {color.x=r;color.y=g;color.z=b;alpha =1;m_wireframe=wireframe;}

void CGLShapeBox::draw()
{
	if(m_wireframe)
	{
		glColor4d(color.x,color.y, color.z,alpha);
		IPAGL::makeWireBox(Min.x,Max.x,Min.y,Max.x,Min.z,Max.z);
		return;
	}
	else
	{
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glColor4d(color.x,color.y, color.z,alpha);
	glBegin(GL_QUADS);
		glNormal3i( 1, 0, 0);
		glVertex3d(Max.x, Max.y, Max.z);
		glVertex3d(Max.x, Min.y, Max.z);
		glVertex3d(Max.x, Min.y, Min.z);
		glVertex3d(Max.x, Max.y, Min.z);
		glNormal3i(-1, 0, 0);
		glVertex3d(Min.x, Max.y, Max.z);
		glVertex3d(Min.x, Max.y, Min.z);
		glVertex3d(Min.x, Min.y, Min.z);
		glVertex3d(Min.x, Min.y, Max.z);
		glNormal3i( 0, 1, 0);
		glVertex3d(Max.x, Max.y, Max.z);
		glVertex3d(Max.x, Max.y, Min.z);
		glVertex3d(Min.x, Max.y, Min.z);
		glVertex3d(Min.x, Max.y, Max.z);
		glNormal3i( 0,-1, 0);
		glVertex3d(Max.x, Min.y, Max.z);
		glVertex3d(Max.x, Min.y, Min.z);
		glVertex3d(Min.x, Min.y, Min.z);
		glVertex3d(Min.x, Min.y, Max.z);
		glNormal3i( 0, 0, 1);
		glVertex3d(Max.x, Max.y, Max.z);
		glVertex3d(Max.x, Min.y, Max.z);
		glVertex3d(Min.x, Min.y, Max.z);
		glVertex3d(Min.x, Max.y, Max.z);
		glNormal3i( 0, 0,-1);
		glVertex3d(Max.x, Max.y, Min.z);
		glVertex3d(Max.x, Min.y, Min.z);
		glVertex3d(Min.x, Min.y, Min.z);
		glVertex3d(Min.x, Max.y, Min.z);
	glEnd();
	glDisable(GL_BLEND);
	}
}


CGLShapeCone::CGLShapeCone(const CVector3& Apex, const CVector3& Axis, const double& Aperture)
: apex(Apex), axis(Axis), aperture(Aperture), color(CVector3(1,0,0)), alpha(1){}

CGLShapeCone::CGLShapeCone(const CVector3& Apex, const CVector3& Axis, const double& Aperture, const CVector3& Color, const double& Alpha)
: apex(Apex), axis(Axis), aperture(Aperture), color(Color), alpha(Alpha) {}

void CGLShapeCone::draw()
{
	IPAGL::makeCone(apex,axis,aperture, color, alpha);
}

CGLShapeEllipsoid::CGLShapeEllipsoid(const CFrame& Frame, const CVector3& Radius, const CVector3& Color, const double& Alpha):frame(Frame),radius(Radius),color(Color), alpha(Alpha) {}
void CGLShapeEllipsoid::draw()
{
	glPushMatrix();
		MultFrame(&frame);

		IPAGL::makeEllipsoid(radius, color, alpha);
	// reset the matrix stack
	::glPopMatrix();
}

CGLShapeLineStrip::CGLShapeLineStrip(vector<CVector3>& lineStrip, const float& lineWidth, const int& factor, const unsigned short& pattern, const CVector3& color,const double& alpha)
: m_lineStrip(lineStrip),  m_lineWidth(lineWidth), m_factor(factor), m_pattern(pattern), m_color(color), m_alpha(alpha){}

void CGLShapeLineStrip::draw()
{
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glColor4d(m_color.x,m_color.y, m_color.z, m_alpha);

	glEnable(GL_LINE_STIPPLE);

	glLineStipple(m_factor, m_pattern);
	glLineWidth(m_lineWidth);
	glBegin(GL_LINE_STRIP);
	for (unsigned int i = 0; i < m_lineStrip.size(); i++)
	{
		glVertex3d(m_lineStrip[i].x, m_lineStrip[i].y, m_lineStrip[i].z);
	}
	glEnd();
	glLineWidth(1.0); // set default line width
	glDisable(GL_LINE_STIPPLE);
	glDisable(GL_BLEND);
}

CGLShapePointCloud::CGLShapePointCloud(vector<CVector3>& pointCloud, const float& pointSize, const CVector3& color,const double& alpha)
: m_pointCloud(pointCloud),  m_pointSize(pointSize), m_color(color), m_alpha(alpha){}

void CGLShapePointCloud::draw()
{
       glEnable(GL_BLEND);
       glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
       glColor4d(m_color.x,m_color.y, m_color.z, m_alpha);

       glEnable(GL_LINE_STIPPLE);
	   glPointSize(m_pointSize);
       glBegin(GL_POINTS);
       for (unsigned int i = 0; i < m_pointCloud.size(); i++)
       {
             glVertex3d(m_pointCloud[i].x, m_pointCloud[i].y, m_pointCloud[i].z);
       }
       glEnd();
       glLineWidth(1.0); // set default line width
       glDisable(GL_LINE_STIPPLE);
       glDisable(GL_BLEND);
}




CGLShapeVector::CGLShapeVector(const CVector3 &v1, const CVector3 &v2, const double &radius, const CVector3& color, const double& alpha)
: m_v1(v1), m_v2(v2), m_Radius(radius), m_Color(color), m_Alpha(alpha) {}


void CGLShapeVector::draw()
{
	IPAGL::makeVector(m_v1, m_v2, m_Radius, m_Color, m_Alpha);
}
//********************************************************************
// This creates a texture in OpenGL that we can texture map
//********************************************************************

void CGLShapeBitmap::CreateTexture(LPSTR strFileName, GLuint& textureID)
{
    AUX_RGBImageRec *pBitmap = NULL;
    
    if(!strFileName)                                    // Return from the function if no file name was passed in
        return;

    pBitmap = auxDIBImageLoad(strFileName);             // Load the bitmap and store the data
    
    if (pBitmap == NULL)                                 // If we can't load the file, quit!
        return;

    // Generate a texture with the associative texture ID stored in the array
    glGenTextures(1, &textureID);

    // This sets the alignment requirements for the start of each pixel row in memory.
    glPixelStorei (GL_UNPACK_ALIGNMENT, 1);

    // Bind the texture to the texture arrays index and init the texture
    glBindTexture(GL_TEXTURE_2D, textureID);

    // Build Mipmaps (builds different versions of the picture for distances - looks better)
    gluBuild2DMipmaps(GL_TEXTURE_2D, 3, pBitmap->sizeX, pBitmap->sizeY, GL_RGB, GL_UNSIGNED_BYTE, pBitmap->data);

    // Lastly, we need to tell OpenGL the quality of our texture map.  GL_LINEAR_MIPMAP_LINEAR
    // is the smoothest.  GL_LINEAR_MIPMAP_NEAREST is faster than GL_LINEAR_MIPMAP_LINEAR, 
    // but looks blochy and pixilated.  Good for slower computers though.  
        
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_NEAREST);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR_MIPMAP_LINEAR);

    // Now we need to free the bitmap data that we loaded since openGL stored it as a texture

    if (pBitmap->data)                              // If there is texture data
        free(pBitmap->data);                        // Free the texture data, we don't need it anymore

    free(pBitmap);                                  // Free the bitmap structure
}  

//********************************************************************
// Klasse CGLShape3DS                                                 *
//********************************************************************

CGLShape3DS::CGLShape3DS( const string& filename, const CVector3& trans, 
                             const CMatrix3& rot, const double& s)  
{
    translate=trans;
    rotate=rot;
    scale=s;
	
    CLoad3DS loader;
    Frame3DSObject.numOfMaterials=0;
    Frame3DSObject.numOfObjects=0;
    loader.Import3DS(&Frame3DSObject,(char*)filename.c_str());
    // Depending on how many textures we found, load each one (Assuming .BMP)
    // If you want to load other files than bitmaps, you will need to adjust CreateTexture().
    // Below, we go through all of the materials and check if they have a texture map to load.
    // Otherwise, the material just holds the color information and we don't need to load a texture.

    // Go through all the materials
    for(int i = 0; i < Frame3DSObject.numOfMaterials; i++)
    {
        // Check to see if there is a file name to load in this material
        if(strlen(Frame3DSObject.pMaterials[i].strFile) > 0)
        {
            // Use the name of the texture file to load the bitmap, with a texture ID (i).
            // We pass in our global texture array, the name of the texture, and an ID to reference it.
            CreateTexture(Frame3DSObject.pMaterials[i].strFile, textureID[Frame3DSObject.pMaterials[i].texureId]);          
        }

        // Set the texture ID for this material
        Frame3DSObject.pMaterials[i].texureId = i;
    }

	//build the object in the next free display list (but don't execute jet)
	listIndex = glGenLists(1);
	glNewList(listIndex,GL_COMPILE);

	glPushMatrix();
        
    // lokale Transformation for 3DS Objekt erzeugen
    GLdouble dm[] ={1.0,0.0,0.0,0.0,
                   0.0,1.0,0.0,0.0,
                   0.0,0.0,1.0,0.0,
                   0.0,0.0,0.0,scale};
    
    for ( int j = 0 ; j < 3 ; j++ )
    {
        for ( int k = 0 ; k < 3 ; k++ )
        {
            dm[k+4*j] = rotate(k+1,j+1) ;
        }   
        dm[j+12]  = translate(j+1);
    }
    glMultMatrixd(dm);

    // I am going to attempt to explain what is going on below up here as not to clutter the 
    // code below.  We have a model that has a certain amount of objects and textures.  We want 
    // to go through each object in the model, bind it's texture map to it, then render it.
    // To render the current object, we go through all of it's faces (Polygons).  
    // What is a face you ask?  A face is just (in this case) a triangle of the object.
    // For instance, a cube has 12 faces because each side has 2 triangles.
    // You might be thinking.  Well, if there are 12 faces in a cube, that makes
    // 36 vertices that we needed to read in for that object.  Not really true.  Because
    // a lot of the vertices are the same, since they share sides, they only need to save
    // 8 vertices, and ignore the duplicates.  Then, you have an array of all the
    // unique vertices in that object.  No 2 vertices will be the same.  This cuts down
    // on memory.  Then, another array is saved, which is the index numbers for each face,
    // which index in to that array of vertices.  That might sound silly, but it is better
    // than saving tons of duplicate vertices.  The same thing happens for UV coordinates.
    // You don't save duplicate UV coordinates, you just save the unique ones, then an array
    // that index's into them.  This might be confusing, but most 3D files use this format.
    // This loop below will stay the same for most file formats that you load, so all you need
    // to change is the loading code.  You don't need to change this loop (Except for animation).

    // Since we know how many objects our model has, go through each of them.
    for(int i = 0; i < Frame3DSObject.numOfObjects; i++)
    {
        // Make sure we have valid objects just in case. (size() is in the vector class)
        if(Frame3DSObject.pObject.size() <= 0) break;

        // Get the current object that we are displaying
        t3DObject *pObject = &Frame3DSObject.pObject[i];
            
        // Check to see if this object has a texture map, if so bind the texture to it.
        if(pObject->bHasTexture) {

            // Turn on texture mapping and turn off color
            glEnable(GL_TEXTURE_2D);

            // Reset the color to normal again
            glColor3ub(255, 255, 255);

            // Bind the texture map to the object by it's materialID
            glBindTexture(GL_TEXTURE_2D, textureID[pObject->materialID]);
        } else {

            // Turn off texture mapping and turn on color
            glDisable(GL_TEXTURE_2D);

            // Reset the color to normal again
            glColor3ub(255, 255, 255);
        }

        // This determines if we are in wireframe or normal mode
        glBegin(GL_TRIANGLES);          // Begin drawing with our selected mode (triangles or lines)

            // Go through all of the faces (polygons) of the object and draw them
            for(int j = 0; j < pObject->numOfFaces; j++)
            {
                // Go through each corner of the triangle and draw it.
                for(int whichVertex = 0; whichVertex < 3; whichVertex++)
                {
                    // Get the index for each point of the face
                    int index = pObject->pFaces[j].vertIndex[whichVertex];
            
                    // Give OpenGL the normal for this vertex.
                    glNormal3f(pObject->pNormals[ index ].x, pObject->pNormals[ index ].y, pObject->pNormals[ index ].z);
                
                    // If the object has a texture associated with it, give it a texture coordinate.
                    if(pObject->bHasTexture) {

                        // Make sure there was a UVW map applied to the object or else it won't have tex coords.
                        if(pObject->pTexVerts) {
                            glTexCoord2f(pObject->pTexVerts[ index ].x, pObject->pTexVerts[ index ].y);
                        }
                    } else {

                        // Make sure there is a valid material/color assigned to this object.
                        // You should always at least assign a material color to an object, 
                        // but just in case we want to check the size of the material list.
                        // if the size is at least one, and the material ID != -1,
                        // then we have a valid material.
                        if(Frame3DSObject.pMaterials.size() && pObject->materialID >= 0) 
                        {
                            // Get and set the color that the object is, since it must not have a texture
                            BYTE *pColor = Frame3DSObject.pMaterials[pObject->materialID].color;

                            // Assign the current color to this model
                            glColor3ub(pColor[0], pColor[1], pColor[2]);
                        }
                    }

                    // Pass in the current vertex of the object (Corner of current face)
                    glVertex3f(pObject->pVerts[ index ].x, pObject->pVerts[ index ].y, pObject->pVerts[ index ].z);
                }
            }

        glEnd();                                // End the drawing
    }
	//glDisable(GL_TEXTURE_2D);

    glPopMatrix();

	// close the display list
	glEndList();

}

void CGLShape3DS::draw()
{
	// we already have a list, so we just call it
	glCallList(listIndex); 
}

//********************************************************************
// Klasse CGLShapeTriangulatedSurface                                *
//********************************************************************

CGLShapeTriangulatedSurface::CGLShapeTriangulatedSurface()
{
	drawmode=GL_TRIANGLES;
	color.R = 0.8;
	color.G = 0.2;
	color.B = 0.2;
}

void CGLShapeTriangulatedSurface::clear()
{
	vertices.clear();
}

void CGLShapeTriangulatedSurface::addTriangle(const CVector3& a, const CVector3& b, const CVector3& c)
{
	vertices.push_back(a);
	vertices.push_back(b);
	vertices.push_back(c);
}

void CGLShapeTriangulatedSurface::draw()
{
	glBegin(drawmode);
		glColor3d(color.R,color.G,color.B);
		for (unsigned int i=0; i<vertices.size(); i+=3)
		{
			CVector3 n = (vertices[i+1]-vertices[i]) % (vertices[i+2]-vertices[i]);
			n.normalize();
			glNormal3dv(&n.x);
			glVertex3dv(&vertices[i].x);
			glVertex3dv(&vertices[i+1].x);
			glVertex3dv(&vertices[i+2].x);
		}
	glEnd();
}

//****************************************************************************
// Class CGLShapeList													     *
//****************************************************************************

CGLShapeList::CGLShapeList()
{
	pFrame = &Frame;
}

/*! delete all shapes in the content and childs container. frees the memory if 
 *  the shape was dynamically allocated. 
 */
CGLShapeList::~CGLShapeList()
{
	clear();
}

//! delete all shapes and all childs to clean up this shape list object
void CGLShapeList::clear()
{
	deleteAllShapes();
	deleteAllFrames();
}

//! delete all shapes on a given number labeled layer
void CGLShapeList::deleteLayer(unsigned int layer)
{
	// search this list for all shapes of the given layer
	for (list<CGLShape*>::iterator itor = content.begin(); itor!=content.end(); )
	{
		if ((*itor)->iLayer==layer)
		{
			if ((*itor)->bDynamicMemory)
				delete *itor;
			itor = content.erase(itor);
		}
		else 
			++itor;
	}
	// search all childs for shapes of the given layer
	for (list<CGLShapeList*>::iterator itor = childs.begin(); itor!=childs.end(); ++itor)
		(*itor)->deleteLayer(layer);
}

//! delete all shapes on a given string labeled layer
void CGLShapeList::deleteLayer(string layer)
{
	// search this list for all shapes of the given layer
	for (list<CGLShape*>::iterator itor = content.begin(); itor!=content.end(); )
	{
		if ((*itor)->sLayer.compare(layer) == 0)// check if layer strings match
		{
			if ((*itor)->bDynamicMemory)
				delete *itor;
			itor = content.erase(itor);
		}
		else 
			++itor;
	}
	// search all childs for shapes of the given layer
	for (list<CGLShapeList*>::iterator itor = childs.begin(); itor!=childs.end(); ++itor)
		(*itor)->deleteLayer(layer);
}


//! set attributes for all objects on a given layer
void CGLShapeList::setLayerAttributes(unsigned int layer, const bool bVisible)
{
	// search this list for all shapes of the given layer
	for (list<CGLShape*>::iterator itor = content.begin(); itor!=content.end(); ++itor)
	{
		if ((*itor)->iLayer==layer)
		{
			(*itor)->bVisible = bVisible;
		}	
	}
	// search all childs for shapes of the given layer
	for (list<CGLShapeList*>::iterator itor = childs.begin(); itor!=childs.end(); ++itor)
		(*itor)->setLayerAttributes(layer, bVisible);
}



//! delete the shape with the given id (with recursive search in all childs)
//! returns true if the shape of the given id was deleted
bool CGLShapeList::deleteShape(unsigned int id)
{
	// search this list for the shape with the given id
	for (list<CGLShape*>::iterator itor = content.begin(); itor!=content.end(); ++itor)
	{
		if ((*itor)->getID()==id)
		{
			if ((*itor)->bDynamicMemory)
				delete *itor;
			itor = content.erase(itor);
			return true;
		}
	}
	// search all childs for the shape with the given id
	for (list<CGLShapeList*>::iterator itor = childs.begin(); itor!=childs.end(); ++itor)
		if ((*itor)->deleteShape(id))
			return true;
	// we did find it, so return false
	return false;
}

//! delete all shapes in this list
void CGLShapeList::deleteAllShapes()
{	
	while (!content.empty())
	{
		if (content.back()->bDynamicMemory)
			if (content.back()->ID != 0 && content.back()->ID < 99999999) delete content.back();
		content.pop_back();
	}
}

//! delete the frame with the given id (with resursive search in all childs)
bool CGLShapeList::deleteFrame(unsigned int id)
{
	// search all childs for the child with the given id
	for (list<CGLShapeList*>::iterator itor = childs.begin(); itor!=childs.end(); ++itor)
	{
		if ((*itor)->getID()==id)
		{
			delete (*itor);
			childs.erase(itor);
			return true;
		}
		else
			if ((*itor)->deleteFrame(id)==true)
				return true;
	}
	return false;
}

//! delete all frames
void CGLShapeList::deleteAllFrames()
{
	while (!childs.empty())
	{
		delete childs.back();
		childs.pop_back();
	}
}

void CGLShapeList::draw()
{
	// save the matrix stack
	::glPushMatrix();

	// apply the transformation matrix of our reference frame
	MultFrame(pFrame);
	
	// draw each shape in the list
	for (list<CGLShape*>::iterator itor = content.begin(); itor!=content.end(); ++itor)
	if ((*itor)->ID <  99999999 && (*itor)->ID != 0)
		if ((*itor)->bVisible)
			(*itor)->draw();

	// call the draw function of each child list
	for (list<CGLShapeList*>::iterator itor = childs.begin(); itor!=childs.end(); ++itor)
		(*itor)->draw();
	
	// reset the matrix stack
	::glPopMatrix();
}

/*! search the frame identified by id
 *  \return a pointer to the sought shapelist object with the given frame id
 *  or a NULL pointer if such an object cannot be found in the hierarchy 
 */
CGLShapeList* CGLShapeList::getFrame(unsigned int id)
{
	// test if this class is the right one
	if (getID()==id)
		return this;
	// test if a child fits
	for (list<CGLShapeList*>::iterator itor=childs.begin(); itor!=childs.end(); ++itor)
	{
		CGLShapeList* pSL = (*itor)->getFrame(id);
		if (pSL)
			return pSL;
	}
	// nothing found, return 0 pointer
	return 0;
}

#ifdef EXTENDED_DRAWING

//********************************************************************
// Klasse CGLStdShape                                                 *
//********************************************************************

/*! draws a coordiante frame with colors axis. red represents the x-axis, green for y-axis
 *  and blue for z-axis.
 */
void CGLStdShape::makeFrame()
{
	glColor3d(0,0.8,0);
    IPAGL::makeSphere(0.01*FrameShrinkRate);
    // y-axis
    glPushMatrix();
        glPushMatrix();
            glTranslated(0.0,(0.2*FrameShrinkRate-1),0.0);
			lglColor3d(0,0.8,0);
            IPAGL::makeCylinder( 0.006*FrameShrinkRate,0.2*FrameShrinkRate );
        glPopMatrix();
        glTranslated(0.0,0.2*FrameShrinkRate,0.0);
        IPAGL::makeCone(0.2*FrameShrinkRate);
    glPopMatrix();
    // z-axis
    glPushMatrix();
        glRotated(90,1.0,0.0,0.0);
        glPushMatrix();
            glTranslated(0.0,(0.2*FrameShrinkRate-1),0.0);
			glColor3d(0,0,0.8);
            IPAGL::makeCylinder( 0.006*FrameShrinkRate,0.2*FrameShrinkRate );
        glPopMatrix();
        glTranslated(0.0,0.2*FrameShrinkRate,0.0);
        IPAGL::makeCone(0.2*FrameShrinkRate);
    glPopMatrix();
      // x-axis
    glPushMatrix();
        glRotated(-90,0.0,0.0,1.0);
        glPushMatrix();
            glTranslated(0.0,(0.2*FrameShrinkRate-1),0.0);
			glColor3d(0.8,0,0);
            IPAGL::makeCylinder( 0.006*FrameShrinkRate,0.2*FrameShrinkRate );
        glPopMatrix();
        glTranslated(0.0,0.2*FrameShrinkRate,0.0);
        IPAGL::makeCone(0.2*FrameShrinkRate);
    glPopMatrix();
}


/*! draw a revolute joint
    \todo control the numer of surfaces on the circles through IPAGL::facettes
 */
void CGLStdShape::makeJoint()
{
	GLdouble x,z,angle,x1,z1;  
		
    //  Zeichnet die Zylindermandtelflächen der äußeren Hülle des Gelenks
    glPushMatrix();
        glTranslated(0.0,0.05*JointShrinkRate-1,0.0);  //mittlere Mantel-Teil des Rotationsgelenks
		loadColor(SC_RevoluteJoint_1);
            IPAGL::makeCylinder( 0.08*JointShrinkRate,0.1*JointShrinkRate );
	    glPushMatrix();
		    glTranslated(0.0,0.1*JointShrinkRate,0.0);     //rechte Mantel-Teil des Rotationsgelenks
			loadColor(SC_RevoluteJoint_2);
		    IPAGL::makeCylinder( 0.08*JointShrinkRate,0.1*JointShrinkRate );
	    glPopMatrix();
	    glPushMatrix();
		    glTranslated(0.0,-0.1*JointShrinkRate,0.0);    //linke Mantel-Teil des Rotationsgelenks
			loadColor(SC_RevoluteJoint_2);
		    IPAGL::makeCylinder( 0.08*JointShrinkRate,0.1*JointShrinkRate );
	    glPopMatrix();
    glPopMatrix();

    glDisable(GL_CULL_FACE);
	glPolygonMode(GL_BACK,GL_LINE);  //Liniendarstellung der Rückseite

//original codes for revolute joint

/*	// Center of fan is at the origin
	glColor3d( 0.8, 0.2, 0.8 );
	glPushMatrix();
		glTranslated(0.0,-0.15*JointShrinkRate,0.0);
		glRotated(180.0,1.0,0.0,0.0);  //Fläche nur von einer Seite aus sichtbar -> rotation um 180 grad
		glBegin(GL_TRIANGLE_FAN);
			glVertex3d(0.0, 0.0,0.0);
			for(angle = 0.0; angle <= (2.01*GL_PI); angle += (GL_PI/6.0))
			{
			    // Calculate x and y position of the next vertex
				x = 0.08*JointShrinkRate*sin(angle);
				z = 0.08*JointShrinkRate*cos(angle);
	
		
			    // Specify the next vertex for the triangle fan
			    glVertex3d(x,0.0,z);
		}
		// Done drawing the fan that covers the bottom
		glEnd();
	glPopMatrix();

	glPushMatrix();
		glTranslated(0.0,0.15*JointShrinkRate,0.0);
		glBegin(GL_TRIANGLE_FAN);
			glVertex3d(0.0, 0.0,0.0);
			for(angle = 0.0; angle <= (2.01*GL_PI); angle += (GL_PI/6.0))
			{
			    // Calculate x and y position of the next vertex
				x = 0.08*JointShrinkRate*sin(angle);
				z = 0.08*JointShrinkRate*cos(angle);
			
			    // Specify the next vertex for the triangle fan
			    glVertex3d(x,0.0,z);
			}
			// Done drawing the fan that covers the bottom
		glEnd();
	glPopMatrix();
  
    // Zeichnen der inneren Achse des Gelenks
    glColor3d( 0.8, 0.8, 0.0 );
	glPushMatrix();
	    glTranslated(0.0,0.1525*JointShrinkRate-1,0.0);
	    IPAGL::makeCylinder( 0.05*JointShrinkRate,0.305*JointShrinkRate );
	glPopMatrix();

	glPushMatrix();
		glTranslated(0.0,0.1525*JointShrinkRate,0.0);
		glBegin(GL_TRIANGLE_FAN);
			glVertex3d(0.0, 0.0, 0.0);
			for(angle = 0; angle <= (2.01*GL_PI); angle += (GL_PI/6.0))
			{
			    // Calculate x and y position of the next vertex
				x = 0.05*JointShrinkRate*sin(angle);
				z = 0.05*JointShrinkRate*cos(angle);
			
			    // Specify the next vertex for the triangle fan
			    glVertex3d(x,0.0,z);
			}
			// Done drawing the fan that covers the bottom
		glEnd();
	glPopMatrix();

	glPushMatrix();
		glTranslated(0.0,-0.1525*JointShrinkRate,0.0);
		glRotated(180.0,1.0,0.0,0.0);
		glBegin(GL_TRIANGLE_FAN);
			glVertex3d(0.0, 0.0, 0.0);
			for(angle = 0; angle <= (2.01*GL_PI); angle += (GL_PI/6.0))
			{
			    // Calculate x and y position of the next vertex
				x = 0.05*JointShrinkRate*sin(angle);
				z = 0.05*JointShrinkRate*cos(angle);
			
			    // Specify the next vertex for the triangle fan
			    glVertex3d(x,0.0,z);
			}
			// Done drawing the fan that covers the bottom
		glEnd();
	glPopMatrix();
*/

// new code for revolute joint;  Yan

	
	glPushMatrix();
		glTranslated(0.0,-0.15*JointShrinkRate,0.0);
		glRotated(180.0,1.0,0.0,0.0);  //Fläche nur von einer Seite aus sichtbar -> rotation um 180 grad
		loadColor(SC_RevoluteJoint_2);
		glBegin(GL_QUAD_STRIP);
			
			for(angle = 0.0; angle <= (2.01*GL_PI); angle += (GL_PI/6.0))
			{
			    // Calculate x and z position of the next two vertices
				x = 0.08*JointShrinkRate*sin(angle);
				z = 0.08*JointShrinkRate*cos(angle);
				x1= 0.05*JointShrinkRate*sin(angle);
				z1= 0.05*JointShrinkRate*cos(angle);
		
			    // Specify the next two vertices for the QUAD_STRIP
			    glVertex3d(x1,0.0,z1);
				glVertex3d(x,0.0,z);
			}
		
		glEnd();
	glPopMatrix();

	glPushMatrix();
		glTranslated(0.0,0.15*JointShrinkRate,0.0);
		glBegin(GL_QUAD_STRIP);
			
			for(angle = 0.0; angle <= (2.01*GL_PI); angle += (GL_PI/6.0))
			{
			    // Calculate x and y position of the next vertex
				x = 0.08*JointShrinkRate*sin(angle);
				z = 0.08*JointShrinkRate*cos(angle);
				x1= 0.05*JointShrinkRate*sin(angle);
				z1= 0.05*JointShrinkRate*cos(angle);
		
			    // Specify the next vertex for the triangle fan
			    glVertex3d(x1,0.0,z1);
				glVertex3d(x,0.0,z);
			}
			// Done drawing the fan that covers the bottom
		glEnd();
	glPopMatrix();

    // Zeichnen der inneren Achse des Gelenks
	loadColor(SC_RevoluteJoint_1);
    
	glPushMatrix();
		glTranslated(0.0,0.15*JointShrinkRate,0.0);
		glBegin(GL_TRIANGLE_FAN);
			glVertex3d(0.0, 0.0, 0.0);
			for(angle = 0; angle <= (2.01*GL_PI); angle += (GL_PI/6.0))
			{
			    // Calculate x and z position of the next vertex
				x = 0.05*JointShrinkRate*sin(angle);
				z = 0.05*JointShrinkRate*cos(angle);
			
			    // Specify the next vertex for the triangle fan
			    glVertex3d(x,0.0,z);
			}
			// Done drawing the fan that covers the bottom
		glEnd();
	glPopMatrix();

	glPushMatrix();
		glTranslated(0.0,-0.15*JointShrinkRate,0.0);
		glRotated(180.0,1.0,0.0,0.0);
		glBegin(GL_TRIANGLE_FAN);
			glVertex3d(0.0, 0.0, 0.0);
			for(angle = 0; angle <= (2.01*GL_PI); angle += (GL_PI/6.0))
			{
			    // Calculate x and y position of the next vertex
				x = 0.05*JointShrinkRate*sin(angle);
				z = 0.05*JointShrinkRate*cos(angle);
			
			    // Specify the next vertex for the triangle fan
			    glVertex3d(x,0.0,z);
			}
			// Done drawing the fan that covers the bottom
		glEnd();
	glPopMatrix();
}

//! draws a prismatic joint
void CGLStdShape::makePrismaticJoint(double l)
{
    glPushMatrix();
		loadColor(SC_PrismaticJoint_1);
        IPAGL::makeBox( 0.12*JointShrinkRate, 0.12*JointShrinkRate, 0.3*JointShrinkRate);
		loadColor(SC_PrismaticJoint_2);
        glTranslated(0.0, 0.0, l/2),
        IPAGL::makeBox( 0.04*JointShrinkRate, 0.04*JointShrinkRate, l);
    glPopMatrix(); 
}

void CGLStdShape::makeLink(double l,double fac, MoShape which_shape)
{
	loadColor(SC_RigidLink);
    glPushMatrix();
        if(which_shape==JOINT_LINK)
        {
            glTranslated(0.0,l-1,0.0);
            IPAGL::makeCylinder( 0.02*LinkShrinkRate,(l-fac*(2*0.08*JointShrinkRate)));
        }
        else
        {
            glTranslated(0.0,l-1,0.0);
            IPAGL::makeCylinder( 0.02*LinkShrinkRate,l );
        }
    glPopMatrix();
}

void CGLStdShape::makeWireLink(double l,double fac, MoShape which_shape)
{
	loadColor(SC_RigidLink);

    glPushMatrix();
        if(which_shape==JOINT_LINK)
        {
            glTranslated(0.0,l-1,0.0);
            IPAGL::makeWireCylinder( 0.02*LinkShrinkRate,(l-fac*(2*0.08*JointShrinkRate)));
        }
        else
        {
            IPAGL::makeWireCylinder( 0.02*LinkShrinkRate,l );
        }
    glPopMatrix();
}

void CGLStdShape::makeRL()
{
	loadColor(SC_RigidLink);

    ::glPushMatrix();
        ::glTranslated(0.1*JointShrinkRate,2*0.08*JointShrinkRate-1,0);
        IPAGL::makeCylinder( 0.02*LinkShrinkRate,1.2*(0.08*JointShrinkRate));
    ::glPopMatrix();
    ::glPushMatrix();
        ::glTranslated(0.1*JointShrinkRate,2*0.08*JointShrinkRate,0);
        IPAGL::makeSphere( 0.02*LinkShrinkRate);
    ::glPopMatrix();
    ::glPushMatrix();
        ::glTranslated(-0.1*JointShrinkRate,2*0.08*JointShrinkRate-1,0);
        IPAGL::makeCylinder( 0.02*LinkShrinkRate,1.2*(0.08*JointShrinkRate));
    ::glPopMatrix();
    ::glPushMatrix();
        ::glTranslated(-0.1*JointShrinkRate,2*0.08*JointShrinkRate,0);
        IPAGL::makeSphere( 0.02*LinkShrinkRate);
    ::glPopMatrix();
    ::glPushMatrix();
        ::glRotated(90,0,0,1);
        ::glTranslated(2*0.08*JointShrinkRate,0.1*JointShrinkRate-1,0);
        IPAGL::makeCylinder( 0.02*LinkShrinkRate,2*0.1*JointShrinkRate);
    ::glPopMatrix();
}


/*void CGLStdShape::makeWireRL()
{
  ::glColor3d( 0.8, 0.8, 0.8 );

    ::glPushMatrix();
        ::glTranslated(0.1*JointShrinkRate,2*0.08*JointShrinkRate-1,0);
        IPAGL::makeWireCylinder( 0.02*LinkShrinkRate,1.2*(0.08*JointShrinkRate));
    ::glPopMatrix();
    ::glPushMatrix();
        ::glTranslated(0.1*JointShrinkRate,2*0.08*JointShrinkRate,0);
        IPAGL::makeWireSphere( 0.02*LinkShrinkRate);
    ::glPopMatrix();
    ::glPushMatrix();
        ::glTranslated(-0.1*JointShrinkRate,2*0.08*JointShrinkRate-1,0);
        IPAGL::makeWireCylinder( 0.02*LinkShrinkRate,1.2*(0.08*JointShrinkRate));
    ::glPopMatrix();
    ::glPushMatrix();
        ::glTranslated(-0.1*JointShrinkRate,2*0.08*JointShrinkRate,0);
        IPAGL::makeWireSphere( 0.02*LinkShrinkRate);
    ::glPopMatrix();
    ::glPushMatrix();
        ::glRotated(90,0,0,1);
        ::glTranslated(2*0.08*JointShrinkRate,0.1*JointShrinkRate-1,0);
        IPAGL::makeWireCylinder( 0.02*LinkShrinkRate,2*0.1*JointShrinkRate);
    ::glPopMatrix();
}
*/

//********************************************************************
// Klasse CGLStdShape                                                 *
//********************************************************************

CGLStdShape::CGLStdShape( CFrame& frame)  
{
    baseFrame=&frame;
    ShapeType=FRAME;
}

void CGLStdShape::makeSpherePatch(CVector3 &v1, CVector3 &v2, CVector3 &v3, int depth)
{
    if (depth==0)
    {
        glBegin(GL_TRIANGLES);
            glNormal3dv((double*)&v1);
            glVertex3dv((double*)&(v1*radius*JointShrinkRate));
            glNormal3dv((double*)&v2);
            glVertex3dv((double*)&(v2*radius*JointShrinkRate));
            glNormal3dv((double*)&v3);
            glVertex3dv((double*)&(v3*radius*JointShrinkRate));
        glEnd();
    }
    else
    {
        depth--;
        CVector3 v12=(v1+v2);v12.normalize();
        CVector3 v23=(v2+v3);v23.normalize();
        CVector3 v31=(v1+v3);v31.normalize();
        makeSpherePatch(v1,v12,v31,depth);
        makeSpherePatch(v2,v23,v12,depth);
        makeSpherePatch(v3,v31,v23,depth);
        makeSpherePatch(v12,v23,v31,depth);
    }
}

void CGLStdShape::draw()
{
    double zAng,xAng,yAng,l;
    CVector3 r_temp;
    glPushMatrix();
    MultFrame(baseFrame);   
    switch (ShapeType)
    {
    case FRAME:
            makeFrame();    
        break;
    
    case R_JOINT:
                switch (JointAxis)
                {
                case xAxis: glRotated(-90,0.0,0.0,1.0); break;
                case yAxis: break;
                case zAxis: glRotated(90,1.0,0.0,0.0); break;
                }
                makeJoint();
        break;

    case P_JOINT:
                switch (JointAxis)
                {
                case xAxis: 
                    l=(topFrame->r.x)-(baseFrame->r.x);
                    glRotated(90,0.0,1.0,0.0);
                    break;
                case yAxis: 
                    l=(topFrame->r.y)-(baseFrame->r.y);
                    glRotated(-90,1.0,0.0,0.0);
                    break;
                case zAxis: 
                    l=(topFrame->r.z)-(baseFrame->r.z);
                    break;
                }
                makePrismaticJoint(l);
        break;
    
    case LINK:        
            if (*m_referenceTo == HEAD)
                r_temp=(*m_R)*(*vector);
            else
                r_temp=*vector;

            zAng=atan2(r_temp.x,r_temp.y);
            xAng=atan2(r_temp.z,sqrt(r_temp.x*r_temp.x+r_temp.y*r_temp.y));
            l=sqrt(r_temp*r_temp);

            glRotated(-zAng*RAD_TO_DEG,0,0,1);
            glRotated(xAng*RAD_TO_DEG,1,0,0);
            makeLink(l, 1, LINK);
        break;

    case JOINT_LINK:
            ::glPushMatrix();
                switch (JointAxis)
                {
                case xAxis: ::glRotated(-90,0.0,0.0,1.0); break;
                case yAxis: break;
                case zAxis: ::glRotated(90,1.0,0.0,0.0); break;
                }
                makeJoint();
            ::glPopMatrix();

            if (*m_referenceTo == HEAD)
                r_temp=(*m_R)*(*vector);
            else
                r_temp=*vector;
                switch (JointAxis)
                {
                case xAxis:
                    xAng=atan2(r_temp.z, r_temp.y);
                    zAng=atan2(r_temp.x, sqrt(pow(r_temp.z,2)+pow(r_temp.y,2)) );
                    ::glRotated(xAng*RAD_TO_DEG,1,0,0);
                    l=sqrt(r_temp*r_temp);
                    if ( (fabs(zAng)>0.6) && (fabs(zAng)<2.5))// if 35<Ang<145 ; einfaches Link wird gezeichnet
                    {
//                        ::glRotated(-zAng*RAD_TO_DEG,0,0,1);
                        makeLink(l,1,LINK);
                    }
                    else
                    {
                        makeRL();
//                        ::glRotated(-zAng*RAD_TO_DEG,0,0,1);
                        makeLink(l,1/cos(zAng),JOINT_LINK);
                    }
                    break;
                case yAxis:
                    yAng=atan2(r_temp.z, r_temp.x);
                    zAng=atan2(r_temp.y, sqrt(pow(r_temp.z,2)+pow(r_temp.x,2)) );
                    ::glRotated(-yAng*RAD_TO_DEG,0,1,0);
                    ::glRotated(-90,0,0,1);
                    l=sqrt(r_temp*r_temp);
                    if ( (fabs(zAng)>0.6) && (fabs(zAng)<2.5))// if 35<Ang<145 ; einfaches Link wird gezeichnet
                    {
//                        ::glRotated(zAng*RAD_TO_DEG,0,0,1);
                        makeLink(l,1,LINK);
                    }
                    else
                    {
                        makeRL();
//                        ::glRotated(zAng*RAD_TO_DEG,0,0,1);
                        makeLink(l,1/cos(zAng),JOINT_LINK);
                    }
                    break;
                case zAxis:
                    zAng=atan2(r_temp.x, r_temp.y);
                    xAng=atan2(r_temp.z, sqrt(pow(r_temp.x,2)+pow(r_temp.y,2)) );
                    ::glRotated(-zAng*RAD_TO_DEG,0,0,1);
                    l=sqrt(r_temp*r_temp);
                    if ( (fabs(xAng)>0.6) && (fabs(xAng)<2.5))// if 35<Ang<145 ; einfaches Link wird gezeichnet
                    {
//                        ::glRotated(xAng*RAD_TO_DEG,1,0,0);
                        makeLink(l,1,LINK);
                    }
                    else
                    {
                        ::glPushMatrix();
//                            ::glRotated(90,0,1,0);
                            makeRL();
                        ::glPopMatrix();
//                        ::glRotated(xAng*RAD_TO_DEG,1,0,0);
                        makeLink(l,1/cos(xAng),JOINT_LINK);
                    }
                    break;
                }
        break;
    case S_JOINT:   
		loadColor(SC_SphericalJoint);
        IPAGL::makeSphere(radius*JointShrinkRate);
        break;
    case MASSELEMENT:
        glPushMatrix();
            if( vector != 0 )
                ::glTranslated( vector->x, vector->y, vector->z );
			loadColor(SC_MassElement_1);
            makeSpherePatch(CVector3( 1,0,0),CVector3(0, 1,0),CVector3(0,0, 1),3);
            makeSpherePatch(CVector3( 1,0,0),CVector3(0,-1,0),CVector3(0,0,-1),3);
            makeSpherePatch(CVector3(-1,0,0),CVector3(0,-1,0),CVector3(0,0, 1),3);
            makeSpherePatch(CVector3(-1,0,0),CVector3(0, 1,0),CVector3(0,0,-1),3);
			loadColor(SC_MassElement_2);
            makeSpherePatch(CVector3( 1,0,0),CVector3(0,0, 1),CVector3(0,-1,0),3);
            makeSpherePatch(CVector3( 1,0,0),CVector3(0,0,-1),CVector3(0, 1,0),3);
            makeSpherePatch(CVector3(-1,0,0),CVector3(0,0, 1),CVector3(0, 1,0),3);
            makeSpherePatch(CVector3(-1,0,0),CVector3(0,0,-1),CVector3(0,-1,0),3);
        glPopMatrix();
        break;
    }
    glPopMatrix();
}

void CGLStdShape::drawWireframe()
{
    double zAng,xAng,l;//,yAng
    CVector3 r_temp;
//	int i;
//	double x,z,angle;
    glPushMatrix();
	glDisable(GL_LIGHTING);     //Lichteffekte werden ausgeschalten
    MultFrame(baseFrame);   
    switch (ShapeType)
    {
    case FRAME:                  
            makeWireFrame();    
        break;
    
    case R_JOINT:              
        switch (JointAxis)
                {
                case xAxis: ::glRotated(-90,0.0,0.0,1.0); break;
                case yAxis: break;
                case zAxis: ::glRotated(90,1.0,0.0,0.0); break;
                }
 
	    glPushMatrix();
	      glTranslated(0.0,-0.05*JointShrinkRate,0.0);
		    glPushMatrix();
			loadColor(SC_RevoluteJoint_1);
			    IPAGL::makeWireCylinder( 0.08*JointShrinkRate,0.1*JointShrinkRate );

			    glPushMatrix();
			    glTranslated(0.0,0.1*JointShrinkRate,0.0);
			loadColor(SC_RevoluteJoint_2);
			    IPAGL::makeWireCylinder( 0.08*JointShrinkRate,0.1*JointShrinkRate );
			    glPopMatrix();

			    glPushMatrix();
			    glTranslated(0.0,-0.1*JointShrinkRate,0.0);
			    IPAGL::makeWireCylinder( 0.08*JointShrinkRate,0.1*JointShrinkRate );
			    glPopMatrix();
		    glPopMatrix();

		    // Zeichnen der inneren Achse des Gelenks
		    glPushMatrix();
			loadColor(SC_RevoluteJoint_1);
			    glTranslated(0.0,-0.10*JointShrinkRate,0.0);
			IPAGL::makeWireCylinder( 0.05*JointShrinkRate,0.10*JointShrinkRate );
		    glPopMatrix();
		glPushMatrix();
			glTranslated(0.0, 0.10*JointShrinkRate,0.0);
			IPAGL::makeWireCylinder( 0.05*JointShrinkRate,0.10*JointShrinkRate );
		glPopMatrix();

	    glPopMatrix();
        break;			

    case P_JOINT:      
        switch (JointAxis)
        {
        case xAxis: 
            l=(topFrame->r.x)-(baseFrame->r.x);
            ::glRotated(90,0.0,1.0,0.0);
            break;
        case yAxis: 
            l=(topFrame->r.y)-(baseFrame->r.y);
            ::glRotated(-90,1.0,0.0,0.0);
            break;
        case zAxis: 
            l=(topFrame->r.z)-(baseFrame->r.z);
            break;
        }
        
        glPushMatrix();
				loadColor(SC_PrismaticJoint_1);
        IPAGL::makeWireBox( 0.12*JointShrinkRate, 0.12*JointShrinkRate, 0.3*JointShrinkRate);
				loadColor(SC_PrismaticJoint_2);
        if (l<=(0.15*JointShrinkRate))  // add edges on the intersection surface of Prismatic Joint, Yan
        {
            glTranslated(0.0, 0.0, l/2),
                IPAGL::makeWireBox( 0.04*JointShrinkRate, 0.04*JointShrinkRate, l);
        }
        else
        {
            glTranslated(0.0,0.0,0.075*JointShrinkRate);
            IPAGL::makeWireBox(0.04*JointShrinkRate, 0.04*JointShrinkRate, 0.15*JointShrinkRate);
            glTranslated(0.0,0.0,l/2);
            IPAGL::makeWireBox(0.04*JointShrinkRate, 0.04*JointShrinkRate, l-0.15*JointShrinkRate);
            
        }
        glPopMatrix(); 
        break;

    case LINK:                               //in Bearbeitung
        if (*m_referenceTo == HEAD)
            r_temp=(*m_R)*(*vector);
        else
            r_temp=*vector;
        
        zAng=atan2(r_temp.x,r_temp.y);
        xAng=atan2(r_temp.z,sqrt(r_temp.x*r_temp.x+r_temp.y*r_temp.y));
        l=sqrt(r_temp*r_temp);
        
            glRotated(-zAng*RAD_TO_DEG,0,0,1);
            glRotated(xAng*RAD_TO_DEG,1,0,0);
        makeWireLink(l, 1, LINK);  //makeWireLink ist in Bearbeitung
        break;


    case S_JOINT:                        
		loadColor(SC_SphericalJoint);
        IPAGL::makeWireSphere(radius*JointShrinkRate);
        break;
    case MASSELEMENT:                    //noch nicht bearbeitet
        glPushMatrix();
            if( vector != 0 )
                ::glTranslated( vector->x, vector->y, vector->z );
			loadColor(SC_MassElement_3);
        
            glPushMatrix();										//5. Achtel
            IPAGL::makeWireEighthSphere(radius*JointShrinkRate);
            glPopMatrix();
        
            glPushMatrix();										//6. Achtel
            glRotated(180,0.0,0.0,1.0);
            IPAGL::makeWireEighthSphere(radius*JointShrinkRate);
            glPopMatrix();
        
            glPushMatrix();										//7. Achtel
            glRotated(180,0.0,1.0,0.0);
            IPAGL::makeWireEighthSphere(radius*JointShrinkRate);
            glPopMatrix();
        
            glPushMatrix();										//8. Achtel
            glRotated(180,1.0,0.0,0.0);
            IPAGL::makeWireEighthSphere(radius*JointShrinkRate);
            glPopMatrix();
        
			loadColor(SC_MassElement_1);
            glPushMatrix();
            glRotated(90,1.0,0.0,0.0);
            glRotated(90,0.0,1.0,0.0);
            glRotated(90,0.0,0.0,1.0);
            IPAGL::makeWireEighthSphere(radius*JointShrinkRate); //1. Achtel
        
            glPushMatrix();										//2. Achtel
            glRotated(180,0.0,0.0,1.0);
            IPAGL::makeWireEighthSphere(radius*JointShrinkRate);
            glPopMatrix();
        
            glPushMatrix();										//3. Achtel
            glRotated(180,0.0,1.0,0.0);
            IPAGL::makeWireEighthSphere(radius*JointShrinkRate);
            glPopMatrix();
        
            glPushMatrix();										//4. Achtel
            glRotated(180,1.0,0.0,0.0);
            IPAGL::makeWireEighthSphere(radius*JointShrinkRate);
            glPopMatrix();
        
            glPopMatrix();	
        glPopMatrix();

    break;
    }
	glEnable(GL_LIGHTING);
    glPopMatrix();
}

//********************************************************************
// Klasse CGLShapeArrow                                              *
//********************************************************************

CGLShapeArrow::CGLShapeArrow(CFrame& frame, CVector3& Vec, const double Scale, bool DirectonToFrame)
: scale(Scale), toFrame(DirectonToFrame), v(&Vec)
{
	baseFrame = &frame;
}

void CGLShapeArrow::draw()
{
	if (length(*v)<1e-100) return;
    glPushMatrix();
        CVector3 start,end;
        start = baseFrame->R * baseFrame->r;
        end = start + baseFrame->R * *v * scale;
        CVector3 r_temp=end-start;

        double zAng=atan2(r_temp.x,r_temp.y);
        double xAng=atan2(r_temp.z,sqrt(r_temp.x*r_temp.x+r_temp.y*r_temp.y));
        double l=sqrt(r_temp*r_temp);
        
        glTranslated(start.x,start.y,start.z);
        glRotated(-zAng*RAD_TO_DEG,0,0,1);
        glRotated(xAng*RAD_TO_DEG,1,0,0);
        glTranslated(0.0,l-1,0.0);
        IPAGL::makeCylinder( 0.02*FrameShrinkRate*scale,l );
		glTranslated(0.0,1,0.0);
		IPAGL::makeCone(FrameShrinkRate*scale);
    glPopMatrix();
}

//********************************************************************
// Klasse CGLShapePrimitive                                           *
//********************************************************************

CGLShapePrimitive::CGLShapePrimitive(CFrame& BaseFrame, MoShape Type, double SizeX, 
                                   double SizeY, double SizeZ) 
                                   : sizex(SizeX), sizey(SizeY), sizez(SizeZ)
{
    baseFrame = &BaseFrame;
    topFrame = &BaseFrame;
    ShapeType = Type;
	nColor = SC_ShapePrimitive;
}

//! \todo add internal color management
//! \todo replace auxSolidSphere with internal functions
void CGLShapePrimitive::draw()
{
    int i;
	glPushMatrix();
		MultFrame(baseFrame);
		loadColor(nColor);
		switch (ShapeType)
		{
		case SPHERE:
			IPAGL::makeSphere(sizex*FrameShrinkRate);
			break;
		case FLOOR:
			glBegin(GL_QUADS);
				glNormal3d(0.0,0.0,1.0);
				glVertex3d(-sizex/2,-sizex/2,0.0);
				glVertex3d( sizex/2,-sizex/2,0.0);
				glVertex3d( sizex/2, sizex/2,0.0);
				glVertex3d(-sizex/2, sizex/2,0.0);
			glEnd();
            break;
        case CYLINDER:
            glBegin(GL_TRIANGLE_FAN);
                glNormal3i(0,0,-1);
                glVertex3d(0,0,0);
                for (i=0; i<=IPAGL::facettes; i++)
                    glVertex3d(sizex*cos(i*2*GL_PI/IPAGL::facettes),sizey*sin(i*2*GL_PI/IPAGL::facettes),0);
            glEnd();
            glBegin(GL_TRIANGLE_FAN);
                glNormal3i(0,0,1);
                glVertex3d(0,0,sizez);
                for (i=IPAGL::facettes; i>=0; i--)
                    glVertex3d(sizex*cos(i*2*GL_PI/IPAGL::facettes),sizey*sin(i*2*GL_PI/IPAGL::facettes),sizez);
            glEnd();
            glBegin(GL_QUAD_STRIP);
                for (i=0; i<=IPAGL::facettes; i++)
                {
                    glNormal3d(sizex*cos(i*2*GL_PI/IPAGL::facettes),sizey*sin(i*2*GL_PI/IPAGL::facettes),0);
                    glVertex3d(sizex*cos(i*2*GL_PI/IPAGL::facettes),sizey*sin(i*2*GL_PI/IPAGL::facettes),0);
                    glVertex3d(sizex*cos(i*2*GL_PI/IPAGL::facettes),sizey*sin(i*2*GL_PI/IPAGL::facettes),sizez);
                }
            glEnd();
            break;
        case TEAPOT:
            auxSolidTeapot(sizex*FrameShrinkRate);
            break;
        case ARROW:
            // x-axis
            glPushMatrix();
                glScaled(sizex,sizex,sizex);
                glRotated(-90,0.0,0.0,1.0);
                glPushMatrix();
                    glTranslated(0.0,(0.2*FrameShrinkRate-1),0.0);
                    glColor3d( 0.8, 0.0, 0.0 );
                    IPAGL::makeCylinder( 0.006*FrameShrinkRate,0.2*FrameShrinkRate );
                glPopMatrix();
                glTranslated(0.0,0.2*FrameShrinkRate,0.0);
                IPAGL::makeCone(0.2*FrameShrinkRate);
            glPopMatrix();
            break;
        case CUBE:
			glColor3d(1,0.7,0.2);	//color for rectangle element
            IPAGL::makeBox(sizex,sizey,sizez);
            break;
		}
	glPopMatrix();
}





//********************************************************************
// Klasse CGLShapeRectangleTexture                                    *
//********************************************************************

/*! Create a rectangle that is filled with a texture. By default the rectangle is drawn 
 *  with the frame K is its plain. The plain may be moved through the setOffset function.
 *  \param K [in] the rectangle is atteched to this frame
 *  \param filename [in] openGL loads the texture from the BMP file. The loader was tested
 *         for 24bit color files.
 *  \param Normal [in] the normal vector for the rectangle. Default is z-Axis.
 *  \param sizex [in] the length of the rectangle in the first direction. If normal is z-Axis, this is
 *         the length in x-direction. Default is 1.0
 *  \param sizex [in] the width of the rectangle in the second direction. If normal is z-Axis, this is
 *         the width in y-direction.  Default is 1.0
 */
CGLShapeRectangleTexture::CGLShapeRectangleTexture( CFrame& K, const string& filename, MoAxis Normal, double sizex, 
												 double sizey) : normal(Normal), SizeX(sizex), SizeY(sizey)

{
	baseFrame = &K;
	topFrame = &K;
    ScaleU=1.0;
    ScaleV=1.0;
    normaloffset=0.0;
	CreateTexture((char*)filename.c_str(),materialID);
}

void CGLShapeRectangleTexture::draw()
{
	glPushMatrix();
		MultFrame(baseFrame);		
		glEnable(GL_TEXTURE_2D);

		// Reset the color to normal again
		glColor3ub(255, 255, 255);

		// Bind the texture map to the object by it's materialID
	 	glBindTexture(GL_TEXTURE_2D, materialID);

		glBegin(GL_QUADS);
        switch (normal)
        {
        case xAxis:
		    glNormal3i(1,0,0);
			glTexCoord2f(0.0,0.0); glVertex3f(normaloffset,-SizeX/2,-SizeY/2);
			glTexCoord2f(ScaleU,0.0); glVertex3f(normaloffset, SizeX/2,-SizeY/2);
			glTexCoord2f(ScaleU,ScaleV); glVertex3f(normaloffset, SizeX/2, SizeY/2);
			glTexCoord2f(0.0,ScaleV); glVertex3f(normaloffset,-SizeX/2, SizeY/2);
            break;
        case yAxis:
		    glNormal3i(0,1,0);
			glTexCoord2f(0.0,0.0); glVertex3f(-SizeX/2,normaloffset,-SizeY/2);
			glTexCoord2f(ScaleU,0.0); glVertex3f( SizeX/2,normaloffset,-SizeY/2);
			glTexCoord2f(ScaleU,ScaleV); glVertex3f( SizeX/2,normaloffset, SizeY/2);
			glTexCoord2f(0.0,ScaleV); glVertex3f(-SizeX/2,normaloffset, SizeY/2);
            break;
        case zAxis:
		    glNormal3i(0,0,1);
			glTexCoord2f(0.0,0.0); glVertex3f(-SizeX/2,-SizeY/2,normaloffset);
			glTexCoord2f(ScaleU,0.0); glVertex3f( SizeX/2,-SizeY/2,normaloffset);
			glTexCoord2f(ScaleU,ScaleV); glVertex3f( SizeX/2, SizeY/2,normaloffset);
			glTexCoord2f(0.0,ScaleV); glVertex3f(-SizeX/2, SizeY/2,normaloffset);
            break;
        }
		glEnd();
		glDisable(GL_TEXTURE_2D);
	glPopMatrix();
}


#endif // EXTENDED_DRAWING

void CGLShapeVrmlIndexedFace::draw()
{
	// draw whatever is in our data model; we will later extend the implementation by display lists to speed up the procedure
	unsigned int i=0;
	glBegin(GL_POLYGON);
	// compute the normal for the first face
	if (i+3<index.size())
	{
		CVector3 normal = (pCoords->vertices[index[i+1]]-pCoords->vertices[index[i]])%(pCoords->vertices[index[i+2]]-pCoords->vertices[index[i]]);
		glNormal3dv(&normal.x);					
	}
	while (i<index.size())
	{
		// paranoia testing
		if (index[i]>=(int)pCoords->vertices.size())
			break;
		if (index[i]==-1)
		{
			glEnd();
			i++;
			if (i<index.size())
			{
				glBegin(GL_POLYGON);
				// compute the normal for the next face
				if (i+3<(int)index.size())
				{
					CVector3 normal = (pCoords->vertices[index[i+1]]-pCoords->vertices[index[i]])%(pCoords->vertices[index[i+2]]-pCoords->vertices[index[i]]);
					glNormal3dv(&normal.x);					
				}
			}
			else
				break;
		}
		// paranoia testing
		if (index[i]>=(int)pCoords->vertices.size())
			break;
		glVertex3dv(&pCoords->vertices[index[i]].x);
		i++;
	}
	// one more paranoia call
	glEnd();
	return;
}

bool CGLShapeVrmlIndexedFace::read(ifstream& file)
{
//	printf("Parsing IndexedFaceSet\n");
	string token;
	int i;
	while (!file.eof() && token!="]" && token!="coordIndex")
		file >> token;
	if (token=="coordIndex")
		file >> token;	// extract the "["
	if (token!="[")
		printf("Warning: no '[' after coordIndex found\n");
	while (!file.eof() && token!="]")
	{
		file >> i >> token;
		if (!file.fail() && token==",")		// successfully read the vector
		{
			if (i > (int)pCoords->vertices.size())
			{
				printf("Warning: Vertex ID exceeds the size of the related Coordinate3 structure\n");
				i=0;
			}
			index.push_back(i);	// store the vector in the data model
			continue;
		}
		if (token==",")
		{
			file.clear();
			// we found a ',' without data; check if reached the end
			if (file.peek()==']')
			{
				file >> token;	// eat the closing bracket and leave the while loop
				break;
			}
		}
		// we simple go forward to the next comma ","
		file.ignore(-1,',');
		// check if we reached the end
		if (file.peek()==']')
			break;
	}

	return true;
}

CGLShapeVRML::CGLShapeVRML( CFrame& frame, const string& filename, const CVector3& trans, 
                             const CMatrix3& rot, const double& s, const double& r_, const double& g_, const double& b_, const double& alpha_)
							 : CGLShapeSTL(frame,filename,trans,rot,s,r_,g_,b_,alpha_)
{
	// load the VRML file; due to the structure of the program we make a call the STL loader which is not intended
	try {
		// call the old parser
		// ReadVRMLFile(filename);

		// call the new parser
		ifstream file(filename);
		parseVrml(file,false);
	}
	catch (...)
	{ std::cout << "Error while loading VRML file: " << filename.c_str() << endl;	}
}

CGLShapeVRML::~CGLShapeVRML()
{
	// we have to clean up the memory
	for (unsigned int i=0; i<coords.size(); i++)
		delete coords[i];
	for (unsigned int i=0; i<drawList.size(); i++)
		delete drawList[i];
}

bool stringToDouble(char t[],float &reel)
{
	// we could use atof instead
	if (1==sscanf(t,"%f",&reel))
		return true;
	else 
		return false;
}

//! helper function to extract a color value from an VRML file
bool getVRMLColor(ifstream& file, CGLRGBColor& Color)
{
	string token;
	CGLRGBColor color;
	file >> ws;				// eat whitespace
	if (file.peek()=='[')
		file >> token;		// eat the token
	file >> color.R >> color.G >> color.B;
	if (file.fail())
		return false;
	Color = color;
	file >> ws;				// eat whitespace
	if (file.peek()==']')
		file >> token;		// eat the token
	return true;
}

//! helper function to extract a double value from an VRML file
bool getVRMLDouble(ifstream& file, double& value)
{
	string token;
	double temp;
	file >> ws;				// eat whitespace
	if (file.peek()=='[')
		file >> token;		// eat the token
	file >> temp;
	if (file.fail())
		return false;
	value = temp;
	file >> ws;				// eat whitespace
	if (file.peek()==']')
		file >> token;		// eat the token
	return true;
}


bool CGLShapeVrmlMaterial::read(ifstream& file)
{
//	printf("Parsing Material\n");
	bool run=true;
	string token;
	while (run)
	{
		file >> token;
		if (token=="{")
			continue;
		if (token[0]=='#')
			file.ignore(2048);
		if (token=="ambientColor")
		{
			if (getVRMLColor(file,ambientColor))
				continue;
			printf("Error: Cannot read color data\n");
		}
		if (token=="diffuseColor")
		{
			if (getVRMLColor(file,diffuseColor))
				continue;
			printf("Error: Cannot read color data\n");
		}
		if (token=="specularColor")
		{
			if (getVRMLColor(file,specularColor))
				continue;
			printf("Error: Cannot read color data\n");
		}
		if (token=="emissiveColor")
		{
			if (getVRMLColor(file,emissiveColor))
				continue;
			printf("Error: Cannot read color data\n");
		}
		if (token=="shininess")
		{
			if (getVRMLDouble(file,shininess))
				continue;
			printf("Error: Cannot read color data\n");
		}
		if (token=="transparency")
		{
			if (getVRMLDouble(file,transparency))
				continue;
			printf("Error: Cannot read color data\n");
		}
		if (token=="}")
			return true;
		printf("Warning: Unknown token found in material\n");
	}
	// we simple go forward to the next braket "}"
	file.ignore(-1,'}');
	return false;
}

//! perform a "draw" operation for the material, i.e. set the material 
//! properties without really drawing something on the screen. Anyway
//! the following draw operations will use the color settings defined here.
void CGLShapeVrmlMaterial::draw()
{
	glColor3dv(&diffuseColor.R);
	float mat[3];
	mat[0] = (float)ambientColor.R;
	mat[1] = (float)ambientColor.G;
	mat[2] = (float)ambientColor.B;
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat);

	mat[0] = (float)specularColor.R;
	mat[1] = (float)specularColor.G;
	mat[2] = (float)specularColor.B;
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat);

	mat[0] = (float)diffuseColor.R;
	mat[1] = (float)diffuseColor.G;
	mat[2] = (float)diffuseColor.B;
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat);

	glMaterialf(GL_FRONT, GL_SHININESS, (float)shininess*128.0f);
}

//! read the subnote from an VRML file
bool CGLShapeVrmlCoordinate3::read(ifstream& file)
{
//	printf("Parsing Coordinate3\n");
	string token;
	double x,y,z;
	while (!file.eof() && token!="]" && token!="point")
		file >> token;
	if (token=="point")
		file >> token;	// extract the "["
	if (token!="[")
		printf("Warning: no '[' after point found\n");
	while (!file.eof() && token!="]")
	{
		file >> x >> y >> z >> token;
		if (!file.fail() && (token=="," || token=="]"))		// successfully read the vector
		{
			vertices.push_back(CVector3(x,y,z));	// store the vector in the data model
			continue;
		}
		if (token==",")
		{
			file.clear();
			// we found a ',' without data; check if reached the end
			if (file.peek()==']')
			{
				file >> token;	// eat the closing bracket and leave the while loop
				break;
			}
		}
		// we simple go forward to the next comma ","
		file.ignore(-1,',');
		// check if we reached the end
		if (file.peek()==']')
			break;
	}
	return true;
}

/*! new vrml parser by asp/321
 *  \param file the stream to process
 *  \return true, if successful otherwise false
 */
bool CGLShapeVRML::parseVrml(ifstream& file, bool seekEndOfBlock)
{
	bool good=true;
	string token;
	char buffer[2048];
	if ( !file ) 
	{
		cout << "VRML-File not found\n";
		return true; 
	}

	while (!file.eof() && good)
	{
		// extract a token
		file >> token;

		// identify the token and process the token
		
		// empty token?
		if (token.length()==0)
		{
			good=false;
			continue;
		}
		
		// comment --> ignore everying till the end of line
		if (token[0]=='#')
		{
			file.getline(buffer,2048);
			continue;
		}

		if (token[0]=='}' && seekEndOfBlock)
			return true;

		// check for a DEF statement
		if (token=="DEF")
		{
			file >> token;	// extrect the name after DEF and ignore it
			continue;
		}

		// Separator: make a recursive call 
		if (token=="Separator")
		{
			file >> token;
			if (token != "{")
			{
				good = false;
				continue;
			}
			parseVrml(file,true);
		}

		// now we search for useful data nodes
		if (token=="Material")
		{
			CGLShapeVrmlMaterial* pMaterial = new CGLShapeVrmlMaterial;
			if (pMaterial->read(file))
				drawList.push_back(pMaterial);
			else
				delete pMaterial;
			continue;
		}

		if (token=="Coordinate3")
		{
			CGLShapeVrmlCoordinate3* pCoord = new CGLShapeVrmlCoordinate3;
			if (pCoord->read(file))
				coords.push_back(pCoord);
			else
				delete pCoord;

			continue;
		}
		if (token=="IndexedFaceSet")
		{
			if (coords.size()==0)
			{
				printf("Critical Error in file: found an IndexedFaceSet without previous definition of Coordinate3\n");
				return false;
			}
			CGLShapeVrmlIndexedFace* pFace = new CGLShapeVrmlIndexedFace(*coords.back());
			if (pFace->read(file))
				drawList.push_back(pFace);
			else
				delete pFace;

			continue;
		}

		// Unknown token
		file >> ws;					// eat possible whitespace and check if a block opens
		if (file.peek()=='{')
		{
			printf("unknown block %s. we try to parse whatever we find inside\n",token.c_str());
			file >> token;			// eat the { and call the parser for the block
			parseVrml(file,true);	// try to get whatever is in the block and hopefully we return after the block ended
		}
	}
	return good;
}

/*! vrml file reading contributed by Matthias Palzkill, 2012
 *  integrated into IPAGL by asp/321
 *  We only search for triangles inside the file. We do also not care about
 *  colors in the file. Thus, there is pleanty of room for improved future versions.
 *  \param filenameVRML name of the VRML v1 file
 */
void CGLShapeVRML::ReadVRMLFile(std::string filenameVRML) //, TRANSFORMATION& trafoMZP_L_V)
{
	// VRML-Datei einlesen
	FILE *file;                                                                                                                                         // VRML-file
	fopen_s(&file, filenameVRML.c_str(), "r");        // open the VRML-file

	if (file == 0) 
	{
		cout << "VRML-File not found\n";
		return; // \todo: print an error message or indicate an error
	}
	char buffer[1024];
	bool out = false;
	while(!out)                                                       // looking for "point" or "point["
	{
		fscanf_s(file, "%s", buffer, 1024);
		if (strcmp(buffer, "point") ==0 )
		{
			fscanf_s(file, "%s", buffer, 1024);
			out = true;
		}
		if (strcmp(buffer, "point[") == 0)
			out = true;
	}

	out = false;
	float x, y, z;
	while(!out)                                                       // reading the point-coordinates
	{
		fscanf_s(file, "%s", buffer, 1024);

		if (buffer[0] == ']')
			out = true;
		else
		{
			// read another triplet of vertex data
			stringToDouble(buffer, x);
			fscanf_s(file, "%s", buffer, 1024); 
			stringToDouble(buffer, y);
			fscanf_s(file, "%s", buffer, 1024); 
			stringToDouble(buffer, z);
			// save one vertices (and implicitly its index)
			vertices.push_back(CVector3(x,y,z));
		}
	}

	out=false;
	while(!out)                                                       // looking for "coordIndex" or "coordIndex["
	{
		fscanf_s(file, "%s", buffer, 1024);
		if (strcmp(buffer,"coordIndex")==0)
		{
			fscanf_s(file, "%s", buffer, 1024);
			out=true;
		}
		if (strcmp(buffer,"coordIndex[")==0)   out=true;
	}

	out=false;
	float f;
	int i;
	while(!out)                                                       // reading the polygones
	{
		fscanf_s(file, "%s", buffer, 1024);

		if (buffer[0]==']')
			out=true;
		else
		{
			// read three indices
			stringToDouble(buffer, f);                                             // erster Punkt
			triangles.push_back((int)f);

			fscanf_s(file, "%s", buffer, 1024);                                           // zweiter Punkt
			stringToDouble(buffer, f);
			triangles.push_back((int)f);

			fscanf_s(file, "%s", buffer, 1024);                                           // dritter Punkt
			stringToDouble(buffer, f);
			triangles.push_back((int)f);

			fscanf_s(file, "%s", buffer, 1024);                                           // "-1" oder vierter Punkt
			stringToDouble(buffer, f);
			i = (int)f;

			if (i != -1)
			{
				// no idea what this block should do
//				p4 = &(m_PointsWorld.at(i));
//				CADTriangle triangle2(p1, p3, p4);
//				m_Triangles.push_back(triangle2);
//				index[1] = index[2]; index[2] = i;
//				std::vector<int> vi2(index, index + sizeof(index)/sizeof(int));
//				m_TrianglesIndices.push_back(vi2);
				fscanf_s(file, "%s", buffer, 1024);                                           // "-1"
			}
		}
	}

	fclose(file);                                                       // close the VRML-file
}

//! this is an adoption from the STL draw routine.
void CGLShapeVRML::draw()
{
#ifdef ASP_ENABLE_DISPLAY_LISTS
	if (listIndex==0)
	{	// try to create the display list
		listIndex=glGenLists(1);
		// if it worked start the list otherwise just execute the drawing program
		if (listIndex!=0)
			glNewList(listIndex,GL_COMPILE_AND_EXECUTE);
#endif //ASP_ENABLE_DISPLAY_LISTS

		// perform the normal drawing operations
		glColor3dv(&color.R);

		glEnable (GL_BLEND); 
		glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glColor4d(color.R,color.G,color.B,alpha);
		glPushMatrix();
			MultFrame(baseFrame);
			// lokale Transformation for STL Objekt erzeugen
			GLdouble dm[] ={1.0,0.0,0.0,0.0,
						   0.0,1.0,0.0,0.0,
						   0.0,0.0,1.0,0.0,
						   0.0,0.0,0.0,scale};
			for ( int j = 0 ; j < 3 ; j++ )
			{
				for ( int k = 0 ; k < 3 ; k++ )
				{
					dm[k+4*j] = rotate(k+1,j+1) ;
				}   
				dm[j+12]  = translate(j+1);
			}
			glMultMatrixd(dm);
			
			// call all draw functions of the embedded VRML structures
			for (unsigned int i=0; i<drawList.size(); i++)
				drawList[i]->draw();

		glPopMatrix();
		// close the display list
		if (listIndex!=0)
			glEndList();
#ifdef ASP_ENABLE_DISPLAY_LISTS
	}
	else
		// if we already have a list, we just call it
		glCallList(listIndex);
#endif // ASP_ENABLE_DISPLAY_LISTS
}