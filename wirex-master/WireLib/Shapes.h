/*
* WireX  -  WireLib
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

/*! \file Shapes.h
 *
 *	\author   Andreas Pott
 *
 *  \dependency
 *		openGL		for visualization of graphical aspects of parallel cable robots
 *      IPAGL		as drawing framework
 *
 *  \par brief
 *  Implementation graphical representations cable robots and their properties such
 *  as workspace. The implementation of the openGL code is fully inline. In this way
 *  the binary file of the library does not depend on openGL and one can add the 
 *  draw functions just by additionally including this header.
 * 
 *  This file is therefore understood to be a supplement of wirelib, i.e. wirelib
 *  runs without this implementation but this file adds quite some help when using
 *  wirelib with an openGL capable application (such as WireCenter).
 */

#pragma once

#include <string>
#include <vector>
#include <IPAGL/GLShape.h>
#include "Workspace.h"
#include "Kinematics.h"
#include "Interference.h"
#include "PoseList.h"
#include "PosePropertyEvaluation.h"

namespace PCRL {

//! InternalShapeIDs to differ the internal scene objects names given by the OpenGL feedback engine
typedef enum InternalShapeIDs {
	isIdRobotFrame=100,
	isIdWorldFrame,
	isIdWires,
	isIdHull,
	isIdWinchesLabels,
	isIdWinches,
	isIdRobotWorldFrame,
	isIdBaseFrames,
	isIdFrameBoundingbox,
	isIdGravityArrow
} eIPAGLFeedbackShapeIDs;

/*! The drawing functions for displaying the workspace hull 
 */
class CShapeWorkspaceHull : public CGLShape
{
	CWorkspaceHull* pHull;
	CGLRGBColor color;

public:
	explicit CShapeWorkspaceHull(CWorkspaceHull& hull)
		: pHull(&hull),color(1,0,0)
	{
			bCenterofInertiaVisible=false;
			bWorkspaceParallelProjectionVisible=false;
			bPhongShading=false;
			bbMin=Vector3d(0,0,0);
			bbMax=Vector3d(0,0,0);
	}

	//! set the color of the workspace hull
	bool setHullColor(double r, double g, double b)
	{
		color.R=r;
		color.G=g;
		color.B=b;
		return true;
	}
	
	//! bool, if the center of inertia is drawn
	bool bCenterofInertiaVisible;
	//! bool, if the parallel projection of the workspace is drawn
	bool bWorkspaceParallelProjectionVisible;
	//! smooth the surface
	bool bPhongShading;
	//! local variables to store the frame boundingbox values
	Vector3d bbMin,bbMax;
	
	//! the draw function for the hull
	void draw()
	{
		glPushName(isIdHull);
		glPushName(0); //dummy who will be overwritten with glLoadName

		for (int i=0; i<pHull->FinishedTriangles; i++)
		{
			glColor3d(color.R,color.G,color.B);
			glLoadName(i);
			if (bPhongShading)
			{
				Vector3d normal;
				glBegin(GL_TRIANGLES);
					normal = pHull->vertices[pHull->Triangles[i].i];
					glNormal3dv(&normal.x());
					glVertex3dv(&pHull->vertices[pHull->Triangles[i].i].x());

					normal = pHull->vertices[pHull->Triangles[i].j];
					glNormal3dv(&normal.x());
					glVertex3dv(&pHull->vertices[pHull->Triangles[i].j].x());

					normal = pHull->vertices[pHull->Triangles[i].k];
					glNormal3dv(&normal.x());
					glVertex3dv(&pHull->vertices[pHull->Triangles[i].k].x());
				glEnd();
			}
			else
			{
				Vector3d normal=(pHull->vertices[pHull->Triangles[i].i]-pHull->vertices[pHull->Triangles[i].j]).cross(pHull->vertices[pHull->Triangles[i].i]-pHull->vertices[pHull->Triangles[i].k]);
				glBegin(GL_TRIANGLES);
					glNormal3dv(&normal.x());
					glVertex3dv(&pHull->vertices[pHull->Triangles[i].i].x());
					glVertex3dv(&pHull->vertices[pHull->Triangles[i].j].x());
					glVertex3dv(&pHull->vertices[pHull->Triangles[i].k].x());
				glEnd();
			}
		}

		glPopName(); //pop the vertex number 
		glPopName(); //pop the hull indexing number

		//draw the Center of Inertia
		if (bCenterofInertiaVisible)
		{
			//save all current attributes (slow but effective)
			glPushAttrib( GL_ALL_ATTRIB_BITS );
			
			//temporarily disable the depth test
			glDisable(GL_DEPTH_TEST);
			
			//a temporary vector
			GLdouble vec[3];
			
			//set line width, stipple pattern and color
			glEnable(GL_LINE_STIPPLE);
			glColor3d(0.0,0.0,1.0);
			glLineWidth(3.0);
			glLineStipple(1,0x180);

			//draw the intersecting lines
			glBegin(GL_LINES);
				vec[0] = pHull->bbMin.x(); vec[1] = pHull->CoI.y() ; vec[2] = pHull->CoI.z();	
				glVertex3dv(&vec[0]);
				vec[0]= pHull->bbMax.x();
				glVertex3dv(&vec[0]);
				
				vec[0] = pHull->CoI.x(); vec[1] = pHull->bbMin.y() ; vec[2] = pHull->CoI.z();	
				glVertex3dv(&vec[0]);
				vec[1]= pHull->bbMax.y();
				glVertex3dv(&vec[0]);
			
				vec[0] = pHull->CoI.x(); vec[1] = pHull->CoI.y() ; vec[2] = pHull->bbMin.z();	
				glVertex3dv(&vec[0]);
				vec[2]= pHull->bbMax.z();
				glVertex3dv(&vec[0]);
			glEnd();
			
			//set block color, size and alter polygon mode 
			GLdouble size = 0.05;
			glColor3d(0.0,1.0,0.0);
			
			//draw a box
			glPushMatrix();
				glTranslated(pHull->CoI.x(),pHull->CoI.y(),pHull->CoI.z());
				IPAGL::makeBox(size,size,size);
			glPopMatrix();

			//retrieve old attributes (slow but effective)
			glPopAttrib();
		}
		
		//draw the parallel projections of the hull object
		if (bWorkspaceParallelProjectionVisible)
		{
			//save all current attributes (slow but effective)
			glPushAttrib( GL_ALL_ATTRIB_BITS );

			//disable lightning
			glDisable(GL_LIGHTING);

			//set opacity of the colors
			GLdouble color_opacity = 0.6;

			//draw object again (but without naming and without normal vector)
			for (int i=0; i<pHull->FinishedTriangles; i++)
			{
				glBegin(GL_TRIANGLES);
					
					//DRAW THE x-PLANE PROJECTION
					glColor3d(color_opacity,color_opacity,1.0);
					glVertex3d(pHull->vertices[pHull->Triangles[i].i].x(),pHull->vertices[pHull->Triangles[i].i].y(),bbMin.z());
					glVertex3d(pHull->vertices[pHull->Triangles[i].j].x(),pHull->vertices[pHull->Triangles[i].j].y(),bbMin.z());
					glVertex3d(pHull->vertices[pHull->Triangles[i].k].x(),pHull->vertices[pHull->Triangles[i].k].y(),bbMin.z());
					
					//DRAW THE y-PLANE PROJECTION
					glColor3d(0.5,1.0,0.5);
					glVertex3d(pHull->vertices[pHull->Triangles[i].i].x(),bbMin.y(),pHull->vertices[pHull->Triangles[i].i].z());
					glVertex3d(pHull->vertices[pHull->Triangles[i].j].x(),bbMin.y(),pHull->vertices[pHull->Triangles[i].j].z());
					glVertex3d(pHull->vertices[pHull->Triangles[i].k].x(),bbMin.y(),pHull->vertices[pHull->Triangles[i].k].z());
					
					//DRAW THE z-PLANE PROJECTION
					glColor3d(1.0,color_opacity,color_opacity);
					glVertex3d(bbMin.x(),pHull->vertices[pHull->Triangles[i].i].y(),pHull->vertices[pHull->Triangles[i].i].z());
					glVertex3d(bbMin.x(),pHull->vertices[pHull->Triangles[i].j].y(),pHull->vertices[pHull->Triangles[i].j].z());
					glVertex3d(bbMin.x(),pHull->vertices[pHull->Triangles[i].k].y(),pHull->vertices[pHull->Triangles[i].k].z());

				glEnd();
			}

			//recover old attributes
			glPopAttrib();
		}

		if (true) // m_bOrientationSetVisible
		{
			std::vector<Matrix3d> ori = pHull->getAllOrientations();
			for (unsigned int i=0; i<ori.size(); i++)
			{
				double beta,ux,uy,uz;
				glPushMatrix();
				Matrix3d R_base = ori[i];
				PCRL::getAxisAngleFromMatrix(beta, ux, uy, uz, R_base);
//				glTranslated(pRobot->getBase(i).x(),pRobot->getBase(i).y(),pRobot->getBase(i).z());
				glRotated((beta/GL_PI)*180,ux, uy, uz);
				IPAGL::makeFrame();
				glPopMatrix();
			}
		}
	}
};

/*! visualization function for the differential workspace. Show the difference 
 *  between the determined workspace as colors.
 *  \todo Add a flag to control visibility of the two main functions (hull and
 *        arrows.
 */
class CShapeWorkspaceDiffHull : public CGLShape
{
	CWorkspaceDifferentialHull* pHull;
	CGLRGBColor colorMin;
	CGLRGBColor colorMax;
public:
	int DifferenceSelector;		//!< selects which of the calculated hulls is shown by the draw function; 0 <=selector<m*6
	explicit CShapeWorkspaceDiffHull(CWorkspaceDifferentialHull& hull)
		: pHull(&hull), colorMin(0,0,1), colorMax(1,0,0), DifferenceSelector(0) {}

	//! the draw function for the hull
	void draw()
	{
		if (pHull->pDiffHull[DifferenceSelector]->vertices.size()==0)
			return;
		double Min= 1e100;
		double Max=-1e100;
		double span;
		// calculate the range of divations between the nominal workspace and the differential workspace
		// this is needed for the coloring and appropriate scaling of the vector arrows
		for (unsigned int i=0; i<pHull->vertices.size(); i++)
		{
			Min = min(pHull->pDiffHull[DifferenceSelector]->vertices[i].norm() - pHull->vertices[i].norm(), Min);
			Max = max(pHull->pDiffHull[DifferenceSelector]->vertices[i].norm() - pHull->vertices[i].norm(), Max);
		}
		span = 2*max(fabs(Min),fabs(Max));
#ifndef ASP_OUT
		// draw the differential hull and indicate differential changes by coloring
		glBegin(GL_TRIANGLES);
		for (int i=0; i<pHull->FinishedTriangles; i++)
		{
			Vector3d normal=(pHull->vertices[pHull->Triangles[i].i]-pHull->vertices[pHull->Triangles[i].j]).cross(pHull->vertices[pHull->Triangles[i].i]-pHull->vertices[pHull->Triangles[i].k]);
			glNormal3dv(&normal.x());
			// calculate the factor c for blending between the min and max color
			// set the color between colorMin and colorMax; then draw the nominal vertex
			double c = (pHull->pDiffHull[DifferenceSelector]->vertices[pHull->Triangles[i].i].norm() - pHull->vertices[pHull->Triangles[i].i].norm())/span +0.5;
			glColor3d(colorMin.R+(colorMax.R-colorMin.R)*c,colorMin.G+(colorMax.G-colorMin.G)*c,colorMin.B+(colorMax.B-colorMin.B)*c);
			glVertex3dv(&pHull->vertices[pHull->Triangles[i].i].x());
			c = (pHull->pDiffHull[DifferenceSelector]->vertices[pHull->Triangles[i].j].norm() - pHull->vertices[pHull->Triangles[i].j].norm())/span+0.5;
			glColor3d(colorMin.R+(colorMax.R-colorMin.R)*c,colorMin.G+(colorMax.G-colorMin.G)*c,colorMin.B+(colorMax.B-colorMin.B)*c);
			glVertex3dv(&pHull->vertices[pHull->Triangles[i].j].x());
			c = (pHull->pDiffHull[DifferenceSelector]->vertices[pHull->Triangles[i].k].norm() - pHull->vertices[pHull->Triangles[i].k].norm())/span+0.5;
			glColor3d(colorMin.R+(colorMax.R-colorMin.R)*c,colorMin.G+(colorMax.G-colorMin.G)*c,colorMin.B+(colorMax.B-colorMin.B)*c);
			glVertex3dv(&pHull->vertices[pHull->Triangles[i].k].x());

		}
		glEnd();
#endif // ASP_OUT

		// we place arrows on the surface showing the derivatives instead of coloring the surface
		for (unsigned int i=0; i<pHull->vertices.size(); i++)
		{
			Vector3d u = pHull->vertices[i] - pHull->center; u.normalize();
			u *= (pHull->pDiffHull[DifferenceSelector]->vertices[i].norm() - pHull->vertices[i].norm()) / Max;

			double c = ((pHull->pDiffHull[DifferenceSelector]->vertices[i].norm() - pHull->vertices[i].norm())/span)+0.5;
			
			// compute end point
			Vector3d V2 = pHull->vertices[i]+u;

			CVector3 v1,v2;
			v1.x = pHull->vertices[i].x();
			v1.y = pHull->vertices[i].y();
			v1.z = pHull->vertices[i].z();

			v2.x = V2.x();	// this lacks the required scaling of arrows
			v2.y = V2.y();	// this lacks the required scaling of arrows
			v2.z = V2.z();	// this lacks the required scaling of arrows

			IPAGL::makeVector(v1,v2,
				0.01,	// radius
				CVector3(colorMin.R+(colorMax.R-colorMin.R)*c,colorMin.G+(colorMax.G-colorMin.G)*c,colorMin.B+(colorMax.B-colorMin.B)*c), // color
				1);
		}
	}
};

/*! drawing function for the robot shape including frame, platform and wires
 */
class CShapeRobot : public CGLShape
{
	CRobotData *pRobot;
	Matrix3d* pR;
	Vector3d* pr;
	MatrixXd* pl;
public:
	CGLRGBColor color_winches;
	CGLRGBColor color_platform;
	std::vector<CGLRGBColor> color_wires; // a dynamic vector of colors for all wires
	double winch_size;
	bool bWorldFrameVisible;
	bool bWinchesVisible;
	bool bWinchesLabelVisible;
	bool bPlatformVisible;
	bool bPlatformBBVisible;
	bool bFrameBBVisible;
	bool bFrameBBFancyFrame;
	bool bBaseFramesVisible;
	bool bWiresVisible;
	bool bGravityArrowVisible;
	int iHighlightedFrameDimension;
	
	CFrame* Winch_frame;
	CGLShapeLabel* pLabel;

public:
	CShapeRobot(CRobotData& robot, Vector3d& r, Matrix3d& R, MatrixXd* l)  // CRobotData does not contain r,R, so it has to be provided separately
		: pRobot(&robot), pr(&r), pR(&R), pl(l),
		color_winches(0.8,0.8,0.8), 
		color_platform(0.2,0.8,0.2), 
		winch_size(0.05)
	{
		bWorldFrameVisible=true;
		bWinchesVisible=true;
		bPlatformVisible=true;
		bPlatformBBVisible=true;
		bFrameBBFancyFrame=true;
		bFrameBBVisible=true;
		bWiresVisible=true;
		bBaseFramesVisible = false;
		string sLabel;
		pLabel= new CGLShapeLabel(*Winch_frame,sLabel,12,0,1,1);  // definition of font size and color of the winch labels
		//resize the color_wires array to the number of wires
		color_wires.resize(pRobot->getNow());
		iHighlightedFrameDimension=0;
	}

	~CShapeRobot(){delete pLabel;}

	//! draw the robot frame and the cables
	void draw()
	{
		
		// draw the world coordinate system
		if (bWorldFrameVisible)
		{
			glPushName(isIdWorldFrame);
			IPAGL::makeFrame();
			glPopName();
		}
		
		// draw the winches
		if (bWinchesVisible)
		{
			glColor3dv(&color_winches.R);
			glPushName(isIdWinches);
			for (int i=0; i<pRobot->getNow(); i++)
			{
				glLoadName(i);
				glPushMatrix();
				glTranslated(pRobot->getBase(i).x(),pRobot->getBase(i).y(),pRobot->getBase(i).z());
				IPAGL::makeBox(winch_size,winch_size,winch_size);
				glPopMatrix();
			}
		}

		// draw the labels of the winches
		if (bWinchesLabelVisible)
		{
			glPushName(isIdWinchesLabels);
			for (int i=0; i<pRobot->getNow(); i++)
			{
				
				char buffer[20];
				sprintf_s(buffer," W %i",i+1);
				Winch_frame=new CFrame(CVector3(pRobot->getBase(i).x(),pRobot->getBase(i).y(),pRobot->getBase(i).z()+0.3),CMatrix3(1,0,0,0,1,0,0,0,1));
				string sLabel=buffer;
			    
				pLabel->setbaseFrame(*Winch_frame);
				pLabel->setText(sLabel);
				pLabel->draw();
				delete Winch_frame;
			}
			glPopName();
		}

		// draw the orientation of the winches for each frame

		// draw the platform
		if (bPlatformVisible)
		{
			glLineWidth(3);
			CVector3 r;
			r.x = pr->x();
			r.y = pr->y();
			r.z = pr->z();
			CMatrix3 R(pR->col(0).x(),pR->col(0).y(),pR->col(0).z(),
				pR->col(1).x(),pR->col(1).y(),pR->col(1).z(),
				pR->col(2).x(),pR->col(2).y(),pR->col(2).z());
			glPushMatrix();
			glTranslated(r.x,r.y,r.z);

				glPushName(isIdRobotWorldFrame);
				IPAGL::makeFrame(iHighlightedFrameDimension);
				glPopName();

			glPushName(isIdRobotFrame);
			glPushName(0);
			for (int i=0; i<pRobot->getNow(); i++)
			{
				CVector3 platform;
				platform.x=pRobot->getPlatform(i).x();
				platform.y=pRobot->getPlatform(i).y();
				platform.z=pRobot->getPlatform(i).z();
				platform=R*platform;
				glLoadName(i);
				
				//if selected, draw it in orange color else use the standard color
				if (iHighlightedFrameDimension==4) glColor3d(1.0,0.5,0.0);
				else glColor3dv(&color_platform.R);

				glBegin(GL_LINES);
					glVertex3d(0,0,0);
					glVertex3d(platform.x,platform.y,platform.z);
				glEnd();

				//if selected, draw an additional thick red wireframe
				/*if (iHighlightedFrameDimension==4) 
				{
					//save state
					glPushAttrib( GL_ALL_ATTRIB_BITS );

					//set an polygon offset to -3.0 (because the line width will be 6, and lines are placed centered)
					// -> setting GL_POLYGON_OFFSET_FILL attribute will manipulate the depth-buffer, so as a result only the 
					//	  OUTLINING lines will be visible whereas all other edges are NOT highlighted
					glEnable( GL_POLYGON_OFFSET_FILL );
					glPolygonOffset( -3.0f , -3.0f );
					
					//draw in normal fill mode
					glPushMatrix();
						glTranslated(platform.x,platform.y,platform.z);
						CGLShape::MultFrame(&CFrame(CVector3(0,0,0),R));
						IPAGL::makeBox(winch_size,winch_size,winch_size);
					glPopMatrix();
					
					//set GL-Engine in polygon mode
					glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
					
					//set line width and color
					glLineWidth(6);
					glColor3d(1.0,0.0,0.0);
					
					//draw the boxes in wireframe - mode
					glPushMatrix();
						glTranslated(platform.x,platform.y,platform.z);
						CGLShape::MultFrame(&CFrame(CVector3(0,0,0),R));
						IPAGL::makeBox(winch_size,winch_size,winch_size);
					glPopMatrix();
					
					//recover old state
					glPopAttrib();
				}
				else
				{
					//draw in normal fill mode
					glPushMatrix();
						glTranslated(platform.x,platform.y,platform.z);
						CGLShape::MultFrame(&CFrame(CVector3(0,0,0),R));
						IPAGL::makeBox(winch_size,winch_size,winch_size);
					glPopMatrix();
				}*/

				glPushMatrix();
					glTranslated(platform.x,platform.y,platform.z);
					CGLShape::MultFrame(&CFrame(CVector3(0,0,0),R));
					IPAGL::makeBox(winch_size,winch_size,winch_size);
				glPopMatrix();
			}
			glPopName();
			glPopName();
			glPopMatrix();
		}

		// draw the gravity arrow
		if (bGravityArrowVisible)
		{
			glPushName(isIdGravityArrow);
			double FrameShrinkRate=1;
			glColor3d(0,0.8,0);
			glLineWidth(5);
			CVector3 r;
			r.x = pr->x();
			r.y = pr->y();
			r.z = pr->z();

			glPushMatrix();
				glRotated(-90,1.0,0.0,0.0);
				glPushMatrix(); 
					glTranslated(r.x,-r.z+(0.2*FrameShrinkRate-1),r.y);
					glColor3d(0,0,0);
					IPAGL::makeCylinder( 0.006*FrameShrinkRate,0.2*FrameShrinkRate );
				glPopMatrix();
				glPushMatrix();
					glTranslated(r.x,-r.z+0.2*FrameShrinkRate,r.y);
					IPAGL::makeCone(0.2*FrameShrinkRate);
				glPopMatrix();
			glPopMatrix();
			glPopName();
		}

		// draw the wires
		if (bWiresVisible)
		{
			glPushName(isIdWires);
			
			Vector3d* pB = new Vector3d[pRobot->getNow()];
			Vector3d* pC = new Vector3d[pRobot->getNow()];

			for (int i=0; i<pRobot->getNow(); i++)
			{
				pB[i] = *pr + *pR * pRobot->getPlatform(i);
			}

			// Visualize robot model with fixed a_i
			if (pRobot->getRobotKinematicsModel() == CRobotData::FIXED)
			{
				for (int i=0; i<pRobot->getNow(); i++)
				{
					pC[i]= pRobot->getBase(i);
				}
			}
			else if (pRobot->getRobotKinematicsModel() == CRobotData::PULLEY)
			{
				CElastoKinematics kinematics(*pRobot);
				double* pBeta = new double[pRobot->getNow()];
				double* pGamma = new double[pRobot->getNow()];
				Vector3d* pU = new Vector3d[pRobot->getNow()];
								
				kinematics.doInverseKinematicsPulleyEx(*pr, *pR, 0, 0, pBeta, pGamma, pC, pU);
	
				double beta, ux, uy, uz, rp;

				for (int i=0; i<pRobot->getNow(); i++)
				{
					rp=pRobot->r_pulley[i];
					glPushMatrix();
					Matrix3d R_base = pRobot->getBaseOrientation(i);
					PCRL::getAxisAngleFromMatrix(beta, ux, uy, uz, R_base);

					// change position and orientation to the winch frame
					glTranslated(pRobot->getBase(i).x(),pRobot->getBase(i).y(),pRobot->getBase(i).z());
					glRotated((beta/GL_PI)*180,ux,uy,uz);

					glRotated((pGamma[i]/GL_PI)*180,0, 0, 1);
					glTranslated(rp,0,0);
					IPAGL::makeCylinder_Symmetric(rp,rp/10,CVector3(0,1,1));
					glPopMatrix();
					glPopName();
				}
				
				/* // draw lines on pulley
				glLineWidth(1.5);
				glColor3d(0,0,0);
				for (int i=0; i<pRobot->getNow(); i++)
				{

					Matrix3d R_base = pRobot->getBaseOrientation(i);
					PCRL::getAxisAngleFromMatrix(beta, ux, uy, uz, R_base);
					//glTranslated(pRobot->getBase(i).x(),pRobot->getBase(i).y(),pRobot->getBase(i).z());

					// compute center point M of the pulley
					pM[i] = pRobot->getBase(i)+R_base*(Matrix3d::ZRotationMatrix3d(pGamma[i])*Vector3d(rp,h/2+0.001,0)); 


					glBegin(GL_LINES);
						glVertex3d(pC[i].x(),pC[i].y(),pC[i].z());
						glVertex3d(pM[i].x(),pM[i].y(),pM[i].z());
					glEnd();
				} */
				delete[]  pBeta; delete[] pGamma; delete[] pU;
			}
			else
			{
				cout << "Error during the visualization of  the cables\n";
			}

			if (pRobot->getElasticityModel() == CRobotData::NONELASTIC ||  pRobot->getElasticityModel() == CRobotData::LIN_ELASTIC)
			{
				// save current OpenGL line and lightning state
				glPushAttrib(GL_LINE_BIT | GL_ENABLE_BIT);
				
				// disable lightning to disable lightning on the wires resulting in color changes
				glDisable(GL_LIGHTING);

				for (int i=0; i<pRobot->getNow(); i++)
				{
					glLineWidth(2);
					
					//if forces are too high or low as given in the hull options, paint thick wires
					if  (((color_wires[i].R==255) && (color_wires[i].G==0) && (color_wires[i].B==255)) 
					   ||((color_wires[i].R==0)   && (color_wires[i].G==0) && (color_wires[i].B==255)))
					{ glLineWidth((float)6.0); }
					
					glPushName(i);
					glColor3d(color_wires[i].R,color_wires[i].G,color_wires[i].B);
					glBegin(GL_LINES);
						glVertex3d(pC[i].x(),pC[i].y(),pC[i].z());
						glVertex3d(pB[i].x(),pB[i].y(),pB[i].z());
					glEnd();
					glPopName();
				}

				//retrieve old OpenGL line and lightning state
				glPopAttrib();
			}
			else if (pRobot->getElasticityModel() == CRobotData::SAGGING)
			{
				double distBC = (pB[0]-pC[0]).norm();;
				if (distBC > 0.01) // error check
				{

					glLineWidth(1.5);
					glColor3d(color_wires[0].R,color_wires[0].G,color_wires[0].B); // has to be changed to get the right coloring 
					int steps =100;
					Vector3d* samplePoints = new Vector3d[steps];

					for (int i=0; i<pRobot->getNow(); i++)
					{
						int nCons = pRobot->pCable[i].nContraints + 2;  // number of constraints (2 constraints a_i, b_i + additional constraints)
						Vector3d* pCons = new Vector3d[nCons];         
						int simMode = 2;
						double f_ini = 1000;
						CCableChain cableChain(&nCons,pRobot->pCable[i].weight, pRobot->pCable[i].k_spec, f_ini);
						
						// set the first two point to pB, pC
						pCons[0] = pB[i];
						pCons[1] = pC[i];
						
						// set the remaining points to the additional constraints given by pCable.constraints
						for(int j=0; j<nCons-2; j++)
						{
							pCons[j+2] = pRobot->pCable[i].pConstraints[j];//pC[i]*1.2;
						}

						double l_nominal = pl->data()[i];
						cableChain.set_Parameters(pCons,&l_nominal,simMode);
						cableChain.get_Catenary(steps,samplePoints);
						
						glBegin(GL_LINE_STRIP);
							for (int j=0; j<steps; j++)
							{
								glVertex3d(samplePoints[j].x(),samplePoints[j].y(),samplePoints[j].z());
							}
						glEnd();

						delete [] pCons;
					}
					delete [] samplePoints;
				}
			}

			//delete the dynamic arrays again
			//note: delete[] pB,pC won't work!
			delete[] pB; delete[] pC;

			glPopName();
		}

		// draw an axis aligned frame around the all winches
		if (bFrameBBVisible)
		{
			glPushName(isIdFrameBoundingbox);
			//set frame color to grey
			glColor3d(0.29,0.29,0.29);
			//set frame width bigger than the wires
			glLineWidth(3);
			Vector3d bbmin,bbmax;
			pRobot->getBoundingBoxBase(bbmin,bbmax);
			glPushMatrix();
			glTranslated((bbmax.x()+bbmin.x())/2,(bbmax.y()+bbmin.y())/2,(bbmax.z()+bbmin.z())/2);
			IPAGL::makeWireBox(bbmax.x()-bbmin.x(),bbmax.y()-bbmin.y(),bbmax.z()-bbmin.z());		
			glPopMatrix();
			glPopName();
		}

		if (bFrameBBFancyFrame) 
		{
			// set the drawing parameters (color, size)
			double width=0.08;
			glColor3d(0.8,0.8,0.8);
			// extract the geometry from the robot to draw the frame around it
			Vector3d bbmin,bbmax;
			pRobot->getBoundingBoxBase(bbmin,bbmax);
			// draw the 12 bars
			IPAGL::makeFancyBox(CVector3(bbmin.x(),bbmin.y(),bbmin.z()),CVector3(bbmax.x(),bbmin.y(),bbmin.z()),width);
			IPAGL::makeFancyBox(CVector3(bbmin.x(),bbmin.y(),bbmin.z()),CVector3(bbmin.x(),bbmax.y(),bbmin.z()),width);
			IPAGL::makeFancyBox(CVector3(bbmax.x(),bbmin.y(),bbmin.z()),CVector3(bbmax.x(),bbmax.y(),bbmin.z()),width);
			IPAGL::makeFancyBox(CVector3(bbmin.x(),bbmax.y(),bbmin.z()),CVector3(bbmax.x(),bbmax.y(),bbmin.z()),width);

			IPAGL::makeFancyBox(CVector3(bbmin.x(),bbmin.y(),bbmin.z()),CVector3(bbmin.x(),bbmin.y(),bbmax.z()),width);
			IPAGL::makeFancyBox(CVector3(bbmax.x(),bbmax.y(),bbmin.z()),CVector3(bbmax.x(),bbmax.y(),bbmax.z()),width);
			IPAGL::makeFancyBox(CVector3(bbmax.x(),bbmin.y(),bbmin.z()),CVector3(bbmax.x(),bbmin.y(),bbmax.z()),width);
			IPAGL::makeFancyBox(CVector3(bbmin.x(),bbmax.y(),bbmin.z()),CVector3(bbmin.x(),bbmax.y(),bbmax.z()),width);

			IPAGL::makeFancyBox(CVector3(bbmin.x(),bbmin.y(),bbmax.z()),CVector3(bbmax.x(),bbmin.y(),bbmax.z()),width);
			IPAGL::makeFancyBox(CVector3(bbmin.x(),bbmin.y(),bbmax.z()),CVector3(bbmin.x(),bbmax.y(),bbmax.z()),width);
			IPAGL::makeFancyBox(CVector3(bbmax.x(),bbmin.y(),bbmax.z()),CVector3(bbmax.x(),bbmax.y(),bbmax.z()),width);
			IPAGL::makeFancyBox(CVector3(bbmin.x(),bbmax.y(),bbmax.z()),CVector3(bbmax.x(),bbmax.y(),bbmax.z()),width);
			// draw the eight corner connector
			glColor3d(0.1,0.1,0.1);
			glPushMatrix(); glTranslated(bbmin.x(),bbmin.y(),bbmin.z()); IPAGL::makeBox(0.1,0.1,0.1); glPopMatrix();
			glPushMatrix(); glTranslated(bbmin.x(),bbmin.y(),bbmax.z()); IPAGL::makeBox(0.1,0.1,0.1); glPopMatrix();
			glPushMatrix(); glTranslated(bbmin.x(),bbmax.y(),bbmin.z()); IPAGL::makeBox(0.1,0.1,0.1); glPopMatrix();
			glPushMatrix(); glTranslated(bbmin.x(),bbmax.y(),bbmax.z()); IPAGL::makeBox(0.1,0.1,0.1); glPopMatrix();
			glPushMatrix(); glTranslated(bbmax.x(),bbmin.y(),bbmin.z()); IPAGL::makeBox(0.1,0.1,0.1); glPopMatrix();
			glPushMatrix(); glTranslated(bbmax.x(),bbmin.y(),bbmax.z()); IPAGL::makeBox(0.1,0.1,0.1); glPopMatrix();
			glPushMatrix(); glTranslated(bbmax.x(),bbmax.y(),bbmin.z()); IPAGL::makeBox(0.1,0.1,0.1); glPopMatrix();
			glPushMatrix(); glTranslated(bbmax.x(),bbmax.y(),bbmax.z()); IPAGL::makeBox(0.1,0.1,0.1); glPopMatrix();
		}

		// draw an axis aligned bounding box around the platform
		if (bPlatformVisible)
		{
			CMatrix3 R(pR->col(0).x(),pR->col(0).y(),pR->col(0).z(),
				pR->col(1).x(),pR->col(1).y(),pR->col(1).z(),
				pR->col(2).x(),pR->col(2).y(),pR->col(2).z());
			glLoadName(4);
			Vector3d bbmin,bbmax;
			pRobot->getBoundingBoxPlatform(bbmin,bbmax);
			glPushMatrix();
			glTranslated(pr->x(),pr->y(),pr->z());
			CGLShape::MultFrame(&CFrame(CVector3(0,0,0),R));
			glTranslated((bbmax.x()+bbmin.x())/2,(bbmax.y()+bbmin.y())/2,(bbmax.z()+bbmin.z())/2);
			
			IPAGL::makeWireBox(bbmax.x()-bbmin.x(),bbmax.y()-bbmin.y(),bbmax.z()-bbmin.z());

			glPopMatrix();
		}


		if (bBaseFramesVisible)
		{
			// todo: implement winch frames properly
			//Matrix3dR_base;

			double beta, ux, uy, uz;
			glPushName(isIdBaseFrames);
			for (int i=0; i<pRobot->getNow(); i++)
			{
				glPushName(i);
				glPushMatrix();
				Matrix3d R_base = pRobot->getBaseOrientation(i);
				PCRL::getAxisAngleFromMatrix(beta, ux, uy, uz, R_base);
				glTranslated(pRobot->getBase(i).x(),pRobot->getBase(i).y(),pRobot->getBase(i).z());
				glRotated((beta/GL_PI)*180,ux, uy, uz);
				IPAGL::makeFrame();
				glPopMatrix();
				glPopName();
			}

			//IPAGL::makeFrame(r);
			//glVertex3d(pRobot->getBase(i).x(),pRobot->getBase(i).y(),pRobot->getBase(i).z());
			//glVertex3d(b.x(),b.y(),b.z());

			glPopName();
		}		
	}
};

/*! The drawing functions for displaying the workspace hull 
 */
class CShapeInterference : public CGLShape
{
	CInterference* pI;
	CGLRGBColor color_distal,color_proximal;
public:
	explicit CShapeInterference(CInterference& Interference)
		: pI(&Interference),color_distal(1,0,0),color_proximal(0,1,0) {}

	//! the draw function for the hull
	void draw()
	{
		// the clipping is performed by the openGL engine through clipping planes
		// this is much more efficient and easier to implement compared to calculating the
		// intersections between the region and the cube of interest
		double planes[] =  {-1, 0, 0, pI->xmax, 1, 0, 0, -pI->xmin, 0,-1, 0, pI->ymax, 0, 1, 0, -pI->ymin, 0, 0,-1, pI->zmax, 0, 0, 1, -pI->zmin};
		unsigned int i,j;

		// beschränke die Ausgabe auf einen Quader, der durch Cliping planes beschnitten wird
		for (i=0; i<6; i++)
		{
			glEnable(GL_CLIP_PLANE0+i);
			glClipPlane(GL_CLIP_PLANE0+i,planes+4*i);
		}

		// draw the green triangles calculated by interference methods
		glBegin(GL_TRIANGLES);
		for (i=0; i<pI->triangles.size(); i+=3)
		{

			glColor3d(0,1,0);
			glVertex3dv(&pI->triangles[i].x());
			glVertex3dv(&pI->triangles[i+1].x());
			glVertex3dv(&pI->triangles[i+2].x());
		}
		glEnd();

		// draw the two generating edges of each triangle in order to underline the structure
		glBegin(GL_LINES);
		glColor3d(0,0,0);
		for (i=0; i<pI->triangles.size(); i+=3)
		{
			glVertex3dv(&pI->triangles[i].x());
			glVertex3dv(&pI->triangles[i+1].x());
			glVertex3dv(&pI->triangles[i].x());
			glVertex3dv(&pI->triangles[i+2].x());
			glVertex3dv(&pI->triangles[i+1].x());
			glVertex3dv(&pI->triangles[i+2].x());
		}
		glEnd();

		// draw the red triangles calculated by interference methods
		glBegin(GL_TRIANGLES);	
		for (i=0; i<pI->redtriangles.size(); i+=3)
		{

			glColor3d(1,0,0);
			glVertex3dv(&pI->redtriangles[i].x());
			glVertex3dv(&pI->redtriangles[i+1].x());
			glVertex3dv(&pI->redtriangles[i+2].x());
		}
		glEnd();

		// draw the two generating edges of each triangle in order to underline the structure
		glBegin(GL_LINES);
		glColor3d(0,0,0);
		for (i=0; i<pI->redtriangles.size(); i+=3)
		{
			glVertex3dv(&pI->redtriangles[i].x());
			glVertex3dv(&pI->redtriangles[i+1].x());
			glVertex3dv(&pI->redtriangles[i].x());
			glVertex3dv(&pI->redtriangles[i+2].x());
			glVertex3dv(&pI->redtriangles[i+1].x());
			glVertex3dv(&pI->redtriangles[i+2].x());
		}
		glEnd();

		// draw black looping lines 
		for (i=0; i<pI->looplines.size(); i++)
        {
                glBegin(GL_LINE_LOOP);
                glColor3d(0,0,0);
				for (j=0; j<pI->looplines[i].size(); j++)
				{
					glVertex3dv(&pI->looplines[i][j].x());
				}
                glEnd();
        }

		// draw layer connections
		for (i=0; i<pI->layerconnections.size(); i++)
        {
                glBegin(GL_TRIANGLE_STRIP);
                CGLRGBColor color(pI->layerconnections[i][0](0),pI->layerconnections[i][0](1),pI->layerconnections[i][0](2));
                glColor3dv(&color.R);
				for (j=1; j<pI->layerconnections[i].size(); j++)
				{
					glVertex3dv(&pI->layerconnections[i][j].x());
				}
                glEnd();
        }

        // draw triangle fan

        for (i=0; i<pI->printshapes.size(); i++)
        {
                glBegin(GL_TRIANGLE_FAN);
                CGLRGBColor color(pI->printshapes[i][0](0),pI->printshapes[i][0](1),pI->printshapes[i][0](2));

                glColor3dv(&color.R);
                for (j=1; j<pI->printshapes[i].size(); j++)
                {
                        glVertex3dv(&pI->printshapes[i][j].x());
                }
                glEnd();

        }


		// disable the clipping function
		for (i=0; i<6; i++)
			glDisable(GL_CLIP_PLANE0+i);

		// draw the grid with the detected obstacle-cable collisions
		glPointSize(5);
		glBegin(GL_POINTS);
			glColor3d(0,1,0);
			for (i=0; i<pI->validPositions.size(); i++)
				glVertex3dv(&pI->validPositions[i].x());
		glEnd();
		// reset to default value
		glPointSize(1);
	}
};

/*! The drawing functions for the wires and the wire span
 */
class CShapeWireSpan : public CGLShape
{
	CWorkspaceHull* pHull;
	CRobotData* pRobot;
	Matrix3d* pR;
	CGLRGBColor color;
public:
	int cableSelector;	//!< determines which cable is visible; negative values make all cables visible
public:
	CShapeWireSpan(CWorkspaceHull& Hull, CRobotData& Robot, Matrix3d& R)
		: pHull(&Hull),pRobot(&Robot),pR(&R), color(0,1,0), cableSelector(-1) {}

	//! the draw function for the hull
	void draw()
	{
		for (int j=0; j<pRobot->getNow(); j++)
		{
			if (cableSelector>=0 && j!=cableSelector)
				continue;
			Vector3d ai = pRobot->getBase(j);	
			glBegin(GL_LINES);
			glColor3dv(&color.R);
			for (int i=0; i<pHull->FinishedTriangles; i++)
			{
				glVertex3dv(&ai.x());
				Vector3d bi = *pR * pRobot->getPlatform(j) + pHull->vertices[pHull->Triangles[i].i];
				glVertex3dv(&bi.x());
				glVertex3dv(&ai.x());
				bi = *pR * pRobot->getPlatform(j) + pHull->vertices[pHull->Triangles[i].j];
				glVertex3dv(&bi.x());
				glVertex3dv(&ai.x());
				bi = *pR * pRobot->getPlatform(j) + pHull->vertices[pHull->Triangles[i].k];
				glVertex3dv(&bi.x());
			}
			glEnd();
		}
	}
};

/*! The drawing functions for the wires span as cone
 */
class CShapeWireSpanCone : public CGLShape
{
	CWorkspaceHull* pHull;
	CRobotData* pRobot;
	Matrix3d* pR;
	CGLRGBColor color;
public:
	int cableSelector;	//!< determines which cable is visible; negative values make all cables visible
public:
	CShapeWireSpanCone(CWorkspaceHull& Hull, CRobotData& Robot, Matrix3d& R)
		: pHull(&Hull),pRobot(&Robot),pR(&R), color(0,1,1), cableSelector(-1) {}

	//! the draw function for the hull
	void draw()
	{
		// draw the wire span as triangulated hull
		if (pHull->CableSpanTriangulation.size()>0)
		{
			glBegin(GL_TRIANGLES);
			glColor3dv(&color.R);
			int segments = pHull->CableSpanTriangulation.cols() / pRobot->getNow();
			for (int i=0; i<pHull->CableSpanTriangulation.cols()-1; i++)
			{
				int cable = i / segments;
				if (cableSelector>=0 && cable!=cableSelector)	// skip the cables that are not selected for visualization
					continue;
				int point = i % segments;
				Vector3d v1 = pHull->CableSpanTriangulation.block(0,i,3,1);
				Vector3d v2;
				if (point!=segments-1) // the last vertex in the block is connect to the first
					v2= pHull->CableSpanTriangulation.block(0,i+1,3,1);
				else
					v2= pHull->CableSpanTriangulation.block(0,i+1-segments,3,1);
				Vector3d v3 = pRobot->getBase(cable);
				Vector3d normal=(v2-v1).cross(v3-v1);
				glNormal3dv(&normal.x());
				glVertex3dv(&v1.x());
				glVertex3dv(&v2.x());
				glVertex3dv(&v3.x());
			}
			glEnd();
		}
	}
};

/*! The drawing functions for the wires
 */
class CShapeCrosssection : public CGLShape
{
	CWorkspaceCrosssection* pCS;
	CGLRGBColor color;
public:
	explicit CShapeCrosssection(CWorkspaceCrosssection& CS) 
		: pCS(&CS),	color(0,0,0) {}

	//! the draw function for the cross section
	void draw()
	{
		// drawing mode for cross sections 
		glBegin(GL_LINES);
		glColor3dv(&color.R);
		glNormal3d(0,0,1);
		unsigned int i;
		for (i=0; i<pCS->Lines.size(); i++)
		{
			glVertex3dv(&pCS->vertices[pCS->Lines[i].first].x());
			glVertex3dv(&pCS->vertices[pCS->Lines[i].second].x());
		}
		glEnd();
	}
};

/*! drawing function for the platform related forces
 */
class CShapePlatformForces : public CGLShape
{
	CRobotData *pRobot;
	Matrix3d* pR;
	Vector3d* pr;
	MatrixXd* pF, *pw; // pointer to cable forces and applied wrench
	MatrixXd* pl;
public:
	CGLRGBAColor color_forceVectors;
	double scalingFactor;
	bool bVisible;

public:
	// robot: robot data
	// r,R,F, w: robot state from CRobotDocument
	CShapePlatformForces(CRobotData& robot, Vector3d& r, Matrix3d& R, MatrixXd& l, MatrixXd& F, MatrixXd& w)
		: pRobot(&robot), pr(&r), pR(&R), pF(&F), pw(&w), pl(&l),
		color_forceVectors(1,1,0,1),
		scalingFactor(0.001)
	{
		bVisible=true;
	}

	void draw()
	{
		if(false)
		{
			double radius = 0.01;
			CVector3 rgb(color_forceVectors.R,color_forceVectors.G,color_forceVectors.B);
			// Draw gravitational vector at platform center point
			CVector3 f_applied((*pw)(0), (*pw)(1), (*pw)(2)); // compute gravitational force
			CVector3 v1(pr->x(), pr->y(), pr->z());
			CVector3 v2 = v1 + f_applied * scalingFactor;
			Vector3d* pC = new Vector3d[pRobot->getNow()];

			if (length(v2-v1) > 0.001)
			{
				IPAGL::makeVector(v1,v2,radius,rgb,color_forceVectors.A);
			}

			CElastoKinematics kinematics(*pRobot);
			if (pRobot->getRobotKinematicsModel() == CRobotData::FIXED)
			{
				for (int i=0; i<pRobot->getNow(); i++)
				{
					pC[i] = pRobot->getBase(i);
				}
			}
			else if (pRobot->getRobotKinematicsModel() == CRobotData::PULLEY)
			{
				kinematics.doInverseKinematicsPulleyEx(*pr, *pR, 0, 0, 0, 0, pC, 0);
			}

			Vector3d PB, u, PF;
			Vector3d u1,u2;
			double f1,f2;
			f1= 0; f2 =0;

			for (int i=0; i<pRobot->getNow(); i++)
			{
				if (pRobot->getElasticityModel() == CRobotData::NONELASTIC ||  pRobot->getElasticityModel() == CRobotData::LIN_ELASTIC)
				{
					PB = *pr + *pR * pRobot->getPlatform(i);
					u1 =  pC[i]- PB;
					if (u1.norm() != 0)
					{
						u1.normalize();
					}

					PF = PB + u1 * ((*pF)(i) *scalingFactor);
					//PF = PB + u1 * (f1 *scalingFactor);

					v1 = CVector3(PB.x(), PB.y(), PB.z());
					v2 = CVector3(PF.x(), PF.y(), PF.z());

					if (length(v2-v1) > 0.001)
					{
						if (u1.x()*(v2-v1).x < 0 || u1.y()*(v2-v1).y < 0 || u1.z()*(v2-v1).z < 0)
						{
							rgb.x=1;
							rgb.y=0;
							rgb.z=0;
						}
						else
						{
							rgb.x=1;
							rgb.y=1;
							rgb.z=0;
						}
						IPAGL::makeVector(v1,v2,radius,rgb,color_forceVectors.A);
					}
				}
				else if (pRobot->getElasticityModel() == CRobotData::SAGGING)
				{
					int simMode = 2;

					PB = *pr + *pR * pRobot->getPlatform(i);
					//CCable cable(&PB, &pC[i], &(pl->data()[i]), pRobot->pCable[i].weight, pRobot->pCable[i].k_spec); // k_spec = EA0
					//cable.set_Parameters(&PB, &pC[i], &(pl->data()[i]),simMode);
					//cable.compute_AnchorageForces(u1,u2,f1,f2);	
					
					int nCons = pRobot->pCable[i].nContraints + 2;  // number of constraints (2 constraints a_i, b_i + additional constraints)
					Vector3d* pCons = new Vector3d[nCons];     
					double f_ini = 1000;
					CCableChain cableChain(&nCons,pRobot->pCable[i].weight, pRobot->pCable[i].k_spec, f_ini);
						
					pCons[0] = PB;
					pCons[1] = pC[i];
						
					// set the remaining points to the additional constraints given by pCable.constraints
					for(int j=0; j<nCons-2; j++)
					{
						pCons[j+2] = pRobot->pCable[i].pConstraints[j];//pC[i]*1.2;
					}

					double l_nominal = pl->data()[i];
					cableChain.set_Parameters(pCons,&l_nominal,simMode);

					Vector3d PF;
					for(int j=0; j<nCons-1; j++)
					{
						cableChain.compute_AnchorageForces(j,u1,u2,f1,f2); // show anchorage forces of catenary 1 of cable i

						// draw all force vectors of the catenaries aside from the platform force vectors
						PF = pCons[j] + u1 * f1 *scalingFactor;
						v1 = CVector3(pCons[j].x(), pCons[j].y(), pCons[j].z());
						v2 = CVector3(PF.x(), PF.y(), PF.z());
						IPAGL::makeVector(v1,v2,radius,rgb,color_forceVectors.A);

						PF = pCons[j+1] - u2 * f2 *scalingFactor;
						v1 = CVector3(pCons[j+1].x(), pCons[j+1].y(), pCons[j+1].z());
						v2 = CVector3(PF.x(), PF.y(), PF.z());
						IPAGL::makeVector(v1,v2,radius,rgb,color_forceVectors.A);
					}

					delete [] pCons;
				}	
			}
			// todo: maybe it is better to merge the cable visualization and force visualization
		}
	}
};


/*! Draw winch pulleys */
class CShapeWinchPulleys : public CGLShape
{
	CRobotData* pRobot;
	CElastoKinematics kinematics;
	MatrixXd pl;

public:
	CGLRGBAColor color_pulleys;
	double thickness;
	bool bVisible;

public:
	// robot: robot data
	// r,R,F, w: robot state from CRobotDocument
	CShapeWinchPulleys(CRobotData& robot, Vector3d& r, Matrix3d& R)
		: kinematics(robot), pRobot(&robot), thickness(0.1)
	{
		
		bVisible=true;
	}

	void draw()
	{
		if(bVisible)
		{
			//kinematics.doInverseKinematicsPulley(r,R,
			//pRobot->R_base[0]	
		}
	}
};


/*! Draw pose list*/
template <class T>
class CShapePoseList: public CGLShape
{
protected:
	int factor;
	//! The line pattern is defined by a 16bit integer (0xffff yields a closed line)

public:
	bool show_pointCloud;
	bool show_lineStrip;
	bool show_rotationVectors;
	CGLRGBAColor color_pointCloud;
	CGLRGBAColor color_lineStrip;
	CGLRGBAColor color_rotationVector;
	unsigned int linePattern;
	float lineWidth;
	float pointSize;
	double vector_radius;

	CPoseList<T> m_poseList;

public:
	//CShapePoseList(CPoseList<T>& x){;};
	explicit CShapePoseList(CPoseList<T>& poseList): m_poseList(poseList), color_pointCloud(1,0,0,1), color_lineStrip(0,1,0,1), color_rotationVector(0,0,1,1),
		factor(1), linePattern(0xffff), lineWidth(1), pointSize(5.0), vector_radius(0.005), show_pointCloud(true), show_lineStrip(false) {}
	
	~CShapePoseList()
	{
		m_poseList.clear();
	}

	void draw()
	{
		CPoseList<T>::const_iterator it;

		// Render point cloud
		if(show_pointCloud)
		{
		   glEnable(GL_BLEND);
		   glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
		   glColor4d(color_pointCloud.R,color_pointCloud.G, color_pointCloud.B, color_pointCloud.A);

		   glEnable(GL_LINE_STIPPLE);
		   glPointSize(pointSize);
		   glBegin(GL_POINTS);
				it=m_poseList.begin(); 
				for (it;it!=m_poseList.end();it++)
				{
					T* pw = *it;
					glVertex3d(pw->r[0], pw->r[1], pw->r[2]);
				}
		   glEnd();
		   glPointSize(1.0); // set default line width
		   glDisable(GL_LINE_STIPPLE);
		   glDisable(GL_BLEND);
		}

		// render roation vectors using axis angle representation
		if(show_rotationVectors)
		{
			double angle;
			Vector3d rotAxis;

			it=m_poseList.begin(); 
			for (it;it!=m_poseList.end();it++)
			{
				T* pw = *it;
				pw->getAxisAngle(angle, rotAxis);
				CVector3 v1(pw->r[0],pw->r[1],pw->r[2]), v2(pw->r[0]+rotAxis[0],pw->r[1]+rotAxis[1],pw->r[2]+rotAxis[2]);			
				IPAGL::makeVector(v1, v2, vector_radius, CVector3(color_rotationVector.R, color_rotationVector.G, color_rotationVector.B), color_rotationVector.A);
			}
		}

		// Render line strip
		if(show_lineStrip)
		{
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
			glColor4d(color_lineStrip.R,color_lineStrip.G, color_lineStrip.B, color_lineStrip.A);

			glEnable(GL_LINE_STIPPLE);

			glLineStipple(factor, linePattern);
			glLineWidth(lineWidth);
			glBegin(GL_LINE_STRIP);

				it=m_poseList.begin(); 

				for (it;it!=m_poseList.end();it++)
				{
					T* pw = *it;
					glVertex3d(pw->r[0], pw->r[1], pw->r[2]);
				}
			glEnd();
			glLineWidth(1.0); // set default line width
			glDisable(GL_LINE_STIPPLE);
			glDisable(GL_BLEND);
		}
	}
};

class CShapePoseListStatic: public CShapePoseList<CPoseStatic>
{
public:
	explicit CShapePoseListStatic(CPoseList<CPoseStatic>& poseList): CShapePoseList<CPoseStatic>(poseList) {}
};

class CShapePoseListKinetostatic: public CShapePoseList<CPoseKinetostatic>
{
public:
	bool show_velocity;
	bool show_acceleration;
	bool show_angularVelocity;
	bool show_angularAcceleration;
	bool show_force;
	bool show_torque;
	
	CGLRGBAColor color_velocity;
	CGLRGBAColor color_acceleration;
	CGLRGBAColor color_angularVelocity;
	CGLRGBAColor color_angularAcceleration;
	CGLRGBAColor color_force;
	CGLRGBAColor color_torque;

public:
	explicit CShapePoseListKinetostatic(CPoseList<CPoseKinetostatic>& poseList): CShapePoseList<CPoseKinetostatic>(poseList),
	show_velocity(false), show_acceleration(false), show_angularVelocity(false), show_angularAcceleration(true), show_force(false), show_torque(false),
	color_velocity(1,1,0,1), color_acceleration(1,1,0,1), color_angularVelocity(1,1,0,1), color_angularAcceleration(1,1,0,1),
	color_force(1,1,0,1), color_torque(1,1,0,1) {}

	void draw()
	{
		CShapePoseList::draw(); 
		
		CPoseList<CPoseKinetostatic>::const_iterator it;

		// Render velocity vectors
		if(show_velocity)
		{
			it=m_poseList.begin(); 
			for (it;it!=m_poseList.end();it++)
			{
				CPoseKinetostatic* pw = *it;
				CVector3 v1(pw->r[0],pw->r[1],pw->r[2]), v2(pw->r[0]+pw->v[0],pw->r[1]+pw->v[1],pw->r[2]+pw->v[2]);			
				IPAGL::makeVector(v1, v2, vector_radius, CVector3(color_velocity.R, color_velocity.G, color_velocity.B), color_velocity.A);
			}
		}

		// Render acceleration vectors
		if(show_acceleration)
		{
			it=m_poseList.begin(); 
			for (it;it!=m_poseList.end();it++)
			{
				CPoseKinetostatic* pw = *it;
				CVector3 v1(pw->r[0],pw->r[1],pw->r[2]), v2(pw->r[0]+pw->a[0],pw->r[1]+pw->a[1],pw->r[2]+pw->a[2]);			
				IPAGL::makeVector(v1, v2, vector_radius, CVector3(color_acceleration.R, color_acceleration.G, color_acceleration.B), color_acceleration.A);
			}
		}

		// Render angular velocity vectors
		if(show_angularVelocity)
		{
			it=m_poseList.begin(); 
			for (it;it!=m_poseList.end();it++)
			{
				CPoseKinetostatic* pw = *it;
				CVector3 v1(pw->r[0],pw->r[1],pw->r[2]), v2(pw->r[0]+pw->omega[0],pw->r[1]+pw->omega[1],pw->r[2]+pw->omega[2]);			
				IPAGL::makeVector(v1, v2, vector_radius, CVector3(color_angularVelocity.R, color_angularVelocity.G, color_angularVelocity.B), color_angularVelocity.A);
			}
		}

		// Render angular acceleration vectors
		if(show_angularAcceleration)
		{
			it=m_poseList.begin(); 
			for (it;it!=m_poseList.end();it++)
			{
				CPoseKinetostatic* pw = *it;
				CVector3 v1(pw->r[0],pw->r[1],pw->r[2]), v2(pw->r[0]+pw->alpha[0],pw->r[1]+pw->alpha[1],pw->r[2]+pw->alpha[2]);			
				IPAGL::makeVector(v1, v2, vector_radius, CVector3(color_angularAcceleration.R, color_angularAcceleration.G, color_angularAcceleration.B), color_angularAcceleration.A);
			}
		}

		// Render force vectors
		if(show_force)
		{
			it=m_poseList.begin(); 
			for (it;it!=m_poseList.end();it++)
			{
				CPoseKinetostatic* pw = *it;
				CVector3 v1(pw->r[0],pw->r[1],pw->r[2]), v2(pw->r[0]+pw->f[0],pw->r[1]+pw->f[1],pw->r[2]+pw->f[2]);			
				IPAGL::makeVector(v1, v2, vector_radius, CVector3(color_force.R, color_force.G, color_force.B), color_force.A);
			}
		}

		// Render torque vectors
		if(show_torque)
		{
			it=m_poseList.begin(); 
			for (it;it!=m_poseList.end();it++)
			{
				CPoseKinetostatic* pw = *it;
				CVector3 v1(pw->r[0],pw->r[1],pw->r[2]), v2(pw->r[0]+pw->tau[0],pw->r[1]+pw->tau[1],pw->r[2]+pw->tau[2]);			
				IPAGL::makeVector(v1, v2, vector_radius, CVector3(color_torque.R, color_torque.G, color_torque.B), color_torque.A);
			}
		}

	}
};


// Class to visualize PosePropertyEvaluation results
// Limited to visualize boolean results. Could be extended to visualize also other results.
class CShapePosePropertyEvaluation: public CGLShape
{
	PCRL::CPosePropertyEvaluation* pPPE;
	PCRL::CPoseListKinetostatic* pPL;
	
public:
	int iCol;

public:
	explicit CShapePosePropertyEvaluation(CPoseListKinetostatic& PoseListKinStat, CPosePropertyEvaluation& PosePropEval): pPL(&PoseListKinStat), pPPE(&PosePropEval), iCol(0){}

	// Method to set the index of the column of the PosePropertyEvaluator that shall be visualized
	void setColumn(int& iColumn)
	{
		iCol = iColumn;
	}

	void draw()
	{
		// get PosePropertyEvaluator results
		MatrixXd PoseEvaluatorResults = pPPE->getResult();

		// Check if number of poses in PoseList is equal to the number of results in PosePropertyEvaluator, check to ensure that the POseList belongs to the PoseEvaluator results,
		// !Check is not 100% secure. If the size of another pose list is the same it fails.
		if(pPPE->getResultCount() != pPL->size())
		{
			return;
		}

		// draw PosePropertyEvaluator bool results in i-th column in green if true (0), in red if false (1)
		glPointSize(5);
		glDisable(GL_LIGHTING);
		
		int i = 0;
		for (auto itor = pPL->begin(); itor!=pPL->end(); ++itor)
		{
			if(PoseEvaluatorResults(i, iCol))
			{
				//green
				glColor3d(0,1,0);
			}
			else
			{
				//red
				glColor3d(1,0,0);
			}

			glBegin(GL_POINTS);
			Vector3d r = (**itor).r;
			glVertex3d(r.x(),r.y(),r.z());

			i += 1;
		}
		glEnd();
	};
};

} // end of namespace PCRL