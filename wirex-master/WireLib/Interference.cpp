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

/*!*******************************************************************
 *  \file   : Interference.cpp
 *
 *  Project   Wirelib
 *
 *  \Author   Andreas Pott
 *
 *  \Date     11.12.2009	(part of MoWiRoInterference)
 *			  15.12.2009	(refactored by asp)
 *
 *********************************************************************
 */ 

#include "Interference.h"
#include "motionPlanning/Utilities.h"

#include <iostream>
#include <fstream>
#include <queue>  
#include <deque>

namespace PCRL {

CInterference::CInterference(CRobotData& robot)
: CAlgorithm(robot)
{
	// assign the default values for the parameter
	xmin = -5.0, xmax = 5.0, ymin = -4.0, ymax = 4.0, zmin = -2.0, zmax = 4.0; // range for clipping planes
	// range and step size for discretization of the orientation workspace 
	alphaMin	= -10*DEG_TO_RAD;
	alphaMax	=  10*DEG_TO_RAD;
	deltaAlpha	=  10*DEG_TO_RAD;
	betaMin		= -10*DEG_TO_RAD;
	betaMax		=  10*DEG_TO_RAD;
	deltaBeta	=  10*DEG_TO_RAD;
	gammaMin    =   0*DEG_TO_RAD;	// we start with 0 for compatibility reasons
	gammaMax    =   0*DEG_TO_RAD;	// we start with 0 for compatibility reasons
	deltaGamma  =   0*DEG_TO_RAD;	// we start with 0 for compatibility reasons
	TriangleLength = 10;		// indicate the size of the triangle region to be considered (theoretically infinity)
	eps = 1e-6;				// threshold for angle between two lines to be considered as parallel
	R0 = Matrix3d::ZRotationMatrix3d(0.0);
	cableCableCollisions.resize(pRobot->getNow(),pRobot->getNow());
	cableCableBoxCollisions.resize(pRobot->getNow(),pRobot->getNow());
	firstCable = 0;
	lastCable = 1000000000;	// this is a magic number as we expect no robot to have more cables

	etaMax = 0.0001; // set size of platform orientation set to zero, so that it only contains the default orientation

	deltaalpha = 5*DEG_TO_RAD; // set a default value for the deltaalpha parameter controling the discretization of the cable-cable interference sets
	deltaeta = 5*DEG_TO_RAD; // set a default value for the deltaeta parameter controling the discretization of the cable-cable interference disc and sphere cap subsets
	stepscircle = 9; // set a default value for the stepscircle parameter controling the discretization of the cable-cable interference disc and sphere cap subsets
	displayAllCableCableInterferenceSets = false; // set option to display only a subset of interference sets

	displayshadows = true; // set option to display shadows of printable shapes
	printheight = 1.0;     // set default hight of printing interval
	numberofprintlayers = 100;    //!< set default number of printing layers in height interval

	printParaVisNumber = 0; // set option to display the largest parallelogram in terms of area
	pn << 0,0,-0.3;		  // set a default value for the printing nozzle
	printcenter << 0,0,0; // set a default value for the center of printed shapes
	d1 << 1,0,0;          // set a default value for the first printing direction
	d2 << 0,1,0;          // set a default value for the second printing direction
	pEllipse = 2.0;         // set a default value for exponent parameter of superellipses

	platformcenterposition << 0,0,0; // set a default value for the center of the cable-platform collision free workspace
	refinement = 3;                    // set a default value for the refinement parameter of the cable-platform collision free workspace
	searchsteps = 10;                  // set a default value for the searchsteps parameter of the cable-platform collision free workspace

	printcolor   << 0,1,0;			 //!< set a default value for the color of printable shapes
	shadowcolor  << 1,0,0;			 //!< set a default value for the color of shadow shapes

	multlayermode = false;     //!< set a default value to disable the multiple layer option in the parallelogram calculation
}

CInterference::~CInterference(void)
{
}

/*! visualize cable-cable Interference sets
 *	first etaMax is set and then all necessary methods are called to compute and visualize  cable-cable Interference sets
 */
void CInterference::visualizeInterferenceCableCable()
{
	updateClippingPlanes();
	setupCableCableInterferenceSets();
	calculateCableCableInterferenceSetsHulls();
	setupCableCableInterferenceHullsTriangles();
}

/*! visualize boundary ofcable-platform interference set
 *  \param filename path and name to the file containing the platform shape
 */
void CInterference::visualizeInterferenceCablePlatform(const string& filename)
{
	redtriangles.clear();
	updateClippingPlanes();
	loadplatformstldata(filename); // "C:/Users/vls-mf/Documents/Masterarbeit/code/Geometryfiles/PlattformNeu2.STL");
	drawplatformstldata();
	setupCablePlatformCollisionCones();
	drawCablePlatformCones();
	newCablePlatformRayBall();
	calculateCablePlatform();
	drawCablePlatformCollisionFreeSpace();
}

/*! visualize printable star-shaped polygon
 */
void CInterference::visualizeInterferenceCablePrintStarShape()
{
	layerconnections.clear();
	looplines.clear();
	printshapes.clear();
	updateClippingPlanes();
	calculatePrintShadowData();
	calculateInterferenceWithPrintStarShape();
	drawStarShape();	
}

/*! visualize printable superellipse
 */
void CInterference::visualizeInterferenceCablePrintEllipse()
{
	layerconnections.clear();
	looplines.clear();
	printshapes.clear();
	updateClippingPlanes();
	calculatePrintShadowData();
	calculateInterferenceWithPrintEllipse();
	drawEllipse();	
}

/*! visualize printable parallelogram
 */
void CInterference::visualizeInterferenceCablePrintParallelogram()
{
	layerconnections.clear();
	looplines.clear();
	printshapes.clear();
	updateClippingPlanes();
	calculatePrintShadowData();
	calculateInterferenceWithPrintParallelogram();
	drawParallelogram();	
}

/*! visualize multiple layers of printable superellipses if the layers can be calculated independently
 */
bool CInterference::visualizeInterferenceCablePrintEllipseMultipleLayers()
{
	if (checkInterferenceWithPrintEllipseMultLayerCriteria() == false)
	{
		return false;
	}

	looplines.clear();
	printshapes.clear();
	layerconnections.clear();
	updateClippingPlanes();

	calculatePrintShadowData();
	calculateInterferenceWithPrintEllipse();
	
	const double printcenterbasez = printcenter(2);

	const int steps1 = 100;
	const double delta1 = 1/(double)steps1;

	vector<Vector3d> loop;

	for (int i=0; i < steps1; i++)
	{
		double y = pow(delta1*i,1/pEllipse);
		double x = pow(1-delta1*i,1/pEllipse);
		loop.push_back(+x*d1+y*d2);

	}
	for (int i=0; i < steps1; i++)
	{
		double y = pow(1-delta1*i,1/pEllipse);
		double x = pow(delta1*i,1/pEllipse);
		loop.push_back(-x*d1+y*d2);

	}
	for (int i=0; i < steps1; i++)
	{
		double y = pow(delta1*i,1/pEllipse);
		double x = pow(1-delta1*i,1/pEllipse);
		loop.push_back(-x*d1-y*d2);

	}
	for (int i=0; i < steps1; i++)
	{
		double y = pow(1-delta1*i,1/pEllipse);
		double x = pow(delta1*i,1/pEllipse);
		loop.push_back(+x*d1-y*d2);
	}
	
	vector<Vector3d> printloop;
	printloop.push_back(printcolor);
	for (int k=1; k < (int)(loop.size()) ; k++)
	{
		printloop.push_back(printcenter+loop[k]*printellipsesize);
	}
	printshapes.push_back(printloop);
	printloop.clear();
	Vector3d printcenterlower = printcenter;

	for (int k=0; k < numberofprintlayers; k++)
	{
		printcenter(2) = printcenter(2)+(printheight/(double) numberofprintlayers);
		calculatePrintShadowData();
		calculateInterferenceWithPrintEllipse();

		printloop.push_back(printcolor);
		for (int k=1; k < (int)(loop.size()); k++)
		{
			printloop.push_back(printcenter+loop[k]*printellipsesize);
		}
		printshapes.push_back(printloop);
		printloop.clear();

		vector<Vector3d> printloop;
		for (int k=1; k < (int)(loop.size()); k++)
		{
			printloop.push_back(printcenter+loop[k]*printellipsesize);
		}
		looplines.push_back(printloop);
		printloop.clear();

		Vector3d printcenterlower = printcenter;
		printcenterlower(2) = printcenterlower(2)-(printheight/(double) numberofprintlayers);

		for (int k=1; k < (int)(loop.size()); k++)
		{
			printloop.push_back(printcenterlower+loop[k]*printellipsesize);
		}
		looplines.push_back(printloop);
		printloop.clear();
		printloop.push_back(printcolor);
		for (int k=0; k < (int)(loop.size()); k++)
		{
			printloop.push_back(printcenter     +loop[k]*printellipsesize);
			printloop.push_back(printcenterlower+loop[k]*printellipsesize);
		}
		layerconnections.push_back(printloop);
		printloop.clear();

		printcenterlower = printcenter;
	}
	
	printcenter(2) = printcenterbasez;
	return true;
}

/*! calculate multiple layers of printable parallelograms if the layers can be calculated independently
 */
bool CInterference::calculateInterferenceCablePrintParalellogramMultipleLayers()
{
	if (checkInterferenceWithPrintParaMultLayerCriteria() == false)
	{
		return false;
	}

	const double eps = 0.000001;

	multparasizes.clear();

	calculatePrintShadowData();
	calculateInterferenceWithPrintParallelogram();	

	printParaVisNumber = printParaVisNumber%(int)printparasizes.size();
	double res1 = charparasizes[printparasizes[printParaVisNumber][0]][0];
	double res2 = charparasizes[printparasizes[printParaVisNumber][1]][1];

	vector<double> rescur(2,0);
	rescur[0] = res1;
	rescur[1] = res2;
	multparasizes.push_back(rescur);

	multlayermode = true;
	notexpandable.clear();
	for (int i=0; i  < (int) shadowcenters.size(); i++)
	{
		if (res1>charparasizes[i][0]+eps)
		{
			notexpandable.push_back(1);
		} else if (res2>charparasizes[i][1]+eps)
		{
			notexpandable.push_back(0);
		} else
		{
			notexpandable.push_back(2);
		}
	}

	const double printcenterbasez = printcenter(2);
	for (int k=0; k<numberofprintlayers; k++)
	{
		printcenter(2) = printcenter(2)+(printheight/(double) numberofprintlayers);
		calculatePrintShadowData();
		calculateInterferenceWithPrintParallelogram();
		rescur[0] = charparasizes[printparasizes[0][0]][0];
		rescur[1] = charparasizes[printparasizes[0][1]][1];
		multparasizes.push_back(rescur);	
	}
	
	printcenter(2) = printcenterbasez;
	multlayermode = false;
	return true;
}

/*! visualize multiple layers of printable parallelograms
 */
void CInterference::visualizeInterferenceCablePrintParalellogramMultipleLayers()
{
	looplines.clear();
	printshapes.clear();
	layerconnections.clear();
	updateClippingPlanes();

	calculateInterferenceCablePrintParalellogramMultipleLayers();

	int printParaVisNumberOld = printParaVisNumber%(int)printparasizes.size();
	printParaVisNumber = 0;	

	Vector3d printcenterprevious;
	const double printcenterbasez = printcenter(2);

	for (int k=0; k < (int) multparasizes.size(); k++)
	{
		vector<Vector3d> printloop;
		printloop.push_back(printcolor);
		printloop.push_back(printcenter+multparasizes[k][0]*d1+multparasizes[k][1]*d2);
		printloop.push_back(printcenter-multparasizes[k][0]*d1+multparasizes[k][1]*d2);
		printloop.push_back(printcenter-multparasizes[k][0]*d1-multparasizes[k][1]*d2);
		printloop.push_back(printcenter+multparasizes[k][0]*d1-multparasizes[k][1]*d2);
		printloop.push_back(printcenter+multparasizes[k][0]*d1+multparasizes[k][1]*d2);
		printshapes.push_back(printloop);
		printloop.erase(printloop.begin());
		looplines.push_back(printloop);
		printloop.clear();

		
		if (k>0)
		{
			vector<Vector3d> printloop;
			printloop.push_back(printcolor);
			printloop.push_back(printcenter+multparasizes[k][0]*d1+multparasizes[k][1]*d2);
			printloop.push_back(printcenterprevious+multparasizes[k][0]*d1+multparasizes[k][1]*d2);
			printloop.push_back(printcenter-multparasizes[k][0]*d1+multparasizes[k][1]*d2);
			printloop.push_back(printcenterprevious-multparasizes[k][0]*d1+multparasizes[k][1]*d2);
			printloop.push_back(printcenter-multparasizes[k][0]*d1-multparasizes[k][1]*d2);
			printloop.push_back(printcenterprevious-multparasizes[k][0]*d1-multparasizes[k][1]*d2);
			printloop.push_back(printcenter+multparasizes[k][0]*d1-multparasizes[k][1]*d2);
			printloop.push_back(printcenterprevious+multparasizes[k][0]*d1-multparasizes[k][1]*d2);
			printloop.push_back(printcenter+multparasizes[k][0]*d1+multparasizes[k][1]*d2);
			printloop.push_back(printcenterprevious+multparasizes[k][0]*d1+multparasizes[k][1]*d2);
			layerconnections.push_back(printloop);
			printloop.clear();
		}
		printcenterprevious = printcenter;
		printcenter(2) = printcenter(2)+(printheight/(double) numberofprintlayers);
	}
	
	printcenter(2) = printcenterbasez;
	printParaVisNumber = printParaVisNumberOld;
}


/*! check if parallelograms in the height interval [printcenter(2), printcenter(2)+printheight] can be calculated independently
 *
 *  \return: true, if layers can be calculated independently
 *			 false, otherwise
 */
bool CInterference::checkInterferenceWithPrintParaMultLayerCriteria()
{
	double a = d1(0);
	double b = d2(0);
	double c = d1(1);
	double d = d2(1);
	double det = (a*d)-(b*c);

	for (int i=max(0,firstCable); i < min(pRobot->getNow(),lastCable); i++)
	{
		Vector3d ai = pRobot->getBase(i);
		if (printcenter(2) < ai(2))
		{
			continue;
		}
		Vector3d bi = R0*pRobot->getPlatform(i);

		double hmin,hmax,cl0,cl1,cr0,cr1,dl0,dl1,dr0,dr1;

		hmin = printcenter(2)-ai(2);
		hmax = printcenter(2)-ai(2)+printheight;

		cl0 = (-pn(0)+bi(0))*hmin+ (ai(0)-printcenter(0)*(-pn(2)+bi(2)));
		cl1 = (-pn(1)+bi(1))*hmin+ (ai(1)-printcenter(1)*(-pn(2)+bi(2)));
		
		cr0=-(-pn(0)+bi(0)) + 2*(ai(0)-printcenter(0));
		cr1=-(-pn(1)+bi(1)) + 2*(ai(1)-printcenter(1));

		dl0 = ( d*cl0 -b*cl1 )/det;
		dl1 = (-c*cl0 +a*cl1 )/det;

		dr0 = ( d*cr0 -b*cr1 )/det;
		dr1 = (-c*cr0 +a*cr1 )/det;

		if ( (dl0*dr0 < 0) || (dl0*dr0 < 0) )
		{
			return false;
		}
	
		cl0 = (-pn(0)+bi(0))*hmax+ (ai(0)-printcenter(0)*(-pn(2)+bi(2)));
		cl1 = (-pn(1)+bi(1))*hmax+ (ai(1)-printcenter(1)*(-pn(2)+bi(2)));

		dl0 = ( d*cl0 -b*cl1 )/det;
		dl1 = (-c*cl0 +a*cl1 )/det;

		if ( (dl0*dr0 < 0) || (dl0*dr0 < 0) )
		{
			return false;
		}
	}
	return true;
}

/*! check if superellipses in the height interval [printcenter(2), printcenter(2)+printheight] can be calculated independently
 *
 *  \return: true, if layers can be calculated independently
 *			 false, otherwise
 */
bool CInterference::checkInterferenceWithPrintEllipseMultLayerCriteria()
{
	const double a = d1(0);
	const double b = d2(0);
	const double c = d1(1);
	const double d = d2(1);
	const double det = (a*d)-(b*c);
	const double deltal = 0.01;
	
	for (int i=max(0,firstCable); i < min(pRobot->getNow(),lastCable); i++)
	{
		Vector3d ai = pRobot->getBase(i);
		if (printcenter(2) < ai(2))
		{
			continue;
		}
		Vector3d bi = R0*pRobot->getPlatform(i);

		double hmin = printcenter(2)-ai(2);
		double hmax = printcenter(2)-ai(2)+printheight;

		double rbz = -pn(2)+bi(2);
		double lmin = rbz/(rbz+hmax);

		double clmin0=-pn(0)+bi(0)+ lmin*(ai(0)-printcenter(0)-(-pn(0)+bi(0)));
		double clmin1=-pn(1)+bi(1)+ lmin*(ai(1)-printcenter(1)-(-pn(1)+bi(1)));

		double d1 = abs(( d*clmin0 -b*clmin1 )/det);
		double d2 = abs((-c*clmin0 +a*clmin1 )/det);

		double acurp = pow(d1,pEllipse)+pow(d2,pEllipse);
		double acurlmin = pow(acurp,1/pEllipse);

		lmin = lmin+deltal;

		clmin0=-pn(0)+bi(0)+ lmin*(ai(0)-printcenter(0)-(-pn(0)+bi(0)));
		clmin1=-pn(1)+bi(1)+ lmin*(ai(1)-printcenter(1)-(-pn(1)+bi(1)));

		d1 = abs(( d*clmin0 -b*clmin1 )/det);
		d2 = abs((-c*clmin0 +a*clmin1 )/det);

		acurp = pow(d1,pEllipse)+pow(d2,pEllipse);
		double acurlmindl = pow(acurp,1/pEllipse);
		double factor = (rbz+hmax)/(rbz+2*hmax);
		double diffquot = (acurlmindl-acurlmin)/deltal;

		if (diffquot+factor*acurlmin < 0)
		{
			return false;
		}
	}
	return true;
}

/*! write robot geometry into .csv file
 *  In each line of the output file three coordinates of a vector are saved.
 *  First the prximal, then the distal anchor points and finally the printing nozzel are saved in this pattern.
 *
 *	\param [in] filename: filename of .csv output file
 *
 *  \return: true
 */
bool CInterference::writeRobotGeometryToCSV(const string& filename)
{
	ofstream outputfile;
	outputfile.open(filename);
	for (int i=max(0,firstCable); i < min(pRobot->getNow(),lastCable); i++)
	{
		Vector3d ai = pRobot->getBase(i);	
		outputfile << ai(0) << "," << ai(1) << "," << ai(2) << endl;
	}
	for (int i=max(0,firstCable); i < min(pRobot->getNow(),lastCable); i++)
	{
		Vector3d bi = R0*pRobot->getPlatform(i);
		outputfile << bi(0) << "," << bi(1) << "," << bi(2) << endl;
	}
	outputfile << pn(0) << "," << pn(1) << "," << pn(2) << endl;
	outputfile.close();
	return true;
}

/*! compute largest printable star-shaped polygon 
 */
void CInterference::calculateInterferenceWithPrintStarShape()
{
	const int spokes = 360;                                //!< number of spokes 
	const int refinements = 1;
	const double deltaphi = 2*MO_PI/(double)spokes;

	//! Bounding box for printable star-shaped polygon
	const double BoundingMaxX = xmax;
	const double BoundingMinX = xmin;
	const double BoundingMinY = xmin;
	const double BoundingMaxY = xmax;

	//! initialize queue for spokes
	deque<int> templatespoke;
	for (int i=0; i < spokes; i++)
	{
		templatespoke.push_back(i);
	}
	queue<int> templatespokequeue(templatespoke);

	pEllipse = 2;
	Vector3d d1save = d1;
	Vector3d d2save = d2;
	d1 << 1,0,0;
	d2 << 0,1,0;

	calculateInterferenceWithPrintEllipse();

	d1 = d1save;
	d2 = d2save;

	Vector3d d1rot = Matrix3d::ZRotationMatrix3d(MO_PI/2)*d1;

	double radius = printellipsesize/d1.norm();
	radius = min(radius,BoundingMaxX-printcenter(0));
	radius = min(radius,BoundingMaxY-printcenter(1));
	radius = min(radius,printcenter(0)-BoundingMinX);
	radius = min(radius,printcenter(1)-BoundingMinY);
	
	spokelist.clear();
	for (int i=0; i < spokes; i++)
	{
		spokelist.push_back(Matrix3d::ZRotationMatrix3d(i*deltaphi)*d1*radius);
	}

	double maxspoke = radius;				
	double deltaspoke = maxspoke/200;	
	int spokepointer = 0;

	//! start extending the spokes
	for (int ref=0;ref < refinements;ref++)
	{	
		//! start refinement step by calculating the lengthunit to be added to the spokes 
		double delspoke = deltaspoke/pow(2.0,(double) ref);
		int selectspoke = 0;
		bool collision = false;
		queue<int> spokequeue = templatespokequeue;

		//! iterate over the spokes
		while (spokequeue.empty() == false)
		{
			//! select spoke and assume there is no collision
			collision = false;
			selectspoke = spokequeue.front();
			Vector3d newspoke = spokelist[selectspoke]+delspoke*spokelist[selectspoke].normalized();
		
			//! try to increase the length of the selected spoke
			for (int i=0; i  < (int) shadowcenters.size(); i++)
			{
				Vector3d dist = shadowcenters[i]+newspoke*shadowscalings[i]-printcenter; //!< distance from extended shadowspoke to the printcenter of the printable shape
				
				//! check if dist vector is inside the current printable shape
				if (dist.norm() < maxspoke)
				{
					//! select relevant rays of printcenter
					double eta = atan2(d1rot.dot(dist),d1.dot(dist));
					if (eta < 0)
					{
						eta = 2*MO_PI+eta;
					}
					int relevantrayindex1 = (int) (eta/deltaphi);
					int relevantrayindex2 = relevantrayindex1+1;
					if (relevantrayindex2 == spokes)
					{
						relevantrayindex2 = 0;
					}

					double beta = eta-relevantrayindex1*deltaphi;
					double d = (spokelist[relevantrayindex1]-spokelist[relevantrayindex2]).norm();
					double gamma = asin(spokelist[relevantrayindex2].norm()*sin(deltaphi)/d);
					double n = spokelist[relevantrayindex1].norm()*sin(gamma)/sin(MO_PI-beta-gamma);

					if (dist.norm() < n)
					{				
						collision = true; //! Collsion occurs!
					}
				}

				dist = printcenter+newspoke-shadowcenters[i];
				if (dist.norm() < maxspoke*shadowscalings[i])
				{
					//calculate n and compare with dist, select relevant rays of printcenter
					double eta = atan2(d1rot.dot(dist),d1.dot(dist));
					if (eta < 0)
					{
						eta = 2*MO_PI+eta;
					}
					int relevantrayindex1 = (int) (eta/deltaphi);
					int relevantrayindex2 = relevantrayindex1+1;
					if (relevantrayindex2 == spokes)
					{
						relevantrayindex2 = 0;
					}

					double beta = eta-relevantrayindex1*deltaphi;
					double d = shadowscalings[i]*(spokelist[relevantrayindex1]-spokelist[relevantrayindex2]).norm();
					double gamma = asin(spokelist[relevantrayindex2].norm()*shadowscalings[i]*sin(deltaphi)/d);
					double n = spokelist[relevantrayindex1].norm()*shadowscalings[i]*sin(gamma)/sin(MO_PI-beta-gamma);
						
					if (dist.norm() < n)
					{
						collision = true; //! Collsion occurs!
					}
				}
			}

			Vector3d spokepoint = printcenter+newspoke;

			if (spokepoint(0)>BoundingMaxX || spokepoint(0) < BoundingMinX || spokepoint(1)>BoundingMaxY || spokepoint(1) < BoundingMinY)
			{
				collision = true;
			}
			if (collision == true)
			{
				spokequeue.pop();
				
			} else {
				spokelist[selectspoke] = newspoke;
				maxspoke = max(maxspoke,spokelist[selectspoke].norm());
				spokequeue.pop();
				spokequeue.push(selectspoke);					
			}
		}
	}
}

/*! compute largest printable superellipse
 */
void CInterference::calculateInterferenceWithPrintEllipse()
{
	const double eps = 0.0001;
    const double a = d1(0);
	const double b = d2(0);
	const double c = d1(1);
	const double d = d2(1);
	const double det = (a*d)-(b*c);
	
	printellipsesize = 500;

	for (int i=0; i  < (int) shadowcenters.size(); i++)
		{
			Vector3d diff = shadowcenters[i]-printcenter;

			double d1 = abs(( d*diff(0) -b*diff(1))/det)/(1+shadowscalings[i]);
			double d2 = abs((-c*diff(0) +a*diff(1))/det)/(1+shadowscalings[i]);


			double acurp = pow(d1,pEllipse)+pow(d2,pEllipse);
			double acur = pow(acurp,1/pEllipse);
			printellipsesize = min(printellipsesize,acur);
		}
}

/*! compute shadow centers and shadow scaling factors
 */
void CInterference::calculatePrintShadowData()
{
	shadowcenters.clear();
	shadowscalings.clear();

	for (int i=max(0,firstCable); i < min(pRobot->getNow(),lastCable); i++)
	{
		Vector3d ai = pRobot->getBase(i);
		if (printcenter(2) < ai(2))
		{
			continue;
		}
		Vector3d bi = R0*pRobot->getPlatform(i);

		double rbz=-pn(2)+bi(2);
		double h = printcenter(2)-ai(2);
		double shadowscaling = h/(rbz+h);
		Vector3d p = printcenter-pn+bi;
		Vector3d shadowcenter = p-(p-ai)*rbz/(rbz+h);

		shadowcenters.push_back(shadowcenter);
		shadowscalings.push_back(shadowscaling);
	}
}


/*! struct for sorting the list of printable paralellograms in descending order according to their area
 */
struct sortstruct
{
	// sortstruct needs to know its containing object
	CInterference* m;
	sortstruct(CInterference* p) : m(p) {};
 
	// this is our sort function, which makes use
	// of some non-static data (sortascending)
	bool operator() ( vector<int> a, vector<int> b )
	{
		return m->charparasizes[a[0]][0]*m->charparasizes[a[1]][1]>m->charparasizes[b[0]][0]*m->charparasizes[b[1]][1];
	}
};

/*! compute lists of characteristic and printable paralellograms 
 */
void CInterference::calculateInterferenceWithPrintParallelogram()
{
	const double eps = 0.0001;
	const double a = d1(0);
	const double b = d2(0);
	const double c = d1(1);
	const double d = d2(1);
	const double det = (a*d)-(b*c);

	charparasizes.clear();

	for (int i=0; i  < (int) shadowcenters.size(); i++)
	{
		Vector3d diff = shadowcenters[i]-printcenter;
		double res1 = abs(( d*diff(0) -b*diff(1))/det)/(1+shadowscalings[i]);
		double res2 = abs((-c*diff(0) +a*diff(1))/det)/(1+shadowscalings[i]);

		vector<double> rescur(2,0);
		rescur[0] = res1;
		rescur[1] = res2;
		charparasizes.push_back(rescur);
	}

	printparasizes.clear();
	
	vector<double> rescur(2,0);
	rescur[0] = TriangleLength;
	rescur[1] = TriangleLength;
	charparasizes.push_back(rescur);

	for (int i=0; i < (int)(charparasizes.size()); i++)
	{
		rescur[0] = charparasizes[i][0];

		for (int j=0; j < (int)(charparasizes.size()); j++)
		{
			rescur[1] = charparasizes[j][1];
			bool printable = true;
			for (int k=0; k < (int)(charparasizes.size())-1; k++)
			{
				if (charparasizes[k][0] < rescur[0]-eps && charparasizes[k][1] < rescur[1]-eps)
				{
					printable = false;
				}		

				if (multlayermode)
				{
					if ( (notexpandable[i] == 1 &&  charparasizes[i][1] < rescur[1]-eps) || (notexpandable[j] == 0 &&  charparasizes[j][0] < rescur[0]-eps) )
					{
						printable = false;
					}	
				}
			}
			if (printable == true)
			{
				vector<int> printcomb(2,0);
				printcomb[0] = i;
				printcomb[1] = j;
				printparasizes.push_back(printcomb);
			}
		}
	}
	sortstruct s(this);
	sort(printparasizes.begin(),printparasizes.end(),s);
}

/*! write parallelogram data into .csv file
 */
bool CInterference::writeParallelogramToCSV(const string filename)
{
	ofstream outputfile;

	outputfile.open(filename);

	outputfile << printcenter(0) << "," << printcenter(1) << "," << printcenter(2) << endl;
	outputfile << d1(0) << "," << d1(1) << "," << d1(2) << endl;
	outputfile << d2(0) << "," << d2(1) << "," << d2(2) << endl;
	outputfile << (int) shadowcenters.size() << "," << (int)charparasizes.size() << "," << (int)printparasizes.size() << endl;

	for (int i=0; i <(int) shadowcenters.size(); i++)
	{
		outputfile << shadowcenters[i](0) << "," << shadowcenters[i](1) << "," << shadowcenters[i](2) << "," << shadowscalings[i] << endl;
	}

	for (int k=0; k < (int)charparasizes.size(); k++)
	{
		outputfile << charparasizes[k][0] << "," << charparasizes[k][1] << endl;
	}

	for (int k=0; k < (int)printparasizes.size(); k++)
	{
		outputfile << printparasizes[k][0] << "," << printparasizes[k][1] << endl;
	}

	outputfile.close();

	return true;
}

/*! write multiple layer parallelogram data into .csv file
 */
bool CInterference::writeParallelogramMultipleLayerToCSV(const string filename)
{
	ofstream outputfile;

	outputfile.open(filename);

	outputfile << printcenter(0) << "," << printcenter(1) << "," << printcenter(2) << endl;
	outputfile << d1(0) << "," << d1(1) << "," << d1(2) << endl;
	outputfile << d2(0) << "," << d2(1) << "," << d2(2) << endl;
	outputfile << (int) multparasizes.size() << "," << printheight << "," << numberofprintlayers << endl;

	for (int k=0; k < (int)multparasizes.size(); k++)
	{
		outputfile << multparasizes[k][0] << "," << multparasizes[k][1] << endl;
	}

	outputfile.close();

	return true;
}

/*! draw selected printable parallelogram from list
 */
void CInterference::drawParallelogram(){

	printParaVisNumber = printParaVisNumber%(int)printparasizes.size();

	double res1 = charparasizes[printparasizes[printParaVisNumber][0]][0];
	double res2 = charparasizes[printparasizes[printParaVisNumber][1]][1];

	vector<Vector3d> printloop;
	printloop.push_back(printcolor);
	printloop.push_back(printcenter+res1*d1+res2*d2);
	printloop.push_back(printcenter-res1*d1+res2*d2);
	printloop.push_back(printcenter-res1*d1-res2*d2);
	printloop.push_back(printcenter+res1*d1-res2*d2);
	printloop.push_back(printcenter+res1*d1+res2*d2);
	printshapes.push_back(printloop);
	printloop.clear();

	if (displayshadows)
	{
		for (int i=0; i <(int) shadowcenters.size(); i++)
		{
			vector<Vector3d> printloop;
			printloop.push_back(shadowcolor);
			printloop.push_back(shadowcenters[i]+shadowscalings[i]*(+res1*d1+res2*d2));
			printloop.push_back(shadowcenters[i]+shadowscalings[i]*(-res1*d1+res2*d2));
			printloop.push_back(shadowcenters[i]+shadowscalings[i]*(-res1*d1-res2*d2));
			printloop.push_back(shadowcenters[i]+shadowscalings[i]*(+res1*d1-res2*d2));
			printloop.push_back(shadowcenters[i]+shadowscalings[i]*(+res1*d1+res2*d2));
			printshapes.push_back(printloop);
			printloop.clear();
		}
	}
}

/*! draw printable star-shaped polygon
 */
void CInterference::drawStarShape(){

	vector<Vector3d> fan;
	fan.push_back(printcolor);
	fan.push_back(printcenter);
	for (int i=0; i<(int)spokelist.size(); i++)
	{
		fan.push_back(printcenter+spokelist[i]);
	}
	fan.push_back(printcenter+spokelist[0]);
	printshapes.push_back(fan);

	if (displayshadows == true)
	{ 
		//displayshadows = false;
		for (int i=0; i  < (int) shadowcenters.size(); i++)
		{
			fan.clear();
			fan.push_back(shadowcolor);
			fan.push_back(shadowcenters[i]);
	
			for (int j=0; j < (int)spokelist.size(); j++)
			{
				fan.push_back(shadowcenters[i]+spokelist[j]*shadowscalings[i]);
			}

			fan.push_back(shadowcenters[i]+spokelist[0]*shadowscalings[i]);
			printshapes.push_back(fan);
		}
	}
}

/*! draw printable superellipse connection between two layers
 */
void CInterference::drawEllipseLayerConnection(){
			
	const int steps1 = 100;
	const double delta1 = 1/(double)steps1;
	const double dh = (printheight/(double) numberofprintlayers);
	
	looplines.clear();
	layerconnections.clear();

	vector<Vector3d> loop;

	for (int i=0; i < steps1; i++)
	{
		double y = pow(delta1*i,1/pEllipse);
		double x = pow(1-delta1*i,1/pEllipse);
		loop.push_back(+x*d1+y*d2);

	}
	for (int i=0; i < steps1; i++)
	{
		double y = pow(1-delta1*i,1/pEllipse);
		double x = pow(delta1*i,1/pEllipse);
		loop.push_back(-x*d1+y*d2);

	}
	for (int i=0; i < steps1; i++)
	{
		double y = pow(delta1*i,1/pEllipse);
		double x = pow(1-delta1*i,1/pEllipse);
		loop.push_back(-x*d1-y*d2);

	}
	for (int i=0; i < steps1; i++)
	{
		double y = pow(1-delta1*i,1/pEllipse);
		double x = pow(delta1*i,1/pEllipse);
		loop.push_back(+x*d1-y*d2);
	}
	
	vector<Vector3d> printloop;
	for (int k=1; k < (int)(loop.size()); k++)
	{
		printloop.push_back(printcenter+loop[k]*printellipsesize);
	}
	looplines.push_back(printloop);
	printloop.clear();

	Vector3d printcenterlower = printcenter;
	printcenterlower(2) = printcenterlower(2)-dh;

	for (int k=1; k < (int)(loop.size()); k++)
	{
		printloop.push_back(printcenterlower+loop[k]*printellipsesize);
	}
	looplines.push_back(printloop);
	printloop.clear();
	printloop.push_back(printcolor);
	for (int k=1; k < (int)(loop.size()); k++)
	{
		printloop.push_back(printcenter     +loop[k]*printellipsesize);
		printloop.push_back(printcenterlower+loop[k]*printellipsesize);
	}
	layerconnections.push_back(printloop);
	printloop.clear();
}

/*! draw printable superellipse
 */
void CInterference::drawEllipse()
{			
	const int steps1 = 100;
	const double delta1 = 1/(double)steps1;

	vector<Vector3d> loop;

	for (int i=0; i < steps1; i++)
	{
		double y = pow(delta1*i,1/pEllipse);
		double x = pow(1-delta1*i,1/pEllipse);
		loop.push_back(+x*d1+y*d2);

	}
	for (int i=0; i < steps1; i++)
	{
		double y = pow(1-delta1*i,1/pEllipse);
		double x = pow(delta1*i,1/pEllipse);
		loop.push_back(-x*d1+y*d2);

	}
	for (int i=0; i < steps1; i++)
	{
		double y = pow(delta1*i,1/pEllipse);
		double x = pow(1-delta1*i,1/pEllipse);
		loop.push_back(-x*d1-y*d2);

	}
	for (int i=0; i < steps1; i++)
	{
		double y = pow(1-delta1*i,1/pEllipse);
		double x = pow(delta1*i,1/pEllipse);
		loop.push_back(+x*d1-y*d2);
	}
	
	vector<Vector3d> printloop;
	printloop.push_back(printcolor);
	for (int k=1; k < (int)(loop.size()); k++)
	{
		printloop.push_back(printcenter+loop[k]*printellipsesize);
	}
	printshapes.push_back(printloop);
	printloop.clear();

	if (displayshadows)
	{
		for (int i=0; i <(int) shadowcenters.size(); i++)
		{
			vector<Vector3d> shadowloop;
			shadowloop.push_back(shadowcolor);
			for (int k=1; k < (int)(loop.size()); k++)
			{
				shadowloop.push_back(loop[k]*printellipsesize*shadowscalings[i]+shadowcenters[i]);
			}
			printshapes.push_back(shadowloop);
			shadowloop.clear();
		}
	}	
}


/*! write superellipse data into .csv file
 */
bool CInterference::writeEllipseToCSV(const string filename)
{
	ofstream outputfile;
	outputfile.open(filename);

	outputfile << printcenter(0) << "," << printcenter(1) << "," << printcenter(2) << endl;
	outputfile << d1(0) << "," << d1(1) << "," << d1(2) << endl;
	outputfile << d2(0) << "," << d2(1) << "," << d2(2) << endl;
	outputfile << pEllipse << "," << printellipsesize   << endl;

	for (int i=0; i <(int) shadowcenters.size(); i++)
	{
		outputfile << shadowcenters[i](0) << "," << shadowcenters[i](1) << "," << shadowcenters[i](2) << "," << shadowscalings[i] << endl;
	}

	outputfile.close();
	
	return true;
}

/*! write star-shape data into .csv file
 */
bool CInterference::writeStarShapeToCSV(const string filename)
{
	ofstream outputfile;
	outputfile.open(filename);

	outputfile << printcenter(0) << "," << printcenter(1) << "," << printcenter(2) << "," << (int) spokelist.size() << endl;

	for (int i=0; i <(int) spokelist.size(); i++)
	{
		outputfile << spokelist[i](0) << "," << spokelist[i](1) << endl;
	}

	for (int i=0; i <(int) shadowcenters.size(); i++)
	{
		outputfile << shadowcenters[i](0) << "," << shadowcenters[i](1) << "," << shadowcenters[i](2) << "," << shadowscalings[i] << endl;
	}

	outputfile.close();
	
	return true;
}

/*! set vector of printing nozzle
 *	The vector points from the center of the platform to the head of the printing nozzle
 */
void CInterference::setPn(Vector3d p)
{
	pn = p;
}

/*! set cable-cable display setting
 */
void CInterference::setCableCableSetting(bool setting)
{
	displayAllCableCableInterferenceSets = setting;
}

/*! set cable-print display setting
 */
void CInterference::setPrintDisplayShadows(bool setting)
{
	displayshadows = setting;
}

/*! set direction vectors for printing parallelograms and superellipses
 */
void CInterference::setPrintDir(Vector3d dir1,Vector3d dir2)
{
	d1 = dir1;
	d2 = dir2;
}

/*! set center location for printing parallelograms and superellipses
 */
void CInterference::setPrintCenter(Vector3d center)
{
	printcenter = center;
}

/*! set center location for cable-platform calculation
 */
void CInterference::setPlatformCenter(Vector3d center)
{
	platformcenterposition = center;
}

/*! set discretization parameter for cable-cable interference
 */
void CInterference::setDeltaAlpha(double a)
{
	deltaalpha = a;
}

/*! set discretization parameter for cable-cable interference
 */
void CInterference::setDeltaEta(double h)
{
	deltaeta = h;
}

/*! set discretization parameter for cable-cable interference
 */
void CInterference::setStepsCircle(int h)
{
	stepscircle = h;
}

/*! set exponent parameter for printing superellipses
 */
void CInterference::setPrintHeight(double h)
{
	printheight = h;
}

/*! set exponent parameter for printing superellipses
 */
void CInterference::setEllipseExp(double t)
{
	pEllipse = t;
}

/*! set refinement parameter for cable-platform ray data structure
 */
void CInterference::setRefinement(int r)
{
	refinement = r;
}

/*! set searchsteps parameter for cable-platform calculation
 */
void CInterference::setSearchSteps(int s)
{
	searchsteps = s;
}

/*! set printParaVisNumber parameter for selecting visualized printable parallelogram
 */
void CInterference::setPrintParaVisNumber(int s)
{
	printParaVisNumber = s;
}

/*! set numberofprintlayers parameter for selecting the number of layers in the printing height interval
 */
void CInterference::setNumberOfPrintLayers(int s)
{
	numberofprintlayers = s;
}

/*! check there is a cable-platform collision at position x and any orientation from the orientation set
 *	\param [in] x: platform position to be checked
 *
 *  \return: true, no collision is found at x
 *			 false, otherwise
 */
bool CInterference::cablePlatformPosistionCheck(const Vector3d& x)
{
	Matrix3d R = R0;
	const int normalblendsteps = 10;
	
	for (int i=max(0,firstCable); i < min(pRobot->getNow(),lastCable); i++)
	{
		Vector3d ai = pRobot->getBase(i);	
		Vector3d bi = pRobot->getPlatform(i);
		Vector3d aix = ai - x;
		double aixnorm = aix.norm();

		for (int l=0; l < (int) cablecones[i].size(); l++)
		{
			bool outsideofcone = false;
			vector<Vector3d> coneboundary = cablecones[i][l];
			coneboundary.insert(coneboundary.begin(),cablecones[i][l][(int) cablecones[i][l].size()-1]);
			coneboundary.push_back(cablecones[i][l][0]);

			for (int j=1; j <= (int) cablecones[i][l].size(); j++)
			{
				Vector3d n1=-coneboundary[j-1].cross(coneboundary[j  ]);
				Vector3d n2=-coneboundary[j  ].cross(coneboundary[j+1]);

				for (int s=0; s <= normalblendsteps; s++)
				{
					Vector3d ns = ( n1*s + n2*(normalblendsteps-s) )/normalblendsteps;
					ns.normalize();
					bool allRoutside = true;
					double nsbi = ns.dot(bi);
					double beta = acos(aix.dot(ns)/aixnorm);
					if (cos(beta-etaMax)*aixnorm>nsbi)
 					{
 						allRoutside = false;
 					}
					if (cos(beta+etaMax)*aixnorm>nsbi)
 					{
 						allRoutside = false;
 					}
					double angleintervalstart = fmod(beta-etaMax,2*MO_PI);
 					double angleintervalend   = fmod(beta+etaMax,2*MO_PI); 
 					if (angleintervalstart < 0)
 					{
 						angleintervalstart = 2*MO_PI+angleintervalstart;
 					}
 					if (angleintervalend < angleintervalstart)
 					{
 						if (aixnorm>nsbi)
 						{
 							allRoutside = false;
 						}
 					}
					if (allRoutside)
					{
						outsideofcone = true;
						break;
					}
				}
				if (outsideofcone)
				{
					break;
				}
			}
			if (outsideofcone == false)
			{
				return false;
			}
		}
	}
	return true;
}

/*! calculate the convex collision cone partitions based on the platform data in platformstldata
 *	Th data of the cones is saved in the vector cablecones
 */
void CInterference::setupCablePlatformCollisionCones()
{
	const int sectornumber = 36;
	const double deltasector = 2*MO_PI/(double)sectornumber;
	const int edgesteps = 10;
	const double conelength = 1;
	
	const Vector3d e1(1,0,0);
	const Vector3d e2(0,1,0);

	vector< Vector3d > bin;
	vector< Vector3d > birn1;
	vector< Vector3d > birn2;

	cablecones.clear();

	for (int i=max(0,firstCable); i < min(pRobot->getNow(),lastCable); i++)
	{
		Vector3d bi = pRobot->getPlatform(i);
		Vector3d bin = bi.normalized();
		Vector3d birn1;

		if (abs(e1.dot(bi)) < abs(e2.dot(bi)))
		{
			birn1 = e1-e1.dot(bin)*bin;
		} else {
			birn1 = e2-e2.dot(bin)*bin;
		}

		birn1.normalize();
		Vector3d birn2 = bin.cross(birn1);
		birn2.normalize();

		double sectorangle [sectornumber];
		int k;

		for (k=0; k < sectornumber; k++)
		{
			sectorangle[k] = 7;
		}

		int sz = (int)platformstldata.size();

		for (int s=0; s < sz; s=s+3)
		{
			Vector3d ray1 = platformstldata[s]-bi;
			Vector3d ray2 = platformstldata[s+1]-bi;
			Vector3d ray3 = platformstldata[s+2]-bi;
			ray1.normalize();
			ray2.normalize();
			ray3.normalize();

			double eta1 = atan2(ray1.dot(birn2),ray1.dot(birn1));
			if (eta1 < 0)
			{
				eta1 = 2*MO_PI+eta1;
			}

			double eta2 = atan2(ray2.dot(birn2),ray2.dot(birn1));
			if (eta2 < 0)
			{
				eta2 = 2*MO_PI+eta2;
			}

			double eta3 = atan2(ray3.dot(birn2),ray3.dot(birn1));
			if (eta3 < 0)
			{
				eta3 = 2*MO_PI+eta3;
			}

			int secindex1 = (int) (eta1/deltasector);
			sectorangle[secindex1] = min(sectorangle[secindex1], acos(bin.dot(ray1)) );

			int secindex2 = (int) (eta2/deltasector);
			sectorangle[secindex2] = min(sectorangle[secindex2], acos(bin.dot(ray2)) );

			int secindex3 = (int) (eta3/deltasector);
			sectorangle[secindex3] = min(sectorangle[secindex3], acos(bin.dot(ray3)) );

			if (abs(eta1-eta2)>deltasector)
			{
				for (int j=1; j < edgesteps; j++)
				{
					double lambda = (double)j/(double)edgesteps;
					ray1 = lambda * platformstldata[s] + (1-lambda) * platformstldata[s+1] -bi;
					ray1.normalize();

					eta1 = atan2(ray1.dot(birn2),ray1.dot(birn1));
					if (eta1 < 0)
					{
						eta1 = 2*MO_PI+eta1;
					}
					secindex1 = (int) (eta1/deltasector);
					sectorangle[secindex1] = min(sectorangle[secindex1], acos(bin.dot(ray1)) );
				}
			}

			if (abs(eta1-eta3)>deltasector)
			{
				for (int j=1; j < edgesteps; j++)
				{
					double lambda = (double)j/(double)edgesteps;
					ray1 = lambda * platformstldata[s] + (1-lambda) * platformstldata[s+2] -bi;
					ray1.normalize();

					eta1 = atan2(ray1.dot(birn2),ray1.dot(birn1));
					if (eta1 < 0)
					{
						eta1 = 2*MO_PI+eta1;
					}
					secindex1 = (int) (eta1/deltasector);
					sectorangle[secindex1] = min(sectorangle[secindex1], acos(bin.dot(ray1)) );
				}
			}

			if (abs(eta2-eta3)>deltasector)
			{
				for (int j=1; j < edgesteps; j++)
				{
					double lambda = (double)j/(double)edgesteps;
					ray1 = lambda * platformstldata[s+1] + (1-lambda) * platformstldata[s+2] -bi;
					ray1.normalize();

					eta1 = atan2(ray1.dot(birn2),ray1.dot(birn1));
					if (eta1 < 0)
					{
						eta1 = 2*MO_PI+eta1;
					}
					secindex1 = (int) (eta1/deltasector);
					sectorangle[secindex1] = min(sectorangle[secindex1], acos(bin.dot(ray1)) );
				}
			}
		}
		//assemble cone sets
		vector< vector<Vector3d> > cableconeset;
		vector<Vector3d> cablecone;
		cablecone.push_back(-bin*conelength);

		k = sectornumber-1;
		Vector3d rotaxis = NalphaRotation(bin,k*deltasector+deltasector/2)*birn2;
		Vector3d conelast2 = NalphaRotation(rotaxis,sectorangle[k])*bin;
		cablecone.push_back(conelast2*conelength);

		k = 0;
		rotaxis = NalphaRotation(bin,k*deltasector+deltasector/2)*birn2;
		Vector3d conelast1 = NalphaRotation(rotaxis,sectorangle[k])*bin;
		cablecone.push_back(conelast1*conelength);


		for (k=1; k < sectornumber; k++)
		{
			Vector3d rotaxis = NalphaRotation(bin,k*deltasector+deltasector/2)*birn2;
			Vector3d cone = NalphaRotation(rotaxis,sectorangle[k])*bin;

			if (  ( (int)cablecone.size() == 0 && k> (int) (sectornumber/2) ) || ((conelast2.cross(conelast1)).dot(cone)>0 )  ) 
			{		
				cableconeset.push_back(cablecone);
				cablecone.clear();
				cablecone.push_back(-bin*conelength);
				cablecone.push_back(conelast1*conelength);
			}
			cablecone.push_back(cone*conelength);

			conelast2 = conelast1;
			conelast1 = cone;
		}
		cableconeset.push_back(cablecone);
		cablecones.push_back(cableconeset);
		cableconeset.clear();
		cablecone.clear();
	}
}

/*! calculates to the hull of the cable-platform collision free workspace
 */
void CInterference::calculateCablePlatform()
{
	const int raynumber  = (int) cableplatformraylist.size();

	for (int i=0; i < raynumber; i++)
	{
		Vector3d ray = cableplatformraylist[i];
		ray.normalize();
		double intervalmin = 0;
		double intervalmax = TriangleLength;

		for (int st=0; st < searchsteps ;st++)
		{
			double intervalmid = (intervalmin+intervalmax)/2;
			Vector3d x = platformcenterposition+ray*intervalmid;

			//bool middleok = cablePlatformPoseCheck(x,R0);
			bool middleok = cablePlatformPosistionCheck(x);

			if (middleok)
			{
				intervalmin = intervalmid;
			}
			else{
				intervalmax = intervalmid;
			}	
		}
		cableplatformraylist[i] = ray*intervalmin;
	}
}

//! copies triangles from the cable-platform collision free workspace hull into a list to be visualized by Shapes.h 
void CInterference::drawCablePlatformCollisionFreeSpace()
{	
	int sz = (int)cableplatformtrianglelist.size();
	for (int i=0; i < sz; i=i+1)
	{
		redtriangles.push_back(cableplatformraylist[cableplatformtrianglelist[i][0]]);
		redtriangles.push_back(cableplatformraylist[cableplatformtrianglelist[i][1]]);
		redtriangles.push_back(cableplatformraylist[cableplatformtrianglelist[i][2]]);
	}
}

//! copies triangles from the stl data of the platform into a list to be visualized by Shapes.h 
void CInterference::drawplatformstldata()
{
	int sz = (int)platformstldata.size();
	for (int i=0; i < sz; i=i+3)
	{
		redtriangles.push_back(platformstldata[i]);
		redtriangles.push_back(platformstldata[i+1]);
		redtriangles.push_back(platformstldata[i+2]);
	}
}

/*! write data of platform geometry into .csv file
 *	
 *  \param [in] filename: name of outputfile for .csv data. 
 *
 *  \return: true
 */
bool CInterference::writeCablePlatformPlatformToCSV(const string& filename)
{
	ofstream outputfile (filename);

	int sz = (int)platformstldata.size();
	for (int i=0; i < sz; i=i+3)
	{	
		outputfile << platformstldata[i](0)   << "," << platformstldata[i](1)   << "," << platformstldata[i](2)   << ","  ;
		outputfile << platformstldata[i+1](0) << "," << platformstldata[i+1](1) << "," << platformstldata[i+1](2) << ","  ;
		outputfile << platformstldata[i+2](0) << "," << platformstldata[i+2](1) << "," << platformstldata[i+2](2) << endl ;
	}					
	outputfile.close();

	return true;
}

/*! write data of cable-platform collision free workspace hull triangulation into .csv file
 *	
 *  \param [in] filename: name of outputfile for .csv data. 
 *
 *  \return: true
 */
bool CInterference::writeCablePlatformTrianglelistToCSV(const string& filename)
{
	ofstream outputfile(filename);

	for (int l=0; l< (int) cableplatformtrianglelist.size() ;l++)
	{	
		outputfile << cableplatformtrianglelist[l][0] << "," << cableplatformtrianglelist[l][1] << "," << cableplatformtrianglelist[l][2] << endl ;
	}					
	outputfile.close();

	return true;
}

/*! write data of cable-platform collision free workspace hull ray length into .csv file
 *	
 *  \param [in] filename: name of outputfile for .csv data. 
 *
 *  \return: true
 */
bool CInterference::writeCablePlatformRaysToCSV(const string& filename)
{
	ofstream outputfile(filename);

	for (int l=0; l < (int) cableplatformraylist.size(); l++)
	{	
		outputfile << cableplatformraylist[l](0) << "," << cableplatformraylist[l](1) << "," << cableplatformraylist[l](2) << endl ;
	}					
	outputfile.close();

	return true;
}

/*! write data of cable-platform collision cones into .csv file
 *	
 *  \param [in] filename: name of outputfile for .csv data. 
 *
 *  \return: true
 */
bool CInterference::writeCablePlatformConesToCSV(const string& filename)
{
	ofstream outputfile(filename);
	double factor = 0.2;

	for (int i=max(0,firstCable); i < min(pRobot->getNow(),lastCable); i++)
	{
		Vector3d bi = R0*pRobot->getPlatform(i);

		for (int l=0; l < (int) cablecones[i].size(); l++)
		{
			outputfile << i << "," << (int) cablecones[i][l].size() << endl;

			for (int j=1; j < (int) cablecones[i][l].size(); j++)
			{			
				Vector3d cray1 = bi+cablecones[i][l][j-1]*factor;
				Vector3d cray2 = bi+cablecones[i][l][j]*factor;

				outputfile << bi(0) << "," << bi(1) << "," << bi(2) << "," ;
				outputfile << cray1(0) << "," << cray1(1) << "," << cray1(2) << "," ;
				outputfile << cray2(0) << "," << cray2(1) << "," << cray2(2) << "," ;
			}

			Vector3d cray1 = bi+cablecones[i][l][(int) cablecones[i][l].size()-1]*factor;
			Vector3d cray2 = bi+cablecones[i][l][0]*factor;

			outputfile << bi(0) << "," << bi(1) << "," << bi(2) << "," ;
			outputfile << cray1(0) << "," << cray1(1) << "," << cray1(2) << "," ;
			outputfile << cray2(0) << "," << cray2(1) << "," << cray2(2)        ;
			outputfile << endl;
		}				
	}
	outputfile.close();

	return true;
}

//! copies triangles from the cable-platform collision cone data into a list to be visualized by Shapes.h 
void CInterference::drawCablePlatformCones()
{
	triangles.clear();		
	double factor = 0.2;

	for (int i=max(0,firstCable); i < min(pRobot->getNow(),lastCable); i++)
	{
		Vector3d bi = R0*pRobot->getPlatform(i);

		for (int l=0; l< (int) cablecones[i].size() ;l++)
		{
			for (int j=1; j < (int) cablecones[i][l].size(); j++)
			{			
				triangles.push_back(bi);
				triangles.push_back(bi+cablecones[i][l][j-1]*factor);
				triangles.push_back(bi+cablecones[i][l][j]  *factor);
			}
			triangles.push_back(bi);
			triangles.push_back(bi+cablecones[i][l][(int) cablecones[i][l].size()-1]*factor);
			triangles.push_back(bi+cablecones[i][l][0]  *factor);
		}
	}
}

/*! set up the data structure needed for the computation of the set of positions
 *  where no cable-platform collisions occur.
 */
void CInterference::newCablePlatformRayBall()
{
	cableplatformraylist.clear();
	cableplatformtrianglelist.clear();

	// setup the regular isocahedron
	const double raynorm = sqrt(5.0+sqrt(5.0))/sqrt(2.0);
	const double t = (1.0+sqrt(5.0))/2.0/raynorm;
	const double o = 1.0/raynorm;
	
	Vector3d ray;

	ray << -o, t, 0;
	cableplatformraylist.push_back(ray);
	ray <<  o,  t,  0;
	cableplatformraylist.push_back(ray);
	ray << -o, -t,  0;
	cableplatformraylist.push_back(ray);
	ray <<  o, -t,  0;
	cableplatformraylist.push_back(ray);
	
	ray <<  0, -o,  t;
	cableplatformraylist.push_back(ray);
	ray <<  0,  o,  t;
	cableplatformraylist.push_back(ray);
	ray <<  0, -o, -t;
	cableplatformraylist.push_back(ray);
	ray <<  0,  o, -t;
	cableplatformraylist.push_back(ray);
	
	ray <<  t,  0, -o;
	cableplatformraylist.push_back(ray);
	ray <<  t,  0,  o;
	cableplatformraylist.push_back(ray);
	ray << -t,  0, -o;
	cableplatformraylist.push_back(ray);
	ray << -t,  0,  o;
	cableplatformraylist.push_back(ray);

	vector<int> triangle(3,0);

	triangle[0] =   0; triangle[1] = 11; triangle[2] =  5; cableplatformtrianglelist.push_back(triangle);
	triangle[0] =   0; triangle[1] =  5; triangle[2] =  1; cableplatformtrianglelist.push_back(triangle);
	triangle[0] =   0; triangle[1] =  1; triangle[2] =  7; cableplatformtrianglelist.push_back(triangle);
	triangle[0] =   0; triangle[1] =  7; triangle[2] = 10; cableplatformtrianglelist.push_back(triangle);
	triangle[0] =   0; triangle[1] = 10; triangle[2] = 11; cableplatformtrianglelist.push_back(triangle);
													 
	triangle[0] =   1; triangle[1] =  5; triangle[2] =  9; cableplatformtrianglelist.push_back(triangle);
	triangle[0] =   5; triangle[1] = 11; triangle[2] =  4; cableplatformtrianglelist.push_back(triangle);
	triangle[0] =  11; triangle[1] = 10; triangle[2] =  2; cableplatformtrianglelist.push_back(triangle);
	triangle[0] =  10; triangle[1] =  7; triangle[2] =  6; cableplatformtrianglelist.push_back(triangle);
	triangle[0] =   7; triangle[1] =  1; triangle[2] =  8; cableplatformtrianglelist.push_back(triangle);
													 
	triangle[0] =   3; triangle[1] =  9; triangle[2] =  4; cableplatformtrianglelist.push_back(triangle);
	triangle[0] =   3; triangle[1] =  4; triangle[2] =  2; cableplatformtrianglelist.push_back(triangle);
	triangle[0] =   3; triangle[1] =  2; triangle[2] =  6; cableplatformtrianglelist.push_back(triangle);
	triangle[0] =   3; triangle[1] =  6; triangle[2] =  8; cableplatformtrianglelist.push_back(triangle);
	triangle[0] =   3; triangle[1] =  8; triangle[2] =  9; cableplatformtrianglelist.push_back(triangle);
													 
	triangle[0] =   4; triangle[1] =  9; triangle[2] =  5; cableplatformtrianglelist.push_back(triangle);
	triangle[0] =   2; triangle[1] =  4; triangle[2] = 11; cableplatformtrianglelist.push_back(triangle);
	triangle[0] =   6; triangle[1] =  2; triangle[2] = 10; cableplatformtrianglelist.push_back(triangle);
	triangle[0] =   8; triangle[1] =  6; triangle[2] =  7; cableplatformtrianglelist.push_back(triangle);
	triangle[0] =   9; triangle[1] =  8; triangle[2] =  1; cableplatformtrianglelist.push_back(triangle);

	// refine the triangles of the isocahedron by splitting them into 4 triangles
	for (int i=0; i<refinement; i++)
	{
		vector< vector<int> > newtrianglelist;
		vector< vector<int> > edgelist;
		
		int trianglenumber = (int) cableplatformtrianglelist.size();
		int rayindex;
		bool ismember;
		Vector3d middle;

		for (int j=0; j<trianglenumber; j++)
		{	
			vector<int> edge(3,0);

			edge[0] = min(cableplatformtrianglelist[j][0],cableplatformtrianglelist[j][1]);
			edge[1] = max(cableplatformtrianglelist[j][0],cableplatformtrianglelist[j][1]);
			ismember = false;
			rayindex = -1;
			

			for (int k=0; k<(int) edgelist.size() ;k++)
			{ 
				if (edgelist[k][0] == edge[0] && edgelist[k][1] == edge[1])
				{
					ismember = true;
					rayindex = edgelist[k][2];
					break;
				}
			}

			if (ismember == false)
			{
				middle = cableplatformraylist[edge[0]]+cableplatformraylist[edge[1]];
				middle.normalize();
				cableplatformraylist.push_back(middle);

				edge[2] = (int) cableplatformraylist.size()-1;
				rayindex = edge[2];
				edgelist.push_back(edge);				
			}

			int inda = rayindex;

			edge[0] = min(cableplatformtrianglelist[j][2],cableplatformtrianglelist[j][1]);
			edge[1] = max(cableplatformtrianglelist[j][2],cableplatformtrianglelist[j][1]);
			ismember = false;
			rayindex = -1;

			for (int k=0; k<(int) edgelist.size() ;k++)
			{ 
				if (edgelist[k][0] == edge[0] && edgelist[k][1] == edge[1])
				{
					ismember = true;
					rayindex = edgelist[k][2];
					break;
				}
			}

			if (ismember == false)
			{
				middle = cableplatformraylist[edge[0]]+cableplatformraylist[edge[1]];
				middle.normalize();
				cableplatformraylist.push_back(middle);

				edge[2] = (int) cableplatformraylist.size()-1;
				rayindex = edge[2];
				edgelist.push_back(edge);				
			}

			int indb = rayindex;

			edge[0] = min(cableplatformtrianglelist[j][0],cableplatformtrianglelist[j][2]);
			edge[1] = max(cableplatformtrianglelist[j][0],cableplatformtrianglelist[j][2]);
			ismember = false;
			rayindex = -1;
			
			for (int k=0; k<(int) edgelist.size() ;k++)
			{ 
				if (edgelist[k][0] == edge[0] && edgelist[k][1] == edge[1])
				{
					ismember = true;
					rayindex = edgelist[k][2];
					break;
				}
			}

			if (ismember == false)
			{
				middle = cableplatformraylist[edge[0]]+cableplatformraylist[edge[1]];
				middle.normalize();
				cableplatformraylist.push_back(middle);

				edge[2] = (int) cableplatformraylist.size()-1;
				rayindex = edge[2];
				edgelist.push_back(edge);				
			}

			int indc = rayindex;

			triangle[0] = cableplatformtrianglelist[j][0]; triangle[1] = inda; triangle[2] = indc; newtrianglelist.push_back(triangle);
			triangle[0] = cableplatformtrianglelist[j][1]; triangle[1] = indb; triangle[2] = inda; newtrianglelist.push_back(triangle);
			triangle[0] = cableplatformtrianglelist[j][2]; triangle[1] = indc; triangle[2] = indb; newtrianglelist.push_back(triangle);

			triangle[0] = inda; triangle[1] = indb; triangle[2] = indc; newtrianglelist.push_back(triangle);
		}
		cableplatformtrianglelist.clear();
		cableplatformtrianglelist = newtrianglelist;
	}
}

/*! load platform geometry data from .stl file
 *	
 *  \param [in] filename: name of input of .stl data. 
 *
 *  \return: true
 */
bool CInterference::loadplatformstldata(const string& filename)
{
	double importfactor = 0.001;
	// clear the data model
	platformstldata.clear();
	// load the STL file
	try {
		// we need some typedefs to match the data structure of the STL data
		typedef struct { float x,y,z; } CVector3f;
		typedef struct { CVector3f n,a,b,c; char res[2]; } CSTLRecord;
		std::ifstream file(filename.c_str(),ios::binary);
		char buf[80];	// the header
		file.read(buf,80);
		int anz = 0;
		file.read((char*)&anz,4);
		CSTLRecord patch;
		while (anz-- && !file.eof())
		{
			file.read((char*)&patch,50);
			// cast the vertex and normal data from float (STL-patch) to double (GL-patches)
			// and convert the data to the internal data model for intersection (point-direction form)
			platformstldata.push_back(Vector3d(patch.a.x*importfactor,patch.a.y*importfactor,patch.a.z*importfactor));
			platformstldata.push_back(Vector3d(patch.b.x*importfactor,patch.b.y*importfactor,patch.b.z*importfactor));
			platformstldata.push_back(Vector3d(patch.c.x*importfactor,patch.c.y*importfactor,patch.c.z*importfactor));
		}
//		printf("Triangles loaded: %i\n",vertices.size()/3);
	}
	catch (...)
	{ std::cout << "Error while loading STL file: " << filename.c_str() << endl; return false;}
	return true;
}

/*! calculate a rotation matrix from axis–angle convention
 * 
 *	\param [in] n: rotation axis
 *	\param [in] alpha: rotation angle in the interval [0, pi]
 *
 *  \return: rotation matrix
 */
Matrix3d CInterference::NalphaRotation(const Vector3d& n, const double& alpha) const
{
	Vector3d nn = n.normalized();

	Matrix3d res;
	res << nn(0)*nn(0)*(1-cos(alpha))+cos(alpha),       nn(0)*nn(1)*(1-cos(alpha))-nn(2)*sin(alpha), nn(0)*nn(2)*(1-cos(alpha))+nn(1)*sin(alpha),
		   nn(1)*nn(0)*(1-cos(alpha))+nn(2)*sin(alpha), nn(1)*nn(1)*(1-cos(alpha))+cos(alpha),       nn(1)*nn(2)*(1-cos(alpha))-nn(0)*sin(alpha),
		   nn(2)*nn(0)*(1-cos(alpha))-nn(1)*sin(alpha), nn(2)*nn(1)*(1-cos(alpha))+nn(0)*sin(alpha), nn(2)*nn(2)*(1-cos(alpha))+cos(alpha); 
	return res;
}

/*! write data of cable-cable interference regions into .csv file
 *	
 *  \param [in] filename: name of outputfile for .csv data. 
 *
 *  \return: true
 */
bool CInterference::writeCableCableSetsToCSV(const string& filename)
{
	ofstream outputfile (filename);

	for (int l=0; l< (int) cablecablepartitionedpointlist.size(); l++)
	{	
		for (int i=0; i< (int) cablecablepartitionedpointlist[l].size(); i++)
		{	
			outputfile << cablecablepartitionedpointlist[l][i](0) << "," << cablecablepartitionedpointlist[l][i](1) << "," << cablecablepartitionedpointlist[l][i](2) << "," ;
		}
		outputfile << endl ;
	}					
	outputfile.close();

	return true;
}

//! executes calculateConvexHull on all prepared sets
void CInterference::calculateCableCableInterferenceSetsHulls()
{
	for (int i=0; i< (int) cablecablepartitionedpointlist.size(); i++)
	{
		calculateConvexHull( cablecablepartitionedpointlist[i], cablecablepartininedtrianglelist[i] );
	}
}

//! prepares triangeles for the cable-cable interference visualization in Shapes.h
void CInterference::setupCableCableInterferenceHullsTriangles()
{
	redtriangles.clear();
	for (int i=0; i< (int) cablecablepartitionedpointlist.size(); i++)
	{
		for (int j=0; j< (int) cablecablepartininedtrianglelist[i].size(); j++)
		{
			redtriangles.push_back(cablecablepartitionedpointlist[i][cablecablepartininedtrianglelist[i][j][0]]);
			redtriangles.push_back(cablecablepartitionedpointlist[i][cablecablepartininedtrianglelist[i][j][1]]);
			redtriangles.push_back(cablecablepartitionedpointlist[i][cablecablepartininedtrianglelist[i][j][2]]);
		}
	}
}

/*! calculate the convex partition of a cable-cable interference subset which is defined by two sphere caps
 * 
 *	\param [in] bcur: vector defining the first sphere cap
 *	\param [in] bijstep: vector connecting the first to the second sphere cap
 *	\param [in] aj: vector defining the common center of the sphere caps
 *	\param [in] aij: vector which defines the line segment used in the Minkowski summation
 */
void CInterference::connectSphereSections(const Vector3d& bcur, const Vector3d& bijstep, const Vector3d& aj, const Vector3d& aij)
{
	const double eps = 0.0001;
	const int stepsEta = ((int) (etaMax/deltaeta))+1; 
	const double deleta = etaMax/(double)stepsEta ;
	const double deltacircle  = 2*MO_PI/(double)stepscircle;

	Vector3d bcurlong = bcur+bijstep;

	Vector3d ob = bcur.cross(bijstep);
	if (ob.norm()<eps)
	{
		Vector3d e1(1,0,0);
		Vector3d e2(0,1,0);
				
		if (abs(e1.dot(bcur))<abs(e2.dot(bcur)))
		{
			ob = e1-e1.dot(bcur)/bcur.norm()*bcur;
		} else {
			ob = e2-e2.dot(bcur)/bcur.norm()*bcur;
		}
	}
	
	for (int i=0; i < stepsEta; i++)
	{
		for (int j=0; j < stepscircle; j++)
		{

			Vector3d obrot0 = NalphaRotation(bcur, j   *deltacircle)*ob;
			Vector3d obrot1 = NalphaRotation(bcur,(j+1)*deltacircle)*ob;

			Vector3d bcurrot00 = NalphaRotation(obrot0,deleta* i   )*bcur;
			Vector3d bcurrot01 = NalphaRotation(obrot0,deleta*(i+1))*bcur;
			Vector3d bcurrot10 = NalphaRotation(obrot1,deleta* i   )*bcur;
			Vector3d bcurrot11 = NalphaRotation(obrot1,deleta*(i+1))*bcur;

			Vector3d obrot0long = NalphaRotation(bcurlong, j   *deltacircle)*ob;
			Vector3d obrot1long = NalphaRotation(bcurlong,(j+1)*deltacircle)*ob;

			Vector3d bcurrot00long = NalphaRotation(obrot0long,deleta* i   )*bcurlong;
			Vector3d bcurrot01long = NalphaRotation(obrot0long,deleta*(i+1))*bcurlong;
			Vector3d bcurrot10long = NalphaRotation(obrot1long,deleta* i   )*bcurlong;
			Vector3d bcurrot11long = NalphaRotation(obrot1long,deleta*(i+1))*bcurlong;


			vector<Vector3d> localpointlist; 
			vector< vector<int> > localtrianglelist;
			vector<int> startingtriangle;

			localpointlist.push_back(aj+bcurrot00);
			localpointlist.push_back(aj+bcurrot01);
			localpointlist.push_back(aj+bcurrot11);
			localpointlist.push_back(aj+bcurrot00long);
			localpointlist.push_back(aj+bcurrot01long);
			localpointlist.push_back(aj+bcurrot11long);

			localpointlist.push_back(aij*TriangleLength+aj+bcurrot00);
			localpointlist.push_back(aij*TriangleLength+aj+bcurrot01);
			localpointlist.push_back(aij*TriangleLength+aj+bcurrot11);
			localpointlist.push_back(aij*TriangleLength+aj+bcurrot00long);
			localpointlist.push_back(aij*TriangleLength+aj+bcurrot01long);
			localpointlist.push_back(aij*TriangleLength+aj+bcurrot11long);

			cablecablepartitionedpointlist.push_back(localpointlist);
				

			Vector3d nin = (localpointlist[1]-localpointlist[0]).cross(localpointlist[2]-localpointlist[0]);
			if (aij.dot(nin)>0)
			{
				startingtriangle.push_back(0);
				startingtriangle.push_back(1);
				startingtriangle.push_back(2);
			}
			else{
				startingtriangle.push_back(0+3);
				startingtriangle.push_back(2+3);
				startingtriangle.push_back(1+3);
			}		

			localtrianglelist.push_back(startingtriangle);

			cablecablepartininedtrianglelist.push_back(localtrianglelist);

			localpointlist.clear(); 
			localtrianglelist.clear();
			startingtriangle.clear();

			if (i>0)
			{
				localpointlist.push_back(aj+bcurrot00);
				localpointlist.push_back(aj+bcurrot11);
				localpointlist.push_back(aj+bcurrot10);
				localpointlist.push_back(aj+bcurrot00long);
				localpointlist.push_back(aj+bcurrot11long);
				localpointlist.push_back(aj+bcurrot10long);

				localpointlist.push_back(aij*TriangleLength+aj+bcurrot00);
				localpointlist.push_back(aij*TriangleLength+aj+bcurrot11);
				localpointlist.push_back(aij*TriangleLength+aj+bcurrot10);
				localpointlist.push_back(aij*TriangleLength+aj+bcurrot00long);
				localpointlist.push_back(aij*TriangleLength+aj+bcurrot11long);
				localpointlist.push_back(aij*TriangleLength+aj+bcurrot10long);

				cablecablepartitionedpointlist.push_back(localpointlist);
				

				Vector3d nin = (localpointlist[1]-localpointlist[0]).cross(localpointlist[2]-localpointlist[0]);
				if (aij.dot(nin)>0)
				{
					startingtriangle.push_back(0);
					startingtriangle.push_back(1);
					startingtriangle.push_back(2);
				}
				else{
					startingtriangle.push_back(0+3);
					startingtriangle.push_back(2+3);
					startingtriangle.push_back(1+3);
				}		

				localtrianglelist.push_back(startingtriangle);
				cablecablepartininedtrianglelist.push_back(localtrianglelist);
				localpointlist.clear(); 
				localtrianglelist.clear();
				startingtriangle.clear();
			}				
		}
	}		
}

/*! calculate the convex partition of a cable-cable interference subset which is defined by two discs
 * 
 *	\param [in] bcur: vector defining the first disc
 *	\param [in] bijstep: vector connecting the first to the second disc
 *	\param [in] aj: vector defining the common center of the discs
 *	\param [in] aij: vector which defines the line segment used in the Minkowski summation
 */
void CInterference::connectSphereCircles(const Vector3d& bcur, const Vector3d& bijstep, const Vector3d& aj, const Vector3d& aij)
{
	const double eps = 0.0001;
	const double deltacircle  = 2*MO_PI/(double)stepscircle;

	Vector3d bcurlong = bcur+bijstep;
	Vector3d ob = bcur.cross(bijstep);
	if (ob.norm()<eps)
	{
		Vector3d e1(1,0,0);
		Vector3d e2(0,1,0);
				
		if (abs(e1.dot(bcur))<abs(e2.dot(bcur)))
		{
			ob = e1-e1.dot(bcur)/bcur.norm()*bcur;
		} else {
			ob = e2-e2.dot(bcur)/bcur.norm()*bcur;
		}
	}

	for (int j=0; j < stepscircle; j++)
	{
		Vector3d obrot0 = NalphaRotation(bcur, j   *deltacircle)*ob;
		Vector3d obrot1 = NalphaRotation(bcur,(j+1)*deltacircle)*ob;
		Vector3d bcurrot0 = NalphaRotation(obrot0,etaMax)*bcur;
		Vector3d bcurrot1 = NalphaRotation(obrot1,etaMax)*bcur;

		Vector3d obrot0long = NalphaRotation(bcurlong, j   *deltacircle)*ob;
		Vector3d obrot1long = NalphaRotation(bcurlong,(j+1)*deltacircle)*ob;
		Vector3d bcurrot0long = NalphaRotation(obrot0long,etaMax)*bcurlong;
		Vector3d bcurrot1long = NalphaRotation(obrot1long,etaMax)*bcurlong;

		vector<Vector3d> localpointlist; 

		localpointlist.push_back(aj+bcur*cos(etaMax));
		localpointlist.push_back(aj+bcurrot0);
		localpointlist.push_back(aj+bcurrot1);
		localpointlist.push_back(aj+bcurlong*cos(etaMax));
		localpointlist.push_back(aj+bcurrot0long);
		localpointlist.push_back(aj+bcurrot1long);

		localpointlist.push_back(aij*TriangleLength+aj+bcur*cos(etaMax));
		localpointlist.push_back(aij*TriangleLength+aj+bcurrot0);
		localpointlist.push_back(aij*TriangleLength+aj+bcurrot1);
		localpointlist.push_back(aij*TriangleLength+aj+bcurlong*cos(etaMax));
		localpointlist.push_back(aij*TriangleLength+aj+bcurrot0long);
		localpointlist.push_back(aij*TriangleLength+aj+bcurrot1long);


		cablecablepartitionedpointlist.push_back(localpointlist);	
		vector<int> startingtriangle;			

		Vector3d nin = (localpointlist[1]-localpointlist[0]).cross(localpointlist[2]-localpointlist[0]);
		if (aij.dot(nin)>0)
		{
			startingtriangle.push_back(0);
			startingtriangle.push_back(1);
			startingtriangle.push_back(2);
		}
		else{
			startingtriangle.push_back(0+3);
			startingtriangle.push_back(2+3);
			startingtriangle.push_back(1+3);
		}
		

		vector< vector<int> > localtrianglelist;
		localtrianglelist.push_back(startingtriangle);
		cablecablepartininedtrianglelist.push_back(localtrianglelist);
		localpointlist.clear(); 
		localtrianglelist.clear();
	}	
}



/*! calculate the convex hull of a set of points starting with an initial triangle on the hull
 *  this method could be replacd by a more robust and efficient convex hull algorithm
 *  
 *	\param [in] pointlist: list of points whose convex hull is calculated
 *	\param [in] trianglelist: list of triangles containing the indices of the corresponding points. 
 *                            this list should contain the one triangle which is part of the convex hull
 */
void CInterference::calculateConvexHull( vector<Vector3d>& pointlist, vector< vector<int> >& trianglelist)
{
	const double eps = 0.000001;
	int pointnumber = (int) pointlist.size();
	deque< vector<int> > openedges;
	deque< vector<int> > markededges;

	//load first triangle  (i = 0)
	int i = 0;
	Vector3d p0 = pointlist[trianglelist[i][0]];
	Vector3d p1 = pointlist[trianglelist[i][1]];
	Vector3d p2 = pointlist[trianglelist[i][2]];

	vector<int> newedge(3,0);

	newedge[0] = trianglelist[i][0];
	newedge[1] = trianglelist[i][1];
	newedge[2] = i;
	openedges.push_back(newedge); 

	newedge[0] = trianglelist[i][0];
	newedge[1] = trianglelist[i][2];
	newedge[2] = i;
	openedges.push_back(newedge); 

	newedge[0] = trianglelist[i][1];
	newedge[1] = trianglelist[i][2];
	newedge[2] = i;
	openedges.push_back(newedge); 
	
	while(openedges.empty() == false)
	{
		vector<int> edge = openedges.front();
		openedges.pop_front();

		bool notfound = true;

		for (int k=0; k<(int)(markededges.size()) ;k++)
		{
			if ( (markededges[k][0] == edge[0] && markededges[k][1] == edge[1]) || (markededges[k][0] == edge[1] && markededges[k][1] == edge[0]) )
			{
				notfound = false;
				break;
			}
		}

		if (notfound)
		{
			// find new triangle
			int i = edge[2];

			Vector3d p0 = pointlist[trianglelist[i][0]];
			Vector3d p1 = pointlist[trianglelist[i][1]];
			Vector3d p2 = pointlist[trianglelist[i][2]];

			Vector3d nin = (p1-p0).cross(p2-p0);


			int edgestart = 0;
			int edgeend = 1;

			if (trianglelist[i][0] != edge[0] && trianglelist[i][0] != edge[1])
			{
				edgestart = 1;
				edgeend = 2;

			} else if (trianglelist[i][1] != edge[0] && trianglelist[i][1] != edge[1])
			{
				edgestart = 2;
				edgeend = 0;
			}

			Vector3d edgedir=pointlist[trianglelist[i][edgeend]]-pointlist[trianglelist[i][edgestart]];
			Vector3d outdir = edgedir.cross(nin);
			outdir.normalize();

			nin.normalize();

			double anglemin = 4;
			int minindex = -1;
			vector<int> flatpoints;

			for (int j=0; j <pointnumber; j++) 
			{
				if ( j != trianglelist[i][0] && j != trianglelist[i][1] && j != trianglelist[i][2] ) 
				{
					Vector3d point = pointlist[j];

					// axis angle
					double y = nin.dot(point-pointlist[trianglelist[i][edgestart]]);
					double x = outdir.dot(point-pointlist[trianglelist[i][edgestart]]);

					if (y<eps)
					{
						y=+0;
					}
					
					double angle = atan2(y,x);
					
					if (angle<anglemin-eps && (abs(x)>eps || abs(y)>eps) )
					{
						anglemin = angle;
						minindex = j;

						flatpoints.clear();
						flatpoints.push_back(minindex);

					} else if (angle<anglemin+eps)
					{
						flatpoints.push_back(j);		
					}
				}
			}

			if (true || (int)flatpoints.size() == 1)
			{
				// add new triangle
				vector<int> newtriangle(3,0);
				newtriangle[0] = trianglelist[i][edgeend];
				newtriangle[1] = trianglelist[i][edgestart];
				newtriangle[2] = minindex;
				trianglelist.push_back(newtriangle);

				// add new edges
				vector<int> newedge(3,0);
				bool notopened;

				newedge[0] = trianglelist[i][edgestart];
				newedge[1] = minindex;
				newedge[2] = (int)(trianglelist.size())-1;

				notopened = true;
				for (int k=0; k<(int)(openedges.size()) ;k++)
				{
					if ( (openedges[k][0] == newedge[0] && openedges[k][1] == newedge[1]) || (openedges[k][0] == newedge[1] && openedges[k][1] == newedge[0]) )
					{
						notopened = false;
						markededges.push_back(newedge);
						break;
					}
				}
				if (notopened)
				{
					openedges.push_back(newedge); 
				}

				newedge[0] = trianglelist[i][edgeend];
				newedge[1] = minindex;
				newedge[2] = (int)(trianglelist.size())-1;

				notopened = true;
				for (int k=0; k<(int)(openedges.size()) ;k++)
				{
					if ( (openedges[k][0] == newedge[0] && openedges[k][1] == newedge[1]) || (openedges[k][0] == newedge[1] && openedges[k][1] == newedge[0]) )
					{
						notopened = false;
						markededges.push_back(newedge);
						break;
					}
				}
				if (notopened)
				{
					openedges.push_back(newedge); 
				}
			} else {

				//do 2D convex hull
				flatpoints.push_back(trianglelist[i][edgestart]);
				flatpoints.push_back(trianglelist[i][edgeend]);

				int flatpointnumber = (int)flatpoints.size();
				
				int start = flatpointnumber-2;
				int next = flatpointnumber-1;

				int startpoint = start;

				vector<int> radialpoints;
				radialpoints.push_back(start);
				radialpoints.push_back(next);

				while(next != startpoint)
				{
					double minalpha = 4;
					int minindex = -1;
					for (int j=0; j <flatpointnumber; j++) 
					{
						if ( j != start && j != next)
						{
							Vector3d dir = pointlist[flatpoints[next]]-pointlist[flatpoints[start]];
							Vector3d newdir = pointlist[flatpoints[j]]-pointlist[flatpoints[next]];

							double alpha = acos(dir.dot(newdir)/(dir.norm()*newdir.norm()));

							if (alpha<minalpha)
							{
								minalpha = alpha;
								minindex = j;
							}
						}
					}
					radialpoints.push_back(minindex);
					start = next;
					next = minindex;
				}

				Vector3d middle(0,0,0);
				int radialpointnumber = (int)radialpoints.size();
				for (int j=1; j <radialpointnumber; j++) 
				{
					middle = middle+pointlist[flatpoints[radialpoints[j]]];
				}
				middle = middle/((double)radialpointnumber-1);
				pointlist.push_back(middle);

				for (int j=1; j <radialpointnumber; j++) 
				{
					// add new triangle
					vector<int> newtriangle(3,0);
					newtriangle[0] = flatpoints[radialpoints[j]];
					newtriangle[1] = flatpoints[radialpoints[j-1]];
					newtriangle[2] = (int) pointlist.size()-1;
					trianglelist.push_back(newtriangle);

					// add new edges
					vector<int> newedge(3,0);
					bool notopened;

					newedge[0] = flatpoints[radialpoints[j]];
					newedge[1] = flatpoints[radialpoints[j-1]];
					newedge[2] = (int)(trianglelist.size())-1;

					notopened = true;
					for (int k=0; k<(int)(openedges.size()) ;k++)
					{
						if ( (openedges[k][0] == newedge[0] && openedges[k][1] == newedge[1]) || (openedges[k][0] == newedge[1] && openedges[k][1] == newedge[0]) )
						{
							notopened = false;
							markededges.push_back(newedge);
							break;
						}
					}
					if (notopened)
					{
						openedges.push_back(newedge); 
					}

				}

			}
			//mark edge as processed
			markededges.push_back(edge);
		}
	}	
}


/*! calculate the cable-cable interference sets
 *  this method partitions the sets and calls the methods connectSphereSections and connectSphereCircles on these parts
 */
void CInterference::setupCableCableInterferenceSets()
{
	int maxi = 1;
	if (displayAllCableCableInterferenceSets)
	{
		maxi = min(pRobot->getNow(),lastCable);
	}

	cablecablepartitionedpointlist.clear();
	cablecablepartininedtrianglelist.clear();

	for (int i=max(0,firstCable); i < maxi; i++)
	{
		for (int j=i+1; j < min(pRobot->getNow(),lastCable); j++) 
		{
			Vector3d bi = R0*pRobot->getPlatform(i);
			Vector3d bj = R0*pRobot->getPlatform(j);

			Vector3d ai = pRobot->getBase(i);
			Vector3d aj = pRobot->getBase(j);

			Vector3d aij = aj-ai; 
			// it may happen that aij has zero length if ai == aj; then we skip this pair
			if (aij.lpNorm<2>()<eps)
				continue;
			aij.normalize();
					
			Vector3d bij = bj-bi; 
			// it may happen that aij has zero length if ai == aj; then we skip this pair
			if (bij.lpNorm<2>()<eps)
				continue;
			bij.normalize();

			double alpha = acos(-bi.dot(bij)/bi.norm());
			int alphasteps = (int) (alpha/deltaalpha);
			double alphacircle = alpha-asin(cos(etaMax)*sin(alpha));
			double dist1 = 0;
			double dist2 = 0;
			Vector3d bf,bs;

			for (int k=1; k < alphasteps; k++)  
			{
				double beta = alpha-k*deltaalpha;
				dist2 = sin(k*deltaalpha)/sin(beta)*bi.norm();
				bf = bij*dist1;
				bs = bij*dist2;
				if (beta>MO_PI/2)
				{
					bf = bij*dist2;
					bs = -bij*dist1;
				}
				
				if (alphacircle<(k-1)*deltaalpha)
				{
					connectSphereCircles(-bi+bf, bs, aj, aij);
					if (displayAllCableCableInterferenceSets)
					{
						connectSphereCircles(-bj-bf, -bs, ai, -aij);
					}
				} else {
					connectSphereSections(-bi+bf, bs, aj, aij);
					if (displayAllCableCableInterferenceSets)
					{
						connectSphereSections(-bj-bf, -bs, ai, -aij);
					}
				}
				dist1 = dist2;
			}

			dist2 = dist1+TriangleLength;
			bf = bij*dist1;
			bs = bij*dist2;
				
			connectSphereCircles(-bi+bf, bs, aj, aij);
			if (displayAllCableCableInterferenceSets)
			{
				connectSphereCircles(-bj-bf, -bs, ai, -aij);
			}
		}
	}
}

/*! update position of clipping planes to cut off rendering of objects outside of the area of intrest around the robot.
 *	also updates the characteristic length of the robot.
 */
void CInterference::updateClippingPlanes(){
	// assumption: Origin is in the center of the geometry
	double padding = 2.0;

	xmin = 0, xmax = 0, ymin = 0, ymax = 0, zmin = 0, zmax = 0;
	for (int i=max(0,firstCable); i < min(pRobot->getNow(),lastCable); i++)
	{
		Vector3d ai = pRobot->getBase(i);

		xmin = min(xmin,ai(0));
		ymin = min(ymin,ai(1));
		zmin = min(zmin,ai(2));

		xmax = max(xmax,ai(0));
		ymax = max(ymax,ai(1));
		zmax = max(zmax,ai(2));
	}

	xmin = xmin*padding;
	ymin = ymin*padding;
	zmin = zmin*padding;
	
	xmax = xmax*padding;
	ymax = ymax*padding;
	zmax = zmax*padding;

	TriangleLength = max(xmax,max(ymax,zmax))-min(xmin,min(ymin,zmin));
}

/*! delete currently visualized shapes
 */
void CInterference::clearShapes(){
	looplines.clear();
	triangles.clear();
	printshapes.clear();
	layerconnections.clear();
	redtriangles.clear();
}

/*! set eta as maximal angle of the orientation set
 *	\param [in] eta: size parameter of orientation set
 */
void CInterference::setEtaMax(double eta)
{
	etaMax = eta;
}

/*! calculate the minimal value of eta such that the associated orientation set contains collisions everywhere
 *  \return: angle
 */
double CInterference::calculateMinInterferenceAngle()
{
	double maxalpha = 0.0;
				
	for (int i=max(0,firstCable); i < min(pRobot->getNow(),lastCable); i++)
	{
		for (int j=i+1; j < min(pRobot->getNow(),lastCable); j++) 
		{
			Vector3d aij = pRobot->getBase(j)-pRobot->getBase(i); 
			// it may happen that aij has zero length if ai == aj; then we skip this pair
			if (aij.lpNorm<2>()<eps)
				continue;
			aij.normalize();
					
			Vector3d bij = R0*(pRobot->getPlatform(j)-pRobot->getPlatform(i)); 
			// it may happen that aij has zero length if ai == aj; then we skip this pair
			if (bij.lpNorm<2>()<eps)
				continue;
			bij.normalize();
			// check for parallelity between aij || bij by calculating the angle between the spanning unity vectors
			double alpha = acos(aij.dot(bij));

			maxalpha = max(maxalpha,alpha);
		}
	}					
	return MO_PI-maxalpha;			
}



//! add the symbolic names to the configuration parameters
//! the naming in this implementation is preliminary and may need a revision in the future versions
void CInterference::bind()
{
	CAlgorithm::bind();
	if (!pReflector)
		return;
	pReflector->bind(etaMax,"etaMax","interference/@etaMax");

	pReflector->bind(deltaeta,"deltaeta","interference/cable/@deltaeta");
	pReflector->bind(deltaalpha,"deltaalpha","interference/cable/@deltaalpha");
	pReflector->bind(stepscircle,"stepscircle","interference/cable/@stepscircle");
	pReflector->bind(displayAllCableCableInterferenceSets,"displayAllCableCableInterferenceSets","interference/cable/@displayAllCableCableInterferenceSets");

	
	pReflector->bind(printParaVisNumber,"printVisMode","interference/print/@printParaVisNumber");
	pReflector->bind(numberofprintlayers,"numberofprintlayers","interference/print/@numberofprintlayers");

	pReflector->bind(pn.x(),"pn.x","interference/print/printingnozzle/@x");
	pReflector->bind(pn.y(),"pn.y","interference/print/printingnozzle/@y");
	pReflector->bind(pn.z(),"pn.z","interference/print/printingnozzle/@z");

	pReflector->bind(printcolor.x(),"printcolor.R","interference/print/printcolor/@R");
	pReflector->bind(printcolor.y(),"printcolor.G","interference/print/printcolor/@G");
	pReflector->bind(printcolor.z(),"printcolor.B","interference/print/printcolor/@B");

	pReflector->bind(shadowcolor.x(),"shadowcolor.R","interference/print/shadowcolor/@R");
	pReflector->bind(shadowcolor.y(),"shadowcolor.G","interference/print/shadowcolor/@G");
	pReflector->bind(shadowcolor.z(),"shadowcolor.B","interference/print/shadowcolor/@B");

	pReflector->bind(pEllipse,"pEllipse","interference/print/@pEllipse");
	pReflector->bind(printheight,"printheight","interference/print/@printheight");
	pReflector->bind(displayshadows,"displayshadows","interference/print/@displayshadows");

	pReflector->bind(d1.x(),"d1.x","interference/print/d1/@x");
	pReflector->bind(d1.y(),"d1.y","interference/print/d1/@y");

	pReflector->bind(d2.x(),"d2.x","interference/print/d2/@x");
	pReflector->bind(d2.y(),"d2.y","interference/print/d2/@y");

	pReflector->bind(printcenter.x(),"printcenter.x","interference/print/center/@x");
	pReflector->bind(printcenter.y(),"printcenter.y","interference/print/center/@y");
	pReflector->bind(printcenter.z(),"printcenter.z","interference/print/center/@z");

	pReflector->bind(platformcenterposition.x(),"platformcenter.x","interference/platform/center/@x");
	pReflector->bind(platformcenterposition.y(),"platformcenter.y","interference/platform/center/@y");
	pReflector->bind(platformcenterposition.z(),"platformcenter.z","interference/platform/center/@z");

	pReflector->bind(refinement,"refinement","interference/platform/@refinement");
	pReflector->bind(searchsteps,"searchsteps","interference/platform/@searchsteps");

	pReflector->bind(xmin,"xmin","interference/boundingBox/@xmin");
	pReflector->bind(xmax,"xmax","interference/boundingBox/@xmax");
	pReflector->bind(ymin,"ymin","interference/boundingBox/@ymin");
	pReflector->bind(ymax,"ymax","interference/boundingBox/@ymax");
	pReflector->bind(zmin,"zmin","interference/boundingBox/@zmin");
	pReflector->bind(zmax,"zmax","interference/boundingBox/@zmax");
	pReflector->bind(alphaMin,"alphaMin","interference/orientationOld/@alphaMin");
	pReflector->bind(alphaMax,"alphaMax","interference/orientationOld/@alphaMax");
	pReflector->bind(deltaAlpha,"deltaAlpha","interference/orientationOld/@deltaAlpha");
	pReflector->bind(betaMin,"betaMin","interference/orientationOld/@betaMin");
	pReflector->bind(betaMax,"betaMax","interference/orientationOld/@betaMax");
	pReflector->bind(deltaBeta,"deltaBeta","interference/orientationOld/@deltaBeta");
	pReflector->bind(gammaMin,"gammaMin","interference/orientationOld/@gammaMin");
	pReflector->bind(gammaMax,"gammaMax","interference/orientationOld/@gammaMax");
	pReflector->bind(deltaGamma,"deltaGamma","interference/orientationOld/@deltaGamma");
	pReflector->bind(TriangleLength,"TriangleLength","interference/@TriangleLength");
	pReflector->bind(firstCable,"firstCable","orientationOld/@firstCable");
	pReflector->bind(lastCable,"lastCable","orientationOld/@lastCable");
	pReflector->bind(eps,"eps","orientationOld/@eps");
}

//! set the orientation range for interference calculation
//! the values are checked for feasibility before being set
void CInterference::setOrientationWorkspace(const double& Alpha_min, const double& Alpha_max, const double& Delta_alpha, const double& Beta_min, const double& Beta_max, const double& Delta_beta,  const double& Gamma_min, const double& Gamma_max, const double& Delta_gamma)
{
	const double maxSteps = 10000;	//! the lower bound of the step size is a fraction of the interval length (needed to prevent memory overflow in calculation)
	alphaMin	= min(Alpha_min,Alpha_max);		// use the smaller one for lower limit
	alphaMax	= max(Alpha_min,Alpha_max);		// use the bigger one for upper limit
	deltaAlpha	= max((alphaMax-alphaMin)/maxSteps,max(Delta_alpha,-Delta_alpha));	// prevent negative, zero, or too small stepsizes
	betaMin	= min(Beta_min,Beta_max);		// use the smaller one for lower limit
	betaMax	= max(Beta_min,Beta_max);		// use the bigger one for upper limit
	deltaBeta	= max((betaMax-betaMin)/maxSteps,max(Delta_beta,-Delta_beta));		// prevent negative, zero, or too small stepsizes
	gammaMin	= min(Gamma_min,Gamma_max);		// use the smaller one for lower limit
	gammaMax	= max(Gamma_min,Gamma_max);		// use the bigger one for upper limit
	deltaGamma	= max((gammaMax-gammaMin)/maxSteps,max(Delta_gamma,-Delta_gamma));		// prevent negative, zero, or too small stepsizes
}

//! set the bounding box for outer clipping of the region
//! the values are checked for feasibility before being set
void CInterference::setClippingBox(double& Xmin, double& Xmax, double& Ymin, double& Ymax, double& Zmin, double& Zmax)
{
	xmin = min(Xmin,Xmax); 
	xmax = max(Xmin,Xmax); 
	ymin = min(Ymin,Ymax); 
	ymax = max(Ymin,Ymax); 
	zmin = min(Zmin,Zmax); 
	zmax = max(Zmin,Zmax); 
}

//! get the orientation range for interference calculation
//! the values are checked for feasibility before being set
void CInterference::getOrientationWorkspace(double& Alpha_min, double& Alpha_max, double& Delta_alpha, double& Beta_min, double& Beta_max, double& Delta_beta) const
{
	Alpha_min   = alphaMin;
	Alpha_max   = alphaMax;
	Delta_alpha = deltaAlpha;

	Beta_min   = betaMin;
	Beta_max   = betaMax;
	Delta_beta = deltaBeta;
}

//! get the bounding box for outer clipping of the region
void CInterference::getClippingBox(double& Xmin, double& Xmax, double& Ymin, double& Ymax, double& Zmin, double& Zmax) const
{
	Xmin = xmin; 
	Xmax = xmax; 
	Ymin = ymin; 
	Ymax = ymax; 
	Zmin = zmin; 
	Zmax = zmax; 
}


/*! Calculate if a pose is in front or behind a collision plane in orthogonal direction of the plane with respect to the initial position of 
 * the cable robot platform.
 *  \param [in] c: initial position of the cable-robot platform
 *  \param [in] r: cartesian position of the pose
 *  \param [in] R: orientation of the pose
 * Description:
 * This function first calculates the cable-cable intersection planes for 
 * the pose handed to the function as an input value. Afterwards 
 * the normal vectors of the intersection planes are calculated and the vector
 * from the initial position of the cable robot platform (c) and the tip of the triangle 
 * of the collision plane is calculated. By the scalarproduct of this vector and the normal
 * vector it is decided if the pose is in front or behind a collision plane with respect to the initial
 * of the cable-robot platform (c).
 *
 *  \return true, if pose is valid (in front of collision plane)
 *
 *  \todo replace copied code from calculateInterference
 */

bool CInterference::checkPoseForCableCableCollision(const Vector3d& c, const Vector3d& r, const Matrix3d& R)
{
	// Parameter declaration
	bool NoCollisionIndicator = true;
	Vector3d p1Pos, p2Pos, p3Pos, p1Neg, p2Neg, p3Neg; // Points defining the edges of the intersection area
	Vector3d p12Pos, p13Pos, p12Neg, p13Neg; // Direction vectors of the edges of the intersection area, pointing away from p1, norm = 1
	Vector3d pCrossPos, pCrossNeg; // normal vector on the intersection area u and v, norm = 1
	Vector3d cp1Pos, cp1Neg; // Vector from initial platform position to p1
	double scalPos, scalNeg; // Scalar product of cp1 and normal vector of the interference plane
	Vector3d tPos, tNeg; // Vector of the factors:  d = t.x() * p12 + t.y() * p13 + t.z() * pCross
	Vector3d dPos, dNeg; // Distance vector from p1 to the position of the pose

	// Calculate the interference areas with the orientation of the pose (CInterference::calculateInterference())

	// ** Start Copied code from: CInterference::calculateInterference() ** //

	// we loop through pairs of cables computing all cable combination with index j>i.
	// additional, only the range from first to lastCable is tested. 
	for (int i=max(0,firstCable); i < min(pRobot->getNow(),lastCable); i++)
		for (int j=i+1; j < min(pRobot->getNow(),lastCable); j++) 
		{
			Vector3d aij = pRobot->getBase(j)-pRobot->getBase(i); 
			// it may happen that aij has zero length if ai == aj; then we skip this pair
			if (aij.lpNorm<2>()<eps)
				continue;
			aij.normalize();
					
			Vector3d bij = R*(pRobot->getPlatform(j)-pRobot->getPlatform(i)); 
			// it may happen that aij has zero length if ai == aj; then we skip this pair
			if (bij.lpNorm<2>()<eps)
				continue;
			bij.normalize();
			// check for parallelity between aij || bij by calculating the angle between the spanning unity vectors
			double alpha = acos(aij.dot(bij));
			if (alpha<eps || alpha>MO_PI-eps)
			{
				// \todo Implement the code to construct the line of interference here using the list "lines"
/*							if ((aij*bij)>0)
					printf("Parallel for %i and %i\n",i+1,j+1);
				else
					printf("Paralle and bad design for %i and %i\n",i+1,j+1);*/
				continue;
			}
			double l = TriangleLength/cos(alpha/2);

			// ** End Copied code from: CInterference::calculateInterference() ** // 
					
			p1Pos = pRobot->getBase(j) - R*pRobot->getPlatform(i);
			p2Pos = p1Pos + l*aij;
			p3Pos = p1Pos + l*bij;
									
			p1Neg = pRobot->getBase(i) - R*pRobot->getPlatform(j);
			p2Neg = p1Neg - l*aij;
			p3Neg = p1Neg - l*bij;
			

			// Calculate the normalized direction vectors and the normal vector on the instersection plane
			p12Pos = p2Pos - p1Pos;
			p12Pos.normalize();
		
			p13Pos = p3Pos - p1Pos;
			p13Pos.normalize();
		
			pCrossPos = p12Pos.cross(p13Pos);
			pCrossPos.normalize();

			p12Neg = p2Neg - p1Neg;
			p12Neg.normalize();
		
			p13Neg = p3Neg - p1Neg;
			p13Neg.normalize();
		
			pCrossNeg = p12Neg.cross(p13Neg);
			pCrossNeg.normalize();

			// Calculate the vector form initial platform position c to p1
			cp1Pos = p1Pos - c;
			cp1Neg = p1Neg - c;

			// Calculate the scalar product of the vector from the initial platform position c to p1 and the normal vector
			scalPos = cp1Pos.dot(pCrossPos);
			scalNeg = cp1Neg.dot(pCrossNeg);

			// Calculate the distance vector dPlus, dMinus from p1 to the position of the pose 
			dPos = r - p1Pos;
			dNeg = r - p1Neg;

			// Compose linear equation system
			Matrix3d aPos;
			aPos << p12Pos, p13Pos, pCrossPos;

			Matrix3d aNeg;
			aNeg << p12Neg, p13Neg, pCrossNeg;

			// Solve linear equation system to get t
			tPos = aPos.colPivHouseholderQr().solve(dPos);
			tNeg = aNeg.colPivHouseholderQr().solve(dNeg);
			
			// Check if position is "in front or behind" intersection plane
			if (tPos.x() > 0 && tPos.y() > 0)
				if ((tPos.z() > 0 && scalPos > 0) || (tPos.z() < 0 && scalPos < 0))
				{
					NoCollisionIndicator = false;

					return NoCollisionIndicator;
				}

			if (tNeg.x() > 0 && tNeg.y() > 0)
				if ((tNeg.z() > 0 && scalNeg > 0) || (tNeg.z() < 0 && scalNeg < 0))
				{
					NoCollisionIndicator = false;

					return NoCollisionIndicator;
				}

		}// end for i, j

	return NoCollisionIndicator;
}


/*! Calculate the region of intersection for a parallel wire robot
 *  The basic algorithm was adopted from Pessaut et. al 2010 who introduced
 *  a purely geometric way to calculate the regions of intersection for a 
 *  constant orientation workspace. This implementation generalized the result 
 *  to some extend by calculating also variations on the the rotation matrix
 *  \return true, if the computation was successful otherwise false
 *  \todo Implement more parameters to control the set of orientations to be 
 *        taken into account
 */
bool CInterference::calculateInterference()
{
	// clear the memory
	triangles.clear();
	lines.clear();

	// start the calculation 
	// for each combination of cables draw the region where intersection may occure
//	for (double alpha = alphaMin; alpha<=alphaMax; alpha+=deltaAlpha)
//		for (double beta  = betaMin; beta<=betaMax; beta+=deltaBeta)
	int stepsAlpha = deltaAlpha == 0 ? 1 : (int)((alphaMax-alphaMin)/deltaAlpha);
	int stepsBeta  = deltaBeta  == 0 ? 1 : (int)((betaMax-betaMin)/deltaBeta);
	int stepsGamma = deltaGamma == 0 ? 1 : (int)((gammaMax-gammaMin)/deltaGamma);
	for (int i = 0; i<stepsAlpha; i++)
		for (int j = 0; j<stepsBeta; j++)
			for (int k = 0; k<stepsGamma; k++)
			{
				double alpha = alphaMin+i*deltaAlpha;
				double beta = betaMin+j*deltaBeta;
				double gamma = gammaMin+k*deltaGamma;
				// multiply the reference orientation matrix R0 with the variations of the orientation matrix
				Matrix3d R = R0*Matrix3d::XRotationMatrix3d(alpha)*Matrix3d::YRotationMatrix3d(beta)*Matrix3d::ZRotationMatrix3d(gamma);
				// we loop through pairs of cables computing all cable combination with index j>i.
				// additional, only the range from first to lastCable is tested. 
				for (int i=max(0,firstCable); i < min(pRobot->getNow(),lastCable); i++)
					for (int j=i+1; j < min(pRobot->getNow(),lastCable); j++) 
					{
						Vector3d aij = pRobot->getBase(j)-pRobot->getBase(i); 
						// it may happen that aij has zero length if ai == aj; then we skip this pair
						if (aij.lpNorm<2>()<eps)
							continue;
						aij.normalize();
					
						Vector3d bij = R*(pRobot->getPlatform(j)-pRobot->getPlatform(i)); 
						// it may happen that aij has zero length if ai == aj; then we skip this pair
						if (bij.lpNorm<2>()<eps)
							continue;
						bij.normalize();
						// check for parallelity between aij || bij by calculating the angle between the spanning unity vectors
						double alpha = acos(aij.dot(bij));
						if (alpha<eps || alpha>MO_PI-eps)
						{
							// \todo Implement the code to construct the line of interference here using the list "lines"
/*							if ((aij*bij)>0)
								printf("Parallel for %i and %i\n",i+1,j+1);
							else
								printf("Paralle and bad design for %i and %i\n",i+1,j+1);*/
							continue;
						}
						double l = TriangleLength/cos(alpha/2);
					
						Vector3d p1 = pRobot->getBase(j) - R*pRobot->getPlatform(i);
						Vector3d p2 = p1 + l*aij;
						Vector3d p3 = p1 + l*bij;
						triangles.push_back(p1);
						triangles.push_back(p2);
						triangles.push_back(p3);
									
						p1 = pRobot->getBase(i) - R*pRobot->getPlatform(j);
						p2 = p1 - l*aij;
						p3 = p1 - l*bij;
						triangles.push_back(p1);
						triangles.push_back(p2);
						triangles.push_back(p3);
					}
			}
	return true;
}


/*! Robot Interference algorithm draft
 *  This approach extends the intersection algorithm by Perrault et al. by 
 *  taking into account the effect of different orientations. Using polar 
 *  soring the set orientation dependent set of vectors b_ij is sorted such
 *  that a triangulation for the region of interference is extracted.
 *
 *  BUG Report: The current version (as of 22.11.16) makes wrong assumptions
 *  as the geometric model does not take into account the change of base point
 *  as the base point of the constant orientation triable is a_j-b_j and this 
 *  point is NOT constant as b_j is computed from Rs*b_j and thus depends on 
 *  the platform orientation. 
 *
 *  \return true, if successful otherwise false
 *  \WARNING: Still a couple of errors inside!! The function is under development
 */
bool CInterference::calculateCollisionsOrientationWorkspace()
{
	// algorithm setting to be moved to class parameters in order to change at runtime
	const int segments = 36;	// the number of segments used in the polar decomposition (and thus related to the accuracy of the methods)
	// clear the internal storage for collision data
	triangles.clear();
	lines.clear();

	// 1) Determine the orientation set R_s of interest to be used for the interference region
	// this block was copied from the function above; if it is of further use, we have to make a 
	// seperate function from it.
	list<Matrix3d> Rs;
	int stepsAlpha = deltaAlpha == 0 ? 1 : (int)((alphaMax-alphaMin)/deltaAlpha);
	int stepsBeta  = deltaBeta  == 0 ? 1 : (int)((betaMax-betaMin)/deltaBeta);
	int stepsGamma = deltaGamma == 0 ? 1 : (int)((gammaMax-gammaMin)/deltaGamma);
	for (int i = 0; i<stepsAlpha; i++)
		for (int j = 0; j<stepsBeta; j++)
			for (int k = 0; k<stepsGamma; k++)
			{
				double alpha = alphaMin+i*deltaAlpha;
				double beta = betaMin+j*deltaBeta;
				double gamma = gammaMin+k*deltaGamma;
				// multiply the reference orientation matrix R0 with the variations of the orientation matrix
				Matrix3d R = R0*Matrix3d::XRotationMatrix3d(alpha)*Matrix3d::YRotationMatrix3d(beta)*Matrix3d::ZRotationMatrix3d(gamma);
				Rs.push_back(R);
			}

	// 2) for each pair (i,j) of cables with i<j compute b_ij for each orientation R_k in R_s
//	for (int i=0; i<pRobot->getNow(); i++)
//		for (int j=i+1; j<pRobot->getNow(); j++) 
	for (int i=0, j=1; i<1; i++)	// only for debug purpose to check only one cable pair
		{
			Vector3d aij = pRobot->getBase(j)-pRobot->getBase(i); 
			// it may happen that aij has zero length if ai == aj; then we skip this pair
			if (aij.lpNorm<2>()<eps)
				continue;
			aij.normalize();
			
			// list of all bij and their mean value
			list<Vector3d> bijs;
			Vector3d bij_mean = Vector3d::Zero();
			// loop through all orientations in Rs
			for (auto itor=Rs.begin(); itor!=Rs.end(); itor++)
			{			
				Vector3d bij = (*itor)*(pRobot->getPlatform(j)-pRobot->getPlatform(i)); 
				// it may happen that aij has zero length if ai == aj; then we skip this pair
				if (bij.lpNorm<2>()<eps)
					continue;
				bij.normalize();
				// store the normalized vector
				bijs.push_back(bij);
				bij_mean += bij;
			}

			// 3) determine the "mean" direction b_mean and a polar decomposition. use polar sorting to compute the hull of the cone in n facettes

			// estimate the central ray a_i -> center
			Vector3d uc = bij_mean;
			uc.normalize();
			// to compute the vector phi, we have to define a local frame. the x axis is
			// given by the direction of the line a_ij, the z-axis is given by uc
			Vector3d e1 = aij;
			// extract the orthogonal component of the first vector in the list
			// \todo: we numerical stability can be largely improved if a vector is sought that is has a larger angle with the reference vector uc
			e1-=e1.dot(uc)*uc;
			e1.normalize();
			// the third vector of the local frame is now constructed by cross product
			Vector3d e2 = uc.cross(e1);
			e2.normalize();
			// now we have a local frame K=(uc,e1,e2) 

			// the data model for intermediate storage of the hull model for b_ij (mapping phi -> cos_alpha)
			map<double,double> span;
			// perform a coordinate transformation from world frame to polar form (uc,e1,e2)
			for (auto itor=bijs.begin(); itor!=bijs.end(); itor++)
			{
				// compute the angle between the central axis uc and the bij
				double cos_alpha = (*itor).dot(uc);
				// compute the direction about the central axis uc
				double phi = atan2((*itor).dot(e2),(*itor).dot(e1));
				// store result in local storage span (since it is a map we implicitly sort the data thereby)
				span[phi] = cos_alpha;
			}
		
			// now we try to reduce the number of points by deleting intermediat points if
			// a) they smaller than their neighbours and
			// b) the distance between the neighbours is smaller that 2*PI/36
			const double delta_phi = 2*MO_PI/segments;
			for (int segment=0; segment<segments; segment++)
			{
				double low = -MO_PI+segment*delta_phi;
				double high = -MO_PI+(segment+1)*delta_phi;
				map<double,double>::iterator itor_low = span.lower_bound(low);
				map<double,double>::iterator itor_high = span.upper_bound(high);
				// \todo WARNING: We might do other comparison because we saved cos_alpha (not alpha itself)
				double max_alpha = 1;
				for (; itor_low!=itor_high; itor_low++)
				{
					max_alpha = min(max_alpha,itor_low->second);
				}

				// 4) construct the interference region based on the cone and the line aij and store the results in an array of triangles
				
				if (low>-0.5*MO_PI && high<-0.5*MO_PI)
					// the "backside" of the cone is connect to Ai
					triangles.push_back(pRobot->getBase(i));
				else
					// the "frontside" of the cone is connected to Aj
					triangles.push_back(pRobot->getBase(j));
				// the other two vertices are the same for both alternative triangles
				triangles.push_back(pRobot->getBase(j)+ Matrix3d::YRotationMatrix3d(acos(max_alpha))*Matrix3d::ZRotationMatrix3d(low)*bij_mean*TriangleLength);
				triangles.push_back(pRobot->getBase(j)+ Matrix3d::YRotationMatrix3d(acos(max_alpha))*Matrix3d::ZRotationMatrix3d(high)*bij_mean*TriangleLength);

				// close the open end of the cone
				triangles.push_back(pRobot->getBase(i)+ bij_mean*TriangleLength);
				triangles.push_back(pRobot->getBase(i)+ Matrix3d::YRotationMatrix3d(acos(max_alpha))*Matrix3d::ZRotationMatrix3d(low)*bij_mean*TriangleLength);
				triangles.push_back(pRobot->getBase(i)+ Matrix3d::YRotationMatrix3d(acos(max_alpha))*Matrix3d::ZRotationMatrix3d(high)*bij_mean*TriangleLength);

				// in the end, we have to add two more triangles, i.e. those that are exactly tangential to the cone in order to close the surface
			}

		}

	return true;
}


//! return the computed triangles as matrix array
bool CInterference::getInterferenceMatrix(MatrixXd& mat)
{
	if (triangles.size() == 0)
		return false;
	mat.resize(triangles.size()/3,9);
	for (unsigned int i=0; i<triangles.size(); i+=3)
	{
		mat.block(i/3,0,1,3) = triangles[i].transpose();
		mat.block(i/3,3,1,3) = triangles[i+1].transpose();
		mat.block(i/3,6,1,3) = triangles[i+2].transpose();
	}
	return true;
}

/*! load a stl file into the internal buffer 
 *  Due to the data model of the collision checker, the vertices (u,v,w) of each triangles are converted to 
 *  a point-direction form that is stored as a triplet (u, v-u, w-u)
 */
bool CInterference::loadObstacle(const string& filename)
{
	// clear the data model
	obstacle.clear();
	// load the STL file
	try {
		// we need some typedefs to match the data structure of the STL data
		typedef struct { float x,y,z; } CVector3f;
		typedef struct { CVector3f n,a,b,c; char res[2]; } CSTLRecord;
		std::ifstream file(filename.c_str(),ios::binary);
		char buf[80];	// the header
		file.read(buf,80);
		int anz = 0;
		file.read((char*)&anz,4);
		CSTLRecord patch;
		while (anz-- && !file.eof())
		{
			file.read((char*)&patch,50);
			// cast the vertex and normal data from float (STL-patch) to double (GL-patches)
			// and convert the data to the internal data model for intersection (point-direction form)
			obstacle.push_back(Vector3d(patch.a.x,patch.a.y,patch.a.z));
			obstacle.push_back(Vector3d(patch.b.x-patch.a.x,patch.b.y-patch.a.y,patch.b.z-patch.a.z));
			obstacle.push_back(Vector3d(patch.c.x-patch.a.x,patch.c.y-patch.a.y,patch.c.z-patch.a.z));
		}
//		printf("Triangles loaded: %i\n",vertices.size()/3);
	}
	catch (...)
	{ std::cout << "Error while loading STL file: " << filename.c_str() << endl; return false;}
	return true;
}

/*! perform the collision check for one pose (r,R) between all line segments define by the respective cables
 *  and the obstacle represented by the triangles stored in obstacle.
 *  \return true, if the pose is free of collsions, otherwise false.
 */
int CInterference::getObstacleCableCollisions(const Vector3d& r, const Matrix3d& R)
{
	int hits = 0;
	for (int i=0; i<pRobot->getNow(); i++)
	{
		// calculate the direction vector (this is somewhat an inline version of the inverse kinematics)
		Vector3d n_a = r + R*pRobot->getPlatform(i) - pRobot->getBase(i);
		for (vector<Vector3d>::iterator itor=obstacle.begin(); itor!= obstacle.end(); itor+=3)
		{
			if (getIntersectionTriangleLine(pRobot->getBase(i),n_a,(*itor),*(itor+1),*(itor+2)))
				hits++;
		}
	}
	return hits;
}

/*! create a regular grid depending on the current motion pattern of the robot (either 2D or 3D)
 *  and check each position for collisions.
 *
 *  \return true, if the grid was successfully generated and checked, othterwise false
 */
bool CInterference::calculateCollisions()
{
	Vector3d min,max,eps(0,0,0);
	pRobot->getBoundingBoxBase(min,max);
	// check the parameter
	if (max.x()<min.x() || max.y()<min.y() || max.z()<min.z())
		return false;
	if (eps.x()<0 || eps.y()<0 || eps.z()<0)
		return false;
	// auto adjust the accuracy of the discretisation?
	Vector3d Eps;
	if (eps.x()>0 && eps.y()>0 && eps.z()>0)
		Eps = eps;
	else
		Eps = (max-min)/10.0;	// calculate Eps for 10 or 11 steps
	
	// delete all data in the current data model
	vector<Vector3d> vertices;

	if (pRobot->getMotionPattern() == CRobotData::MP2T || pRobot->getMotionPattern() == CRobotData::MP1R2T )
	{
		// verify a regular grid given by min,max,Eps
		Vector3d pos;
		pos.z() = 0;
		for (pos.x() = min.x(); pos.x()<=max.x(); pos.x()+=Eps.x())
			for (pos.y() = min.y(); pos.y()<=max.y(); pos.y()+=Eps.y())
				vertices.push_back(pos);
	}
	else if (pRobot->getMotionPattern() == CRobotData::MP3R3T || pRobot->getMotionPattern() == CRobotData::MP2R3T || pRobot->getMotionPattern() == CRobotData::MP3T)
	{
		// verify a regular grid given by min,max,Eps
		Vector3d pos;
		for (pos.x() = min.x(); pos.x()<=max.x(); pos.x()+=Eps.x())
			for (pos.y() = min.y(); pos.y()<=max.y(); pos.y()+=Eps.y())
				for (pos.z() = min.z(); pos.z()<=max.z(); pos.z()+=Eps.z())
					vertices.push_back(pos);
	}

	// now we have the desired grid; we check each vertex with the collision checker
	Matrix3d R;
	R.setIdentity();
	for (int i=0; i<(signed)vertices.size(); i++)
		if (getObstacleCableCollisions(vertices[i],R) == 0)
			validPositions.push_back(vertices[i]);
	cout << "Number of valid poses: " << validPositions.size() << endl;
	return true;
}

/*! calculate collision matrix (i lines, j columns)
 *  \param [in] r:	cartesian position of the platform
 *  \param [in] R:	orientation of the platform
 *  \param [in] joints:	consider collisions on the plattform joints
 *  \changed:		cableCableCollisions	-> Eigen::Infinity: reject it!
											-> 0			  : collision between i and j
											-> distance		  : value between cables i and j
 *  \return: true,	if collision occurs
 *			 false, otherwise
 */
bool CInterference::calculateCableCableCollisions(const Vector3d& r, const Matrix3d& R, const bool& joints)
{
	double radius, diameter;	//< cable radius and diameter
	radius = pRobot->pCable->r_cable;
	diameter = 2*radius;
	if (!joints)
		radius = 0;

	cableCableCollisions.resize(pRobot->getNow(), pRobot->getNow());

	Vector3d ai,aj;			//< i-winch position // j-winch position
	Vector3d bi,bj;			//< i-platform point // j-platform point
	Vector3d ci,cj,cij;		//< vectors from platform pointing to the winches // cij: cross product
	Matrix3d vectorChain;	//< [ ci,-cj, (ci x cj)]
	Vector3d ddd;			//< [ di, dj, distance ]
	bool collisionFlag = false;
	cableCableCollisions.setConstant(Eigen::Infinity); //< initialise
	for (int i=0; i<pRobot->getNow(); i++)
	{
		ai = pRobot->getBase(i);
		bi = r+R*pRobot->getPlatform(i);
		for (int j=i+1; j<pRobot->getNow(); j++)
		{
			// ci*di - cj*dj + (ci x cj)*dist = bj-bi
			aj = pRobot->getBase(j);
			bj = r+R*pRobot->getPlatform(j);
			
			ci = (ai-bi);
			cj = (aj-bj);
			cij = ci.cross(cj);
			ci.normalize();
			cj.normalize();
			cij.normalize();

			vectorChain << ci, -cj, cij;
			if (!vectorChain.determinant()) 
				break; //< break, if matrix is singular
			ddd = vectorChain.colPivHouseholderQr().solve(bj-bi);

			// considering finite cables (ddd(0) = di and ddd(1) = dj)
			if (ddd(0)>-radius && ddd(0)<(bi-ai).norm()+radius && \
				ddd(1)>-radius && ddd(1)<(bj-aj).norm()+radius)
			{
				if (abs(ddd(2)) <= diameter)	//< collision! (ddd(2)=distance)
				{
					cableCableCollisions(i,j) = 0;
					cableCableCollisions(j,i) = 0;
					if (!collisionFlag)
						collisionFlag = true;
				}
				else						//< distance between cables
				{
					cableCableCollisions(i,j) = ddd(2);
					cableCableCollisions(j,i) = ddd(2);
				}
			}
		}
	}

	return collisionFlag;
}

/*! calculate cable-cable collisions inside a given box parameter, considering permutations of the orientation
 *	\param [in] minBox: minimal box position
 *  \param [in] maxBox: maximal box position
 *	\param [in] orientation: max angles
 *  \param [in] joints:	consider collisions on the plattform joints
 *  \changed: cableCableBoxCollisions	-> 1, i x j cables collision
 *										-> 0, otherwise
 *  \return: true, if collision occurs
 *			 false, otherwise
*/
bool CInterference::calculateCableCableBoxCollisions(const Vector3d& minBox,const Vector3d& maxBox, const Vector3d& orientation, const bool& joints)
{
	MatrixXd box(1,6);
	box << minBox(0),minBox(1),minBox(2),maxBox(0),maxBox(1),maxBox(2);
	cableCableBoxCollisions.resize(pRobot->getNow(), pRobot->getNow());

	// create rotations: calculate all possible angle permutations with the negative and positive values of orientation
	Matrix3d R;
	vector<Matrix3d> rotations;
	for (int x=-1; x<=1; x+=2)			//< x = [-1,1]
		for (int y=-1; y<=1; y+=2)		//< y = [-1,1]
			for (int z=-1; z<=1; z+=2)	//< z = [-1,1]
			{
				PCRL::getMatrixFromXYZ(R, Vector3d(orientation.z()*z, -orientation.y()*y, orientation.x()*x));
				rotations.push_back(R);
			}

	// create setCollisionMatrix: set of matrices for each test point (vertices with rotations), whose values are: Eigen::Infinity, -1 or distance between the cables
	vector<MatrixXd> setCollisionMatrix;	
	for (int x=0; x<6; x+=3)			//< x = [0,3]
		for (int y=1; y<6; y+=3)		//< y = [1,4]
			for (int z=2; z<6; z+=3)	//< z = [2,5]
				for (unsigned int i=0; i<rotations.size(); i++)
				{
					calculateCableCableCollisions(Vector3d(box(x), box(y), box(z)), rotations[i], joints);
					setCollisionMatrix.push_back(cableCableCollisions);
				}

	// all the Upper-Elements from the matrix of the setCollisionMatrix are analysed (matrices are symmetric)
	int signalCheck;
	int currentSignal;	//< if: -1: get signalCheck value / 0: negative / 1: positive
	cableCableBoxCollisions.setZero(); //< initialize
	for (int i=0; i<pRobot->getNow(); i++)
		for (int j=i+1; j<pRobot->getNow(); j++)
		{
			signalCheck = -1;
			for (unsigned int matrixCounter = 0;matrixCounter<setCollisionMatrix.size();matrixCounter++)
			{
				// explicite collision (distance between cables < diameter)
				if (setCollisionMatrix[matrixCounter](i,j) == 0)
				{
					cableCableBoxCollisions(i,j) = 1;
					cableCableBoxCollisions(j,i) = 1;
				}
				// implicite collision (cable-cable side change: change of the distance signal)
				else if (setCollisionMatrix[matrixCounter](i,j) != Eigen::Infinity)
				{
					if (setCollisionMatrix[matrixCounter](i,j) > 0 ? currentSignal=1:0) //< get the current distance signal
					if (signalCheck == -1) signalCheck = currentSignal;					//< get the signal of the first element
					else if (signalCheck != currentSignal)								//< if signal change -> collision!
					{
						cableCableBoxCollisions(i,j) = 1;
						cableCableBoxCollisions(j,i) = 1;
					}
				}
			}
		}

	if (cableCableBoxCollisions.maxCoeff() == 1) // if any collision verified: return true!
		return true;
	return false;
}

/*! calculate collisions betwenn platform and cables
 *  \param [in] r: cartesian position of the platform
 *  \param [in] R: orientation of the platform
 *  \changed: cablePlatformCollisions	-> 1: collision
 *										-> 0: otherwise
 *  \return: true, if collisions occurs
 *			 false, otherwise
 */
bool CInterference::calculateCablePlatformCollisions(const Vector3d& r, const Matrix3d& R)
{
	double diameter = pRobot->pCable->r_cable*2; //< cable diameter
	cablePlatformCollisions.resize(1,pRobot->getNow());

	Vector3d cableDirection;			//< a-b
	vector<Vector3d> perpendicularPlatformDirection;
	Vector3d bbmin,bbmax;				//< platform dimensions
	cablePlatformCollisions.setZero();	//< initialize
	bool positiveFlag;
	for (int i=0; i<pRobot->getNow(); i++)
	{
		cableDirection = pRobot->getBase(i)-(r+R*pRobot->getPlatform(i));
		pRobot->getBoundingBoxPlatform(bbmin,bbmax);
		
		// identify platform faces of the cable-platform contact and save perpendicular directions
		perpendicularPlatformDirection.clear();
		if (bbmin[0] == pRobot->getPlatform(i)[0])
			perpendicularPlatformDirection.push_back(Vector3d(-1, 0, 0));
		if (bbmax[0] == pRobot->getPlatform(i)[0])
			perpendicularPlatformDirection.push_back(Vector3d( 1, 0, 0));
		if (bbmin[1] == pRobot->getPlatform(i)[1])
			perpendicularPlatformDirection.push_back(Vector3d( 0,-1, 0));
		if (bbmax[1] == pRobot->getPlatform(i)[1])
			perpendicularPlatformDirection.push_back(Vector3d( 0, 1, 0));
		if (bbmin[2] == pRobot->getPlatform(i)[2])
			perpendicularPlatformDirection.push_back(Vector3d( 0, 0,-1));
		if (bbmax[2] == pRobot->getPlatform(i)[2])
			perpendicularPlatformDirection.push_back(Vector3d( 0, 0, 1));

		// dot products between face and cable direction -> collision!
		positiveFlag = false;
		for (unsigned int j=0; j<perpendicularPlatformDirection.size(); j++)
			if (cableDirection.dot(R*perpendicularPlatformDirection[j])>0)
				positiveFlag = true;
		if (!positiveFlag) //< if no positive value -> collision!
			cablePlatformCollisions(i) = 1;
	}

	if (cablePlatformCollisions.maxCoeff() == 1)
		return true;
	return false;
}

/*! calculate cable-platform collisions inside a given box parameter, considering permutations of the orientation
 *	\param [in] minBox: minimal box position
 *  \param [in] maxBox: maximal box position
 *	\param [in] orientation: max angles
 *  \changed: cablePlatformBoxCollisions	-> 1, cable-platform collision
 *											-> 0, otherwise
 *  \return: true, if collision occurs
 *			 false, otherwise
 */
bool CInterference::calculateCablePlatformBoxCollisions(const Vector3d& minBox,const Vector3d& maxBox, const Vector3d& orientation)
{
	MatrixXd box(1,6);
	box << minBox(0),minBox(1),minBox(2),maxBox(0),maxBox(1),maxBox(2);
	cablePlatformBoxCollisions.resize(1,pRobot->getNow());

	// create rotations: calculate all possible angle permutations with the negative and positive values of orientation
	Matrix3d R;
	vector<Matrix3d> rotations;
	for (int x=-1;x<=1;x+=2)
		for (int y=-1;y<=1;y+=2)
			for (int z=-1;z<=1;z+=2)
			{
				PCRL::getMatrixFromXYZ(R,Vector3d(orientation.z()*z,-orientation.y()*y,orientation.x()*x));
				rotations.push_back(R);
			}

	// create setCollisionMatrix: set of matrices for each test point (vertices with rotations)
	vector<MatrixXd> setCollisionMatrix;
	for (int x=0;x<6;x+=3)
		for (int y=1;y<6;y+=3)
			for (int z=2;z<6;z+=3)
				for (unsigned int i=0; i<rotations.size(); i++)
				{
					calculateCablePlatformCollisions(Vector3d(box(x),box(y),box(z)),rotations[i]);
					setCollisionMatrix.push_back(cablePlatformCollisions);
				}

	// verify all vertice collisions for computed poses
	cablePlatformBoxCollisions.setZero(); // initialise
	for (int i=0; i<pRobot->getNow(); i++)
		for (unsigned int matrixCounter=0;matrixCounter<setCollisionMatrix.size();matrixCounter++)
			if (setCollisionMatrix[matrixCounter](i) == 1)
				cablePlatformBoxCollisions(i) = 1;

	if (cablePlatformBoxCollisions.maxCoeff() == 1)
		return true;
	return false;
}

} // end namespace PCRL
