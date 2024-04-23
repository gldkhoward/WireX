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

/*! \file RobotDocument.h
 *
 *	\author   Andreas Pott
 *
 *  \brief The class CRobotDocument is an application level objects (facade) 
 *  that aggregates data and algorithm classes for practical calculations with 
 *  cable robots. Furthermore, CRobotDocument represents the specified XML 
 *  data structure. Therefore, it also encapsulates a database for 
 *  winch and cable parameters.
 *  
 *  This class was origionally designed for WireCenter and later integrated into 
 *  Wirelib. Therefore, no dependencies to the wirecenter application shall be 
 *  integrated. Contrary, functions from CWirCenterDoc that are independent 
 *  from both OS and GUI shall be integrated into this class.
 *
 *  \par Usage and behaviour
 *  The class is designed to behave like a state machine. It mostly connects
 *  the elements of the WireLib libraray in a useful way, where possible
 *  does not give access to all features. CRobotDocument is designed to make
 *  it easy to work with the class in WireLib by providing some "glue" to hold
 *  the structure together.
 * 
 *  \par Python
 *  The python scripting of WireCenter and WiPy is introduced on top of the 
 *  level of CRobotDocument since the details of the scripting systems should 
 *  be seperated from WireLib. Although it is not an integral part of wirelib
 *  the python bindings for wirelib are shipped as a supplement library
 *  called WireLibPyBindings. This class deals as facade for the python 
 *  interface by providing all exported functions through one convinient 
 *  objects.
 *
 */

#pragma once

#include <string>
#include <list>
#include <vector>
#include "RobotData.h"
#include "ApplicationRequirement.h"
#include "Workspace.h"
#include "Interference.h"
#include "Kinematics.h"
#include "CableWear.h"
#include <motionPlanning/NcInterpolator.h>
#include "GeometryGenerator.h"


namespace PCRL {

class CRobotDocument : public CRobotData
{
public:
	typedef vector<CWinchParameter*> WinchDBType;
	typedef vector<CCableParameter*> CableDBType;
	
	//! meta information about the robot
	string name;							//!< the name of the robot
	string author;							//!< the creator/manufacturer
	string id;								//!< the unique id string of the robot
	string desc;							//!< a longer description text about the robot

	//! data base for design of robots
	CApplicationRequirement AppReq;			//!< application requirements
	WinchDBType WinchDB;					//!< collection of technical parameter of different winches
	CWinchParameter* currentWinch;			//!< the winch from the DB used for the current robot
	CableDBType CableDB;					//!< collection of technical parameter of different cables
	CCableParameter* currentCable;			//!< the cable from the DB used for the current robot

	//! the algorithms objects
	CWorkspaceDifferentialHull WSHull;		//!< workspace calcuation
	CWorkspaceCrosssection Crosssection;	//!< a cross-section of the workspace
	CWorkspaceGrid WSGrid;					//!< workspace calculation based on discrete distribution of positions
	CInterference Interference;				//!< interference calculation
	CElastoKinematics Kinematics;			//!< kinematic transformation
	CWrenchSet WrenchSet;					//!< determination, if a wrench set is feasible
	CVelocitySet VelocitySet;				//!< determination, if a velocity set is feasible
	CForceDistribution ForceDistribution;	//!< structure matrix and algorithms for force distribution
	CStiffness Stiffness;					//!< determination of the stiffness of the robot
	CNcInterpolator Interpolator;			//!< trajectory planning and interpolation based on a nc programm
	CCableWear CableWear;					//!< track cable wear for an interpolated trajectory

	//! the geometry generator objects
	CGeometryGeneratorList ParametricModels;//!< the database for the parametrisations

	//! the robot's actual kinematic state
	Matrix3d R;								//!< the current orientation of the robot
	Vector3d r;								//!< the current position of the robot
	Vector3d v,omega;						//!< the twist of the platform, i.e. linear and angular velocity
	Vector3d a,alpha;						//!< linear and angular acceleration of the platform
	Vector3d* C;							//!< The exit point from the pulley calculated by doInverseKinematicsPulleyEx
	MatrixXd l;								//!< the current wire length
	MatrixXd dl;							//!< the current velocity of the cables
	MatrixXd ddl;							//!< the current acceleration of the cables (not in use yet)
	MatrixXd F;								//!< the current cable forces
	Vector3d* pu;							//!< unit vectors in cable direction
	MatrixXd w;								//!< current applied wrench
	Matrix3d* pR_pulley;					//!< pointer to  the current pulley rotations with respect to the local frame defined by R_base
	MatrixXd fkPoseEstimateBox_lb;			//!< current pose estimator lowerbound
	MatrixXd fkPoseEstimateBox_ub;			//!< current pose estimator upperbound

	// constructor and destructur
	explicit CRobotDocument(const int now=8, CRobotData::MotionPatternType mp=CRobotData::MP3R3T);
	~CRobotDocument(void);
	//copy construtor and overloaded "="-operator:
	CRobotDocument(const CRobotDocument& cRobotDocument);
	CRobotDocument& operator=(const CRobotDocument& cRobotDocument);

	void AddWinch();
	void AddCable();

	// driver functions to perform the call to the algorithm functions using 
	// the state variables of this class
	bool doInverseKinematics();
	bool doInverseKinematicsPulley();
	void doPoseEstimation();
	bool doForwardKinematics();
	bool doForwardKinematicsDist();
	bool doForwardKinematicsPulley();
	bool getStructureMatrix();
	bool getForceDistribution();
	bool doElastoGeometricalForwardKinematics();

	// methods
	//! reset all internal states and resize the internal memory if necessary
	void resetStates();

	//! the internal function which actually stored the data in a xml structure
	bool saveXmlCore(TiXmlDocument& doc);
	//! save the robot data into an xml-based file
	bool saveXml(const string& filename);
	//! a driver function that provides the xml data of the robot as string
	bool getXml(string& xml);
	//! load the robot data from an xml-based file
	bool loadXml(const string& filename);
	//! load the robot data from an xml-based file with passing back error information from TinyXML
	bool loadXml(const string& filename, int& Row, int& Col);

	//! open the specified xml file and update this file with the data of the current robot structure
//	bool updateXml(const string& filename);
	
	//! print the current robot state
	bool printState();

	//! copy algorithm settings between the members of the algorithm collection
	void synchronizeAlgorithmSettings();

	//! add all algorithm reflectors to the list
	void getAlgorithmReflectorList(list<CReflection*> &algList);
	//! save the configuration of all algorithms to an xml file
	bool saveAlgorithmConfiguration(const string& filename);
	//! load the configuration of all algorithms from an xml file
	bool loadAlgorithmConfiguration(const string& filename);

	//! save WSGrid
	bool saveWSGrid(const string& filename);
};

}	// end of namespace PCRL