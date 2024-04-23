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
 *  \file   : RobotDocument.cpp
 *
 *  Project   Wirelib
 *
 *  \Author   Andreas Pott
 *
 *  \Date     09.03.2011
 *
 *********************************************************************
 */ 

#include "RobotDocument.h"

namespace PCRL {

CRobotDocument::CRobotDocument(const int now, CRobotData::MotionPatternType mp) : 
	CRobotData(now,mp), 
	WSHull(*this), 
	Crosssection(*this), 
	Interference(*this),
	Kinematics(*this),
	Stiffness(*this),
	WSGrid(*this),
	Interpolator(),
	WrenchSet(*this),
	VelocitySet(*this),
	ParametricModels(*this),
	ForceDistribution(*this),
	CableWear(*this)
{
	R=Matrix3d::ZRotationMatrix3d(0.0);
	r.setZero();
	v.setZero();
	omega.setZero();
	alpha.setZero();
	C = new Vector3d[now];
	r.setZero();
	l.resize(1,now);
	dl.resize(1,now);
	ddl.resize(1,now);
	F.resize(1,now);
	fkPoseEstimateBox_lb.resize(6, 1);
	fkPoseEstimateBox_ub.resize(6, 1);

	pu = new Vector3d[now];
	w.setZero(getDof(),1);
	pR_pulley = new Matrix3d[now];
	pR_pulley->diagonal(1); //! create diagonal matrix \todo FIXME: it seems that only the first (out of now) elements is initialized; 
	WinchDB.push_back(new CWinchParameter);
	currentWinch=WinchDB.front();
	CableDB.push_back(new CCableParameter);
	currentCable=CableDB.front();
	name="default";
	author="WireCenter";
	id="";
	desc="";
	for (int i=0; i<now; i++)
	{
		base[i].setZero();
		R_base[i].setIdentity();
		platform[i].setZero();
		R_platform[i].setIdentity();
	}
	WSHull.attachEvaluator(ForceDistribution,Kinematics,WrenchSet,VelocitySet,Stiffness);
	Crosssection.attachEvaluator(ForceDistribution,Kinematics,WrenchSet,VelocitySet,Stiffness);
	WSGrid.attachEvaluator(ForceDistribution,Kinematics,WrenchSet,VelocitySet,Stiffness);
	//CableWear.cable_segment_size=0.001; //cable segment Size in mm
}

//! free the memory
CRobotDocument::~CRobotDocument(void)
{
	// loop through the winchDB and CableDB and delete all entries
	for (WinchDBType::iterator itor=WinchDB.begin(); itor != WinchDB.end(); itor++)
		delete *itor;
	for (CableDBType::iterator itor=CableDB.begin(); itor != CableDB.end(); itor++)
		delete *itor;
	delete [] pR_pulley;
	delete [] pu;
	delete [] C;
}

//! copy construtor; at the moment this function should not be used!
CRobotDocument::CRobotDocument(const CRobotDocument &cRobotDocument) : 
	CRobotData(), 
	WSHull(*this), 
	Crosssection(*this), 
	Interference(*this),
	Kinematics(*this),
	Stiffness(*this),
	WSGrid(*this),
	Interpolator(),
	WrenchSet(*this),
	VelocitySet(*this),
	ParametricModels(*this),
	ForceDistribution(*this),
	CableWear(*this)
{
	cout << "We have used the copy constructor";
}

//! this assignment operator copies the robot's kinetostatic state of a CRobotDocument 
//! object from the source object. The configuration of the embedded 
//! algorithms is left unchanged. 
CRobotDocument& CRobotDocument::operator=(const CRobotDocument& source)
{
	// we call the assignment operator of the base class first to copy all attribute from CRobotData
	CRobotData::operator=(source);
	
	// firstly all states are resetted because this also includes resizing dynamic fields to the new size (now)
	resetStates();
	// copy the states
	R=source.R;
	r = source.r;
	v = source.v;
	omega = source.omega;
	a = source.a;
	alpha = source.alpha;
	l = source.l;
	dl = source.dl;
	ddl = source.ddl;
	F = source.F;
	w = source.w;
	for (int i=0; i<now; i++)
	{
		C[i] = source.C[i];
		pu[i] = source.pu[i];
		pR_pulley[i] = source.pR_pulley[i];
	}
	return *this;
}


/*! reset all kinematic stats of the robot at the same time, memory allocation 
 *  for the states are adjusted
 */
void CRobotDocument::resetStates()
{
	R=Matrix3d::ZRotationMatrix3d(0.0);
	r.setZero();
	v.setZero();
	omega.setZero();
	a.setZero();
	alpha.setZero();
	w.setZero();
	// resizing is only necessary if the size changed
	if (l.cols() != now)
	{
		l.resize(1,now);
		dl.resize(1,now);
		ddl.resize(1,now);
		F.resize(1,now);
		// if the size changed we have to reallocate memory for C, pu and pR_pulley
		if (C) delete [] C;
		C = new Vector3d[now];
		if (pu) delete [] pu;
		pu = new Vector3d[now];
		if (pR_pulley) delete [] pR_pulley;
		pR_pulley = new Matrix3d[now];
	}
}

//! add a winch to the winch database
void CRobotDocument::AddWinch()
{
	WinchDB.push_back(new CWinchParameter);
}

//! add a cable to the cable database
void CRobotDocument::AddCable()
{
	CableDB.push_back(new CCableParameter);
}

//! calculate inverse kinematics with simple method
bool CRobotDocument::doInverseKinematics()
{
	return Kinematics.doInverseKinematics(r,R,l);
}

//! calculate inverse kinematics for pulley mechanisms
bool CRobotDocument::doInverseKinematicsPulley()
{
	return Kinematics.doInverseKinematicsPulleyEx(r,R, l.data(),NULL, NULL, NULL, C, NULL); //!< todo: expect there is a difference in evaluating winch orientation
}

//! calculate pose Estimation
void CRobotDocument::doPoseEstimation()
{
	MatrixXd p(6,1);
	Kinematics.poseestimateMain(l,p,fkPoseEstimateBox_ub,fkPoseEstimateBox_lb,r,R, emPoseEstimate::basicSimpleBoundingbox);
	//cout<<l << ' ' << 'r: '<< p[0]<<endl; 
	//cout<<fkPoseEstimateBox_ub[0]<<fkPoseEstimateBox_ub[1]<<fkPoseEstimateBox_ub[2]<<endl;
	//cout<<fkPoseEstimateBox_lb[0]<<fkPoseEstimateBox_lb[1]<<fkPoseEstimateBox_lb[2]<<endl;
}


//! calculate forward kinematics with simple method
bool CRobotDocument::doForwardKinematics()
{	
	return Kinematics.doForwardKinematics(l,r,R, 100, emForwardAlgorithm::levmar_der, emPoseEstimate::basicSimpleBoundingbox);
}

//! calculate forward kinematics for standard model with distance method
bool CRobotDocument::doForwardKinematicsDist()
{
	return Kinematics.doForwardKinematicsDist(l,r,R, 100, emForwardAlgorithm::levmar_der, emPoseEstimate::basicSimpleBoundingbox);
}

//! calculate forward kinematics with pulley kinematics
bool CRobotDocument::doForwardKinematicsPulley()
{
	return Kinematics.doForwardKinematicsPulley(l,r,R, 100, emForwardAlgorithm::levmar_der, emPoseEstimate::basicSimpleBoundingbox);
}

//! calculate forward kinematics with pulley kinematics
bool CRobotDocument::getStructureMatrix()
{
	return ForceDistribution.getStructureMatrix(r,R);
}

//! calculate a force distrubution for the current pose
bool CRobotDocument::getForceDistribution()
{
	w = MatrixXd::Zero(getDof(),1);
	ForceDistribution.setWrenchVector(w,ForceDistribution.f,ForceDistribution.tau);
	if (!ForceDistribution.getStructureMatrix(r,R))
		return false;
	return ForceDistribution.getDistribution(w,fmin, fmax,F);
}

bool CRobotDocument::doElastoGeometricalForwardKinematics()
{
	this->w << 0,0,- this->platform_mass * 9.81, 0,0,0; // set applied wrench for visualization and data consistency

	Vector3d rot_xyz;
	Vector3d r;
	// use default initial guess
	r.setZero();
	rot_xyz.setZero();
	//getXYZFromMatrix(rot_xyz, this->R);
	// use r, R(xyz) as initial guess
	return Kinematics.doElastoGeometricalForwardKinematics(this->l,this->r,rot_xyz,this->F);
	getMatrixFromXYZ(this->R, rot_xyz); // compute rotation matrix and save in robot state
}

/*! the internal core function for saving the robot parameter into an XML document.
 *  this is a new implementation. The function stores the data into a new XML document
 *  (which shall be new generated and overwrites possibly existing data). Modifications 
 *  of existing data in the TiXmlDocument is not yet supported. 
 * 
 *  The structure of the XML file is based on the ATLAS specification document
 *  XML version 0.32 from 27.04.2016.
 *  This implementation shall be the reference implementation for generating 
 *  XML file that conform to the ATLAS XML definition. Therefore, we must take
 *  care to be very accurate here.
 *  \param doc [in] reference to an xml document where the data shall be stored
 *  \return true, if the xml document was successfully populated, otherwise false.
 */
bool CRobotDocument::saveXmlCore(TiXmlDocument& doc)
{
	const char* version="0.32";

	// create the models nodes (for support of multiple models in one file)
	TiXmlElement root("models");
	root.SetAttribute("version",version);

	// create the main robot model
	TiXmlElement xrobot("robot");

	// set the meta tags/attributes based on the information in this class
	xrobot.SetAttribute("name",name.c_str());
	xrobot.SetAttribute("id",id.c_str());
	xrobot.SetAttribute("author",author.c_str());
	xrobot.SetAttribute("description",desc.c_str());
	
	// set the main data elements of the robot tag
	setXmlNode(xrobot);

	// insert the robot tag into the xml scheme
	root.InsertEndChild(xrobot);

	// insert dummy elements with the <cable> and <winch> tags
	// we will later call the methods of the CWinchParameter und CCableParameter objects
	TiXmlElement cable("cable");
	currentCable->setXmlNode(&cable);
	TiXmlElement winch("winch");
	currentWinch->setXmlNode(&winch);
	root.InsertEndChild(cable);
	root.InsertEndChild(winch);

	// finally insert the root node into the document
	doc.InsertEndChild(root);

	return true;
}

/*! save robot parameter into an XML file; 
 * 
 *  \param filename [in] name of the xml file to be written by the libraray
 *  \return true, if the file was successfully written, otherwise false.
 */
bool CRobotDocument::saveXml(const string& filename)
{
	// create the xml document
	TiXmlDocument doc; // = new TiXmlDocument();

	// let the core function fill the xml document with the content (i.e. the robot data)
	if (!saveXmlCore(doc))
		return false;
	// save the file to disk
	doc.SaveFile(filename.c_str());
	return true;
}

/*! receive the robot XML file as a string
 *  \param [out] xml The string to store the content of the robot xml document
 *  \return true, if the file was successfully written to the string, otherwise false
 */
bool CRobotDocument::getXml(string& xml)
{
	TiXmlDocument doc;
	// populate the xmlDocument with the content
	if (!saveXmlCore(doc))
		return false;
	
	// Declare a printer    
	TiXmlPrinter printer;
	// attach it to the document you want to convert in to a std::string 
	doc.Accept(&printer);

	// fill the given string with the xml content
	xml = printer.CStr();
	return true;
}

/*! load robot parameter from an XML file; this is a new implementation
 *  The function loads data from a new file. 
 * 
 *  The structure of the XML file is based on the ATLAS specification document
 *  XML version 0.32 from 03.08.2011.
 *  This implementation shall be the reference implementation for reading
 *  XML file that conform to the ATLAS XML definition. Therefore, we must take
 *  care to be very accurate there.
 *  \todo change the name of the XML generator to saveXML as soon as it is feature-completed
 */
bool CRobotDocument::loadXml(const string& filename, int& Row, int& Col)
{
	CRobotDocument temp;

	//! this is somewhat a hack but for now we go for it:
	//! the parser accepts an XML string rather than a filename 
	//! if filename starts with a "<"

	TiXmlDocument doc;
	if (filename.length()>0 && filename.at(0)=='<')
	{
		if (!doc.Parse(filename.c_str()))
		{
			Row = doc.ErrorRow();
			Col = doc.ErrorCol();
			return false;
		}
	}
	else
	{
		if (!doc.LoadFile(filename.c_str()))
		{
			Row = doc.ErrorRow();
			Col = doc.ErrorCol();
			return false;
		}
	}
	// try to get the root element
	TiXmlElement* pModels = doc.RootElement();
	// check if it is feasible
	if (strcmp(pModels->Value(),"models"))
	{
		cout << "ERROR: root node in XML file is not <models>\n";
		return false;
	}

	TiXmlNode *pRobot = pModels->FirstChild("robot");
	if (!pRobot)
	{
		cout << "ERROR: did not find a <robot> tag\n";
		return false;
	}

	//! \todo we have to implement read methods in the three child classes CRobotData, CWinchData, and CCableData first!!!

	temp.getXmlNode(*pRobot->ToElement());

	// read the meta data from the xml file
	if (pRobot->ToElement()->Attribute("name"))
		name = pRobot->ToElement()->Attribute("name");
	if (pRobot->ToElement()->Attribute("description")) 
		desc = pRobot->ToElement()->Attribute("description");
	if (pRobot->ToElement()->Attribute("id"))
		id = pRobot->ToElement()->Attribute("id");
	if (pRobot->ToElement()->Attribute("author"))
		author = pRobot->ToElement()->Attribute("author");

	// THUS:
	// incomplete implementation

	*this = temp;
	return true;
}

//! load robot geometry from xml file
bool CRobotDocument::loadXml(const string& filename)
{
	int row,column; 
	return loadXml(filename,row,column);
}


bool CRobotDocument::printState()
{
	cout << "Robot state:" << endl;
	cout << "-------------------------------------" << endl;
	cout << "r = " << this->r.transpose().format(CleanFmt) << endl << endl;
	cout << "R = " << endl << this->R.format(CleanFmt) << endl << endl;
	cout << "l = " << this->l.format(CleanFmt) << endl << endl;
	cout << "F = " << this->F.format(CleanFmt) << endl << endl;
	cout << "w = " << this->w.transpose().format(CleanFmt) << endl;
	cout << "-------------------------------------" << endl;

	return true;
}

/*! copy algorithm settings between the members of the algorithm collection
 *  \todo The need for this function is the result of design shortcommings
 *  in the class hierachy of wirelib. Some configurations parameters exists 
 *  in multiple instances within the algorithm collection, e.g. in WorkspaceHull 
 *  and  WorkspaceCrosssection. This function copies the redundant parameters to
 *  synchronize these algorihtms.
 *  The synchronization strategy is to distribute the settings in WSHull (as leading
 *  configuration object) to the local settings in grid and cross section.
 */
void CRobotDocument::synchronizeAlgorithmSettings()
{
	// spread the workspace configuration from WSHull to the other two objects
	WSGrid.getAlgorithmParameter(WSHull);
	Crosssection.getAlgorithmParameter(WSHull);
}

//! add pointers to reflection objects of all algorithm objects to the list
//! \param algList [in/out] the list objects to which the reflectors are added
void CRobotDocument::getAlgorithmReflectorList(list<CReflection*> &algList)
{
	algList.push_back(&WSHull.Reflector());
	algList.push_back(&Interference.Reflector());
	algList.push_back(&Kinematics.Reflector());
	algList.push_back(&Crosssection.Reflector());
	algList.push_back(&Stiffness.Reflector());
	algList.push_back(&WSGrid.Reflector());
//	algList.push_back(&Interpolator.Reflector());
	algList.push_back(&WrenchSet.Reflector());
	algList.push_back(&ParametricModels.Reflector());
	algList.push_back(&ForceDistribution.Reflector());
}

//! save the configuration of all algorithms to an xml file
bool CRobotDocument::saveAlgorithmConfiguration(const string& filename)
{
	// list the algorithms to saved in the config file
	list<CReflection*> x;
	getAlgorithmReflectorList(x);
	// write the xml data into the file
	return CReflection::writeAggrigator(filename,x);
}

//! read the configuration data for all algorithm objects under reflection
//! from the given file.
bool CRobotDocument::loadAlgorithmConfiguration(const string& filename)
{
	// list the algorithms to be saved in the config file
	list<CReflection*> x;
	getAlgorithmReflectorList(x);
	// write the xml data into the file
	return CReflection::readAggrigator(filename,x);
}

bool CRobotDocument::saveWSGrid(const string& filename)
{
	return WSGrid.saveWSGrid(filename);
}

}	// end of namespace PCRL
