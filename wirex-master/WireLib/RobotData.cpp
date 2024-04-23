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
 *  \file   : RobotData.cpp
 *
 *  Project   Wirelib
 *
 *  \Author   Andreas Pott
 *
 *  \Date     10.10.2008 (part of MoWiRoGeometry)
 *			  15.12.2009 (refactored by asp)
 *
 *********************************************************************
 */ 

#include "WireLib.h"
#include "RobotData.h"
#include <tinyXML/tinyxml.h>
#include "motionPlanning/Utilities.h"

namespace PCRL {

////////////////////////////////////////////////////////////////////////////////
// Implementation of the class CCableParameter
////////////////////////////////////////////////////////////////////////////////

/*! create and write the attributes in the given node "parent".
 *  \remark the names if the attributes are draft which are not yet specified
 *  in the XML definition.
 */
bool CCableParameter::setXmlNode(TiXmlElement* parent) const
{
	parent->SetAttribute("name",name.c_str());
	parent->SetDoubleAttribute("elasticity",E_wire);
	parent->SetDoubleAttribute("radius",r_cable);
	parent->SetDoubleAttribute("breaking_load",F_breakingload);
	parent->SetDoubleAttribute("damping",damping);
	parent->SetDoubleAttribute("weight",weight);
	parent->SetDoubleAttribute("minimum_load",minimum_load);
	parent->SetDoubleAttribute("max_length",max_length);
	parent->SetDoubleAttribute("meter_cost", meter_cost);
	parent->SetAttribute("material", material.c_str());
	return true;
}

/*
bool CCableParameter::getXmlNode(TiXmlElement* parent)
{	
	if (!parent)
		return false;
	string name_tmp;
	double radius_tmp, F_breakingload_tmp, damping_tmp, weight_tmp;
	if (parent->Attribute("name",&name_tmp) == TIXML_SUCCESS && // read the values from the tags and write into temp variables
		parent->QueryDoubleAttribute("radius",&radius_tmp) == TIXML_SUCCESS && 
		parent->QueryDoubleAttribute("breakingload",&F_breakingload_tmp) == TIXML_SUCCESS && 
		parent->QueryDoubleAttribute("damping",&damping_tmp) == TIXML_SUCCESS && 
		parent->QueryDoubleAttribute("weight",&weight_tmp) == TIXML_SUCCESS 
		
	{
		name=name_tmp;
		radius = radius_tmp;
		F_breakingload = F_breakingload_tmp;
		damping = damping_tmp;
		weight = weight_tmp;

		return true;
	}
	else 
		return false;
}
*/

bool CCableParameter::loadXML(TiXmlElement* parent,int id) //could rename to  bool getXmlNode(TiXmlElement* parent);
{
	CCableParameter T;

	TiXmlNode *cable = parent->FirstChild("cable");
	if (!cable)
	{
		cout << "WARNING: there is no node <cable> in function CCableParameter::loadXML\n"; //if this occurs it is probably a bad function call
		return false;
	}

	for (int i=0; i<id-1;i++)
		cable = cable->NextSibling("cable");
	if (!cable)
	{
		cout << "WARNING: there are not enough <cable>-Tags\n"; //if this occurs it is probably a bad function call
		return false;
	}
	
	if (cable->ToElement()->Attribute("name"))//VLS: Apparently, QueryStringAttribute has better error checking
		T.name = cable->ToElement()->Attribute("name");	
	else	
		cout << "WARNING: there is no attribute name in <winch>; assuming default value\n";
	
	if (cable->ToElement()->QueryDoubleAttribute("elasticity",&T.E_wire) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute elasticity in <cable>; assuming default value\n";
	if (cable->ToElement()->QueryDoubleAttribute("radius",&T.r_cable) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute radius in <cable>; assuming default value\n";
	if (cable->ToElement()->QueryDoubleAttribute("breaking_load",&T.F_breakingload) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute breaking_load in <cable>; assuming default value\n";
	if (cable->ToElement()->QueryDoubleAttribute("damping",&T.damping) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute damping in <cable>; assuming default value\n";
	if (cable->ToElement()->QueryDoubleAttribute("weight",&T.weight) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute weight in <cable>; assuming default value\n";
	if (cable->ToElement()->QueryDoubleAttribute("minimum_load",&T.minimum_load) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute minimum_load in <cable>; assuming default value\n";
	if (cable->ToElement()->QueryDoubleAttribute("max_length",&T.max_length) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute max_length in <cable>; assuming default value\n";
	if (cable->ToElement()->QueryDoubleAttribute("meter_cost",&T.meter_cost) != TIXML_SUCCESS)
		cout << "WARNING: there is no attribute meter_cost in <cable>; assuming default value\n";
	if (cable->ToElement()->Attribute("material"))
		T.material = cable->ToElement()->Attribute("material");
	else
		cout << "WARNING: there is no attribute material in <cable>; assuming default value\n";
	
	*this = T;
	return true;
}

/*! store the parameters of the cable in an XML file
 *  \param filename [in] name of the file to be create, on extension is appended
 *  \return true, if the file was writtten, otherwise false
 */
bool CCableParameter::saveXML(const string& filename)
{
	TiXmlDocument doc(filename.c_str());
	if (!doc.LoadFile())
	{
    		cout<<"No XML structure found:"<<endl;
	}
	TiXmlElement* root = new TiXmlElement("models");
	TiXmlElement* cable = new TiXmlElement("cable"); //create cable pointer

	if (doc.RootElement()) //is the document empty?
		root = doc.RootElement();
	int cc = 1; //cablecount
	int oldcableid = 1;
	bool newcable = true; 

	//check for existing winch with the same name
	for (TiXmlNode* cablecount = root->FirstChild("cable"); cablecount; cablecount = cablecount->NextSibling("cable")) //Test for more than one <winch>-tag the following if statement then only reads winch data if only one exist
	{
		if (cablecount->ToElement()->Attribute("name"))
		{
			if (name == cablecount->ToElement()->Attribute("name"))
			{
				cable = cablecount->ToElement(); //point to existing winch to ovewrite
				cout<<"Overwriting existing cable: "<<name<<endl;
				newcable = false;
				cable->QueryIntAttribute("id",&oldcableid);
			}
		}
		cc++;//VLS: should we count id?
	}
			
	cable->SetAttribute("id",oldcableid); //add id
	setXmlNode(cable); //call for information from RobotDoc
		
	if (newcable)
	{
		cable->SetAttribute("id",cc); //amend id
		root->LinkEndChild(cable);
	}
			
	if (!doc.RootElement())
		doc.LinkEndChild(root);

	if (doc.SaveFile(filename.c_str()))
		return true;
	else
		return false;
}

//! define the unique name for the cable type
bool CCableParameter::setname(const string& name)
{
	this->name = name;
	return true;
}

//! define the young modulus of the cable material [N/m²] 
bool CCableParameter::setE_wire(const double& E_wire)
{
	this->E_wire = E_wire;
	return true;
}

//! define the radius of the cable [m]
bool CCableParameter::setr_cable(const double& r_cable)
{
	this->r_cable = r_cable;
	return true;
}

//! define the maximum feasible load of the cable before it breaks down
bool CCableParameter::setF_breakingload(const double& F_breakingload)
{
	this->F_breakingload = F_breakingload;
	return true;
}

//! define the specific damping constant (as damping per length)
bool CCableParameter::setdamping(const double& damping)
{
	this->damping = damping;
	return true;
}

//! define the specific weight per length (kg/m)
bool CCableParameter::setweight(const double& weight)
{
	this->weight = weight;
	return true;
}


////////////////////////////////////////////////////////////////////////////////
// Implementation of the class CWinchParameter
////////////////////////////////////////////////////////////////////////////////

/*! create and write the attributes in the given node "parent".
 *  \remark the names if the attributes are draft which are not yet specified
 *  in the XML definition.
 */
bool CWinchParameter::setXmlNode(TiXmlElement* winch) const
{
	//insert char array
	winch->SetAttribute("name",name.c_str());
	winch->SetAttribute("description", description.c_str());

	// load cable related attributes
	TiXmlElement* cable;

	//check for existing tag:<cable>
	if (winch->FirstChild("cable"))
		cable = winch->FirstChild("cable")->ToElement();
	else
		cable = new TiXmlElement("cable");

	//load attributes
	cable->SetAttribute("nr_cables",nr_cables);
	cable->SetDoubleAttribute("f_max",f_max);
	cable->SetDoubleAttribute("f_hold_max",f_hold_max);
	cable->SetDoubleAttribute("string_orientation",string_orientation);
	cable->SetDoubleAttribute("dl_max",dl_max);
	cable->SetDoubleAttribute("l_min",l0);
	cable->SetDoubleAttribute("max_velocity",v);
	cable->SetDoubleAttribute("max_acceleration",a);
	cable->SetDoubleAttribute("max_jerk",j);
	cable->SetDoubleAttribute("position_sensor_accuracy",position_sensor_accuracy);
	cable->SetDoubleAttribute("force_sensor_accuracy",force_sensor_accuracy);
	cable->SetDoubleAttribute("spring_constant",spring_constant);
	cable->SetDoubleAttribute("length_offset",length_offset);
	
	if (!winch->FirstChild("cable"))
		winch->InsertEndChild(*cable); //if not tag:<cable> does not exist add to <winch>
	
	// load motor related attributes
	TiXmlElement* motor;

	//check for existing tag:<motor>
	if (winch->FirstChild("motor"))
		motor = winch->FirstChild("motor")->ToElement();
	else
		motor = new TiXmlElement("motor");
	
	//load attributes
	motor->SetAttribute("motor_type", motor_type.c_str());
	motor->SetDoubleAttribute("inertia", inertia);
	motor->SetDoubleAttribute("mass",mass);
	motor->SetDoubleAttribute("max_power",max_power);
	motor->SetDoubleAttribute("angular_velocity",omega_motor);
	motor->SetDoubleAttribute("angular_acceleration",alpha_motor);
	motor->SetDoubleAttribute("gear_ratio",gear_ratio);
	
	if (!winch->FirstChild("motor"))
		winch->InsertEndChild(*motor); //if not tag:<motor> does not exist add to <winch>
	
	// load the drum related attributes
	TiXmlElement* drum;

	//check for existing tag:<motor>
	if (winch->FirstChild("drum"))
		drum = winch->FirstChild("drum")->ToElement();
	else
		drum = new TiXmlElement("drum");

	//load attributes
	drum->SetDoubleAttribute("r_drum",r_drum);
	drum->SetDoubleAttribute("l_drum",l_drum);
	drum->SetDoubleAttribute("nr_groove",nr_groove);
	drum->SetDoubleAttribute("dfriction",dfriction);
	drum->SetDoubleAttribute("sfriction",sfriction);
	drum->SetDoubleAttribute("backlash",backlash);
	
	if (!winch->FirstChild("drum"))
		winch->InsertEndChild(*drum); //if not tag:<drum> does not exist add to <winch>
	
	// load the guidinglimit related attributes
	TiXmlElement* guidinglimit;

	//check for existing tag:<guidinglimit>
	if (winch->FirstChild("guidinglimit"))
		guidinglimit = winch->FirstChild("guidinglimit")->ToElement();
	else
		guidinglimit = new TiXmlElement("guidinglimit");
	
	//load attributes
	guidinglimit->SetDoubleAttribute("apeture",aperture);
	guidinglimit->SetDoubleAttribute("cone_offset",cone_offset);

	if (!winch->FirstChild("guidinglimit"))
		winch->InsertEndChild(*guidinglimit); //if not tag:<guidinglimit> does not exist add to <winch>

	//!todo: xml implementation for linear actuators

	//load attributes
	TiXmlElement* anchoring;

	if (winch->FirstChild("anchoring"))
		anchoring = winch->FirstChild("anchoring")->ToElement();
	else
		anchoring = new TiXmlElement("anchoring");

	anchoring->SetAttribute("anchoring_type", anchoring_type.c_str());
	anchoring->SetDoubleAttribute("max_force", max_force);
	anchoring->SetDoubleAttribute("x",acting_moment.x());
	anchoring->SetDoubleAttribute("y",acting_moment.y());
	anchoring->SetDoubleAttribute("z",acting_moment.z());
	
	if (!winch->FirstChild("anchoring"))
		winch->InsertEndChild(*anchoring);

	return true;
}

//! This function receives the parent element for which the XML, and which winch (optional)
bool CWinchParameter::loadXML(TiXmlElement* parent, int id=1) 
{
	CWinchParameter T;

	TiXmlNode *winch = parent->FirstChild("winch");
	if (!winch)
	{
		cout << "WARNING: there is no node <winch> in function CWinchParameter::loadXML\n"; //if this occurs it is probably a bad function call
		return false;
	}

	for (int i=0; i<id-1;i++)
		winch = winch->NextSibling("winch");
	if (!winch)
	{
		cout << "WARNING: there are not enough <winch>-Tags\n"; //if this occurs it is probably a bad function call
		return false;
	}
	else
	{
	//load attributes
	if (winch->ToElement()->Attribute("name"))//VLS: Apparently, QueryStringAttribute has better error checking
		T.name = winch->ToElement()->Attribute("name");	
	else	
		cout << "WARNING: there is no attribute name in <winch>; assuming default value\n";
	if (winch->ToElement()->Attribute("description"))
		T.description = winch->ToElement()->Attribute("description");
	else
		cout << "WARNING: there is no attribute description in <winch>; assuming default value\n";

	// load cable related attributes
	TiXmlNode *cable = winch->FirstChild("cable");
	if (!cable)
		cout << "WARNING: there is no node <cable>\n";
	else
	{
		if (cable->ToElement()->QueryIntAttribute("nr_cables",&T.nr_cables) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute nr_cables in <cable>; assuming default value\n";
		if (cable->ToElement()->QueryDoubleAttribute("f_max",&T.f_max) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute f_max in <cable>; assuming default value\n";
		if (cable->ToElement()->QueryDoubleAttribute("f_hold_max",&T.f_hold_max) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute f_hold_max in <cable>; assuming default value\n";
		if (cable->ToElement()->QueryDoubleAttribute("string_orientation",&T.string_orientation) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute string_orientation in <cable>; assuming default value\n";
		if (cable->ToElement()->QueryDoubleAttribute("dl_max",&T.dl_max) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute dl_max in <cable>; assuming default value\n";	
		if (cable->ToElement()->QueryDoubleAttribute("l_min",&T.l0) != TIXML_SUCCESS)				//!!!! Careful, definition not identical
			cout << "WARNING: there is no attribute l_min in <cable>; assuming default value\n";
		if (cable->ToElement()->QueryDoubleAttribute("max_velocity",&T.v) != TIXML_SUCCESS)		//!!!! Careful, definition not identical
			cout << "WARNING: there is no attribute max_velocity in <cable>; assuming default value\n";
		if (cable->ToElement()->QueryDoubleAttribute("max_acceleration",&T.a) != TIXML_SUCCESS)	//!!!! Careful, definition not identical
			cout << "WARNING: there is no attribute max_acceleration in <cable>; assuming default value\n";
		if (cable->ToElement()->QueryDoubleAttribute("max_jerk",&T.j) != TIXML_SUCCESS)			//!!!! Careful, definition not identical
			cout << "WARNING: there is no attribute max_jerk in <cable>; assuming default value\n";
		if (cable->ToElement()->QueryDoubleAttribute("position_sensor_accuracy",&T.position_sensor_accuracy) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute position_sensor_accuracy in <cable>; assuming default value\n";	
		if (cable->ToElement()->QueryDoubleAttribute("force_sensor_accuracy",&T.force_sensor_accuracy) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute force_sensor_accuracy in <cable>; assuming default value\n";	
		if (cable->ToElement()->QueryDoubleAttribute("spring_constant",&T.spring_constant) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute spring_constant in <cable>; assuming default value\n";	
		if (cable->ToElement()->QueryDoubleAttribute("length_offset",&T.length_offset) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute lenght_offset in <cable>; assuming default value\n";	
	}
		
		
	// load motor related attributes
	TiXmlNode *motor = winch->FirstChild("motor");
	if (!motor)
		cout << "WARNING: there is no node <motor>\n";
	else
	{
		if (motor->ToElement()->Attribute("motor_type"))		
			T.motor_type = motor->ToElement()->Attribute("motor_type");		//VLS: Apparently, QueryStringAttribute has better error checking
		else
			cout << "WARNING: there is no attribute motor_type in <motor>; assuming default value\n";
		if (motor->ToElement()->QueryDoubleAttribute("inertia",&T.inertia) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute inertia in <motor>; assuming default value\n";
		if (motor->ToElement()->QueryDoubleAttribute("mass",&T.mass) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute mass in <motor>; assuming default value\n";
		if (motor->ToElement()->QueryDoubleAttribute("max_power",&T.max_power) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute max_power in <motor>; assuming default value\n";
		if (motor->ToElement()->QueryDoubleAttribute("angular_velocity",&T.omega_motor) != TIXML_SUCCESS)			//!!!! Careful, definition not identical
			cout << "WARNING: there is no attribute angluar_velocity in <motor>; assuming default value\n";
		if (motor->ToElement()->QueryDoubleAttribute("angular_acceleration",&T.alpha_motor) != TIXML_SUCCESS)		//!!!! Careful, definition not identical
			cout << "WARNING: there is no attribute angular_acceleration in <motor>; assuming default value\n";
		if (motor->ToElement()->QueryDoubleAttribute("gear_ratio",&T.gear_ratio) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute gear_ratio in <motor>; assuming default value\n";
	}
	
	// load the drum related attributes
	TiXmlNode *drum = winch->FirstChild("drum");
	if (!drum)
		cout << "WARNING: there is no node <drum>\n";
	else
	{
		if (drum->ToElement()->QueryDoubleAttribute("r_drum",&T.r_drum) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute r_drum in <drum>; assuming default value\n";
		if (drum->ToElement()->QueryDoubleAttribute("l_drum",&T.l_drum) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute l_drum in <drum>; assuming default value\n";
		if (drum->ToElement()->QueryDoubleAttribute("nr_groove",&T.nr_groove) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute nr_groove in <drum>; assuming default value\n";
		if (drum->ToElement()->QueryDoubleAttribute("dfriction",&T.dfriction) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute dfriction in <drum>; assuming default value\n";
		if (drum->ToElement()->QueryDoubleAttribute("sfriction",&T.sfriction) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute sfriction in <drum>; assuming default value\n";
		if (drum->ToElement()->QueryDoubleAttribute("backlash",&T.backlash) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute backlash in <drum>; assuming default value\n";
	}
	
	// load the guidinglimit related attributes
	TiXmlNode *guidinglimit = winch->FirstChild("guidinglimit");
	if (!guidinglimit)
		cout << "WARNING: there is no node <guidinglimit>\n";
	else
	{
		if (guidinglimit->ToElement()->QueryDoubleAttribute("apeture",&T.aperture) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute apeture in <drum>; assuming default value\n";
		if (guidinglimit->ToElement()->QueryDoubleAttribute("cone_offset",&T.cone_offset) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute cone_offset in <drum>; assuming default value\n";		//!!!!!!!!!Should be Vector!
	} 
	
	// loading was successful; copy the loaded data into the current instance
	TiXmlNode *lineardrive = winch->FirstChild("lineardrive");
	if (lineardrive)
		cout << "WireCenter does not accomodate for linearly actuated winches... yet";

	TiXmlNode *anchoring = winch->FirstChild("anchoring");
	if (!anchoring)
		cout << "WARNING: there is no node <anchoring>\n";
	else
	{
		if (anchoring->ToElement()->Attribute("anchoring_type"))		
			T.anchoring_type = anchoring->ToElement()->Attribute("anchoring_type");		//VLS: Apparently, QueryStringAttribute has better error checking
		else
			cout << "WARNING: there is no attribute anchoring_type in <anchoring>; assuming default value\n";
		
		if (anchoring->ToElement()->QueryDoubleAttribute("max_force",&T.max_force) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute max_force in <anchoring>; assuming default value\n";
		if (anchoring->ToElement()->QueryDoubleAttribute("x",&T.acting_moment.x()) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute x in <anchoring>; assuming default value\n";
		if (anchoring->ToElement()->QueryDoubleAttribute("y",&T.acting_moment.y()) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute y in <anchoring>; assuming default value\n";
		if (anchoring->ToElement()->QueryDoubleAttribute("z",&T.acting_moment.x()) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute z in <anchoring>; assuming default value\n";
	} 

	}

	*this = T;
	return true;
}

//! store the winche parameters in a file  named filename
//! \return true, if succesful
bool CWinchParameter::saveXML(const string& filename)
{
	TiXmlDocument doc(filename.c_str());
	if (!doc.LoadFile())
	{
    		cout<<"No XML structure found:"<<endl;
	}

	TiXmlElement* root = new TiXmlElement("models");
	TiXmlElement* winch = new TiXmlElement("winch"); //create winch pointer

	if (doc.RootElement()) //is the document empty?
		root = doc.RootElement();
	int wc = 1; //winchcount
	int oldwinchid = 1;
	bool newwinch = true; 

	//check for existing winch with the same name
	for (TiXmlNode* winchcount = root->FirstChild("winch"); winchcount; winchcount = winchcount->NextSibling("winch")) //Test for more than one <winch>-tag the following if statement then only reads winch data if only one exist
	{
		if (winchcount->ToElement()->Attribute("name"))
		{
			if (name == winchcount->ToElement()->Attribute("name"))
			{
				winch = winchcount->ToElement(); //point to existing winch to ovewrite
				cout<<"Overwriting existing winch: "<<name<<endl;
				newwinch = false;
				winch->QueryIntAttribute("id",&oldwinchid);
			}
		}
		wc++;//VLS: should we count id?
	}
			
	winch->SetAttribute("id",oldwinchid); //add id
	setXmlNode(winch); //call for information from RobotDoc
		
	if (newwinch)
	{
		winch->SetAttribute("id",wc); //amend id
		root->LinkEndChild(winch);
	}
			
	if (!doc.RootElement())
		doc.LinkEndChild(root);

	if (doc.SaveFile(filename.c_str()))
		return true;
	else
		return false;
}


//!< get the effective radius of the winch (including spool traverse)
double  CWinchParameter::getWinch_EffectiveRadius()
{
	double slope_drum = l_drum/(n_drum*2.0*MO_PI); // l_drum/n_drum is the movement of the spool traverse for 360°

	// sqrt(r^2+slope^2)+spool_direction*slope
	return sqrt(pow(r_drum,2.0)+pow(spool_direction*slope_drum,2.0)) + spool_direction*slope_drum;
}



//!< get the equivalent mass in kg of the rotating parts of the winch
double  CWinchParameter::getWinch_EquivalentMass()
{
	double inertia_total; // total inertia of the actuator unit including motor, gear box, winch
	double m_winch; // equivalent mass of the rotating parts
	
	double inertia_motor = inertia; // according to xml-spec, inertia corresponds to motor inertia
	double inertia_gearbox; // respective the motor axis
	double inertia_winch; // respective the drum axis

	inertia_gearbox = 0.0; // optional, may be included in "inertia"
	inertia_winch = 0.0;  // optional, may be included in "inertia"

	// to transform inertia from drum system to motor system, one has to divide by gear_ratio^2
	inertia_total = inertia_motor + inertia_gearbox + inertia_winch*(pow(gear_ratio,2.0)); // kg m², refered to motor axis

	// to transform rotational inertia into translational, one has to divide by radius^2
	m_winch = inertia_total / (pow(gear_ratio,2.0)) / pow(getWinch_EffectiveRadius(),2.0);  // m = J * i² / r²

	return m_winch;
}


////////////////////////////////////////////////////////////////////////////////
// Implementation of the class CRobotData
////////////////////////////////////////////////////////////////////////////////


CRobotData::CRobotData(const unsigned int& numberOfWires, const MotionPatternType mp)
{
	motionPattern = mp;
	robotKinematicsModel = CRobotData::FIXED; // set default values for kinematic model (default: no pulleys regarded)
	elasticityModel = CRobotData::NONELASTIC;	// set elasticity model (default: no elasticities regarded)
	platform = base = 0;
	R_base = R_platform = 0;
	pCable = 0;
	pWinch = 0;
	r_pulley = 0;
	setMotionPattern(numberOfWires,mp);
	fmin = 1;
	fmax = 10;
	lmin = 0;
	lmax = 10;
	for (int i=0; i<now; i++)
	{
		R_base[i] = R_platform[i] = Matrix3d::ZRotationMatrix3d(0);
	}

	platform_name = "default_platform";
	platform_desc = "default_desc";
	platform_mass = 25;
	platform_swingangle = 50;
	platform_inertia <<  2,0,0,
						 0,3,0,
						 0,0,4;
	centerofgravity << 0,1,0.2;

	controller_ipotakt = 4;

/*	application_name = "default_application";
	application_id = 1;
	application_desc = "default_application_description";
	application_load_mass = 25;
	application_load_moment = 2;
	application_min_cableload = 40;
	application_max_cableload = 1000;*/
}

CRobotData::~CRobotData(void)
{
	if (platform)
		delete [] platform;
	if (base)
		delete [] base;
	if (R_platform)
		delete [] R_platform;
	if (R_base)
		delete [] R_base;
	if (r_pulley)
		delete [] r_pulley;
	if (pCable)
		delete [] pCable;
	if (pWinch)
		delete [] pWinch;
}

/*! Define the motion pattern and the number of wires.
 *  The functions reserves the required memory for the both the base and 
 *  platform vectors and matrizes. All data in the geometry entries is
 *  lost.
 *  \param numberOfWires [in] the number of cable m for the robot
 *  \param mp [in] the motion pattern of the robot to set; must be a valid element of the enum type
 */
void CRobotData::setMotionPattern(const int& numberOfWires, const MotionPatternType mp)
{
	if (numberOfWires<0)
		return;
	motionPattern = mp;
	switch (motionPattern)
	{
	case MP1T: dof = 1; break;
	case MP2T: dof = 2; break;
	case MP3T: dof = 3; break;
	case MP1R2T: dof = 3; break;
	case MP2R3T: dof = 5; break;
	case MP3R3T: dof = 6; break;
	}

	now = numberOfWires;

	// clean up the memory and reserve new memory according to DOF and now
	if (platform) 
		delete [] platform;
	platform = new Vector3d[now];
	if (R_platform)
		delete [] R_platform;
	R_platform = new Matrix3d[now];
	if (base)
		delete [] base;
	base = new Vector3d[now];
	if (R_base)
		delete [] R_base;
	R_base = new Matrix3d[now];
	if (r_pulley)
		delete [] r_pulley;
	r_pulley = new double[now];
	if (pCable)
		delete [] pCable;
	pCable = new CCableParameter[now];
	if (pWinch)
		delete [] pWinch;
	pWinch = new CWinchParameter[now];

	//initialize rotation matrices as identity matrix
	for (int i=0;i<now;i++)
	{
		R_platform[i].setIdentity();
		R_base[i].setIdentity();
		r_pulley[i] = 0.1; // default parameter for pulley radius (defined 
	}
}

//! assign Values for Object CRobotData equal to that of a different Object CRobotData src
//! \todo FIXME: the content of pCable and pWinch is not copied! The implementation for the cables
//! would be straight forward but for pWinch no copy or assignment operator= exists!
CRobotData& CRobotData::operator=(const CRobotData& src)
{
	setMotionPattern(src.now,src.motionPattern);
	for (int i=0; i<now; i++)
	{
		base[i] = src.base[i];
		R_base[i] = src.R_base[i];
		platform[i] = src.platform[i];
		R_platform[i] = src.R_platform[i];
		r_pulley[i] = src.r_pulley[i];
	}
	fmin = src.fmin;
	fmax = src.fmax;
	lmin = src.lmin;
	lmax = src.lmax;
	
	platform_mass = src.platform_mass;
	platform_swingangle = src.platform_swingangle;
	platform_inertia = src.platform_inertia;
	platform_name = src.platform_name;
	platform_desc = src.platform_desc;
	centerofgravity = src.centerofgravity;
	robotKinematicsModel = src.robotKinematicsModel;
	elasticityModel = src.elasticityModel;
	controller_ipotakt = src.controller_ipotakt;
	return *this;
}

//! set the geometry of the frame and the platform for leg id to b and p, respectively.
void CRobotData::setLeg(const int& id, const Vector3d& b, const Vector3d& p)
{
	if (id < 0 || id >= now)
		return;
	platform[id] = p;
	base[id] = b;
}

//! get the geometry of the frame and the platform for leg id to b and p, respectively.
void CRobotData::getLeg(const int& id, Vector3d& b, Vector3d& p)
{
	if (id < 0 || id >= now)
		return;
	p = platform[id];
	b = base[id];
}

//! copy the geometry data of base and platform vector by vector into AB
//! \param AB [out] The matrix is resized to be 6 times now. The functions copies the 
//! geometry data of the robot into this matrix
void CRobotData::getLegMatrix(MatrixXd& AB)
{
	AB.resize(6, now);
	for (int i=0; i<now; i++)
	{
		AB.block(0,i,3,1) = base[i];
		AB.block(3,i,3,1) = platform[i];
	}
}

//! \return The matrix composed from the base (proximal) anchor points of the robot as columns
MatrixXd CRobotData::getBaseMatrix()
{
	MatrixXd A(3,now);
	for (int i=0; i<now; i++)
		A.block(0,i,3,1) = base[i];
	return A;
}

//! \return The matrix composed from the platform (distal) anchor points of the robot as columns
MatrixXd CRobotData::getPlatformMatrix()
{
	MatrixXd B(3,now);
	for (int i=0; i<now; i++)
		B.block(0,i,3,1) = base[i];
	return B;
}

//! set the geometry of the frame for leg id to b
void CRobotData::setBase(const int& id, const Vector3d& b)
{
	if (id<0 || id >= now)
		return;
	base[id] = b;
}

//! define the orientation of the base frame, especially the alignment
//! of the pulley.
void CRobotData::setBaseOrientation(const int& id, const Matrix3d& R)
{
	if (id<0 || id >= now)
		return;
	R_base[id] = R;
}

//! set the geometry of the mobile platform for leg id to p
void CRobotData::setPlatform(const int& id, const Vector3d& p)
{
	if (id<0 || id >= now)
		return;
	platform[id] = p;
}

//! calculate an axis-aligned bounding box for the frame/base
bool CRobotData::getBoundingBoxBase(Vector3d& bbmin, Vector3d &bbmax)
{
	if (now <= 0)
		return false;
	bbmin = bbmax = base[0];
	for (int i=1; i<now; i++)
	{
		bbmin.x() = min(bbmin.x(),base[i].x());
		bbmin.y() = min(bbmin.y(),base[i].y());
		bbmin.z() = min(bbmin.z(),base[i].z());
		bbmax.x() = max(bbmax.x(),base[i].x());
		bbmax.y() = max(bbmax.y(),base[i].y());
		bbmax.z() = max(bbmax.z(),base[i].z());
	}
	return true;
}

//! calculate a axis-aligned bounding box for the platform
bool CRobotData::getBoundingBoxPlatform(Vector3d& bbmin, Vector3d &bbmax)
{
	if (now <= 0)
		return false;
	bbmin = bbmax = platform[0];
	for (int i=1; i<now; i++)
	{
		bbmin.x() = min(bbmin.x(),platform[i].x());
		bbmin.y() = min(bbmin.y(),platform[i].y());
		bbmin.z() = min(bbmin.z(),platform[i].z());
		bbmax.x() = max(bbmax.x(),platform[i].x());
		bbmax.y() = max(bbmax.y(),platform[i].y());
		bbmax.z() = max(bbmax.z(),platform[i].z());
	}
	return true;
}

//! return the mid point of the base's bounding box
Vector3d CRobotData::getBoundingBoxCenterBase()
{
	Vector3d bbmin,bbmax;
	getBoundingBoxBase(bbmin,bbmax);
	return 0.5*(bbmin+bbmax);
}

//! return the mid point of the platforms's bounding box
Vector3d CRobotData::getBoundingBoxCenterPlatform()
{
	Vector3d bbmin,bbmax;
	getBoundingBoxPlatform(bbmin,bbmax);
	return 0.5*(bbmin+bbmax);
}

/*! return a reference to the geometry variable selected by id
 *  the values for ID are limited to the range [0 ... now*6-1].
 *  the geometry variables are ordered as follows
 *  [base_1.x(), base_1.y(), base_1.z(), platform_1.x(), ... , platform_now.z()]
 *  if ID is out of range, it is mapped to the nearest end of the interval!
 */
double& CRobotData::getByID(const int& id)
{
	if (id<0)
		return base[0].x();
	if (id>6*now)
		return platform[now-1].z();
	int leg = id / 6;
	int coord = id % 6;
	switch (coord)
	{
		case 0: return base[leg].x();
		case 1: return base[leg].y();
		case 2: return base[leg].z();
		case 3: return platform[leg].x();
		case 4: return platform[leg].y();
	default:
		case 5: return platform[leg].z();
	}
}

/*! write the basic parameter of the wire robot into a text file
 *  \remark The base text file format is deprecated. Further extensions
 *  will be implemented in the XML file format.
 *  \todo Implement a method to write the orientations into the file
 */
bool CRobotData::Save(const string& filename)
{
	ofstream file(filename.c_str());
	file << "// Parameter of Wire Robot\n\n";

	file<< "// General Parameters\n"
		<< "// dof - degree-of-freedom\n"
		<< "dof " << dof << endl
		<< "\n// now - number of wires\n"
		<< "now " << now << endl
		<< "\n// lmin - minimal length of wires\n"
		<< "lmin " << lmin << endl
		<< "\n// lmax - maximal length of wires\n"
		<< "lmax " << lmax << endl
		<< "\n// fmin - minimal force in wires\n"
		<< "fmin " << fmin << endl
		<< "\n// fmax - maximal force in wires\n"
		<< "fmax " << fmax << endl << endl;
	file << "\n// Geometry information\n"
		<< "// base|platform INDEX X Y Z\n\n";
	int i;
	for (i=0; i<now; i++)
		file << "base " << i+1 << " " 
			<< base[i].x() << " "
			<< base[i].y() << " "
			<< base[i].z() << endl;
	for (i=0; i<now; i++)
		file << "platform " << i+1 << " " 
			<< platform[i].x() << " "
			<< platform[i].y() << " "
			<< platform[i].z() << endl;

	file << endl << "// End of File\n";

	return true;
}

/*! Save the basic parameters of the robot into a text file formatted for parameter assignment in
 *  numeric tools such as matlab, python, or maple
 *  \param [in] filename of the file to be generated by this function.
 *  \param [in] formatselector  defines the specific text format to be used in the file (matlab, python, maple).
 *  \return true, if successful otherwise false.
 *
	bool CRobotData::savePlain(const string& filename, const CPoseList::formatSelector format)
	{
		// open the respective file

		// write the dof (n) into the file
	
		// write the number of wires (m) intothe file
	
		// write the matrix A = [a_1 ... a_m] to the file
	
		// write the matrix B = [b_1 ... b_m] into the file
		return true;
	}
*/

//! set the number of cables for the current robot
bool CRobotData::SetNow(const int now)
{
	this->now = now;
	return true;
}

/*! read the geometry file for a wire robot
 *  \remark This file format is depricated
 *  \todo Implement a method to read the orientations from the file
 */
bool CRobotData::Load(const string& filename)
{
//	printf("Parsing file %s...\n",filename.c_str());
	int i_dof = -1,
		i_now = -1,
		index;
	Vector3d
		*i_base = 0,
		*i_platform = 0;
	double i_fmin = -1,i_fmax = -1,i_lmin = -1,i_lmax = -1,x,y,z;
	try {
		ifstream file(filename.c_str());
		
		if (!file.good())
			throw 1;

		char buf[1024],token[1024];
		
		int line = 0;
		while (!file.eof())
		{
			// read a line
			file.getline(buf,1024);
			line++;

			// get the first token
			if (sscanf(buf,"%s",token) != 1)
				continue;

			// is a comment line?
			if (strncmp(token,"//",2) == 0)
				continue;

			// can we identify the token
			
			// "dof"
			if (strcmp(token,"dof") == 0)
			{
				if (sscanf(buf,"%s %i",token,&i_dof) != 2)
					printf("Line %i: Could not read parameter after dof\n",line);
				continue;
			}

			// "now"
			if (strcmp(token,"now") == 0)
			{
				if (i_now != -1)
				{
					printf("Line %i: Only one instance of parameter 'now' is allowed. New value is ignored.\n",line);
					continue;
				}
				if (sscanf(buf,"%s %i",token,&i_now) == 2)
				{
					if (i_now>0 && i_now < 1000)
					{
						i_base = new Vector3d[i_now];
						i_platform = new Vector3d[i_now];
					}
					else
					{
						printf("Line %i: Fatal error. Number of wires %i is invalid\n",line,i_now);
						throw;
					}
				}
				else
					printf("Line %i: Could not read parameter after now\n",line);
				continue;
			}

			// "fmin"
			if (strcmp(token,"fmin") == 0)
			{
				if (sscanf(buf,"%s %lf",token,&i_fmin) != 2)
					printf("Line %i: Could not read parameter after fmin\n",line);
				continue;
			}
		
			// "fmax"
			if (strcmp(token,"fmax") == 0)
			{
				if (sscanf(buf,"%s %lf",token,&i_fmax) != 2)
					printf("Line %i: Could not read parameter after fmax\n",line);
				continue;
			}

			// "lmin"
			if (strcmp(token,"lmin") == 0)
			{
				if (sscanf(buf,"%s %lf",token,&i_lmin) != 2)
					printf("Line %i: Could not read parameter after lmin\n",line);
				continue;
			}
		
			// "lmax"
			if (strcmp(token,"lmax") == 0)
			{
				if (sscanf(buf,"%s %lf",token,&i_lmax) != 2)
					printf("Line %i: Could not read parameter after lmax\n",line);
				continue;
			}

			// "base"
			if (strcmp(token,"base") == 0)
			{
				if (i_now<0)
				{
					printf("Line %i: Fatal error. Geometry data 'base' only allowed after 'now' parameter.\n",line);
					throw;
				}
				if (sscanf(buf,"%s %i %lf %lf %lf",token,&index,&x,&y,&z) == 5)
				{

					if (index>0 && index <= i_now)
					{
						i_base[index-1].x() = x;
						i_base[index-1].y() = y;
						i_base[index-1].z() = z;
					}
					else
					{
						printf("Line %i: Index %i for 'base' not valid. Ignoring this line.\n",line,index);
						continue;
					}
				}
				else
					printf("Line %i: Error in parameter format for base vertex. Syntax is 'base INDEX X Y Z'\n",line);
				continue;
			}

			// "platform"
			if (strcmp(token,"platform") == 0)
			{
				if (i_now<0)
				{
					printf("Line %i: Fatal error. Geometry data 'platform' only allowed after 'now' parameter.\n",line);
					throw;
				}
				if (sscanf(buf,"%s %i %lf %lf %lf",token,&index,&x,&y,&z) == 5)
				{
					if (index>0 && index <= i_now)
					{
						i_platform[index-1].x() = x;
						i_platform[index-1].y() = y;
						i_platform[index-1].z() = z;
					}
					else
					{ 
						printf("Line %i: Index %i for 'platform' not valid. Ignoring this line.\n",line,index);
						continue;
					}
				}
				else
					printf("Line %i: Error in parameter format for platform vertex. Syntax is 'platform INDEX X Y Z'\n",line);
				continue;
			}

			// token could no be identified
			printf("Line %i: '%s'. Unknown parameter ignored\n",line,buf);
		}

		// parsing completed; copy data from temp variables to class members
		
		// check if parameter are feasible
		if (i_now >= 0 && i_dof >= 0 && i_base && i_platform)
		{
			now = i_now;
			dof = i_dof;
			if (i_lmin > 0)
				lmin = i_lmin;
			if (i_lmax > 0)
				lmax = i_lmax;
			if (i_fmin > 0)
				fmin = i_fmin;
			if (i_fmax > 0)
				fmax = i_fmax;
			for (int i=0; i<now; i++)
			{
				platform[i] = i_platform[i];
				base[i] = i_base[i];
			}
		}
		else
		{
			printf("Fatal Error. Parameter file is not consistent!\n");
			throw;
		}

		// clean up memory if needed
		if (i_base) delete [] i_base;
		if (i_platform) delete [] i_platform;
		// parsing is now completed; if program was 

		return true;
	}

	// general error handler
	catch (...)
	{
		// clean up memory if needed
		if (i_base) delete [] i_base;
		if (i_platform) delete [] i_platform;
	
		return false;
	}
}

/*! read the robot from the XML file filename
 * this implementation extracts the following data from the xml file
 * - the number of wires
 * - the geometry of the frame
 * - the geometry of the platform
 * all other values in the XML file are ignored and set to the default values
 * of the class CRobotData.
 * \param filename [in] filename of the XML file to parse
 * \return true, if all parameters could be read, otherwise false;
 * \remark The current robot is unmodified if the function returns false
 * \todo Check if the current implementation with a pointer on the XML document
 * releases the memory reserved. It seems that it does not!
 * \remark This file format is deprecated and will be removed in future versions of Wirelib
 */
bool CRobotData::loadXml(const string& filename)
{
	CRobotData tRobot;

	TiXmlDocument* doc = new TiXmlDocument(filename.c_str());
	if (!doc->LoadFile())
		return false;

	// parse the XML document
	TiXmlElement *root = doc->RootElement();
	// check if root element is <models>
	if (strcmp(root->Value(),"models"))
	{
		cout << "ERROR: root node in XML file is not <models>\n";
		return false;
	}

	// search the model named robotname; if no robot was specified, just take the first one
	TiXmlNode *robot = root->FirstChild("wireRobot");
	if (!robot)
	{
		cout << "ERROR: there is no node <wireRobot>\n";
		return false;
	}

	// get the number of wires from the file
	TiXmlNode *sysParam = robot->FirstChild("systemParameter");
	if (!sysParam)
	{
		cout << "ERROR: could not find the tag <systemParameter>\n";
		return false;
	}
	int i_now;
	if (sysParam->ToElement()->QueryIntAttribute("units",&i_now) != TIXML_SUCCESS)
	{
	}
	
	// configure the temporary robot
	tRobot.setMotionPattern(i_now,MP3R3T);

	// search the platform tag
	TiXmlNode* platform = robot->FirstChild("platform");
	if (!platform)
	{
		cout << "ERROR: could not find the tag <platform>\n";
		return false;
	}

	// and read the geometry for the platform vectors
	TiXmlNode* anchorPoint;
	int i; 
	for (i=0, anchorPoint=platform->FirstChild("anchorPoint"); i<i_now; i++,anchorPoint=anchorPoint->NextSibling("anchorPoint"))
	{
		if (!anchorPoint)
		{
			cout << "ERROR: could not find " << i+1 << "the <anchorPoint> tag\n";
			return false;
		}
		if (anchorPoint->ToElement()->QueryDoubleAttribute("x",&tRobot.platform[i].x()) != TIXML_SUCCESS ||
			anchorPoint->ToElement()->QueryDoubleAttribute("y",&tRobot.platform[i].y()) != TIXML_SUCCESS ||
			anchorPoint->ToElement()->QueryDoubleAttribute("z",&tRobot.platform[i].z()) != TIXML_SUCCESS)
		{
			cout << "ERROR: could not read the coordinates x,y,z from <anchorPoint>\n";
			return false;
		}
	}

	// search for the base vectors and read their values
	// and read the geometry for the platform vectors
	TiXmlNode* unit;
	for (i=0, unit=robot->FirstChild("unit"); i<i_now; i++,unit=unit->NextSibling("unit"))
	{
		if (!unit)
		{
			cout << "ERROR: could not find " << i+1 << "th <unit> tag\n";
			return false;
		}
		TiXmlNode* winch = unit->FirstChild("winch");
		if (!winch)
		{
			cout << "ERROR: could not find <winch> tags in <unit>\n";
			return false;
		}
		TiXmlNode* tendonContactPoint = winch->FirstChild("tendonContactPoint");
		if (!tendonContactPoint)
		{
			cout << "ERROR: could not find <tendonContactPoint> tags in <winch>\n";
			return false;
		}
		if (tendonContactPoint->ToElement()->QueryDoubleAttribute("x",&tRobot.base[i].x()) != TIXML_SUCCESS ||
			tendonContactPoint->ToElement()->QueryDoubleAttribute("y",&tRobot.base[i].y()) != TIXML_SUCCESS ||
			tendonContactPoint->ToElement()->QueryDoubleAttribute("z",&tRobot.base[i].z()) != TIXML_SUCCESS)
		{
			cout << "ERROR: could not read the coordinates x,y,z from <tendonContactPoint>\n";
			return false;
		}
	}

	// if parsing was successful, copy the values stored in tempRobot to the this robot
	*this = tRobot;

	return true;
}

/*! the core xml write function for the node (and subelement) of the <robot> tag
 * 
 *  The structure of the XML file is based on the ATLAS specification document
 *  XML version 0.32 from 27.04.2016.
 */
bool CRobotData::setXmlNode(TiXmlElement& robot)
{
	robot.SetAttribute("generator",(string("Wirelib Model writer, (C)opyright 2009-2018 Andreas Pott. Build: ")+string(PCRL::name)+string(PCRL::versionString)).c_str());
	switch (motionPattern) 
	{
	case MP1T:   robot.SetAttribute("motionpattern","1T"); break;
	case MP2T:   robot.SetAttribute("motionpattern","2T"); break;
	case MP3T:   robot.SetAttribute("motionpattern","3T"); break;
	case MP1R2T: robot.SetAttribute("motionpattern","1R2T"); break;
	case MP2R3T: robot.SetAttribute("motionpattern","2R3T"); break;
	case MP3R3T: 
	default:     robot.SetAttribute("motionpattern","3R3T"); break;
	}

	// insert the geometry node
	TiXmlElement geometry("geometry");

	char buf[20];
	// add one <chain> node for each cable in our robot
	for (int i=0; i<now; i++)
	{
		// create the winch node
		TiXmlElement chain("chain");
		sprintf(buf,"%i",i+1);
		chain.SetAttribute("id",buf);
//		chain.SetAttribute("id",itoa(i+1,buf,10));
		chain.SetAttribute("winchtype","IPAnema-Winch");

		// create the platform and base nodes for each winch
		TiXmlElement xplatform("platform");
		setPositionVectorAttrib(&xplatform,platform[i]);
		setOrientationMatrixAttrib(&xplatform,R_platform[i]);

		TiXmlElement xbase("base");
		setPositionVectorAttrib(&xbase,base[i]);
		setOrientationMatrixAttrib(&xbase,R_base[i]);
		xbase.SetDoubleAttribute("radius",r_pulley[i]); // Attention: same pulley radius for all winches

		chain.InsertEndChild(xbase);
		chain.InsertEndChild(xplatform);
		
		geometry.InsertEndChild(chain);
	}
	robot.InsertEndChild(geometry);

	// insert dummy elements for the <platform>, <controller>, and <application> nodes
	TiXmlElement tplatform("platform");
		tplatform.SetAttribute("name",platform_name.c_str());
		tplatform.SetAttribute("description",platform_desc.c_str());
		tplatform.SetDoubleAttribute("mass", platform_mass);
		tplatform.SetDoubleAttribute("swing_angle", platform_swingangle);

		TiXmlElement inertia("inertiatensor");
        setInertiaTensorAttrib(&inertia, platform_inertia);
			//inertia.SetAttribute("Ixx", 14); // Achtung: hier muss noch die richtige Funktion rein für platform_inertia
		tplatform.InsertEndChild(inertia);

		TiXmlElement tcenterofgravity("centerofgravity");
			setPositionVectorAttrib(&tcenterofgravity,centerofgravity);
		tplatform.InsertEndChild(tcenterofgravity);
	robot.InsertEndChild(tplatform);


	TiXmlElement controller("controller");
		controller.SetDoubleAttribute("ipo_clock",controller_ipotakt);
	robot.InsertEndChild(controller);

/*
	TiXmlElement application("application");
		application.SetAttribute("name", application_name.c_str());
		application.SetAttribute("id", application_id);
		application.SetAttribute("description", application_desc.c_str());
		application.SetDoubleAttribute("load_mass",application_load_mass);
		application.SetDoubleAttribute("load_moment",application_load_moment);
		application.SetDoubleAttribute("min_cable_load",application_min_cableload);
		application.SetDoubleAttribute("max_cable_load",application_max_cableload);
	robot.InsertEndChild(application);*/
	return true;
}

/*! the core xml getter function for the <robot> tag (and its subelement) 
 * 
 *  The structure of the XML file is based on the ATLAS specification document
 *  XML version 0.32 from 27.04.2016.
 */
bool CRobotData::getXmlNode(TiXmlElement& robot)
{
	TiXmlNode *geometry = robot.FirstChildElement("geometry");
	if (!geometry)
	{
		cout << "ERROR: could not find <geometry> tag\n";
		return false;
	}

	// get the motion pattern from the robot attribute
	const char* pmp = robot.ToElement()->Attribute("motionpattern");
	if (!pmp)
		motionPattern = MP3R3T;
	else if (strcmp("1T",pmp) == 0)
		motionPattern = MP1T;
	else if (strcmp("2T",pmp) == 0)
		motionPattern = MP2T;
	else if (strcmp("3T",pmp) == 0)
		motionPattern = MP3T;
	else if (strcmp("1R2T",pmp) == 0)
		motionPattern = MP1R2T;
	else if (strcmp("2R3T",pmp) == 0)
		motionPattern = MP2R3T;
	else if (strcmp("3R3T",pmp) == 0)
		motionPattern = MP3R3T;
	else 
		motionPattern = MP3R3T;

	// now that we have the geometry tag, we count the number of <chain> to get the
	// number of wires "now".
	now = 0;
	for (TiXmlNode* pChain = geometry->FirstChild("chain"); pChain!=0; pChain = pChain->NextSibling("chain"))
		now++;
	
	// no chain tags found; we cannot processed
	if (now == 0)
	{
		cout << "ERROR: could not find <chain> tags\n";
		return false;
	}
	
	setMotionPattern(now,motionPattern);

	// now we loop through the <chain> tags to read the geometry data for platform and base
	int i = 0;
	for (TiXmlNode* pChain = geometry->FirstChild("chain"); pChain!=0; pChain = pChain->NextSibling("chain"))
	{
		// read winch type
		//pChain->ToElement()->
		// read position and orientation of the base
		TiXmlNode* pBase = pChain->FirstChild("base");
		if (!pBase)
		{
			cout << "WARNUNG: could not find <base> tag; assuming zero vector\n";
			base[i] = Vector3d(0,0,0);
			R_base[i] = Matrix3d::ZRotationMatrix3d(0);
		}
		else
		{
			// read the position
			if (!getPositionVectorAttrib(pBase->ToElement(),base[i]))
			{
				cout << "WARNUNG: could not extract position from <base> tag; assuming zero vector\n";
				base[i] = Vector3d(0,0,0);
			}
			// read the orientation
			if (!getOrientationMatrixAttrib(pBase->ToElement(),R_base[i]))
			{
				cout << "WARNUNG: could not extract orientation from <base> tag; assuming identity matrix\n";
				R_base[i] = Matrix3d::ZRotationMatrix3d(0);
			}
			double radius = 0.1;
			if (pBase->ToElement()->QueryDoubleAttribute("radius",&radius) == TIXML_SUCCESS)
			{
				r_pulley[i] = radius;
			}
			else 
				cout << "WARNING: could not extract pulley radius from <base> tag; assuming default radius 0.1\n";
		}
		// read position and orientation of the platform
		TiXmlNode* pPlatform = pChain->FirstChild("platform");
		if (!pPlatform)
		{
			cout << "WARNUNG: could not find <platform> tag; assuming zero vector\n";
			platform[i] = Vector3d(0,0,0);
			R_platform[i] = Matrix3d::ZRotationMatrix3d(0);
		}
		else
		{
			// read the position
			if (!getPositionVectorAttrib(pPlatform->ToElement(),platform[i]))
			{
				cout << "WARNUNG: could not extract position from <platform> tag; assuming zero vector\n";
				platform[i] = Vector3d(0,0,0);
			}
			// read the orientation
			if (!getOrientationMatrixAttrib(pPlatform->ToElement(),R_platform[i]))
			{
				cout << "WARNUNG: could not extract orientation from <platform> tag; assuming identity matrix\n";
				R_platform[i] = Matrix3d::ZRotationMatrix3d(0);
			}
		}
		i++;
	}
	// we are still not yet finished with the implementation
	// thus, we should return FALSE here for for testing we don't
	return true;
}

/*! save robot parameter into an XML file; this is a new implementation
 *  The function saves data into a new file. Modifications of existing files
 *  are not yet supported. 
 * 
 *  The structure of the XML file is based on the ATLAS specification document
 *  XML version 0.32 from 27.04.2016.
 *  
 *  \todo change the name of the XML generator to saveXML as soon as it is feature-completed
 *  \remark 
 */
bool CRobotData::saveXmlAtlas(const string& filename)
{
	const char* version = "0.32";

	TiXmlDocument doc; // = new TiXmlDocument();
	// create the models nodes (for support of multiple models in one file)
	TiXmlElement root("models");
	root.SetAttribute("version",version);

	// create the main robot model
	TiXmlElement robot("robot");
	setXmlNode(robot);
	root.InsertEndChild(robot);

	// finally insert the root node into the document
	doc.InsertEndChild(root);

	// write the document in the file
	doc.SaveFile(filename.c_str());
	return true;
}

/*! set the base anchor points (base[i] to a cubic geometry. The cube is 
 *  centered in the xy plain around the origin. In z-direction its base
 *  is on z=0 and the cube is placed in z+ direction.
 *  \param length [in] dimension in x-direction of the cube
 *  \param width [in] dimension in y-direction of the cube
 *  \param heigt [in] dimension in z-direction of the cube
 *  \return true, if the geometry was set successfully 
 */
bool CRobotData::setFrameGeometryCube(const double length, const double width, const double height)
{
	if (now != 8)
		return false;
	base[0] = Vector3d(-length/2,  width/2, height);
	base[1] = Vector3d( length/2,  width/2, height);
	base[2] = Vector3d( length/2, -width/2, height);
	base[3] = Vector3d(-length/2, -width/2, height);
	base[4] = Vector3d(-length/2,  width/2, 0);
	base[5] = Vector3d( length/2,  width/2, 0);
	base[6] = Vector3d( length/2, -width/2, 0);
	base[7] = Vector3d(-length/2, -width/2, 0);
	return true;
}

/*! set the base anchor points (base[i] to a cubic geometry. The cube is 
 *  centered in the xy plain around the origin. In z-direction its base
 *  is on z=0 and the cube is placed in z+ direction.
 *  \param length [in] dimension in x-direction of the cube
 *  \param width [in] dimension in y-direction of the cube
 *  \param heigt [in] dimension in z-direction of the cube
 *  \return true, if the geometry was set successfully 
 */
bool CRobotData::setPlatformGeometryCube(const double length, const double width, const double height)
{
	if (now != 8)
		return false;
	platform[0] = Vector3d(-length/2,  width/2, height);
	platform[1] = Vector3d( length/2,  width/2, height);
	platform[2] = Vector3d( length/2, -width/2, height);
	platform[3] = Vector3d(-length/2, -width/2, height);
	platform[4] = Vector3d(-length/2,  width/2, 0);
	platform[5] = Vector3d( length/2,  width/2, 0);
	platform[6] = Vector3d( length/2, -width/2, 0);
	platform[7] = Vector3d(-length/2, -width/2, 0);
	return true;
}

//! tranlate all frame geometry vectors by an offset
bool CRobotData::translateFrameGeometry(const Vector3d& dr)
{
	for (int i=0; i<now; i++)
		base[i]+=dr;
	return true;
}

//! rotate all frame geometry vectors by a rotation matrix 
bool CRobotData::rotateFrameGeoemetry(const Matrix3d& dR)
{
	for (int i=0; i<now; i++)
		base[i] = dR*base[i];
	return true;
}

//! scale all frame geometry vectors by values in the vector s
bool CRobotData::scaleFrameGeometry(const Vector3d& s)
{
	for (int i=0; i<now; i++)
	{
		base[i].x()*=s.x();
		base[i].y()*=s.y();
		base[i].z()*=s.z();
	}
	return true;
}

//! scale the frame such that its bounding box equals the given volumen
bool CRobotData::normalizeFrameGeometry(const double& volume)
{
	// compute the scaling factor
	Vector3d minBB,maxBB;
	getBoundingBoxBase(minBB,maxBB);
	Vector3d ext = maxBB-minBB;
	double curVol = ext.x()*ext.y()*ext.z();
	Vector3d scale(1,1,1);
	if (curVol <= 0)
		return false;
	scale/=pow(curVol,1.0/3.0);
	// scale the frame
	return scaleFrameGeometry(scale);
}

//! tranlate all frame geometry vectors by an offset
bool CRobotData::translatePlatformGeometry(const Vector3d& dr)
{
	for (int i=0; i<now; i++)
		platform[i]+=dr;
	return true;
}

//! rotate all frame geometry vectors by a rotation matrix 
bool CRobotData::rotatePlatformGeoemetry(const Matrix3d& dR)
{
	for (int i=0; i<now; i++)
		platform[i] = dR*platform[i];
	return true;
}

//! scale all frame geometry vectors by values in the vector s
bool CRobotData::scalePlatformGeometry(const Vector3d& s)
{
	for (int i=0; i<now; i++)
	{
		platform[i].x()*=s.x();
		platform[i].y()*=s.y();
		platform[i].z()*=s.z();
	}
	return true;
}

} // end namespace PCRL
