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
 *  \file   : ApplicationRequirement.cpp
 *
 *  Project   Wirelib
 *
 *  \Author   Andreas Pott
 *
 *  \Date     17.03.2011
 *
 *********************************************************************
 */ 

#include "ApplicationRequirement.h"
#include <tinyXML/tinyxml.h>

namespace PCRL {

//! put some default values here.
CApplicationRequirement::CApplicationRequirement(void)
{
	minWS = Vector3d(-1,-1, 0);
	maxWS = Vector3d( 1, 1, 1);
	minIS.setZero();	
	maxIS.setZero();	

	payload = 0;	
	velocity = 1;	
	acceleration = 1;
	force = 0;		
	torque = 0;	
}

CApplicationRequirement::~CApplicationRequirement(void)
{
}

/*! load the requirement profil from an XML file with filename
 *  \param filename [in] filename of the XML file to parse
 *  \return true, if all parameters could be loaded, otherwise false.
 *  If the function returen false nothing is altered in the current object.
 */
bool CApplicationRequirement::loadXML(const string& filename)
{
	CApplicationRequirement T;
	
	TiXmlDocument doc;
	if (filename.length() > 0 && filename.at(0) == '<')
	{
		if (!doc.Parse(filename.c_str()))
			return false;
	}
	else
		if (!doc.LoadFile(filename.c_str()))
			return false;

	// parse the XML document
	TiXmlElement *root = doc.RootElement();
	// check if root element is <models>
	if (strcmp(root->Value(),"models"))
	{
		cout << "ERROR: root node in XML file is not <models>\n";
		return false;
	}

	// search the model named robotname; if no robot was specified, just take the first one
	TiXmlNode *req = root->FirstChild("requirement");
	if (!req)
	{
		cout << "ERROR: there is no node <requirement>\n";
		return false;
	}

	// load the workspace requirement into minWS, maxWS
	TiXmlNode *workspace = req->FirstChild("workspace");
	if (!workspace)
		cout << "WARNING: there is no node <workspace>\n";
	else
	{
		if (workspace->ToElement()->QueryDoubleAttribute("xmin",&T.minWS[0]) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute xmin in <workspace>; assuming default value\n";
		if (workspace->ToElement()->QueryDoubleAttribute("ymin",&T.minWS[1]) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute ymin in <workspace>; assuming default value\n";
		if (workspace->ToElement()->QueryDoubleAttribute("zmin",&T.minWS[2]) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute zmin in <workspace>; assuming default value\n";
		if (workspace->ToElement()->QueryDoubleAttribute("xmax",&T.maxWS[0]) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute xmax in <workspace>; assuming default value\n";
		if (workspace->ToElement()->QueryDoubleAttribute("ymax",&T.maxWS[1]) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute ymax in <workspace>; assuming default value\n";
		if (workspace->ToElement()->QueryDoubleAttribute("zmax",&T.maxWS[2]) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute zmax in <workspace>; assuming default value\n";
	}

	// load the workspace requirement into minMF, maxMF
	TiXmlNode *instspace = req->FirstChild("installationspace");
	if (!instspace)
		cout << "WARNING: there is no node <installationspace>\n";
	else
	{
		if (instspace->ToElement()->QueryDoubleAttribute("xmin",&T.minIS[0]) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute xmin in <installationspace>; assuming default value\n";
		if (instspace->ToElement()->QueryDoubleAttribute("ymin",&T.minIS[1]) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute ymin in <installationspace>; assuming default value\n";
		if (instspace->ToElement()->QueryDoubleAttribute("zmin",&T.minIS[2]) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute zmin in <installationspace>; assuming default value\n";
		if (instspace->ToElement()->QueryDoubleAttribute("xmax",&T.maxIS[0]) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute xmax in <installationspace>; assuming default value\n";
		if (instspace->ToElement()->QueryDoubleAttribute("ymax",&T.maxIS[1]) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute ymax in <installationspace>; assuming default value\n";
		if (instspace->ToElement()->QueryDoubleAttribute("zmax",&T.maxIS[2]) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute zmax in <installationspace>; assuming default value\n";
	}

	TiXmlNode *dyn = req->FirstChild("dynamics");
	if (!dyn)
		cout << "WARNING: there is no node <dynamics>\n";
	else
	{
		if (dyn->ToElement()->QueryDoubleAttribute("velocity",&T.velocity) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute 'velocity' in <dynamics>; assuming default value\n";
		if (dyn->ToElement()->QueryDoubleAttribute("acceleration",&T.acceleration) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute 'acceleration' in <dynamics>; assuming default value\n";
	}

	TiXmlNode *Static = req->FirstChild("static");
	if (!Static)
		cout << "WARNING: there is no node <static>\n";
	else
	{
		if (Static->ToElement()->QueryDoubleAttribute("force",&T.force) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute 'force' in <static>; assuming default value\n";
		if (Static->ToElement()->QueryDoubleAttribute("torque",&T.torque) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute 'torque' in <static>; assuming default value\n";
		if (Static->ToElement()->QueryDoubleAttribute("payload",&T.payload) != TIXML_SUCCESS)
			cout << "WARNING: there is no attribute 'payload' in <static>; assuming default value\n";
	}

	// loading was successful; copy the loaded data into the current instance
	*this = T;
	return true;
}

/*! save the requirement profil into an XML file with filename
 *  \param filename [in] filename of the XML file to parse
 *  \return true, if all parameters could be saved, otherwise false;
 */
bool CApplicationRequirement::saveXML(const string& filename)
{
	TiXmlDocument doc; 
	// create the models nodes (for support of multiple models in one file)
	TiXmlElement root("models");
	// create the main requirement node
	TiXmlElement req("requirement");
	
	req.SetAttribute("author","generated by WireLib; (C)opyright 2009-2018 Andreas Pott");

	// just write one data entry after the other into the file

	// create the workspace and installation space nodes in the file
	TiXmlElement workspace("workspace");
	workspace.SetDoubleAttribute("xmin", minWS.x());
	workspace.SetDoubleAttribute("ymin", minWS.y());
	workspace.SetDoubleAttribute("zmin", minWS.z());
	workspace.SetDoubleAttribute("xmax", maxWS.x());
	workspace.SetDoubleAttribute("ymax", maxWS.y());
	workspace.SetDoubleAttribute("zmax", maxWS.z());
	req.InsertEndChild(workspace);

	TiXmlElement instspace("installationspace");
	instspace.SetDoubleAttribute("xmin", minIS.x());
	instspace.SetDoubleAttribute("ymin", minIS.y());
	instspace.SetDoubleAttribute("zmin", minIS.z());
	instspace.SetDoubleAttribute("xmax", maxIS.x());
	instspace.SetDoubleAttribute("ymax", maxIS.y());
	instspace.SetDoubleAttribute("zmax", maxIS.z());
	req.InsertEndChild(instspace);

	// set the dynamic requirements
	TiXmlElement dyn("dynamics");
	dyn.SetDoubleAttribute("velocity",velocity);
	dyn.SetDoubleAttribute("acceleration",acceleration);
	req.InsertEndChild(dyn);

	// set the payload and process parameters
	TiXmlElement Static("static");
	Static.SetDoubleAttribute("force",force);
	Static.SetDoubleAttribute("torque",torque);
	Static.SetDoubleAttribute("payload",payload);
	req.InsertEndChild(Static);

	root.InsertEndChild(req);
	doc.InsertEndChild(root);
	
	doc.SaveFile(filename.c_str());
	return true;
}

} // end namespace PCRL