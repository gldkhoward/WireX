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

/*!*******************************************************************
 *  \file   : PersistantDefaultValue.cpp
 *
 *  Project   WireCenter
 *
 *  \Author   Andreas Pott
 *
 *  \Date     06.04.2015
 *
 *  Last Change: 13.04.2015
 *
 *********************************************************************
 */ 

#include "StdAfx.h"
#include "PersistantDefaultValue.h"


CPersistantDefaultValue::CPersistantDefaultValue(void)
{
	// in contrast to the default behaviour of CReflection we want to 
	// meta data to be written and read from xml files
	writeSchemeData=true;
}


CPersistantDefaultValue::~CPersistantDefaultValue(void)
{
	// in contrast to the base class CReflection, this class reserved memory
	// for the information stored in the reflection structre. Therefore, we
	// have to free the memory when destructing.
	for (auto itor = getInstance().getValueMap().begin(); itor!=getInstance().getValueMap().end(); ++itor)
	{
		switch (itor->second.type)
		{
		case PCRL::TRVBool: delete itor->second.value.pBool; break;
		case PCRL::TRVInt: delete itor->second.value.pInt; break;
		case PCRL::TRVDouble: delete itor->second.value.pDouble; break;
		case PCRL::TRVString: delete itor->second.value.pString; break;
		}
	}
}

//! the main function to extract and build the scheme data in the memory
//! the functions iterates through the provided root tags and inspects all
//! tags named <scheme>. A Meta Tag is expected to contain three attributes
//! storing the type, name, and xpath of the desired entity.
//!
bool CPersistantDefaultValue::readSchemeCore(TiXmlElement* root)
{
	bool res=true;
	// iterate through all scheme tags
	TiXmlNode* meta = root->FirstChild("scheme");
	while (meta)
	{
		// we have an meta entry; we try to extract to data of interest, i.e.
		// type, name, and xpath
		const char *type = meta->ToElement()->Attribute("type");
		const char *name = meta->ToElement()->Attribute("name");
		const char* xpath = meta->ToElement()->Attribute("xpath");
		// we received a valid entry; at least all three elements were defined
		if (type && name && xpath)
		{
			if (type[0]=='i')
				persist(0,name,xpath);
			else if (type[0]=='d')
				persist(0.0,name,xpath);
			else if (type[0]=='s')
				persist("",name,xpath);
			else if (type[0]=='b')
				persist(false,name,xpath);
		}
		else
			res = false;
		meta = meta->NextSiblingElement("scheme");
//		meta = root->NextSiblingElement("scheme");
	} 
	return res;
}

bool CPersistantDefaultValue::readScheme(const std::string& filename)
{
	TiXmlDocument doc(filename.c_str());
	if (!doc.LoadFile())
		return false;
	TiXmlElement* root = doc.RootElement();
	// read the data for all data maps
	return readSchemeCore(root);
}

/*! return the one and only instance of the class
 *  this static function is part of the singleton implementation. If you need 
 *  thread safe access to the default value storage, you have to protect this 
 *  call e.g. by critial sections. Anyhow, thread safe functions are difficult 
 *  to implement in a platform neutral manner.
 */
CPersistantDefaultValue& CPersistantDefaultValue::getInstance()
{
	static CPersistantDefaultValue PDV;
	return PDV;
}

/*! use this function(s) do either register a new value for persistancy or to 
 *  look up and existing value. If name exists in the storage of 
 *  CPersistantDefaultValue, the number passed through the value parameter
 *  is discarded. The typical usage to manage default values makes only one
 *  call to the persist function for each default value. there is no technical
 *  limitation to using the same name at different positions in the program.
 *  However, the behaviour of the persist function may be irritating since
 *  the developer must keep track of the names and default values.
 *
 *  There four flavours of this function for the four prefered datatypes
 *  int, bool, dobule, and string, respectively.
 *
 *  \param value [in] the default value used in the orignial code that shall be
 *         added to the persistant storage.
 *  \param name [in] the symbolic name that will be associated with the default
 *         value. The symbolic name must be usique and will be used as unique key
 *         in memory storage, in an ini file, in the xml file, and if as key in the
 *         data base.
 *  \param xpath [in] a string according to the xml-xpath syntax defining where
 *         to store the value when writing to an xml file.
 *  \return the given value or the value look up from the persistant storage
 */
int CPersistantDefaultValue::persist(const int& value, std::string name, std::string xpath)
{
	// look up the name
	auto itor = getInstance().getValueMap().find(name);
	if (itor==getInstance().getValueMap().end())
	{	// does not exist jet
		getInstance().bind(*new int (value),name,xpath);
		return value;
	}
	else
		return *(itor->second.value.pInt);
}

bool CPersistantDefaultValue::persist(const bool& value, std::string name, std::string xpath)
{
	// look up the name
	auto itor = getInstance().getValueMap().find(name);
	if (itor==getInstance().getValueMap().end())
	{	// does not exist jet
		getInstance().bind(*new bool (value),name,xpath);
		return value;
	}
	else
		return *(itor->second.value.pBool);
}

double CPersistantDefaultValue::persist(const double& value, std::string name, std::string xpath)
{
	// look up the name
	auto itor = getInstance().getValueMap().find(name);
	if (itor==getInstance().getValueMap().end())
	{	// does not exist jet
		getInstance().bind(*new double (value),name,xpath);
		return value;
	}
	else
		return *(itor->second.value.pDouble);
}

std::string CPersistantDefaultValue::persist(const std::string& value, std::string name, std::string xpath)
{
	// look up the name
	auto itor = getInstance().getValueMap().find(name);
	if (itor==getInstance().getValueMap().end())
	{	// does not exist jet
		getInstance().bind(*new std::string(value),name,xpath);
		return value;
	}
	else
		return *(itor->second.value.pString);
}
