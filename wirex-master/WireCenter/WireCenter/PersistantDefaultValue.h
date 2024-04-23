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

/*! \file		PersistantDefaultValue.h
 *
 *  \author		Andreas Pott
 *
 *  \date		06.04.2015
 * 
 *  \class CPersistantDefaultValue
 *  
 *  \brief Store standard values in a persistent and inspectable
 *  way with minimum overhead for the application development. 
 * 
 *  \par Desciption
 *  The objective of the the class is to create a persistant storage for 
 *  default values used in wirecenter, especially the parameters passed as 
 *  standard arguments to calls in WireLib. Also the default filesnames for 
 *  reading and writing files can be stored in the persistant storage.
 *  
 *  The behaviour of the class shall be as follows. The class is implemented 
 *  as singleton to provide one single access point for default parameters 
 *  throughout the whole application. Using the default parameter engine is 
 *  achieved through a macro collection that basically just wrap the constant 
 *  parameter of type int bool double and string to br passed to a certain 
 *  function call. Let the name of the macro be PERSiST, than a typical usage 
 *  shall be
 *  
 *  Smaple_function(PERSIST(100));
 *  
 *  Or
 *  
 *  Smaple_function(PERSIST(100),"symolic-name","xpath-name");
 *  
 *  When firstly called, the macro actually returns the value, where repeated 
 *  calls either within the same session or between different sessions of the 
 *  main program whill memorize the last value used for  the variable. 
 *  
 *  The underlying implementetion is a singleton class that implement a map 
 *  between strings and the reflection variant type. If the additional string 
 *  constants are passed to the macro, then the persistent storage creates an 
 *  entry with the passed values. The persistant storage looks up the name, 
 *  if it does not exist it is created and the passed default value is added 
 *  to the dictionary. If the name already existe, the value is look up rather 
 *  than generated. 
 *  
 *  Clearly, just storing the key value pairs does nothing new with the 
 *	application. However, an extended part of the API of the storage class 
 *  can be used to manipulate the data. The foolow functions are foreseen to 
 *  be supported :
 *  	- Store the key value pairs in an xml document and restore the values 
 *		  from an xml. Support for file or inmemory  string
 *  	- Store or restore the key value pairs from an sqlite database.
 *  	- Store and restore the key Values from the registry
 *  	- Manipulate the content 
 * 
 *  \remark There is an issue with reading back a stored database before the
 *  respective values have been used in the program. Since the global store
 *  is not yet allocated through using the main program (e.g. WireCenter), no
 *  xpath information is available of what needs to be extracted from an xml
 *  file, a database, or the ini file. For the xml use case it is proposed
 *  to add an additional meta information section to the xml file that stores
 *  the self description of the persistent stores. The data could be formatted
 *  as a series of xml tags of the form
 * 
 *  <meta type="i" value="10" name="symbolic-name" xpath="symbolic-xpath" />
 *
 *  Looping through all meta-tag of the above structure, one could setup the 
 *  required content for CReflection::valueMap. 
 *  
 *  \dependency
 *		tinyXML		for handling xml files
 *  
 *////////////////////////////////////////////////////////////////////////////

#pragma once

#include <WireLib/Reflection.h>

//! the following macro(s) provide a convenient interface to use the data
//! stores provided by CPersistantDefaultValue
#define PERSIST(value,name,xpath) CPersistantDefaultValue::persist((value),(name),(xpath))

/*! \class CPersistantDefaultValue
 */
class CPersistantDefaultValue : public PCRL::CReflection
{
private:
	CPersistantDefaultValue(void);
	~CPersistantDefaultValue(void);

	bool readSchemeCore(TiXmlElement* root);
public:
	//! return the one and only instance of the class
	static CPersistantDefaultValue& getInstance();

	//! read the data scheme from an xml file and create the respective data structre
	bool readScheme(const std::string& filename);

	//! make the numerical value passed to the function persistant
	static int persist(const int& value, std::string name="", std::string xpath="");
	static bool persist(const bool& value, std::string name="", std::string xpath="");
	static double persist(const double& value, std::string name="", std::string xpath="");
	static std::string persist(const std::string& value, std::string name="", std::string xpath="");
};
