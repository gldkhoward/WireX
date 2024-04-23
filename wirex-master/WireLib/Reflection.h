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

/*! \file		Reflection.h
 * 
 *  \author		Andreas Pott
 *	
 *  \version	1.2
 *  \date		06.04.2005
 *  \change		19.07.2012
 * 
 *  \brief Provide a simple reflection mechanism to allow for
 *  Parameter persistance for derived objects and inspection based on symbolic
 *  names
 * 
 *  \par Desciption
 *  CReflection provides access to class member through symbolic names.
 *  The implementation allows for parameter persistance for derived classes. 
 *  Derive your class from CReflection or embed a member variable of CReflection
 *  to enable the class to read and write a configuration file. Use the function
 *  bind e.g. in the constructur of the class to generate a generic binding of the 
 *  passed variable. 
 *  Calls to WriteFile(...) write an appropriate parameter file to disk
 *  while cass to ReadFile(...) restore the parameters.
 *  If you do not like your variable names as names in the configuration file
 *  you can use the different bind(...) function to define another name
 *  for the variable the parameter file.
 *  Additionally, tinyXML is used to store the variables in an XML file. Using the
 *  extended binding syntax, the position of the varaible in the XML file can be
 *  specified by a subset of xpath syntax.
 *
 *  \remark
 *  The implementation is based on a earlier version used in the MoSynthesizer
 *  package. This extended version was motivated by the usage of Python  as scripting
 *  language and xml as widely used file format. Thus, the new version is designed 
 *  to support getter/setter function with symbolic names.
 *
 *  \dependency
 *		tinyXML		for handling xml files
 *  
 *  \todo Implement an error handling for file IO.
 *////////////////////////////////////////////////////////////////////////////

#pragma once

#include <map>
#include <list>
#include <string>
#include <iostream>
#include <fstream>
#include <tinyXML/tinyxml.h>

namespace PCRL {

using namespace std;

// #define REFLECT(x) bind((x),#x)
// #define REFLECTXML(x,y) bind((x),#x,(y))

//! union data type to store typed pointers inplace
//! note that every entry creates a pointer (with has the same size)
union UReflectionVariant
{
	void* pVoid;
	bool* pBool;
	int* pInt;
	double* pDouble;
	string* pString;
};

//! the eumeration contains the symbols for the entries of the UReflectionVariant
typedef enum { TRVvoid=0, TRVBool, TRVInt, TRVDouble, TRVString } eReflectionType;

//! data descriptor including the pointer union, an enumerator data type specifing
//! what is stored in the union and a string with the xpath name of the location in
//! an XML file.
struct TReflectionVariant
{
	// The following dummy function are used to restrict the template functions 
	// to the types that are allowed for templates parameters. This is a trick
	// to hinder the user from calling e.g. CReflection::bind() for parameters 
	// that are not supported by the variamt data type. If the type differs than
	// to match will be found for the "SupportedTypes" function and therefore
	// the compiler will present an error to the user.
	// posibly the source of the error redicts you to this position in the source 
	// code. Limitations on the reflected types are by design since reflection only
	// works on data types having appropriate get/set as well as read/write function.
/*	void supportedTypes(bool& x) const {}
	void supportedTypes(int& x) const {}
	void supportedTypes(double& x) const {}
	void supportedTypes(string& x) const {}*/
	// the data model of the reflection variant class
	//! enum type of the type
	eReflectionType type;
	//! union type of the actually stored pointer
	UReflectionVariant value;
	//! string containing the xpath adress of the assosiated data in the xml file
	string xpath;

	//! the default constructur creates an undefined variant
	TReflectionVariant() : type(TRVvoid) { value.pVoid=0; }
	//! the typed constructures are able to generated the correctly types variants
	explicit TReflectionVariant(bool& x, const string& XPath="") 
		: type(TRVBool), xpath(XPath) { value.pBool=&x; }
	explicit TReflectionVariant(int& x, const string& XPath="") 
		: type(TRVInt), xpath(XPath) { value.pInt=&x; }
	explicit TReflectionVariant(double& x, const string& XPath="") 
		: type(TRVDouble), xpath(XPath) { value.pDouble=&x; }
	explicit TReflectionVariant(string& x, const string& XPath="") 
		: type(TRVString), xpath(XPath) { value.pString=&x; }
	//! generic getter/settering function with type-safe testing
	void set(const bool& x)	  { assert(type==TRVBool);		*value.pBool=x; }
	void set(const int& x)	  { assert(type==TRVInt);		*value.pInt=x; }
	void set(const double& x) { assert(type==TRVDouble);	*value.pDouble=x; }
	void set(const string& x) { assert(type==TRVString);	*value.pString=x; }
	void get(bool& x) const	  { assert(type==TRVBool);		x=*value.pBool; }
	void get(int& x) const	  { assert(type==TRVInt);		x=*value.pInt; }
	void get(double& x) const { assert(type==TRVDouble);	x=*value.pDouble; }
	void get(string& x) const { assert(type==TRVString);	x=*value.pString; }
};

/*! \class CReflection
 *  The implementation of reflection that loads and saves the parameters
 *  to an XML node, file, or string.
 *
 *  \remark This class implements a very small subset of the xpath
 *  syntax. We only use some definitions and rules to uniquely specifiy tags
 *  and attributes with an xml file or more precisely with in the document object 
 *  model (DOM). All selections have to be explicit and unique; we do neither 
 *  deal with multi-selection nor do we search for nodes in the xml tree with wildcards.
 *
 *  \todo A couple of template functions are implemented as global functions in the cpp
 *  file although it would be better to embed them into this class as private members.
 *  This results from issues with implementing template member functions in the cpp file 
 *  (which led to unresolved symbols). Moving these functions to this class is highly 
 *  desireable.
 *
 *  \todo Add the generic getter and setter functions to access data by symbolic names.
 */
class CReflection  
{
    static char comment[];		//!< seperating character to indicate comments in the ini file
	static map<string, const char**> enumValues;	//!< contains the symbolic name constants for the enums under reflection
private:	
	typedef map<string, TReflectionVariant> VariantMap;
	//! the new general map based on a variant data type
	VariantMap valueMap;
	//! internal function to perform the xml read function 
	bool readXmlCore(TiXmlElement* root);
	//! internal function to perform the xml write function 
	bool writeXmlCore(TiXmlElement* root);
	//! internal function to perform the xml read function 
	bool readXmlSchemeCore(TiXmlElement* root);
	//! internal function to perform the xml write function 
	bool writeXmlSchemeCore(TiXmlElement* root);
protected:
	//! flag indicating if a full meta description of the data is written to xml files; defaults to false
	bool writeSchemeData;
public:
	// default constructor
	CReflection() : writeSchemeData(false) {}
	//! basic parameter binding based on template function
	template<typename T> void bind(T& x, const string& name, const string& xpath="")   
	{ 
		TReflectionVariant V(x,xpath); // If you get an error for this line of code, you try to bind a data type that is not supported by the implementation of CReflection. This is not a bug but a limitation of the implementation. 
		valueMap[name]=V; 
	} 
	//! write the binding table to the desired output stream. 
	bool writeStream(ostream& file) const;
	//! print the content and the state of the binding table to the console
	bool printBindingTable() const { writeStream(cout); return true; }
	//! write the reflectable parameters into a file
	bool writeFile(const string& filename) const;
	//! store values in xml file
	bool writeXmlFile(const string& filename);
	//! store the configuration data in a string
	bool writeXmlString(string& str);

	//! read the data from table stream
	bool readStream(istream& file);
	//! read the reflectable parameters from a file
    bool readFile(const string& filename);			
	//! restored vales from xml file
	bool readXmlFile(const string& filename);
	//! flash the configuration data from a string into the class members 
	bool readXmlString(string& str);

	//! write the content of multiple reflection objects into one file
	static bool writeAggrigator(const string& filename, list<CReflection*> mappings);
	//! write the content of multiple reflection objects into one stream
	static bool writeAggrigatorStream(ostream& file, list<CReflection*> mappings);
	//! read the content for multiple reflection objects from one file
	static bool readAggrigator(const string& filename, list<CReflection*> mappings);
	//! add the symbolic names of all binded parameters to the list names
	void getParamNames(list<string>& names);
	//! return a const reference of the value map
	const VariantMap& getValueMap() const { return valueMap; }

	//! generic getter function
	template<typename T> bool get(const string& name, T& value)
	{
		if (valueMap.find(name)!=valueMap.end())
		{
			valueMap[name].get(value);
			return true;
		}
		else return false;
	}

	//! generic setter function
	template<typename T> bool set(const string& name, const T& value)
	{
		if (valueMap.find(name)!=valueMap.end())
		{
			VariantMap::iterator itor;
			itor = valueMap.find(name);
			itor->first;
			itor->second;
			itor->second.type;
			valueMap[name];
			valueMap[name].set(value);
			return true;
		}
		else return false;
	}

	static bool addEnumTable(const string& xpath, const char** names)
	{
		if (enumValues.find(xpath)!=enumValues.end())
			return false;
		enumValues[xpath] = names;
		return true;
	}

	static const char** getEnumTable(const string& xpath)
	{
		if (enumValues.find(xpath)==enumValues.end())
			return 0;
		return enumValues[xpath];
	}
};

}	// end of namespace PCRL
