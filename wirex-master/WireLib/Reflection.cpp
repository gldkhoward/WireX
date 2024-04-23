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
 *  \file   : Reflection.cpp
 *
 *  Project   Wirelib
 *
 *  \Author   Andreas Pott
 *
 *  \Date     06.04.2005
 *
 *********************************************************************
 */ 

#include "Reflection.h"

namespace PCRL {

char CReflection::comment[] = "//";

//! initialize an empty map
map<string, const char**> CReflection::enumValues ;

//! write the binding table to the desired output stream. 
//! The output has one line for each entry
//! [name] [value]
//! \param file [in] a stream object to write into
//! \return true if successful
bool CReflection::writeStream(ostream& file) const
{
	VariantMap::const_iterator itor;
	for (itor=valueMap.begin(); itor != valueMap.end(); ++itor)
	{
		const char** enumtable = this->getEnumTable(itor->second.xpath);
		file << itor->first << " ";
		switch (itor->second.type)
		{
		case TRVBool:	file << *itor->second.value.pBool; break;
		case TRVInt:	file << *itor->second.value.pInt;
			if (enumtable)
				file << " "<< comment << enumtable[*itor->second.value.pInt];
			break;
		case TRVDouble: file << *itor->second.value.pDouble; break;
		case TRVString: file << *itor->second.value.pString; break;
		default: continue;
		}
		file << endl;
	}
    return true;
}

//! write all data to disk. the file is specified by filename
//! \return true if successful
bool CReflection::writeFile(const string& filename) const
{
    ofstream file(filename.c_str());
    return writeStream(file);
}

/*! read the binding table from a given stream
 *  The file must have one for each entry;
 *  line beginning with the comment seperator CReflection::comment[]
 *  are ignored
 */
bool CReflection::readStream(istream& file)
{
	char buffer[2048];
	string str;
	int i = 0;
	while (!file.eof() && i < valueMap.size())
	{
		file >> str;
		if (str.compare(0,2,comment) == 0)
		{	// the line begins with the comment characters; we ignore the rest of the line
			file.getline(buffer,2048);
			continue;
		}
		// str is the first token in the line; we search for the entry in our binding table
		VariantMap::iterator itor = valueMap.find(str);
		if (itor == valueMap.end())
		{	// we did not find the string in the database; we ignore the end of the line
			file.getline(buffer,2048);
			continue;
		}
		// the key exists; we have to read the next token to get the possible value;
		// since we know the key, we expect a certain type which can be found by asking the valueMap
		i++;
		switch (itor->second.type)
		{
		case TRVBool:	file >> *itor->second.value.pBool; break;
		case TRVInt:	file >> *itor->second.value.pInt;  break;
		case TRVDouble: file >> *itor->second.value.pDouble; break;
		case TRVString: file >> *itor->second.value.pString; break;
		default: ;
		}
		// read read and ignore whatever remains of that line
		file.getline(buffer,2048);
	}
    return !file.eof();
}

//! read all data from a file 
bool CReflection::readFile(const string& filename)
{
    ifstream file(filename.c_str());
    return readStream(file);
}

// we implement a number of wrappers to be used by the template version of the setValue Function
void setAttribute(TiXmlElement* root, const string& attribute, double& value)
{ if (root)	root->SetDoubleAttribute(attribute.c_str(),value); }

void setAttribute(TiXmlElement* root, const string& attribute, int& value)
{ if (root)	root->SetAttribute(attribute.c_str(),value); }

void setAttribute(TiXmlElement* root, const string& attribute, string& value)
{ if (root)	root->SetAttribute(attribute.c_str(),value.c_str()); }

void setAttribute(TiXmlElement* root, const string& attribute, bool& value)
{ if (root) root->SetAttribute(attribute.c_str(),value?"true":"false"); }

void getAttribute(TiXmlElement* root, const string& attribute, double& value)
{ if (root)	root->QueryDoubleAttribute(attribute.c_str(),&value); }

void getAttribute(TiXmlElement* root, const string& attribute, int& value)
{ if (root)	root->QueryIntAttribute(attribute.c_str(),&value); }

void getAttribute(TiXmlElement* root, const string& attribute, string& value)
{ if (root)	value = root->Attribute(attribute.c_str()); }

void getAttribute(TiXmlElement* root, const string& attribute, bool& value)
{ if (root) value = strcmp(root->Attribute(attribute.c_str()),"true") == 0?true:false; }

/*! search the node or attribute specified by xpath. if it exists the content
 *  is changed to value. otherwise the respective tags and attributes are
 *  inserted into the structure given by root
 *  \return true, if successful, otherwise false
 *  \todo Fix the bug that hinders the implementation to create more than one 
 *        hierachy level within one call.
 */
template <typename T> bool setNode(TiXmlElement* root, const string& xpath, T& value)
{
	// we do nothing if xpath is empty or if root is NULL
	if (xpath.length() <= 0 || root == 0)
		return false;

	// now we check for the most likely cases of the content of the xpath-string

	// - an attribute is specified with a string "@XYZ"
	if (xpath.size()>1 &&xpath[0] == '@')
	{
		string attrib = xpath.substr(1,string::npos);
		// ** implement the set function here **
		setAttribute(root,attrib,value);
		return true;
	}

	// - the path starts with "/", in the context of this function, this is an error
	if (xpath[0] == '/')
		// although part of the xpath syntax, we do not support reference to the root node here
		return false;

	// - the path starts with "../", relating the following string to the parent element
	if (xpath.size()>3 && xpath.compare(0,2,"../") == 0)
	{
		// access the higher hierachie level in the xml tree
		if (root->Parent() != 0 && xpath.size() > 3)
			return setNode(root->Parent()->ToElement(),xpath.substr(3,string::npos),value);
		else
			return false;
	}

	// - the xpath starts with "./", relating the following string to the current element
	if (xpath.size()>2 && xpath.compare(0,1,"./") == 0)
		// we cut this sequence from the string and make a recursive call will the shorter string
		return setNode(root,xpath.substr(2,string::npos),value);

	// - a path is specified, since the xpath string has the structure "XYZ/..."
	size_t pos = xpath.find('/');
	if (pos == string::npos)
	{
		//! \todo Provide an appropirate implementation for writing node data (and for reading it respectively)
		// the following implementation seems to be not in use and has definitely errors
		// currently writing a node (rather than an note's attribute) is not used by the implemetation
		// an implementation should be similar to the overloaded versions of setAttribute, i.e.
		// overloaded versions of setNode(node,value) are required to perform reading and writing
		// however, since tinyXML provides no conviniance functions for writing node data
		// we skip the implemenation for now
		// btw: the current implementation for reading the node does not support reading the data
		// either. therefore, it is unlikely, that that errors occur (based on the state of the
		// rest of the wirelib implementation). 
		return false;

		char Value[50];
//		sprintf_s(Value,50,"%15.15f",value);			//!< *** this needs to be fixed; this kind of formatting is not save when makeing the function a template and the type of value becomes general ***

		// the whole xpath string is the desired tag name
		TiXmlNode *child = root->FirstChild(xpath.c_str());
		if (!child)
		{
			// no such child here; thus, we create one
			TiXmlElement Child(xpath.c_str());
			TiXmlText content(Value);
			Child.InsertEndChild(content);
			root->InsertEndChild(Child);
			return true;
		}
		child->SetValue(Value);
		return true;
	}
	else
	{
		// paranoia test
		if (xpath.size() <= pos)
			return false;
		// the first part of the string is a tag-name but we want to access its sub-items
		string tag = xpath.substr(0,pos);
		TiXmlNode *child = root->FirstChild(tag.c_str());
		if (child)
			return setNode(child->ToElement(),xpath.substr(pos+1,string::npos),value);
		else
		{
			// create the node
			TiXmlElement Child(tag.c_str());
			bool res = setNode(&Child,xpath.substr(pos+1,string::npos),value);
			if (res)
			{
				root->InsertEndChild(Child);
				return true;
			}
			else
				return false;
		}
	}

	// for some reason we found nothing to do with the xpath string; perhaps the xpath string is not valid
	return false;
}

/*! use the xpath string to search for the node or attribute starting at root. The implementation
 *  for xpath parsing is almost the same as for the setNode but we return false if some node could not
 *  be found.
 *  \return true, if the value was read correctly, otherwise false */
template <typename T> bool getNode(TiXmlElement* root, const string& xpath, T& value)
{
	// we do nothing if xpath is empty or if root is NULL
	if (xpath.length() <= 0 || root == 0)
		return false;

	// now we check for the most likely cases of the content of the xpath-string

	// - an attribute is specified with a string "@XYZ"
	if (xpath.size() > 1 &&xpath[0] == '@')
	{
		string attrib = xpath.substr(1,string::npos);
		// ** implement the set function here **
		getAttribute(root,attrib,value);
		return true;
	}

	// - the path starts with "/", in the context of this function, this is an error
	if (xpath[0] == '/')
		// although part of the xpath syntax, we do not support reference to the root node here
		return false;

	// - the path starts with "../", relating the following string to the parent element
	if (xpath.size() > 3 && xpath.compare(0,2,"../") == 0)
	{
		// access the higher hierachie level in the xml tree
		if (root->Parent() != 0 && xpath.size() > 3)
			return getNode(root->Parent()->ToElement(),xpath.substr(3,string::npos),value);
		else
			return false;
	}

	// - the xpath starts with "./", relating the following string to the current element
	if (xpath.size()>2 && xpath.compare(0,1,"./") == 0)
		// we cut this sequence from the string and make a recursive call will the shorter string
		return getNode(root,xpath.substr(2,string::npos),value);

	// - a path is specified, since the xpath string has the structure "XYZ/..."
	size_t pos = xpath.find('/');
	if (pos == string::npos)
	{
		// the whole xpath string is the desired tag name
		TiXmlNode *child = root->FirstChild(xpath.c_str());
		if (!child || !child->ToElement())
			return false;
		// we will need some kind of parsing and value conversion
		// value = child->ToElement()->GetText();
		return true;
	}
	else
	{
		// paranoia test
		if (xpath.size() <= pos)
			return false;
		// the first part of the string is a tag-name but we want to access its sub-items
		string tag = xpath.substr(0,pos);
		TiXmlNode *child = root->FirstChild(tag.c_str());
		if (child)
			return getNode(child->ToElement(),xpath.substr(pos+1,string::npos),value);
		else
			return false;
	}

	// for some reason we found nothing to do with the xpath string; perhaps the xpath string is not valid
	return false;
}

//! internal function to perform the xml read function 
bool CReflection::readXmlCore(TiXmlElement* root)
{
	if (!root)
		return false;
	// loop through all elements and call the getNode function
	VariantMap::const_iterator itor;
	for (itor=valueMap.begin(); itor != valueMap.end(); ++itor)
	{
		switch (itor->second.type)
		{
		case TRVBool:	getNode(root,itor->second.xpath,*itor->second.value.pBool); break;
		case TRVInt:	getNode(root,itor->second.xpath,*itor->second.value.pInt); break;
		case TRVDouble: getNode(root,itor->second.xpath,*itor->second.value.pDouble); break;
		case TRVString: getNode(root,itor->second.xpath,*itor->second.value.pString); break;
		default: continue;
		}
	}

	return true;
}

//! internal function to perform the xml read function
//! \todo Implement better error handling
bool CReflection::writeXmlCore(TiXmlElement* root)
{
	if (!root)
		return false;
	// loop through all elements and call the getNode function
	VariantMap::const_iterator itor;
	for (itor=valueMap.begin(); itor != valueMap.end(); ++itor)
	{
		switch (itor->second.type)
		{
		case TRVBool:	setNode(root,itor->second.xpath,*itor->second.value.pBool); break;
		case TRVInt:	setNode(root,itor->second.xpath,*itor->second.value.pInt); break;
		case TRVDouble: setNode(root,itor->second.xpath,*itor->second.value.pDouble); break;
		case TRVString: setNode(root,itor->second.xpath,*itor->second.value.pString); break;
		default: continue;
		}
	}

	return true;
}

//! internal function to perform the xml read function
//! \todo Implement better error handling
bool CReflection::writeXmlSchemeCore(TiXmlElement* root)
{
	if (!root)
		return false;
	// in addition to the data tree modelled through the internal structure we write a redundant copy of the
	// data as scheme-tags in the file. in contrast to the data written with writeXmlCore, we can reconstruct
	// the full data structure of the valueMap from the data in the scheme table
	VariantMap::const_iterator itor;
	for (itor=valueMap.begin(); itor != valueMap.end(); ++itor)
	{
		TiXmlElement node("scheme");
		switch (itor->second.type)
		{
		case TRVBool: 
			node.SetAttribute("type","b");
			setAttribute(&node,"value",*itor->second.value.pBool);
			break;
		case TRVInt: 
			node.SetAttribute("type","i");
			setAttribute(&node,"value",*itor->second.value.pInt);
			break;
		case TRVDouble: 
			node.SetAttribute("type","d");
			setAttribute(&node,"value",*itor->second.value.pDouble);
			break;
		case TRVString: 
			node.SetAttribute("type","s");
			setAttribute(&node,"value",*itor->second.value.pString);
			break;
		}
		node.SetAttribute("name",itor->first.c_str());
		node.SetAttribute("xpath",itor->second.xpath.c_str());
		root->InsertEndChild(node);
	}
	return true;
}

/*! create and save the all data into an xml file.
 *  \return true, if successful otherwise false
 */
bool CReflection::writeXmlFile(const string& filename)
{
	TiXmlDocument doc;
	TiXmlElement node("xml");
	// write the data of all maps into the node
	writeXmlCore(&node);
	// insert the result
	doc.InsertEndChild(node);
	if (writeSchemeData)
		writeXmlSchemeCore(doc.RootElement());
	// save file to disk
	doc.SaveFile(filename.c_str());
	return true;
}

//! read the data from a file and copy the read values into the binded variables
//! \param filename [in] name of the xml file
//! \return true, if successful otherwise false
bool CReflection::readXmlFile(const string& filename)
{
	TiXmlDocument doc(filename.c_str());
	if (!doc.LoadFile())
		return false;
	TiXmlElement* root = doc.RootElement();
	// read the data for all data maps
	readXmlCore(root);

	return true;
}

//! store the configuration data in a string
bool CReflection::writeXmlString(string& str)
{
	TiXmlElement node("xml");
	writeXmlCore(&node);
	TiXmlDocument doc;
	doc.InsertEndChild(node);
	// create a printer that convert the data into a string
	TiXmlPrinter printer;
	if (!doc.Accept(&printer))
		return false;
	str = printer.CStr();
	return true;
}

//! flash the configuration data from a string into the class members
//! \todo: Fix me: This does not work yet; how can we use a std::string where a FILE* is required?
bool CReflection::readXmlString(string& str)
{
	TiXmlDocument doc;
	doc.LoadFile();
	doc.Parse(str.c_str());
	return true;
}

//! write the content of multiple reflection objects into one file
//! this function must be a static member to maintain the encapsulation.
bool CReflection::writeAggrigator(const string& filename, list<CReflection*> mappings)
{
	TiXmlElement node("xml");
	for (list<CReflection*>::iterator itor=mappings.begin(); itor != mappings.end(); ++itor)
		(*itor)->writeXmlCore(&node);
	TiXmlDocument doc;
	doc.InsertEndChild(node);
	// save the file to disk
	doc.SaveFile(filename.c_str());
	return true;
}

//! write the content of multiple reflection objects into one file
//! this function must be a static member to maintain the encapsulation.
bool CReflection::writeAggrigatorStream(ostream& file, list<CReflection*> mappings)
{
	TiXmlElement node("xml");
	for (list<CReflection*>::iterator itor=mappings.begin(); itor != mappings.end(); ++itor)
		(*itor)->writeXmlCore(&node);
	TiXmlDocument doc;
	doc.InsertEndChild(node);
	// save to memory 
	TiXmlPrinter printer;
	printer.SetIndent( " " );

	doc.Accept( &printer );
	std::string xmltext = printer.CStr();
	file << xmltext;	
	return true;
}


//! read the content for multiple reflection objects from one file
//! simiar to robot document, we execpt filename to be the content
//! of an xml file rather than the name of the file to open. in this
//! case, the content in filename must start with '<'.
bool CReflection::readAggrigator(const string& filename, list<CReflection*> mappings)
{
	TiXmlDocument doc;
	if (filename.length()>0 && filename.at(0) == '<')
	{
		if (!doc.Parse(filename.c_str()))
			return false;
	}
	else
		if (!doc.LoadFile(filename.c_str()))
			return false;

	TiXmlElement* root = doc.RootElement();
	// read the data for all data maps
	for (list<CReflection*>::iterator itor=mappings.begin(); itor != mappings.end(); ++itor)
		(*itor)->readXmlCore(root);

	return true;
}

//! add the symbolic names of all binded parameters to the list names
void CReflection::getParamNames(list<string>& names)
{
	for (VariantMap::iterator itor=valueMap.begin(); itor != valueMap.end(); ++itor)
		names.push_back(itor->first);
}

} // end namespace PCRL
