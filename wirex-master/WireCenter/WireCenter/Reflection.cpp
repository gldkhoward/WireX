// Reflection.cpp: Implementierung der Klasse CReflection.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Reflection.h"

char CReflection::comment[] = "//";

//! template writer function used for all maps in CReflection
template<class T> void write(ofstream& file, map<string,T> thisMap)
{
    map<string,T>::const_iterator itor;
    for (itor=thisMap.begin(); itor!=thisMap.end(); itor++)
        file << itor->first << " " << *(itor->second) << endl;
}

//! template print function used for all maps in CReflection
template<class T> void write(ostream& file, map<string,T> thisMap) // type of cout is ostream
{
    map<string,T>::const_iterator itor;
    for (itor=thisMap.begin(); itor!=thisMap.end(); itor++)
        file << itor->first << " " << *(itor->second) << endl;
}

//! template read function used for all maps in CReflection
template<class T> void read(ifstream& file, const string& str, map<string,T> thisMap)
{
    map<string, T>::iterator itor=thisMap.find(str);
    if (itor!=thisMap.end())
        file >> *(itor->second);
}

//! template get function
template<class T> bool getValue(const string& str, map<string, T> thisMap, T value)
{
    map<string, T>::iterator itor=thisMap.find(str);
    if (itor!=thisMap.end())
	{
		*value = *(itor->second);
		return true;
	}
	else
		return false;
}

//! template set function
template<class T> bool setValue(const string& str, map<string, T> thisMap, T value)
{
    map<string, T>::iterator itor=thisMap.find(str);
    if (itor!=thisMap.end())
	{
		*(itor->second)=*value;
		return true;
	}
	else
		return false;
}

//! print the content and the state of the binding table to the console
bool CReflection::printBindingTable() const
{
    write(cout,boolMap);
    write(cout,intMap);
    write(cout,doubleMap);
    write(cout,stringMap);
    return true;
}

//! write all data to disk
bool CReflection::WriteFile(const string& filename) const
{
    ofstream file(filename.c_str());
    write(file,boolMap);
    write(file,intMap);
    write(file,doubleMap);
    write(file,stringMap);
    return true;
}

//! read all data from a file
bool CReflection::ReadFile(const string& filename)
{
    ifstream file(filename.c_str());
    while (!file.eof())
    {
        string str;
        file >> str;
        read(file,str,boolMap);
        read(file,str,intMap);
        read(file,str,doubleMap);
        read(file,str,stringMap);
    }
    return true;
}

//! generic getter function for the supported types
bool CReflection::get(const string& name, bool& x) const
{ return getValue(name,boolMap,&x); }

bool CReflection::get(const string& name, int& x) const
{ return getValue(name,intMap,&x); }

bool CReflection::get(const string& name, double& x) const
{ return getValue(name,doubleMap,&x); }

bool CReflection::get(const string& name, string& x) const
{ return getValue(name,stringMap,&x); }

//! generic setter functions for the supported types
bool CReflection::set(const string& name, bool& x) const
{ return setValue(name,boolMap,&x); }

bool CReflection::set(const string& name, int& x) const
{ return setValue(name,intMap,&x); }

bool CReflection::set(const string& name, double& x) const
{ return setValue(name,doubleMap,&x); }

bool CReflection::set(const string& name, string& x) const
{ return setValue(name,stringMap,&x); }
