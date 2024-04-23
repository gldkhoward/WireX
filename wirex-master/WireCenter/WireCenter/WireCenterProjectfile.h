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

/*! \file		WireCenterProjectFile.h
 *
 *  \author		Andreas Pott
 *
 *  \version	1.0
 *  \date		14.01.2015
 * 
 *  \class CWireCenterProjectfile
 *
 *  \brief CWireCenterProjectfile is used to encapsulate the generation of 
 *  application level project files from the current state of the application.
 * 
 *  \par Desciption
 *	This implementation is complete specific for WireCenter. Since
 *	the file format is based on the sqlite3 library, an implementation seperate
 *  from the document class with its build in serialization support is provided.
 *  Amongst others, the project file is setup as a container for embedded text and
 *  binary data. This is achieved inside the sqlite database by setting up a
 *  data table with a file system like structure, which we will call database file 
 *  system (dbfs).
 *
 *  \todo Implement an acceptable error handling without the currently used exessive
 *  printing to the console.
 *  \todo Implement reading methods, to read back the stored data.
 *////////////////////////////////////////////////////////////////////////////

#pragma once
#include "WireCenterView.h"
#include "WireCenterDoc.h"
#include <sqlite/sqlite3.h>

class CWireCenterProjectfile
{
	//! pointer to the data sources
	CWireCenterDoc *pDoc;	//!< the data source of the document class
	CWireCenterView *pView;	//!< the data source of the view class
	sqlite3 *pDb;			//!< the data base object for communication with the sqlite database

	//! unique id for files stored in the dbfs
	typedef enum {
		idRobotGeometry=1,			//!< robot xml geometry file (wcrfx)
		idAlgorithmConfig=2,		//!< algorithm configuration file (xml)
		idPythonScript=3,			//!< current python script (py)
		idApplicationRequirement=4,	//!< application requirement specification (wcrqx)
		idWinch=5,					//!< winch database (wcwdqx)
		idCable=6					//!< cable parameter database (wccdqx)
	} DbfsId;

// internal private operations
	void createDocumentReflector(PCRL::CReflection& reflector);

	//! this functions defines the structure of the database
	void createFileLayout();

	//! write a file like data entry into the internal file system of the database (dbfs)
	bool storeObject(const int id, const string& name, const string& content);
	
	//! read the object from the dbfs
	bool restoreObject(const int id, string& content);

	//! this function is simiar to storeObject but the content is taken from the file with filename
	bool storeFile(const int& id, const string& name, const string& filename);

	//! this function uses an reflector object to store its content into a database table of the given name
	bool storeReflector(const string& table, PCRL::CReflection& reflector);
	
	//! search for the data entries specified by reflector in the table and try to restore these data
	bool restoreReflector(const string& table, PCRL::CReflection& reflector);
	
	//! this function is an example for using a call-back function with the sqlite3 interface
	static int callback(void *NotUsed, int argc, char **argv, char **azColName);
 
public:
	CWireCenterProjectfile(CWireCenterDoc& doc, CWireCenterView& view)
		: pDoc(&doc), pView(&view) {}
	
	//! create a database "filename" and store the WireCenter project in that database.
	bool store(const string& filename);

	//! read by the content of the database into the connected data structure
	bool restore(const string& filename);
};
