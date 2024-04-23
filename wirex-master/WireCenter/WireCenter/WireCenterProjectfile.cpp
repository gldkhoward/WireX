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

// CWireCenterDocDb.cpp : Implementierung der Klasse CWireCenterDocDb
//

#include "StdAfx.h"
#include "WireCenterProjectfile.h"

// experimental usage of SQLITE as container file

// we include the sqlite3 libraray through a compiler pragma
// rather than changing the project file
#ifdef NDEBUG
	#pragma comment(lib, "sqliteMD.lib")
#else
	#pragma comment(lib, "sqliteMDd.lib")
#endif

/*! write a file like data entry into the internal file system of the database (dbfs)
 *  \param id provide an identifier for the record to be inserted
 *  \param name specify the internal filename in database filesystem
 *  \param content the string of the textfile to be stored in the dbfs
 *  \return true, if successful
 */
bool CWireCenterProjectfile::storeObject(const int id, const string& name, const string& content)
{
	// we format an sql string to execute the insert command
	char sql[1024];
	sqlite3_stmt * stmt;
	sprintf_s(sql,"INSERT OR REPLACE INTO 'wcDBFS' VALUES (%i, '%s', ?);",id,name.c_str());
	// 1) insert the algorithm configuration into the database
	if (sqlite3_prepare_v2(pDb, sql, -1, &stmt, 0))
	{
		printf("SQLITE prepare: %s\n",sqlite3_errmsg(pDb));
		//printf("Error while preparing the statement\n");
	}
	// write the actual data here!
	if (sqlite3_bind_blob(stmt,1,content.c_str(),content.length(),SQLITE_TRANSIENT))
		printf("SQLITE bind: %s\n",sqlite3_errmsg(pDb));
	if (sqlite3_step(stmt) != SQLITE_DONE)
		printf("SQLITE step: %s\n",sqlite3_errmsg(pDb));
	// finall we reset the statement
	if (sqlite3_reset(stmt))
		printf("SQLITE reset: %s\n",sqlite3_errmsg(pDb));
	if (sqlite3_clear_bindings(stmt))
		printf("SQLITE clear_bind: %s\n",sqlite3_errmsg(pDb));
	return true;
}

/*! receive the content of an entry from the dbfs specified by its id
 *  \param id [in] provide an identifier for the record to be inserted
 *  \param content [out] the string of the textfile return by the function
 *  \return true, if successful. if the function returns false the content of
 *		content is undefined.
 */
bool CWireCenterProjectfile::restoreObject(const int id, string& content)
{
	// prepare the query
	sqlite3_stmt * stmt;
	char sql[1024];
	sprintf_s(sql,1024,"SELECT data FROM wcDBFS WHERE ID=%i;",id);
	if (sqlite3_prepare_v2(pDb,sql,-1, &stmt,0))
	{
		printf("Failed to prepare sql statement\n");
		return false;
	}
	
	// if step does not return SQLITE_ROW, we were not able to receive the desired information
	if (sqlite3_step(stmt)!=SQLITE_ROW)
		printf("SQLITE prepare: %s\n",sqlite3_errmsg(pDb));

	// we expect exactly one or zeros entries. only one column is selected in sql and
	// only one data record may be connected to one id in the dbfs. we can easily get the content 
	content = reinterpret_cast<const char*>(sqlite3_column_text(stmt,0)); 
	
	// free the resources allocated by the statement
	if (sqlite3_reset(stmt))
		printf("reset failed\n");

	return true;
}

/*! this function is similar to storeObject() but the content is taken from the file with filename
 *  rather than from a memeory buffer. after extracting the file, storeObject() is called to
 *  write the content of the file into the dbfs
 */
bool CWireCenterProjectfile::storeFile(const int& id, const string& name, const string& filename)
{
	// insert the robot configuration into the internal file system of the data base

	// firstly, we copy the robot configration into a string object
	// implementation for reading a file into a string
	std::ifstream file(filename.c_str());
	if (!file.good())
		return false;
	std::stringstream ssbuffer;
	ssbuffer << file.rdbuf();
	// now we have the content in the stream buffer and we can send in into the database
	return storeObject(id, name, ssbuffer.str());
}

/*! this function uses an reflector object to store its content into a
 *  database table of the given name
 *  \param table [in] the symbolic name of the table in the database where the content
 *         of the reflection object is stored
 *  \param reflector [in] the source of the data to be stored in the database
 *  \return true, if successful
 */
bool CWireCenterProjectfile::storeReflector(const string& table, PCRL::CReflection& reflector)
{
	sqlite3_stmt * stmt;
	// not the two '?' that are going to be replayed by the bind calls later
	char sql[1024];
	sprintf_s(sql,"INSERT OR REPLACE INTO '%s' VALUES (? , ?);",table.c_str());
	// string SQL = "INSERT INTO 'wcSettings' VALUES (? , ?);";

	// now we prepare the generic sql statement for later use
	if (sqlite3_prepare_v2(pDb, sql, -1, &stmt, 0))
		printf("SQLITE prepare: %s\n",sqlite3_errmsg(pDb));

	// after preparing the query string we can use it multiple times and change the data to be written
	// through call to the sqlit3_bind_X functions
	for (auto itor = reflector.getValueMap().begin(); itor!=reflector.getValueMap().end(); ++itor)
	{
		// we substitute the first and second parameter (index 1 and index 2) into the prepared SQL statement
		if (sqlite3_bind_text(stmt,1,itor->first.c_str(),itor->first.length(),SQLITE_TRANSIENT))
			printf("SQLITE bind_text: %s\n",sqlite3_errmsg(pDb));
		// currently, we only support storing bool data
		if (itor->second.type==PCRL::TRVBool)
		{
			if (sqlite3_bind_text(stmt,2,*itor->second.value.pBool?"1":"0",1,0))
				printf("SQLITE bind: %s\n",sqlite3_errmsg(pDb));
		} else if (itor->second.type == PCRL::TRVDouble)
		{
			if (sqlite3_bind_double(stmt,2,*itor->second.value.pDouble))
				printf("SQLITE bind_double: %s\n",sqlite3_errmsg(pDb));
		} else if (itor->second.type == PCRL::TRVInt)
		{
			if (sqlite3_bind_int(stmt,2,*itor->second.value.pInt))
				printf("SQLITE bind_int: %s\n",sqlite3_errmsg(pDb));
		} else if (itor->second.type == PCRL::TRVString)
		{
			if (sqlite3_bind_text(stmt,2,itor->second.value.pString->c_str(),itor->second.value.pString->length(),0))
				printf("SQLITE bind_text: %s\n",sqlite3_errmsg(pDb));
		}

		// we let sqlite execute the statement with the binded values
		if (sqlite3_step(stmt)!=SQLITE_DONE)
			printf("SQLITE step: %s\n",sqlite3_errmsg(pDb));
		// finally we reset the statement
		if (sqlite3_reset(stmt))
			printf("SQLITE reset: %s\n",sqlite3_errmsg(pDb));
		if (sqlite3_clear_bindings(stmt))
			printf("SQLITE clear_bindings: %s\n",sqlite3_errmsg(pDb));
	}
	return true;
}

bool CWireCenterProjectfile::restoreReflector(const string& table, PCRL::CReflection& reflector)
{
	sqlite3_stmt* stmt;
	// not the two '?' that are going to be replayed by the bind calls later
	char sql[1024];
	sprintf_s(sql,"SELECT ( value ) FROM '%s' WHERE name =  ? ;",table.c_str());

	// now we prepare the generic sql statement for later use
	if (sqlite3_prepare_v2(pDb, sql, -1, &stmt, 0))
		printf("SQLITE prepare: %s\n",sqlite3_errmsg(pDb));

	// after preparing the query string we can use it multiple times and change the data to be written
	// through call to the sqlit3_bind_X functions
	for (auto itor = reflector.getValueMap().begin(); itor!=reflector.getValueMap().end(); ++itor)
	{
		// we substitute the first and second parameter (index 1 and index 2) into the prepared SQL statement
		if (sqlite3_bind_text(stmt,1,itor->first.c_str(),itor->first.length(),0))
			printf("SQLITE bind_text: %s\n",sqlite3_errmsg(pDb));
				
		// we let sqlite execute the statement with the binded values
  		if (sqlite3_step(stmt)!=SQLITE_ROW)
			printf("SQLITE step: %s\n",sqlite3_errmsg(pDb));

		// currently, we only support storing bool data
		if (itor->second.type==PCRL::TRVBool)
		{
			// if the executing was successful we should be able to receive exactly one value
			if (sqlite3_column_int(stmt,0))
				*(itor->second.value.pBool) = true;
			else
				*(itor->second.value.pBool) = false;
		} else if (itor->second.type==PCRL::TRVBool)
			*itor->second.value.pDouble = sqlite3_column_double(stmt,0);
		else if (itor->second.type == PCRL::TRVInt)
			*itor->second.value.pInt = sqlite3_column_int(stmt,0);
//		else if (itor->second.type == PCRL::TRVString)
//			*itor->second.value.pString = sqlite3_column_text(stmt,0);
		// finally we reset the statement
		if (sqlite3_reset(stmt))
			printf("SQLITE reset: %s\n",sqlite3_errmsg(pDb));
		if (sqlite3_clear_bindings(stmt))
			printf("SQLITE clear: %s\n",sqlite3_errmsg(pDb));
	}
	return true;
}

//! this internal functions prepare the database file by creating the
//! tables and possible further elements of the sqlite scheme. Thus
//! in this file the internal structure of the store is defined
void CWireCenterProjectfile::createFileLayout()
{
	// create a table with a file system like data structure:
	// table name: wcDBFS
	//	id - integer primare key 
	//	name - text
	//  data - blob
	if (sqlite3_exec(pDb,"CREATE TABLE IF NOT EXISTS `wcDBFS` (`id` integer PRIMARY KEY AUTOINCREMENT, `name` text, `data` blob);",0,0,0))
		printf("SQLITE create table: %s\n",sqlite3_errmsg(pDb));

	// create a table for storing the settings represented by reflection objects
	// table name: wcSettings
	//	name - text primare key 
	//  value - text 
	if (sqlite3_exec(pDb,"CREATE TABLE IF NOT EXISTS `wcSettings` (`name` text PRIMARY KEY, `value` text);",0,0,0))
		printf("SQLITE create table: %s\n",sqlite3_errmsg(pDb));
}

/*! CWireCenterDoc has no intrinsic support for reflection. some configuration 
 *  variables in CWireCenterDoc need management through a reflector object. 
 *  Since we do the serialization into the database through a CReflection 
 *  object, we declare how these additionally values shall be stored
 *  in the database.
 *  currently we store the Roi in the database
 */
void CWireCenterProjectfile::createDocumentReflector(PCRL::CReflection& reflector)
{
	// register the ROI for storage
	reflector.bind(pDoc->Roi.lower().x(),"Roi.minx");
	reflector.bind(pDoc->Roi.lower().y(),"Roi.miny");
	reflector.bind(pDoc->Roi.lower().z(),"Roi.minz");
	reflector.bind(pDoc->Roi.upper().x(),"Roi.maxx");
	reflector.bind(pDoc->Roi.upper().y(),"Roi.maxy");
	reflector.bind(pDoc->Roi.upper().z(),"Roi.maxz");
}

/*! create a database "filename" and store the WireCenter project in that
 *  database.
 */
bool CWireCenterProjectfile::store(const string& filename)
{
	// the following block is a prototypic implementation of using sqlite
	// as a project level file format
	
	//! \todo In later versions of the file format implementation we want to be able
	//! to update a database file. However, for now, we delete the database to write
	//! only into frech databases
	DeleteFile(filename.c_str());

	// open the database file
	if (sqlite3_open(filename.c_str(),&pDb))
	{
		printf("Error while opening the database\n");
		return false;
	}

	createFileLayout();

	// after setting up the structure of the database we can polulate it with application data

	TCHAR szTempDirectory[MAX_PATH] ;
    GetTempPath(MAX_PATH, szTempDirectory) ;
	TCHAR szTempFileName[MAX_PATH] ;
    GetTempFileName(szTempDirectory, "wcTempFile", 0, szTempFileName) ;

	// we write the current robot to a temp file and copy it into the database afterwards
	if (pDoc->robotDoc.saveXml(szTempFileName))
	// store the robot document
		storeFile(idRobotGeometry, "robot.wcrfx", szTempFileName);

	if (pDoc->robotDoc.saveAlgorithmConfiguration(szTempFileName))
	// store the config algorithm
		storeFile(idAlgorithmConfig,"AlgorthmConfig.xml",szTempFileName);

	if (pDoc->py_filename.GetLength()>0)
		if (!storeFile(idPythonScript, "python.py", pDoc->py_filename.GetBuffer()))
			printf("Error while saving the python script into the libraray\n");
	
	// insert the robot requirement setting into the dbfs 
	if (pDoc->robotDoc.AppReq.saveXML(szTempFileName))
		storeFile(idApplicationRequirement, "AppReq.wcrqx", szTempFileName);

	// insert the winch data into the dbfs
	if (pDoc->robotDoc.currentWinch->saveXML(szTempFileName))
		storeFile(idWinch, "winch.wcwdqx", szTempFileName);

	// insert the cable data into the dbfs
	if (pDoc->robotDoc.currentCable->saveXML(szTempFileName))
		storeFile(idCable, "cable.wccdqx", szTempFileName);

	// usage of the temp file is over; we delete the file
	DeleteFile(szTempFileName);

	//! store additional settings from WireCenterDoc in the configuration table
	PCRL::CReflection reflector;
	createDocumentReflector(reflector);
	storeReflector("wcSettings",reflector);

	// store the configuration of the view class. we loop through all parameters that we can find in reflection
	// of CWireCenterView and write an entry into the database
	// in the refactored version we use prepare and bind for a more elegant and hopefull more efficient code
	storeReflector("wcSettings", *pView);

	// further items to be stored in the project file 
	// * insert the cable data into the database
	// * store the filename(s) and path settings into the data base
	// * store the current pose list in the data base
	// * store python filename, the current NC program, and report name in the data base

	// close the database after all information have been submitted
	sqlite3_close(pDb);

	return true;
}


/*! read the project file from disk and extract all relevent properties
 */
bool CWireCenterProjectfile::restore(const string& filename)
{
	if (sqlite3_open(filename.c_str(),&pDb))
	{
		printf("Error while opening the db\n");
		return false;
	}

	// restore the main settings settings
	if (!restoreReflector("wcSettings",*pView))
		printf("Warning: Settings from table wcSettings were not extracted\n");

	//! store additional settings from WireCenterDoc in the configuration table
	PCRL::CReflection reflector;
	createDocumentReflector(reflector);
	if (!restoreReflector("wcSettings",reflector))
		printf("Warning: Could not extract document settings from table wcSettings\n");

	// restore a complete file from the dbfs
	// robot geometry
	string content;
	if (restoreObject(idRobotGeometry,content))
	{
		if (!pDoc->robotDoc.loadXml(content))
			printf("Error while parsing the dbfs embedded robot.wcrfx file\n");
	}
	else
		printf("Restore object for id=1 failed\n");

	// algorithm configuration
	content="";
	if (restoreObject(idAlgorithmConfig,content))
	{
		if (!pDoc->robotDoc.loadAlgorithmConfiguration(content))
			printf("Error while parsing the dbfs embedded AlgorithmConfig.xml\n");
	}
	else
		printf("Restore object for id=2 failed\n");

	// python script
/*	content="";
	if (restoreObject(idPythonScript,content))
	{
		if (!pDoc->robotDoc.loadAlgorithmConfiguration(content))
			printf("Error while parsing the dbfs embedded python.py\n");
	}
	else
		printf("Restore object for id=3 failed\n");*/

	// application requirement
	content="";
	if (restoreObject(idApplicationRequirement,content))
	{
		if (!pDoc->robotDoc.AppReq.loadXML(content))
			printf("Error while parsing the dbfs embedded AppReq.wcrqx\n");
	}
	else
		printf("Restore object for id=4 failed\n");

	/*
	// winch database
	content="";
	if (restoreObject(idWinch,content))
	{
		if (!pDoc->robotDoc.currentWinch->loadXML(content))
			printf("Error while parsing the dbfs embedded winch.wcwdqx\n");
	}
	else
		printf("Restore object for id=5 failed\n");

	// cable database
	content="";
	if (restoreObject(idCable,content))
	{
		if (!pDoc->robotDoc.currentCable->loadXML(content))
			printf("Error while parsing the dbfs embedded cable.wccdqx\n");
	}
	else
		printf("Restore object for id=6 failed\n");*/

	// close the database after all information have been submitted
	sqlite3_close(pDb);

	return true;
}


#ifdef ASP_ORIGINAL_CODE_BASE_FOR_SQLITE3
//! this function is called back from the sqlite database in order to process the data read from the 
//! table. We might consider to have a specific callback for each table of the database
//! to make the interpretion process easier to understand.	
int CWireCenterProjectfile::callback(void *Self, int argc, char **argv, char **azColName)
{
	CWireCenterProjectfile* This = (CWireCenterProjectfile*)Self;
	int i;
	for(i=0; i<argc; i++)
	{
		printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
	}
	// let's parse the content
/*	if (argv[0])
	{
		This->pDoc->robotDoc.loadXml(
	}*/
	return 0;
}

// this is some legacy code with code snipplets from the development of sqlite data access
// we keep it only for limited time until we are sure that the fragments are completely deprecated.
bool store(const string& filename)
{
	// the following block is a prototypic implementation of using sqlite
	// as a project level file format
		
	// open the database file

	if (sqlite3_open(filename.c_str(),&pDb))
		printf("Error while opening the db\n");

	// create a table with a file system like data structure
	if (sqlite3_exec(pDb,"CREATE TABLE `wcDataBlock` (`id` integer PRIMARY KEY AUTOINCREMENT, `name` text, `data`	blob);",0,0,0))
		printf("Error while creating table\n");

	// create a table with a file system like data structure
	if (sqlite3_exec(pDb,"CREATE TABLE `wcSettings` (`name` text PRIMARY KEY, `value` text);",0,0,0))
		printf("Error while creating table\n");

/*		// insert the robot configuration into the internal file system of the data base
	// firstly, we copy the robot configration into a string object
	std::ifstream t;
	int length;
	t.open("robot.wcrfx");      // open input file
	t.seekg(0, std::ios::end);    // go to the end
	length = t.tellg();           // report location (this is the length)
	t.seekg(0, std::ios::beg);    // go back to the beginning
	char* buffer = new char[length];    // allocate memory for a buffer of appropriate dimension
	t.read(buffer, length);       // read the whole file into the buffer
	t.close();                    // close file handle
*/
/*		// secondly, we format an sql string to execute the insert command
	char* sql = new char[length+200];
	sprintf(sql,"INSERT INTO 'wcDataBlock' VALUES (1, 'robot.wcrfx', '%s');",buffer);
	if (sqlite3_exec(pDb,sql,0,0,0))
		printf("Error while inserting values\n");
	delete [] sql;*/


/*		// 1) insert the algorithm configuration into the database
	sqlite3_stmt * stmt;
	if (sqlite3_prepare_v2(pDb, "INSERT INTO 'wcDataBlock' Values (2, 'AlgorthmConfig.xml', ?);",-1,&stmt,0))
		printf("Error while preparing the statement\n");
	// write the actual data here!
	if (sqlite3_bind_text(stmt,1,"<xml></xml>",11,0))
		printf("Error while binding the value\n");
	if (sqlite3_step(stmt) != SQLITE_DONE)
		printf("Error while executing\n");
	// finall we reset the statement
	if (sqlite3_reset(stmt))
		printf("reset failed\n");
	if (sqlite3_clear_bindings(stmt))
		printf("clear_binding failed\n");
*/
	// we write the current robot to file and copy it into the database afterwards
	pDoc->robotDoc.saveXml("_robot.wcrfx");
	// store the robot document
	storeFile(1,"robot.wcrfx","_robot.wcrfx");

	pDoc->robotDoc.saveAlgorithmConfiguration("_AlgorthmConfig.xml");
	// store the config algorithm (currently with a dummy string)
	storeFile(2,"AlgorthmConfig.xml","_AlgorthmConfig.xml");

	// 2) insert the robot requirement setting into the database
	// 3) insert the winch data into the database
	// 4) insert the cable data into the database
	// 5) store the filename(s) and path settings into the data base
	// 6) store the current pose list in the data base
	// 7) store python filename, the current NC program, and report name in the data base
	// 8) store the Roi in the database
	// 

	// store the configuration of the view class. we loop through all parameters that we can find in reflection
	// of CWireCenterView and write an entry into the database
#ifdef ASP_SQLITE_ORIGIAL_VERSION
	for (auto itor = CWireCenterView::This->getValueMap().begin(); itor!=CWireCenterView::This->getValueMap().end(); ++itor)
	{
		char sql[1024];
		// currently, we only support storing bool data
		if (itor->second.type==PCRL::TRVBool)
		{
			sprintf(sql,"INSERT INTO 'wcSettings' VALUES ('%s', '%s');",itor->first.c_str(),*itor->second.value.pBool?"1":"0");
			if (sqlite3_exec(pDb,sql,0,0,0))
				printf("Error while inserting values\n");
		}
	}
#else
	// in the refactored version we use prepare and bind for a more elegant and hopefull more efficient code
	storeReflector("wcSettings",*CWireCenterView::This);

/*		// not the two '?' that are going to be replayed by the bind calls later on
	sqlite3_stmt * stmt;
	string SQL = "INSERT INTO 'wcSettings' VALUES (? , ?);";

	// now we prepare the generic sql statement for later use
	if (sqlite3_prepare_v2(pDb, SQL.c_str(), -1, &stmt, 0))
		printf("Error while preparing the query\n");

	// after preparing the query string we can use it multiple times and change the data to be written
	// through call to the sqlit3_bind_X functions
	for (auto itor = CWireCenterView::This->getValueMap().begin(); itor!=CWireCenterView::This->getValueMap().end(); ++itor)
	{
		// currently, we only support storing bool data
		if (itor->second.type==PCRL::TRVBool)
		{
			// we substitute the first and second parameter (index 1 and index 2) into the prepared SQL statement
			if (sqlite3_bind_text(stmt,1,itor->first.c_str(),itor->first.length(),0))
				printf("bind_text failed\n");
			if (sqlite3_bind_text(stmt,2,*itor->second.value.pBool?"1":"0",1,0))
				printf("bind_text failed\n");
			// we let sqlite execute the statement with the binded values
			if (sqlite3_step(stmt)!=SQLITE_DONE)
				printf("SQL execution through sqlite3_step failed\n");
			// finally we reset the statement
			if (sqlite3_reset(stmt))
				printf("reset failed\n");
			if (sqlite3_clear_bindings(stmt))
				printf("clear_binding failed\n");
		}
	}*/
#endif
	// close the database after all information have been submitted
	sqlite3_close(pDb);
	return true;
}

/*
static int callback(void *NotUsed, int argc, char **argv, char **azColName){
int i;
for(i=0; i<argc; i++){
printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
}
printf("\n");
return 0;
}
*/

#endif // ASP_ORIGINAL_CODE_BASE_FOR_SQLITE3
