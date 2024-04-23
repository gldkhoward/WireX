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
 *  \file   : PosePropertyEvaluation.cpp
 *
 *  Project   Wirelib
 *
 *  \Author   Andreas Pott
 *
 *  \data	  23.04.2014
 *
 *********************************************************************
 */ 

#include "PosePropertyEvaluation.h"
#include "PoseEvaluator.h"
#include <time.h>
#include <motionPlanning\Utilities.h>

namespace PCRL {

// helper functions
/////////////////////////////////////////////////////////////////////

// implementation of CPoseEvaluator
/////////////////////////////////////////////////////////////////////

CPosePropertyEvaluation::CPosePropertyEvaluation(CRobotData& robot) : CAlgorithm(robot)
{
	results.resize(1,1);	// we make the matrix 1x1 by default to indicate its empty
	bCheckZeroThreshold = false; 
	//Add Pose Mapper by Default
	PCRL::emPoseProperty::Type PPT = (PCRL::emPoseProperty::Type)(PCRL::emPoseProperty::PoseMapper);
	this->attachProperty(new PCRL::CActivePoseProperty(PPT, this->createPoseProperty(PPT)));
}

CPosePropertyEvaluation::~CPosePropertyEvaluation() 
{
	for (auto itor=activeProperties.begin(); itor != activeProperties.end(); ++itor)
		delete (*itor)->Property;
}	

/*! add a pose property object to the internal list
 *  \param PP [in] the pose property to be added for evaluation
 *  \return true, if property was accepted and added to the list
 */
bool CPosePropertyEvaluation::attachProperty(CActivePoseProperty* pAPP)
{
	// before accepting a new pose property we test it for correctness
	int count = pAPP->Property->getPropertyCount();
	vector<string> names;
	if (!pAPP->Property->getPropertyNames(names))
		return false;
	if (names.size() != count)
	{	
		cout << "invalid Pose evaluator. check property count\n";
		return false;
	}
	// adding a new property invalidates the result data; since we do not know the
	// amount of rows we set the matrix to 1x1
	results.resize(1,1);
	// everything seems to be fine; we accept the property and put it into the list
	activeProperties.push_back(pAPP);
	return true;
}

bool CPosePropertyEvaluation::removeProperty(CActivePoseProperty* pAPP) {
	delete pAPP->Property;
	activeProperties.remove(pAPP);
	return true;
}


bool CPosePropertyEvaluation::removeAllProperties() {
	for (auto itor=activeProperties.begin(); itor != activeProperties.end(); ++itor)
		delete (*itor)->Property; 
	activeProperties.clear();
	return true;
}

bool CPosePropertyEvaluation::moveUp(const CPoseProperty* target)
{
	auto itor=activeProperties.begin();
	while (itor != activeProperties.end() && (*itor)->Property != target)
		++itor;
	if (itor == activeProperties.end())
		return false;	// nothing found, we reached the end without finding the respective element

	// we found the matching item and itor points at it
	// now, lets swap it with its precursor (which make the element move "up" in the list
	auto itor2 = itor;
	--itor2;
	if (itor2 == activeProperties.end())
		return false;
	// exchange the content of the iterator; since the content is a pointer this is possible
	swap(*itor,*itor2);
	
	return true;
}


bool CPosePropertyEvaluation::moveDown(const CPoseProperty* target)
{
	auto itor=activeProperties.begin();
	while (itor != activeProperties.end() && (*itor)->Property != target)
		++itor;
	if (itor == activeProperties.end())
		return false;
	// we found the matching item in itor
	// it seems that the container list does not directly support a swap operation...
	auto itor2 = itor;
	++itor2;
	if (itor2 == activeProperties.end())
		return false;
	swap(*itor,*itor2);
	
	return true;
}


//! count the number of all active properties provided through the attached objects
int CPosePropertyEvaluation::getPropertyCount()
{
	int PropertyCount=0;
	for (auto itor= activeProperties.begin(); itor != activeProperties.end(); itor++)
		PropertyCount+= (*itor)->Property->getPropertyCount();
	return PropertyCount;
}


/*! perform the calculation for each pose and each property and store the computed
 *  data in an internal storage. This implementation loops through all properties
 *  and performs the calculation in column-blocks.
 *  The internal data stores is deleted when this function is called, so ealier
 *  results will be lost.
 *  \param poseList [in] The coordinates of the pose to be evaluationed
 *  \return true, if successful, otherwise false. The content of the result 
 *   matrix is undefined when false is returned.
 */
bool CPosePropertyEvaluation::calculate(CPoseListKinetostatic& poseList)
{
	// verify that the configuration is valid
	if (getPropertyCount() == 0)
		return false;	// no properties to process

	const int iInfosize = 2;	// we have 2 additional columns related to id, time

	// init the data model
	// set the size of the result table; now we know the number of rows for the matrix
	results.resize(poseList.size(), getPropertyCount()+iInfosize);		// we have currently 2 additional columns related to id, time
	// delete the content of the meta data before new data is generated
	meta.clear();

	// update the header
	header.clear();
	// before adding the names of the properties, we have to add the columns with the id and time
	//! \todo Introduce flags for customizing which standard columns are written to the table
	header.push_back("id");
	header.push_back("t");

	// after adding the default column names, we append the names of the other properties
	for (auto itor= activeProperties.begin(); itor != activeProperties.end(); itor++)
		(*itor)->Property->getPropertyNames(header);

	// prepare the data in the results block

	// write the info columns with id and time
	int id=0;
	for (CPoseListKinetostatic::iterator itor=poseList.begin(); itor != poseList.end(); ++itor, ++id)
	{
		double time=id;
		// write the initial two (iInfosize) info columns
		results.block(id,0,1,iInfosize) << id, time;
	}

	// compute and add meta data for the first two columns
	for (int i=0; i<iInfosize; i++)
	{
		CProperty prop;
		prop.header = header[i];
		prop.min = results.col(i).minCoeff();
		prop.max = results.col(i).maxCoeff();
		prop.mean = results.col(i).mean();
		prop.std = stddev(results.col(i));				
		prop.calculationTime = 0;	// we do not provide computation time for the info columns
		meta.push_back(prop);
	}	

	// start the iteration after the info columns 
	int offset = iInfosize;	
	// outer loop through all properties (columns) registrated for CPoseEvaluator
	for (auto iProp = activeProperties.begin(); iProp != activeProperties.end(); ++iProp)
	{
		time_t start = clock();	// start time measurement
		MatrixXd values;		// reserve space for the computed properties

		// inner loop through all poses (rows)
		int id=0;	
		for (CPoseListKinetostatic::iterator itor=poseList.begin(); itor != poseList.end(); ++itor, ++id)
		{
			// compute the properties
			CPoseDynProperty* pPoseDynProperty=dynamic_cast<CPoseDynProperty *>((*iProp)->Property); 

			if  (pPoseDynProperty!=0)
			//(*iProp)->Property->computeProperty((*itor)->r, (*itor)->R, values);
				pPoseDynProperty->computeProperty((*itor)->r, (*itor)->R, (*itor)->v,(*itor)->omega, (*itor)->a,(*itor)->alpha, (*itor)->f,(*itor)->tau, values);
			else
				(*iProp)->Property->computeProperty((*itor)->r, (*itor)->R, values);
				
			// and store the values in the result table
			results.block(id,offset,1,values.rows()) = values.transpose();
		}
		// the property was computed for all poses; stop the time measurement
		int calculationTime = (int)(clock()-start);

		// compute statistic values of the columns of the table: min, max, mean, std-variation 
		for (int i=offset; i<offset+values.rows(); i++)
		{
			CProperty prop;
			prop.header = header[i];
			prop.min = results.col(i).minCoeff();
			prop.max = results.col(i).maxCoeff();
			prop.mean = results.col(i).mean();
			prop.std = stddev(results.col(i));	
			prop.calculationTime = calculationTime;	// careful, we put the same computation time to all properties of the same block
			meta.push_back(prop);
		}	

		// increase the offset to iterate through all columns
		offset+=values.rows();
	}
	return true;
}


/*! save the computed data to a CSV file with name filename. The name
 *  found in the field header is used for the first line of the table.
 *  This function is typically used after calculate() to store the
 *  computed data in a file.
 *  \param filename [in] The name of the csv file to be written
 *  \param seperator [in] by default we put a colon to seperate the 
 *  	values but  we can customize the behaviour by provding another 
 *  	character to be placed between columns
 *  \return true, if successful
 */
bool CPosePropertyEvaluation::saveCsv(const string& filename, char seperator)
{
	ofstream file(filename);
	if (!file.good()) {
		cout << "Save calculation results of PoseEvaluator: ERROR on saving the .csv-file" << endl;
		return false;
	}
	
	stringstream ss; 
	ss << seperator;// << " ";
	string sep = ss.str();
	
	// write the header
	for (unsigned int i=0; i<header.size(); i++) {
		file << header[i];
        if ( i < ( header.size() - 1 ) ) {
            file << sep;
        }
    }
	file << endl;

	// write the data matrix; iterate through the rows and cols
	for (int id=0; id<results.rows(); id++)
	{
		for (int col=0; col<results.cols(); col++) {
			file << results(id,col);
            if ( col < ( results.cols() - 1 ) ) {
                file << sep;
            }
        }
		file << endl;
	}
	return true;
}

/*! save the computed data to a HTML file with name filename. The name
 *  found in the field header is used for the first line of the table 
 *  to be generated inside the HTML file. In the last line fo the table
 *  the statistics data on min, max, etc. are stored.
 *  \param filename [in] The name of the csv file to be written
 *  \return true, if successful
 */
bool CPosePropertyEvaluation::saveHtml(const string& filename)
{
	ofstream file(filename);
	if (!file.good()) {
		cout << "Save calculation results of PoseEvaluator: ERROR on saving the .html-file" << endl;
		return false;
	}

	// write the html header
	file << "<html>\n";
    file << "<body>\n";
	// write the table header
	file << "\t<table>\n";

    // write the column header of the main table
    file << "\t\t<thead>\n";
    file << "\t\t\t<tr>\n";
	for (unsigned int i=0; i<header.size(); i++)
		file << "\t\t\t\t<th>" << header[i] << "</th>\n";
	// end the column head of the html table
	file << "\t\t\t</tr>\n";
    file << "\t\t</thead>\n";

    // Start the table body
	file << "\t\t<tbody>\n";

	// write the data matrix; iterate through the rows and cols
	for (int id=0; id<results.rows(); id++)
	{
		file << "\t\t\t<tr>\n";
		for (int col=0; col<results.cols(); col++)
		{
			if (-1.0e-05 < results(id,col) && results(id,col) < 1.0e-05 && bCheckZeroThreshold)
			{
				file << "\t\t\t\t<td>" << "0" << "</td>\n";
			}
			else
			{
				file << "\t\t\t\t<td>" << results(id,col) << "</td>\n";
			}
		}
		file << "\t\t\t</tr>\n";
	}
    // Close the table body
    file << "\t\t</tbody>\n";

    // Start the table footer
    file << "\t\t<tfoot>\n";
	// add the statistics at the bottom of the table
	file << "\t\t\t<tr>\n";
	for (auto itor = meta.begin(); itor != meta.end(); ++itor)
		file << "\t\t\t\t<th>" 
			<< itor->min << "<br/>"
			<< itor->max << "<br/>"
			<< itor->mean << "<br/>"
			<< itor->std << "<br/>"
			<< itor->calculationTime << "</th>\n";
	file << "\t\t\t</tr>\n";
	
	// write the table footer
	file << "\t\t\t<tr>\n";
	// write the column header also at the base of the main table
	for (unsigned int i=0; i<header.size(); i++)
		file << "\t\t\t\t<th>" << header[i] << "</th>\n";
	// write the footer of the table and the html document
	file << "\t\t\t</tr>\n";
    file << "\t\t</tfoot>\n";
    file << "\t</table>\n";
	file << "</body>\n";
    file << "</html>\n";

	return true;
}

//! save only the statistics of the computed data to a CSV file
bool CPosePropertyEvaluation::saveStatsCsv(const string& filename, char seperator)
{
	return false;
}

/*! save only the statistics of the computed data to a html file
 *  the function generated a html table with the precomputed statistics
 *  the written tables has one line for each property computed, where
 *  the columns in the table represent the statistic figures
 *  \param [in] filename of the HTML file to be generated
 *  \return true, if successful
 */
bool CPosePropertyEvaluation::saveStatsHtml(const string& filename)
{
	ofstream file(filename);
	if (!file.good()) {
		cout << "Save calculation results of PoseEvaluator: ERROR on saving the stats.csv-file" << endl;
		return false;
	}

	// write the html header
	file << "<html>\n";
    file << "<body>\n";
	// write the table header
	file << "\t<table>\n";


	// write the header for the statistics table
	file << "\t\t<thead>\n"
         << "\t\t\t<tr>\n"
         << "\t\t\t\t<th>Column</th>\n" 
         << "\t\t\t\t<th>Property Name</th>\n"
         << "\t\t\t\t<th>min</th>\n"
         << "\t\t\t\t<th>max</th>\n"
         << "\t\t\t\t<th>mean</th>\n"
         << "\t\t\t\t<th>std deviation</th>\n"
         << "\t\t\t\t<th>computation time</th>\n"
         << "\t\t\t</tr>\n"
         << "\t\t</thead>\n";

    file << "\t\t<tbody>\n";
	int i=0;
	for (auto itor = meta.begin(); itor != meta.end(); ++itor)
	{
		file << "\t\t\t<tr>\n"
			<< "\t\t\t\t<td>" << i++ << "</td>\n"
			<< "\t\t\t\t<td>" << (*itor).header << "</td>\n"
			<< "\t\t\t\t<td>" << (*itor).min << "</td>\n"
			<< "\t\t\t\t<td>" << (*itor).max << "</td>\n"
			<< "\t\t\t\t<td>" << (*itor).mean << "</td>\n"
			<< "\t\t\t\t<td>" << (*itor).std << "</td>\n"
			<< "\t\t\t\t<td>" << (*itor).calculationTime << "</td>\n"
			<< "\t\t\t</tr>\n";
	}

    file << "\t\t</tbody>\n";
	
	// write the footer of the table and the html document
	file << "\t</table>\n";
	file << "</body>\n";
    file << "</html>\n";
	return true;
}

/*!
 *  Save the current configuration of the object into a textfile
 *  \param filename [in] The filename of the configuration to be generated
 *  \return true, if successful.
 */
bool CPosePropertyEvaluation::saveSettings(const string& filename)
{
	// consider testing for suffix
	string tab = "  ";
	ofstream file(filename);
	if (!file.good()) {
		cout << "Save calculation results of PoseEvaluator: ERROR on saving the settings.txt-file" << endl;
		return false;
	}

	// save creation-time
	file << "Savingtime: " << currentDateTime() << endl;

	// save used WireCenter and Wirelib Version
	file << "WireCenter-Version: " << PCRL::majorVersion << "." << PCRL::minorVersion << " " << PCRL::versionName << endl;
	file << "WireLib-Version: " << PCRL::versionString << endl << endl;

	// save currently available PoseProperties
	file << "// All current existing PosePropertyTypes and their respective number:" << endl;
	for (int i = 0; emPoseProperty::Names[i] != 0; i++) {
		file << "// " << tab << i << " = " << emPoseProperty::Names[i] << endl;
	}

	file << endl << "// Start Properties:" << endl;
	for (auto itor = activeProperties.begin(); itor !=  activeProperties.end(); ++itor) {
		file << endl;
		file << "Type " << (*itor)->Type << " " << emPoseProperty::Names[(*itor)->Type] << endl;
		CReflection reflection = (*itor)->Property->Reflector();
		reflection.writeStream(file);
	}
	file << endl;
	return true;
}

bool CPosePropertyEvaluation::loadSettings(const string& filename)
{
	ifstream file(filename);
	char buffer[2048];
	string str;
	for (file >> str; !file.eof(); file >> str) {
		if (str.compare(0,19,"WireCenter-Version:") == 0) {
			file >> str;
			if (str.compare(string(PCRL::majorVersion) + "." + PCRL::minorVersion) != 0) {
				cout << "WireCenter-Version of the loaded file does not match the used WireCenter-Version of this setting. Errors during loading poseevaluator-settings might occur due to different implementation." << endl;
			}
			file >> str;
			if (str.compare(string(PCRL::versionName))) {
				cout << "WireCenter-Version-Name of the loaded file does not match the used WireCenter-Version-Name of this setting. Errors during loading poseevaluator-settings might occur due to different implementation." << endl;
			}
			file.getline(buffer,2048);
			continue;
		}
		if (str.compare(0,16,"WireLib-Version:") == 0) {
			file >> str;
			if (str.compare(string(PCRL::versionString))) {
				cout << "WireLib-Version of the loaded file does not match the used WireCenter-Version of this setting. Errors during loading poseevaluator-settings might occur" << endl;
			}
			file.getline(buffer,2048);
			continue;
		}
		if (str.compare(0,4,"Type") == 0)
		{   // line indicates that data of a poseProperty is following
			// get the type of the Property
			int type;
			file >> type;
			//read and ignore whatever remains of that line
			file.getline(buffer,2048);
			// try to create the property
			CPoseProperty* prop = createPoseProperty((emPoseProperty::Type)type);
			// if successfully created, read in the parameter values from the file
			if (prop) {
				CReflection reflection = prop->Reflector();
				reflection.readStream(file);
				this->attachProperty(new CActivePoseProperty((emPoseProperty::Type)type, prop));
			}

		}
		else
		{	// ignore the line if it is not the start of a new Property
			file.getline(buffer,2048);
			continue;
		}
	}
	return true;
}

/*! the class factory method for pose evaulation. Generate a poseEvaluator object 
 *  from the type given as parameter.
 */
CPoseProperty* CPosePropertyEvaluation::createPoseProperty(emPoseProperty::Type type)
{
	CPoseProperty* Property;
	switch(type) {
	case emPoseProperty::PoseMapper:
		Property = new CEuclidianPoseEvaluator(*pRobot);
		break;
	case emPoseProperty::InverseKinematics_Standard:
		Property = new CIKEvaluator(*pRobot, ((CRobotDocument*)pRobot)->Kinematics);
		break;
	case emPoseProperty::InverseKinematics_Pulley:
		Property = new CIKPulleyEvaluator(*pRobot, ((CRobotDocument*)pRobot)->Kinematics);
		break;
	case emPoseProperty::ForwardKinematicsIt:
		Property = new CFKEvaluatorIt(*pRobot, ((CRobotDocument*)pRobot)->Kinematics);
		break;
	case emPoseProperty::OrientationWorkspaceSize:
		Property = new COrientationWorkspaceEvaluator(*pRobot, ((CRobotDocument*)pRobot)->WSHull);
		break;
	case emPoseProperty::CableForcesMethod:
		Property = new CForceDistributionEvaluator(*pRobot, ((CRobotDocument*)pRobot)->ForceDistribution);
		break;
	case emPoseProperty::CableForcesAllMethods:
		Property = new CForceDistributionAllEvaluator(*pRobot, ((CRobotDocument*)pRobot)->ForceDistribution);
		break;
	case emPoseProperty::Dexterity:
		Property = new CDexterityEvaluator(*pRobot, ((CRobotDocument*)pRobot)->ForceDistribution);
		break;
	case emPoseProperty::BaseFrameForces:
		Property = new CBaseFrameForceEvaluator(*pRobot, ((CRobotDocument*)pRobot)->ForceDistribution);
		break;
	case emPoseProperty::Stiffness:
		Property = new CStiffnessEvaluator(*pRobot, ((CRobotDocument*)pRobot)->Stiffness);
		break;
	case emPoseProperty::PoseEstimation:
		Property = new CPoseEstimateEvaluator(*pRobot, ((CRobotDocument*)pRobot)->Kinematics);
		break;
	case emPoseProperty::CableWear:
		Property = new CCableWearEvaluator(*pRobot, ((CRobotDocument*)pRobot)->Kinematics, ((CRobotDocument*)pRobot)->CableWear, true); //,NULL);
		break;
	case emPoseProperty::WrenchSet:
		Property = new CWrenchSetEvaluator(*pRobot, ((CRobotDocument*)pRobot)->ForceDistribution); 
		break;
	case emPoseProperty::ForwardKinematicsDisturbance:
		Property = new CFKDisturbanceEvaluator(*pRobot, ((CRobotDocument*)pRobot)->Kinematics); 
		break;
	case emPoseProperty::NullEvaluator:
		Property = new CNullEvaluator(*pRobot);
		break;
	case emPoseProperty::CableForcesMethodDynamic:
		Property = new CForceDistributionEvaluatorDynamic(*pRobot, ((CRobotDocument*)pRobot)->ForceDistribution);
		break;



	default: Property = NULL;
	}
	return Property;
}

}  // namespace PCRL
