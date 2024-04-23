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
 *  \file   : CableWear.cpp
 *
 *  Project   Wirelib
 *
 *  \Author   Valentin Schmidt
 *
 *  \Date     27.08.2013
 *
 *********************************************************************
 */

#include "CableWear.h"
#include "motionPlanning/Utilities.h"

namespace PCRL {

CCableWear::CCableWear(CRobotData& robot) : CAlgorithm(robot) //, CCable(mpu,EA0)
{
	pRobot->getNow(); //this should probably take in the number of wires to intiallize the containers
	cable_segment_size = 0.001;
	v_CableWear.resize(pRobot->getNow());
	cablewear_initializeFictionalBendingPositions();
}

CCableWear::~CCableWear(void)
{
}

bool CCableWear::cablewear_initializeFictionalBendingPositions()
{
	vector<double> bend_positions;
	// bending characteristics defined by half bending positions (in relation to point a) negatively to cable segments?
	bend_positions.push_back(0.02);
	bend_positions.push_back(0);
	bend_positions.push_back(-0.2);
	bend_positions.push_back(-0.23);
	bend_positions.push_back(-0.4);
	v_cableguide_bend_positions.resize(pRobot->getNow(),bend_positions);
	return 1;
}

bool CCableWear::cablewear_setBendingPositions(int cable_id, vector<double> bend_positions)
{
	if ( (0 > cable_id) || (cable_id > pRobot->getNow()) )
		return 0;
	v_cableguide_bend_positions.at(cable_id).clear();
	for(unsigned int i=0; i<bend_positions.size(); i++)
		v_cableguide_bend_positions.at(cable_id).push_back(bend_positions.at(i));

	v_cableguide_bend_positions.at(cable_id) = bend_positions;
	return 1;
}

vector<double> CCableWear::cablewear_getBendingPositions(int cable_id)
{
	vector<double> empty;
	if ( (0 > cable_id) || (cable_id > pRobot->getNow()))
		return empty;
	if(v_cableguide_bend_positions.at(cable_id).empty())
		return empty;

	return v_cableguide_bend_positions.at(cable_id);
}

bool CCableWear::cablewear_analyzePath(double absolute_startlength, double absolute_endlength, vector<int>* discreteSegments, vector<double> cableguide_bend_positions)
{
	if (absolute_startlength == absolute_endlength)
	{
		return 0; // no difference in cable length
	}

	for (unsigned int i=0; i < cableguide_bend_positions.size(); i++)
	{
		unsigned int start_index;
		unsigned int end_index;
		if (absolute_startlength < absolute_endlength)
		{
			start_index = (int) (-cableguide_bend_positions.at(i)/ cable_segment_size + absolute_startlength / cable_segment_size); // depending on parameterization of cableguide_bend_positions, this could be cableguide_bend_positions.at(i) - absolute_startlength
			end_index = (int) (-cableguide_bend_positions.at(i)/ cable_segment_size + absolute_endlength / cable_segment_size);
		}
		else // regardless of the direction of movement for the cable, the index will go from small to large
		{
			end_index = (int) (-cableguide_bend_positions.at(i)/ cable_segment_size + absolute_startlength / cable_segment_size);
			start_index = (int) (-cableguide_bend_positions.at(i)/ cable_segment_size + absolute_endlength / cable_segment_size);
		}
		// test if container is large enough
		if (end_index > (int)discreteSegments->size())
		{
			// enlarged container for additional cable length
			discreteSegments->resize(end_index);
		}
		for (unsigned int j=start_index; j < end_index; j++)
			discreteSegments->at(j)++;
	}
	//adjust bend positions according to absolute startlength
	//adjust bend positions according to absolute endlength MAYBE
	//test which discrete cable points pass through bend positions
	return 1;
}


/*! function to take two cable lengths and the cable container for the cable 
 *  wear profile, and increment the segments with cable force requires 
 *  discreeteSegments to contain doubles?
 *  we make a linear interpolation from pos1 to pos2
 */
bool CCableWear::cablewear_analyzePath(int cableID, double absolute_startlength, double absolute_endlength, vector<int>* discreteSegments)
{
	return cablewear_analyzePath(absolute_startlength,absolute_endlength,discreteSegments,v_cableguide_bend_positions.at(cableID));
}

bool CCableWear::cablewear_analyzePath(int cableID, double absolute_startlength, double absolute_endlength)
{
	return cablewear_analyzePath(absolute_startlength,absolute_endlength,&v_CableWear.at(cableID),v_cableguide_bend_positions.at(cableID));
}

/*! Write the results of the wear analysis into a file.
 *  The values in the file are comma seperated. 
 *  \param filename [in] the name of the file to be generated
 *  \return true, if successful
 */
bool CCableWear::cablewear_exportCableWear(char *filename)
{
	const char* seperator = ", ";
	v_CableWear.size();
	v_CableWear.at(1).size();

	ofstream file(filename);
	unsigned int max_size = 0;
	for (unsigned int i=0; i<v_CableWear.size(); i++)
	{
		file << "l[" << i << "]" << seperator;
		unsigned int size = v_CableWear.at(i).size();
		max_size = max(max_size,size);
	}
	file << endl;

	for (unsigned int j=0; j<max_size; j++)
	{
		for (unsigned int i=0; i<v_CableWear.size(); i++)
		{
			if (v_CableWear.at(i).size() > j)
				file << v_CableWear.at(i).at(j) << seperator;
			else
				file << "0" <<seperator;
		}
		file << endl;
	}
	file.close();

	return true;
}

bool CCableWear::cablewear_resetCableWear()
{
	for (unsigned int i=0; i < v_CableWear.size(); i++)
		v_CableWear.at(i).resize(1);

	return 1;
}

} //end namespace PCRL