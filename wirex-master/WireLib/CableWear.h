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

/*! \file CableWear.h
 *
 *	\author   Valentin Schmidt
 *
 *
 *  \dependency
 *
 */

#pragma once

#include <list>
#include <vector>
#include "Algorithm.h"
#include <motionPlanning/Utilities.h>

namespace PCRL {

/*! \class CCableWear 
 * enables the tracking of wear introduced by cable robot movement. wear in the
 * cable is important since breaking of the cable due to fatigue is more important
 * in practice than breaking the cable through violating the maximum load.
 * This is done by saving the number of "Biegewechsel" that occur for discrete 
 * segments and the associated force.
 * The cable is discretized into segments and the class maps the biegewechsel
 * to the segments when moving along a trajectory.
 */
class CCableWear : public CAlgorithm //, CCable
{
	//vector<int> discreteSamplePoints;  //container for Cable discretizations
	//container for Robot Model -> In Cable parameterisation
public:
	explicit CCableWear(CRobotData& robot);
	~CCableWear(void);
	
private:	
	//! Size of discretizations
	double cable_segment_size; 
	//! Two dimensional container for Geometric Bend Positions in m
	vector<vector<double> > v_cableguide_bend_positions;
	//! function to fill vector<int>* discreteSegments with number of bending cycles per segment for absolute rope length changes between absolute_start- /endlength and takes in bend_positions
	bool cablewear_analyzePath(double absolute_startlength, double absolute_endlength, vector<int>* discreteSegments, vector<double> cableguide_bend_positions);
public:
	//! Two dimensional container for CableWear
	vector<vector<int> > v_CableWear;

	//! function to fill vector<int>* discreteSegments with number of bending cycles per segment for absolute rope length changes between absolute_start- /endlength 
	bool cablewear_analyzePath(int cableID, double absolute_startlength, double absolute_endlength, vector<int>* discreteSegments);
	bool cablewear_analyzePath(int cableID, double absolute_startlength, double absolute_endlength);

	bool cablewear_initializeFictionalBendingPositions();
	bool cablewear_setBendingPositions(int cable_id, vector<double> bend_positions);
	vector<double> cablewear_getBendingPositions(int cable_id);

	bool cablewear_resetCableWear();

	bool cablewear_exportCableWear(char *filename); 
	// Get Info from the winch object as to how many "Biegewechsel" are contained 
	// at which distance Absolute Nominal Length
};

} //end namespace PCRL
