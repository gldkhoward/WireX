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

/*! \file PosePropertyEvaluation.h
 *
 *	\author   Andreas Pott
 *
 *  \dependency
 *		Eigen3		for IR³ algebra
 *
 *  \par General Information
 *  This class manages the evaluation of a list of poses and analyzes 
 *  the results. The conception of this class is somewhat similar to
 *  CWorkspaceAlgorithm. However, the main objective of PoseEvaluator
 *  is to compute pose properties or performance indicies for a given
 *  list of poses rather than finding poses inside the workspace. 
 *  Therefore, the main output (or data model) of this class is a table
 *  with the pose and its computed properties.
 *
 *  This class is a refactoring of a feature (Trajectory Analysis) 
 *  origionally implemented in WireCenter. However, this class largely
 *  extends the usage by refining the implementation
 *  - using an improved interface of pose property allows for much 
 *	  easier extension, espeically to add new properties as independent
 *    modules.
 *  - trajectories are considered as a special case of (also) unsorted 
 *    pose lists that can be generated from other soruces such as grid 
 *    generators or through conversion of workspace data.
 *  - seperate of concerns: configuration (through addPoseProperty), 
 *    from computation  (calcuate) and communication (save). Therefore, 
 *    we can alter each module without modification of the others (e.g. 
 *    add new file format) or add a new pose property. 
 *  - The new implementation allows to easily compute statistical data
 *    for the columns of the table (such as min, max, average, sum, etc.)
 *  - the OOP design allows for a flexible configuration and usage. 
 *    e.g. introspection can be used in the GUI.
 *  - finally, it shall be noted that this kind of pose evaluation classes
 *    are designed to be stateless with respect to the coupled algorithm
 *    object. Calling a poseEvaluator should not alter the configuration
 *    state of the called CAlgorihtm derived object.
 *
 *  \remark Compared to the original WireCenter center code, some new
 *  meta pose properties have to be implemented, e.g. a property for
 *  comparison between standard and pulley code for inverse kinematics.
 *
 *  \remark An issue in the implementation is to deal with generated data
 *  other than doubles. While CPosePropertyEvaluation stored the result data in
 *  a eigen3 matrix, this is not general since properties might include
 *  int, bool, or even string extressions which cannot be mapped to the
 *  the eigen matrix. For now, it remains open how this issue can be solved.
 * 
 *  The class includes some internal mechanism to determine 
 *  computational time. Note that the computational time can only be measured
 *  for all properties within one property object. Computational times
 *  are only meaningful, if the propoerty consumes a significant amount of time.
 *  For the windows runtime libraray this is around 10ms. 
 *
 *  \remark as of 14.09.2015 the underlying PosePropertyEvaluation is stored in
 *  a seperate file from the specific Pose Evaluators.
 *
 *  \par Planned extension
 *  Another extension is to allow configuration of the orientation 
 *  parameterization written into the result table, e.g. write the 
 *  coefficient of the matrix R, quaternion, different angles. etc. The
 *  base class for this behaviour is implemented through 
 *  CEuclidianPoseEvaluator. However, additional internal implementation
 *  is needed to configure the different parameterizations.
 */

#pragma once

#include "Algorithm.h"
#include "EnumTables.h"
#include "PoseList.h"


namespace PCRL {

/*! The helper struct collects meta and statistics data for a column of 
 *  the data matrix, e.g. for interpreting the results and for enriching
 *  the generation of reports. 
 */
struct CProperty {
	string header;		//!< the name to be displayed as tables
	string desc;		//!< more information on the property e.g. to be used in labels
	string unit;		//!< a string representation of the unit used of the data
	double min;			//!< the minimum value in the column
	double max;			//!< the maximum value in the column
	double mean;		//!< the mean values 1/n sum (x_i)
	double std;			//!< standard deviation from the mean value
	int calculationTime;//!< the calculation time for the column; note that the time is the same for all properties in the same evaluator
};

struct CActivePoseProperty {
	emPoseProperty::Type Type;
	CPoseProperty* Property;
	CActivePoseProperty(emPoseProperty::Type Type, CPoseProperty* Property)
		: Type(Type), Property(Property) {}
};

/*! \class CPoseEvaluator
 *  The Pose Evaluator is designed to coordinate a bunch of CPoseProperty
 *  objects to perform performance analysis of the robot either of a given pose
 *  or for a list of poses. The list can be constructed from a property, a grid,
 *  or any other point cloud (more precisely pose cloud) object. 
 *  CPoseEvaluator is design to aggregate and manage the computed data: The evaluation
 *  of each pose in a list generates a possibly large table with result data which is
 *  stored in this class. Typical member function coordinate the generation of this
 *  data by
 *  - configuration of the properties to be computed
 *  - determination of statistics to be taken from the data
 *  - export the data for further processing in other tools such as CSV, Excel, 
 *    Matlab, matplotlib, HTML, XML, SQL, etc.
 *  Note, that more than one function may be implemented for exporting in order
 *  to support different formats.
 * 
 *  \remark It might be good to exchange the list<CPoseProperties*> by a map<string,CPoseProperties*>
 *  in order to add a symbolic name to the properties. Such a name can be used both for runtime
 *  identification (e.g. use in a GUI) but also for generating nice output data
 *  However, the transition from list to map remove the possibility to define a custom order of
 *  the properties. A good implementation should provide both, unique identification through names
 *  and the possibility to sort the entries. 
 *  
 *  \remark It migt be worthwhile to extend the header to meta information on the columns of the 
 *  matrix, e.g. string header, string description, string unit, double min,max,mean
 */
class CPosePropertyEvaluation : public CAlgorithm
{
protected:
	// the data model of this class
	MatrixXd results;					//!< a large table (matrix) containing the computed results
	vector<string> header;				//!< the symbolic names containing the header information of the result table
	list<CProperty> meta;				//!< contains meta information for each column such as describptions
	list<CActivePoseProperty*> activeProperties;
	
public:
	explicit CPosePropertyEvaluation(CRobotData& robot);
	~CPosePropertyEvaluation(void);

	//! add a pose property with is settings object to the internal list
	CPoseProperty* createPoseProperty(emPoseProperty::Type type);
	bool attachProperty(CActivePoseProperty* pAPP);
	bool removeProperty(CActivePoseProperty* pAPP);
	bool removeAllProperties();
	const list<CActivePoseProperty*>& getActiveProperties() { return activeProperties; }

	//! manipulator the list by moving the given property up in the list
	bool moveUp(const CPoseProperty* target);
	bool moveDown(const CPoseProperty* target);
	bool bCheckZeroThreshold;

	//! return the total number of performance indices computed by CPoseEvaluator
	int getPropertyCount();
	//! return the total number of result data sets 
	int getResultCount() 
		{ return results.rows(); }
	//! return a copy of the result matrix
	MatrixXd getResult() const
		{ return results; }
	//! perform the calculation for each pose and each property
	bool calculate(CPoseListKinetostatic& poseList);
	//! save the computed data to a CSV file
	bool saveCsv(const string& filename, char seperator=',');
	//! save the computed data to a html file
	bool saveHtml(const string& filename);
	//! save the computed data to a CSV file
	bool saveStatsCsv(const string& filename, char seperator=',');
	//! save the computed data to a html file
	bool saveStatsHtml(const string& filename);
	//! save the evaluator settings from a file
	bool saveSettings(const string& filename);
	//! load the evaluator settings from a file
	bool loadSettings(const string& filename);
};

} // namespace PCRL
