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

/*! \file PoseEvaluator.h
 *
 *	\author   Andreas Pott
 *
 *  \par General Information
 *  This file contains several Classes to evaluate Pose Properties 
 *  using several exisiting algorithms. The classes are managed by
 *  the base class CPosePropertyEvaluation and provide an interface
 *  and some additional evaluation computations to underlying 
 *  algorithms such as those provided by the Kinematics and 
 *  Workspace. 
 *
 *********************************************************************
 */ 

#pragma once

#include <string>
#include <list>
#include <vector>
#include "PoseList.h"
#include "Kinematics.h"
#include "StructureMatrix.h"
#include "Workspace.h"
#include <motionPlanning/NcInterpolator.h>
#include "CableWear.h"
#include "EnumTables.h"
#include "RobotDocument.h"
#include "WireLib.h"
#include <motionPlanning/Utilities.h>

namespace PCRL {

/*! \class CEuclidianPoseEvaluator
 *	This class just rewrites the given pose (r,R) in the desired format for   
 *  report generation in PoseEvaluator.
 *  \todo In the first draft we just generate the columns x,y,z, R11,.., R33. 
 *  However, it is straight forward to introduce configuration flags to reduce 
 *  the pose to pure position, pure orientation, resolve position to 
 *  cyclindrical or spherical coordiante, decompose the orientation matrix in
 *  whatever representation needed for the plot. We can also make the class 
 *  sensitve to the motion pattern found in the CRobotData object.
 *  \todo We might introduce a distinction of cases to store the orientation 
 *  parameterization selected by a state variable of the evaluator class.
 *  however, the first implementation has hard wired coefficients of R
 */
class CEuclidianPoseEvaluator : public CPoseProperty
{
protected:
	emOrientationParameter::Type parameterization;
public:
	explicit CEuclidianPoseEvaluator(CRobotData& robot);
	//! get the number of properties computed
	int getPropertyCount();
	//! return the names of the properties to be used as headers in tables by appending the names to the vector
	bool getPropertyNames(std::vector<string>& names);
	//! declare the bindings 
	virtual void bind();
	//! do the actual computation of the properties for the given pose (r,R)
	bool computeProperty(const Vector3d& r, const Matrix3d& R, MatrixXd& values);
};

//! \class CIKEvaluator
//! This evaluator just uses the standard kinematic function to compute the cable length for the 
//! respective pose and exposes the m cable length. 
class CIKEvaluator : public CPoseProperty
{
	CKinematics* pKin;
public:
	CIKEvaluator(CRobotData& robot, CKinematics& Kin) : CPoseProperty(robot), pKin(&Kin) {}

	//! get the number of properties computed
	int getPropertyCount() { return pRobot->getNow(); }
	//! return the names of the properties to be used as headers in tables by appending the names to the vector
	bool getPropertyNames(std::vector<string>& names) 
	{	
		for (int i=0; i<pRobot->getNow(); i++)
		{
			stringstream ss;
			ss << "l" << i;
			names.push_back(ss.str());
		}
		return true;
	}
	//! do the actual computation of the properties for the given pose (r,R)
	bool computeProperty(const Vector3d& r, const Matrix3d& R, MatrixXd& values)
	{
		values.resize(getPropertyCount(),1);
		return pKin->doInverseKinematics(r,R,values);
	}
};

/*! CIKPulleyEvaluator maps the computation of the advanced pulley model to 
 *  the data model of the pose evaulator interface.
 */
class CIKPulleyEvaluator : public CPoseProperty
{
	CKinematics* pKin;		//!< a pointer to the kinematics objects used for the evaluation
public:
	CIKPulleyEvaluator(CRobotData& robot, CKinematics& Kin) : CPoseProperty(robot), pKin(&Kin) {}
	//! get the number of properties computed
	int getPropertyCount() { return pRobot->getNow()*3; }
	//! return the names of the properties to be used as headers in tables by appending the names to the vector
	bool getPropertyNames(std::vector<string>& names) 
	{
		// we use an array with the names of the columns to iterate through the names
		static const char* titles[] = {"lp","beta","gamma",0};
		for (int i=0; i<pRobot->getNow(); i++)
		{
			for (int j=0; titles[j]!=0; j++)
			{
				stringstream ss;			
				ss << titles[j] << i;
				names.push_back(ss.str());
			}
		}
		return true;
	}
	//! do the actual computation of the properties for the given pose (r,R)
	bool computeProperty(const Vector3d& r, const Matrix3d& R, MatrixXd& values)
	{
		// resize the value type
		values.resize(getPropertyCount(),1);
		MatrixXd beta(pRobot->getNow(),1);
		MatrixXd gamma(pRobot->getNow(),1);
		MatrixXd l(pRobot->getNow(),1);
		// caluclate inverse kinematics with simple method
		if (pKin->doInverseKinematicsPulleyEx(r,R,l.data(),0,beta.data(),gamma.data()))
		{
			for (int i=0; i<pRobot->getNow(); i++)
			{
				values(i*3)   = l(i);
				values(i*3+1) = beta(i);
				values(i*3+2) = gamma(i);
			}
			return true;
		}
		else
			return false;
	}
};

/*! \class CFKEvaluator
 *  This evaluator is a base class (abstract) 
 *  it uses a chosen inverse-kinematic function to compute the cable lengths 
 *  for the respective pose, adds errors to these lengths and recalculates 
 *  a pose with the chosen forward-kinematic function (implemented in derived 
 *  classes).  Stored are the calculated pose-properties, the length of vector 
 *  delta_p (between orginal and calculated pose), as well as the needed 
 *  amount of iterations and the exit-code of the forward-kinematic function
 */
class CFKEvaluator : public CPoseProperty
{
protected:
	CKinematics* pKin; //!< a pointer to the kinematics objects used for the evaluation
	bool showWireError; //!< boolean indicating wether the added wirelengtherrors shall be shown
	emWireErrors::Type wireErrorType;
	double errorrange;
	emInverseModel::Type inverseSolverType;

public:	
	CFKEvaluator(CRobotData& robot, CKinematics& Kin, emInverseModel::Type inverseSolverType, emWireErrors::Type wireErrorType, double errorrange, bool showWireError) 
		: CPoseProperty(robot), pKin(&Kin), inverseSolverType(inverseSolverType), wireErrorType(wireErrorType), errorrange(errorrange), showWireError(showWireError) {}

	//! declare the bindings 
	virtual void bind()
	{
		CAlgorithm::bind();
		if (!pReflector)
			return;
		pReflector->bind(showWireError,"showWireError","FKEvaluator/@showWireError");
		pReflector->bind((int&)wireErrorType,"wireErrorType","FKEvaluator/@wireErrorType");
		pReflector->bind(errorrange,"errorrange","FKEvaluator/@errorrange");
		pReflector->bind((int&)inverseSolverType,"inverseSolverType","FKEvaluator/@inverseSolverType");
		pReflector->addEnumTable("FKEvaluator/@wireErrorType", emWireErrors::Names);
		pReflector->addEnumTable("FKEvaluator/@inverseSolverType", emInverseModel::Names);
	}

	//! get the number of properties computed
	virtual int getPropertyCount() { return (showWireError)?(14+pRobot->getNow()):14; }

	//! return the names of the properties to be used as headers in tables by appending the names to the vector
	virtual bool getPropertyNames(std::vector<string>& names) 
	{
		const char* titles[] = {
			"x_result","y_result","z_result",
			"R11_result", "R21_result", "R31_result", "R12_result", "R22_result", "R32_result", "R13_result", "R23_result", "R33_result",
			"delta_pn","delta_rot", 
			};
		for (int j=0; j<CFKEvaluator::getPropertyCount()-((showWireError)?pRobot->getNow():0); j++)
		{
			stringstream ss;			
			ss << titles[j];
			names.push_back(ss.str());
		}
		if(showWireError){
			for (int i=0; i<pRobot->getNow(); i++)
			{
				stringstream ss;
				ss << "w" << i << "_err";
				names.push_back(ss.str());
			}		
		}

		return true;
	}

	//! do the actual computation of the properties for the given pose (r,R)
	//! \todo exchange the double* construct by eigen3 data types; one can access also the underlying array .data field of eigen3
	virtual bool computeProperty(const Vector3d& r, const Matrix3d& R, MatrixXd& values)
	{
		MatrixXd l(pRobot->getNow(),1);
		MatrixXd wireErrors(pRobot->getNow(), 1); 
		chooseInverseKinematics(r,R,l);
		generateWireErrors(l, pRobot->getNow(), wireErrors, wireErrorType, errorrange);
		Vector3d rResult(0,0,0);
		Matrix3d RResult(Matrix3d::ZRotationMatrix3d(0));
		chooseForwardKinematics(l, rResult, RResult);
		values.resize(getPropertyCount(),1);
		values.block(0,0,3,1) = rResult;
		values.block(3,0,3,1) = RResult.col(0);
		values.block(6,0,3,1) = RResult.col(1);
		values.block(9,0,3,1) = RResult.col(2);
		values(12) = (rResult-r).norm();
		values(13) = getSO3Distance(RResult,R);
		if(showWireError){
			for(int i = 0; i < pRobot->getNow(); i++){
				values(i+14) = wireErrors(i);
			}
		}
		return true;
	}

protected:
	virtual void chooseForwardKinematics(const MatrixXd& l, Vector3d& r, Matrix3d& R)	= 0;

	void chooseInverseKinematics(const Vector3d& r, const Matrix3d& R, MatrixXd& l, double* lf=0, double* beta=0, double* gamma=0, Vector3d* C=0, Vector3d* U=0)
	{
		switch(inverseSolverType) {
			// Use doInverseKinematics
			case emInverseModel::standard:
				pKin->doInverseKinematics(r, R, l);
				break;
			// Use doInverseKinematicsPulley
			case emInverseModel::pulley:
				pKin->doInverseKinematicsPulley(r, R, l);
				break;
			// Use doInverseKinematicsPulleyEx
			//case IK_PULLEY_EX:
			//	pKin->doInverseKinematicsPulleyEx(r, R, l, lf, beta, gamma, C,U);
			//	break;
			// same as case Inv_Standard
			default:
				pKin->doInverseKinematics(r, R, l);
				break;
		}
	}
};


/*! \class CFKDisturbanceEvaluator
 *  The class is employed to analyze the impact of simplifying assumptions
 *  in the inverse kinematic model. Here, the specific aspects deals with 
 *  a 3T robot with eight cables and four actuators where two cables
 *  are connected to the same actuator. Therefore, two cables are coupled and
 *  the evaluator can be used to reveal the motion disturbance caused by
 *  the induced coupling
 *  Remark: The current implementation is alpha state, quickly coded for the ASME2018 
 *  paper. An improved version shall be derived from CFKEvaluator (which already defines
 *  helper functions).
 *  The current version does not make use of the XSolverType statevariables for 
 *  configuration. 
 */
class CFKDisturbanceEvaluator : public CPoseProperty
{
protected:
	CKinematics* pKin; //!< a pointer to the kinematics objects used for the evaluation
	emWireErrors::Type wireErrorType;
	double errorrange;
	emInverseModel::Type inverseSolverType;

	emForwardModel::Type forwardSolverType;
	int itmax; //!< maximum number of iterations allowed for forwardkinematics
	emPoseEstimate::Type estimateType; //!< chosen poseEstimatorType, see CKinematics
	emForwardAlgorithm::Type algType; //!< chosen algorithmtype, see CKinematics

public:	
	CFKDisturbanceEvaluator(CRobotData& robot, CKinematics& Kin, 
		emForwardModel::Type forwardSolverType_ = emForwardModel::standard, 
		emInverseModel::Type inverseSolverType_ = emInverseModel::standard, 
		int itmax_ = 100, 
		emPoseEstimate::Type estimateType_ = emPoseEstimate::basicSimpleBoundingbox, 
		emForwardAlgorithm::Type algType_ = emForwardAlgorithm::levmar_der, 
		emWireErrors::Type wireErrorType_ = emWireErrors::poseConsistent, 
		double errorrange = 0.0) 
	: CPoseProperty(robot), pKin(&Kin), 
		wireErrorType(wireErrorType), errorrange(errorrange),
		inverseSolverType(inverseSolverType_), forwardSolverType(forwardSolverType_),  
		itmax(itmax_), estimateType(estimateType_), algType(algType_) {}

	//! declare the bindings 
	virtual void bind()
	{
		CAlgorithm::bind();
		if (!pReflector)
			return;
		pReflector->bind((int&)wireErrorType,"wireErrorType","FKDisturbanceEvaluator/@wireErrorType");
		pReflector->bind(errorrange,"errorrange","FKDisturbanceEvaluator/@errorrange");
		pReflector->bind((int&)inverseSolverType,"FKDisturbanceEvaluator","FKEvaluator/@inverseSolverType");
		pReflector->addEnumTable("FKDisturbanceEvaluator/@wireErrorType", emWireErrors::Names);
		pReflector->addEnumTable("FKDisturbanceEvaluator/@inverseSolverType", emInverseModel::Names);
	}

	//! get the number of properties computed
	virtual int getPropertyCount() { return 14; }		// to be updated

	//! return the names of the properties to be used as headers in tables by appending the names to the vector
	virtual bool getPropertyNames(std::vector<string>& names) 
	{
		const char* titles[] = {
			"x_result","y_result","z_result",
			"R11_result", "R21_result", "R31_result", "R12_result", "R22_result", "R32_result", "R13_result", "R23_result", "R33_result",
			"delta_pn","delta_rot", 0
			};
		// output the first 14 columns
		for (int i=0; titles[i]!=0; i++)
		{
			stringstream ss;			
			ss << titles[i];
			names.push_back(ss.str());
		}

		return true;
	}

	//! do the actual computation of the properties for the given pose (r,R)
	virtual bool computeProperty(const Vector3d& r, const Matrix3d& R, MatrixXd& values)
	{
		// the current version only supports exactly eight cables
		if (pRobot->getNow() != 8)
			return false;

		MatrixXd l(pRobot->getNow(), 1);
		
		// compute the standard IK
		pKin->doInverseKinematics(r, R, l);

		// permute the cable length
		// the following assignment shall be transformed into a generalized matrix form to allow use in arbitrary robot
		// configurations but for the time being, we use a hard-coded selection which cables is mapped
		l(1)=l(0);
		l(3)=l(2);
		l(5)=l(4);
		l(7)=l(6);

		Vector3d rResult(0,0,0);
		Matrix3d RResult(Matrix3d::ZRotationMatrix3d(0));

		// compute forward kinematics
		pKin->doForwardKinematics(l, rResult, RResult, itmax, algType, estimateType);

		// map the results to the output vector
		values.resize(getPropertyCount(),1);
		values.block(0,0,3,1) = rResult;
		values.block(3,0,3,1) = RResult.col(0);
		values.block(6,0,3,1) = RResult.col(1);
		values.block(9,0,3,1) = RResult.col(2);
		values(12) = (rResult-r).norm();
		values(13) = getSO3Distance(RResult,R);

		return true;
	}
};


//! \class CFKEvaluatorIt
//! used for calculations done in CFKEvaluator based on iterative forwardKinematics (LM_Standard, LM_Pulleys, DIST_Standard)
class CFKEvaluatorIt : public CFKEvaluator
{
protected:
	emForwardModel::Type forwardSolverType;
	int itmax; //!< maximum number of iterations allowed for forwardkinematics
	emPoseEstimate::Type estimateType; //!< chosen poseEstimatorType, see CKinematics
	emForwardAlgorithm::Type algType; //!< chosen algorithmtype, see CKinematics
	
public:
	CFKEvaluatorIt(CRobotData& robot, CKinematics& Kin, 
		emForwardModel::Type forwardSolverType = emForwardModel::standard, 
		emInverseModel::Type inverseSolverType = emInverseModel::standard, 
		int itmax = 100, 
		emPoseEstimate::Type estimateType = emPoseEstimate::basicSimpleBoundingbox, 
		emForwardAlgorithm::Type algType = emForwardAlgorithm::levmar_der, 
		emWireErrors::Type wireErrorType = emWireErrors::poseConsistent, 
		double errorrange = 0.0, 
		bool showWireError = false) 
		: CFKEvaluator(robot, Kin, inverseSolverType, wireErrorType, errorrange, showWireError), forwardSolverType(forwardSolverType),  itmax(itmax), estimateType(estimateType), algType(algType) {}

	int getPropertyCount() { return CFKEvaluator::getPropertyCount()+2; }

	virtual bool getPropertyNames(std::vector<string>& names) 
	{
		CFKEvaluator::getPropertyNames(names);
		const char* titles[] = {
			"Iterations", "ExitCode"
			};
		for (int j=0; j<getPropertyCount()-CFKEvaluator::getPropertyCount(); j++)
		{
			stringstream ss;			
			ss << titles[j];
			names.push_back(ss.str());
		}
		return true;
	}

	virtual void bind()
	{
		CFKEvaluator::bind();
		if (!pReflector)
			return;
		pReflector->bind((int&)estimateType,"poseEstimateType","FKEvaluator/FKEvaluatorIt/@poseEstimateType");
		pReflector->bind((int&)forwardSolverType,"forwardSolverType","FKEvaluator/FKEvaluatorIt/@forwardSolverType");
		pReflector->bind((int&)algType,"algType","FKEvaluator/FKEvaluatorIt/@algType");
		pReflector->bind(itmax,"maxIterations","FKEvaluator/FKEvaluatorIt/@itmax");
		pReflector->addEnumTable("FKEvaluator/FKEvaluatorIt/@poseEstimateType", emPoseEstimate::Names);
		pReflector->addEnumTable("FKEvaluator/FKEvaluatorIt/@forwardSolverType", emForwardModel::Names);
		pReflector->addEnumTable("FKEvaluator/FKEvaluatorIt/@algType", emForwardAlgorithm::Names);
	}

	bool computeProperty(const Vector3d& r, const Matrix3d& R, MatrixXd& values){
		bool ret = CFKEvaluator::computeProperty(r,R,values);
		values(CFKEvaluator::getPropertyCount()) = pKin->getIterations();
		values(CFKEvaluator::getPropertyCount()+1) = pKin->getTerminationCode();
		return ret;
	}

	void setForwardSolverType(emForwardModel::Type forwardSolverType){this->forwardSolverType = forwardSolverType;} 
	void setInverseSolverType(emInverseModel::Type inverseSolverType){this->inverseSolverType = inverseSolverType;}
	void setMaxIterations(int itmax){this->itmax = itmax;}
	void setPoseEstimateType(emPoseEstimate::Type estimateType){this->estimateType = estimateType;} 
	void setAlgType(emForwardAlgorithm::Type algType){this->algType = algType;}
	void setWireErrorType(emWireErrors::Type wireErrorType){this->wireErrorType = wireErrorType;} 
	void setErrorrange(double errorrange){this->errorrange = errorrange;}
	void setShowWireError(bool showWireError){this->showWireError = showWireError;}


protected:
	void chooseForwardKinematics(const MatrixXd& l, Vector3d& r, Matrix3d& R)
	{
		switch(forwardSolverType) {
			// Use doForwardKinematics
			case emForwardModel::standard:
				pKin->doForwardKinematics(l, r, R, itmax, algType, estimateType);
				break;
			// Use doForwardKinematicsPulley
			case emForwardModel::pulley:
				pKin->doForwardKinematicsPulley(l, r, R, itmax, algType, estimateType);
				break;
			// Use doForwardKinematicsDist
			case emForwardModel::distance:
				pKin->doForwardKinematicsDist(l, r, R, itmax, algType, estimateType);
				break;
			// same as case LM_Standard
			case emForwardModel::standard_abcpreconditioning:
				pKin->doForwardKinematics_preconditionedabc(l, r, R, itmax, algType, estimateType);
				break;
			case emForwardModel::standard_Transform_Geometry:
				pKin->doForwardKinematics_geometrytransform(l, r, R, itmax, algType, estimateType);
				break;
			default:
				pKin->doForwardKinematics(l, r, R, itmax, algType, estimateType);
				break;
		}
	}
};


//! \class CForceDistributionEvaluator
//! This evaluator just uses the standard kinematic function to compute the cable length for the 
//! respective pose and exposes the m cable length. 
class CForceDistributionEvaluator : public CPoseProperty
{
	CForceDistribution* pFD;
	emForceDistributionMethod::Type forceDistributionMethod;
	emEvaluatorCriteria::Type evaluatorCriteria;
public:
	CForceDistributionEvaluator(CRobotData& robot, CForceDistribution& FD, emForceDistributionMethod::Type forceDistributionMethod = emForceDistributionMethod::bruteForce, emEvaluatorCriteria::Type evaluatorCriteria = emEvaluatorCriteria::infinity) 
		: CPoseProperty(robot), pFD(&FD), forceDistributionMethod(forceDistributionMethod), evaluatorCriteria(evaluatorCriteria) {}

	//! get the number of properties computed
	int getPropertyCount() { return pRobot->getNow() + 8; }
	//! return the names of the properties to be used as headers in tables by appending the names to the vector
	bool getPropertyNames(std::vector<string>& names);

	virtual void bind();

	//! do the actual computation of the properties for the given pose (r,R)
	bool computeProperty(const Vector3d& r, const Matrix3d& R, MatrixXd& values);

	void setForceDistributionMethod(emForceDistributionMethod::Type forceDistributionMethod){this->forceDistributionMethod = forceDistributionMethod;}
	void setEvaluatorCriteria(emEvaluatorCriteria::Type evaluatorCriteria){this->evaluatorCriteria = evaluatorCriteria;}
};




//! \class CForceDistributionEvaluatorDynamic
//! x 
//! x
class CForceDistributionEvaluatorDynamic : public CPoseDynProperty
{
	CForceDistribution* pFD;
	emForceDistributionMethod::Type forceDistributionMethod;
	emEvaluatorCriteria::Type evaluatorCriteria;
public:
	CForceDistributionEvaluatorDynamic(CRobotData& robot, CForceDistribution& FD, emForceDistributionMethod::Type forceDistributionMethod = emForceDistributionMethod::dykstra, emEvaluatorCriteria::Type evaluatorCriteria = emEvaluatorCriteria::infinity) 
		: CPoseDynProperty(robot), pFD(&FD), forceDistributionMethod(forceDistributionMethod), evaluatorCriteria(evaluatorCriteria) {}

	//! get the number of properties computed
	int getPropertyCount() { return 7*pRobot->getNow() + 3*pRobot->getDof() + 9; }
	//! return the names of the properties to be used as headers in tables by appending the names to the vector
	bool getPropertyNames(std::vector<string>& names);

	virtual void bind();

	//! do the actual computation of the properties for the given pose (r,R)
	bool computeProperty(const Vector3d& r, const Matrix3d& R, const Vector3d& v,
					  const Vector3d& omega, const Vector3d& a,const Vector3d& alpha,
					  const Vector3d& f, const Vector3d& tau, MatrixXd& values);

	void setForceDistributionMethod(emForceDistributionMethod::Type forceDistributionMethod){this->forceDistributionMethod = forceDistributionMethod;}
	void setEvaluatorCriteria(emEvaluatorCriteria::Type evaluatorCriteria){this->evaluatorCriteria = evaluatorCriteria;}
};







/*! This evaluator computes all force distribution for comparison.
 *  
 */
class CForceDistributionAllEvaluator : public CPoseProperty
{
	CForceDistribution* pFD;
public:
	CForceDistributionAllEvaluator(CRobotData& robot, CForceDistribution& FD) : CPoseProperty(robot), pFD(&FD) {}

	//! get the number of properties computed
	int getPropertyCount() { return pRobot->getNow()*10+6; }
	//! return the names of the properties to be used as headers in tables by appending the names to the vector
	bool getPropertyNames(std::vector<string>& names) 
	{	
		// we use an array with the names of the columns to iterate through the names
		static const char* titles[] = {"f_BF", "f_Dyk", "f_WS", "f_ACF", "f_CFEE", "f_Punc", "f_ACFEE", "f_QP", "f_CF","f_Bary",0};
		for (int j=0; titles[j]!=0; j++)
		{
			for (int i=0; i<pRobot->getNow(); i++)
			{
				stringstream ss;			
				ss << titles[j] << i;
				names.push_back(ss.str());
			}
		}
		names.push_back("number_of_vertices");
		names.push_back("dykstra_iterations");
		names.push_back("QP_iterations");
		names.push_back("QP_converged");
		names.push_back("Bary_nov");
		names.push_back("Bary_area");
		return true;
	}

	/*******************************************************************************************************************/
	/*******************************************************************************************************************/
	// additionally map the follow properties: 
	// * number of vertices of \mathcal F
	// * iterations for Dykstra
	// * different norms of the respective f, distance from center
	/*******************************************************************************************************************/
	/*******************************************************************************************************************/

	//! do the actual computation of the properties for the given pose (r,R)
	bool computeProperty(const Vector3d& r, const Matrix3d& R, MatrixXd& values)
	{
		// compute the structure matrix; it is required for all distribution methods
		if (!pFD->getStructureMatrix(r,R))
			return false;

		// resize the result vector
		values.resize(getPropertyCount(),1);

		// set the applied wrench to zero; clearly, this is somewhat special but we do not have
		// better values in this setting
		MatrixXd w = MatrixXd::Zero(pRobot->getDof(),1);
		pFD->setWrenchVector(w,pFD->f,pFD->tau);
		MatrixXd f_min = MatrixXd::Constant(pRobot->getNow(),1, pRobot->fmin);		// the minimum force vector
		MatrixXd f_max = MatrixXd::Constant(pRobot->getNow(),1, pRobot->fmax);		// the maximum force vector
	
		// do the computation for each method 
		MatrixXd f_BF,f_Dyk, f_WS,f_ACF,f_CFEE,f_Punc, f_ACFEE, f_QP, f_CF,f_All,f_Bary;
		double nov=0;	// number of vertices
		int inov=0;		// number of vertices as int
		double area=0;	// area of solutions space

		if (!pFD->getDistributionBruteForce(w,f_BF))
			return false;

		if (!pFD->getDistributionDykstra(w, f_min, f_max, f_Dyk))
			return false;

		if (!pFD->getDistributionWeightedSum(w, f_WS))
			f_WS = MatrixXd::Constant(pRobot->getNow(),1, -1);

		if (!pFD->getDistributionAdvancedClosedForm(w,f_min,f_max,f_ACF))
			f_ACF = MatrixXd::Constant(pRobot->getNow(),1, -1);

		if (!pFD->getDistributionClosedFormEnergyEfficient(w,f_min, f_max, f_CFEE))
			f_CFEE = MatrixXd::Constant(pRobot->getNow(),1, -1);

		if (!pFD->getDistributionPuncture(w,f_min, f_max, f_Punc))
			f_Punc = MatrixXd::Constant(pRobot->getNow(),1, -1);

		if (!pFD->getDistributionAdvancedClosedFormEnergyEfficient(w,f_min,f_max,f_ACFEE))
			f_ACFEE = MatrixXd::Constant(pRobot->getNow(),1, -1);

		if (!pFD->getDistributionQuadraticProgramming(w,f_min, f_max, f_QP))
			f_QP = MatrixXd::Constant(pRobot->getNow(),1, -1);

		if (!pFD->getDistributionClosedForm(w,f_min, f_max, f_CF))
			f_CF = MatrixXd::Constant(pRobot->getNow(),1, -1);

		if (!pFD->getDistributionAllVertices(w,f_All))
			nov = 0;
		else
			nov = f_All.cols();

		if (!pFD->getDistributionBarycenter(w,f_min,f_max,f_Bary,inov,area))
			false;

		values << f_BF,f_Dyk, f_WS,f_ACF,f_CFEE,f_Punc, f_ACFEE, f_QP, f_CF,f_Bary,
			nov,(double)pFD->getDykstraIterations(),(double)pFD->getQPconverged(),(double)pFD->getQPIterations(), (double)inov , area;

		return true; 
	}
};

//! \class COrientationWorkspaceEvaluator
//! use the workspace algorithm to evaluate the available rotation capacity at the
//! current position.
class COrientationWorkspaceEvaluator : public CPoseProperty
{
	CWorkspaceAlgorithm* pWSA;	//!< reference to the workspace algorith to be used in the computation
public:
	COrientationWorkspaceEvaluator(CRobotData& robot, CWorkspaceAlgorithm& WS) : CPoseProperty(robot), pWSA(&WS) {}

	//! get the number of properties computed
	int getPropertyCount();
	//! return the names of the properties to be used as headers in tables by appending the names to the vector
	bool getPropertyNames(std::vector<string>& names);
	//! do the actual computation of the properties for the given pose (r,R)
	bool computeProperty(const Vector3d& r, const Matrix3d& R, MatrixXd& values);
};

//! \class CBaseFrameForceEvaluator
class CBaseFrameForceEvaluator: public CPoseProperty
{
	CForceDistribution* pFD;
public:
	CBaseFrameForceEvaluator(CRobotData& robot, CForceDistribution& FD) : CPoseProperty(robot), pFD(&FD) {}

	//! get the number of properties computed
	int getPropertyCount() { return pRobot->getNow()*3; }
	//! return the names of the properties to be used as headers in tables by appending the names to the vector
	bool getPropertyNames(std::vector<string>& names) 
	{
		// we use an array with the names of the columns to iterate through the names
		static const char* titles[] = {"fAx","fAy","fAz",0};
		for (int i=0; i<pRobot->getNow(); i++)
		{
			for (int j=0; titles[j]!=0; j++)
			{
				stringstream ss;			
				ss << titles[j] << i;
				names.push_back(ss.str());
			}
		}
		return true;
	}
	//! do the actual computation of the properties for the given pose (r,R)
	bool computeProperty(const Vector3d& r, const Matrix3d& R, MatrixXd& values)
	{
		values.resize(getPropertyCount(),1);
		MatrixXd f = MatrixXd::Zero(pRobot->getNow(),1);
		MatrixXd w = MatrixXd::Zero(pRobot->getDof(),1);
		pFD->setWrenchVector(w,pFD->f,pFD->tau);
		if (!pFD->getStructureMatrix(r,R))
			return false;
		pFD->getDistribution(w, pRobot->fmin, pRobot->fmax, f);
		for (int j=0; j < f.rows(); j++)
		{
			values(3*j)   = -f(j) * pFD->getElement(0,j);
			values(3*j+1) = -f(j) * pFD->getElement(1,j);
			values(3*j+2) = -f(j) * pFD->getElement(2,j);
		}
		return true;
	}
};

//! \class CWrenchSetEvaluator
class CWrenchSetEvaluator: public CPoseProperty
{
	CForceDistribution* pFD;
public:
	CWrenchSetEvaluator(CRobotData& robot, CForceDistribution& FD) : CPoseProperty(robot), pFD(&FD) {}

	//! get the number of properties computed
	int getPropertyCount() { return 12; }
	//! return the names of the properties to be used as headers in tables by appending the names to the vector
	bool getPropertyNames(std::vector<string>& names) 
	{
		names.push_back("f_x_min");
		names.push_back("f_y_min");
		names.push_back("f_z_min");
		names.push_back("M_x_min");
		names.push_back("M_y_min");
		names.push_back("M_z_min");
		names.push_back("f_x_max");
		names.push_back("f_y_max");
		names.push_back("f_z_max");
		names.push_back("M_x_max");
		names.push_back("M_y_max");
		names.push_back("M_z_max");
		return true;
	}
	//! do the actual computation of the properties for the given pose (r,R)
	bool computeProperty(const Vector3d& r, const Matrix3d& R, MatrixXd& values)
	{
		values.resize(getPropertyCount(),1);
		Vector3d fmin,fmax,taumin,taumax;
		if (pFD->getMaximumWrenchForPose(r,R,fmin,taumin,fmax,taumax))
			values << fmin,taumin,fmax,taumax;
		else
			values.setZero();

		return true;
	}
};

/*! This class does nothing but returning zero. Results do not depend 
*   on the actual pose.
 *  This behavior is useful in some GUI scenarios as well as to
 *  insert some kind of seperator column into a table.
 */
class CNullEvaluator : public CPoseProperty
{
public:
	CNullEvaluator(CRobotData& robot) : CPoseProperty(robot) {}
	//! get the number of properties computed
	int getPropertyCount() { return 1; }
	//! return the names of the properties to be used as headers in tables by appending the names to the vector
	bool getPropertyNames(std::vector<string>& names) 
	{
		names.push_back("zero");
		return true;
	}
	//! do nothing but returning a vector with a single zero element
	bool computeProperty(const Vector3d& r, const Matrix3d& R, MatrixXd& values)
	{
		values.resize(getPropertyCount(),1);
		values.setZero();
		return true;
	}
};

#ifdef ASP_USE_NEW_DYNAMICS_EVALUATOR
//! \class CDynamicsEvaluator
class CDynamicsEvaluator: public CPoseProperty
{
	CForceDistribution* pFD;
	CNcInterpolator* pIpo;
public:
	CDynamicsEvaluator(CRobotData& robot, CForceDistribution& FD, CNcInterpolator& IPO) : CPoseProperty(robot), pFD(&FD), pIpo(%IPO) {}

	//! get the number of properties computed
	int getPropertyCount() 
	{ 
		if (pIpo->iInterpolationMethod == 1) // simple interpolator with constant velocity
			return 12 + pRobot->getNow() * 7;
		else if (pIpo->iInterpolationMethod == 2)
			return 18 + pRobot->getNow() * 9;
		else
			return 0;
	}

	//! return the names of the properties to be used as headers in tables by appending the names to the vector
	bool getPropertyNames(std::vector<string>& names) 
	{
		if (pIpo->iInterpolationMethod == 1) // simple interpolator with constant velocity
		{
			names.push_back("v_x");
			names.push_back("v_y"); 
			names.push_back("v_z"); 
			names.push_back("omega_x"); 
			names.push_back("omega_y"); 
			names.push_back("omega_z");
			names.push_back("w_x"); 
			names.push_back("w_y"); 
			names.push_back("w_z"); 
			names.push_back("w_a"); 
			names.push_back("w_b"); 
			names.push_back("w_c");
			for (int i=0; i<pRobot->getNow(); i++)
			{
				stringstream ss;
				ss << "f" << i+1;
				names.push_back(ss.str());
			}
			for (int j=0; j < pRD->l.cols(); j++)
				file<< seperator << "fA" << j << "x"
					<< seperator << "fA" << j << "y"
					<< seperator << "fA" << j << "z";
			for (int j=0; j < pRD->l.cols(); j++)
				file << seperator << "v_wires" << j+1;
			for (int j=0; j < pRD->l.cols(); j++)
				file << seperator << "M_motor" << j+1;
			for (int j=0; j < pRD->l.cols(); j++)
				file << seperator << "omega_Motor" << j+1;
		}
		else if (pIpo->iInterpolationMethod == 2)
		{
			names.push_back("v_x");
			names.push_back("v_y"); 
			names.push_back("v_z" );
			names.push_back("omega_x"); 
			names.push_back("omega_y"); 
			names.push_back("omega_z");
			names.push_back("a_x"); 
			names.push_back("a_y"); 
			names.push_back("a_z"); 
			names.push_back("alpha_x"); 
			names.push_back("alpha_y"); 
			names.push_back("alpha_z");
			names.push_back("w_x"); 
			names.push_back("w_y"); 
			names.push_back("w_z"); 
			names.push_back("w_a"); 
			names.push_back("w_b"); 
			names.push_back("w_c");
			for (int j=0; j < pRD->l.cols(); j++)
				file << seperator << "f" << j+1;
			for (int j=0; j < pRD->l.cols(); j++)
				file<< seperator << "fA" << j << "x"
					<< seperator << "fA" << j << "y"
					<< seperator << "fA" << j << "z";
			for (int j=0; j < pRD->l.cols(); j++)
				file << seperator << "v_wires" << j+1;
			for (int j=0; j < pRD->l.cols(); j++)
				file << seperator << "a_wires" << j+1;
			for (int j=0; j < pRD->l.cols(); j++)
				file << seperator << "M_motor" << j+1;
			for (int j=0; j < pRD->l.cols(); j++)
				file << seperator << "omega_Motor" << j+1;
			for (int j=0; j < pRD->l.cols(); j++)
				file << seperator << "alpha_Motor" << j+1;
		}
		return true;
	}
	//! do the actual computation of the properties for the given pose (r,R)
	bool computeProperty(const Vector3d& r, const Matrix3d& R, MatrixXd& values)
	{
		return false;
/*		values.resize(getPropertyCount(),1);
		MatrixXd f = MatrixXd::Zero(pRobot->getNow(),1);
		MatrixXd w = MatrixXd::Zero(pRobot->getDof(),1);
		pFD->setWrenchVector(w,pFD->f,pFD->tau);
		if (!pFD->getStructureMatrix(r,R))
			return false;
		pFD->getDistribution(w, pRobot->fmin, pRobot->fmax, f);
		for (int j=0; j < f.rows(); j++)
		{
			values(3*j)   = -f(j) * pFD->getElement(0,j);
			values(3*j+1) = -f(j) * pFD->getElement(1,j);
			values(3*j+2) = -f(j) * pFD->getElement(2,j);
		}
		return true;*/
	}
};
#endif


//! \class CCableWearEvaluator
//! For now this evaluator uses the CCableWear-Class to calculate the amount of bendings due to the cablelength change 
//! between two poses. Depending on the mode it stores the number of segments affected by bending as well as the total
//! number of bendings for all segments of a wire either seperatly for each two following poses or as a sum of all
//! previous computed data.
class CCableWearEvaluator : public CPoseProperty
{
	vector<double> prevWireLengths; //!< vector to access the wirelengths of the previous pose
	CKinematics* pKin;	//!< a pointer to the kinematics objects used for the evaluation
	CCableWear* pCWear; //!< a pointer to the cablewear object used for computation
	bool trajectory;

public:
	CCableWearEvaluator(CRobotData& robot, CKinematics& Kin, PCRL::CCableWear& cwear, bool trajectory) //, vector<vector<double>> bend_positions = vector<vector<double>>()) 
		: CPoseProperty(robot), pKin(&Kin), pCWear(&cwear), trajectory(trajectory) 
	{
		pCWear->cablewear_resetCableWear(); // needed?
		//if (!bend_positions.empty() && bend_positions.size() == pRobot->getNow())
		//{
		//	//! \todo removed a double "<" comarison [ i < i <pRobot()... ] which seems to be a typo; 
		//	//! in case that this was an error, please revise the implementation such that it more expressive (asp, 21.05.2015)
		//	for (int i = 0; i < pRobot->getNow(); i++)		
		//	{
		//		pCWear->cablewear_setBendingPositions(i, bend_positions.at(i));
		//	}
		//}
	}

	//! get the number of properties computed
	int getPropertyCount() { return pRobot->getNow()*4; }

	//! return the names of the properties to be used as headers in tables by appending the names to the vector
	bool getPropertyNames(std::vector<string>& names) 
	{
		for (int i=0; i<pRobot->getNow(); i++)
		{
			names.push_back("Segments_w" + to_string((long long) i+1));
			names.push_back("totalBW_w" + to_string((long long) i+1));
			names.push_back("Minlength_w"+ to_string((long long) i+1));
			names.push_back("Maxlength_w" + to_string((long long) i+1));
		}
		return true;
	}

	//! do the actual computation of the properties for the given pose (r,R)
	bool computeProperty(const Vector3d& r, const Matrix3d& R, MatrixXd& values)
	{
		values.resize(getPropertyCount(),1);
		MatrixXd l(pRobot->getNow(),1);
		pKin->doInverseKinematics(r,R,l);

		if (!prevWireLengths.empty())
		{	// if not first pose, then calculate Biegewechsel
			if (!trajectory)
				pCWear->cablewear_resetCableWear(); //clear CableWearData if only data between the two poses is needed#

			vector<vector<double>> cableWear;			
			cableWear.resize(pRobot->getNow());
			for (int i=0; i <pRobot->getNow(); i++)
			{
				pCWear->cablewear_analyzePath(i,prevWireLengths[i], l(i));
				cableWear[i] = pCWear->cablewear_getBendingPositions(i);
				
				int numberOfSeg = 0;			// number of segments with Biegewechsel > 0
				double totalBiegewechsel = 0;	// total number of Biegewechsel in all segments
				
				for (unsigned int j = 0; j < pCWear->v_CableWear[i].size(); j++)
				{
					if (pCWear->v_CableWear[i][j] > 0)
					{
						numberOfSeg++;
						totalBiegewechsel += pCWear->v_CableWear[i][j];
					}
				}
				values(i*4) = numberOfSeg;
				values(i*4+1) = totalBiegewechsel;

				if (prevWireLengths[i] < l(i))
				{	// show minimum wirelength first, regardless of moving direction
					values(i*4+2) = prevWireLengths[i];
					values(i*4+3) = l(i);
				} else {
					values(i*4+2) = l(i);
					values(i*4+3) = prevWireLengths[i];
				}
			}
		} else {	// if first pose, then initialize vector for previous wirelengths
			prevWireLengths.resize(pRobot->getNow());
			for (int i=0; i < getPropertyCount(); i++)
			{
				values(i) = 0;
			}
		}		// save wirelengths for next computation
		for (int i = 0; i < pRobot->getNow(); i++)
		{
			prevWireLengths[i] = l(i);
		}
		return true;
	}
};

/*! This evaluator computes the structure matrix and lists 
    dexterity related properties of the strcuture matrix such as
	rank, column and row norm as well as the smallest and largest 
	singular values of the matrix A^T.
 */
class CDexterityEvaluator : public CPoseProperty
{
	CForceDistribution *pForceDistribution;
public:
	CDexterityEvaluator(CRobotData& robot, CForceDistribution& ForceDistribution) : CPoseProperty(robot), pForceDistribution(&ForceDistribution) {}

	//! get the number of properties computed
	int getPropertyCount() { return 5; }
	//! return the names of the properties to be used as headers in tables by appending the names to the vector
	bool getPropertyNames(std::vector<string>& names) 
	{
		names.push_back("rank");
		names.push_back("column_norm");
		names.push_back("row_norm");
		names.push_back("max_singularvalue");
		names.push_back("min_singularvalue");
		return true;
	}
	//! do the actual computation of the properties for the given pose (r,R)
	bool computeProperty(const Vector3d& r, const Matrix3d& R, MatrixXd& values)
	{
		values.resize(getPropertyCount(),1);
		// get the structure matrix
		if (!pForceDistribution->getStructureMatrix(r,R))
			return false;
		// compute the eingular values 
		MatrixXd sv;
		pForceDistribution->getSingularValues(sv);
		// map the relevant properties in the order defined through the names 
		// above into the values vector
		values << 
			pForceDistribution->getRank(),
			pForceDistribution->getMaxColumnNorm(),
			pForceDistribution->getMaxRowNorm(),
			sv(0),
			sv(pRobot->getDof()-1);
		return true;
	}
};

/*! This properties computes the stiffness matrix and returns the singular values of the
 *  stiffness matrix for the given pose.
 */
class CStiffnessEvaluator : public CPoseProperty
{
	CStiffness *pStiffness;
public:
	CStiffnessEvaluator(CRobotData& robot, CStiffness& Stiffness) : CPoseProperty(robot), pStiffness(&Stiffness) {}

	//! get the number of properties computed
	int getPropertyCount() { return pRobot->getDof(); }
	//! return the names of the properties to be used as headers in tables by appending the names to the vector
	bool getPropertyNames(std::vector<string>& names) 
	{	
		for (int i=0; i<pRobot->getDof(); i++)
		{
			stringstream ss;
			ss << "K_sigma" << i;
			names.push_back(ss.str());
		}
		return true;
	}
	//! do the actual computation of the properties for the given pose (r,R)
	bool computeProperty(const Vector3d& r, const Matrix3d& R, MatrixXd& values)
	{
		values.resize(getPropertyCount(),1);
		if (!pStiffness->StiffnessMatrix(r,R))
			return false;
		values = pStiffness->getStiffnessMatrix().jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).singularValues();
		return true;
	}
};

class CPoseEstimateEvaluator : public CPoseProperty
{
protected:
	emInverseModel::Type inverseSolverType;
	emPoseEstimate::Type poseEstimateType;
	emWireErrors::Type wireErrorType;
	double errorrange;
	bool showWireErrors;
	bool showBounds;
	CKinematics* pKin; 

public:
	CPoseEstimateEvaluator(CRobotData& robot, CKinematics& Kin, emPoseEstimate::Type poseEstimateType = emPoseEstimate::basicSimpleBoundingbox, emInverseModel::Type inverseSolverType = emInverseModel::standard, emWireErrors::Type wireErrorType =emWireErrors::poseConsistent, double errorrange = 0.0, bool showWireErrors = false, bool showBounds = false)
		: CPoseProperty(robot), pKin(&Kin), poseEstimateType(poseEstimateType), inverseSolverType(inverseSolverType), wireErrorType(wireErrorType), errorrange(errorrange), showWireErrors(showWireErrors), showBounds(showBounds){}

	//! get the number of properties computed
	int getPropertyCount() { 
		int propertyCount = 8;
		if(showWireErrors) propertyCount += 8;
		if(showBounds) propertyCount += 18;
		return propertyCount;
	}

	virtual void bind()
	{
		CAlgorithm::bind();
		if (!pReflector)
			return;
		pReflector->bind((int&)inverseSolverType,"inverseSolverType","PoseEstimateEvaluator/@inverseSolverType");
		pReflector->bind((int&)poseEstimateType,"poseEstimateType","PoseEstimateEvaluator/@poseEstimateType");
		pReflector->bind((int&)wireErrorType,"wireErrorType","PoseEstimateEvaluator/@wireErrorType");
		pReflector->bind(errorrange,"errorrange","PoseEstimateEvaluator/@errorrange");		
		pReflector->bind(showWireErrors,"showWireErrors","PoseEstimateEvaluator/@showWireErrors");
		pReflector->bind(showBounds,"showBounds","PoseEstimateEvaluator/@showBounds");
		pReflector->addEnumTable("PoseEstimateEvaluator/@wireErrorType", emWireErrors::Names);
		pReflector->addEnumTable("PoseEstimateEvaluator/@inverseSolverType", emInverseModel::Names);
		pReflector->addEnumTable("PoseEstimateEvaluator/@poseEstimateType", emPoseEstimate::Names);
	}

	//! return the names of the properties to be used as headers in tables by appending the names to the vector
	bool getPropertyNames(std::vector<string>& names) 
	{	
		static const char* titles[] = {"x","xlb","xub","xooB",	// value in the mid between upper and lower bound | (l)ower (b)ound | (u)pper (b)ound | indicator wether real value is (o)ut (o)f (b)ounds
										"y","ylb","yub","yooB",
										"z","zlb","zub","zooB",
										"a","alb","aub","aooB",
										"b","blb","bub","booB",
										"c","clb","cub","cooB",
										"delta_pn", "delta_rot",
										"w1_err", "w2_err", "w3_err", "w4_err", "w5_err", "w6_err", "w7_err", "w8_err",
										0};
		if(showBounds){
			for (int j=0; j<26; j++){
				stringstream ss;			
				ss << titles[j];
				names.push_back(ss.str());
			}
		}else{
			for (int j=0; j<6; j++){
				stringstream ss;			
				ss << titles[j*4];
				names.push_back(ss.str());
			}
			names.push_back("delta_pn");
			names.push_back("delta_rot");			
		}
		if(showWireErrors){
			for(int j = 26; titles[j]!=0; j++){
				stringstream ss;			
				ss << titles[j];
				names.push_back(ss.str());
			}
		}
		return true;
	}
	//! do the actual computation of the properties for the given pose (r,R)
	bool computeProperty(const Vector3d& r, const Matrix3d& R, MatrixXd& values)
	{
		MatrixXd l(pRobot->getNow(),1);
		MatrixXd wireErrors(8,1); wireErrors = MatrixXd::Zero(8,1);
		chooseInverseKinematics(r,R,l);

		MatrixXd p(6,1), lb(6,1), ub(6,1);
		p = MatrixXd::Zero(6, 1);
		lb = MatrixXd::Zero(6, 1);
		ub = MatrixXd::Zero(6, 1);		
		
		generateWireErrors(l, pRobot->getNow(), wireErrors, wireErrorType, errorrange);
		choosePoseEstimate(l, p, lb, ub);
		values.resize(getPropertyCount(),1);

		double check[6] = {r(0), r(1), r(2), 0,0,0};
		getXYZFromMatrix(check[3],check[4],check[5], R);
		for(int i = 0; i < 6; i++){
			if(showBounds){
				values(4*i) = p(i);
				values(4*i+1) = lb(i);
				values(4*i+2) = ub(i);
				values(4*i+3) = (check[i] < lb(i) || check[i] > ub(i))? 1 : 0;		// if there are no bounds set by the poseestimator, it will (in most cases) return 1
			}else{
				values(i) = p(i);
			}
		}
		Vector3d rResult(p(0),p(1),p(2));
		values((showBounds) ? 24 : 6) = (rResult-r).norm();

		Matrix3d RResult;
		Vector3d xyz(p(3),p(4),p(5));
		getMatrixFromXYZ(RResult, xyz);
		values((showBounds) ? 25 : 7) = getSO3Distance(RResult,R);
		
		if(showWireErrors){
			for(int i = 0; i < 8; i++){
				values(((showBounds) ? 26 : 8)+i) = wireErrors(i);
			}
		}

		return true;
	}

	void setInverseSolverType(emInverseModel::Type inverseSolverType){this->inverseSolverType = inverseSolverType;}
	void setPoseEstimateType(emPoseEstimate::Type poseEstimateType){this->poseEstimateType = poseEstimateType;}
	void setShowBounds(bool showBounds){this->showBounds = showBounds;}
	void setWireErrorType(emWireErrors::Type wireErrorType){this->wireErrorType = wireErrorType;}
	void setErrorrange(double errorrange){this->errorrange = errorrange;}
	void setShowWireerrors(bool showWireErrors){this->showWireErrors = showWireErrors;}

private:
	void chooseInverseKinematics(const Vector3d& r, const Matrix3d& R, MatrixXd& l, double* lf=0, double* beta=0, double* gamma=0, Vector3d* C=0, Vector3d* U=0)
	{
		switch(inverseSolverType) {
			// Use doInverseKinematics
			case emInverseModel::standard:
				pKin->doInverseKinematics(r, R, l);
				break;
			// Use doInverseKinematicsPulley
			case emInverseModel::pulley:
				pKin->doInverseKinematicsPulley(r, R, l);
				break;
			// same as case IK
			default:
				pKin->doInverseKinematics(r, R, l);
				break;
		}
	}

	void choosePoseEstimate(const MatrixXd& l, MatrixXd& p, MatrixXd& lb, MatrixXd& ub){
		switch(poseEstimateType) {
			// Use basic without bounding box
			case emPoseEstimate::basic:
				pKin->poseestimate(l, p);
				break;
			// Use basic with simple bounding box
			case emPoseEstimate::basicSimpleBoundingbox:
				pKin->poseestimate(l, p, ub, lb);
				break;
			/*case CKinematics::PoseEstimateType::inherit:
				pKin->poseestimateInherit(l, p, r, R);
				break; */
			/* Use basic with more complicated bounding box
			case basicComplexBoundingBox:
				poseestimateInt(l, p, ub, lb);
				break;*/
			case emPoseEstimate::intersectionMethod:
				pKin->poseestimate_intersectionmethod(l, p);
				break;
			// same as basicSimpleBoundingbox
			default:
				pKin->poseestimate(l, p, ub, lb);
				break;
		}
	}
};

} // namespace PCRL