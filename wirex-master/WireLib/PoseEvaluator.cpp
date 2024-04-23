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
 *  \file   : PoseEvaluator.cpp
 *
 *  Project   Wirelib
 *
 *  \Author   Andreas Pott
 *
 *  \data	  23.04.2014
 *
 *********************************************************************
 */ 

#include "PoseEvaluator.h"
#include <time.h>
#include <motionPlanning/Utilities.h>


namespace PCRL {
// implementation of CEuclidianPoseEvaluator
/////////////////////////////////////////////////////////////////////

//helper function


CEuclidianPoseEvaluator::CEuclidianPoseEvaluator(CRobotData& robot) 
	: CPoseProperty(robot), parameterization() {}


//! get the number of properties computed
int CEuclidianPoseEvaluator::getPropertyCount() 
{
	switch (parameterization)
	{
	case emOrientationParameter::rotationMatrix: 
		return 12;
	case emOrientationParameter::quaternion: 
		return 7;
	case emOrientationParameter::eulerAngles: 
		return 6;
	default:
		return 0;
	}
}


//! return the names of the properties to be used as headers in tables by appending the names to the vector
bool CEuclidianPoseEvaluator::getPropertyNames(std::vector<string>& names) 
{
	// statically return the six column names
	names.push_back("x");
	names.push_back("y");
	names.push_back("z");
	switch (parameterization)
	{
	case emOrientationParameter::rotationMatrix: 
		names.push_back("R11");
		names.push_back("R21");
		names.push_back("R31");
		names.push_back("R12");
		names.push_back("R22");
		names.push_back("R32");
		names.push_back("R13");
		names.push_back("R23");
		names.push_back("R33");
		break;
	case emOrientationParameter::quaternion: 
		names.push_back("q0");
		names.push_back("q1");
		names.push_back("q2");
		names.push_back("q3");
		break;
	case emOrientationParameter::eulerAngles: 
		names.push_back("Euler_a");
		names.push_back("Euler_b");
		names.push_back("Euer_c");
		break;
	}
	return true;
}


//! declare the bindings 
void CEuclidianPoseEvaluator::bind()
{
	CAlgorithm::bind();
	if (!pReflector)
		return;
	pReflector->bind((int&)parameterization,"parameterization","euclidianPoseEvaluator/@parameterization");
	pReflector->addEnumTable("euclidianPoseEvaluator/@parameterization", emOrientationParameter::Names);
}


//! do the actual computation of the properties for the given pose (r,R)
bool CEuclidianPoseEvaluator::computeProperty(const Vector3d& r, const Matrix3d& R, MatrixXd& values)
{
	values.resize(getPropertyCount(),1);
	values.block(0,0,3,1) = r;
	switch (parameterization)
	{
	case emOrientationParameter::rotationMatrix: 
		values.block(3,0,3,1) = R.col(0);
		values.block(6,0,3,1) = R.col(1);
		values.block(9,0,3,1) = R.col(2);
		break;
	case emOrientationParameter::quaternion: 
		getQuaternionFromMatrix(values(3,0),values(4,0),values(5,0),values(6,0),R);
		break;
	case emOrientationParameter::eulerAngles:
		getXYZFromMatrix(values(3,0),values(4,0),values(5,0),R);
		break;
	}
	return true;
}


// implementation of CForceDistributionEvaluator
/////////////////////////////////////////////////////////////////////

bool CForceDistributionEvaluator::getPropertyNames(std::vector<string>& names) 
{	
	for (int i=0; i<pRobot->getNow(); i++)
	{
		stringstream ss;
		ss << "f" << i;
		names.push_back(ss.str());
	}
	names.push_back("f_2norm");
	names.push_back("f_infnorm");
	names.push_back("f_deviation");
	names.push_back("f_mean");
	names.push_back("fv_2norm");
	names.push_back("fv_infnorm");
	names.push_back("fv_mean");
	names.push_back("fv_deviation");
	return true;
}


void CForceDistributionEvaluator::bind()
{
	if (!pReflector)
		return;
	pReflector->bind((int&)forceDistributionMethod,"forceDistributionMethod","ForceDistributionEvaluator/@forceDistributionMethod");
	pReflector->bind((int&)evaluatorCriteria,"evaluatorCriteria","ForceDistributionEvaluator/@evaluatorCriteria");
	pReflector->addEnumTable("ForceDistributionEvaluator/@forceDistributionMethod", emForceDistributionMethod::Names);
	pReflector->addEnumTable("ForceDistributionEvaluator/@evaluatorCriteria", emEvaluatorCriteria::Names);
}


//! do the actual computation of the properties for the given pose (r,R)
bool CForceDistributionEvaluator::computeProperty(const Vector3d& r, const Matrix3d& R, MatrixXd& values)
{
	pFD->setMethod(forceDistributionMethod, evaluatorCriteria); // gets set every pose, although just needed once in the beginning, therefore no second implementation of choosing method in this evaluator
	values.resize(getPropertyCount(),1);
	MatrixXd w = MatrixXd::Zero(pRobot->getDof(),1);
	pFD->setWrenchVector(w,pFD->f,pFD->tau);
	if (!pFD->getStructureMatrix(r,R))
		return false;
	MatrixXd f_wires;
	if (!pFD->getDistribution(w, pRobot->fmin, pRobot->fmax, f_wires))
		return false;
	MatrixXd f_v = f_wires-MatrixXd::Constant(pRobot->getNow(),1,0.5*(pRobot->fmax+pRobot->fmin)); 
	values <<
		f_wires,
		f_wires.lpNorm<2>(),
		f_wires.lpNorm<Eigen::Infinity>(),
		f_wires.mean(),
		stddev(f_wires),
		f_v.lpNorm<2>(),
		f_v.lpNorm<Eigen::Infinity>(),
		f_v.mean(),
		stddev(f_v);
	return true;
}





//! do the actual computation of the properties for the given pose (r,R)
bool CForceDistributionEvaluatorDynamic::computeProperty(const Vector3d& r, const Matrix3d& R, const Vector3d& v,
					  const Vector3d& omega, const Vector3d& a,const Vector3d& alpha,
					  const Vector3d& f, const Vector3d& tau, MatrixXd& values)
{
	pFD->setMethod(forceDistributionMethod, evaluatorCriteria); // gets set every pose, although just needed once in the beginning, therefore no second implementation of choosing method in this evaluator
	values.resize(getPropertyCount(),1);
	MatrixXd w = MatrixXd::Zero(pRobot->getDof(),1);

	Vector3d fDynamic;
	Vector3d tauDynamic;
	Vector3d gravity(0, 0, 9.81);
	Vector3d acceleration;

	// Add gravity
	acceleration = a + gravity;

	pFD->getWrenchDynamic(R, v, omega, acceleration, alpha, fDynamic,  tauDynamic);
	for (int i=0;i<3;i++) // load f and tau into w
	{
		w(i) = fDynamic(i);
		w(i+3) = tauDynamic(i);
	}

	//pFD->setWrenchVector(w,pFD->f,pFD->tau);
	if (!pFD->getStructureMatrix(r,R))
		return false;
	MatrixXd f_wires;
	if (!pFD->getDistribution(w, pRobot->fmin, pRobot->fmax, f_wires))
		return false;

	// check if the pose is feasible
	bool bPoseValid = pFD->rateDistributionFeasible(f_wires)!=0 ? true : false;
	
	MatrixXd x_dot = MatrixXd::Zero(pRobot->getDof(),1);
	MatrixXd x_ddot = MatrixXd::Zero(pRobot->getDof(),1);
	MatrixXd l_dot_set = MatrixXd::Zero(pRobot->getNow(),1);
	MatrixXd l_ddot_set = MatrixXd::Zero(pRobot->getNow(),1);

	MatrixXd q_dot = MatrixXd::Zero(pRobot->getNow(),1);
	MatrixXd q_ddot = MatrixXd::Zero(pRobot->getNow(),1);
	MatrixXd torque = MatrixXd::Zero(pRobot->getNow(),1);
	MatrixXd power = MatrixXd::Zero(pRobot->getNow(),1);

	for (int i=0;i<3;i++) 
	{
		x_dot(i) = v(i);
		x_dot(i+3) = omega(i);
		x_ddot(i) = a(i);
		x_ddot(i+3) = alpha(i);
	}

	MatrixXd AT_dot = MatrixXd::Zero(pRobot->getDof(),pRobot->getNow());
	MatrixXd AT = MatrixXd::Zero(pRobot->getDof(),pRobot->getNow());
	pFD->getMatrix(AT);
	pFD->getStructureMatrixTimeDerivated(AT_dot, r, R, v, omega);

	// get cable velocity
	l_dot_set=-AT.transpose()*x_dot;
	// get cable acceleration
	l_ddot_set=-AT.transpose()*x_ddot - AT_dot.transpose()*x_dot;
	
	double gear_ratio;	//!< gear ratio (values >1 reduce the velocity of the cable)
	double r_drum;		//!< radius of the drum in the winch [m]
	double n_drum;		//!< maximum number of windings on the drum
	double l_drum;		//!< axial length of the drum [m]
	int spool_direction;
	double inertia_motor; // respective to the motor axis; has to include all rotational intertia of the winch
	//double inertia_gearbox;
	//double inertia_winch; // respective the drum axis
	double efficiency_gearbox;
	double efficiency_winch;

	double M_wire, M_inertia, M_friction;
	double inertia_total;

	for (int i=0;i<pRobot->getNow();i++) 
	{
		// get winch data from pRobot
		gear_ratio = pRobot->pWinch->gear_ratio;
		r_drum = pRobot->pWinch->r_drum;
		n_drum = pRobot->pWinch->n_drum;
		l_drum = pRobot->pWinch->l_drum;
		inertia_motor = pRobot->pWinch->inertia;
	
		// not included in XML-Spec
		//inertia_gearbox = 0.0;
		//inertia_winch = 0.0; 
		efficiency_gearbox = 1.0; //0.99;
		efficiency_winch = 1.0; //0.95; 
		spool_direction = 1;
	
		q_dot(i) = l_dot_set(i) / (r_drum + spool_direction*l_drum/(n_drum*2.0*MO_PI)) * gear_ratio; // rad/s
		q_ddot(i) = l_ddot_set(i) / (r_drum + spool_direction*l_drum/(n_drum*2.0*MO_PI)) * gear_ratio; // rad/s²

		inertia_total = inertia_motor; // + inertia_gearbox + inertia_winch/(pow(gear_ratio,2.0)); // kg m²
	
		M_wire = f_wires(i) * (r_drum + spool_direction*l_drum/(n_drum*2.0*3.14)) / gear_ratio; 
		M_inertia = q_ddot(i)*inertia_total;
		M_friction = M_wire*(1.0/(efficiency_gearbox*efficiency_winch)-1.0); 
		torque(i) = M_wire + M_inertia + M_friction;
		power(i) = torque(i)*q_dot(i);
	}

	MatrixXd f_v = f_wires-MatrixXd::Constant(pRobot->getNow(),1,0.5*(pRobot->fmax+pRobot->fmin)); 
	values <<
		f_wires,
		f_wires.lpNorm<2>(),
		f_wires.lpNorm<Eigen::Infinity>(),
		f_wires.mean(),
		stddev(f_wires),
		f_v.lpNorm<2>(),
		f_v.lpNorm<Eigen::Infinity>(),
		f_v.mean(),
		stddev(f_v),
		w,
		bPoseValid,
		x_dot,
		x_ddot,
		l_dot_set,
		l_ddot_set,
		torque,
		q_dot,
		q_ddot,
		power
		;
	return true;
}


// implementation of CForceDistributionEvaluatorDynamic
/////////////////////////////////////////////////////////////////////

bool CForceDistributionEvaluatorDynamic::getPropertyNames(std::vector<string>& names) 
{	

	for (int i=0; i<pRobot->getNow(); i++)
	{
		stringstream ss;
		ss << "f" << i;
		names.push_back(ss.str());
	}
	names.push_back("f_2norm");
	names.push_back("f_infnorm");
	names.push_back("f_deviation");
	names.push_back("f_mean");
	names.push_back("fv_2norm");
	names.push_back("fv_infnorm");
	names.push_back("fv_mean");
	names.push_back("fv_deviation");
	names.push_back("wrench_x");
	names.push_back("wrench_y");
	names.push_back("wrench_z");
	names.push_back("wrench_a");
	names.push_back("wrench_b");
	names.push_back("wrench_c");
	names.push_back("bPoseValid");
	for (int i=0; i<pRobot->getDof(); i++)
	{
		stringstream ss;
		ss << "x_dot" << i;
		names.push_back(ss.str());
	}
	for (int i=0; i<pRobot->getDof(); i++)
	{
		stringstream ss;
		ss << "x_ddot" << i;
		names.push_back(ss.str());
	}
	for (int i=0; i<pRobot->getNow(); i++)
	{
		stringstream ss;
		ss << "l_dot" << i;
		names.push_back(ss.str());
	}
	for (int i=0; i<pRobot->getNow(); i++)
	{
		stringstream ss;
		ss << "l_ddot" << i;
		names.push_back(ss.str());
	}
	for (int i=0; i<pRobot->getNow(); i++)
	{
		stringstream ss;
		ss << "torque" << i;
		names.push_back(ss.str());
	}
	for (int i=0; i<pRobot->getNow(); i++)
	{
		stringstream ss;
		ss << "q_dot" << i;
		names.push_back(ss.str());
	}
	for (int i=0; i<pRobot->getNow(); i++)
	{
		stringstream ss;
		ss << "q_ddot" << i;
		names.push_back(ss.str());
	}
	for (int i=0; i<pRobot->getNow(); i++)
	{
		stringstream ss;
		ss << "power" << i;
		names.push_back(ss.str());
	}
	return true;
}


void CForceDistributionEvaluatorDynamic::bind()
{
	if (!pReflector)
		return;
	pReflector->bind((int&)forceDistributionMethod,"forceDistributionMethod","ForceDistributionEvaluatorDynamic/@forceDistributionMethod");
	pReflector->bind((int&)evaluatorCriteria,"evaluatorCriteria","ForceDistributionEvaluatorDynamic/@evaluatorCriteria");
	pReflector->addEnumTable("ForceDistributionEvaluatorDynamic/@forceDistributionMethod", emForceDistributionMethod::Names);
	pReflector->addEnumTable("ForceDistributionEvaluatorDynamic/@evaluatorCriteria", emEvaluatorCriteria::Names);
}



// implementation of COrientationWorkspaceEvaluator
/////////////////////////////////////////////////////////////////////

//! get the number of properties computed
int COrientationWorkspaceEvaluator::getPropertyCount() { return 7; }

//! return the names of the properties to be used as headers in tables by appending the names to the vector
bool COrientationWorkspaceEvaluator::getPropertyNames(std::vector<string>& names) 
{	
	// statically return the six column names
	names.push_back("alphamin");
	names.push_back("betamin");
	names.push_back("gammamin");
	names.push_back("alphamax");
	names.push_back("betamax");
	names.push_back("gammamax");
	names.push_back("ows_volume");
	return true;
}

//! do the actual computation of the properties for the given pose (r,R)
bool COrientationWorkspaceEvaluator::computeProperty(const Vector3d& r, const Matrix3d& R, MatrixXd& values)
{
	values.resize(getPropertyCount(),1);
	values(0,0) = pWSA->lineSearchAngular(r,Vector3d(-1,0,0),0,MO_PI/2.0) / MO_PI * -180.0;
	values(1,0) = pWSA->lineSearchAngular(r,Vector3d(0,-1,0),0,MO_PI/2.0) / MO_PI * -180.0;
	values(2,0) = pWSA->lineSearchAngular(r,Vector3d(0,0,-1),0,MO_PI/2.0) / MO_PI * -180.0;
	values(3,0) = pWSA->lineSearchAngular(r,Vector3d( 1,0,0),0,MO_PI/2.0) / MO_PI * 180.0;
	values(4,0) = pWSA->lineSearchAngular(r,Vector3d(0, 1,0),0,MO_PI/2.0) / MO_PI * 180.0;
	values(5,0) = pWSA->lineSearchAngular(r,Vector3d(0,0, 1),0,MO_PI/2.0) / MO_PI * 180.0;
	values(6,0) = (values(3,0)-values(0,0)) * (values(4,0)-values(1,0)) * (values(5,0)-values(2,0));	// perhaps we need plus instead of minus
	return true;
}

} // namespace PCRL
