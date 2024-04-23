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
 *  \file   : StructureMatrix.cpp
 *
 *  Project   Wirelib
 *
 *  \Author   Andreas Pott
 *
 *  \Date     11.12.2009	(part of MoWiRoGeometry)
 *			  15.12.2009	(refactored by asp)
 *
 *********************************************************************
 */ 

#include "StructureMatrix.h"
#include "Kinematics.h"

namespace PCRL {
extern "C" {
#ifdef WIRELIB_HAS_CVXGEN
#include <cvxgen/solver.h> // needs to be in the namespace PCRL
#endif // WIRELIB_HAS_CVXGEN
}

CStructureMatrix::CStructureMatrix(CRobotData& robot)
: CPoseProperty(robot)
{
	f = tau.setZero();
	AT.resize(pRobot->getDof(),pRobot->getNow());
	usePulley = false;
}

void CStructureMatrix::bind()
{
	CPoseProperty::bind();
	if (!pReflector)
		return;
	pReflector->bind(usePulley,"usePulley","structureMatrix/@usePulley");
	pReflector->bind(f.x(),"f.x","structureMatrix/wrench/@fx");
	pReflector->bind(f.y(),"f.y","structureMatrix/wrench/@fy");
	pReflector->bind(f.z(),"f.z","structureMatrix/wrench/@fz");
	pReflector->bind(tau.x(),"tau.x","structureMatrix/wrench/@taux");
	pReflector->bind(tau.y(),"tau.y","structureMatrix/wrench/@tauy");
	pReflector->bind(tau.z(),"tau.z","structureMatrix/wrench/@tauz");
}

/*! setup the structure matrix AT that fullfils the static equilibrium equation
 *  A^T*f+w = 0
 *  where A^T is the pose dependant structure matrix, f is the vector of the 
 *  wire forces and w are the forces and torques applied to the mobile platform.
 *  \param [in] r: the Cartesian position of the platform
 *  \param [in] R: the orientation of the platform
 *  \return true, if successful. In this case the internal variable AT contains the structure matrix, otherwise false
 *  
 *  \remark
 *  The current implementation supports the following motion patterns of the robot
 *  (MP3R3T, MP2T, MP3T, MP1R2T). It will return false if the robots has an unsupportet motion pattern.
 */
bool CStructureMatrix::getStructureMatrix(const Vector3d& r, const Matrix3d& R)
{
	// ------------------------------------------
	// TAD 24. Mar. 2010: This is a hot fix. 
	// Upon loading of the robot the size structure matrix is initialized. 
	// Subsequent loading of a robot with a lower number of tendons did not trigger a resize of the structure matrix
	// The following line fixes this issue but is only a hot fix that will decrease performance:
	AT.resize(pRobot->getDof(),pRobot->getNow()); 
	// ------------------------------------------

	// calculate the structure matrix for the full spatial 3R3T motion pattern
	if (pRobot->getMotionPattern() == CRobotData::MP3R3T && !usePulley)
	{
		for (int i=0; i<pRobot->getNow(); i++)
		{
			// get platform pivot in world coordinates
			Vector3d P = R*pRobot->getPlatform(i);
			// calculate wire vector 
			Vector3d u = pRobot->getBase(i)-r-P;
			// normalize the vector to get the wire direction (and thus the direction of the force)
			u.normalize();
			// fill the first column of the structure matrix
			Vector3d B; 
			B = P.cross(u);
			AT.block(0,i,3,1) = u;
			AT.block(3,i,3,1) = B;
		}

		return true;
	}

	// calculate the structure matrix for the full spatial 3R3T motion pattern 
	// taking into account the influence of pulleys in the winches
	if (pRobot->getMotionPattern() == CRobotData::MP3R3T && usePulley)
	{
		PCRL::CKinematics kin(*pRobot);
		Vector3d *U = new Vector3d[pRobot->getNow()];
		kin.doInverseKinematicsPulleyEx(r,R,0,0,0,0,0,U);

		for (int i=0; i<pRobot->getNow(); i++)
		{
			// get platform pivot in world coordinates
			Vector3d P = R*pRobot->getPlatform(i);
			// calculate wire vector from the above received vector U
			U[i].normalize();
			// fill the first column of the structure matrix
			Vector3d B; 
			B = P.cross(U[i]);
			AT.block(0,i,3,1) = U[i];
			AT.block(3,i,3,1) = B;
		}

		delete [] U;
		return true;
	}

	// calculate the structure matrix for the full spatial purely translational 3T motion pattern
	else if (pRobot->getMotionPattern() == CRobotData::MP3T)
	{
		for (int i=0; i<pRobot->getNow(); i++)
		{
			// get platform pivot in world coordinates
			Vector3d P = R*pRobot->getPlatform(i);
			// calculate wire vector 
			Vector3d u = pRobot->getBase(i)-r-P;
			// normalize the vector to get the wire direction (and thus the direction of the force)
			u.normalize();
			AT.block(0,i,3,1) = u;
		}
		return true;
	}
	// calculate the structure matrix for the planar purely translational 2T motion pattern
	//! \todo Implement a method to check if the anchor points platform and base are really in a common xy-plane!
	else if (pRobot->getMotionPattern() == CRobotData::MP2T)
	{
		for (int i=0; i<pRobot->getNow(); i++)
		{
			// get platform pivot in world coordinates
			Vector3d P = R*pRobot->getPlatform(i);
			// calculate wire vector 
			Vector3d u = pRobot->getBase(i)-r-P;
			// normalize the vector to get the wire direction (and thus the direction of the force)
			u.normalize();
			AT(0,i) = u.x();
			AT(1,i) = u.y();
		}
		return true;
	}
	else if (pRobot->getMotionPattern() == CRobotData::MP1R2T)
	{
		for (int i=0; i<pRobot->getNow(); i++)
		{
			// get platform pivot in world coordiantes
			Vector3d P = R*pRobot->getPlatform(i);
			// calculate wire vector 
			Vector3d u = pRobot->getBase(i)-r-P;
			// normalize the vector to get the wire direction (and thus the direction of the force)
			u.normalize();
			// fill the first column of the structure matrix
			Vector3d B; 
			B = P.cross(u);
			AT(0,i) = u.x();
			AT(1,i) = u.y();
			AT(2,i) = B.z();
		}
		return true;
	}

	// the selected motion pattern is not supported
	return false;
}

//! calculate the structure matrix for pose (r,R) without normalization of the direction vectors
//! currently only the cases 3R3T and 1R2T are supported. This implementation is somewhat a
//! draft. We might do better with having a boolean flag 'normalize' and do a conditional call to 
//! normalization of vector u in the standard structure matrix function
bool CStructureMatrix::getNonNormalizedStructureMatrix(const Vector3d& r, const Matrix3d& R)
{
	// do resizing only if required
	if (AT.rows() != pRobot->getDof() || AT.cols() != pRobot->getNow())
		AT.resize(pRobot->getDof(),pRobot->getNow()); 

	// calculate the structure matrix for the full spatial 3R3T motion pattern
	if (pRobot->getMotionPattern() == CRobotData::MP3R3T && !usePulley)
	{
		for (int i=0; i<pRobot->getNow(); i++)
		{
			// get platform pivot in world coordinates
			Vector3d P = R*pRobot->getPlatform(i);
			// calculate wire vector 
			Vector3d u = pRobot->getBase(i)-r-P;
			// fill the first column of the structure matrix
			Vector3d B; 
			B = P.cross(u);
			AT.block(0,i,3,1) = u;
			AT.block(3,i,3,1) = B;
		}

		return true;
	}
	else if (pRobot->getMotionPattern() == CRobotData::MP1R2T)
	{
		for (int i=0; i<pRobot->getNow(); i++)
		{
			// get platform pivot in world coordiantes
			Vector3d P = R*pRobot->getPlatform(i);
			// calculate wire vector 
			Vector3d u = pRobot->getBase(i)-r-P;
			// fill the first column of the structure matrix
			Vector3d B; 
			B = P.cross(u);
			AT(0,i) = u.x();
			AT(1,i) = u.y();
			AT(2,i) = B.z();
		}
		return true;
	}

	// the selected motion pattern is not supported
	return false;
}


/*! return the unit direction vector u[i] of the i-th cable by reading the 
 *  respective elements of the structure matrix
 *  \param i [in] zero-based index of the cable
 *  \return direction of the vector if successful, otherwise the null vector is returned
 *  \todo Distinguish between the motion patterns
 */
Vector3d CStructureMatrix::getDirection(const unsigned int& i)
{
	if (i >= (unsigned int)pRobot->getNow())
		return Vector3d::Zero();
	if (pRobot->getMotionPattern() == CRobotData::MP3R3T || 
		pRobot->getMotionPattern() == CRobotData::MP2R3T || 
		pRobot->getMotionPattern() == CRobotData::MP3T)
	{
		Vector3d u;
		u.x() = AT(0,i);
		u.y() = AT(1,i);
		u.z() = AT(2,i);
		return u;
	}
	return Vector3d::Zero();
}

/*! set wrench vector w based on the entries in vectors F and Tau
 *  the assignment from (F,Tau)-->w is based on the current motion pattern.
 *  \return true, if successfully setting the vector. false e.g. if the 
 *  current motion pattern in not supported. 
 */
bool CStructureMatrix::setWrenchVector(MatrixXd& w, const Vector3d& F, const Vector3d& Tau)
{
	if (pRobot->getMotionPattern() == CRobotData::MP2T)
	{
		// apply the load (only for the dof=2 case)
		w.resize(2,1);
		w(0) = f.x();
		w(1) = f.y();
		return true;
	}
	else if (pRobot->getMotionPattern() == CRobotData::MP3T)
	{	
		// apply the load (only for the dof=3 case)
		w.resize(3,1);
		w(0) = f.x();
		w(1) = f.y();
		w(2) = f.z();
		return true;
	}
	else if (pRobot->getMotionPattern() == CRobotData::MP3R3T)
	{
		// apply the load (only for the dof=6 case)
		w.resize(6,1);
		w(0) = f.x();
		w(1) = f.y();
		w(2) = f.z();
		w(3) = tau.x();
		w(4) = tau.y();
		w(5) = tau.z();
		return true;
	}
	else if (pRobot->getMotionPattern() == CRobotData::MP1R2T)
	{
		// apply planar forces in xy and the z-component of the torque
		w.resize(3,1);
		w(0) = f.x();
		w(1)= f .y();
		w(2) = tau.z();
		return true;
	}
	return false;
}

//! get the maximum norm for all columns of the structure matrix
//! \todo This can be more elegant using the functions of eigen3!
double CStructureMatrix::getMaxColumnNorm()
{
	double max = 0;
	for (int i=0; i<pRobot->getDof(); i++)
	{
		double sum = 0;
		for (int j=0; j<pRobot->getNow(); j++)
			sum += fabs(AT(i,j));
		if (sum > max)
			max = sum;
	}
	return max;
}

//! get the maximum norm for all rows of the structure matrix
//! \todo This can be more elegant using the functions of eigen3!
double CStructureMatrix::getMaxRowNorm()
{
	double max = 0;
	for (int j=0; j<pRobot->getNow(); j++)
	{
		double sum = 0;
		for (int i=0; i<pRobot->getDof(); i++)
			sum += fabs(AT(i,j));
		if (sum > max)
			max = sum;
	}
	return max;
}

//! compute the singular values of the structure matrix
//! \param [out] sv The computed singular values are stored in this variable
//! \return true, if succesfull.
bool CStructureMatrix::getSingularValues(MatrixXd& sv)
{
	sv = AT.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).singularValues();
	return true;
}

//! determine the rank of the structure matrix
int CStructureMatrix::getRank()
{
	return AT.fullPivLu().rank();
}

//! test if the structure matrix has the full rank or in other words, test if the robot is in a singular configuration
bool CStructureMatrix::isFullRank()
{
	return getRank() == pRobot->getDof();
}

//! get an element of the structureMatrix
double CStructureMatrix::getElement(const unsigned int& i, const unsigned int& j) const
{
	if (i < static_cast<unsigned int>(pRobot->getDof()) && j < static_cast<unsigned int>(pRobot->getNow()))
		return AT(i,j);
	else
		return numeric_limits<double>::quiet_NaN();
}

/*! the same as the other function but with conviniece parameter passing through pose objects
 */
bool CStructureMatrix::getWrenchDynamic(const CPoseKinetostatic& poseKinetostatic, MatrixXd& w)
{
	Vector3d fDynamic, tauDynamic;
	if (this->getWrenchDynamic(poseKinetostatic.R, poseKinetostatic.v, poseKinetostatic.omega, poseKinetostatic.a, poseKinetostatic.alpha, fDynamic, tauDynamic))
	{
		for (int i=0;i<3;i++) // load f and tau into w
		{
			w(i) = fDynamic(i);
			w(i+3) = tauDynamic(i);
		}
		return true;
	}
	else 
		return false;
}

/*! get the wrench dynamic based on given velocity (v, omega) and acceleration (a, alpha) 
 *  Newton-Euler equations are used to determine the wrench dynamic
 *  for inertia tensor, platform_mass and center of gravity the "robot data"-description is used
 *  this is a beta-implementation, the robot parameters are hard-coded in this function
 *  \return true, if successful
 */
bool CStructureMatrix::getWrenchDynamic(const Matrix3d& R, const Vector3d& v, const Vector3d& omega, const Vector3d& a, const Vector3d& alpha, Vector3d& fDynamic, Vector3d& tauDynamic)
{
	Matrix3d inertiaTensorRotated;
	Matrix3d crossProductMatrix = Matrix3d::Zero(3,3);
	Matrix3d massMatrix;

//	Matrix3d inertiaTensor=Matrix3d::Zero(3,3);

	massMatrix = Matrix3d::Identity(3,3)*pRobot->platform_mass;

	inertiaTensorRotated = R*pRobot->platform_inertia*R.transpose();

	// skew matrix for the center of gravity
	crossProductMatrix(0,1) = -(R*(pRobot->centerofgravity)).z();
	crossProductMatrix(0,2) = (R*(pRobot->centerofgravity)).y();
	crossProductMatrix(1,0) = (R*(pRobot->centerofgravity)).z();
	crossProductMatrix(1,2) = -(R*(pRobot->centerofgravity)).x();
	crossProductMatrix(2,0) = -(R*(pRobot->centerofgravity)).y();
	crossProductMatrix(2,1) = (R*(pRobot->centerofgravity)).x();

	fDynamic = massMatrix*a - (massMatrix*crossProductMatrix)*alpha + massMatrix*omega.cross(omega.cross(R*(pRobot->centerofgravity))) ;
	tauDynamic = massMatrix*crossProductMatrix*a + (inertiaTensorRotated - massMatrix*crossProductMatrix*crossProductMatrix)*alpha    + omega.cross((inertiaTensorRotated - massMatrix*crossProductMatrix*crossProductMatrix)*omega);

	fDynamic = -fDynamic;
	tauDynamic = -tauDynamic;

	return true;
}


//! get AT_dot based on a given plattform velocity
void CStructureMatrix::getStructureMatrixTimeDerivated(MatrixXd& At_dot, const Vector3d& r, const Matrix3d& R, const Vector3d& v, const Vector3d& omega)
{


	//MatrixXd  AT_derivated(6,6); // derivation of the Structure matrix along the Cartesian axes
	Vector3d li, ui, li_point, ui_point, vi_point; // helper variables
	double li_norm, li_point_norm; // helper variables
	MatrixXd t(6,1); // direction of the derivation
	for (int i=0; i<3; i++)
	{
		t(i,0)=v(i);
		t(i+3,0)=omega(i);
	}

	// skew matrix for the angular velocity
	Matrix3d crossProductMatrix = Matrix3d::Zero(3,3);
	crossProductMatrix(0,1) = -t(5);
	crossProductMatrix(0,2) = t(4);
	crossProductMatrix(1,0) = t(5);
	crossProductMatrix(1,2) = -t(3);
	crossProductMatrix(2,0) = -t(4);
	crossProductMatrix(2,1) = t(3);


	for (int i=0; i<pRobot->getNow(); i++)
	{
		li =- r - R*pRobot->getPlatform(i) + pRobot->getBase(i);
		ui = li.normalized();

		li_norm=li.norm();

		li_point = -t.block(0,0,3,1)-crossProductMatrix*R*pRobot->getPlatform(i);
		li_point_norm = (li.dot(li_point))/li_norm;
		ui_point = (li_point*li_norm-li*li_point_norm)/(li_norm*li_norm);
		for (int i_=0; i_<3; i_++)
			At_dot(i_,i) = ui_point(i_);
		
		vi_point = (crossProductMatrix*R*pRobot->getPlatform(i)).cross(ui) + (R*pRobot->getPlatform(i)).cross(ui_point);
		for (int i_=0; i_<3; i_++)
			At_dot(i_+3,i) = vi_point(i_);	

	}	
}


CForceDistribution::CForceDistribution(CRobotData& robot)
: CStructureMatrix(robot)
{
	forceDistributionMethod = emForceDistributionMethod::closedForm;
	evaluatorCriteria = emEvaluatorCriteria::feasible;
	linearSolver = emLinearSolverMethod::llt;	// this one seems rather fast
	condMin = 0.01; 
	f_ref_QP = 0; // 0 will lead to minimum cable forces
#ifdef WIRELIB_HAS_CVXGEN
	set_defaults(); // set the default values to the QP algoritm 
#endif // WIRELIB_HAS_CVXGEN
	QPIterations = 0;	
	QPconverged = 0;	
	dykstraIterations = 0;
}


void CForceDistribution::bind()
{
	CStructureMatrix::bind();
	if (!pReflector)
		return;
	pReflector->bind((int&)forceDistributionMethod,"ForceDistributionMethod","structureMatrix/ForceDistribution/@ForceDistributionMethod");
	pReflector->bind((int&)evaluatorCriteria,"EvaluatorCriteria","structureMatrix/ForceDistribution/@EvaluatorCriteria");
	pReflector->bind((int&)linearSolver,"linearSolver","structureMatrix/ForceDistribution/@linearSolver");
	pReflector->bind(condMin,"condMin","structureMatrix/ForceDistribution/@condMin");
	CReflection::addEnumTable("structureMatrix/ForceDistribution/@ForceDistributionMethod",emForceDistributionMethod::Names);
	CReflection::addEnumTable("structureMatrix/ForceDistribution/@EvaluatorCriteria",emEvaluatorCriteria::Names);
	CReflection::addEnumTable("structureMatrix/ForceDistribution/@linearSolver",emLinearSolverMethod::Names);
}

void CForceDistribution::setMethod(int method, int criteria)
{
	forceDistributionMethod = (emForceDistributionMethod::Type)method;

	switch (criteria)
	{
	case 0: evaluatorCriteria = emEvaluatorCriteria::feasible; break;
	case 1: evaluatorCriteria = emEvaluatorCriteria::euclidean; break;
	case 2:
	default:evaluatorCriteria = emEvaluatorCriteria::infinity; break;
	}
}

void CForceDistribution::getMethod(int &method, int &criteria)
{
	method = forceDistributionMethod;
	criteria = evaluatorCriteria;
}

void CForceDistribution::setLinearSolver(int solver) 
{ 
	if (solver <= 0) 
		linearSolver = emLinearSolverMethod::lu;
	else if (solver >= 5)
		linearSolver = emLinearSolverMethod::explicitPseudoinverse;
	else
		linearSolver = (emLinearSolverMethod::Type)solver;
}

//! calculate the force distribution with the selected method
bool CForceDistribution::getDistribution(MatrixXd& w, double& df_min, double& df_max, MatrixXd& f_wires)
{
	MatrixXd f_min = MatrixXd::Constant(pRobot->getNow(),1, df_min);		// the minimum force vector
	MatrixXd f_max = MatrixXd::Constant(pRobot->getNow(),1, df_max);		// the maximum force vector
	return getDistribution(w, f_min, f_max, f_wires);
}

//! calculate the force distribution with the selected method
bool CForceDistribution::getDistribution(MatrixXd& w, MatrixXd& f_min, MatrixXd& f_max, MatrixXd& f_wires)
{
	switch (forceDistributionMethod)
	{
	case emForceDistributionMethod::bruteForce:
		if (!getDistributionBruteForce(w,f_wires))
			return false;
		break;
	case emForceDistributionMethod::dykstra:
		if (!getDistributionDykstra(w, f_min, f_max, f_wires))
			return false;
		break;
	case emForceDistributionMethod::weightedSum:
		if (!getDistributionWeightedSum(w, f_wires))
			return false;
		break;
	case emForceDistributionMethod::advancedClosedForm:
		if (!getDistributionAdvancedClosedForm(AT,w,f_min,f_max,f_wires))
			return false;
		break;
	case emForceDistributionMethod::closedFormEnergyEfficient:
		if (!getDistributionClosedFormEnergyEfficient(w,f_min, f_max, f_wires))
			return false;
		break;
	case emForceDistributionMethod::puncture:
		if (!getDistributionPuncture(w,f_min, f_max, f_wires))
			return false;
		break;
	case emForceDistributionMethod::advancedClosedFormEnergyEfficient:
		if (!getDistributionAdvancedClosedFormEnergyEfficient(AT,w,f_min,f_max,f_wires))
			return false;
		break;
	case emForceDistributionMethod::quadraticProgramming:
		if (!getDistributionQuadraticProgramming(w,f_min, f_max, f_wires))
			return false;
		break;
	case emForceDistributionMethod::barycenter:
		int nov; double area;
		if (!getDistributionBarycenter(w,f_min,f_max,f_wires,nov,area))
			return false;
		break;
	case emForceDistributionMethod::closedForm:
	default:
		if (!getDistributionClosedForm(w,f_min, f_max, f_wires))
			return false;
		break;
	}
	return true;
}

/*! The core implementation to compute the closed-form solution in the 
 *  redundant case. To allow for a very high flexibility of usage event
 *  the current structure matrix is a parameter in this function call.
 *  This function does not reply on the settings of the current robot but
 *  only on the dimension deduced from the passed structure matrix AT.
 *  Therefore, this functions can serve as a subroutine in more complex
 *  computation schemes.
 *
 *  \param AT [in] the structure matrix to be used
 *  \param w [in] the wrench to be applied to the platform
 *  \param f_ref [in] reference minimal force for each cable
 *  \param f_wires [out] contains the computed distribution, if the function 
 *  	    returns true; otherwise undefined
 *  \return true, if computation was successfull. 
 */
bool CForceDistribution::getDistributionClosedFormCore(MatrixXd& AT, MatrixXd& w, MatrixXd& f_ref, MatrixXd& f_wires)
{
	if (AT.cols()<AT.rows())
		return false;
	else if (AT.cols() == AT.rows())
	{	
		// solve, if number of wires equals the degree of freedom
		// solve AT*f=-w
		f_wires = AT.lu().solve(-w);
		return true;
	}

	// general solver for redundant robots with more wires than degrees-of-freedom
	MatrixXd A = AT.transpose();
	MatrixXd ATA = AT * A;

	// we will now select the solver based on a state variable of this class
	if (linearSolver == emLinearSolverMethod::lu)
		// new implementation; from the analysis of the formula, we can expand the
		// pseudo-inverse product with the bracket and replace the inversion with a 
		// linear solver (which is faster than explictly computing the inverse)
		f_wires = f_ref - A * ATA.lu().solve(w+AT*f_ref);
	else if (linearSolver == emLinearSolverMethod::llt)
		f_wires = f_ref - A * ATA.llt().solve(w+AT*f_ref);
	else if (linearSolver == emLinearSolverMethod::ldlt)
		f_wires = f_ref - A * ATA.ldlt().solve(w+AT*f_ref);
	else if (linearSolver == emLinearSolverMethod::householderQr)
		f_wires = f_ref - AT.householderQr().solve(w+AT*f_ref);
	else if (linearSolver == emLinearSolverMethod::explicitPseudoinverse)
	{	// the old implementation: compute the pseudo-inverse explicitly
		// compute the pseudo-inverse of AT from A * (AT*A)^-1
		MatrixXd Ai = A * ATA.inverse();
		// compute optimal force distribution by the pure formula as given in the paper
		f_wires = f_ref - Ai*(w + AT*f_ref);
	} 
	else if (linearSolver == emLinearSolverMethod::jacobiSvd)
		f_wires = f_ref - AT.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(w+AT*f_ref);
			
	return true;
}

/*! Compute a force distribution for the actual structure matrix AT.
 *  If there are more wires than dof the formula
 *		f_wires = f_ref - A^+T*(w + AT*f_ref)
 *  is used where f_ref is a given reference force distribution to be 
 *  approximated with f_wires, and A^+T is the Moore-Penrose generalized 
 *  matrix inverse of the stucture matrix AT. 
 *
 *  The current implementation supports six different matrix decompositions to 
 *  compute the pseudo inverse. The different decompositions differ in 
 *  computation time and robustness. Use setLinearSolver to select the method 
 *  to be used in this function. Note, that the closed-form formula may fail 
 *  to find solution although solutions exists. The lack of convergency is 
 *  inherent to the assumptions used to derive the formula (and not a 
 *  consequence of using an instable numerical method to solve the equation).  
 * 
 *  If the number of wires equals the dof then the linear system
 *  f_wires = AT^-1 * w
 *  is solved. The current implementation returns false, if there are more dof 
 *  than wires or in other words does not support such robots.
 *  
 *  \param w [in] the wrench to be applied to the platform
 *  \param f_min [in] reference minimal force for each cable
 *  \param f_max [in] reference maximum force for each cable
 *  \param f_wires [out] contains the computed distribution, if the function 
 *  	    returns true; otherwise undefined
 *  \return true, if computation was successfull. returnt true does not 
 *          indicate that the force distribtion is feasible, i.e. in 
 *		    f_wires in [fmin, fmax]
 *  \todo use getDistributionClosedFormCore to compute the distribution.
 *  \todo use the decomposition method in linearSolver also for now==dof
 */
bool CForceDistribution::getDistributionClosedForm(MatrixXd& w, MatrixXd& f_min, MatrixXd& f_max, MatrixXd& f_wires)
{
	const int dof = pRobot->getDof();
	const int now = pRobot->getNow();
	MatrixXd f_ref(now,1);
	f_ref = (f_min+f_max)/2.0;

	if (dof > now) // the current implementation does not support under-actuated cable robots
		return false;

	return getDistributionClosedFormCore(AT,w,f_ref,f_wires);
}


/*! calculate the force distribution with the advanced closed-form method
 */
bool CForceDistribution::getDistributionAdvancedClosedForm(MatrixXd& AT, MatrixXd& w, MatrixXd& f_min, MatrixXd& f_max, MatrixXd& f_wires)
{
	const int now = AT.cols();
	MatrixXd f_ref = 0.5*(f_min+f_max);

	// 1) compute the first guess using the known closed-form formula
	if (!getDistributionClosedFormCore(AT, w,f_ref,f_wires))
		return false;

	// 2) check if vector f_wires to violate the force limits and
	// find the "larges" violation. 
	int index = -1;
	double violation = 0;
	for (int i=0; i<f_wires.size(); i++)
	{
		if (f_wires(i)<f_min(i))
		{
			if (violation<f_min(i)-f_wires(i))
			{
				violation = max(violation,f_min(i)-f_wires(i));
				index = i;	// we use the normal index to indicate a minimum and index + now to indicate a maximum
			}
		}
		else if (f_wires(i)>f_max(i))
		{
			if (violation<f_wires(i)-f_max(i))
			{	
				violation = max(violation,f_wires(i)-f_max(i));
				index = i+now;
			}
		}
	}

	if (index == -1)	
		return true;		// we already found the desired solution

	// is further reduction possible
	if (AT.cols()==AT.rows())
		return false;

	// 3) solution is not yet feasible, we set this violation to the maximum feasible value, and reduce the structure matrix
	int j = 0;
	MatrixXd fminred(now-1,1), fmaxred(now-1,1);

	MatrixXd ATred(AT.rows(),AT.cols()-1);
	for (int i=0; i<now; i++)
		if (i == index || i+now==index)
			continue;	// skip this column
		else
		{
			fminred(j) = f_min(i);
			fmaxred(j) = f_max(i);
			ATred.col(j++) = AT.col(i);
		}
	
	MatrixXd wred=w;
	// modify w by the effect of the constant cable force
	if (index < now)		
		// minimum force
		wred+=AT.col(index)*f_min(index);
	else
		// maximum force
		wred+=AT.col(index-now)*f_max(index-now);

	// make a copy of the resulting cable forces
	MatrixXd f_wires_red = f_wires;

	// 4) now we perform a recursive call to do the same with the reduced matrix ATred
	bool res = getDistributionAdvancedClosedForm(ATred, wred,fminred,fmaxred,f_wires_red);

	{	// we have to substitute the computed force back into the vector
		j = 0;
		for (int i=0; i<now; i++)
			if (i == index)
				f_wires(i) = f_min(i);
			else if (i+now == index)
				f_wires(i) = f_max(i);
			else
				f_wires(i) = f_wires_red(j++);
	}

	return res;
}


/*! calculate the force distribution with the advanced closed-form method and optimization regarding energy efficiency
 * it reduces f_ref in a way, that at least one cable has the minimum cable force
 */
bool CForceDistribution::getDistributionAdvancedClosedFormEnergyEfficient(MatrixXd& AT, MatrixXd& w, MatrixXd& f_min, MatrixXd& f_max, MatrixXd& f_wires)
{
	const int now = AT.cols();
	MatrixXd f_ref = 0.5*(f_min+f_max);

	// 1) compute the first guess using the known closed-form formula
	
	if (pRobot->getNow() == now) // use the closed form energy efficient implementation ONLY in the first iteration
	{
		if (!getDistributionClosedFormEnergyEfficient(w, f_min, f_max, f_wires))
			return false;
	}
	else
	{
		if (!getDistributionClosedFormCore(AT, w,f_ref,f_wires))
			return false;
	}

	// 2) check if vector f_wires to violate the force limites and
	// find the "larges" violation. 
	int index = -1;
	double violation = 0;
	for (int i=0; i<f_wires.size(); i++)
	{
		if (f_wires(i) < f_min(i))
		{
			if (violation < f_min(i)-f_wires(i))
			{
				violation = max(violation,f_min(i)-f_wires(i));
				index = i;	// we use the normal index to indicate a minimum and index + now to indicate a maximum
			}
		}
		else if (f_wires(i) > f_max(i))
		{
			if (violation<f_wires(i)-f_max(i))
			{	
				violation = max(violation,f_wires(i)-f_max(i));
				index = i+now;
			}
		}
	}
	if (index == -1)	
		return true;		// we already found the desired solution

	// is further reduction possible
	if (AT.cols() == AT.rows())
		return false;

	// 3) solution is not yet feasible, we set this violation to the maximum feasible value, and reduce the structure matrix
	int j = 0;
	MatrixXd fminred(now-1,1), fmaxred(now-1,1);

	MatrixXd ATred(AT.rows(),AT.cols()-1);
	for (int i=0; i<now; i++)
		if (i == index || i+now == index)
			continue;	// skip this column
		else
		{
			fminred(j) = f_min(i);
			fmaxred(j) = f_max(i);
			ATred.col(j++) = AT.col(i);
		}
	
	MatrixXd wred=w;
	// modify w by the effect of the constant cable force
	if (index < now)		
		// minimum force
		wred += AT.col(index)*f_min(index);
	else
		// maximum force
		wred += AT.col(index-now)*f_max(index-now);

	// make a copy of the resulting cable forces
	MatrixXd f_wires_red = f_wires;

	// 4) now we perform a recursive call to do the same with the reduced matrix ATred
	bool res = getDistributionAdvancedClosedFormEnergyEfficient(ATred, wred,fminred,fmaxred,f_wires_red);

	{	// we have to substitue the computed force back into the vector
		j = 0;
		for (int i=0; i<now; i++)
			if (i == index)
				f_wires(i) = f_min(i);
			else if (i+now == index)
				f_wires(i) = f_max(i);
			else
				f_wires(i) = f_wires_red(j++);
	}

	return res;
}


/*! Compute a energy efficient force distribtion for the actual struture matrix AT
 * it reduces f_ref in a way, that at least one cable has the minimum cable force
 *  Beta-Implementation!!
 *  
 *  \param w [in] the wrench to be applied to the platform
 *  \param f_min [in] reference minimal force for each cable
 *  \param f_max [in] reference maximum force for each cable
 *  \param f_wires [out] contains the computed distribution, if the function 
 *  	    returns true; otherwise undefined
 *  \return true, if computation was successfull. returnt true does not 
 *          indicate that the force distribtion is feasible, i.e. in 
 *		    f_wires in [fmin, fmax]
 *  \todo use getDistributionClosedFormCore to compute the distribution.
 *  \todo use the decomposition method in linearSolver also for now==dof
 */
bool CForceDistribution::getDistributionClosedFormEnergyEfficient(MatrixXd& w, MatrixXd& f_min, MatrixXd& f_max, MatrixXd& f_wires)
{
	const int dof = pRobot->getDof();
	const int now = pRobot->getNow();
	MatrixXd f_ref(now,1);
	f_ref = (f_min+f_max)/2.0;
	const double eps = 0.0000001;

	if (dof > now) // the current implementation does not support under-actuated cable robots
		return false;

	getDistributionClosedFormCore(AT,w,f_ref,f_wires);

	MatrixXd A = AT.transpose();
	MatrixXd ATA = AT * A;
	MatrixXd Ai = A * ATA.inverse();
	
	MatrixXd gradient = MatrixXd::Ones(pRobot->getNow(),1) ;

	gradient = Ai*AT* MatrixXd::Ones(pRobot->getNow(),1);

	if (f_wires.minCoeff() > f_min(0))
	{
		MatrixXd df = f_wires-f_min; 

		double f_min_ist;
		int i_min_ist = 0;
			

		f_min_ist = df(0)/(1.0-gradient(0));

		for (int i=1; i<f_wires.size(); i++)
		{
			if (df(i)/(1.0-gradient(i)) < f_min_ist)
			{
				f_min_ist = df(i)/(1.0-gradient(i));
				i_min_ist = i;
			}
		}
		if (f_min_ist > 0) // ensure, that the f_ref is decreased with the algorithm
			f_ref = MatrixXd::Constant(pRobot->getNow(),1,f_ref(0)-(df(i_min_ist))/(1.0-gradient(i_min_ist)+eps));
	}

	return getDistributionClosedFormCore(AT,w,f_ref,f_wires);
}


/*! Compute a force distribtion for the actual struture matrix AT.
 *   according to Puncture Method from University Duisburg 2013
 *   Beta-Implementation!!
 *  
 *  \param w [in] the wrench to be applied to the platform
 *  \param f_min [in] reference minimal force for each cable
 *  \param f_max [in] reference maximum force for each cable
 *  \param f_wires [out] contains the computed distribution, if the function 
 *  	    returns true; otherwise undefined
 *  \return true, if computation was successfull. return true does not 
 *          indicate that the force distribtion is feasible, i.e. in 
 *		    f_wires in [fmin, fmax]
 */
bool CForceDistribution::getDistributionPuncture(MatrixXd& w, MatrixXd& f_min, MatrixXd& f_max, MatrixXd& f_wires)
{
	const int dof = pRobot->getDof();
	const int now = pRobot->getNow();
	const double eps = 0.000001;
	
	MatrixXd A = AT.transpose();
	MatrixXd ATA = AT * A;
	MatrixXd Ai = A * ATA.inverse(); // compute the pseudo-inverse of AT from A * (AT*A)^-1
	
	MatrixXd f_ref(now,1);
	MatrixXd f_0(now,1);

	for (int i=0; i<f_wires.size(); i++)
	{
		f_min(i)=f_min(i)+eps;
		f_max(i)=f_max(i)-eps;

	}

	f_ref=(f_min+f_max)/2.0;
	
	// use the advanced closed form
	getDistributionAdvancedClosedForm( AT, w, f_min,f_max, f_wires);
	//	return false;
	f_0 = - Ai*w; // cable forces which are needed to compensate the wrench
	double df_;
	
	MatrixXd df = f_wires-f_0; // slope of the straight line
	
	double d_min_;
	double d_max_;

	MatrixXd d_min(now,1);

	// identification of the scale factor df_ for the straight line: f_wires= f_0 + df_*df
	for (int i=0; i<f_wires.size(); i++)
	{
		d_min_=(f_min(i)-f_0(i))/df(i);
		d_max_=(f_max(i)-f_0(i))/df(i);

		if (d_min_<d_max_)
		{
			d_min(i)=d_min_;
		}
		else
		{
			d_min(i)=d_max_;
		}
	}

	df_=d_min.maxCoeff();

	f_wires = f_0 + df_*df;
	
	return true;
}

#ifdef WIRELIB_HAS_CVXGEN
// global variables for QP-algorithm
// \todo Embed these variables to some kind of scope removing them from the global scope
Vars vars;
Params params;
Workspace work;
Settings settings;


/*! calculate a force distrubution using the Quadratic Programming Optimization 
 *  method, aiming at achieving arbitrary cable forces the tension level can be 
 *  chosen stepless by c. QP algorithm will find the cable force distribution with 
 *  minimal euclidean distance to c 
 *  Quadratic programming algorithm is generated with cvxgen: 
 *  http://cvxgen.com/docs/c_interface.html
 *  The algorithm minimizes the function ((x-c)^T Q (x-c)), solves the equation 
 *  A x = b and fulfills the constraints x>=xmin and x<=x_max
 */
bool CForceDistribution::getDistributionQuadraticProgramming(MatrixXd& w, MatrixXd& f_min, MatrixXd& f_max, MatrixXd& f_wires)
{
	const int dof = pRobot->getDof();
	const int now = pRobot->getNow();
	const double eps_initial = 0.00001; // f_min+eps_initial and f_max-eps_initial   and break condition regarding difference between the two projections [N]
	const double eps_wrench = 0.000001; // verify, if f_wires solves the structure equation
	
//	MatrixXd f_ref = MatrixXd::Zero(now,1); // f_ref is copied to c. Hint: This should be an input parameter of this function, as it allows to control the tension level
	
	if (dof!=6 || now!=8) // The QP-Implementation is designed via code generation for 6 DOF and 8 Cables
		return false;
	
	f_wires.resize(8,1);

	int i, j;
	long num_iters; // number of iterations

	// Q = identity matrix 8x8, but parametrized as diagonal matrix with only 8 elements for perfomance reasons
	for (i=0; i<8; i++)
		params.Q[i] = 1;

	// AT
	for (i=0; i<6; i++)
		for (j=0; j<8; j++)
		params.A[i+j*6] = AT(i,j);

	// b
	for (i=0; i<6; i++)
		params.b[i] = -w(i);

	// c
	for (i=0; i<8; i++)
		params.c[i] = f_ref_QP;

	// lb
	for (i=0; i<8; i++)
		params.x_min[i]=f_min(i)+eps_initial;
  
	//ub
	for (i=0; i<8; i++)
		params.x_max[i]=f_max(i)-eps_initial;

	//set_defaults(); moved to the constructor of CForceDistribution
	setup_indexing();

	settings.verbose = 0; // disable the output, otherwise the console dialog overflows and WireCenter freezes
	settings.debug = 0;

	num_iters = solve(); // call the QP algorithm
    // Recommended: check work.converged == 1.

	QPconverged = work.converged;
	QPIterations = num_iters;

	// copy the result to f_wires
	for (i=0; i<8; i++)
		f_wires(i)=vars.x[i];

	if ((AT*f_wires+w).lpNorm<Eigen::Infinity>() > eps_wrench) // ensure, that f_wires solves the structure equation
		for (i=0; i<8; i++)
			f_wires(i) = -1;

	return rateDistributionFeasible(f_wires)>0;
}

//! set special parameters of the QP method (default parameter already deliver good results)
bool CForceDistribution::setQPparameters(int max_iters, double resid_tol, double eps, int refine_steps, double kkt_reg)
{
	set_defaults(); 

	// if -1, the default value remains
	if (max_iters != -1)
		settings.max_iters = max_iters;
	if (resid_tol != -1)
		settings.resid_tol = resid_tol;
	if (eps != -1)
		settings.eps = eps;
	if (refine_steps != -1)
		settings.refine_steps = refine_steps;
	if (kkt_reg != -1)
		settings.kkt_reg = kkt_reg;

	return true;
}

#else

// these are dummy implementations to run the program if the convex optimization code is not availabe 
bool CForceDistribution::getDistributionQuadraticProgramming(MatrixXd& w, MatrixXd& f_min, MatrixXd& f_max, MatrixXd& f_wires)
	{ return false; }

bool CForceDistribution::setQPparameters(int max_iters, double resid_tol, double eps, int refine_steps, double kkt_reg)
	{ return false; }

#endif // WIRELIB_HAS_CVXGEN


/*! Compute a force distribution by a "brute-force" method. The performance will be very
 *  bad but hopefully we do not miss a solution if it exists.
 *  project each of the 2^now corners of the feasible force cube onto the kernel of AT
 *  and test if the result is feasible... very slow but for sure this approach does not
 *  miss solutions if they exist.
 *
 *  \warning The bit-wise coding for the corner support only 32 cable (due to int32). 
 *           At least we have to report an error for m>32. Clearly, we run into 
 *           problems earlier due to excessive memory and computational time.
 *  
 */
bool CForceDistribution::getDistributionBruteForce(MatrixXd& w, MatrixXd& f_wires)
{
	if (pRobot->getNow()>31)
		return false;

	try {
		const int dof = pRobot->getDof();
		const int now = pRobot->getNow();

		MatrixXd ATA(dof,dof);
		MatrixXd A(now,dof);
		MatrixXd Ai(dof,now);
		MatrixXd f_mid = MatrixXd::Ones(now,1);

		MatrixXd Aiw(now,1);
		MatrixXd AiAT = MatrixXd::Identity(now,now);

		// compute the pseudo inverse; only needed once for all corners
		A = AT.transpose();
		ATA = AT*A;
		Ai = A * ATA.inverse();
		Aiw = Ai*w;
		AiAT -= Ai*AT; // identity-Ai*AT

		for (int i=0; i<(1<<now); i++)
		{
			// the bit pattern of j is used to code min/max setting in f_mid
			for (int j=0; j<now; j++)
				f_mid(j) = (i & (1<<j)) ? pRobot->fmax : pRobot->fmin;

			// project the corner onto the kernel
			f_wires = AiAT*f_mid - Aiw;

			if (rateDistributionInfinity(f_wires) > 0)
			{
				return true;
			}
		}
		return true;
	}
	catch (...)
	{
		// an exception occured (probably while inverting the matrix)
		return false;
	}
}

/*! Compute a force distribution with the Dykstra-Algorithm.
 *  The method is proposed in the Paper:
 *  "Analysis of bounded cable tensions in cable-actuated parallel manipulators"
 *  Mahir Hassan and Amir Khajepour, October 2011
 *  
 *  \todo make the local settings global configuration variables of the class
 */
bool CForceDistribution::getDistributionDykstra(MatrixXd& w, MatrixXd& f_min, MatrixXd& f_max, MatrixXd& f_wires)
{
	const int i_max = 5000; // maximum number of iterations
	const double eps_initial = 0.001; // f_min+eps_initial and f_max-eps_initial   and break condition regarding difference between the two projections [N]
	const double eps_break = 0.00001; // break condition regarding convergence [N]
	dykstraIterations = 0;

	try {
		const int dof = pRobot->getDof();
		const int now = pRobot->getNow();

		MatrixXd ATA(dof,dof);
		MatrixXd A(now,dof);
		MatrixXd Ai(dof,now);
		MatrixXd Aiw(now,1);
		MatrixXd AiAT = MatrixXd::Identity(now,now);

		MatrixXd projA = MatrixXd::Zero(now,1); // projection on the nullspace, translated by Ai*w
		MatrixXd projAold = MatrixXd::Zero(now,1); 
		MatrixXd projC = MatrixXd::Zero(now,1); // projection on the hyper-rectangle of the force limits
		MatrixXd projCold = MatrixXd::Zero(now,1); 

		MatrixXd tauMin = MatrixXd(f_min); 
		MatrixXd tauMax = MatrixXd(f_max);
		MatrixXd f_eps = MatrixXd::Constant(pRobot->getNow(),1,eps_initial);
		tauMin = tauMin + f_eps;
		tauMax = tauMax - f_eps;

		// compute the pseudo inverse 
		A = AT.transpose();
		ATA = AT*A;
		Ai = A * ATA.inverse();
		Aiw = Ai*w;
		AiAT -= Ai*AT; // identity-Ai*AT

		projA = projC = (tauMin+tauMax)/2.0 - Ai*(w + AT*(tauMin+tauMax)/2.0); // start value is the closed-form-solution
		
		int i;	
		for (i=0; i<i_max; i++)
		{	
			projAold = projA;
			// projection on A (nullspace of A, translated by Ai*w)
			projA = AiAT*projC-Aiw;

			projCold = projC;
			// projection on C (hyper-rectangle of the force limits f_min and f_max)
			for (int j=0; j<now; j++)
			{
				projC(j) = max(projA(j),tauMin(j));
				projC(j) = min(projC(j),tauMax(j));
			}

			// break conditions
			if ((projC-projCold).lpNorm<Eigen::Infinity>() < eps_break)
				if ((projA-projAold).lpNorm<Eigen::Infinity>() < eps_break)
				{
					break;
				}
			if ((projA-projC).lpNorm<Eigen::Infinity>()<eps_initial)
			{
				break;
			}
		}
		dykstraIterations = i;
		f_wires = projA;
		
		return true;
	}
	catch (...)
	{
		// an exception occured (probably while inverting the matrix)
		return false;
	}
}

/*! This is an experimental refactoring of the brute force method, where we try to 
 *  use a more elaborated method to figure out which corners need to be checked.
 *  The bases of this implementation is just a copy of getDistribution_BruteForce
 */
bool CForceDistribution::getDistributionBruteForceEx(MatrixXd& w, MatrixXd& f_wires)
{
	try {
		const int dof = pRobot->getDof();
		const int now = pRobot->getNow();

		MatrixXd ATA(dof,dof);						// intermediate results
		MatrixXd A(now,dof);							// intermediate results
		MatrixXd Ai(dof,now);						// intermediate results
		MatrixXd f_mid = MatrixXd::Ones(now,1);	// the middle of the force cube
		MatrixXd f_Ei = MatrixXd::Ones(now,1);			// the corners of the force cube
		MatrixXd f_opt = MatrixXd::Ones(now,1);		// the optimal solution w.r.t. to the closed form solution

		// initialize f_mid to be the center of the feasible force cube
		for (int i=0; i<now; i++)
			f_mid(i)=(pRobot->fmax+pRobot->fmin)/2;

		// compute the pseudo inverse; only needed once for all corners
		A = AT.transpose();
		ATA = AT * A;
		// use version without screen output instead of ATA.invert()
		Ai = A*ATA.inverse();

		// compute the optimal solution
		f_opt = f_mid - Ai*(w + AT*f_mid);

		// loop through all corners of the force cube
		for (int i=0; i<(1<<now); i++)
		{
			// the bit pattern of j is used to code min/max setting in f_Ei
			for (int j = 0; j<now; j++)
				f_Ei(j) = (i & (1<<j)) ? pRobot->fmax : pRobot->fmin;

			// NEW: compute the angle between f_mid->f_opt and f_opt->f_Ei
			// only if this angle is >90° we can expect the corner to be useful 
			// (at least our new hypothesis says so :-) )
			MatrixXd f1 = MatrixXd::Zero(now,1);	
			MatrixXd f2 = MatrixXd::Zero(now,1);
			f1 = (f_opt-f_mid);
			f2 = (f_Ei-f_opt);
			double cosphi = 0;
			for (int k=0; k<now; k++)
				cosphi += f1(k)*f2(k);

			// project the corner onto the kernel
			f_wires = f_Ei - Ai*(w + AT*f_Ei);

			if (rateDistributionInfinity(f_wires)>0)
			{
				printf("%i: phi: %f3.2 ok\n",i,cosphi);
//				return true;
			}
			else
				printf("%i: phi: %f3.2 nok\n",i,cosphi);

		}
		return true;
	}
	catch (...)
	{
		// an exception occured (probably while inverting the matrix)
		return false;
	}
}

/*! compute all vertices of the solution set by intersecting all permutations of
 *  now-dof force limit planes (e.g. f_i = fmin or fmax) with the kernal space of
 *  the structure matrix. The dimension of the matrix f_wires depends on the number
 *  of solutions found. 
 *  this is a low level routine used e.g. by WeightedSum method or just for development
 *  of other force distribution stategies.
 *  \param w [in] the load applied to the robot
 *  \param f_wires [out] a matrix with now rows and n columns where the number of columns
 *         is the number of vertices found. the matrix can be empty, if no vertex is found
 *  \return true, if computation was successful otherwise false.
 *
 *  \todo There are some issues with this function that produce more vertices than
 *        actually exist. Firstly, some identical vertices are found that are not
 *        eliminated. Secondly, some vertices with a distance in the range of the machine
 *        accuracy are separately found but virtually correspond to the same vertex.
 *        This vertices are handled by the post-processing step; however, correctness of this 
 *        part of the algorithm is not sufficiently analyzed.
 */
bool CForceDistribution::getDistributionAllVertices(MatrixXd& w, MatrixXd& f_wires)
{
	const int dof = pRobot->getDof();
	const int now = pRobot->getNow();
	const int r = now-dof;

	// we only deal with actuator redundancy
	if (dof >= now)	
		return false;

	vector<MatrixXd> Fvertex;

	// choose k from n realized with a permutation of zero and ones
	std::vector<int> ints;
	for (int i = pRobot->getNow()-1; i>=0; i--)
		ints.push_back( (i < dof) ? 1 : 0);

	int k = 0;
	MatrixXd Ap(dof, dof);
	MatrixXd As(dof, r);
	int nperm = 1 << r;	// 2^r permutations of fmin/fmax possible
	do	// loop through all permutations "ints" of the columns of AT to seperate the values into Ap and As
	{
		// seperate into (dof x dof) matrix and (dof x r) matrix Ap and As
		int p = 0, s = 0;
		for (int i=0; i<now; i++)
			if (ints[i] == 1)
				Ap.col(p++) = AT.col(i);
			else
				As.col(s++) = AT.col(i);
			
		// invert the primary matrix Ap; 
		Eigen::FullPivLU<MatrixXd> LU;
		LU.compute(Ap);
		// firstly check for singular matrix
		if (!LU.isInvertible())
			continue;
		MatrixXd Apinv = LU.inverse();
			
		// loop through all permutations of the cable forces
		for (int i=0; i<nperm; i++)
		{
			//! compute modified w_m = -w - k * As(i) with k=+/- 1
			MatrixXd w_m = -w;
			for (int j = 0; j<r; j++)
				if (i & (1 << j))
					w_m -= As.col(j)*pRobot->fmin;
				else
					w_m -= As.col(j)*pRobot->fmax;
				
			// solve the system
			MatrixXd fs = Apinv * w_m;

			// is f feasible?
			if (rateDistributionFeasible(fs) > 0)
			{
				// restore the original vector (i.e. insert the fmin,fmax into fs where it was dropped)
				MatrixXd f(now,1);
				int p = 0, s = 0;
				for (int k=0; k<now; k++)
					if (ints[k] == 1)
						f(k) = fs(p++);
					else
						f(k) =  (i & (1 << s++))? pRobot->fmin : pRobot->fmax ;
				Fvertex.push_back(f);
			}
		}
	}
	while (std::next_permutation(ints.begin(),ints.end())); // STL-function
	
	// now we have an array with all solution; we have to copy it to the matrix f_wires to return the results
	if (Fvertex.size() == 0)
	{
		f_wires.resize(0,0);
		return false;
	}

	// the vertices in Fvertex may contain doublets and needs to be checked 
	for (unsigned int i=0; i<Fvertex.size()-1; i++)
		for (unsigned int j=i+1; j<Fvertex.size(); j++)
		{
			// the current vertex j is too close to i, we drop j
			if ((Fvertex[i]-Fvertex[j]).norm() < 1e-10 )
				Fvertex.erase(Fvertex.begin()+j--);			// decrease j by j-- after dropping the vector as we have to repeat this step for the index
		}

	f_wires.resize(now,Fvertex.size());
	for (unsigned int i=0; i<Fvertex.size(); i++)
		f_wires.col(i) = Fvertex[i];
	return true;
}

//! experimental force distribution method that computes all vertices of the solution set.
//! then, the average value is choosen as force distribution.
//! this methed should find a force distribution whenever it exists. It is rather slow
//! for higher redundancy due to the quickly increasing number of permutations to compute.
//! \param w [in] The load to be applied to the platform
//! \param f_wires [out] on output the determined force distribution
//! \return true, if computation was successful, otherwise false
bool CForceDistribution::getDistributionWeightedSum(MatrixXd& w, MatrixXd& f_wires)
{
	MatrixXd f;
	if (!getDistributionAllVertices(w,f))
		return false;

	f_wires.resize(AT.cols(),1);
	for (int i=0; i<f.rows(); i++)
		f_wires(i) = f.row(i).mean();

	return true;
}


/*! This function computes the force distribution at the barycenter of the 
 *  solution space which was proposed by Mikelsons et al. as force distribution.
 *  According to the special nature of the method, it is only applicable in the 
 *  case m=n+2, i.e. for a degree-of-redundency of two.
 *  Additionally, the number of vertices and the area of the solution space is computed
 *  \param w [in] the load applied to the robot
 *  \param f_min [in] a vector with the respective minimum cable forces
 *  \param f_max [in] a vector with the respective maximum cable forces
 *  \param f_wires [out] a vector with the determined force distribtion
 *  \param numberOfVertices [out] the number of vertices of the convex polyope of the solution space S
 *  \param numberOfVertices [out] the area of the convex polyope of the solution space S
 *  \return true, if computation was successful otherwise false.
 */
bool CForceDistribution::getDistributionBarycenter(MatrixXd& w, MatrixXd& f_min, MatrixXd& f_max, MatrixXd& f_wires, int& numberOfVertices, double& areaSolutionSpace)
{
	// init the values with meaningful default values
	areaSolutionSpace = 0;
	numberOfVertices = 0;
	MatrixXd vertices;
	f_wires = MatrixXd::Zero(pRobot->getNow(),1);	// provide a zero vector by default or in case no distribution can be found

	// test if	the robot has a matching number of cables and dof
	if (pRobot->getNow() != 2+pRobot->getDof())
		return false;

	// compute all vertices, return false if this computation of vertices failed
	if (!getDistributionAllVertices(w, vertices))
		return false;

	numberOfVertices = vertices.cols();
	// the computation was successful but there are no solutions
	if (numberOfVertices == 0)
		return true;

	// We handle the case with only one vertex here; unlikely but
	// the rest will fail if there is only one vertex.
	if (numberOfVertices == 1)
	{
		// the only one vertex is the sough solution
		f_wires = vertices.col(0);
		return true;
	}

	// two vertices is also special as the area is zero but we can compute a barycenter
	if (numberOfVertices == 2)
	{
		f_wires = 0.5*(vertices.col(0)+vertices.col(1));
		areaSolutionSpace = 0;
		return true;
	}

	MatrixXd f_m = MatrixXd::Zero(pRobot->getNow(), 1);	// init mean force vector with zeros
	// compute the arithmetic center (mean value) of the vertices which may not to be confused with the barycenter of the polytope
	for (int i=0; i<numberOfVertices; i++)
		f_m += vertices.col(i);
	f_m /= (double)numberOfVertices;

	// compute the vectors from the center to the vertices
	MatrixXd f_v(vertices);
	for (int i=0; i<numberOfVertices; i++)
	{
		f_v.col(i) -= f_m;
		// only for debug and testing
		if (f_v.col(i).norm()<1e-6)
			cout << "relative vertex is zero!\n"; 
	}

	// an improved way to sort the vertices is  "polar sorting" which is done be 
	// decomposing the vectors in a spanning base in the kernel space and to sort 
	// them by the absolute angle with respect to the first vector

	// compute the base vectors vA and vB of the kernel
	MatrixXd vA = f_v.col(0);
	vA.normalize();
	MatrixXd vB(pRobot->getNow(),1);
	for (int i=1; i<numberOfVertices; i++)
	{
		// is the second vector sufficietly different from the first one
		if (vA.col(0).dot(f_v.col(i)) / vA.col(0).norm() / f_v.col(i).norm()  < 0.99)
		{
			vB = f_v.col(i)-vA.col(0).dot(f_v.col(i))*vA.col(0);
			break;
		}
	}

	std::map<double, int> polarIndexing;
	for (int i=0; i<numberOfVertices; i++)
	{
		// compute the absolute angle w.r.t. the first vector
		double phi = atan2(vB.col(0).dot(f_v.col(i)),vA.col(0).dot(f_v.col(i)));
		// store the angle/index pair in the map
		polarIndexing[phi] = i;
	}

	// once the map is prepared, we have can loop through the map to get the 
	// indices in the order of ascending phi values. then we compute the area A
	// and the barycenter of the polytope. area computation in implemented in 
	// multiple steps although the area can be computed in one line.
	
	// get the first and last index
	int first = polarIndexing.begin()->second;
	int last = polarIndexing.rbegin()->second;

	double abn = f_v.col(first).norm() * f_v.col(last).norm();	// the product of the norms of the two vectors
	double abs = f_v.col(first).dot(f_v.col(last));				// the scalar product of the two vectors															
	double A = 0.5 * abn * sqrt (1 - min(abs*abs / abn / abn,1.0));
	f_wires = (f_v.col(first)+f_v.col(last))/3.0 * A;				// compute the weighted fraction contributed by this triangle to the barycenter

	// now we loop through the rest of the list
	for (auto itor=polarIndexing.begin(); itor!=polarIndexing.end(); itor++)
	{
		auto itor2 = itor; itor2++; 
		// skip the last item as it as no successor
		if (itor2 == polarIndexing.end()) 
			break;
		// get the actual and the next index
		int i = itor->second;
		int j = itor2->second;
		// add the area of the triangle f_m--f_v(i)--f_v(i+1)
		abn = f_v.col(i).norm() * f_v.col(j).norm();	// the product of the norms of the two vectors
		abs = f_v.col(i).dot(f_v.col(j));				// the scalar product of the two vectors															
		double A_i = 0.5 * abn * sqrt (1.0 - min(abs*abs / abn / abn,1.0));
		// add the fraction of the triangle for the barycenter
		f_wires += (f_v.col(i)+f_v.col(j))/3.0 * A_i;
		// add the fraction of the area of the triangle for the polytope
		A += A_i;
	}

	// after computing the weighted sum we have to normalize to the full area
	f_wires /= A;
	// and eventually translate this relative barycenter to the absolute forces
	f_wires += f_m;
	// finally assign the computed area of the solution space to the external variable
	areaSolutionSpace = A;

	return true;
}


/*! Test for each element in the vector f if fmin < f_i < fmax, i=1,..,dim(f)
 *  \return 1 if feasible, otherwise 0
 *  \todo port the computation of the quality indicies to eigen3's internal function for 
 *  computing norms (this also allies for the other rateDistributionX functions)
 */
double CForceDistribution::rateDistributionFeasible(const MatrixXd& f) const
{
	for (int i=0; i<f.size(); i++)
	{
		if (f(i) < pRobot->fmin)
			return 0;
		else if (f(i) > pRobot->fmax)
			return 0;
	}
	return 1.0;
}

/*! calculate a quality index for the force distribution; the distance between
 *  the f_mid and the given distribution f_wires is calculated and compared to
 *  the diameter of the cube of feasible wire forces, where the metric has its
 *  maximum value of 1 if f_wires matches exactly the center and is 0 on the center
 *  of the surfaces.
 *  \return the quality index
 */
double CForceDistribution::rateDistributionEuclidean(const MatrixXd& f_wires) const
{
	MatrixXd f_mid = MatrixXd::Ones(pRobot->getNow(),1) * 0.5*(pRobot->fmin+pRobot->fmax) ;
	double norm = (f_wires-f_mid).norm();

	return 1-(2*norm/(pRobot->fmax-pRobot->fmin));
}

/*! calculate a quality index for the force distribution; the distance between
 *  the f_mid and the given distribution f_wires is calculated with the infinity
 *  norm and compared to the diameter of the cube of feasible wire forces,
 *  where the metric has its maximum value of 1 if f_wires matches exactly the 
 *  center and is 0 on the whole surface.
 *  \return the quality index
 */
double CForceDistribution::rateDistributionInfinity(const MatrixXd& f_wires) const
{
	double f_mid = 0.5*(pRobot->fmin+pRobot->fmax);
	double norm = 0;
	// calculate the infinity norm of ||f_wires - f_mid ||_inf
	for (int i=0; i<pRobot->getNow(); i++)
		if (norm < fabs(f_wires(i)-f_mid))
			norm = fabs(f_wires(i)-f_mid);
	return 1-(2*norm/(pRobot->fmax-pRobot->fmin)); 
}

/*! driver function for testing if a pose belongs to the workspace 
 *  (w.r.t. existance of a feasible force distribution)
 *  \param r [in] position of the platform 
 *  \param R [in] orientation of the platform 
 *  \return index indicating the quality of the workspace
 */
double CForceDistribution::testWorkspace(const Vector3d& r, const Matrix3d& R)
{
	if (forceDistributionMethod == emForceDistributionMethod::wrenchClosure)
		return testForceClosure(r,R)?1.0:0.0;

	if (!getStructureMatrix(r,R))
		return 0.0;
	
	MatrixXd f_wires = MatrixXd::Zero(pRobot->getNow(),1);	// the force distribution in the wires
	MatrixXd w = MatrixXd::Zero(pRobot->getDof(),1);			// the wrench applied to the platform
	MatrixXd f_min = MatrixXd::Constant(pRobot->getNow(),1,pRobot->fmin);		// the minimum force vector
	MatrixXd f_max = MatrixXd::Constant(pRobot->getNow(),1,pRobot->fmax);		// the maximum force vector
	
	if (!setWrenchVector(w,f,tau))
		return -1;

	if (!getDistribution(w, f_min, f_max,f_wires))
		return 0;

	// evaluate the quality of the force distribution
	switch (evaluatorCriteria)
	{
	case emEvaluatorCriteria::euclidean: 
		return rateDistributionEuclidean(f_wires);
	case emEvaluatorCriteria::infinity:
		return rateDistributionInfinity(f_wires);
	case emEvaluatorCriteria::feasible:
	default:
		return rateDistributionFeasible(f_wires);
	}
}

/*! overloaded virtual function; the workspace existence test called from this 
 *  method will be used in generic evaluation such as workspace computation
 */
bool CForceDistribution::testPose(const Vector3d& r, const Matrix3d& R)
{
	return (testWorkspace(r,R)>condMin)?true:false;
}

/*! Calculate and print the force distribution for the given pose (r,R)
 *  \return true, after successful calcuation
 *  \warning this function only supports the motion pattern 3R3T
 */
bool CForceDistribution::printForceDistribution(const Vector3d& r, const Matrix3d& R)
{
	if (!getStructureMatrix(r,R))
		return false;
	if (pRobot->getMotionPattern() != CRobotData::MP3R3T)
		return false;

	MatrixXd f_wires = MatrixXd::Zero(pRobot->getNow(),1);		    // the force distribution in the wires
	MatrixXd w = MatrixXd::Zero(pRobot->getDof(),1);				// the wrench applied to the platform	

	// apply the load (only for the dof=6 case)
	if (!setWrenchVector(w,f,tau))
		return false;

	// calculate the force distribtion
	if (!getDistribution(w,pRobot->fmin, pRobot->fmax ,f_wires))
		return 0;

	cout << "f_wires="
		<< f_wires << endl;

	return true;
}

/*! determine the maximum forces (fmin,fmax) and torques (Taumin, Taumax) 
 *  that can be applied to the given pose (r,R)
 *  The implementation uses a line search method to change the external wrench w 
 *  starting from the origin in each direction until the binary search determined 
 *  the maximum value. As the wrench set is known to be convex the limit found 
 *  this way is correct if the current pose belongs the the wrench feasible 
 *  workspace for external wrench w=[0,0,0,0,0,0].
 *  The 12 vertices return by the methods form a convex hull which is a subset 
 *  of the actually available wrench set witch min/max forces/torques in each 
 *  coordinate direction. 
 *  \param r [in] the position of the robot to be analyzed
 *  \param R [in] the orientation of the robot to be analyzed
 *  \param fmin [out] the minimum forces of the wrench set
 *  \param Taumin [out] the minimum torque of the wrench set
 *  \param fmax [out] the maxium force of the wrench set
 *  \param Taumax [out] the maximum torque of the wrench set
 *  \return true, if successful
 */
bool CForceDistribution::getMaximumWrenchForPose(const Vector3d& r, const Matrix3d& R, Vector3d& fmin, Vector3d& Taumin,Vector3d& fmax, Vector3d& Taumax)
{
	// push the current values of f,Tau
	Vector3d f_old = f;
	Vector3d Tau_old = tau;
	bool erg = false;
	//! \todo the algorithm parameters, it would be nice to control them through the class...
	const double eps = 0.001;
	const double Lambda_min = 0;
	const double Lambda_max = pRobot->fmax*10;

	// firstly check if the pose belongs to the workspace, if not we are done
	if (!(testWorkspace(r,R) <= 0))
	{
		// now perform a line search to iterate the extremal wrenches
		// create a mapping to the values to calculated and changed
		double *pWrench[6] = { &f.x(), &f.y(), &f.z(), &tau.x(), &tau.y(), &tau.z()};
		double *pExtWrench[12] = { &fmax.x(), &fmax.y(), &fmax.z(), &Taumax.x(), &Taumax.y(), &Taumax.z(),
			&fmin.x(), &fmin.y(), &fmin.z(), &Taumin.x(), &Taumin.y(), &Taumin.z() };
		for (int i=0; i<12; i++)
		{
			tau = Tau_old;
			f = f_old;

			// now we do the calculation
			double lambda_min = Lambda_min;
			double lambda_max = Lambda_max;
			double lambda = Lambda_max;
			while (lambda_max-lambda_min > eps)
			{
				if (i < 6)
					*pWrench[i%6] = lambda;
				else 
					*pWrench[i%6] =- lambda;
				if (testWorkspace(r,R) > 0)
					lambda_min = lambda;
				else
					lambda_max = lambda;
				lambda = (lambda_min+lambda_max)/2;
			}
			if (i < 6)
				*pExtWrench[i] = lambda;
			else
				*pExtWrench[i] = -lambda;
		}
		erg = true;
	}

	// pop f,Tau
	f = f_old;
	tau = Tau_old;

	return erg;
}

//! test for wrench closure; however; we should generate several member function for this, since we
//! are developing serveral methods
//! - Verhoeven's kernel test (applicable for m=n+1)
//! - Sheng's coefficient test (applicable for m=n+1 and m=n+2)
//! - Pott's analytic-numeric algorihtm (applicable currently only for n=3, m=4)
bool CForceDistribution::testForceClosure(const Vector3d& r, const Matrix3d& R)
{
	// we use a specialized version for 1R2T and four cables; this implementation works for any robot
	// with m=n+1 (NoW=DoF+1)
	if (pRobot->getNow() == 4 && pRobot->getMotionPattern() == CRobotData::MP1R2T)
	{
		// we apply Verhoeven's kernel sign test
		if (!getStructureMatrix(r,R))
			return false;
		MatrixXd kern = AT.fullPivLu().kernel();
		if (kern.minCoeff() > 0.0 || kern.maxCoeff() < 0.0)
			return true;
		return false;
	}

	// the determinant methods requires some more parameterizations to be generalized
	// to other values than Now==7 
	if (pRobot->getNow() == 7 && pRobot->getMotionPattern() == CRobotData::MP3R3T)
	{
		if (!getStructureMatrix(r,R))
			return false;

		bool allPos = true;
		bool allNeg = true;
		for (int i=0; i<7; i++)
		{
			// drop i-th column and compute determinant
			MatrixXd ATL = AT;
			if (i != 6)  // for i = 6 all we have to do is resizing without the copy operation
				ATL.block(0,i,6,6-i) = ATL.block(0,i+1,6,6-i);
			ATL.conservativeResize(6,6);
			double det = ATL.determinant() ;
			if (i&1)
				det *= -1.0;
	//		cout << "  " << det;
			if (det < 0)
				allPos = false;
			if (det > 0)
				allNeg = false;
			// we can cancel the loop of both test failed before reaching the end of the for-loop
			if (!allPos && !allNeg)
				break;
		}
		return allPos||allNeg;
	}

	// if non of the above tests were executed, we did not find out anything
	return false;
}

/*! compute a parametric representation of the constant orientation wrench-closure workspace
 *  and print the parametric curves on the console.
 *  the current implementation is limited to the planar case with 4 cables and 1R2T; however,
 *  it should be possible to generalize both for more cables and other motion pattern. 
 *  status: work in progress; the current results seems to be wrong; however, the no error in 
 *  the basic approach could be identified yet. 
 */ 
bool CForceDistribution::calculateWorkspaceWrenchClosure()
{
	if (pRobot->getNow() != 4)
		return false;

	Matrix3d R = Matrix3d::Identity();
//	Vector3d r(0,0,0);

	// our local workspace model
	const int steps = 360;
	double deltaPhi = 2.0*MO_PI/steps;
	double ws[steps] = {};

//	printf("Ni: axx * x^2 + ax*x  +ayy * y^2 + ay*y + axy * x * y + a0 = 0\n");

	double a0[4], ax[4], axx[4], ay[4], ayy[4], axy[4];
	
	// compute the coefficients of the bounding polynomial
	// N1: axx * x^2 + ax*x  +ayy * y^2 + ay*y + axy * x * y + a0

	// compute a0 by computing the determinant for position (0,0)
	getNonNormalizedStructureMatrix(Vector3d(0,0,0), R);
	MatrixXd ATL;
	for (int i=0; i<4; i++)
	{
		ATL = AT;
		ATL.block(0,i,3,3-i) = ATL.block(0,i+1,3,3-i);
		ATL.conservativeResize(3,3);
		a0[i] = ATL.determinant() * ((i&1) ? 1.0 : -1.0);
	}

	double d1[4], d2[4];

	getNonNormalizedStructureMatrix(Vector3d(1,0,0), R);
	for (int i=0; i<4; i++)
	{
		// compute axx,ax with the ATL(1,0) and ATL(2,0); we should do this with 1,0 and -1,0 in the future
		ATL = AT;
		ATL.block(0,i,3,3-i) = ATL.block(0,i+1,3,3-i);
		ATL.conservativeResize(3,3);
		d1[i] = ATL.determinant()  * ((i&1) ? 1.0 : -1.0);
	}

	getNonNormalizedStructureMatrix(Vector3d(2,0,0), R);
	for (int i=0; i<4; i++)
	{
		ATL = AT;
		ATL.block(0,i,3,3-i) = ATL.block(0,i+1,3,3-i);
		ATL.conservativeResize(3,3);
		d2[i] = ATL.determinant() * ((i&1) ? 1.0 : -1.0);
		// this is the explicit solution using the pre-computed matrix inverse for ax and axx (see remark on other vectors above)
		ax[i] = 2 * (d1[i]-a0[i]) - 0.5 * (d2[i]-a0[i]);
		axx[i] = -(d1[i]-a0[i]) + 0.5 * (d2[i]-a0[i]);
	}

	// compute ayy,ay with the ATL(0,1) and ATL(0,2)
	getNonNormalizedStructureMatrix(Vector3d(0,1,0), R);
	for (int i=0; i<4; i++)
	{
		ATL = AT;
		ATL.block(0,i,3,3-i) = ATL.block(0,i+1,3,3-i);
		ATL.conservativeResize(3,3);
		d1[i] = ATL.determinant() * ((i&1) ? 1.0 : -1.0);
	}

	getNonNormalizedStructureMatrix(Vector3d(0,2,0), R);
	for (int i=0; i<4; i++)
	{
		ATL = AT;
		ATL.block(0,i,3,3-i) = ATL.block(0,i+1,3,3-i);
		ATL.conservativeResize(3,3);
		d2[i] = ATL.determinant() * ((i&1) ? 1.0 : -1.0);
		// this is the explicit solution using the pre-computed matrix inverse for ay and ayy
		ay[i] = 2 * (d1[i]-a0[i]) - 0.5 * (d2[i]-a0[i]);
		ayy[i] = -(d1[i]-a0[i]) + 0.5 * (d2[i]-a0[i]);
	}

	getNonNormalizedStructureMatrix(Vector3d(1,1,0), R);
	for (int i=0; i<4; i++)
	{
		// compute axy
		ATL = AT;
		ATL.block(0,i,3,3-i) = ATL.block(0,i+1,3,3-i);
		ATL.conservativeResize(3,3);
		axy[i] = ATL.determinant() * ((i&1) ? 1.0 : -1.0) - axx[i] - ax[i] - ay[i] - ayy[i] - a0[i];
	
//		printf("N%i:= %15.15f * x^2 + %15.15f * x  + %15.15f * y^2 + %15.15f * y + %15.15f * x * y + %15.15f = 0;\n",i,axx[i],ax[i],ayy[i],ay[i],axy[i],a0[i]);
	}

	// compute the workspace
	MatrixXd Lambda(steps,8);
	for (int i=0; i<4; i++)
	{
		// now that we have the coefficients we do inline the evaluation of all rays for workspace computation
		for (int j=0; j<steps; j++)
		{
			double phi = j*deltaPhi;
			double A = (pow(cos(phi),2.0)*axx[i]+cos(phi)*axy[i]*sin(phi)+ayy[i]*pow(sin(phi),2.0));
			double B = (cos(phi)*ax[i]+ay[i]*sin(phi));
			double C = a0[i];
			double p = B/A;
			double q = C/A;
			double radikant = p*p/4.0-q;
			if (radikant<0)
			{
				Lambda(j,2*i) = 0;
				Lambda(j,2*i+1) = 0;
				continue;
			}
			Lambda(j,2*i)   = -p/2.0 + sqrt(radikant);
			Lambda(j,2*i+1) = -p/2.0 - sqrt(radikant);
		}
	}

	return true;
}


///////////////////////////////////////////////////////////////////////////////
// Implementation of CWrenchSet
///////////////////////////////////////////////////////////////////////////////


CWrenchSet::CWrenchSet(CRobotData& robot)
	: CStructureMatrix(robot)
{
	// set the size of the matrices according to the binomal coefficient based on now and dof
	C.resize(factorial(pRobot->getNow())/(factorial(pRobot->getNow() - (pRobot->getDof()-1))*factorial(pRobot->getDof() - 1)),pRobot->getDof() - 1); // fac(now)/(fac(now-(dof-1))*fac(dof-1))
	C_komp.resize(factorial(pRobot->getNow())/(factorial(pRobot->getNow() - (pRobot->getDof()-1))*factorial(pRobot->getDof() - 1)),pRobot->getNow() - (pRobot->getDof() - 1)); // fac(now)/(fac(now-(dof-1))*fac(dof-1))
	N.resize(2*factorial(pRobot->getNow())/(factorial(pRobot->getNow() - (pRobot->getDof()-1))*factorial(pRobot->getDof() - 1)),pRobot->getDof()); // 2*fac(now)/(fac(now-(dof-1))*fac(dof-1))
	d.resize(2*factorial(pRobot->getNow())/(factorial(pRobot->getNow() - (pRobot->getDof()-1))*factorial(pRobot->getDof() - 1)),1);  // 2*fac(now)/(fac(now-(dof-1))*fac(dof-1))

    // choose k from n realized with a permutation of zero and ones
	std::vector<int> ints;
	for (int i = pRobot->getNow()-1; i >= 0; i--)
	{
		if (i < pRobot->getDof()-1)
			ints.push_back(1);
		else
			ints.push_back(0);
	}
	
	// transforming the results into the final permutation matrices C and C_komp (needs only be done once for given now and dof)
	int k = 0;
	do
	{
		int j = 0, j_komp = 0; // manage the indices of C and C_komp
		for (int i = 0; i < pRobot->getNow(); i++)
		{
			if (ints[i] == 1) 
			{
				C(k,j) = i; // i corresponds to the cable-nr, beginning with 0
				j++;
			}
			else
			{
				C_komp(k,j_komp) = i; // i corresponds to the cable-nr, beginning with 0
				j_komp++;
			}
		}
		k = k+1;
	}
	while (PCRL::next_permutation(ints.begin(),ints.end())); // STL-function
}

//! factorial function, e.g. for calculating binomial coefficients
int CWrenchSet::factorial(int n)
{
 	int retval = 1;
 	for (int i = n; i > 1; --i)
 		retval *= i;
 	return retval;
}
 

/*! calculate the hyperplane representation according to Bouchard, Gosselin and Moore  
 *  in "ON THE ABILITY OF A CABLE-DRIVEN ROBOT TO GENERATE A PRESCRIBED SET OF
 *  WRENCHES" on ASME 2008
 */
bool CWrenchSet::getHyperPlaneRepresentation(const Vector3d& r, const Matrix3d& R)
{
	const double eps_det = 0.00000001; // check if cross product is ok
	const int dof = pRobot->getDof();
	const int now = pRobot->getNow();
	
	double hPlus, hMinus, lj;
	double deltaF = pRobot->fmax - pRobot->fmin;
	int shift;

	MatrixXd w(dof,1), pPlus(dof,1), pMinus(dof,1);
	MatrixXd UnitWrenchArray(dof, dof-1);
	MatrixXd UnitWrenchArray_tmp(dof-1,dof-1);
	MatrixXd n(dof,1);
	MatrixXd tMin = MatrixXd::Constant(now,1,pRobot->fmin); // vector with minimal cable forces
	
	getStructureMatrix(r,R); // calculate Structure Matrix
	
	for (int i=0; i<C.rows(); i++)
	{
		for (int j=0; j<dof-1; j++)
			UnitWrenchArray.col(j) = AT.col(C(i,j));
		
		// generalized cross product
		for (int j=0; j<dof; j++)
		{	
			shift = 0;
			for (int k=0; k<dof; k++)
			{
				if (k != j)
					UnitWrenchArray_tmp.row(k-shift) = UnitWrenchArray.row(k);
				else
					shift = 1;
			}

			n(j) = pow(-1.0,j)*UnitWrenchArray_tmp.determinant();
		}
		
		if (n.norm() > eps_det) // check, if the vectors were linear independent
			n.normalize(); // normalize the cross product
		else
			n = MatrixXd::Zero(dof,1);
		

		hPlus = hMinus = 0.0;

		for (int j=0; j<now-(dof-1); j++)
		{
			lj = AT.col(C_komp(i,j)).transpose().dot(n.col(0)); // scalar product
			hPlus = hPlus + deltaF*max(0.0, lj);
			hMinus = hMinus + deltaF*min(0.0, lj);
		}

		pPlus = hPlus*n + AT*tMin;
		pMinus = hMinus*n + AT*tMin;
		
		d(2*i+1) = n.col(0).dot(pPlus.col(0));
		d(2*i) = -n.col(0).dot(pMinus.col(0));
		N.row(2*i+1) = n.transpose();
		N.row(2*i) = -n.transpose();
	}
	return true;
}


/*! check if N*w <= d
  * if the inequality holds, there exist a feasible force distribution for the wrench
  * and return true
  */
bool CWrenchSet::checkPoint(MatrixXd& w)
{
	for (int i=0; i<N.rows(); i++)
		if (N.row(i).dot(-w.col(0)) > d(i)) // Attention: minus w
			return false;

	return true;
}

/*! check if the edges of a box are feasible
 */
bool CWrenchSet::checkBox()
{
	cout << "not implemented yet\n";
	return true;
}

/*! check if a wrench described by an ellipsoid is feasible
  * if the inequality holds, there exist a feasible force distribution for the wrench
  * and return true
  * [Matrix a]: size of the ellipsoid in xyz and abc, typically 6x1-vector
  */
bool CWrenchSet::checkEllipsoid(MatrixXd& a)
{
	const int dof = pRobot->getDof();
	const int now = pRobot->getNow();
	MatrixXd n(dof,1), e1(dof,1), e2(dof,1);
	MatrixXd diag = MatrixXd::Identity(dof,dof);
	double k;
	for (int i=0; i<N.rows()/2; i++)
	{
		n = N.row(2*i+1).transpose(); // get the normal vector (positive)

		k = 0.0;
		for (int j=0; j<dof; j++)
			k +=  pow(a(j)*n(j),2.0);
		if (k != 0.0)
		{
			k = 1.0/sqrt(k);
			for (int j=0; j<dof; j++)
				diag(j,j) = pow(a(j),2.0);
			
			e1 = diag*n*k;
			e2 = -diag*n*k;

			if (N.row(2*i+1).dot(e1.col(0)) > d(2*i+1)) 
				return false;
			if (N.row(2*i).dot(e2.col(0)) > d(2*i)) 
				return false;
		}
	}
	return true;
}

//! implement the virtual function test pose to perform a one-function-call
//! test if the given pose has the desired wrench set
//! The wrench is derived from the actual workspace algorithms configuration
//! and is interpreted as !ellipsoid!
bool CWrenchSet::testPose(const Vector3d& r, const Matrix3d& R)
{
	// compute the hyperplanes
	getHyperPlaneRepresentation(r,R);
	
	// combine force, torque to form a wrench vector
	// expand this for arbitrary DOF!
	MatrixXd w(6,1);
	w(0) = f.x();
	w(1) = f.y();
	w(2) = f.z();
	w(3) = tau.x();
	w(4) = tau.y();
	w(5) = tau.z();
	
	// evaluate the wrench hyperellipsoid
	if (w.norm() != 0)
		return checkEllipsoid(w);
	else 
		return checkPoint(w);
}

///////////////////////////////////////////////////////////////////////////////
// Implementation of CVelocitySet
///////////////////////////////////////////////////////////////////////////////

CVelocitySet::CVelocitySet(CRobotData& robot)
	: CStructureMatrix(robot)
{
	setC();		//< create permutation matrix
	cablesVelocity.resize(1,pRobot->getNow());
	velocitySet.resize(1,pRobot->getDof());
}

/*! generate C-matrix (2^Dof,Dof), with all the possible 2^Dof binary permutations
 * \changed: C
 */
void CVelocitySet::setC()
{
	C.setZero(1 << pRobot->getDof(), pRobot->getDof());

	for (int i=1; i<C.rows(); i++)
	{
		for (int j=0; j<pRobot->getDof(); j++)
		{
			C.row(i) = C.row(i-1);
			while (C(i,j) == 1)
				C(i,j++) = 0;
			C(i,j) = 1;
			break;
		}
	}
}

/*!	calculate the required cables' velocity for a given velocity set
 *  \param [in] r: cartesian position of the platform
 *  \param [in] R: orientation of the platform
 *  \considered: velocitySet, pRobot->pWinch->getCable_MaxVelocity()
 *	\changed: cablesVelocity
 *  \return: -> true, if the biggest cable velocity is smaller (or equal) than the maximal winch velocity
 *			 ->	false, otherwise
 */
bool CVelocitySet::testPose(const Vector3d& r, const Matrix3d& R)
{
	MatrixXd tmin(1, pRobot->getDof()), tmax(1,pRobot->getDof());
	getVelocitySet(tmax);
	tmin = -tmax; //< consider velocity set symmetric

	// range of velocity set
	MatrixXd deltaT(1,pRobot->getDof());
	deltaT = tmax-tmin;

	getStructureMatrix(r,R); //< refresh structure matrix
	MatrixXd A = AT.transpose();

	MatrixXd testVelocity(1,pRobot->getDof());
	cablesVelocity.setZero(1,pRobot->getNow()); //< initialize cables' velocity
	for (int i=0; i < C.rows(); i++)
	{
		// -A((tmin)+(C(i)*deltaT));
		testVelocity = -A * (tmin.transpose()+(C.row(i).cwiseProduct(deltaT)).transpose());
		// refresh cablesVelocity, if new values are bigger than older ones
		for (int j=0; j<testVelocity.size(); j++)
			if (abs(testVelocity(j)) > cablesVelocity(j))
				cablesVelocity(j) = abs(testVelocity(j));
	}

	// check, if cables' velocity is feasible
	if (cablesVelocity.maxCoeff() <= pRobot->pWinch->v)
		return true;
	return false;
}

/*!	calculate and returns the required max cables' velocity
 *	\param [in] minBox: minimal box position
 *  \param [in] maxBox: maximal box position
 *	\param [in] eps:	calculation step
 *  \return: max cables velocity
 *	\remark: velocity set muss be different from zero
 */
double CVelocitySet::getMaxCablesVelocityBox(const Vector3d& minBox, const Vector3d& maxBox, const Vector3d& eps)
{
	MatrixXd box(1,6);
	box << minBox(0),minBox(1),minBox(2),maxBox(0),maxBox(1),maxBox(2);

	if (velocitySet.isZero()) //< neglect the case: velocitySet equals zero
		return 0;
	
	// check grid
	double maxVelocity = 0;
	for (double x=minBox(0); x <= maxBox(0); x+=eps(0))
		for (double y=minBox(1); y <= maxBox(1); y+=eps(1))
			for (double z=minBox(2); z <= maxBox(2); z+=eps(2))
			{
				testPose(Vector3d(x,y,z), Matrix3d::Identity(3,3));
				if (cablesVelocity.maxCoeff() > maxVelocity) //< take the biggest value
					maxVelocity = cablesVelocity.maxCoeff();
			}

	return maxVelocity;
}

///////////////////////////////////////////////////////////////////////////////
// Implementation of CStiffness
///////////////////////////////////////////////////////////////////////////////

CStiffness::CStiffness(CRobotData& robot)
: CStructureMatrix(robot)
{
	// estimate for constant k'= E * A in [N] assuming E=110000Nmm^-2 and A = 314mm^2; 
	// Source: Klaus Feyrer, "Drahtseile - Bemessung, Betrieb, Sicherheit", 2. Auflage, Springer Verlag Wien 2000, p.92-94
	k = 110000 * 314;	// 6.594e7;			
    //!\todo Ricardo propses to compute the stiffness from the current's cable properties here
    //! however, the existing abstraction layer does not allow to access the data from this position
    //! therefore, we have to reconsider the concept of parameter passing into stiffness matrix in order
    //! to allow appropriate usage of stiffness coefficients.
	// k = pRobot->pCable->E_wire * MO_PI * pRobot->pCable->r_cable * pRobot->pCable->r_cable; // 110000 * 314;	// 6.594e7;			
	K.resize(pRobot->getDof(),pRobot->getDof());
	K.setZero();
	K_g.resize(pRobot->getDof(),pRobot->getDof());
	K_g.setZero();

	l_additional.resize(pRobot->getNow(), 1);
	l_additional.setZero();

	requiredStiffness = 0;
}

/*! calculate the stiffness matrix K for the given pose (r,R) using Verhoevens 
 *  formula (see Diss Verhoeven 2004, p. 50, Eq. (3.29)):
 *  
 *  K(x) = k' AT(x) * L^-1 (x) * A(x) 
 * 
 *  where x is the pose defined by (r,R).  The calculation is based on a the simplified 
 *  winch kinematics model, where the stiffness only depends on the free cable 
 *  length and uneffected by winch mechanics, motors, and controls.
 *  \param r [in] position of the platform
 *  \param R [in] orientation of the platform
 *  \return true, if the calculation was successfull; In this case the class member 
 *  K is a dof x dof matrix containing the stiffness matrix for the
 *  pose (r,R). Otherwise the values in K are undefined. 
 */
bool CStiffness::StiffnessMatrix(const Vector3d& r, const Matrix3d& R)
{
	// verify that the stiffness matrix has the right size
	K.resize(pRobot->getDof(),pRobot->getDof());

	// calcuate the structure matrix and the cable length
	if (!getStructureMatrix(r,R))
		return false;

	// calculate matrix Linv (with the the reciprocal wire length); 
	// the formula for inverse kinematics for wire length is embedded below
	MatrixXd Linv = MatrixXd::Zero(pRobot->getNow(),pRobot->getNow());
	for (int i=0; i<pRobot->getNow(); i++)
		Linv(i,i) = k/((r + R*pRobot->getPlatform(i) - pRobot->getBase(i)).norm()+l_additional(i));

	// caluclate the stiffness matrix K
	K = AT * Linv * AT.transpose();
	K_c = K; // prototype...
	return true;
}

//! calculation of the stiffness matrix for pose (r,R): cable + geometrical stiffness
bool CStiffness::StiffnessMatrixMain(const Vector3d& r, const Matrix3d& R, const MatrixXd& f)
{
	StiffnessMatrix(r, R);
	GeometricalStiffnessMatrix(r, R, f);

	K_total = K_c+K_g;

	return true;
}


//! calculation of the geoemtrical stiffness matrix for pose (r,R) and cable forces f
bool CStiffness::GeometricalStiffnessMatrix(const Vector3d& r, const Matrix3d& R, const MatrixXd& f)
{
	K_g = MatrixXd::Zero(6,6); // geometrical stiffness matrix

	MatrixXd  AT_derivated(6,6); // derivation of the Structure matrix along the Cartesian axes
	Vector3d li, ui, li_point, ui_point, vi_point; // helper variables
	double li_norm, li_point_norm; // helper variables
	MatrixXd t(6,1); // direction of the derivation

	for (int i=0; i<pRobot->getNow(); i++)
	{
		li =- r - R*pRobot->getPlatform(i) + pRobot->getBase(i);
		ui = li.normalized();
		li_norm=li.norm();

		for (int j=0; j<3; j++) // derivation along x,y,z
		{
			t = MatrixXd::Zero(6,1);
			t(j) = 1;
			li_point = -t.block(0,0,3,1);
			li_point_norm = (li.dot(li_point))/li_norm;

			ui_point = (li_point*li_norm-li*li_point_norm)/(li_norm*li_norm);
	
			for (int i_=0; i_<3; i_++)
				AT_derivated(i_,j) = ui_point(i_);
		
			vi_point = (R*pRobot->getPlatform(i)).cross(ui_point);
			for (int i_=0; i_<3; i_++)
				AT_derivated(i_+3,j) = vi_point(i_);
		}
		
		for (int j=3; j<6; j++) // derivation along a, b, c
		{
			t = MatrixXd::Zero(6,1);
			t(j) = 1;
			// skew matrix for the angular velocity
			Matrix3d crossProductMatrix = Matrix3d::Zero(3,3);
			crossProductMatrix(0,1) = -t(5);
			crossProductMatrix(0,2) = t(4);
			crossProductMatrix(1,0) = t(5);
			crossProductMatrix(1,2) = -t(3);
			crossProductMatrix(2,0) = -t(4);
			crossProductMatrix(2,1) = t(3);

			li_point = -crossProductMatrix*R*pRobot->getPlatform(i);

			li_point_norm = (li.dot(li_point))/li_norm;

			ui_point = (li_point*li_norm-li*li_point_norm)/(li_norm*li_norm);
	
			for (int i_=0; i_<3; i_++)
				AT_derivated(i_,j) = ui_point(i_);
		
			vi_point = (crossProductMatrix*R*pRobot->getPlatform(i)).cross(ui) + (R*pRobot->getPlatform(i)).cross(ui_point);
			for (int i_=0; i_<3; i_++)
				AT_derivated(i_+3,j) = vi_point(i_);
		}
		
		for (int j_=0; j_<6; j_++)
			for (int i_=0; i_<6; i_++)
				K_g(i_,j_) = K_g(i_,j_)-AT_derivated(j_,i_)*f(i); // Attention: AT_derivated is transposed
	}	
	return true;
}


/*! Compute the stiffness matrix and scale the matrix such that the rotation part
 *  of the matrix is related to a characteristic length. This length p_c is determined
 *  from the robot geometric e.g. by considering the length of the platform vectors
 *  |b_i|. This we get the following matrix
 *  
 *  K(x) = k' S * AT(x) * L^-1 (x) * A(x) * S
 *
 *  where S is the scaling matrix diag (1, 1, 1, 1/p_c, 1/p_c, 1/p_c)
 * 
 *  \param r [in] position of the platform
 *  \param R [in] orientation of the platform
 *  \return true, if the calculation was successfull; In this case the class member 
 *  K is a dof x dof matrix containing the stiffness matrix for the
 *  pose (r,R). Otherwise the values in K are undefined. 
 * 
 *  \remark
 *  The following code is still experimental; it should normalize torques to forces 
 *  (and radiant to length) for  the stiffness computation. 
 */
bool CStiffness::StiffnessMatrixHomogenous(const Vector3d& r, const Matrix3d& R)
{
	// compute the standard form of the stiffness matrix
	if (!StiffnessMatrix(r,R))
		return false;

	// now we apply the transformation to make the matrix homogenous

	// firstly, compute a normalized stiffness matrix
	// Verhoeven proposes to use the avarage length of |b_i| as normalizing length p_c
	// however, we use the maximum max |b_i| for p_c

	double p_c = 1e-6;		// we use the scale 1e-6 as smallest possible value to avoid p_c=0 for a point-shaped platform
	for (int i=0; i<pRobot->getNow(); i++)
		p_c = max(p_c,pRobot->getPlatform(i).norm());

	// compute the normalization matrix according to Verhoeven 2004, Eq. (3.38)
	// from verhoevens work it is expected that the factor should be p_c, but 
	// the results seems to be meaningful if we use 1/p_c instead (which means 
	// that we compute S^-1 rather than S)
	MatrixXd S = MatrixXd::Zero(pRobot->getDof(),pRobot->getDof());
	int dim = 6, trans = 3;
	switch (pRobot->getMotionPattern())
	{
	case CRobotData::MP1T: dim = 1; trans = 1; break;
	case CRobotData::MP2T: dim = 2; trans = 2; break;
	case CRobotData::MP3T: dim = 3; trans = 3; break;
	case CRobotData::MP1R2T: dim = 3; trans = 2; break;
	case CRobotData::MP2R3T: dim = 5; trans = 3; break;
	case CRobotData::MP3R3T: dim = 6; trans = 3; break;
	default: return false; // we cannot compute the stiffness matrix in this case; should never happen
	}
	for (int i=0; i<dim; i++)
		if (i < trans)
			S(i,i) = 1;			// 1
		else
			S(i,i) = 1/p_c;		// p_c 

	// according to Verhoeven Eq. (3.38) we have to multiply the normalizing 
	// length from both sides to K.
	K = S*K*S;		
	return true;
}

/*! Calculate the translational stiffness matrix K for the given pose (r,R) 
 *  using Verhoevens  formula (see Diss Verhoeven 2004, p. 50, Eq. (3.29)):
 *
 *  K(x) = k' AT(x) * L^-1 (x) * A(x) 
 *
 *  where x is the pose defined by (r,R). Only the rows of AT corresponding 
 *  with translational displacements are considered. The result is largely 
 *  simplified and may lead to wrong values. The function only works if the 
 *  first three rows of AT correspond with the translational displacements.
 *  \param r [in] position of the platform
 *  \param R [in] orientation of the platform
 *  \return true, if the calculation was successfull. In this case the size of 
 *  matrix K is 3 x 3 and the class member K containts the stiffness matrix 
 *  for the pose (r,R). Otherwise the values in K are undefined. 
 */
bool CStiffness::StiffnessMatrixTranslational(const Vector3d& r, const Matrix3d& R)
{
	// verify that the stiffness matrix has the right size
	K.resize(3,3);

	// calcuate the structure matrix
	if (!getStructureMatrix(r,R))
		return false;

	// calculate matrix Linv (with the the reciprocal wire length); 
	// the formula for inverse kinematics for wire length is embedded below
	MatrixXd Linv = MatrixXd::Zero(pRobot->getNow(), pRobot->getNow());

	for (int i=0; i<pRobot->getNow(); i++)
		Linv(i,i) = k/(( r + R*pRobot->getPlatform(i) - pRobot->getBase(i) ).norm()+l_additional(i));

	// Calculate the translational stiffness matrix (no consideration of 
	// angular displacement)
	MatrixXd at(3,pRobot->getNow());
	at = AT.block(0,0,3,pRobot->getNow()); 
	//at.getSubmatrix(AT,1,1); todo
	K = at * Linv * at.transpose();
	return true;
}

void CStiffness::getStiffnessMatrixTranslational(Matrix3d& K_)
{
	K_ = Matrix3d(K);
}

//! get minimal stiffness (i.e. the smallest eigenvalue of K)
//! \todo remove the printing statements in this class and
//! store results either as states or return the results.
double CStiffness::getMinimalStiffness(int i)
{
	MatrixXd EV;
	switch (i) // analyse the first character
	{
		case 0: // K_c
		{
			// calculate the eigenvalues to find the minimum stiffness for an arbitraray displacement
			Eigen::SelfAdjointEigenSolver<MatrixXd> eigensolver(K);
			EV = eigensolver.eigenvalues();
			Eigen::SelfAdjointEigenSolver<MatrixXd> es(K);
			cout << "The eigenvalues of K_c are:" << endl << es.eigenvalues() << endl;
			cout << "The matrix of eigenvectors, V, is:" << endl << es.eigenvectors() << endl << endl;
			break;
		}
		case 1: // K_g
		{
			// calculate the eigenvalues to find the minimum stiffness for an arbitraray displacement
			Eigen::SelfAdjointEigenSolver<MatrixXd> eigensolver(K_g);
			EV = eigensolver.eigenvalues();
			Eigen::SelfAdjointEigenSolver<MatrixXd> es(K_g);
			cout << "The eigenvalues of K_g are:" << endl << es.eigenvalues() << endl;
			cout << "The matrix of eigenvectors, V, is:" << endl << es.eigenvectors() << endl << endl;
			break;
		}
		case 2: // K_total = K_c + K_g
		{
			// calculate the eigenvalues to find the minimum stiffness for an arbitraray displacement
			Eigen::SelfAdjointEigenSolver<MatrixXd> eigensolver(K_total);
			EV = eigensolver.eigenvalues();
			Eigen::SelfAdjointEigenSolver<MatrixXd> es(K_total);
			cout << "The eigenvalues of K_total are:" << endl << es.eigenvalues() << endl;
			cout << "The matrix of eigenvectors, V, is:" << endl << es.eigenvectors() << endl << endl;
			break;
		}
		default:
			return -1;
	}
	return EV(0);
}

//! the following functions only perform some "example calculations"...
//! \remark this function is deprecated and subject to be deleted
bool CStiffness::examples()
{
	// apply a load in z-direction and calculate the displacement
	MatrixXd dw = MatrixXd::Zero(6,1);
	dw(3) = 10000;
	MatrixXd displacement = K*dw;

	return true;
}

//! the following function prints the cable stiffness
bool CStiffness::printWireStiffness(const Vector3d& r, const Matrix3d& R)
{ 
	cout << "Stiffness in the Wires for specific stiffness of " << k << endl;
	for (int i=0; i<pRobot->getNow(); i++)
		cout << k/(( r + R*pRobot->getPlatform(i) - pRobot->getBase(i) ).norm()+l_additional(i)) << endl;

	const int dof = pRobot->getDof();
	const int now = pRobot->getNow();

	if (now > dof)
	{	
		MatrixXd H(now,now-dof);
		// now we compute explicitly the spanning basis of the kernel of AT
		H = AT.fullPivLu().kernel();
		cout << "Kernel in delta L " << endl;

		for (int i=0; i<pRobot->getNow(); i++)
			cout << H(i,0)/(k/(( r + R*pRobot->getPlatform(i) - pRobot->getBase(i) ).norm()+l_additional(i))) << " , " << H(i,1)/(k/(( r + R*pRobot->getPlatform(i) - pRobot->getBase(i) ).norm()+l_additional(i)))  << endl;
	}
	return true;
}

//! calculate stiffness of a pose
//! \param r: position
//! \param R: orientation
//! \return: true, if minimal stiffness is bigger than the minimal required stiffness
//!			 false, otherwise
bool CStiffness::testPose(const Vector3d& r, const Matrix3d& R)
{	
	if(StiffnessMatrix(r,R))
		if (getMinimalStiffness(0) > requiredStiffness)
			return true;

	return false;
}

// CCableSagging
//////////////////////////////////////////////////////////////////////////////

//! perform a number of computations around cable sagging
bool CCableSagging::analyzePose(const Vector3d& r, const Matrix3d& R)
{
	// 1. compute the vector l_i using inverse kinematics
	PCRL::CKinematics kin(*pRobot);
	MatrixXd l = MatrixXd::Zero(pRobot->getNow(),1);
	if (!kin.doInverseKinematics(r,R,l))
		return false;

	// compute the structure matrix; we need the nominal direction n_i of the 
	// cables which is stored in the structure matrix
	if (!getStructureMatrix(r,R))
		return false;

	// we need to compute the forces; therefore, we create a force distribution object ad-hoc
	MatrixXd wrench = MatrixXd::Zero(pRobot->getDof(),1);
	MatrixXd f_wires = MatrixXd::Zero(pRobot->getNow(),1);
	if (!getDistribution(wrench,f_wires))
		return false;

	// check if forces are feasible before proceeding
	if (!rateDistributionFeasible(f_wires))
		return false;

	for (int i=0; i<pRobot->getNow(); i++)
	{
		// 2. compute the specific constants related to sagging, i.e.
		//   w ... specific cable gravity force
		//   H ... horizontal cable force on the platform
		//   mu = w/H
		//   xb ... horizontal length of the cable 
		//   zb ... vertical length of the cable
		double w = pCable->weight;

		// the cable force vector can be computed as follows
		Vector3d F = f_wires(i) * getDirection(i);

		// project the cable force into the horizontal plane
		double H = sqrt(F.x()*F.x() + F.y()*F.y());
		double mu = w/H;

		// the cable length vector is computed from
		Vector3d L = l(i) * getDirection(i);
		double xb = sqrt(L.x()*L.x() + L.y()*L.y());
		double zb = -L.z();		// we need to revert the direction since u is pointing from the platform to the base

		// 3. compute the desired properties
		//   length of the sagged cable (i.e. the additonal length required for the curved shape of the cable)
		// the following code is generated using maple
		double l_sag;
		{
			double t1 = xb * mu;
			double t3 = exp(t1);
			double t5 = zb * zb;
			double t6 = mu * mu;
			double t8 = t3 * t3;
			double t13 = sqrt(t8 * t6 * t5 + t8 * t3 - 0.2e1 * t8 + t3);
			double t14 = t3 * mu * zb + t13;
			double t17 = t3 - 0.1e1;
			double t19 = 0.1e1 / t17 / t3 * t14;
			double t20 = log(t19);
			double t21 = t1 + t20;
			double t22 = sinh(t21);
			double t23 = t22 * t22;
			double t25 = sqrt(0.1e1 + t23);
			double t27 = cosh(t21);
			double t29 = 0.1e1 / mu;
			double t34 = t17 * t3 / t14;
			double t35 = t19 - t34;
			double t38 = sqrt(0.1e1 + t35 * t35 / 0.4e1);
			l_sag = t29 / t27 * t22 * t25 - t29 / (t19 + t34) * t35 * t38;
		}

		//   maxium sagging from the linear shape
		//   delta_beta ... change of the attack angle on the platform. this code is also auto generated by maple
		double beta_sag;
		{	
			double t1 = xb * mu;
			double t3 = exp(t1);
			double t5 = zb * zb;
			double t6 = mu * mu;
			double t8 = t3 * t3;
			double t13 = sqrt(t8 * t6 * t5 - 0.2e1 * t8 + t8 * t3 + t3);
			double t20 = log(0.1e1 / (t3 - 0.1e1) / t3 * (t3 * mu * zb + t13));
			double t22 = sinh(t1 + t20);
			beta_sag = atan(t22);
		}
		double beta = atan(zb/xb);
	   
		// compute the elongation of the cable under current load, minimum load, and maximum load
		double c = l(i)* 4.0 / (pCable->E_wire * pCable->r_cable * pCable->r_cable * MO_PI);
		double dlcur = c*f_wires(i);
		double dlmin = c*pRobot->fmax;
		double dlmax = c*pRobot->fmax;;

		// 4. for now, we just print our results to the console; for future usage,
		//  we should store the results in an internal buffer and provide getter functions
		//  to read the results or store the results in an external file
		printf("%i: Linear length: %3.6f, sagging length: %3.6f delta: %f3.6\n",i,l(i),l_sag,l_sag-l(i));
		printf("  : linear angle: %3.6f, sagging angle: %3.6f delta: %f3.6\n",beta,beta_sag,beta_sag-beta);
		printf("  : elastic elongation dlmin = %3.6f dl = %3.6f dlmax = %3.6f\n",dlmin,dlcur,dlmax);
	}
	return true;
}

/*! return the maximum sagging for a horizontal cable with the given parameter
 *  \param H horizonal cable force [N]
 *  \param l length of the cable [m]
 *  \param g weight of the cable per length unit in [N/m]
 *  \return total amount of sagging of the cable from the straight form in the middle
 */
double CCableSagging::getMaxSag(const double& H, const double& l, const double& g)
{ return H/g*(cosh(g*l/H/2.0)-1.0); }

/*! return the maximum change elongation for a horizontal cable with the given parameter
 *  \param H horizonal cable force [N]
 *  \param l length of the cable [m]
 *  \param g weight of the cable per length unit in [N/m]
 *  \return total elongation of sagging of the cable from the straight form in the middle
 */
double CCableSagging::getMaxElongation(const double& H, const double& l, const double& g)
{ return 2.0*sinh(g*l/H/2.0)*H/g-l; }

} // end namespace PCRL
