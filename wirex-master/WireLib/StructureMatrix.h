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

/*! \file StructureMatrix.h
 *
 *	\author   Andreas Pott
 *
 *  \dependency
 *		Eigen3		for IR³ algebra and for linear algebra of general matrices
 */

#pragma once

#include "Algorithm.h"
#include "PoseList.h"
#include "EnumTables.h"

// If you have access to the convex optimization code for force distribution 
// activate this macro to include the wirelib interface. 

// #define WIRELIB_HAS_CVXGEN

namespace PCRL {

/*! \class CStructureMatrix
 *  CStructureMatrix encapsulates algorithms for determination and analysis
 *  of the structure matrix of parallel cable robots. The structure matrix
 *  fulfills the so-called structure equation (or static equilibrium) 
 *		A^T * f + w = 0
 *  where A^T is the structure matrix, f is a positive vector collecting the 
 *  cable forces, and w is the applied wrench at the mobile platform, i.e. 
 *  the forces and torques acting on the platform.
 *  The structure matrix is the central object to analyse the workspace and
 *  mobility of cable robots.
 *  Thus many algorithm use the matrix as input datum.
 *
 *  \todo: Consider removing the local storage for f, tau from this class. The 
 *         redesign of CRobotDocument should collect all state data in this class.
 */
class CStructureMatrix : public CPoseProperty
{
protected:
	void bind(); //!< connect the configuration parameters to the reflection object
	MatrixXd AT; //!< structure matrix mapping cable forces to end-effector forces
public:
	Vector3d f;			//!< forces applied to the platform (in world coordinates)
	Vector3d tau;		//!< torque applied to the platform
	bool usePulley;		//!< if true, the influence of pulleys is taken into account when setting up the structure matrix
public:
	//! standard constructur to connect an instance of this class to the robot object
	explicit CStructureMatrix(CRobotData& robot);
	//! set Wrench (the applied forces and torques) for the platform
	void setWrench(const Vector3d& F, const Vector3d& Tau) {f=F; tau=Tau; }
	//! calculate the structure matrix for pose (r,R)
	bool getStructureMatrix(const Vector3d& r, const Matrix3d& R);
	//! calculate the structure matrix for pose (r,R) without normalization of the direction vectors
	bool getNonNormalizedStructureMatrix(const Vector3d& r, const Matrix3d& R);
	//! get the maximum norm for all columns of the structure matrix
	double getMaxColumnNorm();
	//! get the maximum norm for all rows of the structure matrix
	double getMaxRowNorm();
	//! get the singular values
	bool getSingularValues(MatrixXd& sv);
	//! determine the rank of the structure matrix
	int getRank();
	//! test if the structure matrix has the full rank or in other words, test if the robot is in a singular configuration
	bool isFullRank();
	//! return the direction vector of the i-th cable
	Vector3d getDirection(const unsigned int& i);
	//! set wrench vector
	bool setWrenchVector(MatrixXd& w, const Vector3d& F, const Vector3d& Tau);
	//! print the elements of the StructureMatrix on the console
//	void print();
	bool computeKernel(MatrixXd& H) { H = AT.fullPivLu().kernel(); return true; }
	//! get an element of the structureMatrix
	double getElement(const unsigned int& i, const unsigned int& j) const;
	//! return the structure matrix as a whole in the arguemtn At
	void getMatrix(MatrixXd& At) const { At=AT; }
	//! get the wrench generated from rigid body dynamic effects based on velocity and acceleration
	bool getWrenchDynamic(const CPoseKinetostatic& poseKinetostatic, MatrixXd& w);
	bool getWrenchDynamic(const Matrix3d& R, const Vector3d& v, const Vector3d& omega, const Vector3d& a, const Vector3d& alpha, Vector3d& fDynamic, Vector3d& tauDynamic);
	//! get At_dot based on a given plattform velocity
	void getStructureMatrixTimeDerivated(MatrixXd& At_dot,const Vector3d& r, const Matrix3d& R, const Vector3d& v, const Vector3d& omega);

};

/*! \class CForceDistribution
 *  The class presents some algorithms to solve the structure equation for
 *  given values of AT and w. Basically, the problem is a system of linear 
 *  equations. The problem for solving the system is that only positive solutions
 *  in a certain range are feasible.
 */
class CForceDistribution : public CStructureMatrix
{

protected:
	void bind();
private:

	emForceDistributionMethod::Type forceDistributionMethod;
	//! selector for calculation method
	//! selector for evaluation criteria
	emEvaluatorCriteria::Type evaluatorCriteria;

	emLinearSolverMethod::Type linearSolver;	//!< determine based the enum type which numeric solver shall be used to compute force distributions
	double condMin;			//!< minimum condition number to be accepted as part of the workspace

	double f_ref_QP; //!< reference force for QP algorithm which allows to control the tension level

	// internal states about the algorithms, the following member contain runtime information
	// from the respective algorithms in this class
	int dykstraIterations;		//!< after calls to dykstra method, the variable stores the number of iterations
	int QPIterations;		//!< after calls to QP, the variable stores the number of iterations
	int QPconverged;		//!< after calls to QP, the variable stores if the algorithm converged

public:
	explicit CForceDistribution(CRobotData& robot);
	//! check the existence/quality of the workspace for the pose (r,R)
	double testWorkspace(const Vector3d& r, const Matrix3d& R);
	//! overloaded virtual function; the workspace existence test called from this method will be used in generic evaluation such as workspace computation
	bool testPose(const Vector3d& r, const Matrix3d& R);
	//! preliminary testfunction for experimental force closure algorithm
	bool testForceClosure(const Vector3d& r);
	bool testForceClosure(const Vector3d& r, const Matrix3d& R);
	//! compute the parametric form of the constant orientation wrench-closure workspace
	bool calculateWorkspaceWrenchClosure();

	//! test if wire force f is valid
	double rateDistributionFeasible(const MatrixXd& f_wires) const;
	double rateDistributionEuclidean(const MatrixXd& f_wires) const;
	double rateDistributionInfinity(const MatrixXd& f_wires) const;

	//! calculate the force distribution with the selected method. fmin and fmax is extract from the current robot
	bool getDistribution(MatrixXd& w, MatrixXd& f_wires) { return getDistribution(w,pRobot->fmin,pRobot->fmax,f_wires); }
	//! calculate the force distribution with the selected method with same fmin/fmax for every wire
	bool getDistribution(MatrixXd& w, double& df_min, double& df_max, MatrixXd& f_wires);
	//! calculate the force distribution with the selected method
	bool getDistribution(MatrixXd& w, MatrixXd& f_min, MatrixXd& f_max, MatrixXd& f_wires);
	
	//! internal subroutine that compute doing the core computation of the closed-form formula
	bool getDistributionClosedFormCore(MatrixXd& AT, MatrixXd& w, MatrixXd& f_ref, MatrixXd& f_wires);
	//! calculate the force distribution with the closed-form formula
	bool getDistributionClosedForm(MatrixXd& w, MatrixXd& f_min, MatrixXd& f_max, MatrixXd& f_wires);
	//! calculate the force distribution with the advanced closed-form method
	bool getDistributionAdvancedClosedForm(MatrixXd& AT, MatrixXd& w, MatrixXd& f_min, MatrixXd& f_max, MatrixXd& f_wires);
	//! calculate the force distribution with the advanced closed-form method and optimization regarding energy efficiency
	bool getDistributionAdvancedClosedFormEnergyEfficient(MatrixXd& AT, MatrixXd& w, MatrixXd& f_min, MatrixXd& f_max, MatrixXd& f_wires);
	//! driver function advanced closed-form method and optimization regarding energy efficiency
	bool getDistributionAdvancedClosedFormEnergyEfficient( MatrixXd& w, MatrixXd& f_min, MatrixXd& f_max, MatrixXd& f_wires)
		{ return getDistributionAdvancedClosedFormEnergyEfficient(AT,w,f_min,f_max,f_wires); }
	//! driver function for advanced closed form method using the current structure matrix
	bool getDistributionAdvancedClosedForm(MatrixXd& w, MatrixXd& f_min, MatrixXd& f_max, MatrixXd& f_wires)
		{ return getDistributionAdvancedClosedForm(AT,w,f_min,f_max,f_wires); }
	//! calculate the force distribution with the closed-form method for energy efficient force distributions
	bool getDistributionClosedFormEnergyEfficient(MatrixXd& w, MatrixXd& f_min, MatrixXd& f_max, MatrixXd& f_wires);
	//! calculate a force distrubution using the pucture method, aiming at achieving relatively low feasible forces
	bool getDistributionPuncture(MatrixXd& w, MatrixXd& f_min, MatrixXd& f_max, MatrixXd& f_wires);
	//! calculate a force distrubution using the Quadratic Programming Optimization method, aiming at achieving relatively low feasible forces
	bool getDistributionQuadraticProgramming(MatrixXd& w, MatrixXd& f_min, MatrixXd& f_max, MatrixXd& f_wires);
	//! set special parameters of the QP method (default parameter already deliver good results)
	bool setQPparameters(int max_iters, double resid_tol, double eps, int refine_steps, double kkt_reg);
	//! calculate a force distribution using the brute force method (very slow)
	bool getDistributionBruteForce(MatrixXd& w, MatrixXd& f_wires);
	//! calculate a force distribution using Dykstra-Algorithm
	bool getDistributionDykstra(MatrixXd& w, MatrixXd& fmin, MatrixXd& f_max, MatrixXd& f_wires);
	//! experimental force distribution using an extended brute force method (hopefully less slow)
	bool getDistributionBruteForceEx(MatrixXd& w, MatrixXd& f_wires);
	//! experimental force distribution using a kernel based approach to compute the smallest soluution
	bool getDistributionWeightedSum(MatrixXd& w, MatrixXd& f_wires);
	//! compute all vertices of the convex solution set resulting from the intersection of the cube C with the kernel of the struture equation
	bool getDistributionAllVertices(MatrixXd& w, MatrixXd& f_wires);
	//! compute the force distribution with the Barycenter method
	bool getDistributionBarycenter(MatrixXd& w, MatrixXd& f_min, MatrixXd& f_max, MatrixXd& f_wires, int& numberOfVertices, double& areaSolutionSpace);

	//! return the number of iterations required for the last call to getDistributionDykstra
	int getDykstraIterations() { return dykstraIterations; }
	//! return the number of iterations required for the last call to getDistributionQuadraticProgramming
	int getQPIterations() { return QPIterations; }
	//! return the converged flag for the last call to getDistributionQuadraticProgramming
	int getQPconverged() { return QPconverged; }
	
	//! calculate and print the force distribution
	bool printForceDistribution(const Vector3d& r, const Matrix3d& R);
	bool printForceDistribution(const Vector3d& r) { return printForceDistribution(r,Matrix3d::ZRotationMatrix3d(0.0)); }
	
	//! define the method and criteria to generate and rate force distributions
	void setMethod(int method=0, int criteria=0);
	//! get the current settings for the method and the criteria for generating force distributions
	void getMethod(int &method, int &criteria);
	//! set the reference force used in QP algorithm
	void setFrefQP(double f_ref_QP){this->f_ref_QP=f_ref_QP; }
	//! get the reference force used in QP algorithm
	double  getFrefQP(){return f_ref_QP;}

	//! select the linear solver
	void setLinearSolver(emLinearSolverMethod::Type solver) {linearSolver = solver; }
	void setLinearSolver(int solver);
	//! determine the maximum forces and torques that can be applied to the given pose
	bool getMaximumWrenchForPose(const Vector3d& r, const Matrix3d& R, Vector3d& fmin, Vector3d& Taumin,Vector3d& fmax, Vector3d& Taumax);
	//! set the minimum value for the condition number
	void setConditionMin(const double& cm) { condMin=cm; }
	//! get the current minimal condition number for a pose to be accepted as workspace
	double getConditionMin() const { return condMin; }
};


/*! \class CWrenchSet
 *  The class presents some algorithms to solve the structure equation for
 *  given values of AT and w. Basically, the problem is a system of linear 
 *  equations. The problem for solving the system is that only positive solutions
 *  in a certain range are feasible.
 *  The concept and implementation is based on the paper 
 *  "On the ability of a cable-driven robot to generate a prescribed set of wrenches"
 *  by Samuel Bouchard et al. 2008.
 */
class CWrenchSet : public CStructureMatrix
{
protected:
	// depend only on now and dof: Were initialized once in the constructor of CWrenchSet
	Eigen::MatrixXi C; //!< permutations
	Eigen::MatrixXi C_komp; //!< complement to the permutation matrix C

	// depend on the pose r, R
	MatrixXd N; //!< normal vectors
	MatrixXd d; //!< distances

protected:
	//! factorial function, e.g. for calculating binomial coefficients
	int factorial(int n);

public:
	explicit CWrenchSet(CRobotData& robot);
	//! calculate the hyperplane representation for a pose
	bool getHyperPlaneRepresentation(const Vector3d& r, const Matrix3d& R);
	//! check if a wrench is feasible under the actual HyperPlaneRepresentation
	bool checkPoint(MatrixXd& w);
	//! check if the edges of a box are feasible (not implemented yet)
	bool checkBox();
	//! check if a wrench described by an ellipsoid is feasible under the actual HyperPlaneRepresentation
	bool checkEllipsoid(MatrixXd& a);
	//! test pose if it lies in the workspace. 
	bool testPose(const Vector3d& r, const Matrix3d& R);
};

/*! VelocitySet is designed to applied the concept of a wrenchset to the
 *  velocity space providing similar functions to velocities transmission
 *  as it is available for force transmission.
 *  More precisely speaking, the name should be TwistSet (as velocities are
 *  understood to be both linear and angular)
 *  \remark Implementation status is beta.
 */
class CVelocitySet : public CStructureMatrix
{
protected:
	Eigen::MatrixXd C;					//!< permutations
	Eigen::MatrixXd velocitySet;		//!< platform velocity set
	Eigen::MatrixXd cablesVelocity;		//!< cables' velocity

protected:
	void setC();						//!< permutation matrix

public:
	explicit CVelocitySet(CRobotData& robot);
	
	//! generates cables' velocity and test if it's feasible
	bool testPose(const Vector3d& r, const Matrix3d& R);	//!< velocity set -> cables' velocity
	//! get cables's velocity calculated by testPose
	void getCablesVelocity(MatrixXd& velocity) {velocity=cablesVelocity;}

	//! set velocity set to be tested
	void setVelocitySet(MatrixXd vs)	{velocitySet=vs;}
	//! get defined velocity set
	void getVelocitySet(MatrixXd& vs)	{vs=velocitySet;}

	//! calculate required max cables' velocity in a box
	double getMaxCablesVelocityBox(const Vector3d& minBox, const Vector3d& maxBox, const Vector3d& eps);
};

/*! \class CStiffness
 *  The class represents the pose dependent stiffness matrix of the cable robot.
 *  The stiffness matrix is deduced from the structure matrix.
 */
class CStiffness : public CStructureMatrix 
{
protected:
	double k;				//!< specific spring constant of the cables
	double requiredStiffness;	//!< required minimal robot stiffness
	MatrixXd l_additional;  //!< additional cable length between drum and point Ai
	MatrixXd K;				//!< the stiffness matrix 
	MatrixXd K_total;		//!< the stiffness matrix for cable stiffness and geometrical stiffness
	MatrixXd K_g;			//!< the stiffness matrix for geometrical stiffness
	MatrixXd K_c;			//!< the stiffness matrix for cable stiffness

public:
	explicit CStiffness(CRobotData& robot);
	
	//! set minimal required robot stiffness
	void setRequiredStiffness(const double x) {this->requiredStiffness=x;}
	//! get minimal required robot stiffness
	double getRequiredStiffness(){return this->requiredStiffness;}

	//! calculation of the stiffness matrix for pose (r,R)
	bool StiffnessMatrix(const Vector3d& r, const Matrix3d& R);
	//! calculation of the stiffness matrix for pose (r,R): cable + geometrical stiffness
	bool StiffnessMatrixMain(const Vector3d& r, const Matrix3d& R, const MatrixXd& f);
	//! calculation of the geoemtrical stiffness matrix for pose (r,R) and cable forces f
	bool GeometricalStiffnessMatrix(const Vector3d& r, const Matrix3d& R, const MatrixXd& f);
	//! calculate the stiffness matrix and apply the transformation to make in homogenous
	bool StiffnessMatrixHomogenous(const Vector3d& r, const Matrix3d& R);
	//! calculation of the translational stiffness matrix for pose (r,R)
	bool StiffnessMatrixTranslational(const Vector3d& r, const Matrix3d& R);
	//! return the translational stiffness matrix 
	void getStiffnessMatrixTranslational(Matrix3d& K_);
	//! get minimal stiffness (i.e. the smallest eigenvalues of K)
	double getMinimalStiffness(int i=0);
	//! set the specific spring constant of the cables
	void setStiffnessCoefficient(const double& k_) {k = k_;}
	//! set the additional cable length between drum and point Ai
	void setAdditionalCableLength(const int i, const double& l_additional_) {l_additional(i) = l_additional_;}
	//! return the stiffness matrix
	MatrixXd getStiffnessMatrix() { return K; }
	//! some example calculations
	bool examples();
	//! the following function prints the cable stiffness
	bool printWireStiffness(const Vector3d& r, const Matrix3d& R);
	//! test pose and compare to minimal stiffness (for grid tasks)
	bool testPose(const Vector3d& r, const Matrix3d& R);
};

/*! \class CDexterity
 *  At the moment this is only a thin concept of what can be implemented.
 */ 
class CDexterity : public CStructureMatrix 
{
public:
	explicit CDexterity(CRobotData& robot) : CStructureMatrix(robot) {}
};

/*! \class CCableSagging
 *  This class provides pose properties for sagging cables in order to analyze
 *  the cable behaviour at a given position
 */
class CCableSagging : public CForceDistribution
{
	CCableParameter *pCable;	//!< the current cable properties
public:
	CCableSagging(CRobotData& robot, CCableParameter& cable) : CForceDistribution(robot),pCable(&cable) {}
	~CCableSagging() {}

	//! analyze the pose with respect to sagging and print the results
	bool analyzePose(const Vector3d& r, const Matrix3d& R);
	//! static function returning the maximum sagging
	static double getMaxSag(const double& H, const double& l, const double& g);
	//! static function returning the length difference between the sagging and straight cable
	static double getMaxElongation(const double& H, const double& l, const double& g);
};

} // end namespace PCRL