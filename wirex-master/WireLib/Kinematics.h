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

/*! \file Kinematics.h
 *
 *	\author   Andreas Pott
 *
 *  \dependency
 *		eigen3		for IR3 algebra
 *		LevMarLib	for solving the overconstraint direct kinematics
 */

#pragma once

#include <list>
#include "Algorithm.h"
#include "EigenLib.h"
#include <motionPlanning/Utilities.h>
#include "PoseList.h"
#include "EnumTables.h"
#include "levmar.h"
#include <Eigen/unsupported/Eigen/LevenbergMarquardt>

namespace PCRL {

/*! \class CKinematics
 *  Implement algorithms to compute for the forward and inverse kinematics of parallel cable robots
 *  Inverse kinematics is straight forward (at least for the simplified model). Forward kinematics
 *  is much more involved and requires solving systems of nonlinear equations. 
 */
class CKinematics : public CPoseProperty
{
private:
	MatrixXd wireLength;			//!< internal storage for the actual wire length (private, because only used in iterations of the forward kinematics)
	mutable Vector3d* base;		//!< pointer to pivot points on the base (this points to the original data in CRobotData)
	mutable Vector3d* platform;	//!< pointer to pivot points on the platform (this points to the original data in CRobotData)
	mutable Matrix3d* R_base;	//!< pointer to orientation matrix of the base frame's winches; only used for advanced kinematic codes

protected:
	//! renew the pointer after a changes in the associated CRobotData object
	double *info; //double info[LM_INFO_SZ];				//!< information regarding the minimization
	void setPointer() const;

public:
	explicit CKinematics(CRobotData& robot);
	~CKinematics();

	//! calculate the actually used range of the wires within a workspace box
	bool getWireRangeForBox(MatrixXd& Lmin, MatrixXd& Lmax, const Vector3d& Min, const Vector3d& Max, const Matrix3d& R) const;
	bool getWireRangeForBoxDriver( const Vector3d& Min, const Vector3d& Max, double* dMax=0) const;

	void poseestimateMain(const MatrixXd& l, MatrixXd& p, MatrixXd& ub, MatrixXd& lb, Vector3d& r, Matrix3d& R, emPoseEstimate::Type m_nPoseEstimator); //!< pose estimation for kinematic algorithms 
	void poseestimate(const MatrixXd& l, MatrixXd& p);
	void poseestimate(const MatrixXd& l, MatrixXd& p, MatrixXd& ub, MatrixXd& lb);
	void poseestimateInherit(const MatrixXd& l, MatrixXd& p, Vector3d& r, Matrix3d& R); //!< takes initial values given in calls for kinematic function
	void poseestimate_intersectionmethod(const MatrixXd& l, MatrixXd& p);//, Vector3d& r);
		
	//! Maximum Number of Iterations for Numerical Solvers
	int m_nItmax;		

	//! Algorithm Type to be used for LevenbergMarquardth (levmar) and Gauss Newton solvers
	emForwardAlgorithm::Type m_nAlgType;
	void setSolverAlgorithm(const emForwardAlgorithm::Type solverType) {m_nAlgType = solverType;}

	//! Approximation used for numerical methods
	emPoseEstimate::Type m_nPoseEstimator;
	emForwardModel::Type m_nForwardSolverType;
	emInverseModel::Type m_nInverseSolverType;
		
	//! calculate the kinematics
	bool doForwardKinematics(const MatrixXd& l, Vector3d& r, Matrix3d& R) {return doForwardKinematics( l, r, R , m_nItmax, m_nAlgType, m_nPoseEstimator);}
	bool doForwardKinematics(const MatrixXd& l, Vector3d& r, Matrix3d& R, int itmax, emForwardAlgorithm::Type m_nAlgType, emPoseEstimate::Type m_nPoseEstimator);
	
	bool doForwardKinematics_preconditionedabc(const MatrixXd& l, Vector3d& r, Matrix3d& R){return doForwardKinematics_preconditionedabc( l, r, R , m_nItmax, m_nAlgType, m_nPoseEstimator);};
	bool doForwardKinematics_preconditionedabc(const MatrixXd& l, Vector3d& r, Matrix3d& R, int itmax, emForwardAlgorithm::Type m_nAlgType, emPoseEstimate::Type m_nPoseEstimator);
	
	bool doForwardKinematics_geometrytransform(const MatrixXd& l, Vector3d& r, Matrix3d& R) {return doForwardKinematics_geometrytransform( l, r, R , m_nItmax, m_nAlgType, m_nPoseEstimator);}
	bool doForwardKinematics_geometrytransform(const MatrixXd& l, Vector3d& r, Matrix3d& R, int itmax, emForwardAlgorithm::Type m_nAlgType, emPoseEstimate::Type m_nPoseEstimator);
	
	bool doForwardKinematicsPulley(const MatrixXd& l, Vector3d& r, Matrix3d& R){return doForwardKinematicsPulley( l, r, R , m_nItmax, m_nAlgType, m_nPoseEstimator);};
	bool doForwardKinematicsPulley(const MatrixXd& l, Vector3d& r, Matrix3d& R, int itmax, emForwardAlgorithm::Type m_nAlgType, emPoseEstimate::Type m_nPoseEstimator);
	
	//! calculate forward kinematics based on redundant distance constraints
	bool doForwardKinematicsDist(const MatrixXd& l, Vector3d& r, Matrix3d& R){return doForwardKinematicsDist( l, r, R , m_nItmax, m_nAlgType, m_nPoseEstimator);}
	bool doForwardKinematicsDist(const MatrixXd& l, Vector3d& r, Matrix3d& R, int itmax, emForwardAlgorithm::Type m_nAlgType, emPoseEstimate::Type m_nPoseEstimator);
	
	bool doInverseKinematicsMain(const Vector3d& r, const Matrix3d& R, double* l, double* lf=0, double* beta=0, double* gamma=0, Vector3d* C=0, Vector3d* U=0) const; //!< Calculate inverse kinematics
	bool doInverseKinematics(const Vector3d& r, const Matrix3d& R, MatrixXd& l) const;
	bool doInverseKinematicsPulley(const Vector3d& r, const Matrix3d& R, MatrixXd& l) const;
	bool doInverseKinematicsPulleyEx(const Vector3d& r, const Matrix3d& R, double* l=0, double* lf=0, double* beta=0, double* gamma=0, Vector3d* C=0, Vector3d* U=0) const;
	bool doInverseKinematicsVelocity(const Vector3d&r, const Matrix3d& R, const Vector3d& v, const Vector3d& omega, double* dl);
	bool doInverseKinematicsVelocity(const Vector3d&r, const Matrix3d& R, const Vector3d& v, const Vector3d& omega, MatrixXd& dl);
	bool doInverseKinematicsVelocity(const CPoseKinetostatic& poseKinetostatic, MatrixXd& dl);
	bool doInverseKinematicsAcceleration(const Vector3d&r, const Matrix3d& R, const Vector3d& v, const Vector3d& omega, const Vector3d& a, const Vector3d& alpha, MatrixXd& ddl);
	bool doInverseKinematicsAcceleration(const CPoseKinetostatic& poseKinetostatic, MatrixXd& ddl);
	void printZeroLength();

	//! compute the potential energy of the robot
	double getPotentialEnergy(const Vector3d& r, const Matrix3d& R, const MatrixXd& l);

	void bind();

	//! evaluate the inverse kinematics and compare the results with the minimum and maximum cable length
	bool testWorkspaceWireLength(const Vector3d& r, const Matrix3d& R);

	//! evaluate the pose
	bool testPose(const Vector3d& r, const Matrix3d& R);

	// get algorithm information on forward kinematics (only meaningful after calls to doForwardKinematics)
	double getInitalObjectiveFunctionError() const { return info[0]; }  //!< get the initial error of the objective function for found paramter vector
	double getFinalObjectiveFunctionError() const { return info[1]; }	//!< get the final error of the objective function for found paramter vector
	double getFinalJacobianNorm() const { return info[2]; }	//!< get the infinite norm of the Jacobian for the found paramter vector
	double getDeltaPose() const { return info[3]; }			//!< get the norm of the last parameter improvement
	double getIterations() const { return info[5]; }		//!< get the number of iterations
	double getTerminationCode() const { return info[6]; }	//!< get reason for termination; the codes can be taken from the source code
	double getFunctionEvals() const { return info[7]; }		//!< number of evaluations of the objective function
	double getJacobianEvals() const { return info[8]; }		//!< number of evaluations of the Jacobian
	bool getLevmarInfo(double* pInfo) {memcpy(pInfo,this->info,LM_INFO_SZ*sizeof(double)); return true;}	//!< get pointer to levmar info

protected:
	// callback functions for levmar optimizer (solver for direct kinematics)
	// standard kinemtic model
	static void fnc(double *p, double *hx, int m, int n, void *adata);
	void fnc2(double *p, double *hx, int m, int n);
	static void jacf(double *p, double *hx, int m, int n, void *adata);
	void jacf2(double *p, double *hx, int m, int n);
	// standard kinematic model with pre-conditioning of rotation angles
	static void fnc_preconabc(double *p, double *hx, int m, int n, void *adata);
	void fnc_preconabc2(double *p, double *hx, int m, int n);
	static void jacf_preconabc(double *p, double *hx, int m, int n, void *adata);
	void jacf_preconabc2(double *p, double *hx, int m, int n);
	// pulley model
	static void fncPu(double *p, double *hx, int m, int n, void *adata);
	void fncPulley(double *p, double *hx, int m, int n);
	static void jacf8Pu(double *p, double *hx, int m, int n, void *adata);
	void jacfPulley(double *p, double *jac, int m, int n);

	// additional "Callback" objects used for the template/OOP like implementation of levenberg-marquardt in eigen3

	/* this class is required as interface to map the internal call back methods for standard model forward kinematics
	* to the respecitve functor class required by eigens levmar implementation
	* we simply forward the calls to the member fucntions of CKinematics
	* \todo we need to make this more variable in terms of size and improve elegance of eigen usage
	*/
	struct StdModelFunctor : Eigen::DenseFunctor<double>
	{
		PCRL::CKinematics *pKin;
		StdModelFunctor(PCRL::CKinematics &Kin) : DenseFunctor<double>(6, 8), pKin(&Kin) {}
		int operator()(/*const*/ VectorXd &x, VectorXd &fvec) const
		{
			pKin->fnc2(x.data(), fvec.data(), 8, 6);
			return 0;
		}

		int df(/*const*/VectorXd &x, MatrixXd &fjac) const
		{
			// the two inplace transpose commands compromise performance but match the format 
			// to comply with the structure of the expected Jacobian (it somewhat converts 
			// between row-major and column-major storage)
			fjac.transposeInPlace();
			pKin->jacf2(x.data(), fjac.data(), 8, 6);
			fjac.transposeInPlace();
			return 0;
		}
	};

	struct StdPreConFunctor : Eigen::DenseFunctor<double>
	{
		PCRL::CKinematics *pKin;
		StdPreConFunctor(PCRL::CKinematics &Kin) : DenseFunctor<double>(6, 8), pKin(&Kin) {}
		int operator()(/*const*/ VectorXd &x, VectorXd &fvec) const
		{
			pKin->fnc_preconabc2(x.data(), fvec.data(), 8, 6);
			return 0;
		}

		int df(/*const*/VectorXd &x, MatrixXd &fjac) const
		{
			fjac.transposeInPlace();
			pKin->jacf_preconabc2(x.data(), fjac.data(), 8, 6);
			fjac.transposeInPlace();
			return 0;
		}
	};

	struct PulleyModelFunctor : Eigen::DenseFunctor<double>
	{
		PCRL::CKinematics *pKin;
		PulleyModelFunctor(PCRL::CKinematics &Kin) : DenseFunctor<double>(6, 8), pKin(&Kin) {}
		int operator()(/*const*/ VectorXd &x, VectorXd &fvec) const
		{
			pKin->fncPulley(x.data(), fvec.data(), 8, 6);
			return 0;
		}

		int df(/*const*/VectorXd &x, MatrixXd &fjac) const
		{
			fjac.transposeInPlace();
			pKin->jacfPulley(x.data(), fjac.data(), 8, 6);
			fjac.transposeInPlace();
			return 0;
		}
	};

	// experimental distance equations
	static void fnc_dist(double *p, double *hx, int m, int n, void *adata);
	void fnc_dist2(double *p, double *hx, int m, int n);
};


/*! The class provides some functions to compute the elasto geometrical model of the robot
 */
class CElastoKinematics : public CKinematics 
{
private:
	MatrixXd k_spec;		//!< local array of spring constants (for performance reasons)
	MatrixXd l_nominal;		//!< local storage for the nominal cable length, which is needed for the forward kinematics function
	MatrixXd f_resultant;	//!< cable forces in the static equilibrium

public:
	explicit CElastoKinematics(CRobotData& robot);
	~CElastoKinematics();

	//! compute the platform pose under the consideration of cable forces
	bool doElastoGeometricalForwardKinematics(const MatrixXd& l_nominal, Vector3d& r, Vector3d& vEuler, MatrixXd& f, int itmax = 100);
	//! compute cable forces
	bool computeCableForces(const MatrixXd& l_nominal, const Vector3d& r, const Matrix3d& R, MatrixXd& f, Vector3d* u);
	//! compute resultant wrench w_res form the cable forces f and applied wrench w (w_res = AT*f+w)
	bool computeResultantPlatformWrench(MatrixXd& w_res, const MatrixXd& f, const MatrixXd& w,const Vector3d& r, const Matrix3d& R, const Vector3d* u);
	//! callback functions for levmar optimizer (solver for direct kinematics)
	static void fncElastoKin_static(double *p, double *hx, int m, int n, void *adata);
	void fncElastoKin(double *p, double *hx, int m, int n);
};


/*! This class represents a single sagging cable to be used with the 
 *  elastic cable model.
 */
class CCable
{
protected:
	const double* pL0;			//!< nominal cable length (use pointer to avoid copy operation)
	const Vector3d* P1;			//!< fix points
	const Vector3d* P2;
	double h,v;					//!< horizontal and vertical distance between P1, P2
	double mpu;					//!< mass per unit length [N/m]
	double EA0;					//!< cable stiffness (k=EA0/L0)
	double Fx_P2, Fz_P2;		//!< Forces on the second anchor point P2

public:
	CCable(const Vector3d* P1, const Vector3d* P2, const double* l_nominal, const double& mpu, const double& EA0);
	CCable(const double& mpu, const double& EA0);
	~CCable();

	bool set_Parameters(const Vector3d* P1, const Vector3d* P2, const double* l_nominal, const int& simulationMode);
	bool get_Catenary(const int& nPoints, Vector3d* samplePoints);
	bool compute_elasticCatenary(const double& s, double& x, double& z,const double& L0, const  double& FxP2,const double& FzP2);
	bool compute_CableForce(const double& s, Vector3d& u, double& f);
	bool compute_AnchorageForces(Vector3d& u1, Vector3d& u2, double& f1, double& f2);

	// callback functions for levmar optimizer (solver for catenary parameters)
	static void  fncCateneryParam_static(double *p, double *hx, int m, int n, void *adata);
	void  fncCateneryParam(double *p, double *hx, int m, int n);
	static void  fncElasticCateneryParam_static(double *p, double *hx, int m, int n, void *adata);
	void  fncElasticCateneryParam(double *p, double *hx, int m, int n);
	double compute_dxds(const double& s, const double& L0, const double& FxP2,const double& FzP2);
	double compute_dzds(const double& s, const double& L0, const double& FxP2,const double& FzP2);
	double computeTension(const double& s,const double& L0, const double& FxP2, const double& FzP2);
	double compute_phi(const double& s, const double& L0, const double& FxP2,const double& FzP2);
	double atanh(double x);
	double asinh(double x);
};


/*! CCableCain represents a sequence of sagging cables, e.g. for modeling
 *  the sagging between a number of guiding pulleys. 
 */
class CCableChain : private CCable
{
private:
	const Vector3d* pSP; // list of support points
	const int* nSP; // number of support points
	const double* L0_tot;
	double* ph;
	double* pv;
	double* pFx; 
	double* pFz; 
	double* pL0_single; // nominal length of the individual catenaries
	double f_ini;

public:
	CCableChain(const int* nSupportPoints, const double& mpu, const double& EA0, const double& f_ini);
	~CCableChain();

	bool set_Parameters(const Vector3d* supportPointList, const double* l_nominal, const int& simulationMode);
	bool get_Catenary(const int& cableIndex, const int& nPoints, Vector3d* samplePoints); // get sample points for catenary i
	bool get_Catenary(const int& nPoints, Vector3d* samplePoints); // get sample points for all catenaries
	bool compute_CableForce(const int& cableIndex, const double& s, Vector3d& u, double& f);
	bool compute_AnchorageForces(const int& cableIndex, Vector3d& u1, Vector3d& u2, double& f1, double& f2);

	static void fncElasticCateneryChainParam_static(double *p, double *hx, int m, int n, void *adata);
	void  fncElasticCateneryChainParam(double *p, double *hx, int m, int n);

	//bool setNominalLength(const double& L0_tot);
	//bool addPoint(const Vector3d* point);
	//bool insertPoint(const Vector3d* point, const int& i);
	//bool replacePoint(const Vector3d* point);
	//Vector3d* getSupportPoint(const int& i);
};

} // end namespace PCRL
