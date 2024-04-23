/*
* WireX  -  motionPlanning
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

/*! \file NcInterpolator.h
 *
 *	\author   Andreas Pott, Werner Kraus
 *
 *  The class CNcInterpolator takes a NC-program and samples the program
 *  to form a time serie of poses that can be used for e.g. simulation 
 *  and visualization
 */

#pragma once
#include <motionPlanning/NcProgram.h>
#include <vector>
#include <WireLib/EigenLib.h>
#include <WireLib/PoseList.h>
#include <WireLib/StructureMatrix.h>


namespace PCRL {

struct StateDynWinches{MatrixXd f_wires; MatrixXd  v_wires; MatrixXd a_wires; MatrixXd M_motor; MatrixXd omega_motor; MatrixXd alpha_motor;
StateDynWinches(int now){f_wires=MatrixXd::Zero(now,1);v_wires=MatrixXd::Zero(now,1);a_wires=MatrixXd::Zero(now,1);M_motor=MatrixXd::Zero(now,1);omega_motor=MatrixXd::Zero(now,1);alpha_motor=MatrixXd::Zero(now,1);}

};
//struct StateDynWinch{double f_wires[8]; double  v_wires[8]; double a_wires[8]; double M_motor[8]; double omega_motor[8]; double alpha_motor[8];};
struct NcPermutation{double dOverride; double amax; double jmax; double f_ref; bool bForce; bool  bSpeed; bool  bTorquePeak; bool  bTorqueRMS; double trajectoryTime;};



class CPathSegments
{
public:
	virtual bool getTrajectoryPoint(const double t_, CPoseKinetostatic* poseKinetostatic){return true;};
	virtual bool getStateDynamic(CPoseListKinetostatic& sStateDyn){return true;};
	double Ramp(double v0, double v1, double vmax, double amax, double jmax, double dist, int nr);
	virtual bool calculateTrajectory(){return true;};
public:
	double t[4][8], s[4][8], v[4][8], a[4][8], j[4][8]; // [1]=position, [2-4]=orientation
	double amax, jmax;
};

class CStraightLine: public CPathSegments
{
	public:
	CStraightLine(){v0=v1=vmax=0;}
public:
	bool calculateTrajectory();
	bool getTrajectoryPoint(const double t_, CPoseKinetostatic* poseKinetostatic);
	Vector3d r1,r2; // position start point, position end point
	Vector3d angles1, angles2; // orientation start point, orientation end point
	double getLength(){return (r1-r2).norm();};
	double v0,v1,vmax;
	double duration;
	double scale[4];
};

class CBezier5: public CPathSegments
{
	public:
	CBezier5(){v0=v1=vmax=0;}
public:
	bool calculateTrajectory();
	bool getTrajectoryPoint(const double t_, CPoseKinetostatic* poseKinetostatic) ;
	Vector3d a_coeff[6];
	Vector3d angles1, angles2;
	double v0,v1,vmax,u_point_max;
	bool getr1(Vector3d& r1){r1=a_coeff[0]; return true;};
	bool getr2(Vector3d& r2){r2=a_coeff[0]+a_coeff[1] + a_coeff[2] + a_coeff[3] + a_coeff[4] + a_coeff[5]; return true;};
};

class CDwell: public CPathSegments
{
	public:
	CDwell(){;}
public:
	bool calculateTrajectory();
	bool getTrajectoryPoint(const double t_, CPoseKinetostatic* poseKinetostatic);
	Vector3d r1;
	Vector3d angles;
	double dwellTime;
	double getLength(){return 0.0;};
	double duration;
};

class CNcInterpolator//: public CAlgorithm 
{
public:
	CNcInterpolator();
	~CNcInterpolator(void);
	CNcProgram* pProgram;
	double cycleTime;		//!< the cycle time of the interpolator in ms
	double gearRatio;
	double amax, jmax;
	CRobotData* m_pRobot;
	double trajectoryTime;
	double trajectoryLength;
	int iNrPoses;
	int iInterpolationMethod;
	int iBezier;
	double dOverride;
	Vector3d StartPosition;
	Vector3d StartOrientation;
	std::vector<Vector3d> LineStrip;
	CPoseListKinetostatic listStateDyn;
public:
	bool setCycleTime(const double& cycleTime);
	bool setgearRatio(const double& gearRatio){this->gearRatio=gearRatio; return true;};	//! \todo added a return true, although rather useless; why does this function return a value?
	double getCycleTime();
	double getTrajectoryTime(double& trajectorytime){trajectoryTime=this->trajectoryTime; return trajectoryTime; }; //! \todo added a return statement. why does this function use call-by-reference rather than just returning the desired value?

	double getTorqueRMS();
	bool setAmax(const double& amax);
	bool setJmax(const double& jmax);
	bool setInterpolationMethod(const int& iInterpolationMethod);
	bool setBezier(const int& iBezier);
	bool setOverride(const double& dOverride);
	bool setStartPosition(const Vector3d StartPosition_);
	bool setStartOrientation(const Vector3d StartOrientation_);

	bool GeometricPathPlanning(const CNcProgram& Program, std::vector<CPathSegments*>& PathSegments);
	bool TrajectoryPlanning(std::vector<CPathSegments*>& PathSegments);
	bool Bezier5Geometric(Vector3d r_a, Vector3d r_b, Vector3d r_c, Vector3d (&a_coeff)[6] , Vector3d& r_b1, Vector3d& r_b2);

	//! the main generator function: interprete the program and generate the desired list of pose
	bool interpolateProgram(const CNcProgram& Program, CPoseListKinetostatic& PoseList);
	bool interpolateProgramStatic(const CNcProgram& Program, CPoseListKinetostatic& PoseList);
	bool interpolateProgramDynamic(const CNcProgram& Program, CPoseListKinetostatic& PoseList);
	
	bool Ramp_upgrade();
	bool PrintPathSegments(std::vector<CPathSegments*>& PathSegments);
	bool PlotPathSegments(std::vector<CPathSegments*>& PathSegments, std::vector<Vector3d>& LineStrip);
	bool InterpolatePathSegments(std::vector<CPathSegments*>& PathSegments, CPoseListKinetostatic& PoseList);
	bool DynamicAnalysisPathSegments(std::vector<CPathSegments*>& PathSegments, CPoseListKinetostatic& sStateDyn);
	bool ExportStateDyn(CPoseListKinetostatic& sStateDyn);
	bool ExportPoseList(CPoseListStatic& PoseList);
	bool calculateWinchTransmission(const double& f_wire, const double& v_wire, const double& a_wire, double& M_motor, double& omega_motor, double& alpha_motor); 
	bool getStateDynWinches(const MatrixXd& f_wires, const MatrixXd& v_wires, const MatrixXd& a_wires, StateDynWinches& sStateDynWinches);
	//bool CalculateTrajectoryParameters(std::list<PCRL::StateDynWinch*>& listStateDynWinch, double* f_min, double* f_max, double* n_min, double* n_max, double* M_min, double* M_max, double* M_RMS);
	//bool EvaluateTrajectory(std::list<PCRL::StateDynWinch*>& listStateDynWinch, bool& bForce, bool& bSpeed, bool& bTorquePeak, bool& bTorqueRMS);
	bool printTrajectoryParameters();
	
};
	

} // end namespace PCRL
