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
 *  \file   : Kinematics.cpp
 *
 *  Project   Wirelib
 *
 *  \Author   Andreas Pott
 *
 *  \Date     10.10.2009  (originally part of the MoWiRoGeometry implementation)
 *			  15.12.2009  (refactored by asp)
 *
 *********************************************************************
 */ 

#include "Kinematics.h"

namespace PCRL {

CKinematics::CKinematics(CRobotData& robot) : CPoseProperty(robot)
{
	wireLength.resize(pRobot->getNow(), 1);
 	setPointer();
	info = new double[LM_INFO_SZ];
	for (int i=0; i<LM_INFO_SZ; i++)
		info[i] = 0;
	
	m_nItmax = 100;
	m_nAlgType = emForwardAlgorithm::levmar_der;
	m_nPoseEstimator = emPoseEstimate::basic;
	m_nForwardSolverType = emForwardModel::pulley;
	m_nInverseSolverType = emInverseModel::pulley;
}

//! internal function to reinit the pointers caching the geometry settings of 
//! the current robot.
void CKinematics::setPointer() const
{
	if (pRobot)
	{
		base = pRobot->base;
		platform = pRobot->platform;
		R_base = pRobot->R_base;
	}
}

	
//! init the parameter to name binding for reflection
void CKinematics::bind()
{
	CPoseProperty::bind();
	if (!pReflector)
		return;
	
	pReflector->bind(m_nItmax,"nItmax","kinematics/@maxIterations");
	pReflector->bind((int&)m_nAlgType,"nAlgorithmType","kinematics/@AlgorithmType");
	pReflector->bind((int&)m_nForwardSolverType,"nForwardSolverType","kinematics/@ForwardSolverType");
	pReflector->bind((int&)m_nInverseSolverType,"nInverseSolverType","kinematics/@InverseSolverType");
	pReflector->bind((int&)m_nPoseEstimator,"nPoseEstimator","kinematics/@PoseEstimator");
	CReflection::addEnumTable("kinematics/@AlgorithmType",emForwardAlgorithm::Names);
	CReflection::addEnumTable("kinematics/@ForwardSolverType",emForwardModel::Names);
	CReflection::addEnumTable("kinematics/@InverseSolverType",emInverseModel::Names);
	CReflection::addEnumTable("kinematics/@PoseEstimator",emPoseEstimate::Names);
}


CKinematics::~CKinematics()
{
	delete [] info;
}


// adapter callback fnc for levenberg marquart optimizer; forward the call
// to the member function
void CKinematics::fnc(double *p, double *hx, int m, int n, void *adata)
{
	((CKinematics*)adata)->fnc2(p,hx,m,n);
}

// the non-static version of the callback function for use in levmar solver.
// compute the quadratic (squred) constrained error imposed by ideal stiff 
// cables. Note, that the constrained error is return rather than an estimate 
// of energy. The errors are compiled for each cable in the i-th entry of hx[].
// The rational behind this approach is described in the ARK2010 paper by Pott.
void CKinematics::fnc2(double *p, double *hx, int m, int n)
{
/*	Vector3d r(p[0],p[1],p[2]);
	Matrix3d R(1,0,0,0,1,0,0,0,1); 
	R = Matrix3d::ZRotationMatrix3d(p[3])*Matrix3d::YRotationMatrix3d(p[4])*Matrix3d::XRotationMatrix3d(p[5]);
	
	for (int i=0; i<now; i++)
		hx[i] = length(r+R*platform[i]-base[i])-wire_length[i]; // original: works
//		hx[i] = squaredLength(r+R*platform[i]-base[i])-wire_length[i]*wire_length[i]; // seems to work as well
*/

	// maple generated version of the system of equations above;
	// slightly hacked to access the internal variables platform[i] and base[i]
	double t1 = cos(p[3]);
	double t2 = cos(p[4]);
	double t5 = sin(p[3]);
	double t6 = cos(p[5]);
	double t8 = sin(p[4]);
	double t9 = t1 * t8;
	double t10 = sin(p[5]);
	for (int i=0; i<pRobot->now; i++)
	{
		double t19 = pow(p[0] + t1 * t2 * platform[i].x() + (-t5 * t6 + t9 * t10) * platform[i].y() + (t5 * t10 + t9 * t6) * platform[i].z() - base[i].x(), 0.2e1);
		double t23 = t5 * t8;
		double t32 = pow(p[1] + t5 * t2 * platform[i].x() + (t1 * t6 + t23 * t10) * platform[i].y() + (-t1 * t10 + t23 * t6) * platform[i].z() - base[i].y(), 0.2e1);
		double t39 = pow(p[2] - t8 * platform[i].x() + t2 * t10 * platform[i].y() + t2 * t6 * platform[i].z() - base[i].z(), 0.2e1);
		hx[i] = t19 + t32 + t39 - wireLength(i)*wireLength(i);
	}
}


// adapter callback fnc for levenberg marquart optimizer; forward the call
// to the member function
void CKinematics::fnc_preconabc(double *p, double *hx, int m, int n, void *adata){((CKinematics*)adata)->fnc_preconabc2(p,hx,m,n);
}

// the non-static version of the callback function
void CKinematics::fnc_preconabc2(double *p, double *hx, int m, int n)
{
	// maple generated version of the system of equations;
	// slightly hacked to access the internal variables platform[i] and base[i]
	double a,b,c;
	a=(p[3]*2*MO_PI)/(abs(p[3])+1);
	b=(p[4]*2*MO_PI)/(abs(p[4])+1);
	c=(p[5]*2*MO_PI)/(abs(p[5])+1);

	double t1 = cos(a);
	double t2 = cos(b);
	double t5 = sin(a);
	double t6 = cos(c);
	double t8 = sin(b);
	double t9 = t1 * t8;
	double t10 = sin(c);
	for (int i=0; i<pRobot->now; i++)
	{
		double t19 = pow(p[0] + t1 * t2 * platform[i].x() + (-t5 * t6 + t9 * t10) * platform[i].y() + (t5 * t10 + t9 * t6) * platform[i].z() - base[i].x(), 0.2e1);
		double t23 = t5 * t8;
		double t32 = pow(p[1] + t5 * t2 * platform[i].x() + (t1 * t6 + t23 * t10) * platform[i].y() + (-t1 * t10 + t23 * t6) * platform[i].z() - base[i].y(), 0.2e1);
		double t39 = pow(p[2] - t8 * platform[i].x() + t2 * t10 * platform[i].y() + t2 * t6 * platform[i].z() - base[i].z(), 0.2e1);
		hx[i] = t19 + t32 + t39 - wireLength(i)*wireLength(i);
	}
}

// adapter callback jacf for Levenberg-Marquart optimizer; forward the call
// to the member function
void CKinematics::jacf_preconabc(double *p, double *hx, int m, int n, void *adata)
{
	((CKinematics*)adata)->jacf_preconabc2(p,hx,m,n);
}

// the non-static version of the Jacobian callback function
void CKinematics::jacf_preconabc2(double *p, double *jac, int m, int n)
{
	// maple generated version of the system of equations;
	// slightly hacked to access platform[i] and base[i]
	double a,b,c;
	a=(p[3]*MO_PI)/(abs(p[3])+1);
	b=(p[4]*MO_PI)/(abs(p[4])+1);
	c=(p[5]*MO_PI)/(abs(p[5])+1);

	double t1 = cos(a);
	double t2 = cos(b);
	double t3 = t1 * t2;
	double t5 = sin(a);
	double t6 = cos(c);
	double t8 = sin(b);
	double t10 = sin(c);
	for (int i=0,j=0; i<pRobot->getNow(); i++)
	{
		double t4 = t3 * platform[i].x();
		double t9 = t1 * t8;
		double t12 = -t5 * t6 + t9 * t10;
		double t13 = t12 * platform[i].y();
		double t16 = t5 * t10 + t9 * t6;
		double t17 = t16 * platform[i].z();
		double t18 = p[0] + t4 + t13 + t17 - base[i].x();
		double t19 = t5 * t2;
		double t20 = t19 * platform[i].x();
		double t22 = t5 * t8;
		double t24 = t1 * t6 + t22 * t10;
		double t28 = -t1 * t10 + t22 * t6;
		double t30 = p[1] + t20 + t24 * platform[i].y() + t28 * platform[i].z() - base[i].y();
		double t32 = t2 * t10;
		double t34 = t2 * t6;
		double t36 = p[2] - t8 * platform[i].x() + t32 * platform[i].y() + t34 * platform[i].z() - base[i].z();
		double t45 = t10 * platform[i].y();
		double t47 = t6 * platform[i].z();
		jac[j++] = 0.2e1 * t18;
		jac[j++] = 0.2e1 * t30;
		jac[j++] = 0.2e1 * t36;
		jac[j++] = (0.2e1 * t18 * (-t20 - t24 * platform[i].y() - t28 * platform[i].z()) + 0.2e1 * t30 * (t4 + t13 + t17)) * (2*MO_PI)/pow(abs(p[3])+1,2);
		jac[j++] = (0.2e1 * t18 * (-t9 * platform[i].x() + t3 * t45 + t3 * t47) + 0.2e1 * t30 * (-t22 * platform[i].x() + t19 * t45 + t19 * t47) + 0.2e1 * t36 * (-t2 * platform[i].x() - t8 * t10 * platform[i].y() - t8 * t6 * platform[i].z())) * (2*MO_PI)/pow(abs(p[4])+1,2);
		jac[j++] = (0.2e1 * t18 * (t16 * platform[i].y() - t12 * platform[i].z()) + 0.2e1 * t30 * (t28 * platform[i].y() - t24 * platform[i].z()) + 0.2e1 * t36 * (t34 * platform[i].y() - t32 * platform[i].z())) * (2*MO_PI)/pow(abs(p[5])+1,2);
	}
}

//! adapter callback fnc for levenberg marquart optimizer; forward the call
//! to the member function
void CKinematics::fncPu(double *p, double *hx, int m, int n, void *adata)
{
	((CKinematics*)adata)->fncPulley(p,hx,m,n);
}

//! Function for optimizer --> derived from doInverseKinematicsPulley
//! \todo only valid for 8 wires and with a identical radius of all pulleys
void CKinematics::fncPulley(double *p, double *hx, int m, int n)
{
	Vector3d r;
	r << p[0],p[1],p[2];
	Matrix3d R;
	R.setIdentity(); 
	R = Matrix3d::ZRotationMatrix3d(p[3])*Matrix3d::YRotationMatrix3d(p[4])*Matrix3d::XRotationMatrix3d(p[5]);
	
	MatrixXd l(pRobot->getNow(), 1);
	Matrix3d A;
	double rp;
	
	bool res=true;
	for (int i=0; i<pRobot->getNow(); i++)
	{
		rp=pRobot->r_pulley[i];	//!< radius of the pulley
		Vector3d AP = r+R*platform[i] - base[i];		//Find the vector from base to platform point of cable connect (A to P)
		Vector3d AM = (R_base[i].col(2).cross(AP)).cross(R_base[i].col(2));	//Find Vector from base to centre of winch where e1 is assumed to be vector of rotation matrix along the winch axis
		Vector3d AMnorm = Vector3d(AM.x()/AM.norm(),AM.y()/AM.norm(),AM.z()/AM.norm());
		Vector3d M = base[i] + rp*AMnorm;			//Vector defining point M=A + r*unitvector(AM)
		double a = AP.norm();						//length from A to P
		double b = (r+R*platform[i]-M).norm();		//length from M to P
		double theta = acos((b * b + rp * rp - a * a) / (2 * b * rp)) - acos(rp / b);	//finding 'hug'angle Umschlingunswinkel
		double thetax = 2 * (MO_PI) - theta - 2 * acos(rp / b);					//the other possible 'hug'angle 

		if ((R_base[i].col(2).dot(AP))>0)
		{
			l(i)=theta * rp + sqrt(rp *rp + b * b);
		}
		else 
		{
			l(i)=thetax * rp + sqrt(rp * rp + b * b);
		}
		hx[i]=l(i)*l(i)-wireLength(i)*wireLength(i);
	}
}


// adapter callback jacf for Levenberg-Marquart optimizer; forward the call
// to the member function
void CKinematics::jacf(double *p, double *hx, int m, int n, void *adata)
{
	((CKinematics*)adata)->jacf2(p,hx,m,n);
}

// the non-static version of the Jacobian callback function
void CKinematics::jacf2(double *p, double *jac, int m, int n)
{
	// maple generated version of the system of equations;
	// slightly hacked to access platform[i] and base[i]
	double t1 = cos(p[3]);
	double t2 = cos(p[4]);
	double t3 = t1 * t2;
	double t5 = sin(p[3]);
	double t6 = cos(p[5]);
	double t8 = sin(p[4]);
	double t10 = sin(p[5]);
	for (int i=0,j=0; i<pRobot->getNow(); i++)
	{
		double t4 = t3 * platform[i].x();
		double t9 = t1 * t8;
		double t12 = -t5 * t6 + t9 * t10;
		double t13 = t12 * platform[i].y();
		double t16 = t5 * t10 + t9 * t6;
		double t17 = t16 * platform[i].z();
		double t18 = p[0] + t4 + t13 + t17 - base[i].x();
		double t19 = t5 * t2;
		double t20 = t19 * platform[i].x();
		double t22 = t5 * t8;
		double t24 = t1 * t6 + t22 * t10;
		double t28 = -t1 * t10 + t22 * t6;
		double t30 = p[1] + t20 + t24 * platform[i].y() + t28 * platform[i].z() - base[i].y();
		double t32 = t2 * t10;
		double t34 = t2 * t6;
		double t36 = p[2] - t8 * platform[i].x() + t32 * platform[i].y() + t34 * platform[i].z() - base[i].z();
		double t45 = t10 * platform[i].y();
		double t47 = t6 * platform[i].z();
		jac[j++] = 0.2e1 * t18;
		jac[j++] = 0.2e1 * t30;
		jac[j++] = 0.2e1 * t36;
		jac[j++] = 0.2e1 * t18 * (-t20 - t24 * platform[i].y() - t28 * platform[i].z()) + 0.2e1 * t30 * (t4 + t13 + t17);
		jac[j++] = 0.2e1 * t18 * (-t9 * platform[i].x() + t3 * t45 + t3 * t47) + 0.2e1 * t30 * (-t22 * platform[i].x() + t19 * t45 + t19 * t47) + 0.2e1 * t36 * (-t2 * platform[i].x() - t8 * t10 * platform[i].y() - t8 * t6 * platform[i].z());
		jac[j++] = 0.2e1 * t18 * (t16 * platform[i].y() - t12 * platform[i].z()) + 0.2e1 * t30 * (t28 * platform[i].y() - t24 * platform[i].z()) + 0.2e1 * t36 * (t34 * platform[i].y() - t32 * platform[i].z());
	}
}


// adapter callback jacf for Levenberg-Marquart optimizer for the function with pulley; forward the call
// to the member function
void CKinematics::jacf8Pu(double *p, double *hx, int m, int n, void *adata)
{
	((CKinematics*)adata)->jacfPulley(p,hx,m,n);
}

void CKinematics::jacfPulley(double *p, double *jac, int m, int n)
{
	double rhom = pRobot->r_pulley[0]; // Attention: the radius of pulley 1 is used for all cables
	double t1 = cos(p[4]);
	double t2 = cos(p[5]);
	double t3 = t1 * t2;
	double t5 = sin(p[5]);
	double t6 = t1 * t5;	
	double t8 = sin(p[4]);
	double t12 = cos(p[3]);
	double t13 = t12 * t8;
	double t15 = sin(p[3]);
	double t17 = -t13 * t2 + t15 * t5;
	double t21 = t13 * t5 + t15 * t2;
	double t23 = t12 * t1;
	double t29 = t15 * t8;
	double t32 = t29 * t2 + t12 * t5;
	double t36 = -t29 * t5 + t12 * t2;
	double t38 = t15 * t1;
	for (int i=0,j=0; i<pRobot->getNow(); i++)
	{
		double t4 = t3 * platform[i].x();
		double t7 = t6 * platform[i].y();
		double t9 = t8 * platform[i].z();
		double t10 = p[0] + t4 - t7 + t9 - base[i].x();
		double t18 = t17 * platform[i].x();
		double t22 = t21 * platform[i].y();
		double t24 = t23 * platform[i].z();
		double t25 = p[2] + t18 + t22 + t24 - base[i].z();
		double t27 = R_base[i].col(2).z() * t10 - R_base[i].col(2).x() * t25;
		double t33 = t32 * platform[i].x();
		double t37 = t36 * platform[i].y();
		double t39 = t38 * platform[i].z();
		double t40 = p[1] + t33 + t37 - t39 - base[i].y();
		double t43 = R_base[i].col(2).x() * t40 - R_base[i].col(2).y() * t10;
		double t45 = t27 * R_base[i].col(2).z() - t43 * R_base[i].col(2).y();
		double t46 = fabs(t45);
		double t47 = t46 * t46;
		double t51 = R_base[i].col(2).y() * t25 - R_base[i].col(2).z() * t40;
		double t53 = -t43 * R_base[i].col(2).x() + t51 * R_base[i].col(2).z();
		double t54 = fabs(t53);
		double t55 = t54 * t54;
		double t58 = -t51 * R_base[i].col(2).y() + t27 * R_base[i].col(2).x();
		double t59 = fabs(t58);
		double t60 = t59 * t59;
		double t61 = t47 + t55 + t60;
		double t62 = sqrt(t61);
		double t64 = rhom / t62; //
		double t66 = -p[0] - t4 + t7 - t9 + base[i].x() + t64 * t45;
		double t67 = fabs(t66);
		double t68 = t67 * t67;
		double t70 = p[1] + t33 + t37 - t39 - base[i].y() + t64 * t53;
		double t71 = fabs(t70);
		double t72 = t71 * t71;
		double t74 = p[2] + t18 + t22 + t24 - base[i].z() + t64 * t58;
		double t75 = fabs(t74);
		double t76 = t75 * t75;
		double t77 = rhom * rhom;
		double t78 = t68 + t72 + t76 - t77;
		double t79 = sqrt(t78);
		double t80 = t68 + t72 + t76;
		double t81 = sqrt(t80);
		double t82 = 0.1e1 / t81; //
		double t84 = acos(t79 * t82);
		double t88 = R_base[i].col(2).x() * t10 + t40 * R_base[i].col(2).y() + t25 * R_base[i].col(2).z();
		double t90 = acos(t88 * t82);
		double t93 = (t84 + t90) * rhom + t79;
		double t94 = 0.1e1 / t79; //
		double t95 = t94 * t82;
		double t96 = (t66 > 0) - (t66 < 0);//fabs(t66) / t66; //
		double t97 = t67 * t96;
		double t100 = rhom / t62 / t61; //
		double t101 = (t45 > 0) - (t45 < 0);//fabs(t45) / t45; //
		double t102 = t46 * t101;
		double t103 = pow(R_base[i].col(2).z(), 0.2e1);
		double t104 = pow(R_base[i].col(2).y(), 0.2e1);
		double t105 = t103 + t104;
		double t107 = (t53 > 0) - (t53 < 0);//fabs(t53) / t53; //
		double t108 = t54 * t107;
		double t109 = R_base[i].col(2).x() * R_base[i].col(2).y();
		double t111 = (t58 > 0) - (t58 < 0);//fabs(t58) / t58; //
		double t112 = t59 * t111;
		double t113 = R_base[i].col(2).x() * R_base[i].col(2).z();
		double t115 = t102 * t105 + t108 * t109 + t112 * t113;
		double t122 = (t70 > 0) - (t70 < 0);//fabs(t70) / t70; //
		double t123 = t71 * t122;
		double t127 = t64 * t109;
		double t130 = (t74 > 0) - (t74 < 0); //fabs(t74) / t74; //
		double t131 = t75 * t130;
		double t135 = t64 * t113;
		double t138 = t97 * (-0.1e1 - t100 * t45 * t115 + t64 * t105) + t123 * (-t100 * t53 * t115 + t127) + t131 * (-t100 * t58 * t115 + t135);
		double t141 = 0.1e1 / t81 / t80; //
		double t142 = t79 * t141;
		double t145 = 0.1e1 / t80; //
		double t148 = sqrt(0.1e1 - t78 * t145);
		double t149 = 0.1e1 / t148; //
		double t152 = t88 * t141;
		double t156 = t88 * t88;
		double t159 = sqrt(0.1e1 - t156 * t145);
		double t160 = 0.1e1 / t159; //
		double t170 = pow(R_base[i].col(2).x(), 0.2e1);
		double t171 = -t170 - t103;
		double t173 = R_base[i].col(2).z() * R_base[i].col(2).y();
		double t175 = -t102 * t109 + t108 * t171 + t112 * t173;
		double t190 = t64 * t173;
		double t193 = t97 * (-t100 * t45 * t175 - t127) + t123 * (0.1e1 - t100 * t53 * t175 + t64 * t171) + t131 * (-t100 * t58 * t175 + t190);
		double t212 = -t104 - t170;
		double t214 = -t102 * t113 + t108 * t173 + t112 * t212;
		double t231 = t97 * (-t100 * t45 * t214 - t135) + t123 * (-t100 * t53 * t214 + t190) + t131 * (0.1e1 - t100 * t58 * t214 + t64 * t212);
		double t248 = t33 + t37 - t39;
		double t251 = -t17 * platform[i].x();
		double t252 = -t21 * platform[i].y();
		double t253 = t251 + t252 - t24;
		double t256 = -R_base[i].col(2).x() * t248 * R_base[i].col(2).z() - R_base[i].col(2).x() * t253 * R_base[i].col(2).y();
		double t261 = R_base[i].col(2).y() * t248 - R_base[i].col(2).z() * t253;
		double t263 = -t170 * t253 + t261 * R_base[i].col(2).z();
		double t267 = -t261 * R_base[i].col(2).y() - t170 * t248;
		double t269 = t102 * t256 + t108 * t263 + t112 * t267;
		double t288 = t97 * (-t100 * t45 * t269 + t64 * t256) + t123 * (t251 + t252 - t24 - t100 * t53 * t269 + t64 * t263) + t131 * (t33 + t37 - t39 - t100 * t58 * t269 + t64 * t267);
		double t309 = t8 * t2 * platform[i].x();
		double t311 = t8 * t5 * platform[i].y();
		double t312 = t1 * platform[i].z();
		double t313 = -t309 + t311 + t312;
		double t315 = t2 * platform[i].x();
		double t316 = t23 * t315;
		double t317 = t5 * platform[i].y();
		double t318 = t23 * t317;
		double t319 = t13 * platform[i].z();
		double t320 = -t316 + t318 - t319;
		double t322 = R_base[i].col(2).z() * t313 - R_base[i].col(2).x() * t320;
		double t324 = t38 * t315;
		double t325 = t38 * t317;
		double t326 = t29 * platform[i].z();
		double t327 = t324 - t325 + t326;
		double t330 = R_base[i].col(2).x() * t327 - R_base[i].col(2).y() * t313;
		double t332 = t322 * R_base[i].col(2).z() - t330 * R_base[i].col(2).y();
		double t337 = R_base[i].col(2).y() * t320 - R_base[i].col(2).z() * t327;
		double t339 = -t330 * R_base[i].col(2).x() + t337 * R_base[i].col(2).z();
		double t343 = -t337 * R_base[i].col(2).y() + t322 * R_base[i].col(2).x();
		double t345 = t102 * t332 + t108 * t339 + t112 * t343;
		double t364 = t97 * (t309 - t311 - t312 - t100 * t45 * t345 + t64 * t332) + t123 * (t324 - t325 + t326 - t100 * t53 * t345 + t64 * t339) + t131 * (-t316 + t318 - t319 - t100 * t58 * t345 + t64 * t343);
		double t385 = t6 * platform[i].x();
		double t386 = t3 * platform[i].y();
		double t387 = -t385 - t386;
		double t389 = t21 * platform[i].x();
		double t390 = -t17 * platform[i].y();
		double t391 = t389 + t390;
		double t393 = R_base[i].col(2).z() * t387 - R_base[i].col(2).x() * t391;
		double t395 = t36 * platform[i].x();
		double t396 = -t32 * platform[i].y();
		double t397 = t395 + t396;
		double t400 = R_base[i].col(2).x() * t397 - R_base[i].col(2).y() * t387;
		double t402 = t393 * R_base[i].col(2).z() - t400 * R_base[i].col(2).y();
		double t407 = R_base[i].col(2).y() * t391 - R_base[i].col(2).z() * t397;
		double t409 = -t400 * R_base[i].col(2).x() + t407 * R_base[i].col(2).z();
		double t413 = -t407 * R_base[i].col(2).y() + t393 * R_base[i].col(2).x();
		double t415 = t102 * t402 + t108 * t409 + t112 * t413;
		double t434 = t97 * (t385 + t386 - t100 * t45 * t415 + t64 * t402) + t123 * (t395 + t396 - t100 * t53 * t415 + t64 * t409) + t131 * (t389 + t390 - t100 * t58 * t415 + t64 * t413);
		jac[j++] = 0.2e1 * t93 * ((-(0.2e1 * t95 * t138 - 0.2e1 * t142 * t138) * t149 / 0.2e1 - (R_base[i].col(2).x() * t82 - t152 * t138) * t160) * rhom + t94 * t138);
		jac[j++] = 0.2e1 * t93 * ((-(0.2e1 * t95 * t193 - 0.2e1 * t142 * t193) * t149 / 0.2e1 - (R_base[i].col(2).y() * t82 - t152 * t193) * t160) * rhom + t94 * t193);
		jac[j++] = 0.2e1 * t93 * ((-(0.2e1 * t95 * t231 - 0.2e1 * t142 * t231) * t149 / 0.2e1 - (R_base[i].col(2).z() * t82 - t152 * t231) * t160) * rhom + t94 * t231);
		jac[j++] = 0.2e1 * t93 * ((-(0.2e1 * t95 * t288 - 0.2e1 * t142 * t288) * t149 / 0.2e1 - ((t253 * R_base[i].col(2).y() + t248 * R_base[i].col(2).z()) * t82 - t152 * t288) * t160) * rhom + t94 * t288);
		jac[j++] = 0.2e1 * t93 * ((-(0.2e1 * t95 * t364 - 0.2e1 * t142 * t364) * t149 / 0.2e1 - ((R_base[i].col(2).x() * t313 + t327 * R_base[i].col(2).y() + t320 * R_base[i].col(2).z()) * t82 - t152 * t364) * t160) * rhom + t94 * t364);
		jac[j++] = 0.2e1 * t93 * ((-(0.2e1 * t95 * t434 - 0.2e1 * t142 * t434) * t149 / 0.2e1 - ((R_base[i].col(2).x() * t387 + t397 * R_base[i].col(2).y() + t391 * R_base[i].col(2).z()) * t82 - t152 * t434) * t160) * rhom + t94 * t434);
	}
}


/*! For the forward kinematics optimization algorithms are often used. For these 
 *  an pose estimate is made the following function estimates the pose as the 
 *  center of a guaranteed bounding box. No Orientation is estimated.
 *  Also calculate bounding box (for constrained solvers)
 */
void CKinematics::poseestimateMain(const MatrixXd& l, MatrixXd& p, MatrixXd& ub, MatrixXd& lb, Vector3d& r, Matrix3d& R, emPoseEstimate::Type m_nPoseEstimator)
{
	// Use the pose estimater function specified by m_nPoseEstimator
	switch(m_nPoseEstimator) {
		// Use basic with no bounding box
		case emPoseEstimate::basic:
			poseestimate(l, p);
			break;
		// Use basic with simple bounding box
		case emPoseEstimate::basicSimpleBoundingbox:
			poseestimate(l, p, ub, lb);
			break;
		case emPoseEstimate::inherit:
			poseestimateInherit(l, p, r, R);
			break;
		/* Use basic with more complicated bounding box
		case BASIC_COMPLEX_BOUNDINGBOX:
			poseestimateInt(l, p, ub, lb);
			break;*/
		// same as basicSimpleBoundingbox
		case emPoseEstimate::intersectionMethod:
			poseestimate_intersectionmethod(l, p);//, r);
			break;
		default:
			poseestimate(l, p, ub, lb);
			break;
	}

}

/*!
 * Use the interval intersection method to estimate the pose of the platform. This function
 * makes only estimates for position (x,y,z) in the first three entries of p; the other three
 * elements are set to zero.
 * \param l [in] the current cable length used in the estimator
 * \param p [out] the pose vector (x,y,z,a,b,c) which is estimated by the function
 */
void CKinematics::poseestimate(const MatrixXd& l, MatrixXd& p)
{
	Vector3d rmin,rmax,ones(1,1,1);
	rmin = base[0] - ones*(l(0)+platform[0].norm());
	rmax = base[0] + ones*(l(0)+platform[0].norm());
	for (int i=1; i<pRobot->getNow(); i++)
	{
		Vector3d r0 = base[i] - ones*(l(i)+platform[i].norm());
		if ( r0.x() > rmin.x() ) rmin.x() = r0.x();
		if ( r0.y() > rmin.y() ) rmin.y() = r0.y();
		if ( r0.z() > rmin.z() ) rmin.z() = r0.z();
		r0 = base[i] + ones*(l(i)+platform[i].norm());
		if ( r0.x() < rmax.x() ) rmax.x() = r0.x();
		if ( r0.y() < rmax.y() ) rmax.y() = r0.y();
		if ( r0.z() < rmax.z() ) rmax.z() = r0.z();
	}

	// use the center of the bound box as initial estimate
	p(0) = (rmax.x()+rmin.x())/2.0;
	p(1) = (rmax.y()+rmin.y())/2.0;
	p(2) = (rmax.z()+rmin.z())/2.0; // <<<--- vorsicht
	// estimate the orientation with the identity matrix
	p(3) = p(4)= p(5)= 0;
}

//! Also calculate bounding box (for constrained solvers)
void CKinematics::poseestimate(const MatrixXd& l, MatrixXd& p, MatrixXd& ub, MatrixXd& lb)
{
	Vector3d rmin,rmax,ones(1,1,1);
	rmin = base[0] - ones*(l(0)+platform[0].norm());
	rmax = base[0] + ones*(l(0)+platform[0].norm());
	for (int i=1; i<pRobot->getNow(); i++)
	{
		Vector3d r0 = base[i] - ones*(l(i)+platform[i].norm());
		if ( r0.x() > rmin.x() ) rmin.x() = r0.x();
		if ( r0.y() > rmin.y() ) rmin.y() = r0.y();
		if ( r0.z() > rmin.z() ) rmin.z() = r0.z();
		r0 = base[i] + ones*(l(i)+platform[i].norm());
		if ( r0.x() < rmax.x() ) rmax.x() = r0.x();
		if ( r0.y() < rmax.y() ) rmax.y() = r0.y();
		if ( r0.z() < rmax.z() ) rmax.z() = r0.z();
	}

	// use the center of the bound box as initial estimate
	p(0) = (rmax.x()+rmin.x())/2.0;
	p(1) = (rmax.y()+rmin.y())/2.0;
	p(2) = (rmax.z()+rmin.z())/2.0; // <<<--- vorsicht
	// estimate the orientation with the identity matrix
	p(3) = p(4) = p(5) = 0;

	// Find the bounding box around the centre point
	lb(0) = rmin.x();			ub(0) = rmax.x();
	lb(1) = rmin.y();			ub(1) = rmax.y();
	lb(2) = rmin.z();			ub(2) = rmax.z();
	// use a reasonable bound for the orientation of the platform
	lb(3) = -90 * DEG_TO_RAD; ub(3) = 90 * DEG_TO_RAD;
	lb(4) = -90 * DEG_TO_RAD; ub(4) = 90 * DEG_TO_RAD;
	lb(5) = -90 * DEG_TO_RAD; ub(5) = 90 * DEG_TO_RAD;
}


void CKinematics::poseestimateInherit(const MatrixXd& l, MatrixXd& p, Vector3d& r, Matrix3d& R)
{
	//if poseestimateInherit is used unintentionaly expect perfect instant convergence from levenberg marquadrth!
	p(0)=r.x();
	p(1)=r.y();
	p(2)=r.z();
	getXYZFromMatrix(p(3),p(4),p(5),R);
}

/*! 
 *	Pose Estimation based on GPS Algorithms, a simple intersection method
 *	This method takes one cable and subtracts the distance in each pair 
 *  and essentially solves the equations with one less measurement
 *	Rotation is ignored and the algorithm works with any number of cables 
 *  greater than n.
 *
 *	More details can be found in the respecitve IROS-2015 paper by Schmidt and Pott.
 */
void CKinematics::poseestimate_intersectionmethod(const MatrixXd& l, MatrixXd& p)//, Vector3d& r)
{
	int now = pRobot->now;
	Vector3d *s = new Vector3d[now];
	Vector3d pose;
	
	//normalize rotation
	for (int i=0; i<pRobot->getNow(); i++)
	{
		s[i]=base[i]-platform[i];
	}
	
	MatrixXd A;
	MatrixXd b;
	MatrixXd D;
	MatrixXd DA;   //container for Matrix Product
	A.resize(now,3);
	b.resize(now,1);
	D.resize(now-1,now);
	for(int i=0; i<pRobot->getNow(); i++)
	{
		A(i,0)=2*s[i](0);
		A(i,1)=2*s[i](1);
		A(i,2)=2*s[i](2);
		b(i,0)=s[i].norm()*s[i].norm()-l(i)*l(i);
	}

	for(int i=0; i<pRobot->getNow()-1; i++)
	{
        for(int j=0; j<pRobot->getNow(); j++)
        {
			if (j==0)
				D(i,j)=-1;
            else if(i+1==j)
                D(i,j)=1;
            else
                D(i,j)=0;
		}
	}
	
	DA = D * A;
	pose = DA.colPivHouseholderQr().solve(D*b);

	for(int i=0;i<3;i++)
		p(i) = pose(i);
	p(3) = p(4) = p(5) = 0.0;

	delete[] s;
}

/*! \brief Calculate the pose (r,R) of the platform from give length l of the wires.
 *  The algorithm for forward kinematics works with an arbitrary number of wires m>n. 
 *  The solution (r,R) is computed in two steps. Firstly, possible values of the vector
 *  r are bounded. Therefore, for each wire an enclosing spheres (with radius=2*|l_i|*|b_i|) 
 *  is computed that must contain the vector r. Then these spheres are bounded by 
 *  axis aligned bounding boxes and intersection of all boxes yields a guaranteed 
 *  bounding box for the vector r. 
 *  In the second step, a Levenberg-Marquardt least square solver is applied to iterate
 *  the over-constrained system of equations
 *  \f$ \Phi_i = |r + R*b_i - a_i|^2 - l_i^2 \f$
 *  Furthermore, an analytic Jacobian can be used to speed up iterations. 
 *  \param l [in] the m-dimensional vector of the wire length, where m is the number of the wires
 *  \param r [out] The given position r=(x,y,z) of the platform
 *  \param R [out] The given orientation R of the platform
 *  \return The function returns true, if successful. 
 *  
 *  \todo add a control parameter to bound the max number of iterations (currently its constant 100)
 *  \todo implement a memory management to speed up computations
 *  \todo The use of the levmar algorithm may improved by tuning the parameters 
 *    to fit better the forward kinematics problem (currently only default settings are used)
 */
bool CKinematics::doForwardKinematics(const MatrixXd& l, Vector3d& r, Matrix3d& R, int itmax, emForwardAlgorithm::Type m_nAlgType, emPoseEstimate::Type m_nPoseEstimator)
{
	if (l.size() != pRobot->getNow())
		return false;
	// set cached geometry data to latest settings
	setPointer();
	// Position and dimensions of bounding box
	MatrixXd p(6,1); p = MatrixXd::Zero(6,1);	// this data member must be vector to fulfill the interface needs of the optimization functions
	MatrixXd ub(6,1), lb(6,1);
	
	double del = 1e-4;	//value of delta i.e incremental value for forward difference method

	//// use the initial estimate with a Levenberg-Marquardt solver to iterate the solution 
	wireLength = l;
	
	CKinematics::poseestimateMain(l,p,ub,lb,r,R,m_nPoseEstimator);
	
	double *x=0;		/* I: measurement vector. NULL implies a zero vector */
	int m=6;			/* I: parameter vector dimension (i.e. #unknowns) */
	int n=8;			/* I: measurement vector dimension */
	int ret;            /* return true if max number of iterations was not violated */
//	int itmax=m_nItmax;		/* I: maximum number of iterations */   // <<<--- vorsicht
	double *opts=0;
	//double opts[5] = {0,0,0,0,0};
						/* I: opts[0-4] = minim. options [\mu, \epsilon1, \epsilon2, \epsilon3, \delta]. Respectively the
						 * scale factor for initial \mu, stopping thresholds for ||J^T e||_inf, ||Dp||_2 and ||e||_2 and
						 * the step used in difference approximation to the Jacobian. Set to NULL for defaults to be used.
						 * If \delta<0, the Jacobian is approximated with central differences which are more accurate
						 * (but slower!) compared to the forward differences employed by default. 
						 */
	//double info[LM_INFO_SZ];	     /* O: information regarding the minimization. Set to NULL if don't care; info is now a member variable
						 /** info[0]= ||e||_2 at initial p.
						 * info[1-4]=[ ||e||_2, ||J^T e||_inf,  ||Dp||_2, mu/max[J^T J]_ii ], all computed at estimated p.
						 * info[5]= # iterations,
						 * info[6]=reason for terminating: 1 - stopped by small gradient J^T e
						 *                                 2 - stopped by small Dp
						 *                                 3 - stopped by itmax
						 *                                 4 - singular matrix. Restart from current p with increased mu 
						 *                                 5 - no further error reduction is possible. Restart with increased mu
						 *                                 6 - stopped by small ||e||_2
						 *                                 7 - stopped by invalid (i.e. NaN or Inf) "func" values. This is a user error
						 * info[7]= # function evaluations
						 * info[8]= # Jacobian evaluations
                         * info[9]= # linear systems solved, i.e. # attempts for reducing error
						 */
	double *work=0;		/* working memory at least LM_DIF_WORKSZ() reals large, allocated if NULL */
	double *covar=0;	/* O: Covariance matrix corresponding to LS solution; mxm. Set to NULL if not needed. */
	void *adata=this;	/* pointer to possibly additional data, passed uninterpreted to func.
					 	 * Set to NULL if not needed */
	
	// Use the algorithm specified by m_nAlgType
	switch(m_nAlgType) {
		// use the unconstrained LM-optimizer without analytic Jacobian
		case emForwardAlgorithm::levmar_dif:
			ret=wc_dlevmar_dif(fnc,p.data(),x,m,n,itmax,opts,info,work,covar,adata);
			break;
		// use the unconstrained LM-optimizer with analytic Jacobian (this seems to be the fastest and is the default)
		case emForwardAlgorithm::levmar_der: 
			ret=wc_dlevmar_der(fnc,jacf,p.data(),x,m,n,itmax,opts,info,work,covar,adata);
			break;
		// use the constrained LM-optimizer without analytic Jacobian
		// the constrained solver seem to work as well but they are much slower (~10 times)
		case emForwardAlgorithm::levmar_difBounded:
			ret=wc_dlevmar_bc_dif(fnc,p.data(),x,m,n,lb.data(),ub.data(),itmax,opts,info,work,covar,adata);
			break;
		// use the constrained LM-optimizer with analytic Jacobian
		case emForwardAlgorithm::levmar_derBounded:
			ret=wc_dlevmar_bc_der(fnc,jacf,p.data(),x,m,n,lb.data(),ub.data(),itmax,opts,info,work,covar,adata);
			break;
		// function call for gauss newton optimzer without pulley
		case emForwardAlgorithm::gauss_newton:
			ret = Gauss_Newton(fnc,p.data(),itmax,info,m,n,del,adata);
			break;
		// do the computation with the eigen levmar implementation coming from CMINPACK
		case emForwardAlgorithm::eigen_levmar_dif: {
			StdModelFunctor functor(*this);
			Eigen::LevenbergMarquardt<StdModelFunctor> lm(functor);
			// configure algorithms: apply the settings which worked well for the levmar implementation
			lm.setEpsilon(1e-17);
			lm.setFtol(1e-17);
			lm.setGtol(1e-17);
			lm.setXtol(1e-17);
			lm.setMaxfev(100);
			VectorXd P = p.col(0);
			int eigen_info = lm.minimize(P);
			p.col(0) = P;
			// store some statistics results
			info[5] = lm.iterations();	// we map the eigen3-lm parameters to the info array as used in levmar
			info[6] = eigen_info;
			info[7] = lm.nfev();
			info[8] = lm.njev();
			} break; 
		default:
			ret=wc_dlevmar_der(fnc,jacf,p.data(),x,m,n,itmax,opts,info,work,covar,adata);
			break;
	}

	// copy the parameter vector to the variables
	r = Vector3d(p(0), p(1), p(2));
	//! \todo the following cacluation is a model, that should not be defined here but together with the call-backfunction fnc from kinematics
	R = Matrix3d::ZRotationMatrix3d(p(3))*Matrix3d::YRotationMatrix3d(p(4))*Matrix3d::XRotationMatrix3d(p(5));
	
	// return true if max number of iterations was not violated
	return true; //ret<itmax;
}

/*! \brief calculate the pose (r,R) of the platform from give length l of the wires
 *  The algorithm for forward kinematics works with an arbitrary number of wires m>n. 
 *  The solution (r,R) is computed in two steps. Firstly, possible values of the vector
 *  r are bounded. Therefore, for each wire an enclosing spheres (with radius=2*|l_i|*|b_i|) 
 *  is computed that must contain the vector r. Then these spheres are bounded by 
 *  axis aligned bounding boxes and intersection of all boxes yields a guaranteed 
 *  bounding box for the vector r. 
 *  Further this fucntion transforms the geometry to be more numerically stable.
 *  In the second step, a Levenberg-Marquardt least square solver is applied to iterator
 *  the overconstrained system of equations
 *  \f$ \Phi_i = |r + R*b_i - a_i|^2 - l_i^2 \f$
 *  Furthermore, an analytic Jacobian can be used to speed up iterations. 
 *  \param l [in] the m-dimensional vector of the wire length, where m is the number of the wires
 *  \param r [out] The given position r=(x,y,z) of the platform
 *  \param R [out] The given orientation R of the platform
 *  \return The function returns true, if successful. 
 *  
 */
bool CKinematics::doForwardKinematics_geometrytransform(const MatrixXd& l, Vector3d& r, Matrix3d& R, int itmax, emForwardAlgorithm::Type m_nAlgType, emPoseEstimate::Type m_nPoseEstimator)
{
	if (l.size() != pRobot->getNow())
		return false;
	// set cached geometry data to latest settings
	setPointer();
	// Position and dimensions of bounding box
	MatrixXd p(6,1); p = MatrixXd::Zero(6,1);
	MatrixXd ub(6,1), lb(6,1);
	double del = 1e-4;	//value of delta i.e incremental value for forward difference method

	Matrix3d R_t = Matrix3d::ZRotationMatrix3d(45*DEG_TO_RAD)
			*Matrix3d::YRotationMatrix3d(45*DEG_TO_RAD)
			*Matrix3d::XRotationMatrix3d(45*DEG_TO_RAD);
	
	pRobot->rotateFrameGeoemetry(R_t);
	pRobot->rotatePlatformGeoemetry(R_t);
	
	//// use the initial estimate with a Levenberg-Marquardt solver to iterate the solution 
	wireLength = l;

	CKinematics::poseestimateMain(l,p,ub,lb,r,R,m_nPoseEstimator);
	
	double *x=0;		/* I: measurement vector. NULL implies a zero vector */
	int m=6;			/* I: parameter vector dimension (i.e. #unknowns) */
	int n=8;			/* I: measurement vector dimension */
	int ret;            /* return true if max number of iterations was not violated */
//	int itmax=m_nItmax;		/* I: maximum number of iterations */   // <<<--- vorsicht
	double *opts=0;
	double *work=0;		/* working memory at least LM_DIF_WORKSZ() reals large, allocated if NULL */
	double *covar=0;	/* O: Covariance matrix corresponding to LS solution; mxm. Set to NULL if not needed. */
	void *adata=this;	/* pointer to possibly additional data, passed uninterpreted to func.
					 	 * Set to NULL if not needed */
	
	// Use the algorithm specified by m_nAlgType
	switch(m_nAlgType) {
		// use the unconstrained LM-optimizer without analytic Jacobian
		case emForwardAlgorithm::levmar_dif:
			ret=wc_dlevmar_dif(fnc,p.data(),x,m,n,itmax,opts,info,work,covar,adata);
			break;
		// use the unconstrained LM-optimizer with analytic Jacobian (this seems to be the fastest and is the default)
		case emForwardAlgorithm::levmar_der:
			ret=wc_dlevmar_der(fnc,jacf,p.data(),x,m,n,itmax,opts,info,work,covar,adata);
			break;
		// use the constrained LM-optimizer without analytic Jacobian
		// the constrained solver seem to work as well but they are much slower (~10 times)
		case emForwardAlgorithm::levmar_difBounded:
			ret=wc_dlevmar_bc_dif(fnc,p.data(),x,m,n,lb.data(),ub.data(),itmax,opts,info,work,covar,adata);
			break;
		// use the constrained LM-optimizer with analytic Jacobian
		case emForwardAlgorithm::levmar_derBounded:
			ret=wc_dlevmar_bc_der(fnc,jacf,p.data(),x,m,n,lb.data(),ub.data(),itmax,opts,info,work,covar,adata);
			break;
		//function call for gauss newton optimzer without pulley
		case emForwardAlgorithm::gauss_newton:
			ret = Gauss_Newton(fnc,p.data(),itmax,info,m,n,del,adata);
			break;
			// do the computation with the eigen levmar implementation coming from CMINPACK
		case emForwardAlgorithm::eigen_levmar_dif: {
			StdModelFunctor functor(*this);
			Eigen::LevenbergMarquardt<StdModelFunctor> lm(functor);
			// configure algorithms: apply the settings which worked well for the levmar implementation
			lm.setEpsilon(1e-17);
			lm.setFtol(1e-17);
			lm.setGtol(1e-17);
			lm.setXtol(1e-17);
			lm.setMaxfev(100);
			VectorXd P = p.col(0);
			int eigen_info = lm.minimize(P);
			p.col(0) = P;
			// store some statistics results
			info[5] = lm.iterations();	// we map the eigen3-lm parameters to the info array as used in levmar
			info[6] = eigen_info;
			info[7] = lm.nfev();
			info[8] = lm.njev();
		} break;
		default:
			ret=wc_dlevmar_der(fnc,jacf,p.data(),x,m,n,itmax,opts,info,work,covar,adata);
			break;
	}

	pRobot->rotateFrameGeoemetry(R_t.inverse());
	pRobot->rotatePlatformGeoemetry(R_t.inverse());
	
	// copy the parameter vector to the variables
	r = Vector3d(p(0), p(1), p(2));
	//! \todo the following cacluation is a model, that should not be defined here
	R = (Matrix3d::ZRotationMatrix3d(p(3))*Matrix3d::YRotationMatrix3d(p(4))*Matrix3d::XRotationMatrix3d(p(5)));//*R_t.inverse();

	// return true if max number of iterations was not violated
	return true; //ret<itmax;
}


/*! \brief calculate the pose (r,R) of the platform from give length l of the wires using preconditioned angles
 *  The algorithm for forward kinematics works with an arbitrary number of wires m>n. 
 *  The solution (r,R) is computed in two steps. Firstly, possible values of the vector
 *  r are bounded. Therefore, for each wire an enclosing spheres (with radius=2*|l_i|*|b_i|) 
 *  is computed that must contain the vector r. Then these spheres are bounded by 
 *  axis aligned bounding boxes and intersection of all boxes yields a guaranteed 
 *  bounding box for the vector r. 
 *  In the second step, a Levenberg-Marquardt least square solver is applied to iterator
 *  the overconstrained system of equations
 *  \f$ \Phi_i = |r + R*b_i - a_i|^2 - l_i^2 \f$
 *  Furthermore, an analytic Jacobian can be used to speed up iterations. 
 *  \param l [in] the m-dimensional vector of the wire length, where m is the number of the wires
 *  \param r [out] The given position r=(x,y,z) of the platform
 *  \param R [out] The given orientation R of the platform
 *  \return The function returns true, if successful. 
 */
bool CKinematics::doForwardKinematics_preconditionedabc(const MatrixXd& l, Vector3d& r, Matrix3d& R, int itmax, emForwardAlgorithm::Type m_nAlgType, emPoseEstimate::Type m_nPoseEstimator)
{
	if (l.size() != pRobot->getNow())
		return false;
	// set cached geometry data to latest settings
	setPointer();
	// Position and dimensions of bounding box
	MatrixXd p(6,1); p = MatrixXd::Zero(6,1);
	MatrixXd ub(6,1), lb(6,1);
	//// use the initial estimate with a Levenberg-Marquardt solver to iterate the solution 
	wireLength = l;
	double del = 1e-4;	//value of delta i.e incremental value for forward difference method

	CKinematics::poseestimateMain(l,p,ub,lb,r,R,m_nPoseEstimator);

	double *x=0;		/* I: measurement vector. NULL implies a zero vector */
	int m=6;			/* I: parameter vector dimension (i.e. #unknowns) */
	int n=8;			/* I: measurement vector dimension */
	int ret;            /* return true if max number of iterations was not violated */
//	int itmax=m_nItmax;		/* I: maximum number of iterations */   // <<<--- vorsicht
	double *opts=0;
	double *work=0;		/* working memory at least LM_DIF_WORKSZ() reals large, allocated if NULL */
	double *covar=0;	/* O: Covariance matrix corresponding to LS solution; mxm. Set to NULL if not needed. */
	void *adata=this;	/* pointer to possibly additional data, passed uninterpreted to func.
					 	 * Set to NULL if not needed */
	
	// Use the algorithm specified by m_nAlgType
	switch(m_nAlgType) {
		// use the unconstrained LM-optimizer without analytic Jacobian
		case emForwardAlgorithm::levmar_dif:
			ret=wc_dlevmar_dif(fnc_preconabc,p.data(),x,m,n,itmax,opts,info,work,covar,adata);
			break;
		// use the unconstrained LM-optimizer with analytic Jacobian (this seems to be the fastest and is the default)
		case emForwardAlgorithm::levmar_der:
			ret=wc_dlevmar_der(fnc_preconabc,jacf_preconabc,p.data(),x,m,n,itmax,opts,info,work,covar,adata);
			break;
		// use the constrained LM-optimizer without analytic Jacobian
		// the constrained solver seem to work as well but they are much slower (~10 times)
		case emForwardAlgorithm::levmar_difBounded:
			ret=wc_dlevmar_bc_dif(fnc_preconabc,p.data(),x,m,n,lb.data(),ub.data(),itmax,opts,info,work,covar,adata);
			break;
		// use the constrained LM-optimizer with analytic Jacobian
		case emForwardAlgorithm::levmar_derBounded:
			ret=wc_dlevmar_bc_der(fnc_preconabc,jacf_preconabc,p.data(),x,m,n,lb.data(),ub.data(),itmax,opts,info,work,covar,adata);
			break;
		//function call for gauss newton optimzer without analytic Jacobian
		case emForwardAlgorithm::gauss_newton:
			ret = Gauss_Newton(fnc_preconabc,p.data(),itmax,info,m,n,del,adata);
			break;
		// do the computation with the eigen levmar implementation coming from CMINPACK
		case emForwardAlgorithm::eigen_levmar_dif: {
			StdPreConFunctor functor(*this);
			Eigen::LevenbergMarquardt<StdPreConFunctor> lm(functor);
			// configure algorithms: apply the settings which worked well for the levmar implementation
			lm.setEpsilon(1e-17);
			lm.setFtol(1e-17);
			lm.setGtol(1e-17);
			lm.setXtol(1e-17);
			lm.setMaxfev(100);
			VectorXd P = p.col(0);
			int eigen_info = lm.minimize(P);
			p.col(0) = P;
			// store some statistics results
			info[5] = lm.iterations();	// we map the eigen3-lm parameters to the info array as used in levmar
			info[6] = eigen_info;
			info[7] = lm.nfev();
			info[8] = lm.njev();
		} break;
		// same as DLEVMAR_DER
		default:
			ret=wc_dlevmar_der(fnc_preconabc,jacf_preconabc,p.data(),x,m,n,itmax,opts,info,work,covar,adata);
			break;
	}

	double a,b,c;
	a = (p(3) * 2 * MO_PI) / (abs(p(3)) + 1);
	b = (p(4) * 2 * MO_PI) / (abs(p(4)) + 1);
	c = (p(5) * 2 * MO_PI) / (abs(p(5)) + 1);

	// copy the parameter vector to the variables
	r = Vector3d(p(0), p(1), p(2));
	//! \todo the following cacluation is a model, that should not be defined here
	R = Matrix3d::ZRotationMatrix3d(a)*Matrix3d::YRotationMatrix3d(b)*Matrix3d::XRotationMatrix3d(c);

	// return true if max number of iterations was not violated
	return true; //ret<itmax;
}


/*! \brief calculate the pose (r,R) of the platform from give length l of the wires
 *  The algorithm for forward kinematics works with an arbitrary number of wires m>n. 
 *  The solution (r,R) is computed in two steps. Firstly, possible values of the vector
 *  r are bounded. Therefore, for each wire an enclosing spheres (with radius=2*|l_i|*|b_i|) 
 *  is computed that must contain the vector r. Then these spheres are bounded by 
 *  axis aligned bounding boxes and intersection of all boxes yields a guaranteed 
 *  bounding box for the vector r. 
 *  In the second step, a Levenberg-Marquardt least square solver is applied to iterator
 *  the overconstrained system of equations
 *  \todo: changes in doForwardKinematics should also be implemented here
 *  \todo: at the moment this will only work with 8 wires because n=8
 *	\todo: return valuable information
 *  \todo: reconsider the pose estimator: pulleys with larger radius may undermine the strict estimation scheme
 */
bool CKinematics::doForwardKinematicsPulley(const MatrixXd& l, Vector3d& r, Matrix3d& R, int itmax, emForwardAlgorithm::Type m_nAlgType, emPoseEstimate::Type m_nPoseEstimator)
{
	// set cached geometry data to latest settings
	setPointer();	
	MatrixXd p(6,1); p = MatrixXd::Zero(6,1);
	MatrixXd ub(6,1), lb(6,1);

	double del = 1e-5;								//value of delta i.e incremental value for forward difference method

	//// use the initial estimate with a Levenberg-Marquardt solver to iterate the solution 
	wireLength = l;

	CKinematics::poseestimateMain(l,p,ub,lb, r, R, m_nPoseEstimator);

	double *x=0;		/* I: measurement vector. NULL implies a zero vector */
	int m=6;			/* I: parameter vector dimension (i.e. #unknowns) */
	int n=8;			/* I: measurement vector dimension */
	int ret;            /* return true if max number of iterations was not violated */
//	following line not needed anymore, only here because of the warning
// int itmax=m_nItmax;	/* I: maximum number of iterations */   // <<<--- vorsicht 
	double *opts=0;		/* I: opts[0-4] = minim. options [\mu, \epsilon1, \epsilon2, \epsilon3, \delta]. */
	double *work=0;		/* working memory at least LM_DIF_WORKSZ() reals large, allocated if NULL */
	double *covar=0;	/* O: Covariance matrix corresponding to LS solution; mxm. Set to NULL if not needed. */
	void *adata=this;	/* pointer to possibly additional data, passed uninterpreted to func.
					 	 * Set to NULL if not needed */
	
	// Use the algorithm specified by m_nAlgType //!<todo: for pulley kinematics no bounded/jacobian implemented yet
	switch(m_nAlgType) {
		// use the unconstrained LM-optimizer without analytic Jacobian
		case emForwardAlgorithm::levmar_dif:
			ret=wc_dlevmar_dif(fncPu,p.data(),x,m,n,itmax,opts,info,work,covar,adata);
			break;
		// use the unconstrained LM-optimizer with analytic Jacobian (this seems to be the fastest and is the default)
		case emForwardAlgorithm::levmar_der:
			ret=wc_dlevmar_der(fncPu,jacf8Pu,p.data(),x,m,n,itmax,opts,info,work,covar,adata);
			break;
		// use the constrained LM-optimizer without analytic Jacobian
		// the constrained solver seem to work as well but they are much slower (~10 times)
		case emForwardAlgorithm::levmar_difBounded:
			ret=wc_dlevmar_bc_dif(fncPu,p.data(),x,m,n,lb.data(),ub.data(),itmax,opts,info,work,covar,adata);
			break;
		// use the constrained LM-optimizer with analytic Jacobian
		case emForwardAlgorithm::levmar_derBounded:
			ret=wc_dlevmar_bc_der(fncPu,jacf8Pu,p.data(),x,m,n,lb.data(),ub.data(),itmax,opts,info,work,covar,adata);
			break;
		//function call for gauss newton optimzer without pulley
		// same as levmar_der
		case emForwardAlgorithm::gauss_newton:
			ret = Gauss_Newton(fncPu,p.data(),itmax,info,m,n,del,adata);
			break;
		// do the computation with the eigen levmar implementation coming from CMINPACK
		case emForwardAlgorithm::eigen_levmar_dif: {
			PulleyModelFunctor functor(*this);
			Eigen::LevenbergMarquardt<PulleyModelFunctor> lm(functor);
			// configure algorithms: apply the settings which worked well for the levmar implementation
			lm.setEpsilon(1e-17);
			lm.setFtol(1e-17);
			lm.setGtol(1e-17);
			lm.setXtol(1e-17);
			lm.setMaxfev(100);
			VectorXd P = p.col(0);
			int eigen_info = lm.minimize(P);
			p.col(0) = P;
			// store some statistics results
			info[5] = lm.iterations();	// we map the eigen3-lm parameters to the info array as used in levmar
			info[6] = eigen_info;
			info[7] = lm.nfev();
			info[8] = lm.njev();
		} break;
		default:
			ret=wc_dlevmar_der(fncPu,jacf8Pu,p.data(),x,m,n,itmax,opts,info,work,covar,adata);
			break;
	}

	// copy the parameter vector to the variables
	r = Vector3d(p(0), p(1), p(2));
	//! \todo the following cacluation is a model, that should not be defined here
	R = Matrix3d::ZRotationMatrix3d(p(3))*Matrix3d::YRotationMatrix3d(p(4))*Matrix3d::XRotationMatrix3d(p(5));

	return true;
}

bool doElastoGeometricalForwardKinematics(const double* l, Vector3d& r, Matrix3d& R)
{
	return true;
}


// forward kinematics of redundant cable robots based on distance equations
// system: | a_i - B_i |^2 - l_i^2 = 0				for	i = [1 ... now]
//		   | B_i - B_j |^2 - | b_i - b_j|^2 = 0		for i>j, i,j = [1,  ... now]
void CKinematics::fnc_dist(double *p, double *hx, int m, int n, void *adata) 
{ ((CKinematics*)adata)->fnc_dist2(p,hx,m,n); }

void CKinematics::fnc_dist2(double *p, double *hx, int m, int n)    
{
	int now = pRobot->now;
	Vector3d *B = new Vector3d[now];

	// copy the sought variables to the local vector B[i]
	for (int i=0; i<now; i++)
		B[i] = Vector3d(p[3*i],p[3*i+1],p[3*i+2]);

	// compute the first n.o.w. equiations
	for (int i = 0; i < now; i++)
		hx[i] = (base[i] - B[i]).squaredNorm() - wireLength(i)*wireLength(i);

	// compute the remaining distance equations (now * (now-1)/2)
	int k=now;
	// compute the distance equations between Bi and Bj
	for (int i=0; i<now; i++)
		for (int j=i+1; j<now; j++)
			hx[k++] = (B[i]-B[j]).squaredNorm() - (platform[i] - platform[j]).squaredNorm();
	
	// free the memory		
	delete [] B;
}

//! new driver function for forward kinematics with distance equations
bool CKinematics::doForwardKinematicsDist(const MatrixXd& l, Vector3d& r, Matrix3d& R, int itmax, emForwardAlgorithm::Type m_nAlgType, emPoseEstimate::Type m_nPoseEstimator)
{
	int now = pRobot->now;
	// set cached geometry data to latest settings
	setPointer();	
	
	// use the initial estimate with a Levenberg-Marquardt solver to iterate the solution 
	wireLength = l;

	MatrixXd pp(6,1), ub(6,1), lb(6,1);

	CKinematics::poseestimateMain(l,pp,ub,lb, r, R, m_nPoseEstimator);
	MatrixXd p(now+(now*(now-1)/2), 1);

	for (int i=0; i<now; i++)
	{
		// use the center as initial estimate of the points
		p(i*3)   = pp(0)+platform[i].x();
		p(i*3+1) = pp(1)+platform[i].y();
		p(i*3+2) = pp(2)+platform[i].z();
	}

	double *x=0;		/* I: measurement vector. NULL implies a zero vector */
	int m=now*3;			/* I: parameter vector dimension (i.e. #unknowns) */
	int n=now+(now*(now-1)/2);			/* I: measurement vector dimension */
	int ret;            /* return true if max number of iterations was not violated */
	double *opts=0;		/* I: opts[0-4] = minim. options [\mu, \epsilon1, \epsilon2, \epsilon3, \delta]. */
	double *work=0;		/* working memory at least LM_DIF_WORKSZ() reals large, allocated if NULL */
	double *covar=0;	/* O: Covariance matrix corresponding to LS solution; mxm. Set to NULL if not needed. */
	void *adata=this;	/* pointer to possibly additional data, passed uninterpreted to func.
					 	 * Set to NULL if not needed */
	
	// Use the algorithm specified by m_nAlgType
	switch(m_nAlgType) {
		// use the unconstrained LM-optimizer without analytic Jacobian
		case emForwardAlgorithm::levmar_dif:// use the unconstrained LM-optimizer without analytic Jacobian
			ret=wc_dlevmar_dif(fnc_dist,p.data(),x,m,n,itmax,opts,info,work,covar,adata);
			break;
		// same as levmar_dif
		default:
			ret=wc_dlevmar_dif(fnc_dist,p.data(),x,m,n,itmax,opts,info,work,covar,adata);
			break;
	}

	// print the results
 	Vector3d *B = new Vector3d[now];

	// copy the sought variables to the local vector B[i]
	for (int i=0; i<now; i++)
		Vector3d B(p(3*i),p(3*i+1),p(3*i+2));

	// printf("Iterations %i\n", ret);
	Vector3d r1_orig,r2_orig,r3_orig,r1,r2,r3;
	//use first three points to construct coordinate system (hope they are linearly independent) TODO: check for linear independence
	r1_orig = (platform[1] - platform[0]).normalized();
	r2_orig = ((platform[1] - platform[0]).cross(platform[2] - platform[0])).normalized();
	r3_orig = r1_orig.cross(r2_orig);

	//construct the same coordinate system again using levmar p
	Vector3d _co1(p(3)-p(0),p(4)-p(1),p(5)-p(2));
	Vector3d _co2(p(6)-p(0),p(7)-p(1),p(8)-p(2));
	r1 = (_co1).normalized();
	r2 = ((_co1).cross(_co2)).normalized();
	r3 = r1.cross(r2);

	Matrix3d _mo1,_mo2;
	_mo1 << r1_orig , r2_orig , r3_orig;
	_mo2 << r1 , r2 , r3;
	
	R = _mo1 * _mo2.transpose();
	
	//use zero point to get platform origin
	Vector3d _b1(p(0),p(1),p(2));
	r = _b1 - R.transpose() * platform[0];
	
	R.transposeInPlace();
	// todo: Compute r and R from B[i]
	delete [] B;

	// return true if max number of iterations was not violated
	return true; //ret<itmax;
}


/*! \brief calculate the length l of the wires for a given pose (r,R)
 *  \param r [in] The given position r=(x,y,z) of the platform
 *  \param R [in] The given orientation R of the platform
 *  \param l [out] the m-dimensional vector of the wire length, where m is the number of the wires
 *  \return The function returns true, if successful. The function returns false if 
 *          the parameter were invalid or if the solution vector l is not bounded by [lmin;lmax]
 */
bool CKinematics::doInverseKinematics(const Vector3d& r, const Matrix3d& R, MatrixXd& l) const
{
	// set cached geometry data to latest settings
	setPointer();	
	bool res=true;
	for (int i=0; i<pRobot->getNow(); i++)
	{
		l(i) = ( r + R*platform[i] - base[i] ).norm();
		//if (l(i)<pRobot->lmin || l(i)>pRobot->lmax)
		//	res = false;
	}
	return res;
}

/*! \brief Calculate the length l of the wires for a given pose (r,R) where a pulley kinematics is
 *  assumed.
 *  \param r [in] The given position r=(x,y,z) of the platform
 *  \param R [in] The given orientation R of the platform
 *  \param l [out] the m-dimensional vector of the wire length, where m is the number of the wires
 *  \return The function returns true, if successful. The function returns false if 
 *          the parameter were invalid or if the solution vector l is not bounded by [lmin;lmax]
 *  \remark The radius of the pulley is hardcoded into this function; the axis of the first
 *          pulley axis is the z-axis of the world frame; it is assumed that the wire leaves the
 *          winch in the direction of the positive z-axis before it is wraped around the pulley.
 */
bool CKinematics::doInverseKinematicsPulley(const Vector3d& r, const Matrix3d& R, MatrixXd& l) const
{
	Matrix3d A;
	double rp;	//!< radius of the pulley
	if (l.size() != pRobot->getNow())
		return false;
	// set cached geometry data to latest settings
	setPointer();	
	bool res = true;
	for (int i=0; i<pRobot->getNow(); i++)
	{
		rp = pRobot->r_pulley[i]; //!< radius of the pulley
		Vector3d AP = r+R*platform[i] - base[i];		//Find the vector from base to platform point of cable connect (A to P)
		Vector3d AM = (R_base[i].col(2).cross(AP)).cross(R_base[i].col(2));	//Find Vector from base to centre of winch where e1 is assumed to be vector of rotation matrix along the winch axis
		Vector3d AMnorm = Vector3d(AM.x()/AM.norm(),AM.y()/AM.norm(),AM.z()/AM.norm());
		Vector3d M = base[i] + rp*AMnorm;			// vector defining point M=A + r*unitvector(AM)
		double a = AP.norm();						// length from A to P
		double b = (r+R*platform[i]-M).norm();		// length from M to P
		
		double theta = acos((b * b + rp * rp - a * a) / (2 * b * rp)) - acos(rp / b);	// finding 'hug' angle Umschlingunswinkel
		double thetax = 2 * (4.0*atan(1.0)) - theta - 2 * acos(rp / b);					// the other possible 'hug'angle ((4*atan(1)=pi)

		if ((R_base[i].col(2).dot(AP)) > 0)
		{
			l(i)=theta * rp + sqrt(b * b - rp * rp);
		}
		else 
		{
			l(i)=thetax * rp + sqrt(b * b - rp * rp);
		}

		// check for min and max of l
		if (l(i)<pRobot->lmin || l(i)>pRobot->lmax)
			res = false;
	}
	return res;
}


/*! New implementation of inverse kinematics taking into account pulleys. The 
 *  formula uses a arccos for the computation which seems to valid for any 
 *  pose of the robot without a distinction of cases. The extended 
 *  implementation provide the rotation angles beta, gamma of the pulley 
 *  mechanism as well as the actual tangient point C where the cables leaves 
 *  the pulley. Furthermore, one can determine the direction of the cable as 
 *  is needed e.g. for the structure matrix.
 * 
 *  One can pass NULL pointer for the variable (l, beta, gamma, C or U) that 
 *  are not needed to be calculated and returned.
 *
 *  \return true, if successful and if the length of the cables is valid, 
 *          otherwise false
 */
bool CKinematics::doInverseKinematicsPulleyEx(const Vector3d& r, const Matrix3d& R, double* l, double* lf, double* beta, double* gamma, Vector3d* C, Vector3d* U) const
{
	double eps = 1e-6;			//!< threshold distance between the point Bi and the z-axis of the frame K_Ai
	double rp;	//!< radius of the pulley 

	// set cached geometry data to latest settings
	setPointer();	
	bool res=true;

	for (int i=0; i<pRobot->getNow(); i++)
	{
		rp=pRobot->r_pulley[i];	//!< radius of the pulley 
		Vector3d AP = r+R*platform[i] - base[i];		//Find the vector from base to platform point of cable connect (A to P)
		// compute vektor b_i in the local frame Ai of the winch
		Vector3d bi = pRobot->R_base[i].transpose()*(AP);
		// determine the polar cylinder coordinates
		double bxy=sqrt(bi.x()*bi.x()+bi.y()*bi.y());
		double bz =bi.z();
		double BM = sqrt((bxy-rp)*(bxy-rp)+bz*bz);
		
		// check if bi is too close to the winch, the distance between the bi and M < 2*rp
		if (BM<2*rp || bxy < eps)
		{
			res = false;
			continue;
		}
		// calculate the first rotation of the pulley (around local z-axis)
		double Gamma = atan2(bi.y(),bi.x());

		// calculate the free length of the cable lf
		double lf_ = sqrt(bz*bz + (bxy-rp)*(bxy-rp) - rp*rp);

		// calculate the wound up angle of the pulley (around local y-axis)
		double Beta = acos(lf_/BM) + acos (bz/BM);

		// calculate the cable length l
		double L = Beta * rp + lf_;

		// check for min and max of l
		if (L<pRobot->lmin || L>pRobot->lmax)
			res = false;

		// optional: calculate the location of the point Ci
		Vector3d C_ = base[i]+R_base[i]*(Matrix3d::ZRotationMatrix3d(Gamma)*(Vector3d(rp,0,0)+Matrix3d::YRotationMatrix3d(Beta)*Vector3d(-rp,0,0)));

		// optional: get the direction of the cable (vector u)
		Vector3d U_ = C_ - R*platform[i] - r;

		// copy the result, if requested
		if (lf) lf[i]=lf_;
		if (l) l[i]=L;
		if (gamma) gamma[i]=Gamma;
		if (beta) beta[i]=Beta;
		if (C) C[i]=C_;
		if (U) U[i]=U_;
	}
	return res;
}


void CKinematics::printZeroLength()
{
	MatrixXd l(pRobot->getNow(),1);

	Vector3d r = Vector3d::Zero();
	Matrix3d R = Matrix3d::Identity();
	doInverseKinematics(r,R,l);
	cout << "Zero Length: ";
	for (int i=0; i<pRobot->getNow(); i++)
		cout << l(i) << "\n ";
	cout << endl;
}

/*! evaluate the inverse kinematics and compare the results with the minimum 
 *  and maximum cable length
 *  \param r [in] position vector (x,y,z) of the platform to be tested
 *  \param R [in] orientation matrix of the platform to be tested
 *  \return true if the calculated cable length are feasible; the also returns false, if the robot data are inconsistent 
 *  \todo improve the memory management for the cable length. performce may be 
 *  poor due to unneccassary new/delete calls
 */
bool CKinematics::testWorkspaceWireLength(const Vector3d& r, const Matrix3d& R)
{
	// check feasibility of parameter
	if (!pRobot) 
		return false;
	if (pRobot->lmin > pRobot->lmax)
		return false;
	// evaluate inverse kinematics
	MatrixXd l(pRobot->now,1);
	return doInverseKinematics(r,R,l);
}

//! call the current evaluation test; currently there is only one such test
//! in CKinematics but we may want to include other tests (e.g. testing 
//! if the rotation capabilities of the pulley mechanisms are feasible)
bool CKinematics::testPose(const Vector3d& r, const Matrix3d& R)
{
	return testWorkspaceWireLength(r,R);
}

/*! simplified interface for getWireRangeForBox() with output of the results on the screen.
 *  \param Min [in] the lower corner of the workspace box to be checked
 *  \param Max [in] the higher corner of the workspace box to be checked
 *  \return true if test was successful. If dMax was not NULL, it contains the vector with the maximum
 *          cable legnth.
 */
bool CKinematics::getWireRangeForBoxDriver( const Vector3d& Min, const Vector3d& Max, double* dMax) const
{
	const int now = pRobot->getNow();
	MatrixXd Lmin(now,1), Lmax(now,1);

	Matrix3d R = Matrix3d::ZRotationMatrix3d(0);
	bool erg = getWireRangeForBox(Lmin,Lmax,Min,Max,R);
	cout << "Minimum and maxmimum cable length for a workspace box\n";
	for (int i=0; i<now; i++)
		cout << i << ": " << Lmin(i) << " -- " << Lmax(i) << " = " << Lmax(i)-Lmin(i) << endl;
	// commpute the maximum length of a single Wire
	if (dMax) // check if pointer is defined
		for (int i=0; i<now; i++)
			*dMax = max(*dMax,(Lmax(i))); //returns the maximum length for one winch. Possible variation could be the sum of all maximum values for total cable length

	return erg;
}

/*! \brief Calculate the actually used range of the wires within a workspace box
 *  \param Lmin [out] an array of size number of wires with the minimal wire length for each winch
 *  \param Lmax [out] an array of size number of wires with the maximum wire length for each winch
 *  \param Min [in] the lower bounds on the (x,y,z) coordiates of box
 *  \param Max [in] the upper bound on the (x,y,z) coordiates of box
 *  \param R [in] the orientation of the platform used for the calculation
 *  \return true, if successful, otherwise false. In the latter case the results in Lmin and Lmax are undefined
 *  \todo The algorithms has limitations: At least for some geometries it is possible that the
 *  minimal length is shorter than reported from testing the eight corners of the box!
 */
bool CKinematics::getWireRangeForBox(MatrixXd& Lmin, MatrixXd& Lmax, const Vector3d& Min, const Vector3d& Max, const Matrix3d& R) const
{
	// check parameter for consistency
	if (Lmin.size()!=pRobot->getNow() || Lmax.size()!=pRobot->getNow())
		return false;
	bool erg=false;
	const int now = pRobot->getNow();
	// extract the eight corners in array
	Vector3d corners[8];
	corners[0]=Vector3d(Min.x(),Min.y(),Min.z());
	corners[1]=Vector3d(Max.x(),Min.y(),Min.z());
	corners[2]=Vector3d(Min.x(),Max.y(),Min.z());
	corners[3]=Vector3d(Max.x(),Max.y(),Min.z());
	corners[4]=Vector3d(Min.x(),Min.y(),Max.z());
	corners[5]=Vector3d(Max.x(),Min.y(),Max.z());
	corners[6]=Vector3d(Min.x(),Max.y(),Max.z());
	corners[7]=Vector3d(Max.x(),Max.y(),Max.z());
	// create a temporary vector of the cable length
	MatrixXd l(now,1);
	// the box to be checked is in general convex; therefore, the maximum 
	// cable length is reached at the corners of the box given by (Min,Max)
	// The mininum is also checked only at the corners, where this is 
	// only an estimate; e.g. if a winches is inside the box, the minimum should be 0 
	// where the algorithm will report a larger value!
	if (!doInverseKinematics(corners[0],R,Lmin))
		goto return_label;
	// copy the inially determined values to Lmax
	for (int i=0; i<now; i++) 
		Lmax(i)=Lmin(i);
	// now we go through the seven remaining corners
	for (int j=1; j<8; j++)
	{
		if (!doInverseKinematics(corners[j],R,l))
			goto return_label;
		for (int i=0; i<now; i++) 
		{
			if (Lmax(i)<l(i))
				Lmax(i)=l(i);
			if (Lmin(i)>l(i))
				Lmin(i)=l(i);
		}
	}

	erg=true;
return_label:
	return erg;
}

/*! Calculate the inverse velocity transmission of the robot based on the 
 *  simplified kinematic model. The algorithms is based on the equations
 *  given in R. Verhoeven (2004): "Analysis of the Workspace of Tendon-based 
 *  Stewart Platforms", p.99. This function basically evaluates the the 
 *  product dl=A(r,R)*[v, omega], where A is the pose dependent structure 
 *  matrix (not transposed!), [v,omega] is the twist of the platform and (r,R)
 *  is the pose of the platform.
 *  
 *  \param r [in] is the position of the platform
 *  \param R [in] is the orientation of the platform as rotation matrix
 *  \param v [in] is the linear velocity of the platform
 *  \param omega [in] is the angular velocity of the platform
 *  \param dl [out] is the pointer to an array with now elements that contains the desired cable velocities
 *  \return true, if successful; if the function return false, the value of dl is undefined.
 *
 *  \remark The current implementation is rather preliminary, since it makes 
 *  no use of the position inverse kinematics. Furthermore, the kinematic code
 *  was not optimize w.r.t performance. 
 */
bool CKinematics::doInverseKinematicsVelocity(const Vector3d&r, const Matrix3d& R, const Vector3d& v, const Vector3d& omega, double* dl)
{
	if (!dl)
		return false;
	// set cached geometry data to latest settings
	setPointer();	
	
	// compute the velocities of the platform component-wise
	for (int i=0; i<pRobot->now; i++)
	{
		// compute u = b[i] - r - R*p[i]
		Vector3d u = pRobot->base[i];
		u-= r;
		u-= R*pRobot->platform[i];
		u.normalize();
		dl[i] = -u.dot(v+omega.cross(platform[i]));
	}
	return true;
}

/*! compute the velocity transmission of the robot's inverse kinematics
 *  i.e. for given pose (r,R)  and the respective velocities (v, omega) 
 *  or the platform the velocities  in the cables are determined.
 */
bool CKinematics::doInverseKinematicsVelocity(const Vector3d& r, const Matrix3d& R, const Vector3d& v, const Vector3d& omega, MatrixXd& dl)
{
	bool returnValue;
	MatrixXd dl_(pRobot->now,1);
	returnValue=doInverseKinematicsVelocity(r, R, v, omega, dl_);
	for (int i=0;i<pRobot->now;i++)
		dl(i,0)=dl_(i);

	return returnValue;
}

//! same as the other inverse velocity kinematics but other data types to 
//! store the the current pose
bool CKinematics::doInverseKinematicsVelocity(const CPoseKinetostatic& poseKinetostatic, MatrixXd& dl)
{
	MatrixXd dl_(pRobot->now,1);

	if (!doInverseKinematicsVelocity(poseKinetostatic.r, poseKinetostatic.R, poseKinetostatic.v, poseKinetostatic.omega, dl_))
		return false;
	
	for (int i=0;i<pRobot->now;i++)
		dl(i,0)=dl_(i);

	return true;
}

/*! Calculate the inverse acceleration transmission of the robot based on the 
 *  simplified kinematic model. The algorithms is based on the equations
 *  given in R. Verhoeven (2004): "Analysis of the Workspace of Tendon-based 
 *  Stewart Platforms", p.99. This function basically evaluates the the 
 *  product ddl=-A*d[v, omega]/dt - dA/dt [v, omega] , where A is the pose dependent structure 
 *  matrix (not transposed!), dA/dt is the time derivated matrix A
 *  [v,omega] is the twist of the platform, [a, alpha] are the translational and 
 *  angular acceleration and is the pose of the platform (r,R)
 *  
 *  \param r [in] is the position of the platform
 *  \param R [in] is the orientation of the platform as rotation matrix
 *  \param v [in] is the linear velocity of the platform
 *  \param omega [in] is the angular velocity of the platform
 *  \param a [in] is the linear acceleration of the platform
 *  \param alpha [in] is the angular acceleration of the platform
 *  \param ddl [out] is the pointer to an array with now elements that contains the desired cable accelerations
 *  \return true, if successful; if the function return false, the value of dl is undefined.
 *
 *  \remark The current implementation is rather preliminary, since it makes 
 *  no use of the position inverse kinematics. Furthermore, the kinematic code
 *  was not optimize w.r.t performance
 */
bool CKinematics::doInverseKinematicsAcceleration(const Vector3d&r, const Matrix3d& R, const Vector3d& v, const Vector3d& omega, const Vector3d& a, const Vector3d& alpha, MatrixXd& ddl)
{	
	// set cached geometry data to latest settings
	setPointer();	
	MatrixXd A(pRobot->getNow(),6); // Jacobian = AT transposed

	// calculate the structure matrix
	for (int i=0; i<pRobot->getNow(); i++)
	{
		// get platform pivot in world coordinates
		Vector3d P=R*pRobot->getPlatform(i);
		// calculate wire vector 
		Vector3d u = pRobot->getBase(i)-r-P;
		// normalize the vector to get the wire direction (and thus the direction of the force)
		u.normalize();
		// fill the first column of the structure matrix
		Vector3d B; 
		B = P.cross(u);
		
		A(i,0)=u.x();
		A(i,1)=u.y();
		A(i,2)=u.z();
		A(i,3)=B.x();
		A(i,4)=B.y();
		A(i,5)=B.z();
	}

	MatrixXd A_der(8,6); // time derivated Jacobian Matrix = d(AT)/dt transposed
	MatrixXd t(6,1); // twist = [v,omega]
	MatrixXd t_point(6,1); // time derivated twist = [a, alpha]

	for (int i=0; i<3;i++) // concatenate the vectors for translation and rotation to one six-element vector
	{
		t(i)=v(i);
		t(i+3)=omega(i);
		t_point(i)=a(i);
		t_point(i+3)=alpha(i);
	}

	// Declarations for the analytic calculation of the time derivated structure matrix
	Vector3d P;		// platform pivot in world coordinates
	Vector3d li;	// cable vector
	double li_norm;	// cable length
	Vector3d li_point;	// time derivation of the cable vector
	double li_point_norm;	// cable velocity
	Vector3d ui;			// cable direction (unit vector)
	Vector3d ui_point;	// time derivation of the cable direction
	Vector3d vi_point;		// time derivation of the cross product bi x ui

	// skew matrix for the angular velocity
	Matrix3d crossProductMatrix=Matrix3d::Zero(3,3);
	crossProductMatrix(0,1)=-omega.z();
	crossProductMatrix(0,2)=omega.y();
	crossProductMatrix(1,0)=omega.z();
	crossProductMatrix(1,2)=-omega.x();
	crossProductMatrix(2,0)=-omega.y();
	crossProductMatrix(2,1)=omega.x();

	// calculate the time derivated structure matrix
	for (int i=0; i<pRobot->getNow(); i++)
	{
		// get platform pivot in world coordinates
		P=R*pRobot->getPlatform(i);
		
		// calculate wire vector 
		li = pRobot->getBase(i)-r-P;
		
		li_norm=li.norm();

		ui=li/li_norm; // normalize the vector to get the wire direction (and thus the direction of the force)

		li_point=-v-crossProductMatrix*R*pRobot->getPlatform(i);

		li_point_norm=(li.dot(li_point))/li_norm;

		ui_point = (li_point*li_norm-li*li_point_norm)/(li_norm*li_norm);
	
		for (int j=0;j<3;j++)
			A_der(i,j)=ui_point(j);
		
		vi_point = (crossProductMatrix*R*pRobot->getPlatform(i)).cross(ui) + (R*pRobot->getPlatform(i)).cross(ui_point);
		for (int j=0;j<3;j++)
			A_der(i,j+3)=vi_point(j);
	}

	ddl=-A*t_point - A_der*t;

	return true;
}

bool CKinematics::doInverseKinematicsAcceleration(const CPoseKinetostatic& poseKinetostatic, MatrixXd& ddl)
{
	return doInverseKinematicsAcceleration(poseKinetostatic.r, poseKinetostatic.R, poseKinetostatic.v, poseKinetostatic.omega, poseKinetostatic.a, poseKinetostatic.alpha, ddl);
}

/*! This function evaluates the potential energy of the robot for use in kinematic and dynamic
 *  algorihtms. The main sources of potential energy are the potential energy of the platform 
 *  defined by gravity (w.r.t. to the world frame) and the potential energy that is store in the
 *  elastic deformation of the cables. The latter clearly depends on the assumed cable model.
 *  In contrast to other kinematic approaches, the potential energy computation is independent
 *  from the kinematic classification of the robot as the energy can be computed disregaring if the
 *  robot is planar or spatial and also if it is IPRM, CPRM, or RRPM.
 *  Note that the potential energy may be negativ if the platform's pose lies in negative 
 *  gravitorial with respect to the workd frame K0.
 *
 *  \todo Add means of controlling the parameters cable model and bConsiderPlatform from outside
 *  
 *  \param r [in] the cartesian position of the platform
 *  \param R [in] the orientation of the platform w.r.t. to the world frame
 *  \param l [in] the (unstrained) length of the cables; all components of the vector must be strictly positive 
 *  \return the potential energy of the robot, NaN if computation was not possible (e.g. because 
 *		    of parameter mismatch)
 */
double CKinematics::getPotentialEnergy(const Vector3d& r, const Matrix3d& R, const MatrixXd& l)
{
	// flags what and how to concider the components
	const Vector3d grav = Vector3d(0, 0, -9.81);
	const bool bConsiderPlatform=true;
	emPotentialEnergyModel::Type cableModel = emPotentialEnergyModel::LinearUnilateral;
	const double eps_dl = 1e-6;	// the minimum positive cable length that is required to compute a spring constant for the cable
	
	// test for correctness of parameters: size matching?
	if (l.size()!=pRobot->getNow())
		return std::numeric_limits<double>::quiet_NaN();
	// test all cable length to be strictly positive
	for (int i=0; i<pRobot->getNow(); i++)
		if (l(i) < eps_dl)
			return std::numeric_limits<double>::quiet_NaN();

	// init the potential energy
	double U = 0;

	// compute the potential energy of the platform
	if (bConsiderPlatform)
		U += pRobot->platform_mass * r.dot(-grav);

	// loop through all cables i and compute the energy of the cables
	for (int i=0; i<pRobot->getNow(); i++)
	{
		// determine the geometry distance between Ai and Bi
		Vector3d li = pRobot->getBase(i) - R*pRobot->getPlatform(i) - r;
		// get the scalar distance
		double d = li.norm();
		double dU = 0, c=0;
		// spring constant of the cable which is determined from c = A*E/l
		c = pRobot->pCable->r_cable * pRobot->pCable->r_cable * MO_PI * pRobot->pCable->E_wire / l(i); // Area * E/l;

		// apply the cable model and determine the energy
		switch (cableModel) 
		{
		case emPotentialEnergyModel::LinearBilateral:
			// compute the energy for a strut like model (storing energy symmetrically both in compression and extensions)
			// U = 1/2 * c * dl^2    where dl is the elongation and c is the spring constant 
			dU = 0.5 * c * (d - l(i))*(d - l(i));
			break;

		case emPotentialEnergyModel::LinearUnilateral:
			// compute the energy for a ideal tendon model (storing only energy in tension but no energy in compression)
			// U = 1/2 * c * dl^2 if dl>0 otherwise 0    where dl is the elongation and c is the spring constant 
			if ( d > l(i) )		
				// geometric distance is larger than cable length --> cable is strechted
				dU = 0.5 * c * (d - l(i)) * (d - l(i));
			else
				dU = 0;
			break;
		case emPotentialEnergyModel::NoCable:
		default:
			// nothing is added to the energy if the cable model was not covered by the implementation above
			break;
		}
		// sum up the energy
		U+=dU;
	}
	return U;
}


//****************************************************************************
// Methods of the elasto-kinematics class CElastoKinematics
//****************************************************************************

CElastoKinematics::CElastoKinematics(CRobotData& robot): CKinematics(robot)
{
	// Copy all stiffness coefficients to the local variables
	k_spec = MatrixXd(1,pRobot->getNow());
	for (int i=0; i<pRobot->getNow(); i++)
	{
		k_spec(i) = pRobot->pCable[i].k_spec;
	}
}

CElastoKinematics::~CElastoKinematics()
{
}


/*//! compute the platform pose under the consideration of cable forces
*  \param l_nominal: vector with the cable length
*  \param r: vector with initial guess of position for LM + returns the position
*  \param vEuler: vector with initial guess of orientation for LM + returns the orientation
*  \param f: returns the cable forces
*  \param itmax: set the max. number of iterations of LM
*/
bool CElastoKinematics::doElastoGeometricalForwardKinematics(const MatrixXd& l_nominal, Vector3d& r, Vector3d& vEuler, MatrixXd& f, int itmax)
{
	// set cached geometry data to latest settings
	setPointer();
	this->l_nominal = l_nominal; // copy nominal cable length to local member in order to make it accessable in the callback function used in levmar
	double param[6] = {r(0),r(1),r(2),vEuler(0),vEuler(1),vEuler(2)}; // use input pose as initial guess
	double* pParam; // create parameter vector (initial guess for the platform pose)
	pParam = param;

	double del = 1e-4;
	double *x=0;		/* I: measurement vector. NULL implies a zero vector */
	int m=6;			/* I: parameter vector dimension (i.e. #unknowns) */
	int n=6;			/* I: measurement vector dimension */
	//int itmax=100;		/* I: maximum number of iterations */   // <<<--- vorsicht
	double *opts=0;		/* I: opts[0-4] = minim. options [\mu, \epsilon1, \epsilon2, \epsilon3, \delta]. Respectively the
						 * scale factor for initial \mu, stopping thresholds for ||J^T e||_inf, ||Dp||_2 and ||e||_2 and
						 * the step used in difference approximation to the Jacobian. Set to NULL for defaults to be used.
						 * If \delta<0, the Jacobian is approximated with central differences which are more accurate
						 * (but slower!) compared to the forward differences employed by default. 
						 */
	double *work=0;		/* working memory at least LM_DIF_WORKSZ() reals large, allocated if NULL */
	double *covar=0;	/* O: Covariance matrix corresponding to LS solution; mxm. Set to NULL if not needed. */
	void *adata=this;	/* pointer to possibly additional data, passed uninterpreted to func.
					 	 * Set to NULL if not needed */
	
	// use the unconstrained LM-optimizer without analytic Jacobian
	int ret=wc_dlevmar_dif(fncElastoKin_static,pParam,x,m,n,itmax,opts,info,work,covar,adata);

	// copy the parameter vector which was returend by levmar to the variables
	r = Vector3d(pParam[0],pParam[1],pParam[2]);
	vEuler = Vector3d(pParam[3],pParam[4],pParam[5]);
	//! \todo the following cacluation is a model, that should not be defined here
	f = this->f_resultant; // return cable forces from local member variable

	//cout << "--------------r:" << pParam[2]<< endl;
	//cout << "R:" << R << endl;
	cout<<"Translation\n"<<r<<endl;
	cout<<"Rotation\n"<<vEuler<<endl;

	// return true if max number of iterations was not violated
	return ret<itmax;
}

/*! adapter callback fnc for levenberg marquart optimizer; forward the call
 *  to the member function
 *  \param h: measurement vector, residual function (6x1 platform wrench)
 *  \param p: input parameters which are adjusted by the levmar optimizer (6x1 platform pose)
 *  \param m: input parameter vector dimension
 *  \param n: measurement vector dimension
 *  \param adata: additional data for the algorithm
 */
void CElastoKinematics::fncElastoKin_static(double *p, double *hx, int m, int n, void *adata)
{
	((CElastoKinematics*)adata)->fncElastoKin(p,hx,m,n);
}

//! the non-static version of the callback function
//! (residual function, non-squared)
void CElastoKinematics::fncElastoKin(double *p, double *hx, int m, int n)
{
	// parse platform pose
	Vector3d r(p[0],p[1],p[2]);
	Matrix3d R = Matrix3d::ZRotationMatrix3d(p[3])*Matrix3d::YRotationMatrix3d(p[4])*Matrix3d::XRotationMatrix3d(p[5]);
	MatrixXd f(1, this->pRobot->getNow());
	Vector3d* u = new Vector3d[this->pRobot->getNow()];
	double g = -9.81;				// gravitational constant
	computeCableForces(this->l_nominal,r,R,f,u);	// compute cable forces using the nominal cable length stored in the member variable l_nominal
	MatrixXd w_res(6,1);			// 6x1 vector for the resultant platform wrench
	MatrixXd w(6,1);				// 6x1 applied wrench (right now, only the gravitational force is considered)
	w << 0, 0, pRobot->platform_mass * g, 0, 0 ,0; // [0,0,fg,0,0,0]'
	
	computeResultantPlatformWrench(w_res,f, w, r, R, u); // compute w_res
	// return measurement vector (error), here the resultant platform wrench
	memcpy(hx,w_res.data(),sizeof(double)*6);
	
	// store cable forces in member variable in order to make it public
	this->f_resultant = f;

	delete [] u;

	/*cout << "r:" << r.transpose() << endl;
	cout << "R:" << R << endl;
	cout << "f:" << this->f_resultant << endl;
	cout << "w_res:" << hx[2] << endl << endl;*/
}


//! compute cable forces and cable unit vectors u_i
bool CElastoKinematics::computeCableForces(const MatrixXd& l_nominal, const Vector3d& r, const Matrix3d& R, MatrixXd& f, Vector3d* u)
{
	Vector3d* pB = new Vector3d[pRobot->getNow()];
	Vector3d* pC = new Vector3d[pRobot->getNow()];
	MatrixXd l(1,pRobot->getNow());
	Vector3d* pCons;
	
	for (int i=0; i<pRobot->getNow(); i++)
	{
		pB[i] = r + R * pRobot->getPlatform(i);
	}

	// deal with different kinematic model
	if (pRobot->getRobotKinematicsModel() == CRobotData::FIXED)
	{
		for (int i=0; i<pRobot->getNow(); i++)
		{
			pC[i] = pRobot->getBase(i);
			u[i]  = pC[i] - pB[i];
			u[i].normalize();
			int nCons = pRobot->pCable[i].nContraints + 2;
			pCons = new Vector3d[nCons];   
			pCons[0] = pB[i];
			pCons[1] = pC[i];
			l(i) = 0; // compute effective cable length
			l(i) += (pCons[0]-pCons[1]).norm();
			for(int j=1; j<nCons-1; j++)
			{
				pCons[j+1] = pRobot->pCable[i].pConstraints[j-1];
				l(i) += (pCons[j]- pCons[j+1]).norm(); 
			}

			delete [] pCons;
		}
	}
	else if (pRobot->getRobotKinematicsModel() == CRobotData::PULLEY)
	{
		doInverseKinematicsPulleyEx(r, R, l.data(), 0, 0, 0, pC, u); // compute effective cable length
	}

	// deal with different elasticity models
	if (pRobot->getElasticityModel() == CRobotData::LIN_ELASTIC)
	{
		MatrixXd delta_l = l - l_nominal;
		for (int i=0; i<pRobot->getNow(); i++)
		{
			f(i) = k_spec(i) / l_nominal(i) * delta_l(i); //convert to array for elementwise multiplication
			if (f(i) < 0)
			{
				f(i) = f(i) * 0.000001;
			}
		}
	}
	else if (pRobot->getElasticityModel() == CRobotData::SAGGING)
	{
		Vector3d u2;
		double f2; // not used

		MatrixXd delta_l = l - l_nominal; // used for computing the initial guess for the forces

		for (int i=0; i<pRobot->getNow(); i++)
		{
			//CCable cable(&pB[i], &pC[i], &(l_nominal.data()[i]),pRobot->pCable[i].weight,pRobot->pCable[i].k_spec); // k_spec = EA0
			//cable.set_Parameters(&pB[i], &pC[i], &(l_nominal.data()[i]),simMode); // is already called in the constructor
			//cable.compute_AnchorageForces(u[i],u2,f(i),f2);

			int simMode = 2;
			int nCons = pRobot->pCable[i].nContraints + 2;  // number of constraints (2 constraints a_i, b_i + additional constraints)
			pCons = new Vector3d[nCons];    

			// compute initial guess for the forces

			double f_ini = k_spec(i) * delta_l(i); //convert to array for elementwise multiplication
			if (f_ini < 0)
			{
				f_ini = 1;
			}

			CCableChain cableChain(&nCons,pRobot->pCable[i].weight, pRobot->pCable[i].k_spec, f_ini);
						
			pCons[0] = pB[i];
			pCons[1] = pC[i];
						
			// set the remaining points to the additional constraints given by pCable.constraints
			for(int j=0; j<nCons-2; j++)
			{
				pCons[j+2] = pRobot->pCable[i].pConstraints[j];//pC[i]*1.2;
			}

			cableChain.set_Parameters(pCons,&(l_nominal.data()[i]),simMode);
			cableChain.compute_AnchorageForces(0,u[i],u2,f(i),f2); // show anchorage forces of catenary 0 of cable i

			delete [] pCons;
		}
	}

	delete []  pB, pC;
	return true;
}


//! compute resultant wrench w_res form the cable forces f and applied wrench w
bool CElastoKinematics::computeResultantPlatformWrench(MatrixXd& w_res, const MatrixXd& f, const MatrixXd& w, const Vector3d& r, const Matrix3d& R, const Vector3d* u)
{
	// set cached geometry data to latest settings
	setPointer();	
	MatrixXd A(pRobot->getNow(),6); // Jacobian = AT transposed

	// calculate the structure matrix
	for (int i=0; i<pRobot->getNow(); i++)
	{
		// get platform pivot in world coordinates
		Vector3d P=R*pRobot->getPlatform(i);
		// fill the first column of the structure matrix
		Vector3d B; 
		B = P.cross(u[i]);

		A(i,0)=u[i].x();
		A(i,1)=u[i].y();
		A(i,2)=u[i].z();
		A(i,3)=B.x();
		A(i,4)=B.y();
		A(i,5)=B.z();
	}

	w_res = A.transpose()*f.transpose()+w;

	return true;
}


//////////////////////////////////////////////////////////////////////////////
// Class CCable
//////////////////////////////////////////////////////////////////////////////

CCable::CCable(const Vector3d* P1, const Vector3d* P2, const double* l_nominal, const double& mpu, const double& EA0)
{
	this->EA0 = EA0; 
	this->mpu = mpu;
	int simMode = 2;
	this->set_Parameters(P1, P2, l_nominal,simMode);
}

CCable::CCable(const double& mpu, const double& EA0)
{
	this->EA0 = EA0; 
	this->mpu = mpu;
	int simMode = 2;

	// for proper use one has to set the parametes using the set_Parameters method before computing the catenary
	this->P1 = new Vector3d(0,0,0);
	this->P2 = new Vector3d(0,0,0);
	this->Fx_P2 = 0;
	this->Fx_P2 = 0;
	this->pL0   = 0;
}

CCable::~CCable()
{
}

//! set parameters an compute catenary
bool CCable::set_Parameters(const Vector3d* P1, const Vector3d* P2, const double* l_nominal, const int& simulationMode)
{
	this->P1 = P1; //new Vector3d(0,0,0);
	this->P2 = P2; //new Vector3d(20,0,-10);
	this->pL0 = l_nominal; // new double(23);//l_nominal;

	this->h = sqrt(pow(this->P2->x()-this->P1->x(),2)+ pow(this->P2->y()-this->P1->y(),2));
	this->v= this->P2->z()-this->P1->z();
	

	double *x=0;		/* I: measurement vector. NULL implies a zero vector */
	int itmax=100;		/* I: maximum number of iterations */   // <<<--- vorsicht
	double *opts=0;		/* I: opts[0-4] = minim. options [\mu, \epsilon1, \epsilon2, \epsilon3, \delta]. Respectively the
						 * scale factor for initial \mu, stopping thresholds for ||J^T e||_inf, ||Dp||_2 and ||e||_2 and
						 * the step used in difference approximation to the Jacobian. Set to NULL for defaults to be used.
						 * If \delta<0, the Jacobian is approximated with central differences which are more accurate
						 * (but slower!) compared to the forward differences employed by default. 
						 */
	double info[LM_INFO_SZ];
	double *work=0;		/* working memory at least LM_DIF_WORKSZ() reals large, allocated if NULL */
	double *covar=0;	/* O: Covariance matrix corresponding to LS solution; mxm. Set to NULL if not needed. */
	void *adata=this;	/* pointer to possibly additional data, passed uninterpreted to func.
					 	 * Set to NULL if not needed */
	
	double* pparam = 0;
	int m,n;
	int ret;

	if (simulationMode == 1) // stiff sagging cable
	{
		double param[2] = {100,100} ; // todo: compute initial guess using elasticity model without sagging
		pparam = param; // set pointer to array
		m=2;			/* I: parameter vector dimension (i.e. #unknowns) */
		n=2;			/* I: measurement vector dimension */
		// use the unconstrained LM-optimizer without analytic Jacobian
		int ret=wc_dlevmar_dif(fncCateneryParam_static,pparam,x,m,n,itmax,opts,info,work,covar,adata);
	}
	else if (simulationMode == 2) // elastic sagging cable
	{
		double param[2] = {100,100} ; // todo: compute initial guess using elasticity model without sagging
		pparam = param;	// set pointer to array
		m=2;			/* I: parameter vector dimension (i.e. #unknowns) */
		n=2;			/* I: measurement vector dimension */
		// use the unconstrained LM-optimizer without analytic Jacobian
		ret=wc_dlevmar_dif(fncElasticCateneryParam_static,pparam,x,m,n,itmax,opts,info,work,covar,adata);
	}
	// copy parameters to member variables
	this->Fx_P2 = pparam[0];
	this->Fz_P2 = pparam[1];

	return ret < itmax;
}


// Methods for stiff catenary
//////////////////////////////////////////////////////////////////////////////


/*! Static adapter function which allows to compute the parametrization of the catenary function
 *  (it is not allowed to provide a pointer to a nonstatic member function)
 *  \param h: measurement vector, residual function (1x1 objective function)
 *  \param p: input parameters which are adjusted by the levmar optimizer (1x1 catenary parametization)
 *  \param m: input parameter vector dimension 1x1
 *  \param n: measurement vector dimension 1x1
 *  \param adata: additional data for the algorithm 
 */
void CCable::fncCateneryParam_static(double *p, double *hx, int m, int n, void *adata)
{
	((CCable*)adata)->fncCateneryParam(p,hx,m,n);
}

//! the non-static version of the callback function
//! (residual function, non-squared)
void CCable::fncCateneryParam(double *p, double *hx, int m, int n)
{
	// find optimal a so that f(p) = 0
	// objective function
	*hx = (*p/this->h) * sqrt(pow(*this->pL0,2)-pow(this->v,2))-sinh(*p);
}

bool CCable::get_Catenary(const int& nPoints, Vector3d* samplePoints)
{
	Vector3d vP1P2 = *this->P2 - *this->P1;
	vP1P2.z() = 0;		// project on xy-plane
	vP1P2.normalize();  // create unit vector on xy-plane
	double s,x,z;

	double steps = nPoints - 1;
	double increment = *pL0/steps; // increment curve parameter s
	for(int i=0; i<nPoints; i++)
	{
		s = increment * i;
		this->compute_elasticCatenary(s,x,z,*this->pL0,this->Fx_P2, this->Fz_P2);

		samplePoints[i] = vP1P2 * x;		// step in xy-plane
		samplePoints[i].z() =  z;			// associated z-value
		samplePoints[i] = samplePoints[i] + *this->P1; // displcement according to offset of P1
	}
	return true;
}

double CCable::atanh(double x)
{
	return (log(1+x) - log(1-x))/2;
}

double CCable::asinh(double x)
{
	return log(x + sqrt(x*x+ 1));
}

bool CCable::compute_CableForce(const double& s, Vector3d& u, double& f)
{
	Vector3d vP1P2 = *this->P2 - *this->P1;
	vP1P2.z() = 0;		// project on xy-plane
	vP1P2.normalize();  // create unit vector on xy-plane
	
	double phi = compute_phi(s,*this->pL0,this->Fx_P2, this->Fz_P2);
	//phi = atan(this->Fz_P2/this->Fx_P2);
	f = computeTension(s,*this->pL0,this->Fx_P2, this->Fz_P2);
	u = vP1P2 *cos(phi); // scale unit vector by projected length
	u.z() = sin(phi);
	return true;
}

bool CCable::compute_AnchorageForces(Vector3d& u1, Vector3d& u2, double& f1, double& f2)
{
	compute_CableForce(0,u1,f1);
	compute_CableForce(*pL0,u2,f2);
	return true;
}


// Methods for elastic catenary
//////////////////////////////////////////////////////////////////////////////

/*! static adapter function which allows to compute the parametrization of the catenary function
 *  (it is not allowed to provide a pointer to a nonstatic member function)
 *  \param h: measurement vector, residual function (2x1 objective function)
 *  \param p: input parameters which are adjusted by the levmar optimizer (1x1 catenary parametization)
 *  \param m: input parameter vector dimension 2x1
 *  \param n: measurement vector dimension 2x1
 *  \param adata: additional data for the algorithm
 */
void CCable::fncElasticCateneryParam_static(double *p, double *hx, int m, int n, void *adata)
{
	((CCable*)adata)->fncElasticCateneryParam(p,hx,m,n);
}

//! the non-static version of the callback function
//! (residual function, non-squared)
void CCable::fncElasticCateneryParam(double *p, double *hx, int m, int n)
{
	// find optimal a so that f(p) = 0
	// objective function
	// compute FxB,FzB using s=l_nominal at P2
	// change FxB,FzB by the optimizer

	double s = *pL0;
	double x,z;
	this->compute_elasticCatenary(s,x,z,*pL0,p[0],p[1]);
	// Assuming that P1 = [0.0]
	hx[0] = h-x; 
	hx[1] = v-z;
}


bool CCable::compute_elasticCatenary(const double& s, double& x, double& z,const double& L0, const  double& FxP2,const double& FzP2)
{
	double Dl = s - L0;

	x = FxP2 *s/EA0
		+ FxP2/mpu*(asinh((FzP2+mpu*(Dl))/FxP2)
		-asinh((FzP2-mpu* L0)/FxP2));

	z = FzP2*s/EA0
		+ (mpu*(0.5*s*s-L0*s))/EA0
		+ (sqrt(pow(FxP2,2)+pow(FzP2+mpu*Dl,2))-sqrt(pow(FxP2,2)+pow(FzP2-mpu* L0,2))) /mpu;

	return true;
}

//! compute derivatives of catenary function
double CCable::compute_dxds(const double& s, const double& L0, const double& FxP2,const double& FzP2)
{
	double T = computeTension(s, L0, FxP2, FzP2);
	double dxds = FxP2/EA0 + FxP2/T;

	return dxds;
}

//! compute derivatives of catenary function
double CCable::compute_dzds(const double& s, const double& L0, const double& FxP2,const double& FzP2)
{
	double Dl = s - L0;
	double T = computeTension(s, L0, FxP2, FzP2);

	double dzds = FzP2/EA0
				+ mpu*Dl/EA0
				+ (FzP2+mpu*Dl) / T;
	return dzds;
}

double CCable::compute_phi(const double& s, const double& L0, const double& FxP2,const double& FzP2)
{
	//double s2 = *pL0;
	double dzds = compute_dzds(s, L0, FxP2, FzP2);
	double dxds = compute_dxds(s, L0, FxP2, FzP2);

	//double rate_x = dzds/dxds;
	//double rate_f = Fz_P2/Fx_P2;

	return atan(dzds/dxds);
}

double CCable::computeTension(const double& s, const double& L0, const double& FxP2,const double& FzP2)
{
	return sqrt(pow(FxP2,2)+pow(FzP2+mpu*(s-L0),2));
}

CCableChain::CCableChain(const int* nSupportPoints, const double& mpu, const double& EA0, const double& f_ini) : CCable(mpu,EA0)
{
	// intialize arrays
	this->nSP = nSupportPoints;
	this->ph = new double[*nSP-1];
	this->pv = new double[*nSP-1];
	this->pFx = new double[*nSP-1];
	this->pFz = new double[*nSP-1];
	this->pL0_single = new double[*nSP-1];
	this->f_ini = f_ini;
}

CCableChain::~CCableChain()
{
	delete [] ph;
	delete [] pv;
	delete [] pFx;
	delete [] pFz;
	delete [] pL0;
}

bool CCableChain::set_Parameters(const Vector3d* supportPointList,  const double* l_nominal, const int& simulationMode)
{
	this->pSP = supportPointList;
	this->L0_tot = l_nominal;
	// compute horizontal and vertical distance of all points

	for(int i=0; i < *nSP-1; i++)
	{
		this->ph[i] = sqrt(pow(pSP[i+1].x()-pSP[i].x(),2)+ pow(pSP[i+1].y()-pSP[i].y(),2));
		this->pv[i]= pSP[i+1].z()-pSP[i].z();
	}

	double *x=0;		/* I: measurement vector. NULL implies a zero vector */
	int itmax=500;		/* I: maximum number of iterations */   // <<<--- vorsicht
	double *opts=0;		/* I: opts[0-4] = minim. options [\mu, \epsilon1, \epsilon2, \epsilon3, \delta]. */
	double info[LM_INFO_SZ];
	double *work=0;		/* working memory at least LM_DIF_WORKSZ() reals large, allocated if NULL */
	double *covar=0;	/* O: Covariance matrix corresponding to LS solution; mxm. Set to NULL if not needed. */
	void *adata=this;	/* pointer to possibly additional data, passed uninterpreted to func.
					 	 * Set to NULL if not needed */
	
	double* pparam = new double[(*nSP-1)*3];
	for(int i=0; i < (*nSP-1); i++)
	{
		// compute initial guess from direction vector and precomputed linear elastic cable forces
		double lenght_i =  sqrt(pow(ph[i],2) + pow(pv[i],2));
		double Fx_ini = ph[i]/lenght_i * f_ini; // f_ini refers to the tension of the linear elastic cable without mass
		double Fz_ini = pv[i]/lenght_i * f_ini;

		pparam[i*3] = Fx_ini ; // use initial guess for Fx_i
		pparam[i*3+1] = Fz_ini; // initial guess for Fz_i
		pparam[i*3+2] = lenght_i; // initial guess for the nominal cable lengths
	}
	int m,n;
	int ret;

	if (simulationMode == 1) // stiff sagging cable
	{
		double param[2] = {100,100} ; // todo: compute initial guess using elasticity model without sagging
		pparam = param; // set pointer to array
		m=2;			/* I: parameter vector dimension (i.e. #unknowns) */
		n=2;			/* I: measurement vector dimension */
		// use the unconstrained LM-optimizer without analytic Jacobian
		int ret=wc_dlevmar_dif(fncElasticCateneryChainParam_static,pparam,x,m,n,itmax,opts,info,work,covar,adata);
	}
	else if (simulationMode == 2) // elastic sagging cable
	{
		m=(*nSP-1)*3;			/* I: parameter vector dimension (i.e. #unknowns) */
		n=(*nSP-1)*3;			/* I: measurement vector dimension */
		// use the unconstrained LM-optimizer without analytic Jacobian
		ret=wc_dlevmar_dif(fncElasticCateneryChainParam_static,pparam,x,m,n,itmax,opts,info,work,covar,adata);
	}

	// copy data to member variables
	for(int i=0; i < *nSP-1; i++)
	{
		this->pFx[i] = pparam[i*3];		   // initial guess for Fx_i
		this->pFz[i] = pparam[i*3+1];		   // initial guess for Fz_i
		this->pL0_single[i] = pparam[i*3+2]; // initial guess for the nominal cable lengths
	}

	delete [] pparam;

	return ret<itmax;
}


// Methods for elastic catenary
//////////////////////////////////////////////////////////////////////////////

/*! static adapter function which allows to compute the parametrization of the catenary function
 *  (it is not allowed to provide a pointer to a nonstatic member function)
 *  \param h: measurement vector, residual function (2x1 objective function)
 *  \param p: input parameters which are adjusted by the levmar optimizer (1x1 catenary parametization)
 *  \param m: input parameter vector dimension 2x1
 *  \param n: measurement vector dimension 2x1
 *  \param adata: additional data for the algorithm 
 */
void CCableChain::fncElasticCateneryChainParam_static(double *p, double *hx, int m, int n, void *adata)
{
	((CCableChain*)adata)->fncElasticCateneryChainParam(p,hx,m,n);
}

//! the non-static version of the callback function
//! (residual function, non-squared)
void CCableChain::fncElasticCateneryChainParam(double *p, double *hx, int m, int n)
{
	// find optimal a so that f(p) = 0
	// objective function
	// compute Fx_i,Fz_i, L0_i
	int j = 0;
	double x,z;
	double L_sum = 0;
	double L0_i;
	double Fx, Fz;
	double* T1 = new double[*nSP-1];
	double* T2 = new double[*nSP-1];

	for(int i=0; i < (*nSP-1); i++)
	{
		Fx = p[3*i];
		Fz = p[3*i+1];
		L0_i = p[3*i+2];

		// compute catenary in a local coordinate system PA = (0,0) -> PB = (h,v)
		this->compute_elasticCatenary(L0_i,x,z,L0_i,Fx,Fz);

		// compute residual vector
		// residual for support points
		hx[j] = ph[i]-x;
		j++;
		hx[j] = pv[i]-z;
		j++;
		// residual for tension equilibrium
		T1[i] = computeTension(0,L0_i,Fx,Fz);	// tension at the beginning of the cable
		T2[i] = computeTension(L0_i,L0_i,Fx,Fz); // tension at the end of the cable
		L_sum+=L0_i;
	}

	for(int i=0; i < *nSP-2; i++)
	{
		hx[j] = T1[i+1]-T2[i]; // error in cable tension equilibrium
		j++;
	}
	// last part of the rsidual vector considers the total cable length
	hx[j] = L_sum - *L0_tot;

	delete [] T1;
	delete [] T2;
}


bool CCableChain::get_Catenary(const int& cableIndex, const int& nPoints, Vector3d* samplePoints)
{
	Vector3d vP1P2 = this->pSP[cableIndex+1]- this->pSP[cableIndex];
	vP1P2.z() = 0;		// project on xy-plane
	vP1P2.normalize();  // create unit vector on xy-plane
	double s,x,z;

	double steps = nPoints - 1;
	double increment = this->pL0_single[cableIndex]/steps; // increment curve parameter s
	for(int i=0; i<nPoints; i++)
	{
		s = increment * i;
		this->compute_elasticCatenary(s,x,z,this->pL0_single[cableIndex],this->pFx[cableIndex], this->pFz[cableIndex]);

		samplePoints[i] = vP1P2 * x;		// step in xy-plane
		samplePoints[i].z() =  z;			// associated z-value
		samplePoints[i] = samplePoints[i] + this->pSP[cableIndex]; // displacement according to offset of support point SP[i]
		//samplePoints[i].x() = i;
		//samplePoints[i].z() = i;
	}

	return true;
}

bool CCableChain::get_Catenary(const int& nPoints, Vector3d* samplePoints)
{
	int pointsPerCable = nPoints / (*nSP-1); 

	for(int i=0; i<(*nSP-1); i++)
	{
		get_Catenary(i, pointsPerCable, &samplePoints[i*pointsPerCable]);
	}

	return true;
}


bool CCableChain::compute_CableForce(const int& cableIndex, const double& s, Vector3d& u, double& f)
{
	Vector3d vP1P2 = this->pSP[cableIndex+1]- this->pSP[cableIndex];
	vP1P2.z() = 0;		// project on xy-plane
	vP1P2.normalize();  // create unit vector on xy-plane
	
	double phi = compute_phi(s,this->pL0_single[cableIndex],this->pFx[cableIndex], this->pFz[cableIndex]);
	f = computeTension(s,this->pL0_single[cableIndex],this->pFx[cableIndex], this->pFz[cableIndex]);
	u = vP1P2 *cos(phi); // scale unit vector by projected length
	u.z() = sin(phi);
	return true;
}

bool CCableChain::compute_AnchorageForces(const int& cableIndex, Vector3d& u1, Vector3d& u2, double& f1, double& f2)
{
	compute_CableForce(cableIndex, 0,u1,f1); // compute cable force for the starting point of the cables (s=0)
	compute_CableForce(cableIndex,this->pL0_single[cableIndex],u2,f2); // compute the cable force for the end point of the cable (s=L0_single)
	return true;
}

/*
bool CCable::compute_CableForce(const double& s, Vector3d& u, double& f)
{
	Vector3d vP1P2 = *this->P2 - *this->P1;
	vP1P2.z() = 0;		// project on xy-plane
	vP1P2.normalize();  // create unit vector on xy-plane
	
	double phi = compute_phi(s,*this->pL0,this->Fx_P2, this->Fz_P2);
	//phi = atan(this->Fz_P2/this->Fx_P2);
	f = computeTension(s,*this->pL0,this->Fx_P2, this->Fz_P2);
	u = vP1P2 *cos(phi); // scale unit vector by projected length
	u.z() = sin(phi);
	return true;
}

bool CCable::compute_AnchorageForces(Vector3d& u1, Vector3d& u2, double& f1, double& f2)
{
	compute_CableForce(0,u1,f1);
	compute_CableForce(*pL0,u2,f2);
	return true;

*/

} // end namespace PCRL