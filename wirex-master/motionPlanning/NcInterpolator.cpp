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

/*!*******************************************************************
 *  \file   : NcInterpolator.cpp
 *
 *  Project   motionPlanning
 *
 *  \Author   Werner Kraus
 *
 *********************************************************************
 */ 

#include "NcInterpolator.h"
#include <list>

#include <iostream>
#include <fstream>


namespace PCRL {

CNcInterpolator::CNcInterpolator()//: CAlgorithm((CRobotData)0)
{
	cycleTime=50.0; 
	amax=30.0;
	jmax=30.0;
	trajectoryTime=0.0;
	trajectoryLength=0.0;
	iNrPoses=0;
	iInterpolationMethod=1; // default is the static version
	gearRatio=12;
	iBezier=1; // 1=activated
	dOverride=100;
	StartPosition=Vector3d(0,0,0);
	StartOrientation=Vector3d(0.0,0.0,0.0);
}


CNcInterpolator::~CNcInterpolator(void)
{
}

bool CNcInterpolator::setCycleTime(const double& cycleTime)
{
	this->cycleTime=cycleTime;
	return true;
}

double CNcInterpolator::getCycleTime()
{
	return cycleTime;
}

bool CNcInterpolator::setAmax(const double& amax)
{
	this->amax=amax;
	return true;
}

bool CNcInterpolator::setJmax(const double& jmax)
{
	this->jmax=jmax;
	return true;
}

bool CNcInterpolator::setInterpolationMethod(const int& iInterpolationMethod)
{
	this->iInterpolationMethod=iInterpolationMethod;
	return true;
}

bool CNcInterpolator::setBezier(const int& iBezier)
{
	this->iBezier=iBezier;
	return true;
}

bool CNcInterpolator::setOverride(const double& dOverride)
{
	this->dOverride=dOverride;
	return true;
}
bool CNcInterpolator::setStartPosition(const Vector3d StartPosition_)
{
	this->StartPosition=StartPosition_;
	return true;
}

bool CNcInterpolator::setStartOrientation(const Vector3d StartOrientation_)
{
	this->StartOrientation=StartOrientation_;
	return true;
}

//! print the current setting to the cout stream
bool CNcInterpolator::printTrajectoryParameters()
{
	cout << "Trajectory duration: " << trajectoryTime << " sec"<< endl;
	cout << "Trajektory length: " << trajectoryLength << " m"<< endl;
	cout << "max. acceleration: " << amax << " m/s^2"<< endl;
	cout << "max. jerk: " << jmax << " m/s^3"<< endl;
	cout << "Cycle time: " << cycleTime << " ms"<< endl;
	cout << "Number of setpoits: " << iNrPoses << endl;
	cout << "Interpolation method: " << iInterpolationMethod << endl;
	return true;
}

/*! translate the G-Code-commands stored in CNcProgramm into a geometric path stored in a Vector
 *  \param Program [in] list of parsed CNC-Commands
 *  \param PathSegments [out] list of different types of path segments like straight lines, bezier curves
 */
bool CNcInterpolator::GeometricPathPlanning(const CNcProgram& Program, std::vector<CPathSegments*>& PathSegments)
{
	// 1. straight lines, 2. bezier5

	list<CNcCommand*>::const_iterator it_Nc=Program.begin(); // Iterator 1 for the CNC Commandlist

	Vector3d rTmp=StartPosition; // start position
	Vector3d anglesTmp=StartOrientation; // orientation in start position

	// translate the CNC-Commands into a list of straight lines with a defined velocity
	for (it_Nc;it_Nc!=Program.end();it_Nc++) 
	{
		CNcLinear* nc_lin=dynamic_cast<CNcLinear *>(*it_Nc); 
		CNcDwellTime* nc_dwell=dynamic_cast<CNcDwellTime *>(*it_Nc); 

		if (nc_lin!=0)
		{
			CStraightLine* StraightLine_tmp=new CStraightLine();
			// begin pose of the straight line
			StraightLine_tmp->r1=rTmp;
			StraightLine_tmp->angles1=anglesTmp;
			// end pose of the straight line
			StraightLine_tmp->r2=nc_lin->targetposition;
			StraightLine_tmp->angles2=nc_lin->targetorientation;
			StraightLine_tmp->vmax=nc_lin->v*dOverride/100.0;
			StraightLine_tmp->amax=this->amax;
			StraightLine_tmp->jmax=this->jmax;
			PathSegments.push_back(StraightLine_tmp);
			rTmp=nc_lin->targetposition;
			anglesTmp=nc_lin->targetorientation;	
		}
		else if (nc_dwell!=0 )
		{
			CDwell* Dwell_tmp=new CDwell();
			Dwell_tmp->r1=rTmp;
			Dwell_tmp->angles=anglesTmp;
			Dwell_tmp->dwellTime=nc_dwell->time;
			PathSegments.push_back(Dwell_tmp);
		}
	}

	// add bezier
	if (iBezier==1)
	{
		vector<CPathSegments*>::iterator it_PS1=PathSegments.begin(); // Iterator 1
		vector<CPathSegments*>::iterator it_PS2=PathSegments.begin(); it_PS2++; // Iterator 2
	
		Vector3d a_coeff[6] , r_b1, r_b2;

		for (it_PS1;it_PS2!=PathSegments.end();it_PS1++,it_PS2++) 
		{
			CStraightLine* SL_tmp1=dynamic_cast<CStraightLine *>(*it_PS1); 
			CStraightLine* SL_tmp2=dynamic_cast<CStraightLine *>(*it_PS2); 	
		
			if (SL_tmp1!=0 && SL_tmp2!=0) // both path segments are straight lines
			{
				if (Bezier5Geometric(SL_tmp1->r1, SL_tmp1->r2, SL_tmp2->r2, a_coeff , r_b1, r_b2))
				{
					// shorten the straight lines
					SL_tmp1->r2=r_b1;
					SL_tmp2->r1=r_b2;

					CBezier5* Bezier5_tmp=new CBezier5();
					Bezier5_tmp->amax=this->amax;
					Bezier5_tmp->jmax=this->jmax;
					Bezier5_tmp->angles1=SL_tmp1->angles2;
					Bezier5_tmp->angles2=SL_tmp2->angles1;

					for (int i=0; i<6;i++)
						Bezier5_tmp->a_coeff[i]=a_coeff[i];
			
					it_PS1=it_PS2=PathSegments.insert(it_PS2,Bezier5_tmp);
					if (it_PS2!=PathSegments.end())
						it_PS2++;
				}
			}
		}
	}

	return true;
}

/*! save the parameters of each path segments into a text file for debug issues
 *  \param PathSegments [in] list of different types of path segments like straight lines, bezier curves
 */
bool CNcInterpolator::PrintPathSegments(std::vector<CPathSegments*>& PathSegments)
{
	vector<CPathSegments*>::iterator it_PS1=PathSegments.begin(); 
	// for debugging write the parameters of the path into a textfile
	ofstream out;
	out.open ("path.txt");
	for (it_PS1=PathSegments.begin();it_PS1!=PathSegments.end();it_PS1++) 
	{	
		CStraightLine* SL_tmp=dynamic_cast<CStraightLine *>(*it_PS1); 
		CBezier5* Bezier5_tmp=dynamic_cast<CBezier5 *>(*it_PS1); 
		CDwell* Dwell_tmp=dynamic_cast<CDwell *>(*it_PS1); 

		if (SL_tmp!=0)
		{
			out << "StraightLine" << " v0: " << SL_tmp->v0 << " v1: " << SL_tmp->v1 << " vmax: " << SL_tmp->vmax << endl;
			out << "r1 x: " << SL_tmp->r1.x() << " y: " << SL_tmp->r1.y() << " z: " << SL_tmp->r1.z()<< endl;
			out << "r2 x: " << SL_tmp->r2.x() << " y: " << SL_tmp->r2.y() << " z: " << SL_tmp->r2.z()<< endl;
			out << "ti: ";
			for (int i=0;i<8;i++)
				out << SL_tmp->t[0][i] << " ";
			
			out << endl<< "si: ";
			for (int i=0;i<8;i++)
				out << SL_tmp->s[0][i] << " ";
			
			out << endl<< "vi: ";
			for (int i=0;i<8;i++)
				out << SL_tmp->v[0][i] << " ";
			
			out << endl<< "ai: ";
			for (int i=0;i<8;i++)
				out << SL_tmp->a[0][i] << " ";
			
			out << endl<< "ji: ";
			for (int i=0;i<8;i++)
				out << SL_tmp->j[0][i] << " ";
			out << endl;
			
		}
		if (Bezier5_tmp!=0)
		{
			out << "Bezier" << " v0: " << Bezier5_tmp->v0 << " v1: " << Bezier5_tmp->v1 <<  endl;
			out << "u_point_max: " << Bezier5_tmp->u_point_max << endl;
			Vector3d cv;
			Bezier5_tmp->getr1(cv);
			out << "b1 " << cv.x() << " " << cv.y() << " " << cv.z()<< endl;
			Bezier5_tmp->getr2(cv);
			out << "b2 " << cv.x() << " " << cv.y() << " " << cv.z()<< endl;
		}
		if (Dwell_tmp!=0)
		{
			out << "dwell" << " time: " << Dwell_tmp->dwellTime << endl;
			out << "r1 x: " << Dwell_tmp->r1.x() << " y: " << Dwell_tmp->r1.y() << " z: " << Dwell_tmp->r1.z()<< endl;
		}
	}
	return true;
}

/*! create a linestrip of a defined geometric path
 *  \param PathSegments [in] list of different types of path segments like straight lines, bezier curves
 *  \param LineStrip [out] vector of points, which build a linestrip
 */
bool CNcInterpolator::PlotPathSegments(std::vector<CPathSegments*>& PathSegments, std::vector<Vector3d>& LineStrip)
{
	vector<CPathSegments*>::iterator it_PS1=PathSegments.begin(); 

	for (it_PS1=PathSegments.begin();it_PS1!=PathSegments.end();it_PS1++) 
	{	
		CStraightLine* SL_tmp=dynamic_cast<CStraightLine *>(*it_PS1); 
		CBezier5* Bezier5_tmp=dynamic_cast<CBezier5 *>(*it_PS1); 
		if (SL_tmp!=0)
		{
			LineStrip.push_back((SL_tmp->r1)/1000);
			LineStrip.push_back((SL_tmp->r2)/1000);
		}
		if (Bezier5_tmp!=0)
		{
			Vector3d cv;
			Bezier5_tmp->getr1(cv);
			LineStrip.push_back(cv/1000);

			for (double u=0.0;u<1.0;u=u+0.1)
			{	
				cv=Bezier5_tmp->a_coeff[0] + Bezier5_tmp->a_coeff[1]*u + Bezier5_tmp->a_coeff[2]*pow(u,2.0) + Bezier5_tmp->a_coeff[3]*pow(u,3.0) + Bezier5_tmp->a_coeff[4]*pow(u,4.0) + Bezier5_tmp->a_coeff[5]*pow(u,5.0); // a_coeff in [mm]
				LineStrip.push_back(cv/1000);
			}
			Bezier5_tmp->getr2(cv);
			LineStrip.push_back(cv/1000);
		}
	}
	return true;
}

/*! Define a Tractory based on PathSegments and dynamic parameters. The results where saved in each path segment
 *  \param PathSegments [in] list of different types of path segments like straight lines, bezier curves
 */
bool CNcInterpolator::TrajectoryPlanning(std::vector<CPathSegments*>& PathSegments)
{
// 1. straight lines, 2. bezier5	
	Vector3d r_a,r_b,r_c;
	Vector3d r_ab, r_bc;
	double l_ab,l_bc,s_ab;
	double angle;
	double eps_angle=0.2;

	vector<CPathSegments*>::iterator it_PS1=PathSegments.begin(); 
	vector<CPathSegments*>::iterator it_PS2=PathSegments.begin();  
	vector<CPathSegments*>::iterator it_PS3=PathSegments.begin(); 

	while (it_PS2!=PathSegments.end())
	{
		it_PS2++;
		if (it_PS2==PathSegments.end())
			break;
		CStraightLine* SL_tmp2=dynamic_cast<CStraightLine *>(*it_PS2); 
		while (SL_tmp2==0 && it_PS2!=PathSegments.end())
		{
			it_PS2++;
			if (it_PS2!=PathSegments.end())
				SL_tmp2=dynamic_cast<CStraightLine *>(*it_PS2);
		}
		
		if (distance(it_PS1,it_PS2)==1) // StraightLine [it_PS1] | StraightLine [it_PS2]
		{
			CStraightLine* SL_tmp1=dynamic_cast<CStraightLine *>(*it_PS1); 
			r_a=SL_tmp1->r1;
			r_b=SL_tmp1->r2;
			r_c=SL_tmp2->r2;

			
			r_ab=r_b-r_a;
			r_bc=r_c-r_b;
			l_ab=r_ab.norm(); // length of path segment 1
			l_bc=r_bc.norm(); // length of path segment 1
			s_ab=r_ab.dot(r_bc); // dotprod

			if (abs(l_ab*l_bc)>0.01) // avoid division by zero
				angle=acos(s_ab/(l_ab*l_bc)); // angle [rad] between the two path segments (0 means, that the next path segments goes in the same direction, therefore no deceleration to standstill is needed, as there is no "corner")
			else
				angle=0;

			if (abs(angle)<0.01 && l_ab > 0.0001) // no deceleration and check, if the length of the first path segment is larger than 0.0001 m
				SL_tmp1->v1=min(SL_tmp1->vmax,SL_tmp2->vmax); // end velocity of path segment 1 is the minimum velocity of segment 1 and 2
			else
				SL_tmp1->v1=0; // set velocity to 0 if there is a corner or path segment 1 has no length

			SL_tmp2->v0=SL_tmp1->v1; // end velocity of segment 1 = start velocity of segment 2
		}
		if (distance(it_PS1,it_PS2)==2) // StraightLine [it_PS1] | Bezier5 [it_PS3] | StraightLine [it_PS2]
		{
			CStraightLine* SL_tmp1=dynamic_cast<CStraightLine *>(*it_PS1); 
			it_PS3=it_PS1;
			it_PS3++;
			CBezier5* Bezier5_tmp=dynamic_cast<CBezier5 *>(*it_PS3); 
			if (Bezier5_tmp!=0)
			{
				Bezier5_tmp->vmax=min(SL_tmp1->vmax,SL_tmp2->vmax);
				Bezier5_tmp->calculateTrajectory(); // set v0 and v1
				SL_tmp1->v1=min(SL_tmp1->vmax,Bezier5_tmp->v0);
				SL_tmp2->v0=min(SL_tmp2->vmax,Bezier5_tmp->v1);
			}
		}
		it_PS1=it_PS2;
	}
	
	// calculate the acceleration ramps for each path segment
	for (it_PS1=PathSegments.begin();it_PS1!=PathSegments.end();it_PS1++) 
	{
		(*it_PS1)->calculateTrajectory();
	}

	return true;
}

/*! create the poselist based on path segments
 *  \param PathSegments [in] list of different types of path segments like straight lines, bezier curves
 *  \param PoseList [out] list of poses in a defined time step
 */
bool CNcInterpolator::InterpolatePathSegments(std::vector<CPathSegments*>& PathSegments, CPoseListKinetostatic& PoseList)
{
	vector<CPathSegments*>::iterator it_PS1=PathSegments.begin(); 
	CPoseKinetostatic* poseKinetostatic;
	double time=0;
	double time_tmp;
	
	Vector3d rtmp, angletmp;
	for (it_PS1=PathSegments.begin();it_PS1!=PathSegments.end();it_PS1++) 
	{
		time_tmp=time;

		while((*it_PS1)->getTrajectoryPoint(time-time_tmp, poseKinetostatic=new CPoseKinetostatic(Vector3d::Zero(),Matrix3d::Zero())))
		{
			PoseList.push_back(poseKinetostatic);
			/*
			angletmp=angletmp*DEG_TO_RAD;
			Rtemp=Matrix3d::ZRotationMatrix3d(angletmp(0))*Matrix3d::YRotationMatrix3d(angletmp(1))*Matrix3d::XRotationMatrix3d(angletmp(2));
			CPoseKinetostatic* pose_tmp= new CPoseKinetostatic(rtmp/1000.0,Rtemp); // linear interpolieren // VORSICHT: Rotation immer Einheitsmatrix
			PoseList.push_back(pose_tmp);
			*/
			time+=cycleTime/1000.0;
		}
	}
	trajectoryTime=PoseList.size()*cycleTime/1000.0;
	return true;
}

/*! Calculate the dynamic states of a trajectory. The result can be used for different issue, e.g. dimensioning
 *  winches or frame
 *  \param PathSegments [in] list of different types of path segments like straight lines, bezier curves
 *  \param sStateDyn [out] list of dynamic states in a defined time step
 */
bool CNcInterpolator::DynamicAnalysisPathSegments(std::vector<CPathSegments*>& PathSegments, CPoseListKinetostatic& sStateDyn)
{
	vector<CPathSegments*>::iterator it_PS1=PathSegments.begin(); 
	
	for (it_PS1=PathSegments.begin();it_PS1!=PathSegments.end();it_PS1++) 
	{
		(*it_PS1)->getStateDynamic(sStateDyn);
	}

	return true;
}

bool CNcInterpolator::ExportStateDyn(CPoseListKinetostatic& sStateDyn)
{
	ofstream out;
	out.open ("StateDyn.txt");

	CPoseListKinetostatic::iterator it_SD; 
	for (it_SD=sStateDyn.begin();it_SD!=sStateDyn.end();it_SD++) 
	{
		out << (*it_SD)->r.x() << " , " << (*it_SD)->r.y() << " , "<< (*it_SD)->r.z() << " , " ;
		//out << (*it_SD)->phi.x() << " , " << (*it_SD)->phi.y() << " , "<< (*it_SD)->phi.z() << " , " ;
		out << (*it_SD)->v.x() << " , " << (*it_SD)->v.y() << " , "<< (*it_SD)->v.z() << " , " ;
		out << (*it_SD)->omega.x() << " , " << (*it_SD)->omega.y() << " , "<< (*it_SD)->omega.z() << " , " ;
		out << (*it_SD)->a.x() << " , " << (*it_SD)->a.y() << " , " << (*it_SD)->a.z() << " , " ;
		out << (*it_SD)->alpha.x() << " , " << (*it_SD)->alpha.y() << " , " << (*it_SD)->alpha.z() << " , " ;
		out << (*it_SD)->v.norm() << " , "<< (*it_SD)->a.norm() << endl ;
	}
	return true;
}

bool CNcInterpolator::ExportPoseList(CPoseListStatic& PoseList)
{
	ofstream out;
	out.open ("PoseLi.txt");
	
	CPoseListStatic::iterator it_SD; 
	for (it_SD=PoseList.begin();it_SD!=PoseList.end();it_SD++) 
	{
		out << (*it_SD)->r.x() << " , " << (*it_SD)->r.y() << " , "<< (*it_SD)->r.z() << " , " << endl;
	//	out << (*it_SD)->phi.x << " , " << (*it_SD)->phi.y << " , "<< (*it_SD)->phi.z << " , "  ;
	}
	return true;
}
/*
bool CNcInterpolator::getStateDynWinches(const MatrixXd& f_wires, const MatrixXd& v_wires, const MatrixXd& a_wires, StateDynWinches& sStateDynWinches)
{
	sStateDynWinches.f_wires=f_wires;
	sStateDynWinches.v_wires=v_wires;
	sStateDynWinches.a_wires=a_wires;

	for (int j=0; j < f_wires.rows(); j++)
	{
		calculateWinchTransmission(f_wires(j), v_wires(j), a_wires(j), sStateDynWinches.M_motor(j), sStateDynWinches.omega_motor(j), sStateDynWinches.alpha_motor(j));
	}
	return true;
}

bool CNcInterpolator::calculateWinchTransmission(const double& f_wire, const double& v_wire, const double& a_wire, double& M_motor, double& omega_motor, double& alpha_motor)
{
	double gear_ratio;	//!< gear ratio (values >1 reduce the velocity of the cable)
	double r_drum;		//!< radius of the drum in the winch [m]
	double n_drum;		//!< maximum number of windings on the drum
	double l_drum;		//!< axial length of the drum [m]
	int spool_direction;
	double inertia_motor;
	double inertia_gearbox;
	double inertia_winch; // respective the drum axis
	double efficiency_gearbox;
	double efficiency_winch;

	double M_wire, M_inertia, M_friction;
	double inertia_total;

	gear_ratio = pRobot->pWinch->gear_ratio;
	r_drum = pRobot->pWinch->r_drum;
	n_drum = pRobot->pWinch->n_drum;
	l_drum = pRobot->pWinch->l_drum;
	inertia_motor = pRobot->pWinch->inertia;
	
	// not included in XML-Spec
	inertia_gearbox = 0.0;
	inertia_winch = 0.0; 
	efficiency_gearbox = 1.0; //0.99;
	efficiency_winch = 1.0; //0.95; 
	spool_direction = 1;
	
	omega_motor = v_wire / (r_drum + spool_direction*l_drum/(n_drum*2.0*MO_PI)) * gear_ratio; // rad/s
	alpha_motor = a_wire / (r_drum + spool_direction*l_drum/(n_drum*2.0*MO_PI)) * gear_ratio; // rad/s²

	inertia_total = inertia_motor + inertia_gearbox + inertia_winch/(pow(gear_ratio,2.0)); // kg m²
	
	M_wire = f_wire * (r_drum + spool_direction*l_drum/(n_drum*2.0*3.14)) / gear_ratio; 
	M_inertia = alpha_motor*inertia_total;
	M_friction = M_wire*(1.0/(efficiency_gearbox*efficiency_winch)-1.0); 
	M_motor = M_wire + M_inertia + M_friction;
	return true;
}


/*
bool CNcInterpolator::CalculateTrajectoryParameters(std::list<PCRL::StateDynWinch*>& listStateDynWinch, double* f_min, double* f_max, double* n_min, double* n_max, double* M_min, double* M_max, double* M_RMS)
{
	for (int i=0;i<pRobot->getNow();i++) // set initial values
	{
		f_min[i]=10000;
		n_min[i]=10000;
		M_min[i]=10000;
		f_max[i]=-10000;
		n_max[i]=-10000;
		M_max[i]=-10000;
		M_RMS[i]=0;
	}	

	std::list<PCRL::StateDynWinch*>::iterator it_SD=listStateDynWinch.begin(); // iterate trough the list of winch states
	for (it_SD;it_SD!=listStateDynWinch.end();it_SD++) 
	{
		for (int i=0;i<pRobot->getNow();i++)
		{
			f_min[i]=min((*it_SD)->f_wires[i],f_min[i]);
			n_min[i]=min((*it_SD)->omega_motor[i],n_min[i]);
			M_min[i]=min((*it_SD)->M_motor[i],M_min[i]);
			f_max[i]=max((*it_SD)->f_wires[i],f_max[i]);
			n_max[i]=max((*it_SD)->omega_motor[i],n_max[i]);
			M_max[i]=max((*it_SD)->M_motor[i],M_max[i]);
			M_RMS[i]=M_RMS[i]+(*it_SD)->M_motor[i]*(*it_SD)->M_motor[i];
		}
	}

	// calculate the RMS-value of the torque
	for (int i=0;i<pRobot->getNow();i++)
		M_RMS[i]=pow(M_RMS[i]/listStateDynWinch.size(),0.5);

	return true;
}

bool CNcInterpolator::EvaluateTrajectory(std::list<PCRL::StateDynWinch*>& listStateDynWinch, bool& bForce, bool& bSpeed, bool& bTorquePeak, bool& bTorqueRMS)
{
	double* f_min = new double[pRobot->getNow()];
	double* n_min = new double[pRobot->getNow()];
	double* f_max = new double[pRobot->getNow()];
	double* n_max = new double[pRobot->getNow()];
	double* M_RMS = new double[pRobot->getNow()];
	
	// cable forces
	double f_min_limit=100;
	double f_max_limit=1800;
	double M_RMS_limit=3.2;

	double n;

	double omega_limit=6000/30*3.14;
	double M_S1, M_S8;

	bForce=true; //
	bSpeed=true; //
	bTorquePeak=true; //
	bTorqueRMS=true; //

	for (int i=0;i<pRobot->getNow();i++) // set initial values
	{
		n_min[i]=10000;
		n_max[i]=-10000;
		f_min[i]=10000;
		f_max[i]=-10000;
		M_RMS[i]=0;
	}
	std::list<PCRL::StateDynWinch*>::iterator it_SD=listStateDynWinch.begin(); // iterate through the list of winch states
	for (it_SD;it_SD!=listStateDynWinch.end();it_SD++) 
	{
		for (int i=0;i<pRobot->getNow();i++)
		{
			f_min[i]=min((*it_SD)->f_wires[i],f_min[i]);
			n_min[i]=min((*it_SD)->omega_motor[i],n_min[i]);
			f_max[i]=max((*it_SD)->f_wires[i],f_max[i]);
			n_max[i]=max((*it_SD)->omega_motor[i],n_max[i]);
			
			n=(*it_SD)->omega_motor[i]*30/3.14; // rpm

			// calculate the torque limits respect to the speed n
			//M_S1=3.4-abs(n)/6000.0*(3.4-2.1);
			if (abs(n)<4600)
				M_S8=9.0;
			else
				M_S8=9.0-(abs(n)-4600)/(6000.0-4600.0)*(3.4-2.1);
			if ((*it_SD)->M_motor[i]>M_S8)
				bTorquePeak=false;
	
			M_RMS[i]=M_RMS[i]+(*it_SD)->M_motor[i]*(*it_SD)->M_motor[i];

		}
	}

	// calculate the RMS-value of the torque
	for (int i=0;i<pRobot->getNow();i++)
	{
		M_RMS[i]=pow(M_RMS[i]/listStateDynWinch.size(),0.5);
		if (M_RMS[i]>M_RMS_limit)
		{
			bTorqueRMS=false;
		}
		if (f_min[i]<f_min_limit)
		{
			bForce=false;
		}
		if (f_max[i]>f_max_limit)
		{
			bForce=false;
		}
		if (abs(n_min[i])>omega_limit)
		{
			bSpeed=false;
		}
		if (abs(n_max[i])>omega_limit)
		{
			bSpeed=false;
		}
	}

	delete [] f_min;
	delete [] n_min;
	delete [] f_max;
	delete [] n_max;
	delete [] M_RMS;
	return true;
}
*/

/*! Calculate the parameters to add a Bezier curve between two straight lines
 *  the algorithm is from "Trajectory Planning for Automatic Machines and Robots"
 *  Luigi Biagiotti, Claudio Melchiorri, 2008, Springer, pages 406-412
 *  \param r_a  [in] beginning point of the first straight line
 *  \param r_b  [in] connection point between first and second straight line
 *  \param r_c  [in] end point of the second straight line
 *  \param a_coeff [out] coefficients of the bezier polynom 
 *  \param r_b1 [out] new end point of the first straight line
 *  \param r_b2 [out] new begin point of the second straight line
 */
bool CNcInterpolator::Bezier5Geometric(Vector3d r_a, Vector3d r_b, Vector3d r_c, Vector3d (&a_coeff)[6] , Vector3d& r_b1, Vector3d& r_b2)
{
	Vector3d r_ab, r_bc;
	double l_ab,l_bc,s_ab;
	double radius, angle, angle_d;
	double eps_angle=0.2;
	double a, b, c; // coefficients
	double pq_p, pq_q, alpha_k;
	Vector3d p[6], tang[6];

	r_ab=r_b-r_a;
	r_bc=r_c-r_b;
	l_ab=r_ab.norm();
	l_bc=r_bc.norm();
	s_ab=r_ab.dot(r_bc); // dotprod
	if (abs(l_ab*l_bc)>0.01)
	{
		angle=acos(s_ab/(l_ab*l_bc)); // angle [rad]
	}
	else
		angle=0;
	angle_d=angle*RAD_TO_DEG;

	if (abs(angle_d)>eps_angle && abs(abs(angle_d)-180.0)>eps_angle)
	{
		radius=(l_ab+l_bc)/2.0*0.1;
		radius=min(radius,min(l_ab/2.0,l_bc/2.0));
		r_b1=r_b-r_ab/r_ab.norm()*radius; // begin of the spline
		r_b2=r_b+r_bc/r_bc.norm()*radius; // end of the spline
		// 2 constraints are determined straight forward
		p[0]=r_b1;
		p[5]=r_b2;
		
		// normalized tangential vectors
		tang[0]=(r_b-r_b1)/radius;
		tang[5]=(r_b2-r_b)/radius;
		
		// special values to determine p[1] and p[3]
		a=256.0 - 49.0*(tang[0]+tang[5]).squaredNorm();
		b=420.0*(p[5]-p[0]).dot((tang[0]+tang[5]));
		c=-900.0*(p[5]-p[0]).squaredNorm();
		
		// solve a*alpha_k^2+b*alpha_k + c = 0 and take the maximum solution 
		pq_p=b/a;
		pq_q=c/a;
		alpha_k=max(-pq_p/2.0+sqrt(pow(pq_p/2.0,2.0)-pq_q),-pq_p/2.0-sqrt(pow(pq_p/2.0,2.0)-pq_q));
	
		// missing 4 constraints can be determined with alpha_k
		p[1]=p[0] + alpha_k/5.0*tang[0];
		p[2]=2*p[1]-p[0];
		p[4]=p[5] - alpha_k/5.0*tang[5];
		p[3]=2*p[4] - p[5];
		
		// determination of the coefficients for the spline between r_b1 and r_b2
		a_coeff[0]=       p[0];
		a_coeff[1]=( -5.0*p[0]  + 5.0*p[1]);
		a_coeff[2]=( 10.0*p[0]  -20.0*p[1] +10.0*p[2]);
		a_coeff[3]=(-10.0*p[0]  +30.0*p[1] -30.0*p[2] +10.0*p[3]);
		a_coeff[4]=(  5.0*p[0]  -20.0*p[1] +30.0*p[2] -20.0*p[3] +5*p[4]);
		a_coeff[5]=( -1.0*p[0]  + 5.0*p[1] -10.0*p[2] +10.0*p[3] -5*p[4] +p[5]);
	}
	else
		return false;
	return true;
}

/*! driver function for interpolator
 *  \param Program [in] list of parsed CNC-Commands
 *  \param PoseList [out] list of poses in a defined time step
*/
bool CNcInterpolator::interpolateProgram(const CNcProgram& Program, CPoseListKinetostatic& PoseList)
{
	switch(iInterpolationMethod)
	{
		case 1:
			cout << "InterpolationMethod static" << endl;
			return interpolateProgramStatic(Program,PoseList);
		case 2:
			//cout << "InterpolationMethod dynamic" << endl;
			return interpolateProgramDynamic(Program,PoseList);
		default:
			cout << "Unknown Interpolation Method selected." << endl;
			return false;
	}
}

/*! advanced interpolator with velocity profiles, bezier curves
 *  \param Program [in] list of parsed CNC-Commands
 *  \param PoseList [out] list of poses in a defined time step
*/
bool CNcInterpolator::interpolateProgramDynamic(const CNcProgram& Program, CPoseListKinetostatic& PoseList)
{
	if (listStateDyn.size()>0)
	{
		std::list<CPoseKinetostatic*>::iterator it_SDw=listStateDyn.begin(); // iterate trough the list of winch states
		for (it_SDw;it_SDw!=listStateDyn.end();it_SDw++) 
			delete (*it_SDw);
	}
	listStateDyn.clear();
	std::vector<PCRL::CPathSegments*> PathSegments;
	GeometricPathPlanning(Program,PathSegments); // create the geometric path based on the parsed g-code
	PlotPathSegments(PathSegments, LineStrip); // get a Linestrip from the geometric path
	TrajectoryPlanning(PathSegments); // calculate the trajectory based on the geometric path
	//Interpolator.PrintPathSegments(PathSegments);
	InterpolatePathSegments(PathSegments, PoseList); 
	//PrintPathSegments(PathSegments);
	//std::list<StateDyn*> sStateDyn;
	//DynamicAnalysisPathSegments(PathSegments, PoseList);
	
	//DynamicAnalysisPathSegments(PathSegments, sStateDyn);
	//ExportPoseList(PoseList);
	ExportStateDyn(listStateDyn);
	//listStateDyn=sStateDyn;
	std::vector<PCRL::CPathSegments*>::iterator it_SDw=PathSegments.begin(); // iterate trough the list of winch states
	for (it_SDw;it_SDw!=PathSegments.end();it_SDw++) 
		delete (*it_SDw);
	PathSegments.clear();
	return true;
}

/*! simple Interpolator which generates a poselist 
 *  this interpolator is a standalone version
 *  only the v_max defined in the G-Code is taken into account
 *  \param Program [in] list of parsed CNC-Commands
 *  \param PoseList [out] list of poses in a defined time step
 */
bool CNcInterpolator::interpolateProgramStatic(const CNcProgram& Program, CPoseListKinetostatic& PoseList)
{
	double length_=0;
	double length_total=0;
	double time=0;
	double time_trans=0;
	double time_rot=0;
	double time_total=0;

	list<CNcCommand*>::const_iterator it_Nc=Program.begin(); // Iterator 1
	Vector3d rTmp(StartPosition); // start position
	Vector3d anglesTmp(StartOrientation); // orientation in start position
	CPoseKinetostatic* pose_tmp;
	Vector3d r_delta;
	Vector3d angles_delta;
	Vector3d angles;
	Matrix3d Rtemp; // save the orientation in a rotation matrix
	CNcLinear *pLinear = dynamic_cast<CNcLinear *>(*it_Nc); 

	double t;
	double ipo_takt=cycleTime/1000.0/60.0; // 10 ms umgerechnet in min

	for (it_Nc;it_Nc!=Program.end();it_Nc++) 
	{
		CNcLinear* nc_lin=dynamic_cast<CNcLinear *>(*it_Nc); 
		
		CNcDwellTime* nc_dwell=dynamic_cast<CNcDwellTime *>(*it_Nc); 
		
		if (nc_lin!=0)
		{
			r_delta=nc_lin->targetposition -rTmp; // calculate the connection vector
			angles_delta=nc_lin->targetorientation -anglesTmp; // calculate the connection vector for rotation
			length_= r_delta.norm();
			length_total+=length_;	// total length
			time_trans=length_/(nc_lin->v);	// duration for translation
			time_rot=max(max(abs(angles_delta(0)),abs(angles_delta(1))),abs(angles_delta(2)))/(nc_lin->v/2); //30.000°/min -->500°/s
			time=max(time_trans,time_rot);
			time_total+=time;	// total duration
			
			t=0;

			// Sollwerttabelle generieren
			while (t<time)
			{
				t=t+ipo_takt;
				if (t>time)
					t=time;
				angles=anglesTmp+angles_delta*t/time;
				angles=angles*DEG_TO_RAD;
				Rtemp=Matrix3d::ZRotationMatrix3d(angles(2))*Matrix3d::YRotationMatrix3d(angles(1))*Matrix3d::XRotationMatrix3d(angles(0));
				pose_tmp= new CPoseKinetostatic((rTmp+r_delta*t/time)/1000.0,Rtemp);
				if (r_delta.norm()>0.0001)
					pose_tmp->v=nc_lin->v/1000.0/60.0*r_delta.normalized();
				if (angles_delta.norm()>0.0001)
					pose_tmp->omega=nc_lin->v/1000.0/60.0/2.0*angles_delta.normalized()*DEG_TO_RAD;
				
				PoseList.push_back(pose_tmp);
			}
			rTmp=nc_lin->targetposition;
			anglesTmp=nc_lin->targetorientation;
		}
		else if (nc_dwell!=0)
		{
			t=0;
			time=nc_dwell->time/60.0;
			time_total+=time;	// total duration

			// Sollwerttabelle generieren
			angles=anglesTmp*DEG_TO_RAD;
			Rtemp=Matrix3d::ZRotationMatrix3d(angles(2))*Matrix3d::YRotationMatrix3d(angles(1))*Matrix3d::XRotationMatrix3d(angles(0));
						
			while (t<time)
			{
				t=t+ipo_takt;
				if (t>time)
					t=time;
				pose_tmp= new CPoseKinetostatic(rTmp/1000.0,Rtemp);
				PoseList.push_back(pose_tmp);
			}
		}
	}

	cout << "length in mm: " << length_total << endl << "duration in min: " << time_total << endl << "number of sample pointer: " << PoseList.size() << endl;
	
	trajectoryTime=time_total*60.0;
	trajectoryLength=length_total/1000.0;
	iNrPoses=PoseList.size();
	return true;
}

/*! Function to generate jerk limited acceleration profiles with a defined v0 und v1
 *  the algorithm is from "Trajectory Planning for Automatic Machines and Robots"
 *  Luigi Biagiotti, Claudio Melchiorri, 2008, Springer, pages 79-93
 *  \param v0 [in] velocity at the begin of the path in [m/s]
 *  \param v1 [in] velocity at the end of the path in [m/s]
 *  \param vmax [in] maximal velocity in [m/s]
 *  \param amax [in] maximal acceleration and deceleration in [m/s^2]
 *  \param jmax [in] maximal jerk in [m/s^3]
 *  \param dist [in] distance in [m]
 *  \param nr [in]
 *  nr=0 | Translation
 *  nr=1 | Rotation angle 1
 *  nr=2 | Rotation angle 2
 *  nr=3 | Rotation angle 3
 *  \return value: time in [sec] for the path
 *  the results of the function where saved as membervariables in the object CPathSegments
 *  results: t[8], s[8], v[8], a[8], j[8] 
 *  the results where used in getTrajectoryPoint to generate the interpolated trajectory
 */
double CPathSegments::Ramp(double v0, double v1, double vmax, double amax, double jmax, double dist, int nr)
{
	double Tj1, Ta, Tj2, Td, Tv, T;

	// Case 1
	if ((vmax-v0)*jmax < amax*amax)
	{
		Tj1=pow((vmax-v0)/jmax,0.5);
		Ta=2.0*Tj1;
	}
	else
	{
		Tj1=amax/jmax;
		Ta=Tj1+(vmax-v0)/amax;
	}

	if ((vmax-v1)*jmax < amax*amax)
	{
		Tj2=pow((vmax-v1)/jmax,0.5);
		Td=2.0*Tj2;
	}
	else
	{
		Tj2=amax/jmax;
		Td=Tj2+(vmax-v1)/amax;
	}

	Tv=dist/vmax-Ta/2.0*(1+v0/vmax)-Td/2.0*(1+v1/vmax);
	if (Tv>0)
	{
		; // finished
	}
	else // maximum velocity is not reached
	{
		// Case 2
		Tv=0;
		bool bCorrect=true;
		bool bCorrect1, bCorrect2;
		double amax1=amax,amax2=amax;
		double delta1, delta2;
		do
		{
			Tj1=amax1/jmax;
			Tj2=amax2/jmax;
			delta1=pow(amax1*amax1/jmax,2.0)+2.0*(v0*v0+v1*v1)+amax1*(4.0*dist-2.0*amax1/jmax*(v0+v1));
			delta2=pow(amax2*amax2/jmax,2.0)+2.0*(v0*v0+v1*v1)+amax2*(4.0*dist-2.0*amax2/jmax*(v0+v1));
			Ta=(amax1*amax1/jmax-2.0*v0+pow(delta1,0.5))/(2.0*amax1);
			Td=(amax2*amax2/jmax-2.0*v1+pow(delta2,0.5))/(2.0*amax2);

			if (Ta<0||Td<0)
			{
				bCorrect=true;
				if (Ta<0)
				{
					Td=2.0*dist/(v1+v0);
					Tj2=(jmax*dist-pow(jmax*(jmax*dist*dist+(v1+v0)*(v1+v0)*(v1-v0)),0.5))/(jmax*(v1+v0));
					Tj1=0;
					Ta=0;
				}
				if (Td<0)
				{
					Ta=2.0*dist/(v1+v0);
					Tj1=(jmax*dist-pow(jmax*(jmax*dist*dist-(v1+v0)*(v1+v0)*(v1-v0)),0.5))/(jmax*(v1+v0));
					Tj2=0;
					Td=0;
				}
			}
			else
			{
				if (Ta>2.0*Tj1)
				{
					bCorrect1=true; // finished
				}
				else
				{
					bCorrect1=false;
					amax1=amax1*0.99;// scale amax until the condition is true
				}
				if (Td>2.0*Tj2)
				{
					bCorrect2=true; // finished
				}
				else
				{
					bCorrect2=false;
					amax2=amax2*0.99;// scale amax until the condition is true
				}
				bCorrect=bCorrect1 && bCorrect2;
			}
		}
		while (bCorrect==false);
	}

	T=Ta+Tv+Td; // total time=acceleration + constant velocity + deceleration

	// switching times
	t[nr][0]=0;
	t[nr][1]=Tj1;
	t[nr][2]=Ta-Tj1;
	t[nr][3]=Ta;
	t[nr][4]=Ta+Tv;
	t[nr][5]=T-Td+Tj2;
	t[nr][6]=T-Tj2;
	t[nr][7]=T;

	/*
	cout << "ti: ";
	for (int i=0;i<8;i++)
		cout << t[i] << " ";
	*/

	// jerk
	j[nr][0]=j[nr][2]=j[nr][4]=j[nr][6]=0;
	j[nr][1]=j[nr][7]=jmax;
	j[nr][3]=j[nr][5]=-jmax;

	// initial values
	s[nr][0]=0;
	v[nr][0]=v0;
	a[nr][0]=0;

	for (int i=1;i<8;i++)
	{
		s[nr][i]=s[nr][i-1] + v[nr][i-1]*(t[nr][i]-t[nr][i-1]) + 1.0/2.0*a[nr][i-1]*pow((t[nr][i]-t[nr][i-1]),2.0)+ 1.0/6.0*j[nr][i]*pow((t[nr][i]-t[nr][i-1]),3.0);
		if (i==4)
			v[nr][i]=v[nr][i-1];
		else
			v[nr][i]=v[nr][i-1] + a[nr][i-1]*(t[nr][i]-t[nr][i-1]) + 1.0/2.0*j[nr][i]*pow((t[nr][i]-t[nr][i-1]),2.0);
		
		a[nr][i]=a[nr][i-1] + j[nr][i]  *(t[nr][i]-t[nr][i-1]);
	}
	
	return T;
}

bool CStraightLine::calculateTrajectory()
{
	double v0_SI=v0/60.0/1000.0;
	double v1_SI=v1/60.0/1000.0;
	double vmax_SI=vmax/60.0/1000.0;
	double dist_SI=getLength()/1000.0;
	int i;
	double durationR[3];
	double distAngle;
	double duration_max;

	// Interpolation for Translation
	if (dist_SI>0)
	{
		duration=Ramp(v0_SI, v1_SI, vmax_SI, amax, jmax, dist_SI,0);
		duration_max=duration;
	}
	else
	{
		duration=0.0;
		for (int i2=0;i2<8;i2++)
		{
			t[0][i2]=0.0;
			s[0][i2]=0.0;
			v[0][i2]=0.0;
			a[0][i2]=0.0;
			j[0][i2]=0.0;
		}
	}

	// Interpolation for Rotation
	for (i=0;i<3;i++)
	{
		distAngle=abs((angles2(i)-angles1(i))/180.0); // Angles in degree
		if (distAngle>0)
		{
			durationR[i]=Ramp(0.0, 0.0, vmax_SI, amax, jmax, distAngle,i+1);
			duration_max=max(duration_max,durationR[i]);
		}
		else
		{
			durationR[i]=0.0;
			for (int i2=0;i2<8;i2++)
			{
				t[i+1][i2]=0.0;
				s[i+1][i2]=0.0;
				v[i+1][i2]=0.0;
				a[i+1][i2]=0.0;
				j[i+1][i2]=0.0;
			}
			
		}
	}
	// the trajectory is based on 4 single movements (1*R, 3*T) with different durations
	// with array scale the durations where scaled to the maximum duration, so all movement
	// begin and end at the same time
	if (duration_max>0)
	{
		scale[0]=duration/duration_max;
		for (i=0;i<3;i++)
		{
			scale[i+1]=max(0.0,durationR[i]/duration_max);
		}
	}
	else
	{
		for (i=0;i<4;i++)
		{
			scale[i]=0.0;
		}
	}

	return true;
}

bool CStraightLine::getTrajectoryPoint(const double t_, CPoseKinetostatic* poseKinetostatic)
{
	Vector3d r, angle, omega, alpha;
	Matrix3d Rtemp;
	if (t_*scale[0]>=t[0][7] && t_*scale[1]>=t[1][7] && t_*scale[2]>=t[2][7] && t_*scale[3]>=t[3][7])
		return false;

	int i=1;
	double s_=0.0, v_=0.0, a_=0.0;
	double vTrans=0.0, aTrans=0.0;
	if (t_*scale[0]<t[0][7])
	{
		while(!(t_*scale[0]>=t[0][i-1] && t_*scale[0]<t[0][i])) // find the segment, which corresponds to the desired time t_
		{	
			i++;
		}
		s_=s[0][i-1] + v[0][i-1]*(t_*scale[0]-t[0][i-1]) + 1.0/2.0*a[0][i-1]*pow((t_*scale[0]-t[0][i-1]),2.0)+ 1.0/6.0*j[0][i]*pow((t_*scale[0]-t[0][i-1]),3.0);
		v_=v[0][i-1] + a[0][i-1]*(t_*scale[0]-t[0][i-1]) + 1.0/2.0*j[0][i]*pow((t_*scale[0]-t[0][i-1]),2.0);
		a_=a[0][i-1] + j[0][i]  *(t_*scale[0]-t[0][i-1]);
		vTrans=v_;
		aTrans=a_;
	}
	if ((r2-r1).norm()>0.01)
		r=r1+s_*1000.0*(r2-r1)/(r2-r1).norm(); // Attention: r1 and r2 in mm, s[i] in m
	else
		r=r1;
	for (int k=0;k<3;k++)
	{
		i=0;
		s_=0.0; v_=0.0; a_=0.0;
		if (t_*scale[k+1]<t[k+1][7])
		{
			while(!(t_*scale[k+1]>=t[k+1][i-1] && t_*scale[k+1]<t[k+1][i]))
			{	
				i++;
			}
			s_=s[k+1][i-1] + v[k+1][i-1]*(t_*scale[k+1]-t[k+1][i-1]) + 1.0/2.0*a[k+1][i-1]*pow((t_*scale[k+1]-t[k+1][i-1]),2.0)+ 1.0/6.0*j[k+1][i]*pow((t_*scale[k+1]-t[k+1][i-1]),3.0);
			v_=v[k+1][i-1] + a[k+1][i-1]*(t_*scale[k+1]-t[k+1][i-1]) + 1.0/2.0*j[k+1][i]*pow((t_*scale[k+1]-t[k+1][i-1]),2.0);
			a_=a[k+1][i-1] + j[k+1][i]  *(t_*scale[k+1]-t[k+1][i-1]);
		}
		if (abs(angles2(k)-angles1(k))>0.001)
			angle(k)=angles1(k)+s_*180.0*(angles2(k)-angles1(k))/abs(angles2(k)-angles1(k));
		else
			angle(k)=angles1(k);
		omega(k)=v_;
		alpha(k)=a_;
	}
	angle=angle*DEG_TO_RAD;
	Rtemp=Matrix3d::ZRotationMatrix3d(angle(0))*Matrix3d::YRotationMatrix3d(angle(1))*Matrix3d::XRotationMatrix3d(angle(2));
	poseKinetostatic->r=r/1000.0;
	poseKinetostatic->R=Rtemp; 
	if ((r2-r1).norm()>0.01)
		poseKinetostatic->v=vTrans*(r2-r1)/(r2-r1).norm();
	else
		poseKinetostatic->v=Vector3d(0,0,0);
	poseKinetostatic->omega=omega;
	if ((r2-r1).norm()>0.01)
		poseKinetostatic->a=aTrans*(r2-r1)/(r2-r1).norm();
	else
		poseKinetostatic->a=Vector3d(0,0,0);
	poseKinetostatic->alpha=alpha;

	return true;
}


bool CBezier5::calculateTrajectory()
{
	double u=0.0; // arc length for the polynom [0, 1]
			
	Vector3d dpdu1, dpdu2; // dpdu1= dp(u)/du  dpdu2= d^2p(u)/du^2  
	double dpdu1_max=0.0, dpdu2_max=0.0;
	
	// determine the maximum value of first and second derivation in the intervall u[0,1]
	// result: the position u on the Bezier curve, where the maximum acceleration occurs
	while (u<=1.0)
	{
		dpdu1=a_coeff[1] + 2.0*a_coeff[2]*u + 3.0*a_coeff[3]*pow(u,2.0) +  4.0*a_coeff[4]*pow(u,3.0) +  5.0*a_coeff[5]*pow(u,4.0) ;
		dpdu2=			   2.0*a_coeff[2]   + 6.0*a_coeff[3]*u          + 12.0*a_coeff[4]*pow(u,2.0) + 20.0*a_coeff[5]*pow(u,3.0) ;
		dpdu1_max=max(dpdu1.norm(),dpdu1_max);
		dpdu2_max=max(dpdu2.norm(),dpdu2_max);
		u+=0.01;
	}	

	double vmax_, amax_; // [m/s] bzw [m/s^2]

	vmax_=vmax; //min(v0,v1); // gegeben in [mm/min]
	amax_=amax*60.0*60.0*1000.0; // [m/s^2] -> [mm/min^2]

	u_point_max=min(vmax_/dpdu1_max,pow(amax_/dpdu2_max,0.5)); // du/dt_max=min(vmax, amax)
	
	v0=a_coeff[1].norm()*u_point_max;
	v1=(a_coeff[1] + 2.0*a_coeff[2] + 3.0*a_coeff[3] +  4.0*a_coeff[4] +  5.0*a_coeff[5]).norm()*u_point_max ;

	return true;
}


bool CBezier5::getTrajectoryPoint(const double t_, CPoseKinetostatic* poseKinetostatic) // t_ in [sec] u_point_max in [1/min]
{
	Vector3d r, v_tmp, a_tmp,dpdu1, dpdu2;
	double u=u_point_max*t_/60.0;
	if (u>1)
		return false;
	dpdu1=a_coeff[1] + 2.0*a_coeff[2]*u + 3.0*a_coeff[3]*pow(u,2.0) +  4.0*a_coeff[4]*pow(u,3.0) +  5.0*a_coeff[5]*pow(u,4.0); // first derivation
	dpdu2=			   2.0*a_coeff[2]   + 6.0*a_coeff[3]*u          + 12.0*a_coeff[4]*pow(u,2.0) + 20.0*a_coeff[5]*pow(u,3.0); // second derivation
	r=a_coeff[0] + a_coeff[1]*u + a_coeff[2]*pow(u,2.0) + a_coeff[3]*pow(u,3.0) + a_coeff[4]*pow(u,4.0) + a_coeff[5]*pow(u,5.0); // a_coeff in [mm]
	v_tmp=u_point_max*dpdu1/1000.0/60.0;
	a_tmp=pow(u_point_max,2.0)*dpdu2/1000.0/60.0/60.0;
	
	poseKinetostatic->r=r/1000.0;
	poseKinetostatic->R=Matrix3d::ZRotationMatrix3d(this->angles1(0)*DEG_TO_RAD)*Matrix3d::YRotationMatrix3d(this->angles1(1)*DEG_TO_RAD)*Matrix3d::XRotationMatrix3d(this->angles1(2)*DEG_TO_RAD);
	poseKinetostatic->v = v_tmp;
	poseKinetostatic->omega = Vector3d(0,0,0);
	poseKinetostatic->a = a_tmp;
	poseKinetostatic->alpha = Vector3d(0,0,0);

	return true;
}

bool CDwell::calculateTrajectory()
{
	return true;
}

bool CDwell::getTrajectoryPoint(const double t_, CPoseKinetostatic* poseKinetostatic)
{
	if (t_>dwellTime)
		return false;
	poseKinetostatic->r=this->r1/1000.0; 
	poseKinetostatic->R=Matrix3d::ZRotationMatrix3d(this->angles(2))*Matrix3d::YRotationMatrix3d(this->angles(1))*Matrix3d::XRotationMatrix3d(this->angles(0));
	poseKinetostatic->v=Vector3d::Zero();
	poseKinetostatic->omega=Vector3d::Zero();
	poseKinetostatic->a=Vector3d::Zero();
	poseKinetostatic->alpha=Vector3d::Zero();
	return true;
}

} // end of namespace PCRL

