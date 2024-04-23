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
 *  \file   : Utilities.cpp
 *
 *  \Author   Philipp Miermeister
 *
 *  \Date     09.03.2011 
 *
 *********************************************************************
 */ 

#include "Utilities.h"

namespace PCRL {

// CRay methods
// ------------------------------------------------------
CRay::CRay(){}

CRay::CRay(const Vector3d &o, const Vector3d &d) 
{
	origin = o;
	direction = d;
	inv_direction = Vector3d(1/d.x(), 1/d.y(), 1/d.z());
	sign[0] = (inv_direction.x() < 0);
	sign[1] = (inv_direction.y() < 0);
	sign[2] = (inv_direction.z() < 0);
}
	
CRay::CRay(const CRay &r) 
{
	origin = r.origin;
	direction = r.direction;
	inv_direction = r.inv_direction;
	sign[0] = r.sign[0]; sign[1] = r.sign[1]; sign[2] = r.sign[2];
}

// CBox methods
// ------------------------------------------------------

CBox::CBox(){}

CBox::CBox(const Vector3d &min, const Vector3d &max) {
	  assert(min[0] < max[0] && min[1] < max[1] && min[2] < max[2]);
	  parameters[0] = min;
	  parameters[1] = max;
	}

bool CBox::intersect(const CRay &r) const 
{
	double tmin, tmax, tymin, tymax, tzmin, tzmax;

	tmin = (parameters[r.sign[0]].x() - r.origin.x()) * r.inv_direction.x();
	tmax = (parameters[1-r.sign[0]].x() - r.origin.x()) * r.inv_direction.x();
	tymin = (parameters[r.sign[1]].y() - r.origin.y()) * r.inv_direction.y();
	tymax = (parameters[1-r.sign[1]].y() - r.origin.y()) * r.inv_direction.y();
	if ( (tmin > tymax) || (tymin > tmax) ) 
		return false;
	if (tymin > tmin)
		tmin = tymin;
	if (tymax < tmax)
		tmax = tymax;
	tzmin = (parameters[r.sign[2]].z() - r.origin.z()) * r.inv_direction.z();
	tzmax = (parameters[1-r.sign[2]].z() - r.origin.z()) * r.inv_direction.z();
	if ( (tmin > tzmax) || (tzmin > tmax) ) 
		return false;

	return true;
}

//! Edge constructor
CEdge::CEdge(unsigned int v1ID, unsigned int v2ID, int ed1ID, int ed2ID)
{
	this->v1ID = v1ID;
	this->v2ID = v2ID;
	this->ed1ID = ed1ID;
	this->ed2ID = ed2ID;
	inverse = false;
}

//! Invert edge
void CEdge::invert()
{
	inverse = !inverse;
	swap(v1ID,v2ID);
	swap(ed1ID,ed2ID);
}


// general utility functions
////////////////////////////

//! utility function for calculating the duration of a one-axis trajectory 
//! segment based on a 3rd order differential motion profil
//! \return the time for the path; the return value is <0 if the parameters are invalid
double getTrajectoryTime(const double& ds, const double& vmax, const double& amax, const double& rmax)
{
	// parameter valid?
	if (ds<=0 || vmax<=0 || amax<= 0 || rmax <= 0)
		return -1;

	double T=0;				// time for the full trajectory
	double T1=0,T2=0,T4=0;	// time for the different phases (totally 7 phases exists, where only 3 have different values)

	// which of the four trajectory type is given
	// the formalism is taken from robotics script, p. 80, Uni DuE, Chair for Mechatronics
	// where the equations were origianlly given by W. Risse (1993)
	int Fall=0;
	if (sqrt(vmax/rmax)<=(amax/rmax))
	{	
		// Fall 2 oder 4
		if (ds>=2*rmax*pow(vmax/rmax,1.5))
		{
			// Fall 2;
			T1 = sqrt(vmax/rmax);
			T2 = 0;
			T4 = (ds-2*rmax*pow(T1,3))/vmax;
		}
		else
		{
			// Fall 4;
			T1 = pow(ds/(2*rmax),1/3.0);
			T2 = 0;
			T4 = 0;
		}
	}
	else
	{
		double T1max = sqrt(vmax/rmax);
		double T2max = vmax/(rmax*T1max)-T1max;
		// Fall 1 oder 3
		if (ds>=rmax*T1max*(2*T1max*T1max+T2max*T2max+3*T1max*T2max))
		{
			// Fall 1
			T1 = amax / rmax;
			T2 = vmax / amax - T1;
			T4 = (ds - rmax*T1*(2*T1*T1 + T2*T2 + 3*T1*T2))/vmax;
		}
		else
		{
			// Fall 3
			T1 = amax / rmax;
			T2 = -3*T1/2 + sqrt(T1*T1/4 + ds/(rmax*T1));
			T4 = 0;
		}
	}
	// calculation of the duration
	T = 4*T1 + 2*T2 + T4;
	return T;
}


/*! read the position vector attributes of a given tag "parent" and store it in value
 *  value is only modified if (and only if) all attributed x,y,z where found and successfully
 *  read from the tag.*/
bool getPositionVectorAttrib(const TiXmlElement* parent, Vector3d& value)
{
	if (!parent)
		return false;
	Vector3d tmp(0,0,0);
	if (parent->QueryDoubleAttribute("x",&tmp.x())==TIXML_SUCCESS &&
		parent->QueryDoubleAttribute("y",&tmp.y())==TIXML_SUCCESS &&
		parent->QueryDoubleAttribute("z",&tmp.z())==TIXML_SUCCESS)
	{
		value = tmp;
		return true;
	}
	else 
		return false;
}

//! write the position vector attributes into a given tag "parent" 
bool setPositionVectorAttrib(TiXmlElement* parent, const Vector3d& value)
{	
	parent->SetDoubleAttribute("x",value.x());
	parent->SetDoubleAttribute("y",value.y());
	parent->SetDoubleAttribute("z",value.z());
	return true;
}

/*! get the orientation matrix attributes of a given tag "parent" and store it in value
 *  value is only modified if (and only if) all nine attributes of the matrix where found 
 *  and successfully read from the tag.*/
bool getOrientationMatrixAttrib(const TiXmlElement* parent, Matrix3d& value)
{	
	if (!parent)
		return false;
	Matrix3d tmp;
	
	tmp.setIdentity();
	if (parent->QueryDoubleAttribute("a11",&tmp.col(0).x())==TIXML_SUCCESS && // read the values from the tags and write into tmp
		parent->QueryDoubleAttribute("a21",&tmp.col(0).y())==TIXML_SUCCESS && 
		parent->QueryDoubleAttribute("a31",&tmp.col(0).z())==TIXML_SUCCESS && 
		parent->QueryDoubleAttribute("a12",&tmp.col(1).x())==TIXML_SUCCESS && 
		parent->QueryDoubleAttribute("a22",&tmp.col(1).y())==TIXML_SUCCESS && 
		parent->QueryDoubleAttribute("a32",&tmp.col(1).z())==TIXML_SUCCESS && 
		parent->QueryDoubleAttribute("a13",&tmp.col(2).x())==TIXML_SUCCESS && 
		parent->QueryDoubleAttribute("a23",&tmp.col(2).y())==TIXML_SUCCESS && 
		parent->QueryDoubleAttribute("a33",&tmp.col(2).z())==TIXML_SUCCESS) 
		
	{
		value = tmp;
		return true;
	}
	else 
		return false;
}

/*! get the inertia tensor attributes of a given tag "parent" and store it in value
 *  value is only modified if (and only if) all nine attributes of the matrix where found 
 *  and successfully read from the tag.*/
bool getInertiaTensorAttrib(const TiXmlElement* parent, Matrix3d& value)
{	
	if (!parent)
		return false;
	Matrix3d tmp = Matrix3d::Identity();
	if (parent->QueryDoubleAttribute("Ixx",&tmp.col(0).x())==TIXML_SUCCESS && // read the values from the tags and write into tmp
		parent->QueryDoubleAttribute("Iyx",&tmp.col(0).y())==TIXML_SUCCESS && 
		parent->QueryDoubleAttribute("Izx",&tmp.col(0).z())==TIXML_SUCCESS && 
		parent->QueryDoubleAttribute("Ixy",&tmp.col(1).x())==TIXML_SUCCESS && 
		parent->QueryDoubleAttribute("Iyy",&tmp.col(1).y())==TIXML_SUCCESS && 
		parent->QueryDoubleAttribute("Izy",&tmp.col(1).z())==TIXML_SUCCESS && 
		parent->QueryDoubleAttribute("Ixz",&tmp.col(2).x())==TIXML_SUCCESS && 
		parent->QueryDoubleAttribute("Iyz",&tmp.col(2).y())==TIXML_SUCCESS && 
		parent->QueryDoubleAttribute("Izz",&tmp.col(2).z())==TIXML_SUCCESS) 
		
	{
		value = tmp;
		return true;
	}
	else 
		return false;
}

/*! get the quaternion attributes of a given tag "parent" and store it in value
 *  value is only modified if (and only if) all nine attributes of the matrix where found 
 *  and successfully read from the tag.*/
bool getQuaternionAttrib(const TiXmlElement* parent, double& q0, double& q1, double& q2, double& q3)
{	
	if (!parent)
		return false;
	double tq0;
	double tq1;
	double tq2;
	double tq3;

	if (parent->QueryDoubleAttribute("q0",&tq0)==TIXML_SUCCESS && // read the values from the tags and write into tmp
		parent->QueryDoubleAttribute("q1",&tq1)==TIXML_SUCCESS && 
		parent->QueryDoubleAttribute("q2",&tq2)==TIXML_SUCCESS && 
		parent->QueryDoubleAttribute("q3",&tq3)==TIXML_SUCCESS)
	{
		q0 = tq0;
		q1 = tq1;
		q2 = tq2;
		q3 = tq3;
		return true;
	}
	else 
		return false;
}

/*! get the axis angle attributes of a given tag "parent" and store it in value
 *  value is only modified if (and only if) all nine attributes of the matrix where found 
 *  and successfully read from the tag.*/
bool getAxisAngleAttrib(const TiXmlElement* parent, double& beta, double& ux, double& uy, double& uz)
{	
	if (!parent)
		return false;
	double tbeta;
	double tux;
	double tuy;
	double tuz;

	if (parent->QueryDoubleAttribute("beta",&tbeta)==TIXML_SUCCESS && // read the values from the tags and write into tmp
		parent->QueryDoubleAttribute("ux",&tux)==TIXML_SUCCESS && 
		parent->QueryDoubleAttribute("uy",&tuy)==TIXML_SUCCESS && 
		parent->QueryDoubleAttribute("uz",&tuz)==TIXML_SUCCESS)
	{
		beta = tbeta;
		ux = tux;
		uy = tuy;
		uz = tuz;
		return true;
	}
	else 
		return false;
}

//! write the orientation matrix attributes into a given tag "parent" 
bool setOrientationMatrixAttrib(TiXmlElement* parent, const Matrix3d& value)
{	
	parent->SetDoubleAttribute("a11",value.col(0).x());
	parent->SetDoubleAttribute("a21",value.col(0).y());
	parent->SetDoubleAttribute("a31",value.col(0).z());

	parent->SetDoubleAttribute("a12",value.col(1).x());
	parent->SetDoubleAttribute("a22",value.col(1).y());
	parent->SetDoubleAttribute("a32",value.col(1).z());

	parent->SetDoubleAttribute("a13",value.col(2).x());
	parent->SetDoubleAttribute("a23",value.col(2).y());
	parent->SetDoubleAttribute("a33",value.col(2).z());
	
    return true;
}

//! write the Tensor Matrix attributes into a given tag "parent" 
bool setInertiaTensorAttrib(TiXmlElement* parent, const Matrix3d& value)
{	
	parent->SetDoubleAttribute("Ixx",value.col(0).x());
	parent->SetDoubleAttribute("Ixy",value.col(1).x());
	parent->SetDoubleAttribute("Ixz",value.col(2).x());

	parent->SetDoubleAttribute("Iyx",value.col(0).y());
	parent->SetDoubleAttribute("Iyy",value.col(1).y());
	parent->SetDoubleAttribute("Iyz",value.col(2).y());

	parent->SetDoubleAttribute("Izx",value.col(0).z());
	parent->SetDoubleAttribute("Izy",value.col(1).z());
	parent->SetDoubleAttribute("Izz",value.col(2).z());
	return true;
}

//! read a scalar attribute of a given tag "parent" and store it in value
bool getScalarAttrib(const TiXmlElement* parent, double& value)
{
	return false;
}

/*
 * \param abc is a vector R^3 with three angles
 * \return the rotation matrix R=Rz(a)*Ry(b)*Rx(c) where a,b,c are 
 *         the components of the vector abc
 */
bool getMatrixFromXYZ(Matrix3d& R, const Vector3d& abc)
{
	R = Matrix3d::ZRotationMatrix3d(abc[0])*Matrix3d::YRotationMatrix3d(abc[1])*Matrix3d::XRotationMatrix3d(abc[2]);
	return true;
}

//! get the XYZ vector from Rotation Matrix
bool getXYZFromMatrix(Vector3d& abc, const Matrix3d& R)
{
	return getXYZFromMatrix(abc(0),abc(1),abc(2), R);
}


//! get the XYZ from Rotation Matrix
bool getXYZFromMatrix(double& a, double& b, double& c, const Matrix3d& R)
{
	if (R(2,0) < +1)
	{
		if (R(2,0) > -1)
		{
			b = asin(-R(2,0)); //thetaY
			a = atan2(R(1,0),R(0,0)); //thetaZ
			c = atan2(R(2,1),R(2,2)); //thetaX
		}
		else // r20 = -1
		{
		// Not a unique solution: thetaX - thetaZ = atan2(-r12,r11)
			b = (4.0*atan(1.0))/2;  //thetaY
			a = -atan2(-R(1,2),R(1,1)); //thetaZ
			c = 0; //thetaX
		}
	}
	else // r20 = +1
	{
	// Not a unique solution: thetaX + thetaZ = atan2(-r12,r11)
		b = -(4.0*atan(1.0))/2; //thetaY
		a = atan2(-R(1,2),R(1,1)); //thetaZ
		c = 0; //thetaX
	}
	return true;
}

//! get the rotation matrix from a quaternion
bool getMatrixFromQuaternion(Matrix3d& R, const double& q0, const double& q1, const double& q2, const double& q3)
{
	if((q0*q0+q1*q1+q2*q2+q3*q3) < 0.00001) // if Quaternion is zeros, return identity matrix
	{
		R(0,0) = 1;
		R(0,1) = 0;
		R(0,2) = 0;
		R(1,0) = 0;
		R(1,1) = 1;
		R(1,2) = 0;
		R(2,0) = 0;
		R(2,1) = 0;
		R(2,2) = 1;
	}
	//Normalize Quaternion
	double magnitude = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
 	double nq0 = q0 / magnitude;
	double nq1 = q1 / magnitude;
	double nq2 = q2 / magnitude;
	double nq3 = q3 / magnitude;

	//Implement conversion
	R(0,0) = 1 - 2*nq2*nq2 - 2*nq3*nq3;
	R(0,1) = 2*nq1*nq2-2*nq0*nq3;
	R(0,2) = 2*nq1*nq3+2*nq2*nq0;
	R(1,0) = 2*nq1*nq2+2*nq3*nq0;
	R(1,1) = 1 - 2*nq1*nq1 - 2*nq3*nq3;
	R(1,2) = 2*nq2*nq3 -2*nq1*nq0;
	R(2,0) = 2*nq1*nq3 - 2*nq2*nq0;
	R(2,1) = 2*nq2*nq3 + 2*nq1*nq0;
	R(2,2) = 1 - 2*nq1*nq1 - 2*nq2*nq2;
	return true;
}

//! get the quaternion from rotation matrix
bool getQuaternionFromMatrix(double& q0, double& q1, double& q2, double& q3, const Matrix3d& R)
{
	double diagonale = R(0,0) + R(1,1) + R(2,2); //sum of matrix diagonals
	double wurz;			//parameter to store intermediate value sqrt(diagonals+1)*2 for ease of calculation
	if(diagonale > 0)		//if sum of diagonals is not 0 implement basic equation
	{
		wurz = sqrt(diagonale+1)*2;
		q0 = wurz/4;
		q1 = (R(2,1)-R(1,2))/wurz;
		q2 = (R(0,2)-R(2,0))/wurz;
		q3 = (R(1,0)-R(0,1))/wurz;
	}
	else if ((R(0,0)>R(1,1))&&(R(0,0)>R(2,2)))	//if diagonals = 0 and term R(1,1) is the greatets implement modified equation
	{
		wurz = sqrt(1.0+R(0,0)-R(1,1)-R(2,2))*2;
		q0 = (R(2,1)-R(1,2))/wurz;
		q1 = wurz/4;
		q2 = (R(0,1)+R(1,0))/wurz;
		q2 = (R(0,2)+R(2,0))/wurz;
	}
	else if (R(1,1)>R(2,2))					//if diagonals = 0 and term R(2,2) is the greatets implement modified equation
	{
		wurz = sqrt(1.0 + R(1,1)-R(0,0)-R(2,2))*2;
		q0 = (R(0,2)-R(2,0))/wurz;
		q1 = (R(0,1)-R(1,0))/wurz;
		q2 = wurz/4;
		q3 = (R(1,2)+R(2,1))/wurz;
	}
	else									//if diagonals = 0 and term R(2,2) is the greatets implement modified equation
	{
		wurz = sqrt(1.0 + R(2,2)-R(0,0)-R(1,1))*2;
		q0 = (R(1,0)-R(0,1))/wurz;
		q1 = (R(0,2)+R(2,0))/wurz;
		q2 = (R(1,2)+R(2,1))/wurz;
		q3 = wurz/4;
	}
	return true;
}

//! get Axis Angle from rotation matrix
bool getAxisAngleFromMatrix(double& beta, double& ux, double& uy, double& uz, const Matrix3d& R)
{
	double epsilon = 0.0000001; // margin to allow for rounding errors
	double epsilon2 = 0.1; // margin to distinguish between 0 and 180 de

	if ((fabs(R(0,1)-R(1,0))< epsilon) && (fabs(R(0,2)-R(2,0))< epsilon) && (fabs(R(1,2)-R(2,1))< epsilon)) // singularity 
	{
		if ((fabs(R(0,1)+R(1,0)) < epsilon2) && (fabs(R(0,2)+R(2,0)) < epsilon2) && (fabs(R(1,2)+R(2,1)) < epsilon2) && (fabs(R(0,0)+R(1,1)+R(2,2)-3) < epsilon2)) // first check for identity matrix which must have +1 for all terms
		//  in leading diagonal and zero in other terms
		{
			// this singularity is identity matrix so angle = 0
			beta=0;
			ux = 69;
			uy = 69;
			uz = 69;
		}
		else // otherwise this singularity is angle = 180
		{
			beta = 3.141592653589793238462;
			double xx = (R(0,0)+1)/2;
			double yy = (R(1,1)+1)/2;
			double zz = (R(2,2)+1)/2;
			double xy = (R(0,1)+R(1,0))/4;
			double xz = (R(0,2)+R(2,0))/4;
			double yz = (R(1,2)+R(2,1))/4;
			if ((xx > yy) && (xx > zz))  // R(1,1) is the largest diagonal term
			{
				if (xx< epsilon) 
				{
					ux = 0;
					uy = 1;
					uz = 1;
				} 
				else 
				{
					ux = sqrt(xx);
					uy = xy/ux;
					uz = xz/ux;
				}
			} 
			else if (yy > zz) // R(2,2) is the largest diagonal term
			{ 
				if (yy< epsilon) 
				{
					ux = 1;
					uy = 0;
					uz = 1;
				} 
				else 
				{
					uy = sqrt(yy);
					ux = xy/uy;
					uz = yz/uy;
				}
			} 
			else  // R(3,3) is the largest diagonal term so base result on this
			{
				if (zz< epsilon)
				{
					ux = 0.7071;
					uy = 0.7071;
					uz = 0;
				} 
				else 
				{
					uz = sqrt(zz);
					ux = xz/uz;
					uy = yz/uz;
				}
			
			}
		}
		// return 180 deg Rotation
	}
	else //no singularity
	{
	beta = acos((R(0,0) + R(1,1) + R(2,2) - 1)/2);
	double s = sqrt((R(2,1) - R(1,2))*(R(2,1) - R(1,2))
		+(R(0,2) - R(2,0))*(R(0,2) - R(2,0))
		+(R(1,0) - R(0,1))*(R(1,0) - R(0,1)));

	ux = (R(2,1) - R(1,2))/s;
	uy = (R(0,2) - R(2,0))/s;
	uz = (R(1,0) - R(0,1))/s;
	}
	
	//normalize
	double AAl= sqrt(ux*ux+uy*uy+uz*uz);
	ux = ux/AAl;
	uy = uy/AAl;
	uz = uz/AAl;
	return true;

}	

//! get rotation matrix from Axis Angle
bool getMatrixFromAxisAngle(Matrix3d& R, const double& beta, const double& ux, const double& uy, const double& uz)
{
	//Normalize Vector
	double nux, nuy, nuz;
	if((ux*ux+uy*uy+uz*uz) != 0)
	{
		double AAl= sqrt(ux*ux+uy*uy+uz*uz);
		nux = ux/AAl;
		nuy = uy/AAl;
		nuz = uz/AAl;
	}
	else
	{
		nux = ux;
		nuy = uy;
		nuz = uz;
	}

	//Implement Equations
	R(0,0) = (1-cos(beta))*nux*nux+cos(beta);
	R(0,1) = (1-cos(beta))*nux*nuy-nuz*sin(beta);
	R(0,2) = (1-cos(beta))*nux*nuz+nuy*sin(beta);
	R(1,0) = (1-cos(beta))*nux*nuy+nuz*sin(beta);
	R(1,1) = (1-cos(beta))*nuy*nuy+cos(beta);
	R(1,2) = (1-cos(beta))*nuy*nuz-nux*sin(beta);
	R(2,0) = (1-cos(beta))*nux*nuz-nuy*sin(beta);
	R(2,1) = (1-cos(beta))*nuy*nuz+nux*sin(beta);
	R(2,2) = (1-cos(beta))*nuz*nuz+cos(beta);

	return true;
}

//! get Axis Angle from quaternion
bool getAxisAngleFromQuaternion(double& beta, double& ux, double& uy, double& uz, const double& q0, const double& q1, const double& q2, const double& q3)
{
	beta = 2*acos(q0);
	if ((1-q0*q0)<0.00001) //zero turn on quaternion results in null vector 
	{
		ux = 0;
		uy = 0;
		uz = 0;
	}
	else
	{
		ux = q1 / sqrt(1-q0*q0);
		uy = q2 / sqrt(1-q0*q0);
		uz = q3 / sqrt(1-q0*q0);
	}
	return true;
}

//!get quaternion from Axis Angle
bool getQuaternionFromAxisAngle(double& q0, double& q1, double& q2, double& q3, const double& beta, const double& ux, const double& uy, const double& uz)
{
	q0 = cos(beta/2);
	q1 = ux*sin(beta/2);
	q2 = uy*sin(beta/2);
	q3 = uz*sin(beta/2);
	return true;
}

//! get Rodrigues Parameters from Axis Angle
bool getAxisAngleFromQuaternion(double& Rx, double& Ry, double& Rz, const double& beta, const double& ux, const double& uy, const double& uz)
{
	if((ux*ux+uy*uy+uz*uz)<0.00001) //if Axis is 0 return 0
	{
		Rx = 0;
		Ry = 0;
		Rz = 0;
	}
	else //normalize vector with length of beta (angle)
	{
		Rx = ux*beta/sqrt(ux*ux+uy*uy+uz*uz);
		Ry = uy*beta/sqrt(ux*ux+uy*uy+uz*uz);
		Rz = uz*beta/sqrt(ux*ux+uy*uy+uz*uz);
	}
	return true;
}

bool getAxisAngleFromRodrigues(double& beta, double& ux, double& uy, double& uz, const double& Rx, const double& Ry, const double& Rz)
{
	if((Rx*Rx+Ry*Ry+Rz*Rz) < 0.00001) //if Axis is 0 return 0
	{
		beta = 0;
		ux = 0;
		uy = 0;
		uz = 0;
	}
	else //implement conversion
	{
		beta = sqrt(Rx*Rx+Ry*Ry+Rz*Rz);
		ux = Rx/beta;
		uy = Ry/beta;
		uz = Rz/beta;
	}
	return true;
}

/*!
 * compute a uniformly distributed random orientation matrix
 * \param R the matrix to be computed
 * \return true, if successful otherwise false
 */
bool getMatrixRandomOrientation(Matrix3d& R)
{
	// generate a normal distributed random vector v in Re^4
	MatrixXd v(4,1);
	for (int i=0; i<4; i++)
	{
		// get two pseudo-random numbers x1, x2 in [0,1]
		double x1 = (double)rand() / (double)RAND_MAX;
		double x2 = (double)rand() / (double)RAND_MAX;
		v(i) = sqrt(-2.0 * log(x1)) * cos(2.0 * MO_PI * x2);
	}

	// normalize the vector and interprete it as quaternion
	v /= v.norm();

	// transforming the vector which is understood to be a unit quaternion into a 
	// rotation matrix gives a uniformly distributed random orientation in SO(3)
	return getMatrixFromQuaternion(R, v(0), v(1), v(2), v(3));
}

/*! Calculate the intersection between a line segment and a triangle. 
 *  The line segment is given by the vector a and the direction n_a
 *  L : x = a + lambda* n_a   with   lambda in [0,1]
 *  and the triangle is defined by the vertex u and the direction vectors n_v and n_w
 *  T : x = u + nu * n_v + mu * n_w  with nu,mu,nu+mu in [0,1]
 *  \return true, if a intersection was detected otherweise false.
 *  \todo consider also the special cases where det(A)=0
 */
bool getIntersectionTriangleLine(const Vector3d& a, const Vector3d& n_a, const Vector3d& u, const Vector3d& n_v, const Vector3d& n_w)
{
	const double eps = 1e-6;		// if the determinant of the system matrix is smaller eps, it is considered singular
	// setup the system matrix for the linear system
	Matrix3d A;
	A << n_a, -n_v, -n_w;
	// setup the right-hand-side 
	Vector3d B = u-a;
	// just for debugging purpose, check the determinant for zero before solving
	if (fabs(A.determinant())<eps)
		return false;
	// solve the system (perhapts other solvers are faster...)
	Vector3d sol = A.fullPivLu().solve(B);
	// check if lambda (alias x), mu and nu is in the range [0,1]; if all conditions apply we have an intersection
	if (0 <= sol.x() && sol.x() <= 1 &&
		0 <= sol.y() && 
		0 <= sol.z() && 
		sol.y() + sol.z()  <= 1)
		return true;
	else
		return false;
}

/*! Calculate the "distance" between two rotation matrices
 *
 * R_1^T*R_2 = R_diff
 * R_diff converted to axis angle representation
 * 
 * returns absolute value of angle
 */
double getSO3Distance(const Matrix3d& R_1, const Matrix3d& R_2)
{
	Matrix3d R_difference; //rotation from R_1 to R_2
	double angle,x,y,z;
	
	R_difference = R_1.transpose()*R_2;
	//cout<<R_difference<<endl;
	getAxisAngleFromMatrix(angle, x, y, z, R_difference);
	//cout<<angle<<' '<<x<<' '<<y<<' '<<z<<endl;
	return abs(angle);
}

/*! get the eigenvectors and eigenvalues for a given 3D-Matrix */
bool getEigenvectorBase(const Matrix3d& K, Matrix3d& eigenvectors, Vector3d& eigenvalues)
{
	Eigen::SelfAdjointEigenSolver<Matrix3d> es(K);
	eigenvectors = es.eigenvectors();
	eigenvalues = es.eigenvalues();
	cout << "The eigenvalues of A are:" << endl << es.eigenvalues() << endl;
	cout << "The matrix of eigenvectors, V, is:" << endl << es.eigenvectors() << endl << endl;
	return true;
}


void generateWireErrors(MatrixXd& l, int now, MatrixXd& wireErrors, emWireErrors::Type wireErrorType, double errorrange)
{
	unsigned int seed = 0;
	if(wireErrorType == emWireErrors::poseConsistent){
		for(int i = 0; i < now; i++)
			seed += (unsigned int)(l(i)*1000);
	}
	else
	{
		seed = (unsigned int)time(NULL);
	}
	srand(seed);
	// \todo Rewrite this look; after change from double* to MatrixXd this should be much easier to compute
	for(int i = 0; i < now; i++)
	{
		wireErrors(i) = (((double)rand()*2)/RAND_MAX -1)*errorrange;
		l(i)+=wireErrors(i);
	}
}

//! Get current date/time, format is YYYY-MM-DD HH:mm:ss
const string currentDateTime() 
{
    time_t now = time(0);
    tm tstruct;
    char ret[40];
    tstruct = *localtime(&now);
    strftime(ret, sizeof(ret), "%Y-%m-%d %X", &tstruct);
    return string(ret);
}

// for some reason we do not have a stddev function. The implementation is quite un-eigen, but shoud
// basically work as long as we call it for a vector shaped matrix
double stddev(const MatrixXd& v)
{
	double mean = v.mean();
	double sum=0.0;
	for (int i=0; i<v.rows(); ++i)
		sum+= (v(i)-mean)*(v(i)-mean);
	return sqrt(sum/(v.size()-1.0));
}

/*! simple implementation of GN-Solver, based on levmar function call signature.	*
 *		-jacobian is calculated and does not need to be given.						*
 *  p - solution vector																*
 *	m x n gives the problem dimensions 												*
 *  l is the evaluation result of dimension n										*
 *  *adata pointer for miscalaneous data											*/
bool Gauss_Newton(void (*func)(double*p, double *l,int m, int n,void *adata), double*p,int itmax,double *info,int m, int n,double del,void *adata)
{
	MatrixXd residue(n,1);								//initialized an array for storing the resdiual value		
	MatrixXd fwd_val(n,1);								//initialized an array for storing  value of function at (x_i+del)
	double *temp = new double[sizeof(double)*n];		//temp array
	MatrixXd jac(n,m);									//Matrix of 8 rows and 6 columns to store jacobian
	MatrixXd jacT(n,m); 
	MatrixXd jacTjac(m,m);								//Matrix for product of jacobian transposed with jacobian
	MatrixXd stat_p(m,1);								//Vector for calculating the stationary point			
	MatrixXd step(m,1);									//descent value
	int i = 0;
	//void *adata = this;	
	double *hx = new double[sizeof(double)*n];
	MatrixXd err_fin_objfunc(n,1),err_ini_objfunc(n,1);
	double eps1 = 1e-10;			// ||J^T e||_inf 10
	double eps2 = 1e-9;				// ||Dp||_2  9 
	double eps3 = 1e-10;				// ||e||_2	4
	double d;
	
	//Calculation for info[0] = ||e||_2 pose from pose estimation function.
	(*func)(p,hx,m,n,adata);
	err_ini_objfunc = Eigen::Map<MatrixXd>(hx,n,1);
	info[0] = err_ini_objfunc.norm();
	info[4] = info[4]+1; 
	for (i = 0; i < itmax; i++)
	{
		//calculate the residue value of the function at the estimated pose
		(*func)(p,hx,m,n,adata);
		residue = Eigen::Map<MatrixXd>(hx,n,1);			//array converted to matrix for easier calculations
		
		memcpy(temp,p,sizeof(double)*n);				//initializing an array for forward difference method of differentiation
		
		for (int k = 0; k < m; k++)
		{
			d = abs(del*temp[k]);
			if (d<del)									//determine d=max(1E-04*|p[j]|, del)
				d=del;									
			temp[k] = temp[k] + d;						//ith value of array as(x+delta_h)
			(*func)(temp,hx,m,n,adata);					//function value at incremented ith value
			fwd_val = Eigen::Map<MatrixXd>(hx,n,1);			//array converted to matrix
			for (int j = 0; j<n; j++)
			{
				jac(j, k) = (fwd_val(j) - residue(j)) / d;
			}
			temp[k] = temp[k] - d;
		}
		
		//descent direction
		jacT = jac.transpose();
		jacTjac = (jacT)*jac;					//J^T*J
		
		stat_p = (jacT)*residue;				//stationary point
		info[2] = stat_p.maxCoeff();					//||J^T e||_inf

		//calculating the descent step
		step = -1*jacTjac.ldlt().solve(stat_p);	
		info[3] = step.norm();							//Calculation for info[3] = ||Dp||_2 for final calculated pose	
		
		for (int it=0;it<m;it++)			
		{
			p[it] = p[it] + step(it);				//P_i+1 = P_i + step
		}
				
		(*func)(p,hx,m,n,adata);
		err_fin_objfunc = Eigen::Map<MatrixXd>(hx,n,1);
		info[1] = err_fin_objfunc.norm();				//Calculation for info[1] = ||e||_2 for final calculated pose.
		
		//exit conditions
		if (info[1] < eps3)
			{info[6] = 6;								//stopped by small ||e||_2
				break;}
		else if (info[2]<eps1)
		{info[6] = 1;									//stopped by small gradient J^T e
			break;}
		else if(info[3] < eps2)
			{info[6] = 2;								//stopped by small Dp
			break;}
	}
	if(i == itmax)										//stopped by maximum iterations
			{info[6] = 3;}	
	info[5]=i;											//gives the number of iterations
	return true;
}

}// namespace PCRL