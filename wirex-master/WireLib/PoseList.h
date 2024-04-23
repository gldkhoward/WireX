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

/*! \file   : PoseList.h
 *
 *  \Author   Philipp Miermeister
 *
 *  \Date     22.02.2012 
 *
 *  \brief The class provides the data stucture and some basic functions
 *	in order to generate and manipulate pose lists
 **********************************************************************/ 

#pragma once

#include <list>
#include <string>
#include <WireLib/EigenLib.h>
#include <motionPlanning/Utilities.h>

namespace PCRL {

/*! Static pose class
 *  A static pose consists of a position r and the orientation matrix R.
 */
class CPoseStatic
{
public:
	//! Constructor expects position and rotation matrix
	CPoseStatic(const Vector3d& r, const Matrix3d& R) { this->r=r;this->R=R; }

	//! Constructor expects position and axis angle 
	CPoseStatic(const Vector3d& r, const double& angle, const Vector3d& axis)
	{
		this->r=r;
		getMatrixFromAxisAngle(this->R, angle, axis[0], axis[1], axis[2]);
	};
	~CPoseStatic() {}
public:
	// data model
	Vector3d r; //!< position
	Matrix3d R;	//!< orientation

public:
	// methods

	//! The function returns the axis angle representation of the platform rotation
	void getAxisAngle(double& angle_out, Vector3d& axis_out)
	{
		getAxisAngleFromMatrix(angle_out, axis_out[0], axis_out[1], axis_out[2], R);
	}

	//! get the content of the pose as filestream
	void getString(ostream& file, Eigen::IOFormat IOFormat_){file << r.format(IOFormat_) << " , " << R.format(IOFormat_);}
};


//! Kinetostatic pose class
class  CPoseKinetostatic : public CPoseStatic
{
public:
	CPoseKinetostatic(const Vector3d& r, const Matrix3d& R) : CPoseStatic(r,R)
	{
		this->v = Vector3d::Zero(); this->omega =  Vector3d::Zero(); this->a = Vector3d::Zero(); this->alpha = Vector3d::Zero();
		this->f = Vector3d::Zero(); this->tau =  Vector3d::Zero();
	}

	CPoseKinetostatic(const Vector3d& r, const Matrix3d& R, const Vector3d& v,
					  const Vector3d& omega, const Vector3d& a,const Vector3d& alpha,
					  const Vector3d& f, const Vector3d& tau):CPoseStatic(r,R)
	{
		this->v = v; this->omega = omega; this->a =a;
		this->f =f; this->tau = tau;
	}

	CPoseKinetostatic(const Vector3d& r, const double& angle, const Vector3d& axis):CPoseStatic(r,angle, axis)
	{
		this->v = Vector3d::Zero(); this->omega =  Vector3d::Zero(); this->a = Vector3d::Zero(); this->alpha = Vector3d::Zero();
		this->f = Vector3d::Zero(); this->tau =  Vector3d::Zero();
	}
	~CPoseKinetostatic(){ }

public:
	// methods

	//! get the content of the pose as filestream
	void getString(ostream& file, Eigen::IOFormat IOFormat_){file << r.format(IOFormat_) << " , " << R.format(IOFormat_) << " , " << v.format(IOFormat_) << " , " << omega.format(IOFormat_) << " , " << a.format(IOFormat_) << " , " << alpha.format(IOFormat_) << " , " << f.format(IOFormat_) << " , " << tau.format(IOFormat_);}

public:
	// data model
	Vector3d v,omega;	//!< the twist of the platform, i.e. linear and angular velocity
	Vector3d a,alpha;	//!< acceleration, angular acceleration
	Vector3d f;			//!< force
	Vector3d tau;		//!< torque
};

//----------------------------------------------------------------------------

/*! Template base class for pose lists. Poses are stored in this list as 
 *  pointers this class takes care about the internal memory management of 
 *  the stored poses.
 *  The constructor is protected to prohibit instances of the class.
 *  The class contains some basic generator functions for pose lists.
 *  
 *  \todo FIXME: Pose list stores its entries in pointer and allocates memory
 *        dynamically but do not implement appropriate copy and assignment operations.
 */
template <class T>
class CPoseList: public std::list<T*>
{
private:
	// we put private implementations here since the current implementation does not
	// support a copy constructor and assignment operations.
//	CPoseList(const CPoseList& src) {}
//	CPoseList& operator=(const CPoseList& src) {}
public:
	enum FileFormat{
		MAPLE,
		MATLAB,
		CSV};

protected:
	typename std::list<T*>::iterator cursor;
	
public:
	CPoseList() { cursor=std::list<T*>::begin(); }

	~CPoseList()
	{
		deleteAllPoses();
	}

	//! put the cursor to the beginning of the list
	void resetCursor()
	{
		cursor=std::list<T*>::begin();
	}

	//! get the current pose, i.e. the pose that is marked with the cursor
	T* getCurrent()
	{
		if (cursor!=std::list<T*>::end())
			return *cursor;
		else
			return 0;
	}
	//! iterate the cursor and get the new pose
	T* nextPose()
	{
		cursor++;
		return getCurrent();
	}

	//! revert the cursor and get the old pose
	T* previousPose()
	{
		cursor--;
		return getCurrent();
	}
	
	/*! Generate uniformly distributed spatial grid
	 *  \param r_min [in] min values for r.x, r.y, r.z
	 *  \param r_max [in] max values for r.x, r.y, r.z
	 *  \param r_stepSize [in] grid step size in x-, y-, z-direction
	 *  \todo Add a parameter checking: validate r_max > r_min and r_stepsize>0
	 */
	void generate_uniformGrid(const Vector3d& r_min, const Vector3d& r_max, const Vector3d& r_stepSize, const Matrix3d& R = Matrix3d::ZRotationMatrix3d(0))
	{
		double x,y,z;

		for (x=r_min[0]; x<=r_max[0]; x=x+r_stepSize[0])
		{
			for(y=r_min[1]; y<=r_max[1]; y=y+r_stepSize[1])
			{
				for(z=r_min[2]; z<=r_max[2]; z=z+r_stepSize[2])
				{
					T* p1= new T(Vector3d(x,y,z),R);
					this->push_back(p1);
				}
			}
		}
	};

	void generate_uniformGridRandomRotation(const Vector3d& r_min, const Vector3d& r_max, const Vector3d& r_stepSize, const Vector3d& R_limit = Vector3d(180*DEG_TO_RAD, 180*DEG_TO_RAD, 180*DEG_TO_RAD))
	{
		double x,y,z;
		srand((unsigned int)time(NULL));
		for (x=r_min[0]; x<=r_max[0]; x=x+r_stepSize[0])
		{
			for(y=r_min[1]; y<=r_max[1]; y=y+r_stepSize[1])
			{
				for(z=r_min[2]; z<=r_max[2]; z=z+r_stepSize[2])
				{
					double ea=((double)rand()*2)/RAND_MAX -1;
					double eb=((double)rand()*2)/RAND_MAX -1;
					double ec=((double)rand()*2)/RAND_MAX -1;
					Matrix3d R=Matrix3d::ZRotationMatrix3d(ea*R_limit[0])*Matrix3d::YRotationMatrix3d(eb*R_limit[1])*Matrix3d::XRotationMatrix3d(ec*R_limit[2]);
					T* p1= new T(Vector3d(x,y,z),R);
					this->push_back(p1);
				}
			}
		}
	};

	void generate_randomPoses(int numberOfPoses, const Vector3d& r_min, const Vector3d& r_max, const Vector3d& R_limit)
	{
		double x,y,z;
		srand((unsigned int)time(NULL));
		for(int i = 0; i < numberOfPoses; i++)
		{
			double ex=((double)rand())/RAND_MAX;
			double ey=((double)rand())/RAND_MAX;
			double ez=((double)rand())/RAND_MAX;
			double ea=((double)rand()*2)/RAND_MAX -1;
			double eb=((double)rand()*2)/RAND_MAX -1;
			double ec=((double)rand()*2)/RAND_MAX -1;
			x = (r_max[0]-r_min[0])*ex + r_min[0];
			y = (r_max[1]-r_min[1])*ey + r_min[1];
			z = (r_max[2]-r_min[2])*ez + r_min[2];
			Matrix3d R=Matrix3d::ZRotationMatrix3d(ea*R_limit[0])*Matrix3d::YRotationMatrix3d(eb*R_limit[1])*Matrix3d::XRotationMatrix3d(ec*R_limit[2]);
			T* p1= new T(Vector3d(x,y,z),R);
			this->push_back(p1);
		}
	}


	//! delete all poses of the pose list
	void deleteAllPoses()
	{
		// free dynamic allocated memory and delete poses
		while (!this->empty())
		{
			deleteLastPose();
		}
	}

	//! delete last pose
	void deleteLastPose()
	{
		// free memory and delete pose
		delete this->back();
		this->pop_back();
	}
	
	//! delete a pose defined by id (zero based)
	//! \todo This might be no good idea. Pose lists are "lists" not vectors, therefore
	//!      accessing by index is very inefficient. If index access is important, a 
	//!      poseList must be internally implented using the std::vector<> rather then std::list<>
	bool deletePose(const unsigned int& id)
	{
		typename T::const_iterator it=this->begin();
		std::advance(it, id);
		delete it;
		unsigned int count = 0;

		if (id < 0) return false;

		for (typename T::const_iterator it=this->begin();it!=this->end();it++)
		{
			if (count < id)
			{
				delete *it;
				this.erase(it);
				count++;
			}
			else
			{
				return true;
			}
		}
	}

	//! export the poselist in a specified format to a text file
	void SavePoseListToFile(const Eigen::IOFormat IOFormat_ = IOCSV, const string& filename = "Poselist.csv")
	{
		ofstream out(filename);
//		out.open (filename);
		for (typename std::list<T*>::const_iterator it=this->begin();it!=this->end();it++)
		{
			(*it)->getString(out,IOFormat_);
			out << endl;
		}
	}

	//! import the poselist from a text file in a specified format (matlab & csv so far)
	//! deletes all poses previously stored in the poselist
	//! \todo: The specialized function does not make sence with a template class
	//!   The implementation is specific to a certain template parameter and will
	//!   cause errors when instantiated for any other template.

	void LoadPoseListFromFile(const string& filename, const int& format)
	{
		double phi;
		Vector3d r, u;
		Matrix3d R;

		this->deleteAllPoses();

		ifstream in(filename);
//		in.open (filename);
		switch (format)	
		{
			case MATLAB: // xyz + axis angles [Matlab Command: save sim_result_x.csv x_  -ascii -tabs ]
				while (in >> r.x() >> r.y() >> r.z() >> u.x() >> u.y() >> u.z() >> phi)
				{
					CPoseKinetostatic* pose_tmp= new CPoseKinetostatic(r, phi, u);
					this->push_back(pose_tmp);
				}
				break;
			case CSV:
				char sep;
				
				if(in){
					string line;
					while (getline(in, line)) { // read line by line, that way poselists with AND without [v, omega, a, alpha, f, tau] can be read
						Vector3d v(0,0,0), omega(0,0,0), a(0,0,0), alpha(0,0,0), f(0,0,0), tau(0,0,0);
						stringstream seperator(line);
						seperator >> r(0) >> sep >> r(1) >> sep >> r(2) >> sep
						  >> R(0,0) >> sep >> R(0,1) >> sep >> R(0,2) >> sep 
						  >> R(1,0) >> sep >> R(1,1) >> sep >> R(1,2) >> sep 
						  >> R(2,0) >> sep >> R(2,1) >> sep >> R(2,2) >> sep
						  >> v(0) >> sep >> v(1) >> sep >> v(2) >> sep
						  >> omega(0) >> sep >> omega(1) >> sep >> omega(2) >> sep
						  >> a(0) >> sep >> a(1) >> sep >> a(2) >> sep
						  >> alpha(0) >> sep >> alpha(1) >> sep >> alpha(2) >> sep
						  >> f(0) >> sep >> f(1) >> sep >> f(2) >> sep
						  >> tau(0) >> sep >> tau(1) >> sep >> tau(2);
						CPoseKinetostatic* pose_tmp= new CPoseKinetostatic(r, R, v, omega, a, alpha, f, tau);
						this->push_back(pose_tmp);
					}
				}
				break;
		}
	}

};

class CPoseListStatic : public CPoseList<CPoseStatic>{};

class CPoseListKinetostatic : public CPoseList<CPoseKinetostatic>
{
public:
	CPoseListKinetostatic(): CPoseList<CPoseKinetostatic>()
	{
	}

	//! Set all parameters to the same value
	void setAll(const Vector3d& v, const Vector3d& a, const Vector3d& omega,const Vector3d& alpha, const Vector3d& f, const Vector3d& tau)
	{
		CPoseListKinetostatic::const_iterator it=this->begin(); 

		for (it;it!=this->end();it++)
		{
			CPoseKinetostatic* pp = *it;
			pp->v = v;
			pp->a = a;
			pp->omega = omega;
			pp->alpha = alpha;
			pp->f = f;
			pp->tau = tau;
		}
	}
};

} // end of namespace PCRL

