/*
* WireX  -  WireLibPyBindings
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

/*!********************************
 *  \file   : IkinBindings.cpp
 *  \Author   Philipp Miermeister
 *  \Date     25.02.2014
 *	\brief	  This file contains all python bindings related to robot kinematics
 *********************************
 */ 

#include "WireLibPyBindings.h"


static PyObject* WireRobot_getInitalObjectiveFunctionError(PyObject *self, PyObject *args)
{
	return Py_BuildValue("d", pyHelperKinematics()->getInitalObjectiveFunctionError());
}

static PyObject* WireRobot_getFinalObjectiveFunctionError(PyObject *self, PyObject *args)
{
	return Py_BuildValue("d", pyHelperKinematics()->getFinalObjectiveFunctionError());
}

static PyObject* WireRobot_getFinalJacobianNorm(PyObject *self, PyObject *args)
{
	return Py_BuildValue("d", pyHelperKinematics()->getFinalJacobianNorm());
}

static PyObject* WireRobot_getDeltaPose(PyObject *self, PyObject *args)
{
	return Py_BuildValue("d", pyHelperKinematics()->getDeltaPose());
}

static PyObject* WireRobot_getIterations(PyObject *self, PyObject *args)
{
	return Py_BuildValue("d", pyHelperKinematics()->getIterations());
}

static PyObject* WireRobot_getTerminationCode(PyObject *self, PyObject *args)
{
	return Py_BuildValue("d", pyHelperKinematics()->getTerminationCode());
}

static PyObject* WireRobot_getFunctionEvals(PyObject *self, PyObject *args)
{
	return Py_BuildValue("d", pyHelperKinematics()->getFunctionEvals());
}

static PyObject* WireRobot_getJacobianEvals(PyObject *self, PyObject *args)
{
	return Py_BuildValue("d", pyHelperKinematics()->getJacobianEvals());
}

/* set robot kinematics model */
static PyObject* WireRobot_SetKinematicsModel(PyObject *self, PyObject *args)
{
	int kinType;

	if ( ! PyArg_ParseTuple(args, "i",&kinType) )
        return NULL;

	pyHelperRobot()->setRobotKinematicsModel((PCRL::CRobotData::RobotKinematicsType)kinType);

    Py_RETURN_NONE;
}

/* set elasticity model */
static PyObject* WireRobot_SetElasticityModel(PyObject *self, PyObject *args)
{
	int elType;

	if ( ! PyArg_ParseTuple(args, "i",&elType) )
        return NULL;

	pyHelperRobot()->setElasticityModel((PCRL::CRobotData::ElasticityModelType)elType);

    Py_RETURN_NONE;
}

/* set solver algorithm type */
static PyObject* WireRobot_SetSolverAlgorithm(PyObject *self, PyObject *args)
{
	int solverType;

	if ( ! PyArg_ParseTuple(args, "i",&solverType) )
        return NULL;

	pyHelperKinematics()->setSolverAlgorithm((PCRL::emForwardAlgorithm::Type)solverType);

    Py_RETURN_NONE;
}


/* set the specific stiffness coefficient of the cables  */
static PyObject* WireRobot_setStiffnessCoefficient(PyObject *self, PyObject *args)
{
	double k;

	if ( ! PyArg_ParseTuple(args, "d",&k) )
        return NULL;

	pyHelperStiffness()->setStiffnessCoefficient(k);
    
    Py_RETURN_NONE;
}

/* set the additional cable length  */
static PyObject* WireRobot_setAdditionalCableLength(PyObject *self, PyObject *args)
{
	int i;
	double length;
	
    if ( ! PyArg_ParseTuple(args, "id",&i,&length) )
        return NULL;

	pyHelperStiffness()->setAdditionalCableLength(i,length);

    Py_RETURN_NONE;
}


/* print the resulting cable stiffness for the actual cable length */
static PyObject* WireRobot_printWireStiffness(PyObject *self, PyObject *args)
{
	pyHelperStiffness()->printWireStiffness(getRobotDocument()->r,getRobotDocument()->R);

    Py_RETURN_NONE;
}

/* compute the forward kinematics for the given cable length and 
 * return the pose as list r,R [l1, ..., lm] */
static PyObject* WireRobot_forwardKinematics(PyObject *self, PyObject *args)
{
	// count the number of passed elements
	Py_ssize_t size = PyTuple_Size(args);
	if (size==1) // perhaps we received a list instead of m single parameters
	{
		if ( ! PyList_CheckExact(PyTuple_GetItem(args,0)))
            Py_RETURN_FALSE;

        args = PyList_AsTuple(PyTuple_GetItem(args,0));
		if (PyTuple_Size(args)!=getRobotDocument()->getNow())
			Py_RETURN_FALSE;
		else
			size = PyTuple_Size(args);
	}
	else
		if ( size != pyHelperRobot()->getNow() )
			Py_RETURN_FALSE;

	// we try to extract the cable length vector
	MatrixXd L(pyHelperRobot()->getNow(), 1);
	for ( int i = 0; i < size; i++ )
	{
		PyObject* val = PyTuple_GetItem(args,i);
		L(i) = PyFloat_AsDouble(val);
	}

	Vector3d r;
	Matrix3d R;
	bool res = pyHelperKinematics()->doForwardKinematics(L, r, R);
//	cout << "pose R, r = " <<R<< endl;
//	cout <<"\n"<< r<<endl;
	MatrixXd pose(3,4);
	for (int i=0; i<3; i++)
	{
		pose(i,3)=r(i);
		for (int j=0; j<3; j++)
			pose(i,j)=R(i,j);
	}

    if ( res )
        return pyToList(pose);

    Py_RETURN_FALSE;
}


static PyObject* WireRobot_forwardKinematicsPulley(PyObject *self, PyObject *args)
{
	// count the number of passed elements
	Py_ssize_t size = PyTuple_Size(args);
	if (size==1) // perhaps we received a list instead of m single parameters
	{
		if ( ! PyList_CheckExact(PyTuple_GetItem(args,0)))
            Py_RETURN_FALSE;

        args = PyList_AsTuple(PyTuple_GetItem(args,0));
		if (PyTuple_Size(args)!=getRobotDocument()->getNow())
			Py_RETURN_FALSE;
		else
			size = PyTuple_Size(args);
	}
	else
		if ( size != pyHelperRobot()->getNow() )
			Py_RETURN_FALSE;

	// we try to extract the cable length vector
	MatrixXd L(pyHelperRobot()->getNow(), 1);
	for ( int i = 0; i < size; i++ )
	{
		PyObject* val = PyTuple_GetItem(args,i);
		L(i) = PyFloat_AsDouble(val);
	}

	Vector3d r;
	Matrix3d R;
	bool res = pyHelperKinematics()->doForwardKinematicsPulley(L, r, R);
//	cout << "pose R, r = " <<R<< endl;
//	cout <<"\n"<< r<<endl;
	MatrixXd pose(3,4);
	for (int i=0; i<3; i++)
	{
		pose(i,3)=r(i);
		for (int j=0; j<3; j++)
			pose(i,j)=R(i,j);
	}

    if ( res )
        return pyToList(pose);

    Py_RETURN_FALSE;
}

//! \todo We may want to factor out the argment conversion from this function and the above function 
//!       into a helper function (to avoid repetion of identical code)
static PyObject* WireRobot_getPotentialEnergy(PyObject *self, PyObject *args)
{
	// count the number of passed elements
	Py_ssize_t size = PyTuple_Size(args);
	if (size==1) // perhaps we received a list instead of m single parameters
	{
		if ( ! PyList_CheckExact(PyTuple_GetItem(args,0)))
            Py_RETURN_FALSE;

        args = PyList_AsTuple(PyTuple_GetItem(args,0));
		if (PyTuple_Size(args)!=getRobotDocument()->getNow())
			Py_RETURN_FALSE;
		else
			size = PyTuple_Size(args);
	}
	else
		if ( size != pyHelperRobot()->getNow() )
			Py_RETURN_FALSE;

	// we try to extract the cable length vector
	MatrixXd L(pyHelperRobot()->getNow(), 1);
	for ( int i = 0; i < size; i++ )
	{
		PyObject* val = PyTuple_GetItem(args,i);
		L(i) = PyFloat_AsDouble(val);
	}

	double res = pyHelperKinematics()->getPotentialEnergy(getRobotDocument()->r,getRobotDocument()->R, L);

	return Py_BuildValue("d",res);
}

static PyObject* WireRobot_inverseKinematics2(PyObject *self, PyObject *args)
{
	MatrixXd l(getRobotDocument()->getNow(),1);

	pyHelperKinematics()->doInverseKinematics(getRobotDocument()->r,getRobotDocument()->R,l);

	return pyToList(l.data(),getRobotDocument()->getNow());
}


static PyObject* WireRobot_inverseKinematicsPulley(PyObject *self, PyObject *args)
{
	MatrixXd l(getRobotDocument()->getNow(),1);

	pyHelperKinematics()->doInverseKinematicsPulley(getRobotDocument()->r,getRobotDocument()->R,l);

	return pyToList(l.data(),getRobotDocument()->getNow());
}

/* 
This function calculates the forward kinematic elasto geometrical for a given pose (r, rot_xyz) and offset in cable lengths. 
This allows to investigate the FK e.g. when a cable is shortened by a certain length l_off

*/
static PyObject* WireRobot_forwardKinematicsElastoGeometrical(PyObject *self, PyObject *args)
{
	Vector3d r, rot_xyz;
	Matrix3d R = Matrix3d::ZRotationMatrix3d(0);
		
	MatrixXd l (1,pyHelperRobot()->getNow()); // cable length from IK for the given pose
	MatrixXd l_off (1,pyHelperRobot()->getNow()); // cable offsets 
	l_off.setZero();

	double *pl_off = l_off.data();
	
	if ( ! PyArg_ParseTuple(args, "dddddddddddddd|dddddd",&r(0), &r(1), &r(2),&rot_xyz(0), &rot_xyz(1), &rot_xyz(2),&l_off(0),&l_off(1),&l_off(2),&l_off(3),&l_off(4),&l_off(5),&l_off(6),&l_off(7)) )
       return NULL;

	//&r(0), &r(1), &r(2),  &rot_xyz(0), &rot_xyz(1), &rot_xyz(2),&l_off(1),&l_off(2),&l_off(3),&l_off(4),&l_off(5),&l_off(6),&l_off(7)
	// use platform pose from robot data model in order to compute the geometrical ideal cable length
	getRobotDocument()->r = r;
	PCRL::getMatrixFromXYZ(R, rot_xyz);
	getRobotDocument()->R = R;
	getRobotDocument()->doInverseKinematics();
	// add user defined offsets to cable length and store them in the robot model (the forward kinematics uses this values)
	getRobotDocument()->l += l_off;
	
	// run forward kinematics
	if (getRobotDocument()->doElastoGeometricalForwardKinematics() )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* */
static PyObject* WireRobot_setPosition(PyObject *self, PyObject *args)
{
	double x,y,z;

	if ( ! PyArg_ParseTuple(args, "ddd",&x,&y,&z) )
        return NULL;

	getRobotDocument()->r=Vector3d(x,y,z);

    Py_RETURN_NONE;
}

static PyObject* WireRobot_setRotationR(PyObject *self, PyObject *args)
{
	int id = 0;
	Matrix3d R=Matrix3d::ZRotationMatrix3d(0.0);
	
	if ( ! PyArg_ParseTuple(args, "ddddddddd", &R(0,0), &R(1,0), &R(2,0), &R(0,1), &R(1,1), &R(2,1), &R(0,2), &R(1,2), &R(2,2)) )
        return NULL;

	getRobotDocument()->R = R;

    Py_RETURN_NONE;
}

static PyObject* WireRobot_setRotationXYZ(PyObject *self, PyObject *args)
{
	int id = 0;
	double x,y,z;
	Matrix3d R=Matrix3d::ZRotationMatrix3d(0.0);
	
	if ( ! PyArg_ParseTuple(args, "ddd", &x, &y, &z) )
        return NULL;

	PCRL::getMatrixFromXYZ(R,Vector3d(z,-y,x)); // warning getMatrixFromXYZ seems to be defined by convention z-yx
	getRobotDocument()->R = R;

    Py_RETURN_NONE; 
}

static PyObject* WireRobot_setRotationAxisAngle(PyObject *self, PyObject *args)
{
	int id = 0;
	double ux,uy,uz;
	double beta;
	Matrix3d R=Matrix3d::ZRotationMatrix3d(0.0);
	
	if ( ! PyArg_ParseTuple(args, "dddd", &ux, &uy, &uz, &beta) )
        return NULL;
    
    PCRL::getMatrixFromAxisAngle(R,beta,ux,uy,uz);
	getRobotDocument()->R = R;

    Py_RETURN_NONE;
}

static PyObject* WireRobot_setRotationQuaternion(PyObject *self, PyObject *args)
{
	int id = 0;
	double q0,q1,q2,q3;
	Matrix3d R=Matrix3d::ZRotationMatrix3d(0.0);
	
	if ( ! PyArg_ParseTuple(args, "dddd", &q0, &q1, &q2, &q3) )
        return NULL;

	PCRL::getMatrixFromQuaternion(R,q0,q1,q2,q3);	
	getRobotDocument()->R = R;

    Py_RETURN_NONE; 
}

static PyObject* WireRobot_getPosition(PyObject *self, PyObject *args)
{
	return pyToList(getRobotDocument()->r);
}

static PyObject* WireRobot_getOrientation(PyObject *self, PyObject *args)
{
	return pyToList(getRobotDocument()->R);
}

/* determine the cable length for the i-th cable*/
static PyObject* WireRobot_inverseKinematics(PyObject *self, PyObject *args)
{
	Vector3d r;
	Matrix3d R = Matrix3d::ZRotationMatrix3d(0);
	int i;

	if ( ! PyArg_ParseTuple(args, "iddd",&i,&r[0],&r[1],&r[2]) )
        return NULL;

	MatrixXd l(pyHelperRobot()->getNow(), 1);
	if ( pyHelperKinematics()->doInverseKinematics(r,R,l) )
        Py_RETURN_FALSE;

	double ret = l(i);

	return Py_BuildValue("d", ret);
}



static PyObject* WireRobot_doPoseEstimate(PyObject *self, PyObject *args)
{
	PCRL::emPoseEstimate::Type i;
	if ( ! PyArg_ParseTuple(args, "i",&i))
        return NULL;
	Vector3d pose;
	Vector3d r = getRobotDocument()->r;
	Matrix3d R = getRobotDocument()->R;
	MatrixXd l(pyHelperRobot()->getNow(),1);
	if ( !pyHelperKinematics()->doInverseKinematics(r,R,l) )
        Py_RETURN_FALSE;
	MatrixXd ub(6, 1), lb(6, 1);
	MatrixXd p(pyHelperRobot()->getDof(),1);
	 	
	pyHelperKinematics()->poseestimateMain(l, p, ub, lb, r, R, i);
	//cout<<r<<endl;
	for (int i=0;i<3;i++)
		pose(i)=p(i);

	return Py_BuildValue("ddd", pose(0),pose(1),pose(2));
}


static PyObject* WireRobot_BoundingBoxArea(PyObject *self, PyObject *args)
{
	Vector3d pose;
	Vector3d r = getRobotDocument()->r;
	Matrix3d R = getRobotDocument()->R;
	MatrixXd l(pyHelperRobot()->getNow(),1);
	if ( !pyHelperKinematics()->doInverseKinematics(r,R,l) )
        Py_RETURN_FALSE;
	MatrixXd ub(6, 1), lb(6, 1);
	double Area;
	MatrixXd p(pyHelperRobot()->getDof(), 1);

	pyHelperKinematics()->poseestimate(l, p, ub, lb);
	Area=abs(ub(0)-lb(0))*abs(ub(1)-lb(1))*abs(ub(2)-lb(2));

	for (int i=0;i<3;i++)
		pose(i)=p(i);

	return Py_BuildValue("d", Area);
}

// return platform vectors in world coordinates
static PyObject* WireRobot_getPlatformTransformed(PyObject *self, PyObject *args) 
{ 
	int i;

	if ( ! PyArg_ParseTuple(args, "i", &i) )
		return NULL;

	Vector3d b = pyHelperRobot()->getPlatform(i);
	Vector3d r = getRobotDocument()->r;
	Matrix3d R = getRobotDocument()->R;
	Vector3d b0 = r + R*b;

	return pyToList(b0); 
}


/* */
static PyObject* WireRobot_getWireRangeForBoxDriver(PyObject *self, PyObject *args)
{
	Vector3d Min,Max;
	double dMax=0;

	if ( ! PyArg_ParseTuple(args, "dddddd",&Min[0],&Min[1],&Min[2],&Max[0],&Max[1],&Max[2]) )
        return NULL;

	if ( ! pyHelperKinematics()->getWireRangeForBoxDriver(Min,Max,&dMax) )
        Py_RETURN_FALSE;

	return Py_BuildValue("d", dMax);
}

/* */
static PyObject* WireRobot_getWinchAperture(PyObject *self, PyObject *args)
{
	Vector3d Min,Max,u(0,0,0);
	int i;

	if ( ! PyArg_ParseTuple(args, "iddddddddd",&i,&Min[0],&Min[1],&Min[2],&Max[0],&Max[1],&Max[2],&u[0],&u[1],&u[2]) )
        return NULL;

	return Py_BuildValue("d", pyHelperHull()->getWinchAperture(i,Min,Max,u));
}

/* */
static PyObject* WireRobot_getStiffnessMatrix(PyObject *self, PyObject *args)
{
	Vector3d r;
	Matrix3d R=Matrix3d::ZRotationMatrix3d(0.0);

	if ( ! PyArg_ParseTuple(args, "ddd",&r[0],&r[1],&r[2]) )
        return NULL;

	MatrixXd f_wires=MatrixXd::Ones(8,1);
	if ( ! pyHelperStiffness()->GeometricalStiffnessMatrix(r,R,f_wires) )
       Py_RETURN_FALSE;

    if ( pyHelperStiffness()->StiffnessMatrix(r, R) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;

	//return Py_BuildValue("i", pyHelperStiffness()->StiffnessMatrix(r,R)?1:0);
}

/* */
static PyObject* WireRobot_getStiffnessMatrixMatrix(PyObject *self, PyObject *args)
{
	Vector3d r;
	Matrix3d R=Matrix3d::ZRotationMatrix3d(0.0);

	if ( ! PyArg_ParseTuple(args, "ddd",&r[0],&r[1],&r[2]) )
        return NULL;

	MatrixXd f_wires=MatrixXd::Ones(8,1);
	if ( ! pyHelperStiffness()->StiffnessMatrix(r,R) )
        Py_RETURN_FALSE;

	MatrixXd C;
	C = pyHelperStiffness()->getStiffnessMatrix();

	// now, return the matrix
	return pyToList(C);
}

/* */
static PyObject* WireRobot_getStiffnessMatrixMain(PyObject *self, PyObject *args)
{
	Vector3d r;
	MatrixXd f_wires=MatrixXd::Ones(8,1);
	Matrix3d R=Matrix3d::ZRotationMatrix3d(0.0);

	if ( ! PyArg_ParseTuple(args, "ddddddddddd",&r[0],&r[1],&r[2],&f_wires(0),&f_wires(1),&f_wires(2),&f_wires(3),&f_wires(4),&f_wires(5),&f_wires(6),&f_wires(7)) )
        return NULL;

    if ( pyHelperStiffness()->StiffnessMatrixMain(r, R, f_wires) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;

	//return Py_BuildValue("i", pyHelperStiffness()->StiffnessMatrixMain(r,R,f_wires)?1:0);
}

/* */
static PyObject* WireRobot_getStructureMatrix(PyObject *self, PyObject *args)
{
	Vector3d r;
	Matrix3d R=Matrix3d::ZRotationMatrix3d(0.0);

	if ( ! PyArg_ParseTuple(args, "ddd",&r[0],&r[1],&r[2]) )
        return NULL;

    if ( pyHelperFD()->getStructureMatrix(r,R) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;

	//return Py_BuildValue("i", pyHelperFD()->getStructureMatrix(r,R)?1:0);
}

static PyObject* WireRobot_isStructureMatrixFullRank(PyObject *self, PyObject *args)
{
	return Py_BuildValue("i", pyHelperFD()->isFullRank());
}

/* get the maximum norm for all columns of the structure matrix */
static PyObject* WireRobot_getStructureMatrixMaxColumnNorm(PyObject *self, PyObject *args)
{
	return Py_BuildValue("d",pyHelperFD()->getMaxColumnNorm());
}

/* get the maximum norm for all rows of the structure matrix [void] */
static PyObject* WireRobot_getStructureMatrixMaxRowNorm(PyObject *self, PyObject *args)
{
	return Py_BuildValue("d",pyHelperFD()->getMaxRowNorm());
}

/* get the singular values of the structure matrix [void]*/
static PyObject* WireRobot_getStructureMatrixSingularValues(PyObject *self, PyObject *args)
{
	MatrixXd sv;
	if ( pyHelperFD()->getSingularValues(sv) )
        return pyToList(sv);

	Py_RETURN_FALSE;
}

/* */
static PyObject* WireRobot_getStructureMatrixCoef(PyObject *self, PyObject *args)
{
	int i,j;

	if ( ! PyArg_ParseTuple(args, "ii",&i,&j) )
        return NULL;

	return Py_BuildValue("d", pyHelperFD()->getElement(i,j));
}

/* return the pre-computed structure matrix as a nested list */
static PyObject* WireRobot_getStructureMatrixMatrix (PyObject *self, PyObject *args)
{
	MatrixXd AT;

	pyHelperFD()->getMatrix(AT);

	return pyToList(AT);
}

/* */
static PyObject* WireRobot_StiffnessEllipsoid(PyObject *self, PyObject *args)
{
	Vector3d r;
	Matrix3d R=Matrix3d::ZRotationMatrix3d(0.0);
	Matrix3d K;

	if ( ! PyArg_ParseTuple(args, "ddd",&r[0],&r[1],&r[2]) )
        return NULL;

	if ( ! pyHelperStiffness()->StiffnessMatrixTranslational(r,R) )
        Py_RETURN_FALSE;

	pyHelperStiffness()->getStiffnessMatrixTranslational(K);
	Matrix3d eigenvectors;
	Vector3d eigenvalues;
	PCRL::getEigenvectorBase(K, eigenvectors, eigenvalues); // get the orientation and size of the ellipsoid
	eigenvalues.normalize(); // size of the ellipsoid

	// todo: add separate routine for visualization

	// cast from Eigen3 to GLGeometry
	/*CMatrix3 axis;
	for (int i=0;i<3;i++)
		for (int j=0;j<3;j++)
			axis(i+1,j+1)=eigenvectors(i,j);
	CVector3 radius, position;
	for (int i=0;i<3;i++)
	{
		radius(i+1)=eigenvalues(i);
		position(i+1)=r(i);
	}
	CFrame frame(position,axis);
	// create the ellipsoid
	double dAlpha=0.5;
	
	//CWireCenterView::This->m_Scene.createEllipsoid(frame,radius ,CVector3(0,1,1),dAlpha);*/

	Py_RETURN_TRUE;
}

/* */
static PyObject* WireRobot_getStiffnessMatrixTranslational(PyObject *self, PyObject *args)
{
	Vector3d r;
	Matrix3d R=Matrix3d::ZRotationMatrix3d(0.0);

	if ( ! PyArg_ParseTuple(args, "ddd",&r[0],&r[1],&r[2]) )
        return NULL;

    if ( pyHelperStiffness()->StiffnessMatrixTranslational(r,R) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;

	//return Py_BuildValue("i", pyHelperStiffness()->StiffnessMatrixTranslational(r,R)?1:0);
}

/* */
static PyObject* WireRobot_getMinimalStiffness(PyObject *self, PyObject *args)
{
	int i=0; // default is K_c

	if ( ! PyArg_ParseTuple(args, "|i",&i) )
        return NULL;

    return Py_BuildValue("d", pyHelperStiffness()->getMinimalStiffness(i));
}

/* */
static PyObject* WireRobot_setRequiredStiffness(PyObject *self, PyObject *args)
{
	double stiffness=0;
	if(!PyArg_ParseTuple(args, "d",&stiffness))
        return NULL;

	pyHelperStiffness()->setRequiredStiffness(stiffness);

	Py_RETURN_TRUE;
}

/* */
static PyObject* WireRobot_getRequiredStiffness(PyObject *self, PyObject *args)
{
	return Py_BuildValue("d", pyHelperStiffness()->getRequiredStiffness());
}

/* */
static PyObject* WireRobot_setInterferenceOrientationWorkspace(PyObject *self, PyObject *args)
{
	double alpha_min,alpha_max,delta_alpha,	beta_min,beta_max,delta_beta;

	if ( ! PyArg_ParseTuple(args, "dddddd",&alpha_min,&alpha_max,&delta_alpha,&beta_min,&beta_max,&delta_beta) )
        return NULL;

	pyHelperInterference()->setOrientationWorkspace(alpha_min,alpha_max,delta_alpha,beta_min,beta_max,delta_beta);

	Py_RETURN_NONE;
}

/* */
static PyObject* WireRobot_setInterferenceClippingBox(PyObject *self, PyObject *args)
{
	double xmin, xmax, ymin, ymax, zmin, zmax;

	if ( ! PyArg_ParseTuple(args, "dddddd",&xmin, &xmax, &ymin, &ymax, &zmin, &zmax) )
        return NULL;

	pyHelperInterference()->setClippingBox(xmin, xmax, ymin, ymax, zmin, zmax);
	
    Py_RETURN_NONE;
}

/* calculate cable-cable collisions of a pose */
static PyObject* WireRobot_calculateCableCableCollisions(PyObject *self, PyObject *args)
{
	double x, y, z, a=0, b=0, c=0, i=0;
	bool joints;
	if(!PyArg_ParseTuple(args, "ddd|dddd", &x, &y, &z, &a, &b, &c, &i))
        return NULL;
	
	if (i==1) joints=true;
	else joints=false;

	Matrix3d R = Matrix3d::ZRotationMatrix3d(0.0);
	PCRL::getMatrixFromXYZ(R,Vector3d(c,-b,a));

	if (pyHelperInterference()->calculateCableCableCollisions(Vector3d(x, y, z),R,joints))
		Py_RETURN_TRUE;
	Py_RETURN_FALSE;
}

/* return cable-cable collisions of a pose */
static PyObject* WireRobot_getCableCableCollisions(PyObject *self, PyObject *args)
{	
	MatrixXd collisions;

	pyHelperInterference()->getCableCableCollisions(collisions);

	return pyToList(collisions);
}

/* calculate cable-cable collisions inside a box */
static PyObject* WireRobot_calculateCableCableBoxCollisions(PyObject *self, PyObject *args)
{
	double minx, miny, minz, maxx, maxy, maxz, a=0, b=0, c=0;
	if(!PyArg_ParseTuple(args, "dddddd|ddd",&minx, &miny, &minz, &maxx, &maxy, &maxz, &a, &b, &c))
        return NULL;

	if (pyHelperInterference()->calculateCableCableBoxCollisions(Vector3d(minx, miny, minz),Vector3d(maxx, maxy, maxz),Vector3d(a,b,c)))
		Py_RETURN_TRUE;

	Py_RETURN_FALSE;
}

/* return cable-cable collisions inside a box */
static PyObject* WireRobot_getCableCableBoxCollisions(PyObject *self, PyObject *args)
{	
	MatrixXd collisions;

	pyHelperInterference()->getCableCableBoxCollisions(collisions);

	return pyToList(collisions);
}

/* */
static PyObject* WireRobot_printZeroLength(PyObject *self, PyObject *args)
{
	pyHelperKinematics()->printZeroLength();

	Py_RETURN_NONE;
}


static PyObject* WireRobot_HelpIkin(PyObject *self, PyObject *args);

PyMethodDef EmbMethodsIkin[] = {
	{"help", WireRobot_HelpIkin, METH_NOARGS,  "Print the names of all methods in Ikin [void]"},
    {"setAdditionalCableLength", WireRobot_setAdditionalCableLength, METH_VARARGS,  "set the additional cable length between drum and Ai for stiffness calculation [cable nr, additional length in m]"},
    {"setElasticityModel", WireRobot_SetElasticityModel, METH_VARARGS, "Set elasticity model [0: NONELASTIC, 1: LIN_ELASTIC, 2: SAGGING]"},
    {"setInterferenceClippingBox", WireRobot_setInterferenceClippingBox, METH_VARARGS,  "set the clipping planes for the intersection region [xmin, xmax, ymin, ymax, zmin, zmax]"},
    {"setInterferenceOrientationWorkspace", WireRobot_setInterferenceOrientationWorkspace, METH_VARARGS,  "set the search interface for intersection testing [Alpha_min, Alpha_max, Delta_alpha, Delta_min, Beta_max, Delta_beta]"},
    {"setKinematicsModel", WireRobot_SetKinematicsModel, METH_VARARGS, "Set kinematics model [0: FIXED, 1: PULLEY]"},
    {"setPosition", WireRobot_setPosition, METH_VARARGS,  "set the current position of the robot [VECTOR (3 doubles)]"},
    {"setRotationAxisAngle", WireRobot_setRotationAxisAngle, METH_VARARGS,  "set the current rotation of the robot [double ux, double uy, double uz, double beta]"},
    {"setRotationQuaternion", WireRobot_setRotationQuaternion, METH_VARARGS,  "set the current rotation of the robot [VECTOR (double q1, double q2, double q3, double q4)]"},
    {"setRotationR", WireRobot_setRotationR, METH_VARARGS,  "set the current rotation of the robot (column-major) [(9 doubles)]"},
    {"setRotationXYZ", WireRobot_setRotationXYZ, METH_VARARGS,  "set the current rotation of the robot [double x double y double z]"},
    {"setStiffnessCoefficient", WireRobot_setStiffnessCoefficient, METH_VARARGS,  "set the specific spring constant in N [k]"},
	{"setSolverAlgorithm",WireRobot_SetSolverAlgorithm, METH_VARARGS, "Set the type of solver algorithm for forward kinematics [int] types are 0:levmar_dif, 1:levmar_der, 2:levmar_difBounded, 3:levmar_derBounded, 4:gauss_newton" },
    
    {"getMinimalStiffness", WireRobot_getMinimalStiffness, METH_VARARGS,  "return the minimum eigenvalue of the stiffness matrix [integer] K_c(i=1), K_g(i=2), K_total(i=3)"},
    {"getOrientation", WireRobot_getOrientation, METH_NOARGS,  "get the current orientation of the mobile platform of the robot as nested list [void]"},
    {"getPosition", WireRobot_getPosition, METH_NOARGS,  "get the current position of the mobile platform of the robot as list [void]"},
    {"getStiffnessMatrix", WireRobot_getStiffnessMatrix, METH_VARARGS,  "Calculate the stiffness matrix at the pose r, R and store it internally [VECTOR: a, b, c (3 doubles)]"},
    {"getStiffnessMatrixMain", WireRobot_getStiffnessMatrixMain, METH_VARARGS,  "Calculate the stiffness matrix at the pose r, R [VECTOR: a, b, c (3 doubles), cable forces (8 doubles)]"},
    {"getStiffnessMatrixMatrix", WireRobot_getStiffnessMatrixMatrix, METH_VARARGS,  "Calculate the stiffness matrix at the pose r, R and return it as nested list [VECTOR: a, b, c (3 doubles)]"},
    {"getStiffnessMatrixTranslational", WireRobot_getStiffnessMatrixTranslational, METH_VARARGS,  "Calculate the translational stiffness matrix at the pose r, R [VECTOR: a, b, c (3 doubles)]"},
    {"setRequiredStiffness", WireRobot_setRequiredStiffness, METH_VARARGS,  "set the minimal required robot stiffness [k]"},
    {"getRequiredStiffness", WireRobot_getRequiredStiffness, METH_NOARGS,  "return the minimal required robot stiffness"},
    {"getStructureMatrix", WireRobot_getStructureMatrix, METH_VARARGS,  "Calculate the structure matrix at the pose r, R and store it internally for later use (e.g. for getting coefficients) [VECTOR: a, b, c (3 doubles)]"},
    {"getStructureMatrixCoef", WireRobot_getStructureMatrixCoef, METH_VARARGS,  "return a coefficient of the previously calculated structure matrix [i, j (2 ints)]"},
    {"getStructureMatrixMatrix", WireRobot_getStructureMatrixMatrix, METH_NOARGS,  "return the the pre-computed structure matrix AT as a nested list [void]"},
    {"getStructureMatrixMaxColumnNorm", WireRobot_getStructureMatrixMaxColumnNorm, METH_VARARGS,  "get the maximum norm for all columns of the structure matrix [void]"},
    {"getStructureMatrixMaxRowNorm", WireRobot_getStructureMatrixMaxRowNorm, METH_VARARGS,  "get the maximum norm for all rows of the structure matrix [void]"},
    {"getStructureMatrixSingularValues", WireRobot_getStructureMatrixSingularValues, METH_VARARGS,  "get the singular values of the structure matrix [void]"},
    {"getWireRangeForBoxDriver", WireRobot_getWireRangeForBoxDriver, METH_VARARGS,  "Calculate the minimum and maximum cable length required for a box [VECTOR.Min (3 doubles), VECTOR.Max (3 doubles)]"},
    {"getWinchAperture", WireRobot_getWinchAperture, METH_VARARGS,  "get the aperture for the i-th winch, a given workspace (min,max) and a given vector u [VECTOR.Min (3 doubles), VECTOR.Max (3 doubles), u(0,0,0)]"},

    {"getPotentialEnergy", WireRobot_getPotentialEnergy, METH_VARARGS,  "returnt the potential energy of the system for current pose (r,R) and given unstretched cable length l [l1, ..., lm]"},

	{"poseEstimate",WireRobot_doPoseEstimate,METH_VARARGS, "do pose estimation using current cable length [int choose Method]"},
	{"poseEstimateBoundingBoxArea",WireRobot_BoundingBoxArea,METH_NOARGS, ""},
    {"forwardKinematics", WireRobot_forwardKinematics, METH_VARARGS,  "compute the forward kinematics for the given cable length and return the pose as list r, R [l1, ..., lm]"},
	{"forwardKinematicsPulley", WireRobot_forwardKinematicsPulley, METH_VARARGS,  "compute the forward kinematics with pulley for the given cable length and return the pose as list r, R [l1, ..., lm]"},
	{"forwardKinematicsElasto", WireRobot_forwardKinematicsElastoGeometrical, METH_VARARGS,  "compute elasto geometrical forward kinematics for a given pose and length offset r (3 doubles [m]), R (3 doubles [rad]) | [l_off_1, ..., l_off_m]"},
	
    {"inverseKinematics", WireRobot_inverseKinematics, METH_VARARGS,  "return the length of the i-th cable for position x, y, z using the standard model [cable, VECTOR (3 doubles)]"},
    {"inverseKinematics2", WireRobot_inverseKinematics2, METH_NOARGS,  "return the length of all cables for the current position as list using the standard model [void]"},
    {"inverseKinematicsPulley", WireRobot_inverseKinematicsPulley, METH_VARARGS,  "return the length of all cables for the current position as list using the pulley model [void]"},
    
    {"levmarDeltaPose", WireRobot_getDeltaPose, METH_NOARGS, "get the norm of the last parameter improvement [void]"},
    {"levmarFinalObjFuncError", WireRobot_getFinalObjectiveFunctionError, METH_NOARGS, "get the final error of the objective function for found parameter vector [void]"},
    {"levmarFinalJacobianNorm", WireRobot_getFinalJacobianNorm, METH_NOARGS, "get the infinite norm of the Jacobian for the found parameter vector [void]"},
    {"levmarFunctionEvals", WireRobot_getFunctionEvals, METH_NOARGS, "number of evaluations of the objective function [void]"},
    {"levmarInitialObjFuncError", WireRobot_getInitalObjectiveFunctionError, METH_NOARGS, "get the initial error of the objective function for found parameter vector [void]"},
    {"levmarIterations", WireRobot_getIterations, METH_NOARGS, "get the number of iterations [void]"},
    {"levmarJacobianEvals", WireRobot_getJacobianEvals, METH_NOARGS, "number of evaluations of the jacobian [void]"},
    {"levmarTerminationCode", WireRobot_getTerminationCode, METH_NOARGS, "get reason for termination: [1] stopped by small gradient J^T e [2] stopped by small Dp [3] stopped by itmax [4] singular matrix. Restart from current p with increased mu [5] no further error reduction is possible. Restart with increased mu [6] stopped by small ||e||_2 [7] stopped by invalid (i.e. NaN or Inf) func values. This is a user error"},
    
    {"printZeroLength", WireRobot_printZeroLength, METH_NOARGS,  "print the reference (zero) length of the robot for the home pose [void]"},
    
    {"calculateCableCableCollisions", WireRobot_calculateCableCableCollisions, METH_VARARGS,  "calculate cable-cable collisions of a pose [x, y, z, a, b, c]"},
	{"getCableCableCollisions", WireRobot_getCableCableCollisions, METH_NOARGS,  "get cable-cable collisions of a pose"},
	{"calculateCableCableBoxCollisions", WireRobot_calculateCableCableBoxCollisions, METH_VARARGS,  "calculate cable-cable collisions inside a box [xmin, ymin, zmin, xmax, ymax, zmax]"},
	{"getCableCableBoxCollisions", WireRobot_getCableCableBoxCollisions, METH_NOARGS,  "get cable-cable collisions inside a box"},

	{"StiffnessEllipsoid", WireRobot_StiffnessEllipsoid, METH_VARARGS,  "plot the ellipsoid of the translational stiffness matrix at the pose r, R [VECTOR: a, b, c (3 doubles)]"},
    
	{"getPlatformTransformed", WireRobot_getPlatformTransformed, METH_VARARGS,  "return the transformed platform vector b0 of the i-th cable"},
	{NULL, NULL, 0, NULL}
};

/* print the full method table of the Ikin object */
static PyObject* WireRobot_HelpIkin(PyObject *self, PyObject *args)
{
	for (PyMethodDef* ptr = EmbMethodsIkin; ptr->ml_name!=0; ptr++)
        PySys_WriteStdout("Ikin.%s: %s\n", ptr->ml_name, ptr->ml_doc);
	
	Py_RETURN_NONE;
}