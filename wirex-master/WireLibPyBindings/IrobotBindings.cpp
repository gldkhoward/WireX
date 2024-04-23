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
 *  \file   : IrobotBindings.cpp
 *  \Author   Philipp Miermeister
 *  \Date     25.02.2014
 *	\brief 
 *********************************
 */ 

#include "WireLibPyBindings.h"
#include "motionPlanning/Utilities.h"

/*CWorkspaceDifferentialHull* pyHelperHull() {
	PCRL::CRobotDocument* pRobotDoc;
	int now;
	int motionPatternType;
	now = 8; 
	motionPatternType = 32;
	//delete pRobotDoc;
	pRobotDoc = getRobotDocument(); 
	//pRobotDoc = new CRobotDocument(now,(CRobotDocument::MotionPatternType)motionPatternType);
	return &(pRobotDoc->WSHull); 
}*/

PYHELPER_BOOL_STRING ( WireRobot_Load, getRobotDocument()->loadXml )
PYHELPER_BOOL_STRING ( WireRobot_Save, getRobotDocument()->saveXmlAtlas )

/* store the robot geometry in an xml string */
static PyObject* WireRobot_GetXml(PyObject *self, PyObject *args)
{
	string str;
	if ( !getRobotDocument()->getXml(str) )
        Py_RETURN_NONE;

	return Py_BuildValue("s", str.c_str());
}

/* set the geoemtry of one leg's base*/
static PyObject* WireRobot_SetBase(PyObject *self, PyObject *args)
{
	int i;
	double x,y,z;
	
    if ( ! PyArg_ParseTuple(args, "iddd",&i,&x,&y,&z) )
        return NULL;

	pyHelperRobot()->setBase(i,Vector3d(x,y,z));
	
    Py_RETURN_NONE;
}

/* set the orientation of one leg's base*/
static PyObject* WireRobot_SetBaseOrientation(PyObject *self, PyObject *args)
{
	int i;
	double x,y,z;
	Matrix3d R_base;

	if ( ! PyArg_ParseTuple(args, "iddd",&i,&x,&y,&z) )
        return NULL;

	R_base=Matrix3d::ZRotationMatrix3d(z*DEG_TO_RAD)*Matrix3d::YRotationMatrix3d(y*DEG_TO_RAD)*Matrix3d::XRotationMatrix3d(x*DEG_TO_RAD);
	pyHelperRobot()->setBaseOrientation(i,R_base);

	Py_RETURN_NONE;
}

/* set the geometry of one leg's platform*/
static PyObject* WireRobot_SetPlatform(PyObject *self, PyObject *args)
{
	int i;
	double x,y,z;

	if ( ! PyArg_ParseTuple(args, "iddd",&i,&x,&y,&z) )
        return NULL;

	pyHelperRobot()->setPlatform(i,Vector3d(x,y,z));

	Py_RETURN_NONE;
}

/* Export Robot Geometry to CSV*/
static PyObject* WireRobot_ExportRobotGeometryCSV(PyObject *self, PyObject *args)
{
	char* filename;
	if ( ! PyArg_ParseTuple(args, "s",&filename) )
        return NULL;

    ofstream ofsFile(filename);

    try
    {
	    for (int i=0; i<pyHelperRobot()->getNow();i++)
	    {
		    Vector3d base = pyHelperRobot()->getBase(i);
		    Vector3d platform = pyHelperRobot()->getPlatform(i);
		    ofsFile << base.x() << "; " << base.y() << "; "  << base.z() << "; " << platform.x() << "; " << platform.y() << "; "  << platform.z() << ";"  <<  "\n";
            ofsFile.flush();
        }
    
	    ofsFile.close();

	    Py_RETURN_TRUE;
    }
    catch ( ... )
    {
        if ( ofsFile.is_open() )
            ofsFile.close();

        Py_RETURN_FALSE;
    }
}

/* set the motion pattern of the robot (number of cables, motion type) */
static PyObject* WireRobot_SetMotionPattern(PyObject *self, PyObject *args)
{
	int now,mp=5;
	PCRL::CRobotData::MotionPatternType MPT;
	if ( ! PyArg_ParseTuple(args, "i|i",&now,&mp) )
        return NULL;

	switch (mp)
	{
	    case 0: MPT=PCRL::CRobotData::MP1T; break;
	    case 1: MPT=PCRL::CRobotData::MP2T; break;
	    case 2: MPT=PCRL::CRobotData::MP3T; break;
	    case 3: MPT=PCRL::CRobotData::MP1R2T; break;
	    case 4: MPT=PCRL::CRobotData::MP2R3T; break;
	    default:
	    case 5: MPT=PCRL::CRobotData::MP3R3T; break;
	}
	pyHelperRobot()->setMotionPattern(now,MPT);

	Py_RETURN_NONE;
}

/* */
static PyObject* WireRobot_SetWireLength(PyObject *self, PyObject *args)
{
	double lmin,lmax;
	if ( ! PyArg_ParseTuple(args, "dd",&lmin,&lmax) )
        return NULL;

	pyHelperRobot()->setWireLength(lmin,lmax);

	Py_RETURN_NONE;
}

/* calculate the equivalent mass of the specified winch in robot data */
static PyObject* WireRobot_getWinch_EquivalentMass(PyObject *self, PyObject *args)
{
	return Py_BuildValue("d", getRobotDocument()->currentWinch->getWinch_EquivalentMass());
}


/* */
static PyObject* WireRobot_getBoundingBoxPlatform(PyObject *self, PyObject *args)
{
	Vector3d minbb,maxbb;

	pyHelperRobot()->getBoundingBoxPlatform(minbb,maxbb);

	return pyToList(minbb,maxbb);
}

/* */
static PyObject* WireRobot_getBoundingBoxBase(PyObject *self, PyObject *args)
{
	Vector3d minbb,maxbb;

	pyHelperRobot()->getBoundingBoxBase(minbb,maxbb);

	return pyToList(minbb,maxbb);
}

/* */
static PyObject* WireRobot_getBoundingBoxCenterPlatform(PyObject *self, PyObject *args)
{
	Vector3d center = pyHelperRobot()->getBoundingBoxCenterPlatform();

	return pyToList(center);
}

/* */
static PyObject* WireRobot_getBoundingBoxCenterBase(PyObject *self, PyObject *args)
{
	Vector3d center = pyHelperRobot()->getBoundingBoxCenterBase();

	return pyToList(center);
}

PYHELPER_VOID_VOID( WireRobot_PrintRobotState, getRobotDocument()->printState() )

/* get the time for a trajectory segment */
static PyObject* WireRobot_getTrajectoryTime(PyObject *self, PyObject *args)
{
	double s,v,a,r;

	if ( ! PyArg_ParseTuple(args, "dddd",&s,&v,&a,&r) )
        return NULL;

	double dt=PCRL::getTrajectoryTime(s,v,a,r);

	return Py_BuildValue("d", dt);
}

/* Calculate the intersection between a line segment and a triangle. [A,n_A,u,n_v,n_w] */
static PyObject* WireRobot_getIntersectionTriangleLine(PyObject *self, PyObject *args)
{
	double ax,ay,az,
		nax,nay,naz,
		ux,uy,uz,
		nvx,nvy,nvz,
		nwx,nwy,nwz;

	if ( ! PyArg_ParseTuple(args, "ddddddddddddddd",&ax,&ay,&az,&nax,&nay,&naz,&ux,&uy,&uz,&nvx,&nvy,&nvz,&nwx,&nwy,&nwz) )
        return NULL;

    if ( PCRL::getIntersectionTriangleLine(Vector3d(ax,ay,az),Vector3d(nax,nay,naz),Vector3d(ux,uy,uz),Vector3d(nvx,nvy,nvz),Vector3d(nwx,nwy,nwz)) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}


/* get a the force distribution as list and associated wrench */
static PyObject* WireRobot_getForceDistribution(PyObject *self, PyObject *args)
{
	Vector3d r;
    double R21, R22, R23, R31, R32, R33;
	R21=R22=R23=R31=R32=R33=1e308;
    double a, b, c;
	a=b=c=1e308;
    
    if ( ! PyArg_ParseTuple(args, "ddd|ddddddddd", &r.x(), &r.y(), &r.z(), &a, &b, &c, &R21, &R22, &R23, &R31, &R32, &R33) )
        return NULL;

    Matrix3d R;

	if ( a == 1e308 || b == 1e308 || c == 1e308)
    {
        R = Matrix3d::Identity();
    }
    else if (R21 == 1e308 || R22 == 1e308 || R23 == 1e308 || R31 == 1e308 || R32 == 1e308 || R33 == 1e308)
    {
        R = Matrix3d::ZRotationMatrix3d(c*DEG_TO_RAD)*Matrix3d::YRotationMatrix3d(b*DEG_TO_RAD)*Matrix3d::XRotationMatrix3d(a*DEG_TO_RAD);
    }
	else
    {
        R << a, b, c,
            R21, R22, R23,
            R31, R32, R33;
    }

    
    //Matrix3d R = ( a && b && c ? Matrix3d::ZRotationMatrix3d(c*DEG_TO_RAD)*Matrix3d::YRotationMatrix3d(b*DEG_TO_RAD)*Matrix3d::XRotationMatrix3d(a*DEG_TO_RAD) : Matrix3d::Identity() );
	MatrixXd w = MatrixXd::Constant(6,1,0);
	MatrixXd fWires(pyHelperRobot()->getNow(),1);
	pyHelperFD()->getStructureMatrix(r,R);
	pyHelperFD()->setWrenchVector(w,pyHelperFD()->f,pyHelperFD()->tau);
	MatrixXd f_min = MatrixXd::Constant(pyHelperRobot()->getNow(),1,pyHelperRobot()->fmin);		// the minimum force vector
	MatrixXd f_max = MatrixXd::Constant(pyHelperRobot()->getNow(),1,pyHelperRobot()->fmax);		// the maximum force vector


	if ( pyHelperFD()->getDistribution(w, f_min, f_max,fWires) )
	{
		PyObject* list_f = pyToList(fWires);
		PyObject* list_w = pyToList(w);
		PyObject* list_fmin = pyToList(f_min);
		PyObject* list_fmax= pyToList(f_max);

		return Py_BuildValue("O",list_f);
	}

    Py_RETURN_FALSE;
}

/* get a test list */
static PyObject* WireRobot_getDistributionWeightedSum(PyObject *self, PyObject *args)
{
	MatrixXd w = MatrixXd::Constant(6,1,0);

	if ( ! PyArg_ParseTuple(args, "|dddddd",&w(0),&w(1),&w(2),&w(3),&w(4),&w(5)) )
		return NULL;
	
	MatrixXd f(pyHelperRobot()->getNow(),1);

	if ( pyHelperFD()->getDistributionWeightedSum(w,f) )
		return pyToList(f);

	Py_RETURN_FALSE;
}

/* get a list of lists with the all vertices that span the solution set */
static PyObject* WireRobot_getDistributionVertices(PyObject *self, PyObject *args)
{
	MatrixXd w = MatrixXd::Constant(6,1,0);

	if ( ! PyArg_ParseTuple(args, "|dddddd",&w(0),&w(1),&w(2),&w(3),&w(4),&w(5)) )
		return NULL;
	
	MatrixXd f(pyHelperRobot()->getNow(),1);

	if ( pyHelperFD()->getDistributionAllVertices(w,f) )
	{
		PyObject* erg = PyList_New(0);
		for (int i=0; i<f.cols(); i++)
		{
			MatrixXd fi = f.col(i);
			PyList_Append(erg,pyToList(fi));
		}

		return erg;
	}

	Py_RETURN_FALSE;
}


/* set a value for the model parameter */
static PyObject* WireRobot_setModelParameter(PyObject *self, PyObject *args)
{
	char *model,*param;
	double value;

	if ( ! PyArg_ParseTuple(args, "ssd",&model,&param,&value) )
        return NULL;

    if ( getRobotDocument()->ParametricModels.setParameter(model, param, value) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

PYHELPER_BOOL_STRING( WireRobot_applyModel, getRobotDocument()->ParametricModels.applyModel )

/* */
static PyObject* WireRobot_PlatformRotation(PyObject *self, PyObject *args)
{
	double a,b,c;
	Matrix3d R;

	if ( ! PyArg_ParseTuple(args, "ddd",&a,&b,&c) )
        return NULL;

	R=Matrix3d::ZRotationMatrix3d(a*DEG_TO_RAD)*Matrix3d::YRotationMatrix3d(b*DEG_TO_RAD)*Matrix3d::XRotationMatrix3d(c*DEG_TO_RAD);
	pyHelperRobot()->rotatePlatformGeoemetry(R);
	
	Py_RETURN_NONE;
}

/* */
static PyObject* WireRobot_BaseRotation(PyObject *self, PyObject *args)
{
	double a,b,c;
	Matrix3d R;

	if ( ! PyArg_ParseTuple(args, "ddd",&a,&b,&c) )
        return NULL;

	R=Matrix3d::ZRotationMatrix3d(a*DEG_TO_RAD)*Matrix3d::YRotationMatrix3d(b*DEG_TO_RAD)*Matrix3d::XRotationMatrix3d(c*DEG_TO_RAD);
	pyHelperRobot()->rotateFrameGeoemetry(R);
	
	Py_RETURN_NONE;
}

/* */
static PyObject* WireRobot_BaseTranslation(PyObject *self, PyObject *args)
{
	double x,y,z;

	if ( ! PyArg_ParseTuple(args, "ddd",&x,&y,&z) )
        return NULL;

	pyHelperRobot()->translateFrameGeometry(Vector3d(x,y,z));

	Py_RETURN_NONE;
}

/* */
static PyObject* WireRobot_PlatformTranslation(PyObject *self, PyObject *args)
{
	double x,y,z;

	if ( ! PyArg_ParseTuple(args, "ddd",&x,&y,&z) )
        return NULL;

	pyHelperRobot()->translatePlatformGeometry(Vector3d(x,y,z));
	Py_RETURN_NONE;
}

static PyObject* WireRobot_GeometryTransform(PyObject *self, PyObject *args)
{
	double x,y,z,a,b,c;
	Matrix3d R;
	
	if ( ! PyArg_ParseTuple(args, "dddddd",&x,&y,&z,&a,&b,&c) )
        return NULL;
	R=Matrix3d::ZRotationMatrix3d(a*DEG_TO_RAD)*Matrix3d::YRotationMatrix3d(b*DEG_TO_RAD)*Matrix3d::XRotationMatrix3d(c*DEG_TO_RAD);
	pyHelperRobot()->rotateFrameGeoemetry(R);
	pyHelperRobot()->rotatePlatformGeoemetry(R);
	pyHelperRobot()->translateFrameGeometry(Vector3d(x,y,z));
	pyHelperRobot()->translatePlatformGeometry(Vector3d(x,y,z));
	Py_RETURN_NONE;
}

PYHELPER_BOOL_VOID ( WireRobot_resetCableWear, getRobotDocument()->CableWear.cablewear_resetCableWear() )

static PyObject* WireRobot_CableWearAnalyzePath(PyObject *self, PyObject *args)
{
	vector<int> test;
	test.resize(1);
	if ( ! getRobotDocument()->CableWear.cablewear_analyzePath(1,1.00, 1.20,&test) )
        Py_RETURN_FALSE; //testing analyzePath

	for(unsigned int i=0;i<test.size();i++)
	{
        PySys_WriteStdout("%d, ", test.at(i));
		//cout << test.at(i) <<", ";
	}

    Py_RETURN_NONE;
}

PYHELPER_BOOL_STRING( WireRobot_CableWearExport, getRobotDocument()->CableWear.cablewear_exportCableWear )

static PyObject* WireRobot_cw_setBendingPostions(PyObject *self, PyObject *args)
{
	int i;
	vector<double> v_bendpos;

	PyObject *obj;
	if ( ! PyArg_ParseTuple(args, "iO", &i, &obj) )
		return NULL;

	if ( PyList_Check(obj) && pyToVector(obj, &v_bendpos) )
	{
		if ( getRobotDocument()->CableWear.cablewear_setBendingPositions(i,v_bendpos) )
			Py_RETURN_TRUE;
	}

	Py_RETURN_FALSE;
}

static PyObject* WireRobot_cw_getBendingPostions(PyObject *self, PyObject *args)
{
	int i;

	if (!PyArg_ParseTuple(args, "i", &i))
		return NULL;

	return pyToList(getRobotDocument()->CableWear.cablewear_getBendingPositions(i));
}

static PyObject* WireRobot_cwFakeBendingPostions(PyObject *self, PyObject *args)
{
	if ( getRobotDocument()->CableWear.cablewear_initializeFictionalBendingPositions() )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

static PyObject* WireRobot_getNow(PyObject *self, PyObject *args)
{
	return Py_BuildValue("i",getRobotDocument()->getNow());
}

static PyObject* WireRobot_getDof(PyObject *self, PyObject *args)
{
	return Py_BuildValue("i",getRobotDocument()->getDof());
}

static PyObject* WireRobot_getBase(PyObject *self, PyObject *args) 
{ 
	int i;

	if ( ! PyArg_ParseTuple(args, "i", &i) )
		return NULL;

	Vector3d a = pyHelperRobot()->getBase(i);

	return pyToList(a); 
}

/* set the orientation of one leg's base*/
static PyObject* WireRobot_getBaseOrientation(PyObject *self, PyObject *args)
{
	int i;

	if ( ! PyArg_ParseTuple(args, "i", &i) )
		return NULL;

    Matrix3d R = pyHelperRobot()->getBaseOrientation(i).transpose();

    return pyToList(R);
}

static PyObject* WireRobot_getPlatform(PyObject *self, PyObject *args) 
{ 
	int i;

	if ( ! PyArg_ParseTuple(args, "i", &i) )
		return NULL;

	Vector3d b = pyHelperRobot()->getPlatform(i);

	return pyToList(b); 
}


static PyObject* WireRobot_printWinchParameter(PyObject *self, PyObject *args) 
{ 
	
	double gear_ratio;	//!< gear ratio (values >1 reduce the velocity of the cable)
	double r_drum;		//!< radius of the drum in the winch [m]
	double n_drum;		//!< maximum number of windings on the drum
	double l_drum;		//!< axial length of the drum [m]
	int spool_direction;
	double inertia_motor; // respective to the motor axis; has to include all rotational intertia of the winch

	// get winch data from pRobot
	gear_ratio = pyHelperRobot()->pWinch->gear_ratio;
	r_drum = pyHelperRobot()->pWinch->r_drum;
	n_drum = pyHelperRobot()->pWinch->n_drum;
	l_drum = pyHelperRobot()->pWinch->l_drum;
	inertia_motor = pyHelperRobot()->pWinch->inertia;


	cout << "gear ratio: " << gear_ratio <<" \n";
	cout << "r_drum: " << r_drum <<" \n";
	cout << "n_drum: " << n_drum <<" \n";
	cout << "l_drum: " << l_drum <<" \n";
	cout << "inertia_motor: " << inertia_motor <<" \n";

	Py_RETURN_NONE; 
}

static PyObject* WireRobot_setWinchParameter(PyObject *self, PyObject *args) 
{ 
	
	double gear_ratio;	//!< gear ratio (values >1 reduce the velocity of the cable)
	double r_drum;		//!< radius of the drum in the winch [m]
	double n_drum;		//!< maximum number of windings on the drum
	double l_drum;		//!< axial length of the drum [m]
	double inertia_motor; // respective to the motor axis; has to include all rotational intertia of the winch

	if ( ! PyArg_ParseTuple(args, "ddddd", &gear_ratio, &r_drum, &n_drum, &l_drum, &inertia_motor) )
			return NULL;

	// get winch data from pRobot
	pyHelperRobot()->pWinch->setMotor_GearRatio(gear_ratio);
	pyHelperRobot()->pWinch->setDrum_Radius(r_drum);
	pyHelperRobot()->pWinch->setDrum_MaxWindings(n_drum);
	pyHelperRobot()->pWinch->setDrum_Length(l_drum);
	pyHelperRobot()->pWinch->setMotor_Inertia(inertia_motor);

	cout << "gear ratio: " << pyHelperRobot()->pWinch->getMotor_GearRatio() <<" \n";
	cout << "r_drum: " << pyHelperRobot()->pWinch->getDrum_Radius() <<" \n";
	cout << "n_drum: " << pyHelperRobot()->pWinch->getDrum_MaxWindings() <<" \n";
	cout << "l_drum: " << pyHelperRobot()->pWinch->getDrum_Length() <<" \n";
	cout << "inertia_motor: " << pyHelperRobot()->pWinch->getMotor_Inertia() <<" \n";

	Py_RETURN_NONE; 
}

/* set the radius of the guiding pulley of the winch's kinematic*/
static PyObject* WireRobot_setPulleyRadius(PyObject *self, PyObject *args)
{
	double r;
	int i=-1;
	if(!PyArg_ParseTuple(args, "d|i",&r, &i))
        return NULL;

	if (i==-1) // if no winch nr is provided, set the pulley radius of all winches
		for (int ii=0;ii<pyHelperRobot()->getNow();ii++)
			pyHelperRobot()->setPulleyRadius(r,ii);
	else
		if (i>=0 && i< pyHelperRobot()->getNow())
			pyHelperRobot()->setPulleyRadius(r,i);
		else
			cout << "index out of range \n";

	Py_RETURN_NONE;
}

static PyObject* WireRobot_getLeg(PyObject *self, PyObject *args)
{ 
	int i;

	if ( ! PyArg_ParseTuple(args, "i", &i) )
		return NULL;

	Vector3d a = pyHelperRobot()->getBase(i);
	Vector3d b = pyHelperRobot()->getPlatform(i);

	return pyToList(a,b); 
}

/* Set the mass of the platform */
static PyObject* WireRobot_SetPlatformMass(PyObject *self, PyObject *args)
{
	double mass;

	if( ! PyArg_ParseTuple(args, "d",&mass) )
        return NULL;

	pyHelperRobot()->platform_mass=mass;
	
    Py_RETURN_NONE;
}

/* Set the vector to the center of gravity*/
static PyObject* WireRobot_setPlatformCenterOfGravity(PyObject *self, PyObject *args)
{
	double x,y,z;

	if ( ! PyArg_ParseTuple(args, "ddd",&x,&y,&z) )
        return NULL;

	pyHelperRobot()->centerofgravity=Vector3d(x,y,z);

	Py_RETURN_NONE;
}

static PyObject* WireRobot_rotateRobot(PyObject *self, PyObject *args)
{
	Matrix3d R_t; 
	double a,b,c;

	if ( ! PyArg_ParseTuple(args, "ddd",&a,&b,&c) )
        return NULL;

	R_t = Matrix3d::ZRotationMatrix3d(a*DEG_TO_RAD)
			*Matrix3d::YRotationMatrix3d(b*DEG_TO_RAD)
			*Matrix3d::XRotationMatrix3d(c*DEG_TO_RAD);

	pyHelperRobot()->rotateFrameGeoemetry(R_t);
	pyHelperRobot()->rotatePlatformGeoemetry(R_t);
    Py_RETURN_TRUE;
}

static PyObject* WireRobot_getMetadata(PyObject *self, PyObject *args)
{
	return Py_BuildValue("(sss)",getRobotDocument()->name.c_str(), getRobotDocument()->desc.c_str(), getRobotDocument()->author.c_str());
}

PYHELPER_BOOL_STRING( WireRobot_loadAlgorithmConfiguration, getRobotDocument()->loadAlgorithmConfiguration )
PYHELPER_BOOL_STRING( WireRobot_saveAlgorithmConfiguration, getRobotDocument()->saveAlgorithmConfiguration )

//! the method table of all API functions exposed through the Irobot interface
PyMethodDef EmbMethodsIrobot[] = {
    //{"help", WireRobot_HelpIRobot, METH_NOARGS,  "Print the names of all methods in Irobot [void]"},
    
    {"getBase", WireRobot_getBase, METH_VARARGS,  "return a tuple (a_ix, a_iy, a_iz) with the geometry of the i-th base anchor point (vector a_i) [i]"},
    {"getBaseOrientation", WireRobot_getBaseOrientation, METH_VARARGS, "return a tuple of tuples with the orientation of the i-th base anchor point (matrix R_i) [i]"},
    {"getBoundingBoxBase", WireRobot_getBoundingBoxBase, METH_NOARGS,  "return a tuple (xmin, ymin, zmin, xmax, ymax, zmax) with the bounding box of the base frame [void]"},
    {"getBoundingBoxCenterPlatform", WireRobot_getBoundingBoxCenterPlatform, METH_NOARGS,  "return a tuple (x, y, z) with the center of the bounding box of the mobile platform [void]"},
    {"getBoundingBoxCenterBase", WireRobot_getBoundingBoxCenterBase, METH_NOARGS,  "return a tuple (x, y, z) with the center of the bounding box of the base frame [void]"},
    {"getBoundingBoxPlatform", WireRobot_getBoundingBoxPlatform, METH_NOARGS,  "return a tuple (xmin, ymin, zmin, xmax, ymax, zmax) with the bounding box of the mobile platform"},
    {"getDistributionWeightedSum", WireRobot_getDistributionWeightedSum, METH_VARARGS,  "Return the weighted sum force distribution computed for the wrench w. [fx, fy, fz, Mx, My, Mz]"},
    {"getDistributionVertices", WireRobot_getDistributionVertices, METH_VARARGS,  "Return a list of spanning vertices of the force distribution computed for the wrench w. [fx, fy, fz, Mx, My, Mz]"},
    {"getDof", WireRobot_getDof, METH_NOARGS,  "Return the degree-of-freedom (n) of the robot [void]"},
    {"getForceDistribution", WireRobot_getForceDistribution, METH_VARARGS,  "Get the force distribution for the given pose [x, y, z | a, b, c | R21, R22, R23, R31, R32, R33]"},
    {"getIntersectionTriangleLine", WireRobot_getIntersectionTriangleLine, METH_VARARGS,  "Calculate the intersection between a line segment and a triangle. [A, n_A, u, n_v, n_w]"},
    {"getLeg", WireRobot_getLeg, METH_VARARGS,  "return a tuple (a_ix, a_iy, a_iz, b_ix, b_iy, b_iz) with the geometry of the i-th base and platform anchor point (vector b_i) [i]"},
    {"getNow", WireRobot_getNow, METH_NOARGS,  "Return the number of wires (m) of the robot [void]"},
    {"getPlatform", WireRobot_getPlatform, METH_VARARGS,  "return a tuple (b_ix, b_iy, b_iz) with the geometry of the i-th platform anchor point (vector b_i) [i]"},
    {"getTrajectoryTime", WireRobot_getTrajectoryTime, METH_VARARGS,  "calculate the time taken to move along a trajectory with length s under the limitations of maximum velocity, acceleration and jerk [s, v, a, r]"},
    {"getWinch_EquivalentMass", WireRobot_getWinch_EquivalentMass, METH_NOARGS,  "return the equivalent mass of the rotational parts of the winch [void]"},
	{"rotateRobot",WireRobot_rotateRobot,METH_VARARGS, "rotate the robot by angles in degrees"},
    {"setBase", WireRobot_SetBase, METH_VARARGS,  "Set the geometry of one leg's base [BASE ID, VECTOR (3 doubles)]"},
    {"setBaseOrientation", WireRobot_SetBaseOrientation, METH_VARARGS,  "Set the orientation of one leg's base [BASE ID, VECTOR (3 doubles)]"},
    {"setModelParameter", WireRobot_setModelParameter, METH_VARARGS,  "set the value in a parametric geometry model [modelname, paramname, value]"},
    {"setMotionPattern", WireRobot_SetMotionPattern, METH_VARARGS,  "Set the degree-of-freedom and number of wires  [0:MP1T, 1:MP2T, 2:MP3T, 3:MP1R2T, 4:MP2R3T, 5:MP3R3T]"},
    {"setPlatform", WireRobot_SetPlatform, METH_VARARGS,  "Set the geometry of one leg's platform [PLATFORM ID, VECTOR (3 doubles)]"},
    {"setPlatformCenterOfGravity", WireRobot_setPlatformCenterOfGravity, METH_VARARGS,  "Set the vector the center of gravity [x, y, z]"},
    {"setPlatformMass", WireRobot_SetPlatformMass, METH_VARARGS,  "Set the mass of the platform [mass]"},
    {"setPulleyRadius", WireRobot_setPulleyRadius, METH_VARARGS,  "set the radius of the winch's guiding pulley [double radius]"},
    {"setWireLength", WireRobot_SetWireLength, METH_VARARGS,  "Set the minimum and maximum length for the cables [min, max]"},
    {"printWinchParameter", WireRobot_printWinchParameter, METH_NOARGS,  "prints the actual dataset of the winch database [void]"},
	{"setWinchParameter", WireRobot_setWinchParameter, METH_VARARGS,  "Set the dataset of the winch parameters: double gear_ratio, double r_drum [m], double n_drum, double l_drum [m], double inertia_motor [kg m²]"},

    {"applyModel", WireRobot_applyModel, METH_VARARGS,  "execute the setGeometry function of the model defined by its name [modelname]"},
	{"setGeometryTransformPlatformRotation", WireRobot_PlatformRotation, METH_VARARGS, "Transform Geometry by Rotating Platform by Angles [a, b, c]"},
	{"setGeometryTransformBaseRotation", WireRobot_BaseRotation, METH_VARARGS, "Transform Geometry by Rotating Base by Angles [a, b, c]"},
	{"setGeometryTransformBaseTranslation",WireRobot_BaseTranslation, METH_VARARGS, "Transform Geometry by Translating Base by [x, y, z]"},
	{"setGeometryTransformPlatformTranslation",WireRobot_PlatformTranslation, METH_VARARGS, "Transform Geometry by Translating Platform by [x, y, z]"},
	{"setGeometryTransform",WireRobot_GeometryTransform, METH_VARARGS, "Transform Geometry Numerically by Translating [x, y, z, a, b, c]"},
    
    //cable wear
    {"cwGetBendingPositions", WireRobot_cw_getBendingPostions, METH_VARARGS, "get cable wear bending positions for Winch(id)"},
    {"cwSetBendingPositions", WireRobot_cw_setBendingPostions, METH_VARARGS, "set cable wear bending positions for Winch(id), []"},
    {"cwSetFakeBendingPositions", WireRobot_cwFakeBendingPostions, METH_NOARGS, "set some cable wear bending positions for all Winches"},
    {"cwResetCableWearStatistics", WireRobot_resetCableWear, METH_NOARGS, "reset CableWear statistics [void]"},
    {"cwAnalyzeCableWear",WireRobot_CableWearAnalyzePath, METH_NOARGS, "analyze CableWear for path [void]"},
    {"cwExportCableWear", WireRobot_CableWearExport, METH_VARARGS, "export CableWear to file"},
    
    {"exportRobotGeometryCSV", WireRobot_ExportRobotGeometryCSV, METH_VARARGS,  "Save the robot geometry to csv [filename]"},
    
    {"load", WireRobot_Load, METH_VARARGS,  "Load a robot geometry file [filename]"},
    {"loadGeometry", WireRobot_Load, METH_VARARGS,  "Load a robot geometry file [filename]"},
	{"loadAlgorithmConfiguration", WireRobot_loadAlgorithmConfiguration, METH_VARARGS,  "load the algorithm settings from an XML file [filename]"},
    //{"loadCNCList", WireRobot_LoadCNCList, METH_VARARGS,  "Load a CNC list file with robot geometry [filename]"},
    
    {"printRobotState", WireRobot_PrintRobotState, METH_NOARGS,  "Print current robot state"},
    
    {"save", WireRobot_Save, METH_VARARGS,  "Save a robot geometry file [filename]"},
    {"getXml", WireRobot_GetXml, METH_NOARGS,  "Return the xml string of the robot [void]"},
    {"saveGeometry", WireRobot_Save, METH_VARARGS,  "Save a robot geometry file [filename]"},
	{"saveAlgorithmConfiguration", WireRobot_saveAlgorithmConfiguration, METH_VARARGS,  "save the algorithm settings in an XML file [filename]"},
    //{"saveXML", WireRobot_saveXML, METH_VARARGS,  "Save a robot geometry as XML file [filename]"},
    
	// meta information on the robot
	{"getMetadata", WireRobot_getMetadata, METH_NOARGS,  "return a tuple of strings with the robot's (name, description, author) [void]"},

	{NULL, NULL, 0, NULL}
};


/* print the full method table of the Iapp object */
static PyObject* WireRobot_HelpIRobot(PyObject *self, PyObject *args)
{
	for (PyMethodDef* ptr = EmbMethodsIrobot; ptr->ml_name!=0; ptr++)
        PySys_WriteStdout("Irobot.%s: %s\n",ptr->ml_name,ptr->ml_doc);
	
	Py_RETURN_NONE;
}
