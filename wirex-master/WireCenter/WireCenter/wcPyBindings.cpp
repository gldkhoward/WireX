/*
* WireX  -  WireCenter
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

// wcPyBindings.cpp: Implementierungsdatei
//

#include "StdAfx.h"
#include "PythonInterface.h"
#include "WireCenterDoc.h"
#include "WireCenterView.h"
#include "MainFrm.h"
#include "IPC.h"
#include "aes256.h"
#include <WireLibPyBindings\WireLibPyBindings.h>
#include "PersistantDefaultValue.h"

// return handle to robot document during runtime
PCRL::CRobotDocument* getRobotDocument() { return CWireCenterView::This->GetRobotDoc(); }

CWireCenterDoc* pyHelperDoc() { return CWireCenterView::This->GetDocument(); }
CWireCenterView* pyHelperView() { return CWireCenterView::This; }
CMainFrame* pyHelperMainFrm() { return (CMainFrame*)AfxGetMainWnd(); }


// implementation of the wrapper functions to be used in the method tables
//////////////////////////////////////////////////////////////////////////////

PYHELPER_BOOL_STRING( WireRobot_LoadCNCList, pyHelperDoc()->KalibrierungSteuerungsdateiimportieren )
PYHELPER_BOOL_STRING( WireRobot_loadXml, getRobotDocument()->loadXml )
PYHELPER_BOOL_STRING( WireRobot_loadProject, pyHelperDoc()->loadProject )
PYHELPER_BOOL_STRING( WireRobot_saveProject, pyHelperDoc()->saveProject )

/* load a wirecenter project from a file */
static PyObject* WireRobot_loadPersistantDefaults(PyObject *self, PyObject *args)
{
	char* filename = NULL;

	if ( ! PyArg_ParseTuple(args, "s",&filename) )
		return NULL;
	
	if (CPersistantDefaultValue::getInstance().readScheme(filename) &&
		CPersistantDefaultValue::getInstance().readXmlFile(filename))
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

PYHELPER_BOOL_STRING( WireRobot_savePersistantDefaults, CPersistantDefaultValue::getInstance().writeXmlFile )

static PyObject* WireRobot_generatePoseListRandom(PyObject *self, PyObject *args)
{

	int iNumberOfPoses=0;
	Vector3d r_min;
	Vector3d r_max;
	Vector3d R_limit;
	
	if ( ! PyArg_ParseTuple(args, "i|ddddddddd",&iNumberOfPoses, &r_min[0],&r_min[1],&r_min[2],&r_max[0],&r_max[1],&r_max[2],&R_limit[0],&R_limit[1],&R_limit[2]) )
        return NULL;

	CWireCenterView::This->createPoseListRandom(iNumberOfPoses, r_min, r_max, R_limit);

	Py_RETURN_NONE;
}

static PyObject* WireRobot_generatePoseListGrid(PyObject *self, PyObject *args)
{
	Vector3d r_min;
	Vector3d r_max;
	Vector3d r_stepsize;
	
	if ( ! PyArg_ParseTuple(args, "ddddddddd", &r_min[0],&r_min[1],&r_min[2],&r_max[0],&r_max[1],&r_max[2],&r_stepsize[0],&r_stepsize[1],&r_stepsize[2]) )
        return NULL;

	CWireCenterView::This->createPoseListGrid(r_min, r_max, r_stepsize);

	Py_RETURN_NONE;
}

static PyObject* WireRobot_generatePoseListGridRandomRot(PyObject *self, PyObject *args)
{
	Vector3d r_min;
	Vector3d r_max;
	Vector3d r_stepsize;
	Vector3d R_limit;
	
	if ( ! PyArg_ParseTuple(args, "dddddddddddd", &r_min[0],&r_min[1],&r_min[2],&r_max[0],&r_max[1],&r_max[2],&r_stepsize[0],&r_stepsize[1],&r_stepsize[2],&R_limit[0],&R_limit[1],&R_limit[2]) )
        return NULL;

	CWireCenterView::This->createPoseListGridRandomRot(r_min, r_max, r_stepsize,R_limit);

	Py_RETURN_NONE;
}

PYHELPER_VOID_VOID( WireRobot_deletePoseList, pyHelperView()->PoseList.deleteAllPoses() )

/* save a robot geometry as xml file */
static PyObject* WireRobot_saveXml(PyObject *self, PyObject *args)
{
	char* filename = NULL;
	if ( ! PyArg_ParseTuple(args, "|s",&filename) )
        return NULL;

	//choice between filename and no filename
	if ( filename == NULL )
		pyHelperDoc()->OnDateiSpeichernalsxml();
	else
    {
		if ( getRobotDocument()->saveXml(filename) )
            Py_RETURN_TRUE;
        
        Py_RETURN_FALSE;
    }

    Py_RETURN_NONE;
}

PYHELPER_BOOL_STRING( WireRobot_ncLoadNCProgram, CWireCenterView::This->LoadNcProgram )
PYHELPER_BOOL_STRING( WireRobot_ncLoadPoseEvaluatorSettings, CWireCenterView::This->LoadPoseEvaluatorSettings )
PYHELPER_BOOL_STRING( WireRobot_ncSavePoseEvaluatorSettings, CWireCenterView::This->SavePoseEvaluatorSettings )

/* */
static PyObject* WireRobot_ncExportPoselist(PyObject *self, PyObject *args)
{
	char *sFilename;
	if ( ! PyArg_ParseTuple(args, "s",&sFilename) )
        return NULL;

	CWireCenterView::This->PoseList.SavePoseListToFile(IOCSV,sFilename);

	Py_RETURN_NONE;
}

/* */
static PyObject* WireRobot_ncImportPoselist(PyObject *self, PyObject *args)
{
	char *sFilename;
	int iFormat=1;
	if ( ! PyArg_ParseTuple(args, "s|i",&sFilename, &iFormat) )
        return NULL;

	CWireCenterView::This->PoseList.LoadPoseListFromFile(sFilename, iFormat);

	Py_RETURN_NONE;
}

PYHELPER_VOID_STRING( WireRobot_TrajektorieAnalysieren, CWireCenterView::This->BerechnenTrajektorieanalysieren )

/* load a robot geometry */
static PyObject* WireRobot_Invalidate(PyObject *self, PyObject *args)
{
	CWireCenterView::This->UpdateStatistics();
	CWireCenterView::This->Invalidate();

	Py_RETURN_NONE;
}

PYHELPER_VOID_VOID( WireRobot_PrintGLParam, CWireCenterView::This->m_Scene.printParam() )

/* set the projection mode of open GL */
static PyObject* WireRobot_setGLProjection(PyObject *self, PyObject *args)
{
	bool bFlag;

	if ( ! PyArg_ParseTuple(args, "b",&bFlag) )
        return NULL;

	pyHelperView()->m_Scene.setPerspectiveProjection(bFlag);

	Py_RETURN_NONE;
}

/* change GL-Parameters*/
static PyObject* WireRobot_setGLParameters(PyObject *self, PyObject *args)
{
	double dZoom;
	if ( ! PyArg_ParseTuple(args, "d",&dZoom) )
        return NULL;

    pyHelperView()->m_Scene.adjustZoom(dZoom);
	
    Py_RETURN_NONE;
}

/* change GL-Parameters*/
static PyObject* WireRobot_setGLPosition(PyObject *self, PyObject *args)
{
	double x,y,z;
	if ( ! PyArg_ParseTuple(args, "ddd",&x,&y,&z) )
        return NULL;

	pyHelperView()->m_Scene.setViewPoint(CVector3(x,y,z));
	
    Py_RETURN_NONE;
}

/* change GL-Parameters*/
static PyObject* WireRobot_setGLOrientation(PyObject *self, PyObject *args)
{
	double q0,q1,q2,q3;
	if ( ! PyArg_ParseTuple(args, "dddd",&q0,&q1,&q2,&q3) )
        return NULL;

	pyHelperView()->m_Scene.setQuaternionScalarQ0(q0);
	pyHelperView()->m_Scene.setQuaternionImaginary(CVector3(q1,q2,q3));
	
    Py_RETURN_NONE;
}

/* set the background color */
static PyObject* WireRobot_setBackgroundColor(PyObject *self, PyObject *args)
{
	double LowerColor[4], UpperColor[4];
	for (int i=0;i<4;i++)
		LowerColor[i]= UpperColor[i]=-1;   // initialize the arrays

	if ( ! PyArg_ParseTuple(args, "dddd|dddd",&LowerColor[0],&LowerColor[1],&LowerColor[2],&LowerColor[3],&UpperColor[0],&UpperColor[1],&UpperColor[2],&UpperColor[3]) )
        return NULL;

	for (int i=0;i<4;i++)
		if (UpperColor[i]==-1) // if upper color is not parametrized, use the values of lower colors. 
			UpperColor[i]=LowerColor[i];

	pyHelperView()->m_Scene.setLowerBackgroundColor(LowerColor[0], LowerColor[1], LowerColor[2], LowerColor[3]);
	pyHelperView()->m_Scene.setUpperBackgroundColor(UpperColor[0], UpperColor[1], UpperColor[2], UpperColor[3]);

	Py_RETURN_NONE;
}

/* set the color of the workspace hull*/
static PyObject* WireRobot_setHullColor(PyObject *self, PyObject *args)
{
	double r,g,b;

	if( ! PyArg_ParseTuple(args, "ddd",&r,&g,&b) )
        return NULL;

	pyHelperView()->pHull->setHullColor(r,g,b);

	Py_RETURN_NONE;
}


/*  */
static PyObject* WireRobot_setWorkspaceVisibility(PyObject *self, PyObject *args)
{
	int isVisible;

	if ( ! PyArg_ParseTuple(args, "i",&isVisible) )
        return NULL;

	CWireCenterView::This->setWorkspaceVisibility(isVisible ? true : false);

	Py_RETURN_NONE;
}

/*  */
static PyObject* WireRobot_setWiresVisibility(PyObject *self, PyObject *args)
{
	int isVisible;

	if ( ! PyArg_ParseTuple(args, "i",&isVisible) )
        return NULL;

	CWireCenterView::This->setWiresVisibility(isVisible ? true : false);
	
    Py_RETURN_NONE;
}

/*  */
static PyObject* WireRobot_setPlatformVisibility(PyObject *self, PyObject *args)
{
	int isVisible;

	if ( ! PyArg_ParseTuple(args, "i",&isVisible) )
        return NULL;

	CWireCenterView::This->setPlatformVisibility(isVisible ? true : false);
	
    Py_RETURN_NONE;
}

/*  */
static PyObject* WireRobot_setPlanesVisibility(PyObject *self, PyObject *args)
{
	int isVisible;

	if ( ! PyArg_ParseTuple(args, "i",&isVisible) )
        return NULL;

	CWireCenterView::This->setPlanesVisibility(isVisible ? true : false);
	
    Py_RETURN_NONE;
}

/*  */
static PyObject* WireRobot_setInterferenceVisibility(PyObject *self, PyObject *args)
{
	int isVisible;

	if ( ! PyArg_ParseTuple(args, "i",&isVisible) )
        return NULL;

	CWireCenterView::This->setInterferenceVisibility(isVisible ? true : false);
	
    Py_RETURN_NONE;
}

/*  */
static PyObject* WireRobot_setPoseEstimateBoxVisibility(PyObject *self, PyObject *args)
{
	int isVisible;
	if(!PyArg_ParseTuple(args, "i",&isVisible)) return NULL;
	CWireCenterView::This->setPoseEstimateBoxVisibility(isVisible?true:false);
	Py_RETURN_TRUE;
}


/*  */
static PyObject* WireRobot_setPoseEstimateBoxSphereVisibility(PyObject *self, PyObject *args)
{
	int isVisible;
	if(!PyArg_ParseTuple(args, "i",&isVisible)) return NULL;
	CWireCenterView::This->setPosEstimateBoxSpheresVisibility(isVisible?true:false);
	Py_RETURN_TRUE;
}

/*  */
static PyObject* WireRobot_createBox(PyObject *self, PyObject *args)
{
	CVector3 Min,Max;
	double r = -1, g= -1, b = -1;
	bool wireframe=false;

	if ( ! PyArg_ParseTuple(args, "dddddd|dddb",&Min.x,&Min.y,&Min.z,&Max.x,&Max.y,&Max.z,&r,&g,&b,&wireframe) )
        return NULL;

	if (r < 0 || g < 0 || b < 0) 
	{
		CWireCenterView::This->m_Scene.createBox(Min,Max,NULL,wireframe);
	}
	else
	{
		CWireCenterView::This->m_Scene.createBox(Min,Max,r,g,b,NULL,wireframe);
	}

	Py_RETURN_NONE;
}

/*  */
static PyObject* WireRobot_createCone(PyObject *self, PyObject *args)
{
	CVector3 Apex, Axis;
	double Aperture;

	if ( ! PyArg_ParseTuple(args, "ddddddd",&Apex.x,&Apex.y,&Apex.z,&Axis.x,&Axis.y,&Axis.z,&Aperture) )
        return NULL;

	CWireCenterView::This->m_Scene.createCone(Apex,Axis,Aperture);
	
    Py_RETURN_NONE;
}

/*  */
static PyObject* WireRobot_createSTL(PyObject *self, PyObject *args)
{
	char* filename;
	double r_ = -1, g_= -1, b_ = -1, alpha_=-1;
	CVector3 r(0,0,0);
	CMatrix3 R(1,0,0,0,1,0,0,0,1);
	CMatrix3 Rtemp;
	double s=1;
	double angle[3];
	angle[0]=angle[1]=angle[2]=0;

	if ( ! PyArg_ParseTuple(args, "s|ddddddddddd",&filename,&r.x,&r.y,&r.z,&s,&angle[0],&angle[1], &angle[2],&r_,&g_,&b_,&alpha_) )
        return NULL;

	angle[0] = angle[0]/RAD_TO_DEG;
	angle[1] = angle[1]/RAD_TO_DEG;
	angle[2] = angle[2]/RAD_TO_DEG;

	mul(Rtemp,CMatrix3(1,0,0,0,cos(angle[0]),sin(angle[0]), 0,-sin(angle[0]),cos(angle[0])),
		CMatrix3(cos(angle[1]), 0,sin(angle[1]),0,1,0, -sin(angle[1]),0,cos(angle[1])));
		mul(R,Rtemp,
		CMatrix3(cos(angle[2]), sin(angle[2]),0, -sin(angle[2]),cos(angle[2]),0,0,0,1));

	if (r_ < 0 || g_ < 0 || b_ < 0 || alpha_ < 0) 
		CWireCenterView::This->m_Scene.createSTL(filename,r,R,s);
	else
		CWireCenterView::This->m_Scene.createSTL(filename,r,R,s, r_, g_, b_, alpha_);

	Py_RETURN_NONE;
}

/*  */
static PyObject* WireRobot_createVRML(PyObject *self, PyObject *args)
{
	char* filename;
	double r_ = -1, g_= -1, b_ = -1, alpha_=-1;
	CVector3 r(0,0,0);
	CMatrix3 R(1,0,0,0,1,0,0,0,1);
	CMatrix3 Rtemp;
	double s=1;
	double angle[3];
	angle[0]=angle[1]=angle[2]=0;

	if ( ! PyArg_ParseTuple(args, "s|ddddddddddd",&filename,&r.x,&r.y,&r.z,&s,&angle[0],&angle[1], &angle[2],&r_,&g_,&b_,&alpha_) )
        return NULL;
	
	angle[0] = angle[0]/RAD_TO_DEG;
	angle[1] = angle[1]/RAD_TO_DEG;
	angle[2] = angle[2]/RAD_TO_DEG;

	mul(Rtemp,CMatrix3(1,0,0,0,cos(angle[0]),sin(angle[0]), 0,-sin(angle[0]),cos(angle[0])),
		CMatrix3(cos(angle[1]), 0,sin(angle[1]),0,1,0, -sin(angle[1]),0,cos(angle[1])));
		mul(R,Rtemp,
		CMatrix3(cos(angle[2]), sin(angle[2]),0, -sin(angle[2]),cos(angle[2]),0,0,0,1));
	if (r_ < 0 || g_ < 0 || b_ < 0 || alpha_ < 0) 
		CWireCenterView::This->m_Scene.createVRML(filename,r,R,s);
	else
		CWireCenterView::This->m_Scene.createVRML(filename,r,R,s, r_, g_, b_, alpha_);

	Py_RETURN_NONE;
}

/* loads a winch model with correct coordinate system to the winch base frame */
static PyObject* WireRobot_createVRML_Winch(PyObject *self, PyObject *args)
{
	char* filename;
	double r_ = -1, g_= -1, b_ = -1, alpha_=-1;
	int i; // winch-nr
	CVector3 r(0,0,0);
	CMatrix3 R(1,0,0,0,1,0,0,0,1);
	CMatrix3 Rtemp;
	double s=1;
	double angle[3];
	angle[0]=angle[1]=angle[2]=0;

	if ( ! PyArg_ParseTuple(args, "si|ddddd",&filename,&i, &s,&r_,&g_,&b_,&alpha_) )
        return NULL;
	
	Matrix3d R_base = pyHelperRobot()->getBaseOrientation(i);
	Vector3d r_base = pyHelperRobot()->getBase(i);
		
	for (int i_=0;i_<3;i_++)
	{
		r(i_+1)=r_base(i_);
		for (int j_=0;j_<3;j_++)
			R(i_+1,j_+1)=R_base(i_,j_);	
	}

		
	if (r_ < 0 || g_ < 0 || b_ < 0 || alpha_ < 0) 
		CWireCenterView::This->m_Scene.createVRML(filename,r,R,s);
	else
		CWireCenterView::This->m_Scene.createVRML(filename,r,R,s, r_, g_, b_, alpha_);

	Py_RETURN_NONE;
}

/*  */
static PyObject* WireRobot_create3DS(PyObject *self, PyObject *args)
{
	char* filename;
	CVector3 r(0,0,0);
	CMatrix3 R(1,0,0,0,1,0,0,0,1);
	CMatrix3 Rtemp;
	double s=1;
	double angle[3];
	angle[0]=angle[1]=angle[2]=0;

	if ( ! PyArg_ParseTuple(args, "s|ddddddd",&filename,&r.x,&r.y,&r.z,&s,&angle[0],&angle[1], &angle[2]) )
        return NULL;
	
	angle[0]=angle[0]/RAD_TO_DEG;
	angle[1]=angle[1]/RAD_TO_DEG;
	angle[2]=angle[2]/RAD_TO_DEG;

	mul(Rtemp,CMatrix3(1,0,0,0,cos(angle[0]),sin(angle[0]), 0,-sin(angle[0]),cos(angle[0])),
		CMatrix3(cos(angle[1]), 0,sin(angle[1]),0,1,0, -sin(angle[1]),0,cos(angle[1])));
		mul(R,Rtemp,
		CMatrix3(cos(angle[2]), sin(angle[2]),0, -sin(angle[2]),cos(angle[2]),0,0,0,1));

	CWireCenterView::This->m_Scene.create3DS(filename,r,R,s);

	Py_RETURN_NONE;
}

PYHELPER_VOID_VOID(WireRobot_deleteInternalShapes, CWireCenterView::This->m_Scene.deleteLayer(0) )

/* call a generic getter function */
static PyObject* WireRobot_setAttribute(PyObject *self, PyObject *args)
{
	bool b;
	int i;
	double d;
	string s;
	char* name;

	if ( PyArg_ParseTuple(args, "sb",&name,&b) ) // bool
	{
		if ( CWireCenterView::This->set(name,b) )
			Py_RETURN_TRUE;
	}
	else if ( PyArg_ParseTuple(args, "si",&name,&i) ) // integer
	{
		if ( CWireCenterView::This->set(name,i) )
			Py_RETURN_TRUE;
	}
	else if ( PyArg_ParseTuple(args, "sd",&name,&d) ) // double
	{
		if ( CWireCenterView::This->set(name,d) )
			Py_RETURN_TRUE;
	}
	else if ( PyArg_ParseTuple(args, "ss",&name,&s) ) // string
	{
		if ( CWireCenterView::This->set(name,s) )
			Py_RETURN_TRUE;
	}

    PyErr_Format(PyExc_TypeError, "Attribute %s unknown\n", name);

	return NULL;
}

/* call a generic getter function */
static PyObject* WireRobot_selectDifferentialHull(PyObject *self, PyObject *args)
{
	int i;
	if ( ! PyArg_ParseTuple(args, "i",&i) )
        return NULL;

	CWireCenterView::This->pWDH->DifferenceSelector = i;

	Py_RETURN_NONE;
}

/* save a screenshot in bmp */
static PyObject* WireRobot_saveBmp(PyObject *self, PyObject *args)
{
	char *filename;
	int width=640,height=480;

	if ( ! PyArg_ParseTuple(args, "s|ii",&filename,&width,&height) )
        return NULL;

	CWireCenterView::This->m_Scene.SaveBmp(filename,width,height);

	Py_RETURN_NONE;
}

PYHELPER_VOID_STRING( WireRobot_saveEps, CWireCenterView::This->m_Scene.SaveEps )
PYHELPER_VOID_VOID( startServer, CWireCenterView::This->m_pIPC->startServer() )
PYHELPER_VOID_STRING( WireRobot_saveLog, pyHelperMainFrm()->log.save )

/* empty the content of the log */
static PyObject* WireRobot_clearLog(PyObject *self, PyObject *args)
{
	if ( pyHelperMainFrm()->log.clear() )
        Py_RETURN_TRUE;

	Py_RETURN_FALSE;
}

/* save Log to a file */
static PyObject* WireRobot_setShapeDefaultAttribute(PyObject *self, PyObject *args)
{
	TShapeAttribute SA;
	int visible=SA.bVisibile?1:0;
	pyHelperView()->m_Scene.getDefaultAttribute(SA);

	if ( ! PyArg_ParseTuple(args, "|iii",&SA.iFrame,&SA.iLayer,&visible) )
        return NULL;

	SA.bVisibile = ( visible == 0 ? false : true );
	pyHelperView()->m_Scene.setDefaultAttribute(SA);

	Py_RETURN_NONE;
}

/* create new frame with regards to a selected base frame and returns the ID of the new frame*/
static PyObject* WireRobot_createFrame(PyObject *self, PyObject *args)
{
	unsigned int baseID = 0;
	/*unsigned int ID = 0;*/

	if ( ! PyArg_ParseTuple(args, "i",&baseID) )
        return NULL;

	return Py_BuildValue("i", pyHelperView()->m_Scene.createFrame(baseID));
}

/* delete frame named by ID*/
static PyObject* WireRobot_deleteFrame(PyObject *self, PyObject *args)
{
	unsigned int ID = 0;

	if ( ! PyArg_ParseTuple(args, "i",&ID) )
        return NULL;

	if ( pyHelperView()->m_Scene.deleteFrame(ID) )
        Py_RETURN_TRUE;

	Py_RETURN_FALSE;
}

PYHELPER_VOID_VOID ( WireRobot_deleteAllFrames, pyHelperView()->m_Scene.deleteAllFrames() ) 

/* delete layer named by ID*/
static PyObject* WireRobot_deleteLayer(PyObject *self, PyObject *args)
{
	unsigned int ID = 0;

	if ( ! PyArg_ParseTuple(args, "i",&ID) )
        return NULL;

	pyHelperView()->m_Scene.deleteLayer(ID);

	Py_RETURN_NONE;
}

/* set layer attributes*/
static PyObject* WireRobot_setLayerAttributes(PyObject *self, PyObject *args)
{
	unsigned int ID = 0;
	bool isVisible = true;

	if ( ! PyArg_ParseTuple(args, "ib", &ID, &isVisible) )
        return NULL;

	pyHelperView()->m_Scene.setLayerAttributes(ID,isVisible);

    Py_RETURN_NONE;
}

/* get position of frame named by ID */
static PyObject* WireRobot_getFramePosition(PyObject *self, PyObject *args)
{
	int id = 0;	
	if ( ! PyArg_ParseTuple(args, "i",&id) )
        return NULL;

	CFrame* pFrame = pyHelperView()->m_Scene.getFrame(id);

	return Py_BuildValue("ddd",pFrame->r.x,pFrame->r.y,pFrame->r.z);
}

/* set frame position*/
static PyObject* WireRobot_setFramePosition(PyObject *self, PyObject *args)
{
	//int platformFrameID = pyHelperView()->hGlPlatform;
	int id = 0;
	double x,y,z;

	if ( ! PyArg_ParseTuple(args, "iddd",&id, &x, &y, &z) )
        return NULL;

	CFrame* pFrame = pyHelperView()->m_Scene.getFrame(id);
	pFrame->r.x = x;
	pFrame->r.y = y;
	pFrame->r.z = z;

	Py_RETURN_NONE;
}

/* set frame rotation using 3x3 rotation matrix (column-major definition)*/
static PyObject* WireRobot_setFrameRotationR(PyObject *self, PyObject *args)
{
	int id = 0;
	Matrix3d R=Matrix3d::ZRotationMatrix3d(0.0);
	
	if ( ! PyArg_ParseTuple(args, "iddddddddd",&id, &R(0,0), &R(1,0), &R(2,0), &R(0,1), &R(1,1), &R(2,1), &R(0,2), &R(1,2), &R(2,2)) )
        return NULL;

	CFrame* pFrame = pyHelperView()->m_Scene.getFrame(id);
	memcpy(&pFrame->R,&R,sizeof(double) * 9); // copy data from Matrix3d to CMatrix3

	Py_RETURN_NONE;
}

/* set frame rotation using XYZ description*/
static PyObject* WireRobot_setFrameRotationXYZ(PyObject *self, PyObject *args)
{
	int id = 0;
	double x,y,z;
	Matrix3d R=Matrix3d::ZRotationMatrix3d(0.0);
	
	if ( ! PyArg_ParseTuple(args, "iddd",&id, &x, &y, &z) )
        return NULL;

	PCRL::getMatrixFromXYZ(R,Vector3d(z,-y,x)); // warning getMatrixFromXYZ seems to be defined by convention z-yx
	CFrame* pFrame = pyHelperView()->m_Scene.getFrame(id);
	memcpy(&pFrame->R,&R,sizeof(double) * 9); // copy data from Matrix3d to CMatrix3

	Py_RETURN_NONE;
}

/* set frame rotation using axis angle description*/
static PyObject* WireRobot_setFrameRotationAxisAngle(PyObject *self, PyObject *args)
{
	int id = 0;
	double ux,uy,uz;
	double beta;
	Matrix3d R=Matrix3d::ZRotationMatrix3d(0.0);
	
	if ( ! PyArg_ParseTuple(args, "idddd",&id, &ux, &uy, &uz, &beta) )
        return NULL;

	PCRL::getMatrixFromAxisAngle(R,beta,ux,uy,uz);
	CFrame* pFrame = pyHelperView()->m_Scene.getFrame(id);
	memcpy(&pFrame->R,&R,sizeof(double) * 9); // copy data from Matrix3d to CMatrix3

	Py_RETURN_NONE;
}

/* set frame rotation using quaternion description*/
static PyObject* WireRobot_setFrameRotationQuaternion(PyObject *self, PyObject *args)
{
	int id = 0;
	double q0,q1,q2,q3;
	Matrix3d R=Matrix3d::ZRotationMatrix3d(0.0);
	
	if ( ! PyArg_ParseTuple(args, "idddd",&id, &q0, &q1, &q2, &q3) )
        return NULL;
    
    PCRL::getMatrixFromQuaternion(R,q0,q1,q2,q3);	
	CFrame* pFrame = pyHelperView()->m_Scene.getFrame(id);
	memcpy(&pFrame->R,&R,sizeof(double) * 9); // copy data from Matrix3d to CMatrix3

	Py_RETURN_NONE;
}

/* get frame rotation using 3x3 rotation matrix (column-major definition)*/
static PyObject* WireRobot_getFrameRotationR(PyObject *self, PyObject *args)
{
	int id = 0;
	Matrix3d R=Matrix3d::ZRotationMatrix3d(0.0);
	
    if ( ! PyArg_ParseTuple(args, "i",&id) )
        return NULL;
    
    CFrame* pFrame = pyHelperView()->m_Scene.getFrame(id);
	memcpy(&R,&pFrame->R,sizeof(double) * 9); // copy data from CMatrix3 to Matrix3d

	return Py_BuildValue("ddddddddd",R(0,0), R(1,0), R(2,0), R(0,1), R(1,1), R(2,1), R(0,2), R(1,2), R(2,2));
}

/* get frame rotation using XYZ description*/
static PyObject* WireRobot_getFrameRotationXYZ(PyObject *self, PyObject *args)
{
	int id = 0;
	double x,y,z;
	Matrix3d R=Matrix3d::ZRotationMatrix3d(0.0);

	if ( ! PyArg_ParseTuple(args, "i",&id) )
        return NULL;

	CFrame* pFrame = pyHelperView()->m_Scene.getFrame(id);
	memcpy(&R,&pFrame->R,sizeof(double) * 9); // copy data from CMatrix3 to Matrix3d
	PCRL::getXYZFromMatrix(z,y,x,R); // warning getXYZFromMatrix seems to be defined by convention z-yx

	return Py_BuildValue("ddd",x,-y,z);
}

/* get frame rotation using axis angle description*/
static PyObject* WireRobot_getFrameRotationAxisAngle(PyObject *self, PyObject *args)
{
	int id = 0;
	double ux,uy,uz;
	double beta;
	
    Matrix3d R=Matrix3d::ZRotationMatrix3d(0.0);

	if ( ! PyArg_ParseTuple(args, "i",&id) )
        return NULL;

	CFrame* pFrame = pyHelperView()->m_Scene.getFrame(id);
	memcpy(&R,&pFrame->R,sizeof(double) * 9); // copy data from CMatrix3 to Matrix3d
	PCRL::getAxisAngleFromMatrix(beta,ux,uy,uz,R);

	return Py_BuildValue("dddd",ux,uy,uz,beta);
}

/* get frame rotation using quaternion description*/
static PyObject* WireRobot_getFrameRotationQuaternion(PyObject *self, PyObject *args)
{
	int id = 0;
	double q0,q1,q2,q3;

	Matrix3d R=Matrix3d::ZRotationMatrix3d(0.0);

	if ( ! PyArg_ParseTuple(args, "i",&id) )
        return NULL;

	CFrame* pFrame = pyHelperView()->m_Scene.getFrame(id);
	memcpy(&R,&pFrame->R,sizeof(double) * 9); // copy data from CMatrix3 to Matrix3d
	PCRL::getQuaternionFromMatrix(q0,q1,q2,q3,R);

	return Py_BuildValue("dddd",q0,q1,q2,q3);
}

/* save Log to a file */
static PyObject* WireRobot_signPythonScript(PyObject *self, PyObject *args)
{
	char* source=0, *dest=0;

	if ( ! PyArg_ParseTuple(args, "s|s",&source,&dest) )
        return NULL;

	if ( dest == 0 )
		dest=source;

	if ( aes256_encryptFile(source,dest) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* save Log to a file */
static PyObject* WireRobot_unsignPythonScript(PyObject *self, PyObject *args)
{
	char* source=0, *dest=0;

	if ( ! PyArg_ParseTuple(args, "s|s",&source,&dest) )
        return NULL;

	if ( dest == 0 )
		dest=source;

	if ( aes256_decryptFile(source,dest) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* save Log to a file */
static PyObject* WireRobot_setSpanCableSelector(PyObject *self, PyObject *args)
{
	int cs=-1;

	if ( ! PyArg_ParseTuple(args, "|i",&cs) )
        return NULL;
	
	pyHelperView()->pWireSpan->cableSelector=cs;
	pyHelperView()->pWireSpanCone->cableSelector=cs;

    Py_RETURN_TRUE;
}

/* save Log to a file */
static PyObject* WireRobot_getPlatformFrame(PyObject *self, PyObject *args)
{
	return Py_BuildValue("i",pyHelperView()->hGlPlatform);
}

static PyObject* WireRobot_printBindingTable(PyObject *self, PyObject *args)
{
	if ( CWireCenterView::This->printBindingTable() )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

static PyObject* WireRobot_AppReqMaxWorkspace(PyObject *self, PyObject *args)
{
	Vector3d maxWS = pyHelperApp()->maxWS;

	return pyToList(maxWS);
}

static PyObject* WireRobot_AppReqMinWorkspace(PyObject *self, PyObject *args)
{
	Vector3d minWS = pyHelperApp()->minWS;

	return pyToList(minWS);
}

static PyObject* WireRobot_AppReqMaxInstallationSpace(PyObject *self, PyObject *args)
{
	Vector3d maxIS = pyHelperApp()->maxIS;

	return pyToList(maxIS);
}

static PyObject* WireRobot_AppReqMinInstallationSpace(PyObject *self, PyObject *args)
{
	Vector3d minIS = pyHelperApp()->minIS;

	return pyToList(minIS);
}

static PyObject* WireRobot_AppRequirements(PyObject *self, PyObject *args)
{
	double* Requirements = (double*)malloc(sizeof(double)*5);

	Requirements[0] = pyHelperApp()->velocity;
	Requirements[1] = pyHelperApp()->acceleration;
	Requirements[2] = pyHelperApp()->payload;
	Requirements[3] = pyHelperApp()->force;
	Requirements[4] = pyHelperApp()->torque;

	return pyToList(Requirements, 5);
}

static PyObject* WireRobot_setRoi(PyObject *self, PyObject *args)
{
	Vector3d lower,upper;
	if ( ! PyArg_ParseTuple(args, "dddddd", &lower.x(), &lower.y(), &lower.z(), &upper.x(), &upper.y(), &upper.z()) )
        return NULL;

	pyHelperDoc()->Roi.lower() = lower;
	pyHelperDoc()->Roi.upper() = upper;
	pyHelperDoc()->Roi.normalize();

	Py_RETURN_NONE;
}

static PyObject* WireRobot_getRoi(PyObject *self, PyObject *args)
{
	return pyToList(pyHelperDoc()->Roi.lower(),pyHelperDoc()->Roi.upper());
}

/**
 * Set the Region of Interest auto use flag to the provided value
 */
static PyObject* WireRobot_setRoiAutoUse(PyObject *self, PyObject *args)
{
	int iUse = 0;

	if ( ! PyArg_ParseTuple(args, "i", &iUse) )
        return NULL;

	pyHelperView()->m_bAutoRoiUse = ( iUse != 0 ? true : false );

    Py_RETURN_NONE;
}

/**
 * Set the Region of Interest visibility
 */
static PyObject* WireRobot_setRoiVisibility(PyObject *self, PyObject *args)
{
	int iVisible = 0;

	if ( ! PyArg_ParseTuple(args, "i", &iVisible) )
        return NULL;

	pyHelperView()->m_bRoiVisible = ( iVisible != 0 ? true : false );

    Py_RETURN_NONE;
}

/*  */
static PyObject* WireRobot_setColPosePropToVisualize(PyObject *self, PyObject *args)
{
    int iColumn=0;

	if ( ! PyArg_ParseTuple(args, "i", &iColumn) )
        return NULL;

	CWireCenterView::This->pPosePropertyEvaluation->setColumn(iColumn);

	Py_RETURN_NONE;
}

PYHELPER_VOID_VOID( WireRobot_BerechnenKinematik, pyHelperDoc()->OnBerechnenKinematik() )
PYHELPER_BOOL_STRING( WireRobot_SavePoseEvaluatorSettings, pyHelperView()->SavePoseEvaluatorSettings )
PYHELPER_BOOL_STRING( WireRobot_LoadPoseEvaluatorSettings, pyHelperView()->LoadPoseEvaluatorSettings )

static PyObject* WireRobot_HelpIApp(PyObject *self, PyObject *args);

// create the "method table" for the functions related to the application object (wire center)
static PyMethodDef EmbMethodsIapp[] = {
    {"help", WireRobot_HelpIApp, METH_NOARGS,  "Print the names of all methods in Iapp"},
    
    {"setAttribute", WireRobot_setAttribute, METH_VARARGS,  "generic setter function for attributes"},
    {"setFramePosition", WireRobot_setFramePosition, METH_VARARGS,  "set frame position (int id, double x, double y, double z)"},
    {"setFrameRotationAxisAngle", WireRobot_setFrameRotationAxisAngle, METH_VARARGS,  "set frame rotation using axis angle (int id, double ux, double uy, double uz, double beta)"},
    {"setFrameRotationQuaternion", WireRobot_setFrameRotationQuaternion, METH_VARARGS,  "set frame rotation using quaternion (int id, double q1, double q2, double q3, double q4)"},
    {"setFrameRotationR", WireRobot_setFrameRotationR, METH_VARARGS,  "set frame rotation using 3x3 matrix (column-major) (int id, double R(0,0), double R(1,0), ... )"},
    {"setFrameRotationXYZ", WireRobot_setFrameRotationXYZ, METH_VARARGS,  "set frame rotation using xyz convention (int id, double rot_x, double rot_y, double rot_z)"},
    {"setGLProjection", WireRobot_setGLProjection, METH_VARARGS,  "set the projection mode of open GL [0=orthogonal, 1=perspective]"},
    {"setInterferenceVisibility", WireRobot_setInterferenceVisibility, METH_VARARGS,  "set visibility of cable interference region [0:false int:true]"},
    {"setLayerAttributes", WireRobot_setLayerAttributes, METH_VARARGS,  "set attributes of specified layer (int id, bool isVisible)"},
    {"setPlanesVisibility", WireRobot_setPlanesVisibility, METH_VARARGS,  "set visibility for workspace hull [0:false int:true]"},
    {"setPlatformVisibility", WireRobot_setPlatformVisibility, METH_VARARGS,  "set visibility for workspace hull [0:false int:true]"},
    {"setRoi", WireRobot_setRoi, METH_VARARGS, "set the region of interest (ROI) to be used by a number of analysis and workspace functions [xmin, ymin, zmin, xmax, ymax, zmax]"},
    {"setRoiAutoUse", WireRobot_setRoiAutoUse, METH_VARARGS, "Set the auto use flag for the region of interest (ROI) [0:false, int:true]"},
    {"setRoiVisibility", WireRobot_setRoiVisibility, METH_VARARGS, "Set the visibility of the region of interest (ROI) [0:false, int:true]"},
    {"setShapeDefaultAttribute", WireRobot_setShapeDefaultAttribute, METH_VARARGS,  "set the default values for new shape objects (int Frame, int Layer, int visible)"},
    {"setWorkspaceVisibility", WireRobot_setWorkspaceVisibility, METH_VARARGS,  "set visibility for workspace hull [0:false int:true]"},
    {"setWiresVisibility", WireRobot_setWiresVisibility, METH_VARARGS,  "set visibility for workspace hull [0:false int:true]"},
    
    {"getFramePosition", WireRobot_getFramePosition, METH_VARARGS,  "get frame position of specified frame (int id) -> returns (double x, double y, double z)"},
    {"getFrameRotationAxisAngle", WireRobot_getFrameRotationAxisAngle, METH_VARARGS,  "get frame rotation using axis angle (int id) -> returns (double ux, double uy, double uz, double beta)"},
    {"getFrameRotationR", WireRobot_getFrameRotationR, METH_VARARGS,  "get frame rotation using 3x3 matrix (column-major) (int id) -> returns (double R(0,0), double R(1,0), ... )"},
    {"getFrameRotationQuaternion", WireRobot_getFrameRotationQuaternion, METH_VARARGS,  "get frame rotation using quaternion (int id) -> returns (double q1, double q2, double q3, double q4)"},
    {"getFrameRotationXYZ", WireRobot_getFrameRotationXYZ, METH_VARARGS,  "get frame rotation using xyz convention (int id) -> returns (double rot_x, double rot_y, double rot_z)"},
    {"getPlatformFrame", WireRobot_getPlatformFrame, METH_NOARGS,  "determine the GL handle of the platform frame"},
    {"getRoi", WireRobot_getRoi, METH_NOARGS, "get the region of interest (ROI) and return it as a list (xmin, ymin, zmin, xmax, ymax, zmax)"},
    
    {"clearLog", WireRobot_clearLog, METH_VARARGS,  "clear the console window"},
    {"createBox", WireRobot_createBox, METH_VARARGS,  "adds a box to the GL scene [VECTOR (3 doubles), VECTOR (3 doubles)]"},
    {"createCone", WireRobot_createCone, METH_VARARGS,  "adds a cone to the GL scene [Apex, Axis]"},
    {"createFrame", WireRobot_createFrame, METH_VARARGS,  "create new frame with regards to a selected base frame (int baseID) -> returns int frameID"},
    {"createSTL", WireRobot_createSTL, METH_VARARGS,  "load a STL file to the scene [filename | position x, y, z and scale]"},
    {"createVRML", WireRobot_createVRML, METH_VARARGS,  "load a VRML file to the scene [filename | position x, y, z and scale]"},
    {"create3DS", WireRobot_create3DS, METH_VARARGS,  "load a 3DS file to the scene [filename | position x, y, z and scale]"},
    
    {"deleteAllFrames", WireRobot_deleteAllFrames, METH_VARARGS,  "delete all frames"},
    {"deleteAllShapes", WireRobot_deleteInternalShapes, METH_VARARGS,  "delete all shapes from the scene [void]"},
    {"deleteFrame", WireRobot_deleteFrame, METH_VARARGS,  "delete selected frame (int id)"},
    {"deleteLayer", WireRobot_deleteLayer, METH_VARARGS,  "delet specified layer (int id)"},
    
    {"invalidate", WireRobot_Invalidate, METH_NOARGS,  "Call the invalidate method of the MFC framework to refresh the screen [void]"},
    
    {"printBindingTable", WireRobot_printBindingTable, METH_NOARGS,  "print the name of the attributes"},
    {"printGLParam", WireRobot_PrintGLParam, METH_NOARGS,  "print some parameters related to the current scene [void]"},
    
    {"saveBmp", WireRobot_saveBmp, METH_VARARGS,  "save a screenshot to a bmp file [filename, | width, height]"},
    //{"saveEps", WireRobot_saveEps, METH_VARARGS,  "save a screenshot to a eps file [filename]"},
    {"saveLog", WireRobot_saveLog, METH_VARARGS,  "save the console to a given filename [name, double]"},
    {"saveXml", WireRobot_saveXml, METH_VARARGS, "save robot xml (.wcrfx) [filename]"},
    {"saveProject", WireRobot_saveProject, METH_VARARGS, "save the current robot project into a WireCenterProjectFile (.wcpf) [filename]"},
    {"saveDefaults", WireRobot_savePersistantDefaults, METH_VARARGS,  "save the persistent default values to a file including values and name scheme [filename]"},

    {"selectDifferentialHull", WireRobot_selectDifferentialHull, METH_VARARGS,  "choose which parameter variation is shown [0 <= int <m*6]"},
    {"signPythonScript", WireRobot_signPythonScript, METH_VARARGS,  "convert a normal python script to signed script (wcss) [pyfile | wcssfile]"},
    {"startServer", startServer, METH_VARARGS,  "Start IPC server"},
    
    {"loadCNCList", WireRobot_LoadCNCList, METH_VARARGS,  "Load a CNC list file with robot geometry [filename]"},
    {"loadXml", WireRobot_loadXml, METH_VARARGS, "load robot xml (.wcrfx) [filename]"},
    {"loadProject", WireRobot_loadProject, METH_VARARGS, "load the current robot project from a WireCenterProjectFile (.wcpf) [filename]"},
    {"loadDefaults", WireRobot_loadPersistantDefaults, METH_VARARGS,  "load the persistent default values from a file including their values and the name scheme [filename]"},
    
    {"unsignPythonScript", WireRobot_unsignPythonScript, METH_VARARGS,  "convert a signed script (wcss) python script to a normal script [wcssfile | pyfile ]"},
    
    {"AppReqMaxInstallationSpace", WireRobot_AppReqMaxInstallationSpace, METH_NOARGS, "Returns max Installation space Vector for Workspace Requirements"},
    {"AppReqMaxWorkspace", WireRobot_AppReqMaxWorkspace, METH_NOARGS, "Returns max Workspace Vector for Workspace Requirements"},
    {"AppReqMinInstallationSpace", WireRobot_AppReqMinInstallationSpace, METH_NOARGS, "Returns min Installation space Vector for Workspace Requirements"},
    {"AppReqMinWorkspace", WireRobot_AppReqMinWorkspace, METH_NOARGS, "Returns min Workspace Vector for Workspace Requirements"},
    {"AppRequirements", WireRobot_AppRequirements, METH_NOARGS, "maximum platform velocity [m/s]; maximum platform acceleration [m/s²]; payload [N]; maximum applied force [N]; maximum applied torque [Nm]"},
    
	{"generatePoseListRandom", WireRobot_generatePoseListRandom, METH_VARARGS,  "creates a Poselist with random poses\n[numberOfPoses, rx_min, ry_min, rzmin, rx_max, ry_max, rz_max, Ra_limit, Rb_limit, Rc_limit]"},
	{"generatePoseListGrid", WireRobot_generatePoseListGrid, METH_VARARGS,  "creates a Poselist as a grid of poses (no rotation)\n[rx_min, ry_min, rzmin, rx_max, ry_max, rz_max, rx_stepsize, ry_stepsize, rz_stepsize]"},
	{"generatePoseListGridRandomRot", WireRobot_generatePoseListGridRandomRot, METH_VARARGS,  "creates a Poselist as a grid of poses (random rotation)\n[rx_min, ry_min, rzmin, rx_max, ry_max, rz_max, rx_stepsize, ry_stepsize, rz_stepsize, Ra_limit, Rb_limit, Rc_limit]"},
	{"PoseListdeleteAllPoses",WireRobot_deletePoseList, METH_NOARGS, "remove all Poses in current PoseList"},

    // NC-Interpolator -> should be moved to WireLib bindings
    {"ncExportPoselist", WireRobot_ncExportPoselist, METH_VARARGS,  "export the poselist as csv [filename]"},
    {"ncImportPoselist", WireRobot_ncImportPoselist, METH_VARARGS,  "import a poselist from csv [filename, format]"},
    {"ncLoadCNCList", WireRobot_LoadCNCList, METH_VARARGS,  "Load a CNC list file with robot geometry [filename]"},
    {"ncLoadProgram", WireRobot_ncLoadNCProgram, METH_VARARGS,  "parse and interpolate a NC-File"},
	{"ncLoadPoseEvaluatorSettings", WireRobot_ncLoadPoseEvaluatorSettings, METH_VARARGS,  "load settings for the PoseEvaluator from a txt-file [filename]"},
	{"ncSavePoseEvaluatorSettings", WireRobot_ncSavePoseEvaluatorSettings, METH_VARARGS,  "save settings for the PoseEvaluator in a txt-file [filename]"},
    {"ncTrajektorie", WireRobot_TrajektorieAnalysieren, METH_VARARGS, "start Trajectory Analysis [filename]"},

	//Kinematik Test
	{"vlsTestScenario", WireRobot_BerechnenKinematik, METH_NOARGS, "Run Kinematik Test"},
	{"peSaveSettings", WireRobot_SavePoseEvaluatorSettings, METH_VARARGS, "Save Pose Evaluator Settings using reflection [filename]"},
	{"peLoadSettings", WireRobot_LoadPoseEvaluatorSettings, METH_VARARGS, "Load Pose Evaluator Settings using reflection [filename]"},
	{"setColPosePropToVisualize", WireRobot_setColPosePropToVisualize, METH_VARARGS, "Set the index of the column of the PosePropertyEvaluator results which shall be visualized."},

	{NULL, NULL, 0, NULL}
};

/* print the full method table of the Iapp object */
static PyObject* WireRobot_HelpIApp(PyObject *self, PyObject *args)
{
	for ( PyMethodDef* ptr = EmbMethodsIapp; ptr->ml_name != 0; ptr++)
        PySys_WriteStdout("Iapp.%s: %s\n", ptr->ml_name, ptr->ml_doc);
	
	Py_RETURN_NONE;
}

/* Method wrapper for CPathGeneratorClass *
 ******************************************/

CPathGenerator* pyHelperPG() { return &(CWireCenterView::This->PathGenerator); }

PYHELPER_VOID_STRING( PathGenerator_Load, pyHelperPG()->load )
PYHELPER_VOID_STRING( PathGenerator_Save, pyHelperPG()->savePath )
PYHELPER_BOOL_VOID (PathGenerator_GeneratePath, pyHelperPG()->generatePath() )

static PyObject* PathGenerator_SetEps(PyObject *self, PyObject *args)
{
	double eps_LayerDist,eps_PathDist;
	if ( ! PyArg_ParseTuple(args, "dd",&eps_LayerDist,&eps_PathDist) )
        return NULL;
	
    pyHelperPG()->eps_LayerDist=eps_LayerDist;
	pyHelperPG()->eps_PathDist=eps_PathDist;

	Py_RETURN_NONE;
}

/* */
static PyObject* PathGenerator_setUseExternalBoundingBox(PyObject *self, PyObject *args)
{
	bool bExternalBB;
	if ( ! PyArg_ParseTuple(args, "b",&bExternalBB) )
        return NULL;

	pyHelperPG()->setUseExternalBoundingBox(bExternalBB);
	
    Py_RETURN_NONE;
}

/*  */
static PyObject* PathGenerator_setExternalBoundingBox(PyObject *self, PyObject *args)
{
	CVector3 Min,Max;

	if ( ! PyArg_ParseTuple(args, "dddddd",&Min.x,&Min.y,&Min.z,&Max.x,&Max.y,&Max.z) )
        return NULL;
	
	pyHelperPG()->setExternalBoundingBox(Min,Max);

	Py_RETURN_NONE;
}

/*  */
static PyObject* PathGenerator_setExternalBoundingBoxToRobotBoundingBox(PyObject *self, PyObject *args)
{
	CVector3 Min,Max; // IPAGL
	Vector3d bbmin, bbmax; // Eigen 3

	if ( ! pyHelperDoc()->robotDoc.getBoundingBoxBase(bbmin,bbmax) )
        Py_RETURN_FALSE;

	// convert from Eigen3 to IPAGl-class
	Min.x=bbmin.x(); Min.y=bbmin.y(); Min.z=bbmin.z();
	Max.x=bbmax.x(); Max.y=bbmax.y(); Max.z=bbmax.z();

	pyHelperPG()->setExternalBoundingBox(Min,Max);

	Py_RETURN_NONE;
}

/* */
static PyObject* PathGenerator_setMeanderShapedPath(PyObject *self, PyObject *args)
{
	bool bMeanderShaped;
	if ( ! PyArg_ParseTuple(args, "b",&bMeanderShaped) )
        return NULL;

	pyHelperPG()->setMeanderShapedPath(bMeanderShaped);

	Py_RETURN_NONE;
}


static PyObject* PathGenerator_HelpIpg(PyObject *self, PyObject *args);

// create the "method table" for the functions related to the path generator object
static PyMethodDef EmbMethodsIpg[] = {
    {"help", PathGenerator_HelpIpg, METH_NOARGS,  "Print the names of all methods in Ipg"},

    {"setEps", PathGenerator_SetEps, METH_VARARGS,  "set the algorithms parameters [Layer Distance between paths (x),Path Distance between planes (z)]"},
	{"setExternalBoundingBox", PathGenerator_setExternalBoundingBox, METH_VARARGS,  "set the min and max values of the external bounding box [xmin, ymin, zmin, xmax, ymax, zmax]"},
	{"setExternalBoundingBoxToRobotBoundingBox", PathGenerator_setExternalBoundingBoxToRobotBoundingBox, METH_NOARGS,  "set the min and max values of the external bounding box to bounding box of the robot frame []"},
	{"setMeanderShapedPath", PathGenerator_setMeanderShapedPath, METH_VARARGS,  "enable meander shaped path [bool]"},
    {"setUseExternalBoundingBox", PathGenerator_setUseExternalBoundingBox, METH_VARARGS,  "enable the use of the external bounding box to define the scope of PG [bool]"},

    {"generatePath", PathGenerator_GeneratePath, METH_NOARGS,  "perform the calculation [void]"},

    {"load", PathGenerator_Load, METH_VARARGS,  "load the given STL file [filename]"},

    {"save", PathGenerator_Save, METH_VARARGS,  "save the generated path in ASCII file [filename]"},
    
    {NULL, NULL, 0, NULL}
};

/* print the full method table of the Iapp object */
static PyObject* PathGenerator_HelpIpg(PyObject *self, PyObject *args)
{
	for (PyMethodDef* ptr = EmbMethodsIpg; ptr->ml_name!=0; ptr++)
		PySys_WriteStdout("Ipg.%s: %s\n",ptr->ml_name,ptr->ml_doc);
	
	Py_RETURN_NONE;
}

static PyObject* WireCenterView_ViewPlane(PyObject *self, PyObject *args)
{
	char *input;
	if ( ! PyArg_ParseTuple(args, "s",&input) )
        return NULL;

	if ( ! strcmp(input,"xy") )
		pyHelperView()->OnAnsichtAusrichtenxyebene();
	else if ( ! strcmp(input,"xz") )
		pyHelperView()->OnAnsichtAusrichtenxzebene();
	else
		pyHelperView()->OnAnsichtAusrichtenyzebene();
	
	Py_RETURN_NONE;
}




static PyObject* WireRobot_HelpIscene(PyObject *self, PyObject *args);

PyMethodDef EmbMethodsIscene[] = {
    {"help", WireRobot_HelpIscene, METH_NOARGS,  "Print the names of all methods in Iscene"},
    
    {"setBackgroundColor", WireRobot_setBackgroundColor, METH_VARARGS,  "set the background color in RGBA-Model [lower r, g, b, a | upper r, g, b, a ]"},
    {"setGLParameters", WireRobot_setGLParameters, METH_VARARGS, "change GL Parameters [Zoom]"},
    {"setGLPosition", WireRobot_setGLPosition, METH_VARARGS, "set the position in the openGL model perspective [x,y,z]"},
    {"setGLOrientation", WireRobot_setGLOrientation, METH_VARARGS, "set the orienation in the openGL model perspective by a quaternion [q0,q1,q2,q3]"},
    {"setGLProjection", WireRobot_setGLProjection, METH_VARARGS,  "set the projection mode of open GL [0=orthogonal, 1=perspective]"},
    {"setHullColor", WireRobot_setHullColor, METH_VARARGS,  "set the workspace hull color in RGB-Model [r, g, b]"}, 
    {"setInterferenceVisibility", WireRobot_setInterferenceVisibility, METH_VARARGS,  "set visibility of cable interference region [0:false int:true]"},
    {"setPlanesVisibility", WireRobot_setPlanesVisibility, METH_VARARGS,  "set visibility for the planes [0:false int:true]"},
    {"setPlatformVisibility", WireRobot_setPlatformVisibility, METH_VARARGS,  "set visibility for the platform [0:false int:true]"},
    {"setShapeDefaultAttribute", WireRobot_setShapeDefaultAttribute, METH_VARARGS,  "set the default values for new shape objects (int Frame, int Layer, int visible)"},
    {"setWiresVisibility", WireRobot_setWiresVisibility, METH_VARARGS,  "set visibility for the wires [0:false int:true]"},
    {"setWorkspaceVisibility", WireRobot_setWorkspaceVisibility, METH_VARARGS,  "set visibility for workspace hull [0:false int:true]"},

	{"setSpanCableSelector", WireRobot_setSpanCableSelector, METH_VARARGS,  "selects the visible cable in span; negative numbers make all cables visible (default) [cableSelector]"},

    {"getPlatformFrame", WireRobot_getPlatformFrame, METH_NOARGS,  "determine the GL handle of the platform frame"},
    
    {"changeViewPlane", WireCenterView_ViewPlane, METH_VARARGS, "change view to plane view [xz OR xy OR yz]"},
	{"setPoseEstimateBoxVisibility",WireRobot_setPoseEstimateBoxVisibility, METH_VARARGS, "set visibility of pose estimate box [0:false int:true]"},
	{"setPoseEstimateBoxVisibility",WireRobot_setPoseEstimateBoxSphereVisibility, METH_VARARGS, "set visibility of pose estimate box spheres [0:false int:true]"},
    {"createBox", WireRobot_createBox, METH_VARARGS,  "adds a box to the GL scene [VECTOR (3 doubles), VECTOR (3 doubles)]"},
    {"createCone", WireRobot_createCone, METH_VARARGS,  "adds a cone to the GL scene [Apex, Axis]"},
    {"createSTL", WireRobot_createSTL, METH_VARARGS,  "load a STL file to the scene [filename | position x,y,z, scale | angle a, angle b, angle c, r, g, b, transparency]"},
    {"createVRML", WireRobot_createVRML, METH_VARARGS,  "load a VRML file to the scene [filename | position x,y,z and scale | angle a, angle b, angle c, r, g, b, transparency]"},
    {"createVRMLWinch", WireRobot_createVRML_Winch, METH_VARARGS,  "load a winch VRML file to the scene and put it to the winch base frame [filename base nr | position x, y, z and scale]"},
    {"create3DS", WireRobot_create3DS, METH_VARARGS,  "load a 3DS file to the scene [filename | position x,y,z and scale | angle a, angle b, angle c]"},
    
    {"deleteAllShapes", WireRobot_deleteInternalShapes, METH_VARARGS,  "delete all shapes from the scene [void]"},
    
    {"invalidate", WireRobot_Invalidate, METH_NOARGS,  "Call the invalidate method of the MFC framework to refresh the screen [void]"},
    
    {"printGLParam", WireRobot_PrintGLParam, METH_NOARGS,  "print some parameters related to the current scene [void]"},
    
    {"saveBmp", WireRobot_saveBmp, METH_VARARGS,  "save a screenshot to a bmp file [filename, | width, height]"},
    {"saveEps", WireRobot_saveEps, METH_VARARGS,  "save a screenshot to a eps file [filename]"},
    {"selectDifferentialHull", WireRobot_selectDifferentialHull, METH_VARARGS,  "choose which parameter variation is shown [0 <= int <m*6]"},

    {NULL, NULL, 0, NULL}
};

static PyObject* Plugin_invoke(PyObject *self, PyObject *args)
{
	// extract the name of the called function as string
	PyObject* pyFunction;
	Py_ssize_t size = PyTuple_Size(args);

	if ( size <= 1 )
        Py_RETURN_FALSE;

	pyFunction = PyTuple_GetItem(args, 0);

	char* function=PyString_AsString(pyFunction);

	// check if a plugin is loaded
	CMainFrame* pMF=(CMainFrame*)AfxGetApp()->GetMainWnd();

	if ( pMF && pMF->myPlugin && pMF->myPlugin->handle )
	{
		// we truncate the argument tuple by the first elements that stored the name of function to be called
		PyObject* args2=PyTuple_GetSlice(args,1,size);
		// if we successed we call the Python Invoke method inside the DLL
		if (args2)
			return pMF->myPlugin->PyInvoke(function,self,args2);
		
		return NULL;
	}

	return NULL;
}

static PyObject* Plugin_printInfo(PyObject *self, PyObject *args)
{
	// check if a plugin is loaded
	CMainFrame* pMF=(CMainFrame*)AfxGetApp()->GetMainWnd();
	if ( pMF && pMF->myPlugin && pMF->myPlugin->handle )
		pMF->myPlugin->printInfos();

	Py_RETURN_NONE;
}

static PyObject* Plugin_release(PyObject *self, PyObject *args)
{
	// check if a plugin is loaded
	CMainFrame* pMF=(CMainFrame*)AfxGetApp()->GetMainWnd();

	if (pMF && pMF->myPlugin && pMF->myPlugin->handle)
	{
		pMF->myPlugin->release();
	}

	Py_RETURN_NONE;
}

static PyMethodDef EmbMethodsIplugin[] = {
	{"printInfo", Plugin_printInfo, METH_NOARGS, "print the name and desciprtion of all functions in the plugin"},

	{"release", Plugin_release, METH_NOARGS, "unload the current plugin"},

    {"invoke", Plugin_invoke, METH_VARARGS, "Call a function in the plugin and possibly pass any number of parameters to that function [filename ...]"},

	{NULL, NULL, 0, NULL}
};


/* print the full method table of the Iscene object */
static PyObject* WireRobot_HelpIscene(PyObject *self, PyObject *args)
{
	for (PyMethodDef* ptr = EmbMethodsIscene; ptr->ml_name!=0; ptr++)
		PySys_WriteStdout("Iscene.%s: %s\n", ptr->ml_name, ptr->ml_doc);
	
	Py_RETURN_NONE;
}

//////////////////////////////////////////////////////////////////////////////
// initialize WireCenter's python scripting tables
//////////////////////////////////////////////////////////////////////////////

/*! this function connects the module names with the method tables
 *  which in turn connect the module's function names with the
 *  wrapper functions.
 *  Call this function for WireCenter to fill the abstract python interface 
 *  with life and access to this application's function libraries.
 */
bool wcInitPythonBinding()
{
	CPythonInterface::getInstance().addMethodTable("Irobot", EmbMethodsIrobot);		// Methods for accessing Wirelib robot data
	CPythonInterface::getInstance().addMethodTable("Iapp", EmbMethodsIapp);			// WireCenter related 
	CPythonInterface::getInstance().addMethodTable("Ipg", EmbMethodsIpg);			// WireCenter related
	CPythonInterface::getInstance().addMethodTable("Ikin", EmbMethodsIkin);			// Wirelib kinematics
	CPythonInterface::getInstance().addMethodTable("Iws", EmbMethodsIws);			// Wirelib workspace
	CPythonInterface::getInstance().addMethodTable("Iscene", EmbMethodsIscene);		// WireCenter scene graph
	CPythonInterface::getInstance().addMethodTable("Iplugin", EmbMethodsIplugin);	// Interface to plugin dlls
	CPythonInterface::getInstance().addMethodTable("Icontrol", EmbMethodsControl);	// Interface to control methods

	return true;
}
