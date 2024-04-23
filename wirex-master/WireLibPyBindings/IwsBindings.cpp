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
 *  \file   : IwsBindings.cpp
 *  \Author   Philipp Miermeister
 *  \Date     25.02.2014
 *	\brief This file contains all python bindings realted to workspace computation
 *********************************
 */ 
#include "WireLibPyBindings.h"

using namespace PCRL;

PYHELPER_VOID_VOID(WireRobot_clearShapes, pyHelperInterference()->clearShapes() );

PYHELPER_VOID_VOID(WireRobot_visualizeInterferenceCableCable, pyHelperInterference()->visualizeInterferenceCableCable() );

PYHELPER_VOID_STRING(WireRobot_visualizeInterferenceCablePlatform, pyHelperInterference()->visualizeInterferenceCablePlatform );

PYHELPER_BOOL_VOID(WireRobot_checkInterferenceWithPrintEllipseMultLayerCriteria,pyHelperInterference()->checkInterferenceWithPrintEllipseMultLayerCriteria());
PYHELPER_BOOL_VOID(WireRobot_checkInterferenceWithPrintParaMultLayerCriteria,pyHelperInterference()->checkInterferenceWithPrintParaMultLayerCriteria());


PYHELPER_VOID_VOID(WireRobot_calculatePrintShadowData, pyHelperInterference()->calculatePrintShadowData() );
PYHELPER_VOID_VOID(WireRobot_visualizeInterferenceCablePrintEllipseMultipleLayers, pyHelperInterference()->visualizeInterferenceCablePrintEllipseMultipleLayers() );
PYHELPER_VOID_VOID(WireRobot_visualizeInterferenceCablePrintParalellogramMultipleLayers, pyHelperInterference()->visualizeInterferenceCablePrintParalellogramMultipleLayers() );
PYHELPER_BOOL_VOID(WireRobot_calculateInterferenceCablePrintParalellogramMultipleLayers, pyHelperInterference()->calculateInterferenceCablePrintParalellogramMultipleLayers() );
PYHELPER_VOID_VOID(WireRobot_visualizeInterferenceCablePrintStarShape, pyHelperInterference()->visualizeInterferenceCablePrintStarShape() );
PYHELPER_VOID_VOID(WireRobot_visualizeInterferenceCablePrintEllipse, pyHelperInterference()->visualizeInterferenceCablePrintEllipse() );
PYHELPER_VOID_VOID(WireRobot_visualizeInterferenceCablePrintParallelogram, pyHelperInterference()->visualizeInterferenceCablePrintParallelogram() );

PYHELPER_VOID_STRING(WireRobot_writeStarShapeToCSV, pyHelperInterference()->writeStarShapeToCSV );
PYHELPER_VOID_STRING(WireRobot_writeParallelogramToCSV, pyHelperInterference()->writeParallelogramToCSV );
PYHELPER_VOID_STRING(WireRobot_writeParallelogramMultipleLayerToCSV, pyHelperInterference()->writeParallelogramMultipleLayerToCSV );
PYHELPER_VOID_STRING(WireRobot_writeEllipseToCSV, pyHelperInterference()->writeEllipseToCSV );

/* updateClippingPlanes */
PYHELPER_VOID_VOID(WireRobot_updateClippingPlanes, pyHelperInterference()->updateClippingPlanes() );

/* writeCablePlatformPlatformToCSV*/
static PyObject* WireRobot_writeCablePlatformPlatformToCSV(PyObject *self, PyObject *args)
{
	char* filename;

	if ( ! PyArg_ParseTuple(args, "s",&filename) )
        return NULL;

	if ( pyHelperInterference()->writeCablePlatformPlatformToCSV(filename) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* writeRobotGeometryToCSV*/
static PyObject* WireRobot_writeRobotGeometryToCSV(PyObject *self, PyObject *args)
{
	char* filename;

	if ( ! PyArg_ParseTuple(args, "s",&filename) )
        return NULL;

	if ( pyHelperInterference()->writeRobotGeometryToCSV(filename) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* setPn */
static PyObject* WireRobot_setPn(PyObject *self, PyObject *args)
{
	double d0,d1,d2;
	Vector3d p;

    if ( ! PyArg_ParseTuple(args, "ddd", &d0, &d1, &d2) )
        return NULL;

	p(0)=d0;
	p(1)=d1;
	p(2)=d2;

	pyHelperInterference()->setPn(p);
	
    Py_RETURN_NONE;
}

/* setPrintCenter */
static PyObject* WireRobot_setPrintCenter(PyObject *self, PyObject *args)
{
	double d0,d1,d2;
	Vector3d center;

    if ( ! PyArg_ParseTuple(args, "ddd", &d0, &d1, &d2) )
        return NULL;

	center(0)=d0;
	center(1)=d1;
	center(2)=d2;

	pyHelperInterference()->setPrintCenter(center);
	
    Py_RETURN_NONE;
}

/* setPlatformCenter */
static PyObject* WireRobot_setPlatformCenter(PyObject *self, PyObject *args)
{
	double d0,d1,d2;
	Vector3d center;

    if ( ! PyArg_ParseTuple(args, "ddd", &d0, &d1, &d2) )
        return NULL;

	center(0)=d0;
	center(1)=d1;
	center(2)=d2;

	pyHelperInterference()->setPlatformCenter(center);
	
    Py_RETURN_NONE;
}

/* setPrintDir */
static PyObject* WireRobot_setPrintDir(PyObject *self, PyObject *args)
{
	double d0,d1,d2,d3;
	Vector3d dir1,dir2;

    if ( ! PyArg_ParseTuple(args, "dddd", &d0, &d1, &d2, &d3) )
        return NULL;

	dir1 << d0, d1, 0;
	dir2 << d2, d3, 0;

	pyHelperInterference()->setPrintDir(dir1,dir2);
	
    Py_RETURN_NONE;
}

/* setPrintHeight */
static PyObject* WireRobot_setPrintHeight(PyObject *self, PyObject *args)
{
	double d0;

    if ( ! PyArg_ParseTuple(args, "d", &d0) )
        return NULL;

	pyHelperInterference()->setPrintHeight(d0);
	
    Py_RETURN_NONE;
}

/* setDeltaAlpha */
static PyObject* WireRobot_setDeltaAlpha(PyObject *self, PyObject *args)
{
	double d0;

    if ( ! PyArg_ParseTuple(args, "d", &d0) )
        return NULL;

	pyHelperInterference()->setDeltaAlpha(d0);
	
    Py_RETURN_NONE;
}

/* setDeltaEta */
static PyObject* WireRobot_setDeltaEta(PyObject *self, PyObject *args)
{
	double d0;

    if ( ! PyArg_ParseTuple(args, "d", &d0) )
        return NULL;

	pyHelperInterference()->setDeltaEta(d0);
	
    Py_RETURN_NONE;
}

/* setEllipseExp */
static PyObject* WireRobot_setEllipseExp(PyObject *self, PyObject *args)
{
	double d0;

    if ( ! PyArg_ParseTuple(args, "d", &d0) )
        return NULL;

	pyHelperInterference()->setEllipseExp(d0);
	
    Py_RETURN_NONE;
}

/* setPrintVisMode */
static PyObject* WireRobot_setPrintParaVisNumber(PyObject *self, PyObject *args)
{
	int d0;

    if ( ! PyArg_ParseTuple(args, "i", &d0) )
        return NULL;

	pyHelperInterference()->setPrintParaVisNumber(d0);
	
    Py_RETURN_NONE;
}

/* setPrintVisMode */
static PyObject* WireRobot_setNumberOfPrintLayers(PyObject *self, PyObject *args)
{
	int d0;

    if ( ! PyArg_ParseTuple(args, "i", &d0) )
        return NULL;

	pyHelperInterference()->setNumberOfPrintLayers(d0);
	
    Py_RETURN_NONE;
}

/* setStepsCircle */
static PyObject* WireRobot_setStepsCircle(PyObject *self, PyObject *args)
{
	int d0;

    if ( ! PyArg_ParseTuple(args, "i", &d0) )
        return NULL;

	pyHelperInterference()->setStepsCircle(d0);
	
    Py_RETURN_NONE;
}

/* setRefinement */
static PyObject* WireRobot_setRefinement(PyObject *self, PyObject *args)
{
	int d0;

    if ( ! PyArg_ParseTuple(args, "i", &d0) )
        return NULL;

	pyHelperInterference()->setRefinement(d0);
	
    Py_RETURN_NONE;
}

/* setSearchSteps */
static PyObject* WireRobot_setSearchSteps(PyObject *self, PyObject *args)
{
	int d0;

    if ( ! PyArg_ParseTuple(args, "i", &d0) )
        return NULL;

	pyHelperInterference()->setSearchSteps(d0);
	
    Py_RETURN_NONE;
}

/* setCableCableSetting */
static PyObject* WireRobot_setCableCableSetting(PyObject *self, PyObject *args)
{
	bool d0;

    if ( ! PyArg_ParseTuple(args, "b", &d0) )
        return NULL;

	pyHelperInterference()->setCableCableSetting(d0);
	
    Py_RETURN_NONE;
}

/* setPrintDisplayShadows */
static PyObject* WireRobot_setPrintDisplayShadows(PyObject *self, PyObject *args)
{
	bool d0;

    if ( ! PyArg_ParseTuple(args, "b", &d0) )
        return NULL;

	pyHelperInterference()->setPrintDisplayShadows(d0);
	
    Py_RETURN_NONE;
}

/* calculateInterferenceWithPrintEllipse */
PYHELPER_VOID_VOID(WireRobot_calculateInterferenceWithPrintEllipse, pyHelperInterference()->calculateInterferenceWithPrintEllipse() );

/* calculateInterferenceWithPrintParallelogram */
PYHELPER_VOID_VOID(WireRobot_calculateInterferenceWithPrintParallelogram, pyHelperInterference()->calculateInterferenceWithPrintParallelogram() );

/* calculateInterferenceWithPrintMaxStar */
PYHELPER_VOID_VOID(WireRobot_calculateInterferenceWithPrintStarShape, pyHelperInterference()->calculateInterferenceWithPrintStarShape() );


/* setupCablePlatformCollisionCones */
PYHELPER_VOID_VOID(WireRobot_setupCablePlatformCollisionCones, pyHelperInterference()->setupCablePlatformCollisionCones() );

/* calculateCablePlatform */
PYHELPER_VOID_VOID(WireRobot_calculateCablePlatform, pyHelperInterference()->calculateCablePlatform() );

/* newCablePlatformRayBall */
PYHELPER_VOID_VOID(WireRobot_newCablePlatformRayBall, pyHelperInterference()->newCablePlatformRayBall() );

/* writeCablePlatformConesToCSV*/
static PyObject* WireRobot_loadplatformstldata(PyObject *self, PyObject *args)
{
	char* filename;

	if ( ! PyArg_ParseTuple(args, "s",&filename) )
        return NULL;

	if ( pyHelperInterference()->loadplatformstldata(filename) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* writeCablePlatformConesToCSV*/
static PyObject* WireRobot_writeCablePlatformConesToCSV(PyObject *self, PyObject *args)
{
	char* filename;

	if ( ! PyArg_ParseTuple(args, "s",&filename) )
        return NULL;

	if ( pyHelperInterference()->writeCablePlatformConesToCSV(filename) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* writeCablePlatformTrianglelistToCSV*/
static PyObject* WireRobot_writeCablePlatformTrianglelistToCSV(PyObject *self, PyObject *args)
{
	char* filename;

	if ( ! PyArg_ParseTuple(args, "s",&filename) )
        return NULL;

	if ( pyHelperInterference()->writeCablePlatformTrianglelistToCSV(filename) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* writeCablePlatformRaysToCSV*/
static PyObject* WireRobot_writeCablePlatformRaysToCSV(PyObject *self, PyObject *args)
{
	char* filename;

	if ( ! PyArg_ParseTuple(args, "s",&filename) )
        return NULL;

	if ( pyHelperInterference()->writeCablePlatformRaysToCSV(filename) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* writeCableCableSetsToCSV*/
static PyObject* WireRobot_writeCableCableSetsToCSV(PyObject *self, PyObject *args)
{
	char* filename;

	if ( ! PyArg_ParseTuple(args, "s",&filename) )
        return NULL;

	if ( pyHelperInterference()->writeCableCableSetsToCSV(filename) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* setupCableCableInterferenceSets */
PYHELPER_VOID_VOID(WireRobot_setupCableCableInterferenceSets, pyHelperInterference()->setupCableCableInterferenceSets() );

/* calculateCableCableInterferenceSetsHulls */
PYHELPER_VOID_VOID(WireRobot_calculateCableCableInterferenceSetsHulls, pyHelperInterference()->calculateCableCableInterferenceSetsHulls() );

/* return the current setting for the accuracy of the line search */
static PyObject* WireRobot_getMinInterferenceAngle(PyObject *self, PyObject *args)
{
	return Py_BuildValue("d",pyHelperInterference()->calculateMinInterferenceAngle());
}

/* set etaMax */
static PyObject* WireRobot_SetEtaMax(PyObject *self, PyObject *args)
{
	double eta;

    if ( ! PyArg_ParseTuple(args, "d",&eta) )
        return NULL;

	pyHelperInterference()->setEtaMax(eta);
	
    Py_RETURN_NONE;
}

//Old functions
	
/* start calculation of optimal cone*/
static PyObject* WireRobot_CalculateOptimalCone(PyObject *self, PyObject *args)
{
	unsigned int id;
	Vector3d apex, axis_out;
	double aperture_out;

	if ( ! PyArg_ParseTuple(args, "i", &id) )
        return NULL;

	pyHelperHull()->calculateOptimalCone(id, axis_out, aperture_out);
	// create 3d cone for debugging
	// (to be removed)
	apex = pyHelperRobot()->getBase(id);
	//! \todo: wrap painter function in separate module associated with the GUI
	//CWireCenterView::This->m_Scene.createCone(CVector3(apex.x(),apex.y(),apex.z()),CVector3(axis_out.x(),axis_out.y(),axis_out.z()),aperture_out, CVector3(0,0.5,0),0.5);

	Py_RETURN_TRUE;
}

/* start calculation of disabled workspace*/
static PyObject* WireRobot_CalcDisabledWspc(PyObject *self, PyObject *args)
{
	Vector3d Min, Max;

	if ( ! PyArg_ParseTuple(args, "dddddd",&Min[0],&Min[1],&Min[2],&Max[0],&Max[1],&Max[2]) )
        return NULL;

	pyHelperHull()->calcDisabledWspc(Min,Max);

	Py_RETURN_TRUE;
}

/* start the workspace calculation */
static PyObject* WireRobot_CalculateWorkspace(PyObject *self, PyObject *args)
{
	pyHelperHull()->calculateWorkspace();

	Py_RETURN_TRUE;
}

/* start the workspace calculation */
static PyObject* WireRobot_IntersectWorkspace(PyObject *self, PyObject *args)
{
	pyHelperHull()->intersectWorkspace();

	Py_RETURN_TRUE;
}

/* start the workspace calculation */
static PyObject* WireRobot_UniteWorkspace(PyObject *self, PyObject *args)
{
	pyHelperHull()->uniteWorkspace();

	Py_RETURN_TRUE;
}

/* start the workspace intersection */
static PyObject* WireRobot_IntersectWorkspaceWireLength(PyObject *self, PyObject *args)
{
//	pyHelperHull()->intersectWorkspace_WireLength();
	Py_RETURN_FALSE; // as long as the function fall is disabled we return false instead of true pyTrue();
}

/* start the workspace crosssection calculation */
static PyObject* WireRobot_CalculateWorkspaceCrosssection(PyObject *self, PyObject *args)
{
	char normal;

    if ( ! PyArg_ParseTuple(args, "c",&normal) )
        return NULL;

	pyHelperCS()->calculateWorkspaceCrosssection(normal);

	Py_RETURN_TRUE;
}

/* start the statistics calculation */
static PyObject* WireRobot_CalculateWorkspaceProperties(PyObject *self, PyObject *args)
{
	if ( ! pyHelperHull()->calculateWorkspaceProperties() )
        Py_RETURN_FALSE;

	double volume = pyHelperHull()->Volume;
	double surface = pyHelperHull()->Surface;
	Vector3d CoI = pyHelperHull()->CoI;

	return Py_BuildValue("{s:d,s:d,s:(d,d,d)}","volume", volume,"surface",surface,"CoI",CoI[0], CoI[1], CoI[2]);
}

PYHELPER_VOID_VOID( WireRobot_PrintProperties, pyHelperHull()->printProperties())

/* print statistics on the screen */
/*static PyObject* WireRobot_PrintProperties(PyObject *self, PyObject *args)
{
	pyHelperHull()->printProperties();

    Py_RETURN_NONE;
}*/

/* set iterations */
static PyObject* WireRobot_SetIteration(PyObject *self, PyObject *args)
{
	int iterations;

    if ( ! PyArg_ParseTuple(args, "i",&iterations) )
        return NULL;

	pyHelperHull()->setIterations(iterations);

    Py_RETURN_NONE;
}

/* set projection center */
static PyObject* WireRobot_SetProjectionCenter(PyObject *self, PyObject *args)
{
	double x,y,z;

    if ( ! PyArg_ParseTuple(args, "ddd",&x,&y,&z) )
        return NULL;

	pyHelperHull()->setProjectionCenter(Vector3d(x,y,z));
	
    Py_RETURN_NONE;
}

/* set eps */
static PyObject* WireRobot_SetEps(PyObject *self, PyObject *args)
{
	double eps;

    if ( ! PyArg_ParseTuple(args, "d",&eps) )
        return NULL;

	pyHelperHull()->setEps(eps);
	
    Py_RETURN_NONE;
}

/* set eps */
static PyObject* WireRobot_SetEpsGeometry(PyObject *self, PyObject *args)
{
	double eps;

    if ( ! PyArg_ParseTuple(args, "d",&eps))
        return NULL;

	pyHelperHull()->setEpsGeometry(eps);
	
    Py_RETURN_NONE;
}

/* set range for line search*/
static PyObject* WireRobot_SetSearchRange(PyObject *self, PyObject *args)
{
	double range;

    if ( ! PyArg_ParseTuple(args, "d",&range) )
        return NULL;

	pyHelperHull()->setSearchRange(range);

    Py_RETURN_NONE;
}

/* set range for line search*/
static PyObject* WireRobot_SetConditionMin(PyObject *self, PyObject *args)
{
	double min;

    if ( ! PyArg_ParseTuple(args, "d",&min) )
        return NULL;

	pyHelperFD()->setConditionMin(min);

    Py_RETURN_NONE;
}

static PyObject* WireRobot_saveWorkspace(PyObject *self, PyObject *args)
{
	char* filename;

	if ( ! PyArg_ParseTuple(args, "s",&filename) )
        return NULL;

	if ( pyHelperHull()->saveWorkspace(filename) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

PYHELPER_BOOL_STRING( WireRobot_saveWorkspaceVerticesCSV, pyHelperHull()->saveVerticesCSV )
/* save workspace to CSV*/
/*static PyObject* WireRobot_saveWorkspaceVerticesCSV(PyObject *self, PyObject *args)
{
	char* filename;

	if ( ! PyArg_ParseTuple(args, "s",&filename) )
        return NULL;

	if ( pyHelperHull()->saveVerticesCSV(filename) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}*/

/* load workspace from CSV*/
static PyObject* WireRobot_loadWorkspaceVerticesCSV(PyObject *self, PyObject *args)
{
	char* filename;
	int IterationDepth;

	if ( ! PyArg_ParseTuple(args, "si",&filename,&IterationDepth) )
        return NULL;

	if ( pyHelperHull()->loadWorkspaceCSV(filename, IterationDepth) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

PYHELPER_BOOL_STRING( WireRobot_saveWorkspaceMatlab, pyHelperHull()->saveWorkspaceMatlab )

/* save workspace */
/*static PyObject* WireRobot_saveWorkspaceMatlab(PyObject *self, PyObject *args)
{
	char* filename;

	if ( ! PyArg_ParseTuple(args, "s",&filename) )
        return NULL;

	if ( pyHelperHull()->saveWorkspaceMatlab(filename) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}*/

static PyObject* WireRobot_getWorkspaceCrosssectionMatrix(PyObject *self, PyObject *args)
{
	MatrixXd cs;

	if ( pyHelperCS()->getWorkspaceCrosssectionMatrix(cs) )
		return pyToList(cs);

    Py_RETURN_FALSE;
}

static PyObject* WireRobot_getCableSpanMatrix(PyObject *self, PyObject *args)
{
	MatrixXd cs;

	if ( pyHelperHull()->CableSpanMatrix.size() > 0)
		return pyToList(pyHelperHull()->CableSpanMatrix);

    Py_RETURN_FALSE;
}

static PyObject* WireRobot_getCableSpanTriangulation(PyObject *self, PyObject *args)
{
	MatrixXd cs;

	if ( pyHelperHull()->CableSpanTriangulation.size() > 0)
		return pyToList(pyHelperHull()->CableSpanTriangulation);

    Py_RETURN_FALSE;
}


/* get workspace geometry data*/
static PyObject* WireRobot_getWorkpaceGeometry(PyObject *self, PyObject *args)
{
	vector<Vector3d> vertices;
	vector<CTriangleIndices> triangles;
	int finishedTriangles;
	if ( ! pyHelperHull()->getWorkspaceGeometry(vertices, triangles, finishedTriangles) )
        Py_RETURN_FALSE;

	PyObject* pyTriangleList = PyList_New(0);
	for ( int w = 0; w < finishedTriangles; w++ )
	{
		PyObject* pyTriangleTuple = Py_BuildValue("(iii)",triangles[w].i,triangles[w].j,triangles[w].k);
		PyList_Append(pyTriangleList,pyTriangleTuple);
	}

	PyObject* pyVertexList = PyList_New(0);
	for ( unsigned int i = 0; i < vertices.size(); i++ )
	{
		PyObject* vertexTuple = pyToTuple(vertices[i]);
		PyList_Append(pyVertexList,vertexTuple);
	}

	return Py_BuildValue("OO",pyVertexList,pyTriangleList);

}

static PyObject* WireRobot_saveCableSpaceEnvelope(PyObject *self, PyObject *args)
{
	char* filename;
	int i=-1;

	if ( ! PyArg_ParseTuple(args, "s|i",&filename,&i) )
        return NULL;
    
	if ( pyHelperHull()->saveCableSpaceEnvelope(filename,i) )
        Py_RETURN_TRUE;

	Py_RETURN_FALSE;
}

/* verifyWorkspaceBox */
static PyObject* WireRobot_verifyWorkspaceBox(PyObject *self, PyObject *args)
{
	double minx, miny, minz, maxx, maxy, maxz, epsx, epsy, epsz;

	if ( ! PyArg_ParseTuple(args, "ddddddddd",&minx, &miny, &minz, &maxx, &maxy, &maxz, &epsx, &epsy, &epsz) )
        return NULL;

    if ( pyHelperGrid()->verifyWorkspaceBox(minx, miny, minz, maxx, maxy, maxz, epsx, epsy, epsz) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* verify Point */
static PyObject* WireRobot_verifyPoint(PyObject *self, PyObject *args)
{
	double x,y,z;

	if ( ! PyArg_ParseTuple(args, "ddd",&x, &y, &z) )
        return NULL;

    if ( pyHelperHull()->workspaceEvaluator(Vector3d(x, y, z)) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* set the applied wrench */
static PyObject* WireRobot_setWrench(PyObject *self, PyObject *args)
{
	double fx, fy, fz, mx, my, mz;

	if ( ! PyArg_ParseTuple(args, "dddddd",&fx, &fy, &fz, &mx, &my, &mz) )
        return NULL;

	pyHelperFD()->setWrench(Vector3d(fx, fy, fz), Vector3d(mx, my, mz));

	Py_RETURN_NONE;
}

/* save a cross section of the robot's geometry */
static PyObject* WireRobot_SaveCrosssection(PyObject *self, PyObject *args)
{
	char* filename;
	char normal;

	if ( ! PyArg_ParseTuple(args, "sc",&filename,&normal) )
        return NULL;

    if ( pyHelperCS()->saveWorkspaceCrosssection(filename,normal) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

PYHELPER_BOOL_STRING( WireRobot_saveWorkspaceCrosssectionMatlab, pyHelperCS()->saveWorkspaceCrosssectionMatlab )
/* save a cross section of the robot's geometry in a raw file */
/*static PyObject* WireRobot_saveWorkspaceCrosssectionMatlab(PyObject *self, PyObject *args)
{
	char* filename;

	if ( ! PyArg_ParseTuple(args, "s",&filename) )
        return NULL;

    if ( pyHelperCS()->saveWorkspaceCrosssectionMatlab(filename) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}*/

static PyObject* WireRobot_printForceDistribution(PyObject *self, PyObject *args)
{
	double x,y,z;

	if ( ! PyArg_ParseTuple(args, "ddd",&x,&y,&z) )
        return NULL;

	pyHelperFD()->printForceDistribution(Vector3d(x,y,z));
	
    Py_RETURN_NONE;
}

PYHELPER_BOOL_VOID(WireRobot_calculateDifferentialWorkspace, pyHelperHull()->calculateWorkspaceDiff() )
/* */
/*static PyObject* WireRobot_calculateDifferentialWorkspace(PyObject *self, PyObject *args)
{
	if ( pyHelperHull()->calculateWorkspaceDiff() )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}*/

PYHELPER_VOID_VOID( WireRobot_printDifferential, pyHelperHull()->printDifferential() )
/* */
/*static PyObject* WireRobot_printDifferential(PyObject *self, PyObject *args)
{
	pyHelperHull()->printDifferential();

	Py_RETURN_NONE;
}*/

/* get the volume of the workspace */
static PyObject* WireRobot_getWorkspaceVolume(PyObject *self, PyObject *args)
{
	return Py_BuildValue("d", pyHelperHull()->Volume);
}

/* get the surface of the workspace */
static PyObject* WireRobot_getWorkspaceSurface(PyObject *self, PyObject *args)
{
	return Py_BuildValue("d", pyHelperHull()->Surface);
}

/* get the calculation time of the workspace */
static PyObject* WireRobot_getCalculationTime(PyObject *self, PyObject *args)
{
	return Py_BuildValue("i", pyHelperHull()->getCalculationTime());
}

/* */
static PyObject* WireRobot_wireForcesWorkspaceBoxDriver(PyObject *self, PyObject *args)
{
	Vector3d Min,Max,eps(0,0,0);

	if ( ! PyArg_ParseTuple(args, "dddddd|ddd",&Min[0],&Min[1],&Min[2],&Max[0],&Max[1],&Max[2],&eps[0],&eps[1],&eps[2]) )
        return NULL;

    if ( pyHelperGrid()->wireForcesWorkspaceBoxDriver(Min,Max,eps) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* */
static PyObject* WireRobot_setBoundingBox(PyObject *self, PyObject *args)
{
	Vector3d minBB,maxBB;

	if ( ! PyArg_ParseTuple(args, "dddddd",&minBB[0],&minBB[1],&minBB[2],&maxBB[0],&maxBB[1],&maxBB[2]) )
        return NULL;

	pyHelperHull()->setBoundingBox(minBB,maxBB);
	
    Py_RETURN_NONE;
}

/* */
static PyObject* WireRobot_SetForceLimits(PyObject *self, PyObject *args)
{
	double fmin,fmax;

	if ( ! PyArg_ParseTuple(args, "dd",&fmin,&fmax) )
        return NULL;

	pyHelperRobot()->setForceLimits(fmin,fmax);
	
    Py_RETURN_NONE;
}

/* */
static PyObject* WireRobot_GetForceLimits(PyObject *self, PyObject *args)
{
	// return a list with fmin and fmax
	PyObject* myList = PyList_New(0);

	PyList_Append(myList,Py_BuildValue("d",pyHelperRobot()->fmin));
	PyList_Append(myList,Py_BuildValue("d",pyHelperRobot()->fmax));

	return myList;
}

/* set the Workspace Criterion*/
static PyObject* WireRobot_setMethod(PyObject *self, PyObject *args)
{
	int method=0,evaluation=0;

	if ( ! PyArg_ParseTuple(args, "|ii",&method,&evaluation) )
        return NULL;

	pyHelperFD()->setMethod(method,evaluation);
	
    Py_RETURN_NONE;
}

/* set the reference force used in QP algorithm*/
static PyObject* WireRobot_setReferenceForceQP(PyObject *self, PyObject *args)
{
	double f_ref;

	if ( ! PyArg_ParseTuple(args, "d",&f_ref) )
        return NULL;

	pyHelperFD()->setFrefQP(f_ref);
	
    Py_RETURN_NONE;
}

/* set the reference force used in QP algorithm*/
static PyObject* WireRobot_setQPparameters(PyObject *self, PyObject *args)
{
	int max_iters; 
	double resid_tol; 
	double eps; 
	int refine_steps; 
	double kkt_reg;

	if ( ! PyArg_ParseTuple(args, "iddid",&max_iters, &resid_tol, &eps, &refine_steps, &kkt_reg))
        return NULL;

	pyHelperFD()->setQPparameters(max_iters, resid_tol, eps, refine_steps, kkt_reg);
	
    Py_RETURN_NONE;
}

/* determine if the radius of the pulley is taken into account when calcuating the structure matrix*/
static PyObject* WireRobot_setUsePulleyForStructureMatrix(PyObject *self, PyObject *args)
{
	bool use=true;

	if ( ! PyArg_ParseTuple(args, "|b",&use) )
        return NULL;

	pyHelperFD()->usePulley=use;

	Py_RETURN_NONE;
}


/* set the method for force calculation and evaluation*/
static PyObject* WireRobot_setLinearSolver(PyObject *self, PyObject *args)
{
	int method=0;

	if ( ! PyArg_ParseTuple(args, "|i",&method) )
        return NULL;

	pyHelperFD()->setLinearSolver(method);
	
    Py_RETURN_NONE;
}

/* set the orientation for workspace hull calculation */
static PyObject* WireRobot_setOrientation(PyObject *self, PyObject *args)
{
	double a,b,c;

	if ( ! PyArg_ParseTuple(args, "ddd",&a,&b,&c) )
        return NULL;

	Matrix3d R = Matrix3d::ZRotationMatrix3d(c)*Matrix3d::YRotationMatrix3d(b)*Matrix3d::XRotationMatrix3d(a);
	pyHelperHull()->setOrientation(R);

    Py_RETURN_NONE;
}

/* set the orientation for workspace hull calculation */
static PyObject* WireRobot_addOrientation(PyObject *self, PyObject *args)
{
	double a,b,c;

	if ( ! PyArg_ParseTuple(args, "ddd",&a,&b,&c) )
        return NULL;

	Matrix3d R = Matrix3d::ZRotationMatrix3d(c)*Matrix3d::YRotationMatrix3d(b)*Matrix3d::XRotationMatrix3d(a);
	pyHelperHull()->addOrientation(R);

	Py_RETURN_NONE;
}

/* set the orientation for workspace hull calculation */
static PyObject* WireRobot_createOrientationSet(PyObject *self, PyObject *args)
{
	double a=0,b=0,c=0;
	int steps=10;

	if ( ! PyArg_ParseTuple(args, "|dddi",&a,&b,&c,&steps) )
        return NULL;

	pyHelperHull()->createOrientationSet(a,b,c,steps);
	
    Py_RETURN_NONE;
}

/* */
static PyObject* WireRobot_getOrienationWorkspaceCoverage(PyObject *self, PyObject *args)
{
	double x=0,y=0,z=0;

	if ( ! PyArg_ParseTuple(args, "ddd",&x,&y,&z) )
        return NULL;

    return Py_BuildValue("d", pyHelperGrid()->getOrienationWorkspaceCoverage(Vector3d(x,y,z)));
}

/* set the orientation for workspace hull calculation */
static PyObject* WireRobot_createRandomOrientationSet(PyObject *self, PyObject *args)
{
	int count;
	double a=0,b=0,c=0, max_angle=2*MO_PI;	// Bryant angles of the reference orienation
	if ( ! PyArg_ParseTuple(args, "i|dddd",&count, &a, &b, &c, &max_angle) )
        return NULL;

	Matrix3d R;
	getMatrixFromXYZ(R,Vector3d(a,b,c));
	pyHelperGrid()->createRandomOrientationSet(count, R, max_angle);
	pyHelperHull()->createRandomOrientationSet(count, R, max_angle);

    Py_RETURN_NONE;
}


/* set the orientation for workspace hull calculation */
static PyObject* WireRobot_setOrientationRequirement(PyObject *self, PyObject *args)
{
	int selector;

	if ( ! PyArg_ParseTuple(args, "i",&selector) )
        return NULL;

	pyHelperHull()->setOrientationRequirement(selector!=0);
	
    Py_RETURN_NONE;
}

/* set the method for force calculation and evaluation*/
static PyObject* WireRobot_setWorkspaceCriterion(PyObject *self, PyObject *args)
{
	int iWorkspaceCriterion=0;

	if ( ! PyArg_ParseTuple(args, "|i",&iWorkspaceCriterion) )
        return NULL;

	pyHelperHull()->setWorkspaceCriterion((PCRL::CWorkspaceAlgorithm::eWorkspaceCriterion)iWorkspaceCriterion);

	Py_RETURN_NONE;
}

/* clip the workspcace by the given bounding box */
static PyObject* WireRobot_clipByBoundingBox(PyObject *self, PyObject *args)
{
	Vector3d minBB,maxBB;

	if ( ! PyArg_ParseTuple(args, "dddddd",&minBB[0],&minBB[1],&minBB[2],&maxBB[0],&maxBB[1],&maxBB[2]) )
        return NULL;

	pyHelperHull()->clipByBoundingBox(minBB,maxBB);
	
    Py_RETURN_NONE;
}

/* synchronize the state between the workspace algorithm objects */
PYHELPER_VOID_VOID(WireRobot_synchronizeSettings, getRobotDocument()->synchronizeAlgorithmSettings() )
/*static PyObject* WireRobot_synchronizeSettings(PyObject *self, PyObject *args)
{
	getRobotDocument()->synchronizeAlgorithmSettings();

	Py_RETURN_NONE;
}*/

/* create the Grid in the object WSGrid */
static PyObject* WireRobot_createGrid(PyObject *self, PyObject *args)
{
	double minx, miny, minz, maxx, maxy, maxz, epsx=0, epsy=0, epsz=0;

	if ( ! PyArg_ParseTuple(args, "dddddd|ddd",&minx, &miny, &minz, &maxx, &maxy, &maxz, &epsx, &epsy, &epsz) )
        return NULL;

    if ( getRobotDocument()->WSGrid.makeGrid(Vector3d(minx, miny, minz), Vector3d(maxx, maxy, maxz), Vector3d(epsx, epsy, epsz)) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* create the Grid in the object WSGrid */
static PyObject* WireRobot_createRandomGrid(PyObject *self, PyObject *args)
{
	double minx, miny, minz, maxx, maxy, maxz;
	int i;

	if ( ! PyArg_ParseTuple(args, "ddddddi",&minx, &miny, &minz, &maxx, &maxy, &maxz, &i) )
        return NULL;

    if ( getRobotDocument()->WSGrid.makeRandomGrid(Vector3d(minx, miny, minz), Vector3d(maxx, maxy, maxz), i) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* create the Grid in the object WSGrid */
static PyObject* WireRobot_createCylinderGrid(PyObject *self, PyObject *args)
{
	Vector3d min,max;
	char normal='z';
	int layerSteps=10, radialSteps=10, angularSteps=36;

	if ( ! PyArg_ParseTuple(args, "dddddd|ciii",&min.x(), &min.y(), &min.z(), &max.x(), &max.y(), &max.z(), &normal, &layerSteps, &radialSteps, &angularSteps) )
        return NULL;

    if ( getRobotDocument()->WSGrid.makeCylinderGrid(min,max,normal,layerSteps, radialSteps, angularSteps) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* start the calculation in the object WSGrid */
PYHELPER_BOOL_VOID( WireRobot_evaluateGrid, getRobotDocument()->WSGrid.calculateWorkspace() )
/*static PyObject* WireRobot_evaluateGrid(PyObject *self, PyObject *args)
{
	getRobotDocument()->WSGrid.calculateWorkspace();
	return pyTrue();
}*/

/* start the calculation of the inner box based on WSGrid */
PYHELPER_BOOL_VOID( WireRobot_calculateInnerBox, getRobotDocument()->WSGrid.calculateInnerBox() )
/*static PyObject* WireRobot_calculateInnerBox(PyObject *self, PyObject *args)
{
	if ( getRobotDocument()->WSGrid.calculateInnerBox() )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}*/

/* activate/ deactivate the check of internal points during the determination of the inner box*/
static PyObject* WireRobot_checkInternalPoints(PyObject *self, PyObject *args)
{
	bool bInternalPoints=true;

	if ( ! PyArg_ParseTuple(args, "b",&bInternalPoints) )
        return NULL;

	getRobotDocument()->WSGrid.setCheckInternalPoints(bInternalPoints);

    Py_RETURN_NONE;
}

/* start the calculation in the object WSGrid */
static PyObject* WireRobot_getGridCoverage(PyObject *self, PyObject *args)
{
	return Py_BuildValue("d", getRobotDocument()->WSGrid.getCoverage());
}

/* Calculate the hyperplanes for a given pose */
static PyObject* WireRobot_calculateHyperplanes(PyObject *self, PyObject *args)
{
	double x,y,z;

	if ( ! PyArg_ParseTuple(args, "ddd",&x,&y,&z) )
        return NULL;

	if ( getRobotDocument()->WrenchSet.getHyperPlaneRepresentation(Vector3d(x,y,z), MatrixXd::Identity(3,3)) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* Check if a single wrench is feasible */
static PyObject* WireRobot_checkWrenchPoint(PyObject *self, PyObject *args)
{
	double fx,fy,fz, taux=0, tauy=0, tauz=0;

	if ( ! PyArg_ParseTuple(args, "ddd|ddd",&fx,&fy,&fz,&taux,&tauy,&tauz) )
        return NULL;

	MatrixXd w(6,1);
	w(0)=fx;w(1)=fy;w(2)=fz;w(3)=taux;w(4)=tauy;w(5)=tauz;

	if ( getRobotDocument()->WrenchSet.checkPoint(w) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* Check if an ellipsoidal wrench is feasible */
static PyObject* WireRobot_checkWrenchEllipsoid(PyObject *self, PyObject *args)
{
	double fx,fy,fz, taux=0, tauy=0, tauz=0;

	if ( ! PyArg_ParseTuple(args, "ddd|ddd",&fx,&fy,&fz,&taux,&tauy,&tauz) )
        return NULL;

	MatrixXd w(6,1);
	w(0)=fx;w(1)=fy;w(2)=fz;w(3)=taux;w(4)=tauy;w(5)=tauz;

	if ( getRobotDocument()->WrenchSet.checkEllipsoid(w) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/*! calculate cables' velocity for a given pose
 *	\return 1, if bigger than max cables velocity
 *			0, otherwise
 */
static PyObject* WireRobot_calculateCablesVelocity(PyObject *self, PyObject *args)
{
	double x, y, z, a=0, b=0, c=0;
	if(!PyArg_ParseTuple(args, "ddd|ddd", &x, &y, &z, &a, &b, &c))
        return NULL;

	Matrix3d R;
	PCRL::getMatrixFromXYZ(R,Vector3d(a,b,c));

	return Py_BuildValue("i", getRobotDocument()->VelocitySet.testPose(Vector3d(x,y,z),R)?1:0);
}

/*  */
static PyObject* WireRobot_getMaxCablesVelocityBox(PyObject *self, PyObject *args)
{
	double minx, miny, minz, maxx, maxy, maxz, epsx=0.25, epsy=0.25, epsz=0.25;
	if(!PyArg_ParseTuple(args, "dddddd|ddd",&minx, &miny, &minz, &maxx, &maxy, &maxz, &epsx, &epsy, &epsz))
		return NULL;

	return Py_BuildValue("d", getRobotDocument()->VelocitySet.getMaxCablesVelocityBox(Vector3d(minx, miny, minz),Vector3d(maxx, maxy, maxz),Vector3d(epsx,epsy,epsz)));
}

/*  */
static PyObject* WireRobot_setVelocitySet(PyObject *self, PyObject *args)
{
	MatrixXd vs(1,getRobotDocument()->getDof());
	vs.setZero();
	if(!PyArg_ParseTuple(args, "dd|dddd", &vs(0,0),&vs(0,1), &vs(0,2), &vs(0,3), &vs(0,4), &vs(0,5)))
        return NULL;

	getRobotDocument()->VelocitySet.setVelocitySet(vs);
	
	Py_RETURN_TRUE;

}

/*  */
static PyObject* WireRobot_getVelocitySet(PyObject *self, PyObject *args)
{
	MatrixXd vs(1,getRobotDocument()->getDof());
	getRobotDocument()->VelocitySet.getVelocitySet(vs);
	return pyToList(vs);
}

/*  */
static PyObject* WireRobot_getCablesVelocity(PyObject *self, PyObject *args)
{
	Eigen::MatrixXd velocity;
	getRobotDocument()->VelocitySet.getCablesVelocity(velocity);
	return pyToList(velocity);
}

PYHELPER_BOOL_STRING( WireRobot_loadObstacle, pyHelperInterference()->loadObstacle )
/*static PyObject* WireRobot_loadObstacle(PyObject *self, PyObject *args)
{
	char* filename;

	if( ! PyArg_ParseTuple(args, "s",&filename) )
        return NULL;

	if ( pyHelperInterference()->loadObstacle(filename) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}*/

PYHELPER_BOOL_VOID( WireRobot_calculateIntersectionObstacleCables, pyHelperInterference()->calculateCollisions() )
PYHELPER_BOOL_VOID( WireRobot_calculateInterference, pyHelperInterference()->calculateInterference() )

/*static PyObject* WireRobot_calculateIntersectionObstacleCables(PyObject *self, PyObject *args)
{
	if ( pyHelperInterference()->calculateCollisions() )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

static PyObject* WireRobot_calculateInterference(PyObject *self, PyObject *args)
{
	if ( pyHelperInterference()->calculateInterference() )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}*/

/*  */


/* checkPoseForCableCableCollision */
static PyObject* WireRobot_checkPoseForCableCableCollision(PyObject *self, PyObject *args)
{
  Vector3d c,r;
  r << 0, 0, 0;
  c << 0, 0, 0;
  Matrix3d R=Matrix3d::ZRotationMatrix3d(0.0);
  double edgeDistance = 0, normalDistance = 0;

	if ( ! PyArg_ParseTuple(args, "ddddddddddddddd", &c(0), &c(1), &c(2), &r(0), &r(1), &r(2), &R(0,0), &R(0,1), &R(0,2), &R(1,0), &R(1,1), &R(1,2), &R(2,0), &R(2,1), &R(2,2)))
      return NULL;

	if (pyHelperInterference()->checkPoseForCableCableCollision(c, r, R))
	{
		Py_RETURN_TRUE;
	}

	Py_RETURN_FALSE;
}


static PyObject* WireRobot_getInterferenceMatrix(PyObject *self, PyObject *args)
{
	MatrixXd mat;
	if ( pyHelperInterference()->getInterferenceMatrix(mat) ) 
		return pyToList(mat);

    Py_RETURN_FALSE;
}

PYHELPER_BOOL_VOID( WireRobot_calculateWorkspaceWrenchClosure, pyHelperFD()->calculateWorkspaceWrenchClosure() )
/*static PyObject* WireRobot_calculateWorkspaceWrenchClosure(PyObject *self, PyObject *args)
{
	if ( pyHelperFD()->calculateWorkspaceWrenchClosure()) 
		Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}*/

/*!
 *  Bloat the hull by the value given
 */
static PyObject* WireRobot_bloatHullBy(PyObject *self, PyObject *args)
{
	double deltaLength;
	if ( ! PyArg_ParseTuple(args, "d", &deltaLength) )
		return NULL;

	if ( pyHelperHull()->bloatHullBy(deltaLength) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* get a test list */
static PyObject* WireRobot_getBoundingBox(PyObject *self, PyObject *args)
{
	return pyToList(pyHelperHull()->bbMin,pyHelperHull()->bbMax);
}

/* return the current setting for the projection center */
static PyObject* WireRobot_getProjectionCenter(PyObject *self, PyObject *args)
{
	Vector3d c;
	pyHelperHull()->getProjectionCenter(c);

	return pyToList(c);
}

/* return the current setting for the iteration depth used in workspace computation */
static PyObject* WireRobot_getIterations(PyObject *self, PyObject *args)
{
	return Py_BuildValue("i",pyHelperHull()->getIterations());
}

/* return the current setting for the accuracy of the line search */
static PyObject* WireRobot_getEps(PyObject *self, PyObject *args)
{
	return Py_BuildValue("d",pyHelperHull()->getEps());
}

/* return the current setting for the accuracy of the line search */
static PyObject* WireRobot_getCoI(PyObject *self, PyObject *args)
{
	return pyToList(pyHelperHull()->CoI);
}

PYHELPER_BOOL_VOID( WireRobot_calculateCableSpan, pyHelperHull()->calculateCableSpan() )
/*static PyObject* WireRobot_calculateCableSpan(PyObject *self, PyObject *args)
{
	if ( pyHelperHull()->calculateCableSpan() )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}*/

// Create method table and module help
//------------------------------------------------------------------------

static PyObject* WireRobot_HelpIws(PyObject *self, PyObject *args);

PyMethodDef EmbMethodsIws[] = {
    {"help", WireRobot_HelpIws, METH_NOARGS,  "Print the names of all methods in Iws [void]"},
    
	{"clearShapes", WireRobot_clearShapes, METH_NOARGS, "delete currently visualized shapes [void]"},
	{"setEtaMax", WireRobot_SetEtaMax, METH_VARARGS,  "set size of orientation set [double]"},
	{"getMinInterferenceAngle", WireRobot_getMinInterferenceAngle, METH_NOARGS,  "get minimal size of the orientation set for which cable-cable collisions can occur everywhere [void]"},
	{"updateClippingPlanes", WireRobot_updateClippingPlanes, METH_NOARGS,  "update the clipping planes and the Trianglelength parameter according to the robot geometry [void]"},
	{"writeRobotGeometryToCSV", WireRobot_writeRobotGeometryToCSV, METH_VARARGS,  "write robot geometry into .csv file [filename]"},

	{"setStepsCircle", WireRobot_setStepsCircle, METH_VARARGS,  "set discretization parameter for cable-cable interference [setStepsCircle]"},
	{"setDeltaEta", WireRobot_setDeltaEta, METH_VARARGS,  "set discretization parameter for cable-cable interference [setDeltaEta]"},
	{"setDeltaAlpha", WireRobot_setDeltaAlpha, METH_VARARGS,  "set discretization parameter for cable-cable interference [setDeltaAlpha]"},
	{"setupCableCableInterferenceSets", WireRobot_setupCableCableInterferenceSets, METH_NOARGS, "compute points of cable-cable collision sets [void]"},
	
	{"visualizeInterferenceCableCable", WireRobot_visualizeInterferenceCableCable, METH_NOARGS, "execute all involved methods to visualize cable-cable interference sets [void]"},
	{"visualizeInterferenceCablePlatform", WireRobot_visualizeInterferenceCablePlatform, METH_VARARGS, "execute all involved methods to visualize cable-platform collision free workspace [filename]"},
	
	{"calculatePrintShadowData", WireRobot_calculatePrintShadowData, METH_NOARGS, "calculate shadow centers and shadow scaling factors [void]"},
	{"visualizeInterferenceCablePrintParallelogram", WireRobot_visualizeInterferenceCablePrintParallelogram, METH_NOARGS, "execute all involved methods to visualize a printable parallelogram [void]"},
	{"visualizeInterferenceCablePrintEllipse", WireRobot_visualizeInterferenceCablePrintEllipse, METH_NOARGS, "execute all involved methods to visualize a printable superellipse [void]"},
	{"visualizeInterferenceCablePrintStarShape", WireRobot_visualizeInterferenceCablePrintStarShape, METH_NOARGS, "execute all involved methods to visualize a printable star-shaped polygon [void]"},
	{"visualizeInterferenceCablePrintEllipseMultipleLayers", WireRobot_visualizeInterferenceCablePrintEllipseMultipleLayers, METH_NOARGS, "execute all involved methods to visualize printable superellipses in the given height interval [void]"},
	{"visualizeInterferenceCablePrintParalellogramMultipleLayers", WireRobot_visualizeInterferenceCablePrintParalellogramMultipleLayers, METH_NOARGS, "visualize printable parallelograms in the given height interval [void]"},
	{"calculateInterferenceCablePrintParalellogramMultipleLayers", WireRobot_calculateInterferenceCablePrintParalellogramMultipleLayers, METH_NOARGS, "calculate printable parallelograms in the given height interval [void]"},
	
	{"setNumberOfPrintLayers", WireRobot_setNumberOfPrintLayers, METH_VARARGS,  "set numberofprintlayers parameter for selecting the number of layers in the printing height interval [setNumberOfPrintLayers]"},
	{"setPrintParaVisNumber", WireRobot_setPrintParaVisNumber, METH_VARARGS,  "set printParaVisNumber parameter for selecting visualized printable parallelogram [printParaVisNumber]"},
	{"setPrintDir", WireRobot_setPrintDir, METH_VARARGS,  "set direction vectors for printing parallelograms and superellipses [d1(0), d1(1), d2(0), d2(1)]"},
	{"setPrintCenter", WireRobot_setPrintCenter, METH_VARARGS,  "set center location for printing parallelograms and superellipses [center(0), center(1), center(2)]"},
	{"setEllipseExp", WireRobot_setEllipseExp, METH_VARARGS,  "set exponent parameter for printing superellipses [exponent]"},
	{"setPrintHeight", WireRobot_setPrintHeight, METH_VARARGS,  "set printheight parameter for printing [PrintHeight]"},

	{"setRefinement", WireRobot_setRefinement, METH_VARARGS,  "set refinement parameter for cable-platform ray data structure [refinement]"},
	{"setSearchSteps", WireRobot_setSearchSteps, METH_VARARGS,  "set searchsteps parameter for cable-platform calculation [searchsteps]"},
	{"setCableCableSetting", WireRobot_setCableCableSetting, METH_VARARGS,  "set cable-cable display setting [CableCableSetting]"},
	{"setPrintDisplayShadows", WireRobot_setPrintDisplayShadows, METH_VARARGS,  "set cable-print display setting [PrintDisplayShadows]"},

	{"setPlatformCenter", WireRobot_setPlatformCenter, METH_VARARGS,  "set center location for cable-platform calculation [center(0), center(1), center(2)]"},

	{"calculateCableCableInterferenceSetsHulls", WireRobot_calculateCableCableInterferenceSetsHulls, METH_NOARGS,  "calculate convex hulls for all cable-cable collision sets [void]"},
	//{"setupCableCableInterferenceHullsTriangles", WireRobot_setupCableCableInterferenceHullsTriangles, METH_NOARGS,  "setupCableCableInterferenceHullsTriangles"},
	{"writeCableCableSetsToCSV", WireRobot_writeCableCableSetsToCSV, METH_VARARGS,  "write points of cable-cable collision sets into .csv file [filename]"},

	{"loadplatformstldata", WireRobot_loadplatformstldata, METH_VARARGS,  "load platform geometry in .stl format [filename]"},
	{"setupCablePlatformCollisionCones", WireRobot_setupCablePlatformCollisionCones, METH_NOARGS,  "compute collision cones at distal anchorpoints that encapsulate the platform [void]"},
	{"newCablePlatformRayBall", WireRobot_newCablePlatformRayBall, METH_NOARGS,  "setup data structure to approximate the set of collision free platform positions [void] "},
	{"calculateCablePlatform", WireRobot_calculateCablePlatform, METH_NOARGS,  "compute set of platform positions without cable-platform collisions around center [void]"},
	{"writeCablePlatformPlatformToCSV", WireRobot_writeCablePlatformPlatformToCSV, METH_VARARGS,  "write platform geometry data into .csv file [filename]"},
	{"writeCablePlatformConesToCSV", WireRobot_writeCablePlatformConesToCSV, METH_VARARGS,  "write collision cone data into .csv file [filename]"},
	{"writeCablePlatformTrianglelistToCSV", WireRobot_writeCablePlatformTrianglelistToCSV, METH_VARARGS,  "write triangulation indices of cable-platform data structure into .csv file [filename]"},
	{"writeCablePlatformRaysToCSV", WireRobot_writeCablePlatformRaysToCSV, METH_VARARGS,  "write ray vectors of cable-platform data structure into .csv file [filename]"},

	{"setPn", WireRobot_setPn, METH_VARARGS,  "set vector of printing nozzle in the platform coordinate frame [pn(0), pn(1), pn(2)]"},
	{"calculateInterferenceWithPrintParallelogram", WireRobot_calculateInterferenceWithPrintParallelogram, METH_NOARGS,  "calculate list of printable parallelograms [void]"},
	{"calculateInterferenceWithPrintEllipse", WireRobot_calculateInterferenceWithPrintEllipse, METH_NOARGS,  "calculate largest printable superellipse [void]"},
	{"calculateInterferenceWithPrintStarShape", WireRobot_calculateInterferenceWithPrintStarShape, METH_NOARGS,  "calculate largest printable star-shaped polygon [void]"},
	{"writeParallelogramToCSV", WireRobot_writeParallelogramToCSV, METH_VARARGS,  "write data of listed printable parallelograms into .csv file [filename]"},
	{"writeParallelogramMultipleLayerToCSV", WireRobot_writeParallelogramMultipleLayerToCSV, METH_VARARGS,  "write data of listed printable parallelograms in multiple layers into .csv file [filename]"},
	{"writeEllipseToCSV", WireRobot_writeEllipseToCSV, METH_VARARGS,  "write data of largest printable superellipse into .csv file [filename]"},
	{"writeStarShapeToCSV", WireRobot_writeStarShapeToCSV, METH_VARARGS,  "write data of largest printable star-shaped polygon into .csv file [filename]"},
	{"checkInterferenceWithPrintEllipseMultLayerCriteria", WireRobot_checkInterferenceWithPrintEllipseMultLayerCriteria, METH_NOARGS,  "check if superellipses in the height interval (center(2), center(2)+printheight) can be calculated independently  [void]"},
	{"checkInterferenceWithPrintParaMultLayerCriteria", WireRobot_checkInterferenceWithPrintParaMultLayerCriteria, METH_NOARGS,  "check if parallelograms in the height interval (center(2), center(2)+printheight) can be calculated independently  [void]"},
    
    {"setBoundingBox", WireRobot_setBoundingBox, METH_VARARGS,  "set the bounding box for the workspace calculation [x_min, y_min, z_min, x_max, y_max, z_max]"},
    {"setConditionMin", WireRobot_SetConditionMin, METH_VARARGS,  "set the minimum condition number of acceptance of workspace [double]"},
    {"setEps", WireRobot_SetEps, METH_VARARGS,  "set accuracy for line search [double]"},
    {"setEpsGeometry", WireRobot_SetEpsGeometry, METH_VARARGS,  "set delta for computed discrete differences in differential hull [double]"},
    {"setForceLimits", WireRobot_SetForceLimits, METH_VARARGS,  "Set the minimum and maximum force for the cables [min, max]"},
    {"setIterations", WireRobot_SetIteration, METH_VARARGS,  "set the iteration depth for recursive subdivision [int<10]"},
    {"setLinearSolver", WireRobot_setLinearSolver, METH_VARARGS,  "select the solver used to check the workspace criterion [lu=0, llt=1, ldlt=2, householerQr=3, jacobiSvd=4, explicitPseudoinverse=5]"},
    {"setMethod", WireRobot_setMethod, METH_VARARGS,  "define the calculation method and criteria for force distributions; controls method for workspace calculation [METHOD 0 (default):closedForm; 1:bruteForce; Criteria 0 (default):feasible; 1:euclidean; 2:infinity;"},
    {"setReferenceForceQP", WireRobot_setReferenceForceQP, METH_VARARGS,  "set the reference force used in QP algorithm to control the tension level. reasonable intervall is from f_min to f_max [double]"},
    {"setQPparameters", WireRobot_setQPparameters, METH_VARARGS,  "set the parameters of QP algorithm, -1 = default value [int max_iters, double resid_tol, double eps, int refine_steps, double kkt_reg]"},
	{"setOrientation", WireRobot_setOrientation, METH_VARARGS,  "set the orientation (a ,b, c) R = Rz(c)*Ry(b)*Rx(a) (deletes existing) [a, b, c]"},
    {"setOrientationRequirement", WireRobot_setOrientationRequirement, METH_VARARGS,  "select if at least one or all orientations in the set needs to be satisfied in workspace calculation [Single Orientation:0, Else all]"},
    {"setProjectionCenter", WireRobot_SetProjectionCenter, METH_VARARGS,  "set the center for the projection in workspace calculation [x, y, z]"}, 
    {"setSearchRange", WireRobot_SetSearchRange, METH_VARARGS,  "set the maximum range for line search [double]"},
    {"setUsePulleyForStructureMatrix", WireRobot_setUsePulleyForStructureMatrix, METH_VARARGS,  "if set to true, the influence of guideing pulleys is taken into account for the computation of the structure matrix [bool use]"},
	{"setVelocitySet", WireRobot_setVelocitySet, METH_VARARGS,  "set velocity set [vs(1), vs(2), ..., vs(dof)]"},
    {"setWrench", WireRobot_setWrench, METH_VARARGS,  "set the applied wrench [forces x, y, z, and torque x, y, z]"},
    {"setWorkspaceCriterion", WireRobot_setWorkspaceCriterion, METH_VARARGS,  "set the workspace criteria [0=forceFeasible, 1=wireLength, 2=boundingBox, 3=wrenchSet]"},
    
    {"getBoundingBox", WireRobot_getBoundingBox, METH_VARARGS,  "return a list with the axis aligned bounding box of the workspace. returns a list with six values [void]"},
	{"getCablesVelocity", WireRobot_getCablesVelocity, METH_NOARGS,  "get cables' velocity [void]"},
    {"getCalculationTime", WireRobot_getCalculationTime, METH_NOARGS,  "get the computation time for the most recent workspace calculation [void]"},
    {"getEps", WireRobot_getEps, METH_NOARGS,  "get the accuracy eps for the line search used for workspace hull computation [void]"},
    {"getForceLimits", WireRobot_GetForceLimits, METH_NOARGS,  "Get the minimum and maximum force for the cables as list with two element [void]"},
    {"getGridCoverage", WireRobot_getGridCoverage, METH_NOARGS,  "return the percentage in the range between 0 and 1 of the vertices in the grid that belong to the workspace [void]"},
    {"getOrienationWorkspaceCoverage", WireRobot_getOrienationWorkspaceCoverage, METH_VARARGS,  "return the percentage in the orientation workspace at the position (x,y,z) in the current orientation workspace belonging the the workspace [x,y,z]"},
    {"getIterations", WireRobot_getIterations, METH_NOARGS,  "get the current number of iterations used in line search [void]"},
    {"getInterferenceMatrix", WireRobot_getInterferenceMatrix, METH_NOARGS,  "return the interference matrix with is 9xn containint triangles with the interference region [void]"},  
	{"getMaxCablesVelocityBox", WireRobot_getMaxCablesVelocityBox, METH_VARARGS,  "get max required cables' velocity in a box [min.x, min.y, min.z, max.x, max.y, max.z, eps.x, eps.y, eps.z]"},
    {"getProjectionCenter", WireRobot_getProjectionCenter, METH_NOARGS,  "get the center for the projection in workspace calculation and return a list with x, y, z value [void]"},
	{"getVelocitySet", WireRobot_getVelocitySet, METH_VARARGS,  "get velocity set [void]"},
    {"getWorkspaceCoI", WireRobot_getEps, METH_NOARGS,  "get the center of inertia in x,y,z coordinates of the volume occupied by the workspace hull [void]"},
    {"getWorkspaceCrosssectionMatrix", WireRobot_getWorkspaceCrosssectionMatrix, METH_NOARGS,  "return a 6 by n matrix (a list of lists) containing in each column begin and end of line segments of the cross section [void]"},
	{"getCableSpanMatrix", WireRobot_getCableSpanMatrix, METH_NOARGS,  "return a segments by n matrix (a list of lists) containing in each the column number of segments angels of the cable span. These angles are evenly distributed on a 360 circle around the local frame of A_i [void]"},
    {"getCableSpanTriangulation", WireRobot_getCableSpanTriangulation, METH_NOARGS,  "return a 3 by (segments*n) matrix (a list of lists) containing n blocks with each number of segments position vectors with the cartesian cable span. Together with the respecitve values of a_i, the generalized cone of the cable span is defined [void]"},
    {"getWorkspaceCrosssectionMatrix", WireRobot_getWorkspaceCrosssectionMatrix, METH_NOARGS,  "return a 6 by n matrix (a list of lists) containing in each column begin and end of line segments of the cross section [void]"},

    {"getWorkspaceGeometry", WireRobot_getWorkpaceGeometry, METH_NOARGS, " Returns the geometry data of the workspace [vertices, triangles, number of triangles]"},
    {"getWorkspaceSurface", WireRobot_getWorkspaceSurface, METH_VARARGS,  "get the surface of the workspace [void]"},
    {"getWorkspaceVolume", WireRobot_getWorkspaceVolume, METH_VARARGS,  "get the volume of the workspace [void]"},
    
    {"addOrientation", WireRobot_addOrientation, METH_VARARGS,  "add one orientation [a, b, c] with R = Rz(c)*Ry(b)*Rx(a) to the list [a, b, c]"},   
    {"bloatHullBy", WireRobot_bloatHullBy, METH_VARARGS, "Bloat the hull by a given delta length in meter away from the hull center [delta]"},
    
    {"calcDisabledWspc", WireRobot_CalcDisabledWspc, METH_VARARGS,  "Calculate disabled workspace for a box [Min.x, Min.y, Min.z, Max.x, Max.y, Max.z]"},
    {"calcHyperplanes", WireRobot_calculateHyperplanes, METH_VARARGS,  "get the Hyperplane representation for a pose [r.x, r.y, r.z]"}, 
    {"calculateCableSpan", WireRobot_calculateCableSpan, METH_NOARGS, "calculate the wirespan [void]"},
    {"calculateDifferentialWorkspace", WireRobot_calculateDifferentialWorkspace, METH_NOARGS,  "calculate the differential variations of the workspace [void]"},
    {"calculateDisabledWspc", WireRobot_CalcDisabledWspc, METH_VARARGS,  "Calculate disabled workspace for a box [Min.x, Min.y, Min.z, Max.x, Max.y, Max.z]"},
    {"calculateHyperplanes", WireRobot_calculateHyperplanes, METH_VARARGS,  "get the Hyperplane representation for a pose [r.x, r.y, r.z]"}, 
    {"calculateInnerBox", WireRobot_calculateInnerBox, METH_NOARGS,  "calculate the biggest box inside the workspace [void]"},
    {"calculateIntersectionObstacleCables", WireRobot_calculateIntersectionObstacleCables, METH_NOARGS,  "calculate the intersection of the cables with an obstacle based on a regular grid [void]"},  
    {"calculateInterference", WireRobot_calculateInterference, METH_NOARGS,  "calculate global cable-cable interference for constant platform orientation and store the interference regions internally [void]"}, 

	{"checkPoseForCableCableCollision", WireRobot_checkPoseForCableCableCollision, METH_VARARGS,  "Check if a pose is behind or in front of a collision plane with respect to an initial position."}, 

    {"calculateWorkspaceWrenchClosure", WireRobot_calculateWorkspaceWrenchClosure, METH_NOARGS,  "calculate the coefficients of the wrench-closure workspace for the current robot [void]"},  
    {"calculateOptimalCone", WireRobot_CalculateOptimalCone, METH_VARARGS,  "Calculate optimal cone for a given workspace using a given base index. [ID, Axis Out, Aperture Out]"},
    {"calculateWorkspace", WireRobot_CalculateWorkspace, METH_NOARGS,  "Start the workspace calculation for the current robot object. [void]"},
    {"calculateWorkspaceCrosssection", WireRobot_CalculateWorkspaceCrosssection, METH_VARARGS,  "Calculate a cross-section of the workspace. [x/y/z]"},
    {"calculateWorkspaceProperties", WireRobot_CalculateWorkspaceProperties, METH_NOARGS,  "Calculate workspace properties like volume etc. and return the results in a dictionary [void]"},
    {"checkInternalPointsForInnerBox", WireRobot_checkInternalPoints, METH_VARARGS,  "enable the check of internal points during the determination of the inner box [bool]"},
    {"checkWrenchPoint", WireRobot_checkWrenchPoint, METH_VARARGS,  "check, if one wrench is feasible for the precalculated hyperplane representation [f.x, f.y, f.z, | tau.x, tau.y, tau.z]"},
    {"checkWrenchEllipsoid", WireRobot_checkWrenchEllipsoid, METH_VARARGS,  "check, if a ellipsoidal wrench is feasible for the precalculated hyperplane representation [f.x, f.y, f.z, | tau.x, tau.y, tau.z]"},
    {"createGrid", WireRobot_createGrid, METH_VARARGS,  "create a regular grid for use in evaluateGrid in order to analyze the workspace [min.x, min.y, min.z, max.x, max.y, max.z, eps.x, eps.y, eps.z]"},
    {"createCylinderGrid", WireRobot_createCylinderGrid, METH_VARARGS,  "create a concentric grid with cylinders for use in evaluateGrid in order to analyze the workspace [min.x, min.y, min.z, max.x, max.y, max.z, normal, layerSteps, radiualSteps, angularSteps]"},
    {"createOrientationSet", WireRobot_createOrientationSet, METH_VARARGS,  "generate a whole set of orientation (delta a, b, c) R = Rz(c)*Ry(b)*Rx(a) in defined nr. of steps [a, b, c, number of steps (default:10)]"},
    {"createRandomOrientationSet", WireRobot_createRandomOrientationSet, METH_VARARGS,  "generate a uniformly distributed random orientation set with a given number of orientations. If a,b,c and max_angle is given, only orientation around the reference orientation (a,b,c) with distance max_angle are generated [OrientationCount | a,b,c,max_anlge]"},

    {"createRandomGrid", WireRobot_createRandomGrid, METH_VARARGS,  "create a random grid for use in evaluateGrid in order to analyze the workspace inside the box given by the grid [min.x, min.y, min.z, max.x, max.y, max.z, number_of_gridpoints]"},
    {"clipByBoundingBox", WireRobot_clipByBoundingBox, METH_VARARGS,  "clip the bounding box for the workspace calculation [x_min, y_min, z_min, x_max, y_max, z_max]"},
    
    {"evaluateGrid", WireRobot_evaluateGrid, METH_NOARGS,  "check each vertex in the workspace grid [void]"},
    
    {"intersectWorkspace", WireRobot_IntersectWorkspace, METH_NOARGS,  "Intersect the current robot workspace with new parameters (perform boolean AND operation) [void]"},
    {"intersectWorkspaceWireLength", WireRobot_IntersectWorkspaceWireLength, METH_NOARGS,  "Intersect the current robot workspace for limitation in wire length [void]"},
    
    {"loadObstacle", WireRobot_loadObstacle, METH_VARARGS,  "load an obstacle for cable-obstacle interference from a STL file [filename]"},
    {"loadWorkspaceVerticesCSV", WireRobot_loadWorkspaceVerticesCSV, METH_VARARGS,  "load workspace vertices from CSV [filename, iteration depth]"},
    
    {"uniteWorkspace", WireRobot_UniteWorkspace, METH_NOARGS,  "Expand (unite) the current robot workspace with new parameters (perform boolean OR operation) [void]"},
    
    {"printDifferential", WireRobot_printDifferential, METH_NOARGS,  "print some results from the differential calculation [void]"},
    {"printForceDistribution", WireRobot_printForceDistribution, METH_VARARGS,  "calculate and print the force distribution for the given pose [x, y, z]"},
    {"printProperties", WireRobot_PrintProperties, METH_NOARGS,  "print the properties of the workspace on the screen [void]"},
    
    {"saveCableSpaceEnvelope", WireRobot_saveCableSpaceEnvelope, METH_VARARGS, "save the envelope occupied by the cables to a STL file [filename, cableselector]"},
    {"saveWorkspace", WireRobot_saveWorkspace, METH_VARARGS,  "save workspace to STL file [filename]"},
    {"saveWorkspaceCrosssection", WireRobot_SaveCrosssection, METH_VARARGS,  "save workspace cross-section to SVG file [filename]"},
    {"saveWorkspaceCrosssectionMatlab", WireRobot_saveWorkspaceCrosssectionMatlab, METH_VARARGS,  "save workspace cross-section to a matlab file such that the file generates a diagram of the workspace [filename]"},
    {"saveWorkspaceMatlab", WireRobot_saveWorkspaceMatlab, METH_VARARGS,  "save workspace to a .m file such that Matlab can generate a diagram from it [filename]"},
    {"saveWorkspaceVerticesCSV", WireRobot_saveWorkspaceVerticesCSV, METH_VARARGS,  "save workspace vertices to CSV [filename]"},
    {"synchronizeSettings", WireRobot_synchronizeSettings, METH_NOARGS,  "synchronize settings between workspace algorithm objects [void]"},
    
    {"verifyPoint", WireRobot_verifyPoint, METH_VARARGS,  "verify, if a point lies in the workspace using the currently set workspace evaluator [x, y, z]"},
    {"verifyWorkspaceBox", WireRobot_verifyWorkspaceBox, METH_VARARGS,  "verify if a given box fully belongs to the workspace [min.x, min.y, min.z, max.x, max.y, max.z, eps.x, eps.y, eps.z]"},
    
    {"wireForcesWorkspaceBoxDriver", WireRobot_wireForcesWorkspaceBoxDriver, METH_VARARGS,  "Calculate the minimum and maximum forces in each cable for a box [VECTOR.Min (3 doubles), VECTOR.Max (3 doubles), | eps.y, eps.x, eps.z]"},
	{"calculateCablesVelocity", WireRobot_calculateCablesVelocity, METH_VARARGS,  "calculate cables' velocity for a given pose [x, y, z, a, b, c], based on min and max velocity set"},

    {NULL, NULL, 0, NULL}
};


/* print the full method table of the Iws object */
static PyObject* WireRobot_HelpIws(PyObject *self, PyObject *args)
{
	for (PyMethodDef* ptr = EmbMethodsIws; ptr->ml_name!=0; ptr++)
        PySys_WriteStdout("Iws.%s: %s\n",ptr->ml_name,ptr->ml_doc);
	
	Py_RETURN_NONE;
}