/*
* WireX  -  IPAGL
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

/*! \file GLScene.h
 *
 *	\author   Andreas Pott
 *  
 *  \class CGLScene
 * 
 *  \brief CGLScene represents the drawing windows in a openGL application.
 *  \par The scene object includes all information on camera position, view perspective
 *  the handles to the openGL engine as well as all objects to be drawn with openGL.
 *  
 *  \par Usage
 *  The class CGLScene is the core class of the IPA openGL library. In order to extend 
 *  a MFC SDI application with a openGL view, one has to perform the following steps:
 *  
 *  - create a member variable m_Scene of type CGLScene in the view class CAppNameView 
 *    of the application
 *  
 *  - override the following member functions:
 *	    - void OnDestroy();
 *      - void OnLButtonDown(CPoint point); 
 *	    - bool OnMouseMove(CPoint point); 
 *	    - void OnLButtonUp(CPoint point); 
 * 	    - void OnRButtonDown(CPoint point); 
 *      - void OnRButtonUp(CPoint point);
 *      - BOOL OnEraseBkgnd(CDC* pDC);
 *    by adding a code like:
 * 
 *  void CAppNameView::OnSize(UINT nType, int cx, int cy)
 *	{
 *	    CView::OnSize(nType, cx, cy);
 *  
 *      // check for invalid size of the window
 *  	if ( 0 >= cx || 0 >= cy )
 *  		return;
 *  	// pass parameters to the openGL scene object
 *  	m_Scene.OnSize(cx,cy);
 *  }
 *  
 *  void CAppNameView::OnDestroy()
 *	{
 *		CView::OnDestroy();
 *  	m_Scene.OnDestroy();
 *  }
 *  
 *	void CAppNameView::OnLButtonDown(UINT nFlags, CPoint point) 
 *{
 *  	m_Scene.OnLButtonDown(point);
 *  	CView::OnLButtonDown(nFlags, point);
 *  }
 *  
 *  void CAppNameView::OnMouseMove(UINT nFlags, CPoint point) 
 *  {
 *		if (m_Scene.OnMouseMove(point))
 *  		InvalidateRect(FALSE);
 *		CView::OnMouseMove(nFlags, point);
 * 	}
 *  
 *  void CAppNameView::OnLButtonUp(UINT nFlags, CPoint point) 
 *{
 *		m_Scene.OnLButtonUp(point);
 *  	CView::OnLButtonUp(nFlags, point);
 *  }
 *  
 *  void CAppNameView::OnRButtonDown(UINT nFlags, CPoint point) 
 *  {
 *  	m_Scene.OnRButtonDown(point);
 *		CView::OnRButtonDown(nFlags, point);
 *  }
 *	
 * 	void CAppNameView::OnRButtonUp(UINT nFlags, CPoint point) 
 *  {
 *  	m_Scene.OnRButtonUp(point);
 * 		CView::OnRButtonUp(nFlags, point);
 *  }
 *
 *  - If these member functions were not added by the wizard, the following MESSAGE_MAP macros have to be added manually:
 *
 *	BEGIN_MESSAGE_MAP(CAppNameView, CView)
 *  	ON_WM_ERASEBKGND()
 *		ON_WM_CREATE()
 * 		ON_WM_DESTROY()
 *  	ON_WM_SIZE()
 *  	ON_WM_LBUTTONDOWN()
 * 		ON_WM_MOUSEMOVE()
 *  	ON_WM_LBUTTONUP()
 *		ON_WM_RBUTTONDOWN()
 *  	ON_WM_RBUTTONUP()
 *	END_MESSAGE_MAP()
 *
 *  - Add the call m_Scene.InitPerspective(1.5); to the constructor of CAppNameView
 *  - Add m_Scene.RenderScene(); at the beginning of the function CAppNameView::OnDraw(CDC*)
 *  - Add m_Scene.InitializeOpenGL(this); to the function int CAppNameView::OnCreate(LPCREATESTRUCT)
 */

#ifndef  CGLSCENE_H
#define  CGLSCENE_H

#include <map>
#include <vector>
#include <string>
#include "GLShape.h"
#include "GLMemoryContext.h"
#include <vfw.h>

using namespace std;

class CGLScene 
{
	static const double TrackBallSize;	//!< constants for the quaternion calculation of rotation

	double UpperBackgroundColor[4];		//!< the upper color of the background
	double LowerBackgroundColor[4];		//!< the lower color of the background

protected:
	//! the device context used by openGL for rendering
	CDC*	m_pDC;			//!< a MFC device context (DC)
	HGLRC	m_hRC;			//!< a handle to the opengGL rendering context (GLRC)
	//! state variables used for mouse motion tracking
	CPoint m_CurPos;		//!< actual mouse x,y coordiante for tracking the movement
	CPoint m_LastPos;		//!< last mouse x,y coordinate for tracking the movement
	bool m_bLIsDown;        //!< is left mouse button pressed
	bool m_bRIsDown;        //!< is right mouse button pressed
	// state variables used to define the current camera position
	double m_zO;			//!< initial distance camera/origin as determined by the scene
	double m_zZoom;			//!< actual distance due to zooming
	double m_fovy;			//!< the angle [in deg] of the viewing volume
	CVector3 m_Trans;		//!< actual x,y,z displacement of the viewpoint
	double matRot[4][4];    //!< the actual rotation matrix determined from the quaternion
    double cur_q0, last_q0; //!< actual and last scalar part of the quarternions of the view
	CVector3 cur_q, last_q; //!< actual and last vector (imaginary) part of the querternions
	// openGL settings
	double m_ClipZMin;      //!< minimal z clipping plane
	double m_ClipZMax;      //!< maximal z clipping plane
    bool m_PerspectiveProjection; //!< switch display mode between perspective and orthographic projections
	CVector3 r_max;			//!< used to determine the visible part of the scene
	int Width,Height;       //!< height and width of the GL window
	double aspect_ratio;	//!< width/height ratio of the window
	GLuint selectBuf[512];	//!< buffer for selection mode
	bool bRenderBackground;	//!< indicate if the scene objects generates a color gradient as background
	double m_fZoomSpeed;	//!< speed for zooming via movement of the mouse
	double m_fPanSpeed;		//!< speed for panning via movement of the mouse
    bool m_bSingleLightSetup; //!< true -> use single light source, false -> use 6 light sources at CGLScene::setOpenGLParameters()
    bool m_bFullSpecularReflectivity; //!< All materials hereafter have full specular reflectivity with a high shine

public:
	//! constructor and destructor
    CGLScene();
    ~CGLScene() ;  

	bool bIsDragging;		//!< flag that indicates if someone is currently dragging with the mouse
	bool bEnableRotation;	//!< flag that enables rotation with the mouse
	bool bEnableZoom;		//!< flag that enables zooming with the mouse
	bool bEnablePan;		//!< flag that enables panning with the mouse
	bool bEnableDragging;	//!< flag that enables dragging with the mouse
	bool bPanWithRightMouseButton; //!< flag that activates the alternative mode: pan with right mouse button, zoom with both mouse buttons

private:
    //! quaternion operations (for rotation of the view)
	void calculateQuaternion(double xalt, double yalt, double x, double y);
	//! calcuate the rotation matrix from the quaternions
	void buildRotationMatrix();
	//! utility function for scene rotation
	double projectToSphere(double r, double x, double y);
	//! set the pixel format descriptor to the default values
	BOOL SetupPixelFormat();
protected:
	//! Rendering Process
	void SetupSceneTransformation();
    //! set some openGL properties concerning the shading, lighting, etc. to default values
	void setOpenGLParameters();

public:
    //! Set the rotation matrix with a row major double[16] array.
    void setRotationMatrix(double *pMatrix);
    //! Get the rotation matrix (4x4 row major) - Content is copied and needs to be deleted by the caller
    double* getRotationMatrix() const;
	//! initial setup of the perspective
	void InitPerspective(double AspectRatio);
	//! reset the view parameter to default
	void ResetView();

	//! get the current perspective settings
	void getPerspectiveSettings(double& _clip_z_min, double& _clip_z_max, double& _field_of_view){
		_clip_z_min=m_ClipZMin;_clip_z_max=m_ClipZMax;_field_of_view=m_fovy;}
	//! set the current perspective settings
	void setPerspectiveSettings(const double _clip_z_min, const double _clip_z_max, const double _field_of_view){
		m_ClipZMin=_clip_z_min;m_ClipZMax=_clip_z_max;m_fovy=_field_of_view;}

	//! change Zoom
	void adjustZoom(double new_Zoom);
	//! adjust the current view parameters such that the frame K is visible
	void adjustView(CFrame* K);
	//! getViewPoint
	CVector3 getViewPoint() { return m_Trans; }
	//! setViewPoint
	void setViewPoint(CVector3 viewPoint) { m_Trans = viewPoint; }
    //! Get actual scalar part of the quaternions of the view
    double getQuaternionScalarQ0(){ return cur_q0; };
    //! Set actual scalar part of the quaternions of the view
    void setQuaternionScalarQ0(double q0){ cur_q0 = q0; };
    //! Get actual (imaginary) part of the querternions
    CVector3 getQuaternionImaginary(){ return cur_q; };
    //! Set actual (imaginary) part of the querternions
    void setQuaternionImaginary(CVector3 q){ cur_q = q; };
	//! draw the scene into the buffer and swap buffers
	virtual void RenderScene();
	//! draw the background of the scene
	void RenderBackground();
	//! add a user defined (and owned) object to the draw list
	CGLShape* addShape ( CGLShape& ) ;			
	//! set the background color for the drawing window in RGBA color model
	void setBackgroundColor(double r=0.0, double g=0.0, double b=0.0, double a=1.0) 
	  { UpperBackgroundColor[0]=r;UpperBackgroundColor[1]=g;UpperBackgroundColor[2]=b;UpperBackgroundColor[3]=a;
		LowerBackgroundColor[0]=r;LowerBackgroundColor[1]=g;LowerBackgroundColor[2]=b;LowerBackgroundColor[3]=a;}
	//! set the upper color for the background in RGBA color model
	void setUpperBackgroundColor(double r=0.8, double g=0.8, double b=1.0, double a=1.0) 
	  { UpperBackgroundColor[0]=r;UpperBackgroundColor[1]=g;UpperBackgroundColor[2]=b;UpperBackgroundColor[3]=a;}
	//! set the lower color for the background in RGBA color model
	void setLowerBackgroundColor(double r=0.1, double g=0.1, double b=1.0, double a=1.0)
	  { LowerBackgroundColor[0]=r;LowerBackgroundColor[1]=g;LowerBackgroundColor[2]=b;LowerBackgroundColor[3]=a;}
	//! set the flag for rendering the color gradient
	void setRenderBackground(bool flag) { bRenderBackground=flag; }
	//! get the flag for rendering the color gradient
	bool getRenderBackground() const { return bRenderBackground; }
	//! get the vector with the upper background colors as (red,green,blue,alpha)
	double* getUpperBackgroundColor(){return UpperBackgroundColor;}
	//! get the vector with the lower background colors as (red,green,blue,alpha)
	double* getLowerBackgroundColor(){return LowerBackgroundColor;}
	//! rotate the view such that is is perpenticualar to the XY plane
	void setXYPlane();
	//! rotate the view such that is is perpenticualar to the XZ plane
	void setXZPlane();
	//! rotate the view such that is is perpenticualar to the YZ plane
	void setYZPlane();
	//! switch between perspective (default) and orthogonal projection
	void setPerspectiveProjection(bool flag){m_PerspectiveProjection=flag;}
	//! get the perspective flag
	bool getPerspectiveProjection()const{return m_PerspectiveProjection;}
	//! set the zoom speed
	void setZoomSpeed(double fZoomSpeed) { m_fZoomSpeed = fZoomSpeed; }
	//! set the pan speed
	void setPanSpeed(double fPanSpeed) { m_fPanSpeed = fPanSpeed; }
	//! set the alternative mode: pan with right mouse button, zoom with both mouse buttons
	void setPanWithRightMouseButton(bool bActive) { bPanWithRightMouseButton = bActive; }

	//! initialize the openGL environment
	BOOL InitializeOpenGL(CWnd* Wnd);
	//! driver to be coupled with OnSize of the conncected view class
	void OnSize(int cx, int cy);
	//! destroy the drawing context of openGL
	void OnDestroy();
	//! handle mouse move and clicks to the openGL window for rotation, zooming and panning
	void OnLButtonDown(CPoint point); 
	//! handle mouse move and clicks to the openGL window for rotation, zooming and panning
	void OnLButtonUp(CPoint point); 
	//! handle mouse move and clicks to the openGL window for rotation, zooming and panning
	void OnRButtonDown(CPoint point); 
	//! handle mouse move and clicks to the openGL window for rotation, zooming and panning
	void OnRButtonUp(CPoint point); 
	//! handle mouse move and clicks to the openGL window for rotation, zooming and panning
	bool OnMouseMove(CPoint point);
	//! handle mouse move for rotation
	bool OnMouseMoveRotate(CPoint point);
	//! handle mouse move for panning
	bool OnMouseMovePan(CPoint point);
	//! handle mouse move for zooming
	bool OnMouseMoveZoom(CPoint point);
	//! pan the scene by a defined vector 
	void addTrans(CVector3 Trans);
	//! save the current scene to a BMP file
	void SaveBmp(const std::string& filename, const int& width=640, const int& height=480);
	//! get BMP with current scene
	void GetBmp(CGLMemoryContext& MC);
	//! save current scene to image file
	void SaveImage(const std::string& filename, int fileType);
	//! save the current scene to an EPS file
	void SaveEps(const std::string& filename);
	//! print the current parameter of the scene to console
	void printParam() const;
	//! low-level API to create animations in AVI files
	//! create a new avi 
	bool initAvi(const std::string& filename, int m_ResX=640, int m_ResY=480);
	//! save the current frame into the stream
	bool saveAviFrame();
	//! close the avi stream
	bool finalizeAvi(bool ok=true);

    void setSingleLightMode(bool bSingleLight){ m_bSingleLightSetup = bSingleLight; };
    void setFullSpecularReflectivity(bool bFull){ m_bFullSpecularReflectivity = bFull; };

private:
	// local state variables for avi generation
	CGLMemoryContext *pMC;
	PAVISTREAM ps, psCompressed;
	PAVIFILE pfile;
	int iFrame;
    // Set during a call to setRotationMatrix - Used in MouseMove
    bool m_bRotationMatrixChanged;
};

/*! \class CGLSceneGraph
 *  the new class CGLSceneGraph is derived from CGLScene and present the new 
 *  tree like data structure as well as the operations on the scene graph to 
 *  the application.
 * 
 *  This class extends the concept of a CGLScene with an advanced API to create 
 *  a complex, tree-like data structure of movable reference frames with assosiated 
 *  shapes to be drawn in the scene.
 *  The data model was completely refactored into a tree structure (the scene 
 *  graph) allowing for an arbitrary number of reference frames in a recursive 
 *  structure.
 */ 
class CGLSceneGraph : public CGLScene
{
	TShapeAttribute currentAttributes;	//!< the current default attributes for all create* functions
protected:	
	CGLShapeList root;					//!< the data model for the scene graph is just one list in this class
private:
	double m_dAbsoluteZMin;					//!< a value to save the absoluteZMin for further calculations and moving objects
	double m_VecDisplacedCursorOnObject[3];	//!< a value to save the displaced position of the cursor
	CPoint m_SavedMousePos;			//!< a value to save the mouse position if needed 
protected:
	//! this functions integrated a new shape into the scene graph
	unsigned int createShapeCore(CGLShape *pShape, TShapeAttribute* pSA, bool bDynamicMemory);
public:
	// call all openGL draw functions
	void RenderScene();

	//! return a pointer to the root object
	CGLShapeList* GetHandle(){return &(this->root);}

	//! save and get the last mouse position
	void SaveMousePos(const CPoint pos){ m_SavedMousePos = pos; }
	CPoint GetSavedMousePos(){ return m_SavedMousePos; }

	//! return current zoom
	double GetZoom() { return m_zZoom; }

	//! selection test and save a list with the nameing IDs (selectionResult)
	bool selectionTest(const int& x, const int& y);
	
	//! the list to save the last clicked scene object
	std::list<double> selectionResult;

	// convert the movement from 2D to a 3D space based on the m_dAbsoluteZMin as moving plane parallel to camera view
	bool convertMovement2Dto3D(const CPoint& point2D, double& x_3D, double& y_3D, double& z_3D, const double& feedback_x_3D, const double& feedback_y_3D, const double& feedback_z_3D);

	// manipulate the scene graph
	//! create a new brench in the scene graph
	unsigned int createFrame(unsigned int baseId=0);
	CFrame* getFrame(unsigned int id);
	//! delete frame in the scene graph
	bool deleteFrame(unsigned int id) {return root.deleteFrame(id);}
	//! delete all frames in the scene graph
	void deleteAllFrames() {root.deleteAllFrames();}

    // Get screenshot of the scene with specified transformations. Useful to create icons of the scene object
    void GetPreparedBmp(CGLMemoryContext& MC, double* pRotationMatrix, double dZoom);


	//! add a user defined (and owned) object to the draw list
	unsigned int addShape(CGLShape&, TShapeAttribute* pSA=0, bool bDynamicMemory=false);			

	//! get and set the default attributes for new shapes
	void getDefaultAttribute(TShapeAttribute& attrib) {attrib = currentAttributes; }
	void setDefaultAttribute(TShapeAttribute& attrib) {currentAttributes = attrib; } 

	//! delete all items on the given layer
	void deleteLayer(unsigned int layer) { root.deleteLayer(layer); }
	void deleteLayer(std::string layer) { root.deleteLayer(layer); }

	//! delete all layers
	void deleteAllShapes() {root.deleteAllShapes();}

	//! set attributes for all objects on the specified layer
	void setLayerAttributes(unsigned int layerID, const bool bVisible) { root.setLayerAttributes(layerID,bVisible); }

	// driver functions for quick population of the scene with some standard objects
	//! create an axis aligned box of given size color and aplpha value
	unsigned int createBox(const CVector3& Min, const CVector3& Max, const CVector3& color, const double& alpha, TShapeAttribute* pSA=0);
	//! create an axis aligned box of given size and color
	unsigned int createBox(const CVector3& Min, const CVector3& Max, double r=0, double g=1, double b=0, TShapeAttribute* pSA=0, bool wireframe=false);
	//! create a cone
	unsigned int createCone(const CVector3& Apex, const CVector3& Axis, const double& Aperture, TShapeAttribute* pSA=0);
	unsigned int createCone(const CVector3& Apex, const CVector3& Axis, const double& Aperture,const CVector3& color, const double& alpha, TShapeAttribute* pSA=0);
	unsigned int createEllipsoid(const CFrame& frame,const CVector3& radius, const CVector3& color,const double& alpha, TShapeAttribute* pSA=0);

	//! create line strip using a given color
	unsigned int createLineStrip(vector<CVector3> lineStrip, const CVector3& color, TShapeAttribute* pSA=0);
	unsigned int createLineStrip(vector<CVector3> lineStrip, const float& lineWidth, const int& factor, const unsigned short& pattern, const CVector3& color,const double& alpha, TShapeAttribute* pSA=0);
	//! create a 3D vector defined by two points v1,v2 using the radius, color and alpha value
	unsigned int createVector(const CVector3 &v1, const CVector3 &v2, const double &radius, const CVector3& color, const double& alpha, TShapeAttribute* pSA=0);
	//! create poseList object for visualization
	template <class T>
	unsigned int create3DPoseList(list<T*>& poseList, TShapeAttribute* pSA =0)
	{
		return createShapeCore(new CGLShapePoseList(poseList),pSA,true);
	}
	//! load an STL file
	unsigned int createSTL(const std::string& filename, const CVector3& dr, const CMatrix3& dR, const double& s, TShapeAttribute* pSA=0);
	unsigned int createSTL(const std::string& filename, const CVector3& dr, const CMatrix3& dR, const double& s, const double& r_, const double& g_, const double& b_, const double& alpha_, TShapeAttribute* pSA=0);
	unsigned int createVRML(const std::string& filename, const CVector3& dr, const CMatrix3& dR, const double& s, TShapeAttribute* pSA=0);
	unsigned int createVRML(const std::string& filename, const CVector3& dr, const CMatrix3& dR, const double& s, const double& r_, const double& g_, const double& b_, const double& alpha_, TShapeAttribute* pSA=0);

	//! load an 3DS file
	unsigned int create3DS(const std::string& filename, const CVector3& dr, const CMatrix3& dR, const double& s, TShapeAttribute* pSA=0);
};

#endif  //  CGLSCENE_H
