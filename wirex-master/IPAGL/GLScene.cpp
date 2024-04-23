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

/*! \file GLScene.cpp
 *
 *	\author   Andreas Pott
 */

#include <afxwin.h>         // MFC-Kern- und -Standardkomponenten
#include <gl\gl.h>			// Header File For The OpenGL32 Library
#include <gl\glu.h>			// Header File For The GLu32 Library
#include <locale.h>
#include <vfw.h>

#include "GLScene.h"
#include "GLMemoryContext.h"
#include "Dib.h"
#include "gl2ps.h"

const double CGLScene::TrackBallSize=0.8;	//!< sensity for rotation with the mouse; higher values make the motion slower but more precise


CGLScene::CGLScene()
    : m_bRotationMatrixChanged(false)
{
	//! set the lower and upper color for the background in RGBA color model
	UpperBackgroundColor[0]=0.8;
	UpperBackgroundColor[1]=0.8;
	UpperBackgroundColor[2]=1.0;
	UpperBackgroundColor[3]=1.0;
	LowerBackgroundColor[0]=0.1;
	LowerBackgroundColor[1]=0.1;
	LowerBackgroundColor[2]=1.0;
	LowerBackgroundColor[3]=1.0;

	m_Trans=CVector3(0,0,0);
	m_zO=-3;
	m_zZoom=0;
	aspect_ratio=0;
	m_fovy=40;
    m_ClipZMin=0.1f;      //!< minimal z clipping plane
    m_ClipZMax=1000;      //!< maximal z clipping plane
	m_PerspectiveProjection     = true;
    m_bSingleLightSetup         = true;
    m_bFullSpecularReflectivity = true;

	bPanWithRightMouseButton = false;

	m_fZoomSpeed = 0.05;
	m_fPanSpeed = 0.01;

	bPanWithRightMouseButton = false;

	// init the rotation matrix with the identity matrix
	matRot[0][0] = 1.0; matRot[1][0] = 0.0; matRot[2][0] = 0.0; matRot[3][0] = 0.0;
	matRot[0][1] = 0.0; matRot[1][1] = 1.0; matRot[2][1] = 0.0; matRot[3][1] = 0.0;
	matRot[0][2] = 0.0; matRot[1][2] = 0.0; matRot[2][2] = 1.0; matRot[3][2] = 0.0;
	matRot[0][3] = 0.0;	matRot[1][3] = 0.0; matRot[2][3] = 0.0; matRot[3][3] = 1.0;

	m_bLIsDown = false;
	m_bRIsDown = false;

	cur_q0=1;			// initialize the quaternions
	last_q0=1;

	bEnableRotation=true;	
	bEnableZoom=true;		
	bEnablePan=true;	
	bEnableDragging=false;
	bIsDragging=false;
	
	bRenderBackground = true;
	Height=0;
	Width=0;
	m_hRC = 0;
	m_pDC = 0;

	// init the variables for avi generation
	pMC = 0;
	ps = NULL;
	psCompressed = NULL;
	pfile = NULL;
	iFrame = 0;
}

CGLScene::~CGLScene() 
{
}


void CGLScene::adjustZoom(double new_Zoom)
{
	m_zZoom=new_Zoom;
    //bRedraw=true; 
}


void CGLScene::adjustView(CFrame* K)
{
    if(fabs((K->R*K->r).y)>fabs(r_max.y)) {
          r_max.y=(K->R*K->r).y;
          r_max.z=(K->R*K->r).z;
    }
    if(fabs((K->R*K->r).x)>fabs(r_max.x)) {
          r_max.x=(K->R*K->r).x;
          r_max.z=(K->R*K->r).z;
    }
}

void CGLScene::calculateQuaternion(double xalt, double yalt, double x, double y)
{
	CVector3 p_alt,p_neu;

	if(xalt==x && yalt==y)	// no rotation
	{
		last_q=CVector3(0,0,0);
		last_q0=1;
		return;
	} else {	
		p_alt.x=xalt;
		p_alt.y=yalt;
		p_alt.z=projectToSphere(TrackBallSize,xalt,yalt);

		p_neu.x=x;
		p_neu.y=y;
		p_neu.z=projectToSphere(TrackBallSize,x,y);

		last_q=p_neu%p_alt;
		last_q.normalize();
		double t=(length (p_alt-p_neu))/(2.0*TrackBallSize);

		if (t > 1.0f) 
			t = 1.0f;
		if (t < -1.0f) 
			t = -1.0f;
		double phi = 2.0f * asin(t);

		last_q*=sin(phi/2.0);
		last_q0=cos(phi/2.0);
	}
	// trackball_add_quats()
	CVector3 tf=last_q*cur_q0 + cur_q*last_q0+cur_q%last_q;
    double tf_q0 = last_q0 * cur_q0 - (last_q*cur_q);

	cur_q = tf;
	cur_q0 = tf_q0;

	// renorm the quaternion
	double mag = cur_q*cur_q+cur_q0*cur_q0;
	cur_q  /= mag;
	cur_q0 /= mag;
}

//! calculate the 4x4 rotation matrix from the quarternions
void CGLScene::buildRotationMatrix()
{
    matRot[0][0] = 1.0 - 2.0 * (cur_q.y * cur_q.y + cur_q.z * cur_q.z);
    matRot[0][1] = 2.0 * (cur_q.x * cur_q.y - cur_q.z * cur_q0);
    matRot[0][2] = 2.0 * (cur_q.z * cur_q.x + cur_q.y * cur_q0);
    matRot[0][3] = 0.0;

    matRot[1][0] = 2.0 * (cur_q.x * cur_q.y + cur_q.z * cur_q0);
    matRot[1][1] = 1.0 - 2.0 * (cur_q.z * cur_q.z + cur_q.x * cur_q.x);
    matRot[1][2] = 2.0 * (cur_q.y * cur_q.z - cur_q.x * cur_q0);
    matRot[1][3] = 0.0;

    matRot[2][0] = 2.0 * (cur_q.z * cur_q.x - cur_q.y * cur_q0);
    matRot[2][1] = 2.0 * (cur_q.y * cur_q.z + cur_q.x * cur_q0);
    matRot[2][2] = 1.0 - 2.0 * (cur_q.y * cur_q.y + cur_q.x * cur_q.x);
    matRot[2][3] = 0.0;

    matRot[3][0] = 0.0;
    matRot[3][1] = 0.0;
    matRot[3][2] = 0.0;
    matRot[3][3] = 1.0;
}

void CGLScene::setRotationMatrix(double* pMatrix)
{
    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            matRot[row][col] = pMatrix[row * 4 + col];
        }
    }
    m_bRotationMatrixChanged = true;
}

double CGLScene::projectToSphere(double r, double x, double y)
{
    double d, t, z;

    d =  sqrt(x*x + y*y);
    if (d < r * 0.70710678118654752440) {    /* Inside sphere */
	z = sqrt(r*r - d*d);
    } else {           /* On hyperbola */
        t = r / 1.41421356237309504880;
        z = t*t / d;
    }
    return z;
}

void CGLScene::InitPerspective(double AspectRatio)
{
	aspect_ratio=AspectRatio;
    double zx,zy;
    zx=(fabs(r_max.x)/(tan(m_fovy/2*GL_DEG_TO_RAD)*aspect_ratio)+r_max.z);
    zy=(fabs(r_max.y)/tan(m_fovy/2*GL_DEG_TO_RAD)+r_max.z);
    if(zx>zy)
        m_zO=-1.1*zx;
    else
        m_zO=-1.1*zy;
    if(m_zO>-1)
        m_zO=-1;
    
	last_q=CVector3(0,0,0);
	last_q0=1;
    cur_q=last_q;
    cur_q0=last_q0;
    buildRotationMatrix();
}

void CGLScene::ResetView()
{
	m_Trans = CVector3(0,0,0);
	m_zZoom=0;

	InitPerspective(aspect_ratio);
}


/*! set the GL_PROJECTION matrix according to the current perspective in the GL window
 *  and set the model view matrix according to current rotation, paning, zooming
 */
void CGLScene::SetupSceneTransformation()
{
    // select the projection matrix and clear it
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    // switch between parallel and perspective projection
    if (m_PerspectiveProjection)
        gluPerspective( m_fovy, aspect_ratio, m_ClipZMin, m_ClipZMax );
    else
    {
	    // select the viewing volume
		double x = -(m_zZoom + m_zO);
		glOrtho(-x*tan(m_fovy/2*GL_DEG_TO_RAD)*aspect_ratio,x*tan(m_fovy/2*GL_DEG_TO_RAD)*aspect_ratio,-x*tan(m_fovy/2*GL_DEG_TO_RAD),x*tan(m_fovy/2*GL_DEG_TO_RAD), m_ClipZMin, m_ClipZMax );
    }
    
	// switch back to the modelview matrix and clear it
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();

	// apply currect rotation and translation matrices
	glTranslated( 0.0, 0.0, m_zO + m_zZoom );
	buildRotationMatrix();	// update the rotation matrix from the quaternion before redraw
    glMultMatrixd(*matRot);
	glTranslated(m_Trans.x,m_Trans.y,m_Trans.z);
}

/*! Draw the color gradiant as background for the scene
 */
void CGLScene::RenderBackground()
{
	if (!bRenderBackground)
		return;
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();								// save the projection matrix
		glDisable(GL_LIGHTING);					// Switch off lighting and depth buffer
		glDisable(GL_DEPTH_TEST);
		glLoadIdentity();						// clear the transformation matrix
		glOrtho(-1.0,1.0,-1.0,1.0,-1.0,1.0);	// load an orthogonal projection matrix
		glMatrixMode(GL_MODELVIEW);				// save the modelview matrix
		glPushMatrix();
			glLoadIdentity();
			glBegin(GL_QUADS);					// draw a rect for the background
			glColor4dv(LowerBackgroundColor);   // the lower color
				glVertex3d(-1,-1,1);
				glVertex3d( 1,-1,1);
				glColor4dv(UpperBackgroundColor);
				glVertex3d( 1, 1,1);
				glVertex3d(-1, 1,1);
			glEnd();
		glPopMatrix();							// restore the modelview matrix
		glMatrixMode(GL_PROJECTION);
		glEnable(GL_LIGHTING);					// switch on lighting and depth buffer
		glEnable(GL_DEPTH_TEST);
	glPopMatrix();								// restore the projection matrix
	glMatrixMode(GL_MODELVIEW);					// active the model transformation matrix
}

//! Render the scene by invoking the draw methods of the CGLShape objects
void CGLScene::RenderScene()
{
	//empty the OpenGL color and depth buffer
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	SetupSceneTransformation();
	
	// draw the color gradient for the background of the scene
	RenderBackground();

	// draw all internal and external shapes
	
	IPAGL::makeFrame();							// \todo: a frame which is always visible; perhaps we have to remove this one

	glFinish();

	// swap buffers to avoid flickering
    SwapBuffers( m_pDC->GetSafeHdc() );
    //if ( FALSE == SwapBuffers( m_pDC->GetSafeHdc() ) )
		//MessageBox(0,"SwapBuffers failed","OpenGL",MB_OK);

}
void CGLScene::setOpenGLParameters()
{
    glEnable(GL_TEXTURE_2D);	// Enables Texture Mapping
	glEnable(GL_DEPTH_TEST);	// Hidden surface removal
	glFrontFace(GL_CCW);		// Counter clock-wise polygons face out
	glEnable(GL_CULL_FACE);		// Do not calculate inside of jet

	glEnable(GL_NORMALIZE);

	// Enable lighting
	glEnable(GL_LIGHTING);

    // Specular light values
    GLfloat  specref[] =  { 1.0f, 1.0f, 1.0f, 0.7f };


    // Light position setups
    if (m_bSingleLightSetup)
    {
    	// Light values and coordinates
    	GLfloat  ambientLight[] = { 0.3f, 0.3f, 0.3f, 0.7f };
    	GLfloat  diffuseLight[] = { 0.7f, 0.7f, 0.7f, 0.7f };
    	GLfloat  specular[] = { 1.0f, 1.0f, 1.0f, 0.7f};
    	GLfloat	 lightPos[] = { 0.0f, 150.0f, 150.0f, 0.7f };
    
    	// Setup and enable light 0
    	glLightfv(GL_LIGHT0,GL_AMBIENT,ambientLight);
    	glLightfv(GL_LIGHT0,GL_DIFFUSE,diffuseLight);
    	glLightfv(GL_LIGHT0,GL_SPECULAR,specular);
    	glLightfv(GL_LIGHT0,GL_POSITION,lightPos);
    	glEnable(GL_LIGHT0);
    
    }
    else
    {
        // Light values and coordinates
        GLfloat ambientLight[] = { 0.1f, 0.1f, 0.1f, 0.7f };
        GLfloat diffuseLight[] = { 0.1f, 0.1f, 0.1f, 0.7f };
        GLfloat specular[]     = { 1.0f, 1.0f, 1.0f, 0.7f};
        GLfloat lightPos[]     = { 0.0f, 2000.0f, 0.0f, 0.0f };
        GLfloat lightPos1[]    = { 0.0f, -2000.0f, 0.0f, 0.0f};
        GLfloat lightPos2[]    = { 2000.0f, 0.0f, 0.0f, 0.0f};
        GLfloat lightPos3[]    = { -2000.0f, 0.0f, 0.0f, 0.0f};
        GLfloat lightPos4[]    = { 0.0f, 0.0f, 2000.0f, 0.0f};
        GLfloat lightPos5[]    = { 0.0f, 0.0f, 2000.0f, 0.0f};

        // Setup and enable light 0
        glLightfv(GL_LIGHT0,GL_AMBIENT,ambientLight);
        glLightfv(GL_LIGHT0,GL_DIFFUSE,diffuseLight);
        glLightfv(GL_LIGHT0,GL_SPECULAR,specular);
        glLightfv(GL_LIGHT0,GL_POSITION,lightPos);
        glEnable(GL_LIGHT0);

        // Setup and enable light 1
        glLightfv(GL_LIGHT1,GL_AMBIENT,ambientLight);
        glLightfv(GL_LIGHT0,GL_DIFFUSE,diffuseLight);
        glLightfv(GL_LIGHT1,GL_SPECULAR,specular);
        glLightfv(GL_LIGHT1,GL_POSITION,lightPos1);
        glEnable(GL_LIGHT1);

        // Setup and enable light 2
        glLightfv(GL_LIGHT2,GL_AMBIENT,ambientLight);
        glLightfv(GL_LIGHT2,GL_DIFFUSE,diffuseLight);
        glLightfv(GL_LIGHT2,GL_SPECULAR,specular);
        glLightfv(GL_LIGHT2,GL_POSITION,lightPos2);
        glEnable(GL_LIGHT2);

        // Setup and enable light 3
        glLightfv(GL_LIGHT3,GL_AMBIENT,ambientLight);
        glLightfv(GL_LIGHT0,GL_DIFFUSE,diffuseLight);
        glLightfv(GL_LIGHT3,GL_SPECULAR,specular);
        glLightfv(GL_LIGHT3,GL_POSITION,lightPos3);
        glEnable(GL_LIGHT3);

        // Setup and enable light 4
        glLightfv(GL_LIGHT4,GL_AMBIENT,ambientLight);
        glLightfv(GL_LIGHT4,GL_DIFFUSE,diffuseLight);
        glLightfv(GL_LIGHT4,GL_SPECULAR,specular);
        glLightfv(GL_LIGHT4,GL_POSITION,lightPos4);
        glEnable(GL_LIGHT4);

        // Setup and enable light 5
        glLightfv(GL_LIGHT5,GL_AMBIENT,ambientLight);
        glLightfv(GL_LIGHT0,GL_DIFFUSE,diffuseLight);
        glLightfv(GL_LIGHT5,GL_SPECULAR,specular);
        glLightfv(GL_LIGHT5,GL_POSITION,lightPos5);
        glEnable(GL_LIGHT5);
    }

	// Enable color tracking
	glEnable(GL_COLOR_MATERIAL);
	
	// Set Material properties to follow glColor values
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

    if (m_bFullSpecularReflectivity)
    {
	    // All materials hereafter have full specular reflectivity
	    // with a high shine
	    glMaterialfv(GL_FRONT, GL_SPECULAR,  specref);
	    glMateriali( GL_FRONT, GL_SHININESS, 128);
    }

	// black background
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f );

	// properties for drawing faces and polygons
	glDisable(GL_CULL_FACE);
	glPolygonMode(GL_BACK,GL_FILL);    
}


double* CGLScene::getRotationMatrix() const
{
    double* pResult = new double[16];

    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            pResult[row * 4 + col] = matRot[row][col];
        }
    }

    return pResult;
}

BOOL CGLScene::SetupPixelFormat()
{
    static PIXELFORMATDESCRIPTOR pfd = 
    {
        sizeof(PIXELFORMATDESCRIPTOR),  // size of this pfd
            1,                              // version number
            PFD_DRAW_TO_WINDOW |            // support window
            PFD_SUPPORT_OPENGL |            // support OpenGL
            PFD_DOUBLEBUFFER |			    // double buffered
			PFD_STEREO_DONTCARE,
            PFD_TYPE_RGBA,                  // RGBA type
            24,                             // 24-bit color depth
            0, 0, 0, 0, 0, 0,               // color bits ignored
            0,                              // no alpha buffer
            0,                              // shift bit ignored
            0,                              // no accumulation buffer
            0, 0, 0, 0,                     // accum bits ignored
            32,                             // 32-bit z-buffer
            0,                              // no stencil buffer
            0,                              // no auxiliary buffer
            PFD_MAIN_PLANE,                 // main layer
            0,                              // reserved
            0, 0, 0                         // layer masks ignored
    };
    int pixelformat;
    
    if ( 0 == (pixelformat = ::ChoosePixelFormat(m_pDC->GetSafeHdc(), &pfd)) )
    {
        //MessageBoxA(0,"ChoosePixelFormat failed", "openGL",MB_OK);
        return FALSE;
    }
    
    if ( FALSE == ::SetPixelFormat(m_pDC->GetSafeHdc(), pixelformat, &pfd) )
    {
       	//MessageBoxA(0,"SelectPixelFormat failed", "openGL",MB_OK);
        return FALSE;
    }
    return TRUE;
}

BOOL CGLScene::InitializeOpenGL(CWnd* Wnd)
{
    m_pDC = new CClientDC(Wnd);
    
    if ( NULL == m_pDC ) // failure to get DC
    {
		//MessageBox(0,"Unable to get a DC", "openGL",MB_OK);
        return FALSE;
    }
    
    if (!SetupPixelFormat())
        return FALSE;
    
    if ( 0 == (m_hRC = ::wglCreateContext( m_pDC->GetSafeHdc() ) ) )
    {
        //MessageBox(0,"wglCreateContext failed", "openGL",MB_OK);
        return FALSE;
    }
    
    if ( FALSE == ::wglMakeCurrent( m_pDC->GetSafeHdc(), m_hRC ) )
    {
        //MessageBox(0,"wglMakeCurrent failed", "openGL",MB_OK);
        return FALSE;
    }
    setOpenGLParameters();
	InitPerspective(1.5);
    return TRUE;
}

void CGLScene::OnSize(int cx, int cy)
{
	// store the size of the window for later use
	Width = cx;
	Height = cy;

    // tell openGL to use the whole window for rendering
    glViewport(0, 0, cx, cy);

    // compute the aspect ratio. this will keep all dimension scales equal
	aspect_ratio = (double)cx/(double)cy;
}

void CGLScene::OnDestroy()
{
    ::wglMakeCurrent( 0, 0 );
    //if ( FALSE == ::wglMakeCurrent( 0, 0 ) )
    //    MessageBox(0,"Failed to unselect the view context by wglMakeCurrent", "openGL",MB_OK);
    
    ::wglDeleteContext( m_hRC );
    //if ( FALSE == ::wglDeleteContext( m_hRC ) )
    //    MessageBox(0,"Failed to delete the openGL view context", "openGL",MB_OK);
    
    if ( m_pDC )
        delete m_pDC;
}

//! handle mouse move and clicks to the openGL window for rotation, zooming and panning
void CGLScene::OnLButtonDown(CPoint point) 
{
    // Viewpoint Rotation
	m_bLIsDown = TRUE ;
	m_CurPos = point;
	m_LastPos = point;
}

//! handle mouse move and clicks to the openGL window for rotation, zooming and panning
void CGLScene::OnLButtonUp(CPoint point) 
{
	m_bLIsDown=FALSE;
}

//! handle mouse move and clicks to the openGL window for rotation, zooming and panning
void CGLScene::OnRButtonDown(CPoint point) 
{
	m_bRIsDown=TRUE;
	m_CurPos = point;
}

//! handle mouse move and clicks to the openGL window for rotation, zooming and panning
void CGLScene::OnRButtonUp(CPoint point) 
{
	m_bRIsDown=FALSE;
	m_LastPos=point;
}

//! handle mouse move and clicks to the openGL window for rotation, zooming and panning
//! return true, if redrawing is needed
bool CGLScene::OnMouseMove(CPoint point) 
{
	bool bRedraw=false;
	if(m_bLIsDown&&(!bIsDragging))
        if(!m_bRIsDown)
        {
			bRedraw = OnMouseMoveRotate(point);
        }
        else
        {
			if (bPanWithRightMouseButton) bRedraw = OnMouseMoveZoom(point);
			else bRedraw = OnMouseMovePan(point);
        }
     
    if(m_bRIsDown)
        if(!m_bLIsDown)
        {
			if (bPanWithRightMouseButton) bRedraw = OnMouseMovePan(point);
			else bRedraw = OnMouseMoveZoom(point);
        }

	return bRedraw;
}

//! handle mouse move for rotation
bool CGLScene::OnMouseMoveRotate(CPoint point)
{
	bool bRedraw(false);
	if (point != m_LastPos && bEnableRotation)
	{
		if (m_bRotationMatrixChanged)
		{
			m_LastPos = point;
			m_bRotationMatrixChanged = false;
		}
		calculateQuaternion((2.0*m_LastPos.x / Width) - 1,
			-(2.0*m_LastPos.y / Height) + 1,
			(2.0*point.x / Width) - 1,
			-(2.0*point.y / Height) + 1);
		buildRotationMatrix();
		m_LastPos = point;
		bRedraw = true;
	}
	return bRedraw;
}

//! handle mouse move for panning
bool CGLScene::OnMouseMovePan(CPoint point)
{
	bool bRedraw(false);
	if (bEnablePan)
	{
		CVector3 r;
		CMatrix3 R;
		for (int i = 1; i < 4; i++)
			for (int j = 1; j < 4; j++)
				R(j, i) = matRot[i - 1][j - 1];

		r.x = (point.x - m_CurPos.x) * m_fPanSpeed;
		r.y = -(point.y - m_CurPos.y) * m_fPanSpeed;
		r.z = 0.0;

		m_Trans += r*R;
		m_CurPos = point;
		bRedraw = true;
	}
	return bRedraw;
}

//! handle mouse move for zooming
bool CGLScene::OnMouseMoveZoom(CPoint point)
{
	bool bRedraw(false);
	if (bEnableZoom)
	{
		m_zZoom += (point.y - m_CurPos.y) * m_fZoomSpeed;
		m_CurPos.y = point.y;
		bRedraw = true;
	}
	return bRedraw;
}

//! pan the scene by a defined vector 
void CGLScene::addTrans(CVector3 Trans)
{
	m_Trans+=Trans;
}

//! perform a selection test for a given klick position and return an std::list<double> which represent the hierarchical structure like first_element.second_element.[...].last_element
 bool CGLSceneGraph::selectionTest(const int& x, const int& y)
{
	//set the pick radius to a default value
	const int pick_radius=2;

	//reset displacement vector for moving operations (see convertMovement2Dto3D(...))
	m_VecDisplacedCursorOnObject[0] = 0; m_VecDisplacedCursorOnObject[1] = 0; m_VecDisplacedCursorOnObject[2] = 0;

	//set up selection buffer
	glSelectBuffer(512,selectBuf);
	
	//get actual viewport pos and size
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);
	
	// select the projection matrix and "clear" it
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	
	//set the picking matrix first
	gluPickMatrix((GLdouble)x,(GLdouble)(viewport[3]-y),(GLdouble)pick_radius,(GLdouble)pick_radius,viewport);

	// switch whether parallel or perspective projection is selected
	if (m_PerspectiveProjection) gluPerspective( m_fovy, aspect_ratio, m_ClipZMin, m_ClipZMax );
	else
	{
		// select the viewing volume
		double x = -(m_zZoom + m_zO);
		glOrtho(-x*tan(m_fovy/2*GL_DEG_TO_RAD)*aspect_ratio,x*tan(m_fovy/2*GL_DEG_TO_RAD)*aspect_ratio,-x*tan(m_fovy/2*GL_DEG_TO_RAD),x*tan(m_fovy/2*GL_DEG_TO_RAD), m_ClipZMin, m_ClipZMax );
	}
	
	// switch to the modelview matrix and "clear" it too
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();

	// apply currect rotation and translation matrices
	glTranslated( 0.0, 0.0, m_zO + m_zZoom );		
	glMultMatrixd(*matRot);
	glTranslated(m_Trans.x,m_Trans.y,m_Trans.z);

	// create the name stack and flag the objects before drawing
	glInitNames();
	
	//switch to selection mode
	glRenderMode(GL_SELECT);
	
	//render scene in GL_SELECT mode
	//<! \todo: If large 3ds-files are in the scenegraph object, 
	//this draw take a LOT of time, so the movement is quite stagnant if selection mode is on (bEnableDragging=true)
	root.draw();		

	//get hits in GL_RENDER mode
	GLint hits = glRenderMode(GL_RENDER);

	//restore the original projection matrix to avoid painting errors
	SetupSceneTransformation();

	//now its time to evaluate the returned list of objects

	//set up a "list-list" for storing all objects names and a simple double-value list for simpler editing
	std::list<double> scene_object;
	std::list<std::list<double>> name_storage;
	
	//get elements with their namestack and zmin,zmax infos
	//and print some debug information from the selection test into the console
	int index=0;
	for( int i = 0; i < hits; i++ )
	{
		UINT nitems = selectBuf[index++];
		UINT zmax = selectBuf[index++];
		UINT zmin = selectBuf[index++];	

		//store zmin and zmax in the matrix after dividing it by max_int to scale z values from 0 to 1
		scene_object.push_back((double)zmax/(double)UINT_MAX);
		scene_object.push_back((double)zmin/(double)UINT_MAX);

		//loop over all names found in the name stack and push it in the list
		for(UINT j = 0; j < nitems; j++ )
		{
			int item = selectBuf[index++];
			scene_object.push_back(item);
		}

		//insert the object in the storage and clear the scene object list
		name_storage.push_back(scene_object);
		scene_object.clear();

	}
	
	// if elements are found, search the topmost one!
	if (name_storage.size()>0)
	{
		//set the first object as reference start object
		scene_object = (*name_storage.begin());
		m_dAbsoluteZMin = (*scene_object.begin());

		//find the nearest object (-> minimum max_z value)
		for (std::list<std::list<double>>::iterator itor = name_storage.begin(); itor!=name_storage.end(); itor++)
		{
			//if next element has a lower max_z value ... use that object as new "nearest object" and set the global absoluteZMin value
			if ((*itor).front() < scene_object.front()) 
			{
				scene_object = (*itor); 
				m_dAbsoluteZMin = (*scene_object.begin());
			}
		}

		//if nothing found on click position, return false (if size < 3 means, no names were on the name stack!)
		if (scene_object.size()<3) return false;

		/*//print debug infos
		printf("Found topmost Pick-Object:\n");
		printf("max Z: %f\n", (*scene_object.begin())); scene_object.pop_front();
		printf("min Z: %f\n", (*scene_object.begin())); scene_object.pop_front();
		printf("Name: %.0f",  (*scene_object.begin()));
		std::list<double>::iterator itor = scene_object.begin(); itor++;
		while (itor!=scene_object.end())
		{
			printf(".%.0f",(*itor)); 
			itor++;
		}
		printf("\n");
		//*/
		
		//pop the first two elements (z_max and z_min) (WARNING: has to be removed, if the debug print is on!)
		scene_object.pop_front();
		scene_object.pop_front();
	}

	//handling the results in the public selectionResult list
	selectionResult = scene_object;

	//return true, because everthing worked out fine
	return true;

}

/*! Render the current image into a bitmap
 *  \todo Implement a dialog to choose resolution
 */
void CGLScene::SaveBmp(const std::string& filename, const int& width, const int& height) 
{
	// Generate a memory rendering context (it will be automatically destroyed at end of this scope)
	CGLMemoryContext MC(width,height);
	// make this memory context the current one for rendering
	MC.makeCurrent();
	// set the lighting etc. openGL parameters
	setOpenGLParameters();
	// set the viewport (size of the image)
	MC.setViewport();
	// select the viewing volumn and the depth range
	MC.setPerspective( m_fovy, m_ClipZMin, m_ClipZMax );
	// Render the scene
	RenderScene();
	// Save results to file
	CDib Dib;
	Dib.Create(MC.Width(),MC.Height(),24);
	memcpy(Dib.GetImagePtr(),MC.getImagePtr(),MC.getImageSize());
	Dib.SaveBMP(filename.c_str());
	// return to normal DC for openGL
	wglMakeCurrent(m_pDC->GetSafeHdc(),m_hRC);
}

/*! Render the current image into a bitmap and return the bitmap
 */
void CGLScene::GetBmp(CGLMemoryContext& MC) 
{
	// make this memory context the current one for rendering
	MC.makeCurrent();
	// set the lighting etc. openGL parameters
	setOpenGLParameters();
	// set the viewport (size of the image)
	MC.setViewport();
	// select the viewing volumn and the depth range
	MC.setPerspective( m_fovy, m_ClipZMin, m_ClipZMax );
	// Render the scene
	RenderScene();
	// return to normal DC for openGL
	wglMakeCurrent(m_pDC->GetSafeHdc(), m_hRC);
}

/*! Save the current GLWindow to an image file.
 *  It is possible to choose between different file formats.
 *  /note Method does not support unicode compilation and is empty if symbol UNICODE is defined!
 *  /param [in] fileType:  PS (0), EPS (1), TEX(2), PDF(3) SVG(4), PGF(5)
 */
void CGLScene::SaveImage(const std::string& filename, int fileType) 
{
// gl2ps functions do currently not support unicode
#ifndef UNICODE
	// push the state of the renderBackground and some locals flag
	bool oldRB = bRenderBackground;	
	bRenderBackground=false;
	char *oldlocale = setlocale(LC_NUMERIC, "C");

	FILE *fp;
	fopen_s(&fp,filename.c_str(), "wb");
    GLint buffsize = 0, state = GL2PS_OVERFLOW;
    GLint viewport[4];
    glShadeModel(GL_FLAT);			// set shading to flat (rather than SMOOTH)
    glGetIntegerv(GL_VIEWPORT, viewport);
	// evaluate the feedback buffer until the size is large enough
    while( state == GL2PS_OVERFLOW ){
        buffsize += 1024*1024;
        gl2psBeginPage ( 
            "Screenshot",                    // title of the plot in %%Title
            "IPAGL",						 // plot producer in %%For
            viewport,                        // the viewport
            fileType,                       // the file format
            GL2PS_BSP_SORT,                  // sort strategy 
            GL2PS_BEST_ROOT|                 // option: no messages on error stream
            GL2PS_USE_CURRENT_VIEWPORT,      // use the viewport from current GL scene
            GL_RGBA,                         // RGBA color model
            0,                               // size of color map
            NULL,                            // the used color map
            0,								 // nr controls the number of triangle subdivisions for red color
            0,								 // ng same for green
            0,								 // nb for blue
            buffsize,                        // the size of the opengl feedback buffer
            fp,                              // the file where the output is written
			filename.c_str());               // the name of the outputstream
		RenderScene();
        state = gl2psEndPage();
    }
    glShadeModel(GL_SMOOTH);
    fclose(fp);
	// pop the state of the renderBackground and locals flags
	setlocale(LC_NUMERIC, oldlocale);
	bRenderBackground=oldRB;	
#endif // UNICODE
}


/*! Save the current GLWindow to a PS file. The technique for writing PS directly from the
 *  openGL buffer is implemented the gl2ps library. 
 */
void CGLScene::SaveEps(const std::string& filename) 
{
	SaveImage(filename,GL2PS_EPS);
}

/*! create a new avi 
 *  \todo Can we remove text stream from the video stream?
 */
bool CGLScene::initAvi(const std::string& filename, int width, int height)
{
   // Prüfen ob Video für Windows vorhanden, und wenn ja in der richtigen Version !!!
	if (HIWORD(VideoForWindowsVersion()) < 0x010a) // Video für Windows Treiber muß mindestens Version 1.1 sein
		return false;

	if (pMC)
		return finalizeAvi(false);

    // create the memory rendering context
    pMC = new CGLMemoryContext(width,height);

	// AVI init
	AVISTREAMINFO strhdr;
	pfile = NULL;
	ps = NULL, psCompressed = NULL;
	AVICOMPRESSOPTIONS opts;
	AVICOMPRESSOPTIONS FAR *aopts[1] = {&opts};

	iFrame = 0;

	AVIFileInit();

	if (AVIFileOpenA( &pfile, LPCSTR(filename.c_str()), OF_CREATE|OF_WRITE, NULL) != AVIERR_OK)
	{
		// AVI Datei konnte nicht erzeugt werden
		AVIFileExit();
		return false;
	}

	// Header erstellen
	memset( &strhdr, 0, sizeof(strhdr));
	strhdr.fccType		= streamtypeVIDEO;
	strhdr.fccHandler	= 0;
	strhdr.dwScale		= 1;
	strhdr.dwRate		= 25;
	strhdr.dwSuggestedBufferSize = pMC->getImageSize();

	SetRect(&strhdr.rcFrame, 0, 0,width,height);

	// And create the stream
	if (AVIFileCreateStream(pfile,	&ps, &strhdr) != AVIERR_OK) // Error while creating AVI file stream
		return finalizeAvi(false);

	memset(&opts, 0, sizeof(opts));

	// open the dialog box and let the user select the configuration information
	// now that we know where the dialog call is, we may want to save the respective
	// values in a class member and seperate compression configuration from rendering!
	if (!AVISaveOptions(NULL, ICMF_CHOOSE_DATARATE | ICMF_CHOOSE_KEYFRAME , 1, &ps, (LPAVICOMPRESSOPTIONS FAR *) &aopts))
	{
		AVIStreamClose(ps);
	    return false;
	}

	// connect the compressed stream (psCompressed) to the raw stream (ps) where the used compressor is
	// given by opts
	if (AVIMakeCompressedStream(&psCompressed, ps, &opts, NULL) != AVIERR_OK) // Error while creating compressed stream
		return finalizeAvi(false);

	if (AVIStreamSetFormat(psCompressed, 0, pMC->getBMIH(), pMC->getBMIH()->biSize+pMC->getBMIH()->biClrUsed * sizeof(RGBQUAD)) != AVIERR_OK) // Error while setting video stream format
		return finalizeAvi(false);

    pMC->makeCurrent();
    setOpenGLParameters();
    // set the viewport (size of the image)
    pMC->setViewport();
	// select the viewing volumn and the depth range
    pMC->setPerspective( m_fovy, m_ClipZMin, m_ClipZMax );
	
	return true;
}

//! save the current frame into the stram
//! \todo Check if a we should check if the stream is open before proceding!
bool CGLScene::saveAviFrame()
{
	if (!pMC)
		return false;

    // Render the scene
	RenderScene();

    // Save results to file
	HRESULT hr = AVIStreamWrite(psCompressed,	// stream pointer
		iFrame,							// time of this frame
		1,								// number to write
            pMC->getImagePtr(),           // pointer to the image data
            pMC->getImageSize(),          // length of the image data in byte
		AVIIF_KEYFRAME,					// flags....
		NULL, NULL);

	if (hr != AVIERR_OK) // Error while writing video frame to stream
		return false;

	iFrame++;
	return true;
}

/*! close the avi stream
 *  This functions cleans up the Win32 API for AVI after finishing the 
 *  video or if an error occured.
 *  This method is automatically called by initAvi, if an error occures
 *  the flag ok is then set to false to indicate that the data may be 
 *  already corrupted. 
 */
bool CGLScene::finalizeAvi(bool ok)
{
	// return to normal DC for openGL rendering on the screen buffer
    wglMakeCurrent(m_pDC->GetSafeHdc(),m_hRC);

    // Now close the file
	if (ps) AVIStreamClose(ps);
	if (psCompressed) AVIStreamClose(psCompressed);
	if (pfile) AVIFileClose(pfile);

	delete pMC;
	pMC = 0;

	AVIFileExit();
	return ok;
}

void CGLScene::printParam() const
{
	printf("Parameter of CGLScene\n");
	printf("TrackballSize: %f\n",TrackBallSize);	//!< constants for the quaternion calculation of rotation
	printf("CurPos: %i , %i\n",m_CurPos.x,m_CurPos.y);		//!< actual mouse x,y coordiante for tracking the movement
	printf("LastPos: %i , %i\n",m_LastPos.x,m_LastPos.y);		//!< last mouse x,y coordinate for tracking the movement
	printf("z0: %f\n",m_zO);					//!< initial distance camera/origin as determined by the scene
	printf("zZoom: %f\n",m_zZoom);				//!< actual distance due to zooming
	printf("fovy: %f\n",m_fovy);				//!< the angle [in deg] of the viewing volume
	printf("ClipZMin: %f\n", m_ClipZMin);		//!< minimal z clipping plane
	printf("ClipZMax: %f\n", m_ClipZMax);		//!< maximal z clipping plane
	printf("Trans: (%f, %f, %f)\n",m_Trans.x,m_Trans.y,m_Trans.z);		//!< actual x,y,z displacement of the viewpoint
	printf("Rotation Quaternion: q0=%f, q=(%f, %f, %f)\n",cur_q0,cur_q.x,cur_q.y,cur_q.z);   //!< actual and last scalar part of the quarternions of the view
	printf("r_max: (%f, %f, %f)\n",r_max.x,r_max.y,r_max.z);			//!< used to determine the visible part of the scene
	printf("Width,Height: %i x %i\n",Width,Height);       //!< height and width of the GL window
	printf("aspect_ratio: %f\n", aspect_ratio);	//!< width/height ratio of the window
}

void CGLScene::setXYPlane() 
{
    last_q=sin(0*GL_DEG_TO_RAD)*CVector3(1,1,1)/sqrt(3.0);
    last_q0=cos(0*GL_DEG_TO_RAD);
	cur_q=last_q;
	cur_q0=last_q0;
	buildRotationMatrix();
}

void CGLScene::setXZPlane() 
{
    last_q=sin(45*GL_DEG_TO_RAD)*CVector3(1,0,0);
    last_q0=cos(45*GL_DEG_TO_RAD);
	cur_q=last_q;
	cur_q0=last_q0;
	buildRotationMatrix();
}

void CGLScene::setYZPlane() 
{
    last_q=sin(60*GL_DEG_TO_RAD)*CVector3(1,1,1)/sqrt(3.0);
    last_q0=cos(60*GL_DEG_TO_RAD);
	cur_q=last_q;
	cur_q0=last_q0;
	buildRotationMatrix();
}


// ***************************************************************************
// CGLSceneGraph
// ***************************************************************************

void CGLSceneGraph::RenderScene()
{
    // empty the OpenGL color and depth buffer
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	
	//make necessary scene transformations to move the camera position
	SetupSceneTransformation();

	// draw the color gradient for the background of the scene
	RenderBackground();

	// draw the full scene graph
	root.draw();

	// swap buffers to avoid flickering
    SwapBuffers( m_pDC->GetSafeHdc() );
  //  if ( FALSE == SwapBuffers( m_pDC->GetSafeHdc() ) )
		//MessageBox(0,"SwapBuffers failed","OpenGL",MB_OK);
}

/*! create a new branch in the scene graph. if baseID is 0 the new branch is 
 *  created at the root.
 *  \return handle to the new branch or 0 if the branch could not be created
 */
unsigned int CGLSceneGraph::createFrame(unsigned int baseId)
{
	CGLShapeList *pSL;
	// insert the new frame to the root element
	if (baseId==0)
	{
		pSL = new CGLShapeList();
		root.append(pSL);
		return pSL->getID();
	}
	// search the given frame id
	CGLShapeList *pBase = root.getFrame(baseId);
	if (pBase)
	{
		pSL = new CGLShapeList();
		pBase->append(pSL);
		return pSL->getID();
	}
	return 0;
}


CFrame* CGLSceneGraph::getFrame(unsigned int id)
{
	CGLShapeList *pSL = root.getFrame(id);
	if (!pSL)
		return 0;
	else
		return pSL->pFrame;
}


void CGLSceneGraph::GetPreparedBmp(CGLMemoryContext& MC, double* pRotationMatrix, double dZoom)
{
    // Get current scene transformations
    CVector3 oldViewPoint = getViewPoint();
    double dOldZoom = m_zZoom;
    double dOldRotation[4][4];
    std::copy(&matRot[0][0], &matRot[0][0] + 16, &dOldRotation[0][0]);
    CVector3 oldTrans = m_Trans;

    // Apply scene transformations necessary for taking the picture
    setViewPoint(CVector3(0,0,0));
    setRotationMatrix(pRotationMatrix);
    adjustZoom(dZoom);

    // make this memory context the current one for rendering
	MC.makeCurrent();
	// set the lighting etc. openGL parameters
	setOpenGLParameters();
	// set the viewport (size of the image)
	MC.setViewport();
	// select the viewing volumn and the depth range
	MC.setPerspective( m_fovy, m_ClipZMin, m_ClipZMax );
	// Render the scene
	RenderScene();
    // return to normal DC for openGL
	wglMakeCurrent(m_pDC->GetSafeHdc(),m_hRC);

    // Reset to old scene transformations
    setViewPoint(oldViewPoint);
    std::copy(&dOldRotation[0][0], &dOldRotation[0][0] + 16, &matRot[0][0]);
    adjustZoom(dOldZoom);
}


/*! this functions integrated a new shape into the scene graph. the function 
 *  should only be called from the create* function
 *  of CGLSceneGraph 
 *  \param pShape is a pointer to the shape object which shall be inserted into the scene graph
 *  \param pSA is a pointer to a data structure with the object attributes (visibility, layer, reference coordinate frame, pointer to user data); if pSA is 0, default values are applied
 *  \param bDynamicMemory is true, if the scene graph should take responsibility to free the memory when needed
 *  \return a handle to the shape object
 */
unsigned int CGLSceneGraph::createShapeCore(CGLShape *pShape, TShapeAttribute* pSA, bool bDynamicMemory)
{
	// if no specific attributes are passed we use the default values
	if (!pSA)
		pSA = &currentAttributes;

	// set the attributes of the shape
	pShape->bVisible = pSA->bVisibile;
	pShape->iLayer = pSA->iLayer;
	pShape->sLayer = pSA->sLayer;
	pShape->userData = pSA->userData;
	//set the dynamic memory flag to delete the dynamic objects from memory
	pShape->bDynamicMemory = bDynamicMemory;

	// just for the moment we are lazy and put the shape into the root list
	if (pSA->iFrame==0)
		root.append(pShape);
	else
	{
		// find the right position to insert or delete the shape if needed
		CGLShapeList *pSL =root.getFrame(pSA->iFrame);
		if (pSL)
			pSL->append(pShape);
		else if (pShape && bDynamicMemory)
			delete pShape;
		return 0;
	}
	return pShape->getID();
}

//! add a user defined (and owned) object to the draw list
unsigned int CGLSceneGraph::addShape(CGLShape& Shape, TShapeAttribute* pSA, bool bDynamicMemory)
{
	return createShapeCore(&Shape,pSA,bDynamicMemory);
}

/*! create an axis aligned box of given size
 *  \param min [in] the minimum coordinates of the box
 *  \param max [in] the maximum coordinates of the box
 *  \return returns a handle to the shape
 */
unsigned int CGLSceneGraph::createBox(const CVector3& Min, const CVector3& Max, const CVector3& color, const double& alpha, TShapeAttribute* pSA)
{
	return createShapeCore(new CGLShapeBox(Min,Max,color,alpha),pSA, true);
}

//! same as the other createBox but the colors are coded in r,g,b rather that in a vector
unsigned int CGLSceneGraph::createBox(const CVector3& Min, const CVector3& Max, double r, double g, double b, TShapeAttribute* pSA, bool wireframe)
{
	if (r>=0 && r<=1 && g>=0 && g<=1 && b>=0 && b<=1)
		return createShapeCore(new CGLShapeBox(Min,Max,r,g,b,wireframe),pSA, true);	// if the color values are correct, use them
	else
		return createShapeCore(new CGLShapeBox(Min,Max),pSA, true);	// otherwise use the default colors
}

unsigned int CGLSceneGraph::createCone(const CVector3& Apex, const CVector3& Axis, const double& Aperture, TShapeAttribute* pSA)
{
	return createShapeCore(new CGLShapeCone(Apex,Axis,Aperture),pSA,true);
}

unsigned int CGLSceneGraph::createCone(const CVector3& Apex, const CVector3& Axis, const double& Aperture,const CVector3& color, const double& alpha, TShapeAttribute* pSA)
{
	return createShapeCore(new CGLShapeCone(Apex,Axis,Aperture,color,alpha),pSA,true);
}

unsigned int CGLSceneGraph::createEllipsoid(const CFrame& frame,const CVector3& radius, const CVector3& color,const double& alpha, TShapeAttribute* pSA)
{
	return createShapeCore(new CGLShapeEllipsoid(frame,radius,color, alpha),pSA,true);
}

//! load a STL file
unsigned int CGLSceneGraph::createSTL(const std::string& filename, const CVector3& dr, const CMatrix3& dR, const double& s, TShapeAttribute* pSA)
{
	return createShapeCore(new CGLShapeSTL(IPAGL::K0,filename, dr, dR, s),pSA,true);
}

//! load a STL file
unsigned int CGLSceneGraph::createSTL(const std::string& filename, const CVector3& dr, const CMatrix3& dR, const double& s, const double& r_, const double& g_, const double& b_, const double& alpha_, TShapeAttribute* pSA)
{
	return createShapeCore(new CGLShapeSTL(IPAGL::K0,filename, dr, dR, s,r_, b_, g_, alpha_),pSA,true);
}

//! load a VRML file
unsigned int CGLSceneGraph::createVRML(const std::string& filename, const CVector3& dr, const CMatrix3& dR, const double& s, TShapeAttribute* pSA)
{
	return createShapeCore(new CGLShapeVRML(IPAGL::K0,filename,dr,dR,s),pSA,true);
}

//! load a STL file
unsigned int CGLSceneGraph::createVRML(const std::string& filename, const CVector3& dr, const CMatrix3& dR, const double& s, const double& r_, const double& g_, const double& b_, const double& alpha_, TShapeAttribute* pSA)
{
	return createShapeCore(new CGLShapeVRML(IPAGL::K0,filename, dr, dR, s,r_, b_, g_, alpha_),pSA,true);
}

//! load an 3DS file
unsigned int CGLSceneGraph::create3DS(const std::string& filename, const CVector3& dr, const CMatrix3& dR, const double& s, TShapeAttribute* pSA)
{
	return createShapeCore(new CGLShape3DS(filename, dr, dR, s),pSA,true);
}

//! Create line strip using a given line width, pattern factor, line pattern, color and alpha value
//! The factor specifies a multiplier for each bit in the line pattern
//! The line pattern is defined by a 16bit integer (0xffff yields a closed line)
//! The name can be used to identify the object in the shapeContainer
unsigned int CGLSceneGraph::createLineStrip(vector<CVector3> lineStrip, const CVector3& color, TShapeAttribute* pSA)
{
	return createShapeCore(new CGLShapeLineStrip(lineStrip, 1.0, 1, 0xFFFF, color,1.0),pSA,true);
}

unsigned int CGLSceneGraph::createLineStrip(vector<CVector3> lineStrip, const float& lineWidth, const int& factor, const unsigned short& pattern, const CVector3& color,const double& alpha, TShapeAttribute* pSA)
{
	return createShapeCore(new CGLShapeLineStrip(lineStrip, lineWidth, factor, pattern, color,alpha),pSA,true);
}


//! create a 3D vector defined by two points v1,v2 using the radius, color and alpha value
unsigned int CGLSceneGraph::createVector(const CVector3 &v1, const CVector3 &v2, const double &radius, const CVector3& color, const double& alpha, TShapeAttribute* pSA)
{
	return createShapeCore(new CGLShapeVector(v1, v2, radius, color, alpha),pSA,true);
}

bool CGLSceneGraph::convertMovement2Dto3D(const CPoint& point2D, double& x_3D, double& y_3D, double& z_3D, const double& feedback_x_3D, const double& feedback_y_3D, const double& feedback_z_3D)
{
	//declare variables to save current openGL-matrices
	GLint viewport[4];
	GLdouble mvmatrix[16], projmatrix[16];
	GLint x = point2D.x;	   // mouse x - position from window
	GLint y = point2D.y;	   // mouse y - position from window
	GLdouble dummy;
	
	//get the current matrices
	glGetIntegerv (GL_VIEWPORT, viewport);
	glGetDoublev (GL_MODELVIEW_MATRIX, mvmatrix);
	glGetDoublev (GL_PROJECTION_MATRIX, projmatrix);

	//calculate the inverse transformations
	gluUnProject ((GLdouble) x, (GLdouble)(viewport[3] - (GLint) y - 1), m_dAbsoluteZMin, mvmatrix, projmatrix, viewport, 
				  &(GLdouble)x_3D, &(GLdouble)y_3D, &(GLdouble)z_3D);
	
	//if this is the first move operation, then add the displacement vector
	if (m_VecDisplacedCursorOnObject[0]==0&&m_VecDisplacedCursorOnObject[1]==0&&m_VecDisplacedCursorOnObject[2]==0) 
	{
		m_VecDisplacedCursorOnObject[0]=feedback_x_3D-x_3D;
		m_VecDisplacedCursorOnObject[1]=feedback_y_3D-y_3D;
		m_VecDisplacedCursorOnObject[2]=feedback_z_3D-z_3D;
	}

	//add the displacement vector to the calculated vector
	x_3D = x_3D + m_VecDisplacedCursorOnObject[0];
	y_3D = y_3D + m_VecDisplacedCursorOnObject[1];
	z_3D = z_3D + m_VecDisplacedCursorOnObject[2];

	//Problem: if objects moves "in" or "out" the screendimension z, a new z-value has to be aquired
	//so we map the actual object position with mousepointer displacement to the 3D space again and retrieve the new z-value
	// in fact, this method is quick and works quite well, if the viewing angle is not to sharp
	gluProject(feedback_x_3D-m_VecDisplacedCursorOnObject[0], feedback_y_3D-m_VecDisplacedCursorOnObject[1], feedback_z_3D-m_VecDisplacedCursorOnObject[2] ,
		mvmatrix, projmatrix, viewport, &dummy , &dummy , &m_dAbsoluteZMin);

	//print infos; but be careful, it really slows down the painting, since the console has no or a large maximum entry number
	//printf ("World coords at z=clickedZ are (%f, %f, %f)\n", x_3D, y_3D, z_3D);

	//return true, because everything worked fine
	return true;
}
