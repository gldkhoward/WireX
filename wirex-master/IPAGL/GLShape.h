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

/*! \file GLShape.h
 *
 *	\author   Andreas Pott
 */

#ifndef  CGLSHAPE_H
#define  CGLSHAPE_H

#include <afxwin.h>         // MFC-Kern- und -Standardkomponenten
#include <gl\gl.h>										// Header File For The OpenGL32 Library
#include <gl\glu.h>										// Header File For The GLu32 Library
#include <map>
#include <list>
#include "GLGeometry.h"
#include "GL3DS.h"

using namespace std;

//! the namespace IPAGL collects utility function for the openGL drawing library
namespace IPAGL 
{
    static int facettes=10;            //!< the number of faces on round surfaces
	static CFrame K0;
	void setSceneAttributes(int facettes_);	
    void makeBox(const double& dx=1, const double& dy=1, const double& dz=1);
    void makeFancyBox(const CVector3& a, const CVector3& b, const double& width);
    void makeWireBox(const double& dx=1, const double& dy=1, const double& dz=1);
	void makeWireBox(const double& minx, const double& maxx, const double& miny, const double& maxy, const double& minz, const double& maxz);
    void makeCylinder(const double& radius=1, const double& height=1);
	void makeCylinder_Symmetric(const double& r=1, const double& h=1, const CVector3& color = CVector3(0,1,1));
    void makeSpherePatch(CVector3 &v1, CVector3 &v2, CVector3 &v3, int depth);
    void makeSphere(const double& r=1);
	void makeCone(double r=1); 
	void makeCone(const CVector3& apex, const CVector3& axis, const double& aperture );
	void makeCone(const CVector3& apex, const CVector3& axis, const double& aperture, const CVector3& color, const double& alpha);
	void makeEllipsoid(const CVector3& radius, const CVector3& color,const double& alpha);
	void makeFrame(int iHighlightedFrameDimension=0);
	void makeRing(float fInnerRadius, float fOuterRadius);
	void makeFrame(float fxAxisLength, float fyAxisLength, float fzAxisLength, bool bTransparent = false, bool bPaintRotationCircles = false, int nActiveAxis = -1, bool bNamedAxes = true); 
	void makeRotationFrame(int iHighlightedFrameDimension=0);
	void makeVector(const CVector3 &v1, const CVector3 &v2, const double &radius, const CVector3& color, const double& alpha);
};

/*! TShapeAttribute collects shape releated data in a compact struct. 
 *  The main usage of TShapeAttribute is in the create* functions of
 *  CGLScene to define the details of new shapes in only one (optional) 
 *  data struct as well as to set default values. 
 */
struct TShapeAttribute
{
	unsigned int iLayer;			//!< select a number labeled layer for an object. Layer allow to group objects and to perform collective operations
	string sLayer;					//!< select a string labeled layer for an object. Layer allow to group objects and to perform collective operations
	unsigned int iFrame;			//!< select the reference frame
	void* userData;					//!< pointer to arbitrary user data
	bool bVisibile;					//!< indicates if the shape is drawn
	TShapeAttribute() : iLayer(0), sLayer(string()), iFrame(0), userData(0), bVisibile(true) {}
};

/*! \class CGLRGBColor 
 *  CGLRGBColor is used for creating a color,
 *	it consists of three "double" elements: red, green and blue 
 */
class CGLRGBColor
{
public:
	double R;	//!< red color component 0.0 < R < 1.0
	double G;	//!< green color component 0.0 < G < 1.0
	double B;	//!< blue color component 0.0 < B < 1.0
	CGLRGBColor(const double& r=1.0, const double& g=1.0, const double& b=1.0):R(r),G(g),B(b) {}
};

/*! color with alpha channel
*/
class CGLRGBAColor
{
public:
	double R;	//!< red color component 0<R<1
	double G;
	double B;
	double A;
	CGLRGBAColor(const double& r=1.0, const double& g=1.0, const double& b=1.0, const double& a=1.0):R(r),G(g),B(b),A(a) {}
};

/*! \class CGLShape
 *  CGLShape is the abstract base class for all drawable entities in IPAGL.
 *  Thus, the interface for these entities is define within this class. 
 *
 *  \par Class description
 *  CGLShape and its derived classes are the core of the visualization 
 *  functions. All the drawing things are done within these classes. The 
 *  base class itself CGLShape is intended to be abstract (constructor is protected).
 *  When the scene is drawn CGLScene invokes the draw() method for each object, 
 *	so it is up to each object to display itself. Since the base class should 
 *	only provide the interface. The drawing caperbilities of openGL can be 
 *	extended through the CGLShape interface. 
 *  The CGLShape class has a system to generate unique ID for each instance. These ID
 *  can be used as a kind of handles to identify a certain object.
 * 
 *  CGLShape has three important attributes for each shape: id, layer, and 
 *  bDynamicMemory. 
 *  id is a unique identifier for each shape to locate it in the scene graph. 
 *  id can be used as handle. layer allows the implementation of groups of 
 *  objects. operations such as deleting shapes can be performed for all objects 
 *  that share a layer. therefore, layer is NOT unique. bDynamicMemory indicates 
 *  that the memory management (espically deallocation) is managed by the scene
 *  graph. this extensions allows to mix dynamically allocated shapes with static 
 *  shapes in the same data model.
 */
class CGLShape
{
    friend class CGLSceneGraph;  
	friend class CGLShapeList;
private:
	static unsigned int nextID;		//!< the next ID to be used
protected:
	unsigned int ID;				//!< a unique number for each shape
	unsigned int iLayer;			//!< select a number labeled layer for an object. Layer allow to group objects and to perform collective operations
	string sLayer;					//!< select a string labeled layer for an object. Layer allow to group objects and to perform collective operations
	bool bDynamicMemory;			//!< indicates that the sceneGraph is responsible for dynamic allocation/deallocation of this object
	//! construct a new shape and initialize the member. The class CGLShape is pure virtual.
	CGLShape() { bVisible=true; userData=0; ID=nextID; nextID++; iLayer=0; sLayer=""; bDynamicMemory=false; }					

public:
	bool bVisible;		//!< indicates that the shape is visible
	void* userData;		//!< pointer to arbitrary user data; can be used for selection mode

	virtual ~CGLShape() {} 

	// delete all shapes and all childs to clean up this shape list object
	void clear();

	//! multiply the transformation matrix onto the matrix stack
	static void MultFrame(CFrame* K);	
	//! Interface für openGL drawing code. this function must be implemented in derived classes
	virtual void draw()=0;
	//! read the unique ID for this shape
	unsigned int getID() const { return ID; }
	//! return the layer for this shape
	unsigned int getLayer() const { return iLayer; }
};

//! Basic class for shapes with fonts
class CGLShapeFont : public CGLShape
{
    static int ref_count;   //!< keep track of the number of instances of this class
protected:
    static int start;       //!< the first character that is used from the font
    static int end;         //!< the last character that is used from the font
    static int ListStart;   //!< the starting index of the display lists
    static GLYPHMETRICSFLOAT agmf[256]; //!< stores the metric of the glyphs in the font

    float deviation;        //!< specifies the maximum chordal deviation from the true outlines
    float extrusion;        //!< extrusion value in the negative z direction
    int format;             //!< specifies line segments or polygons in display lists

	bool InitFonts();		//!< generated the displayed lists used for drawing the text
public:
    double color[3];        //!< the color of the text

    CGLShapeFont();
    ~CGLShapeFont();
};

/*! Zeichnet einen Schriftzug an einem Koordinatensystem in 3D Schriftart ein.
 *  Die Schrift liegt in der xy-Ebene des Koordinatensystems, in das sie angehängt
 *  wurde.
 */
class CGLShape3DText : public CGLShapeFont
{
	CFrame* baseFrame;
    string text;            //!< the text that is drawn
public:
    double size;            //!< the relative size of the text (default=1)

    CGLShape3DText( CFrame& K, const string& str) : text(str), size(1.0) { baseFrame=&K; }
    void draw();
};

/*! Zeichnet ein Textlabel an einem Koordiantensystem in ebener Schrift. Das Label
 *  rotiert nicht mit dem Rest der Scene und ist stets senkrecht zum Bildschirm
 *  ausgerichtet. Der Text wird nicht gezoomt. 
 */
class CGLShapeLabel : public CGLShapeFont
{
	CFrame* baseFrame;
    string text;            //!< the text that is drawn
	int font_size;
	double color[3];
public:
    CGLShapeLabel( CFrame& K, const string& str) : text(str) 
	{ baseFrame=&K; font_size=16; color[0]=1.0;  color[1]=0.0; color[2]=0.0;} //!<  standard constructor
	CGLShapeLabel( CFrame& K, const string& str, int font_size_, double r, double g, double b) : text(str) 
	{ baseFrame=&K; font_size=font_size_; color[0]=r;  color[1]=g; color[2]=b;} //!<  special constructor
	void draw();
	void setbaseFrame(CFrame& K){ baseFrame=&K; }
	void setText(const string& str){text=str; }
};

/*struct CVector3f 
{
	float x,y,z;
	CVector3f() : x(0),y(0),z(0) {}
	CVector3f(const float& X,const float& Y,const float& Z) : x(X),y(Y),z(Z) {}
};*/

struct CSTLRecord
{
	CVector3f n,a,b,c;
	char res[2];
};

//! draws an objects read from a STL file
//! see http://rpdrc.ic.polyu.edu.hk/old_files/stl_colour_binary_format.htm
//! for information how colors can be coded within an STL file
class CGLShapeSTL : public CGLShape
{
protected:
	list<CVector3> patches;			//!< list of normals and vertices
    CVector3 translate;				//!< relative translation of stl object
    CMatrix3 rotate;				//!< local roation matrix of stl object
    double scale;					//!< scale factor for the object
	GLenum drawmode;				//!< select the drawmode
	CGLRGBColor color;				//!< the color of the shape
	double alpha;					//!< alpha value for transparency
    CFrame* baseFrame;				//!< reference frame to draw the shape
	GLuint listIndex;				//!< index of the display list if available
public:
    CGLShapeSTL( CFrame& , const string&, const CVector3&, const CMatrix3&, const double& s, const double& r_=0.7, const double& g_=0.7, const double& b_=0.7, const double& alpha_=1.0) ;   //!<STL
    ~CGLShapeSTL();
	void setColor(double r, double g, double b) { color.R=r; color.G=g; color.B=b; }
    void draw();
    void drawWireframe();
};

/*! helper class to read and draw substructures of an VRML 1.0 file
 * here we implement the storage of vertex lists, e.g. just a list of 3D coordinates
 */
class CGLShapeVrmlCoordinate3 // : public CGLShape
{
public:
	vector<CVector3> vertices;
	bool read(ifstream& file);
};

/*! index lists refer to Coordinate3 lists to define polygons (mostly triangles)
 */
class CGLShapeVrmlIndexedFace  : public CGLShape
{
	CGLShapeVrmlCoordinate3 *pCoords;
	vector<int> index;
public:
	CGLShapeVrmlIndexedFace(CGLShapeVrmlCoordinate3& Coords) : pCoords(&Coords) {}

	bool read(ifstream& file);
	void draw() ;
};

/*! materials store color and other surface informations and can be enqueued
 *  between indexedFaces to set individual colors*/
class CGLShapeVrmlMaterial  : public CGLShape
{
public:
	CGLRGBColor ambientColor;
	CGLRGBColor diffuseColor;
    CGLRGBColor specularColor;
    CGLRGBColor emissiveColor;
    double shininess;
    double transparency;

	bool read(ifstream& file);
	void draw();
};

/*! \class CGLShapeVRML
 *  The class provides loading and drawing of objects stored in the VRML text format.
 *  The recent extended version of the base stl loader that can deal with some elements in a VRML file
 *  the parser and the data model was fully refactored to handle more complicated files
 *  anyway, the class is far away from dealing with all features of VRML. Tests with files
 *  exported from SolidWorks give ancouraging results.
 *  \remark The class is derived from CGLShapeSTL for legacy reasons and also because it uses the same
 *  data structure for managing glDrawLists, colors, and relative translation. However, parts
 *  of the implementation and the call to the STL constructure is ill-structured and needs revision.
 */
class CGLShapeVRML : public CGLShapeSTL
{
	// new data model
	vector<CGLShapeVrmlCoordinate3*> coords;
	vector<CGLShape*> drawList;

	// old data model (depricated; will be removed in a future version)
	vector<CVector3> vertices;
	vector<int> triangles;

	//! file loader imported read by mzp/321; this implementation is depricated
	void ReadVRMLFile(std::string filenameVRML); 
//	bool doMaterial(ifstream &file, CGLShapeVrmlMaterial& material);
//	bool doCoordinate3(ifstream &file, CGLShapeVrmlCoordinate3& coord);
//	bool doFaceSet(ifstream& file, CGLShapeVrmlIndexedFace& Faces);
	//! new parser with support for the advanced data model
	bool parseVrml(ifstream& file, bool seekEndOfBlock=true);

public:
    CGLShapeVRML( CFrame& , const string&, const CVector3&, const CMatrix3&, const double& s, const double& r_=0.7, const double& g_=0.7, const double& b_=0.7, const double& alpha_=1.0) ;   //!<STL
	~CGLShapeVRML();

    void draw();
};

class CGLShapeCoordinatePlanes : public CGLShape
{
	double grid;	//!< size of the coordinate planes
	double alpha;	//!< alpha value for transparency
	CGLRGBColor color_planes;	//!< color of the planes
	CGLRGBColor color_border;	//!< color of the plane's border
public:
	CGLShapeCoordinatePlanes ( CFrame& K0 = IPAGL::K0 ); 
	
	void draw();
};

class CGLShapeGroundPlane : public CGLShape
{
	double grid;	//!< size of the ground plane
	double alpha;	//!< alpha value for transparency
	double corner1[3];
	double corner2[3];
	CGLRGBColor color_planes;	//!< color of the planes
	CGLRGBColor color_border;	//!< color of the plane's border
public:
	CGLShapeGroundPlane ( CFrame& K0 = IPAGL::K0 ); 
	void setGroundSize(double *corner1, double *corner2); 
	void draw();
};

class CGLShapeTriangulatedSurface : public CGLShape
{
	vector<CVector3> vertices;		//!< vertex data of the surface
	GLenum drawmode;				//!< draw mode (allows for points, edges, and faces where the latter is default
	CGLRGBColor color;				//!< the color of the shape
public:
	CGLShapeTriangulatedSurface();
	//! delete content
	void clear();					
	//! add a triangle with vertices a,b,c to the shape
	void addTriangle(const CVector3& a, const CVector3& b, const CVector3& c);
	//! call the openGL rendering routine
	void draw();
	//! define the color of the surface in rgb values [0..1]
	void setColor(double r, double g, double b) { color.R=r; color.G=g; color.B=b; }
};

//! vom Benutzer per Callback Funktion gezeichnete Objekte 
class CGLShapeCallback : public CGLShape 
{
protected:
    void (*userdraw)(void);         //!< Interface für benutzergezeichnete Elemente
public:
    CGLShapeCallback( void (*fkt)(void) ) {userdraw=fkt; }
    ~CGLShapeCallback() {}
    void draw() { userdraw();}
};

/*! Base class for a new type of shape called manipulators (abr. manip)
 *  Classes derived from CGLShapeManip work as wrappers (or in terms of
 *  the design pattern, a decorator) for other shapes
 *  in order to perform certain initialization and/or restoration tasks
 *  before and/or after drawing the encapsulated shape. 
 */
class CGLShapeManip : public CGLShape
{
    CGLShape *wrap;      //!< pointer to the wrapped shape
public:
    CGLShapeManip(CGLShape& shape) 
    { wrap = &shape; }

    void draw();
    //! performs initialization before drawing the encapsulated shape
    virtual void preDraw()=0;
    //! performs restoreation after drawing the encapsulated shape
    virtual void postDraw()=0;
};

/*! Make the encapsulated shape transperent. The transperency is currently
 *  fixed to 50%. */
class CGLShapeTransperency : public CGLShapeManip
{
public:
    CGLShapeTransperency(CGLShape& shape) : CGLShapeManip(shape) {}

    void preDraw();
    void postDraw();
};

/*! Apply a constant translation and/or rotation to the selected shape type */
class CGLShapeTransformator : public CGLShapeManip
{
    CVector3 dr;
    CMatrix3 dR;
public:
    CGLShapeTransformator(CGLShape& shape, CVector3);
    CGLShapeTransformator(CGLShape& shape, CMatrix3);
    CGLShapeTransformator(CGLShape& shape, CVector3, CMatrix3);

    void preDraw();
    void postDraw();
};

//! another shape attached to a frame movable in space
class CGLShapeMobileFrame : public CGLShapeManip
{
	CVector3* dr;		//!< pointer to the translational offset
	CMatrix3* dR;		//!< pointer to the rotational offset
public:
	CGLShapeMobileFrame(CGLShape& shape, CFrame& frame);
	CGLShapeMobileFrame(CGLShape& shape, CVector3&r);
	CGLShapeMobileFrame(CGLShape& shape, CVector3&r, CMatrix3& R);
	void preDraw();
	void postDraw();
};

/*! Draw an axis aligned box */
class CGLShapeBox : public CGLShape
{
	CVector3 Min,Max, color;
	double alpha;
	bool m_wireframe;
public:
	CGLShapeBox(const CVector3& Min_, const CVector3& Max_, const CVector3& color_, const double& alpha_);
	CGLShapeBox(const CVector3& Min_, const CVector3& Max_, double r=0, double g=1, double b=0);
	CGLShapeBox(const CVector3& Min_, const CVector3& Max_, double r, double g, double b, bool wireframe);
	void draw();
};

/*! Draw a cone */
class CGLShapeCone : public CGLShape
{
	CVector3 apex,axis, color;
	double aperture, alpha;
public:
	CGLShapeCone(const CVector3& Apex, const CVector3& Axis, const double& Aperture);
	CGLShapeCone(const CVector3& Apex, const CVector3& Axis, const double& Aperture, const CVector3& Color, const double& Alpha);
	void draw();
};

/*! Draw an ellipsoid */
class CGLShapeEllipsoid : public CGLShape
{
	CFrame frame;
	CVector3 radius, color;
	double alpha;
public:
	CGLShapeEllipsoid(const CFrame& Frame, const CVector3& Radius, const CVector3& Color, const double& Alpha);
	void draw();
};

/*! Draw line strip*/
class CGLShapeLineStrip: public CGLShape
{
	vector<CVector3> m_lineStrip;
	float m_lineWidth;
	int m_factor;
	int m_pattern;
	CVector3 m_color;
	double m_alpha;

public:
	CGLShapeLineStrip(vector<CVector3>& lineStrip, const float& lineWidth, const int& factor, const unsigned short& pattern, const CVector3& color,const double& alpha);
	void draw();
};

/*! Draw a point cloud*/
class CGLShapePointCloud: public CGLShape
{
       vector<CVector3> m_pointCloud;
       float m_pointSize;
       CVector3 m_color;
       double m_alpha;

public:
       CGLShapePointCloud(vector<CVector3>& pointCloud, const float& pointSize, const CVector3& color,const double& alpha);
       void draw();
};

/*! Draw 3D vector*/
class CGLShapeVector: public CGLShape
{
	CVector3 m_v1, m_v2, m_Color;
	double m_Radius, m_Alpha;

public:		
	CGLShapeVector(const CVector3 &v1, const CVector3 &v2, const double &radius, const CVector3& color, const double& alpha);
	void draw();
};

/*! Base for Shapes, that use texture mapping
 *  \todo Implement destructors, that free the reserved textureIDs through glDeleteTexture(..)
 */
class CGLShapeBitmap : public CGLShape
{
protected:
    static void CreateTexture(LPSTR strFileName, GLuint& textureID);
};

//! draws an objects read from a 3DS file
//  \remark This object requires an implementation of a 3DS file loader which is
//  currently not included in the project.
class CGLShape3DS : public CGLShapeBitmap 
{
    C3DSModel Frame3DSObject;		//!< reference for 3DS object
    CVector3 translate;				//!< relative translation of 3ds object
    CMatrix3 rotate;				//!< local roation matrix of 3ds object
    double scale;					//!< scale factor for the object
    map<int,GLuint> textureID;      //!< map of all used texture IDs
	GLint listIndex;				//!< the index for the OpenGL lists

public:
    CGLShape3DS( const string&, const CVector3&, const CMatrix3&, const double&) ;   //!<3DS 
    ~CGLShape3DS() {}
    void draw();
};


/*! This class implements the main concepts for constructing a scene graph from shape objects.
 *  CGLShapeList represents both, a reference frame and a list of embedded objects (which can contain
 *  other CGLShapeList). 
 * 
 *  Create a list of shape objects. Calling the draw function will cause
 *  every object in the list to be drawn. Before drawing, CGLShapeList applied the transformation matrix of
 *  the frame and after calling the list the transformation will be clean up.
 *
 *  The shape ID of this class can be used to identify certain frames in the scene.
 */
class CGLShapeList : public CGLShape
{
	friend class CGLSceneGraph;
	//! data model
public:
	//! a list of all shapes to be drawn with respect to the local coordiante frame
	list <CGLShape*> content;
	//! a list of further ShapeList that are drawn relative to this list. In this way a hiecharchy can be implemented
	list <CGLShapeList*> childs;
	//! pointer to the frame containing the transformation of this shape
	CFrame *pFrame;
	//! an internal storage of a transformation. currentFrame points to internalFrame if no external transformation is given
	CFrame Frame;

public:
	CGLShapeList();
	~CGLShapeList();

	//! delete all shapes and all childs to clean up this shape list object
	void clear();
	//! delete all shapes on a given number labeled layer
	void deleteLayer(unsigned int layer);
	//! delete all shapes on a given string labeled layer
	void deleteLayer(string layer);
	//! set attributes for all objects on a given layer
	void setLayerAttributes(unsigned int layer, const bool bVisible);
	//! delete the shape with the given id (with recursive search in all childs)
	bool deleteShape(unsigned int id);
	//! delete all shapes in this list
	void deleteAllShapes();
	//! delete the frame with the given id (with resursive search in all childs)
	bool deleteFrame(unsigned int id);
	//! delete all frames
	void deleteAllFrames();
	//! draw 
	void draw();
	//! add a shape to the list
	void append(CGLShape* pShape) { content.push_back(pShape); }
	//! add a shape to the list
	void append(CGLShapeList* pShape) { childs.push_back(pShape); }
	//! get shape list for frame id
	CGLShapeList* getFrame(unsigned int id);
};

#ifdef EXTENDED_DRAWING

/*! Draw the standard elements of multi-body systems (rigid link, revolute joint, 
 *	prismatic joint, spherical joint, coordinate (frame), mass element )
 */
class CGLStdShape : public CGLShape 
{
	//! draws a frame
	void makeFrame();
	//! draws a rigid link
	void makeRL();
	//! draws a simple cone

//	void makeSpherePatch(CVector3 &v1, CVector3 &v2, CVector3 &v3, int depth);
//	CFrame				*baseFrameRL;
public:
//	CGLStdShape( CFrame& );

	void draw();
};


//! Draw an arrow with respect to the specified frame
class CGLShapeArrow : public CGLShape
{
	CVector3 *v;
	double scale;
	bool toFrame;
public:
	CGLShapeArrow(CFrame& frame, CVector3& Vec, const double Scale=1.0, bool DirectonToFrame=false);
	void draw();
};


//! Zeichnet ein Bitmap auf eine Fläche
class CGLShapeRectangleTexture : public CGLShapeBitmap
{
    MoAxis normal;      //!< the normal vector for the rectangle
    double normaloffset;//!< relative offset in the direction of the normal vector
    double ScaleU;      //!< stretch factor for the bitmap in x (left-right) direction
    double ScaleV;      //!< stretch factor fot the bitmap in y (bottom-top) direction
	double SizeX;		//!< size of the rectangle in x-direction
	double SizeY;		//!< size of the rectangle in y-direction
	GLuint materialID;	//!< the ID of the current texture
public:
	CGLShapeRectangleTexture( CFrame&, const string&, MoAxis Normal=zAxis, double sizex=1, double sizey=1);
    //! moves the rectangle along the axis of the normal
    void setOffset(double offset=0.0) { normaloffset=offset; }
    //! set the scaling factor for the texture on the rectangle. u,v>1 produce a tessellation effect
    void setStretch(double u=1.0, double v=1.0) { ScaleU=u; ScaleV=v; }
	void draw();
};

//! Draw geometrical primitive
class CGLShapePrimitive : public CGLShape 
{
	CGLShapeColor nColor;		//!< index of the color used for drawing this object
private:
    //! Abhängig vom aktuellen Shape werden nicht alle Parameter benötigt
    double sizex;   //!< Ausdehnung in x
    double sizey;   //!< Ausdehnung in y
    double sizez;   //!< Ausdehnung in z
public:
    CGLShapePrimitive(CFrame& BaseFrame, CGLShape Type, double SizeX=1.0, 
        double SizeY=1.0, double SizeZ=1.0);
    ~CGLShapePrimitive() {}
    void draw();
    void drawWireframe();
};

#endif // EXTENDED_DRAWING

#endif  //  CGLSHAPE_H
