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
 * 
 *  \class    MoGLMemoryContext
 *
 *  \par Class description
 *  CLMemoryContext provides a device independant OpenGL rendering
 *  context that allows the user to render OpenGL command directly into
 *  the memory onto a DIBSection. This basic technique is required for
 *	saving single images with arbitrary resolution to disk or to render
 *  animations. The usage of a OpenGL memory context avoids to read the
 *  data from the screen buffer and therefore avoids interference with
 *  dialog boxes, screen savers and the mouse.\n
 *  The OpenGL memory context is fixed to a color depth of 24bit. 

 *  \error There seems to be a bug (or issue) in OpenGL that creates display 
 *  lists for each device context individually. Since most OpenGL library 
 *  that work with display lists use some kind of caching mechanism, it may
 *	happen that these elements are not drawn if the corresponding objects
 *  have been drawn on another context before. In this case the display 
 *  list is created in the other device context and here we cannot access
 *  the list. Thus, this object draws nothing in the memory context.
 */

#if !defined(CGLMEMORYCONTEXT_H)
#define CGLMEMORYCONTEXT_H

#include <gl/gl.h>
#include <gl/glu.h>

class CGLMemoryContext  
{
    int width;              //!< x resolution of the memory buffer
    int height;             //!< y resolution of the momory buffer

    BITMAPINFOHEADER BIH;   //!< structure with informations about the memory context (size, colors, etc)
    CDC* m_pdc;             //!< a DC as base for the memory buffer

    void* m_pBits;          //!< pointer to the image data in the memory
    HBITMAP m_hbmp;         //!< handle for the bitmap (DIBSection)
    
	CBitmap* pBitmap;       //!< pointer to the bitmap (DIBSection)
	BITMAP bmInfo ;         //!< information about the DIBSection that is assosiated this the rendering context
    PIXELFORMATDESCRIPTOR pfd ;
    HGLRC m_hrc;            //!< a handle to the OpenGL rendering context

	double aspect_ratio;    //!< the aspect ratio. this will keep all dimension scales equal

public:
	CGLMemoryContext(int Width=640, int Height=480);
	virtual ~CGLMemoryContext();

    //! make the memory context the current OpenGL rendering context. any further openGL
    //! commands will be executed in this rendering context
    void makeCurrent() const;

    //! return the pointer to the image
    void* getImagePtr() const { return m_pBits; }
    //! return the size in byte that is required for the image
    int getImageSize() const { return width*height*3; }
    //! set the viewport for OpenGL to the full extend of the memory buffer
    void setViewport() const { glViewport(0, 0, width, height); }
    //! set the perspective matrix to the values given through fovy, near and far clipping plane
    void setPerspective(double fovy, double near_, double far_) const
        { gluPerspective( fovy, (double)width/(double)height, near_, far_ ); }
    //! get the height in pixel of the memory buffer
    int Height() const { return height; }
    //! get the width in pixel of the memory buffer
    int Width() const {return width; }
    //! get a pointer to the BitmapInfoHeader
    BITMAPINFOHEADER* getBMIH() { return &BIH; }
};

#endif // !defined(CGLMEMORYCONTEXT_H)
