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

/*! \file GLMemoryContext.cpp
 *
 *	\author   Andreas Pott
 */

#include "GLScene.h"
#include "GLMemoryContext.h"

//////////////////////////////////////////////////////////////////////
// Konstruktion/Destruktion
//////////////////////////////////////////////////////////////////////

CGLMemoryContext::CGLMemoryContext(int Width, int Height)
: width(Width), height(Height)
{
    // Create a DIBSection to paint on!
    int iSize = sizeof(BITMAPINFOHEADER);
    memset(&BIH,0,iSize);

    BIH.biSize          = iSize;
    BIH.biWidth         = width;
    BIH.biHeight        = height;
    BIH.biPlanes        = 1;
    BIH.biBitCount      = 24;
    BIH.biCompression   = BI_RGB;

    // Create a DC as base for the memory buffer
    m_pdc = new CDC;
    m_pdc->CreateCompatibleDC(NULL);    // NULL means memory device context

    // Create the DibSection
    m_hbmp = CreateDIBSection(m_pdc->GetSafeHdc(),(BITMAPINFO*) &BIH, DIB_PAL_COLORS, &m_pBits, NULL, 0);
    
    ASSERT(m_hbmp);
    ASSERT(m_pBits);

    // Select the new bitmap into the buffer DC
    SelectObject(m_pdc->GetSafeHdc(),m_hbmp);

	pBitmap = m_pdc->GetCurrentBitmap() ;
	pBitmap->GetObject(sizeof(BITMAP), &bmInfo) ;

	ASSERT(bmInfo.bmPlanes == 1) ;
	ASSERT((bmInfo.bmBitsPixel == 8) || (bmInfo.bmBitsPixel == 16) || (bmInfo.bmBitsPixel == 24)) ;		

	// Fill in the Pixel Format Descriptor
    memset(&pfd,0, sizeof(PIXELFORMATDESCRIPTOR)) ;
    pfd.nSize = sizeof(PIXELFORMATDESCRIPTOR);  
    pfd.nVersion   = 1 ;                           // Version number
    pfd.dwFlags    = PFD_SUPPORT_OPENGL|PFD_SUPPORT_GDI|PFD_DRAW_TO_BITMAP ;
    pfd.iPixelType = PFD_TYPE_RGBA ;
    pfd.cColorBits = (BYTE)bmInfo.bmBitsPixel ;
    pfd.cDepthBits = 32 ;						// 32-bit depth buffer
    pfd.iLayerType = PFD_MAIN_PLANE ;           // Layer type

	// Chose the pixel format.
    int nPixelFormat = ChoosePixelFormat(m_pdc->m_hDC, &pfd);
    if (nPixelFormat == 0)
	{
		TRACE("ChoosePixelFormat Failed %d\r\n",GetLastError()) ;
		return ;
	}
	TRACE("Pixel Format %d\r\n", nPixelFormat) ;

	// Set the pixel format.
    BOOL bResult = SetPixelFormat(m_pdc->m_hDC, nPixelFormat, &pfd);
    if (!bResult)
	{
		TRACE("SetPixelFormat Failed %d\r\n",GetLastError()) ;
		return;
	}
	
    // Create a rendering context.
    m_hrc = wglCreateContext(m_pdc->m_hDC);
	if (!m_hrc)
	{
		TRACE("wglCreateContext Failed %x\r\n", GetLastError()) ;
		return;
	}
}

CGLMemoryContext::~CGLMemoryContext()
{
    // if this objects Rendering context is the current, unselect it 
    // in order to free it
    if (wglGetCurrentContext()==m_hrc)
        wglMakeCurrent(0,0);

    // delete the DIBSection
    DeleteObject(m_hbmp);

    // Delete the local rendering context
    wglDeleteContext(m_hrc);

    // Delete the device context
    delete m_pdc;

    //! \todo Prüfen, ob wirklich alle Belegten Resourcen wieder freigegeben wurden!
}


/*! Make this memory context the current memory context for OpenGL
 */
void CGLMemoryContext::makeCurrent() const
{
	// make the new context the current
    wglMakeCurrent(m_pdc->GetSafeHdc(),m_hrc);
}