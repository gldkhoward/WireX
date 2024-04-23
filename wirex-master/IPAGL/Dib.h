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

/*! \file Dib.h
 *
 *	\author   Andreas Pott
 *
 *  \class		CDib
 * 
 *  Projekt:			IPAGL
 *  \par Desciption
 *  Routines to display bitmaps in an Win32/MFC context
 *  The class provides functions for loading and saveing BMP files.
 *
 */

#if !defined(IPAGL_CDIB_H)
#define IPAGL_CDIB_H

#include <afxwin.h>         // MFC-Kern- und -Standardkomponenten

class CDib : public CObject
{
public:			// Konstruktoren und Destruktor
	CDib();
	CDib(const CDib &quelle);
	virtual ~CDib();

public:			// Allgemeine Methoden
	BOOL Create(int ResX, int ResY, int BitDepth);
	BOOL LoadBMP(CString filename);
	BOOL SaveBMP(CString filename);
	BOOL Draw(CDC *pDC, CPoint origin, CSize size);
	CDib& operator=(const CDib& quelle);
	BOOL Grayscale(CDib &quelle, double r=0.3333, double g=0.3333, double b=0.3333);
	BOOL CopySection(CDib& quelle, CRect rec);
	BOOL CreateColorTable();

public:			// Datenschnittstelle
	int GetWidth();
	int GetHeight();
	int GetBitsPerPixel();
	int GetImageSize();
	LPBYTE GetImagePtr() { return m_Image; }

protected:		// Interne Methoden
	void Empty();

protected:		// Daten
	LPBYTE m_Image;
	LPBITMAPINFOHEADER m_BMIH;
	int m_ImageSize;
	int m_ColorTableEntries;
public:

// Funktionen in Entwicklung
public:
	CDib& operator-=(const CDib& quelle);
};

//--------------------------------------------------------------------------- 

class AUX_RGBImageRec {
   void convertBGRtoRGB();
 public:
   byte *data;
   DWORD sizeX;
   DWORD sizeY;
   bool NoErrors;
   AUX_RGBImageRec(): NoErrors(false), data(NULL),sizeX(0),sizeY(0) {};
   AUX_RGBImageRec(const char *FileName);
   ~AUX_RGBImageRec();
   bool loadFile(const char *FileName);
   friend AUX_RGBImageRec *auxDIBImageLoad(const char *FileName);
};

#endif // !defined(IPAGL_CDIB_H)
