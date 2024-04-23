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

// Dib.cpp: Implementierung der Klasse CDib.
//
//////////////////////////////////////////////////////////////////////

#include "Dib.h"

//////////////////////////////////////////////////////////////////////
// Konstruktion/Destruktion
//////////////////////////////////////////////////////////////////////

CDib::CDib()
{
	m_Image=NULL;
	m_BMIH=NULL;
	m_ImageSize=0;
	m_ColorTableEntries=0;
}

CDib::CDib(const CDib &q)
{
	m_ImageSize=q.m_ImageSize;
	m_ColorTableEntries=q.m_ColorTableEntries;
//	if (q.m_hBitmap==NULL)
	if (q.m_BMIH==NULL)
		m_BMIH=NULL;
	else
	{
		int nSize = sizeof(BITMAPINFOHEADER) + sizeof(RGBQUAD) * q.m_ColorTableEntries;
		m_BMIH = (LPBITMAPINFOHEADER) new char[nSize];
		memcpy(m_BMIH, q.m_BMIH, nSize);
	}
	if (q.m_Image==NULL)
		m_Image=NULL;
	else
	{
		m_Image = (LPBYTE) new char[q.m_BMIH->biSizeImage];
		memcpy(m_Image,q.m_Image,q.m_BMIH->biSizeImage);
	}
}

CDib::~CDib()
{
	if (m_Image!=NULL)
		delete [] m_Image;
	if (m_BMIH!=NULL)
		delete [] m_BMIH;
}

//////////////////////////////////////////////////////////////////////
// static Variablen
//////////////////////////////////////////////////////////////////////

BOOL CDib::Create(int ResX, int ResY, int BitDepth)
{
	if (BitDepth==8)
	{
		m_ColorTableEntries=256;
		CreateColorTable();
	}
	else
		m_ColorTableEntries=0;
	int nSize = sizeof(BITMAPINFOHEADER) + sizeof(RGBQUAD) * m_ColorTableEntries;
	m_BMIH = (LPBITMAPINFOHEADER) new char[nSize];
	m_BMIH->biBitCount=BitDepth;
	m_BMIH->biClrImportant=0;
	m_BMIH->biClrUsed=0;
	m_BMIH->biCompression=BI_RGB;
	m_BMIH->biSize=sizeof(BITMAPINFOHEADER);
	m_BMIH->biSizeImage=sizeof(BYTE)*BitDepth*ResX*ResY/3;
	m_BMIH->biXPelsPerMeter=1000;
	m_BMIH->biYPelsPerMeter=1000;
	m_BMIH->biPlanes=1;
	m_BMIH->biHeight=ResY;
	m_BMIH->biWidth=ResX;
	// allocate memory for the image
	m_Image = (LPBYTE) new char[m_BMIH->biSizeImage];
	memset(m_Image,0,m_BMIH->biSizeImage);
	m_ImageSize=m_BMIH->biSizeImage;

	return true;
}

CDib& CDib::operator=(const CDib &q)
{
	if (this!=&q)	// Selbstzuweisung verhindern
	{
		m_ImageSize=q.m_ImageSize;
		m_ColorTableEntries=q.m_ColorTableEntries;
	//	if (q.m_hBitmap==NULL)
		if (q.m_BMIH==NULL)
			m_BMIH=NULL;
		else
		{
			delete [] m_BMIH;
			int nSize = sizeof(BITMAPINFOHEADER) + sizeof(RGBQUAD) * q.m_ColorTableEntries;
			m_BMIH = (LPBITMAPINFOHEADER) new char[nSize];
			memcpy(m_BMIH, q.m_BMIH, nSize);
		}
		if (q.m_Image==NULL)
			m_Image=NULL;
		else
		{
			delete [] m_Image;
			m_Image = (LPBYTE) new char[q.m_BMIH->biSizeImage];
			memcpy(m_Image,q.m_Image,q.m_BMIH->biSizeImage);
		}
	}
	return *this;
}

BOOL CDib::LoadBMP(CString filename)
{
	Empty();
	CFile hFile;
	if (!hFile.Open(filename,CFile::modeRead))
	{
		//AfxMessageBox("CDib::LoadBMP - error while reading file");
		return false;
	}

	int nCount, nSize;
	BITMAPFILEHEADER bmfh;
	try {
		nCount = hFile.Read((LPVOID) &bmfh, sizeof(BITMAPFILEHEADER));
		if(nCount != sizeof(BITMAPFILEHEADER)) {
			throw new CResourceException;
		}
		if(bmfh.bfType != 0x4d42) {
			throw new CResourceException;
		}
		nSize = bmfh.bfOffBits - sizeof(BITMAPFILEHEADER);
		m_BMIH = (LPBITMAPINFOHEADER) new char[nSize];
		nCount = hFile.Read(m_BMIH, nSize); // Info-Header & Farbtabelle
		if (m_BMIH->biSizeImage==0)
			m_BMIH->biSizeImage=m_BMIH->biHeight*m_BMIH->biWidth*m_BMIH->biBitCount/8;
		m_ImageSize=m_BMIH->biSizeImage;
		m_Image = (LPBYTE) new char[m_ImageSize];
		nCount = hFile.Read(m_Image, m_ImageSize); // nur Grafik
	}
	catch(CException* pe) {
		//AfxMessageBox("CDib::LoadBMP - Lesefehler");
		pe->Delete();
		hFile.Close();
		return FALSE;
	}
	hFile.Close();
	return TRUE;		
}

BOOL CDib::SaveBMP(CString filename)
{
	BITMAPFILEHEADER bmfh;
	bmfh.bfType = 0x4d42;  // 'BM'
	int nSizeHdr = sizeof(BITMAPINFOHEADER) + sizeof(RGBQUAD) * m_ColorTableEntries;
	bmfh.bfSize = 0;
	bmfh.bfReserved1 = bmfh.bfReserved2 = 0;
	bmfh.bfOffBits = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER) +
			sizeof(RGBQUAD) * m_ColorTableEntries;	
	CFile hFile;
	try {
		if (!hFile.Open(filename,CFile::modeCreate | CFile::modeWrite))
		{
			printf("CDib::SaveBMP:: Error while file %s\n",filename.GetBuffer());
			//AfxMessageBox("CDib::SaveBMP - error while writing file");
			return false;
		}
		hFile.Write((LPVOID) &bmfh, sizeof(BITMAPFILEHEADER));
		hFile.Write((LPVOID) m_BMIH,  nSizeHdr);
		hFile.Write((LPVOID) m_Image, m_ImageSize);
	}
	catch(CException* pe) {
		pe->Delete();
		//AfxMessageBox("CDib::SaveBMP - Fehler beim Schreiben");
		hFile.Close();
		return FALSE;
	}
	hFile.Close();
	return TRUE;
}

BOOL CDib::Draw(CDC *pDC, CPoint origin, CSize size)
{
	if(m_BMIH == NULL) return FALSE;
	pDC->SetStretchBltMode(COLORONCOLOR);
	::StretchDIBits(pDC->GetSafeHdc(), origin.x, origin.y, size.cx, size.cy,
		0, 0, m_BMIH->biWidth, m_BMIH->biHeight,
		m_Image, (LPBITMAPINFO) m_BMIH, DIB_RGB_COLORS, SRCCOPY);
	return TRUE;
}

BOOL CDib::Grayscale(CDib &q, double r, double g, double b)
{
	int rr=(int)(r*255);
	int gg=(int)(g*255);
	int bb=(int)(b*255);
	int x;
	BYTE Graytable[256*sizeof(RGBQUAD)];
	Empty();
	if (q.m_BMIH->biSizeImage==0)
		return false;
	m_ImageSize=q.m_BMIH->biHeight*q.m_BMIH->biWidth;
	m_Image = (LPBYTE) new char[m_ImageSize];
	int nSize = sizeof(BITMAPINFOHEADER) + sizeof(RGBQUAD) * 256;
	m_BMIH = (LPBITMAPINFOHEADER) new char[nSize];
	memcpy(m_BMIH, q.m_BMIH, sizeof(BITMAPINFOHEADER));
	for (x=0; x<256; x++)
	{
		Graytable[x*4]=(BYTE)x;
		Graytable[x*4+1]=(BYTE)x;
		Graytable[x*4+2]=(BYTE)x;
		Graytable[x*4+3]=0;
	}
	memcpy(m_BMIH+1,Graytable, 256*sizeof(RGBQUAD));	// m_BMIH+1 ergibt die Adresse direkt hinter dem Header aufgrund der scalierten Zeigerarithmetik
	switch (q.m_BMIH->biBitCount)
	{
	case 1:	return false;
	case 4: return false;
	case 8: return false;
	case 16:return false;
	case 24:
		for (x=0; x<m_ImageSize; x++)
			m_Image[x]=(rr*q.m_Image[x*3]+gg*q.m_Image[x*3+1]+bb*q.m_Image[x*3+2])/(255);
		m_BMIH->biBitCount=8;
		m_BMIH->biSizeImage=m_ImageSize;
		break;
	case 32:return false;
	default:return false;
	}
	return true;
}

BOOL CDib::CopySection(CDib& q, CRect rec)
{
	if (q.m_Image==NULL)
		return false;
	Empty();
	if (rec.IsRectEmpty())
		return false;
	rec.NormalizeRect();
	CRect imgrec(0,0,q.m_BMIH->biWidth,q.m_BMIH->biHeight);
	if (!(imgrec.PtInRect(rec.TopLeft()))
		||(!imgrec.PtInRect(rec.BottomRight())))
		return false;
	
	int nSize = sizeof(BITMAPINFOHEADER) + sizeof(RGBQUAD) * q.m_ColorTableEntries;
	m_BMIH = (LPBITMAPINFOHEADER) new char[nSize];
	memcpy(m_BMIH, q.m_BMIH, nSize);

	m_BMIH->biWidth=rec.Width()+1;
	m_BMIH->biHeight=rec.Height()+1;
	m_BMIH->biSizeImage=m_BMIH->biWidth*m_BMIH->biHeight*m_BMIH->biBitCount/8;
	m_ImageSize=m_BMIH->biSizeImage;
	
	m_Image = (LPBYTE) new char[m_ImageSize];

	int x,y;
	int dx=rec.left*m_BMIH->biBitCount/8;
	int dy=rec.top;
	int rowoffdes=m_BMIH->biWidth*m_BMIH->biBitCount/8;
	int rowoffsrc=q.m_BMIH->biWidth*q.m_BMIH->biBitCount/8;
	for (x=0; x<rowoffdes; x++)
		for (y=0; y<m_BMIH->biHeight; y++)
			m_Image[x+y*rowoffdes]=q.m_Image[dx+x+(dy+y)*rowoffsrc];

	return true;
}

CDib& CDib::operator-=(const CDib &q)
{
	if ((q.m_Image==NULL)||(m_Image==NULL))
		return *this;		// Kein Bild in Quelle/Ziel -> keine Aenderungen
	if (q.m_BMIH->biBitCount!=m_BMIH->biBitCount)
		return *this;		// Nicht Kompatible Bildformate
	if ((q.m_BMIH->biWidth>=m_BMIH->biWidth)&&
		(q.m_BMIH->biHeight>=m_BMIH->biHeight))
	{	// Quelle groesser als Ziel -> nur Ueberlappung bearbeiten
		int bpp = m_BMIH->biBitCount/8;			// Byte per pixel
		int rowofsq = q.m_BMIH->biWidth*bpp;	// Zeilenoffset quelle
		int rowofsd = m_BMIH->biWidth*bpp;		// Zeilenoffset ziel
		for (int y=0; y<m_BMIH->biHeight; y++)
			for (int x=0; x<rowofsd; x++)
				m_Image[x+y*rowofsd] = abs(q.m_Image[x+y*rowofsq]-m_Image[x+y*rowofsd]); 
	}
	else
		return *this;		// Quelle kleiner Ziel

	return *this;
}

void CDib::Empty()
{
	if (m_Image!=NULL) delete [] m_Image;
	if (m_BMIH!=NULL) delete m_BMIH;
	m_ImageSize=0;
	m_ColorTableEntries=0;
}

int CDib::GetWidth()
{
	if (m_BMIH!=NULL)
		return m_BMIH->biWidth;
	else 
		return -1;
}

int CDib::GetHeight()
{
	if (m_BMIH!=NULL)
		return m_BMIH->biHeight;
	else 
		return -1;
}

int CDib::GetBitsPerPixel()
{
	if (m_BMIH!=NULL)
		return m_BMIH->biBitCount;
	else 
		return -1;
}

int CDib::GetImageSize()
{
	if (m_BMIH!=NULL)
		return m_BMIH->biSizeImage;
	else 
		return -1;
}

BOOL CDib::CreateColorTable()
{
	if (m_BMIH==NULL)
		return false;
	if (m_ColorTableEntries!=256)
		return false;
	BYTE Graytable[256*sizeof(RGBQUAD)];
	int i;
	for (i=0; i<85; i++)
	{
		Graytable[i*sizeof(RGBQUAD)]=i*3;
		Graytable[i*sizeof(RGBQUAD)+1]=0;
		Graytable[i*sizeof(RGBQUAD)+2]=0;
		Graytable[i*sizeof(RGBQUAD)+3]=0;
	}
	for (i=0; i<85; i++)
	{
		Graytable[(85+i)*sizeof(RGBQUAD)]=(85-i)*3;
		Graytable[(85+i)*sizeof(RGBQUAD)+1]=i*3;
		Graytable[(85+i)*sizeof(RGBQUAD)+2]=0;
		Graytable[(85+i)*sizeof(RGBQUAD)+3]=0;
	}
	for (i=0; i<85; i++)
	{
		Graytable[(170+i)*sizeof(RGBQUAD)]=0;
		Graytable[(170+i)*sizeof(RGBQUAD)+1]=(85-i)*3;
		Graytable[(170+i)*sizeof(RGBQUAD)+2]=i*3;
		Graytable[(170+i)*sizeof(RGBQUAD)+3]=0;
	}
	Graytable[255*sizeof(RGBQUAD)]=0;
	Graytable[255*sizeof(RGBQUAD)+1]=0;
	Graytable[255*sizeof(RGBQUAD)+2]=255;
	Graytable[255*sizeof(RGBQUAD)+3]=0;
	memcpy(m_BMIH+1, Graytable, m_ColorTableEntries*sizeof(RGBQUAD));
	return true;
}


/*
 BMP Loader - a quick and dirty substitute for GLaux 
 if you only use GLaux to load BMP files will load any format of a 
 windows DIB BMP format graphics file Only works on a windows box   
 Caution! memory for the data is allocated using 'new'.  
 In the NeHe tutorials the memory is reclaimed using 'free'.   
 For the small tutorials its not a big deal but not a good practice in 
 larger projects (heap trashing not good). J.M. Doyle : 12 Jan 2003
*/

AUX_RGBImageRec *auxDIBImageLoad(const char *FileName)
{ 
	 return new AUX_RGBImageRec(FileName);
}

void AUX_RGBImageRec::convertBGRtoRGB()
{
	const DWORD BitmapLength = sizeX * sizeY * 3;
	byte Temp;  // not quick but it works  
	for(DWORD i=0; i< BitmapLength; i += 3) 
	{
	    Temp = data[i];
	    data[i] = data[i+2];
	    data[i+2] = Temp;
	    }
	}

AUX_RGBImageRec::AUX_RGBImageRec(const char *FileName): data(NULL), NoErrors(false)
{ 
	loadFile(FileName);
}

AUX_RGBImageRec::~AUX_RGBImageRec()
{
	if (data != NULL) delete data;
	data = NULL;
}

bool AUX_RGBImageRec::loadFile(const char* Filename)
{
	BITMAPINFO BMInfo;								// need the current OpenGL device contexts in order to make use of windows DIB utilities  
	const HDC gldc = wglGetCurrentDC();   			// a handle for the current OpenGL Device Contexts
					  								// assume there are errors until file is loaded successfully into memory  
	NoErrors = false;  								// release old data since this object could be used to load multiple Textures  
	if(data != NULL) delete data;					// windows needs this info to determine what header info we are looking for  
	BMInfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);  // Get windows to determine color bit depth in the file for us  
	BMInfo.bmiHeader.biBitCount = 0;				// Get windows to open and load the BMP file and handle the messy decompression if the file is compressed  
													// assume perfect world and no errors in reading file, Ha Ha  
	HANDLE DIBHandle = LoadImageA(0,Filename, IMAGE_BITMAP, 0, 0,LR_DEFAULTCOLOR | LR_CREATEDIBSECTION | LR_LOADFROMFILE);  // use windows to get header info of bitmap - assume no errors in header format 

	GetDIBits(gldc, (HBITMAP)DIBHandle, 0,0, NULL, &BMInfo, DIB_RGB_COLORS);
	sizeX = BMInfo.bmiHeader.biWidth;
	sizeY = BMInfo.bmiHeader.biHeight;				// change color depth to 24 bits (3 bytes (BGR) / pixel)  
	BMInfo.bmiHeader.biBitCount = 24;				// don't want the data compressed  
	BMInfo.bmiHeader.biCompression = BI_RGB;  
	const DWORD BitmapLength = sizeX * sizeY * 3;	// 3 bytes (BGR) per pixel (24bp)  
													// allocate enough memory to hold the pixel data in client memory  
	data = new byte[BitmapLength];					// Get windows to do the dirty work of converting the BMP into the format needed by OpenGL  
													// if file is already 24 bit color then this is a waste of time but makes for short code  
													// Get the actual Texel data from the BMP object  
	
	if (GetDIBits(gldc, (HBITMAP)DIBHandle, 0, sizeY, data, &BMInfo, DIB_RGB_COLORS)) 
	{
		NoErrors = true;
		convertBGRtoRGB();							// NOTE: BMP is in BGR format but OpenGL needs RGB unless you use GL_BGR_EXT
	}

	DeleteObject(DIBHandle);						// don't need the BMP Object anymore  
	return NoErrors;
}        
