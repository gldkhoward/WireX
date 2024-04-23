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

/*! \file WireCenter.h
 *
 *	\author   Andreas Pott
 *
 *  \brief The core application object. It does very little but initializing
 *  the application but some core functions have to be implemented on this
 *  highest level.
 */

#pragma once

#ifndef __AFXWIN_H__
	#error "\"stdafx.h\" vor dieser Datei für PCH einschließen"
#endif

#include "resource.h"       // Hauptsymbole
#include <strstream>

// CWireCenterApp:
// Siehe WireCenter.cpp für die Implementierung dieser Klasse
//

class CWireCenterApp : public CWinAppEx
{
public:
	CWireCenterApp();
	~CWireCenterApp();

// Überschreibungen
public:
	virtual BOOL InitInstance();

	static HANDLE rPipe;

// Implementierung
	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()
	afx_msg void OnHilfePythonscripting();
	virtual int ExitInstance();
	virtual BOOL OnIdle(LONG lCount);
	afx_msg void OnClipButton();
};

extern CWireCenterApp theApp;