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

/*! \file DraftDlg.h
 *
 *	\author   Daniel K�the
 *
 *  \dependency
 *		eigen3      for vector data-types used in wirelib
 *
 *  \brief Provides a configuration dialogs for the NC-interpolator.
 */

#pragma once

#include "resource.h"
#include <motionPlanning/NcInterpolator.h>


// CNCOptionsDlg-Dialogfeld

class CNCOptionsDlg : public CDialog
{
	DECLARE_DYNAMIC(CNCOptionsDlg)

public:
	CNCOptionsDlg(CWnd* pParent = NULL);   // Standardkonstruktor
	virtual ~CNCOptionsDlg();

// Dialogfelddaten
	enum { IDD = IDD_NC_OPTIONSDLG };

//the interpolator data
	double dOverride,amax,jmax,cycleTime;
	int iBezier,iInterpolationMethod;


protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV-Unterst�tzung

	DECLARE_MESSAGE_MAP()

public:
	virtual BOOL OnInitDialog();
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedReset();
};
