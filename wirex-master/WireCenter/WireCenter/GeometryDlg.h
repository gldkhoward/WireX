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

/*! \file GeometryDlg.h
 *
 *	\author   Andreas Pott
 *
 *  \brief A simple dialog box to edit the geometrical
 *  parameters of a wire robot. The data is directly stored
 *  to an CRobotData object; typically the one in the document
 *  is used.
 */

#pragma once
#include "afxcmn.h"
#include <WireLib/RobotData.h>

// CGeometryDlg-Dialogfeld

class CGeometryDlg : public CDialog
{
	DECLARE_DYNAMIC(CGeometryDlg)

public:
	CGeometryDlg(CWnd* pParent = NULL);   // Standardkonstruktor
	virtual ~CGeometryDlg();

	PCRL::CRobotData *pRobot;

// Dialogfelddaten
	enum { IDD = IDD_ROBOT_GEOMETRY_DIALOG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV-Unterstützung

	DECLARE_MESSAGE_MAP()
public:
	CListCtrl m_List;
public:
	virtual BOOL OnInitDialog();
};
