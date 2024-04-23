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

/*! \file LegGeometryDlg.h
 *
 *	\author   Andreas Pott
 *
 *  \dependency
 *		eigen3      for vector data-types used in wirelib
 *
 *  \brief Simple Dialog box to edit proximal and distal anchor point coordinates
 */

#pragma once


// CLegGeometryDlg-Dialogfeld

class CLegGeometryDlg : public CDialog
{
	DECLARE_DYNAMIC(CLegGeometryDlg)

public:
	CLegGeometryDlg(CWnd* pParent = NULL);   // Standardkonstruktor
	virtual ~CLegGeometryDlg();
	BOOL OnInitDialog();

// Dialogfelddaten
	enum { IDD = IDD_LEGGEOMETRY_DIALOG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV-Unterstützung

	DECLARE_MESSAGE_MAP()
public:
	double m_bx;
	double m_by;
	double m_bz;
	double m_px;
	double m_py;
	double m_pz;
	int legid;
	
};
