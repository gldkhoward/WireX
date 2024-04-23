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
 *	\author   Andreas Pott
 *
 *  \dependency
 *		eigen3      for vector data-types used in wirelib
 *
 *  \brief Provides a configuration dialogs for workspace computation
 */

#pragma once

// CWorkspaceOptionsDlg-Dialogfeld

class CWorkspaceOptionsDlg : public CDialog
{
	DECLARE_DYNAMIC(CWorkspaceOptionsDlg)

public:
	CWorkspaceOptionsDlg(CWnd* pParent = NULL);   // Standardkonstruktor
	virtual ~CWorkspaceOptionsDlg();

// Dialogfelddaten
	enum { IDD = IDD_WORKSPACE_OPTIONS_DIALOG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV-Unterstützung

	DECLARE_MESSAGE_MAP()
public:
	double m_x;
	double m_y;
	double m_z;
	double m_eps;
	double m_MinCond;
	double m_maxRadius;
	double m_fx;
	double m_fy;
	double m_fz;
	double m_tx;
	double m_ty;
	double m_tz;

public:
	int m_Iterations;
public:
	afx_msg void OnBnClickedOk();
	double m_fmin;
	double m_fmax;
	BOOL m_CheckAllOrientations;
	int m_Criterion_Selector;
	int m_WorkspaceMethod_Selector;
	int m_WorkspaceCriterion_Selector;
};
