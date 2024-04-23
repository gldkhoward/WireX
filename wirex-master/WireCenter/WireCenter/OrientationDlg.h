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

/*! \file OrientationDlg.h
 *
 *	\author   Andreas Pott
 *
 *  \brief Simple modal dialog box to configure the orientation 
 *  for workspace calculation. The class only cares for data 
 *  transfer between the respective dialog resource and the
 *  data members.
 */

#pragma once

#include <WireLib/Workspace.h>

// COrientationDlg dialog

class COrientationDlg : public CDialogEx
{
	DECLARE_DYNAMIC(COrientationDlg)

public:
	COrientationDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~COrientationDlg();

public:
	// the variables connected to the edits of the dialog box
	double m_a;
	double m_b;
	double m_c;
	double m_mina;
	double m_minb;
	double m_minc;
	double m_maxa;
	double m_maxb;
	double m_maxc;
	int m_steps;		//!< number of steps for discretisation
	int m_OrientationInfoSource;
	int m_OrientationCriterion;
	int m_OrientationSetSize;			//!< the current number of elements in the orientation set to be displayed in the dialog box
	PCRL::CWorkspaceAlgorithm *m_pWA;	//!< a pointer to the workspace algorithm object to be configured by the dialog box

// Dialog Data
	enum { IDD = IDD_ORIENTATION_DIALOG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedReplaceSingleOrientationButton();
	afx_msg void OnBnClickedAddSingleOrientationButton();
	afx_msg void OnBnClickedReplaceOrientationSpectrumButton();
	afx_msg void OnBnClickedAddOrientationSpecturmButton();
	
	virtual BOOL OnInitDialog();
};
