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

/*! \file PoseEvaluatorDlg.h
 *
 *	\author   Andreas Pott
 *
 *  \brief 
 */

#pragma once
#include "afxwin.h"
#include "afxpropertygridctrl.h"
#include <map>
#include <WireLib/PosePropertyEvaluation.h>
#include <WireLib/RobotDocument.h>

// CPoseEvaluatorDlg-Dialogfeld

class CPoseEvaluatorDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CPoseEvaluatorDlg)

	void UpdateLists();

public:
	CPoseEvaluatorDlg(CWnd* pParent = NULL);   // Standardkonstruktor
	virtual ~CPoseEvaluatorDlg();

	PCRL::CPosePropertyEvaluation *pEvaluator;

// Dialogfelddaten
	enum { IDD = IDD_POSEEVALUATOR_DIALOG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV-Unterstützung

	DECLARE_MESSAGE_MAP()
public:
	int m_PosePropertyCount;
	int m_AttributeCount;
	int m_bSetZeroThreshold;
	CListBox m_ActiveProperties;
	CListBox m_PosePropertyTypes;
	CListBox m_Attibutes;
	CMFCPropertyGridCtrl m_Configuration;
	afx_msg void OnBnClickedAddPosepropertyButton();
	afx_msg void OnBnClickedRemovePosepropertyButton();
	afx_msg void OnBnClickedUpPosepropertyButton();
	afx_msg void OnBnClickedDownPosepropertyButton();
	afx_msg void OnLbnSelchangeActiveEvaluatorList();
	virtual BOOL OnInitDialog();
	afx_msg void OnLbnDblclkActiveEvaluatorList();
	afx_msg void OnLbnDblclkInactiveEvaluatorList();
	LRESULT CPoseEvaluatorDlg::OnPropertyChanged(__in WPARAM wparam, __in LPARAM lparam);
	afx_msg void OnBnClickedCheckzerothreshold();
};
