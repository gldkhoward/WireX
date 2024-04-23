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
 *  \brief Provides a configuration dialogs for "New Robot"
 */

#pragma once

#include <WireLib/RobotData.h>
#include <WireLib/RobotDocument.h>

// CRobotPropertiesDlg-Dialogfeld

class CRobotPropertiesDlg : public CDialog
{
	DECLARE_DYNAMIC(CRobotPropertiesDlg)

public:
	CRobotPropertiesDlg(CWnd* pParent = NULL);   // Standardkonstruktor
	virtual ~CRobotPropertiesDlg();

// Dialogfelddaten
	enum { IDD = IDD_NEWROBOTDLG };

	PCRL::CRobotData::MotionPatternType getMotionPattern();
	PCRL::CRobotDocument RDoc;

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV-Unterstützung
	
	//custom functions
	void ListSuitableRobotModels(CComboBox* pTarget, bool b_effects_frame, bool b_effects_platform, bool b_is_transformator); // List all robot models with matching "dof" and "now"
	void BuiltListPropertyDlg(CStatic* pTarget_space_defined_by_image, CMFCPropertyGridCtrl* pTarget_property_control, CComboBox* pCombobox_for_choosing); //built the property control
	void BuiltTransformationListCtrl(int entry_number);
	void ApplyValuesFromPropertyGrid(CMFCPropertyGridCtrl* pPropList);

	DECLARE_MESSAGE_MAP()
	//! ID Nummer des Motion Pattern
	int m_motionPattern;

	//the CMFCPropertyGrid controls locations
	CStatic m_wndPropListLocation;
	CStatic m_wndPropListLocation2;
	CStatic m_wndPropListLocation3;

	//the CMFCPropertyGrid controls themselves
	CMFCPropertyGridCtrl m_wndPropList;
	CMFCPropertyGridCtrl m_wndPropList2;
	CMFCPropertyGridCtrl m_wndPropList3;

	//the comboboxes
	CComboBox m_parametricrobots_combobox;
	CComboBox m_parametricrobots2_combobox;

	//the radiobuttons
	CButton m_completedesign_radio;
	CButton m_seperateconfiguration_radio;

	//the checkboxes
	CButton m_transformframe_checkbox;
	CButton m_transformplatform_checkbox;

public:
	//! Anzahl der Seile
	int m_now;
	//! Anzahl der Freiheitsgrade (ergibt sich aus dem MotionPattern)
	int m_dof;
	CString m_Name;
	CString m_Author;
	CString m_ID;
	CString m_Desc;
	virtual BOOL OnInitDialog();
	afx_msg void OnEnChangeNumberofwiresEdit();
	afx_msg void OnCbnSelchangeMotionpatternCombo();
	afx_msg void OnCbnSelchangeParametricrobotsCombo();
	afx_msg void OnClickedRadio();
	afx_msg void OnCbnSelchangeParametricrobots2Combo();
	afx_msg void OnClickedCheckbox();
	afx_msg void OnBnClickedOk();
};
