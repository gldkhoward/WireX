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
 *	\author   Daniel Küthe
 *
 *  \dependency
 *		eigen3      for vector data-types used in wirelib
 *
 *  \brief Provides a number of configuration dialogs.
 */

#pragma once

#include <WireLib/ApplicationRequirement.h>
#include <WireLib/RobotData.h>

// ONE HEADER FILE FOR ALL DRAFT DIALOGS

/////////////////////////////////////////////////////
// "Describe application" dialog
/////////////////////////////////////////////////////

class CDescribeApplicationDlg : public CDialog
{
	DECLARE_DYNAMIC(CDescribeApplicationDlg)

public:
	CDescribeApplicationDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~CDescribeApplicationDlg();
	bool m_bSimplifiedDialog;

// Dialog Data
	enum { IDD = IDD_DESCRIBE_APPLICATION };

	PCRL::CApplicationRequirement Req;
	void setRequirements(PCRL::CApplicationRequirement& value) { value = Req; }
	void getRequirements(const PCRL::CApplicationRequirement& value) { Req = value; }
	void calculateValues(int choosen_output_radiobutton_id);
	CString ToString(double value);
private:
	int m_previouslycheckedRadioButton;
	double m_delta_minus_edit_x_value;
	double m_delta_minus_edit_y_value;
	double m_delta_minus_edit_z_value;

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	void ActivateDeltaEditFields(bool visible);
	void ActiveSimplifiedDialog();
	virtual BOOL OnInitDialog();
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedReset();
	afx_msg void OnBnClickedRadiobutton();
};

/////////////////////////////////////////////////////
// "Edit drum parameters" dialog
/////////////////////////////////////////////////////

class CEditDrumParametersDlg : public CDialog
{
	DECLARE_DYNAMIC(CEditDrumParametersDlg)

public:
	CEditDrumParametersDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~CEditDrumParametersDlg();

// Dialog Data
	enum { IDD = IDD_EDIT_DRUMPARAMETERS };

	PCRL::CWinchParameter settings;
	void setWinchSettings(PCRL::CWinchParameter& value) { value = settings; }
	void getWinchSettings(PCRL::CWinchParameter& value) { settings = value; }
private:

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	virtual BOOL OnInitDialog();
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedReset();
};

/////////////////////////////////////////////////////
// "Edit wire parameters" dialog
/////////////////////////////////////////////////////

class CEditWireParametersDlg : public CDialog
{
	DECLARE_DYNAMIC(CEditWireParametersDlg)

public:
	CEditWireParametersDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~CEditWireParametersDlg();

// Dialog Data
	enum { IDD = IDD_EDIT_WIREPARAMETERS };

	PCRL::CCableParameter settings;
	void setWireSettings(PCRL::CCableParameter& value) { value = settings; }
	void getWireSettings(PCRL::CCableParameter& value) { settings = value; }
private:

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	virtual BOOL OnInitDialog();
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedReset();
};