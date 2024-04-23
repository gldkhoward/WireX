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

// NCDlg.cpp : implementation file for nc-interpolator settings dialog
//

#include "stdafx.h"
#include "NCDlg.h"
#include "afxdialogex.h"
#include <sstream>

/////////////////////////////////////////////////////////////
// CNCOptionsDlg dialog
/////////////////////////////////////////////////////////////

IMPLEMENT_DYNAMIC(CNCOptionsDlg, CDialog)

CNCOptionsDlg::CNCOptionsDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CNCOptionsDlg::IDD, pParent)
{
#ifndef _WIN32_WCE
	EnableActiveAccessibility();
#endif
}

CNCOptionsDlg::~CNCOptionsDlg()
{
}

void CNCOptionsDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//connect IDs from the control with their variables
	DDX_Text(pDX, IDC_NCOPTIONS_MAXJERK_EDIT, jmax);
	DDX_Text(pDX, IDC_NCOPTIONS_MAXACCERLERATION_EDIT, amax);
	DDX_Text(pDX, IDC_NCOPTIONS_MAXOVERRIDE_EDIT, dOverride);
	DDX_Text(pDX, IDC_NCOPTIONS_CYCLETIME_EDIT, cycleTime);
	DDX_Check(pDX, IDC_NCOPTIONS_BEZIER_CHECKBOX, iBezier);
}


BEGIN_MESSAGE_MAP(CNCOptionsDlg, CDialog)
	ON_BN_CLICKED(IDOK, &CNCOptionsDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDC_RESET_NCOPTIONS, &CNCOptionsDlg::OnBnClickedReset)
END_MESSAGE_MAP()


// DescribeApplicationDlg message handlers

BOOL CNCOptionsDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	//get the values from the variables into the linked controls
	UpdateData(false);

	//special treatment for the radiobutton controls
	if (iInterpolationMethod==1)  CheckDlgButton(IDC_NCOPTIONS_INTERPOLATIONMETHOD_SIMPLE_RADIO,BST_CHECKED);
	if (iInterpolationMethod==2) CheckDlgButton(IDC_NCOPTIONS_INTERPOLATIONMETHOD_ENHANCED_RADIO,BST_CHECKED);
	
	return TRUE;  // return TRUE unless you set the focus to a controlE
}


void CNCOptionsDlg::OnBnClickedOk()
{
	//get the values from the controls into the variables
	//but calculate the correct value
	CDialog::OnOK();

	//get the values from the controls into the variables
	UpdateData(true);

	//special treatment for the radiobutton controls
	if (IsDlgButtonChecked(IDC_NCOPTIONS_INTERPOLATIONMETHOD_SIMPLE_RADIO)) iInterpolationMethod=1;
	if (IsDlgButtonChecked(IDC_NCOPTIONS_INTERPOLATIONMETHOD_ENHANCED_RADIO)) iInterpolationMethod=2;
	
}


void CNCOptionsDlg::OnBnClickedReset()
{
	//get the values from the variables into the controls
	UpdateData(false);

}