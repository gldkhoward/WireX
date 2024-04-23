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

// WorkspaceOptionsDlg.cpp: Implementierungsdatei
//

#include "stdafx.h"
#include "WireCenter.h"
#include "WorkspaceOptionsDlg.h"


// CWorkspaceOptionsDlg-Dialogfeld

IMPLEMENT_DYNAMIC(CWorkspaceOptionsDlg, CDialog)

CWorkspaceOptionsDlg::CWorkspaceOptionsDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CWorkspaceOptionsDlg::IDD, pParent)
	, m_x(0)
	, m_y(0)
	, m_z(0)
	, m_fx(0)
	, m_fy(0)
	, m_fz(0)
	, m_tx(0)
	, m_ty(0)
	, m_tz(0)
	, m_eps(0)
	, m_MinCond(0)
	, m_maxRadius(0)
	, m_Iterations(0)
	, m_fmin(0)
	, m_fmax(0)
	, m_CheckAllOrientations(FALSE)
	, m_Criterion_Selector(0)
	, m_WorkspaceMethod_Selector(0)
	, m_WorkspaceCriterion_Selector(0)
{
}

CWorkspaceOptionsDlg::~CWorkspaceOptionsDlg()
{
}

void CWorkspaceOptionsDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_XCENTER_EDIT, m_x);
	DDX_Text(pDX, IDC_YCENTER_EDIT, m_y);
	DDX_Text(pDX, IDC_ZCENTER_EDIT, m_z);
	DDX_Text(pDX, IDC_FORCEX_EDIT, m_fx);
	DDX_Text(pDX, IDC_FORCEY_EDIT, m_fy);
	DDX_Text(pDX, IDC_FORCEZ_EDIT, m_fz);
	DDX_Text(pDX, IDC_TORQUEX_EDIT, m_tx);
	DDX_Text(pDX, IDC_TORQUEY_EDIT, m_ty);
	DDX_Text(pDX, IDC_TORQUEZ_EDIT, m_tz);
	DDX_Text(pDX, IDC_EPS_EDIT, m_eps);
	DDX_Text(pDX, IDC_MINCONDITIONNUMBER_EDIT, m_MinCond);
	DDX_Text(pDX, IDC_MAXRADIUS_EDIT, m_maxRadius);
	DDX_Text(pDX, IDC_ITERATIONS_EDIT, m_Iterations);
	DDX_Text(pDX, IDC_FMIN_EDIT, m_fmin);
	DDX_Text(pDX, IDC_FMAX_EDIT, m_fmax);
	DDX_Check(pDX, IDC_ALLORIENTATIONS_CHECK, m_CheckAllOrientations);
	DDX_CBIndex(pDX, IDC_METHOD_COMBO, m_Criterion_Selector);
	DDX_CBIndex(pDX, IDC_WORKSPACE_METHOD_COMBO, m_WorkspaceMethod_Selector);
	DDX_CBIndex(pDX, IDC_WORKSPACE_CRITERION_COMBO, m_WorkspaceCriterion_Selector);
}

BEGIN_MESSAGE_MAP(CWorkspaceOptionsDlg, CDialog)
	ON_BN_CLICKED(IDOK, &CWorkspaceOptionsDlg::OnBnClickedOk)
END_MESSAGE_MAP()


// CWorkspaceOptionsDlg-Meldungshandler

void CWorkspaceOptionsDlg::OnBnClickedOk()
{
	UpdateData(TRUE);
	OnOK();
}
