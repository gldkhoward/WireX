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

// OrientationDlg.cpp : implementation file
//

#include "stdafx.h"
#include "WireCenter.h"
#include "OrientationDlg.h"
#include "afxdialogex.h"
#include <motionPlanning/Utilities.h>

// COrientationDlg dialog

IMPLEMENT_DYNAMIC(COrientationDlg, CDialogEx)

COrientationDlg::COrientationDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(COrientationDlg::IDD, pParent)
	, m_a(0)
	, m_b(0)
	, m_c(0)
	, m_mina(0)
	, m_minb(0)
	, m_minc(0)
	, m_maxa(0)
	, m_maxb(0)
	, m_maxc(0)
	, m_steps(10)		//!< number of steps for discretisation
	, m_OrientationInfoSource(0)
	, m_OrientationCriterion(0)
	, m_OrientationSetSize(0)
{
}

COrientationDlg::~COrientationDlg()
{
}

void COrientationDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX,IDC_ROTA_EDIT    ,  m_a);
	DDX_Text(pDX,IDC_ROTB_EDIT    ,  m_b);
	DDX_Text(pDX,IDC_ROTC_EDIT    ,  m_c);
	DDX_Text(pDX,IDC_ROTA_MIN_EDIT,  m_mina);
	DDX_Text(pDX,IDC_ROTB_MIN_EDIT,  m_minb);
	DDX_Text(pDX,IDC_ROTC_MIN_EDIT,  m_minc);
	DDX_Text(pDX,IDC_ROTA_MAX_EDIT,  m_maxa);
	DDX_Text(pDX,IDC_ROTB_MAX_EDIT,  m_maxb);
	DDX_Text(pDX,IDC_ROTC_MAX_EDIT,  m_maxc);
	DDX_Text(pDX,IDC_NUMBER_DISCRETSTEPS_EDIT, m_steps);
	DDX_Radio(pDX, IDC_ORIENTATION_SOURCE_RADIO, m_OrientationInfoSource);
	DDX_Radio(pDX, IDC_ORIENTATION_CRITERION_RADIO, m_OrientationCriterion);
	DDX_Text(pDX, IDC_ORIENTATION_SET_SIZE_EDIT, m_OrientationSetSize);
}


BEGIN_MESSAGE_MAP(COrientationDlg, CDialogEx)
	ON_BN_CLICKED(IDC_REPLACE_SINGLE_ORIENTATION_BUTTON, &COrientationDlg::OnBnClickedReplaceSingleOrientationButton)
	ON_BN_CLICKED(IDC_ADD_SINGLE_ORIENTATION_BUTTON, &COrientationDlg::OnBnClickedAddSingleOrientationButton)
	ON_BN_CLICKED(IDC_REPLACE_ORIENTATION_SPECTRUM_BUTTON, &COrientationDlg::OnBnClickedReplaceOrientationSpectrumButton)
	ON_BN_CLICKED(IDC_ADD_ORIENTATION_SPECTURM_BUTTON, &COrientationDlg::OnBnClickedAddOrientationSpecturmButton)
END_MESSAGE_MAP()


// COrientationDlg message handlers


void COrientationDlg::OnBnClickedReplaceSingleOrientationButton()
{
	UpdateData(TRUE);
	Matrix3d R;
	PCRL::getMatrixFromXYZ(R,Vector3d(m_a*DEG_TO_RAD, m_b*DEG_TO_RAD, m_c*DEG_TO_RAD));
	m_pWA->setOrientation(R);
	// update the control items
	m_OrientationSetSize = m_pWA->getOrientationCount();
	UpdateData(FALSE);
}


void COrientationDlg::OnBnClickedAddSingleOrientationButton()
{
	UpdateData(TRUE);
	Matrix3d R;
	PCRL::getMatrixFromXYZ(R,Vector3d(m_a*DEG_TO_RAD, m_b*DEG_TO_RAD, m_c*DEG_TO_RAD));
	m_pWA->addOrientation(R);
	// update the control items
	m_OrientationSetSize = m_pWA->getOrientationCount();
	UpdateData(FALSE);
}


void COrientationDlg::OnBnClickedReplaceOrientationSpectrumButton()
{
	// before implementing this function, make sure you enabled the function in the MESSAGE_MAP
}


void COrientationDlg::OnBnClickedAddOrientationSpecturmButton()
{
	UpdateData(TRUE);
	// valide the data set in the dialog box
	if (m_mina > m_maxa)
		swap(m_mina, m_maxa);
	if (m_minb > m_maxb)
		swap(m_minb, m_maxb);
	if (m_minc > m_maxc)
		swap(m_minc, m_maxc);
	if (m_steps < 2)
		m_steps = 2;

	Matrix3d R;
	const double eps = 1e-6;
	// loop through the desired angles and add the respective orientations
	for (double a=m_mina; a<=m_maxa; a+=(m_maxa-m_mina+eps)/m_steps)
		for (double b=m_minb; b<=m_maxb; b+=(m_maxb-m_minb+eps)/m_steps)
			for (double c=m_minc; c<=m_maxc; c+=(m_maxc-m_minc+eps)/m_steps)
			{
				PCRL::getMatrixFromXYZ(R,Vector3d(a*DEG_TO_RAD, b*DEG_TO_RAD, c*DEG_TO_RAD));
				m_pWA->addOrientation(R);
			}
	
	// update the control items		
	m_OrientationSetSize = m_pWA->getOrientationCount();
	UpdateData(FALSE);
}


BOOL COrientationDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// update the control items		
	m_OrientationSetSize = m_pWA->getOrientationCount();
	UpdateData(FALSE);

	return TRUE;  // return TRUE unless you set the focus to a control
}
