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

// LegGeometryDlg.cpp: Implementierungsdatei
//

#include "stdafx.h"
#include "WireCenter.h"
#include "LegGeometryDlg.h"


// CLegGeometryDlg-Dialogfeld

IMPLEMENT_DYNAMIC(CLegGeometryDlg, CDialog)

BOOL CLegGeometryDlg::OnInitDialog()
{
	CDialog::OnInitDialog();
	CString chr;
	chr.Format(_T("Leg Geometry %i"), legid);
	SetWindowText(chr);
	return true;// return TRUE unless you set the focus to a control               
}


CLegGeometryDlg::CLegGeometryDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CLegGeometryDlg::IDD, pParent)
	, m_bx(0)
	, m_by(0)
	, m_bz(0)
	, m_px(0)
	, m_py(0)
	, m_pz(0)
{

}

CLegGeometryDlg::~CLegGeometryDlg()
{
}

void CLegGeometryDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_BX_EDIT, m_bx);
	DDX_Text(pDX, IDC_BY_EDIT, m_by);
	DDX_Text(pDX, IDC_BZ_EDIT, m_bz);
	DDX_Text(pDX, IDC_PX_EDIT, m_px);
	DDX_Text(pDX, IDC_PY_EDIT, m_py);
	DDX_Text(pDX, IDC_PZ_EDIT, m_pz);
}


BEGIN_MESSAGE_MAP(CLegGeometryDlg, CDialog)
END_MESSAGE_MAP()


// CLegGeometryDlg-Meldungshandler
