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

// GeometryDlg.cpp: Implementierungsdatei
//

#include "stdafx.h"
#include "WireCenter.h"
#include "GeometryDlg.h"


// CGeometryDlg-Dialogfeld

IMPLEMENT_DYNAMIC(CGeometryDlg, CDialog)

CGeometryDlg::CGeometryDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CGeometryDlg::IDD, pParent)
{
	pRobot=0;
}

CGeometryDlg::~CGeometryDlg()
{
}

void CGeometryDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_GEOMETRY_LIST, m_List);
}


BEGIN_MESSAGE_MAP(CGeometryDlg, CDialog)
END_MESSAGE_MAP()


// CGeometryDlg-Meldungshandler

BOOL CGeometryDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	m_List.InsertColumn(0,"Leg",LVCFMT_RIGHT,40);
	m_List.InsertColumn(1,"Platform x",LVCFMT_RIGHT,75);
	m_List.InsertColumn(2,"Platform y",LVCFMT_RIGHT,75);
	m_List.InsertColumn(3,"Platform z",LVCFMT_RIGHT,75);
	m_List.InsertColumn(4,"Base x",LVCFMT_RIGHT,75);
	m_List.InsertColumn(5,"Base y",LVCFMT_RIGHT,75);
	m_List.InsertColumn(6,"Base z",LVCFMT_RIGHT,75);

	for (int i=0; i<pRobot->getNow(); i++)
	{
		CString str;
		str.Format("%d",i+1);
		m_List.InsertItem(i,str);
		str.Format("%f",pRobot->getBase(i).x()); m_List.SetItemText(i,4,str);
		str.Format("%f",pRobot->getBase(i).y()); m_List.SetItemText(i,5,str);
		str.Format("%f",pRobot->getBase(i).z()); m_List.SetItemText(i,6,str);
		str.Format("%f",pRobot->getPlatform(i).x()); m_List.SetItemText(i,1,str);
		str.Format("%f",pRobot->getPlatform(i).y()); m_List.SetItemText(i,2,str);
		str.Format("%f",pRobot->getPlatform(i).z()); m_List.SetItemText(i,3,str);
	}
	return TRUE;  // return TRUE unless you set the focus to a control
	// AUSNAHME: OCX-Eigenschaftenseite muss FALSE zurückgeben.
}
