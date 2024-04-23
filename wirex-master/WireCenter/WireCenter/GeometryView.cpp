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

// GeometryView.cpp: Implementierungsdatei
//

#include "stdafx.h"
#include "WireCenter.h"
#include "GeometryView.h"
#include "LegGeometryDlg.h"

// CGeometryView

IMPLEMENT_DYNCREATE(CGeometryView, CFormView)

CGeometryView::CGeometryView()
	: CFormView(CGeometryView::IDD)
	, m_dx(0)
	, m_dy(0)
	, m_dz(0)
	, m_da(0)
	, m_db(0)
	, m_dc(0)
	, m_ds(1)
	, m_l(1)
	, m_b(1)
	, m_h(1)
	, m_GeometryMode(0)
{
}

CGeometryView::~CGeometryView()
{
}

void CGeometryView::DoDataExchange(CDataExchange* pDX)
{
	CFormView::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_GEOMETRYVIEW_LIST, m_List);
	DDX_Text(pDX, IDC_TRANSLATE_X_EDIT, m_dx);
	DDX_Text(pDX, IDC_TRANSLATE_Y_EDIT, m_dy);
	DDX_Text(pDX, IDC_TRANSLATE_Z_EDIT, m_dz);
	DDX_Text(pDX, IDC_ROTATE_A_EDIT, m_da);
	DDX_Text(pDX, IDC_ROTATE_B_EDIT, m_db);
	DDX_Text(pDX, IDC_ROTATE_C_EDIT, m_dc);
	DDX_Text(pDX, IDC_SCALE_EDIT, m_ds);
	DDX_Text(pDX, IDC_GEOMETRY_L_EDIT, m_l);
	DDX_Text(pDX, IDC_GEOMETRY_B_EDIT, m_b);
	DDX_Text(pDX, IDC_GEOMETRY_H_EDIT, m_h);
	DDX_Radio(pDX, IDC_GEOMETRYEDIT_MODUS_RADIO1, m_GeometryMode);
}

BEGIN_MESSAGE_MAP(CGeometryView, CFormView)
	ON_NOTIFY(NM_CLICK, IDC_GEOMETRYVIEW_LIST, &CGeometryView::OnNMClickGeometryviewList)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_TRANSLATE_BUTTON, &CGeometryView::OnBnClickedTranslateButton)
	ON_BN_CLICKED(IDC_ROTATE_BUTTON, &CGeometryView::OnBnClickedRotateButton)
	ON_BN_CLICKED(IDC_SCALE_BUTTON, &CGeometryView::OnBnClickedScaleButton)
	ON_BN_CLICKED(IDC_CREATE_CUBE_BUTTON, &CGeometryView::OnBnClickedCreateCubeButton)
END_MESSAGE_MAP()


// CGeometryView-Diagnose

#ifdef _DEBUG
void CGeometryView::AssertValid() const
{
	CFormView::AssertValid();
}

#ifndef _WIN32_WCE
void CGeometryView::Dump(CDumpContext& dc) const
{
	CFormView::Dump(dc);
}
#endif
#endif //_DEBUG


// CGeometryView-Meldungshandler

void CGeometryView::OnInitialUpdate()
{
	CFormView::OnInitialUpdate();

	CreateGeometryTable();
	UpdateGeometryData();
}

// CWireCenterView-Diagnose

#ifdef _DEBUG
CWireCenterDoc* CGeometryView::GetDocument() const // Nicht-Debugversion ist inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CWireCenterDoc)));
	return (CWireCenterDoc*)m_pDocument;
}
#endif //_DEBUG

void CGeometryView::CreateGeometryTable()
{
	m_List.InsertColumn(0,"Leg",LVCFMT_RIGHT,40);
	m_List.InsertColumn(1,"Platform x",LVCFMT_RIGHT,75);
	m_List.InsertColumn(2,"Platform y",LVCFMT_RIGHT,75);
	m_List.InsertColumn(3,"Platform z",LVCFMT_RIGHT,75);
	m_List.InsertColumn(4,"Base x",LVCFMT_RIGHT,75);
	m_List.InsertColumn(5,"Base y",LVCFMT_RIGHT,75);
	m_List.InsertColumn(6,"Base z",LVCFMT_RIGHT,75);
}

void CGeometryView::UpdateGeometryData()
{
	PCRL::CRobotData* pRobot=&GetDocument()->robotDoc;
	m_List.DeleteAllItems();
	for (int i=0; i<pRobot->getNow(); i++)
	{
/*		CString str;
		str.Format("%d",i+1);
		m_List.InsertItem(i,str);
		str.Format("%f",pRobot->getBase(i).x()); m_List.SetItemText(i,4,str);
		str.Format("%f",pRobot->getBase(i).y()); m_List.SetItemText(i,5,str);
		str.Format("%f",pRobot->getBase(i).z()); m_List.SetItemText(i,6,str);
		str.Format("%f",pRobot->getPlatform(i).x()); m_List.SetItemText(i,1,str);
		str.Format("%f",pRobot->getPlatform(i).y()); m_List.SetItemText(i,2,str);
		str.Format("%f",pRobot->getPlatform(i).z()); m_List.SetItemText(i,3,str);*/
		CString str;
		str.Format("%d    % 10.6f    % 4.6f    % 4.6f    % 4.6f    % 4.6f    % 4.6f",
			i+1,
			pRobot->getBase(i).x(),
			pRobot->getBase(i).y(),
			pRobot->getBase(i).z(),
			pRobot->getPlatform(i).x(),
			pRobot->getPlatform(i).y(),
			pRobot->getPlatform(i).z());
		m_List.InsertItem(i,str);
	}
}

void CGeometryView::OnNMClickGeometryviewList(NMHDR *pNMHDR, LRESULT *pResult)
{
	tagNMITEMACTIVATE *pTag = (tagNMITEMACTIVATE*)pNMHDR;
	
	if (pTag->iItem!=-1)
	{
		PCRL::CRobotData* pRobot=&GetDocument()->robotDoc;
		CLegGeometryDlg LGD;
		LGD.legid=pTag->iItem+1;
		LGD.m_bx = pRobot->getBase(pTag->iItem).x();
		LGD.m_by = pRobot->getBase(pTag->iItem).y();
		LGD.m_bz = pRobot->getBase(pTag->iItem).z();
		LGD.m_px = pRobot->getPlatform(pTag->iItem).x();
		LGD.m_py = pRobot->getPlatform(pTag->iItem).y();
		LGD.m_pz = pRobot->getPlatform(pTag->iItem).z();
		if (LGD.DoModal()==IDOK)
		{
			pRobot->setLeg(pTag->iItem,Vector3d(LGD.m_bx,LGD.m_by,LGD.m_bz),Vector3d(LGD.m_px,LGD.m_py,LGD.m_pz));
		}
	}
	// TODO: Fügen Sie hier Ihren Kontrollbehandlungscode für die Benachrichtigung ein.
	UpdateGeometryData();
	*pResult = 0;
}

void CGeometryView::OnUpdate(CView* /*pSender*/, LPARAM /*lHint*/, CObject* /*pHint*/)
{
	UpdateGeometryData();
}

void CGeometryView::OnTimer(UINT_PTR nIDEvent)
{
	CFormView::OnTimer(nIDEvent);
}

void CGeometryView::OnBnClickedTranslateButton()
{
	UpdateData();
	PCRL::CRobotData* pRobot=&GetDocument()->robotDoc;
	Vector3d r(m_dx,m_dy,m_dz);
	if (m_GeometryMode==0)
		pRobot->translateFrameGeometry(r);
	else
		pRobot->translatePlatformGeometry(r);
	UpdateGeometryData();
}

void CGeometryView::OnBnClickedRotateButton()
{
	UpdateData();
	PCRL::CRobotData* pRobot=&GetDocument()->robotDoc;
	Matrix3d R=Matrix3d::ZRotationMatrix3d(m_dc*DEG_TO_RAD)*Matrix3d::YRotationMatrix3d(m_db*DEG_TO_RAD)*Matrix3d::XRotationMatrix3d(m_da*DEG_TO_RAD);
	if (m_GeometryMode==0)
		pRobot->rotateFrameGeoemetry(R);
	else
		pRobot->rotatePlatformGeoemetry(R);
	UpdateGeometryData();
}

void CGeometryView::OnBnClickedScaleButton()
{
	UpdateData();
	PCRL::CRobotData* pRobot=&GetDocument()->robotDoc;
	Vector3d s(m_ds,m_ds,m_ds);
	if (m_GeometryMode==0)
		pRobot->scaleFrameGeometry(s);
	else
		pRobot->scalePlatformGeometry(s);
	UpdateGeometryData();
}

void CGeometryView::OnBnClickedCreateCubeButton()
{
	UpdateData();
	PCRL::CRobotData* pRobot=&GetDocument()->robotDoc;
	if (m_GeometryMode==0)
		pRobot->setFrameGeometryCube(m_l,m_b,m_h);
	else
		pRobot->setPlatformGeometryCube(m_l,m_b,m_h);
	UpdateGeometryData();
}

// CReportView
//////////////////////////////////////////////////////////////////////////////

IMPLEMENT_DYNCREATE(CReportView, CHtmlView)

	
BEGIN_MESSAGE_MAP(CReportView, CHtmlView)
END_MESSAGE_MAP()


CReportView::CReportView()
{
}

CReportView::~CReportView()
{
}

#ifdef _DEBUG
CWireCenterDoc* CReportView::GetDocument() const // Nicht-Debugversion ist inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CWireCenterDoc)));
	return (CWireCenterDoc*)m_pDocument;
}
#endif //_DEBUG


// CGeometryView-Meldungshandler

void CReportView::OnInitialUpdate()
{
	CHtmlView::OnInitialUpdate();
	// make this adress a dynamic property loaded e.g. from CWireCenterDoc
	Navigate(GetDocument()->report);
}

void CReportView::OnUpdate(CView* /*pSender*/, LPARAM /*lHint*/, CObject* /*pHint*/)
{
//	Navigate(GetDocument()->report);
	Refresh();
}
