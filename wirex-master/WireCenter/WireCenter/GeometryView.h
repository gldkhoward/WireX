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

/*! \file GeometryView.h
 *
 *	\author   Andreas Pott
 *
 *  \brief One of the main views of the application. The view class
 *  is based on a form view, i.e. it looks like a big dialog box. 
 *  This view is dedicated to edit the robot data is a mostly textual 
 *  form.
 */

#pragma once
#include "afxcmn.h"
#include <WireLib/RobotData.h>
#include "WireCenterDoc.h"
#include <afxhtml.h>

// CGeometryView-Formularansicht

class CGeometryView : public CFormView
{
	DECLARE_DYNCREATE(CGeometryView)

public:
	CGeometryView();           // Dynamische Erstellung verwendet geschützten Konstruktor
	virtual ~CGeometryView();
	CWireCenterDoc* GetDocument() const;

protected:
	void CreateGeometryTable();
	void UpdateGeometryData();

public:
	enum { IDD = IDD_GEOMETRY_VIEW };
#ifdef _DEBUG
	virtual void AssertValid() const;
#ifndef _WIN32_WCE
	virtual void Dump(CDumpContext& dc) const;
#endif
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV-Unterstützung

	DECLARE_MESSAGE_MAP()
public:
	CListCtrl m_List;
public:
	virtual void OnInitialUpdate();
public:
	afx_msg void OnNMClickGeometryviewList(NMHDR *pNMHDR, LRESULT *pResult);
protected:
	virtual void OnUpdate(CView* /*pSender*/, LPARAM /*lHint*/, CObject* /*pHint*/);
public:
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	double m_dx;
	double m_dy;
	double m_dz;
	double m_da;
	double m_db;
	double m_dc;
	double m_ds;
	double m_l;
	double m_b;
	double m_h;
	afx_msg void OnBnClickedTranslateButton();
	afx_msg void OnBnClickedRotateButton();
	afx_msg void OnBnClickedScaleButton();
	// select the edit mode for the geometry (0=base, 1=platform)
	int m_GeometryMode;
	afx_msg void OnBnClickedCreateCubeButton();
};

#ifndef _DEBUG  // Debugversion in GeometryView.cpp
inline CWireCenterDoc* CGeometryView::GetDocument() const
   { return reinterpret_cast<CWireCenterDoc*>(m_pDocument); }
#endif


//! experimental HTML report view
class CReportView : public CHtmlView
{
	DECLARE_DYNCREATE(CReportView)

public:
	CReportView();           // Dynamische Erstellung verwendet geschützten Konstruktor
	virtual ~CReportView();
	CWireCenterDoc* GetDocument() const;

public:
	enum { IDD = IDD_REPORT_VIEW };

	DECLARE_MESSAGE_MAP()

protected:
	virtual void OnUpdate(CView* /*pSender*/, LPARAM /*lHint*/, CObject* /*pHint*/);

public:
	virtual void OnInitialUpdate();
};

#ifndef _DEBUG  // Debugversion in GeometryView.cpp
inline CWireCenterDoc* CReportView::GetDocument() const
   { return reinterpret_cast<CWireCenterDoc*>(m_pDocument); }
#endif

