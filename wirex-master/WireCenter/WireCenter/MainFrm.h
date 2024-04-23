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

/*! \file MainFrm.h
 *
 *	\author   Andreas Pott
 *
 *  \brief The main frame (window) of the application. The menu bar, the tool bars
 *  and the main views are child objects of this window. The python console is 
 *  hooked in here to make it available in most views of the application.
 */

#pragma once

#include "ConsoleDlg.h"
#include "InteractiveGeometryDlg.h"
#include "AlgorithmConfigPane.h"
#include "PoseDlg.h"
#include "ShapeListViewDlg.h"
#include "wcPlugin.h"

/*! the class CLog implements a log to be used in multiple threads.
 *  the getter and setter functions are designed to garant thread-save
 *  access to the buffer used to store the log.
 */
class CLog
{
private:
	int timeOut;	//!< timeout for sync
	CString log;	//!< containt the full log of the console window
	HANDLE mxLog;	//!< a mutex object to control the access to the log object
	bool newdata;	//!< flag that indicates if new data are pending since the last call to setLogWindow
public:
	CLog();
	~CLog();
	//! add the string to the log
	bool addEntry(const CString& str);
	//! empty the existing log
	bool clear();
	//! save the log into a file with given name
	bool save(const CString& filename);
	//! copy the log into the given window
	bool setLogWindow(CEdit* pEdit, bool forceUpdate=false);
	//! get a copy of the buffer; we cannot return a reference since the log should be thread-save
	bool getLog(CString& log);
};

/////////////////////////////////////////////////////////////////////////////
// CProgressBar -  status bar progress control
//
// Copyright (c) Chris Maunder, 1997
// Please feel free to use and distribute.

class CProgressBar: public CMFCRibbonProgressBar
// Creates a ProgressBar in the status bar
{
public:
	CProgressBar(int nID=1000, int iSize=300, int iMaxValue=100, int nTextPane=0, CString sPaneText="Bereit");
	~CProgressBar();
	DECLARE_DYNCREATE(CProgressBar)

// operations
public:
	void  SetPos(int nPos);
	int  StepIt(int nStep);

// implementation
protected:
	int		m_nStepSize;	// step size
	int		m_nPane;		// ID of status bar pane progress bar is to appear in
	int		m_nID;			// ID of the Progressbar
    CString m_strPrevText;  // Previous text in status bar
	CMFCRibbonStatusBar* m_pStatusBar; // Set in the ctor to explicitly state which status bar to use.
	CMFCRibbonProgressBar* m_pProgressBar; // Hold the current ProgressBar
	CMFCRibbonStatusBarPane* m_pStatusText;
};


// CMFCRibbonBarNoQAT class
class CMFCRibbonBarNoQAT : public CMFCRibbonBar
{
	DECLARE_DYNAMIC(CMFCRibbonBarNoQAT)
public:
	CMFCRibbonBarNoQAT() {}
	virtual ~CMFCRibbonBarNoQAT() {}
	virtual BOOL OnShowRibbonContextMenu(CWnd* pWnd, int x, int y, CMFCRibbonBaseElement* pHit)
	{ return TRUE; }
	virtual BOOL OnShowRibbonQATMenu(CWnd* pWnd, int x, int y, CMFCRibbonBaseElement* pHit) 
	{ return FALSE; }
	virtual void OnFillBackground(CDC* pDC, CRect rectClient)
	{
       //define rect = (0)
       CRect rect;
       //set the toolbarbutton to 0x0 size - because SetVisible(FALSE) isn't working...
       m_QAToolbar.GetButton(0)->SetRect(rect);
       //pass on the painting function
       CMFCRibbonBar::OnFillBackground(pDC, rectClient);
	}
protected:
	DECLARE_MESSAGE_MAP()
};


class CMainFrame : public CFrameWndEx
{
	
protected: // Nur aus Serialisierung erstellen
	CMainFrame();
	DECLARE_DYNCREATE(CMainFrame)

private:
	enum eView {VISUALIZATION=0, GEOMETRY=1, REPORT=2};
	//! change the state of the application such that the specified view is the active view
	void SwitchToView(eView nView);

// Attribute
public:
	bool bRunConsoleThread;			//!< flag indicating that the thread to forward the console output is running (and shall be running)
	HANDLE hConsoleThread;			//!< handle to the console thread 

// Vorgänge
public:

// Überschreibungen
public:
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);

// Implementierung
public:
	virtual ~CMainFrame();

	void ConsoleThreadProc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

public:  // Eingebettete Member der Steuerleiste
	//! a ribbon style status bar
	CMFCRibbonStatusBar m_wndStatusBar;
public:
	//! Ribbon Bar for the application
#ifdef WIRECENTER_LIGHTVERSION
	CMFCRibbonBarNoQAT m_wndRibbonBar;
#else
	CMFCRibbonBar m_wndRibbonBar;
#endif
	//! a thread-save memory for the redirected console output
	CLog log;	
	//! experimental pane; refactored from the interactiveGeometryDlg
	CInteractiveGeometryPane m_wndGeomPane;		
	//! experimental pane showing the algorithm configuration
	CAlgorithmConfigPane m_wndAlgorithmConfig;		
	//! the console pane, a dockable window showing the output to stdout and printf
	CConsolePane m_wndConsolePane;
	//! the pose pane, a dockable window showing the current position and orientation of the platform and associated properties
	CPosePane m_wndPosePane;
	//! the shapelist view pane, a dockable window showing the scene graph and its properties
	CShapeListViewPane m_wndShapeListPane;
	//! pointer to an instance of the DLL interface 
	CwcPlugin *myPlugin;

// Generierte Funktionen für die Meldungstabellen
protected:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnAnsichtVisualization();
public:
	afx_msg void OnAnsichtGeometrie();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnAnsichtKonsole();
	afx_msg void OnUpdateAnsichtKonsole(CCmdUI *pCmdUI);
	afx_msg void OnDestroy();
	afx_msg void OnDateiSpeichernkonsole();
	afx_msg void OnBearbeitenKonsoleloeschen();
	afx_msg void OnDynamicMenuItems( UINT nID );
	afx_msg void OnScriptPluginladen();
	afx_msg void OnUpdateAnsichtPose(CCmdUI *pCmdUI);
	afx_msg void OnAnsichtPose();
	afx_msg void OnAnsichtGeometrieinteraktiv();
	afx_msg void OnUpdateAnsichtGeometrieinteraktiv(CCmdUI *pCmdUI);
	afx_msg void OnAnsichtScenegraph();
	afx_msg void OnUpdateAnsichtScenegraph(CCmdUI *pCmdUI);
	afx_msg BOOL OnToolTipNotify(UINT id, NMHDR *pNMHDR,LRESULT *pResult);
	afx_msg void OnReportButton();
	afx_msg void OnPythonRunactionButton();
	afx_msg void OnResetPythonButton();
	afx_msg void OnUpdateResetPythonButton(CCmdUI *pCmdUI);
	afx_msg void OnAnsichtAlgorithmconfig();
	afx_msg void OnUpdateAnsichtAlgorithmconfig(CCmdUI *pCmdUI);
};


