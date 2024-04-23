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

// MainFrm.cpp : Implementierung der Klasse CMainFrame
//

#include "stdafx.h"
#include "WireCenter.h"

#include "MainFrm.h"
#include "WireCenterDoc.h"
#include "WireCenterView.h"
#include "GeometryView.h"
#include <WireLib/WireLib.h>
#include "PythonInterface.h"
#include "wcPyBindings.h"
#include "GenericParamDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// CMFCRibbonBarNoQAT class
IMPLEMENT_DYNAMIC(CMFCRibbonBarNoQAT, CMFCRibbonBar)
BEGIN_MESSAGE_MAP(CMFCRibbonBarNoQAT, CMFCRibbonBar)
END_MESSAGE_MAP()


// CMainFrame

IMPLEMENT_DYNCREATE(CMainFrame, CFrameWndEx)

BEGIN_MESSAGE_MAP(CMainFrame, CFrameWndEx)
	ON_WM_CREATE()
	ON_COMMAND(ID_ANSICHT_VISUALIZATION, &CMainFrame::OnAnsichtVisualization)
	ON_COMMAND(ID_ANSICHT_GEOMETRIE, &CMainFrame::OnAnsichtGeometrie)
	ON_WM_TIMER()
	ON_WM_DESTROY()
	// link the messages from the console toolbar buttons to the functions here
	ON_COMMAND(ID_CONSOLE_TOOLBARBUTTON_SAVE, &CMainFrame::OnDateiSpeichernkonsole)
	ON_COMMAND(ID_CONSOLE_TOOLBARBUTTON_CLEAR, &CMainFrame::OnBearbeitenKonsoleloeschen)
	ON_CONTROL_RANGE(BN_CLICKED,CwcPlugin::baseID,CwcPlugin::baseID+100,OnDynamicMenuItems)
	ON_COMMAND(ID_SCRIPT_PLUGINLADEN, &CMainFrame::OnScriptPluginladen)
	ON_COMMAND(ID_ANSICHT_POSE, &CMainFrame::OnAnsichtPose)
	ON_COMMAND(ID_ANSICHT_GEOMETRIEINTERAKTIV, &CMainFrame::OnAnsichtGeometrieinteraktiv)
	ON_COMMAND(ID_ANSICHT_KONSOLE, &CMainFrame::OnAnsichtKonsole)
	ON_COMMAND(ID_ANSICHT_SCENEGRAPH, &CMainFrame::OnAnsichtScenegraph)
	ON_UPDATE_COMMAND_UI(ID_ANSICHT_POSE, &CMainFrame::OnUpdateAnsichtPose)
	ON_UPDATE_COMMAND_UI(ID_ANSICHT_GEOMETRIEINTERAKTIV, &CMainFrame::OnUpdateAnsichtGeometrieinteraktiv)
	ON_UPDATE_COMMAND_UI(ID_ANSICHT_KONSOLE, &CMainFrame::OnUpdateAnsichtKonsole)
	ON_UPDATE_COMMAND_UI(ID_ANSICHT_SCENEGRAPH, &CMainFrame::OnUpdateAnsichtScenegraph)
	//ON_NOTIFY_EX_RANGE(TTN_NEEDTEXTW, 0, 0xFFFF, OnToolTipNotify)
    //ON_NOTIFY_EX_RANGE(TTN_NEEDTEXTA, 0, 0xFFFF, OnToolTipNotify)
	ON_COMMAND(ID_REPORT_BUTTON, &CMainFrame::OnReportButton)
	ON_COMMAND(ID_PYTHON_RUNACTION_BUTTON, &CMainFrame::OnPythonRunactionButton)
	ON_UPDATE_COMMAND_UI(ID_RESET_PYTHON_BUTTON, &CMainFrame::OnUpdateResetPythonButton)
	ON_COMMAND(ID_RESET_PYTHON_BUTTON, &CMainFrame::OnResetPythonButton)
	ON_COMMAND(ID_ANSICHT_ALGORITHMCONFIG, &CMainFrame::OnAnsichtAlgorithmconfig)
	ON_UPDATE_COMMAND_UI(ID_ANSICHT_ALGORITHMCONFIG, &CMainFrame::OnUpdateAnsichtAlgorithmconfig)
END_MESSAGE_MAP()

//! \todo This structure is deprecated; it belongs to the MFC42 style status bar
static UINT indicators[] =
{
	ID_SEPARATOR,           // Statusleistenanzeige
	ID_SEPARATOR,           // Statusleistenanzeige
	ID_INDICATOR_CAPS,
	ID_INDICATOR_NUM,
	ID_INDICATOR_SCRL,
};


// CMainFrame-Erstellung/Zerstörung

CMainFrame::CMainFrame()
{
	// TODO: Hier Code für die Memberinitialisierung einfügen
	hConsoleThread = 0;
	bRunConsoleThread = false;
	myPlugin = 0;
}

CMainFrame::~CMainFrame()
{
	if (myPlugin)
		delete myPlugin;
}


// Notification handler
BOOL CMainFrame::OnToolTipNotify(UINT id, NMHDR *pNMHDR, LRESULT *pResult)
{
   // need to handle both ANSI and UNICODE versions of the message
   TOOLTIPTEXTA* pTTTA = (TOOLTIPTEXTA*)pNMHDR;
   TOOLTIPTEXTW* pTTTW = (TOOLTIPTEXTW*)pNMHDR;
   TOOLINFOA* pTI = (TOOLINFOA*)pNMHDR;
   CString strTipText;

   CWnd* pWnd;
   CString str;
   pWnd = FromHandle(pNMHDR->hwndFrom);
   pWnd->GetWindowTextA(str);
   
   UINT nID = pNMHDR->idFrom;

   if (pNMHDR->code == TTN_NEEDTEXTA && (pTTTA->uFlags & TTF_IDISHWND) ||
      pNMHDR->code == TTN_NEEDTEXTW && (pTTTW->uFlags & TTF_IDISHWND))
   {
      // idFrom is actually the HWND of the tool
      nID = ::GetDlgCtrlID((HWND)nID);
   }

   TCHAR sz[160]; 
   LoadString(GetModuleHandle(NULL),ID_IGP_FAST_DEC+nID-1,sz,80);
   lstrcpyn(pTTTA->szText,(LPCSTR)sz,sizeof(pTTTA->szText));
   *pResult = 0;
  
   return TRUE;    // message was handled
}

/*  this procedure runs in an additional thread to process data in the console
 *  pipe. Copying the data from the log to a window is now done elsewhere
 *  to prevent hang-ups.
 */
void CMainFrame::ConsoleThreadProc()
{
	while (bRunConsoleThread)	// exit if the main window set the termination signal
	{
		if (CWireCenterApp::rPipe != 0)
		{
			bool newData = false;
			DWORD bytesRead;
			char buf[1024];
			CString str;
			fflush(stdout); // necessary, or you could wait long before seeing lines
			do {
				bytesRead = 0;
				// look first in the pipe to prevent ReadFile from blocking
				if (PeekNamedPipe(CWireCenterApp::rPipe, buf, sizeof(buf)-1, &bytesRead, NULL, NULL) && bytesRead) 
				{
					if (ReadFile(CWireCenterApp::rPipe, buf, bytesRead, &bytesRead, NULL))
					{
						buf[bytesRead] = 0;
						str += buf;
						newData = true;
					}
				}
			} while (bytesRead);
			if (newData)
			{
//				str += "\r\n";			// add a carriage return at the end of the block read from the pipe
				log.addEntry(str);		// add the latest content to the log; updating the window is done by the idle task of the application object
			}
		}
		Sleep(10);		// cycle time for updating the console window
	}
}

//! adapter function to call the console thread member function.
DWORD WINAPI timerProc(LPVOID lpParameter)
{
	// get and store the pointer to the main window; this is the link between this thread
	// and the application
	CMainFrame* pMF = (CMainFrame*)lpParameter;
	pMF->ConsoleThreadProc();
	return 0;
}

int CMainFrame::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	// set this to false, so no extra ON_COMMAND or ON_UPDATE_CMD_UI is necessary to enable the 
	// menu entry right from the beginning
	m_bAutoMenuEnable = FALSE;
	
	if (CFrameWndEx::OnCreate(lpCreateStruct) == -1)
		return -1;
	
	// add the visual manager and assign the windows 7 style
	CMFCVisualManager::SetDefaultManager(RUNTIME_CLASS(CMFCVisualManagerWindows7));

	// Create the ribbon bar
	if (!m_wndRibbonBar.Create(this))
	{
		TRACE0("Fehler beim Erstellen der Ribbonbar.\n");
		return -1;
	}
	
	// load the ribbon bar
	m_wndRibbonBar.LoadFromResource(IDR_RIBBON1);
	// Enable Tooltips
	m_wndRibbonBar.EnableToolTips(TRUE,TRUE);
	EnableToolTips(TRUE);
	      
#ifdef WIRECENTER_LIGHTVERSION
	// in the light version we disable all but the items on the first ribbon bar
	// we can change the behaviour such another ribbon with specialized commands is visible
	int  catCount = m_wndRibbonBar.GetCategoryCount();
//	printf("CategoryCount: %i\n",catCount);
	// Disable all but the last categories on the ribbon bar (this holds the ribbon bar with the wcLight options)
	for (int i=1; i<catCount-1; i++)
		m_wndRibbonBar.ShowCategory(i,0);
	CMFCRibbonCategory* pCat = m_wndRibbonBar.GetCategory(catCount-1);
	m_wndRibbonBar.SetActiveCategory(pCat,FALSE);
#else
	// in the stardard version of WireCenter we disable the light-ribbon bar
	m_wndRibbonBar.ShowCategory(m_wndRibbonBar.GetCategoryCount()-1,0);
#endif // WIRECENTER_LIGHTVERSION

#ifndef HAS_STATUS_BAR			// change this to #ifdef to remove the statusbar from the application; currently nothing is done by the status bar
	if (!m_wndStatusBar.Create(this))
	{
		TRACE0("Fehler beim Erstellen der Statusleiste.\n");
		return -1;      // Fehler beim Erstellen
	}

	CString strTitlePane1("Status:");
	CString strTitlePane2("Ready");

	//! \todo: Add apropriate ID's to the section of the status bar; otherwise we cannot acces the fields in the program
	m_wndStatusBar.AddElement(new CMFCRibbonStatusBarPane(ID_STATUSBAR_PANE1, strTitlePane1, TRUE), strTitlePane1);
	// we can add further elements if we like, e.g. an edit or button
	// m_wndStatusBar.AddElement(new CMFCRibbonEdit(4,400,"Py:"),_T("Py:"));
	// m_wndStatusBar.AddElement(new CMFCRibbonButton(5,"Ausführen"),_T("Ausführen"));
	// m_wndStatusBar.AddElement(new CMFCRibbonProgressBar(5),"Progress",1);
	m_wndStatusBar.AddExtendedElement(new CMFCRibbonStatusBarPane(ID_STATUSBAR_PANE2, strTitlePane2, TRUE), strTitlePane2);
#endif

	// Andockfensterverhalten wie in Visual Studio 2005 aktivieren
	CDockingManager::SetDockingMode(DT_SMART);
	// Automatisches Ausblenden von Andockfenstern wie in Visual Studio 2005 aktivieren
	EnableAutoHidePanes(CBRS_ALIGN_ANY);

	// my new console pane (including starting of the thread)
	if (!m_wndConsolePane.Create(_T("Console"),this,CRect(0,0,100,100),TRUE,ID_PANE_CONSOLE, 
		WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | WS_CLIPCHILDREN | WM_CTLCOLORBTN | CBRS_LEFT | CBRS_FLOAT_MULTI))
	{
		TRACE0("Fehler beim Erstellen des Console Dockable Pane.\n");
		return FALSE;
	}
	
	// create and start the thread to forward the console output to the window
	bRunConsoleThread = true;
	hConsoleThread = CreateThread(0,0,timerProc,this,0,0);

	m_wndConsolePane.EnableDocking(CBRS_ALIGN_ANY);
	DockPane(&m_wndConsolePane);
	// end of init console 

	// create the interactive geometry pane
	if (!m_wndGeomPane.Create(_T("Interactive Geometry"),this,CRect(0,0,200,200),TRUE,ID_PANE_INTERACTIVEGEOMETRY,
		WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | WS_CLIPCHILDREN | CBRS_LEFT | CBRS_RIGHT | CBRS_FLOAT_MULTI))
	{
		TRACE0("Fehler beim Erstellen des Interactive Geometry Pane.\n");
		return FALSE;
	}
	m_wndGeomPane.EnableDocking(CBRS_ALIGN_ANY);
	m_wndGeomPane.SetWindowPos(0,300,200,-1,-1,SWP_NOSIZE|SWP_SHOWWINDOW|SWP_NOACTIVATE);
	m_wndGeomPane.SetMinSize(CSize(180,200));
	DockPane(&m_wndGeomPane, AFX_IDW_DOCKBAR_LEFT);

	// create the algorithm configuration pane
	if (!m_wndAlgorithmConfig.Create(_T("Algorithm Configuration"),this,CRect(0,0,200,200),TRUE,ID_PANE_ALGORITHMCONFIG,
		WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | WS_CLIPCHILDREN | CBRS_LEFT | CBRS_RIGHT | CBRS_FLOAT_MULTI))
	{
		TRACE0("Fehler beim Erstellen des Algorithm Config Pane.\n");
		return FALSE;
	}
	m_wndAlgorithmConfig.EnableDocking(CBRS_ALIGN_ANY);
	m_wndAlgorithmConfig.SetWindowPos(0,300,200,-1,-1,SWP_NOSIZE|SWP_SHOWWINDOW|SWP_NOACTIVATE);
	m_wndAlgorithmConfig.SetMinSize(CSize(180,200));
	DockPane(&m_wndAlgorithmConfig, AFX_IDW_DOCKBAR_LEFT);

	// create the pose inspector pane
	if (!m_wndPosePane.Create(_T("Pose Inspector"),this,CRect(50,50,200,400),TRUE,ID_PANE_POSEINSPECTOR,
		WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | WS_CLIPCHILDREN | CBRS_LEFT | CBRS_RIGHT | CBRS_FLOAT_MULTI))
	{
		TRACE0("Fehler beim Erstellen des Pose Inspektor Pane.\n");
		return FALSE;
	}
	m_wndPosePane.EnableDocking(CBRS_ALIGN_ANY);
	m_wndPosePane.SetWindowPos(0,400,200,-1,-1,SWP_NOSIZE|SWP_SHOWWINDOW|SWP_NOACTIVATE);
	m_wndPosePane.SetMinSize(CSize(180,200));
	DockPane(&m_wndPosePane, AFX_IDW_DOCKBAR_LEFT);

	// create the shapelist view pane
	if (!m_wndShapeListPane.Create(_T("Analysis Objects"),this,CRect(50,50,200,400),TRUE,ID_PANE_GRAPHICOBJECTS,
		WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | WS_CLIPCHILDREN | CBRS_LEFT | CBRS_RIGHT | CBRS_FLOAT_MULTI))
	{
		TRACE0("Fehler beim Erstellen des Analyseobjekte Pane.\n");
		return FALSE;
	}
	m_wndShapeListPane.EnableDocking(CBRS_ALIGN_ANY);
	m_wndShapeListPane.SetWindowPos(0,400,200,-1,-1,SWP_NOSIZE|SWP_SHOWWINDOW|SWP_NOACTIVATE);
	m_wndShapeListPane.SetMinSize(CSize(180,200));
	DockPane(&m_wndShapeListPane, AFX_IDW_DOCKBAR_LEFT);
	
	// set in all elements of the ribbon the ToolTipText property to their own names
	// -> is this the only way to get the tool tips on screen?
	// BTW: if no tooltip shows up, it mean, that the button is not properly connected with a function
	// the "brute-force" way is necessary, because the functions GetItemIDsList , GetElementsByID
	// won't find the split button tooltips
	
	CMFCRibbonBaseElement* pElement;

	// start from 32771, because this is the first ribbon resource id
	for (int i=32771; i<MAXWORD; i++)
	{
		// get all elements with the current ID
		CArray<CMFCRibbonBaseElement* ,CMFCRibbonBaseElement*> arButtons;
		m_wndRibbonBar.GetElementsByID(i,arButtons);
		
		// set the tooltiptext in all elements
		for (int j=0; j<arButtons.GetCount(); j++)
		{
			pElement = arButtons.GetAt(j);
			if (pElement)
			{
				pElement->UpdateTooltipInfo();
				pElement->SetToolTipText(pElement->GetText());
			}
		}
	}

	// the following code is for creating a new tab-control for the pane-windows
	// unfortunatly it is necessary to create a new class to enhance the appearance :(
	
	//CMFCOutlookBar m_wndTabs;
	//m_wndTabs.Create (_T("Shortcuts"), this, CRect (0, 0, 60, 30), ID_VIEW_OUTPUTWND+123, WS_VISIBLE);
	//m_wndTabs.SetMode2003(TRUE);
	//m_wndPosePane.SetTabbedPaneRTC(m_wndTabs.GetRuntimeClass());
	//m_wndConsolePane.SetTabbedPaneRTC(m_wndTabs.GetRuntimeClass());
	//m_wndGeomPane.SetTabbedPaneRTC(m_wndTabs.GetRuntimeClass());
#ifdef WIRECENTER_LIGHTVERSION
	m_wndConsolePane.ShowPane(FALSE,FALSE,TRUE);
#endif
	return 0;
}

BOOL CMainFrame::PreCreateWindow(CREATESTRUCT& cs)
{
	if( !CFrameWndEx::PreCreateWindow(cs) )
		return FALSE;
	// TODO: Ändern Sie hier die Fensterklasse oder die Darstellung, indem Sie
	// CREATESTRUCT cs modifizieren.

	return TRUE;
}

//! This function is the core element for have more than one view (where view relates
//! to the Mircosoft's document-view-architecture).
//! Implementation adapted from "Inside Visual C++ 6", p.467
void CMainFrame::SwitchToView(eView nView)
{
	CView* pOldActiveView = GetActiveView();
	if (pOldActiveView->GetRuntimeClass() == RUNTIME_CLASS(CWireCenterView))
		pOldActiveView->SetDlgCtrlID(VISUALIZATION);
	CView* pNewActiveView = (CView*)GetDlgItem(nView);
	if (pNewActiveView == pOldActiveView)
		return;
	if (pNewActiveView == NULL) {

		switch (nView) {
			case VISUALIZATION:		pNewActiveView = (CView*) new CWireCenterView;          break;
			case GEOMETRY:			pNewActiveView = (CView*) new CGeometryView;			break;
			case REPORT:			pNewActiveView = (CView*) new CReportView;				break;
			default:
				MessageBox("Internal Error: unknown view selected in CMainFrame::SwitchToView. This should never happen","Error",MB_OK);
				return;
				// not yet implemented
				break;
		}
		CCreateContext context;
		context.m_pCurrentDoc = pOldActiveView->GetDocument();
		pNewActiveView->Create(NULL, NULL, WS_BORDER | WS_CHILD,	// added WS_CHILD; needed for CFormView based views; I have no idea if this affects the other views...
			CFrameWndEx::rectDefault, this, nView, &context);
		pNewActiveView->OnInitialUpdate();
	}
	SetActiveView(pNewActiveView);
	pNewActiveView->ShowWindow(SW_SHOW);
	pOldActiveView->ShowWindow(SW_HIDE);
 
	if (pOldActiveView->GetRuntimeClass() == RUNTIME_CLASS(CWireCenterView))	
		pOldActiveView->SetDlgCtrlID(VISUALIZATION);
	else if(pOldActiveView->GetRuntimeClass() == RUNTIME_CLASS(CGeometryView))
		pOldActiveView->SetDlgCtrlID(GEOMETRY);
	else if(pOldActiveView->GetRuntimeClass() == RUNTIME_CLASS(CReportView))
		pOldActiveView->SetDlgCtrlID(REPORT);
	else
		MessageBox("Internal Error: unknown view selected in CMainFrame::SwitchToView. This should never happen","Error",MB_OK);// pOldActiveView->SetDlgCtrlID(DEFAULT);

	pNewActiveView->SetDlgCtrlID(AFX_IDW_PANE_FIRST);
	RecalcLayout();
}

// CMainFrame-Diagnose

#ifdef _DEBUG
void CMainFrame::AssertValid() const
{
	CFrameWndEx::AssertValid();
}

void CMainFrame::Dump(CDumpContext& dc) const
{
	CFrameWndEx::Dump(dc);
}

#endif //_DEBUG


// CMainFrame-Meldungshandler

void CMainFrame::OnAnsichtVisualization()
{
	SwitchToView(VISUALIZATION);
}

void CMainFrame::OnAnsichtGeometrie()
{
	SwitchToView(GEOMETRY);
}

void CMainFrame::OnReportButton()
{
	SwitchToView(REPORT);
	// this is a bit like a hack; to update the report view we call a update all views
	GetActiveView()->GetDocument()->UpdateAllViews(0);
}

void CMainFrame::OnTimer(UINT_PTR nIDEvent)
{
	CFrameWndEx::OnTimer(nIDEvent);
}

void CMainFrame::OnDestroy()
{
	CFrameWndEx::OnDestroy();

	// stop the thread of the console before the window is terminated
	bRunConsoleThread = false;
	WaitForSingleObject(hConsoleThread,100);
}

void CMainFrame::OnScriptPluginladen()
{
	CFileDialog FileDlg(TRUE,	// load File Dialog
						".dll",
						"myExtDll.dll",
						OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"WireCenter Plugins (*.dll)|*.dll|");
	if (FileDlg.DoModal() == IDOK)
	{
		if (!myPlugin)
		{
			myPlugin = new CwcPlugin(FileDlg.GetPathName().GetBuffer());
			myPlugin->App.wcVersion = PCRL::versionString;
		}
		else
			// the plugin is already loaded; it's not a good idea to repeat loading
			return;

		printf("Connection with Plugin established\n");
		if (!myPlugin->load())
			return;

		if (!myPlugin->extendMenu())
			return;
	}
	else  // the user did not click ok
		return;
}

void CMainFrame::OnDynamicMenuItems( UINT nID )
{
	if (!myPlugin)
		return;
	if (myPlugin->Invoke)
		myPlugin->Invoke(nID-myPlugin->baseID,0,0);

}

void CMainFrame::OnDateiSpeichernkonsole()
{
	CFileDialog FileDlg(FALSE,	// save File Dialog
						".txt",
						"log.txt",
						OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"Text Files (*.txt)|*.txt|");
	if (FileDlg.DoModal() == IDOK)
	{
		log.save(FileDlg.GetPathName());
	}
}

void CMainFrame::OnBearbeitenKonsoleloeschen()
{
	log.clear();
	log.setLogWindow(&m_wndConsolePane.m_wndEdit);
}

void CMainFrame::OnUpdateAnsichtKonsole(CCmdUI *pCmdUI)
{ pCmdUI->SetCheck(m_wndConsolePane.IsVisible()?1:0); }

void CMainFrame::OnUpdateAnsichtPose(CCmdUI *pCmdUI)
{ pCmdUI->SetCheck(m_wndPosePane.IsVisible()?1:0); }

void CMainFrame::OnUpdateAnsichtGeometrieinteraktiv(CCmdUI *pCmdUI)
{ pCmdUI->SetCheck(m_wndGeomPane.IsVisible()?1:0); }

void CMainFrame::OnUpdateAnsichtScenegraph(CCmdUI *pCmdUI)
{ pCmdUI->SetCheck(m_wndShapeListPane.IsVisible()?1:0); }

void CMainFrame::OnUpdateAnsichtAlgorithmconfig(CCmdUI *pCmdUI)
{ pCmdUI->SetCheck(m_wndAlgorithmConfig.IsVisible()?1:0); }

void CMainFrame::OnAnsichtKonsole()
{
	// test if the window is currently visible
	if (m_wndConsolePane.IsVisible())
		m_wndConsolePane.ShowPane(FALSE,FALSE,TRUE);
	else
		m_wndConsolePane.ShowPane(TRUE,FALSE,TRUE);
}

void CMainFrame::OnAnsichtPose()
{
	// test if the window is currently visible
	if (m_wndPosePane.IsVisible())
		m_wndPosePane.ShowPane(FALSE,FALSE,TRUE);
	else
		m_wndPosePane.ShowPane(TRUE,FALSE,TRUE);
}

void CMainFrame::OnAnsichtGeometrieinteraktiv()
{
	// test if the window is currently visible
	if (m_wndGeomPane.IsVisible())
		m_wndGeomPane.ShowPane(FALSE,FALSE,TRUE);
	else
		m_wndGeomPane.ShowPane(TRUE,FALSE,TRUE);
}

void CMainFrame::OnAnsichtScenegraph()
{
	// test if the window is currently visible
	if (m_wndShapeListPane.IsVisible())
		m_wndShapeListPane.ShowPane(FALSE,FALSE,TRUE);
	else
		m_wndShapeListPane.ShowPane(TRUE,FALSE,TRUE);
}

void CMainFrame::OnAnsichtAlgorithmconfig()
{
	// test if the window is currently visible
	if (m_wndAlgorithmConfig.IsVisible())
		m_wndAlgorithmConfig.ShowPane(FALSE,FALSE,TRUE);
	else
		m_wndAlgorithmConfig.ShowPane(TRUE,FALSE,TRUE);
}

////////////////////////////////////////////////////////////////////////////////
// CLog
////////////////////////////////////////////////////////////////////////////////

CLog::CLog()
{
	timeOut = 500;
	newdata = false;
	mxLog = CreateMutex(NULL, FALSE, NULL);
}

CLog::~CLog()
{
	CloseHandle(mxLog);
}

//! add the string to the log
bool CLog::addEntry(const CString& str)
{
	DWORD dwResult = WaitForSingleObject(mxLog, timeOut);
	if (dwResult == WAIT_OBJECT_0)
	{
		log += str;
		newdata = true;
		ReleaseMutex(mxLog);
		return true;
	}
	else
		return false;
}

//! empty the existing log
bool CLog::clear()
{
	newdata = true;
	DWORD dwResult = WaitForSingleObject(mxLog, timeOut);
	if (dwResult == WAIT_OBJECT_0)
	{
		log = "";
		ReleaseMutex(mxLog);
		return true;
	}
	else
		return false;
}

//! save the log into a file with given name
bool CLog::save(const CString& filename)
{
	DWORD dwResult = WaitForSingleObject(mxLog, timeOut);
	if (dwResult == WAIT_OBJECT_0)
	{
		std::string s((LPCTSTR)filename);
		ofstream file(s.c_str());
		file << log;
		ReleaseMutex(mxLog);
		return true;
	}
	else
		return false;	
}

//! get a copy of the buffer; we cannot return a reference since the log should be thread-save
bool CLog::getLog(CString& str)
{
	DWORD dwResult = WaitForSingleObject(mxLog,timeOut);
	if (dwResult == WAIT_OBJECT_0)
	{
		str = log;
		ReleaseMutex(mxLog);
		return true;
	}
	else
		return false;
}

//! copy the log into the given window
bool CLog::setLogWindow(CEdit* pEdit, bool forceUpdate)
{
	if (!newdata && !forceUpdate)
		return true;
	if (!pEdit)
		return false;
	DWORD dwResult = WaitForSingleObject(mxLog,timeOut);
	if (dwResult == WAIT_OBJECT_0)
	{
		// this call may block if the main thread (with the mfc message map) is busy
		pEdit->SetWindowTextA(log);
		pEdit->LineScroll(pEdit->GetLineCount());	// scroll to the end of the window
		newdata = false;
		ReleaseMutex(mxLog);
		return true;
	}
	else
		return false;	
}

IMPLEMENT_DYNCREATE(CProgressBar, CMFCRibbonProgressBar)

CProgressBar::CProgressBar(int nID, int iSize, int iMaxValue, int nTextPane, CString sPaneText)
{
	CWnd *pMainWnd = AfxGetMainWnd();
	if (!pMainWnd) 
	{
		cout << "Error: Did not find the main window. ProgressBar was not created. " << endl;
		return;
	}

	// If main window is a frame window, use normal methods...
	if (pMainWnd->IsKindOf(RUNTIME_CLASS(CFrameWndEx)))
	{
		CWnd* pMessageBar = ((CFrameWndEx*)pMainWnd)->GetMessageBar();
		m_pStatusBar = DYNAMIC_DOWNCAST(CMFCRibbonStatusBar, pMessageBar);
	}
	// otherwise traverse children to try and find the status bar...
	else 
		m_pStatusBar = DYNAMIC_DOWNCAST(CMFCRibbonStatusBar, pMainWnd->GetDescendantWindow(AFX_IDW_STATUS_BAR));	
	
	if (!m_pStatusBar) 
	{
		cout << "Error: Did not find status bar. ProgressBar was not be created. " << endl;
		return;
	}

	// save ID for deleting the right element, create the progressbar object and set the range
	m_nID = nID;
	m_pProgressBar = new CMFCRibbonProgressBar(m_nID, iSize);
	m_pStatusBar->AddElement(m_pProgressBar, "ProgressBar", TRUE);
	m_pProgressBar->SetRange(0, iMaxValue);

	// if a text is here, use it on the Pane with the nTextPane-ID!
	if (sPaneText != "")
	{
		// find the Pane with the correct ID
		m_pStatusText = (CMFCRibbonStatusBarPane*)m_pStatusBar->FindElement(nTextPane);
		
		// if no pane with the desired ID was found, throw error, but go on...
		if (!m_pStatusText) 
		{
			cout << "Error: Could not create text field. Status bar pane not found." << endl;
		}
		else
		{
			// store the old text for recovering and ...
			m_strPrevText = m_pStatusText->GetText();
			// ... set the new text
			m_pStatusText->SetText(sPaneText);
		}
	}
	
	// recalc layout and redraw window to see the effect
	m_pStatusBar->RecalcLayout(); 
	m_pStatusBar->RedrawWindow();
}

CProgressBar::~CProgressBar()
{
	// delete the progressbar
	m_pStatusBar->RemoveElement(m_nID);
	// reset the statusbar text and repaint it
	m_pStatusText->SetText(m_strPrevText);
	// recalc layout and update window
	m_pStatusBar->RecalcLayout();
	m_pStatusBar->RedrawWindow();
}

void CProgressBar::SetPos(int nPos)	   
{
	m_pProgressBar->SetPos(nPos);
}


int CProgressBar::StepIt(int nStep) 			
{ 
	m_pProgressBar->SetPos(m_pProgressBar->GetPos()+nStep);
	return m_pProgressBar->GetPos();
}

void CMainFrame::OnPythonRunactionButton()
{
	// get a list of all actions in the script
	list<string> actions;
	CPythonInterface::getInstance().getActions(actions);
	// if no actions are available we do nothing
	if (actions.size() == 0)
	{
		MessageBox("Python Error: No actions are available in the system script",
			"Error", MB_OK | MB_ICONSTOP);
		return;
	}
	// unfortunately, the data types between GUI und library are not compatible; we have to
	// convert from list<string> to CStringArray
	CStringArray SAactions,shortname;
	for (list<string>::iterator itor=actions.begin(); itor!=actions.end(); itor++)
	{
		SAactions.Add((*itor).c_str());
		shortname.Add((*itor).substr(6,string::npos).c_str());
	}
	// preselect the first action in the list
	CString selAction = ""; //SAactions[0];
	int sel = 0;
	// create an enhanced dialog box
	CEnhancedDialog DLG("Choose Action");
	DLG.addItem("Actions in the python script",sel,shortname);
	// show the dialogbox
	if (DLG.DoModal() != IDOK) return;	

	// call the respective python function from the system script
	CPythonInterface::getInstance().runSystemScript(SAactions[sel].GetBuffer());
	
	// a bit heavy but hopefully effective update of all views including html
	CWireCenterView::This->GetDocument()->UpdateAllViews(NULL);
}

void CMainFrame::OnResetPythonButton()
{
	CPythonInterface::getInstance().reset();
	wcInitPythonBinding(); // refill the module table after reset
}

void CMainFrame::OnUpdateResetPythonButton(CCmdUI *pCmdUI)
{ /*pCmdUI->Enable(true);*/ }
