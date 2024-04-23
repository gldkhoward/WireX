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

// WireCenter.cpp : Definiert das Klassenverhalten für die Anwendung.
//

#include "stdafx.h"
#include "WireCenter.h"
#include "MainFrm.h"
#include <afxsock.h>

#include "WireCenterDoc.h"
#include "WireCenterView.h"
#include "PythonInterface.h"
#include "wcPyBindings.h"
#include <WireLib/WireLib.h>

//activate new fancy windows 7 dialog boxes style
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='x86' publicKeyToken='6595b64144ccf1df' language='neutral' \"")

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


HANDLE CWireCenterApp::rPipe=0;

// CWireCenterApp

BEGIN_MESSAGE_MAP(CWireCenterApp, CWinApp)
	ON_COMMAND(ID_APP_ABOUT, &CWireCenterApp::OnAppAbout)
	// Dateibasierte Standarddokumentbefehle
	ON_COMMAND(ID_FILE_NEW, &CWinApp::OnFileNew)
	ON_COMMAND(ID_FILE_OPEN, &CWinApp::OnFileOpen)
	ON_COMMAND(ID_HILFE_PYTHONSCRIPTING, &CWireCenterApp::OnHilfePythonscripting)
	ON_COMMAND(ID_CLIP_BUTTON, &CWireCenterApp::OnClipButton)
END_MESSAGE_MAP()


// CWireCenterApp-Erstellung

CWireCenterApp::CWireCenterApp()
{
	// TODO: Hier Code zur Konstruktion einfügen
	// Alle wichtigen Initialisierungen in InitInstance positionieren
	rPipe = 0;
} 

CWireCenterApp::~CWireCenterApp()
{
	if (rPipe)
		CloseHandle(rPipe);
}

// Das einzige CWireCenterApp-Objekt

CWireCenterApp theApp;


// CWireCenterApp-Initialisierung

BOOL CWireCenterApp::InitInstance()
{
	//prevent the creation of multiple instances due to problems in the python-interpreter if running with multiple instances
	CreateMutex(NULL,TRUE,"WIRECENTERAPPLICATION");
	if (GetLastError()==ERROR_ALREADY_EXISTS) 
	{
			AfxMessageBox("WireCenter is already running!\n\nNo additional instance can be started.",MB_OK | MB_ICONSTOP);
			PostQuitMessage(0);
			return true;
	}

	// InitCommonControlsEx() ist für Windows XP erforderlich, wenn ein Anwendungsmanifest
	// die Verwendung von ComCtl32.dll Version 6 oder höher zum Aktivieren
	// von visuellen Stilen angibt. Ansonsten treten beim Erstellen von Fenstern Fehler auf.
	INITCOMMONCONTROLSEX InitCtrls;
	InitCtrls.dwSize = sizeof(InitCtrls);
	// Legen Sie dies fest, um alle allgemeinen Steuerelementklassen einzubeziehen,
	// die Sie in Ihrer Anwendung verwenden möchten.
	InitCtrls.dwICC = ICC_WIN95_CLASSES;
	InitCommonControlsEx(&InitCtrls);

	CWinApp::InitInstance();

	//Initialize Windows Sockets (link ws2_32.lib)
    if (!AfxSocketInit())
	{
	   AfxMessageBox(_T("Failed to Initialize Sockets"), MB_OK | MB_ICONSTOP);
	   return FALSE;
	}

	// OLE-Bibliotheken initialisieren
	if (!AfxOleInit())
	{
		AfxMessageBox(IDP_OLE_INIT_FAILED);
		return FALSE;
	}
	AfxEnableControlContainer();

	// BEGIN: new initializers for VS2010; no idea what they do in detail
	EnableTaskbarInteraction(FALSE);
	InitContextMenuManager();
	InitShellManager();

	InitKeyboardManager();

	InitTooltipManager();
	CMFCToolTipInfo ttParams;
	ttParams.m_bVislManagerTheme = TRUE;
	theApp.GetTooltipManager()->SetTooltipParams(AFX_TOOLTIP_TYPE_ALL,
	RUNTIME_CLASS(CMFCToolTipCtrl), &ttParams);
	// END: new initializers

	// Standardinitialisierung
	// Wenn Sie diese Features nicht verwenden und die Größe
	// der ausführbaren Datei verringern möchten, entfernen Sie
	// die nicht erforderlichen Initialisierungsroutinen.
	// Ändern Sie den Registrierungsschlüssel, unter dem Ihre Einstellungen gespeichert sind.
	// TODO: Ändern Sie diese Zeichenfolge entsprechend,
	// z.B. zum Namen Ihrer Firma oder Organisation.
	SetRegistryKey(_T("WireCenter"));
	SetRegistryBase(_T("Settings"));
	LoadStdProfileSettings(4);  // Standard INI-Dateioptionen laden (einschließlich MRU)
	// Dokumentvorlagen der Anwendung registrieren. Dokumentvorlagen
	//  dienen als Verbindung zwischen Dokumenten, Rahmenfenstern und Ansichten.
	CSingleDocTemplate* pDocTemplate;
	pDocTemplate = new CSingleDocTemplate(
		IDR_MAINFRAME,
		RUNTIME_CLASS(CWireCenterDoc),
		RUNTIME_CLASS(CMainFrame),       // Haupt-SDI-Rahmenfenster
		RUNTIME_CLASS(CWireCenterView));
	if (!pDocTemplate)
		return FALSE;
	AddDocTemplate(pDocTemplate);

	//! \todo Add a command line option or registry option to choose between classic console
	//! and redirected console
	if (false) // true: use classic console; false: redirect the console output into the GUI
	{
		//! open a console window
		if (!AllocConsole())
				MessageBox(NULL,"Console Window could not be initialized",
					"WireCenter",MB_ICONERROR|MB_OK);
			else
			{
				HANDLE hConsole=GetStdHandle(STD_OUTPUT_HANDLE);
				// Use 640 kB for Console Buffer
				COORD coord = { 80, 1000}; 
				SetConsoleScreenBufferSize(hConsole,coord);
				SetConsoleTitle("WireCenter - Console");
				FILE *file;
				freopen_s(&file,"conin$", "r", stdin); 
				freopen_s(&file,"conout$", "w", stdout); 
				freopen_s(&file,"conout$", "w", stderr);
			}
	}
	else
	{
		//! \todo An good implementation requires an own thread to read the pipe at the other side. Otherwise
		//! the program can block, because the writting code waits for the pipe to be emptied which will not happen 
		//! if the reading code is in the same thread!

		// Create the pipe, we only need inbound
		rPipe = CreateNamedPipe("\\\\.\\pipe\\wirecenter" , PIPE_ACCESS_INBOUND,0,1,16384,16384,500,NULL);

		// We can already connect the reading end of the pipe and assign it to stdout
		FILE *file;
		freopen_s(&file,"\\\\.\\pipe\\wirecenter", "w", stdout);

		// This call is necessary and will FAIL. This is NORMAL. GetLastError() should
		// return ERROR_PIPE_CONNECTED
		ConnectNamedPipe(rPipe, NULL);

		// The reason for the above order (freopen, then ConnectNamedPipe) is simple
		// This way, ConnectNamedPipe won't block waiting for a connection
	}

	// initialize the python scripting
	wcInitPythonBinding();
	CPythonInterface::getInstance().initSystemScript();

	// Befehlszeile auf Standardumgebungsbefehle überprüfen, DDE, Datei öffnen
	CCommandLineInfo cmdInfo;
	ParseCommandLine(cmdInfo);

	// Verteilung der in der Befehlszeile angegebenen Befehle. Gibt FALSE zurück, wenn
	// die Anwendung mit /RegServer, /Register, /Unregserver oder /Unregister gestartet wurde.
	if (!ProcessShellCommand(cmdInfo))
		return FALSE;

	// Das einzige Fenster ist initialisiert und kann jetzt angezeigt und aktualisiert werden.
	m_pMainWnd->ShowWindow(SW_SHOW);
	m_pMainWnd->UpdateWindow();
	// Rufen Sie DragAcceptFiles nur auf, wenn eine Suffix vorhanden ist.
	//  In einer SDI-Anwendung ist dies nach ProcessShellCommand erforderlich
	return TRUE;
}

int CWireCenterApp::ExitInstance()
{
	// TODO: Add your specialized code here and/or call the base class
	AfxOleTerm(FALSE);

	return CWinAppEx::ExitInstance();
}

// CAboutDlg-Dialogfeld für Anwendungsbefehl "Info"

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialogfelddaten
	enum { IDD = IDD_ABOUTBOX };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV-Unterstützung

// Implementierung
protected:
	DECLARE_MESSAGE_MAP()
public:
	virtual BOOL OnInitDialog();
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
}

/*/ Try to load the text file with the given resource ID from the resource part
 *  of the application. Such text file were embedded into the executable file by
 *  the resource compiler. The type of the file must be TEXTFILE (which is a custom
 *  type used inside WireCenter). 
 *  \param [in] resourceID A unique identifier of the textfile to be loaded. The ID is
 *    usually defined in resource.h.
 *  \return if successful a const pointer to the text file
 *     otherwise 0. Check the return value to be sure to
 *     receive a valid text file!
 */
const char* loadTextResource(const unsigned int resourceID)
{
	// receive the handle of the COPYRIGHT file stored in the resources of this project
	HMODULE handle = AfxGetInstanceHandle();
	if (!handle)
		return 0;
	HRSRC rc = ::FindResource(handle, 
		MAKEINTRESOURCE(resourceID),		// load the resource given by resourceID
        MAKEINTRESOURCE(TEXTFILE));			// the resource must be of type TEXTFILE
	if (!rc)
		return 0;
    HGLOBAL rcData = ::LoadResource(handle, rc);
    DWORD size = ::SizeofResource(handle, rc);
	if (!rcData || size==0)
		return 0;
	return static_cast<const char*>(::LockResource(rcData));
}


BOOL CAboutDlg::OnInitDialog()
{
	CDialog::OnInitDialog();
	
	CStatic* pStatic = (CStatic*)GetDlgItem(IDC_WIRECENTER_VERSION_STATIC);
	CString str;
#ifdef WIRECENTER_LIGHTVERSION
	str.Format("WireCenter Light Version: %s.%s %s",PCRL::majorVersion,PCRL::minorVersion,PCRL::versionName);
#else													  
	str.Format("WireCenter Version: %s.%s %s",PCRL::majorVersion,PCRL::minorVersion,PCRL::versionName);
#endif
	pStatic->SetWindowTextA(str);

	pStatic = (CStatic*)GetDlgItem(IDC_BUILT_STATIC);
	str.Format("WireCenter Built: %s\nThis version was linked with\n%s\nLibrary Version:%s",__DATE__,PCRL::name,PCRL::versionString);
	pStatic->SetWindowTextA(str);

	// receive the handle of the COPYRIGHT file stored in the resources of this project
	const char* data = loadTextResource(IDR_COPYRIGHT_TEXTFILE);

	// the content of the file is not loaded into the data; we can display the text in the edit
	CEdit* pEdit = (CEdit*)GetDlgItem(IDC_COPYRIGHTLICENSE_EDIT);
	pEdit->SetWindowTextA(data);

	return TRUE;  // return TRUE unless you set the focus to a control
	// AUSNAHME: OCX-Eigenschaftenseite muss FALSE zurückgeben.
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
END_MESSAGE_MAP()

// Anwendungsbefehl zum Ausführen des Dialogfelds
void CWireCenterApp::OnAppAbout()
{
	CAboutDlg aboutDlg;
	aboutDlg.DoModal();
}

// CWireCenterApp-Meldungshandler

void CWireCenterApp::OnHilfePythonscripting()
{
	// execute the python script for help
	CPythonInterface::getInstance().runSystemScript("OnHelp");
}

BOOL CWireCenterApp::OnIdle(LONG lCount)
{
	// TODO: Add your specialized code here and/or call the base class
	CMainFrame* pMF = (CMainFrame*)m_pMainWnd;
	// get a pointer to the control to be changed
	CEdit* pEdit = &pMF->m_wndConsolePane.m_wndEdit;
	if (pEdit && lCount>0)
	{
		pMF->log.setLogWindow(pEdit);
	}
	return CWinAppEx::OnIdle(lCount);
}

void CWireCenterApp::OnClipButton()
{
	// TODO: Add your command handler code here
}
