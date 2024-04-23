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

// ConsoleDlg.cpp: Implementierungsdatei
//

#include "stdafx.h"
#include "WireCenter.h"
#include "ConsoleDlg.h"
#include "PythonInterface.h"
#include <fstream>
#include <Winuser.h>
#include "MainFrm.h"

// IMPLEMENTATION OF CConsolePane
/////////////////////////////////

BEGIN_MESSAGE_MAP(CConsolePane, CDockablePane)
	ON_WM_CREATE()
	ON_WM_SIZE()
	ON_WM_PAINT()
	ON_COMMAND(IDC_PYEXECUTE_BUTTON, &CConsolePane::OnPyexecuteButton)
	ON_UPDATE_COMMAND_UI(IDC_PYEXECUTE_BUTTON, &CConsolePane::OnUpdatePyexecuteButton)
	//handlers for activating the buttons
	ON_UPDATE_COMMAND_UI(ID_CONSOLE_TOOLBARBUTTON_CLEAR, &CConsolePane::OnUpdateClearHistoryButton)
	ON_UPDATE_COMMAND_UI(ID_CONSOLE_TOOLBARBUTTON_SAVE, &CConsolePane::OnUpdateSaveHistoryButton)

	ON_EN_CHANGE(IDC_PYTHON_COMMAND_EDIT, &CConsolePane::OnEnChangePythonCommandEdit) 
END_MESSAGE_MAP()

CConsolePane::CConsolePane()
{
	const int max_items = 50; // maximum lines in history file

	// read the command history from a text file
	std::ifstream file("cmdhist.txt");
	
	if (file.good())
		while (!file.eof())
		{
			char buf[2048];
			file.getline(buf,2048,'\n');
			if (buf[0] != 0)
				history.push_back(buf);
		}

	file.close();

	std::ofstream file_out("cmdhist.txt");

	while((int)history.size()>max_items)
		history.erase(history.begin());
	for (std::list<CString>::iterator itor=history.begin(); itor!=history.end(); itor++)
	{
		file_out << (*itor).GetBuffer() << std::endl;
	}

	std::map<std::string, PyMethodDef*>::const_iterator itor;
	for (itor=CPythonInterface::getInstance().getModuleTable().begin(); itor!=CPythonInterface::getInstance().getModuleTable().end(); itor++)
	{
		std::list<CString> listFunctionTmp; // list with the method names for autocomplete
		PyMethodDef* pMethodTable = itor->second; // pointer to the python method table
		
		for (pMethodTable; pMethodTable->ml_name!=0; pMethodTable++)
		{
			listFunctionTmp.push_back(CString(pMethodTable->ml_name));
		}
		listFunctionTmp.sort();
		listModules.push_back(listFunctionTmp);
	}        

	lock = false;
	bHistory = true;
	iSelected = 1;
}

//! compute the layout of the console pane; the layout does not depend on a 
//! dialog resource but on result of this functions.
void CConsolePane::AdjustLayout()
{
	if (GetSafeHwnd() == NULL) return;
	CRect rectClient;
	GetClientRect(rectClient);

	int dy_button = 24;		// height of the execute button and of the command edit control
	int dx_button = 100;	// width of the execute button (fixed)
	int border = 5;			// border between the buttons and the frame of the pane

	//calculate the y-size of the toolbar
	int cyTlb = m_wndToolBar.CalcFixedLayout(FALSE,TRUE).cy;

	// we still have to add some spacing between the controls
	m_wndToolBar.SetWindowPos(NULL,rectClient.left,rectClient.top,rectClient.Width(),cyTlb, SWP_NOACTIVATE | SWP_NOZORDER);
	//m_wndToolBar.ShowWindow(SW_HIDE);

	m_wndTabs.SetWindowPos(NULL,rectClient.left,rectClient.top+cyTlb,rectClient.Width(),rectClient.Height()-dy_button-2*border-cyTlb, SWP_NOACTIVATE | SWP_NOZORDER);
	m_wndTabs.SetActiveTabBoldFont(TRUE);

	m_wndCmdLine.SetWindowPos(NULL,rectClient.left+border,rectClient.bottom-dy_button-border,rectClient.Width()-dx_button-2*border,dy_button, SWP_NOACTIVATE | SWP_NOZORDER);
	m_wndExecute.SetWindowPos(NULL,rectClient.Width()-dx_button,rectClient.bottom-dy_button-border,dx_button-border,dy_button, SWP_NOACTIVATE | WS_EX_TOPMOST);
	m_wndCmdLine.SetLimitText(120);

	// paint a BtnFace colored background
	CRect rc;
	GetClientRect(rc);
	FillRect(GetDC()->m_hDC, rc, (HBRUSH)COLOR_BTNSHADOW);
	RedrawWindow();
	m_wndExecute.RedrawWindow();
}


int CConsolePane::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CDockablePane::OnCreate(lpCreateStruct) == -1)
		return -1;

	CRect rectDummy;
	rectDummy.SetRectEmpty();

	if (!m_wndTabs.Create(CMFCTabCtrl::STYLE_3D_ROUNDED, rectDummy, this, IDD_CP_TABCTRL))
	{
		printf("Error while creating the tab control in CConsolePane\n");
		return -1;
	}

	if (!m_wndEdit.Create(WS_CHILD | WS_VISIBLE | WS_BORDER | WS_CLIPSIBLINGS | WS_CLIPCHILDREN | ES_MULTILINE | ES_READONLY | ES_LEFT | WS_HSCROLL |  WS_VSCROLL ,rectDummy,&m_wndTabs, IDD_CP_CMDLINEEDIT))
	{
		printf("Error while creating the main edit control\n");
		return -1;
	}

	if (!m_wndCmdLine.Create(WS_CHILD | WS_VISIBLE | WS_BORDER | WS_CLIPSIBLINGS | WS_CLIPCHILDREN | ES_AUTOHSCROLL ,rectDummy,this, IDC_PYTHON_COMMAND_EDIT))
	{
		printf("Error while creating the command edit control\n");
		return -1;
	}

	if (!m_wndExecute.Create("Execute",WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | WS_CLIPCHILDREN,rectDummy,this, IDC_PYEXECUTE_BUTTON))
	{
		printf("Error while creating the command execute button\n");
		return -1;
	}
	
	if (!m_wndProperty.Create(WS_CHILD | WS_VISIBLE | WS_BORDER | WS_CLIPSIBLINGS | WS_CLIPCHILDREN,rectDummy,&m_wndTabs, IDD_CP_PYPROPERTY))
	{
		printf("Error while creating the property grid control\n");
		return -1;
	}

	// create the control to be used in the tab for PyActions
	if (!m_wndPyActions.Create(WS_CHILD | WS_VISIBLE | WS_BORDER | WS_CLIPSIBLINGS | WS_CLIPCHILDREN,rectDummy,&m_wndTabs, IDD_CP_PYACTION))
	{
		printf("Error while creating the property grid control for python actions\n");
		return -1;
	}

	// create the control to be used by the history
	if (!m_wndHistory.Create(WS_CHILD | WS_VISIBLE | WS_BORDER | WS_CLIPSIBLINGS | WS_CLIPCHILDREN | LVS_LIST, rectDummy, &m_wndTabs,IDD_CP_HISTORY))
	{
		printf("Error while creating the property grid control for python actions\n");
		return -1;
	}

	// create the window with the inline source code
	if (!m_wndSourceEdit.Create(WS_CHILD | WS_VISIBLE | WS_BORDER | WS_CLIPSIBLINGS | WS_CLIPCHILDREN | ES_WANTRETURN | ES_MULTILINE | ES_LEFT | WS_HSCROLL | WS_VSCROLL , rectDummy, &m_wndTabs, IDD_CP_SOURCECODEEDIT))
	{
		printf("Error while creating the main source edit control\n");
		return -1;
	}	

	m_wndProperty.EnableDescriptionArea();

	// create toolbar
	m_wndToolBar.Create(this, AFX_DEFAULT_TOOLBAR_STYLE, IDR_TOOLBAR_CONSOLE);
	m_wndToolBar.LoadToolBar(IDR_TOOLBAR_CONSOLE, 0, 0, TRUE /* Ist gesperrt */);
	m_wndToolBar.SetPaneStyle(m_wndToolBar.GetPaneStyle() | CBRS_TOOLTIPS | CBRS_FLYBY);
	m_wndToolBar.SetOwner(this);
	
	// Alle Befehle werden über dieses Steuerelement geleitet, nicht über den übergeordneten Rahmen:
	m_wndToolBar.SetRouteCommandsViaFrame(FALSE);

	//Set text to the Buttons
	m_wndToolBar.GetButton(0)->m_bText = TRUE;
	m_wndToolBar.GetButton(0)->m_strText = "Delete";
	m_wndToolBar.GetButton(1)->m_bText = TRUE;
	m_wndToolBar.GetButton(1)->m_strText = "Save";
	m_wndToolBar.GetButton(2)->m_bText = TRUE;
	m_wndToolBar.GetButton(2)->m_strText = "Options";

	// add the window(s) to the tab
	m_wndTabs.AddTab(&m_wndEdit, _T("Console"), IDD_CP_CMDLINEEDIT);	
#ifndef WIRECENTER_LIGHTVERSION
	m_wndTabs.AddTab(&m_wndProperty, _T("Python"), IDD_CP_PYPROPERTY);
	m_wndTabs.AddTab(&m_wndPyActions, _T("Actions"), IDD_CP_PYACTION);
	m_wndTabs.AddTab(&m_wndHistory, _T("History"), IDD_CP_HISTORY);
	m_wndTabs.AddTab(&m_wndSourceEdit, _T("Script"), IDD_CP_SOURCECODEEDIT);
	m_wndTabs.EnableTabSwap(false); // disable swap to make sure, that the tab "Python" has the index 1 (solves a problem with the hit test for the grid control
	// populate the property grid with the python commands
	CreatePythonList();
#endif
	// now we do some fancy stuff with the style of the controls
	::DeleteObject(m_fntPropList.Detach());

	LOGFONT lf;
	afxGlobalData.fontRegular.GetLogFont(&lf);

	NONCLIENTMETRICS info;
	info.cbSize = sizeof(info);

	afxGlobalData.GetNonClientMetrics(info);

	lf.lfHeight = info.lfMenuFont.lfHeight;
	lf.lfWeight = info.lfMenuFont.lfWeight;
	lf.lfItalic = info.lfMenuFont.lfItalic;

	m_fntPropList.CreateFontIndirect(&lf);

	m_wndEdit.SetFont(&m_fntPropList);
	m_wndCmdLine.SetFont(&m_fntPropList);
	m_wndExecute.SetFont(&m_fntPropList);
	m_wndSourceEdit.SetFont(&m_fntPropList);

	AdjustLayout();

	EnableToolTips(TRUE);

#ifdef WIRECENTER_LIGHTVERSION
	// restrict the useage of this pane for the light version; we simply hide the controlls
	m_wndCmdLine.ShowWindow(SW_HIDE);
	m_wndExecute.ShowWindow(SW_HIDE);
#endif
	return 0;
}

void CConsolePane::CreatePythonList()
{
	// fill the property grid with name of the python commands
	std::map<std::string, PyMethodDef*>::const_iterator itor;
	for (itor = CPythonInterface::getInstance().getModuleTable().begin(); itor!=CPythonInterface::getInstance().getModuleTable().end(); itor++)
	{
		CMFCPropertyGridProperty* pTable = new CMFCPropertyGridProperty(itor->first.c_str(), 0, 0);
		m_wndProperty.AddProperty(pTable);
		for (PyMethodDef* ptr = itor->second; (ptr)->ml_meth!=0; ptr++)
		{
			CString str = "";
			if (ptr->ml_flags != METH_NOARGS)
			{
				// try to find the default parameter list in the commands documentation
				str = ptr->ml_doc;
				int b = str.Find("[",0);
				int e = str.Find("]",max(b,0));
				if (b>0 && e>0 && e>b+1)
					str = str.Mid(b+1, e-b-1);
				else
					str = "";
			}
			pTable->AddSubItem(new CMFCPropertyGridProperty(ptr->ml_name,str,ptr->ml_doc,0));
		}
	}

	// fill the list with the python actions of the system script
	std::map<std::string, std::string> actionMap;
	CMFCPropertyGridProperty* pSystem = new CMFCPropertyGridProperty("System Actions",0,0);
	m_wndPyActions.AddProperty(pSystem,0,0);
	if (CPythonInterface::getInstance().getActionMap(actionMap))
	{
		for (auto itor=actionMap.begin(); itor!=actionMap.end(); itor++)
		{
			CMFCPropertyGridProperty* pTable = new CMFCPropertyGridProperty(itor->first.c_str(),itor->second.c_str(),0,0);
			pSystem->AddSubItem(pTable);
		}
	}

	// fill the list of histroy items; 
	//! \todo we might move this block to update the histroy list whenever the list is changed
	m_wndHistory.DeleteAllItems();
	m_wndHistory.SetColumnWidth(0,1024);
	for (auto itor = history.rbegin(); itor!=history.rend(); ++itor)
	{
		m_wndHistory.InsertItem(0,*itor);
	}
}

void CConsolePane::OnSize(UINT nType, int cx, int cy)
{
	CDockablePane::OnSize(nType,cx,cy);
	AdjustLayout();
}

void CConsolePane::OnPyexecuteButton()
{
	// apply the execute to the multi-line input if active tab is active
	if (m_wndTabs.GetActiveTab()==4)
	{
		CString str;
		m_wndSourceEdit.GetWindowTextA(str);
		printf("> %s\n", str.GetBuffer());
		CPythonInterface::getInstance().runCommand(str.GetBuffer());
		return;
	}

	// print the executed command on the console window
	CString str;
	m_wndCmdLine.GetWindowTextA(str);
	printf("> %s\n", str.GetBuffer());

	// execute the command
	if(CPythonInterface::getInstance().runCommand(str.GetBuffer()))
	{
		// store the command in the history buffer
		if (history.size() > 0) 
			if (strcmp(history.back(),str.GetBuffer()) != 0) // check, if it equals the last element
				history.push_back(str.GetBuffer());
		// store the command in a textfile
		std::fstream file("cmdhist.txt", std::fstream::app );
		file << str.GetBuffer() << std::endl;
	}
		
	bHistory = true;
	iSelected = 0;
}

// we have to enable the button when ever the framework asks us; otherwise it 
// is disabled, strange but true
void CConsolePane::OnUpdatePyexecuteButton(CCmdUI *pCmdUI)
{ pCmdUI->Enable(); }

BOOL CConsolePane::PreTranslateMessage( MSG* pMsg )
{
	if ( pMsg->message == WM_KEYDOWN && GetFocus()->GetDlgCtrlID() == IDC_PYTHON_COMMAND_EDIT)
	{
		lock=false;
		switch (pMsg->wParam)
		{
		case VK_UP:
			{
				iSelected = iSelected-1;
				OnEnChangePythonCommandEdit();
				return true;
			}
		case VK_DOWN:
			{
				iSelected = iSelected+1;
				OnEnChangePythonCommandEdit();
				return true;
			}
		case VK_RIGHT: // if autocomplete, then set the cursor between the two brackets when arrow right
			{
				int sel_start, sel_end;
				m_wndCmdLine.GetSel(sel_start, sel_end);
				if (sel_start != sel_end)
				{
					CString sInput;
					m_wndCmdLine.GetWindowTextA(sInput);
					m_wndCmdLine.SetSel(sInput.GetLength()-iCursorOffset,sInput.GetLength()-iCursorOffset);
					return true;
				}
			}
		case VK_BACK:
			{
				bHistory = false;
				lock = true;
				break;
			}
		case VK_DELETE:
			{
				bHistory = false;
				lock = true;
				break;
			}
		default:
			{
				iSelected = 1;
				bHistory = false;
			}
		}
	}

	// pressing enter in the script tab is not handled here
	if (pMsg->message == WM_KEYDOWN && pMsg->wParam == VK_RETURN && m_wndTabs.GetActiveTab() == 4)
	{
		return false;
	}

	// execute the script if enter is pressed 
	if (pMsg->message == WM_KEYDOWN && pMsg->wParam == VK_RETURN)
	{
		OnPyexecuteButton();
		return true;
	}
	
	// copy the respective command from the python command list to the command line 
	if (pMsg->message == WM_LBUTTONDBLCLK && m_wndTabs.GetActiveTab()==1)
	{
		CPoint point;
		point = (CPoint)pMsg->lParam;
		CMFCPropertyGridProperty *pProp= m_wndProperty.HitTest(point);
		if (pProp)
		{
			CString str;
			CString str2 = pProp->GetValue();
			// we can only call the parents getName function if a parent exists
			if (pProp->GetParent())	
				str.Format("%s.%s(%s)",pProp->GetParent()->GetName(),pProp->GetName(),str2);
			else
				return TRUE;
			m_wndCmdLine.SetFocus();
			lock=true;
			m_wndCmdLine.SetWindowTextA(str);
			m_wndCmdLine.SetSel((int)(strlen(str)-strlen(str2)-1),(int)(strlen(str)-1)); // select the arguments to overwrite them directly
			return TRUE;
		}
	}

	// execute the respective system action after double click and update WireCenter GUI afterwards
	if (pMsg->message == WM_LBUTTONDBLCLK && m_wndTabs.GetActiveTab()==2)
	{
		CPoint point;
		point = (CPoint)pMsg->lParam;
		CMFCPropertyGridProperty *pProp= m_wndPyActions.HitTest(point);
		if (pProp)
		{
			// if a item was double clicked we try to execute the respective system action
			CPythonInterface::getInstance().runSystemScript(pProp->GetName());
			CWireCenterView::This->GetDocument()->UpdateAllViews(0);
			return TRUE;
		}
	}

	// copy the respective command from the histroy into the command line
	if (pMsg->message == WM_LBUTTONDBLCLK && m_wndTabs.GetActiveTab() == 3)
	{
		CPoint point;
		point = (CPoint)pMsg->lParam;
		int res = m_wndHistory.HitTest(point);
		if (res >= 0)
		{
			CString str;
			str = m_wndHistory.GetItemText(res,0);
			m_wndCmdLine.SetFocus();
			lock = true;
			m_wndCmdLine.SetWindowTextA(str);
			return TRUE;
		}
	}

	// support for strg+c, strg+v, strg+x
	if (pMsg->message == WM_KEYDOWN && IsDialogMessage(pMsg)) 
		return TRUE;

	return CDockablePane::PreTranslateMessage(pMsg);
}

void CConsolePane::OnEnChangePythonCommandEdit()
{
	CEdit* pCommandLine = (CEdit*)GetDlgItem(IDC_PYTHON_COMMAND_EDIT);

	CString sInput;
	pCommandLine->GetWindowTextA(sInput);

	int sel_start, sel_end;
	pCommandLine->GetSel(sel_start,sel_end);
	CString sInput_deselected;
	sInput_deselected=sInput.Left(sel_start);

	CString sSearch = "";
	CString sBegin = "";

	if (lock == false)
	{
		if (strlen(sInput)==0) // if length of input = 0, enable history mode
			{
				bHistory = true;
			}
		if (bHistory == true) // method for the history mode (controlled by up and down arrow)
		{
			std::list<CString>::reverse_iterator it;
			int iNr = 0;
			CString sCompare = "";
			iSelected=min(iSelected, 1);
			if (iSelected <= 0)
			{
				it=history.rbegin(); // iterate backwards
				for (; it != history.rend(); ++it) // search the matching string
				{
					sCompare =* it; // string from python to compare
					iNr++;
					if (iNr == 1-iSelected)
						break;
				}
				iSelected = 1-iNr;
			}
			lock = true;
			m_wndCmdLine.SetWindowTextA(sCompare);		
			return;
		}
		else // method for autocomplete
		{
			if (sel_end != strlen(sInput)) // catch states, where the cursor respective the selection isn't at the end
				return;
			if (strlen(sInput) < 1) // catch states, where the input is empty
				return;

			std::map<std::string, PyMethodDef*>::const_iterator it1; // iterator for the map containing the name and a pointer to the registered modules
			std::list<std::list<CString>>::iterator it2;	// iterator for the list, which contains the method tables for each registered module
			std::list<CString>::iterator it3;				// iterator for a method table
			
			it2 = listModules.begin();
			int iNr = 0;
			iCursorOffset = 0; // offset for setting the cursor after autocomplete. To set the cursor into the brackets () it has to be set to 1
			
			CString sProposal = sInput;
			CString sCompare; // string from list to compare with the input string
			sSearch = sInput_deselected;
			iSelected = max(iSelected, 1);

			for (it1 = CPythonInterface::getInstance().getModuleTable().begin();it1!=CPythonInterface::getInstance().getModuleTable().end();it1++) // search the matching string
			{
				sCompare = it1->first.c_str(); // string from list to compare
				if (sCompare.Find(sSearch) == 0) // phase 1: search the corresponding module like Iapp or Irobot
				{
					sProposal = sBegin + sCompare;
					iNr++;
					if (iNr == iSelected)
					{
						break; // start phase 2
					}
				}
				else if(sSearch.Find(sCompare) == 0) // phase 2: search the corresponding function for the selected module
				{
					CString sModuleName = it1->first.c_str(); // name of the module
					it3 = (*it2).begin(); // iterator which references on the list with the function names
					
					if (strcmp(sInput.Left(strlen(sModuleName)+1),sModuleName+".") == 0) // check, if the begin string is a registered module + point
					{
						sBegin = sInput.Left(strlen(sModuleName)+1);
						sSearch = sInput_deselected.Right(strlen(sInput_deselected)-strlen(sModuleName)-1);
						std::list<CString>::iterator it;
						iNr = 0;
						sProposal = sInput;
						iSelected = max(iSelected, 1);
						for (it3; it3!=(*it2).end(); it3++) // search the matching string
						{
							sCompare=*it3; // string from list to compare
							if (sCompare.Find(sSearch) == 0)
							{
								sProposal = sBegin + sCompare + "()";
								iCursorOffset = 1; // set the cursor between the brackets
								iNr++;
								if (iNr == iSelected)
									break;
							}
						}
					}
					break;
				}
				it2++;
			}

			iSelected = iNr;
			m_wndCmdLine.SetWindowTextA(sProposal); // write the string into the commandline
			m_wndCmdLine.SetSel((int)strlen(sInput_deselected),(int)strlen(sProposal)); // select the new part
		}
	}
	lock = true;
}

void CConsolePane::OnUpdateSaveHistoryButton(CCmdUI *pCmdUI)
{ pCmdUI->Enable(); }

void CConsolePane::OnUpdateClearHistoryButton(CCmdUI *pCmdUI)
{ pCmdUI->Enable(); }
