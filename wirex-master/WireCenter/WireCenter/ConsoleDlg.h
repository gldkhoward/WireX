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

/*! \file ConsoleDlg.h
 *
 *	\author   Andreas Pott
 *
 *  \dependency
 *		Python      for scripting engine
 *
 *  \brief Provide a console window. This class implements the 
 *  end of the named pipe created and WireCenter.cpp to show
 *  printf and cout in a window. Furthermore, the class implements
 *  a python command line to allow for direct execution of single
 *  python commands.
 */

#pragma once

#include <list>
#include <map>


//! a simple toolbar for the console control pane
class CConsoleToolbar : public CMFCToolBar
{
	virtual void OnUpdateCmdUI(CFrameWnd* /*pTarget*/, BOOL bDisableIfNoHndler)
	{
		CMFCToolBar::OnUpdateCmdUI((CFrameWnd*) GetOwner(), bDisableIfNoHndler);
	}

	virtual BOOL AllowShowOnList() const { return FALSE; }
};


/*! The prototype of a tool window ported to VS2010 pane structure. 
 *  It seems that implementing a message_map with OnCreate and OnSize is the 
 *  key to avoid program crashes when implementing a CDockablePane derived window. 
 *  The functions to handle the python execute button as well as the auto-complete 
 *  and history feature where ported from the CConsoleDlg class. 
 */
class CConsolePane : public CDockablePane
{
private:
	enum { IDD_CP_BASEID = 0,
		IDD_CP_TABCTRL = 1,
		IDD_CP_CMDLINEEDIT = 2,
		IDD_CP_PYPROPERTY = 5,
		IDD_CP_PYACTION = 6,
		IDD_CP_HISTORY = 7,
		IDD_CP_SOURCECODEEDIT = 8,
	};

public:
	CConsolePane();
	virtual ~CConsolePane() {}
	
// History and autocomplete	
	std::list<std::list<CString>> listModules;
	bool lock;		//!< control edit mode for autocomplete function
	bool bHistory;
	int iSelected;			//!< state in autocomplete function
	int iCursorOffset;		//!< cursor position for autocomplete function

// Attribute: The controls used in this pane
	CEdit m_wndEdit;		//!< the content of the console window
	CEdit m_wndSourceEdit;	//!< the content of the multi-line source editor
protected:
	CEdit m_wndCmdLine;		//!< the input line for python commands
	CMFCPropertyGridCtrl m_wndProperty;	//!< list with all python commands
	CMFCPropertyGridCtrl m_wndPyActions;	//!< list of all actions provided by pyhon system script
	CListCtrl m_wndHistory;		//!< list all entries of the histroy list
	CButton m_wndExecute;	//!< the execute button to start the command line	
	CFont m_fntPropList;	//!< this object is needed to change the font of the control
	CConsoleToolbar m_wndToolBar;

	CMFCTabCtrl m_wndTabs;	//!< a tab control to allow for different overlaying windows
	//! The current Python Command to be executed
	std::list<CString> history;

	//! calculate the layout of the controls after resizing the pane
	void AdjustLayout();
	void CreatePythonList();
protected:
	BOOL PreTranslateMessage( MSG* pMsg );
	DECLARE_MESSAGE_MAP()
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnPyexecuteButton();
	afx_msg void OnClearHistoryButton();
	afx_msg void OnSaveHistoryButton();
	afx_msg void OnUpdateClearHistoryButton(CCmdUI *pCmdUI);
	afx_msg void OnUpdateSaveHistoryButton(CCmdUI *pCmdUI);

protected:
	afx_msg void OnEnChangePythonCommandEdit();
	afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnUpdatePyexecuteButton(CCmdUI *pCmdUI);
};
