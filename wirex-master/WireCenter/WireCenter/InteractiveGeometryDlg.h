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

/*! \file InteractivePoseDlg.h
 *
 *	\author   Andreas Pott
 *
 *  \brief A non-modal dialog box for interactive change of the robot geoemtry
 */

#pragma once
#include "afxwin.h"
#include <vector>
#include "UserMessages.h"
#include <WireLib/Workspace.h>
#include <WireLib/RobotDocument.h>

//! a simple toolbar for the interactive geometry control
class CInteractiveGeometryToolbar : public CMFCToolBar
{
	virtual void OnUpdateCmdUI(CFrameWnd* /*pTarget*/, BOOL bDisableIfNoHndler)
	{
		CMFCToolBar::OnUpdateCmdUI((CFrameWnd*) GetOwner(), bDisableIfNoHndler);
	}

	virtual BOOL AllowShowOnList() const { return FALSE; }
};

// new class as pane test
class CInteractiveGeometryPane : public CDockablePane
{
public:
	CInteractiveGeometryPane();
	virtual ~CInteractiveGeometryPane() {}

private:
	double smallstep;
	double largestep;
	int currentID;
	std::vector <CString> parameterLabels;		//!< Contains labels for the parameters in the listbox
	CMFCPropertyGridProperty *general;			//!< the group with general information
	CMFCPropertyGridProperty *workspaceHull;	//!< the group with workspace hull statistics
	CMFCPropertyGridProperty *workspaceGrid;	//!< the group with workspace grid statistics
	CMFCPropertyGridProperty *workspaceCrosssection;	//!< the group with workspace cross section statistics

protected:
	DECLARE_MESSAGE_MAP()

	BOOL m_AutomaticWorkspaceUpdate;
	double m_currentValue;

	void AdjustLayout();
	void changeCurrentItem(double value);

public:
	void setWorkspaceProperty(const PCRL::CWorkspaceHull* pHull=0);
	void setWorkspaceGridProperty(const PCRL::CWorkspaceGrid* pGrid=0);
	void setWorkspaceCrosssectionProperty(const PCRL::CWorkspaceCrosssection* pCS=0);
	void setRobotProperties(const PCRL::CRobotDocument* pRobotDoc);

	afx_msg LRESULT OnPropertyChanged(__in WPARAM wparam, __in LPARAM lparam);

	//! updates the interactive geometry pane with the current robot and base geometries
	void updateAllData(bool AllowWorkspaceUpdate=false);

protected:
	CFont m_fntPropList;

	CMFCPropertyGridCtrl m_wndProperties;		//!< the main contrl showing the grid with the parameter
	CInteractiveGeometryToolbar m_wndToolBar;	//!< the tool bar showing the available commands

protected:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnSize(UINT nType, int cx, int cy);
public:
	afx_msg void OnIgpApply();
	afx_msg void OnIgpAutoapply();
	afx_msg void OnIgpDec();
	afx_msg void OnIgpFastDec();
	afx_msg void OnIgpFastInc();
	afx_msg void OnIgpInc();
	afx_msg void OnIgpOptions();
	afx_msg void OnUpdateIgpApply(CCmdUI *pCmdUI);
	afx_msg void OnUpdateIgpAutoapply(CCmdUI *pCmdUI);
	afx_msg void OnUpdateIgpDec(CCmdUI *pCmdUI);
	afx_msg void OnUpdateIgpFastDec(CCmdUI *pCmdUI);
	afx_msg void OnUpdateIgpFastInc(CCmdUI *pCmdUI);
	afx_msg void OnUpdateIgpInc(CCmdUI *pCmdUI);
	afx_msg void OnUpdateIgpOptions(CCmdUI *pCmdUI);
};
