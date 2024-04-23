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

/*! \file AlgorithmConfigPane.h
 *
 *	\author   Andreas Pott
 *
 *  \brief A pane element, i.e. a non-model window to configure
 *  the algorithm settings exposed through reflection. 
 *  AlgorthmConfig is a power user API to configure the low-level
 *  parameter setting of WireLibs objects. Since the parameterrs
 *  to be configured are exposed through CRefelctions introspection
 *  the AlgorithmConfig class is capable to automatically adopt
 *  any new parameter visible by introspection.
 *
 *  \todo Add the functions that lets the user edit the content of 
 *  of the fields shown in the property grid. Currently, the support
 *  for chaning the value is very limited and error prone. 
 */

#pragma once
#include "afxwin.h"
#include <vector>
#include "UserMessages.h"
#include <WireLib/Reflection.h>

//! a simple toolbar for the interactive geometry control
class CAlgorithmConfigToolbar : public CMFCToolBar
{
	virtual void OnUpdateCmdUI(CFrameWnd* /*pTarget*/, BOOL bDisableIfNoHndler)
	{
		CMFCToolBar::OnUpdateCmdUI((CFrameWnd*) GetOwner(), bDisableIfNoHndler);
	}

	virtual BOOL AllowShowOnList() const { return FALSE; }
};

// new class as pane test
class CAlgorithmConfigPane : public CDockablePane
{
public:
	CAlgorithmConfigPane();
	virtual ~CAlgorithmConfigPane() {}

private:
	CMFCPropertyGridProperty *general;		//!< the group with general information


protected:
	DECLARE_MESSAGE_MAP()

	void AdjustLayout();
//	void changeCurrentItem(double value);

public:
	afx_msg LRESULT OnPropertyChanged(__in WPARAM wparam, __in LPARAM lparam);

	//! updates the interactive geometry pane with the current robot and base geometries
//	void updateAllData();
	void addAlgorithm(PCRL::CReflection& Reflector);

protected:
	CFont m_fntPropList;

	CMFCPropertyGridCtrl m_wndProperties;		//!< the main contrl showing the grid with the parameter
	CAlgorithmConfigToolbar m_wndToolBar;	//!< the tool bar showing the available commands

protected:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnSize(UINT nType, int cx, int cy);
public:
};

/*! this class provides helper functions to visualize the content of class
 *  that can be traversed at runtime through the CReflection mechanism in
 *  a CMFCPropertyGrid.
 *  This class makes it easy to connect the respective MFC control with the
 *  introspection implemented through CReflection.
 *  A future version of this class will be implement in its own files to give
 *  other windows also access to this nice mechanism.
 *  \todo Implement the new function that allows for generation of dropdown
 *  items on enum types.
 */
class CPropertyGridReflectorManager
{
public:
	bool setNode(CMFCPropertyGridProperty* root, const string& xpath, const PCRL::TReflectionVariant& value);
	void setAttribute(CMFCPropertyGridProperty* root, const string& attrib, const PCRL::TReflectionVariant& value);
	bool CPropertyGridReflectorManager::getNode(CMFCPropertyGridProperty* pProp);
};