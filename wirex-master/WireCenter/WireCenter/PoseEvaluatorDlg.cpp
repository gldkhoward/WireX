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

// PoseEvaluatorDlg.cpp: Implementierungsdatei
//

#include "stdafx.h"
#include "WireCenter.h"
#include "PoseEvaluatorDlg.h"
#include "afxdialogex.h"
#include "AlgorithmConfigPane.h"
#include <WireLib/PoseEvaluator.h>

// CPoseEvaluatorDlg-Dialogfeld

IMPLEMENT_DYNAMIC(CPoseEvaluatorDlg, CDialogEx)

CPoseEvaluatorDlg::CPoseEvaluatorDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CPoseEvaluatorDlg::IDD, pParent)
	, m_PosePropertyCount(0)
	, m_AttributeCount(0)
{
}

CPoseEvaluatorDlg::~CPoseEvaluatorDlg()
{
}

void CPoseEvaluatorDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_ACTIVE_EVALUATOR_LIST, m_ActiveProperties);
	DDX_Control(pDX, IDC_INACTIVE_EVALUATOR_LIST, m_PosePropertyTypes);
	DDX_Control(pDX, IDC_EVALUATOR_ATTRIBUTE_LIST, m_Attibutes);
	DDX_Control(pDX, IDC_EVALUATOR_PROPERTIES_MFCPROPERTYGRID, m_Configuration);
	DDX_Check(pDX, IDC_CHECKZEROTHRESHOLD, m_bSetZeroThreshold);
	DDX_Text(pDX, IDC_NUMBER_OF_PROPERTIES_EDIT, m_PosePropertyCount);
	DDX_Text(pDX, IDC_NUMBER_OF_ATTRIBUTES_EDIT, m_AttributeCount);
}


BEGIN_MESSAGE_MAP(CPoseEvaluatorDlg, CDialogEx)
	ON_REGISTERED_MESSAGE(AFX_WM_PROPERTY_CHANGED, &CPoseEvaluatorDlg::OnPropertyChanged)
	ON_BN_CLICKED(IDC_ADD_POSEPROPERTY_BUTTON, &CPoseEvaluatorDlg::OnBnClickedAddPosepropertyButton)
	ON_BN_CLICKED(IDC_REMOVE_POSEPROPERTY_BUTTON, &CPoseEvaluatorDlg::OnBnClickedRemovePosepropertyButton)
	ON_BN_CLICKED(IDC_UP_POSEPROPERTY_BUTTON, &CPoseEvaluatorDlg::OnBnClickedUpPosepropertyButton)
	ON_BN_CLICKED(IDC_DOWN_POSEPROPERTY_BUTTON, &CPoseEvaluatorDlg::OnBnClickedDownPosepropertyButton)
	ON_LBN_SELCHANGE(IDC_ACTIVE_EVALUATOR_LIST, &CPoseEvaluatorDlg::OnLbnSelchangeActiveEvaluatorList)
	ON_LBN_DBLCLK(IDC_ACTIVE_EVALUATOR_LIST, &CPoseEvaluatorDlg::OnLbnDblclkActiveEvaluatorList)
	ON_LBN_DBLCLK(IDC_INACTIVE_EVALUATOR_LIST, &CPoseEvaluatorDlg::OnLbnDblclkInactiveEvaluatorList)
	ON_BN_CLICKED(IDC_CHECKZEROTHRESHOLD, &CPoseEvaluatorDlg::OnBnClickedCheckzerothreshold)
END_MESSAGE_MAP()


void CPoseEvaluatorDlg::UpdateLists()
{
	// clear the data and the elements before recalculation
	m_AttributeCount=0;
	m_PosePropertyCount=0;
	m_bSetZeroThreshold = pEvaluator->bCheckZeroThreshold ? 1 : 0;
	
	m_ActiveProperties.ResetContent();
	//m_PosePropertyTypes.ResetContent();

	// loop through backwarts through all entities and put them into the respective lists
	for (auto itor=pEvaluator->getActiveProperties().rbegin(); itor!=pEvaluator->getActiveProperties().rend(); ++itor)
	{
		// skip the entry if it is empty; this should never happen; however we test it
		if (!(*itor)->Property)
			continue;
		// convert the pointer to the property to a int32 which will serve as a handle
		const PCRL::CActivePoseProperty *pAPP = (itor.operator*());
		m_ActiveProperties.InsertString(0,PCRL::emPoseProperty::Names[(*itor)->Type]);
		m_ActiveProperties.SetItemData(0,(DWORD_PTR)pAPP);
		m_PosePropertyCount++;
		m_AttributeCount+=(*itor)->Property->getPropertyCount();
	}

	// upload the latest changes to the screen
	UpdateData(FALSE);
}

// CPoseEvaluatorDlg-Meldungshandler

// move an entry from the inactive list to the active list
void CPoseEvaluatorDlg::OnBnClickedAddPosepropertyButton()
{
	// get the data item of the selected element
	int cur = m_PosePropertyTypes.GetCurSel();
	if (cur<0)
		return;
	PCRL::emPoseProperty::Type PPT = (PCRL::emPoseProperty::Type)(m_PosePropertyTypes.GetItemData(cur));
	pEvaluator->attachProperty(new PCRL::CActivePoseProperty(PPT, pEvaluator->createPoseProperty(PPT)));

	UpdateLists();

	// select the added element
	m_ActiveProperties.SetCurSel(m_ActiveProperties.GetCount()-1);
	OnLbnSelchangeActiveEvaluatorList();
}

// move an entry from the active list to the inactive list
void CPoseEvaluatorDlg::OnBnClickedRemovePosepropertyButton()
{
	// get the data item of the selected element
	int cur = m_ActiveProperties.GetCurSel();
	if (cur<0)
		return;
	PCRL::CActivePoseProperty* pAPP = (PCRL::CActivePoseProperty*)(m_ActiveProperties.GetItemData(cur));
	if (!pAPP)
		return;
	pEvaluator->removeProperty(pAPP);

	UpdateLists();

	// after moving the element, select a remaining element in the list
	if (m_ActiveProperties.GetCount() > 0)
	{
		if (cur < m_ActiveProperties.GetCount())
			m_ActiveProperties.SetCurSel(cur);
		else
			m_ActiveProperties.SetCurSel(m_ActiveProperties.GetCount()-1);
	}
	OnLbnSelchangeActiveEvaluatorList();
}


void CPoseEvaluatorDlg::OnBnClickedUpPosepropertyButton()
{
	// get the current entry in the active list box list
	int cur = m_ActiveProperties.GetCurSel();
	if (cur<=0)
		return; // no item or the first item is selected; we cannot move up

	PCRL::CActivePoseProperty *pAPP = (PCRL::CActivePoseProperty*)(m_ActiveProperties.GetItemData(cur));
	if (!pAPP)
		return;

	pEvaluator->moveUp(pAPP->Property);

	UpdateLists();

	m_ActiveProperties.SetCurSel(cur-1);
	OnLbnSelchangeActiveEvaluatorList();
}

// basically the same as the move up function but with slight modification to identify
// the successor and also for the range tests
void CPoseEvaluatorDlg::OnBnClickedDownPosepropertyButton()
{
	// get the current entry in the active list box list
	int cur = m_ActiveProperties.GetCurSel();
	if (cur<0)
		return;	// no item selected
	if (cur >= m_ActiveProperties.GetCount()-1)
		return; // the last item is selected, we cannot move that item down

	PCRL::CActivePoseProperty *pAPP = (PCRL::CActivePoseProperty*)(m_ActiveProperties.GetItemData(cur));
	if (!pAPP)
		return;

	pEvaluator->moveDown(pAPP->Property);
	
	UpdateLists();

	m_ActiveProperties.SetCurSel(cur+1);
	OnLbnSelchangeActiveEvaluatorList();
}


void CPoseEvaluatorDlg::OnLbnSelchangeActiveEvaluatorList()
{
	// remove all items from the list
	m_Attibutes.ResetContent();
	int cur = m_ActiveProperties.GetCurSel();
	if (cur<0)
		return;
	
	// add the configuration attributes of the current property to the respective CMFCPropertyGrid UI
	PCRL::CActivePoseProperty *pAPP = (PCRL::CActivePoseProperty*)(m_ActiveProperties.GetItemData(cur));
	if (!pAPP)
		return;
	vector<string> names;
	pAPP->Property->getPropertyNames(names);
	for (auto itor = names.begin(); itor!=names.end(); ++itor)
		m_Attibutes.AddString(itor->c_str());

	m_Configuration.RemoveAll();
	CPropertyGridReflectorManager mgt;
	
	list<PCRL::CActivePoseProperty*> appList = pEvaluator->getActiveProperties();
	int i = 0;
	for (auto itor = appList.begin(); itor!=appList.end(); ++itor, ++i){
		// Create Base-GridProperty for new PoseProperty. Needed to distinguish between two or more PoseProperties of the same type, as their xpaths will be the same
		CString cstring("Property ");
		cstring.Append(to_string((long long)i).c_str());
		CMFCPropertyGridProperty* gridProperty = new CMFCPropertyGridProperty(_T(cstring));

		// iterate through all elements in the binding table of reflection and extract
		// all information that we can get.
		PCRL::CReflection& Reflector = (*itor)->Property->Reflector();
		for (auto itor2 = Reflector.getValueMap().cbegin(); itor2!=Reflector.getValueMap().cend(); ++itor2)
			mgt.setNode(gridProperty,itor2->second.xpath,itor2->second);
		
		//expand only the currently selected property
		if((*itor)==pAPP)
			gridProperty->Expand(true);
		else
			gridProperty->Expand(false);

		//add only evaluators with settings to propertygrid to keep it as clear as possible
		if(gridProperty->GetSubItemsCount() > 0)
			m_Configuration.AddProperty(gridProperty);
	}
}


BOOL CPoseEvaluatorDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  hier zusätzliche Initialisierung hinzufügen.
	UpdateLists();

	for (int i=0; PCRL::emPoseProperty::Names[i]!=0; i++)
	{
		// convert the pointer to the property to a int32 which will serve as a handle
		const PCRL::emPoseProperty::Type PPT = (PCRL::emPoseProperty::Type)i;
		m_PosePropertyTypes.InsertString(i,PCRL::emPoseProperty::Names[PPT]);
		m_PosePropertyTypes.SetItemData(i,PPT);
	}
	
	// the following code sized the columns in the control to an acceptable value: set width of both colums to 50% by forcing a call of OnSize()
	CRect rect;
	m_Configuration.GetWindowRect(&rect);
	m_Configuration.PostMessage(WM_SIZE, 0, MAKELONG(rect.Width(),rect.Height()));

	return TRUE;  // return TRUE unless you set the focus to a control
}


void CPoseEvaluatorDlg::OnLbnDblclkActiveEvaluatorList()
{
	OnBnClickedRemovePosepropertyButton();
}


void CPoseEvaluatorDlg::OnLbnDblclkInactiveEvaluatorList()
{
	OnBnClickedAddPosepropertyButton();
}

/*! This callback function is triggered when an element of the property grid 
 *  was edited by the user. If the modification was feasible we store the
 *  changed information in the connected reflection variable
 */
LRESULT CPoseEvaluatorDlg::OnPropertyChanged(__in WPARAM wparam, __in LPARAM lparam)
{
	// get a pointer the the respective property element 
	CMFCPropertyGridProperty* pProp = (CMFCPropertyGridProperty*) lparam; 
	// let the reflection manager do the work for us
	CPropertyGridReflectorManager mgt;
	return mgt.getNode(pProp);
}

void CPoseEvaluatorDlg::OnBnClickedCheckzerothreshold()
{
	pEvaluator->bCheckZeroThreshold = !pEvaluator->bCheckZeroThreshold;
}
