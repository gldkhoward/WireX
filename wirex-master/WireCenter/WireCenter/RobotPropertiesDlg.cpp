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

// RobotPropertiesDlg.cpp: Implementierungsdatei
//

#include "stdafx.h"
#include "WireCenter.h"
#include "RobotPropertiesDlg.h"
#include <WireLib/RobotDocument.h>
#include <WireLib/GeometryGenerator.h>
#include "WireCenterView.h"


// CRobotPropertiesDlg-Dialogfeld

IMPLEMENT_DYNAMIC(CRobotPropertiesDlg, CDialog)

CRobotPropertiesDlg::CRobotPropertiesDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CRobotPropertiesDlg::IDD, pParent)
	, m_now(8)
	, m_motionPattern(5)
	, m_dof(6)
	, m_Name(_T("Cable Robot"))
	, m_Author(_T("Fraunhofer IPA"))
	, m_ID(_T(""))
	, m_Desc(_T(""))
{
}

CRobotPropertiesDlg::~CRobotPropertiesDlg()
{
}

void CRobotPropertiesDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_NUMBEROFWIRES_EDIT, m_now);
	DDX_CBIndex(pDX, IDC_MOTIONPATTERN_COMBO, m_motionPattern);

	//an image control for determine the CMFCRibbonPropertyList space
	DDX_Control(pDX, IDC_PROPLISTLOCATION_IMAGE, m_wndPropListLocation);
	//a second image control for determine the CMFCRibbonPropertyList space
	DDX_Control(pDX, IDC_PROPLISTLOCATION2_IMAGE, m_wndPropListLocation2);
	//a third image control for determine the CMFCRibbonPropertyList space
	DDX_Control(pDX, IDC_PROPLISTLOCATION3_IMAGE, m_wndPropListLocation3);
	
	//parametrized robot selection combo box
	DDX_Control(pDX, IDC_PARAMETRICROBOTS_COMBO, m_parametricrobots_combobox);
	
	//the second selection combo box
	DDX_Control(pDX, IDC_PARAMETRICROBOTS2_COMBO, m_parametricrobots2_combobox);

	//the radiobutton controls (necessary because of problems with the resourceID)
	// => GetCheckedRadioButton function search in an interval which is not correct ...
	DDX_Control(pDX, IDC_COMPLETEDESIGN_RADIO, m_completedesign_radio);
	DDX_Control(pDX, IDC_SEPERATECONFIGURATION_RADIO, m_seperateconfiguration_radio);

	//the checkbox controls
	DDX_Control(pDX, IDC_TRANSFORMFRAME_CHECKBOX, m_transformframe_checkbox);
	DDX_Control(pDX, IDC_TRANSFORMPLATFORM_CHECKBOX, m_transformplatform_checkbox);

	//other text controls
	DDX_Text(pDX, IDC_DEGREEOFFREEDOM_EDIT, m_dof);
	DDX_Text(pDX, IDC_ROBOTNAME_EDIT, m_Name);
	DDX_Text(pDX, IDC_ROBOTAUTHOR_EDIT, m_Author);
	DDX_Text(pDX, IDC_ROBOTID_EDIT, m_ID);
	DDX_Text(pDX, IDC_ROBOTDESC_EDIT, m_Desc);
}


BEGIN_MESSAGE_MAP(CRobotPropertiesDlg, CDialog)
	ON_CBN_SELCHANGE(IDC_MOTIONPATTERN_COMBO, &CRobotPropertiesDlg::OnCbnSelchangeMotionpatternCombo)
	ON_EN_CHANGE(IDC_NUMBEROFWIRES_EDIT, &CRobotPropertiesDlg::OnEnChangeNumberofwiresEdit)
	ON_CBN_SELCHANGE(IDC_PARAMETRICROBOTS_COMBO, &CRobotPropertiesDlg::OnCbnSelchangeParametricrobotsCombo)
	ON_CBN_SELCHANGE(IDC_PARAMETRICROBOTS2_COMBO, &CRobotPropertiesDlg::OnCbnSelchangeParametricrobots2Combo)
	//map all radiobutton clicks to one handler
	ON_BN_CLICKED(IDC_COMPLETEDESIGN_RADIO, &CRobotPropertiesDlg::OnClickedRadio)
	ON_BN_CLICKED(IDC_SEPERATECONFIGURATION_RADIO, &CRobotPropertiesDlg::OnClickedRadio)
	//map all checkbox clicks to one handler too
	ON_BN_CLICKED(IDC_TRANSFORMFRAME_CHECKBOX, &CRobotPropertiesDlg::OnClickedCheckbox)
	ON_BN_CLICKED(IDC_TRANSFORMPLATFORM_CHECKBOX, &CRobotPropertiesDlg::OnClickedCheckbox)
	ON_BN_CLICKED(IDOK, &CRobotPropertiesDlg::OnBnClickedOk)
END_MESSAGE_MAP()

PCRL::CRobotData::MotionPatternType CRobotPropertiesDlg::getMotionPattern()
{
	switch (m_motionPattern)
	{
	case 0: return PCRL::CRobotData::MP1T;
	case 1: return PCRL::CRobotData::MP2T;
	case 2: return PCRL::CRobotData::MP3T;
	case 3: return PCRL::CRobotData::MP1R2T;
	case 4: return PCRL::CRobotData::MP2R3T;
	case 5:
	default: return PCRL::CRobotData::MP3R3T;
	}
}

void CRobotPropertiesDlg::ListSuitableRobotModels(CComboBox* pTarget, bool b_effects_frame, bool b_effects_platform, bool b_is_transformator)
{
	//clear old combobox
	pTarget->ResetContent();

	//List all suitable models
	int count = -1, pos=0;
	for (vector<PCRL::CGeometryGenerator*>::iterator itor=RDoc.ParametricModels.begin(); itor!=RDoc.ParametricModels.end(); itor++)
	{
		//count up with the position in the ParametricModelsList
		count++;

		//if motionpattern doesn't match -> next turn
		if (((*itor)->getMotionPattern() & RDoc.getMotionPattern()) == 0) continue;

		//if generator scope doesn't match -> next turn
		if (((*itor)->effectsFrame() != b_effects_frame)) continue;
		if (((*itor)->effectsPlatform() != b_effects_platform)) continue;
		if (((*itor)->isTransformator() != b_is_transformator)) continue;

		//if NoW is identical -> add string to combobox
		if ((*itor)->getNow() == RDoc.getNow())
		{
			//add the ParametricModel to the list and save position in combobox
			pos = pTarget->AddString((*itor)->getName().c_str());
			//store position from ParametricsModels list in the ItemDataField of the matching comboedit-field
			pTarget->SetItemData(pos,count);
		}
	}

	//set new value as current selected
	pTarget->SetCurSel(0);
}

void CRobotPropertiesDlg::BuiltListPropertyDlg(CStatic* pTarget_space_defined_by_image, CMFCPropertyGridCtrl* pTarget_property_control, CComboBox* pCombobox_for_choosing)
{
	//find correct robot parametrisation by getting the "ItemData"
	int choosen_robot = pCombobox_for_choosing->GetItemData(pCombobox_for_choosing->GetCurSel());

	//if no entries in combobox -> nothing to do here ... bye
	if (choosen_robot==-1) return;
	
	//create CMFCPropertyGrid and its properties
	CRect rectPropList;

	//get position from the image object
	pTarget_space_defined_by_image->GetClientRect(&rectPropList);
	pTarget_space_defined_by_image->MapWindowPoints(this, &rectPropList);

	//destroy the former window
	if (pTarget_property_control) pTarget_property_control->DestroyWindow();

	//create a new control with special default properties
	pTarget_property_control->Create(WS_CHILD | WS_VISIBLE | WS_TABSTOP | WS_BORDER, rectPropList, this, (UINT)-1);
	//no header and no description area
	pTarget_property_control->EnableHeaderCtrl(FALSE,_T("Property"),_T("Value"));
	pTarget_property_control->EnableDescriptionArea(FALSE);
	//set .NET-Look and mark modiefed properties
	pTarget_property_control->SetVSDotNetLook(TRUE);
	pTarget_property_control->MarkModifiedProperties(TRUE);

	//create new item group
	std::auto_ptr<CMFCPropertyGridProperty> apGroup1(new CMFCPropertyGridProperty(RDoc.ParametricModels[choosen_robot]->getName().c_str()));

	//load parameters for the choosen parametrized robot model
	list<string> names;

	RDoc.ParametricModels[choosen_robot]->getParamNames(names);

	for (list<string>::iterator itor=names.begin(); itor!=names.end(); itor++)
	{
		CString str;
		double value;
		str.Format("%s",itor->c_str());
		// also put the default values in the propertyedit-control
		RDoc.ParametricModels[choosen_robot]->getParameter(itor->c_str(),value);
		//add new sub item
		apGroup1->AddSubItem(new CMFCPropertyGridProperty(str, value, ""));
	}

	//finally add the created group
	pTarget_property_control->AddProperty(apGroup1.release());
}

void CRobotPropertiesDlg::BuiltTransformationListCtrl(int entry_number)
{
	//create new item group
	std::auto_ptr<CMFCPropertyGridProperty> apGroup1(new CMFCPropertyGridProperty(RDoc.ParametricModels[entry_number]->getName().c_str()));

	//load parameters for the choosen parametrized robot model
	list<string> names;
	RDoc.ParametricModels[entry_number]->getParamNames(names);

	for (list<string>::iterator itor=names.begin(); itor!=names.end(); itor++)
	{
		CString str;
		double value;
		str.Format("%s",itor->c_str());
		// also put the default values in the propertyedit-control
		RDoc.ParametricModels[entry_number]->getParameter(itor->c_str(),value);
		//add new sub item
		apGroup1->AddSubItem(new CMFCPropertyGridProperty(str, value, ""));
	}

	//finally add the created group
	m_wndPropList3.AddProperty(apGroup1.release());
}

void CRobotPropertiesDlg::ApplyValuesFromPropertyGrid(CMFCPropertyGridCtrl* pPropList)
{

	// find correct model and assign the new property values in this model

	for (int i=0; i<(pPropList->GetPropertyCount()); i++)
	{
		// find suitable parametric model again by string search
		int active_parametric_model=-1;
		for (vector<PCRL::CGeometryGenerator*>::iterator itor=RDoc.ParametricModels.begin(); itor!=RDoc.ParametricModels.end(); itor++)
		{
			active_parametric_model++;
			// find the name in the GeometryGenerator list again (always exists)
			if ((*itor)->getName()==pPropList->GetProperty(i)->GetName()) break;
		}

		//assign the new values in the RDoc object
		for (int j=0; j<(pPropList->GetProperty(i)->GetSubItemsCount()); j++)
		{
			RDoc.ParametricModels[active_parametric_model]->setParameter(pPropList->GetProperty(i)->GetSubItem(j)->GetName(),pPropList->GetProperty(i)->GetSubItem(j)->GetValue().dblVal);
		}

		//apply the choosen parametric model on the platform and frame points
		RDoc.ParametricModels[active_parametric_model]->setGeometry();
	}

}



// CRobotPropertiesDlg-MessageHandler

BOOL CRobotPropertiesDlg::OnInitDialog()
{
	CDialog::OnInitDialog();
	
	//call the init routine
	OnCbnSelchangeMotionpatternCombo();

	return TRUE;  // return TRUE unless you set the focus to a controlE
}



void CRobotPropertiesDlg::OnCbnSelchangeMotionpatternCombo()
{
	//set motionpattern and degrees of freedom
	UpdateData(TRUE);	
	RDoc.setMotionPattern(m_now,getMotionPattern());
	m_dof = RDoc.getDof();
	UpdateData(FALSE);

	//set radiobutton to complete design and call the clicked function
	CheckRadioButton(IDC_COMPLETEDESIGN_RADIO,IDC_SEPERATECONFIGURATION_RADIO,IDC_COMPLETEDESIGN_RADIO);
	OnClickedRadio();
}

void CRobotPropertiesDlg::OnEnChangeNumberofwiresEdit()
{
	//write control content to linked variable
	UpdateData(TRUE);

	//set object number of wires
	RDoc.SetNow(m_now);

	//call the setMotionPattern routine; this is necessary because of some dynamic memory allocations; otherwise the application will crash due to heap corruption
	RDoc.setMotionPattern(m_now,getMotionPattern());

	//set radiobutton to complete design and call the clicked function
	CheckRadioButton(IDC_COMPLETEDESIGN_RADIO,IDC_SEPERATECONFIGURATION_RADIO,IDC_COMPLETEDESIGN_RADIO);
	OnClickedRadio();
}

void CRobotPropertiesDlg::OnCbnSelchangeParametricrobotsCombo()
{
	//built the matching property-control
	BuiltListPropertyDlg(&m_wndPropListLocation,&m_wndPropList,&m_parametricrobots_combobox);
	OnClickedCheckbox();
}

void CRobotPropertiesDlg::OnCbnSelchangeParametricrobots2Combo()
{
	//built the matching property-control and let the current transformation as it is
	BuiltListPropertyDlg(&m_wndPropListLocation2,&m_wndPropList2,&m_parametricrobots2_combobox);
	OnClickedCheckbox();
}


void CRobotPropertiesDlg::OnClickedRadio()
{

	//destroy the former windows of the property list
	if (m_wndPropList) m_wndPropList.DestroyWindow();
	if (m_wndPropList2) m_wndPropList2.DestroyWindow();
	if (m_wndPropList3) m_wndPropList3.DestroyWindow();

	//uncheck the checkboxes
	//m_transformframe_checkbox.SetCheck(BST_UNCHECKED);
	//m_transformplatform_checkbox.SetCheck(BST_UNCHECKED);

	//use combobox as width-indicator
	CRect rect;
	m_parametricrobots2_combobox.GetClientRect(rect);

	//check, which radiobutton has been choosen
	//look in DoDataExchange() in this CRobotPropertiesDlg class, 
	//to understand why GetCheckedRadioButton() isn't used;
	int checked_radiobutton_id = -1;
	if (m_completedesign_radio.GetCheck()==BST_CHECKED) checked_radiobutton_id = IDC_COMPLETEDESIGN_RADIO;
	else checked_radiobutton_id = IDC_SEPERATECONFIGURATION_RADIO;

	switch (checked_radiobutton_id)
	{
		case IDC_COMPLETEDESIGN_RADIO: //1096
			//set visibility and size of some controls due to the choosen mode
			m_parametricrobots2_combobox.ShowWindow(FALSE); 
			m_wndPropListLocation2.ShowWindow(FALSE);
			m_wndPropListLocation.SetWindowPos(&wndTop,0,0,(rect.right-rect.left),200,SWP_NOMOVE);
			//call function for filling the combobox
			ListSuitableRobotModels(&m_parametricrobots_combobox,true,true,false);
			//call the combobox change routine
			OnCbnSelchangeParametricrobotsCombo();
			break;

		case IDC_SEPERATECONFIGURATION_RADIO: //1160
			//set visibility and size of some controls due to the choosen mode	
			m_parametricrobots2_combobox.ShowWindow(TRUE);
			m_wndPropListLocation2.ShowWindow(TRUE);
			m_wndPropListLocation.SetWindowPos(&wndTop,0,0,(rect.right-rect.left),85,SWP_NOMOVE);
			//call functions for filling the comboboxes
			ListSuitableRobotModels(&m_parametricrobots_combobox,true,false,false);
			ListSuitableRobotModels(&m_parametricrobots2_combobox,false,true,false);
			//call the comboboxes changes routines
			OnCbnSelchangeParametricrobotsCombo();
			OnCbnSelchangeParametricrobots2Combo();
			break;

		default:
			break;
	}

	//repaint window to avoid "sticky" lines from the image controls
	RedrawWindow();
}

void CRobotPropertiesDlg::OnClickedCheckbox()
{

	//create CMFCPropertyGrid and its properties
	CRect rectPropList;

	//get position from the image object
	m_wndPropListLocation3.GetClientRect(&rectPropList);
	m_wndPropListLocation3.MapWindowPoints(this, &rectPropList);

	//destroy the former window if present
	if (m_wndPropList3) m_wndPropList3.DestroyWindow();

	//create a new control with special default properties
	m_wndPropList3.Create(WS_CHILD | WS_VISIBLE | WS_TABSTOP | WS_BORDER, rectPropList, this, (UINT)-1);
	//no header and no description area
	m_wndPropList3.EnableHeaderCtrl(FALSE,_T("Property"),_T("Value"));
	m_wndPropList3.EnableDescriptionArea(FALSE);
	//set .NET-Look and mark modiefed properties
	m_wndPropList3.SetVSDotNetLook(TRUE);
	m_wndPropList3.MarkModifiedProperties(TRUE);


	//find the TransformatorGenerators
	int frame_transformator_number=0,platform_transformator_number=0;
	int counter = -1;

	for (vector<PCRL::CGeometryGenerator*>::iterator itor=RDoc.ParametricModels.begin(); itor!=RDoc.ParametricModels.end(); itor++)
	{
		//use counter variable ... is there a better solution, for example using the itor ?
		counter++;
		
		//if isn't transformator doesn't match -> next turn
		if (((*itor)->isTransformator() == FALSE)) continue;
		//matches the transformators to frame- and platform transformator number variables
		if ((*itor)->effectsFrame() == TRUE) frame_transformator_number=counter;
		if ((*itor)->effectsPlatform() == TRUE) platform_transformator_number=counter;
	}


	//check, which checkboxes are checked and add the suitable properties to the grid control
	if (m_transformframe_checkbox.GetCheck()==BST_CHECKED)
	{
		BuiltTransformationListCtrl(frame_transformator_number);
	}

	if (m_transformplatform_checkbox.GetCheck()==BST_CHECKED)
	{
		BuiltTransformationListCtrl(platform_transformator_number);
	}

}

void CRobotPropertiesDlg::OnBnClickedOk()
{
	//save all robot data and following transformations in the RDoc object
	UpdateData(TRUE);	

	//apply values from first property list , if exists
	if (m_wndPropList) ApplyValuesFromPropertyGrid(&m_wndPropList);

	//apply values from second property list , if exists
	if (m_wndPropList2) ApplyValuesFromPropertyGrid(&m_wndPropList2);

	//apply values from third property list (which represent additional transformations), if exists
	if (m_wndPropList3) ApplyValuesFromPropertyGrid(&m_wndPropList3);
	
	// save the name strings
	RDoc.name = m_Name;
	RDoc.author = m_Author;
	RDoc.id = m_ID;
	RDoc.desc = m_Desc;

	//call underlayed OnOK routine
	CDialog::OnOK();
}
