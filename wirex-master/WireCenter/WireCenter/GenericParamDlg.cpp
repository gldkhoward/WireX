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

// GenericParamDlg.cpp: Implementierungsdatei
//

#include "stdafx.h"
#include "WireCenter.h"
#include "GenericParamDlg.h"


// CGenericParamDlg-Dialogfeld

IMPLEMENT_DYNAMIC(CGenericParamDlg, CDialog)

CGenericParamDlg::CGenericParamDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CGenericParamDlg::IDD, pParent)
{
}

CGenericParamDlg::~CGenericParamDlg()
{
}

void CGenericParamDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_GENERIC_PARAMETER_LIST, m_ListCtrl);
}

BEGIN_MESSAGE_MAP(CGenericParamDlg, CDialog)
	ON_BN_CLICKED(IDC_APPLY_VALUES_BUTTON, &CGenericParamDlg::OnBnClickedApplyValuesButton)
	ON_BN_CLICKED(IDC_RELOAD_VALUES_BUTTON, &CGenericParamDlg::OnBnClickedReloadValuesButton)
	ON_NOTIFY(LVN_ENDLABELEDIT, IDC_GENERIC_PARAMETER_LIST, &CGenericParamDlg::OnLvnEndlabeleditGenericParameterList)
END_MESSAGE_MAP()

//! add a new double variable to the buffering list
void CGenericParamDlg::addItem(const string& name, double& value)
{
	io_map[name].type = eDouble;
	io_map[name].dPtr = &value;
	io_map[name].dValue = value;
}

//! add a new int variable to the buffering list
void CGenericParamDlg::addItem(const string& name, int& value)
{
	io_map[name].type = eInt;
	io_map[name].iPtr = &value;
	io_map[name].iValue = value;
}

//! add a new bool variable to the buffering list
void CGenericParamDlg::addItem(const string& name, bool& value)
{
	io_map[name].type = eBool;
	io_map[name].bPtr = &value;
	io_map[name].bValue = value;
}

//! add a new string variable to the buffering list
void CGenericParamDlg::addItem(const string& name, string& value)
{
	io_map[name].type = eString;
	io_map[name].sPtr = &value;
	io_map[name].sValue = value;
}

//! add a new string variable to the buffering list
void CGenericParamDlg::addItem(const string& name, char& value)
{
	io_map[name].type = eChar;
	io_map[name].cPtr = &value;
	io_map[name].cValue = value;
}

//! copy the values from the original memory to the temporary memory of the mapped values
//! e.g. load the temp values
void CGenericParamDlg::readData()
{
	map_iterator itor;
	for (itor = io_map.begin(); itor!=io_map.end(); itor++)
	{
		itor->second.readData();
	}
}

//! copy the temporary values from the buffered memory to the origional memory
//! e.g. save the temp values
void CGenericParamDlg::writeData()
{
	if (bCBactivated) // for bool values
	{
		int i = 0;
		for (map_iterator itor = io_map.begin(); itor!=io_map.end(); itor++,i++)
		{
			itor->second.bValue = (m_ListCtrl.GetCheck(i) != FALSE); // read the state of the checkbox i
			itor->second.writeData();
		}
	}
	else // for other variable types
	{
		for (map_iterator itor = io_map.begin(); itor!=io_map.end(); itor++)
			itor->second.writeData();
	}
}

// CGenericParamDlg-Meldungshandler

//! update already existing entries in the control
//! the entries are identified by their respective handle
void CGenericParamDlg::updateTreeView()
{	
	for (map_iterator itor = io_map.begin(); itor!=io_map.end(); itor++)
		m_ListCtrl.SetItemText(itor->second.pos,2,itor->second.toString());
}

BOOL CGenericParamDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// read the data from the source into the buffer
	readData();

	// setup the header
	RECT rect;
	m_ListCtrl.GetWindowRect(&rect);

	int i = 0;
	
	// check, if there are only bool variables
	bCBactivated = true;
	for (map_iterator itor = io_map.begin(); itor != io_map.end(); itor++,i++)
	{
		itor->second.pos = i;
		if (itor->second.type != eBool)
		{
			bCBactivated = false;
			break;
		}
	}

	if (bCBactivated) // only bool variables --> use checkboxes
		m_ListCtrl.SetExtendedStyle(m_ListCtrl.GetStyle()|LVS_EX_CHECKBOXES);

	// head of the list
	m_ListCtrl.InsertColumn(1, "Value",0,75);
	m_ListCtrl.InsertColumn(2, "Description",0,rect.right-rect.left-75-4);

	// loop through all entries of the map and add an item to the list control
	i = 0;
	for (map_iterator itor = io_map.begin(); itor != io_map.end(); itor++,i++)
	{
		itor->second.pos = i;
		
		if (bCBactivated) // only bool variables --> use checkboxes
		{
			m_ListCtrl.InsertItem(i,""); // create new row
			m_ListCtrl.SetCheck(i,itor->second.bValue); // set the checkbox
		}
		else 
		{
			m_ListCtrl.InsertItem(i,itor->second.toString()); // create new row and write the value in the first column
		}
		m_ListCtrl.SetItemText(i,1,itor->first.c_str());  // write the attribute name in the second column
	}

	return TRUE;
}

void CGenericParamDlg::OnBnClickedApplyValuesButton()
{
	writeData();	
}

void CGenericParamDlg::OnBnClickedReloadValuesButton()
{
	readData();
	updateTreeView();
}

void CGenericParamDlg::OnLvnEndlabeleditGenericParameterList(NMHDR *pNMHDR, LRESULT *pResult)
{
	NMLVDISPINFO *pDispInfo = reinterpret_cast<NMLVDISPINFO*>(pNMHDR);
	// TODO: Fügen Sie hier Ihren Kontrollbehandlungscode für die Benachrichtigung ein.
	// loop through the io_map to find the associated entry
	map_iterator itor;
	for (itor = io_map.begin(); itor != io_map.end(); itor++)
	{
		// check if the is the correct item; parse and interprete the content of the edit control
		if (itor->second.pos == pDispInfo->item.iItem)
			itor->second.fromString(pDispInfo->item.pszText);
	}
	if (pDispInfo->item.pszText)
		*pResult = 1;
	else	
		*pResult = 0;
}

void CGenericParamDlg::OnOK()
{
	// write the temp values into the original variables, if ok is pressed
	writeData();

	CDialog::OnOK();
}


//################################################################################
//################################################################################
//					CEnhancedDialog dialog
//################################################################################
//################################################################################

IMPLEMENT_DYNAMIC(CEnhancedDialog, CDialog)

CEnhancedDialog::CEnhancedDialog(CString title, int _element_width, bool _activate_resetbutton, int _element_margin, int _element_height, int _global_margin): CDialog(CEnhancedDialog::IDD)
{
	#ifndef _WIN32_WCE
		EnableActiveAccessibility();
	#endif

	// initiate the variables
	m_current_ID = 10; // start at 10 , because e.g. 1 and 2 are used for the IDOK and IDCANCEL buttons
	bWouldIntersectGroupBox = false; // because if nothing is created, there is no groupbox
	m_global_maximum_height = 0; // don't need an explanation

	// copy the variables
	element_margin = _element_margin; 
	element_height = _element_height;
	element_width = _element_width;
	m_global_margin = _global_margin;

	m_global_height = m_global_margin;	
	m_global_width  = m_global_margin;

	main_title = title;

	activate_resetbutton = _activate_resetbutton;

	// alter the standard font
	LOGFONT lf;
	afxGlobalData.fontRegular.GetLogFont(&lf);
	NONCLIENTMETRICS info;
	info.cbSize = sizeof(info);
	afxGlobalData.GetNonClientMetrics(info);
	lf.lfHeight = info.lfMenuFont.lfHeight;
	lf.lfWeight = info.lfMenuFont.lfWeight;
	lf.lfItalic = info.lfMenuFont.lfItalic;
	
	// set the font height (set this here hard coded to 9, but where should i get the right value?)
	font_height = 9;
}

CEnhancedDialog::~CEnhancedDialog()
{
	// loo trough all elements and ...
	for (itor=element_storage.begin(); itor!=element_storage.end(); itor++)
	{
		// ...delete the element and the corresponding static (if not NULL)
		delete itor->m_pElement;
		delete itor->m_pStatic;
	}

	// and clear the element_storage	itself
	element_storage.clear();
}

BEGIN_MESSAGE_MAP(CEnhancedDialog, CDialog)
	ON_BN_CLICKED(IDOK, &CEnhancedDialog::OnBnClickedOk)
	ON_BN_CLICKED(IDCANCEL, &CEnhancedDialog::OnBnClickedCancel)
	ON_BN_CLICKED(IDRESET, &CEnhancedDialog::OnBnClickedReset)
END_MESSAGE_MAP()

//#############################################
// create-routines for all controls
//#############################################

int CEnhancedDialog::CreateEdit(int* variable_ref , CString title)
{
	// create pointer and allocate memory
	CEdit* pCreatedEdit;
	CStatic* pCreatedEditStatic;
	pCreatedEdit = new CEdit;
	pCreatedEditStatic = new CStatic;

	// create a containter to store the elements and hook all pointer in the container
	TElementType new_edit(eETEditElement, pCreatedEdit, eDTInteger, variable_ref, pCreatedEditStatic, title);
	element_storage.push_back(new_edit);

	// return a value
	return 0;
}

int CEnhancedDialog::CreateEdit(double* variable_ref , CString title)
{
	// create pointer and allocate memory
	CEdit* pCreatedEdit;
	CStatic* pCreatedEditStatic;
	pCreatedEdit = new CEdit;
	pCreatedEditStatic = new CStatic;

	// create a containter to store the elements and hook all pointer in the container
	TElementType new_edit(eETEditElement, pCreatedEdit, eDTDouble, variable_ref, pCreatedEditStatic, title);
	element_storage.push_back(new_edit);

	// return a value
	return 0;
}

int CEnhancedDialog::CreateEdit(CString* variable_ref , CString title)
{
	// create pointer and allocate memory
	CEdit* pCreatedEdit;
	CStatic* pCreatedEditStatic;
	pCreatedEdit = new CEdit;
	pCreatedEditStatic = new CStatic;

	// create a containter to store the elements and hook all pointer in the container
	TElementType new_edit(eETEditElement, pCreatedEdit, eDTCString, variable_ref, pCreatedEditStatic, title);
	element_storage.push_back(new_edit);

	// return a value
	return 0;
}

int CEnhancedDialog::CreateEdit(string* variable_ref, CString title)
{
	TElementType new_edit(eETEditElement, new CEdit, eDTString, variable_ref, new CStatic, title);
	element_storage.push_back(new_edit);
	return 0;
}

int CEnhancedDialog::CreateCheckbox(bool* variable_ref , CString title)
{
	// create pointer and allocate memory
	CButton* pCreatedCheckBox;
	pCreatedCheckBox = new CButton;

	// create a containter to store the elements and hook all pointer in the container
	TElementType new_checkbox(eETCheckboxElement, pCreatedCheckBox, eDTBoolean, variable_ref, NULL, title);
	element_storage.push_back(new_checkbox);

	// return a value
	return 0;
}

int CEnhancedDialog::CreateLabel(CString title)
{
	// create pointer and allocate memory
	CStatic* pCreatedStaticLabel;
	pCreatedStaticLabel = new CStatic;

	// create a containter to store the elements and hook all pointer in the container
	TElementType new_label(eETLabelElement, pCreatedStaticLabel, eDTUnknown, NULL, NULL, title);
	element_storage.push_back(new_label);

	// return a value
	return 0;
}

int CEnhancedDialog::CreateComboBox(int* selected_entry, CStringArray* input_strings, CString title)
{
	// create pointer and allocate memory
	CComboBox* pCreatedComboBox;
	CStatic* pCreatedComboboxStatic;
	pCreatedComboBox = new CComboBox;
	pCreatedComboboxStatic = new CStatic;

	// create an NON-editable combobox element
	// create a containter to store the elements and hook all pointer in the container
	TElementType new_combobox(eETNonEditableComboboxElement, pCreatedComboBox, eDTInteger, selected_entry, pCreatedComboboxStatic, title, input_strings);
	element_storage.push_back(new_combobox);

	// return a value
	return 0;
}

int CEnhancedDialog::CreateComboBox(CString* selected_entry, CStringArray* input_strings, CString title)
{
	// create pointer and allocate memory
	CComboBox* pCreatedComboBox;
	CStatic* pCreatedComboboxStatic;
	pCreatedComboBox = new CComboBox;
	pCreatedComboboxStatic = new CStatic;

	// create an editable combobox element
	// create a containter to store the elements and hook all pointer in the container
	TElementType new_combobox(eETEditableComboboxElement, pCreatedComboBox, eDTCString, selected_entry, pCreatedComboboxStatic, title, input_strings);
	element_storage.push_back(new_combobox);

	// return a value
	return 0;
}

int CEnhancedDialog::StartGroupBox(CString title)
{
	// create pointer and allocate memory
	CButton* pCreatedGroupBox;
	pCreatedGroupBox = new CButton;

	// create a containter to store the elements and hook all pointer in the container
	TElementType new_groupbox(eETGroupboxElementStart, pCreatedGroupBox, eDTUnknown, NULL, NULL, title);
	element_storage.push_back(new_groupbox);

	// return a value
	return 0;
}

int CEnhancedDialog::EndGroupBox()
{

	// create a containter to store the elements and hook all pointer in the container
	// just use a dummy element here to indicate, that this is the end of the current groupbox
	TElementType new_groupbox_end(eETGroupboxElementEnd, NULL, eDTUnknown, NULL, NULL, "");
	element_storage.push_back(new_groupbox_end);

	// return a value
	return 0;

}

int CEnhancedDialog::NextDialogColumn()
{

	// create a containter to store the elements and hook all pointer in the container
	// just use a dummy element here to indicate, that this is the time for an linewrap
	TElementType new_column(eETNextDialogColumn, NULL, eDTUnknown, NULL, NULL, "");
	element_storage.push_back(new_column);

	// return a value
	return 0;

}

//#############################################
// building and saveing routines for all controls
//#############################################

int CEnhancedDialog::BuildElements(bool first_run)
{
	// loop throug all elements and build them
	for (itor=element_storage.begin(); itor!=element_storage.end(); itor++)
	{
		// checks if the length of the dialogbox exceeds the height of the screen - 150, then "make a new column", but only if no groupbox is currently active
		if (m_global_height>(GetSystemMetrics(SM_CYSCREEN)-150)&&(!bWouldIntersectGroupBox))
		{
			m_global_height = m_global_margin;
			m_global_width += element_width+m_global_margin;
		}
		
	
		// if the element is an edit-control
		if (itor->m_ElementType == eETEditElement)
		{
			
			// cast the data to an CEdit and CStatic pointer
			CEdit* pEditElement = (CEdit*)(itor->m_pElement);
			CStatic* pStaticElement = (CStatic*)(itor->m_pStatic);
			
			// if this is the first run , create the controls!
			if (first_run)
			{
				// create the static at the momentary y-position
				pStaticElement->Create(itor->m_StaticText,SS_LEFT,CRect(m_global_width,m_global_height,m_global_width+element_width-element_margin,m_global_height+element_height),this,SetID());
				m_global_height+=element_height;

				// create the edit at the momentary y-position (WS_TABSTOP to access the element by TAB-key)
				pEditElement->Create(WS_BORDER | ES_AUTOHSCROLL | WS_TABSTOP,CRect(m_global_width,m_global_height,m_global_width+element_width-element_margin,m_global_height+element_height),this,SetID());
				m_global_height+=element_height+element_margin;
			}

			// retrive the actual variable value, cast it depending on its type and put it in the edit
			CString s;
			if (itor->m_DataType==eDTInteger) 
				s.Format("%i", (*(int*)(itor->m_pData)));
			else 
				if (itor->m_DataType==eDTDouble) s.Format("%f", (*(double*)(itor->m_pData)));
			else 
				if (itor->m_DataType==eDTCString) s.Format("%s", (*(CString*)(itor->m_pData)));
			else 
				if (itor->m_DataType==eDTString) s.Format("%s", ((string*)(itor->m_pData))->c_str());
			pEditElement->SetWindowTextA(s);
			
			// alter the font
			pEditElement->SetFont(&afxGlobalData.fontRegular);
			pStaticElement->SetFont(&afxGlobalData.fontRegular);

			// show the windows
			pEditElement->ShowWindow(SW_SHOW);
			pStaticElement->ShowWindow(SW_SHOW);
		}
		
		// if the element is an checkbox-control
		if (itor->m_ElementType==eETCheckboxElement)
		{
			
			// cast the data to an CButton pointer
			CButton* pCheckboxElement = (CButton*)(itor->m_pElement);
			
			// if this is the first run , create the controls!
			if (first_run)
			{
				// create the static at the momentary y-position (WS_TABSTOP to access the element by TAB-key)
				pCheckboxElement->Create(itor->m_StaticText, BS_AUTOCHECKBOX | BS_CHECKBOX | WS_TABSTOP,
					CRect(m_global_width,m_global_height,m_global_width+element_width-element_margin,m_global_height+element_height),this,SetID());
				m_global_height+=element_height+element_margin;
			}

			// retrive the actual variable value, cast it depending on its type and put it in the edit
			pCheckboxElement->SetCheck((*(bool*)(itor->m_pData)));
			
			// alter the font
			pCheckboxElement->SetFont(&afxGlobalData.fontRegular);

			// show the windows
			pCheckboxElement->ShowWindow(SW_SHOW);
		}

		// if the element is an "label"-control
		if (itor->m_ElementType==eETLabelElement)
		{
			
			// cast the data to an CButton pointer
			CStatic* pLabelElement = (CStatic*)(itor->m_pElement);
			
			// if this is the first run , create the controls!
			if (first_run)
			{
				// save the current value
				int old_element_height = element_height;

				// adapt the element height to the text length
				int text_length = itor->m_StaticText.GetLength();

				while (text_length*font_height > element_width) 
				{
					element_height += font_height;
					text_length -= element_width/font_height;
				}

				// create the static at the momentary y-position and set the element height temporary appropriate to the text amount
				pLabelElement->Create(itor->m_StaticText,0,CRect(m_global_width,m_global_height,m_global_width+element_width-element_margin,
					m_global_height+element_height),this,SetID());
				m_global_height+=element_height+element_margin;

				// reset the old value
				element_height = old_element_height;
			}

			// alter the font
			pLabelElement->SetFont(&afxGlobalData.fontRegular);

			// show the windows
			pLabelElement->ShowWindow(SW_SHOW);
		}

		// if the element is an non editable combobox-control
		if ((itor->m_ElementType == eETNonEditableComboboxElement) || (itor->m_ElementType == eETEditableComboboxElement))
		{
			
			// cast the data to an CCombobox and CStatic pointer and to the additional input data (as CStringArray)
			CComboBox* pComboBoxElement = (CComboBox*)(itor->m_pElement);
			CStatic* pStaticElement = (CStatic*)(itor->m_pStatic);
			CStringArray* pStringArray = (CStringArray*)(itor->m_pAdditional_inputdata);

			// if this is the first run , create the controls!
			if (first_run)
			{
				// create the static at the momentary y-position
				pStaticElement->Create(itor->m_StaticText,SS_LEFT,CRect(m_global_width,m_global_height,m_global_width+element_width-element_margin,m_global_height+element_height),this,SetID());
				m_global_height+=element_height;

				// decide on the element type, if the combox is editable or not
				if (itor->m_ElementType==eETNonEditableComboboxElement)
				{
					// create the combobox at the momentary y-position (WS_TABSTOP to access the element by TAB-key)
					pComboBoxElement->Create(WS_CHILD | WS_VISIBLE | WS_VSCROLL | WS_TABSTOP | CBS_DROPDOWNLIST,
						CRect(m_global_width,m_global_height,m_global_width+element_width-element_margin,m_global_height+element_height),this,SetID());
					m_global_height+=element_height+element_margin;
				}
				else
				{
					pComboBoxElement->Create(WS_CHILD | WS_VISIBLE | WS_VSCROLL | WS_TABSTOP | CBS_DROPDOWN,
						CRect(m_global_width,m_global_height,m_global_width+element_width-element_margin,m_global_height+element_height),this,SetID());
					m_global_height+=element_height+element_margin;
				}

				// add the strings stored in the m_pAdditional_inputdata field to the combobox
				for (int i=0; i<(pStringArray->GetSize()); i++)
				{
					pComboBoxElement->AddString(pStringArray->GetAt(i));
				}
			}
			
			// select the current selected (by value) ( even invalid numbers are permitted but have no effect )
			pComboBoxElement->SetCurSel((*(int*)(itor->m_pData)));

			// alter the font
			pComboBoxElement->SetFont(&afxGlobalData.fontRegular);
			pStaticElement->SetFont(&afxGlobalData.fontRegular);

			// show the windows
			pComboBoxElement->ShowWindow(SW_SHOW);
			pStaticElement->ShowWindow(SW_SHOW);
		}

		// if the element is a "start-groupbox"-control
		if (itor->m_ElementType == eETGroupboxElementStart && first_run)
		{
			
			// signal, that a groupbox is active
			bWouldIntersectGroupBox=true;

			// decrease the element width due to indenting
			element_width -= m_global_margin;

			// cast the data to an CButton pointer
			CButton* pGroupBoxElement = (CButton*)(itor->m_pElement);

			// store the current element in a groupbox list
			pGroupBoxList.push_back(pGroupBoxElement);

			// if this is the first run , create the controls!
			if (first_run)
			{
				// create the groupbox at the momentary y-position
				pGroupBoxElement->Create(itor->m_StaticText, WS_VISIBLE | WS_CHILD | BS_GROUPBOX, CRect(m_global_width, m_global_height,0,0), this, SetID());
				m_global_height+=element_height;
			}

			// alter the font
			pGroupBoxElement->SetFont(&afxGlobalData.fontRegular);

			// increase the indent for the elements in Groupbox 
			m_global_width+=m_global_margin;
		}

		// if the element is an "end-groupbox"-control
		if(itor->m_ElementType==eETGroupboxElementEnd&&first_run)
		{
			
			// check if there is someone trying to closed a non-exisiting GroupBox (because someone forgot the "StartGroupBox" function!)
			if (pGroupBoxList.empty()) 
			{
				AfxMessageBox("Internal Error: Found an redundant 'EndGroupBox' in EnhancedDialog!");
				continue;
			}
			
			// cast the data to an CButton pointer and pop the last element
			CButton* pGroupBoxElement = pGroupBoxList.back();

			// get the starting position mapped in dialog coordinates
			CRect rect, rect2;
			pGroupBoxElement->GetWindowRect(&rect);
			this->GetWindowRect(&rect2);
			rect.top = rect.top - rect2.top;

			// set the actual end position
			m_global_height+=element_height+element_margin;

			// set the new size and show the window
			pGroupBoxElement->SetWindowPos(NULL,0,0,element_width+m_global_margin,m_global_height-rect.top, SWP_NOZORDER | SWP_NOMOVE);
			pGroupBoxElement->ShowWindow(SW_SHOW);

			// correct the new height again
			m_global_height -= element_height-element_margin;
			
			// decrease the indent and increase the element width again
			m_global_width -= m_global_margin;
			element_width += m_global_margin;

			// remove the element
			pGroupBoxList.pop_back();

			// signal, that the groupbox has finished, if there is no more groupbox
			if (pGroupBoxList.empty()) 
				bWouldIntersectGroupBox=false;
		}


		if (itor->m_ElementType==eETNextDialogColumn&&first_run)
		{
			// increase the current width position
			m_global_height = m_global_margin;
			m_global_width += element_width+m_global_margin;
		}


		//############################
		//INSERT NEW ELEMENTS HERE
		//############################

		// save the maximum height
		if (m_global_height>m_global_maximum_height) m_global_maximum_height = m_global_height;

	}
	
	
	// check if there are some not closed GroupBoxes (because someone forgot the "EndGroupBox" function!)
	if (!pGroupBoxList.empty()) 
	{
		AfxMessageBox("Internal Error: A Groupbox in EnhancedDialog was not closed!");
	}

	return 0;
}

int CEnhancedDialog::SaveElements()
{
	// transfer the values from the controls back to their references, so loop through all elements
	for (itor=element_storage.begin(); itor!=element_storage.end(); itor++)
	{
		// if the element is an edit-control
		if(itor->m_ElementType==eETEditElement)
		{
			// cast the data to an CEdit and CStatic pointer
			CEdit* pEditElement = (CEdit*)(itor->m_pElement);
			
			// get the string
			CString str;
			pEditElement->GetWindowTextA(str);
			
			// type conversion
			if (itor->m_DataType==eDTInteger) 
				(*(int*)(itor->m_pData)) = atoi(str);
			else 
				if (itor->m_DataType==eDTDouble) (*(double*)(itor->m_pData)) = atof(str);
			else 
				if (itor->m_DataType==eDTCString) (*(CString*)(itor->m_pData)) = str;
			else 
				if (itor->m_DataType==eDTString) (*(string*)(itor->m_pData)) = str.GetBuffer();
		}

		// if the element is an checkbox-control
		if(itor->m_ElementType==eETCheckboxElement)
		{
			// cast the data to an CEdit and CStatic pointer
			CButton* pCheckBoxElement = (CButton*)(itor->m_pElement);

			// type conversion
			if (itor->m_DataType==eDTBoolean) (*(bool*)(itor->m_pData)) = pCheckBoxElement->GetCheck() ? true : false;
		}

		// if the element is an non-editable combobox-control
		if (itor->m_ElementType==eETNonEditableComboboxElement)
		{
			// cast the data to an CComboBox
			CComboBox* pComboBoxElement = (CComboBox*)(itor->m_pElement);

			// type conversion
			if (itor->m_DataType==eDTInteger) (*(int*)(itor->m_pData)) = pComboBoxElement->GetCurSel();
		}

		// if the element is an editable combobox-control
		if (itor->m_ElementType==eETEditableComboboxElement)
		{
			// cast the data to an CComboBox
			CComboBox* pComboBoxElement = (CComboBox*)(itor->m_pElement);

			// type conversion
			if (itor->m_DataType==eDTCString) pComboBoxElement->GetWindowTextA((*(CString*)(itor->m_pData)));
		}
	}

	return 0;
}

//#############################################
// helping routines for the dialog
//#############################################


BOOL CEnhancedDialog::OnInitDialog()
{
	CDialog::OnInitDialog();

	// build all elements
	BuildElements(true);

	// resize and center the dialog window
	this->SetWindowTextA(main_title);
	this->SetWindowPos(NULL,0,0,m_global_width+element_width+2*m_global_margin,m_global_maximum_height+4*m_global_margin,SWP_HIDEWINDOW);
	this->CenterWindow();
	
	// move the OK, Cancel and Reset buttons depending on the window size and depending if the reset-button should be shown
	CRect rect, button_rect;
	GetClientRect(&rect);
	if(activate_resetbutton)
	{
		GetDlgItem(IDRESET)->GetClientRect(&button_rect);
		GetDlgItem(IDRESET)->SetWindowPos(&wndBottom,rect.Width()-(button_rect.Width())-m_global_margin,m_global_maximum_height,0,0,SWP_NOZORDER|SWP_NOSIZE);
		GetDlgItem(IDCANCEL)->GetClientRect(&button_rect);
		GetDlgItem(IDCANCEL)->SetWindowPos(&wndBottom,rect.Width()-2*(button_rect.Width())-m_global_margin,m_global_maximum_height,0,0,SWP_NOZORDER|SWP_NOSIZE);
		GetDlgItem(IDOK)->GetClientRect(&button_rect);
		GetDlgItem(IDOK)->SetWindowPos(&wndBottom,rect.Width()-3*(button_rect.Width())-m_global_margin,m_global_maximum_height,0,0,SWP_NOZORDER|SWP_NOSIZE);
	}
	else
	{
		GetDlgItem(IDCANCEL)->GetClientRect(&button_rect);
		GetDlgItem(IDCANCEL)->SetWindowPos(&wndBottom,rect.Width()-(button_rect.Width())-m_global_margin,m_global_maximum_height,0,0,SWP_NOZORDER|SWP_NOSIZE);
		GetDlgItem(IDOK)->GetClientRect(&button_rect);
		GetDlgItem(IDOK)->SetWindowPos(&wndBottom,rect.Width()-2*(button_rect.Width())-m_global_margin,m_global_maximum_height,0,0,SWP_NOZORDER|SWP_NOSIZE);
	}
	
	// show the window
	this->ShowWindow(SW_SHOW);

	return TRUE;  
}


void CEnhancedDialog::OnBnClickedOk()
{
	CDialog::OnOK();

	// save the values back
	SaveElements();
}


void CEnhancedDialog::OnBnClickedCancel()
{
	// do nothing and end the dialog
	CDialog::OnCancel();
}

void CEnhancedDialog::OnBnClickedReset()
{
	// reset the values in the field, therefore simply rebuild all elements
	BuildElements(false);
}


//*****************************************************************************
//					CMatrixDialog
//*****************************************************************************

IMPLEMENT_DYNAMIC(CMatrixDialog, CDialog)

BEGIN_MESSAGE_MAP(CMatrixDialog, CDialog)
	ON_BN_CLICKED(IDOK, &CMatrixDialog::OnBnClickedOk)
	ON_BN_CLICKED(IDD_MENU_BUTTON, &CMatrixDialog::OnMenuButton)
	ON_WM_RBUTTONUP()
	ON_COMMAND(IDD_MV_COPY2CLIPBOARD, &CMatrixDialog::OnCopyToClipboard)
	ON_COMMAND(IDD_MV_MAPLE2CLIPBOARD, &CMatrixDialog::OnCopyMapleClipboard)
	ON_COMMAND(IDD_MV_MATLAB2CLIPBOARD, &CMatrixDialog::OnCopyMatlabClipboard)
	ON_COMMAND(IDD_MV_CSV2CLIPBOARD, &CMatrixDialog::OnCopyCsvClipboard)
	ON_COMMAND(IDD_MV_LATEX2CLIPBOARD, &CMatrixDialog::OnCopyLatexClipboard)
	ON_COMMAND(IDD_MV_HTML2CLIPBOARD, &CMatrixDialog::OnCopyHtmlClipboard)
	ON_COMMAND(IDD_MV_PYTHON2CLIPBOARD, &CMatrixDialog::OnCopyPythonClipboard)
	ON_COMMAND(IDD_MV_TRANSPOSE_MATRIX, &CMatrixDialog::OnTransposeMatrix)
	ON_COMMAND(IDD_MV_INVERSE_MATRIX, &CMatrixDialog::OnInverseMatrix)
//	ON_WM_SIZE()
END_MESSAGE_MAP()


CMatrixDialog::CMatrixDialog(CString title="Default-Dialog", MatrixXd mat=MatrixXd::Identity(1,1)): CDialog(CEnhancedDialog::IDD)
{
	#ifndef _WIN32_WCE
		EnableActiveAccessibility();
	#endif

	// initiate the variables
	mainTitle = title;
	pLC = 0;
	thisMatrix = mat;

	// alter the standard font
	LOGFONT lf;
	afxGlobalData.fontRegular.GetLogFont(&lf);
	NONCLIENTMETRICS info;
	info.cbSize = sizeof(info);
	afxGlobalData.GetNonClientMetrics(info);
	lf.lfHeight = info.lfMenuFont.lfHeight;
	lf.lfWeight = info.lfMenuFont.lfWeight;
	lf.lfItalic = info.lfMenuFont.lfItalic;
	
	// set the font height (set this here hard coded to 9, but where should i get the right value?)
	font_height = 9;
	// the following parameters control the layout of the dialog box
	border = 30;
}

CMatrixDialog::~CMatrixDialog()
{
}

/*! This is a helper functions that empties the clipboard and copies the
 *  string in text to the clipboard.
 *  \param text The string to be copied to the clipboard
 *  \return true, if successful, otherwise false
 *  The baseline of theis implementation is copied from 
 *  http://stackoverflow.com/questions/2253476/does-mfc-provide-a-quick-way-to-throw-text-on-the-clipboard
 */
bool CMatrixDialog::CopyTextToCpliboard(std::string& test)
{
	CString szData,d2;
	szData = test.c_str();
	d2 = szData;

    HGLOBAL h;
    LPTSTR arr;

    h = GlobalAlloc(GMEM_MOVEABLE, szData.GetLength()+1);
    arr = (LPTSTR)GlobalLock(h);
    strcpy_s((char*)arr, szData.GetLength()+1, szData.GetBuffer());
    szData.ReleaseBuffer();
    GlobalUnlock(h);

    ::OpenClipboard(NULL);
    EmptyClipboard();
    SetClipboardData(CF_TEXT, h); 
    CloseClipboard();

	// for output in the Edit, we have to convert "\n" to "\n\r"
	d2.Replace("\n","\r\n");
	m_wndTabs.GetDlgItem(IDD_MATRIX_EDIT)->SetWindowTextA(d2);

	return true;
}

/* clear the list view, copy the data from the matrix into the control 
 * and update the respective tabs for statistics and algebraic properties.
 */ 
void CMatrixDialog::updateMatrixContent()
{
	// remove all items before creating new ones
	// we may want to improve the behavior by overwriting items if they exist 
	pLC->DeleteAllItems();
	while( pLC->DeleteColumn( 0 ) );

	// paranoia testing; if matrix empty is empty, we do nothing
	if (thisMatrix.cols() <= 0 || thisMatrix.rows() <= 0)
		return;
	
	// create columns
	for (int j=0; j<thisMatrix.cols()+1; j++)
	{
		// add the custom column lables if provided
		CString str;
		str.Format("Col %d", j);
		pLC->InsertColumn(j, str,LVCFMT_RIGHT,75);
	}

	// now, we fill the matrix with content. copy theMatrix into the ListCtrl
	for (int i=0; i<thisMatrix.rows(); i++)
	{
		CString str;
		str.Format("Row %d:",i+1);		// add the custom row where if provided
		pLC->InsertItem(i,str);	
		for (int j=0; j<thisMatrix.cols(); j++)
		{
			CString str;
			str.Format("%10.6f",thisMatrix(i,j));
			pLC->SetItemText(i,j+1,str);	// we move the matrix one column to the right to allow labels on the left
		}
	}

	// afterwards, we update the status text with some stats on our matrix (size, min, max)
	CString str;
	str.Format("Matrix Re^( %d x %d ) -- max: %10.6f | min: %10.6f",
		thisMatrix.rows(), thisMatrix.cols(),
		thisMatrix.maxCoeff(), thisMatrix.minCoeff());
	GetDlgItem(IDD_STATUSTEXT_EDIT)->SetWindowTextA(str);

	// update the algebraic and statistic text edit
	doAlgebraicAnalysis();
	doStatisticAnalysis();
}

void CMatrixDialog::setMatrix( MatrixXd& mat )
{
	thisMatrix = mat;
}

/*! This functions dynamically sizes the control elements to fill the 
 *  dialog box.
 */
void CMatrixDialog::AdjustLayout()
{
	// determine the size of the current window
	if (GetSafeHwnd() == NULL) 
		return;
	CRect rectClient;
	GetClientRect(rectClient);

	// our custom settings for the size and layout
	const int width = rectClient.Width();
	const int height = rectClient.Height();
	
	CRect tabRect;
	m_wndTabs.GetClientRect(&tabRect);
	// place the main view as the main controls are now tabbed in the 
	// tab-control, we have to resize the tab control whereas the child 
	// windows inherit the size from the tab control
	m_wndTabs.SetWindowPos(0, border, border, width-2*border, height-2*border, SWP_SHOWWINDOW);

	GetDlgItem(IDOK)->ShowWindow(TRUE);
	GetDlgItem(IDOK)->SetWindowPos(0, border, height-border,0,0,SWP_NOSIZE);
	RedrawWindow();
}

void CMatrixDialog::doAlgebraicAnalysis()
{
	// compile the report
	stringstream ss;
	// the following properties are tested
	// size (dimension)
	ss << "Size: " << thisMatrix.rows() << " x " << thisMatrix.cols() << endl;
	// rank
	ss << "Rank: " << thisMatrix.fullPivLu().rank() << endl;
	// for square matrices: Determinant
	if (thisMatrix.cols() == thisMatrix.rows())
	{
		ss << "Determinant: " << thisMatrix.fullPivLu().determinant() << endl;
	}
	// eigenvalues, singular values
	ss << "Singular Values: "  << thisMatrix.jacobiSvd().singularValues().transpose() << endl;
	
	// copy the analysis to the respective control
	CString d2;
	d2 = ss.str().c_str();
	d2.Replace("\n","\r\n");
	m_wndTabs.GetDlgItem(IDD_MATRIX_ALGEBRA_EDIT)->SetWindowTextA(d2);	
}

void CMatrixDialog::doStatisticAnalysis()
{
	// compile the report
	stringstream ss;
	// size (dimension)
	ss << "Size: " << thisMatrix.rows() << " x " << thisMatrix.cols() << endl;
	// show min/max element per column
	ss << "per column statistics" << endl
	  << "min:  " << thisMatrix.colwise().maxCoeff() << std::endl
	  << "mean: " << thisMatrix.colwise().mean() << std::endl
	  << "max:  " << thisMatrix.colwise().minCoeff() << std::endl;

	// show min/max element per row
//	MatrixXd rowstats;
//	rowstats << thisMatrix.rowwise().minCoeff(), 
//		thisMatrix.rowwise().mean(),
//		thisMatrix.rowwise().maxCoeff();
//	ss << "per row statistics: min | mean | max" << endl
//	  << rowstats << std::endl;

	// copy the analysis to the respective control
	CString d2;
	d2 = ss.str().c_str();
	d2.Replace("\n","\r\n");
	m_wndTabs.GetDlgItem(IDD_MATRIX_STAT_EDIT)->SetWindowTextA(d2);	
}

/*! This function generates the respecitve control elements and fills the content of the matrix
 *  into the list control. 
 *  This function is shall be split into sub routines to better reflect different stages of usage.
 *  Eventually, the content of the matrix is copied to the clipboard, which should better be done
 *  in a seperate event. 
 */
BOOL CMatrixDialog::OnInitDialog()
{
	CDialog::OnInitDialog();

	// the metrics of the dialog is stored here which is better than hard coded; however,
	// we might want to further customize the size of the elements.
	const int width = min(800, (thisMatrix.cols()+1)*78 + 20 + 2*border);
	const int height = min(600, (thisMatrix.rows()+1)*28 + 25 + 2*border);

	// build the main view element: The List View to Display the Matrix
	// resize and center the dialog window
	SetWindowTextA(mainTitle);

	CRect rectDummy(border,border,width-border,height-1*border);
	CRect rectDummy2;
	rectDummy2.SetRectEmpty();
	if (!m_wndTabs.Create(CMFCTabCtrl::STYLE_3D_ROUNDED, rectDummy, this, IDD_TAB_CTRL, CMFCTabCtrl::LOCATION_TOP))
	{
		printf("Error while creating the tab control in CMatrixDialog\n");
		return -1;
	}
	
	// create the main List Control which shall display the matrix;
	// we have to yet tested large matrices with scrolling
	if (pLC)
		delete pLC;
	pLC = new CListCtrl();

	// generate the menu button to open the context menu
	CButton* pButton = new CButton;
	pButton->Create("Menu", WS_CHILD | WS_VISIBLE, CRect(0,0,50,25),this, IDD_MENU_BUTTON);

	// generate the edit file to show status information
	CEdit *pEdit = new CEdit;
	pEdit->Create(WS_CHILD | WS_VISIBLE , CRect(55,0, width-border,25), this, IDD_STATUSTEXT_EDIT);

	ModifyStyle(0, WS_SIZEBOX | WS_POPUP | WS_THICKFRAME | WS_CAPTION | WS_SYSMENU);

	CRect rect(border,border,width-border,height-2*border);
	if (!pLC->Create(WS_CHILD | WS_VISIBLE | LVS_REPORT | WS_HSCROLL | WS_VSCROLL, rectDummy2, &m_wndTabs, IDD_MATRIX_VIEW))
	{
		TRACE0("Error while creating the matrix view element in CMatrixDialog.\n");
		return false;
	}

	// this control shall show the text rendering of the matrix (that is also transfered to the clipboard)
	CEdit *pE = new CEdit;
	pE->Create(WS_CHILD | WS_VISIBLE | ES_MULTILINE | ES_READONLY | ES_LEFT | WS_HSCROLL |  WS_VSCROLL, rectDummy2, &m_wndTabs, IDD_MATRIX_EDIT);
	std::stringstream ss;
	ss << thisMatrix;//.format(IOMapleMatrix);
	pE->SetWindowTextA(ss.str().c_str());

	// this contorl is foreseen to show the algebraic properties of the matrix
	CEdit *pEAlgebra = new CEdit;
	pEAlgebra->Create(WS_CHILD | WS_VISIBLE | ES_MULTILINE | ES_READONLY | ES_LEFT | WS_HSCROLL |  WS_VSCROLL, rectDummy2, &m_wndTabs, IDD_MATRIX_ALGEBRA_EDIT);
	// this contorl is foreseen to show the statistic properties of the matrix
	CEdit *pEStat = new CEdit;
	pEStat->Create(WS_CHILD | WS_VISIBLE | ES_MULTILINE | ES_READONLY | ES_LEFT | WS_HSCROLL |  WS_VSCROLL, rectDummy2, &m_wndTabs, IDD_MATRIX_STAT_EDIT);

	m_wndTabs.AddTab(pLC,_T("Grid"),(UINT)-1);	
	m_wndTabs.AddTab(pE,_T("Text"),(UINT)-1);	
	m_wndTabs.AddTab(pEAlgebra,_T("Algebra"),(UINT)-1);	
	m_wndTabs.AddTab(pEStat,_T("Statistic"),(UINT)-1);	
	m_wndTabs.EnableTabSwap(false); // disable swap to make sure, that the tab "Python" has the index 1 (solves a problem with the hit test for the grid control

	SetWindowPos(NULL,0,0,width,height,SWP_HIDEWINDOW);
	AdjustLayout();

	CenterWindow();

	// make the default buttons invisible
	GetDlgItem(IDCANCEL)->ShowWindow(FALSE);
	GetDlgItem(IDRESET)->ShowWindow(FALSE);
	
	// fill the main control with the content of the matrix
	updateMatrixContent();

	// show the window
	ShowWindow(SW_SHOW);

	// return TRUE
	return TRUE;  
}

void CMatrixDialog::OnMenuButton()
{
	if (!GetDlgItem(IDD_MENU_BUTTON))
		return;
	CRect rectClient;
	GetDlgItem(IDD_MENU_BUTTON)->GetClientRect(rectClient);
	OnRButtonUp(0,CPoint(rectClient.top,rectClient.left));
}

void CMatrixDialog::OnRButtonUp(UINT nFlags, CPoint point)
{
	// if the right button is clicked (release) we show the context menu
	// the code is taken from the ShapeListViewDlg where a similar behavior is required
	CMenu mMenu,mSubMenu;
	VERIFY(mSubMenu.CreateMenu());
	VERIFY(mMenu.CreateMenu());
		
	// create a popup menu and insert some entries
	mMenu.AppendMenu(MF_STRING, IDD_MV_COPY2CLIPBOARD, _T("&Copy to Clipboard"));
//	mMenu.AppendMenu(MF_STRING, IDD_MV_SAVETOFILE, _T("&Save to File ..."));
	mMenu.AppendMenu(MF_SEPARATOR,0,_T(""));
	mMenu.AppendMenu(MF_STRING, IDD_MV_CSV2CLIPBOARD, _T("Copy in CS&V to Clipboard"));
	mMenu.AppendMenu(MF_STRING, IDD_MV_MAPLE2CLIPBOARD, _T("Copy in &MAPLE to Clipboard"));
	mMenu.AppendMenu(MF_STRING, IDD_MV_MATLAB2CLIPBOARD, _T("Copy in M&ATLAB to Clipboard"));
	mMenu.AppendMenu(MF_STRING, IDD_MV_PYTHON2CLIPBOARD, _T("Copy in &PYTHON to Clipboard"));
	mMenu.AppendMenu(MF_STRING, IDD_MV_LATEX2CLIPBOARD, _T("Copy in &LaTeX to Clipboard"));
	mMenu.AppendMenu(MF_STRING, IDD_MV_HTML2CLIPBOARD, _T("Copy in &HTML to Clipboard"));
	mMenu.AppendMenu(MF_SEPARATOR,0,_T(""));
	mMenu.AppendMenu(MF_STRING, IDD_MV_TRANSPOSE_MATRIX, _T("&Transpose Matrix"));
	mMenu.AppendMenu(MF_STRING, IDD_MV_INVERSE_MATRIX, _T("&Inverse Matrix"));
	mSubMenu.AppendMenu(MF_POPUP, (UINT)mMenu.m_hMenu, "");
	ClientToScreen(&point);
	// start the menu, the result is sent as windows messange 
	mMenu.TrackPopupMenu(TPM_LEFTALIGN, point.x, point.y, this);
}

void CMatrixDialog::OnBnClickedOk()
{
	CDialog::OnOK();
}

/*! Copy the matrix to the clipboard in plain text. The following function
 *  with similar 
 */ 
void CMatrixDialog::OnCopyToClipboard()
{
	// this code shall copy the current matrix to the clipboard
	// prepare the matrix by converting to stringstram and then to CString
	std::stringstream ss;
	ss << thisMatrix;
	CopyTextToCpliboard(ss.str());
}

void CMatrixDialog::OnCopyMapleClipboard()
{
	std::stringstream ss;
	ss << thisMatrix.format(IOMapleMatrix);
	CopyTextToCpliboard(ss.str());
}

void CMatrixDialog::OnCopyMatlabClipboard()
{
	std::stringstream ss;
	ss << thisMatrix.format(IOMatlabMatrix);
	CopyTextToCpliboard(ss.str());
}

void CMatrixDialog::OnCopyCsvClipboard()
{
	std::stringstream ss;
	ss << thisMatrix.format(IOCSV);
	CopyTextToCpliboard(ss.str());
}

void CMatrixDialog::OnCopyLatexClipboard()
{
	static const Eigen::IOFormat IOLatexMatrix(Eigen::StreamPrecision, 0, "& ", "\\\\\n", "", "", "", "");
	std::stringstream ss;
	ss << thisMatrix.format(IOLatexMatrix);
	CopyTextToCpliboard(ss.str());
}

void CMatrixDialog::OnCopyHtmlClipboard()
{
	static const Eigen::IOFormat IOHtmlMatrix(Eigen::StreamPrecision, 0, "</td><td> ", "\n", "<tr><td>", "</td></tr>", "<table>", "</table>");
	std::stringstream ss;
	ss << thisMatrix.format(IOHtmlMatrix);
	CopyTextToCpliboard(ss.str());
}

void CMatrixDialog::OnCopyPythonClipboard()
{
	static const Eigen::IOFormat IOPythonMatrix(Eigen::StreamPrecision, 0, ", ", "\n", "[", "],", "[", "]");
	std::stringstream ss;
	ss << thisMatrix.format(IOPythonMatrix);
	CopyTextToCpliboard(ss.str());
}

void CMatrixDialog::OnTransposeMatrix()
{
	thisMatrix.transposeInPlace();
	updateMatrixContent();
}

void CMatrixDialog::OnInverseMatrix()
{
	// invert only if squre
	if (thisMatrix.cols() == thisMatrix.rows())
		thisMatrix = thisMatrix.inverse();
	updateMatrixContent();
}

void CMatrixDialog::OnSize(UINT nType, int cx, int cy)
{
	CDialog::OnSize(nType, cx, cy);
	AdjustLayout();
}
