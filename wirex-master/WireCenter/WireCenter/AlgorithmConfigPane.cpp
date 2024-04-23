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

// AlgorithmConfigPane.cpp: Implementierungsdatei
//

#include "stdafx.h"
//#include "WireCenter.h"
#include "AlgorithmConfigPane.h"
#include "resource.h"
//#include "WireCenterDoc.h"
//#include "WireCenterView.h"


/////////////////////////////////////////////
// IMPLEMENTATION OF CAlgorithmConfigPane
/////////////////////////////////////////////

CAlgorithmConfigPane::CAlgorithmConfigPane()
{
}

BEGIN_MESSAGE_MAP(CAlgorithmConfigPane, CDockablePane)
	ON_WM_CREATE()
	ON_WM_SIZE()
	ON_REGISTERED_MESSAGE(AFX_WM_PROPERTY_CHANGED, &CAlgorithmConfigPane::OnPropertyChanged)
END_MESSAGE_MAP()

void CAlgorithmConfigPane::AdjustLayout()
{
	if (GetSafeHwnd() == NULL) return;
	CRect rectClient;
	GetClientRect(rectClient);

	int cyTlb = m_wndToolBar.CalcFixedLayout(FALSE,TRUE).cy;

	// we still have to add some spacing between the controls
	m_wndToolBar.SetWindowPos(NULL,rectClient.left,rectClient.top,rectClient.Width(),cyTlb,SWP_NOACTIVATE | SWP_NOZORDER);
	m_wndProperties.SetWindowPos(NULL,rectClient.left, cyTlb, rectClient.Width(), rectClient.Height()-cyTlb, SWP_NOACTIVATE | SWP_NOZORDER);
}

int CAlgorithmConfigPane::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CDockablePane::OnCreate(lpCreateStruct) == -1)
		return -1;

	CRect rectDummy;
	rectDummy.SetRectEmpty();

	if(!m_wndProperties.Create(WS_CHILD | WS_VISIBLE | WS_BORDER | WS_CLIPSIBLINGS | WS_CLIPCHILDREN ,rectDummy,this,3))
		return -1;

	// create my toolbar
	m_wndToolBar.Create(this, AFX_DEFAULT_TOOLBAR_STYLE, IDR_INTERACTIVEGEOMETRY_TOOLBAR);
	m_wndToolBar.LoadToolBar(IDR_INTERACTIVEGEOMETRY_TOOLBAR, 0, 0, TRUE /* Ist gesperrt */);
	m_wndToolBar.SetPaneStyle(m_wndToolBar.GetPaneStyle() | CBRS_TOOLTIPS | CBRS_FLYBY);
	m_wndToolBar.SetOwner(this);

	// Alle Befehle werden über dieses Steuerelement geleitet, nicht über den übergeordneten Rahmen:
	m_wndToolBar.SetRouteCommandsViaFrame(FALSE);

	// now we do some fancy stuff with the property grid's font 
	m_wndProperties.SetVSDotNetLook(FALSE);
	m_wndProperties.SetGroupNameFullWidth(TRUE);

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

	m_wndProperties.SetFont(&m_fntPropList);

	// that's all done; we set the font to something nice
	m_wndProperties.EnableHeaderCtrl(FALSE);
	m_wndProperties.EnableDescriptionArea();
	m_wndProperties.SetVSDotNetLook();
	m_wndProperties.MarkModifiedProperties();

	// it's time to put some elements into the grid control
	
	// add the general section
	general = new CMFCPropertyGridProperty(_T("Configuration (readonly)"));
	m_wndProperties.AddProperty(general);
	AdjustLayout();
	return 0;
}


//! Populate the property grid control with the elements found in the reflector
void CAlgorithmConfigPane::addAlgorithm(PCRL::CReflection& Reflector)
{
	// iterate through all elements in the binding table of reflection and extract
	// all information that we can get.
	CPropertyGridReflectorManager mgt;
	for (auto itor = Reflector.getValueMap().cbegin(); itor!=Reflector.getValueMap().cend(); ++itor)
		mgt.setNode(general,itor->second.xpath,itor->second);
}


void CAlgorithmConfigPane::OnSize(UINT nType, int cx, int cy)
{
	CDockablePane::OnSize(nType,cx,cy);
	AdjustLayout();
}


/*! This callback function is triggered when an element of the property grid 
 *  was edited by the user. If the modification was feasible we store the
 *  changed information in the connected reflection variable
 */
LRESULT CAlgorithmConfigPane::OnPropertyChanged(__in WPARAM wparam, __in LPARAM lparam)
{
	// get a pointer the the respective property element 
	CMFCPropertyGridProperty* pProp = (CMFCPropertyGridProperty*) lparam; 
	// let the reflection manager do the work for us
	CPropertyGridReflectorManager mgt;
	return mgt.getNode(pProp);
}

//////////////////////////////////////////////////
// IMPLEMENTATION OF CPropertyGridReflectorManager
//////////////////////////////////////////////////

//! add a data child with a name/value pair under the given root element
//! we should firstly check if a property with the desired name exists in order to update
//! it rather than generating it
void CPropertyGridReflectorManager::setAttribute(CMFCPropertyGridProperty* root, const string& attrib, const PCRL::TReflectionVariant& value)
{
	if (!root)
		return;

	CString label, Value, xpath;
	label.Format("%s", attrib.c_str());
	xpath.Format("%s", value.xpath.c_str());
	// unfortunately, we have to perform a low level data conversion
	switch (value.type)
	{
	case PCRL::TRVDouble: 
		Value.Format("%f",*(value.value.pDouble)); 
		break;
	case PCRL::TRVString: 
		Value.Format("%s",value.value.pString->c_str()); break;
	case PCRL::TRVInt:
		Value.Format("%i",*(value.value.pInt)); 
		break;
	case PCRL::TRVBool:   
		Value = *(value.value.pBool) ? _T("true") : _T("false"); 
		break;
	default:			 
		Value = _T("Value not recognized");
		break;
	}
	// we check if a property with the desired name exists
	CMFCPropertyGridProperty *child=0;
	for (int i=0; i<root->GetSubItemsCount(); i++)
	{
		CString str1,str2;
		str1 = root->GetSubItem(i)->GetName();
		str2.Format("%s",attrib.c_str());
		if (str1.Compare(str2)==0)
		{
			child=root->GetSubItem(i); 
			break;
		}
	}

	if (child)
		// modify the found property
		if (value.type == PCRL::TRVDouble)
			child->SetValue((double)*value.value.pDouble);
		else
			child->SetValue(Value);
	else
	{
		// generate a new property and we save the adress of the ReflectionVariant in order to
		// recovert data if needed. 
		if (value.type == PCRL::TRVDouble)
			root->AddSubItem(new CMFCPropertyGridProperty(label, (double)*value.value.pDouble, xpath,(DWORD)&value));
		if (value.type == PCRL::TRVBool)
		{
			CMFCPropertyGridProperty *pItem = new CMFCPropertyGridProperty(label, Value, xpath,(DWORD)&value);
			pItem->AddOption("false",1);
			pItem->AddOption("true",1);
			root->AddSubItem(pItem);
		}
		if (value.type == PCRL::TRVString)
		{
			root->AddSubItem(new CMFCPropertyGridProperty(label, Value, xpath,(DWORD)&value));
		}
		if (value.type == PCRL::TRVInt)
		{
			// check if the item is of int type AND the respective xpath can be found in the options database
			// then the reflected value is an enum type and we have the options to offer for the user
			if (PCRL::CReflection::getEnumTable(xpath.GetBuffer()))
			{
				// get the options for the enum type
				const char** names = PCRL::CReflection::getEnumTable(xpath.GetBuffer());
				// create the new entry
				CMFCPropertyGridProperty *pItem = new CMFCPropertyGridProperty(label, names[*value.value.pInt], xpath,(DWORD)&value);
				for (int i=0; names[i]!=0; ++i)
				{
					const char *str = names[i];
					pItem->AddOption(str,1);
				}
				root->AddSubItem(pItem);
			}
			else // add a "normal" int value without options
				root->AddSubItem(new CMFCPropertyGridProperty(label, Value, xpath,(DWORD)&value));
		}
	}
}

//! parse the xpath syntax and navigate to the respective MFCPropertyGridProperty
//! the implementation was adopted from the xpath parser used in CReflection for finding the respective XML node
//! however, we have to dublicate some code here since the XML navigation uses the tinyXML API where we have to
//! navigate through MFC classes here. 
bool CPropertyGridReflectorManager::setNode(CMFCPropertyGridProperty* root, const string& xpath, const PCRL::TReflectionVariant& value)
{
	// we do nothing if xpath is empty or if root is NULL
	if (xpath.length()<=0 || root==0)
		return false;

	// now we check for the most likely cases of the content of the xpath-string

	// - an attribute is specified with a string "@XYZ"
	if (xpath.size()>1 &&xpath[0]=='@')
	{
		string attrib = xpath.substr(1,string::npos);
		// ** implement the set function here **
		setAttribute(root,attrib,value);
		return true;
	}

	// - the path starts with "/", in the context of this function, this is an error
	if (xpath[0]=='/')
		// although part of the xpath syntax, we do not support reference to the root node here
		return false;

	// - the path starts with "../", relating the following string to the parent element

	if (xpath.size()>3 && xpath.compare(0,2,"../")==0)
	{
		// access the higher hierachie level in the xml tree
		if (root->GetParent()!=0 && xpath.size()>3)
			return setNode(root->GetParent(),xpath.substr(3,string::npos),value);
		else
			return false;
	}

	// - the xpath starts with "./", relating the following string to the current element
	if (xpath.size()>2 && xpath.compare(0,1,"./")==0)
		// we cut this sequence from the string and make a recursive call will the shorter string
		return setNode(root,xpath.substr(2,string::npos),value);

	// - a path is specified, since the xpath string has the structure "XYZ/..."
	size_t pos = xpath.find('/');
	if (pos==string::npos)
	{
		//! \todo Provide an appropirate implementation for writing node data (and for reading it respectively)
		// the following implementation seems to be not in use and has definitely errors
		// currently writing a node (rather than an note's attribute) is not used by the implemetation
		// an implementation should be similar to the overloaded versions of setAttribute, i.e.
		// overloaded versions of setNode(node,value) are required to perform reading and writing
		// however, since tinyXML provides no conviniance functions for writing node data
		// we skip the implemenation for now
		// btw: the current implementation for reading the node does not support reading the data
		// either. therefore, it is unlikely, that that errors occur (based on the state of the
		// rest of the wirelib implementation). 
		return false;

		// ...
	}
	else
	{
		// paranoia test
		if (xpath.size() <= pos)
			return false;
		// the first part of the string is a tag-name but we want to access its sub-items
		string tag = xpath.substr(0,pos);
		// it seems that property grids cannot easily search their children by name
		// therefore, we implement our own little search through the items
		CMFCPropertyGridProperty *child=0;
		for (int i=0; i<root->GetSubItemsCount(); i++)
		{
			CString str1,str2;
			str1 = root->GetSubItem(i)->GetName();
			str2.Format("%s",tag.c_str());
			if (str1.Compare(str2)==0)
			{
				child=root->GetSubItem(i); 
				break;
			}
		}
		if (child)
			return setNode(child,xpath.substr(pos+1,string::npos),value);
		else
		{
			// create the node
			CMFCPropertyGridProperty *Child = new CMFCPropertyGridProperty(tag.c_str());
    		root->AddSubItem(Child);
			return setNode(Child,xpath.substr(pos+1,string::npos),value);
		}
	}

	// for some reason we found nothing to do with the xpath string; perhaps the xpath string is not valid
	return false;
}


//! set the binded value after user change of the GridProperty.
//! we could use the pointer received from pProp->GetData() to read the 
//! content of the field that was edited. However, we do not know the 
//! \param pProp [in] pointer to the property in the CPropertyGrid that was changed
//! \return true, if the value was successfully extracted and transfered into the reflection structure
bool CPropertyGridReflectorManager::getNode(CMFCPropertyGridProperty* pProp)
{
	// safety first; we check the pointer to be nonzero
	if (!pProp)
		return false;
	// extract the connected data member, i.e. the target adress where the changed value 
	// shall be stored
	PCRL::TReflectionVariant *var = (PCRL::TReflectionVariant *)pProp->GetData();
	if (!var)
		return false;	// perhaps we should return something else than 0 to indicate that the conversion was not done
	
	// extract the value if the type is double
	if (var->type == PCRL::TRVDouble)
	{
		*(var->value.pDouble) = pProp->GetValue().dblVal;
		return true;
	}
	// extract the value if the type is int
	if (var->type == PCRL::TRVInt)
	{
		// receive the value
		CString str(pProp->GetValue().bstrVal);
		
		// test if has an enum option list and if yes, which one was selected
		const char **stringtable = PCRL::CReflection::getEnumTable(var->xpath);
		// try to convert to an int
		if (stringtable==0)
			*(var->value.pInt) = atoi(str);
		else
		{
			// we have to search to figure out if the option is valid
			for (int i=0; stringtable[i]!=0; ++i)
			{
				if (str==CString(stringtable[i]))
				{
					*(var->value.pInt) = i;
					break;
				}
			}
		}
		return true;
	}
	// extract the value if the type is bool
	if (var->type == PCRL::TRVBool)
	{
		CString str(pProp->GetValue().bstrVal);
		if (str=="true")
			*(var->value.pBool)=true;
		else 
			*(var->value.pBool)=false;
		return true;
	}
	// extract the value if the type is string
	if (var->type == PCRL::TRVString)
	{
		CString str(pProp->GetValue().bstrVal);
		*(var->value.pString)=str.GetBuffer();
		return true;
	}

	// for some reason the property had non of the expected types
	return false;
}
