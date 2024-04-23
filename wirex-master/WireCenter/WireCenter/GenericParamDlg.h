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

/*! \file GenericParamDlg.h
 *
 *	\author   Andreas Pott
 *
 *  \brief General purpose to generate quickly input dialogs
 *  for a variaty of options to be set.
 *	This file consists of two classes:
 *	CGenericParamDlg: A simple dialog box

 *	CEnhancedDialog: A more enhanced dialog box, which generates 
 *  its appearance automatically, depending on the input data types.
 *	
 *	Examples for using CEnhancedDialog:
 *	---------------------------------------------------
 *  
 *	CEnhancedDialog GPD("Einstellungen",300);		//Erzeugt ein neues DialogFeld mit dem angegebenen Titel und der Breite (weitere Optionen möglich!)
 *	GPD.StartGroupBox("Gebiet 1");					//Beginne eine neue GroupBox
 *		GPD.addItem("Wert 1",m_TypInteger);			//addItem entscheidet selbst (typenabhängig), welche Eingabeform benutzt wird (Edit+Label, Checkbox, ...)
 *		GPD.addItem("Eine Ganzzahl:", m_TypInteger);
 *		GPD.StartGroupBox("Gebiet 1.1");			//beginne noch eine neue GroupBox (verschachtelt!)
 *			GPD.addItem("Info-Label:");				//ohne Typ wird es nur ein Label...
 *			GPD.addItem("Wahr?",m_TypeBool);		//sonst hängts vom Typ ab (hier : CheckBox)
 *			GPD.addItem("Double-Zahl:",m_TypeDouble);	// hier : Label+Edit
 *			GPD.addItem("Ein String:",m_TypeString);
 *		GPD.EndGroupBox();							//innere Groupbox endet hier
 *	GPD.EndGroupBox();								//äußere Groupbox endet hier
 *	GPD.NextDialogColumn();							//erzwinge manuellen Umbruch (verhindert automatisch Umbruch mitten in GroupBox)
 *	GPD.StartGroupBox("Gebiet 2");
 *		GPD.addItem("Test 1",m_TypeBool);
 *		GPD.addItem("Test 2",m_TypeBool);
 *		GPD.CreateEdit("Manuelles Erstellen eines Edits:",m_bStringTyp);
 *	GPD.EndGroupBox();
 *	
 */

#pragma once
#include "afxcmn.h"

#include <map>
#include <string>
#include <list>

using namespace std;

//#########################################################
// CGenericParamDlg-Dialogfeld
//#########################################################

class CGenericParamDlg : public CDialog
{
	DECLARE_DYNAMIC(CGenericParamDlg)

public:
	CGenericParamDlg(CWnd* pParent = NULL);   // Standardkonstruktor
	virtual ~CGenericParamDlg();
	
// Dialogfelddaten
	enum { IDD = IDD_GENERIC_PARAMETER_DLG_DIALOG };

private:
	enum eType { eDouble, eInt, eString, eBool, eChar };
	bool bCBactivated; // flag for activating the checkboxes (true means, there are only bool values in the list and the checkbox can be used)

	//! embedded variant type to handle some standard types in the generic editor
	class TGenericValueVariant {
	public:	// data
		eType type;				//!< internal type selector
		double dValue,*dPtr;	//!< pointer-value pair of mapped double value
		int iValue,*iPtr;		//!< pointer-value pair of mapped int value
		string sValue,*sPtr;	//!< pointer-value pair of mapped string value
		bool bValue,*bPtr;		//!< pointer-value pair of mapped bool value
		char cValue,*cPtr;		//!< pointer-value pair of mapped char value
		int pos;				//!< position of the item in a CListCtrl (this id is used to identity the related item)
	public: // methods
		void readData()
		{
			switch (type)
			{
			case eDouble:	dValue = *dPtr; break;
			case eInt:		iValue = *iPtr; break;
			case eBool:		bValue = *bPtr; break;
			case eString:	sValue = *sPtr; break;
			case eChar:		cValue = *cPtr; break;
			}
		}
		void writeData()
		{
			switch (type)
			{
				case eDouble:	*dPtr = dValue; break;
				case eInt:		*iPtr = iValue; break;
				case eBool:		*bPtr = bValue; break;
				case eString:	*sPtr = sValue; break;
				case eChar:		*cPtr = cValue; break;
			}
		}
		CString toString()
		{
			CString str;
			switch (type)
			{
				case eDouble:	str.Format("%f",dValue); break;
				case eInt:		str.Format("%i",iValue); break;
				case eBool:		str.Format("%s",bValue?"true":"false"); break;
				case eString:	str.Format("%s",sValue); break;
				case eChar:		str.Format("%c",cValue); break;
			}
			return str;
		}
		void fromString(const CString& str)
		{
			// parse and interprete the content of the edit control
			switch (type)
			{
				case eDouble:	dValue = atof(str); break;
				case eInt:		iValue = atoi(str); break;
				case eBool:		bValue = strcmp(str,"true")==0; break;
				case eString:	sValue = str; break;
				case eChar:		cValue = str[0]; break;
			}	
		}
	};
	//! the io_map mapping the symbolic name to the variant datatype
	typedef map<string,TGenericValueVariant> value_map;
	typedef value_map::iterator map_iterator;
	value_map io_map;

public:
	//!< add a double value to the io mapping
	void addItem(const string& name, double& value);
	void addItem(const string& name, int& value);
	void addItem(const string& name, bool& value);
	void addItem(const string& name, string& value);
	void addItem(const string& name, char& value);

	//!< copy current value from the mapped values into the buffer
	void readData();
	//!< flash the buffered values into the mapped variables
	void writeData();

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV-Unterstützung

	DECLARE_MESSAGE_MAP()
public:
	void CGenericParamDlg::updateTreeView();
	//!< the control object for the main list view
	CListCtrl m_ListCtrl;
	virtual BOOL OnInitDialog();
	afx_msg void OnBnClickedApplyValuesButton();
	afx_msg void OnBnClickedReloadValuesButton();
	afx_msg void OnLvnEndlabeleditGenericParameterList(NMHDR *pNMHDR, LRESULT *pResult);
protected:
	virtual void OnOK();
};


//####################################################
// CEnhancedDialog-Dialogfeld
//####################################################

//###################################################
//Planned Enhancements:
//		- Create a ComboBox element
//		- Create a Separator element
//		- Create a RadioButton(Group) element

//! enumerate the different types of controls here
enum eElementType { eETEditElement, eETLabelElement, eETCheckboxElement, eETNonEditableComboboxElement, 
		eETEditableComboboxElement, eETGroupboxElementStart, eETGroupboxElementEnd, eETNextDialogColumn};

//! enumerate the different types of supported data formats here
enum eDataType { eDTInteger, eDTDouble, eDTCString, eDTString, eDTBoolean, eDTUnknown };

class TElementType 
{
public:			
	//members for data storage
	eElementType m_ElementType;	//!< the element class
	CWnd* m_pElement;			//!< the element itself (depends on m_ElementType)
	eDataType m_DataType;		//!< the type of the given value
	void* m_pData;				//!< the pointer to the data (it's " void* " , because it could be int,double,string...) (depends on m_DataType)
	CStatic* m_pStatic;			//!< the optional static to show the name of some elements
	CString m_StaticText;		//!< the optional static text
	void* m_pAdditional_inputdata; //!< some additional input data if necessary

	//! constructor of TElementType
	TElementType(eElementType _element_type, CWnd* _pElement, eDataType _DataType, void* _pData, CStatic* _pStatic = NULL, CString _StaticText="", void* _additional_inputdata = NULL) 
	{ 
		//set the members from the values in the constructor
		m_ElementType = _element_type;
		m_pElement = _pElement;
		m_DataType = _DataType;
		m_pData = _pData;
		m_pStatic = _pStatic;
		m_StaticText = _StaticText;
		m_pAdditional_inputdata = _additional_inputdata;
	}

	//! destructor of TElementType
	~TElementType() { }
};


class CEnhancedDialog : public CDialog
{
	DECLARE_DYNAMIC(CEnhancedDialog)

public:
	//! create a new enhanced dialog with a given title
	CEnhancedDialog(CString title="Default-Dialog", int element_width = 250, bool activate_resetbutton = true,
					int element_margin = 8, int element_height = 20, int global_margin = 15);   // Standardkonstruktor
	~CEnhancedDialog();

	// the dialog dummy
	enum { IDD = IDD_EMPTYDIALOG };

	//! create a new edit field combined with a descriptive text on the left
	int CreateEdit(int* variable_ref, CString title = "Dummy-Edit");
	int CreateEdit(CString* variable_ref, CString title = "Dummy-Edit");
	int CreateEdit(string* variable_ref, CString title = "Dummy-Edit");
	int CreateEdit(double* variable_ref , CString title = "Dummy-Edit");
	//! create a new checkbox
	int CreateCheckbox(bool* variable_ref , CString title  = "Dummy-Checkbox");
	//! create a new label
	int CreateLabel(CString title = "Dummy-Label");
	//! create a combobox
	int CreateComboBox(int* selected_entry, CStringArray* input_strings, CString title = "Dummy-Combobox");
	int CreateComboBox(CString* selected_entry, CStringArray* input_strings, CString title = "Dummy-Combobox");
	//! create a radiobutton (NOT IMPLEMENTED YET!)
	int CreateRadioButton(int* variable_ref, int group_index, CString title = "Dummy-Radiobutton"){};

	//! creates an element just depending on the type of the given variable ( and for backward compatibility for CGenericParamDlg )
	void addItem(CString title, bool& variable_ref)		{CreateCheckbox(&variable_ref,title);}
	void addItem(CString title, CString& variable_ref)	{CreateEdit(&variable_ref,title);}
	void addItem(CString title, string& variable_ref)   {CreateEdit(&variable_ref,title);}
	void addItem(CString title, int& variable_ref)		{CreateEdit(&variable_ref,title);}
	void addItem(CString title, double& variable_ref)	{CreateEdit(&variable_ref,title);}
	void addItem(CString title)							{CreateLabel(title);}
	void addItem(CString title, int& selected_entry, CStringArray& input_strings) {CreateComboBox(&selected_entry, &input_strings, title);}
	void addItem(CString title, CString& selected_entry, CStringArray& input_strings) {CreateComboBox(&selected_entry, &input_strings, title);}

	//! create a groupbox which can be nested ( always use with EndGroupBox() to determine end ! )
	int StartGroupBox(CString title = "Dummy-GroupBox");
	int EndGroupBox();

	//! forces a new column while avoiding to break GroupBoxes
	int NextDialogColumn();

protected:
	DECLARE_MESSAGE_MAP()

public:
	virtual BOOL OnInitDialog();
	
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedCancel();
	afx_msg void OnBnClickedReset();

private:
	// the new font
	CFont default_font;
	// the font height		
	int font_height;

	//! set the margin between the elements and a global margin for the window
	int element_margin;
	int m_global_margin;

	//! set the element height and width here
	int element_height;
	int element_width;
	
	//! a member variable to store the current ID
	int m_current_ID;

	//! store the actual height, width of all objects
	int m_global_height;
	int m_global_width;

	//! store the maximum heigh ever reached
	int m_global_maximum_height;

	//! store the groupboxes for alignment
	std::list<CButton*> pGroupBoxList;
	//! check, if an column wrap would intersect a groupbox
	bool bWouldIntersectGroupBox;

	//! do you want a resetbutton?
	bool activate_resetbutton;

	//! the title of the dialog box
	CString main_title;

	//! a dynamic array to store the element pointer (all elements buttons, edit, statics have a common ancestor CWnd, so lets take this buddy to provide minimum functions)
	std::list<TElementType> element_storage;
	//! a global iterator
	std::list<TElementType>::iterator itor;

	//! increase the intern ID counter
	int SetID()
	{
		m_current_ID++;
		return m_current_ID;
	}

	//! build the elements
	afx_msg int BuildElements(bool first_run);
	//! save the elements back
	afx_msg int SaveElements();
};


/*! This class is similar to the generic dialogs provided by CEnhancedDialog 
 *  and CGenericParamDialog as it extens the concept to display matrices from 
 *  eigen3. The view class provided the following functions
 *    - copy the matrix to the clipboard (this is essentially 
 *      provided in the OnInitDialog method where an unformatted version of the 
 *      matrix is copy to the clipboard.)
 *    - select the formatting of the matrix, e.g. maple, matlab, 
 *      space separated values, python nested lists, csv, html, 
 *    - additional information on the matrix e.g. size, min/mean/max values.
 *    - transpose and invert the matrix (only in this view); possibily provide 
 *      other functions that are available "out-of-the-box" from eigen3 such as
 *      (pseudo-)invert
 *    - compute some simple properties of the matrix such as size, determinant,
 *      and eigenvalues.
 *  
 *  \remark Some ideas on nice extension:
 *    - add an option to save the matrix to a file
 *    - add behavior to deal with huge matrices, e.g. by providing upper limits 
 *      of what is accepted as matrix; first GU tests show that even for 50x50
 *      performance decreases. Another test with a 500x20 matrix showed acceptable 
 *      performance
 *    - add variable size to the dialog box, choose proper initial size on create.
 *    - add custom labels to columns and rows
 */
class CMatrixDialog : public CDialog
{
	DECLARE_DYNAMIC(CMatrixDialog)

public:
	//! create a new enhanced dialog with a given title
	CMatrixDialog(CString title, MatrixXd mat);   // Standardkonstruktor
	~CMatrixDialog();

	//! creates an element just depending on the type of the given variable ( and for backward compatibility for CGenericParamDlg )
	void setMatrix(MatrixXd& mat );

protected:
	// the dialog dummy
	enum { IDD = IDD_EMPTYDIALOG };
	// the ids used for our internal controls (e.g. the context menu and the CWnd controls)
	enum { 
		IDD_MV_FIRST_ITEM = 10,
		// dialog item ids
		IDD_TAB_CTRL,
		IDD_MATRIX_VIEW,
		IDD_MENU_BUTTON,
		IDD_STATUSTEXT_EDIT,
		IDD_MATRIX_EDIT,
		IDD_MATRIX_ALGEBRA_EDIT,
		IDD_MATRIX_STAT_EDIT,
		// commands ids
		IDD_MV_COPY2CLIPBOARD,
		IDD_MV_SAVETOFILE,
		IDD_MV_CSV2CLIPBOARD,
		IDD_MV_MAPLE2CLIPBOARD,
		IDD_MV_MATLAB2CLIPBOARD,
		IDD_MV_PYTHON2CLIPBOARD,
		IDD_MV_HTML2CLIPBOARD,
		IDD_MV_LATEX2CLIPBOARD,
		IDD_MV_TRANSPOSE_MATRIX,
		IDD_MV_INVERSE_MATRIX,
		IDD_MV_LAST_ITEM
	};
	
	// GUI related members 
	CFont default_font;		//!< the new font
	int font_height;		//!< the font height		
	CListCtrl* pLC;			//!< the main list control
	int border;				//!< a constant for layouting the dialog
	CMFCTabCtrl m_wndTabs;

	// data model related members
	CString mainTitle;		//!< the title of the dialog box
	MatrixXd thisMatrix;	//!< the matrix to be shown by the dialog

	// helper functions
	//! send the content of the string to the clipboard
	bool CopyTextToCpliboard(std::string& test);
	// update the man matrix grid view
	void updateMatrixContent();
	// compute the placing and size of the controls after change in the size of the dialog
	void AdjustLayout();
	// compute algebraic properties of the matrix and show a report in the respective edit
	void doAlgebraicAnalysis();
	// compute statistic properties of the matrix and show a report in the respective edit
	void doStatisticAnalysis();

protected:
	DECLARE_MESSAGE_MAP()

public:
	virtual BOOL OnInitDialog();
	
	afx_msg void OnBnClickedOk();
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnMenuButton();
	afx_msg void OnCopyToClipboard();
	afx_msg void OnCopyMapleClipboard();
	afx_msg void OnCopyMatlabClipboard();
	afx_msg void OnCopyCsvClipboard();
	afx_msg void OnCopyLatexClipboard();
	afx_msg void OnCopyHtmlClipboard();
	afx_msg void OnCopyPythonClipboard();
	afx_msg void OnTransposeMatrix();
	afx_msg void OnInverseMatrix();
	afx_msg void OnSize(UINT nType, int cx, int cy);
};
