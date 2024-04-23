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

// DraftDlgs.cpp : implementation file for serveral dialog for robot drafting
//

#include "stdafx.h"
#include "WireCenter.h"
#include "DraftDlgs.h"
#include "afxdialogex.h"
#include <sstream>

/////////////////////////////////////////////////////////////
// DescribeApplicationDlg dialog
/////////////////////////////////////////////////////////////

IMPLEMENT_DYNAMIC(CDescribeApplicationDlg, CDialog)

CDescribeApplicationDlg::CDescribeApplicationDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CDescribeApplicationDlg::IDD, pParent)
{
#ifndef _WIN32_WCE
	EnableActiveAccessibility();
#endif
	m_delta_minus_edit_x_value = 0;
	m_delta_minus_edit_y_value = 0;
	m_delta_minus_edit_z_value = 0;
	m_bSimplifiedDialog = FALSE;
}

CDescribeApplicationDlg::~CDescribeApplicationDlg()
{
}

void CDescribeApplicationDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_WORKSPACE_XMIN_EDIT, Req.minWS.x());
	DDX_Text(pDX, IDC_WORKSPACE_XMAX_EDIT, Req.maxWS.x());
	DDX_Text(pDX, IDC_WORKSPACE_YMIN_EDIT, Req.minWS.y());
	DDX_Text(pDX, IDC_WORKSPACE_YMAX_EDIT, Req.maxWS.y());
	DDX_Text(pDX, IDC_WORKSPACE_ZMIN_EDIT, Req.minWS.z());
	DDX_Text(pDX, IDC_WORKSPACE_ZMAX_EDIT, Req.maxWS.z());
	DDX_Text(pDX, IDC_WORKSPACE_XDELTAMINUS_EDIT, m_delta_minus_edit_x_value);
	DDX_Text(pDX, IDC_WORKSPACE_YDELTAMINUS_EDIT, m_delta_minus_edit_y_value);
	DDX_Text(pDX, IDC_WORKSPACE_ZDELTAMINUS_EDIT, m_delta_minus_edit_z_value);
	DDX_Text(pDX, IDC_CROSSSECTION_XMIN_EDIT , Req.minIS.x());
	DDX_Text(pDX, IDC_CROSSSECTION_XMAX_EDIT , Req.maxIS.x());
	DDX_Text(pDX, IDC_CROSSSECTION_YMIN_EDIT , Req.minIS.y());
	DDX_Text(pDX, IDC_CROSSSECTION_YMAX_EDIT , Req.maxIS.y()); 
	DDX_Text(pDX, IDC_CROSSSECTION_ZMIN_EDIT , Req.minIS.z());
	DDX_Text(pDX, IDC_CROSSSECTION_ZMAX_EDIT , Req.maxIS.z()); 
	DDX_Text(pDX, IDC_ROBOT_MAXVELOCITY_EDIT , Req.velocity); 
	DDX_Text(pDX, IDC_ROBOT_MAXACCERLATION_EDIT , Req.acceleration);
	DDX_Text(pDX, IDC_ROBOT_MAXLOADFORCE_EDIT , Req.payload); 
	DDX_Text(pDX, IDC_ROBOT_MAXPROCESSFORCE_EDIT , Req.force);
	DDX_Text(pDX, IDC_ROBOT_MAXTORQUE_EDIT , Req.torque);
}


BEGIN_MESSAGE_MAP(CDescribeApplicationDlg, CDialog)
	ON_BN_CLICKED(IDOK, &CDescribeApplicationDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDC_RESET, &CDescribeApplicationDlg::OnBnClickedReset)
	// map all click messages from the radio buttons to one procedure
	ON_BN_CLICKED(IDC_MINMAX_RADIOBUTTON, &CDescribeApplicationDlg::OnBnClickedRadiobutton)
	ON_BN_CLICKED(IDC_MIDDLERADIUS_RADIOBUTTON, &CDescribeApplicationDlg::OnBnClickedRadiobutton)
	ON_BN_CLICKED(IDC_MIDDLEDELTA_RADIOBUTTON, &CDescribeApplicationDlg::OnBnClickedRadiobutton)
END_MESSAGE_MAP()

//convert function for formatted string
CString CDescribeApplicationDlg::ToString(double value)
{
	CString str;
	str.Format("%.5f", value);
	return str;
}

void CDescribeApplicationDlg::calculateValues(int choosen_output_radiobutton_id)
{
	//old solution with:
	// 	 GetDlgItemText(IDC_WORKSPACE_XMIN_EDIT,str);
	//   SetDlgItemText(IDC_WORKSPACE_XMIN_EDIT,ToString(center - radius));
			 
	//declare temporary variables
	double min,max,center,radius,delta_plus,delta_minus;

	//Get values from the controls into the linked variables
	UpdateData(true);

	// test, which mode was previously choosen and convert values to min, max values
	switch ( m_previouslycheckedRadioButton )
	{
         case IDC_MINMAX_RADIOBUTTON: // min,max option was checked -> no calculations necessary
			 //NOTHING TO DO HERE
			 break;

		 case IDC_MIDDLERADIUS_RADIOBUTTON: // center,radius option was checked -> calculate min, max values
			 //x values
			 center = Req.minWS.x();
			 radius = Req.maxWS.x();
			 Req.minWS.x() = center - radius;
			 Req.maxWS.x() = center + radius;
			 //y values
			 center = Req.minWS.y();
			 radius = Req.maxWS.y();
			 Req.minWS.y() = center - radius;
			 Req.maxWS.y() = center + radius;
			 //z values
			 center = Req.minWS.z();
			 radius = Req.maxWS.z();
			 Req.minWS.z() = center - radius;
			 Req.maxWS.z() = center + radius;
			 break;

		 case IDC_MIDDLEDELTA_RADIOBUTTON: // center,delta+,delta- option was checked -> calculate min, max values
			 //x values
			 center = Req.minWS.x();
			 delta_plus = Req.maxWS.x(); 
			 delta_minus = m_delta_minus_edit_x_value;
			 Req.minWS.x() = center - delta_minus;
			 Req.maxWS.x() = center + delta_plus;
			 //y values
			 center = Req.minWS.y();
			 delta_plus = Req.maxWS.y(); 
			 delta_minus = m_delta_minus_edit_y_value;
			 Req.minWS.y() = center - delta_minus;
			 Req.maxWS.y() = center + delta_plus;
			 //z values
			 center = Req.minWS.z();
			 delta_plus = Req.maxWS.z(); 
			 delta_minus = m_delta_minus_edit_z_value;
			 Req.minWS.z() = center - delta_minus;
			 Req.maxWS.z() = center + delta_plus;
			 break;

		 default:
			 break;
	}

	// test, which mode is now choosen and set all edit fields to the correspondent values
	switch ( choosen_output_radiobutton_id )
	{
         case IDC_MINMAX_RADIOBUTTON: // min,max option is checked -> no calculations necessary
			 // NOTHING TO DO HERE
			 break;

		 case (IDC_MIDDLERADIUS_RADIOBUTTON): // (center,radius) option or (center, delta+, delta-) option is checked -> calculate values
			 // x values
			 min = Req.minWS.x();
			 max = Req.maxWS.x();
			 // write values suitable for output option
			 Req.minWS.x() = (max-min)/2+min; 
			 Req.maxWS.x() = (max-min)/2; 	 
			 // y values
			 min = Req.minWS.y();
			 max = Req.maxWS.y();
			 // write values suitable for output option
			 Req.minWS.y() = (max-min)/2+min; 
			 Req.maxWS.y() = (max-min)/2; 	
			 // z values
			 min = Req.minWS.z();
			 max = Req.maxWS.z();
			 // write values suitable for output option
			 Req.minWS.z() = (max-min)/2+min; 
			 Req.maxWS.z() = (max-min)/2; 	
			 break;

		 case (IDC_MIDDLEDELTA_RADIOBUTTON): // (center,radius) option or (center, delta+, delta-) option is checked -> calculate values
			 // x values 
			 min = Req.minWS.x();
			 max = Req.maxWS.x();
			 Req.minWS.x() = 0.000; //because there is no definite solution
			 Req.maxWS.x() = max;
			 m_delta_minus_edit_x_value = -min; 
			 // y values
			 min = Req.minWS.y();
			 max = Req.maxWS.y();
			 Req.minWS.y() = 0.000; //because there is no definite solution
			 Req.maxWS.y() = max;
			 m_delta_minus_edit_y_value = -min; 
			 // z values
			 min = Req.minWS.z();
			 max = Req.maxWS.z();
			 Req.minWS.z() = 0.000; //because there is no definite solution
			 Req.maxWS.z() = max;
			 m_delta_minus_edit_z_value = -min; 
			 break;

		 default:
			 break;
	}

	//Get values from the variables into the linked controls
	UpdateData(false);

	// save this state for new conversions later
	m_previouslycheckedRadioButton = GetCheckedRadioButton(IDC_MINMAX_RADIOBUTTON,IDC_MIDDLEDELTA_RADIOBUTTON);
}


void CDescribeApplicationDlg::ActivateDeltaEditFields(bool visible)
{
	// create pointer
	CWnd* pWnd = NULL;
	// find hwnd, suitable for desired edit field and set visible mode
	pWnd = GetDlgItem(IDC_WORKSPACE_XDELTAMINUS_EDIT);
	pWnd->EnableWindow(visible);
    pWnd = GetDlgItem(IDC_WORKSPACE_YDELTAMINUS_EDIT);
	pWnd->EnableWindow(visible);
	pWnd = GetDlgItem(IDC_WORKSPACE_ZDELTAMINUS_EDIT);
	pWnd->EnableWindow(visible);

	pWnd = GetDlgItem(IDC_WORKSPACE_XDELTAMINUS_STATIC);
	pWnd->EnableWindow(visible);
	pWnd = GetDlgItem(IDC_WORKSPACE_YDELTAMINUS_STATIC);
	pWnd->EnableWindow(visible);
	pWnd = GetDlgItem(IDC_WORKSPACE_ZDELTAMINUS_STATIC);
	pWnd->EnableWindow(visible);
}

void CDescribeApplicationDlg::ActiveSimplifiedDialog()
{
    GetDlgItem(IDC_CROSSSECTION_ZMIN_EDIT	   )->EnableWindow(false);
    GetDlgItem(IDC_CROSSSECTION_XMIN_EDIT	   )->EnableWindow(false);
    GetDlgItem(IDC_CROSSSECTION_YMIN_EDIT	   )->EnableWindow(false);
    GetDlgItem(IDC_CROSSSECTION_XMAX_EDIT	   )->EnableWindow(false);
    GetDlgItem(IDC_CROSSSECTION_YMAX_EDIT	   )->EnableWindow(false);
    GetDlgItem(IDC_CROSSSECTION_ZMAX_EDIT	   )->EnableWindow(false);
    GetDlgItem(IDC_ROBOT_MAXACCERLATION_EDIT   )->EnableWindow(false);
    GetDlgItem(IDC_ROBOT_MAXPROCESSFORCE_EDIT  )->EnableWindow(false);
    GetDlgItem(IDC_ROBOT_MAXTORQUE_EDIT		   )->EnableWindow(false);
}

// DescribeApplicationDlg message handlers

BOOL CDescribeApplicationDlg::OnInitDialog()
{
	CDialog::OnInitDialog();
	
	// get the values from the variables into the linked controls
	UpdateData(false);

	// set the min-max-radio button to checked state
	CheckRadioButton(IDC_MINMAX_RADIOBUTTON,IDC_MIDDLEDELTA_RADIOBUTTON,IDC_MINMAX_RADIOBUTTON);

	// set correct integer for calculation method 
	m_previouslycheckedRadioButton = IDC_MINMAX_RADIOBUTTON;

	if (m_bSimplifiedDialog)
		ActiveSimplifiedDialog();

	return TRUE;  // return TRUE unless you set the focus to a controlE
}


void CDescribeApplicationDlg::OnBnClickedOk()
{
	// get the values from the controls into the variables
	// but calculate the correct value
	calculateValues(IDC_MINMAX_RADIOBUTTON);

	CDialog::OnOK();
}


void CDescribeApplicationDlg::OnBnClickedReset()
{
	//get the values from the variables into the controls
	UpdateData(false);

}


void CDescribeApplicationDlg::OnBnClickedRadiobutton()
{
	// test, which radio button is choosen
	int checked_item = GetCheckedRadioButton(IDC_MINMAX_RADIOBUTTON,IDC_MIDDLEDELTA_RADIOBUTTON);

	// test, which radio button is choosen
	switch ( checked_item )
    {
        case IDC_MINMAX_RADIOBUTTON:
			//set correspondent Label texts
			SetDlgItemText(IDC_WORKSPACE_XMIN_STATIC,"x_min [m]:");
			SetDlgItemText(IDC_WORKSPACE_YMIN_STATIC,"y_min [m]:");
			SetDlgItemText(IDC_WORKSPACE_ZMIN_STATIC,"z_min [m]:");
			SetDlgItemText(IDC_WORKSPACE_XMAX_STATIC,"x_max [m]:");
			SetDlgItemText(IDC_WORKSPACE_YMAX_STATIC,"y_max [m]:");
			SetDlgItemText(IDC_WORKSPACE_ZMAX_STATIC,"z_max [m]:");

			//deactivate Editboxes
			ActivateDeltaEditFields(FALSE);
        break;

		case IDC_MIDDLERADIUS_RADIOBUTTON:
			// set correspondent Label texts
			SetDlgItemText(IDC_WORKSPACE_XMIN_STATIC,"x_center [m]:");
			SetDlgItemText(IDC_WORKSPACE_YMIN_STATIC,"y_center [m]:");
			SetDlgItemText(IDC_WORKSPACE_ZMIN_STATIC,"z_center [m]:");
			SetDlgItemText(IDC_WORKSPACE_XMAX_STATIC,"x_radius [m]:");
			SetDlgItemText(IDC_WORKSPACE_YMAX_STATIC,"y_radius [m]:");
			SetDlgItemText(IDC_WORKSPACE_ZMAX_STATIC,"z_radius [m]:");

			// deactivate Editboxes
			ActivateDeltaEditFields(FALSE);
		break;

		case IDC_MIDDLEDELTA_RADIOBUTTON:
			// set correspondent Label texts
			SetDlgItemText(IDC_WORKSPACE_XMIN_STATIC,"x_center [m]:");
			SetDlgItemText(IDC_WORKSPACE_YMIN_STATIC,"y_center [m]:");
			SetDlgItemText(IDC_WORKSPACE_ZMIN_STATIC,"z_center [m]:");
			SetDlgItemText(IDC_WORKSPACE_XMAX_STATIC,"x_delta+ [m]:");
			SetDlgItemText(IDC_WORKSPACE_YMAX_STATIC,"y_delta+ [m]:");
			SetDlgItemText(IDC_WORKSPACE_ZMAX_STATIC,"z_delta+ [m]:");

			// activate Editboxes
			ActivateDeltaEditFields(TRUE);
		break;

        default:
        break;
    }

	// set the values in the right calculation mode
	calculateValues(GetCheckedRadioButton(IDC_MINMAX_RADIOBUTTON,IDC_MIDDLEDELTA_RADIOBUTTON));
}


/////////////////////////////////////////////////////////////
// EditDrumParametersDlg dialog
/////////////////////////////////////////////////////////////

IMPLEMENT_DYNAMIC(CEditDrumParametersDlg, CDialog)

CEditDrumParametersDlg::CEditDrumParametersDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CEditDrumParametersDlg::IDD, pParent)
{
#ifndef _WIN32_WCE
	EnableActiveAccessibility();
#endif
}

CEditDrumParametersDlg::~CEditDrumParametersDlg()
{
}

void CEditDrumParametersDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	
	// better use the get- and set- methods here?
	// reduce strings ?
	DDX_Text(pDX, IDC_WINCH_LENGTH_EDIT, settings.l_drum);
	DDX_Text(pDX, IDC_WINCH_RADIUS_EDIT, settings.r_drum);
	DDX_Text(pDX, IDC_WINCH_WINDINGS_EDIT, settings.n_drum);
	DDX_Text(pDX, IDC_WINCH_FREELENGTH_EDIT, settings.l0);
	DDX_Text(pDX, IDC_WINCH_GEARFACTOR_EDIT, settings.gear_ratio);
	DDX_Text(pDX, IDC_WINCH_APERTUREANGLE_EDIT, settings.aperture);
	DDX_Text(pDX, IDC_WINCH_MAXVELOCITY, settings.v);
	DDX_Text(pDX, IDC_WINCH_MAXACCERATION, settings.a);
	DDX_Text(pDX, IDC_WINCH_MAXFORCEONWIRE_EDIT, settings.f_max);
	DDX_Text(pDX, IDC_WINCH_MAXWIRELENGTH_EDIT, settings.dl_max);
	DDX_Text(pDX, IDC_WINCH_MAXTORQUE_EDIT, settings.M_motor);
	DDX_Text(pDX, IDC_WINCH_MAXANUGLARVELOCITY_EDIT, settings.omega_motor);
	DDX_Text(pDX, IDC_WINCH_MAXANGULARACCERLATION_EDIT, settings.alpha_motor);
	//DDX_Text(pDX, IDC_WINCH_MAXWIRERADIUS_EDIT, // ?? what property is used here?
	//DDX_Text(pDX, IDC_WINCH_YOUNGMODUL_EDIT, //CABLE PROPERTY!
}


BEGIN_MESSAGE_MAP(CEditDrumParametersDlg, CDialog)
	ON_BN_CLICKED(IDOK, &CEditDrumParametersDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDC_RESET_WINCHSETTINGS_BUTTON, &CEditDrumParametersDlg::OnBnClickedReset)
END_MESSAGE_MAP()


// EditDrumParametersDlg message handlers

BOOL CEditDrumParametersDlg::OnInitDialog()
{
	CDialog::OnInitDialog();
	
	// get the values from the variables into the controls
	UpdateData(false);

	return TRUE;  // return TRUE unless you set the focus to a controlE
}


void CEditDrumParametersDlg::OnBnClickedOk()
{
	// save the data
	UpdateData(true);

	CDialog::OnOK();
}


void CEditDrumParametersDlg::OnBnClickedReset()
{
	// get the values from the variables into the controls for resetting
	UpdateData(false);
}


/////////////////////////////////////////////////////////////
// EditWireParametersDlg dialog
/////////////////////////////////////////////////////////////

IMPLEMENT_DYNAMIC(CEditWireParametersDlg, CDialog)

CEditWireParametersDlg::CEditWireParametersDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CEditWireParametersDlg::IDD, pParent)
{
#ifndef _WIN32_WCE
	EnableActiveAccessibility();
#endif
}

CEditWireParametersDlg::~CEditWireParametersDlg()
{
}

void CEditWireParametersDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	
	// better use the get- and set- methods here?
	// reduce stringslengths ?
	DDX_Text(pDX, IDC_WIRE_ELASTICFORCEMODULUS_EDIT, settings.E_wire);
	DDX_Text(pDX, IDC_WIRE_SPECIFICDAMPINGCONSTANT_EDIT, settings.damping);
	DDX_Text(pDX, IDC_WIRE_SPECIFICSPRINGCONSTANT_EDIT, settings.k_spec);
	DDX_Text(pDX, IDC_WIRE_SPECIFICMASSPERLENGTHCONSTANT_EDIT, settings.weight);
	DDX_Text(pDX, IDC_WIRE_RADIUS_EDIT, settings.r_cable);
	DDX_Text(pDX, IDC_WIRE_MAXLENGTH_EDIT, settings.max_length);
	DDX_Text(pDX, IDC_WIRE_MINWIREFORCE_EDIT, settings.minimum_load);
	DDX_Text(pDX, IDC_WIRE_MAXWIREFORCE_EDIT, settings.F_breakingload);
}


BEGIN_MESSAGE_MAP(CEditWireParametersDlg, CDialog)
	ON_BN_CLICKED(IDOK, &CEditWireParametersDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDC_RESET_WIRESETTINGS_BUTTON, &CEditWireParametersDlg::OnBnClickedReset)
END_MESSAGE_MAP()


// EditWireParametersDlg message handlers

BOOL CEditWireParametersDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// do the "backward" - conversion of some units here
	// convert "Young modulus" from N/m� to kN/mm� for better usability
	settings.E_wire = settings.E_wire/1000/1000/1000; 
	// convert "wire radius" from m to mm for further calculations
	settings.r_cable = settings.r_cable*1000;

	// get the values from the variables into the controls
	UpdateData(false);

	return TRUE;  // return TRUE unless you set the focus to a controlE
}


void CEditWireParametersDlg::OnBnClickedOk()
{
	CDialog::OnOK();

	// get the values from the controls
	UpdateData(true);
	
	// do the "forward" - conversion of some units here
	// convert "Young modulus" from kN/mm� to N/m� for further calculations
	settings.E_wire = settings.E_wire*1000*1000*1000;
	// convert "wire radius" from mm to m for further calculations
	settings.r_cable = settings.r_cable/1000;
}


void CEditWireParametersDlg::OnBnClickedReset()
{
	// get the values from the variables into the controls for resetting
	UpdateData(false);
}
