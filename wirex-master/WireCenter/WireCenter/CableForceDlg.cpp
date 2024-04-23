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

// CableForceDlg.cpp: Implementierungsdatei
//

#include "stdafx.h"
#include "WireCenter.h"
#include "CableForceDlg.h"
#include "afxdialogex.h"

#include <WireLib/ForceLimits.h>
#include <WireLib/TechKB.h>

// CCableForceDlg-Dialogfeld

IMPLEMENT_DYNAMIC(CCableForceDlg, CDialogEx)

CCableForceDlg::CCableForceDlg(PCRL::CRobotDocument& Robot, CWnd* pParent /*=NULL*/)
	: pRoboDoc(&Robot), CDialogEx(CCableForceDlg::IDD, pParent)
{
	// init the dialog with some more or less meaningful values
	m_dFmin = 0.0;
	m_dFmax = 0.0;      
	m_dRatio = 0.0;    
	m_dSagMaxConf = 0.001;     
	m_dSagMaxCalc = 0.0;     
	m_dSagMaxRef = 0.0;      
	m_dSagDeltaConf = 0.001;   
	m_dSagDeltaCalc = 0.0;   
	m_dSagDeltaRef = 0.0;    
	m_dSagAlphaConf = 0.1;   
	m_dSagAlphaCalc = 0.0;   
	m_dSagAlphaRef = 0.0;    
	m_dLinearBeginConf = 0.0;
	m_dLinearBeginCalc = 0.0;
	m_dLinearBeginRef = 0.0; 
	m_dEigenfrequencyConf = 10.0;
	m_dEigenfrequencyCalc = 0.0;
	m_dEigenfrequencyRef = 0.0; 
	m_dCableBreakConf = 10.0; 
	m_dCableBreakCalc = 0.0; 
	m_dCableBreakRef = 0.0;  
	m_CableFatigueConf = 10.0;
	m_CableFatigueCalc = 0.0;
	m_CableFatigueRef = 0.0; 
	m_dMotorConf = 0.0;      
	m_dMotorCalc = 0.0;      
	m_dMotorRef = 0.0;       
	m_dGearboxConf = 0.0;    
	m_dGearboxCalc = 0.0;    
	m_dGearboxRef = 0.0;     
	m_dWinchConf = 0.0;      
	m_dWinchCalc = 0.0;      
	m_dWinchRef = 0.0;       
	m_dFminMin = 0.0;       
	m_dFminMax = 0.0;       
	m_dFmaxMin = 0.0;       
	m_dFmaxMax = 0.0;       
	m_dLinearEndConf = 0.0;  
	m_dLinearEndCalc = 0.0;  
	m_dLinearEndRef = 0.0;

	// these values are design constants of the robot
	//! \todo we must derive these data from the robot document; however for testing we set them to some constants
	Vector3d bbmin,bbmax,bb;
	pRoboDoc->getBoundingBoxBase(bbmin,bbmax);
	bb=bbmax-bbmin;
	l=bb.norm();
	g = pRoboDoc->currentCable->weight * 9.81;
	// g=0.0035;
}

CCableForceDlg::~CCableForceDlg()
{
}

void CCableForceDlg::DoDataExchange(CDataExchange* pDX)
{
   	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_FMIN_EDIT			   , m_dFmin);
	DDX_Text(pDX, IDC_FMAX_EDIT            , m_dFmax );      
	DDX_Text(pDX, IDC_RATIO_EDIT           , m_dRatio );    
	DDX_Text(pDX, IDC_SAGMAXCONF_EDIT      , m_dSagMaxConf );       
	DDX_Text(pDX, IDC_SAGMAXCALC_EDIT      , m_dSagMaxCalc );       
	DDX_Text(pDX, IDC_SAGMAXREF_EDIT       , m_dSagMaxRef );       
	DDX_Text(pDX, IDC_SAGDELTACONF_EDIT   , m_dSagDeltaConf );       
	DDX_Text(pDX, IDC_SAGDELTACALC_EDIT    , m_dSagDeltaCalc );       
	DDX_Text(pDX, IDC_SEGDELTAREF_EDIT     , m_dSagDeltaRef );       
	DDX_Text(pDX, IDC_SAGALPHACONF_EDIT    , m_dSagAlphaConf );       
	DDX_Text(pDX, IDC_SAGALPHACALC_EDIT    , m_dSagAlphaCalc );       
	DDX_Text(pDX, IDC_SAGALPHAREF_EDIT     , m_dSagAlphaRef );       
	DDX_Text(pDX, IDC_LINEARBEGINCONF_EDIT , m_dLinearBeginConf );       
	DDX_Text(pDX, IDC_LINEARBEGINCALC_EDIT , m_dLinearBeginCalc );       
	DDX_Text(pDX, IDC_LINEARBEGINREF_EDIT  , m_dLinearBeginRef );       
	DDX_Text(pDX, IDC_EIGENFREQUENCYCONF_EDIT , m_dEigenfrequencyConf );       
	DDX_Text(pDX, IDC_EIGENFREQUENCYCALC_EDIT , m_dEigenfrequencyCalc );       
	DDX_Text(pDX, IDC_EIGENFREQUENCYREF_EDIT  , m_dEigenfrequencyRef );       
	DDX_Text(pDX, IDC_CABLEBREAKCONF_EDIT  , m_dCableBreakConf );       
	DDX_Text(pDX, IDC_CABLEBREAKCALC_EDIT  , m_dCableBreakCalc );       
	DDX_Text(pDX, IDC_CALBEBREAKREF_EDIT   , m_dCableBreakRef );       
	DDX_Text(pDX, IDC_CABLEFATIGUECONF_EDIT, m_CableFatigueConf );       
	DDX_Text(pDX, IDC_CABLEFATIGUECALC_EDIT, m_CableFatigueCalc );       
	DDX_Text(pDX, IDC_CABLEFATIGUEREF_EDIT , m_CableFatigueRef );       
	DDX_Text(pDX, IDC_MOTORCONF_EDIT       , m_dMotorConf );       
	DDX_Text(pDX, IDC_MOTORCALC_EDIT       , m_dMotorCalc );       
	DDX_Text(pDX, IDC_MOTORREF_EDIT        , m_dMotorRef );       
	DDX_Text(pDX, IDC_GEARBOXCONF_EDIT     , m_dGearboxConf );       
	DDX_Text(pDX, IDC_GEARBOXCALC_EDIT     , m_dGearboxCalc );       
	DDX_Text(pDX, IDC_GEARBOXREF_EDIT      , m_dGearboxRef );       
	DDX_Text(pDX, IDC_WINCHCONF_EDIT       , m_dWinchConf );       
	DDX_Text(pDX, IDC_WINCHCALC_EDIT       , m_dWinchCalc );       
	DDX_Text(pDX, IDC_WINCHREF_EDIT        , m_dWinchRef );       
	DDX_Text(pDX, IDC_FMINMIN_EDIT         , m_dFminMin );       
	DDX_Text(pDX, IDC_FMINMAX_EDIT         , m_dFminMax );       
	DDX_Text(pDX, IDC_FMAXMIN_EDIT         , m_dFmaxMin );       
	DDX_Text(pDX, IDC_FMAXMAX_EDIT         , m_dFmaxMax );       
	DDX_Text(pDX, IDC_LINEARENDCONF_EDIT   , m_dLinearEndConf );       
	DDX_Text(pDX, IDC_LINEARENDCALC_EDIT   , m_dLinearEndCalc );       
	DDX_Text(pDX, IDC_LINEARENDREF_EDIT    , m_dLinearEndRef );
	// ** unhandled items: **
	// IDC_CABLE_COMBO                 
	// IDC_MOTOR_COMBO                 
	// IDC_GEARBOX_COMBO               
	// IDC_WINCH_COMBO  
}

BEGIN_MESSAGE_MAP(CCableForceDlg, CDialogEx)
	ON_BN_CLICKED(IDC_SAGMAXAPPLY_BUTTON, &CCableForceDlg::OnBnClickedSagMaxApplyButton)
	ON_BN_CLICKED(IDC_SAGDELTAAPPLY_BUTTON, &CCableForceDlg::OnBnClickedSagDeltaApplyButton)
	ON_BN_CLICKED(IDC_SAGALPHAAPPLY_BUTTON, &CCableForceDlg::OnBnClickedSagAlphaApplyButton)
	ON_BN_CLICKED(IDC_LINEARBEGINAPPLY_BUTTON, &CCableForceDlg::OnBnClickedLinearbeginApplyButton)
	ON_BN_CLICKED(IDC_EIGENFREQUENCYAPPLY_BUTTON, &CCableForceDlg::OnBnClickedEigenfrequencyApplyButton)
	ON_BN_CLICKED(IDC_CABLEBREAKAPPLY_BUTTON, &CCableForceDlg::OnBnClickedCablebreakApplyButton)
	ON_BN_CLICKED(IDC_CABLEFATIGUEAPPLY_BUTTON, &CCableForceDlg::OnBnClickedCableFatigueApplyButton)
	ON_BN_CLICKED(IDC_MOTORAPPLY_BUTTON, &CCableForceDlg::OnBnClickedMotorApplyButton)
	ON_BN_CLICKED(IDC_GEARBOXAPPLY_BUTTON, &CCableForceDlg::OnBnClickedGearboxApplyButton)
	ON_BN_CLICKED(IDC_WINCHAPPLY_BUTTON, &CCableForceDlg::OnBnClickedWinchApplyButton)
	ON_BN_CLICKED(IDC_LINEARENDAPPLY_BUTTON, &CCableForceDlg::OnBnClickedLinearendApplyButton)
	ON_BN_CLICKED(IDC_FMINMINAPPLY_BUTTON, &CCableForceDlg::OnBnClickedFMinMinApplyButton)
	ON_BN_CLICKED(IDC_FMINMAXAPPLY_BUTTON, &CCableForceDlg::OnBnClickedFMinMaxApplyButton)
	ON_BN_CLICKED(IDC_FMAXMINAPPLY_BUTTON, &CCableForceDlg::OnBnClickedFMaxMinApplyButton)
	ON_BN_CLICKED(IDC_FMAXMAXAPPLY_BUTTON, &CCableForceDlg::OnBnClickedFMaxMaxApplyButton)
	ON_EN_KILLFOCUS(IDC_FMIN_EDIT, &CCableForceDlg::OnEnChangeConfEdits)
	ON_EN_KILLFOCUS(IDC_FMAX_EDIT, &CCableForceDlg::OnEnChangeConfEdits)
	ON_EN_KILLFOCUS(IDC_SAGMAXCONF_EDIT, &CCableForceDlg::OnEnChangeConfEdits)
	ON_EN_KILLFOCUS(IDC_SAGDELTACONF_EDIT, &CCableForceDlg::OnEnChangeConfEdits)
	ON_EN_KILLFOCUS(IDC_SAGALPHACONF_EDIT, &CCableForceDlg::OnEnChangeConfEdits)
	ON_EN_KILLFOCUS(IDC_LINEARBEGINCONF_EDIT, &CCableForceDlg::OnEnChangeConfEdits)
	ON_EN_KILLFOCUS(IDC_EIGENFREQUENCYCONF_EDIT, &CCableForceDlg::OnEnChangeConfEdits)
	ON_EN_KILLFOCUS(IDC_CABLEBREAKCONF_EDIT, &CCableForceDlg::OnEnChangeConfEdits)
	ON_EN_KILLFOCUS(IDC_CABLEFATIGUECONF_EDIT, &CCableForceDlg::OnEnChangeConfEdits)
	ON_EN_KILLFOCUS(IDC_MOTORCONF_EDIT, &CCableForceDlg::OnEnChangeConfEdits)
	ON_EN_KILLFOCUS(IDC_GEARBOXCONF_EDIT, &CCableForceDlg::OnEnChangeConfEdits)
	ON_EN_KILLFOCUS(IDC_WINCHCONF_EDIT, &CCableForceDlg::OnEnChangeConfEdits)
	ON_EN_KILLFOCUS(IDC_LINEARENDCONF_EDIT, &CCableForceDlg::OnEnChangeConfEdits)
	ON_CBN_SELCHANGE(IDC_GEARBOX_COMBO, &CCableForceDlg::OnEnChangeConfEdits)
	ON_CBN_SELCHANGE(IDC_MOTOR_COMBO, &CCableForceDlg::OnEnChangeConfEdits)
END_MESSAGE_MAP()


// CCableForceDlg-Meldungshandler

// code for all apply buttons; we simply copy the respective fXXXmin/fXXXmax value to the general fmin/fmax control
void CCableForceDlg::OnBnClickedSagMaxApplyButton()			{ UpdateData(TRUE);	m_dFmin = m_dSagMaxCalc; UpdateData(FALSE);	}
void CCableForceDlg::OnBnClickedSagDeltaApplyButton()		{ UpdateData(TRUE); m_dFmin = m_dSagDeltaCalc; UpdateData(FALSE); }
void CCableForceDlg::OnBnClickedSagAlphaApplyButton()		{ UpdateData(TRUE); m_dFmin = m_dSagAlphaCalc; UpdateData(FALSE); }
void CCableForceDlg::OnBnClickedLinearbeginApplyButton()	{ UpdateData(TRUE); m_dFmin = m_dLinearBeginCalc; UpdateData(FALSE); }
void CCableForceDlg::OnBnClickedEigenfrequencyApplyButton() { UpdateData(TRUE);	m_dFmin = m_dEigenfrequencyCalc; UpdateData(FALSE);	}
void CCableForceDlg::OnBnClickedCablebreakApplyButton()		{ UpdateData(TRUE); m_dFmax = m_dCableBreakCalc; UpdateData(FALSE); }
void CCableForceDlg::OnBnClickedCableFatigueApplyButton()	{ UpdateData(TRUE); m_dFmax = m_dEigenfrequencyCalc; UpdateData(FALSE); }
void CCableForceDlg::OnBnClickedMotorApplyButton()			{ UpdateData(TRUE); m_dFmax = m_dMotorCalc; UpdateData(FALSE); }
void CCableForceDlg::OnBnClickedGearboxApplyButton()		{ UpdateData(TRUE); m_dFmax = m_dGearboxCalc; UpdateData(FALSE); }
void CCableForceDlg::OnBnClickedWinchApplyButton()			{ UpdateData(TRUE); m_dFmax = m_dWinchCalc; UpdateData(FALSE); }
void CCableForceDlg::OnBnClickedLinearendApplyButton()		{ UpdateData(TRUE); m_dFmax = m_dLinearEndCalc; UpdateData(FALSE); }
void CCableForceDlg::OnBnClickedFMinMinApplyButton()		{ UpdateData(TRUE); m_dFmin = m_dFminMin; UpdateData(FALSE); }
void CCableForceDlg::OnBnClickedFMinMaxApplyButton()		{ UpdateData(TRUE); m_dFmin = m_dFminMax; UpdateData(FALSE); }
void CCableForceDlg::OnBnClickedFMaxMinApplyButton()		{ UpdateData(TRUE); m_dFmax = m_dFmaxMin; UpdateData(FALSE); }
void CCableForceDlg::OnBnClickedFMaxMaxApplyButton()		{ UpdateData(TRUE); m_dFmax = m_dFmaxMax; UpdateData(FALSE); }

//! we can have a generic update function that is called whenever an edit field is changed
//! the computations are all very fast, therefore we update all of them
void CCableForceDlg::OnUpdateComputation()
{
	UpdateData(TRUE);		// download the latest values from the dialog box to the local variables
	
	// update the calculated sagging
	PCRL::CForceLimits limits(*pRoboDoc);
	m_dSagMaxCalc = limits.getMinForceSag(l, g, m_dSagMaxConf);

	// update the sagging for the actual fmin
	m_dSagMaxRef = PCRL::CCableSagging::getMaxSag(m_dFmin, l, g);

	// update the calcuated elongation
	m_dSagDeltaCalc = limits.getMinForceSaggingContraction(l, g,m_dSagDeltaConf);

	// update the elongation for the actual fmin
	m_dSagDeltaRef = PCRL::CCableSagging::getMaxElongation(m_dFmin, l, g);

	// update the calcuated force for eigenfrequency
	m_dEigenfrequencyCalc = limits.getMinForceEigenfrequency(l,g, m_dEigenfrequencyConf);
	m_dEigenfrequencyRef = limits.getEigenfrequency(l,g,m_dFmin);

	// update the breaking load from the data sheet
	m_dCableBreakCalc = pRoboDoc->currentCable->F_breakingload / m_dCableBreakConf;
	m_dCableBreakRef = pRoboDoc->currentCable->F_breakingload / m_dFmax;

	// update the ratio and the min/max fields
	m_dFminMin = min(min(min(m_dSagMaxCalc,m_dSagDeltaCalc),min(m_dSagAlphaCalc,m_dEigenfrequencyCalc)),m_dLinearBeginCalc);
	m_dFminMax = max(max(max(m_dSagMaxCalc,m_dSagDeltaCalc),max(m_dSagAlphaCalc,m_dEigenfrequencyCalc)),m_dLinearBeginCalc);
	m_dFmaxMin = min(min(min(m_dCableBreakCalc,m_CableFatigueCalc),min(m_dMotorCalc,m_dGearboxCalc)),min(m_dWinchCalc,m_dLinearEndCalc));
	m_dFmaxMax = max(max(max(m_dCableBreakCalc,m_CableFatigueCalc),max(m_dMotorCalc,m_dGearboxCalc)),max(m_dWinchCalc,m_dLinearEndCalc));
	m_dRatio = m_dFmax / m_dFmin;

	// use the values of the currently selected motor
	CComboBox* pCB = (CComboBox*)GetDlgItem(IDC_MOTOR_COMBO);
	int curselmotor = pCB->GetCurSel();
	pCB = (CComboBox*)GetDlgItem(IDC_GEARBOX_COMBO);
	int curselgearbox = pCB->GetCurSel();
	m_dMotorConf = MotorParameterKB[curselmotor].M0;
	m_dMotorCalc = MotorParameterKB[curselmotor].M0 * GearboxParameterKB[curselgearbox].i / pRoboDoc->currentWinch->r_drum;
	m_dGearboxConf = GearboxParameterKB[curselgearbox].Mout100;
	m_dGearboxCalc = GearboxParameterKB[curselgearbox].Mout100 / pRoboDoc->currentWinch->r_drum;

	m_dMotorRef = m_dFmax * pRoboDoc->currentWinch->r_drum;
	m_dGearboxRef = m_dFmax * pRoboDoc->currentWinch->r_drum;

	// update the winch configuration text
	CStatic* pStatic = (CStatic*)GetDlgItem(IDC_FMAXCONFIG_STATIC);
	CString str;
	str.Format("Motor: %s\nGearbox: %s\nWinch: N.A.",
		MotorParameterKB[curselmotor].name,
		GearboxParameterKB[curselgearbox].name);
	pStatic->SetWindowTextA(str);

	// update the robot config text
	pStatic = (CStatic*)GetDlgItem(IDC_ROBOTCABLECONF_STATIC);
	str.Format("ref. cable length: %3.3f [m]\ncable weight: %3.5f",
		l,
		g);
	pStatic->SetWindowTextA(str);

	UpdateData(FALSE);		// upload the modifications to the dialog box
}

//! \todo only a proxy function; perhaps we put all the code here later
void CCableForceDlg::OnEnChangeConfEdits()
{ OnUpdateComputation(); }


BOOL CCableForceDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  hier zusätzliche Initialisierung hinzufügen.

	// fill the gearbox combo boxes with the desired values
	CComboBox* pCB = (CComboBox*)GetDlgItem(IDC_GEARBOX_COMBO);
	if (!pCB)
		MessageBox("Combo Box does not work.");
	else
	{
		for (int i=0; i<sizeof(GearboxParameterKB)/sizeof(TGearboxParameter); i++)
		{
			CString str;
			str.Format("M=%3.2f Nm (%s)",GearboxParameterKB[i].Mout100, GearboxParameterKB[i].name);
			pCB->AddString(str);
			pCB->SetCurSel(0);
		}
	}

	// fill the gearbox combo boxes with the desired values
	pCB = (CComboBox*)GetDlgItem(IDC_MOTOR_COMBO);
	if (!pCB)
		MessageBox("Combo Box does not work.");
	else
	{
		for (int i=0; i<sizeof(MotorParameterKB)/sizeof(TMotorParameter); i++)
		{
			CString str;
			str.Format("M=%3.2f Nm (%s)",MotorParameterKB[i].M0, MotorParameterKB[i].name);
			pCB->AddString(str);
		}
		pCB->SetCurSel(0);
	}

	OnUpdateComputation();

	return TRUE;  // return TRUE unless you set the focus to a control
	// AUSNAHME: OCX-Eigenschaftenseite muss FALSE zurückgeben.
}
