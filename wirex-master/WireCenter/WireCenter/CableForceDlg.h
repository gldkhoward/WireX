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

/*! \file CableForceDlg.h
 *
 *	\author   Andreas Pott
 *
 *  \dependency
 *		WireLib		
 *
 *  This class implements the behaviour of a dialog box that
 *  serves as an assistent to set the minimum and maximum
 *  cable forces.
 */

#pragma once

#include <WireLib/RobotDocument.h>

// CCableForceDlg-Dialogfeld

class CCableForceDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CCableForceDlg)

	PCRL::CRobotDocument* pRoboDoc;
	double l;		// lendth of the considered cable
	double g;		// weight/length of the considered cable

public:	
	// member variables connected to the edits of the dialog box
	double m_dSagMaxConf;
	double m_dSagMaxCalc;
	double m_dSagMaxRef;

	double m_dSagDeltaConf;
	double m_dSagDeltaCalc;
	double m_dSagDeltaRef;

	double m_dSagAlphaConf;
	double m_dSagAlphaCalc;
	double m_dSagAlphaRef;

	double m_dLinearBeginConf;
	double m_dLinearBeginCalc;
	double m_dLinearBeginRef;

	double m_dEigenfrequencyConf;
	double m_dEigenfrequencyCalc;
	double m_dEigenfrequencyRef; 

	double m_dCableBreakConf;
	double m_dCableBreakCalc;
	double m_dCableBreakRef;

	double m_CableFatigueConf;
	double m_CableFatigueCalc;
	double m_CableFatigueRef;

	double m_dLinearEndConf;
	double m_dLinearEndCalc;
	double m_dLinearEndRef;

	double m_dGearboxConf;
	double m_dGearboxCalc;
	double m_dGearboxRef;

	double m_dMotorConf;
	double m_dMotorCalc;
	double m_dMotorRef;

	double m_dWinchConf;
	double m_dWinchCalc;
	double m_dWinchRef;

	double m_dFmin;
	double m_dFmax;
	double m_dRatio;
	double m_dFminMin;
	double m_dFmaxMin;
	double m_dFminMax;
	double m_dFmaxMax;

public:
	CCableForceDlg(PCRL::CRobotDocument& Robot, CWnd* pParent = NULL);   // Standardkonstruktor
	virtual ~CCableForceDlg();

// Dialogfelddaten
	enum { IDD = IDD_CABLEFORCEASSISTANT_DIALOG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV-Unterstützung

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedSagMaxApplyButton();
	afx_msg void OnBnClickedEigenfrequencyApplyButton();
	afx_msg void OnBnClickedSagDeltaApplyButton();
	afx_msg void OnBnClickedSagAlphaApplyButton();
	afx_msg void OnBnClickedLinearbeginApplyButton();
	afx_msg void OnBnClickedCablebreakApplyButton();
	afx_msg void OnBnClickedCableFatigueApplyButton();
	afx_msg void OnBnClickedMotorApplyButton();
	afx_msg void OnBnClickedGearboxApplyButton();
	afx_msg void OnBnClickedWinchApplyButton();
	afx_msg void OnBnClickedLinearendApplyButton();
	afx_msg void OnBnClickedFMinMinApplyButton();
	afx_msg void OnBnClickedFMinMaxApplyButton();
	afx_msg void OnBnClickedFMaxMinApplyButton();
	afx_msg void OnBnClickedFMaxMaxApplyButton();
	afx_msg void OnUpdateComputation();
	afx_msg void OnEnChangeConfEdits();
	virtual BOOL OnInitDialog();
};
