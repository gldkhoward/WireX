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

/*! \file PoseDlg.h
 *
 *	\author   Andreas Pott
 *
 *  \brief The pose dialog or pane class is used as a non-modal dialog box
 *  (a tool box) to set the current pose of the robot. It should
 *  be extended to allow for a more intuitive interaction with
 *  the robot. Forward and inverse kinematics can be calculated.
 *  The dialog starts some calculation concerning the current pose
 *  e.g. stiffness evaluation.
 *
 *  \remark The refactoring towards Windows 7 style with ribbons and panes
 *  made the original implementation of the CPoseDlg depricated. The current
 *  implementation uses CPosePane instead. The class CPoseDlg will be discarded
 *  in future versions but the filename may be a lecacy for a while.
 */

#pragma once
#include <WireLib/EigenLib.h>
#include "afxcmn.h"
#include "UserMessages.h"
#include <WireLib/PosePropertyEvaluation.h>

//! a simple toolbar for the interactive geometry control
class CPoseToolbar : public CMFCToolBar
{
	virtual void OnUpdateCmdUI(CFrameWnd* /*pTarget*/, BOOL bDisableIfNoHndler)
	{
		CMFCToolBar::OnUpdateCmdUI((CFrameWnd*) GetOwner(), bDisableIfNoHndler);
	}

	virtual BOOL AllowShowOnList() const { return FALSE; }
};

//! refactoring of the pose dialog as pane
class CPosePane : public CDockablePane
{
public:
	CPosePane();
	virtual ~CPosePane() {}
	
protected:
	CFont m_fntPropList;
	//! the grid control with the pose properties
	CMFCPropertyGridCtrl m_wndProp;
	//! the specific toolbar for PosePanes
	CPoseToolbar m_wndToolBar;

	//! switch, whether a coninuous update during the movement of the robot is desired
	bool m_bInstantPosePaneUpdate;

	//! the fixed items in the properties window	
	CMFCPropertyGridProperty *pose;
	CMFCPropertyGridProperty *general;
	CMFCPropertyGridProperty *cablelength;		
	CMFCPropertyGridProperty *pulleycablelength;		
	CMFCPropertyGridProperty *orientationWorkspace;
	CMFCPropertyGridProperty *translationWorkspace;
	CMFCPropertyGridProperty *wrenchset;			
	CMFCPropertyGridProperty *stiffness;		
	CMFCPropertyGridProperty *advancedKinematics;
	CMFCPropertyGridProperty *dexterity;			
	CMFCPropertyGridProperty *forcedistribution; 

	//! calculate the layout of the controls after resizing the pane
	void AdjustLayout();

public:
	//! setter function for the properties in the grid control within the pane
	bool SetWireLength(int anz, const double* l);
	bool SetPulleyWireLength(int anz, const double* l, const double* alpha, const double* gamma, const Matrix3d* KAi);
	bool SetOrientationWorkspace(const Vector3d& minAngle, const Vector3d& maxAngle);
	bool SetTranslationalWorkspace(const MatrixXd& ranges);
	bool SetValidCableLength(bool ValidCableLength);
	bool SetPoseComputationTime(int computationTime);
	bool SetPoseParameter(const Vector3d& r, const Matrix3d& R);
	bool SetControllableWorkspce(bool ControllableWorkspace);
	bool SetForceDistribution(int anz, const double* f);
	bool SetWrenchSet(const Vector3d& fmin, const Vector3d& fmax, const Vector3d& Mmin,const Vector3d& Mmax);
	bool SetDexterityMeasure(const double& columnNorm, const double& rowNorm, const double& minSV=0, const double& maxSV=0);
	bool SetStiffness(const double& minStiffness, const double& minTranslationalStiffness);
	bool GetPoseParameter(Vector3d& r, Matrix3d& R);
	//! a set function to show or hide the propertygrid control due to performance issues
	bool SetPropertyGridVisible(const bool bVisible);
	bool SetEvaluator(PCRL::CPosePropertyEvaluation& Evaluator, Vector3d& r, Matrix3d& R);
protected:
	DECLARE_MESSAGE_MAP()
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnSetPose();
	afx_msg void OnUpdateSetPose(CCmdUI *pCmdUI);
	afx_msg void OnAutoUpdatePose();
	afx_msg void OnUpdateAutoUpdatePose(CCmdUI *pCmdUI);
	afx_msg void OnUpdateOptionsAutomaticPoseAnalysis(CCmdUI *pCmdUI);
public:
	virtual BOOL PreTranslateMessage(MSG* pMsg);
};
