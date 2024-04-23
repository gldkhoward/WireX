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

// PoseDlg.cpp: Implementierungsdatei
//

#include "stdafx.h"
#include "WireCenter.h"
#include "PoseDlg.h"
#include "WireCenterDoc.h"
#include "WireCenterView.h"

// IMPLEMENTATION OF CPosePane
/////////////////////////////////

BEGIN_MESSAGE_MAP(CPosePane, CDockablePane)
	ON_WM_CREATE()
	ON_WM_SIZE()

	ON_COMMAND(ID_POSEPANE_SET_BUTTON, &CPosePane::OnSetPose)
	ON_UPDATE_COMMAND_UI(ID_POSEPANE_SET_BUTTON, &CPosePane::OnUpdateSetPose)
	ON_COMMAND(ID_POSEPANE_AUTOUPDATE_BUTTON, &CPosePane::OnAutoUpdatePose)
	ON_UPDATE_COMMAND_UI(ID_POSEPANE_AUTOUPDATE_BUTTON, &CPosePane::OnUpdateAutoUpdatePose)
	ON_UPDATE_COMMAND_UI(ID_BERECHNEN_AUTOMATISCHEPOSENANALYSEOPTIONEN, &CPosePane::OnUpdateOptionsAutomaticPoseAnalysis)
END_MESSAGE_MAP()

CPosePane::CPosePane()
{
	pose = new CMFCPropertyGridProperty(_T("Pose"));
	general = new CMFCPropertyGridProperty(_T("General"));
	cablelength = new CMFCPropertyGridProperty(_T("Cable Length"));		
	pulleycablelength = new CMFCPropertyGridProperty(_T("Kinematics (Pulley)"));		
	orientationWorkspace= new CMFCPropertyGridProperty(_T("Available Orientations"));		
	translationWorkspace= new CMFCPropertyGridProperty(_T("Available Translations"));		
	wrenchset = new CMFCPropertyGridProperty(_T("Available wrenches"));			
	stiffness = new CMFCPropertyGridProperty(_T("Stiffness"));		
	advancedKinematics = new CMFCPropertyGridProperty(_T("Kinematics"));
	dexterity = new CMFCPropertyGridProperty(_T("Dexterity Index"));			
	forcedistribution = new CMFCPropertyGridProperty(_T("Force Distributions"));
}

void CPosePane::AdjustLayout()
{
	if (GetSafeHwnd() == NULL) return;
	CRect rectClient;
	GetClientRect(rectClient);
	int cyTlb = m_wndToolBar.CalcFixedLayout(FALSE,TRUE).cy;
	int dy = cyTlb;

	// we still have to add some spacing between the controls
	m_wndToolBar.SetWindowPos(NULL,rectClient.left,rectClient.top,rectClient.Width(),cyTlb,SWP_NOACTIVATE | SWP_NOZORDER);
	m_wndProp.SetWindowPos(NULL,rectClient.left,rectClient.top+dy,rectClient.Width(),rectClient.Height()-dy,SWP_NOACTIVATE | SWP_NOZORDER);
}

int CPosePane::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CDockablePane::OnCreate(lpCreateStruct) == -1)
		return -1;

	CRect rectDummy;
	rectDummy.SetRectEmpty();

	// create my toolbar
	m_wndToolBar.Create(this, AFX_DEFAULT_TOOLBAR_STYLE, IDR_POSE_TOOLBAR);
	m_wndToolBar.LoadToolBar(IDR_POSE_TOOLBAR, 0, 0, TRUE /* Ist gesperrt */);
	m_wndToolBar.SetPaneStyle(m_wndToolBar.GetPaneStyle() | CBRS_TOOLTIPS | CBRS_FLYBY);
	m_wndToolBar.SetOwner(this);
	// Alle Befehle werden über dieses Steuerelement geleitet, nicht über den übergeordneten Rahmen:
	m_wndToolBar.SetRouteCommandsViaFrame(FALSE);

	if (!m_wndProp.Create(WS_CHILD | WS_VISIBLE | WS_BORDER | WS_CLIPSIBLINGS | WS_CLIPCHILDREN,rectDummy,this,2))
	{
		printf("Error while creating the property grid control\n");
		return -1;
	}
	// now we do some fancy stuff with the property grid's font 
	m_wndProp.SetVSDotNetLook(TRUE);
	m_wndProp.SetGroupNameFullWidth(TRUE);

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

	m_wndProp.SetFont(&m_fntPropList);

	// that's all done; we set the font to something nice
	m_wndProp.EnableHeaderCtrl(TRUE);
	m_wndProp.EnableDescriptionArea();
	m_wndProp.SetVSDotNetLook();
	m_wndProp.MarkModifiedProperties();
	
	bool flag = false;
	general->AddSubItem(new CMFCPropertyGridProperty(_T("Workspace"),(_variant_t)flag,
		_T("Indicates if the current pose belongs to the wrench-feasbile workspace"),0));
	general->AddSubItem(new CMFCPropertyGridProperty(_T("Cable Length"),(_variant_t)flag,
		_T("Is the pose reachable, i.e. are the current cable length valid"),0));
	general->AddSubItem(new CMFCPropertyGridProperty(_T("Calcuation Time"),(_variant_t)0,
		_T("Computation time for the determination of all pose properties"),0));

	// disables InstantPosePaneUpdate at the beginning
	m_bInstantPosePaneUpdate = false;

	// add the entries of the grid property; firstly we insert a couple of main categories here
	m_wndProp.AddProperty(pose);
	m_wndProp.AddProperty(general);
	m_wndProp.AddProperty(cablelength);	
	// the following properties have dynamic conterparts based on the (newer) PoseProperty implementation
	// therefore, we deactivate the proerties here without removing the properties from this class
	// \todo Warning: Not adding the properties may fool the dynamic memory management as the w_wndProp would
	//   delete its children where the implementation of this class does not. Therefore, the current solution
	//   is temporarily accaptable but should be revised later.
//	m_wndProp.AddProperty(pulleycablelength);		
//	m_wndProp.AddProperty(orientationWorkspace);		
//	m_wndProp.AddProperty(translationWorkspace);		
//	m_wndProp.AddProperty(wrenchset);			
	m_wndProp.AddProperty(forcedistribution);
//	m_wndProp.AddProperty(stiffness);		
//	m_wndProp.AddProperty(dexterity);			
//	m_wndProp.AddProperty(advancedKinematics); // we skip this one as long as no data is available and delete the created property again to avoid heap corruption
	delete advancedKinematics;

	// set some more or less useful initial parameters
	Matrix3d zero = zero.setZero();
	SetPoseParameter(Vector3d(0,0,0),zero);
	SetWrenchSet(Vector3d(0,0,0),Vector3d(0,0,0),Vector3d(0,0,0),Vector3d(0,0,0));
	SetDexterityMeasure(-1,-1);
	SetOrientationWorkspace(Vector3d(0,0,0),Vector3d(0,0,0));
	MatrixXd ranges(6,1);
	SetTranslationalWorkspace(ranges);
	double l[8] = {0,0,0,0,0,0,0,0};
	SetWireLength(8,l);
	SetPulleyWireLength(8,l,l,l,0);
	for (int i=0; i<8; i++)
		l[i] *= i;
	SetForceDistribution(8,l);
	SetStiffness(1,2);
	AdjustLayout();
	return 0;
}

void CPosePane::OnSize(UINT nType, int cx, int cy)
{
	CDockablePane::OnSize(nType,cx,cy);
//	this->Invalidate();
	AdjustLayout();
}

bool CPosePane::SetPoseParameter(const Vector3d& r, const Matrix3d& R)
{
	// retrieve the mapping from the rotation matrix on every axis
	Vector3d vec;
	vec = R.eulerAngles(0,1,2);

	if (pose->GetSubItemsCount() > 0)
	{
		pose->GetSubItem(0)->SetValue(r.x());
		pose->GetSubItem(1)->SetValue(r.y());
		pose->GetSubItem(2)->SetValue(r.z());

		pose->GetSubItem(3)->SetValue(vec.x()*RAD_TO_DEG);
		pose->GetSubItem(4)->SetValue(vec.y()*RAD_TO_DEG);
		pose->GetSubItem(5)->SetValue(vec.z()*RAD_TO_DEG);
	}
	else
	{
		// add the controls to set to position and orientation of the platform
		pose->AddSubItem(new CMFCPropertyGridProperty(_T("x"),(_variant_t)r.x(), _T("X-coordinate of the platform")));
		pose->AddSubItem(new CMFCPropertyGridProperty(_T("y"),(_variant_t)r.y(), _T("Y-coordinate of the platform")));
		pose->AddSubItem(new CMFCPropertyGridProperty(_T("z"),(_variant_t)r.z(), _T("Z-coordinate of the platform")));
		pose->AddSubItem(new CMFCPropertyGridProperty(_T("a"),vec.x()*RAD_TO_DEG,_T("A-coordinate of the platform")));
		pose->AddSubItem(new CMFCPropertyGridProperty(_T("b"),vec.y()*RAD_TO_DEG,_T("B-coordinate of the platform")));
		pose->AddSubItem(new CMFCPropertyGridProperty(_T("c"),vec.z()*RAD_TO_DEG,_T("C-coordinate of the platform")));
		// add a control to set the motion pattern of the platform; this is experimental
		CMFCPropertyGridProperty *pGP = new CMFCPropertyGridProperty(_T("Motion Pattern"),(_variant_t)_T("3R3T"),_T("Motion pattern of the robot"),0,0,0,_T("123RT"));
		pGP->AddOption(_T("2T"));
		pGP->AddOption(_T("1R2T"));
		pGP->AddOption(_T("3T"));
		pGP->AddOption(_T("2R3T"));
		pGP->AddOption(_T("3R3T"));
		pGP->AllowEdit(FALSE);
		pose->AddSubItem(pGP);
	}
	return true;
}

//! fill or update the property grid with the given wire length l
bool CPosePane::SetWireLength(int anz, const double* l)
{
	if (!l)
		return false;
	if (anz<1)
		return false;	
	// firstly, we adjust the number of items to the number of cables
	// delete items if we have to many
	while (cablelength->GetSubItemsCount()>anz)
	{
		CMFCPropertyGridProperty* pProp = cablelength->GetSubItem(0);
		cablelength->RemoveSubItem(pProp);
	}
	int i = 0;
	double dummy = 0.0;
	// add items if we do not have enough space in the box
	while (cablelength->GetSubItemsCount()<anz)
		cablelength->AddSubItem(new CMFCPropertyGridProperty(_T(""),dummy,_T("cable length"),0));
	// now we can set the data in the property list named cablelength
	CString str;
	for (int i=0; i<anz; i++)
	{
		str.Format("l%i",i+1);
		cablelength->GetSubItem(i)->SetName(str,TRUE);
		cablelength->GetSubItem(i)->SetValue((double)l[i]);
	}
	return true;
}

//! fill or update the property grid with the parameters
//! from pulley inverse kinematics
bool CPosePane::SetPulleyWireLength(int anz, const double* l, const double* beta, const double* gamma, const Matrix3d* KAi)
{
	// check the parameter for correctness
	if (!l || !beta || !gamma)
		return false;
	if (anz<1)
		return false;
	// we compute the required number of elements in the property grid
	const int groupsize = 5; // we have for each cable: cable length, beta, gamma, and two normal vectors 
	int items = anz*groupsize; 

	// firstly, we adjust the number of items to the number of cables
	// delete items if we have to many
	while (pulleycablelength->GetSubItemsCount()>items)
	{
		// grow and shrink the properties in chunks of groupsize
		for (int i=0; i<groupsize; ++i)
		{
			CMFCPropertyGridProperty* pProp = pulleycablelength->GetSubItem(0);
			pulleycablelength->RemoveSubItem(pProp);
		}
	}
	int i = 0;
	double dummy = 0.0;
	// add items if we do not have enough space in the box
	while (pulleycablelength->GetSubItemsCount()<items)
	{
		// again, we add only chunks corresponding to groupsize and with the desired type(!), here double and string
		CString str("");
		pulleycablelength->AddSubItem(new CMFCPropertyGridProperty(_T(""),dummy,_T(""),0));
		pulleycablelength->AddSubItem(new CMFCPropertyGridProperty(_T(""),dummy,_T(""),0));
		pulleycablelength->AddSubItem(new CMFCPropertyGridProperty(_T(""),dummy,_T(""),0));
		pulleycablelength->AddSubItem(new CMFCPropertyGridProperty(_T(""),str,_T(""),0));
		pulleycablelength->AddSubItem(new CMFCPropertyGridProperty(_T(""),str,_T(""),0));
	}
	// now we can set the data in the property list named pulleycablelength
	// we copy the geometry related properties from directly from CRobotDocument
	// this is easy but not exactly in line with the API approach of this class
	// which takes most of the other data through setter functions 
	CWireCenterView* pView = CWireCenterView::This;
	CString str;
	for (int i=0; i<anz; i++)
	{
		// format cable length
		str.Format("l%i",i+1);
		pulleycablelength->GetSubItem(i*5)->SetName(str,TRUE);
		pulleycablelength->GetSubItem(i*5)->SetValue((double)l[i]);
		// format cable beta
		str.Format("beta%i [°]",i+1);
		pulleycablelength->GetSubItem(i*5+1)->SetName(str,TRUE);
		pulleycablelength->GetSubItem(i*5+1)->SetValue((double)beta[i]*RAD_TO_DEG);
		// format cable gamma
		str.Format("gamma%i [°]",i+1);
		pulleycablelength->GetSubItem(i*5+2)->SetName(str,TRUE);
		pulleycablelength->GetSubItem(i*5+2)->SetValue((double)gamma[i]*RAD_TO_DEG);
		// format cable z-axis
		Matrix3d K = pView->GetRobotDoc()->getBaseOrientation(i);
		str.Format("z-axis%i",i+1);
		pulleycablelength->GetSubItem(i*5+3)->SetName(str,TRUE);
		str.Format("(%1.3f, %1.3f, %1.3f)",K.col(2).x(),K.col(2).y(),K.col(2).z());
		pulleycablelength->GetSubItem(i*5+3)->SetValue(str);
		// format cable x-axis
		str.Format("x-axis%i",i+1);
		pulleycablelength->GetSubItem(i*5+4)->SetName(str,TRUE);
		str.Format("(%1.3f, %1.3f, %1.3f)",K.col(0).x(),K.col(0).y(),K.col(0).z());
		pulleycablelength->GetSubItem(i*5+4)->SetValue(str);
	}
	return true;
}

bool CPosePane::SetValidCableLength(bool ValidCableLength) 
{
	if (general->GetSubItemsCount()>0)
		general->GetSubItem(1)->SetValue((_variant_t)ValidCableLength);
	return true; 
}

bool CPosePane::SetControllableWorkspce(bool ControllableWorkspace) 
{
	if (general->GetSubItemsCount()>0)
		general->GetSubItem(0)->SetValue((_variant_t)ControllableWorkspace);
	return true; 
}

bool CPosePane::SetPoseComputationTime(int computationTime)
{
	if (general->GetSubItemsCount()>0)
		general->GetSubItem(2)->SetValue((_variant_t)computationTime);
	return true; 
}

bool CPosePane::SetForceDistribution(int anz, const double* f) 
{
	if (!f)
		return false;
	if (anz<1)
		return false;	
	// firstly, we adjust the number of items to the number of cables
	// delete items if we have to many
	while (forcedistribution->GetSubItemsCount()>anz)
	{
		CMFCPropertyGridProperty* pProp = forcedistribution->GetSubItem(0);
		forcedistribution->RemoveSubItem(pProp);
	}
	int i = 0;
	double dummy = 0.0f;
	// add items if we do not have enough space in the box
	while (forcedistribution->GetSubItemsCount()<anz)
		forcedistribution->AddSubItem(new CMFCPropertyGridProperty(_T(""),dummy,_T("force distribution"),0));
	// now we can set the data in the property list named forcedistribution
	CString str;
	for (int i=0; i<anz; i++)
	{
		str.Format("f%i",i+1);
		forcedistribution->GetSubItem(i)->SetName(str);
		str.Format("%6.3f",f[i]);
		forcedistribution->GetSubItem(i)->SetValue((_variant_t)f[i]);
	}
	return true;
}

bool CPosePane::SetOrientationWorkspace(const Vector3d& minAngle, const Vector3d& maxAngle)
{
	// set or modify the elements in the treeview
	CString str;
	if (orientationWorkspace->GetSubItemsCount() > 0)
	{
		// there are already some items; we have to modify them
		str.Format("%6.3f -- %6.3f",minAngle.x(),maxAngle.x()); orientationWorkspace->GetSubItem(0)->SetValue((_variant_t)str);
		str.Format("%6.3f -- %6.3f",minAngle.y(),maxAngle.y()); orientationWorkspace->GetSubItem(1)->SetValue((_variant_t)str);
		str.Format("%6.3f -- %6.3f",minAngle.z(),maxAngle.z()); orientationWorkspace->GetSubItem(2)->SetValue((_variant_t)str);
	}
	else
	{
		// no items in property forcedistribution present; we have to insert new items
		str.Format("%6.3f -- %6.3f",minAngle.x(),maxAngle.x()); orientationWorkspace->AddSubItem(new CMFCPropertyGridProperty(_T("alpha"),(_variant_t)str,_T("Possible rotations around the x-axis")));
		str.Format("%6.3f -- %6.3f",minAngle.y(),maxAngle.y()); orientationWorkspace->AddSubItem(new CMFCPropertyGridProperty(_T("beta"), (_variant_t)str,_T("Possible rotations around the y-axis")));
		str.Format("%6.3f -- %6.3f",minAngle.z(),maxAngle.z()); orientationWorkspace->AddSubItem(new CMFCPropertyGridProperty(_T("gamma"),(_variant_t)str,_T("Possible rotations around the z-axis")));
	}
	return true;
}

bool CPosePane::SetTranslationalWorkspace(const MatrixXd& ranges)
{
 	if (ranges.rows()<6)
		return false;
	// set or modify the elements in the treeview
	CString str;
	if (translationWorkspace->GetSubItemsCount() > 0)
	{
		// there are already some items; we have to modify them
		str.Format("%6.3f -- %6.3f",ranges(3),ranges(0)); translationWorkspace->GetSubItem(0)->SetValue((_variant_t)str);
		str.Format("%6.3f -- %6.3f",ranges(4),ranges(1)); translationWorkspace->GetSubItem(1)->SetValue((_variant_t)str);
		str.Format("%6.3f -- %6.3f",ranges(5),ranges(2)); translationWorkspace->GetSubItem(2)->SetValue((_variant_t)str);
	}
	else
	{
		// no items in property forcedistribution present; we have to insert new items
		str.Format("%6.3f -- %6.3f",ranges(3),ranges(0)); translationWorkspace->AddSubItem(new CMFCPropertyGridProperty(_T("delta x"),(_variant_t)str,_T("available relative translation in x-direction (absolute values in negative and positive direction)")));
		str.Format("%6.3f -- %6.3f",ranges(4),ranges(1)); translationWorkspace->AddSubItem(new CMFCPropertyGridProperty(_T("delta y"),(_variant_t)str,_T("available relative translation in y-direction (absolute values in negative and positive direction)")));
		str.Format("%6.3f -- %6.3f",ranges(5),ranges(2)); translationWorkspace->AddSubItem(new CMFCPropertyGridProperty(_T("delta z"),(_variant_t)str,_T("available relative translation in z-direction (absolute values in negative and positive direction)")));
	}
	return true;
}

bool CPosePane::SetWrenchSet(const Vector3d& fmin, const Vector3d& fmax, const Vector3d& Mmin,const Vector3d& Mmax) 
{
	// set or modify the elements in the treeview
	CString str;
	
	if (wrenchset->GetSubItemsCount() > 0)
	{
		// there are already some items; we have to modify them
		str.Format("%6.3f -- %6.3f",fmin.x(),fmax.x()); wrenchset->GetSubItem(0)->SetValue((_variant_t)str);
		str.Format("%6.3f -- %6.3f",fmin.y(),fmax.y()); wrenchset->GetSubItem(1)->SetValue((_variant_t)str);
		str.Format("%6.3f -- %6.3f",fmin.z(),fmax.z()); wrenchset->GetSubItem(2)->SetValue((_variant_t)str);
		str.Format("%6.3f -- %6.3f",Mmin.x(),Mmax.x()); wrenchset->GetSubItem(3)->SetValue((_variant_t)str);
		str.Format("%6.3f -- %6.3f",Mmin.y(),Mmax.y()); wrenchset->GetSubItem(4)->SetValue((_variant_t)str);
		str.Format("%6.3f -- %6.3f",Mmin.z(),Mmax.z()); wrenchset->GetSubItem(5)->SetValue((_variant_t)str);
	}
	else
	{
		// no items in property forcedistribution present; we have to insert new items
		str.Format("%6.3f -- %6.3f",fmin.x(),fmax.x()); wrenchset->AddSubItem(new CMFCPropertyGridProperty(_T("Fx"),(_variant_t)str,_T("Available force in x-directions")));
		str.Format("%6.3f -- %6.3f",fmin.y(),fmax.y()); wrenchset->AddSubItem(new CMFCPropertyGridProperty(_T("Fy"),(_variant_t)str,_T("Available force in y-directions")));
		str.Format("%6.3f -- %6.3f",fmin.z(),fmax.z()); wrenchset->AddSubItem(new CMFCPropertyGridProperty(_T("Fz"),(_variant_t)str,_T("Available force in z-directions")));
		str.Format("%6.3f -- %6.3f",Mmin.x(),Mmax.x()); wrenchset->AddSubItem(new CMFCPropertyGridProperty(_T("Mx"),(_variant_t)str,_T("Available torques around the x-axis")));
		str.Format("%6.3f -- %6.3f",Mmin.y(),Mmax.y()); wrenchset->AddSubItem(new CMFCPropertyGridProperty(_T("My"),(_variant_t)str,_T("Available torques around the y-axis")));
		str.Format("%6.3f -- %6.3f",Mmin.z(),Mmax.z()); wrenchset->AddSubItem(new CMFCPropertyGridProperty(_T("Mz"),(_variant_t)str,_T("Available torques around the z-axis")));
	}
	return true;
}

bool CPosePane::SetDexterityMeasure(const double& columnNorm, const double& rowNorm, const double& minSV, const double& maxSV) 
{
	// set or modify the elements in the treeview
	CString str;
	if (dexterity->GetSubItemsCount() > 0)
	{
		// there are already some items; we have to modify them
		str.Format("%6.5f",columnNorm); 
		dexterity->GetSubItem(0)->SetValue((_variant_t)str);
		str.Format("%6.5f",rowNorm);  
		dexterity->GetSubItem(1)->SetValue((_variant_t)str);
		str.Format("%6.5f",minSV);  
		dexterity->GetSubItem(2)->SetValue((_variant_t)str);
		str.Format("%6.5f",maxSV);  
		dexterity->GetSubItem(3)->SetValue((_variant_t)str);
	}
	else
	{
		// no items in property forcedistribution present; we have to insert new items
		str.Format("%6.5f",columnNorm); 
		dexterity->AddSubItem(new CMFCPropertyGridProperty(_T("Column norm"),(_variant_t)str,_T("Column norm of the structure matrix")));
		str.Format("%6.5f",rowNorm);  
		dexterity->AddSubItem(new CMFCPropertyGridProperty(_T("row norm"),(_variant_t)str,_T("row norm of the structure matrix")));
		str.Format("%6.5f",minSV);  
		dexterity->AddSubItem(new CMFCPropertyGridProperty(_T("min. singular value"),(_variant_t)str,_T("smallest singular value computed with SVD of the structure matrix")));	
		str.Format("%6.5f",maxSV);  
		dexterity->AddSubItem(new CMFCPropertyGridProperty(_T("max. singular value"),(_variant_t)str,_T("largest singular value computed with SVD of the structure matrix")));	
	}	
	return true;
}

bool CPosePane::SetStiffness(const double& minStiffness, const double& minTranslationalStiffness) 
{
	// set or modify the elements in the treeview
	CString str;
	if (stiffness->GetSubItemsCount() > 0)
	{
		// there are already some items; we have to modify them
		str.Format("%6.3f",minStiffness); 
		stiffness->GetSubItem(0)->SetValue((_variant_t)str);
		str.Format("%6.3f",minTranslationalStiffness); 
		stiffness->GetSubItem(1)->SetValue((_variant_t)str);
	}
	else
	{
		// no items in property forcedistribution present; we have to insert new items
		str.Format("%6.3f",minStiffness); 
		stiffness->AddSubItem(new CMFCPropertyGridProperty(_T("min. stiffness"),(_variant_t)str,_T("minimum stiffness of the robot related to translation and rotation")));
		str.Format("%6.3f",minTranslationalStiffness);  
		stiffness->AddSubItem(new CMFCPropertyGridProperty(_T("translational stiffness"),(_variant_t)str,_T("translational stiffness of the robot only considering the translational part of the structure matrix")));
	}	

	return true;
}

bool CPosePane::GetPoseParameter(Vector3d& r, Matrix3d& R)
{
	if (pose->GetSubItemsCount() > 0)
	{
		// set robot position
		r.x() = pose->GetSubItem(0)->GetValue().dblVal;
		r.y() = pose->GetSubItem(1)->GetValue().dblVal;
		r.z() = pose->GetSubItem(2)->GetValue().dblVal;

		// perform rotation
		R =  Matrix3d::XRotationMatrix3d(pose->GetSubItem(3)->GetValue().dblVal*DEG_TO_RAD)
			*Matrix3d::YRotationMatrix3d(pose->GetSubItem(4)->GetValue().dblVal*DEG_TO_RAD)
			*Matrix3d::ZRotationMatrix3d(pose->GetSubItem(5)->GetValue().dblVal*DEG_TO_RAD);
		
		return true;
	}
	else
		return false;
}

bool CPosePane::SetPropertyGridVisible(const bool bVisible)
{
	if (bVisible)
	{
		m_wndProp.ShowWindow(SW_SHOW);
		m_wndProp.RedrawWindow();
	}
	else
	{
		m_wndProp.ShowWindow(SW_HIDE);
	}

	return true;
}

/*! use the pose evaluator framework to evaluat a bunch of properties in one run
 *  this function auto generations the respective entries in the propertyGrid
 *  \param Evaluator the PoseEvaluator object storing all the poseProperties t be computed
 *  \param r the position to be evaluated
 *  \param R the orienation to be evaluated
 */
bool CPosePane::SetEvaluator(PCRL::CPosePropertyEvaluation& Evaluator, Vector3d& r, Matrix3d& R)
{
	const char formatstr[] = "%3.6f";

	m_wndProp.ShowWindow(SW_HIDE);
	// storage for the return values
	MatrixXd values(1,1);
	list<PCRL::CActivePoseProperty*> appList = Evaluator.getActiveProperties();

	// delete all non-static items TODO: this method implicitly assumes 10 previous panes! 
	int max = m_wndProp.GetPropertyCount();
	for (int i=10; i<max; i++){
		CMFCPropertyGridProperty* prop = m_wndProp.GetProperty(10);
		m_wndProp.DeleteProperty(prop);
	}
	
	// generate and update data managed through poseEvaluator
	for (auto itor = appList.begin(); itor != appList.end(); ++itor)
	{
		// search through all items if the evaluator has aloready a CMFCPropertyGridProperty Item
		CMFCPropertyGridProperty* root = 0;
		for (int i=0; i<m_wndProp.GetPropertyCount(); i++)
		{
			// we internally label the group with the adress of the respective property as reference
			DWORD handle = m_wndProp.GetProperty(i)->GetData();	
			if (handle == (DWORD)(*itor)->Property)
			{
				// we found it!
				root = m_wndProp.GetProperty(i);
				break;
			}
		}

		// the property is active and needs to be computed
		// test if the computation of the property is successful before using the values
		if (!(*itor)->Property->computeProperty(r,R,values))
			continue;
		// prepare to add the items into the list
		vector<string> names;
		(*itor)->Property->getPropertyNames(names);

		// if the CMFCPropertyGridProperty does not exist, we generate a respective entry
		if (!root)
		{
			root = new CMFCPropertyGridProperty(PCRL::emPoseProperty::Names[(*itor)->Type],(DWORD)(*itor)->Property);
			m_wndProp.AddProperty(root);
			for (unsigned int i=0; i<names.size(); i++)
			{
				CString str,label;
				str.Format(formatstr,values(i));
				label = names[i].c_str();
				root->AddSubItem(new CMFCPropertyGridProperty(label,str));
			}
		}
		else
		{	// only change the existing items; root points to the base item
			//! \todo We have to add code to handle the case where the number of entries has changed
			for (unsigned int i=0; i<names.size(); i++)
			{
				CString str;
				str.Format(formatstr,values(i));
				root->GetSubItem(i)->SetValue(str);
			}
		}
	}
	
	m_wndProp.ShowWindow(SW_SHOW);
	m_wndProp.RedrawWindow();
	return true;
}

// message handler of the CPosePane
///////////////////////////////////

void CPosePane::OnSetPose()
{
	CWireCenterView* pView = CWireCenterView::This;
	// update the robot pose to the entered pos
	GetPoseParameter(pView->GetRobotDoc()->r,pView->GetRobotDoc()->R);
	// send a meesage to the view class to update the 3d view
	pView->PostMessage(WM_UPDATEPOSE,IDOK);
}

void CPosePane::OnUpdateSetPose(CCmdUI *pCmdUI) 
{ pCmdUI->Enable(TRUE); }

void CPosePane::OnAutoUpdatePose()
{
	CWireCenterView* pView = CWireCenterView::This;
	m_bInstantPosePaneUpdate = !m_bInstantPosePaneUpdate;
	pView->m_bInstantPosePaneUpdate = m_bInstantPosePaneUpdate;
}

void CPosePane::OnUpdateAutoUpdatePose(CCmdUI *pCmdUI) 
{ pCmdUI->Enable(TRUE); pCmdUI->SetCheck(m_bInstantPosePaneUpdate);}

void CPosePane::OnUpdateOptionsAutomaticPoseAnalysis(CCmdUI *pCmdUI) 
{ pCmdUI->Enable(TRUE);}

BOOL CPosePane::PreTranslateMessage(MSG* pMsg)
{
	if ( pMsg->wParam == VK_RETURN)
	{
		OnSetPose();
		return true;
	}
	return CDockablePane::PreTranslateMessage(pMsg);
}
