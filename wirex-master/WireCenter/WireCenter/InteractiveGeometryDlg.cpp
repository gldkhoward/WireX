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

// InteractiveGeometryDlg.cpp: Implementierungsdatei
//

#include "stdafx.h"
#include "WireCenter.h"
#include "InteractiveGeometryDlg.h"
#include "WireCenterDoc.h"
#include "WireCenterView.h"
#include "GenericParamDlg.h"


/////////////////////////////////////////////
// IMPLEMENTATION OF CInteractiveGeometryPane
/////////////////////////////////////////////

CInteractiveGeometryPane::CInteractiveGeometryPane()
{
	m_AutomaticWorkspaceUpdate=FALSE; 
	m_currentValue=0; 
	currentID=0;
	// set the step width for changing the parameter
	smallstep=.05;
	largestep=.5;
}

BEGIN_MESSAGE_MAP(CInteractiveGeometryPane, CDockablePane)
	ON_WM_CREATE()
	ON_WM_SIZE()
	ON_REGISTERED_MESSAGE(AFX_WM_PROPERTY_CHANGED, &CInteractiveGeometryPane::OnPropertyChanged)
	ON_COMMAND(ID_IGP_APPLY, &CInteractiveGeometryPane::OnIgpApply)
	ON_COMMAND(ID_IGP_AUTOAPPLY, &CInteractiveGeometryPane::OnIgpAutoapply)
	ON_COMMAND(ID_IGP_DEC, &CInteractiveGeometryPane::OnIgpDec)
	ON_COMMAND(ID_IGP_FAST_DEC, &CInteractiveGeometryPane::OnIgpFastDec)
	ON_COMMAND(ID_IGP_FAST_INC, &CInteractiveGeometryPane::OnIgpFastInc)
	ON_COMMAND(ID_IGP_INC, &CInteractiveGeometryPane::OnIgpInc)
	ON_COMMAND(ID_IGP_OPTIONS, &CInteractiveGeometryPane::OnIgpOptions)
	ON_UPDATE_COMMAND_UI(ID_IGP_APPLY, &CInteractiveGeometryPane::OnUpdateIgpApply)
	ON_UPDATE_COMMAND_UI(ID_IGP_AUTOAPPLY, &CInteractiveGeometryPane::OnUpdateIgpAutoapply)
	ON_UPDATE_COMMAND_UI(ID_IGP_DEC, &CInteractiveGeometryPane::OnUpdateIgpDec)
	ON_UPDATE_COMMAND_UI(ID_IGP_FAST_DEC, &CInteractiveGeometryPane::OnUpdateIgpFastDec)
	ON_UPDATE_COMMAND_UI(ID_IGP_FAST_INC, &CInteractiveGeometryPane::OnUpdateIgpFastInc)
	ON_UPDATE_COMMAND_UI(ID_IGP_INC, &CInteractiveGeometryPane::OnUpdateIgpInc)
	ON_UPDATE_COMMAND_UI(ID_IGP_OPTIONS, &CInteractiveGeometryPane::OnUpdateIgpOptions)
END_MESSAGE_MAP()

void CInteractiveGeometryPane::AdjustLayout()
{
	if (GetSafeHwnd() == NULL) return;
	CRect rectClient;
	GetClientRect(rectClient);

	int cyTlb = m_wndToolBar.CalcFixedLayout(FALSE,TRUE).cy;

	// we still have to add some spacing between the controls
	m_wndToolBar.SetWindowPos(NULL,rectClient.left,rectClient.top,rectClient.Width(),cyTlb,SWP_NOACTIVATE | SWP_NOZORDER);
	m_wndProperties.SetWindowPos(NULL,rectClient.left, cyTlb, rectClient.Width(), rectClient.Height()-cyTlb, SWP_NOACTIVATE | SWP_NOZORDER);
}

int CInteractiveGeometryPane::OnCreate(LPCREATESTRUCT lpCreateStruct)
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
	general = new CMFCPropertyGridProperty(_T("Robot General"));
	general->AddSubItem(new CMFCPropertyGridProperty(_T("Name"), (_variant_t) _T("IPAnema"), _T("Defines the name of the current robot")));
	m_wndProperties.AddProperty(general);
	general->AddSubItem(new CMFCPropertyGridProperty(_T("Author"),(_variant_t) _T("Fraunhofer IPA"),_T("Name of the designer")));
	general->AddSubItem(new CMFCPropertyGridProperty(_T("ID"),(_variant_t) _T(""),_T("Identification code")));
	general->AddSubItem(new CMFCPropertyGridProperty(_T("Description"),(_variant_t) _T(""),_T("Description text for the robot")));
	
	// add the workspace property group	
	workspaceHull = new CMFCPropertyGridProperty(_T("Workspace Properties (Hull)"));
	m_wndProperties.AddProperty(workspaceHull);
	setWorkspaceProperty();

	// add the workspace grid group
	workspaceGrid = new CMFCPropertyGridProperty(_T("Workspace Properties (Grid)"));
	m_wndProperties.AddProperty(workspaceGrid);
	setWorkspaceGridProperty();

	// add the workspace grid group
	workspaceCrosssection = new CMFCPropertyGridProperty(_T("Workspace Properties (Contour)"));
	m_wndProperties.AddProperty(workspaceCrosssection);
	setWorkspaceCrosssectionProperty();

	// set the geometry data in the control
	CWireCenterDoc* pDoc = (CWireCenterDoc*)(CWireCenterView::This->GetDocument());
	if (pDoc)
	{
		int param_id=0;
		for (int i=0; i<pDoc->robotDoc.getNow(); i++)
		{
			CString str;
			// add the group
			str.Format("Winch %i",i+1);
			CMFCPropertyGridProperty* pGroup1 = new CMFCPropertyGridProperty(str);

			// add the items to the group
			str.Format("a%i.x",i+1);		
			pGroup1->AddSubItem(new CMFCPropertyGridProperty(str, (_variant_t)pDoc->robotDoc.getBase(i).x(), _T("x-coordinate of the robot frame (Ai)"),param_id++,0,0,_T("+-0123456789.")));
			str.Format("a%i.y",i+1);		
			pGroup1->AddSubItem(new CMFCPropertyGridProperty(str, (_variant_t)pDoc->robotDoc.getBase(i).y(), _T("y-coordinate of the robot frame (Ai)"),param_id++,0,0,_T("+-0123456789.")));
			str.Format("a%i.z",i+1);		
			pGroup1->AddSubItem(new CMFCPropertyGridProperty(str, (_variant_t)pDoc->robotDoc.getBase(i).z(), _T("z-coordinate of the robot frame (Ai)"),param_id++,0,0,_T("+-0123456789.")));
			str.Format("b%i.x",i+1);		
			pGroup1->AddSubItem(new CMFCPropertyGridProperty(str, (_variant_t)pDoc->robotDoc.getPlatform(i).x(), _T("x-coordinate of the robot platform (Bi)"),param_id++,0,0,_T("+-0123456789.")));
			str.Format("b%i.y",i+1);		
			pGroup1->AddSubItem(new CMFCPropertyGridProperty(str, (_variant_t)pDoc->robotDoc.getPlatform(i).y(), _T("y-coordinate of the robot platform (Bi)"),param_id++,0,0,_T("+-0123456789.")));
			str.Format("b%i.z",i+1);		
			pGroup1->AddSubItem(new CMFCPropertyGridProperty(str, (_variant_t)pDoc->robotDoc.getPlatform(i).z(), _T("z-coordinate of the robot platform (Bi)"),param_id++,0,0,_T("+-0123456789.")));
			
			m_wndProperties.AddProperty(pGroup1);
		}
	}
	AdjustLayout();
	return 0;
}

void CInteractiveGeometryPane::setRobotProperties(const PCRL::CRobotDocument* pRobotDoc)
{
	if (pRobotDoc==0)
		return;

	if (general->GetSubItemsCount() == 0)
	{
		// generate the items
	}
	else
	{
		CString str;
		// set the values
		str.Format("%s",pRobotDoc->name.c_str());
		general->GetSubItem(0)->SetValue(str);
		str.Format("%s",pRobotDoc->author.c_str());
		general->GetSubItem(1)->SetValue(str);
		str.Format("%s",pRobotDoc->id.c_str());
		general->GetSubItem(2)->SetValue(str);
		str.Format("%s",pRobotDoc->desc.c_str());
		general->GetSubItem(3)->SetValue(str);
	}
}


/*! this function defines which properties of a workspace hull object are 
 *  displayed as subitems in the workspace hull group.
 */
void CInteractiveGeometryPane::setWorkspaceProperty(const PCRL::CWorkspaceHull* pHull)
{
	// if no properties are in the group, we add them
	if (workspaceHull->GetSubItemsCount() == 0)
	{
		double i=0;
		workspaceHull->AddSubItem(new CMFCPropertyGridProperty(_T("Calculation time"),_T(""),_T("Computation time in ms for the determination of the workspace"),0));
		workspaceHull->AddSubItem(new CMFCPropertyGridProperty(_T("Triangles count"),_T(""),_T("Number of triangles of the hull of the workspace"),0));
		workspaceHull->AddSubItem(new CMFCPropertyGridProperty(_T("Vertex count"),_T(""),_T("Number of vertex points for the triangulated workspace hull"),0));
		workspaceHull->AddSubItem(new CMFCPropertyGridProperty(_T("Vertex/second"),_T(""),_T("Computation efficiency: Number of Vertices computed per second"),0));
		workspaceHull->AddSubItem(new CMFCPropertyGridProperty(_T("Surface"),_T(""),_T("Surface of the workspace hull"),0));
		workspaceHull->AddSubItem(new CMFCPropertyGridProperty(_T("Volume"),_T(""),_T("Volume of the workspace hull"),0));
		workspaceHull->AddSubItem(new CMFCPropertyGridProperty(_T("Bounding Box X"),_T("0.00 -- 0.00"),_T("Dimension of the axis aligned bounding box in x-direction"),0));
		workspaceHull->AddSubItem(new CMFCPropertyGridProperty(_T("Bounding Box Y"),_T("0.00 -- 0.00"),_T("Dimension of the axis aligned bounding box in y-direction"),0));
		workspaceHull->AddSubItem(new CMFCPropertyGridProperty(_T("Bounding Box Z"),_T("0.00 -- 0.00"),_T("Dimension of the axis aligned bounding box in z-direction"),0));
		workspaceHull->AddSubItem(new CMFCPropertyGridProperty(_T("Center of Gravity"),_T("0.00, 0.00, 0.00"),_T("Center of gravity of the workspace hull (x,y,z)"),0));
		workspaceHull->AddSubItem(new CMFCPropertyGridProperty(_T("Orientations count"),_T(""),_T("Number of elements in the orientation set"),0));
	}
	// if we got data in pHull, we copy them into the control
	if (pHull)
	{
		CString str;
		// set the values
		str.Format("%d",pHull->getCalculationTime());
		workspaceHull->GetSubItem(0)->SetValue(str);
		str.Format("%d",pHull->Triangles.size());
		workspaceHull->GetSubItem(1)->SetValue(str);
		str.Format("%d",pHull->vertices.size());
		workspaceHull->GetSubItem(2)->SetValue(str);
		str.Format("%8.3f", pHull->getCalculationTime()>0 ? pHull->vertices.size() * 1000.0 / pHull->getCalculationTime() : 0.0);
		workspaceHull->GetSubItem(3)->SetValue(str);
		str.Format("%3.6f",pHull->Surface);
		workspaceHull->GetSubItem(4)->SetValue(str);
		str.Format("%3.6f",pHull->Volume);
		workspaceHull->GetSubItem(5)->SetValue(str);
		str.Format("%3.3f -- %3.3f",pHull->bbMin.x(),pHull->bbMax.x());
		workspaceHull->GetSubItem(6)->SetValue(str);
		str.Format("%3.3f -- %3.3f",pHull->bbMin.y(),pHull->bbMax.y());
		workspaceHull->GetSubItem(7)->SetValue(str);
		str.Format("%3.3f -- %3.3f",pHull->bbMin.z(),pHull->bbMax.z());
		workspaceHull->GetSubItem(8)->SetValue(str);
		str.Format("%3.3f, %3.3f, %3.3f",pHull->CoI.x(),pHull->CoI.y(),pHull->CoI.z());
		workspaceHull->GetSubItem(9)->SetValue(str);
		str.Format("%d",pHull->getOrientationCount());
		workspaceHull->GetSubItem(10)->SetValue(str);
	}
}

/*! this function defines which properties of a workspa    ce hull object are displayed as subitems
 *  in the workspace hull group. When extending or chainging the implementation make sure
 *  the order between the creation and setting group is maintained.
 */
void CInteractiveGeometryPane::setWorkspaceGridProperty(const PCRL::CWorkspaceGrid* pGrid)
{
	// it no properties are in the group, we add them
	if (workspaceGrid->GetSubItemsCount() == 0)
	{
		double i=0;
		workspaceGrid->AddSubItem(new CMFCPropertyGridProperty(_T("Calculation time"),_T(""),_T("Computation time in ms for the workspace computation"),0));
		workspaceGrid->AddSubItem(new CMFCPropertyGridProperty(_T("Coverage"),_T(""),_T("Percentage of the valid grid points in relation to the overall size of the grid."),0));
		workspaceGrid->AddSubItem(new CMFCPropertyGridProperty(_T("Sample point count"),_T(""),_T("Number of grid sample points."),0));
		workspaceGrid->AddSubItem(new CMFCPropertyGridProperty(_T("Valid points"),_T(""),_T("Number of valid points in the grid."),0));
		workspaceGrid->AddSubItem(new CMFCPropertyGridProperty(_T("Invalid points"),_T(""),_T("Number if invalid points in the grid."),0));
		workspaceGrid->AddSubItem(new CMFCPropertyGridProperty(_T("Sample / second"),_T(""),_T("Efficiency: Number of evaluations per second."),0));
	}
	// if we got data in pGrid, we copy them into the control
	if (pGrid)
	{
		CString str;
		// set the values
		str.Format("%d",pGrid->getCalculationTime());
		workspaceGrid->GetSubItem(0)->SetValue(str);
		str.Format("%3.2f",pGrid->getCoverage()*100);
		workspaceGrid->GetSubItem(1)->SetValue(str);
		str.Format("%d",pGrid->vertices.size());
		workspaceGrid->GetSubItem(2)->SetValue(str);
		str.Format("%d",pGrid->workspace.size());
		workspaceGrid->GetSubItem(3)->SetValue(str);
		str.Format("%d",pGrid->workspace_out.size());
		workspaceGrid->GetSubItem(4)->SetValue(str);
		str.Format("%8.3f", pGrid->getCalculationTime()>0 ? pGrid->vertices.size() * 1000.0 / pGrid->getCalculationTime() : 0.0);
		workspaceGrid->GetSubItem(5)->SetValue(str);

	}
}

/*! this function defines which properties of a workspace cross section object 
 *  are displayed as subitems in the workspace cross section group. When 
 *  extending or changing the implementation make sure the order between the 
 *  creation and setting group below is maintained.
 */
void CInteractiveGeometryPane::setWorkspaceCrosssectionProperty(const PCRL::CWorkspaceCrosssection* pCS)
{
	// if no properties are in the group, we (initally) add them
	if (workspaceCrosssection->GetSubItemsCount() == 0)
	{
		double i=0;
		workspaceCrosssection->AddSubItem(new CMFCPropertyGridProperty(_T("Calculation time"),_T(""),_T("Computation time for the workspace cross section in ms."),0));
		workspaceCrosssection->AddSubItem(new CMFCPropertyGridProperty(_T("Line count"),_T(""),_T("Number of line segments used for the cross section."),0));
		workspaceCrosssection->AddSubItem(new CMFCPropertyGridProperty(_T("Circumfence"),_T(""),_T("Length of the circumfence."),0));
		workspaceCrosssection->AddSubItem(new CMFCPropertyGridProperty(_T("Projection center"),_T(""),_T("Projection center of the cross section."),0));
	}
	// if we got data in pGrid, we copy them into the control
	if (pCS)
	{
		CString str;
		// set the values
		str.Format("%d",pCS->getCalculationTime());
		workspaceCrosssection->GetSubItem(0)->SetValue(str);
		str.Format("%d",pCS->vertices.size());
		workspaceCrosssection->GetSubItem(1)->SetValue(str);
		str.Format("not implemented");
		workspaceCrosssection->GetSubItem(2)->SetValue(str);
		double x,y,z;
		pCS->getProjectionCenter(x,y,z);
		str.Format("%3.3f, %3.3f, %3.3f",x,y,z);
		workspaceCrosssection->GetSubItem(3)->SetValue(str);		
	}
}

void CInteractiveGeometryPane::updateAllData(bool AllowWorkspaceUpdate)
{	
	CWireCenterDoc* pDoc = (CWireCenterDoc*)(CWireCenterView::This->GetDocument());
	
	if (pDoc)
	{
		//hide the window to avoid window updates at every SetData call
		m_wndProperties.ShowWindow(SW_HIDE);

		int num_count = m_wndProperties.GetPropertyCount();
		// search through all top level elements
		for (int i=0; i<num_count; i++)
		{
			int sub_num_count = m_wndProperties.GetProperty(i)->GetSubItemsCount();
			// and all their childs
			for (int j=0; j<sub_num_count; j++)
			{
				int currentID = m_wndProperties.GetProperty(i)->GetSubItem(j)->GetData();
				if (currentID!=0) 
					m_wndProperties.GetProperty(i)->GetSubItem(j)->SetValue((double)pDoc->robotDoc.getByID(currentID));
				// ASP: disabled the special handling for a1x by VLS. Adding properties (such workspace properties) corrupts the fixed offset (currently 4)
				// furtherore, it seems that checking for data entry 0 triggers for all properties except for those of interest.
//				else 
//					m_wndProperties.GetProperty(4)->GetSubItem(0)->SetValue((double)pDoc->robotDoc.getByID(0)); //VLS: fix for update of a1.x (which has currentID=0) could use a better solution
			}
		
		}
		// update the general properties section if we have access to the robot document
		setRobotProperties(&pDoc->robotDoc);

		// show the window to enforce new paint only here
		m_wndProperties.ShowWindow(SW_SHOW);
		m_wndProperties.RedrawWindow();
	}
	
	// recalc the workspace if wanted
	if (m_AutomaticWorkspaceUpdate && AllowWorkspaceUpdate) 
		OnIgpApply();
}

void CInteractiveGeometryPane::OnSize(UINT nType, int cx, int cy)
{
	CDockablePane::OnSize(nType,cx,cy);
	AdjustLayout();
}

void CInteractiveGeometryPane::changeCurrentItem(double setValue)
{
	CWireCenterDoc* pDoc = (CWireCenterDoc*)(CWireCenterView::This->GetDocument());
	if (pDoc)
	{
		if (m_wndProperties.GetCurSel()) 
		{
			double newValue = m_wndProperties.GetCurSel()->GetValue().dblVal+setValue;
			m_wndProperties.GetCurSel()->SetValue(newValue);
			// handle the change in the OnPropertyChanged function
			OnPropertyChanged(0,(LPARAM)m_wndProperties.GetCurSel());
		}
	}
}


LRESULT CInteractiveGeometryPane::OnPropertyChanged(__in WPARAM wparam, __in LPARAM lparam)
{
	// get the pointer to the robot doc 
	CWireCenterDoc* pDoc = (CWireCenterDoc*)(CWireCenterView::This->GetDocument());
	CMFCPropertyGridProperty* pProp = (CMFCPropertyGridProperty*) lparam; 

	// set the respective value
	if (pDoc) pDoc->robotDoc.getByID(pProp->GetData())=pProp->GetValue().dblVal;

	// recalc the workspace if wanted
	if (m_AutomaticWorkspaceUpdate) OnIgpApply();
	
	// refresh view
	CWireCenterView::This->Invalidate();

	// return 0 because everything worked fine
	return 0;
}

// currently all event handlers are dummy; we still need to port the functions from the dialog version
void CInteractiveGeometryPane::OnIgpApply()
{
	CWireCenterView::This->OnArbeitsraumBerechnen();
}

void CInteractiveGeometryPane::OnIgpAutoapply()
{ 
	m_AutomaticWorkspaceUpdate=1-m_AutomaticWorkspaceUpdate;
}

void CInteractiveGeometryPane::OnIgpDec()
{ changeCurrentItem(-smallstep); }

void CInteractiveGeometryPane::OnIgpFastDec()
{ changeCurrentItem(-largestep); }

void CInteractiveGeometryPane::OnIgpFastInc()
{ changeCurrentItem(largestep); }

void CInteractiveGeometryPane::OnIgpInc()
{ changeCurrentItem(smallstep); }

void CInteractiveGeometryPane::OnIgpOptions()
{
	// present an options dialog to the to change the step size
	CEnhancedDialog GPD("Interactive Geometry Settings",300);
	GPD.addItem("small step",this->smallstep);
	GPD.addItem("large step",this->largestep);
	GPD.addItem("automatic workspace updated",this->m_AutomaticWorkspaceUpdate);
	GPD.DoModal();
	updateAllData(); 
}

void CInteractiveGeometryPane::OnUpdateIgpApply(CCmdUI *pCmdUI)
{ pCmdUI->Enable(); }

void CInteractiveGeometryPane::OnUpdateIgpAutoapply(CCmdUI *pCmdUI)
{	pCmdUI->Enable();
	pCmdUI->SetCheck(m_AutomaticWorkspaceUpdate);
}

void CInteractiveGeometryPane::OnUpdateIgpDec(CCmdUI *pCmdUI)
{ pCmdUI->Enable(); }

void CInteractiveGeometryPane::OnUpdateIgpFastDec(CCmdUI *pCmdUI)
{ pCmdUI->Enable(); }

void CInteractiveGeometryPane::OnUpdateIgpFastInc(CCmdUI *pCmdUI)
{ pCmdUI->Enable(); }

void CInteractiveGeometryPane::OnUpdateIgpInc(CCmdUI *pCmdUI)
{ pCmdUI->Enable(); }

void CInteractiveGeometryPane::OnUpdateIgpOptions(CCmdUI *pCmdUI)
{ pCmdUI->Enable(); }
