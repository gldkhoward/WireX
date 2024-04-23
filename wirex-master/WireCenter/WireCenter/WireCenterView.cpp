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

// WireCenterView.cpp : Implementierung der Klasse CWireCenterView
//

#include "stdafx.h"
#include "WireCenter.h"
#include "WireCenterDoc.h"
#include "WireCenterView.h"
#include "PoseDlg.h"
#include "InteractiveGeometryDlg.h"
#include "MainFrm.h"
#include "PythonInterface.h"
#include "GenericParamDlg.h"
#include "IPC.h"
#include <WireLib/PoseEvaluator.h>
#include "PersistantDefaultValue.h"
#include "PoseEvaluatorDlg.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// a static pointer to the view class is needed for python call-backs
CWireCenterView* CWireCenterView::This=0;

// CWireCenterView

IMPLEMENT_DYNCREATE(CWireCenterView, CView)

BEGIN_MESSAGE_MAP(CWireCenterView, CView)
	ON_WM_ERASEBKGND()
	ON_WM_CREATE()
	ON_WM_DESTROY()
	ON_WM_SIZE()
	ON_WM_LBUTTONDOWN()
	ON_WM_MOUSEMOVE()
	ON_WM_LBUTTONUP()
	ON_WM_RBUTTONDOWN()
	ON_WM_RBUTTONUP()
	ON_BN_CLICKED(IDC_CALCULATEWORKSPACE_BUTTON, &CWireCenterView::OnBnClickedCalculateworkspaceButton)
	ON_MESSAGE(WM_UPDATEPOSE,&CWireCenterView::OnUpdatePose)
	ON_MESSAGE(WM_UPDATEWORKSPACE,&CWireCenterView::OnUpdateWorkspace)
	ON_WM_SHOWWINDOW()
	ON_COMMAND(ID_ARBEITSRAUM_BERECHNEN, &CWireCenterView::OnArbeitsraumBerechnen)
	ON_COMMAND(ID_ARBEITSRAUM_DIFFERENTIELBERECHNEN, &CWireCenterView::OnArbeitsraumDifferentielberechnen)
	ON_COMMAND(ID_ANSICHT_AUSRICHTENXYEBENE, &CWireCenterView::OnAnsichtAusrichtenxyebene)
	ON_COMMAND(ID_ANSICHT_AUSRICHTENXZEBENE, &CWireCenterView::OnAnsichtAusrichtenxzebene)
	ON_COMMAND(ID_ANSICHT_AUSRICHTENYZEBENE, &CWireCenterView::OnAnsichtAusrichtenyzebene)
	ON_COMMAND(ID_ANSICHT_ZURUECKSETZEN, &CWireCenterView::OnAnsichtZuruecksetzen)
	ON_COMMAND(ID_ARBEITSRAUM_INTERVALLMETHODE, &CWireCenterView::OnArbeitsraumIntervallmethode)
	ON_COMMAND(ID_ARBEITSRAUM_KONTUR_CUSTOM_X, &CWireCenterView::OnArbeitsraumKonturCustomX)
	ON_COMMAND(ID_ARBEITSRAUM_KONTUR_CUSTOM_Y, &CWireCenterView::OnArbeitsraumKonturCustomY)
	ON_COMMAND(ID_ARBEITSRAUM_KONTUR_CUSTOM_Z, &CWireCenterView::OnArbeitsraumKonturCustomZ)
	ON_UPDATE_COMMAND_UI(ID_ARBEITSRAUM_KONTUR, &CWireCenterView::OnUpdateArbeitsraumKontur)
	ON_COMMAND(ID_ARBEITSRAUM_VEREINIGEN, &CWireCenterView::OnArbeitsraumVereinigen)
	ON_COMMAND(ID_ARBEITSRAUM_SCHNEIDEN, &CWireCenterView::OnArbeitsraumSchneiden)
	ON_COMMAND(ID_ARBEITSRAUM_ORIENTIERUNGAUTOMATISCHUEBERNEHMEN, &CWireCenterView::OnArbeitsraumOrientierungautomatischuebernehmen)
	ON_UPDATE_COMMAND_UI(ID_ARBEITSRAUM_ORIENTIERUNGAUTOMATISCHUEBERNEHMEN, &CWireCenterView::OnUpdateArbeitsraumOrientierungautomatischuebernehmen)
	ON_UPDATE_COMMAND_UI(ID_ARBEITSRAUM_ALLEORIENTIERUNGENPR32824, &CWireCenterView::OnUpdateArbeitsraumAlleorientierungenpr32824)
	ON_COMMAND(ID_ENTWURF_ANWENDUNGSDATENANZEIGEN, &CWireCenterView::OnEntwurfAnwendungsdatenanzeigen)
	ON_UPDATE_COMMAND_UI(ID_ENTWURF_ANWENDUNGSDATENANZEIGEN, &CWireCenterView::OnUpdateEntwurfAnwendungsdatenanzeigen)
	ON_UPDATE_COMMAND_UI(ID_ARBEITSRAUM_KOLLISIONSZONENSICHTBAR, &CWireCenterView::OnUpdateArbeitsraumKollisionszonensichtbar)
	ON_COMMAND(ID_ARBEITSRAUM_KOLLISIONSZONENSICHTBAR, &CWireCenterView::OnArbeitsraumKollisionszonensichtbar)
	ON_COMMAND(ID_BERECHNEN_AUTOMATISCHEPOSENANALYSEOPTIONEN, &CWireCenterView::OnBerechnenAutomatischeposenanalyseoptionen)
	ON_WM_TIMER()
	ON_COMMAND(ID_NC_LADEN, &CWireCenterView::OnNcLaden)
	ON_COMMAND(ID_NC_OPTIONEN, &CWireCenterView::OnNcOptionen)
	ON_COMMAND(ID_NC_STARTEN, &CWireCenterView::OnNcStarten)
	ON_COMMAND(ID_NC_STOPPEN, &CWireCenterView::OnNcStoppen)
	ON_UPDATE_COMMAND_UI(ID_NC_STARTEN, &CWireCenterView::OnUpdateNcStarten)
	ON_UPDATE_COMMAND_UI(ID_NC_STOPPEN, &CWireCenterView::OnUpdateNcStoppen)
	ON_COMMAND(ID_NC_PAUSE, &CWireCenterView::OnNcFortsetzen)
	ON_UPDATE_COMMAND_UI(ID_NC_PAUSE, &CWireCenterView::OnUpdateNcFortsetzen)
	ON_COMMAND(ID_BERECHNEN_TRAJEKTORIEANALYSIEREN, &CWireCenterView::OnBerechnenTrajektorieanalysieren)
	ON_UPDATE_COMMAND_UI(ID_BERECHNEN_TRAJEKTORIEANALYSIEREN, &CWireCenterView::OnUpdateBerechnenTrajektorieanalysieren)
	ON_COMMAND(ID_NC_AVIEXPORTIEREN, &CWireCenterView::OnNcAviexportieren)
	ON_UPDATE_COMMAND_UI(ID_NC_AVIEXPORTIEREN, &CWireCenterView::OnUpdateNcAviexportieren)
	ON_UPDATE_COMMAND_UI(ID_NC_LADEN, &CWireCenterView::OnUpdateNcLaden)
	ON_COMMAND(ID_ITERATION_EDIT, &CWireCenterView::OnIterationEdit)
	ON_COMMAND(ID_EPS_EDIT, &CWireCenterView::OnEpsEdit)
	ON_COMMAND(ID_ARBEITSRAUM_MULTITHREADING, &CWireCenterView::OnArbeitsraumMultithreading)
	ON_UPDATE_COMMAND_UI(ID_ARBEITSRAUM_MULTITHREADING, &CWireCenterView::OnUpdateArbeitsraumMultithreading)
	ON_COMMAND(ID_GITTER_SICHTBAR_BUTTON, &CWireCenterView::OnGitterSichtbarButton)
	ON_UPDATE_COMMAND_UI(ID_GITTER_SICHTBAR_BUTTON, &CWireCenterView::OnUpdateGitterSichtbarButton)
	ON_COMMAND(ID_KONTUR_SICHTBAR_BUTTON, &CWireCenterView::OnKonturSichtbarButton)
	ON_UPDATE_COMMAND_UI(ID_KONTUR_SICHTBAR_BUTTON, &CWireCenterView::OnUpdateKonturSichtbarButton)
	ON_COMMAND(ID_ARBEITSRAUM_GITTER_BUTTON, &CWireCenterView::OnArbeitsraumGitterButton)
	ON_COMMAND(ID_ARBEITSRAUM_GITTER_CUSTOM_BUTTON, &CWireCenterView::OnArbeitsraumGitterCustomButton)
	ON_COMMAND(ID_DIFFERENZIEL_SICHTBAR_BUTTON, &CWireCenterView::OnDifferenzielSichtbarButton)
	ON_UPDATE_COMMAND_UI(ID_DIFFERENZIEL_SICHTBAR_BUTTON, &CWireCenterView::OnUpdateDifferenzielSichtbarButton)
	ON_COMMAND(IDC_WINCH_SELECTOR_COMBO, &CWireCenterView::OnWinchSelectorCombo)
	ON_COMMAND(ID_CLIPTOBOX_BUTTON, &CWireCenterView::OnCliptoboxButton)
	ON_COMMAND(ID_CLIPTOFRAME_BUTTON, &CWireCenterView::OnCliptoframeButton)
	ON_COMMAND(ID_PARAMETERMODEL_COMBO, &CWireCenterView::OnParameterModelCombo)		// we only have a dummy function here
	ON_COMMAND(ID_PARAMETERTYPE_COMBO, &CWireCenterView::OnParameterTypeCombo)			// we only have a dummy function here
	ON_COMMAND(ID_PARAMMODELVALUE_EDIT, &CWireCenterView::OnParameterValueEdit)			// we only have a dummy function here
	ON_COMMAND(ID_PARAMMODELSET_BUTTON, &CWireCenterView::OnParameterSetButton)			// we only have a dummy function here
	ON_COMMAND(ID_PARAMDEC_BUTTON, &CWireCenterView::OnParamdecButton)
	ON_COMMAND(ID_PARAMFASTDEC_BUTTON, &CWireCenterView::OnParamfastdecButton)
	ON_COMMAND(ID_PARAMFASTINC_BUTTON, &CWireCenterView::OnParamfastincButton)
	ON_COMMAND(ID_PARAMINC_BUTTON, &CWireCenterView::OnParamincButton)
	ON_COMMAND(ID_WORKSPACE_GRID_SHOW_INVALID_BUTTON4, &CWireCenterView::OnArbeitsraumGridShowInvalid)
	ON_UPDATE_COMMAND_UI(ID_WORKSPACE_GRID_SHOW_INVALID_BUTTON4, &CWireCenterView::OnUpdateArbeitsraumGridShowInvalid)

	ON_UPDATE_COMMAND_UI(ID_ARBEITSRAUM_HUELLE_SICHTBAR_BUTTON, &CWireCenterView::OnUpdateArbeitsraumHuelleSichtbarButton)
	ON_COMMAND(ID_ARBEITSRAUM_HUELLE_SICHTBAR_BUTTON, &CWireCenterView::OnArbeitsraumHuelleSichtbarButton)
	ON_COMMAND(ID_SCENE_BACKGROUND_BUTTON, &CWireCenterView::OnSceneBackgroundButton)
	ON_UPDATE_COMMAND_UI(ID_SCENE_BACKGROUND_BUTTON, &CWireCenterView::OnUpdateSceneBackgroundButton)
	ON_COMMAND(ID_SCENE_UPPERCOLOR_BUTTON, &CWireCenterView::OnSceneUppercolorButton)
	ON_COMMAND(ID_SCENE_LOWERCOLOR_BUTTON, &CWireCenterView::OnSceneLowercolorButton)
	ON_COMMAND(ID_SCENE_PERSPECTIVE_BUTTON, &CWireCenterView::OnScenePerspectiveButton)
	ON_UPDATE_COMMAND_UI(ID_SCENE_PERSPECTIVE_BUTTON, &CWireCenterView::OnUpdateScenePerspectiveButton)
	ON_COMMAND(ID_SCENE_DELETEALLSHAPES_BUTTON, &CWireCenterView::OnSceneDeleteallshapesButton)
	ON_COMMAND(ID_SCREENSHOT_BUTTON, &CWireCenterView::OnScreenshotButton)
	ON_COMMAND(ID_SCENESETTINGS_BUTTON, &CWireCenterView::OnSceneSettingsButton)
	ON_COMMAND(ID_DRAGMODE3DVIEW_BUTTON, &CWireCenterView::OnDragMode3DViewButton)
	ON_UPDATE_COMMAND_UI(ID_DRAGMODE3DVIEW_BUTTON, &CWireCenterView::OnUpdateDragMode3DViewButton)
	ON_COMMAND(ID_ROI_VISIBLE_BUTTON, &CWireCenterView::OnRoiVisibleButton)
	ON_UPDATE_COMMAND_UI(ID_ROI_VISIBLE_BUTTON, &CWireCenterView::OnUpdateRoiVisibleButton)
	ON_COMMAND(ID_ROI_AUTO_APPLY_BUTTON, &CWireCenterView::OnRoiAutoApplyButton)
	ON_UPDATE_COMMAND_UI(ID_ROI_AUTO_APPLY_BUTTON, &CWireCenterView::OnUpdateRoiAutoApplyButton)
	ON_COMMAND(ID_CLIPTOROI_BUTTON, &CWireCenterView::OnClipToRoiButton)
	ON_UPDATE_COMMAND_UI(ID_ANSICHT_SICHTBARKEIT, &CWireCenterView::OnUpdateAnsichtSichtbarkeit)
	//notify commands for using menu to switch visibilities, for more information see at OnAnsichtSichtbarkeitRange routine
	ON_COMMAND_RANGE(40000,40100, &CWireCenterView::OnAnsichtSichtbarkeitRange)
	ON_UPDATE_COMMAND_UI_RANGE(40000,40100, &CWireCenterView::OnUpdateAnsichtSichtbarkeitRange)
	ON_COMMAND(ID_GENERATE_GRID_BUTTON_BUTTON, &CWireCenterView::OnGenerateGridButtonButton)
	ON_COMMAND(ID_CREATE_GRID_RANDOM_BUTTON, &CWireCenterView::OnCreateGridRandomButton)
	ON_COMMAND(ID_CREATE_GRID_ZLAYER_BUTTON, &CWireCenterView::OnCreateGridZlayerButton)
	ON_COMMAND(ID_EXPORTROTATIONANIMATION_BUTTON, &CWireCenterView::OnAviExportRotation)
	ON_COMMAND(ID_CABLESPAN_LINES_VISIBLE_BUTTON, &CWireCenterView::OnCablespanLinesVisibleButton)
	ON_COMMAND(ID_CABLESPAN_CONE_VISIBILE_BUTTON, &CWireCenterView::OnCablespanConeVisibileButton)
	ON_COMMAND(ID_POSELIST_VIEW_BUTTON, &CWireCenterView::OnPoselistViewButton)
	ON_COMMAND(ID_POSELIST_SAVE_BUTTON, &CWireCenterView::OnPoselistSaveButton)
	ON_UPDATE_COMMAND_UI(ID_POSELIST_SAVE_BUTTON, &CWireCenterView::OnUpdatePoselistSaveButton)
	ON_UPDATE_COMMAND_UI(ID_POSELIST_VIEW_BUTTON, &CWireCenterView::OnUpdatePoselistViewButton)
	ON_COMMAND(ID_CONFIGURE_SHAPE_COLORS_BUTTON, &CWireCenterView::OnConfigureShapeColorsButton)
	END_MESSAGE_MAP()

// CWireCenterView-Erstellung/Zerst�rung

CWireCenterView::CWireCenterView()
{
	// default scene visible settings
	m_bWorkspaceVisible = true;
	m_bWireSpanVisible = false;
	m_bWireSpanConeVisible = false;
	m_bWiresVisible = true;
	m_bPlatformVisible = true;
	m_bPlanesVisible = false;
	m_bInterferenceVisible = false;
	m_bCrossSectionVisible = true;
	m_bFrameBoundingBoxVisible = true;
	m_bFrameFancyVisible = true;
	m_bPlatformBoundBoxVisible = false;
	m_bRequirementBoxVisible = false;
	m_bAutoApplyOrientation = false;
	m_bWinchesLabelVisible = true;
	m_bGravityArrowVisible = false;
	m_bGroundPlaneVisible = false;
	m_bBaseFramesVisible = true;
	m_bWorldFrameVisible = false;
	m_bWinchesVisible = false;
	m_bWorkspaceGridVisible = true;
	m_bWorkspaceGridOutVisible = true;
	m_bWorkspaceInnerBoxVisible = true;
	m_bWorkspaceHullGridVisible = true;
	m_bCableForcesVisible = true;
	m_bWinchPulleysVisible = true;
	m_bWrenchSetVisible = true;
	m_bNcLineStripVisible = true;
	m_bPoseListControlPointsVisible = true;
	m_bPosePropertyEvaluation = false;
	m_bWireForceColoringVisible = false;
	m_bCenterofInertiaVisible = false;
	m_bWorkspaceParallelProjectionVisible = false;
	m_bWorkspacePhongShadingVisible = false;
	m_bRoiVisible = false;
	m_bPoseEstimateBoxVisible = false;
	m_bPoseEstimateBoxSpheresVisible = false;

	// default colors
	ColorControlPoints = CGLRGBColor(1, 1, 0);

	// default configuration
	m_bUseThreading = true;
	m_bAutoRoiUse = false;
	m_bInstantPosePaneUpdate = false;	// no instant pose pane update at startup

	// pose dialog auto analysis settings
	m_bAnalyzeInverseKinematics						= true;
	m_bAnalyzeInverseKinematicsPulley				= false;
	m_bAnalyzeForwardKinematics						= false;
	m_bAnalyzeForwardKinematicsDistance				= false;
	m_bAnalyzeForwardKinematicsPulley  				= false;
	m_bAnalyzeElastoGeometricalForwardKinematics	= false;
	m_bAnalyzeVelocityTransmission					= false;
	m_bAnalyzeDexterity								= false;
	m_bAnalyzeStiffness								= false;
	m_bAnalyzeOrientationRange						= false;
	m_bAnalyzeTranslationRange						= false;
	m_bAnalyzeWrenchSet								= false;
	m_bAnalyzeForceDistribution						= true;
	m_bAnalyzeForceFrame							= false;
	m_bCompareInverseKinematics						= false;
	m_bAnalyzeDynamic								= false;
	m_bVerboseMode									= false;
	m_bAnalyseCableWear								= true;

	// no program is being played at startup
	m_bRunningProgram = false;

	// register variables for reflection
	bind(m_bWorkspaceVisible,"WorkspaceVisible");
	bind(m_bWireSpanVisible,"WireSpanVisible");
	bind(m_bWireSpanConeVisible,"WireSpanConeVisible");
	bind(m_bWiresVisible,"WiresVisible");
	bind(m_bPlatformVisible,"PlatformVisible");
	bind(m_bCableForcesVisible,"CableForcesVisible");
	bind(m_bPlanesVisible,"PlanesVisible");
	bind(m_bInterferenceVisible,"InterferenceVisible");
	bind(m_bCrossSectionVisible,"CrossSectionVisible");
	bind(m_bWorkspaceGridVisible,"WorkspaceGridVisible");
	bind(m_bWorkspaceGridOutVisible,"WorkspaceGridOutVisible");
	bind(m_bWorkspaceInnerBoxVisible,"WorkspaceInnerBoxVisible");
	bind(m_bFrameBoundingBoxVisible,"FrameBoundingBoxVisible");
	bind(m_bFrameFancyVisible,"FrameFancyVisible");
	bind(m_bPlatformBoundBoxVisible,"PlatformBoundingBoxVisible");
	bind(m_bAutoApplyOrientation,"AutoApplyOrientation");
	bind(m_bRequirementBoxVisible,"RequirementBoxVisible");
	bind(m_bWinchesLabelVisible,"WinchesLabelsVisible");
	bind(m_bGravityArrowVisible,"GravityArrowVisible");
	bind(m_bWinchPulleysVisible,"WinchPulleysVisible");
	bind(m_bWorldFrameVisible,"WorldFrameVisible");
	bind(m_bBaseFramesVisible,"BaseFramesVisible");
	bind(m_bCenterofInertiaVisible,"CenterofInertiaVisible");
	bind(m_bWorkspaceParallelProjectionVisible,"WorkspaceParallelProjectionVisible");
	bind(m_bWorkspacePhongShadingVisible,"WorkspacePhongShadingVisible");
	bind(m_bPoseEstimateBoxVisible,"PoseEstimateBoxVisible");
	bind(m_bPoseEstimateBoxSpheresVisible,"PoseEstimateBoxWireLengthSpheresVisible");
	bind(m_bGroundPlaneVisible,"GroundPlaneVisible");
	bind(m_bWorkspaceGridVisible,"WorkspaceGridVisible");
	bind(m_bWorkspaceHullGridVisible,"WorkspaceHullGridVisible");
	bind(m_bWrenchSetVisible,"WrenchSetVisible");
	bind(m_bNcLineStripVisible,"NcLineStripVisible");
	bind(m_bWireForceColoringVisible,"WireForceColoringVisible");
	bind(m_bPoseListControlPointsVisible,"PoseListControlPointsVisible");
	bind(m_bPosePropertyEvaluation,"PosePropertyEvaluationVisible");
	
	bind(m_bAnalyzeInverseKinematics,"AnalyzeInverseKinematics");
	bind(m_bAnalyzeInverseKinematicsPulley,"AnalyzeInverseKinematicsPulley");
	bind(m_bAnalyzeElastoGeometricalForwardKinematics,"AnalyzeInverseKinematicsPulley");
	bind(m_bAnalyzeForwardKinematics,"AnalyzeForwardKinematics");
	bind(m_bAnalyzeForwardKinematicsDistance,"AnalyzeForwardKinematicsDistance");
	bind(m_bAnalyzeForwardKinematicsPulley,"AnalyzeForwardKinematicsPulley");
	bind(m_bAnalyzeVelocityTransmission,"AnalyzeVelocityTransmission");
	bind(m_bAnalyzeDexterity,"AnalyzeDexterity");
	bind(m_bAnalyzeStiffness,"AnalyzeStiffness");
	bind(m_bAnalyzeWrenchSet,"AnalyzeWrenchSet", "wc/@AnalyzeWrenchSet");
	bind(m_bAnalyzeTranslationRange,"AnalyzeTranslationRange");
	bind(m_bAnalyzeOrientationRange,"AnalyzeOrientationRange");
	bind(m_bAnalyzeForceDistribution,"AnalyzeForceDistribution");
	bind(m_bAnalyzeForceFrame,"AnalyzeForceFrame");
	bind(m_bAnalyzeDynamic,"AnalyzeDynamic");
	bind(m_bCompareInverseKinematics,"CompareInverseKinematics");
	bind(m_bVerboseMode,"PoseUpdateVerboseMode");

	bind(ColorControlPoints.R, "ColorControlPoints.R", "Color/ControlPoints/@R");
	bind(ColorControlPoints.G, "ColorControlPoints.G", "Color/ControlPoints/@G");
	bind(ColorControlPoints.B, "ColorControlPoints.B", "Color/ControlPoints/@B");

	bind(m_bUseThreading,"UseThreading");
	bind(m_bAutoApplyOrientation,"AutoApplyOrientation");
	bind(m_bAutoRoiUse,"AutoRoiUse");

	bind(m_bInstantPosePaneUpdate,"InstantPosePaneUpdate");

	m_pIPC = new CIPC(this);
	m_bPanCamera = false;
}

CWireCenterView::~CWireCenterView()
{
	// delete the non-model dialog boxes
	delete m_pIPC; 
	m_pIPC = NULL;
}

BOOL CWireCenterView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: �ndern Sie hier die Fensterklasse oder die Darstellung, indem Sie
	//  CREATESTRUCT cs modifizieren.

	return CView::PreCreateWindow(cs);
}

// for some strange reason the edit controls are not active unless we have this dummy event handlers
void CWireCenterView::OnIterationEdit() {}
void CWireCenterView::OnEpsEdit() {}

void CWireCenterView::OnInitialUpdate()	
{
	CView::OnInitialUpdate();

	// init the values in the control bar
	CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
	if (!pMF)
	{
		MessageBox("Fatal error in OnInitialUpdate(). No pointer to mainframe could be obtained!");
		return;
	}

	CMFCRibbonEdit* pEdit = DYNAMIC_DOWNCAST(CMFCRibbonEdit,pMF->m_wndRibbonBar.FindByID(ID_ITERATION_EDIT));
	if (pEdit)
		pEdit->SetEditText("3");
	
	pEdit = DYNAMIC_DOWNCAST(CMFCRibbonEdit,pMF->m_wndRibbonBar.FindByID(ID_EPS_EDIT));
	if (pEdit)
		pEdit->SetEditText("0.001");
	
	PCRL::CRobotDocument *pRD = &GetDocument()->robotDoc; //VLS: hoping Document is built before OnInitialUpdate
	CMFCRibbonComboBox* pCEdit = DYNAMIC_DOWNCAST(CMFCRibbonComboBox, pMF->m_wndRibbonBar.FindByID(IDC_WINCH_SELECTOR_COMBO));
//	ASSERT(pCEdit);
	if (pCEdit)
	{
		pCEdit->RemoveAllItems(); //VLS: can remove this line if element initialized with no Data
		pCEdit->AddItem(pRD->currentWinch->name.c_str());
		pCEdit->SelectItem(0);

		// fill the parametric model database with some custom data
		pCEdit = DYNAMIC_DOWNCAST(CMFCRibbonComboBox, pMF->m_wndRibbonBar.FindByID(ID_PARAMETERMODEL_COMBO));
		ASSERT(pCEdit);
		pCEdit->RemoveAllItems();
		// loop through all available parametric model and add their names to the combo box
		for (vector<PCRL::CGeometryGenerator*>::iterator itor = pRD->ParametricModels.begin(); itor!=pRD->ParametricModels.end(); itor++)
		{
			CString tag;
			if (( (*itor)->getMotionPattern() & pRD->getMotionPattern()) == 0)
				tag+=" =!mp";
			if ((*itor)->getNow() != pRD->getNow())
				tag+=" =!now";
			pCEdit->AddItem((*itor)->getName().c_str()+tag);
		}
		// call the event handle as if someone has just selected the first item to fill the remaining controls with data
		pCEdit->SelectItem(0);
		OnParameterModelCombo();
		OnParameterTypeCombo();
	}

	//################################################################
	// dynamically generate the options for displaying stuff

	// find the "Visibility Button" in the Ribbon and cast down to it
	// CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
	CMFCRibbonButton* pViewButton = DYNAMIC_DOWNCAST(CMFCRibbonButton, pMF->m_wndRibbonBar.FindByID(ID_ANSICHT_SICHTBARKEIT));
	if (!pViewButton) 
		return;
	
	// remove all elements
	pViewButton->RemoveAllSubItems();
	
	// set up an list for storing the variable names and fill it
	std::list<std::string> list;
	getParamNames(list);
	// help counter for ... counting
	int counter = 0;

	CString str;
	CMFCRibbonButton* pNewButton;

	// iterator through the string list
	for (std::list<string>::iterator itor = list.begin(); itor!=list.end(); itor++)
	{
		// only print variables with the string "Visible" in it (we assume that we can use this way)
		int test = itor->find("Visible");
		if (itor->npos==test) 
			continue;
		// cut the "Visible" from the string...
		str = itor->c_str();
		str = str.Left(test);
		// set the ID up with start at 40000, because values over 65535 won't work correctly and under 40000 might taken by other resources 
		pNewButton = new CMFCRibbonButton(40000+counter, str); // could use another string here
		// store the "identification string", which is used to find the correct variables again, in the description
		pNewButton->SetDescription(itor->c_str());
		pViewButton->AddSubItem(pNewButton);
		// increase counter
		counter++;	
	}

	// update the interactive geometry pane for the first time
	if (pMF) 
		pMF->m_wndGeomPane.updateAllData();

	if (pMF)
	{
		pMF->m_wndAlgorithmConfig.addAlgorithm(GetRobotDoc()->WSHull.Reflector());
		pMF->m_wndAlgorithmConfig.addAlgorithm(GetRobotDoc()->Interference.Reflector());
		pMF->m_wndAlgorithmConfig.addAlgorithm(GetRobotDoc()->Kinematics.Reflector());
		pMF->m_wndAlgorithmConfig.addAlgorithm(GetRobotDoc()->Crosssection.Reflector());
		pMF->m_wndAlgorithmConfig.addAlgorithm(GetRobotDoc()->Stiffness.Reflector());
		pMF->m_wndAlgorithmConfig.addAlgorithm(GetRobotDoc()->WSGrid.Reflector());
//		pMF->m_wndAlgorithmConfig.addAlgorithm(GetRobotDoc()->Interpolator.Reflector());
		pMF->m_wndAlgorithmConfig.addAlgorithm(GetRobotDoc()->WrenchSet.Reflector());
		pMF->m_wndAlgorithmConfig.addAlgorithm(GetRobotDoc()->ParametricModels.Reflector());
		pMF->m_wndAlgorithmConfig.addAlgorithm(GetRobotDoc()->ForceDistribution.Reflector());
		// debug purpose (this is just to make the content visible, swithched off by default)
		// pMF->m_wndAlgorithmConfig.addAlgorithm(CPersistantDefaultValue::getInstance());
		pMF->m_wndAlgorithmConfig.addAlgorithm(*this);
	}
	// update the pose pane once for the robot position and other parameters
	OnUpdatePose(0,0);
}

// CWireCenterView-Zeichnung

void CWireCenterView::OnDraw(CDC* /*pDC*/)
{
	// update the visibility flags for the shapes
	pHull->bVisible = m_bWorkspaceVisible;
	pHull->bCenterofInertiaVisible = m_bCenterofInertiaVisible;
	pHull->bWorkspaceParallelProjectionVisible = m_bWorkspaceParallelProjectionVisible;
	pHull->bPhongShading = m_bWorkspacePhongShadingVisible;
	//HullShape.bVisible = m_bWorkspaceVisible;

	pInterference->bVisible = m_bInterferenceVisible;
	pCoordinatePlanes->bVisible = m_bPlanesVisible;	
	pWireSpan->bVisible = m_bWireSpanVisible;
	pWireSpanCone->bVisible = m_bWireSpanConeVisible;
	pCS->bVisible = m_bCrossSectionVisible;
	pRobotShape->bWorldFrameVisible = m_bWorldFrameVisible;
	pRobotShape->bBaseFramesVisible = m_bBaseFramesVisible;
	pRobotShape->bFrameBBVisible = m_bFrameBoundingBoxVisible;
	pRobotShape->bPlatformBBVisible = m_bPlatformBoundBoxVisible;
	pRobotShape->bFrameBBFancyFrame = m_bFrameFancyVisible;
	pRobotShape->bPlatformVisible = m_bFrameFancyVisible;
	pRobotShape->bWiresVisible = m_bWiresVisible;
	pRobotShape->bWinchesVisible = m_bWinchesVisible;
	pRobotShape->bWinchesLabelVisible = m_bWinchesLabelVisible;
	pRobotShape->bGravityArrowVisible = m_bGravityArrowVisible;
	pGroundPlanes->bVisible = m_bGroundPlaneVisible;
	pNcLineStrip->bVisible = m_bNcLineStripVisible;
	pPosePropertyEvaluation->bVisible = m_bPosePropertyEvaluation;
	pPlatformForces->bVisible = m_bCableForcesVisible;
	pWinchPulleys->bVisible = m_bWinchPulleysVisible;
	WrenchSet.bVisible = m_bWrenchSetVisible;

	if (m_bWireForceColoringVisible)
	{
		// get the force Distribution
		GetRobotDoc()->getForceDistribution();

		// calculate color for each wire depending on it's cable force
		for (int i=0; i<GetRobotDoc()->getNow(); i++)
		{
			// if cable force matrix isn't in correct format, print error and get the needed forcedistribution
			if ((GetRobotDoc()->F.cols()<1) || (GetRobotDoc()->F.rows()!=GetRobotDoc()->getNow()))
			{
				cout << "Cannot read cable force matrix for coloring wires! Wrong cable force matrix format!" << endl; 
				break;
			}

			//IDEA: use HSV colorspace with Saturation = 1 and Value = 1 from 0 (-> red) to ~120 (-> green)
		
			// map hue value ( result in degree ) from minimum force up to maximum force in HSV color space
			double color_hue = 120/(GetRobotDoc()->fmin - GetRobotDoc()->fmax) * (GetRobotDoc()->F(i,0)-GetRobotDoc()->fmax);

			// convert in RBG space (based on http://de.wikipedia.org/wiki/HSV-Farbraum#Umrechnung_HSV_in_RGB )
			double color_h = floor(color_hue/60.0);
			double color_f = (color_hue/60.0 - color_h);
			double color_q = (1 - color_f);
			double color_t = color_f;
			if (color_h == 0) {pRobotShape->color_wires[i].R = 1;			pRobotShape->color_wires[i].G = color_t;	pRobotShape->color_wires[i].B = 0;}
			if (color_h == 1) {pRobotShape->color_wires[i].R = color_q;		pRobotShape->color_wires[i].G = 1;			pRobotShape->color_wires[i].B = 0;}
			if (color_h == 2) {pRobotShape->color_wires[i].R = 0;			pRobotShape->color_wires[i].G = 1;			pRobotShape->color_wires[i].B = color_t;}
			if (color_h == 3) {pRobotShape->color_wires[i].R = 0;			pRobotShape->color_wires[i].G = color_q;	pRobotShape->color_wires[i].B = 1;}
			if (color_h == 4) {pRobotShape->color_wires[i].R = color_t;		pRobotShape->color_wires[i].G = 0;			pRobotShape->color_wires[i].B = 1;}
	
			// if cable force is more than maximum set cable color to PINK
			if (GetRobotDoc()->F(i,0) >= GetRobotDoc()->fmax) 
			{
				pRobotShape->color_wires[i].R = 255;
				pRobotShape->color_wires[i].G = 0;
				pRobotShape->color_wires[i].B = 255;
			}

			// if cable force is more than minimum set cable color to BLUE
			if (GetRobotDoc()->F(i,0) <= GetRobotDoc()->fmin) 
			{
				pRobotShape->color_wires[i].R = 0;
				pRobotShape->color_wires[i].G = 0;
				pRobotShape->color_wires[i].B = 255;
			}
		}
	}
	else
	{
		// if no coloring is desired, set all colors to winches color
		pRobotShape->color_wires.assign(GetRobotDoc()->getNow(), pRobotShape->color_winches) ;
	}

	// copy the latest platform position to the data model of the scene graph
	CFrame* pFrame = m_Scene.getFrame(hGlPlatform);
	pFrame->r.x = GetRobotDoc()->r.x();
	pFrame->r.y = GetRobotDoc()->r.y();
	pFrame->r.z = GetRobotDoc()->r.z();

	CMatrix3 Rtmp;
	for (int i=0; i<3; i++)
	{
		for (int j=0; j<3; j++)
		{
			Rtmp(i+1,j+1) = GetDocument()->robotDoc.R(i,j);
		}
	}
	pFrame->R = Rtmp;

	// draw the openGL scene
	m_Scene.RenderScene();
}

// CWireCenterView-Diagnose

#ifdef _DEBUG
void CWireCenterView::AssertValid() const
{
	CView::AssertValid();
}

void CWireCenterView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CWireCenterDoc* CWireCenterView::GetDocument() const // Nicht-Debugversion ist inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CWireCenterDoc)));
	return (CWireCenterDoc*)m_pDocument;
}

PCRL::CRobotDocument* CWireCenterView::GetRobotDoc() const // Nicht-Debugversion ist inline
{
	return  &static_cast<CWireCenterDoc*>(m_pDocument)->robotDoc;  
}

#endif //_DEBUG

/*! call the invalidate function to trigger a redraw of the openGL windows.
 */
void CWireCenterView::Invalidate(BOOL bErase)
{
	CView::Invalidate(bErase);
}

// CWireCenterView-Meldungshandler

//! return true for a openGL window;
BOOL CWireCenterView::OnEraseBkgnd(CDC* pDC)
{
	// override the default method since openGL takes care of erasing the background
	return TRUE; // CView::OnEraseBkgnd(pDC);
}

int CWireCenterView::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CView::OnCreate(lpCreateStruct) == -1)
		return -1;

	// initialize the openGL framework
	m_Scene.InitializeOpenGL(this);
	
	CWireCenterDoc* pDoc = GetDocument();
	PCRL::CRobotDocument *pRD = &GetDocument()->robotDoc;
	pHull = new PCRL::CShapeWorkspaceHull(pRD->WSHull);
	pWDH = new PCRL::CShapeWorkspaceDiffHull(pRD->WSHull);
	pRobotShape = new PCRL::CShapeRobot(*pRD,pRD->r,pRD->R, &pRD->l); // CRobotDocument is casted to CRobotData which does not contain r,R,l -> extra parameters
	pInterference = new PCRL::CShapeInterference(pRD->Interference);
	pCoordinatePlanes = new CGLShapeCoordinatePlanes;
	pGroundPlanes = new CGLShapeGroundPlane;
	pWireSpan = new PCRL::CShapeWireSpan(pRD->WSHull,*pRD,pRD->R);
	pWireSpanCone = new PCRL::CShapeWireSpanCone(pRD->WSHull,*pRD,pRD->R);
	pCS = new PCRL::CShapeCrosssection(pRD->Crosssection);
	pPlatformForces = new PCRL::CShapePlatformForces(*pRD,pRD->r, pRD->R, pRD->l, pRD->F, pRD->w);
	pWinchPulleys = new PCRL::CShapeWinchPulleys(*pRD, pRD->r, pRD->R);
	pNcLineStrip = new PCRL::CShapePoseListKinetostatic(PoseList);
	pPosePropertyEvaluation = new PCRL::CShapePosePropertyEvaluation(PoseList, GetDocument()->Evaluator);

	// add one coordinate frame at the origin
	CFrame K;
	m_Scene.adjustView(&K);
	This = this;

	// we add shapes controlled by wire center to layer -1
	TShapeAttribute SA;
	SA.iLayer = -1;

	// add the shapes with dynamic memory attribute to delete the memory automatically
	m_Scene.addShape(*pHull,&SA,true);
	m_Scene.addShape(*pWDH,&SA,true);
	m_Scene.addShape(*pRobotShape,&SA,true);
	m_Scene.addShape(*pInterference,&SA,true);
	m_Scene.addShape(*pWireSpan,&SA,true);
	m_Scene.addShape(*pWireSpanCone,&SA,true);
	m_Scene.addShape(*pCS,&SA,true);
	m_Scene.addShape(*pCoordinatePlanes,&SA,true);
	m_Scene.addShape(*pGroundPlanes,&SA,true);
	
	// dont set the path generator dynamic memory attribute, because PathGenerator is not created by "new"!
	m_Scene.addShape(PathGenerator,&SA,false);
	//m_Scene.addShape(HullShape,&SA);
	
	m_Scene.addShape(*pNcLineStrip,&SA,true); //no dynamic memory, could conflicting the pose list construct
	m_Scene.addShape(*pPlatformForces, &SA,true);
	m_Scene.addShape(*pWinchPulleys, &SA,true);

	m_Scene.addShape(*pPosePropertyEvaluation, &SA, true);

	// register the callback funcion draw() of this class for openGL rendering 
	m_Scene.addShape(*this,&SA,false); 

	// add one movable coordiante system to the scene graph
	hGlPlatform = m_Scene.createFrame();

	// add shapes to the moving frame
	SA.iFrame = hGlPlatform;
	m_Scene.addShape(WrenchSet,&SA);
	
	return 0;
}

//! this function implements openGL painting directly within the MFC framework
//! by overriding the virtual draw() method from IPAGL::CGLShape. 
void CWireCenterView::draw()
{
	PCRL::CApplicationRequirement *pAR = &GetDocument()->robotDoc.AppReq;

	glColor3d(0.8,0.8,0.8);

	if (m_bRequirementBoxVisible)
	{
		// set appearance
		glEnable (GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
		// draw the desired workspace box
		glColor4d(0,1,0,0.3);
		glPushMatrix();
		glTranslated((pAR->maxWS.x()+pAR->minWS.x())/2, (pAR->maxWS.y()+pAR->minWS.y())/2, (pAR->maxWS.z()+pAR->minWS.z())/2);
		IPAGL::makeBox(pAR->maxWS.x()-pAR->minWS.x(), pAR->maxWS.y()-pAR->minWS.y(), pAR->maxWS.z()-pAR->minWS.z());
		glPopMatrix();

		// draw the installation space 
		glColor4d(1,0,0,0.3);
		glPushMatrix();
		glTranslated((pAR->maxIS.x()+pAR->minIS.x())/2, (pAR->maxIS.y()+pAR->minIS.y())/2, (pAR->maxIS.z()+pAR->minIS.z())/2);
		IPAGL::makeBox(pAR->maxIS.x()-pAR->minIS.x(), pAR->maxIS.y()-pAR->minIS.y(), pAR->maxIS.z()-pAR->minIS.z());
		glPopMatrix();

		glDisable(GL_BLEND);
	}

	if (m_bRoiVisible)
	{
		PCRL::CBox* pRoi = &GetDocument()->Roi;
		// set appearance
		glEnable (GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

		// draw the desired workspace box
		glColor4d(0,0,1,0.3);
		glPushMatrix();
		glTranslated((pRoi->upper().x()+pRoi->lower().x())/2, (pRoi->upper().y()+pRoi->lower().y())/2, (pRoi->upper().z()+pRoi->lower().z())/2);
		IPAGL::makeBox(pRoi->upper().x()-pRoi->lower().x(), pRoi->upper().y()-pRoi->lower().y(), pRoi->upper().z()-pRoi->lower().z());
		glPopMatrix();

		glDisable(GL_BLEND);
	}

	if (m_bPoseEstimateBoxSpheresVisible)
	{
		m_bPoseEstimateBoxVisible = true;
		double planes[] =  {-1, 0, 0, GetRobotDoc()->fkPoseEstimateBox_ub(0), 1, 0, 0, -GetRobotDoc()->fkPoseEstimateBox_lb(0), 0,-1, 0, GetRobotDoc()->fkPoseEstimateBox_ub(1), 0, 1, 0, -GetRobotDoc()->fkPoseEstimateBox_lb(1), 0, 0,-1,GetRobotDoc()->fkPoseEstimateBox_ub(2), 0, 0, 1,-GetRobotDoc()->fkPoseEstimateBox_lb(2)}; // {-1, 0, 0, pI->xmax, 1, 0, 0, -pI->xmin, 0,-1, 0, pI->ymax, 0, 1, 0, -pI->ymin, 0, 0,-1, pI->zmax, 0, 0, 1, -pI->zmin};
		for (int i=0; i<6; i++)
		{
			glEnable(GL_CLIP_PLANE0+i);
			glClipPlane(GL_CLIP_PLANE0+i,planes+4*i);
		}
		
		for (int i=0; i<GetRobotDoc()->getNow(); i++)
		{
			glPushMatrix();
			glTranslated(GetRobotDoc()->getBase(i).x(),GetRobotDoc()->getBase(i).y(),GetRobotDoc()->getBase(i).z());
			IPAGL::makeSphere(GetRobotDoc()->l(i)+GetRobotDoc()->getPlatform(i).norm());
			glPopMatrix();				
		}
		for (int i=0; i<6; i++)
			glDisable(GL_CLIP_PLANE0+i);
	}
	
	if (m_bPoseEstimateBoxVisible)
	{
		glColor4d(0,0,1,0.3);
		glPushMatrix();
		
		//GetRobotDoc()->doPoseEstimation();
		IPAGL::makeWireBox(GetRobotDoc()->fkPoseEstimateBox_lb(0),GetRobotDoc()->fkPoseEstimateBox_ub(0),GetRobotDoc()->fkPoseEstimateBox_lb(1),GetRobotDoc()->fkPoseEstimateBox_ub(1),GetRobotDoc()->fkPoseEstimateBox_lb(2),GetRobotDoc()->fkPoseEstimateBox_ub(2));
		glPopMatrix();
	}

	// resize the ground plane
	Vector3d bbmin, bbmax;
	GetRobotDoc()->getBoundingBoxBase(bbmin,bbmax);

	// set the new boundingbox dimension in the pHull object (for parallel projection)
	pHull->bbMin = bbmin;
	pHull->bbMax = bbmax;

	double c1[3], c2[3];
	c1[0] = bbmin.x()-(bbmax.x()-bbmin.x())*0.2;
	c1[1] = bbmin.y()-(bbmax.y()-bbmin.y())*0.2;
	c1[2] = min(0.0,bbmin.z());
	c2[0] = bbmax.x()+(bbmax.x()-bbmin.x())*0.2;
	c2[1] = bbmax.y()+(bbmax.y()-bbmin.y())*0.2;
	c2[2] = min(0.0,bbmin.z());
	pGroundPlanes->setGroundSize(c1,c2);
/*	// draw the trace object
	for (int i=0; i<GetDocument()->trace.size(); i++)
	{
		CFrame* K = GetDocument()->trace[i];
		glPushMatrix();
			CVector3 r = K->R*K->r;
			double m[] ={K->R.e1.x, K->R.e1.y, K->R.e1.z, 0.0,
						 K->R.e2.x, K->R.e2.y, K->R.e2.z, 0.0,
						 K->R.e3.x, K->R.e3.y, K->R.e3.z, 0.0,
						 r.x,       r.y,       r.z,       1.0}; 
			glMultMatrixd(m);
			IPAGL::makeFrame();
		glPopMatrix();
	}*/
	
	if (m_bWorkspaceGridVisible)
	{
		vector<Vector3d> 
			*pVertex = &(GetDocument()->robotDoc.WSHull.vertices),
			*pWS = &(GetDocument()->robotDoc.WSGrid.workspace);
		glPointSize(5);
		glDisable(GL_LIGHTING);
		glColor3d(0,1,0);
		glBegin(GL_POINTS);
			for (unsigned int i=0; i<pWS->size(); i++)
				glVertex3d(pWS->at(i).x(),pWS->at(i).y(),pWS->at(i).z());
		glEnd();

		// draw the points outside the workspace in red
		if (m_bWorkspaceGridOutVisible)
		{
			vector<Vector3d> 
				*pWS_out = &(GetDocument()->robotDoc.WSGrid.workspace_out);
			glPointSize(5);
			glDisable(GL_LIGHTING);
			glColor3d(1,0,0);
			glBegin(GL_POINTS);
				for (unsigned int i=0; i<pWS_out->size(); i++)
					glVertex3d(pWS_out->at(i).x(),pWS_out->at(i).y(),pWS_out->at(i).z());
			glEnd();
		}
		// reset to default value
		glEnable(GL_LIGHTING);
		glPointSize(1);
	}

	if (m_bWorkspaceInnerBoxVisible)
	{
		CVector3 v_min(GetDocument()->robotDoc.WSGrid.bbmin.x(),GetDocument()->robotDoc.WSGrid.bbmin.y(),GetDocument()->robotDoc.WSGrid.bbmin.z());
		CVector3 v_max(GetDocument()->robotDoc.WSGrid.bbmax.x(),GetDocument()->robotDoc.WSGrid.bbmax.y(),GetDocument()->robotDoc.WSGrid.bbmax.z());

		// set appearance
		glEnable (GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

		// draw the desired workspace box
		glColor4d(0,0,1,0.5);
		glPushMatrix();
		glTranslated((v_min.x+v_max.x)/2.0, (v_min.y+v_max.y)/2.0,(v_min.z+v_max.z)/2.0);
		IPAGL::makeBox(v_max.x-v_min.x, v_max.y-v_min.y, v_max.z-v_min.z);
		glPopMatrix();

		glDisable(GL_BLEND);
	}

	if (m_bPoseListControlPointsVisible)
	{
		glPointSize(5);
		glDisable(GL_LIGHTING);
		glColor3d(ColorControlPoints.R, ColorControlPoints.G, ColorControlPoints.B);
		glBegin(GL_POINTS);
		for (auto itor = PoseList.begin(); itor!=PoseList.end(); ++itor)
		{
			Vector3d r = (**itor).r;
			glVertex3d(r.x(),r.y(),r.z());
		}
		glEnd();
		// reset to default value
		glEnable(GL_LIGHTING);
		glPointSize(1);
	}
}

void CWireCenterView::OnSize(UINT nType, int cx, int cy)
{
	CView::OnSize(nType, cx, cy);

    // check for invalid size of the window
	if ( 0 >= cx || 0 >= cy )
		return;
	// pass parameters to the openGL scene object;
	m_Scene.OnSize(cx,cy);
}

//! destroy the openGL scene and delete the drawing context
void CWireCenterView::OnDestroy()
{
	CView::OnDestroy();
	
	/* DONT clean up memory of openGL objects, because they will be 
	cleaned up automatically when this class is destroyed
	
	CWireCenterView:m_Scene (CGLSceneGraph)
		-> CGLSceneGraph.root (CGLShapeList)
		-> ~CGLShapeList (calls a clear function, which deletes all elements)
	
	delete pHull;
	delete pWDH;
	delete pRobotShape;
	delete pInterference;
	delete pCoordinatePlanes;
	delete pGroundPlanes;
	delete pWireSpan;
	delete PWireSpanCone;
	delete pCS;
	delete pPlatformForces;
	delete pWinchPulleys;
	
	m_Scene.OnDestroy();
	*/
}

//! handle mouse move and clicks to the openGL window for selecting, rotation, zooming and panning
void CWireCenterView::OnLButtonDown(UINT nFlags, CPoint point) 
{
	if (m_Scene.bEnableDragging)
	{
		// do the selection test
		m_Scene.selectionTest(point.x,point.y);

		// parse the results
		std::list<double>::iterator itor = m_Scene.selectionResult.begin();
		if (m_Scene.selectionResult.size()!=0)
		{
			// if the grabbed object is one of the robot world frame ...
			if ((*itor) == PCRL::isIdRobotWorldFrame)
			{
				itor++; // get next entry in list => "subobject"
				//...look, which axis is grabbed and colorize it
				if ((*itor) == 1) 
					pRobotShape->iHighlightedFrameDimension = 1;
				else if ((*itor) == 2) 
					pRobotShape->iHighlightedFrameDimension = 2;
				else if ((*itor) == 3) 
					pRobotShape->iHighlightedFrameDimension = 3;
				//set in dragging mode
				m_Scene.bIsDragging = true;
			}

			// if the grabbed object is the robot frame itself: colorize it
			if ((*itor) == PCRL::isIdRobotFrame)
			{
				pRobotShape->iHighlightedFrameDimension = 4;	
				//set in dragging mode
				m_Scene.bIsDragging = true;
			}

			//if the grabbed object is one of the base frames ...
			if ((*itor) == PCRL::isIdBaseFrames)
			{
				// set in dragging mode
				m_Scene.bIsDragging = true;
			}
				
			// if the grabbed object is the hull ...
			if ((*itor) == PCRL::isIdHull)
			{
				itor++; // get next entry in list => "subobject"
				int triangle_number = (int)(*itor);
				// ...find the corresponding element and return the vertex coordinates
				cout << "##### FOUND WORKSPACE HULL #####" << endl;
				cout << "Triangle No." << triangle_number << " : " << endl;
				printf("\tVertex 1: x: %.5f y: %.5f z: %.5f \n",
					GetRobotDoc()->WSHull.vertices[GetRobotDoc()->WSHull.Triangles[triangle_number].i].x(),
					GetRobotDoc()->WSHull.vertices[GetRobotDoc()->WSHull.Triangles[triangle_number].i].y(),
					GetRobotDoc()->WSHull.vertices[GetRobotDoc()->WSHull.Triangles[triangle_number].i].z());
				printf("\tVertex 2: x: %.5f y: %.5f z: %.5f \n",
					GetRobotDoc()->WSHull.vertices[GetRobotDoc()->WSHull.Triangles[triangle_number].j].x(),
					GetRobotDoc()->WSHull.vertices[GetRobotDoc()->WSHull.Triangles[triangle_number].j].y(),
					GetRobotDoc()->WSHull.vertices[GetRobotDoc()->WSHull.Triangles[triangle_number].j].z());
				printf("\tVertex 3: x: %.5f y: %.5f z: %.5f \n",
					GetRobotDoc()->WSHull.vertices[GetRobotDoc()->WSHull.Triangles[triangle_number].k].x(),
					GetRobotDoc()->WSHull.vertices[GetRobotDoc()->WSHull.Triangles[triangle_number].k].y(),
					GetRobotDoc()->WSHull.vertices[GetRobotDoc()->WSHull.Triangles[triangle_number].k].z());
			}
		}
	}

	m_Scene.OnLButtonDown(point);
	CView::OnLButtonDown(nFlags, point);
	Invalidate();
}

//! handle mouse move and clicks to the openGL window for selecting, rotation, zooming and panning
void CWireCenterView::OnMouseMove(UINT nFlags, CPoint point) 
{
	if (m_Scene.OnMouseMove(point))
		InvalidateRect(FALSE);
	
	//if any dragging action is activated... handle it here
	if (m_Scene.bEnableDragging&&(!m_Scene.selectionResult.empty()))
	{
		//get a iterator to go manually through the list
		std::list<double>::iterator itor = m_Scene.selectionResult.begin();
		
		if (m_Scene.selectionResult.size()!=0)
		{
			//if the grabbed object is one of the robot world frame ...
			if ((*itor) == PCRL::isIdRobotWorldFrame)
			{
				// get on level deeper in the clicked-object naming description
				itor++;

				// enable rotating when right mouse is pressed too
				if (nFlags & MK_RBUTTON)
				{
					// create a quaternion from the Eigen::Library and set it to an identity quaternion
					Eigen::Quaterniond quat;
					quat.setIdentity();

					// map mouse movement to rotation angle calculated by the mouse pos differences
					double drotation_angle = ((m_Scene.GetSavedMousePos().x-point.x)+(m_Scene.GetSavedMousePos().y-point.y))*0.005;
					
					// if the appropriate axis is selected, alter the quaterion to get an rotation around the selected axis
					// maybe it would be better to use a function like GetLastClickedObject() or similar, to avoid the dependency on the colouring of the robot frame arrows
					if ((*itor)==1) 
						quat.x() = sin(drotation_angle);
					else if ((*itor)==2) 
						quat.y() = sin(drotation_angle);
					else if ((*itor)==3) 
						quat.z() = sin(drotation_angle);
					
					// normalize the quaternion to prevent "exploding" from platform if mouse moves too quickly ... pretty awkward
					quat.normalize();

					// apply the rotation quaternion on the rotation matrix
					GetRobotDoc()->R = quat.toRotationMatrix()*GetRobotDoc()->R;
				}
				else
				{
					double x3D,y3D,z3D;
					m_Scene.convertMovement2Dto3D(point,x3D,y3D,z3D,GetRobotDoc()->r.x(),GetRobotDoc()->r.y(),GetRobotDoc()->r.z());

					// assign new position to robot
					// maybe it would be better to use a function like GetLastClickedObject() or similar, to avoid the dependency on the colouring of the robot frame arrows
					if ((*itor)==1) 
						GetRobotDoc()->r.x() = x3D;
					else if ((*itor) == 2) 
						GetRobotDoc()->r.y() = y3D;
					else if ((*itor) == 3) 
						GetRobotDoc()->r.z() = z3D;
					// or : move in plane parallel to screen plane if the green frame cubes are grabbed
					else if ((*itor) == 4) 
					{
						GetRobotDoc()->r.x() = x3D;
						GetRobotDoc()->r.y() = y3D;
						GetRobotDoc()->r.z() = z3D;
					}
				}

				//update the Pose-Pane
				UpdatePosePane();
				if (m_bInstantPosePaneUpdate) OnUpdatePose(0,0);
			}

			//if the grabbed object is one of the robot world frame ...
			if ((*itor) == PCRL::isIdRobotFrame)
			{
				double x3D,y3D,z3D;
				m_Scene.convertMovement2Dto3D(point,x3D,y3D,z3D,GetRobotDoc()->r.x(),GetRobotDoc()->r.y(),GetRobotDoc()->r.z());

				//assign new position to robot
				//move in plane parallel to screen plane if the green frame cubes are grabbed
				GetRobotDoc()->r.x() = x3D;
				GetRobotDoc()->r.y() = y3D;
				GetRobotDoc()->r.z() = z3D;

				//update the Pose-Pane
				UpdatePosePane();
				if (m_bInstantPosePaneUpdate) 
					OnUpdatePose(0,0);
			}

			//if the grabbed object is one of the base frames ...
			if ((*itor) == PCRL::isIdBaseFrames)
			{
				//get on level deeper in the clicked-object naming description; this is the number of the selected base frame
				itor++;
				int base_frame_number = (int)(*itor);
				
				//get current base pos
				Eigen::Vector3d current_base_vector = GetRobotDoc()->getBase(base_frame_number);
				
				//get on level deeper in the clicked-object naming description; this is the number of the selected base frame AXIS (0: move in plane, 1:x, 2:y, 3:z)
				itor++;

				//make some preperations for movement recognition
				double x3D,y3D,z3D;
				m_Scene.convertMovement2Dto3D(point,x3D,y3D,z3D,current_base_vector.x(),current_base_vector.y(),current_base_vector.z());

				//if on of the axis is selected...
				if ((*itor) ==1 )
					current_base_vector.x() = x3D; //move the selected base frame as wished (x-axis)
				else if ((*itor) == 2) 
					current_base_vector.y() = y3D; //move the selected base frame as wished (y-axis)
				else if ((*itor) == 3) 
					current_base_vector.z() = z3D; //move the selected base frame as wished (z-axis)
				else if ((*itor) == 4) 
				{
					//move the selected base frame as wished (all axis)
					current_base_vector.x() = x3D;
					current_base_vector.y() = y3D;
					current_base_vector.z() = z3D;
				}

				//apply the moved vector
				GetRobotDoc()->setBase(base_frame_number,current_base_vector);

				//update the interactive geometry pane
				CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
				if (pMF) 
				{
					pMF->m_wndGeomPane.updateAllData(true);
				}

				//update the Pose-Pane
				UpdatePosePane();
				if (m_bInstantPosePaneUpdate) 
					OnUpdatePose(0,0);
			}

			//redraw window to render the current view and see the result immediately when moving
			RedrawWindow();
		}
	}
	
	//save the last mouse position to calculate the differential mouse movement
	m_Scene.SaveMousePos(point);

	//call handler from CWnd class
	CView::OnMouseMove(nFlags, point);
}

//! handle mouse move and clicks to the openGL window for rotation, zooming and panning
void CWireCenterView::OnLButtonUp(UINT nFlags, CPoint point) 
{
	//reset the highlighted worldframe and repaint the scene
	pRobotShape->iHighlightedFrameDimension = 0;

	//reset the dragging mode and repaint the scene
	m_Scene.selectionResult.clear();
	m_Scene.bIsDragging = false;
	m_Scene.RenderScene();
	
	m_Scene.OnLButtonUp(point);

	CView::OnLButtonUp(nFlags, point);
}

//! handle mouse move and clicks to the openGL window for rotation, zooming and panning
void CWireCenterView::OnRButtonDown(UINT nFlags, CPoint point) 
{
	m_Scene.OnRButtonDown(point);

	CView::OnRButtonDown(nFlags, point);
}

//! handle mouse move and clicks to the openGL window for rotation, zooming and panning
void CWireCenterView::OnRButtonUp(UINT nFlags, CPoint point) 
{
	m_Scene.OnRButtonUp(point);

	CView::OnRButtonUp(nFlags, point);
}

void CWireCenterView::getWorkspaceOptionsFromSidebar()
{
	PCRL::CRobotData *pRobot = GetRobotDoc();
	PCRL::CWorkspaceHull* pHull = &(GetRobotDoc()->WSHull);

	// read parameter from DlgBar
	CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();

	CMFCRibbonEdit* pEdit = DYNAMIC_DOWNCAST(CMFCRibbonEdit,pMF->m_wndRibbonBar.FindByID(ID_ITERATION_EDIT));
	if (pEdit)
	{
		CString str;
		str = pEdit->GetEditText();
		int i = atoi(str);
		if (i<=0)
		{
			printf("Invalid argument for iteration depth. Setting iteratons to 2\n");
			pEdit->SetEditText("2");
			pHull->setIterations(2);
		}
		else
			pHull->setIterations(i);
	}

	pEdit = DYNAMIC_DOWNCAST(CMFCRibbonEdit,pMF->m_wndRibbonBar.FindByID(ID_EPS_EDIT));
	if (pEdit)
	{
		CString str;
		str = pEdit->GetEditText();
		double eps = atof(str);
		if (eps<=0)
		{
			printf("Invalid argument for parametrs accuracy eps. Setting eps=0.001\n");
			pEdit->SetEditText("0.001");
			pHull->setEps(0.001);
		}
		else
			pHull->setEps(eps);
	}

	if (m_bAutoApplyOrientation)
	{
//		CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
		Vector3d r;
		Matrix3d R;
		pMF->m_wndPosePane.GetPoseParameter(r,R);

		//! \todo Once again, we have a decentralized occurance of the rotation
		//! parametrization that have to be centralized. Anyway, at the moment
		//! it seems to be in line with other such definitions.

		pHull->setOrientation(R);
	}
}

//! copy the values for iteration depth and eps into the ribbon 
void CWireCenterView::setWorkspaceOptionsInSidebar()
{
	PCRL::CWorkspaceHull* pHull = &(GetRobotDoc()->WSHull);
	
	// set parameter in the ribbon bar
	CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
	
	CMFCRibbonEdit* pEdit = DYNAMIC_DOWNCAST(CMFCRibbonEdit,pMF->m_wndRibbonBar.FindByID(ID_ITERATION_EDIT));
	if (pEdit)
	{
		CString str;
		str.Format("%d",pHull->getIterations());
		pEdit->SetEditText(str);
	}

	pEdit = DYNAMIC_DOWNCAST(CMFCRibbonEdit,pMF->m_wndRibbonBar.FindByID(ID_EPS_EDIT));
	if (pEdit)
	{
		CString str;
		str.Format("%3.3f",pHull->getEps());
		pEdit->SetEditText(str);
	}
}

//! thread function for parallel workspace computation.
//! do a partial calculation of some hull vertices in a thread
DWORD WINAPI func(LPVOID lpParameter)
{
	PCRL::CWorkspaceHull* pHull = (PCRL::CWorkspaceHull*)lpParameter;
	time_t start = clock();
	pHull->calculateWorkspaceCore();
	pHull->setCalculationTime((int)(clock()-start));
	return 0;
}

/*! compute the workspace hull of the robot
 *  this functions includes parts of the implementation  for multi-core 
 *  parallel processing. to this end we do not want thread specific 
 *  implementation in WireLib, therefore, it is an issue for WireCenter 
 *  to handle creating and timing of worker threads. However, WireLib 
 *  should provide more efficient support for multi-threading e.g. my 
 *  management of distribution and collection of results to multiply
 *  objects.
 */
void CWireCenterView::OnBnClickedCalculateworkspaceButton()
{
	CWireCenterDoc *pDoc = GetDocument();
	
	string configName = "\\__tempAlgorithmConf__.xml";
	configName.insert(0,pDoc->getWireCenterPath());


	PCRL::CRobotData *pRobot = GetRobotDoc();
	PCRL::CWorkspaceDifferentialHull* pHull = &GetRobotDoc()->WSHull;

	getWorkspaceOptionsFromSidebar();

	bool bThreadingSafe = true;
	int method, criteria;
	PCRL::CRobotDocument *pRD = GetRobotDoc();
	pRD->ForceDistribution.getMethod(method, criteria);
	
	if (method == 8 && m_bUseThreading)
	{
		bThreadingSafe = false;
		cout << "Multithreading is disabled for  Quadratic programming\n";
	}

	if (!m_bUseThreading || !bThreadingSafe)
		pHull->calculateWorkspace();
	else
	{
		// we do the computation in parallel threads:
		// firstly distribute the task into 2 * #CPU parts
		// then, compute the partial results in independent threads
		// finally, collect the vertex data in the main "WSHull" object

		// 1. generate the data model, distribute the work and start the independant threads
		// get the number of available CPU cores
		SYSTEM_INFO sysinfo;
		GetSystemInfo( &sysinfo );
		const int threads = sysinfo.dwNumberOfProcessors*2;
		pHull->makeSphere();
		// create copies of the whole robot document to get independent instances of all objects; slow but it seems to work fine
		PCRL::CRobotDocument** pDocs = new PCRL::CRobotDocument*[threads];			
		HANDLE* myThreads = new HANDLE[threads];
		//! \todo save the current settings to disk; it would be nice to save them in memory instead but the current
		//! implementation only supports writing data to disk
		GetRobotDoc()->saveAlgorithmConfiguration(configName);
		// 2. do things in parallel
		for (int i=0; i<threads; i++)
		{
			// we create a new object ...
			pDocs[i] = new PCRL::CRobotDocument(GetRobotDoc()->getNow(),GetRobotDoc()->getMotionPattern());;
			// and use the assignment operator to copy the geometry and the kinematic state
			*pDocs[i] = *GetRobotDoc();
			
			// copy the vertex data to the individual threads; 
			//! \todo This should be an implementation of the CWorkspace class
			//! the current implementation consumses a lot of resources since we
			//! must copy more data than we actually need.
			//! A main problem for the implementation is to generate independent copies of the
			//! objects that actually do the computations. We need copies of these objects
			//! since we cannot expect the single instances to be thread safe.
			pDocs[i]->loadAlgorithmConfiguration(configName);
			pDocs[i]->WSHull.getAlgorithmParameter(*pHull);
			pDocs[i]->WSHull.makeSphere();
			pDocs[i]->WSHull.setVertexRoi(i*pHull->vertices.size()/threads, (i+1)*pHull->vertices.size()/threads);
			// start all worker threads
			myThreads[i] = CreateThread(0,0,func,&(pDocs[i]->WSHull),0,0);
		}

		// delete the temporary file
		DeleteFile(configName.c_str());

		// calculation is running in the other threads; we wait for all threads to end...
		if (WaitForMultipleObjects(threads,myThreads,TRUE,100000) == WAIT_TIMEOUT)
		{
			printf("Error in parallel threads, 100 second time out reached\n");
			// we should kill the threads that are still running...
		}

		// 3. collect the data from the worker theads
		int ct = 0;
		for (int i=0; i<threads; i++)
		{
			if (!pHull->copyVertexData(pDocs[i]->WSHull,i*pHull->vertices.size()/threads, (i+1)*pHull->vertices.size()/threads))
				printf("Error in copyVertexData of %i thread\n",i);
			ct += pDocs[i]->WSHull.getCalculationTime();
		}

		pHull->FinishedTriangles = pHull->Triangles.size();
		pHull->setCalculationTime(ct);

		// clean up the dynamically reserved memory
		delete [] myThreads;
		for (int i=0; i<threads; i++)
			delete pDocs[i];
		delete [] pDocs;
	}

	pHull->calculateWorkspaceProperties();
	OnUpdate(0,0,0);
}

/*! \remark This is the foreseen place to copy the hull data model into 
 *  a distince structure in the open GL scene graph in order
 *  to seperate the current state of what can be seen in the GUI.
 *  However, the function was not provided yet ...
 */
void CWireCenterView::UpdateWorkspaceHull()
{
	/*
	//get the hull vertices and save in pHull (remember: pHull(CWorkspaceHull) != pHull(CShapeWorkspaceHull)
	PCRL::CWorkspaceHull* pHull = &GetRobotDoc()->WSHull;
	
	//clear the object
	HullShape.clear();
	//set the color
	HullShape.setColor(0.0,0.0,1.0);
	
	// create the triangles based on the Hull-Triangles x-coordinates
	for (unsigned int i=0; i<pHull->Triangles.size(); i++)
	{
		CVector3 
			a(&pHull->vertices[pHull->Triangles[i].i].x()),
			b(&pHull->vertices[pHull->Triangles[i].j].x()),
			c(&pHull->vertices[pHull->Triangles[i].k].x());
		HullShape.addTriangle(a,b,c);
	}
	*/
}

/*! Update all controls showing robot properties including
 *  - Set the workspace statistics data in the interactive geometry pane
 *  
 */
void CWireCenterView::UpdateStatistics()
{
	PCRL::CWorkspaceHull* pHull = &GetRobotDoc()->WSHull;
	CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();

	pMF->m_wndGeomPane.setWorkspaceProperty(pHull);
	pMF->m_wndGeomPane.setWorkspaceGridProperty(&GetRobotDoc()->WSGrid);
	pMF->m_wndGeomPane.setWorkspaceCrosssectionProperty(&GetRobotDoc()->Crosssection);
}

/*! new implementation of the update pose (or more precisely) analyze pose
 *  function. The refactored implementation should be more modular, fault tolerant
 *  and compatible with the CRobotDocument
 */
LRESULT CWireCenterView::OnUpdatePose(WPARAM wParam, LPARAM lParam)
{
	time_t start = clock();
	//cout<<"*~*~*~*~*~*~====POSE UPDATE====~*~*~*~*~*~*" << endl;
	CWireCenterDoc *pDoc = GetDocument();
	PCRL::CRobotDocument *pRD = GetRobotDoc();
	CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
	GetRobotDoc()->doPoseEstimation();
	//pRD->setKinSettings(100,1, 2, 3, 4);
	// execute the python script for pose update
	CPythonInterface::getInstance().runSystemScript("OnUpdatePose");

	// get the pose from the dialog box and store it in the robot document class

	//pMF->m_wndPosePane.GetPoseParameter(pRD->r,pRD->R);
	//! \todo: This is still confusing: the orientation model is locally stored 
	//         and one can hardly keep track of the parameterization used. This 
	//         kind of calculations need to be centralized

	// hide the control first, then update the parameter and show it again
	//! \todo: Is hiding the PropertyGrid, set the values and show it again the only possibility to boost the draw process?
	pMF->m_wndPosePane.SetPropertyGridVisible(false);

	// compute the inverse kinematics with the standard kinematic model
	if (m_bAnalyzeInverseKinematics)
	{
		// caluclate inverse kinematics with simple method
		if (pRD->doInverseKinematics())
		{
			if (m_bVerboseMode == true) 
				cout << endl << "Cable length (standard-model): " << pRD->l << endl;
			
			pMF->m_wndPosePane.SetWireLength(pRD->getNow(),pRD->l.data());
			pMF->m_wndPosePane.SetValidCableLength(true);
		}
		else
		{
			if (m_bVerboseMode == true) 
				cout << "Inverse kinematics failed\n";
			
			pMF->m_wndPosePane.SetValidCableLength(false);
		}
	}

	if (m_bAnalyzeElastoGeometricalForwardKinematics)
	{

		// calculate simplified forward kinematics for cable length of standard
		// save the current set-point pose
		Vector3d r_old = pRD->r;
		Matrix3d R_old = pRD->R;

		if (!pRD->doElastoGeometricalForwardKinematics())
			if (m_bVerboseMode == true) 
				cout << "Elasto geometrical forward kinematics failed\n";
		else
		{
			if (m_bVerboseMode == true)
			{
				cout<< "Results from elasto geometrical forward kinematics:" << endl;
				cout<< "---------------------------------------------------" << endl << endl;
				cout<<"Estimated position r: "<< endl << pRD->r.transpose() << endl << endl;
				cout<<"Estimated orientation R: "<< endl << pRD->R.transpose() << endl << endl;
				cout<< "Cable forces f: " << pRD->F << endl << endl;
				cout<< "dr = " << (r_old-pRD->r).norm() << endl
					<< "dR = " << endl <<R_old-pRD->R << endl;		// \todo: Compute a resonable distance metric here; the matrix difference not not really meaningful
			}
		}

		/*MatrixXd w_res;
		MatrixXd w(6,1);
		Vector3d* u = new Vector3d[pRD->getNow()];
		w << 0,0,-9.81*pRD->platform_mass,0,0,0 ;
		pRD->Kinematics.computeResultantPlatformWrench(w_res, pRD->F, w, pRD->r, pRD->R, u);
		
		cout << "Resultant wrench (expected to be zero): " << endl << w_res.transpose() << endl;
		cout << "---------------------------------------------------" << endl;*/
		// restore the saved pose
		pRD->r = r_old;
		pRD->R = R_old;

		//delete [] u;
	}

	if (m_bCompareInverseKinematics)
	{
		// Create arrays to store the wirelengths and percentage differences
		MatrixXd lstandard; // Wirelengths from standard model
		MatrixXd  lpulley; // Wirelengths from pulley model
		MatrixXd  ldiff; // Percentage difference in wirelengths
		//Vector3d* C; // Wire exit point from the pulley calculated by doInverseKinematicsPulleyEx

		// calculate inverse kinematics with simple method
		if (pRD->doInverseKinematics())
		{
			if (m_bVerboseMode == true) 
				cout << endl << "Cable length (standard-model): " << pRD->l << endl;
		}
		lstandard = pRD->l;

		// Calculate inverse kinematics with the pulley method
		if (pRD->doInverseKinematicsPulley())
		{
			if (m_bVerboseMode == true) 
				cout << endl << "Cable length (pulley-model): " << pRD->l << endl;
		}
		lpulley = pRD->l;
		ldiff = pRD->l;
		// Calculate percentage differences
		for(int i = 0; i < pRD->getNow(); i++)
			ldiff(i) = ((lpulley(i)-lstandard(i))/lstandard(i))*100;
	
		if (m_bVerboseMode == true) cout << endl << "Difference(%): " << ldiff << endl;

		// Get the exit point from the pulley calculated by doInverseKinematicsPulleyEx
		if (m_bVerboseMode == true)
		{
			for(int i = 0; i < pRD->getNow(); i++)
			{
				cout << endl << "Winch" << i+1 << ":"<< endl;
				cout<<"x: " << pRD->C[i][0] << endl;
				cout<<"y: " << pRD->C[i][1] << endl;
				cout<<"z: " << pRD->C[i][2] << endl;
			}
		}
	}

	if (m_bAnalyzeForwardKinematics)
	{
		// calculate simplified forward kinematics for cable length of standard
		// save the current set-point pose
		Vector3d r_old = pRD->r;
		Matrix3d R_old = pRD->R;

		if ( !pRD->doForwardKinematics() )
		{
			if (m_bVerboseMode == true) 
				cout << "Calculation of forward kinematics failed\n";
		}
		else
		{
			if (m_bVerboseMode == true) 
			{
				cout << endl << "Estimated pose from forward kinematics is [r,R]: " << pRD->r.x() << " , " << pRD->r.y() << " , "  << pRD->r.z() << endl<< pRD->R << endl;
				Vector3d angles;
				PCRL::getXYZFromMatrix(angles,pRD->R);
				cout<< "angles:" << endl << angles << endl 
					<< "dr = " << (r_old-pRD->r).norm() << endl
					<< "dR = " << endl <<R_old-pRD->R << endl;		// \todo: Compute a resonable distance metric here; the matrix difference not not really meaningful
			}
		}
		// restore the saved pose
		pRD->r = r_old;
		pRD->R = R_old;
	}

	if (m_bAnalyzeForwardKinematicsDistance)
	{
		// calculate distance forward kinematics for cable length of standard model
		// save the current set-point pose
		Vector3d r_old = pRD->r;
 		Matrix3d R_old = pRD->R;
		if (!pRD->doForwardKinematicsDist())
			if (m_bVerboseMode == true) 
				cout << "Calculation of forward kinematics with distance method failed\n";
		else
		{
			if (m_bVerboseMode == true)
			{
				cout << "Estimated pose from distance forward kinematics is [r,R]: "<< pRD->r.x() << " , " << pRD->r.y() << " , "  << pRD->r.z() << endl<< pRD->R << endl;
				Vector3d angles;
				PCRL::getXYZFromMatrix(angles,pRD->R);
				cout<< "angles:" << angles << endl 
					<< "dr = "   << (r_old-pRD->r).norm() << endl
					<< "dR = "   << endl<<R_old-pRD->R << endl;		// \todo: Compute a resonable distance metric here; the matrix difference not not really meaningful
			}
		}
		// restore the saved pose
		pRD->r = r_old;
		pRD->R = R_old;
	}
	
	// compute the inverse kinematics with the pulley model
	if (m_bAnalyzeInverseKinematicsPulley)
	{
		// we do all computations here our self because robot document
		// has no states for computing beta and gamma
		MatrixXd beta(pRD->getNow(),1);
		MatrixXd gamma(pRD->getNow(),1);
		MatrixXd l(pRD->getNow(),1);
		// caluclate inverse kinematics with simple method
		if (pRD->Kinematics.doInverseKinematicsPulleyEx(pRD->r,pRD->R,l.data(),0,beta.data(),gamma.data()))
		{
			if (m_bVerboseMode == true) 
				cout << endl << "Cable length (pulley-model): " << pRD->l << endl;
			pMF->m_wndPosePane.SetPulleyWireLength(pRD->getNow(),l.data(),beta.data(),gamma.data(),0);
		}
		else
			if (m_bVerboseMode == true) cout << "Invese kinematics (pulley) failed\n";
	}

	if (m_bAnalyzeForwardKinematicsPulley)
	{
		// calculate pulley forward kinematics for cable length 
		// save the current set-point pose
		Vector3d r_old = pRD->r;
		Matrix3d R_old = pRD->R;
		if (!pRD->doForwardKinematicsPulley())
			if (m_bVerboseMode == true) 
				cout << "Calculation of forward kinematics with pulley method failed\n";
		else
		{
			if (m_bVerboseMode == true)
			{
				cout << "Estimated pose from pulley forward kinematics is [r,R]: "<< pRD->r.x() << " , " << pRD->r.y() << " , "  << pRD->r.z() << endl << pRD->R << endl;
				cout<< "dr = " << (r_old-pRD->r).norm() << endl
					<< "dR = " << endl<<R_old-pRD->R << endl;		// \todo: Compute a resonable distance metric here; the matrix difference not not really meaningful
			}
		}
		// restore the saved pose
		pRD->r = r_old;
		pRD->R = R_old;
			
		/*the following lines of code set the lengths back to the standard inverse kinematics 
		in order to ensure functionality of following code until a good interface to choose between kinematic methods is developed*/ 
		pRD->doInverseKinematics();
		pMF->m_wndPosePane.SetWireLength(pRD->getNow(),pRD->l.data());
	}
		
	if (m_bAnalyzeVelocityTransmission)
	{
		// compute the velocity transmission and the instantaneous power consumption
		// 
		if (pRD->getMotionPattern() == PCRL::CRobotData::MP3R3T)
		{
			double dlx[100];
			double dly[100];
			double dlz[100];
			double dla[100];
			double dlb[100];
			double dlc[100];
			MatrixXd w = MatrixXd::Zero(6,1);
			MatrixXd f_wires = MatrixXd::Zero(8,1);

			pRD->Kinematics.doInverseKinematicsVelocity(pRD->r,pRD->R,Vector3d(1,0,0),Vector3d(0,0,0),dlx);
			pRD->Kinematics.doInverseKinematicsVelocity(pRD->r,pRD->R,Vector3d(0,1,0),Vector3d(0,0,0),dly);
			pRD->Kinematics.doInverseKinematicsVelocity(pRD->r,pRD->R,Vector3d(0,0,1),Vector3d(0,0,0),dlz);
			pRD->Kinematics.doInverseKinematicsVelocity(pRD->r,pRD->R,Vector3d(0,0,0),Vector3d(1,0,0),dla);
			pRD->Kinematics.doInverseKinematicsVelocity(pRD->r,pRD->R,Vector3d(0,0,0),Vector3d(0,1,0),dlb);
			pRD->Kinematics.doInverseKinematicsVelocity(pRD->r,pRD->R,Vector3d(0,0,0),Vector3d(0,0,1),dlc);
			pRD->ForceDistribution.getDistribution(w,pRD->fmin, pRD->fmax,f_wires);
			//cout << f_wires << endl;
			if (m_bVerboseMode == true)
			{
				double P = 0;
				cout << "\nVelocity transmission\n";
				for (int i=0; i<pRD->getNow(); i++)
				{		
					cout<< "i: "
						<< dlx[i] << "\t"
						<< dly[i] << "\t"
						<< dlz[i] << "\t"
						<< dla[i] << "\t"
						<< dlb[i] << "\t"
						<< dlc[i] << "\n";
					P += dlx[i]*f_wires(i);
				}
				cout << "Power: " << P << endl << endl;
			}
		}
		else
			if (m_bVerboseMode == true) 
				cout << "Velocity and Power Computation is only implemented for 3R3T robots\n";
	}

	if (m_bAnalyzeDexterity)
	{
		// some calculations towards dexterity (preliminary versions)
		if (pRD->getStructureMatrix())
		{
			MatrixXd sv;
			pRD->ForceDistribution.getSingularValues(sv);
			if (m_bVerboseMode == true)
			{
				cout << "rank of A^T = " << pRD->ForceDistribution.getRank() << endl;
				cout << "column norm ||A^T||_inf = " << pRD->ForceDistribution.getMaxColumnNorm() << endl;
				cout << "row norm ||A||_inf = " << pRD->ForceDistribution.getMaxRowNorm() << endl << endl;
				cout << "singular values of A = " << sv << endl;
			}
			pMF->m_wndPosePane.SetDexterityMeasure(pRD->ForceDistribution.getMaxColumnNorm(),pRD->ForceDistribution.getMaxRowNorm(), sv(pRD->getDof()-1),sv(0));
		}
		else
			if (m_bVerboseMode == true) 
				cout << "Failed to compute structure matrix\n\n";
	}

	if (m_bAnalyzeStiffness)
	{
		// evaluate the stiffness of the robot
		pRD->Stiffness.StiffnessMatrix(pRD->r,pRD->R);
		double minStiffness = pRD->Stiffness.getMinimalStiffness();
		if (m_bVerboseMode == true) 
			cout << "minimal Stiffness: " << minStiffness << endl;
		pRD->Stiffness.StiffnessMatrixTranslational(pRD->r,pRD->R);
		if (m_bVerboseMode == true) 
			cout << "minimal translational stiffness: " << pRD->Stiffness.getMinimalStiffness() << endl<< endl;
		pMF->m_wndPosePane.SetStiffness(minStiffness,pRD->Stiffness.getMinimalStiffness());
	}

	if (m_bAnalyzeWrenchSet)
	{
		// calculate the accessable wrench set (or more precisely some characteristic vectors along the unit forces and torques)
		Vector3d fmin,fmax,taumin,taumax;
		if (pRD->ForceDistribution.getMaximumWrenchForPose(pRD->r,pRD->R,fmin,taumin,fmax,taumax))
		{
			if (m_bVerboseMode == true) 
				cout << "Pose belongs to the workspace\nMaximum feasible wrench set (forces and torques) for pose is: " << endl
					<< "force:\n  " << fmin << " -- " << fmax << endl
					<< "torque:\n " << taumin<< " -- " << taumax << endl << endl;
			pMF->m_wndPosePane.SetWrenchSet(fmin,fmax,taumin,taumax);
			pMF->m_wndPosePane.SetControllableWorkspce(true);
			WrenchSet.clear();
			double s = pPlatformForces->scalingFactor;		// scale the forces
			CVector3 
				xp(-fmax.x()*s,0,0),
				xm(-fmin.x()*s,0,0),
				yp(0,-fmax.y()*s,0),
				ym(0,-fmin.y()*s,0),
				zp(0,0,-fmax.z()*s),
				zm(0,0,-fmin.z()*s);
			WrenchSet.addTriangle(xp,yp,zp);
			WrenchSet.addTriangle(yp,xm,zp);
			WrenchSet.addTriangle(xm,ym,zp);
			WrenchSet.addTriangle(ym,xp,zp);
			WrenchSet.addTriangle(xp,yp,zm);
			WrenchSet.addTriangle(yp,xm,zm);
			WrenchSet.addTriangle(xm,ym,zm);
			WrenchSet.addTriangle(ym,xp,zm);
		}
		else
		{
			if (m_bVerboseMode == true) 
				cout << "Pose does not belong to the workspace\n\n";
			fmin = fmax = taumin = taumax = Vector3d::Zero();
			pMF->m_wndPosePane.SetWrenchSet(fmin,fmax,taumin,taumax);
			pMF->m_wndPosePane.SetControllableWorkspce(false);
			WrenchSet.clear();
		}
	}
		
	if (m_bAnalyzeForceDistribution)
	{
		if (pRD->getMotionPattern() == PCRL::CRobotData::MP3R3T && pRD->getForceDistribution())
		{
			if (m_bVerboseMode == true) 
				cout << "force distribution:\n " << pRD->F << endl;
			pMF->m_wndPosePane.SetForceDistribution(pRD->getNow(),pRD->F.data());
		}
		else
			if (m_bVerboseMode == true) 
				cout << "Could not determine a force distribution.\n";
	}

	if (true)	// to be inserted m_bAnalyzeCableCollision
	{
		time_t start = clock();
		int cols = pRD->Interference.getObstacleCableCollisions(pRD->r,pRD->R);
		if (m_bVerboseMode == true)
		{
			cout << "Number of Collisions between cables and obstacles: " << cols << endl;
			cout << "Computation time: " << clock()-start << endl;
		}
	}

	/* if (true)	// add an bool option here
	{
		cout << "Compute some sagging\n";
		PCRL::CCableSagging sag(*pRD,*pRD->currentCable);
		sag.analyzePose(pRD->r,pRD->R);
	} */

	// compute the orientation workspace for the current position based from R=I
	if (m_bAnalyzeOrientationRange) // m_bComputeOrientationRange orientation workspace for current pose
	{
		// use the pose property in stand-alone mode to evaluate the desired
		// properties
		PCRL::COrientationWorkspaceEvaluator OWE(*pRD,pRD->WSHull);
		MatrixXd values(OWE.getPropertyCount(),1);
		OWE.computeProperty(pRD->r,pRD->R,values);
		Vector3d 
			anglemin = values.block(0,0,3,1),
			anglemax = values.block(3,0,3,1);

		pMF->m_wndPosePane.SetOrientationWorkspace(anglemin,anglemax);
	}

	// compute the free range of travel (translational workspace)
	if (m_bAnalyzeTranslationRange) // m_bAnalyzeTranslationRange
	{
		PCRL::CWorkspaceHull Hull(*pRD);
		Hull.attachEvaluator(pRD->ForceDistribution,pRD->Kinematics,pRD->WrenchSet,pRD->VelocitySet,pRD->Stiffness);

		// copy general settings
		Hull.getAlgorithmParameter(pRD->WSHull);
		// adjust to the specific things 
		Hull.setIterations(0);
		Hull.setProjectionCenter(pRD->r);
		Hull.calculateWorkspace();
		MatrixXd ranges(6,1);
		if (Hull.bCenterValid)
			for (int i=0; i<6; i++)
			{
				if (m_bVerboseMode)
					cout << i << " " << Hull.getVertexRay(i).transpose() << " | " << Hull.getVertexRayLength(i) << endl;
				ranges(i) = Hull.getVertexRayLength(i);
			}
		else
			if (m_bVerboseMode)
				cout << "The current pose itself is not part of the workspace\n";
		pMF->m_wndPosePane.SetTranslationalWorkspace(ranges);
	}

	// update the workspace calculation if the flag is set to interactive mode
	if (m_bAutoApplyOrientation)
	{
		pRD->Interference.setReferenceOrientation(pRD->R);
		OnArbeitsraumBerechnen();
	}

	// update the interference test
	//pRD->Interference.calculateInterference();

	// show the control PropertyGrid control angain 
	pMF->m_wndPosePane.SetPropertyGridVisible(true);

	pMF->m_wndPosePane.SetPoseComputationTime((int)(clock()-start));

	GetDocument()->UpdateAllViews(0);
	return 0L;
}

LRESULT CWireCenterView::OnUpdateWorkspace(WPARAM wParam, LPARAM lParam)
{
	OnBnClickedCalculateworkspaceButton();
	return 0L;
}

/* ! Update the pose (r,R) information in the Pose Pane
 */
void CWireCenterView::UpdatePosePane()
{
	CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
	// write current location and rotation into the pane
	pMF->m_wndPosePane.SetPoseParameter(GetRobotDoc()->r,GetRobotDoc()->R);
	//and the other stuff
	//pMF->m_wndPosePane.SetStiffness(pMF->m_wndPosePane.SetStiffness(GetRobotDoc()->minStiffness,GetRobotDoc()->Stiffness.getMinimalStiffness());
	pMF->m_wndPosePane.SetEvaluator(GetDocument()->Evaluator, GetRobotDoc()->r,GetRobotDoc()->R);
}

/*! perform all update operations required to update the information displayed in the
 *  geometry pane
 */
void CWireCenterView::UpdateGeometryPane()
{
	// update the interactive geometry pane
	CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
	if (pMF) 
		pMF->m_wndGeomPane.updateAllData();
}

void CWireCenterView::setWorkspaceVisibility(const bool isVisible)
{
	// do nothing if no change is requested
	if (m_bWorkspaceVisible == isVisible)
		return;
	m_bWorkspaceVisible = isVisible;
	Invalidate();
}

void CWireCenterView::setWireSpanVisibility(const bool isVisible)
{
	// do nothing if no change is requested
	if (m_bWireSpanVisible == isVisible)
		return;
	m_bWireSpanVisible = isVisible;
	Invalidate();
}

void CWireCenterView::setWiresVisibility(const bool isVisible)
{
	// do nothing if no change is requested
	if (m_bWiresVisible == isVisible)
		return;
	m_bWiresVisible = isVisible;
	Invalidate();
}

void CWireCenterView::setWinchesVisibility(const bool isVisible)
{
	// do nothing if no change is requested
	if (m_bWinchesVisible == isVisible)
		return;
	m_bWinchesVisible = isVisible;
	Invalidate();
}

void CWireCenterView::setPlatformVisibility(const bool isVisible)
{
	// do nothing if no change is requested
	if (m_bPlatformVisible == isVisible)
		return;
	m_bPlatformVisible = isVisible;
	Invalidate();
}

void CWireCenterView::setFrameBoundingBoxVisibility(const bool isVisible)
{
	// do nothing if no change is requested
	if (m_bFrameBoundingBoxVisible == isVisible)
		return;
	m_bFrameBoundingBoxVisible = isVisible;
	Invalidate();
}

void CWireCenterView::setPlatformBoundingBoxVisibility(const bool isVisible)
{
	// do nothing if no change is requested
	if (m_bPlatformBoundBoxVisible == isVisible)
		return;
	m_bPlatformBoundBoxVisible = isVisible;
	Invalidate();
}

void CWireCenterView::setPlanesVisibility(const bool isVisible)
{
	// do nothing if no change is requested
	if (m_bPlanesVisible == isVisible)
		return;
	m_bPlanesVisible = isVisible;
	Invalidate();
}

void CWireCenterView::setInterferenceVisibility(const bool isVisible)
{
	// do nothing if no change is requested
	if (m_bInterferenceVisible == isVisible)
		return;
	m_bInterferenceVisible = isVisible;
	Invalidate();
}

void CWireCenterView::setCableForcesVisibility(const bool isVisible)
{
	// do nothing if no change is requested
	if (m_bCableForcesVisible == isVisible)
		return;
	m_bCableForcesVisible = isVisible;
	Invalidate();
}

void CWireCenterView::setWinchPulleyVisibility(const bool isVisible)
{
	// do nothing if no change is requested
	if (m_bWinchPulleysVisible == isVisible)
		return;
	m_bWinchPulleysVisible = isVisible;
	Invalidate();
}

void CWireCenterView::OnShowWindow(BOOL bShow, UINT nStatus)
{
	CView::OnShowWindow(bShow, nStatus);

	// TODO: F�gen Sie hier Ihren Meldungsbehandlungscode ein.
}

void CWireCenterView::setPoseEstimateBoxVisibility(const bool isVisible)
{
	// do nothing if no change is requested
	if (m_bPoseEstimateBoxVisible == isVisible)
		return;
	m_bPoseEstimateBoxVisible = isVisible;
	Invalidate();
}

void CWireCenterView::setPosEstimateBoxSpheresVisibility(const bool isVisible)
{
	// do nothing if no change is requested
	if (m_bPoseEstimateBoxSpheresVisible == isVisible)
		return;
	// make sure Pose Estimate Box is also Visible
	if (m_bPoseEstimateBoxVisible != isVisible)
		setPoseEstimateBoxVisibility(isVisible);

	m_bPoseEstimateBoxSpheresVisible = isVisible;
	Invalidate();
}

/*! This function is called by the MFC framework to update visual elements
 *  after chages in the document (here CWireCenterDocument including RobotDocument).
 */
void CWireCenterView::OnUpdate(CView* /*pSender*/, LPARAM /*lHint*/, CObject* /*pHint*/)
{
	setWorkspaceOptionsInSidebar();
	UpdateStatistics();
	UpdateWorkspaceHull();
	UpdatePosePane();
	UpdateGeometryPane();
	Invalidate(); 
}

void CWireCenterView::OnArbeitsraumBerechnen()
{ OnBnClickedCalculateworkspaceButton(); }

void CWireCenterView::OnArbeitsraumDifferentielberechnen()
{
	PCRL::CRobotData *pRobot = GetRobotDoc();
	PCRL::CWorkspaceDifferentialHull* pHull = &(GetRobotDoc()->WSHull);
	pHull->calculateWorkspaceDiff();
	pHull->printDifferential();
	OnUpdate();
}

void CWireCenterView::OnAnsichtAusrichtenxyebene()
{
	m_Scene.setXYPlane();
	Invalidate(FALSE);
}

void CWireCenterView::OnAnsichtAusrichtenxzebene()
{
	m_Scene.setXZPlane();
	Invalidate(FALSE);
}

void CWireCenterView::OnAnsichtAusrichtenyzebene()
{
	m_Scene.setYZPlane();
	Invalidate(FALSE);
}

void CWireCenterView::OnAnsichtZuruecksetzen()
{
	m_Scene.ResetView();
	Invalidate(FALSE);
}

#ifdef USE_SANDBOX_FOR_INTERVAL_METHOD
// #include "WireCenterViewSandBox.cpp"
#include "IntervalDocument.h"
#else

void CWireCenterView::OnArbeitsraumIntervallmethode() 
{
	MatrixXd R2 = MatrixXd::Random(10,10);
	
	MatrixXd R_ = GetRobotDoc()->R ;
	CMatrixDialog MD("Orientation Matrix View", R2);
	MD.DoModal();
	return;

	time_t start = clock();
	for (int i=0; i<10000; i++)
		GetRobotDoc()->ForceDistribution.calculateWorkspaceWrenchClosure();
	cout << "Computation time for 1000 calls: " << clock() - start << " ms\n";
	return;

	const TMotorParameter *p = MotorParameterKB;
	while (p->name != 0)
	{
		printf("%s | Mmax = %f | Tau = %f | alpha_max = %f | a_max = %f\n",p->name,p->M0,p->J,p->M0/p->J, p->M0/p->J*p->D/2.0);
		p++;
	}
}

#endif

//! load the nc-program named filename, generate the trajectory in poselist, and start the playback
void CWireCenterView::NC_Driver(CString& filename)
{
	if (LoadNcProgram(filename))
		OnNcStarten();
	else
		cout << "Error while parsing or interpolating NC program: " << filename << endl;
}

bool CWireCenterView::LoadPoseEvaluatorSettings(const CString& filename)
{
	GetDocument()->Evaluator.removeAllProperties();
	return GetDocument()->Evaluator.loadSettings(string(filename));
}

bool CWireCenterView::SavePoseEvaluatorSettings(const CString& filename)
{
	return GetDocument()->Evaluator.saveSettings(string(filename));
}

//! load the NC-program and generate the pose list
bool CWireCenterView::LoadNcProgram(const CString& filename)
{
	// delete all poses from the last program (if any)
	PoseList.deleteAllPoses();
	
	if (filename.GetLength() == 0)
		return false;

	PCRL::CNcParser Parser = PCRL::CNcParser(); // initialize class with the nc-Parser
	PCRL::CNcProgram NcProgram = PCRL::CNcProgram(); // initialize a object, where the interpreted commands are saved
	cout << "Calling parser for " << filename << endl;

	// parse the program
	if (!Parser.ParseNcfile(filename.GetString(),NcProgram))
		return false; 
	PCRL::CRobotDocument *pRD = &GetDocument()->robotDoc;

	if (NcProgram.empty())
	{	
		cout << "No Pose found, check file / filename" << endl;
		return false;
	}
	// interpolate
	if (!pRD->Interpolator.interpolateProgram(NcProgram,PoseList))
		return false; 

	//copy the poselist
	pNcLineStrip->m_poseList = PoseList;

	// configuration of the visualization of the linestrip.
	pNcLineStrip->show_velocity = false;
	pNcLineStrip->show_acceleration = false;
	pNcLineStrip->show_angularVelocity = false;
	pNcLineStrip->show_angularAcceleration = false;
	pNcLineStrip->show_force = false;
	pNcLineStrip->show_torque = false;
	pNcLineStrip->show_rotationVectors = false;
	pNcLineStrip->show_pointCloud = false;
	pNcLineStrip->show_lineStrip = true; // LineStrip
	
	pNcLineStrip->linePattern = 0xF00F;
	pNcLineStrip->color_lineStrip = CGLRGBAColor(1.0,0.0,0.0);

	return true;
}

void CWireCenterView::createPoseListRandom(int numberOfPoses, const Vector3d& r_min, const Vector3d& r_max, const Vector3d& R_limit)
{
	PoseList.deleteAllPoses();
	PoseList.generate_randomPoses(numberOfPoses, r_min, r_max, R_limit);
}

void CWireCenterView::createPoseListGrid(const Vector3d& r_min, const Vector3d& r_max, const Vector3d& r_stepSize)
{
	PoseList.deleteAllPoses();
	PoseList.generate_uniformGrid(r_min, r_max, r_stepSize);
}

void CWireCenterView::createPoseListGridRandomRot(const Vector3d& r_min, const Vector3d& r_max, const Vector3d& r_stepSize, const Vector3d& R_limit)
{
	PoseList.deleteAllPoses();
	PoseList.generate_uniformGridRandomRotation(r_min, r_max, r_stepSize, R_limit);
}

//! process the menu item: show the open file dialog and then load the program
void CWireCenterView::OnNcLaden()
{
	CFileDialog FileDlg(TRUE, ".nc","IPAnema_Demo_asp.nc",OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"G-Code (*.nc)|*.nc||");

	std::string WirecenterPath;
	WirecenterPath = GetDocument()->getWireCenterPath();
	WirecenterPath.append("\\nc");

	FileDlg.m_ofn.lpstrInitialDir = WirecenterPath.c_str(); 

	if (FileDlg.DoModal() == IDOK)
	{
		LoadNcProgram(FileDlg.GetPathName());
		m_NC_filename = FileDlg.GetPathName(); // save the path and filename into the member variable of WireCenterView for automatic reloading the NC-program after changing options of NC-Interpolator
	}
	else
		m_NC_filename = "";
}

void CWireCenterView::OnNcOptionen()
{
	// create dialog instance
	CNCOptionsDlg NCOptionsDlg;

	// find the active robotdoc
	PCRL::CRobotDocument *pRD = &GetDocument()->robotDoc;

	// read all values individually because copying the whole object could overwrite some parameters
	NCOptionsDlg.amax = pRD->Interpolator.amax;
	NCOptionsDlg.dOverride = pRD->Interpolator.dOverride;
	NCOptionsDlg.jmax = pRD->Interpolator.jmax;
	NCOptionsDlg.cycleTime = pRD->Interpolator.cycleTime;
	NCOptionsDlg.iBezier = pRD->Interpolator.iBezier;
	NCOptionsDlg.iInterpolationMethod = pRD->Interpolator.iInterpolationMethod;

	// show modal dialog and if OK...
	if (NCOptionsDlg.DoModal() == IDOK) 
	{
		// ...write back all values
		pRD->Interpolator.amax = NCOptionsDlg.amax;
		pRD->Interpolator.dOverride = NCOptionsDlg.dOverride;
		pRD->Interpolator.jmax = NCOptionsDlg.jmax;
		pRD->Interpolator.cycleTime = NCOptionsDlg.cycleTime;
		pRD->Interpolator.iBezier = NCOptionsDlg.iBezier;
		pRD->Interpolator.iInterpolationMethod = NCOptionsDlg.iInterpolationMethod;
		// reload and interpolate the NC program with the new parameters
		if (strlen(m_NC_filename) != 0)
			LoadNcProgram(m_NC_filename);
		return;
	}
}

// start the timer for playback
void CWireCenterView::OnNcStarten()
{
	m_bRunningProgram = true;
	PoseList.resetCursor();
	SetTimer(2,1,NULL);
}

// stop the program by killing the timer
void CWireCenterView::OnNcStoppen()
{
	m_bRunningProgram = false;
	KillTimer(2);
	cout << "Program ended" << endl;
}

void CWireCenterView::OnNcFortsetzen()
{
	if (m_bRunningProgram)
		return;
	
	m_bRunningProgram=true;
	SetTimer(2,1,NULL);
}

// set the activity of the menu item NC-Programm->Laden
void CWireCenterView::OnUpdateNcLaden(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(!m_bRunningProgram);
}

// set the activity of the menu item NC-Programm->Fortsetzen
void CWireCenterView::OnUpdateNcFortsetzen(CCmdUI *pCmdUI)
{
	if (!m_bRunningProgram && PoseList.size()>0)
		pCmdUI->Enable(TRUE);
	else
		pCmdUI->Enable(FALSE);
}

// set the activity of the menu item NC-Programm->Starten
void CWireCenterView::OnUpdateNcStarten(CCmdUI *pCmdUI)
{
	if (!m_bRunningProgram && PoseList.size()>0)
		pCmdUI->Enable(TRUE);
	else
		pCmdUI->Enable(FALSE);
}

// set the activity of the menu item NC-Programm->Stoppen
void CWireCenterView::OnUpdateNcStoppen(CCmdUI *pCmdUI)
{
	if (m_bRunningProgram && PoseList.size()>0)
		pCmdUI->Enable(TRUE);
	else
		pCmdUI->Enable(FALSE);
}

// set the activity of the menu item NC-Programm->AVI exportieren
void CWireCenterView::OnUpdateNcAviexportieren(CCmdUI *pCmdUI)
{
	if (!m_bRunningProgram && PoseList.size()>0)
		pCmdUI->Enable(TRUE);
	else
		pCmdUI->Enable(FALSE);
}

/*! create a video from the trajectory stored in PoseList. The framerate
 *  of the video file is 25 fps. If the video should be running in realtime
 *  the IPO must have been set to 1/25 = 40ms !
 */
void CWireCenterView::OnNcAviexportieren()
{
	if (PoseList.size() == 0)
		return;
	PCRL::CRobotDocument *pRD = &GetDocument()->robotDoc;

	// declare some variables and get the current dimensions
	CRect rect; GetWindowRect(rect);
	int x_res = rect.Width();
	int y_res = rect.Height();
	double scale_factor = 1.0;

	// create an enhanced dialog box
	CEnhancedDialog DLG("Create Video");
	DLG.StartGroupBox("Settings");
		DLG.addItem("x-Resolution:",x_res);
		DLG.addItem("y-Resolution:",y_res);	
		DLG.addItem("Info: The default resolution is derived from the current size of the 3D view window.");
		DLG.addItem("Scaling Factor:",scale_factor);
		DLG.addItem("Info: The scaling factor is multiplied with the resolution and can be later applied to the view");
	DLG.EndGroupBox();
	
	// show the dialogbox
	if (DLG.DoModal() != IDOK) return;

	CFileDialog FileDlg(FALSE, ".avi","output.avi",OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"Audio Video Interleaved (*.avi)|*.avi||");
	
	if (FileDlg.DoModal() != IDOK)
		return;

	// set the new window size and repaint it
	SetWindowPos(NULL,0,0,x_res,y_res,SWP_NOMOVE);
	RedrawWindow();

	if (!m_Scene.initAvi(FileDlg.GetPathName().GetString(),(int)(scale_factor*x_res),(int)(scale_factor*y_res)))
		return;

	// Create progressbar (which automatically destroys itself if function gets out of scope
	CProgressBar ProgressBar(ID_STATUSBAR_PROGRESSBAR, 300, PoseList.size(), ID_STATUSBAR_PANE2, "Computing Video-Output...");

	int i = 0;
	PCRL::CPoseStatic* frame;
	PoseList.resetCursor();
	
	while (frame = PoseList.nextPose())
	{
		//increase frame index
		i++;
		
		//update the ProgressBar
		ProgressBar.StepIt(1);

		//analyze the pose given in the list; copy the current values to the robot document
		pRD->r = frame->r;
		pRD->R = frame->R;

		//save the current frame in the avi stream
		m_Scene.saveAviFrame();
	}
	// stop the capture engine
	m_Scene.finalizeAvi();

	//rescale and repaint the scene again
	SetWindowPos(NULL,0,0,rect.Width(),rect.Height(),SWP_NOMOVE);
	RedrawWindow();
}

/*! create a video from the current scene where a full rotation around the z-axis is made
 *  e.g. useful to illustrate a rotation of the robot, geometry, or workspace
 */
void CWireCenterView::OnAviExportRotation()
{
//	PCRL::CRobotDocument *pRD = &GetDocument()->robotDoc;

	//declare some variables and get the current dimensions
	CRect rect; GetWindowRect(rect);
	int x_res = 640;
	int y_res = 480;
	double scale_factor = 1.0;
	double length = 5.0;

	// create an enhanced dialog box
	CEnhancedDialog DLG("Create 360� Video");
	DLG.StartGroupBox("Settings");
		DLG.addItem("x-Resolution:",x_res);
		DLG.addItem("y-Resolution:",y_res);	
		DLG.addItem("animation length [s]:",length);
	DLG.EndGroupBox();

	//show the dialogbox
	if (DLG.DoModal() != IDOK) 
		return;

	CFileDialog FileDlg(FALSE, ".avi","output.avi",OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"Audio Video Interleaved (*.avi)|*.avi||");
	
	if (FileDlg.DoModal() != IDOK)
		return;

	//set the new window size and repaint it
	SetWindowPos(NULL,0,0,x_res,y_res,SWP_NOMOVE);
	RedrawWindow();

	if (!m_Scene.initAvi(FileDlg.GetPathName().GetString(),(int)(scale_factor*x_res),(int)(scale_factor*y_res)))
		return;

	// create progressbar (which automatically destroys itself if function gets out of scope
	CProgressBar ProgressBar(ID_STATUSBAR_PROGRESSBAR, 300, (int)(length*25), ID_STATUSBAR_PANE2, "Computing Video-Output...");

	// store the original orienation of the base frame
	auto oldR = m_Scene.getFrame(2)->R;

	for (double t=0, phi=0; t<length; phi+=2*MO_PI/length/25.0, t+=0.04)
	{		
		//update the ProgressBar
		ProgressBar.StepIt(1);

		// set the rotation to the base frame of the scene
		Matrix3d R = Matrix3d::ZRotationMatrix3d(phi);
		// data types are incompatible, so we have to copy one-by-one
		for (int i=0; i<3; i++)
			for (int j=0; j<3; j++)
				m_Scene.getFrame(2)->R(i+1,j+1) = R(i,j);

		//save the current frame in the avi stream
		m_Scene.saveAviFrame();
	}
	// stop the capture engine
	m_Scene.finalizeAvi();

	// restore the frame orienation
	m_Scene.getFrame(2)->R = oldR ;
	//rescale and repaint the scene again
	SetWindowPos(NULL,0,0,rect.Width(),rect.Height(),SWP_NOMOVE);
	RedrawWindow();
}

void CWireCenterView::OnTimer(UINT_PTR nIDEvent)
{
	// nIDEvent == 2: timer event for playback of trajectories
	if (nIDEvent == 2 )
		playPoseList();

	CView::OnTimer(nIDEvent);
}

void CWireCenterView::playPoseList()
{
	if (PoseList.getCurrent())
	{
		PCRL::CPoseStatic* pose = PoseList.getCurrent();
		
		if (m_bPanCamera)
			m_Scene.addTrans(-CVector3(pose->r.x(),pose->r.y(),pose->r.z())+CVector3(GetDocument()->robotDoc.r.x(),GetDocument()->robotDoc.r.y(),GetDocument()->robotDoc.r.z()));
	
		GetDocument()->robotDoc.r = Vector3d(pose->r.x(),pose->r.y(),pose->r.z());
		Matrix3d R_tmp;

		R_tmp = pose->R;

		GetDocument()->robotDoc.R = R_tmp;

		Invalidate(); 
		PoseList.nextPose();
	}
	else // End
	{
		cout << "Program ended" << endl;
		OnNcStoppen();
	}
}

//! start the pose evaluator for the current list
void CWireCenterView::OnBerechnenTrajektorieanalysieren()
{
	if (PoseList.size() == 0)
		return;
	// do all calculations
	GetDocument()->Evaluator.calculate(PoseList);

}

// the following code executes the trajectory evaluation configured through
// the Evaluator-object
void CWireCenterView::BerechnenTrajektorieanalysieren(CString filename)
{
	// This function probably needs a meaningful behavior as it is also called from external
	OnBerechnenTrajektorieanalysieren();
	// the following four lines are copied from OnPoselistSaveButton() but it seems that doubling the code is acceptable
	GetDocument()->Evaluator.saveCsv(filename.GetBuffer());
	GetDocument()->Evaluator.saveHtml((filename+CString(".html")).GetBuffer());
	GetDocument()->Evaluator.saveStatsHtml((filename+CString(".stats.html")).GetBuffer());
	GetDocument()->Evaluator.saveSettings((filename+CString(".txt")).GetBuffer());
}

//! ask for a filename and save the result matrix into respective files
void CWireCenterView::OnPoselistSaveButton()
{
	CFileDialog FileDlg(FALSE, ".csv","output.csv",OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"CSV Data File (*.csv)|*.csv||");

	std::string WirecenterPath;
	WirecenterPath = GetDocument()->getWireCenterPath();
	WirecenterPath.append("\\nc");

	FileDlg.m_ofn.lpstrInitialDir = WirecenterPath.c_str(); 
	
	// cancel if user did not click ok
	if (FileDlg.DoModal() != IDOK)
		return;

	CString filename = FileDlg.GetPathName();
	GetDocument()->Evaluator.saveCsv(filename.GetBuffer());
	GetDocument()->Evaluator.saveHtml((filename+CString(".html")).GetBuffer());
	GetDocument()->Evaluator.saveStatsHtml((filename+CString(".stats.html")).GetBuffer());
	GetDocument()->Evaluator.saveSettings((filename+CString(".txt")).GetBuffer());
}

//! just open the matrix view for the result matrix of the evaluator
void CWireCenterView::OnPoselistViewButton()
{
	// we show the general matrix view dialog for the result matrix
	CMatrixDialog MD("Results",	GetDocument()->Evaluator.getResult());
	MD.DoModal();
}

void CWireCenterView::OnUpdatePoselistSaveButton(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(GetDocument()->Evaluator.getResultCount()>1);
}

void CWireCenterView::OnUpdatePoselistViewButton(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(GetDocument()->Evaluator.getResultCount()>1);
}

void CWireCenterView::OnUpdateBerechnenTrajektorieanalysieren(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(PoseList.size()>0);
}

void CWireCenterView::OnUpdateArbeitsraumKontur(CCmdUI *pCmdUI)
{
	pCmdUI->Enable();
}

void CWireCenterView::OnArbeitsraumKonturCustomX()
{
	// call the library function with the parameter and invalidate the window
	GetRobotDoc()->synchronizeAlgorithmSettings();
	GetRobotDoc()->Crosssection.calculateWorkspaceCrosssection('x');
	OnUpdate(0,0,0);
}

void CWireCenterView::OnArbeitsraumKonturCustomY()
{
	// call the library function with the parameter and invalidate the window
	GetRobotDoc()->synchronizeAlgorithmSettings();
	GetRobotDoc()->Crosssection.calculateWorkspaceCrosssection('y');
	OnUpdate(0,0,0);
}

void CWireCenterView::OnArbeitsraumKonturCustomZ()
{
	// call the library function with the parameter and invalidate the window
	GetRobotDoc()->synchronizeAlgorithmSettings();
	GetRobotDoc()->Crosssection.calculateWorkspaceCrosssection('z');
	OnUpdate(0,0,0);
}

void CWireCenterView::OnArbeitsraumVereinigen()
{
	GetRobotDoc()->WSHull.uniteWorkspace();
	OnUpdate(0,0,0);
}

void CWireCenterView::OnArbeitsraumSchneiden()
{
	GetRobotDoc()->WSHull.intersectWorkspace();
	OnUpdate(0,0,0);
}

void CWireCenterView::EvaluateVisibilityButtons(UINT nID, BOOL b_IsUpdateCall, CCmdUI *pCmdUI)
{
	//find the "Visibility Button" in the Ribbon and cast down to the selected button
	CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
	CMFCRibbonBaseElement* pViewButton = DYNAMIC_DOWNCAST(CMFCRibbonButton, pMF->m_wndRibbonBar.FindByID(ID_ANSICHT_SICHTBARKEIT));
	if (!pViewButton) return;
	CMFCRibbonBaseElement* pSelectedButton = pViewButton->FindByID(nID);
	if (!pSelectedButton) return;

	// decide, if this is an update to check the buttons or a click event
	bool m_bIsVisible;
	if (!b_IsUpdateCall)
	{
		//if is a click event, get the actual boolean value and invert it plus save it back
		get(pSelectedButton->GetDescription().GetString(),m_bIsVisible);
		set(pSelectedButton->GetDescription().GetString(),!m_bIsVisible);
	}
	else
	{
		//if is a update event, get the actual boolean value set the check symbol ... or not
		get(pSelectedButton->GetDescription().GetString(),m_bIsVisible);
		pCmdUI->SetCheck(m_bIsVisible);
	}

	//repaint the openGL window to see the changes immediately
	Invalidate();
}

void CWireCenterView::OnUpdateAnsichtSichtbarkeit(CCmdUI *pCmdUI)
{ pCmdUI->Enable(TRUE); }

void CWireCenterView::OnUpdateAnsichtSichtbarkeitRange(CCmdUI *pCmdUI)
{
	//see EvaluateVisibilityButtons for further information
	EvaluateVisibilityButtons(pCmdUI->m_nID,TRUE,pCmdUI);
}

void CWireCenterView::OnAnsichtSichtbarkeitRange(UINT nID)
{
	// see EvaluateVisibilityButtons for further information
	EvaluateVisibilityButtons(nID,FALSE,NULL);
}

void CWireCenterView::OnBerechnenAutomatischeposenanalyseoptionen()
{
	// create the dialog for configuration and feed the data in
	CPoseEvaluatorDlg PED;
	PED.pEvaluator = &GetDocument()->Evaluator;
	if (PED.DoModal() == IDOK)
		Invalidate();
}


void CWireCenterView::OnArbeitsraumOrientierungautomatischuebernehmen()
{ m_bAutoApplyOrientation =! m_bAutoApplyOrientation; }

void CWireCenterView::OnUpdateArbeitsraumOrientierungautomatischuebernehmen(CCmdUI *pCmdUI)
{ pCmdUI->SetCheck(m_bAutoApplyOrientation); }

void CWireCenterView::OnUpdateArbeitsraumAlleorientierungenpr32824(CCmdUI *pCmdUI)
{
	pCmdUI->SetCheck(GetRobotDoc()->WSHull.getOrientationRequirement());
	pCmdUI->Enable(!m_bAutoApplyOrientation);
}

void CWireCenterView::OnEntwurfAnwendungsdatenanzeigen()
{
	m_bRequirementBoxVisible =! m_bRequirementBoxVisible;
	Invalidate();
}

void CWireCenterView::OnUpdateEntwurfAnwendungsdatenanzeigen(CCmdUI *pCmdUI)
{ pCmdUI->SetCheck(m_bRequirementBoxVisible); }

void CWireCenterView::OnUpdateArbeitsraumKollisionszonensichtbar(CCmdUI *pCmdUI)
{ pCmdUI->SetCheck(m_bInterferenceVisible); }

void CWireCenterView::OnArbeitsraumKollisionszonensichtbar()
{
	m_bInterferenceVisible =! m_bInterferenceVisible;
	Invalidate();
}

void CWireCenterView::OnArbeitsraumMultithreading()
{
	m_bUseThreading =! m_bUseThreading;
}

void CWireCenterView::OnUpdateArbeitsraumMultithreading(CCmdUI *pCmdUI)
{ pCmdUI->SetCheck(m_bUseThreading); }


void CWireCenterView::OnGitterSichtbarButton()
{
	m_bWorkspaceGridVisible =! m_bWorkspaceGridVisible;
	Invalidate();
}

void CWireCenterView::OnUpdateGitterSichtbarButton(CCmdUI *pCmdUI)
{ pCmdUI->SetCheck(m_bWorkspaceGridVisible); }


void CWireCenterView::OnKonturSichtbarButton()
{
	m_bCrossSectionVisible =! m_bCrossSectionVisible;
	Invalidate();
}

void CWireCenterView::OnUpdateKonturSichtbarButton(CCmdUI *pCmdUI)
{ pCmdUI->SetCheck(m_bCrossSectionVisible); }

void CWireCenterView::OnArbeitsraumGridShowInvalid()
{
	m_bWorkspaceGridOutVisible =! m_bWorkspaceGridOutVisible;
	Invalidate();
}

void CWireCenterView::OnUpdateArbeitsraumGridShowInvalid(CCmdUI *pCmdUI)
{ pCmdUI->SetCheck(m_bWorkspaceGridOutVisible); }


void CWireCenterView::OnArbeitsraumGitterButton()
{
	Vector3d bbmin = GetDocument()->Roi.lower();
	Vector3d bbmax = GetDocument()->Roi.upper();
	getWorkspaceOptionsFromSidebar();
	GetRobotDoc()->synchronizeAlgorithmSettings();
	// override the above default
	if (!m_bAutoRoiUse)
		GetRobotDoc()->getBoundingBoxBase(bbmin,bbmax);
	GetRobotDoc()->WSGrid.makeGrid(bbmin,bbmax);
	GetRobotDoc()->WSGrid.calculateWorkspace();
	OnUpdate(0,0,0);
}

void CWireCenterView::OnArbeitsraumGitterCustomButton()
{
	//declare some variables	
	Vector3d bbmin = GetDocument()->Roi.lower();
	Vector3d bbmax = GetDocument()->Roi.upper();
	Vector3d eps = Vector3d(0.2,0.2,0.2);
	bool bCylinder = false; // use cyclinic grid rather than rectangular; 
	//built generic param dialog and show it
	CEnhancedDialog GPD("Compute fixed grid");
	GPD.StartGroupBox("Grid Settings");
		GPD.addItem("Step Size in x-direction [m]:",eps.x());
		GPD.addItem("Step Size in y-direction [m]:",eps.y());
		GPD.addItem("Step Size in z-direction [m]:",eps.z());
		GPD.addItem("Cylinderical Grid:",bCylinder);
	GPD.EndGroupBox();
	
	//if dialog returns an "OK", calculate the grid
	if (GPD.DoModal() == IDOK)
	{
		// use the bounding box of the robot if roi is disabled
		if (!m_bAutoRoiUse)
			GetDocument()->robotDoc.getBoundingBoxBase(bbmin,bbmax);
		if (bCylinder)
			GetDocument()->robotDoc.WSGrid.makeCylinderGrid(bbmin,bbmax);
		else
			GetDocument()->robotDoc.WSGrid.makeGrid(bbmin,bbmax,eps);
		GetDocument()->robotDoc.WSGrid.calculateWorkspace();
		OnUpdate(0,0,0);
	}
}


void CWireCenterView::OnDifferenzielSichtbarButton()
{
	// TODO: Add your command handler code here
}


void CWireCenterView::OnUpdateDifferenzielSichtbarButton(CCmdUI *pCmdUI)
{  }


void CWireCenterView::OnWinchSelectorCombo()
{
	PCRL::CRobotDocument *pRD = &GetDocument()->robotDoc;
	CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
	CMFCRibbonComboBox* pEdit = DYNAMIC_DOWNCAST(CMFCRibbonComboBox, pMF->m_wndRibbonBar.FindByID(IDC_WINCH_SELECTOR_COMBO));
	pEdit->GetCurSel();
	pRD->currentWinch = pRD->WinchDB.at(pEdit->GetCurSel()); //VLS: presents risky code if list in ComboBox does not match Vector length! Better Implementation?
}

void CWireCenterView::OnCliptoboxButton()
{
	Vector3d bbmin(-1,-1,0), bbmax(1,1,1);
	// create a generic parameter dialog object and add all parameters to the mapping
	CEnhancedDialog GPD("Clip Workspace to Box");
	GPD.StartGroupBox("Required Workspace");
		GPD.addItem("Bounding Box x_min [m]:",bbmin.x());
		GPD.addItem("Bounding Box y_min [m]:",bbmin.y());
		GPD.addItem("Bounding Box z_min [m]:",bbmin.z());
		GPD.addItem("Bounding Box x_max [m]:",bbmax.x());
		GPD.addItem("Bounding Box y_max [m]:",bbmax.y());
		GPD.addItem("Bounding Box z_max [m]:",bbmax.z());
	GPD.EndGroupBox();
	if (GPD.DoModal() != IDOK)
		return;

	// clip the workspace
	if (!GetRobotDoc()->WSHull.clipByBoundingBox(bbmin,bbmax))
		printf("Error while clipping the workspace, Maybe the projection center is outside of the given bounding box.\n");
	// update the properties and the GUI elements
	GetRobotDoc()->WSHull.calculateWorkspaceProperties();
	OnUpdate();
}


void CWireCenterView::OnCliptoframeButton()
{
	// get the bounding box from the robot model 
	Vector3d bbmin,bbmax;
	GetRobotDoc()->getBoundingBoxBase(bbmin,bbmax);
	// clip the workspace
	GetRobotDoc()->WSHull.clipByBoundingBox(bbmin,bbmax);
	// update the properties and the GUI elements
	GetRobotDoc()->WSHull.calculateWorkspaceProperties();
	OnUpdate();
}

//! the following function handle the GUI interface to select, edit, and applied the parametric model
//! call this function when the parametric model selection combo is changed
void CWireCenterView::OnParameterModelCombo() 
{
	PCRL::CRobotDocument *pRD = &GetDocument()->robotDoc; 
	CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
	CMFCRibbonComboBox* pCEdit = DYNAMIC_DOWNCAST(CMFCRibbonComboBox, pMF->m_wndRibbonBar.FindByID(ID_PARAMETERMODEL_COMBO));
	int sel = pCEdit->GetCurSel();
	
	// loop through all parameters of the selected model and put their names to the combo list
	pCEdit = DYNAMIC_DOWNCAST(CMFCRibbonComboBox, pMF->m_wndRibbonBar.FindByID(ID_PARAMETERTYPE_COMBO));
	pCEdit->RemoveAllItems();
	list<string> names;

	pRD->ParametricModels[sel]->getParamNames(names);
	
	for (list<string>::iterator itor = names.begin(); itor!=names.end(); itor++)
	{
		CString str;
		str.Format("%s",itor->c_str());
		pCEdit->AddItem(str);
	}
	pCEdit->SelectItem(0);
}

//! call this function when the parameter selector is changed
void CWireCenterView::OnParameterTypeCombo() 
{
	PCRL::CRobotDocument *pRD = &GetDocument()->robotDoc; 
	CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
	
	CMFCRibbonComboBox* pCEdit = DYNAMIC_DOWNCAST(CMFCRibbonComboBox, pMF->m_wndRibbonBar.FindByID(ID_PARAMETERMODEL_COMBO));
	int selModel = pCEdit->GetCurSel();

	pCEdit = DYNAMIC_DOWNCAST(CMFCRibbonComboBox, pMF->m_wndRibbonBar.FindByID(ID_PARAMETERTYPE_COMBO));
	CString selParam = pCEdit->GetEditText();

	CMFCRibbonEdit* pEdit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMF->m_wndRibbonBar.FindByID(ID_PARAMMODELVALUE_EDIT));
	CString str;
	double value;
	pRD->ParametricModels[selModel]->getParameter(selParam.GetBuffer(),value);
	str.Format("%f",value);
	pEdit->SetEditText(str);
}

//! call this function after the value in the edit was changed
void CWireCenterView::OnParameterValueEdit()
{
	PCRL::CRobotDocument *pRD = &GetDocument()->robotDoc; 
	CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
	
	CMFCRibbonComboBox* pCEdit = DYNAMIC_DOWNCAST(CMFCRibbonComboBox, pMF->m_wndRibbonBar.FindByID(ID_PARAMETERMODEL_COMBO));
	int selModel = pCEdit->GetCurSel();

	pCEdit = DYNAMIC_DOWNCAST(CMFCRibbonComboBox, pMF->m_wndRibbonBar.FindByID(ID_PARAMETERTYPE_COMBO));
	CString selParam = pCEdit->GetEditText();

	CMFCRibbonEdit* pEdit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMF->m_wndRibbonBar.FindByID(ID_PARAMMODELVALUE_EDIT));
	CString str;
	str = pEdit->GetEditText();
	double value;
	if (1 == sscanf_s(str.GetBuffer(),"%lf",&value))
		pRD->ParametricModels[selModel]->setParameter(selParam.GetBuffer(),value);
	OnParameterTypeCombo();
}

//! call this function to apply the current parametric model to the robot data 
void CWireCenterView::OnParameterSetButton()
{
	PCRL::CRobotDocument *pRD = &GetDocument()->robotDoc; 
	CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
	
	CMFCRibbonComboBox* pCEdit = DYNAMIC_DOWNCAST(CMFCRibbonComboBox, pMF->m_wndRibbonBar.FindByID(ID_PARAMETERMODEL_COMBO));
	int selModel = pCEdit->GetCurSel();

	if (!pRD->ParametricModels[selModel]->setGeometry())
		MessageBox("The parameterization was not successfully applied");
	OnUpdate();
}

//! \todo make the constants small_increment and large_increment member variables of the view class
//! and add these variables to set of reflected properties
const double small_increment = 0.1;
const double large_increment = 1;

void CWireCenterView::modifyParameterValue(const double& change)
{
	PCRL::CRobotDocument *pRD = &GetDocument()->robotDoc; 
	CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
	
	CMFCRibbonComboBox* pCEdit = DYNAMIC_DOWNCAST(CMFCRibbonComboBox, pMF->m_wndRibbonBar.FindByID(ID_PARAMETERMODEL_COMBO));
	int selModel = pCEdit->GetCurSel();

	pCEdit = DYNAMIC_DOWNCAST(CMFCRibbonComboBox, pMF->m_wndRibbonBar.FindByID(ID_PARAMETERTYPE_COMBO));
	CString selParam = pCEdit->GetEditText();

	double value;

	pRD->ParametricModels[selModel]->getParameter(selParam.GetBuffer(),value);
	value += change;
	pRD->ParametricModels[selModel]->setParameter(selParam.GetBuffer(),value);

	// write the parameter into the edit
	CMFCRibbonEdit* pEdit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMF->m_wndRibbonBar.FindByID(ID_PARAMMODELVALUE_EDIT));
	CString str;
	str.Format("%f",value);
	pEdit->SetEditText(str);
}

void CWireCenterView::OnParamdecButton()
{
	modifyParameterValue(-small_increment);
}

void CWireCenterView::OnParamfastdecButton()
{
	modifyParameterValue(-large_increment);
}

void CWireCenterView::OnParamfastincButton()
{
	modifyParameterValue(large_increment);
}

void CWireCenterView::OnParamincButton()
{
	modifyParameterValue(small_increment);
}

void CWireCenterView::OnArbeitsraumHuelleSichtbarButton()
{
	m_bWorkspaceVisible = !m_bWorkspaceVisible;
	Invalidate();
}

void CWireCenterView::OnUpdateArbeitsraumHuelleSichtbarButton(CCmdUI *pCmdUI)
{ pCmdUI->SetCheck(m_bWorkspaceVisible); }

void CWireCenterView::OnSceneBackgroundButton()
{
	m_Scene.setRenderBackground(!m_Scene.getRenderBackground());
	Invalidate();
}

void CWireCenterView::OnUpdateSceneBackgroundButton(CCmdUI *pCmdUI)
{ pCmdUI->SetCheck(m_Scene.getRenderBackground()); }

void CWireCenterView::OnSceneUppercolorButton()
{
	//find the ColorButton in the RibbonMenu
	CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
	CMFCRibbonColorButton* pButton = DYNAMIC_DOWNCAST(CMFCRibbonColorButton,pMF->m_wndRibbonBar.FindByID(ID_SCENE_UPPERCOLOR_BUTTON));

	//get the selected color, assign it to the scene and repaint!
	COLORREF rgb = pButton->GetColor();
	m_Scene.setUpperBackgroundColor(GetRValue(rgb)/255.0,GetGValue(rgb)/255.0,GetBValue(rgb)/255.0);
	Invalidate();	
}

void CWireCenterView::OnSceneLowercolorButton()
{
	//find the ColorButton in the RibbonMenu
	CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
	CMFCRibbonColorButton* pButton = DYNAMIC_DOWNCAST(CMFCRibbonColorButton,pMF->m_wndRibbonBar.FindByID(ID_SCENE_LOWERCOLOR_BUTTON));

	//get the selected color, assign it to the scene and repaint!
	COLORREF rgb = pButton->GetColor();
	m_Scene.setLowerBackgroundColor(GetRValue(rgb)/255.0,GetGValue(rgb)/255.0,GetBValue(rgb)/255.0);
	Invalidate();	
}

void CWireCenterView::OnScenePerspectiveButton()
{
	m_Scene.setPerspectiveProjection(!m_Scene.getPerspectiveProjection());
	m_Scene.RenderScene();
	Invalidate();
}

void CWireCenterView::OnUpdateScenePerspectiveButton(CCmdUI *pCmdUI)
{ pCmdUI->SetCheck(m_Scene.getPerspectiveProjection()); }

void CWireCenterView::OnSceneDeleteallshapesButton()
{
	m_Scene.deleteLayer(0);
	Invalidate();
}

void CWireCenterView::OnScreenshotButton()
{
	//declare some variables
	CRect rect; GetWindowRect(rect);
	int x_res = rect.Width();
	int y_res = rect.Height();
	double scale_factor = 1.0;

	// create an enhanced dialog box
	CEnhancedDialog DLG("Create Screen Shot");
	DLG.StartGroupBox("Settings");
		// provide some default settings for resolution
		CStringArray Resolutions;
		Resolutions.Add("Default");				// option 0
		Resolutions.Add("640 x 480 (VGA)");		// option 1
		Resolutions.Add("800 x 600 (VGA2)");	// option 2
		Resolutions.Add("1024 x 768 (SVGA)");	// option 3
		Resolutions.Add("1920 x 1080 (HD)");	// option 4
		Resolutions.Add("3840 x 2160 (4K)");	// option 5

		int sel = 0;	// custom resolution is default
		DLG.addItem("Resolution",sel,Resolutions);

		DLG.addItem("x-Resolution:",x_res);
		DLG.addItem("y-Resolution:",y_res);	
		DLG.addItem("Info: The default resolution is generated from the current window size. Other sizes might clip the window.");
		DLG.addItem("Scaling Factor:",scale_factor);
		DLG.addItem("Info: Background is only save in Bitmap formats");		
	DLG.EndGroupBox();
	
	// show the dialogbox
	if (DLG.DoModal() != IDOK) return;

	// overwrite the resolution if a predefined resolution was selected with the combo box
	if (sel == 1) { x_res = 640; y_res = 480; }
	if (sel == 2) { x_res = 800; y_res = 600; }
	if (sel == 3) { x_res = 1024; y_res = 768; }
	if (sel == 4) { x_res = 1920; y_res = 1080; }
	if (sel == 5) { x_res = 3840; y_res = 2160; }

	// open the save as dialog
	CFileDialog FileDlg(FALSE,	// save File Dialog
				".bmp",
				"screenshot.bmp",
				OFN_HIDEREADONLY,
				"Bitmap (*.bmp)|*.bmp|"
				"Scalable Vector Graphics (*.svg)|*.svg|"
				"Portable Document Format (*.pdf)|*.pdf|"
				"Encapsulated PostScript (*.eps)|*.eps|"
				"PostScript (*.ps)|*.ps|"
				"Portable Graphics Format (*.pgf)|*.pgf|");
	
	// if dialog was successful, get the file name and its path
	if (FileDlg.DoModal() != IDOK) 
		return;
	std::string filename = FileDlg.GetPathName();

	// set the new window size and repaint it
	SetWindowPos(NULL,0,0,x_res,y_res,SWP_NOMOVE);
	RedrawWindow();

	// save the current scene depending on its file name
	if (FileDlg.GetFileExt().MakeLower() == "bmp") 
		m_Scene.SaveBmp(filename,(int)(x_res*scale_factor),(int)(y_res*scale_factor));
	else if (FileDlg.GetFileExt().MakeLower() == "ps") 
		m_Scene.SaveImage(filename,0); // outputformat : ps (see magic numbers in gl2ps.h)
	else if (FileDlg.GetFileExt().MakeLower() == "eps") 
		m_Scene.SaveImage(filename,1); // outputformat : eps (see magic numbers in gl2ps.h)
	else if (FileDlg.GetFileExt().MakeLower() == "pdf") 
		m_Scene.SaveImage(filename,3); // outputformat : pdf (see magic numbers in gl2ps.h)	 
	else if (FileDlg.GetFileExt().MakeLower() == "svg") 
		m_Scene.SaveImage(filename,4); // outputformat : svg (see magic numbers in gl2ps.h)
	else if (FileDlg.GetFileExt().MakeLower() == "pgf") 
		m_Scene.SaveImage(filename,5); // outputformat : pgf (see magic numbers in gl2ps.h)
	else 
		AfxMessageBox("Did not recognize the file format for the screen shot!");

	//rescale and repaint the scene again
	SetWindowPos(NULL,0,0,rect.Width(),rect.Height(),SWP_NOMOVE);
	RedrawWindow();
}

void CWireCenterView::OnSceneSettingsButton()
{
	//declare some variables and fill the values
	double z_clip_min,z_clip_max,field_of_view;
	m_Scene.getPerspectiveSettings(z_clip_min,z_clip_max,field_of_view);

	// create an enhanced dialog box
	CEnhancedDialog DLG("3D Scene Object Settings");
	DLG.StartGroupBox("General Settings");
		DLG.addItem("Clip-Z (min):",z_clip_min);
		DLG.addItem("Clip-Z (max):",z_clip_max);
		DLG.addItem("Field of View:", field_of_view);
		DLG.addItem("Info: This settings only apply in perspective projection.");
	DLG.EndGroupBox();
	DLG.NextDialogColumn();
	DLG.StartGroupBox("Iteractive Settings");
		DLG.addItem("Unlock Rotation", m_Scene.bEnableRotation);
		DLG.addItem("Unlock Zoom", m_Scene.bEnableZoom);
		DLG.addItem("Unlock Panning", m_Scene.bEnablePan);
	DLG.EndGroupBox();

	//show the modal dialog box and return, if user abort
	if (DLG.DoModal() != IDOK) return;

	//set the new settings
	m_Scene.setPerspectiveSettings(z_clip_min,z_clip_max,field_of_view);
	
	//repaint the window content
	Invalidate();
}

void CWireCenterView::OnDragMode3DViewButton()
{
	//enable / disable 3D-drag mode
	m_Scene.bEnableDragging =! m_Scene.bEnableDragging;
}

void CWireCenterView::OnUpdateDragMode3DViewButton(CCmdUI *pCmdUI)
{
	//sets the button state
	pCmdUI->SetCheck(m_Scene.bEnableDragging);
}

void CWireCenterView::OnRoiVisibleButton()
{ m_bRoiVisible =! m_bRoiVisible; }

void CWireCenterView::OnUpdateRoiVisibleButton(CCmdUI *pCmdUI)
{ pCmdUI->SetCheck(m_bRoiVisible); }

void CWireCenterView::OnRoiAutoApplyButton()
{ m_bAutoRoiUse =! m_bAutoRoiUse; }

void CWireCenterView::OnUpdateRoiAutoApplyButton(CCmdUI *pCmdUI)
{ pCmdUI->SetCheck(m_bAutoRoiUse); }

void CWireCenterView::OnClipToRoiButton()
{
	// clip the workspace
	GetRobotDoc()->WSHull.clipByBoundingBox(GetDocument()->Roi.lower(), GetDocument()->Roi.upper());
	// update the properties and the GUI elements
	GetRobotDoc()->WSHull.calculateWorkspaceProperties();
	OnUpdate();
}

void CWireCenterView::OnGenerateGridButtonButton()
{
	//declare some variables	
	Vector3d bbmin = GetDocument()->Roi.lower();
	Vector3d bbmax = GetDocument()->Roi.upper();
	Vector3d eps = Vector3d(0.5,0.5,0.5);

	Vector3d rot_min = Vector3d(0.0,0.0,0.0);
	Vector3d rot_max = Vector3d(0.0,0.0,0.0);
	Vector3d rot_eps = Vector3d(5,5,5);

	// use the bounding box of the robot if roi is disabled
	if (!m_bAutoRoiUse)
		GetDocument()->robotDoc.getBoundingBoxBase(bbmin,bbmax);	//built generic param dialog and show it
	CEnhancedDialog GPD("Generate Rectengular regular grid");
	GPD.StartGroupBox("Grid setting Position");
		GPD.addItem("min x [m]",bbmin.x());
		GPD.addItem("min y [m]",bbmin.y());
		GPD.addItem("min z [m]",bbmin.z());
		GPD.addItem("max x [m]",bbmax.x());
		GPD.addItem("max y [m]",bbmax.y());
		GPD.addItem("max z [m]",bbmax.z());
		GPD.addItem("Step Size in x-direction [m]:",eps.x());
		GPD.addItem("Step Size in y-direction [m]:",eps.y());
		GPD.addItem("Step Size in z-direction [m]:",eps.z());
	GPD.EndGroupBox();
	GPD.NextDialogColumn();
	GPD.StartGroupBox("Grid setting orientation");
		GPD.addItem("min a [�]",rot_min.x());
		GPD.addItem("min b [�]",rot_min.y());
		GPD.addItem("min c [�]",rot_min.z());
		GPD.addItem("max a [�]",rot_max.x());
		GPD.addItem("max b [�]",rot_max.y());
		GPD.addItem("max c [�]",rot_max.z());
		GPD.addItem("Step Size in a-direction [degrees]:",rot_eps.x());
		GPD.addItem("Step Size in b-direction [degrees]:",rot_eps.y());
		GPD.addItem("Step Size in c-direction [degrees]:",rot_eps.z());
		GPD.addItem("Warning: Number of poses increases exponentionally with the chosen directions.");
	GPD.EndGroupBox();
	
	//if dialog returns an "OK", calculate the grid
	if (GPD.DoModal() == IDOK)
	{
		//convert degrees to radians:
		rot_min.x() = rot_min.x()*DEG_TO_RAD;
		rot_min.x() = rot_min.x()*DEG_TO_RAD;
		rot_min.y() = rot_min.y()*DEG_TO_RAD;
		rot_min.z() = rot_min.z()*DEG_TO_RAD;
		rot_max.x() = rot_max.x()*DEG_TO_RAD;
		rot_max.y() = rot_max.y()*DEG_TO_RAD;
		rot_max.z() = rot_max.z()*DEG_TO_RAD;
		rot_eps.x() = rot_eps.x()*DEG_TO_RAD;
		rot_eps.y() = rot_eps.y()*DEG_TO_RAD;
		rot_eps.z() = rot_eps.z()*DEG_TO_RAD;

		// we create a grid object. it has public access to its data model; therefore, we
		// can easily transfer to results from the grid generator and discard the workspace 
		// objects afterwards
		PCRL::CWorkspaceGrid grid(*GetRobotDoc());
		grid.makeGrid(bbmin,bbmax,eps);
		
		//generate another Grid for Rotation
		PCRL::CWorkspaceGrid rot_grid(*GetRobotDoc());
		if (rot_min.x() == rot_max.x() && rot_min.y() == rot_max.y() && rot_min.z() == rot_max.z())
		{
			rot_grid.vertices.push_back(Vector3d(0.0,0.0,0.0)); //base rotation if no grid is created
		}
		else
		{
			rot_grid.makeGrid(rot_min,rot_max,rot_eps);
		}
		
		// now we have to copy the generated grid from our temporary object to the
		// path object
		PoseList.clear();
		for (auto itor = grid.vertices.begin(); itor!=grid.vertices.end(); ++itor)
			for (auto rot_itor = rot_grid.vertices.begin(); rot_itor!=rot_grid.vertices.end(); ++rot_itor)
			{
				Matrix3d R;
				R =  Matrix3d::ZRotationMatrix3d(rot_itor->x())*Matrix3d::YRotationMatrix3d(rot_itor->y())*Matrix3d::XRotationMatrix3d(rot_itor->z());
				PoseList.push_back(new PCRL::CPoseKinetostatic(*itor,R));
			}
	
		OnUpdate();
	}
}


void CWireCenterView::OnCreateGridRandomButton()
{
	//declare some variables	
	Vector3d bbmin = GetDocument()->Roi.lower();
	Vector3d bbmax = GetDocument()->Roi.upper();
	int anz = 1000;

	// use the bounding box of the robot if roi is disabled
	if (!m_bAutoRoiUse)
		GetDocument()->robotDoc.getBoundingBoxBase(bbmin,bbmax);	//built generic param dialog and show it
	CEnhancedDialog GPD("Create Random Grid");
	GPD.StartGroupBox("Box for random Grid");
		GPD.addItem("min x [m]",bbmin.x());
		GPD.addItem("min y [m]",bbmin.y());
		GPD.addItem("min z [m]",bbmin.z());
		GPD.addItem("max x [m]",bbmax.x());
		GPD.addItem("max y [m]",bbmax.y());
		GPD.addItem("max z [m]",bbmax.z());
	GPD.EndGroupBox();
	GPD.addItem("Number of Samples:",anz);

	//if dialog returns an "OK", calculate the grid
	if (GPD.DoModal() == IDOK)
	{
		// we create a grid object. it has public access to its data model; therefore, we
		// can easily transfer to results from the grid generator and discard the workspace 
		// objects afterwards
		PCRL::CWorkspaceGrid grid(*GetRobotDoc());
		grid.makeRandomGrid(bbmin,bbmax,anz);
		
		// now we have to copy the generated grid from our temporary object to the
		// path object
		PoseList.clear();
		Matrix3d R = Matrix3d::Identity();
		for (auto itor = grid.vertices.begin(); itor!=grid.vertices.end(); ++itor)
			PoseList.push_back(new PCRL::CPoseKinetostatic(*itor,R));
	
		OnUpdate();
	}
}


void CWireCenterView::OnCreateGridZlayerButton()
{
	//declare some variables	
	Vector3d bbmin = GetDocument()->Roi.lower();
	Vector3d bbmax = GetDocument()->Roi.upper();
	Vector3d eps = Vector3d(0.5,0.5,0.5);

	// use the bounding box of the robot if roi is disabled
	if (!m_bAutoRoiUse)
		GetDocument()->robotDoc.getBoundingBoxBase(bbmin,bbmax);	//built generic param dialog and show it

	bbmin.z() = (bbmax.z()+bbmin.z())/2.0;

	CEnhancedDialog GPD("Define Plane");
	GPD.addItem("min x [m]",bbmin.x());
	GPD.addItem("min y [m]",bbmin.y());
	GPD.addItem("max x [m]",bbmax.x());
	GPD.addItem("max y [m]",bbmax.y());
	GPD.addItem("Step Size in x-Direction [m]:",eps.x());
	GPD.addItem("Step Size in y-Direction [m]:",eps.y());
	GPD.addItem("z-Layer [m]",bbmin.z());

	//if dialog returns an "OK", calculate the grid
	if (GPD.DoModal() == IDOK)
	{
		bbmax.z() = bbmin.z();
		// we create a grid object. it has public access to its data model; therefore, we
		// can easily transfer to results from the grid generator and discard the workspace 
		// objects afterwards
		PCRL::CWorkspaceGrid grid(*GetRobotDoc());
		grid.makeGrid(bbmin,bbmax,eps);
		
		// now we have to copy the generated grid from our temporary object to the
		// path object
		PoseList.clear();
		Matrix3d R = Matrix3d::Identity();
		for (auto itor = grid.vertices.begin(); itor!=grid.vertices.end(); ++itor)
			PoseList.push_back(new PCRL::CPoseKinetostatic(*itor,R));
	
		OnUpdate();
	}
}


void CWireCenterView::OnCablespanLinesVisibleButton()
{
	m_bWireSpanVisible =! m_bWireSpanVisible;
	Invalidate();
}


void CWireCenterView::OnCablespanConeVisibileButton()
{
	m_bWireSpanConeVisible =! m_bWireSpanConeVisible;
	Invalidate();
}


void CWireCenterView::OnConfigureShapeColorsButton()
{
	CEnhancedDialog GPD("Configure Internal Shape Colors");
	GPD.StartGroupBox("Control Point Color");
		GPD.addItem("red (0 - 1)", ColorControlPoints.R);
		GPD.addItem("green (0 - 1)",ColorControlPoints.G);
		GPD.addItem("blue (0 - 1)", ColorControlPoints.B);
	GPD.EndGroupBox();
	
	//if dialog returns an "OK", calculate the grid
	if (GPD.DoModal() == IDOK)
	{
		OnUpdate(0,0,0);
	}
}
