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

/*! \file WireCenterView.h
 *
 *	\author   Andreas Pott
 *
 *  \brief The main view with the openGL visualization. The class controls
 *  most of the functions of the application. Anyway, a couple of event handles
 *  should be move to CWireCenterDoc or CMainFrame because they modify the appication
 *  or document rather than the 3D-view.
 */

#pragma once

#include <IPAGL/GLScene.h>
#include <IPAGL/GLGeometry.h>
#include <WireLib/RobotData.h>
#include <WireLib/Shapes.h>
#include <motionPlanning/NcParser.h>
#include <motionPlanning/NcInterpolator.h>
//#include "Reflection.h"
#include "PathGenerator.h"
#include <WireLib/PoseList.h>
#include "WireCenterDoc.h"
#include "NCDlg.h"

// forward declarations
class CPoseDlg;
class CInteractiveGeometryDlg;
class CIPC;

/// <summary>Meine Zusammenfassung der Klasse</summary>
class CWireCenterView : public CView, public PCRL::CReflection, public CGLShape
{
public://protected: // Nur aus Serialisierung erstellen
	CWireCenterView();
	DECLARE_DYNCREATE(CWireCenterView)

// Attribute
public:
	CWireCenterDoc* GetDocument() const;
	PCRL::CRobotDocument* GetRobotDoc() const;

// Vorgänge
public:
	CGLSceneGraph m_Scene;				//!< the class encapsulating the openGL interface
	unsigned int hGlPlatform ;			//!< handle to the platforms mobile frame

	CIPC* m_pIPC;						//!< Class for interprocess communication

	// visibility settings
	bool m_bWorkspaceVisible;			//!< flag that indicates if the workspace should be drawn in openGL
	bool m_bWireSpanVisible;			//!< flag that indicates if the region covered by the wires is visible
	bool m_bWireSpanConeVisible;		//!< flag that indicates if the region covered by the wires span cone is visible
	bool m_bWiresVisible;				//!< flag that indicates if the wires are visible
	bool m_bWinchesVisible;				//!< flag that indicates if the winches are visible
	bool m_bPlatformVisible;			//!< flag that indicates if the platform is visible
	bool m_bPlanesVisible;				//!< flag that indicates if coordinates planes visible
	bool m_bInterferenceVisible;		//!< flag that indicates if regions of cable interference are calculated and visible
	bool m_bCrossSectionVisible;		//!< flag that indicates if the workspace hull is interpreted as a cross section
	bool m_bFrameBoundingBoxVisible;	//!< flag that indicates if the bounding box around the fixed frame is visible
	bool m_bPlatformBoundBoxVisible;	//!< flag that indicates if the bounding box around the mobile platform is visible
	bool m_bFrameFancyVisible;			//!< flag that indicates if the machine frame is drawn with more details (as aluminum bars)
	bool m_bRequirementBoxVisible;		//!< flag that indicates if the desired workspace and available installation space are visible
	bool m_bWinchesLabelVisible;		//!< flag that indicates if the winches labels should be drawn in openGL
	bool m_bGravityArrowVisible;		//!< flag that indicates if the gravity arrow should be drawn in openGL
	bool m_bGroundPlaneVisible;			//!< flag that indicates if the ground plane should be drawn in openGL
	bool m_bBaseFramesVisible;			//!< flag that indicates the visibility of the winch base frames
	bool m_bWorldFrameVisible;			//!< flag that indicates the visibility of the world frame (coordinate system)
	bool m_bWorkspaceGridVisible;		//!< flag that indicates if the workspace based on grid calculation is visible
	bool m_bWorkspaceGridOutVisible;	//!< flag that indicates if the points not in the workspace based on grid calculation are visible
	bool m_bWorkspaceInnerBoxVisible;	//!< flag that indicates if inner box of the workspace
	bool m_bWorkspaceHullGridVisible;	//!< flag that indicates if the workspace hull grid is visible
	bool m_bCableForcesVisible;			//!< flag that indicates if the cable forces are visible
	bool m_bWinchPulleysVisible;		//!< flag that indicates if the pulleys of the winches are visible
	bool m_bWrenchSetVisible;			//!< flag that indicates if the estimation of the available wrench set is visible
	bool m_bNcLineStripVisible;			//!< flag that indicates if the LineStrip generated by NC is visible
	bool m_bPosePropertyEvaluation;		//!< flag that indicates if the viusalization of a result of a pose property evaluation is visible
	bool m_bWireForceColoringVisible;	//!< flag that indicates if the cable force of the wires is mapped to a color range
	bool m_bCenterofInertiaVisible;		//!< flag that indicates if the center of inertia of the hull should be shown
	bool m_bWorkspaceParallelProjectionVisible; //!< flag that indicates if the parallel projections of the hull on the bounding box frames are shown
	bool m_bWorkspacePhongShadingVisible;		//!< flag that indicates if phong shading is enabled for the workspace hull
	bool m_bRoiVisible;					//!< flag that indicates if the region of interest (ROI) is visiable
	bool m_bPoseEstimateBoxVisible;		//!< flag to inidicate the Pose Estimation Box (whose center builds the starting point for forward kinematic optimizers)
	bool m_bPoseEstimateBoxSpheresVisible; //!< flag to indicate the visibility of spheres representing cable length around a_i within workspace estimationbox
	bool m_bPoseListControlPointsVisible;	//!< flag to indicate that the control points on the nc path are marked with a dot

	// application settings
	bool m_bUseThreading;						//!< activate the multi-thread system for workspace calculation
	bool m_bAutoApplyOrientation;				//!< use the orientation defined by the non-modal pose dialog for workspace calculation
	bool m_bAutoRoiUse;							//!< specifies that the ROI is used instead of default settings

	// settings for pose analysis
	bool m_bAnalyzeInverseKinematics;			//!< compute inverse kinematics using the standard model (point-point)
	bool m_bAnalyzeInverseKinematicsPulley;		//!< compute inverse kineamtics using the pulley model 
	bool m_bAnalyzeForwardKinematics;			//!< evaluate forward kinematics with the standard model
	bool m_bAnalyzeForwardKinematicsDistance;	//!< test forward kinematics with the standard model but the distant equation formulation
	bool m_bAnalyzeForwardKinematicsPulley;		//!< test forward kinematics with the extended pulley kinematics equations
	bool m_bAnalyzeElastoGeometricalForwardKinematics;
	bool m_bAnalyzeVelocityTransmission;
	bool m_bAnalyzeDexterity;					//!< compute key figures on the pose dependent structure matrix
	bool m_bAnalyzeStiffness;					//!< perform stiffness computations for the pose
	bool m_bAnalyzeWrenchSet;					//!< compute the available wrench set for the pose
	bool m_bAnalyzeOrientationRange;			//!< a pose pedendent workspace test for measuring available rotation
	bool m_bAnalyzeTranslationRange;			//!< a pose dependent workspace test for measuring the free translational rang
	bool m_bAnalyzeForceDistribution;			//!< compute a force distribution for the current pose
	bool m_bAnalyzeForceFrame;					//!< calculate the force vectors acting on the base frame 
	bool m_bAnalyzeDynamic;						//!< compute kinetostatic effects for given velocity and acceleration
	bool m_bAnalyseCableWear;					//!< track cable wear with movement
	bool m_bCompareInverseKinematics;			//!< enable difference testing for standard inverse kinematics and pulley kinematics

	bool m_bPanCamera;							//!< if true, then pan the camera while playing nc-programm
	bool m_bVerboseMode;						//!< if true, then UpdatePose will print information to console

	//! a boolean to indicate, if the pose pane should be updated immediatly when moving the robot
	bool m_bInstantPosePaneUpdate;

	//! there should be only one CWireCenterView object; some methods (e.g. callbacks) need to access this objects through a static pointer
	static CWireCenterView* This;
	
	//! loading and playback of nc programs; PoseList stores the trajectory as a list of poses
	PCRL::CPoseListKinetostatic PoseList;

	CString m_NC_filename; //!< save the path and filename into the member variable of WireCenterView for automatic reloading the NC-program after changing options of NC-Interpolator

	void NC_Driver(CString& filename);
	bool LoadNcProgram(const CString& filename);
	bool LoadPoseEvaluatorSettings(const CString& filename);
	bool SavePoseEvaluatorSettings(const CString& filename);
	void playPoseList();

	void createPoseListRandom(int numberOfPoses, const Vector3d& r_min, const Vector3d& r_max, const Vector3d& R_limit);
	void createPoseListGrid(const Vector3d& r_min, const Vector3d& r_max, const Vector3d& r_stepSize);
	void createPoseListGridRandomRot(const Vector3d& r_min, const Vector3d& r_max, const Vector3d& r_stepSize, const Vector3d& R_limit);
 

private:
	// the private members are states of the running instance rather than configuration parameter
	bool m_bRunningProgram;					//!< indicate the an NC program is currently played back

public:
	CPathGenerator PathGenerator;	//!< class to generate axis aligned line section through an STL surface

	// shape objects, i.e. instances of visual entities in the IPAGL scene graph
	PCRL::CShapeWorkspaceHull* pHull;
	PCRL::CShapeWorkspaceDiffHull* pWDH;
	PCRL::CShapeRobot* pRobotShape;
	PCRL::CShapeInterference* pInterference;
	PCRL::CShapeWireSpan* pWireSpan;
	PCRL::CShapeWireSpanCone* pWireSpanCone;
	PCRL::CShapeCrosssection* pCS;
	PCRL::CShapePlatformForces* pPlatformForces;
	PCRL::CShapeWinchPulleys* pWinchPulleys;
	PCRL::CShapePoseListKinetostatic* pNcLineStrip;
	PCRL::CShapePosePropertyEvaluation* pPosePropertyEvaluation;
	CGLShapeCoordinatePlanes* pCoordinatePlanes;
	CGLShapeGroundPlane* pGroundPlanes;

	// control coloring of shapes
	CGLRGBColor ColorControlPoints;

	//CGLShapeTriangulatedSurface HullShape;
	CGLShapeTriangulatedSurface WrenchSet;
	
	//! recalculate properties displayed in the view
	void UpdateStatistics();
	void UpdateWorkspaceHull();

	void setWorkspaceVisibility(const bool isVisible);
	void setWireSpanVisibility(const bool isVisible);
	void setWiresVisibility(const bool isVisible);
	void setWinchesVisibility(const bool isVisible);
	void setPlatformVisibility(const bool isVisible);
	void setFrameBoundingBoxVisibility(const bool isVisible);
	void setPlatformBoundingBoxVisibility(const bool isVisible);
	void setPlanesVisibility(const bool isVisible);
	void setInterferenceVisibility(const bool isVisible);
	void setCableForcesVisibility(const bool isVisible);
	void setWinchPulleyVisibility(const bool isVisible);
	void setPoseEstimateBoxVisibility(const bool isVisible);
	void setPosEstimateBoxSpheresVisibility(const bool isVisible);
	void UpdatePosePane();
	void UpdateGeometryPane();
	void getWorkspaceOptionsFromSidebar();
	void setWorkspaceOptionsInSidebar();

	void EvaluateVisibilityButtons(UINT nID, BOOL b_IsUpdateCall, CCmdUI *pCmdUI);

	void Invalidate(BOOL bErase = 1);
	void modifyParameterValue(const double& change);

	void draw(); //!< virtual call-back function of CGLShape to enable openGL drawing code
// Überschreibungen

	void BerechnenTrajektorieanalysieren(CString filename);
public:
	virtual void OnDraw(CDC* pDC);  // Überschrieben, um diese Ansicht darzustellen
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:

// Implementierung
public:
	virtual ~CWireCenterView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

// user defined message
	afx_msg LRESULT OnUpdatePose(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnUpdateWorkspace(WPARAM wParam, LPARAM lParam);

protected:

// Generierte Funktionen für die Meldungstabellen
protected:
	DECLARE_MESSAGE_MAP()
public:
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnDestroy();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnBnClickedCalculateworkspaceButton();
	virtual void OnInitialUpdate();
	afx_msg void OnBnClickedWorkspaceVisibleCheck();
	afx_msg void OnBnClickedWorkspaceWiresVisibleCheck();
	afx_msg void OnShowWindow(BOOL bShow, UINT nStatus);
	virtual void OnUpdate(CView* /*pSender*/=0, LPARAM /*lHint*/=0, CObject* /*pHint*/=0);
	afx_msg void OnBnClickedWorkspaceInterferenceVisibleCheck();
	afx_msg void OnBnClickedWorkspacePlanesVisibleCheck();
	afx_msg void OnArbeitsraumBerechnen();
	afx_msg void OnArbeitsraumDifferentielberechnen();
	afx_msg void OnAnsichtAusrichtenxyebene();
	afx_msg void OnAnsichtAusrichtenxzebene();
	afx_msg void OnAnsichtAusrichtenyzebene();
	afx_msg void OnAnsichtZuruecksetzen();
	afx_msg void OnArbeitsraumIntervallmethode();
	afx_msg void OnArbeitsraumKonturCustomX();
	afx_msg void OnArbeitsraumKonturCustomY();
	afx_msg void OnArbeitsraumKonturCustomZ();
	afx_msg void OnUpdateArbeitsraumKontur(CCmdUI *pCmdUI);
	afx_msg void OnArbeitsraumVereinigen();
	afx_msg void OnArbeitsraumSchneiden();
	afx_msg void OnUpdateAnsichtSichtbarkeit(CCmdUI *pCmdUI);
	afx_msg void OnAnsichtSichtbarkeitRange(UINT nID);
	afx_msg void OnUpdateAnsichtSichtbarkeitRange(CCmdUI *pCmdUI);
	afx_msg void OnArbeitsraumOrientierungautomatischuebernehmen();
	afx_msg void OnUpdateArbeitsraumOrientierungautomatischuebernehmen(CCmdUI *pCmdUI);
	afx_msg void OnUpdateArbeitsraumAlleorientierungenpr32824(CCmdUI *pCmdUI);
	afx_msg void OnEntwurfAnwendungsdatenanzeigen();
	afx_msg void OnUpdateEntwurfAnwendungsdatenanzeigen(CCmdUI *pCmdUI);
	afx_msg void OnUpdateArbeitsraumKollisionszonensichtbar(CCmdUI *pCmdUI);
	afx_msg void OnArbeitsraumKollisionszonensichtbar();
	afx_msg void OnBerechnenAutomatischeposenanalyseoptionen();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnNcLaden();
	afx_msg void OnNcOptionen();
	afx_msg void OnNcStarten();
	afx_msg void OnNcStoppen();
	afx_msg void OnNcFortsetzen();
	afx_msg void OnUpdateNcStarten(CCmdUI *pCmdUI);
	afx_msg void OnUpdateNcStoppen(CCmdUI *pCmdUI);
	afx_msg void OnUpdateNcFortsetzen(CCmdUI *pCmdUI);
	afx_msg void OnBerechnenTrajektorieanalysieren();
	afx_msg void OnUpdateBerechnenTrajektorieanalysieren(CCmdUI *pCmdUI);
	afx_msg void OnNcAviexportieren();
	afx_msg void OnAviExportRotation();
	afx_msg void OnUpdateNcAviexportieren(CCmdUI *pCmdUI);
	afx_msg void OnUpdateNcLaden(CCmdUI *pCmdUI);
	afx_msg void OnIterationEdit();
	afx_msg void OnEpsEdit();
	afx_msg void OnNumberorientationEdit();
	afx_msg void OnUpdateNumberorientationEdit(CCmdUI *pCmdUI);
	afx_msg void OnArbeitsraumMultithreading();
	afx_msg void OnUpdateArbeitsraumMultithreading(CCmdUI *pCmdUI);
	afx_msg void OnGitterSichtbarButton();
	afx_msg void OnUpdateGitterSichtbarButton(CCmdUI *pCmdUI);
	afx_msg void OnKonturSichtbarButton();
	afx_msg void OnUpdateKonturSichtbarButton(CCmdUI *pCmdUI);
	afx_msg void OnArbeitsraumGitterButton();
	afx_msg void OnArbeitsraumGitterCustomButton();
	afx_msg void OnDifferenzielSichtbarButton();
	afx_msg void OnUpdateDifferenzielSichtbarButton(CCmdUI *pCmdUI);
	afx_msg void OnWinchSelectorCombo();
	afx_msg void OnCliptoboxButton();
	afx_msg void OnCliptoframeButton();
	afx_msg void OnParameterModelCombo();
	afx_msg void OnParameterTypeCombo();
	afx_msg void OnParameterValueEdit();
	afx_msg void OnParameterSetButton();
	afx_msg void OnParamdecButton();
	afx_msg void OnParamfastdecButton();
	afx_msg void OnParamfastincButton();
	afx_msg void OnParamincButton();
	afx_msg void OnArbeitsraumHuelleSichtbarButton();
	afx_msg void OnUpdateArbeitsraumHuelleSichtbarButton(CCmdUI *pCmdUI);
	afx_msg void OnSceneBackgroundButton();
	afx_msg void OnUpdateSceneBackgroundButton(CCmdUI *pCmdUI);
	afx_msg void OnSceneUppercolorButton();
	afx_msg void OnSceneLowercolorButton();
	afx_msg void OnScenePerspectiveButton();
	afx_msg void OnUpdateScenePerspectiveButton(CCmdUI *pCmdUI);
	afx_msg void OnSceneDeleteallshapesButton();
	afx_msg void OnScreenshotButton();
	afx_msg void OnExportWorkspaceButton();
	afx_msg void OnExportCrosssectionButton();
	afx_msg void OnSceneSettingsButton();
	afx_msg void OnDragMode3DViewButton();
	afx_msg void OnUpdateDragMode3DViewButton(CCmdUI *pCmdUI);
	afx_msg void OnRoiVisibleButton();
	afx_msg void OnUpdateRoiVisibleButton(CCmdUI *pCmdUI);
	afx_msg void OnRoiAutoApplyButton();
	afx_msg void OnUpdateRoiAutoApplyButton(CCmdUI *pCmdUI);
	afx_msg void OnClipToRoiButton();
	afx_msg void OnArbeitsraumGridShowInvalid();
	afx_msg void OnUpdateArbeitsraumGridShowInvalid(CCmdUI *pCmdUI);
	afx_msg void OnGenerateGridButtonButton();
	afx_msg void OnCreateGridRandomButton();
	afx_msg void OnCreateGridZlayerButton();
	afx_msg void OnCablespanLinesVisibleButton();
	afx_msg void OnCablespanConeVisibileButton();
	afx_msg void OnPoselistViewButton();
	afx_msg void OnPoselistSaveButton();
	afx_msg void OnUpdatePoselistSaveButton(CCmdUI *pCmdUI);
	afx_msg void OnUpdatePoselistViewButton(CCmdUI *pCmdUI);
	afx_msg void OnConfigureShapeColorsButton();
};

#ifndef _DEBUG  // Debugversion in WireCenterView.cpp
inline CWireCenterDoc* CWireCenterView::GetDocument() const
   { return static_cast<CWireCenterDoc*>(m_pDocument); }
inline PCRL::CRobotDocument* CWireCenterView::GetRobotDoc() const
{ return &static_cast<CWireCenterDoc*>(m_pDocument)->robotDoc; }
#endif

