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

/*! \file WireCenterDoc.h
 *
 *	\author   Andreas Pott
 *
 *  \brief The document class should store all data object and make them
 *  accessable throughout the application (especially in the view classes). 
 *  Beside the pure data objects of the robot also the algorithm objects
 *  are located here. 
 */

#pragma once

#include <WireLib/RobotDocument.h>
#include <WireLib/ApplicationRequirement.h>
#include <WireLib/PosePropertyEvaluation.h>
#include <IPAGL/GLGeometry.h>

class CWireCenterDoc : public CDocument
{
protected: // Nur aus Serialisierung erstellen
	CWireCenterDoc();
	DECLARE_DYNCREATE(CWireCenterDoc)

// Attribute
public:
	PCRL::CRobotDocument robotDoc;				//!< the cable robot object including geometry data model and algorithm collection
	PCRL::CPosePropertyEvaluation Evaluator;				//!< the collection of evaluation algorithms

	// region of interest properties
	PCRL::CBox Roi;								//!< region of interest for computations performed for the robot

	//! visualitation data for a trace
	std::vector<CFrame*> trace;

	CString filename;							//!< filename of the current robot configuration file
	CString py_filename;						//!< filename of the current python script
	CString report;								//!< filename of the current report to be shown by the view
	bool firstInit;								//!< indicates the first start-up of the program
// Vorgänge
public:
	bool ReloadRobot();
	bool KalibrierungSteuerungsdateiimportieren(CString filename);
	std::string getWireCenterPath();

	bool loadProject(const string& filename);
	bool saveProject(const string& filename);

// Überschreibungen
public:
	virtual BOOL OnNewDocument();
	BOOL DoSave(LPCTSTR lpszPathName, BOOL bReplace);
	virtual void Serialize(CArchive& ar);
	bool LoadRobot(CString& filename);

// Implementierung
public:
	virtual ~CWireCenterDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generierte Funktionen für die Meldungstabellen
protected:
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBerechnenSteifigkeit();
	afx_msg void OnBerechnenKinematik();
	afx_msg void OnBearbeitenGeometrie();
	afx_msg void OnArbeitsraumOptionen();
	afx_msg void OnFileOpen();
	afx_msg void OnKalibrierungSteuerungsdateiexportieren();
	afx_msg void OnKalibrierungSteuerungsdateiimportieren();
	afx_msg void OnTransformierenMesspunktezurahmenpunkte();
	afx_msg void OnScriptEditieren();
	afx_msg void OnScriptLaden();
	afx_msg void OnScriptStarten();
	afx_msg void OnArbeitsraumAlleorientierungenpr32824();
	afx_msg void OnArbeitsraumOrientierungfestlegen();
	afx_msg void OnArbeitsraumOrientierungenhinzuf32822();
	afx_msg void OnArbeitsraumOrientierungen();
	afx_msg void OnBearbeitenSeillaengen();
	afx_msg void OnDateiSpeichernalsxml();
	afx_msg void OnEntwurfAnwendungbeschreiben();
	afx_msg void OnEntwurfSeillaengeFuerArbeitsraum();
	afx_msg void OnEntwurfSeilkraefteFuerArbeitsraum();
	afx_msg void OnEntwurfSeilwinkelFuerArbeitsraum();
	afx_msg void OnEntwurfWindenparameter();
	afx_msg void OnEntwurfAnforderungenladen();
	afx_msg void OnEntwurfAnwendungsanforderungenspeichern();
	afx_msg void OnEntwurfKontrolierbarkeitfuerarbeitsraum();
	afx_msg void OnArbeitsraumKollisionberechnen();
	afx_msg void OnArbeitsraumKonturgitter();
	afx_msg void OnOrientationOptions();
	afx_msg void OnEntwurfWindenparameterladen();
	afx_msg void OnEntwurfWindenparameterspeichern();
	afx_msg void OnEntwurfSeilparameter();
	afx_msg void OnEntwurfSeilparameterladen();
	afx_msg void OnEntwurfSeilparameterspeichern();
	afx_msg void OnApplicationFeasibilitycheck();
	afx_msg void OnExportCrosssectionButton();
	afx_msg void OnExportWorkspaceButton();
	afx_msg void OnAutogenRoiButton();
	afx_msg void OnRoiEditButton();
	afx_msg void OnRoiFromrequirementButton();
	afx_msg void OnRoiFromreqInstallationButton();
	afx_msg void OnPermutationButton();
	afx_msg void OnPermutePlatformXpButton();
	afx_msg void OnPermuteBaseXpButton();
	afx_msg void OnPermuteBaseYpButton();
	afx_msg void OnPermuteBaseZpButton();
	afx_msg void OnPermutePlatformYpButton();
	afx_msg void OnPermutePlatformZpButton();
	afx_msg void OnCableforceassistentButton();
	afx_msg void OnSaveConfiguriatonXmlButton();
	afx_msg void OnLoadConfiguriatonXmlButton();
	afx_msg void OnArbeitsraumGridCylinder();
	afx_msg void OnArbeitsraumGridRandom();
	afx_msg void OnConfigureKinematicModelButton();
	afx_msg void OnBearbeitenUmlenkrollenButton();
	afx_msg void OnSaveGridButton();
	afx_msg void OnRobotMetadataButton();
	afx_msg void OnImportRobotGeometryButton();
	afx_msg void OnExportRobotButton();
	afx_msg void OnCablespanCalculateButton();
	afx_msg void OnShowStructurematrixButton();
	afx_msg void OnShowStiffnessmatrixButton();
	afx_msg void OnInterferenceCablecablematrixButton();
	afx_msg void OnInterferenceCableplatformmatrixButton();
	afx_msg void OnShowDifferentialhullParameters();
	afx_msg void OnBearbeitenGeometry();
};