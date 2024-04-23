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

// WireCenterDoc.cpp : Implementierung der Klasse CWireCenterDoc
//

#include "stdafx.h"
#include "WireCenter.h"

#include "WireCenterDoc.h"
#include "PoseDlg.h"
#include "GeometryDlg.h"
#include "WorkspaceOptionsDlg.h"
#include "GenericParamDlg.h"
#include "RobotPropertiesDlg.h"
#include "OrientationDlg.h"
#include "CableForceDlg.h"
#include "DraftDlgs.h"

#include "PythonInterface.h"
#include "PyScriptDlg.h"
#include "MainFrm.h"
#include "aes256.h"
#include "WireCenterProjectfile.h"
#include "PersistantDefaultValue.h"
 
#include <map>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CWireCenterDoc

IMPLEMENT_DYNCREATE(CWireCenterDoc, CDocument)

BEGIN_MESSAGE_MAP(CWireCenterDoc, CDocument)
	ON_COMMAND(ID_BERECHNEN_STEIFIGKEIT, &CWireCenterDoc::OnBerechnenSteifigkeit)
	ON_COMMAND(ID_BERECHNEN_KINEMATIK, &CWireCenterDoc::OnBerechnenKinematik)
	ON_COMMAND(ID_BEARBEITEN_GEOMETRIE, &CWireCenterDoc::OnBearbeitenGeometrie)
	ON_COMMAND(ID_ARBEITSRAUM_OPTIONEN, &CWireCenterDoc::OnArbeitsraumOptionen)
	ON_COMMAND(ID_FILE_OPEN, &CWireCenterDoc::OnFileOpen)
	ON_COMMAND(ID_KALIBRIERUNG_STEUERUNGSDATEIEXPORTIEREN, &CWireCenterDoc::OnKalibrierungSteuerungsdateiexportieren)
	ON_COMMAND(ID_KALIBRIERUNG_STEUERUNGSDATEIIMPORTIEREIN, &CWireCenterDoc::OnKalibrierungSteuerungsdateiimportieren)
	ON_COMMAND(ID_TRANSFORMIEREN_MESSPUNKTEZURAHMENPUNKTE, &CWireCenterDoc::OnTransformierenMesspunktezurahmenpunkte)
	ON_COMMAND(ID_SCRIPT_EDITIEREN, &CWireCenterDoc::OnScriptEditieren)
	ON_COMMAND(ID_SCRIPT_LADEN, &CWireCenterDoc::OnScriptLaden)
	ON_COMMAND(ID_SCRIPT_STARTEN, &CWireCenterDoc::OnScriptStarten)
	ON_COMMAND(ID_ARBEITSRAUM_ALLEORIENTIERUNGENPR32824, &CWireCenterDoc::OnArbeitsraumAlleorientierungenpr32824)
	ON_COMMAND(ID_ARBEITSRAUM_ORIENTIERUNGFESTLEGEN, &CWireCenterDoc::OnArbeitsraumOrientierungfestlegen)
	ON_COMMAND(ID_ARBEITSRAUM_ORIENTIERUNGENHINZUF32822, &CWireCenterDoc::OnArbeitsraumOrientierungenhinzuf32822)
	ON_COMMAND(ID_ARBEITSRAUM_ORIENTIERUNGEN, &CWireCenterDoc::OnArbeitsraumOrientierungen)
	ON_COMMAND(ID_BEARBEITEN_SEILLAENGEN, &CWireCenterDoc::OnBearbeitenSeillaengen)
	ON_COMMAND(ID_DATEI_SPEICHERNALSXML, &CWireCenterDoc::OnDateiSpeichernalsxml)
	ON_COMMAND(ID_ENTWURF_ANWENDUNGBESCHREIBEN, &CWireCenterDoc::OnEntwurfAnwendungbeschreiben)
	ON_COMMAND(ID_ENTWURF_SEILLAENGE_FUER_ARBEITSRAUM, &CWireCenterDoc::OnEntwurfSeillaengeFuerArbeitsraum)
	ON_COMMAND(ID_ENTWURF_SEILKRAEFTE_FUER_ARBEITSRAUM, &CWireCenterDoc::OnEntwurfSeilkraefteFuerArbeitsraum)
	ON_COMMAND(ID_ENTWURF_SEILWINKEL_FUER_ARBEITSRAUM, &CWireCenterDoc::OnEntwurfSeilwinkelFuerArbeitsraum)
	ON_COMMAND(ID_ENTWURF_WINDENPARAMETER, &CWireCenterDoc::OnEntwurfWindenparameter)
	ON_COMMAND(ID_ENTWURF_ANFORDERUNGENLADEN, &CWireCenterDoc::OnEntwurfAnforderungenladen)
	ON_COMMAND(ID_ENTWURF_ANWENDUNGSANFORDERUNGENSPEICHERN, &CWireCenterDoc::OnEntwurfAnwendungsanforderungenspeichern)
	ON_COMMAND(ID_ENTWURF_KONTROLIERBARKEITFUERARBEITSRAUM, &CWireCenterDoc::OnEntwurfKontrolierbarkeitfuerarbeitsraum)
	ON_COMMAND(ID_ARBEITSRAUM_KOLLISIONBERECHNEN, &CWireCenterDoc::OnArbeitsraumKollisionberechnen)
	ON_COMMAND(ID_ARBEITSRAUM_KONTURGITTER, &CWireCenterDoc::OnArbeitsraumKonturgitter)
	ON_COMMAND(ID_WORKSPACE_GRID_CYLINDRIC_BUTTON, &CWireCenterDoc::OnArbeitsraumGridCylinder)
	ON_COMMAND(ID_WORKSPACE_GRID_RANDOM_BUTTON, &CWireCenterDoc::OnArbeitsraumGridRandom)
	ON_COMMAND(ID_ORIENTATION_OPTIONS, &CWireCenterDoc::OnOrientationOptions)
	ON_COMMAND(ID_ENTWURF_WINDENPARAMETERLADEN, &CWireCenterDoc::OnEntwurfWindenparameterladen)
	ON_COMMAND(ID_ENTWURF_WINDENPARAMETERSPEICHERN, &CWireCenterDoc::OnEntwurfWindenparameterspeichern)
	ON_COMMAND(ID_ENTWURF_SEILPARAMETER, &CWireCenterDoc::OnEntwurfSeilparameter)
	ON_COMMAND(ID_ENTWURF_SEILPARAMETERLADEN, &CWireCenterDoc::OnEntwurfSeilparameterladen)
	ON_COMMAND(ID_ENTWURF_SEILPARAMETERSPEICHERN, &CWireCenterDoc::OnEntwurfSeilparameterspeichern)
	ON_COMMAND(IDC_APPLICATION_FEASIBILITYCHECK, &CWireCenterDoc::OnApplicationFeasibilitycheck)
	ON_COMMAND(ID_EXPORT_WORKSPACE, &CWireCenterDoc::OnExportWorkspaceButton)
	ON_COMMAND(ID_EXPORT_CROSSSECTION, &CWireCenterDoc::OnExportCrosssectionButton)
	ON_COMMAND(ID_AUTOGEN_ROI_BUTTON, &CWireCenterDoc::OnAutogenRoiButton)
	ON_COMMAND(ID_ROI_EDIT_BUTTON, &CWireCenterDoc::OnRoiEditButton)
	ON_COMMAND(ID_ROI_FROMREQUIREMENT_BUTTON, &CWireCenterDoc::OnRoiFromrequirementButton)
	ON_COMMAND(ID_ROI_FROMREQ_INSTALLATION_BUTTON, &CWireCenterDoc::OnRoiFromreqInstallationButton)
	ON_COMMAND(ID_PERMUTATION_BUTTON, &CWireCenterDoc::OnPermutationButton)
	ON_COMMAND(ID_PERMUTE_PLATFORM_XP_BUTTON, &CWireCenterDoc::OnPermutePlatformXpButton)
	ON_COMMAND(ID_PERMUTE_BASE_XP_BUTTON, &CWireCenterDoc::OnPermuteBaseXpButton)
	ON_COMMAND(ID_PERMUTE_BASE_YP_BUTTON, &CWireCenterDoc::OnPermuteBaseYpButton)
	ON_COMMAND(ID_PERMUTE_BASE_ZP_BUTTON, &CWireCenterDoc::OnPermuteBaseZpButton)
	ON_COMMAND(ID_PERMUTE_PLATFORM_YP_BUTTON, &CWireCenterDoc::OnPermutePlatformYpButton)
	ON_COMMAND(ID_PERMUTE_PLATFORM_ZP_BUTTON, &CWireCenterDoc::OnPermutePlatformZpButton)
	ON_COMMAND(ID_CABLEFORCEASSISTENT_BUTTON, &CWireCenterDoc::OnCableforceassistentButton)
	ON_COMMAND(ID_SAVE_CONFIGURATION_XML_BUTTON, &CWireCenterDoc::OnSaveConfiguriatonXmlButton)
	ON_COMMAND(ID_LOAD_CONFIGURATION_XML_BUTTON, &CWireCenterDoc::OnLoadConfiguriatonXmlButton)
	ON_COMMAND(ID_CONFIGURE_KINEMATIC_MODEL_BUTTON, &CWireCenterDoc::OnConfigureKinematicModelButton)
	ON_COMMAND(ID_BEARBEITEN_UMLENKROLLEN_BUTTON, &CWireCenterDoc::OnBearbeitenUmlenkrollenButton)
	ON_COMMAND(ID_SAVEGRID_BUTTON,&CWireCenterDoc::OnSaveGridButton)
	ON_COMMAND(ID_ROBOT_METADATA_BUTTON, &CWireCenterDoc::OnRobotMetadataButton)
	ON_COMMAND(ID_IMPORT_ROBOT_GEOMETRY_BUTTON, &CWireCenterDoc::OnImportRobotGeometryButton)
	ON_COMMAND(ID_EXPORT_ROBOT_BUTTON, &CWireCenterDoc::OnExportRobotButton)
	ON_COMMAND(ID_CABLESPAN_CALCULATE_BUTTON, &CWireCenterDoc::OnCablespanCalculateButton)
	ON_COMMAND(ID_SHOW_STRUCTUREMATRIX_BUTTON8, &CWireCenterDoc::OnShowStructurematrixButton)
	ON_COMMAND(ID_SHOW_STIFFNESSMATRIX_BUTTON, &CWireCenterDoc::OnShowStiffnessmatrixButton)
	ON_COMMAND(ID_BUTTON5, &CWireCenterDoc::OnInterferenceCablecablematrixButton)
	ON_COMMAND(ID_INTERFERENCE_CABLECABLEMATRIX_BUTTON, &CWireCenterDoc::OnInterferenceCablecablematrixButton)
	ON_COMMAND(ID_INTERFERENCE_CABLEPLATFORMMATRIX_BUTTON, &CWireCenterDoc::OnInterferenceCableplatformmatrixButton)
	ON_COMMAND(ID_SHOW_DIFFERENTIALHULL_PARAMETERS, &CWireCenterDoc::OnShowDifferentialhullParameters)
	ON_COMMAND(ID_BEARBEITEN_GEOMETRY, &CWireCenterDoc::OnBearbeitenGeometry)
END_MESSAGE_MAP()


// CWireCenterDoc-Erstellung/Zerstörung

CWireCenterDoc::CWireCenterDoc() : Evaluator(robotDoc)
{
	// \todo generate the filename of the report file; this is somewhat dangerous but pragmatic
	char path[1024];
	GetCurrentDirectoryA(1024,path);
	report="file:///";
    report+=path;
	report+="\\report.html";

	firstInit = true;
	
	std::string WirecenterPath;
	WirecenterPath=getWireCenterPath();
	WirecenterPath.append("\\IPAnema.wcrfx"); // for release: filename="robots\\IPAnema.wcrfx";
	
	filename= WirecenterPath.c_str();

	Roi.lower() = Vector3d(0,0,0);
	Roi.upper() = Vector3d(0,0,0);

	py_filename="";
}

CWireCenterDoc::~CWireCenterDoc()
{
}

BOOL CWireCenterDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	// during the inital startup of WireCenter we load the default robot
	if (firstInit)
	{
		firstInit=false;
		robotDoc.ParametricModels.applyModel("IPAnema1Design");
/*		if (LoadRobot(filename))
			this->SetTitle(filename);*/
	}
	else
	{	// new during operation opens the new robot dialog
		// generate a new robot from a dialog information
		CRobotPropertiesDlg RP;
		if (RP.DoModal()==IDOK)
		{
			// copy the whole robot document object
			RP.RDoc.resetStates(); // hot fix: ensure, that the data model is consistent
			robotDoc = RP.RDoc;

			// copy some additional informations from the dialog box
			robotDoc.name = RP.RDoc.name;
			robotDoc.desc = RP.RDoc.desc;
			robotDoc.author = RP.RDoc.author;
			robotDoc.id = RP.RDoc.id;
			//set title of application window
			SetTitle(RP.m_Name);
		}
		else
			return FALSE;	// the new robot was not accepted by the used; we have to discard changes
	}

	// TODO: Hier Code zur Reinitialisierung einfügen
	// (SDI-Dokumente verwenden dieses Dokument)

	//! \todo reset the workspace hull object WSHull here!
	UpdateAllViews(0);
	return TRUE;
}


// CWireCenterDoc-Serialisierung

void CWireCenterDoc::Serialize(CArchive& ar)
{
	TRACE0("CWireCenterDoc::Serialize(...): this function is not implemted");
	if (ar.IsStoring())
	{
		// TODO: Hier Code zum Speichern einfügen
	}
	else
	{
		// TODO: Hier Code zum Laden einfügen
	}
}


std::string CWireCenterDoc::getWireCenterPath()
{
	char buffer[MAX_PATH];
	std::string::size_type pos; 
	std::string WireCenterPath; 
	
	GetModuleFileName(NULL, buffer, MAX_PATH);

	pos = std::string( buffer ).find_last_of( "\\/" );
	WireCenterPath = string( buffer ).substr( 0, pos);
		
	return WireCenterPath;
}

// CWireCenterDoc-Diagnose

#ifdef _DEBUG
void CWireCenterDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CWireCenterDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG

void CWireCenterDoc::OnBerechnenSteifigkeit()
{
	cout << "Stiffness for psoe (r,R) = " << robotDoc.r.transpose() << ",\n " << robotDoc.R << endl;
	// specific stiffness coefficient per length unit k' = E*A = E*r^2*Pi
	
	// the following method is not compatible with actual xml (WEK, 03/12)
	//robotDoc.Stiffness.setStiffnessCoefficient(robotDoc.currentCable->E_wire * robotDoc.currentCable->r_cable*robotDoc.currentCable->r_cable*MO_PI);

	robotDoc.Stiffness.StiffnessMatrix(robotDoc.r,robotDoc.R);
	cout << "Minimum stiffness: " << robotDoc.Stiffness.getMinimalStiffness() <<endl;

	robotDoc.Stiffness.StiffnessMatrixHomogenous(robotDoc.r,robotDoc.R);
	cout << "Minimum stiffenss homogenious: " << robotDoc.Stiffness.getMinimalStiffness() << endl;

	robotDoc.Stiffness.StiffnessMatrixTranslational(robotDoc.r,robotDoc.R);
	cout << "Minimum translational stiffness: " << robotDoc.Stiffness.getMinimalStiffness() <<endl;
}

#ifdef USE_ORIGINAL_CODE

// Testroutine für die Kinematik für die 6D-Posenschätzung mit den 
// Seilauszugssensoren.
// special code for M. Kapica's debugging
void CWireCenterDoc::OnBerechnenKinematik()
{
	CFileDialog FileDlg(TRUE,	// load File Dialog
						".txt",
						"Messprotokoll_mp_5.txt",
						OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"Textfile (*.txt)|*.txt||");
	if (FileDlg.DoModal()!=IDOK)
		return;

	ifstream file(FileDlg.GetFileName().GetString());
	ofstream prot((FileDlg.GetFileName()+".prot").GetString());

	double L[6];
	Vector3d r;
	Matrix3d R=Matrix3d::ZRotationMatrix3d(0);

	int k=0;

	// clear the data model before refilling it
	for (unsigned int i=0; i<trace.size(); i++)
		delete trace[i];
	trace.clear();

	while (!file.eof())
	{
		k++;
		prot << k << "\t";
		// read one data set from the file
		for (int i=0; i<6; i++)
		{
			file >> L[i];
			L[i]/=1000.0;	// rescale Matthias data from [mm] to [m]
			L[i]+=0.0975;	// perform offset compensation
			prot << L[i] << "\t";
		}
		prot << endl;

		// calculate the kinematics
		if (robotDoc.Kinematics.doForwardKinematics(L,r,R))
		{
			prot << "Success: Estimated Pose is " << r << R << endl
			    << "Delta Pose: " << Kinematics.getDeltaPose() <<endl
				<< "FunctionEvals: " << Kinematics.getFunctionEvals()<<endl
				<< "Iterations: " << Kinematics.getIterations()<<endl
				<< "TerminationCode: " << Kinematics.getTerminationCode()<<endl
				<< "InitalObjectiveFunctionError: " << Kinematics.getInitalObjectiveFunctionError()<<endl
				<< "FinalObjectiveFunctionError: " << Kinematics.getFinalObjectiveFunctionError()<<endl<<endl;
			CFrame *pFrame = new CFrame;
			r=r*R;
			pFrame->r.x = r.x;
			pFrame->r.y = r.y;
			pFrame->r.z = r.z;
			pFrame->R.e1.x = R.e1.x;
			pFrame->R.e1.y = R.e1.y;
			pFrame->R.e1.z = R.e1.z;
			pFrame->R.e2.x = R.e2.x;
			pFrame->R.e2.y = R.e2.y;
			pFrame->R.e2.z = R.e2.z;
			pFrame->R.e3.x = R.e3.x;
			pFrame->R.e3.y = R.e3.y;
			pFrame->R.e3.z = R.e3.z;
			trace.push_back(pFrame);
		}
		else
		{
			prot<< "Pose estimated failed\n"
			    << "Delta Pose: " << Kinematics.getDeltaPose() <<endl
				<< "FunctionEvals: " << Kinematics.getFunctionEvals()<<endl
				<< "Iterations: " << Kinematics.getIterations()<<endl
				<< "TerminationCode: " << Kinematics.getTerminationCode()<<endl
				<< "InitalObjectiveFunctionError: " << Kinematics.getInitalObjectiveFunctionError()<<endl
				<< "FinalObjectiveFunctionError: " << Kinematics.getFinalObjectiveFunctionError()<<endl;
		}
	}
}

#else

// test forward and inverse kinematic
// generate templates for diagrams in matlab style
void CWireCenterDoc::OnBerechnenKinematik()
{
	time_t start=clock();
	// create a cable vector
	MatrixXd l(robotDoc.getNow(),1);
	// testing parameters
	double 
		dx=0.8,
		dy=0.8,
		dz=0.5,
		da=5*DEG_TO_RAD,
		db=5*DEG_TO_RAD,
		dc=5*DEG_TO_RAD;
	double noise=0.0005;
	int numberOfTests = 5000;

	// statistical evaluation
	vector<double> IterationHistogram; IterationHistogram.resize(101);
	vector<int> ExitCodes;			ExitCodes.resize(8);
	vector<double> iterations;	iterations.resize(numberOfTests);
	vector<double> delta_p0;	delta_p0.resize(numberOfTests);
	vector<double> delta_pn;	delta_pn.resize(numberOfTests);

	// perform a couple of computations (here 1000)
	for (int i=0; i<numberOfTests; i++)
	{
		// generate random numbers [-1..1]
		double ex=((double)rand()*2)/RAND_MAX -1;
		double ey=((double)rand()*2)/RAND_MAX -1;
		double ez=((double)rand()*2)/RAND_MAX -1;
		double ea=((double)rand()*2)/RAND_MAX -1;
		double eb=((double)rand()*2)/RAND_MAX -1;
		double ec=((double)rand()*2)/RAND_MAX -1;

		Vector3d r(ex*dx,ey*dy,ez*dz);
		Matrix3d R=Matrix3d::ZRotationMatrix3d(ea*da)*Matrix3d::YRotationMatrix3d(eb*db)*Matrix3d::XRotationMatrix3d(ec*dc);
//		cout << r << " " << R << endl;

		robotDoc.Kinematics.doInverseKinematics(r,R,l);
		
		// add some noise to the wire length...
		for (int j=0; j<8; j++)
			l(j)+=(((double)rand()*2)/RAND_MAX -1)*noise;

		Matrix3d R0=R;
		Vector3d r0=r;
		robotDoc.Kinematics.doForwardKinematics(l,r,R);
		//robotDoc.Kinematics.doForwardKinematicsTwoStep(l,r,R);
/*		cout << "Error (Initial, Final,|J|, Delta_poes) = "
			<< Kinematics.getInitalObjectiveFunctionError() << "\t"
			<< Kinematics.getFinalObjectiveFunctionError() << "\t"
			<< Kinematics.getFinalJacobianNorm() << "\t"
			<< Kinematics.getDeltaPose() << "\n"
		    << "Evals (Iteration, Obj.-Func., Jacobian) = " 
			<< Kinematics.getIterations() << "\t"
			<< Kinematics.getFunctionEvals() << "\t"
			<< Kinematics.getJacobianEvals() << endl
			<< "Exit Code                               = "
			<< Kinematics.getTerminationCode() << endl;*/

		// save the statistics
		iterations[i] = robotDoc.Kinematics.getIterations();
		delta_p0[i] = robotDoc.Kinematics.getInitalObjectiveFunctionError();
		delta_pn[i] = (r-r0).norm();// Kinematics.getFinalObjectiveFunctionError();

		IterationHistogram[(int)robotDoc.Kinematics.getIterations()]+=1;
		ExitCodes[(int)robotDoc.Kinematics.getTerminationCode()]+=1;
		// check the overall error
//		if (length(r-r0)>1e-3) cout << "Error in Numerics: " << length(r-r0) << endl;
//		cout << length(robot.r-r0) 
//			<< " " << robot.R-R0 
//			<< endl;
	}

	cout << "Calculation time per function evaluation: " << (double)(clock()-start)/numberOfTests << " ms" << endl;
	
/*	cout << "Histogram #Iterations\n";
	for (int i=0; i<50; i++)
		cout << i << " " << IterationHistogram[i] << endl;*/
	
	cout << "Histogram Exit Codes\n";
	for (int i=1; i<8; i++)
		cout << i << " " << ExitCodes[i] << endl;

	// create matlab output
	std::ofstream file("kinematic.m");
	file << "% MATLAB Source\niter = [";
	for (int i=0; i<numberOfTests; i++)
		file << iterations[i] << ((i%20)?" ":" ...\n");
	file << "];\n";

	file << "delta_p0 = [";
	for (int i=0; i<numberOfTests; i++)
		file << delta_p0[i] << ((i%20)?" ":" ...\n");
	file << "];\n";

	file << "delta_pn = [";
	for (int i=0; i<numberOfTests; i++)
		file << delta_pn[i] << ((i%20)?" ":" ...\n");
	file << "];\n";

	file << "figure(1);\n"
		 << "iter_range=1:20;\n"
		 << "iter_his=hist(iter,iter_range);\n"
		 << "bar(iter_range,iter_his);\n"
		 << "grid on;\n"
		 << "xlabel('x');\n"
		 << "ylabel('y');\n"
		 << "\n"
	     << "figure(2);\n"
		 << "p0_range=logspace(log10(min(delta_p0)),log10(max(delta_p0)),20);\n"
		 << "p0_his=hist(delta_p0,p0_range);\n"
		 << "semilogx(p0_range,p0_his,'LineWidth',2);\n"
		 << "grid on;\n"
		 << "xlabel('x');\n"
		 << "ylabel('y');\n"
		 << "\n"
	     << "figure(3);\n"
		 << "pn_range=logspace(log10(min(delta_pn)),log10(max(delta_pn)),20);\n"
		 << "pn_his=hist(delta_pn,pn_range);\n"
		 << "semilogx(pn_range,pn_his,'LineWidth',2);\n"
		 << "grid on;\n"
		 << "xlabel('x');\n"
		 << "ylabel('y');\n";
}

#endif // USE_ORIGINAL_CODE

void CWireCenterDoc::OnBearbeitenGeometrie()
{
	CGeometryDlg Dlg;
	Dlg.pRobot=&robotDoc;
	Dlg.DoModal();
}

//! set the algorithm parameter for workspace determination
void CWireCenterDoc::OnArbeitsraumOptionen()
{
	CWorkspaceOptionsDlg Dlg;
	// copy actual data to dialog
	robotDoc.WSHull.getProjectionCenter(Dlg.m_x,Dlg.m_y,Dlg.m_z);
	Dlg.m_eps = robotDoc.WSHull.getEps();
	Dlg.m_Iterations = robotDoc.WSHull.getIterations();
	Dlg.m_MinCond = robotDoc.ForceDistribution.getConditionMin();
	Dlg.m_maxRadius = robotDoc.WSHull.getSearchRange();
	Dlg.m_fx = robotDoc.ForceDistribution.f.x();
	Dlg.m_fy = robotDoc.ForceDistribution.f.y();
	Dlg.m_fz = robotDoc.ForceDistribution.f.z();
	Dlg.m_tx = robotDoc.ForceDistribution.tau.x();
	Dlg.m_ty = robotDoc.ForceDistribution.tau.y();
	Dlg.m_tz = robotDoc.ForceDistribution.tau.z();
	Dlg.m_fmin = robotDoc.fmin;
	Dlg.m_fmax = robotDoc.fmax;
	Dlg.m_CheckAllOrientations = robotDoc.WSHull.getOrientationRequirement();
	Dlg.m_Criterion_Selector = robotDoc.WSHull.getWorkspaceCriterion();
	robotDoc.ForceDistribution.getMethod(Dlg.m_WorkspaceMethod_Selector,Dlg.m_WorkspaceCriterion_Selector);

	if (Dlg.DoModal()==IDOK)
	{
		// copy chaged data from dialog back to algorithm parameter
		robotDoc.WSHull.getProjectionCenter(Vector3d(Dlg.m_x,Dlg.m_y,Dlg.m_z));
		robotDoc.WSHull.setEps(Dlg.m_eps);
		robotDoc.WSHull.setIterations(Dlg.m_Iterations );
		robotDoc.ForceDistribution.setConditionMin(Dlg.m_MinCond);
		robotDoc.WSHull.setSearchRange(Dlg.m_maxRadius);
		robotDoc.ForceDistribution.f = Vector3d(Dlg.m_fx,Dlg.m_fy,Dlg.m_fz);
		robotDoc.ForceDistribution.tau = Vector3d(Dlg.m_tx,Dlg.m_ty,Dlg.m_tz);
		robotDoc.fmin = Dlg.m_fmin;
		robotDoc.fmax = Dlg.m_fmax;
		robotDoc.WSHull.setOrientationRequirement(Dlg.m_CheckAllOrientations==TRUE);
		robotDoc.ForceDistribution.setMethod(Dlg.m_WorkspaceMethod_Selector,Dlg.m_WorkspaceCriterion_Selector);
		robotDoc.WSHull.setWorkspaceCriterion((PCRL::CWorkspaceAlgorithm::eWorkspaceCriterion)Dlg.m_Criterion_Selector);
		//! \todo FIXME: we simply copy the settings from the main workspace hull object to the other workspace objects
		robotDoc.WSGrid.getAlgorithmParameter(robotDoc.WSHull);
		robotDoc.Crosssection.getAlgorithmParameter(robotDoc.WSHull);
		
		UpdateAllViews(0);
	}
}


/*! Open a WireCenter project file
 */
void CWireCenterDoc::OnFileOpen()
{
	CFileDialog FileDlg(TRUE,	// open File Dialog
						".wcpf",
						filename,
						OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"WireCenter Project File (*.wcpf)|*.wcpf||");
	
	//------------- VLS: Deployment to be reviewed -----------
	std::string WirecenterPath;
	WirecenterPath=getWireCenterPath();
	WirecenterPath.append("\\projects");

	FileDlg.m_ofn.lpstrInitialDir  = WirecenterPath.c_str(); 
	//------------- VLS: Deployment to be reviewed -----------

	if (FileDlg.DoModal()==IDOK)
	{
		loadProject(FileDlg.GetPathName().GetBuffer());
		UpdateAllViews(0);
	}
}

bool CWireCenterDoc::LoadRobot(CString& filename)
{
	// open the default document name
	if (filename.Find(".txt")>=0)
	{
		if (!robotDoc.Load((LPCTSTR)filename)) 
			// don't set the owner window handle to NULL, the application may crash sometimes, if it looses its focus on unappropriate moments
			MessageBox(AfxGetMainWnd()->m_hWnd,"Failed to load the robot data from txt file","Error",MB_OK);	
	}
	else if (filename.Find(".xml")>=0)
	{
		if (!robotDoc.loadXml((LPCTSTR)filename)) 
		{
			MessageBox(AfxGetMainWnd()->m_hWnd,"Failed to load the robot data from xml file","Error",MB_OK);
			return false;
		}
	}
	else if (filename.Find(".wcrfx")>=0)
	{
		int Row=0,Col=0;
		if (!robotDoc.loadXml((LPCTSTR)filename,Row,Col))		// this function should read the ATLAS-XML dialect
		{
			if (Row!=0)
			{
				std::ostringstream ErrorMessage;
				ErrorMessage<<"Failed to load the robot data! Check Line: " <<Row<< " and Collumn: "<<Col<<" in XML File";
				MessageBox(AfxGetMainWnd()->m_hWnd,ErrorMessage.str().c_str(),"Error",MB_OK);
				return false;
			}
			else
			{
				MessageBox(AfxGetMainWnd()->m_hWnd,"Failed to load the robot data from wcrfx file","Error",MB_OK);
				return false;
			}
		}
	}
	else
	{
		MessageBox(AfxGetMainWnd()->m_hWnd,"Failed to identify the type of the data file","Error",MB_OK);
		return false;
	}
	// if loading was successful, store the filename in the respective class member
	this->filename=filename;
	return true;
}

/*! Load the geometry of a robot from a file
 */
void CWireCenterDoc::OnImportRobotGeometryButton()
{
	CFileDialog FileDlg(TRUE,	// open File Dialog
						".txt;*.wcrfx",
						filename,
						OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"All Geometry Files (*.txt;*.wcrfx)|*.txt;*.wcrfx|"
						"Text Geometry Files (*.txt)|*.txt|"
						"XML Geometry Files (*.wcrfx)|*.wcrfx||");
	
	//------------- VLS: Deployment to be reviewed -----------
	std::string WirecenterPath;
	WirecenterPath=getWireCenterPath();
	WirecenterPath.append("\\robots");

	FileDlg.m_ofn.lpstrInitialDir  = WirecenterPath.c_str(); 
	//------------- VLS: Deployment to be reviewed -----------

	if (FileDlg.DoModal()==IDOK)
	{
		if (LoadRobot(FileDlg.GetPathName()))
			SetTitle(filename);

		UpdateAllViews(0);
	}
}


BOOL CWireCenterDoc::DoSave(LPCSTR lpszPathName, BOOL bReplace)
{
	OnDateiSpeichernalsxml();
	return true;
}

//! first simple implementation; just generate a generic file with the format 
//! of the .lis file "sda_mds1.lis"
void CWireCenterDoc::OnKalibrierungSteuerungsdateiexportieren()
{
	if (robotDoc.getNow()!=7)
	{
		MessageBox(0,"This feasture is only applicable to a robot with seven winches.","Error during Export",MB_OK);
		return;
	}

	CFileDialog FileDlg(FALSE,	// save File Dialog
						".lis",
						"kinematik.lis",
						OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"ISG list file (*.lis)|*.lis||");

	//------------- VLS: Deployment to be reviewed -----------
	std::string WirecenterPath;
	WirecenterPath=getWireCenterPath();
	WirecenterPath.append("\\robots");

	FileDlg.m_ofn.lpstrInitialDir  = WirecenterPath.c_str(); 
	//------------- VLS: Deployment to be reviewed -----------

	if (FileDlg.DoModal()!=IDOK)
		return;

	ofstream file(FileDlg.GetPathName().GetString());

	// there are 4 placeholders; ID for the transformation (65), id if the parameter, the value, and an explaination string
	char* formatstring = "kinematik[%i].wz_kopf_versatz[%i]\t%i %s\n";
	char buffer[1024];
	int i;
	int ID=66; // WEK
	int iparam = 0;

	// write the data into the output file
	for (i=0; i<robotDoc.getNow(); i++)
	{
		sprintf_s(buffer,formatstring,ID,iparam++,(int)(robotDoc.getBase(i).x() * 1e7),"");
		file << buffer;
		sprintf_s(buffer,formatstring,ID,iparam++,(int)(robotDoc.getBase(i).y() * 1e7),"");
		file << buffer;
		sprintf_s(buffer,formatstring,ID,iparam++,(int)(robotDoc.getBase(i).z() * 1e7),"");
		file << buffer;
		sprintf_s(buffer,formatstring,ID,iparam++,(int)(robotDoc.getPlatform(i).x() * 1e7),"");
		file << buffer;
		sprintf_s(buffer,formatstring,ID,iparam++,(int)(robotDoc.getPlatform(i).y() * 1e7),"");
		file << buffer;
		sprintf_s(buffer,formatstring,ID,iparam++,(int)(robotDoc.getPlatform(i).z() * 1e7),"");
		file << buffer;
	}
}

//! load the geometry setting from a "liste" as defined by ISG for the NC-kernel
//! \todo Refactor this import method into a library
void CWireCenterDoc::OnKalibrierungSteuerungsdateiimportieren()
{
	if (robotDoc.getNow()!=8)
	{
		MessageBox(0,"This feature is only available for robots with eight winches","Error in Import",MB_OK);
		return;
	}

	CFileDialog FileDlg(TRUE,	// load File Dialog
						".lis",
						"sda_mds1.lis",
						OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"ISG list file (*.lis)|*.lis||");
	
	//------------- VLS: Deployment to be reviewed -----------
	std::string WirecenterPath;
	WirecenterPath=getWireCenterPath();
	WirecenterPath.append("\\robots");

	FileDlg.m_ofn.lpstrInitialDir  = WirecenterPath.c_str(); 
	//------------- VLS: Deployment to be reviewed -----------

	if (FileDlg.DoModal()!=IDOK)
		return;

	KalibrierungSteuerungsdateiimportieren(FileDlg.GetPathName().GetString());
}

bool CWireCenterDoc::loadProject(const string& filename)
{
	CWireCenterProjectfile PF(*this,*CWireCenterView::This);
	return PF.restore(filename);
}

bool CWireCenterDoc::saveProject(const string& filename)
{
	CWireCenterProjectfile PF(*this,*CWireCenterView::This);
	return PF.store(filename);
}

	//! load the geometry setting from a "liste" as defined by ISG for the NC-kernel
//! \todo Refactor this import method into a library
bool CWireCenterDoc::KalibrierungSteuerungsdateiimportieren(CString filename)
{

	// read the file
	// parse the exising file...
	char buffer[1024];
	map<int,double> geometry;
	ifstream src(filename);
	int ID;
	if (src)
	{
		while (!src.eof())
		{
			int iReturn =0;
			src.getline(buffer,1024);
			int iparam,param, iIndex;
			// old syntax: ISG and TwinCAT 3.1 until Build 4010
			if (sscanf_s(buffer,"kinematik[%i].wz_kopf_versatz[%i]\t%i",&ID,&iparam,&param)!=0)
				iReturn++; // do not save this value and read the next line

			// new syntax: TwinCAT 3.1 Build 4012
			if (sscanf_s(buffer,"kinematik[%i].param[%i]\t%i",&ID,&iparam,&param)!=0)
				iReturn++; // do not save this value and read the next line

			// new syntax: TwinCAT 3.1 Build 4016
			if (sscanf_s(buffer,"trafo[%i].id\t%i",&iIndex,&ID)!=0)
				iReturn++; // do not save this value and read the next line
			if (sscanf_s(buffer,"trafo[%i].param[%i]\t%i",&iIndex,&iparam,&param)!=0)
				iReturn++; // do not save this value and read the next line

			if (iReturn >0)
			{
				if (ID==65)
				{
					if (iparam>=0 && iparam <72)
						geometry[iparam] = param / 1.0e7;
				}
				if (ID==66)
				{
					if (iparam>=0 && iparam <72)
						geometry[iparam] = param / 1.0e7;
				}
			}
		}
	}
	else
	{
		return FALSE;
	}

	for (int i=0; i<8*6; i++)
		if (geometry.find(i)==geometry.end())
		{
			CString msg;
			msg.Format("Configuration file incomplete. No entry for PARAM=%i. Import is canceled.",i);
			MessageBox(0,msg,"Error during Import",MB_OK);
			return FALSE;
		}

	cout << "Setting the geometric parameteters for kin-ID " << ID << "\n";
	for (int i=0; i<8; i++)
	{
		robotDoc.setLeg(i,Vector3d(geometry[6*i],geometry[6*i+1],geometry[6*i+2]),Vector3d(geometry[6*i+3],geometry[6*i+4],geometry[6*i+5]));
		cout << i << ": P=" << robotDoc.getPlatform(i) << " B=" << robotDoc.getBase(i) << endl;
	}
	return TRUE;
}

/*! the function can only be used as a template implementation; the transformation is
 *  specific to the IPAnema 1 frame used around 2009. since this frame is no longer in
 *  use, the function has only academic value.
 */
void CWireCenterDoc::OnTransformierenMesspunktezurahmenpunkte()
{
	if (robotDoc.getNow()!=7)
	{
		MessageBox(0,"This feature is only applicable for robots with seven winches. The predefined offsets can only be used for a special robot configuration!","Error during Import",MB_OK);
		return;
	}

	// source: Documentation by Jing Jin 20.06.2009
	double radius_rolle=0.025;				// Radius der Umlenkrolle
	double a = radius_rolle*sin(MO_PI/4);	// Offset in x Richtung
	double a2 = radius_rolle;				// voller offset in einer richtung
	double b = radius_rolle*cos(MO_PI/4);	// Offset in y Richtung
	double c = - 0.017678;					// Versatz in z-richtung aus Reflektor, Halterung, Rollenradius, Seildurchmesser, Dicke der Hülse und Dicke des Driftnests

	// Umrechnen der Rahmenpunkte laut Tabelle von Jing
	robotDoc.setBase(0, robotDoc.getBase(0) + Vector3d( a, -b, -c));
	robotDoc.setBase(1, robotDoc.getBase(1) + Vector3d(-a, -b, -c));
	robotDoc.setBase(2, robotDoc.getBase(2) + Vector3d(-a,  b, -c));
	robotDoc.setBase(3, robotDoc.getBase(3) + Vector3d( a,  b, -c));
	robotDoc.setBase(4, robotDoc.getBase(4) + Vector3d( a2, 0, -c));
	robotDoc.setBase(5, robotDoc.getBase(5) + Vector3d(-a, -b, -c));
	robotDoc.setBase(6, robotDoc.getBase(6) + Vector3d(-a,  b, -c));
}

void CWireCenterDoc::OnScriptEditieren()
{
	CPyScriptDlg pyDlg;
	pyDlg.m_filename=py_filename;
	pyDlg.DoModal();
}

void CWireCenterDoc::OnScriptLaden()
{
	CPythonInterface::getInstance().reset();
	// select the scripting file
	CFileDialog FileDlg(TRUE,	// load File Dialog
						".py",
						"script.py",
						OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"Python Scripts (*.py)|*.py||");
	
	//------------- VLS: Deployment to be reviewed -----------
	std::string WirecenterPath;
	WirecenterPath=getWireCenterPath();
	WirecenterPath.append("\\py");

	FileDlg.m_ofn.lpstrInitialDir  = WirecenterPath.c_str(); 
	//------------- VLS: Deployment to be reviewed -----------

	if (FileDlg.DoModal()!=IDOK)
		return;

	// initialize the python scripting interface and load the scripting functions
	// by calling the main function in the script "def main()"
	char* mainFunctionName="main";
	if (!CPythonInterface::getInstance().run(FileDlg.GetPathName().GetString(),mainFunctionName))
		AfxMessageBox("Failed to load python script. Perhaps the file could not be opened or the main() function is not implemented.",MB_OK);
	else
		// store the selected filename if the script was successfully started
		py_filename=FileDlg.GetPathName();
}

void CWireCenterDoc::OnScriptStarten()
{
	if (py_filename.GetLength()==0)
		OnScriptLaden();
	else
	{
		// if starting a script we reset the interpreter
		CPythonInterface::getInstance().reset();
		// initialize the python scripting interface and load the scripting functions
		// by convention the main function in the script is "def main()"
		char* mainFunctionName="main";
		if (!CPythonInterface::getInstance().run(py_filename.GetString(),mainFunctionName))
			AfxMessageBox("Failed to load python script. Perhaps the file could not be opened or the main() function is not implemented.",MB_OK);
	}
}

void CWireCenterDoc::OnArbeitsraumAlleorientierungenpr32824()
{ robotDoc.WSHull.setOrientationRequirement(!robotDoc.WSHull.getOrientationRequirement()); }

void CWireCenterDoc::OnArbeitsraumOrientierungfestlegen()
{
	// create a list of local variables for the function to be called
	double a=0,b=0,c=0;
	// create a generic parameter dialog object and add all parameters to the mapping
	CEnhancedDialog GPD("Define Orientation");
	GPD.addItem("Rotation around x-axis [degree]",a);
	GPD.addItem("Rotation around y-axis [degree]",b);
	GPD.addItem("Rotation around z-axis [degree]",c);

	// start the modal dialog box
	if (GPD.DoModal()==IDOK)
	{	
		Matrix3d R=Matrix3d::ZRotationMatrix3d(c*DEG_TO_RAD)*Matrix3d::YRotationMatrix3d(b*DEG_TO_RAD)*Matrix3d::XRotationMatrix3d(a*DEG_TO_RAD);
		// call the library function with the edited parameters
		robotDoc.WSHull.setOrientation(R);
		// refresh the orientations list in the pane
		CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
		pMF->m_wndShapeListPane.RecursiveEnumeration();
	}
}

void CWireCenterDoc::OnArbeitsraumOrientierungenhinzuf32822()
{
	// create a list of local variables for the function to be called
	double a=0,b=0,c=0;
	// create a generic parameter dialog object and add all parameters to the mapping
	CEnhancedDialog GPD("Add Orientation");
	GPD.addItem("Rotation around x-axis [degree]",a);
	GPD.addItem("Rotation around y-axis [degree]",b);
	GPD.addItem("Rotation around z-axis [degree]",c);

	// start the modal dialog box
	if (GPD.DoModal()==IDOK)
	{	
		Matrix3d R=Matrix3d::ZRotationMatrix3d(c*DEG_TO_RAD)*Matrix3d::YRotationMatrix3d(b*DEG_TO_RAD)*Matrix3d::XRotationMatrix3d(a*DEG_TO_RAD);
		// call the library function with the edited parameters
		robotDoc.WSHull.addOrientation(R);
		// refresh the orientations list in the pane
		CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
		pMF->m_wndShapeListPane.RecursiveEnumeration();
	}
}

void CWireCenterDoc::OnArbeitsraumOrientierungen()
{
	// create a list of local variables for the function to be called
	double a=0,b=0,c=0;
	double delta_a=0;
	double delta_b=0;
	double delta_c=0;
	int steps=10;
	// create a generic parameter dialog object and add all parameters to the mapping
	CEnhancedDialog GPD("Add orientation spectrum hinzufügen",300);
	GPD.StartGroupBox("Initial orientation (center):");
		GPD.addItem("Rotation around x-axis [degree]",a);
		GPD.addItem("Rotation around y-axis [degree]",b);
		GPD.addItem("Rotation around z-axis [degree]",c);
	GPD.EndGroupBox();
	GPD.StartGroupBox("Spectrum and discretisation:");
		GPD.addItem("Number of steps:",steps);
		GPD.addItem("Rotation around x-axis [degree]",delta_a);
		GPD.addItem("Rotation around y-axis [degree]",delta_b);
		GPD.addItem("Rotation around z-axis [degree]",delta_c);
	GPD.EndGroupBox();

	if (GPD.DoModal()==IDOK)
	{	
		Matrix3d R=Matrix3d::ZRotationMatrix3d(c*DEG_TO_RAD)*Matrix3d::YRotationMatrix3d(b*DEG_TO_RAD)*Matrix3d::XRotationMatrix3d(a*DEG_TO_RAD);
		robotDoc.WSHull.createOrientationSet(delta_a*DEG_TO_RAD,delta_b*DEG_TO_RAD,delta_c*DEG_TO_RAD,steps,R);	
		// refresh the orientations list in the pane
		CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
		pMF->m_wndShapeListPane.RecursiveEnumeration();
	}
}

//! set the minimum and maximum cable length for the robot
void CWireCenterDoc::OnBearbeitenSeillaengen()
{
	// create a generic parameter dialog object and add all parameters to the mapping
	CEnhancedDialog GPD("Set cable length");
	GPD.StartGroupBox("Cable length");
		GPD.addItem("Minimum cable length:",robotDoc.lmin);
		GPD.addItem("Maximum cable length:",robotDoc.lmax);
	GPD.EndGroupBox();
	GPD.DoModal();
}

/*! change the function name; it is rather confusion  since the implemtnation
 *  is "save project as"
 */
void CWireCenterDoc::OnDateiSpeichernalsxml()
{
	CFileDialog FileDlg(FALSE,	// save File Dialog
						".wcpf",
						"robot.wcpf",
						OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"WireCenter Project File (*.wcpf)|*.wcpf||");
	
	//------------- VLS: Deployment to be reviewed -----------
	std::string WirecenterPath;
	WirecenterPath=getWireCenterPath();
	WirecenterPath.append("\\robots");

	FileDlg.m_ofn.lpstrInitialDir  = WirecenterPath.c_str(); 
	//------------- VLS: Deployment to be reviewed -----------
	
	if (FileDlg.DoModal()==IDOK)
	{
		saveProject(FileDlg.GetPathName().GetBuffer());
	}
}

void CWireCenterDoc::OnEntwurfAnwendungbeschreiben()
{
	//use new dialog box for editing the robot application settings
	//with improved methods for datainput

	//create dialog
	CDescribeApplicationDlg DLG_DescribeApplicationDlg;

#ifdef WIRECENTER_LIGHTVERSION
	// reduce the available options in the light version
	DLG_DescribeApplicationDlg.m_bSimplifiedDialog=TRUE;
#endif
	
	//LOAD current values into the dialog box variables
	DLG_DescribeApplicationDlg.getRequirements(robotDoc.AppReq);

	//show modal dialog and wait for result...
	if (DLG_DescribeApplicationDlg.DoModal()==IDOK)
	{
		//if OK clicked, then SAVE values into global variables of the robotDoc object
		DLG_DescribeApplicationDlg.setRequirements(robotDoc.AppReq);
	};
}

void CWireCenterDoc::OnEntwurfWindenparameter()
{
	//use new dialog box for editing the winch parameter settings

	//create dialog
	CEditDrumParametersDlg DLG_EditDrumParameters;

	//LOAD current values into the dialog box variables
	if (robotDoc.currentWinch)
		DLG_EditDrumParameters.getWinchSettings(*robotDoc.currentWinch);
	else
		return;		// error printen; currentWinch ist NULL

	//show modal dialog and wait for result...
	if (DLG_EditDrumParameters.DoModal()==IDOK)
	{
		//if OK clicked, then SAVE values into global variables of the robotDoc object
		DLG_EditDrumParameters.setWinchSettings(*robotDoc.currentWinch);
	};

}

void CWireCenterDoc::OnEntwurfSeillaengeFuerArbeitsraum()
{ robotDoc.Kinematics.getWireRangeForBoxDriver(robotDoc.AppReq.minWS,robotDoc.AppReq.maxWS); }

void CWireCenterDoc::OnEntwurfSeilkraefteFuerArbeitsraum()
{ robotDoc.WSGrid.wireForcesWorkspaceBoxDriver(robotDoc.AppReq.minWS,robotDoc.AppReq.maxWS); }

void CWireCenterDoc::OnEntwurfSeilwinkelFuerArbeitsraum()
{
	for (int i=0; i<robotDoc.getNow(); i++)
	{
		Vector3d axis;
		double aperture;
		robotDoc.WSHull.calculateOptimalCone(i,axis, aperture);
		cout << i << ": axis=" << axis << " aperture= " << aperture << endl;
	}
}

void CWireCenterDoc::OnEntwurfAnforderungenladen()
{
	CFileDialog FileDlg(TRUE,	// open File Dialog
						".wcrqx",
						"requirements.wcrqx",
						OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"Wire Center Requirements Extensible Markup Language (*.wcrqx)|*.wcrqx||");
	
	//------------- VLS: Deployment to be reviewed -----------
	std::string WirecenterPath;
	WirecenterPath=getWireCenterPath();
	WirecenterPath.append("\\robots\\requirements");

	FileDlg.m_ofn.lpstrInitialDir  = WirecenterPath.c_str(); 
	//------------- VLS: Deployment to be reviewed -----------
	
	if (FileDlg.DoModal()==IDOK)
		robotDoc.AppReq.loadXML(FileDlg.GetPathName().GetString());
}

void CWireCenterDoc::OnEntwurfAnwendungsanforderungenspeichern()
{
	CFileDialog FileDlg(FALSE,	// save File Dialog
						".wcrqx",
						"requirements.wcrqx",
						OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"Wire Center Requirements Extensible Markup Language (*.wcrqx)|*.wcrqx||");
	
	//------------- VLS: Deployment to be reviewed -----------
	std::string WirecenterPath;
	WirecenterPath=getWireCenterPath();
	WirecenterPath.append("\\robots\\requirements");

	FileDlg.m_ofn.lpstrInitialDir  = WirecenterPath.c_str(); 
	//------------- VLS: Deployment to be reviewed -----------
	
	if (FileDlg.DoModal()==IDOK)
		robotDoc.AppReq.saveXML(FileDlg.GetPathName().GetString());
}

void CWireCenterDoc::OnEntwurfKontrolierbarkeitfuerarbeitsraum()
{
	if (robotDoc.WSGrid.verifyWorkspaceBox(robotDoc.AppReq.minWS,robotDoc.AppReq.maxWS))
		printf("Workspcae is fully covered by the grid. Orientation have not been taken into account.\n");
	else
		printf("Workspace is not fully covered by the grid.\n");
}

void CWireCenterDoc::OnArbeitsraumKollisionberechnen()
{
	// the control parameters of CInterference are not public; we have to copy them before we can set them
	double xmin, xmax, ymin, ymax, zmin, zmax; 
	double alpha_min,alpha_max,delta_alpha,beta_min,beta_max,delta_beta;
	double gamma_min=0,gamma_max=0,delta_gamma=0;
	robotDoc.Interference.getClippingBox(xmin, xmax, ymin, ymax, zmin, zmax);
	robotDoc.Interference.getOrientationWorkspace(alpha_min,alpha_max,delta_alpha,beta_min,beta_max,delta_beta);

	// create a generic parameter dialog object and add all parameters to the mapping
	CEnhancedDialog GPD("Compute Cable-Cable Interference ",300);
	GPD.StartGroupBox("Angular Spectrum");
		GPD.addItem("Alpha_min [rad]:",alpha_min);		
		GPD.addItem("Alpha_max [rad]:",alpha_max);		
		GPD.addItem("Step width delta_alpha [rad]:",delta_alpha);		
		GPD.addItem("Beta_min [rad]:",beta_min);		
		GPD.addItem("Beta_max [rad]:",beta_max);		
		GPD.addItem("Step width delta_beta [rad]:",delta_beta);		
		GPD.addItem("Gamma_min [rad]:",gamma_min);		
		GPD.addItem("Gamma_max [rad]:",gamma_max);		
		GPD.addItem("Step width delta_gamma [rad]:",delta_gamma);		
	GPD.EndGroupBox();
	GPD.NextDialogColumn();
	GPD.StartGroupBox("Bounding Box for Clipping (openGL)");
		GPD.addItem("Clipping Xmin [m]:",xmin);		
		GPD.addItem("Clipping Xmax [m]:",xmax);		
		GPD.addItem("Clipping Ymin [m]:",ymin);		
		GPD.addItem("Clipping Ymax [m]:",ymax);		
		GPD.addItem("Clipping Zmin [m]:",zmin);		
		GPD.addItem("Clipping Zmax [m]:",zmax);		
	GPD.EndGroupBox();
	// WARNING: Remove the following two lines after testing
	static bool newImplementation=false;	// make the last choice persistant
	GPD.addItem("Use experimental implementation",newImplementation);

	if (GPD.DoModal()==IDOK)
	{
		// update the parameters in the object
		robotDoc.Interference.setClippingBox(xmin, xmax, ymin, ymax, zmin, zmax);
		robotDoc.Interference.setOrientationWorkspace(alpha_min,alpha_max,delta_alpha,beta_min,beta_max,delta_beta,gamma_min,gamma_max,delta_gamma);
		if (newImplementation)
			robotDoc.Interference.calculateCollisionsOrientationWorkspace();	// WARNING: Remove this call after revision as it is experimental!!!

	}
}

void CWireCenterDoc::OnArbeitsraumKonturgitter()
{
	// create a list of local variables for the function to be called
	int steps = 10;
	bool bcalculate_x = true;
	bool bcalculate_y = false;
	bool bcalculate_z = false;
	
	// create a generic parameter dialog object and add all parameters to the mapping
	CEnhancedDialog GPD("Settings for Contour Grid");
	GPD.addItem("Steps per diretion:",steps);
	GPD.StartGroupBox("Active Directions");
	GPD.addItem("Include x-direction",bcalculate_x);
	GPD.addItem("Include y-direction",bcalculate_y);
	GPD.addItem("Include z-direction",bcalculate_z);
	GPD.EndGroupBox();

	// start the modal dialog box
	if (GPD.DoModal()==IDOK)
	{	
		// call the library function with the edited parameters
		// use region of interest or determine the block from the frame depending on the current mode of bROI
		Vector3d minBB = Roi.lower();
		Vector3d maxBB = Roi.upper();
		if (!CWireCenterView::This->m_bAutoRoiUse)
			robotDoc.getBoundingBoxBase(minBB,maxBB);
		
		int test = (bcalculate_z<<2)+(bcalculate_y<<1)+(bcalculate_x);
		if (test<8) robotDoc.Crosssection.axis=test;
		robotDoc.Crosssection.calculateWorkspaceCrosssectionGrid(minBB,maxBB,steps);
		UpdateAllViews(0);
	}
}

void CWireCenterDoc::OnArbeitsraumGridCylinder()
{
	// create a list of local variables for the function to be called
	int steps = PERSIST(10,"WorkspaceCylinderGridSteps","defaultValues/@WorkspaceCylinderGridSteps");
	int angSteps = PERSIST(36,"WorkspaceCylinderGridangularSteps","defaultValues/@WorkspaceCylinderGridangularSteps");
	int layers = PERSIST(10,"WorkspaceCylinderGridLayers","defaultValues/@WorkspaceCylinderGridLayers");
	bool bROI = false;
	Vector3d minBB = Roi.lower();
	Vector3d maxBB = Roi.upper();
	
	// create a generic parameter dialog object and add all parameters to the mapping
	CEnhancedDialog GPD("Cylindric grid properties");
	GPD.addItem("Radial steps",steps);
	GPD.addItem("Polar steps",angSteps);
	GPD.addItem("Layers",layers);
	GPD.addItem("Use Region of Interest",bROI);

	// start the modal dialog box
	if (GPD.DoModal()==IDOK)
	{	
		// call the library function with the edited parameters
		Vector3d minBB,maxBB;
		if (!bROI)
			robotDoc.getBoundingBoxBase(minBB,maxBB);
		robotDoc.WSGrid.makeCylinderGrid(minBB,maxBB,'z',layers,steps,angSteps);
		robotDoc.WSGrid.calculateWorkspace();
		UpdateAllViews(0);
	}
}

void CWireCenterDoc::OnArbeitsraumGridRandom()
{
	// create a list of local variables for the function to be called
	int samples = PERSIST(1000,"RandomGridSamples","defaultValues/@RandomGridSamples");
	bool bROI = false;
	Vector3d minBB = Roi.lower();
	Vector3d maxBB = Roi.upper();
	
	// create a generic parameter dialog object and add all parameters to the mapping
	CEnhancedDialog GPD("Random Grid Properties");
	GPD.addItem("Number of Position Samples",samples);
	GPD.addItem("Use Region of Interest",bROI);

	// start the modal dialog box
	if (GPD.DoModal()==IDOK)
	{	
		// call the library function with the edited parameters
		Vector3d minBB,maxBB;
		if (!bROI)
			robotDoc.getBoundingBoxBase(minBB,maxBB);
		robotDoc.WSGrid.makeRandomGrid(minBB,maxBB,samples);
		robotDoc.WSGrid.calculateWorkspace();
		UpdateAllViews(0);
	}}

void CWireCenterDoc::OnOrientationOptions()
{
	// open the orientation dialog
	COrientationDlg OD;
	OD.m_OrientationCriterion = robotDoc.WSHull.getOrientationRequirement()?1:0;
	OD.m_OrientationInfoSource = CWireCenterView::This->m_bAutoApplyOrientation?1:0;
	OD.m_pWA = &robotDoc.WSHull;
	if (OD.DoModal()==IDOK)
	{
		// write back the states selection in the dialog box
		robotDoc.WSHull.setOrientationRequirement(OD.m_OrientationCriterion==1?true:false);
		CWireCenterView::This->m_bAutoApplyOrientation = OD.m_OrientationInfoSource == 1?true:false;
		UpdateAllViews(0);
		return;
	}
}

//! \todo Seperate the load xml functions from the GUI elements. The implementation of the loading
//! function should be placed in the library.
void CWireCenterDoc::OnEntwurfWindenparameterladen()
{
	CString OpenFilter;
		OpenFilter = "Wire Center WinchDatabase Extensible Markup Language (*.wcwdqx)|*.wcwdqx|";
		OpenFilter += "All Files (*.*)|*.*||";
	
	CFileDialog FileDlg(TRUE,	// open File Dialog
						".wcwdqx",
						"Winch.wcwdqx",
						OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						OpenFilter);

	//------------- VLS: Deployment to be reviewed -----------
	std::string WirecenterPath;
	WirecenterPath=getWireCenterPath();
	WirecenterPath.append("\\robots\\winches");

	FileDlg.m_ofn.lpstrInitialDir  = WirecenterPath.c_str(); 
	//------------- VLS: Deployment to be reviewed -----------
	
	if (FileDlg.DoModal()==IDOK)
	{
		TiXmlDocument doc(FileDlg.GetPathName().GetString());
		if (!doc.LoadFile())
		{
			cout<<"Problem Loading XML File in Line: "<<doc.ErrorRow()<<" Column: "<<doc.ErrorCol()<<endl;
			return; // exit function don't load anything
		}
		
		// parse the XML document
		TiXmlElement *root = doc.RootElement();
		
		// check if root element is <models> !!!! 
		if (strcmp(root->Value(),"models"))
		{
			cout << "ERROR: root node in XML file is not <models>\n"<<endl;
			return;
		}
		
		int i=0;
		for (TiXmlNode* winchcount = root->FirstChild("winch"); winchcount; winchcount = winchcount->NextSibling("winch")) //Test for more than one <winch>-tag the following if statement then only reads winch data if only one exist
			i++;
		
		if (i>0)
		{
			int entireDatabase = AfxMessageBox("Load Entire Database?", MB_YESNO|MB_ICONQUESTION);
			if (entireDatabase==IDYES)
			{
				CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
				CMFCRibbonComboBox* pEdit = DYNAMIC_DOWNCAST(CMFCRibbonComboBox, pMF->m_wndRibbonBar.FindByID(IDC_WINCH_SELECTOR_COMBO));
				pEdit->RemoveAllItems(); //must keep vector in combo box equal to robotDoc.WinchDB
					
				robotDoc.WinchDB.resize(1);
				robotDoc.currentWinch = robotDoc.WinchDB.at(0); 
				robotDoc.currentWinch->loadXML(root, 1);
				
				pEdit->AddItem(robotDoc.currentWinch->name.c_str());
				pEdit->SelectItem(0);
				for(int j=1; j<i ;j++)
				{
					robotDoc.AddWinch();
					robotDoc.WinchDB.back()->loadXML(root, (j+1));
					pEdit->AddItem(robotDoc.WinchDB.back()->name.c_str());
				}
			}
			if (entireDatabase==IDNO) //create a new Database
			{
				CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
				CMFCRibbonComboBox* pEdit = DYNAMIC_DOWNCAST(CMFCRibbonComboBox, pMF->m_wndRibbonBar.FindByID(IDC_WINCH_SELECTOR_COMBO));
					
				/*-----Here we need user input-----*/
				cout<<"I counted "<<i<<" winches"<<endl;
				int iRand = (rand() % (i)) + 1;
				cout<<"Using winch "<<iRand<<"..."<<endl;
				/*---------------------------------*/
			
				TiXmlNode *winch = root->FirstChild("winch");

				for(int i=0; i<iRand-1;i++)
					winch = winch->NextSibling("winch");
				
				bool winchcheck=false;
				//check for existing Winch with the same name
				for(unsigned int j=0; j<robotDoc.WinchDB.size();j++)
				{	
					if (robotDoc.WinchDB.at(j)->name==winch->ToElement()->Attribute("name"))
						winchcheck=true;
				}
				if (winchcheck==true)
					cout<<"ERROR: Winch with same name found, will not load winch!"<<endl;
				else
				{
					robotDoc.AddWinch();
					robotDoc.WinchDB.back()->loadXML(root,iRand);
					pEdit->AddItem(robotDoc.WinchDB.back()->name.c_str()); //VLS: This line should not be needed for proper implementation
				}
			}
		}
		else
			cout<<"No Winch Found!"<<endl;
	}
}


void CWireCenterDoc::OnEntwurfWindenparameterspeichern()
{
	cout<<"Saving Current Winch..."<<endl; //TODO: THIS ONLY SAYES THE CURRENT WINCH! carefuls
	//int newDatabase = AfxMessageBox("Create a new Database?", MB_YESNO|MB_ICONQUESTION); //prompt user 
	
	CString OpenFilter;
		OpenFilter = "Wire Center Winch Database Extensible Markup Language (*.wcwdqx)|*.wcwdqx|";
		OpenFilter += "All Files (*.*)|*.*||";
	CFileDialog FileDlg(FALSE,	// save File Dialog
				".wcwdqx",
				"Winden.wcwdqx",
				OFN_HIDEREADONLY,
				OpenFilter);
	
	//------------- VLS: Deployment to be reviewed -----------
	std::string WirecenterPath;
	WirecenterPath=getWireCenterPath();
	WirecenterPath.append("\\robots\\winches");

	FileDlg.m_ofn.lpstrInitialDir  = WirecenterPath.c_str(); 
	//------------- VLS: Deployment to be reviewed -----------
	
	
	if (FileDlg.DoModal()==IDOK) //Success select File
	{
		robotDoc.currentWinch->saveXML(FileDlg.GetPathName().GetString());
	}
}

void CWireCenterDoc::OnEntwurfSeilparameter()
{
	//use new dialog box for editing the wire settings

	//create dialog
	CEditWireParametersDlg DLG_EditWireParametersDlg;
	
	//LOAD current values into the dialog box variables
	if (robotDoc.currentCable)
		DLG_EditWireParametersDlg.getWireSettings(*robotDoc.currentCable);
	else
		return;		// error printen; currentWinch ist NULL

	//show modal dialog and wait for result...
	if (DLG_EditWireParametersDlg.DoModal()==IDOK)
	{
		//if OK clicked, then SAVE values into global variables of the robotDoc object
		DLG_EditWireParametersDlg.setWireSettings(*robotDoc.currentCable);
	};

}

void CWireCenterDoc::OnEntwurfSeilparameterladen()
{
	CString OpenFilter;
	OpenFilter = "Wire Center CableDatabase Extensible Markup Language (*.wccdqx)|*.wccdqx|";
	OpenFilter += "All Files (*.*)|*.*||";
	
	CFileDialog FileDlg(TRUE,	// open File Dialog
						".wccdqx",
						"Cables.wccdqx",
						OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						OpenFilter);
	
	//------------- VLS: Deployment to be reviewed -----------
	std::string WirecenterPath;
	WirecenterPath=getWireCenterPath();
	WirecenterPath.append("\\robots\\cables");

	FileDlg.m_ofn.lpstrInitialDir  = WirecenterPath.c_str(); 
	//------------- VLS: Deployment to be reviewed -----------
	
	if (FileDlg.DoModal()==IDOK)
	{
	TiXmlDocument doc(FileDlg.GetPathName().GetString());
	if (!doc.LoadFile())
	{
		cout<<"No XML structure found"<<endl;
		return; // exit function don't load anything
	}

	// parse the XML document
	TiXmlElement *root = doc.RootElement();

	// check if root element is <models> !!!! 
	if (strcmp(root->Value(),"models"))
	{
		cout << "ERROR: root node in XML file is not <models>\n"<<endl;
		return;
	}
	
	int i=0;
	for (TiXmlNode* cablecount = root->FirstChild("cable"); cablecount; cablecount = cablecount->NextSibling("cable")) //Test for more than one <winch>-tag the following if statement then only reads winch data if only one exist
		i++;
		if (i>0)
		{
			int entireDatabase = AfxMessageBox("Load Entire Database?", MB_YESNO|MB_ICONQUESTION);
			if (entireDatabase==IDYES)
			{
				//CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
				//CMFCRibbonComboBox* pEdit = DYNAMIC_DOWNCAST(CMFCRibbonComboBox, pMF->m_wndRibbonBar.FindByID(IDC_WINCH_SELECTOR_COMBO)); No Cable Selector
				//pEdit->RemoveAllItems();
					
				robotDoc.CableDB.resize(1);
				robotDoc.currentCable = robotDoc.CableDB.at(0); 
				robotDoc.currentCable->loadXML(root, 1);
				
				//pEdit->AddItem(robotDoc.currentWinch->name.c_str());
				//pEdit->SelectItem(0); No Cable Selector
				for(int j=1; j<i ;j++)
				{
					robotDoc.AddCable();
					robotDoc.CableDB.back()->loadXML(root, (j+1));
					//pEdit->AddItem(robotDoc.WinchDB.back()->name.c_str()); No Cable Selector
				} 
				
				/*-----Here we need user input-----*/ //Cable Selector
				cout<<"I counted "<<i<<" cables"<<endl;
				
				int iRand = (rand() % (i)) + 1;
				
				cout<<"Using cable "<<iRand<<"..."<<endl;
				/*---------------------------------*/
				
				robotDoc.currentCable = robotDoc.CableDB.at(iRand);
			}
			if (entireDatabase==IDNO) //create a new Database
			{
				//CMainFrame* pMF = (CMainFrame*)AfxGetMainWnd();
				//CMFCRibbonComboBox* pEdit = DYNAMIC_DOWNCAST(CMFCRibbonComboBox, pMF->m_wndRibbonBar.FindByID(IDC_WINCH_SELECTOR_COMBO)); No Cable Selector
					
				/*-----Here we need user input-----*/
				cout<<"I counted "<<i<<" cables"<<endl;
				
				int iRand = (rand() % (i)) + 1;
				
				cout<<"Using cable "<<iRand<<"..."<<endl;
				/*---------------------------------*/
				
				robotDoc.currentCable->loadXML(root, iRand);

				TiXmlNode *cable = root->FirstChild("cable"); //ignore possible duplicates	
			}
		}
		else
			cout<<"No Cable Found!"<<endl;
	}
}

void CWireCenterDoc::OnEntwurfSeilparameterspeichern()
{
		//int newDatabase = AfxMessageBox("Create a new Database?", MB_YESNO|MB_ICONQUESTION); //prompt user 
	CString OpenFilter;
		OpenFilter = "Wire Center CableDatabase Extensible Markup Language (*.wccdqx)|*.wccdqx|";
		OpenFilter += "All Files (*.*)|*.*||";
	//if(newDatabase==IDYES) //create a new Database
	//{
	CFileDialog FileDlg(FALSE,	// save File Dialog
				".wccdqx",
				"Cables.wccdqx",
				OFN_HIDEREADONLY,
				OpenFilter);
	
	//------------- VLS: Deployment to be reviewed -----------
	std::string WirecenterPath;
	WirecenterPath=getWireCenterPath();
	WirecenterPath.append("\\robots\\cables");

	FileDlg.m_ofn.lpstrInitialDir  = WirecenterPath.c_str(); 
	//------------- VLS: Deployment to be reviewed -----------

	int entireDatabase = AfxMessageBox("Save Entire Database?", MB_YESNO|MB_ICONQUESTION);
	if (entireDatabase == IDYES)
	{
		if (FileDlg.DoModal()==IDOK)
		{
			for(unsigned int i=0; i<robotDoc.CableDB.size(); i++)
				robotDoc.CableDB.at(i)->saveXML(FileDlg.GetPathName().GetString()); //!< Warning: WireLib checks and overwrites cables with the same name!
		}
	}
	else
	{
		if (FileDlg.DoModal()==IDOK) //Success select File
		{
			robotDoc.currentCable->saveXML(FileDlg.GetPathName().GetString());
		}
	}
}

//! start the predefined signated wirecenter script.
//! \todo This function needs refactoring; the implementation
//! shall be moved e.g. to CPythonInterface.
void CWireCenterDoc::OnApplicationFeasibilitycheck()
{
	string filename="feasibilitytest.wcss";

	// load the encrypted script from the disk
	string myScript;
	aes256_decryptFromFile(filename.c_str(),myScript);

	// add a call to the main() of the script as last line
	myScript+="\n\nmain()\n";
	// there seems to be a problem with the encoding in strings; therefore we exchange
	// all linebrakes characters "13" by "32" (normal space)
	for (unsigned int i=0; i<myScript.size(); i++)
		if (myScript[i]==13)
			myScript[i]=32;

	// start the python script; this is a bit low level and must be improved
	int res=PyRun_SimpleString(myScript.c_str());
	if (res)
		PyErr_Print();
}

void CWireCenterDoc::OnExportWorkspaceButton()
{
	//opens an file save dialog to export the workspace as .stl or .m (matlab) file

	CFileDialog FileDlg(FALSE, ".stl","workspace.stl",OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"Surface Tesselation Language (*.stl)|*.stl|Matlab-File (*.m)|*.m|CSV-File (*.csv)|*.csv||");
	
	//------------- VLS: Deployment to be reviewed -----------
	std::string WirecenterPath;
	WirecenterPath=getWireCenterPath();
	WirecenterPath.append("\\robots\\workspaces");

	FileDlg.m_ofn.lpstrInitialDir  = WirecenterPath.c_str(); 
	//------------- VLS: Deployment to be reviewed -----------


	if (FileDlg.DoModal()!=IDOK)
		return;

	////if .stl file output is selected...
	if (FileDlg.GetFileExt()=="stl")
	{
		robotDoc.WSHull.saveWorkspace(FileDlg.GetPathName().GetString());
	}
	//if matlab file output is selected...
	else if (FileDlg.GetFileExt()=="m")
	{
		robotDoc.WSHull.saveWorkspaceMatlab(FileDlg.GetPathName().GetString());
	}
	//if csv output is selected the vertices were exported...
	else if (FileDlg.GetFileExt()=="csv")
	{
		robotDoc.WSHull.saveVerticesCSV(FileDlg.GetPathName().GetString());
	}

	else return;

}

void CWireCenterDoc::OnExportCrosssectionButton()
{
	//opens an file save dialog to export the workspace cross section as .svg or .m (matlab) file

	CFileDialog FileDlg(FALSE, ".svg","workspace.svg",OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"Scalable Vector Graphics (*.svg)|*.svg|Matlab-File (*.m)|*.m||");
	
	//------------- VLS: Deployment to be reviewed -----------
	std::string WirecenterPath;
	WirecenterPath=getWireCenterPath();
	WirecenterPath.append("\\robots\\workspaces");

	FileDlg.m_ofn.lpstrInitialDir  = WirecenterPath.c_str(); 
	//------------- VLS: Deployment to be reviewed -----------

	if (FileDlg.DoModal()!=IDOK)
		return;

	//if scaleable vector file output is selected...
	if (FileDlg.GetFileExt()=="svg")
	{
		robotDoc.Crosssection.saveWorkspaceCrosssection(FileDlg.GetPathName().GetString());
	}
	//if matlab file output is selected..
	else if (FileDlg.GetFileExt()=="m")
	{
		robotDoc.Crosssection.saveWorkspaceCrosssectionMatlab(FileDlg.GetPathName().GetString());
	}
	else return;
}

void CWireCenterDoc::OnAutogenRoiButton()
{
	robotDoc.getBoundingBoxBase(Roi.lower(), Roi.upper());
}


/*! Build a dialog box to edit the entries of the Roi member variables
 */
void CWireCenterDoc::OnRoiEditButton()
{
	// create a generic parameter dialog object and add all parameters to the mapping
	CEnhancedDialog GPD("Add Orientation");
	GPD.addItem("lower vertex of the axis aligned bounding box in x-axis [m]:",Roi.lower().x());
	GPD.addItem("lower vertex of the axis aligned bounding box in y-axis [m]:",Roi.lower().y());
	GPD.addItem("lower vertex of the axis aligned bounding box in z-axis [m]:",Roi.lower().z());
	GPD.addItem("upper vertex of the axis aligned bounding box in x-axis [m]:",Roi.upper().x());
	GPD.addItem("upper vertex of the axis aligned bounding box in y-axis [m]:",Roi.upper().y());
	GPD.addItem("upper vertex of the axis aligned bounding box in z-axis [m]:",Roi.upper().z());

	// start the modal dialog box
	if (GPD.DoModal()==IDOK)
	{
		Roi.normalize();
	}
}


void CWireCenterDoc::OnRoiFromrequirementButton()
{
	Roi.lower()=robotDoc.AppReq.minWS;
	Roi.upper()=robotDoc.AppReq.maxWS;
}


void CWireCenterDoc::OnRoiFromreqInstallationButton()
{
	Roi.lower()=robotDoc.AppReq.minIS;
	Roi.upper()=robotDoc.AppReq.maxIS;
}


void CWireCenterDoc::OnPermutationButton()
{
}

void CWireCenterDoc::OnPermutePlatformXpButton()
{
	if (robotDoc.getNow()<8)
		return;
	PCRL::CGeometryPermutator perm(robotDoc);
	unsigned int seq[]= {0,2,6,3,4,1,5,7};	// cycle the positive x-axis
	perm.setPermutation(seq,8);
	perm.effectBase=false;
	perm.setGeometry();
	UpdateAllViews(0);
}


void CWireCenterDoc::OnPermutePlatformYpButton()
{
	if (robotDoc.getNow()<8)
		return;
	PCRL::CGeometryPermutator perm(robotDoc);
	unsigned int seq[]= {4,1,2,0,5,3,6,7};	// cycle the positive x-axis
	perm.setPermutation(seq,8);
	perm.effectBase=false;
	perm.setGeometry();
	UpdateAllViews(0);
}


void CWireCenterDoc::OnPermutePlatformZpButton()
{
	if (robotDoc.getNow()<4)
		return;
	PCRL::CGeometryPermutator perm(robotDoc);
	perm.cycle(0,3);
	perm.effectBase=false;
	perm.setGeometry();
	UpdateAllViews(0);
}


void CWireCenterDoc::OnPermuteBaseXpButton()
{
	if (robotDoc.getNow()<8)
		return;
	PCRL::CGeometryPermutator perm(robotDoc);
	unsigned int seq[]= {0,2,6,3,4,1,5,7};	// cycle the positive x-axis
	perm.setPermutation(seq,8);
	perm.effectPlatform=false;
	perm.setGeometry();
	UpdateAllViews(0);
}


void CWireCenterDoc::OnPermuteBaseYpButton()
{
	if (robotDoc.getNow()<8)
		return;
	PCRL::CGeometryPermutator perm(robotDoc);
	unsigned int seq[]= {4,1,2,0,5,3,6,7};	// cycle the positive x-axis
	perm.setPermutation(seq,8);
	perm.effectPlatform=false;
	perm.setGeometry();
	UpdateAllViews(0);
}


void CWireCenterDoc::OnPermuteBaseZpButton()
{
	if (robotDoc.getNow()<4)
		return;
	PCRL::CGeometryPermutator perm(robotDoc);
	perm.cycle(0,3);
	perm.effectPlatform=false;
	perm.setGeometry();
	UpdateAllViews(0);
}


void CWireCenterDoc::OnCableforceassistentButton()
{
	CCableForceDlg Dlg(robotDoc);
	Dlg.m_dFmin = robotDoc.fmin;
	Dlg.m_dFmax = robotDoc.fmax;
	if (Dlg.DoModal()==IDOK)
	{
		robotDoc.fmin = Dlg.m_dFmin;
		robotDoc.fmax = Dlg.m_dFmax;
	}
}

void CWireCenterDoc::OnSaveConfiguriatonXmlButton()
{
	CFileDialog FileDlg(FALSE,	// save File Dialog
						".xml",
						"wcConfig.xml",
						OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"XML configuration file (*.xml)|*.xml||");
	if (FileDlg.DoModal()!=IDOK)
		return;
	if (!robotDoc.saveAlgorithmConfiguration(FileDlg.GetPathName().GetString()))
		MessageBox(0,"Error while saving the XML configuration file","Error",MB_OK | MB_ICONSTOP);
}

void CWireCenterDoc::OnLoadConfiguriatonXmlButton()
{
	CFileDialog FileDlg(TRUE,	// save File Dialog
						".xml",
						"wcConfig.xml",
						OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"XML configuration file (*.xml)|*.xml||");
	if (FileDlg.DoModal()!=IDOK)
		return;
	if (!robotDoc.loadAlgorithmConfiguration(FileDlg.GetPathName().GetString()))
		MessageBox(0,"Error while loading the XML configuration file","Error",MB_OK | MB_ICONSTOP);
}


void CWireCenterDoc::OnConfigureKinematicModelButton()
{
	// create a generic parameter dialog object and add all parameters to the mapping
	CEnhancedDialog GPD("Configure Kinematics Model");

	CStringArray names_km;
	names_km.Add("Point-shaped (standard model)");
	names_km.Add("Panning Pulleys");
	int km=robotDoc.getRobotKinematicsModel();
	GPD.addItem("Kinematic model for cable guiding",km,names_km);

	CStringArray names_cm;
	names_cm.Add("perfect stiff");
	names_cm.Add("elastic");
	names_cm.Add("sagging");
	int cm=robotDoc.getElasticityModel();
	GPD.addItem("Cable model",cm,names_cm);

	GPD.addItem("max. Number of Iterations for Forward Kinematics",robotDoc.Kinematics.m_nItmax);

	// start the modal dialog box
	if (GPD.DoModal()==IDOK)
	{
		robotDoc.setRobotKinematicsModel((PCRL::CRobotData::RobotKinematicsType)km);
		robotDoc.setElasticityModel((PCRL::CRobotData::ElasticityModelType)cm);
	}
}


void CWireCenterDoc::OnBearbeitenUmlenkrollenButton()
{
	// create a generic parameter dialog object and add all parameters to the mapping
	CEnhancedDialog GPD("Configure Pulley Parameters");

	GPD.addItem("Apply Pulleys in Structure Matrix",robotDoc.ForceDistribution.usePulley);
	GPD.StartGroupBox("Radius of the pulleys");
		for (int i=0; i<robotDoc.getNow(); ++i)
		{
			CString str;
			str.Format("Radius r_%i [m]",i+1);
			GPD.addItem(str,robotDoc.r_pulley[i]);
		}
	GPD.EndGroupBox();

	// start the modal dialog box
	if (GPD.DoModal()==IDOK)
	{
		UpdateAllViews(0);
	}
}

void CWireCenterDoc::OnSaveGridButton()
{
	CFileDialog FileDlg(FALSE,	// save File Dialog
						".csv",
						"wsGrid.csv",
						OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"Comma Separated Value (*.csv)|*.csv||");
	if (FileDlg.DoModal()!=IDOK)
		return;
	if (!robotDoc.saveWSGrid(FileDlg.GetPathName().GetString()))
		MessageBox(0,"Error while saving the workspace grid","Error",IDOK);
	//cout<<"NOT IMPLEMENTED YET."<<"\n";
}


void CWireCenterDoc::OnRobotMetadataButton()
{
	// create a generic parameter dialog object and add all parameters to the mapping
	CEnhancedDialog GPD("Edit Robot Description");

	GPD.StartGroupBox("Robot");
	GPD.addItem("Name of the robot",this->robotDoc.name);
	GPD.addItem("Description",robotDoc.desc);
	GPD.addItem("Vendor",robotDoc.author);
	GPD.addItem("ID",robotDoc.id);
	GPD.EndGroupBox();

	// start the modal dialog box
	if (GPD.DoModal()==IDOK)
	{
		UpdateAllViews(0);
	}
}


void CWireCenterDoc::OnExportRobotButton()
{
	CFileDialog FileDlg(FALSE,	// save File Dialog
						".wcrfx",
						"robot.wcrfx",
						OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"Wire Center Robot File Extensible Markup Language (*.wcrfx)|*.wcrfx||");
	
	//------------- VLS: Deployment to be reviewed -----------
	std::string WirecenterPath;
	WirecenterPath=getWireCenterPath();
	WirecenterPath.append("\\robots");

	FileDlg.m_ofn.lpstrInitialDir  = WirecenterPath.c_str(); 
	//------------- VLS: Deployment to be reviewed -----------
	
	if (FileDlg.DoModal()==IDOK)
	{
		robotDoc.saveXml((FileDlg.GetPathName()).GetString());
	}
}


void CWireCenterDoc::OnCablespanCalculateButton()
{
	robotDoc.WSHull.calculateCableSpan();
	UpdateAllViews(0);
}


void CWireCenterDoc::OnShowStructurematrixButton()
{
	MatrixXd AT;
	robotDoc.ForceDistribution.getMatrix(AT);
	CString header; header.Format("Structure Matrix A^T(r) for r=[ %3.3f , %3.3f , %3.3f ]",
		robotDoc.r.x(),robotDoc.r.y(),robotDoc.r.z());
	CMatrixDialog MD(header, AT);
	MD.DoModal();
}


void CWireCenterDoc::OnShowStiffnessmatrixButton()
{
	MatrixXd C;
	C=robotDoc.Stiffness.getStiffnessMatrix();
	CMatrixDialog MD("Stiffness Matrix", C);
	MD.DoModal();
}


void CWireCenterDoc::OnInterferenceCablecablematrixButton()
{
	MatrixXd C;
	robotDoc.Interference.calculateCableCableCollisions(robotDoc.r,robotDoc.R);
	robotDoc.Interference.getCableCableCollisions(C);
	CMatrixDialog MD("Cable-Cable Interference Matrix", C);
	MD.DoModal();
}


void CWireCenterDoc::OnInterferenceCableplatformmatrixButton()
{
	MatrixXd C;
	robotDoc.Interference.calculateCablePlatformCollisions(robotDoc.r,robotDoc.R);
	robotDoc.Interference.getCablePlatformCollisions(C);
	CMatrixDialog MD("Cable-Platform Interference Matrix", C);
	MD.DoModal();
}


void CWireCenterDoc::OnShowDifferentialhullParameters()
{
	CMatrixDialog MD("Differential Hull: Differential Parameters", robotDoc.WSHull.getParameterDifferentials());
	MD.DoModal();
}


void CWireCenterDoc::OnBearbeitenGeometry()
{
	MatrixXd AB;
	robotDoc.getLegMatrix(AB);
	CMatrixDialog MD("Inspect Robot Geometry",AB);
	MD.DoModal();
}
