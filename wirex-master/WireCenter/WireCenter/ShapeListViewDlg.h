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

/*! \file ShapeListViewDlg.h
 *
 *	\author   Daniel Küthe
 *
 *  \brief A non-modal pane window for interactive view and change of shapes in 3D view
 */

#pragma once

#include <list>
#include "WireCenterView.h"

#include <WireLib/Workspace.h>

//! a new namespace to seperate the new objects from the old ones
namespace AnalysisObjects {

//! an enum to differentiate the different types
enum AnalysisObjectType {aotNoObject, aotUnknown, aotHullObject, aotGridObject, aotCrosssectionObject, aotPoseListObject};
	
//! a base class for all hull-, contour-, grid- and pose-objects
class CAnalysisObject
{
public:
	//! the name of the analysis object
	CString name;
	//! a description
	CString info_str;
	//! the color of the rendered objects (NOTE: not applicable for all objects)
	double color[3];
	//! the elements visibility
	bool b_IsVisible;
	
private:
	//! the actual object type
	AnalysisObjectType object_type;
	//! the object id
	int nID;
	//! the timestamp
	CTime timestamp;
	//! the image indices for showing the icon in a view
	int visible_image_index;
	int invisible_image_index;

public:
	//! constructor of the class
	CAnalysisObject(int _nID, AnalysisObjectType _object_type, int _visible_image_index = 0, int _invisible_image_index = 0)
	{
		// set the object ID, object type and image indices and the current time
		nID = _nID;
		object_type =_object_type;
		visible_image_index = _visible_image_index;
		invisible_image_index = _invisible_image_index;
		timestamp = CTime::GetCurrentTime();
	}
	//! destructor of the class
	~CAnalysisObject() {};

	//! return the object type of this object
	AnalysisObjectType GetObjectType() { return object_type; }
	//! return the object type of this object
	int GetID() { return nID; }
	//! return the visible image index of this object
	int GetVisibleImageIndex() { return visible_image_index; }
	//! return the visible image index of this object
	int GetInvisibleImageIndex() { return invisible_image_index; }
	//! return the timestamp as string
	CString GetTimestamp() { return timestamp.Format("%a, %d.%m.%Y, um %H:%M"); }
	//! convert the current object to a poselist
	virtual PCRL::CPoseListKinetostatic* ConverttoPoselist() { return false; }
};

//! a class for containing the pose lists
class CPoseList: public CAnalysisObject
{
public:
	PCRL::CPoseListKinetostatic m_poselist;
	CPoseList(int _n_ID) : CAnalysisObject(_n_ID,/*AnalysisObjects::*/aotPoseListObject,145,145) {};

	//! sorts a poselist by weighted x, y and z-values and returns a pointer to the new pose_list object
	static PCRL::CPoseListKinetostatic* SortPoselist(PCRL::CPoseListKinetostatic* input_pose_list, int x_weight=10000, int y_weight=100, int z_weight=1);
};
	
//! a class for describing the hull objects
class CHull: public CAnalysisObject
{
public:
	vector<Vector3d> vertices;
	vector<PCRL::CTriangleIndices> triangles;

	//! the transparency value for hull objects
	double transparency;

	//! the constructor
	CHull(int _n_ID) : CAnalysisObject(_n_ID, aotHullObject, 158, 167) {}
	//! specific convert to poselist and returns a pointer to the new pose list object, which should be included in the poselist-list
	PCRL::CPoseListKinetostatic* ConverttoPoselist();
};
	
//! a class for describing the grid objects
class CGrid: public CAnalysisObject
{
public:
	vector<Vector3d> vertices;
		
	//! the constructor
	CGrid(int _n_ID) : CAnalysisObject(_n_ID, aotGridObject, 161, 168) {}
	//! specific convert to poselist and returns a pointer to the new pose list object, which should be included in the poselist-list
	PCRL::CPoseListKinetostatic* ConverttoPoselist();
};
	
//! a class for describing the crosssection objects
class CCrossSection: public CAnalysisObject
{
public:
	vector<Vector3d> vertices;
	vector<pair<int,int>> lines;

	//! the constructor
	CCrossSection(int _n_ID) : CAnalysisObject(_n_ID, aotCrosssectionObject, 129, 169) {}
	//! specific convert to poselist and returns a pointer to the new pose list object, which should be included in the poselist-list
	PCRL::CPoseListKinetostatic* ConverttoPoselist();
};


//! a new class containing hull-,contour- and grid-objects as well as a pose list data base (inherits from CGLShape to clip this object in the shape list and draw it!)
class CAnalysisObjectsContainer: public CGLShape
{
private:
	//! the pointer list member to store all information
	list<CAnalysisObject*> object_database;

	//! the global upcounting ID
	int m_globalID;

	//! a basic function to retrieve data from the original objects and store 
	//! them in the internal storages which returns the current added object 
	//! again and is never be called outside its class (only the overloaded 
	//! public methods are used)
	CAnalysisObject* RetrieveData(void* _new_object, AnalysisObjectType _object_type, COLORREF color, CString name, CString description, bool visible);

public:
	//constructor
	CAnalysisObjectsContainer() { m_globalID = 1; }

	//destructor
	~CAnalysisObjectsContainer()
	{
		//delete all objects
		DeleteAnalysisObject(NULL, AnalysisObjects::aotHullObject);
		DeleteAnalysisObject(NULL, AnalysisObjects::aotCrosssectionObject);
		DeleteAnalysisObject(NULL, AnalysisObjects::aotGridObject);
		DeleteAnalysisObject(NULL, AnalysisObjects::aotPoseListObject);
	}
	
	//! overloaded functions to retrieve new data and add function call specific values
	void RetrieveData(PCRL::CWorkspaceHull& _workspacehull, COLORREF color = RGB(255,100,100), CString name = "UNNAMED HULL", CString description = "", bool visible = true, double transparency = 0.0)
	{
		AnalysisObjects::CHull* new_hull = static_cast<AnalysisObjects::CHull*>(RetrieveData(&_workspacehull, AnalysisObjects::aotHullObject, color, name, description, visible));
		new_hull->transparency = transparency;	
	}
	
	void RetrieveData(PCRL::CWorkspaceGrid& _workspacegrid, COLORREF color = RGB(100,255,100), CString name = "UNNAMED GRID", CString description = "", bool visible = true)
	{ 
		RetrieveData(&_workspacegrid, AnalysisObjects::aotGridObject, color, name, description, visible); 
	}
	
	void RetrieveData(PCRL::CWorkspaceCrosssection& _workspacecrosssection, COLORREF color = RGB(100,100,255), CString name = "UNNAMED CROSSSECTION", CString description = "", bool visible = true)
	{
		RetrieveData(&_workspacecrosssection, AnalysisObjects::aotCrosssectionObject, color, name, description, visible);
	}

	AnalysisObjects::CAnalysisObject* RetrieveData(PCRL::CPoseListKinetostatic& _poselist, COLORREF color = RGB(100,100,255), CString name = "UNNAMED POSELIST", CString description = "", bool visible = true)
	{
		return RetrieveData(&_poselist, AnalysisObjects::aotPoseListObject, color, name, description, visible);
	}

	//! draw function (overrides the virtual draw function in CGLShape)
	void draw();
	
	//! get a reference to an object by its ID
	AnalysisObjects::CAnalysisObject* GetAnalysisObjectByID(int nID);

	//! delete an object or an group of objects ( the parameters are boolean OR linked )
	bool DeleteAnalysisObject(AnalysisObjects::CAnalysisObject* _analysis_object, AnalysisObjects::AnalysisObjectType _object_type);

	//! save the current container in a file
	bool SaveContainer() { return false; }
	
	//! get the current global object id
	int GetCurrentGlobalID() { return m_globalID; }

	//! get the type of the specified object
	AnalysisObjects::AnalysisObjectType GetAnalysisObjectType(AnalysisObjects::CAnalysisObject* _analysis_object)
		{ if(_analysis_object) return _analysis_object->GetObjectType(); return AnalysisObjects::aotNoObject; };

	//! hide all elements of an specified type
	void HideAllElements(AnalysisObjects::AnalysisObjectType object_type);
};

}	// namespace AnalysisObjects

//################################################################################################################################
//################################################################################################################################
// The following implementation is again MFC style. We add classes to show the 
// pane window and to handle events generated in that windows

//! create a enum for simpler using the ID_COMMANDS for the treeview entries 
//! to control the scene object
//! these command ID are used in the same way as the command ID declared in
//! resource.h. However, since all these commands are not bound to dialog
//! resources (but instead are only used in inline definition of menus)
//! we can declare the IDs in a type-safe way through enums. 
enum TreeViewItemsID {
	tvitemNONCLICKABLE=500,				//!< use this identifier for non clickable objects (start at 500 for having "savety margin" at the begining)
	
	tvitemPOSELISTOBJECTS_BRANCH,		//!< this is the ID for the branch containing the calculated poselist objects (and their sub elements)
	tvitemHULLOBJECTS_BRANCH,			//!< this is the ID for the branch containing the saved hull-objects
	tvitemGRIDOBJECTS_BRANCH,			//!< this is the ID for the branch containing the saved grid-objects
	tvitemCROSSSECTIONOBJECTS_BRANCH,	//!< this is the ID for the branch containing the saved crosssection-objects
	tvitemORIENTATIONS_BRANCH,			//!< this is the ID for the branch containing the global orientations
	tvitemINTERNALOBJECTS_BRANCH,		//!< this is the ID for the branch containing the internal scene graph objects
	tvitemUSEROBJECTS_BRANCH,			//!< this is the ID for the branch containing the user merged scene graph objects

	tvitemBEGIN_COMMANDS=800,			//!< insert direct command entries NOTE: there's a maximum of 199 different commands which can be handled
	tvitemEND_COMMANDS=999,

	tvitemBEGIN_ORIENTATIONS=1000,		//!< insert direct command entries NOTE: there's a maximum of 8999 different commands which can be handled
	tvitemEND_ORIENTATIONS=9999,

	tvitemBEGIN_ANALYSISOBJECTS=10000,	//!< insert analysisobjects down here; NOTE: there's a maximum of 9999 different objects, which can be handled
	tvitemEND_ANALYSISOBJECTS=19999, 

	tvitemBEGIN_GRAPHICALOBJECTS=20000,	//!< insert shape objects here; NOTE: there's a maximum of 9999 different objects, which can be handled
	tvitemEND_GRAPHICALOBJECTS=30000
};

//! create a enum for simpler using the ID_COMMANDS for the context menu
enum PopupMenuElementsID {
	pmeFIRSTITEM=1,						//!< pmeFIRSTITEM and pmeLASTITEM are necessary to determine the range of the ON_COMMAND_EX_RANGE command

	// general purpose menu elements
	pmeAlterVisibility,
	pmeLayerUp,
	pmeLayerDown,
	pmeDeleteObject,
	pmeConverttoPoselist,
	pmeShowGeneralProperties,

	// hull specific menu elements for analysis objects
	pmeAddCurrentWorkspaceHull,
	pmeAddCurrentGrid,
	pmeAddCurrentCrosssection,
	pmeDeleteAllObjectsInBranch,

	// pose list specific menu elements
	pmeNCInterpretePoseList,
	pmeNCLoadPoseList,
	pmeAnalyzePoseList,
	pmeShowPoseListProperties,

	//insert new elements here

	pmeLASTITEM						//!< this item's value reflects the number of pme symbols within this enum
};

//! a new class derived from CTreeView with right click context menu support
//! this control is the major part in the analysis objects pane and contains
//! a number of application specific event handler
class CMenuTreeCtrl : public CTreeCtrl
{
public:
	//! the constructor saves the pointer to the wirecenter scene object in a private member variable
	CMenuTreeCtrl() {
		pSceneObject = &CWireCenterView::This->m_Scene; pShapeList = pSceneObject->GetHandle();
		last_selItem = NULL;
	}
	//! handles the right click and calls the OnContextMenu function
	void OnRClick(NMHDR* pNMHDR, LRESULT* pResult); 
	//! handles the left click
	void OnLClick(NMHDR* pNMHDR, LRESULT* pResult); 
	//! handles the doubleclick selecting and marking of the entries and special functions (e.g. deactivate camera movement)
	void OnDBLClick(NMHDR* pNMHDR, LRESULT* pResult);
	//! handles the context menu creation on right click
	void OnContextMenu(CWnd* pWnd, CPoint ptMousePos);
	
	//! returns the HTREEITEM value for the item referenced by its ID (NOTE: this function only does a traversal by siblings of the root!)
	HTREEITEM FindTreeItemByID(int ID);

	//! the current selected item
	HTREEITEM selItem;

	//! the analysis objects container (SHOULDN'T BE STORED HERE ; ONLY FOR TESTING PURPOSE)
	AnalysisObjects::CAnalysisObjectsContainer analysisobjectscontainer;

protected:
	DECLARE_MESSAGE_MAP()

private:
	//! handles the selection in the context menu of the TreeControl
	afx_msg BOOL OnTreeMenuClicked(UINT nID);
	
	//! the before selected item
	HTREEITEM last_selItem;

	//! Save the pointer locally to achieve and maintain access to the SceneGraph and the shape list
	CGLSceneGraph* pSceneObject;
	CGLShapeList* pShapeList;
};

//! a new shape list pane class
class CShapeListViewPane : public CDockablePane
{
public:
	CShapeListViewPane();
	virtual ~CShapeListViewPane() {
		//delete the pTreeCtrl and the pImageList!
		delete pTreeCtrl; delete pImageList;
	}
	void AdjustLayout();
	
	//! Build the tree control object
	void BuildTreeView();

	//! Find the given item recursivly by its ID
	CGLShape* RecursiveFind(CGLShapeList* node, int unique_id);

	//! for rebuilding the scene object graph
	void RecursiveEnumeration();

protected:
	DECLARE_MESSAGE_MAP()
	CFont m_fntPropList;
	
	//! a tree view control and the imagelist
	CMenuTreeCtrl* pTreeCtrl;
	CImageList* pImageList;

	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnSize(UINT nType, int cx, int cy);

private:
	//! build the tree list recursivly from any given node
	void RecursiveEnumeration(CGLShapeList* node);
	
	//! Save the pointer locally to achieve and maintain access to the SceneGraph and a certain ShapeList
	CGLShapeList* pShapeList;
	CGLSceneGraph* pSceneObject;
};
