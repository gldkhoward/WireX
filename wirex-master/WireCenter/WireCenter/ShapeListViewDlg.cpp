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

// ShapeListViewDlg.cpp: Implementierungsdatei
//

#include "stdafx.h"
#include "ShapeListViewDlg.h"
#include "WireCenterView.h"
#include "GenericParamDlg.h"

namespace AnalysisObjects {

////////////////////////////////////////////////////////////////
// temporary IMPLEMENTATION OF the namespace AnalysisObjects
///////////////////////////////////////////////////////////////

//! implementation for copying data from the workspace hull to the container object
CAnalysisObject* CAnalysisObjectsContainer::RetrieveData(void* _saveable_object, AnalysisObjectType _object_type, COLORREF color, CString name, CString description, bool visible)
{
	// set up a pointer to an analysis object and recast it later
	CAnalysisObject* _new_object; 

	// create a new object depending on the type and add type-specific basic values
	if (_object_type==aotHullObject) 
	{
		_new_object = new CHull(m_globalID);

		// copy the connecting information for the vertices
		static_cast<CHull*>(_new_object)->triangles = (static_cast<PCRL::CWorkspaceHull*>(_saveable_object))->Triangles;
		// eventually copy the vertices
		static_cast<CHull*>(_new_object)->vertices = (static_cast<PCRL::CWorkspaceHull*>(_saveable_object))->vertices;
	}
	else if (_object_type==aotGridObject)
	{
		 _new_object = new CGrid(m_globalID);

		 // copy the workspace vertices
		 static_cast<CGrid*>(_new_object)->vertices = static_cast<PCRL::CWorkspaceGrid*>(_saveable_object)->workspace;
	}
	else if (_object_type==aotCrosssectionObject)
	{
		_new_object = new CCrossSection(m_globalID);

		//copy the connecting information for the vertices
		static_cast<CCrossSection*>(_new_object)->lines = (static_cast<PCRL::CWorkspaceCrosssection*>(_saveable_object))->Lines;
		//eventually copy the vertices
		static_cast<CCrossSection*>(_new_object)->vertices = (static_cast<PCRL::CWorkspaceCrosssection*>(_saveable_object))->vertices;
	}
	else if (_object_type==aotPoseListObject)
	{
		_new_object = new CPoseList(m_globalID);
		
		// copy the pose list
		// ----------------------------------
		// NOTE: while all elements in the poselist are POINTER to special robot position vectors and matrices
		// the "copy operator" ( = ) won't work as expected, as it only copies the POINTERS and not the real robot positions
		// ... so we have to push all elements in the list manually
		// ((CPoseList*)_new_object)->m_poselist = *((PCRL::CPoseListKinetostatic*)_saveable_object);
		
		PCRL::CPoseListKinetostatic::iterator itor;
		for (itor=static_cast<PCRL::CPoseListKinetostatic*>(_saveable_object)->begin(); itor!=((PCRL::CPoseListKinetostatic*)_saveable_object)->end();itor++)
		{
			static_cast<CPoseList*>(_new_object)->m_poselist.push_back(new PCRL::CPoseKinetostatic((*itor)->r,(*itor)->R));
		}
	}
	// type not found -> return null-pointer
	else 
		return NULL;

	// increase global ID
	m_globalID++;

	// set the color
	_new_object->color[0] = (double)GetRValue(color)/255;
	_new_object->color[1] = (double)GetGValue(color)/255;
	_new_object->color[2] = (double)GetBValue(color)/255;
	// set the name
	_new_object->name = name;
	// set the description
	_new_object->info_str = description;
	// set the initial visibiliy
	_new_object->b_IsVisible = visible;
	// save the new hull object in the database
	object_database.push_back(_new_object);

	// everything worked out fine, so return the pointer to this new object
	return _new_object;
}

//! the overridden draw function to draw all elements in the container
void CAnalysisObjectsContainer::draw()
{
	// draw the objects
	for (list<CAnalysisObject*>::iterator itor = object_database.begin(); itor!=object_database.end(); itor++)
	{
		// if object isn't visible, go to the next
		if (!(*itor)->b_IsVisible) 
			continue;

		// if element is a hull object and it is visible...
		else if ((*itor)->GetObjectType()==aotHullObject)
		{
			// ...draw it!
			// enable transluceny
			glEnable(GL_BLEND); 
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			// cast the itor to an CHull object
			CHull* hull = static_cast<CHull*>(*itor); 
			// set the wished color and the alpha value
			glColor4d((*itor)->color[0], (*itor)->color[1], (*itor)->color[2], (1.0-hull->transparency));

			// run through all triangles:
			for (UINT curVer = 0 ; curVer < hull->triangles.size(); curVer++)
			{
				// calculate the normal vector
				Vector3d normal=(hull->vertices[hull->triangles[curVer].i]- hull->vertices[hull->triangles[curVer].j]).cross(hull->vertices[hull->triangles[curVer].i]-hull->vertices[hull->triangles[curVer].k]);
				// draw the triangle
				glBegin(GL_TRIANGLES);
					glNormal3dv(&normal.x());
					glVertex3dv(&hull->vertices[hull->triangles[curVer].i].x());
					glVertex3dv(&hull->vertices[hull->triangles[curVer].j].x());
					glVertex3dv(&hull->vertices[hull->triangles[curVer].k].x());
				glEnd();
			}

			// disable blending again
			glDisable(GL_BLEND);
		}	

		// if element is a grid object and it is visible...
		else if ((*itor)->GetObjectType()==aotGridObject)
		{
			// ...draw it!
			// set point size
			glPointSize(5.0);
			// cast the itor to an CHull object
			CGrid* grid = static_cast<CGrid*>(*itor); 
			// set the wished color
			glColor3d((*itor)->color[0], (*itor)->color[1], (*itor)->color[2]);
			
			// draw the point
			glBegin(GL_POINTS);
			for (unsigned int curVer = 0 ; curVer < grid->vertices.size(); curVer++) 
				glVertex3dv(&grid->vertices[curVer].x());
			glEnd();

			// reset point size again
			glPointSize(1.0);
		}	
		
		// if element is a crosssection object and it is visible...
		else if ((*itor)->GetObjectType()==aotCrosssectionObject)
		{
			// cast the itor to an CHull object
			CCrossSection* crosssection = static_cast<CCrossSection*>(*itor); 
			// set the wished color and a simple normal vector
			glColor3d((*itor)->color[0], (*itor)->color[1], (*itor)->color[2]);
			glNormal3d(0,0,1);

			// run through all triangles:
			for (UINT cur_line = 0 ; cur_line < crosssection->lines.size(); cur_line++)
			{
				// draw the lines
				glBegin(GL_LINES);
					glVertex3dv(&crosssection->vertices[crosssection->lines[cur_line].first].x());
					glVertex3dv(&crosssection->vertices[crosssection->lines[cur_line].second].x());
				glEnd();
			}
		}	
	}
}

CAnalysisObject* CAnalysisObjectsContainer::GetAnalysisObjectByID(int nID)
{
	// search through all objects and look for the searched one
	for (list<CAnalysisObject*>::iterator itor = object_database.begin(); itor!=object_database.end(); itor++) 
	{ 
		if ((*itor)->GetID()==nID) 
			return (*itor); 
	}
	// if nothing found, return NULL
	return NULL;
}

bool CAnalysisObjectsContainer::DeleteAnalysisObject(CAnalysisObject* _analysis_object, AnalysisObjectType _object_type)
{
	// search through all objects and look for the searched one
	list<CAnalysisObject*>::iterator itor=object_database.begin();
	while (itor != object_database.end()) 
	{ 
		if ((*itor)==_analysis_object || (*itor)->GetObjectType()==_object_type) 
		{
			// delete the object from the list (we have to distinguish between all object types to avoid memory leaks)
			if ((*itor)->GetObjectType()==aotHullObject) 
				delete static_cast<CHull*>(*itor);
			else if ((*itor)->GetObjectType()==aotGridObject) 
				delete static_cast<CGrid*>(*itor);
			else if ((*itor)->GetObjectType()==aotCrosssectionObject) 
				delete static_cast<CCrossSection*>(*itor);
			else if ((*itor)->GetObjectType()==aotPoseListObject) 
				delete static_cast<CPoseList*>(*itor);
			else 
				delete (*itor); // no error , if object type isn't correct, but some memory stays allocated
			itor = object_database.erase(itor);
		}
		else 
			itor++;
	}

	// if nothing was deleted, return false
	return false;
}


void CAnalysisObjectsContainer::HideAllElements(AnalysisObjectType object_type)
{
	// search through all objects and look for the searched type and hide it
	for (list<CAnalysisObject*>::iterator itor=object_database.begin(); itor!=object_database.end(); itor++) 
	{ 
		if ((*itor)->GetObjectType()==object_type) 
			(*itor)->b_IsVisible = false;
	}
}

// the specific function to convert the current hull to a pose list
PCRL::CPoseListKinetostatic* CHull::ConverttoPoselist()
{
	// create a new poselist object for storing
	PCRL::CPoseListKinetostatic* pose_list = new PCRL::CPoseListKinetostatic;

	// a temporary rotation matrix (entity matrix at the moment)
	Matrix3d R(Matrix3d::Identity());

	// copy all vertices position to the pose list
	for (std::vector<Vector3d>::iterator itor = vertices.begin(); itor != vertices.end(); itor++)
	{
		// copy the vertices positions to the poselist list member
		pose_list->push_back(new PCRL::CPoseKinetostatic((*itor),R));
	}

	// conversion succeeded and return the list!
	return pose_list;
}

//! the specific function to convert the current grid to a pose list
PCRL::CPoseListKinetostatic* CGrid::ConverttoPoselist()
{	
	// create a new poselist object for storing
	PCRL::CPoseListKinetostatic* pose_list = new PCRL::CPoseListKinetostatic;

	// a temporary rotation matrix (entity matrix at the moment)
	Matrix3d R(Matrix3d::Identity());

	// copy all vertices position to the pose list
	for (std::vector<Vector3d>::iterator itor = vertices.begin(); itor != vertices.end(); itor++)
	{
		//copy the vertices positions to the poselist list member
		pose_list->push_back(new PCRL::CPoseKinetostatic((*itor), R));
	}

	// conversion succeeded and return the list!
	return pose_list;
}

//! the specific function to convert the current crosssection to a pose list
PCRL::CPoseListKinetostatic* CCrossSection::ConverttoPoselist()
{
	// create a new poselist object for storing
	PCRL::CPoseListKinetostatic* pose_list = new PCRL::CPoseListKinetostatic;

	// a temporary rotation matrix (entity matrix at the moment)
	Matrix3d R(Matrix3d::Identity());

	// since the crosssection represents a circle, it would be nice to traverse the circle correctly!
	// copy all vertices position to the pose list based on the lines pair
	for (std::vector<std::pair<int,int>>::iterator itor = lines.begin(); itor != lines.end(); itor++)
	{
		// copy the vertices positions to the poselist list member
		pose_list->push_back(new PCRL::CPoseKinetostatic(vertices.at((*itor).first),R));
	}

	// conversion succeeded and return the list!
	return pose_list;
}

//! the specific function to sort the pose list (simply by sorting with weighted x,y and z coordinates)
PCRL::CPoseListKinetostatic* CPoseList::SortPoselist(PCRL::CPoseListKinetostatic* input_pose_list, int x_weight, int y_weight, int z_weight)
{
	// the resulting poselist
	PCRL::CPoseListKinetostatic* result_pose_list = new PCRL::CPoseListKinetostatic;
	
	// some helping variables
	PCRL::CPoseKinetostatic* current_used_pose;
	PCRL::CPoseListKinetostatic::iterator del_itor;

	// set the maximum distance to maximum double values
	double distance_element=DBL_MAX-1;
	double distance_compare=DBL_MAX;
	
	// while list isn't empty ... go on
	while (!input_pose_list->empty())
	{
		// reset the distance
		distance_compare=DBL_MAX;

		// find minimum from all vectors
		for (PCRL::CPoseListKinetostatic::iterator itor = input_pose_list->begin(); itor != input_pose_list->end(); itor++)
		{
			// get the global weighted position...
			distance_element = x_weight*((*itor)->r.x())+y_weight*((*itor)->r.y())+z_weight*((*itor)->r.z());

			// ... and if it is smaller or equal as the current one, save it!
			if ((distance_element<=distance_compare)) 
			{
				//set new length
				distance_compare = distance_element;
				// save the itor element
				current_used_pose = (*itor);
				del_itor = itor;
			}
		}

		// insert element in the new list
		result_pose_list->push_back(current_used_pose);

		// delete the current used pose from the old list
		input_pose_list->remove(current_used_pose);
	}

	// delete the input pose list (because all elements from this list are store in the result pose list)
	delete input_pose_list;

	// conversion succeeded and return the list!
	return result_pose_list;
}

} // end of namespace AnalysisObjects

/////////////////////////////////////////////
// IMPLEMENTATION OF CMenuTreeCtrl
/////////////////////////////////////////////

BEGIN_MESSAGE_MAP(CMenuTreeCtrl, CTreeCtrl)
	ON_NOTIFY_REFLECT(NM_CLICK, OnLClick)
	ON_NOTIFY_REFLECT(NM_RCLICK, OnRClick)
	ON_NOTIFY_REFLECT(NM_DBLCLK, OnDBLClick)
	ON_WM_CONTEXTMENU()
	//use a ON_COMMAND_RANGE_EX to retrieve the ID for case decision while mapping all IDs on the same procedure on the whole PopupMenuElementsID
	ON_COMMAND_EX_RANGE(pmeFIRSTITEM,pmeLASTITEM, &CMenuTreeCtrl::OnTreeMenuClicked)
END_MESSAGE_MAP()

void CMenuTreeCtrl::OnRClick(NMHDR* pNMHDR, LRESULT* pResult) 
{
	// Send WM_CONTEXTMENU to self
	SendMessage(WM_CONTEXTMENU, (WPARAM) m_hWnd, GetMessagePos());
	// Mark message as handled and suppress default handling
	*pResult = 1;
}

//! refresh the internal objects, if any of this branches is clicked
void CMenuTreeCtrl::OnLClick(NMHDR* pNMHDR, LRESULT* pResult) 
{
	// get the position of the last message
	CPoint ptMousePos = (CPoint) GetMessagePos();
	// transform coordinates
	ScreenToClient(&ptMousePos);
	
	// get parent again
	CShapeListViewPane* pParent = (CShapeListViewPane*)this->GetParent();
	
	// determine, which element was clicked
	UINT uFlags;
	HTREEITEM htItem = HitTest( ptMousePos, &uFlags );

	// if no item is selected, return
	if (!htItem) return;

	// get the selected item and its data, if available
	int item_data = GetItemData(htItem);
	
	// handle case for rebuilding the internal objects and the orientations part again
	if (item_data==tvitemUSEROBJECTS_BRANCH||item_data==tvitemINTERNALOBJECTS_BRANCH||item_data==tvitemORIENTATIONS_BRANCH) 
	{
		// rebuild the internal, hull and user objects again
		pParent->RecursiveEnumeration();
	}
}

void CMenuTreeCtrl::OnDBLClick(NMHDR* pNMHDR, LRESULT* pResult) 
{
	// get the position of the last message
	CPoint ptMousePos = (CPoint) GetMessagePos();
	// transform coordinates
	ScreenToClient(&ptMousePos);
	
	// determine, which element was clicked
	UINT uFlags;
	HTREEITEM htItem = HitTest( ptMousePos, &uFlags );

	// get parent again
	CShapeListViewPane* pParent = (CShapeListViewPane*)this->GetParent();

	// get the selected item and its data
	selItem = GetSelectedItem();
	int item_data = GetItemData(selItem);
	
	// if doubleclicked element is an orientation, ...
	if (item_data>tvitemBEGIN_ORIENTATIONS&&item_data<tvitemEND_ORIENTATIONS)
	{
		// set the current orientation of the robot to this and repaint the view
		CWireCenterView::This->GetRobotDoc()->R = CWireCenterView::This->GetRobotDoc()->WSHull.getOrientation(item_data-tvitemBEGIN_ORIENTATIONS-1);
		CWireCenterView::This->Invalidate();
	}
	else // ... else switch visibility of objects by simulating the tree menu click
	{
		OnTreeMenuClicked(pmeAlterVisibility);
	}
}

/*! This function generates the context menus on the fly. In contract to than other parts 
 *  of WireCenter, this context menus are not based on menu ressources but are created
 *  explicitly in the code instead.
 *  Changing and editing the context menues must be done through modification in the code
 *  of this function.
 */
void CMenuTreeCtrl::OnContextMenu(CWnd* pWnd, CPoint ptMousePos) 
{
	// get the clicked item
	ScreenToClient(&ptMousePos);
	UINT uFlags;
	HTREEITEM htItem = HitTest( ptMousePos, &uFlags );
	
	// if no item was found, nothing is to do here
	if( htItem == NULL ) 
		return;

	// select the item and get the item data
	SelectItem(htItem);
	int item_data = GetItemData(htItem);

	// get the type of the selected object
	AnalysisObjects::AnalysisObjectType object_type = analysisobjectscontainer.GetAnalysisObjectType(analysisobjectscontainer.GetAnalysisObjectByID(item_data-tvitemBEGIN_ANALYSISOBJECTS));

	// create the CMenu objects already here
	CMenu mMenu,mSubMenu;
	VERIFY(mSubMenu.CreateMenu());
	VERIFY(mMenu.CreateMenu());
		
	// differentiate the clicked tree view items
	// if selected item is the grouping POSELIST BRANCH...
	if (item_data==tvitemPOSELISTOBJECTS_BRANCH)
	{
		// create a popup menu and insert some entries
		mMenu.AppendMenu(MF_STRING, pmeDeleteAllObjectsInBranch, _T("&Delete all pose objects"));
	}
	// if selected item is the grouping HULLOBJECT BRANCH...
	else if (item_data==tvitemHULLOBJECTS_BRANCH) 
	{
		// create a popup menu and insert some entries
		mMenu.AppendMenu(MF_STRING, pmeAddCurrentWorkspaceHull, _T("&add current workspace hull"));
		mMenu.AppendMenu(MF_SEPARATOR, 0, _T(""));
		mMenu.AppendMenu(MF_STRING, pmeDeleteAllObjectsInBranch, _T("d&elete all hulls"));
	}
	// if selected item is the grouping GRIDOBJECT BRANCH...
	else if (item_data==tvitemGRIDOBJECTS_BRANCH) 
	{
		// create a popup menu and insert some entries
		mMenu.AppendMenu(MF_STRING, pmeAddCurrentGrid, _T("&add current workspace grid"));
		mMenu.AppendMenu(MF_SEPARATOR, 0, _T(""));
		mMenu.AppendMenu(MF_STRING, pmeDeleteAllObjectsInBranch, _T("d&elete all grid objects"));
	}
	// if selected item is the grouping CROSSSECTION BRANCH...
	else if (item_data==tvitemCROSSSECTIONOBJECTS_BRANCH) 
	{
		// create a popup menu and insert some entries
		mMenu.AppendMenu(MF_STRING, pmeAddCurrentCrosssection, _T("&add current cross section"));
		mMenu.AppendMenu(MF_SEPARATOR, 0, _T(""));
		mMenu.AppendMenu(MF_STRING, pmeDeleteAllObjectsInBranch, _T("d&elete all cross sections"));
	}
	// if selected item is a HULLOBJECT ...
	else if (object_type==AnalysisObjects::aotHullObject) 
	{
		// create a popup menu
		mMenu.AppendMenu(MF_STRING, pmeConverttoPoselist, _T("&Convert hull to pose list"));
		mMenu.AppendMenu(MF_STRING, pmeShowGeneralProperties, _T("&Properties of the hull"));
		mMenu.AppendMenu(MF_SEPARATOR, 0, _T(""));
		mMenu.AppendMenu(MF_STRING, pmeDeleteObject, _T("&Delete object"));
	}
	
	// if selected item is a GRIDOBJECT...
	else if (object_type==AnalysisObjects::aotGridObject) 
	{
		// create a popup menu and insert some entries
		mMenu.AppendMenu(MF_STRING, pmeShowGeneralProperties, _T("&Properties of the grid object"));
		mMenu.AppendMenu(MF_STRING, pmeConverttoPoselist, _T("&Convert grid to pose list"));
		mMenu.AppendMenu(MF_SEPARATOR, 0, _T(""));
		mMenu.AppendMenu(MF_STRING, pmeDeleteObject, _T("&Delete grid object"));
	}
	// if selected item is a CROSSSECTIONOBJETC...
	else if (object_type==AnalysisObjects::aotCrosssectionObject) 
	{
		// create a popup menu and insert some entries
		mMenu.AppendMenu(MF_STRING, pmeShowGeneralProperties, _T("Properties of the cross sections"));
		mMenu.AppendMenu(MF_STRING, pmeConverttoPoselist, _T("Convert cross section to pose list"));
		mMenu.AppendMenu(MF_SEPARATOR, 0, _T(""));
		mMenu.AppendMenu(MF_STRING, pmeDeleteObject, _T("Delete cross section object"));
	}
	// if selected item is a POSELISTOBJECT...
	else if (object_type==AnalysisObjects::aotPoseListObject) 
	{
		// create a popup menu
		mMenu.AppendMenu(MF_STRING, pmeNCInterpretePoseList, _T("&Interpolate (resample) with NC Interpreter"));
		mMenu.AppendMenu(MF_STRING, pmeNCLoadPoseList, _T("&Copy into interpolator pose list"));
		mMenu.AppendMenu(MF_STRING, pmeAnalyzePoseList, _T("&Analyze and export poses"));
		mMenu.AppendMenu(MF_STRING, pmeShowPoseListProperties, _T("&Properties of the pose lits"));
		mMenu.AppendMenu(MF_SEPARATOR, 0, _T(""));
		mMenu.AppendMenu(MF_STRING, pmeDeleteObject, _T("&Delete Pose List"));
	}
	// if selected item is in the graphical objects group...
	else if (item_data>=tvitemBEGIN_GRAPHICALOBJECTS&&item_data<=tvitemEND_GRAPHICALOBJECTS)
	{
		// create a popup menu and insert some entries
		mMenu.AppendMenu(MF_STRING, pmeAlterVisibility, _T("&Toggle visibility"));
		mMenu.AppendMenu(MF_SEPARATOR, 0, _T(""));
		mMenu.AppendMenu(MF_STRING, pmeLayerUp, _T("Layer up (not implemented)"));
		mMenu.AppendMenu(MF_STRING, pmeLayerDown, _T("Layer down (not implemented)"));
		mMenu.AppendMenu(MF_SEPARATOR, 0, _T(""));
		mMenu.AppendMenu(MF_STRING, pmeDeleteObject, _T("&Delete"));
	}
	else 
		return; // if we have no menu for the selected item, do nothing

	// built the menu, map the menu to screen and make the menu entries choosable
	mSubMenu.AppendMenu(MF_POPUP, (UINT)mMenu.m_hMenu, "");
	ClientToScreen(&ptMousePos);
	mMenu.TrackPopupMenu(TPM_LEFTALIGN, ptMousePos.x, ptMousePos.y, this);
}

/*! When a menu items in the context menu is clicked, this function mostly acts as
 *  event handler to perform the desired actions. 
 */
BOOL CMenuTreeCtrl::OnTreeMenuClicked(UINT nID)
{
	// get parent again
	CShapeListViewPane* pParent = (CShapeListViewPane*)this->GetParent();

	// a pointer to an shape
	CGLShape* pElement = NULL;

	// get the selected item
	selItem = GetSelectedItem();
	int item_data = GetItemData(selItem);

	// get the selected analysis object
	AnalysisObjects::CAnalysisObject* selected_object = analysisobjectscontainer.GetAnalysisObjectByID(item_data-tvitemBEGIN_ANALYSISOBJECTS);

	// test, if the command require to search an object in the CGLShapeList
	if (item_data>tvitemBEGIN_GRAPHICALOBJECTS&&item_data<tvitemEND_GRAPHICALOBJECTS)
	{
		// find the id in the ShapeList struct and get its adress
		pElement = pParent->RecursiveFind(pShapeList,item_data-tvitemBEGIN_GRAPHICALOBJECTS);

		// if found, change the values otherwise throw an error
		if (!pElement) 
		{
			cout << "Node with ID " << item_data << " not found!" << endl;
			return false;
		}
	}

	if (nID == pmeAlterVisibility)
	{
		// distinguish between analysis objects and graph-objects...
		if (item_data>tvitemBEGIN_GRAPHICALOBJECTS && item_data<tvitemEND_GRAPHICALOBJECTS)
		{
			// invert the visibility...
			pElement->bVisible = !pElement->bVisible;
			// ... and change the item text
			CString str;
			str.Format("Object ID: %i ; Name: %s ; Layer: %i ; Visibility: %s", pElement->getID(), "Unnamed" , pElement->getLayer() ,pElement->bVisible ? "Yes" : "No");
			SetItemText(selItem,str);
		}
		else if (item_data>tvitemBEGIN_ANALYSISOBJECTS && item_data<tvitemEND_ANALYSISOBJECTS)
		{
			// invert the visibility
			selected_object->b_IsVisible=!selected_object->b_IsVisible;
			// alter the image (grayed out, if not visible)
			if (selected_object->b_IsVisible) 
				SetItemImage(selItem,selected_object->GetVisibleImageIndex(),selected_object->GetVisibleImageIndex());
			else 
				SetItemImage(selItem,selected_object->GetInvisibleImageIndex(),selected_object->GetInvisibleImageIndex());
		}
			
	}
	else if (nID == pmeDeleteObject)
	{
		// if the object exists, you can delete it!
		if (selected_object)
		{
			// delete the object
			analysisobjectscontainer.DeleteAnalysisObject(selected_object,AnalysisObjects::aotUnknown);
		}
		else if (item_data>tvitemBEGIN_GRAPHICALOBJECTS&&item_data<tvitemEND_GRAPHICALOBJECTS)
		{
			// delete this particluar element
			pShapeList->deleteShape(item_data-tvitemBEGIN_GRAPHICALOBJECTS);
		}
		// delete the tree view entry
		DeleteItem(selItem);
	}
	else if (nID == pmeDeleteAllObjectsInBranch)
	{
		// delete all objects from this type in the container, if a branch was selected before
		if (GetItemData(selItem)==tvitemHULLOBJECTS_BRANCH) 
			analysisobjectscontainer.DeleteAnalysisObject(NULL,AnalysisObjects::aotHullObject);
		else if(GetItemData(selItem)==tvitemGRIDOBJECTS_BRANCH) 
			analysisobjectscontainer.DeleteAnalysisObject(NULL,AnalysisObjects::aotGridObject);		
		else if(GetItemData(selItem)==tvitemPOSELISTOBJECTS_BRANCH) 
			analysisobjectscontainer.DeleteAnalysisObject(NULL,AnalysisObjects::aotPoseListObject);	
		else if(GetItemData(selItem)==tvitemCROSSSECTIONOBJECTS_BRANCH) 
			analysisobjectscontainer.DeleteAnalysisObject(NULL,AnalysisObjects::aotCrosssectionObject);
		else 
			return false;
		
		// delete the objects in the tree view: while there are some child objects...delete them
		while (GetChildItem(FindTreeItemByID(GetItemData(selItem)))!=NULL) 
			DeleteItem(GetChildItem(FindTreeItemByID(GetItemData(selItem))));
	}
	else if (nID == pmeShowGeneralProperties)
	{
		// create a dialog and show / alter the properties
		CEnhancedDialog DLG("Properties of the object",300);
		CString str;
		bool change_color = false;
		DLG.StartGroupBox("Object properties:");
			str.Format("ID: %i",selected_object->GetID()); DLG.addItem(str);
			str.Format("Time stamp: %s",selected_object->GetTimestamp()); DLG.addItem(str);
			DLG.addItem("Name:",selected_object->name);
			DLG.addItem("Description:",selected_object->info_str);
			DLG.addItem("Change color",change_color);
			if (selected_object->GetObjectType()==AnalysisObjects::aotHullObject) 
				DLG.addItem("Transparency",((AnalysisObjects::CHull*)selected_object)->transparency);
		DLG.EndGroupBox();
		// show dialog
		DLG.DoModal();

		// set the (maybe?) new name
		SetItemText(selItem,selected_object->name);

		// alter color, if needed
		if (change_color)
		{
			CColorDialog DLG;
			if (DLG.DoModal()==IDOK) 
			{
				selected_object->color[0]=GetRValue(DLG.GetColor());
				selected_object->color[1]=GetGValue(DLG.GetColor());
				selected_object->color[2]=GetBValue(DLG.GetColor());
			}
		}
	}
	else if (nID == pmeShowPoseListProperties)
	{
		// create a dialog and show / alter the properties
		CEnhancedDialog DLG("Properties of Pose Lists",300);
		CString str;
		bool change_color = false;
		DLG.StartGroupBox("Poselist:");
			str.Format("ID: %i",selected_object->GetID()); DLG.addItem(str);
			str.Format("Time stamp: %s",selected_object->GetTimestamp()); DLG.addItem(str);
			DLG.addItem("Name:",selected_object->name);
			DLG.addItem("Description:",selected_object->info_str);
			if (selected_object->GetObjectType()==AnalysisObjects::aotPoseListObject)
			{
				str.Format("Number of Poses: %i",((AnalysisObjects::CPoseList*)selected_object)->m_poselist.size()); 
				DLG.addItem(str);
			}
			DLG.addItem("Change color",change_color);
			if (selected_object->GetObjectType()==AnalysisObjects::aotHullObject) 
				DLG.addItem("Transparency",((AnalysisObjects::CHull*)selected_object)->transparency);
		DLG.EndGroupBox();
		// show dialog
		if (DLG.DoModal() == IDOK)
		{
			// set the possibly new name
			SetItemText(selItem,selected_object->name);

			// alter color, if selected in the dialog
			if (change_color)
			{
				CColorDialog DLG;
				if (DLG.DoModal()==IDOK) 
				{
					selected_object->color[0]=GetRValue(DLG.GetColor());
					selected_object->color[1]=GetGValue(DLG.GetColor());
					selected_object->color[2]=GetBValue(DLG.GetColor());
				}
			}
		}
	}
	else if (nID == pmeAddCurrentWorkspaceHull)
	{
		// get the hull vertices and save in pHull (remember: here pHull(CWorkspaceHull) != pHull(CShapeWorkspaceHull) )
		PCRL::CWorkspaceHull* pHull = &CWireCenterView::This->GetRobotDoc()->WSHull;
	
		// if there are no finished triangles, it makes no sense to save the object
		if (pHull->FinishedTriangles == 0)
		{
			AfxMessageBox(CString("The current workspace hull was not yet computed\n and is thus not added!"),MB_OK|MB_ICONWARNING);
			return false;
		}
		
		// built the dialog and show it
		CString name = "Unnamed Hull";
		CString description = "";
		bool choose_color = false;
		bool visible = true;
		bool transparency = false;
		COLORREF color = RGB(255,40,40);

		CEnhancedDialog DLG("Add curent workspace hull");
		DLG.addItem("Name:",name);
		DLG.addItem("Description:",description);
		DLG.addItem("Visible",visible);
		DLG.addItem("Choose color", choose_color);
		DLG.addItem("Half-Transparency",transparency);
		DLG.addItem("Hint: Transparency does to work with clipping!");
		if (DLG.DoModal()!=IDOK) 
			return false;

		// if color wants to be choosen
		if (choose_color)
		{
			CColorDialog DLG(color);
			if (DLG.DoModal()==IDOK) color = DLG.GetColor();
		}

		// store the data and set a transparency value for the boolean flag
		analysisobjectscontainer.RetrieveData(*pHull, color, name, description, visible, transparency? 0.4 : 0.0);
		
		// insert the new hull in the tree view, set the data to identify the object and expand the branch
		HTREEITEM hNewHull = InsertItem(name,158,158,FindTreeItemByID(tvitemHULLOBJECTS_BRANCH),TVI_LAST);
		SetItemData(hNewHull,tvitemBEGIN_ANALYSISOBJECTS+analysisobjectscontainer.GetCurrentGlobalID()-1);
		Expand(FindTreeItemByID(tvitemHULLOBJECTS_BRANCH),TVE_EXPAND);

	}
	else if (nID == pmeAddCurrentGrid)
	{
		// get the grid vertices and save in pGrid
		PCRL::CWorkspaceGrid* pGrid = &CWireCenterView::This->GetRobotDoc()->WSGrid;
	
		// if there are no finished triangles, it makes no sense to save the object
		if (pGrid->workspace.size()==0)
		{
			AfxMessageBox(CString("Current Grid is empty\n and was not added!"),MB_OK|MB_ICONWARNING);
			return false;
		}
		
		// built the dialog and show it
		CString name = "Unnamed Grid";
		CString description = "";
		bool choose_color = false;
		bool visible = true;
		COLORREF color = RGB(40,255,40);

		CEnhancedDialog DLG("Add curent workspace grid");
		DLG.addItem("Name:",name);
		DLG.addItem("Description:",description);
		DLG.addItem("Chosoe Color", choose_color);
		DLG.addItem("Visible",visible);
		if (DLG.DoModal()!=IDOK) 
			return false;

		// if color wants to be choosen
		if (choose_color)
		{
			CColorDialog DLG(color);
			if (DLG.DoModal()==IDOK) 
				color = DLG.GetColor();
		}

		// store the data
		analysisobjectscontainer.RetrieveData(*pGrid,color,name,description,visible);

		// insert the new grid in the tree view, set the data to identify the object and expand the branch
		HTREEITEM hNewGrid = InsertItem(name,161,161,FindTreeItemByID(tvitemGRIDOBJECTS_BRANCH),TVI_LAST);
		SetItemData(hNewGrid,tvitemBEGIN_ANALYSISOBJECTS+analysisobjectscontainer.GetCurrentGlobalID()-1);
		Expand(FindTreeItemByID(tvitemGRIDOBJECTS_BRANCH),TVE_EXPAND);
	}
	else if (nID == pmeAddCurrentCrosssection)
	{
		// get the crosssection vertices and save in pCross
		PCRL::CWorkspaceCrosssection* pCross = &CWireCenterView::This->GetRobotDoc()->Crosssection;
	
		// if there are no finished vertices, it makes no sense to save the object
		if (pCross->vertices.size()==0)
		{
			AfxMessageBox(CString("Current cross section has not not been computed\nand was not added!"),MB_OK|MB_ICONWARNING);
			return false;
		}
		
		// built the dialog and show it
		CString name = "Unnamed Crosssection";
		CString description = "";
		bool choose_color = false;
		bool visible = true;
		COLORREF color = RGB(40,40,255);

		CEnhancedDialog DLG("Add current workspace cross section");
		DLG.addItem("Name:",name);
		DLG.addItem("Description:",description);
		DLG.addItem("Choose Color", choose_color);
		DLG.addItem("Visible",visible);
		if (DLG.DoModal()!=IDOK) 
			return false;

		// if color wants to be choosen
		if (choose_color)
		{
			CColorDialog DLG(color);
			if (DLG.DoModal()==IDOK) color = DLG.GetColor();
		}

		// store the data
		analysisobjectscontainer.RetrieveData(*pCross,color,name,description,visible);

		// insert the new grid in the tree view, set the data to identify the object and expand the branch
		HTREEITEM hNewCrosssection = InsertItem(name,129,129,FindTreeItemByID(tvitemCROSSSECTIONOBJECTS_BRANCH),TVI_LAST);
		SetItemData(hNewCrosssection,tvitemBEGIN_ANALYSISOBJECTS+analysisobjectscontainer.GetCurrentGlobalID()-1);
		Expand(FindTreeItemByID(tvitemCROSSSECTIONOBJECTS_BRANCH),TVE_EXPAND);
	}
	else if (nID == pmeConverttoPoselist)
	{
		// if there is no selected object, stop execution
		if (!selected_object) return false;
			 
		// pop a dialog for the name ( and the other parameters)
		CString name = "POSENLIST FROM ANALYSISOBJECT";
		CString description = "";
		bool sort_list = false;
		CEnhancedDialog DLG("Convert to pose list");
		DLG.addItem("Name:",name);
		DLG.addItem("Description:",description);
		// if it is a crosssection object, it makes no sense to sort it...
		if (selected_object->GetObjectType()!=AnalysisObjects::aotCrosssectionObject) 
		{
			sort_list = true;
			DLG.addItem("Sort list",sort_list);
		}
		
		// show the dialog box
		if (DLG.DoModal()!=IDOK) 
			return false;
		
		// cast the object and convert it to an pose list, otherwise throw an error
		PCRL::CPoseListKinetostatic* pPoseList = NULL;
		pPoseList = selected_object->ConverttoPoselist();
		if (!pPoseList)
		{
			AfxMessageBox("Object can not be converted to a pose list!");
			return false;
		}
		// if list should be sorted, show another dialog
		if (sort_list) 
		{
			// show a dialog to tune the weighting parameters
			CEnhancedDialog DLG2("Weights for sorting");
			int x_weight=10000; int y_weight=100; int z_weight=1;
			DLG2.StartGroupBox("Weighting");
				DLG2.addItem("weight for x-direction:",x_weight);
				DLG2.addItem("weight for y-direction:",y_weight);
				DLG2.addItem("weight for z-direction:",z_weight);
			DLG2.EndGroupBox();
			// show the dialog
			DLG2.DoModal();
		
			// sort the list
			pPoseList = AnalysisObjects::CPoseList::SortPoselist(pPoseList,x_weight,y_weight,z_weight);
		}

		// insert it in the global list
		analysisobjectscontainer.RetrieveData(*pPoseList,0,name,description);
		
		// delete the poselist again (because a copy of the real data is stored in the analysisobjectscontainer)
		delete pPoseList;
	
		// add the tree part for the pose object
		HTREEITEM hNewPose = InsertItem(name,145,145,FindTreeItemByID(tvitemPOSELISTOBJECTS_BRANCH),TVI_LAST);
		SetItemData(hNewPose,tvitemBEGIN_ANALYSISOBJECTS+analysisobjectscontainer.GetCurrentGlobalID()-1);
		Expand(FindTreeItemByID(tvitemPOSELISTOBJECTS_BRANCH),TVE_EXPAND);

	}
	else if (nID == pmeNCLoadPoseList)
	{
		// copy the selected pose list in the NC program
		
		// cast the selected object to the poselist
		PCRL::CPoseListKinetostatic* pPoseList = &((AnalysisObjects::CPoseList*)selected_object)->m_poselist;

		// alterative implementation: we perform a straight memory-to-memory transfer to copy the list
		CWireCenterView::This->PoseList.deleteAllPoses();
		// copy the poses to the pose list 
		for (auto itor = pPoseList->begin(); itor!=pPoseList->end(); ++itor)
			CWireCenterView::This->PoseList.push_back(new PCRL::CPoseKinetostatic((*itor)->r,(*itor)->R));
	}
	else if (nID == pmeNCInterpretePoseList)
	{
		// interprete the selected pose list in the NC program
		
		// cast the selected object to the poselist
		PCRL::CPoseListKinetostatic* pPoseList = &((AnalysisObjects::CPoseList*)selected_object)->m_poselist;

		// WORKAROUND: open file for NC-command output
		CString nc_prog_str;
		fstream filestr;
		filestr.open ("nc_output.nc", fstream::out);

		// declare the resulting parser string and fill some basic infos
		filestr << "#KIN ID [65]" << endl << "#TRAFO ON" << endl;

		// go through the list and append the nc commands to the file
		for (PCRL::CPoseListKinetostatic::iterator itor = pPoseList->begin(); itor != pPoseList->end(); itor++)
		{
			nc_prog_str.Format("G90 G01 W1=%i W2=%i W3=%i F60000 \n",(int)((*itor)->r.x()*1000),(int)((*itor)->r.y()*1000),(int)((*itor)->r.z()*1000));
			filestr << nc_prog_str;
		}

		// close the file
		filestr.close();

		// and load the program
		CWireCenterView::This->LoadNcProgram("nc_output.nc");
	}
	else if (nID == pmeAnalyzePoseList)
	{
		// analyze the selected pose list 
		
		// cast the selected object to the poselist
		PCRL::CPoseListKinetostatic* pPoseList = &((AnalysisObjects::CPoseList*)selected_object)->m_poselist;

		// show the analysis options here
		CWireCenterView::This->OnBerechnenAutomatischeposenanalyseoptionen();

		// clear the current pose list
		CWireCenterView::This->PoseList.deleteAllPoses();

		// COPY the poses
		for (PCRL::CPoseListKinetostatic::iterator itor = pPoseList->begin(); itor!=pPoseList->end(); ++itor)
		{
			CWireCenterView::This->PoseList.push_back(new PCRL::CPoseKinetostatic((*itor)->r,(*itor)->R));
		}

		// calculate the poses
		CWireCenterView::This->OnBerechnenTrajektorieanalysieren();
	}
	
	// redraw the scene
	pSceneObject->RenderScene();

	// redraw the treeView
	RedrawWindow();
	
	// return true to avoid message routing to other windows
	return true;
}


HTREEITEM CMenuTreeCtrl::FindTreeItemByID(int ID)
{
	// get the root item
	HTREEITEM current_item = GetRootItem();
	
	// while the current item is not NULL ...
	while (current_item!=NULL) 
	{
		// check if the current item is the wanted item
		if (GetItemData(current_item)==ID) 
			return current_item;
		// get the next item
		current_item = GetNextItem(current_item,TVGN_NEXT);
	}

	// if nothing was found, return NULL
	return NULL;
}

/////////////////////////////////////////////
// IMPLEMENTATION OF CShapeListViewPane
/////////////////////////////////////////////

CShapeListViewPane::CShapeListViewPane()
{
}

BEGIN_MESSAGE_MAP(CShapeListViewPane, CDockablePane)
	ON_WM_CREATE()
	ON_WM_SIZE()
END_MESSAGE_MAP()


void CShapeListViewPane::RecursiveEnumeration() 
{
	// delete the whole USEROBJECTS branch
	while (pTreeCtrl->GetChildItem(pTreeCtrl->FindTreeItemByID(tvitemUSEROBJECTS_BRANCH))!=NULL) 
		pTreeCtrl->DeleteItem(pTreeCtrl->GetChildItem(pTreeCtrl->FindTreeItemByID(tvitemUSEROBJECTS_BRANCH)));
	// delete the whole INTERNALOBJECTS branch
	while (pTreeCtrl->GetChildItem(pTreeCtrl->FindTreeItemByID(tvitemINTERNALOBJECTS_BRANCH))!=NULL) 
		pTreeCtrl->DeleteItem(pTreeCtrl->GetChildItem(pTreeCtrl->FindTreeItemByID(tvitemINTERNALOBJECTS_BRANCH)));
	// delete the whole ORIENTATIONS branch
	while (pTreeCtrl->GetChildItem(pTreeCtrl->FindTreeItemByID(tvitemORIENTATIONS_BRANCH))!=NULL) 
		pTreeCtrl->DeleteItem(pTreeCtrl->GetChildItem(pTreeCtrl->FindTreeItemByID(tvitemORIENTATIONS_BRANCH)));

	// get all current orientations
	// (CAREFUL: CURRENT ORIENTATIONS ONLY AFFECT THE HULL (like CWireCenterDoc::OnArbeitsraumOrientierungfestlegen()!)
	vector<Matrix3d> orientations = CWireCenterView::This->GetRobotDoc()->WSHull.getAllOrientations();
	int i=0;
	for (vector<Matrix3d>::iterator itor = orientations.begin(); itor != orientations.end(); itor++)
	{
		i++;
		// add the tree parts for all orientations
		CString str;
		str.Format("Orientations %i",i,(*itor));
		HTREEITEM hNewOrientation = pTreeCtrl->InsertItem(str, 134, 134, pTreeCtrl->FindTreeItemByID(tvitemORIENTATIONS_BRANCH), TVI_LAST);
		pTreeCtrl->SetItemData(hNewOrientation, tvitemBEGIN_ORIENTATIONS+i);
		// break, if too many
		if (i > 100)
		{
			AfxMessageBox("More than 100 orientations defined!\nRestricting the visalization to the first 100 orientations!");
			break;
		}
	}

	// and at least insert the shape elements again
	RecursiveEnumeration(pShapeList);
}


void CShapeListViewPane::RecursiveEnumeration(CGLShapeList* node)
{
	// every new call of this functions means a new sub catergory
	CString strShapeList;
	strShapeList.Format("Frame-ID: %i ; Layer: %i; Frame: %i",node->getID(),node->getLayer(),node->getFrame(node->getID())); 
	HTREEITEM hShapeListBranch = pTreeCtrl->InsertItem(strShapeList, 154, 154, pTreeCtrl->FindTreeItemByID(tvitemINTERNALOBJECTS_BRANCH), TVI_LAST);
	// save the value of ("it's ID" + tvitemBEGIN_INTERNALOBJECTS) in the data field to determine the selected object
	pTreeCtrl->SetItemData(hShapeListBranch, node->getID()+tvitemBEGIN_GRAPHICALOBJECTS);
	
	// go deeper in a recursive way
	for (list<CGLShapeList*>::iterator itor = node->childs.begin(); itor!=node->childs.end(); itor++)
	{
		RecursiveEnumeration((*itor));
	}
	
	// print content
	for (list<CGLShape*>::iterator itor = node->content.begin(); itor!=node->content.end(); itor++)
	{
		// add subitems for every shape
		CString strMerged;
		strMerged.Format("Object-ID: %i ; Name: %s ; Layer: %i ; Visibility: %s",(*itor)->getID(), "Unnamed" , (*itor)->getLayer() , (*itor)->bVisible ? "Yes" : "No");

		// place internal objects in the internal branch
		HTREEITEM hShapeBranch;
		if ((*itor)->getLayer()==-1)
			hShapeBranch = pTreeCtrl->InsertItem(strMerged, 45, 45, hShapeListBranch, TVI_SORT);
		else 
			hShapeBranch = pTreeCtrl->InsertItem(strMerged, 45, 45, pTreeCtrl->FindTreeItemByID(tvitemUSEROBJECTS_BRANCH), TVI_SORT);

		// save the value of ("it's ID" + tvitemBEGIN_INTERNALOBJECTS) in the data field to determine the selected object
		pTreeCtrl->SetItemData(hShapeBranch, (*itor)->getID()+tvitemBEGIN_GRAPHICALOBJECTS);
	}
}

CGLShape* CShapeListViewPane::RecursiveFind(CGLShapeList* node, int unique_id)
{
	// maybe this node is the wanted one...
	if (node->getID()==unique_id) 
		return node;

	// go deeper in a recursive way
	for (list<CGLShapeList*>::iterator itor = node->childs.begin(); itor!=node->childs.end(); ++itor)
	{
		// if item was found give me the adress
		if ((*itor)->getID()==unique_id) 
			return (*itor);
		// go recursive and give back the result, if the correct object was found
		CGLShape* result = RecursiveFind((*itor),unique_id);
		if (result) 
			return result;
	}
	
	// search for content
	for (list<CGLShape*>::iterator itor = node->content.begin(); itor!=node->content.end(); ++itor)
	{
		// if item was found give me the adress
		if ((*itor)->getID()==unique_id) 
			return (*itor);
	}

	// if nothing was found return NULL-pointer
	return NULL;
}

void CShapeListViewPane::AdjustLayout()
{
	if (GetSafeHwnd() == NULL) 
		return;
	CRect rectClient;
	GetClientRect(rectClient);
	pTreeCtrl->SetWindowPos(NULL,rectClient.left, 0, rectClient.Width(), rectClient.Height(), SWP_NOACTIVATE | SWP_NOZORDER);
}

int CShapeListViewPane::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CDockablePane::OnCreate(lpCreateStruct) == -1)
		return -1;

	CRect rect;

	// alter the standard font
	LOGFONT lf;
	afxGlobalData.fontRegular.GetLogFont(&lf);
	NONCLIENTMETRICS info;
	info.cbSize = sizeof(info);
	afxGlobalData.GetNonClientMetrics(info);
	lf.lfHeight = info.lfMenuFont.lfHeight;
	lf.lfWeight = info.lfMenuFont.lfWeight;
	lf.lfItalic = info.lfMenuFont.lfItalic;
	m_fntPropList.CreateFontIndirect(&lf);

	// create a new TreeControl which covers the whole Pane
	GetClientRect(rect);
	pTreeCtrl = new CMenuTreeCtrl;
	pTreeCtrl->Create(WS_VISIBLE | WS_TABSTOP | WS_CHILD | WS_BORDER | TVS_HASBUTTONS | TVS_LINESATROOT | TVS_HASLINES | TVS_DISABLEDRAGDROP, rect , this , IDC_SHAPELIST_TREECONTROL);

	// load the image list for using neat pictures and attach it to the TreeCtrl
	pImageList = new CImageList;
	CBitmap bmp;
	bmp.LoadBitmap(IDB_FILESMALL);
	pImageList->Create(16, 16, ILC_MASK | ILC_COLOR32, 0, 0);
	pImageList->Add(&bmp, RGB(255, 0, 255));
	pTreeCtrl->SetImageList(pImageList, TVSIL_NORMAL);

	// build the treeview for the first time
	BuildTreeView();
	
	// add the analysis objects container to the root in the shape list in wirecenterview
	TShapeAttribute SA; SA.iLayer = -1;
	CWireCenterView::This->m_Scene.addShape(pTreeCtrl->analysisobjectscontainer, &SA, false);

	return 0;
}

void CShapeListViewPane::OnSize(UINT nType, int cx, int cy)
{
	CDockablePane::OnSize(nType, cx, cy);
	AdjustLayout();
}

void CShapeListViewPane::BuildTreeView()
{
	// load the pointer to achive quicker access
	pSceneObject = &CWireCenterView::This->m_Scene;
	pShapeList = pSceneObject->GetHandle();

	// it's time to put some elements into the control, so delete all elements if remaining
	pTreeCtrl->DeleteAllItems();

	// add the tree part for the current orientations
	HTREEITEM hCurrentOrientations = pTreeCtrl->InsertItem(_T("Current Orientations"), 132, 132, NULL, NULL);
	pTreeCtrl->SetItemData(hCurrentOrientations, tvitemORIENTATIONS_BRANCH);

	// add the tree part for the path objects
	HTREEITEM hPathObjects = pTreeCtrl->InsertItem(_T("Pose Object"), 166, 166, NULL, NULL);
	pTreeCtrl->SetItemData(hPathObjects, tvitemPOSELISTOBJECTS_BRANCH);

	// add the tree part for HullObjects
	HTREEITEM hHullObjects = pTreeCtrl->InsertItem(_T("Hull Objects"), 162, 162, NULL, NULL);
	pTreeCtrl->SetItemData(hHullObjects, tvitemHULLOBJECTS_BRANCH);

	// add the tree part for the grid objects
	HTREEITEM hGridObjects = pTreeCtrl->InsertItem(_T("Grid Objects"), 164, 164, NULL, NULL);
	pTreeCtrl->SetItemData(hGridObjects, tvitemGRIDOBJECTS_BRANCH);

	// add the tree part for the crosssection objects
	HTREEITEM hContourObjects = pTreeCtrl->InsertItem(_T("Contour Objects"), 165, 165, NULL, NULL);
	pTreeCtrl->SetItemData(hContourObjects, tvitemCROSSSECTIONOBJECTS_BRANCH);

	// add the tree part for the user shape objects
	HTREEITEM hUserObjects = pTreeCtrl->InsertItem(_T("User Objects"), 47, 47, NULL, NULL);
	pTreeCtrl->SetItemData(hUserObjects, tvitemUSEROBJECTS_BRANCH);

	// add the tree part for the internal shape objects
	HTREEITEM hInternalObjects = pTreeCtrl->InsertItem(_T("Internal Shape Objects (Debug)"), 47, 47, NULL, NULL);
	pTreeCtrl->SetItemData(hInternalObjects, tvitemINTERNALOBJECTS_BRANCH);

	// parse recursivly the ShapeList class from the m_scene instance in WireCenterView and fill the tree
	RecursiveEnumeration();

	// generate the layout
	AdjustLayout();
}
