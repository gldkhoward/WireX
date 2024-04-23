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

/*!*******************************************************************
 *  \file   : IPC.cpp
 *
 *  Project   WireCenter
 *
 *  \Author   Philipp Miermeister
 *
 *  \Date     05.08.2011 
 *
 *  (c)       Fraunhofer IPA
 *
 *********************************************************************
 */ 

#include "stdafx.h"
#include "WireCenterDoc.h"
#include "WireCenterView.h"
#include "IPC.h"
#include "PythonInterface.h"
#include <afxsock.h>
#include "motionPlanning\Utilities.h"
#include "WireLib\RobotData.h"

//----------------------------------------------
// Socket base class
//----------------------------------------------


CIPC_SocketBase::CIPC_SocketBase()
{
}

CIPC_SocketBase::~CIPC_SocketBase()
{
}

void CIPC_SocketBase::sendData(const char* pData, const int &msgSize)
{
	// pData: Contains the adress of the data to be sent 
	// here it is adressed by a char pointer but the pointer could be any data type
	// msgSize: The size of the data array in bytes

	// Send data to the client
	int numelTot = 0;
	int numel;
	int err;

	//double* pDouble = (double*)pData;
	//double dData1 = pDouble[1];

	while(numelTot < msgSize)
	{
		numel = this->Send(pData+numelTot,msgSize-numelTot);
		switch(numel)
		{
		case 0: // client has closed the connection
			this->Close(); // simply close this socket
			break;
		case SOCKET_ERROR:
			err = this->GetLastError();
			// todo: error handling
			break;
		}
		numelTot += numel;
	}

}


//----------------------------------------------
// Receiver class methods
//----------------------------------------------

// Constructor
CIPC_SocketRecv::CIPC_SocketRecv(CWireCenterView* pWireCenterView)
{
	//m_pObject = pObject;
	//m_pMessageParser = pMessageParser;
	m_pWireCenterView = pWireCenterView;
	m_userInformationState = USER_INFO_NORMAL;
	m_bufferSize  = 100000;
	m_buffer.reserve(m_bufferSize); // allocate memory
	m_buffer.resize(m_bufferSize,0); // initialize elements (needed for [] operator)
	m_wIndex = 0;
	m_rIndex = 0;
	m_clientClosed = false;

}

// Destructor
CIPC_SocketRecv::~CIPC_SocketRecv()
{
}


void CIPC_SocketRecv::OnReceive(int nErrorCode)
{
	//char* pLocalBuffer = new char[4]; // reserve memory for local buffer

	//vector<char> vBuffer(1024,0);

	int numel = 0;
	int err = 0;


	numel = this->Receive(&m_buffer[m_wIndex], m_bufferSize - m_wIndex);

	switch(numel)
	{
	case 0: 
		this->Close();
		printf("Connection closed by the client.\n");
		break;
	case SOCKET_ERROR:
		err = this->GetLastError();
		this->Close();
		printf("Error: Lost data.\n");
		break;
	default:
		// the first element contains the size of the message excluding the first element itself
		int msgSize = *(int*)(&m_buffer[0]);
		
		// Adjust buffer size if necessary
		if (msgSize + 4 > m_bufferSize)
		{
			int oldBufferSize = m_bufferSize;		
			m_bufferSize = msgSize * 2; //double bufferSize
			m_buffer.reserve(m_bufferSize); // increase buffer size (all pointers to the buffer must be renewed) 
			m_buffer.resize(m_bufferSize,0); // initialize new elements (needed for [] operator)
			if(m_userInformationState == USER_INFO_ALL)
			{
				printf("Message size exceeds buffer size.\n");
				printf("Buffer size was changed from %d bytes to %d bytes.\n", oldBufferSize, m_bufferSize);
			}

		}

		m_wIndex += numel;

		// check if at least one message was completely transmitted 
		if (m_wIndex >= msgSize+4)
		{	 
			messageParser();
		}

	}

	// Close the client if the state variable m_clientClosed is set to TRUE
	// and no further data is available
	DWORD remainingNumel = 0;
	if(m_clientClosed)
	{
		this->IOCtl(FIONREAD,&remainingNumel);

		if (remainingNumel == 0)
		{
			this->Close();
		}
	}

	CAsyncSocket::OnReceive(nErrorCode);
	//delete pLocalBuffer;
}




// The readElement() method returns a copy of the buffered element
template <class T>
void CIPC_SocketRecv::readElement(T &x_Out, int &rIndex_InOut)
{
	x_Out = ((T*)(&m_buffer[rIndex_InOut]))[0]; 
	rIndex_InOut += sizeof(T);
}
template <class T>
void CIPC_SocketRecv::readElement(T*& px_Out, const int &size, int &rIndex_InOut)
{
	int numBytes = sizeof(T) * size;
	px_Out = new T[size];	
	memcpy(px_Out, &m_buffer[rIndex_InOut], numBytes); 
	rIndex_InOut += numBytes; //bytes * size
}


void CIPC_SocketRecv::readElement(CVector3 &x_Out, int &rIndex_InOut)
{
	double* pDouble = (double*)(&m_buffer[rIndex_InOut]); 
	x_Out = CVector3(pDouble[0],pDouble[1],pDouble[2]); // Copy new data to x_Out
	rIndex_InOut += 3*sizeof(double); //24bytes
}

void CIPC_SocketRecv::readElement(Vector3d &x_Out, int &rIndex_InOut)
{
	double* pDouble = (double*)(&m_buffer[rIndex_InOut]); 
	x_Out = Vector3d(pDouble[0],pDouble[1],pDouble[2]);
	rIndex_InOut += 3*sizeof(double); //24bytes
}

void CIPC_SocketRecv::readElement(Matrix3d &x_Out, int &rIndex_InOut)
{
	Vector3d a,b,c;
	this->readElement(a,rIndex_InOut);
	this->readElement(b,rIndex_InOut);
	this->readElement(c,rIndex_InOut);
		
	x_Out << a ,b, c; // column wise matrix construction
}


void CIPC_SocketRecv::readElement(string &x_Out, int &rIndex_InOut)
{	
	// the string element consists of two elements namely
	// the string length (int32) and the string data itself.
	int stringLength = ((int*)(&m_buffer[rIndex_InOut]))[0]; // read string size
	rIndex_InOut += sizeof(int); //4bytes

	char* pChar = ((char*)(&m_buffer[rIndex_InOut])); // read the string data
	x_Out = std::string(pChar,stringLength); // assumes a string with fixed length and without null termination
	rIndex_InOut += stringLength;
}

template <class T>
void CIPC_SocketRecv::sendElement(const T &x)
{
	this->sendData((char*)&x,sizeof(T)); //1byte
}

// the pointer type needs some information about the number of elements
template <class T>
void CIPC_SocketRecv::sendElement(T* px, const int &size)
{
	this->sendData((char*)px,sizeof(T) * size); //1byte * size
}

// special case for strings
void CIPC_SocketRecv::sendElement(const string &s)
{
	this->sendData(s.c_str(),s.size()); //24bytes
}


void CIPC_SocketRecv::sendElement(const CVector3 &v)
{
	double result[3] = {v.x, v.y, v.z};
	this->sendData((char*)&result,3*sizeof(double)); //24bytes
}

void CIPC_SocketRecv::sendElement(const Vector3d &v)
{
	double result[3] = {v.x(), v.y(), v.z()};
	this->sendData((char*)&result,3*sizeof(double)); //24bytes
}

void CIPC_SocketRecv::sendElement(const Matrix3d &v)
{
	double result[9] = {v(0,0), v(1,0), v(2,0), v(0,1), v(1,1), v(2,1), v(0,2), v(1,2), v(2,2)};
	this->sendData((char*)&result,9*sizeof(double)); //72bytes
}

void CIPC_SocketRecv::sendElement(const MatrixXd &v)
{
	this->sendElement(v.data(), v.size()); // columnwise layout is expected (this is the default setting of Eigen3) 
}


void CIPC_SocketRecv::OnClose(int nErrorCode)
{

	printf("Connection closed by the client.\n");

	DWORD remainingNumel = 0;
	this->IOCtl(FIONREAD,&remainingNumel); // return remaining elements

	// Check if data is still available
	if (remainingNumel == 0) 
	{
		m_clientClosed = true;
		this->Close();		   // set state variable and close socket immediately
	}
	else
	{
		m_clientClosed = true; // set state variable (socket will close after it has received all data)
	}

	CAsyncSocket::OnClose(nErrorCode);
}

// This method is used by the caller class to get the socket state
bool CIPC_SocketRecv::isClosed()
{
	return m_clientClosed;
}




//----------------------------------------------
// Socket class methods
//----------------------------------------------


CIPC_Socket::CIPC_Socket(CWireCenterView* pWireCenterView): CAsyncSocket()
{
	m_pWireCenterView = pWireCenterView;
	m_pSocketRecv = NULL;
}

CIPC_Socket::~CIPC_Socket()
{
	delete m_pSocketRecv; 
	m_pSocketRecv = NULL;
}

bool CIPC_Socket::initialize()
{
	unsigned int socketPort = 9050;

	this->Create(socketPort);

	if(Listen()==FALSE) 
	{ 
		printf("Unable to listen to port %d\n",socketPort); 
		printf("New state: Connection closed\n"); 
		this->Close();
		return false;
	}else
	{
		printf("Server listens to port %d\n",socketPort); 
	}

	return true;

}


void CIPC_Socket::OnAccept(int nErrorCode)
{
//	int numel;
	int numelTot;
//	int err;
	const int RESP_SIZE = 1;
	char response[RESP_SIZE]; // 1 extra element for the terminating null character
	CString strIP; 
	unsigned int socketPort; 



	// Create new receiver socket if no socket is connected
	// or if the socket is already closed
	if(m_pSocketRecv == NULL || m_pSocketRecv->isClosed())
	{

		delete m_pSocketRecv; // delete closed socket

		m_pSocketRecv = new CIPC_SocketRecv(m_pWireCenterView);

		// Accept new socket for receiving data
		// The original socket stays open and listens for further connections
		if(Accept(*m_pSocketRecv))
		{
			m_pSocketRecv->GetSockName(strIP,socketPort);
			printf("Connection established\n");
			printf("Client IP: " + strIP + "Port: %d\n", socketPort);

			// Send a response to the client
			numelTot = 0;
			//strcpy_s(response,"s_ok");
			response[0] = 1;
			m_pSocketRecv->sendData(response,RESP_SIZE);
		}
		else
		{
			// todo: error handling
		}
	}
	else // socket object is already existing
	{
		// create a second socket which is used to turn the client down
		CIPC_SocketBase socket;
		numelTot = 0;
		//strcpy_s(response,"busy");
		response[0] = -1;
		if(Accept(socket))
		{	
			socket.sendData(response,RESP_SIZE);

			// close socket after sending the response
			socket.Close();
		}	
		else
		{
			//todo: error handling
		}
	}
	CAsyncSocket::OnAccept(nErrorCode);
}

void CIPC_Socket::OnConnect(int nErrorCode)
{
	CAsyncSocket::OnConnect(nErrorCode);
}

void CIPC_Socket::OnOutOfBandData(int nErrorCode)
{
	CAsyncSocket::OnOutOfBandData(nErrorCode);
}

void CIPC_Socket::OnSend(int nErrorCode)
{
	CAsyncSocket::OnSend(nErrorCode);
}




//----------------------------------------------
// IPC class methods
//----------------------------------------------


CIPC::CIPC(CWireCenterView* pWireCenterView)
{
	m_pSocket = new CIPC_Socket(pWireCenterView); // create m_pSocket member
}

CIPC::~CIPC()
{
	delete m_pSocket; // delete m_pSocket member
}


void CIPC::startServer() 
{	
	m_pSocket->initialize();

	//printf("Thread started\n");
	//HANDLE h_waitForClient_Thread = 0;
	//int i = 0;
	//h_waitForClient_Thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)waitForClient,  (LPVOID) i, 0,NULL);
}






/******************************************************************************************************************
****																										   ****
****                                                Message Parser                                             **** 
****																										   ****
*******************************************************************************************************************/

/* All methods that shell be made available in a third party software have to be added to the message parser.
   How to make a wireCenter method available:
   1. Choose a new id.
   2. Define all variables used by the method.
   3. Call the this->readElement(var_receive, rIndex) method for each variable in the same order as they are received.
   4. Pass the variables to the wireCenter method.
   5. Call the this->sendElement(var_send) method for each variable in the same order as they are sent.

   The following data types can be used with the parser method:
   - All built in data types like bool, char, int, double...
   - CVector3
   - Vector3d
   - Matrix3d
   - strings
*/

void CIPC_SocketRecv::messageParser()
{	
	int rIndex = 0; // index for reading the buffer
	int msgSize;
	int flag;

	PCRL::CWorkspaceDifferentialHull* pWSHull = &m_pWireCenterView->GetRobotDoc()->WSHull;
	PCRL::CForceDistribution* pFD = &m_pWireCenterView->GetRobotDoc()->ForceDistribution;
	PCRL::CWorkspaceGrid* pGrid = &m_pWireCenterView->GetRobotDoc()->WSGrid;
	PCRL::CWorkspaceCrosssection* pCross = &m_pWireCenterView->GetRobotDoc()->Crosssection;
	PCRL::CElastoKinematics* pKinematics = &m_pWireCenterView->GetRobotDoc()->Kinematics;	
	PCRL::CStiffness* pStiffness = &m_pWireCenterView->GetRobotDoc()->Stiffness;
	PCRL::CInterference* pInterference = &m_pWireCenterView->GetRobotDoc()->Interference;		
	PCRL::CRobotData* pRobot = m_pWireCenterView->GetRobotDoc();
	CWireCenterDoc* pDoc = m_pWireCenterView->GetDocument();
	CPathGenerator* pPG = &m_pWireCenterView->PathGenerator;
//	CGLScene* pScene = &m_pWireCenterView->m_Scene;
	CGLSceneGraph* pScene = &m_pWireCenterView->m_Scene;

	this->readElement(msgSize,rIndex); // read message size

	while(rIndex + msgSize  <= m_wIndex) 
	{	

		//flag
		this->readElement(flag,rIndex);

		switch(flag)
		{
		case 001: // Call python methods
			{
				std::string cmdStr;
				this->readElement(cmdStr, rIndex);	
				CPythonInterface::getInstance().runCommand(cmdStr);
				bool result = true;
				this->sendElement(result);
				break;
			}
		case 002: // Marshal python methods
			{
				std::string strObj_in, strObj_out;
				this->readElement(strObj_in, rIndex);	
				CPythonInterface::getInstance().marshal(strObj_in, strObj_out);
				this->sendElement(strObj_out.length()); // send number of elements
				this->sendElement(strObj_out); // send serialized python object
				break;
			}


			//--------------------------------------
			// Irobot methods
			//--------------------------------------
		case 100: // compute optimal cone
			{	
				int id;
				Vector3d axis_out, apex_out;
				double aperture_out;

				this->readElement(id, rIndex);		
				pWSHull->calculateOptimalCone(id, axis_out, aperture_out);
				apex_out = pRobot->getBase(id);
				this->sendElement(apex_out);
				this->sendElement(axis_out);
				this->sendElement(aperture_out);
				break;
			}

		case 101: // compute disabled workspace
			{	
				Vector3d boxMin, boxMax;

				this->readElement(boxMin, rIndex);
				this->readElement(boxMax, rIndex);
				pWSHull->calcDisabledWspc(boxMin,boxMax);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 102: // compute workspace
			{	
				bool result = pWSHull->calculateWorkspace();
				this->sendElement(result);
				break;
			}

		case 103: // intersect workspace
			{	
				bool result = pWSHull->intersectWorkspace();
				this->sendElement(result);
				break;
			}

		case 104: // unite workspace
			{	
				bool result = pWSHull->uniteWorkspace();
				this->sendElement(result);
				break;
			}

		case 105: // calculate workspace cross section
			{	
				char normal;
				this->readElement(normal, rIndex);
				pCross->calculateWorkspaceCrosssection(normal);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 106: // calculate statistics
			{	
				pWSHull->calculateWorkspaceProperties();
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 107: // print statistics
			{	
				pWSHull->printProperties();
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 108: // set iterations
			{	
				int iterations;
				this->readElement(iterations, rIndex);
				pWSHull->setIterations(iterations);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 109: // set eps
			{	
				double eps;
				this->readElement(eps, rIndex);
				pWSHull->setEps(eps);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 110: // set range for line search
			{	
				double range;
				this->readElement(range, rIndex);
				pWSHull->setSearchRange(range);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 111: // set condition
			{	
				double min;
				this->readElement(min, rIndex);
				pFD->setConditionMin(min);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 112: // save workspace
			{	
				std::string fileName;
				this->readElement(fileName, rIndex);
				bool result = pWSHull->saveWorkspace(fileName);
				this->sendElement(result);
				break;
			}

		case 113: // verfiy workspace box
			{	
				Vector3d min, max, eps;
				this->readElement(min, rIndex);
				this->readElement(max, rIndex);
				this->readElement(eps, rIndex);
				bool result = pGrid->verifyWorkspaceBox(min,max,eps);
				this->sendElement(result);
				break;
			}

		case 114: // set wrench
			{	
				Vector3d force,torque;
				this->readElement(force, rIndex);
				this->readElement(torque, rIndex);
				pFD->setWrench(force,torque);
				bool result = true;
				this->sendElement(result);
				break;
			}


		case 115: // load robot geometry
			{	
				std::string fileName;
				this->readElement(fileName, rIndex);
				bool result = pRobot->Load(fileName);
				this->sendElement(result);
				break;
			}

		case 116: // save robot geometry
			{	
				std::string fileName;
				this->readElement(fileName, rIndex);
				bool result = pRobot->Save(fileName);
				this->sendElement(result);
				break;
			}

		case 117: // save robot geometry as xml
			{	
				std::string fileName;
				this->readElement(fileName, rIndex);
				bool result = pDoc->robotDoc.saveXml(fileName);
				this->sendElement(result);
				break;
			}
			
		case 118: // save cross section
			{	
				std::string fileName;
				char normal;
				this->readElement(fileName, rIndex);
				this->readElement(normal, rIndex);
				bool result = pCross->saveWorkspaceCrosssection(fileName,normal);
				this->sendElement(result);
				break;
			}

		case 119: // set the geoemtry of one leg's base
			{	
				int i;
				Vector3d v;

				this->readElement(i, rIndex);
				this->readElement(v, rIndex);
				pRobot->setBase(i,v);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 120: // set the geoemtry of one leg's platform
			{	
				int i;
				Vector3d v;

				this->readElement(i, rIndex);
				this->readElement(v, rIndex);
				pRobot->setPlatform(i,v);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 121: // set the motion pattern of the robot (number of cables, motion type) 
			{	
				int numWires, motionPattern;
				PCRL::CRobotData::MotionPatternType MPT;

				this->readElement(numWires, rIndex);
				this->readElement(motionPattern, rIndex);

				switch (motionPattern)
				{
				case 0: MPT=PCRL::CRobotData::MP1T; break;
				case 1: MPT=PCRL::CRobotData::MP2T; break;
				case 2: MPT=PCRL::CRobotData::MP3T; break;
				case 3: MPT=PCRL::CRobotData::MP1R2T; break;
				case 4: MPT=PCRL::CRobotData::MP2R3T; break;
				default:
				case 5: MPT=PCRL::CRobotData::MP3R3T; break;
				}

				pRobot->setMotionPattern(numWires,MPT);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 122: // set force limits
			{	
				double fmin, fmax;

				this->readElement(fmin, rIndex);
				this->readElement(fmax, rIndex);
				pRobot->setForceLimits(fmin,fmax);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 123: // set wire length
			{	
				double lmin, lmax;

				this->readElement(lmin, rIndex);
				this->readElement(lmax, rIndex);
				pRobot->setWireLength(lmin,lmax);
				bool result = true;
				this->sendElement(result);
				break;
			}


		case 124: // getWireRangeForBoxDriver
			{	
				Vector3d min, max;
				this->readElement(min, rIndex);
				this->readElement(max, rIndex);
				bool result = pKinematics->getWireRangeForBoxDriver(min,max);
				this->sendElement(result);
				break;
			}

		case 125: // wireForcesWorkspaceBoxDriver
			{	
				Vector3d min, max, eps;
				this->readElement(min, rIndex);
				this->readElement(max, rIndex);
				this->readElement(eps, rIndex);
				bool result = pGrid->wireForcesWorkspaceBoxDriver(min,max,eps);
				this->sendElement(result);
				break;
			}

		case 126: // getWinchAperture
			{	
				int i;
				Vector3d min, max, u;
				this->readElement(i, rIndex);
				this->readElement(min, rIndex);
				this->readElement(max, rIndex);
				this->readElement(u, rIndex);
				double result = pWSHull->getWinchAperture(i,min,max,u);
				this->sendElement(result);
				break;
			}

		case 127: // get stiffness matrix
			{	
				Vector3d r;
				Matrix3d R=Matrix3d::ZRotationMatrix3d(0.0);

				this->readElement(r, rIndex);
				bool result = pStiffness->StiffnessMatrix(r,R);
				this->sendElement(result);
				break;
			}

		case 128: // get translational stiffness matrix
			{	
				Vector3d r;
				Matrix3d R=Matrix3d::ZRotationMatrix3d(0.0);

				this->readElement(r, rIndex);
				bool result = pStiffness->StiffnessMatrixTranslational(r,R);
				this->sendElement(result);
				break;
			}

		case 129: // get minimal stiffness
			{	
				double result = pStiffness->getMinimalStiffness();
				this->sendElement(result);
				break;
			}

		case 130: // setInterferenceOrientationWorkspace
			{	
				double* pArg=0;
				this->readElement(pArg,6, rIndex);
				//alpha_min,alpha_max,delta_alpha,beta_min,beta_max,delta_beta
				pInterference->setOrientationWorkspace(pArg[0],pArg[1],pArg[2],pArg[3],pArg[4],pArg[5]);
				bool result = true;
				this->sendElement(result);
				delete [] pArg;
				break;
			}

		case 131: // setInterferenceClippingBox
			{	
				double* pArg=0;
				this->readElement(pArg,6, rIndex);

				//xmin, xmax, ymin, ymax, zmin, zmax
				pInterference->setClippingBox(pArg[0],pArg[1],pArg[2],pArg[3],pArg[4],pArg[5]);
				bool result = true;
				this->sendElement(result);
				delete [] pArg;
				break;
			}

		case 132: // set platform position
			{	
				Vector3d r;
				this->readElement(r, rIndex);
				pDoc->robotDoc.r = r;
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 133: // print force distribution
			{	
				Vector3d v;
				this->readElement(v, rIndex);
				bool result = pFD->printForceDistribution(v);
				this->sendElement(result);
				break;
			}

		case 134: // calculate differential workspace
			{	
				bool result = pWSHull->calculateWorkspaceDiff();
				this->sendElement(result);
				break;
			}


		case 135: // print differential
			{	
				pWSHull->printDifferential();
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 136: // print zero length
			{	
				pKinematics->printZeroLength();
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 137: // determine the cable length for all cables
			{	
				Vector3d r;
				Matrix3d R = Matrix3d::ZRotationMatrix3d(0);		 
				this->readElement(r, rIndex);
				int m = pRobot->getNow(); // number of cables
				MatrixXd pl(m,1);
				pKinematics->doInverseKinematics(r,R,pl);
				this->sendElement(pl.data(),m);
				break;
			}

		case 138: // set the method for force calculation and evaluation
			{	
				int method, evaluation;
				this->readElement(method, rIndex);
				this->readElement(evaluation, rIndex);
				pFD->setMethod(method,evaluation);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 139: // set the orientation for workspace hull calculation 
			{	
				double* pAngle=0;
				this->readElement(pAngle, 3 , rIndex);
				Matrix3d R = Matrix3d::ZRotationMatrix3d(pAngle[2])*Matrix3d::YRotationMatrix3d(pAngle[1])*Matrix3d::XRotationMatrix3d(pAngle[0]);
				pWSHull->setOrientation(R);
				bool result = true;
				this->sendElement(result);
				delete [] pAngle;
				break;
			}

		case 140: // add orientation for workspace hull calculation 
			{	
				double* pAngle=0;
				this->readElement(pAngle, 3 , rIndex);
				Matrix3d R = Matrix3d::ZRotationMatrix3d(pAngle[2])*Matrix3d::YRotationMatrix3d(pAngle[1])*Matrix3d::XRotationMatrix3d(pAngle[0]);
				pWSHull->addOrientation(R);
				bool result = true;
				this->sendElement(result);
				delete [] pAngle;
				break;
			}

		case 141: // create orientation set for workspace hull calculation
			{	
				double* pDAngle=0;
				int steps;
				this->readElement(pDAngle, 3 , rIndex);
				this->readElement(steps , rIndex);
				bool result = pWSHull->createOrientationSet(pDAngle[0], pDAngle[1], pDAngle[2], steps);
				this->sendElement(result);
				delete [] pDAngle;
				break;
			}

		case 142: // setOrientationRequirement
			{	
				bool selector;
				this->readElement(selector , rIndex);
				pWSHull->setOrientationRequirement(selector);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 143: // get the time for a trajectory segment 
			{	
				double *pArg=0;
				this->readElement(pArg, 4, rIndex);
				//ds, vmax, amax, rmax
				double dt=PCRL::getTrajectoryTime(pArg[0],pArg[1],pArg[2],pArg[3]);
				this->sendElement(dt);
				delete [] pArg;
				break;
			}

		case 144: // set platform orientation
			{	
				Matrix3d R;
				this->readElement(R, rIndex);
				pDoc->robotDoc.R = R;
				bool result = true;
				this->sendElement(result);
				break;
			}


		case 145: // compute elasto geometrical forward kinematics
			{	
				double* pl=0;
				Vector3d r, rot_xyz;
				int itermax;
				
				this->readElement(pl,pRobot->getNow(), rIndex);
				this->readElement(r, rIndex);
				this->readElement(rot_xyz, rIndex);
				
				Eigen::Map<MatrixXd> l_nominal(pl, 1, pRobot->getNow()); // create map in order to reuse cable length vector in Eigen
				MatrixXd f(1,pRobot->getNow());
				this->readElement(itermax, rIndex);

				pKinematics->doElastoGeometricalForwardKinematics(l_nominal,r,rot_xyz,f, itermax);

				// send results from forward kinematics (the robot state model is not updatet, this has to be done with MATLAB)
				this->sendElement(r);
				this->sendElement(rot_xyz);
				this->sendElement(f);

				delete [] pl;

				break;
			}
			//--------------------------------------
			// Iapp methods
			//--------------------------------------


		//case 146: // Run inverse kinematics main
  //      {
  //              Vector3d r;
  //              Matrix3d R;
  //              double* l = new double[pDoc->robotDoc.getNow()]; // create a cable vector m_pWireCenterDoc->robotDoc.getNow()

  //              // Read values from Matlab
  //              this->readElement(r, rIndex);
  //              this->readElement(R, rIndex);

  //              pKinematics->doInverseKinematicsMain(r,R,l);	// Run doInverseKinematicsMain
  //              this->sendElement(l,pDoc->robotDoc.getNow());							// Send values to Matlab
		//		delete[] l;
  //              break;
  //       }

		case 147: // Return current robot state
        {
                // Send values to Matlab
                this->sendElement(pDoc->robotDoc.r);
				this->sendElement(pDoc->robotDoc.R);  
				this->sendElement(pDoc->robotDoc.F.data(), pDoc->robotDoc.getNow());  
				this->sendElement(pDoc->robotDoc.w.data(), 6);  
                break;
		}

		case 148: // set applied platform wrench w
			{	
				double* pw=0;
				this->readElement(pw,6, rIndex);
				memcpy(pDoc->robotDoc.w.data(),pw, sizeof(double) * 6);
				bool result = true;
				this->sendElement(result);

				delete [] pw;
				break;
			}

		case 149: // set cable forces
			{	
				double* pf=0;
				this->readElement(pf,pDoc->robotDoc.getNow(), rIndex);
				memcpy(pDoc->robotDoc.F.data(),pf, sizeof(double) * pDoc->robotDoc.getNow());
				bool result = true;
				this->sendElement(result);

				delete [] pf;
				break;
			}

		case 150: // set model parameters
			{	
				int now;
				now = pDoc->robotDoc.getNow();
				Vector3d ai;
				for (int i=0; i<now; i++)
				{
					this->readElement(ai, rIndex);
					pDoc->robotDoc.setBase(i,ai);
				}

				Vector3d bi;
				for (int i=0; i<now; i++)
				{
					this->readElement(bi, rIndex);
					pDoc->robotDoc.setPlatform(i,bi);
				}

				this->readElement(pDoc->robotDoc.platform_mass, rIndex); 

				for (int i=0; i<now; i++)
				{
					this->readElement(pDoc->robotDoc.pCable[i].k_spec, rIndex);
				}

				for (int i=0; i<now; i++)
				{
					this->readElement(pDoc->robotDoc.pCable[i].weight, rIndex);
				}
				
				Vector3d constraint; // 

				for (int i = 0; i<now; i++)
				{
					this->readElement(pDoc->robotDoc.pCable[i].nContraints, rIndex); // read number of constraints for each cable
				}

				for (int i = 0; i<now; i++)
				{
					int nConstraints = pDoc->robotDoc.pCable[i].nContraints;
					pDoc->robotDoc.pCable[i].pConstraints = new Vector3d[nConstraints]; // allocate memory for each cable (memory deallocation is handled by the object)

					// add all constraints for the selected cable
					for (int j=0; j<nConstraints; j++)
					{
						this->readElement(constraint, rIndex);
						pDoc->robotDoc.pCable[i].pConstraints[j] = constraint;
					}
				}

				bool result = true;
				this->sendElement(result);
				break;
			}


		case 151: // get model parameters
			{	
				int now = pDoc->robotDoc.getNow();
				Vector3d ai;
				for (int i=0; i<now; i++)
				{
					this->sendElement(pDoc->robotDoc.getBase(i));
				}

				Vector3d bi;
				for (int i=0; i<now; i++)
				{
					this->sendElement(pDoc->robotDoc.getPlatform(i));
				}

				this->sendElement(pDoc->robotDoc.platform_mass); 

				for (int i=0; i<now; i++)
				{
					this->sendElement(pDoc->robotDoc.pCable[i].k_spec);
				}
				break;
			}


		case 152: // set platform rotation (axis angle, parametrization)
			{	
				Matrix3d R;
				Vector3d axis;
				double angle;

				this->readElement(axis, rIndex);
				this->readElement(angle, rIndex);

				PCRL::getMatrixFromAxisAngle(R, angle, axis.x(), axis.y(), axis.z());
				pDoc->robotDoc.R = R;

				bool result = true;
				this->sendElement(result);
				break;
			}

		case 153: // set platform rotation (xyz)
			{	
				Matrix3d R;
				Vector3d abc;

				this->readElement(abc, rIndex);

				PCRL::getMatrixFromXYZ(R,abc);
				pDoc->robotDoc.R = R;

				bool result = true;
				this->sendElement(result);
				break;
			}

		case 154: // set cable length
			{
				for (int i=0; i < pDoc->robotDoc.getNow(); i++)
				{
					this->readElement(pDoc->robotDoc.l(i),rIndex);
				}

				bool result = true;
				this->sendElement(result);
				break;
			}

		case 155: // define robot kinematics model
			{
				int kinType;
				this->readElement(kinType, rIndex);
				pDoc->robotDoc.setRobotKinematicsModel((PCRL::CRobotData::RobotKinematicsType)kinType); // cast int to enum
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 156: // define elasticity model
			{
				int elasticityType;
				this->readElement(elasticityType, rIndex);
				pDoc->robotDoc.setElasticityModel((PCRL::CRobotData::ElasticityModelType)elasticityType); // cast int to enum
				bool result = true;
				this->sendElement(result);
				break;
			}
			
		case 157: // compute extended inverse kinematics with pulleys
			{

				Vector3d r;
                Matrix3d R;
				double* lf = new double[pDoc->robotDoc.getNow()]; // free cable length
                double* l = new double[pDoc->robotDoc.getNow()];
				double* beta = new double[pDoc->robotDoc.getNow()];
				double* gamma = new double[pDoc->robotDoc.getNow()];
				Vector3d* C = new Vector3d[pDoc->robotDoc.getNow()];
				Vector3d* U = new Vector3d[pDoc->robotDoc.getNow()];
				double* Cv = new double[pDoc->robotDoc.getNow()*3];
				double* Uv = new double[pDoc->robotDoc.getNow()*3];
				

                // Read values from Matlab
                this->readElement(r, rIndex);
                this->readElement(R, rIndex);
				
                pKinematics->doInverseKinematicsPulleyEx(r,R,l,lf,beta,gamma,C,U);	

				// store vectors in one array
				for(int i=0; i<pDoc->robotDoc.getNow(); i++)
				{
					memcpy(&Cv[i*3],C[i].data(), sizeof(double)*3);
					memcpy(&Uv[i*3],U[i].data(), sizeof(double)*3);
				}


				// Send values to Matlab
				this->sendElement(pDoc->robotDoc.getNow());	
                this->sendElement(lf,pDoc->robotDoc.getNow());

				this->sendElement(pDoc->robotDoc.getNow());	
                this->sendElement(l,pDoc->robotDoc.getNow());

				this->sendElement(pDoc->robotDoc.getNow());	
				this->sendElement(beta,pDoc->robotDoc.getNow());

				this->sendElement(pDoc->robotDoc.getNow());	
				this->sendElement(gamma,pDoc->robotDoc.getNow());

				this->sendElement(pDoc->robotDoc.getNow()*3);	
				this->sendElement(Cv,pDoc->robotDoc.getNow()*3);

				this->sendElement(pDoc->robotDoc.getNow()*3);	
				this->sendElement(Uv,pDoc->robotDoc.getNow()*3);
				

				delete[] lf, l,beta,gamma,C,U, Cv, Uv;
                break;
			}

		case 158: // get levmar info
			{
				double* pInfo = new double[LM_INFO_SZ];
		
				pKinematics->getLevmarInfo(pInfo);
				this->sendElement(pInfo,LM_INFO_SZ);
				break;
			}

		case 159: // set number of wires
			{
				int now;
				this->readElement(now, rIndex);
				pRobot->SetNow(now);
				bool result = true;
				this->sendElement(result);
				break;
			}


		case 160: // compute catenary 
			{

				int nCons; // number of constraints
				double mpu; // mass per unit length
				double k_spec;
				double l_nominal;
				int simMode;
				int steps;
				
				this->readElement(nCons, rIndex);
				this->readElement(mpu, rIndex);
				this->readElement(k_spec, rIndex);
				this->readElement(l_nominal, rIndex);
				this->readElement(simMode, rIndex);
				this->readElement(steps, rIndex);

				Vector3d* pCons = new Vector3d[nCons];
				Vector3d* samplePoints = new Vector3d[steps];

				for(int i=0; i<nCons; i++)
				{
					this->readElement(pCons[i], rIndex);
				}
				double f_ini = 1000;
				PCRL::CCableChain cableChain(&nCons,mpu, k_spec, f_ini);
				cableChain.set_Parameters(pCons,&l_nominal,simMode);
				
				// compute anchorage forces of all catenaries of one cable
				Vector3d* pu = new Vector3d[(nCons-1)*2];
				double* pf = new double[(nCons-1)*2];

				for(int i=0; i<nCons-1; i+=2)
				{
					cableChain.compute_AnchorageForces(i, pu[i], pu[i+1], pf[i], pf[i+1]);
				}
				
				// -----------------------------------------------------------------------
				// send u1 vectors
				this->sendElement((nCons-1)*3); // send number of elements (3x1 vectors)
				for(int i=0; i<(nCons-1)*2; i+=2)
				{
					this->sendElement(pu[i]);
				}

				// send u2 vectors
				this->sendElement((nCons-1)*3); // send number of elements
				for(int i=1; i<(nCons-1)*2; i+=2)
				{
					this->sendElement(pu[i]);
				}

				
				// send f1 values 
				this->sendElement((nCons-1)); // send number of elements
				for(int i=0; i<(nCons-1)*2; i+=2)
				{
					this->sendElement(pf[i]);
				}

				// send f2 values 
				this->sendElement((nCons-1)); // send number of elements
				for(int i=1; i<(nCons-1)*2; i+=2)
				{
					this->sendElement(pf[i]);
				}


				cableChain.get_Catenary(steps, samplePoints);

				this->sendElement(steps*3); // send number of points in the catenary
				this->sendElement(samplePoints, steps);

			

				delete [] pCons;
				delete [] samplePoints;
				delete [] pu;
				delete [] pf;
			}
			

		case 200: // load a robot geometry 
			{	
				m_pWireCenterView->UpdateStatistics();
				m_pWireCenterView->Invalidate();
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 201: // PrintGLParam
			{	
				pScene->printParam();
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 202: // set workspace visibility
			{	
				bool isVisible;
				this->readElement(isVisible, rIndex);
				m_pWireCenterView->setWorkspaceVisibility(isVisible);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 203: // set cable visibility
			{	
				bool isVisible;
				this->readElement(isVisible, rIndex);
				m_pWireCenterView->setWiresVisibility(isVisible);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 204: // set platform visibility
			{	
				bool isVisible;
				this->readElement(isVisible, rIndex);
				m_pWireCenterView->setPlatformVisibility(isVisible);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 205: // set plane visibility
			{	
				bool isVisible;
				this->readElement(isVisible, rIndex);
				m_pWireCenterView->setPlanesVisibility(isVisible);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 206: // set interference visibility
			{	
				bool isVisible;
				this->readElement(isVisible, rIndex);
				m_pWireCenterView->setInterferenceVisibility(isVisible);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 207: // create Box
			{
				CVector3 v1, v2, color;
				double alpha;
				string layerName;
				unsigned int frameID;

				this->readElement(layerName, rIndex);
				this->readElement(frameID, rIndex);
				this->readElement(v1, rIndex);	
				this->readElement(v2, rIndex);
				this->readElement(color, rIndex);	
				this->readElement(alpha, rIndex);
			
				TShapeAttribute shapeAttributes;
				shapeAttributes.sLayer = layerName;
				shapeAttributes.iFrame = frameID;
				shapeAttributes.bVisibile = true;

				pScene->createBox(v1,v2, color, alpha, &shapeAttributes);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 208: // create Cone
			{
				CVector3 apex, axis, color;
				double aperture, alpha;
				string layerName;
				unsigned int frameID;
				
				this->readElement(layerName, rIndex);
				this->readElement(frameID, rIndex);
				this->readElement(apex, rIndex);
				this->readElement(axis, rIndex);	
				this->readElement(aperture, rIndex);	
				this->readElement(color, rIndex);	
				this->readElement(alpha, rIndex);
				
				TShapeAttribute shapeAttributes;
				shapeAttributes.sLayer = layerName;
				shapeAttributes.iFrame = frameID;
				shapeAttributes.bVisibile = true;

				pScene->createCone(apex,axis,aperture,color,alpha,&shapeAttributes);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 209: // create STL
			{
				std::string filename;
				CVector3 r;
				double s;
				CMatrix3 R(1,0,0,0,1,0,0,0,1);

				this->readElement(filename, rIndex);	
				this->readElement(r, rIndex);
				this->readElement(s, rIndex);	
				pScene->createSTL(filename,r,R,s);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 210: // create 3DS
			{
				std::string filename;
				CVector3 r;
				double s;
				CMatrix3 R(1,0,0,0,1,0,0,0,1);

				this->readElement(filename, rIndex);	
				this->readElement(r, rIndex);
				this->readElement(s, rIndex);	
				pScene->create3DS(filename,r,R,s);
				bool result = true;
				this->sendElement(result);
				break;
			}
		case 211: // delete user defined shapes on the specified layer
			{
				string layerName;
				this->readElement(layerName, rIndex);
				m_pWireCenterView->m_Scene.deleteLayer(layerName);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 212: // call a generic setter function (boolean version)
			{
				std::string paramName;
				bool x;
				this->readElement(paramName, rIndex);
				this->readElement(x, rIndex);
				bool result = m_pWireCenterView->set(paramName,x);
				this->sendElement(result);
				break;
			}

		case 213: // call a generic setter function (integer version)
			{
				std::string paramName;
				int x;
				this->readElement(paramName, rIndex);
				this->readElement(x, rIndex);
				bool result = m_pWireCenterView->set(paramName,x);
				this->sendElement(result);
				break;
			}

		case 214: // call a generic setter function (double version)
			{
				std::string paramName;
				double x;
				this->readElement(paramName, rIndex);
				this->readElement(x, rIndex);
				bool result = m_pWireCenterView->set(paramName,x);
				this->sendElement(result);
				break;
			}

		case 215: // call a generic setter function (string version)
			{
				std::string paramName;
				std::string x;
				this->readElement(paramName, rIndex);
				this->readElement(x, rIndex);
				bool result = m_pWireCenterView->set(paramName,x);
				this->sendElement(result);
				break;
			}

		case 216: // select differential hull
			{
				int i;
				this->readElement(i, rIndex);
				int result = m_pWireCenterView->pWDH->DifferenceSelector = i;
				this->sendElement(result);
				break;
			}

		case 217: // save a screenshot as bmp 
			{
				std::string fileName;
				int width, height;
				this->readElement(fileName, rIndex);
				this->readElement(width, rIndex);
				this->readElement(height, rIndex);
				pScene->SaveBmp(fileName,width,height);
				bool result = true;
				this->sendElement(result);
				break;
			}


		case 218: // save a screenshot as bmp 
			{
				std::string fileName;
				this->readElement(fileName, rIndex);
				pScene->SaveEps(fileName);
				bool result = true;
				this->sendElement(result);
				break;
			}


		case 219: // render scene
			{
				m_pWireCenterView->m_Scene.RenderScene();
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 220: // set appearance of workspace hull
			{
				int glBeginMode;
				this->readElement(glBeginMode, rIndex);
				//GL_POINTS = 0
				//GL_LINE_LOOP = 2
				//GL_TRIANGLES = 4
				//m_pWireCenterView->pHull->setAppearance(glBeginMode);
				bool result = false; //todo
				this->sendElement(result);
				break;
			}

		case 221: // draw line strip
			{
				string layerName;
				unsigned int frameID;
				int numVertices, factor;
				CVector3 v;
				CVector3 color;
				vector<CVector3> lineStrip;
				float lineWidth;
				unsigned short pattern;
				double alpha;
				
				this->readElement(layerName, rIndex);
				this->readElement(frameID, rIndex);
				this->readElement(numVertices, rIndex);
				lineStrip.reserve(numVertices);

				for (int i = 0; i < numVertices; i++)
				{
					this->readElement(v, rIndex);
					lineStrip.push_back(v);	
				}
				
				this->readElement(lineWidth, rIndex);
				this->readElement(factor, rIndex);
				this->readElement(pattern, rIndex);
				this->readElement(color, rIndex);
				this->readElement(alpha, rIndex);
				
				TShapeAttribute shapeAttributes;
				shapeAttributes.sLayer = layerName;
				shapeAttributes.iFrame = frameID;
				shapeAttributes.bVisibile = true;

				pScene->createLineStrip(lineStrip, lineWidth, factor, pattern, color, alpha, &shapeAttributes);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 222: // make vector
			{
				CVector3 v1, v2, color;
				double radius, alpha;
				string layerName;
				unsigned int frameID;

				this->readElement(layerName, rIndex);
				this->readElement(frameID, rIndex);
				this->readElement(v1, rIndex);
				this->readElement(v2, rIndex);
				this->readElement(radius, rIndex);
				this->readElement(color, rIndex);
				this->readElement(alpha, rIndex);
				

				TShapeAttribute shapeAttributes;
				shapeAttributes.sLayer = layerName;
				shapeAttributes.iFrame = frameID;
				shapeAttributes.bVisibile = true;

				pScene->createVector(v1, v2, radius, color, alpha, &shapeAttributes);
				bool result = true;
				this->sendElement(result);
				break;
			}


		case 223: // set frame bounding box visibility
		{	
			bool isVisible;
			this->readElement(isVisible, rIndex);
			m_pWireCenterView->setFrameBoundingBoxVisibility(isVisible);
			bool result = true;
			this->sendElement(result);
			break;
		}

		case 224: // set platform bounding box visibility
		{	
			bool isVisible;
			this->readElement(isVisible, rIndex);
			m_pWireCenterView->setPlatformBoundingBoxVisibility(isVisible);
			bool result = true;
			this->sendElement(result);
			break;
		}

		case 225: // delete all shapes
		{
			m_pWireCenterView->m_Scene.deleteAllShapes();
			bool result = true;
			this->sendElement(result);
			break;
		}

		case 226: // set wires visibility
		{	
			bool isVisible;
			this->readElement(isVisible, rIndex);
			m_pWireCenterView->setWiresVisibility(isVisible);
			bool result = true;
			this->sendElement(result);
			break;
		}

		case 227: // set winches visibility
		{	
			bool isVisible;
			this->readElement(isVisible, rIndex);
			m_pWireCenterView->setWinchesVisibility(isVisible);
			bool result = true;
			this->sendElement(result);
			break;
		}

		case 228: // set layer visibility
		{
			unsigned int layerID;
			bool bVisible;
			this->readElement(layerID, rIndex);
			this->readElement(bVisible, rIndex);
			m_pWireCenterView->m_Scene.setLayerAttributes(layerID, bVisible);
			bool result = true;
			this->sendElement(result);
			break;
		}

		/* Obsolete Functions
		case 229: // Set algorithm type
		{
			unsigned int nAlgType;
			this->readElement(nAlgType, rIndex);
			pKinematics->setAlgType(nAlgType);
			bool result = true;
			this->sendElement(result);
			break;
		}

		case 230: // Set Forward solver type
		{
			unsigned int nSolverType;
			this->readElement(nSolverType, rIndex);
			pKinematics->setForwardSolverType(nSolverType);
			bool result = true;
			this->sendElement(result);
			break;
		}

		case 231: // Set Inverse solver type
		{
			unsigned int nSolverType;
			this->readElement(nSolverType, rIndex);
			pKinematics->setInverseSolverType(nSolverType);
			bool result = true;
			this->sendElement(result);
			break;
		}

		case 232: // Set pose estimator
		{
			unsigned int nPoseEstimator;
			this->readElement(nPoseEstimator, rIndex);
			pKinematics->setPoseEstimator(nPoseEstimator);
			bool result = true;
			this->sendElement(result);
			break;
		}*/

		//case 233: // Run forward kinematics main
		//	{
		//		// Declare position vector
		//		Vector3d r;

		//		// Declare orientation matrix
		//		Matrix3d R;

		//		// create a cable vector
		//		double* l = new double[8];

		//		// Read values from Matlab
		//		this->readElement(l, 8, rIndex);

		//		// Run doForwardKinematicsMain
		//		pKinematics->doForwardKinematicsMain(l,r,R);

		//		// Send values to Matlab
		//		this->sendElement(r);
		//		this->sendElement(R);
		//		break;
		//	}

			//case 235: // Run forward kinematics pulley
			//{
			//	// Declare position vector
			//	Vector3d r;

			//	// Declare orientation matrix
			//	Matrix3d R;

			//	// create a cable vector
			//	double* l = new double[8];

			//	// Read values from Matlab
			//	this->readElement(l, 8, rIndex);

			//	// Run doForwardKinematicsMain
			//	pKinematics->doForwardKinematicsPulley(l,r,R);

			//	// Send values to Matlab
			//	this->sendElement(r);
			//	this->sendElement(R);
			//	break;
			//}

			case 236: // Run inverse kinematics pulley
			{
				// Declare position vector
				Vector3d r;

				// Declare orientation matrix
				Matrix3d R;

				// create a cable vector m_pWireCenterDoc->robotDoc.getNow()
				MatrixXd l(8,1);

				// Read values from Matlab
				this->readElement(r, rIndex);
				this->readElement(R, rIndex);

				// Run doInverseKinematicsMain
				pKinematics->doInverseKinematicsPulley(r,R,l);

				// Send values to Matlab
				this->sendElement(l.data(),8);
			
				break;
			}


			case 237: // send bitmap to MATLAB
			{

				int sizeX, sizeY;
				
				this->readElement(sizeX, rIndex);
				this->readElement(sizeY, rIndex);

				CGLMemoryContext MC(sizeX,sizeY);
				m_pWireCenterView->m_Scene.GetBmp(MC);

				int imageSize = MC.getImageSize();

				sendElement(imageSize); // send number of elements
				sendElement((unsigned char*)MC.getImagePtr(),imageSize);

				break;
			}

			case 238: // save image to file
			{
				string fileName;
				int fileType;
				this->readElement(fileName, rIndex); 
				this->readElement(fileType, rIndex); 

				m_pWireCenterView->m_Scene.SaveImage(fileName,fileType);

				bool result = true;
				this->sendElement(result);
				break;
			}

			case 239: // set scene attributes like number of facettes for round objects
			{
				int facettes_;
				this->readElement(facettes_, rIndex); 
				IPAGL::setSceneAttributes(facettes_);
				bool result = true;
				this->sendElement(result);
				break;
			}


			// Run 


			//--------------------------------------
			// Path generator methods
			//--------------------------------------
		case 300: // load STL
			{
				std::string fileName;
				this->readElement(fileName, rIndex);
				pPG->load(fileName);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 301: // save path
			{
				std::string fileName;
				this->readElement(fileName, rIndex);
				pPG->savePath(fileName);
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 302: // generate path
			{
				bool result = pPG->generatePath();
				this->sendElement(result);
				break;
			}

		case 303: // set eps
			{
				double eps_LayerDist,eps_PathDist;
				this->readElement(eps_LayerDist, rIndex);
				this->readElement(eps_PathDist, rIndex);
				pPG->eps_LayerDist = eps_LayerDist;
				pPG->eps_PathDist = eps_PathDist;
				bool result = true;
				this->sendElement(result);
				break;
			}

		case 401: //load NC program from text file and generate path
			{
				std::string fileName;
				this->readElement(fileName, rIndex);
				CString fileNameC(fileName.c_str());
				bool result = m_pWireCenterView->LoadNcProgram(fileNameC);
				this->sendElement(result);
				break;
			}

		case 402: //load NC program from text string provided by MATLAB
			{
				break;
			}


		case 403: //get poseList
			{
				//CVector3 position;
				//CMatrix3 orientation;

				PCRL::CPoseListKinetostatic PoseList = m_pWireCenterView->PoseList;
				PCRL::CPoseListKinetostatic::iterator it_SD; 
				double cycleTime = m_pWireCenterView->GetRobotDoc()->Interpolator.getCycleTime();
				this->sendElement(cycleTime);
				int numel = PoseList.size()*12; // 3x pos, 9x orientation
				this->sendElement(numel); // send number of elements in the pose list		
				for (it_SD=PoseList.begin();it_SD!=PoseList.end();it_SD++) 
				{
					// send position
					this->sendElement((*it_SD)->r);
					// send rotation matrix column wise
					this->sendElement((*it_SD)->R.col(1));
					this->sendElement((*it_SD)->R.col(2));
					this->sendElement((*it_SD)->R.col(3));
				}
				break;
			}

		case 500: // compute roatation matrix from euler angles
			{
				Vector3d rot_xyz;
				Matrix3d R;
				this->readElement(rot_xyz, rIndex);
				PCRL::getMatrixFromXYZ(R,rot_xyz);
				this->sendElement(R);
				break;		
			}


		default: printf("Command not recognized\n");
		}

		if (rIndex > m_wIndex)
		{
			printf("Error in message parser: index out of range\n");
			break; 
		}

		if (rIndex == m_wIndex) // no more messages in buffer
		{
			break;
		}
		else
		{
			// read size of next message
			this->readElement(msgSize,rIndex);
		}
	}
	// Move remaining elements to front of buffer
	int i_dest = 0;
	for (int i = rIndex; i< m_wIndex;i++)
	{
		m_buffer[i_dest] = m_buffer[rIndex];
		i_dest++;
	}
	// set new write index
	m_wIndex = i_dest;
}
//*******************************************************************************************************
//*******************************************************************************************************