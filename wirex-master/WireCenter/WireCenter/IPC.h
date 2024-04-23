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

/*! \file IPC.h.h
 *
 *	\author   Philipp Miermeister
 *
 *  \brief The class provides an interprocess communication interface for WireCenter.
 *  It can be used with third party tools like Matlab or the RTX environment.
 *  The class may provide different communication method in the future.
 *  Right now it provides only a socket interface.
 */

#pragma once
#include <afxsock.h>


// forward declerations
class CWireCenterView;
class CGlScene;
class CRobotDocument;

#define COMMAND 1
#define FRAME 2

#define USER_INFO_NONE 0
#define USER_INFO_NORMAL 1
#define USER_INFO_ALL 2



// Base class
class CIPC_SocketBase : public CAsyncSocket
{
public:
	CIPC_SocketBase();
	~CIPC_SocketBase();

public:
	void sendData(const char* pData, const int &msgSize);
};

// The CIPC_SocketRecv class provides a socket for receiving a data stream
// The class is used by CIPC_Socket

class CIPC_SocketRecv : public CIPC_SocketBase
{


	 // Attributes
  private:
	int m_bufferSize;
	int m_userInformationState;
	vector<char> m_buffer;
	int m_wIndex;
	int m_rIndex;
	bool m_clientClosed;

	CWireCenterView* m_pWireCenterView;

	

	//void* m_pObject;	// pointer to the calling object
	//void (*m_pMessageParser)(void*,unsigned char*);  // pointer to the paraser function
	 
	
  // Operations
  public:
	  CIPC_SocketRecv(CWireCenterView* pWireCenterView);
     ~CIPC_SocketRecv();
	 bool isClosed();
	 void messageParser();

	 template <class T>
	 void readElement(T &x_Out, int &rIndex_InOut);
	 template <class T>
	 void readElement(T*& px_Out, const int &size, int &rIndex_InOut);

	 void readElement(CVector3 &x_Out, int &rIndex_InOut);
	 void readElement(Vector3d &x_Out, int &rIndex_InOut);
	 void readElement(Matrix3d &x_Out, int &rIndex_InOut);
	 void readElement(string &x_Out, int &rIndex_InOut);
	
	 template <class T>
	 void sendElement(const T &x);
	 template <class T>
	 void sendElement(T* px, const int &size);

	 void sendElement(const string &s);
	 void sendElement(const CVector3 &x);
	 void sendElement(const Vector3d &x);
	 void sendElement(const Matrix3d &v);
	 void sendElement(const MatrixXd &v);
  
  // Overrides
  public:

      void OnReceive(int nErrorCode);
	  void OnClose(int nErrorCode);
	 
};


// The CIPC_Socket class provides a socket for interprocess communication
// It listens for messages of a client and creates a new socket object to receive the data.

class CIPC_Socket : public CAsyncSocket
{


// Attributes
private:
	CIPC_SocketRecv* m_pSocketRecv; // Socket for receiving data
	CWireCenterView* m_pWireCenterView;
	
// Operations
public:
	  CIPC_Socket(CWireCenterView* pWireCenterView);
	 ~CIPC_Socket();
	 bool initialize();

  
// Overrides
public:

	void OnAccept(int nErrorCode);
	void OnConnect(int nErrorCode);
	// void OnClose(int nErrorCode);
	void OnOutOfBandData(int nErrorCode);
	void OnSend(int nErrorCode);


};

//---------------------------------------------

class CIPC
{

private:
	CIPC_SocketRecv* m_pSocketRecv;
	CIPC_Socket* m_pSocket;
	
public:
	// Constructor
	CIPC(CWireCenterView* pWireCenterView);
	~CIPC();
	void startServer();
	//static void messageParser(void* pObject,char* pMsg);	

	DWORD waitForClient(LPVOID lpParam);
};

