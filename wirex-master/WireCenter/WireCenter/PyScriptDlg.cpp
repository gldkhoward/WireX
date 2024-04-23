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

// PyScriptDlg.cpp: Implementierungsdatei
//

#include "stdafx.h"
#include "WireCenter.h"
#include "PyScriptDlg.h"
#include "PythonInterface.h"

#include <iostream>
#include <fstream>

// CPyScriptDlg-Dialogfeld

IMPLEMENT_DYNAMIC(CPyScriptDlg, CDialog)

CPyScriptDlg::CPyScriptDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CPyScriptDlg::IDD, pParent)
	, m_filename(_T(""))
{
}

CPyScriptDlg::~CPyScriptDlg()
{
}

void CPyScriptDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_SCRIPTFILENAME_EDIT, m_filename);
}


BEGIN_MESSAGE_MAP(CPyScriptDlg, CDialog)
	ON_BN_CLICKED(IDC_PYLOAD_BUTTON, &CPyScriptDlg::OnBnClickedPyloadButton)
	ON_BN_CLICKED(IDC_PYRUN_BUTTON, &CPyScriptDlg::OnBnClickedPyrunButton)
END_MESSAGE_MAP()

// CPyScriptDlg-Meldungshandler

void CPyScriptDlg::OnBnClickedPyloadButton()
{
	CFileDialog FileDlg(TRUE,	// load File Dialog
						".py",
						"Script.py",
						OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
						"Python Scripts (*.py)|*.py||");
	if (FileDlg.DoModal()!=IDOK)
		return;

	m_filename=FileDlg.GetFileName();
	UpdateData(FALSE);
	
	std::ifstream file(m_filename.GetString());
	CString str; 
	char buf[1024];
	while (!file.eof())
	{
		file.getline(buf,1024);
		str+=buf;
		str+="\r\n";
	}
	// copy the text into the edit control
	CEdit* pEdit = (CEdit*)GetDlgItem(IDC_PYSCRIPT_EDIT);
	pEdit->SetWindowTextA(str);
	UpdateData(FALSE);
}

void CPyScriptDlg::OnBnClickedPyrunButton()
{
	// initialize the python scripting interface and load the scripting functions
	// by concention the main function in the script is "def main()"
	char* mainFunctionName="main";
	if (!CPythonInterface::getInstance().run(m_filename.GetString(),mainFunctionName))
		AfxMessageBox("Failed to load python script. Perhaps the file could not be opened or the main function is not implemented.",MB_OK);
}
