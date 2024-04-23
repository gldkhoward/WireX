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

/*! \file PyScriptDlg.h
 *
 *	\author   Andreas Pott
 *
 *  \brief A dialog box to load and view python script that
 *  can be executed inside the application. The feature is not
 *  yet fully implemented.
 */

#pragma once


// CPyScriptDlg-Dialogfeld

class CPyScriptDlg : public CDialog
{
	DECLARE_DYNAMIC(CPyScriptDlg)

public:
	CPyScriptDlg(CWnd* pParent = NULL);   // Standardkonstruktor
	virtual ~CPyScriptDlg();

// Dialogfelddaten
	enum { IDD = IDD_PYTHONSCRIPT_DIALOG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV-Unterstützung

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedPyloadButton();
	afx_msg void OnBnClickedPyrunButton();
	CString m_filename;
};
