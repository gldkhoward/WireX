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

/*! \file UserMessages.h
 *
 *	\author   Andreas Pott
 *
 *  \brief A light-weight header file with the define statements of the
 *  user message ID. This is needed for unique ID codes between different
 *  windows.
 */

#pragma once

// define the message to close the windows by parent application
// for some reason it is adviced to start the ID with a little offset from the base address
#define WM_CLOSEPOSEDLG					WM_USER + 5
#define WM_UPDATEPOSE					WM_USER + 6
#define WM_CLOSEINTERACTIVEGEOMETRYDLG	WM_USER + 7
#define WM_UPDATEWORKSPACE				WM_USER + 8