/*
* WireX  -  WireLib
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

/*! \file levmar.cpp
 * 
 */

#include "levmar.h"

#ifdef WIRELIB_HAS_LEVMAR
#include <levmar/levmar.h>
#endif

int wc_dlevmar_der(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	void(*jacf)(double *p, double *j, int m, int n, void *adata),
	double *p, double *x, int m, int n, int itmax, double *opts,
	double *info, double *work, double *covar, void *adata)
{
#ifdef WIRELIB_HAS_LEVMAR
	return dlevmar_der(func, jacf, p, x, m, n, itmax, opts, info, work, covar, adata);
#else
	return -1;
#endif // WIRELIB_HAS_LEVMAR
}

int wc_dlevmar_dif(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	double *p, double *x, int m, int n, int itmax, double *opts,
	double *info, double *work, double *covar, void *adata)
{
#ifdef WIRELIB_HAS_LEVMAR
	return dlevmar_dif(func, p, x, m, n, itmax, opts, info, work, covar, adata);
#else
	return -1;
#endif // WIRELIB_HAS_LEVMAR
}

/* box-constrained minimization */
int wc_dlevmar_bc_der(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	void(*jacf)(double *p, double *j, int m, int n, void *adata),
	double *p, double *x, int m, int n, double *lb, double *ub,
	int itmax, double *opts, double *info, double *work, double *covar, void *adata)
{
#ifdef WIRELIB_HAS_LEVMAR
	return dlevmar_bc_der(func, jacf, p, x, m, n, lb, ub, itmax, opts, info, work, covar, adata);
#else
	return -1;
#endif // WIRELIB_HAS_LEVMAR
}

int wc_dlevmar_bc_dif(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	double *p, double *x, int m, int n, double *lb, double *ub,
	int itmax, double *opts, double *info, double *work, double *covar, void *adata)
{
#ifdef WIRELIB_HAS_LEVMAR
	return dlevmar_bc_dif(func, p, x, m, n, lb, ub, itmax, opts, info, work, covar, adata);
#else
	return -1;
#endif // WIRELIB_HAS_LEVMAR
}
