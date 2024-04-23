/*! \file levmar.h
*
*	\author   Andreas Pott
*
*  (C)opyright 2019, Andreas Pott
*
*
* This is a proxy interface to use WireLib withlout levmar
*/

#pragma once

// #define WIRELIB_HAS_LEVMAR

#ifdef WIRELIB_HAS_LEVMAR
#include <levmar/levmar.h>
#else
// as we do not have levmar we need to define a minimum set of 
// constants to make the implemenation compatible
#define LM_INFO_SZ 1
#endif

int wc_dlevmar_der(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	void(*jacf)(double *p, double *j, int m, int n, void *adata),
	double *p, double *x, int m, int n, int itmax, double *opts,
	double *info, double *work, double *covar, void *adata);

int wc_dlevmar_dif(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	double *p, double *x, int m, int n, int itmax, double *opts,
	double *info, double *work, double *covar, void *adata);

int wc_dlevmar_bc_der(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	void(*jacf)(double *p, double *j, int m, int n, void *adata),
	double *p, double *x, int m, int n, double *lb, double *ub,
	int itmax, double *opts, double *info, double *work, double *covar, void *adata);

int wc_dlevmar_bc_dif(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	double *p, double *x, int m, int n, double *lb, double *ub,
	int itmax, double *opts, double *info, double *work, double *covar, void *adata);
