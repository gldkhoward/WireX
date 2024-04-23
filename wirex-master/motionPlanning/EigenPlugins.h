/*
* WireX  -  motionPlanning
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
 *  \file   : EigenPlugins.h
 *
 *  Project   motionPlanning
 *
 *  \Author   Philipp Miermeister
 *
 *  This header contains the plugins to extend the Eigen library.
 *  More precisely, these functions are added to the matrix template to
 *  become member function of the class matrix.
 *********************************************************************
 */ 

#pragma once

// Returns a 3x3 x-rotation matrix
inline static Derived XRotationMatrix3d(const RealScalar& alpha) 
{
	Derived res;
	res << 1,0,0,
		0,internal::cos(alpha),-internal::sin(alpha),
		0,internal::sin(alpha),internal::cos(alpha); 
	return res;
}

// Returns a 3x3 y-rotation matrix
inline static Derived YRotationMatrix3d(const RealScalar& alpha) 
{
	Derived res;
	res << internal::cos(alpha),0,internal::sin(alpha),
		0,1,0,
		-internal::sin(alpha),0,internal::cos(alpha); 
	return res;
}

// Returns a 3x3 z-rotation matrix
inline static Derived ZRotationMatrix3d(const RealScalar& alpha) 
{
	Derived res;
	res <<internal::cos(alpha),-internal::sin(alpha),0,
		internal::sin(alpha),internal::cos(alpha),0,
		0,0,1;
	return res;
}

// Sets matrix to a 3x3 x-rotation matrix
inline void setXRotationMatrix3d(const RealScalar& alpha) {derived() = XRotationMatrix3d(alpha);}

// Sets matrix to a 3x3 y-rotation matrix
inline void setYRotationMatrix3d(const RealScalar& alpha) {derived() = YRotationMatrix3d(alpha);}

// Sets matrix to a 3x3 z-rotation matrix
inline void setZRotationMatrix3d(const RealScalar& alpha) {derived() = ZRotationMatrix3d(alpha);}

// ORIGINAL 
//inline void setXRotationMatrix3d(const RealScalar& alpha) {derived() = AngleAxisd(alpha,Vector3d(1.0,0.0,0.0)).toRotationMatrix();}

// Sets matrix to a 3x3 y-rotation matrix
//inline void setYRotationMatrix3d(const RealScalar& alpha) {derived() = AngleAxisd(alpha,Vector3d(0,1,0)).toRotationMatrix();}

// Sets matrix to a 3x3 z-rotation matrix
//inline void setZRotationMatrix3d(const RealScalar& alpha) {derived() = AngleAxisd(alpha,Vector3d(0,0,1)).toRotationMatrix();}

// Returns a 3x3 x-rotation matrix
//inline static Derived XRotationMatrix3d(const RealScalar& alpha) {return AngleAxisd(alpha,Vector3d(1,0,0)).toRotationMatrix();}

// Returns a 3x3 y-rotation matrix
//inline static Derived YRotationMatrix3d(const RealScalar& alpha) {return AngleAxisd(alpha,Vector3d(0,1,0)).toRotationMatrix();}

// Returns a 3x3 z-rotation matrix
//inline static Derived ZRotationMatrix3d(const RealScalar& alpha) {return AngleAxisd(alpha,Vector3d(0,0,1)).toRotationMatrix();}


