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

/*! \file EigenLib.h
 *
 *  \Author   Philipp Miermeister
 *
 *  \Date     29.09.2011 
 */

#pragma once
// Load plugins
#define EIGEN_MATRIXBASE_PLUGIN <WireLib/EigenPlugins.h>

#include <Eigen/Eigen>
#include <cstdlib>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>


using namespace std;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

static const double MO_PI = 3.141592653589793 ;
static const double RAD_TO_DEG = 57.29577951308232 ;
static const double DEG_TO_RAD = 1.7453292519943300E-002 ;

// Matlab
static const Eigen::IOFormat IOMatlabMatrix(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
static const Eigen::IOFormat IOMatlabVector(Eigen::StreamPrecision, 0, ", ", "; ", "", "", "[", "]");
// Maple
static const Eigen::IOFormat IOMapleMatrix(Eigen::StreamPrecision, 0, ", ", ",\n", "[", "]", "Matrix([", "])");
static const Eigen::IOFormat IOMapleVector(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "Vector([", "])");
// CSV
static const Eigen::IOFormat IOCSV(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "", "");
// Console (add brackets and align matrices and vectors)
static const Eigen::IOFormat CleanFmt(3, Eigen::Aligned, ", ", "\n", "[", "]");
