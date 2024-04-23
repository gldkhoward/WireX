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

/*! \file EnumTables.h
 *
 *	\author   Valentin Schmidt
 *
 *  \brief This file contains all enumerators which have corresponding string-tables
 *  (for usage in class-reflection, automatic generation of GUI elements, mapping to 
 *  files, ...).
 *
 *  \remark
 *  Make sure the number of items in the enums ::Type correlate exactly with 
 *  the string tables ::Names[]. The implementation will unsupervised use 
 *  a (int)-casting to access the respective string. Mismatch between enums 
 *  and string tables results in segmentation fault.
 */


#pragma once

namespace PCRL {
	namespace emWireErrors { //encapsulating enumerators to unclutter PCRL namespace (enables enums with identical values) workaround if C++11 should switch to "enum class"
		enum Type {
			poseConsistent, //! Pose Error which is consistent for a given set of cable lengths
			randomErrors,   //! Randomly distributed errors
		};
		static const char* Names[] = {
			"poseConsistent", 
			"randomErrors",
			0
		};
	};

	namespace emForwardAlgorithm{
		enum Type {
			levmar_dif,	//! unconstrained LM-optimizer without analytic Jacobian
			levmar_der,	//! unconstrained LM-optimizer with analytic Jacobian (this seems to be the fastest and is the default)
			// the constrained solver seem to work as well but they are much slower (~10 times)
			levmar_difBounded, //! constrained LM-optimizer without analytic Jacobian
			levmar_derBounded,  //! constrained LM-optimizer with analytic Jacobian
			gauss_newton,		//! Gauss-Newton Method
			eigen_levmar_dif	//! use the levmar implementation coming from eigen/unsupported which is essentially a port of cminpack
		};
		//! string table for the selector for least-square optimization
		static const char* Names[] = {
			"Levmar_without_analytic_Jacobian", 
			"Levmar_with_analytic_Jacobian", 
			"Constrained_Levmar_without_analytic_Jacobian", 
			"Constrained_Levmar_with_analytic_Jacobian",
			"Gauss_Newton",
			"Eigen_Levmar_with_analytic_Jacobian",
			0
		};
	};	
	
	namespace emForwardModel{
		enum Type {
			standard,   //! using Standard-Model robot geometry
			standard_Transform_Geometry,
			standard_abcpreconditioning,
			distance,	//! Standard-Model using distance equations as parameterization 
			pulley	    //! using Pulley-Model robot geometry
		};
		//! string table for the selector for Forward Kinematics Algorithm
		static const char* Names[] = {
			"Fk_Standard_Model",
			"Fk_Standard_Transform_Geometry",
			"Fk_Standard_Model_Angle_Preconditioning",
			"Fk_Standard_Model_Distance_Equations", 
			"Fk_Pulley_Model", 
			0
		};
	};

	namespace emInverseModel{
		enum Type {
			standard,   //! using Standard-Model robot geometry
			pulley		//! using Pulley-Model robot geometry
		};
	
		//! string table for the selector for Inverse Kinematics Algorithm
		static const char* Names[] = {
			"Ik_Standard_Model", 
			"Ik_Pulley_Model", 
			0
		};
	};

	namespace emPoseEstimate{
		enum Type {
			basic,						//! Interval-Based pose estimation
			basicSimpleBoundingbox,		//! Interval-Based pose estimation
			basicComplexBoundingbox,	
			inherit,					//! Use "previous" pose as estimation
			intersectionMethod,			//! Squared-Regression method of pose estimation
		};
		//! string table for the selector for pose estimation method
		static const char* Names[] = {
			"Basic", 
			"BasicSimpleBoundingbox", 
			"BasicComplexBoundingbox",
			"Inherit", 
			"IntersectionMethod", 
			0
		};
	};

	//! selector for calculation method
	namespace emForceDistributionMethod{
		enum Type{ 
			closedForm,							//! close Form Force Distribution Pseudo-Inverse
			bruteForce,							//! brute force Force Distribution
			dykstra,							//! dykstra Force Distribution
			weightedSum, 						//! simple weighted sum force distribution
			advancedClosedForm,					//! close Form Force Distribution with advanced cases
			closedFormEnergyEfficient,			//! close Form Force Distribution with minimal forces
			puncture,							//! force distribution puncture method
			advancedClosedFormEnergyEfficient,	//! close Form Force Distribution with minimal forces and advanced cases
			quadraticProgramming,				//! quadratic programming approach for force distribution 1/2xQx+c
			wrenchClosure,						//! for debugging: only test wrench-closure instead of computing a distribution
			barycenter
		};
	
		static const char* Names[] = {
			"closedForm",
			"bruteForce", 
			"dykstra", 
			"weightedSum",
			"advancedClosedForm",
			"closedFormEnergyEfficient",
			"puncture",		 
			"advancedClosedFormEnergyEfficient",
			"quadraticProgramming",
			"wrenchClosure",
			"barycenter",
			0
		};
	};

	namespace emEvaluatorCriteria{
		enum Type {
			feasible,    //! Evaluate force distribution by min max
			euclidean,   //! Evaluate total force using normalized value
			infinity	 //! Evaluate total force using normalized value with margins
		};
	
		static const char* Names[] = {
			"feasible", 
			"euclidean", 
			"infinity",
			0
		};
	};

	namespace emLinearSolverMethod{
		enum Type { 
			lu,						//! Linear Solver: LU-decomposition
			llt,					//! Linear Solver: Standard Cholesky decomposition 
			ldlt,					//! Robust Cholesky decomposition of a matrix with pivoting.
			householderQr,			//! Householder QR decomposition of a matrix. 
			jacobiSvd,				//! Two-sided Jacobi SVD decomposition of a rectangular matrix
			explicitPseudoinverse	//! Linear Solver: Explicit Pesudo-inverse
		};
	
		//! string table for the selector for calculation method
		static const char* Names[] = {
			"lu", 
			"llt", 
			"ldlt", 
			"householderQr", 
			"jacobiSvd", 
			"explicitPseudoinverse",
			0
		};
	}

	namespace emPoseProperty {
		enum Type {
			PoseMapper,
			InverseKinematics_Standard,
			InverseKinematics_Pulley,
			ForwardKinematicsIt,
			OrientationWorkspaceSize,
			CableForcesMethod,
			CableForcesAllMethods,
			Dexterity,
			BaseFrameForces,
			Stiffness,
			PoseEstimation,
			CableWear,
			WrenchSet,
			ForwardKinematicsDisturbance,
			NullEvaluator,
			CableForcesMethodDynamic
		};
		static const char* Names[] = {
			"PoseMapper",
			"InverseKinematics_Standard",
			"InverseKinematics_Pulley",
			"ForwardKinematicsIt",
			"OrientationWorkspaceSize",
			"CableForcesMethod",
			"CableForcesAllMethods",
			"Dexterity",
			"BaseFrameForces",
			"Stiffness",
			"PoseEstimation",
			"CableWear",
			"WrenchSet",
			"ForwardKinematicsDisturbance",
			"NullEvaluator",
			"CableForcesMethodDynamic",
			0
		};
	}

	namespace emOrientationParameter { 
		enum Type {
			rotationMatrix, //!< rotation matrix r11,...,r33
			quaternion,		//!< four coefficient quaternion rotation parameterization
			eulerAngles,	//!< three parameter euler angle z,x,z rotation
		};
		static const char* Names[] = {
			"rotationMatrix", 
			"quaternion",
			"eulerAngles",
			0
		};
	};

	namespace emPotentialEnergyModel {
		enum Type {
			NoCable,			//!< do not apply a cable model
			LinearBilateral,	//!< model the cable as linear elastic slender bar storing energy in compression and elongation
			LinearUnilateral,	//!< ideal cable model storing energy only in elongation while being perfectly slack in compression
			Sagging,			//!< Irvine's model of the sagging cable 
			SaggingExponential  //!< an experimental ansatz function approximating the energy in the cable with an exponential function
		};
		static const char* Names[] = {
			"NoCable", 
			"LinearBilateral", 
			"LinearUnilateral", 
			"Sagging", 
			"SaggingExponential",
			0
		};	
	};

} // end namespace PCRL