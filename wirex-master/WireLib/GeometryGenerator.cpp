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

/*!*******************************************************************
 *  \file   : GeometryGenerator.cpp
 *
 *  Project   Wirelib
 *
 *  \Author   Andreas Pott
 *
 *  \Date     29.10.2012
 *
 *********************************************************************
 */ 

#include "GeometryGenerator.h"

namespace PCRL {

// implementation of CGeometryGenerator
/////////////////////////////////////////////////////////////////////

//! set the value of a parameter by its name
bool CGeometryGenerator::setParameter(const string& name, const double& value)
{
	return Reflector().set(name,value);
}

//! get the value of a parameter by its name
bool CGeometryGenerator::getParameter(const string& name, double& value)
{
	return Reflector().get(name,value);
}

//! return a list with the symbolic names of the parameters
void CGeometryGenerator::getParamNames(list<string>& names)
{
	Reflector().getParamNames(names);
}

// implementation of CParametricBoxFrame
/////////////////////////////////////////////////////////////////////

CParametricBoxFrame::CParametricBoxFrame(CRobotData& robot) : CGeometryGenerator(robot) 
{ 
	nop = 3; 
	now = 8; 
	length = 4; 
	width = 3; 
	height = 2; 
	name = "ParametricBoxFrame"; 
	motionPattern = CRobotData::MP2R3T | CRobotData::MP3T | CRobotData::MP3R3T;
	generatorScope = GSeffectFrame;
}

void CParametricBoxFrame::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(length,"length","generator/boxFrame/@length");
	pReflector->bind(width,"width","generator/boxFrame/@width");
	pReflector->bind(height,"height","generator/boxFrame/@height");
}

bool CParametricBoxFrame::setGeometry()
{
	if (pRobot->getNow() != now)
		return false;
	pRobot->setBase(0, Vector3d(-length/2,  width/2, height));
	pRobot->setBase(1, Vector3d( length/2,  width/2, height));
	pRobot->setBase(2, Vector3d( length/2, -width/2, height));
	pRobot->setBase(3, Vector3d(-length/2, -width/2, height));
	pRobot->setBase(4, Vector3d(-length/2,  width/2, 0));
	pRobot->setBase(5, Vector3d( length/2,  width/2, 0));
	pRobot->setBase(6, Vector3d( length/2, -width/2, 0));
	pRobot->setBase(7, Vector3d(-length/2, -width/2, 0));
	return true;
}

// implementation of CParametricBoxPlatform
/////////////////////////////////////////////////////////////////////

CParametricBoxPlatform::CParametricBoxPlatform(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{ 
	nop = 3; 
	now = 8; 
	length = 1; 
	width = 0.8; 
	height = 0.6; 
	name = "ParametricBoxPlatform"; 
	motionPattern = CRobotData::MP3R3T;
	generatorScope = GSeffectPlatform;
}

void CParametricBoxPlatform::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(length,"length","generator/boxPlatform/@length");
	pReflector->bind(width,"width","generator/boxPlatform/@width");
	pReflector->bind(height,"height","generator/boxPlatform/@height");
}

bool CParametricBoxPlatform::setGeometry()
{
	if (pRobot->getNow() != now)
		return false;
	pRobot->setPlatform(0, Vector3d(-length/2,  width/2, height));
	pRobot->setPlatform(1, Vector3d( length/2,  width/2, height));
	pRobot->setPlatform(2, Vector3d( length/2, -width/2, height));
	pRobot->setPlatform(3, Vector3d(-length/2, -width/2, height));
	pRobot->setPlatform(4, Vector3d(-length/2,  width/2, 0));
	pRobot->setPlatform(5, Vector3d( length/2,  width/2, 0));
	pRobot->setPlatform(6, Vector3d( length/2, -width/2, 0));
	pRobot->setPlatform(7, Vector3d(-length/2, -width/2, 0));
	return true;
}

// implementation of CIPAnema1Design
/////////////////////////////////////////////////////////////////////

CIPAnema1Design::CIPAnema1Design(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "IPAnema1Design";
	nop = 5;	// five parameters
	now = 8;	// eight cables
	frameLength = 4; // the original design parameters of the IPAnema 1 aluminium frame		
	frameWidth = 3;		
	frameHeight = 2;		
	platformLength = 0.12;
	platformWidth = 0.12;	
	motionPattern = CRobotData::MP3R3T;
	generatorScope = GSeffectFrame | GSeffectPlatform;
}
	
void CIPAnema1Design::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(frameLength,"frameLength","generator/IPAnema1Design/@frameLength");
	pReflector->bind(frameWidth,"frameWidth","generator/IPAnema1Design/@frameWidth");
	pReflector->bind(frameHeight,"frameHeight","generator/IPAnema1Design/@frameHeight");
	pReflector->bind(platformLength,"platformLength","generator/IPAnema1Design/@platformLength");
	pReflector->bind(platformWidth,"platformWidth","generator/IPAnema1Design/@platformWidth");
}

bool CIPAnema1Design::setGeometry()
{
	if (pRobot->getNow() != now)
		return false;

	pRobot->setPlatform(0, Vector3d(-platformLength/2,  platformWidth/2, 0));
	pRobot->setPlatform(1, Vector3d( platformLength/2,  platformWidth/2, 0));
	pRobot->setPlatform(2, Vector3d( platformLength/2, -platformWidth/2, 0));
	pRobot->setPlatform(3, Vector3d(-platformLength/2, -platformWidth/2, 0));
	pRobot->setPlatform(4, Vector3d(-platformLength/2,  platformWidth/2, 0));
	pRobot->setPlatform(5, Vector3d( platformLength/2,  platformWidth/2, 0));
	pRobot->setPlatform(6, Vector3d( platformLength/2, -platformWidth/2, 0));
	pRobot->setPlatform(7, Vector3d(-platformLength/2, -platformWidth/2, 0));

	pRobot->setBase(0, Vector3d(-frameLength/2,  frameWidth/2, frameHeight));
	pRobot->setBase(1, Vector3d( frameLength/2,  frameWidth/2, frameHeight));
	pRobot->setBase(2, Vector3d( frameLength/2, -frameWidth/2, frameHeight));
	pRobot->setBase(3, Vector3d(-frameLength/2, -frameWidth/2, frameHeight));
	pRobot->setBase(4, Vector3d(-frameLength/2,  frameWidth/2, 0));
	pRobot->setBase(5, Vector3d( frameLength/2,  frameWidth/2, 0));
	pRobot->setBase(6, Vector3d( frameLength/2, -frameWidth/2, 0));
	pRobot->setBase(7, Vector3d(-frameLength/2, -frameWidth/2, 0));	
	
	return true;
}

// implementation of CIPAnema2Frame
/////////////////////////////////////////////////////////////////////

CIPAnema2Frame::CIPAnema2Frame(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{ 
	nop = 5; 
	now = 8; 
	length = 8; 
	width = 6; 
	height = 5.5; 
	h0 = 1; 
	delta = 0.2;  
	name = "IPAnema2Frame";
	motionPattern = CRobotData::MP3R3T | CRobotData::MP2R3T | CRobotData::MP3T;
	generatorScope = GSeffectFrame;
}

void CIPAnema2Frame::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(length,"length","generator/IPAnema2/@length");
	pReflector->bind(width,"width","generator/IPAnema2/@width");
	pReflector->bind(height,"height","generator/IPAnema2/@height");
	pReflector->bind(height,"delta","generator/IPAnema2/@delta");
	pReflector->bind(height,"h0","generator/IPAnema2/@height");
}

bool CIPAnema2Frame::setGeometry()
{
	if (pRobot->getNow() != now)
		return false;
	pRobot->setBase(0, Vector3d(-length/2+delta,  width/2-delta, height));
	pRobot->setBase(1, Vector3d( length/2-delta,  width/2-delta, height));
	pRobot->setBase(2, Vector3d( length/2-delta, -width/2+delta, height));
	pRobot->setBase(3, Vector3d(-length/2+delta, -width/2+delta, height));
	pRobot->setBase(4, Vector3d(-length/2+delta,  width/2-delta, h0));
	pRobot->setBase(5, Vector3d( length/2-delta,  width/2-delta, h0));
	pRobot->setBase(6, Vector3d( length/2-delta, -width/2+delta, h0));
	pRobot->setBase(7, Vector3d(-length/2+delta, -width/2+delta, h0));
	return true;
}

// implementation of CFalcon7Design
/////////////////////////////////////////////////////////////////////

CFalcon7Design::CFalcon7Design(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "Falcon7Design";
	nop = 7; 
	now = 7; 
	frameLength = 1.45;		
	frameWidth = 1.45;		
	frameHeight = 0;		
	frameHeight0 = 1.25;	
	platformLength = 0;
	platformWidth = 0.1;	
	platformHeight = 1;
	motionPattern = CRobotData::MP3R3T;
	generatorScope = GSeffectFrame | GSeffectPlatform;
}
	
void CFalcon7Design::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(frameLength,"frameLength","generator/Falcon7Design/@frameLength");
	pReflector->bind(frameWidth,"frameWidth","generator/Falcon7Design/@frameWidth");
	pReflector->bind(frameHeight,"frameHeight","generator/Falcon7Design/@frameHeight");
	pReflector->bind(frameHeight0,"frameHeight0","generator/Falcon7Design/@frameHeight0");
	pReflector->bind(platformLength,"platformLength","generator/Falcon7Design/@platformLength");
	pReflector->bind(platformWidth,"platformWidth","generator/Falcon7Design/@platformWidth");
	pReflector->bind(platformHeight,"platformHeight","generator/Falcon7Design/@platformHeight");
}

bool CFalcon7Design::setGeometry()
{
	if (pRobot->getNow() != now)
		return false;

	pRobot->setPlatform(0, Vector3d( 0,  0, platformHeight));
	pRobot->setPlatform(1, Vector3d( 0,  0, platformHeight));
	pRobot->setPlatform(2, Vector3d( 0, -0, platformHeight));
	pRobot->setPlatform(3, Vector3d(-platformLength/2,  platformWidth/2, 0));
	pRobot->setPlatform(4, Vector3d( platformLength/2,  platformWidth/2, 0));
	pRobot->setPlatform(5, Vector3d( platformLength/2, -platformWidth/2, 0));
	pRobot->setPlatform(6, Vector3d(-platformLength/2, -platformWidth/2, 0));

	pRobot->setBase(0, Vector3d(-frameLength/2,  frameWidth/2, frameHeight0+frameHeight));
	pRobot->setBase(1, Vector3d( frameLength/2,				0, frameHeight0+frameHeight));
	pRobot->setBase(2, Vector3d(-frameLength/2, -frameWidth/2, frameHeight0+frameHeight));
	pRobot->setBase(3, Vector3d(-frameLength/2,  frameWidth/2, frameHeight0));
	pRobot->setBase(4, Vector3d( frameLength/2,  frameWidth/2, frameHeight0));
	pRobot->setBase(5, Vector3d( frameLength/2, -frameWidth/2, frameHeight0));
	pRobot->setBase(6, Vector3d(-frameLength/2, -frameWidth/2, frameHeight0));
	return true;
}

// implementation of CIPAnema17Design
/////////////////////////////////////////////////////////////////////

CIPAnema17Design::CIPAnema17Design(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "IPAnema17Design";
	nop = 5; 
	now = 7; 
	frameLength = 4;		
	frameWidth = 3;		
	frameHeight = 2;		
	platformLength = 0.25;
	platformWidth = 0.25;	
	motionPattern = CRobotData::MP3R3T;
	generatorScope = GSeffectFrame | GSeffectPlatform;
}
	
void CIPAnema17Design::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(frameLength,"frameLength","generator/IPAnema17Design/@frameLength");
	pReflector->bind(frameWidth,"frameWidth","generator/IPAnema17Design/@frameWidth");
	pReflector->bind(frameHeight,"frameHeight","generator/IPAnema17Design/@frameHeight");
	pReflector->bind(platformLength,"platformLength","generator/IPAnema17Design/@platformLength");
	pReflector->bind(platformWidth,"platformWidth","generator/IPAnema17Design/@platformWidth");
}

bool CIPAnema17Design::setGeometry()
{
	if (pRobot->getNow() != now)
		return false;

	pRobot->setPlatform(0, Vector3d(-platformLength/2,              0, 0));
	pRobot->setPlatform(1, Vector3d( platformLength/2,              0, 0));
	pRobot->setPlatform(2, Vector3d(			    0,  platformWidth, 0));
	pRobot->setPlatform(3, Vector3d(			    0,  platformWidth, 0));
	pRobot->setPlatform(4, Vector3d(-platformLength/2,              0, 0));
	pRobot->setPlatform(5, Vector3d( platformLength/2,              0, 0));
	pRobot->setPlatform(6, Vector3d(			    0,  platformWidth, 0));

	pRobot->setBase(0, Vector3d(             0,          0, 0));
	pRobot->setBase(1, Vector3d(   frameLength,          0, 0));
	pRobot->setBase(2, Vector3d(   frameLength, frameWidth, 0));
	pRobot->setBase(3, Vector3d(             0, frameWidth, 0));
	pRobot->setBase(4, Vector3d(             0,          0, frameHeight));
	pRobot->setBase(5, Vector3d(   frameLength,          0, frameHeight));
	pRobot->setBase(6, Vector3d( frameLength/2, frameWidth, frameHeight));
	return true;
}

// implementation of CSegesta7Design
/////////////////////////////////////////////////////////////////////

CSegesta7Design::CSegesta7Design(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "Segesta7Design";
	nop = 5; 
	now = 7; 
	frameLength = 0.83;		
	frameWidth = 0.63;		
	frameHeight = 1;		
	platformLength = 0.105;
	platformWidth = 0.2;	
	motionPattern = CRobotData::MP3R3T;
	generatorScope = GSeffectFrame | GSeffectPlatform;
}
	
void CSegesta7Design::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(frameLength,"frameLength","generator/Segesta7Design/@frameLength");
	pReflector->bind(frameWidth,"frameWidth","generator/Segesta7Design/@frameWidth");
	pReflector->bind(frameHeight,"frameHeight","generator/Segesta7Design/@frameHeight");
	pReflector->bind(platformLength,"platformLength","generator/Segesta7Design/@platformLength");
	pReflector->bind(platformWidth,"platformWidth","generator/Segesta7Design/@platformWidth");
}

bool CSegesta7Design::setGeometry()
{
	if (pRobot->getNow() != now)
		return false;

	pRobot->setPlatform(0, Vector3d(-platformLength/2,  platformWidth/2, 0));
	pRobot->setPlatform(1, Vector3d( platformLength/2,  platformWidth/2, 0));
	pRobot->setPlatform(2, Vector3d(                0, -platformWidth/2, 0));
	pRobot->setPlatform(3, Vector3d(-platformLength/2,  platformWidth/2, 0));
	pRobot->setPlatform(4, Vector3d( platformLength/2,  platformWidth/2, 0));
	pRobot->setPlatform(5, Vector3d(                0, -platformWidth/2, 0));
	pRobot->setPlatform(6, Vector3d(                0, -platformWidth/2, 0));

	pRobot->setBase(0, Vector3d(-frameLength/2,  frameWidth/2, frameHeight));
	pRobot->setBase(1, Vector3d( frameLength/2,  frameWidth/2, frameHeight));
	pRobot->setBase(2, Vector3d(             0, -frameWidth/2, frameHeight));
	pRobot->setBase(3, Vector3d(-frameLength/2,  frameWidth/2, 0));
	pRobot->setBase(4, Vector3d( frameLength/2,  frameWidth/2, 0));
	pRobot->setBase(5, Vector3d( frameLength/2, -frameWidth/2, 0));
	pRobot->setBase(6, Vector3d(-frameLength/2, -frameWidth/2, 0));

	return true;
}

// implementation of CSegesta8Design
/////////////////////////////////////////////////////////////////////

CSegesta8Design::CSegesta8Design(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "Segesta8Design";
	nop = 5; 
	now = 8; 
	frameLength = 0.83;		
	frameWidth = 0.63;		
	frameHeight = 1;		
	platformLength = 0.105;
	platformWidth = 0.2;	
	motionPattern = CRobotData::MP3R3T;
	generatorScope = GSeffectFrame | GSeffectPlatform;
}
	
void CSegesta8Design::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(frameLength,"frameLength","generator/Segesta8Design/@frameLength");
	pReflector->bind(frameWidth,"frameWidth","generator/Segesta8Design/@frameWidth");
	pReflector->bind(frameHeight,"frameHeight","generator/Segesta8Design/@frameHeight");
	pReflector->bind(platformLength,"platformLength","generator/Segesta8Design/@platformLength");
	pReflector->bind(platformWidth,"platformWidth","generator/Segesta8Design/@platformWidth");
}

bool CSegesta8Design::setGeometry()
{
	if (pRobot->getNow() != now)
		return false;

	pRobot->setPlatform(0, Vector3d(-platformLength/2,  platformWidth/2, 0));
	pRobot->setPlatform(1, Vector3d( platformLength/2,  platformWidth/2, 0));
	pRobot->setPlatform(2, Vector3d(                0, -platformWidth/2, 0));
	pRobot->setPlatform(3, Vector3d(                0, -platformWidth/2, 0));
	pRobot->setPlatform(4, Vector3d(-platformLength/2,  platformWidth/2, 0));
	pRobot->setPlatform(5, Vector3d( platformLength/2,  platformWidth/2, 0));
	pRobot->setPlatform(6, Vector3d(                0, -platformWidth/2, 0));
	pRobot->setPlatform(7, Vector3d(                0, -platformWidth/2, 0));

	pRobot->setBase(0, Vector3d(-frameLength/2,  frameWidth/2, frameHeight));
	pRobot->setBase(1, Vector3d( frameLength/2,  frameWidth/2, frameHeight));
	pRobot->setBase(2, Vector3d( frameLength/2, -frameWidth/2, frameHeight));
	pRobot->setBase(3, Vector3d(-frameLength/2, -frameWidth/2, frameHeight));
	pRobot->setBase(4, Vector3d(-frameLength/2,  frameWidth/2, 0));
	pRobot->setBase(5, Vector3d( frameLength/2,  frameWidth/2, 0));
	pRobot->setBase(6, Vector3d( frameLength/2, -frameWidth/2, 0));
	pRobot->setBase(7, Vector3d(-frameLength/2, -frameWidth/2, 0));

	return true;
}

// implementation of CIPAnema3Design
/////////////////////////////////////////////////////////////////////

CIPAnema3Design::CIPAnema3Design(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "IPAnema3Design";
	nop = 11; 
	now = 8;

	// inital parameter guess by wek and asp from 11.11.2013
	frameLength = 8;			
	frameWidth = 3.5;			
	frameHeight = 3.25;			
	frameHeight0 = 0.25;		
	frameDeltaLength = 0.2;	
	frameDeltaWidth = 0.25;		
	platformLength = 1.5;		
	platformWidth = 0.5;		
	platformHeight = 0.5;		
	platformDeltaLength = 0.2;	
	platformDeltaWidth = 0.1;	

	motionPattern = CRobotData::MP3R3T;
	generatorScope = GSeffectFrame | GSeffectPlatform;
}
	
void CIPAnema3Design::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(frameLength,"frameLength","generator/IPAnema3Design/@frameLength");
	pReflector->bind(frameWidth,"frameWidth","generator/IPAnema3Design/@frameWidth");
	pReflector->bind(frameHeight,"frameHeight","generator/IPAnema3Design/@frameHeight");
	pReflector->bind(frameHeight0,"frameHeight0","generator/IPAnema3Design/@frameHeight0");
	pReflector->bind(frameDeltaLength,"frameDeltaLength","generator/IPAnema3Design/@frameDeltaLength");
	pReflector->bind(frameDeltaWidth,"frameDeltaWidth","generator/IPAnema3Design/@frameDeltaWidth");
	pReflector->bind(platformLength,"platformLength","generator/IPAnema3Design/@platformLength");
	pReflector->bind(platformWidth,"platformWidth","generator/IPAnema3Design/@platformWidth");
	pReflector->bind(platformHeight,"platformHeight","generator/IPAnema3Design/@platformHeight");
	pReflector->bind(platformDeltaLength,"platformDeltaLength","generator/IPAnema3Design/@platformDeltaLength");
	pReflector->bind(platformDeltaWidth,"platformDeltaWidth","generator/IPAnema3Design/@platformDeltaWidth");
}

bool CIPAnema3Design::setGeometry()
{
	if (pRobot->getNow() != now)
		return false;

	pRobot->setPlatform(0, Vector3d(-platformLength/2+platformDeltaLength,  platformWidth/2,                    0 ));
	pRobot->setPlatform(1, Vector3d( platformLength/2-platformDeltaLength,  platformWidth/2,                    0 ));
	pRobot->setPlatform(2, Vector3d( platformLength/2-platformDeltaLength, -platformWidth/2,                    0 ));
	pRobot->setPlatform(3, Vector3d(-platformLength/2+platformDeltaLength, -platformWidth/2,                    0 ));
	pRobot->setPlatform(4, Vector3d(-platformLength/2,					    platformWidth/2-platformDeltaWidth, platformHeight));
	pRobot->setPlatform(5, Vector3d( platformLength/2,                      platformWidth/2-platformDeltaWidth, platformHeight));
	pRobot->setPlatform(6, Vector3d( platformLength/2,                     -platformWidth/2+platformDeltaWidth, platformHeight));
	pRobot->setPlatform(7, Vector3d(-platformLength/2,                     -platformWidth/2+platformDeltaWidth, platformHeight));

	pRobot->setBase(0, Vector3d(-frameLength/2+frameDeltaLength,  frameWidth/2,                 frameHeight ));
	pRobot->setBase(1, Vector3d( frameLength/2-frameDeltaLength,  frameWidth/2,                 frameHeight ));
	pRobot->setBase(2, Vector3d( frameLength/2-frameDeltaLength, -frameWidth/2,                 frameHeight ));
	pRobot->setBase(3, Vector3d(-frameLength/2+frameDeltaLength, -frameWidth/2,                 frameHeight ));
	pRobot->setBase(4, Vector3d(-frameLength/2,					  frameWidth/2-frameDeltaWidth, frameHeight0));
	pRobot->setBase(5, Vector3d( frameLength/2,                   frameWidth/2-frameDeltaWidth, frameHeight0));
	pRobot->setBase(6, Vector3d( frameLength/2,                  -frameWidth/2+frameDeltaWidth, frameHeight0));
	pRobot->setBase(7, Vector3d(-frameLength/2,                  -frameWidth/2+frameDeltaWidth, frameHeight0));

	return true;
}

// implementation of CCoGiRo8Design
/////////////////////////////////////////////////////////////////////

CCoGiRo8Design::CCoGiRo8Design(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "CoGiRo8Design";
	nop = 7; 
	now = 8; 
	frameLength = 18;		
	frameWidth = 12;		
	frameHeight = 2;		
	frameHeight0 = 7;	
	platformLength = 1;
	platformWidth = 1;	
	platformHeight = 1;
	motionPattern = CRobotData::MP3R3T;
	generatorScope = GSeffectFrame | GSeffectPlatform;
}
	
void CCoGiRo8Design::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(frameLength,"frameLength","generator/CoGiRo8Design/@frameLength");
	pReflector->bind(frameWidth,"frameWidth","generator/CoGiRo8Design/@frameWidth");
	pReflector->bind(frameHeight,"frameHeight","generator/CoGiRo8Design/@frameHeight");
	pReflector->bind(frameHeight0,"frameHeight0","generator/CoGiRo8Design/@frameHeight0");
	pReflector->bind(platformLength,"platformLength","generator/CoGiRo8Design/@platformLength");
	pReflector->bind(platformWidth,"platformWidth","generator/CoGiRo8Design/@platformWidth");
	pReflector->bind(platformHeight,"platformHeight","generator/CoGiRo8Design/@platformHeight");
}

//! set the geometry; the design is roughly the box design for platform and base, BUT
//! the connection for the winches is connected to different anchor points on the platform
//! where the upper winches are clockwise rotate and the lower winches are connected in a
//! counter-clockwise manner.
bool CCoGiRo8Design::setGeometry()
{
	if (pRobot->getNow() != now)
		return false;

	pRobot->setPlatform(0, Vector3d( platformLength/2,  platformWidth/2, platformHeight));
	pRobot->setPlatform(1, Vector3d( platformLength/2, -platformWidth/2, platformHeight));
	pRobot->setPlatform(2, Vector3d(-platformLength/2, -platformWidth/2, platformHeight));
	pRobot->setPlatform(3, Vector3d(-platformLength/2,  platformWidth/2, platformHeight));
	pRobot->setPlatform(4, Vector3d(-platformLength/2, -platformWidth/2, 0));
	pRobot->setPlatform(5, Vector3d(-platformLength/2,  platformWidth/2, 0));
	pRobot->setPlatform(6, Vector3d( platformLength/2,  platformWidth/2, 0));
	pRobot->setPlatform(7, Vector3d( platformLength/2, -platformWidth/2, 0));

	//! the naming for the anchor point on the base is taken from the standard
	pRobot->setBase(0, Vector3d(-frameLength/2,  frameWidth/2, frameHeight0+frameHeight));
	pRobot->setBase(1, Vector3d( frameLength/2,	 frameWidth/2, frameHeight0+frameHeight));
	pRobot->setBase(2, Vector3d( frameLength/2, -frameWidth/2, frameHeight0+frameHeight));
	pRobot->setBase(3, Vector3d(-frameLength/2, -frameWidth/2, frameHeight0+frameHeight));
	pRobot->setBase(4, Vector3d(-frameLength/2,  frameWidth/2, frameHeight0));
	pRobot->setBase(5, Vector3d( frameLength/2,  frameWidth/2, frameHeight0));
	pRobot->setBase(6, Vector3d( frameLength/2, -frameWidth/2, frameHeight0));
	pRobot->setBase(7, Vector3d(-frameLength/2, -frameWidth/2, frameHeight0));
	return true;
}

// implementation of CMotionSimulatorDesign
/////////////////////////////////////////////////////////////////////

CMotionSimulatorDesign::CMotionSimulatorDesign(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "MotionSimulatorDesign";
	nop = 4; 
	now = 8; 
	frameLength = 12;		
	frameWidth=12;		
	frameHeight=8;		
	platformRadius=1.35;
	motionPattern=CRobotData::MP3R3T;
	generatorScope = GSeffectFrame | GSeffectPlatform;
}
	
void CMotionSimulatorDesign::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(frameLength,"frameLength","generator/MotionSimulatorDesign/@frameLength");
	pReflector->bind(frameWidth,"frameWidth","generator/MotionSimulatorDesign/@frameWidth");
	pReflector->bind(frameHeight,"frameHeight","generator/MotionSimulatorDesign/@frameHeight");
	pReflector->bind(platformRadius,"platformRadius","generator/MotionSimulatorDesign/@platformRadius");
}

//! the platform geometry puts the distal anchor points to the vertices of
//! an icosahedron.
bool CMotionSimulatorDesign::setGeometry()
{
	if (pRobot->getNow() != now)
		return false;

	// we have to compute some support items to generate the icosahedron
	double h = platformRadius * cos(MO_PI/5.0) / (1+cos(MO_PI/5.0));
	double rm = platformRadius * 2.0*sqrt(5.0)/5.0;
	Vector3d b1234(rm,0,-h), b5678 (rm,0,h);
	Matrix3d::ZRotationMatrix3d(108*DEG_TO_RAD)*b1234;
	pRobot->setPlatform(0, Matrix3d::ZRotationMatrix3d( 108*DEG_TO_RAD)*b1234);
	pRobot->setPlatform(1, Matrix3d::ZRotationMatrix3d( 36*DEG_TO_RAD)*b1234);
	pRobot->setPlatform(2, Matrix3d::ZRotationMatrix3d( -36*DEG_TO_RAD)*b1234);
	pRobot->setPlatform(3, Matrix3d::ZRotationMatrix3d(-108*DEG_TO_RAD)*b1234);
	pRobot->setPlatform(4, Matrix3d::ZRotationMatrix3d( 144*DEG_TO_RAD)*b5678);
	pRobot->setPlatform(5, Matrix3d::ZRotationMatrix3d(  72*DEG_TO_RAD)*b5678);
	pRobot->setPlatform(6, Matrix3d::ZRotationMatrix3d( -72*DEG_TO_RAD)*b5678);
	pRobot->setPlatform(7, Matrix3d::ZRotationMatrix3d(-144*DEG_TO_RAD)*b5678);

	//! the naming for the anchor point on the base is taken from the standard
	pRobot->setBase(0, Vector3d(-frameLength/2,  frameWidth/2, frameHeight));
	pRobot->setBase(1, Vector3d( frameLength/2,	 frameWidth/2, frameHeight));
	pRobot->setBase(2, Vector3d( frameLength/2, -frameWidth/2, frameHeight));
	pRobot->setBase(3, Vector3d(-frameLength/2, -frameWidth/2, frameHeight));
	pRobot->setBase(4, Vector3d(-frameLength/2,  frameWidth/2, 0));
	pRobot->setBase(5, Vector3d( frameLength/2,  frameWidth/2, 0));
	pRobot->setBase(6, Vector3d( frameLength/2, -frameWidth/2, 0));
	pRobot->setBase(7, Vector3d(-frameLength/2, -frameWidth/2, 0));
	return true;
}

// implementation of CSSM6Design
/////////////////////////////////////////////////////////////////////

CSSM6Design::CSSM6Design(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "SSM6Design";
	nop = 4; 
	now = 6; 
	frameRadius = 1;		
	frameAlpha = 15*DEG_TO_RAD;		
	platformRadius = 0.1;		
	platformAlpha = 30*DEG_TO_RAD;	
	motionPattern = CRobotData::MP3R3T;
	generatorScope = GSeffectFrame | GSeffectPlatform;
}
	
void CSSM6Design::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(frameRadius,"frameRadius","generator/CoGiRo8Design/@frameRadius");
	pReflector->bind(frameAlpha,"frameAlpha","generator/CoGiRo8Design/@frameAlpha");
	pReflector->bind(platformRadius,"platformRadius","generator/CoGiRo8Design/@platformRadius");
	pReflector->bind(platformAlpha,"platformAlpha","generator/CoGiRo8Design/@platformAlpha");
}

//! Note that we cannot maintain the standard numberic scheme for this robot
//! since the number scheme despends on the value of frameAlpha: for large values (>30 deg)
//! the 6th anchoir point should be the first. however, since the connection between
//! platform and base keeps correct we do not perform renaming based on the parameter values.
bool CSSM6Design::setGeometry()
{
	if (pRobot->getNow() != now)
		return false;

	pRobot->setPlatform(0, Vector3d( -platformRadius*sin(platformAlpha),  platformRadius*cos(platformAlpha), 0));
	pRobot->setPlatform(1, Vector3d(  platformRadius*sin(platformAlpha),  platformRadius*cos(platformAlpha), 0));
	pRobot->setPlatform(2, Vector3d(  platformRadius*sin(platformAlpha+MO_PI/3.0),  -platformRadius*cos(platformAlpha+MO_PI/3.0), 0));
	pRobot->setPlatform(3, Vector3d(  platformRadius*cos(platformAlpha+MO_PI/6.0),  -platformRadius*sin(platformAlpha+MO_PI/6.0), 0));
	pRobot->setPlatform(4, Vector3d( -platformRadius*cos(platformAlpha+MO_PI/6.0),  -platformRadius*sin(platformAlpha+MO_PI/6.0), 0));
	pRobot->setPlatform(5, Vector3d( -platformRadius*sin(platformAlpha+MO_PI/3.0),  -platformRadius*cos(platformAlpha+MO_PI/3.0), 0));

	//! the naming for the anchor point on the base is taken from the standard
	pRobot->setBase(0, Vector3d( -frameRadius*sin(frameAlpha),  frameRadius*cos(frameAlpha), 0));
	pRobot->setBase(1, Vector3d(  frameRadius*sin(frameAlpha),  frameRadius*cos(frameAlpha), 0));
	pRobot->setBase(2, Vector3d(  frameRadius*sin(frameAlpha+MO_PI/3.0),  -frameRadius*cos(frameAlpha+MO_PI/3.0), 0));
	pRobot->setBase(3, Vector3d(  frameRadius*cos(frameAlpha+MO_PI/6.0),  -frameRadius*sin(frameAlpha+MO_PI/6.0), 0));
	pRobot->setBase(4, Vector3d(- frameRadius*cos(frameAlpha+MO_PI/6.0),  -frameRadius*sin(frameAlpha+MO_PI/6.0), 0));
	pRobot->setBase(5, Vector3d( -frameRadius*sin(frameAlpha+MO_PI/3.0),  -frameRadius*cos(frameAlpha+MO_PI/3.0), 0));
	return true;
}

// implementation of CPlanarRobot
/////////////////////////////////////////////////////////////////////

CPlanarRobot::CPlanarRobot(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "PlanarRobot";
	now = 4; 
	nop = 4;
	frameLength = 33;		
	frameWidth = 30;		
	platformLength = 1;
	platformWidth = 1;
	motionPattern = CRobotData::MP1R2T;
	generatorScope = GSeffectFrame | GSeffectPlatform;
}
	
void CPlanarRobot::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(frameLength,"frameLength","generator/PlanarRobot/@frameLength");
	pReflector->bind(frameWidth,"frameWidth","generator/PlanarRobot/@frameWidth");
	pReflector->bind(platformLength,"platformLength","generator/PlanarRobot/@platformLength");
	pReflector->bind(platformWidth,"platformWidth","generator/PlanarRobot/@platformWidth");
}

bool CPlanarRobot::setGeometry()
{
	if (pRobot->getNow() != now)
		return false;

	//! the naming for the anchor point on the base is taken from the standard
	pRobot->setPlatform(0, Vector3d(-platformLength/2,  platformWidth/2, 0));
	pRobot->setPlatform(1, Vector3d( platformLength/2,  platformWidth/2, 0));
	pRobot->setPlatform(2, Vector3d( platformLength/2, -platformWidth/2, 0));
	pRobot->setPlatform(3, Vector3d(-platformLength/2, -platformWidth/2, 0));

	//! the naming for the anchor point on the base is taken from the standard
	pRobot->setBase(0, Vector3d(-frameLength/2,  frameWidth/2, 0));
	pRobot->setBase(1, Vector3d( frameLength/2,	 frameWidth/2, 0));
	pRobot->setBase(2, Vector3d( frameLength/2, -frameWidth/2, 0));
	pRobot->setBase(3, Vector3d(-frameLength/2, -frameWidth/2, 0));
	return true;
}

// implementation of CEndlessZ9Design
/////////////////////////////////////////////////////////////////////

CEndlessZ9Design::CEndlessZ9Design(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "EndlessZ9Design";
	now = 9; 
	nop = 6;
	platformHeight = 0.4;	
	platformHeight2 = 0.5;	
	platformRadius = 0.3;	
	frameRadius = 2;		
	frameHeight = 2;		
	frameHeight2 = 3;	
	motionPattern = CRobotData::MP3R3T;
	generatorScope = GSeffectFrame | GSeffectPlatform;
}
	
void CEndlessZ9Design::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(platformHeight, "platformHeight", "generator/EndlessZ9Design/@platformHeight");
	pReflector->bind(platformHeight2,"platformHeight2","generator/EndlessZ9Design/@platformHeight2");
	pReflector->bind(platformRadius,"platformRadius","generator/EndlessZ9Design/@platformRadius");
	pReflector->bind(frameRadius,"frameRadius","generator/EndlessZ9Design/@frameRadius");
	pReflector->bind(frameHeight,"frameHeight","generator/EndlessZ9Design/@frameHeight");
	pReflector->bind(frameHeight2,"frameHeight2","generator/EndlessZ9Design/@frameHeight2");
}

bool CEndlessZ9Design::setGeometry()
{
	if (pRobot->getNow() != now)
		return false;

	pRobot->setPlatform(0, Vector3d( platformRadius, 0, platformHeight2));
	pRobot->setPlatform(1, Vector3d( platformRadius, 0, platformHeight2));
	pRobot->setPlatform(2, Vector3d( platformRadius, 0, platformHeight2));
	pRobot->setPlatform(3, Vector3d( 0,				 0, platformHeight));
	pRobot->setPlatform(4, Vector3d( 0,				 0, platformHeight));
	pRobot->setPlatform(5, Vector3d( 0,				 0, platformHeight));
	pRobot->setPlatform(6, Vector3d( 0,				 0,				 0));
	pRobot->setPlatform(7, Vector3d( 0,				 0,				 0));
	pRobot->setPlatform(8, Vector3d( 0,				 0,				 0));

	//! the naming for the anchor point on the base is taken from the standard
	pRobot->setBase(0, Vector3d( cos( 120*DEG_TO_RAD)*frameRadius,  sin( 120*DEG_TO_RAD)*frameRadius, frameHeight2));
	pRobot->setBase(1, Vector3d( cos(   0*DEG_TO_RAD)*frameRadius,  sin(   0*DEG_TO_RAD)*frameRadius, frameHeight2));
	pRobot->setBase(2, Vector3d( cos(-120*DEG_TO_RAD)*frameRadius,  sin(-120*DEG_TO_RAD)*frameRadius, frameHeight2));
	pRobot->setBase(3, Vector3d( cos(-120*DEG_TO_RAD)*frameRadius,  sin(-120*DEG_TO_RAD)*frameRadius, frameHeight));
	pRobot->setBase(4, Vector3d( cos(   0*DEG_TO_RAD)*frameRadius,  sin(   0*DEG_TO_RAD)*frameRadius, frameHeight));
	pRobot->setBase(5, Vector3d( cos( 120*DEG_TO_RAD)*frameRadius,  sin( 120*DEG_TO_RAD)*frameRadius, frameHeight));
	pRobot->setBase(6, Vector3d( cos( 120*DEG_TO_RAD)*frameRadius,  sin( 120*DEG_TO_RAD)*frameRadius, 0));
	pRobot->setBase(7, Vector3d( cos(   0*DEG_TO_RAD)*frameRadius,  sin(   0*DEG_TO_RAD)*frameRadius, 0));
	pRobot->setBase(8, Vector3d( cos(-120*DEG_TO_RAD)*frameRadius,  sin(-120*DEG_TO_RAD)*frameRadius, 0));
	return true;
}

// implementation of CEndlessZ12Design
/////////////////////////////////////////////////////////////////////

CEndlessZ12Design::CEndlessZ12Design(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "EndlessZ12Design";
	now = 12; 
	nop = 6;
	platformHeight = 0.4;	
	platformHeight2 = 0.5;	
	platformRadius = 0.3;	
	frameRadius = 2;		
	frameHeight = 2;		
	frameHeight2 = 3;	
	motionPattern = CRobotData::MP3R3T;
	generatorScope = GSeffectFrame | GSeffectPlatform;
}
	
void CEndlessZ12Design::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(platformHeight, "platformHeight", "generator/EndlessZDesign12/@platformHeight");
	pReflector->bind(platformHeight2,"platformHeight2","generator/EndlessZDesign12/@platformHeight2");
	pReflector->bind(platformRadius,"platformRadius","generator/EndlessZDesign12/@platformRadius");
	pReflector->bind(frameRadius,"frameRadius","generator/EndlessZDesign12/@frameRadius");
	pReflector->bind(frameHeight,"frameHeight","generator/EndlessZDesign12/@frameHeight");
	pReflector->bind(frameHeight2,"frameHeight2","generator/EndlessZDesign12/@frameHeight2");
}

bool CEndlessZ12Design::setGeometry()
{
	if (pRobot->getNow() != now)
		return false;

	pRobot->setPlatform( 0, Vector3d(platformRadius, 0,  platformHeight2));
	pRobot->setPlatform( 1, Vector3d(platformRadius, 0,  platformHeight2));
	pRobot->setPlatform( 2, Vector3d(platformRadius, 0,  platformHeight2));
	pRobot->setPlatform( 3, Vector3d(platformRadius, 0,  platformHeight2));
	pRobot->setPlatform( 4, Vector3d(             0, 0,  platformHeight));
	pRobot->setPlatform( 5, Vector3d(             0, 0,  platformHeight));
	pRobot->setPlatform( 6, Vector3d(             0, 0,  platformHeight));
	pRobot->setPlatform( 7, Vector3d(             0, 0,  platformHeight));
	pRobot->setPlatform( 8, Vector3d(             0, 0,  0));
	pRobot->setPlatform( 9, Vector3d(             0, 0,  0));
	pRobot->setPlatform(10, Vector3d(             0, 0,  0));
	pRobot->setPlatform(11, Vector3d(             0, 0,  0));

	//! the naming for the anchor point on the base is taken from the standard
	pRobot->setBase( 0, Vector3d( -frameRadius,  frameRadius,  frameHeight2));
	pRobot->setBase( 1, Vector3d(  frameRadius,  frameRadius,  frameHeight2));
	pRobot->setBase( 2, Vector3d(  frameRadius, -frameRadius,  frameHeight2));
	pRobot->setBase( 3, Vector3d( -frameRadius, -frameRadius,  frameHeight2));
	pRobot->setBase( 4, Vector3d( -frameRadius,  frameRadius,  frameHeight));
	pRobot->setBase( 5, Vector3d(  frameRadius,  frameRadius,  frameHeight));
	pRobot->setBase( 6, Vector3d(  frameRadius, -frameRadius,  frameHeight));
	pRobot->setBase( 7, Vector3d( -frameRadius, -frameRadius,  frameHeight));
	pRobot->setBase( 8, Vector3d( -frameRadius,  frameRadius,  0));
	pRobot->setBase( 9, Vector3d(  frameRadius,  frameRadius,  0));
	pRobot->setBase(10, Vector3d(  frameRadius, -frameRadius,  0));
	pRobot->setBase(11, Vector3d( -frameRadius, -frameRadius,  0));
	
	return true;
}

// implementation of CFrenchGermanDesign
/////////////////////////////////////////////////////////////////////

CFrenchGermanDesign::CFrenchGermanDesign(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "FrenchGermanDesign";
	now = 12; 
	nop = 6;
	frameLength = 1;		
	frameWidth = 1;		
	frameHeight = 1;		
	platformLength = 0.1;
	platformWidth = 0.1;
	platformHeight = 0.1;
	motionPattern = CRobotData::MP3R3T;
	generatorScope = GSeffectFrame | GSeffectPlatform;
}
	
void CFrenchGermanDesign::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(frameLength,   "frameLength",		"generator/FrenchGermanDesign/@frameLength");
	pReflector->bind(frameWidth,    "frameWidth",		"generator/FrenchGermanDesign/@frameWidth");
	pReflector->bind(frameHeight,   "frameHeight",		"generator/FrenchGermanDesign/@frameHeight");
	pReflector->bind(platformLength,"platformLength",   "generator/FrenchGermanDesign/@platformLength");
	pReflector->bind(platformWidth, "platformWidth",    "generator/FrenchGermanDesign/@platformWidth");
	pReflector->bind(platformHeight,"platformHeight",   "generator/FrenchGermanDesign/@platformHeight");
}

bool CFrenchGermanDesign::setGeometry()
{
	if (pRobot->getNow() != now)
		return false;

	pRobot->setPlatform( 0, Vector3d( platformLength/2, 0,  0));
	pRobot->setPlatform( 1, Vector3d(-platformLength/2, 0,  0));
	pRobot->setPlatform( 2, Vector3d( 0, 0,  platformHeight/2));
	pRobot->setPlatform( 3, Vector3d( 0, 0, -platformHeight/2));
	pRobot->setPlatform( 4, Vector3d( 0,  platformWidth/2, 0));
	pRobot->setPlatform( 5, Vector3d( 0, -platformWidth/2, 0));
	pRobot->setPlatform( 6, Vector3d( 0, 0,  platformHeight/2));
	pRobot->setPlatform( 7, Vector3d( 0, 0, -platformHeight/2));
	pRobot->setPlatform( 8, Vector3d( 0,  platformWidth/2, 0));
	pRobot->setPlatform( 9, Vector3d( 0, -platformWidth/2, 0));
	pRobot->setPlatform(10, Vector3d( platformLength/2, 0,  0));
	pRobot->setPlatform(11, Vector3d(-platformLength/2, 0,  0));

	//! the assignment of the vertices from platform to base is taken from Verhoeven 2004, p.96, Fig. 5.5f
	pRobot->setBase( 0, Vector3d(  0,  0,  frameHeight/2));
	pRobot->setBase( 1, Vector3d(  0,  0,  frameHeight/2));
	pRobot->setBase( 2, Vector3d(  0,    frameWidth/2, 0));
	pRobot->setBase( 3, Vector3d(  0,    frameWidth/2, 0));
	pRobot->setBase( 4, Vector3d(   frameLength/2, 0,  0));
	pRobot->setBase( 5, Vector3d(   frameLength/2, 0,  0));
	pRobot->setBase( 6, Vector3d(  0,  -frameWidth/2,  0));
	pRobot->setBase( 7, Vector3d(  0,  -frameWidth/2,  0));
	pRobot->setBase( 8, Vector3d(  -frameLength/2, 0,  0));
	pRobot->setBase( 9, Vector3d(  -frameLength/2, 0,  0));
	pRobot->setBase(10, Vector3d(  0,  0, -frameHeight/2));
	pRobot->setBase(11, Vector3d(  0,  0, -frameHeight/2));

	return true;
}

// implementation of CKawamura8Design
/////////////////////////////////////////////////////////////////////

CKawamura8Design::CKawamura8Design(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "Kawamura8Design";
	now = 8; 
	nop = 6;
	frameLength = 1;		
	frameWidth = 1;		
	frameHeight = 0;		
	platformLength = 0.1;
	platformWidth = 0.1;
	platformHeight = 0.1;
	motionPattern = CRobotData::MP3R3T;
	generatorScope = GSeffectFrame | GSeffectPlatform;
}
	
void CKawamura8Design::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(frameLength,   "frameLength",		"generator/Kawamura8Design/@frameLength");
	pReflector->bind(frameWidth,    "frameWidth",		"generator/Kawamura8Design/@frameWidth");
	pReflector->bind(frameHeight,   "frameHeight",		"generator/Kawamura8Design/@frameHeight");
	pReflector->bind(platformLength,"platformLength",   "generator/Kawamura8Design/@platformLength");
	pReflector->bind(platformWidth, "platformWidth",    "generator/Kawamura8Design/@platformWidth");
	pReflector->bind(platformHeight,"platformHeight",   "generator/Kawamura8Design/@platformHeight");
}

bool CKawamura8Design::setGeometry()
{
	if (pRobot->getNow() != now)
		return false;

	pRobot->setPlatform( 0, Vector3d(-platformLength/2, 0,   platformHeight/2));
	pRobot->setPlatform( 1, Vector3d( platformLength/2, 0,   platformHeight/2));
	pRobot->setPlatform( 2, Vector3d( platformLength/2, 0,	 platformHeight/2));
	pRobot->setPlatform( 3, Vector3d(-platformLength/2, 0,	 platformHeight/2));
	pRobot->setPlatform( 4, Vector3d( 0,  platformWidth/2,	-platformHeight/2));
	pRobot->setPlatform( 5, Vector3d( 0,  platformWidth/2,	-platformHeight/2));
	pRobot->setPlatform( 6, Vector3d( 0, -platformWidth/2,	-platformHeight/2));
	pRobot->setPlatform( 7, Vector3d( 0, -platformWidth/2,	-platformHeight/2));

	//! the assignment of the vertices from platform to base is taken from Verhoeven 2004, p.96, Fig. 5.5c
	pRobot->setBase( 0, Vector3d( -frameLength,  frameWidth,  frameHeight));
	pRobot->setBase( 1, Vector3d(  frameLength,  frameWidth,  frameHeight));
	pRobot->setBase( 2, Vector3d(  frameLength, -frameWidth,  frameHeight));
	pRobot->setBase( 3, Vector3d( -frameLength, -frameWidth,  frameHeight));
	pRobot->setBase( 4, Vector3d( -frameLength,  frameWidth,  0));
	pRobot->setBase( 5, Vector3d(  frameLength,  frameWidth,  0));
	pRobot->setBase( 6, Vector3d(  frameLength, -frameWidth,  0));
	pRobot->setBase( 7, Vector3d( -frameLength, -frameWidth,  0));

	return true;
}

// implementation of CTranslational9Design
/////////////////////////////////////////////////////////////////////
CTranslational9Design::CTranslational9Design(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "Translational9Design";
	now = 9; 
	nop = 6;
	frameLength = 1;		
	frameHeight = 1;		
	platformLength = 0.05;
	motionPattern = CRobotData::MP3R3T;
	generatorScope = GSeffectFrame | GSeffectPlatform;
}
	
void CTranslational9Design::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(frameLength,   "frameLength",		"generator/Translational9Design/@frameLength");
	pReflector->bind(frameHeight,   "frameHeight",		"generator/Translational9Design/@frameHeight");
	pReflector->bind(platformLength,"platformLength",   "generator/Translational9Design/@platformLength");
}

bool CTranslational9Design::setGeometry()
{
	if (pRobot->getNow() != now)
		return false;
	pRobot->setPlatform( 0, Vector3d(-platformLength/2, (1/3.0)*sqrt(3.0)*platformLength, 0));
	pRobot->setPlatform( 1, Vector3d( platformLength/2, (1/3.0)*sqrt(3.0)*platformLength, 0));
	pRobot->setPlatform( 2, Vector3d(                0, 0, 0));
	pRobot->setPlatform( 3, Vector3d( (3/4.0)*platformLength, (1/12.0)*sqrt(3.0)*platformLength, 0));
	pRobot->setPlatform( 4, Vector3d( (1/4.0)*platformLength, -(5/12.0)*sqrt(3.0)*platformLength, 0));
	pRobot->setPlatform( 5, Vector3d( 0, 0, 0));
	pRobot->setPlatform( 6, Vector3d(-platformLength/4, -(5/12.0)*sqrt(3.0)*platformLength, 0));
	pRobot->setPlatform( 7, Vector3d(-(3/4.0)*platformLength,  (1/12.0)*sqrt(3.0)*platformLength, 0));
	pRobot->setPlatform( 8, Vector3d( 0,  0, 0));

	//! set the frame
	pRobot->setBase( 0, Vector3d( -platformLength/2,  sqrt(1/3.0)*frameLength,  frameHeight));
	pRobot->setBase( 1, Vector3d(  platformLength/2,  sqrt(1/3.0)*frameLength,  frameHeight));
	pRobot->setBase( 2, Vector3d(				  0,  sqrt(1/3.0)*(frameLength-platformLength),  frameHeight));
	pRobot->setBase( 3, Vector3d(  (1/4.0)*platformLength+(1/2.0)*frameLength, (1/4.0)*sqrt(3.0)*platformLength-(1/6.0)*sqrt(3.0)*frameLength,  frameHeight));
	pRobot->setBase( 4, Vector3d( -(1/4.0)*platformLength+(1/2.0)*frameLength, -(1/4.0)*sqrt(3.0)*platformLength-(1/6.0)*sqrt(3.0)*frameLength,  frameHeight));
	pRobot->setBase( 5, Vector3d( (1/2.0)*sqrt(3.0)*((1/3.0)*sqrt(3.0)*frameLength-(1/3.0)*sqrt(3.0)*platformLength), -(1/6.0)*sqrt(3.0)*frameLength+(1/6.0)*sqrt(3.0)*platformLength,  frameHeight));
	pRobot->setBase( 6, Vector3d( (1/4.0)*platformLength-(1/2.0)*frameLength, -(1/4.0)*sqrt(3.0)*platformLength-(1/6.0)*sqrt(3.0)*frameLength,  frameHeight));
	pRobot->setBase( 7, Vector3d( -(1/4.0)*platformLength-(1/2.0)*frameLength,  (1/4.0)*sqrt(3.0)*platformLength-(1/6.0)*sqrt(3.0)*frameLength,  frameHeight));
	pRobot->setBase( 8, Vector3d(  -(1/2.0)*sqrt(3.0)*((1/3.0)*sqrt(3.0)*frameLength-(1/3.0)*sqrt(3.0)*platformLength),  -(1/6.0)*sqrt(3.0)*frameLength+(1/6.0)*sqrt(3.0)*platformLength,  frameHeight));

	return true;
}

// implementation of CParallelogram6Design
/////////////////////////////////////////////////////////////////////
CParallelogram6Design::CParallelogram6Design(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "Parallelogram6Design";
	now = 6; 
	nop = 4;
	frameRadius = 1.0;	
	platformRadius = 0.1;
	alpha = MO_PI/3.0;
	parallelDistance = 2*platformRadius;
	motionPattern = CRobotData::MP3R3T;
	generatorScope = GSeffectFrame | GSeffectPlatform;
}
	
void CParallelogram6Design::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(frameRadius,		"frameRadius",		"generator/Parallelogram6Design/@frameRadius");
	pReflector->bind(platformRadius,	"platformRadius",	"generator/Parallelogram6Design/@platformRadius");
	pReflector->bind(alpha,				"alpha",			"generator/Parallelogram6Design/@alpha");
	pReflector->bind(parallelDistance,	"parallelDistance",	"generator/Parallelogram6Design/@parallelDistance");
}

bool CParallelogram6Design::setGeometry()
{
	if (pRobot->getNow() != now)
		return false;
	pRobot->setPlatform( 0, Vector3d(  0.5, -0.5*sqrt(3.0), 0)*platformRadius);
	pRobot->setPlatform( 1, Vector3d( -0.5,  0.5*sqrt(3.0), 0)*platformRadius);
	pRobot->setPlatform( 2, Vector3d(  0.5,  0.5*sqrt(3.0), 0)*platformRadius);
	pRobot->setPlatform( 3, Vector3d( -0.5, -0.5*sqrt(3.0), 0)*platformRadius);
	pRobot->setPlatform( 4, Vector3d( -1.0,            0.0, 0)*platformRadius);
	pRobot->setPlatform( 5, Vector3d(  1.0,            0.0, 0)*platformRadius);

	//! set the frame
	pRobot->setBase( 0, Vector3d( 1.0,           0.0, 0)*frameRadius + pRobot->getPlatform(0));
	pRobot->setBase( 1, Vector3d( 1.0,           0.0, 0)*frameRadius + pRobot->getPlatform(1));
	pRobot->setBase( 2, Vector3d(-0.5, 0.5*sqrt(3.0), 0)*frameRadius + pRobot->getPlatform(2));
	pRobot->setBase( 3, Vector3d(-0.5, 0.5*sqrt(3.0), 0)*frameRadius + pRobot->getPlatform(3));
	pRobot->setBase( 4, Vector3d(-0.5,-0.5*sqrt(3.0), 0)*frameRadius + pRobot->getPlatform(4));
	pRobot->setBase( 5, Vector3d(-0.5,-0.5*sqrt(3.0), 0)*frameRadius + pRobot->getPlatform(5));

	return true;
}


// implementation of CTriangularFrame
/////////////////////////////////////////////////////////////////////
CTriangularFrame::CTriangularFrame(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "TriangularFrame";
	now = 3; 
	nop = 1;
	frameRadius = 1;		
	motionPattern = CRobotData::MP3T | CRobotData::MP2T | CRobotData::MP1R2T;
	generatorScope = GSeffectFrame;
}
	
void CTriangularFrame::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(frameRadius,   "frameRadius",		"generator/TriangularFrame/@frameRadius");
}

bool CTriangularFrame::setGeometry()
{
	if (pRobot->getNow() != now)
		return false;

	//! set the frame
	pRobot->setBase( 0, Vector3d( 0,  frameRadius,  0));
	pRobot->setBase( 1, Vector3d( sqrt(0.75)*frameRadius, -frameRadius/2.0, 0 ));
	pRobot->setBase( 2, Vector3d( -sqrt(0.75)*frameRadius, -frameRadius/2.0, 0 ));

	return true;
}

// implementation of CPointPlatform
/////////////////////////////////////////////////////////////////////

CPointPlatform::CPointPlatform(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "PointPlatform";
	now = -1; 
	nop = 0;
	motionPattern = CRobotData::MP2T | CRobotData::MP3T;
	generatorScope = GSeffectPlatform;
}
	
bool CPointPlatform::setGeometry()
{
	if (pRobot->getNow() <= 0)
		return false;
	
	for (int i=0; i<pRobot->getNow(); i++)
		pRobot->setPlatform(i,Vector3d(0,0,0));

	return true;
}

// implementation of CPointFrame
/////////////////////////////////////////////////////////////////////

CPointFrame::CPointFrame(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "PointFrame";
	now = -1; 
	nop = 0;
	motionPattern = CRobotData::MP3R3T;
	generatorScope = GSeffectFrame;
}
	
bool CPointFrame::setGeometry()
{
	if (pRobot->getNow() <= 0)
		return false;
	
	for (int i=0; i<pRobot->getNow(); i++)
		pRobot->setBase(i,Vector3d(0,0,0));

	return true;
}

// implementation of CPlatformTransformator
/////////////////////////////////////////////////////////////////////

CPlatformTransformator::CPlatformTransformator(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "PlatformTransformator";
	now = -1;							// any number of wires is allowed: 
	nop = 9;
	dx = dy = dz = 0;
	da = db = dc = 0;
	Sx = Sy = Sz = 1;
	motionPattern = 0xFFFFFFFF;		// applicable for any motion pattern
	generatorScope = GSeffectPlatform | GSTransformator;
}
	
void CPlatformTransformator::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(dx,"dx","generator/PlatformTransformator/@dx");
	pReflector->bind(dy,"dy","generator/PlatformTransformator/@dy");
	pReflector->bind(dz,"dz","generator/PlatformTransformator/@dz");
	pReflector->bind(da,"da","generator/PlatformTransformator/@da");
	pReflector->bind(db,"db","generator/PlatformTransformator/@db");
	pReflector->bind(dc,"dc","generator/PlatformTransformator/@dc");
	pReflector->bind(Sx,"Sx","generator/PlatformTransformator/@Sx");
	pReflector->bind(Sy,"Sy","generator/PlatformTransformator/@Sy");
	pReflector->bind(Sz,"Sz","generator/PlatformTransformator/@Sz");
}

bool CPlatformTransformator::setGeometry()
{
	// translate all parameters
	pRobot->translatePlatformGeometry(Vector3d(dx,dy,dz));

	// rotate all parameters	
	Matrix3d R = Matrix3d::ZRotationMatrix3d(dc*DEG_TO_RAD)
			*Matrix3d::YRotationMatrix3d(db*DEG_TO_RAD)
			*Matrix3d::XRotationMatrix3d(da*DEG_TO_RAD);
	pRobot->rotatePlatformGeoemetry(R);

	// scale all parameter
	pRobot->scalePlatformGeometry(Vector3d(Sx,Sy,Sz));

	return true;
}

// implementation of CFrameTransformator
/////////////////////////////////////////////////////////////////////

CFrameTransformator::CFrameTransformator(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "FrameTransformator";
	now = -1;							// any number of cables is allowed: 
	nop = 9;
	dx = dy = dz = 0;
	da = db = dc = 0;
	Sx = Sy = Sz = 1;
	motionPattern = 0xFFFFFFFF;		// applicable for any motion pattern
	generatorScope = GSeffectFrame | GSTransformator;
}
	
void CFrameTransformator::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(dx,"dx","generator/FrameTransformator/@dx");
	pReflector->bind(dy,"dy","generator/FrameTransformator/@dy");
	pReflector->bind(dz,"dz","generator/FrameTransformator/@dz");
	pReflector->bind(da,"da","generator/FrameTransformator/@da");
	pReflector->bind(db,"db","generator/FrameTransformator/@db");
	pReflector->bind(dc,"dc","generator/FrameTransformator/@dc");
	pReflector->bind(Sx,"Sx","generator/FrameTransformator/@Sx");
	pReflector->bind(Sy,"Sy","generator/FrameTransformator/@Sy");
	pReflector->bind(Sz,"Sz","generator/FrameTransformator/@Sz");
}

bool CFrameTransformator::setGeometry()
{
	// translate all parameters
	pRobot->translateFrameGeometry(Vector3d(dx,dy,dz));

	// rotate all parameters	
	Matrix3d R = Matrix3d::ZRotationMatrix3d(dc*DEG_TO_RAD)
			*Matrix3d::YRotationMatrix3d(db*DEG_TO_RAD)
			*Matrix3d::XRotationMatrix3d(da*DEG_TO_RAD);
	pRobot->rotateFrameGeoemetry(R);

	// scale all parameter
	pRobot->scaleFrameGeometry(Vector3d(Sx,Sy,Sz));

	return true;
}

// implementation of CGeometryPermutator
/////////////////////////////////////////////////////////////////////

CGeometryPermutator::CGeometryPermutator(CRobotData& robot) 
	: CGeometryGenerator(robot) 
{
	name = "GeometryPermutator";
	now = -1;							// any number of cables is allowed: 
	nop = 0;							// we do not have really parameters
	motionPattern = 0xFFFFFFFF;		// applicable for any motion pattern
	generatorScope = GSeffectFrame | GSTransformator | GSTransformator; // effects both platform and base; the geometry is modified

	effectPlatform = true;			// by default the permutation allies to platform
	effectBase = true;				// and base

	// setup the initial map wiht a 1:1 mapping of the robot's legs
	for (unsigned int i=0; i<(unsigned int)pRobot->getNow(); i++)
		connectionMap.push_back(i);
}

CGeometryPermutator::~CGeometryPermutator()
{
}

void CGeometryPermutator::bind()
{
	CGeometryGenerator::bind();

	if (!pReflector)
		return;
	
	pReflector->bind(effectBase,"effectBase","generator/GeometryPermutator/@effectBase");
	pReflector->bind(effectPlatform,"effectPlatform","generator/GeometryPermutator/@effectPlatform");
}

bool CGeometryPermutator::setPermutation(vector<unsigned int> &table)
{
	// check size of the table
	if (table.size() > connectionMap.size())
		return false;
	// check the entries of the table not to exceed the number of wires
	for (unsigned int i=0; i<table.size(); i++)
		if (table[i] > (unsigned int)pRobot->getNow())
			return false;
	// copy the table into the internal storage
	for (unsigned int i=0; i<table.size(); i++)
		connectionMap[i] = table[i];
	return true;
}

//! use the permutation map given by a plain C array with given size.
//! clearly, we cannot check if size is the real size.
bool CGeometryPermutator::setPermutation(unsigned int* table, const unsigned int &size)
{
	// check for table being empty or size of the table begin to large
	if (table == 0 || size > connectionMap.size())
		return false;
	// check the entries of the table not to exceed the number of wires
	for (unsigned int i=0; i<size; i++)
		if (table[i] > (unsigned int)pRobot->getNow())
			return false;
	// copy the table into the internal storage
	for (unsigned int i=0; i<size; i++)
		connectionMap[i] = table[i];
	return true;}


//! modify the permutation map by exchanging the ids given by first and second.
//! returns true if successful, otherwise (e.g. because first and second are 
//! invalid) false.
bool CGeometryPermutator::flip(unsigned int first, unsigned int second)
{
	if (first > (unsigned int)pRobot->getNow() || second > (unsigned int)pRobot->getNow())
		return false;
	swap(connectionMap[first],connectionMap[second]);
	return true;
}

/*! make a cycling replacement within the 0-based range from begin to end (including end).
 *  the range cycle(1,2) will exchange winch 2 (id 1) with 3 (id 2). cycle (1,3) will reorder
 *  the original sequence (0,1,2,3,4,5) -> (0,2,3,1,4,5). note that you can cycle in the other
 *  direction by exchaning the boarder. thus, cycle (3,1) will undo the change generated
 *  by cycle(1,3).
 *  \param begin [in] the first zero-based element to be cycled
 *  \param end [in] the last zero-based element to be cycled
 *  \returns false, if the parameters where unfeasible (mostly out of range), true if
 *  cycling was successful.
 */
bool CGeometryPermutator::cycle(unsigned int begin, unsigned int end)
{
	// limits valid?
	if (begin > (unsigned int)pRobot->getNow() || end > (unsigned int)pRobot->getNow())
		return false;
	// limits equal
	if (begin == end)
		return false;
	// have positive increments when forward cycling, otherwise negative
	if (begin < end)
	{
		for (unsigned int i=begin; i<end; i++)
			swap(connectionMap[i],connectionMap[i+1]);
	}
	else
	{
		for (unsigned int i=end; i>begin; i--)
			swap(connectionMap[i],connectionMap[i-1]);
	}
	return true;
}

//! change the connectionMap by exchanging element specified in table
bool CGeometryPermutator::cycle(vector<unsigned int> &table)
{
	// only valid elements
	for (unsigned int i=0; i<table.size(); i++)
		if (table[i] > (unsigned int)pRobot->getNow())
			return false;
	// perform the forward cycling
	for (unsigned int i=0; i<table.size()-1; i++)
		swap(connectionMap[table[i]],connectionMap[table[i+1]]);
	return true;
}

//! execute the permutation defined by the internal connectionMap. 
bool CGeometryPermutator::setGeometry()
{
	// check if connectionMap is feasible
	if (connectionMap.size() > (unsigned int)pRobot->getNow())
		return false;
	// use the connectionMap to exchange the
	if (effectPlatform) // apply to platform
	{
		// generate a temporary map
		Vector3d* tempMap = new Vector3d[pRobot->getNow()];
		// assign the new values to the temp map
		for (unsigned int i=0; i < (unsigned int)pRobot->getNow(); i++)
			tempMap[i] = pRobot->getPlatform(connectionMap[i]);
		// assign the temp map to the robot geometry
		for (unsigned int i=0; i < (unsigned int)pRobot->getNow(); i++)
			pRobot->setPlatform(i,tempMap[i]);
		// delete the temp map
		delete [] tempMap;
	}

	if (effectBase) // apply to base
	{
		// generate a temporary map
		Vector3d* tempMap = new Vector3d[pRobot->getNow()];
		// assign the new values to the temp map
		for (unsigned int i=0; i < (unsigned int)pRobot->getNow(); i++)
			tempMap[i] = pRobot->getBase(connectionMap[i]);
		// assign the temp map to the robot geometry
		for (unsigned int i=0; i < (unsigned int)pRobot->getNow(); i++)
			pRobot->setBase(i,tempMap[i]);
		// delete the temp map
		delete [] tempMap;
	}
	return true;
}

// implementation of CGeometryGeneratorList
/////////////////////////////////////////////////////////////////////

//! create a list of all default geometry generators. This class shall be used as a top
//! level interface to include all parametric designs at the same time
CGeometryGeneratorList::CGeometryGeneratorList(CRobotData& robot) : CAlgorithm(robot) 
{
	// add all build-in generator here
	push_back(new CParametricBoxFrame(robot));
	push_back(new CParametricBoxPlatform(robot));
	push_back(new CIPAnema1Design(robot));
	push_back(new CIPAnema2Frame(robot));
	push_back(new CIPAnema3Design(robot));
	push_back(new CFalcon7Design(robot));
	push_back(new CCoGiRo8Design(robot));
	push_back(new CMotionSimulatorDesign(robot));
	push_back(new CSSM6Design(robot));
	push_back(new CPlanarRobot(robot));
	push_back(new CFrameTransformator(robot));
	push_back(new CPlatformTransformator(robot));
	push_back(new CEndlessZ9Design(robot));
	push_back(new CEndlessZ12Design(robot));
	push_back(new CFrenchGermanDesign(robot));
	push_back(new CKawamura8Design(robot));
	push_back(new CSegesta7Design(robot));
	push_back(new CSegesta8Design(robot));
	push_back(new CPointPlatform(robot));
	push_back(new CPointFrame(robot));
	push_back(new CTranslational9Design(robot));
	push_back(new CTriangularFrame(robot));
	push_back(new CIPAnema17Design(robot));
	push_back(new CParallelogram6Design(robot));
}

CGeometryGeneratorList::~CGeometryGeneratorList()
{
	for (unsigned int i=0; i<size(); i++)
		delete at(i);
}

//! set the value of a parameter by its name
//! \return true, if the value was successfully written, otherwise false (e.g. if either model or param does not exist)
bool CGeometryGeneratorList::setParameter(const string& model, const string& param, const double& value)
{
	// we have to loop through the list of all parameterizations to model 
	for (unsigned int i=0; i<size(); i++)
	{
		if (at(i)->getName() == model)
			return at(i)->Reflector().set(param,value);
	}
	return false;
}

//! get the value of a parameter by its name
//! \return true, if the value was successfully read, otherwise false (e.g. if either model or param does not exist)
bool CGeometryGeneratorList::getParameter(const string& model, const string& param, double& value)
{
	// we have to loop through the list of all parameterizations to model 
	for (unsigned int i=0; i<size(); i++)
	{
		if (at(i)->getName() == model)
			return Reflector().get(param,value);
	}
	return false;
}

//! apply the model of the given name
//! \return true, if the model was successfully applied, otherwise false (e.g. if  model does not exist)
bool CGeometryGeneratorList::applyModel(const string& model)
{
	// we have to loop through the list of all parameterizations to model 
	for (unsigned int i=0; i<size(); i++)
	{
		if (at(i)->getName() == model)
			return at(i)->setGeometry();
	}
	return false;
}


} // end namespace PCRL
