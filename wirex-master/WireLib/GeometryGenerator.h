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

/*! \file GeometryGenerator.h
 *
 *	\author   Andreas Pott
 *
 *  \dependency
 *		Eigen3		for IR³ algebra
 *
 *  Base class for generating and manipulating the robot data in instances of 
 *  the class CRobotData.
 *
 *  \remark
 *  The naming scheme of the class is as follows: a post-fix "-design" indicates that
 *  the parameterazion includes platform and base, the post-fix "-platform" or "-frame"
 *  markes parametric model that only apply to the mobile platform or the static base, 
 *  respectively. Please keep this wording for future extensions of the collection.
 *
 *  Beside this, other algorithm might target other sets of parameters. 
 */

#pragma once

#include <string>
#include <vector>
#include <list>
#include "Algorithm.h"

namespace PCRL {

/*! GeometryGenerator is the base class for algorithms that modify the data in 
 *  CRobotData objects. A typical implementation of a derived class 
 *  provides parameterizations for the robot frame and platform. 
 *  Such Parameterizations are typical components in design algorithms for 
 *  dimensional synthesis of robots, i.e. computing the values of the 
 *  design parameters to achieve given requirements or to optimize a design
 *  criterion. The parameterizations are the interface between the reduced 
 *  design space used by the synthesis algorithm and the full model of the robot.
 *  Furthermore, parameterizations are a handy tool to quickly set up and configure
 *  archetypes of robot model such as frame of rectengular box shap. 
 *
 *  The class CGeometryGenerator is inteded to be used as an abstract base class
 *  provided a unified interface for a collection of parameterization and modification
 *  operators. The most important virtual function is setGeometry() which renders the 
 *  model described by an actual implementation into the data model of the CRobotData
 *  object. 
 */
class CGeometryGenerator : public CAlgorithm
{
protected:
	//! an internal enum type for the bitfield generatorScope
	typedef enum { 
		GSeffectPlatform=1,		//!< flag indicating that the algorithm effects the platform
		GSeffectFrame=2, 		//!< flag indicating that the algorithm effects the frame
		GSeffectTechnology=4, 	//!< flag indicating that the algorithm effects technology parameters (such as feasbile cable forces)
		GSTransformator=8 		//!< flag indicating that the algorihtm uses the previous value (it is modified); not set if results is a new value
	} emGeneratorScope;
	int nop;				//!< number of parameter
	int now;				//!< number of cables set by the algorithm
	string name;			//!< symbolic name of the parameterization
	int motionPattern;		//!< a bitfield defining all applicable motion pattern
	int generatorScope;		//!< a bitfield defining what parts and how the robot is effected by the geometry generator
public:
	//! initialize this class with appropriate default parameters
	explicit CGeometryGenerator(CRobotData& robot) : CAlgorithm(robot), nop(0), now(0), motionPattern(0xFFFFFFFF), generatorScope(0xFFFFFFFF) {}
	virtual ~CGeometryGenerator(void) {}

	//! compute and set the geometry data in the robot document
	virtual bool setGeometry()=0;
	//! determine the number of parameters (nop)
	int getNop() const { return nop; }
	//! determine the number of cables (now)
	int getNow() const { return now; }
	//! get the bitfield with the supported motion Pattern
	int getMotionPattern() const { return motionPattern; }
	//! set the value of a parameter where the parameter is determined by its symbolic name
	bool setParameter(const string& name, const double& value);
	//! get the value of a parameter where the parameter is determined by its symbolic name
	bool getParameter(const string& name, double& value);
	//! get a list of all parameters
	void getParamNames(list<string>& names);
	//! return the name of the algorithm for auto-identification purpose
	string& getName() { return name; }
	//! getter functions to ease reading the configuration bitfield
	bool effectsPlatform() const { return (generatorScope & GSeffectPlatform)>0; }
	bool effectsFrame() const { return (generatorScope & GSeffectFrame)>0; }
	bool effectsTechnology() const { return (generatorScope & GSeffectTechnology)>0; }
	bool isTransformator() const { return (generatorScope & GSTransformator)>0; }
};

//! a simple geometry setter for the frame geometry
class CParametricBoxFrame : public CGeometryGenerator
{
public:
	double length;		//!< the length in x direction of the frame
	double width;		//!< the width in y direction of the frame
	double height;		//!< the height in z direction of the frame

public:
	explicit CParametricBoxFrame(CRobotData& robot);
	void bind();
	bool setGeometry();
};

//! a simple geometry setter for the platform geometry
class CParametricBoxPlatform : public CGeometryGenerator
{
public:
	double length;		//!< the length in x direction of the frame
	double width;		//!< the width in y direction of the frame
	double height;		//!< the height in z direction of the frame

public:
	explicit CParametricBoxPlatform(CRobotData& robot);
	void bind();
	bool setGeometry();
};

/*! the IPAnema1 reference design as it was build and used between 2007-2010. The machine frame
 *  was made from aluminium bars, the platform was a bare planar sheet of aluminium with clamped
 *  cables. Around 2009 two new generations of platforms with different parameters were used
 *  and the winches were reconfigured on the frame leading to the so-called IPAnema 1.5 design.
 */
class CIPAnema1Design : public CGeometryGenerator
{
public:
	double frameLength;		//!< the length in x direction of the frame; nominal value = 4 m
	double frameWidth;		//!< the width in y direction of the frame; nominal value = 3 m
	double frameHeight;		//!< the height in z direction of the frame; nominal value = 2 m
	double platformLength;	//!< the height in x direction of the platform; nominal value = 0.12 m
	double platformWidth;	//!< the height in y direction of the platform; nominal value = 0.12 m

public:
	explicit CIPAnema1Design(CRobotData& robot);
	void bind();
	bool setGeometry();
};

//! IPAnema 2 design scheme
class CIPAnema2Frame : public CGeometryGenerator
{
public:
	double length;		//!< the length in x direction of the frame
	double width;		//!< the width in y direction of the frame
	double height;		//!< the height in z direction of the frame
	double h0;			//!< the height of the lower winches in z-direction
	double delta;		//!< the offset from the corners of the frame in xy-direction

public:
	explicit CIPAnema2Frame(CRobotData& robot);
	void bind();
	bool setGeometry();
};

/*! IPAnema 3 design scheme for platform and machine frame
 *  The geometry model is the nominal geometry of the IPAnema 3 robot in 2013
 *  for initial operation of the robot in the IPA's lab Industriestrasse after
 *  applying the IPAnema 3 winch modules.
 *  The basic geometry model mostly coincides with the geometry model used for
 *  the IPAnema 2 system but now both frame and platform have a xy shift in the
 *  anchor points for crossing cables. 
 */ 
class CIPAnema3Design : public CGeometryGenerator
{
public:
	double frameLength;			//!< dimension in x-direction of the robot frame; nominal value=1.45m
	double frameWidth;			//!< dimension in y-direction of the robot frame; nominal value=1.45m
	double frameHeight;			//!< height of the robots frame in z-direction for winch 1-4
	double frameHeight0;		//!< base distance from the xy plane; this parameter is
	double frameDeltaLength;	//!< the reduction of length for winches 1-4
	double frameDeltaWidth;		//!< the reduction of width for the winches 5-8
	double platformLength;		//!< dimension in x-direction of the mobile platform; this parameter is added and its nomial value is 0
	double platformWidth;		//!< dimension in y-direction of the mobile platform; nominal value = 0.1; this paper is not given in the paper but estimated from the pictures
	double platformHeight;		//!< dimension in z-direction of the mobile platform, i.e. the length of the rod; nominal value 1
	double platformDeltaLength;	//!< the reduction of length for anchor points 1-4
	double platformDeltaWidth;	//!< the reduction of width for anchor points 5-8

public:
	explicit CIPAnema3Design(CRobotData& robot); 
	void bind();
	bool setGeometry();
};


/*! Falcon7 design scheme for platform and machine frame
 *  the geometry model is reverse-engineered from the Kawamura's paper 
 *  "Development of an ultrahigh speed robot FALCON using wire drive system" by
 *  S. Kawamura, W. Choe, S. Tanaka, and S.R.Pandian, 1993.
 *  The configuration of the robot was reconstructed by the text and the figures
 *  The nomincal numbers are taken from the desciption. Additionally, we introduce
 *  some parameters which a nominal value of 0 (i.e. that could be neglected in the
 *  nominal design). The paper is not very accurate with the nominal parameters. Values are
 *  given for the frame but it is not clear if these are the locations of the effective 
 *  anchor points a_i and b_i or only the maximum dimension of the frame.
 */ 
class CFalcon7Design : public CGeometryGenerator
{
public:
	double frameLength;			//!< dimension in x-direction of the robot frame; nominal value=1.45m
	double frameWidth;			//!< dimension in y-direction of the robot frame; nominal value=1.45m
	double frameHeight;			//!< height of the robots frame in z-direction; this parameter is not present in the paper and gives the distance between the winches acting on the upper and lower end of the rod
	double frameHeight0;		//!< base distance from the xy plane; this parameter is kinematically not relevant but represents the setup of having the winches at the upper bars of a frame; nominal value 1.25m
	double platformLength;		//!< dimension in x-direction of the mobile platform; this parameter is added and its nomial value is 0
	double platformWidth;		//!< dimension in y-direction of the mobile platform; nominal value = 0.1; this paper is not given in the paper but estimated from the pictures
	double platformHeight;		//!< dimension in z-direction of the mobile platform, i.e. the length of the rod; nominal value 1

public:
	explicit CFalcon7Design(CRobotData& robot); 
	void bind();
	bool setGeometry();
};

/*! IPAnema1_7 design scheme for platform and machine frame
 *  The origianl seven winch design of the first IPAnema system as of 2008/2009.
 *  The robot is mentioned in the ARK2008 paper. 
 */ 
class CIPAnema17Design : public CGeometryGenerator
{
public:
	double frameLength;			//!< dimension in x-direction of the robot frame; nominal value=4.0m
	double frameWidth;			//!< dimension in y-direction of the robot frame; nominal value=3.0m
	double frameHeight;			//!< height of the robots frame in z-direction; nominal value 2.0m
	double platformLength;		//!< dimension in x-direction of the mobile platform; nominal value 0.25m
	double platformWidth;		//!< dimension in y-direction of the mobile platform; nominal value = 0.25m

public:
	explicit CIPAnema17Design(CRobotData& robot); 
	void bind();
	bool setGeometry();
};

//! Segesta is the prototype designed and tested at the University of Duisburg-Essen, Germany.
//! nominal parameter as well as the design is taken from Diss Bruckmann 2010. Note that the
//! Duisburg convention for the geometry is not based on a vertical z-axis. To conform with the
//! other designs it was adopted such that z-axis is vertical. The frame is a box; the platform is
//! a flat triangle. The design is adopted from the pictures Fig 6.6 (Segesta7) and 6.7 (Segesta8) on p.80-81
//! The Segesta7Design is a 7-3 configuration.
class CSegesta7Design : public CGeometryGenerator 
{
public:
	double frameLength;			//!< dimension in x-direction of the robot frame; nominal value=
	double frameWidth;			//!< dimension in y-direction of the robot frame; nominal value=
	double frameHeight;			//!< dimension in z-direction of the robot frame; nominal value=
	double platformLength;		//!< dimension in x-direction of the mobile platform; nomial value is 0
	double platformWidth;		//!< dimension in y-direction of the mobile platform; nominal value = 0.1

public:
	explicit CSegesta7Design(CRobotData& robot); 
	void bind();
	bool setGeometry();
};

class CSegesta8Design : public CGeometryGenerator 
{
public:
	double frameLength;			//!< dimension in x-direction of the robot frame; nominal value=
	double frameWidth;			//!< dimension in y-direction of the robot frame; nominal value=
	double frameHeight;			//!< dimension in z-direction of the robot frame; nominal value=
	double platformLength;		//!< dimension in x-direction of the mobile platform; nomial value is 0
	double platformWidth;		//!< dimension in y-direction of the mobile platform; nominal value = 0.1

public:
	explicit CSegesta8Design(CRobotData& robot); 
	void bind();
	bool setGeometry();
};

/*! Parametric model for robots with the CoGiRo design scheme. The architecture was
 *  used by LIRMM as well as by KNTU for an eight cable robot.
 *  Unfortunately, no nominal parameters are known from the literature. Therefore,
 *  we use a very rough estimate that should be in the scale of the CoGiRo prototype. 
 */
class CCoGiRo8Design : public CGeometryGenerator
{
public:
	double frameLength;			//!< dimension in x-direction of the robot frame; nominal value=18
	double frameWidth;			//!< dimension in y-direction of the robot frame; nominal value=12
	double frameHeight;			//!< height of the robots frame in z-direction; nominal value 1m
	double frameHeight0;		//!< base distance from the xy plane; this parameter is kinematically not relevant but represents the setup of having the winches at the upper bars of a frame; nominal value 7m
	double platformLength;		//!< dimension in x-direction of the mobile platform; nomial value = 1
	double platformWidth;		//!< dimension in y-direction of the mobile platform; nominal value = 1; 
	double platformHeight;		//!< dimension in z-direction of the mobile platform, nominal value = 1

public:
	explicit CCoGiRo8Design(CRobotData& robot); 
	void bind();
	bool setGeometry();
};

/*! Parametric model for the MPI motion simulator consisting of a parametric box frame
 *  and an icosahedron platform. The architecture was used by Miermeister for the MPI
 *  cable motion simulator design. 
 */
class CMotionSimulatorDesign : public CGeometryGenerator
{
public:
	double frameLength;			//!< dimension in x-direction of the robot frame; nominal value=12
	double frameWidth;			//!< dimension in y-direction of the robot frame; nominal value=12
	double frameHeight;			//!< height of the robots frame in z-direction; nominal value 8m
	double platformRadius;		//!< radius of the platform icosahedron 

public:
	explicit CMotionSimulatorDesign(CRobotData& robot); 
	void bind();
	bool setGeometry();
};

/*! The simplified symmetric manipulator is a well known reference design for
 *  parallel robot where both the platform and the base a are planar. The anchor
 *  points are located on a hexagon with the respective radii. When using this
 *  design for a cable robot, it is expected to be operated in a suspended i.e.
 *  crane-like configuration.
 */
class CSSM6Design : public CGeometryGenerator
{
public:
	double frameRadius;			//!< the radius of the base frame. nominal value is 1 meter
	double frameAlpha;			//!< the symmetric displacement angle from the nomial symmetrie axis; nominal value=15 degree
	double platformRadius;		//!< the radius of the base frame. nominal value is 0.1 meter
	double platformAlpha;		//!< the symmetric displacement angle from the nomial symmetrie axis; nominal value=30 degree
public:
	explicit CSSM6Design(CRobotData& robot); 
	void bind();
	bool setGeometry();
};

//! future planed extensions
class CPlanarRobotFrame : public CGeometryGenerator {};

//! a simple planar robot
class CPlanarRobot: public CGeometryGenerator 
{
public:
	double frameLength;			//!< dimension in x-direction of the robot frame; nominal value=18
	double frameWidth;			//!< dimension in y-direction of the robot frame; nominal value=12
	double platformLength;		//!< dimension in x-direction of the mobile platform; nomial value = 1
	double platformWidth;		//!< dimension in y-direction of the mobile platform; nominal value = 1; 
public:
	explicit CPlanarRobot(CRobotData& robot); 
	void bind();
	bool setGeometry();
};

//! endless Z robot with 9-3 configuration
class CEndlessZ9Design : public CGeometryGenerator
{
public:
	double platformHeight;		//!< dimension in z-direction of the platform from tcp to middle platform level
	double platformHeight2;		//!< dimension in z-direction of the platform from tcp to top platform level
	double platformRadius;		//!< dimension in radial direction of the platform for the excentric point
	double frameRadius;			//!< dimension in radial direction of the frame
	double frameHeight;			//!< dimension in z-direction of the frame for the middle frame level
	double frameHeight2;		//!< dimension in x-direction of the frame of the top frame level
public:
	explicit CEndlessZ9Design(CRobotData& robot); 
	void bind();
	bool setGeometry();
};

//! endless Z robot with 12-3 configuration
class CEndlessZ12Design : public CGeometryGenerator
{
public:
	double platformHeight;		//!< dimension in z-direction of the platform from tcp to middle platform level
	double platformHeight2;		//!< dimension in z-direction of the platform from tcp to top platform level
	double platformRadius;		//!< dimension in radial direction of the platform for the excentric point
	double frameRadius;			//!< dimension in radial direction of the frame
	double frameHeight;			//!< dimension in z-direction of the frame for the middle frame level
	double frameHeight2;		//!< dimension in x-direction of the frame of the top frame level
public:
	explicit CEndlessZ12Design(CRobotData& robot); 
	void bind();
	bool setGeometry();
};

//! robot design by Verhoeven and LaFourcade with 12 cables and antagonistic 
//! actuation. The geometry is in a 6-6 configuration with star like
//! platform and octahedron frame. In the related publication the geoemtry is not
//! defined by numbers.
class CFrenchGermanDesign : public CGeometryGenerator
{
public:
	double frameLength;			//!< dimension in x-direction of the robot frame; nominal value=1
	double frameWidth;			//!< dimension in y-direction of the robot frame; nominal value=1
	double frameHeight;			//!< dimension in z-direction of the robot frame; nominal value=1; 
	double platformLength;		//!< dimension in x-direction of the mobile platform; nomial value = 0.1
	double platformWidth;		//!< dimension in y-direction of the mobile platform; nominal value = 0.1; 
	double platformHeight;		//!< dimension in z-direction of the mobile platform, nominal value = 0.1
public:
	explicit CFrenchGermanDesign(CRobotData& robot); 
	void bind();
	bool setGeometry();
};

//! robot design by Kawamura reconstructed from the picture in Verhoeven2004, p. 96.
//! the nominal geometry is a 4-4 configuration but we add frame height that lets us seperate coinciding winches
class CKawamura8Design : public CGeometryGenerator
{
public:
	double frameLength;			//!< dimension in x-direction of the robot frame; nominal value=1
	double frameWidth;			//!< dimension in y-direction of the robot frame; nominal value=1
	double frameHeight;			//!< dimension in z-direction of the robot frame; nominal value=1; 
	double platformLength;		//!< dimension in x-direction of the mobile platform; nomial value = 0.1
	double platformWidth;		//!< dimension in y-direction of the mobile platform; nominal value = 0.1
	double platformHeight;		//!< dimension in z-direction of the mobile platform, nominal value = 0.1
public:
	explicit CKawamura8Design(CRobotData& robot); 
	void bind();
	bool setGeometry();
};

/*! robot design by Dinh-Son Vu, Eric Barnett, Anne-Marie Zaccarin, and Clement 
 *  Gosselin proposed on CableCon 2017. the nominal geometry is a 6-6 purely 
 *  translational robot where pairs of cables shall be driven by a single actuator
 *  \remark The current implementation does not make use of all geometric parameters. 
 *          The paper introduces more parameters but the example ignore the 
 *          parallelDistance as well as the angle alpha
 */
class CParallelogram6Design : public CGeometryGenerator
{
public:
	double frameRadius;		//!< the radius of the robot frame; nominal value=1
	double platformRadius;	//!< the radius of the mobile platform; nominal value=0.1
	double alpha;			//!< the angle of rotation for the parallel cables as given in the paper; nomnial value = -Pi/3
	double parallelDistance;//!< the distance between the parallel cables; nomnial value=2*platformRadius
public:
	explicit CParallelogram6Design(CRobotData& robot); 
	void bind();
	bool setGeometry();
};

//! robot design by O. Saber proposed in a paper draft in 2013.
//! the nominal geometry is a 9-9 purely translational configuration when three groups of each three cables are kept 
//! under equial distances. 
class CTranslational9Design : public CGeometryGenerator
{
public:
	double frameLength;			//!< the length of the frame triangle; nominal value=1
	double frameHeight;			//!< dimension in z-direction of the robot frame; nominal value=1; 
	double platformLength;		//!< the length of the platform triangle; nomial value = 0.05
public:
	explicit CTranslational9Design(CRobotData& robot); 
	void bind();
	bool setGeometry();
};

//! a simpy frame for typically used for a 3T suspended cable robot
class CTriangularFrame : public CGeometryGenerator
{
public:
	double frameRadius;			//! radius of the circumcircle of the triangular base
public:
	explicit CTriangularFrame(CRobotData& robot);
	void bind();
	bool setGeometry();
};

//! extremely simple platform geometry for 2T and 3T robots with any number of cables
//! all vectors b_i are exactly zero
class CPointPlatform : public CGeometryGenerator
{
public:
	explicit CPointPlatform(CRobotData& robot); 
	bool setGeometry();
};

//! extremely simple frame geometry for special robots with any number of cables
//! all vectors a_i are exactly zero.
class CPointFrame : public CGeometryGenerator
{
public:
	explicit CPointFrame(CRobotData& robot); 
	bool setGeometry();
};

//! geometry transformation for the platform
class CPlatformTransformator : public CGeometryGenerator
{
public:
	double dx,dy,dz;				//!< translation of the platform
	double da,db,dc;				//!< rotation of the platform
	double Sx,Sy,Sz;				//!< scaling of the platform
public:
	explicit CPlatformTransformator(CRobotData& robot); 
	void bind();
	bool setGeometry();
};

//! geometry transformation for the frame
class CFrameTransformator : public CGeometryGenerator
{
public:
	double dx,dy,dz;				//!< translation of the frame
	double da,db,dc;				//!< rotation angles of the frame in degree; model is Rz(dc)*Ry(db)*Rx(da)
	double Sx,Sy,Sz;				//!< scaling of the frame
public:
	explicit CFrameTransformator(CRobotData& robot); 
	void bind();
	bool setGeometry();
};

/*! \class CGeometryPermutator
 *  This class allows to change, which proximal anchor points are connected to
 *  the distal anchor pointer, i.e. the class allows to remap the a_i to b_j with
 *  i!=j. The effect can also be achieved by reading and writing anchor point 
 *  data. However, this class emulated the idea of re-connecting cables to 
 *  different points on the platform.
 */
class CGeometryPermutator : public CGeometryGenerator
{
	// we make the connectionMap private. Thus we do all operations on
	// the map such that its entries are valid. therefore, we have to 
	// carefully check for manipulators but we can skip validation before
	// using the map.
	vector<unsigned int> connectionMap;	//!< the map containing the planed permutation
public:
	bool effectPlatform;	//!< boolean flag indicating that permutation is applied to platform
	bool effectBase;		//!< boolean flag indicating that permutation is applied to base

	explicit CGeometryPermutator(CRobotData& robot);
	~CGeometryPermutator();

	//! override the virtual bind function
	void bind();

// configure permutations
	//! setup a permutation table
	bool setPermutation(vector<unsigned int> &table);
	//! setup a permutation table
	bool setPermutation(unsigned int* table, const unsigned int &size);
	//! change the connectionMap by exchanging element first and second
	bool flip(unsigned int first, unsigned int second);
	//! change the connectionMap by exchanging element first and second
	bool cycle(unsigned int begin, unsigned int end);
	//! change the connectionMap by exchanging element specified in table
	bool cycle(vector<unsigned int> &table);
	// bool setPermutation(args);
// operations planed: execute permutations
	//! perform the permutation stored in the local storage for the platform of the robot
	bool permutePlatform();
	//! perform the permutation stored in the local storage for the base of the robot
	bool permuteBase();
	//! permute both platform and base; this should not change the properties of the robot
	bool permuteRobot();
	//! do the permutation specified by the state variables
	bool setGeometry();
	//! cycle the block of anchor points specified by the range
	bool cyclePlatform(const unsigned int begin, const unsigned int end);
	bool cycleBase(const unsigned int begin, const unsigned int end);
};

//! CGeometryGeneratorList is a container for all available geometry generator
class CGeometryGeneratorList : public CAlgorithm, public vector<CGeometryGenerator*>
{
public:
	explicit CGeometryGeneratorList(CRobotData& robot);
	~CGeometryGeneratorList();
	//! set the value of a parameter in the specified model by its name
	bool setParameter(const string& model, const string& param, const double& value);
	//! get the value of a parameter in the specified model by its name
	bool getParameter(const string& model, const string& param, double& value);
	//! call the setGeometry function for the model defined by its name
	bool applyModel(const string& model);
};

}	// end of namespace PCRL
