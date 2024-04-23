cmake_minimum_required (VERSION 2.6)
# project (WireLib)

# add compiler flags for GCC to ignore some nasty problems with eigen3
# i.e. do not bother with some undeclared class names -fpermissive
# enable c++0x support to allow std::string for filename in ofstreams -std=c++0x
set (GCC_COVERAGE_COMPILE_FLAGS "-fpermissive -w -std=c++0x -fPIC")

# and pass these falgs to the gcc compiler
add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})

include_directories("${IPA_PROJECT_HOME}")

add_library(WireLib STATIC
	Algorithm.cpp
	ApplicationRequirement.cpp
	CableWear.cpp
	ForceLimits.cpp
	GeometryGenerator.cpp
	Interference.cpp
	Kinematics.cpp
	levmar.cpp
	PoseEvaluator.cpp
	RobotData.cpp
	RobotDocument.cpp
	StructureMatrix.cpp
	Workspace.cpp
	Reflection.cpp
	)
