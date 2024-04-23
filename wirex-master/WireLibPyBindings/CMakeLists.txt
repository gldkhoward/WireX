cmake_minimum_required ( VERSION 2.6 )
# project (WireLibPyBindings)

# add compiler flags for GCC to ignore some nasty problems with eigen3
# i.e. do not bother with some undeclared class names -fpermissive
# enable c++0x support to allow std::string for filename in ofstreams -std=c++0x
# set (GCC_COVERAGE_COMPILE_FLAGS "-fpermissive -w -std=c++0x")

# this flag seems to be required to use the code later in a shared library
set (GCC_COVERAGE_COMPILE_FLAGS "-fPIC -w")

# and pass these flags to the gcc compiler
add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})

include_directories("${IPA_PROJECT_HOME}")
include_directories("/usr/include")

add_library(WireLibPyBindings STATIC
	IcontrolBindings.cpp
	IExperimentalBindings.cpp
	IkinBindings.cpp
	IrobotBindings.cpp
	IwsBindings.cpp
	pyHelper.cpp
	)
