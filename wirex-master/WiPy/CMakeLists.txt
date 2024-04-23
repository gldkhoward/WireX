cmake_minimum_required( VERSION 2.6 )

project( WiPy)

find_package(PythonInterp)
find_package(PythonLibs)

INCLUDE_DIRECTORIES(${IPA_PROJECT_HOME})
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
LINK_DIRECTORIES(/usr/local/lib)
LINK_DIRECTORIES(${IPA_PROJECT_HOME}/lib)

OPTION(BUIDL_SHARED_LIBS "turn OFF for .a lib" ON)

# make gcc ignore the warnings
add_definitions("-w")

add_library( WiPy SHARED
	pyBindings.cpp
	WiPy.cpp
)

#SET_TARGET_PROPERTIES(WiPy
#PROPERTIES
#	SOVERSION 1.0
#	VERSION 1.0
#)

target_link_libraries( WiPy 
	WireLib 
	tinyXML 
	WireLibPyBindings 
	motionPlanningLib
)

add_custom_command ( TARGET WiPy
	POST_BUILD
	COMMAND cp libWiPy.so WiPy.so
)

# file (RENAME libWiPy.so WiPy.so)

#INSTALL (TARGETS WiPy
#	LIBRARY DESTINATION lib
#)

#find_program(PYTHON "python")
#
#if (PYTHON)
#	set (SETUP_PY_IN "${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in")
#	set (SETUP_PY	 "${CMAKE_CURRENT_BINARY_DIR}/setup.py")
#	set (DEPS        "${CMAKE_CURRENT_SOURCE_DIR}/module/__init__.py")
#	set (OUTPUT	 "${CMAKE_CURRENT_BINARY_DIR}/build/timestamp")
#
#	configure_file(${SETUP_PY_IN} ${SETUP_PY})
#
#	add_custom_command(OUTPUT ${OUTPUT}
#			   COMMAND ${PYTHON} ${SETUP} build
#			   COMMAND ${CMAKE_COMMAND} -E touch ${OUTPUT}
#			   DEPENDS ${DEPS})
#
#	add_custom_target(target ALL DEPENDS ${OUTPUT})
#
#	install(CODE "execute_process(COMMAND ${PYTHON} ${SETUP_PY} install)")
#
#endif()

