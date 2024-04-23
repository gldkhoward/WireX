cmake_minimum_required( VERSION 2.6 )

# configure the following path to conform with your local settings
set (IPA_PROJECT_HOME ${CMAKE_CURRENT_LIST_DIR}/)

# set the directory where the compiler will store the static libraries
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${IPA_PROJECT_HOME}lib/)

project( ATLAS )

# add the folders that contain CMakeLists.txt files with the subprojects
add_subdirectory(tinyXML)
add_subdirectory(sqlite)
add_subdirectory(motionPlanning)
add_subdirectory(WireLib)
add_subdirectory(WireLibPyBindings)
add_subdirectory(WiPy)
