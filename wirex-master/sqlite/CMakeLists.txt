cmake_minimum_required ( VERSION 2.6 )
# project (tinyXML)

# this flag seems to be required to use the code later in a shared library
set (GCC_COVERAGE_COMPILE_FLAGS "-fPIC")

# and pass these falgs to the gcc compiler
add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})

include_directories("${IPA_PROJECT_HOME}")

add_library(sqlite STATIC
	sqlite3.c
)
