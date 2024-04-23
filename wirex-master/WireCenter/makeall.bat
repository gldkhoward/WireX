@ECHO OFF
ECHO WireX Build All Script.
rem Makescript to build the binaries of WireCenter and related projects (including WiPy) 
rem in Release mode

rem Use the environmental variable to set Visual Studio 2010 paths
call "%VS140COMNTOOLS%\vsvars32.bat"

set SolutionDir=%CD%

ECHO "Project Home Is:"
@SET SolutionDir

rem Start the build based on the main project/solution file of WireCenter
ECHO Calling MSBUILD to create WireCenter Binaries.
msbuild WireCenter.2015.sln /p:Configuration=Release /v:m

ECHO Done.
pause