@echo off
rem * Generate the doxygen help for the project.
rem * This file is part of the WireX project and is used for the generation 
rem * of the project help and documentation system
rem * (c)opyright 2006-2018 Andreas Pott

rem set paths if not already done
rem set DOT_PATH=C:\Programme\GraphViz\bin
set HHC_PATH=C:\Programme\HTML Help Workshop

set GENERATE_HTML=YES
set GENERATE_HTMLHELP=YES
set GENERATE_LATEX=NO
set GRAPHICS_TYPE=OpenGL
set GENERATE_GRAPHS=NO

rem Generate the doxygen part
doxygen Doxyfile.WireCenter
if not errorlevel 0 goto :doxygen_error
copy ipalogo.gif html

rem Compile the latex document
pdflatex WireDoc
if not errorlevel 0 goto :latex_error

rem create Windows HTML Help (WireCenter.chm)
hhc html/index.hhp
if not errorlevel 0 goto :hcc_error
copy html\index.chm .\ATLAS.chm
del html /Q
rd html
goto end

rem ERROR Handling
:hcc_error
ECHO Cound not generate ATLAS.chm
ECHO Maybe Mircosoft HTML Help Compiler (HHC) is 
ECHO not installed or it is not in your PATH.
goto end

:doxygen_error
ECHO Could not generate the html help from sources
ECHO maybe doxygen is not installed on this computer or 
ECHO it is not in your PATH.
goto end

:latex_error
ECHO Could not generate the WireDoc files
ECHO maybe pdflatex is not installed on this computer or 
ECHO it is not in your PATH.
goto end

:end
ECHO Finished
