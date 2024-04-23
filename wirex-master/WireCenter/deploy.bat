@ECHO OFF
ECHO This script deploys the latest binary files of WireCenter to the predefined location.

rem generate the path for the deployment if it does not exist
rem if not EXIST ..\bin\WireCenter create it .. but how?

if NOT exist ..\bin\ mkdir ..\bin\

rem Deploy the WirecCenter binary
if exist bin\WireCenter.exe copy bin\WireCenter.exe ..\bin\WireCenter\
if exist bin\WireCenterD.exe copy bin\WireCenterD.exe ..\bin\WireCenter\WireCenterD.exe
copy WireCenter\WireCenter.py ..\bin\WireCenter\
copy WireCenter\IPAnema.wcrfx ..\bin\WireCenter\
copy ..\lib\Python27.dll ..\bin\WireCenter\
rem deploy the WireCenter Python Toolkit
copy ..\PythonScripts\*.py ..\bin\WireCenter

rem Deploy the configuration dicrectories
xcopy ..\config\*.* ..\bin\WireCenter /s /e /c /Y

