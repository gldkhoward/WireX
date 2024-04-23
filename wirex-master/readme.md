Readme WireX project
====================
> WireX -- Scientic Software for Analysis and Design of Cable-driven Parallel Robots

Copyright (c) 2006-2019 Andreas Pott


Content
-------

The WireX repository is a collection of libraries and programs for cable-driven parallel robots.
The main projects are

- **WireLib**: a C++ class library for analysis of cable robots (kinematics, statics, 
  workspace, design)

- **motionPlanning**: a C++ class library for general robotics motion description including a
  NC-path interpreter and interpolator

- **WireCenter**: A Windows GUI for visual editing, analysis, and design of cable robots. 

- **WiPy**: A stand-alone cross-platform Python extension that allows to use WireLib in Python.

- **IPAGL**: a C++ class library for openGL in Windows MFC applications.

- **PythonScripts**: A collection of useful python helpers, visualizers, and unittest for 
  WireCenter, WiPy, and WireLib.

Furthermore, the sources of some third party libraries are included for ease of use. ```sqlite``` 
(database), ```tinyXml``` (XML parser), ```eigen3``` (linear algebra), ```python 2.7``` 
(scripting), ```levmar``` (optimization).


Getting started (Windows)
-------------------------

On the Windows platform and using Visual Studio 2015, one has to simply open the solution file
```WireCenter/WireCenter.sln``` and compile the projects ```WireCenter``` in release mode. The
software can be started from Visual Studio. The binary of WireCenter is built
in ```WireCenter/bin/WireCenter.exe```.

Some nice ressources such as sample robots and sample nc files can be bundled. Use the script
```WireCenter/deploy.bat```. If the force is with you, you will be able to run 
```WireCenter/bin/WireCenter.exe``` afterwards.

Compiling can also be done using a new build script (Windows). Navigate the explorer 
to ```WireCenter/``` and start ```buildAll.bat``` or type on command line

    cd WireCenter
    buildAll
    
This triggers a compile-all in release mode and create ```WireCenter.exe``` as well as the
python extension libraray ```WiPy.pyd```. To use the latter you must copy this file 
to ```path/to/your/python/lib/site-packages```.


Getting started (Linux)
-----------------------

On linux a ```cmake``` based system is used to compile the main library ```WireLib``` and its 
dependencies. As the Linux version has no GUI, access is provided only through the Python 
interfaces ```WiPy```. To compile WireX from source you need cmake and gcc. On command line use

    cmake .
    make

To compile all required libraries. There is (yet) no deployment scripts and you need to copy the
file WiPy.so manually to the python include dir (usually ```.../python2.7/site-pacakges```). 
To test if compiling was successful and WiPy is ready to go, change to WiPy/ directory

    cd WiPy
    python
    > import WiPy
    
where the last command is entered without the > on the interactive python command line. A rather
simple example on the command line is

    > Irobot.applyModel("IPAnema1Design")
    > Iws.calculateWorkspace()
    > Iws.calculateWorkspaceProperties()

If everything went fine, you should see the results of the workspace computation in terms of 
the volume and surface of the workspace. 


Docu
----

The WireX project containts two types of documentation. Firstly, a dedicated written manual
to be found under ```doc/```. The documentation is written in LaTeX and can be translated using

    pdflatex WireDoc.tex

Open the ```doc/WireDoc.pdf``` afterwards. 
Furthermore, most subprojects in WireX are written in C++ and feature inline documentation in
doxygen syntax. A doxyfile can be found in ```doc/```. Use the batch script 
  
    cd doc
    dox.bat 

to compile all available docu from source (given you have installed the respective tools).


Version numbers and build information
-------------------------------------

Major and minor version numbers are assigned by the project mangement. The build
number for WireLib is extracted with git on the master branch. 

    git rev-list --count HEAD    

The respective number must be manually edited in ```WireLib/WireLib.h``` .


Code Snippet for dry running the CI Pipeline
--------------------------------------------

Call the following command line to execute locally on a Windows system the 
.gitlab-ci.yml file with a gitlab runner

    /c/Portable/runner-gitlab/gitlab-runner-windows-386.exe exec shell test


Conventions for project Files 
-----------------------------

The version number (e.g. 2008 or 2015) is added to the filename of the visual 
studio project file ```Projectfiles XYZ.vcproj``` to allow for use of different
versions of Visual Studio. Note, that as of today Visual Studio files are saved 
for Version 2015 where many filenames still are named 2008.


from tad/asp, 06.04.2010, latest update: 02.06.2019
