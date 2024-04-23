# WireX - Python 3 Compatibility Update

## Introduction
WireX is a software library originally developed by Andreas Pott for the analysis and synthesis of cable-driven parallel robots. This README outlines the changes made to update the original WireX project for compatibility with Python 3. 

## Purpose
Why not just use older python? - What's the fun in that, I needed a copy of the repo for work on my capstone project and thought why not try and see if I can change the whole thing to work with a newer/different versions of python, turns out it was quite easy! and also it was interesting exploring how the project was constructed as I planned to develop my own specific software to run my capstone.

## Changes
The project has been updated to support Python 3, reflecting the discontinuation of support for Python 2. Below is a summary of the key changes:

### Python C API Adjustments
- Replaced all Python 2 specific API calls with Python 3 equivalents.
- Updated module initialization to use `PyModuleDef` and `PyInit` functions.
- Changed the way strings are handled to accommodate Python 3's Unicode strings.

### Code Modernization
- Improved error handling to be more in line with Python 3 standards.

### Build System
- Modified the build scripts and instructions to be compatible with Python 3 environments.
- Updated dependency information to ensure all libraries are compatible with Python 3.

## Installation and Usage
To install and use the updated WireX library, follow these steps:

1. Ensure you have Python 3.6 or higher installed.
2. Clone the repository:
   ```bash
   git clone https://github.com/gldkhoward/WireX.git
   ```
3. Build the project (adjust commands as necessary):
   ```bash
   cd wirex-master
   cmake .
   make
   ```
4. Test the installation:
   ```bash
   cd WiPy
   python
   > import WiPy
   ```
where the last command is entered without the > on the interactive python command line. The following can be used to test the lib: 
```bash
> WiPy.Irobot.applyModel("IPAnema1Design")
True
> WiPy.Iws.calculateWorkspace()
True
> WiPy.Iws.calculateWorkspaceProperties()
{'volume': 5.731436529861277, 'surface': 17.789244457956457, 'CoI': (-4.2978881874605485e-17, -9.988050013112543e-18, 1.0000000000000002)}
```
where lines without the > are the expected response

### Changes for Different Python versions

Different version can probably be accomadated idk, things to change:
- Line 40 in WireLibPyBindings.h #include <python3.10/Python.h> -> your version
- Line 43 in WiPy/pyBindings.h #include <python3.10/Python.h> -> your version


### Further Details
The details above cover the very simple changes made to the existing project, for further information refer to the readMe within the project or the following:
- [WireX Repo](https://gitlab.cc-asp.fraunhofer.de/wek/wirex "Original WireX Repo")
- [WireX Presentation](https://www.researchgate.net/publication/334164626_WireX_-_An_Open_Source_Initiative_Scientific_Software_for_Analysis_and_Design_of_Cable-driven_Parallel_Robots "Some additional information on the WireX software")
