# CASPR #
The *Cable-robot Analysis and Simulation Platform for Research (CASPR)*  is a MATLAB toolbox that facilitates a range of analysis approaches on arbitrary cable-robot models.
## Purpose: ##
The CASPR toolbox represents a unified development platform for the analysis of cable-driven parallel robots.  The current version of the toolbox contains analysis tools for each of the following fields of study:
*  Dynamics and Control
  * Forward Dynamics
  * Inverse Dynamics
  * Motion Control
* Kinematics
  * Forward Kinematics
  * Inverse Kinematics
* Workspace Analysis
* Design Optimisation

## Environment Requirements: ##
### Operating System: ###
CASPR has been developed in Windows and ideally should be run on 32-bit and 64-bit computers running that operating system. The toolbox has also been tested to work on the Linux Ubuntu distributions from 14.04 onwards.

### MATLAB: ###
As the software is run through MATLAB, it is necessary that MATLAB is first installed before running CASPR. CASPR has been tested for MATLAB versions 2014a onwards. The core of CASPR will work for older versions, however certain newer functions may need to be replaced with the equivalent depreciated function call.

## Installation: ##
1. Download CASPR from the repository **INSERT REPOSITORY**.
2. Extract the downloaded CASPR and place the extracted folder into the directory of your choosing.
3. Open MATLAB and change the file path to be that of the extracted CASPR root directory.
4. Run the script setup_CASPR.m. This will add all of the necessary file paths such that CASPR can be used.

In addition to the core code base, some analysis techniques within CASPR require the use of additional software dependencies. These dependencies include
* 'qhull' - This is a convex hull library that is used throughout the *Workspace* analysis module.
* 'optitoolbox' - This is a MATLAB optimisation toolbox. The toolbox is used within the *Inverse Dynamics* and *Design Optimisation* modules.

Further information for each of the toolboxes is contained in the dependencies folder.

## Contributors ##
As an open source platform CASPR new developments into CASPR are welcome as are feedback/notification of bugs. CASPR is currently maintained by
* [Darwin Lau](darwinlau@mae.cuhk.edu.hk )
* [Jonathan Eden](jpeden@student.unimelb.edu.au)

## Further Documentation: ##
Now that you have installed CASPR please look at the *overview* and *getting_started* pdfs for further information.
