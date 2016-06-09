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
CASPR has been developed in Windows and ideally should be run on 32-bit and 64-bit computers running that operating system. The toolbox has also been tested to work on the Linux Ubuntu distributions from 14.04 onwards, however some of the dependencies cannot be used.

### MATLAB: ###
As the software is run through MATLAB, it is necessary that MATLAB is first installed before running CASPR. CASPR has been tested for MATLAB versions 2014a onwards. The core of CASPR will work for older versions, however certain newer functions may need to be replaced with the equivalent depreciated function call.

## Installation: ##
1. Download the latest version of CASPR from the repository.
  * To allow for updates and to make changes to the repository please do this through the operation
    git clone https://github.com/darwinlau/CASPR_private.git
2. Extract the downloaded CASPR folder and place the extracted folder into the directory of your choosing.
3. Open MATLAB and change the file path to be that of the extracted CASPR root directory.
4. Run the script setup_CASPR.m. This will test the installation and confirm that CASPR is ready to be run.

## Contributors ##
As an open source platform CASPR new developments into CASPR are welcome as are feedback/notification of bugs. CASPR is currently maintained by
* Darwin Lau:     <darwinlau@mae.cuhk.edu.hk>
* Jonathan Eden:  <jpeden@student.unimelb.edu.au>

## Further Documentation: ##
Now that you have installed CASPR please look at the *CASPR_101* pdf for further information regarding .

## Dependency Installation ##
In addition to the core code base, some analysis techniques within CASPR require the use of additional software dependencies. These dependencies include
* 'qhull' - This is a convex hull library that is used throughout the *Workspace* analysis module.
* 'optitoolbox' - This is a MATLAB optimisation toolbox. The toolbox is used within the *Inverse Dynamics* and *Design Optimisation* modules.

Further information for setting up each of these toolboxes is contained below
### qhull Installation ###
1. Go to the qhull website <http://www.qhull.org/download/> and download the latest version of qhull that is appropriate for your operating system.
2. Follow the installation instructions provided for your operating system in the file *README.txt*
3. To allow for CASPR to access the compiled qhull code, move the compiled versionof the code to the subdirectory *dependencies* that is located in your *CASPR* root directory.
### optitoolbox Installation ###
1. Go to the optitoolbox website <http://www.i2c2.aut.ac.nz/Wiki/OPTI/index.php/DL/DownloadOPTI> and download the latest version of the toolbox.
2. Extract the downloaded folder (preferably into the *dependencies* subdirectory).
3. Open MATLAB into the directory of the extracted folder.
4. Run opti_Install to finalise the installation.
