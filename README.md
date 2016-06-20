# CASPR #
The *Cable-robot Analysis and Simulation Platform for Research (CASPR)*  is an open-source software platform developed in MATLAB that facilitates a range of analysis approaches on arbitrary cable-robot models.
## Purpose: ##
CASPR is designed for researchers to perform study on Cable-Driven Parallel Robots (*CDPRs*). CASPR aims to provide researchers with:

1. The ability to use the extensive libraries and models of CASPR from the community for your research.
2. The ability to easily and efficiently add, test and validate your algorithms and models.
3. The ability to share your research to the community.

The CASPR software platform represents a unified development platform for the analysis of cable-driven parallel robots.  The current version of the platform contains analysis tools for each of the following fields of study:
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
CASPR can be operated any version of Windows and Linux that runs MATLAB (refer below). It should be noted that some optional features cannot be used when run on Linux.

### MATLAB: ###
As the software is run through MATLAB, it is necessary that MATLAB is first installed before running CASPR. CASPR has been tested for MATLAB versions 2013a onwards. The core of CASPR will work for older versions, however certain newer functions may need to be replaced with the equivalent depreciated function call.

## Setup: ##
1. There are two ways to download CASPR to your computer:

  * [READ-ONLY ACCESS] - git clone https://github.com/darwinlau/CASPR.git
  * [READ-CONTRIBUTE ACCESS] - Fork the CASPR repository onto your github account and then clone the new repository.
    git clone https://github.com/YOUR_USER_NAME/CASPR.git
   
 **Note:** For additional information regarding forking on github please refer to <https://help.github.com/articles/fork-a-repo/>

2. (Optionally) install the CASPR depedencies. See [Dependency Installation](#dependency_install) for further details
3. Open MATLAB and change the file path to be that of the extracted CASPR root directory.
4. Run the script setup_CASPR.m within the *scripts/setup/* folder. This will test the installation and confirm that CASPR is ready to be run.

**Additional Note:** All subsequent sessions running CASPR should be started by running the initialise_CASPR.m function from the CASPR root directory.

## Contributors ##
As an open source platform new developments into CASPR are welcome as are feedback/notification of bugs. Contributors for the development of this project include
* Darwin Lau:     <darwinlau@mae.cuhk.edu.hk>
* Jonathan Eden:  <jpeden@student.unimelb.edu.au>

For notification of issues please use the issues tab within the github page <https://github.com/darwinlau/CASPR/issues>.  For more detailed communication please email Darwin Lau at <darwinlau@mae.cuhk.edu.hk>.

## Further Documentation: ##
Now that you have installed CASPR, please look at *CASPR_101.pdf* (located in the *docs* directory) for further information regarding the operation of CASPR.

## <a name="dependency_install"></a> Dependency Installation ##
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
2. Extract the downloaded folder.
3. Open MATLAB into the directory of the extracted folder.
4. Run opti_Install.m to finalise the installation.
