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

## Installing CASPR: ##
### Environment Requirements: ###
CASPR can be operated any version of Windows and Linux with MATLAB installed (refer below). It should be noted that some optional features cannot be used when run on Linux.

### MATLAB Requirements: ###
CASPR has been tested for MATLAB versions 2013a onwards. The core of CASPR will work for older versions, however certain newer functions may need to be replaced with the equivalent depreciated function call. **Note**: Unit testing requires MATLAB 2014a or later.

### Setup Procedure: ###
**Since CASPR is an actively developed and will be frequently updated, it is strongly advised NOT to simply download the zip. Please CLONE or FORK as recommended below so that you can easily obtain updates to CASPR.**

1. There are two ways to download CASPR to your computer:
  * [READ-ONLY ACCESS] - git clone https://github.com/darwinlau/CASPR.git  
  * [READ-CONTRIBUTE ACCESS] - Fork the CASPR repository onto your github account and then clone the new repository.
    git clone https://github.com/YOUR_USER_NAME/CASPR.git
 
 **Note:** For additional information regarding forking on github please refer to <https://help.github.com/articles/fork-a-repo/>

2. (Optionally) install the CASPR depedencies. See [Dependency Installation](#dependency_install) for further details
3. Open MATLAB and change the file path to be that of the extracted CASPR root directory.
4. Run the script initialise_CASPR.m from the root directory. This will test the installation and confirm that CASPR is ready to be run.

## Citation: ##
If you use CASPR in your research please cite the 2016 IROS paper:

    @inproceedings{lau2016CASPR,
     title={{CASPR}: A Comprehensive Cable-Robot Analysis and Simulation Platform for the Research of Cable-Driven Parallel Robots},
     author={Lau, Darwin and Eden, Jonathan and Tan, Ying and Oetomo, Denny},
     booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
     year={2016},
     organization={IEEE}
    }


## Contributors: ##
As an open source platform new developments into CASPR are welcome as are feedback/notification of bugs. Contributors for the development of this project include
* Darwin Lau:     <darwinlau@mae.cuhk.edu.hk>
* Jonathan Eden:  <jpeden@student.unimelb.edu.au>

For notification of issues please use the issues tab within the github page <https://github.com/darwinlau/CASPR/issues>.  For more detailed communication please email Darwin Lau at <darwinlau@mae.cuhk.edu.hk>.

## Getting Started: ##
To start using CASPR, please follow the steps below

1. Navigate to the CASPR root directory folder.
2. Run the script initialise_CASPR.m.  This will ensure that your path libraries have been set up and **should be run everytime that you use CASPR**.
3. Go into the script folders to look at examples or open up the CASPR GUI using the CASPR_GUI command.

For further information regarding the operation of CASPR please refer to *CASPR_101.pdf* (located in the *docs* directory). Aditional documentation including the 2016 IROS paper can also be found within this directory.

## Keeping CASPR Up to Date: ##
To keep CASPR up to date please periodically execute the command *git pull* from your root directory.

## <a name="dependency_install"></a> Dependency Installation (Optional) ##
In addition to the core code base, some analysis techniques within CASPR require the use of additional software dependencies. These dependencies include
* 'qhull' - This is a convex hull library that is used throughout the *Workspace* analysis module.
* 'optitoolbox' - This is a MATLAB optimisation toolbox. The toolbox is used within the *Inverse Dynamics* and *Design Optimisation* modules.

Further information for setting up each of these toolboxes is contained below
### qhull Installation ###
1. Go to the qhull website <http://www.qhull.org/download/> and download the latest version of qhull that is appropriate for your operating system.
2. Follow the installation instructions provided for your operating system in the file *README.txt* which will be in the downloaded qhull folder.
3. To allow for CASPR to access the compiled qhull code, move the compiled version of the code to the subdirectory *dependencies* that is located in your *CASPR* root directory.

### optitoolbox Installation ###
1. Go to the optitoolbox website <http://www.i2c2.aut.ac.nz/Wiki/OPTI/index.php/DL/DownloadOPTI> and download the latest version of the toolbox.
2. Extract the downloaded folder.
3. Open MATLAB into the directory of the extracted folder.
4. Run opti_Install.m to finalise the installation.
