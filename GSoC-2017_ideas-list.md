# Google Summer of Code Ideas 2107

## Introduction

NaveGo: an open-source MATLAB/GNU Octave toolbox for processing integrated navigation systems and performing inertial sensors profiling analysis.

NaveGo is an open-source framework for processing integrated navigation systems and performing inertial sensors profiling analysis that is freely available online. It is developed under MATLAB/GNU Octave due to this programming language has become a *de facto* standard for simulation and mathematical computing. NaveGo has been verified by processing real-world data from a real trajectory and contrasting results with a commercial, closed-source software package. Difference between both solutions have shown to be negligible. 

Main features of NaveGo are:

* Processing of an inertial navigation system (INS).
* Processing of a loosely-coupled integrated navigation system (INS/GPS).
* Implementation of the Allan variance procedure to characterize inertial sensors' typical errors.
* Simulation of inertial sensors and GPS (in a very early stage).

NaveGo is supported at the moment by three academic research groups: GridTics at the National University of Technology (Argentina), ITIC at the National University of Cuyo (Argentina), and DIATI at the Politecnico di Torino (Italy). 


## Proposal Guidelines

It is mandatory that students write and submit a proposal. We have added the [applying to GSoC page] to help guide our students on what we would like to see in those proposals. We welcome original ideas in addition to what is listed here. You can suggest something that you consider interesting for NaveGo.

## NaveGo Project Ideas

### Project 1: DEVELOPMENT OF A GRAPHICAL INTERFACE (GUI).

* **Brief Explanation:** Development of a drag-and-drop GUI for simplifying the usual tasks required for integrated navigation on NaveGo .

* **Expected results:** A web based graphical interface for easily representing the workflow usually done in integrated navigation systems. These tasks should graphically be represented as blocks with different levels of connections, in a similar way as [Rapidminer] does. This GUI must deal with 1) data import processing from different types of sensors, 2) selecting initialization parameters (initial position, velocity and attitude; Kalman filter intial parameters; etc.), 3) generating outputs files and plotting trajectories, 4) profiling inertial sensors, and 5) statistical validation of data. All previous tasks already exist in NaveGo but should be refactored following a full web services architecture for facilitating future reusability. 

* **Prequisites:** Node.js, Python, NoSQL databases.

* **Mentor:**  Carlos Catania (ITIC, National University of Cuyo).

--------

### Project 2: DEVELOPMENT OF A LIBRARY FOR IMPORT/EXPORT IMPROVEMENT.

* **Brief Explanation:** Capability to import files in GNSS standard formats and to export files for most popular GIS programs.

* **Expected results:** One GNU Octave function must be developed for each import/export format. Among GNSS standard formats to import are RINEX, RTCM, UBLOX, NMEA. NaveGo results should be exported to most popular GIS programs in SHP format (Shapefile) and in NMEA and KML formats. A few more import/export formats may arise during project discussion with the student. A User Guide for this library should be written.

* **Prequisites:** GNU Octave/MATLAB, GNSS, GPS, GIS.

* **Mentor:** Paolo Dabove (DIATI, Polictecnico di Torino).

--------

### Project 3: NAVEGO EXECUTION OPTIMIZATION FOR GNU OCTAVE.

* **Brief Explanation:** Even though NaveGo actually can be executed on Octave, several functions should be reimplemented following the optimal strategy for  improve Octave memory and time complexity. 

* **Expected results:** The long term goal consists of achieving a performance close to MATLAB. However, since this could be unaffordable during the GSoC program, the central idea is to profile all NaveGo code and to concentrate in most consuming functions. Such functions should be reimplemented using a more efficient vectorized version or a C/C++ version, if necessary.

* **Prequisites:** GNU Octave/MATLAB, profiling techniques, C/C++.

* **Mentor:**  Marco Piras (DIATI, Polictecnico di Torino).
--------

### Project 4: DEVELOPMENT OF A LIBRARY FOR INS ALIGNMENT AND ZERO VELOCITY UPDATES.

* **Brief Explanation:** Implementation of the alignment methods proposed in (Groves, 2008), chapters 5 and 13.

* **Expected results:** Alignment methods for improving the calibration of an inertial navigation system (INS) attitude solution and inertial sensor errors between the initialization of the INS and the use of its navigation solution. One GNU Octave function must be developed for each alignment method. The library should be tested with real data that will be previously provided. A User Guide for this library should be written.

* **Prequisites:** GNU Octave/MATLAB, NaveGo, basic inertial navigation concepts.

* **Mentor:**  Rodrigo Gonzalez (GridTics, National University of Technology).


(Groves, 2008) Paul D. Groves. Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems. ISBN-13: 978-1-58053-255-6. Artech House. London, 2008.

[applying to GSoC page]:https://github.com/rodralez/NaveGo/blob/master/GSoC-2017_how-to-apply.md "Applying to GSoC"

[Rapidminer]:https://rapidminer.com/ "Rapidminer"
