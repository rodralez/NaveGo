# Google Summer of Code Ideas 2107


# Guidelines

NaveGo: an open-source MATLAB/GNU Octave toolbox for processing integrated navigation systems and performing inertial sensors profiling analysis.

NaveGo is an open-source framework for processing INS/GPS sensors that is freely available online. It is developed under MATLAB/GNU Octave due to this programming language has become a *de facto* standard for simulation and mathematical computing. NaveGo has been verified by processing real-world data from a real trajectory and contrasting results with a commercial, closed-source software package. Difference between both solutions have shown to be negligible. 

Actually, NaveGo is supported by three academic research groups: GridTics at the National University of Technology (Argentina), ITIC at the National University of Cuyo (Argentina), and DIATI at the Politecnico di Torino (Italy). 


# Proposal Guidelines

It is mandatory that students write and submit a proposal. We have added the [applying to GSoC page] to help guide our students on what we would like to see in those proposals. We welcome original ideas in addition to what is listed here. You can suggest something that you consider interesting for NaveGo.

# NaveGo Project Ideas

## **Project:** GRAPHICAL INTERFACE (GUI)

* **Brief Explanation:** Develop a GUI for simplifying the usual tasks required for integrated navigation on NaveGo .

* **Expected results:** A web based graphical interface for easily representing the workflow usually used in integrated navigation systems. These tasks should graphically be represented as blocks with different levels of connections, in a similar way as [Rapidminer] does. This GUI must deal with 1) data import processing from different types of sensors, 2) selecting initialization parameters (initial PVA, Kalman filter, etc.), 3) generating outputs files and plotting trajectories, 4) profiling sensor, and 5) statistical validation of data. All previous tasks already exist in NaveGo but should be refactored following a full web services architecture for facilitating future reusability. 

* **Prequisites:** Node.js, Python, NoSQL databases

* **Mentor:**  Carlos Catania.

--------

## **Project:** OPTIMIZE NAVEGO TO RUN ON OCTAVE

* **Brief Explanation:** Even though NaveGo actually can be executed on Octave, several functions should be reimplemented following the optimal strategy for  improve Octave memory and time complexity. 

* **Expected results:** The long term goal consists of achieving a performance close to MATLAB. However, since this could be unaffordable during the GSOC program, the central idea is to profile all the NaveGo code and concentrate in most consuming methods. Such methods should be reimplemented using a more efficient vectorize version or reimplementing a C/C++ version if necessary.

* **Prequisites:** Octave and MATLAB, Profiling techniques, C/C++,

* **Mentor:**  Rodrigo Gonzalez.

--------

[applying to GSoC page]:https://github.com/rodralez/NaveGo/blob/master/GSoC-2017_how-to-apply.md "Applying to GSoC"

[Rapidminer]:https://rapidminer.com/ "Rapidminer"
