# Google Summer of Code Ideas 2107


NaveGo: an open-source MATLAB/GNU Octave toolbox for processing integrated navigation systems and performing inertial sensors profiling analysis.

NaveGo is an open-source framework for processing INS/GPS sensors that is freely available online. It is developed under MATLAB/GNU Octave due to this programming language has become a de facto standard for simulation and mathematical computing. NaveGo has been verified by processing real-world data from a real trajectory and contrasting results with a commercial, closed-source software package. Difference between both solutions have shown to be negligible. 

Actually, NaveGo is supported by three academic research groups: GridTics at the National University of Technology (Argentina), ITIC at the National University of Cuyo (Argentina), and DIATI at the Politecnico di Torino (Italy). 


# Proposal Guidelines

It is mandatory that students write and submit a proposal. We have added the applying to GSoC page to help guide our students on what we would like to see in those proposals. 

# NaveGo Ideas

* **Project:** OPTIMIZE NAVEGO TO RUN ON OCTAVE
* **Brief Explanation:** Even Though current NaveGo implementation can be executed on Octave, several functions should be re implemented following the optimal strategy for  improve Octave memory and time complexity. 
* ** Expected results:** The long term goal consists of  achieving a performance close to matlab However, since that could be unaffordable during the GSOC program, the central idea is to profile all the NaveGo code and concentrate in most consuming methods.  Such methods should be reimplemented using a more efficient vectorize version or reimplementing a C/C++ version if necessary.
* **Prequisites:** Octave and Matlab, Profiling techniques, C/C++,
* **Mentor:**  Rodrigo Gonzalez.

* **Project:** GRAPHICAL INTERFACE (GUI)
* **Brief Explanation:** Develop a simple GUI for simplifying the usual tasks required for executing a trajectory simulation on NaveGo .
* **Expected results:** A web based graphical interface for dealing with the data import process from different types of sensors, selecting the possible initialization parameters of the kalman filter and the generated trajectories outputs. Ideally, all the previous tasks should be reimplemented as a Rest web service for further reutilization.
* **Prequisites:** Node.js, Python, NoSQL databases,
* **Mentor:**  Carlos Catania.

