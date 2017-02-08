# Google Summer of Code Ideas 2107


NaveGo: an open-source MATLAB/GNU Octave toolbox for processing integrated navigation systems and performing inertial sensors profiling analysis.

NaveGo is an open-source framework for processing INS/GPS sensors that is freely available online. It is developed under MATLAB/GNU Octave due to this programming language has become a de facto standard for simulation and mathematical computing. NaveGo has been verified by processing real-world data from a real trajectory and contrasting results with a commercial, closed-source software package. Difference between both solutions have shown to be negligible. 

Actually, NaveGo is supported by three academic research groups: GridTics at the National University of Technology (Argentina), ITIC at the National University of Cuyo (Argentina), and DIATI at the Politecnico di Torino (Italy). 


# Proposal Guidelines

It is mandatory that students write and submit a proposal. We have added the applying to GSoC page to help guide our students on what we would like to see in those proposals. 

# NaveGo Ideas

Project: Biological Data Visualization
Brief explanation: Support for biological data, representations, and visualization
Expected results: Add support for molecular fragments on top of the molecule model, extending this to residues, and supporting reading/writing this secondary structure (e.g., PDB format). Additional rendering modes for secondary biological structures (i.e. ribbons, cartoons, etc.), building up a biomolecule from residues, and adding residue labels. Code and algorithms may be adapted from 3DMol.js. Since biological molecules are often large (10^3 to 10^6 atoms and bonds), such implementations should be highly efficient and optimized, adopting symmetry and other techniques to improve interactivity and rendering performance.
Prerequisites: Experience in C++, some experience with OpenGL and an biochemistry ideally, but not necessary.
Mentor: Marcus D. Hanwell (marcus dot hanwell at kitware dot com). 

