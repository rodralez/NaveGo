# NaveGo

[![Releases](https://img.shields.io/badge/release-v1.2-green.svg?style=plastic)](https://github.com/rodralez/NaveGo/releases) [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.2536950.svg)](https://doi.org/10.5281/zenodo.2536950)

NaveGo: an open-source MATLAB/GNU-Octave toolbox for processing integrated navigation systems and performing inertial sensors profiling analysis.

NaveGo (ˈnævəˈgəʊ) is an open-source MATLAB/GNU Octave toolbox for processing integrated navigation systems and simulating inertial sensors and a GNSS receiver. It also performs inertial sensors analysis using the Allan variance. It is freely available online. It is developed under MATLAB/GNU-Octave due to this programming language has become a *de facto* standard for simulation and mathematical computing.

NaveGo's motto is "to bring integrated navigation to the masses".

NaveGo is supported at the moment by three academic research groups: GridTics at the National University of Technology (Argentina), Engineering School at the National University of Cuyo (Argentina), and DIATI at the Politecnico di Torino (Italy).

## Features

Main features of NaveGo are:

* Processing of an inertial navigation system (INS).

* Processing of a loosely-coupled integrated navigation system (INS/GNSS).

* Simulation of inertial sensors and GNSS.

* Zero Velocity Update (ZUPT) detection algorithm.

* Allan variance technique to characterize inertial sensors' both deterministic and stochastic errors.


## NaveGo Mathematical Model

The underlying mathematical model of NaveGo is based on the two following articles:

* R. Gonzalez, J.I. Giribet, and H.D. Patiño. NaveGo: a simulation framework for low-cost integrated navigation systems, Journal of Control Engineering and Applied Informatics, vol. 17, issue 2, pp. 110-120, 2015. [Download](http://ceai.srait.ro/index.php?journal=ceai&page=article&op=view&path%5B%5D=2478).

* R. Gonzalez, J.I. Giribet, and H.D. Patiño. An approach to benchmarking of loosely coupled low-cost navigation systems. Mathematical and Computer Modelling of Dynamical Systems, vol. 21, issue 3, pp. 272-287, 2015. [Download](http://www.tandfonline.com/doi/abs/10.1080/13873954.2014.952642).

Be aware that the original Kalman filter state vector has been reduced from 21 to 15 states.


##  NaveGo Model Validation

NaveGo has been validated by processing real-world data from a real trajectory and contrasting results against Inertial Explorer, a commercial, closed-source software package. Differences between both solutions have shown to be negligible. For more information read the following paper:

* R. Gonzalez, C.A. Catania, P. Dabove, J.C. Taffernaberry, and M. Piras. Model validation of an open-source framework for post-processing INS/GNSS systems. III International Conference on Geographical Information Systems Theory, Applications and Management (GISTAM 2017). Porto, Portugal. April 2017. [Download](https://www.scitepress.org/PublicationsDetail.aspx?ID=Ds7NC2qnRSw%3d&t=1).


## Roadmap

Future features of NaveGo will be:

* Tightly-coupled INS/GNSS.

* RTS smoother.

* Adaptive Kalman filter.

* Unscented Kalman filter.

* KML file generator.


# Please, cite our work!

If you are using NaveGo in your research, we kindly ask you to add the following two cites to your future papers:

* R. Gonzalez, C.A. Catania, P. Dabove, J.C. Taffernaberry, and M. Piras. Model validation of an open-source framework for post-processing INS/GNSS systems. III International Conference on Geographical Information Systems Theory, Applications and Management (GISTAM 2017). Porto, Portugal. April 2017. [Download](https://www.scitepress.org/PublicationsDetail.aspx?ID=Ds7NC2qnRSw%3d&t=1).

* R. Gonzalez, J.I. Giribet, and H.D. Patiño. NaveGo: a simulation framework for low-cost integrated navigation systems, Journal of Control Engineering and Applied Informatics, vol. 17, issue 2, pp. 110-120, 2015. [Download](http://ceai.srait.ro/index.php?journal=ceai&page=article&op=view&path%5B%5D=2478).

An URL to NaveGo should be provided as the following cite:

R. Gonzalez, C. Catania, and P. Dabove. NaveGo: An Open-Source MATLAB/GNU-Octave Toolbox for Processing Integrated Navigation Systems and Performing Inertial Sensors Profiling Analysis. Version 1.2. URL: https://github.com/rodralez/NaveGo. DOI: 10.5281/zenodo.2536950. January 2019.


# Donations

Your donation helps us to improve NaveGo. You can make a donation to support our work using:

* PAYPAL: [paypal.me/supportnavego](https://www.paypal.com/paypalme/supportnavego).

* BITCOIN public key: bc1qdcf4c5t0t2a0uhqluae67yzuj5uerj8hgrxwwf .

* ETHEREUM (or DAI) public key: 0x92EC236bD4C5CAD90CB2F7B2BA327047e853A6cE .


# Contributions

We are looking for contributors to NaveGo! Since integrated navigation is a topic used in several fields such as Geomatics, Geology, Mobile Mapping, Autonomous Driving, and even Veterinary (yes, Veterinary!) for animal tracking, we hope other communities other than the navigation community compromise and contribute to this open-source project.

You can contribute in many ways:

* Writing code.
* Writing a manual.
* Reporting bugs.
* Suggesting new features.

If you are interested in joining NaveGo, please feel free to contact Dr. Rodrigo Gonzalez at rodralez [at] frm [dot] utn [dot] edu [dot] ar.


# Examples

The `example` folder contains several types of examples which try to be a kind of user manual for NaveGo.

## Allan variance example

This example can be analyzed by just executing the file `navego_example_allan.m`. Firstly, Allan variance is applied to 2-hours of real static measurements from a Sensonor STIM300 IMU. Then, almost 5 hours of synthetic inertial data are created and Allan variance is run on these simulated data.


## INS/GNSS integration example using synthetic (simulated) data

The NaveGo example with synthetic data is based on the output of a trajectory generator. This program provided both truth accelerations and angular velocities for a previous defined trajectory. NaveGo does not provide a trajectory generator.

The file `navego_example_synth.m` tries to expose how NaveGo can be used step by step. Two IMU measurements are simulated according to the error profiles of ADIS16405 IMU and ADIS16488 IMU. Then, both IMU are fused using a simulated GNSS sensor. Finally, performances of the two simulated INS/GNSS systems are compared.


### References

* R. Gonzalez, J.I. Giribet, and H.D. Patiño. NaveGo: a simulation framework for low-cost integrated navigation systems, Journal of Control Engineering and Applied Informatics, vol. 17, issue 2, pp. 110-120, 2015. [Download](http://ceai.srait.ro/index.php?journal=ceai&page=article&op=view&path%5B%5D=2478).

* Analog Devices. ADIS16400/ADIS16405 datasheet. High Precision Tri-Axis Gyroscope, Accelerometer, Magnetometer. Rev. B.
[Download](http://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16400_16405.pdf).

* Analog Devices. ADIS16488 datasheet. Tactical Grade Ten Degrees of Freedom Inertial Sensor. Rev. G.
[Download](http://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16488.pdf).

* Garmin International, Inc. GPS 18x TECHNICAL SPECIFICATIONS. Revision D. October 2011.
[Download](http://static.garmin.com/pumac/GPS_18x_Tech_Specs.pdf).


## INS/GNSS integration example using real data

Two examples of how to use NaveGo to post-process real data are provided as `navego_example_real_xxxx.m`, one for Ekinox-D IMU and another for MPU-6000 IMU. Both IMUs are integrated with Ekinox-D GNSS. These datasets were generated by driving a vehicle through the streets of Turin city (Italy).

These two real examples are part of the data collection used in the article (Gonzalez and Dabove, 2019).

### References

* R. Gonzalez and P. Dabove. Performance Assessment of an Ultra Low-Cost Inertial Measurement Unit for Ground Vehicle Navigation. Sensors 2019, 19(18), 3865; https://doi.org/10.3390/s19183865. September 2019. [Download](https://www.mdpi.com/1424-8220/19/18/3865/pdf).

* SBG Systems. SBG Ekinox-D High Accuracy Inertial System Brochure, Tactical grade MEMS Inertial Systems, v1.0. February 2014.

* InvenSense Inc. MPU-6000/MPU-6050 Product Specification. Document Number: PS-MPU-6000A-00. Revision: 3.4. Release Date: 08/19/2013.

# Researchers who are using NaveGo

The following 11 papers have expressed the use of NaveGo within their research:

1. Benjamin Noack, Christopher Funk, Susanne Radtke,and Uwe D. Hanebeck. State Estimation with Event-Based InputsUsing Stochastic Triggers. 1st Virtual IFAC World Congress (IFAC-V 2020). Germany, July 11-17, 2020 [Link](https://isas.iar.kit.edu/pdf/IFAC20_Noack.pdf).

2. W. Sun, J. Wu, W. Ding and S. Duan, "A Robust Indirect Kalman Filter Based on the Gradient Descent Algorithm for Attitude Estimation During Dynamic Conditions," in IEEE Access, vol. 8, pp. 96487-96494, 2020, doi: 10.1109/ACCESS.2020.2997250.

3. R. Rabiee, X. Zhong, Y. Yan and W.P. Tay, "LaIF: A Lane-Level Self-Positioning Scheme for Vehicles in GNSS-Denied Environments," in IEEE Transactions on Intelligent Transportation Systems, vol. 20, no. 8, pp. 2944-2961, August 2019. doi: 10.1109/TITS.2018.2870048. [Link](https://ieeexplore.ieee.org/document/8489926). NaveGo is used as a benchmark to compare to a proposed fusion algorithm based on a particle filter to achieve lane-level tracking accuracy under a GNSS-denied environment.

4. O. Tokluoğlu and E. Çavuş, "Study of Utilizing Multiple IMUs for Inertial Navigation Systems Without GPS Aid," 2019 1st Global Power, Energy and Communication Conference (GPECOM), Nevsehir, Turkey, June 2019, pp. 86-89. doi: 10.1109/GPECOM.2019.8778612.  [Link](https://ieeexplore.ieee.org/abstract/document/8778612). The purpose of NaveGo in this work is to test the performance of an INS/GNSS system with multiple IMUs.

5. Bac Nghia Vu, Khanh Nhu Nguyen and Mung Huy Vu, "Practical Considerations of IMU Data Generator," 2019 3rd International Conference on Recent Advances in Signal Processing, Telecommunications & Computing (SigTelCom), Hanoi, Vietnam, March 2019, pp. 63-68. doi: 10.1109/SIGTELCOM.2019.8696196. [Link](https://ieeexplore.ieee.org/abstract/document/8696196/). NaveGo is used to simulate gyros data. Then, these data is compared to the output of a proposed method for the same goal.

6. Mohamed Atia. "Design and simulation of sensor fusion using symbolic engines." Mathematical and Computer Modelling of Dynamical Systems 25.1 (2019): 40-62. February 2019. [Link](https://www.tandfonline.com/doi/abs/10.1080/13873954.2019.1566266). This work proposes a simulation framework for inertial sensors and an Attitude and Heading Reference System (AHRS). Atia uses as true data input to simulate sensors the same true data that NaveGo provides (see Fig. 7). Unfortunately, Atias' simulator and NaveGo performances are not compared in this work.

7. Ren, X., Sun, M., Jiang, C., Liu, L., & Huang, W. (2018). An Augmented Reality Geo-Registration Method for Ground Target Localization from a Low-Cost UAV Platform. Sensors, October 2018, vol. 18, no 11, p. 3739. [Link](https://www.mdpi.com/1424-8220/18/11/3739/htm). NaveGo is used to process RTK GPS data in the context of an INS/GNSS system for geo-registration and target localization.

8. M. Pachwicewicz and J. Weremczuk, "Accuracy Estimation of the Sounding Rocket Navigation System," 2018 XV International Scientific Conference on Optoelectronic and Electronic Sensors (COE), Warsaw, June 2018, pp. 1-4. doi: 10.1109/COE.2018.8435180. [Link](https://ieeexplore.ieee.org/abstract/document/8435180). In this paper NaveGo is used as a simulation framework for three types of IMUs.

9. M.G. Deepika and A. Arun, "Analysis of INS Parameters and Error Reduction by Integrating GPS and INS Signals," 2018 International Conference on Design Innovations for 3Cs Compute Communicate Control (ICDI3C), Bangalore, April 2018, pp. 18-23. doi: 10.1109/ICDI3C.2018.00013. [Link](https://ieeexplore.ieee.org/abstract/document/8436621). This work is completely based on the synthetic data example provided by NaveGo. It is not clear what is the contribution of this paper.

10. P.K. Diamantidis, ‘Attitude Navigation using a Sigma-Point Kalman Filter in an Error State Formulation’, Dissertation for Master degree. Department of Space and Plasma Physics, School of Electrical Engineering, KTH Royal Institute of Technology, Stockholm, Sweden. 2017. [Link](http://www.diva-portal.org/smash/get/diva2:1141205/FULLTEXT01.pdf). A 30-minutes static measurement of a gyroscope was made with and its Allan Variance plot is presented by using the NaveGo functions.

11. Shaoxing Hu, Shike Xu 1, Duhu Wang and Aiwu Zhang. Optimization Algorithm for Kalman Filter Exploiting the Numerical Characteristics of SINS/GPS Integrated Navigation Systems.  Sensors 2015, 15(11), 28402-28420. [Link](https://www.mdpi.com/117278). The mathematical model of this work is base on NaveGo's proposed mathematical model.


# Acknowledgments

We would like to thank to the many people that have contributed to make NaveGo a better tool:

* Dr. Juan Ignacio Giribet (National University of Buenos Aires, Argentina) for this continuous support on theory aspects of INS/GNSS systems.

* Dr. Charles K. Toth (The Ohio State University, USA), Dr. Allison Kealy, and M.Sc. Azmir Hasnur-Rabiain (both from The University of Melbourne, Australia) for generously sharing IMU and GNSS datasets, and in particular, for Azmir's unselfish support and help.

* Prof. Zhu, Dr. Yang, and Mr. Bo Sun, all from the Laboratory of Precision Measuring Technology and Instruments, Tianjin University, Tianjin, China, for contributing with IMU static measurements to test Allan variance routines.

* Dr. Paolo Dabove and Dr. Marco Piras (both from DIATI, Politecnico di Torino, Italy) for helping to debug NaveGo and suggesting new features.

* Dr. Paolo Zoccarato for his comments on attitude conversion.

	
