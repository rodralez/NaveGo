# NaveGo

[![Releases](https://img.shields.io/badge/release-v1.4-green.svg?style=plastic)](https://github.com/rodralez/NaveGo/releases) 
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.6549626.svg)](https://doi.org/10.5281/zenodo.6549626)

NaveGo: an open-source MATLAB/GNU-Octave toolbox for processing integrated navigation systems and performing inertial sensors profiling analysis.

NaveGo (ˈnævəˈgəʊ) is an open-source MATLAB/GNU Octave toolbox for processing integrated navigation systems and simulating inertial sensors and a GNSS receiver. It also performs analysis of an inertial sensor using the Allan variance. It is freely available online. It is developed under MATLAB/GNU-Octave due to this programming language has become a *de facto* standard for engineering and modeling of physical systems.

NaveGo's motto is "to bring integrated navigation to the masses".

## Important Update for the NaveGo Community

Dear NaveGo Community,

I am reaching out to share an important update regarding the NaveGo project. Due to a shift in both my professional career and personal interests away from navigation systems, I have made the difficult decision to step down from my role as the lead developer of NaveGo.

Effective immediately, NaveGo will transition to a community-driven project. This change opens up new opportunities for collaboration, innovation, and leadership within our vibrant community. I encourage each of you to actively participate, whether by answering questions in the discussion forum, contributing to the development of new features, or sharing your unique insights and expertise.

This project has always thrived on the enthusiasm, creativity, and dedication of its community members, and I have every confidence that NaveGo will continue to grow and evolve in exciting ways. If you or someone you know is passionate about taking on a leadership role within the NaveGo project, please do not hesitate to contact me. Your involvement could make a significant impact on the direction and success of NaveGo.

I want to express my deepest gratitude to all of you for your support and contributions to NaveGo. It has been an honor to work alongside such a talented and dedicated group of individuals. I look forward to witnessing the future achievements of this project and the community that has been its backbone.

Thank you for your understanding and continued support.

Warm regards,

Dr. Rodrigo Gonzalez

## Attention!

Please, you should open a new [issue](https://github.com/rodralez/NaveGo/issues) only if:

1. You found a bug or error in the source code of NaveGo,
2. You want to ask for new features. Maybe some contributor will gently take this issue and code your suggested feature.

But, if you have any question of any kind or you want to share some feedback about NaveGo, please leave a comment at the [discussion forum](https://github.com/rodralez/NaveGo/discussions).

## Features

Main features of NaveGo are:

* Processing of an inertial navigation system (INS).

* Processing of a loosely-coupled integrated navigation system (INS/GNSS).

* Processing of a loosely-coupled integrated navigation system with magnetometers (INS/GNSS/MAG).

* ***NEW*** Processing of a loosely-coupled integrated visual navigation system (VISUAL/INS/GNSS). 

* Compass heading using magnetometers.

* Simulation of inertial sensors and GNSS.

* Zero Velocity Update (ZUPT) detection algorithm.

* Allan variance technique to characterize inertial sensors' both deterministic and stochastic errors.

* Better visualization of GNSS outages.

## NaveGo Mathematical Model

The underlying mathematical model of NaveGo has been evolving. It is based on several books, mostly on:

* Paul D. Groves (2013). Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (2nd Ed.). Artech House, USA.

* D.H. Titterton and J.L. Weston (2004). Strapdown Inertial Navigation Technology (2nd Ed.). Institution of Engineering and Technology, USA.

Two previous articles used to expose NaveGo mathematical model, but currently these two documents are partially outdated:

* R. Gonzalez, J.I. Giribet, and H.D. Patiño. NaveGo: a simulation framework for low-cost integrated navigation systems, Journal of Control Engineering and Applied Informatics, vol. 17, issue 2, pp. 110-120, 2015. [Download](http://ceai.srait.ro/index.php?journal=ceai&page=article&op=view&path%5B%5D=2478).

* R. Gonzalez, J.I. Giribet, and H.D. Patiño. An approach to benchmarking of loosely coupled low-cost navigation systems. Mathematical and Computer Modelling of Dynamical Systems, vol. 21, issue 3, pp. 272-287, 2015. [Download](http://www.tandfonline.com/doi/abs/10.1080/13873954.2014.952642).

For example, the original Kalman filter state vector has been reduced from 21 to 15 states. Therefore, please take these two articles just to have a glimpse of NaveGo structure.

##  NaveGo Model Validation

NaveGo has been validated by processing real-world data from a real trajectory and contrasting results against Inertial Explorer, a commercial, closed-source software package. Differences between both solutions have shown to be negligible. For more information read the following paper:

* R. Gonzalez, C.A. Catania, P. Dabove, J.C. Taffernaberry, and M. Piras. Model validation of an open-source framework for post-processing INS/GNSS systems. III International Conference on Geographical Information Systems Theory, Applications and Management (GISTAM 2017). Porto, Portugal. April 2017. [Download](https://www.scitepress.org/PublicationsDetail.aspx?ID=Ds7NC2qnRSw%3d&t=1).

## Roadmap

Hopefully, these three future features will be added to NaveGo:
   
* Barometer as aiding sensor for altitude.

* G-sensitivity correction for gyroscopes.

* Tightly-coupled INS/GNSS integration.

# Contributions

We are looking for contributors to NaveGo! Since integrated navigation is a topic used in several fields such as Geomatics, Geology, Mobile Mapping, Autonomous Driving, and even Veterinary (yes, Veterinary!) for animal tracking, we hope other communities other than the navigation community compromise and contribute to this open-source project.

You can contribute in many ways:

* Writing code.
* Writing a manual.
* Reporting bugs.

If you want to contribute to the NaveGo project, you should follow the Github Workflow methodology summarized at `./doc/github-workflow.md`.

If you are interested in joining NaveGo, please feel free to contact Dr. Rodrigo Gonzalez at rodralez [at] frm [dot] utn [dot] edu [dot] ar.

## Branches

NaveGo has two main branches:

1. `master`: contains the stable releases of NaveGo.

2. `develop`: every new feature of NaveGo will be implemented in this branch first. `develop` will be merged to `master` from time to time.

More branches would be created to develop particular features for NaveGo. These extra branches will be eventually merged to `develop`. 


# Please, cite our work!

If you are using NaveGo in your research, we kindly ask you to add the following two cites to your future papers:

* R. Gonzalez, C.A. Catania, P. Dabove, J.C. Taffernaberry, and M. Piras. Model validation of an open-source framework for post-processing INS/GNSS systems. III International Conference on Geographical Information Systems Theory, Applications and Management (GISTAM 2017). Porto, Portugal. April 2017. [Download](https://www.scitepress.org/PublicationsDetail.aspx?ID=Ds7NC2qnRSw%3d&t=1).

* R. Gonzalez, J.I. Giribet, and H.D. Patiño. NaveGo: a simulation framework for low-cost integrated navigation systems, Journal of Control Engineering and Applied Informatics, vol. 17, issue 2, pp. 110-120, 2015. [Download](http://ceai.srait.ro/index.php?journal=ceai&page=article&op=view&path%5B%5D=2478).

An URL to NaveGo should be provided as the following cite:

Gonzalez, Rodrigo (2022). NaveGo: an open-source MATLAB/GNU-Octave toolbox for processing integrated navigation systems and performing inertial sensors profiling analysis. NaveGo Release v1.4 (v1.4). Zenodo. https://doi.org/10.5281/zenodo.6549626

# Donations

Your donation helps us to improve NaveGo. You can make a donation to support our work using:

* BITCOIN public key:  bc1q8uehhf0y36jtwyua29z0xhqxvd7q2thuuwys28 (BTC network).
* Ethereum public key: 0x97aae6533AaeD1ba38D1863B4a8C35e7Cc5261E8 (ERC20 network).
* USDT public key:     0x97aae6533AaeD1ba38D1863B4a8C35e7Cc5261E8 (ERC20 network).
* PAYPAL: [paypal.me/supportnavego](https://www.paypal.com/paypalme/supportnavego).


# Examples

The `example` folder contains several types of examples which try to be a kind introducción to the use of NaveGo.

## Allan variance example

This example can be analyzed by just executing the file `navego_example_allan.m`. Firstly, Allan variance is applied to 2-hours of real static measurements from a Sensonor STIM300 IMU. Then, almost 5 hours of synthetic inertial data are created and Allan variance is run on these simulated data.


## INS/GNSS integration example using synthetic (simulated) data

The NaveGo example with synthetic data is based on the output of a trajectory generator. This program provided both truth accelerations and angular velocities for a previous defined trajectory. NaveGo does not provide a trajectory generator.

The file `navego_example_synth.m` tries to expose how NaveGo can be used step by step. Two IMU measurements are simulated according to the error profiles of the ADIS16405 IMU and the ADIS16488 IMU. Then, both IMU are fused using a simulated GNSS sensor. Finally, performances of the two simulated INS/GNSS systems are compared.


### References

* R. Gonzalez, J.I. Giribet, and H.D. Patiño. NaveGo: a simulation framework for low-cost integrated navigation systems, Journal of Control Engineering and Applied Informatics, vol. 17, issue 2, pp. 110-120, 2015. [Download](http://ceai.srait.ro/index.php?journal=ceai&page=article&op=view&path%5B%5D=2478).

* Analog Devices. ADIS16400/ADIS16405 datasheet. High Precision Tri-Axis Gyroscope, Accelerometer, Magnetometer. Rev. B.
[Download](http://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16400_16405.pdf).

* Analog Devices. ADIS16488 datasheet. Tactical Grade Ten Degrees of Freedom Inertial Sensor. Rev. G.
[Download](http://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16488.pdf).

* Garmin International, Inc. GPS 18x TECHNICAL SPECIFICATIONS. Revision D. October 2011.
[Download](http://static.garmin.com/pumac/GPS_18x_Tech_Specs.pdf).


## INS/GNSS integration example using real data

Two examples of how to use NaveGo to post-process real data are provided as `navego_example_real_xxxx.m`, one for tactical-grade Ekinox-D IMU and another for consumer-grade MPU-6000 IMU. Both IMUs are integrated with Ekinox-D GNSS. These datasets were generated by driving a vehicle through the streets of Turin city (Italy).

These two real examples are part of the data collection used in the article (Gonzalez and Dabove, 2019).

### References

* R. Gonzalez and P. Dabove. Performance Assessment of an Ultra Low-Cost Inertial Measurement Unit for Ground Vehicle Navigation. Sensors 2019, 19(18), 3865; https://doi.org/10.3390/s19183865. September 2019. [Download](https://www.mdpi.com/1424-8220/19/18/3865/pdf).

* SBG Systems. SBG Ekinox-D High Accuracy Inertial System Brochure, Tactical grade MEMS Inertial Systems, v1.0. February 2014.

* InvenSense Inc. MPU-6000/MPU-6050 Product Specification. Document Number: PS-MPU-6000A-00. Revision: 3.4. Release Date: 08/19/2013.

## VISUAL/INS/GNSS integration example using real data

Two examples are provided for VISUAL/INS/GNSS integration:

1. `navego_example_canada_data.m`.
2. `navego_example_katwijk_data.m`.

Please, check the folder `examples/visual-data/`.

### References
 
*  Johann Diep et al. (2022). Investigating the Performance of LCNS with Visual-Inertial Odometry for Lunar Rover Navigation. NAVITEC 2022, April 2022.

* Johann Diep (2022). Investigating the Performance of LCNS with Visual-Inertial Odometry for Lunar Rover Navigation. [Youtube video](https://youtu.be/sPnPMBs1gSU).

# Researchers who are using NaveGo

The following 16 papers have expressed the use of NaveGo for research:

1. Paul Rawiel (2022). Positioning of Pedelecs for a Pedelec Sharing System with Free-Floating Bikes. In: Coors, V., Pietruschka, D., Zeitler, B. (eds) iCity. Transformative Research for the Livable, Intelligent, and Sustainable City. Springer, Cham. [link](https://doi.org/10.1007/978-3-030-92096-8_5)

2. Ren, X.; Sun, M.; Zhang, X.; Liu, L.; Wang, X.; Zhou, H. An AR Geo-Registration Algorithm for UAV TIR Video Streams Based on Dual-Antenna RTK-GPS. Remote Sens. 2022, 14, 2205. https://doi.org/10.3390/rs14092205. This paper use NaveGo to improve the geographic registration (geo-registration) accuracy by fusing the positioning and heading data from the dual-antenna real-time kinematic global positioning system (RTK-GPS) with onboard internal measurement unit (IMU) data.

3. Johann Diep et al. Investigating the Performance of LCNS with Visual-Inertial Odometry for Lunar Rover Navigation. NAVITEC 2022 conference, April 2022. NaveGo is used as part of a VISUAL/INS/GNSS navigation system. [Youtube video](https://youtu.be/sPnPMBs1gSU).

4. Canhui Tao, Zhiping Song, and Zhenping Weng. MCTLS-Assisted Completed SINS/GPS Integrated and Applied to Low-Cost Attitude and Heading Reference System. Mathematical Problems in Engineering 2021 (2021). [Link](https://www.hindawi.com/journals/mpe/2021/4260162/). NaveGo is used as a benchmark for comparing a proposed heading determination approach.

5. Nagui, N., Attallah, O., Zaghloul, M.S. et al. Improved GPS/IMU Loosely Coupled Integration Scheme Using Two Kalman Filter-based Cascaded Stages. Arab J Sci Eng 46, 1345–1367 (2021). [Link](https://doi.org/10.1007/s13369-020-05144-8). Authors use NaveGo as a benchmark for a new proposed integrated navigation scheme.

6. Benjamin Noack, Christopher Funk, Susanne Radtke,and Uwe D. Hanebeck. State Estimation with Event-Based Inputs Using Stochastic Triggers. First Virtual IFAC World Congress (IFAC-V 2020). Germany, July 11-17, 2020 [Link](https://isas.iar.kit.edu/pdf/IFAC20_Noack.pdf).

7. W. Sun, J. Wu, W. Ding and S. Duan. A Robust Indirect Kalman Filter Based on the Gradient Descent Algorithm for Attitude Estimation During Dynamic Conditions, in IEEE Access, vol. 8, pp. 96487-96494, 2020, doi: 10.1109/ACCESS.2020.2997250.

8. R. Rabiee, X. Zhong, Y. Yan and W.P. Tay. LaIF: A Lane-Level Self-Positioning Scheme for Vehicles in GNSS-Denied Environments, in IEEE Transactions on Intelligent Transportation Systems, vol. 20, no. 8, pp. 2944-2961, August 2019. doi: 10.1109/TITS.2018.2870048. [Link](https://ieeexplore.ieee.org/document/8489926). NaveGo is used as a benchmark to assess a proposed fusion algorithm based on a particle filter to achieve lane-level tracking accuracy under a GNSS-denied environment.

9. O. Tokluoğlu and E. Çavuş. Study of Utilizing Multiple IMUs for Inertial Navigation Systems Without GPS Aid, 2019 1st Global Power, Energy and Communication Conference (GPECOM), Nevsehir, Turkey, June 2019, pp. 86-89. doi: 10.1109/GPECOM.2019.8778612.  [Link](https://ieeexplore.ieee.org/abstract/document/8778612). The purpose of NaveGo in this work is to test the performance of an INS/GNSS system with multiple IMUs.

10. Bac Nghia Vu, Khanh Nhu Nguyen and Mung Huy Vu. Practical Considerations of IMU Data Generator, 2019 3rd International Conference on Recent Advances in Signal Processing, Telecommunications & Computing (SigTelCom), Hanoi, Vietnam, March 2019, pp. 63-68. doi: 10.1109/SIGTELCOM.2019.8696196. [Link](https://ieeexplore.ieee.org/abstract/document/8696196/). NaveGo is used to simulate gyros data. Then, these data is compared to the output of a proposed method for the same goal.

11. Mohamed Atia. Design and simulation of sensor fusion using symbolic engines. Mathematical and Computer Modelling of Dynamical Systems 25.1 (2019): 40-62. February 2019. [Link](https://www.tandfonline.com/doi/abs/10.1080/13873954.2019.1566266). This work proposes a simulation framework for inertial sensors and an Attitude and Heading Reference System (AHRS). Atia uses the same true data that NaveGo provides (see Fig. 7) as true data input to simulate sensors. Unfortunately, Atias' simulator and NaveGo performances are not compared in this work.

12. Ren, X., Sun, M., Jiang, C., Liu, L., & Huang, W. (2018). An Augmented Reality Geo-Registration Method for Ground Target Localization from a Low-Cost UAV Platform. Sensors, October 2018, vol. 18, no 11, p. 3739. [Link](https://www.mdpi.com/1424-8220/18/11/3739/htm). NaveGo is used to process RTK GPS data in the context of an INS/GNSS system for geo-registration and target localization.

13. M. Pachwicewicz and J. Weremczuk. Accuracy Estimation of the Sounding Rocket Navigation System. 2018 XV International Scientific Conference on Optoelectronic and Electronic Sensors (COE), Warsaw, June 2018, pp. 1-4. doi: 10.1109/COE.2018.8435180. [Link](https://ieeexplore.ieee.org/abstract/document/8435180). In this paper NaveGo is used as a simulation framework for three types of IMUs.

14. M.G. Deepika and A. Arun. Analysis of INS Parameters and Error Reduction by Integrating GPS and INS Signals. 2018 International Conference on Design Innovations for 3Cs Compute Communicate Control (ICDI3C), Bangalore, April 2018, pp. 18-23. doi: 10.1109/ICDI3C.2018.00013. [Link](https://ieeexplore.ieee.org/abstract/document/8436621). This work is completely based on the synthetic data example provided by NaveGo. It is not clear what the contribution of this paper is.

15. P.K. Diamantidis. Attitude Navigation using a Sigma-Point Kalman Filter in an Error State Formulation. Dissertation for Master degree. Department of Space and Plasma Physics, School of Electrical Engineering, KTH Royal Institute of Technology, Stockholm, Sweden. 2017. [Link](http://www.diva-portal.org/smash/get/diva2:1141205/FULLTEXT01.pdf). A 30-minutes static measurement of a gyroscope was made with and its Allan Variance plot is presented by using the NaveGo functions.

16. Shaoxing Hu, Shike Xu 1, Duhu Wang and Aiwu Zhang. Optimization Algorithm for Kalman Filter Exploiting the Numerical Characteristics of SINS/GPS Integrated Navigation Systems.  Sensors 2015, 15(11), 28402-28420. [Link](https://www.mdpi.com/117278). The mathematical model of this work is base on NaveGo's proposed mathematical model.

# Acknowledgments

We would like to thank to the many people that have contributed to make NaveGo a better tool:

* M.Sc. Johann Diep for contributing with code and examples for VISUAL/INS/GNSS integration as part of his research at the European Space Agency (ESA). You can learn more about his research by watching [this video](https://youtu.be/sPnPMBs1gSU).

* Mr. Daniel Lu, GIS Analyst at Western Heritage (Canada), for his comments on the sign of acceleration when accelerometers sense gravity.

* Dr. Paolo Zoccarato for his comments on attitude conversion.

* Dr. Paolo Dabove and Dr. Marco Piras (both from DIATI, Politecnico di Torino, Italy) for helping to debug NaveGo and suggesting new features.

* Prof. Zhu, Dr. Yang, and Mr. Bo Sun, all from the Laboratory of Precision Measuring Technology and Instruments, Tianjin University, Tianjin, China, for contributing with IMU static measurements to test Allan variance routines.

* Dr. Charles K. Toth (The Ohio State University, USA), Dr. Allison Kealy, and M.Sc. Azmir Hasnur-Rabiain (both from The University of Melbourne, Australia) for generously sharing IMU and GNSS datasets, and in particular, for Azmir's unselfish support and help.

* Dr. Juan Ignacio Giribet (National University of Buenos Aires, Argentina) for his continuous support on theory aspects of INS/GNSS systems.
