# NaveGo

[![Releases](https://img.shields.io/badge/release-v1.1-green.svg?style=plastic)](https://github.com/rodralez/NaveGo/releases) [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.1464202.svg)](https://doi.org/10.5281/zenodo.1464202)

NaveGo: an open-source MATLAB/GNU-Octave toolbox for processing integrated navigation systems and performing inertial sensors profiling analysis.

NaveGo is an open-source MATLAB/GNU Octave toolbox for processing integrated navigation systems and simulating inertial sensors and a GNSS receiver. It also performs inertial sensors analysis using the Allan variance. It is freely available online. It is developed under MATLAB/GNU-Octave due to this programming language has become a *de facto* standard for simulation and mathematical computing. NaveGo has been verified by processing real-world data from a real trajectory and contrasting results against a commercial, closed-source software package. Difference between both solutions have shown to be negligible. For more information read the following [paper](http://www.scitepress.org/Papers/2017/63139/index.html).

NaveGo is supported at the moment by three academic research groups: GridTics at the National University of Technology (Argentina), Engineering School at the National University of Cuyo (Argentina), and DIATI at the Politecnico di Torino (Italy). 

## Features

Main features of NaveGo are:

* Processing of an inertial navigation system (INS).

* Processing of a loosely-coupled integrated navigation system (INS/GNSS).

* Simulation of inertial sensors and GNSS.

* Zero Velocity Update (ZUPT) detection algorithm.

* Allan variance technique to characterize inertial sensors' typical errors.

## Please, cite our work

If you are using NaveGo in your research, we gently ask you to add the following two cites to your future papers:

* R. Gonzalez, C.A. Catania, P. Dabove, J.C. Taffernaberry, and M. Piras. Model validation of an open-source framework for post-processing INS/GNSS systems. III International Conference on Geographical Information Systems Theory, Applications and Management (GISTAM 2017). Porto, Portugal. April 2017. [Download](http://www.scitepress.org/Papers/2017/63139/index.html).

* R. Gonzalez, J.I. Giribet, and H.D. Patiño. NaveGo: a simulation framework for low-cost integrated navigation systems, Journal of Control Engineering and Applied Informatics, vol. 17, issue 2, pp. 110-120, 2015. [Download](http://ceai.srait.ro/index.php?journal=ceai&page=article&op=view&path%5B%5D=2478).

An URL to NaveGo should be provided as the following cite:

R. Gonzalez, C. Catania, and P. Dabove. NaveGo: An Open-Source MATLAB/GNU-Octave Toolbox for Processing Integrated Navigation Systems and Performing Inertial Sensors Profiling Analysis. Version 1.1. URL: https://github.com/rodralez/NaveGo. DOI: doi.org/10.5281/zenodo.1464202. October 2018.

## Contributions

We are looking for contributors to NaveGo! Since integrated navigation is a topic used in several fields as Geomatics, Geology, Mobile Mapping, Autonomous Driving, and even Veterinary (yes, Veterinary!) for animal tracking, we hope other communities other than the navigation community compromise and contribute to this open-source project.

You can contribute in many ways: 

* Writing code. 
* Writing a manual. 
* Reporting bugs.
* Suggesting new features.

If you are interested in joining to NaveGo, please feel free to contact Dr. Rodrigo Gonzalez at rodralez [at] frm [dot] utn [dot] edu [dot] ar.

## Publications

The underlying mathematical model of NaveGo is based on two articles whose reading is recommended: 

* R. Gonzalez, J.I. Giribet, and H.D. Patiño. NaveGo: a simulation framework for low-cost integrated navigation systems, Journal of Control Engineering and Applied Informatics, vol. 17, issue 2, pp. 110-120, 2015. [Download](http://ceai.srait.ro/index.php?journal=ceai&page=article&op=view&path%5B%5D=2478).

* R. Gonzalez, J.I. Giribet, and H.D. Patiño. An approach to benchmarking of loosely coupled low-cost navigation systems. Mathematical and Computer Modelling of Dynamical Systems, vol. 21, issue 3, pp. 272-287, 2015. [Download](http://www.tandfonline.com/doi/abs/10.1080/13873954.2014.952642).

Other publications related to the development of NaveGo:

* R. Gonzalez, E.M. Martinez, and P. Dabove. Assessment of Discrete Stochastic Models of MEMS Inertial Sensors by Using the Allan Variance. In the III International Conference on Sensors and Electronics Instrumentation Advances (SEIA' 2017), 20-22 September 2017, Moscow, Russia.

* R. Gonzalez, C.A. Catania, P. Dabove, J.C. Taffernaberry, and M. Piras. Model validation of an open-source framework for post-processing INS/GNSS systems. III International Conference on Geographical Information Systems Theory, Applications and Management (GISTAM 2017). Porto, Portugal. April 2017.


## Roadmap

Future features of NaveGo will be:

* Tightly-coupled INS/GNSS. 

* RTS smoother.

* Adaptive Kalman filter.


## Who is using NaveGo?

The following authors have expressed the use of NaveGo within their research framework:

* R. Rabiee, X. Zhong, Y. Yan and W. P. Tay, LaIF: A Lane-Level Self-Positioning Scheme for Vehicles in GNSS-Denied Environments. IEEE Transactions on Intelligent Transportation Systems. doi: 10.1109/TITS.2018.2870048. [Link](http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8489926&isnumber=4358928).

* M. Pachwicewicz and J. Weremczuk, Accuracy Estimation of the Sounding Rocket Navigation System. 2018 XV International Scientific Conference on Optoelectronic and Electronic Sensors (COE), Warsaw, 2018, pp. 1-4. doi: 10.1109/COE.2018.8435180. [Link](http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8435180&isnumber=8435144).

* Ren, X., Sun, M., Jiang, C., Liu, L., & Huang, W. (2018). An Augmented Reality Geo-Registration Method for Ground Target Localization from a Low-Cost UAV Platform. Sensors, 2018, vol. 18, no 11, p. 3739. [Link](https://www.mdpi.com/1424-8220/18/11/3739/htm).

* D. M G and A. A, "Analysis of INS Parameters and Error Reduction by Integrating GPS and INS Signals," 2018 International Conference on Design Innovations for 3Cs Compute Communicate Control (ICDI3C), Bangalore, 2018, pp. 18-23. doi: 10.1109/ICDI3C.2018.00013.   [Link](http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8436621&isnumber=8436608).

* Walendziuk, Wojciech; Oldziej, Daniel; Szatyłowicz, Ewa; Slowik, Maciej. Unmanned aerial vehicle as a measurement tool in engineering and environmental protection. Conference: Photonics Applications in Astronomy, Communications, Industry, and High-Energy Physics Experiments 2018. DOI: 10.1117/12.2501378. 

## Acknowledgments

We would like to thank to many people that have contribute to make NaveGo a better tool:

* Dr. Juan Ignacio Giribet (National University of Buenos Aires, Argentina) for this continuous support on theory aspects of INS/GNSS systems.

* Dr. Charles K. Toth (The Ohio State University, USA), Dr. Allison Kealy, and M.Sc. Azmir Hasnur-Rabiain (both from The University of Melbourne, Australia) for generously sharing IMU and GNSS datasets, and in particular, for Azmir's unselfish support and help.

* Prof. Zhu, Dr. Yang, and Mr. Bo Sun, all from the Laboratory of Precision Measuring Technology and Instruments, Tianjin University, Tianjin, China, for contributing with IMU static measurements to test Allan variance routines.

* Dr. Paolo Dabove and Dr. Marco Piras (both from DIATI, Politecnico di Torino, Italy) for helping to debug NaveGo and suggesting new features.


# Examples

The `example` folder contains several types of examples.

## Allan variance example

Just execute the file `navego_example_allan.m`. Firstly, it process 2-hours of static measurements from an Sensonor STIM300 IMU. Then, it process about 5 hours of synthetic inertial data.

## INS/GNSS integration example using real data

An example of how to use NaveGo to post-process real data is provided by `navego_example_real.m`. This script integrates measurements coming from an Ekinox-D IMU and Ekinox-D GNSS. This dataset was generated by driving a vehicle through the streets of Turin city (Italy).

## INS/GNSS integration example using synthetic (simulated) data

The file `navego_example_synth.m` tries to demonstrate how NaveGo works. It compares the performances of two simulated INS/GNSS systems, one using an ADIS16405 IMU and another using an ADIS16488 IMU, both fused using a simulated GNSS receptor.

Next, a description of this file.

### Reset section

```matlab

clc
close all
clear
matlabrc

addpath ../../
addpath ../../simulation/
addpath ../../conversions/

versionstr = 'NaveGo, release v1.2';

fprintf('\n%s.\n', versionstr)
fprintf('\nNaveGo: starting simulation ... \n')

```
###  Code execution parameters

```matlab

% Comment any of the following parameters in order to NOT execute a particular portion of code

GNSS_DATA = 'ON';   % Generate synthetic GNSS data
IMU1_DATA = 'ON';   % Generate synthetic ADIS16405 IMU data
IMU2_DATA = 'ON';   % Generate synthetic ADIS16488 IMU data

IMU1_INS  = 'ON';   % Execute INS/GNSS integration for ADIS16405 IMU
IMU2_INS  = 'ON';   % Execute INS/GNSS integration for ADIS16488 IMU

PLOT      = 'ON';   % Plot results.

% If a particular parameter is commented above, it is set by default to 'OFF'.

if (~exist('GNSS_DATA','var')),  GNSS_DATA  = 'OFF'; end
if (~exist('IMU1_DATA','var')), IMU1_DATA = 'OFF'; end
if (~exist('IMU2_DATA','var')), IMU2_DATA = 'OFF'; end
if (~exist('IMU1_INS','var')),  IMU1_INS  = 'OFF'; end
if (~exist('IMU2_INS','var')),  IMU2_INS  = 'OFF'; end
if (~exist('PLOT','var')),      PLOT      = 'OFF'; end

```

### Conversion constants

```matlab

G =  9.80665;       % Gravity constant, m/s^2
G2MSS = G;          % g to m/s^2
MSS2G = (1/G);      % m/s^2 to g

D2R = (pi/180);     % degrees to radians
R2D = (180/pi);     % radians to degrees

KT2MS = 0.514444;   % knot to m/s
MS2KMH = 3.6;       % m/s to km/h

```

### Load reference data

```matlab

fprintf('NaveGo: loading reference dataset from a trajectory generator... \n')

load ref.mat

% ref.mat contains the reference data structure from which inertial 
% sensors and GNSS wil be simulated. It must contain the following fields:

%         t: Nx1 time vector (seconds).
%       lat: Nx1 latitude (radians).
%       lon: Nx1 longitude (radians).
%         h: Nx1 altitude (m).
%       vel: Nx3 NED velocities (m/s).
%      roll: Nx1 roll angles (radians).
%     pitch: Nx1 pitch angles (radians).
%       yaw: Nx1 yaw angle vector (radians).
%     DCMnb: Nx9 Direct Cosine Matrix nav-to-body. Each row contains 
%            the elements of one DCM matrix ordered by columns as 
%            [a11 a21 a31 a12 a22 a32 a13 a23 a33].
%      freq: sampling frequency (Hz).


```

### ADIS16405 IMU error profile

```matlab

% IMU data structure:
%         t: Ix1 time vector (seconds).
%        fb: Ix3 accelerations vector in body frame XYZ (m/s^2).
%        wb: Ix3 turn rates vector in body frame XYZ (radians/s).
%       arw: 1x3 angle random walks (rad/s/root-Hz).
%      arrw: 1x3 angle rate random walks (rad/s^2/root-Hz).
%       vrw: 1x3 velocity random walks (m/s^2/root-Hz).
%      vrrw: 1x3 velocity rate random walks (m/s^3/root-Hz).
%    g_std: 1x3 gyros standard deviations (radians/s).
%    a_std: 1x3 accrs standard deviations (m/s^2).
%    gb_sta: 1x3 gyros static biases or turn-on biases (radians/s).
%    ab_sta: 1x3 accrs static biases or turn-on biases (m/s^2).
%    gb_dyn: 1x3 gyros dynamic biases or bias instabilities (radians/s).
%    ab_dyn: 1x3 accrs dynamic biases or bias instabilities (m/s^2).
%   gb_corr: 1x3 gyros correlation times (seconds).
%   ab_corr: 1x3 accrs correlation times (seconds).
%    gb_psd: 1x3 gyros dynamic biases PSD (rad/s/root-Hz).
%    ab_psd: 1x3 accrs dynamic biases PSD (m/s^2/root-Hz);
%      freq: 1x1 sampling frequency (Hz).
% ini_align: 1x3 initial attitude at t(1), [roll pitch yaw] (rad).
% ini_align_err: 1x3 initial attitude errors at t(1), [roll pitch yaw] (rad).

ADIS16405.arw      = 2   .* ones(1,3);     % Angle random walks [X Y Z] (deg/root-hour)
ADIS16405.arrw     = zeros(1,3);           % Angle rate random walks [X Y Z] (deg/root-hour/s)
ADIS16405.vrw      = 0.2 .* ones(1,3);     % Velocity random walks [X Y Z] (m/s/root-hour)
ADIS16405.vrrw     = zeros(1,3);           % Velocity rate random walks [X Y Z] (deg/root-hour/s)
ADIS16405.gb_sta   = 3   .* ones(1,3);     % Gyro static biases [X Y Z] (deg/s)
ADIS16405.ab_sta   = 50  .* ones(1,3);     % Acc static biases [X Y Z] (mg)
ADIS16405.gb_dyn   = 0.007 .* ones(1,3);   % Gyro dynamic biases [X Y Z] (deg/s)
ADIS16405.ab_dyn   = 0.2 .* ones(1,3);     % Acc dynamic biases [X Y Z] (mg)
ADIS16405.gb_corr  = 100 .* ones(1,3);     % Gyro correlation times [X Y Z] (seconds)
ADIS16405.ab_corr  = 100 .* ones(1,3);     % Acc correlation times [X Y Z] (seconds)
ADIS16405.freq     = ref.freq;             % IMU operation frequency [X Y Z] (Hz)
% ADIS16405.m_psd     = 0.066 .* ones(1,3);  % Magnetometer noise density [X Y Z] (mgauss/root-Hz)

% ref dataset will be used to simulate IMU sensors.
ADIS16405.t = ref.t;                       % IMU time vector
dt = mean(diff(ADIS16405.t));              % IMU sampling interval

imu1 = imu_si_errors(ADIS16405, dt);       % Transform IMU manufacturer error units to SI units.

imu1.ini_align_err = [3 3 10] .* D2R;                   % Initial attitude align errors for matrix P in Kalman filter, [roll pitch yaw] (radians)  
imu1.ini_align = [ref.roll(1) ref.pitch(1) ref.yaw(1)]; % Initial attitude align at t(1) (radians).

```

### ADIS16488 IMU error profile

```matlab

ADIS16488.arw      = 0.3  .* ones(1,3);     % Angle random walks [X Y Z] (deg/root-hour)
ADIS16488.arrw     = zeros(1,3);            % Angle rate random walks [X Y Z] (deg/root-hour/s)
ADIS16488.vrw      = 0.029.* ones(1,3);     % Velocity random walks [X Y Z] (m/s/root-hour)
ADIS16488.vrrw     = zeros(1,3);            % Velocity rate random walks [X Y Z] (deg/root-hour/s)
ADIS16488.gb_sta   = 0.2  .* ones(1,3);     % Gyro static biases [X Y Z] (deg/s)
ADIS16488.ab_sta   = 16   .* ones(1,3);     % Acc static biases [X Y Z] (mg)
ADIS16488.gb_dyn   = 6.5/3600  .* ones(1,3);% Gyro dynamic biases [X Y Z] (deg/s)
ADIS16488.ab_dyn   = 0.1  .* ones(1,3);     % Acc dynamic biases [X Y Z] (mg)
ADIS16488.gb_corr  = 100  .* ones(1,3);     % Gyro correlation times [X Y Z] (seconds)
ADIS16488.ab_corr  = 100  .* ones(1,3);     % Acc correlation times [X Y Z] (seconds)
ADIS16488.freq     = ref.freq;              % IMU operation frequency [X Y Z] (Hz)
% ADIS16488.m_psd = 0.054 .* ones(1,3);       % Magnetometer noise density [X Y Z] (mgauss/root-Hz)

% ref dataset will be used to simulate IMU sensors.
ADIS16488.t = ref.t;                        % IMU time vector
dt = mean(diff(ADIS16488.t));               % IMU sampling interval

imu2 = imu_si_errors(ADIS16488, dt);        % Transform IMU manufacturer error units to SI units.

imu2.ini_align_err = [1 1 5] .* D2R;                     % Initial attitude align errors for matrix P in Kalman filter, [roll pitch yaw] (radians)  
imu2.ini_align = [ref.roll(1) ref.pitch(1) ref.yaw(1)];  % Initial attitude align at t(1) (radians).

```

### Garmin 5-18 Hz GPS error profile

```matlab

% GNSS data structure:
%         t: Mx1 time vector (seconds).
%       lat: Mx1 latitude (radians).
%       lon: Mx1 longitude (radians).
%         h: Mx1 altitude (m).
%       vel: Mx3 NED velocities (m/s).
%       std: 1x3 position standard deviations, [lat lon h] (rad, rad, m).
%      stdm: 1x3 position standard deviations, [lat lon h] (m, m, m).
%      stdv: 1x3 velocity standard deviations, [Vn Ve Vd] (m/s).
%      larm: 3x1 lever arm from IMU to GNSS antenna (x-fwd, y-right, z-down) (m).
%      freq: 1x1 sampling frequency (Hz).
%   zupt_th: 1x1 ZUPT threshold (m/s).
%  zupt_win: 1x1 ZUPT time window (seconds).
%       eps: 1x1 time interval to compare IMU time vector to GNSS time vector (seconds).

gnss.stdm = [5 5 10];                   % GNSS positions standard deviations [lat lon h] (meters)
gnss.stdv = 0.1 * KT2MS .* ones(1,3);   % GNSS velocities standard deviations [Vn Ve Vd] (meters/s)
gnss.larm = zeros(3,1);                 % GNSS lever arm from IMU to GNSS antenna (x-fwd, y-right, z-down) (m).
gnss.freq = 5;                          % GNSS operation frequency (Hz)

% Parameters for ZUPT detection algorithm
gnss.zupt_th = 0.5;   % ZUPT threshold (m/s).
gnss.zupt_win = 4;    % ZUPT time window (seconds).

gnss.eps = 1E-3;

```

### Generate GNSS synthetic data

```matlab

rng('shuffle')                  % Reset pseudo-random seed

if strcmp(GNSS_DATA, 'ON')       % If simulation of GNSS data is required ...
    
    fprintf('NaveGo: generating GNSS synthetic data... \n')
    
    gnss = gnss_err_profile(ref.lat(1), ref.h(1), gnss); % Transform GNSS manufacturer error units to SI units.
    
    gnss = gnss_gen(ref, gnss);  % Generate GNSS dataset from reference dataset.

    save gnss.mat gnss
    
else
    
    fprintf('NaveGo: loading GNSS data... \n') 
    
    load gnss.mat
end

```

### Generate IMU1 synthetic data 

```matlab

rng('shuffle')                  % Reset pseudo-random seed

if strcmp(IMU1_DATA, 'ON')      % If simulation of IMU1 data is required ...
    
    fprintf('NaveGo: generating IMU1 ACCR synthetic data... \n')
    
    fb = acc_gen (ref, imu1);   % Generate acc in the body frame
    imu1.fb = fb;
    
    fprintf('NaveGo: generating IMU1 GYRO synthetic data... \n')
    
    wb = gyro_gen (ref, imu1);  % Generate gyro in the body frame
    imu1.wb = wb;
    
    save imu1.mat imu1
    
    clear wb fb;
    
else
    fprintf('NaveGo: loading IMU1 data... \n')
    
    load imu1.mat
end

```

### Generate IMU2 synthetic data

```matlab

rng('shuffle')					% Reset pseudo-random seed

if strcmp(IMU2_DATA, 'ON')      % If simulation of IMU2 data is required ...
    
    fprintf('NaveGo: generating IMU2 ACCR synthetic data... \n')
    
    fb = acc_gen (ref, imu2);   % Generate acc in the body frame
    imu2.fb = fb;
    
    fprintf('NaveGo: generating IMU2 GYRO synthetic data... \n')
    
    wb = gyro_gen (ref, imu2);  % Generate gyro in the body frame
    imu2.wb = wb;
    
    save imu2.mat imu2
    
    clear wb fb;
    
else
    fprintf('NaveGo: loading IMU2 data... \n')
    
    load imu2.mat
end

```

### Print navigation time

```matlab

to = (ref.t(end) - ref.t(1));

fprintf('NaveGo: navigation time is %.2f minutes or %.2f seconds. \n', (to/60), to)o)

```

### INS/GNSS integration using IMU1

```matlab

if strcmp(IMU1_INS, 'ON')
    
    fprintf('NaveGo: INS/GNSS navigation estimates for IMU1... \n')
    
    % Execute INS/GNSS integration
    % ---------------------------------------------------------------------
    nav1_e = ins_gnss(imu1, gnss, 'dcm');
    % ---------------------------------------------------------------------
    
    save nav1_e.mat nav1_e
    
else
    
    fprintf('NaveGo: loading INS/GNSS integration for IMU1... \n')
    
    load nav1_e.mat
end

```

### INS/GNSS integration using IMU2

```matlab

if strcmp(IMU2_INS, 'ON')
    
    fprintf('NaveGo: INS/GNSS navigation estimates for IMU2... \n')
    
    % Execute INS/GNSS integration
    % ---------------------------------------------------------------------
    nav2_e = ins_gnss(imu2, gnss, 'quaternion');
    % ---------------------------------------------------------------------
    
    save nav2_e.mat nav2_e
    
else
    
    fprintf('NaveGo: loading INS/GNSS integration for IMU2... \n')
    
    load nav2_e.mat
end

```

### Interpolate INS/GNSS dataset 

```matlab

% INS/GNSS estimates and GNSS data are interpolated according to the
% reference dataset.

[nav1_ref, ref_1] = navego_interpolation (nav1_e, ref);
[nav2_ref, ref_2] = navego_interpolation (nav2_e, ref);
[gnss_ref, ref_g] = navego_interpolation (gnss, ref);

```

### Print on console RMSE from INS/GNSS IMU1

```matlab

print_rmse (nav1_ref, gnss_ref, ref_1, ref_g, 'INS/GNSS IMU1');

```

### Print on console RMSE from INS/GNSS IMU2

```matlab

print_rmse (nav2_ref, gnss_ref, ref_2, ref_g, 'INS/GNSS IMU2');

```

### References

* R. Gonzalez, J.I. Giribet, and H.D. Patiño. NaveGo: a simulation framework for low-cost integrated navigation systems, Journal of Control Engineering and Applied Informatics, vol. 17, issue 2, pp. 110-120, 2015. [Download](http://ceai.srait.ro/index.php?journal=ceai&page=article&op=view&path%5B%5D=2478).

* Analog Devices. ADIS16400/ADIS16405 datasheet. High Precision Tri-Axis Gyroscope, Accelerometer, Magnetometer. Rev. B. 
[Download](http://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16400_16405.pdf).

* Analog Devices. ADIS16488 datasheet. Tactical Grade Ten Degrees of Freedom Inertial Sensor. Rev. G. 
[Download](http://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16488.pdf).

* Garmin International, Inc. GPS 18x TECHNICAL SPECIFICATIONS. Revision D. October 2011. 
[Download](http://static.garmin.com/pumac/GPS_18x_Tech_Specs.pdf).
