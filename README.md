# NaveGo

[![Releases](https://img.shields.io/badge/release-v0.7.1--alpha-green.svg?style=plastic)](https://github.com/rodralez/NaveGo/releases) [![DOI](https://zenodo.org/badge/12745155.svg)](https://zenodo.org/badge/latestdoi/12745155)


NaveGo: an open-source MATLAB toolbox for simulating integrated navigation systems.

## Features

Main features of NaveGo are:

* Simulation of an inertial navigation system (INS).

* Simulation of a loosely-coupled integrated navigation system (INS/GPS).

* Implementation of the Allan variance procedure in order to characterize the typical errors of an IMU. 

* Simulation of IMU sensors (in a very early stage).

## Contributions

We are looking for contributors for NaveGo! Until now, NaveGo is a one-man project. We hope the navigation community compromise and contribute with this open-source tool.

You can contribute in many ways: 

* Writing code. 
* Writing a manual. 
* Reporting bugs.
* Suggesting new features.

If you are interested, please feel free to contact Dr. Rodrigo Gonzalez at rodralez [at] frm [dot] utn [dot] edu [dot] ar.

## Documentation

The underlying mathematical model of NaveGo is based on two articles that are recommended for reading: 

* R. Gonzalez, J.I. Giribet, and H.D. Patiño. NaveGo: a simulation framework for low-cost integrated navigation systems, Journal of Control Engineering and Applied Informatics, vol. 17, issue 2, pp. 110-120, 2015. [Link](http://ceai.srait.ro/index.php?journal=ceai&page=article&op=view&path%5B%5D=2478).

* R. Gonzalez, J.I. Giribet, and H.D. Patiño. An approach to benchmarking of loosely coupled low-cost navigation systems. Mathematical and Computer Modelling of Dynamical Systems, vol. 21, issue 3, pp. 272-287, 2015. [Link](http://www.tandfonline.com/doi/abs/10.1080/13873954.2014.952642).

## Working example

The file `navego_example_of_use.m` tries to demonstrate the use of NaveGo. It compares the performances of two simulated IMUs, ADIS16405 IMU and ADIS16488 IMU, integrated with a simulated GPS.

Next, a description of this file.

### Reset section

```matlab

clc
close all
clear
matlabrc

fprintf('\nStarting simulation ... \n')
```

###  Global variables

```matlab
global D2R
global R2D
```
###  Code execution parameters

```matlab

% Comment any of the following parameters in order to NOT execute a particular portion of code

GPS_DATA  = 'ON';   % Simulate GPS data
IMU1_DATA = 'ON';   % Simulate ADIS16405 IMU data
IMU2_DATA = 'ON';   % Simulate ADIS16488 IMU data

IMU1_INS  = 'ON';   % Execute INS/GPS integration for ADIS16405 IMU
IMU2_INS  = 'ON';   % Execute INS/GPS integration for ADIS16488 IMU

RMSE      = 'ON';   % Show on consolte RMSE results.
PLOT      = 'ON';   % Plot results.

% If a particular parameter is commented above, set its value to 'OFF'.

if (~exist('GPS_DATA','var')),  GPS_DATA  = 'OFF'; end
if (~exist('IMU1_DATA','var')), IMU1_DATA = 'OFF'; end
if (~exist('IMU2_DATA','var')), IMU2_DATA = 'OFF'; end
if (~exist('IMU1_INS','var')),  IMU1_INS = 'OFF'; end
if (~exist('IMU2_INS','var')),  IMU2_INS = 'OFF'; end
if (~exist('RMSE','var')),      RMSE = 'OFF'; end
if (~exist('PLOT','var')),      PLOT = 'OFF'; end

```

### Conversion constants

```matlab

G = 9.81;           % Gravity constant, m/s^2
G2MSS = G;          % g to m/s^2
MSS2G = (1/G);      % m/s^2 to g

D2R = (pi/180);     % degrees to radians
R2D = (180/pi);     % radians to degrees

KT2MS = 0.514444;   % knot to m/s
MS2KMH = 3.6;       % m/s to km/h

```

### Load reference data

```matlab

fprintf('Loading reference dataset from a trajectory generator... \n')

load ref.mat

% ref.mat contains the reference data structure from which inertial 
% sensors and GPS wil be simulated. It must contain the following fields:

%         t: time vector (seconds).
%       lat: latitude vector (radians).
%       lon: longitude vector (radians).
%         h: altitude vector (meters).
%       vel: NED velocities vectors, [north east down] (meter/s).
%      roll: roll angle vector (radians).
%     pitch: pitch angle vector (radians).
%       yaw: yaw angle vector (radians).
%        kn: number of elements of time vector.
%     DCMnb: Direct Cosine Matrix nav-to-body, with 'kn' rows and 9
%     columns. Each row contains the elements of one matrix ordered by
%     columns as [a11 a21 a31 a12 a22 a32 a13 a23 a33]. Use reshape()
%     built-in MATLAB function to get the original 3x3 matrix
%     (reshape(DCMnb(row,:),3,3)).
%      freq: sampling frequency (Hz).

```

### ADIS16405 IMU error profile

```matlab

ADIS16405.arw       = 2   .* ones(1,3);     % Angle random walks [X Y Z] (deg/root-hour)
ADIS16405.vrw       = 0.2 .* ones(1,3);     % Velocity random walks [X Y Z] (m/s/root-hour)
ADIS16405.gb_fix    = 3 .* ones(1,3);       % Gyro static biases [X Y Z] (deg/s)
ADIS16405.ab_fix    = 50 .* ones(1,3);      % Acc static biases [X Y Z] (mg)
ADIS16405.gb_drift  = 0.007 .* ones(1,3);   % Gyro dynamic biases [X Y Z] (deg/s)
ADIS16405.ab_drift  = 0.2 .* ones(1,3);     % Acc dynamic biases [X Y Z] (mg)
ADIS16405.gcorr     = 100 .* ones(1,3);     % Gyro correlation times [X Y Z] (seconds)
ADIS16405.acorr     = 100 .* ones(1,3);     % Acc correlation times [X Y Z] (seconds)
ADIS16405.freq      = 100;                  % IMU operation frequency [X Y Z] (Hz)
% ADIS16405.m_psd     = 0.066 .* ones(1,3);   % Magnetometer noise [X Y Z] (mgauss/root-Hz)

% ref dataset is used to simulate IMU sensors.

dt = mean(diff(ref.t));               % IMU mean period

imu1 = imu_err_profile(ADIS16405, dt);  % Transform IMU manufacturer units to SI units

imu1.att_init = [1 1 5] .* D2R;         % Initial attitude for matrix P in Kalman filter, [roll pitch yaw] (radians)  
imu1.t = ref.t;                       % IMU time vector
imu1.freq = ref.freq;                 % IMU operation frequency

```

### ADIS16488 IMU error profile

```matlab

ADIS16488.arw = 0.3     .* ones(1,3);       % Angle random walks [X Y Z] (deg/root-hour)
ADIS16488.vrw = 0.029   .* ones(1,3);       % Velocity random walks [X Y Z] (m/s/root-hour)
ADIS16488.gb_fix = 0.2  .* ones(1,3);       % Gyro static biases [X Y Z] (deg/s)
ADIS16488.ab_fix = 16   .* ones(1,3);       % Acc static biases [X Y Z] (mg)
ADIS16488.gb_drift = 6.5/3600  .* ones(1,3);% Gyro dynamic biases [X Y Z] (deg/s)
ADIS16488.ab_drift = 0.1  .* ones(1,3);     % Acc dynamic biases [X Y Z] (mg)
ADIS16488.gcorr = 100 .* ones(1,3);         % Gyro correlation times [X Y Z] (seconds)
ADIS16488.acorr = 100 .* ones(1,3);         % Acc correlation times [X Y Z] (seconds)
ADIS16488.freq = 100;                       % IMU operation frequency [X Y Z] (Hz)
% ADIS16488.m_psd = 0.054 .* ones(1,3);        % Magnetometer noise [X Y Z] (mgauss/root-Hz)

% ref dataset is used to simulate IMU sensors.

dt = mean(diff(ref.t));               % Mean period

imu2 = imu_err_profile(ADIS16488, dt);  % Transform IMU manufacturer error units to SI units.

imu2.att_init = [0.5 0.5 1] .* D2R;     % [roll pitch yaw] Initial attitude for matrix P in Kalman filter
imu2.t = ref.t;                       % IMU time vector
imu2.freq = ref.freq;                 % IMU operation frequency

```

### Garmin 5-18 Hz GPS error profile

```matlab

gps.stdm = [5, 5, 10];                 % GPS positions standard deviations [lat lon h] (meters)
gps.stdv = 0.1 * KT2MS .* ones(1,3);   % GPS velocities standard deviations [Vn Ve Vd] (meters/s)
gps.larm = zeros(3,1);                 % GPS lever arm [X Y Z] (meters)
gps.freq = 5;                          % GPS operation frequency (Hz)

```

### Simulate GPS

```matlab

rng('shuffle')                  % Reset pseudo-random seed

if strcmp(GPS_DATA, 'ON')       % If simulation of GPS data is required ...
    
    fprintf('Simulating GPS data... \n')
    
    gps = gps_err_profile(ref.lat(1), ref.h(1), gps); % Transform GPS manufacturer error units to SI units.
    
    [gps] = gps_gen(ref, gps);  % Generate GPS dataset from reference dataset.

    save gps.mat gps
    
else
    
    fprintf('Loading GPS data... \n') 
    
    load gps.mat
end

```

### Simulate IMU1

```matlab

rng('shuffle')                  % Reset pseudo-random seed

if strcmp(IMU1_DATA, 'ON')      % If simulation of IMU1 data is required ...
    
    fprintf('Generating IMU1 ACCR data... \n')
    
    fb = acc_gen (ref, imu1); % Generate acc in the body frame
    imu1.fb = fb;
    
    fprintf('Generating IMU1 GYRO data... \n')
    
    wb = gyro_gen (ref, imu1);% Generate gyro in the body frame
    imu1.wb = wb;
    
    save imu1.mat imu1
    clear wb fb;
    
else
    fprintf('Loading IMU1 data... \n')
    
    load imu1.mat
end

```

### Simulate IMU2

```matlab

rng('shuffle')					% Reset pseudo-random seed

if strcmp(IMU2_DATA, 'ON')      % If simulation of IMU2 data is required ...
    
    fprintf('Generating IMU2 ACCR data... \n')
    
    fb = acc_gen (ref, imu2); % Generate acc in the body frame
    imu2.fb = fb;
    
    fprintf('Generating IMU2 GYRO data... \n')
    
    wb = gyro_gen (ref, imu2);% Generate gyro in the body frame
    imu2.wb = wb;
    
    save imu2.mat imu2
    
    clear wb fb;
    
else
    fprintf('Loading IMU2 data... \n')
    
    load imu2.mat
end

```

### IMU1/GPS integration with EKF

```matlab

if strcmp(IMU1_INS, 'ON')
    
    fprintf('INS/GPS integration for IMU1... \n')
    
    % Sincronize GPS data with IMU data.
    
    % Guarantee that gps.t(1) < imu1.t(1) < gps.t(2)
    if (imu1.t(1) < gps.t(1)),
        
        igx  = find(imu1.t > gps.t(1), 1, 'first' );
        
        imu1.t  = imu1.t  (igx:end, :);
        imu1.fb = imu1.fb (igx:end, :);
        imu1.wb = imu1.wb (igx:end, :);        
    end
    
    % Guarantee that imu1.t(end-1) < gps.t(end) < imu1.t(end)
    if (imu1.t(end) <= gps.t(end)),
        
        fgx  = find(gps.t < imu1.t(end), 1, 'last' );
        
        gps.t   = gps.t  (1:fgx, :);
        gps.lat = gps.lat(1:fgx, :);
        gps.lon = gps.lon(1:fgx, :);
        gps.h   = gps.h  (1:fgx, :);
        gps.vel = gps.vel(1:fgx, :);
    end
    
    % Execute INS/GPS integration
    % ---------------------------------------------------------------------
    [imu1_e] = ins(imu1, gps, ref, 'quaternion', 'double');
    % ---------------------------------------------------------------------
    
    save imu1_e.mat imu1_e
    
else
    
    fprintf('Loading INS/GPS integration for IMU1... \n')
    
    load imu1_e.mat
end

```

### IMU2/GPS integration with EKF

```matlab

if strcmp(IMU2_INS, 'ON')
    
    fprintf('\nINS/GPS integration for IMU2... \n')
    
    % Sincronize GPS data and IMU data.
    
    % Guarantee that gps.t(1) < imu2.t(1) < gps.t(2)
    if (imu2.t(1) < gps.t(1)),
        
        igx  = find(imu2.t > gps.t(1), 1, 'first' );
        
        imu2.t  = imu2.t  (igx:end, :);
        imu2.fb = imu2.fb (igx:end, :);
        imu2.wb = imu2.wb (igx:end, :);        
    end
    
    % Guarantee that imu2.t(end-1) < gps.t(end) < imu2.t(end)
    if (imu2.t(end) <= gps.t(end)),
        
        fgx  = find(gps.t < imu2.t(end), 1, 'last' );
        
        gps.t   = gps.t  (1:fgx, :);
        gps.lat = gps.lat(1:fgx, :);
        gps.lon = gps.lon(1:fgx, :);
        gps.h   = gps.h  (1:fgx, :);
        gps.vel = gps.vel(1:fgx, :);
    end
    
    % Execute INS/GPS integration
    % ---------------------------------------------------------------------
    [imu2_e] = ins(imu2, gps, ref, 'quaternion', 'double');
    % ---------------------------------------------------------------------
    
    save imu2_e.mat imu2_e
    
else
    
    fprintf('Loading INS/GPS integration for IMU2... \n')
    
    load imu2_e.mat
end

```

### Print navigation time

```matlab

to = (ref.t(end) - ref.t(1));

fprintf('\n>> Navigation time: %4.2f minutes or %4.2f seconds. \n', (to/60), to)

```

### Print RMSE from IMU1

```matlab

[ref_1, ref_g] = print_rmse (imu1_e, gps, ref, 'IMU1/GPS');

```

### Print RMSE from IMU2

```matlab

ref_2 = print_rmse (imu2_e, gps, ref, 'IMU2/GPS');

```

### References

* R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a
simulation framework for low-cost integrated navigation systems,
Journal of Control Engineering and Applied Informatics, vol. 17,
issue 2, pp. 110-120, 2015. Eq. 26.

* Analog Devices. ADIS16400/ADIS16405 datasheet. High Precision 
Tri-Axis Gyroscope, Accelerometer, Magnetometer. Rev. B. 
http://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16400_16405.pdf

* Analog Devices. ADIS16488 datasheet. Tactical Grade Ten Degrees 
of Freedom Inertial Sensor. Rev. G. 
http://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16488.pdf

* Garmin International, Inc. GPS 18x TECHNICAL SPECIFICATIONS.
Revision D. October 2011. 
http://static.garmin.com/pumac/GPS_18x_Tech_Specs.pdf
