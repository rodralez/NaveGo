% navego_allan_example: example of how to implement the Allan variance 
% procedure with NaveGo functions.
%
%   Copyright (C) 2014, Rodrigo Gonzalez, all rights reserved.
%
%   This file is part of NaveGo, an open-source MATLAB toolbox for
%   simulation of integrated navigation systems.
%
%   NaveGo is free software: you can redistribute it and/or modify
%   it under the terms of the GNU Lesser General Public License (LGPL)
%   version 3 as published by the Free Software Foundation.
%
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU Lesser General Public License for more details.
%
%   You should have received a copy of the GNU Lesser General Public
%   License along with this program. If not, see
%   <http://www.gnu.org/licenses/>.
%
% References:
%
%       Sensonor AS. Sensonor STIM300 Inertial Measurement Unit 
%       http://www.sensonor.com/gyro-products/inertial-measurement-units/stim300.aspx7.
%
%       Two hours of static measurements from STIM300 IMU were generously
%       provided by Prof. Zhu, Dr. Yang, and Mr. Bo Sun, all from the Laboratory 
%       of Precision Measuring Technology and Instruments, Tianjin University, 
%       Tianjin, China.
%
% Version: 003
% Date:    2017/07/28
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

clc
clear
close all
matlabrc

versionstr = 'NaveGo, release v1.0';

fprintf('\n%s.\n', versionstr)
fprintf('\nNaveGo: Allan variance analysis from real IMU STIM300... \n')

load stim300

%% ALLAN VARIANCE FOR STIM300 IMU

% IMU data structure:
%         t: Ix1 time vector (seconds).
%        fb: Ix3 accelerations vector in body frame XYZ (m/s^2).
%        wb: Ix3 turn rates vector in body frame XYZ (radians/s).
%       arw: 1x3 angle random walks (rad/s/root-Hz).
%      arrw: 1x3 angle rate random walks (rad/s^2/root-Hz).
%       vrw: 1x3 velocity random walks (m/s^2/root-Hz).
%      vrrw: 1x3 velocity rate random walks (m/s^3/root-Hz).
%      gstd: 1x3 gyros standard deviations (radians/s).
%      astd: 1x3 accrs standard deviations (m/s^2).
%    gb_fix: 1x3 gyros static biases or turn-on biases (radians/s).
%    ab_fix: 1x3 accrs static biases or turn-on biases (m/s^2).
%  gb_drift: 1x3 gyros dynamic biases or bias instabilities (radians/s).
%  ab_drift: 1x3 accrs dynamic biases or bias instabilities (m/s^2).
%   gb_corr: 1x3 gyros correlation times (seconds).
%   ab_corr: 1x3 accrs correlation times (seconds).
%     gpsd : 1x3 gyros dynamic biases PSD (rad/s/root-Hz).
%     apsd : 1x3 accrs dynamic biases PSD (m/s^2/root-Hz);
%      freq: 1x1 sampling frequency (Hz).

to = (stim300.t(end) - stim300.t(1));
fprintf('NaveGo: dataset duration is %.2f hours or %.2f minutes or %.2f seconds. \n\n', (to/60/60), (to/60), to)

[stim300] = allan_imu (stim300);

stim300_arw = stim300.arw
stim300_vrw = stim300.vrw

stim300_ab_drift = stim300.ab_drift
stim300_gb_drift = stim300.gb_drift

stim300_ab_corr = stim300.ab_corr
stim300_gb_corr = stim300.gb_corr

%% ALLAN VARIANCE FOR SIMULATED IMU

fprintf('NaveGo: Allan variance analysis from simulated Microstrain 3DM-GX3-35 IMU... \n')

%% REF DATASET

% Simulated static data
N = 1917878;
M = [N, 3];

ref.freq = 100;

ref.t = ((0:N-1)/ref.freq)';   % Simulated time vector is about 5.3275 hours
to = (ref.t(end) - ref.t(1));
fprintf('NaveGo: dataset duration is %.2f hours or %.2f minutes or %.2f seconds. \n', (to/60/60), (to/60), to)

ref.lat = zeros(N,1);
ref.vel = zeros(M); 
ref.h   = zeros(N,1);

ref.fb = zeros(M);
ref.wb = zeros(M);

%     DCMnb: Nx9 Direct Cosine Matrix nav-to-body. Each row contains 
%            the elements of one matrix ordered by columns as 
%            [a11 a21 a31 a12 a22 a32 a13 a23 a33].

ref.DCMnb = zeros(N,9);
ref.DCMnb(:,1) = ones(N,1);
ref.DCMnb(:,5) = ones(N,1);
ref.DCMnb(:,9) = ones(N,1);

%% Microstrain 3DM-GX3-35 IMU

% IMU data structure:
%         t: Ix1 time vector (seconds).
%        fb: Ix3 accelerations vector in body frame XYZ (m/s^2).
%        wb: Ix3 turn rates vector in body frame XYZ (radians/s).
%       arw: 1x3 angle random walks (rad/s/root-Hz).
%      arrw: 1x3 angle rate random walks (rad/s^2/root-Hz).
%       vrw: 1x3 velocity random walks (m/s^2/root-Hz).
%      vrrw: 1x3 velocity rate random walks (m/s^3/root-Hz).
%      gstd: 1x3 gyros standard deviations (radians/s).
%      astd: 1x3 accrs standard deviations (m/s^2).
%    gb_fix: 1x3 gyros static biases or turn-on biases (radians/s).
%    ab_fix: 1x3 accrs static biases or turn-on biases (m/s^2).
%  gb_drift: 1x3 gyros dynamic biases or bias instabilities (radians/s).
%  ab_drift: 1x3 accrs dynamic biases or bias instabilities (m/s^2).
%   gb_corr: 1x3 gyros correlation times (seconds).
%   ab_corr: 1x3 accrs correlation times (seconds).
%     gpsd : 1x3 gyros dynamic biases PSD (rad/s/root-Hz).
%     apsd : 1x3 accrs dynamic biases PSD (m/s^2/root-Hz);
%      freq: 1x1 sampling frequency (Hz).

ustrain.astd = [0.00643187932253599  0.00661386698561032  0.00673225201283004];
ustrain.gstd = [0.00272391738310747  0.00248849782611228  0.00272332577563485];

ustrain.ab_fix = [1.73301445792617e-13 -7.93732502701179e-13 -1.84847751355576e-13];
ustrain.gb_fix = [4.00424136983284e-14 4.98197419961447e-15 -6.5696457219509e-15];
      
ustrain.ab_corr = [40 20 1000];
ustrain.gb_corr = [500 700 200];
   
ustrain.vrrw = [0.00031522133759985 0.000519606636158211 0.000396688807571295];      
ustrain.arrw = [8.21484738626e-05 4.54275740041735e-05 0.000103299115514897]; 

ustrain.ab_drift = [0.000252894096875598 0.000349683866037958 0.000323068534025731];
ustrain.gb_drift = [7.6339404800228e-05 4.50248175403541e-05 8.75796277840371e-05];

ustrain.freq = ref.freq;
ustrain.t = ref.t;

fprintf('NaveGo: generating IMU ACCR data... \n')

fb = acc_gen (ref, ustrain);   % Generate acc in the body frame
ustrain.fb = fb;

fprintf('NaveGo: generating IMU GYRO data... \n')

wb = gyro_gen (ref, ustrain);  % Generate gyro in the body frame
ustrain.wb = wb;

%% ALLAN VARIANCE

[ustrain_allan] = allan_imu (ustrain);

ustrain_allan_arw = ustrain_allan.arw
ustrain_allan_vrw = ustrain_allan.vrw

ustrain_allan_ab_drift = ustrain_allan.ab_drift
ustrain_allan_gb_drift = ustrain_allan.gb_drift

ustrain_allan_ab_corr = ustrain_allan.ab_corr
ustrain_allan_gb_corr = ustrain_allan.gb_corr
