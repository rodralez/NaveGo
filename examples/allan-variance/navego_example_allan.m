% navego_example_allan: example of how to implement the Allan variance 
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
% Version: 005
% Date:    2021/12/07
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% NOTE: NaveGo assumes that IMU is aligned with respect to body-frame as
% X-forward, Y-right and Z-down.
%
% NOTE: NaveGo assumes that yaw angle (heading) is positive clockwise.

clc
clear
close all
matlabrc

addpath ../../.
addpath ../../ins/
addpath ../../simulation/
addpath ../../conversions/
addpath ../../allan-variance/
addpath ../../plot/

D2R = (pi/180);     % degrees to radians

navego_print_version;
 
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
%     g_std: 1x3 gyros standard deviations (radians/s).
%     a_std: 1x3 accrs standard deviations (m/s^2).
%    gb_sta: 1x3 gyros static biases or turn-on biases (radians/s).
%    ab_sta: 1x3 accrs static biases or turn-on biases (m/s^2).
%    gb_dyn: 1x3 gyros dynamic biases or bias instabilities (radians/s).
%    ab_dyn: 1x3 accrs dynamic biases or bias instabilities (m/s^2).
%   gb_corr: 1x3 gyros correlation times (seconds).
%   ab_corr: 1x3 accrs correlation times (seconds).
%    gb_psd: 1x3 gyros dynamic biases root-PSD (rad/s/root-Hz).
%    ab_psd: 1x3 accrs dynamic biases root-PSD (m/s^2/root-Hz);
%      freq: 1x1 sampling frequency (Hz).

to = (stim300.t(end) - stim300.t(1));
fprintf('NaveGo: dataset time span is %.2f hours or %.2f minutes or %.2f seconds. \n\n', (to/60/60), (to/60), to)

verbose = 2;

stim300_allan = allan_imu (stim300)

%% ALLAN VARIANCE FOR SYNTHETIC IMU DATA

fprintf('\nNaveGo: Allan variance analysis from synthetic data based on Microstrain 3DM-GX3-35 IMU... \n')

%% ERROR IMU PROFILE FROM MICROSTRAIN 3DM-GX3-35

% IMU data structure:
%         t: Ix1 time vector (secoor in body frame XYZ (m/s^2).
%        wb: Ix3 turn rates vector in body frame XYZ (radians/s).
%       arw: 1x3 angle random walks (rad/s/root-Hz).
%      arrw: 1x3 angle rate random walks (rad/s^2/root-Hz).
%       vrw: 1x3 velocity random walks (m/s^2/root-Hz).
%      vrrw: 1x3 velocity rate random walks (m/s^3/root-Hz).
%    g_std: 1x3 gyros standard deviations (radians/s).
%    a_std: 1x3 accrs standard deviations (m/s^2).
%    gb_sta: 1x3 gyros static biases or turn-on biases (radians/s).
%    ab_sta: 1x3 accrs static biases or turn-on biases (m/s^2).
%  gb_dyn: 1x3 gyros dynamic biases or bias instabilities (radians/s).
%  ab_dyn: 1x3 accrs dynamic biases or bias instabilities (m/s^2).
%   gb_corr: 1x3 gyros correlation times (seconds).
%   ab_corr: 1x3 accrs correlation times (seconds).
%    gb_psd: 1x3 gyros dynamic biases root-PSD (rad/s/root-Hz).
%    ab_psd: 1x3 accrs dynamic biases root-PSD (m/s^2/root-Hz);
%      freq: 1x1 sampling frequency (Hz).

ustrain.a_std = [0.00643187932253599  0.00661386698561032  0.00673225201283004];
ustrain.g_std = [0.00272391738310747  0.00248849782611228  0.00272332577563485];

ustrain.ab_dyn = [0.000252894096875598 0.000349683866037958 0.000323068534025731];
ustrain.gb_dyn = [7.6339404800228e-05  4.50248175403541e-05 8.75796277840371e-05];

ustrain.ab_corr = [ 40  20 100];
ustrain.gb_corr = [500 700 200];

ustrain.ab_sta = [1.73301445792617e-13 -7.93732502701179e-13 -1.84847751355576e-13];
ustrain.gb_sta = [4.00424136983284e-14 4.98197419961447e-15 -6.5696457219509e-15];

ustrain.arrw = [8.21484738626e-05 4.54275740041735e-05 0.000103299115514897]; 
ustrain.vrrw = [0.00031522133759985 0.000519606636158211 0.000396688807571295];      

%% STATIC SYNTHETIC REFERENCE DATASET

N = 6 * 60 * 60;    % 6 hours of simulation

ref.freq = 100;     % IMU frequency
dt = 1/ref.freq;    % IMU sampling period
ref.t = (0:dt:N)';  % IMU time vector    

to = (ref.t(end) - ref.t(1));
fprintf('NaveGo: dataset time span is %.2f hours or %.2f minutes or %.2f seconds. \n', (to/60/60), (to/60), to)

M = max(size(ref.t));

ref.vel = zeros(M,3);   % Velocity is zero
ref.lat = ones(M,1) * -32.8903 * D2R;
ref.lon = ones(M,1) * -68.8472 * D2R;
ref.h   = ones(M,1) * 700;

% DCMnb_m: Nx9 matrix with nav-to-body direct cosine matrices (DCM).
% Each row of DCMnb_m contains the 9 elements of a particular DCMnb_m
% matrix ordered as [a11 a21 a31 a12 a22 a32 a13 a23 a33].

ref.DCMnb_m = zeros(M,9);       % The platform is leveled
ref.DCMnb_m(:,1) = ones(M,1);
ref.DCMnb_m(:,5) = ones(M,1);
ref.DCMnb_m(:,9) = ones(M,1);

%% STATIC SYNTHETIC IMU DATASET

fprintf('NaveGo: generating IMU ACCR synthetic data... \n')

fb = acc_gen (ref, ustrain);   % Generate acc in the body frame
imu.fb = fb;

fprintf('NaveGo: generating IMU GYRO synthetic data... \n')

wb = gyro_gen (ref, ustrain);  % Generate gyro in the body frame
imu.wb = wb;

imu.t = ref.t;
imu.freq = get_freq(ref.t);

clear fb wb;

%% ALLAN VARIANCE

ustrain_allan = allan_imu (imu)

