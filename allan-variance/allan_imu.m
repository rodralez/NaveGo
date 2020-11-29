function imu = allan_imu (imu_sta, verbose)
% allan_imu: performs Allan variance analysis on inertial measurements
% coming from an IMU in order to characterize several types of IMU errors.
%
% INPUT
%   imu_sta: input data structure must contains the following fields
%   fb: Nx3 accelerations [X Y Z] (m/s^2).
%   wb: Nx3 turn rates [X Y Z] (rad/s).
%   t: Nx1 time vector (s).
%
%   verbose: verbose level for allan_overlap function.
%      0 = silent & no data plots; 1 = status messages; 2 = all messages    
%
% OUTPUT
%   imu: input data structure with the following addtional fields
%
%   arw: 1x3 angle random walk (rad/root-s). Value is taken 
%       straightfoward from the plot at t = 1 s.
%       Note: units of rad/s from the plot have to be transformed to 
%       rad/root-s. This is done by multiplying (rad/s * root-s/root-s) = 
%       (rad/s * root-s/1) = rad/root-s, since root-s = 1 for tau = 1, time 
%       at which random walk is evaluated.     
%
%	vrw: 1x3 velocity random walk (m/s/root-s). Value is taken 
%       straightfoward from the plot at t = 1 s.
%       Note: units of m/s^2 from the plot have to be transformed to 
%       m/s/root-s. This is done by multiplying (m/s^2 * root-s/root-s) = 
%       (m/s^2 * root-s/1) = m/s/root-s, since root-s = 1 for tau = 1, time 
%       at which random walk is evaluated.
%
%	gb_dyn: 1x3 gyros bias instability in rad/s. Value is taken
%       from the plot at the minimun value.
%
%   ab_dyn: 1x3 accrs bias instability in m/s^2. Value is taken
%       from the plot at the minimun value.
%
%   gb_corr: 1x3 gyros correlation times (s).
%   ab_corr: 1x3 accrs correlation times (s).
%
%   g_std: 1x3 gyros standard deviations (rad/s).
%   a_std: 1x3 accrs standard deviations (m/s^2).
%
%   g_max: 1x3 gyros maximum values (rad/s).
%   a_max: 1x3 accrs maximum values (m/s^2).
%
%   g_min: 1x3 gyros minimum values (rad/s).
%   a_min: 1x3 accrs maximum values (m/s^2).
%
%   g_mean: 1x3 gyros mean values (rad/s).
%   a_meam: 1x3 accrs mean values (m/s^2).
%
%   g_median: 1x3 gyros median values (rad/s).
%   a_median: 1x3 accrs median values (m/s^2).
%
%   fb_tau:   Mx3 time vector from AV for accelerometers [X Y Z].
%   fb_allan: Mx3 AV vector for accelerometers [X Y Z].
%   fb_error: Mx3 AV errors for accelerometers [X Y Z].
%
%   wb_tau:   Mx3 time vector from AV for gyros [X Y Z].
%   wb_allan: Mx3 AV vector for gyros [X Y Z].
%   wb_error: Mx3 AV errors for gyros [X Y Z].
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
%   IEEE-SA Standards Board. IEEE Standard Specification Format
% Guide and Test Procedure for Single-Axis Interferometric Fiber Optic
% Gyros. ISBN 1-55937-961-8. September 1997.
%
%   Naser El-Sheimy et at. Analysis and Modeling of Inertial Sensors
% Using Allan Variance. IEEE TRANSACTIONS ON INSTRUMENTATION AND
% MEASUREMENT, VOL. 57, NO. 1, JANUARY 2008.
%
%   Oliver J. Woodman. An introduction to inertial navigation. Technical
% Report. ISSN 1476-2986. University of Cambridge, Computer Laboratory.
% August 2007.
%
%   M.A. Hopcroft. Allan overlap MATLAB function v2.24.
% https://www.mathworks.com/matlabcentral/fileexchange/13246-allan
%
% Version: 007
% Date:    2019/02/18
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% Verbose for allan_overlap function
if (nargin < 2), verbose = 2; end

%% PREALLOCATION

% Random walk
imu.vrw = zeros(1,3);
imu.arw = zeros(1,3);

% Bias instability
imu.ab_dyn = zeros(1,3);
imu.gb_dyn = zeros(1,3);

% Bias instability correlation time
imu.ab_corr = zeros(1,3);
imu.gb_corr = zeros(1,3);

% Static bias 
imu.ab_sta = zeros(1,3);
imu.gb_sta = zeros(1,3);

% Statistics
imu.a_std    = zeros(1,3);
imu.a_max    = zeros(1,3);
imu.a_min    = zeros(1,3);
imu.a_mean   = zeros(1,3);
imu.a_median = zeros(1,3);    
imu.a_outliers = zeros(1,3);    
imu.a_linear = zeros(3,2);  

imu.g_std    = zeros(1,3);
imu.g_max    = zeros(1,3);
imu.g_min    = zeros(1,3);
imu.g_mean   = zeros(1,3);
imu.g_median = zeros(1,3);   
imu.g_outliers = zeros(1,3); 
imu.g_linear = zeros(3,2);  

%% TIME VECTOR FOR ALLAN VARIANCE

% Find the sampling time and data frequency
dt = median(diff(imu_sta.t));

% Frequency must be rounded to an integer number for allan_overlap function
real_freq = ceil(1/dt);
real_freq_frac = round(real_freq / 10);
data.rate = real_freq_frac * 10;

% From allan_overlap:
%   For rate-based data, ADEV is computed only for tau values greater than the
%   minimum time between samples and less than the half of total time.
T = (imu_sta.t(end) - imu_sta.t(1)) ;
exp_min = floor( log10( dt ) );
exp_max = floor( log10( T /2 ) );

TAU = 10.^(exp_min:exp_max);

tau_v = [];
for i = 1:length(TAU)-1
    
    tau_v = [tau_v TAU(i):TAU(i):TAU(i+1) ];
end

% Delete repeated elements
dd = diff (tau_v);
idl = dd ~= 0;
idl = [idl true];
tau_v = tau_v(idl);

fprintf('allan_imu: length of time is %02.3d hours or %.2f minutes or %.2f seconds. \n\n', (T/60/60), (T/60), T)

%% ACCELEROMETERS

for i=1:3
    
    fprintf('\nallan_imu: Allan variance for FB %d \n', i)   
    
    data.freq = imu_sta.fb(:,i);
    
    [allan_o, s, error, tau] = allan_overlap(data, tau_v ,'allan_overlap', verbose);
    
    imu.fb_tau  (:,i) = tau';
    imu.fb_allan(:,i) = allan_o';
    imu.fb_error(:,i) = error';
    
    vrw = allan_get_rw (tau, allan_o, dt);
    imu.vrw(i) = vrw;
    
    [b_dyn, t_corr] = allan_get_b_dyn (tau, allan_o);
    imu.ab_dyn(i) = b_dyn;
    imu.ab_corr(i)  = t_corr;
    
    imu.ab_sta(i)    = s.mean; 
        
    imu.a_std(i)    = s.std;
    imu.a_mean(i)   = s.mean;    
    imu.a_max(i)    = s.max;
    imu.a_min(i)    = s.min;
    imu.a_median(i) = s.median;
    imu.a_outliers(i) = s.outliers;
    imu.a_linear(i,:) = s.linear;
end

% Plot ACCRS
figure
loglog(imu.fb_tau, imu.fb_allan, '-o');    
grid on
title('ACCRS ALLAN VARIANCES')
legend('ACC X','ACC Y', 'ACC Z' )

%% GYROSCOPES

for i=1:3
    
    fprintf('\nallan_imu: Allan variance for WB %d \n', i)
    
    data.freq = imu_sta.wb(:,i);
    
    [allan_o, s, error, tau] = allan_overlap(data, tau_v ,'allan_overlap', verbose);
    
    imu.wb_tau  (:,i) = tau;
    imu.wb_allan(:,i) = allan_o;
    imu.wb_error(:,i) = error;
    
    arw = allan_get_rw (tau, allan_o, dt);
    imu.arw(i) = arw;
    
    [b_dyn, t_corr] = allan_get_b_dyn (tau, allan_o);
    imu.gb_dyn(i) = b_dyn;
    imu.gb_corr(i) = t_corr;
    
    imu.gb_sta(i) = s.mean;
    
    imu.g_std(i)    = s.std;
    imu.g_mean(i)   = s.mean;     
    imu.g_max(i)    = s.max;
    imu.g_min(i)    = s.min;
    imu.g_median(i) = s.median;
    imu.g_outliers(i) = s.outliers;
    imu.g_linear(i,:) = s.linear;
end

imu.freq = data.rate;

% Plot GYROS
figure
loglog(imu.wb_tau, imu.wb_allan, '-o');
grid on
title('GYROS ALLAN VARIANCES')
legend('GYRO X','GYRO Y', 'GYRO Z' )

end
