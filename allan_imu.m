function [imu] = allan_imu (imu)
% allan_imu: performs Allan variance analysis of inertial measurements
% coming from an IMU in order to characterize several IMU errors.
%
% -------------------------------------------------------------------------
%
% INPUT
% - imu. Input data structure must contains the following fields:
%
%     fb, Nx3 matrix, accelerations [X Y Z] (m/s^2).
%     wb, Nx3 matrix, turn rates [X Y Z] (rad/s).
%     t,  Nx1, time vector (s).
%
% OUTPUT
% - imu. Input data structure is added with the following new fields:
%
%     arw, 1x3 vector, angle random walk (rad/root-s). Value is taken 
%       straightfoward from the plot at t = 1 s.
%       Note: units of rad/s from the plot have to be transformed to 
%       rad/root-s. This is done by multiplying (rad/s * root-s/root-s) = 
%       (rad/s * root-s/1) = rad/root-s, since root-s = 1 for tau = 1, time 
%       at which random walk is evaluated.
%     
%
%     vrw, 1x3 vector, velocity random walk (m/s/root-s). Value is taken 
%       straightfoward from the plot at t = 1 s.
%       Note: units of m/s^2 from the plot have to be transformed to 
%       m/s/root-s. This is done by multiplying (m/s^2 * root-s/root-s) = 
%       (m/s^2 * root-s/1) = m/s/root-s, since root-s = 1 for tau = 1, time 
%       at which random walk is evaluated.
%
%     gb_drift, 1x3 vector, gyros bias instability in rad/s. Value is taken
%     from the plot at the minimun value.
%
%     ab_drift, 1x3 vector, accs bias instability in m/s^2. Value is taken
%     from the plot at the minimun value.
%
%     gb_corr, 1x3 vector, gyros correlation times (s).
%
%     ab_corr, 1x3 vector, accs correlation times (s).
%
%     g_std, 1x3 vector, gyros standard deviation (rad/s).
%
%     a_std, 1x3 vector, accs standard deviation (m/s^2).
%
%     fb_tau, Mx3 with time vector from AV for accelerometers [X Y Z],
%     respectively.
%
%     fb_allan, Mx3 with AV vector for accelerometers [X Y Z],
%     respectively.
%
%     fb_error, Mx3 with AV errors for accelerometers [X Y Z],
%     respectively.
%
%     wb_tau, Mx3 with time vector from AV for gyros [X Y Z],
%     respectively.
%
%     wb_allan, Mx3 with AV vector for gyros [X Y Z],
%     respectively.
%
%     wb_error, Mx3 with AV errors for gyros [X Y Z],
%     respectively.
%
% -------------------------------------------------------------------------
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
% -------------------------------------------------------------------------
%
% References:
%
%       IEEE-SA Standards Board. IEEE Standard Specification Format
%       Guide and Test Procedure for Single-Axis Interferometric Fiber Optic
%       Gyros. ISBN 1-55937-961-8. September 1997.
%
%       Naser El-Sheimy et at. Analysis and Modeling of Inertial Sensors
%       Using Allan Variance. IEEE TRANSACTIONS ON INSTRUMENTATION AND
%       MEASUREMENT, VOL. 57, NO. 1, JANUARY 2008.
%
%       Oliver J. Woodman. An introduction to inertial navigation. Technical
%       Report. ISSN 1476-2986. University of Cambridge, Computer Laboratory.
%       August 2007.
%
%		M.A. Hopcroft. Allan overlap MATLAB function v2.24.
%       https://www.mathworks.com/matlabcentral/fileexchange/13246-allan
%
% -------------------------------------------------------------------------
%
% Version: 003
% Date:    2016/12/01
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego
%
% -------------------------------------------------------------------------

% Verbose for allan_overlap
verbose = 2;

if (isfield(imu, 'fb_tau') )
   
fields = {'fb_tau','fb_allan','fb_error','wb_tau','wb_allan','wb_error'};  
            
    imu = rmfield(imu, fields);
end

%% PREALLOCATE

% Random walk
imu.arw = zeros(1,3);
imu.vrw = zeros(1,3);

% Bias instability
imu.ab_drift = zeros(1,3);
imu.gb_drift = zeros(1,3);

% Bias instability correlation time
imu.ab_corr = zeros(1,3);
imu.gb_corr = zeros(1,3);

% Standard deviation
imu.astd = zeros(1,3);
imu.gstd = zeros(1,3);

% Bias 
imu.ab_fix = zeros(1,3);
imu.gb_fix = zeros(1,3);

%% TIME VECTOR FOR ALLAN VARIANCE

% Find time period and data frequency
dt = mean(diff(imu.t));
data.rate = round(1/dt);

% From allan_overlap:
%   For rate-based data, ADEV is computed only for tau values greater than the
%   minimum time between samples and less than the half of total time.
T = (imu.t(end) - imu.t(1)) ;
exp_min = floor( log10(dt) );
exp_max = ceil( log10( T ) /2 );

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

plot_line = [ '-ob'; '-og'; '-or' ];

fprintf('allan_imu: processing %.3d hours of data \n', T/60/60)

%% ACCELEROMETERS

for i=1:3
    
    fprintf('allan_imu: Allan variance for FB %d \n', i)   
    
    data.freq = imu.fb(:,i);
    
    [allan_o, ~, error, tau] = allan_overlap(data, tau_v ,'allan_overlap', verbose);
    
    imu.fb_tau  (:,i) = tau';
    imu.fb_allan(:,i) = allan_o';
    imu.fb_error(:,i) = error';
    
    vrw = allan_get_rw (tau, allan_o, dt);
    imu.vrw(i) = vrw;
    
    [b_drift, t_corr] = allan_get_bdrift (tau, allan_o);
    imu.ab_drift(i) = b_drift;
    imu.ab_corr(i) = t_corr;
    
    imu.astd(i) = std(data.freq);
    imu.ab_fix(i) = mean(data.freq); 
end

% Plot
figure;

for i=1:3
    loglog(imu.fb_tau(:,i), imu.fb_allan(:,i), plot_line(i,:));
    
    hold on
end

hold off
grid on
title('ACCRS ALLAN VARIANCES')
legend('ACC X','ACC Y', 'ACC Z' )

%% GYROSCOPES

for i=1:3
    
    fprintf('allan_imu: Allan variance for WB %d \n', i)
    
    data.freq = imu.wb(:,i);
    
    % AV Method 1. BEST METHOD
    [allan_o, ~, error, tau] = allan_overlap(data, tau_v ,'allan_overlap', verbose);
    
    imu.wb_tau  (:,i) = tau;
    imu.wb_allan(:,i) = allan_o;
    imu.wb_error(:,i) = error;
    
    arw = allan_get_rw (tau, allan_o, dt);
    imu.arw(i) = arw;
    
    [b_drift, t_corr] = allan_get_bdrift (tau, allan_o);
    imu.gb_drift(i) = b_drift;
    imu.gb_corr(i) = t_corr;
    
    imu.gstd(i) = std(data.freq); 
    imu.gb_fix(i) = mean(data.freq); 
end

% Plot
figure;

for i=1:3
    loglog(imu.wb_tau(:,i), imu.wb_allan(:,i), plot_line(i,:));
    hold on
end

hold off
grid on
title('GYROS ALLAN VARIANCES')
legend('GYRO X','GYRO Y', 'GYRO Z' )

end
