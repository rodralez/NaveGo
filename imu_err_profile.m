function imu_nav = imu_err_profile(imu, dt)
% imu_err_profile: converts IMU error manufacturer units to SI units.
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
% Reference: 
%			R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a 
% simulation framework for low-cost integrated navigation systems, 
% Journal of Control Engineering and Applied Informatics, vol. 17, 
% issue 2, pp. 110-120, 2015. Eq. 9, 14, and 30.
%
% Version: 001
% Date:    2014/09/11
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

d2r = (pi/180);     % deg to rad
g2mss = 9.81;       % g to m/s^2

imu_nav.arw = (imu.arw ./ 60) .* d2r;   % deg/root-hour -> rad/s/root-Hz
imu_nav.vrw = (imu.vrw ./ 60);          % m/s/root-hour -> m/s^2/root-Hz

% Sigmas
imu_nav.astd   = imu_nav.vrw ./ sqrt(dt); % m/s^2/root-Hz  ->  m/s^2
imu_nav.gstd   = imu_nav.arw ./ sqrt(dt); % rad/s/root-Hz  ->  rad/s

% Static bias
imu_nav.ab_fix = imu.ab_fix .* 0.001 * g2mss;    % mg -> m/s^2
imu_nav.gb_fix = imu.gb_fix .* d2r;              % deg/s -> rad/s;

% Dynamic bias
imu_nav.ab_drift = imu.ab_drift .* 0.001 .* g2mss;  % mg -> m/s^2
imu_nav.gb_drift = imu.gb_drift .* d2r;             % deg/s -> rad/s;

% Dynamic bias PSD
if (isinf(imu.gcorr))
    imu_nav.gb_n = imu_nav.gb_drift;  % rad/s/root-Hz;
else
    imu_nav.gb_n = imu_nav.gb_drift .* sqrt(imu.gcorr);  % rad/s/root-Hz; 
end

if (isinf(imu.acorr))
    imu_nav.ab_n = imu_nav.ab_drift;  % m/s^2/root-Hz
else
    imu_nav.ab_n = imu_nav.ab_drift .* sqrt(imu.acorr);  % m/s^2/root-Hz
end

% Correlation time
imu_nav.acorr = imu.acorr;
imu_nav.gcorr = imu.gcorr;

% MAG
%imu_nav.mstd = (imu.m_psd .* 1e-3) ./ sqrt(dt) .* 1e-4; % mgauss/root-Hz -> tesla

end
