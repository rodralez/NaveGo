function imu_si = imu_si_errors(imu, dt)
% imu_err_profile: converts IMU errors manufacturer units to SI units.
%
% INPUT:
%		imu, data structure with IMU error profile in manufacturer units.
%         imu.arw:      angle random walks [X Y Z] (deg/root-hour)
%         imu.arrw:     angle rate random walks [X Y Z] (deg/root-hour/s)
%         imu.vrw:      velocity random walks [X Y Z] (m/s/root-hour)
%		  imu.vrrw:     velocity rate random walks [X Y Z] (deg/root-hour/s)
%         imu.gb_fix:   gyro static biases [X Y Z] (deg/s)
%         imu.ab_fix:   acc static biases [X Y Z] (mg)
%         imu.gb_drift: gyro dynamic biases [X Y Z] (deg/s)
%         imu.ab_drift: acc dynamic biases [X Y Z] (mg)
%         imu.gb_corr:  gyro correlation times [X Y Z] (seconds)
%         imu.ab_corr:  acc correlation times [X Y Z] (seconds)
%         imu.m_psd:    magnetometer noise density [X Y Z] (mgauss/root-Hz)
%		dt:  IMU sample time.
%
% OUTPUT:
%		imu_si: data structure with IMU error profile in SI units.
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
% Version: 005
% Date:    2017/11/01
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

D2R = (pi/180);     % deg to rad
G2MSS = 9.81;       % g to m/s^2

% Copy previois fields
imu_si = imu;

% Noise PSD
imu_si.arw = (imu.arw ./ 60) .* D2R;   % deg/root-hour -> rad/s/root-Hz
imu_si.vrw = (imu.vrw ./ 60);          % m/s/root-hour -> m/s^2/root-Hz

% Standard deviation
imu_si.ab_std   = imu_si.vrw ./ sqrt(dt); % m/s^2/root-Hz  ->  m/s^2
imu_si.gb_std   = imu_si.arw ./ sqrt(dt); % rad/s/root-Hz  ->  rad/s

% Static bias
imu_si.ab_fix = imu.ab_fix .* 0.001 * G2MSS;    % mg -> m/s^2
imu_si.gb_fix = imu.gb_fix .* D2R;              % deg/s -> rad/s;

% Dynamic bias
imu_si.ab_drift = imu.ab_drift .* 0.001 .* G2MSS;  % mg -> m/s^2
imu_si.gb_drift = imu.gb_drift .* D2R;             % deg/s -> rad/s;

% Dynamic bias PSD
if (isinf(imu.gb_corr))
    imu_si.gb_psd = imu_si.gb_drift;  % rad/s (approximation)
else
    imu_si.gb_psd = imu_si.gb_drift .* sqrt(imu.gb_corr);  % rad/s/root-Hz; 
end

if (isinf(imu.ab_corr))
    imu_si.ab_psd = imu_si.ab_drift;  % m/s^2 (approximation)
else
    imu_si.ab_psd = imu_si.ab_drift .* sqrt(imu.ab_corr);  % m/s^2/root-Hz
end

% Correlation time
imu_si.ab_corr = imu.ab_corr;
imu_si.gb_corr = imu.gb_corr;

% MAG
%imu_nav.mstd = (imu.m_psd .* 1e-3) ./ sqrt(dt) .* 1e-4; % mgauss/root-Hz -> tesla

end
