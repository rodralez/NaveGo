function imu_si = imu_si_errors(imu, dt)
% imu_si_errors: converts IMU errors manufacturer units to SI units.
%
% INPUT
%	imu, data structure with IMU error profile in manufacturer units.
%       imu.arw:      angle random walks [X Y Z] (deg/root-hour).
%       imu.arrw:     angle rate random walks [X Y Z] (deg/root-hour/s).
%       imu.vrw:      velocity random walks [X Y Z] (m/s/root-hour).
%		imu.vrrw:     velocity rate random walks [X Y Z] (deg/root-hour/s).
%       imu.gb_sta:   gyro static biases [X Y Z] (deg/s).
%       imu.ab_sta:   acc static biases [X Y Z] (mg).
%       imu.gb_dyn:   gyro dynamic biases [X Y Z] (deg/s).
%       imu.ab_dyn:   acc dynamic biases [X Y Z] (mg).
%       imu.gb_corr:  gyro correlation times [X Y Z] (seconds).
%       imu.ab_corr:  acc correlation times [X Y Z] (seconds).
%       imu.m_psd:    magnetometer noise density [X Y Z] (mgauss/root-Hz).
%		dt:           IMU sampling interval (seconds).
%
% OUTPUT
%	imu_si, data structure with IMU error profile in SI units.
%       imu_si.arw:      angle random walks [X Y Z] (rad/s/root-Hz).
%       imu_si.arrw:     angle rate random walks [X Y Z] (rad/s^2/root-Hz).
%       imu_si.vrw:      velocity random walks [X Y Z] (m/s^2/root-Hz).
%		imu_si.vrrw:     velocity rate random walks [X Y Z]
%		(m/s^3/root-Hz).
%       imu_si.gb_sta:   gyro static biases [X Y Z] (rad/s).
%       imu_si.ab_sta:   acc static biases [X Y Z] (m/s^2).
%       imu_si.gb_dyn:   gyro dynamic biases [X Y Z] (rad/s).
%       imu_si.ab_dyn:   acc dynamic biases [X Y Z] (m/s^2).
%       imu_si.gb_corr:  gyro correlation times [X Y Z] (seconds).
%       imu_si.ab_corr:  acc correlation times [X Y Z] (seconds)
%       imu_si.gb_psd:   gyro dynamic bias PSD [X Y Z] (rad/s/root-Hz).
%       imu_si.ab_psd:   acc dynamic bias PSD [X Y Z] (m/s^2/root-Hz).
%       imu_si.m_std:    magnetometer noise [X Y Z] (tesla). 
%		dt:              IMU sampling interval (seconds).
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
%
%	R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a 
% simulation framework for low-cost integrated navigation systems, 
% Journal of Control Engineering and Applied Informatics, vol. 17, 
% issue 2, pp. 110-120, 2015. Eq. 9, 14, and 30.
%
% Version: 010
% Date:    2021/04/27
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

% Constanst
G =  9.80665;       % Gravity constant, m/s^2
D2R = (pi/180);     % Degrees to radians
% R2D = (180/pi);     % Radians to degrees
G2T = 1E-4;         % Gauss to Tesla

% Copy previous fields
imu_si = imu;

% PSD noise
imu_si.arw = (imu.arw ./ 60) .* D2R;    % deg/root-hour -> rad/s/root-Hz
imu_si.vrw = (imu.vrw ./ 60);           % m/s/root-hour -> m/s^2/root-Hz

% Standard deviation
imu_si.a_std   = imu_si.vrw ./ sqrt(dt); % m/s^2/root-Hz  ->  m/s^2
imu_si.g_std   = imu_si.arw ./ sqrt(dt); % rad/s/root-Hz  ->  rad/s

% Static bias
imu_si.ab_sta = imu.ab_sta .* 0.001 * G;    % mg -> m/s^2
imu_si.gb_sta = imu.gb_sta .* D2R;          % deg/s -> rad/s;

% Dynamic bias
imu_si.ab_dyn = imu.ab_dyn .* 0.001 .* G;   % mg -> m/s^2
imu_si.gb_dyn = imu.gb_dyn .* D2R;          % deg/s -> rad/s;

% Dynamic bias PSD
if (isinf(imu.gb_corr))
    imu_si.gb_psd = imu_si.gb_dyn;          % rad/s (approximation)
else
    imu_si.gb_psd = imu_si.gb_dyn ./ sqrt(imu.gb_corr);  % rad/s/root-Hz
end

if (isinf(imu.ab_corr))
    imu_si.ab_psd = imu_si.ab_dyn;          % m/s^2 (approximation)
else
    imu_si.ab_psd = imu_si.ab_dyn ./ sqrt(imu.ab_corr);  % m/s^2/root-Hz
end

% Correlation time
imu_si.ab_corr = imu.ab_corr;
imu_si.gb_corr = imu.gb_corr;

% MAG
if isfield(imu , {'m_psd'})
    imu_si.m_std = (imu.m_psd .* 1e-3) ./ sqrt(dt) .* G2T; % mGauss/root-Hz -> Tesla
end

end
