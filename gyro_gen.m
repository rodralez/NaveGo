function wb_sim = gyro_gen (ref, imu)
% gyro_gen: generates simulated gyros measurements from reference data and
%          imu error profile.
%
% INPUT:
%		ref: data structure with true trajectory.
%		imu: data structure with IMU error profile.
%
% OUTPUT:
%		wb_sim: Nx3 matrix with [wx, wy, wz] simulated gryos in the
%		body frame.
%
%   Copyright (C) 2014, Rodrigo González, all rights reserved.
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
% issue 2, pp. 110-120, 2015. Sec. 2.1.
%
%           Aggarwal, P. et al. MEMS-Based Integrated Navigation. Artech
% House. 2010.
%
% Version: 004
% Date:    2017/07/28
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

N = max(size(ref.t));
M = [N, 3];

%% SIMULATE GYRO

% If true turn rates are provided...
if (isfield(ref, 'wb'))
    
    gyro_b = ref.wb;
    
% If not, obtain turn rates from DCM
else
    gyro_raw = gyro_gen_delta(ref.DCMnb, diff(ref.t));
    gyro_raw = [gyro_raw; 0 0 0;];
    gyro_b  = sgolayfilt(gyro_raw, 10, 45);
end

%% SIMULATE TRANSPORTE AND EARTH RATES

g_err_b = zeros(M);
for i = 1:N,
    
    dcmnb = reshape(ref.DCMnb(i,:), 3, 3);
    omega_ie_n = earthrate(ref.lat(i));
    omega_en_n = transportrate(ref.lat(i), ref.vel(i,1), ref.vel(i,2), ref.h(i));
    omega_in_b = dcmnb * (omega_en_n + omega_ie_n );
    g_err_b(i,:) = ( omega_in_b )';
end

%% SIMULATE NOISES

% -------------------------------------------------------------------------
% Simulate static bias
[g_sbias] = noise_sbias (imu.gb_fix, N);

% -------------------------------------------------------------------------
% Simulate white noise
wn = randn(M);
g_wn = zeros(M);

for i=1:3

    g_wn(:, i) = imu.gb_std(i).* wn(:,i);
end

% -------------------------------------------------------------------------
% Simulate dynamic bias (bias instability) as a First-order Gauss-Markov model

dt = 1/imu.freq; 
[g_dbias] = noise_dbias (imu.gb_corr, imu.gb_drift, dt, M);

% -------------------------------------------------------------------------
% Simulate rate random walk

[g_rrw] = noise_rrw (imu.arrw, dt, M);

% -------------------------------------------------------------------------

wb_sim = gyro_b + g_err_b + g_wn + g_sbias + g_dbias + g_rrw;

end
