function wb_sim = gyro_gen (ref, imu)
% gyro_gen: generates simulated gyros measurements from reference data and
%          IMU error profile.
%
% INPUT
%	ref, data structure with true trajectory.
%	imu, data structure with IMU error profile.
%
% OUTPUT
%	wb_sim, Nx3 matrix with simulated gyros in the body frame [X Y Z] 
%     (rad, rad, rad).
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
%  	R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a
% simulation framework for low-cost integrated navigation systems,
% Journal of Control Engineering and Applied Informatics, vol. 17,
% issue 2, pp. 110-120, 2015. Sec. 2.1.
%
%   Aggarwal, P. et al. MEMS-Based Integrated Navigation. Artech
% House. 2010.
%
% Version: 010
% Date:    2022/08/22
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

N = max(size(ref.t));
M = [N, 3];

%% SIMULATION OF TURN RATES

% If true, turn rates are provided...
if (isfield(ref, 'wb'))
    
    gyro_b = ref.wb;
    
% If not, turn rates are obtained from DCM...
else
    gyro_raw = gyro_gen_delta(ref.DCMnb_m, ref.t);
    gyro_raw = [0 0 0; gyro_raw;];
    
    % Noise introduced by derivatives at gyro_gen_delta() should be smoothed
    gyro_b = my_sgolayfilt(gyro_raw);
end

%% SIMULATION OF TRANSPORTE AND EARTH RATES

om_b = zeros(M);
for i = 1:N
    
    dcmnb = reshape(ref.DCMnb_m(i,:), 3, 3);
    omega_ie_n = earth_rate(ref.lat(i));
    omega_en_n = transport_rate(ref.lat(i), ref.vel(i,1), ref.vel(i,2), ref.h(i));
    om_ie_n = skewm_inv(omega_ie_n);
    om_en_n = skewm_inv(omega_en_n);
    om_b(i,:)  = (dcmnb * (om_ie_n + om_en_n))';
end

%% SIMULATION OF NOISES

% -------------------------------------------------------------------------
% Simulation of static bias as a constant random variable

gb_sta = noise_b_sta (imu.gb_sta, N);

% -------------------------------------------------------------------------
% Simulation of white noise

wn = randn(M);
gyro_wn = zeros(M);

for i=1:3

    gyro_wn(:, i) = imu.g_std(i) .*  wn(:,i);
end

% -------------------------------------------------------------------------
% Simulation of dynamic bias (bias instability) as a first-order Gauss-Markov model

dt = 1/ref.freq; 
gb_dyn = noise_b_dyn (imu.gb_corr, imu.gb_dyn, dt, M);

% -------------------------------------------------------------------------
% Simulation of rate random walk

gyro_rrw = noise_rrw (imu.arrw, dt, M);

% -------------------------------------------------------------------------

wb_sim = gyro_b + om_b + gyro_wn + gb_sta + gb_dyn + gyro_rrw;

end
