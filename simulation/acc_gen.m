function [fb_sim] = acc_gen (ref, imu)
% acc_gen: generates simulated accelerometers measurements from reference
%  data and IMU error profile.
%
% INPUT
%	ref: data structure with true trajectory.
%	imu: data structure with IMU error profile.
%
% OUTPUT
%	fb_sim: Nx3 matrix with simulated accelerations in the
%		body frame [X Y Z] (m/s^2, m/s^2, m/s^2).
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
% References:
%
%	R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a
% simulation framework for low-cost integrated navigation systems,
% Journal of Control Engineering and Applied Informatics, vol. 17,
% issue 2, pp. 110-120, 2015. Sec. 2.2.
%
%   Aggarwal, P. et al. MEMS-Based Integrated Navigation. Artech
% House. 2010.
%
%   Thinking about accelerometers and gravity by Dave Redell
% http://www.lunar.org/docs/LUNARclips/v5/v5n1/Accelerometers.html
%
% Version: 012
% Date:    2022/08/22
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

N = max(size(ref.t));
M = [N, 3];

%% SIMULATION OF ACC

% If true, accelerations are provided...
if (isfield(ref, 'fb'))
    
    acc_b = ref.fb;
    
% If not, accelerations are obtained from velocity
elseif (isfield(ref, 'vel'))
    
    acc_raw = (diff(ref.vel)) ./ [diff(ref.t) diff(ref.t) diff(ref.t)];
    acc_raw = [ 0 0 0; acc_raw; ];
    
    % Noise introduced by derivatives should be smoothed
    acc_ned = my_sgolayfilt(acc_raw);
    acc_b = acc_nav2body(acc_ned, ref.DCMnb_m);
    
% If not, accelerations are obtained from position
else
    
    % Method: LLH > ECEF > NED
    [~, acc_ned] = pllh2vned (ref);
    acc_b = acc_nav2body(acc_ned, ref.DCMnb_m);
end

%% SIMULATION OF GRAVITY AND CORIOLIS

% Gravity and Coriolis in nav-ref
g_n = -gravity(ref.lat, ref.h);              % Accelerometer in Z axis senses an 
                                                % acceleration of 1.0 G straight up
cor_n = coriolis(ref.lat, ref.vel, ref.h);

% Gravity and Coriolis from nav-ref to body-ref
g_b = zeros(M);
cor_b = zeros(M);
for i = 1:N
    dcmnb = reshape(ref.DCMnb_m(i,:), 3, 3);
    gb = dcmnb * g_n(i,:)';
    corb =  dcmnb * cor_n(i,:)';
    g_b(i,:) = gb';
    cor_b(i,:) = corb';
end

%% SIMULATION OF NOISES

% -------------------------------------------------------------------------
% Simulation of static bias as a constant random variable

ab_sta = noise_b_sta (imu.ab_sta, N);

% -------------------------------------------------------------------------
% Simulation of white noise

wn = randn(M);
acc_wn = zeros(M);

for i=1:3

    acc_wn(:, i) = imu.a_std(i).* wn(:,i);
end

% -------------------------------------------------------------------------
% Simulation of dynamic bias (bias instability) as a first-order Gauss-Markov model

dt = 1/ref.freq; 
ab_dyn = noise_b_dyn (imu.ab_corr, imu.ab_dyn, dt, M);

% -------------------------------------------------------------------------
% Simulation of rate random walk

acc_rrw = noise_rrw (imu.vrrw, dt, M);

% -------------------------------------------------------------------------

fb_sim = acc_b + cor_b + g_b + acc_wn + ab_sta + ab_dyn + acc_rrw;

end
