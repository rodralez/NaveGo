function [fb_sim] = acc_gen (ref, imu)
% acc_gen: generates simulated accelerometers measurements from reference
%           data and imu error profile.
% INPUT:
%		ref: data structure with true trajectory.
%		imu: data structure with IMU error profile.
%
% OUTPUT:
%		fb_sim: Nx3 matrix with [fx, fy, fz] simulated accelerations in the
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
% issue 2, pp. 110-120, 2015. Sec. 2.2.
%
%           Aggarwal, P. et al. MEMS-Based Integrated Navigation. Artech
% House. 2010.
%
% Version: 004
% Date:    2017/07/27
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

N = max(size(ref.t));
M = [N, 3];

%% SIMULATE ACC

% If true accelerations are provided...
if (isfield(ref, 'fb'))
    
    acc_b = ref.fb;
    
% If not, obtain acceleration from velocity
elseif (isfield(ref, 'vel'))
    
    acc_raw = (diff(ref.vel)) ./ [diff(ref.t) diff(ref.t) diff(ref.t)];
    acc_raw = [ 0 0 0; acc_raw; ];
    % Noise introduced by differentation should be smoothed.
    acc_ned = sgolayfilt(acc_raw, 10, 45);  
    acc_b = acc_nav2body(acc_ned, ref.DCMnb);
    
% If not, obtain acceleration from position
else
    
    % Method: LLH > ECEF > NED
    [~, acc_ned] = pllh2vned (ref);
    acc_b = acc_nav2body(acc_ned, ref.DCMnb);
end

%% SIMULATE GRAVITY AND CORIOLIS

% Gravity and Coriolis in nav-ref
grav_n = gravity(ref.lat, ref.h);
cor_n = coriolis(ref.lat, ref.vel, ref.h);

% Gravity and Coriolis from nav-ref to body-ref
grav_b = zeros(M);
cor_b = zeros(M);
for i = 1:N
    dcm = reshape(ref.DCMnb(i,:), 3, 3);
    gb = dcm * grav_n(i,:)';
    corb =  dcm * cor_n(i,:)';
    grav_b(i,:) = gb';
    cor_b(i,:) = corb';
end

%% SIMULATE NOISES

% -------------------------------------------------------------------------
% Simulate static bias
[a_sbias] = noise_sbias (imu.ab_fix, N);

% -------------------------------------------------------------------------
% Simulate white noise
wn = randn(M);
a_wn = zeros(M);

for i=1:3

    a_wn(:, i) = imu.ab_std(i).* wn(:,i);
end

% -------------------------------------------------------------------------
% Simulate dynamic bias (bias instability) as a First-order Gauss-Markov model

dt = 1/imu.freq; 
[a_dbias] = noise_dbias (imu.ab_corr, imu.ab_drift, dt, M);

% -------------------------------------------------------------------------
% Simulate rate random walk

[a_rrw] = noise_rrw (imu.vrrw, dt, M);

% sigma_aK = ustrain.fb_allan(idx) ; %.* sqrt(3/TAU)

% for i=1:3
%     
%     b_noise = randn(N-1,1);
%     
%     for j=2:N
%         arrw (j, i) = arrw(j-1, i) + imu.arrw(i) * dt .* b_noise(j-1);
%     end
% end

% -------------------------------------------------------------------------

fb_sim = acc_b - cor_b + grav_b + a_wn + a_sbias + a_dbias + a_rrw;

end
