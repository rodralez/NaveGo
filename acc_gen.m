function [fb] = acc_gen (ref, imu)
% acc_gen: generates simulated accelerometers measurements from reference
%           data and imu error profile.
% INPUT:
%		ref: data structure with true trajectory.
%		imu: data structure with IMU error profile.
%
% OUTPUT:
%		fb: Nx3 matrix with [fx, fy, fz] simulated accelerations in the
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
% Version: 002
% Date:    2015/08/20
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

% If accelerations are provided...
if (isfield(ref, 'fb'))    
    
    acc_b = ref.fb;

% Obtain acceleration from velocity    
elseif (isfield(ref, 'vel'))
    
    acc_raw = (diff(ref.vel)) ./ [diff(ref.t) diff(ref.t) diff(ref.t)];
    acc_raw = [ 0 0 0; acc_raw; ];
    acc_ned = sgolayfilt(acc_raw, 10, 45);
    acc_b = acc_nav2body(acc_ned, ref.DCMnb); 
else

%   Method: LLH > ECEF > NED  
    [~, acc_ned] = pllh2vned (ref);
    acc_b = acc_nav2body(acc_ned, ref.DCMnb); 
end

g_n = gravity(ref.lat, ref.h);       
cor_n = coriolis(ref.lat, ref.vel, ref.h); 

g_b = zeros(ref.kn,3);
cor_b = zeros(ref.kn,3);
for i = 1:ref.kn    
    dcm = reshape(ref.DCMnb(i,:), 3, 3);    
    gb = dcm * g_n(i,:)';
    corb =  dcm * cor_n(i,:)'; 
    g_b(i,:) = gb';
    cor_b(i,:) = corb';
end

% Set static bias
a = -imu.ab_fix(1);
b = imu.ab_fix(1);
ab_fix = (b-a) .* rand(3,1) + a;

% Random vectors
r1=randn(ref.kn,1);
r2=randn(ref.kn,1);
r3=randn(ref.kn,1);
r4=randn(ref.kn,1);
r5=randn(ref.kn,1);
r6=randn(ref.kn,1);
o=ones(ref.kn,1);

if (isinf(imu.ab_corr))
    sigmc = imu.ab_drift;
    ab_corr = [sigmc(1).*r4 sigmc(2).*r5 sigmc(3).*r6];
else
    % Acc correlation noise
    ab_corr = zeros(ref.kn,3);
    dt = mean(diff(ref.t));
    alpha = exp(-dt./imu.ab_corr);
    sigmc = imu.ab_drift .* sqrt(1 - alpha.^2);
    ab_corr(1,:) = ab_fix' + [sigmc(1).*r4(1) sigmc(2).*r5(1) sigmc(3).*r6(1)];
    for i = 2:ref.kn    
        ab_corr (i,:) = alpha .* ab_corr (i-1,:) + [sigmc(1).*r4(i) sigmc(2).*r5(i) sigmc(3).*r6(i)]; 
    end
end

fb = acc_b - cor_b + g_b + ...
     [imu.astd(1)  .*r1  imu.astd(2)  .*r2  imu.astd(3)  .*r3 ] + ...
     [ab_fix(1) .*o   ab_fix(2) .*o   ab_fix(3) .*o] + ...
     ab_corr; 
 
end
