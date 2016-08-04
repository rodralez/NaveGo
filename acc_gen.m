function fb = acc_gen (ref, imu)
% acc_gen: generates simulated accelerometers measurements from reference
%           data and imu error profile.
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
% issue 2, pp. 110-120, 2015. Sec. 2.2.
%
%           K. Jerath, Noise Modeling of Sensors: The Allan Variance
% Method. Presentation. Page 40. URL: eecs.wsu.edu/~taylorm/16_483/Jerath.pptx
%
% Version: 003
% Date:    2016/05/31
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

% If accelerations are provided...
if (isfield(ref, 'fb'))    
    
    acc_b = ref.fb;
    
    N = ref.kn;

% If not, obtain acceleration from velocity    
elseif (isfield(ref, 'vel'))
    
%     acc_raw = (diff(ref.vel)) ./ [diff(ref.t) diff(ref.t) diff(ref.t)];
    d_even = diff(ref.vel(1:2:end,:));
    t_even = diff(ref.t(1:2:end));
    acc_even = d_even ./ [t_even t_even t_even] ;
    d_odd = diff(ref.vel(2:2:end,:));
    t_odd = diff(ref.t(2:2:end));
    acc_odd = d_odd ./ [t_odd t_odd t_odd] ;
    
    acc_raw = zeros(size(ref.vel));
    acc_raw(2:2:end-1,:) = acc_even;
    acc_raw(3:2:end,:) = acc_odd;
    acc_raw = acc_raw(1:end-1,:);
    
    acc_ned = sgolayfilt(acc_raw, 15, 49);
    acc_b = acc_nav2body(acc_ned, ref.DCMnb); 
    
    N = ref.kn - 1;
% If not, obtain acceleration from positicion      
else

%   Method: LLH > ECEF > NED  
    [~, acc_ned] = pllh2vned (ref);
    acc_b = acc_nav2body(acc_ned, ref.DCMnb);
    
    N = ref.kn - 1;
end

g_n = gravity(ref.lat, ref.h);       
coriolis_n = coriolis(ref.lat, ref.vel, ref.h); 

g_b = zeros(N,3);
coriolis_b = zeros(N,3);

for i = 1:N    
    dcm = reshape(ref.DCMnb(i,:), 3, 3);    
    gb = dcm * g_n(i,:)';
    cor_b =  dcm * coriolis_n(i,:)'; 
    
    g_b(i,:) = gb';
    coriolis_b(i,:) = cor_b';
end

% Set static bias randomly from the interval [-fix fix]
a = -imu.ab_fix(1);
b_drift = imu.ab_fix(1);
ab_fix = (b_drift-a).*rand(3,1) + a;

% Random vectors
r1=randn(N,1);
r2=randn(N,1);
r3=randn(N,1);
r4=randn(N,1);
r5=randn(N,1);
r6=randn(N,1);

o=ones(N,1);

if (isinf(imu.acorr))
    sigmc = imu.ab_drift;
    b_drift = [sigmc(1).*r4 sigmc(2).*r5 sigmc(3).*r6];
else
    % Acc correlation noise
    b_drift = zeros(N,3);
    dt=mean(diff(imu.t));
    alpha = exp(-dt./imu.acorr);
    sigmc = imu.ab_drift .* sqrt(1 - alpha.^2);
    b_drift(1,:) = ab_fix' + [sigmc(1).*r4(1) sigmc(2).*r5(1) sigmc(3).*r6(1)];
    
    for i=2:N   
        b_drift (i,:) = alpha .* b_drift (i-1,:) + [sigmc(1).*r4(i) sigmc(2).*r5(i) sigmc(3).*r6(i)]; 
    end
end

% Simulate bias instability (Jerath)
% if (isinf(imu.acorr))
%     
%     sigmc = imu.ab_drift;
%     b_drift = [sigmc(1).*r4 sigmc(2).*r5 sigmc(3).*r6];
% else
%     TC = imu.acorr;
%     B = imu.ab_drift;
%     dt = mean(diff(imu.t));
%     
%     b_drift = zeros(N,3);
%     b_drift(1,:) = zeros(1,3);
%     
%     for i = 2:1:N
%         
%         bdot = (-dt./TC) * b_drift(i-1) + B .* randn(1,3);
%         b_drift(i,:) = b_drift(i-1,:) + bdot;
%     end
% end

fb = acc_b - coriolis_b + (g_b) + ...
     [imu.astd(1).*r1  imu.astd(2).*r2  imu.astd(3).*r3 ] + ...
     [ab_fix(1).*o   ab_fix(2).*o   ab_fix(3).*o] + ...
     b_drift; 
 
end
