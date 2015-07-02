function [fb] = acc_gen (ref, imu)
% acc_gen: generates gyros measurements from reference data and imu
%          error profile.
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
% Version: 001
% Date:    2014/09/11
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

if (isfield(ref, 'acc'))    
    
    acc_ned = ref.acc;
    
elseif (isfield(ref, 'vel'))
    
    acc_raw = (diff(ref.vel)) ./ [diff(ref.t) diff(ref.t) diff(ref.t)];
    acc_raw = [ 0 0 0; acc_raw; ];
    acc_ned = sgolayfilt(acc_raw, 15, 299);
else

%   Method: LLH > ECEF > NED  
    [~, acc_ned] = pllh2vned (ref); 
end

acc_b = acc_nav2body(acc_ned, ref.DCMnb); 

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
ab_fix = (b-a).*rand(3,1) + a;

% Random vectors
r1=randn(ref.kn,1);
r2=randn(ref.kn,1);
r3=randn(ref.kn,1);
r4=randn(ref.kn,1);
r5=randn(ref.kn,1);
r6=randn(ref.kn,1);
o=ones(ref.kn,1);

if (isinf(imu.acorr))
    sigmc = imu.ab_drift;
    acorr = [sigmc(1).*r4 sigmc(2).*r5 sigmc(3).*r6];
else
    % Acc correlation noise
    acorr = zeros(ref.kn,3);
    dt=mean(diff(imu.t));
    alpha = exp(-dt./imu.acorr);
    sigmc = imu.ab_drift .* sqrt(1 - alpha.^2);
    acorr(1,:) = ab_fix' + [sigmc(1).*r4(1) sigmc(2).*r5(1) sigmc(3).*r6(1)];
    for i=2:ref.kn    
        acorr (i,:) = alpha .* acorr (i-1,:) + [sigmc(1).*r4(i) sigmc(2).*r5(i) sigmc(3).*r6(i)]; 
    end
end

fb = acc_b - cor_b - g_b + ...
     [imu.astd(1)  .*r1  imu.astd(2)  .*r2  imu.astd(3)  .*r3 ] + ...
     [ab_fix(1).*o   ab_fix(2).*o   ab_fix(3).*o] + ...
     acorr; 

end
