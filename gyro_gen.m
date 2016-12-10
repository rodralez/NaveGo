function wb = gyro_gen (ref, imu)
% gyro_gen: generates simulated gyros measurements from reference data and 
%          imu error profile.
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
% Version: 002
% Date:    2015/08/20
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

% True gyro
if (isfield(ref, 'wb'))    
    
    gyro_b = ref.wb;

% Obtain gyros from DCM    
else
    gyro_raw = gyro_gen_delta(ref.DCMnb, diff(ref.t));
    gyro_raw = [gyro_raw; 0 0 0;];
    gyro_b  = sgolayfilt(gyro_raw, 10, 45);
end

% omega_en_n and omega_ie_n
gyro_err_b = zeros(ref.kn, 3); 
for i = 1:ref.kn,
    
    dcmnb = reshape(ref.DCMnb(i,:), 3, 3); 
    omega_ie_n = earthrate(ref.lat(i));
    omega_en_n = transportrate(ref.lat(i), ref.vel(i,1), ref.vel(i,2), ref.h(i));
    omega_in_b = dcmnb * (omega_en_n + omega_ie_n ); 
    gyro_err_b(i,:) = ( omega_in_b )';   
end

% Set static bias randomly from interval
a = -imu.gb_fix(1);
b = imu.gb_fix(1);
gb_fix = (b-a).*rand(3,1) + a;

% Random vectors
r1=randn(ref.kn,1);
r2=randn(ref.kn,1);
r3=randn(ref.kn,1);
r4=randn(ref.kn,1);
r5=randn(ref.kn,1);
r6=randn(ref.kn,1);
o=ones(ref.kn,1);

if (isinf(imu.gb_corr))
    sigmc = imu.gb_drift;
    gb_corr = [sigmc(1).*r4 sigmc(2).*r5 sigmc(3).*r6];
else
    % Gyro correlation noise
    gb_corr = zeros(ref.kn,3);
    dt=mean(diff(ref.t));
    alpha = exp(-dt./imu.gb_corr);
    sigmc = imu.gb_drift .* sqrt(1 - alpha.^2);
    gb_corr(1,:) = gb_fix' + [sigmc(1).*r4(1) sigmc(2).*r5(1) sigmc(3).*r6(1)];

    for i=2:ref.kn    
        gb_corr (i,:) = alpha .* gb_corr (i-1,:) + [sigmc(1).*r4(i) sigmc(2).*r5(i) sigmc(3).*r6(i)]; 
    end
end

wb = gyro_b + gyro_err_b + ...
     [imu.gstd(1).*r1    imu.gstd(2).*r2    imu.gstd(3).*r3 ] + ...
     [gb_fix(1).*o   gb_fix(2).*o   gb_fix(3).*o] + ...
     gb_corr; 

end
