function [gps, gps_r] = gps_gen(ref, gps)
% gps_gen: generates GPS position and GPS velocity in the n-frame from reference data.
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
% Journal of Control Engineering and Applied Informatics}, vol. 17, 
% issue 2, pp. 110-120, 2015. Sec. 2.3.
%
% Version: 001
% Date:    2014/09/11
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

[m,n] = size (ref.t);

if n>m, m=n; end

% Downsampling GPS estimates from 1/dt Hz to freq Hz.
dt   = mean(diff(ref.t));
freq = 1/dt;
dspl = round(freq / gps.freq);

dow = floor(m/dspl);

gps_r.t     = ref.t    (1:dspl:end, :);
gps_r.lat   = ref.lat  (1:dspl:end, :);
gps_r.lon   = ref.lon  (1:dspl:end, :);
gps_r.h     = ref.h    (1:dspl:end, :);
gps_r.vel   = ref.vel  (1:dspl:end, :);

gps_r.kn = dow;
gps_r.freq = round(1/mean(diff(gps_r.t)));

r1 = randn(size(gps_r.t));
r2 = randn(size(gps_r.t));
r3 = randn(size(gps_r.t));
r4 = randn(size(gps_r.t));
r5 = randn(size(gps_r.t));
r6 = randn(size(gps_r.t));

gps.t   = gps_r.t;
gps.lat = gps_r.lat + gps.std(1) .* r1;
gps.lon = gps_r.lon + gps.std(2) .* r2;
gps.h   = gps_r.h   + gps.std(3) .* r3;
gps.vel = gps_r.vel + [gps.stdv(1).*r4  gps.stdv(2).*r5  gps.stdv(3).*r6];

end

