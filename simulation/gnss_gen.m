function [gnss, gnss_r] = gnss_gen(ref, gnss)
% gnss_gen: generates GNSS position and GNSS velocity in the n-frame from reference data.
%
% INPUT
%   ref: reference data structure.
%   gnss: GNSS data structure.
%
% OUTPUT
%   gnss: GNSS data structure with noisy measurements.
%   gnss_r: GNSS data structure with true measurements.
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
%   R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a 
% simulation framework for low-cost integrated navigation systems, 
% Journal of Control Engineering and Applied Informatics}, vol. 17, 
% issue 2, pp. 110-120, 2015. Sec. 2.3.
%
%   gen_gps.m, gnss_gen is based on this previous function.
%
% Version: 001
% Date:    2018/10/10
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

[m,n] = size (ref.t);

if n > m, m = n; end

% Downsampling GNSS estimates from ref.freq to gnss.freq.
dt   = mean(diff(ref.t));
freq = 1/dt;
dspl = round(freq / gnss.freq);

gnss_r.t     = ref.t    (1:dspl:end, :);
gnss_r.lat   = ref.lat  (1:dspl:end, :);
gnss_r.lon   = ref.lon  (1:dspl:end, :);
gnss_r.h     = ref.h    (1:dspl:end, :);
gnss_r.vel   = ref.vel  (1:dspl:end, :);

gnss_r.freq = round(1/mean(diff(gnss_r.t)));

% Gaussian noise vectors
r1 = randn(size(gnss_r.t));
r2 = randn(size(gnss_r.t));
r3 = randn(size(gnss_r.t));

r4 = randn(size(gnss_r.t));
r5 = randn(size(gnss_r.t));
r6 = randn(size(gnss_r.t));

gnss.t   = gnss_r.t;
gnss.lat = gnss_r.lat + gnss.std(1) .* r1;
gnss.lon = gnss_r.lon + gnss.std(2) .* r2;
gnss.h   = gnss_r.h   + gnss.std(3) .* r3;
gnss.vel = gnss_r.vel + [gnss.stdv(1).*r4  gnss.stdv(2).*r5  gnss.stdv(3).*r6];

end

