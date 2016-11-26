function pos = pos_update(pos, vel, dt)
% pos_update: updates position in the navigation frame.
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
%			Titterton, D.H. and Weston, J.L. (2004). Strapdown
% Inertial Navigation Technology (2nd Ed.). Institution
% of Engineering and Technology, USA. Eq. 3.79-3.81.
%
%			R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a 
% simulation framework for low-cost integrated navigation systems, 
% Journal of Control Engineering and Applied Informatics, vol. 17, 
% issue 2, pp. 110-120, 2015. Eq. 18.
%
% Version: 003
% Date:    2016/11/21
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

persistent vn_m
persistent ve_m
persistent vd_m

if(isempty(vn_m))
    vn_m = zeros(2,1);
end
if(isempty(ve_m))
    ve_m = zeros(2,1);
end
if(isempty(vd_m))
    vd_m = zeros(2,1);
end

lat = pos(1); 
lon = pos(2); 
h   = pos(3);
vn  = vel(1);
ve  = vel(2);
vd  = vel(3);

%% Altitude

vd_m(1) = vd;
h_n  = h - trapz(vd_m) * dt;
vd_m(2) = vd_m(1);

if h_n < 0
    h_n = 0;
    warning('pos_update: altitude is negative.')
end

%% Latitude

if (isa(h,'single')) 
    [RM,~] = radius(lat, 'single');
else
    [RM,~] = radius(lat, 'double');
end

vn_m(1) = vn / (RM + h_n);
lat_n = lat + trapz(vn_m) * dt;
vn_m(2) = vn_m(1);

%% Longitude

if (isa(h,'single')) 
    [~, RN] = radius(lat_n, 'single');
else
    [~, RN] = radius(lat_n, 'double');
end

ve_m(1) = ve / ((RN + h_n) * cos (lat_n));
lon_n = lon + trapz(ve_m) * dt;
ve_m(2) = ve_m(1);

%% Position update

pos = [lat_n lon_n h_n]';

end
