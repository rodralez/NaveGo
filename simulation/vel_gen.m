function vel_ned = vel_gen(lat, lon, h, t)
% vel_gen: generates NED velocities from llh position
%
% INPUT:
%   lat,  Nx1 latitude (radians)
%   lon,  Nx1 longitude (radians)
%   h,    Nx1 altitude (m)
%   dt,   1x1 sample time (s)
%
% OUTPUT:
%   vel,  Nx3 NED velocities [Vn Ve Vd] (m/s)
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
%   pos_update() function from NaveGo.
%
% Version: 001
% Date:    2017/11/03
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

%% Down Velocity 

idl = h < 0;

if any(idl)
    warning('vel_gen: negative altitude.')
end

% h_n  = h - (vd) * dt;

vd = - diff_central(t, h);

%% North Velocity 

if (isa(h,'single'))
    [RM,~] = radius(lat(2:end-1), 'single');
else
    [RM,~] = radius(lat(2:end-1), 'double');
end

% lat_n = lat + (vn_c) * dt;
% vn_c = vn / (RM + h_n);

% vn_c = diff(lat) ./ diff(t);

vn_c = diff_central(t, lat);
vn = vn_c .* (RM + h(2:end-1));

%% East Velocity 

if (isa(h,'single'))
    [~, RN] = radius(lat(2:end-1), 'single');
else
    [~, RN] = radius(lat(2:end-1), 'double');
end

% lon_n = lon + (ve_c) * dt;
% ve_c  = ve / ((RN + h_n) * cos (lat_n));

ve_c = diff_central(t, lon);
ve   = ve_c .* (RN + h(2:end-1)) .* cos (lat(2:end-1));

%% NED Velocity
  
vel = [vn ve vd];

vel_ned = sgolayfilt(vel, 5, 15);


end
