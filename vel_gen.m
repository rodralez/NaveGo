function vel = vel_gen(lat, lon, h, dt)
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
    error('vel_gen: negative altitude.')
end

% h_n  = h - (vd) * dt;

h_diff = diff(h);
vd = -h_diff * dt;

%% North Velocity 

if (isa(h,'single'))
    [RM,~] = radius(lat(2:end), 'single');
else
    [RM,~] = radius(lat(2:end), 'double');
end

% lat_n = lat + (vn_c) * dt;
% vn_c = vn / (RM + h_n);

lat_diff = diff(lat);
vn_c = lat_diff * dt;
vn = vn_c .* (RM + h(2:end));

%% East Velocity 

if (isa(h,'single'))
    [~, RN] = radius(lat(2:end), 'single');
else
    [~, RN] = radius(lat(2:end), 'double');
end

% lon_n = lon + (ve_c) * dt;
% ve_c  = ve / ((RN + h_n) * cos (lat_n));

lon_diff = diff(lon);
ve_c = lon_diff * dt;
ve   = ve_c .* ((RN + h(2:end)) .* cos (lat(2:end)));

%% NED Velocity
  
vel = [vn ve vd];

end
