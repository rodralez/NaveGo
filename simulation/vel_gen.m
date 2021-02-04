function vel_ned = vel_gen(lat, lon, h, t)
% vel_gen: generates NED velocities from llh position
%
% INPUT
%   lat: Nx1 latitude (radians).
%   lon: Nx1 longitude (radians).
%   h:   Nx1 altitude (m).
%   dt:  1x1 sampling time (s).
%
% OUTPUT
%   vel: Nx3 NED velocities [VN VE VD] (m/s, m/s, m/s).
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
%
%   pos_update() function from NaveGo.
%
% Version: 002
% Date:    2019/04/03
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

%% Down Velocity 

idl = h < 0;

if any(idl)
    warning('vel_gen: negative altitude.')
end

vd = - diff_central(t, h);

%% North Velocity 

[RM, ~] = radius(lat(2:end-1));

vn_c = diff_central(t, lat);
vn = vn_c .* (RM + h(2:end-1));

%% East Velocity 

[~, RN] = radius(lat(2:end-1));

ve_c = diff_central(t, lon);
ve   = ve_c .* (RN + h(2:end-1)) .* cos (lat(2:end-1));

%% NED Velocity
  
vel = [vn ve vd];

vel_ned = my_sgolayfilt(vel);

end
