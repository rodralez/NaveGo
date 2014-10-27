function pos = pos_update(pos, vel, dt)
% pos_update: updates position in the navigation frame.
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
% Version: 001
% Date:    2014/09/11
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 
%
% Reference: R. González, J. Giribet, and H. Patiño, “An approach to 
% benchmarking of loosely coupled low-cost navigation systems,” 
% Mathematical and Computer Modelling of Dynamical Systems, Sept. 2014.
% Eq. 18.

lat_old = pos(1); 
lon_old = pos(2); 
hc = (pos(3));
v_n = vel(1);
v_e = vel(2);
v_d = vel(3);

if (isa(hc,'single')) 
    [RM,~] = radius(lat_old, 'single');
else
    [RM,~] = radius(lat_old, 'double');
end

h  = (hc - (v_d * dt));

lat = (lat_old + ( v_n / (RM + (h)) ) * dt);

if (isa(hc,'single')) 
    [~, RN] = radius(lat, 'single');
else
    [~, RN] = radius(lat, 'double');
end

lon = (lon_old + (v_e / ((RN + h) * cos (lat))) * dt );    

pos = [lat lon h]';
end
