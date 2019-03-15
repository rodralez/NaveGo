function yaw_new = correct_yaw (yaw)
% correct_yaw: guarantee that yaw angle is between -pi and pi.
%
% INPUT:
%   yaw, Nx1 vector (radians).
%
% OUTPUT:
%   yaw_new, Nx1 vector (radians).
%
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
% Version: 001
% Date:    2019/01/09
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

yaw_new = yaw;

idx = (yaw > pi);
yaw_new(idx) = yaw(idx) - 2*pi;

idx = (yaw < -pi);
yaw_new(idx) = yaw(idx) + 2*pi;

end
