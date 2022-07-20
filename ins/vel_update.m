function vel_n = vel_update(fn, vel_o, omega_ie_n, omega_en_n, gn, dt)
% vel_update: updates velocity vector in the NED frame.
%
% INPUT
%   fn, 3x1 specific forces in the nav-frame (m/s^2).    
%   vel_o, 3x1 previous (old) velocity vector in the nav-frame (m/s). 
%   omega_ie_n, 3x3 previous skew-symmetric Earth rate matrix in the nav-frame (rad/s).
%   omega_en_n, 3x3 previous skew-symmetric transport rate matrix in the nav-frame (rad/s).
%   gn, 3x1 previous gravity in the nav-frame (m/s^2).
%   dt, 1x1 integration time step (s).
%
% OUTPUT
%   vel_n, 3x1 updated velocity vector in the nav-frame (m/s).    
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
%   Paul D. Groves. Principles of GNSS, Inertial, and
% Multisensor Integrated Navigation Systems. Second Edition.
% Eq. 5.53 and 5.54, page 179.
%
% Version: 006
% Date:    2022/01/25
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego
 
coriolis = (omega_en_n + 2 * omega_ie_n);  % Coriolis 

fn_c = fn + gn - (coriolis * vel_o');      % Corrected specific force in nav-frame

vel_n = vel_o + (fn_c' * dt) ;             % Velocity update

end
