function vel_n = vel_update(fn, vel_n, omega_ie_n, omega_en_n, g, dt)
% vel_update: updates velocity vector in the NED frame.
%
% INPUT
%   fn, 3x1 specific forces in the nav-frame.    
%   vel_n, 3x1 previous (old) velocity vector in the nav-frame. 
%   omega_ie_n, 3x1 Earth rate in the nav-frame.
%   omega_en_n, 3x1 transport rate in the nav-frame.
%   g, 3x1 gravity in the nav-frame.
%   dt, 1x1 integration time step.
%
% OUTPUT
%   vel_n, 3x1 velocity vector in the nav-frame.    
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
%	Titterton, D.H. and Weston, J.L. (2004). Strapdown
% Inertial Navigation Technology (2nd Ed.). Institution
% of Engineering and Technology, USA. Eq 3.69.
%
%	R. Gonzalez, J. Giribet, and H. Patiño. An approach to
% benchmarking of loosely coupled low-cost navigation systems,
% Mathematical and Computer Modelling of Dynamical Systems, vol. 21,
% issue 3, pp. 272-287, 2015. Eq. 17.
%
% Version: 003
% Date:    2020/10/05
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

S = skewm(vel_n);                               % Skew matrix with velocities
 
coriolis = S * (omega_en_n + 2 * omega_ie_n);   % Coriolis 

fn_c = fn - coriolis - g;                       % Corrected specific force in nav-frame

vel_n = vel_n + (fn_c' * dt);

end
