function [vel_upd, coriolis] = vel_update(fn, velold, omega_ie_N, omega_en_N, g, dt)
% vel_update: updates velocity in the NED frame.
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
% of Engineering and Technology, USA. Eq 3.69.
%
%  			R. Gonzalez, J. Giribet, and H. Patiño. An approach to
% benchmarking of loosely coupled low-cost navigation systems,
% Mathematical and Computer Modelling of Dynamical Systems, vol. 21,
% issue 3, pp. 272-287, 2015. Eq. 17.
%
% Version: 002
% Date:    2015/07/18
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

S = skewm(velold);

coriolis = S * (omega_en_N + 2 * omega_ie_N);

% KNOWN ISSUES: acceleration in Z axis is considered negative downward because 
% Navego works (surprisingly) better this way. This Z axis orientation does 
% not correspond with NED coordinates. So, g is positive in the following equation.

fn_cor = fn - coriolis - (-g);

vel_upd = velold + fn_cor * dt;

end
