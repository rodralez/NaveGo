function [vel_upd, coriolis] = vel_update(fn, velold, omega_ie_N, omega_en_N, g, dt)
% qua_update: updates velocity in the NED frame.
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
% Eq. 17.
%
% Reference: Crassidis, Eq. 7.39, p. 458.

    S = skewm(velold);

    coriolis = S * (omega_en_N + 2*omega_ie_N);

    fn_cor = fn + coriolis + g;

    vel_upd = velold + fn_cor * dt;

end
