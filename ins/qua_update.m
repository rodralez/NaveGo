function qua_n = qua_update(qua, wb_n, dt)
% qua_update: updates quaternions.
%
% INPUT:
%   qua,	4x1 quaternion.
%   wb_n,	3x1 incremental turn rates in body-frame (rad/s).
%   dt,     1x1 IMU sampling interval (s).
%
% OUTPUT:
%   qua_n,      4x1 updated quaternion.
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

% References:
%  			R. Gonzalez, J. Giribet, and H. Patiño. An approach to
% benchmarking of loosely coupled low-cost navigation systems,
% Mathematical and Computer Modelling of Dynamical Systems, vol. 21,
% issue 3, pp. 272-287, 2015. Eq. 13.
%
%			Crassidis, J.L. and Junkins, J.L. (2011). Optimal Esti-
% mation of Dynamic Systems, 2nd Ed. Chapman and Hall/CRC, USA.
% Eq. 7.39 and 7.40, p. 458.
%
% Version: 002
% Date:    2016/11/26
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

wnorm = norm(wb_n);

if wnorm == 0,
    
    qua_n = qua;
else
    
    co=cos(0.5*wnorm*dt);
    si=sin(0.5*wnorm*dt);
    
    n1 = wb_n(1)/wnorm;
    n2 = wb_n(2)/wnorm;
    n3 = wb_n(3)/wnorm;
    
    qw1 = n1*si;
    qw2 = n2*si;
    qw3 = n3*si;
    qw4 = co;
    
    Om=[ qw4  qw3 -qw2 qw1;
        -qw3  qw4  qw1 qw2;
         qw2 -qw1  qw4 qw3;
        -qw1 -qw2 -qw3 qw4];
    
    qua_n = Om * qua;
end

end
