function qua = qua_update(qua, wb, dt)
% qua_update: updates quaternions.
%
% INPUT
%   qua,    4x1 quaternion.
%   wb,     3x1 incremental turn rates in body-frame [X Y Z] (rad/s).
%   dt,     1x1 IMU sampling interval (s).
%
% OUTPUT
%   qua,    4x1 updated quaternion.
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
%
%   Crassidis, J.L. and Junkins, J.L. (2011). Optimal Esti-
% mation of Dynamic Systems, 2nd Ed. Chapman and Hall/CRC, USA.
% Eq. 7.39 to 7.41, p. 458.
%
% Version: 005
% Date:    2022/04/07
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

wnorm = norm(wb);

if wnorm < 1.E-8
    
    return;
else
    
    co = cos(0.5*wnorm*dt);
    si = sin(0.5*wnorm*dt);
    
    % Eq. 7.41
    psi = (si / wnorm) * wb;  
    
    % Eq. 7.40
    Om = [ (co*eye(3)-skewm(psi))  psi; % 3x4
           -psi'                   co]; % 1x4
    
    % Eq. 7.39
    qua = Om * qua;
end

end
