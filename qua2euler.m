function euler = qua2euler(qin)
% qua2euler: transforms quaternion to Euler angles.
%
% INPUT:
%   qin,    4x1 quaternion.
%
% OUTPUT:
%   euler,  3x1 Euler angles (radians).
%
%   Copyright (C) 2014, Rodrigo Gonzalez, all rights reserved.
%
%   This file is part of NaveGo, an open-source MATLAB toolbox for
%   simulation of integrated navigation systems.
%
%   NaveGo is free software: you can redistribute it and/or modify
%   it under the terms of the GNU Lesser General Public License (LGPL)
%   version 3 as published by the Free Software Foundation.000000000
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
%			Dr. Paolo Zoccarato's comments at
%   https://github.com/rodralez/NaveGo/pull/9
%
% Version: 005
% Date:    2017/12/05
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% Quaternion format as used in Crassidis' book quaternion equations.

DCMbn = qua2dcm(qin);

phi   = atan2( DCMbn(3,2), DCMbn(3,3) );    % roll
theta = asin (-DCMbn(3,1) );                % pitch
psi   = atan2( DCMbn(2,1), DCMbn(1,1) );    % yaw

euler = [phi theta psi];

end
