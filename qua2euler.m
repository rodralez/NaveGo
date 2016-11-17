function ang_v = qua2euler(qin)
% qua2euler: transforms quaternion to Euler angles.
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
% Version: 002
% Date:    2016/05/09
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% Quaternion format used in Crassidis' quaternion equations.
qua(1) = qin(4);
qua(2) = qin(1);
qua(3) = qin(2);
qua(4) = qin(3);

% ZYX rotation sequence
c1 = 2.*(qua(2).*qua(3) + qua(1).*qua(4));
c2 = qua(1).^2 + qua(2).^2 - qua(3).^2 - qua(4).^2;

c3 = -2.*(qua(2).*qua(4) - qua(1).*qua(3));

c4 = 2.*(qua(3).*qua(4) + qua(1).*qua(2));
c5 = qua(1).^2 - qua(2).^2 - qua(3).^2 + qua(4).^2;

psi = atan2( c1, c2 );  % yaw
theta = asin( c3 );     % pitch
phi = atan2( c4, c5 );  % roll

ang_v = [phi theta psi];

end
