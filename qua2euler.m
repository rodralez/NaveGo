function ang_v = qua2euler(qua)
% qua2dcm: transforms quaternion to Euler angles.
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
% Reference: 

    q4 = qua(4); q1 = qua(1); q2 = qua(2); q3 = qua(3);

    phi =   atan2( (2*((q4*q1)+(q2*q3))) ./ (1-(2*(((q1)^2) + ((q2)^2)))) , 1); % C_32 / C_33

    theta = asin(2*((q4*q2)-(q1*q3))); % - C_31

    psi =   atan2(2*(q4*q3+q1*q2), 1-2*(((q2)^2) + ((q3)^2)));  % C_21 / C_11

    ang_v = [phi theta psi];

end
