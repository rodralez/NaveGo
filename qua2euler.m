function ang_v = qua2euler(qin)
% qua2dcm: transforms quaternion to Euler angles.
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

% Crassidis quaternion format
    qua(:,1) = qin(:,4);
    qua(:,2) = qin(:,1);
    qua(:,3) = qin(:,2);
    qua(:,4) = qin(:,3);
    
    r11 = 2.*(qua(:,2).*qua(:,3) + qua(:,1).*qua(:,4));
    r12 = qua(:,1).^2 + qua(:,2).^2 - qua(:,3).^2 - qua(:,4).^2;

    r21 = -2.*(qua(:,2).*qua(:,4) - qua(:,1).*qua(:,3));

    r31 = 2.*(qua(:,3).*qua(:,4) + qua(:,1).*qua(:,2));
    r32 = qua(:,1).^2 - qua(:,2).^2 - qua(:,3).^2 + qua(:,4).^2;

    psi = atan2( r11, r12 );
    theta = asin( r21 );
    phi = atan2( r31, r32 );

    ang_v = [phi theta psi];

end
