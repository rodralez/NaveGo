function yawm = hd_update( mag, roll, pitch, D)
% hd_update: estimates magnetic heading from magnetometer data.
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
%		M.J. Caruso. Applications of magnetic sensors for low cost 
%		compass systems. In Position Location and Navigation Symposium, 
%		IEEE 2000 (pp. 177-184).
%
% Version: 001
% Date:    2016/08/03
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

B = [ cos(pitch) sin(pitch)*sin(roll) -cos(roll)*sin(pitch); ...
    0 cos(roll) sin(roll); ];

magh = (B * mag');

yh = magh(2);
xh = magh(1);
atanh = atan2(yh, xh);

if xh < 0
    
    yawm =  pi - atanh ;
    
elseif (xh > 0 && yh < 0)
    
    yawm = -atanh ;
    
elseif (xh > 0 && yh > 0)
    
    yawm = 2*pi - atanh;
    
elseif (xh == 0 && yh < 0)
    
    yawm = pi/2;
    
elseif (xh == 0 && yh > 0)
    
    yawm = (3/2) * pi;
    
else
    
    warning('hd_update: default option!') ;
end

% Correct magnetic declination
yawm = yawm + D;

% Yaw must be in the range of [-pi pi]
if yawm < -pi
    
    yawm = yawm + 2*pi;
    
elseif yawm > pi
    
    yawm = yawm - 2*pi;
    
else
    % dumb
    yawm = yawm;    
end


