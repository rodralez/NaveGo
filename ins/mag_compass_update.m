function yawm = mag_compass_update(m_n, dec, inc, DCMnb, roll, pitch)
% magh_update: calculates magnetic heading angle (yaw) from magnetometer data.
%
% INPUT
%   mag, 1x3 magnetic flux density (Tesla).
%   dec, magnetic declination angle (rad).
%   inc, magnetic inclination angle (rad).
%   DCMnb, 3x3 DCM nav-to-body.
%   roll, roll angle (rad).
%   pitch, pitch angle (rad).
%
% OUTPUT
%   yawm, yaw angle from magnetometer data.
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
%   Paul D. Groves. Principles of GNSS, Inertial, and
% Multisensor Integrated Navigation Systems. Second Edition.
% Eq. 6.2 to 6.6, page 219.
%
%   NOAA, Magnetic Field Calculators. 
% https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
%
% Version: 002
% Date:    2021/03/20
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

B = norm( m_n );                    % Magnitude of the flux density
 
D = [ cos(dec) * cos(inc); sin(dec) * cos(inc); sin(inc); ];

m_b = ( DCMnb * D * B ) ;

x =  -m_b(2) * cos(roll)  + m_b(3) * sin(roll) ;
y =   m_b(1) * cos(pitch) + m_b(2) * sin(roll) * sin(pitch) ...
    + m_b(3) * cos(roll) * sin(pitch) ;

yawm = correct_yaw (atan2(x , y) + dec);    % Four-quadrant arctangent function

end
