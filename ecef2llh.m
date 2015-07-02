function llh = ecef2llh(ecef)
% ecef2llh:	converts from ECEF coordinates to navigation coordinates
%           (latitude, longitude and altitude).
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
% INPUTS
%	ecef(1) = ECEF x-coordinate in meters
%	ecef(2) = ECEF y-coordinate in meters
%	ecef(3) = ECEF z-coordinate in meters
%
% OUTPUTS
%	llh(1) = latitude in radians
%	llh(2) = longitude in radians
%	llh(3) = height above ellipsoid in meters
%
% References: 
%			H. Vermeille. Direct transformation from geocentric 
% coordinates to geodetic coordinates. Journal of Geodesy, 2002. 
%
%  			R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a 
% simulation framework for low-cost integrated navigation systems, 
% Journal of Control Engineering and Applied Informatics}, vol. 17, 
% issue 2, pp. 110-120, 2015. Eq. 17.
%
% Version: 001
% Date:    2014/09/11
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

	x = ecef(1);
	y = ecef(2);
	z = ecef(3);
    
	a = 6378137.0000;	% Earth radius in meters
	b = 6356752.3142;	% Earth semiminor in meters	
    e = 0.0818191908426;% Eccentricity 

    p = (x^2+y^2)/a^2;
	q = ((1-e^2) * z^2)/ a^2 ;
    r = (p + q - e^4)/6;
    s = (e^4 * p * q)/(4 * r^3);
    t = (1+s+sqrt(s*(2+s)))^(1/3);
    u = r * (1 + t + (1/t));
    v = sqrt(u^2 + q * e^4);
    w = e^2*(u+v-q)/(2*v);
    k = sqrt(u+v+w^2)-w;
    
    D = k*sqrt(x^2+y^2)/(k+e^2);        

    % Latitude
%     llh(1) = 2 * atan((z /(D+sqrt(D^2 + z^2))));
    llh(1) = atan2(z , D);
    
    % Longitude
%     llh(2) = 2 * atan((y /(x+sqrt(x^2 + y^2))));
    llh(2) = atan2(y , x);
    
    % Altitude
	llh(3) = (k + e^2 -1)/k *(sqrt(D^2 + z^2));

end

