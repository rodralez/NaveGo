function  gps = gps_err_profile (lat, h, gps)
% gps_err_profile: converts GPS standard deviation from meters to radians.
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
% References: 
%			R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a 
% simulation framework for low-cost integrated navigation systems, 
% Journal of Control Engineering and Applied Informatics, vol. 17, 
% issue 2, pp. 110-120, 2015. Eq. 20.
%
%  			R. Gonzalez, J. Giribet, and H. Patiño. An approach to 
% benchmarking of loosely coupled low-cost navigation systems, 
% Mathematical and Computer Modelling of Dynamical Systems, vol. 21, 
% issue 3, pp. 272-287, 2015. Eq. 7.
%
% Version: 001
% Date:    2014/09/11
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

gps.std =  zeros(1,3);

[RM, RN] = radius(lat, 'double');

gps.std(1) = gps.stdm(1) / (RM + h);                  
gps.std(2) = gps.stdm(2) / (RN + h) / cos (lat);    
gps.std(3) = gps.stdm(3);

end
