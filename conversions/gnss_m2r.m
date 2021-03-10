function  gnss = gnss_m2r (lat, h, gnss)
% gnss_m2r: converts GPS standard deviation from meters to radians.
%
% INPUT
%   gnss, GNSS data structure with fields: 
%       lat, 1x1 latitude (radians).
%       h,   1x1 altitude (meters).
%       stdm, 1x3 position error profile (m, m, m).
%
% OUTPUT
%   gnss.std, 1x3 position error profile (rad, rad, m).
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
%
%   R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a 
% simulation framework for low-cost integrated navigation systems, 
% Journal of Control Engineering and Applied Informatics, vol. 17, 
% issue 2, pp. 110-120, 2015. Eq. 20.
%
%  	R. Gonzalez, J. Giribet, and H. Patiño. An approach to 
% benchmarking of loosely coupled low-cost navigation systems, 
% Mathematical and Computer Modelling of Dynamical Systems, vol. 21, 
% issue 3, pp. 272-287, 2015. Eq. 7.
%
% Version: 003
% Date:    2021/03/09
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

gnss.std =  zeros(1,3);

[RM, RN] = radius(lat);

gnss.std(1) = gnss.stdm(1) / (RM + h);                  
gnss.std(2) = gnss.stdm(2) / (RN + h) / cos (lat);    
gnss.std(3) = gnss.stdm(3);

end
