function cor_n = coriolis(lat, vel, h)
% coriolis: calculates Coriolis forces in the navigation frame.
%
% INPUT
%       lat, Mx1 latitude (radians).
%       vel, Mx3 NED velocities (m/s).
%         h, Mx1 altitude (m).
%
% OUTPUT
%		cor_n, Mx3 Coriolis forces vector in the nav-frame (m/s^2).
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
%	R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a 
% simulation framework for low-cost integrated navigation systems, 
% Journal of Control Engineering and Applied Informatics}, vol. 17, 
% issue 2, pp. 110-120, 2015. Eq. 11.
%
% Version: 002
% Date:    2022/01/25
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

M = max(size(lat)); 

cor_n = zeros(M, 3);

for i = 1:M
   
    omega_en_n = transport_rate(lat(i), vel(i,1), vel(i,2), h(i));
    omega_ie_n = earth_rate(lat(i));

    cor_n(i,:) = (omega_en_n + 2*omega_ie_n) * vel(i,:)';       
end
