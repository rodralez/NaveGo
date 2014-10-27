function omega_en_N = transportrate(lat, Vn, Ve, h)
% omega_en_N: calculates the transport rate in the navigation frame.
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
% Reference: R. González, J. Giribet, and H. Patiño, “An approach to 
% benchmarking of loosely coupled low-cost navigation systems,” 
% Mathematical and Computer Modelling of Dynamical Systems, Sept. 2014.
% Eq. 10.
%
% Reference: Grover (p.131) and Farrel (pag.413) 
  

h=abs(h);

if (isa(Vn,'single')) 
    
    [RM,RN] = radius(lat, 'single');

    omega_en_N(1,1) = single(Ve /(RN + h));                 % North 
    omega_en_N(2,1) = single(-(Vn /(RM + h)));              % East
    omega_en_N(3,1) = single(-(Ve * tan(lat) / (RN + h)));  % Down
else
    
    [RM,RN] = radius(lat);

    omega_en_N(1,1) = (Ve /(RN + h));                 % North 
    omega_en_N(2,1) = (-(Vn /(RM + h)));              % East
    omega_en_N(3,1) = (-(Ve * tan(lat) / (RN + h)));  % Down

end                               

end
