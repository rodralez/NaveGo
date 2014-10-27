function g_n = gravity(lat, h)
% gravity: calculates gravity vector in the navigation frame.
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

% Reference: R. González, J. Giribet, and H. Patiño, “An approach to 
% benchmarking of loosely coupled low-cost navigation systems,” 
% Mathematical and Computer Modelling of Dynamical Systems, Sept. 2014.
% Eq. 16.

% Set gravity uncertainty
% a = 9.81 * 0.1;
% b = 9.81 * 0.1;
% g_noise = (b-a).*rand(1) + a;

h = abs(h);
sin1 = sin(lat);
sin2 = sin(2.*lat);

g0 = 9.780318*( 1 + 5.3024e-03.*(sin1).^2 - 5.9e-06.*(sin2).^2);   

if (isa(h,'single')) 
    
    [RM,RN] = radius(lat, 'single');
else
    
    [RM,RN] = radius(lat);
end

Ro = sqrt(RN .* RM); 

g = (g0 ./ (1 + (h ./ Ro)).^2);    

g_n = [zeros(size(g)) zeros(size(g)) g];

end
   
