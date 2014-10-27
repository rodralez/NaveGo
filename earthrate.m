function omega_ie_N = earthrate(lat, precision)
% omega_ie_N: turn rate of the Earth in the navigation frame.
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
% Eq. 13.

if nargin < 2, precision = 'double'; end

if strcmp(precision, 'single')
    
    omega_ie_N = single((7.2921155e-5) .* [ single(cos(lat));  single(0) ; single(-sin(lat)) ]); 
else
    
    omega_ie_N = (7.2921155e-5) .* [ (cos(lat));  0 ; -sin(lat) ]; 
end
