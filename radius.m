function [RM,RN] = radius(lat, precision)
% radius: calculates meridian radius and normal radius. 
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
% Eq. 11.

if nargin < 2, precision = 'double'; end

if strcmp(precision, 'single')
    
    a = single(6378137.0);    
    e = single(0.0818191908426); 

    e2 = e^2;
    den = 1 - e2.*single(sin(lat)).^2;

    RM = a * (1-e2) ./ (den).^(3/2); 

    RN = a ./ sqrt(den); 

else
    a = (6378137.0);    
    e = (0.0818191908426); 

    e2 = e^2;
    den = 1 - e2.*(sin(lat)).^2;

    RM = a * (1-e2) ./ (den).^(3/2); 

    RN = a ./ sqrt(den);  

end

end
