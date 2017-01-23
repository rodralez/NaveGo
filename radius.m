function [RM,RN] = radius(lat, precision)
% radius: calculates meridian and normal radii of curvature.
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
%			Titterton, D.H. and Weston, J.L. (2004). Strapdown
% Inertial Navigation Technology (2nd Ed.). Institution
% of Engineering and Technology, USA. Eq. 2.6 and 2.7.
%
% 			Groves, P. (2008). Principles of GNSS, Inertial, and
% Multisensor Integrated Navigation Systems. Artech House, UK.
% Eq. 2.65 and 2.66.
%
%  			R. Gonzalez, J. Giribet, and H. Patiño. An approach to
% benchmarking of loosely coupled low-cost navigation systems,
% Mathematical and Computer Modelling of Dynamical Systems, vol. 21,
% issue 3, pp. 272-287, 2015. Eq. 11.
%
% Version: 003
% Date:    2017/01/19
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

if nargin < 2, precision = 'double'; end

if strcmp(precision, 'single')
    
    a = single(6378137.0);
    e = single(0.0818191908426);
    
    e2 = e^2;
    den = 1 - e2.*single(sin(lat)).^2;
    
else
    a = (6378137.0);
    e = (0.0818191908426);
    
    e2 = e^2;
    den = 1 - e2.*(sin(lat)).^2;
end

% Meridian radius of curvature: radius of curvature for north-south motion.
RM = a * (1-e2) ./ (den).^(3/2);

% Normal radius of curvature: radius of curvature for east-west motion. 
% AKA transverse radius.
RN = a ./ sqrt(den);

end
