function g_b = gravity_b(lat, h, DCMnb_m)
%  gravity_b: computes gravity vector in the b-frame.
%
% INPUT
%	lat: Nx1 latitudes (radians).
%	h: Nx1 altitudes (meters)
%   DCMnb_m: Nx9 matrix with nav-to-body direct cosine matrices (DCM).
%       Each row of DCMnb_m contains the 9 elements of a particular DCMnb
%       matrix ordered as [a11 a21 a31 a12 a22 a32 a13 a23 a33].
%
% OUTPUT
%	g_b: Nx1 gravity force in the body frame.
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
% issue 2, pp. 110-120, 2015. Eq. 12.
%
% Version: 002
% Date:    2020/11/03
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

M = max(size(lat));

g_b = zeros(M,3);

for i = 1:M
    
    dcmnb = reshape(DCMnb_m(i,:), 3, 3);
    g_n = gravity(lat(i), h(i));
    gb = dcmnb * g_n';
    g_b(i,:) = gb';
end

end
