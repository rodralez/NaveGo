function deltha = gyro_gen_delta(DCMnb_m, t)
% gyro_gen_delta: calculates gyros delta angles.
%
% INPUT
%   DCMnb_m: Nx9 matrix with nav-to-body direct cosine matrices (DCM).
%       Each row of DCMnb_m contains the 9 elements of a particular DCMnb
%       matrix ordered as [a11 a21 a31 a12 a22 a32 a13 a23 a33].
%	t: Mx1 time vector (seconds).
%
% OUTPUT
%	deltha: Mx3 gyros delta angles [X, Y, Z] (rad, rad, rad).
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
% Journal of Control Engineering and Applied Informatics, vol. 17,
% issue 2, pp. 110-120, 2015. Eq. 5 and 6.
%
% Version: 004
% Date:    2020/11/03
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

M = max(size(DCMnb_m));

% Preallocation
dphi   = zeros(M-1, 1);
dtheta = zeros(M-1, 1);
dpsi   = zeros(M-1, 1);

I = eye(3);

% Computation of gyros delta angles
for k = 2:M
    
    dcmnb = reshape(DCMnb_m(k,:), 3, 3);
    dcmnb_old = reshape(DCMnb_m(k-1,:), 3, 3);
    
    dPSI = (dcmnb_old * dcmnb') - I;
    
    dphi(k-1, 1)   = dPSI(3, 2);
    dtheta(k-1, 1) = dPSI(1, 3);
    dpsi(k-1, 1)   = dPSI(2, 1);
end

% Computation of derivatives 
t_diff = diff(t);
deltha = [dphi./t_diff dtheta./t_diff dpsi./t_diff];

end
