function euler_m = dcm2euler_m(DCMnb_m)
% dcm2euler_m: transforms a nav-to-body DCM matrix to an Euler angles matrix.
%
% INPUT
%   DCMnb_m: Nx9 matrix with nav-to-body direct cosine matrices (DCM).
% Each row of DCMnb_m contains the 9 elements of a particular DCMnb
% matrix ordered as [a11 a21 a31 a12 a22 a32 a13 a23 a33].
%
% OUTPUT
%   euler_m: Nx3 Euler angles ordered by column, [roll pitch yaw] (rad, rad, rad).
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
% 	Titterton, D.H. and Weston, J.L. (2004). Strapdown
% Inertial Navigation Technology (2nd Ed.). Institution
% of Engineering and Technology, USA. Eq. 11.4. Eq. 3.66, p. 46.
%
%   R. Gonzalez, J. Giribet, and H.D. Patiño,. An approach to
% benchmarking of loosely coupled low-cost navigation systems,
% Mathematical and Computer Modelling of Dynamical Systems, vol. 21,
% issue 2, pp. 272-287. Eq. 15.
%
% Version: 001
% Date:    2020/12/17
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

[N,~] = size (DCMnb_m);

euler_m = zeros(N,3);

for i=1:N
    
    DCMnb = reshape(DCMnb_m (i,:), 3, 3);
    DCMbn = DCMnb';                         % nav-to-body > body-to-nav
    
    euler_m (i,:) = dcm2euler(DCMbn);       % phi theta psi
end

end
