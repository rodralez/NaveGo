function fb = acc_nav2body (acc_n, DCMnb_m)
% acc_nav2body: transforms accelerations from navigation frame to body
% frame.
%
% INPUT
%	acc_n: Nx3 matrix with [fn, fe, fd] accelerations in the navigation
%		frame.
%   DCMnb_m: Nx9 matrix with nav-to-body direct cosine matrices (DCM).
%       Each row of DCMnb_m contains the 9 elements of a particular DCMnb
%       matrix ordered as [a11 a21 a31 a12 a22 a32 a13 a23 a33].
%
% OUTPUT
%	fb: Nx3 matrix with [fx, fy, fz] simulated accelerations in the
%		body frame.
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
%   R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a
% simulation framework for low-cost integrated navigation systems,
% Journal of Control Engineering and Applied Informatics}, vol. 17,
% issue 2, pp. 110-120, 2015. Eq. 10.
%
% Version: 002
% Date:    2020/11/03
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

fb = zeros(size(acc_n));

M = max(size(acc_n));

for k = 1:M
    
    dcmnb = reshape(DCMnb_m(k,:), 3, 3);
    fb(k,:) = ( dcmnb * ( acc_n(k,:)' ) )';
end

end
