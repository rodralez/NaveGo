function DCMbn_n = dcm_update(DCMbn, euler)
% dcm_update: updates  body-to-nav DCM.
%
% INPUT:
%   DCMbn,	3x3 body-to-nav DCM.
%   euler,	3x1 Euler angles (rad).
%
% OUTPUT:
%   DCMbn_n,    3x3 updated DCM body-to-nav.
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
%			Titterton, D.H. and Weston, J.L. (2004). Strapdown
% Inertial Navigation Technology (2nd Ed.). Institution
% of Engineering and Technology, USA. Eq. 11.4 and 11.10.
%
% Version: 002
% Date:    2016/11/26
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

S = skewm(euler);
magn = norm(euler);

if magn == 0,
    
    A = eye(3);
else
    % A(k), Eq. 11.10, p. 312.
    A = eye(3) + (sin(magn)/magn) * S + ((1-cos(magn))/(magn^2)) * S * S;
end

% Eq. 11.4, p. 311.
DCMbn_n = DCMbn * A;
