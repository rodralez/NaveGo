function DCMbn = dcm_update(DCMbn, euler)
% dcm_update: updates  body-to-nav DCM.
%
% INPUT
%   DCMbn,	3x3 body-to-nav DCM.
%   euler,	3x1 Euler angles [roll pitch yaw] (rad).
%
% OUTPUT
%   DCMbn,  3x3 updated DCM body-to-nav.
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
%	Titterton, D.H. and Weston, J.L. (2004). Strapdown
% Inertial Navigation Technology (2nd Ed.). Institution
% of Engineering and Technology, USA. Eq. 11.4 and 11.10.
%
%   Groves, P.D. (2013), Principles of GNSS, Inertial, and
% Multisensor Integrated Navigation Systems (2nd Ed.). Artech House.
% 
% Version: 004
% Date:    2021/03/18
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

S = skewm(euler);
mag_e = norm(euler);

if mag_e < 1.E-8
    
    A = eye(3);
else
    
    % Rodrigues' formula
    % Titterton, A(k), Eq. 11.10, p. 312
    % Groves, Eq. 5.73
    % A = eye(3) + (sin(mag_e)/mag_e) * S + ((1-cos(mag_e))/(mag_e^2)) * S * S;
    
    % Exact expression. Groves, Eq. 5.69.
    A = expm(S);
end

% Titterton, Eq. 11.4, p. 311.
DCMbn = DCMbn * A;

% Brute-force orthogonalization, Groves, Eq 5.79
c1 = DCMbn(:,1);
c2 = DCMbn(:,2);
c3 = DCMbn(:,3);

c1 = c1 - 0.5 * (c1'*c2) * c2 - 0.5 * (c1'*c3) * c3 ;
c2 = c2 - 0.5 * (c1'*c2) * c1 - 0.5 * (c2'*c3) * c3 ;
c3 = c3 - 0.5 * (c1'*c3) * c1 - 0.5 * (c2'*c3) * c2 ;

% Brute-force normalization, Groves, Eq 5.80
c1 = c1 / sqrt(c1'*c1);
c2 = c2 / sqrt(c2'*c2);
c3 = c3 / sqrt(c3'*c3);

DCMbn = [c1 , c2 , c3 ];
