function DCMnb = euler2dcm(euler)
% euler2dcm: converts from Euler angles to DCM nav-to-body.
% 
% INPUT:
%   euler,	3x1 Euler angles (rad).
%
% OUTPUT:
%   DCMnb,  3x3 nav-to-body DCM.
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
% of Engineering and Technology, USA. Eq. 3.47, p. 41.
%
% Version: 001
% Date:    2014/09/11
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

  phi = euler(1); theta = euler(2); psi = euler(3);

  C1 = [cos(psi)  sin(psi) 0; ...
        -sin(psi) cos(psi) 0; ...
         0     0   1];
     
  C2 = [cos(theta)  0  -sin(theta); ...
          0   1     0 ; ...
        sin(theta)  0   cos(theta)];
    
  C3 = [1   0    0;   ...
        0  cos(phi) sin(phi); ...
        0 -sin(phi) cos(phi)];  
 
  DCMnb = C3 * (C2 * C1);

end
