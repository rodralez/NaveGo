function deltha = gyro_gen_delta(DCMnb, t)
% gyro_gen_delta: calculates gyros delta angles.
%
% INPUT:
%		DCMnb: Mx9 DCM nav-to-body matrices.
%		t: Mx1 time vector (s).
%
% OUTPUT:
%		deltha: Mx3 gyros delta angles (radians).
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
%			R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a 
% simulation framework for low-cost integrated navigation systems, 
% Journal of Control Engineering and Applied Informatics, vol. 17, 
% issue 2, pp. 110-120, 2015. Eq. 5 and 6.
%
% Reference:
%
% Version: 003
% Date:    2019/01/09
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 


M = max(size(DCMnb));

% Preallocate

dphi   = zeros(M-1,1);
dtheta = zeros(M-1,1);
dpsi   = zeros(M-1,1);

% Calculate gyros delta angles.

for k = 2:M

  dcmnb = reshape(DCMnb(k,:), 3, 3); 
  dcmnb_old = reshape(DCMnb(k-1,:), 3, 3);  

  dPSI = (dcmnb_old * dcmnb') - eye(3);
   
  dphi(k-1,1)   = dPSI(3,2);
  dtheta(k-1,1) = dPSI(1,3);
  dpsi(k-1,1)   = dPSI(2,1); 
end

% Derivates

t_diff = diff(t);
deltha = [dphi./t_diff dtheta./t_diff dpsi./t_diff];

end
