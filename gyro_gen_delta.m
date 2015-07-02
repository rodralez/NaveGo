function deltha = gyro_gen_delta(DCMnb, dt)
% gyro_gen_delta: calculates gyros delta measurements.
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
% issue 2, pp. 110-120, 2015. Eq. 5.
%
% Version: 001
% Date:    2014/09/11
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

kn = max(size(DCMnb));

dphi = zeros(kn-1,1);
dtheta = zeros(kn-1,1);
dpsi = zeros(kn-1,1);

for k = 2:kn

  dcmnb2 = reshape(DCMnb(k,:), 3, 3); 
  dcmnb1 = reshape(DCMnb(k-1,:), 3, 3);  

  dPSI = (dcmnb1 * dcmnb2') - eye(3);
   
  dphi(k-1,1)   = dPSI(3,2) ;
  dtheta(k-1,1) = dPSI(1,3) ;
  dpsi(k-1,1)   = dPSI(2,1) ;
 
end

deltha = [dphi./dt dtheta./dt dpsi./dt];

end
