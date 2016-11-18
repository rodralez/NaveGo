function  [xpo, Upr, dpr] = ud_filtering(xpr, y, F, H, G, Q, R, Upr, dpr, dt)
% ud_filtering: version of kalman filter with numerical stability
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
% issue 2, pp. 110-120, 2015. Eq. 9, 14, and 30.
%
% Version: 001
% Date:    2016/05/11
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

% *************************************************************************
% Bierman KF 
%
% IS = (R + H * P * H');
% K = (P * H') / (IS) ;
% xu = K*y;
% *************************************************************************

l = max(size(R));

for j = 1:l

    h = H(j,:); 
    r = R(j,j);
    z = y(j);
   
     [xpo, Upo, Dpo] = bierman(z, r, h, xpr, Upr, diag(dpr));      
     dpo = diag(Dpo);

    xpr=xpo; Upr=Upo; dpr=dpo;
end

% *************************************************************************
% Thornton KF
%
% Pp = A * Pu * A' + Qd;
% Pp =  0.5 * (Pp + Pp');
% *************************************************************************

n = max(size(xpr));

if (isa(Upr,'single'))   
    
    I = single(eye(n));    
else
    
    I = single(eye(n));    
end

A = single(I + (F*dt));
Qd = G * Q * G' * dt;

[Upr, Dpr, ~] = thornton(A, G, Q*dt, Upo, diag(dpo), xpo);
dpr = diag(Dpr);

end
