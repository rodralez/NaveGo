function DCMbn = qua2dcm(qua_vec)
% qua2dcm: transforms quaternion to DCM.
%
%   Copyright (C) 2014, Rodrigo González, all rights reserved. 
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
% Version: 001
% Date:    2014/09/11
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 
%
% Reference: R. González, J. Giribet, and H. Patiño, “An approach to 
% benchmarking of loosely coupled low-cost navigation systems,” 
% Mathematical and Computer Modelling of Dynamical Systems, Sept. 2014.
% Eq. 14.
  
a = qua_vec(4); b = qua_vec(1); c = qua_vec(2); d = qua_vec(3);

DCMbn(1,1) = a*a + b*b - c*c - d*d;
DCMbn(1,2) = 2*(b*c - a*d);
DCMbn(1,3) = 2*(a*c + b*d);
DCMbn(2,1) = 2*(a*d + b*c);
DCMbn(2,2) = a*a - b*b + c*c - d*d;
DCMbn(2,3) = 2*(c*d - a*b);  
DCMbn(3,1) = 2*(b*d - a*c);
DCMbn(3,2) = 2*(c*d + a*b);
DCMbn(3,3) = a*a - b*b - c*c + d*d;

end
