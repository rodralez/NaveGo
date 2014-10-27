function ang_v = dcm2euler(DCMbn)
% dcm2euler: converts from DCM to euler angles.
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
% Eq. 15.
%
% Reference: D. H. Titterton and J. L. Weston, Strapdown Inertial 
% Navigation Technology. USA: Institution of Engineering and Technology, 
% 2nd ed., 2004. Eq. 3.66, pag 46.

phi =   atan( DCMbn(3,2) ./ DCMbn(3,3) ); % C_32 / C_33
theta = asin(-DCMbn(3,1) ); % - C_31
psi =   atan2( DCMbn(2,1), DCMbn(1,1) );  % C_21 / C_11

ang_v = [phi theta psi];

end
