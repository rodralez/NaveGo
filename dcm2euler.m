function euler = dcm2euler(DCMbn)
% dcm2euler: converts from DCM to euler angles.
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
% 			Titterton, D.H. and Weston, J.L. (2004). Strapdown
% Inertial Navigation Technology (2nd Ed.). Institution
% of Engineering and Technology, USA. Eq. 11.4. Eq. 3.66, p. 46.
%
%			R. Gonzalez, J. Giribet, and H.D. Patiño,. An approach to
% benchmarking of loosely coupled low-cost navigation systems,
% Mathematical and Computer Modelling of Dynamical Systems, vol. 21,
% issue 2, pp. 272-287. Eq. 15.
%
% Version: 001
% Date:    2014/09/11
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

phi =   atan( DCMbn(3,2) ./ DCMbn(3,3) ); % C_32 / C_33
theta = asin(-DCMbn(3,1) ); % - C_31
psi =   atan2( DCMbn(2,1), DCMbn(1,1) );  % C_21 / C_11

euler = [phi theta psi];

end
