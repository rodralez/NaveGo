function DCMbn = qua2dcm(qua)
% qua2dcm: transforms quaternion to body-to-nav DCM.
%
% INPUT:
%   qua,	4x1 quaternion.
%
% OUTPUT:
%   DCMbn,	3x3 body-to-nav DCM.
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
%			Titterton, D.H. and Weston, J.L. (2004). Strapdown
% Inertial Navigation Technology (2nd Ed.). Institution
% of Engineering and Technology, USA. Eq. 3.63.
%
%  			R. Gonzalez, J. Giribet, and H. Patiño. An approach to
% benchmarking of loosely coupled low-cost navigation systems,
% Mathematical and Computer Modelling of Dynamical Systems, vol. 21,
% issue 3, pp. 272-287, 2015. Eq. 14.
%
%			Crassidis, J.L. and Junkins, J.L. (2011). Optimal Esti-
% mation of Dynamic Systems, 2nd Ed. Chapman and Hall/CRC, USA.
% Eq. 7.39, p. 458.
%
% Version: 002
% Date:    2016/11/26
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% Quaternion format used in Crassidis' quaternion equations.
a = qua(4); b = qua(1); c = qua(2); d = qua(3);

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
