function S = skewm(v)
% skewm: forms a skew-symmetric matrix from a 3-elements vector.
%
%   INPUT:
%   v, vector.
%
%	OUTPUT:
%	S, 3x3 matrix.
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
% 			Farrell, J. (2008). Aided Navigation: GPS With High Rate
% Sensors. McGraw-Hill Professional, USA. Eq. B.15, p. 463.
%
% Version: 002
% Date:    2018/09/14
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

x = v(1);
y = v(2);
z = v(3);

S = [ 0 -z   y;
      z  0  -x;
     -y  x   0; ];

end
