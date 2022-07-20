function v = skewm_inv(S)
% skewm_inv: returns a 3-elements vector from a skew-symmetric matrix.
%
% INPUT
%    S: 3x3 skew-symmetric matrix.
%
%	OUTPUT
%	  v: 3x1 vector.
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
%   Farrell, J. (2008). Aided Navigation: GPS With High Rate
% Sensors. McGraw-Hill Professional, USA. Eq. B.15, p. 463.
%
% Version: 001
% Date:    2022/03/06
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% S = [ 0 -z   y;
%       z  0  -x;
%      -y  x   0; ];
 
x = S(3,2);
y = S(1,3);
z = S(2,1);

v = [x y z]';

end
