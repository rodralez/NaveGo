function qua = euler2qua(euler)
% euler2qua: converts from Euler angles to quaternion.
% 
% INPUT:
%   euler,	3x1 Euler angles (rad).
%
% OUTPUT:
%   qua,    4x1 quaternion.
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
%			Crassidis, J.L. and Junkins, J.L. (2011). Optimal Esti-
% mation of Dynamic Systems, 2nd Ed. Chapman and Hall/CRC, USA.
% Eq. 7.39, p. 458.
%
% Version: 003
% Date:    2016/11/26
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% Rearrange vector for ZYX rotation sequence
euler = [euler(3) euler(2) euler(1)];

c_eul = cos( euler./2 );
s_eul = sin( euler./2 );

% ZYX rotation sequence
q = [ c_eul(1).*c_eul(2).*c_eul(3) + s_eul(1).*s_eul(2).*s_eul(3), ...
      c_eul(1).*c_eul(2).*s_eul(3) - s_eul(1).*s_eul(2).*c_eul(3), ...
      c_eul(1).*s_eul(2).*c_eul(3) + s_eul(1).*c_eul(2).*s_eul(3), ...
      s_eul(1).*c_eul(2).*c_eul(3) - c_eul(1).*s_eul(2).*s_eul(3)];

% Quaternion format used in Crassidis' quaternion update.
qua = [q(2) q(3) q(4) q(1)]';

end
