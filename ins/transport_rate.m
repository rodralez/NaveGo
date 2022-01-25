function omega_en_n = transport_rate(lat, Vn, Ve, h)
% transport_rate: calculates the transport rate in the navigation frame.
%
% INPUT
%	lat, 1x1 latitude (rad).
%	Vn, 1x1 North velocity (m/s).
%   Ve, 1x1 East velocity (m/s).
%   h, altitude (m).
%
% OUTPUT
%	omega_en_n, 3x3 transport rate (rad/s).
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
%
%   Paul D. Groves. Principles of GNSS, Inertial, and
% Multisensor Integrated Navigation Systems. Second Edition.
% Eq. 5.44, page 177.
%
% Version: 002
% Date:    2022/01/25
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

h = abs(h);


[RM,RN] = radius(lat);

om_en_n(1,1) =   Ve / (RN + h);              % North
om_en_n(2,1) = -(Vn / (RM + h));             % East
om_en_n(3,1) = -(Ve * tan(lat) / (RN + h));  % Down

omega_en_n = skewm(om_en_n);

end
