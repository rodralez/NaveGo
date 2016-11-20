function [qua_n, DCMbn_n, euler] = att_update(w, DCMbn, qua, omega_ie_N, omega_en_N, dt, mode)
% att_update: updates attitude using quaternion or DCM.
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
%			Crassidis, J.L. and Junkins, J.L. (2011). Optimal Esti-
% mation of Dynamic Systems, 2nd Ed. Chapman and Hall/CRC, USA.
% Eq. 7.39, p. 458.
%
% Version: 002
% Date:    2016/10/25
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

%% Correct gyros output for Earth rate and Transport rate

w_bn = ( w - DCMbn' * (omega_ie_N + omega_en_N))';

if strcmp(mode, 'quaternion')
%% Quaternion update   
    qua_n   = qua_update(qua, w_bn, dt);
    qua_n   = qua_n/norm(qua_n);
    DCMbn_n = qua2dcm(qua_n);
    euler   = qua2euler(qua_n);
    
elseif strcmp(mode, 'dcm')
%% DCM update    
    
    euler   = w_bn * dt;
    DCMbn_n = dcm_update(DCMbn, euler);
    euler   = dcm2euler(DCMbn_n);
    qua_n   = euler2qua(euler);
    qua_n   = qua_n/norm(qua_n);
    
else
    error('att_update: no attitude update mode defined.')
end

end
