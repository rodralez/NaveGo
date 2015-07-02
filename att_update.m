function [quanew, DCMbn_new, ang_v] = att_update(w, DCMbn_old, quaold, omega_ie_N, omega_en_N, dt ) 
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
% Version: 001
% Date:    2014/09/11
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

% Corrected gyros output
w_bn = ( w - DCMbn_old' * (omega_ie_N + omega_en_N))'; 

%% Quaternion version

quanew =    qua_update(quaold, w_bn, dt);
quanew =    quanew/norm(quanew);
DCMbn_new = qua2dcm(quanew); 
% ang_v =     qua2euler(quanew);
ang_v =     dcm2euler(DCMbn_new);

%% DCM version

% ang = w_bn * dt;
% DCMbn_new = dcm_update(DCMbn_old, ang);
% ang_v = dcm2eulr(DCMbn_new);
% quanew = euler2qua(ang_v);

end
