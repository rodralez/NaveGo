function [b_drift, t_corr] = allan_get_bdrift (tau, allan)
% allan_get_bdrift: finds bias instability values from Allan variance.
%
% INPUT
% - tau, Nx1 Allan variance time vector in seconds.
% - allan, Nx1 Allan variance vector.
% 
% OUTPUT
% - b_drift, bias instability values.
% - t_corr, correlation time values.
%
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
%			None.
%
% Version: 001
% Date:    2016/11/02
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

idx = find (allan == min(allan));   % Index for minimun value of AV.   

b_drift = allan(idx) ;              % BI. For gyro, rad-per-sec.
                                    % BI. For acc,  meters-per-sec^2.
t_corr  = tau(idx)  ;               % BI, correlation time, in seconds.  
