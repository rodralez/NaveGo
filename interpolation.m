function v_int = interpolation(v1, v2, dt, dt_v12)
% interpolation: obtains the interpolated value between v1 and v2, using
%  dt_v12, the time difference between v2 and v1, and dt simulation  
%  step.
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
% Reference:

if dt_v12 < 0,  error('interpolation: dt_v12 is negative'), end
if dt == 0,     error('interpolation: dt equal to zero'), end
if dt_v12 > dt, error('interpolation: dt_v12 must be >= td'), end

v_int = v1 + ( (v2 - v1)  .* dt_v12 ./ dt );
