function [ref_ds] = downsampling (ref, dt_b)
% downsampling: downsample ref using the time step dt_b
%
%   Copyright (C) 2014, Rodrigo Gonzalez, all rights reserved. 
%     
%   This file is part of NaveGo, an open-source MATLAB toolbox for 
%   the simulation of integrated navigation systems.
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
% Version: 002
% Date:    2015/08/20
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

dt_r = mean(diff(ref.t));

if (dt_b >= dt_r)

    dspl = floor(dt_b / dt_r);
else
    
    error('downsampling: dt_r > dt_b')    
end

if (isfield(ref, 'fb')) 
    ref_ds.fb  = ref.fb (1:dspl:end, :);
    ref_ds.wb  = ref.wb(1:dspl:end, :);
end

ref_ds.roll  = ref.roll (1:dspl:end, :);
ref_ds.pitch = ref.pitch(1:dspl:end, :);
ref_ds.yaw   = ref.yaw  (1:dspl:end, :);
ref_ds.DCMnb = ref.DCMnb(1:dspl:end, :);

ref_ds.t     = ref.t    (1:dspl:end, :);
ref_ds.lat   = ref.lat  (1:dspl:end, :);
ref_ds.lon   = ref.lon  (1:dspl:end, :);
ref_ds.h     = ref.h    (1:dspl:end, :);
ref_ds.vel   = ref.vel  (1:dspl:end, :);

ref_ds.kn = max(size(ref_ds.t));
ref_ds.freq = round(1/mean(diff(ref_ds.t)));

end
