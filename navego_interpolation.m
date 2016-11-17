function ref_i = navego_interpolation (data, ref)
% navego_interpolation: interpolates reference dataset using INS/GPS time vector
% or GPS time vector.
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
% Version: 001
% Date:    2016/11/17
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

MI = max(size(data.t));
MR = max(size(ref.t));

if (MR/2 > MI)
    method = 'linear';
else
    method = 'nearest';
end

if (isfield(data, 'roll'))  % If dataset is from INS/GPS solution...
    
    fprintf('navego_interpolation: %s method to interpolate INS/GPS solution\n', method)
    
    ref_i.t     = data.t;
    ref_i.roll  = interp1(ref.t, ref.roll,  data.t, method);
    ref_i.pitch = interp1(ref.t, ref.pitch, data.t, method);
    ref_i.yaw   = interp1(ref.t, ref.yaw,   data.t, method);
    ref_i.lat   = interp1(ref.t, ref.lat,   data.t, method);
    ref_i.lon   = interp1(ref.t, ref.lon,   data.t, method);
    ref_i.h     = interp1(ref.t, ref.h,     data.t, method);
    ref_i.vel   = interp1(ref.t, ref.vel,   data.t, method);
    
    if( any(isnan(ref_i.t)) | any(isnan(ref_i.roll)) | any(isnan(ref_i.pitch)) | any(isnan(ref_i.yaw)) |  ...
            any(isnan(ref_i.lat)) | any(isnan(ref_i.lon)) | any(isnan(ref_i.h)) | any(isnan(ref_i.vel)) )
        
        error('navego_interpolation: NaN value in INS/GPS interpolated solution')
    end
    
else                        % If dataset is from GPS-only solution...
    
    fprintf('navego_interpolation: %s method to interpolate GPS solution\n', method)
    
    ref_i.t   = data.t;
    ref_i.lat = interp1(ref.t, ref.lat, data.t, method);
    ref_i.lon = interp1(ref.t, ref.lon, data.t, method);
    ref_i.h   = interp1(ref.t, ref.h,   data.t, method);
    ref_i.vel = interp1(ref.t, ref.vel, data.t, method);
    
    if( any(isnan(ref_i.t)) | any(isnan(ref_i.lat)) | any(isnan(ref_i.lon)) | any(isnan(ref_i.vel)) | any(isnan(ref_i.h)) )
        
        error('navego_interpolation: NaN value in GPS interpolated solution')
    end
end
