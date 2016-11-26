function [ref_i, data] = navego_interpolation (data, ref)
% navego_interpolation: interpolates data using INS/GPS time vector
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

method = 'linear';

% Adjust data structure before interpolating
if (data.t(1) < ref.t(1))
    
    fprintf('navego_interpolation: adjusting first element of data ... \n')
    
    idx  = find(data.t >= ref.t(1), 1, 'first' );
    if(isempty(idx))
        error('navego_interpolation: idx empty index.')
    end
    
    data.t   = data.t(idx:end);
    data.lat = data.lat(idx:end);
    data.lon = data.lon(idx:end);
    data.h   = data.h(idx:end);
    
    if (isfield(data, 'vel'))
        data.vel = data.vel(idx:end, :);
    end
    
    if (isfield(data, 'roll'))
        data.roll = data.roll(idx:end);
        data.pitch = data.pitch(idx:end);
        data.yaw   = data.yaw(idx:end);
    end
end

if (data.t(end) > ref.t(end))
    
    fprintf('navego_interpolation: adjusting last element of data... \n')
    
    idx  = 1;
    fdx  = find(data.t <= ref.t(end), 1, 'last' );
    if(isempty(fdx))
        error('navego_interpolation: fdx empty index.')
    end
    
    data.t   = data.t(idx:fdx);
    data.lat = data.lat(idx:fdx);
    data.lon = data.lon(idx:fdx);
    data.h   = data.h(idx:fdx);
    
    if (isfield( data, 'vel'))
        data.vel = data.vel(idx:fdx, :);
    end
    
    if (isfield(data, 'roll'))
        data.roll = data.roll(idx:fdx);
        data.pitch = data.pitch(idx:fdx);
        data.yaw   = data.yaw(idx:fdx);
    end
end

% Interpolate
if (isfield(data, 'roll') & isfield(ref, 'roll'))  % If data is from INS/GPS solution...
    
    fprintf('navego_interpolation: %s method to interpolate INS/GPS solution\n', method)
    
    ref_i.t     = data.t;
    ref_i.roll  = interp1(ref.t, ref.roll,  data.t, method);
    ref_i.pitch = interp1(ref.t, ref.pitch, data.t, method);
    ref_i.yaw   = interp1(ref.t, ref.yaw,   data.t, method);
    ref_i.lat   = interp1(ref.t, ref.lat,   data.t, method);
    ref_i.lon   = interp1(ref.t, ref.lon,   data.t, method);
    ref_i.h     = interp1(ref.t, ref.h,     data.t, method);
    
    if (isfield( ref, 'vel') & isfield( data, 'vel'))
        ref_i.vel   = interp1(ref.t, ref.vel,   data.t, method);
        
        flag = any(isnan(ref_i.t)) | any(isnan(ref_i.roll)) | any(isnan(ref_i.pitch)) | any(isnan(ref_i.yaw)) |  ...
            any(isnan(ref_i.lat)) | any(isnan(ref_i.lon)) | any(isnan(ref_i.h)) | any(isnan(ref_i.vel));
    else
        flag = any(isnan(ref_i.t)) | any(isnan(ref_i.roll)) | any(isnan(ref_i.pitch)) | any(isnan(ref_i.yaw)) |  ...
            any(isnan(ref_i.lat)) | any(isnan(ref_i.lon)) | any(isnan(ref_i.h));
    end
    
    % Test interpolated dataset
    if(flag)
        
        error('navego_interpolation: NaN value in INS/GPS interpolated solution')
    end
    
else                        % If dataset is from GPS-only solution...
    
    fprintf('navego_interpolation: %s method to interpolate GPS solution\n', method)
    
    ref_i.t   = data.t;
    ref_i.lat = interp1(ref.t, ref.lat, data.t, method);
    ref_i.lon = interp1(ref.t, ref.lon, data.t, method);
    ref_i.h   = interp1(ref.t, ref.h,   data.t, method);
    
    if (isfield(ref, 'vel') & isfield( data, 'vel'))
        ref_i.vel   = interp1(ref.t, ref.vel,   data.t, method);
        
        flag = any(isnan(ref_i.t)) | any(isnan(ref_i.lat)) | any(isnan(ref_i.lon)) | ...
            any(isnan(ref_i.h)) | any(isnan(ref_i.vel));
    else
        flag = any(isnan(ref_i.t)) | any(isnan(ref_i.lat)) | any(isnan(ref_i.lon)) | any(isnan(ref_i.h));
    end
    
    % Test interpolated dataset
    if(flag)
        
        error('navego_interpolation: NaN value in GPS interpolated solution')
    end
end

end

