function [ref_i, ref] = navego_interpolation (data, ref)
% navego_interpolation: interpolates data using reference time vector.
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
% Version: 003
% Date:    2017/05/18
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

D = max(size(data.t));
R = max(size(ref.t));

if (D > R)
    
    method = 'nearest';
else
    
    method = 'linear';
end

%% Adjust reference data structure before interpolating

if (ref.t(1) < data.t(1))
    
    fprintf('navego_interpolation: adjusting first element of ref ... \n')
    
    idx  = find(ref.t >= data.t(1), 1, 'first' );
    if(isempty(idx))
        error('navego_interpolation: idx empty index.')
    end
    
    ref.t   = ref.t  (idx:end);
    ref.lat = ref.lat(idx:end);
    ref.lon = ref.lon(idx:end);
    ref.h   = ref.h(  idx:end);
    
    if (isfield(ref, 'vel'))
        ref.vel = ref.vel(idx:end, :);
    end
    
    if (isfield(ref, 'roll'))
        ref.roll  = ref.roll (idx:end);
        ref.pitch = ref.pitch(idx:end);
        ref.yaw   = ref.yaw  (idx:end);
    end
end

if (ref.t(end) > data.t(end))
    
    fprintf('navego_interpolation: adjusting last element of ref... \n')
    
    idx  = 1;
    fdx  = find(ref.t <= data.t(end), 1, 'last' );
    if(isempty(fdx))
        error('navego_interpolation: fdx empty index.')
    end
    
    ref.t   = ref.t  (idx:fdx);
    ref.lat = ref.lat(idx:fdx);
    ref.lon = ref.lon(idx:fdx);
    ref.h   = ref.h  (idx:fdx);
    
    if (isfield( ref, 'vel'))
        ref.vel = ref.vel(idx:fdx, :);
    end
    
    if (isfield(ref, 'roll'))
        ref.roll  = ref.roll (idx:fdx);
        ref.pitch = ref.pitch(idx:fdx);
        ref.yaw   = ref.yaw  (idx:fdx);
    end
end

%% Interpolate

if (isfield(data, 'roll') & isfield(ref, 'roll'))  % If data is from INS/GPS solution...
    
    fprintf('navego_interpolation: %s method to interpolate INS/GPS solution\n', method)
    
    ref_i.t     = ref.t;
    ref_i.roll  = interp1(data.t, data.roll,  ref.t, method);
    ref_i.pitch = interp1(data.t, data.pitch, ref.t, method);
    ref_i.yaw   = interp1(data.t, data.yaw,   ref.t, method);
    ref_i.lat   = interp1(data.t, data.lat,   ref.t, method);
    ref_i.lon   = interp1(data.t, data.lon,   ref.t, method);
    ref_i.h     = interp1(data.t, data.h,     ref.t, method);
    
    if (isfield(ref, 'vel') & isfield( data, 'vel'))
        
        ref_i.vel = interp1(data.t, data.vel,   ref.t, method);
        flag_vel  = any(isnan(ref_i.vel));        
    else
        flag_vel = logical(zeros(1,3));
    end
    
    flag = any(isnan(ref_i.t)) | any(isnan(ref_i.roll)) | any(isnan(ref_i.pitch)) | any(isnan(ref_i.yaw)) |  ...
        any(isnan(ref_i.lat)) | any(isnan(ref_i.lon)) | any(isnan(ref_i.h)) | flag_vel;
    
    % Test interpolated dataset
    if(flag)
        
        error('navego_interpolation: NaN value in INS/GPS interpolated solution')
    end
    
else  % If dataset is from GPS-only solution...
    
    fprintf('navego_interpolation: %s method to interpolate GPS-only solution\n', method)
    
    ref_i.t   = ref.t;
    ref_i.lat = interp1(data.t, data.lat, ref.t, method);
    ref_i.lon = interp1(data.t, data.lon, ref.t, method);
    ref_i.h   = interp1(data.t, data.h,   ref.t, method);
    
    if (isfield(ref, 'vel') & isfield( data, 'vel'))
        
        ref_i.vel = interp1(data.t, data.vel,   ref.t, method);        
        flag_vel = any(isnan(ref_i.vel));        
    else
        flag_vel = logical(zeros(1,3));
    end
    
    flag = any(isnan(ref_i.t)) | any(isnan(ref_i.lat)) | any(isnan(ref_i.lon)) | ...
        any(isnan(ref_i.h)) | flag_vel;
    
    % Test interpolated dataset
    if(flag)
        
        error('navego_interpolation: NaN value in GPS-only interpolated solution')
    end
end

end
