function [nav_i, ref] = navego_interpolation (nav, ref)
% navego_interpolation: interpolates navigation data using a reference time
% vector.
%
% INPUT
%   nav, navigation data structure to be interpolated.
%   ref, reference data structure.
%
% OUTPUT
%   nav_i, navigation data structure interpolated by reference time vector.
%   ref,   reference data structure adjusted by the nav time vector.
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
% Version: 004
% Date:    2019/01/10
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

D = max(size(nav.t));
R = max(size(ref.t));

if (D > R)
    
    method = 'nearest';
else
    
    method = 'spline';
end

%% Adjust reference data structure before interpolating

if (ref.t(1) < nav.t(1))
    
    fprintf('navego_interpolation: adjusting first element of ref ... \n')
    
    idx  = find(ref.t >= nav.t(1), 1, 'first' );
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

if (ref.t(end) > nav.t(end))
    
    fprintf('navego_interpolation: adjusting last element of ref... \n')
    
    idx  = 1;
    fdx  = find(ref.t <= nav.t(end), 1, 'last' );
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

%% Interpolation

% If data is from INS/GNSS solution...

if (isfield(nav, 'roll') & isfield(ref, 'roll'))
    
    fprintf('navego_interpolation: %s method to interpolate INS/GNSS solution\n', method)
    
    nav_i.t     = ref.t;
    nav_i.roll  = interp1(nav.t, nav.roll,  ref.t, method);
    nav_i.pitch = interp1(nav.t, nav.pitch, ref.t, method);
    nav_i.yaw   = interp1(nav.t, nav.yaw,   ref.t, method);
    nav_i.lat   = interp1(nav.t, nav.lat,   ref.t, method);
    nav_i.lon   = interp1(nav.t, nav.lon,   ref.t, method);
    nav_i.h     = interp1(nav.t, nav.h,     ref.t, method);
    
    if (isfield(ref, 'vel') & isfield( nav, 'vel'))
        
        nav_i.vel = interp1(nav.t, nav.vel,   ref.t, method);
        flag_vel  = any(isnan(nav_i.vel));
    else
        flag_vel = false(1,3);
    end
    
    flag = any(isnan(nav_i.t)) | any(isnan(nav_i.roll)) | any(isnan(nav_i.pitch)) | any(isnan(nav_i.yaw)) |  ...
        any(isnan(nav_i.lat)) | any(isnan(nav_i.lon)) | any(isnan(nav_i.h)) | flag_vel;
    
    % Test interpolated dataset
    if(flag)
        
        error('navego_interpolation: NaN value in INS/GNSS interpolated solution')
    end
    
% If dataset is from a GNSS-only solution...

else
    
    fprintf('navego_interpolation: %s method to interpolate GNSS-only solution\n', method)
    
    nav_i.t   = ref.t;
    nav_i.lat = interp1(nav.t, nav.lat, ref.t, method);        nav_i.vel = interp1(nav.t, nav.vel,   ref.t, method);

    nav_i.lon = interp1(nav.t, nav.lon, ref.t, method);
    nav_i.h   = interp1(nav.t, nav.h,   ref.t, method);
    
    if (isfield(ref, 'vel') & isfield( nav, 'vel'))
        
        nav_i.vel = interp1(nav.t, nav.vel,   ref.t, method);
        flag_vel = any(isnan(nav_i.vel));
    else
        flag_vel = false(1,3);
    end
    
    flag = any(isnan(nav_i.t)) | any(isnan(nav_i.lat)) | any(isnan(nav_i.lon)) | ...
        any(isnan(nav_i.h)) | flag_vel;
    
    % Test interpolated dataset
    if(flag)
        
        error('navego_interpolation: NaN value in GNSS-only interpolated solution')
    end
end

end
