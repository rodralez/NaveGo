function ref_i = interpolate(ref, base)
% interpolate: interpolates ref values using base time vector.
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

dt_r = mean(diff(ref.t));
dt_b = mean(diff(base.t));

% Test attitude vectors
fields = nnz(isfield(ref, {'roll','pitch','yaw'}));

MAX = max(size(base.t));

% Allocate variable space
if (isa(base.t,'single')) 
    
    if fields == 3
        ref_i.roll  = single(zeros (MAX,1));
        ref_i.pitch = single(zeros (MAX,1));
        ref_i.yaw   = single(zeros (MAX,1));
        ref_i.DCMnb = single(zeros (MAX,9));
    end

    ref_i.vel = single(zeros (MAX,3));
    ref_i.h   = single(zeros (MAX,1));    
else
    
    if fields == 3
    ref_i.roll  = zeros (MAX,1);
    ref_i.pitch = zeros (MAX,1);
    ref_i.yaw   = zeros (MAX,1);
    ref_i.DCMnb = zeros (MAX,9);
    end

    ref_i.vel = zeros (MAX,3);
    ref_i.h   = zeros (MAX,1);
end

ref_i.lat = zeros (MAX,1);
ref_i.lon = zeros (MAX,1);

ref_i.t   = base.t;

%%
% Normal interpolation
if (dt_b < dt_r),
    
    gidx = 2;

    if fields == 3

        for i = 1:MAX

            if (base.t(i) > ref.t(gidx))

                gidx = gidx + 1;
            end

            if isempty(gidx)
                fprintf('i = %d \n', i);
                error ('interpolate: gidx empty');
            end

            td   = ref.t(gidx) - ref.t(gidx-1); 
            tdin = base.t(i) - ref.t(gidx-1);

            ref_i.roll(i)  = interpolation (ref.roll(gidx-1), ref.roll(gidx),  td, tdin);
            ref_i.pitch(i) = interpolation (ref.pitch(gidx-1),ref.pitch(gidx), td, tdin);
            ref_i.yaw(i)   = interpolation (ref.yaw(gidx-1),  ref.yaw(gidx),   td, tdin);
            dmc_nb = euler2dcm([ref_i.roll(i) ref_i.pitch(i) ref_i.yaw(i)]);
            ref_i.DCMnb(i,:) = reshape(dmc_nb,1,9); 
        end

    end

    gidx = 2;

    for i = 1:MAX

        if (base.t(i) > ref.t(gidx))

            gidx = gidx + 1;
        end

        if isempty(gidx)
            fprintf('i = %d \n', i);
            error ('interpolate: gidx empty');
        end

        td   = ref.t(gidx) - ref.t(gidx-1); 
        tdin = base.t(i) - ref.t(gidx-1);

        ref_i.vel(i,:) = interpolation (ref.vel(gidx-1,:),ref.vel(gidx,:),  td, tdin);
        ref_i.lat(i)   = interpolation (ref.lat(gidx-1),  ref.lat(gidx),    td, tdin);
        ref_i.lon(i)   = interpolation (ref.lon(gidx-1),  ref.lon(gidx),    td, tdin);
        ref_i.h(i)     = interpolation (ref.h(gidx-1),    ref.h(gidx),      td, tdin);   
    end
%%

% Find the ref.t closest value to base.t
elseif (dt_b > dt_r)
    
    for i = 1:MAX
		
        gidx = find( abs(base.t(i) - ref.t) == min (abs (base.t(i) - ref.t)) );
        
        ref_i.t(i)     = ref.t  (gidx);
        
        if fields == 3
            ref_i.roll(i)  = ref.roll (gidx);
            ref_i.pitch(i) = ref.pitch(gidx);
            ref_i.yaw(i)   = ref.yaw  (gidx);
            ref_i.DCMnb(i,:) = ref.DCMnb(gidx,:);
        end
    
        ref_i.vel(i,:) = ref.vel(gidx,:);
        ref_i.lat(i)   = ref.lat(gidx);
        ref_i.lon(i)   = ref.lon(gidx);
        ref_i.h(i)     = ref.h  (gidx);
    end
else
    
    ref_i = ref;
end

ref_i.kn = MAX;

end
