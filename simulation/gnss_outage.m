function nav_o = gnss_outage(nav, times_out, ACTION)
% gnss_outage: removes or gets navigation data from navigation data structure
% in GNSS outages periods.
%
% INPUT
%   nav, navigation data structure.
%   times_out, Nx1 vector with outage times where times_out(1) is the start
%     time of 1st GNSS OUTAGE, times_out(2) is the end of 1st GNSS OUTAGE,
%     times_out(3) is the start of the 2nd GNSS OUTAGE, and so on. N should
%     be even.
%   ACTION, 'REMOVE' (default) or 'GET' (string).
%
% OUTPUT
%   nav, same navigation data structure.
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
% Date:    2021/12/14
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

if (nargin < 3)
    
    ACTION = 'REMOVE';
end

OUTAGE_N = max(size(times_out)) / 2;

%% INIT DATA

if (strcmp(ACTION,'REMOVE'))
    
    fprintf('gnss_outage: %d GNSS outages are forced... \n', OUTAGE_N)
    
    nav_o = nav;
    
elseif(strcmp(ACTION,'GET'))
    
    nav_o.t = [];
    nav_o.lat = [];
    nav_o.lon = [];
    nav_o.h = [];
    nav_o.vel = [];
    
    if (isfield(nav,'roll'))
        nav_o.roll = [];
    end
    
    if (isfield(nav,'pitch'))
        nav_o.pitch = [];
    end
    
    if (isfield(nav,'yaw'))
        nav_o.yaw = [];
    end
else
    
    error('gnss_outage: action is not defined')
end

%% REMOVE OR GET

for k = 1:OUTAGE_N
    
    if (strcmp(ACTION,'REMOVE'))
        
        idx  = find(nav_o.t >= times_out(2*k-1), 1, 'first' );
        fdx  = find(nav_o.t <= times_out(2*k), 1, 'last' );
        
        if(isempty(idx) || isempty(fdx))
            error('gnss_outage: outage empty index')
        end
        
        if(idx >= fdx)
            error('gnss_outage: indexes inconsistency')
        end
        
        nav_o.t  (idx:fdx)    = [];
        nav_o.lat(idx:fdx)    = [];
        nav_o.lon(idx:fdx)    = [];
        nav_o.h  (idx:fdx)    = [];
        nav_o.vel(idx:fdx, :) = [];
        
        if (isfield(nav,'roll'))
            nav_o.roll(idx:fdx)  = [];
        end
        
        if (isfield(nav,'pitch'))
            nav_o.pitch(idx:fdx) = [];
        end
        
        if (isfield(nav,'yaw'))
            nav_o.yaw(idx:fdx)   = [];
        end
        
    elseif(strcmp(ACTION,'GET'))
        
        idx  = find(nav.t >= times_out(2*k-1), 1, 'first' );
        fdx  = find(nav.t <= times_out(2*k), 1, 'last' );
        
        if(isempty(idx) || isempty(fdx))
            error('gnss_outage: outage empty index')
        end
        
        if(idx >= fdx)
            error('gnss_outage: indexes inconsistency')
        end
        
        nav_o.t     = [nav_o.t   ; nav.t(idx:fdx)] ;
        nav_o.lat   = [nav_o.lat ; nav.lat(idx:fdx)];
        nav_o.lon   = [nav_o.lon ; nav.lon(idx:fdx)];
        nav_o.h     = [nav_o.h   ; nav.h(idx:fdx) ];
        nav_o.vel   = [nav_o.vel ; nav.vel(idx:fdx, :)];
        
        if (isfield(nav,'roll'))
            nav_o.roll = [nav_o.roll ; nav.roll(idx:fdx)];
        end
        
        if (isfield(nav,'pitch'))
            nav_o.pitch = [nav_o.pitch ; nav.pitch(idx:fdx)];
        end
        
        if (isfield(nav,'yaw'))
            nav_o.yaw = [nav_o.yaw ; nav.yaw(idx:fdx)];
        end
        
    else
        
        error('gnss_outage: action is not defined')
    end
    
end

end

