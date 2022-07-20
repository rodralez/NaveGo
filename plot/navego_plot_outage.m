function navego_plot_outage (nav_t, nav_var, OUTAGE, times_out)
% navego_plot_outage: plots GNSS outages periods.
%
% INPUT
%   nav_t,   Nx1 vector time.
%   nav_var, Nx1 navigation variable.
%   OUTAGE, 'ON' or 'OFF' (string).
%   times_out, Tx1 vector with outage times where times_out(1) is the start
%     time of 1st GNSS OUTAGE, times_out(2) is the end of 1st GNSS OUTAGE,
%     times_out(3) is the start of the 2nd GNSS OUTAGE, and so on. T should
%     be even.
%
% OUTPUT
%   One figure.
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
    
    OUTAGE = 'OFF';
end

%% PARAMETERS

plot_parameters;

%% OUTAGES
    
if (strcmp(OUTAGE,'ON'))

    OUTAGE_N = max(size(times_out)) / 2;
   
    for k = 1:OUTAGE_N
        
        idx  = find(nav_t >= times_out(2*k-1), 1, 'first' );
        fdx  = find(nav_t <= times_out(2*k), 1, 'last' );
     
        plot(nav_t(idx:fdx), nav_var(idx:fdx), '.r', 'MarkerSize', ms, ...
            'LineWidth', lw, 'DisplayName','GNSS OUTAGE') ;
    end
end