function navego_nav2csv(nav)
% navego_nav2csv: exports navigation data to a .csv file.
%
% INPUT
%   nav, INS/GNSS estimations.
%
% OUTPUT
%	.csv file, with the following columns: time (seconds), roll (deg), 
%       pitch (deg), yaw (deg), vel N (m/s), vel E (m/s), vel D (m/s),
%        latitude (deg), longitud (deg), altitude (m).                  
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
% References:
%
%
% Version: 002
% Date:    2021/12/09
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

R2D = (180/pi);     % Radians to degrees

% File name
if(is_octave)
    dtime = datestr(date, 'yymmdd_HHMMSS' ); 
else
    dtime = datetime('now','TimeZone','local','Format','yyMMdd-HHmm');
end

file_name = sprintf('navego_%s.csv', dtime);
  
% Headline
ht = 'time (seconds), ';
ha = 'roll (deg), pitch (deg), yaw (deg), ';
hv = 'vel N (m/s), vel E (m/s), vel D (m/s), ';
hp = 'latitude (deg), longitud (deg), altitude (m),';

% Navigation data
nav_data = [nav.t nav.roll*R2D nav.pitch*R2D nav.yaw*R2D ...
              nav.vel(:,1) nav.vel(:,2) nav.vel(:,3) ...
              nav.lat*R2D nav.lon*R2D nav.h  ]' ;

% Navigation data precision to be saved
pt = '%.6f, ';
pa = repmat('%.3f, ', 1, 3); 
pv = repmat('%.3f, ', 1, 3); 
pp = repmat('%.7f, ', 1, 2);
ph = '%.3f, ';
format_pattern = [pt pa pv pp ph '\n']; 

% Saving the .cvs file
fprintf('navego_nav2cvs: saving navigation data to file %s ... \n', file_name)

fd = fopen(file_name, 'w');

% Saving headline
fprintf(fd, '%s%s%s%s \n', ht, ha, hv, hp);

% Saving data
fprintf(fd, format_pattern , nav_data );    
                                                                        
fclose(fd);

end