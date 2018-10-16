function rmse_v = print_rmse (ins_gnss, gnss, ins_gnss_r, gnss_r, string)
% print_rmse: print on console Root Mean Squared Errors (RMSE) between INS/GNSS
% and a reference, and between GNSS-only and a reference as well.
%
% INPUT:
%   ins_gnss, INS/GNSS data structure.
%   gnss, GNSS data structure.
%   ins_gnss_r, Reference data structure ajusted for INS/GNSS measurements.
%   gnss_r, Reference data structure ajusted for GNSS measurements.
%   string, string to print on console identifying the RMSE.
%
% OUTPUT
%   rmse_v, vector with all RMSE.
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
% Version: 006
% Date:    2018/10/10
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

rmse_v = navego_rmse (ins_gnss, gnss, ins_gnss_r, gnss_r);

% rmse_v = [  RMSE_roll;  RMSE_pitch; RMSE_yaw;    
%             RMSE_vn;    RMSE_ve;    RMSE_vd;
%             RMSE_lat;   RMSE_lon;   RMSE_h;
%             RMSE_vn_g;  RMSE_ve_g;  RMSE_vd_g;
%             RMSE_lat_g; RMSE_lon_g; RMSE_h_g; ];
        
%% Print RMSE

fprintf( '\n>> RMSE for %s\n\n', string);

fprintf(' Roll,  %s = %.4e deg \n',   string, rmse_v(1));
fprintf(' Pitch, %s = %.4e deg \n',   string, rmse_v(2));
fprintf(' Yaw,   %s = %.4e deg \n\n', string, rmse_v(3));

if (isfield(ins_gnss, 'vel') & isfield( ins_gnss_r, 'vel') & isfield(gnss, 'vel') & isfield( gnss_r, 'vel'))
    fprintf(' Vel. N, %s = %.4e m/s, GNSS = %.4e m/s \n',   string, rmse_v(4), rmse_v(10));
    fprintf(' Vel. E, %s = %.4e m/s, GNSS = %.4e m/s \n',   string, rmse_v(5), rmse_v(11));
    fprintf(' Vel. D, %s = %.4e m/s, GNSS = %.4e m/s \n\n', string, rmse_v(6), rmse_v(12));
end

fprintf(' Latitude,  %s = %.4e m, GNSS = %.4e m \n', string, rmse_v(7), rmse_v(13));
fprintf(' Longitude, %s = %.4e m, GNSS = %.4e m \n', string, rmse_v(8), rmse_v(14));
fprintf(' Altitude,  %s = %.4e m, GNSS = %.4e m \n', string, rmse_v(9), rmse_v(15));

end
