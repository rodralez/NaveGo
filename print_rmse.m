function ref_imu = print_rmse (imu_e, ref_imu, gps, ref_g)

global R2D

fe = max(size(imu_e.t));
fr = max(size(ref_imu.t));

% Adjust ref size if it is bigger than estimates size
if (fe < fr)
    
    ref_imu.t     = ref_imu.t(1:fe, :);
    ref_imu.roll  = ref_imu.roll(1:fe, :);
    ref_imu.pitch = ref_imu.pitch(1:fe, :);
    ref_imu.yaw   = ref_imu.yaw(1:fe, :);
    ref_imu.vel   = ref_imu.vel(1:fe, :);
    ref_imu.lat   = ref_imu.lat(1:fe, :);
    ref_imu.lon   = ref_imu.lon(1:fe, :);
    ref_imu.h     = ref_imu.h(1:fe, :);
    ref_imu.DCMnb = ref_imu.DCMnb(1:fe, :);
end

[RN,RE] = radius(imu_e.lat(1), 'double');
LAT2M = (RN + double(imu_e.h(1)));                         % Lat to meters contant                       
LON2M = (RE + double(imu_e.h(1))) .* cos(imu_e.lat(1));   % Lon to meters contant

% INS/GPS attitude RMSE
RMSE_roll   = rmse (imu_e.roll ,  ref_imu.roll)  .* R2D;
RMSE_pitch  = rmse (imu_e.pitch,  ref_imu.pitch) .* R2D;
% RMSE_yaw    = rmse (imu1_e.yaw,   ref_imu.yaw).*r2d;
% Only compare those estimates that have a diff. < pi with respect to ref
idx = find ( abs(imu_e.yaw - ref_imu.yaw) < pi );
RMSE_yaw    = rmse (imu_e.yaw(idx),   ref_imu.yaw(idx)).* R2D;

% INS/GPS velocity RMSE
RMSE_vn     = rmse (imu_e.vel(:,1),  ref_imu.vel(:,1));
RMSE_ve     = rmse (imu_e.vel(:,2),  ref_imu.vel(:,2));
RMSE_vd     = rmse (imu_e.vel(:,3),  ref_imu.vel(:,3));

% INS/GPS position RMSE
RMSE_lat    = rmse (imu_e.lat, ref_imu.lat) .* LAT2M;
RMSE_lon    = rmse (imu_e.lon, ref_imu.lon) .* LON2M;
RMSE_h      = rmse (imu_e.h,         ref_imu.h);

[RN,RE] = radius(gps.lat(1), 'double');
LAT2M = (RN + double(gps.h(1)));                        % Lat to meters contant 
LON2M = (RE + double(gps.h(1))) .* cos(gps.lat(1));     % Lon to meters contant

% GPS RMSE
RMSE_lat_g  = rmse (gps.lat, ref_g.lat) .* LAT2M;
RMSE_lon_g  = rmse (gps.lon, ref_g.lon) .* LON2M;
RMSE_h_g    = rmse (gps.h-gps.larm(3), ref_g.h);
RMSE_vn_g   = rmse (gps.vel(:,1),   ref_g.vel(:,1));
RMSE_ve_g   = rmse (gps.vel(:,2),   ref_g.vel(:,2));
RMSE_vd_g   = rmse (gps.vel(:,3),   ref_g.vel(:,3));

% Print RMSE
fprintf( '\n>> RMSE for IMU1\n');

fprintf( ' Roll,  IMU1 = %.4e deg.\n', ...
    RMSE_roll);
fprintf( ' Pitch, IMU1 = %.4e deg.\n', ...
    RMSE_pitch);
fprintf( ' Yaw,   IMU1 = %.4e deg.\n\n', ...
    RMSE_yaw);

fprintf( ' Vel. N, IMU1 = %.4e m/s, GPS = %.4e. m/s\n', ...
    RMSE_vn, RMSE_vn_g);
fprintf( ' Vel. E, IMU1 = %.4e m/s, GPS = %.4e. m/s\n', ...
    RMSE_ve, RMSE_ve_g);
fprintf( ' Vel. D, IMU1 = %.4e m/s, GPS = %.4e. m/s\n\n', ...
    RMSE_vd, RMSE_vd_g);

fprintf( ' Latitude,  IMU1 = %.4e m, GPS = %.4e. m\n', ...
    RMSE_lat, RMSE_lat_g);
fprintf( ' Longitude, IMU1 = %.4e m, GPS = %.4e. m\n', ...
    RMSE_lon, RMSE_lon_g);
fprintf( ' Altitude,  IMU1 = %.4e m, GPS = %.4e. m\n', ...
    RMSE_h, RMSE_h_g);

