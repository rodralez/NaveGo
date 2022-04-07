function  [F, G] = F_update(upd, DCMbn, imu)
% F_update: updates F and G matrices before the execution of Kalman filter.
%
% INPUT
%   upd, 1x8 vector with data from the INS.
%   DCMbn, DCM body-to-nav.
%   imu, IMU data structure.
%
% OUTPUT
%   F,  15x15 state transition matrix.
%   G,  15x12 control-input matrix.
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
%   Groves, P.D. (2013), Principles of GNSS, Inertial, and
% Multisensor Integrated Navigation Systems (2nd Ed.). Artech House. 
% Matrix F from Eq. 14.63.
%
% 	Farrell, J. (2008). Aided Navigation: GPS With High Rate
% Sensors. McGraw-Hill Professional, USA. Matrix G from Eq. 11.108.
%
% Version: 010
% Date:    2022/03/06
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

Vn =  upd(1);
Ve =  upd(2);
Vd =  upd(3);
lat = upd(4);
h  =  upd(5);

fn =  upd(6:8);
% wn =  upd(9:11);

Om = 7.292115e-5;
I = eye(3);
Z = zeros(3);

[RM,RN] = radius(lat);

RO = sqrt(RN*RM) + h;

%% ATTITUDE MATRICES

a11 = 0;
a12 = -( (Om * sin(lat)) + (Ve / RO * tan(lat)) );
a13 = Vn / RO;
a21 = (Om * sin(lat)) + (Ve / RO * tan(lat));
a22 = 0 ;
a23 = (Om * cos(lat)) + (Ve / RO) ;
a31 = -Vn / RO;
a32 = -Om * cos(lat) - (Ve / RO);
a33 = 0;
F11 = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

% Groves, 14.64
% F11 = skewm(wn);

a11 = 0;
a12 = 1 / RO;
a13 = 0;
a21 = -1 / RO;
a22 = 0;
a23 = 0;
a31 = 0;
a32 = -tan(lat) / RO;
a33 = 0;
F12 = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

a11 = -Om * sin(lat);
a12 = 0;
a13 = -Ve / (RO^2);
a21 = 0 ;
a22 = 0 ;
a23 = Vn / (RO^2);
a31 =  -Om * cos(lat) - (Ve / ((RO) * (cos(lat))^2));
a32 = 0 ;
a33 = (Ve * tan(lat)) / (RO^2) ;
F13 = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

%% VELOCITY MATRICES

F21 = skewm(fn);

a11 = Vd / RO;
a12 = -2 * ((Om * sin(lat)) + ((Ve / RO) * tan(lat))) ;
a13 = Vn / RO ;
a21 = (2 * Om * sin(lat)) + ( (Ve / RO) * tan(lat) );
a22 = (1 / RO) * ((Vn * tan(lat)) + Vd) ;
a23 = 2 * Om * cos(lat) + (Ve / RO);
a31 = (-2 * Vn) / RO;
a32 = -2 * (Om * cos(lat) +  (Ve / RO)) ;
a33 = 0;
F22 = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

e = 0.0818191908425;        % WGS84 eccentricity
res = RN * sqrt( cos(lat)^2 + (1-e^2)^2 * sin(lat)^2);
g = gravity(lat,h);
g0 = g(3);

a11 = -Ve * ((2 * Om * cos(lat)) + (Ve / (RO * (cos(lat))^2)));
a12 = 0 ;
a13 = (1 / RO^2) * ( (Ve^2 * tan(lat)) - (Vn * Vd) );
a21 = 2 * Om * ( (Vn * cos(lat)) - (Vd * sin(lat)) ) + ( (Vn * Ve) / (RO * (cos(lat))^2) ) ;
a22 = 0 ;
a23 = -(Ve / RO^2) * (Vn * tan(lat) + Vd);
a31 = 2 * Om * Ve * sin(lat);
a32 = 0;
% a33 = (1 / RO^2) * (Vn^2 + Ve^2);
a33 = Ve^2 / (RN+h)^2 + Vn^2 / (RM+h)^2 - 2 * g0 / res;
F23 = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

%% POSITIONING MATRICES

F31 = zeros(3);

a11 = 1 / RO;
a12 = 0;
a13 = 0;
a21 = 0;
a22 = 1 / (RO * cos(lat));
a23 = 0;
a31 = 0;
a32 = 0;
a33 = -1;
F32 = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

a11 = 0;
a12 = 0;
a13 = -Vn / RO^2;
a21 = (Ve * tan(lat)) / (RO * cos(lat));
a22 = 0;
a23 = -Ve / (RO^2 * cos(lat));
a31 = 0;
a32 = 0;
a33 = 0;
F33 = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

Fbg = I;
Fba = I;

if (isinf(imu.gb_corr))
    Fgg = Z;
else
    Fgg = -diag( 1./ imu.gb_corr);
    %     Fbg = -diag(sqrt (2 ./ imu.gb_corr .* imu.gb_dyn.^2));
end

if (isinf(imu.ab_corr))
    Faa = Z;
else
    Faa = -diag(1 ./ imu.ab_corr);
    %     Fba = -diag(sqrt (2 ./ imu.ab_corr .* imu.ab_dyn.^2));
end

% Eq. 14.63 from Groves
F = [F11 F12 F13 DCMbn Z  ;
    F21  F22 F23 Z     DCMbn  ;
    F31  F32 F33 Z     Z      ;
    Z    Z   Z   Fgg   Z      ;
    Z    Z   Z   Z     Faa    ;
    ];

% Eq. 11.108 from Farrell
G = [DCMbn Z     Z   Z ;
    Z      DCMbn Z   Z ;
    Z      Z     Z   Z ;
    Z      Z     Fbg Z ;
    Z      Z     Z   Fba ;
    ];

end
