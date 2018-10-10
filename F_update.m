function  [F, G] = F_update(upd, DCMbn, imu)
% F_update: updates F and G matrices before the execution of Kalman filter.
%
% INPUT:
%   upd, 1x8 vector with data fron the INS.
%   DCMbn, DCM body-to-nav.
%   imu, IMU data structure.
%
% OUTPUT:
%   F,  21x21 state transition matrix.
%   G,  21x12 control-input matrix.   
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
%			Titterton, D.H. and Weston, J.L. (2004). Strapdown
% Inertial Navigation Technology (2nd Ed.). Institution
% of Engineering and Technology, USA. Eq. 12.18, p. 345.
%
% 			Farrell, J. (2008). Aided Navigation: GPS With High Rate
% Sensors. McGraw-Hill Professional, USA. Eq. 11.108, p. 407.
%
%			R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a
% simulation framework for low-cost integrated navigation systems,
% Journal of Control Engineering and Applied Informatics, vol. 17,
% issue 2, pp. 110-120, 2015. Eq. 26.
%
%  			R. Gonzalez, J. Giribet, and H. Patiño. An approach to
% benchmarking of loosely coupled low-cost navigation systems,
% Mathematical and Computer Modelling of Dynamical Systems, vol. 21,
% issue 3, pp. 272-287, 2015. Eq. 22.
%
% Version: 003
% Date:    2018/10/08
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

Vn =  upd(1);
Ve =  upd(2);
Vd =  upd(3);
lat = upd(4);
% h   = upd(5);
fn =  upd(6);
fe =  upd(7);
fd =  upd(8);

if (isa(Vn,'single'))
    [RM,RN] = radius(lat, 'single');
    Om = single(7.292115e-5);
    I = single(eye(3));
    Z = single(zeros(3));
else
    [RM,RN] = radius(lat, 'double');
    Om = 7.292115e-5;
    I = eye(3);
    Z = zeros(3);
end

RO = sqrt(RN*RM);

a11 = 0;
a12 = -((Om * sin(lat)) + (Ve/(RO) * tan(lat)));
a13 = Vn/(RO);
a21 = (Om * sin(lat)) + (Ve/(RO) * tan(lat) );
a22 = 0 ;
a23 = (Om * cos(lat)) + (Ve/(RO)) ;
a31 = -Vn/(RO);
a32 = -Om * cos(lat) - (Ve/(RO));
a33 = 0;
Fee = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

a11 = 0;
a12 = 1/(RO);
a13 = 0;
a21 = -1/(RO);
a22 = 0;
a23 = 0;
a31 = 0;
a32 = -tan(lat)/RO;
a33 = 0;
Fev = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

a11 = -Om * sin(lat);
a12 = 0;
a13 = -Ve/(RO^2);
a21 = 0 ;
a22 = 0 ;
a23 = Vn/(RO^2);
a31 =  -Om*cos(lat) - (Ve/((RO)*(cos(lat))^2));
a32 = 0 ;
a33 = (Ve * tan(lat)) / (RO^2) ;
Fep = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

a11 = 0 ;
a12 = -fd ;
a13 = fe ;
a21 = fd ;
a22 = 0 ;
a23 = -fn ;
a31 = -fe ;
a32 = fn ;
a33 = 0 ;
Fve = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

a11 = Vd/(RO);
a12 = -2*((Om * sin(lat)) + ((Ve/(RO)) * tan(lat))) ;
a13 = Vn/RO ;
a21 = (2*Om * sin(lat)) + ( (Ve/(RO)) * tan(lat) );
a22 = (1/(RO)) * ((Vn * tan(lat)) + Vd) ;
a23 = 2*Om * cos(lat) + (Ve/(RO));
a31 = (-2*Vn)/(RO);
a32 = -2*(Om * cos(lat) +  (Ve/(RO))) ;
a33 = 0;
Fvv = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

a11 = -Ve*( (2*Om * cos(lat)) + (Ve/((RO)*(cos(lat))^2)));
a12 = 0 ;
a13 = 1 / (RO^2) * ( ((Ve^2) * tan(lat)) - (Vn * Vd) );
a21 = 2*Om * ( (Vn * cos(lat)) - (Vd * sin(lat)) ) + ( (Vn * Ve) / (RO * (cos(lat))^2) ) ;
a22 = 0 ;
a23 = -(Ve/(RO^2)) * (Vn*tan(lat)+Vd);
a31 = 2 * Om * Ve * sin(lat);
a32 = 0;
a33 =  1/((RO)^2) * ((Vn^2) + (Ve^2));
Fvp = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

Fpe = zeros(3);

a11 = 1/(RO);
a12 = 0;
a13 = 0;
a21 = 0;
a22 = 1/((RO)*cos(lat));
a23 = 0;
a31 = 0;
a32 = 0;
a33 = -1;
Fpv = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

a11 = 0;
a12 = 0;
a13 = -Vn/(RO^2);
a21 = (Ve*tan(lat)) / (RO * cos(lat));
a22 = 0;
a23 = -Ve / (((RO)^2) * cos(lat));
a31 = 0;
a32 = 0;
a33 = 0;
Fpp = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

if (isinf(imu.ab_corr))
    Faa = Z;
else
    Faa = diag(-1./imu.ab_corr);
end

if (isinf(imu.gb_corr))
    Fgg = Z;
else
    Fgg = diag(-1./imu.gb_corr);
end

F= [Fee  Fev  Fep   (DCMbn)  Z       (DCMbn)   Z;
    Fve  Fvv  Fvp   Z       (-DCMbn)  Z       (-DCMbn);
    Fpe  Fpv  Fpp   Z        Z        Z       Z;
    Z    Z    Z     Z        Z        Z       Z;
    Z    Z    Z     Z        Z        Z       Z;
    Z    Z    Z     Z        Z        Fgg     Z;
    Z    Z    Z     Z        Z        Z       Faa;
    ];

G = [DCMbn  Z     Z    Z;
    Z      -DCMbn 	Z    Z;
    Z      Z     	Z    Z;
    Z      Z     	Z    Z;
    Z      Z     	Z    Z;
    Z      Z     	I    Z;
    Z      Z     	Z    I;];
end
