function  [xpo, Upr, dpr] = ud_filtering(xpr, y, F, H, G, Q, R, Upr, dpr, dt)
% ud_filter
%
% Author:       Rodrigo Gonzalez, rodralez@frm.utn.edu.ar
% Version:      002
% Date:         January 13, 2014

% *************************************************************************
% Bierman KF 
%
% IS = (R + H * P * H');
% K = (P * H') / (IS) ;
% xu = K*y;
% *************************************************************************

l = max(size(R));


for j = 1:l

    h = H(j,:); 
    r = R(j,j);
    z = y(j);
   
     [xpo, Upo, Dpo] = bierman(z, r, h, xpr, Upr, diag(dpr));      
     dpo = diag(Dpo);

    xpr=xpo; Upr=Upo; dpr=dpo;
end

% *************************************************************************
% Thornton KF
%
% Pp = A * Pu * A' + Qd;
% Pp =  0.5 * (Pp + Pp');
% *************************************************************************

n = max(size(xpr));

if (isa(Upr,'single'))   
    
    I = single(eye(n));    
else
    
    I = single(eye(n));    
end

A = single(I + (F*dt));
Qd = G * Q * G' * dt;

[Upr, Dpr, ~] = thornton(A, G, Q*dt, Upo, diag(dpo), xpo);
dpr = diag(Dpr);

end
