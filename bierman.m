function [xpo, Upo, Dpo] = bierman(z, r, h, xpr, Upr, Dpr)
%--------------------------------------------------------------------------
% Syntax:  [xpo Upo Dpo] = bierman(z,r,h,xpr,Upr,Dpr);
%
% Inputs:       z is a scalar measurement
%               r is the variance of z
%               h is a row vector of length length(xpr)
%               xpr is the apriori state estimate
%               [Upr Dpr] = myUD(P_prior);
%               
% Outputs:      xpo is the aposteriori state estimate
%               [Upo Dpo] = myUD(P_post);
%
% Description:  This function performs the Bierman square root Kalman
%               filter scalar measurement update. That is, it performs
%
%               K = P_prior * h' / (h * P_prior * h' + r);
%               xpo = xpr + K * (z - h * xpr);
%               P_post = (I - K*h) * P_prior;
%
%               but returns xpo, Upo, and Dpo, where
%               [Upo Dpo] = myUD(P_post);
%
% Author:       Brian Moore
%               brimoor@umich.edu
%
% Date:         June 28, 2012
%--------------------------------------------------------------------------


%% Brian Moore

% xpo = xpr;
% Upo = Upr; 
% Dpo = Dpr;

% a = Upo' * h';       % a (21x1), N^2 flop
% 
% b = Dpo * a;         % b (21x1), N^2 flop  
% 
% dz = z - h * xpr; 
% dz = z;                 % scalar
% 
% alpha = r;                  % 1 flop, scalar
% 
% gamma = 1 / alpha;          % 1 div-flop, scalar
% 
% for j = 1:length(xpr) 
%    
%   beta   = alpha;           % 1 flop, scalar
% 
%   alpha  = alpha + a(j) * b(j); %  2 flop, scalar
%   
%   lambda = -a(j) * gamma;       %  1 flop, scalar
%   
%   gamma  = 1 / alpha;           %  1 div-flop, scalar
%   
%   Dpo(j,j) = beta * gamma * Dpo(j,j);     %  2 flop, scalar
%   
%   for i = 1:j-1 
%     
%     Upo(i,j) = Upo(i,j) + b(i) * lambda;  %  2 flop, scalar
%     b(i) = b(i) + b(j) * Upo(i,j);           %  2 flop, scalar
%   end
%   
% end
% 
% xpo = xpo + gamma * dz * b; %  N flop + 1 flop + N flop =  2N+1 flop

%% Brian Moore modified

n = length(xpr);
% xpo = zeros(n,1);
Upo = zeros(n);

dpr = diag(Dpr);

a = Upr' * h';          % a (21x1)
b = dpr .* a;           % b (21x1)
dz = z - h * xpr;       % dz, scalar

beta = zeros(n+1,1);
beta(1) = r;
c  = a .* b;

for j = 2:n+1
    
    beta(j) = beta(j-1) + c(j-1);
end

gamma = 1 ./ beta;
lambda = -a .* gamma(1:n);
dpo = beta(1:n) .* gamma(2:end) .* dpr;

for j = 1:n
    
    for i = 1:j
        
        Upo(i,j) = Upr(i,j) + b(i) * lambda(j);
        b(i) = b(i) + b(j) * Upr(i,j);
    end
end

xpo = xpr + b * (gamma(n+1)* dz);

Dpo = diag(dpo);

end
