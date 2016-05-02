function [Upr,Dpr,xpr] = thornton(A,G,Q,Upo,Dpo,xpo)
% [U_prior D_prior] = thornton(x_post,U_post,D_post,Uq,Dq,A)
% function [x,Upr,Dpr] = thornton(xin,A,Upo,Dpo,G,Q)
%
% M. S. Grewal, L. R. Weill and A. P. Andrews
% Global Positioning Systems, Inertial Navigation and Integration
% John Wiley & Sons, 2000.
%
%  Catherine Thornton's modified weighted Gram-Schmidt
%  orthogonalization method for the predictor update of
%  the Upr-Dpr factors of the covariance matrix
%  of estimation uncertainty in Kalman filtering
%
% INPUTS(with dimensions)
%      xin(n,1) corrected estimate of state vector
%      A(n,n) state transition matrix
%      Upo(n,n) unit upper triangular factor (Upr) of the modified Cholesky
%               factors (Upr-Dpr factors) of the covariance matrix of
%               corrected state estimation uncertainty (P+) 
%      Dpo(n,n) diagonal factor (Dpr) of the Upr-Dpr factors of the covariance
%               matrix of corrected estimation uncertainty (P+)
%      G(n,r) process noise distribution matrix (modified, if necessary to
%               make the associated process noise covariance diagonal)
%      Q(r,r)   diagonal covariance matrix of process noise
%               in the stochastic system model
% OUTPUTS:
%      x(n,1)  predicted estimate of state vector
%      Upr(n,n)  unit upper triangular factor (Upr) of the modified Cholesky
%              factors (Upr-Dpr factors) of the covariance matrix of
%              predicted state estimation uncertainty (P-) 
%      Dpr(n,n)  diagonal factor (Dpr) of the Upr-Dpr factors of the covariance
%              matrix of predicted estimation uncertainty (P-)

xpr   = A * xpo;   % state update
[n,r] = size(G); % get dimensions of state(n) and process noise (r)
% G     = G;       % move to internal array for destructive updates
Upr   = (eye(n));    % initialize lower triangular part of Upr
Dpr   = (zeros(n));  % initialize Dpr

PhiU  = A*Upo;   % rows of [PhiU,G] are to be orthononalized

for i=n:-1:1,
    sigma = 0;   % 1 flop, scalar
   
    for j=1:n,
      
        sigma = sigma + PhiU(i,j)^2 * Dpo(j,j);       % 5 flop, scalar
        if (j <= r)                                   % IF flop        
            sigma = sigma + G(i,j)^2 * Q(j,j);        % 5 flop, scalar
        end
    end
   
    Dpr(i,i) = sigma;                                 % 1 flop, scalar
   
    for j=1:i-1,
      
      
        sigma = 0;                                    % 1 flop, scalar
        for k=1:n
        
            sigma = sigma + PhiU(i,k)*(Dpo(k,k)*PhiU(j,k));  % 3 flops, scalar
        end
      
        for k=1:r,
        
            sigma = sigma + G(i,k)*(Q(k,k)*G(j,k));          % 3 flops, scalar
        end
      
        Upr(j,i) = sigma / Dpr(i,i);                          % 1 div-flop, scalar
      
        for k=1:n,
            PhiU(j,k) = PhiU(j,k) - Upr(j,i)*PhiU(i,k);      % 2 flops, scalar
        end
            
        for k=1:r,

            G(j,k) = G(j,k) - Upr(j,i)*G(i,k);               % 2 flops, scalar
        end
      
    end
    
end

end
