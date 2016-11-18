function [U, D] = myUD(P)
%--------------------------------------------------------------------------
% Syntax:       [U D] = myUD(P);
%               [U D] = myUD(P,'noerror');
%
% Inputs:       P is a symmetric, positive definite square matrix
%               
%               'noerror' suppresses the error thrown when P is not
%               positive  definite
%
%               Note: myUD() forces P symmetric by only accesing
%               its upper triangle and assuming P(i,j) = P(j,i)
%
% Outputs:      U is a unit upper triangular matrix and D is a diagonal
%               matrix such that P = U * D * U'; 
%              
% Description:  This function computes the UD decomposition of the input
%               square, positive definite, and (assumed) symmetric matrix.
%               If P is not positive definite, an error message is
%               displayed.
%
%               Note: The UD decomposition is a specific form of Cholesky
%               decomposition.
%
% Author:       Brian Moore
%               brimoor@umich.edu
%
% Date:         July 12, 2012
%--------------------------------------------------------------------------

[n,m] = size(P);

if (isa(P,'single')) 
   U = single(zeros(n));
   D = single(zeros(n));
else
    U = zeros(n);
    D = zeros(n);
end

D(end) = P(end);
U(:,end) = P(:,end)./P(end);
    
for j = n-1:-1:1 

    for i = j:-1:1

        sum = P(i,j); % 1 flop
        
        for k = (j+1):n
         sum = sum - U(i,k) * D(k,k) * U(j,k); % 3 flops
        end
        
        if (i == j) % IF flops

            D(j,j) = sum;
            U(j,j) = 1;        
        else
            
            U(i,j) = sum / D(j,j); % DIV flops        
        end
    end
end

end

   