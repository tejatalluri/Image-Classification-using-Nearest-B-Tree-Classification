
function value = similarityMatrix(I, J)

[r, c] = size(I);

A = [];

I=double(I);
J=double(J);


for i = 1:r
    for j = 1:r
        % (sj * sin hj - si * sin hi)^2
        M1 = (I(i, 2) * sin(I(i, 1)) - J(j, 2) * sin(J(j, 1)))^2;
        % (sj * cos hj - si * cos hi)^2
        M2 = (I(i, 2) * cos(I(i, 1)) - J(j, 2) * cos(J(j, 1)))^2;
        % (vj - vi)^2
        M3 = (I(i, 3) - J(j, 3))^2;
        
        M0 = sqrt(M1 + M2 + M3);
        
        %A(i, j) = 1 - 1/sqrt(5) * M0;
        A(i, j) = 1 - (M0/sqrt(5));  
    end
end
        
    
%Obtain Similarity Matrix...
value = A;

% ------------------------------------------------------------
