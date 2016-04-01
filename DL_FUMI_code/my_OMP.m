function [A, Rec_error] = my_OMP( X, D, k )

% Inputs:
%   X - Inputdata, reshaped hyperspectral image treats each pixel as column vector, d by N
%   D - Dictionary matrix d by M;
%   k - sparsity level, predefined sparse number, |p|_0=k


% Outputs:

%   A - Representation value, M by N;
%   Rec_error - Reconstruction error


N = size(X,2);
M=size(D,2);
if k>M
    k=M;
end
A = zeros(M,N);%initial representation

    for i = 1:N
        active_idx=[];
        x = X(:,i);%current data
        r=x;
        active_idx=[];
        for j = 1:k
            corss_corr = r' * D;%cross product
            corss_corr(active_idx)=-inf;
            [~,max_idx] = max(corss_corr);%set used index correlation to -inf
            active_idx=[active_idx max_idx];
            D_active=D(:,active_idx);
            temp_a=pinv(D_active'*D_active)*D_active'*x;
%             temp_a=D_active\x;
            r=x - D_active *temp_a;% compute new residual
        end
        A(active_idx,i)=temp_a;
    end
    
    Rec_error=sqrt(sum((X-D*A).^2));
end