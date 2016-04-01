function [X]=normalize(inputdata,flag)

% Inputs:
%   Inputdata - Inputdata, reshaped hyperspectral image treats each pixel as column vector, d by N
%   flag - method to normalize data, 1, normalize data globally between 0 and 1; 2, normalize data individually to norm 1

%
% Outputs:
%   X - normalized data

D=double(inputdata);
d=size(D,1);

if flag==1
    M=max(D);
    m=min(D);
    index_1=(M==m)&(M==0);
    index_2=(M==m)&(M~=0);
    index_3=~(index_1|index_2);
    X(:,index_1)=D(:,index_1);
    for i=1:length(index_2)
        X(:,index_2(i))=D(:,index_2(i))/M(index_2(i));
    end
    X(:,index_3)=(D(:,index_3)-repmat(m(index_3),d,1))./repmat((M(index_3)-m(index_3)),d,1);
end
    

if flag==2
    L=sqrt(sum(D.^2,1));
    index=(L~=0);
    X=D;
    X(:,index)=D(:,index)./repmat(L(index),d,1);
end

end


