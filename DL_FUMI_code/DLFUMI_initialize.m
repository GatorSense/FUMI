function [E,P]=DLFUMI_initialize(X,labels,parameters)

%  DLFUMI initialization 

% REFERENCE :
% C. Jiao, A. Zare, 
% Multiple Instance Dictionary Learning using Functions of Multiple Instances
% arXiv preprint arXiv:1511.02825 (2015), http://arxiv.org/abs/1511.02825

% If any of the code is used, the above reference must be cited. 

% Inputs:
%   X - Inputdata, treats each pixel as column vector, d by N
%   parameters - struct - parameter structure which can be set using the DLFUMI_parameters() function
%   labels - binary the same size as input data, indicates positive bag with logical '1'
%
% Outputs:
%   E - Initial Dictionary set, d by T+M
%   P - Initial Proportion Values, T+M by N

T=parameters.T;
M=parameters.M;
index_plus=find(labels);
index_minus=find(labels==0);
X_plus=X(:,index_plus);
X_minus=X(:,index_minus);
N_plus=size(X_plus,2);
N_minus=size(X_minus,2);

%E initialization
n=floor(N_plus/T);

sq_target=randperm(N_plus);

%%%use random mean of X_plus;
for i=1:T
    E_plus(:,i)=mean(X_plus(:,sq_target(((i-1)*n+1):i*n)),2);
end



[~,C]=kmeans(X_minus',M);
E_minus=C';



E=[E_plus E_minus];
E=normalize(E,2);

%P initialization
P_plus=pinv(E'*E)*E'*X_plus;%use mean value to initialize proportion values according labels
P_minus=pinv(E_minus'*E_minus)*E_minus'*X_minus;
P(:,index_plus)=P_plus;
P(:,index_minus)=[zeros(T,N_minus);P_minus];

end