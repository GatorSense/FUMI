function [P]=DLFUMI_P_Update(X,Prob_Z,E,P_old,labels,T,parameters)

% Representation update

% REFERENCE :
% C. Jiao, A. Zare, 
% Multiple Instance Dictionary Learning using Functions of Multiple Instances
% arXiv preprint arXiv:1511.02825 (2015), http://arxiv.org/abs/1511.02825

% If any of the code is used, the above reference must be cited. 


% Inputs:
%   X - Inputdata, treats each pixel as column vector, d by N
%   Prob_Z - probability to indicate the probability of points in positive to be real target 2 by N.
%   E - Dictionary matrix calculated from the previous iteration, d by T+M;
%   P_old - Representation Values calculated from the previous iteration, T+M by N 
%   labels - binary the same size as input data, indicates positive bag with logical '1'
%   T - number of target concepts
%   parameters - struct, parameter structure which can be set using the DLFUMI_parameters() function

% Outputs:

%   P - Updated Representation value, T+M by N;


E_minus=E;
E_minus(:,1:T)=0;% form E_minus
E_sub=E;
E_sub(:,1:T)=[];
M=(size(E,2)-T);%NO. of current background endmembers
index_plus=find(labels); %find positive index
index_minus=find(labels==0);%find negative index
X_plus=X(:,index_plus);%extract positive data
X_minus=X(:,index_minus);%extract negative data
N_plus=size(X_plus,2);% total No. of data in positive bags
N_minus=size(X_minus,2);% total No. of data in negative bags
Prob_Z_plus=Prob_Z(:,index_plus);%probability of points in positive bags
P_old_plus=P_old(:,index_plus);
P_old_minus=P_old((T+1):end,index_minus);


P=zeros(M+T,N_plus+N_minus);

eta_plus=1/norm(E'*E);

eta_minus=1/norm(E_sub'*E_sub);
% keyboard

for iter=1:50
    P_plus=P_old_plus+eta_plus*(repmat(Prob_Z_plus(1,:),T+M,1).*(E_minus'*(X_plus-E_minus*P_old_plus))+repmat(Prob_Z_plus(2,:),T+M,1).*(E'*(X_plus-E*P_old_plus)));
    P_minus=P_old_minus+eta_minus*(E_sub'*(X_minus-E_sub*P_old_minus));
    P_plus_ST=DLFUMI_soft_threshold(P_plus,parameters.lambda,Prob_Z_plus(2,:),1,T,M);
    P_minus_ST=DLFUMI_soft_threshold(P_minus,parameters.lambda,ones(1,N_minus),1,0,M);
    P_old_plus=P_plus_ST;
    P_old_minus=P_minus_ST;
end

P(:,index_plus)=P_plus_ST;
P(:,index_minus)=[zeros(T,N_minus);P_minus_ST];



function P_softth=DLFUMI_soft_threshold(P,lambda,Prob_z_1,w,T,M)

lambda_hat=(lambda.*[repmat(Prob_z_1,T,1);ones(M,size(P,2))])/w;
P_softth=sign(P).*max((abs(P)-lambda_hat),0);

    