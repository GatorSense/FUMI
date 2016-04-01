function     [E,E_t,cos_mat]=DLFUMI_E_Update_individual(X,P,E_current,Prob_Z,labels,parameters,T,M)

% Dictionary set estimation

% REFERENCE :
% C. Jiao, A. Zare, 
% Multiple Instance Dictionary Learning using Functions of Multiple Instances
% arXiv preprint arXiv:1511.02825 (2015), http://arxiv.org/abs/1511.02825

% If any of the code is used, the above reference must be cited. 


% Inputs:
%   X - Inputdata, treats each pixel as column vector, d by N
%   P - Representation value, T+M by N;
%   E_current - Dictionary matrix calculated from the previous iteration, d by T+M;
%   Prob_Z - probability to indicate the probability of points in positive to be real target 2 by N
%   labels - binary the same size as input data, indicates positive bag with logical '1'
%   parameters - struct, parameter structure which can be set using the DLFUMI_parameters() function
%   T: number of target atoms
%   M: number of background atoms



% Outputs:

%   E - updated dictionary set, d by T+M
%   E_t - subset of E for target atoms
%   cos_mat - inner product value between target and background dictionary atoms, M by T


w=parameters.w;% weight term for points from positive bags
d=size(X,1);% data dimensions
index_plus=find(labels); %find positive index
index_minus=find(labels==0);%find negative index
X_plus=X(:,index_plus);%extract positive data
X_minus=X(:,index_minus);%extract negative data
N_plus=size(X_plus,2);% total No. of data in positive bags
P_plus=P(:,index_plus);% proportion value for points in positive bags
P_plus_minus=[zeros(T,N_plus);P_plus(T+1:end,:)];%proportion value for points from positive bags with zero target proportion
P_minus=P(:,index_minus);% proportion value for points from negative bags
Prob_Z_plus=Prob_Z(:,index_plus);% probability for points from positive bags to be target


E_old_t=E_current(:,1:T);
E_old_k=E_current(:,T+1:end);

%% update e_t
for t=1:T
    P_plus_t=P_plus;
    P_plus_t(t,:)=0;
    q=(sum(repmat((Prob_Z_plus(2,:).*P_plus(t,:)),d,1).*(X_plus-E_current*P_plus_t),2))/(sum(Prob_Z_plus(2,:).*(P_plus(t,:).^2))+parameters.Eps);
    q=q/norm(q);
    E_current(:,t)=q;
    
end



% re-update E_old_t and cos_mat since E_t is just updated
E_t=E_current(:,1:T);
cos_mat=GF_cos(E_old_k,E_t);
%% update e_k
for k=1:M
    cos_k=cos_mat(k,:);
    P_plus_k=P_plus;
    P_plus_minus_k=P_plus_minus;
    P_minus_k=P_minus;
    P_plus_k(k+T,:)=0;
    P_plus_minus_k(k+T,:)=0;
    P_minus_k(k+T,:)=0;
    r=(sum(w*(repmat(Prob_Z_plus(1,:).*P_plus_minus(k+T,:),d,1).*(X_plus-E_current*P_plus_minus_k)+repmat(Prob_Z_plus(2,:).*P_plus(k+T,:),d,1).*(X_plus-E_current*P_plus_k)),2)+sum(repmat(P_minus(k+T,:),d,1).*(X_minus-E_current*P_minus_k),2)-parameters.tao*sum((repmat(cos_k,d,1).*E_old_t),2))/(sum(w*(P_plus(k+T,:).^2))+sum((P_minus(k+T,:).^2)));
    r=r/norm(r);
    E_current(:,(T+k))=r;
end

E=E_current;

end


function [Y]=GF_cos(E_k,E_t)


Y=(E_k'*E_t)./(sqrt(sum(E_k.^2))'*sqrt(sum(E_t.^2)));



end





