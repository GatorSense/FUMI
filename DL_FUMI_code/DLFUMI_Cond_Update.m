function [Cond, obj_func]=DLFUMI_Cond_Update(X,P,E,Prob_Z,obj_func_old,parameters,cos_mat,T,E_t,W)


% condition update

% REFERENCE :
% C. Jiao, A. Zare, 
% Multiple Instance Dictionary Learning using Functions of Multiple Instances
% arXiv preprint arXiv:1511.02825 (2015), http://arxiv.org/abs/1511.02825

% If any of the code is used, the above reference must be cited. 


% Inputs:
%   X - Inputdata, treats each pixel as column vector, d by N
%   P - Representation value calculated from the previous iteration, T+M by N;
%   E - Dictionary matrix calculated from the previous iteration d by T+M;
%   Prob_Z - probability to indicate the probability of points in positive to be real target 2 by N
%   obj_func_old - objective function value of the previous iteration
%   parameters - struct, parameter structure which can be set using the EF_arameters() function
%   cos_mat - inner product value between target and background dictionary atoms, M by T
%   T: number of target atoms
%   E_t - subset of E for target atoms
%   W - weight vector balances positive and negative bags



% Outputs:
%   Cond - current change in objective function
%   obj_func - current objective function value

E_minus=E(:,(T+1):end);
P_minus=P((T+1):end,:);

P_weighted=[P(1:T,:).*repmat(Prob_Z(2,:),T,1);P_minus];
error_term=0.5*sum(Prob_Z(1,:).*W.*sum((X-E_minus*P_minus).^2))+0.5*sum(Prob_Z(2,:).*W.*sum((X-E*P).^2));
l1_term=parameters.lambda*sum(W.*sum(abs(P_weighted)));

corss_prod_mat=(E_minus'*E_t);
robust_term=(parameters.tao)*sum(sum(cos_mat.*corss_prod_mat));
obj_func=error_term+l1_term+robust_term;

Cond=obj_func_old-obj_func;
end
