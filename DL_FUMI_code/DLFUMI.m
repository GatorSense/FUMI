function [E, P]=DLFUMI(Inputdata,labels,parameters)

% DLFUMI (Dictionary Learning using Functions of Multiple Instances): semi-supervised target concept learning algorithm

% REFERENCE :
% C. Jiao, A. Zare, 
% Multiple Instance Dictionary Learning using Functions of Multiple Instances
% arXiv preprint arXiv:1511.02825 (2015), http://arxiv.org/abs/1511.02825

% If any of the code is used, the above reference must be cited. 

% Inputs:
%   Inputdata - MIL data, d by N, d: number of data dimension, N, total number of data
%   parameters - struct - parameter structure which can be set using the DL_parameters() function
%   labels - binary the same size as input data, indicates positive bag with logical '1'
%
% Outputs:
%   E - Estimated dictionary set, d by T+M, T is number of target concepts, M accounts for the number of background endmembers
%   P - Representation Values, T+M by N 

Inputdata=double(Inputdata);

X=Inputdata;

T=parameters.T;
M=parameters.M;
d=size(X,1);
N=size(X,2);
index_plus=find(labels); 
index_minus=find(labels==0);
X_plus=X(:,index_plus);
X_minus=X(:,index_minus);
N_plus=size(X_plus,2);% total No. of data in positive bags
N_minus=size(X_minus,2);
W_plus=parameters.w*ones(1,N_plus);% weight line for points from positive bags
W_minus=ones(1,N_minus);% weight line for points from negative bags
W(index_plus)=W_plus;
W(index_minus)=W_minus;

%initialize

[E_initial,P]=DLFUMI_initialize(X,labels,parameters);% use random value to initialize 

E=E_initial;

obj_func=inf;

for iter=1:parameters.iterationCap

    Prob_Z=DLFUMI_Prob_Z_Update(X,P,E,labels,parameters,T);
    
%M-step
    %update P
    P_old=P;
    P=DLFUMI_P_Update(X,Prob_Z,E,P_old,labels,T,parameters);

    %update E

    E_old=E;
    [E,E_t,cos_mat]=DLFUMI_E_Update_individual(X,P,E_old,Prob_Z,labels,parameters,T,M);


    %Condition Update        
    obj_func_old=obj_func;
    [Cond, obj_func]=DLFUMI_Cond_Update(X,P,E,Prob_Z,obj_func_old,parameters,cos_mat,T,E_t,W);
    

    if abs(Cond)<parameters.changeThresh %if the change in objective function is smaller than set threshold
        break;
    end
    
   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%prune background endmembers
    
        
    if(M>1)
        
        E_backgr=E(:,T+1:end);% extract background endmembers 
        prune_num_M=sum(max(abs(E_backgr),[],1)<parameters.endmemberPruneThreshold);
        [~,sort_E_backgr_idx]=sort(max(E_backgr,[],1),'ascend');        
        if prune_num_M==M
            prune_num_M=prune_num_M-1;%%%don't prune all atoms
        end
        
        if prune_num_M>0
            pruneIndex_M=sort_E_backgr_idx(1:prune_num_M)+T;
            P(pruneIndex_M,:)=[];%prune proportion values corresponding to endmember eligible to prune
            E(:,pruneIndex_M)=[];%prune unnecessary endmember
            M=M-prune_num_M;
        end

    end       

    
    fprintf(['Iteration ' num2str(iter) '\n']);
    fprintf(['Obj_Func=' num2str(obj_func) '\n']);
    fprintf(['Cond=' num2str(Cond) '\n']);
    fprintf(['T=' num2str(T) '\n']);
    fprintf(['M=' num2str(M) '\n' '\n']);    
    
end








