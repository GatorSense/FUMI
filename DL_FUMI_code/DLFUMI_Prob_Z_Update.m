function [Prob_Z]=DLFUMI_Prob_Z_Update(X,P,E,labels,parameters,T)

% probability estimation, accounts for the E step

% REFERENCE :
% C. Jiao, A. Zare, 
% Multiple Instance Dictionary Learning using Functions of Multiple Instances
% arXiv preprint arXiv:1511.02825 (2015), http://arxiv.org/abs/1511.02825

% If any of the code is used, the above reference must be cited. 

% Inputs:
%   X - Inputdata, treats each pixel as column vector, d by N
%   P - Representation Values, T+M by N 
%   E - Dictionary matrix calculated from the previous iteration, d by T+M;
%   labels - binary the same size as input data, indicates positive bag with logical '1'
%   parameters - struct, parameter structure which can be set using the DLFUMI_parameters() function
%   T - number of current target concepts


% Outputs:

%   Prob_Z - probability to indicate the probability of points in positive to be real target 2 by N.



E_minus=E(:,(T+1):end);%extract background endmembers
N=size(X,2);% total number of data
index_plus=find(labels); %find positive index
index_minus=find(labels==0);%find negative index

Prob_Z=zeros(2,N);
    
Prob_Z(1,index_plus)=exp(-parameters.beta*(sum((X(:,index_plus)-E_minus*P((T+1):end,index_plus)).^2)));%% don't apply normalize by data norm

Prob_Z(2,index_plus)=1-Prob_Z(1,index_plus);

Prob_Z(1,index_minus)=1;
Prob_Z(2,index_minus)=0;

end






