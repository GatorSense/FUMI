function [E, P]=cFUMI(Inputdata,labels,parameters)

% cFUMI (convex Functions of Multiple Instances): semi-supervised target concept learning algorithm

% REFERENCE :
% C. Jiao, A. Zare, 
% Functions of Multiple Instances for Learning Target Signatures,? 
% IEEE transactions on Geoscience and Remote Sensing, Vol. 53, No. 8, Aug. 2015, DOI: 10.1109/TGRS.2015.2406334
%
% SYNTAX : [E, P]=cFUMI(Inputdata,labels,parameters)

% Inputs:
%   Inputdata - hyperspectral image data, can be either in data cube or linear data
%   parameters - struct - parameter structure which can be set using the cFUMI_parameters() function
%   labels - binary values, the same size as input data, indicates positive data point with logical '1'
%
% Outputs:
%   E - Endmembers, d by M+1, d and M account for wavelength bands and number of background endmembers,respectively
%   P - Proportion Values, M+1 by N, N accounts for the total number of input data

% Author: Changzhe Jiao, Alina Zare
% University of Missouri, Department of Electrical and Computer Engineering
% Email Address: cjr25@mail.missouri.edu; zarea@missouri.edu
% Created: October, 2013
% Latest Revision: January, 2015
%
% This product is Copyright (c) 2015 University of Missouri
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions
% are met:
%
%   1. Redistributions of source code must retain the above copyright
%      notice, this list of conditions and the following disclaimer.
%   2. Redistributions in binary form must reproduce the above copyright
%      notice, this list of conditions and the following disclaimer in the
%      documentation and/or other materials provided with the distribution.
%   3. Neither the name of the University nor the names of its contributors
%      may be used to endorse or promote products derived from this software
%      without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY OF MISSOURI AND
% CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
% INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
% MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED.  IN NO EVENT SHALL THE UNIVERSITY OR CONTRIBUTORS
% BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
% EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
% LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES,
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
% HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
% OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

%reshpae data if necessary
flag_Data=0;
Inputdata=double(Inputdata);

if length(size(Inputdata))==3
    flag_Data=1;
    Inputdata_reshape=FUMI_reshape(Inputdata);
    labels=reshape(labels,1,size(Inputdata_reshape,2));
    X=Inputdata_reshape;
else
    X=Inputdata;
end

%extrtact data according labels
M=parameters.M;
d=size(X,1);
N=size(X,2);
index_plus=find(labels); 
index_minus=find(labels==0);
X_plus=X(:,index_plus);
X_minus=X(:,index_minus);
N_plus=size(X_plus,2);
N_minus=size(X_minus,2);
w=parameters.alpha*(N_minus/N_plus);
W_plus=w*ones(1,N_plus);
W_minus=ones(1,N_minus);
W(index_plus)=W_plus;
W(index_minus)=W_minus;

%initialize
[E_initial,P]=cFUMI_VCA_initialize(X,labels,parameters);
E=E_initial;
obj_func=inf;
Cond=inf;

for i=1:parameters.iterationCap
    
    %update P
    P_old=P;
    gamma_vecs=[0;(parameters.gammaconst./(sum(P_old(2:end,:),2)))];
    P=cFUMI_P_Update(X,E,labels,parameters,gamma_vecs);

    %update E
    E_old=E;
    flag_E=parameters.flag_E;
    E=cFUMI_E_Update(X,P,labels,parameters,flag_E);
    
    
    %Condition Update        
    obj_func_old=obj_func;
    [Cond, obj_func]=cFUMI_Cond_Update(X,P,E,W,obj_func_old,parameters,gamma_vecs);
    
    %display condition after each iteration
    fprintf(['Iteration ' num2str(i) '\n']);
    fprintf(['Obj_Func=' num2str(obj_func) '\n']);
    fprintf(['Cond=' num2str(Cond) '\n' '\n']);
    
    if Cond<parameters.changeThresh %if the change in objective function is smaller than set threshold
        break;
    end
    
    %prune unnecessary endmember
    P_backgr=P(2:end,:);
    pruneIndex=max(P_backgr,[],2)<parameters.endmemberPruneThreshold;%find endmember eligible to prune
    P([false; pruneIndex],:)=[];
    E(:,[false; pruneIndex])=[];

end

%display proportion map
if flag_Data==1
    FUMI_viewresults(Inputdata,P);
end
end

