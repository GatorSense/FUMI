function [P]=eFUMI_P_Update(X,E,Prob_Z,labels,parameters,gamma_vecs)


% This function updates proportion values P during each iteration

% REFERENCE :
% C. Jiao, A. Zare, 
% Functions of Multiple Instances for Learning Target Signatures,? 
% IEEE transactions on Geoscience and Remote Sensing, Vol. 53, No. 8, Aug. 2015, DOI: 10.1109/TGRS.2015.2406334
%
% SYNTAX : [P]=eFUMI_P_Update(X,E,Prob_Z,labels,parameters,gamma_vecs)

% Inputs:
%   X - Inputdata, reshaped hyperspectral image treats each pixel as column vector, d by N
%   E - Endmember matrix calculated from the previous iteration d by M+1;
%   Prob_Z - probability indicates the probability of points in positive to be true positive 2 by N
%   labels - binary values, the same size as input data, indicates positive bags with logical '1'
%   parameters - struct - parameter structure which can be set using the EF_arameters() function
%   gamma_vecs - gamma coefficients to promote sparsity

% Outputs:

%   P - Proportion value, M+1 by N;

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


%extract data
E_minus=E(:,2:end);
M=size(E_minus,2);
index_plus=find(labels);
index_minus=find(labels==0);
X_plus=X(:,index_plus);
X_minus=X(:,index_minus);
N_plus=size(X_plus,2);
N_minus=size(X_minus,2);
w=parameters.alpha*(N_minus/N_plus);
Prob_Z_plus=Prob_Z(:,index_plus);
gamma_vecs_minus=gamma_vecs(2:end);


%update P
P_plus=zeros(M+1,N_plus);
for i=1:N_plus
    P_plus(:,i)=get_P_plus(X_plus(:,i), E, Prob_Z_plus(:,i),parameters,gamma_vecs,w);%update proportion value for points from positive bags individually

end
P_minus=get_P_minus(X_minus,E_minus,parameters,gamma_vecs_minus);% update proportion value for points from negative bags as a whole

P(:,index_plus)=P_plus;
P(:,index_minus)=[zeros(1,N_minus);P_minus];

end


% update individual point in postiive bag
function p=get_P_plus(x, E,Prob_Z_plus,parameters,gamma_vecs,w)
E_minus=E(:,2:end);
d=size(x,1);
M=size(E_minus,2);
a=1/(-w*(1-parameters.u));

if M>0
DP=parameters.Eps*eye(M+1,M+1);
U=pinv(Prob_Z_plus(1)*([zeros(d,1) E_minus]'*[zeros(d,1) E_minus])+Prob_Z_plus(2)*(E'*E)+DP);
V=(Prob_Z_plus(1)*[zeros(d,1) E_minus]'+Prob_Z_plus(2)*E');

p=U*(V*x+a*gamma_vecs+ones(M+1,1)*((1-ones(1,M+1)*U*(V*x+a*gamma_vecs))/(ones(1,M+1)*U*ones(M+1,1))));

z=p<0;
if(sum(z)>0)
    locs_plus=find(1-z);%find the location of positive proportion
    flag=1-z;
    if flag(1)==1 
        p_temp=zeros(length(z),1);
        p_temp(locs_plus)=get_P_plus(x, E(:,locs_plus),Prob_Z_plus,parameters,gamma_vecs(locs_plus),w);
        p=p_temp;

    else
        p_temp=zeros(length(z),1);
        p_temp(locs_plus)=get_P_plus_minus(x,E(:,locs_plus),parameters,gamma_vecs(locs_plus));
        p=p_temp;
    end
end
else 
    p=1;
end
    
end


%update individual point in postiive bag if it is a false positive

function p=get_P_plus_minus(x,E,parameters,gamma_vecs)
M=size(E,2);
a=-1/(1-parameters.u);
if M>1
    DP=parameters.Eps*eye(M,M);
    U=pinv(E'*E+ DP);
    V=E'*x;
    p=U*(V+a*gamma_vecs+ones(M,1)*((1-ones(1,M)*U*(V+a*gamma_vecs))/(ones(1,M)*U*ones(M,1))));
    z=p<0;
    if(sum(z)>0)
        locs_plus=find(1-z);
        p_temp=zeros(length(z),1);
        p_temp(locs_plus)=get_P_plus_minus(x,E(:,locs_plus),parameters,gamma_vecs(locs_plus));
        p=p_temp;
    end
else
    p=1;
end
end


%update proportion values for points in negative bags

function P=get_P_minus(X,E,parameters,gamma_vecs_minus)
M=size(E,2);
DP=parameters.Eps*eye(M,M);
N=size(X,2);
a=-1/(1-parameters.u);
if(M>1)
U=pinv(E'*E+ DP);
V=E'*X;
P=U*(V+a*repmat(gamma_vecs_minus,1,N)+ones(M,1)*((1-ones(1,M)*U*(V+a*repmat(gamma_vecs_minus,1,N)))/(ones(1,M)*U*ones(M,1))));
Z=P<0;
if(sum(sum(Z))>0)
    ZZ = unique(Z', 'rows', 'first')';
    for i=1:size(ZZ,2)
        if(sum(ZZ(:,i)))>0
            eLocs=find(1-ZZ(:,i));
            rZZi=repmat(ZZ(:,i),1,N);
            inds=all(Z==rZZi, 1);
            P_temp=get_P_minus(X(:,inds),E(:,eLocs),parameters,gamma_vecs_minus(eLocs));
            P_temp2=zeros(size(ZZ,1),sum(inds));
            P_temp2(eLocs,:)=P_temp;
            P(:,inds)=P_temp2;
        end
    end
end
else 
    P = ones(1,size(X,2));
end
end



            



    