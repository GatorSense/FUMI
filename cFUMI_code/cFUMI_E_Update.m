function     [E]=cFUMI_E_Update(X,P,labels,parameters,flag)

% This function updates endmember maxtrix E during each iteration

% REFERENCE :
% C. Jiao, A. Zare, 
% “Functions of Multiple Instances for Learning Target Signatures,”  
% IEEE transactions on Geoscience and Remote Sensing, DOI: 10.1109/TGRS.2015.2406334
%
% SYNTAX : [E]=cFUMI_E_Update(X,P,labels,parameters,flag)

% Inputs:
%   X - Inputdata, reshaped hyperspectral image treats each pixel as column vector, d by N
%   P - Proportion value calculated from the current iteration, M+1 by N;
%   labels - binary values, the same size as input data, indicates positive points with logical '1'
%   parameters - struct - parameter structure which can be set using the EF_parameters() function
%   flag - method to get E

% Outputs:

%   E - Endmembers, d by M+1, M accounts for the number of background endmembers

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
M=size(P,1)-1;
d=size(X,1);
index_plus=find(labels);
index_minus=find(labels==0);
X_plus=X(:,index_plus);
X_minus=X(:,index_minus);
N_plus=size(X_plus,2);
N_minus=size(X_minus,2);
w=parameters.alpha*(N_minus/N_plus);
P_plus=P(:,index_plus);
P_plus_minus=[zeros(1,N_plus);P_plus(2:end,:)];
P_minus=P(:,index_minus);

% E update
mu=mean(X,2);
DP=parameters.Eps*eye(M+1,M+1);

E=((1-parameters.u)*w*(X_plus*P_plus')+(1-parameters.u)*(X_minus*P_minus')+parameters.u*repmat(mu,1,M+1))*pinv((1-parameters.u)*w*(P_plus*P_plus')+(1-parameters.u)*(P_minus*P_minus')+parameters.u+DP);

if flag==2
    E=normalize(E,2);% if normalization constraint on endmember is added, normalize E after each iteration
end


end





