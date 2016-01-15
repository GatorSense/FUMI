function [Prob_Z]=eFUMI_Prob_Z_Update(X,P,E,labels,parameters)

% This function estimates probability of each data in positive bag to be true positive

% REFERENCE :
% C. Jiao, A. Zare, 
% “Functions of Multiple Instances for Learning Target Signatures,”  
% IEEE transactions on Geoscience and Remote Sensing, DOI: 10.1109/TGRS.2015.2406334
%
% SYNTAX : [Prob_Z]=eFUMI_Prob_Z_Update(X,P,E,labels,parameters)

% Inputs:
%   X - Inputdata, reshaped hyperspectral image treats each pixel as column vector, d by N
%   P - Proportion value calculated from the previous iteration M+1 by N;
%   E - Endmember matrix calculated from the previous iteration d by M+1;
%   labels - binary values, the same size as input data, indicates positive bags with logical '1'
%   parameters - struct - parameter structure which can be set using the EF_parameters() function
%   flag - method to get probability

% Outputs:

%   Prob_Z - probability to indicate the probability of points in positive, 2 by N, 1st row denotes the probability of false positive, 2nd denotes the probability of true positive

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
N=size(X,2);
index_plus=find(labels); 
index_minus=find(labels==0);

% estimate probability

Prob_Z=zeros(2,N);
    
Prob_Z(1,index_plus)=exp(-parameters.beta*(sum((X(:,index_plus)-E_minus*P(2:end,index_plus)).^2)));% use euclidean distance as error
Prob_Z(2,index_plus)=1-Prob_Z(1,index_plus);
    
Prob_Z(1,index_minus)=1;
Prob_Z(2,index_minus)=0;

end



