function [E,P]=eFUMI_VCA_initialize(X,labels,parameters)

% This function initializes eFUMI using VCA by computing corresponding unmixing errors

% REFERENCE :
% C. Jiao, A. Zare, 
% “Functions of Multiple Instances for Learning Target Signatures,”  
% IEEE transactions on Geoscience and Remote Sensing, DOI: 10.1109/TGRS.2015.2406334
%
% SYNTAX : [E,P]=eFUMI_VCA_initialize(X,labels,parameters)

% Inputs:
%   Inputdata - hyperspectral image data, can be both in data cube and linear data
%   parameters - struct - parameter structure which can be set using the EF_parameters() function
%   labels - binary values, the same size as input data, indicates positive bags with logical '1'
%
% Outputs:
%   E - Initial Endmembers value, d and M account for wavelength bands and number of background endmembers,respectively
%   P - Initial Proportion Values, M+1 by N, N accounts for the total number of input data
%


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



%extrtact data according labels

M=parameters.M;
index_plus=find(labels);
index_minus=find(labels==0);
X_plus=X(:,index_plus);
X_minus=X(:,index_minus);
N_plus=size(X_plus,2);
N_minus=size(X_minus,2);

%E initialization

E_minus=VCA(X_minus,'Endmembers',M);% VCA to extract background endmembers
P_plus_unmix=FUMI_unmix(X_plus,E_minus,1);
syn_X_plus=E_minus*P_plus_unmix;
unmix_diff=sqrt(sum((X_plus-syn_X_plus).^2,1));
[~,idx_et]=max(unmix_diff);
e_t=X_plus(:,idx_et);


E=[e_t E_minus];

if parameters.flag_E==2
    E=normalize(E,2);% if normalization constraint on endmember is added, normalize E as well
end


%%%P initialization
P_plus=ones(M+1,N_plus)*(1/(M+1));%use mean value to initialize proportion velues according labels
P_minus=ones(M,N_minus)*(1/M);
P(:,index_plus)=P_plus;
P(:,index_minus)=[zeros(1,N_minus);P_minus];

end
            


    

