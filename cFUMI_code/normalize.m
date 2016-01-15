function [X]=normalize(inputdata,flag)

% This function normalizes each column of input data according flag value

% REFERENCE :
% C. Jiao, A. Zare, 
% “Functions of Multiple Instances for Learning Target Signatures,”  
% IEEE transactions on Geoscience and Remote Sensing, DOI: 10.1109/TGRS.2015.2406334
%
% SYNTAX : [X]=normalize(inputdata,flag)

% Inputs:
%   Inputdata - Inputdata, reshaped hyperspectral image treats each pixel as column vector, d by N
%   flag - method to normalize data, 1, normalize data between 0 and 1; 2, normalize data individually to norm 1

%
% Outputs:
%   X - normalized data

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

D=double(inputdata);
d=size(D,1);

if flag==1
    M=max(D);
    m=min(D);
    index_1=(M==m)&(M==0);
    index_2=(M==m)&(M~=0);
    index_3=~(index_1|index_2);
    X(:,index_1)=D(:,index_1);
    for i=1:length(index_2)
        X(:,index_2(i))=D(:,index_2(i))/M(index_2(i));
    end
    X(:,index_3)=(D(:,index_3)-repmat(m(index_3),d,1))./repmat((M(index_3)-m(index_3)),d,1);
end
    

if flag==2
    L=sqrt(sum(D.^2,1));
    index=(L~=0);
    X=D;
    X(:,index)=D(:,index)./repmat(L(index),d,1);
end

end


