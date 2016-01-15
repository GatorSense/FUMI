function pixelList = FUMI_reshape(imageData)


% This function reshapes the 3D hyperspectral data cube into 2D pixel list

% REFERENCE :
% C. Jiao, A. Zare, 
% “Functions of Multiple Instances for Learning Target Signatures,”  
% IEEE transactions on Geoscience and Remote Sensing, DOI: 10.1109/TGRS.2015.2406334
%
% SYNTAX : pixelList = FUMI_reshape(imageData)

% Inputs:
%   imageData - hyperspectral data cube, M by N by d
%
% Outputs:
%   pixelList - reshaped data, d by (M*N)

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

imageData=double(imageData);

pixelList = reshape(shiftdim(imageData(:,:,:),2),size(imageData,3),size(imageData,1)*size(imageData,2));