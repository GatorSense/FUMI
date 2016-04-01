function parameters= DLFUMI_parameters()

% parameter setting

% REFERENCE :
% C. Jiao, A. Zare, 
% Multiple Instance Dictionary Learning using Functions of Multiple Instances
% arXiv preprint arXiv:1511.02825 (2015), http://arxiv.org/abs/1511.02825

% If any of the code is used, the above reference must be cited. 

    
    parameters.changeThresh = 1e-8; %Stopping criterion, When change drops below this threshold the algorithm stops
    parameters.iterationCap = 100; %Iteration cap, used to stop the algorithm
    parameters.Eps=1e-8; % Parameter used to diagonally load some matrices in the code
    parameters.T=2;% denote how many target endmembers there are
    parameters.M=5;% denote how many background endmembers there are
    parameters.w=1.5; %coefficient of weight for points from positive bag
    parameters.tao=0.001;%large amount should mean more distinct target endmember from background, accounts for the robust term;
    parameters.endmemberPruneThreshold=1e-6;%Prune E(:, i) when max(P(i, :))<this threshold
    parameters.beta=30;%coefficient to scale the distance of current points to the space of background endmembers in calculating Prob_Z 
    parameters.lambda=0.001;%sparsity level, accounts for the l1 term

    
end
