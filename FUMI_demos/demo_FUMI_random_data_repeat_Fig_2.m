function demo_FUMI_random_data_repeat_Fig_2()

% This function repeats the synthetic experiment of cFUMI and eFUMI on random data, corresponds to results shown in Fig. 2 of the refered paper.

% REFERENCE :
% C. Jiao, A. Zare, 
% Functions of Multiple Instances for Learning Target Signatures,? 
% IEEE transactions on Geoscience and Remote Sensing, Vol. 53, No. 8, Aug. 2015, DOI: 10.1109/TGRS.2015.2406334
%
% SYNTAX : demo_FUMI_random_data_repeat_Fig_2()

% Author: Changzhe Jiao, Alina Zare
% University of Missouri, Department of Electrical and Computer Engineering
% Email Address: cjr25@mail.missouri.edu; zarea@missouri.edu

%load data
addpath('..\cFUMI_code')
addpath('..\eFUMI_code')
addpath('..\synthetic_data')
load('E_truth')
load('random_data')
parameters_cFUMI= cFUMI_parameters();
parameters_eFUMI= eFUMI_parameters();


%reset parameters
parameters_cFUMI.u = 0.05;
parameters_cFUMI.M=4;
parameters_cFUMI.gammaconst=10;

parameters_eFUMI.u = 0.05;
parameters_eFUMI.M=4;
parameters_eFUMI.gammaconst=10;
parameters_eFUMI.beta=20;

%run cFUMI
[est_E_cFUMI,est_P_cFUMI]=cFUMI(X,labels_point,parameters_cFUMI);
est_E_cFUMI=normalize(est_E_cFUMI,1);

%run eFUMI
[est_E_eFUMI,est_P_eFUMI]=eFUMI(X,labels_bag,parameters_eFUMI);
est_E_eFUMI=normalize(est_E_eFUMI,1);

%calculate error
NSE_cFUMI=norm(E_truth(:,1)-est_E_cFUMI(:,1))/norm(E_truth(:,1));
SAD_cFUMI=acos((est_E_cFUMI(:,1)'*E_truth(:,1))/(norm(est_E_cFUMI(:,1))*norm(E_truth(:,1)))); 

NSE_eFUMI=norm(E_truth(:,1)-est_E_eFUMI(:,1))/norm(E_truth(:,1));
SAD_eFUMI=acos((est_E_eFUMI(:,1)'*E_truth(:,1))/(norm(est_E_eFUMI(:,1))*norm(E_truth(:,1)))); 

fprintf(['Normalized square error by cFUMI is ' num2str(NSE_cFUMI) '\n']);
fprintf(['Spectral angle distance by cFUMI is ' num2str(SAD_cFUMI) '\n' '\n']);

fprintf(['Normalized square error by eFUMI is ' num2str(NSE_eFUMI) '\n']);
fprintf(['Spectral angle distance by eFUMI is ' num2str(SAD_eFUMI) '\n']);


%plot results
figure;plot(wavelength,est_E_cFUMI(:,1),'r','linewidth',2);hold on;plot(wavelength,est_E_eFUMI(:,1),'g','linewidth',1.5);plot(wavelength,E_truth(:,1),'b','linewidth',1.5);hold off;
legend('Estimated Red Slate cFUMI','Estimated Red Slate eFUMI','Real Red Slate');
xlabel('Wavelength (\mum)');ylabel('Reflectance');
axis([0.4 2.5 0 1.2])



