function demo_cFUMI_noisy_data_repeat_Fig_3_a()
% This function repeats the synthetic experiment of cFUMI on noisy data with SNR 10, 20, 30 and 40 dB, corresponds to results shown in Fig. 3(a) of the refered paper.

% REFERENCE :
% C. Jiao, A. Zare, 
% Functions of Multiple Instances for Learning Target Signatures,? 
% IEEE transactions on Geoscience and Remote Sensing, Vol. 53, No. 8, Aug. 2015, DOI: 10.1109/TGRS.2015.2406334
%
% SYNTAX : demo_cFUMI_noisy_data_repeat_Fig_3_a()

% Author: Changzhe Jiao, Alina Zare
% University of Missouri, Department of Electrical and Computer Engineering
% Email Address: cjr25@mail.missouri.edu; zarea@missouri.edu

%load data
addpath('..\cFUMI_code')
addpath('..\synthetic_data')
load('E_truth')
parameters= cFUMI_parameters();

%reset parameters
parameters.u = 0.05;
parameters.gammaconst=10;

%run cFUMI, SNR=10db
load('noisy_data_SNR_10')
[est_E_SNR_10,est_P_SNR_10]=cFUMI(X,labels_point,parameters);
est_E_SNR_10=normalize(est_E_SNR_10,1);

%run cFUMI, SNR=20db
load('noisy_data_SNR_20')
[est_E_SNR_20,est_P_SNR_20]=cFUMI(X,labels_point,parameters);
est_E_SNR_20=normalize(est_E_SNR_20,1);

%run cFUMI, SNR=30db
load('noisy_data_SNR_30')
[est_E_SNR_30,est_P_SNR_30]=cFUMI(X,labels_point,parameters);
est_E_SNR_30=normalize(est_E_SNR_30,1);

%run cFUMI, SNR=40db
load('noisy_data_SNR_40')
[est_E_SNR_40,est_P_SNR_40]=cFUMI(X,labels_point,parameters);
est_E_SNR_40=normalize(est_E_SNR_40,1);

%calculate error
NSE_SNR_10=norm(E_truth(:,1)-est_E_SNR_10(:,1))/norm(E_truth(:,1));
SAD_SNR_10=acos((est_E_SNR_10(:,1)'*E_truth(:,1))/(norm(est_E_SNR_10(:,1))*norm(E_truth(:,1)))); 

NSE_SNR_20=norm(E_truth(:,1)-est_E_SNR_20(:,1))/norm(E_truth(:,1));
SAD_SNR_20=acos((est_E_SNR_20(:,1)'*E_truth(:,1))/(norm(est_E_SNR_20(:,1))*norm(E_truth(:,1)))); 

NSE_SNR_30=norm(E_truth(:,1)-est_E_SNR_30(:,1))/norm(E_truth(:,1));
SAD_SNR_30=acos((est_E_SNR_30(:,1)'*E_truth(:,1))/(norm(est_E_SNR_30(:,1))*norm(E_truth(:,1)))); 

NSE_SNR_40=norm(E_truth(:,1)-est_E_SNR_40(:,1))/norm(E_truth(:,1));
SAD_SNR_40=acos((est_E_SNR_40(:,1)'*E_truth(:,1))/(norm(est_E_SNR_40(:,1))*norm(E_truth(:,1)))); 

fprintf(['Normalized square error cFUMI on SNR=10 data is ' num2str(NSE_SNR_10) '\n']);
fprintf(['Spectral angle distance cFUMI on SNR=10 ddata is ' num2str(SAD_SNR_10) '\n' '\n']);

fprintf(['Normalized square error cFUMI on SNR=20 data is ' num2str(NSE_SNR_20) '\n']);
fprintf(['Spectral angle distance cFUMI on SNR=20 ddata is ' num2str(SAD_SNR_20) '\n' '\n']);

fprintf(['Normalized square error cFUMI on SNR=30 data is ' num2str(NSE_SNR_30) '\n']);
fprintf(['Spectral angle distance cFUMI on SNR=30 ddata is ' num2str(SAD_SNR_30) '\n' '\n']);

fprintf(['Normalized square error cFUMI on SNR=40 data is ' num2str(NSE_SNR_40) '\n']);
fprintf(['Spectral angle distance cFUMI on SNR=40 ddata is ' num2str(SAD_SNR_40) '\n' '\n']);


%plot results
figure;plot(wavelength,est_E_SNR_10(:,1),'r','linewidth',2);hold on;plot(wavelength,est_E_SNR_20(:,1),'g','linewidth',2);plot(wavelength,est_E_SNR_30(:,1),'color',[0.87 0.49 0],'linewidth',2);plot(wavelength,est_E_SNR_40(:,1),'y','linewidth',2);plot(wavelength,E_truth(:,1),'b','linewidth',1.5);hold off;
legend('Estimated Red Slate cFUMI SNR=10','Estimated Red Slate cFUMI SNR=20','Estimated Red Slate cFUMI SNR=30','Estimated Red Slate cFUMI SNR=40','Real Red Slate');
xlabel('Wavelength (\mum)');ylabel('Reflectance');
axis([0.4 2.5 0 1.2])
