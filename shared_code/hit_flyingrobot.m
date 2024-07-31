%%% This script performs some tests on hybrid system identification 

% Usage: Simply type the name >> hit_flyingrobot in the command window or
% use the run buttons. It is better to *Run Section*.

% Inputs: None 
% Outputs: structures with clusters and 2D/3D projections
% Dependencies: HIT tool, MPT version 3
% Options: use pregenerated data or regenerate using Breach and
% imitation_pb class

% Run >> help hit_regression for more information.

% Initialization
clear; 
regen=0; % 0: use pregenerated data, 1:regenerate

%% DATA GENERATION
% GENERATE or LOAD MPC DATA

if ~regen
    load("data_X_U_10350.mat") % You will see two matrices named (x_in, u_out) in the workspace
else
    %%% Commands from Alex's script
    %------------------------------

    %% Initialize flying robot problem
    % don't forget to change breach path in init_paths.m

    init_flying_problem;

    %% Compute or load traces with MPC from nominal initial state set
    % if parameters matches in data folder, will just load

    Bmpc_traces = pb_flying.get_mpc_data('num_corners',256, 'num_quasi_random', 744, 'seed', 5000);

    %% Filter data
    % transform traces into samples, then filter them through the grid. See
    % grid resolution in FlyingRobot_imitation_pb.m
    [Bsamples, Bgrid]  = pb_flying.get_grid_data(Bmpc_traces);

    %% Extract raw data
    [x_in , u_out] = pb_flying.prepare_training_data(Bsamples);

    %%% END of data generation
end

%% INITIALIZE HIT

% Modify this path 
addpath(genpath('/Users/kekatos/Files/Projects/Gitlab/Tools/MPT/hit/'))

% Instructions: https://gist.github.com/nikos-kekatos/d25fd4f8457031115cd97fb7ff61413b
% Question--> Should we add HIT in the repo?

% initialize the HIT toolbox (Reminder: HIT script  initializes MPT) 
hit_init

% in the workspace
global idpar plotpar

%% SCENARIO 0 -- two-dimensional output

disp('----------------------------------------------')

fprintf('\n Beginning scenario 0 \n')

% Setup all the parameters for hit_regression
% Setup fields of idpar. See hit_init for a description of the fields


% number of modes
idpar.s=5; 

% size of Local Datasets
idpar.c=20; 

% regressor_set
%idpar.Regressor_set=Regressor_set;

% pattern_recognition algorithm
idpar.patt_rec_algo='psvc'; %psvc %svc %mrlp


% The structure idmodes describes the PWA map.
% Type 'help hit_regression' for a description of all the outputs of
% hit_regression


Xid=x_in(1:400,1:2); %small selection of data to get quick results.
yid=u_out(1:400,:);

[idmodes_all,F_all,xi_all,LDs_all,inliers_all]=hit_regression(Xid,yid);

% idmodes_all.par{i} contains the affine approximation computed by the tool
% for the cluster i.

fprintf('\n There are %i clusters and the affine approximation in cluster 1 is given by \n the vector idmodes_all.par{1}.\n',idpar.s)
fprintf('\n Even though there are two outputs, the approximation computed by the tool is one-dimensional/scalar. \n u1=%0.5f*x1+%.5f*x2+%0.5f.\n',idmodes_all.par{1})

fprintf('\n End of scenario 0. \n')

%% SCENARIO 1A -- SINGLE OUTPUT -- ALL 8 INPUTS 

disp('----------------------------------------------')
fprintf('\n Beginning scenario 1A. Press any key to continue. \n')
pause
hit_init

tic;
total_no_points=length(u_out);
fprintf('The total number of available data points is %i.\n',total_no_points)
no_points=10350;
fprintf('We use the first %i points of %i available data points.\n',no_points,total_no_points)

Xid=x_in(1:no_points,:);
yid=u_out(1:no_points,1)+0.1*randn(1);


% number of modes
idpar.s=4; %40

% size of Local Datasets
idpar.c=160; %140

% regressor_set
%idpar.Regressor_set=Regressor_set;

% pattern_recognition algorithm
idpar.patt_rec_algo='psvc'; %psvc %svc %mrlp

[idmodes_1,F_1,xi_1,LDs_1,inliers_1]=hit_regression(Xid,yid);

t=toc;
if size(Xid,2)>3
    var=2:4;
    figure;hold on; plot(projection(idmodes_1.regions(1),var))
    plot(projection(idmodes_1.regions(2),var),'b');
    plot(projection(idmodes_1.regions(3),var),'y');
    plot(projection(idmodes_1.regions(4),var),'g');
end


% Validate the model: Test if all modes have a "similar" mse.
% Modes with "high" mse are likely to be badly reconstructed. 

[mse,mse_mode]=hit_mse(Xid,yid,idmodes_1);

% Test if the pattern recognition algorithm has misclassified few
% datapoints.
% Type 'help hit_regression' for the meaning of 
% idmodes.pattern_rec_valid.correctness
% A correctness of 100 means that no point has been misclassified.
disp(' ')
disp('A correctness of 100 means that no point has been misclassified.')
idmodes_1.pattern_rec_valid.correctness

%% SCENARIO 1  (continued)-- SINGLE OUTPUT (second) -- ALL 8 INPUTS 
disp('----------------------------------------------')
fprintf('\n Beginning scenario 1B. Press any key to continue. \n')
pause
hit_init
tic;
total_no_points=length(u_out);
fprintf('The total number of available data points is %i.\n',total_no_points)
no_points=10350;
fprintf('We use the first %i points of %i available data points.\n',no_points,total_no_points)

Xid=x_in(1:no_points,:);
yid=u_out(1:no_points,2);

% number of modes
idpar.s=4; 
% size of Local Datasets
idpar.c=160; 
% regressor_set
%idpar.Regressor_set=Regressor_set;

% pattern_recognition algorithm
idpar.patt_rec_algo='psvc'; %psvc %svc %mrlp

[idmodes_2,F_2,xi_2,LDs_2,inliers_2]=hit_regression(Xid,yid);

t=toc;
if size(Xid,2)>3
    var=2:4;
    figure;hold on; plot(projection(idmodes_2.regions(1),var))
    plot(projection(idmodes_2.regions(2),var),'b');
    plot(projection(idmodes_2.regions(3),var),'y');
    plot(projection(idmodes_2.regions(4),var),'g');
end

%% SCENARIO 2 -- Mixing local datasets 

disp('----------------------------------------------')
fprintf('\n Beginning scenario 2. Press any key to continue. \n')
pause

hit_init;
Xid=x_in(1:1000,1:2);
yid=u_out(1:1000,2);%+0.1*randn(1);

% number of modes
idpar.s=5;

% size of Local Datasets
idpar.c=20; 

idpar.patt_rec_algo='psvc'; %psvc %svc %mrlp

%idpar.mix_detect='n';
[idmodes_3,F_3,xi_3,LDs_3,inliers_3]=hit_regression(Xid,yid);

figure; hold on;
plot(idmodes_3.regions(1),'r')
plot(idmodes_3.regions(2),'g')
plot(idmodes_3.regions(3),'b')
plot(idmodes_3.regions(4),'m')
plot(idmodes_3.regions(5),'y')

idpar.mix_detect='n';
[idmodes_4,F_4,xi_4,LDs_4,inliers_4]=hit_regression(Xid,yid);

figure; hold on;
plot(idmodes_4.regions(1),'r')
plot(idmodes_4.regions(2),'g')
plot(idmodes_4.regions(3),'b')
plot(idmodes_4.regions(4),'m')
plot(idmodes_4.regions(5),'y')




% plot the model in figure 33
hit_plot_idmodes(Xid, yid,inliers_3,idmodes_3,33);
% hold on
% % put the same axis as in the plot of the original model
% title('Identified PWARX model');
% ylabel('{u(k-1)}','FontSize',16)
% xlabel('{y(k-1)}','FontSize',16)
% zlabel('{y(k)}','FontSize',16)
% set(gca,'FontSize',13)
% hold off

%% SCENARIO 3 -- Continuous vs noncontinuous   
disp('----------------------------------------------')
fprintf('\n Beginning scenario 3. Press any key to continue. \n')
pause
hit_init

%default noncontinuous
%idpar.continuity='D';

Xid=x_in(1:1000,1:2);
yid=u_out(1:1000,2);%+0.1*randn(1);

% number of modes
idpar.s=5; %40

% size of Local Datasets
idpar.c=20; %140

idpar.patt_rec_algo='psvc'; %psvc %svc %mrlp

[idmodes_5,F_5,xi_5,LDs_5,inliers_5]=hit_regression(Xid,yid);

figure; hold on;
plot(idmodes_5.regions(1),'r')
plot(idmodes_5.regions(2),'g')
plot(idmodes_5.regions(3),'b')
plot(idmodes_5.regions(4),'m')
plot(idmodes_5.regions(5),'y')


Xid=x_in(1:1000,1:2);
yid=u_out(1:1000,2);%+0.1*randn(1);

% number of modes
idpar.s=5; %40

% size of Local Datasets
idpar.c=20; %140

idpar.patt_rec_algo='psvc'; %psvc %svc %mrlp
idpar.continuity='c';
[idmodes_6,F_6,xi_6,LDs_6,inliers_6]=hit_regression(Xid,yid);

figure; hold on;
plot(idmodes_6.regions(1),'r')
plot(idmodes_6.regions(2),'g')
plot(idmodes_6.regions(3),'b')
plot(idmodes_6.regions(4),'m')
plot(idmodes_6.regions(5),'y')