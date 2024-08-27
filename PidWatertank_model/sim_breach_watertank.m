function [t,X, p, status] = sim_breach_watertank(control_fn, t, p)
    %% collect parameters
    DimX = 9;                 % last one is cost, which we use with MPC, will see for PID...
    params = p(DimX+1:end); % p(1:DimX) is always 0 for legacy reasons 
    Xinit = params(1:7);     % Initial state 
    ref1  = params(8);    % Additional parameter(s)

    H0=params(1);
    ref0=params(8);

    %% Init signal output
    X = zeros(DimX, numel(t)); % preparing the signals output array
    X(1:7, 1) = Xinit; % Initial state

    Ts = t(2)-t(1);
    
    %% simulation
    
    %Set your desired path for bags
    desiredPath = '/home/sekars/Documents/stage/stl-control-imitation/bags';
    currentDir = pwd;
    cd(desiredPath);

    %launching simulation    
    launch_nodes(H0,ref0);
  
    % return to current path
    cd(currentDir);
    % chemin de fichier .csv
    pose_csv = '/home/sekars/Documents/stage/stl-control-imitation/bags/Info_for_traces/_waterTank1_pose.csv';
    
    X = downsample_and_format_data(pose_csv);
    

	
    status = 0; % required by latest Breach
end
