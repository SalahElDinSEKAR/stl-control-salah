function params = setParamsDrone()

    %% plant

    % physical parameters
    params.g = 9.81; % m.s^(-2)
    params.m = 3.3e-2; % kg
    params.Ix = 1.395e-5; % kg.m^2
    params.Iy = 1.436e-5; % kg.m^2
    params.Iz = 2.173e-5; % kg.m^2
    
    % model parameters
    params.nStates = 12;
    params.nSensorOutputs = 12;
    params.indexSensorOutputs = [1;2;3;4;5;6;7;8;9;10;11;12]; % indices of the sensor outputs in the state vector
    params.nControlInputs = 4;
    
    % plant dynamics
    params.plantDynamics = @quadrotorODE;
    params.computeJacobianPlant = @quadrotorJacobian;
    
    
    %% Attitude controller parameters
    % response time of 0.5s, conseidered as a continuous system
    params.Kphi = 1;%6;
    params.Ktheta = 1;%6;
    params.Kpsi = 1;
    params.KdotPhi = 2.79e-5;
    params.KdotTheta = 2.8e-5;
    params.KdotPsi = 4.35e-5;
    
    %% Desired path, initial set, target region and safe region
    
    % Desired path parameters
    params.nWayPoints = 31;
    
    % Velocity target
    params.velocityTarget = 0.7;
    
    % Safe region parameters
    params.positionTolerance = 1; % maximum authorized distance to path
    params.velocityTolerance = 0.5; 
    params.thetaTolerance = 0.5; 
    params.phiTolerance = 0.5;
    params.subdivision = 6; % subdivision for the tube
    
    % Initial set parameters
    params.positionToleranceInit = 0.15;
    params.velocityToleranceInit = 0.05; 
    params.thetaToleranceInit = 0.05; 
    params.phiToleranceInit = 0.05;
    

    params.subdivisionInitialSet = 3;
    
    params.maxPhi = 0.025;
    params.maxTheta = 0.05;
    params.maxPsi = 0.001;
    params.maxDotPhi = 0.05;
    params.maxDotTheta = 0.05;
    params.maxDotPsi = 0.001;
    
    % desired path
    params.generateWayPoints = @generateWayPointsDrone;
    % random state in safe region
    params.randomStateInSafeRegion = @randomStateInSafeRegionDrone;
    % random state in initial set
    params.randomStateInInitialSet = @randomStateInInitialSetDrone;
    % move state in initial set
    params.keepStateInInitialSet = @keepStateInInitialSetDrone;
    % build safe region
    params.buildSafeRegion = @buildSafeRegionDrone;
    % test in safe region
    params.inSafeRegion = @inSafeRegionDrone;
    % test in initial set
    params.inInitialSet = @inInitialSetDrone;
    % test in target set
    params.inTargetRegion = @inTargetRegionDrone;
    % distance to a way point
    params.calculateDistanceToWayPoint = @calculateDistanceToWayPointDrone;

    
    %% Controllers (MPC and NN)
    
    % limits over control inputs u
    params.lbControlInputs = [0;-pi/8;-pi/8;-pi/8]; % [ N, rad, rad, rad]
    params.ubControlInputs = [0.7;pi/8;pi/8;pi/8]; % [ N, rad, rad, rad]
    
   % MPC parameters
    params.nWayPointsMPC = 4; % the target way point for the MPC is the last of the next nWayPointsMPC way points
    %params.nStepMPC = 14; % number of control steps to reach the target
    params.timeStepMPC = 0.1; % control step (in seconds)
    params.C1 = 0.1;%0.05; % weight on control input norm (energy)
    params.C2 = 0.1; % weight on position error
    params.C3 = 0.0; % weight on velocity error
    params.C4 = 0.05; % weight on yaw error
    params.C5 = 0.0; % weight on the final velocity error 

    params.objectiveFunction = @objectiveFunctionDrone;
    
    
    %% simulation
       
    % simulation parameters
    params.maxDurationSimu = 900; % maximum duration of the simulation (expressed in the number of control steps) 
    
    

end

