function params = setParams()

    %% plant

    % physical parameters
    params.lf = 1.8;
    params.lr = 1.5;
   
    % model parameters
    params.nStates = 4;
    params.nSensorOutputs = 4;
    params.indexSensorOutputs = [1;2;3;4]; % indices of the sensor outputs in the state vector
    params.nControlInputs = 2;
    
    % plant dynamics
    params.plantDynamics = @vehicleODE;
    params.computeJacobianPlant = @vehicleControlJacobian;
    
    %% Desired path, initial set, target region and safe region
    
    % Quadratic Bezier path parameters
    % P0, P1 and P2 must be column vectors
    params.P0 = [0;0;0];
    params.P1 = [2;20;0];
    params.P2 = [30;30;0];
    params.nWayPoints = 21;
    
    % Velocity target
    params.velocityTarget = 8;
    
    % Safe region parameters
    params.positionTolerance = 2; % maximum authorized distance to path
    params.headingTolerance = 0.2; % maximum authorized heading error
    params.velocityTolerance = 0.5; % maximum authorized velocity error
    
    
    % initial set parameters
    params.positionToleranceInit = 0.8; % side of the initial box for position
    params.headingToleranceInit = 0.2; % maximum authorized heading error
    params.velocityToleranceInit = 0.2; % maximum authorized velocity error
    
    % desired path
    params.generateWayPoints = @generateWayPointsVehicle;
    % random state in safe region
    params.randomStateInSafeRegion = @randomStateInSafeRegionVehicle;
    % random state in initial set
    params.randomStateInInitialSet = @randomStateInInitialSetVehicle;
    % build safe region
    params.buildSafeRegion = @buildSafeRegionVehicle;
    % test in safe region
    params.inSafeRegion = @inSafeRegionVehicle;
    % test in initial set
    params.inInitialSet = @inInitialSetVehicle;
    % test in target set
    params.inTargetRegion = @inTargetRegionVehicle;
    % distance to a way point
    params.calculateDistanceToWayPoint = @calculateDistanceToWayPointVehicle;

    
    %% Controllers (MPC and NN)
    
    % limits over control inputs u(1) and u(2)
    params.lbControlInputs = [-5;-0.2];
    params.ubControlInputs = [5;0.2];
    
    % MPC parameters
    params.nWayPointsMPC = 5; % the target way point for the MPC is the last of the next nWayPointsMPC way points
    %params.nStepMPC = 14; % number of control steps to reach the target
    params.timeStepMPC = 0.1; % control step (in seconds)
    params.C1 = 0.05; % weight on control input norm (energy)
    params.C2 = 0.1; % weight on position error
    params.C3 = 0.02; % weight on heading error
    params.C4 = 0.1; % weight on velocity error

    params.objectiveFunction = @objectiveFunctionVehicle;
    
    %% simulation and falsification
       
    % simulation parameters
    params.maxDurationSimu = 100; % maximum duration of the simulation (expressed in the number of control steps) 
    

end

