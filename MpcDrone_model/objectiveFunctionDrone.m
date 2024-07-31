function [J, deriv] = objectiveFunctionDrone(params, state0, expandPath, indexTargetWayPoint, uVec)

    % define target way point
    targetWayPoint = zeros(6,1);
    targetWayPoint(1:3,1) = expandPath.wayPoints(indexTargetWayPoint, :)'; % position
    targetWayPoint(4:6,1) = expandPath.tangent(indexTargetWayPoint, :)'; % position
    
    
    % define the numinal control input u_n
    u_n = [params.m * params.g; 0; 0; 0];

    % reshape uVec so that u is a (params.nControlInputs)xN vector 
    N = size(uVec,1)/params.nControlInputs; % the controller considers the next N steps
    u = reshape(uVec,[params.nControlInputs,N]);
    
    % size of the state
    nStates = size(state0,1);
    
    % Make up a state vector for the N next steps
    states = zeros(nStates, N+1);
    states(:,1) = state0;
    
    %% Objective function is
    %%  J = params.C1 * Sum_{i=1}^n ||u_i||_2^2 + ||(pos_N - pos_d_N)||_2^2 + ||(psi_N - psi_d_N)||_2^2
    %   where pos represents position (x,y), psi represents heading and subscript d means desired
    
    % 1. First calculate the future states using Euler
    for i = 2:(N+1)
       states(:,i) = states(:,i-1) + params.timeStepMPC * params.plantDynamics(params, u(:,i-1), 0, states(:,i-1)); 
    end
    
    J = params.C1 * norm(u(:,1) - u_n)^2;
    for i = 2:N
        yaw = states(9,i);
        J = J + params.C1 * norm(u(:,i) - u_n)^2 ...
            + params.C4 * norm(yaw)^2;

    end
    
    % 2. Calculate the discrepencies with the target way point
    pos = states(1:3, N+1);
    vx = states(4,N+1);
    vy = states(5,N+1);
    vz = states(6,N+1);
    v = [vx; vy; vz];
    yaw = states(9,N+1);
    pos_d = targetWayPoint(1:3);
    vel_d = params.velocityTarget * targetWayPoint(4:6);
    J = J + params.C2 * norm(pos - pos_d)^2 + params.C5 * norm(v - vel_d)^2 ...
        + params.C4 * norm(yaw)^2;
    
    % 3. Now calculate the gradient
    derivMat = calculateDerivativeDrone(params, states, targetWayPoint, u);
    deriv = reshape(derivMat, [params.nControlInputs*N, 1]);
    
end


