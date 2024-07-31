function [J, deriv] = objectiveFunctionVehicle(params, state0, expandPath, indexTargetWayPoint, uVec)

    % define target way point
    targetWayPoint = zeros(3,1);
    targetWayPoint(1:2,1) = expandPath.wayPoints(indexTargetWayPoint, 1:2)';
    targetWayPoint(3,1) = expandPath.heading(indexTargetWayPoint, 1);

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
    
    % calculate first part of objective function J 
    
    J = params.C1 * norm(u)^2;
    
    for i = 2:N
        J = J + params.C4 * norm(states(4,i) - params.velocityTarget)^2;
    end
    
    % 2. Calculate the discrepencies with the target way point
    pos = states(1:2, N+1);
    psi  = states(3, N+1);
    vel  = states(4,N+1);
    xd  = targetWayPoint(1:2);
    psid = targetWayPoint(3);
    veld = params.velocityTarget;
    J = J + params.C2 * norm(pos - xd)^2 + params.C3 * norm(psi - psid)^2 + params.C4 * norm(vel - veld)^2;
    
    % 3. Now calculate the gradient
    derivMat = calculateDerivativeVehicle(params, states, targetWayPoint, u);
    deriv = reshape(derivMat, [params.nControlInputs*N, 1]);
    
end


