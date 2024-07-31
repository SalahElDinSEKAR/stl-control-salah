function deriv = calculateDerivativeDrone(params, states, targetWayPoint, u)

    nStates = size(states,1);
    N = size(u,2); % number of columns in u, the next N time steps are considered
    
    lam = zeros(nStates,N+1); %% lam(:,j) refers to dJ/dx[j]
    deriv = zeros(params.nControlInputs, N);
    
    
    % define the numinal control input u_n
    u_n = [params.m * params.g; 0; 0; 0];
    
    v1 = zeros(nStates,1);
    
    % position
    
    %  \partialJ/\partialx
    v1(1,1) = 2* params.C2 * states(1,N+1) - 2*params.C2* targetWayPoint(1); 
    % \partialJ/\partialy
    v1(2,1) = 2* params.C2 * states(2,N+1) - 2*params.C2*targetWayPoint(2);
    % \partialJ/\partialz
    v1(3,1) = 2* params.C2 * states(3,N+1) - 2*params.C2*targetWayPoint(3);
    
    % velocity
    
    
    % \partialJ/\partialvx
    v1(4,1) = 2 * params.C5 * states(4,N+1) - 2*params.C5*targetWayPoint(4);
    % \partialJ/\partialvy
    v1(5,1) = 2 * params.C5 * states(5,N+1) - 2*params.C5*targetWayPoint(5);
    % \partialJ/\partialvz
    v1(6,1) = 2 * params.C5 * states(6,N+1) - 2*params.C5*targetWayPoint(6);
    
    % yaw angle
    v1(9,1) = 2 * params.C4 * states(9,N+1);

    lam(:,N+1) = v1; 

    for n = N:-1:1
        %% \partialx[n+1]/\partial x[n], \partial x[n+1]/\partian u[n]
        
        
        v1 = zeros(nStates,1);
        
        % yaw angle
        v1(9,1) = 2 * params.C4 * states(9,n);

        
        % Jacobian of plant model with respect to the state and control input u
        [D, U] = params.computeJacobianPlant(params, u(:,n), states(:,n));
        lam(:,n) = v1 + (eye(nStates) + params.timeStepMPC* D')* lam(:,n+1); 
        
        deriv(:,n) = 2 * params.C1 * (u(:,n) - u_n) + params.timeStepMPC *  U'*lam(:,n+1); 
        
    end

end


