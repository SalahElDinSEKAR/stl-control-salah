function deriv = calculateDerivativeVehicle(params, states, targetWayPoint, u)

    nStates = size(states,1);
    N = size(u,2); % number of columns in u, the next N time steps are considered
    
    lam = zeros(nStates,N+1); %% lam(:,j) refers to dJ/dx[j]
    deriv = zeros(params.nControlInputs, N);
    
    v1 = zeros(nStates,1);
    %  \partialJ/\partialx
    v1(1,1) = 2* params.C1 * states(1,N+1) - 2*params.C1* targetWayPoint(1); 
    % \partialJ/\partialy
    v1(2,1) = 2* params.C2 * states(2,N+1) - 2*params.C2*targetWayPoint(2);
    % \partialJ/\partialpsi
    v1(3,1) = 2* params.C3 * states(3,N+1) - 2*params.C3*targetWayPoint(3);
    % \partialJ/\partialv
    v1(4,1) = 2* params.C4 * states(4,N+1) - 2*params.C4*params.velocityTarget;

    lam(:,N+1) = v1; 

    for n = N:-1:1
        %% \partialx[n+1]/\partial x[n], \partial x[n+1]/\partian u[n]
        
        % Jacobian of plant model with respect to the state and control input u
        
        v1 = zeros(nStates,1);
        v1(4,1) = 2* params.C4 * states(4,n) - 2*params.C4*params.velocityTarget;

        
        [D, U] = params.computeJacobianPlant(params, u(:,n), states(:,n));
        lam(:,n) = v1 + (eye(nStates) + params.timeStepMPC* D')* lam(:,n+1); 
        
        deriv(:,n) = 2 * params.C1 * u(:,n) + params.timeStepMPC *  U'*lam(:,n+1); 
        
    end

end


