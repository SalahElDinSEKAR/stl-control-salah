function state = randomStateInInitialSetDrone(params, path)
    
    x_d = path.wayPoints(2,1);
    y_d = path.wayPoints(2,2);
    z_d = path.wayPoints(2,3);
    t_d = path.heading(2,1);
    p_d = path.heading(2,2);
    vel_d = params.velocityTarget;
    
    % random position in the initial set
    x_new = x_d - params.positionToleranceInit + 2*params.positionToleranceInit*rand;
    y_new = y_d - params.positionToleranceInit + 2*params.positionToleranceInit*rand;
    z_new = z_d - params.positionToleranceInit + 2*params.positionToleranceInit*rand;
    
    % random velocity in the neighborhood of the falsified initial state
    t_new = t_d - params.thetaToleranceInit + 2 * params.thetaToleranceInit * rand;
    p_new = p_d - params.phiToleranceInit + 2 * params.phiToleranceInit * rand;
    vel_new = vel_d - params.velocityToleranceInit + 2 * params.velocityToleranceInit * rand;
    dotx_new = cos(p_new)*cos(t_new)*vel_new;
    doty_new = cos(p_new)*sin(t_new)*vel_new;
    dotz_new = sin(p_new)*vel_new;
    
    % random attitude in the neighborhood of the falsified initial state
    phi_new = - params.maxPhi + 2*params.maxPhi*rand;
    theta_new = - params.maxTheta + 2*params.maxTheta*rand;
    psi_new = - params.maxPsi + 2*params.maxPsi*rand;
    
    % random attitude rates in the neighborhood of the falsified initial state
    dotphi_new =  - params.maxDotPhi + 2*params.maxDotPhi*rand;
    dottheta_new =  - params.maxDotTheta + 2*params.maxDotTheta*rand;
    dotpsi_new =  - params.maxDotPsi + 2*params.maxDotPsi*rand;

    state = [x_new; y_new; z_new; dotx_new; doty_new; dotz_new; phi_new; theta_new; psi_new; dotphi_new; dottheta_new; dotpsi_new];
     
end

