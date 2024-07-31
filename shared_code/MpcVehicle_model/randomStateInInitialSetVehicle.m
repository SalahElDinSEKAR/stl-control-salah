function state = randomStateInInitialSetVehicle(params, path)
    
    % random state inside the initial set

    % first way point of the desired path
    x_d = path.wayPoints(2,1);
    y_d = path.wayPoints(2,2);
    heading_d = path.heading(2,1);
    
    % random position in the initial set
     % random position in the initial set
    x_new = x_d - params.positionToleranceInit + 2*params.positionToleranceInit*rand;
    y_new = y_d - params.positionToleranceInit + 2*params.positionToleranceInit*rand;
    
    % random heading in the initial set
    heading_new = heading_d - params.headingToleranceInit + 2*params.headingToleranceInit*rand;
    
    % random velocity in the initial set
    velocity_new = params.velocityTarget - params.velocityToleranceInit + 2*params.velocityToleranceInit*rand;
    
    state = [x_new; y_new; heading_new; velocity_new];
 
end

