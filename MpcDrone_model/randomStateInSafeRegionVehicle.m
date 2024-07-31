function state = randomStateInSafeRegionVehicle(params, path)
    
    % find a random state inside the safe region
    
    % choose a random point along the desired trajectory
    % 1) choose a random distance from the beginning of the path
    dist = 0.22 * path.length;
    
    % 2) find the coordinates of the corresponding point
    l = 0;
    index = 1;
    while l < dist
        dl = norm(path.wayPoints(index+1,:) - path.wayPoints(index,:));
        l = l + dl;
        index = index + 1;
    end
    
    position_mean = path.wayPoints(index,:) ...
        - (path.wayPoints(index-1,:) - path.wayPoints(index,:))/norm(path.wayPoints(index-1,:) - path.wayPoints(index,:))...
        *(l-dist);
    
    x_mean = position_mean(1);
    y_mean = position_mean(2);
    
    % 3) find the heading at the corresponding point
    heading_mean = path.heading(index,1);
    
    % 4) generate a random state within the authorized tolerances around
    % this point
    distance_to_path = params.positionTolerance*1;
    alpha = -pi/8;%2*pi*rand;
    x = x_mean + distance_to_path*cos(alpha);
    y = y_mean + distance_to_path*sin(alpha);
    heading = heading_mean + 1.5*params.headingTolerance;
    velocity = params.velocityTarget + 0*params.velocityTolerance;
    
    state = [x;y;heading;velocity];
    
end

