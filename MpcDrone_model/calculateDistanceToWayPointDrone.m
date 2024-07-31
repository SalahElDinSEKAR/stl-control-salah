function distance = calculateDistanceToWayPointDrone(state, path, indexPath)
    
    % calculate the distance between a point P and a way point WP
    % point P is defined by 'state'
    % way point WP is defined by 'path' and 'indexPath'


    x = state(1);
    y = state(2);
    z = state(3);
    
    x_wp = path.wayPoints(indexPath, 1);
    y_wp = path.wayPoints(indexPath, 2);
    z_wp = path.wayPoints(indexPath, 3);
    
    distance = sqrt((x - x_wp)^2 + (y - y_wp)^2 + (z - z_wp)^2);


end

