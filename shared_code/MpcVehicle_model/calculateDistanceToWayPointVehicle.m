function distance = calculateDistanceToWayPointVehicle(state, path, indexPath)
    
    % calculate the distance between a point P and a way point WP
    % point P is defined by 'state'
    % way point WP is defined by 'path' and 'indexPath'


    x = state(1);
    y = state(2);
    
    x_wp = path.wayPoints(indexPath, 1);
    y_wp = path.wayPoints(indexPath, 2);
    
    distance = sqrt((x - x_wp)^2 + (y - y_wp)^2);


end

