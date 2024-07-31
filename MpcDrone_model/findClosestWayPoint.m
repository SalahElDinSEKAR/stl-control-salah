function [indexClosestWayPoint, distanceClosestWayPoint] = findClosestWayPoint(params, path, state)
    
    % finding the closest way point from a given point
    
    indexClosestWayPoint = 1;
    minimalDistance = params.calculateDistanceToWayPoint(state, path, 1);
    distanceClosestWayPoint = minimalDistance;
    
    for i = 2:size(path.wayPoints,1)
        
        distance = params.calculateDistanceToWayPoint(state, path, i);
        
        if distance < minimalDistance

            indexClosestWayPoint = i;
            minimalDistance = distance;
            distanceClosestWayPoint = minimalDistance;
            
        end
        
    end

end

