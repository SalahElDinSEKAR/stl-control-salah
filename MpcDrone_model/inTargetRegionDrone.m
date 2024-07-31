function isInTargetRegion = inTargetRegionDrone(params, path, state)
    
    isInTargetRegion = true;

    x = state(1);
    y = state(2);
    z = state(3);
    
    if abs(x - path.wayPoints(end-1,1)) > params.positionToleranceInit
        isInTargetRegion = false;
    elseif abs(y - path.wayPoints(end-1,2)) > params.positionToleranceInit
        isInTargetRegion = false;
    elseif abs(z - path.wayPoints(end-1,3)) > params.positionToleranceInit
        isInTargetRegion = false;
    end

end

