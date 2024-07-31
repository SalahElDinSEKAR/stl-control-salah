function isInTargetRegion = inTargetRegionVehicle(params, path, state)
    
    isInTargetRegion = true;

    x = state(1);
    y = state(2);
    v = state(4);
    
    if abs(x - path.wayPoints(end-1,1)) > params.positionToleranceInit
        isInTargetRegion = false;
    elseif abs(y - path.wayPoints(end-1,2)) > params.positionToleranceInit
        isInTargetRegion = false;
    elseif abs(v - params.velocityTarget) > params.velocityToleranceInit
        isInTargetRegion = false;
    end

end

