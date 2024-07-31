function isInInitialSet = inInitialSetDrone(params, path, state)

    isInInitialSet = true;

    x = state(1);
    y = state(2);
    z = state(3);
    
    if abs(x - path.wayPoints(2,1)) > params.positionToleranceInit
        isInInitialSet = false;
    elseif abs(y - path.wayPoints(2,2)) > params.positionToleranceInit
        isInInitialSet = false;
    elseif abs(z - path.wayPoints(2,3)) > params.positionToleranceInit
        isInInitialSet = false;
    end


end

