function isInInitialSet = inInitialSetVehicle(params, path, state)

    isInInitialSet = true;

    x = state(1);
    y = state(2);
    v = state(4);
    
    if abs(x - path.wayPoints(2,1)) > params.positionToleranceInit
        isInInitialSet = false;
    elseif abs(y - path.wayPoints(2,2)) > params.positionToleranceInit
        isInInitialSet = false;
    elseif abs(v - params.velocityTarget) > params.velocityToleranceInit
        isInInitialSet = false;
    end


end

