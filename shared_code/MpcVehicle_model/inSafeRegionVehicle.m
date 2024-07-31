function isInSafeRegion = inSafeRegionVehicle(params, path, safeRegion, state)

    isInSafeRegion = false;
    isInSafeRegionPosition = false;
          
    isInInitialSet = params.inInitialSet(params, path, state);
    isInTargetRegion = params.inTargetRegion(params, path, state);
    
    if isInInitialSet || isInTargetRegion
        isInSafeRegion = true;
    else
    

        P = [state(1);state(2)];
        v = state(4);
        
        %test on the position

        x = safeRegion.x;
        y = safeRegion.y;

        for i = 1:(params.nWayPoints-1)

            P1 = [x(i,1); y(i,1)];
            P2 = [x(i+1,1); y(i+1,1)];
            P3 = [x(i+1,2); y(i+1,2)];
            P4 = [x(i,2); y(i,2)];

            v1 = P - P1;
            v2 = P - P2;
            v3 = P - P3;
            v4 = P - P4;

            n1 = safeRegion.polygons(i,1:2);
            n2 = safeRegion.polygons(i,3:4);
            n3 = safeRegion.polygons(i,5:6);
            n4 = safeRegion.polygons(i,7:8);

            c1 = n1*v1;
            c2 = n2*v2;
            c3 = n3*v3;
            c4 = n4*v4;

            if (sign(c1)==sign(c2) && sign(c2)==sign(c3) && sign(c3)==sign(c4))
                isInSafeRegionPosition = true;
            end

        end
        
        %test on the velocity
        
        if and(abs(v - params.velocityTarget) <= params.velocityTolerance,isInSafeRegionPosition)
            isInSafeRegion = true;
        end
        
    end
    
    
end



