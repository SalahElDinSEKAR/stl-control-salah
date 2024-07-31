function isInSafeRegion = inSafeRegionDrone(params, path, safeRegion, state)

    isInSafeRegion = false;

    P = [state(1);state(2);state(3)];
    
    x = safeRegion.x;
    y = safeRegion.y;
    z = safeRegion.z;
    volumes = safeRegion.volumes;
    
    for i = 1:(params.nWayPoints-1)
        
        listProjections = zeros(params.subdivision+2,1);
        
        for j = 1:(params.subdivision)
        
            P1 = [x(i,j); y(i,j); z(i,j)];
            
            v1 = P - P1;
            v2 = volumes(i, 3*(j-1)+1:3*j);
            
            c = v2*v1;

            listProjections(j) = c;
            
        end
        
        P1 = [x(i,1); y(i,1); z(i,1)];
        
        v1 = P - P1;
        v2 = volumes(i, 3*params.subdivision+1:3*params.subdivision+3);
        
        c1 = v2*v1;
        
        listProjections(params.subdivision + 1) = c1;
        
        P2 = [x(i+1,1); y(i+1,1); z(i+1,1)];
        
        v3 = P - P2;
        v4 =  volumes(i, 3*params.subdivision+4:3*params.subdivision+6);
        
        c2 = v4*v3;
        
        listProjections(params.subdivision + 2) = c2;

        positive = true;
        negative = true;
        for k = 1:(params.subdivision + 2)
            if listProjections(k) < 0
                positive = false;
            elseif listProjections(k) > 0
                negative = false;
            end
        end
        
        if or(negative,positive)
            isInSafeRegion = true;
        end
        
    end
    
    isInInitialSet = params.inInitialSet(params, path, state);
    isInTargetRegion = params.inTargetRegion(params, path, state);
    
    if isInInitialSet
        isInSafeRegion = true;
    elseif isInTargetRegion
        isInSafeRegion = true;
    end
    
    

end





