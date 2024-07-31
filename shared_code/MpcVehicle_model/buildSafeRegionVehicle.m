function safeRegion = buildSafeRegionVehicle(params, path)

    % the safe region is defined by a collection of polygons (with 4 sides
    % each)

    x = zeros(params.nWayPoints, 2);
    y = zeros(params.nWayPoints, 2);
    % polygons: normal to the faces of the polygons, all directed within
    % the polygons or outside
    polygons = zeros(params.nWayPoints-1, 2 * 4);
    
    
    for i = 1:params.nWayPoints
        
        P1 = path.wayPoints(i,:) + path.normal(i,:)*params.positionTolerance;
        P2 = path.wayPoints(i,:) - path.normal(i,:)*params.positionTolerance;
        x(i,:) = [P1(1) P2(1)];
        y(i,:) = [P1(2) P2(2)];
        
    end
    
    for i = 1:(params.nWayPoints - 1)
        P1 = [x(i,1); y(i,1)];
        P2 = [x(i+1,1); y(i+1,1)];
        P3 = [x(i+1,2); y(i+1,2)];
        P4 = [x(i,2); y(i,2)];
        
        v1 = [0 -1; 1 0]*(P2 - P1);
        v2 = [0 -1; 1 0]*(P3 - P2);
        v3 = [0 -1; 1 0]*(P4 - P3);
        v4 = [0 -1; 1 0]*(P1 - P4);
        
        polygons(i,:) = [v1' v2' v3' v4'];
        
    end
    
    
    safeRegion.x = x;
    safeRegion.y = y;
    safeRegion.polygons = polygons;
     

end

