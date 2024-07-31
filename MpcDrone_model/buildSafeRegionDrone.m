function safeRegion = buildSafeRegionDrone(params, path)

    % the safe region is defined by a collection of volumes 

    x_wp = path.wayPoints(:,1);
    y_wp = path.wayPoints(:,2);
    z_wp = path.wayPoints(:,3);
    % volumes: normal to the face of the volumes, all directed within the
    % volumes or outside
    volumes = zeros(params.nWayPoints - 1, 3*(2+params.subdivision));
    
    [x, y, z] = tube(x_wp, y_wp, z_wp, params.positionTolerance, params.subdivision);
    
    for i = 1:(params.nWayPoints - 1)
        
        for j = 1:(params.subdivision)
            P1 = [x(i,j); y(i,j); z(i,j)];
            P2 = [x(i,j+1); y(i,j+1); z(i,j+1)];
            P3 = [x(i+1,j); y(i+1,j); z(i+1,j)];
            
            v1 = P2 - P1;
            v2 = P3 - P1;
            
            v = cross(v1, v2);
            
            volumes(i, 3*(j-1)+1:3*j) = v';
            
            
        end
        
        P1 = [x(i,1); y(i,1); z(i,1)];
        P2 = [x(i,2); y(i,2); z(i,2)];
        P3 = [x(i,3); y(i,3); z(i,3)];
        
        v1 = P3 - P2;
        v2 = P2 - P1;
        
        v = cross(v1, v2);
        
        volumes(i, 3*params.subdivision+1:3*params.subdivision+3) = v'; 
        
        P4 = [x(i+1,1); y(i+1,1); z(i+1,1)];
        P5 = [x(i+1,2); y(i+1,2); z(i+1,2)];
        P6 = [x(i+1,3); y(i+1,3); z(i+1,3)];
        
        v3 = P6 - P5;
        v4 = P5 - P4;
        
        v = cross(v4, v3);
        
        volumes(i, 3*params.subdivision+4:3*params.subdivision+6) = v'; 
        
    end

    
    safeRegion.x = x;
    safeRegion.y = y;
    safeRegion.z = z;
    safeRegion.volumes = volumes;
     

end
