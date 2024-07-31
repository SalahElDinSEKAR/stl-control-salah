function path = generateDesiredPath(params)

    % path is a structure containing 6 fields:
    % the coordinates (x,y,z) of the way points, the tangent unit vector, the normal
    % and binormal of the space curve x,y,z, the heading of the space
    % curve (x,y,z), and the length of the path
    
    % generate a list of way points
    [x, y, z] = params.generateWayPoints(params);
    
    % calculate the Frenet frame for the space curve x,y,z
    [t, n, b] = frenet(x,y,z);
    
    % calculate the heading for the space curve x,y,z
    [theta, phi] = heading(t);
    
    path.wayPoints = [x y z];
    path.tangent = t;
    path.normal = n;
    path.binormal = b;
    path.heading = [theta, phi];
    
    % calculate the length of the path
    l = 0;
    for i = 1:(params.nWayPoints-1)
        dl = norm(path.wayPoints(i+1,:) - path.wayPoints(i,:));
        l = l + dl;
    end

    path.length = l;
    
end

