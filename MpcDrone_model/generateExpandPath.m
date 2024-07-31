function expandPath = generateExpandPath(params, path)

    % tangent at the end of the path
    tangent = path.tangent(end,:);
    % length of path
    l = path.length;

    length_step = l/(params.nWayPoints-1);

    expandPath.wayPoints = [path.wayPoints; zeros(params.nWayPointsMPC, 3)];
    expandPath.tangent = [path.tangent; zeros(params.nWayPointsMPC, 3)];
    expandPath.normal = [path.normal; zeros(params.nWayPointsMPC, 3)];
    expandPath.binormal = [path.binormal; zeros(params.nWayPointsMPC, 3)];
    expandPath.heading = [path.heading; zeros(params.nWayPointsMPC, 2)];

    for i=(params.nWayPoints+1):(params.nWayPoints + params.nWayPointsMPC)

        expandPath.wayPoints(i,:) = expandPath.wayPoints(i-1,:) + length_step*tangent;
        expandPath.tangent(i,:) = expandPath.tangent(params.nWayPoints,:);
        expandPath.normal(i,:) = expandPath.normal(params.nWayPoints,:);
        expandPath.binormal(i,:) = expandPath.binormal(params.nWayPoints,:);
        expandPath.heading(i,:) = expandPath.heading(params.nWayPoints,:);

    end

end

