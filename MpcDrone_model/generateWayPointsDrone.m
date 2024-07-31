function [x, y, z] = generateWayPointsDrone(params)

    % generate way points for the drone

    x = zeros(params.nWayPoints,1);
    y = zeros(params.nWayPoints,1);
    z = zeros(params.nWayPoints,1);
    
    step = pi/(params.nWayPoints-1);
    t=0;
    for i = 1:params.nWayPoints
        x(i) = 2.5 * cos(t);
        y(i) = 2.5 * sin(t);
        z(i) = 5*(-1 + sin(t/2 + 3*pi/2));
        t = t + step;
    end

end

