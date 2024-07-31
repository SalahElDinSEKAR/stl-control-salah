function [x, y, z] = generateWayPoints(params)

    % generate way points along a quadratic Bezier path

    x = zeros(params.nWayPoints,1);
    y = zeros(params.nWayPoints,1);
    z = zeros(params.nWayPoints,1);
    
    step = 1/(params.nWayPoints-1);
    t=0;
    for i = 1:params.nWayPoints
        x(i) = params.P0(1)*(1-t)^2 + params.P1(1)*2*t*(1-t) + params.P2(1)*t^2;
        y(i) = params.P0(2)*(1-t)^2 + params.P1(2)*2*t*(1-t) + params.P2(2)*t^2;
        z(i) = params.P0(3)*(1-t)^2 + params.P1(3)*2*t*(1-t) + params.P2(3)*t^2;
        t = t + step;
    end

end

