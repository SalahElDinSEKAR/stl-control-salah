function dx = vehicleODE(params, u, ~, x)
%vehicleODE Bicycle model of a vehicle with 
% states
%       x(1), x(2): x,y positions
%       x(3): Yaw angle (\psi)
%       x(4): velocity
% control inputs 
%       u(1): acceleration m/s^2
%       u(2): steering angle of front wheel
       
    dx = zeros(4,1);
    beta = atan(params.lr/(params.lr + params.lf) * tan(u(2)));
    dx(1) = x(4) * cos(x(3) + beta);
    dx(2) = x(4) * sin(x(3) + beta);
    dx(3) = x(4)/params.lr * sin(beta);
    dx(4) = u(1);
    
end

