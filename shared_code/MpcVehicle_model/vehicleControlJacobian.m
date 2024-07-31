function [D, U] = vehicleControlJacobian(params, u, x)

    % vehicleODE model of a vehicle with 
    % states
    %       x(1), x(2): x,y positions
    %       x(3): Yaw angle (\psi)
    %       x(4): velocity
    % control inputs 
    %       u(1): acceleration m/s^2
    %       u(2): steering angle of front wheel

    % D: partial derivatives with respect to state variables x(1), x(2),
    % x(3), and x(4)
    D = zeros(4,4);
    % U: partial derivatives with respect to input variables u(1) and u(2)
    U = zeros(4,2);
    
    beta0 = params.lr/(params.lr + params.lf) * tan(u(2));
    beta = atan(beta0);
    beta0deriv = params.lr/(params.lr + params.lf) * sec(u(2)) * sec(u(2)) ;
    betaderiv =  beta0deriv / (1 + beta0^2);
    
    U(1,2) = -x(4) * sin(x(3) + beta) * betaderiv;
    D(1,3) = - x(4) * sin(x(3) + beta); D(1,4) = cos(x(3) + beta);
    U(2,2) = x(4) * cos(x(3) + beta) * betaderiv;
    D(2,3) = x(4)  * cos(x(3)+beta); D(2,4) = sin(x(3)+beta);
    U(3,2) = x(4)/params.lr * betaderiv * cos(beta);
    D(3,4) = 1.0/params.lr * sin(beta);
    U(4,1) = 1;
    
    % reminder: vehicleODE model
    
    %beta = atan(params.lr/(params.lr + params.lf) * tan(u(2)));
    %dx(1) = x(4) * cos(x(3) + beta);
    %dx(2) = x(4) * sin(x(3) + beta);
    %dx(3) = x(4)/params.lr * sin(beta);
    %dx(4) = u(1);
end
