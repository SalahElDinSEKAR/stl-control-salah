function [t, X]= sim_breach_flying(control_fn, t , p)

X = zeros(8, numel(t));
X(1:8, 1) = p(end-7:end); % 1:DimX are initial conditions if DimX is the dimensionality of ODE
Ts = t(2)-t(1);
opt = []; 
for k = 1:numel(t)-1
    xk = X(1:6,k);
    lastMV = X(7:8,k);
        
    [uk, cost, opt] = control_fn([xk; lastMV], opt);
       
    X(9:10,k) = uk;
    X(11,k) = cost; 
    ODEFUN = @(t,xk) ControlFlyingRobotStateFcn(xk,uk);
    [~,YOUT] = ode45(ODEFUN,[0 Ts], xk);
    % Store the state values.
    X(1:6,k+1) = YOUT(end,:)';
    X(7:8,k+1) = uk;
end
X(9:10,end) = uk;
X(11,end) =cost;
end

