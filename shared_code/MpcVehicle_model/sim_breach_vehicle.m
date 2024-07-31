function [t,X,p,status] = sim_breach_vehicle(Sys,controlfn, params, path, safeRegion, t, p)

status = 0; 
%counters
x0 = p(Sys.DimX+1:end);
x_mpc = [];
u_mpc = [];

isInSafeRegion = true;
isInTargetRegion = false;

inSafeRegion = [];

i = 1;

Ts = params.timeStepMPC;
N = t(end)/Ts;

k = 2;  % do we need more times ? not sure.
t_ode = 0:(Ts/k):Ts;
t = 0:Ts/k:t(end);
X = nan(8, numel(t));
cost_mpc = [];

while (i <= N && isInSafeRegion && not(isInTargetRegion))
    
    [u, cost] = controlfn(x0);
    if size(u,1)==1
        u=u';
    end

    odefun = @(t,x) params.plantDynamics(params, u(:,1), t, x);
    [~, st] = ode45(odefun, t_ode, x0);

    x_mpc = [x_mpc;st(1:end-1,:)];
    x0 = x_mpc(end,:)';

    for j = 1:size(st,1)-1
        inSafeRegion = [inSafeRegion;params.inSafeRegion(params, path, safeRegion, st(j,:))];
        u_mpc = [u_mpc;u(:,1)'];
        cost_mpc = [cost_mpc; cost];
    end

    % decide if the simulation should be stopped and determine whether the
    % trace is falsified or not
    isInSafeRegion = params.inSafeRegion(params, path, safeRegion, x0);

    isInTargetRegion = false;
    for q = 1:size(st,1)

        res = params.inTargetRegion(params, path, st(q,:)');
        if res
            isInTargetRegion = true;
        end

    end

    %update the counters
    i = i+1;
end

% last state
x_mpc = [x_mpc;st(end,:)]; 
inSafeRegion = [inSafeRegion;params.inSafeRegion(params, path, safeRegion, st(end,:))];
cost_mpc = [cost_mpc ;  cost];
u_mpc = [u_mpc;u(:,1)'];


X(:, 1:size(x_mpc,1)) = [x_mpc inSafeRegion u_mpc cost_mpc]';
