function [t,X,p, status] = sim_breach_mpc_vehicle(t, p, params, path, safeRegion, expandPath)

status = 0; 
%counter
x0 = p;
x_mpc = [];
u_mpc = [];

isInSafeRegion = true;
isInTargetRegion = false;

inSafeRegion = [];

i = 1;


Ts = params.timeStepMPC;
N = t(end)/Ts;

t_ode = 0:(Ts/10):Ts;
t = 0:Ts/10:t(end);
X = nan(7, numel(t));


while (i <= N && isInSafeRegion && not(isInTargetRegion))

    % find the closest way point
    [indexClosestWayPoint,~] = findClosestWayPoint(params, path, x0);

    % finding the target way point given the chosen initial state state0
    indexTargetWayPoint = indexClosestWayPoint+params.nWayPointsMPC;

    % computing control inputs u
    u = MPC(params, x0, path, expandPath, indexTargetWayPoint);

    odefun = @(t,x) params.plantDynamics(params, u(:,1), t, x);
    [~, st] = ode45(odefun, t_ode, x0);

    x_mpc = [x_mpc;st(1:end-1,:)];
    x0 = x_mpc(end,:)';

    for j = 1:size(st,1)-1
        inSafeRegion = [inSafeRegion;params.inSafeRegion(params, path, safeRegion, st(j,:))];
        u_mpc = [u_mpc;u(:,1)'];
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
u_mpc = [u_mpc;u(:,1)'];


X(:, 1:size(x_mpc,1)) = [x_mpc inSafeRegion u_mpc]';
