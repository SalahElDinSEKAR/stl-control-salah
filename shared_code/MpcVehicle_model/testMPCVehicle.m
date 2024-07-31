% loading parameters and path
params = setParamsVehicle();
path = generateDesiredPath(params);

% build the safe region around the path
safeRegion = params.buildSafeRegion(params, path);

% path is expanded to authorize MPC at the end of path
expandPath = generateExpandPath(params, path);

% plots

% 1) position
figure(1)

% safe region

X = safeRegion.x;
Y = safeRegion.y;
Z = zeros(params.nWayPoints, 2);

s = surf(X,Y,Z, 'FaceAlpha',0.35, 'FaceColor', [0.53 0.81 0.98]);
s.EdgeColor = 'none'; 

hold on

% desired path

plot(path.wayPoints(2:params.nWayPoints-1,1), path.wayPoints(2:params.nWayPoints-1,2), 'b');


% 2) velocity

figure(2)

maxTime = 0;

tf = params.maxDurationSimu*params.timeStepMPC;

% safe region

ubVelocity = params.velocityTarget + params.velocityTolerance;
lbVelocity = params.velocityTarget - params.velocityTolerance;
vsafeRegion = [ ubVelocity ubVelocity; lbVelocity lbVelocity];
tsafeRegion = [0 tf ; 0 tf];
zsafeRegion = [0 0;0 0];

sv = surf(tsafeRegion, vsafeRegion, zsafeRegion, 'FaceAlpha',0.35, 'FaceColor', [0.53 0.81 0.98]);
sv.EdgeColor = 'none'; 

vel = params.velocityTarget*ones(params.maxDurationSimu+1,1);

hold on

plot(0:params.timeStepMPC:tf, vel(:,1), 'b');

% 3) heading

figure(3)


for m = 1:10

    % choosing random initial state inside initial set
    state0 = params.randomStateInInitialSet(params, path);

    % boolean variables used to stop the simulation if the target
    % region is reached or if the trajectory leaves the safe region
    % (= falsified trace)
    isInSafeRegion = true;
    isInTargetRegion = false;

    %simulation
    N = params.maxDurationSimu; % maximum duration of the simulation, in terms of control steps
    nStates = size(state0,1);
    x = zeros(11*N,nStates);
    x(1,:) = state0';
    x0 = state0;
    times = zeros(11*N,1);

    %counters
    count = 1;
    i = 1;

    while (i <= N && isInSafeRegion && not(isInTargetRegion))

        % find the closest way point
        [indexClosestWayPoint,~] = findClosestWayPoint(params, path, x0);

        % finding the target way point given the chosen initial state state0
        indexTargetWayPoint = indexClosestWayPoint+params.nWayPointsMPC;

        % computing control inputs u
        u = MPC(params, x0, path, expandPath, indexTargetWayPoint);

        odefun = @(t,x) params.plantDynamics(params, u(:,1), t, x);
        [ts, st] = ode45(odefun, 0:(params.timeStepMPC/10):params.timeStepMPC, x0);

        k = size(ts,1);
        times(count:count+k-1,1) = (i-1)*params.timeStepMPC * ones(k,1)+ ts;
        x(count:count+k-1,:) = st;
        x0 = x(count+k-1,:)';

        % decide if the simulation should be stopped and determine whether the
        % trace is falsified or not
        isInSafeRegion = params.inSafeRegion(params, path, safeRegion, x0);

        isInTargetRegion = false;
        for q = 1:k

            res = params.inTargetRegion(params, path, st(q,:)');
            if res
                isInTargetRegion = true;
            end

        end

        %update the counters
        count = count + k ;
        i = i+1;
    end

    states = x;

    if i < N

    states(count:11*N,:) = [];
    times(count:11*N,:) = [];

    end

    % plots

    % 1) position
    
    figure(1)

    hold on

    plot(states(:,1), states(:,2), 'r');

    hold on

    plot(state0(1,1), state0(2,1), 'mo');

    % 2) velocity

    figure(2)
    
    finalTime = times(end,1);
    if finalTime > maxTime
        maxTime = finalTime;
    end
    
    hold on

    plot(times, states(:,4), 'r');

    hold on

    plot(0, state0(4,1), 'bx');

    % 3) heading

    figure(3)
    
    hold on

    plot(times, states(:,3), 'r');

    hold on

    plot(0, state0(3,1), 'bx');
    
end

% options for plots

% 1) position
    
figure(1);

hold on

x_init = path.wayPoints(2,1);
y_init = path.wayPoints(2,2);

x_target = path.wayPoints(end-1,1);
y_target = path.wayPoints(end-1,2);

a = params.positionToleranceInit;

% initial set


xInitialSet = [x_init-a x_init-a; x_init+a x_init+a];
yInitialSet = [y_init-a y_init+a; y_init-a y_init+a];
zInitialSet = [0 0;0 0];

spi = surf(xInitialSet, yInitialSet, zInitialSet, 'FaceAlpha',0.2, 'FaceColor', [0.93 0.46 0.13]);
spi.EdgeColor = [0.45 0.45 0.45];

hold on

% target region

xtargetRegion = [x_target-a x_target-a; x_target+a x_target+a];
ytargetRegion = [y_target-a y_target+a; y_target-a y_target+a];
ztargetRegion = [0 0;0 0];

spt = surf(xtargetRegion, ytargetRegion, ztargetRegion, 'FaceAlpha',0.2, 'FaceColor', [0.93 0.46 0.13]);
spt.EdgeColor = [0.45 0.45 0.45];


grid off
view(2)
axis equal
xlabel('$x (m)$','Interpreter','latex')
xticks(0:10:30)
ylabel('$y (m)$','Interpreter','latex')
yticks(0:10:30)

% 2) velocity

figure(2)
grid off
view(2)
axis equal
xlim([0 maxTime])
xlabel('$t (s)$','Interpreter','latex')
ylabel('$v (m.s^{-1})$','Interpreter','latex')

% 3) heading

figure(3)
grid off
view(2)
axis equal



