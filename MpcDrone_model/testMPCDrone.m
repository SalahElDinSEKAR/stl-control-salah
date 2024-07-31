% loading parameters and path
params = setParamsDrone();
path = generateDesiredPath(params);

% build the safe region around the path
safeRegion = params.buildSafeRegion(params, path);

% path is expanded to authorize MPC at the end of path
expandPath = generateExpandPath(params, path);


% plots

positionFig = figure(1);
    
X = safeRegion.x;
Y = safeRegion.y;
Z = safeRegion.z;

% safe region
s = surf(X,Y,Z,'FaceAlpha',0.35, 'FaceColor', [0.53 0.81 0.98]);
s.EdgeColor = 'none';

hold on

% desired path
plot3(path.wayPoints(2:end-1,1), path.wayPoints(2:end-1,2), path.wayPoints(2:end-1,3),'b');

% others

maxTime = 0;

velFig = figure(2);

phiFig = figure(3);

thetaFig = figure(4);

psiFig = figure(5);

dotPhiFig = figure(6);

dotThetaFig = figure(7);

dotPsiFig = figure(8);


rng(0);
for m = 1:1

    % choosing random initial state inside safe region
    state0 = params.randomStateInInitialSet(params, path);

    % boolean variables used to stop the simulation if the target
    % region is reached or if the trajectory leaves the safe region
    % (= falsified trace)
    isInSafeRegion = true;
    isInTargetRegion = false;

    %simulation
    N = params.maxDurationSimu; % maximum duration of the simulation, in terms of control steps
    nStates = size(state0,1);
    x = zeros(51*N,nStates);
    x(1,:) = state0';
    x0 = state0;
    times = zeros(51*N,1);

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
        [ts, st] = ode45(odefun, 0:(params.timeStepMPC/50):params.timeStepMPC, x0);

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

    states(count:51*N,:) = [];
    times(count:51*N,:) = [];

    end
    
    
    % plots

    %position
    
    figure(1)

    hold on

    plot3(states(:,1), states(:,2), states(:,3), 'r');

    hold on

    plot3(state0(1,1), state0(2,1), state0(3,1), 'bx');
    
    % others
    
    finalTime = times(end,1);
    if finalTime > maxTime
        maxTime = finalTime;
    end
    
    figure(2)
    
    
    v = zeros(size(times,1),1);
    for l = 1:size(times,1)
        v(l,1) = sqrt(states(l,4)^2 + states(l,5)^2 + states(l,6)^2);
    end
    
    hold on
    
    plot(times(:,1), v(:,1), 'b');
    
    figure(3)
    
    hold on
    
    plot(times, states(:,7), 'b');
    
    figure(4)
    
    hold on
    
    plot(times, states(:,8), 'b');
    
    figure(5)
    
    hold on
    
    plot(times, states(:,9), 'b');
    
    figure(6)
    
    hold on
    
    plot(times, states(:,10), 'b');
    
    figure(7)
    
    hold on
    
    plot(times, states(:,11), 'b');
    
    figure(8)
    
    hold on
    
    plot(times, states(:,12), 'b');
    

end


% plots

% position

figure(1)

x_init = path.wayPoints(2,1);
y_init = path.wayPoints(2,2);
z_init = path.wayPoints(2,3);

x_target = path.wayPoints(end-1,1);
y_target = path.wayPoints(end-1,2);
z_target = path.wayPoints(end-1,3);

a = params.positionToleranceInit;

% initial set

x_initialSet = [x_init-a x_init-a x_init-a x_init-a x_init-a;...
    x_init+a x_init+a x_init+a x_init+a x_init+a];

y_initialSet = [y_init-a y_init-a y_init+a y_init+a y_init-a;...
    y_init-a y_init-a y_init+a y_init+a y_init-a];

z_initialSet = [z_init-a z_init+a z_init+a z_init-a z_init-a;...
    z_init-a z_init+a z_init+a z_init-a z_init-a];

x_initialSet2 = [x_init-a x_init-a];
x_initialSet3 = [x_init+a x_init+a];

y_initialSet2 = [y_init-a y_init+a];
y_initialSet3 = [y_init-a y_init+a];


z_initialSet2 = [z_init-a z_init+a; z_init-a z_init+a];
z_initialSet3 = [z_init-a z_init+a; z_init-a z_init+a];



s1 = surf(x_initialSet, y_initialSet, z_initialSet, 'FaceAlpha',0.2, 'FaceColor', [0.93 0.46 0.13]); % centered at (x_init, y_init, z_init)
s1.EdgeColor = [0.45 0.45 0.45];
hold on
s2 = surf(x_initialSet2, y_initialSet2, z_initialSet2, 'FaceAlpha',0.2, 'FaceColor', [0.93 0.46 0.13]); % centered at (x_init, y_init, z_init)
s2.EdgeColor = 'none';
hold on
s3 = surf(x_initialSet3, y_initialSet3, z_initialSet3, 'FaceAlpha',0.2, 'FaceColor', [0.93 0.46 0.13]); % centered at (x_init, y_init, z_init)
s3.EdgeColor = 'none';
hold on

% target region

x_targetRegion = [x_target-a x_target-a x_target-a x_target-a x_target-a;...
    x_target+a x_target+a x_target+a x_target+a x_target+a];

y_targetRegion = [y_target-a y_target-a y_target+a y_target+a y_target-a;...
    y_target-a y_target-a y_target+a y_target+a y_target-a];

z_targetRegion = [z_target-a z_target+a z_target+a z_target-a z_target-a;...
    z_target-a z_target+a z_target+a z_target-a z_target-a];

x_targetRegion2 = [x_target-a x_target-a];
x_targetRegion3 = [x_target+a x_target+a];

y_targetRegion2 = [y_target-a y_target+a];
y_targetRegion3 = [y_target-a y_target+a];


z_targetRegion2 = [z_target-a z_target+a; z_target-a z_target+a];
z_targetRegion3 = [z_target-a z_target+a; z_target-a z_target+a];


s4 = surf(x_targetRegion, y_targetRegion, z_targetRegion, 'FaceAlpha',0.2, 'FaceColor', [0.93 0.46 0.13]); % centered at (x_target, y_target, z_target)
s4.EdgeColor = [0.45 0.45 0.45];
hold on
s5 = surf(x_targetRegion2, y_targetRegion2, z_targetRegion2, 'FaceAlpha',0.2, 'FaceColor', [0.93 0.46 0.13]); % centered at (x_target, y_target, z_target)
s5.EdgeColor = 'none';
hold on
s6 = surf(x_targetRegion3, y_targetRegion3, z_targetRegion3, 'FaceAlpha',0.2, 'FaceColor', [0.93 0.46 0.13]); % centered at (x_target, y_target, z_target)
s6.EdgeColor = 'none';


grid off

view([40 10]);

axis equal
xlabel('$x (m)$','Interpreter','latex')
xticks(-2:1:2)
ylabel('$y (m)$','Interpreter','latex')
yticks(0:1:3)
zlabel('$z (m)$','Interpreter','latex')
yticks(0:1:5)

figure(2)
xlim([0 maxTime])

figure(3)
xlim([0 maxTime])

figure(4)
xlim([0 maxTime])

figure(5)
xlim([0 maxTime])

figure(6)
xlim([0 maxTime])

figure(7)
xlim([0 maxTime])

figure(8)
xlim([0 maxTime])



