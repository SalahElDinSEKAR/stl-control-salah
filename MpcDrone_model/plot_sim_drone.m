function plot_sim_drone(params, t, X, ax)
% plots a trace. If no ax is given, creates it

path = generateDesiredPath(params);

% build the safe region around the path
safeRegion = params.buildSafeRegion(params, path);

% path is expanded to authorize MPC at the end of path
expandPath = generateExpandPath(params, path);


if isempty(get(ax, 'Children'))
    setup_background();
end


  hold on;
  
  states = X(1:3,:)';
  state0 = X(1:3,1);
  plot3(states(:,1), states(:,2), states(:,3), 'b');
  plot3(state0(1,1), state0(2,1), state0(3,1), 'bx');
    


    function setup_background()
      
        Xx = safeRegion.x;
        Yy = safeRegion.y;
        Zz = safeRegion.z;

        % safe region
        s = surf(Xx,Yy,Zz,'FaceAlpha',0.35, 'FaceColor', [0.53 0.81 0.98]);
        s.EdgeColor = 'none';

        hold on

        % desired path
        plot3(path.wayPoints(2:end-1,1), path.wayPoints(2:end-1,2), path.wayPoints(2:end-1,3),'k');
        grid on;

    
      
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

view([40 10]);

axis equal
xlabel('$x (m)$','Interpreter','latex')
xticks(-2:1:2)
ylabel('$y (m)$','Interpreter','latex')
yticks(0:1:3)
zlabel('$z (m)$','Interpreter','latex')
yticks(0:1:5)

    
    
    
    
    
    
    end

end

