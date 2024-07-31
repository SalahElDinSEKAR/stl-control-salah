function plot_sim_vehicle(params, t, X, ax)
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
  
  states = X(1:2,:)';
  state0 = X(1:2,1);
  plot(states(:,1), states(:,2), 'b');
  plot(state0(1,1), state0(2,1), 'bx');
    


    function setup_background()

        Xx = safeRegion.x;
        Y = safeRegion.y;
        Z = zeros(params.nWayPoints, 2);

        s = surf(Xx,Y,Z, 'FaceAlpha',0.35, 'FaceColor', [0.53 0.81 0.98]);
        s.EdgeColor = 'none';

        hold on
                
        plot(path.wayPoints(2:params.nWayPoints-1,1), path.wayPoints(2:params.nWayPoints-1,2), 'k');


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


        view(2)
        axis equal
        xlabel('$x (m)$','Interpreter','latex')
        xticks(0:10:30)
        ylabel('$y (m)$','Interpreter','latex')
        yticks(0:10:30)






    end

end

