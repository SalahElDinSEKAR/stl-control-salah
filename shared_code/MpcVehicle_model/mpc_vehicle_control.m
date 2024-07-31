function [u cost] = mpc_vehicle_control(params, path,expandPath, x0)

     % find the closest way point
    [indexClosestWayPoint,~] = findClosestWayPoint(params, path, x0);

    % finding the target way point given the chosen initial state state0
    indexTargetWayPoint = indexClosestWayPoint+params.nWayPointsMPC;

    % computing control inputs u
    [u, cost] = MPC(params, x0, path, expandPath, indexTargetWayPoint);

end