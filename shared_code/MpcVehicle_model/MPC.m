% following modifications in the original code to switch off the display
%   1. display option changed from 'iter' to 'off'
%   2. commented disp(fval);
function [u, fVal] = MPC(params, state0, path, expandPath, indexTargetWayPoint)

    distanceToTargetWP = params.calculateDistanceToWayPoint(state0, expandPath, indexTargetWayPoint);
    N = floor((distanceToTargetWP/params.velocityTarget)/params.timeStepMPC);
    
    
    u0 =  diag(params.lbControlInputs) * ones(params.nControlInputs,N) ...
        + diag(params.ubControlInputs - params.lbControlInputs) * ones(params.nControlInputs,N);

    % reshape vector u so that uVec is column vector
    uVec0 = reshape(u0, [params.nControlInputs*N,1]);
    
    JBest = 100000000.0;
    nTrials=5;
    % use of Matlab fmincon function for gradient descent
    options = optimoptions('fmincon', 'Display', 'off','SpecifyObjectiveGradient', true, 'MaxIterations', 300);

    % (params.nControlInputs)xN vector containing the lower bound for u
    lbOrig = diag(params.lbControlInputs) * ones(params.nControlInputs,N);
    % (params.nControlInputs)xN vector containing the upper bound for u
    ubOrig = diag(params.ubControlInputs) * ones(params.nControlInputs,N);
    % column vector containing the lower bound for u = (throttle, steering rate)
    lb = reshape(lbOrig,[params.nControlInputs*N,1]);
    % column vector containing the upper bound for u = (throttle, steering rate)
    ub = reshape(ubOrig,[params.nControlInputs*N,1]);
    
    
    % Randomly sample for an initialization that is the best among nTrials
    % sample. This seems to cut down the number of iterations in practice but does not
    % seem to have a big effect on the answer at least for the 4 state model.

    for i = 1:nTrials
        u = diag(params.lbControlInputs) * ones(params.nControlInputs,N) + ...
            diag(params.ubControlInputs - params.lbControlInputs) * rand(params.nControlInputs,N);
        uVec = reshape(u, [params.nControlInputs*N,1]);
        J = params.objectiveFunction(params, state0, expandPath, indexTargetWayPoint, uVec);
        if (J < JBest)
            uVec0 = uVec;
            JBest = J;
        end

    end
    
    % use of Matlab fmincon function for gradient descent
    [uVec,fVal] = fmincon(@(u) params.objectiveFunction(params, state0, expandPath, indexTargetWayPoint, u),uVec0,[],[], [], [], lb, ub,[], options);

    % disp(fVal);

    % inputs given by the Controller for next N steps
    u = reshape(uVec,[params.nControlInputs,N]);

end

