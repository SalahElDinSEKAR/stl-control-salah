classdef BreachVehicleSys < BreachSystem

    properties
        nn
    end

    methods
        function this = BreachVehicleSys(nn)

            signals = {'x', 'y', 'h', 'v', 'inSafeRegion', 'u1', 'u2'};
            params ={'x_0', 'y_0', 'h_0', 'v_0'};
            
            params_mpc_vehicle = setParamsVehicle();

            path = generateDesiredPath(params_mpc_vehicle);
            
            % build the safe region around the path
            safeRegion = params_mpc_vehicle.buildSafeRegion(params_mpc_vehicle, path);
            
            % path is expanded to authorize MPC at the end of path
            expandPath = generateExpandPath(params_mpc_vehicle, path);

            % Def p0
            x0 = [0.8200    1.5300   1.310  8.124]';
            p0 = x0;

            this = this@BreachSystem('vehicle_with_NN', ...
                signals,...
                params,...
                p0)

            this.nn = nn;
            this.Sys.sim = @(Sys,t,p)(this.sim_breach_vehicle_neuron(t,p, ...
                         params_mpc_vehicle, path, safeRegion, expandPath));
           
            Ts = 0.01;
            Tf = 15;
            time = 0:Ts:Tf;
            time = [time Tf];
            this.SetTime(time);

          
            %% Default Range
            %this.SetParamRanges({'x1_0', 'x2_0'}, [-4,4]);
            %this.SetParamRanges({'theta_0'}, [-3.2,3.2]);
            %this.SetParamRanges('v1_0', [-2 2 ]);
            %this.SetParamRanges('v2_0', [-2 2 ]);
            %this.SetParamRanges({'omega_0'}, [-1,1]);            
            %this.SetParamRanges({'u1_0', 'u2_0'}, [-umax umax]);

        end

        function SetInitRanges(this)
%             umax = 3;
            this.SetParamRanges({'x_0'}, [-0.5350,1.065]);
            this.SetParamRanges({'y_0'}, [1.175,2.775]);
            this.SetParamRanges({'h_0'}, [1.2032,1.6032]);
            this.SetParamRanges({'v_0'}, [7.8,8.2]);
%             this.SetParamRanges({'u1_0', 'u2_0'}, [-umax umax]);
        end

        function SetSafeRanges(this)
            umax = 3;
            this.SetParamRanges({'x1_0', 'x2_0'}, [-10,10]);
            this.SetParamRanges({'theta_0'}, [-3.2,3.2]);
            this.SetParamRanges('v1_0', [-10 10 ]);
            this.SetParamRanges('v2_0', [-10 10 ]);
            this.SetParamRanges({'omega_0'}, [-5,5]);            
            this.SetParamRanges({'u1_0', 'u2_0'}, [-umax umax]);
        end

        function [t, X,p] = sim_breach_vehicle_neuron(this, t,p, ...
                         params_mpc_vehicle, path, safeRegion, expandPath)            
            x0 = p;
            Tf = t(end);

            Ts = 0.01;
            t = 0:Ts:Tf;

            [xNN, inSafeRegion, uNN, times] = simModelVehicle(x0,this.nn,Ts,Tf, ...
                params_mpc_vehicle, path, safeRegion, expandPath);

            X = [xNN'; inSafeRegion' ;...
                uNN'];  % u_next
            
            t = times';

            if t(end)<Tf
                t(end+1) = Tf;
            end

            function [xHistory, inSafeRegion, uHistory, times] = simModelVehicle(x0,NN,Ts,Tf, ...
                    params, path, safeRegion, expandPath)
                % Performs closed-loop simulation of tracking control and simulates the
                % system for |Tf/Ts+1| steps with DNN
                %
                % Copyright 2019 The MathWorks, Inc.

                x0 = p;
                xHistory = [];
                uHistory = [];
            
                isInSafeRegion = true;
                isInTargetRegion = false;
                
                inSafeRegion = [];
            
                count = 1;
                i = 1;
                N = tf/0.1;
                while (i <= N && not(isInTargetRegion))
                    feature = x0;
                    % computing control inputs u
                    u = predict(NN, feature,'ExecutionEnvironment','cpu');
                    
                    odefun = @(t,x) params.plantDynamics(params, u(:,1), t, x);
                    [ts, st] = ode45(odefun, 0:(params.timeStepMPC/10):params.timeStepMPC, x0);
            
                    k = size(ts,1);
                    times(count:count+k-1,1) = (i-1)*params.timeStepMPC * ones(k,1)+ ts;
                    x(count:count+k-1,:) = st;
                    x0 = x(count+k-1,:)';
                    
                    for j = count:count+k-1
                        inSafeRegion = [inSafeRegion;params.inSafeRegion(params, path, safeRegion, x(j,:))];
                        u_mpc = [u_mpc;u(:,1)'];
                    end
            
                    % decide if the simulation should be stopped and determine whether the
                    % trace is falsified or not
            
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
            
                xHistory = x;
            end


        end



    end
end