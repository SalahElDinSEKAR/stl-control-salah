classdef MpcDrone_imitation_pb < imitation_pb

    methods
        function pb = MpcDrone_imitation_pb

            pb= pb@imitation_pb('MpcDrone_model');

            % sim_mpc function
            params_mpc_drone = setParamsDrone();
            path = generateDesiredPath(params_mpc_drone);

            % first waypoint
            x_wp = path.wayPoints(2,1);
            y_wp = path.wayPoints(2,2);
            z_wp = path.wayPoints(2,3);
            heading_t = path.heading(2,1);
            heading_p = path.heading(2,2);
            vel_target = params_mpc_drone.velocityTarget;

            % params
            pb.params.heading_t.nominal = heading_t;
            pb.params.heading_t.range = heading_t + [-1 1] * params_mpc_drone.thetaToleranceInit;

            pb.params.heading_p.nominal = heading_p;
            pb.params.heading_p.range = heading_p + [-1 1]*params_mpc_drone.phiToleranceInit;

            pb.params.velocity_target.nominal = vel_target;
            pb.params.velocity_target.range = vel_target+[-1 1]*params_mpc_drone.velocityToleranceInit;

            % states
            pb.states.x.nominal=x_wp;
            pb.states.x.range_init=x_wp + [-1 1]*params_mpc_drone.positionToleranceInit;
            pb.states.x.grid_res = diff(pb.states.x.range_init)/10;

            pb.states.y.nominal=y_wp;
            pb.states.y.range_init=y_wp + [-1 1]*params_mpc_drone.positionToleranceInit;
            pb.states.y.grid_res = diff(pb.states.y.range_init)/10;

            pb.states.z.nominal=z_wp;
            pb.states.z.range_init=z_wp + [-1 1]*params_mpc_drone.positionToleranceInit;

            %% grid resolution for x, y, z
            pb.states.x.grid_res = diff(pb.states.x.range_init)/10;
            pb.states.y.grid_res = diff(pb.states.y.range_init)/10;
            pb.states.z.grid_res = diff(pb.states.z.range_init)/10;


            %% These values will be replaced anyway (functions of heading_t, heading_p and vel_target)
            pb.states.x_dot.nominal=0;
            pb.states.x_dot.range_init=[];
            pb.states.y_dot.nominal=0;
            pb.states.y_dot.range_init=[];
            pb.states.z_dot.nominal=0;
            pb.states.z_dot.range_init=[];

            % cannot be relative to x_dot range
            pb.states.x_dot.grid_res = .01;
            pb.states.y_dot.grid_res = .01;
            pb.states.z_dot.grid_res = .01;

            %% phi theta psi
            pb.states.phi.nominal= 0;
            pb.states.phi.range_init=[-1 1]*params_mpc_drone.maxPhi;

            pb.states.theta.nominal= 0;
            pb.states.theta.range_init= [-1 1]*params_mpc_drone.maxTheta;

            pb.states.psi.nominal=0;
            pb.states.psi.range_init=[-1 1]*params_mpc_drone.maxPsi;

            pb.states.phi_dot.nominal=0;
            pb.states.phi_dot.range_init=[-1 1]*params_mpc_drone.maxDotPhi;

            pb.states.theta_dot.nominal=0;
            pb.states.theta_dot.range_init= [-1 1]*params_mpc_drone.maxDotPhi;

            pb.states.psi_dot.nominal=0;
            pb.states.psi_dot.range_init=[-1 1]*params_mpc_drone.maxDotPhi;

            pb.states.phi.grid_res = diff(pb.states.phi.range_init)/10;
            pb.states.theta.grid_res = diff(pb.states.theta.range_init)/10;
            pb.states.psi.grid_res = diff(pb.states.psi.range_init)/10;

            pb.states.phi_dot.grid_res = diff(pb.states.phi_dot.range_init)/10;
            pb.states.theta_dot.grid_res = diff(pb.states.theta_dot.range_init)/10;
            pb.states.psi_dot.grid_res = diff(pb.states.psi_dot.range_init)/10;


            % input ranges
            pb.controls.u1.range = [0 .7];
            pb.controls.u2.range = [-pi/8 pi/8];
            pb.controls.u3.range = [-pi/8 pi/8];
            pb.controls.u4.range = [-pi/8 pi/8];

            pb.controls.u1.grid_res = diff(pb.controls.u1.range)/10;
            pb.controls.u2.grid_res = diff(pb.controls.u2.range)/10;
            pb.controls.u3.grid_res = diff(pb.controls.u3.range)/10;
            pb.controls.u4.grid_res = diff(pb.controls.u4.range)/10;

            % outputs
            pb.outputs.inSafeRegion = struct();

            %% simulation
            Ts = 0.1;
            Tf = 10;
            time = 0:Ts:Tf;
            pb.time= time;

            % sim_mpc function       

            % build the safe region around the path
            safeRegion = params_mpc_drone.buildSafeRegion(params_mpc_drone, path);

            % path is expanded to authorize MPC at the end of path
            expandPath = generateExpandPath(params_mpc_drone, path);

            pb.mpc_controlfn = @(x,options) mpc_drone_control(params_mpc_drone,path,expandPath, x);

            pb.sim_fn = @(Sys,time,p,controlfn)(sim_breach_drone(Sys, controlfn,params_mpc_drone, path, safeRegion,time,p));

            % plotting function

            pb.plot_sim = @(t,X,ax) (plot_sim_drone(params_mpc_drone, t,X,ax));

            %% Requirements

            STL_ReadFile('req_drone.stl');
            pb.phi = BreachRequirement('phi_safety');

        end

        function options = setup_training_options(this, Bvalid)

            if nargin==1
                Bvalid = this.get_mpc_traces('num_corners',0,'num_quasi_random',10,'seed', 2000);
            end

            [in_valid, out_valid] = this.prepare_training_data(Bvalid);
            valid_cell_array = {in_valid,out_valid};
            options = trainingOptions('adam', ...
                'Verbose', false, ...
                'Plots','training-progress',...% 'none', ...
                'Shuffle', 'every-epoch', ...
                'MiniBatchSize', 512, ...
                'ValidationData', valid_cell_array, ...
                'InitialLearnRate', 1e-3, ...
                'ExecutionEnvironment', 'cpu', ...
                'GradientThreshold', 10, ...
                'MaxEpochs', 40 ...
                );
        end

      function layers = setup_layers(this)
            numObservations = numel(fieldnames(this.states));
            numActions = numel(fieldnames(this.controls));

            hiddenLayerSize = 256;
            umax = 5;

            layers = [
                featureInputLayer(numObservations,'Normalization','none','Name','observation')
                fullyConnectedLayer(hiddenLayerSize,'Name','fc1')
                reluLayer('Name','relu1')
                fullyConnectedLayer(hiddenLayerSize,'Name','fc2')
                reluLayer('Name','relu2')
                fullyConnectedLayer(hiddenLayerSize,'Name','fc3')
                reluLayer('Name','relu3')
                fullyConnectedLayer(hiddenLayerSize,'Name','fc4')
                reluLayer('Name','relu4')
                fullyConnectedLayer(hiddenLayerSize,'Name','fc5')
                reluLayer('Name','relu5')
                fullyConnectedLayer(hiddenLayerSize,'Name','fc6')
                reluLayer('Name','relu6')
                fullyConnectedLayer(numActions,'Name','fcLast')
                tanhLayer('Name','tanhLast')
                scalingLayer('Name','ActorScaling','Scale',umax)
                regressionLayer('Name','routput')];
        end


        function str = get_result_file_name(this, res)
            str = ['Res_Drone_' datestr(now,'dd-mmm-yyyy-HH_MM_SS') '.mat'];
        end

        function [X, err] = get_params_from_X(this, X)

            Y = X(4:6,:);
            
            %Y(1) = cos(p_d)*cos(t_d)*vel_d;
            %Y(2) = cos(p_d)*sin(t_d)*vel_d;
            %Y(3) = sin(p_d)*vel_d;
            
            % we have Y, and we need P = (p_d, t_d, vel_d)       
            [P, err] = fminunc(@err_fn, 0*Y);

            X = [X; P];
            function err = err_fn(P)
                 P_d = P(1,:);
                 T_d = P(2,:);
                 VEL_d = P(3,:);
                 Ytilde(1,:) = cos(P_d).*cos(T_d).*VEL_d;
                 Ytilde(2,:) = cos(P_d).*sin(T_d).*VEL_d;
                 Ytilde(3,:) = sin(P_d).*VEL_d;
                err = norm(Y-Ytilde);
            end
        end


    end
end