classdef MpcVehicle_imitation_pb < imitation_pb

    methods
        function pb = MpcVehicle_imitation_pb

            %% Creates class instance with name 
            pb= pb@imitation_pb('MpcVehicle_model');

            %% State space definition
                                    
            % nominal initial state
            pb.states.x.nominal=0.82;
            pb.states.y.nominal=1.53;
            pb.states.h.nominal=1.31;
            pb.states.v.nominal= 8.124;

            % initial ranges
            pb.states.x.range_init=[-0.5350,1.065];
            pb.states.y.range_init=[1.175,2.775];
            pb.states.h.range_init=[1.2032,1.6032];
            pb.states.v.range_init= [7.8,8.2];

            % input ranges
            pb.controls.u1.range = [-5;5];
            pb.controls.u2.range = [-.2;.2];

            % outputs
            pb.outputs.inSafeRegion = struct();

            % grid resolution
            % let it be init_range/10
            pb.states.x.grid_res = diff(pb.states.x.range_init)/10;
            pb.states.y.grid_res = diff(pb.states.y.range_init)/10;
            pb.states.h.grid_res = diff(pb.states.h.range_init)/10;
            pb.states.v.grid_res = diff(pb.states.v.range_init)/10;

            pb.controls.u1.grid_res = diff(pb.controls.u1.range)/10;
            pb.controls.u2.grid_res = diff(pb.controls.u2.range)/10;

            %% Control 
            % sim_mpc function
            params_mpc_vehicle = setParamsVehicle();
            path = generateDesiredPath(params_mpc_vehicle);
            
            % build the safe region around the path
            safeRegion = params_mpc_vehicle.buildSafeRegion(params_mpc_vehicle, path);
            
            % path is expanded to authorize MPC at the end of path
            expandPath = generateExpandPath(params_mpc_vehicle, path);
            pb.mpc_controlfn = @(x, options) mpc_vehicle_control(params_mpc_vehicle,path,expandPath, x);
                                    
            %% simulation
            Ts = 0.1;
            Tf = 10;
            time = 0:Ts:Tf;
            pb.time=  time;
            pb.sim_fn = @(Sys,time,p,controlfn)(sim_breach_vehicle(Sys,controlfn,params_mpc_vehicle, path, safeRegion,time,p));

            % plotting function (optional)
            pb.plot_sim = @(t,X,ax) (plot_sim_vehicle(params_mpc_vehicle, t,X,ax));

            %% Requirements
            STL_ReadFile('req_vehicle.stl');
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
            str = ['Res_Vehicle_' datestr(now,'dd-mmm-yyyy-HH_MM_SS') '.mat'];
        end



    end
end