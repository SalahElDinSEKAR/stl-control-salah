classdef PidWatertank_imitation_pb < imitation_pb

    methods
        function pb = PidWatertank_imitation_pb

            %% Creates class instance with name 
            pb= pb@imitation_pb('PidWatertank_model');

            %% State space definition
                                    
            % nominal initial state
            pb.states.H.nominal = 8.2;
            pb.states.Hp.nominal = 8.2;
            pb.states.Hpp.nominal = 8.2;
            pb.states.ref.nominal = 8.2;
            pb.states.refp.nominal = 8.2;
            pb.states.refpp.nominal = 8.2;
            pb.states.Vp.nominal = 0;

            % initial ranges
            pb.states.H.range_init = [8,12]; 
            pb.states.Hp.range_init = [];
            pb.states.Hpp.range_init = [];
            pb.states.ref.range_init = [8,12];
            pb.states.refp.range_init = [];
            pb.states.refpp.range_init = [];
            pb.states.Vp.range_init = [-10 10];

            % input ranges
            pb.controls.V.range = [-10 10]; 

            % params
            pb.params.ref1.nominal = 10;
            pb.params.ref1.range_init = [8, 12]; % ref changes at time t1 to ref1

            % outputs
%          pb.outputs.inSafeRegion = struct();

            % grid resolution
            % let it be init_range/10
            pb.states.H.grid_res = diff(pb.states.H.range_init)/10;
            pb.states.Hp.grid_res = diff(pb.states.Hp.range_init)/10;
            pb.states.Hpp.grid_res = diff(pb.states.Hpp.range_init)/10;
            pb.states.ref.grid_res = diff(pb.states.ref.range_init)/10;
            pb.states.refp.grid_res = diff(pb.states.refp.range_init)/10;
            pb.states.refpp.grid_res = diff(pb.states.refpp.range_init)/10;

            pb.controls.V.grid_res = diff(pb.controls.V.range)/10;

            %% Control 

           % pb.mpc_controlfn = @(H) pid_watertank_control(H);
           pb.mpc_controlfn = @(H) final_control(H);
                                    
            %% simulation
            Ts = 0.1;
            Tf = 20;
            time = 0:Ts:Tf;
            pb.time=  time;
            pb.sim_fn = @(Sys,time,p,controlfn)(sim_breach_watertank(controlfn, time, p));

            % plotting function (optional)
             % pb.plot_sim = @(t,X,ax) (plot_sim_vehicle(params_mpc_vehicle, t,X,ax));

            %% Requirements
            STL_ReadFile('req_watertank.stl');
            pb.phi = BreachRequirement('phi');

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
            umax = 10;

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
ref

        function str = get_result_file_name(this, res)
            str = ['Res_Watertank_' datestr(now,'dd-mmm-yyyy-HH_MM_SS') '.mat'];
        end



    end
end