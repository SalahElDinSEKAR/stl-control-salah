classdef imitation_pb

    % data
    properties
        name
        params   % constant parameters
        states   % time varying variable with initial state
        controls % time varying controller inputs
        outputs  %  additional signals, computed from states and controls, no initial value required
    end

    % simulation
    properties
        time
        mpc_controlfn
        sim_fn
        plot_sim
    end

    % Requirements
    properties
        phi % BreachRequirement objet, used to evaluate the controllers
    end  

    % misc
    properties
        data_folder ='data'
        parallel = true
    end

    methods

 %% Basic methods

        % Contructor, minimalist. All fields for a given problem have to be
        % filled in a separate function, see init_drone.m, init_vehicle.m,
        % etc.
        function this = imitation_pb(name)
            this.name = name;
            this.params =struct();
            this.states = struct();
            this.controls = struct();
            this.outputs = struct();
        end
       
        function B0 = create_nominal(this, scaling) 
        %     B0 = create_nominal(this [, scaling]) 
        %    Creates a  breach set for simulation with mpc, with default ranges
        %    and nominal value 
        %    scaling is a scalar to scale the initial domain

        if nargin <2
            scaling =1;
        end

            B0 = this.create_mpc_sys();
           
            signals =  [fieldnames(this.states)' fieldnames(this.outputs)' fieldnames(this.controls)' 'cost'];
            params_params = fieldnames(this.params)';
            params_states = cellfun(@(c)([c '_0']), fieldnames(this.states)', 'UniformOutput', 0);

            % param ranges
            for ip = 1:numel(params_params)
                param  = params_params{ip};
                if isfield(this.params.(param),'range')
                    range = this.params.(param).range;
                    if ~isempty(range)
                        if ~isempty(range)
                            center = (range(1)+range(2))/2;
                            epsi = (range(2)-range(1))/2;
                            scaled_range = [center-epsi*scaling center+epsi*scaling];
                            B0.SetParamRanges(param, scaled_range)
                        end
                    end
                end
            end

            % states ranges
            for ip = 1:numel(params_states)
                param  = params_states{ip};
                state = signals{ip};
                if isfield(this.states.(state),'range_init')
                    range = this.states.(state).range_init;   
                    if ~isempty(range)
                        center = (range(1)+range(2))/2;
                        epsi = (range(2)-range(1))/2;
                        scaled_range = [center-epsi*scaling center+epsi*scaling];
                        B0.SetParamRanges(param, scaled_range)
                    end
                end
                if isfield(this.states.(state),'range_max')
                    domain = BreachDomain('double', this.states.(state).range_max);
                    B0.SetDomain(state, domain);                    
                end            
            
            end

        end

        function Bmpc = create_mpc_sys(this)
        % creates system using the mpc controller, without setting initial
        % ranges
            signals =  [fieldnames(this.states)' fieldnames(this.outputs)' fieldnames(this.controls)' 'cost'];
            params_params = fieldnames(this.params)';
            params_states = cellfun(@(c)([c '_0']), fieldnames(this.states)', 'UniformOutput', 0);

            p0_params = cell2mat( cellfun(@(c)(this.params.(c).nominal), fieldnames(this.params)', 'UniformOutput', 0 ))';
            p0_states = cell2mat( cellfun(@(c)(this.states.(c).nominal), fieldnames(this.states)', 'UniformOutput', 0 ))';

            sim_mpc =@(Sys,time, p) (this.sim_fn(Sys,time,p, this.mpc_controlfn));

            Bmpc = BreachSystem([this.name '_nominal_control'], signals,...
                [params_states params_params],[p0_states; p0_params],...
                sim_mpc);
            Bmpc.SetTime(this.time);
            Bmpc.SetInitFn(['init_paths(''' this.name ''')']);
        end
        
        % create a breach set with samples in the param and state space of
        % the system 
        function B = create_state_space_set(this)
            params_params = fieldnames(this.params)';
            params_states = fieldnames(this.states)';

            p0_params = cell2mat( cellfun(@(c)(this.params.(c).nominal), fieldnames(this.params)', 'UniformOutput', 0 ))';
            p0_states = cell2mat( cellfun(@(c)(this.states.(c).nominal), fieldnames(this.states)', 'UniformOutput', 0 ))';
            
            B = BreachSet([params_states params_params]);
            
            % param ranges
            for ip = 1:numel(params_params)
                param  = params_params{ip};
                if isfield(this.params.(param),'range')
                    range = this.params.(param).range;
                    if ~isempty(range)
                        B.SetParamRanges(param, range)
                    end
                end
            end

            % states ranges
            for ip = 1:numel(params_states)
                state = params_states{ip};
                if isfield(this.states.(state),'range_init')
                    range = this.states.(state).range_init;
                    if ~isempty(range)
                        B.SetParamRanges(state, range)
                    end
                end
            end

            B.SetParam([params_states params_params], [p0_params;p0_states]);
         
        end

        % Creates a breach set for simulation with a neural network, with
        % default initial ranges
        function Bnn = create_nn_sys(this, nn, scaling)
            if nargin<3
                scaling=1;
            end
            
            Bnn = this.create_nominal(scaling);
            nn_controlfn= @(x,opt)(this.nn_controlfn(nn, x));
            sim_nn = @(Sys,time, p) (this.sim_fn(Sys,time,p, nn_controlfn));
            Bnn.Sys.sim = sim_nn;
            Bnn.Sys.name = [this.name '_NN_control'];
        end
        
        % Controller using neural net
        function [u, cost, opt] = nn_controlfn(this, nn, x)
            u = predict(nn, x','ExecutionEnvironment','cpu');    % what else ?            
            cost = 0;
            opt = []; 
        end

        % Creates a nominal Breach set, run a simulation, checks phi and
        % plots result
        function  [R, G ] = test_nominal(this, B0,R)

            if nargin<=1
                B0 = this.create_nominal();
            end

            if nargin<3
                R =this.phi.copy();
            end
            B = B0.copy();
            B.Sim();
            if isempty(this.plot_sim)
                signals = B.GetSignalNames();
                G = BreachSignalsPlot(B,signals(1:3));
            else
                this.plot_traces(B);
            end
            if R.Eval(B) > 0
                disp('PASSED.')
            else
                disp('FAILED.')
            end

        end

        function ax = plot_traces(this, B)
            traces = B.GetTraces();

            traj = traces{1};
            figure;
            ax = gca;
            for i = 1:numel(traces)
                traj = traces{i};
                this.plot_sim(traj.time, traj.X,ax);
            end
        end
      
        % Generate initial data and perform training once 
        function [nn, res] = train_default_init(this)
            fprintf('\n-- Step 1 -----------------------------------------------------')
            disp('Getting simulations with mpc.')
            res.Bmpc_train = this.get_mpc_traces();
            res.Bmpc_val = this.get_mpc_traces('num_corners',0, 'num_quasi_random', 10, 'seed', 2000);

            fprintf('\n-- Step 2 ------------------------------------------------------')
            disp('Getting grid data from simulations...')
            res.Btrain = this.get_grid_data_from_traces(res.Bmpc_train);
            res.Bval = this.get_grid_data_from_traces(res.Bmpc_val);

            [in_data_train, out_data_train] = prepare_training_data(this,res.Btrain);
            [in_data_val, out_data_val] = prepare_training_data(this,res.Bval);
            valid_cell_array = {in_data_val, out_data_val};

            fprintf('\n-- Step 3 ------------------------------------------------------')
            disp('Training...')

            res.options = trainingOptions('adam', ...
                'Verbose', false, ...
                'Plots','training-progress',...% 'none', ...
                'Shuffle', 'every-epoch', ...
                'MiniBatchSize', 512, ...
                'ValidationData', valid_cell_array, ...
                'InitialLearnRate', 1e-3, ...
                'ExecutionEnvironment', 'cpu', ...
                'GradientThreshold', 10, ...
                'MaxEpochs', 10 ...
                );

            layers =  this.setup_layers();

            nn = trainNetwork(in_data_train,out_data_train, layers , res.options);
            res.nn  =nn;
        end
        

        %% Main algo 
        % Counter example based algo with falsification using quasirandom
        % sampling, ends after about a thousand quasi random simulations
        % cannot falsify the neural network
        function all_results = algo1(this, varargin)
            
            % initial pool of mpc traces
            opts.init_sampling.num_corners =16 ;
            opts.init_sampling.num_quasi_random = 84;
            opts.init_sampling.seed = 1;           
            
            % pool of validation data for training 
            opts.valid_sampling.num_corners = 0;
            opts.valid_sampling.num_quasi_random = 10;
            opts.valid_sampling.seed = 100000;           
            
            % pool of initial states for counter example search
            opts.falsif_sampling.num_quasi_random = 10; 
            opts.falsif_sampling.seed = 10000;  
            
            opts.num_training  = 10; % number of outer loop iteration
            opts.num_falsif = 10;       % number of consecutive falsification attempts

            opts.max_num_cex_traces = 5;
            opts.max_num_cex_samples = 10; 

            opts.save_file_name = [this.data_folder filesep this.get_result_file_name()];
            opts.resume_results = {};
            opts.diary_file=  ['diary_run_' datestr(now,'dd-mmm-yyyy-HH_MM_SS.txt') ];

            opts = varargin2struct_breach(opts, varargin{:});
            
            diary(opts.diary_file);

            fprintf('Will log results in %s\n', opts.save_file_name);
            fprintf('Saving matlab outputs in %s\n', opts.diary_file);

            init_sampling = opts.init_sampling;
            valid_sampling = opts.valid_sampling;
            falsif_sampling = opts.falsif_sampling;
            num_training = opts.num_training;
            num_falsif = opts.num_falsif; 
            max_num_cex_traces = opts.max_num_cex_traces;
            max_num_cex_samples = opts.max_num_cex_samples; 
            all_results = opts.resume_results;

            %%  
            if isfile(opts.save_file_name)
                bool  = input('File %s exists, continue and override ?', opts.save_file_name );
                if ~bool
                    return;
                end
            end
            
            save(opts.save_file_name,'all_results');              

            if isempty(all_results)

                %% Step 1
                fprintf('\n-- Step 1 -----------------------------------------------------\n')
                disp('Getting simulations with mpc.')
                Btrain_traces = this.get_mpc_traces(init_sampling);
                Bval_traces = this.get_mpc_traces(valid_sampling);
                Bcex_fix_traces = this.get_mpc_traces(falsif_sampling);                  

                %% Step 2
                fprintf('\n-- Step 2 ------------------------------------------------------\n')
                disp('Getting grid data from simulations...')
                [Btrain, Btrain_grid] = this.get_grid_data_from_traces(Btrain_traces);
                Bval = this.get_grid_data_from_traces(Bval_traces);
                all_results = {}; % or not if we resume... (TODO)
            else
                last_res = all_results{end};
                Btrain_traces = last_res.Btrain_traces;
                Bcex_fix_traces = last_res.Bcex_fix_traces;
                %Btrain_grid = last_res.Btrain_grid;
                Bval = last_res.Bval;                      
            end
            
             for iter = 1:num_training
                               
                %% Step 3
                fprintf(['\n-- Step 3 -- ITER ' num2str(iter) ' ----------------------------------------------------\n'])
                disp('TRAINING...') 
                this.disp_all_results(all_results);
                   
                Btrain_traces.Concat(Bcex_fix_traces);
                [Btrain, Btrain_grid] =this.get_grid_data_from_traces(Btrain_traces);
                
                [in_data_train, out_data_train] = prepare_training_data(this,Btrain);
                [in_data_val, out_data_val] = prepare_training_data(this,Bval);
                valid_cell_array = {in_data_val, out_data_val};
                train_options  = this.setup_training_options();
                train_options.ValidationData= valid_cell_array;

                layers =  this.setup_layers();
                nn = trainNetwork(in_data_train,out_data_train, layers , train_options);
                res.nn=nn;
                
                %% Step 4 
                fprintf(['\n-- Step 4 -- ITER ' num2str(iter) ' ----------------------------------------------------\n'])
                disp('Looking for counter examples ...')
                for false_iter = 1:num_falsif
                    
                    fprintf(['\nAttempt ' num2str(false_iter) '...\n']);                    
                    [Bcex_nn_traces, ~ , Rcex] = get_nn_cex_traces(this, nn, falsif_sampling); 
                    
                    if ~isempty(Bcex_nn_traces)
                        break;
                    else
                        falsif_sampling.seed = falsif_sampling.seed+falsif_sampling.num_quasi_random+1;                        
                    end
                                                                                                
                end

                if isempty(Bcex_nn_traces)
                    disp('SUCCESS.')
                    break;
                end

                delete(findall(0)); % close training window
                %% Step 5
                fprintf(['\n-- Step 5 -- ITER ' num2str(iter) ' ----------------------------------------------------\n'])
                disp('Extract counter examples data, compute mpc fix and merge with previous data...')
                
                [Bcex_fix_traces]  =this.fix_cex(Btrain_grid, Bcex_nn_traces,  max_num_cex_traces,  max_num_cex_samples);                

                res.Bcex_nn_traces = Bcex_nn_traces.copy();
                res.Rcex = Rcex; 
                res.Bcex_fix_traces = Bcex_fix_traces.copy();
                res.Bcex_fix_samples =this.get_grid_data_from_traces(Bcex_fix_traces); % for info only
                res.Btrain = Btrain.copy();
                res.Btrain_grid = Btrain_grid.copy();                
                res.Bval = Bval.copy();  % should that ever change ?
                all_results{end+1}= res;
                save(opts.save_file_name,'all_results');                             
                                
            end

        end

        %% Algo 2: matching paper pseudo code
        function all_results = algo2(this, varargin)
            
            % initial pool of mpc traces
            opts.init_sampling.num_corners =26 ;
            opts.init_sampling.num_quasi_random = 84;
            opts.init_sampling.seed = 1;           
            
            % pool of validation data for training 
            opts.valid_sampling.num_corners = 0;
            opts.valid_sampling.num_quasi_random = 10;
            opts.valid_sampling.seed = 100000;           
            
            % pool of initial states for counter example search
            opts.falsif_sampling.num_quasi_random = 10; 
            opts.falsif_sampling.seed = 10000;  
            opts.falsif_sampling.max_num_cex = 10; 
            opts.num_training  = 10; % number of outer loop iteration
            opts.num_falsif = 10;       % number of consecutive falsification attempts
opts.num_random_traces = 10;
            opts.max_num_cex_traces = 10;
            opts.max_num_cex_samples = 1; 

            opts.save_file_name = [this.data_folder filesep this.get_result_file_name()];
            opts.resume_results = {};
            opts.diary_file=  ['logs' filesep 'diary_run_' datestr(now,'dd-mmm-yyyy-HH_MM_SS.txt') ];

            opts = varargin2struct_breach(opts, varargin{:});
            
            diary(opts.diary_file);

            fprintf('Will log results in %s\n', opts.save_file_name);
            fprintf('Saving matlab outputs in %s\n', opts.diary_file);

            init_sampling = opts.init_sampling;
            valid_sampling = opts.valid_sampling;
            falsif_sampling = opts.falsif_sampling;
            num_training = opts.num_training;
            
            falsif_sampling.max_num_cex = opts.max_num_cex_traces;
            max_num_cex_traces = opts.max_num_cex_traces;
            max_num_cex_samples = opts.max_num_cex_samples; 
            
            all_results = opts.resume_results;

            %%  
            if isfile(opts.save_file_name)
                bool  = input('File %s exists, continue and override ?', opts.save_file_name );
                if ~bool
                    return;
                end
            end            
            
            for iter = 1:num_training

                fprintf('\n-- Iter %s ------------------------------------------------------\n', num2str(iter))
                %% Getting new data
                if isempty(all_results) 
                    %initial iteration, no data yet
                    disp('Generate initial traces...')
                    Btrain_traces = this.get_mpc_traces(init_sampling);
                    Bval_traces = this.get_mpc_traces(valid_sampling);
                    [Btrain, Btrain_grid] = this.get_grid_data_from_traces(Btrain_traces);
                    Bval = this.get_grid_data_from_traces(Bval_traces);
                    
                    res.Btrain_traces = Btrain_traces;
                    res.Bval_traces = Bval_traces;
                    res.Btrain= Btrain;
                    res.Btrain_grid = Btrain_grid;
                    res.Bval = Bval;
                else                    
                    this.disp_all_results(all_results);
                    % Looking for counter examples                    
                    prev_res = all_results{end};
                    Bval = prev_res.Bval; 
                    nn = prev_res.nn;
                    disp('Looking for counter examples...')
                    [Bcex_nn_traces , Rcex] = this.falsify(nn, falsif_sampling);                  
                    
               
                    if ~isempty(Bcex_nn_traces)
                        % found cex 
                        opts_rand.num_random_traces = opts.num_random_traces;
                        Brand_traces=  this.get_nn_rand_traces(nn, opts_rand);
                        Bcex_nn_traces.Concat(Brand_traces, 1);

                        disp('Extract counter examples data, compute mpc fix and merge with previous data...')
                        Btrain_grid = prev_res.Btrain_grid.copy();                     
                        [Bcex_fix_traces] = this.fix_cex(Btrain_grid, Bcex_nn_traces,  max_num_cex_traces,  max_num_cex_samples);
                        

                        if ~isempty(Bcex_fix_traces)
                            Btrain_traces = prev_res.Btrain_traces.copy();
                            Btrain_traces.Concat(Bcex_fix_traces, true); % fast concat 
                            [Btrain, Btrain_grid] =this.get_grid_data_from_traces(Btrain_traces);

                            % update prev_res
                            prev_res.Bcex_nn_traces = Bcex_nn_traces.copy();
                            prev_res.Bcex_fix_traces = Bcex_fix_traces.copy();
                            prev_res.Bcex_fix_samples =this.get_grid_data_from_traces(prev_res.Bcex_fix_traces); % for info only

                            prev_res.Rcex = Rcex.copy();
                            all_results{end} = prev_res;

                            % move on to res
                            res.Btrain_traces = Btrain_traces.copy();
                            res.Btrain= Btrain.copy();
                            res.Btrain_grid = Btrain_grid.copy();
                            res.Bval_traces = prev_res.Bval_traces.copy();
                            res.Bval = prev_res.Bval.copy();
                            
                        else
                            disp('STATUS: Data covers all counter examples, consider a finer grid.')
                            diary off;
                            return;
                        end                                          
                    else
                        % Store log of last falsif attempt 
                        prev_res.Rcex = Rcex.copy();
                         all_results{end} = prev_res;
                        disp('SUCCESS.')
                        diary off;
                        return;
                    end
                end
                %% Training
                disp('TRAINING...')
                [in_data_train, out_data_train] = prepare_training_data(this,Btrain);
                [in_data_val, out_data_val] = prepare_training_data(this,Bval);
                valid_cell_array = {in_data_val, out_data_val};
                train_options  = this.setup_training_options();
                train_options.ValidationData= valid_cell_array;
                layers =  this.setup_layers();
                nn = trainNetwork(in_data_train,out_data_train, layers , train_options);
                res.nn=nn;

                all_results{end+1}= res;
                %save(opts.save_file_name,'all_results');
                save([ opts.save_file_name 'iter' num2str(iter)],'res');
                save('last_results_algo2.mat','res');
       
                try
                    delete(findall(0)); % close training window
                end
            end
            prev_res.Rcex = Rcex.copy();
            all_results{end} = prev_res;
            disp('STATUS: Stopped after max iteration reached.')
            diary off;
        end

         %% Data generation       
        function [Bmpc_data, R] = get_mpc_traces(this, varargin)
        % Takes a structure for sampling the initial set and compute traces with mpc controller 
            
            opts.recompute = 0;
            opts.parallel = 0; % Changed by Fares
            opts.num_corners = 0;
            opts.num_quasi_random = 10;
            opts.seed = 1;
            opts.scaling = 1; 
            opts.local_max_obj_eval = 0;
            opts = varargin2struct_breach(opts,varargin{:});

            [success, msg] =  mkdir(this.data_folder);
            if success==0
                error('mkdir for dir %s failed with msg:\n %s', opts.data_folder, msg);
            end
            hash = DataHash(opts);          
            fname = [this.data_folder filesep 'B' this.name '_mpc_data_c' num2str(opts.num_corners) '_qr' num2str(opts.num_quasi_random) ...
                '_seed' num2str(opts.seed) '_' hash(1:6) '.mat'];

            if (exist(fname,'file')~=2)&&opts.recompute==0
                opts.recompute= (input('Data does not exist yet, compute it now (0 or 1)?'));
                if opts.recompute==0
                    Bmpc_data =[];
                    return;
                end
            end

            if opts.recompute~=0
                Bmpc_data = this.create_nominal(opts.scaling);

                if opts.num_corners==0         % only quasi random
                    Bmpc_data.QuasiRandomSample(opts.num_quasi_random, opts.seed);
                elseif opts.num_quasi_random==0 % only corners ?
                    Bmpc_data.CornerSample(opts.num_corners);
                else % I don't expect both to be 0
                    %% corner sampling
                    Bmpc_data.CornerSample(opts.num_corners);

                    %% Quasi random sampling
                    Bmpc_qr = this.create_nominal();
                    Bmpc_qr.QuasiRandomSample(opts.num_quasi_random, opts.seed);
                    Bmpc_data.Concat(Bmpc_qr,1);
                                   
                end
                %% Simulation
                if opts.parallel
                    Bmpc_data.parSim();
                else
                    Bmpc_data.Sim();
                end
                save(fname, 'Bmpc_data');
            else
                disp(['Loading mpc data from ' fname]);
                load(fname);
            end
            R =this.phi.copy();
            val = R.Eval(Bmpc_data);
            if val<0
                warning('Some trace(s) do not satisfy the requirement.')
            end
        end
        
        function [Bcex, Bnn, R] = get_nn_cex_traces(this, nn, varargin)
        % Takes a structure for sampling the initial set, simulate with a neural network and return cex
        % traces, all traces and requirement evaluation. Cex, if any, are sorted
        % from the worst to least bad
            
            opts.parallel = 0;
            opts.num_corners = 0;
            opts.num_quasi_random = 100;
            opts.scaling = 1;
            opts.seed = 1;
            opts.local_max_obj_eval = 0; 
            opts.recompute = 0; % not used, here for compatibility with get_mpc_data
            opts = varargin2struct_breach(opts,varargin{:});

            Bnn = this.create_nn_sys(nn, opts.scaling);

            if opts.num_corners==0         % only quasi random
                Bnn.QuasiRandomSample(opts.num_quasi_random, opts.seed);
            elseif opts.num_quasi_random==0 % only corners ?
                Bnn.CornerSample(opts.num_corners);
            else % I don't expect both to be 0
                %% corner sampling
                Bnn.CornerSample(opts.num_corners);

                %% Quasi random sampling
                Bnn_qr = this.create_nn_sys(nn, opts.scaling);
                Bnn_qr.QuasiRandomSample(opts.num_quasi_random, opts.seed);
                Bnn.Concat(Bnn_qr,1);
            end
            
            %% Simulation
            Bnn.Sim();

            R =this.phi.copy();
            val = R.Eval(Bnn);
            
            [~, traces_vals] = R.Eval(Bnn);
            avg_rob = mean(traces_vals);
            worst_rob = min(traces_vals);

            [sorted_traces_vals, idx_sorted_rob] = sort(traces_vals, 'ascend');            
            idx_cex = find(sorted_traces_vals<=0);

            fprintf('worst_rob:%g, avg_rob: %g, #cex: %g\n', worst_rob, avg_rob, sum(traces_vals<=0));
            
            if ~isempty(idx_cex)
                Bcex = Bnn.ExtractSubset(idx_sorted_rob(idx_cex));
            else
                Bcex= [];
            end

        end

        function [Bnn, R] = get_nn_rand_traces(this, nn, varargin)
        % Takes a structure for sampling the initial set, simulate with a neural network and return cex
        % traces, all traces and requirement evaluation. Cex, if any, are sorted
        % from the worst to least bad
            
            opts.num_random_traces = 100;
            opts.scaling= 1;
            opts = varargin2struct_breach(opts,varargin{:});

            Bnn = this.create_nn_sys(nn, opts.scaling);

            Bnn.SampleDomain(opts.num_random_traces);
   
            %% Simulation
            Bnn.Sim();

            R =this.phi.copy();
            val = R.Eval(Bnn);
            
            [~, traces_vals] = R.Eval(Bnn);
            avg_rob = mean(traces_vals);
            worst_rob = min(traces_vals);       
            fprintf('Random traces --- worst_rob:%g, avg_rob: %g, #cex: %g\n', worst_rob, avg_rob, sum(traces_vals<=0));
            

        end

        function [Bcex,R,pb] = falsify(this, nn, varargin)
        % Takes a structure for sampling the initial set, simulate with a neural network and return cex
        % traces, all traces and requirement evaluation. Cex, if any, are sorted
        % from the worst to least bad
                      
            opts.parallel = 0;
            opts.num_corners = 0;
            opts.num_quasi_random = 100;
            opts.scaling = .8;
            opts.seed = 1;
            opts.local_max_obj_eval = 150; 
            opts.max_obj_eval = 300;  
            opts.max_num_cex = 10;
            opts.freq_update =10;
            opts.display = 'on';
       
            opts.recompute = 0; % not used, here for compatibility with get_mpc_data
            opts = varargin2struct_breach(opts,varargin{:});

            Bnn = this.create_nn_sys(nn, opts.scaling);
            R = this.phi.copy();
            
            pb  = FalsificationProblem(Bnn, R);
            
            % pb configuration
            pb.setup_global_nelder_mead(...
                'num_corners', opts.num_corners...
                ,'num_quasi_rand_samples', opts.num_quasi_random...
                ,'quasi_rand_seed',1 ...                
                ,'local_max_obj_eval',opts.local_max_obj_eval...
                );
            pb.max_obj_eval = opts.max_obj_eval;
            pb.freq_update = opts.freq_update;
            pb.display = opts.display;
            
            
            pb.StopAtFalse=opts.max_num_cex; 
            pb.solve();
            R= pb.GetLog();
            
            traces_vals = R.traces_vals;
            avg_rob = mean(traces_vals);
            worst_rob = min(traces_vals);

            [sorted_traces_vals, idx_sorted_rob] = sort(traces_vals, 'ascend');            
            idx_cex = find(sorted_traces_vals<=0);

            fprintf('worst_rob:%g, avg_rob: %g, #cex: %g\n', worst_rob, avg_rob, sum(traces_vals<=0));
                      
            if ~isempty(idx_cex)
                idx_nn_traces = idx_sorted_rob(1:opts.max_num_cex); % ensure max_num_cex new traces, even if less cex have been found
                Bcex = R.BrSet.ExtractSubset(idx_nn_traces);
            else
                Bcex= [];
            end

        end

        function [Bcex_fix_traces] = fix_cex(this, Btrain, Bcex, max_num_cex_traces, max_num_cex_samples)
            % Cex are sorted with robustness. max_num_cex_traces 
            % pick subsect of counter example ? Sorted with robustness ?
                                    
            % Cut tail of traces from cex (we might think of better way to extract good times, like STL diagnosis) 
            % Not sure this is necessary, some saving on grid computation
            % maybe. Commenting for simplicity for now
            %Bcex_cut = Bcex.copy();
            %t_max = this.time(end)/2;
            %for itraj = 1:numel(Bcex_cut.P.traj)
            %    traj = Bcex_cut.P.traj{itraj};
            %    traj.time = traj.time(traj.time<= t_max);
            %  traj.X = traj.X(:,traj.time<=t_max);
            %    Bcex_cut.P.traj{itraj} = traj;
            % end
           
            %  Extract new X0 
            num_cex_traces= numel(Bcex.P.traj);
            if num_cex_traces>max_num_cex_traces
                Bcex_extract = Bcex.ExtractSubset(1:max_num_cex_traces);
            else
                Bcex_extract =Bcex;
            end

            Bf_cex  = this.get_grid_data_from_traces(Bcex_extract); 
            
            states_names =fieldnames(this.states)';
            X_cex = Bf_cex.GetParam(states_names);  % samples from cex traces            
           
            % Intersect with data already in Training set

            in_training_set = Btrain.check_covered_pts(states_names, X_cex);             
            Bf_cex_out_training = Bf_cex.ExtractSubset(find(~in_training_set));
            
            % sort by time
            time_cex_samples  = Bf_cex_out_training.GetParam('time');
            [ sorted_time, sorted_time_idx  ] =sort(time_cex_samples, 'ascend');

            % take max_num_cex_samples first out training ... 
            
            selected_idx = sorted_time_idx(1:min(numel(sorted_time_idx),max_num_cex_samples));            
            Pts_cex_out = Bf_cex_out_training.GetParam(states_names);  % samples from cex traces            
            new_X0_from_cex = Pts_cex_out(:,selected_idx);
            new_Pts_cex = this.get_params_from_X(new_X0_from_cex); % needed to recover values of parameters from state space
     
            
            Bcex_fix_traces = this.create_mpc_sys();
            params_params = fieldnames(this.params)';
            params_states = cellfun(@(c)([c '_0']), fieldnames(this.states)', 'UniformOutput', 0);
            params_X0 = [ params_states params_params ];                       
            Bcex_fix_traces.SetParam(params_X0, new_Pts_cex);
            
            this.parallel=0; % enleve parallel
            % simulation with mpc            
            if this.parallel
             
                Bcex_fix_traces.parSim();
            else
                Bcex_fix_traces.Sim();
            end
            
            R =this.phi.copy();
            val = R.Eval(Bcex_fix_traces);
            if val<0
                warning('Some trace(s) do not satisfy the requirement.')
            end     

        end
        
        function X = get_params_from_X(this, X)
            
           % needs something if this.params is not empty, i.e., X0 depends
           % on params. We need some form of inversion, see Drone example... 
    
           if ~ isequal(this.params, struct())
               nparams = numel(fieldnames(this.params));
               X = [X ; zeros(nparams, size(X,2))];
           end
                
        end

        function [Bfiltered, Bgrid] = get_grid_data_from_traces(this, Btraces, signals, grid_res)
            % Takes a Breach set with traces, returns grid data covered by it
            % By default takes all signal states

            if ~exist('signals', 'var')||isempty(signals)||isequal(signals,'all')
                signals = fieldnames(this.states)';
            end
            if nargin <4
                grid_res = this.get_grid_res(signals);
            end

            if isfield(this.states.(signals{1}), 'range_max')
                for isig = 1:numel(signals)
                    sig = signals{isig};
                    opts.signals.(sig).range = this.states.(sig).range_max;
                    opts.signals.(sig).grid = grid_res(isig);
                end
                [ Bgrid, Bf, Ba ] = Btraces.GridFilterSignals(signals,opts);
            else % backward compatible (range_max not defined...)
                [ Bgrid, Bf, Ba ] = Btraces.GridFilterSignals(signals, grid_res);
            end
            
            has_cost = ismember('cost',Ba.GetParamList());

            if has_cost
                % Extract samples in cell with best cost
                pts = Bgrid.DeltaGridMapObj.values;
                for i = 1:numel(pts)
                    pt = pts{i};
                    indices = pt.idx;
                    val_for_sorting = Ba.GetParam('cost', indices);
                    [~, this_indices_sorted] = sort(val_for_sorting);
                    indices_min(i) = indices(this_indices_sorted(1));
                end
                Bfiltered = Ba.ExtractSubset(indices_min);
            else
                Bfiltered = Bf;
            end

        end

 %% functions related to NN training 
        function layers = setup_layers(this)
            numObservations = numel(fieldnames(this.states));
            numActions = numel(fieldnames(this.controls));

            hiddenLayerSize = 256;
            umax = 3; % to be changed depending on the problem

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

        function options = setup_training_options(this)            
            options = trainingOptions('adam', ...
                'Verbose', false, ...
                'Plots','training-progress',...% 'none', ...
                'Shuffle', 'every-epoch', ...
                'MiniBatchSize', 512, ...
                'InitialLearnRate', 1e-3, ...
                'ExecutionEnvironment', 'cpu', ...
                'GradientThreshold', 10, ...
                'MaxEpochs', 40 ...
                );
 
        end

        function  [in_data, out_data] = prepare_training_data(this,B)
            % Gets inputs and outputs for neural network training from grid dat breach set

            % Data
            inputs_names = fieldnames(this.states);
            outputs_names =fieldnames(this.controls);

            in_data = B.GetParam(inputs_names)';
            out_data = B.GetParam(outputs_names)';

        end  

        function Bdata = import_data(this, data)
            inputs_names = fieldnames(this.states)';
            outputs_names =fieldnames(this.controls)';
            params = [inputs_names, outputs_names];
            Bdata= BreachSet(params);
            Bdata.SetParam(params,data');
        end

 %% Helper methods for managing results and other stuff
 function sampling =get_init_sampling(this, num_samples, num_corners, scaling, seed)
    if nargin<5
        seed=1;
    end
    if nargin <4
        scaling = 0.8;
    end

    if nargin <3
        switch num_samples
            case 100
                num_corners =16;
            case 1000
                num_corners =256;
            otherwise
                max_corners = 2^numel(fieldnames(this.states));
                num_corners = min(2^(round(log2(num_samples))-2), max_corners);    
        end
    end
    sampling.num_quasi_random = num_samples - num_corners;
        sampling.num_corners = num_corners;
        sampling.scaling=scaling;
        sampling.seed = seed;
     
 end

        function [avg_rob, worst_rob, num_cex, num_training]= disp_all_results(this, all_res)
            % display average rob, worst robustness, number of cex (?), number
            % of samples in training,

            avg_rob = nan(1,numel(all_res)-1);
            worst_rob = nan(1,numel(all_res)-1);
            num_cex = nan(1,numel(all_res)-1);
            num_training = nan(1,numel(all_res)-1);
            if ~isempty( all_res)
                for ires= 1:numel(all_res)-1
                    r = all_res{ires};
                    robs = r.Rcex.traces_vals;
                    avg_rob(ires)=mean(robs);
                    worst_rob(ires) = min(robs);
                    num_cex(ires) = sum(robs<=0);
                    num_training(ires) = r.Btrain.GetNbParamVectors();
                end

                fprintf('Average rob: %s\n', num2str(avg_rob,3));
                fprintf('Worst rob: %s\n', num2str(worst_rob,3));
                fprintf('Num cex: %s\n', num2str(num_cex,3));
                fprintf('Num samples: %s\n', num2str(num_training,5));
                fprintf('New training samples: %s\n', num2str(diff(num_training),4));
            end
        end

        function str = get_result_file_name(this, res)            
            str = ['Res_' datestr(now,'dd-mmm-yyyy-HH_MM_SS')];    
        end

        function params_names= get_names_params(this)
        % returns parameters of the problem, if any
            params_names = fieldnames(this.params)';
        end

        function params_state = get_names_init_state(this)
        % returns parameters for initial states
            params_state = cellfun(@(c)([c '_0']), fieldnames(this.states)', 'UniformOutput', 0);                        
        end

        function signals = get_names_signals(this)
        % returns signals names    
            signals = fieldnames(this.states)';
        end

        function in = get_names_nn_inputs(this)
            params_params = this.get_names_params();
            params_state = this.get_names_signals();
            in = [params_params params_state];
        end
        
        function out = get_names_nn_outputs(this)
            out = fieldnames(this.controls)';
        end

        function grid_res = get_grid_res(this, signals)
  
            if ~exist('signals', 'var')||isempty(signals)||isequal(signals,'all')
                signals = fieldnames(this.states)';
            end
            for is = 1:numel(signals)
                    sig=  signals{is};
                    try
                    if isfield(this.states,sig)  % is it a state
                        grid_res(is) = this.states.(sig).grid_res;
                    elseif isfield(this.controls,sig) % is it a control
                        grid_res(is) = this.controls.(sig).grid_res;
                    elseif isfield(this.outputs,sig) % is it an output
                        grid_res(is) = this.outputs.(sig).grid_res;
                    end
                    catch
                        %warning('no grid_res found for %s, default to %g',sig,0.1)
                        grid_res(is) = .1;
                    end
            end
            

        end

 %% Obsolete or not in use 
        function B = compute_mpc_controls(this, B)
        % Computes one step mpc control with no
        % initialization/history/initial guess. Might be sub optimal thus. 
            states_names = fieldnames(this.states)';
            control_names =fieldnames(this.controls)';
         
            states_val = B.GetParam(states_names);
            controls_val = nan(numel(control_names), size(states_val,2));

            for is =1:size(states_val,2)
                u_mpc=this.mpc_controlfn(states_val(:,is), []);
                controls_val(:, is) = u_mpc(:,1);
            end

            B.SetParam(control_names, controls_val, 1);
        end


        function [Bmerge, Bgrid] = merge_data(this, Bmerge, Bnew)
        
            %params_names = [  fieldnames(this.states)' fieldnames(this.controls)' ];            
            %states_val = Bnew.GetParam(params_names);
            %Bmerge.AddParam(params_names, states_val)
            %Bmerge.RemoveDuplicateParams();

            % recompute grid
            signals = fieldnames(this.states)';
            grid_res = this.get_grid_res(signals);
            Bgrid = Bmerge.GridFilter(signals, grid_res);

        end

    
    
    end
end