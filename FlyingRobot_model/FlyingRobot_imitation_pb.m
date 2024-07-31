classdef FlyingRobot_imitation_pb < imitation_pb

    methods
        function pb = FlyingRobot_imitation_pb(num_cell)
            
            if nargin==0
                num_cell=10;
            end

            pb = pb@imitation_pb('FlyingRobot_model');
            %% Generic problem parameters
            UMAX = 3;

            %% states and control names and initial values and ranges

            % nominal values
            pb.states.x1.nominal= -1.82;
            pb.states.x2.nominal= 0.53;
            pb.states.theta.nominal=-2.3;
            pb.states.v1.nominal=1.17;
            pb.states.v2.nominal=-1.04;
            pb.states.omega.nominal = 0.31;
            pb.states.u1_prev.nominal = -2.18;
            pb.states.u2_prev.nominal = -2.62;

            % ranges init
            pb.states.x1.range_init= [-4 4];
            pb.states.x2.range_init= [-4 4];
            pb.states.theta.range_init=[-pi,pi];
            pb.states.v1.range_init= [-2 2] ;
            pb.states.v2.range_init=[-2 2];
            pb.states.omega.range_init = [-1 1];
            pb.states.u1_prev.range_init = [-UMAX UMAX];
            pb.states.u2_prev.range_init = [-UMAX UMAX];

            % ranges max
            pb.states.x1.range_max= [-50 50];
            pb.states.x2.range_max= [-50 50];
            pb.states.theta.range_max=[-10*pi,10*pi];
            pb.states.v1.range_max= [-50 50] ;
            pb.states.v2.range_max=[-50 50];
            pb.states.omega.range_max = [-50 50];
            pb.states.u1_prev.range_max = [-UMAX UMAX];
            pb.states.u2_prev.range_max = [-UMAX UMAX];


            % grid resolution
            pb.states.x1.grid_res= diff(pb.states.x1.range_init)/num_cell;
            pb.states.x2.grid_res= diff(pb.states.x2.range_init)/num_cell;
            pb.states.theta.grid_res=diff(pb.states.theta.range_init)/num_cell;
            pb.states.v1.grid_res= diff(pb.states.v1.range_init)/num_cell;
            pb.states.v2.grid_res=diff(pb.states.v2.range_init)/num_cell;
            pb.states.omega.grid_res = diff(pb.states.omega.range_init)/num_cell;
            pb.states.u1_prev.grid_res = diff(pb.states.u1_prev.range_init)/num_cell;
            pb.states.u2_prev.grid_res = diff(pb.states.u2_prev.range_init)/num_cell;

            %% Control
            % inputs
            pb.controls.u1.range = [-UMAX UMAX];
            pb.controls.u2.range = [-UMAX UMAX];

            % MPC configuration and function
            mpcverbosity off;
            nlobj= createMPCobjImFlyingRobot(UMAX) ;
            
            pb.mpc_controlfn = @(x, options)(pb.mpc_control(nlobj,x, options));

            % Sim function
            Ts = nlobj.Ts;
            Tf = 15;
            time = 0:Ts:Tf;
            pb.time= [time Tf];
            pb.sim_fn = @(Sys,time,p,controlfn) sim_breach_flying(controlfn,time,p);

            %% Requirement
            STL_ReadFile('requirements.stl');
            R = BreachRequirement({'phi'});
            og = expr_output_gen('y', 'sqrt(x1[t].^2 + x2[t].^2 + theta[t].^2 + v1[t]^2 + v2[t].^2 + omega[t].^2)');
            R.AddPostProcess(og);

            pb.phi = R;
            

        end
    
        function [u, cost, opt] =  mpc_control(this,nlobj, x, opt)
            if isempty(opt)
                opt = nlmpcmoveopt;
            end
            [u,opt,simInfo] = nlmpcmove(nlobj,x(1:6),x(7:8),zeros(1,6),[],opt);
            cost = simInfo.Cost;
        end
                
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
    
        function str = get_result_file_name(this, res)
            str = ['Res_Flying_' datestr(now,'dd-mmm-yyyy-HH_MM_SS')];
        end
        
        function [tau_mean, p_mean, tau,p, Ry ] = get_nominal_pareto(this, sampling)
    
            if nargin <2
                sampling.num_corners= 16;
                sampling.num_quasi_random = 84;
                sampling.scaling = .8;
                sampling.seed=1;                              
            end
            [~, Ry] = get_mpc_traces(this, sampling);
            [tau_mean,p_mean, tau,p] = get_y_max(Ry);
        
        end

        function [tau_mean, p_mean,tau,p, Ry ] = get_nn_pareto(this, nn, sampling)
        % Computes Pareto front for a neural network
    
            if nargin <3
                sampling.num_corners= 16;
                sampling.num_quasi_random = 84;
                sampling.scaling = .8;
                sampling.seed=1;                              
            end
                   
            [Bcex, Bnn, Ry] = this.get_nn_cex_traces(nn, sampling);
            [tau_mean, p_mean, tau,p] = get_y_max(Ry);
            
        end
        
        function vol = get_pareto_volume(this, tau, p)
        % area below pareto, the smaller the better
            if tau(end)<15
                tau(end+1)=15;
                p(end+1) = p(end); 
            end
            dtau = diff(tau);
            vol = sum(dtau.*p(1:end-1));
        end

        function [vol_mean, vol_worst, Ry ] = plot_nns_paretos(this, nns, cfg, names)
                        
            if ~exist('names', 'var')                
                names = arrayfun(@(c)(['NN' num2str(c)]),1:numel(nns),'UniformOutput',false);
            end
            names = [{'Nominal'} names];

            if ~exist('cfg', 'var')||isempty(cfg) % default 100 traces
                cfg.num_corners= 16;
                cfg.num_quasi_random = 84;
                cfg.scaling = .8;
                cfg.seed=1;
            elseif isequal(cfg,1000)  % shortcut for 1000 traces
                cfg = struct();
                cfg.num_corners= 256;
                cfg.num_quasi_random = 744;
                cfg.scaling = .8;
                cfg.seed=1;            
            elseif isequal(cfg,2000)  % shortcut for 2000 traces
                cfg = struct();
                cfg.num_corners= 256;
                cfg.num_quasi_random = 1744;
                cfg.scaling = .8;
                cfg.seed=1;   
            end
                                   
            % Computes nominal values             
            [tau_mean{1}, p_mean{1},tau_worst{1}, p_worst{1}, Ry(1)] = this.get_nominal_pareto(cfg);
            for inn = 2:numel(nns)+1
                [tau_mean{inn}, p_mean{inn}, tau_worst{inn}, p_worst{inn} , Ry(inn)] = this.get_nn_pareto(nns{inn-1}, cfg);                
            end
            for inn = 1:numel(names)
                vol_mean(inn) = this.get_pareto_volume(tau_mean{inn}, p_mean{inn});
                vol_worst(inn) = this.get_pareto_volume(tau_worst{inn}, p_worst{inn});
            end
            
            % Plots
            figure;            
            subplot(2,2,1);    
            hold on; grid on;
            title('Average False Volume')
            bar(vol_mean)
            xticks(1:numel(names));
            xticklabels(names)
            
            % Plotting average
            subplot(2,2,2)
            title('Average Paretos');
            hold on
            for inn = 1:numel(names)
                l_mean(inn) = stairs(tau_mean{inn}, p_mean{inn}, 'DisplayName',names{inn}, 'Visible', 'on');
            end
            legend
            grid on            
            
            % plotting worst            
            subplot(2,2,3);
            hold on; grid on;
                        
            title('Worst False Volume')
            bar(vol_worst);
            xticks(1:numel(names));
            xticklabels(names)
            
            subplot(2,2,4);
            title('Worst performance');
            hold on
            for inn = 1:numel(names)                
                l_worst(inn) = stairs(tau_worst{inn}, p_worst{inn}, 'DisplayName',names{inn}, 'Visible', 'on');
            end
            legend
            grid on
                            
        end

        % Consistency 
        function nn = mathworks_train(this, mergedData)
            umax =3; 
            mpcverbosity off;
            nlobj = createMPCobjImFlyingRobot(umax);
            
            numCol = size(mergedData,2);
            numObs = numCol-2;
            numAct = 2;
            hiddenLayerSize = 256;
            
            imitateMPCNetwork = [
                featureInputLayer(numObs)
                fullyConnectedLayer(hiddenLayerSize)
                reluLayer
                fullyConnectedLayer(hiddenLayerSize)
                reluLayer
                fullyConnectedLayer(hiddenLayerSize)
                reluLayer
                fullyConnectedLayer(hiddenLayerSize)
                reluLayer
                fullyConnectedLayer(hiddenLayerSize)
                reluLayer
                fullyConnectedLayer(hiddenLayerSize)
                reluLayer
                fullyConnectedLayer(numAct)
                tanhLayer
                scalingLayer(Scale=umax)
                regressionLayer
                ];

            validationCellArray = {0,0};

            options = trainingOptions("adam", ...
                Verbose=false, ...
                Plots="training-progress", ...
                Shuffle="every-epoch", ...
                MiniBatchSize=512, ...
                ValidationData=validationCellArray, ...
                InitialLearnRate=1e-3, ...
                ExecutionEnvironment="cpu", ...
                GradientThreshold=10, ...
                MaxEpochs=40 ...
                );

            load("behaviorCloningMPCImDNNObject.mat");

            [dataStruct,nlmpcStruct,tuningParamsStruct,neuralNetStruct] = ...
                loadDAggerParameters(mergedData,numCol,nlobj,umax, ...
                options,  behaviorCloningNNObj.imitateMPCNetObj...
                );
            
            [newTrainInput, newTrainOutput, Options] = prepareDataImDAggerFlyingRobot(...
                                                                       mergedData, dataStruct, neuralNetStruct.options);

            % Train network
            
            nn = trainNetwork(newTrainInput, newTrainOutput, imitateMPCNetwork, Options);

       %??  inputObservationsL2Loss(i,:) = [sqrt(mean((xHistoryMPC - xHistoryDNN).^2)), sqrt(mean((uHistoryMPC - uHistoryDNN).^2))];
                 delete(findall(0)); % close training window

        
        end
    
        function [D, Bs, Bg] = filter_data(this, Data, scale)
        
            grid_res = this.get_grid_res()*scale+1e-10; % hackhackhaaack
            B1= this.import_data(Data);

            p_in = this.get_names_nn_inputs();           
            p = this.get_names_params();

            [Bg, Bs] = B1.GridFilter(p_in, grid_res);
            D = Bs.GetParam(p)';

        
        end
    
        function plot_trace(this,DNN, state0)

            x0 = state0(1:6);
            u0 = state0(7:8);


            UMAX = 3;
            mpcverbosity off;
            nlobj= createMPCobjImFlyingRobot(UMAX) ;
            
            % Duration
            Tf = 15;

            % Sample time
            Ts = nlobj.Ts;

            % Simulation steps
            Tsteps = Tf/Ts+1;

            [xHistoryMPC,uHistoryMPC] = ...
                simModelMPCImFlyingRobot(x0,u0,nlobj,Tf);

            [xHistoryDNN,uHistoryDNN] = ...
                simModelDAggerImFlyingRobot(x0,u0,DNN,Ts,Tf);

            plotSimResultsImFlyingRobot(nlobj, ...
                xHistoryMPC,uHistoryMPC,xHistoryDNN,uHistoryDNN,UMAX,Tf);

        end



        function [out ,l_worst,h_worst,l_mean,h_mean,Ry] = plot_pareto(this, all_results, Ry)
            
            t_pareto = 5:14;
            if nargin <3
                init_sampling1000.num_corners= 256;
                init_sampling1000.num_quasi_random = 744;
                init_sampling1000.scaling = .8;
                init_sampling1000.seed=10000;
                Bmpc = this.get_mpc_traces(init_sampling1000);
                Ry = BreachRequirement({'y[t]>0'});
                og = expr_output_gen('y', 'sqrt(x1[t].^2 + x2[t].^2 + theta[t].^2 + v1[t]^2 + v2[t].^2 + omega[t].^2)');
                Ry.AddPostProcess(og);
                Ry.Eval(Bmpc);
            end
                                   
            h_mean=  figure;
            [out{1}.t1, out{1}.ymean,out{1}.t2,out{1}.yworst] = get_y_max(Ry);
            
            y = interp1(out{1}.t1, out{1}.ymean, t_pareto);
            out{1}.lmean= plot(t_pareto, y,'g', 'DisplayName','Target mean');
            hold on
            legend
            grid on
            set(gca, 'XLim',[5 14]);

            for inn = 2:numel(all_results)+1
                [out{inn}.t1, out{inn}.ymean, out{inn}.t2, out{inn}.yworst ] = get_y_max(all_results{inn-1}.Rcex);                
                y = interp1(out{inn}.t1,out{inn}.ymean, t_pareto);
                l_mean(inn) = plot(t_pareto, y, 'DisplayName',['Average performance NN ' num2str(inn-1)], 'Visible', 'on');
            end
            

            h_worst= figure;
            subplot(3,1, [2 3]);
            y = interp1(out{1}.t2, out{1}.yworst, t_pareto);
            l_worst = plot(t_pareto, y, 'g', 'DisplayName','Target worst');
            hold on
            legend
            grid on
            set(gca, 'XLim',[5 14]);
                
            for inn = 2:numel(all_results)+1
                y = interp1(out{inn}.t2,out{inn}.yworst, t_pareto);
                l_worst(inn) = plot(t_pareto, y, 'DisplayName',['Worst Performance NN ' num2str(inn-1)], 'Visible', 'on');
            end
            legend()      
        
        end

        function [Ry] = getRy(this, res)    

            init_sampling100.num_corners= 16;
            init_sampling100.num_quasi_random = 84;
            init_sampling100.scaling = .8;
            init_sampling100.seed=1;

            init_sampling1000.num_corners= 256;
            init_sampling1000.num_quasi_random = 744;
            init_sampling1000.scaling = .8;
            init_sampling1000.seed=10000;

            init_sampling5000.num_corners= 256;
            init_sampling5000.num_quasi_random = 5000;
            init_sampling5000.scaling = .8;
            init_sampling5000.seed=1;


            for inn = 1:numel(res)
                nn = res{inn}.nn;
                [~, Bnn(inn)] = this.get_nn_cex_traces(nn, init_sampling1000);
                Ry(inn) = BreachRequirement({'alw (y[t]>0)'});
                og = expr_output_gen('y', 'sqrt(x1[t].^2 + x2[t].^2 + theta[t].^2 + v1[t]^2 + v2[t].^2 + omega[t].^2)');
                Ry(inn).AddPostProcess(og);
                Ry(inn).Eval(Bnn(inn));
            end

        end

    end
end