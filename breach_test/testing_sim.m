function [t,X, p, status] = sim_breach_watertank(control_fn, t, p)
    %% collect parameters
    DimX = 9;                 % last one is cost, which we use with MPC, will see for PID...
    params = p(DimX+1:end); % p(1:DimX) is always 0 for legacy reasons 
    Xinit = params(1:7);     % Initial state 
    ref1  = params(8);    % Additional parameter(s)
    Oshoot=0.2; % to be defined
    epsilon=0.02; % to be defined

    %%
    %choose the threshold
    if ref1 > params(1)
        threshold = ref1+ Oshoot;
    else
        threshold = ref1- Oshoot;
    end

%% algo 1    
 
    %% Init signal output
    X = zeros(DimX, numel(t)); % preparing the signals output array
    Xs = zeros(DimX);
    Xf= zeros(DimX);

    X(1:7, 1) = Xinit; % Initial state

    Ts = t(2)-t(1);
    
    %% Main simulation loop
    for k = 1:numel(t)-1
        Xk = X(:,k); % current state, we want to compute Xk+1
        
      
        %% computes next control input, either with pid or NN
        [Vs,Vf] = control_fn(Xk(1:7));
        %X(8,k) = V; % new V
                          % here we could assign a cost for new V, X(9,k)=?
   
        %% update plant
        H = Xk(1);      % current height
        ODEFUN = @(t,H) PlantDynamicsFn(H,Vs);
        [~,H_OUT_S] = ode45(ODEFUN,[0 Ts], H);
        
        ODEFUN = @(t,H) PlantDynamicsFn(H,Vf);
        [~,H_OUT_F] = ode45(ODEFUN,[0 Ts], H);
        %%
        Hs=H_OUT_S(end,:);
        Hf=H_OUT_F(end,:);
	
	V=VSf
	H_OUT=Hf

        X(8,k) = V;

        %% Store the new state values.
        X(2:3,k+1) = Xk(1:2);     % prev H 
        X(1,k+1) = H_OUT(end,:);     % new H
        X(5:6,k+1) = Xk(4:5);      % prev ref        
        if k >= numel(t)/2
            X(4,k+1) = ref1; % new ref 
        else 
            X(4,k+1) = Xk(4);
        end
        X(7,k+1) = V; % prev V 
    
    end
    % last V kept constant
    X(8,end) = V;
    
    status = 0; % required by latest Breach
end




function [t,X, p, status] = sim_breach_watertank(control_fn, t, p)
    %% collect parameters
    DimX = 9;                 % last one is cost, which we use with MPC, will see for PID...
    params = p(DimX+1:end); % p(1:DimX) is always 0 for legacy reasons 
    Xinit = params(1:7);     % Initial state 
    ref1  = params(8);    % Additional parameter(s)
    Oshoot=0.2; % to be defined
    epsilon=0.02; % to be defined

    %%
    %choose the threshold
    if ref1 > params(1)
        threshold = ref1+ Oshoot;
    else
        threshold = ref1- Oshoot;
    end

%% algo 1    
 
    %% Init signal output
    X = zeros(DimX, numel(t)); % preparing the signals output array
    Xs = zeros(DimX);
    Xf= zeros(DimX);

    X(1:7, 1) = Xinit; % Initial state

    Ts = t(2)-t(1);
    
    %% Main simulation loop
    for k = 1:numel(t)-1
        Xk = X(:,k); % current state, we want to compute Xk+1
        
      
        %% computes next control input, either with pid or NN
        [Vs,Vf] = control_fn(Xk(1:7));
        %X(8,k) = V; % new V
                          % here we could assign a cost for new V, X(9,k)=?
   
        %% update plant
        H = Xk(1);      % current height
        ODEFUN = @(t,H) PlantDynamicsFn(H,Vs);
        [~,H_OUT_S] = ode45(ODEFUN,[0 Ts], H);
        
        ODEFUN = @(t,H) PlantDynamicsFn(H,Vf);
        [~,H_OUT_F] = ode45(ODEFUN,[0 Ts], H);
        %%
        Hs=H_OUT_S(end,:);
        Hf=H_OUT_F(end,:);

        if H_OUT_F>threshold
            V=Vs;
            H_OUT=H_OUT_S;
        else
            if threshold- H_OUT_F < epsilon
                if (abs(Hf-X(4))<abs(Hs-X(4))) && abs(abs(Hf-X(4))-abs(Hs-X(4)))>epsilon
                    V=Vf;
                    H_OUT=Hf;
                else
                    V=Vs;
                    H_OUT=Hs;
                end
            else
                if abs(Hf-X(4))<abs(Hs-X(4))
                    V=Vf;
                    H_OUT=Hf;
                else
                    V=Vs;
                    H_OUT=Hs;
                end
            end
        end

        X(8,k) = V;

        %% Store the new state values.
        X(2:3,k+1) = Xk(1:2);     % prev H 
        X(1,k+1) = H_OUT;     % new H
        X(5:6,k+1) = Xk(4:5);      % prev ref        
        if k >= numel(t)/2
            X(4,k+1) = ref1; % new ref 
        else 
            X(4,k+1) = Xk(4);
        end
        X(7,k+1) = V; % prev V 
    
    end
    % last V kept constant
    X(8,end) = V;
    
    status = 0; % required by latest Breach
end


function [t,X, p, status] = sim_breach_watertank(control_fn, t, p)
    %% collect parameters
    DimX = 9;                 % last one is cost, which we use with MPC, will see for PID...
    params = p(DimX+1:end); % p(1:DimX) is always 0 for legacy reasons 
    Xinit = params(1:7);     % Initial state 
    ref1  = params(8);    % Additional parameter(s)
    Oshoot=0.2; % to be defined
    epsilon=0.02; % to be defined

    %%
    %choose the threshold
    if ref1 > params(1)
        threshold = ref1+ Oshoot;
    else
        threshold = ref1- Oshoot;
    end

%% algo 1    
 
    %% Init signal output
    X = zeros(DimX, numel(t)); % preparing the signals output array


    X(1:7, 1) = Xinit; % Initial state

    Ts = t(2)-t(1);
    
    %% Main simulation loop
    for k = 1:numel(t)-1
        Xk = X(:,k); % current state, we want to compute Xk+1
        
      
        %% computes next control input, either with pid or NN
        [Vs,Vf] = control_fn(Xk(1:7));
        %X(8,k) = V; % new V
                          % here we could assign a cost for new V, X(9,k)=?
   
        %% update plant
        H = Xk(1);      % current height
        ODEFUN = @(t,H) PlantDynamicsFn(H,Vs);
        [~,H_OUT_S] = ode45(ODEFUN,[0 Ts], H);
        
        ODEFUN = @(t,H) PlantDynamicsFn(H,Vf);
        [~,H_OUT_F] = ode45(ODEFUN,[0 Ts], H);
        %%
        Hs=H_OUT_S(end,:);
        Hf=H_OUT_F(end,:);

      
                if abs(Hf-X(4))<abs(Hs-X(4))
                    V=Vf;
                    H_OUT=Hf;
                else
                    V=Vs;
                    H_OUT=Hs;
                end
          

        X(8,k) = V;

        %% Store the new state values.
        X(2:3,k+1) = Xk(1:2);     % prev H 
        X(1,k+1) = H_OUT;     % new H
        X(5:6,k+1) = Xk(4:5);      % prev ref        
        if k >= numel(t)/2
            X(4,k+1) = ref1; % new ref 
        else 
            X(4,k+1) = Xk(4);
        end
        X(7,k+1) = V; % prev V 
    
    end
    % last V kept constant
    X(8,end) = V;
    
    status = 0; % required by latest Breach
end

#################################################### BONNE VERSION ####################

function [t,X, p, status] = sim_breach_watertank(control_fn, t, p)
    %% collect parameters
    DimX = 9;                 % last one is cost, which we use with MPC, will see for PID...
    params = p(DimX+1:end); % p(1:DimX) is always 0 for legacy reasons 
    Xinit = params(1:7);     % Initial state 
    ref1  = params(8);    % Additional parameter(s)
    Oshoot=0.2; % to be defined
    epsilon=0.02; % to be defined
    ref= params(4);

    %%
    %choose the threshold
    if ref1 > params(1)
        threshold = ref1+ Oshoot;
    else
        threshold = ref1- Oshoot;
    end

%% algo 1    
 
    %% Init signal output
    X = zeros(DimX, numel(t)); % preparing the signals output array


    X(1:7, 1) = Xinit; % Initial state

    Ts = t(2)-t(1);
    
    %% Main simulation loop
    for k = 1:numel(t)-1
        Xk = X(:,k); % current state, we want to compute Xk+1
        
      
        %% computes next control input, either with pid or NN
        [Vs,Vf] = control_fn(Xk(1:7));
        %X(8,k) = V; % new V
                          % here we could assign a cost for new V, X(9,k)=?
   
        %% update plant
        H = Xk(1);      % current height
        ODEFUN = @(t,H) PlantDynamicsFn(H,Vs);
        [~,H_OUT_S] = ode45(ODEFUN,[0 Ts], H);
        
        ODEFUN = @(t,H) PlantDynamicsFn(H,Vf);
        [~,H_OUT_F] = ode45(ODEFUN,[0 Ts], H);
        %%
        Hs=H_OUT_S(end,:)
        Hf=H_OUT_F(end,:)

      
                if abs(Hf-ref)<abs(Hs-ref)
                    V=Vf;
                    H_OUT=Hf;
                     fprintf('  Choosing fast controller (Vf): H_OUT = %f\n', H_OUT);
                else
                    V=Vs;
                    H_OUT=Hs;
                    fprintf('  Choosing slow controller (Vs): H_OUT = %f\n', H_OUT);
                end
          

        X(8,k) = V;

        %% Store the new state values.
        X(2:3,k+1) = Xk(1:2);     % prev H 
        X(1,k+1) = H_OUT;     % new H
        X(5:6,k+1) = Xk(4:5);      % prev ref        
        if k >= numel(t)/2
            X(4,k+1) = ref1; % new ref
            ref=ref1;
        else 
            X(4,k+1) = Xk(4);
        end
        X(7,k+1) = V; % prev V 
    
    end
    % last V kept constant
    X(8,end) = V;
    
    status = 0; % required by latest Breach
end


#################################### NOUVELLE VERSION #############################



function [t,X, p, status] = sim_breach_watertank(control_fn, t, p)
    %% collect parameters
    DimX = 9;                 % last one is cost, which we use with MPC, will see for PID...
    params = p(DimX+1:end); % p(1:DimX) is always 0 for legacy reasons 
    Xinit = params(1:7);     % Initial state 
    ref1  = params(8);    % Additional parameter(s)
    Oshoot=0.2; % to be defined
    epsilon=0.02; % to be defined
    ref= params(4);
	
    %%
    %choose the threshold
    if ref1 > params(1)
        threshold = ref1+ Oshoot;
    else
        threshold = ref1- Oshoot;
    end

%% algo 1    
 
    %% Init signal output
    X = zeros(DimX, numel(t)); % preparing the signals output array
    Xs = zeros(DimX);
    Xf= zeros(DimX);

    X(1:7, 1) = Xinit; % Initial state

    Ts = t(2)-t(1);
    
    %% Main simulation loop
    for k = 1:numel(t)-1
        Xk = X(:,k); % current state, we want to compute Xk+1
        
      
        %% computes next control input, either with pid or NN
        [Vs,Vf] = control_fn(Xk(1:7));
        %X(8,k) = V; % new V
                          % here we could assign a cost for new V, X(9,k)=?
   
        %% update plant
        H = Xk(1);      % current height
        ODEFUN = @(t,H) PlantDynamicsFn(H,Vs);
        [~,H_OUT_S] = ode45(ODEFUN,[0 Ts], H);
        
        ODEFUN = @(t,H) PlantDynamicsFn(H,Vf);
        [~,H_OUT_F] = ode45(ODEFUN,[0 Ts], H);
        %%
        Hs=H_OUT_S(end,:);
        Hf=H_OUT_F(end,:);

        if H_OUT_F>threshold
            V=Vs;
            H_OUT=H_OUT_S;
        else
            if threshold- H_OUT_F < epsilon
                if (abs(Hf-ref)<abs(Hs-ref)) && abs(abs(Hf-ref)-abs(Hs-ref))>epsilon
                    V=Vf;
                    H_OUT=Hf;
                else
                    V=Vs;
                    H_OUT=Hs;
                end
            else
                if abs(Hf-ref)<abs(Hs-ref)
                    V=Vf;
                    H_OUT=Hf;
                else
                    V=Vs;
                    H_OUT=Hs;
                end
            end
        end

        X(8,k) = V;

        %% Store the new state values.
        X(2:3,k+1) = Xk(1:2);     % prev H 
        X(1,k+1) = H_OUT;     % new H
        X(5:6,k+1) = Xk(4:5);      % prev ref        
        if k >= numel(t)/2
            X(4,k+1) = ref1; % new ref 
            ref=ref1;
        else 
            X(4,k+1) = Xk(4);
        end
        X(7,k+1) = V; % prev V 
    
    end
    % last V kept constant
    X(8,end) = V;
    
    status = 0; % required by latest Breach
end

###############sim latest version ##################

function [t,X, p, status] = sim_breach_watertank(control_fn, t, p)
    %% collect parameters
    DimX = 9;                 % last one is cost, which we use with MPC, will see for PID...
    params = p(DimX+1:end); % p(1:DimX) is always 0 for legacy reasons 
    Xinit = params(1:7);     % Initial state 
    ref1  = params(8);    % Additional parameter(s)
    Oshoot=0.05; % to be defined
    epsilon=0.02; % to be defined
    ref= params(4);
	
    %%
    %choose the threshold
    if ref1 > params(1)
        threshold = ref1+ Oshoot;
    else
        threshold = ref1- Oshoot;
    end

%% algo 1    
 
    %% Init signal output
    X = zeros(DimX, numel(t)); % preparing the signals output array
    Xs = zeros(DimX);
    Xf= zeros(DimX);

    X(1:7, 1) = Xinit; % Initial state

    Ts = t(2)-t(1);
    
    %% Main simulation loop
    for k = 1:numel(t)-1
        Xk = X(:,k); % current state, we want to compute Xk+1
        
      
        %% computes next control input, either with pid or NN
        [Vs,Vf] = control_fn(Xk(1:7));
        %X(8,k) = V; % new V
                          % here we could assign a cost for new V, X(9,k)=?
   
        %% update plant
        H = Xk(1);      % current height
        ODEFUN = @(t,H) PlantDynamicsFn(H,Vs);
        [~,H_OUT_S] = ode45(ODEFUN,[0 Ts], H);
        
        ODEFUN = @(t,H) PlantDynamicsFn(H,Vf);
        [~,H_OUT_F] = ode45(ODEFUN,[0 Ts], H);
        %%
        Hs=H_OUT_S(end,:);
        Hf=H_OUT_F(end,:);

        if H_OUT_F>threshold
            V=Vs;
            H_OUT=Hs;
        else
            if threshold- H_OUT_F < epsilon
                if (abs(Hf-ref)<abs(Hs-ref)) && abs(abs(Hf-ref)-abs(Hs-ref))>epsilon
                    V=Vf;
                    H_OUT=Hf;
                else
                    V=Vs;
                    H_OUT=Hs;
                end
            else
                if abs(Hf-ref)<abs(Hs-ref)
                    V=Vf;
                    H_OUT=Hf;
                else
                    V=Vs;
                    H_OUT=Hs;
                end
            end
        end

        X(8,k) = V;

        %% Store the new state values.
        X(2:3,k+1) = Xk(1:2);     % prev H 
        X(1,k+1) = H_OUT;     % new H
        X(5:6,k+1) = Xk(4:5);      % prev ref        
        if k >= numel(t)/2
            X(4,k+1) = ref1; % new ref 
            ref=ref1;
        else 
            X(4,k+1) = Xk(4);
        end
        X(7,k+1) = V; % prev V 
    
    end
    % last V kept constant
    X(8,end) = V;
    
    status = 0; % required by latest Breach
end

##################################################################################### latest ###############
function [t,X, p, status] = sim_breach_watertank(control_fn, t, p)
    %% collect parameters
    DimX = 9;                 % last one is cost, which we use with MPC, will see for PID...
    params = p(DimX+1:end);  % p(1:DimX) is always 0 for legacy reasons 
    Xinit = params(1:7);     % Initial state 
    ref1  = params(8);       % Additional parameter(s)
    Oshoot=0.07;             % to be defined
    epsilon=0.7;           % to be defined
    ref= params(4);
	
    %%
    %choose the threshold
    if ref1 > params(1)
        threshold = ref1+ Oshoot;
    else
        threshold = ref1- Oshoot;
    end

%% algo 1    
 
    %% Init signal output
    X = zeros(DimX, numel(t)); % preparing the signals output array
    Xs = zeros(DimX);
    Xf= zeros(DimX);

    X(1:7, 1) = Xinit; % Initial state

    Ts = t(2)-t(1);
    
    %% Main simulation loop
    for k = 1:numel(t)-1
        Xk = X(:,k); % current state, we want to compute Xk+1
        
      
        %% computes next control input, either with pid or NN
        [Vs,Vf] = control_fn(Xk(1:7));
        %X(8,k) = V; % new V
                          % here we could assign a cost for new V, X(9,k)=?
   
        %% update plant
        H = Xk(1);      % current height
        ODEFUN = @(t,H) PlantDynamicsFn(H,Vs);
        [~,H_OUT_S] = ode45(ODEFUN,[0 Ts], H);
        
        ODEFUN = @(t,H) PlantDynamicsFn(H,Vf);
        [~,H_OUT_F] = ode45(ODEFUN,[0 Ts], H);
        %%
        Hs=H_OUT_S(end,:);
        Hf=H_OUT_F(end,:);

        if H_OUT_F>threshold
            V=Vs;
            H_OUT=Hs;
         %   fprintf('Out of overshoot\n');
        else
            if threshold- H_OUT_F < epsilon
                if (abs(Hf-ref)<abs(Hs-ref)) && abs(abs(Hf-ref)-abs(Hs-ref))>epsilon
                    V=Vf;
                    H_OUT=Hf;
                   % fprintf('close to overshoot but better solution\n');
                else
                    V=Vs;
                    H_OUT=Hs;
                   % fprintf('close to overshoot but worse solution\n');
                end
            else
                if abs(Hf-ref)<abs(Hs-ref)
                    V=Vf;
                    H_OUT=Hf;
                   % fprintf('fast close \n');
                else
                    V=Vs;
                    H_OUT=Hs;
                   % fprintf('slow close \n');
                end
            end
        end

        X(8,k) = V;

        %% Store the new state values.
        X(2:3,k+1) = Xk(1:2);     % prev H 
        X(1,k+1) = H_OUT;     % new H
        X(5:6,k+1) = Xk(4:5);      % prev ref        
        if k >= numel(t)/2
            X(4,k+1) = ref1; % new ref 
            ref=ref1;
        else 
            X(4,k+1) = Xk(4);
        end
        X(7,k+1) = V; % prev V 
    
    end
    % last V kept constant
    X(8,end) = V;
    
    status = 0; % required by latest Breach
end

############################## wuth 2 thresh ###########################



function [t,X, p, status] = sim_breach_watertank(control_fn, t, p)
    %% collect parameters
    DimX = 9;                 % last one is cost, which we use with MPC, will see for PID...
    params = p(DimX+1:end);  % p(1:DimX) is always 0 for legacy reasons 
    Xinit = params(1:7);  % Initial state 
    ref0 = params(4);
    ref1  = params(8);       % Additional parameter(s)
    Oshoot=0.07;             % to be defined
    epsilon=0.7;           % to be defined
    ref= params(4);
	
    %%
    %choose the threshold
    if ref0 > params(1)
        threshold_H= ref0+ Oshoot;
        threshold_L= -Inf;
    else
        threshold_L = ref0- Oshoot;
        threshold_H = Inf;
    end


%% algo 1    
 
    %% Init signal output
    X = zeros(DimX, numel(t)); % preparing the signals output array
    Xs = zeros(DimX);
    Xf= zeros(DimX);

    X(1:7, 1) = Xinit; % Initial state

    Ts = t(2)-t(1);
    
    %% Main simulation loop
    for k = 1:numel(t)-1
        Xk = X(:,k); % current state, we want to compute Xk+1
        
      
        %% computes next control input, either with pid or NN
        [Vs,Vf] = control_fn(Xk(1:7));
        %X(8,k) = V; % new V
                          % here we could assign a cost for new V, X(9,k)=?
   
        %% update plant
        H = Xk(1);      % current height
        ODEFUN = @(t,H) PlantDynamicsFn(H,Vs);
        [~,H_OUT_S] = ode45(ODEFUN,[0 Ts], H);
        
        ODEFUN = @(t,H) PlantDynamicsFn(H,Vf);
        [~,H_OUT_F] = ode45(ODEFUN,[0 Ts], H);
        %%
        Hs=H_OUT_S(end,:);
        Hf=H_OUT_F(end,:);

        if any(H_OUT_F>threshold_H) || any(H_OUT_F<threshold_L) 
            V=Vs;
            H_OUT=Hs;
            fprintf('Out of overshoot\n');
        else
            if any(threshold_H- H_OUT_F < epsilon) ||  any(H_OUT_F-threshold_L < epsilon)
                if (abs(Hf-ref)<abs(Hs-ref)) && abs(abs(Hf-ref)-abs(Hs-ref))>epsilon
                    V=Vf;
                    H_OUT=Hf;
                   fprintf('close to overshoot but better solution\n');
                else
                    V=Vs;
                    H_OUT=Hs;
                    fprintf('close to overshoot but worse solution\n');
                end
            else
                if abs(Hf-ref)<abs(Hs-ref)
                    V=Vf;
                    H_OUT=Hf;
                    fprintf('fast close \n');
                else
                    V=Vs;
                    H_OUT=Hs;
                    fprintf('slow close \n');
                end
            end
        end

        X(8,k) = V;

        %% Store the new state values.
        X(2:3,k+1) = Xk(1:2);     % prev H 
        X(1,k+1) = H_OUT;     % new H
        X(5:6,k+1) = Xk(4:5);      % prev ref        
        if k >= numel(t)/2
            X(4,k+1) = ref1; % new ref 
            ref=ref1;
            if ref1 > ref0
                threshold_H = ref1+ Oshoot;
                threshold_L = -Inf;
            else
                threshold_L = ref1- Oshoot;
                threshold_H = Inf;
            end
        else 
            X(4,k+1) = Xk(4);
        end
        X(7,k+1) = V; % prev V 
    
    end
    % last V kept constant
    X(8,end) = V;
    
    status = 0; % required by latest Breach
end



####################### 2 PIDS #############


function [t,X, p, status] = sim_breach_watertank(control_fn, t, p)
    %% collect parameters
    DimX = 9;                % last one is cost, which we use with MPC, will see for PID...
    params = p(DimX+1:end);  % p(1:DimX) is always 0 for legacy reasons 
    Xinit = params(1:7);     % Initial state 
    ref0 = params(4);
    ref1  = params(8);       % Additional parameter(s)
    Oshoot=0.07;             % to be defined
    epsilon=0.6;             % to be defined
    epsilon_2= 0.3;
    ref= params(4);
	
    %%
    %choose the threshold
    if ref0 > params(1)
        threshold_H= ref0+ Oshoot;
        threshold_L= -Inf;
    else
        threshold_L = ref0- Oshoot;
        threshold_H = Inf;
    end


%% algo 1    
 
    %% Init signal output
    X = zeros(DimX, numel(t)); % preparing the signals output array
    Xs = zeros(DimX);
    Xf= zeros(DimX);

    X(1:7, 1) = Xinit; % Initial state

    Ts = t(2)-t(1);
    
    %% Main simulation loop
    for k = 1:numel(t)-1
        Xk = X(:,k); % cu4rrent state, we want to compute Xk+1
        
      
        %% computes next control input, either with pid or NN
        [Vs,Vf] = control_fn(Xk(1:7));
        %X(8,k) = V; % new V
                          % here we could assign a cost for new V, X(9,k)=?
   
        %% update plant
        H = Xk(1);      % current height
        ODEFUN = @(t,H) PlantDynamicsFn(H,Vs);
        [~,H_OUT_S] = ode45(ODEFUN,[0 Ts], H);
        
        ODEFUN = @(t,H) PlantDynamicsFn(H,Vf);
        [~,H_OUT_F] = ode45(ODEFUN,[0 Ts], H);
        %%
        Hs=H_OUT_S(end,:);
        Hf=H_OUT_F(end,:);

        if any(H_OUT_F>threshold_H) || any(H_OUT_F<threshold_L) 
            V=Vs;
            H_OUT=Hs;
          %  fprintf('Out of overshoot\n');
        else
            if any(threshold_H- H_OUT_F < epsilon) ||  any(H_OUT_F-threshold_L < epsilon)
                if (abs(Hf-ref)<abs(Hs-ref)) && abs(abs(Hf-ref)-abs(Hs-ref))>epsilon_2
                    V=Vf;
                    H_OUT=Hf;
                  % fprintf('close to overshoot but better solution\n');
                else
                    V=Vs;
                    H_OUT=Hs;
                 %   fprintf('close to overshoot but worse solution\n');
                end
            else
                if abs(Hf-ref)<abs(Hs-ref)
                    V=Vf;
                    H_OUT=Hf;
                  %  fprintf('fast close \n');
                else
                    V=Vs;
                    H_OUT=Hs;
                  %  fprintf('slow close \n');
                end
            end
        end

        X(8,k) = V;

        %% Store the new state values.
        X(2:3,k+1) = Xk(1:2);     % prev H 
        X(1,k+1) = H_OUT;     % new H
        X(5:6,k+1) = Xk(4:5);      % prev ref        
        if k >= numel(t)/2
            X(4,k+1) = ref1; % new ref 
            ref=ref1;
            if ref1 > ref0
                threshold_H = ref1+ Oshoot;
                threshold_L = -Inf;
            else
                threshold_L = ref1- Oshoot;
                threshold_H = Inf;
            end
        else 
            X(4,k+1) = Xk(4);
        end
        X(7,k+1) = V; % prev V 
    
    end
    % last V kept constant
    X(8,end) = V;
    
    status = 0; % required by latest Breach
end


########## 22 august #######

function [t,X, p, status] = sim_breach_watertank(control_fn, t, p)
    %% collect parameters
    DimX = 9;                % last one is cost, which we use with MPC, will see for PID...
    params = p(DimX+1:end);  % p(1:DimX) is always 0 for legacy reasons 
    Xinit = params(1:7);     % Initial state 
    ref0 = params(4);
    ref1  = params(8);       % Additional parameter(s)
    Oshoot=0.07;             % to be defined
    epsilon=0.6;             % to be defined
    epsilon_2= 0.3;
    ref= params(4);
	
    %%
    %choose the threshold
    if ref0 > params(1)
        threshold_H= ref0+ Oshoot;
        threshold_L= -Inf;
    else
        threshold_L = ref0- Oshoot;
        threshold_H = Inf;
    end


%% algo 1    
 
    %% Init signal output
    X = zeros(DimX, numel(t)); % preparing the signals output array
    Xs = zeros(DimX);
    Xf= zeros(DimX);

    X(1:7, 1) = Xinit; % Initial state

    Ts = t(2)-t(1);
    
    %% Main simulation loop
    for k = 1:numel(t)-1
        Xk = X(:,k); % cu4rrent state, we want to compute Xk+1
        
      
        %% computes next control input, either with pid or NN
        [Vs,Vf] = control_fn(Xk(1:7));
        %X(8,k) = V; % new V
                          % here we could assign a cost for new V, X(9,k)=?
   
        %% update plant
        H = Xk(1);      % current height
        ODEFUN = @(t,H) PlantDynamicsFn(H,Vs);
        [~,H_OUT_S] = ode45(ODEFUN,[0 Ts], H);
        
        ODEFUN = @(t,H) PlantDynamicsFn(H,Vf);
        [~,H_OUT_F] = ode45(ODEFUN,[0 Ts], H);
        %%
        Hs=H_OUT_S(end,:);
        Hf=H_OUT_F(end,:);

        if any(H_OUT_F>threshold_H) || any(H_OUT_F<threshold_L) 
            V=Vs;
            H_OUT=Hs;
          %  fprintf('Out of overshoot\n');
        else
            if any(threshold_H- H_OUT_F < epsilon) ||  any(H_OUT_F-threshold_L < epsilon)
                if (abs(Hf-ref)<abs(Hs-ref)) && abs(abs(Hf-ref)-abs(Hs-ref))>epsilon_2
                    V=Vf;
                    H_OUT=Hf;
                  % fprintf('close to overshoot but better solution\n');
                else
                    V=Vs;
                    H_OUT=Hs;
                 %   fprintf('close to overshoot but worse solution\n');
                end
            else
                if abs(Hf-ref)<abs(Hs-ref)
                    V=Vf;
                    H_OUT=Hf;
                  %  fprintf('fast close \n');
                else
                    V=Vs;
                    H_OUT=Hs;
                  %  fprintf('slow close \n');
                end
            end
        end

        X(8,k) = V;

        %% Store the new state values.
        X(2:3,k+1) = Xk(1:2);     % prev H 
        X(1,k+1) = H_OUT;     % new H
        X(5:6,k+1) = Xk(4:5);      % prev ref        
        if k >= numel(t)/2
            X(4,k+1) = ref1; % new ref 
            ref=ref1;
            if ref1 > ref0
                threshold_H = ref1+ Oshoot;
                threshold_L = -Inf;
            else
                threshold_L = ref1- Oshoot;
                threshold_H = Inf;
            end
        else 
            X(4,k+1) = Xk(4);
        end
        X(7,k+1) = V; % prev V 
    
    end
    % last V kept constant
    X(8,end) = V;
    
    status = 0; % required by latest Breach
end


#####

