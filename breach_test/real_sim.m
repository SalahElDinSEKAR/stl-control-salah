function [t,X, p, status] = sim_breach_watertank(control_fn, t, p)
    %% collect parameters
    DimX = 9;                 % last one is cost, which we use with MPC, will see for PID...
    params = p(DimX+1:end); % p(1:DimX) is always 0 for legacy reasons 
    Xinit = params(1:7);     % Initial state 
    ref1  = params(8);    % Additional parameter(s)
 
    %% Init signal output
    X = zeros(DimX, numel(t)); % preparing the signals output array
    X(1:7, 1) = Xinit; % Initial state

    Ts = t(2)-t(1);
    
    %% Main simulation loop
    for k = 1:numel(t)-1
        Xk = X(:,k);   % current state, we want to compute Xk+1 
       
        %% computes next control input, either with pid or NN
        V = double(control_fn(Xk(1:7)));
        X(8,k) = V; % new V
                          % here we could assign a cost for new V, X(9,k)=?
   
        %% update plant
        H = Xk(1);      % current height
        ODEFUN = @(t,H) PlantDynamicsFn(H,V);
        [~,H_OUT] = ode45(ODEFUN,[0 Ts], H);
        
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





