function pb = init_vehicle_problem()

pb=  imitation_pb('MpcVehicle_model');

pb.params =struct(); % no extra parameter
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




%% simulation

Ts = 0.1;
Tf = 10;
time = 0:Ts:Tf;
pb.time=  time;

% sim_mpc function
params_mpc_vehicle = setParamsVehicle();

path = generateDesiredPath(params_mpc_vehicle);

% build the safe region around the path
safeRegion = params_mpc_vehicle.buildSafeRegion(params_mpc_vehicle, path);

% path is expanded to authorize MPC at the end of path
expandPath = generateExpandPath(params_mpc_vehicle, path);

pb.mpc_controlfn = @(x) mpc_vehicle_control(params_mpc_vehicle,path,expandPath, x);

pb.sim_fn = @(Sys,time,p,controlfn)(sim_breach_vehicle(controlfn,params_mpc_vehicle, path, safeRegion,time,p));


% plotting function 

pb.plot_sim = @(t,X,ax) (plot_sim_vehicle(params_mpc_vehicle, t,X,ax));


%% Requirements

STL_ReadFile('req_vehicle.stl');
pb.phi = BreachRequirement('phi_safety');

