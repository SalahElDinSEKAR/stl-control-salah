function pb = init_drone_problem()

% sim_mpc function
params_mpc_drone = setParamsDrone();

path = generateDesiredPath(params_mpc_drone);

% build the safe region around the path
safeRegion = params_mpc_drone.buildSafeRegion(params_mpc_drone, path);

% path is expanded to authorize MPC at the end of path
expandPath = generateExpandPath(params_mpc_drone, path);

% first waypoint
x_wp = path.wayPoints(2,1);
y_wp = path.wayPoints(2,2);
z_wp = path.wayPoints(2,3);
heading_t = path.heading(2,1);
heading_p = path.heading(2,2);
vel_target = params_mpc_drone.velocityTarget;

pb=  imitation_pb('MpcDrone_model');

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
params_mpc_drone = setParamsDrone();

path = generateDesiredPath(params_mpc_drone);

% build the safe region around the path
safeRegion = params_mpc_drone.buildSafeRegion(params_mpc_drone, path);

% path is expanded to authorize MPC at the end of path
expandPath = generateExpandPath(params_mpc_drone, path);

pb.mpc_controlfn = @(x) mpc_drone_control(params_mpc_drone,path,expandPath, x);

pb.sim_fn = @(Sys,time,p,controlfn)(sim_breach_drone(controlfn,params_mpc_drone, path, safeRegion,time,p));


% plotting function 

pb.plot_sim = @(t,X,ax) (plot_sim_drone(params_mpc_drone, t,X,ax));

%% Requirements

STL_ReadFile('requirements.stl');
pb.phi = BreachRequirement('phi_safety');




end
