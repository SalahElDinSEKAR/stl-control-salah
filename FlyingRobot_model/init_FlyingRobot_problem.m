function pb = init_FlyingRobot_problem()

pb = imitation_pb('FlyingRobot_model');
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

% grid resolution
pb.states.x1.grid_res= diff(pb.states.x1.range_init)/10;
pb.states.x2.grid_res= diff(pb.states.x2.range_init)/10;
pb.states.theta.grid_res=diff(pb.states.theta.range_init)/10;
pb.states.v1.grid_res= diff(pb.states.v1.range_init)/10;
pb.states.v2.grid_res=diff(pb.states.v2.range_init)/10;
pb.states.omega.grid_res = diff(pb.states.omega.range_init)/10;
pb.states.u1_prev.grid_res = diff(pb.states.u1_prev.range_init)/10;
pb.states.u2_prev.grid_res = diff(pb.states.u2_prev.range_init)/10;

%% Control 
% inputs 
pb.controls.u1.range = [-UMAX UMAX];
pb.controls.u2.range = [-UMAX UMAX];

% MPC configuration and function
mpcverbosity off;
nlobj= createMPCobjImFlyingRobot(UMAX);
mpc_options= nlmpcmoveopt;
pb.mpc_controlfn = @(x) nlmpcmove(nlobj,x(1:6),x(7:8),zeros(1,6),[],mpc_options);

% Sim function
Ts = nlobj.Ts;
Tf = 15;
time = 0:Ts:Tf;
pb.time= [time Tf];

pb.sim_fn = @(Sys,time,p,controlfn) sim_breach_flying(controlfn,time,p);

%% Requirement
STL_ReadFile('requirements.stl');
pb.phi = BreachRequirement('phi_stable');

end