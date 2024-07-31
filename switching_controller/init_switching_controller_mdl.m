mdl = 'switching_controller_nominal';
simulation.time_step = .1;
simulation.time_window = 10;

B = BreachSimulinkSystem(mdl, {},[]);