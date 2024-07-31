%% Calibration of formula for the nominal controller

init_switching_controller_mdl;
Bnom = B.copy();

%% We consider piecewise constant inputs with two stps
Bnom.SetInputGen('UniStep2')
Bnom.SetParamRanges('ref_u0', [0.1 .5]);
Bnom.SetParamRanges('ref_u1', [0 .5]);

%% Testing on 100 traces
Bnom.SampleDomain(100);
Bnom.Sim();

STL_ReadFile('req_switching.stl');
R = BreachRequirement('phi');
val  = R.Eval(Bnom);
if val <0
    warning('Requirement is not satisfied by nominal controller');
end