classdef switching_imitation_pb < imitation_pb

    methods
        function pb = switching_imitation_pb()
            pb = pb@imitation_pb('switching');

            %% Simulation time
            Ts = 0.1;
            Tf = 10;
            time = 0:Ts:Tf;
            pb.time=  time;


            %% Requirements
            STL_ReadFile('req_switching.stl');
            pb.phi = BreachRequirement('phi');


        end

        function B0 = create_nominal(this, scaling)
            mdl = 'switching_controller_nominal';
            B0 = BreachSimulinkSystem(mdl, {},[]);
            %% We consider piecewise constant inputs with two stps
            B0.SetInputGen('UniStep2')
            B0.SetParamRanges('ref_u0', scaling*[0.1 .5]);
            B0.SetParamRanges('ref_u1', scaling*[0 .5]);
        end


        function Bmpc = create_mpc_sys(this)
            mdl = 'switching_controller_nominal';
            Bmpc = BreachSimulinkSystem(mdl, {},[]);
           
        end


    end
end 