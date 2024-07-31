function [Vs,Vf] = final_control(y)
    %%
    % two different input
    Vs= double(pid_watertank_control_S(y));
    Vf= double(pid_watertank_control(y));
   

end

