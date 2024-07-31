function u = pid_watertank_control_S(y)
    % y = [y1 y2 y3 y4 y5 y6 y7];
    % y1 = H
    % y2 = Hp
    % y3 = Hpp
    % y4 = refL
    % y5 = ref_p
    % y6 = ref_pp
    % y7 = up

    e = y(1) - y(4);
    ep = y(2) - y(5);
    epp = y(3) - y(6);
   
    Kp = -20.8521889762703;
    Ki = -19.9734819620964;
    Kd =- 0.820834909861585;
    
    Ts = .1;
    a = Kp + Ki*Ts/2 + Kd/Ts;
    b = -Kp + Ki*Ts/2 - 2*Kd/Ts;
    c = Kd/Ts;

    u = y(7) + a*e + b*ep + c*epp;
    u = max(u, -10);
    u = min(u, 10);
end
