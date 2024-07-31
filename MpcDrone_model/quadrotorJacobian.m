function [D, U] = quadrotorJacobian(params, u, x)

% reminder : quadrotor model
% quadrotor model with 
% states
%       x(1), x(2), x(3): x, y, z positions
%       x(4), x(5), x(6): dotx, doty, dotz derivatives of positions
%       x(7), x(8), x(9): phi, theta, psi (roll, pitch and yaw angles)
%       x(10), x(11), x(12): dotphi, dottheta, dotpsi derivatives of angles
% control inputs 
%       u(1): Thrust
%       u(2), u(3), u(4): Roll, pitch and yaw torque
%
% % attitude controller (continuous-time proportional controllers)
% PhiCommand = params.Kphi*(u(2)-x(7));
% ThetaCommand = params.Ktheta*(u(3)-x(8));
% PsiCommand = params.Kpsi*(u(4)-x(9));
% % attitude rates controller (continuous-time proportional controllers)
% dotPhiCommand = params.KdotPhi*(PhiCommand-x(10));
% dotThetaCommand = params.KdotTheta*(ThetaCommand-x(11));
% dotPsiCommand = params.KdotPsi*(PsiCommand-x(12));
% 
% % quadrotor dynamics   
% dx = zeros(12,1);
% 
% dx(1) = x(4);
% dx(2) = x(5);
% dx(3) = x(6);
% dx(4) = x(8)*u(1)/params.m; 
% dx(5) = -x(7)*u(1)/params.m; 
% dx(6) = (-params.m*params.g + u(1))/params.m;
% dx(7) = x(10);
% dx(8) = x(11);
% dx(9) = x(12);
% dx(10) = dotPhiCommand/params.Ix;
% dx(11) = dotThetaCommand/params.Iy;
% dx(12) = dotPsiCommand/params.Iz;

    % D: partial derivatives with respect to state variables x(1), x(2),
    % ..., x(params.nStates)
    D = zeros(params.nStates,params.nStates);
    % U: partial derivatives with respect to input variables u(1), u(2), ...,
    % u(params.nControlInputs)
    U = zeros(params.nStates,params.nControlInputs);
    
    D(1,4) = 1;
    D(2,5) = 1;
    D(3,6) = 1;
    D(4,8) = u(1)/params.m;
    D(5,7) = -u(1)/params.m;
    D(7,10) = 1;
    D(8,11) = 1;
    D(9,12) = 1;
    D(10,7) = -params.Kphi*params.KdotPhi/params.Ix;
    D(10,10) = -params.KdotPhi/params.Ix;
    D(11,8) = -params.Ktheta*params.KdotTheta/params.Iy;
    D(11,11) = -params.KdotTheta/params.Iy;
    D(12,9) = -params.Kpsi*params.KdotPsi/params.Iz;
    D(12,12) = -params.KdotPsi/params.Iz;
    
    U(4,1) = x(8)/params.m;
    U(5,1) = -x(7)/params.m;
    U(6,1) = 1/params.m;
    U(10,2) = params.Kphi*params.KdotPhi/params.Ix;
    U(11,3) = params.Ktheta*params.KdotTheta/params.Iy;
    U(12,4) = params.Kpsi*params.KdotPsi/params.Iz;

end

