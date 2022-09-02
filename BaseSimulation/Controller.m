function [u] = Controller(ego_ID,fleet,t)
% -------------------------------------------------------------------------
% Description:
%   Controller for any vehicle. Mission: velocity and lane. Change of lane
%   without collision.
%
% Output:
%   u : [v_I, omega_I]
% -------------------------------------------------------------------------

global gen

% Detect other vehicles 
[other_vehicles] = detect_vehicles(fleet,ego_ID);
unpackStruct(other_vehicles)

% time step
dt = t-gen.t_last;

% lane limits
y_max = gen.y_max(fleet(ego_ID).lane-10);
y_min = gen.y_min(fleet(ego_ID).lane-10);

% Input limits
v_max = vE+gen.a_max*dt;
v_min = vE-gen.a_max*dt;
omega_max = gen.omega_max*dt;

% Mission
v_ref = fleet(ego_ID).mission.v;
y_ref = fleet(ego_ID).mission.lane;
    
%--------------------------------------------------------------------------
% -------------------- Barrier and Lyapunov functions ---------------------
%--------------------------------------------------------------------------
%--- (steering) Lyapunov function to go to middle of the lane
    V1 = 0.5*(y_ref-xE(2))^2;
    cV11 = 10;
    eta1_1 = @(ve) (y_ref-xE(2))*ve*sin(psiE)-cV11*V1;
    cV12 = 1;
    eta1_2 = @(ve,omegae) ...
                [-ve*sin(psiE)+cV11*(y_ref-xE(2));
                 (y_ref-xE(2))*ve*cos(psiE)]'*...
                [ve*sin(psiE);
                 omegae]+...
                cV12*eta1_1(ve);

%--- (safety distance 0F) Barrier function to keep distance to vehicle in front
    b1 = x0F(1)-xE(1)-gen.tau_D*vE;
    c1 = 1;
    
%--- (y_min) High order barrier function for the lane change 2 to 1

    [lambda21,dlambda21,ddlambda21] = lambda(xE,xM1B,vM1B);
    [lambda22,dlambda22,ddlambda22] = lambda(xM1F,xE,vE);
    
    b21 = xE(2)-y_min+lambda21;
    b22 = xE(2)-y_min+lambda22;
    c211 = 100;
    c212 = 100;
    c221 = 100;
    c222 = 100;
    
    if b21 <= b22
    % b11 active (-1B)
    psi2_1 = @(ve) ve*sin(psiE)-dlambda21*ve*cos(psiE)+...
                   dlambda21*vM1B*cos(psiM1B)+c211*b21;
    psi2_2 = @(ve,omegae)...
                [ve*cos(psiE)+dlambda21*ve*sin(psiE);
                 -dlambda21*vM1B*sin(psiM1B);
                 1;
                 (-dlambda21+ddlambda21*ve*cos(psiE)-ddlambda21*vM1B*cos(psiM1B));
                 (dlambda21+ddlambda21*vM1B*cos(psiM1B)-ddlambda21*ve*cos(psiE))]'*...
                 [omegae;
                  omegaM1B;
                  ve*sin(psiE);
                  ve*cos(psiE);
                  vM1B*cos(psiM1B)]+c212*psi2_1(ve);
           
    else
    % b12 active (-1F)
    psi2_1 = @(ve) ve*sin(psiE)+dlambda22*ve*cos(psiE)...
                   -dlambda22*vM1F*cos(psiM1F)+c221*b22;
    psi2_2 = @(ve,omegae)...
                [ve*cos(psiE)-dlambda22*ve*sin(psiE);
                 -dlambda22*vM1F*sin(psiM1F);
                 1;
                 (dlambda22-ddlambda22*ve*cos(psiE)+ddlambda22*vM1F*cos(psiM1F));
                 (-dlambda22-ddlambda22*vM1F*cos(psiM1F)+ddlambda22*ve*cos(psiE))]'*...
                 [omegae;
                  omegaM1F;
                  ve*sin(psiE);
                  ve*cos(psiE);
                  vM1F*cos(psiM1F)]+c222*psi2_1(ve);
    end
           
%--- (y_max) High order barrier function for the lane change 1 to 2
    % (+1B)
    [lambda31,dlambda31,ddlambda31] = lambda(xE,xP1B,vP1B);
    % (+1F)
    [lambda32,dlambda32,ddlambda32] = lambda(xP1F,xE,vE);
    
    b31 = lambda31+y_max-xE(2);
    b32 = lambda32+y_max-xE(2);
    c311 = 1;
    c312 = 1;
    c321 = 1;
    c322 = 1;
    
    if b31 <= b32
    % b31 active (+1B)
    psi3_1 = @(ve)   -ve*sin(psiE)+dlambda31*ve*cos(psiE)...
                     -dlambda31*vP1B*cos(psiP1B)+c311*b31;
    psi3_2 = @(ve,omegae) ...
                [-ve*cos(psiE)-dlambda31*ve*sin(psiE);
                 dlambda31*vP1B*sin(psiP1B);
                 -1;
                 (dlambda31+ddlambda31*ve*cos(psiE)-ddlambda31*vP1B*cos(psiP1B));
                 (-dlambda31+ddlambda31*vP1B*cos(psiP1B)-ddlambda31*ve*cos(psiE))]'*...
                [omegae;
                 omegaP1B;
                 ve*sin(psiE);
                 ve*cos(psiE);
                 vP1B*cos(psiP1B)]+c312*psi3_1(ve);
           
    else
    % b32 active (+1F)
    psi3_1 = @(ve)   -ve*sin(psiE)-dlambda32*ve*cos(psiE)...
                     +dlambda32*vP1F*cos(psiP1F)+c321*b32;
    psi3_2 = @(ve,omegae) ...
                [-ve*cos(psiE)+dlambda32*ve*sin(psiE);
                 -dlambda32*vP1F*sin(psiP1F);
                 -1;
                 (-dlambda32+ddlambda32*ve*cos(psiE)-ddlambda32*vP1F*cos(psiP1F));
                 (dlambda32+ddlambda32*vP1F*cos(psiP1F)-ddlambda32*ve*cos(psiE))]'*...
                 [omegae;
                  omegaP1B;
                  ve*sin(psiE);
                  ve*cos(psiE);
                  vP1B*cos(psiP1B)]+c322*psi3_1(ve);
    end
           
%--- (gap +/-1F) Barrier function to keep distance to incoming vehicle
    % x1: vehicle on other lane in front (FO)
    % x2: vehicle for which the controller is for (stays in its lane)
    
    sigma = @(x) 1.03./(1+exp(11*(x-0.6)))-0.02;
    
    % (-1F)
    b4 = xM1F(1)-xE(1)-gen.tau_D*vE*sigma(xE(2)-xM1F(2));
    
    b4dot = ...
        [-1;
         1;
         -gen.tau_D*vE*11.33*exp(11*(xE(2)-xM1F(2)-0.6))*(1+exp(11*(xE(2)-xM1F(2)-0.6)))^-2;
         gen.tau_D*vE*11.33*exp(11*(xE(2)-xM1F(2)-0.6))*(1+exp(11*(xE(2)-xM1F(2)-0.6)))^-2]';              
    c4 = 1;
    
    % (+1F)
    b5 = xP1F(1)-xE(1)-gen.tau_D*vE*sigma(xP1F(2)-xE(2));
    
    b5dot = ...
      [-1;
       1;
       -gen.tau_D*vE*11.33*exp(11*(xP1F(2)-xE(2)-0.6))*(1+exp(11*(xP1F(2)-xE(2)-0.6)))^-2;
       gen.tau_D*vE*11.33*exp(11*(xP1F(2)-xE(2)-0.6))*(1+exp(11*(xP1F(2)-xE(2)-0.6)))^-2]';              
    c5 = 1;
    
%--------------------------------------------------------------------------
%------------------------------ Controller --------------------------------
%--------------------------------------------------------------------------
    % cost function
    % xi(1): vE; xi(2): omegaE, xi(3): delta1; xi(4): delta2
    cost_fcn = @(xi) xi(1)^2 +70000*xi(2)^2 +1e9*xi(3)^2 +1e9*xi(4)^2;
    
    % Nonlinear constraints: V1,b2,b3
    c = @(xi) [-eta1_2(xi(1),xi(2))-xi(4);   % V1
              -psi2_2(xi(1),xi(2));         % b2
              -psi3_2(xi(1),xi(2))];        % b3
    ceq = [];
    nonlinfcn = @(xi) deal(c(xi),ceq);
    
    % Linear constraints: b1,b4,b5
    A = [cos(psiE) 0 0 0;                            % b1
         -b4dot([1,3])*[cos(psiE);sin(psiE)] 0 0 0;  % b4
         -b5dot([1,3])*[cos(psiE);sin(psiE)] 0 0 0]; % b5

    b = [v0F*cos(psi0F)+c1*b1;
         b4dot([2,4])*[vM1F*cos(psiM1F);vM1F*sin(psiM1F)]+c4*b4;
         b5dot([2,4])*[vP1F*cos(psiP1F);vP1F*sin(psiP1F)]+c5*b5];

    Aeq = [1 0 1 0];
    beq = v_ref;
    
    lb = [v_min,-omega_max,-1e10,-1e10];
    ub = [v_max,omega_max,1e10,1e10];

    % run optimization
    init_u = [vE,omegaE,0,0];
    options = optimoptions('fmincon','Display','off');
    xi_opt = fmincon(cost_fcn,init_u,A,b,Aeq,beq,lb,ub,nonlinfcn,options);
%--------------------------------------------------------------------------   
% Vehicle input
    u = [xi_opt(1);xi_opt(2)];
    
    % for logging purpose:
    gen.u(end+1,ego_ID*2-1:ego_ID*2)=u;  
    
end
