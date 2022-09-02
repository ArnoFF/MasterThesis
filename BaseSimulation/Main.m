%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% -------------------------Main Sim Function-------------------------------
%
% -------------------------------------------------------------------------
% Description:
%   Main simulation function. Initialize constants and vehicles, start step
%   function, create gif.
% -------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all

global gen
    
% ------------- Constants ------------- 
gen.u =[];
% general parameters
gen.tau_D = 0.9;      % min. time distance allowed btw two vehicles for barrier

gen.t_last = 0;         % used in simulation to remember time of last step
gen.a_max = 4;          % maximum change in velocity per second
gen.omega_max = 1.5;    % maximum change in orientation per second

gen.dt = 0.05;          % time step
gen.t_end = 12;         % end time of simulation

gen.y_max = [11.5 12.5]; % y_max for respective lanes
gen.y_min = [10.5 11.5]; % y_min for respective lanes

gen.sens_range = 100;  % distance to recognize other vehicles
gen.epsilon = 0.05;    % bandwidth used to change .lane when mission accomplished

% ------------- Vehicle dynamcis ------------- 
% states: [x,y,psi]; input: [v,omega]
         
gen.g = @(x) [cos(x(3)) 0;
              sin(x(3)) 0;
              0         1];

gen.dyn = @(x,u) gen.g(x)*u;

%% ------------- Vehicles -------------
% Initialize vehicle data:
% vehicle(vehicle_ID, mission, dynamics, initial_lane, initial_state,
% initial_input)

% Vehicle 1
init.c1_states = [110 12 0];
init.c1_velocity = 25;
init.c1_omega = 0;
init.c1_ID = 1;
init.c1_mission.v = 25;
init.c1_mission.lane = 12;

car1 = vehicle(init.c1_ID,init.c1_mission,gen.dyn,init.c1_states(2),init.c1_states,[init.c1_velocity;init.c1_omega]);

% Vehicle 2
init.c2_states = [100 11 0];
init.c2_velocity = 25;
init.c2_omega = 0;
init.c2_ID = 2;
init.c2_mission.v = 25;
init.c2_mission.lane = 11;

car2 = vehicle(init.c2_ID,init.c2_mission,gen.dyn,init.c2_states(2),init.c2_states,[init.c2_velocity;init.c2_omega]);

% Vehicle 3
init.c3_states = [90 12 0];
init.c3_velocity = 25;
init.c3_omega = 0;
init.c3_ID = 3;
init.c3_mission.v = 25;
init.c3_mission.lane = 12;

car3 = vehicle(init.c3_ID,init.c3_mission,gen.dyn,init.c3_states(2),init.c3_states,[init.c3_velocity;init.c3_omega]);

fleet = [car1;car2;car3];

%%  ------------- Simulation ------------- 

% Simulation data initialization
s.Delta_r = [];
s.U_1 = [];
s.CLF = [];

gen.b = [];     % used in Simulation to save value of Lyapunov function
gen.bdot = [];
gen.dot = 1;    % used in Simulation to count time steps
gen.delta1 = [];

% Run simulation
tspan = 0:gen.dt:gen.t_end;
initial_states = [];
for i = 1:length(fleet)
    initial_states(end+1:end+3) = fleet(i).state;
end

% [t,y] = ode45(@(t,x) step_function(x,fleet,t), tspan, initial_states);
[t,y] = FwdEuler(@step_function,tspan,initial_states,fleet);

%% ------------- Simulation GIF ------------- 

% Create gif of moving cars
figure
p = plot(y(1,4),y(1,5),'*g',y(1,7),y(1,8),'*y',y(1,1),y(1,2),'*r','MarkerSize',10);
xlabel('time $t$', 'Interpreter', 'latex')
title('Position')
yline(10.5,'-w','LineWidth',1.5)
yline(11.5,'--w','LineWidth',1.2)
yline(12.5,'-w','LineWidth',1.5)
ylim([10,13])
xlim([90,410])
set(gcf, 'Color','k')
lgnd = legend(p,'$Veh\:1$', '$Veh\:2$', '$Veh\:3$', 'Interpreter', 'latex','FontSize',12);
set(lgnd,'color','k')
set(lgnd,'TextColor','w')
axis off
txt = ['t = 0'];
text(90,12.7,txt,'Color','w')

gif('myfile01.gif')
for k = 2:length(y)
    p = plot(y(k,4),y(k,5),'*g',y(k,7),y(k,8),'*y',y(k,1),y(k,2),'*r','MarkerSize',10);
    xlabel('time $t$', 'Interpreter', 'latex')
    title('Position')
    yline(10.5,'-w','LineWidth',1.5)
    yline(11.5,'--w','LineWidth',1.2)
    yline(12.5,'-w','LineWidth',1.5)
    ylim([10,13])
    xlim([90,410])
    set(gcf, 'Color','k')
    lgnd = legend(p,'$Veh\:1$', '$Veh\:2$', '$Veh\:3$', 'Interpreter', 'latex','FontSize',12);
    set(lgnd,'color','k')
    set(lgnd,'TextColor','w')
    axis off
    txt = ['t = ' num2str(t(k))];
    text(90,12.7,txt,'Color','w')
    
    gif
end

web('myfile01.gif')
