%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% -----------------------------PLATOONING---------------------------------
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all

% ------------- Constants ------------- 
global gen

    gen.V1 = [];
    gen.eta1V1 = [];
    gen.eta2V1 = [];
    gen.t11 = [];
    gen.t111 = [];
    gen.t112 = [];
    gen.t12 = [];
    gen.t2 = [];
    gen.t22 = [];
    gen.t3 = [];
    gen.V1dot = [];
    gen.alpha = [];
    gen.gamma = [];
    gen.U1 = [];
    gen.U2 = [];
    gen.U3 = [];

gen.u =[];
% general parameters
gen.tau_D = 0.9;      % min. time distance allowed btw two vehicles for barrier

gen.t_last = 0;         % used in simulation to remember time of last step
gen.a_max = 4;          % maximum change in velocity per second
gen.omega_max = 1.5;    % maximum change in orientation per second

gen.dt = 0.05;          % time step
gen.t_end = 10;         % end time of simulation

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
% vehicle(vehicle_ID, platoon_pos, dynamics, initial_lane,initial_state,
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
gen.V1 = [];

% Run simulation
tspan = 0:gen.dt:gen.t_end;
initial_states = [];
for i = 1:length(fleet)
    initial_states(end+1:end+3) = fleet(i).state;
end

% [t,y] = ode45(@(t,x) step_function(x,fleet,t), tspan, initial_states);
[t,y] = euler(@step_function,tspan,initial_states,fleet);

%% ------------- Simulation plots ------------- 

% Create gif of moving cars
figure
p = plot(y(1,4),y(1,5),'*g',y(1,7),y(1,8),'*b',y(1,1),y(1,2),'*r');
xlabel('time $t$', 'Interpreter', 'latex')
title('Position')
yline(10.5,'-w','LineWidth',1.5)
yline(11.5,'--w','LineWidth',1.2)
yline(12.5,'-w','LineWidth',1.5)
ylim([10,13])
xlim([0,590])
set(gcf, 'Color','k')
lgnd = legend(p,'$Veh\:1$', '$Veh\:2$', '$Veh\:3$', 'Interpreter', 'latex','FontSize',12);
set(lgnd,'color','k')
set(lgnd,'TextColor','w')
axis off
txt = ['t = 0'];
text(10,12.7,txt,'Color','w')

gif('myfile01.gif')
for k = 2:length(y)
    p = plot(y(k,4),y(k,5),'*g',y(k,7),y(k,8),'*b',y(k,1),y(k,2),'*r');
    xlabel('time $t$', 'Interpreter', 'latex')
    title('Position')
    yline(10.5,'-w','LineWidth',1.5)
    yline(11.5,'--w','LineWidth',1.2)
    yline(12.5,'-w','LineWidth',1.5)
    ylim([10,13])
    xlim([0,590])
    set(gcf, 'Color','k')
    lgnd = legend(p,'$Veh\:1$', '$Veh\:2$', '$Veh\:3$', 'Interpreter', 'latex','FontSize',12);
    set(lgnd,'color','k')
    set(lgnd,'TextColor','w')
    axis off
    txt = ['t = ' num2str(t(k))];
    text(10,12.7,txt,'Color','w')
    
    gif
end

web('myfile01.gif')
% web('catch_up.gif')

figure(2)
subplot 211
plot(1:length(gen.b),gen.b)
subplot 212
plot(1:length(gen.bdot),gen.bdot)

figure(3)
plot(1:length(gen.delta1),gen.delta1)

figure(4)
plot(1:length(gen.sigma),gen.sigma)

figure(5)
plot(1:length(gen.V),gen.V)

%% ----------- Figure 1 of moving car ------------------

figure(2)
p = plot(y(1:end,4),y(1:end,5),'*g');
xlabel('time $t$', 'Interpreter', 'latex')
title('Position')
yline(10.5,'-w','LineWidth',1.5)
yline(11.5,'--w','LineWidth',1.2)
yline(12.5,'-w','LineWidth',1.5)
ylim([10,13])
xlim([0,450])
set(gcf, 'Color','k')
axis off
txt = ['t = [0,10]'];
text(10,12.7,txt,'Color','w','FontSize',12)
set(gcf, 'InvertHardCopy', 'off'); 
saveas(gcf,'Figures/Fig1LaneChange.png');

%% ----------- Figure 3 of moving car ------------------

figure(2)
p = plot(y(1:end,4),y(1:end,5),'*g',y(1:end,7),y(1:end,8),'*b');
xlabel('time $t$', 'Interpreter', 'latex')
title('Position')
yline(10.5,'-w','LineWidth',1.5)
yline(11.5,'--w','LineWidth',1.2)
yline(12.5,'-w','LineWidth',1.5)
ylim([10,13])
xlim([0,450])
set(gcf, 'Color','k')
axis off
txt = ['t = [0,10]'];
text(10,12.7,txt,'Color','w','FontSize',12)
set(gcf, 'InvertHardCopy', 'off'); 
saveas(gcf,'Figures/Fig1LaneChange.png');

clear input
input.data = [t(21:10:200)',y(21:10:200,4),y(21:10:200,5)-10,gen.U2(21:10:200,1),y(21:10:200,7),y(21:10:200,8)-10,gen.U3(21:10:200,1)];
input.dataFormat = {'%.1f',1,'%.2f',6};
latex = latexTable(input);
fid=fopen('MyLatex.tex','w');
[nrows,ncols] = size(latex);
for row = 1:nrows
    fprintf(fid,'%s\n',latex{row,:});
end
fclose(fid);

%% ----------- Figure 4 of moving car ------------------

figure(2)
p = plot(y(1:end,1),y(1:end,2),'*r',y(1:end,4),y(1:end,5),'*g',y(1:end,7),y(1:end,8),'*b');
xlabel('time $t$', 'Interpreter', 'latex')
title('Position')
yline(10.5,'-w','LineWidth',1.5)
yline(11.5,'--w','LineWidth',1.2)
yline(12.5,'-w','LineWidth',1.5)
ylim([10,13])
xlim([0,450])
set(gcf, 'Color','k')
axis off
txt = ['t = [0,10]'];
text(10,12.7,txt,'Color','w','FontSize',12)
set(gcf, 'InvertHardCopy', 'off'); 
saveas(gcf,'Figures/Fig1LaneChange.png');

clear input
input.data = [t(21:10:200)',y(21:10:200,4),y(21:10:200,5)-10,gen.U2(21:10:200,1),y(21:10:200,7),y(21:10:200,8)-10,gen.U3(21:10:200,1),y(21:10:200,1),y(21:10:200,2)-10,gen.U1(21:10:200,1)];
input.dataFormat = {'%.1f',1,'%.2f',9};
latex = latexTable(input);
fid=fopen('MyLatex.tex','w');
[nrows,ncols] = size(latex);
for row = 1:nrows
    fprintf(fid,'%s\n',latex{row,:});
end
fclose(fid);

%% ------------- Simulation plots (only carI) ------------- 

% % Create gif of moving cars
% figure
% p = plot(y(1,7),y(1,8),'*b');
% xlabel('time $t$', 'Interpreter', 'latex')
% title('Position')
% yline(0.5,'-w','LineWidth',1.5)
% yline(1.5,'--w','LineWidth',1.2)
% yline(2.5,'-w','LineWidth',1.5)
% ylim([0,3])
% xlim([0,450])
% set(gcf, 'Color','k')
% axis off
% txt = ['t = 0'];
% text(10,2.7,txt,'Color','w')
% 
% gif('myfile2.gif')
% for k = 2:length(y)
%     p = plot(y(k,7),y(k,8),'*b');
%     xlabel('time $t$', 'Interpreter', 'latex')
%     title('Position')
%     yline(0.5,'-w','LineWidth',1.5)
%     yline(1.5,'--w','LineWidth',1.2)
%     yline(2.5,'-w','LineWidth',1.5)
%     ylim([0,3])
%     xlim([0,450])
%     set(gcf, 'Color','k')
%     axis off
%     txt = ['t = ' num2str(t(k))];
%     text(10,2.7,txt,'Color','w')
%     
%     gif
% end
% 
web('myfileLC2.gif')
% % web('catch_up.gif')
% 
% figure(2)
% plot(1:length(gen.V),gen.V)

% %% ------------------------------------------------------------------------
% 
% figure(2)
% subplot 221
% plot(t(1:end-1),gen.V1,'LineWidth',1.2)
% ylabel('V_1')
% xlabel('t')
% title('c = 10 V_1^6')
% ylim([0,0.6])
% grid on
% 
% subplot 222
% plot(t(1:end-1),gen.t22,'LineWidth',1.2)
% ylabel('\omega_E')
% xlabel('t')
% ylim([-0.15,0.15])
% grid on
% 
% 
% subplot 223
% plot(t(1:end-1),gen.V1dot,t(1:end-1),-0.1*gen.alpha,'LineWidth',1.2)
% legend('$\dot{V}_1$','$-\alpha(\cdot)$','interpreter','latex')
% xlabel('t')
% grid on
% 
% subplot 224
% plot(t(1:end-1),-gen.gamma,t(1:end-1),gen.eta2V1-gen.gamma,'LineWidth',1.2)
% legend('$-\gamma(\cdot)$','$\dot{\eta}_0$','interpreter','latex')
% xlabel('t')
% grid on
% hold on
% 
% %%
% 
% % clear all
% load('dt005c01m01')
% figure(9)
% subplot 321
% plot(t(1:end-1),gen.V1,'LineWidth',1.2)
% ylabel('V_1')
% xlabel('t')
% title('c = 0.1, m = 0.1')
% grid on
% 
% subplot 323
% plot(t(1:end-1),gen.t22,'LineWidth',1.2)
% ylabel('\omega_E')
% xlabel('t')
% grid on
% 
% subplot 325
% plot(t(1:end),y(:,5)-10,'LineWidth',1.2)
% ylabel('y')
% xlabel('t')
% grid on
% 
% subplot 322
% plot(t(1:end-1),-gen.gamma,t(1:end-1),gen.eta2V1-gen.gamma,'LineWidth',1.2)
% legend('$-\gamma(\cdot)$','$\dot{\eta}_0$','interpreter','latex')
% xlabel('t')
% grid on
% 
% subplot 324
% plot(t(1:end-1),gen.V1dot,t(1:end-1),-gen.alpha,'LineWidth',1.2)
% legend('$\dot{V}_1$','$-\alpha(\cdot)$','interpreter','latex')
% xlabel('t')
% grid on