function dxdt = step_function(x,fleet,t)
% -------------------------------------------------------------------------
% Description:
%   Calculate the state change per time step for the euler function. The
%   state vector contains the states of all cars. For each car we find the
%   input required for the current goal.
%
% Output:
%   dxdt
% -------------------------------------------------------------------------
global gen

% Update car states in car objects:
for i = 1:length(fleet)
    fleet(i).state = x(3*(i-1)+1:3*(i-1)+3);
    % change lane assignment if mission is accomplished:
    if fleet(i).lane ~= fleet(i).mission.lane && ...
       abs(fleet(i).state(2)-fleet(i).mission.lane) < gen.epsilon
            fleet(i).lane = fleet(i).mission.lane;
    end
end
            
% Calculate control input for each car and create 'dxdt'
dxdt = [];
U = [];
for i = 1:length(fleet)
    states = fleet(i).state;
%--- Change mission to specific times: -----------------------------------
    if t>=1
        fleet(2).mission.lane = 11;
    end
%     if t>=8
%         fleet(2).mission.lane = 2;
%     end
%     if t>=15
%         fleet(2).mission.lane = 1;
%     end
%-------------------------------------------------------------------------
    u = ctrl_main(i,fleet,t);
    dxdt(end+1:end+3,1) = fleet(i).dyn(states,u);
    U(end+1:end+2) = u;
    if i == 1
        gen.U1(end+1,1) = u(1);
    elseif i == 2
        gen.U2(end+1,1) = u(1);
    elseif i == 3
        gen.U3(end+1,1) = u(1);
    end
end  

% Update car input in car objects
for i = 1:length(fleet)
    fleet(i).input = U(2*(i-1)+1:2*(i-1)+2);
end

gen.t_last = t;
    
end