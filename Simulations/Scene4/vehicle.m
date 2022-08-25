classdef vehicle < handle
    properties
        vehicle_ID  % identification number
        mission     % reference velocity and lane
        dyn         % vehicle dynamcis
        lane;       % current lane
        state;      % current state of the vehicle
        input;      % current input of the vehicle
        logs;       % states, relax. variables, inputs
    end
    methods
        function self = vehicle(vehicle_ID, init_mission, dynamics, initial_lane, initial_state, initial_input)
            self.vehicle_ID = vehicle_ID;
            self.dyn = dynamics;
            self.state = initial_state;
            self.lane = initial_lane;
            self.input = initial_input;
            self.mission = init_mission;
        end
    end
end