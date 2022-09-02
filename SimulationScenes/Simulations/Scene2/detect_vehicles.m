function [vo] = detect_vehicles(fleet,ego_ID)
global gen

% initialize indices
if ego_ID == 1
    i0F = 2;
    i0B = 2;    
    iP1F = 2;
    iP1B = 2;
    iM1F = 2;
    iM1B = 2; 
else
    i0F = 1;
    i0B = 1;    
    iP1F = 1;
    iP1B = 1;
    iM1F = 1;
    iM1B = 1; 
end

mock0F = 1; % if it stays 1 we have to set mock values
mock0B = 1;
mockP1F = 1;
mockP1B = 1;
mockM1F = 1;
mockM1B = 1;
for i  = 1:length(fleet)
    if i == ego_ID
        continue
    end
    % lane: 0
    if fleet(i).lane == fleet(ego_ID).lane
        % 0F
        if fleet(i).state(1) > fleet(ego_ID).state(1)
            if fleet(i).state(1) < fleet(i0F).state(1) || mock0F == 1
                i0F = i;
            end
            mock0F = 0; % there exists a vehicle 0F
        end
        % 0B
        if fleet(i).state(1) < fleet(ego_ID).state(1)
            if fleet(i).state(1) > fleet(i0B).state(1) || mock0B == 1
                i0B = i;
            end
            mock0B = 0; % there exists a vehicle 0B
        end
    end
    % lane: 1
    if fleet(i).lane == fleet(ego_ID).lane + 1
        % P1F
        if fleet(i).state(1) > fleet(ego_ID).state(1)
            if fleet(i).state(1) < fleet(iP1F).state(1) || mockP1F == 1
                iP1F = i;
            end
            mockP1F = 0; % there exists a vehicle P1F
        end
        % P1B
        if fleet(i).state(1) < fleet(ego_ID).state(1)
            if fleet(i).state(1) > fleet(iP1B).state(1) || mockP1B == 1
                iP1B = i;
            end
            mockP1B = 0; % there exists a vehicle P1B
        end
    end
    % lane: -1
    if fleet(i).lane == fleet(ego_ID).lane - 1
        % M1F
        if fleet(i).state(1) > fleet(ego_ID).state(1)
            if fleet(i).state(1) < fleet(iM1F).state(1) || mockM1F == 1
                iM1F = i;
            end
            mockM1F = 0; % there exists a vehicle M1F
        end
        % M1B
        if fleet(i).state(1) < fleet(ego_ID).state(1)
            if fleet(i).state(1) > fleet(iM1B).state(1) || mockM1B == 1
                iM1B = i;
            end
            mockM1B = 0; % there exists a vehicle M1B
        end
    end
end

% 0F
if mock0F == 0
    vo.x0F = fleet(i0F).state(1:2);
    vo.psi0F = fleet(i0F).state(3);
    vo.v0F = fleet(i0F).input(1);
    vo.omega0F = fleet(i0F).input(2);
else
    % use mock values
    vo.x0F(1) = fleet(ego_ID).state(1)+gen.sens_range;
    vo.x0F(2) = fleet(ego_ID).lane;
    vo.psi0F = 0;
    vo.v0F = fleet(ego_ID).input(1);
    vo.omega0F = 0;
end

% 0B
if mock0B == 0
    vo.x0B = fleet(i0B).state(1:2);
    vo.psi0B = fleet(i0B).state(3);
    vo.v0B = fleet(i0B).input(1);
    vo.omega0B = fleet(i0B).input(2);
else 
    % use mock values
    vo.x0B(1) = fleet(ego_ID).state(1)-gen.sens_range;
    vo.x0B(2) = fleet(ego_ID).lane;
    vo.psi0B = 0;
    vo.v0B = fleet(ego_ID).input(1);
    vo.omega0B = 0;
end

% P1F
if mockP1F == 0
    vo.xP1F = fleet(iP1F).state(1:2);
    vo.psiP1F = fleet(iP1F).state(3);
    vo.vP1F = fleet(iP1F).input(1);
    vo.omegaP1F = fleet(iP1F).input(2);
else
    % use mock values
    vo.xP1F(1) = fleet(ego_ID).state(1)+gen.sens_range;
    vo.xP1F(2) = fleet(ego_ID).lane + 1;
    vo.psiP1F = 0;
    vo.vP1F = fleet(ego_ID).input(1);
    vo.omegaP1F = 0;
end

% P1B
if mockP1B == 0
    vo.xP1B = fleet(iP1B).state(1:2);
    vo.psiP1B = fleet(iP1B).state(3);
    vo.vP1B = fleet(iP1B).input(1);
    vo.omegaP1B = fleet(iP1B).input(2);
else 
    % use mock values
    vo.xP1B(1) = fleet(ego_ID).state(1)-gen.sens_range;
    vo.xP1B(2) = fleet(ego_ID).lane + 1;
    vo.psiP1B = 0;
    vo.vP1B = fleet(ego_ID).input(1);
    vo.omegaP1B = 0;
end

% M1F
if mockM1F == 0
    vo.xM1F = fleet(iM1F).state(1:2);
    vo.psiM1F = fleet(iM1F).state(3);
    vo.vM1F = fleet(iM1F).input(1);
    vo.omegaM1F = fleet(iM1F).input(2);
else
    % use mock values
    vo.xM1F(1) = fleet(ego_ID).state(1)+gen.sens_range;
    vo.xM1F(2) = fleet(ego_ID).lane - 1;
    vo.psiM1F = 0;
    vo.vM1F = fleet(ego_ID).input(1);
    vo.omegaM1F = 0;
end

% M1B
if mockM1B == 0
    vo.xM1B = fleet(iM1B).state(1:2);
    vo.psiM1B = fleet(iM1B).state(3);
    vo.vM1B = fleet(iM1B).input(1);
    vo.omegaM1B = fleet(iM1B).input(2);
else 
    % use mock values
    vo.xM1B(1) = fleet(ego_ID).state(1)-gen.sens_range;
    vo.xM1B(2) = fleet(ego_ID).lane - 1;
    vo.psiM1B = 0;
    vo.vM1B = fleet(ego_ID).input(1);
    vo.omegaM1B = 0;
end

% Ego vehicle
vo.xE = fleet(ego_ID).state(1:2);
vo.psiE = fleet(ego_ID).state(3);
vo.vE = fleet(ego_ID).input(1);
vo.omegaE = fleet(ego_ID).input(2);

end