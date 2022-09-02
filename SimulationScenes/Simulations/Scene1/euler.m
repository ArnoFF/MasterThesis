function [tspan,y] = euler(stepfct,tspan,init,fleet)

% number of steps
n = length(tspan);

% time step
dt = tspan(2)-tspan(1);

y = zeros(n,length(init));
y(1,:) = init;
x = init;

% iteration
for i = 2:n
    t = tspan(i);
    y(i,:) = x + (dt.*stepfct(x,fleet,t))';
    x = y(i,:);
end

end