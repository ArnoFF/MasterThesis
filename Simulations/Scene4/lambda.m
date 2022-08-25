function [lambda,dlambda,ddlambda] = lambda(x1,x2,v2)
global gen
vB = v2;
x = (x1(1)-x2(1))/(gen.tau_D*vB);

if x <= 0.9
    lambda = 0.5/0.9*x;
    dlambda = (0.5/0.9)/(gen.tau_D*vB);
    ddlambda = 0;
elseif x > 0.9 && x <= 1
    a2 = (-2.145+sqrt(0.002625))/2.4;
    a1 = 0.5/(3*0.9)*1/(0.9+a2)^2;
    a3 = 0.5-0.5/(3*0.9)*(0.9+a2);
    lambda = a1*(x+a2)^3+a3;
    dlambda = 3*a1*(x+a2)^2/(gen.tau_D*vB);
    ddlambda = 6*a1*(x+a2)/(gen.tau_D*vB)^2;
elseif x > 1   
    b3 = 0.01;
    b2 = -0.9962;
    b1 = 4.5951/(1+b2);
    
    lambda = 1/(1+exp(-b1*(x+b2)))+b3;
    dlambda = (1+exp(-b1*(x+b2)))^-2*b1*exp(-b1*(x+b2))/(gen.tau_D*vB);
    ddlambda = 2*(1+exp(-b1*(x+b2)))^-3*(-b1*exp(-b1*(x+b2)))^2*1/(gen.tau_D*vB)^2 ...
             -(1+exp(-b1*(x+b2)))^-2*(-b1)^2*exp(-b1*(x+b2))*1/(gen.tau_D*vB)^2;
end

end

