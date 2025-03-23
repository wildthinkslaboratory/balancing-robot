% Here we just run a simulation of the cart with no control
% so no motor.

clear all, close all, clc

m = 1;
M = 1;
L = 2;
g = -10;
d = 1;

tspan = 0:.1:10; 
x0 = [0; 0; pi; .5];
[t,x] = ode45(@(t,x)robotMotion(x,m,M,L,g,d,0),tspan,x0);



for k=1:length(t)
    drawRobot(x(k,:),m,M,L);
end

figure
plot(t,x)
