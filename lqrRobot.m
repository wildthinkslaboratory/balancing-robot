clear all, close all, clc

m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

s = 1; % pendulum up (s=1)

A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];



B = [0; 1/M; 0; s*1/(M*L)];
eig(A)

% here we can weight which parts of the state
% we care most about getting to.
Q = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
% how much to we care about minimizing the force of engine
% it's cool to turn this up higher than the values in Q
% the cart uses the weight of the pendulum to push the cart
% keeping the cart cost low.
R = 10;

%%
det(ctrb(A,B))

%%
K = lqr(A,B,Q,R);


%% Simulate closed-loop system
tspan = 0:.001:20;
x0 = [-1; 0; pi-0.2; 0]; % initial condition 
wr = [1; 0; pi; 0];      % reference position
u=@(x)-K*(x - wr);       % control law

% simulate the cart with equations of motion
%[t,x] = ode45(@(t,x)cartpend(x,m,M,L,g,d,u(x)),tspan,x0);


% run the simulation with the linear approximation
[t,x] = ode45(@(t,x)((A-B*K)*(x- wr)),tspan,x0);

% collect the cotrol data
U = zeros( size( t )) ;
for k=1:100:length(t)
    U(k) = u(x(k));
end

for k=1:100:length(t)
    drawRobot(x(k,:),m,M,L);
end

figure
plot(t,x)

figure
plot(t,U)