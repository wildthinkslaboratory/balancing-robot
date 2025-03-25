clear all, close all, clc

m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

% pendulum up (b=1)
% our pendulum is always up. We want a balancing robot
b = 1; 

A = [0 1 0 0;
    0 -d/M b*m*g/M 0;
    0 0 0 1;
    0 -b*d/(M*L) -b*(m+M)*g/(M*L) 0];
B = [0; 1/M; 0; b*1/(M*L)];

eig(A)
det(ctrb(A,B))

%%  Design LQR controller
Q = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
R = 10;

K = lqr(A,B,Q,R);
disp("K");
disp(K);

%% Simulate closed-loop system
tspan = 0:.001:10;
x0 = [-4; 0; pi+.1; 0];  % initial condition 
wr = [1; 0; pi; 0];      % reference position
u=@(x)-K*(x - wr);       % control law
[t,x] = ode45(@(t,x)moveRobot(x,m,M,L,g,d,u(x)),tspan,x0);

U = 0*t;
for k=1:length(t)
    U(k) = u(x(k));
end

for k=1:100:length(t)
    drawRobot(x(k,:),m,M,L);
end

figure(1)
plot(t,x,'LineWidth',2); hold on
l1 = legend('x','v','\theta','\omega')
set(l1,'Location','SouthEast')
set(gcf,'Position',[100 100 500 200])
xlabel('Time')
ylabel('State')
grid on
set(gcf,'PaperPositionMode','auto')
print('-depsc2', '-loose', './FIG_02_LQR');

figure(2)
plot(t,U,'LineWidth',2);
l1 = legend('u')
set(l1,'Location','SouthEast')
set(gcf,'Position',[100 100 500 200])
xlabel('Time')
ylabel('control')
grid on