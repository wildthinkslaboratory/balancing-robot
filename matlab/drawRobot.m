function drawRobot(state,m,M,L)
x = state(1);
th = state(3);

wr = .3;         % wheel radius
mr = .3*sqrt(m); % mass radius
y = wr;          % height of wheel axle

% find the coordinates of the top of the robot
px = x + L*sin(th);
py = y - L*cos(th);

plot([-10 10],[0 0],'k','LineWidth',2)
hold on

plot([x px],[y py],'k','LineWidth',16) % body
rectangle('Position',[x-wr,y-wr,2*wr,2*wr],'Curvature',1,'FaceColor',[0 1 0]) % wheel
rectangle('Position',[px-mr/2,py-mr/2,mr,mr],'Curvature',1,'FaceColor',[.1 0.1 1]) % top

xlim([-5 5]);
ylim([-2 2.5]);
set(gcf,'Position',[100 550 1000 400])

drawnow
hold off