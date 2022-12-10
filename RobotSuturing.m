function [] = RobotSuturing(NeedleRadius, WoundWidth, Depth,time)

addpath ../../

%% Physical parameters of the problem 

p.w = WoundWidth; % wound width in m
p.d = Depth; % wound depth in m
p.m = 0.030; % needle weight in kg.
% p.r = 0.0083;% needle radius in mm (for a half circle needle).
p.r = NeedleRadius;
p.I = p.m/2 *p.r^2; % to be verified
 
NN = 0.001; % NN depth in m
FF = 0.002; % FF depth in m

duration = time;

xFinalSolution = [];
yFinalSolution = [];
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up function handles                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.func.dynamics = @(t,z,u)(dynamics2(z,u,p));
%problem.func.pathObj = @(t,z,u)(u(1,:).^2 + u(2,:).^2); % cost function
problem.func.pathObj = @(t,z,u)(z(1,:).^2 + z(2,:).^2);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up problem bounds                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = duration;
problem.bounds.finalTime.upp = duration;

problem.bounds.state.low = [-0.010;-0.010;0;-0.0010;-0.0010;-pi/6]; %State is x,y,q,dx,dy,dq
problem.bounds.state.upp = [0.010;0.010;2*pi;0.010;0.010;pi/6];

% problem.bounds.state.low = [-0.010;-0.010;pi;-0.010;-0.010;-10];
% problem.bounds.state.upp = [0.010;0.010;2*pi;0.010;0.010;10];

problem.bounds.initialState.low = [-(p.w/2)-0.008;0;pi;-10;-10;-10]; %State is x,y,q,dx,dy,dq
problem.bounds.initialState.upp = [-(p.w/2)-0.004;0;3*pi/2;10;10;10];

% problem.bounds.initialState.low = [-0.007;0;pi;0;0;-10]; %State is x,y,q,dx,dy,dq
% problem.bounds.initialState.upp = [-0.007;0;3*pi/2;0;0;10];


% problem.bounds.finalState.low = [-0.00115;0.002;3*pi/2;0;0;0];   % we consider y as positive
% problem.bounds.finalState.upp = [0.00115;0.004;3*pi/2;0;0;0];

problem.bounds.finalState.low = [0;0.002;3*pi/2;-10;0;-10];
problem.bounds.finalState.upp = [0;Depth;3*pi/2;10;0;10];

% problem.bounds.finalState.low = [p.w/2+0.004;0;3*pi/2;0;0;0];   % we consider y as positive
% problem.bounds.finalState.upp = [p.w/2+0.008;0;2*pi;0;0;0];

% problem.bounds.control.low = [-0.1;-0.1;1];
% problem.bounds.control.upp = [0.1;0.1;10];

problem.bounds.control.low = [-inf;-inf;-inf];
problem.bounds.control.upp = [inf;inf;inf];


intermediate_state = [0;0.002;3*pi/2;0;0;0];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Initial guess at trajectory                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.guess.time = [0,duration];
problem.guess.state = [problem.bounds.initialState.low,problem.bounds.finalState.low];
problem.guess.control = [0,0;0,0;0,0];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Solver options                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.options.nlpOpt = optimset(...
    'Display','iter',...
    'MaxFunEvals',1e7);

problem.options.method = 'trapezoid';
problem.options.defaultAccuracy = 'medium';

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Solve!                                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

soln = optimTraj(problem);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Display Solution                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%% Unpack the simulation
t = linspace(soln.grid.time(1), soln.grid.time(end), 150);
z = soln.interp.state(t);
u = soln.interp.control(t);

x = z(1,:);
y = -z(2,:);
q = z(3,:);
dx = z(4,:);
dy = z(5,:);
dq = z(6,:);

Fx = u(1,:);
Fy = u(2,:);
thau = u(3,:);


%plot the solution:

% figure(1); clf;
% subplot(3,1,1);
% plot(t,x)
% ylabel('x')
% title('Optimal Vertical suturing');
% 
% subplot(3,1,2)
% plot(t,y);
% ylabel('y');
% 
% subplot(3,1,3)
% plot(t,q)
% ylabel('q')
% 
figure(1);
subplot(3,1,1);
plot(t,Fx);
ylabel('Fx');
title(['Input values for Trajectory Optimization for r= ' num2str(NeedleRadius)])


subplot(3,1,2);
plot(t,Fy);
ylabel('Fy');

subplot(3,1,3);
plot(t,thau);
ylabel('thau');

FinalFx = [Fx];
FinalFy = [Fy];
FinalThau = [thau];
xFinalSolution = x;
yFinalSolution = y;



%% Second part of the trajectory 

% problem.bounds.state.low = [-0.010;-0.010;0;-0.0010;-0.0010;-pi/6]; %State is x,y,q,dx,dy,dq
% problem.bounds.state.upp = [0.010;0.010;2*pi;0.010;0.010;pi/6];

problem.bounds.initialState.low = [x(end);y(end);q(end);dx(end);dy(end);dq(end)]; %State is x,y,q,dx,dy,dq
problem.bounds.initialState.upp = [x(end);y(end);q(end);dx(end);dy(end);dq(end)];

problem.bounds.finalState.low = [p.w/2+0.004;0;3*pi/2;0;0;0];   % we consider y as positive
problem.bounds.finalState.upp = [p.w/2+0.008;0;2*pi;0;0;0];

problem.bounds.control.low = [-inf;-inf;-inf];
problem.bounds.control.upp = [inf;inf;inf];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Initial guess at trajectory                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.guess.time = [0,duration];
problem.guess.state = [problem.bounds.initialState.low, problem.bounds.finalState.low];
problem.guess.control = [0,0;0,0;0,0];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Solver options                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.options.nlpOpt = optimset(...
    'Display','iter',...
    'MaxFunEvals',1e7);

problem.options.method = 'trapezoid';
problem.options.defaultAccuracy = 'medium';

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Solve!                                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

soln = optimTraj(problem);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Display Solution                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%% Unpack the simulation
t2 = linspace(soln.grid.time(1), soln.grid.time(end), 150);
z = soln.interp.state(t);
u = soln.interp.control(t);

x2 = z(1,:);
y2 = z(2,:);
q2 = z(3,:);
dx2 = z(4,:);
dy2 = z(5,:);
dq2 = z(6,:);

Fx2 = u(1,:);
Fy2 = u(2,:);
thau2 = u(3,:);

figure(2);
subplot(3,1,1);
plot(t,Fx2);
ylabel('Fx');
title(['Input values for Trajectory Optimization for r= ' num2str(NeedleRadius)])


subplot(3,1,2);
plot(t,Fy2);
ylabel('Fy');

subplot(3,1,3);
plot(t,thau2);
ylabel('thau');


xFinalSolution = [xFinalSolution x2];
yFinalSolution = [yFinalSolution y2];


%% Third part of the trajectory 
% 
problem.bounds.state.low = [-0.010;-0.010;0;-0.0010;-0.0010;-pi/6]; %State is x,y,q,dx,dy,dq
problem.bounds.state.upp = [0.010;0.010;2*pi;0.010;0.010;pi/6];

problem.bounds.initialState.low = [p.w/2+0.001;0;0;0;0;-10]; %State is x,y,q,dx,dy,dq
problem.bounds.initialState.upp = [p.w/2+0.004;0;2*pi;0;0;10];

% problem.bounds.initialState.low = [0.003;0;0;0;0;-10]; %State is x,y,q,dx,dy,dq
% problem.bounds.initialState.upp = [0.003;0;3*pi/2;0;0;10];

problem.bounds.finalState.low = [0;0;3*pi/2;-10;0;-10];   % we consider y as positive
problem.bounds.finalState.upp = [0;0.002;3*pi/2;10;0;10];

problem.bounds.control.low = [-inf;-inf;-inf];
problem.bounds.control.upp = [inf;inf;inf];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Initial guess at trajectory                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.guess.time = [0,duration];
problem.guess.state = [problem.bounds.initialState.low, problem.bounds.finalState.low];
problem.guess.control = [0,0;0,0;0,0];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Solver options                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.options.nlpOpt = optimset(...
    'Display','iter',...
    'MaxFunEvals',1e7);

problem.options.method = 'trapezoid';
problem.options.defaultAccuracy = 'medium';

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Solve!                                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

soln = optimTraj(problem);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Display Solution                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%% Unpack the simulation
t3 = linspace(soln.grid.time(1), soln.grid.time(end), 150);
z = soln.interp.state(t);
u = soln.interp.control(t);

x3 = z(1,:);
y3 = -z(2,:);
q3 = z(3,:);
dx3 = z(4,:);
dy3 = z(5,:);
dq3 = z(6,:);

Fx3 = u(1,:);
Fy3 = u(2,:);
thau3 = u(3,:);



xFinalSolution = [xFinalSolution x3];
yFinalSolution = [yFinalSolution y3];

%% Fourth part of the trajectory 
% 
% problem.bounds.state.low = [-0.010;-0.010;0;-0.0010;-0.0010;-pi/6]; %State is x,y,q,dx,dy,dq
% problem.bounds.state.upp = [0.010;0.010;2*pi;0.010;0.010;pi/6];

problem.bounds.initialState.low = [x3(end);y3(end);q3(end);dx3(end);dy3(end);dq3(end)]; %State is x,y,q,dx,dy,dq
problem.bounds.initialState.upp = [x3(end);y3(end);q3(end);dx3(end);dy3(end);dq3(end)];

problem.bounds.finalState.low = [-p.w/2-0.004;0;pi;0;0;0];   % we consider y as positive
problem.bounds.finalState.upp = [-p.w/2-0.001;0;3*pi/2;0;0;0];

problem.bounds.control.low = [-inf;-inf;-inf];
problem.bounds.control.upp = [inf;inf;inf];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Initial guess at trajectory                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.guess.time = [0,duration];
problem.guess.state = [problem.bounds.initialState.low, problem.bounds.finalState.low];
problem.guess.control = [0,0;0,0;0,0];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Solver options                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.options.nlpOpt = optimset(...
    'Display','iter',...
    'MaxFunEvals',1e7);

problem.options.method = 'trapezoid';
problem.options.defaultAccuracy = 'medium';

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Solve!                                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

soln = optimTraj(problem);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Display Solution                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%% Unpack the simulation
t4 = linspace(soln.grid.time(1), soln.grid.time(end), 150);
z = soln.interp.state(t);
u = soln.interp.control(t);

x4 = z(1,:);
y4 = z(2,:);
q4 = z(3,:);
dx4 = z(4,:);
dy4 = z(5,:);
dq4 = z(6,:);

Fx4 = u(1,:);
Fy4 = u(2,:);
thau4 = u(3,:);


xFinalSolution = [xFinalSolution x4];
yFinalSolution = [yFinalSolution y4];
FinalFx = [Fx Fx2 Fx3 Fx4];
FinalFy = [Fy Fy2 Fy3 Fy4];
FinalThau = [thau thau2 thau3 thau4];


t2 = t2 + t(end);
t3 = t3 + t2(end);
t4 = t4 + t3(end);
TotalTime = [t t2 t3 t4];

%% Plot final Solution 
figure(3)
grid on
ylabel('y (m)')
xlabel('x (m)')
title(['Trajectory for optimal vertical suturing (with r= ' num2str(NeedleRadius) ')'])
hold on;
yline(-0.002,'color','black');
yline(0)
xline(0)

woundX1 = -0.010:0.001:0;
woundX2 = 0:0.001:0.010;

wound1 = -Depth - 2*Depth/WoundWidth.*woundX1;
wound2 = -Depth + 2*Depth/WoundWidth.*woundX2;
plot(woundX1,wound1,'color','red','Linewidth',2);
plot(woundX2,wound2,'color','red','Linewidth',2)
ylim([-0.004 0]);

curve = animatedline('LineWidth',2);
set(gca,'XLim',[-0.010 0.010],'YLim',[min(y)-0.001 0]);
%hold on;
for i=1:length(xFinalSolution)
    addpoints(curve,xFinalSolution(i),yFinalSolution(i));
    head = scatter(xFinalSolution(i),yFinalSolution(i),'filled','MarkerFaceColor','green');
    drawnow
    pause(0.005);
    delete(head)
end

hold off;

%% Plot inputs for the trajectory optimization

% figure(2);
% subplot(3,1,1);
% plot(TotalTime,FinalFx);
% ylabel('Fx');
% title(['Input values for Trajectory Optimization for r= ' num2str(NeedleRadius)])
% 
% 
% subplot(3,1,2);
% plot(TotalTime,FinalFy);
% ylabel('Fy');
% 
% subplot(3,1,3);
% plot(TotalTime,FinalThau);
% ylabel('thau');

end

