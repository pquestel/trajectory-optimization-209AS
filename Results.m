%% Script allowing to visualize results of the Trajectory Optimization

% Demo day: allow everyone to change the parameters and help make the
% trajectoy fail ! 

% Parameters that can be changed: 
%       - Wound width  
%       - Wound depth (min depth: 2mm; max depth: 8mm)
%       - Needle radius (have fun !)
%       - Max duration of the process (have fun!)
%       - Cost function (be inventive)
%       - Boundary Conditions 

r = 0.030; % in meters 
d = 0.004;
w = 0.005;  %max width = 1cm (if higher --> change state boundaries !)
t = 0.8; % in seconds 

% for r=0.001:0.001:0.008
%     DemoDay(r,w,d,t)
% end

%DemoDay(r,w,d,t)
RobotSuturing(r,w,d,t)