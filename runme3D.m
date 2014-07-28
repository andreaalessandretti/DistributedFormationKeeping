% For more info see
% 
% "A virtual vehicle approach to distributed control for formation keeping of underactuated vehicles"
% Indian Control Conference (ICC 2015), Chennai, India

clc;clear all;close all;

N = 4;  % number of agents (leader included)
d = 3; % 3-D case

%dt = 0.001; % Precise but slow
dt = 0.01;  % Less precise due to bad velocity estimation,  but faster

% Formation
formationD = build_dist3D(N, 2);

% Adjacency matrix - chain
A = zeros(N);
A(2:N,1:N-1)= eye(N-1);


%% Leader
v{1} = UAV(...
    'AttitudeRepresentation','Quaternion',...
    'InitialConditions',[-3*ones(3,1);quaternion.theta2quat(zeros(3,1))]...
    );

con = TrackingControllerECC14(...
    'Vehicle',v{1} ,... 
    'Epsilon',-0.1*[1;0;1],... 
    'pd'     ,@(t)10*[cos(0.1*t);sin(0.1*t);0.1*t],...
    'dotPd'  ,@(t)10*[-0.1*sin(0.1*t);0.1*cos(0.1*t);0.1],...
    'Ke'     ,0.1*eye(3)...
    );

v{1}.controller = WrapLeaderController(con,'InitialConditions', -3*ones(3,1));

%% Followers
initPos = [ 0 ,5,-7, 6;
           -1, 9,-3,-7;
           -3,-4,-7,-6];
R = eye(3);

for i = 2:N

    v{i} = UAV();
    
    trackingLaw = TrackingControllerECC14(...
    'Vehicle',v{i} ,... 
    'Epsilon',-0.1*[1;0;1],... 
    'Ke'     ,0.1*eye(3)...
    );

    v{i}.controller = ConsensusController(formationD{i}, A(i,:), dt,trackingLaw);
    v{i}.controller.initialConditions = initPos(:,i);
    v{i}.initialConditions = [initPos(:,i);R(:)];
    
end

%% Communication
s = ConsensusSensor();

%% VirtualArena

a = VirtualArena(v,...
    'StoppingCriteria'  ,@(i,as)i>100/dt,...
    'StepPlotFunction'  ,@(systemsList,log,oldHandles,k)consensusStepPlotFunction(systemsList,log,oldHandles,k,formationD{1},d,dt), ...
    'StopPlotFunction'  ,@(allLogs,va)consensusStopPlotFunction(allLogs,va,formationD{1},d),...
    'SensorsNetwork'    , {s,A},...
    'DiscretizationStep',dt,...
    'PlottingFrequency' ,1/dt);

ret = a.run();
save sim.mat