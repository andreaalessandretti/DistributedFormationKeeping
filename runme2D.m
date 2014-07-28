% For more info see
%
% Paper:      "A virtual vehicle approach to distributed control for formation keeping of underactuated vehicles"
% Conference: Indian Control Conference (ICC 2015), Chennai, India

clc;clear all;close all;

N = 4; % number of agents (leader included)
d = 2; % 2-D case

%dt = 0.001; % Precise but slow
dt = 0.05;  % Less precise due to bad velocity estimation,  but faster

% Formation
formationD = build_dist(N, 2);  

% Adjacency matrix - chain
A = zeros(N);
A(2:N,1:N-1)= eye(N-1);


%% Leader
v{1} = Unicycle('InitialConditions',[0; 0; 0]); 

lc = TrackingControllerECC14(...
    'Vehicle',v{1} ,... 
    'Epsilon',-0.1*[1;0],...
    'pd'     ,   @(t)10*[0.1*t;cos(0.5*t)],...
    'dotPd'  ,@(t)10*[0.1;-0.5*sin(0.5*t)],...
    'Ke'     ,0.1*eye(2)...
    );

v{1}.controller = WrapLeaderController(lc,'InitialConditions',v{1}.initialConditions(1:2));

%% Followers
x0Vehicles = [0 -2 -5 -7; 
              0 -7  2 -5; 
              0  0  0 0 ];

for i = 2:N
    v{i} = Unicycle();
    
    trackingLaw = TrackingControllerECC14(...
        'Vehicle',v{i} ,... 
        'Epsilon',-0.1*[1;0],... 
        'Ke'     ,0.1*eye(2)...
    );

    v{i}.controller = ConsensusController(formationD{i}, A(i,:), dt,trackingLaw);
    v{i}.controller.initialConditions = x0Vehicles(1:v{i}.n,i);
    v{i}.initialConditions = x0Vehicles(:,i);
end

%% Communication
s1 = ConsensusSensor();

%% VirtualArena
a = VirtualArena(v,...
    'StoppingCriteria'  ,@(i,as)i>50/dt,...
    'StepPlotFunction'  ,@(systemsList,log,oldHandles,k) consensusStepPlotFunction(systemsList,log,oldHandles,k,formationD{1},d,dt), ...
    'StopPlotFunction'  ,@(allLogs,va)consensusStopPlotFunction(allLogs,va,formationD{1},d),...
    'SensorsNetwork'    , {s1,A},...
    'DiscretizationStep',dt,...
    'PlottingFrequency' ,1/dt);

ret = a.run();

save  simData.mat
