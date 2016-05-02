%% Formation control of unicycle systems
% Written by Andrea Alessandretti 02-05-16
clc; clear all; close all;

addpath('./lib');

N  = 4;    % number of agents (leader included)
dt = 0.01; % discretization step

%% Formation - Desired displacement vectors
% formationD{i}(:,j) = position agent i - position agent j
formationD = build_dist(N, 2);

%% Communication setup
% The Nth vehicle (with N>2) senses the state of the controller of the 
% and the velocity of the state of the controller of the N-1th vehicle.

% Adjacency matrix - defines the graph of agents 
% A(i,j) = 1 if agent_i senses agent_j, A(i,j) = 0 otherwise
A1            = zeros(N);
A1(2:N,1:N-1) = eye(N-1);
A1(2,1)       = 0;

% Sensors
s1            = AgentSensor(@(agent)agent.controller.x);
s1v           = VelocityAgentSensor(dt,@(agent)agent.controller.x);

% Note: VelocityAgentSensor approximates the velocity using two consecutive
%       measurements. Therefore, the smaller dt is, the better such estimate
%       (and the performance of the controller) gets.

% The second vehicle senses position and velocity of the leader (vehicle 1)

% Adjacency matrix
A2            = zeros(N);
A2(2,1)       = 1;

% Sensosr
s2            = AgentSensor(@(agent)agent.x);
s2v           = VelocityAgentSensor(dt,@(agent)agent.x);

A = A1 + A2;

%% Create Systems 

% Initial Conditions
x0Vehicles = [0 -2 -5 -7;
    0 -7  2 -5;
    0  0  0  0];

%Leader
v{1} = Unicycle('InitialCondition',x0Vehicles(:,1));

% The leader moves independently following the trajectory
% @(t)10*[0.1*t;cos(0.5*t)]
v{1}.controller =  TrackingControllerECC13(...
    'Vehicle', v{1} ,...
    'Epsilon', -0.1*[1;0],...
    'pd'     , @(t)3*[0.1*t;cos(0.5*t)],...
    'dotPd'  , @(t)3*[0.1;-0.5*sin(0.5*t)],...
    'Ke'     , 1*eye(2)...
    );

% Followers
for ii = 2:N
    
    v{ii} = Unicycle('InitialCondition',x0Vehicles(:,ii));
    
    v{ii}.controller = VirtualVehicleFormationController(...
        TrackingControllerECC13(...
            'Vehicle',v{ii} ,...
            'Epsilon',-[0.1;0],...
            'Ke'     ,1*eye(2),...
            'pd',@(t)0,'dotPd',@(t)0 ... %fake desired trajectory
        ),...
        FormationControllerSingleIntegrators(...
            formationD{ii}(:,logical(A(ii,:))),...
            dt...
        ));
    
    v{ii}.controller.initialCondition = x0Vehicles(1:2,ii);
    
end

va = VirtualArena(v,...
    'StoppingCriteria'  , @(t,as)t>30,...
    'StepPlotFunction'  , @(systemsList,log,oldHandles,k) stepPlotUnicycle(systemsList,log,oldHandles,k,formationD), ...
    'SensorsNetwork'    , {s1,A1,s2,A2,s1v,A1,s2v,A2},...
    'DiscretizationStep', dt,...
    'PlottingStep'      , 1);

ret = va.run();

