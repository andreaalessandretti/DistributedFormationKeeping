%% Formation control of single integrator systems
% Written by Andrea Alessandretti 02-05-16
clc; clear all; close all;

addpath('./lib');

N  = 4;     % number of agents (leader included)
dt = 0.01;  % discretization step

%% Formation - Desired displacement vectors 
% formationD{i}(:,j) = position_agent_i - position_agent_j
formationD = build_dist(N, 2);

%% Communication
% We are assuming that every agent senses the position and the velocity 
% of its neighboring agents.

% Sensors
s1  = AgentSensor(@(agent)agent.x);             %Position (state x) sensor)
s1v = VelocityAgentSensor(dt,@(agent)agent.x);  %Velocity sensor)

% Note: VelocityAgentSensor approximates the velocity using two consecutive
%       measurements. Therefore, the smaller dt is, the better such estimate
%       (and the performance of the controller) gets.

% Adjacency matrix - defines the graph of agents 
% A(i,j) = 1 if agent_i senses agent_j, A(i,j) = 0 otherwise
A            = zeros(N);
A(2:N,1:N-1) = eye(N-1);

%% Create Systems 

% Initial Conditions
x0systems = [0 -2 -5 -7;
               0 -7  2 -5 ];
%Leader
v{1} = CtSystem('StateEquation',@(t,x,u)u,'nx',2,'nu',2);

v{1}.controller       = InlineController(@(t,x)[sin(0.1*t);cos(0.1*t)]);
v{1}.initialCondition = x0systems(:,1);

%Followers
for ii = 2:N
    
    v{ii} = CtSystem('StateEquation',@(t,x,u)u,'nx',2,'nu',2,'InitialCondition',x0systems(:,ii));
    
    v{ii}.controller = FormationControllerSingleIntegrators(formationD{ii}(:,logical(A(ii,:))), dt);
    
end

va = VirtualArena(v,...
    'StoppingCriteria'  , @(t,as)t>30,...
    'StepPlotFunction'  , @(systemsList,log,oldHandles,k) stepPlotSingleIntegrators(systemsList,log,oldHandles,k,formationD), ...
    'SensorsNetwork'    , {s1,A,s1v,A},...
    'DiscretizationStep', dt,...
    'PlottingStep'      , 1);

ret = va.run();

