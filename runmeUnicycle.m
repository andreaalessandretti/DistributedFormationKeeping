%% A virtual vehicle approach to distributed control for formation keeping of underactuated vehicles
% For more info see
% Roth, Y., Alessandretti, A., Aguiar, A. P., & Jones, C. N. (2015). 
% A virtual vehicle approach to distributed control for formation keeping 
% of underactuated vehicles. In 1th Indian Control Conf. (ICC 2015), Chennai, India.

clc; clear all; close all;

addpath('./lib');

N  = 4;    % Number of agents (leader included)
dt = 0.01; % discretization step

% Note: VelocityAgentSensor approximates the velocity using two consecutive
%       measurements. Therefore, the smaller dt is, the better such estimate
%       (and the performance of the controller) gets.

%% Formation - Desired displacement vectors
% formationD{i}(:,j) = position agent i - position agent j
formationD = build_dist(N, 2);

%% Leader
v{1} = Unicycle();

% The leader moves independently following the trajectory
% @(t)10*[0.1*t;cos(0.5*t)]
v{1}.controller =  TrackingControllerECC13(...
    'Vehicle', v{1} ,...
    'Epsilon', -0.1*[1;0],...
    'pd'     , @(t)3*[0.1*t;cos(0.5*t)],...
    'dotPd'  , @(t)3*[0.1;-0.5*sin(0.5*t)],...
    'Ke'     , 1*eye(2)...
    );

%% Followers (with N>2)
% The Nth vehicle (with N>2) senses value and velocity
% of the state of the controller of the N-1th vehicle.

% Sensors
s1  = IAgentSensor(@(t,agentId,agent,sensedAgentId,sensedAgent)sensedAgent.controller.x); 
s1v = VelocityAgentSensor(dt,@(agent)agent.controller.x); 

% Adjacency matrix - defines the graph of agents 
% A(i,j) = 1 if agent_i senses agent_j, A(i,j) = 0 otherwise
A1 =  [0     0     0     0
       0     0     0     0
       0     1     0     0
       0     0     1     0];

%% Followers (with N=2)
% The second vehicle senses position and velocity of the leader (vehicle 1)

% Sensosr
s2  = IAgentSensor(@(t,agentId,agent,sensedAgentId,sensedAgent)sensedAgent.x); 
s2v = VelocityAgentSensor(dt, @(agent)agent.x);

% Adjacency matrix
A2 =[ 0     0     0     0
      1     0     0     0
      0     0     0     0
      0     0     0     0];

%% Create Systems 

% Initial Conditions
x0Vehicles = [0 -2 -5 -7;
              0 -7  2 -5;
              0  0  0  0];
          
v{1}.initialCondition = x0Vehicles(:,1);

for ii = 2:N
    
    v{ii} = Unicycle('InitialCondition',x0Vehicles(:,ii));
    
    d = formationD{ii}(:,logical(A1(ii,:) +A2(ii,:)));
    
    if ii == 2
        sensorsToUse = [3,4]; % agent 2 measures the state of the leader
    else
        sensorsToUse = [1,2]; % the others measure the state of the controllers
    end
    
    v{ii}.controller = VirtualVehicleFormationController(...
        TrackingControllerECC13(...
            'Vehicle',v{ii} ,...
            'Epsilon',-[0.1;0],...
            'Ke'     ,1*eye(2),...
            'pd',@(t)0,'dotPd',@(t)0 ... %fake desired trajectory
        ),...
        FormationControllerSingleIntegrators(d),...
        sensorsToUse ...
        );
    
        v{ii}.controller.initialCondition = x0Vehicles(1:2,ii);
    
end

va = VirtualArena(v,...
    'StoppingCriteria'  , @(t,as)t>30,...
    'StepPlotFunction'  , @(systemsList,log,oldHandles,k) stepPlotUnicycle(systemsList,log,oldHandles,k,formationD), ...
    'SensorsNetwork'    , {s1,@(t)A1,s1v,@(t)A1,s2,@(t)A2,s2v,@(t)A2},...
    'DiscretizationStep', dt,...
    'PlottingStep'      , 1);

ret = va.run();

