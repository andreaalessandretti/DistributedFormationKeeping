classdef VirtualVehicleFormationController < CtSystem
    %% VirtualVehicleFormationController
    % A virtual vehicle approach to distributed control for formation keeping of underactuated vehicles
    % For more info see
    % Roth, Y., Alessandretti, A., Aguiar, A. P., & Jones, C. N. (2015).
    % A virtual vehicle approach to distributed control for formation keeping
    % of underactuated vehicles. In 1th Indian Control Conf. (ICC 2015), Chennai, India.
    %
    % VirtualVehicleFormationController Methods:
    %    VirtualVehicleFormationController - constructor
    
    properties
        trackingControlLaw;
        controllerVirtualVehicle;
        sensorsToUse;
    end
    
    methods
        
        function obj = VirtualVehicleFormationController(trackingControlLaw,controllerVirtualVehicle,sensorsToUse)
        %% VirtualVehicleFormationController(trackingControlLaw,controllerVirtualVehicle,sensorsToUse)
        %
        % trackingControlLaw : Controller
        %   Controller able to drive the position of an 'Unicycle' 'CtSystem'
        %   to follow a given trajectory. The trajectoy needs to be specified 
        %   in the two public properties of the controller
        %   - pd     (function handles @(time)-> desired position)
        %   - dotPd  (function handles @(time)-> desired velocity)
        %
        % controllerVirtualVehicle : Controller
        %   Distributed Controller able to drive the position of single
        %   integrators in the desired formation using two sensors, the first 
        %   for position and the second for velocity of the neighbours.
        %
        % sensorsToUse : binary array
        %   Select the two sensors to use among the ones available. E.g., if
        %   the system is using 4 sensors and the one providing the position
        %   and velocity required by controllerVirtualVehicle are the 2nd and
        %   the 3rd, then set sensorsToUse =[2,3].
        
            obj = obj@CtSystem('nx',2,'nu',2,'ny',2);
            
            obj.trackingControlLaw       = trackingControlLaw;
            obj.controllerVirtualVehicle = controllerVirtualVehicle;
            obj.sensorsToUse             = sensorsToUse;
        end
        
        function xDot = f(obj,t,xc,uSysCon,x,readings,varargin)
            
            readings = readings(obj.sensorsToUse);
            xDot     = obj.controllerVirtualVehicle.computeInput(t,xc,readings);
            
        end
        
        function y = h(obj,t,xc,x,readings,varargin)
            
            readings = readings(obj.sensorsToUse);
            
            uvv = obj.controllerVirtualVehicle.computeInput(t,xc,readings);
            
            obj.trackingControlLaw.pd    = @(t)xc;
            
            obj.trackingControlLaw.dotPd = @(t)uvv;
            
            y = obj.trackingControlLaw.computeInput(t,x);
            
        end
        
    end
    
end