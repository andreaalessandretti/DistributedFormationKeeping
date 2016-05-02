classdef VirtualVehicleFormationController < CtSystem
%% VirtualVehicleFormationController(trackingControlLaw,controllerVirtualVehicle)

% Written by Andrea Alessandretti 02-05-16

    properties
        trackingControlLaw;
        controllerVirtualVehicle;
    end
    
    methods
        
        function obj = VirtualVehicleFormationController(trackingControlLaw,controllerVirtualVehicle)
            
            obj = obj@CtSystem('nx',2,'nu',2,'ny',2);
            
            obj.trackingControlLaw       = trackingControlLaw;
            obj.controllerVirtualVehicle = controllerVirtualVehicle;
            
            obj.f = @(t,xc,x,readings)obj.controllerVirtualVehicle.computeInput(t,xc,readings);
            obj.h = @(t,xc,x,readings)obj.computeInputForUnicycle(t,xc,x,readings);
            
        end
        
        function u = computeInputForUnicycle(obj,t,xc,x,readings)
            
            uvv = obj.controllerVirtualVehicle.computeInput(t,xc,readings);
       
            obj.trackingControlLaw.pd    = @(t)xc;
            obj.trackingControlLaw.dotPd = @(t)uvv;
            
            u = obj.trackingControlLaw.computeInput(t,x);
            
        end
    end
    
end