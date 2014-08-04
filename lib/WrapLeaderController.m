classdef WrapLeaderController < DtSystem
	%WrapLeaderController for ConsensusSensor 
    %   extends a given control law with an internal state containing the
    %   current position of the vehicle, which is sent trough the Sensor 
    %   ConsensusSensor to the neighbor vehicles.
    %   
    %   Costructor:
    %
    %   c = WrapLeaderController(controller)
    %
    %   See also ConsensusSensor, DtSystem
    properties
        controlLaw
    end
    
    methods
        function obj = WrapLeaderController(controller,varargin)
            
            n = controller.vehicle.n;
            
            if n == 2
                nx=2; nu=3; ny=2;
            else
                nx=3; nu=12; ny=3;  
            end
            
            obj = obj@DtSystem(...
                    'StateEquation',@(t,x,u,net)u(1:n),...
                    'nx',nx,'nu',nu,'ny',ny,varargin{:});
           
            obj.controlLaw = controller;
            
            obj.h = @(t,xc,x,net)obj.controlLaw.computeInput(t,x);
            
        end
   
        
    end
    
end

