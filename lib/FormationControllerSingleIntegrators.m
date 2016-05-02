classdef FormationControllerSingleIntegrators < Controller
%% FormationControllerSingleIntegrators(d,dt)aw,controllerVirtualVehicle)

% Written by Andrea Alessandretti 02-05-16

    properties
        lastNetworkReadings;
        lastNetworkReadingsT;
        d;
        dt;
    end
    
    methods
        
        function obj = FormationControllerSingleIntegrators(d,dt)
            obj.d  = d;
            obj.dt = dt;
        end
        
        function u = computeInput(obj,t,x,readingsSensors)
            
            readingsPos = {};
            readingsVel = {};
            nVehicles = length(readingsSensors)/2;
            
            for ii=1:nVehicles
                readingsPos = {readingsPos{:},readingsSensors{ii}{:}};
                readingsVel = {readingsVel{:},readingsSensors{ii+nVehicles}{:}};
            end
            
            d            = obj.d;
            nu           = length(d);
            dt           = obj.dt;
            n_detect     = length(readingsPos);
            lastReadings = obj.lastNetworkReadings;
            eta          = 1/n_detect;
            gamma        = 1;
            
            u = zeros(nu,1);
            
            for i = 1:n_detect
                
                vi = readingsVel{i}(1:nu);
                dx = x-readingsPos{i}(1:nu);
                u  = u + 1/eta*(vi-gamma*(dx-d(:,i)));
                
            end
            
        end
    end
    
end