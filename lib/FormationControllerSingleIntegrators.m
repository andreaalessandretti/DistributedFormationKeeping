classdef FormationControllerSingleIntegrators < Controller
%% FormationControllerSingleIntegrators
% Implements the control law
%
% u  = sum_{i=1}^n_neighbours  eta*(vi-gamma*(dx-d(:,i)));
%
% with 
%
% vi = readingsSensors{2}{i}(1:nu);   %readingsSensors{2} = velocity sensor
% dx = x-readingsSensors{1}{i}(1:nu); %readingsSensors{1} = position sensor
% eta = 1/n_neighbours 
% gamma = 1.
%
% FormationControllerSingleIntegrators Methods:
%    FormationControllerSingleIntegrators(d) - constructor
%
    properties
        d;
    end
    
    methods
        
        function obj = FormationControllerSingleIntegrators(d)
            obj.d  = d;
        end
        
        function u = computeInput(obj,t,x,readingsSensors)
           
            nu           = length(obj.d);
            n_detect     = length(readingsSensors{1});
            eta          = 1/n_detect;
            gamma        = 1;
            
            u = zeros(nu,1);
            
            for i = 1:n_detect
                
                vi = readingsSensors{2}{i}(1:nu);   %readingsSensors{2} = velocity sensor
                dx = x-readingsSensors{1}{i}(1:nu); %readingsSensors{1} = position sensor
                u  = u + 1/eta*(vi-gamma*(dx-obj.d(:,i)));
                
            end
            
        end
    end
    
end