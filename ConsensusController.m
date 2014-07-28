classdef ConsensusController < CtSystem
    %%ConsensusController
    %
    % ConsensusController(d, Agraph, communicationDt,trackingLaw)
    %
    %   d(:,i)          - desired offset, x - x_i, with vehicle i
    %   Agraph          - Row of the adjacency matrix associated with this vehicle
    %   communicationDt - time interval between communications used to
    %                     estimate the velocity of the neighbor vehicles
    %   trackingLaw     - controller used to track the virtual vehicle
    %
    
    properties
        lastReadings;
        Agraph;        
        communicationDt;
        d;
        trackingLaw;
        
    end
    
    methods
        
        
        function obj = ConsensusController(d, Agraph, communicationDt,trackingLaw)
            
            if trackingLaw.vehicle.n ==2
                nx=2;nu=2;ny=2;
            else
                nx=3;nu=12;ny=2;
            end
            
            obj = obj@CtSystem('nx',nx,'nu',nu,'ny',ny);
            
            obj.trackingLaw  = trackingLaw;
            obj.f = @(t,x,u,readings)obj.xcDot(x,readings);
            obj.h = @(t,x,z,readings)obj.computeInput(x,z,readings);
            obj.d = d;
            obj.Agraph = Agraph;
            obj.communicationDt = communicationDt;
            
        end
        
        
        function xcDot = xcDot(obj,xc,readings)
           
            xcDot = ConsensusController.computeInputVirtualVehicle(xc,readings,obj.lastReadings,obj.communicationDt,obj.Agraph,obj.d,obj.trackingLaw.vehicle.n);
           
        end
        
        function u = computeInput(obj,xc,x,readings)
           
             u2 = ConsensusController.computeInputVirtualVehicle(xc,readings,obj.lastReadings,obj.communicationDt,obj.Agraph,obj.d,obj.trackingLaw.vehicle.n);
             
             obj.trackingLaw.pd    = @(t)xc;
             obj.trackingLaw.dotPd = @(t)u2; 
             u = obj.trackingLaw.computeInput(0,x);
             obj.lastReadings = readings;
              
        end
    end
    
    methods (Static)
        function u2 = computeInputVirtualVehicle(xc,readings,lastReadings,communicationDt,Agraph,d,nSpace)
            
            
            n_detect = nnz(Agraph);
            nnz_A = find(Agraph);
            eta = sum(Agraph);
            gamma = 1;
            
            
            u2 = zeros(nSpace,1);
            
            for i=1:n_detect
                
                if isempty(lastReadings)
                    vi = zeros(size( readings{1}{i}));
                else
                    vi = (readings{1}{i}-lastReadings{1}{i})/communicationDt;
                end
                
                dx = readings{1}{i}-xc;
                
                u2 = u2 + 1/eta*Agraph(nnz_A(i))*(vi+gamma*(dx+d(:, nnz_A(i))) );
            end
             
            
        end
        
        
    end
    
end