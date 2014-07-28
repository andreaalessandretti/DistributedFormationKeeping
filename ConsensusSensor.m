
classdef ConsensusSensor < Sensor
    %ConsensusSensor reads the internal state of the controllers of the neighbor vehicles
    %   which corresponds with the position of the virtual vehicles
    %
    %   See also ConsensusController, Sensor
    
    properties
    end
    
    methods (Static)
        function  ret = sense(agentId,agent,detectableAgentsList,detectableAgentsIds)
            ret = {};
            nDetectables = length(detectableAgentsIds);
            
            for i = 1 : nDetectables
                
                ret{i} = detectableAgentsList{i}.controller.x;

            end 
        end
        
 
    end
    
end

