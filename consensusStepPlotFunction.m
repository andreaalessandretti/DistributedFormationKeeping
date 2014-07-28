function h = consensusStepPlotFunction(agentsList,hist,plot_handles,i,leaderD,d,dt)

    if not(plot_handles == 0)
      delete(plot_handles)
    end
    
    subplot(3,1,[1,2]);
    
    xlabel('x')
    ylabel('y')
    if d==3
        zlabel('z')
    end
    
    nAgents = length(agentsList);
    colors  = distinguishable_colors(nAgents);
    h       = zeros(1,nAgents);
    
    for k = 1:nAgents
        
        x = hist{k}.stateTrajectory(:,1:i);
        
        
        if d==2
            h(k) = plot(x(1,:),x(2,:),'color', colors(k,:), 'LineWidth', 2);
        else
            h(k) = plot3(x(1,:),x(2,:),x(3,:),'color', colors(k,:), 'LineWidth', 2);
        end
                   
        hold on;
        
    end
    
    li = hist{1}.controllerStateTrajectory(:,i);
    
    formation = repmat(li,1,nAgents)-leaderD;
    formation = [formation(:,2:end),formation(:,2)];
    if d==2
        h(nAgents+1) =  plot(formation(1,:),formation(2,:),'--');
    else
        h(nAgents+1) =  plot3(formation(1,:),formation(2,:),formation(3,:),'--');
    end

    setNicePlot
    
        subplot(3,1,3);
        
        ymin = 0;
         
        for k = 2:nAgents
            
            pError           = hist{k}.controllerStateTrajectory(:,1:i)-hist{1}.stateTrajectory(1:d,1:i)+repmat(leaderD(:,k),1,i);
            pError           = sum(pError.^2,1);
            h(nAgents+1+k-1) = plot(dt*(1:i),pError); hold on 
            ymin             = max(pError(end),ymin);
            
        end
        
        title('Consensus Error');
        
        a = [0,dt*i,0,2*ymin]; axis(a);
        
        setNicePlot 

end
