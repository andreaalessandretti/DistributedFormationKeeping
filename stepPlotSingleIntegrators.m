function h = stepPlotSingleIntegrators(agentsList,hist,plot_handles,i,formationD)

leaderD = formationD{1};

if not(plot_handles == 0)
    delete(plot_handles)
end

subplot(3,1,[1,2])

nAgents = length(agentsList);
colors  = hsv(nAgents);
h       = zeros(1,nAgents);
nSpace  = size(leaderD,1);

for k = 1:nAgents
    
    x    = hist{k}.stateTrajectory(:,1:i);
    h(k) = plot(x(1,:),x(2,:),'color', colors(k,:), 'LineWidth', 2);
    hold on;
    
end

xLeader      = hist{1}.stateTrajectory(1:nSpace,i);
formation    = repmat(xLeader,1,nAgents) - leaderD;
formation    = [formation(:,2:end), formation(:,2)];
h(nAgents+1) = plot(formation(1,:), formation(2,:),'--');

setNicePlot
subplot(3,1,3)

time =  hist{1}.time(1:i);
for k = 1:nAgents
    
    % xk-xi-formationD{k}(:,i)
    
    xk = hist{1}.stateTrajectory(1:nSpace,1:i);
    xi = hist{k}.stateTrajectory(1:nSpace,1:i);
    
    err  = xk-xi-repmat(leaderD(:,k),1,i);
    nerr = sqrt(sum(err.*err));
    h(k) = plot(time,nerr,'color', colors(k,:), 'LineWidth', 2);
    hold on;
    
end
title('Formation error');
xlabel('time [s]')
setNicePlot


end
