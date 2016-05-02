function h = stepPlotUnicycle(agentsList,hist,plot_handles,i,formationD)

leaderD = formationD{1};

if not(plot_handles == 0)
    delete(plot_handles)
end

subplot(4,1,[1,2]);

nAgents = length(agentsList);
colors  = hsv(nAgents);

h       = zeros(1,nAgents);
nSpace  = size(leaderD,1);

for k = 1:nAgents
    
    x    = hist{k}.stateTrajectory(:,1:i);
    h(k) = plot(x(1,:),x(2,:),'color', colors(k,:), 'LineWidth', 2);
    hold on;
    
    if k>1 %follower
        xc   = hist{k}.controllerStateTrajectory(:,1:i);
        h(k) = plot(xc(1,:),xc(2,:),'--','color', colors(k,:), 'LineWidth', 2);
    end
    
end

xLeader      = hist{1}.stateTrajectory(1:nSpace,i);

formation    = repmat(xLeader,1,nAgents) - leaderD;
formation    = [formation(:,2:end), formation(:,2)];
h(nAgents+1) = plot(formation(1,:), formation(2,:),'--');

setNicePlot
subplot(4,1,3)

time =  hist{1}.time(1:i);
for k = 2:nAgents
    
    % xk-xi-formationD{k}(:,i)
    
    xk = hist{1}.stateTrajectory(1:nSpace,1:i);
    xi = hist{k}.controllerStateTrajectory(1:nSpace,1:i);
    
    err  = xk-xi-repmat(leaderD(:,k),1,i);
    nerr = sqrt(sum(err.*err));
    h(k) = plot(time,nerr,'color', colors(k,:), 'LineWidth', 2);
    hold on;
    
end
title('Formation error');
xlabel('time [s]')
setNicePlot

subplot(4,1,4)

time =  hist{1}.time(1:i);

for k = 2:nAgents
    
    % xk-xi-formationD{k}(:,i)
    
    pd = hist{k}.controllerStateTrajectory(1:nSpace,1:i);
    x  = hist{k}.stateTrajectory(:,1:i);
    e = zeros(2,size(x,2));
    for j = 1:size(x,2)
        e(:,j) = agentsList{2}.controller.trackingControlLaw.computeErrorFromPd(pd(:,j),x(:,j));
    end
   
    nerr = sqrt(sum(e.*e));
    h(k) = plot(time,nerr,'color', colors(k,:), 'LineWidth', 2);
    hold on;
    
end
title('Tracking error');
xlabel('time [s]')
setNicePlot


end
