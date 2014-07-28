function h = consensusStopPlotFunction(allLogs,va,leaderD,d)

fh = figure;

i = size(allLogs{1}.stateTrajectory,2)-1;


nAgents = length(allLogs);
colors  = distinguishable_colors(nAgents+1);
h       = zeros(1,nAgents);



for k = 1:nAgents
    
    x = allLogs{k}.stateTrajectory(:,1:i);
    
    if d ==2
        h(2*(k-1)+1) = plot(x(1,:),x(2,:),'color', colors(k,:), 'LineWidth', 2);
    else
        h(2*(k-1)+1) = plot3(x(1,:),x(2,:),x(3,:),'color', colors(k,:), 'LineWidth', 2);
    end
    
    hold on;
    
end

h(2*nAgents+1)  = plotFormation(allLogs,nAgents,i,leaderD,d);

title('Position of the vehicles')

setNicePlot

formationTimes = length(1:5/va.discretizationStep:i);

for k=1:5/va.discretizationStep:i
    
    h(2*nAgents+1)  = plotFormation(allLogs,nAgents,k,leaderD,d);
    
end

setNicePlot

set(fh, 'Position', [100, 100, 400, 300]);

fh = figure;

pErrorAll = zeros(nAgents-1,i);

for k = 2:nAgents
    pError           = allLogs{k}.stateTrajectory(1:d,1:i)-allLogs{1}.stateTrajectory(1:d,1:i)+repmat(leaderD(:,k),1,i);
    pError           = sum(pError.^2,1).^(1/2);
    h(2*nAgents+k)   = plot(va.discretizationStep*(1:i),pError,'color', colors(k,:), 'LineWidth', 2); hold on
    pErrorAll(k-1,:) = pError;
end

title('Position errors of the vehicles formation')
xlabel('time [s]')
ylabel('Formation error [m]')


setNicePlot
grid on

createMagnifier(va.discretizationStep*(1:i),pErrorAll,[46.5,0.1,3,0.6],[0.45,0.45,0.4,0.4],colors);

set(fh, 'Position', [100, 100, 400, 300]);

fh = figure;
%% Input signals
for j = 1:2
    
    subplot(2,1,j);
    
    for k = 2:nAgents
        u = allLogs{k}.inputTrajectory(j,1:i);
        h(nAgents+1+formationTimes+(nAgents-1)+(j-1)*(nAgents-1)+(k-1)) = ...
            plot(va.discretizationStep*(1:i),u,'color', colors(k,:), 'LineWidth', 2); hold on
        
    end
end


subplot(2,1,1);

title('Control inputs Unicycle');

xlabel('time [s]')
ylabel('v_f [m/s]')
a = axis;
a(3:4)= [0,10];
axis(a);
setNicePlot

subplot(2,1,2);
xlabel('time [s]')
ylabel('\omega [rad/s]')
a = axis;
a(3:4)= [-3,3];
axis(a);
setNicePlot


set(fh, 'Position', [100, 100, 400, 300]);



end

function h = plotFormation(allLogs,nAgents,k,leaderD,d)

lk = allLogs{1}.controllerStateTrajectory(:,k);
formation = repmat(lk,1,nAgents)-leaderD;
formation = [formation(:,2:end),formation(:,2)];
if d==2
    h =  plot(formation(1,:),formation(2,:),'--');
else
    h =  plot3(formation(1,:),formation(2,:),formation(3,:),'--');
end



end


function createMagnifier(x,y,window,windowLent,colors)


x_r = window(1); y_r =  window(2); w_r =  window(3); h_r =  window(4);
rectangle('Position', [x_r-w_r/2, y_r-h_r/2, w_r, h_r], ...
    'EdgeColor', [0.4, 0.1, 0.4]);

x_a = windowLent(1); y_a = windowLent(2); w_a = windowLent(3); h_a = windowLent(4);

ax = axes('Units', 'Normalized', ...
    'Position', [x_a, y_a, w_a, h_a], ...
    ...%'XTick', [], ...
    ...%'YTick', [], ...
    'Box', 'on', ...
    ...%'LineWidth', 2, ...
    'Color', [0.95, 0.99, 0.95]);

hold on;

for i  =1:size(y,1)
    plot(x, y(i,:),'color', colors(i+1,:), 'LineWidth', 2); hold on
end


axis([x_r-w_r/2, x_r+w_r/2, y_r-h_r/2, y_r+h_r/2]);

setNicePlot
end
