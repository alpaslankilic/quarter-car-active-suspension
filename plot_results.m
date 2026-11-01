function plot_results(t, road, response, names, p, visible)
%PLOT_RESULTS Show comfort, mechanical limits and actuator effort separately.
colors = [.4 .4 .4; .80 .32 .05; 0 .36 .60];
styles = {':','--','-'};
[fig,layout] = new_figure('Body acceleration after a road bump',2,visible);
ax = nexttile(layout);
plot(ax,t,1e3*road,'Color',[.45 .2 .55],'LineWidth',1.8);
events(ax,p);
ylabel(ax,'Road height [mm]');
ylim(ax,[0,1.3e3*p.bumpHeight]);
title(ax,'(a) Road disturbance','FontWeight','normal');
style(ax,t);
ax = nexttile(layout);
draw_cases(ax,t,response,1,1,colors,styles,names);
events(ax,p);
ylabel(ax,'Body acceleration [m/s^2]');
xlabel(ax,'Time [s]');
title(ax,'(b) Passenger-compartment vertical response','FontWeight','normal');
legend(ax,'Location','southoutside','Orientation','horizontal','Box','off');
style(ax,t);
save_figure(fig,'01_body_response',visible);

[fig,layout] = new_figure('Suspension and road holding',2,visible);
ax = nexttile(layout);
draw_cases(ax,t,response,2,1e3,colors,styles,names);
events(ax,p);
ylabel(ax,'Suspension travel [mm]');
title(ax,'(a) Body-to-wheel displacement','FontWeight','normal');
legend(ax,'Location','southoutside','Orientation','horizontal','Box','off');
style(ax,t);
ax = nexttile(layout);
draw_cases(ax,t,response,3,1e3,colors,styles,names);
events(ax,p);
ylabel(ax,'Tire deflection [mm]');
xlabel(ax,'Time [s]');
title(ax,'(b) Wheel-to-road displacement','FontWeight','normal');
style(ax,t);
save_figure(fig,'02_suspension_travel',visible);

[fig,layout] = new_figure('Active actuator effort',2,visible);
ax = nexttile(layout);
draw_cases(ax,t,response(2:3),4,1,colors(2:3,:),styles(2:3),names(2:3));
events(ax,p);
ylabel(ax,'Actuator force [N]');
title(ax,'(a) Force between body and wheel','FontWeight','normal');
legend(ax,'Location','southoutside','Orientation','horizontal','Box','off');
style(ax,t);
ax = nexttile(layout);
draw_cases(ax,t,response(2:3),5,1,colors(2:3,:),styles(2:3),names(2:3));
events(ax,p);
ylabel(ax,'Mechanical actuator power [W]');
xlabel(ax,'Time [s]');
title(ax,'(b) Signed force-times-relative-velocity power','FontWeight','normal');
style(ax,t);
save_figure(fig,'03_actuator_effort',visible);
end

function draw_cases(ax,t,response,column,scale,colors,styles,names)
hold(ax,'on');
for k = 1:numel(response)
    plot(ax,t,scale*response{k}(:,column),'Color',colors(k,:), ...
        'LineStyle',styles{k},'LineWidth',1.7,'DisplayName',names{k});
end
end

function events(ax,p)
limits = ylim(ax);
ylim(ax,[limits(1),limits(2)+0.15*diff(limits)]);
xline(ax,p.bumpTime,'--','Bump begins','Color',[.45 .2 .55], ...
    'LabelOrientation','horizontal','FontSize',10,'HandleVisibility','off');
xline(ax,p.bumpTime+p.bumpDuration,':','Color',[.45 .2 .55], ...
    'HandleVisibility','off');
end

function [fig,layout] = new_figure(name,panels,visible)
state = 'off';
if visible, state = 'on'; end
fig = figure('Name',name,'Color','w','Visible',state, ...
    'Position',[100,80,980,450+140*(panels-1)]);
layout = tiledlayout(fig,panels,1,'TileSpacing','compact','Padding','loose');
end

function style(ax,t)
set(ax,'FontName','Arial','FontSize',12,'Box','off','TickDir','out', ...
    'YGrid','on','GridAlpha',.12,'LineWidth',.8);
xlim(ax,[t(1),t(end)]);
end

function save_figure(fig,name,visible)
exportgraphics(fig,fullfile('results',[name '.png']),'Resolution',200, ...
    'Padding',8,'Units','points');
exportgraphics(fig,fullfile('results',[name '.pdf']),'ContentType','vector', ...
    'Padding',8,'Units','points');
if ~visible, close(fig); end
end
