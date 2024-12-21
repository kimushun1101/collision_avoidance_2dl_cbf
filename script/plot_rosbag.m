% read data
% dirname = "tb3_sim";
dirname = "ais4_sim6";
filename = dirname+".csv";
data = readtable(filename, VariableNamingRule='preserve');
if startsWith(filename, "tb3")
    cmd_vel = "/cmd_vel";
    scan = "/scan";
else
    cmd_vel = "/suitcase/cmd_vel";
    scan = ["/scan", "back_scan"];
end

startTime = data.("__time")(1);

% cmd_vel
inds = ~isnan(data.(cmd_vel+"/linear/x"));
t_us = data.("__time")(inds) - startTime; 
us = [data.(cmd_vel+"/linear/x")(inds), data.(cmd_vel+"/angular/z")(inds)];

% cmd_vel_ref
inds = ~isnan(data.("/cmd_vel_ref/linear/x"));
t_urefs = data.("__time")(inds) - startTime; 
urefs = [data.("/cmd_vel_ref/linear/x")(inds), data.("/cmd_vel_ref/angular/z")(inds)];

% position
if startsWith(filename, "tb3")
    inds = ~isnan(data.("/odom/header/stamp/sec"));
    q0 = data.("/odom/pose/pose/orientation/w")(inds);
    q1 = data.("/odom/pose/pose/orientation/x")(inds);
    q2 = data.("/odom/pose/pose/orientation/y")(inds);
    q3 = data.("/odom/pose/pose/orientation/z")(inds);
    yaw = atan2(2.0 * (q1.*q2 + q0.*q3), q0.*q0 + q1.*q1 - q2.*q2 - q3.*q3);
    t_ps = data.("__time")(inds) - startTime;
    ps = [data.("/odom/pose/pose/position/x")(inds), data.("/odom/pose/pose/position/y")(inds), yaw];
else
    inds = ~isnan(data.("/cbf_debug/data[5]"));
    t_ps = data.("__time")(inds) - startTime;
    ps = [data.("/cbf_debug/data[5]")(inds), data.("/cbf_debug/data[6]")(inds), data.("/cbf_debug/data[7]")(inds)];
end

% scan
inds = ~isnan(data.("/scan/angle_max"));
t_ranges = data.("__time")(inds) - startTime;
scanRanges = zeros(sum(inds), 360);
for i = 0:359
    scanRanges(:,i+1) = data.("/scan/ranges["+num2str(i)+"]")(inds);
end
ind = find(inds, 1);
rangeMax = data.("/scan/range_max")(ind);
angleMax = data.("/scan/angle_max")(ind);
angleMin = data.("/scan/angle_min")(ind);
scanRanges(isnan(scanRanges)) = rangeMax;

% collision polygon
ind = find(~isnan(data.("/collision_polygon/header/stamp/nanosec")),1);
collision_poly_ = zeros(2, 100);
for i = 0:99
    collision_poly_(1, i+1) = data.("/collision_polygon/polygon/points["+num2str(i)+"]/x")(ind);
    collision_poly_(2, i+1) = data.("/collision_polygon/polygon/points["+num2str(i)+"]/y")(ind);
end
BxP = collision_poly_(1,:);
ByP = collision_poly_(2,:);


% h I J
inds = ~isnan(data.("/cbf_debug/data[0]"));
t_hIJs = data.("__time")(inds) - startTime; 
hs = data.("/cbf_debug/data[0]")(inds);
Is = data.("/cbf_debug/data[1]")(inds);
Js = data.("/cbf_debug/data[2]")(inds);

%% plot figures
close all

figure(Name='Comparison reference and command linear velocities')
plot(t_urefs, urefs(:,1))
hold on
plot(t_us, us(:,1))
legend('$u_{\rm ref1}$', '$u_1 + u_{\rm ref1}$', 'Location', 'southeast')
xlabel('Time [s]')
ylabel('forward [m/s]')

figure(Name='Comparison reference and command rotational velocities')
plot(t_urefs, urefs(:,2))
hold on
plot(t_us, us(:,2))
legend('$u_{\rm ref2}$', '$u_1 + u_{\rm ref2}$', 'Location', 'northwest')
xlabel('Time [s]')
ylabel('rotation [rad/s]')

figure(Name='Comparison linear and anguler velocities of command')
plot(t_us, [us(:,1), us(:,2)])
legend('$u_1$', '$u_2$', 'Location', 'southeast')
xlabel('Time [s]')
ylabel('forward and angular [m/s, rad/s]')

%
figure(Name='CBF value')
plot(t_hIJs, hs)
xlabel('Time [s]')
ylabel('CBF $h$',Interpreter='latex')

%
figure(Name='I and J values')
plot(t_hIJs, [Is, Js])
legend('$I$', '$J$')
xlabel('Time [s]')
ylabel('$I$ or $J$',Interpreter='latex')

%
figure(Name='Trajectory')
plot(ps(:,1), ps(:,2))
axis square
hold on
plot([2.4 -2.4 -2.4 2.4 2.4], [2.4 2.4 -2.4 -2.4 2.4])

gcolor = [0.4660 0.6740 0.1880];

ind = find(t_ps>13.6669, 1);
OxP = BxP*cos(ps(ind,3)) - ByP*sin(ps(ind,3)) + ps(ind,1);
OyP = BxP*sin(ps(ind,3)) + ByP*cos(ps(ind,3)) + ps(ind,2);
patch(OxP, OyP,gcolor, 'EdgeColor',gcolor,'FaceAlpha', 0.5,'LineWidth',2)

ind = find(t_ps>28.3, 1);
OxP = BxP*cos(ps(ind,3)) - ByP*sin(ps(ind,3)) + ps(ind,1);
OyP = BxP*sin(ps(ind,3)) + ByP*cos(ps(ind,3)) + ps(ind,2);
patch(OxP, OyP,gcolor,'EdgeColor',gcolor,'FaceAlpha', 0.5,'LineWidth',2)

ind = size(t_ps, 1);
OxP = BxP*cos(ps(ind,3)) - ByP*sin(ps(ind,3)) + ps(ind,1);
OyP = BxP*sin(ps(ind,3)) + ByP*cos(ps(ind,3)) + ps(ind,2);
patch(OxP, OyP,gcolor,'EdgeColor',gcolor,'FaceAlpha', 0.5,'LineWidth',2)

xlabel('$p_1$ [m]',Interpreter='latex')
ylabel('$p_2$ [m]',Interpreter='latex')

% figure(Name='Robot position')
% plot(t_ps, [ps(:,1), ps(:,2)])

% https://github.com/kimushun1101/arrange-resize-figures
arrangeResizeFigures()
% arrangeResizeFigures("Division",[3, 3],"FigureNumbers",[1,2,3])
% arrangeResizeFigures("Division",[3, 2],"FigureNumbers",4)