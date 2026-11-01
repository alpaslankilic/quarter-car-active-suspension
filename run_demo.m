function report = run_demo(showFigures)
%RUN_DEMO Compare passive, skyhook and saturated LQR suspensions.
if nargin == 0, showFigures = true; end
root = fileparts(mfilename('fullpath'));
previous = pwd;
cleanup = onCleanup(@() cd(previous));
cd(root);
p = project_parameters();
[A, Bu, Br, gains] = quarter_car_model(p);
names = {'Passive', 'Skyhook', 'LQR'};
probe = (0:p.sampleTime:10).';
settling = zeros(3, 1);
for k = 1:3
    states = solve_reference(probe, p, A, Bu, Br, gains(k, :));
    signals = derived_signals(probe, states, p, A, Bu, gains(k, :));
    settling(k) = settling_duration(probe, signals(:, 1:3), p);
end
bumpEnd = p.bumpTime + p.bumpDuration;
p.stopTime = ceil((bumpEnd + 1.2*max(settling))/p.sampleTime)*p.sampleTime;
t = (0:p.sampleTime:p.stopTime).';
road = road_profile(t, p);
roadInput = timeseries(road, t);
response = cell(1, 3);
metrics = repmat(struct(),1,3);
if ~isfolder('results'), mkdir('results'); end
for k = 1:3
    in = Simulink.SimulationInput('quarter_car');
    values = {p, A, Bu, Br, gains(k, :), roadInput};
    variables = {'p','A','Bu','Br','K','roadInput'};
    for j = 1:numel(variables)
        in = in.setVariable(variables{j}, values{j}, 'Workspace', 'quarter_car');
    end
    in = in.setModelParameter('StopTime', num2str(p.stopTime, 12), ...
        'Solver', 'ode45', 'MaxStep', '0.001', 'RelTol', '1e-8', 'AbsTol', '1e-10');
    out = sim(in);
    states = interp1(out.states.Time, out.states.Data, t, 'linear');
    response{k} = derived_signals(t, states, p, A, Bu, gains(k, :));
    exact = solve_reference(t, p, A, Bu, Br, gains(k, :));
    error = max(abs(states-exact), [], 'all');
    % From Workspace linearly interpolates the 1 ms road samples.
    assert(error < 2e-4, 'State response disagrees with ODE reference.');
    assert(max(abs(response{k}(:, 4))) <= p.forceLimit + 1e-6, 'Force limit failed.');
    contact = (p.sprungMass+p.unsprungMass)*9.81 ...
        - p.tireStiffness*response{k}(:, 3);
    assert(min(contact) > 0, 'Linear tire model predicts loss of road contact.');
    after = t >= p.bumpTime;
    metrics(k).controller = names{k};
    metrics(k).body_acceleration_rms_m_s2 = rms(response{k}(after, 1));
    metrics(k).peak_body_acceleration_m_s2 = max(abs(response{k}(:, 1)));
    metrics(k).peak_suspension_travel_mm = 1e3*max(abs(response{k}(:, 2)));
    metrics(k).peak_tire_deflection_mm = 1e3*max(abs(response{k}(:, 3)));
    metrics(k).peak_actuator_force_N = max(abs(response{k}(:, 4)));
    metrics(k).peak_abs_actuator_power_W = max(abs(response{k}(:, 5)));
    metrics(k).minimum_tire_normal_force_N = min(contact);
    metrics(k).ode_max_state_error = error;
    metrics(k).post_bump_settling_s = settling(k);
    data = array2table([t, road, states, response{k}], 'VariableNames', ...
        {'time_s','road_m','body_m','body_velocity_m_s','wheel_m', ...
        'wheel_velocity_m_s','body_acceleration_m_s2','suspension_travel_m', ...
        'tire_deflection_m','actuator_N','actuator_power_W'});
    writetable(data, fullfile('results', [lower(names{k}) '.csv']));
end
report.controllers = metrics;
report.stop_time_s = p.stopTime;
report.lqr_acceleration_rms_change_percent = 100*( ...
    metrics(3).body_acceleration_rms_m_s2/metrics(1).body_acceleration_rms_m_s2 - 1);
report.verification = 'PASS';
write_json('results/metrics.json', report);
write_json('results/parameters.json', p);
write_json('results/controller_gains.json', struct('passive',gains(1,:), ...
    'skyhook',gains(2,:),'lqr',gains(3,:)));
plot_results(t, road, response, names, p, showFigures);
disp(report);
disp(struct2table(metrics));
end

function states = solve_reference(t, p, A, Bu, Br, gain)
options = odeset('RelTol',1e-9,'AbsTol',1e-11,'MaxStep',0.001);
solution = ode45(@(time,x) A*x + Bu*limit(-gain*x,p.forceLimit) ...
    + Br*road_profile(time,p), [t(1),t(end)], zeros(4,1), options);
states = deval(solution,t).';
end

function signals = derived_signals(t, states, p, A, Bu, gain)
force = limit(-states*gain',p.forceLimit);
acceleration = states*A(2,:)' + Bu(2)*force;
travel = states(:,1)-states(:,3);
tire = states(:,3)-road_profile(t,p);
power = force.*(states(:,2)-states(:,4));
signals = [acceleration,travel,tire,force,power];
end

function duration = settling_duration(t, signals, p)
peak = max(abs(signals),[],1);
outside = any(abs(signals) > 0.02*peak,2);
after = find(t >= p.bumpTime+p.bumpDuration);
last = find(outside(after),1,'last');
assert(~isempty(last) && last < numel(after), 'Bump response did not settle.');
duration = t(after(last+1))-p.bumpTime-p.bumpDuration;
end

function value = limit(value, bound)
value = max(-bound,min(bound,value));
end

function write_json(path, value)
file = fopen(path,'w');
assert(file ~= -1,'Cannot write output.');
cleanup = onCleanup(@() fclose(file));
fprintf(file,'%s\n',jsonencode(value,'PrettyPrint',true));
end
