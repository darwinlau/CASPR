clear myMotor task_filter task_controller
try
    close(fig(1));
catch
end

%% global variables are needed for inter-tasks read/write
global myMotor0 myMotor1 myMotor2 myMotor3 myMotor4 myMotor5 dt_rec dt_idx
global q dq qd dqd motorarray currentlimit
global task_filter task_controller task_cmd

myMotor0 = mx_vesc('/dev/ttyACM0');  % define USB port here
myMotor1 = mx_vesc('/dev/ttyACM1');
myMotor2 = mx_vesc('/dev/ttyACM2');
myMotor3 = mx_vesc('/dev/ttyACM3');
%myMotor4 = mx_vesc('/dev/ttyACM4');
%myMotor5 = mx_vesc('/dev/ttyACM5');


motorarray = [myMotor0,  myMotor1,myMotor2, myMotor3];
%myMotor{2} = {mx_vesc('/dev/ttyACM1') mx_vesc('/dev/ttyACM1')};  % if there's other VESCs
dt_rec = zeros(100000, 1);          % for task period histogram
dt_idx = 0;                             
%q = [0 0 0 0 0 0];                              % motor position in rad
q = [0 0 0 0 0 0];
%dq = [0 0 0 0 0 0];                             % motor rate in rad/s
dq = [pi/2 pi 3/2*pi 2*pi 0 0];
%qd = [0 0 0 0 0 0];                             % desired pos
qd = [0 0 0 0 0 0];
%dqd = [0 0 0 0 0 0];                            % desired rate
dqd = [ 0 0 0 0 0 0];
currentlimit = [5 5 5 5 1 1];
%myMotor.curr_dir = -1;              % flips current direction if necessary

% mx_task class definition
% second argument is the desired task period
task_filter = mx_task(@()filter, 1/1200); % filter task
task_controller = mx_task(@()controller, 1/1200); % controller task
task_cmd = mx_task(@()cmd_gen, 1/1200); % command generator task

%% Main tasks
for i = 1:4
    motorarray(i).open;
end


timestart = mx_sleep(0);            % get number of seconds since epoch
timenow = timestart;                % needed to enter the while loop below

while (timenow <= timestart + 20) % runs tasks for 10 seconds                    
    timenow = mx_sleep(0.00001); % sleeps thread for 10us
    
    % place task runs here according to task priorities
    task_filter.run(timenow);
    task_cmd.run(timenow);
    task_controller.run(timenow);
end

for i = 1:4
    motorarray(i).delete;
end

fig(1) = figure(1);
fig(1) = histogram(dt_rec(1: dt_idx));
fig(1).BinWidth = task_controller.period/50;
xlim([0 task_controller.period*5])
hold on
plot(max(dt_rec), 0, 'rx');
hold off

%% Task definitions
function filter()
global task_filter motorarray q dq % define passed global variables here
for i = 1:4
oq = q(i);
motorarray(i).get_sensors(); % get sensor
q(i) = (motorarray(i).sensors.pid_pos - 180) * pi / 180; % read pos and converts it to rad
qt = q(i) - oq;                                    % change in q
qt = qt - 2*pi*(qt>pi) + 2*pi*(qt<=-pi);        % take short side
dq(i) = qt / task_filter.lastPeriod;               % euler approximation on speed
end

end

function controller()
global task_controller motorarray q dq qd dqd dt_rec dt_idx currentlimit

for i = 1:4
qt = q(i) - qd(i);                                    % tracking error
qt = qt - 2*pi*(qt>pi) + 2*pi*(qt<=-pi);        % short side
dqt = dq(i) - dqd(i);                                 % rate error
s = 3*dqt + sign(qt) * min(50*pi, abs(200 * qt));      % sliding surface with saturated rate
cmd_current = -0.05 * s;                           % smooth sliding controller
cmd_current = sign(cmd_current) * min(abs(cmd_current), currentlimit(i)); % current limit
motorarray(i).send_current(cmd_current);
end
% recording periods
dt_idx = dt_idx + 1;
dt_rec(dt_idx) = task_controller.lastPeriod;
end

function cmd_gen()
global task_cmd qd dqd

% qd = pi/4 * sign(sin(task_cmd.timeInitiated * 2*pi*10)); % step generator
% qd = [0 0 0 0];
% dqd = [0 0 0 0];
end

