clear myMotor task_filter task_controller
try
    close(fig(1));
catch
end

%% global variables are needed for inter-tasks read/write
global myMotor dt_rec dt_idx
global q dq qd dqd KD KP KI
global task_filter task_controller task_cmd rotation s0 s1 cmd_rec pos_rec I_pos

myMotor = mx_vesc('/dev/ttyACM0');  % define USB port here
%myMotor{2} = {mx_vesc('/dev/ttyACM1') mx_vesc('/dev/ttyACM1')};  % if there's other VESCs
dt_rec = zeros(100000, 1);          % for task period histogram
cmd_rec = zeros(100000,1);
pos_rec = zeros(100000,1);
dt_idx = 0;                             
q = 0;                              % motor position in rad
dq = 0;                             % motor rate in rad/s
qd = 4*pi;                             % desired pos
dqd = 0;                            % desired rate
I_pos = 0;
s0 = 0;
s1 = 0;
rotation  = 0;
KD = 2;
KP = 0.8;
KI = 0.0001;
%myMotor.curr_dir = -1;        0.      % flips current direction if necessary

% mx_task class definitio
% second argument is the desired task period
task_filter = mx_task(@()filter, 1/500); % filter task
task_controller = mx_task(@()controller, 1/500); % controller task
task_cmd = mx_task(@()cmd_gen, 1/500); % command generator task

%% Main tasks
myMotor.open;                       % open vesc

timestart = mx_sleep(0);            % get number of seconds since epoch
timenow = timestart;                % needed to enter the while loop below

myMotor.get_sensors(); % get sensor
q = (myMotor.sensors.pid_pos - 180) * pi / 180; % read pos and converts it to rad
qd = qd+q;

while (timenow <= timestart +5)
    timenow = mx_sleep(0.00001); % sleeps thread for 10us
    
    % place task runs here according to task priorities
    task_filter.run(timenow);
    task_cmd.run(timenow);
    task_controller.run(timenow);
end

myMotor.delete; 


fig(1) = figure(1);
fig(1) = histogram(dt_rec(1: dt_idx));
fig(1).BinWidth = task_controller.period/50;
xlim([0 task_controller.period*5])
hold on
title("freq");
fig(1) = plot(max(dt_rec), 0, 'rx');
hold off

fig(2)=figure(2);
x = 1:1:dt_idx ;
fig(2) =plot(x, cmd_rec(x),'-b');

hold on
fig(2) = plot(x, pos_rec(x),'r');
title("current command")
hold off
% 
% fig(3) = figure(3);
% hold on
% title("position")
% 
% hold off




%% Task definitions
function filter()
global task_filter myMotor q dq s0 rotation % define passed global variables here

oq = q;
myMotor.get_sensors(); % get sensor
q = (myMotor.sensors.pid_pos - 180) * pi / 180; % read pos and converts it to rad
qq = s0;
qt = q - oq  + 2* pi *rotation;       
if abs(qt) >3/2*pi
    s0 = sign(qt)*1;
%    rotation = rotation + sign(qt)*1;
else 
    s0 =0;
end

if abs(qq -s0) >0 && s0 ==0
   
   rotation = rotation + sign(qt)*1;

end

qt = qt - 2*pi*(qt>pi) + 2*pi*(qt<=-pi);        % take short side
dq = qt / task_filter.lastPeriod;               % euler approximation on speed
end

function controller()
global task_controller myMotor q dq qd dqd dt_rec dt_idx cmd_rec pos_rec I_pos KD KP KI rotation s0

q = q + 2* pi *rotation;
qt = q - qd - s0 *2*pi;                                    % tracking error
I_pos = I_pos +qt;
%qt = qt - 2*pi*(qt>pi) + 2*pi*(qt<=-pi);        % short side
dqt = dq - dqd;                                 % rate error
s = KD*dqt + KP*sign(qt) * min(50*pi, abs(200 * qt)) +KI * I_pos;      % sliding surface with saturated rate
cmd_current = -0.01 * s;                           % smooth sliding controller
cmd_current = sign(cmd_current) * min(abs(cmd_current), 1); % current limit
%myMotor.send_current(cmd_current);

% recording periods
dt_idx = dt_idx + 1;
dt_rec(dt_idx) = task_controller.lastPeriod;
cmd_rec(dt_idx) = I_pos;
pos_rec(dt_idx) = qt/pi;

end

function cmd_gen()
global qd q
    disp (q);
 %qd = pi/4 * sign(sin(task_cmd.timeInitiated * 2*pi*10)); % step generator
% qd = 0;
% dqd = 0;-
end

