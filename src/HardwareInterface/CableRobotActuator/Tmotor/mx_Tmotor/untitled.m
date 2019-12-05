mobj1 = motorobj('/dev/ttyACM0',2,0.0001,0.8,1/500);
mobj1.startm;

% mobj1.motor.get_sensors();
% disp(mobj1.motor.sensors.pid_pos);

ind = zeros(100000,1);
iind = zeros(100000,1);
inde = 0;
timestart = mx_sleep(0);
timenow = timestart;
mobj1.target(4*pi);

while (timenow <= timestart +10) % runs tasks for 5 seconds                    
    timenow = mx_sleep(0.0000001); % sleeps thread for 10us
    mobj1.run(timenow);
    inde = inde + 1;
    ind(inde) = mobj1.C_pos - mobj1.T_pos - mobj1.s0 * 2*pi;
    iind(inde) = mobj1.s1;
end

endm(mobj1);

fig(2)=figure(2);
x = 1:1:inde ;
fig(2) =plot(x, ind(x)/pi,'r');
hold on
fig(2) = plot(x,iind(x),'b');
title("position")
hold off
