classdef mx_task < handle
    % mx_task class
    % linux x64 only
    % works with mx_sleep(time)
    %
    % usage inside a script:
    %
    % my_task = mx_task(@()routine, 1/10); % second entry is desired period
    % 
    % timestart = mx_sleep(0);
    % timenow = timestart;
    % while (timenow <= timestart + 10)    % say 10 secs sim time
    %   timenow = mx_sleep(0.0001);           % thread sleep for 100us
    %   my_task.run(timenow);
    % end
    %
    % function routine()
    % disp("Hello world.");
    % end
    
    properties
        startTimeEpoch;
        timeInitiated;
        period;
        lastTime;
        lastPeriod;
        routine;
    end
    
    methods
        function task = mx_task(handle_routine, des_period)
            task.startTimeEpoch = mx_sleep(0);
            task.timeInitiated = 0;
            task.period = des_period;
            task.lastTime = 0;
            task.lastPeriod = task.period;
            task.routine = handle_routine;
        end
        
        function run(task, timeSinceEpoch)
            task.timeInitiated = timeSinceEpoch - task.startTimeEpoch;
            evalPeriod = task.timeInitiated - task.lastTime;
            if (evalPeriod >= task.period)
                task.lastTime = timeSinceEpoch - task.startTimeEpoch;
                task.lastPeriod = evalPeriod;
                task.routine();
            end
        end
    end
end

