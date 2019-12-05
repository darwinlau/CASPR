classdef TmotorCASPRInterface < CableActuatorInterfaceBase

    properties (Access = public)
        motor
        KP
        KI
        KD
        T_pos
        C_pos
        I_pos
        C_v
        Ierror
        rotation
        s0
        cmd_current
        max_current
        initial_length
        spool_radius
        task_filter
        task_controller
        period1
    end

    methods ( Access = public)
        %please enter port, P gain , I gain , D gain, desired period limit, radius of spool and max current
        function Obj = TmotorCASPRInterface(ttyl, kp,ki,kd, period, Spool_r, max_curr)
            Obj@CableActuatorInterfaceBase();
            Obj.motor = mx_vesc(ttyl);
            Obj.KP = kp;
            Obj.KD = kd;
            Obj.KI = ki;
            Obj.spool_radius = Spool_r;
            Obj.rotation = 0;
            Obj.initial_length = 0;
            Obj.s0 = 0;
            Obj.Ierror = 0;
            Obj.period1 = period;
            Obj.cmd_current = 0;
            Obj.max_current = max_curr;
            Obj.T_pos = 0;
            Obj.C_pos = 0;
            Obj.I_pos = 0;
            Obj.C_v = 0;
            Obj.task_filter =  mx_task(@()Obj.filter, period);
            Obj.task_controller = mx_task(@()Obj.controller, period);
        end

        function open(Obj)
            Obj.motor.open;
        end

        function close(Obj)
            Obj.motor.close;
        end

        function lengthCommandSend(Obj , l_cmd)
             tpos = (l_cmd - Obj.initial_length) / Obj.spool_radius;
             Obj.T_pos = tpos + Obj.I_pos;
             timenow = mx_sleep(0.00001);
             Obj.task_filter.run(timenow);
             Obj.task_controller.run(timenow);
        end

        function lengthInitialSend(Obj,I_length)
             Obj.initial_length = I_length;
             Obj.motor.get_sensors();
             Obj.I_pos = (Obj.motor.sensors.pid_pos - 180)*pi/180;
        end

        function length = lenghtFeedbackRead(Obj)
            Obj.motor.get_sensors();
            Obj.C_pos = (Obj.motor.sensors.pid_pos - 180)*pi/180 + Obj.rotation * 2 * pi;
            length = (Obj.C_pos - Obj.I_pos) * Obj.spool_radius + Obj.initial_length;
        end

        function forceCommandSend(Obj, f_cmd) % Current
             Obj.motor.send_current(f_cmd);
        end

        function force = forceFeedbackRead (Obj)
             force = Obj.cmd_current;
        end

        function systemOnSend(Obj)
        end

        function systemOffSend(Obj)
        end

        function initialise(Obj)
        end

        function switchOperatingMode(Obj)
        end

        function [success] = detectDevice(Obj)
        end

    end

    methods (Access = private)
        function filter(Obj)
        oq = Obj.C_pos;
        Obj.motor.get_sensors();
        % get sensor
        Obj.C_pos = (Obj.motor.sensors.pid_pos - 180) * pi / 180;
        % read pos and converts it to rad
        qq = Obj.s0;
        qt = Obj.C_pos - oq+ 2*pi*Obj.rotation;

        if abs(qt) >3/2*pi
            Obj.s0 = sign(qt)*1;
            %rotation = rotation + sign(qt)*1;
        else
            Obj.s0 =0;
        end

        if abs(qq -Obj.s0) >0 && Obj.s0 ==0
            Obj.rotation = Obj.rotation + sign(qt)*1;
        end

        qt = qt - 2*pi*(qt>pi) + 2*pi*(qt<=-pi);
        % take short side
        Obj.C_v = qt / Obj.task_filter.lastPeriod;
        % euler approximation on speed
        end

        function controller(Obj)
          Obj.C_pos = Obj.C_pos + 2* pi *Obj.rotation;
          qt = Obj.C_pos - Obj.T_pos - Obj.s0 *2*pi;
          % tracking error
          Obj.Ierror = Obj.Ierror +qt;
          s = Obj.KI* Obj.Ierror+Obj.KD*Obj.C_v + Obj.KP*sign(qt) * min(50*pi, abs(200 * qt));
          % sliding surface with saturated rate
          Obj.cmd_current = -0.01 * s;                           % smooth sliding controller
          Obj.cmd_current = sign(Obj.cmd_current) * min(abs(Obj.cmd_current), Obj.max_current); % current limit
          %motorz.s1 = cmd_current;
          %motorz.s1 = motorz.C_pos;
          Obj.motor.send_current(Obj.cmd_current);
        end
    end
end
