classdef TmotorCASPRInterface < CableActuatorInterfaceBase
    properties 
        motor
        KP
        KI
        KD
        T_pos
        C_pos
        I_pos
        s0
        Initial_length
        s1
        C_v
        Ierror
        task_filter
        rotation
        task_controller
        spool_radius
        period1
        
    end
    
    methods
        function Obj = TmotorCASPRInterface(ttyl, kp,ki,kd, period, Spool)
            Clear;
            Obj@CableActuatorInterfaceBase();
            Obj.motor = mx_vesc(ttyl);
            Obj.KP = kp;
            Obj.KD = kd;
            Obj.KI = ki;
            Obj.spool_radius = Spool;
            Obj.rotation = 0;
            Obj.initial_length = 0;
            Obj.s0 = 0;
            Obj.s1 = 0;
            Obj.Ierror = 0;
            Obj.period1 = period;
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
             tpos = l_cmd / Obj.spool_radius;
             Obj.motor.get_sensors();
             Obj.C_pos = (Obj.motor.sensors.pid_pos - 180)*pi/180;
             Obj.T_pos = tpos + Obj.C_pos;
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
            length = (Obj.C_pos - Obj.I_pos) * Obj.spool_radius + Obj.Initial_length;
         end
         
         function forceCommandSend(Obj, f_cmd)
             Obj.motor.current
         end

         
         
     
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
            cmd_current = -0.01 * s;                           % smooth sliding controller
            cmd_current = sign(cmd_current) * min(abs(cmd_current), 1); % current limit
            %motorz.s1 = cmd_current;
            %motorz.s1 = motorz.C_pos;
            Obj.motor.send_current(cmd_current);
         end
    end
end