classdef mx_vesc < handle
    %% mx_vesc Summary
    % Usage example:
    % myVESC = mx_vesc('/dev/ttyACM0');
    % myVESC.open;
    % myVESC.get_sensors;
    % disp(myVESC.sensors.current_motor);
    % clear myVESC
    
    properties
        dev                     % char array
        ptr                     % double carrying a pointer
        print_on_get            % set 1 if you want to display on get_
        port_status             % serial port status
        sensors                 % sensor struct; see below
        config                  % config struct; see below
        new_config_read         % internal use
        fault_data              % see datatypes.h
        pos_dir                 % set this to -1 to flip position direction
        curr_dir                % set this to -1 to flip current direction
    end
    
    methods
        function vesc = mx_vesc(dev)
            % mx_vesc Construct an instance of this class
            %% the first input must be a char array to a serial device
            [~, clidevstr] = system('ls /dev/');
            found_dev = strfind(clidevstr, erase(dev, '/dev/'));
            
            if (length(found_dev) ~= 1 || found_dev <= 0)
                error("The first input must be a char array of a valid serial device, e.g. '/dev/ttyACM0'.");
            end
            
            %% init
            vesc.dev = dev;
            vesc.ptr = double(0);
            vesc.print_on_get = double(0);
            vesc.port_status = 'closed';
            vesc.new_config_read = 'no';
            
            %sensors
            vesc.sensors.v_in = 0;                  % [V]
            vesc.sensors.pid_pos = 0;               % [deg]
            vesc.sensors.current_motor = 0;         % [A]
            vesc.sensors.fault_code = 0;            
            
            %config
            vesc.config.l_current_max = 0;
            vesc.config.l_current_min = 0;
            vesc.config.l_in_current_max = 0;
            vesc.config.l_in_current_min = 0;
            vesc.config.l_abs_current_max = 0;
            vesc.config.l_min_erpm = 0;
            vesc.config.l_max_erpm = 0;
            vesc.config.s_pid_kp = 0;
            vesc.config.s_pid_ki = 0;
            vesc.config.s_pid_kd = 0;
            vesc.config.s_pid_kd_filter = 0;
            vesc.config.s_pid_min_erpm = 0;
            vesc.config.s_pid_allow_braking = 0;
            vesc.config.p_pid_kp = 0;
            vesc.config.p_pid_ki = 0;
            vesc.config.p_pid_kd = 0;
            vesc.config.p_pid_kd_filter = 0;
            vesc.config.p_pid_ang_div = 0;
            
            vesc.curr_dir = 1;
            vesc.pos_dir = 1;
            
            vesc.ptr = mx_vesc_interface('new');    % initiating
            
            fprintf('VESC ready. \n');
        end
        
        function open(vesc)
            disp(['Opening VESC at ', vesc.dev, '...']);
            mx_vesc_interface('open', vesc.ptr, vesc.dev);
            vesc.port_status = 'open';
            vesc.get_config;
        end
        
        function close(vesc)
            disp(['Closing VESC at ', vesc.dev, '...']);
            mx_vesc_interface('close', vesc.ptr);
            vesc.port_status = 'closed';
        end
        
        function delete(vesc)
            disp(['Deleting VESC at ', vesc.dev, '...']);
            mx_vesc_interface('delete', vesc.ptr);
        end
        
        function get_sensors(vesc)
            if (vesc.port_status ~= "open")
                fprintf("Please open the device at %s.\n", vesc.dev);
            else
               vesc.sensors = mx_vesc_interface('get_sensors', vesc.ptr);
               if (vesc.print_on_get)
                   disp(vesc.sensors);
               end
            end
        end
        
        function get_config(vesc)
            if (vesc.port_status ~= "open")
                fprintf("Please open the device at %s.\n", vesc.dev);
            else
                try
                    vesc.config = mx_vesc_interface('get_config', vesc.ptr);
                catch
                    vesc.new_config_read = 'no';
                    return
                end
               vesc.new_config_read = 'yes';
               if (vesc.print_on_get)
                   disp(vesc.config);
               end
            end
        end
        
        function send_current(vesc, current)
            if (vesc.port_status ~= "open")
                fprintf("Please open the device at %s.\n", vesc.dev);
            else
               mx_vesc_interface('send_current', vesc.ptr, current * vesc.curr_dir);
            end
        end
    end
end

