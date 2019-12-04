
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <poll.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "vesc_usb.h"
#include "buffer.h"
#include "crc.h"


vescUSB::vescUSB()
{
	//can_fwd_vesc = -1;

	// Function pointers
	send_func = 0;
	forward_func = 0;

	// Function pointers for received data
	rx_value_func = 0;
	rx_printf_func = 0;
	rx_fw_func = 0;
	rx_rotor_pos_func = 0;
	rx_mcconf_func = 0;
	rx_appconf_func = 0;
	rx_detect_func = 0;
	rx_dec_ppm_func = 0;
	rx_dec_adc_func = 0;
	rx_dec_chuk_func = 0;
	rx_mcconf_received_func = 0;
	rx_appconf_received_func = 0;
	motor_control_set_func = 0;
	values_requested_func = 0;
	mcconf_read = false;
    values_read = false;
}

/******************** packet ********************/

void vescUSB::packet_init(void(vescUSB::*s_func)(unsigned char *data, unsigned int len),
	void(vescUSB::*p_func)(unsigned char *data, unsigned int len), int handler_num) {
	//handler_states[handler_num].send_func = s_func;
	//handler_states[handler_num].process_func = p_func;
}

void vescUSB::packet_send_packet(unsigned char *data, unsigned int len, int handler_num) {
	if (len > PACKET_MAX_PL_LEN) {
		return;
	}

	int b_ind = 0;

	if (len <= 256) {
		handler_states[handler_num].tx_buffer[b_ind++] = 2;
		handler_states[handler_num].tx_buffer[b_ind++] = len;
	}
	else {
		handler_states[handler_num].tx_buffer[b_ind++] = 3;
		handler_states[handler_num].tx_buffer[b_ind++] = len >> 8;
		handler_states[handler_num].tx_buffer[b_ind++] = len & 0xFF;
	}

	memcpy(handler_states[handler_num].tx_buffer + b_ind, data, len);
	b_ind += len;

	unsigned short crc = crc16(data, len);
	handler_states[handler_num].tx_buffer[b_ind++] = (uint8_t)(crc >> 8);
	handler_states[handler_num].tx_buffer[b_ind++] = (uint8_t)(crc & 0xFF);
	handler_states[handler_num].tx_buffer[b_ind++] = 3;

	/*
	if (handler_states[handler_num].send_func) {
		(this->*handler_states[handler_num].send_func)(handler_states[handler_num].tx_buffer, b_ind);
	}
	*/
	if (1) {
		send_packet(handler_states[handler_num].tx_buffer, b_ind);
	}


}

/**
 * Call this function every millisecond.
 */
void vescUSB::packet_timerfunc(void) {
	int i = 0;
	for (i = 0; i < PACKET_HANDLERS; i++) {
		if (handler_states[i].rx_timeout) {
			handler_states[i].rx_timeout--;
		}
		else {
			handler_states[i].rx_state = 0;
		}
	}
}

void vescUSB::packet_process_byte(uint8_t rx_data, int handler_num) {
	switch (handler_states[handler_num].rx_state) {
	case 0:
		if (rx_data == 2) {
			// 1 byte PL len
			handler_states[handler_num].rx_state += 2;
			handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
			handler_states[handler_num].rx_data_ptr = 0;
			handler_states[handler_num].payload_length = 0;
		}
		else if (rx_data == 3) {
			// 2 byte PL len
			handler_states[handler_num].rx_state++;
			handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
			handler_states[handler_num].rx_data_ptr = 0;
			handler_states[handler_num].payload_length = 0;
		}
		else {
			handler_states[handler_num].rx_state = 0;
		}
		break;

	case 1:
		handler_states[handler_num].payload_length = (unsigned int)rx_data << 8;
		handler_states[handler_num].rx_state++;
		handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
		break;

	case 2:
		handler_states[handler_num].payload_length |= (unsigned int)rx_data;
		if (handler_states[handler_num].payload_length > 0 &&
			handler_states[handler_num].payload_length <= PACKET_MAX_PL_LEN) {
			handler_states[handler_num].rx_state++;
			handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
		}
		else {
			handler_states[handler_num].rx_state = 0;
		}
		break;

	case 3:
		handler_states[handler_num].rx_buffer[handler_states[handler_num].rx_data_ptr++] = rx_data;
		if (handler_states[handler_num].rx_data_ptr == handler_states[handler_num].payload_length) {
			handler_states[handler_num].rx_state++;
		}
		handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
		break;

	case 4:
		handler_states[handler_num].crc_high = rx_data;
		handler_states[handler_num].rx_state++;
		handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
		break;

	case 5:
		handler_states[handler_num].crc_low = rx_data;
		handler_states[handler_num].rx_state++;
		handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
		break;

	case 6:
		if (rx_data == 3) {
			if (crc16(handler_states[handler_num].rx_buffer, handler_states[handler_num].payload_length)
				== ((unsigned short)handler_states[handler_num].crc_high << 8
					| (unsigned short)handler_states[handler_num].crc_low)) {
				// Packet received!
				/*
				if (handler_states[handler_num].process_func) {
					(this->*handler_states[handler_num].process_func)(handler_states[handler_num].rx_buffer,
						handler_states[handler_num].payload_length);
				}
				*/
				if (1) {
					bldc_interface_process_packet(handler_states[handler_num].rx_buffer,
						handler_states[handler_num].payload_length);
				}

			}
		}
		handler_states[handler_num].rx_state = 0;
		break;

	default:
		handler_states[handler_num].rx_state = 0;
		break;
	}
}

/******************** comm_uart ********************/

/*
 * This function should be called every time data is requested from VESC.
 * Call this function at least 10ms after calling a get function.
 * Can receive and process multiple packets with a single call
 * as long as SERIAL_RX_BUFFER_SIZE is not exceeded.
 * Returns 1 to indicate that buffer has been read and sent to packet handler.
 */
int vescUSB::receive_packet() {

	// MAIN COMM READ FUNC
	int ret, i;
	static uint8_t buffer[SERIAL_RX_BUFFER_SIZE];
	// Read characters from Serial and put them in a buffer
	ret = read(fd, buffer, SERIAL_RX_BUFFER_SIZE);
	// Loop through the buffer and send each byte to the
	// packet handler
	for (i = 0; i < ret; i++) {

		// MAIN PROCESS BYTE FUNCTION
		//bldc_interface_uart_process_byte(buffer[i]);
		packet_process_byte(buffer[i], PACKET_HANDLER);
	}
	// clear any data on Serial Rx that was not read
	//tcflush(fd, TCIFLUSH);
	return 1;
}

/**
 * Callback that the packet handler uses to send an assembled packet.
 *
 * @param data
 * Data array pointer
 * @param len
 * Data array length
 */
void vescUSB::send_packet(unsigned char *data, unsigned int len) {
	if (len > (PACKET_MAX_PL_LEN + 5)) {
		return;
	}

	// Wait for the previous transmission to finish.
	tcdrain(fd);

	// Copy this data to a new buffer in case the provided one is re-used
	// after this function returns.
	static uint8_t buffer[PACKET_MAX_PL_LEN + 5];
	memcpy(buffer, data, len);

	// Send the data over Serial
	write(fd, buffer, len);
}

/*
 * Call this function once from main program to initialize
 * serial port for reading (non-blocking) and writing.
 */
void vescUSB::comm_init(char* modemDevice) {
	struct termios newtio;

	bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

	/* BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
	   CS8     : 8n1 (8bit,no parity,1 stopbit)
	   CLOCAL  : local connection, no modem contol
	   CREAD   : enable receiving characters */
	newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;

	/* IGNPAR  : ignore bytes with parity errors
	   otherwise make device raw (no other input processing) */
	newtio.c_iflag = IGNPAR;

	/*  Raw output  */
	newtio.c_oflag = 0;

	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 1;

	/* ICANON  : enable canonical input
	   disable all echo functionality, and don't send signals to calling program */
	newtio.c_lflag = 0;

	// Load the pin configuration
	/* Open modem device for reading and writing and not as controlling tty
	because we don't want to get killed if linenoise sends CTRL-C. */
	fd = open(modemDevice, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0) { perror(modemDevice); exit(-1); }

	/* now clean the modem line and activate the settings for the port */
	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &newtio);

	// Initialize the bldc interface and provide a send function
	// MAIN SEND FUNC ASSIGNMENT
	//bldc_interface_uart_init(send_packet);
	// Initialize packet handler
	//packet_init(send_packet, bldc_interface_process_packet, PACKET_HANDLER);
	//handler_states[PACKET_HANDLER].send_func = &vescUSB::send_packet;
	//handler_states[PACKET_HANDLER].process_func = &vescUSB::bldc_interface_process_packet;

	// Initialize the bldc interface and provide a send function
	//bldc_interface_init(send_packet_bldc_interface);
	//bldc_interface_init(packet_send_packet);
	//can_fwd_vesc = -1;
	//send_func = &vescUSB::packet_send_packet;
}

/*
 * Call this function at end of main program to close
 * serial port file descriptor.
 */
void vescUSB::comm_close(void) {
	close(fd);
}

/******************** interface ********************/

void vescUSB::bldc_interface_init(void(vescUSB::*func)(unsigned char *data, unsigned int len)) {
	send_func = func;
}

void vescUSB::bldc_interface_set_forward_func(void(*func)(unsigned char *data, unsigned int len)) {
	forward_func = func;
}

/**
 * Send a packet using the set send function.
 *
 * @param data
 * The packet data.
 *
 * @param len
 * The data length.
 */
void vescUSB::bldc_interface_send_packet(unsigned char *data, unsigned int len) {
	/*
	if (send_func) {
		send_func(data, len);
	}
	*/
	if (1) {
		packet_send_packet(data, len, PACKET_HANDLER);
	}
}

/**
 * Process a received buffer with commands and data.
 *
 * @param data
 * The buffer to process.
 *
 * @param len
 * The length of the buffer.
 */
void vescUSB::bldc_interface_process_packet(unsigned char *data, unsigned int len) {
	if (!len) {
		return;
	}

	if (forward_func) {
		forward_func(data, len);
		return;
	}

	int32_t ind = 0;
	int i = 0;
	unsigned char id = data[0];
	data++;
	len--;

	switch (id) {
	case COMM_FW_VERSION:
		if (len == 2) {
			ind = 0;
			fw_major = data[ind++];
			fw_minor = data[ind++];
		}
		else {
			fw_major = -1;
			fw_minor = -1;
		}
		break;

	case COMM_ERASE_NEW_APP:
	case COMM_WRITE_NEW_APP_DATA:
		// TODO
		break;

	case COMM_GET_VALUES:
        values_read = true;
		ind = 0;
		//values.temp_mos = buffer_get_float16(data, 1e1, &ind);
		values.temp_pcb = buffer_get_float16(data, 1e1, &ind);
		values.temp_motor = buffer_get_float16(data, 1e1, &ind);
		values.current_motor = buffer_get_float32(data, 1e2, &ind);
		values.current_in = buffer_get_float32(data, 1e2, &ind);
		values.id = buffer_get_float32(data, 1e2, &ind);
		values.iq = buffer_get_float32(data, 1e2, &ind);
		values.duty_now = buffer_get_float16(data, 1e3, &ind);
		values.rpm = buffer_get_float32(data, 1e0, &ind);
		values.v_in = buffer_get_float16(data, 1e1, &ind);
		values.amp_hours = buffer_get_float32(data, 1e4, &ind);
		values.amp_hours_charged = buffer_get_float32(data, 1e4, &ind);
		values.watt_hours = buffer_get_float32(data, 1e4, &ind);
		values.watt_hours_charged = buffer_get_float32(data, 1e4, &ind);
		values.tachometer = buffer_get_int32(data, &ind);
		values.tachometer_abs = buffer_get_int32(data, &ind);
		values.fault_code = (mc_fault_code)data[ind++];

		if (ind < (int)len) {
			values.pid_pos = buffer_get_float32(data, 1e6, &ind);
		}
		else {
			values.pid_pos = 0.0;
		}

		if (ind < (int)len) {
			values.vesc_id = data[ind++];
		}
		else {
			values.vesc_id = 255;
		}

		if (ind < (int)len) {
			values.temp_mos1 = buffer_get_float16(data, 1e1, &ind);
			values.temp_mos2 = buffer_get_float16(data, 1e1, &ind);
			values.temp_mos3 = buffer_get_float16(data, 1e1, &ind);
		}
		else {
			values.temp_mos1 = 0;
			values.temp_mos2 = 0;
			values.temp_mos3 = 0;
		}

		if (rx_value_func) {
			rx_value_func(&values);
		}
		break;

	case COMM_PRINT:
		if (rx_printf_func) {
			data[len] = '\0';
			rx_printf_func((char*)data);
		}
		break;

	case COMM_SAMPLE_PRINT:
		// TODO
		break;

	case COMM_ROTOR_POSITION:
		ind = 0;
		rotor_pos = buffer_get_float32(data, 100000.0, &ind);

		if (rx_rotor_pos_func) {
			rx_rotor_pos_func(rotor_pos);
		}
		break;

	case COMM_EXPERIMENT_SAMPLE:
		// TODO
		break;

	case COMM_GET_MCCONF: //COMM_GET_MCCONF:
	case COMM_GET_MCCONF_DEFAULT://COMM_GET_MCCONF_DEFAULT:
		ind = 0;
		mcconf_read = confgenerator_deserialize_mcconf(data, &mcconf);

		
		/*
		ind = 0;
		mcconf.pwm_mode = data[ind++];
		mcconf.comm_mode = data[ind++];
		mcconf.motor_type = data[ind++];
		mcconf.sensor_mode = data[ind++];

		mcconf.l_current_max = buffer_get_float32_auto(data, &ind);
		mcconf.l_current_min = buffer_get_float32_auto(data, &ind);
		mcconf.l_in_current_max = buffer_get_float32_auto(data, &ind);
		mcconf.l_in_current_min = buffer_get_float32_auto(data, &ind);
		mcconf.l_abs_current_max = buffer_get_float32_auto(data, &ind);
		mcconf.l_min_erpm = buffer_get_float32_auto(data, &ind);
		mcconf.l_max_erpm = buffer_get_float32_auto(data, &ind);
		mcconf.l_erpm_start = buffer_get_float32_auto(data, &ind);
		mcconf.l_max_erpm_fbrake = buffer_get_float32_auto(data, &ind);
		mcconf.l_max_erpm_fbrake_cc = buffer_get_float32_auto(data, &ind);
		mcconf.l_min_vin = buffer_get_float32_auto(data, &ind);
		mcconf.l_max_vin = buffer_get_float32_auto(data, &ind);
		mcconf.l_battery_cut_start = buffer_get_float32_auto(data, &ind);
		mcconf.l_battery_cut_end = buffer_get_float32_auto(data, &ind);
		mcconf.l_slow_abs_current = data[ind++];
		mcconf.l_temp_fet_start = buffer_get_float32_auto(data, &ind);
		mcconf.l_temp_fet_end = buffer_get_float32_auto(data, &ind);
		mcconf.l_temp_motor_start = buffer_get_float32_auto(data, &ind);
		mcconf.l_temp_motor_end = buffer_get_float32_auto(data, &ind);
		mcconf.l_temp_accel_dec = buffer_get_float32_auto(data, &ind);
		mcconf.l_min_duty = buffer_get_float32_auto(data, &ind);
		mcconf.l_max_duty = buffer_get_float32_auto(data, &ind);
		mcconf.l_watt_max = buffer_get_float32_auto(data, &ind);
		mcconf.l_watt_min = buffer_get_float32_auto(data, &ind);

		mcconf.lo_current_max = mcconf.l_current_max;
		mcconf.lo_current_min = mcconf.l_current_min;
		mcconf.lo_in_current_max = mcconf.l_in_current_max;
		mcconf.lo_in_current_min = mcconf.l_in_current_min;
		mcconf.lo_current_motor_max_now = mcconf.l_current_max;
		mcconf.lo_current_motor_min_now = mcconf.l_current_min;

		mcconf.sl_min_erpm = buffer_get_float32_auto(data, &ind);
		mcconf.sl_min_erpm_cycle_int_limit = buffer_get_float32_auto(data, &ind);
		mcconf.sl_max_fullbreak_current_dir_change = buffer_get_float32_auto(data, &ind);
		mcconf.sl_cycle_int_limit = buffer_get_float32_auto(data, &ind);
		mcconf.sl_phase_advance_at_br = buffer_get_float32_auto(data, &ind);
		mcconf.sl_cycle_int_rpm_br = buffer_get_float32_auto(data, &ind);
		mcconf.sl_bemf_coupling_k = buffer_get_float32_auto(data, &ind);

		memcpy(mcconf.hall_table, data + ind, 8);
		ind += 8;
		mcconf.hall_sl_erpm = buffer_get_float32_auto(data, &ind);

		mcconf.foc_current_kp = buffer_get_float32_auto(data, &ind);
		mcconf.foc_current_ki = buffer_get_float32_auto(data, &ind);
		mcconf.foc_f_sw = buffer_get_float32_auto(data, &ind);
		mcconf.foc_dt_us = buffer_get_float32_auto(data, &ind);
		mcconf.foc_encoder_inverted = data[ind++];
		mcconf.foc_encoder_offset = buffer_get_float32_auto(data, &ind);
		mcconf.foc_encoder_ratio = buffer_get_float32_auto(data, &ind);
		mcconf.foc_sensor_mode = data[ind++];
		mcconf.foc_pll_kp = buffer_get_float32_auto(data, &ind);
		mcconf.foc_pll_ki = buffer_get_float32_auto(data, &ind);
		mcconf.foc_motor_l = buffer_get_float32_auto(data, &ind);
		mcconf.foc_motor_r = buffer_get_float32_auto(data, &ind);
		mcconf.foc_motor_flux_linkage = buffer_get_float32_auto(data, &ind);
		mcconf.foc_observer_gain = buffer_get_float32_auto(data, &ind);
		mcconf.foc_observer_gain_slow = buffer_get_float32_auto(data, &ind);
		mcconf.foc_duty_dowmramp_kp = buffer_get_float32_auto(data, &ind);
		mcconf.foc_duty_dowmramp_ki = buffer_get_float32_auto(data, &ind);
		mcconf.foc_openloop_rpm = buffer_get_float32_auto(data, &ind);
		mcconf.foc_sl_openloop_hyst = buffer_get_float32_auto(data, &ind);
		mcconf.foc_sl_openloop_time = buffer_get_float32_auto(data, &ind);
		mcconf.foc_sl_d_current_duty = buffer_get_float32_auto(data, &ind);
		mcconf.foc_sl_d_current_factor = buffer_get_float32_auto(data, &ind);
		memcpy(mcconf.foc_hall_table, data + ind, 8);
		ind += 8;
		mcconf.foc_sl_erpm = buffer_get_float32_auto(data, &ind);
		mcconf.foc_sample_v0_v7 = data[ind++];
		mcconf.foc_sample_high_current = data[ind++];
		mcconf.foc_sat_comp = buffer_get_float32_auto(data, &ind);
		mcconf.foc_temp_comp = data[ind++];
		mcconf.foc_temp_comp_base_temp = buffer_get_float32_auto(data, &ind);
		mcconf.foc_current_filter_const = buffer_get_float32_auto(data, &ind);

		mcconf.s_pid_kp = buffer_get_float32_auto(data, &ind);
		mcconf.s_pid_ki = buffer_get_float32_auto(data, &ind);
		mcconf.s_pid_kd = buffer_get_float32_auto(data, &ind);
		mcconf.s_pid_kd_filter = buffer_get_float32_auto(data, &ind);
		mcconf.s_pid_min_erpm = buffer_get_float32_auto(data, &ind);
		mcconf.s_pid_allow_braking = data[ind++];

		mcconf.p_pid_kp = buffer_get_float32_auto(data, &ind);
		mcconf.p_pid_ki = buffer_get_float32_auto(data, &ind);
		mcconf.p_pid_kd = buffer_get_float32_auto(data, &ind);
		mcconf.p_pid_kd_filter = buffer_get_float32_auto(data, &ind);
		mcconf.p_pid_ang_div = buffer_get_float32_auto(data, &ind);

		mcconf.cc_startup_boost_duty = buffer_get_float32_auto(data, &ind);
		mcconf.cc_min_current = buffer_get_float32_auto(data, &ind);
		mcconf.cc_gain = buffer_get_float32_auto(data, &ind);
		mcconf.cc_ramp_step_max = buffer_get_float32_auto(data, &ind);

		mcconf.m_fault_stop_time_ms = buffer_get_int32(data, &ind);
		mcconf.m_duty_ramp_step = buffer_get_float32_auto(data, &ind);
		mcconf.m_current_backoff_gain = buffer_get_float32_auto(data, &ind);
		mcconf.m_encoder_counts = buffer_get_uint32(data, &ind);
		mcconf.m_sensor_port_mode = data[ind++];
		mcconf.m_invert_direction = data[ind++];
		mcconf.m_drv8301_oc_mode = data[ind++];
		mcconf.m_drv8301_oc_adj = data[ind++];
		mcconf.m_bldc_f_sw_min = buffer_get_float32_auto(data, &ind);
		mcconf.m_bldc_f_sw_max = buffer_get_float32_auto(data, &ind);
		mcconf.m_dc_f_sw = buffer_get_float32_auto(data, &ind);
		mcconf.m_ntc_motor_beta = buffer_get_float32_auto(data, &ind);
		mcconf.m_out_aux_mode = data[ind++];

		if (rx_mcconf_func) {
			rx_mcconf_func(&mcconf);
		}
		*/
		break;
	case COMM_GET_APPCONF:
	case COMM_GET_APPCONF_DEFAULT:
		ind = 0;
		appconf.controller_id = data[ind++];
		appconf.timeout_msec = buffer_get_uint32(data, &ind);
		appconf.timeout_brake_current = buffer_get_float32_auto(data, &ind);
		appconf.send_can_status = data[ind++];
		appconf.send_can_status_rate_hz = buffer_get_uint16(data, &ind);
		appconf.can_baud_rate = data[ind++];

		appconf.app_to_use = data[ind++];

		appconf.app_ppm_conf.ctrl_type = data[ind++];
		appconf.app_ppm_conf.pid_max_erpm = buffer_get_float32_auto(data, &ind);
		appconf.app_ppm_conf.hyst = buffer_get_float32_auto(data, &ind);
		appconf.app_ppm_conf.pulse_start = buffer_get_float32_auto(data, &ind);
		appconf.app_ppm_conf.pulse_end = buffer_get_float32_auto(data, &ind);
		appconf.app_ppm_conf.pulse_center = buffer_get_float32_auto(data, &ind);
		appconf.app_ppm_conf.median_filter = data[ind++];
		appconf.app_ppm_conf.safe_start = data[ind++];
		appconf.app_ppm_conf.throttle_exp = buffer_get_float32_auto(data, &ind);
		appconf.app_ppm_conf.throttle_exp_brake = buffer_get_float32_auto(data, &ind);
		appconf.app_ppm_conf.throttle_exp_mode = data[ind++];
		appconf.app_ppm_conf.ramp_time_pos = buffer_get_float32_auto(data, &ind);
		appconf.app_ppm_conf.ramp_time_neg = buffer_get_float32_auto(data, &ind);
		appconf.app_ppm_conf.multi_esc = data[ind++];
		appconf.app_ppm_conf.tc = data[ind++];
		appconf.app_ppm_conf.tc_max_diff = buffer_get_float32_auto(data, &ind);

		appconf.app_adc_conf.ctrl_type = data[ind++];
		appconf.app_adc_conf.hyst = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.voltage_start = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.voltage_end = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.voltage_center = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.voltage2_start = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.voltage2_end = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.use_filter = data[ind++];
		appconf.app_adc_conf.safe_start = data[ind++];
		appconf.app_adc_conf.cc_button_inverted = data[ind++];
		appconf.app_adc_conf.rev_button_inverted = data[ind++];
		appconf.app_adc_conf.voltage_inverted = data[ind++];
		appconf.app_adc_conf.voltage2_inverted = data[ind++];
		appconf.app_adc_conf.throttle_exp = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.throttle_exp_brake = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.throttle_exp_mode = data[ind++];
		appconf.app_adc_conf.ramp_time_pos = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.ramp_time_neg = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.multi_esc = data[ind++];
		appconf.app_adc_conf.tc = data[ind++];
		appconf.app_adc_conf.tc_max_diff = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.update_rate_hz = buffer_get_uint16(data, &ind);

		appconf.app_uart_baudrate = buffer_get_uint32(data, &ind);

		appconf.app_chuk_conf.ctrl_type = data[ind++];
		appconf.app_chuk_conf.hyst = buffer_get_float32_auto(data, &ind);
		appconf.app_chuk_conf.ramp_time_pos = buffer_get_float32_auto(data, &ind);
		appconf.app_chuk_conf.ramp_time_neg = buffer_get_float32_auto(data, &ind);
		appconf.app_chuk_conf.stick_erpm_per_s_in_cc = buffer_get_float32_auto(data, &ind);
		appconf.app_chuk_conf.throttle_exp = buffer_get_float32_auto(data, &ind);
		appconf.app_chuk_conf.throttle_exp_brake = buffer_get_float32_auto(data, &ind);
		appconf.app_chuk_conf.throttle_exp_mode = data[ind++];
		appconf.app_chuk_conf.multi_esc = data[ind++];
		appconf.app_chuk_conf.tc = data[ind++];
		appconf.app_chuk_conf.tc_max_diff = buffer_get_float32_auto(data, &ind);

		appconf.app_nrf_conf.speed = data[ind++];
		appconf.app_nrf_conf.power = data[ind++];
		appconf.app_nrf_conf.crc_type = data[ind++];
		appconf.app_nrf_conf.retry_delay = data[ind++];
		appconf.app_nrf_conf.retries = data[ind++];
		appconf.app_nrf_conf.channel = data[ind++];
		memcpy(appconf.app_nrf_conf.address, data + ind, 3);
		ind += 3;
		appconf.app_nrf_conf.send_crc_ack = data[ind++];

		if (rx_appconf_func) {
			rx_appconf_func(&appconf);
		}
		break;

	case COMM_DETECT_MOTOR_PARAM:
		ind = 0;
		detect_cycle_int_limit = buffer_get_float32(data, 1000.0, &ind);
		detect_coupling_k = buffer_get_float32(data, 1000.0, &ind);
		for (i = 0; i < 8; i++) {
			detect_hall_table[i] = (const signed char)(data[ind++]);
		}
		detect_hall_res = (const signed char)(data[ind++]);

		if (rx_detect_func) {
			rx_detect_func(detect_cycle_int_limit, detect_coupling_k,
				detect_hall_table, detect_hall_res);
		}
		break;

	case COMM_DETECT_MOTOR_R_L: {
		// TODO!
	} break;

	case COMM_DETECT_MOTOR_FLUX_LINKAGE: {
		// TODO!
	} break;

	case COMM_DETECT_ENCODER: {
		// TODO!
	} break;

	case COMM_DETECT_HALL_FOC: {
		// TODO!
	} break;

	case COMM_GET_DECODED_PPM:
		ind = 0;
		dec_ppm = buffer_get_float32(data, 1000000.0, &ind);
		dec_ppm_len = buffer_get_float32(data, 1000000.0, &ind);

		if (rx_dec_ppm_func) {
			rx_dec_ppm_func(dec_ppm, dec_ppm_len);
		}
		break;

	case COMM_GET_DECODED_ADC:
		ind = 0;
		dec_adc = buffer_get_float32(data, 1000000.0, &ind);
		dec_adc_voltage = buffer_get_float32(data, 1000000.0, &ind);
		// TODO for adc2

		if (rx_dec_adc_func) {
			rx_dec_adc_func(dec_adc, dec_adc_voltage);
		}
		break;

	case COMM_GET_DECODED_CHUK:
		ind = 0;
		dec_chuk = buffer_get_float32(data, 1000000.0, &ind);

		if (rx_dec_chuk_func) {
			rx_dec_chuk_func(dec_chuk);
		}
		break;

	case COMM_SET_MCCONF:
		// This is a confirmation that the new mcconf is received.
		if (rx_mcconf_received_func) {
			rx_mcconf_received_func();
		}
		break;

	case COMM_SET_APPCONF:
		// This is a confirmation that the new appconf is received.
		if (rx_appconf_received_func) {
			rx_appconf_received_func();
		}
		break;

	default:
		break;
	}
}

/**
 * Function pointer setters. When data that is requested with the get functions
 * is received, the corresponding function pointer will be called with the
 * received data.
 *
 * @param func
 * A function to be called when the corresponding data is received.
 */

void vescUSB::bldc_interface_set_rx_value_func(void(*func)(mc_values *values)) {
	rx_value_func = func;
}

void vescUSB::bldc_interface_set_rx_printf_func(void(*func)(char *str)) {
	rx_printf_func = func;
}

void vescUSB::bldc_interface_set_rx_fw_func(void(*func)(int major, int minor)) {
	rx_fw_func = func;
}

void vescUSB::bldc_interface_set_rx_rotor_pos_func(void(*func)(float pos)) {
	rx_rotor_pos_func = func;
}

void vescUSB::bldc_interface_set_rx_mcconf_func(void(*func)(mc_configuration *conf)) {
	rx_mcconf_func = func;
}

void vescUSB::bldc_interface_set_rx_appconf_func(void(*func)(app_configuration *conf)) {
	rx_appconf_func = func;
}

void vescUSB::bldc_interface_set_rx_detect_func(void(*func)(float cycle_int_limit, float coupling_k,
	const signed char *hall_table, signed char hall_res)) {
	rx_detect_func = func;
}

void vescUSB::bldc_interface_set_rx_dec_ppm_func(void(*func)(float val, float ms)) {
	rx_dec_ppm_func = func;
}

void vescUSB::bldc_interface_set_rx_dec_adc_func(void(*func)(float val, float voltage)) {
	rx_dec_adc_func = func;
}

void vescUSB::bldc_interface_set_rx_dec_chuk_func(void(*func)(float val)) {
	rx_dec_chuk_func = func;
}

void vescUSB::bldc_interface_set_rx_mcconf_received_func(void(*func)(void)) {
	rx_mcconf_received_func = func;
}

void vescUSB::bldc_interface_set_rx_appconf_received_func(void(*func)(void)) {
	rx_appconf_received_func = func;
}

void vescUSB::bldc_interface_set_sim_control_function(void(*func)(motor_control_mode mode, float value)) {
	motor_control_set_func = func;
}

void vescUSB::bldc_interface_set_sim_values_func(void(*func)(void)) {
	values_requested_func = func;
}

// Setters
void vescUSB::bldc_interface_terminal_cmd(char* cmd) {
	int len = strlen(cmd);
	send_buffer[0] = COMM_TERMINAL_CMD;
	memcpy(send_buffer + 1, cmd, len);
	send_packet_no_fwd(send_buffer, len + 1);
}

void vescUSB::bldc_interface_set_duty_cycle(float dutyCycle) {
	if (motor_control_set_func) {
		motor_control_set_func(MOTOR_CONTROL_DUTY, dutyCycle);
		return;
	}
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_SET_DUTY;
	buffer_append_float32(send_buffer, dutyCycle, 100000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void vescUSB::bldc_interface_set_current(float current) {
	if (motor_control_set_func) {
		motor_control_set_func(MOTOR_CONTROL_CURRENT, current);
		return;
	}
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_SET_CURRENT;
	buffer_append_float32(send_buffer, current, 1000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void vescUSB::bldc_interface_set_current_brake(float current) {
	if (motor_control_set_func) {
		motor_control_set_func(MOTOR_CONTROL_CURRENT_BRAKE, current);
		return;
	}
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_float32(send_buffer, current, 1000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void vescUSB::bldc_interface_set_rpm(int rpm) {
	if (motor_control_set_func) {
		motor_control_set_func(MOTOR_CONTROL_RPM, rpm);
		return;
	}
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_SET_RPM;
	buffer_append_int32(send_buffer, rpm, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void vescUSB::bldc_interface_set_pos(float pos) {
	if (motor_control_set_func) {
		motor_control_set_func(MOTOR_CONTROL_POS, pos);
		return;
	}
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_SET_POS;
	buffer_append_float32(send_buffer, pos, 1000000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void vescUSB::bldc_interface_set_handbrake(float current) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_SET_HANDBRAKE;
	buffer_append_float32(send_buffer, current, 1e3, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void vescUSB::bldc_interface_set_servo_pos(float pos) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_SET_SERVO_POS;
	buffer_append_float16(send_buffer, pos, 1000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void vescUSB::bldc_interface_set_mcconf(const mc_configuration *mcconf) {
	int32_t ind = 0;
	send_buffer[ind++] = COMM_SET_MCCONF;

	ind += confgenerator_serialize_mcconf(send_buffer+1, mcconf);

	/*
	int32_t ind = 0;
	send_buffer[ind++] = COMM_SET_MCCONF;

	send_buffer[ind++] = mcconf->pwm_mode;
	send_buffer[ind++] = mcconf->comm_mode;
	send_buffer[ind++] = mcconf->motor_type;
	send_buffer[ind++] = mcconf->sensor_mode;

	buffer_append_float32_auto(send_buffer, mcconf->l_current_max, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_current_min, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_in_current_max, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_in_current_min, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_abs_current_max, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_min_erpm, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_max_erpm, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_erpm_start, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_max_erpm_fbrake, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_max_erpm_fbrake_cc, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_min_vin, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_max_vin, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_battery_cut_start, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_battery_cut_end, &ind);
	send_buffer[ind++] = mcconf->l_slow_abs_current;
	buffer_append_float32_auto(send_buffer, mcconf->l_temp_fet_start, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_temp_fet_end, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_temp_motor_start, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_temp_motor_end, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_temp_accel_dec, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_min_duty, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_max_duty, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_watt_max, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_watt_min, &ind);

	buffer_append_float32_auto(send_buffer, mcconf->sl_min_erpm, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->sl_min_erpm_cycle_int_limit, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->sl_max_fullbreak_current_dir_change, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->sl_cycle_int_limit, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->sl_phase_advance_at_br, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->sl_cycle_int_rpm_br, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->sl_bemf_coupling_k, &ind);

	memcpy(send_buffer + ind, mcconf->hall_table, 8);
	ind += 8;
	buffer_append_float32_auto(send_buffer, mcconf->hall_sl_erpm, &ind);

	buffer_append_float32_auto(send_buffer, mcconf->foc_current_kp, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_current_ki, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_f_sw, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_dt_us, &ind);
	send_buffer[ind++] = mcconf->foc_encoder_inverted;
	buffer_append_float32_auto(send_buffer, mcconf->foc_encoder_offset, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_encoder_ratio, &ind);
	send_buffer[ind++] = mcconf->foc_sensor_mode;
	buffer_append_float32_auto(send_buffer, mcconf->foc_pll_kp, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_pll_ki, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_motor_l, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_motor_r, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_motor_flux_linkage, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_observer_gain, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_observer_gain_slow, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_duty_dowmramp_kp, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_duty_dowmramp_ki, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_openloop_rpm, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_sl_openloop_hyst, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_sl_openloop_time, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_sl_d_current_duty, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_sl_d_current_factor, &ind);
	memcpy(send_buffer + ind, mcconf->foc_hall_table, 8);
	ind += 8;
	buffer_append_float32_auto(send_buffer, mcconf->foc_sl_erpm, &ind);
	send_buffer[ind++] = mcconf->foc_sample_v0_v7;
	send_buffer[ind++] = mcconf->foc_sample_high_current;
	buffer_append_float32_auto(send_buffer, mcconf->foc_sat_comp, &ind);
	send_buffer[ind++] = mcconf->foc_temp_comp;
	buffer_append_float32_auto(send_buffer, mcconf->foc_temp_comp_base_temp, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_current_filter_const, &ind);

	buffer_append_float32_auto(send_buffer, mcconf->s_pid_kp, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->s_pid_ki, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->s_pid_kd, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->s_pid_kd_filter, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->s_pid_min_erpm, &ind);
	send_buffer[ind++] = mcconf->s_pid_allow_braking;

	buffer_append_float32_auto(send_buffer, mcconf->p_pid_kp, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->p_pid_ki, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->p_pid_kd, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->p_pid_kd_filter, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->p_pid_ang_div, &ind);

	buffer_append_float32_auto(send_buffer, mcconf->cc_startup_boost_duty, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->cc_min_current, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->cc_gain, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->cc_ramp_step_max, &ind);

	buffer_append_int32(send_buffer, mcconf->m_fault_stop_time_ms, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->m_duty_ramp_step, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->m_current_backoff_gain, &ind);
	buffer_append_uint32(send_buffer, mcconf->m_encoder_counts, &ind);
	send_buffer[ind++] = mcconf->m_sensor_port_mode;
	send_buffer[ind++] = mcconf->m_invert_direction;
	send_buffer[ind++] = mcconf->m_drv8301_oc_mode;
	send_buffer[ind++] = mcconf->m_drv8301_oc_adj;
	buffer_append_float32_auto(send_buffer, mcconf->m_bldc_f_sw_min, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->m_bldc_f_sw_max, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->m_dc_f_sw, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->m_ntc_motor_beta, &ind);
	send_buffer[ind++] = mcconf->m_out_aux_mode;
	*/
	send_packet_no_fwd(send_buffer, ind);

}

void vescUSB::bldc_interface_set_appconf(const app_configuration *appconf) {
	int32_t ind = 0;
	send_buffer[ind++] = COMM_SET_APPCONF;
	send_buffer[ind++] = appconf->controller_id;
	buffer_append_uint32(send_buffer, appconf->timeout_msec, &ind);
	buffer_append_float32_auto(send_buffer, appconf->timeout_brake_current, &ind);
	send_buffer[ind++] = appconf->send_can_status;
	buffer_append_uint16(send_buffer, appconf->send_can_status_rate_hz, &ind);
	send_buffer[ind++] = appconf->can_baud_rate;

	send_buffer[ind++] = appconf->app_to_use;

	send_buffer[ind++] = appconf->app_ppm_conf.ctrl_type;
	buffer_append_float32_auto(send_buffer, appconf->app_ppm_conf.pid_max_erpm, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_ppm_conf.hyst, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_ppm_conf.pulse_start, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_ppm_conf.pulse_end, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_ppm_conf.pulse_center, &ind);
	send_buffer[ind++] = appconf->app_ppm_conf.median_filter;
	send_buffer[ind++] = appconf->app_ppm_conf.safe_start;
	buffer_append_float32_auto(send_buffer, appconf->app_ppm_conf.throttle_exp, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_ppm_conf.throttle_exp_brake, &ind);
	send_buffer[ind++] = appconf->app_ppm_conf.throttle_exp_mode;
	buffer_append_float32_auto(send_buffer, appconf->app_ppm_conf.ramp_time_pos, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_ppm_conf.ramp_time_neg, &ind);
	send_buffer[ind++] = appconf->app_ppm_conf.multi_esc;
	send_buffer[ind++] = appconf->app_ppm_conf.tc;
	buffer_append_float32_auto(send_buffer, appconf->app_ppm_conf.tc_max_diff, &ind);

	send_buffer[ind++] = appconf->app_adc_conf.ctrl_type;
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.hyst, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.voltage_start, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.voltage_end, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.voltage_center, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.voltage2_start, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.voltage2_end, &ind);
	send_buffer[ind++] = appconf->app_adc_conf.use_filter;
	send_buffer[ind++] = appconf->app_adc_conf.safe_start;
	send_buffer[ind++] = appconf->app_adc_conf.cc_button_inverted;
	send_buffer[ind++] = appconf->app_adc_conf.rev_button_inverted;
	send_buffer[ind++] = appconf->app_adc_conf.voltage_inverted;
	send_buffer[ind++] = appconf->app_adc_conf.voltage2_inverted;
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.throttle_exp, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.throttle_exp_brake, &ind);
	send_buffer[ind++] = appconf->app_adc_conf.throttle_exp_mode;
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.ramp_time_pos, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.ramp_time_neg, &ind);
	send_buffer[ind++] = appconf->app_adc_conf.multi_esc;
	send_buffer[ind++] = appconf->app_adc_conf.tc;
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.tc_max_diff, &ind);
	buffer_append_uint16(send_buffer, appconf->app_adc_conf.update_rate_hz, &ind);

	buffer_append_uint32(send_buffer, appconf->app_uart_baudrate, &ind);

	send_buffer[ind++] = appconf->app_chuk_conf.ctrl_type;
	buffer_append_float32_auto(send_buffer, appconf->app_chuk_conf.hyst, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_chuk_conf.ramp_time_pos, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_chuk_conf.ramp_time_neg, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_chuk_conf.stick_erpm_per_s_in_cc, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_chuk_conf.throttle_exp, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_chuk_conf.throttle_exp_brake, &ind);
	send_buffer[ind++] = appconf->app_chuk_conf.throttle_exp_mode;
	send_buffer[ind++] = appconf->app_chuk_conf.multi_esc;
	send_buffer[ind++] = appconf->app_chuk_conf.tc;
	buffer_append_float32_auto(send_buffer, appconf->app_chuk_conf.tc_max_diff, &ind);

	send_buffer[ind++] = appconf->app_nrf_conf.speed;
	send_buffer[ind++] = appconf->app_nrf_conf.power;
	send_buffer[ind++] = appconf->app_nrf_conf.crc_type;
	send_buffer[ind++] = appconf->app_nrf_conf.retry_delay;
	send_buffer[ind++] = appconf->app_nrf_conf.retries;
	send_buffer[ind++] = appconf->app_nrf_conf.channel;
	memcpy(send_buffer + ind, appconf->app_nrf_conf.address, 3);
	ind += 3;
	send_buffer[ind++] = appconf->app_nrf_conf.send_crc_ack;

	send_packet_no_fwd(send_buffer, ind);
}

// Getters
void vescUSB::bldc_interface_get_fw_version(void) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_FW_VERSION;
	send_packet_no_fwd(send_buffer, send_index);
}

void vescUSB::bldc_interface_get_values(void) {
	if (values_requested_func) {
		values_requested_func();
		return;
	}
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_GET_VALUES;
	send_packet_no_fwd(send_buffer, send_index);
}

void vescUSB::bldc_interface_get_mcconf(void) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_GET_MCCONF;
	send_packet_no_fwd(send_buffer, send_index);
}

void vescUSB::bldc_interface_get_appconf(void) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_GET_APPCONF;
	send_packet_no_fwd(send_buffer, send_index);
}

void vescUSB::bldc_interface_get_decoded_ppm(void) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_GET_DECODED_PPM;
	send_packet_no_fwd(send_buffer, send_index);
}

void vescUSB::bldc_interface_get_decoded_adc(void) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_GET_DECODED_ADC;
	send_packet_no_fwd(send_buffer, send_index);
}

void vescUSB::bldc_interface_get_decoded_chuk(void) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_GET_DECODED_CHUK;
	send_packet_no_fwd(send_buffer, send_index);
}

// Other functions
void vescUSB::bldc_interface_detect_motor_param(float current, float min_rpm, float low_duty) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_DETECT_MOTOR_PARAM;
	buffer_append_float32(send_buffer, current, 1000.0, &send_index);
	buffer_append_float32(send_buffer, min_rpm, 1000.0, &send_index);
	buffer_append_float32(send_buffer, low_duty, 1000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void vescUSB::bldc_interface_reboot(void) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_REBOOT;
	send_packet_no_fwd(send_buffer, send_index);
}

void vescUSB::bldc_interface_send_alive(void) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_ALIVE;
	send_packet_no_fwd(send_buffer, send_index);
}

void vescUSB::send_values_to_receiver(mc_values *values) {
	if (rx_value_func) {
		rx_value_func(values);
	}
}

// Helpers
const char* vescUSB::bldc_interface_fault_to_string(mc_fault_code fault) {
	switch (fault) {
	case FAULT_CODE_NONE: return "FAULT_CODE_NONE";
	case FAULT_CODE_OVER_VOLTAGE: return "FAULT_CODE_OVER_VOLTAGE";
	case FAULT_CODE_UNDER_VOLTAGE: return "FAULT_CODE_UNDER_VOLTAGE";
	case FAULT_CODE_DRV: return "FAULT_CODE_DRV";
	case FAULT_CODE_ABS_OVER_CURRENT: return "FAULT_CODE_ABS_OVER_CURRENT";
	case FAULT_CODE_OVER_TEMP_FET: return "FAULT_CODE_OVER_TEMP_FET";
	case FAULT_CODE_OVER_TEMP_MOTOR: return "FAULT_CODE_OVER_TEMP_MOTOR";
	default: return "Unknown fault";
	}
}

// Private functions
void vescUSB::send_packet_no_fwd(unsigned char *data, unsigned int len) {
	if (!forward_func) {
		bldc_interface_send_packet(data, len);
	}
}

// confgen
int32_t vescUSB::confgenerator_serialize_mcconf(uint8_t *buffer, const mc_configuration *conf) {
	int32_t ind = 0;

	buffer_append_uint32(buffer, MCCONF_SIGNATURE, &ind);

	buffer[ind++] = conf->pwm_mode;
	buffer[ind++] = conf->comm_mode;
	buffer[ind++] = conf->motor_type;
	buffer[ind++] = conf->sensor_mode;
	buffer_append_float32_auto(buffer, conf->l_current_max, &ind);
	buffer_append_float32_auto(buffer, conf->l_current_min, &ind);
	buffer_append_float32_auto(buffer, conf->l_in_current_max, &ind);
	buffer_append_float32_auto(buffer, conf->l_in_current_min, &ind);
	buffer_append_float32_auto(buffer, conf->l_abs_current_max, &ind);
	buffer_append_float32_auto(buffer, conf->l_min_erpm, &ind);
	buffer_append_float32_auto(buffer, conf->l_max_erpm, &ind);
	buffer_append_float32_auto(buffer, conf->l_erpm_start, &ind);
	buffer_append_float32_auto(buffer, conf->l_max_erpm_fbrake, &ind);
	buffer_append_float32_auto(buffer, conf->l_max_erpm_fbrake_cc, &ind);
	buffer_append_float32_auto(buffer, conf->l_min_vin, &ind);
	buffer_append_float32_auto(buffer, conf->l_max_vin, &ind);
	buffer_append_float32_auto(buffer, conf->l_battery_cut_start, &ind);
	buffer_append_float32_auto(buffer, conf->l_battery_cut_end, &ind);
	buffer[ind++] = conf->l_slow_abs_current;
	buffer_append_float32_auto(buffer, conf->l_temp_fet_start, &ind);
	buffer_append_float32_auto(buffer, conf->l_temp_fet_end, &ind);
	buffer_append_float32_auto(buffer, conf->l_temp_motor_start, &ind);
	buffer_append_float32_auto(buffer, conf->l_temp_motor_end, &ind);
	buffer_append_float32_auto(buffer, conf->l_temp_accel_dec, &ind);
	buffer_append_float32_auto(buffer, conf->l_min_duty, &ind);
	buffer_append_float32_auto(buffer, conf->l_max_duty, &ind);
	buffer_append_float32_auto(buffer, conf->l_watt_max, &ind);
	buffer_append_float32_auto(buffer, conf->l_watt_min, &ind);
	buffer_append_float32_auto(buffer, conf->l_current_max_scale, &ind);
	buffer_append_float32_auto(buffer, conf->l_current_min_scale, &ind);
	buffer_append_float32_auto(buffer, conf->sl_min_erpm, &ind);
	buffer_append_float32_auto(buffer, conf->sl_min_erpm_cycle_int_limit, &ind);
	buffer_append_float32_auto(buffer, conf->sl_max_fullbreak_current_dir_change, &ind);
	buffer_append_float32_auto(buffer, conf->sl_cycle_int_limit, &ind);
	buffer_append_float32_auto(buffer, conf->sl_phase_advance_at_br, &ind);
	buffer_append_float32_auto(buffer, conf->sl_cycle_int_rpm_br, &ind);
	buffer_append_float32_auto(buffer, conf->sl_bemf_coupling_k, &ind);
	buffer[ind++] = (uint8_t)conf->hall_table[0];
	buffer[ind++] = (uint8_t)conf->hall_table[1];
	buffer[ind++] = (uint8_t)conf->hall_table[2];
	buffer[ind++] = (uint8_t)conf->hall_table[3];
	buffer[ind++] = (uint8_t)conf->hall_table[4];
	buffer[ind++] = (uint8_t)conf->hall_table[5];
	buffer[ind++] = (uint8_t)conf->hall_table[6];
	buffer[ind++] = (uint8_t)conf->hall_table[7];
	buffer_append_float32_auto(buffer, conf->hall_sl_erpm, &ind);
	buffer_append_float32_auto(buffer, conf->foc_current_kp, &ind);
	buffer_append_float32_auto(buffer, conf->foc_current_ki, &ind);
	buffer_append_float32_auto(buffer, conf->foc_f_sw, &ind);
	buffer_append_float32_auto(buffer, conf->foc_dt_us, &ind);
	buffer[ind++] = conf->foc_encoder_inverted;
	buffer_append_float32_auto(buffer, conf->foc_encoder_offset, &ind);
	buffer_append_float32_auto(buffer, conf->foc_encoder_ratio, &ind);
	buffer[ind++] = conf->foc_sensor_mode;
	buffer_append_float32_auto(buffer, conf->foc_pll_kp, &ind);
	buffer_append_float32_auto(buffer, conf->foc_pll_ki, &ind);
	buffer_append_float32_auto(buffer, conf->foc_motor_l, &ind);
	buffer_append_float32_auto(buffer, conf->foc_motor_r, &ind);
	buffer_append_float32_auto(buffer, conf->foc_motor_flux_linkage, &ind);
	buffer_append_float32_auto(buffer, conf->foc_observer_gain, &ind);
	buffer_append_float32_auto(buffer, conf->foc_observer_gain_slow, &ind);
	buffer_append_float32_auto(buffer, conf->foc_duty_dowmramp_kp, &ind);
	buffer_append_float32_auto(buffer, conf->foc_duty_dowmramp_ki, &ind);
	buffer_append_float32_auto(buffer, conf->foc_openloop_rpm, &ind);
	buffer_append_float32_auto(buffer, conf->foc_sl_openloop_hyst, &ind);
	buffer_append_float32_auto(buffer, conf->foc_sl_openloop_time, &ind);
	buffer_append_float32_auto(buffer, conf->foc_sl_d_current_duty, &ind);
	buffer_append_float32_auto(buffer, conf->foc_sl_d_current_factor, &ind);
	buffer[ind++] = (uint8_t)conf->foc_hall_table[0];
	buffer[ind++] = (uint8_t)conf->foc_hall_table[1];
	buffer[ind++] = (uint8_t)conf->foc_hall_table[2];
	buffer[ind++] = (uint8_t)conf->foc_hall_table[3];
	buffer[ind++] = (uint8_t)conf->foc_hall_table[4];
	buffer[ind++] = (uint8_t)conf->foc_hall_table[5];
	buffer[ind++] = (uint8_t)conf->foc_hall_table[6];
	buffer[ind++] = (uint8_t)conf->foc_hall_table[7];
	buffer_append_float32_auto(buffer, conf->foc_sl_erpm, &ind);
	buffer[ind++] = conf->foc_sample_v0_v7;
	buffer[ind++] = conf->foc_sample_high_current;
	buffer_append_float32_auto(buffer, conf->foc_sat_comp, &ind);
	buffer[ind++] = conf->foc_temp_comp;
	buffer_append_float32_auto(buffer, conf->foc_temp_comp_base_temp, &ind);
	buffer_append_float32_auto(buffer, conf->foc_current_filter_const, &ind);
	buffer_append_int16(buffer, conf->gpd_buffer_notify_left, &ind);
	buffer_append_int16(buffer, conf->gpd_buffer_interpol, &ind);
	buffer_append_float32_auto(buffer, conf->gpd_current_filter_const, &ind);
	buffer_append_float32_auto(buffer, conf->gpd_current_kp, &ind);
	buffer_append_float32_auto(buffer, conf->gpd_current_ki, &ind);
	buffer_append_float32_auto(buffer, conf->s_pid_kp, &ind);
	buffer_append_float32_auto(buffer, conf->s_pid_ki, &ind);
	buffer_append_float32_auto(buffer, conf->s_pid_kd, &ind);
	buffer_append_float32_auto(buffer, conf->s_pid_kd_filter, &ind);
	buffer_append_float32_auto(buffer, conf->s_pid_min_erpm, &ind);
	buffer[ind++] = conf->s_pid_allow_braking;
	buffer_append_float32_auto(buffer, conf->p_pid_kp, &ind);
	buffer_append_float32_auto(buffer, conf->p_pid_ki, &ind);
	buffer_append_float32_auto(buffer, conf->p_pid_kd, &ind);
	buffer_append_float32_auto(buffer, conf->p_pid_kd_filter, &ind);
	buffer_append_float32_auto(buffer, conf->p_pid_ang_div, &ind);
	buffer_append_float32_auto(buffer, conf->cc_startup_boost_duty, &ind);
	buffer_append_float32_auto(buffer, conf->cc_min_current, &ind);
	buffer_append_float32_auto(buffer, conf->cc_gain, &ind);
	buffer_append_float32_auto(buffer, conf->cc_ramp_step_max, &ind);
	buffer_append_int32(buffer, conf->m_fault_stop_time_ms, &ind);
	buffer_append_float32_auto(buffer, conf->m_duty_ramp_step, &ind);
	buffer_append_float32_auto(buffer, conf->m_current_backoff_gain, &ind);
	buffer_append_uint32(buffer, conf->m_encoder_counts, &ind);
	buffer[ind++] = conf->m_sensor_port_mode;
	buffer[ind++] = conf->m_invert_direction;
	buffer[ind++] = conf->m_drv8301_oc_mode;
	buffer[ind++] = (uint8_t)conf->m_drv8301_oc_adj;
	buffer_append_float32_auto(buffer, conf->m_bldc_f_sw_min, &ind);
	buffer_append_float32_auto(buffer, conf->m_bldc_f_sw_max, &ind);
	buffer_append_float32_auto(buffer, conf->m_dc_f_sw, &ind);
	buffer_append_float32_auto(buffer, conf->m_ntc_motor_beta, &ind);
	buffer[ind++] = conf->m_out_aux_mode;
	buffer[ind++] = (uint8_t)conf->si_motor_poles;
	buffer_append_float32_auto(buffer, conf->si_gear_ratio, &ind);
	buffer_append_float32_auto(buffer, conf->si_wheel_diameter, &ind);
	buffer[ind++] = conf->si_battery_type;
	buffer[ind++] = (uint8_t)conf->si_battery_cells;
	buffer_append_float32_auto(buffer, conf->si_battery_ah, &ind);

	return ind;
}

int32_t vescUSB::confgenerator_serialize_appconf(uint8_t *buffer, const app_configuration *conf) {
	int32_t ind = 0;

	buffer_append_uint32(buffer, APPCONF_SIGNATURE, &ind);

	buffer[ind++] = (uint8_t)conf->controller_id;
	buffer_append_uint32(buffer, conf->timeout_msec, &ind);
	buffer_append_float32_auto(buffer, conf->timeout_brake_current, &ind);
	buffer[ind++] = conf->send_can_status;
	buffer_append_uint16(buffer, conf->send_can_status_rate_hz, &ind);
	buffer[ind++] = conf->can_baud_rate;
	buffer[ind++] = conf->pairing_done;
	buffer[ind++] = conf->permanent_uart_enabled;
	buffer[ind++] = conf->uavcan_enable;
	buffer[ind++] = (uint8_t)conf->uavcan_esc_index;
	buffer[ind++] = conf->app_to_use;
	buffer[ind++] = conf->app_ppm_conf.ctrl_type;
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.pid_max_erpm, &ind);
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.hyst, &ind);
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.pulse_start, &ind);
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.pulse_end, &ind);
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.pulse_center, &ind);
	buffer[ind++] = conf->app_ppm_conf.median_filter;
	buffer[ind++] = conf->app_ppm_conf.safe_start;
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.throttle_exp, &ind);
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.throttle_exp_brake, &ind);
	buffer[ind++] = conf->app_ppm_conf.throttle_exp_mode;
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.ramp_time_pos, &ind);
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.ramp_time_neg, &ind);
	buffer[ind++] = conf->app_ppm_conf.multi_esc;
	buffer[ind++] = conf->app_ppm_conf.tc;
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.tc_max_diff, &ind);
	buffer[ind++] = conf->app_adc_conf.ctrl_type;
	buffer_append_float32_auto(buffer, conf->app_adc_conf.hyst, &ind);
	buffer_append_float32_auto(buffer, conf->app_adc_conf.voltage_start, &ind);
	buffer_append_float32_auto(buffer, conf->app_adc_conf.voltage_end, &ind);
	buffer_append_float32_auto(buffer, conf->app_adc_conf.voltage_center, &ind);
	buffer_append_float32_auto(buffer, conf->app_adc_conf.voltage2_start, &ind);
	buffer_append_float32_auto(buffer, conf->app_adc_conf.voltage2_end, &ind);
	buffer[ind++] = conf->app_adc_conf.use_filter;
	buffer[ind++] = conf->app_adc_conf.safe_start;
	buffer[ind++] = conf->app_adc_conf.cc_button_inverted;
	buffer[ind++] = conf->app_adc_conf.rev_button_inverted;
	buffer[ind++] = conf->app_adc_conf.voltage_inverted;
	buffer[ind++] = conf->app_adc_conf.voltage2_inverted;
	buffer_append_float32_auto(buffer, conf->app_adc_conf.throttle_exp, &ind);
	buffer_append_float32_auto(buffer, conf->app_adc_conf.throttle_exp_brake, &ind);
	buffer[ind++] = conf->app_adc_conf.throttle_exp_mode;
	buffer_append_float32_auto(buffer, conf->app_adc_conf.ramp_time_pos, &ind);
	buffer_append_float32_auto(buffer, conf->app_adc_conf.ramp_time_neg, &ind);
	buffer[ind++] = conf->app_adc_conf.multi_esc;
	buffer[ind++] = conf->app_adc_conf.tc;
	buffer_append_float32_auto(buffer, conf->app_adc_conf.tc_max_diff, &ind);
	buffer_append_uint16(buffer, conf->app_adc_conf.update_rate_hz, &ind);
	buffer_append_uint32(buffer, conf->app_uart_baudrate, &ind);
	buffer[ind++] = conf->app_chuk_conf.ctrl_type;
	buffer_append_float32_auto(buffer, conf->app_chuk_conf.hyst, &ind);
	buffer_append_float32_auto(buffer, conf->app_chuk_conf.ramp_time_pos, &ind);
	buffer_append_float32_auto(buffer, conf->app_chuk_conf.ramp_time_neg, &ind);
	buffer_append_float32_auto(buffer, conf->app_chuk_conf.stick_erpm_per_s_in_cc, &ind);
	buffer_append_float32_auto(buffer, conf->app_chuk_conf.throttle_exp, &ind);
	buffer_append_float32_auto(buffer, conf->app_chuk_conf.throttle_exp_brake, &ind);
	buffer[ind++] = conf->app_chuk_conf.throttle_exp_mode;
	buffer[ind++] = conf->app_chuk_conf.multi_esc;
	buffer[ind++] = conf->app_chuk_conf.tc;
	buffer_append_float32_auto(buffer, conf->app_chuk_conf.tc_max_diff, &ind);
	buffer[ind++] = conf->app_nrf_conf.speed;
	buffer[ind++] = conf->app_nrf_conf.power;
	buffer[ind++] = conf->app_nrf_conf.crc_type;
	buffer[ind++] = conf->app_nrf_conf.retry_delay;
	buffer[ind++] = (uint8_t)conf->app_nrf_conf.retries;
	buffer[ind++] = (uint8_t)conf->app_nrf_conf.channel;
	buffer[ind++] = (uint8_t)conf->app_nrf_conf.address[0];
	buffer[ind++] = (uint8_t)conf->app_nrf_conf.address[1];
	buffer[ind++] = (uint8_t)conf->app_nrf_conf.address[2];
	buffer[ind++] = conf->app_nrf_conf.send_crc_ack;

	return ind;
}

bool vescUSB::confgenerator_deserialize_mcconf(const uint8_t *buffer, mc_configuration *conf) {
	int32_t ind = 0;

	uint32_t signature = buffer_get_uint32(buffer, &ind);
	if (signature != MCCONF_SIGNATURE) {
		return false;
	}

	conf->pwm_mode = buffer[ind++];
	conf->comm_mode = buffer[ind++];
	conf->motor_type = buffer[ind++];
	conf->sensor_mode = buffer[ind++];
	conf->l_current_max = buffer_get_float32_auto(buffer, &ind);
	conf->l_current_min = buffer_get_float32_auto(buffer, &ind);
	conf->l_in_current_max = buffer_get_float32_auto(buffer, &ind);
	conf->l_in_current_min = buffer_get_float32_auto(buffer, &ind);
	conf->l_abs_current_max = buffer_get_float32_auto(buffer, &ind);
	conf->l_min_erpm = buffer_get_float32_auto(buffer, &ind);
	conf->l_max_erpm = buffer_get_float32_auto(buffer, &ind);
	conf->l_erpm_start = buffer_get_float32_auto(buffer, &ind);
	conf->l_max_erpm_fbrake = buffer_get_float32_auto(buffer, &ind);
	conf->l_max_erpm_fbrake_cc = buffer_get_float32_auto(buffer, &ind);
	conf->l_min_vin = buffer_get_float32_auto(buffer, &ind);
	conf->l_max_vin = buffer_get_float32_auto(buffer, &ind);
	conf->l_battery_cut_start = buffer_get_float32_auto(buffer, &ind);
	conf->l_battery_cut_end = buffer_get_float32_auto(buffer, &ind);
	conf->l_slow_abs_current = buffer[ind++];
	conf->l_temp_fet_start = buffer_get_float32_auto(buffer, &ind);
	conf->l_temp_fet_end = buffer_get_float32_auto(buffer, &ind);
	conf->l_temp_motor_start = buffer_get_float32_auto(buffer, &ind);
	conf->l_temp_motor_end = buffer_get_float32_auto(buffer, &ind);
	conf->l_temp_accel_dec = buffer_get_float32_auto(buffer, &ind);
	conf->l_min_duty = buffer_get_float32_auto(buffer, &ind);
	conf->l_max_duty = buffer_get_float32_auto(buffer, &ind);
	conf->l_watt_max = buffer_get_float32_auto(buffer, &ind);
	conf->l_watt_min = buffer_get_float32_auto(buffer, &ind);
	conf->l_current_max_scale = buffer_get_float32_auto(buffer, &ind);
	conf->l_current_min_scale = buffer_get_float32_auto(buffer, &ind);
	conf->sl_min_erpm = buffer_get_float32_auto(buffer, &ind);
	conf->sl_min_erpm_cycle_int_limit = buffer_get_float32_auto(buffer, &ind);
	conf->sl_max_fullbreak_current_dir_change = buffer_get_float32_auto(buffer, &ind);
	conf->sl_cycle_int_limit = buffer_get_float32_auto(buffer, &ind);
	conf->sl_phase_advance_at_br = buffer_get_float32_auto(buffer, &ind);
	conf->sl_cycle_int_rpm_br = buffer_get_float32_auto(buffer, &ind);
	conf->sl_bemf_coupling_k = buffer_get_float32_auto(buffer, &ind);
	conf->hall_table[0] = (int8_t)buffer[ind++];
	conf->hall_table[1] = (int8_t)buffer[ind++];
	conf->hall_table[2] = (int8_t)buffer[ind++];
	conf->hall_table[3] = (int8_t)buffer[ind++];
	conf->hall_table[4] = (int8_t)buffer[ind++];
	conf->hall_table[5] = (int8_t)buffer[ind++];
	conf->hall_table[6] = (int8_t)buffer[ind++];
	conf->hall_table[7] = (int8_t)buffer[ind++];
	conf->hall_sl_erpm = buffer_get_float32_auto(buffer, &ind);
	conf->foc_current_kp = buffer_get_float32_auto(buffer, &ind);
	conf->foc_current_ki = buffer_get_float32_auto(buffer, &ind);
	conf->foc_f_sw = buffer_get_float32_auto(buffer, &ind);
	conf->foc_dt_us = buffer_get_float32_auto(buffer, &ind);
	conf->foc_encoder_inverted = buffer[ind++];
	conf->foc_encoder_offset = buffer_get_float32_auto(buffer, &ind);
	conf->foc_encoder_ratio = buffer_get_float32_auto(buffer, &ind);
	conf->foc_sensor_mode = buffer[ind++];
	conf->foc_pll_kp = buffer_get_float32_auto(buffer, &ind);
	conf->foc_pll_ki = buffer_get_float32_auto(buffer, &ind);
	conf->foc_motor_l = buffer_get_float32_auto(buffer, &ind);
	conf->foc_motor_r = buffer_get_float32_auto(buffer, &ind);
	conf->foc_motor_flux_linkage = buffer_get_float32_auto(buffer, &ind);
	conf->foc_observer_gain = buffer_get_float32_auto(buffer, &ind);
	conf->foc_observer_gain_slow = buffer_get_float32_auto(buffer, &ind);
	conf->foc_duty_dowmramp_kp = buffer_get_float32_auto(buffer, &ind);
	conf->foc_duty_dowmramp_ki = buffer_get_float32_auto(buffer, &ind);
	conf->foc_openloop_rpm = buffer_get_float32_auto(buffer, &ind);
	conf->foc_sl_openloop_hyst = buffer_get_float32_auto(buffer, &ind);
	conf->foc_sl_openloop_time = buffer_get_float32_auto(buffer, &ind);
	conf->foc_sl_d_current_duty = buffer_get_float32_auto(buffer, &ind);
	conf->foc_sl_d_current_factor = buffer_get_float32_auto(buffer, &ind);
	conf->foc_hall_table[0] = buffer[ind++];
	conf->foc_hall_table[1] = buffer[ind++];
	conf->foc_hall_table[2] = buffer[ind++];
	conf->foc_hall_table[3] = buffer[ind++];
	conf->foc_hall_table[4] = buffer[ind++];
	conf->foc_hall_table[5] = buffer[ind++];
	conf->foc_hall_table[6] = buffer[ind++];
	conf->foc_hall_table[7] = buffer[ind++];
	conf->foc_sl_erpm = buffer_get_float32_auto(buffer, &ind);
	conf->foc_sample_v0_v7 = buffer[ind++];
	conf->foc_sample_high_current = buffer[ind++];
	conf->foc_sat_comp = buffer_get_float32_auto(buffer, &ind);
	conf->foc_temp_comp = buffer[ind++];
	conf->foc_temp_comp_base_temp = buffer_get_float32_auto(buffer, &ind);
	conf->foc_current_filter_const = buffer_get_float32_auto(buffer, &ind);
	conf->gpd_buffer_notify_left = buffer_get_int16(buffer, &ind);
	conf->gpd_buffer_interpol = buffer_get_int16(buffer, &ind);
	conf->gpd_current_filter_const = buffer_get_float32_auto(buffer, &ind);
	conf->gpd_current_kp = buffer_get_float32_auto(buffer, &ind);
	conf->gpd_current_ki = buffer_get_float32_auto(buffer, &ind);
	conf->s_pid_kp = buffer_get_float32_auto(buffer, &ind);
	conf->s_pid_ki = buffer_get_float32_auto(buffer, &ind);
	conf->s_pid_kd = buffer_get_float32_auto(buffer, &ind);
	conf->s_pid_kd_filter = buffer_get_float32_auto(buffer, &ind);
	conf->s_pid_min_erpm = buffer_get_float32_auto(buffer, &ind);
	conf->s_pid_allow_braking = buffer[ind++];
	conf->p_pid_kp = buffer_get_float32_auto(buffer, &ind);
	conf->p_pid_ki = buffer_get_float32_auto(buffer, &ind);
	conf->p_pid_kd = buffer_get_float32_auto(buffer, &ind);
	conf->p_pid_kd_filter = buffer_get_float32_auto(buffer, &ind);
	conf->p_pid_ang_div = buffer_get_float32_auto(buffer, &ind);
	conf->cc_startup_boost_duty = buffer_get_float32_auto(buffer, &ind);
	conf->cc_min_current = buffer_get_float32_auto(buffer, &ind);
	conf->cc_gain = buffer_get_float32_auto(buffer, &ind);
	conf->cc_ramp_step_max = buffer_get_float32_auto(buffer, &ind);
	conf->m_fault_stop_time_ms = buffer_get_int32(buffer, &ind);
	conf->m_duty_ramp_step = buffer_get_float32_auto(buffer, &ind);
	conf->m_current_backoff_gain = buffer_get_float32_auto(buffer, &ind);
	conf->m_encoder_counts = buffer_get_uint32(buffer, &ind);
	conf->m_sensor_port_mode = buffer[ind++];
	conf->m_invert_direction = buffer[ind++];
	conf->m_drv8301_oc_mode = buffer[ind++];
	conf->m_drv8301_oc_adj = buffer[ind++];
	conf->m_bldc_f_sw_min = buffer_get_float32_auto(buffer, &ind);
	conf->m_bldc_f_sw_max = buffer_get_float32_auto(buffer, &ind);
	conf->m_dc_f_sw = buffer_get_float32_auto(buffer, &ind);
	conf->m_ntc_motor_beta = buffer_get_float32_auto(buffer, &ind);
	conf->m_out_aux_mode = buffer[ind++];
	conf->si_motor_poles = buffer[ind++];
	conf->si_gear_ratio = buffer_get_float32_auto(buffer, &ind);
	conf->si_wheel_diameter = buffer_get_float32_auto(buffer, &ind);
	conf->si_battery_type = buffer[ind++];
	conf->si_battery_cells = buffer[ind++];
	conf->si_battery_ah = buffer_get_float32_auto(buffer, &ind);

	return true;
}

bool vescUSB::confgenerator_deserialize_appconf(const uint8_t *buffer, app_configuration *conf) {
	int32_t ind = 0;

	uint32_t signature = buffer_get_uint32(buffer, &ind);
	if (signature != APPCONF_SIGNATURE) {
		return false;
	}

	conf->controller_id = buffer[ind++];
	conf->timeout_msec = buffer_get_uint32(buffer, &ind);
	conf->timeout_brake_current = buffer_get_float32_auto(buffer, &ind);
	conf->send_can_status = buffer[ind++];
	conf->send_can_status_rate_hz = buffer_get_uint16(buffer, &ind);
	conf->can_baud_rate = buffer[ind++];
	conf->pairing_done = buffer[ind++];
	conf->permanent_uart_enabled = buffer[ind++];
	conf->uavcan_enable = buffer[ind++];
	conf->uavcan_esc_index = buffer[ind++];
	conf->app_to_use = buffer[ind++];
	conf->app_ppm_conf.ctrl_type = buffer[ind++];
	conf->app_ppm_conf.pid_max_erpm = buffer_get_float32_auto(buffer, &ind);
	conf->app_ppm_conf.hyst = buffer_get_float32_auto(buffer, &ind);
	conf->app_ppm_conf.pulse_start = buffer_get_float32_auto(buffer, &ind);
	conf->app_ppm_conf.pulse_end = buffer_get_float32_auto(buffer, &ind);
	conf->app_ppm_conf.pulse_center = buffer_get_float32_auto(buffer, &ind);
	conf->app_ppm_conf.median_filter = buffer[ind++];
	conf->app_ppm_conf.safe_start = buffer[ind++];
	conf->app_ppm_conf.throttle_exp = buffer_get_float32_auto(buffer, &ind);
	conf->app_ppm_conf.throttle_exp_brake = buffer_get_float32_auto(buffer, &ind);
	conf->app_ppm_conf.throttle_exp_mode = buffer[ind++];
	conf->app_ppm_conf.ramp_time_pos = buffer_get_float32_auto(buffer, &ind);
	conf->app_ppm_conf.ramp_time_neg = buffer_get_float32_auto(buffer, &ind);
	conf->app_ppm_conf.multi_esc = buffer[ind++];
	conf->app_ppm_conf.tc = buffer[ind++];
	conf->app_ppm_conf.tc_max_diff = buffer_get_float32_auto(buffer, &ind);
	conf->app_adc_conf.ctrl_type = buffer[ind++];
	conf->app_adc_conf.hyst = buffer_get_float32_auto(buffer, &ind);
	conf->app_adc_conf.voltage_start = buffer_get_float32_auto(buffer, &ind);
	conf->app_adc_conf.voltage_end = buffer_get_float32_auto(buffer, &ind);
	conf->app_adc_conf.voltage_center = buffer_get_float32_auto(buffer, &ind);
	conf->app_adc_conf.voltage2_start = buffer_get_float32_auto(buffer, &ind);
	conf->app_adc_conf.voltage2_end = buffer_get_float32_auto(buffer, &ind);
	conf->app_adc_conf.use_filter = buffer[ind++];
	conf->app_adc_conf.safe_start = buffer[ind++];
	conf->app_adc_conf.cc_button_inverted = buffer[ind++];
	conf->app_adc_conf.rev_button_inverted = buffer[ind++];
	conf->app_adc_conf.voltage_inverted = buffer[ind++];
	conf->app_adc_conf.voltage2_inverted = buffer[ind++];
	conf->app_adc_conf.throttle_exp = buffer_get_float32_auto(buffer, &ind);
	conf->app_adc_conf.throttle_exp_brake = buffer_get_float32_auto(buffer, &ind);
	conf->app_adc_conf.throttle_exp_mode = buffer[ind++];
	conf->app_adc_conf.ramp_time_pos = buffer_get_float32_auto(buffer, &ind);
	conf->app_adc_conf.ramp_time_neg = buffer_get_float32_auto(buffer, &ind);
	conf->app_adc_conf.multi_esc = buffer[ind++];
	conf->app_adc_conf.tc = buffer[ind++];
	conf->app_adc_conf.tc_max_diff = buffer_get_float32_auto(buffer, &ind);
	conf->app_adc_conf.update_rate_hz = buffer_get_uint16(buffer, &ind);
	conf->app_uart_baudrate = buffer_get_uint32(buffer, &ind);
	conf->app_chuk_conf.ctrl_type = buffer[ind++];
	conf->app_chuk_conf.hyst = buffer_get_float32_auto(buffer, &ind);
	conf->app_chuk_conf.ramp_time_pos = buffer_get_float32_auto(buffer, &ind);
	conf->app_chuk_conf.ramp_time_neg = buffer_get_float32_auto(buffer, &ind);
	conf->app_chuk_conf.stick_erpm_per_s_in_cc = buffer_get_float32_auto(buffer, &ind);
	conf->app_chuk_conf.throttle_exp = buffer_get_float32_auto(buffer, &ind);
	conf->app_chuk_conf.throttle_exp_brake = buffer_get_float32_auto(buffer, &ind);
	conf->app_chuk_conf.throttle_exp_mode = buffer[ind++];
	conf->app_chuk_conf.multi_esc = buffer[ind++];
	conf->app_chuk_conf.tc = buffer[ind++];
	conf->app_chuk_conf.tc_max_diff = buffer_get_float32_auto(buffer, &ind);
	conf->app_nrf_conf.speed = buffer[ind++];
	conf->app_nrf_conf.power = buffer[ind++];
	conf->app_nrf_conf.crc_type = buffer[ind++];
	conf->app_nrf_conf.retry_delay = buffer[ind++];
	conf->app_nrf_conf.retries = (int8_t)buffer[ind++];
	conf->app_nrf_conf.channel = (int8_t)buffer[ind++];
	conf->app_nrf_conf.address[0] = buffer[ind++];
	conf->app_nrf_conf.address[1] = buffer[ind++];
	conf->app_nrf_conf.address[2] = buffer[ind++];
	conf->app_nrf_conf.send_crc_ack = buffer[ind++];

	return true;
}
