
#ifndef VESC_USB_H_
#define VESC_USB_H_

#include <stdint.h>
#include <stdbool.h>
#include "datatypes.h"

// Settings
#define PACKET_RX_TIMEOUT		2
#define PACKET_HANDLERS			1
#define PACKET_MAX_PL_LEN		512
#define BAUDRATE B921600   // Change as needed, keep B
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define SERIAL_RX_BUFFER_SIZE 2048 // Maximum receive buffer size
#define PACKET_HANDLER			0
#define MCCONF_SIGNATURE		1753222668
#define APPCONF_SIGNATURE		1146628643

class vescUSB
{
public:
	vescUSB();

	/******************** comm_uart ********************/
	void comm_init(char* modemDevice);   // FIRST FUNCTION TO CALL
	void comm_close(void); // LAST FUNCTION TO CALL
	int receive_packet(void); // CALL FOR READING NEW PACKETS
	void send_packet(unsigned char *data, unsigned int len);
	int fd;	// File Descriptor

	/******************** packet ********************/
	void packet_init(void(vescUSB::*s_func)(unsigned char *data, unsigned int len),
	void(vescUSB::*p_func)(unsigned char *data, unsigned int len), int handler_num);
	void packet_process_byte(uint8_t rx_data, int handler_num);
	void packet_timerfunc(void);
	void packet_send_packet(unsigned char *data, unsigned int len, int handler_num);

	/******************** interface ********************/

	// variables for received data
	 mc_values values;
	 int fw_major;
	 int fw_minor;
	 float rotor_pos;
	 mc_configuration mcconf;
	 bool mcconf_read;
     bool values_read;
	 app_configuration appconf;
	 float detect_cycle_int_limit;
	 float detect_coupling_k;
	 signed char detect_hall_table[8];
	 signed char detect_hall_res;
	 float dec_ppm;
	 float dec_ppm_len;
	 float dec_adc;
	 float dec_adc_voltage;
	 float dec_chuk;

	// interface functions
	void bldc_interface_init(void(vescUSB::*func)(unsigned char *data, unsigned int len));
	void bldc_interface_set_forward_func(void(*func)(unsigned char *data, unsigned int len));
	void bldc_interface_send_packet(unsigned char *data, unsigned int len);
	void bldc_interface_process_packet(unsigned char *data, unsigned int len);

	// Function pointer setters
	void bldc_interface_set_rx_value_func(void(*func)(mc_values *values));
	void bldc_interface_set_rx_printf_func(void(*func)(char *str));
	void bldc_interface_set_rx_fw_func(void(*func)(int major, int minor));
	void bldc_interface_set_rx_rotor_pos_func(void(*func)(float pos));
	void bldc_interface_set_rx_mcconf_func(void(*func)(mc_configuration *conf));
	void bldc_interface_set_rx_appconf_func(void(*func)(app_configuration *conf));
	void bldc_interface_set_rx_detect_func(void(*func)(float cycle_int_limit, float coupling_k,
		const signed char *hall_table, signed char hall_res));
	void bldc_interface_set_rx_dec_ppm_func(void(*func)(float val, float ms));
	void bldc_interface_set_rx_dec_adc_func(void(*func)(float val, float voltage));
	void bldc_interface_set_rx_dec_chuk_func(void(*func)(float val));
	void bldc_interface_set_rx_mcconf_received_func(void(*func)(void));
	void bldc_interface_set_rx_appconf_received_func(void(*func)(void));

	void bldc_interface_set_sim_control_function(void(*func)(motor_control_mode mode, float value));
	void bldc_interface_set_sim_values_func(void(*func)(void));

	// Setters
	void bldc_interface_terminal_cmd(char* cmd);
	void bldc_interface_set_duty_cycle(float dutyCycle);
	void bldc_interface_set_current(float current);
	void bldc_interface_set_current_brake(float current);
	void bldc_interface_set_rpm(int rpm);
	void bldc_interface_set_pos(float pos);
	void bldc_interface_set_handbrake(float current);
	void bldc_interface_set_servo_pos(float pos);
	void bldc_interface_set_mcconf(const mc_configuration *mcconf);
	void bldc_interface_set_appconf(const app_configuration *appconf);

	// Getters
	void bldc_interface_get_fw_version(void);
	void bldc_interface_get_values(void);
	void bldc_interface_get_mcconf(void);
	void bldc_interface_get_appconf(void);
	void bldc_interface_get_decoded_ppm(void);
	void bldc_interface_get_decoded_adc(void);
	void bldc_interface_get_decoded_chuk(void);

	// Other functions
	void bldc_interface_detect_motor_param(float current, float min_rpm, float low_duty);
	void bldc_interface_reboot(void);
	void bldc_interface_send_alive(void);
	void send_values_to_receiver(mc_values *values);

	// Helpers
	const char* bldc_interface_fault_to_string(mc_fault_code fault);


private:
	/******************** packet ********************/
	typedef struct {
		volatile unsigned char rx_state;
		volatile unsigned char rx_timeout;
		//void(vescUSB::*send_func)(unsigned char *data, unsigned int len);
		//void(vescUSB::*process_func)(unsigned char *data, unsigned int len);
		unsigned int payload_length;
		unsigned char rx_buffer[PACKET_MAX_PL_LEN];
		unsigned char tx_buffer[PACKET_MAX_PL_LEN + 6];
		unsigned int rx_data_ptr;
		unsigned char crc_low;
		unsigned char crc_high;
	} PACKET_STATE_t;
	PACKET_STATE_t handler_states[PACKET_HANDLERS];

	/******************** comm_uart ********************/
	

	/******************** interface ********************/
	// Private variables
	unsigned char send_buffer[1024];
	//int32_t can_fwd_vesc;
	
	// Private functions
	void send_packet_no_fwd(unsigned char *data, unsigned int len);
	//void fwd_can_append(uint8_t *data, int32_t *ind);

	// Function pointers
	void(vescUSB::*send_func)(unsigned char *data, unsigned int len);
	void(*forward_func)(unsigned char *data, unsigned int len);

	// Function pointers for received data
	 void(*rx_value_func)(mc_values *values);
	 void(*rx_printf_func)(char *str) ;
	 void(*rx_fw_func)(int major, int minor) ;
	 void(*rx_rotor_pos_func)(float pos);
	 void(*rx_mcconf_func)(mc_configuration *conf);
	 void(*rx_appconf_func)(app_configuration *conf);
	 void(*rx_detect_func)(float cycle_int_limit, float coupling_k,
		const signed char *hall_table, signed char hall_res) ;
	 void(*rx_dec_ppm_func)(float val, float ms);
	 void(*rx_dec_adc_func)(float val, float voltage) ;
	 void(*rx_dec_chuk_func)(float val) ;
	 void(*rx_mcconf_received_func)(void) ;
	 void(*rx_appconf_received_func)(void) ;
	 void(*motor_control_set_func)(motor_control_mode mode, float value);
	 void(*values_requested_func)(void) ;

	 // confgen
	 int32_t confgenerator_serialize_mcconf(uint8_t *buffer, const mc_configuration *conf);
	 int32_t confgenerator_serialize_appconf(uint8_t *buffer, const app_configuration *conf);

	 bool confgenerator_deserialize_mcconf(const uint8_t *buffer, mc_configuration *conf);
	 bool confgenerator_deserialize_appconf(const uint8_t *buffer, app_configuration *conf);

};
	


#endif /* VESC_USB_H_ */
