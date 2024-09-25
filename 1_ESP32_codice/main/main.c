// my_app: main with DMA + wifi with DC, pressure, flow

// fluss: filo verde
// press: filo giallo

#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver\adc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/timer.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "time.h"
#include "sys/ioctl.h"
#include "esp32/rom/md5_hash.h"
#include "driver/i2c.h"

// SYSTEM CONFIG
#define CONFIG_FREERTOS_HZ 1000
#define DELAY_FACTOR 1
static const BaseType_t app_cpu = 1;
static const BaseType_t pro_cpu = 0;

// esp TTGO + board
//#define POTENTIOMETER GPIO_NUM_34
#define BATTERY_LVL_CH ADC_CHANNEL_4
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTEN ADC_ATTEN_DB_11
//#define GPIO_PWM1A_OUT 2
#define BATTERY_LVL 32
#define PWM_UNIT MCPWM_UNIT_1
#define PWM_TIMER MCPWM_TIMER_0
#define PWM_OUT MCPWM0A
#define PWM_PIN 2
#define PIN_ISR 38
#define PIN_TASK 17

// pressure and flow SPI pins
#define TCK_SPI 30
#define DATA1 GPIO_NUM_33
#define DATA2 GPIO_NUM_25
#define CS_SPI GPIO_NUM_26
#define CLK_SPI GPIO_NUM_27

// SPI SD
#define SPI_SD_CLK 17
#define SPI_SD_CS 15
#define SPI_SD_MOSI 13
#define SPI_SD_MISO 12

// I2C parameters

#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */

#define ACK_CHECK_EN                0x1
#define ACK_CHECK_DIS               0x0
#define NACK_VAL   0x1
#define ACK_VAL   0x0

#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000                     /*!< I2C master clock frequency */
#define T_H_SLAVE_ADDR              0x27
#define P_SLAVE_ADDR                0b01110110

#define K_T 165 / (16384 - 2)
#define K_H 100 / (16384 - 2)
#define Q_T  40

// TIMER PARAMETERS
#define CLOCK_FREQUENCY TIMER_BASE_CLK // 240 MHz
#define TIMER_DIVIDER 16
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER)  //240/16 MHz convert counter value to seconds
#define TIMER_INTERVAL_SEC 0.002    //ISR time (s)
#define AUTORELOAD 1                // 1 = YES, 0 = NO
#define TIMER_GROUP_NUMBER TIMER_GROUP_0
#define TIMER_N 0

// Pressure Sensor Calibration Parameters
#define m_P +0.01592//0.015862
#define q_P -29.3234//-29.3018

#define m_F -0.03802//-0.020427
#define q_F -69.8676//-37.5202

// Ambient sensor parameters
int ambientIndex = 0;
double Temperature[10];
double Humidity[10];
double ambientData[2];

// Wifi Parameters

#define IP_ADDRESS_PC "192.168.4.2"
#define IP_ADDRESS_LENGHT 4
#define DELAY_WIFI 100 // (ms) interval of data transmission
#define SAMPLE_RATE 2   // rate of sample wrt ISR frequency : (2 each 500) -> 250 hz
#define NUM_of_ELEMENTS DELAY_WIFI//  //Elements stored in one pressure - flow - DC array before being overwritten
#define NUM_of_PARAM 3  // DC, PRESS,FLOW
#define DIM_FORMAT 2    // uint_8 = 1, int = 2, float = 4 (?) -> # of bytes allocated by memory according to variable type
#define targetLen 8
#define ASCII_const 65
#define Nsamples 500
#define dim_send_buff_16 77
#define dim_send_buff_8 2 * dim_send_buff_16
#define t_train  10


uint16_t data16bits[7];
uint8_t dataPROM1 = 0;
uint8_t dataPROM2 = 0;
uint8_t dataP1, dataP2, dataP3, dataT1, dataT2, dataT3;
uint16_t m_P_abs;
uint16_t q_P_abs;
uint16_t TCS;
uint16_t TCO;
uint16_t T_ref;
uint16_t T_SENS;
int16_t data_TH_P[dim_send_buff_16];
bool flag_P_abs_reset = 1;

double q;
double m;

// Variables
uint8_t countMonitorData = 0;
bool flagMonitorParam;
bool flagStartAvg = 0;
uint16_t b_lvl;
char rx_buff_str[10];
int new_count = 0;
bool startFlag = 0;
int count = 0;
int counter = 1;
int16_t data_buff_array[dim_send_buff_16];
int16_t data_storage_array[dim_send_buff_16];
double sin_data_array[Nsamples];
double DC = 50.0;
double DC_value;
uint16_t period_index_off_avg=0;
uint16_t array_index = 0;
uint16_t period_size;

unsigned int  idx= 0;
double F_Sensor_Value;
double P_Sensor_Value;
const double resolution = 100;
const double T = 2 * M_PI;
double dt = T / resolution;
int wifi_index = 0;
int ind = 0;
int stopCount = 0;
int period_num_index = 0;
double time_taken;

// Wifi variables
struct sockaddr_storage dest_addr;
const int CONNECTED_BIT = BIT0;
int backlog = 4;
int clientSock;
int client_sock_copy;
int serverSock;
static const char *TAG = "-";
char rcv_buf[10];
bool updateFlag = 0;
bool flagFirstTime = 0;
bool calibFlag = 0;   // to be removed
bool flagPidNew = 0;
int PORT_NUM = 5000;
int16_t buff[dim_send_buff_16];
uint16_t time_ms = 0;
uint32_t time_s = 0;
uint32_t time_amp = 0;
uint32_t time_save_wait = 0;
// Event group
//static EventGroupHandle_t wifi_event_group;
int checkBIT;

// PID :
double Kp_temp = 0;
double Ki_temp = 0;
double Kd_temp = 0;
double sampled_press;
int flag_PID = 0;
double alpha = 0;
bool flagNextPID = 0;
double coeff = 1;

// PID OFFSET
double DC_old = 50;
int count_error_th = 0;
double err_int;
double err_diff;
double PID_out_offset;
double err_prior_offset;
double PID_out_old_offset;
double Kprop_offset = 1;
double Kint_offset = 1;
double Kdiff_offset = 0;
int CL_OFF_winSize = 1;
int win_shift_off = 100;
double offPID_array[1000] = {0};
double offPID_array_sum = 0;
int16_t FOT_array[dim_send_buff_16] = {0};
bool flagSendFOTfreq = 0;

//PID AMPLITUDE
int err = 0;
double err_prior_amp = 0;
double err_int_amp = 0;
int err_diff_amp = 0;
double PID_out_amp = 1;
int PID_out_old_amp = 0;
double max_sin_amp = -100;
double min_sin_amp = 100;
double max_sin_amp_f = -100;
double min_sin_amp_f = 100;
double Kp_amp = 1.50;
double Ki_amp = 2.50;
double Kd_amp = 0.1;
double avg_amp_prior = 0;
//double avg_amp_max = 0;
const double amp_coeff[7] = {0.5,0.7,0.75,1.9,1.7,1.5,1.25};
uint32_t time_save = 0;

// filter ( 4th order Butterworth @ 10 Hz)
#define a1_lp 1
#define a2_lp -2.69261098701744
#define a3_lp 2.86739910911139
#define a4_lp -1.40348467136814
#define a5_lp 0.264454816443504
#define b1_lp  0.00223489169808230
#define b2_lp 0.00893956679232921
#define b3_lp 0.0134093501884938
#define b4_lp 0.00893956679232921
#define b5_lp 0.00223489169808230
double y[5] = {0};
double x[5] = {0};

//PID BASE
double Kp_baseline = 0;
double Ki_baseline = 0;
double Kd_baseline = 0;
double amp_PID_array[Nsamples];
double ampPID_array_sum = 0;
double PID_out_base;
double err_prior_base;
double PID_out_old_base;
double err_int_base;
double err_diff_base;
int BASE_CL_winSize = 0;
bool flag_base = 0;

// detrend
double sumx  = 0;
double sumy  = 0;
double sumxy = 0;
double sumx2 = 0;
uint16_t t= 0;
uint16_t t_min = 0;
uint16_t t_max = 0;


// FOT variables
uint16_t countAcquisition = 0;
bool flagAcquisition = 0;

// Other control parameters
#define DC_minimum 55
#define corr_factor 1
double sinAmp_calib = 1;
double DC_target_value =  DC_minimum;
double DC_step = 1.5;
double seno[Nsamples];
int period_index = 0;
double ampli_range = 0;
double DC_min = 55;
double DC_max = 100;
double freq_threshold = 7.75;
double delta;
double beta;
bool flag_saveOffManual = 0;
bool flagSendData = 0;
double manualDC = 50;
int count_sent = 0;
bool flag_first_avg = 1;
bool flagMeasureEnded = 0;

// struct variables

typedef enum  state {STOP, STARTING, STARTED, TRANSITION, STOPPING} state;
state STATE = STOP;

typedef enum  wifi_state {START_COMMAND, STOP_COMMAND, SET_PRESSURE_AMP,
                    	SET_CL_BASE_AND_AMP, SET_OFFSET, SET_FREQ, UPDATED_ALL,
						  SAVE_CL_OFF, SET_CL_OFF_WIN_SIZE,SET_PID_TYPE,
						  SET_K, PID_STATE,MANUAL_OFF,SAVE_MANUAL_OFF,
                          GET_TEMP_HUM, RESET, SET_DURATION, FINISH_MEASURE} wifi_state;
typedef enum  mode {OFF_MODE, AMP_MODE, BASE_MODE} mode;
typedef enum CONTROL {OPEN_LOOP, CL_OFFSET, CL_AMP, CL_BASE, MANUAL_OFFSET }control;
control CONTROL = CL_OFFSET;

wifi_state firstByte;

struct sinStruct {
	double freq;
	double P_amp;
	double P_offset;
	double duration;
}FOTparam;

/*
struct data_FOT{
	double flow[acqusitionTime_ms][nFreq];
	double pressure[acqusitionTime_ms][nFreq];
	//float flow[acqusitionTime_ms];
}data_FOT;*/

struct HT_sensor_data{
	double humidity;
	double temperature;
}sensor_data;

TickType_t xLastWakeTime = 0;

// ISR task variables
int TransitionFlag = 0;

// AP variables
#define EXAMPLE_ESP_WIFI_SSID      "HorseFOT_Network"
#define EXAMPLE_ESP_WIFI_PASS      "esp32password"
#define EXAMPLE_ESP_WIFI_CHANNEL   1
#define EXAMPLE_MAX_STA_CONN       2
bool AP_flag = 0;


//Queues and Semaphores
//QueueHandle_t DC_queue;
SemaphoreHandle_t xSemaphore;
BaseType_t xHigherPriorityTaskWoken =  pdTRUE;



void initVariables(){

	sinAmp_calib    = 1;
	DC_target_value = DC_minimum;
	DC_step         = 1.5;
	period_index    = 0;
	ampli_range     = 0;
	DC_min          = 55;
	DC_max          = 99;
	freq_threshold  = 7.75;
	delta           = 0;
	beta            = 0;
	manualDC        = 50;
	count_sent      = 0;

	flag_saveOffManual  = 0;
	flagSendData        = 0;
	flag_first_avg      = 1;
	flagMeasureEnded    = 0;
	flag_base           = 0;
	flagMonitorParam    = 0;
	flagStartAvg        = 0;
	startFlag           = 0;
	updateFlag          = 0;
	flagFirstTime       = 0;
	calibFlag           = 0;
	flagPidNew          = 0;
	flagNextPID         = 0;
	flag_P_abs_reset    = 1;
	flag_PID            = 0;
	flagSendFOTfreq     = 0;

	//PID BASE
	Kp_baseline = 0;
	Ki_baseline = 0;
	Kd_baseline = 0;
	ampPID_array_sum = 0;
	BASE_CL_winSize = 0;

	// detrend
	sumx  = 0;
	sumy  = 0;
	sumxy = 0;
	sumx2 = 0;
	t     = 0;
	t_min = 0;
	t_max = 0;

	dataPROM1 = 0;
	dataPROM2 = 0;

	// Variables
	countMonitorData     = 0;
	b_lvl                = 0;
	new_count            = 0;
	count                = 0;
	counter              = 1;
	period_index_off_avg = 0;
	array_index          = 0;
	DC               = 50.0;
	DC_value         = 0;
	period_size      = 0;
	idx              = 0;
	F_Sensor_Value   = 0;
	P_Sensor_Value   = 0;
	wifi_index       = 0;
	ind              = 0;
	stopCount        = 0;
	period_num_index = 0;
	time_taken       = 0;
	dt               = T / resolution;

	// Wifi variables
	PORT_NUM       = 5000;
	time_ms        = 0;
	time_s         = 0;
	time_amp       = 0;
	time_save_wait = 0;

	// PID :
	Kp_temp       = 0;
	Ki_temp       = 0;
	Kd_temp       = 0;
	alpha         = 0;
	coeff         = 1;
	sampled_press = 0;

	// PID OFFSET
	DC_old                      = 50;
	count_error_th              = 0;
	Kprop_offset                = 0;
	Kint_offset                 = 0;
	Kdiff_offset                = 0;
	CL_OFF_winSize              = 1;
	win_shift_off               = 100;
	offPID_array_sum            = 0;

	//PID AMPLITUDE
	err             = 0;
	err_prior_amp   = 0;
	err_int_amp     = 0;
	err_diff_amp    = 0;
	PID_out_amp     = 1;
	PID_out_old_amp = 0;
	max_sin_amp     = -100;
	min_sin_amp     = 100;
	max_sin_amp_f   = -100;
	min_sin_amp_f   = 100;
	Kp_amp          = 0;
	Ki_amp          = 0;
	Kd_amp          = 0;
	avg_amp_prior   = 0;
    time_save       = 0;

    for (uint16_t i = 0; i< Nsamples; i++){
    	amp_PID_array[i]    = 0;
    	sin_data_array[i]   = 0;
    	offPID_array[i]     = 0;
    	offPID_array[i+500] = 0;
    	seno[i]             = 0;
    }
}



//---------------------------------------------------------------------------------------------------------------------------------------------//
// Returns the ASCII value of a string (or int numbers)
void ASCIIfromString(char* string){
	//ESP_LOGI("ASCII","clientSock: %d\n",clientSock);
	//uint8_t len = strlen(string);
	for (uint8_t i = 0; i < dim_send_buff_16; i++){
		buff[i] = (int16_t)(string[2*i] | string[2*i+1]<<8);
	}
	//printf("clientSock after ASCII: %d\n",clientSock);

}



// Reverses a string 'rcv_buf' of length 'len'
void reverse(char* rcv_buf, int len)
{
	int i = 0, j = len - 1, temp;
	while (i < j) {
		temp = rcv_buf[i];
		rcv_buf[i] = rcv_buf[j];
		rcv_buf[j] = temp;
		i++;
		j--;
	}
}




// Converts a given integer x to string rcv_buf[].
// d is the number of digits required in the output.
// If d is more than the number of digits in x,
// then 0s are added at the beginning.
int intToStr(int x, char rcv_buf[], int d)
{
	int i = 0;
	while (x) {
		rcv_buf[i++] = (x % 10) + '0';
		x = x / 10;
	}

	// If number of digits required is more, then
	// add 0s at the beginning
	while (i < d)
		rcv_buf[i++] = '0';

	reverse(rcv_buf, i);
	rcv_buf[i] = '\0';
	return i;
}



void pad(char* stringData){
	printf("clientsock: %d\n",clientSock);

	int len = strlen(stringData);
	printf("srtring:%s\tlen:%d\n",stringData,len);

	for(int i = len; i<dim_send_buff_8; i++){
		*(stringData + i) = '0';
	}
	//*(stringData + dim_send_buff_16)= '/0';

	len = strlen(stringData);
	printf("new string:%s\tlen:%d\n",stringData,len);
	printf("clientsock after pad: %d\n",clientSock);
}



// Converts a floating-point/double number to a string.
void ftoa(float n, char* res, int afterpoint)
{
	// Extract integer part
	int ipart = (int)n;

	// Extract floating part
	float fpart = n - (float)ipart;

	// convert integer part to string
	int i = intToStr(ipart, res, 0);

	// check for display option after point
	if (afterpoint != 0) {
		res[i] = '.'; // add dot

		fpart = fpart * pow(10, afterpoint);

		intToStr((int)fpart, res + i + 1, afterpoint);
	}
}




// generate_Sinusoid() : SINUSOID DC GENERATION @ SIN_FREQ Hz (typically 1 to 20 Hz)

void generate_Sinusoid(){

	DC_target_value = DC_min;// + FOTparam.P_offset;

	for (int i = 0; i<Nsamples; i++){
		seno[i] = sin((double)i/Nsamples * 2 * M_PI);
	}

}



// adjustSinPID():

void adjustSinPID(){
	int x = (floor)(Nsamples/FOTparam.freq);
	double sinPID_temp[x];

	for (int i = 0; i< (floor)(Nsamples/FOTparam.freq); i++){

		sinPID_temp[i] = amp_PID_array[i];

		if (i< period_index){
		    amp_PID_array[i] = amp_PID_array[i+ period_index];
		}
		else{
			amp_PID_array[i] = sinPID_temp[i-period_index];
		}
	}
}



double array_average_update(double new_data, uint16_t winSize, double* array, double array_sum){

	// saving old data in a temp variable, storing new data in the array
	double old_data = *(array + array_index);
	*(array + array_index) = new_data;
	array_sum = array_sum + new_data - old_data;

	// updating array index
	array_index ++;

	// when index reaches array end, start back from zero
	if(array_index == winSize){
		array_index = 0;
	}

	return array_sum;
}


//---------------------------------------------------------------------------------------------------------------------------------------------//



/* my_setup_pin(): configure pin direction (IN/OUT)
 *
 *
 *  */

void my_setup_pin()
{
	gpio_set_direction(PIN_ISR, GPIO_MODE_OUTPUT);
	//gpio_set_direction(PIN_WIFI, GPIO_MODE_OUTPUT);
	gpio_set_direction(PIN_TASK, GPIO_MODE_OUTPUT);
	gpio_set_direction(BATTERY_LVL, GPIO_MODE_INPUT);
	gpio_set_direction(PWM_PIN, GPIO_MODE_OUTPUT);
	gpio_set_direction(CLK_SPI, GPIO_MODE_OUTPUT);
	gpio_set_direction(CS_SPI, GPIO_MODE_OUTPUT);
	gpio_set_direction(DATA1, GPIO_MODE_INPUT);
	gpio_set_direction(DATA2, GPIO_MODE_INPUT);
	gpio_set_direction(SPI_SD_CLK, GPIO_MODE_OUTPUT);
	gpio_set_direction(SPI_SD_CS, GPIO_MODE_OUTPUT);
	gpio_set_direction(SPI_SD_MOSI, GPIO_MODE_OUTPUT);
	gpio_set_direction(SPI_SD_MISO, GPIO_MODE_INPUT);

}




/* my_setup_ADC() : Configure ADC n. 1.*/

void my_setup_ADC(int adc_bit_width, int adc_channel, int adc_attenuation)
{
	adc1_config_width(adc_bit_width);                         // conversion data width
	adc1_config_channel_atten(adc_channel, adc_attenuation);  // attenuation of data (range)

}


static esp_err_t my_i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}




/* my_setup_pwm() : configure PWM registers (timer and pin), set initial DC and frequency.*/

void my_setup_pwm() {

	mcpwm_config_t pwm_config;
	pwm_config.frequency = 500;     //frequency = 500Hz
	pwm_config.cmpr_a = 50.0;       //duty cycle of PWMxA = 0.0 %
	pwm_config.cmpr_b = 0.0;        //duty cycle of PWMxb = 0.0 %
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

	mcpwm_gpio_init(PWM_UNIT, PWM_OUT , PWM_PIN);
	mcpwm_init(PWM_UNIT, PWM_TIMER, &pwm_config);

	mcpwm_start(PWM_UNIT, PWM_TIMER);

}

//---------------------------------------------------------------------------------------------------------------------------------------------//


static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
        AP_flag = 1;
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        STATE = STOP;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    if (event_id == IP_EVENT_STA_LOST_IP){
    	ESP_LOGE("!!!","Event found!\n");
    }
}



//init softAP: initialize and start esp Access Point

void my_wifi_init_softap(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGW(TAG, "ESP_WIFI_MODE_AP");

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}





// Wifi: Socket error codes

int espx_last_socket_errno(int s) {
	int ret = 0;
	u32_t optlen = sizeof(ret);
	getsockopt(s, SOL_SOCKET, SO_ERROR, &ret, &optlen);
	return ret;
}



/* wifi_connect_to_client(): waits for a request connection from a client;
 *                            establishes connection.
 */
void my_wifi_connect_to_client(){

	// create a new socket
	serverSock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if(serverSock < 0) {
		ESP_LOGE(TAG, "socket: %d %s", serverSock, strerror(errno));
	}
	else{
	    printf("Socket allocated, id=%d\n", serverSock);
	}

	struct sockaddr_in serverAddress;
	serverAddress.sin_family = AF_INET;
	serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
	serverAddress.sin_port = htons(PORT_NUM);          // PORTNUM should be an integer
	bind(serverSock, (struct sockaddr *)&serverAddress, sizeof(serverAddress));
	printf("\nWaiting for client to connect ...");

	// listen to port for connection
	int res = listen(serverSock, backlog);
    if(res <= 0){
    	ESP_LOGE(TAG,"error: %d\n",res);
    }
    else{
	     ESP_LOGI(TAG,"\nClient detected. Establishing connection...\n");
    }

	struct sockaddr_in clientAddress;
	socklen_t clientAddressLength = sizeof(clientAddress);
	clientSock = accept(serverSock, (struct sockaddr *)&clientAddress,&clientAddressLength);
	client_sock_copy = clientSock;
	if (clientSock > 0){
		ESP_LOGI(TAG, "Connection established.");
	}
	else {
		ESP_LOGE(TAG,"Connection denied!");
	}

}



//---------------------------------------------------------------------------------------------------------------------------------------------//


/* SPI_read() : implements a virtual SPI to read flow and pressure sensors
 *              values read from sensors (digital values) are stored in
 *              F_Sensor_Value and P_Sensor_Value.
 */
float SPI_read(){

	char index;
	int i=0;                          // Clear Variables and Counters
	int SensorDATAF = 0;
	int SensorDATAP = 0;
	int lvl1;
	int lvl2;
	unsigned int molt = 2048;

	gpio_set_level(CS_SPI, 0);            // Enable Chip Select

	gpio_set_level(CLK_SPI, 0);           // 3 Starting Clock Periods
	for(i=0;i<TCK_SPI;i++);
	gpio_set_level(CLK_SPI, 1);
	for(i=0;i<TCK_SPI;i++);
	gpio_set_level(CLK_SPI, 0);
	for(i=0;i<TCK_SPI;i++);
	gpio_set_level(CLK_SPI, 1);
	for(i=0;i<TCK_SPI;i++);
	gpio_set_level(CLK_SPI, 0);
	for(i=0;i<TCK_SPI;i++);
	gpio_set_level(CLK_SPI, 1);
	for(i=0;i<TCK_SPI;i++);

	// Read and convert data from SPI

	for(index = 0; index < 12; index ++ ){
		gpio_set_level(CLK_SPI, 0);
		lvl1 = gpio_get_level(DATA1);
		lvl2=gpio_get_level(DATA2);

		SensorDATAF = SensorDATAF + (lvl1 * molt);
		SensorDATAP = SensorDATAP + (lvl2 * molt);

		for(i=0;i<TCK_SPI;i++);
		gpio_set_level(CLK_SPI,1);
		for(i=0;i<TCK_SPI;i++);
		molt = molt/2;

	}

	// Last clock cycle
	gpio_set_level(CLK_SPI, 0);
	for(i=0;i<TCK_SPI;i++);
	gpio_set_level(CLK_SPI,1);
	for(i=0;i<TCK_SPI;i++);

	//open S&H
	gpio_set_level(CS_SPI, 1);

	//save data

	new_count ++;
	(P_Sensor_Value ) = SensorDATAP ;
	(F_Sensor_Value ) =  SensorDATAF ;
    return (m_P * P_Sensor_Value  + q_P);

}









// sendData TCP
void TCPsend(int16_t* data, int len){
	int result = send(client_sock_copy, data, len , 0);
	//printf("clientSock: %d\n",clientSock);
	//printf("clientSock copy: %d\n",client_sock_copy);
	//printf("sent len:%d\n",len);
	if(result < 0) {
		int err;
		err = espx_last_socket_errno(serverSock);
		err = espx_last_socket_errno(clientSock);
		ESP_LOGE(TAG,"Unable to send the string, error: %d\nresult %d\n", err,result);

		while(1) vTaskDelay(DELAY_FACTOR * 100 / portTICK_RATE_MS);
	}
	else{
		//ESP_LOGW(TAG,"Data has been sent.\n");
	}
}




// sendData TCP
int TCPreceive(){

	//printf("clientSock: %d\n",clientSock);
	int totalLen = 0; int len;
	char receivedData[targetLen] = "";
	if(recv(client_sock_copy, rx_buff_str, strlen(rx_buff_str)-1, MSG_PEEK)){
		while(totalLen != targetLen){
			len = recv(client_sock_copy, rx_buff_str, strlen(rx_buff_str)-1, MSG_PEEK);
			//printf("len: %d\n",len);
			if (len<1){
				//ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
				return 0;
			}
			else if (len+totalLen>targetLen-1){
				totalLen = targetLen;
				recv(client_sock_copy, receivedData, totalLen, 0);
			}
			else{
				totalLen += len;
			}
		}
	}

	else
		return 0;
	receivedData[totalLen] = 0; // Null-terminate whatever is received and treat it like a string
	printf("rcvBuff: %s\n\n",receivedData);
	for (int i = 0; i< totalLen; i++){
		rcv_buf[i] = receivedData[i];
	}
	//printf("clientSock: %d\n",clientSock);
	ESP_LOGW(TAG,"len receivedData:%d\tlen rcv_buf:%d\n",strlen(receivedData),strlen(rcv_buf));
	ESP_LOGI(TAG,"Received: %s\n",rcv_buf);
	return 1;
}





void readI2C_T_H(){


	uint8_t data1, data2, data3, data4;

    // Measurement request
    i2c_cmd_handle_t cmd1 = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd1));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd1, (T_H_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN ));
	ESP_ERROR_CHECK(i2c_master_stop(cmd1));
	i2c_master_cmd_begin( I2C_MASTER_NUM, cmd1, 1000/portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd1);

	vTaskDelay(50/portTICK_RATE_MS);

    // Fetch request
	i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd2));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd2, (T_H_SLAVE_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd2, &data1, ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd2, &data2, ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd2, &data3, ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd2, &data4, NACK_VAL));
	ESP_ERROR_CHECK(i2c_master_stop(cmd2));
	i2c_master_cmd_begin( I2C_MASTER_NUM, cmd2, 1000/portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd2);

	//printf("d: %d\t%d\t%d\t%d\t\n",data1,data2,data3,data4);


	if(data1>>6) ESP_LOGE("HIH6131-021-001 error: ","wrong i2c status return");
	else {
		double humidity = ((data1<<8 | 0x00 | data2) & (0b0011111111111111)) ;
		double temperature = ((data3<<6 | 0x00 | data4>>2)) ;
		sensor_data.humidity = humidity * K_H;
		sensor_data.temperature = temperature * K_T - Q_T;


		data_TH_P[0] = 0xff54; data_TH_P[1] = (int16_t)(sensor_data.humidity * 100); data_TH_P[2] = (int16_t)(sensor_data.temperature *100);
		/*for (int i = 3; i< 31; i++){
			data_TH_P[i] = 0;
		}*/
	}

	//printf("H: %f\tT: %f\n",sensor_data.humidity, sensor_data.temperature);
}






void readI2C_P_abs(){
	if (flag_P_abs_reset){

		// Reset
		i2c_cmd_handle_t cmd_reset = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd_reset));

		ESP_ERROR_CHECK(i2c_master_write_byte(cmd_reset, 0b11101100 | I2C_MASTER_WRITE, ACK_CHECK_EN ));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd_reset, 0b00011110, ACK_CHECK_EN ));

		ESP_ERROR_CHECK(i2c_master_stop(cmd_reset));
		i2c_master_cmd_begin( I2C_MASTER_NUM, cmd_reset, 1000/portTICK_RATE_MS);
		i2c_cmd_link_delete(cmd_reset);

		vTaskDelay(50/portTICK_RATE_MS);


		// DO ONCE:
		//---------


		// PROM read sequence
		for (int i = 0; i<8; i++){
			//	uint8_t i = 1;
			uint8_t read_command = 0b10100000 | i<<1;

			i2c_cmd_handle_t cmd_PROM = i2c_cmd_link_create();

			ESP_ERROR_CHECK(i2c_master_start(cmd_PROM));

			ESP_ERROR_CHECK(i2c_master_write_byte(cmd_PROM, 0b11101100 | I2C_MASTER_WRITE, ACK_CHECK_EN ));
			ESP_ERROR_CHECK(i2c_master_write_byte(cmd_PROM, read_command, ACK_CHECK_EN ));

			ESP_ERROR_CHECK(i2c_master_stop(cmd_PROM));

			i2c_master_cmd_begin( I2C_MASTER_NUM, cmd_PROM, 1000/portTICK_RATE_MS);
			i2c_cmd_link_delete(cmd_PROM);


			i2c_cmd_handle_t cmd_PROM2 = i2c_cmd_link_create();
			cmd_PROM2 = i2c_cmd_link_create();

			ESP_ERROR_CHECK(i2c_master_start(cmd_PROM2));

			ESP_ERROR_CHECK(i2c_master_write_byte(cmd_PROM2,  (0b11101100) | I2C_MASTER_READ, ACK_CHECK_EN ));
			ESP_ERROR_CHECK(i2c_master_read_byte(cmd_PROM2, &dataPROM1, ACK_VAL));
			ESP_ERROR_CHECK(i2c_master_read_byte(cmd_PROM2, &dataPROM2, NACK_VAL));

			ESP_ERROR_CHECK(i2c_master_stop(cmd_PROM2));

			i2c_master_cmd_begin( I2C_MASTER_NUM, cmd_PROM2, 1000/portTICK_RATE_MS);
			i2c_cmd_link_delete(cmd_PROM2);

			data16bits[i] = (dataPROM1<<8) | dataPROM2 ;
			//	printf("command: %d, it: %d \ndata from EEPROM: %d \t %d \n 16 bit: %d\n\n",read_command, i,dataPROM1, dataPROM2, data16bits);

			flag_P_abs_reset = 0;
		}
		//ESP_LOGE("PROM","ok\n");

		m_P_abs = data16bits[1];
		q_P_abs = data16bits[2];
		TCS = data16bits[3];
		TCO = data16bits[4];
		T_ref = data16bits[5];
		T_SENS = data16bits[6];

		//crc computation
	}

	//PRESSURE

	// Initiate Pressure conversion sequence
	i2c_cmd_handle_t cmd_CONV_P = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd_CONV_P));

	ESP_ERROR_CHECK(i2c_master_write_byte(cmd_CONV_P, (P_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd_CONV_P, 0b01001000, ACK_CHECK_EN));

	ESP_ERROR_CHECK(i2c_master_stop(cmd_CONV_P));
	i2c_master_cmd_begin( I2C_MASTER_NUM, cmd_CONV_P, 1000/portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd_CONV_P);

	vTaskDelay(50/portTICK_RATE_MS);
	//ESP_LOGW("ADC init P","ok\n");

	// ADC read sequence
	i2c_cmd_handle_t cmd_ADC_read_P = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd_ADC_read_P));

	ESP_ERROR_CHECK(i2c_master_write_byte(cmd_ADC_read_P, (P_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd_ADC_read_P, 0x00, ACK_CHECK_EN));

	ESP_ERROR_CHECK(i2c_master_stop(cmd_ADC_read_P));
	i2c_master_cmd_begin( I2C_MASTER_NUM, cmd_ADC_read_P, 1000/portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd_ADC_read_P);

	vTaskDelay(50/portTICK_RATE_MS);

	ESP_LOGW("ADC read P","ok\n");

	// ADC answer sequence
	i2c_cmd_handle_t cmd_ADC_data_P = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd_ADC_data_P));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd_ADC_data_P, (P_SLAVE_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd_ADC_data_P, &dataP1, ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd_ADC_data_P, &dataP2, ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd_ADC_data_P, &dataP3, NACK_VAL));
	ESP_ERROR_CHECK(i2c_master_stop(cmd_ADC_data_P));
	i2c_master_cmd_begin( I2C_MASTER_NUM, cmd_ADC_data_P, 1000/portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd_ADC_data_P);

	vTaskDelay(50/portTICK_RATE_MS);

	//ESP_LOGW("ADC answer P","ok\n");

// TEMPERATURE
    // Initiate Temperature conversion sequence
	i2c_cmd_handle_t cmd_CONV_T = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd_CONV_T));

	ESP_ERROR_CHECK(i2c_master_write_byte(cmd_CONV_T, (P_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd_CONV_T, 0b01011000, ACK_CHECK_EN));

	ESP_ERROR_CHECK(i2c_master_stop(cmd_CONV_T));
	i2c_master_cmd_begin( I2C_MASTER_NUM, cmd_CONV_T, 1000/portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd_CONV_T);

	vTaskDelay(50/portTICK_RATE_MS);

	ESP_LOGW("Temp init","ok\n");

	// ADC read sequence T
	i2c_cmd_handle_t cmd_ADC_read_T = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd_ADC_read_T));

	ESP_ERROR_CHECK(i2c_master_write_byte(cmd_ADC_read_T, (P_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd_ADC_read_T, 0x00, ACK_CHECK_EN));

	ESP_ERROR_CHECK(i2c_master_stop(cmd_ADC_read_T));
	i2c_master_cmd_begin( I2C_MASTER_NUM, cmd_ADC_read_T, 1000/portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd_ADC_read_T);

	vTaskDelay(50/portTICK_RATE_MS);
	//ESP_LOGW("ADC read T","ok\n");

	// ADC answer sequence T
	i2c_cmd_handle_t cmd_ADC_data_T = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd_ADC_data_T));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd_ADC_data_T, (P_SLAVE_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd_ADC_data_T, &dataT1, ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd_ADC_data_T, &dataT2, ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd_ADC_data_T, &dataT3, NACK_VAL));
	ESP_ERROR_CHECK(i2c_master_stop(cmd_ADC_data_T));
	i2c_master_cmd_begin( I2C_MASTER_NUM, cmd_ADC_data_T, 1000/portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd_ADC_data_T);

	vTaskDelay(50/portTICK_RATE_MS);

	//ESP_LOGW("ADC answer T","ok\n");

	unsigned int Pdata = 0x0000 | (dataP1<<16) | (dataP2<<8) | (dataP3);
	unsigned int Tdata = 0x0000 | (dataT1<<16) | (dataT2<<8) | (dataT3);

	int dT = (int) (Tdata - (double)(T_ref * 256));
	float calib_dT = (double)(dT * (double)( (double)T_SENS/(double)8388608));
	int TEMP = 2000 + calib_dT;
	long int OFF = (q_P_abs * 131072) + (double)((TCO * dT) / 64);
	long int SENS = (m_P_abs * 65536) + (double)((TCS * dT) / 128);
	int pressure = (Pdata * (SENS/2097152) - OFF) / 32768;
/*
	printf("T_SENS: %d\nTdata: %d - Tref*256: %d\n",T_SENS, Tdata, T_ref*256);
	printf("dt: %d\n", dT);

	ESP_LOGW("->","P: %d \tT: %d\n\n",Pdata, Tdata);
	ESP_LOGI("->","P: %d \tT: %d\n\n",pressure, TEMP);

*/

	 data_TH_P[3] = (int16_t)(TEMP ); data_TH_P[4] = (int16_t)(pressure/100 );
	for (int i = 5; i< dim_send_buff_16; i++){
		data_TH_P[i] = 0;
	}

}




int get_battery_lvl(void){

	return (adc1_get_raw(BATTERY_LVL_CH) * 2450) / 4096;

}




//calcolo range sinusoide campionata, valore max-valore min-->ampli_range
//poi confronto ampli_range con amplitude della sinusoide del duty cycle

void maxmin(double press){

    if(press>max_sin_amp_f){
        max_sin_amp_f=press;
    }

    if(press<min_sin_amp_f){
        min_sin_amp_f =press;
    }

}



void update_sin_array(double press){
    sumx  += t;
    sumy  += press;
    sumxy += (t)*press;
    sumx2 += (t)*(t);
    sin_data_array[t-1] = press;
	t++;
}







/* de_trend_max_min: return sampled pressure sinusoid amplitude
*              - target amplitude
*			   - max sampled amp
*			   - min sampled amp
*			   -> return amplitude  difference */

double de_trend_max_min(double target, uint16_t length){

	//detrend
	m = (length*sumxy-sumx*sumy)/(length*sumx2-sumx*sumx);
	q = (sumy*sumx2-sumx*sumxy)/(length*sumx2-sumx*sumx);
	//resetting sum and t
	sumx  = 0;
	sumy  = 0;
	sumxy = 0;
	sumx2 = 0;
	t     = 1;
	//Resetting min and max
	min_sin_amp   =  100;
	max_sin_amp   = -100;
	min_sin_amp_f =  100;
	max_sin_amp_f = -100;
    double de_trend_data;

	for(int i= 0; i<length; i++){
		de_trend_data = sin_data_array[i] - (((i+1) * m) + q);
		if(de_trend_data > max_sin_amp)  max_sin_amp = de_trend_data;
		if(de_trend_data < min_sin_amp)  min_sin_amp = de_trend_data;
	}

	double subAmplitude =  max_sin_amp - min_sin_amp;
	double error = subAmplitude-target;

	if((error > (3*target)) || (error < (-3*target)))	subAmplitude = target;

	return subAmplitude;
}





// Reset_PID(): reset to zero PID values.

void Reset_PID(uint8_t mode){

	if (mode == OFF_MODE){
		err_int = 0;
		err_diff = 0;
		PID_out_offset = 1;
		err_prior_offset = 0;
		PID_out_old_offset = 0;
	}

	else if(mode == AMP_MODE){
		err_prior_amp=0;
		err_int_amp=0;
		err_diff_amp=0;
		PID_out_amp=1;
		PID_out_old_amp=0;
		//resetting sum and t
		sumx  = 0;
		sumy  = 0;
		sumxy = 0;
		sumx2 = 0;
		t     = 1;
		//Resetting min and max
		min_sin_amp = 100;
		max_sin_amp = -100;
		min_sin_amp_f = 100;
		max_sin_amp_f = -100;
	}

	else if(mode == BASE_MODE){
		err_int_base = 0;
		err_diff_base = 0;
		PID_out_base = 1;
		err_prior_base = 0;
		PID_out_old_base = 0;
	}

}




/* Update_PID(): computes correction based on a PID controller given:
 *               - target : ideal pressure amplitude
 *               - press : sampled pressure amplitude
 *               - K : PID coefficients
 * */

void update_PID_amp(double target,double press, double Kp_amp,double Ki_amp,double Kd_amp){

	//  target è la sinusoide, press è il valore campionato di pressione
	double err = target - press;

	err_int_amp = err_int_amp + err;

	err_diff_amp = (err-err_prior_amp);

	PID_out_amp = Kp_amp*err + Ki_amp*err_int_amp + Kd_amp*err_diff_amp;

	if(PID_out_amp<=-1){
		PID_out_amp = 1/(-PID_out_amp);
	}
	else if(PID_out_amp<=0){
		PID_out_amp = (-PID_out_amp);
	}
	/*if (PID_out_amp>(PID_out_offset+PID_out_base)){
		PID_out_amp =PID_out_offset+PID_out_base;
	}*/


	err_prior_amp = err;
	PID_out_old_amp = PID_out_amp;
}



/*
 *
 *
 *
 */
void update_PID_offset(double target,double press, double Kp,double Ki,double Kd){

	//  target è la sinusoide, press è il valore campionato di pressione
	double err = target - press;
	//ESP_LOGE(TAG,"err:%f\n\n", err);
	err_int = err_int + err;

	err_diff = (err-err_prior_offset);

	PID_out_offset = Kp*err + Ki*err_int + Kd*err_diff;

	if(PID_out_offset<=-1){
		PID_out_offset = 1/(-PID_out_offset);
	}
	else if (PID_out_offset<=0){
		PID_out_offset =-PID_out_offset;
	}

	err_prior_offset = err;
	PID_out_old_offset = PID_out_offset;
}



/*
 *
 *
 *
 */
void update_PID_base(double target,double press, double Kp,double Ki,double Kd){

	//  target è la sinusoide, press è il valore campionato di pressione
	double err = target - press;
	err_int_base = err_int_base + err;

	err_diff_base = (err-err_prior_base);

	PID_out_base = Kp*err + Ki*err_int_base + Kd*err_diff_base;




	err_prior_base = err;
	PID_out_old_base = PID_out_base;
}


//---------------------------------------------------------------------------------------------------------------------------------------------//


/* wifi_task(): Communication with PC, ask for, receive and send parameters.
 *
 */
void my_wifi_task(void *pvParameter)
{

	char strVal[10] = "";
	char strVal2[10] = "";
	char okString[dim_send_buff_8+1] = "OK";
	char Sstring[dim_send_buff_8+1] = "S";
	while(1){

		switch(STATE){

		case STOP:

			//SEND READY string
			if (stopCount == 0){
				pad(okString);
				ASCIIfromString(Sstring);
				TCPsend(buff, dim_send_buff_8);
				stopCount ++;
			}
			// wait for parameters
			if(TCPreceive()){
			//	printf("clientSock: %d\n",clientSock);
				int length = strlen(rcv_buf);
				for(int i = 1; i<length;  i++){
					strVal[i-1] = rcv_buf[i];
				}
				printf("rcv_buf: %s\tlen:%d\n",rcv_buf,length);
				switch(rcv_buf[0] - ASCII_const){

				case START_COMMAND:
					//start
					startFlag = 1;

					// Setting socket as non-blocking
					int status = fcntl(clientSock, F_SETFL, fcntl(clientSock, F_GETFL, 0) | O_NONBLOCK);
					if (status == -1){
						perror("calling fcntl");
						// handle the error.  By the way, I've never seen fcntl fail in this way
					}
					else{
						printf("Socket in non-blocking-mode\n");
					}
					break;

				case STOP_COMMAND:
					timer_pause(TIMER_GROUP_NUMBER, TIMER_N);
					DC = DC_min;
					STATE = STOP;
					//stop
					break;

				case SET_PRESSURE_AMP:
					//set pressure
					FOTparam.P_amp = atof(strVal);
					break;

				case SET_CL_BASE_AND_AMP:
					//set dutycycle
					flagPidNew = atoi(strVal);
					break;

				case SET_OFFSET:
					//set offset
					FOTparam.P_offset = atof(strVal);
					break;

				case SET_DURATION:
					//set duration
					FOTparam.duration = atof(strVal);

					break;

				case SET_FREQ:
					//set frequency
					FOTparam.freq = atoi(strVal);
					BASE_CL_winSize = (floor)(Nsamples/FOTparam.freq);
					break;

				case UPDATED_ALL:
					//finish update
					ESP_LOGW("Settings:\n",
							 "\tOffset: %.2f cmH2O\n\tAmplitude: %.2f cmH2O\n\tFrequency: %.2f Hz\n",FOTparam.P_offset, FOTparam.P_amp, FOTparam.freq);
					pad(Sstring);
					ASCIIfromString(Sstring);
					TCPsend(buff,dim_send_buff_8);   // start electron main window

					if (CONTROL){
				    	Reset_PID(OFF_MODE);
					    Reset_PID(AMP_MODE);
					    Reset_PID(BASE_MODE);
					}
					break;

				case SAVE_CL_OFF:
					//set  peak to peak ampl
					printf("Off win: %d \n",CL_OFF_winSize);
					//FOTparam.offsetCalib = atof(strVal);
					break;

				case SET_CL_OFF_WIN_SIZE:  // to be removed
					//set  peak to peak ampl
					CL_OFF_winSize = atoi(strVal);
					printf("win size offset:%d\n",CL_OFF_winSize);
					break;

				case SET_K:

					for (int i = 1; i<length-1; i++){
						strVal2[i-1] = rcv_buf[i+1];
					}
					printf("rcv_buf: %s, val: %s \n",rcv_buf, strVal2);
					if(rcv_buf[1] == 'p'){
						Kp_temp = atof(strVal2);
					}
					if(rcv_buf[1] == 'i'){
						Ki_temp = atof(strVal2);
					}
					if(rcv_buf[1] == 'd'){
						Kd_temp = atof(strVal2);
					}
					printf("Kp: %.3f\tKi: %.3f\tKd: %.3f\n\n", Kp_temp, Ki_temp, Kd_temp);
					break;

				case SET_PID_TYPE:
					printf("%s\t%c\n", rcv_buf, rcv_buf[length-1]);
					if(rcv_buf[length-1] == '0'){

						Reset_PID(AMP_MODE);
						Kp_amp = Kp_temp;
						Ki_amp = Ki_temp;
						Kd_amp = Kd_temp;
					}
					if(rcv_buf[length-1] == '1'){

						Reset_PID(OFF_MODE);
						Kprop_offset = Kp_temp;
						Kint_offset = Ki_temp;
						Kdiff_offset = Kd_temp;
					}
					if(rcv_buf[length-1] == '2'){
						Kp_baseline = Kp_temp;
						Ki_baseline = Ki_temp;
						Kd_baseline = Kd_temp;
					}

					break;

				case PID_STATE:
					if (CONTROL == MANUAL_OFFSET && (rcv_buf[length-1]-'0')){
						CONTROL = CL_AMP;
					}
					else {
						CONTROL = rcv_buf[length-1] - '0';
					}
					break;

				case MANUAL_OFF:
					CONTROL = MANUAL_OFFSET;
					manualDC =  atof(strVal);
					printf("manualDC: %.2f\n",manualDC);
					break;

				case SAVE_MANUAL_OFF:
					flag_saveOffManual = 1;
					break;

				case GET_TEMP_HUM:
					readI2C_T_H();
					readI2C_P_abs();
					data_TH_P[5] = b_lvl;
					TCPsend(data_TH_P, dim_send_buff_8);
					break;

				case RESET:
					//reset esp
					esp_restart();
					break;

				default:
					ESP_LOGE(TAG,"%s\n",rx_buff_str);
					break;
				}
			}
			vTaskDelay(DELAY_FACTOR * DELAY_WIFI / portTICK_RATE_MS);

			break;



		case STARTING:
			ESP_LOGI(TAG,"STARTING WIFI");

			if (flagFirstTime){

				// SEND DATA TO APP
				if (flagSendData){
					TCPsend(data_buff_array,dim_send_buff_8);
					flagSendData = 0;
				}
			}
			else{
				flagFirstTime = 1;
			}
			vTaskDelay(DELAY_FACTOR * DELAY_WIFI / portTICK_RATE_MS);
			break;

		case STARTED:

			// SEND ENVIRONMENTAL PARAM
			if(flagMonitorParam){
				TCPsend(data_TH_P, dim_send_buff_8);
				 flagMonitorParam = 0;
			}

			// SEND DATA TO APP
			if (flagSendFOTfreq){

				FOT_array[0] =  0xff42;
				FOT_array[1] = (int16_t) FOTparam.freq;
				/*for (uint8_t i = 0; i < dim_send_buff_16; i++ ){
					FOT_array[i] = 0;
				}*/
				TCPsend(FOT_array,dim_send_buff_8);
				flagSendFOTfreq = 0;
			}


			// SEND DATA TO APP
			if (flagSendData){
				TCPsend(data_buff_array,dim_send_buff_8);
				flagSendData = 0;
			}
			//CHECK IF DATA ARE AVAILABLE
			if(TCPreceive() == 1 && !updateFlag){
				int length = strlen(rcv_buf);
				for(int i = 1; i<length;  i++){
					strVal[i-1] = rcv_buf[i];
				}
				clock_t t;
				t = clock();
				//printf("first char: %c\n",rcv_buf[0]);
				switch(rcv_buf[0] - ASCII_const){

				case START_COMMAND:
					//start
					//updateFlag = 1;
					break;

				case STOP_COMMAND:
					//stop
					timer_pause(TIMER_GROUP_NUMBER, TIMER_N);
					DC =  delta  + DC_min;
					STATE = STOPPING;
					ESP_LOGW("Settings:\n",
							 "\tOffset: %.2f cmH2O\n\tAmplitude: %.2f cmH2O\n\tFrequency: %.2f Hz\n",FOTparam.P_offset, FOTparam.P_amp, FOTparam.freq);
					ESP_LOGW("PID :",
							"\n\t OFF:\n\t Kp %.3f\n\t Ki: %.3f\n\t Kd: %.3f\n\n\t AMP:\n\t Kp %.3f\n\t Ki: %.3f\n\t Kd: %.3f\n\n\t BASE:\n\t Kp %.3f\n\t Ki: %.3f\n\t Kd: %.3f\n",Kprop_offset,Kint_offset,Kdiff_offset,Kp_amp,Ki_amp,Kd_amp,Kp_baseline, Ki_baseline, Kd_baseline);
					break;

				case SET_PRESSURE_AMP:
					//set pressure
					FOTparam.P_amp = atof(strVal);
					break;

				case SET_CL_BASE_AND_AMP:
					//set dutycycle
					flagPidNew = atof(strVal);
					//FOTparam.ampCalib = atof(strVal);
					break;

				case SET_OFFSET:
					//set offset(in pressure)
					FOTparam.P_offset = atof(strVal);
					break;

				case SET_FREQ:
					//set frequency
					FOTparam.freq = atoi(strVal);
					BASE_CL_winSize = (floor)(Nsamples/FOTparam.freq);
					break;

				case UPDATED_ALL:
					//finish update
					ASCIIfromString(Sstring);
					TCPsend(buff,dim_send_buff_8);
					ESP_LOGW("Settings:\n",
							"\tOffset: %.2f cmH2O\n\tAmplitude: %.2f cmH2O\n\tFrequency: %.2f Hz\n",FOTparam.P_offset, FOTparam.P_amp, FOTparam.freq);

					//if(changes in param) updateflag = 1, else no + gestione target DC
					if (FOTparam.P_offset + DC_min != DC_target_value){
						updateFlag = 1;
					}
					if(CONTROL){
					   Reset_PID(OFF_MODE);
					   Reset_PID(AMP_MODE);
					   Reset_PID(BASE_MODE);
					}
					//CONTROL = CL_OFFSET;
					break;

				case SAVE_CL_OFF:
					//set  peak to peak ampl
					flagNextPID = 1;
					break;

				case SET_CL_OFF_WIN_SIZE:
					//set  peak to peak ampl
					CL_OFF_winSize = atof(strVal);
					printf("Off win: %d \n",CL_OFF_winSize);
					break;

				case SET_K:
					for (int i = 1; i<length-2; i++){
						printf("%i : %c/n",i,rcv_buf[i+1]);
						strVal2[i-1] = rcv_buf[i+1];
					}
					if(rcv_buf[1] == 'p'){
						Kp_temp = atof(strVal2);
					}
					if(rcv_buf[1] == 'i'){
						Ki_temp = atof(strVal2);
					}
					if(rcv_buf[1] == 'd'){
						Kd_temp = atof(strVal2);
					}
					break;

				case SET_PID_TYPE:
					printf("%d", length);
					if(rcv_buf[length-1] == '0'){
						Kp_amp = Kp_temp;
						Ki_amp = Ki_temp;
						Kd_amp = Kd_temp;
					}
					else if(rcv_buf[length-1] == '1'){
						Kprop_offset = Kp_temp;
						Kint_offset = Ki_temp;
						Kdiff_offset = Kd_temp;
					}
					else if(rcv_buf[length-1] == '2'){
						Kp_baseline = Kp_temp;
						Ki_baseline = Ki_temp;
						Kd_baseline = Kd_temp;
					}
					break;

				case PID_STATE:
					printf("CONTROL:%d\n",CONTROL);
					printf("rx_buff_str:%s\n",rcv_buf);
					if (CONTROL == MANUAL_OFFSET && (rcv_buf[length-1]-'0')){
						CONTROL = CL_AMP;
					}
					else {

					CONTROL = rcv_buf[length-1] - '0';
					if( CONTROL > 0 && CONTROL < 4){
						Reset_PID(CONTROL - 1);
					}
					printf("new CONTROL:%d\n",CONTROL);
					}
					break;

				case MANUAL_OFF:
					CONTROL = MANUAL_OFFSET;
					manualDC =  atof(strVal);
					printf("manualDC: %.2f\n",manualDC);
					break;

				case SAVE_MANUAL_OFF:
					flag_saveOffManual = 1;
					break;

				case GET_TEMP_HUM:
					readI2C_T_H();
					readI2C_P_abs();
					data_TH_P[5] = b_lvl;
					TCPsend(data_TH_P, dim_send_buff_8);
					break;

				default:
					ESP_LOGE(TAG,"%s\n",rx_buff_str);
					break;
				}
				t = clock() - t;
				//double time_taken = ((double)t)/CLOCKS_PER_SEC * 1000; // in ms
				vTaskDelay(DELAY_FACTOR * (int)(25-time_taken) / portTICK_RATE_MS);
			}
			else{
				vTaskDelay(DELAY_FACTOR * (DELAY_WIFI)  * 0.5/ portTICK_RATE_MS);
			}
			break;

		case TRANSITION:
		    FOT_array[0] = 0xff56;
		 /*   for (uint8_t i = 1; i < dim_send_buff_16; i++ ){
		    	 FOT_array[i] = 0;
		    }*/

		    TCPsend(FOT_array,dim_send_buff_8);
		    // SEND DATA TO APP
		    if(flagSendData){
		    	TCPsend(data_buff_array,dim_send_buff_8);
		    	flagSendData = 0;
		    }

			vTaskDelay(DELAY_FACTOR * DELAY_WIFI / portTICK_RATE_MS);
			break;

		case STOPPING:

			if (FOTparam.freq < 1 && flagMeasureEnded){
				FOT_array[0] = 0xff55;
				TCPsend(FOT_array,dim_send_buff_8);
				flagMeasureEnded = 0;
			}
			ESP_LOGE(TAG,"STOPPING WIFI\t%f\n",DC);
			// SEND DATA TO APP
			if (flagSendData){
				TCPsend(data_buff_array,dim_send_buff_8);
				flagSendData = 0;
			}
			stopCount = 0;
			vTaskDelay(DELAY_FACTOR *50* DELAY_WIFI / portTICK_RATE_MS);
			break;


		}

	}

}





void my_main_task(void *param){
	while(1){

		switch(STATE){

		/* - Setting DC = minimium DC -> MOTOR OFF
		 * - Waiting for DC to be reached
		 * - Set STATE = STARTING */
		case STOP:{
			startFlag = 0;
			DC        = DC_min;
			mcpwm_set_duty(PWM_UNIT, PWM_TIMER, PWM_OUT, 50);

			ESP_LOGE(TAG, "STOP\n");
			// ask for DC value through wi-fi
			while(startFlag == 0){

				xLastWakeTime = xTaskGetTickCount();
				vTaskDelayUntil( &xLastWakeTime, DELAY_FACTOR * 490/portTICK_PERIOD_MS);

			}
			STATE = STARTED;
			generate_Sinusoid();
			timer_start(TIMER_GROUP_NUMBER, TIMER_N);
		}
		break;

		/* Reaching the target DC
		 * - Set STATE = STARTED */

		case STARTING:{

		}
		break;

		/* - Checking if DC has changed by the user (in this case STATE = TRANSITION)
		 * - Otherwise generate PWM oscillations
		 */
		case STARTED:{
			xLastWakeTime = xTaskGetTickCount();
			if (updateFlag == 1){
				updateFlag = 0;
			}
			// read T, H, P, set flag to 1 for wi fi send
			if (countMonitorData == 10 && flagMonitorParam == 0){
				readI2C_T_H();
				readI2C_P_abs();
				data_TH_P[5]     = b_lvl;
				flagMonitorParam = 1;
				countMonitorData = 0;
				printf("----------------------------------");
				for (uint16_t i = 0; i< period_size; i++){
					printf("%.2f\t %d \n",sin_data_array[i],i);
				}
				printf("----------------------------------");
			}	else {countMonitorData++;}
			if(countMonitorData%2==0){
				//printf PID param
				ESP_LOGW("-","PID_OFF = %.2f\nPID_AMP = %.2f\nPID_BASE = %.2f\n ",PID_out_offset, PID_out_amp, PID_out_base);
				ESP_LOGI("-","amp no filter: %.2f\namp detrended: %.2f \n",(max_sin_amp - min_sin_amp),ampli_range);
				ESP_LOGE("-","max: %.2f \nmin: %.2f\nperiod: %d\n",max_sin_amp,min_sin_amp,period_size);
				printf("m: %.2f\tq: %.2f\nsumx: %.2f\tsumy: %.2f\nsumxy: %.2f\tsumx2: %.2f\nt: %d\n",m,q,sumx,sumy,sumxy,sumx2,t);
			}
			vTaskDelayUntil( &xLastWakeTime, DELAY_FACTOR * 490/portTICK_PERIOD_MS);
		}
		break;

		/* Reaching the target DC
		 * - Set STATE = STARTED */

		case TRANSITION:{
			xLastWakeTime = xTaskGetTickCount();
			updateFlag = 0;
			DC_target_value = delta + DC_min;
			if (DC_target_value - DC >= DC_step ){
				DC = DC + DC_step;
				mcpwm_set_duty(PWM_UNIT, PWM_TIMER, PWM_OUT, DC);
				vTaskDelayUntil( &xLastWakeTime, DELAY_FACTOR * 290/portTICK_PERIOD_MS);
			}
			else if (DC - DC_target_value  >= DC_step ){
				DC = DC - DC_step;
				mcpwm_set_duty(PWM_UNIT, PWM_TIMER, PWM_OUT, DC);
				vTaskDelayUntil( &xLastWakeTime, DELAY_FACTOR * 290/portTICK_PERIOD_MS);
			}
			else{
				DC = DC_target_value;
				mcpwm_set_duty(PWM_UNIT, PWM_TIMER, PWM_OUT, DC);
				vTaskDelayUntil( &xLastWakeTime, DELAY_FACTOR * 290/portTICK_PERIOD_MS);
				time_amp = time_s;
				STATE = STARTED;
			}
			xLastWakeTime = xTaskGetTickCount();
			vTaskDelayUntil( &xLastWakeTime, DELAY_FACTOR * 200/portTICK_PERIOD_MS);
		}
		break;

		/* Reaching the minimum DC
		 * - Set STATE = STOP */
		case STOPPING:{
			//ESP_LOGE(TAG,"STOPPING\n");
			xLastWakeTime = xTaskGetTickCount();
			if (DC - DC_min  >  DC_step ){
				DC = DC - (DC_step);
				mcpwm_set_duty(PWM_UNIT, PWM_TIMER, PWM_OUT, DC);
			}
			else{
				DC = DC_min;
				initVariables();
				mcpwm_set_duty(PWM_UNIT, PWM_TIMER, PWM_OUT, 50);
				timer_pause(TIMER_GROUP_NUMBER, TIMER_N);
				STATE = STOP;
			}
			vTaskDelayUntil( &xLastWakeTime, DELAY_FACTOR * 490/portTICK_PERIOD_MS);
		}
		break;
		}
		vTaskDelay(DELAY_FACTOR * 10/portTICK_PERIOD_MS);
	}
}



/* ISR DEFINITION: nb: the name has to be the same in
                       the timer_isr_register function*/

void IRAM_ATTR timer_isr(void *para){
	// READING SENSORS, compute PID and SET DC @ 500 Hz

	timer_spinlock_take(TIMER_GROUP_NUMBER);
	timer_group_clr_intr_status_in_isr(TIMER_GROUP_NUMBER, TIMER_N);
	timer_group_enable_alarm_in_isr(TIMER_GROUP_NUMBER, TIMER_N);


	period_size   =  (ceil)(Nsamples/FOTparam.freq);
	sampled_press = SPI_read();

	if (time_ms == 1000){
		time_s ++;
		time_ms = 0;
	}
	else
		time_ms += 2;

	//_______________________________STARTED STATE_________________________________________________
	if (STATE == STARTED){

		if(CONTROL == MANUAL_OFFSET){
			DC = manualDC;
			if(flag_saveOffManual == 1){
				CONTROL = CL_AMP;
				flag_saveOffManual = 0;
				PID_out_offset = (DC - DC_min) / FOTparam.P_offset;
			}
		}

		else if(CONTROL == CL_OFFSET) {

			// save data ordered in an array; If new data arrives, old data are discarted. Mean is updated
			offPID_array_sum = array_average_update(sampled_press, CL_OFF_winSize, offPID_array, offPID_array_sum);

			// window shift samples count AND a period has already passed
			if (period_index_off_avg % win_shift_off == 0 && flagStartAvg ){

				// se sono passati win_shift offset campioni esegui il pid tra media del segnale e livello di offset desiderato
				update_PID_offset(FOTparam.P_offset, offPID_array_sum/CL_OFF_winSize, Kprop_offset, Kint_offset, Kdiff_offset);
				delta = PID_out_offset;
				period_num_index++;

				//control if error < 7%
				if ((err_prior_offset < 0.10*FOTparam.P_offset) && err_prior_offset> (-0.10)*FOTparam.P_offset){
					count_error_th++;
				}
				else{
					count_error_th = 0;
				}
			}

			// updating index counting up to offset window size
			if (period_index_off_avg == CL_OFF_winSize){
				flagStartAvg = 1;
				period_index_off_avg = 0;
			}
			period_index_off_avg++;

			// updating Duty Cycle
			DC= PID_out_offset + DC_min;

			if (DC > DC_old + 0.0025 && DC < 56){
				DC = DC_old + 0.0025;
			}
			DC_old = DC;

			// Check conditions to pass next PID state
			if (count_error_th > 10 || flagNextPID||(time_s>30)){

				CONTROL = CL_AMP;
				count_error_th = 0;
				delta = PID_out_offset;
				flagNextPID = 0;
				//flagSendFOTfreq = 1;
				flagAcquisition = 1;
				period_num_index = 0;
				period_index = FOTparam.freq + 1;
				ampPID_array_sum = (offPID_array_sum / CL_OFF_winSize) * period_size;
				time_amp = time_s;

				if(period_size >= array_index){
					for (uint16_t i = 0; i<array_index; i++){
						amp_PID_array[period_size - array_index + i] = offPID_array[i];
					}
					for (uint16_t i = 0; i<period_size-array_index; i++){
						amp_PID_array[i] = offPID_array[CL_OFF_winSize - period_size + array_index + i];
					}
				}else{
					for (uint16_t i = 0; i<period_size ; i++){
						amp_PID_array[i] = offPID_array[array_index-period_size+i];
					}
				}
				array_index = 0;
			}
		}

		else if(CONTROL == CL_AMP){

			// save data ordered in an array; If new data arrivess, old data are discarted.
			ampPID_array_sum = array_average_update(sampled_press, period_size, amp_PID_array, ampPID_array_sum);

			//if (flag_base){
			//updating PID baseline compensation
			update_PID_base(FOTparam.P_offset, ampPID_array_sum/period_size, Kp_baseline, Ki_baseline, Kd_baseline);
			beta = PID_out_base;
			//}

			// computing max min each sin period of a filtered signal
/*			for (uint8_t i = 1; i<5; i++){
				y[i] = y[i-1];
				x[i] = x[i-1];
			}
			x[0] = sampled_press;
			y[0] = ( x[0] + x[1] + x[2] + x[3] + x[4] ) /5;

			//maxmin(y[0]);
			 *
			 */

			//maxmin(sampled_press);

			DC=(PID_out_amp*seno[period_index]) + PID_out_base + delta + DC_min;

			// when a period has passed the PID is updated
			if (period_index <= FOTparam.freq ){ //&& (!flag_base)

				//computing max min over a detrended period
				ampli_range = de_trend_max_min(FOTparam.P_amp, period_size-1);
				update_PID_amp(FOTparam.P_amp, ampli_range, Kp_amp, Ki_amp, Kd_amp);
				alpha=PID_out_amp;

				//checking error < 15 %
				if ((err_prior_amp < 0.15*FOTparam.P_amp) && err_prior_amp > (-0.15)*FOTparam.P_amp){
					count_error_th++;
				}
				else{
					count_error_th = 0;
				}
			}
			update_sin_array(sampled_press);

			if((count_error_th == 5 || time_s - time_amp > t_train) && !flag_base){
				// PID base on, PID out amp saved
				flag_base = 1;
				// start saving data in electron from this point
				flagSendFOTfreq = 1;
				time_save = time_s;
			}

			// save data at 250 Hz when PID on Amp is stable
			if(flag_base  && (time_s - time_save > FOTparam.duration + 5)){
				flagAcquisition = 0;
				period_num_index = 0;
			}

			// Check conditions to pass next PID state
			if ( flagAcquisition == 0 || flagNextPID){
				count_error_th = 0;
				FOTparam.freq = FOTparam.freq - 1;
				flagAcquisition = 1;
				flagNextPID = 0;
				flag_base = 0;
				STATE = TRANSITION;
				period_index = 0;
				Reset_PID(AMP_MODE);
				Reset_PID(BASE_MODE);

				// Set DC for transition
				DC = (alpha*seno[period_index]) + delta  + DC_min;

				// adjusting sin frequency if it's too low
				if (FOTparam.freq < 3.5) {

					STATE = TRANSITION;
					CONTROL = OPEN_LOOP;
					flagAcquisition = 1;
					flagSendFOTfreq = 1;
					period_num_index = 0;
					period_index = FOTparam.freq + 1;
					time_amp = time_s;
				}
			}
		}

		// OPEN LOOP with all computed values
		if(CONTROL == OPEN_LOOP){

			if( time_s > time_amp + FOTparam.duration + 5){
				period_num_index = 0;
				FOTparam.freq = FOTparam.freq - 1;
				period_index = FOTparam.freq + 1;
				flagSendFOTfreq = 1;
				STATE = TRANSITION;

				if(FOTparam.freq < 1){
					STATE = STOPPING;
					flagMeasureEnded = 1;
					DC =  delta  + DC_min;
				}

			}
			DC = (amp_coeff[(int)FOTparam.freq-1]*alpha*seno[period_index]) + delta  + DC_min;
		}
	}

	//DC control (max-min)
	if (DC > DC_max){
		DC = DC_max;
	}
	else if(DC < DC_min){
		DC = DC_min;
	}

	// SAVING DC VALUE
	(DC_value ) = DC ;

	//PERIOD INDEX RESETS WHEN A FULL SINUSOIDAL PERIOD HAS BEEN REACHED
	if (period_index >= Nsamples-FOTparam.freq){
		period_index = period_index - Nsamples;
		if (flag_base || CONTROL == OPEN_LOOP){
			period_num_index++;
		}
	}
	period_index += FOTparam.freq;
	wifi_index ++;

	//wifi_index RESETS WHEN A FULL WI-FI PERIOD EXPIRES (100s).
	if(wifi_index%SAMPLE_RATE == 0){
		*(data_storage_array + wifi_index/SAMPLE_RATE ) = (int16_t) ((DC_value ));//* 100) ; // DC cells from 1 to 10
		*(data_storage_array + wifi_index/SAMPLE_RATE + DELAY_WIFI/(2*SAMPLE_RATE) ) =  (int16_t) ((F_Sensor_Value ));// * 100); // FLOW cells from 11 to 20
		*(data_storage_array + wifi_index/SAMPLE_RATE + DELAY_WIFI/SAMPLE_RATE ) = (int16_t) ((P_Sensor_Value ) );//* 100); // PRESSURE cells from 21 to 30
	}
	//Composing array when it's full for one period (100 ms)
	if (wifi_index == DELAY_WIFI/2){
		*data_buff_array = 0b1111111101011010; // cell n. 0
		flagSendData = 1;
		count_sent ++;
		for (int i = 1; i < (dim_send_buff_16) -1 ; i++){
			*(data_buff_array + i) = *(data_storage_array+i);
		}
		*(data_buff_array + (dim_send_buff_16) - 1 ) = CONTROL * 255 + STATE;
		wifi_index = 0;
	}
	// ENABLE ISR TASK
	timer_spinlock_give(TIMER_GROUP_NUMBER);
	xSemaphoreGiveFromISR( xSemaphore,pdFALSE );
	portYIELD_FROM_ISR();
}




/*my _setup_timer() : configuring timer */

static void my_setup_timer(int timer_idx,
		bool auto_reload, double timer_interval_sec)
{
	/* Select and initialize basic parameters of the timer */
	timer_config_t config = {
			.divider = TIMER_DIVIDER,
			.counter_dir = TIMER_COUNT_UP,      // timer will count up
			.counter_en = TIMER_PAUSE,          //timer does not start yet
			.alarm_en = TIMER_ALARM_EN,         // enable alarm when overflow
			.auto_reload = auto_reload,         // set autoreload ON
	}; // default clock source is APB

	timer_init(TIMER_GROUP_NUMBER, timer_idx, &config);
	timer_set_counter_value(TIMER_GROUP_NUMBER, timer_idx, 0x00000000ULL);

	/* Configure the alarm value and the interrupt on alarm. */
	timer_set_alarm_value(TIMER_GROUP_NUMBER, timer_idx, timer_interval_sec * TIMER_SCALE);
	timer_enable_intr(TIMER_GROUP_NUMBER, timer_idx);
	timer_isr_register(TIMER_GROUP_NUMBER, timer_idx, timer_isr,
			(void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

}

void my_DC_task(void *param){

	while(1){
		xSemaphoreTake( xSemaphore, portMAX_DELAY );
		mcpwm_set_duty( PWM_UNIT, PWM_TIMER, PWM_OUT, DC);
	}
}


void app_main(void){

	*data_buff_array = 0b1111111101011010;
	xSemaphore = xSemaphoreCreateBinary();
	my_setup_ADC(ADC_WIDTH, BATTERY_LVL_CH , ADC_ATTEN);
	ESP_ERROR_CHECK(my_i2c_master_init());
	my_setup_pin();
	my_setup_pwm();
    my_wifi_init_softap();
	my_setup_timer(TIMER_N, AUTORELOAD, TIMER_INTERVAL_SEC);
	while(!AP_flag){
        vTaskDelay(DELAY_FACTOR * 500 / portTICK_RATE_MS);
	}
	my_wifi_connect_to_client();

	vTaskDelay(DELAY_FACTOR * 2000 / portTICK_RATE_MS);
	initVariables();

	b_lvl = get_battery_lvl();
	int ideal_V = 12000/2600 * 600;
	printf(" battery voltage:\nCalculated: %d\tideal:%d ",b_lvl,ideal_V);
	xTaskCreatePinnedToCore(my_main_task,"STATE management",2*2048, NULL, 1, NULL, app_cpu);
	xTaskCreatePinnedToCore(my_DC_task,"ISR PROCESSING",2*2048, NULL, 3, NULL, pro_cpu);
	xTaskCreatePinnedToCore(my_wifi_task, "wifi_task", 2*2048, NULL, 2, NULL, app_cpu);


}







