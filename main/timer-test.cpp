/*
* TODO:
*	- get basic test working (once replacement laser pointer comes from Amazon)
*		- testing in realistic environments (try different lighting conditions)
*	- need to get esp_now working again
*		- try to get it working based off of example on github
*			- try to avoid using the FreeRTOS queue stuff if possible, but if we need to-> refactor I guess
*			- FreeRTOS queue system seems to be entirely different from wifi/esp_now stuff
*				- do we want to forego-> just use the globals like we've been doing, and be careful about checking with mutexes so we don't overwrite anything important
*	- issue with task watchdog????
*		- periodically feed the watchdog-> slight overhead but the simplest approach by far
*		- look into disabling watching of IDLE Task on Core 0?
*			- https://www.freertos.org/RTOS-idle-task.html : "It is therefore important in applications that make use of the vTaskDelete() function 
*			to ensure the idle task is not starved of processing time. The idle task has no other active functions so can legitimately be starved 
*			of microcontroller time under all other conditions."
*	- add another timer to "timeout" the waits for various pin interrupts? Or attach a different "break" interrupt to a pin connected to a button?
*		- looks doable via dynamic timer alarms....
*	- buy more "final" parts and start integrating those
*		- OLED+battery ESP32 model for finish line
*		- battery ESP32 model for start line
*	- interrupts for button presses in future
*		- https://espressif-docs.readthedocs-hosted.com/projects/esp8266-rtos-sdk/en/release-v3.4/api-reference/peripherals/gpio.html
*		- gpio_install_isr_service()
*		-  gpio_isr_handler_add()
* NOTES:s
*
* 	- Attaching USB device via WSL
*		- https://learn.microsoft.com/en-us/windows/wsl/connect-usb
*		- https://github.com/microsoft/WSL/issues/7652
*		- flashing issues
*			- https://stackoverflow.com/questions/73923341/unable-to-flash-esp32-the-port-doesnt-exist
*			- PowerShell w/ admin privileges:
*				- usbipd wsl list
*				- usbipd wsl attach --busid <busid>
*			- WSL console
*				- lsusb
*				- sudo chmod a+rw /dev/ttyUSB0
*				- flash with given command
* 
*/

#include <esp_wifi.h> // need to init wifi in order to use esp_now protocol
#include <nvs_flash.h> // need to init the flash memory in order to init the wifi
#include <esp_mac.h> // grabbing the mac address for testing purposes
#include <esp_now.h> // device-to-device communication
#include <driver/gptimer.h> // access to the hardware timer
#include <driver/gpio.h> // gpio pin stuff
//#include <esp_task_wdt.h> // messing with the watchdog
#include <pthread.h> // mutexes for message receieved, sent flags (get rid of after implementing FreeRTOS's message queue system???)
#include <memory.h> // memcpy
#include "Timer_Msg.h" // program-defined statuses for passing esp_now messsages between the start and finish line modules

// wrapping serial prints using #if defined() guards to make the release version slimmer
// automatically set in CmMakeLists.txt according to the configuration held in sdkconfig
#if defined(DEBUG) // if it's a debug build, we want serial prints 
	#define DBG_SERIAL_PRINT(fmt, ...) printf(fmt __VA_OPT__(,) __VA_ARGS__)
#elif defined(RELEASE) // otherwise in a release build have the preprocessor take them out
	#define DBG_SERIAL_PRINT(fmt, ...)
#else // new or unexpected environment-> we probably want prints, should we throw some kind of warning here?
	#define DBG_SERIAL_PRINT(...) printf(__VA_ARGS__)
#endif // DEBUG PRINTING
/*
* Start/Finish Line Build #defines
*/
//#define START_LINE_BUILD
#define FINISH_LINE_BUILD

/*
* Other #defines 
*	- GPIO pin numbers, timer clock info, etc.
*/
// surprised these two aren't defined in gpio.h
#define LOW							0
#define HIGH						1
#define SYNC_PULSE_PIN				(gpio_num_t::GPIO_NUM_18) // 18, pin to connect boards to one another to sync hardware timer start
#define LASER_PIN					(gpio_num_t::GPIO_NUM_4) // 4,pin hooked up to photodiode that reads laser contact
#define HW_TIMER_FREQ				(80000000 / 2) // frequency in MHz, 40MHz seems to be as fast as it can go (couldn't find explicit cap in the docs)
#define HW_TIMER_TICKS_PER_SEC 		HW_TIMER_FREQ // just for readability
/*
* - Calling these two functions together feeds the freeRTOS watchdog
* 	- If the watchdog isn't fed, it causes an error and the board resets
* 	- The watchdog could be disabled, but that hurts stability
* 	- This causes about a 0.000945second delay, so with respect to what 
* 	we're timing, that's an acceptable overhead
* 	- can't call vTaskDelay(0) because that doesn't actually yield to lower
*	priority tasks
*/
//  Component config > ESP System Settings > (for disabling/enabling watchdog monitoring of idle task on each cpu core)
// comment back out if we need this...
//#define WATCHDOG_APPEASE			esp_task_wdt_reset(); vTaskDelay(1);
/*
* Want to compile-out these error code grabs in the release build
*/
#if defined(DEBUG)
	#define DBG_ERR_TO_NAME_R(x)	esp_err_to_name_r(x, g_err_buff, sizeof(g_err_buff))
#elif defined(RELEASE)
	#define DBG_ERR_TO_NAME_R(x)		
#endif //DBG/RELEASE error to name macro

/*
* ESP-NOW defines 
* LOOK AT THESE AGAIN??????
*/
//#define ESPNOW_WIFI_MODE 			WIFI_MODE_STA // this define was in the config file of the example I was following...just going to put in WIFI_MODE_STA directly
#if !defined(CONFIG_ESPNOW_CHANNEL)
	#define CONFIG_ESPNOW_CHANNEL	1
#endif // CONFIG_ESPNOW_CHANNEL

/*
* Globals
*/
#if defined(FINISH_LINE_BUILD)
uint64_t g_start_time_ticks;
uint64_t g_finish_time_ticks;
#endif // FINISH_LINE_BUILD
volatile bool g_msg_recv = false; 
volatile bool g_msg_sent = false;
pthread_mutex_t g_msg_recv_lock = PTHREAD_MUTEX_INITIALIZER;  // are callbacks still async? (completed on the 2nd core)
pthread_mutex_t g_msg_sent_lock = PTHREAD_MUTEX_INITIALIZER;
const uint8_t g_start_MAC_addr[] = { 0x78, 0x21, 0x84, 0xE1, 0xF0, 0xB4 };
const uint8_t g_finish_MAC_addr[] = { 0xEC, 0x94, 0xCB, 0x6B, 0x03, 0xA8 };
timer_msg g_msg_out, g_msg_in;
gptimer_handle_t hw_timer;
#if defined(DEBUG)
char g_err_buff[512];
#endif // DEBUG

void on_msg_sent(const uint8_t* mac_addr, esp_now_send_status_t status) 
{
	pthread_mutex_lock(&g_msg_sent_lock);
	if (status == ESP_NOW_SEND_SUCCESS) {
		g_msg_sent = true;
		DBG_SERIAL_PRINT("Message sent!\n");
	}else {	
		g_msg_sent = false;
		DBG_SERIAL_PRINT("Error sending message! Error: %s\n", DBG_ERR_TO_NAME_R(status));
	}
	pthread_mutex_unlock(&g_msg_sent_lock);
}

void on_msg_recv(const uint8_t* rcv_info, const uint8_t* incoming_data, int len)
//void on_msg_recv(const esp_now_recv_info_t* rcv_info, const uint8_t* incoming_data, int len)
{
	pthread_mutex_lock(&g_msg_recv_lock);
	DBG_SERIAL_PRINT("Message receieved!\n");
	g_msg_recv = true;
	memcpy(&g_msg_in, incoming_data, sizeof(g_msg_in));
	pthread_mutex_unlock(&g_msg_recv_lock);
}

// basic idea is to check if the laser is reliably directed at the photodiode
	// can play around with times but for now maybe 1 second of constant connection?
// at this point the hardware timer should be running so we can use that
bool laser_line_ready()
{
	bool laser_const_contact = true;
	uint64_t start_ticks = 0, curr_ticks = 0;
	esp_err_t err = ESP_OK;
	if((err = gptimer_get_raw_count(hw_timer, &start_ticks)) != ESP_OK) {
		DBG_SERIAL_PRINT("Failed to get the timer's (start) count! Error: %s\n", DBG_ERR_TO_NAME_R(err));
		return false;
	}

	while(true) {
		if((err = gptimer_get_raw_count(hw_timer, &curr_ticks) )!= ESP_OK) { // skip error checking for performance?
			DBG_SERIAL_PRINT("Failed to get the timer's (curr) count! Error: %s\n", DBG_ERR_TO_NAME_R(err));
			return false;
		}
		if((curr_ticks - start_ticks) <= HW_TIMER_TICKS_PER_SEC) { // avoid the expensive division
			if (gpio_get_level(LASER_PIN) == LOW) {
				laser_const_contact = false;
				break;
			}
		}
		else{
			break;
		}
	}

	return laser_const_contact;
}

// several paired functions like these will exist, with a different version for the start/finish line boards
#if defined(START_LINE_BUILD)
timer_status_t timing_run_start_line()
{
	// check to make sure signal coming in from both modules
	// signal an ok if so, otherwise indicate error and return back to the selection "menu"
	// send the esp_now message with the start time
	// wait for a run finished message-> want some sort of timeout here?
		// worst case we could add functionality for the reset button

	esp_err_t err = ESP_OK;
	if (laser_line_ready()) {
		g_msg_out.status = timer_status_t::TIMER_READY;
		if((err = esp_now_send(g_finish_MAC_addr, (uint8_t*)&g_msg_out, sizeof(g_msg_out))) != ESP_OK) { // and send it to the finish line module...
			DBG_SERIAL_PRINT("Failed to send ready message from start to finish! Error: %s\n", DBG_ERR_TO_NAME_R(err));
			return timer_status_t::TIMER_MSG_ERR;
		}

	}
	else {
		DBG_SERIAL_PRINT("Start line laser not properly aligned!\n");
		g_msg_out.status = timer_status_t::TIMER_NREADY;
		if((err = esp_now_send(g_finish_MAC_addr, (uint8_t*)&g_msg_out, sizeof(g_msg_out))) != ESP_OK) { // and send it to the finish line module...
			DBG_SERIAL_PRINT("Failed to send not-ready message from start to finish! Error: %s\n", DBG_ERR_TO_NAME_R(err));
		}
		return timer_status_t::TIMER_SL_LASER_NALIGN;
	}

	while (gpio_get_level(LASER_PIN) == HIGH) { // wait until the start line is crossed
		DBG_SERIAL_PRINT("Start line waiting\n");
		//WATCHDOG_APPEASE;
	} 

	if((err = gptimer_get_raw_count(hw_timer, &(g_msg_out.time))) != ESP_OK) { // record the start time...
		DBG_SERIAL_PRINT("Failed to get raw timer count! Error: %s\n", DBG_ERR_TO_NAME_R(err));
		return timer_status_t::TIMER_ERROR;
	}
	g_msg_out.status = timer_status_t::TIMER_DATA;

	if((err = esp_now_send(g_finish_MAC_addr, (uint8_t*)&g_msg_out, sizeof(g_msg_out))) != ESP_OK) { // and send it to the finish line module...
		DBG_SERIAL_PRINT("Error sending start time! Error: %s\n", DBG_ERR_TO_NAME_R(err));
		return timer_status_t::TIMER_MSG_ERR;
	}

	return timer_status_t::TIMER_OK;
}
#elif defined(FINISH_LINE_BUILD)
timer_status_t timing_run_finish_line()
{
	esp_err_t err = ESP_OK;
	// check to make sure signal coming in from both modules
	if (!laser_line_ready()) {
		DBG_SERIAL_PRINT("Finish line laser not properly aligned!\n");
		return timer_status_t::TIMER_FL_LASER_NALIGN;
	}

	g_msg_out.status = timer_status_t::TIMER_RUN_ENQ;
	if((err = esp_now_send(g_start_MAC_addr, (uint8_t*)&g_msg_out, sizeof(g_msg_out))) != ESP_OK) {
		DBG_SERIAL_PRINT("Failed to send enquiry message to the starting line module! Error: %s\n", DBG_ERR_TO_NAME_R(err));
		return timer_status_t::TIMER_MSG_ERR;
	}

	// wait until we get a reply
	// need to add a way to break out of this if it's taking too long...
		// either add a timeout function, or allow a button press on the device
		// details details...
	timer_status_t peer_status;
	uint64_t peer_time;
	while (true) {
		pthread_mutex_lock(&g_msg_recv_lock);
		if (g_msg_recv) {
			peer_status = g_msg_in.status;
			g_msg_recv = false; // reset the flag
			pthread_mutex_unlock(&g_msg_recv_lock);
			break;
		}
		pthread_mutex_unlock(&g_msg_recv_lock);
	}
	
	if (peer_status != timer_status_t::TIMER_READY) {
		DBG_SERIAL_PRINT("Starting module indicated non-ready status\n");
		return peer_status;
	}

	DBG_SERIAL_PRINT("Ready for a timing run!");

	// wait until the finish line has been crossed
	while (gpio_get_level(LASER_PIN) == HIGH) {
		DBG_SERIAL_PRINT("Finish line waiting\n");
		//WATCHDOG_APPEASE;
	}

	// record the count from the hardware timer
	//g_finish_time_ticks = timerRead(hw_timer);
	if((err = gptimer_get_raw_count(hw_timer, &g_finish_time_ticks)) != ESP_OK) {
		DBG_SERIAL_PRINT("Failed to get the timer's raw count! Error: %s\n", DBG_ERR_TO_NAME_R(err));
		return timer_status_t::TIMER_ERROR;
	}

	pthread_mutex_lock(&g_msg_recv_lock);
	if (g_msg_recv) {
		peer_status = g_msg_in.status;
		peer_time = g_msg_in.time;
		g_msg_recv = false; // reset the flag
		pthread_mutex_unlock(&g_msg_recv_lock);
	}
	else {
		pthread_mutex_lock(&g_msg_recv_lock);
		DBG_SERIAL_PRINT("Never received a starting time!\n");
		return timer_status_t::TIMER_ERROR;
	}
	
	if (peer_status == timer_status_t::TIMER_DATA) {
		g_start_time_ticks = peer_time;
	} else {
		DBG_SERIAL_PRINT("Received unexpected status from starting module in data packet\n");
		return peer_status;
	}
	
	// other common-sense checks we can do on the received data?
	if (g_start_time_ticks >= g_finish_time_ticks) {
		DBG_SERIAL_PRINT("Received invalid timer values!\n");
		return timer_status_t::TIMER_ERROR;
	}
	long double time_sec = (long double)(g_start_time_ticks - g_finish_time_ticks) / (HW_TIMER_TICKS_PER_SEC); // is casting both necessary?

	DBG_SERIAL_PRINT("Run time: %Lf\n", time_sec);
	// Display the calculated time
	// for now just a print over the serial port

	return TIMER_OK;
}
#endif //timing_run_start/finish_line

// ???make this return a bool, if true proceed, if false restart the board???
bool setup()
{
	esp_err_t setup_err = ESP_OK;

	#if defined(START_LINE_BUILD)
	DBG_SERIAL_PRINT("\nSTART LINE BUILD: ");
	#elif defined(FINISH_LINE_BUILD)
	DBG_SERIAL_PRINT("\nFINISH LINE BUILD: ");
	#endif // START/FINISH Line id prints
	DBG_SERIAL_PRINT("Setting things up...\n");

	//if((setup_err = esp_task_wdt_add(NULL)) != ESP_OK){
	//	DBG_SERIAL_PRINT("Failed to add the current task to the watchdog timer! Error: %s\n", DBG_ERR_TO_NAME_R(setup_err));
	//}
	/*
	*
	*	WiFi SET-UP:
	*
    */
	if((setup_err = nvs_flash_init()) != ESP_OK) {
		if ((setup_err == ESP_ERR_NVS_NO_FREE_PAGES) || (setup_err == ESP_ERR_NVS_NEW_VERSION_FOUND)) { // recoverable errors
			if((setup_err = nvs_flash_erase()) != ESP_OK) {
				DBG_SERIAL_PRINT("Error initialzing (erasing) flash! Error: %s\n", DBG_ERR_TO_NAME_R(setup_err));
				return false;
			}
			if((setup_err = nvs_flash_init()) != ESP_OK) {
				DBG_SERIAL_PRINT("Error initialzing (re-initing) flash! Error: %s\n", DBG_ERR_TO_NAME_R(setup_err));
				return false;
			}
		} else {
			DBG_SERIAL_PRINT("Error initialzing the nvs flash! Error: %s\n", DBG_ERR_TO_NAME_R(setup_err));
			return false;
		}
	}

	if ((setup_err = esp_netif_init()) != ESP_OK) {
		DBG_SERIAL_PRINT("Error initialzing netif! Error: %s\n", DBG_ERR_TO_NAME_R(setup_err));
		return false;
	}
	// Event loop necessary for wifi functionality-> FreeRTOS's implementation?
	if((setup_err = esp_event_loop_create_default()) != ESP_OK) {
		DBG_SERIAL_PRINT("Failed to create the default event loop! Error: %s\n", DBG_ERR_TO_NAME_R(setup_err));
		return false;
	}
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	if ((setup_err = esp_wifi_init(&cfg)) != ESP_OK) {
		DBG_SERIAL_PRINT("Error initialzing WiFi! Error: %s\n", DBG_ERR_TO_NAME_R(setup_err));
		return false;
	}
	if((setup_err = esp_wifi_set_storage(WIFI_STORAGE_RAM)) != ESP_OK) {
		DBG_SERIAL_PRINT("Error setting WiFi storage! Error: %s\n", DBG_ERR_TO_NAME_R(setup_err));
		return false;
	}
	if((setup_err = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK) {
		DBG_SERIAL_PRINT("Error setting WiFi mode! Error: %s\n", DBG_ERR_TO_NAME_R(setup_err));
		return false;
	}
	if((setup_err = esp_wifi_start()) != ESP_OK) {
		DBG_SERIAL_PRINT("Error starting WiFi! Error: %s\n", DBG_ERR_TO_NAME_R(setup_err));
		return false;
	}
	if((setup_err = esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE)) != ESP_OK) {
		DBG_SERIAL_PRINT("Error setting the WiFi channel! Error: %s\n", DBG_ERR_TO_NAME_R(setup_err));
		return false;
	}

    #if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    if((setup_err = esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR)) != ESP_OK) {
    	DBG_SERIAL_PRINT("Error setting the WiFi protocol in long range case! Error: %s\n", DBG_ERR_TO_NAME_R(setup_err));	
		return false;
    }
	#endif

	uint8_t mac_base[6] = {0};
	esp_efuse_mac_get_default(mac_base);
	printf("MAC Address: ");
	for(uint8_t i = 0; i < sizeof(mac_base); i++) {
		if(i < sizeof(mac_base) - 1) {
			printf("%02X:", mac_base[i]);
		} else{
			printf("%02X\n", mac_base[i]);
		}
		
	}
	#if defined(START_LINE_BUILD)
	if(memcmp(mac_base, g_finish_MAC_addr, sizeof(mac_base)) == 0) {
		DBG_SERIAL_PRINT("\n\nWARNING\n\nFLASHED START LINE BUILD TO FINISH LINE MODULE!\n\n");
	}
	#elif defined(FINISH_LINE_BUILD)
	if(memcmp(mac_base, g_start_MAC_addr, sizeof(mac_base)) == 0) {
		DBG_SERIAL_PRINT("\n\nWARNING\n\nFLASHED FINISH LINE BUILD TO START LINE MODULE!\n\n");
	}
	#endif

    /*
	*
	*	ESP_NOW SET-UP:
	*
    */
	if ((setup_err = esp_now_init()) != ESP_OK) {
		DBG_SERIAL_PRINT("Error initializing ESP-NOW! Error: %s\n", DBG_ERR_TO_NAME_R(setup_err));
		return false;
	}
	esp_now_peer_info_t peer_info;
	// register the other esp32 as a peer
	#if defined(START_LINE_BUILD)
	memcpy(peer_info.peer_addr, g_finish_MAC_addr, sizeof(g_finish_MAC_addr));
	#elif defined(FINISH_LINE_BUILD)
	memcpy(peer_info.peer_addr, g_start_MAC_addr, sizeof(g_start_MAC_addr));
	#endif // peer_info, start/finish 
	peer_info.channel = 0;
	peer_info.encrypt = false; // might want to look into this later
	// Add peer        
	if ((setup_err = esp_now_add_peer(&peer_info)) != ESP_OK) {
		DBG_SERIAL_PRINT("Failed to add peer! Error: %s\n", DBG_ERR_TO_NAME_R(setup_err));
		return false;
	}
	// register callbacks...
	if ((setup_err = esp_now_register_send_cb(on_msg_sent)) != ESP_OK) { // for data sent
		DBG_SERIAL_PRINT("Failed to register send callback function! Error: %s\n", DBG_ERR_TO_NAME_R(setup_err));
		return false;
	}
	if ((setup_err = esp_now_register_recv_cb(on_msg_recv)) != ESP_OK) { // and data received
		DBG_SERIAL_PRINT("Failed to register receive callback function! Error: %s\n", DBG_ERR_TO_NAME_R(setup_err));
		return false;
	}

	// initialize mutexes
	if(pthread_mutex_init(&g_msg_recv_lock, NULL) != 0) {
	  DBG_SERIAL_PRINT("Failed to initialize the g_msg_recv_lock mutex");
	  return false;
	}
	if(pthread_mutex_init(&g_msg_sent_lock, NULL) != 0) {
	  DBG_SERIAL_PRINT("Failed to initialize the g_msg_recv_lock mutex");
	  return false;
	}

	/*
	*
	*	TIMER/ SYNC PULSE PIN SET-UP:
	*
    */
	gpio_config_t sync_pulse_pin_config = {
		.pin_bit_mask = 1ULL<<(unsigned)SYNC_PULSE_PIN,
		.mode = gpio_mode_t::GPIO_MODE_INPUT,
		.pull_up_en = gpio_pullup_t::GPIO_PULLUP_DISABLE,
		.pull_down_en = gpio_pulldown_t::GPIO_PULLDOWN_ENABLE,
		.intr_type =  gpio_int_type_t::GPIO_INTR_DISABLE
	};
	if((setup_err = gpio_reset_pin(SYNC_PULSE_PIN)) != ESP_OK) {
		DBG_SERIAL_PRINT("Failed to reset GPIO pin %d! Error: %s\n", SYNC_PULSE_PIN, DBG_ERR_TO_NAME_R(setup_err));
		return false;
	}
	if((setup_err = gpio_config(&sync_pulse_pin_config)) != ESP_OK) {
		DBG_SERIAL_PRINT("Failed to configure GPIO pin %d! Error: %s\n", SYNC_PULSE_PIN, DBG_ERR_TO_NAME_R(setup_err));
		return false;
	}
	
	gptimer_config_t timer_config = {
		.clk_src = GPTIMER_CLK_SRC_DEFAULT,
		.direction = GPTIMER_COUNT_UP,
		.resolution_hz = HW_TIMER_FREQ, // 1 MHz
		.flags{.intr_shared = 1} // need to tweak this to use the timer with ISRs?
	};
	if((setup_err = gptimer_new_timer(&timer_config, &hw_timer)) != ESP_OK) {
		DBG_SERIAL_PRINT("Failed to instantiate a new timer instance. Error: %s\n", DBG_ERR_TO_NAME_R(setup_err));
		return false;
	}
	if((setup_err = gptimer_enable(hw_timer)) != ESP_OK) {
		DBG_SERIAL_PRINT("Failed to enable the timer instance. Error: %s\n", DBG_ERR_TO_NAME_R(setup_err));
		return false;
	}

	/*uint32_t timer_res;
	gptimer_get_resolution(hw_timer, &timer_res);
	DBG_SERIAL_PRINT("Timer resolution: %luHz\n", timer_res);*/
	
	// wait for the pin to read HIGH
	while (gpio_get_level(SYNC_PULSE_PIN) == LOW) { // gpio.h
		//DBG_SERIAL_PRINT("Pin low");
		//WATCHDOG_APPEASE;
	}

	if((setup_err = gptimer_start(hw_timer)) != ESP_OK) {
		DBG_SERIAL_PRINT("Failed to start the timer instance. Error: %s\n", DBG_ERR_TO_NAME_R(setup_err));
		return false;
	}

	DBG_SERIAL_PRINT("Sync pulse received!\n");

	/*
	*
	*	FINISH MODULE CONFIRMS START MODULE RECEIVED SYNC PULSE:
	*
    */
	#if defined(START_LINE_BUILD)
	g_msg_out.status = timer_status_t::TIMER_SYNC_PULSE_RCV;
	if((setup_err = esp_now_send(g_finish_MAC_addr, (uint8_t*)&g_msg_out, sizeof(g_msg_out))) != ESP_OK) { // and send it to the finish line module...
		DBG_SERIAL_PRINT("Error sending sync pulse confirmation! Error: %s\n", DBG_ERR_TO_NAME_R(setup_err));
		return false;
	}
	#elif defined(FINISH_LINE_BUILD)
	timer_status_t peer_status = timer_status_t::TIMER_ERROR;
	while (true) {
		//WATCHDOG_APPEASE;
		pthread_mutex_lock(&g_msg_recv_lock);
		if (g_msg_recv) {
			peer_status = g_msg_in.status;
			g_msg_recv = false; // reset the flag
			pthread_mutex_unlock(&g_msg_recv_lock);
			DBG_SERIAL_PRINT("Received sync pulse confirmation from the start line module!\n");
			break;
		}
		pthread_mutex_unlock(&g_msg_recv_lock);
	}
	if(peer_status != timer_status_t::TIMER_SYNC_PULSE_RCV) {
		DBG_SERIAL_PRINT("Received unexpected status from the start line module!");
		return false;
	}
	#endif //START/FINISH Line sync pulse confirm
	// need to add some timeout feature here? Or just let it spin, since the modules are useless without a sync pulse...

	// testing the overhead we incur by keeping the watchdog happy
	/*
	DBG_SERIAL_PRINT("Tick rate: %lu\n", portTICK_PERIOD_MS);
	DBG_SERIAL_PRINT("Tick rate: %d\n", configTICK_RATE_HZ);
	uint64_t start, finish;
	gptimer_get_raw_count(hw_timer, &start);
	esp_task_wdt_reset();
	vTaskDelay(1); // tick rate was set to 1000 in sdkconfig
	gptimer_get_raw_count(hw_timer, &finish);
	DBG_SERIAL_PRINT("Time delay: %llu\n", finish - start);
	*/
	// and the delay by getting around the watchdog
	/*uint64_t start, finish;
	gptimer_get_raw_count(hw_timer, &start);
	while(gpio_get_level(SYNC_PULSE_PIN) == HIGH);
	gptimer_get_raw_count(hw_timer, &finish);
	DBG_SERIAL_PRINT("Time delay: %llu\n", finish - start);*/
	
	if((setup_err = gpio_reset_pin(SYNC_PULSE_PIN)) != ESP_OK) {
		DBG_SERIAL_PRINT("Failed to reset GPIO pin %d! Error: %s\n", SYNC_PULSE_PIN, DBG_ERR_TO_NAME_R(setup_err));
		return false;
	}

	/*
	*
	*	LASER PIN SET-UP:
	*
    */
	if((setup_err = gpio_reset_pin(LASER_PIN)) != ESP_OK) { // good practice to reset pins before using...
		DBG_SERIAL_PRINT("Failed to reset GPIO pin %d! Error: %s\n", LASER_PIN, DBG_ERR_TO_NAME_R(setup_err));
		return false;
	}

	// test out pulldown mode here...?
	gpio_config_t laser_pin_config = {
		.pin_bit_mask = 1ULL<<(unsigned)LASER_PIN,
		.mode = gpio_mode_t::GPIO_MODE_INPUT,
		.pull_up_en = gpio_pullup_t::GPIO_PULLUP_DISABLE,
		.pull_down_en = gpio_pulldown_t::GPIO_PULLDOWN_DISABLE,
		.intr_type =  gpio_int_type_t::GPIO_INTR_DISABLE
	};
	if((setup_err = gpio_config(&laser_pin_config)) != ESP_OK) {
		DBG_SERIAL_PRINT("Failed to configure GPIO pin %d! Error: %s\n", SYNC_PULSE_PIN, DBG_ERR_TO_NAME_R(setup_err));
		return false;
	}

	return true;
}

// don't want to return from app_main, look into some other kind of reset
extern "C" void app_main(void)
{
	timer_status_t peer_status;

	if(!setup()) {
		DBG_SERIAL_PRINT("Setup failed!");
		esp_restart(); // doesn't reset WiFi and Bluetooth-> potential issues?
	}

	while(true) {
		#if defined(START_LINE_BUILD)
		while(true) {
			//WATCHDOG_APPEASE;
			pthread_mutex_lock(&g_msg_recv_lock);
			if (g_msg_recv) {
				peer_status = g_msg_in.status;
				g_msg_recv = false; // reset the flag
				pthread_mutex_unlock(&g_msg_recv_lock);
				break;
			}
		}
		if (peer_status != timer_status_t::TIMER_RUN_ENQ) {
			DBG_SERIAL_PRINT("Received unexpected status from finish line module.");
			continue; // try again
		}
		timing_run_start_line();
		#elif defined(FINISH_LINE_BUILD)
		DBG_SERIAL_PRINT("Hello");
		timing_run_finish_line();
		#endif // START/END_LINE_BUILD
	}
}