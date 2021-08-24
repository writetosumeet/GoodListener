
#include "string.h"
#include "stdio.h"

#include "sys/unistd.h"
#include "sys/stat.h"
#include "esp_spiffs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/spi.h"

//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

#include <lwip/err.h>
#include <lwip/sys.h>

#include <nvs_flash.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_http_server.h>

#include "esp_sharing.h"
#include "esp_radio.h"
#include "ringbuffer.h"
#include "vs1053.h"

static const char *TAG = "good_listener";

#define NUM_OF_FILES 10          // num of files in the ring buffer

// These are defined in vs1053.h
//#define DREQ_PIN            0   // GPIO 0
//#define VS_RST_PIN          4   // GPIO 4
//#define CS_PIN              5   // GPIO 5

// VS1053       NodeMCU         ESP8266       Color
// MOSI         D7              GPIO 13       Blue
// MISO         D6              GPIO 12       Green              
// SCK          D5              GPIO 14       Yellow
// CS           D1              GPIO 5        Orange
// DCS          3V3 (next to D4)              White
// DREQ         D3              GPIO 0        Brown
// RST          D2              GPIO 4        Grey

#define record_button       2     // GPIO 2 - Default : pull up + value = 1
#define radio_button       15    // GPIO 15 - Default : pull up + value = 0
#define share_button       16    // GPIO 16 - Default pull down + value = 0
#define play_button        3     // GPIO 3 - Default pull up + value = 1

// Button         NodeMCU         ESP8266       Button Type           Color
// record_button  D4              GPIO 2        self locking switch   Blue                        
// radio_button   D8              GPIO 15       Capacitive touch[1]   Green                       
// share_button   D0              GPIO 16       Capacitive touch[4]   Purple
// play_button    Rx              GPIO 3        self locking switch   Orange
//                3v3 (next to Tx)              Capacitive touch      Red
//                Gnd (next to Tx)              Capacitive touch      Gray
//                Gnd (next to D6)              self locking switch   White

uint8_t recording_state = RECORDER_OFF;
uint8_t playing_state = PLAYER_OFF;
uint8_t radio_state = RADIO_OFF;

const char *file_header = "/spiffs_main/file_header.ogg";

xQueueHandle record_evt_queue;
xQueueHandle play_evt_queue;
xQueueHandle radio_evt_queue;
xQueueHandle radio_data_queue;

static void record_isr_handler(void *arg)
{
  uint8_t gpio_num = (uint8_t) arg;
  
  // if player is off - only then recorder button press should be valid
  if (playing_state == PLAYER_OFF)
  {
        // if recording button press is valid
        if ( (recording_state == RECORDER_OFF) && (gpio_get_level(gpio_num)==0) )   // self locking switch pressed down 1(default) -> 0(pressed)
        {
          recording_state = START_RECORDING;
          xQueueSendFromISR(record_evt_queue, &gpio_num, NULL);  
        }     
        else if ( (recording_state == STARTED_RECORDING) && (gpio_get_level(gpio_num)==1) ) // self locking switch pressed up 0(pressed) -> 1(default)
          recording_state = STOP_RECORDING;
  }
}


static void play_isr_handler(void *arg)
{
  uint8_t gpio_num = (uint8_t) arg;
  
  // to manage RADIO button press event
  if ((playing_state == PLAYER_OFF) && (recording_state == RECORDER_OFF))
  {
    if ( (gpio_num == radio_button) && (radio_state == RADIO_OFF) ) // touchpad 1
    {
        radio_state = START_RADIO;
        xQueueSendFromISR(radio_evt_queue, &gpio_num, NULL); 
    }
    if ( (gpio_num == radio_button) && (radio_state == STARTED_RADIO) ) // touchpad 1 
    {
        radio_state = STOP_RADIO;
        xQueueSendFromISR(radio_evt_queue, &gpio_num, NULL); 
        // this is a unique scenario where radio_task needs to be triggered on stopping too 
        // since it does not have an infinite loop inside. It starts two other infinite tasks (load and unload)
    }
  }
  
  // to manage PLAYING of recorded ring (SPIFFS files)
  // if recorder is off - only then player button press should be valid
  if (recording_state == RECORDER_OFF)
  {
        // if play button press is valid
      if ( (gpio_num == play_button) && (playing_state == PLAYER_OFF) && (gpio_get_level(gpio_num)==0) )   // self locking switch pressed down 1(default) -> 0(pressed)
      {
        playing_state = START_PLAYING;
        xQueueSendFromISR(play_evt_queue, &gpio_num, NULL);  
      }     
      else if ( (gpio_num == play_button) && (playing_state == STARTED_PLAYING) && (gpio_get_level(gpio_num)==1) ) // self locking switch pressed up 0(pressed) -> 1(default)
        playing_state = STOP_PLAYING;
      else if ( (gpio_num == radio_button) && (playing_state == STARTED_PLAYING) ) // touchpad 1 - move to next file in ring - only if player button is on and state is started playing
        playing_state = PLAY_NEXT;
  }
  
}



void esp_gpio_init()
{
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;          //disable interrupt
  io_conf.pull_down_en = 0;                       //disable pull-down mode
  io_conf.pull_up_en = 0;                         //disable pull-up mode
  
  io_conf.mode = GPIO_MODE_OUTPUT;                 //set as output mode - for rst and cs
  
  // make reset pin as output & set high - vs1053 reset is active low
  io_conf.pin_bit_mask = 1UL<<VS_RST_PIN;         //bit mask
  gpio_config(&io_conf);                         //configure GPIO with given settings
  gpio_set_level(VS_RST_PIN, 1);               // make reset pin high
  
  // make chip select as output & set high - low is SCI, high is SDI
  io_conf.pin_bit_mask = (1UL<<CS_PIN);
  gpio_config(&io_conf);                         
  gpio_set_level(CS_PIN, 1);   
  
  // XDCS is set high using 3.3v output pin from esp8266 - not used as sdi_share mode is used
  
  // setup dreq pin as input
  io_conf.mode = GPIO_MODE_INPUT;                 
  io_conf.pin_bit_mask = 1UL<<DREQ_PIN;           
  gpio_config(&io_conf); 
  
  // setup 4 buttons as input
  io_conf.pin_bit_mask = (1UL<<record_button) | (1UL<<play_button);   // self locking switch - default 1 pressed 0
  io_conf.intr_type = GPIO_INTR_ANYEDGE;
  gpio_config(&io_conf); 
  
  io_conf.pin_bit_mask = (1UL<<radio_button) | (1UL<<share_button) ;  // touchpad - default 0 pressed 1
  io_conf.intr_type = GPIO_INTR_POSEDGE;
  gpio_config(&io_conf); 
  
  ESP_LOGI(TAG, "configured required gpio pins: 4 buttons, cs, dreq & rst *********** XDCS is HIGH");
}


void esp_spi_init()
{
  // setup spi configuration
  // 2MHz / CPOL=0 / CPHA=0 / Msb first
  spi_config_t spi_config;
  // Load default interface parameters
  // CS_EN:1, MISO_EN:1, MOSI_EN:1, BYTE_RX_ORDER:0, BYTE_TX_ORDER:0, BIT_RX_ORDER:0, BIT_TX_ORDER:0, CPHA:0, CPOL:0 
  spi_config.interface.val = SPI_DEFAULT_INTERFACE; // = 0x1C0 = 0001 1100 0000 
  //spi_config.interface.bit_tx_order = SPI_BIT_ORDER_MSB_FIRST; THIS CREATES PROBLEM
  //spi_config.interface.bit_rx_order = SPI_BIT_ORDER_MSB_FIRST; THIS CREATES PROBLEM
  spi_config.interface.byte_tx_order = SPI_BYTE_ORDER_MSB_FIRST;
  spi_config.interface.byte_rx_order = SPI_BYTE_ORDER_MSB_FIRST;
  spi_config.interface.cs_en = 0; // disable hardware chip select via spi
  // Load default interrupt enable
  // TRANS_DONE: true, WRITE_STATUS: false, READ_STATUS: false, WRITE_BUFFER: false, READ_BUFFER: false 
  spi_config.intr_enable.val = SPI_MASTER_DEFAULT_INTR_ENABLE; // = 0x10
  // Register SPI event callback function
  spi_config.event_cb = NULL;
  // Set SPI to master mode 
  spi_config.mode = SPI_MASTER_MODE;
  // Set the SPI clock frequency division factor
  spi_config.clk_div = SPI_2MHz_DIV; 
  
  // initialize spi mode
  spi_init(HSPI_HOST, &spi_config);
  ESP_LOGI(TAG, "spi init done with 2 MHz");
}


void partition_info()
{
  size_t total = 0, used = 0;
  // get maindata spiffs partition information
  esp_err_t ret = esp_spiffs_info("maindata", &total, &used);
  if (ret != ESP_OK) 
    ESP_LOGE(TAG, "Failed to get SPIFFS maindata partition information (%s)", esp_err_to_name(ret));
  else 
    ESP_LOGI(TAG, "maindata Partition size: total: %d, used: %d", total, used);
  
  total = 0;
  used = 0;
  // get ringdata spiffs partition information
  ret = esp_spiffs_info("ringdata", &total, &used);
  if (ret != ESP_OK) 
    ESP_LOGE(TAG, "Failed to get SPIFFS ringdata partition information (%s)", esp_err_to_name(ret));
  else 
    ESP_LOGI(TAG, "ringdata Partition size: total: %d, used: %d", total, used);
}


void esp_spiffs_init(const char* basepath, const char* partitionlabel)
{
  esp_vfs_spiffs_conf_t conf = {
    .base_path = basepath,
    .partition_label = partitionlabel,
    .max_files = 5,     // Maximum files that could be open at the same time
    .format_if_mount_failed = true
  };
  // initialize and mount spiffs
  esp_err_t ret = esp_vfs_spiffs_register(&conf);
  if (ret == ESP_OK) 
    ESP_LOGI(TAG, "%s partition spiffs initialization done", partitionlabel);
  else
  {
    if (ret == ESP_FAIL) 
      ESP_LOGE(TAG, "Failed to mount or format filesystem for %s partition", partitionlabel);
    else
    {
      if (ret == ESP_ERR_NOT_FOUND)
        ESP_LOGE(TAG, "Failed to find SPIFFS %s partition", partitionlabel);
      else
        ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s) for %s partition", esp_err_to_name(ret), partitionlabel);
    }
  }
}

void app_main(void)
{
  // v19 records header in a file, player plays header file first, then all ring and then sends endFillBytes once
  // difference from v18: main play loop is in a separate function, player plays header once and then all ring in a loop
  // no need to check every time whether header has been played or not
  ESP_LOGI(TAG, "Hello from esp to vs1053");
  ESP_LOGI(TAG, "at start of main min so far esp_get_minimum_free_heap_size() = %d", esp_get_minimum_free_heap_size());
  
  esp_gpio_init();    
  esp_spi_init();     // started at 2MHz
  // initialize spiffs maindata partition with basepath /spiffs_main
  esp_spiffs_init("/spiffs_main", "maindata");
  // initialize spiffs ringdata partition with basepath /spiffs_ring
  esp_spiffs_init("/spiffs_ring", "ringdata");
  partition_info();
  
  ring_init(NUM_OF_FILES); // number of files in the ring buffer
  
  vs_hard_reset(); // includes spi speed reduction to 2 MHz, clockF 4.5x, sdi patch
  
  ESP_LOGI(TAG, " Default level of radio_button = %d record_button = %d play_button = %d share_button = %d", gpio_get_level(radio_button), gpio_get_level(record_button), \
           gpio_get_level(play_button), gpio_get_level(share_button));

  record_evt_queue = xQueueCreate(1, sizeof(uint8_t));    // create a queue for record button pressing events
  xTaskCreate(record_task, "record_task", 2048, NULL, 8, NULL);
  
  play_evt_queue = xQueueCreate(1, sizeof(uint8_t));    // create a queue for play button pressing events
  xTaskCreate(play_task, "play_task", 2048, NULL, 8, NULL);
  
  // increased tcp/ip task stack size in menuconfig from 2560 to 3840 (x1.5) ->component config ->lwip ->tcp/ip task stack size
  // task size = 1536 + 16384 - for radio data queue
  radio_evt_queue = xQueueCreate(1, sizeof(uint8_t));    // create a queue for radio button pressing events
  xTaskCreate(radio_task, "radio_task", 17920, NULL, 8, NULL);  
  
  //install gpio isr service
  gpio_install_isr_service(0);
  //hook isr handler for specific gpio pin
  gpio_isr_handler_add(record_button, record_isr_handler, (void *) record_button);
  gpio_isr_handler_add(play_button, play_isr_handler, (void *) play_button);
  
  //gpio_isr_handler_add(share_button, gpio_isr_handler, (void *) share_button);      // on GPIO16 - does not have interrupt feature
  gpio_isr_handler_add(radio_button, play_isr_handler, (void *) radio_button); 
  
  //get_text_from_url();
  //get_music_from_url();
  
  // initialize nvs - memory
  ESP_ERROR_CHECK(nvs_flash_init());
  
  // initialize tcpip
  tcpip_adapter_init();
  ESP_LOGD(TAG, "TCPIP initialized");
  
  // initialize default event loop - for handlers
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_LOGD(TAG, "event loop created");
  
  // initialize wifi
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_LOGD(TAG, "wifi initialized with default config");
  
  // start wifi on esp in access point mode - includes starting httpd server
  start_wifi_ap_mode();
  // esp will remain in this mode except during radio is playing
   
  ESP_LOGI(TAG, "End of main.....");
  ESP_LOGI(TAG, "at end of main min so far esp_get_minimum_free_heap_size() = %d", esp_get_minimum_free_heap_size());
  
}