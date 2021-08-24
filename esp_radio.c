/* Hello Esp

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sys/unistd.h"
#include "sys/stat.h"
#include "esp_spiffs.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include <esp_log.h>
#include <lwip/err.h>
#include <lwip/sys.h>
#include "esp_netif.h"

#include <nvs_flash.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_http_server.h>
#include <netdb.h>
#include <sys/socket.h>

#include "esp_radio.h"
#include "vs1053.h"
#include "esp_sharing.h"
//#include "SampleMp3.h"
//#include "html_chunks.h"
//#include "ringbuffer.h"

static const char *TAG = "esp_radio";

// These should be stored and read from metadata files
#define WEB_WIFI_SSID "CuriePhoneSony"
#define WEB_WIFI_PASSWORD "sumeetsharma"




void wifi_stn_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
      if (event_id == WIFI_EVENT_STA_START)                /**< station start */
      {
        ESP_LOGI(TAG, "station start");
      }
      if (event_id == WIFI_EVENT_STA_STOP)                 /**< station stop */
      {
        ESP_LOGI(TAG, "station stop");
      }
      if (event_id == WIFI_EVENT_STA_CONNECTED)            /**< station connected to AP */
      {
          ESP_LOGD(TAG, "WIFI_EVENT_STA_CONNECTED - esp connected as station to wifi");
      }
      if (event_id == WIFI_EVENT_STA_DISCONNECTED)         /**< station disconnected from AP */
      {
          ESP_LOGD(TAG, "WIFI_EVENT_STA_DISCONNECTED - esp disconnected as station");
          radio_state = STOP_RADIO;
          uint8_t gpio_num = 15;    // radio button is connected on GPIO 15
          xQueueSend(radio_evt_queue, &gpio_num, 10 / portTICK_RATE_MS); 
          
          
        // can perhaps attempt to reconnect using the following code:
        /* system_event_sta_disconnected_t *event = (system_event_sta_disconnected_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi disconnected, trying to reconnect...");
        if (event->reason == WIFI_REASON_BASIC_RATE_NOT_SUPPORT) 
        {
            //Switch to 802.11 bgn mode 
            esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
        }
        ESP_ERROR_CHECK(esp_wifi_connect());*/
        
      }
}

void on_got_ip(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
  ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
  ESP_LOGI(TAG, "IPv4 address: " IPSTR, IP2STR(&event->ip_info.ip));
}


void start_wifi_stn_mode()
{

  // register wifi event handlers
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_stn_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip, NULL));  
  ESP_LOGD(TAG, "wifi and IP event handlers registered");
  
  //ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM)); - check if really needed. default is WIFI_STORAGE_FLASH
  // configure wifi  
  wifi_config_t wifi_config = { 0 };
  strcpy((char *)&wifi_config.sta.ssid, WEB_WIFI_SSID);
  strcpy((char *)&wifi_config.sta.password, WEB_WIFI_PASSWORD);
  
  // setup station mode
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_LOGD(TAG, "wifi Station mode set");
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
  ESP_LOGD(TAG, "wifi new config set");
  
  // start wifi
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_LOGD(TAG, "wifi started");
  
  ESP_ERROR_CHECK(esp_wifi_connect());
  //vTaskDelay(2000 / portTICK_PERIOD_MS);    // this delay is only to give time to esp to connect to wifi router before next statement/task is executed (which may depend on this connection)
  
}

void stop_wifi_stn_mode()
{
  ESP_ERROR_CHECK(esp_wifi_disconnect());
  ESP_ERROR_CHECK(esp_wifi_stop());
  ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_stn_event_handler));
  ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip));
  ESP_LOGD(TAG, "wifi in station mode stopped");
  //vTaskDelay(2000 / portTICK_PERIOD_MS);
}


int radio_socket_connect()
{
  uint8_t num_of_tries=5, i=0;
  int radio_socket= -1;
  
  char* radio_server = "stream.live.vc.bbcmedia.co.uk";    
  char* radio_uri = "/bbc_world_service";
  uint16_t radio_port = 80;
  
  struct hostent *server = NULL;
  char* radio_ip_addr;
  struct sockaddr_in dest;
  
  char radio_request[300];
  strcpy(radio_request, "GET ");
  strcat(radio_request, radio_uri);
  strcat(radio_request, " HTTP/1.0\r\nHost: ");
  strcat(radio_request, radio_server);
  strcat(radio_request, "\r\nUser-Agent: myesp8266\r\n\r\n");
  
    // setup socket connection 
    for (i=0; i<num_of_tries; i++)
    {
                ////*************** Get IP address of radio server **************///
                server = (struct hostent*)gethostbyname(radio_server);
                if (server == NULL)
                {
                  //ESP_LOGE(TAG, "DNS lookup failed err=%d", err);
                  ESP_LOGE(TAG, "DNS lookup failed");
                  vTaskDelay(1000 / portTICK_PERIOD_MS);
                  continue;
                }
                radio_ip_addr = inet_ntoa(*(struct in_addr*)(server -> h_addr_list[0]));
                ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", radio_ip_addr);
                
                ////*************** Allocate a new socket **************///
                radio_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);  // SOCK_STREAM = TCP
                if(radio_socket < 0) 
                {
                  ESP_LOGE(TAG, "... Failed to allocate socket.");
                  //freeaddrinfo(addr_result);
                  vTaskDelay(1000 / portTICK_PERIOD_MS);
                  continue;
                }
                ESP_LOGI(TAG, "... allocated socket %d", radio_socket);
                
                ////*************** Connect the allocated socket to radio server **************///
                bzero(&dest, sizeof(dest));
                dest.sin_family = AF_INET;
                dest.sin_port = htons(radio_port);
                dest.sin_addr.s_addr = inet_addr(radio_ip_addr);
                
                if(connect(radio_socket, (struct sockaddr*)&dest, sizeof(dest)) != 0) 
                {
                  ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
                  close(radio_socket);
                  vTaskDelay(4000 / portTICK_PERIOD_MS);
                  continue;
                }
                ESP_LOGI(TAG, "... socket connected");
                
                ////*************** Send request to radio server **************///
                if (write(radio_socket, radio_request, strlen(radio_request)) < 0) 
                {
                  ESP_LOGE(TAG, "... socket send failed");
                  close(radio_socket);
                  vTaskDelay(4000 / portTICK_PERIOD_MS);
                  continue;
                }
                ESP_LOGI(TAG, "... socket send successful");
                
                ////*************** Set radio server response waiting period (timeout) **************///
                struct timeval receiving_timeout;
                receiving_timeout.tv_sec = 5;   // 5 seconds wait
                receiving_timeout.tv_usec = 0;
                // SOL_SOCKET:options for socket level
                if (setsockopt(radio_socket, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout, sizeof(receiving_timeout)) < 0)
                {
                  ESP_LOGE(TAG, "... failed to set socket receiving timeout");
                  close(radio_socket);
                  vTaskDelay(4000 / portTICK_PERIOD_MS);
                  continue;
                }
                ESP_LOGI(TAG, "... setting socket receiving timeout successful");
                
                // all done - socket setup & sending request to radio server
                return radio_socket;
            
      } //end of for num_of_tries
  
  if (i==num_of_tries)
      ESP_LOGE(TAG, "RADIO_SETUP : Could not establish socket connection after %d tries", i);
  if (radio_socket > -1)
      close(radio_socket);
  //vTaskDelay(1000 / portTICK_PERIOD_MS);
  return -1;
}


#define MAX_HEADER_SIZE 2048

void read_header()
{
  uint16_t header_bytes_read=0;
  int bytes_read=0;
  uint8_t header_buf[257]; // need decent size toa void partial header termination
  uint16_t i=0, found=0;
  
  // check and catch partial header termination - added max_header_size in the meantime
  // NOTE - if need to use header fields (say bitrate, etc) for later - need to save the header in a global array
  
  // read from socket till complete header is read
  //ESP_LOGI(TAG, "inside read_header");
  
  while (header_bytes_read < MAX_HEADER_SIZE)
  {
        // read 1024 bytes from socket 
        //ESP_LOGI(TAG, "inside while");
        bzero(header_buf, sizeof(header_buf));
        bytes_read = read(radio_socket, header_buf, sizeof(header_buf)-1);
        if ((bytes_read > 0) && (bytes_read < sizeof(header_buf)))
        {
                  //ESP_LOGI(TAG, "reading header bytes_read=%d", bytes_read);
                  bytes_read = bytes_read - 3; // since we have to scan next 4 chars
                  // go through bytes read and print on screen
                  for (i=0; i<bytes_read; i++)
                  {
                    printf("%c", header_buf[i]);
                    // if \r\n\r\n (header terminator) - found 
                    if ( (header_buf[i]==0x0D) && (header_buf[i+1]==0x0A) && (header_buf[i+2]==0x0D) && (header_buf[i+3]==0x0A) ) 
                    {
                      found = i+4; // subscript of first audio byte in header_buf
                      break;     
                    }
                  }
                  bytes_read = bytes_read + 3; // normalise it back
                  
                  if (found>0)
                  {
                        // header termination found
                        printf("\n");
                        ESP_LOGI(TAG, "header_size = %d", header_bytes_read+found);
                        //store remaining bytes read in radio queue
                        for (i=found; i< bytes_read; i++)
                              xQueueSend(radio_data_queue, &header_buf[i], portMAX_DELAY); //xTicksToWait can be NULL as queue is unlikely to be full at the start
                        
                        ESP_LOGI(TAG, "Extracted header and stored remaining (audio) data on the queue, total bytes_read=%d",bytes_read);
                        ESP_LOGI(TAG, "In read header, after reading %d audio bytes from socket radio queue size  = %d", bytes_read-found, (uint16_t)uxQueueMessagesWaiting(radio_data_queue) );
                        return;   // return back to load task
                  }
                  else
                  {
                    // print last 3 characters
                    printf("%c%c%c", header_buf[bytes_read-3], header_buf[bytes_read-2], header_buf[bytes_read-1]);
                  }
                  header_bytes_read = header_bytes_read + bytes_read;          
        } // end of - if bytes_read > 0
        else
            ESP_LOGE(TAG, "Invalid header_read bytes read = %d", bytes_read);
  }   // end of reading upto max_header_size
  
  ESP_LOGE(TAG, "\n Header termination not found in %d bytes", header_bytes_read);
  return;
}


// this task loads audio data into queue
void load_task(void *arg)
{
  uint8_t recv_buf[1301], invalid_byte_count=0;
  uint16_t i;
  int bytes_read=0;
  
    // NOTE - ideally should extract header ending with \r\n = 0x0D 0x0A. And send only audio data
    read_header();
    
    while  (radio_state == STARTED_RADIO)  // until isr changes it to STOP_RADIO when radio_button is pressed again
    {
        bzero(recv_buf, sizeof(recv_buf));    // erases data (by writing \0) in n bytes of memory starting from location pointed to by recv_buf
        // read data from socket
        bytes_read = read(radio_socket, recv_buf, sizeof(recv_buf)-1); // this reads 1 byte less than sizeof(recv_buf) ie 1300 bytes
        //ESP_LOGI(TAG, "bytes_read in load task = %d", bytes_read);
        /*queue_status = (uint16_t)uxQueueMessagesWaiting(radio_data_queue);
        if (queue_status < 4096)
        ESP_LOGD(TAG, "load_task- before loading - queue less than 25 percent filled = %d. bytes_read = %d", queue_status, bytes_read);
        if (queue_status > 12288)
        ESP_LOGD(TAG, "load_task- before loading - queue more than 75 percent filled = %d. bytes_read = %d", queue_status, bytes_read);*/
        
        // write data into audio queue. Sometimes bytes_read = 0xFFFF which is invalid. Check number of bytes read and if valid write data into queue
        if ((bytes_read > 0) && (bytes_read < sizeof(recv_buf)) )
        {
            invalid_byte_count = 0;
            for (i=0;i<bytes_read;i++)
            {
                //reduce xTicksToWait to appropriate time in case player is stuck and queue is full and we need a restart, etc
                if (xQueueSend(radio_data_queue, &recv_buf[i], 1000 / portTICK_RATE_MS) == errQUEUE_FULL) //portMAX_DELAY - wait for 1 seconds in case queue is full
                {
                  ESP_LOGE(TAG, "LOAD_TASK: found radio queue full. Had read %d bytes from socket. Dropping %d bytes",bytes_read, (bytes_read-i));
                  break;
                }
            }
          //ESP_LOGI(TAG, "In load_task, after reading %d bytes from socket radio queue size  = %d", bytes_read, (uint16_t)uxQueueMessagesWaiting(radio_data_queue) );
        }
        else
        {
              ESP_LOGE(TAG, "LOAD_TASK: Invalid bytes_read = %d", bytes_read);
              invalid_byte_count++;
              if (invalid_byte_count > 10)
              {
                  vTaskDelay(1000 / portTICK_PERIOD_MS); // this is really to give space for wifi_stn_event_handler (disconnect event) to trigger radio closure
              }
              taskYIELD();    // as this is a high priority task, to allow a bit of time to network stack to get data in case of a network connection jitter
        } // end of if invalid bytes found
      
    }   // end of while radio state is started_radio
       
  if (radio_socket > -1)
      close(radio_socket);
  ESP_LOGD(TAG, "LOAD_TASK: closed socket, deleting load task");
  vTaskDelete(NULL);
}
  
 
void radio_task(void *arg)
{
  uint8_t io_num;
  uint16_t initial_buffer_size = 14745;  //90% of queue size
  //TaskHandle_t loadHandle, unloadHandle; 
  
  while (1) // to keep the task alive
  {
    if (xQueueReceive(radio_evt_queue, &io_num, portMAX_DELAY)) 
    {  
          if (radio_state == START_RADIO)
          {
                // initiate radio - initialize any variables as needed for fresh radio play
                //ESP_LOGI(TAG, "radio state = START_RADIO %d GPIO %d Current value= %d", radio_state, io_num, gpio_get_level(io_num));
                
                // step 1: create radio data queue
                radio_data_queue = xQueueCreate(16384, sizeof(uint8_t));    // create a queue for radio data buffering - 16k
                
                if (radio_data_queue == NULL)
                {
                    ESP_LOGE(TAG, "RADIO_TASK: Not enough memory to hold radio data");
                    radio_state = RADIO_OFF;
                }
                else
                {
                      ESP_LOGI(TAG, "RADIO_TASK: queue created successfully");
                      //vTaskDelay(10 / portTICK_RATE_MS);  // wait after allocating large queue memory - should check queue != NULL ie error assigning memory
                      
                      // step 2: TODO - stop http server / TODO - stop wifi AP mode / connect esp in station mode 
                      // stop wifi in ap mode - includes http server stop
                      stop_wifi_ap_mode();
                      // start esp in station mode
                      start_wifi_stn_mode();
                      
                      // proceed only in case of no internet connection issues ie radio state is still START_RADIO
                      if (radio_state == START_RADIO)
                      {
                            radio_state = STARTED_RADIO;    /// doing this after setup and header to avoid corrupt state due to frequent button press
                        
                            // open a new radio socket and connect to radio server
                            radio_socket = radio_socket_connect();
                            ESP_LOGI(TAG, "RADIO_TASK :radio_socket_connect returned %d", radio_socket);
                            if (radio_socket > -1) // proceed if valid socket created and connected to radio server
                            {
                                  ////*************** Read radio server's http response **************///
                                  xTaskCreate(load_task, "load_task", 2837, NULL, 9, NULL);  // task stack size = 1536 bytes for task + 1301 bytes for tcp recieve buffer
                                  
                                  // read how much radio data has been buffered
                                  while ( ((uint16_t)uxQueueMessagesWaiting(radio_data_queue) < initial_buffer_size) && (radio_state == STARTED_RADIO) )
                                      vTaskDelay(10 / portTICK_RATE_MS);
                                  
                                  ESP_LOGI(TAG, "RADIO_TASK: before calling unload task radio queue size  = %d", (uint16_t)uxQueueMessagesWaiting(radio_data_queue) );
                                  // create task to unload radio data queue to vs1053 for playing
                                  xTaskCreate(unload_task, "unload_task", 2048, NULL, 9, NULL);
                                  
                            } // end of if socket created and connected successfully
                            else
                            {
                                  // disconnect from wifi as could not connect socket to radio server. This would trigger radio_state=STOP_RADIO cleanup via wifi disconnect event
                                  ESP_LOGE(TAG, "RADIO_TASK: station was connected to router but could not connect to radio server, triggering cleanup");
                                  ESP_ERROR_CHECK(esp_wifi_disconnect());
                            }
                      } // end of creating load and unload tasks
                } // end of else - queue was created successfully
          } // end of outer if radio state = start radio
          else if (radio_state == STOP_RADIO)
          {

                // assuming 3s delay is always enough for:
                // 1) load task self delete (including socket close) - after detecting state != STARTED_RADIO
                // 2) unload task self delete (including stop player)
                vTaskDelay(3000 / portTICK_RATE_MS);

                // 3) stop esp station mode 
                //4) start esp ap mode - revert to normal mode
                //5) start http server 
                //6) delete radio queue - if even after this much delay, this happens before load/unload task delete, program will crash
                //7) change radio state - at last, only after this, user can start radio via the button
                            
                // stop wifi in station mode
                stop_wifi_stn_mode();
                // start wifi ap mode - includes http server start
                start_wifi_ap_mode();
                
                // delete radio data queue
                ESP_LOGI(TAG, "deleting radio data queue");
                vQueueDelete(radio_data_queue);
                //vTaskDelay(100 / portTICK_PERIOD_MS);
                
                radio_state = RADIO_OFF;
                ESP_LOGI(TAG, "RADIO_TASK: RADIO STOPPED: minimum heap_size = %d", esp_get_minimum_free_heap_size());
          } // end of else - stop radio
          
    } // end of if radio event occured and added to radio queue by ISR
    
  } // end of infinite while to keep the task alive
  
}   // end of radio_task
