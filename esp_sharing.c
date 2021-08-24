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

#include <nvs_flash.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_http_server.h>
#include <sys/socket.h>

#include "html_chunks.h"
#include "esp_sharing.h"
#include "ringbuffer.h"

static const char *TAG = "esp_sharing";

#define ESP_WIFI_SSID "nodemcu"
#define ESP_WIFI_PASSWORD "petethecat"

void wifi_ap_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
      if (event_id == WIFI_EVENT_AP_START) 
      {
            ESP_LOGI(TAG, "soft-AP start");
            start_http_server();
      }
      if (event_id == WIFI_EVENT_AP_STACONNECTED) 
      {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d", MAC2STR(event->mac), event->aid);
      } 
      if (event_id == WIFI_EVENT_AP_STADISCONNECTED) 
      {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d", MAC2STR(event->mac), event->aid);
      }
      if (event_id == WIFI_EVENT_AP_STOP) 
      {
            ESP_LOGI(TAG, "soft-AP stop");
            stop_http_server();
      }
      if (event_id == WIFI_EVENT_AP_PROBEREQRECVED) 
      {
        ESP_LOGI(TAG, "Receive probe request packet in soft-AP interface");
      }
}


httpd_uri_t myhome = {
  .uri       = "/",
  .method    = HTTP_GET,
  .handler   = index_get_handler,
  .user_ctx  = NULL
};

httpd_uri_t myindex = {
  .uri       = "/index.html",
  .method    = HTTP_GET,
  .handler   = index_get_handler,
  .user_ctx  = NULL
};

httpd_uri_t mysettings = {
  .uri       = "/settings.html",
  .method    = HTTP_GET,
  .handler   = settings_get_handler,
  .user_ctx  = NULL
};

httpd_uri_t myringfile = {
  .uri       = "/ring_file",
  .method    = HTTP_GET,
  .handler   = file_get_handler,
  .user_ctx  = NULL
};
httpd_uri_t myringall = {
  .uri       = "/ring_all",
  .method    = HTTP_GET,
  .handler   = ring_get_handler,
  .user_ctx  = NULL
};


void start_http_server()
{
  // create a default configuration for http server
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  
  // Start the httpd server with default configuration
  ESP_LOGD(TAG, "Starting server on port: '%d'", config.server_port);
  if (httpd_start(&server, &config) == ESP_OK) 
  {
    ESP_LOGD(TAG, "Http server started");
    httpd_register_uri_handler(server, &myhome);
    httpd_register_uri_handler(server, &myindex);
    httpd_register_uri_handler(server, &mysettings);
    // register file-by-file ring download and play handler
    httpd_register_uri_handler(server, &myringfile);
    // register full ring download and play handler
    httpd_register_uri_handler(server, &myringall);
    ESP_LOGI(TAG, "Http server handlers registered");
  }
  else
    ESP_LOGD(TAG, "Error starting http server!");
}


void stop_http_server()
{
  // Ensure handle is non NULL
   if (server != NULL) 
   {
        // Stop the httpd server
        httpd_stop(server);
    }
    ESP_LOGD(TAG, "Stopped http server!");
}

/* HTTP GET handler for base url / */
esp_err_t index_get_handler(httpd_req_t *req)
{
  httpd_resp_send_chunk(req, index_start_str, strlen(index_start_str));
  
  struct stat st;
  // table row = "<tr><td style=\"color:sienna\"><b>01.ogg</b></td>
                //<td>32436</td>
                //<td><a href=\"/ring_file?fname=/spiffs_ring/01.ogg\" download=\"01.ogg\">01.ogg</a></td>
                //<td><audio controls title=\"01.ogg\"><source src=\"/ring_file?fname=/spiffs_ring/01.ogg\" type=\"audio/ogg\">Player unsupported</audio></td></tr>";
  
  uint8_t i, j, index, num_of_files;
  
  char index_middle_str[310];   // current char count in table row = 194 (?). Increase string size as per table row html size
  char *file_name;
  char file_name_short[7];
  char temp_filesize[6];
  
  index = ring_getOldestFilePointer();
  num_of_files = ring_getNumOfFiles();
  
  for (i=0;i<num_of_files;i++)
  {
      file_name = ring_getFileName(index);
      stat(file_name, &st);
      if (st.st_size ==0)
      {
        ring_next(&index);
        continue;
      }
      for(j=0;j<7;j++)
        file_name_short[j] = *(file_name+j+13);    // copies \0 termination too. eg. file_name=/spiffs_ring/00.ogg file_name_short=00.ogg
      strcpy(index_middle_str, "<tr><td style=\"color:sienna\"><b>");
      // add filename
      strcat(index_middle_str, file_name_short);
      
      strcat(index_middle_str, "</b></td><td>");
      // add file size
      itoa((uint32_t)st.st_size, temp_filesize, 10);
      strcat(index_middle_str, temp_filesize);
      
      strcat(index_middle_str, "</td><td><a href=\"/ring_file?fname=");
      // add file link
      strcat(index_middle_str, file_name);
      strcat(index_middle_str, "\" download=\"");
      strcat(index_middle_str, file_name_short);
      strcat(index_middle_str, "\">");
      strcat(index_middle_str, file_name_short);
      strcat(index_middle_str, "</a></td><td><audio controls title=\"");
      strcat(index_middle_str, file_name_short);
      strcat(index_middle_str, "\"><source src=\"/ring_file?fname=");
      
      // add full filename
      strcat(index_middle_str, file_name);
      strcat(index_middle_str, "\" type=\"audio/ogg\">Audio unsupported</audio></td></tr>");
      
      httpd_resp_send_chunk(req, index_middle_str, strlen(index_middle_str));
      ring_next(&index);
  }
  
  httpd_resp_send_chunk(req, index_end_str, strlen(index_end_str));
  httpd_resp_send_chunk(req, NULL, 0);
  
  return ESP_OK;
}

/* HTTP GET handler for settings url / */
esp_err_t settings_get_handler(httpd_req_t *req)
{
  httpd_resp_send_chunk(req, settings_start_str, strlen(settings_start_str));
  httpd_resp_send_chunk(req, NULL, 0);
  
  return ESP_OK;
}


// spiffs file handler
esp_err_t file_get_handler(httpd_req_t *req)
{
  uint8_t temp_buffer[251], bytes_read;
  ESP_LOGD(TAG, "req->uri = %s", req->uri);
  
  size_t query_len;
  char file_name[20];
  char *query_string;
  
  //req->uri = /ring_file?fname=/spiffs_ring/01.ogg
  query_len = httpd_req_get_url_query_len(req) + 1;
  query_string = malloc(query_len);
  httpd_req_get_url_query_str(req, query_string, query_len);
  httpd_query_key_value(query_string, "fname", file_name, sizeof(file_name));
  free(query_string);
  
  // open the required file
  // read in chunks and send as response in chunks
  FILE *f = fopen(file_header, "r");
  // loop through 250 bytes from audio file and send via httpd
  while(1)
  {
          bytes_read = fread(temp_buffer, 1, 250, f);
          //ESP_LOGI(TAG, "temp_buffer = %s", temp_buffer);
          if (bytes_read <= 0)
              break;
          httpd_resp_send_chunk(req, (const char* )temp_buffer, bytes_read);
  
  } // end of while(1) going through the current audio file
  fclose(f);
  
  f = fopen(file_name, "r");
  // loop through 250 bytes from audio file and send via httpd
  while(1)
  {
    bytes_read = fread(temp_buffer, 1, 250, f);
    //ESP_LOGI(TAG, "temp_buffer = %s", temp_buffer);
    if (bytes_read <= 0)
      break;
    httpd_resp_send_chunk(req, (const char* )temp_buffer, bytes_read);
    
  } // end of while(1) going through the current audio file
  fclose(f);

  httpd_resp_send_chunk(req, NULL, 0);
  
  return ESP_OK;
}


// full ring download handler
esp_err_t ring_get_handler(httpd_req_t *req)
  {
    uint8_t i=0;
    uint8_t temp_buffer[251], bytes_read;
    ESP_LOGD(TAG, "req->uri = %s", req->uri);
    
    // read in chunks and send as response in chunks
    FILE *f = fopen(file_header, "r");
    // loop through 250 bytes from header file and send via httpd
    while(1)
    {
      bytes_read = fread(temp_buffer, 1, 250, f);
      if (bytes_read <= 0)
        break;
      httpd_resp_send_chunk(req, (const char* )temp_buffer, bytes_read);
      
    } // end of while(1) going through the header file
    fclose(f);
    
    // loop through the ring starting from oldest pointer
    uint8_t index = ring_getOldestFilePointer();
    uint8_t num_of_files = ring_getNumOfFiles();
    char *file_name;
    struct stat st;
    for (i=0;i<num_of_files;i++)
    {
          file_name = ring_getFileName(index);
          //ESP_LOGI(TAG, "reading %s", file_name);
          stat(file_name, &st);
          if (st.st_size ==0)
          {
            ring_next(&index);
            continue;
          }
          f = fopen(file_name, "r");
          // loop through 250 bytes from audio file and send via httpd
          while(1)
          {
              bytes_read = fread(temp_buffer, 1, 250, f);
              if (bytes_read <= 0)
                break;
              httpd_resp_send_chunk(req, (const char* )temp_buffer, bytes_read);
          } // end of while(1) going through the current audio file
          fclose(f);
          ring_next(&index);
    } // end of for through the entire ring*/
    httpd_resp_send_chunk(req, NULL, 0);
    
    return ESP_OK;
  }



void start_wifi_ap_mode()
{
      // register wifi event handlers
      ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_ap_event_handler, NULL));
      ESP_LOGI(TAG, "AP mode : wifi event handlers registered");
      
      // configure wifi  
      wifi_config_t wifi_config = {
        .ap = {
        .ssid = ESP_WIFI_SSID,
        .ssid_len = strlen(ESP_WIFI_SSID),
        .password = ESP_WIFI_PASSWORD,
        .max_connection = 4,
        .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
      };
      
      // setup AP mode
      ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
      ESP_LOGI(TAG, "AP mode : wifi mode set");
      ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
      ESP_LOGI(TAG, "AP mode : wifi new config set");
      
      // start wifi
      ESP_ERROR_CHECK(esp_wifi_start());
      ESP_LOGD(TAG, "AP mode : wifi started");
      //vTaskDelay(2000 / portTICK_PERIOD_MS); // this delay is only to give time to esp to connect to wifi router before next statement/task is executed (which may depend on this connection)
      
      //start_http_server(); // this is happening in event handler
}


void stop_wifi_ap_mode()
{
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_ap_event_handler));
    ESP_LOGD(TAG, "wifi in ap mode stopped");
    //vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    //stop_http_server();   // this is happening in event handler
}

