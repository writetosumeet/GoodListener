
// These should be stored and read from metadata files
#define ESP_WIFI_SSID "nodemcu"
#define ESP_WIFI_PASSWORD "petethecat"


// global httpd server variable
httpd_handle_t server;

extern const char *file_header;

/* HTTP GET handler for settings url / */
esp_err_t settings_get_handler(httpd_req_t *req);
/* HTTP GET handler for base url / */
esp_err_t index_get_handler(httpd_req_t *req);
// spiffs file handler
esp_err_t file_get_handler(httpd_req_t *req);
// full ring download handler
esp_err_t ring_get_handler(httpd_req_t *req);

void start_http_server();
void stop_http_server();
void start_wifi_ap_mode();
void stop_wifi_ap_mode();
void wifi_ap_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data);






