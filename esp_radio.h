
extern xQueueHandle record_evt_queue;
extern xQueueHandle play_evt_queue;
extern xQueueHandle radio_evt_queue;
extern xQueueHandle radio_data_queue;

int radio_socket;

void start_wifi_stn_mode();
void stop_wifi_stn_mode();

void wifi_stn_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data);

void on_got_ip(void *arg, esp_event_base_t event_base,
               int32_t event_id, void *event_data);

void get_text_from_url();

int radio_socket_connect();
void read_header();
void radio_task(void *arg);

void load_task(void *arg);




