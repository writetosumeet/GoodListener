

#define NOP() asm volatile ("nop")
// This equates to 11kbps as per page 6 VorbisEncoder170c.pdf
#define SKIP_PLUGIN_VARNAME
static const uint16_t ogg_encoder_plugin[] = {
#include "venc08k1q03.plg"
};
#undef SKIP_PLUGIN_VARNAME

#define SKIP_PLUGIN_VARNAME
static const uint16_t vs_decoder_patch[] = {
#include "vs1053b-patches.plg"
};
#undef SKIP_PLUGIN_VARNAME

extern const char *file_header;

#define DREQ_PIN            0   // GPIO 0
#define VS_RST_PIN          4   // GPIO 4
#define CS_PIN              5   // GPIO 5

// VS1053       NodeMCU         ESP8266       Color
// MOSI         D7              GPIO 13       Blue
// MISO         D6              GPIO 12       Green              
// SCK          D5              GPIO 14       Yellow
// CS           D1              GPIO 5        Orange
// DCS          3V3 (next to D4)              White
// DREQ         D3              GPIO 0        Brown
// RST          D2              GPIO 4        Grey

// SCI Register
#define SCI_MODE  0x0
#define SCI_STATUS  0x1
#define SCI_BASS  0x2
#define SCI_CLOCKF  0x3
#define SCI_DECODE_TIME  0x4        // current decoded time in full seconds
#define SCI_AUDATA  0x5
#define SCI_WRAM  0x6
#define SCI_WRAMADDR  0x7
#define SCI_HDAT0  0x8
#define SCI_HDAT1  0x9
#define SCI_AIADDR  0xA
#define SCI_VOL  0xB
#define SCI_AICTRL0  0xC
#define SCI_AICTRL1  0xD
#define SCI_AICTRL2  0xE
#define SCI_AICTRL3  0xF

#define SCI_num_registers  0xF

// SCI_MODE bits
#define SM_RESET 2             // Bitnumber in SCI_MODE soft reset
#define SM_CANCEL 3            // Bitnumber in SCI_MODE cancel song
#define SM_TESTS 5             // Bitnumber in SCI_MODE for tests
#define SM_SDISHARE 10        // Bitnumber in SCI_MODE for sharing CS for SCI and SDI
#define SM_SDINEW 11           // Bitnumber in SCI_MODE always on
#define SM_ADPCM 12             // 1 = active
#define SM_LINE1 14            // 1= Line input 0=Mic

// share_button ie GPIO16 does not have ISR handler available
// this button has dual purpose - prev when playback (in vs) and Share on webpage (in main)
#define share_button       16    // GPIO 16 - Default pull down + value = 0

extern xQueueHandle record_evt_queue;
extern xQueueHandle play_evt_queue;
extern xQueueHandle radio_evt_queue;
extern xQueueHandle radio_data_queue;

#define RECORDER_OFF      0
#define START_RECORDING   1
#define STARTED_RECORDING 2
#define STOP_RECORDING    3
extern uint8_t recording_state;

#define PLAYER_OFF      0
#define START_PLAYING   1
#define STARTED_PLAYING 2
#define STOP_PLAYING    3
#define PLAY_NEXT       4
#define PLAY_PREV       5
extern uint8_t playing_state;

#define RADIO_OFF      0
#define START_RADIO  1
#define STARTED_RADIO  2
#define STOP_RADIO  3
extern uint8_t radio_state;

extern void partition_info();

void data_mode_on();
void data_mode_off();
void control_mode_on();
void control_mode_off();

void dreq_wait();

uint16_t read_register(uint8_t regAddr);
void write_register(uint8_t regAddr, uint16_t regValue);
void read_all_registers();

uint16_t wram_read(uint16_t wramAddr);
void wram_write(uint16_t wramAddr, uint16_t wramValue);

// to set GPIO 0 and GPIO 1 to value 0 (GND) and to set SDI_SHARE to 1 so same (CS) pin is used for both SCI and SDI
// Both these are problems with LC technology board - SDI will not work without any of these changes
// By default LC Tech board starts in MIDI playing mode. This should actually be MP3 mode with GPIO 0 and 1 grounded
// SDI does not work in the default MIDI playing mode.
void sdi_patch();
void vs_hard_reset();
void load_plugin(const uint16_t *plugin, uint16_t len);

void print_header();
void record_task(void *arg);

void player_setup();
void play_file(char *file_name);
void stop_player();
void play_task(void *arg);

void play_radio(uint8_t *stream, uint16_t stream_size);
void unload_task(void *arg);
