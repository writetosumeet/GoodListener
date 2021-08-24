
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

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

#include "ringbuffer.h"
#include "vs1053.h"

static const char *TAG = "vs1053";

static const char *vs_register_names[16] = {"SCI_MODE", "SCI_STATUS", "SCI_BASS", "SCI_CLOCKF", "SCI_DECODE_TIME", "SCI_AUDATA", "SCI_WRAM", \
                                            "SCI_WRAMADDR", "SCI_HDAT0", "SCI_HDAT1", "SCI_AIADDR", "SCI_VOL", "SCI_AICTRL0", "SCI_AICTRL1", "SCI_AICTRL2", "SCI_AICTRL3"};


void data_mode_on()
{
  gpio_set_level(CS_PIN, 0); // bringing from default 1 to 0
  gpio_set_level(CS_PIN, 1); // giving rising edge to start sdi
}  

void data_mode_off()
{
  gpio_set_level(CS_PIN, 0); // giving falling edge to stop sdi
  gpio_set_level(CS_PIN, 1); // bringing back to default 1
}

void control_mode_on()
{
  gpio_set_level(CS_PIN, 1);
  gpio_set_level(CS_PIN, 0); // giving falling edge to start sci
}  

void control_mode_off()
{
  gpio_set_level(CS_PIN, 0); // redundant
  gpio_set_level(CS_PIN, 1); // bringing back to default 1 & giving rising edge to stop sci
}

void dreq_wait()
{
  //50ms = 50*1000 us - timeout 50ms
  uint16_t timeout = 50000;
  
  while(timeout--)
  {
    //delay_us(1);   // wait for 1us
    if (gpio_get_level(DREQ_PIN) == 1)    // if dreq high
        return;
    taskYIELD();
  }
  ESP_LOGE(TAG, "timeout: dreq low even after 50 ms");
}


uint16_t read_register(uint8_t regAddr)
{
  esp_err_t ret;
  uint32_t regValue = 0xAAAAAAAA;
  uint16_t readCmd;
  uint32_t regAddr32 = (uint32_t)(regAddr << 24);
  
  spi_trans_t readTrans;
  readTrans.bits.val = 0;            // clear all bits

  readCmd = SPI_MASTER_READ_DATA_FROM_SLAVE_CMD;
  readTrans.cmd = &readCmd;
  readTrans.bits.cmd = 8;
  readTrans.addr = &regAddr32;
  readTrans.bits.addr = 8;   
  readTrans.miso = &regValue;
  readTrans.bits.miso = 16;
  
  dreq_wait();
  control_mode_on();
  // start transmission
  ret = spi_trans(HSPI_HOST, &readTrans);
  
  if (ret != ESP_OK) 
  {
    if (ret == ESP_FAIL) 
      ESP_LOGE(TAG, "ESP_FAIL - spi has not been initialized yet");
    else 
      if (ret == ESP_ERR_INVALID_ARG) 
        ESP_LOGE(TAG, "ESP_ERR_INVALID_ARG - Parameter error");
      else 
        ESP_LOGE(TAG, "Some other SPI error %s", esp_err_to_name(ret));
      //return; - NEED TO DO SOME ERROR HANDLING HERE - ie do not pick this value
  }
  dreq_wait();
  control_mode_off();
  /*if (( ((uint16_t)regValue) | 0) !=0)
    ESP_LOGE(TAG, "In read register - spi error as regValue lsb not cleared to 0");*/
  
  //ESP_LOGI(TAG, "read reg - full 32 bit reg value = %08X", regValue);
  return ((uint16_t)(regValue >> 16));
}


void write_register(uint8_t regAddr, uint16_t regValue)
{
  esp_err_t ret;
  uint16_t writeCmd;
  uint32_t regValue32 = (uint32_t)(regValue << 16);
  uint32_t regAddr32 = (uint32_t)(regAddr << 24);
  
  //regValue32 = regValue32 | 0xABCD;
  //ESP_LOGI(TAG, "write reg - reg value being sent is %08X", regValue32);

  spi_trans_t writeTrans;
  writeTrans.bits.val = 0;            // clear all bits
  
  writeCmd = SPI_MASTER_WRITE_DATA_TO_SLAVE_CMD;
  writeTrans.cmd = &writeCmd;
  writeTrans.bits.cmd = 8;
  writeTrans.addr = &regAddr32;
  writeTrans.bits.addr = 8;   
  writeTrans.mosi = &regValue32;
  writeTrans.bits.mosi = 16;
  
  dreq_wait();
  control_mode_on();
  // start transmission
  ret = spi_trans(HSPI_HOST, &writeTrans);
  
  if (ret != ESP_OK) 
  {
    if (ret == ESP_FAIL) 
      ESP_LOGE(TAG, "ESP_FAIL - spi has not been initialized yet");
    else 
      if (ret == ESP_ERR_INVALID_ARG) 
        ESP_LOGE(TAG, "ESP_ERR_INVALID_ARG - Parameter error");
      else 
        ESP_LOGE(TAG, "Some other SPI error %s", esp_err_to_name(ret));
      //return; - NEED TO DO SOME ERROR HANDLING HERE - ie do not pick this value
  }
  dreq_wait();
  control_mode_off();
  
  //ESP_LOGI(TAG, "write reg - reg value after sending is %08X", regValue32);
}


void read_all_registers()
{
  uint8_t i = 0;
  uint16_t regValue;
  
  ESP_LOGI(TAG, "Reading all VS1053 registers");
  for (i = 0; i < 16; i++) 
  {
    regValue = read_register(i);      // regValue = read_register(0x02000000);
    ESP_LOGI(TAG, "Read %s = %04X", vs_register_names[i], regValue);
    
    if ((i==0) && (regValue==0))
      ESP_LOGE(TAG, "sci_mode is 0! check all registers!!");
  }
}

uint16_t wram_read(uint16_t wramAddr)
{
  write_register(SCI_WRAMADDR, wramAddr);
  return (read_register(SCI_WRAM));
}


void wram_write(uint16_t wramAddr, uint16_t wramValue)
{
  write_register(SCI_WRAMADDR, wramAddr);
  write_register(SCI_WRAM, wramValue);
  
  // read and confirm wram is written correctly
  /*uint16_t actualWramValue = wram_read(wramAddr);
  if (actualWramValue != wramValue)
    ESP_LOGE(TAG, "Error writing %04X to wram address %04X, value remains %04X", wramValue, wramAddr, actualWramValue);
  else
    ESP_LOGD(TAG, "Write operation complete, wram address %04X has value %04X", wramAddr, wramValue);*/
}



// to set GPIO 0 and GPIO 1 to value 0 (GND) and to set SDI_SHARE to 1 so same (CS) pin is used for both SCI and SDI
// Both these are problems with LC technology board - SDI will not work without any of these changes
void sdi_patch()
{
  
  // By default LC Tech board starts in MIDI playing mode. This should actually be MP3 mode with GPIO 0 and 1 grounded
  // SDI does not work in the default MIDI playing mode.
  
  // GPIO 0 and 1 need to be set as output and grounded
  //0xC017 : GPIO direction - 1 is output
  wram_write(0xC017, 0x0003);
  
  // 0xC019 : GPIO value to be set for the pins that are set to 1 above
  wram_write(0xC019, 0);
  vTaskDelay(2 / portTICK_RATE_MS);
  dreq_wait();
  
  // setup mic / set sdinew / set sdi shared bit - DCS pin in LC technology board is not working so SDI will not work without this
  // reset decoder software by using soft reset
  write_register(SCI_MODE, 0x0C04);
  vTaskDelay(10 / portTICK_RATE_MS);
  dreq_wait();
  
  if (read_register(SCI_AUDATA) == 0x1F40)
    ESP_LOGI(TAG, "sdi patch applied"); // gpio 0 & 1 = GND + SDI share set + soft-reset
  else
    ESP_LOGE(TAG, "sdi patch error: sci_audata is not = 1F40");
  
}   //*********** END OF SDI_PATCH


void vs_hard_reset()
{
  gpio_set_level(VS_RST_PIN, 0); 
  vTaskDelay(100 / portTICK_RATE_MS);
  gpio_set_level(VS_RST_PIN, 1);    
  vTaskDelay(100 / portTICK_RATE_MS);
  // wait for dreq to be 1
  dreq_wait();
  ESP_LOGI(TAG, "hard reset of VS1053 done");
  
  // reduce spi speed to allow clockf update
  spi_clk_div_t spi_clk = SPI_2MHz_DIV;
  esp_err_t ret = spi_set_clk_div(HSPI_HOST, &spi_clk);
  if (ret != ESP_OK) 
    ESP_LOGE(TAG, "spi error - clk NOT set to 2MHz");
  else
    ESP_LOGI(TAG, "spi clk set to 2MHz");
  
  // set vs1053 clock to 4.5x ie 55.296MHz
  write_register(SCI_CLOCKF, 0xC000);
  vTaskDelay(1/ portTICK_RATE_MS);
  dreq_wait();
  
  sdi_patch();
}


void load_plugin(const uint16_t *plugin, uint16_t len) 
{
  uint16_t i = 0;
  uint16_t wram_reg, num_times, wram_value;
  
  
  while (i < len) 
  {
    wram_reg = plugin[i++];
    num_times = plugin[i++];
    
    if (num_times & 0x8000U)        // if 1st digit is 8, write the value num_times
    { 
      num_times &= 0x7FFF;         // find num_times by ignoring 1st digit
      wram_value = plugin[i++];    // find the value to be written num_times
      while (num_times--)         // for (j=0; j < num_times; j++)
          write_register(wram_reg, wram_value); // write the same value num_times
    } 
    else // write the next num_times values
    {           
      while (num_times--) // for (j=0; j < num_times; j++)    
      {
        wram_value = plugin[i++];           // read next value
        write_register(wram_reg, wram_value); // write the next num_times values one by one
      }
    }
  }
}



void print_header()
{
  uint8_t readChar;
  ESP_LOGI(TAG, "\n\nPrinting header \n");
  FILE *f = fopen(file_header, "r");
  uint16_t j;
  uint8_t oggs[4] = {0,0,0,0};
  uint8_t vorbis[6] = {0,0,0,0,0,0};
  
  for (j=0;j<1368; j++)
  {
    readChar = fgetc(f);
    if( feof(f) ) 
      break;
    printf("%02X", readChar);
    
    oggs[0] = oggs[1];
    oggs[1] = oggs[2];
    oggs[2] = oggs[3];
    oggs[3] = readChar;
    if ( (oggs[0]==0x4f) && (oggs[1]==0x67) && (oggs[2]==0x67) && (oggs[3]==0x53) )
      printf("--------> position = %d \n", j);
    vorbis[0] = vorbis[1];
    vorbis[1] = vorbis[2];
    vorbis[2] = vorbis[3];
    vorbis[3] = vorbis[4];
    vorbis[4] = vorbis[5];
    vorbis[5] = readChar;
    if ( (vorbis[0]==0x76) && (vorbis[1]==0x6f) && (vorbis[2]==0x72) && (vorbis[3]==0x62) && (vorbis[4]==0x69) && (vorbis[5]==0x73))
      printf("--------> position = %d \n", j);
  }
  printf("\n");
  fclose(f);
}

void record_task(void *arg)
{
  uint8_t io_num;
  uint8_t state=0;
  uint16_t hdat0_16bit, words_waiting=0, words_to_read, hdat1_16bit;
  uint16_t files_updated = 0; //, j=0
  uint32_t i;
  
  uint8_t audio_data[512];
  uint8_t oggs_count=0;
  uint16_t header_size=0, data_size=0;
  FILE* f_header;
  
  while (1) 
  {
    if (xQueueReceive(record_evt_queue, &io_num, portMAX_DELAY)) 
    {
          // initiate recorder
          ESP_LOGI(TAG, "recording state = START_RECORDING %d GPIO %d Current value= %d", recording_state, io_num, gpio_get_level(io_num));

                // initialize variables required for fresh recording
                state=0;
                words_waiting=0;
                //j=0;
                files_updated = 0;
                oggs_count=0;
                header_size=0;
                data_size=0;
                // 1 ogg page = 4096 bytes (approx)
                // we read 512 bytes = 2 blocks in 1 cycle
                // need to read (4096 / 256) * 7 = 16*7 = 112 blocks for 7 ogg pages = 1 file
              
                /// ********* CREATE AUDIO FILE ************** ///
                f_header = fopen(file_header, "w");
                
                // print partition info used=filesize
                partition_info();
                
                /// ********* Ogg Vorbis Setup ************** ///
                vs_hard_reset();
                
                write_register(SCI_BASS, 0x0);
                // clear adpcm / set mic / set sdinew / set sdishare / set soft reset
                write_register(SCI_MODE, 0x0C04); 
                vTaskDelay(2 / portTICK_RATE_MS); // small delay after soft reset
                dreq_wait();
                
                //disable all interrupts except SCI interrupt
                wram_write(0xc01a, 0x2);
                dreq_wait();
                
                // load ogg vorbis encoder plugin
                load_plugin(ogg_encoder_plugin, sizeof(ogg_encoder_plugin)/sizeof(ogg_encoder_plugin[0]));
                //vTaskDelay(10 / portTICK_RATE_MS); 
                dreq_wait();
                
                // set mic input / set adpcm bit / set sdinew / set sdishare
                write_register(SCI_MODE, 0x1C00);
                
                // Setup - SCI_AICTRL0/1/2/3
                write_register(SCI_AICTRL1, 0);   // 0 for AGC, 1024 = no gain
                write_register(SCI_AICTRL2, 4096);   // 0 if AGC not used ; 4096 recommended for speech in ogg encoder patch pdf, max autogain amplification 0 = 64x, 1024=1x
                write_register(SCI_AICTRL3, 0); // initial value
                
                // activate the encoder
                write_register(SCI_AIADDR,0x34);
                ESP_LOGI(TAG, "RECORDING STARTED ...\n");
                vTaskDelay(2000 / portTICK_RATE_MS); // 2 second delay before starting recording
                dreq_wait();
                ESP_LOGI(TAG, "read SCI_MODE = %04X expected 0x1C00", read_register(SCI_MODE));
                ESP_LOGI(TAG, "read SCI_AICTRL0 = %04X", read_register(SCI_AICTRL0));
                ESP_LOGI(TAG, "read SCI_AICTRL1 = %04X", read_register(SCI_AICTRL1));
                ESP_LOGI(TAG, "read SCI_AICTRL2 = %04X expected 0x1000", read_register(SCI_AICTRL2));
                ESP_LOGI(TAG, "read SCI_AICTRL3 = %04X", read_register(SCI_AICTRL3));
                ESP_LOGI(TAG, "read SCI_HDAT1 = %04X expected non-zero", read_register(SCI_HDAT1));
                
                /// ********* RECORDING STARTED ************** ///
                
                /// ********* RECORD HEADER ************** ///     
                // write first 1368 bytes into header file
                while(header_size < 1368)
                {
                    // read HDAT1 ie words in vs1053 audio buffer
                    hdat1_16bit = read_register(SCI_HDAT1);     
                    if (hdat1_16bit < 0xFFF)
                        words_waiting = hdat1_16bit;    // to avoid using any values more than 4096 ie the buffer size. spi read error can make hdat1 0xffff ie 65,535
                    ESP_LOGI(TAG, "words_waiting = %d", words_waiting);
                      for (i=0; i<words_waiting; i++)
                      {
                           // read a word from hdat0 and write to header file OR ring buffer
                           hdat0_16bit = read_register(SCI_HDAT0);
                           //ESP_LOGI(TAG, "header for words_waiting header_size = %d oggs_count = %d", header_size, oggs_count);
                           // check for 0xFFFF in Hdat0. It should only appear once and as 171 & 172 bytes in the header
                           //if (f_header != NULL)
                           if (header_size < 1368)
                           {
                               // check if hdat0 value is invalid ie (hdat0 = 0xffff and header_size != 170)
                               if( (hdat0_16bit == 0xFFFF) && (header_size!=170))  // this is a problem - ignore this word in the header.
                               {
                                   i = i - 1;    // if invalid - i-- and ignore the word read 
                                   ESP_LOGE(TAG, "hdat0 is 0xFFFF at the header byte %d", header_size);
                               } // end of invalid hdat0 bytes
                               else
                               {
                                 // if valid - - write to header file & increase the header size by 2 bytes 
                                   fputc((uint8_t)(hdat0_16bit >> 8), f_header); // write msb
                                   fputc((uint8_t)(hdat0_16bit & 0x00FF), f_header); // write lsb
                                   //f_header[header_size] = (uint8_t)(hdat0_16bit >> 8); // write msb
                                   //f_header[header_size+1] = (uint8_t)(hdat0_16bit & 0x00FF); // write lsb
                                   
                                   header_size= header_size + 2;
                                   // if header size = 1368, then close header file; set file pointer to null  oggs_count = 1;   files_updated++;
                                   if (header_size==1368)     // assuming header is of 1368 bytes - we have completed capturing the header
                                   {
                                     fclose(f_header);
                                     files_updated++; // assuming there is some audio data and at least one ring buffer file needs to be opened
                                   }
                                        
                               } // end of valid 2 bytes read
                           }   // end of if writing to header file
                           else
                           {
                             // write hdat0 bytes to ring buffer
                             ring_write((uint8_t)(hdat0_16bit >> 8), oggs_count);  // write msb
                             ring_write((uint8_t)(hdat0_16bit & 0x00FF), oggs_count);  // write lsb
                             data_size = data_size + 2;
                           }
                      } // end of reading words_waiting from vs buffer
                      
                      words_waiting = 0;      
                }
                // if any data read ie words_waiting > header size of 1368
                if (data_size > 0)
                {
                  //blocks_read = data_size / 256;
                  // if any data read - then updated oggs_count as first word of data block is OggS
                  oggs_count = 1; // assuming the word Oggs follows after header and will be written in next ring file
                }
                  
                recording_state = STARTED_RECORDING;    /// doing this after setup and header to avoid corrupt data due to frequent button press
                
                /// ********* READ AUDIO DATA ************** ///  
                // state 0: Normal recording;    state 1: requested encoder to stop;    state 2: encoder stopped, reading remaining buffer;     state 3: recording stopped
                // Start reading VS buffer ie HDAT1>0, read 2 bytes from HDAT0 till required say 250 blocks (256 bytes each) are read
                // append each byte to audio file
                while (state < 3)       
                {
                      if ((recording_state==STOP_RECORDING) && (state == 0))   // when user has pressed out record button
                      {
                        /// ********* Request encoder to stop recording ************** ///     
                        state = 1;    //state 1: requested encoder to stop;
                        ESP_LOGI(TAG, "state 1: requested encoder to stop");
                        // set bit 0 of ctrl3 - request encoder to stop
                        write_register(SCI_AICTRL3, read_register(SCI_AICTRL3) | 1);
                      }
                      
                      // read how many 16-bit words in buffer at this time
                      //vTaskDelay(1 / portTICK_RATE_MS); 
                      hdat1_16bit = read_register(SCI_HDAT1);
                      if (hdat1_16bit < 0xFFF)
                        words_waiting = hdat1_16bit;    // to avoid using any values more than 4096 ie the buffer size. spi read error can make hdat1 0xffff ie 65,535
                      //else
                        //ESP_LOGE(TAG, "sci_hdat1 reads %04X", hdat1_16bit);
                      
                      /// ********* Encoder has stopped recording ************** /// 
                      // check bit 1 of ctrl3 - if set - ogg vorbis encoder has finished encoding
                      if ( (state==1) && (read_register(SCI_AICTRL3) & 0x02) )
                      {
                          state = 2;  //state 2: encoder stopped, reading remaining buffer
                          ESP_LOGI(TAG, "state 2: encoder stopped, reading remaining buffer");
                          // re-read how many 16-bit words in buffer at this time - important to capture latest if this has changed from previous read
                          hdat1_16bit = read_register(SCI_HDAT1);
                          if (hdat1_16bit < 0xFFF)
                            words_waiting = hdat1_16bit;    // to avoid using any values more than 4096 ie the buffer size. spi read error can make hdat1 0xffff ie 65,535
                          //else
                            //ESP_LOGE(TAG, "sci_hdat1 reads %04X", hdat1_16bit);
                      }
                      //ESP_LOGI(TAG, "Buffer status = words_waiting = %d   blocks_read = %d   state= %d  counter= %d", words_waiting, blocks_read,state,j++);
                      
                      // while there are words in the buffer to read - 512 bytes if encoder not yet stopped OR at least 1 byte if encoder has stopped
                      while ( ((words_waiting >= 256) && (state < 2)) || ((words_waiting >= 1) && (state >= 2)) )
                      {
                            if(words_waiting < 256)
                            {
                              words_to_read = words_waiting;
                              ESP_LOGI(TAG, "words_waiting < 256, words_to_read = %d", words_to_read);
                            }
                            else
                              words_to_read = 256;
                            
                            // assume we would read at lease words_to_read -> reduce words_waiting by that amount
                            words_waiting = words_waiting - words_to_read;
                            
                            /// ********* Encoder has stopped recording and this is the last block to read ************** ///
                            if ((state==2) && (words_waiting==0))
                                words_to_read = words_to_read - 1;    // need to skip last word as this would be read in a special manner
                            
                            /// ********* This is the main Encoder recording loop ***************** ///
                            //printf("\n");
                            for (i=0; i< words_to_read; i++)
                            {
                                hdat0_16bit = read_register(SCI_HDAT0);
                                audio_data[2*i] = (uint8_t)(hdat0_16bit >> 8); // read msb
                                audio_data[(2*i)+1] = (uint8_t)(hdat0_16bit & 0x00FF); // read lsb
                                //printf("%04X ", hdat0_16bit);
                            } // end of main encoder recording loop
                  
                            // if we have saved 2 blocks of data in buffer
                            if (words_to_read == 256)
                            {
                                    /// ********* CHECK MAIN BUFFER for OGGS PATTERN ***************** ///
                                    // go through the buffer (0 to 4th last byte)- check OggS location - write to correct file accordingly
                                    for(i=0; i<509; i++)
                                    {
                                      // check if complete OggS pattern matched
                                          // manage oggs_count and file to write to : ie header or ring buffer
                                          if ( (audio_data[i]==0x4f) && (audio_data[i+1]==0x67) && (audio_data[i+2]==0x67) && (audio_data[i+3]==0x53) ) 
                                              oggs_count++;
                                          // write byte to the ring buffer
                                          ring_write(audio_data[i], oggs_count);
                                          if (oggs_count > OGGS_PER_FILE)   // when oggs_count = 6
                                          {
                                              oggs_count = 1; // reset ogg_count
                                              files_updated++;
                                          }
                                      } // end of processing 509 bytes from the array
                                    
                                        /// ********* WRITE LAST 3 BYTES ***************** ///
                                          ring_write(audio_data[509], oggs_count);
                                          ring_write(audio_data[510], oggs_count);
                                          ring_write(audio_data[511], oggs_count);
                                          
                                    //blocks_read = blocks_read + 2;
                              } // end of if words_to_read == 256 ie read 2 blocks
                            
                            
                            /// ********* special reading of last byte of last block ************** ///
                            if (words_to_read < 256)
                            {
                                  state = 3;  //state 3: recording stopped
                                  ESP_LOGI(TAG, "state 3: recording stopped");
                                  hdat0_16bit = read_register(SCI_HDAT0);
                                  ring_write((uint8_t)(hdat0_16bit >> 8), oggs_count);  // read msb
                                  
                                  // read ctrl3 twice - if bit2 is set in second read, then do not write lsb from hdat0
                                  // otherwise write lsb to file as usual
                                  read_register(SCI_AICTRL3);
                                  if (!(read_register(SCI_AICTRL3) & 0x04))
                                      ring_write((uint8_t)(hdat0_16bit & 0x00FF), oggs_count);  // read lsb
                            } // end of if last word
                        
                      } // end of while reading blocks from buffer
                } // end of while state < 3
                
                ESP_LOGI(TAG, "RECORDING STOPPED...\n");
                ring_close();   // ensure last file in the ring being written to is closed
                
                // clear adpcm bit / set mic / set sdinew / set sdishare / do soft reset
                write_register(SCI_MODE, 0x0C04);
                vTaskDelay(2 / portTICK_RATE_MS); // small delay after soft reset
                dreq_wait();
                
                //read_all_registers();
                
                /// ********* CHECK RECORDING and ADD HEADER ************** ///
                // first 4 bytes of the header file should be 0x4f 0x67 0x67 0x53 = OggS
                // while recording - can read - recording time / avg bit rate / sample counter
                
              
                print_header();
                //ring_print();
                
                ring_updateOldestFilePointer();   // ensure oldest pointer is updated in the metadata file
                
                // read back partition information to check file size - 251 byte block used by SPIFFS
                partition_info();
                
                recording_state = RECORDER_OFF;
                ESP_LOGI(TAG, "recording state has now changed to = RECORDER_OFF %d", recording_state);
                
    } // end of if record event occured and added to record queue by ISR
  } // end of infinite while to keep the task alive

}   // end of record_task
                
                
                




void player_setup()
{
  //ESP_LOGI(TAG, "\n\nSwitch on speakers - 3 sec delay...- THIS NEEDS TO BE SWITCHED BACK ON LATER");
  //vTaskDelay(3000 / portTICK_RATE_MS);
  
  // ensure XDCS is high - connected to 3.3v output from esp8266
  vs_hard_reset();    // hard reset vs1053 includes sdi_patch
  
  // this decoder patch is not helping - in fact it deteriorates the quality
  
  /*load_plugin(vs_decoder_patch, sizeof(vs_decoder_patch)/sizeof(vs_decoder_patch[0]));   // includes aiaddr = 0x50 as last instruction
  dreq_wait();
  if( (read_register(SCI_WRAM)==0x824e) && (read_register(SCI_WRAMADDR==0x8025)) )
    ESP_LOGI(TAG, "vs decoder patch applied successfully");
  else
    ESP_LOGE(TAG, "error loading vs decoder, wramaddr!= 0x8025 &/or wram!= 0x824e");*/
  
  
  // increase SPI speed
  spi_clk_div_t spi_clk = SPI_8MHz_DIV;
  esp_err_t ret = spi_set_clk_div(HSPI_HOST, &spi_clk);
  if (ret != ESP_OK) 
    ESP_LOGE(TAG, "spi error - clk NOT set to 8MHz");
  else
    ESP_LOGI(TAG, "spi clk set to 8MHz");
  
  // check all registers are fine
  //read_all_registers();
}

void play_radio(uint8_t *stream, uint16_t stream_size)
{
  uint8_t i, j;
  uint16_t offset, index;
  uint32_t audio_buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t chunks_32, chunk_small;
  spi_trans_t writeTrans;
  
  
  chunks_32 = stream_size / 32;
  chunk_small = stream_size % 32;
  
  //ESP_LOGI(TAG, "chunks_32 = %d chunk_small = %d", chunks_32, chunk_small);
  
  // send 32 bytes at a time to play to vs1053
  for (i=0; i<chunks_32; i++)
  {
        offset = i*32;
        //ESP_LOGI(TAG, "inside chunks_32 i = %d", i);
        for (j=0;j<8;j++)
        {
          index = offset+j*4;
          // go through 32 bytes of the audio stream and fill uint32_t buffer
          audio_buffer[j] = (stream[index]<<24) | (stream[index+1]<<16) | (stream[index+2]<<8) | (stream[index+3]);
          //printf("%08X", audio_buffer[j]);
        }
        writeTrans.bits.val = 0;            // clear all bits
        writeTrans.mosi = audio_buffer;
        writeTrans.bits.mosi = 32*8;
        data_mode_on();
        spi_trans(HSPI_HOST, &writeTrans);
        
        //vTaskDelay(30 / portTICK_RATE_MS); // datasheet says dreq needs ~1.8ms
        dreq_wait();
        data_mode_off();
  }
  
  // deal with less than 32 bytes - if any
  if (chunk_small > 0)
  {
          //ESP_LOGI(TAG, "Dealing with less than 32 bytes ie %d bytes", chunk_small);
          uint32_t small_buffer;
          uint8_t quads, remainder;  
          
          offset = chunks_32*32;
          
          quads = chunk_small / 4;
          remainder = chunk_small % 4; 
          for (i=0; i<quads; i++)
          {
            index = offset + i*4;
            small_buffer = (stream[index]<<24) | (stream[index+1]<<16) | (stream[index+2]<<8) | (stream[index+3]);
            
            writeTrans.bits.val = 0;            // clear all bits
            writeTrans.mosi = &small_buffer;
            writeTrans.bits.mosi = 4*8;
            data_mode_on();
            spi_trans(HSPI_HOST, &writeTrans);
            
            //vTaskDelay(40 / portTICK_RATE_MS); // datasheet says dreq needs ~1.8ms
            dreq_wait();
            data_mode_off();
          }
          
          // to reach to the correct position in temp_buffer where remaining (<4) bytes are present
          offset = chunks_32*32+quads*4;
          
          for (j=0;j<remainder; j++)
          {
              small_buffer = (stream[offset+j]<<24);
              
              writeTrans.bits.val = 0;            // clear all bits
              writeTrans.mosi = &small_buffer;
              writeTrans.bits.mosi = 1*8;
              data_mode_on();
              spi_trans(HSPI_HOST, &writeTrans);
              
              //vTaskDelay(40 / portTICK_RATE_MS); // datasheet says dreq needs ~1.8ms
              dreq_wait();
              data_mode_off();
          }
  } // end of else - dealing with last part of the file ie bytes_read < 32

} // end of playing radio

// this task reads audio data from queue and sends to vs1053 32 bytes at a time
void unload_task(void *arg)
{
  uint32_t audio_buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t temp_buffer[4];
  uint8_t i, j;
  spi_trans_t writeTrans;
  
  player_setup();
  
  while  (radio_state == STARTED_RADIO)  // until isr changes it to STOP_RADIO when radio_button is pressed again
  {
      bzero(audio_buffer, sizeof(audio_buffer));    // erases data (by writing \0) in n bytes of memory starting from location pointed to by audio_buffer
      // read 32 bytes from queue - 4 bytes at a time
      for (i=0;i<8;i++)
      {
            bzero(temp_buffer, sizeof(temp_buffer));    // erases data (by writing \0) in n bytes of memory starting from location pointed to by temp_buffer
            // read from queue - wait indefinitely in case queue is empty
            for (j=0;j<4;j++)
            {
                  // would wait in blocked state for 5 seconds for load task to refill the empty queue
                  // with portMAX_DELAY as task block time if queue is empty, this line will not get executed
                  if (xQueueReceive(radio_data_queue, &temp_buffer[j], 5000 / portTICK_RATE_MS) == errQUEUE_EMPTY)
                        ESP_LOGE(TAG, "UNLOAD_TASK: found radio queue empty in unload task j=%d",j); 
                  if (radio_state != STARTED_RADIO)
                        break;
            }
            if (radio_state != STARTED_RADIO)
                  break;
            audio_buffer[i] = (temp_buffer[0]<<24) | (temp_buffer[1]<<16) | (temp_buffer[2]<<8) | (temp_buffer[3]);
      }
      if (radio_state != STARTED_RADIO)
          break;
      
      // send these 32 bytes to vs
      writeTrans.bits.val = 0;            // clear all bits
      writeTrans.mosi = audio_buffer;
      writeTrans.bits.mosi = 32*8;
      data_mode_on();
      spi_trans(HSPI_HOST, &writeTrans);
      
      //vTaskDelay(30 / portTICK_RATE_MS); // datasheet says dreq needs ~1.8ms
      dreq_wait();
      data_mode_off();
      
      //ESP_LOGI(TAG, "In unload_task, after sending 32 bytes to vs radio queue size  = %d", (uint16_t)uxQueueMessagesWaiting(radio_data_queue) );
  } // end of while radio state = started radio
  
  stop_player();
  ESP_LOGD(TAG, "UNLOAD_TASK: stopped player, deleting unload task");
  vTaskDelete(NULL);
}


void play_file(char *file_name)
{
    uint8_t i, j=0, bytes_read, temp_buffer[32];
    uint32_t file_buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    spi_trans_t writeTrans;
    
    FILE *f = fopen(file_name, "r");
    // loop through 32 bytes from audio file and send via sdi
    while(1)
    {
          
          if ( (playing_state == STOP_PLAYING) || (playing_state == PLAY_NEXT) )
                break;    // stop playing the current file and return to play_task
          if (gpio_get_level(share_button) == 1) // share button 0->1 (touchpad) = prev file
          {
              playing_state = PLAY_PREV;
              break;    // stop playing the current file and return to play_task 
          }
          // fread reads uint32_t as little endian (lsb first) due to esp architecture i.e., 0x47, 0x53, 0x68, 0x71 becomes 0x71685347
          // using separate temp_buffer (of 1 byte elements) as fread fills that in desired order
          bytes_read = fread(temp_buffer, 1, 32, f);
          //ESP_LOGI(TAG, "playing_state= %d bytes_read = %d", playing_state, bytes_read);
          if (bytes_read <= 0)
              break;
          if (bytes_read == 32)
          {
                // to send 4 valid bytes in one go in mosi, converting 4 elements of 1 byte each (from temp_buffer) to 1 element of 4 bytes (in file_buffer) - order of bytes remain same
                // e.g. 0x47, 0x53, 0x68, 0x71 becomes 0x47536871      
                for (i=0;i< bytes_read/4; i++)
                  file_buffer[i] = (temp_buffer[i*4]<<24) | (temp_buffer[(i*4)+1]<<16) | (temp_buffer[(i*4)+2]<<8) | (temp_buffer[(i*4)+3]);
                
                writeTrans.bits.val = 0;            // clear all bits
                writeTrans.mosi = file_buffer;
                writeTrans.bits.mosi = bytes_read*8;
                data_mode_on();
                spi_trans(HSPI_HOST, &writeTrans);
                
                //vTaskDelay(30 / portTICK_RATE_MS); // datasheet says dreq needs ~1.8ms
                dreq_wait();
                data_mode_off();
                //ESP_LOGI(TAG, "bytes_read = 32 counter= %d", j++);
          }
          else
          {
                  //ESP_LOGI(TAG, "started dealing with <32 bytes");
                  // deal with less than 32 bytes
                  uint32_t small_buffer;
                  uint8_t quads, remainder;  
                  
                  quads = bytes_read / 4;
                  remainder = bytes_read % 4; 
                  i=0;
                  for (i=0; i<quads; i++)
                  {
                    small_buffer = (temp_buffer[i*4]<<24) | (temp_buffer[(i*4)+1]<<16) | (temp_buffer[(i*4)+2]<<8) | (temp_buffer[(i*4)+3]);
                    
                    writeTrans.bits.val = 0;            // clear all bits
                    writeTrans.mosi = &small_buffer;
                    writeTrans.bits.mosi = 4*8;
                    data_mode_on();
                    spi_trans(HSPI_HOST, &writeTrans);
                    
                    //vTaskDelay(40 / portTICK_RATE_MS); // datasheet says dreq needs ~1.8ms
                    dreq_wait();
                    data_mode_off();
                  }
                  i = i*4; // to reach to the correct position in temp_buffer where remaining (<4) bytes are present
                  for (j=0;j<remainder; j++)
                  {
                    small_buffer = (temp_buffer[i]<<24);
                    i++;
                    
                    writeTrans.bits.val = 0;            // clear all bits
                    writeTrans.mosi = &small_buffer;
                    writeTrans.bits.mosi = 1*8;
                    data_mode_on();
                    spi_trans(HSPI_HOST, &writeTrans);
                    
                    //vTaskDelay(40 / portTICK_RATE_MS); // datasheet says dreq needs ~1.8ms
                    dreq_wait();
                    data_mode_off();
                  }
          } // end of else - dealing with last part of the file ie bytes_read < 32
          
    } // end of while(1) going through the current audio file
    fclose(f);
}

void stop_player()
{
  uint8_t i;
  uint32_t file_buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint16_t endFillByte;
  spi_trans_t writeTrans;
  
  /*readchar = read_register(SCI_HDAT1);
   if(readchar != 0x4F67)
  ESP_LOGE(TAG, "Invalid ogg vorbis - HDAT1 should have been 0x4F67 but it is %04X", readchar);
  else
  ESP_LOGI(TAG, "ogg vorbis file valid, sci_hdat1=0x4F67");*/
  //HDAT1 = 0x4f67 and HDAT0 = avg byte rate
  
  /// ********* Send endfillbyte and stopping player ***************** ///
  // read endFillByte from 0x1e06
  endFillByte = wram_read(0x1e06);
  //ESP_LOGI(TAG, "endFillByte = %04X", endFillByte);
  for (i=0;i<8;i++)
    file_buffer[i] = (endFillByte<<24) | (endFillByte<<16) | (endFillByte<<8) | (endFillByte);
  
  // send at least 2052 endFillBytes 65*32 = 2080
  for (i=0;i<65;i++)
  {
    writeTrans.bits.val = 0;            // clear all bits
    data_mode_on();
    writeTrans.mosi = file_buffer;
    writeTrans.bits.mosi = 32*8;
    spi_trans(HSPI_HOST, &writeTrans);
    //vTaskDelay(40 / portTICK_RATE_MS); // datasheet says dreq needs ~1.8ms
    dreq_wait();
    data_mode_off();
  }
  // set SM_CANCEL
  // set mic / set sdinew / set sdi shared bit - DCS pin in LC technology board is not working so SDI will not work without this
  write_register(SCI_MODE, 0x0C08);
  //vTaskDelay(10 / portTICK_RATE_MS);
  //ESP_LOGI(TAG, "after writing to sci_mode");
  // keep sending 32 endFillBytes, till SM_CANCEL is reset. If 2048 endFillBytes (64 times) sent then soft reset
  i = 0;
  while (read_register(SCI_MODE) & (1<<SM_CANCEL))
  {
    writeTrans.bits.val = 0;            // clear all bits
    data_mode_on();
    writeTrans.mosi = file_buffer;
    writeTrans.bits.mosi = 32*8;
    spi_trans(HSPI_HOST, &writeTrans);
    //vTaskDelay(40 / portTICK_RATE_MS); // datasheet says dreq needs ~1.8ms
    dreq_wait();
    data_mode_off();
    
    i++;
    if(i==64)
    {
      ESP_LOGE(TAG, "SM_CANCEL not being cleared, triggering soft reset [should be rare]");
      // do software reset
      write_register(SCI_MODE, read_register(SCI_MODE) | (1<<SM_RESET));
      //vTaskDelay(10 / portTICK_RATE_MS);  // small delay after soft reset
      dreq_wait();
      break;
    }
  }
  
  // Check hdat0 and 1 = 0
  /*if (!((read_register(SCI_HDAT0)==0) && (read_register(SCI_HDAT1)==0)))
  ESP_LOGE(TAG, "Audio stopped, but HDAT0 / HDAT1 are not 0");
  else
  ESP_LOGI(TAG, "AUDIO STOPPED...\n");*/
}

void play_task(void *arg)
{
  uint8_t io_num, index;

  while (1) // to keep this task alive
  {
    if (xQueueReceive(play_evt_queue, &io_num, portMAX_DELAY)) 
    {  
          // initiate player
          ESP_LOGI(TAG, "playing state = START_PLAYING %d GPIO %d Current value= %d", playing_state, io_num, gpio_get_level(io_num));
              
              // initialize variables required for fresh playing
              
              player_setup();
              
              ESP_LOGI(TAG, "Entering player");
              index = ring_getOldestFilePointer();
              struct stat st;
              
              //play the header file if it exists
              stat(file_header, &st);
              if (st.st_size > 0)
                  play_file(file_header);
              else
                ESP_LOGI(TAG, "Empty header file");
              
              playing_state = STARTED_PLAYING;    /// doing this after setup and header to avoid corrupt state due to frequent button press
              
              /// ********* This outer loop to go through ring buffer playing each file ***************** ///
              // Play each file in the ring buffer - starting from the oldest file in the ring
              while(1) 
              {
                      //ESP_LOGI(TAG, "Checking if playing state = stop playing - then break.. current playing_state = %d", playing_state);
                      if (playing_state == STOP_PLAYING)
                          break;
                      
                      stat(ring_getFileName(index), &st);
                      //ESP_LOGI(TAG, "File size = %ld", st.st_size);
                      if (st.st_size ==0)
                      {
                        //ESP_LOGI(TAG, "Empty file %s. Moving to next file in the ring", ring_getFileName(index));
                        ring_next(&index);
                        continue;
                      }
                      ESP_LOGI(TAG, "Playing %s file size = %ld", ring_getFileName(index), st.st_size);
                      
                    /// ********* Looping through current audio file 32 bytes at a time ***************** ///
                      // loop through 32 bytes from audio file and send via sdi
                      play_file(ring_getFileName(index));
                      
                      /// ********* Move to the prev / next file in the ring buffer ***************** ///
                      if (playing_state == PLAY_PREV)
                      {
                          ring_prev(&index);   
                          vTaskDelay(200 / portTICK_RATE_MS); // to avoid button bounce - as this is a gpio read - it would stay in high state for a long time
                      }
                      else
                          ring_next(&index);
                      
                      // revert to normal playing state if play next or prev executed 
                      if ( (playing_state == PLAY_NEXT) || (playing_state == PLAY_PREV) )
                          playing_state = STARTED_PLAYING;
                }   // end of infinite loop through files of ring buffer
              
                stop_player();    // send endfillbytes
                ESP_LOGI(TAG, "AUDIO STOPPED...\n");        
                
                playing_state = PLAYER_OFF;
                ESP_LOGI(TAG, "playing state has now changed to = PLAYER_OFF %d", playing_state);
                ESP_LOGI(TAG, "at end of play task min so far esp_get_minimum_free_heap_size() = %d", esp_get_minimum_free_heap_size());
                      
    } // end of if play event occured and added to play queue by ISR
    
  } // end of infinite while to keep the task alive
  
}   // end of play_task



