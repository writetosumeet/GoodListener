#include "string.h"
#include "stdio.h"
#include "stdlib.h"

#include "sys/unistd.h"
#include "sys/stat.h"
#include "esp_spiffs.h"
#include "esp_log.h"
#include "ringbuffer.h"

static const char *TAG = "ring_buffer";

static const char* metadata_filename="/spiffs_main/ringbuffer_metadata.txt";

static ring_buffer_t ring;    // static global variable = visible only in ringbuffer.c
// normal global variable here can be accessed as "extern ring_buffer_t ring" from another linked file say main.c

FILE *current_file=NULL;


// store oldest pointer + array size +array contents in a metadata file - parameter: number of files of RING_BUFFER
void ring_metadataCreate(uint8_t num_files)
{
  uint8_t i, j; 
  char file_num[3];
  FILE* audio_f;
  char ring_buffer_temp[num_files][20];     // num_files represents the number of files in ring buffer
  
  // create file to store ring buffer metadata
  // File structure = 
  // Byte 1 = OLDEST_FILE pointer
  // Byte 2 = ring buffer array size
  // Byte 3 to 22 (ie 19 bytes) - 0th file name
  // Byte 23 to 42 (ie 19 bytes) - 1st file name and so on
  FILE* f = fopen(metadata_filename, "w");
  
  fputc(0, f);    // OLDEST_FILE pointer
  fputc(num_files, f);    // ring buffer array size

  for (i=0; i<num_files; i++)
  {
    strcpy(ring_buffer_temp[i], "/spiffs_ring/");
    itoa(i,file_num,10);
    if(i<10)
        strcat(ring_buffer_temp[i],"0");
    strcat(ring_buffer_temp[i], file_num);
    strcat(ring_buffer_temp[i],".ogg");
    //filename length is fixed - 19 e.g. '/spiffs_ring/01.ogg'
    for (j=0; j<19; j++)
    {
      //ESP_LOGI(TAG, "to be written to file = %c", ring_buffer_temp[i][j]);
      fputc(ring_buffer_temp[i][j], f);
    }
    // create corresponding blank file
    audio_f = fopen(ring_buffer_temp[i], "w");
    fclose(audio_f);
    
    ESP_LOGD(TAG, "wrote %s to metadata",ring_buffer_temp[i]);
  }
  fclose(f);
  ESP_LOGI(TAG, "********* ring_buffer metadata file created along with %d empty files ********", num_files);
}


void ring_printMetadata()
{
  uint8_t readChar;
  FILE* f = fopen(metadata_filename, "r");
  printf("%d ", fgetc(f));  // printing oldest file pointer
  printf("%d ", fgetc(f));  // printing number of files in the ring
  while(1)
  {
    readChar = fgetc(f);
    if( feof(f) ) 
        break;
    printf("%c", readChar);
  }
  printf("\n");
  fclose(f);
}



void ring_init(uint8_t num_files)
{
  FILE* f;
  uint8_t i;
  
  f = fopen(metadata_filename, "r");
  if (f==NULL)    // case 1: metadata file does not exist, should happen only if it is first time run after erase_flash
  {
      ring_metadataCreate(num_files);
      ring.num_of_files = 0;   // an indicator that new metadata file has been created
  }
  else    // case 2: this should normally be the case - metadata file exists and num_of_files is not changed
  {
    ring.oldest_file_pointer = fgetc(f);
    ring.num_of_files = fgetc(f);
    if(ring.num_of_files != num_files)   // case 3: metadata file exists but user wants to change num_of_files
    {
        fclose(f);
        ring_metadataCreate(num_files); 
        ring.num_of_files = 0; // an indicator that new metadata file has been created
    }
  }
  
  if (ring.num_of_files == 0) // if a new metadata file has been created
  {
      f = fopen(metadata_filename, "r");
      ring.oldest_file_pointer = fgetc(f);
      ring.num_of_files = fgetc(f);
  }
  
  ring.files_buffer = malloc(num_files * sizeof(char*));  // now that we know the number of rows (array length)
  // copying contents of RING_BUFFER
  for (i=0;i<num_files; i++)
  {
    ring.files_buffer[i] = malloc(20);    // array width = file name length (fixed 19) + 1 for '\0'
    fgets(ring.files_buffer[i], 20, f);   
  }
  fclose(f);
  
  ESP_LOGI(TAG, "ring.files_buffer initialized with %d files, oldest file pointer = %d",num_files, ring.oldest_file_pointer);
  for (i=0;i<num_files; i++)
      ESP_LOGD(TAG, "ring.files_buffer[%d] = '%s'", i, ring.files_buffer[i]);
}


void ring_print()
{
  uint8_t i, readChar;
  uint16_t j;
  FILE* f;
  uint8_t oggs[4] = {0,0,0,0};
  
  for (i=0;i<ring.num_of_files;i++)  
  {
      ESP_LOGI(TAG, "Reading %s", ring.files_buffer[i]);
      f = fopen(ring.files_buffer[i], "r"); 
      j = 0;
      while(1)
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
        j++;
      }
      printf("\n");
      fclose(f); 
  }
}

void ring_close()
{
  // close currently open file - if any
  if (current_file != NULL)
  {
    fclose(current_file);
    current_file = NULL;
  }
}

void ring_move()
{
  // close currently open file - if any
  ring_close();
  
  // open oldest file for writing in w mode (overwrite)
  current_file = fopen(ring.files_buffer[ring.oldest_file_pointer], "w");
  //ESP_LOGI(TAG, "%s file opened for writing", ring.files_buffer[ring.oldest_file_pointer]);
  
  // move oldest pointer
  ring.oldest_file_pointer++;
  if (ring.oldest_file_pointer == ring.num_of_files)    // if at end of buffer then move to beginning of buffer
      ring.oldest_file_pointer = 0;
}



void ring_write(uint8_t c, uint8_t oggs_count)    
{
  // check ogg count > OGGS_PER_FILE    // say ogg_count = 6, then need to open new file and move oldest file pointer
  // OR if ring buffer is used first time and is currently null
  if ((oggs_count > OGGS_PER_FILE) || (current_file == NULL) )
  {
    ESP_LOGI(TAG, "oggs_count = %d, Moving to %s file for writing", oggs_count, ring.files_buffer[ring.oldest_file_pointer]);
    ring_move();    // move to next file in ring buffer
  }
  
  fputc(c, current_file);
}


uint8_t ring_getOldestFilePointer()
{
  return ring.oldest_file_pointer;
}

uint8_t ring_getNumOfFiles()
{
  return ring.num_of_files;
}

char* ring_getFileName(uint8_t index)
{
  return ring.files_buffer[index];
}


void ring_next(uint8_t* index)
{
  (*index)++;
  if (*index == ring.num_of_files)
    *index = 0;
}


void ring_prev(uint8_t* index)
{
  if (*index == 0)
    *index = ring.num_of_files - 1;
  else
    (*index)--;
}

void ring_updateOldestFilePointer()
{
  // update file to store ring buffer metadata - 
  // Byte 1 = OLDEST_FILE pointer
  FILE* f = fopen(metadata_filename, "r+");
  fputc(ring.oldest_file_pointer, f);    // OLDEST_FILE pointer
  fclose(f);
  ESP_LOGI(TAG, "oldest file pointer %d saved in metadata file", ring.oldest_file_pointer);
}

