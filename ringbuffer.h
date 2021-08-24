
// num of ogg pages per ring buffer file to record - this is roughly around 100 blocks - 15 seconds recording
#define OGGS_PER_FILE 5

typedef struct {
  uint8_t oldest_file_pointer;          
  uint8_t num_of_files;
  char **files_buffer;
} ring_buffer_t;


void ring_metadataCreate(uint8_t num_files);
void ring_printMetadata();

void ring_init(uint8_t num_files);
void ring_print();

void ring_close();
void ring_write(uint8_t c, uint8_t oggs_count);
void ring_move();

uint8_t ring_getOldestFilePointer();
uint8_t ring_getNumOfFiles();
char* ring_getFileName(uint8_t index);

void ring_next(uint8_t* index);
void ring_prev(uint8_t* index);

void ring_updateOldestFilePointer();


