#include <SdFat.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include "./gamma.h"

// Ported to SdFat from the native Arduino SD library example by Bill Greiman
// On the Ethernet Shield, CS is pin 4. SdFat handles setting SS
const int chipSelect = 4;
byte data = 0;

int resetPin = 2;
int line = 0;
//#define baudrate 115200
#define baudrate 9600
// CONFIGURABLE STUFF --------------------------------------------------------

#define N_LEDS        40 // Max value is 170 (fits one SD card block)
#define CARD_SELECT    4 // SD card select pin (some shields use #4, not 10)
#define LED_PIN        5 // NeoPixels connect here
#define SPEED         A0 // Speed-setting dial
#define BRIGHTNESS    A0 // Brightness-setting dial
#define TRIGGER        9 // Playback trigger pin
#define CURRENT_MAX 3500 // Max current from power supply (mA)


// NON-CONFIGURABLE STUFF ----------------------------------------------------

#define OVERHEAD 150 // Extra microseconds for loop processing, etc.

uint8_t           sdBuf[512],  // One SD block (also for NeoPixel color data)
                  pinMask;     // NeoPixel pin bitmask
uint16_t          maxLPS,      // Max playback lines/sec
                  nFrames = 0, // Total # of image files
                  frame   = 0; // Current image # being painted
uint32_t          firstBlock,  // First block # in temp working file
                  nBlocks;     // Number of blocks in file
Sd2Card           card;        // SD card global instance (only one)
SdVolume          volume;      // Filesystem global instance (only one)
SdFat             sd;
SdFile            root;        // Root directory (only one)
volatile uint8_t *port;        // NeoPixel PORT register

char buf[33]={0};
unsigned long time, oldtime, delta;
float spd=0;
int swval=0, oldswval=0;
int x=0;
uint8_t  b, startupTrigger, minBrightness;
char     infile[13], outfile[13];
boolean  found;
uint16_t i, n;
SdFile   tmp;
uint32_t lastBlock;

void setup()
{

  digitalWrite(resetPin, HIGH);
  pinMode(resetPin, OUTPUT); 
  digitalWrite(TRIGGER, HIGH);           // Enable pullup on trigger button
  startupTrigger = digitalRead(TRIGGER); // Poll startup trigger ASAP
  Serial.begin(baudrate);
  // change to SPI_FULL_SPEED/SPI_HALF_SPEED for performance. 
  if (!sd.begin(chipSelect, SPI_FULL_SPEED)) sd.initErrorHalt();
  
  pinMode(LED_PIN, OUTPUT);              // Enable NeoPixel output
  digitalWrite(LED_PIN, LOW);            // Default logic state = low
  port    = portOutputRegister(digitalPinToPort(LED_PIN));
  pinMask = digitalPinToBitMask(LED_PIN);
  memset(sdBuf, 0, N_LEDS * 3);          // Clear LED buffer
  show();                                // Init LEDs to 'off' state

  Serial.print(F("Initializing SD card..."));
  if(!card.init(SPI_FULL_SPEED, CARD_SELECT)) {
    error(F("failed. Things to check:\n"
            "* is a card is inserted?\n"
            "* Is your wiring correct?\n"
            "* did you edit CARD_SELECT to match the SD shield or module?"));
  }
  Serial.println(F("OK"));

}

// Startup error handler; doesn't return, doesn't run loop(), just stops.
static void error(const __FlashStringHelper *ptr) {
    Serial.println(ptr); // Show message
    for(;;);             // and hang
}

void loop() {
  while(digitalRead(TRIGGER) == HIGH) {
    if (Serial.available() >= 1) {
        data = Serial.read();
        Serial.println(data);
        Serial.flush();
        if (data == 48) {
          digitalWrite(resetPin, LOW);
        } else {
        sd.remove("frame000.bmp");
        sd.remove("frame000.tmp");
            if (!root.open("frame000.bmp", O_RDWR | O_CREAT | O_AT_END)) {
              sd.errorHalt("frame000.bmp for write failed");
              }
              root.print(char(data));
              root.close();
              line++;
         }        
        
        for(;;){
        if (Serial.available() >= 1) {
          data = Serial.read();
          Serial.flush();
          
          line++;
          // open the file for write at end like the Native SD library
          if (!root.open("frame000.bmp", O_RDWR | O_CREAT | O_AT_END)) {
            sd.errorHalt("frame000.bmp for write failed");
           }
          root.print(char(data));
          root.close();
        } else {
            if (line > 12893) //line 12896
            {
            digitalWrite(resetPin, LOW);
            }
        }
        }
    }
  }
  

 if(!volume.init(&card)) {
    error(F("Could not find FAT16/FAT32 partition.\n"
            "Make sure the card is formatted."));
    digitalWrite(resetPin, LOW);
  }
  root.openRoot(&volume);

  if(startupTrigger == HIGH) { // No button press

    minBrightness = 255;
    do {
      sprintf(infile, "frame%03d.bmp", nFrames);
      b = 255;
      if(found = bmpProcess(root, infile, NULL, &b)) { // b modified to safe max
        nFrames++;
        if(b < minBrightness) minBrightness = b;
      }
    } while(found && (nFrames < 1000));

    Serial.print(nFrames);
    Serial.print(" frames\nbrightness = ");
    Serial.println(minBrightness);


    b = map(1023, 0, 1023, 1, minBrightness);

    for(i=0; i<nFrames; i++) {
      sprintf(infile , "frame%03d.bmp", i);
      sprintf(outfile, "frame%03d.tmp", i);
      b = minBrightness;
      bmpProcess(root, infile, outfile, &b);
    }
    while(digitalRead(TRIGGER) == LOW); // Wait for button release

  } else { // Button held -- use existing data

    do { // Scan for files to get nFrames
      sprintf(infile, "frame%03d.tmp", nFrames);
      if(found = tmp.open(&root, infile, O_RDONLY)) {
        if(tmp.contiguousRange(&firstBlock, &lastBlock)) {
          nFrames++;
        }
        tmp.close();
      }
    } while(found);

  } // end startupTrigger test
  uint32_t delaytime = 100;
  uint32_t block    = 0;     // Current block # within file
  int f = 0;
  uint32_t lastBlock;
  char     infile[13];
  SdFile   tmp;

  // Get existing contiguous tempfile info
  sprintf(infile, "frame%03d.tmp", frame);
  if(!tmp.open(&root, infile, O_RDONLY)) {
    error(F("Could not open NeoPixel tempfile for input"));
  }
  if(!tmp.contiguousRange(&firstBlock, &lastBlock)) {
    error(F("NeoPixel tempfile is not contiguous"));
  }
  nBlocks = tmp.fileSize() / 512;
  tmp.close(); // File handle is no longer accessed, just block reads
  //card.readBlock(firstBlock, sdBuf);
  
  for(;;) {      
    if (Serial.read() == 48) digitalWrite(resetPin, LOW);
    swval = digitalRead(TRIGGER);
     if (oldswval == LOW && swval == HIGH)
      {
        time = micros();
        delta = micros() - oldtime;
        //26" * 25.4 * 3.141592 = 2.074m, 0.002174 km;
        // * 3600 = 7446.4 km/s 
        spd = 7653852 /  (float) delta;
       sprintf(buf, "SPD %3ld RPS %4ld", (long)spd, (long)(1.0 / (float)delta * 1000.0));
//        Serial.print("SPEED : ");
//        Serial.println(spd);   
        oldtime = time;
      }
    if(++block >= nBlocks  || digitalRead(TRIGGER) == LOW) {
        block = 0;   
    }
  oldswval = swval;
  switch (x)
  {
    case 0 : if(spd>1 && spd<2){delaytime=16000;}
    case 1 : if(spd>2 && spd<3){delaytime=14000;}
    case 2 : if(spd>3 && spd<4){delaytime=12000;}
    case 3 : if(spd>4 && spd<5){delaytime=10600;}
    case 4 : if(spd>5 && spd<6){delaytime=9200;}
    case 5 : if(spd>6 && spd<7){delaytime=8000;}
    case 6 : if(spd>7 && spd<8){delaytime=7000;}
    case 7 : if(spd>8 && spd<9){delaytime=6200;}
    case 8 : if(spd>9 && spd<10){delaytime=5500;}
    case 9 : if(spd>10 && spd<11){delaytime=4800;}  
    case 10 : if(spd>11 && spd<12){delaytime=4300;}
    case 11 : if(spd>12 && spd<13){delaytime=3800;}
    case 12 : if(spd>13 && spd<14){delaytime=3300;}
    case 13 : if(spd>14 && spd<15){delaytime=2800;}
    case 14 : if(spd>15 && spd<16){delaytime=2500;}
    case 15 : if(spd>16 && spd<17){delaytime=2250;}
    case 16 : if(spd>17 && spd<18){delaytime=2000;}
    case 17 : if(spd>18 && spd<19){delaytime=1650;}
    case 18 : if(spd>19 && spd<20){delaytime=1450;}
    case 19 : if(spd>20 && spd<21){delaytime=1300;}
    case 20 : if(spd>21 && spd<22){delaytime=1150;}
    case 21 : if(spd>22 && spd<23){delaytime=1000;}
    case 22 : if(spd>23 && spd<24){delaytime=850;}
    case 23 : if(spd>24 && spd<25){delaytime=650;}
    case 24 : if(spd>25 && spd<26){delaytime=500;}
    case 25 : if(spd>26 && spd<27){delaytime=400;}
    case 26 : if(spd>27 && spd<28){delaytime=300;}
    case 27 : if(spd>28 && spd<29){delaytime=200;}
    case 28 : if(spd>29 && spd<30){delaytime=100;}
    case 29 : if(spd>30 && spd<31){delaytime=50;}
    case 30 : if(spd>31 && spd<32){delaytime=25;}
    case 31 : if(spd>32){delaytime=1;} 
  }
  card.readBlock(block + firstBlock, sdBuf); // Load next pixel row
  show();                                    // Display current line
  delayMicroseconds(delaytime);
 }
  if(++frame >= nFrames) frame = 0;
}

// BMP->NEOPIXEL FILE CONVERSION ---------------------------------------------

#define BMP_BLUE  0 // BMP and NeoPixels have R/G/B color
#define BMP_GREEN 1 // components in different orders.
#define BMP_RED   2 // (BMP = BGR, Neo = GRB)
#define NEO_GREEN 0
#define NEO_RED   1
#define NEO_BLUE  2

boolean bmpProcess(
  SdFile  &path,
  char    *inName,
  char    *outName,
  uint8_t *brightness) {

  SdFile    inFile,              // Windows BMP file for input
            outFile;             // NeoPixel temp file for output
  boolean   ok        = false,   // 'true' on valid BMP & output file
            flip      = false;   // 'true' if image stored top-to-bottom
  int       bmpWidth,            // BMP width in pixels
            bmpHeight,           // BMP height in pixels
            bmpStartCol,         // First BMP column to process (crop/center)
            columns,             // Number of columns to process (crop/center)
            row,                 // Current image row (Y)
            column;              // and column (X)
  uint8_t  *ditherRow,           // 16-element dither array for current row
            pixel[3],            // For reordering color data, BGR to GRB
            b = 0,               // 1 + *brightness
            d,                   // Dither value for row/column
            color,               // Color component index (R/G/B)
            raw,                 // 'Raw' R/G/B color value
            corr,                // Gamma-corrected R/G/B
           *ledPtr,              // Pointer into sdBuf (output)
           *ledStartPtr;         // First LED column to process (crop/center)
  uint16_t  b16;                 // 16-bit dup of b
  uint32_t  bmpImageoffset,      // Start of image data in BMP file
            lineMax   = 0L,      // Cumulative brightness of brightest line
            rowSize,             // BMP row size (bytes) w/32-bit alignment
            sum,                 // Sum of pixels in row
            startTime = millis();
  if(brightness)           b = 1 + *brightness; // Wraps around, fun with maths
  else if(NULL == outName) return false; // MUST pass brightness for power est.

  Serial.print(F("Reading file '"));
  Serial.print(inName);
  Serial.print(F("'..."));
  if(!inFile.open(&path, inName, O_RDONLY)) {
    Serial.println(F("error"));
    return false;
  }

  if(inFile.read(sdBuf, 34)             &&    // Load header
    (*(uint16_t *)&sdBuf[ 0] == 0x4D42) &&    // BMP signature
    (*(uint16_t *)&sdBuf[26] == 1)      &&    // Planes: must be 1
    (*(uint16_t *)&sdBuf[28] == 24)     &&    // Bits per pixel: must be 24
    (*(uint32_t *)&sdBuf[30] == 0)) {         // Compression: must be 0 (none)
    // Supported BMP format -- proceed!
    bmpImageoffset = *(uint32_t *)&sdBuf[10]; // Start of image data
    bmpWidth       = *(uint32_t *)&sdBuf[18]; // Image dimensions
    bmpHeight      = *(uint32_t *)&sdBuf[22];
    // That's some nonportable, endian-dependent code right there.

    Serial.print(bmpWidth);
    Serial.write('x');
    Serial.print(bmpHeight);
    Serial.println(F(" pixels"));

    if(outName) { // Doing conversion?  Need outFile.
      // Delete existing outFile file (if any)
      (void)SdFile::remove(&path, outName);
      Serial.print(F("Creating contiguous file..."));
      // NeoPixel working file is always 512 bytes (one SD block) per row
      if(outFile.createContiguous(&path, outName, 512L * bmpHeight)) {
        uint32_t lastBlock;
        outFile.contiguousRange(&firstBlock, &lastBlock);
        // Once we have the first block index, the file handle
        // is no longer needed -- raw block writes are used.
        outFile.close();
        nBlocks = bmpHeight; // See note in setup() re: block calcs
        ok      = true;      // outFile is good; proceed
        Serial.println(F("OK"));
      } else {
        Serial.println(F("error"));
      }
    } else ok = true; // outFile not needed; proceed

    if(ok) { // Valid BMP and contig file (if needed) are ready
      Serial.print(F("Processing..."));

      rowSize = ((bmpWidth * 3) + 3) & ~3; // 32-bit line boundary
      b16     = (int)b;

      if(bmpHeight < 0) {       // If bmpHeight is negative,
        bmpHeight = -bmpHeight; // image is in top-down order.
        flip      = true;       // Rare, but happens.
      }

      if(bmpWidth >= N_LEDS) { // BMP matches LED bar width, or crop image
        bmpStartCol = (bmpWidth - N_LEDS) / 2;
        ledStartPtr = sdBuf;
        columns     = N_LEDS;
      } else {                 // Center narrow image within LED bar
        bmpStartCol = 0;
        ledStartPtr = &sdBuf[((N_LEDS - bmpWidth) / 2) * 3];
        columns     = bmpWidth;
        memset(sdBuf, 0, N_LEDS * 3); // Clear left/right pixels
      }

      for(row=0; row<bmpHeight; row++) { // For each row in image...
        Serial.write('.');


        inFile.seekSet(
          bmpImageoffset + (bmpStartCol * 3) + (rowSize * (flip ?
          (bmpHeight - 1 - row) : // Image is stored top-to-bottom
          row)));                 // Image stored bottom-to-top
        if(!inFile.read(ledStartPtr, columns * 3))  // Load row
          Serial.println(F("Read error"));

        sum       = 0L;
        ditherRow = (uint8_t *)&dither[row & 0x0F]; // Dither values for row
        ledPtr    = ledStartPtr;
        for(column=0; column<columns; column++) {   // For each column...
          if(b) { // Scale brightness, reorder R/G/B
            pixel[NEO_BLUE]  = (ledPtr[BMP_BLUE]  * b16) >> 8;
            pixel[NEO_GREEN] = (ledPtr[BMP_GREEN] * b16) >> 8;
            pixel[NEO_RED]   = (ledPtr[BMP_RED]   * b16) >> 8;
          } else { // Full brightness, reorder R/G/B
            pixel[NEO_BLUE]  = ledPtr[BMP_BLUE];
            pixel[NEO_GREEN] = ledPtr[BMP_GREEN];
            pixel[NEO_RED]   = ledPtr[BMP_RED];
          }

          d = pgm_read_byte(&ditherRow[column & 0x0F]); // Dither probability
          for(color=0; color<3; color++) {              // 3 color bytes...
            raw  = pixel[color];                        // 'Raw' G/R/B
            corr = pgm_read_byte(&gamma[raw]);          // Gamma-corrected
            if(pgm_read_byte(&bump[raw]) > d) corr++;   // Dither up?
            *ledPtr++ = corr;                           // Store back in sdBuf
            sum      += corr;                           // Total brightness
          } // Next color byte
        } // Next column

        if(outName) {
          if(!card.writeBlock(firstBlock + row, (uint8_t *)sdBuf))
            Serial.println(F("Write error"));
        }
        if(sum > lineMax) lineMax = sum;

      } // Next row
      Serial.println(F("OK"));

      if(brightness) {
        lineMax = (lineMax * 20) / 255; // Est current @ ~20 mA/LED
        if(lineMax > CURRENT_MAX) {
          // Estimate suitable brightness based on CURRENT_MAX
          *brightness = (*brightness * (uint32_t)CURRENT_MAX) / lineMax;
        } // Else no recommended change
      }

      Serial.print(F("Processed in "));
      Serial.print(millis() - startTime);
      Serial.println(F(" ms"));

    } // end 'ok' check
  } else { // end BMP header check
    Serial.println(F("BMP format not recognized."));
  }

  inFile.close();
  return ok; // 'false' on various file open/create errors
}

// MISC UTILITY FUNCTIONS ----------------------------------------------------

// Estimate maximum block-read time for card (microseconds)
static uint32_t benchmark(uint32_t block, uint32_t n) {
  uint32_t t, maxTime = 0L;

  do {
    t = micros();
    card.readBlock(block++, sdBuf);
    if((t = (micros() - t)) > maxTime) maxTime = t;
  } while(--n);

  return maxTime;
}

// NEOPIXEL FUNCTIONS --------------------------------------------------------

static void show(void) {
  volatile uint16_t
    i   = N_LEDS * 3; // Loop counter
  volatile uint8_t
   *ptr = sdBuf,      // Pointer to next byte
    b   = *ptr++,     // Current byte value
    hi,               // PORT w/output bit set high
    lo,               // PORT w/output bit set low
    next,
    bit = 8;

  noInterrupts();
  hi   = *port |  pinMask;
  lo   = *port & ~pinMask;
  next = lo;

  asm volatile(
   "head20_%=:"                "\n\t"
    "st   %a[port],  %[hi]"    "\n\t"
    "sbrc %[byte],  7"         "\n\t"
     "mov  %[next], %[hi]"     "\n\t"
    "dec  %[bit]"              "\n\t"
    "st   %a[port],  %[next]"  "\n\t"
    "mov  %[next] ,  %[lo]"    "\n\t"
    "breq nextbyte20_%="       "\n\t"
    "rol  %[byte]"             "\n\t"
    "rjmp .+0"                 "\n\t"
    "nop"                      "\n\t"
    "st   %a[port],  %[lo]"    "\n\t"
    "nop"                      "\n\t"
    "rjmp .+0"                 "\n\t"
    "rjmp head20_%="           "\n\t"
   "nextbyte20_%=:"            "\n\t"
    "ldi  %[bit]  ,  8"        "\n\t"
    "ld   %[byte] ,  %a[ptr]+" "\n\t"
    "st   %a[port], %[lo]"     "\n\t"
    "nop"                      "\n\t"
    "sbiw %[count], 1"         "\n\t"
     "brne head20_%="          "\n"
    : [port]  "+e" (port),
      [byte]  "+r" (b),
      [bit]   "+r" (bit),
      [next]  "+r" (next),
      [count] "+w" (i)
    : [ptr]    "e" (ptr),
      [hi]     "r" (hi),
      [lo]     "r" (lo));

  interrupts();
  // There's no explicit 50 uS delay here as with most NeoPixel code;
  // SD card block read provides ample time for latch!
}
