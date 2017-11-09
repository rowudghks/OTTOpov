#include <SdFat.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include "./gamma.h"

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
SdFile            root;        // Root directory (only one)
volatile uint8_t *port;        // NeoPixel PORT register


//int h = 126;  //height
//int cnt=0;
//unsigned long lastTimeLoop;
//unsigned long timeLoop;
//unsigned long times;
//unsigned long delaytime;
char buf[33]={0};
unsigned long time, oldtime, delta, delaytime = 100;
float spd=0;
int swval=0, oldswval=0;
int x=0;


void setup()
{
  uint8_t  b, startupTrigger, minBrightness;
  char     infile[13], outfile[13];
  boolean  found;
  uint16_t i, n;
  SdFile   tmp;
  uint32_t lastBlock;

  digitalWrite(TRIGGER, HIGH);           // Enable pullup on trigger button
  startupTrigger = digitalRead(TRIGGER); // Poll startup trigger ASAP
  Serial.begin(9600);
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

  if(!volume.init(&card)) {
    error(F("Could not find FAT16/FAT32 partition.\n"
            "Make sure the card is formatted."));
  }
  root.openRoot(&volume);

  if(startupTrigger == HIGH) { // No button press

    minBrightness = 155;
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


      b = map(200, 0, 1023, 1, minBrightness);

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

}
// Startup error handler; doesn't return, doesn't run loop(), just stops.
static void error(const __FlashStringHelper *ptr) {
    Serial.println(ptr); // Show message
    for(;;);             // and hang
}

void loop() { 
  uint32_t block    = 0;     // Current block # within file
  int f = 1;     // Current block # within file
  uint32_t olddelaytime = 100;
  boolean  stopFlag = false; // If set, stop playback loop
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
  //while(digitalRead(TRIGGER) == HIGH);
  
  for(;;) {      
    swval = digitalRead(TRIGGER);
     if (oldswval == LOW && swval == HIGH)
      {
        time = millis();
        delta = millis() - oldtime;
        delaytime = (delta * 1000) / 107;
        //26" * 25.4 * 3.141592 = 2.074m, 0.002174 km;
        // * 3600 = 7446.4 km/s 
        spd = 7466.4 / (float)delta;//7545.6 / (float)delta;
        sprintf(buf, "SPD %3ld RPS %4ld", (long)spd, (long)(1.0 / (float)delta * 1000.0));
        Serial.println(buf);
        Serial.println((unsigned long)delaytime);    
        oldtime = time;
      }
    if(++block >= nBlocks  || digitalRead(TRIGGER) == LOW) {
       if (f==40) {f=0;}
      if(digitalRead(TRIGGER) == HIGH) {       // Trigger released?
        memset(sdBuf, 0, N_LEDS * 3);          // LEDs off on next pass
        stopFlag = true;                       // Stop playback on next pass
        continue;
      }      // Past last block?      // Past last block?
        switch (x)
  {
    case 0 :     if(f<2){block = 0;}
    case 1 :     if(f<4 && f>1){block = 107;}
    case 2 :     if(f<6 && f>3){block = 214;}
    case 3 :     if(f<8 && f>5){block = 321;}
    case 4 :     if(f<10 && f>7){block = 428;}
    case 5 :     if(f<12 && f>9){block = 535;}
    case 6 :     if(f<14 && f>11){block = 642;}
    case 7 :     if(f<16 && f>13){block = 749;}
    case 8 :     if(f<18 && f>15){block = 856;}
    case 9 :     if(f<20 && f>17){block = 963;}
    case 10 :     if(f<22 && f>19){block = 1070;}
    case 11 :     if(f<24 && f>21){block = 1177;}
    case 12 :     if(f<26 && f>23){block = 1284;}
    case 13 :     if(f<28 && f>25){block = 1391;}
    case 14 :     if(f<30 && f>27){block = 1498;}
    case 15 :     if(f<32 && f>29){block = 1605;}
    case 16 :     if(f<34 && f>31){block = 1712;}
    case 17 :     if(f<36 && f>33){block = 1819;}
    case 18 :     if(f<38 && f>35){block = 1926;}
    case 19 :     if(f<40 && f>37){block = 2033;} 
    //case 20 :     if(f<42 && f>39){block = 2140;}
    //case 21 :     if(f<44 && f>41){block = 2247;}
    //case 22 :     if(f<46 && f>43){block = 2354;}
    //case 23 :     if(f<48 && f>45){block = 2461;}
    //case 24 :     if(f<50 && f>47){block = 2568;}
    //case 25 :     if(f<52 && f>49){block = 2675;}
    //case 26 :     if(f<54 && f>51){block = 2782;}
    //case 27 :     if(f<56 && f>53){block = 2889;}
    //case 28 :     if(f<58 && f>55){block = 2996;}
    //case 29 :     if(f<60 && f>57){block = 3103;}
  //case 30 :     if(f<62 && f>59){block = 3210;}
  }
      f+=2;
    }
  oldswval = swval;
  card.readBlock(block + firstBlock, sdBuf); // Load next pixel row
  show();                                    // Display current line
  delayMicroseconds((unsigned long)delaytime);  
 }
  if(++frame >= nFrames) frame = 0;
}

/*
void play(int x){
        card.readBlock(x+firstBlock, sdBuf);
        show();
        times = micros();
        //phase = (phase + 1) % h;
        if(x==0){phase = (phase + 1) % h;}
        if(cnt<h-1){cnt=cnt+1;} else{cnt=0;phase=0;}
} 
//Serial.print("case 0 delay"); Serial.println(delaytime);
//Serial.print("delay"); Serial.println(delaytime);
void sw() {
  switch (phase)
  {
    case 0 : 
      if (GetEmicroTime(times) >= 1000 && digitalRead(TRIGGER)==LOW) { play(0);} break;
    case 1 : 
      if (GetEmicroTime(times) >= 1000 ) { play(cnt);} break;
  } //switch
 }// sw()

unsigned long GetEmicroTime(unsigned long referenceTime)
{
  unsigned long returnValue;
  unsigned long currentMicros = micros();
  if (referenceTime > currentMicros)
  {
    returnValue = 4294967295 + (currentMicros - referenceTime);
    returnValue++;
  }
  else
  {
    returnValue = currentMicros - referenceTime;
  }
  return returnValue;
}
*/
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
