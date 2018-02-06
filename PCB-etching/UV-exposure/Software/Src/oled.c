
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "usb_device.h"
#include "main.h"
#include "oled.h"


void oled_draw(int16_t xMove, int16_t yMove, char* text);
void oled_draw_string(int16_t xMove, int16_t yMove, char* text, uint16_t textLength, const char *data);
void inline oled_draw_internal(int16_t xMove, int16_t yMove, int16_t width, int16_t height, const char *data, uint16_t offset, uint16_t bytesInData);
void oled_command(uint8_t cmd);
void oled_data(uint8_t dat);
void oled_reset();
unsigned char reverse(unsigned char b);

uint8_t oled_buffer[1024] = {0x00};
extern SPI_HandleTypeDef hspi1;

void oled_init(){
  oled_reset();

  oled_command(0xAE);

  oled_command(0xD5);
  oled_command(0x80);

  oled_command(0xA8);
  oled_command(0x3F);

  oled_command(0xD3);
  oled_command(0x00);

  oled_command(0x40);

  oled_command(0x8D);
  oled_command(0x14);

  oled_command(0x20); // Memory mode
  oled_command(0x0);

  oled_command(0xA1); // Set segment remap

  oled_command(0xC8); // Scan dir

  oled_command(0xDA);
  oled_command(0x12);

  oled_command(0x81);
  oled_command(0xCF);

  oled_command(0xD9); // Precharge 
  oled_command(0xF1); // Internal

  oled_command(0xDB); // VCOMH 
  oled_command(0x40);

  oled_command(0xA4); // Displ on/off

  oled_command(0xA6); // Display normal

    // Clear screen  

  // Display on 
  oled_command(0xAF);

}



void oled_command(uint8_t cmd){
  uint16_t data[1];    
  data[0] = (0x0000 | cmd);
  HAL_SPI_Transmit(&hspi1, (uint8_t*)(data), 1, 1000);

}

void oled_data(uint8_t dat){
  uint16_t data[1];    
  data[0] = (0x0100 | dat);
  HAL_SPI_Transmit(&hspi1, (uint8_t*)(data), 1, 1000);

}

void oled_reset(){
    HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_SET);
}

void oled_clear(){
    memset(oled_buffer, 0x00, 1024);
}

unsigned char reverse(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

void oled_show_buffer(){
    oled_command(0x21);
    oled_command(0x00);
    oled_command(127);

    oled_command(0x22);
    oled_command(0x00);
    oled_command(63);

    for(int i=1023; i>=0; i--){
      oled_data(reverse(oled_buffer[i]));
    }
}

void oled_draw(int16_t xMove, int16_t yMove, char* text) {
    oled_draw_string(10, 5, text, strlen(text), dialog);
}

void oled_draw_string(int16_t xMove, int16_t yMove, char* text, uint16_t textLength, const char *data) {
  uint16_t textWidth       = data[WIDTH_POS];
  uint8_t textHeight       = data[HEIGHT_POS];
  uint8_t firstChar        = data[FIRST_CHAR_POS];
  uint16_t sizeOfJumpTable = data[CHAR_NUM_POS]  * JUMPTABLE_BYTES;

  uint8_t cursorX         = 0;
  uint8_t cursorY         = 0;

  
  xMove -= textWidth >> 1; // divide by 2
  
  // Don't draw anything if it is not on the screen.
  if (xMove + textWidth  < 0 || xMove > DISPLAY_WIDTH ) {return;}
  if (yMove + textHeight < 0 || yMove > DISPLAY_HEIGHT) {return;}

  for (uint16_t j = 0; j < textLength; j++) {
    int16_t xPos = xMove + cursorX;
    int16_t yPos = yMove + cursorY;

    char code = text[j];
    if (code >= firstChar) {
      char charCode = code - firstChar;

      // 4 Bytes per char code
      char msbJumpToChar    = data[JUMPTABLE_START + charCode * JUMPTABLE_BYTES ];                  // MSB  \ JumpAddress
      char lsbJumpToChar    = data[JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_LSB];   // LSB /
      char charByteSize     = data[JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_SIZE];  // Size
      char currentCharWidth = data[JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_WIDTH]; // Width

      // Test if the char is drawable
      if (!(msbJumpToChar == 255 && lsbJumpToChar == 255)) {
        // Get the position of the char data
        uint16_t charDataPosition = JUMPTABLE_START + sizeOfJumpTable + ((msbJumpToChar << 8) + lsbJumpToChar);
        oled_draw_internal(xPos, yPos, currentCharWidth, textHeight, data, charDataPosition, charByteSize);
      }

      cursorX += currentCharWidth;
    }
  }
}




void inline oled_draw_internal(int16_t xMove, int16_t yMove, int16_t width, int16_t height, const char *data, uint16_t offset, uint16_t bytesInData) {
  if (width < 0 || height < 0) return;
  if (yMove + height < 0 || yMove > DISPLAY_HEIGHT)  return;
  if (xMove + width  < 0 || xMove > DISPLAY_WIDTH)   return;

  uint8_t  rasterHeight = 1 + ((height - 1) >> 3); // fast ceil(height / 8.0)
  int8_t   yOffset      = yMove & 7;

  bytesInData = bytesInData == 0 ? width * rasterHeight : bytesInData;

  int16_t initYMove   = yMove;
  int8_t  initYOffset = yOffset;


  for (uint16_t i = 0; i < bytesInData; i++) {

    // Reset if next horizontal drawing phase is started.
    if ( i % rasterHeight == 0) {
      yMove   = initYMove;
      yOffset = initYOffset;
    }

    char currentByte = data[offset + i];

    int16_t xPos = xMove + (i / rasterHeight);
    int16_t yPos = ((yMove >> 3) + (i % rasterHeight)) * DISPLAY_WIDTH;

    //int16_t yScreenPos = yMove + yOffset;
    int16_t dataPos    = xPos  + yPos;

    if (dataPos >=  0  && dataPos < DISPLAY_BUFFER_SIZE &&
        xPos    >=  0  && xPos    < DISPLAY_WIDTH ) {

      if (yOffset >= 0) {
        oled_buffer[dataPos] |= currentByte << yOffset;
        if (dataPos < (DISPLAY_BUFFER_SIZE - DISPLAY_WIDTH)) {
          oled_buffer[dataPos + DISPLAY_WIDTH] |= currentByte >> (8 - yOffset);
        }
      } else {
        // Make new offset position
        yOffset = -yOffset;

        oled_buffer[dataPos] |= currentByte >> yOffset;
        // Prepare for next iteration by moving one block up
        yMove -= 8;

        // and setting the new yOffset
        yOffset = 8 - yOffset;
      }
    }
  }
}


void oled_seconds_to_minutes(int s, char *buf, int len){
    //int hours = 0;    
    int minutes = s / 60;
    int seconds = s % 60;

    strcpy(buf, "00:000:000"); 
    if(minutes > 9){
        itoa(minutes, buf+3, 10);
    }
    else{
        itoa(minutes, buf+4, 10);
    }
    if(seconds > 9){
        itoa(seconds, buf+7, 10);
    }
    else{
        itoa(seconds, buf+8, 10);
    }
}
