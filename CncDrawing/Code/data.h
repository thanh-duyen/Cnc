#ifndef DATA_H
#define DATA_H

#define ENA_PIN 2
#define DIR_X_PIN 12
#define STEP_X_PIN 26
#define DIR_Y_PIN 27
#define STEP_Y_PIN 14
#define SERVO_PIN 25
#define LIMIT_X_PIN 35
#define LIMIT_Y_PIN 34

#define DISTANCE_ID 0
#define PLOTTER_COMMAND_ID 1
#define SPEED_ID 2

#include "FS.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include "SD.h"
#include "CNC_Drawing.h"
#include "EepromHandle.h"

#define CALIBRATION_FILE "/calibrationData"
#define MAX_KEY 20

TFT_eSPI tft = TFT_eSPI();
TFT_eSPI_Button key[MAX_KEY];

void clear_key(){
  for(uint8_t i = 0; i < MAX_KEY; i++){
    key[i].setLabelDatum(0,0,MC_DATUM);
    key[i].initButton(&tft, 0,0,0,0,TFT_RED, TFT_BLUE, TFT_RED,"", 1);
  }
}
uint8_t find_key(int16_t id){
  for(uint8_t i = 0; i < MAX_KEY; i++){
    if(key[i].getID() == id){
      return i;
    }
  }
}

bool is_SD_ok = true;
char file_select[25];
File file;

typedef enum eRunProcess{
  emPausing,
  emRunning,
  emFinished
};
typedef enum eFileProcess{
  emReading,
  emDone
};
typedef enum ePageDisplay{
  emHome = 0,
  emSelectFile,
  emSetting,
  em2ndSetting
};

#define PARAMETER_MAX 15
float parameter_buffer[PARAMETER_MAX];


float cnc_maxSpeed = 15; // mm/s
int8_t speed_percent = 50;
String plotter_command = " ";
float distance_run = 0;
uint32_t time_fresh_coor;
int8_t last_cnc_status = -1;

ePageDisplay current_page = emHome;
eFileProcess file_process = emDone;
eRunProcess run_process = emFinished;

float scale_plt_file = 40;
bool go_originWhenStart = false;
bool is_goOrigin = false;
bool is_outOfLength = false;
bool is_printWarning = false;
float max_x, max_y;
bool is_addCommandGoX0Y0 = false;
const char * parameter_name[30] = {"Step/mmX: %.2f","Step/mmY: %.2f","Step/roundX: %.0f","Step/roundY: %.0f",
                                   "Accelerate: %.2fmm/s2", "MaxSpeed: %.2fmm/s","ServoUp: %.0f","ServoDown: %.0f",
                                   "DirMask(XY): %d%d","LimMask(XY): %d%d","LengthX: %.2fmm","WidthY: %.2fmm",
                                   "ScalePltFile: %.2f","GoOriginWhenStart: %.0f","GO ORIGIN"};
char keyLabel[80][5] =     {"Key", "Del", "Done", "Exit", "1", "2", "3", "+", "4", "5", "6", "-", "7", "8", "9", "x", ".", "0", "=", "/",
                            "Key", "Del", "Done", "Exit", "", "abc", "def", "", "ghi", "jkl", "mno", "", "pqrs", "tuv", "wxyz", "", "", "", "", "",
                            "Key", "Del", "Done", "Exit", ";", "ABC", "DEF", "X", "GHI", "JKL", "MNO", "Y", "PQRS", "TUV", "WXYZ", "Z", "U", " ", "D", "L",
                            "Key", "Del", "Done", "Exit", "~", ":", "!", "@", "#", "$", "%", "^", "&", "(", ")", ";", ".", "?", "+-*/", "="};
uint16_t keyColor[20] = {TFT_RED, TFT_DARKCYAN, TFT_DARKGREEN, TFT_VIOLET,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE, TFT_BLUE,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE, TFT_BLUE,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE, TFT_BLUE,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE, TFT_BLUE};
#endif
