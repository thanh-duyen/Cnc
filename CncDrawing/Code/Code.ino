#include "data.h"

CNC_Drawing cnc;
void touch_handler(char *message = NULL);

void init_SD(){
  if(!SD.begin()){
    Serial.println("Card Mount Failed");
    is_SD_ok = false;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    is_SD_ok = false;
  }

  if(is_SD_ok){
    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
      Serial.println("MMC");
    } else if(cardType == CARD_SD){
      Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
      Serial.println("SDHC");
    } else {
      Serial.println("UNKNOWN");
    }
  }
}
void touch_calibrate(bool is_run){
  uint16_t calibrationData[5];
  uint8_t calDataOK = 0;
  
  if (!SPIFFS.begin()) {
    SPIFFS.format();
    SPIFFS.begin();
  }

  if (SPIFFS.exists(CALIBRATION_FILE) && !is_run) {
    File f = SPIFFS.open(CALIBRATION_FILE, "r");
    if (f) {
      if (f.readBytes((char *)calibrationData, 14) == 14)
        calDataOK = 1;
      f.close();
    }
  }
  if (calDataOK) {
    tft.setTouch(calibrationData);
  } else {
    tft.setCursor(20, 0, 2);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);  
    tft.setTextSize(1);
    tft.println("calibration run");
    tft.calibrateTouch(calibrationData, TFT_WHITE, TFT_RED, 15);
    File f = SPIFFS.open(CALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char *)calibrationData, 14);
      f.close();
    }
  }
  tft.fillScreen((0xFFFF));
}
void configCNC(){
  for(uint8_t i = 0; i < PARAMETER_MAX; i++){
    parameter_buffer[i] = getEEPROM(i*3);
  }
  cnc.set_resolution(emMotorX, parameter_buffer[0]);
  cnc.set_resolution(emMotorY, parameter_buffer[1]);
  cnc.set_spr(emMotorX, parameter_buffer[2]);
  cnc.set_spr(emMotorY, parameter_buffer[3]);
  cnc.set_acceleration(parameter_buffer[4]);
  cnc_maxSpeed = parameter_buffer[5];
  cnc.set_maxSpeed(cnc_maxSpeed*speed_percent/100.0); // mm/s
  cnc.set_penProperty(false,parameter_buffer[6]);
  cnc.set_penProperty(true,parameter_buffer[7]);
  cnc.set_mask((int)parameter_buffer[8],(int)parameter_buffer[9]);
  max_x = parameter_buffer[10];
  max_y = parameter_buffer[11];
  scale_plt_file = parameter_buffer[12];
  go_originWhenStart = parameter_buffer[13];
}
void drawKeypad(uint8_t kindKeypad = 0){
  clear_key();
  for (uint8_t row = 0; row < 5; row++) {
    for (uint8_t col = 0; col < 4; col++) {
      uint8_t b = col + row * 4;
      if(strlen(keyLabel[b+kindKeypad*20]) != 0){
        if (b < 4) tft.setFreeFont(&FreeSansOblique12pt7b);
        else tft.setFreeFont(&FreeSansBold12pt7b);
        key[b].initButton(&tft, 40 + col * (74 + 6), 135 + row * (70 + 6), 74, 60, 
                          TFT_WHITE, keyColor[b], TFT_WHITE, keyLabel[b+kindKeypad*20], 1);
      }
      else{
        key[b].initButton(&tft, 40 + col * (74 + 6), 135 + row * (70 + 6), 74, 60, 
                          TFT_DARKGREY, TFT_DARKGREY, TFT_DARKGREY, "", 1);
      }
      key[b].drawButton();
    }
  }
}
String textInput(String keypad, String message = ""){// 0 = number, 1 = low, 2 = high, 3 = specal char
  tft.fillRect(0, 0, 320, 480, TFT_DARKGREY);
  tft.fillRect(0, 10, 320, 90, TFT_BLACK);
  tft.drawRect(0, 10, 320, 90, TFT_WHITE);
  uint8_t currentKeyPostion = 0;
  drawKeypad((int)keypad.charAt(0)-48);
  uint8_t posLastPress = 200, posCharBefore = 0;
  uint16_t t_x = 0, t_y = 0;
  uint32_t timeLastPress = millis();
  const uint8_t NUM_LEN = 40;
  char numberBuffer[NUM_LEN + 1] = "";
  uint8_t numberIndex = 0;
  tft.setTextDatum(TC_DATUM);
  tft.setFreeFont(&FreeSansOblique12pt7b);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(message, 160, 10 + 12);
  while(1){
    TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed=1;
    TIMERG0.wdt_wprotect=0;
    uint16_t touch_X, touch_Y;
    bool pressed = tft.getTouch(&touch_X, &touch_Y);
    for (uint8_t b = 0; b < 20; b++) {
      if (pressed && key[b].contains(touch_X, touch_Y)) {
        key[b].press(true);
      } else {
        key[b].press(false);
      }
    }
  
    for (uint8_t b = 0; b < 20; b++){
      if (b < 4) tft.setFreeFont(&FreeSansOblique12pt7b);
      else tft.setFreeFont(&FreeSansBold12pt7b);
      if (key[b].justReleased()){
        key[b].drawButton();
        
        if (strcmp(key[b].getLabel(),"Done") == 0) {
          key[b].press(false);
          return String(numberBuffer);
        }
        else if (strcmp(key[b].getLabel(),"Exit") == 0) {
          key[b].press(false);
          return "";
        }
        else if (strcmp(key[b].getLabel(),"Del") == 0) {
          numberBuffer[numberIndex] = 0;
          if (numberIndex > 0) {
            numberIndex--;
            numberBuffer[numberIndex] = 0;
          }
          tft.setTextDatum(TL_DATUM);
          tft.setFreeFont(&FreeSans12pt7b);
          tft.setTextColor(TFT_CYAN);
          int xwidth = tft.drawString(numberBuffer, 0 + 4, 10 + 12 + 40);
          tft.fillRect(0 + 4 + xwidth, 10 + 1 + 40, 320 - xwidth - 5, 50 - 2, TFT_BLACK);
        }
        else if (strcmp(key[b].getLabel(),"Key") == 0) {
          currentKeyPostion++;
          if(currentKeyPostion > keypad.length()-1)
            currentKeyPostion = 0;
          drawKeypad((int)keypad.charAt(currentKeyPostion)-48);
        }
      }
  
      else if (key[b].justPressed()) {
        key[b].drawButton(true);
        if(b >= 4) {
          if(posLastPress == b && millis() - timeLastPress < 500 && numberIndex < NUM_LEN+1 && 
          strlen(keyLabel[b+((int)keypad.charAt(currentKeyPostion)-48)*20]) > 1){
            numberIndex--;
            numberBuffer[numberIndex] = 0;

            tft.setTextDatum(TL_DATUM);
            tft.setFreeFont(&FreeSans12pt7b);
            tft.setTextColor(TFT_CYAN);
            int xwidth = tft.drawString(numberBuffer, 0 + 4, 10 + 12 + 40);
            tft.fillRect(0 + 4 + xwidth, 10 + 1 + 40, 320 - xwidth - 5, 50 - 2, TFT_BLACK);
            
            posCharBefore++;
            if(posCharBefore >= strlen(keyLabel[b+((int)keypad.charAt(currentKeyPostion)-48)*20])){
              posCharBefore = 0;
            }
            numberBuffer[numberIndex] = keyLabel[b+((int)keypad.charAt(currentKeyPostion)-48)*20][posCharBefore];
          }
          else if(numberIndex < NUM_LEN){
            numberBuffer[numberIndex] = keyLabel[b+((int)keypad.charAt(currentKeyPostion)-48)*20][0];
          }
          numberIndex++;
          numberBuffer[numberIndex] = 0;

          tft.setTextDatum(TL_DATUM);
          tft.setFreeFont(&FreeSans12pt7b);
          tft.setTextColor(TFT_CYAN);
          int xwidth = tft.drawString(numberBuffer, 0 + 4, 10 + 12 + 40);
          tft.fillRect(0 + 4 + xwidth, 10 + 1 + 40, 320 - xwidth - 5, 50 - 2, TFT_BLACK);
        }
        if(posLastPress != b){
          posCharBefore = 0;
        }
        posLastPress = b;
        timeLastPress = millis();
        delay(5);
      }
    }
  }
}
void handlePlotter(String plt_command){
  uint8_t length = plt_command.length();
  float targetX = NONE_VALUE, targetY = NONE_VALUE;
  eCommand command = emUnchange;
  String available_commands[4] = {"PU","PD","PL","PA"};
  String command_str = "";
  for(uint8_t i = 0; i < 4; i++){
    if(plt_command.indexOf(available_commands[i]) == 0){
      command_str = available_commands[i];
      break;
    }
  }
  if(plt_command.length() > 3){
    char ch;
    uint8_t i = 0;
    String buff = "";
    plt_command.replace(command_str,"");
    ch = plt_command.charAt(i);
    while((int(ch) - 48 >= 0 && int(ch) - 48 <= 9) || ch == '.' || ch == '-') {
      buff += ch;
      i++;
      if(i >= plt_command.length())
        break;
      ch = plt_command.charAt(i);
    }
    buff += ch;
    targetX = buff.toFloat();
    plt_command.replace(buff,"");
    targetY = plt_command.toFloat();
  }
  if(command_str == "PU") command = emUp;
  else if(command_str == "PD") command = emDown;
  else if(command_str == "PA") command = emAbsolute;
  else if(command_str == "PL") command = emRelative;
  if(command_str != ""){
    if(targetX != NONE_VALUE || targetY != NONE_VALUE || command != emUnchange){
      if(targetX != NONE_VALUE)
        targetX = targetX*1.0/scale_plt_file;
      if(targetY != NONE_VALUE)
        targetY = targetY*1.0/scale_plt_file;
      if(run_process != emFinished && (targetX >= max_x || targetY >= max_y)){
        if(is_printWarning == false){
          tft.setTextDatum(TC_DATUM);
          tft.setFreeFont(&FreeSansOblique12pt7b);
          tft.setTextColor(TFT_RED, TFT_DARKGREY);
          tft.drawString("OUT RANGE !!!", 160, 430);
          tft.drawString("Press RUN to ignore", 160, 455);
          if(run_process == emRunning){
            touch_handler("PAUSE");
          }
          is_printWarning = true;
        }
      }
      cnc.prepareRunToBuffer(targetX, targetY, command, length);
    }
  }
  else{
    cnc.add_length(length);
  }
}
void displayForScreen(){
  tft.fillRect(0, 0, 320, 480, TFT_DARKGREY);
  if(run_process != emFinished){
    clear_key();
    char ch[20];
    tft.setFreeFont(&FreeSansOblique12pt7b);
    if(run_process == emRunning)
      key[0].initButton(&tft, 160,90,314,170,TFT_WHITE, TFT_BLUE, TFT_WHITE, "PAUSE", 1);
    else{
      key[0].initButton(&tft, 80,90,154,170,TFT_WHITE, TFT_RED, TFT_WHITE,"STOP", 1);
      key[1].initButton(&tft, 240,90,154,170,TFT_WHITE, TFT_BLUE, TFT_WHITE,"RUN", 1);
    }
    key[2].initButton(&tft, 160,390,314,50,TFT_WHITE, TFT_BLUE, TFT_WHITE, file_select, 1);
    sprintf(ch, "Speed:%d%%", speed_percent);
    key[3].initButton(&tft, 40,270,74,50,TFT_WHITE, TFT_VIOLET, TFT_WHITE,"S-", 1);
    key[4].initButton(&tft, 160,270,154,50,TFT_WHITE, TFT_VIOLET, TFT_WHITE, ch, 1, SPEED_ID);
    key[5].initButton(&tft, 280,270,74,50,TFT_WHITE, TFT_VIOLET, TFT_WHITE,"S+", 1);
    tft.setTextDatum(TL_DATUM);
    tft.setFreeFont(&FreeSansOblique12pt7b);
    tft.setTextColor(TFT_BLACK, TFT_DARKGREY);
    sprintf(ch, "X: %04d.%02d  ", (int)cnc.get_postion(emMotorX), int(abs(cnc.get_postion(emMotorX)*100))%100);
    tft.drawString(ch, 0, 200);
    sprintf(ch, "Y: %04d.%02d  ", (int)cnc.get_postion(emMotorY), int(abs(cnc.get_postion(emMotorY)*100))%100);
    tft.drawString(ch, 160, 200);
    tft.setTextDatum(MC_DATUM);
    sprintf(ch, "Progress: %d%%", cnc.get_progress());
    tft.drawString(ch, 160, 330);
    for(uint8_t i = 0; i < MAX_KEY; i++){
      if(strcmp(key[i].getLabel(),"") != 0){
        key[i].drawButton();
      }
    }
  }
  else if(current_page == emHome){
    clear_key();
    char ch[20];
    tft.setFreeFont(&FreeSansOblique12pt7b);
    key[0].initButton(&tft, 120,30,74,50,TFT_WHITE, TFT_BLUE, TFT_WHITE,"Y+", 1);
    key[1].initButton(&tft, 40,90,74,50,TFT_WHITE, TFT_BLUE, TFT_WHITE,"X-", 1);
    key[2].initButton(&tft, 200,90,74,50,TFT_WHITE, TFT_BLUE, TFT_WHITE,"X+", 1);
    key[14].initButton(&tft, 120,90,74,50,TFT_WHITE, TFT_BLUE, TFT_WHITE,"STOP", 1);
    key[3].initButton(&tft, 120,150,74,50,TFT_WHITE, TFT_BLUE, TFT_WHITE,"Y-", 1);
    key[4].initButton(&tft, 280,30,74,50,TFT_WHITE, TFT_BLUE, TFT_WHITE,"Z+", 1);
    key[11].initButton(&tft, 280,150,74,50,TFT_WHITE, TFT_BLUE, TFT_WHITE,"Z-", 1);
    key[5].initButton(&tft, 80,450,154,50,TFT_WHITE, TFT_BLUE, TFT_WHITE,"FILE", 1);
    key[6].initButton(&tft, 240,450,154,50,TFT_WHITE, TFT_BLUE, TFT_WHITE,"SETTING", 1);
    key[7].initButton(&tft, 40,330,74,50,TFT_WHITE, TFT_BROWN, TFT_WHITE,"Home", 1);
    key[8].initButton(&tft, 120,330,74,50,TFT_WHITE, TFT_BROWN, TFT_WHITE,"Zero", 1);
    sprintf(ch, "%.2fmm", distance_run);
    key[9].initButton(&tft, 240,330,154,50,TFT_WHITE, TFT_BLUE, TFT_WHITE,ch, 1, DISTANCE_ID);
    sprintf(ch, "%s", plotter_command);
    key[10].initButton(&tft, 120,390,234,50,TFT_WHITE, TFT_MAGENTA, TFT_WHITE,ch, 1, PLOTTER_COMMAND_ID);
    key[12].initButton(&tft, 280,390,74,50,TFT_WHITE, TFT_MAGENTA, TFT_WHITE,"Exec", 1);
    sprintf(ch, "%d%%", speed_percent);
    key[15].initButton(&tft, 40,270,74,50,TFT_WHITE, TFT_VIOLET, TFT_WHITE,"S-", 1);
    key[13].initButton(&tft, 160,270,154,50,TFT_WHITE, TFT_VIOLET, TFT_WHITE, ch, 1, SPEED_ID);
    key[16].initButton(&tft, 280,270,74,50,TFT_WHITE, TFT_VIOLET, TFT_WHITE,"S+", 1);
    for(uint8_t i = 0; i < MAX_KEY; i++){
      if(strcmp(key[i].getLabel(),"") != 0){
        key[i].drawButton();
      }
    }
    tft.setTextDatum(TL_DATUM);
    tft.setFreeFont(&FreeSansOblique12pt7b);
    tft.setTextColor(TFT_BLACK, TFT_DARKGREY);
    sprintf(ch, "X: %04d.%02d  ", (int)cnc.get_postion(emMotorX), int(abs(cnc.get_postion(emMotorX)*100))%100);
    tft.drawString(ch, 0, 200);
    sprintf(ch, "Y: %04d.%02d  ", (int)cnc.get_postion(emMotorY), int(abs(cnc.get_postion(emMotorY)*100))%100);
    tft.drawString(ch, 160, 200);
    last_cnc_status = -1;
  }
  else if(current_page == emSelectFile){
    clear_key();
    tft.setFreeFont(&FreeSansOblique12pt7b);
    key[18].initButton(&tft, 80,450,154,50,TFT_WHITE, TFT_BLUE, TFT_WHITE,"RELOAD", 1);
    key[18].drawButton();
    key[19].initButton(&tft, 240,450,154,50,TFT_WHITE, TFT_BLUE, TFT_WHITE,"BACK", 1);
    key[19].drawButton();
    File root = SD.open("/");
    if(!root){
      is_SD_ok = false;
    }
    else{
      uint8_t index = 0;
      File file = root.openNextFile();
      while(file && index < 10){
        if(!file.isDirectory()){
          key[index].setLabelDatum(-150,0,ML_DATUM);
          key[index].initButton(&tft, 160,index*40+20,314,38,TFT_BLACK, TFT_DARKGREY, TFT_BLACK, (char*)file.name(), 1);
          key[index].drawButton();
          index++;
        }
        file = root.openNextFile();
      }
    }
  }
  else if(current_page == emSetting){
    clear_key();
    tft.setFreeFont(&FreeSansOblique12pt7b);
    key[17].initButton(&tft, 80,450,154,50,TFT_WHITE, TFT_BLUE, TFT_WHITE,"SAVE", 1);
    key[17].drawButton();
    key[18].initButton(&tft, 200,450,74,50,TFT_WHITE, TFT_BLUE, TFT_WHITE,"BACK", 1);
    key[18].drawButton();
    key[19].initButton(&tft, 280,450,74,50,TFT_WHITE, TFT_BLUE, TFT_WHITE,">>>", 1);
    key[19].drawButton();
    
    for(uint8_t i = 0; i < 10; i++){
      char ch[25];
      if(i != 8 && i != 9)
        sprintf(ch,parameter_name[i],parameter_buffer[i]);
      else
        sprintf(ch,parameter_name[i],(int)parameter_buffer[i]&0x1,((int)parameter_buffer[i]>>1)&0x1);
      key[i].setLabelDatum(-150,0,ML_DATUM);
      key[i].initButton(&tft, 160,i*40+20,314,38,TFT_BLACK, TFT_DARKGREY, TFT_BLACK, ch, 1);
      key[i].drawButton();
    }
  }
  else if(current_page == em2ndSetting){
    clear_key();
    tft.setFreeFont(&FreeSansOblique12pt7b);
    key[17].initButton(&tft, 80,450,154,50,TFT_WHITE, TFT_BLUE, TFT_WHITE,"SAVE", 1);
    key[17].drawButton();
    key[18].initButton(&tft, 200,450,74,50,TFT_WHITE, TFT_BLUE, TFT_WHITE,"BACK", 1);
    key[18].drawButton();
    key[19].initButton(&tft, 280,450,74,50,TFT_WHITE, TFT_BLUE, TFT_WHITE,"<<<", 1);
    key[19].drawButton();
    
    for(uint8_t i = 10; i < PARAMETER_MAX; i++){
      char ch[25];
      if(parameter_name[i] != "GO ORIGIN")
        sprintf(ch,parameter_name[i],parameter_buffer[i]);
      else
        strcpy(ch,parameter_name[i]);
      key[i-10].setLabelDatum(-150,0,ML_DATUM);
      key[i-10].initButton(&tft, 160,(i-10)*40+20,314,38,TFT_BLACK, TFT_DARKGREY, TFT_BLACK, ch, 1);
      key[i-10].drawButton();
    }
  }
}
void touch_handler(char *message){
  if(message != NULL){
    for (uint8_t b = 0; b < 20; b++){
      if(strcmp(key[b].getLabel(),message) == 0){
        key[b].press(false);
        break;
      }
    }
  }
  else{
    uint16_t touch_X, touch_Y;
    bool pressed = tft.getTouch(&touch_X, &touch_Y);
    for (uint8_t b = 0; b < 20; b++){
      if (pressed && key[b].contains(touch_X, touch_Y)){
        key[b].press(true);
      } else {
        key[b].press(false);
      }
    }
  }
  for (uint8_t b = 0; b < 20; b++){
    if (key[b].justReleased()){
      tft.setFreeFont(&FreeSansOblique12pt7b);
      key[b].drawButton();
      if(key[b].getID() == DISTANCE_ID){
        String str = textInput("0","Enter number");
        if(str != ""){
          distance_run = str.toFloat();
        }
        displayForScreen();
      }
      else if(key[b].getID() == PLOTTER_COMMAND_ID){
        String str = textInput("02","Enter Plotter command");
        if(str != ""){
          plotter_command = str;
        }
        displayForScreen();
      }
      else if(key[b].getID() == SPEED_ID){
        String str = textInput("0","Enter speed(1-100%)");
        if(str != ""){
          if(str.toInt() > 100) speed_percent = 100;
          else if(str.toInt() < 1) speed_percent = 1;
          else speed_percent = str.toInt();
          cnc.set_maxSpeed(cnc_maxSpeed*speed_percent/100.0); // mm/s
        }
        displayForScreen();
      }
      else if(strcmp(key[b].getLabel(),"S+") == 0){
        if(speed_percent < 100)
          speed_percent++;
        char ch[25];
        sprintf(ch, "%d%%", speed_percent);
        key[find_key(SPEED_ID)].initButton(&tft, 160,270,154,50,TFT_WHITE, TFT_VIOLET, TFT_WHITE, ch, 1, SPEED_ID);
        key[find_key(SPEED_ID)].drawButton();
        cnc.set_maxSpeed(cnc_maxSpeed*speed_percent/100.0); // mm/s
      }
      else if(strcmp(key[b].getLabel(),"S-") == 0){
        if(speed_percent > 1)
          speed_percent--;
        char ch[25];
        sprintf(ch, "%d%%", speed_percent);
        key[find_key(SPEED_ID)].initButton(&tft, 160,270,154,50,TFT_WHITE, TFT_VIOLET, TFT_WHITE, ch, 1, SPEED_ID);
        key[find_key(SPEED_ID)].drawButton();
        cnc.set_maxSpeed(cnc_maxSpeed*speed_percent/100.0); // mm/s
      }
      else if(strcmp(key[b].getLabel(),"Home") == 0){
        cnc.prepareRunToBuffer(0,0,emUp);
      }
      else if(strcmp(key[b].getLabel(),"Zero") == 0){
        cnc.set_coor(emMotorX,0);
        cnc.set_coor(emMotorY,0);
      }
      else if(strcmp(key[b].getLabel(),"Exec") == 0){
        handlePlotter(plotter_command);
      }
      else if(strcmp(key[b].getLabel(),"FILE") == 0){
        current_page = emSelectFile;
        displayForScreen();
      }
      else if(strcmp(key[b].getLabel(),"SETTING") == 0){
        for(uint8_t i = 0; i < PARAMETER_MAX; i++){
          parameter_buffer[i] = getEEPROM(i*3);
        }
        current_page = emSetting;
        displayForScreen();
      }
      else if(strcmp(key[b].getLabel(),"RELOAD") == 0){
        init_SD();
        displayForScreen();
      }
      else if(strcmp(key[b].getLabel(),"BACK") == 0){
        current_page = emHome;
        displayForScreen();
      }
      else if(strcmp(key[b].getLabel(),"SAVE") == 0){
        for(uint8_t i = 0; i < PARAMETER_MAX; i++){
           saveEEPROM(i*3,parameter_buffer[i]);
        }
        configCNC();
        current_page = emHome;
        displayForScreen();
      }
      else if(strcmp(key[b].getLabel(),">>>") == 0){
        current_page = em2ndSetting;
        displayForScreen();
      }
      else if(strcmp(key[b].getLabel(),"<<<") == 0){
        current_page = emSetting;
        displayForScreen();
      }
      else if(strcmp(key[b].getLabel(),"STOP") == 0){
        if(run_process != emFinished){
          cnc.clear_buffer();
          cnc.clear_pause();
          if(file)
            file.close();
          run_process = emFinished;
          file_process = emDone;
          displayForScreen();
        }
        else{
          cnc.stop(true);
          is_goOrigin = false;
        }
        last_cnc_status = -1;
      }
      else if(strcmp(key[b].getLabel(),"Z+") == 0){
        cnc.pen_control(false);
      }
      else if(strcmp(key[b].getLabel(),"Z-") == 0){
        cnc.pen_control(true);
      }
      else if(strcmp(key[b].getLabel(),"Y+") == 0){
        String distance = String(key[find_key(DISTANCE_ID)].getLabel());
        if(distance.toFloat() != 0)
          cnc.prepareRunToBuffer(NONE_VALUE,cnc.get_postion(emMotorY)+distance.toFloat(), emUnchange);
        else
          cnc.stop(true);
      }
      else if(strcmp(key[b].getLabel(),"Y-") == 0){
        String distance = String(key[find_key(DISTANCE_ID)].getLabel());
        if(distance.toFloat() != 0)
          cnc.prepareRunToBuffer(NONE_VALUE,cnc.get_postion(emMotorY)-distance.toFloat(), emUnchange);
        else
          cnc.stop(true);
      }
      else if(strcmp(key[b].getLabel(),"X+") == 0){
        String distance = String(key[find_key(DISTANCE_ID)].getLabel());
        if(distance.toFloat() != 0)
          cnc.prepareRunToBuffer(cnc.get_postion(emMotorX)+distance.toFloat(),NONE_VALUE, emUnchange);
        else
          cnc.stop(true);
      }
      else if(strcmp(key[b].getLabel(),"X-") == 0){
        String distance = String(key[find_key(DISTANCE_ID)].getLabel());
        if(distance.toFloat() != 0)
          cnc.prepareRunToBuffer(cnc.get_postion(emMotorX)-distance.toFloat(),NONE_VALUE, emUnchange);
        else
          cnc.stop(true);
      }
      else if(strcmp(key[b].getLabel(),"RUN") == 0){
        tft.fillRect(0,0,320,180,TFT_DARKGREY);
        key[0].initButton(&tft, 160,90,314,170,TFT_WHITE, TFT_BLUE, TFT_WHITE, "PAUSE", 1);
        key[1].initButton(&tft, 0,0,0,0,TFT_WHITE, TFT_BLUE, TFT_WHITE,"", 1);
        key[0].drawButton();
        is_addCommandGoX0Y0 = false;
        run_process = emRunning;
        cnc.clear_pause();
      }
      else if(strcmp(key[b].getLabel(),"PAUSE") == 0){
        tft.fillRect(0,0,320,180,TFT_DARKGREY);
        key[0].initButton(&tft, 80,90,154,170,TFT_WHITE, TFT_RED, TFT_WHITE,"STOP", 1);
        key[1].initButton(&tft, 240,90,154,170,TFT_WHITE, TFT_BLUE, TFT_WHITE,"RUN", 1);
        key[0].drawButton();
        key[1].drawButton();
        run_process = emPausing;
        cnc.stop(true);
        cnc.set_pause();
        cnc.set_saveTargetPostion(true);
      }
      else if(strcmp(key[b].getLabel(),"GO ORIGIN") == 0){
        current_page = emHome;
        displayForScreen();
        is_goOrigin = true;
        cnc.prepareRunToBuffer(-5000,-5000, emUp);
      }
      else if(current_page == emSetting){
        String str = textInput("0","");
        if(str != ""){
          if(b >= 8)
            parameter_buffer[b] = ((int)str.charAt(0)-48) | (((int)str.charAt(1)-48) << 1);
          else
            parameter_buffer[b] = str.toFloat();
        }
        displayForScreen();
      }
      else if(current_page == em2ndSetting){
        String str = textInput("0","");
        if(str != "")
          parameter_buffer[b+10] = str.toFloat();
        displayForScreen();
      }
      else if(current_page == emSelectFile){
        String str = key[b].getLabel();
        str.toCharArray(file_select,str.length()+1);
        file = SD.open(file_select);
        if(!file){
          Serial.println("Cannot open file " + String(file_select));
        }
        else{
          cnc.set_length(file.size());
          file_process = emReading;
          run_process = emPausing;
          cnc.set_pause();
        }
        current_page = emHome;
        displayForScreen();
      }
    }
    else if (key[b].justPressed()){
      tft.setFreeFont(&FreeSansOblique12pt7b);
      key[b].drawButton(true);
      
      if(strcmp(key[b].getLabel(),"Y+") == 0){
        String distance = String(key[find_key(DISTANCE_ID)].getLabel());
        if(distance.toFloat() == 0)
          cnc.prepareRunToBuffer(NONE_VALUE,cnc.get_postion(emMotorY)+9999, emUnchange);
      }
      else if(strcmp(key[b].getLabel(),"Y-") == 0){
        String distance = String(key[find_key(DISTANCE_ID)].getLabel());
        if(distance.toFloat() == 0)
          cnc.prepareRunToBuffer(NONE_VALUE,cnc.get_postion(emMotorY)-9999, emUnchange);
      }
      else if(strcmp(key[b].getLabel(),"X+") == 0){
        String distance = String(key[find_key(DISTANCE_ID)].getLabel());
        if(distance.toFloat() == 0)
          cnc.prepareRunToBuffer(cnc.get_postion(emMotorY)+9999,NONE_VALUE, emUnchange);
      }
      else if(strcmp(key[b].getLabel(),"X-") == 0){
        String distance = String(key[find_key(DISTANCE_ID)].getLabel());
        if(distance.toFloat() == 0)
          cnc.prepareRunToBuffer(cnc.get_postion(emMotorY)-9999,NONE_VALUE, emUnchange);
      } 
    }
  }
}
void setup() {
  Serial.begin (115200);
  EEPROM.begin(100);
  delay(50);
  pinMode(ENA_PIN,OUTPUT);
  digitalWrite(ENA_PIN,LOW);
  tft.init();
  tft.setRotation(0);
  touch_calibrate(false);
  
  init_SD();
  checkEEPROM();
  
  cnc.init();
  configCNC();
  
  displayForScreen();
  if(go_originWhenStart){
    is_goOrigin = true;
    cnc.prepareRunToBuffer(-5000,-5000, emUp);
  }
}
void loop() {
  touch_handler(NULL);
  if((cnc.is_running() || cnc.is_release()) && millis() >= time_fresh_coor && current_page == emHome){
    if(cnc.is_release())
      cnc.clear_release();
    tft.setTextDatum(TL_DATUM);
    tft.setFreeFont(&FreeSansOblique12pt7b);
    tft.setTextColor(TFT_BLACK, TFT_DARKGREY);
    char ch[20];
    sprintf(ch, "X: %04d.%02d   ", (int)cnc.get_postion(emMotorX), int(abs(cnc.get_postion(emMotorX)*100))%100);
    tft.drawString(ch, 0, 200);
    sprintf(ch, "Y: %04d.%02d   ", (int)cnc.get_postion(emMotorY), int(abs(cnc.get_postion(emMotorY)*100))%100);
    tft.drawString(ch, 160, 200);
    if(run_process == emRunning){
      tft.setTextDatum(MC_DATUM);
      sprintf(ch, "Progress: %d%%", cnc.get_progress());
      tft.drawString(ch, 160, 330);
    }
    time_fresh_coor = millis() + 1000;
  }
  if(last_cnc_status != cnc.is_running() && run_process == emFinished && current_page == emHome){
    last_cnc_status = cnc.is_running();
    tft.setFreeFont(&FreeSansOblique12pt7b);
    if(last_cnc_status == 1){
      key[14].initButton(&tft, 120,90,74,50,TFT_WHITE, TFT_RED, TFT_WHITE,"STOP", 1);
      key[14].drawButton();
    }
    else{
      key[14].initButton(&tft, 120,90,74,50,TFT_WHITE, TFT_BLUE, TFT_WHITE,"STOP", 1);
      key[14].drawButton();
    }
  }
  if(is_goOrigin){
    if(cnc.get_motorReachLimit() == emBothMotor){
      cnc.stop(false);
      is_goOrigin = false;
    }
  }
  if(file_process == emReading){
    if(cnc.checkBuffer() >= 0){
      if(file.available()){
        String str = "";
        char ch = file.read();
        int8_t pos = 0;
        while(ch != '\n' && ++pos < 40){
          str += (char) ch;
          ch = file.read();
        }
        str += (char) ch; // add \n to str
        handlePlotter(str);
      }
      else{
        file.close();
        file_process = emDone;
      }
    }
  }
  else if(!cnc.is_running() && run_process == emRunning){
    if((cnc.get_postion(emMotorY) != 0 || cnc.get_postion(emMotorX) != 0) && !is_addCommandGoX0Y0){
      handlePlotter("PU0 0;");
      is_addCommandGoX0Y0 = true;
    }
    else if(is_addCommandGoX0Y0){
      is_printWarning = false;
      run_process = emFinished;
      displayForScreen();
    }
  }
  TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
  TIMERG0.wdt_feed=1;
  TIMERG0.wdt_wprotect=0;
}
