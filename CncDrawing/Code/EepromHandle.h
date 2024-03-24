#include <EEPROM.h>
/*
 * 
 */
void saveEEPROM(uint8_t address, float value){
  /*
   * range: - 32640.00 -> 32640.99
   */
  value += 32640;
  EEPROM.write(address + 0, value / 255);
  EEPROM.write(address + 1, (int)value % 255);
  EEPROM.write(address + 2, (int)(value * 100) % 100);
  EEPROM.commit();
}
void saveEEPROM(uint8_t address, int value){
  /*
   * range: - 32640 -> 326401
   */
  value += 32640;
  EEPROM.write(address + 0, value / 255);
  EEPROM.write(address + 1, value % 255);
  EEPROM.write(address + 2, 0);
  EEPROM.commit();
}
float getEEPROM(uint8_t address){
  float value;
  value = EEPROM.read(address) * 255;
  value += EEPROM.read(address + 1);
  value += EEPROM.read(address + 2)/100.0;
  value -= 32640;
  return value; 
}
void checkEEPROM(){
  float value[8];
  for(uint8_t i = 0; i < 8; i++){
    value[i] = getEEPROM(i*3);
  }

  /*
   * 0: step per mm X
   * 1: step per mm Y
   * 2: step per mm Z
   * 3: accel
   * 4: max speed with G0 command
   * 5: max speed with G1 command
   */
  for(uint8_t i = 0; i < 6; i++){
    if(value[i] > 999.99)
      saveEEPROM(i*3, 20);
    else if(value[i] <= 0)
      saveEEPROM(i*3, 20);
  }

  // step per round
  if(value[6] > 999)
    saveEEPROM(6*3, 400);
  else if(value[6] <= 0)
    saveEEPROM(6*3, 400);

  // distance safe
  if(value[7] >= 255)
    saveEEPROM(7*3, 5);
  else if(value[6] < 0)
    saveEEPROM(6*3, 5);

//  for(uint8_t s = 0; s < 7; s++){
//    int32_t adc_value = getEEPROM(15*3+s*3);
//    if(adc_value >= 4095){
//      saveEEPROM(15*3+s*3, 1024);
//    }
//  }
}
