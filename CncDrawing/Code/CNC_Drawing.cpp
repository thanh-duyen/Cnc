#include "CNC_Drawing.h"

Servo pen;
CNC_Drawing* CNC_Drawing::singleton;
volatile uint32_t *addUpdate = (volatile uint32_t *)0x3FF5F00C;
volatile uint32_t *addLoTimer = (volatile uint32_t *)0x3FF5F004;
volatile uint32_t *addHiTimer = (volatile uint32_t *)0x3FF5F008;

CNC_Drawing::CNC_Drawing(){
  // Do not enything
}
void IRAM_ATTR CNC_Drawing::onTimer(void) {
  portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL_ISR(&timerMux);
  
  singleton->interrup_raise();
  
  portEXIT_CRITICAL_ISR(&timerMux);
}
void IRAM_ATTR CNC_Drawing::interrup_raise(void){
  _interrupted++;
}
void CNC_Drawing::CNC_handler(void * pvParameters){
  while(1){
    singleton->run();
  }
}
uint32_t CNC_Drawing::ticksCount(void){
  *addUpdate = 0x08;
  return *addLoTimer;
}
void CNC_Drawing::setTimer(uint32_t ticks){
  _interrupted = 0;
  if(!timer){
    timer = timerBegin(0, 8, true);  //1 tick = 0.1us
    timerAttachInterrupt(timer, &CNC_Drawing::onTimer, true);
    timerAlarmWrite(timer, ticks, false);
    timerWrite(timer, 0);
    timerAlarmEnable(timer);
  }
  else{
    timerAlarmWrite(timer, ticks, false);
    timerWrite(timer, 0);
    timerAlarmEnable(timer);
  }
}
void CNC_Drawing::offTimer(void){
  if(timer){
    timerEnd(timer);
    timer = NULL;
  }
}
void CNC_Drawing::set_penProperty(bool is_down, uint8_t degree){
  _servo_degree[is_down] = degree;
  pen.write(_servo_degree[_pen_state]);
  _servo_degree[emDeltaDegree] = abs(_servo_degree[emUp] - _servo_degree[emDown]);
}
void CNC_Drawing::pen_control(bool is_down){
  pen.write(_servo_degree[is_down]);
  _pen_state = (eCommand)is_down;
}
bool CNC_Drawing::checkNextRun(void){
  if(_buffer_enable[_buffer_readPostion] >= 0){
    static int16_t count = 0;
    if(--count <= 0)
      _find_decel_proactive = false;
    if(_find_decel_proactive == false){
      _step_runUntil_decel = _buffer_maxStep[_buffer_readPostion];
      eCommand last_command = _buffer_command[_buffer_readPostion];
      int16_t p = _buffer_readPostion + 1;
      if(p >= BUFFER_SIZE_CNC) p = 0;
      count = 0;
      while(_buffer_enable[p] > 0){
        if((_buffer_angle[p] <= 150) || (last_command == emUp && _buffer_command[p] == emDown) 
        || (last_command == emDown && _buffer_command[p] == emUp)){
          _find_decel_proactive = true;
          count = BUFFER_SIZE_CNC;
          break;
        }
        else{
          _step_runUntil_decel += _buffer_maxStep[p];
          if(_run_state == emRun){
            if(_step_runUntil_decel - _buffer_maxStep[_buffer_readPostion] > _ramp_counter)
              break;
          }
        }
        if(++p >= BUFFER_SIZE_CNC)
          p = 0;
        if(++count >= BUFFER_SIZE_CNC){
          _find_decel_proactive = true;
          count = BUFFER_SIZE_CNC/2;
          break;
        }
      }
    }
    
    int8_t value = _buffer_enable[_buffer_readPostion];
    int32_t distance[emNumberOfAsix] = {0,0};
    _max_step_asix = emMotorX;
    for(uint8_t i = 0; i < emNumberOfAsix; i++){
      if((value&(1<<i)) == (1<<i)){
        _postion_moveTo[i] = _buffer_postionMoveTo[i][_buffer_readPostion];
        distance[i] = _postion_moveTo[i] - _postion_current[i];
        if(_postion_moveTo[i] > _postion_current[i]){
          _dir[i] = 1;
          digitalWrite(_dir_pin[i],!_dir_mask[i]);
        }
        else{
          _dir[i] = -1;
          digitalWrite(_dir_pin[i],_dir_mask[i]);
        }
        distance[i] = abs(distance[i]);
      }
      if(distance[i] > distance[_max_step_asix])
        _max_step_asix = (eMotor)i;
    }
    for(uint8_t i = 0; i < emNumberOfAsix; i++){
      _scale_reset[i] = 0;
      if(distance[i] == 0){
        _scale_buffer[i] = NONE_VALUE;
      }
      else{
        _scale[i] = distance[_max_step_asix]*1.0/distance[i];
        _scale_buffer[i] = _scale[i];
      }
    }
    _min_delay = ((long)(_alpha*T1_FREQ))/_maxSpeed_radps;
    if(_actual_delay == _start_delay)
      _run_state = emAccel;
    else if(_actual_delay < _min_delay)
      _run_state = emSlowDown;
    if((_buffer_command[_buffer_readPostion] == emUp && _pen_state == emDown)
    || (_buffer_command[_buffer_readPostion] == emDown && _pen_state == emUp)){
      pen_control((eCommand)_buffer_command[_buffer_readPostion]);
      delay(_servo_degree[emDeltaDegree]*3);
    }
    _buffer_ranLength += _buffer_length[_buffer_readPostion];
    _buffer_command[_buffer_readPostion] = emUnchange;
    _buffer_enable[_buffer_readPostion] = -1;
    _buffer_maxStep[_buffer_readPostion] = 0;
    _buffer_angle[_buffer_readPostion] = -1;
    if(++_buffer_readPostion >= BUFFER_SIZE_CNC)
      _buffer_readPostion = 0;
    return true;
  }
  else
    return false;
}
void CNC_Drawing::set_saveTargetPostion(bool value){
  _save_targetPostion = value;
}
void CNC_Drawing::calculatorSpeed(void){
  static unsigned int rest = 0;
  unsigned int new_step_delay;
  _step_runUntil_decel--;
  if(_run_state == emAccel){
    _ramp_counter++;
    _actual_delay = _last_delay - (((2 * (long)_last_delay) + rest)/(4 * _ramp_counter + 1));
    rest = ((2 * (long)_last_delay) + rest)%(4 * _ramp_counter + 1);
    _last_delay = _actual_delay;
    if(_actual_delay <= _min_delay){ // max speed
      _actual_delay = _min_delay;
      _run_state = emRun;
      rest = 0;
    }
    if(_ramp_counter >= _step_runUntil_decel){
      _run_state = emDecel;
      _ramp_counter = _step_runUntil_decel;
      if(_ramp_counter == 0){ // step run just 1 step
        _run_state = emStop;
        _find_decel_proactive = false;
        _actual_delay = _start_delay;
        _last_delay = _actual_delay;
        rest = 0;
      }
    }
    _ticks = _actual_delay*1.0/TIME_PER_TICK;
  }
  else if(_run_state == emRun){
    if(_ramp_counter >= _step_runUntil_decel){
      _run_state = emDecel;
    }
  }
  else if(_run_state == emSlowDown){
    _ramp_counter--;
    _actual_delay = _last_delay + (((2 * (long)_last_delay) + rest)/(4 * _ramp_counter + 1));
    rest = ((2 * (long)_last_delay)+rest)%(4 * _ramp_counter + 1);
    _last_delay = _actual_delay;
    if(_actual_delay >= _min_delay){ // max speed
      _actual_delay = _min_delay;
      _run_state = emRun;
      rest = 0;
    }
    if(_ramp_counter >= _step_runUntil_decel){
      _run_state = emDecel;
    }
    _ticks = _actual_delay*1.0/TIME_PER_TICK;
  }
  else if(_run_state == emDecel){ // decel to stop
    _ramp_counter--;
    _actual_delay = _last_delay + (((2 * (long)_last_delay) + rest)/(4 * _ramp_counter + 1));
    rest = ((2 * (long)_last_delay)+rest)%(4 * _ramp_counter + 1);
    _last_delay = _actual_delay;
    if(_ramp_counter <= 0){
      if(_save_targetPostion == true){
        if(_postion_moveTo[_max_step_asix] != _postion_current[_max_step_asix]){
          _buffer_readPostion--;
          if(_buffer_readPostion < 0)
            _buffer_readPostion = BUFFER_SIZE_CNC - 1;
          int8_t buffer = 0;
          int32_t step = 0;
          for(uint8_t i = 0; i < emNumberOfAsix; i++){
            if(_postion_moveTo[i] != _postion_current[i]){
              buffer |= 1<<i;
              int32_t temp = abs(_postion_moveTo[i] - _postion_current[i]);
              if(temp > step)
                step = temp;
              _buffer_postionMoveTo[i][_buffer_readPostion] = _postion_moveTo[i];
              _postion_moveTo[i] = _postion_current[i];
            }
          }
          _buffer_maxStep[_buffer_readPostion] = step;
          _buffer_enable[_buffer_readPostion] = buffer;
        }
      }
      else{
        _postion_moveTo[emMotorX] = _postion_current[emMotorX];
        _postion_moveTo[emMotorY] = _postion_current[emMotorY];
      }
      _run_state = emStop;
      _find_decel_proactive = false;
      _actual_delay = _start_delay;
      _last_delay = _actual_delay;
      rest = 0;
    }
    _ticks = _actual_delay*1.0/TIME_PER_TICK;
  }
}
int16_t CNC_Drawing::checkBuffer(void){ // check buffer dose overflow
  uint16_t counter = 0;
  uint16_t postion = _buffer_readPostion;
  while(1){
    if(_buffer_enable[postion] == -1){
      break;
    }
    if(++postion >= BUFFER_SIZE_CNC)
      postion = 0;
    if(++counter >= BUFFER_SIZE_CNC){
      break;
    }
  }
  
  if(counter >= BUFFER_SIZE_CNC-1) // keep 1 buffer to save postion after stop
    return -1;
  else
    return postion;
}
float CNC_Drawing::get_postion(eMotor motor){ // return the postion of asix with mm unit
  return _postion_current[motor]*1.0/_resolution[motor];
}
void CNC_Drawing::set_length(uint32_t length){
  _buffer_totalLength = length;
  _buffer_ranLength = 0;
}
int8_t CNC_Drawing::get_progress(void){
  if(_buffer_totalLength != 0){
    uint8_t progress = _buffer_ranLength*100/_buffer_totalLength;
    if(progress > 100)
      progress = 100;
    return progress;
  }
  else
    return -1;
}
void CNC_Drawing::add_length(uint8_t length){
  _buffer_ranLength += length;
}
void CNC_Drawing::prepareRunToBuffer(double pos_X, double pos_Y, eCommand command, uint8_t length){
  int16_t postion = checkBuffer();
  if(postion >= 0 && postion < BUFFER_SIZE_CNC){
    float a_point[emNumberOfAsix];
    float o_point[emNumberOfAsix];
    float b_point[emNumberOfAsix];
    
    int16_t lastPostion = postion - 1;
    if(lastPostion < 0)
      lastPostion = BUFFER_SIZE_CNC - 1;
    int16_t lastPostion_2 = lastPostion - 1;
    if(lastPostion_2 < 0)
      lastPostion_2 = BUFFER_SIZE_CNC - 1;
    
    int8_t buffer = 0;
    int32_t step = 0;
    int32_t temp = 0;
    float new_postion[emNumberOfAsix] = {pos_X,pos_Y};
    for(uint8_t i = 0; i < emNumberOfAsix; i++){
      if(new_postion[i] != NONE_VALUE){
        if(command == emRelative){
          if(_buffer_enable[lastPostion] > 0){
            _buffer_postionMoveTo[i][postion] = _buffer_postionMoveTo[i][lastPostion];
            _buffer_postionMoveTo[i][postion] += new_postion[i]*1.0*_resolution[i];
          }
          else
            _buffer_postionMoveTo[i][postion] = _postion_current[i] + new_postion[i]*1.0*_resolution[i];
        }
        else
          _buffer_postionMoveTo[i][postion] = new_postion[i]*1.0*_resolution[i];
        buffer |= 1<<i;
        if(_buffer_enable[lastPostion] > 0){
          temp = _buffer_postionMoveTo[i][lastPostion] - _buffer_postionMoveTo[i][postion];
          temp = abs(temp);
          if(temp > step)
            step = temp;
          o_point[i] = _buffer_postionMoveTo[i][lastPostion];
        }
        else{
          temp = _postion_current[i] - _buffer_postionMoveTo[i][postion];
          temp = abs(temp);
          if(temp > step)
            step = temp;
        }
      }
      else{
        if(_buffer_enable[lastPostion] > 0){
          _buffer_postionMoveTo[i][postion] = _buffer_postionMoveTo[i][lastPostion];
          o_point[i] = _buffer_postionMoveTo[i][lastPostion];
        }
        else{
          if(_running)
            _buffer_postionMoveTo[i][postion] = _postion_moveTo[i];
          else
            _buffer_postionMoveTo[i][postion] = _postion_current[i];
        }
      }
      b_point[i] = _buffer_postionMoveTo[i][postion];
    }
    
    if(_buffer_enable[lastPostion_2] > 0){
      sPoint oa, ob;
      a_point[0] = _buffer_postionMoveTo[emMotorX][lastPostion_2];
      a_point[1] = _buffer_postionMoveTo[emMotorY][lastPostion_2];
      oa.x = a_point[0] - o_point[0];
      oa.y = a_point[1] - o_point[1];
      ob.x = b_point[0] - o_point[0];
      ob.y = b_point[1] - o_point[1];
      float dotaaob = (oa.x * ob.x + oa.y * ob.y);
      float lenoa = sqrt(pow(oa.x,2) + pow(oa.y,2));
      float lenob = sqrt(pow(ob.x,2) + pow(ob.y,2));
      float alpha = dotaaob / lenoa / lenob;
      if(alpha <= 1 && alpha >= -1){
        alpha = acos(alpha)*180/3.14;
        _buffer_angle[postion] = alpha;
      }
      else{
        _buffer_angle[postion] = -1;
      }
    }
    else if(_buffer_enable[lastPostion] > 0){
      _buffer_angle[postion] = -2;
    }
    else{
      _buffer_angle[postion] = -3;
    }
//    Serial.println(_buffer_angle[postion]);
    _buffer_length[postion] = length;
    _buffer_command[postion] = command;
    _buffer_maxStep[postion] = step;
    _buffer_enable[postion] = buffer;
  }
}
bool CNC_Drawing::is_running(void){ // check does it running
  return _running;
}
bool CNC_Drawing::is_release(void){ // check does just to target postion
  return _release;
}
void CNC_Drawing::clear_release(void){
  _release = false;
}
void CNC_Drawing::clear_buffer(void){
  while(_buffer_enable[_buffer_readPostion] > 0){
    _buffer_enable[_buffer_readPostion] = -1;
    if(++_buffer_readPostion >= BUFFER_SIZE_CNC)
      _buffer_readPostion = 0;
  }
  set_saveTargetPostion(false);
}
bool CNC_Drawing::is_pause(void){
  return _pause;
}
void CNC_Drawing::set_pause(void){
  _pause = true;
}
void CNC_Drawing::clear_pause(void){
  _pause = false;
  set_saveTargetPostion(false);
}
void CNC_Drawing::set_spr(eMotor motor, uint32_t value){ // set step per round
  _spr[motor] = value;
  _alpha = 2.0*PI/_spr[_max_step_asix];
  set_maxSpeed(_maxSpeed_mmps);
}
uint32_t CNC_Drawing::get_spr(eMotor motor){ // set step per round
  return _spr[motor];
}
void CNC_Drawing::set_maxSpeed(float speed){ // mm/s
  _maxSpeed_mmps = speed;
  _maxSpeed_radps = _maxSpeed_mmps*_resolution[_max_step_asix]*_alpha;
  _min_delay = ((long)(_alpha*T1_FREQ)) / _maxSpeed_radps;
  if(_running){
    if(_actual_delay == _min_delay)
      _run_state = emRun;
    else if(_actual_delay > _min_delay)
      _run_state = emAccel;
    else if(_actual_delay < _min_delay)
      _run_state = emSlowDown;
  }
}
float CNC_Drawing::get_maxSpeed(void){ // mms
  return _maxSpeed_mmps;
}
void CNC_Drawing::set_resolution(eMotor motor, float value){ // unit: step
  _resolution[motor] = value;
}
float CNC_Drawing::get_resolution(eMotor motor){ // unit: step
  return _resolution[motor];
}
void CNC_Drawing::set_acceleration(double value){ // mm/s
  _accel_mmps = value;
  _accel_radps = _accel_mmps*((_resolution[emMotorX]+_resolution[emMotorY])/2)*_alpha;;
  _start_delay = (((int)((T1_FREQ*0.676)))*sqrt(2*_alpha/_accel_radps));
  if(!_running){
    _actual_delay = _start_delay;
    _last_delay = _start_delay;
    _ticks = _actual_delay*1.0/TIME_PER_TICK;
  }
}
double CNC_Drawing::get_acceleration(void){ // mm/s
  return _accel_mmps;
}
void CNC_Drawing::set_coor(eMotor motor, int32_t value){ // set coordinates of asix with step unit
  _postion_current[motor] = _postion_moveTo[motor] = value;
  _release = true;
}
int32_t CNC_Drawing::get_coor(eMotor motor){ // get cordinates of asix with step unit
  return _postion_current[motor];
}
void CNC_Drawing::set_mask(uint8_t dir, uint8_t limit){
  _dir_mask[0] = dir & 0x1;
  _dir_mask[1] = (dir >> 1) & 0x1;
  _limit_mask[0] = limit & 0x1;
  _limit_mask[1] = (limit >> 1) & 0x1;
}
void CNC_Drawing::stop(bool deceleration){ // stop with decelertion or not
  if(!deceleration || _motor_reachLimit == _max_step_asix || _motor_reachLimit == emBothMotor){
    _postion_moveTo[_max_step_asix] = _postion_current[_max_step_asix];
    if(_max_step_asix != emMotorX)
      _postion_moveTo[emMotorX] = _postion_current[emMotorX];
    if(_max_step_asix != emMotorY)
      _postion_moveTo[emMotorY] = _postion_current[emMotorY];

    _actual_delay = _start_delay;
    _last_delay = _start_delay;
    _run_state = emStop;
    _ramp_counter = 0;
    _ticks = _actual_delay*1.0/TIME_PER_TICK;
  }
  else{
    if(_run_state != emStop)
      _run_state = emDecel;
  }
}
bool CNC_Drawing::buffer_enable(){
  return _buffer_enable[_buffer_readPostion] > 0;
}
eMotor CNC_Drawing::get_motorReachLimit(){
  return _motor_reachLimit;
}
void CNC_Drawing::run(){
  if(_postion_moveTo[_max_step_asix] != _postion_current[_max_step_asix]){
    for(uint8_t i = 0; i < emNumberOfAsix; i++){
      _limit_status[i] = digitalRead(_limit_pin[i]);
      if(_limit_status[i] != _limit_mask[i]){
        if(_motor_reachLimit == emNoneMotor)
          _motor_reachLimit = (eMotor)i;
        else if(_motor_reachLimit != (eMotor)i)
          _motor_reachLimit = emBothMotor;
      }
    }
    _running = true;
    if(_postion_moveTo[emMotorX] != _postion_current[emMotorX]){
      _scale_buffer[emMotorX] -= 1;
    }
    if(_postion_moveTo[emMotorY] != _postion_current[emMotorY]){
      _scale_buffer[emMotorY] -= 1;
    }
    uint32_t excess = _ticks;
    eMotor none_max_asix = (eMotor)!_max_step_asix;
    if(_scale_buffer[none_max_asix] < 1 && _scale_buffer[none_max_asix] > NONE_VALUE){
      if(_scale_buffer[none_max_asix] == 0)
        setTimer(_ticks);
      else{
        setTimer(_ticks*_scale_buffer[none_max_asix]);
        excess -= _ticks*_scale_buffer[none_max_asix];
      }
      while(_interrupted == 0){delayMicroseconds(1);}
      if(_limit_status[none_max_asix] == _limit_mask[none_max_asix] || _dir[none_max_asix] != -1){
        _postion_current[none_max_asix] += _dir[none_max_asix];
        digitalWrite(_step_pin[none_max_asix], HIGH);
        digitalWrite(_step_pin[none_max_asix], LOW);
      }
      _scale_reset[none_max_asix] = true;
    }
    if(_scale_buffer[none_max_asix] != _scale_buffer[_max_step_asix]){
      setTimer(excess);
      while(_interrupted == 0){delayMicroseconds(1);}
    }
    if(_limit_status[_max_step_asix] == _limit_mask[_max_step_asix] || _dir[_max_step_asix] != -1){
      _postion_current[_max_step_asix] += _dir[_max_step_asix];
      digitalWrite(_step_pin[_max_step_asix], HIGH);
      digitalWrite(_step_pin[_max_step_asix], LOW);
    }
    _scale_reset[_max_step_asix] = true;
    
    calculatorSpeed();
    if(_postion_moveTo[_max_step_asix] == _postion_current[_max_step_asix]){
      _release = true;
      // if decel, keep run until stop
      if(_pause == false || _run_state == emDecel){
        checkNextRun();
        while(_postion_moveTo[_max_step_asix] == _postion_current[_max_step_asix]){
          _find_decel_proactive = false;
          if(checkNextRun() == false)
            break;
        }
      }
    }
    for(uint8_t i = 0; i < emNumberOfAsix; i++){
      if(_scale_reset[i] == 1){
        _scale_reset[i] = 0;
        _scale_buffer[i] = _scale[i]+_scale_buffer[i];
      }
    }
  }
  else{
    _running = false;
    if(_pause == false){
      _find_decel_proactive = false;
      checkNextRun();
    }
  }
}
void CNC_Drawing::init(){
  singleton = this;
  _accel_radps = _accel_mmps*((_resolution[emMotorX]+_resolution[emMotorY])/2)*_alpha;;
  _start_delay = (((int)((T1_FREQ*0.676)))*sqrt(2*_alpha/_accel_radps ));
  _actual_delay = _start_delay;
  _last_delay = _start_delay;
  _ticks = _actual_delay*1.0/TIME_PER_TICK;
  for(uint16_t i = 0; i < BUFFER_SIZE_CNC; i++)
    _buffer_enable[i] = -1;
  for(uint8_t i = 0; i < emNumberOfAsix; i++){
    pinMode(_dir_pin[i], OUTPUT);
    pinMode(_step_pin[i], OUTPUT);
    pinMode(_limit_pin[i], INPUT);
  }
  pen.attach(_servo_pin);
  pen.write(_servo_degree[emUp]);
  
  xTaskCreatePinnedToCore(singleton->CNC_handler,"Task1",102400,NULL,1,&singleton->core_0,0);
}
