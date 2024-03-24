#ifndef CNC_DRAWING_H
#define CNC_DRAWING_H

#include "Arduino.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "esp_system.h"
#include <Servo.h>

#define TIME_PER_TICK 0.1
#define NONE_VALUE (float)-211111111

// Timer/Counter 1 emRunning on 3,686MHz / 8 = 460,75kHz (2,17uS). (T1-FREQ 460750)
#define T1_FREQ 460750

typedef enum eRunState{
  emStop = 0,
  emAccel,  // emAccelerate
  emDecel,  // emDecelerate
  emSlowDown, 
  emRun     // max speed
};
typedef enum eMotor{
  emMotorX = 0,
  emMotorY = 1,
  emNumberOfAsix = 2,
  emBothMotor = 2,
  emNoneMotor = 3
};
typedef enum eCommand{
  emUp = 0,      // unchange
  emDown,        // unchange
  emDeltaDegree, // unchange
  emAbsolute,
  emRelative,
  emUnchange
};

class CNC_Drawing{
  public:
    CNC_Drawing();
    void init(void);
    void run(void);
    void set_penProperty(bool is_down, uint8_t degree);
    void pen_control(bool is_down);
    void prepareRunToBuffer(double pos_X, double pos_Y, eCommand command, uint8_t length = 0);
    int16_t checkBuffer(void);
    int32_t get_coor(eMotor motor);
    float get_postion(eMotor motor);
    bool is_running(void);
    bool is_release(void);
    void clear_release(void);
    void set_maxSpeed(float speed);
    float get_maxSpeed(void);
    void set_resolution(eMotor motor, float value);
    float get_resolution(eMotor motor);
    void set_acceleration(double value);
    double get_acceleration(void);
    void set_coor(eMotor motor, int32_t value);
    void stop(bool is_decel);
    void set_spr(eMotor motor, uint32_t value); // set step per round
    uint32_t get_spr(eMotor motor); // set step per round
    void clear_buffer(void); // set step per round
    bool buffer_enable(void);
    bool is_pause(void);
    void set_pause(void);
    void clear_pause(void);
    void set_saveTargetPostion(bool value);
    void set_length(uint32_t length);
    int8_t get_progress(void);
    void add_length(uint8_t length);
    void set_mask(uint8_t dir, uint8_t limit);
    eMotor get_motorReachLimit();
  private:
    struct sPoint{
      float x, y;
    };
    static void IRAM_ATTR onTimer(void);
    void IRAM_ATTR interrup_raise(void);
    static void CNC_handler(void * pvParameters);
    void setTimer(uint32_t ticks);
    void offTimer(void);
    uint32_t ticksCount(void);
    void calculatorSpeed(void);
    bool checkNextRun(void);
    
    eMotor _motor_reachLimit = emNoneMotor;
    bool _dir_mask[emNumberOfAsix] = {0,0};
    bool _limit_mask[emNumberOfAsix] = {1,1};
    bool _limit_status[emNumberOfAsix] = {1,1};
    const uint8_t _limit_pin[emNumberOfAsix] = {35,34};
    const uint8_t _dir_pin[emNumberOfAsix] = {12,27};
    const uint8_t _step_pin[emNumberOfAsix] = {26,14};
    const uint8_t _servo_pin = 25;
    TaskHandle_t core_0;
    
    bool _pause = false;
    bool _running;
    bool _release;
    uint32_t _interrupted;
    bool _save_targetPostion = false;
    
    eMotor _max_step_asix = emMotorX;
    eRunState _run_state = emAccel;
    
    uint32_t _spr[emNumberOfAsix] = {400,400};
    double _alpha = 2.0*PI/_spr[_max_step_asix];
    float _resolution[emNumberOfAsix] = {40,40}; //step per mm
    
    double _accel_mmps = 10; // mm/s2
    double _accel_radps = 10; // rad/s2
    float _maxSpeed_mmps = 20; // mm/s
    float _maxSpeed_radps = 20; // rad/s
    
    uint32_t _min_delay; //us
    uint32_t _start_delay; //us
    uint32_t _actual_delay; //us
    uint32_t _last_delay; //us
    uint32_t _ticks;
    uint16_t _servo_degree[3] = {0,90,90}; // element [2] = abs([0] - [1])
    int32_t _ramp_counter = 0;
    
    int8_t _dir[emNumberOfAsix] = {1,1};
    int32_t _postion_current[emNumberOfAsix] = {0,0}, _postion_moveTo[emNumberOfAsix] = {0,0};
    
    float _scale[emNumberOfAsix] = {0,0};
    float _scale_buffer[emNumberOfAsix] = {0,0};
    bool _scale_reset[emNumberOfAsix] = {0,0};
    eCommand _pen_state = emUp;
    bool _decel_proactive = false;
    bool _find_decel_proactive = false;
    int32_t _step_runUntil_decel;
    
    #define BUFFER_SIZE_CNC 300
    uint32_t _buffer_maxStep[BUFFER_SIZE_CNC]; // max step at buffer
    int16_t _buffer_angle[BUFFER_SIZE_CNC];
    int8_t _buffer_enable[BUFFER_SIZE_CNC];
    int32_t _buffer_postionMoveTo[emNumberOfAsix][BUFFER_SIZE_CNC];
    eCommand _buffer_command[BUFFER_SIZE_CNC];
    int16_t _buffer_readPostion = 0;
    uint8_t _buffer_length[BUFFER_SIZE_CNC];
    uint32_t _buffer_totalLength = 0;
    uint32_t _buffer_ranLength = 0;
    
    static CNC_Drawing *singleton; 
    hw_timer_t* timer;
};

#endif
