#ifndef RobotBrain_h
#define RobotBrain_h

#ifdef TESTING
#include "../stubs/Arduino.h"
#include "../stubs/Meccanoid.h"
#else
#include "Arduino.h"
#include "Meccanoid.h"
#endif



#define MAX_NUMBER_CHAINS 4
#define SYNC_ITERATIONS 20

#define MAX_NUMBER_SERVOS 15
#define COMMAND_SIZE 18

#define SERVO_GET_POS 0x01
#define SERVO_SET_MIN 0x02
#define SERVO_GET_MIN 0x03
#define SERVO_SET_MAX 0x04
#define SERVO_GET_MAX 0x05
#define SERVO_SET_MANUAL 0x06
#define SERVO_INC_POS 0x07    
#define SERVO_MOVE_CMD 0x08    // Bluetooth
#define SERVO_REQ_MOD 0x09    
#define SERVO_COLOR_GET 0x0b  
#define SERVO_COLOR_CMD 0x0c  // Bluetooth
#define BASE_MOVE_CMD 0x0d         // Bluetooth

#define LED_COLOR_CMD 0x11    // Bluetooth
    


//#define SYNC_TIME_SLEEP 100

// #define HEADER_BRAIN_COMMAND 0x4A         // 74
// #define HEADER_REPLY_COMMAND 0xFC         // 252
// #define HEADER_CHAIN_CHANNEL_COMMAND 0xCC // 204
// #define HEADER_COMMAND_LEN 0x4F           // 79

// #define MODULE_ERROR_UNKNOWN 0x00

// #define SERVO_WRITE_POS 0x10 // 16
// #define SERVO_READ_POS 0x11  // 17
// #define SERVO_COLOR_SET 0x12 // 18
// #define SERVO_LAST_COMMAND SERVO_COLOR_SET
// #define SERVO_ERROR_COMMAND_UNKNOWN 0x10

// #define LED_COLOR_SET 0x20 // 32
// #define LED_LAST_COMMAND LED_COLOR_SET
// #define LED_ERROR_COMMAND_UNKNOWN 0x20


class RobotBrain{
  public:
    //Buffer reply;
    int ack = 0;

  public:
    RobotBrain();

    ~RobotBrain();

    void reset_command(byte *command);

    void synchronize();

    void update();

    int registerChain(int body_part, int pwmPin);

    int registerWheels(int IN1, int IN2, int IN3, int IN4, int ENA, int ENB);

    void commandMovement(byte left_dir, byte right_dir, byte left_speed, byte right_speed);

    void stopWheels();

    bool isWheelsRegistered(){
      return wheels_registered;
    };

    int processCommand(byte *command);

    void setServosMinCommand(byte *command, int &pos);

    // int getServoMinCommand(byte *command, int &pos);
    
    void setServosMaxCommand(byte *command, int &pos);
    
    // int getServoMaxCommand(byte *command, int &pos);

    void moveServosCommand(byte *command, int &pos);

    void deltaPosServosCommand(byte *command, int &pos);

    void setServosManual(byte *command, int &pos);

    void setServosColor(byte *command, int &pos);


    // Publish data
    int getServosPosition(byte *reply);

    int getServosMode(byte *reply);

    int  getServosColor(byte *reply);

    // int processCommand(byte *command);

    // int processBrainCommand(byte *command, int &pos);

    // int processChainCommand(byte *command, int &pos);

    // void processServoCommand(byte *command, int &pos, byte chain_pos, byte module_command);

    // void processLedCommand(byte *command, int &pos, byte chain_pos, byte module_command);

    // void writeErrorInBufferReply(byte chain_pos, byte module_pos, byte error_code);

  private:
    Chain *chain_list[MAX_NUMBER_CHAINS];
    int chain_map_pos[MAX_NUMBER_CHAINS];
    int last_chain_added = 0;

    MeccanoServo *servo_list[MAX_NUMBER_SERVOS];
    int last_servo_added =0;

    MeccanoLed *smart_led;

    bool wheels_registered = false;

    int error = 0;

    int in1_pin;
    int in2_pin;
    int in3_pin;
    int in4_pin;
    int enA_pin;
    int enB_pin;
};

#endif
