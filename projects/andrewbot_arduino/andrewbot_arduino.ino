#define __ARDUINO_UNO_REDUCED_MEMORY__
#include "src/RobotBrain.h"
#include <ros.h>

#include <andrewbot_msgs/RobotCommand.h>

#define R_HEAD 0x01
#define PIN_HEAD 2

/*Arduino pin is connected to the Motor drive module*/
#define ENA 5
#define ENB 6
#define IN1 7       // LEFT
#define IN2 8       // LEFT
#define IN3 9       // RIGHT
#define IN4 11      // RIGHT



andrewbot_msgs::RobotCommand out_cmd_pos;
andrewbot_msgs::RobotCommand out_cmd_color;
andrewbot_msgs::RobotCommand out_cmd_mode;

//byte reply[COMMAND_SIZE];

RobotBrain robotbrain;
ros::Publisher pub_servo_pos("andrewbot/servo_position", &out_cmd_pos);

void publishServosPosition(){
  byte reply[COMMAND_SIZE];
  robotbrain.getServosPosition(reply);
  out_cmd_pos.command= reply;
  out_cmd_pos.command_length=COMMAND_SIZE;
  pub_servo_pos.publish(&out_cmd_pos);
}

ros::Publisher pub_servo_mode("andrewbot/servo_mode", &out_cmd_mode);

void publishServosMode(){
  byte reply[COMMAND_SIZE];
  robotbrain.getServosMode(reply);
  out_cmd_mode.command=reply;
  out_cmd_mode.command_length=COMMAND_SIZE;
  //for (int i=0;i< COMMAND_SIZE; i++){
  //  out_cmd_mode.command[i]=reply[i];
  //}
  pub_servo_mode.publish(&out_cmd_mode);
}

ros::Publisher pub_servo_color("andrewbot/servo_color", &out_cmd_color);

void publishServosColor(){
  byte reply[COMMAND_SIZE];
  robotbrain.getServosColor(reply);
  out_cmd_color.command=reply;
  out_cmd_color.command_length=COMMAND_SIZE;
  //for (int i=0;i< COMMAND_SIZE; i++){
  //  out_cmd_color.command[i]=reply[i];
  //}
  pub_servo_color.publish(&out_cmd_color);
}


void commandCallback(const andrewbot_msgs::RobotCommand& msg) {
  int pos=0;
  switch (msg.command[pos++]) {
    case BASE_MOVE_CMD:
      if (robotbrain.isWheelsRegistered()){
        robotbrain.commandMovement(msg.command[1], msg.command[2], msg.command[3], msg.command[4]);
      }
      break;

    case SERVO_MOVE_CMD:
      robotbrain.moveServosCommand(msg.command, pos);
      break;
      // ignore others command
  }
}
ros::Subscriber<andrewbot_msgs::RobotCommand> sub_commands("andrewbot/RobotCommand", commandCallback);



ros::NodeHandle nh;
void setup() {
  robotbrain.registerWheels(IN1, IN2, IN3, IN4, ENA, ENB);
  robotbrain.registerChain(R_HEAD, PIN_HEAD);
  robotbrain.synchronize();


  //delay(500);
  Serial.setTimeout(20);
  
  nh.initNode();
  nh.advertise(pub_servo_pos);
  nh.advertise(pub_servo_mode);
  nh.advertise(pub_servo_color);
  nh.subscribe(sub_commands);
}

void loop() {
  robotbrain.update();
  publishServosPosition();
  publishServosColor();
  publishServosMode();
  nh.spinOnce();
  
  delay(10);
}
