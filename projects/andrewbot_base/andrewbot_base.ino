#define __ARDUINO_UNO_REDUCED_MEMORY__
#include <RobotBrain.h>
#include <ros.h>

#include <andrewbot_msgs/RobotCommand.h>

/*Arduino pin is connected to the Motor drive module*/
#define ENA 5
#define ENB 6
#define IN1 7       // LEFT
#define IN2 8       // LEFT
#define IN3 9       // RIGHT
#define IN4 11      // RIGHT


andrewbot_msgs::RobotCommand input_cmd;

byte reply[COMMAND_SIZE];

RobotBrain robotbrain;
ros::Publisher pub_servo_pos("andrewbot/servo_position", &output_cmd);

void commandCallback(const andrewbot_msgs::RobotCommand& msg) {
  int pos=0;
  switch (msg.command[pos++]) {
    case BASE_MOVE_CMD:
      if (robotbrain.isWheelsRegistered()){
        robotbrain.commandMovement(msg.command[1], msg.command[2], msg.command[3], msg.command[4]);
      }
      break;
  }
}
ros::Subscriber<andrewbot_msgs::RobotCommand> sub_commands("andrewbot/RobotCommand", commandCallback);



void publishServoPosition(){
  robotbrain.getServosPosition(reply);
  output_cmd.command=reply;
  pub_servo_pos.publish(&output_cmd);
}

ros::NodeHandle nh;
void setup() {
  robotbrain.registerWheels(IN1, IN2, IN3, IN4, ENA, ENB);
  robotbrain.registerChain(R_HEAD, PIN_HEAD);
  robotbrain.synchronize();


  delay(500);
  Serial.setTimeout(50);
  
  nh.initNode();
  nh.advertise(pub_servo_pos);
  nh.subscribe(sub_commands);
}

void loop() {
  robotbrain.update();
  publishServoPosition();
  nh.spinOnce();
  
  delay(10);
}
