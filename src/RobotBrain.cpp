#include "RobotBrain.h"

RobotBrain::RobotBrain()
{
  for (int chain_pos = 0; chain_pos < MAX_NUMBER_CHAINS; chain_pos++)
  {
    chain_list[chain_pos] = NULL;
    chain_map_pos[chain_pos] = -1;
  }
  for (int servo_pos = 0; servo_pos < MAX_NUMBER_SERVOS; servo_pos++)
  {
    servo_list[servo_pos] = NULL;
  }
  //reply.reset();
  last_chain_added = 0;
  last_servo_added = 0;
  smart_led = NULL;
}

RobotBrain::~RobotBrain()
{
  int chain_pos = 0;
  for (int pos = 0; pos < last_chain_added; pos++)
  {
    chain_pos = chain_map_pos[pos];
    delete chain_list[chain_pos];
  }

  for (int pos = 0; pos < last_servo_added; pos++)
  {
    delete servo_list[pos];
  }
  delete smart_led;
}

void reset_command(byte *command)
{
  for (int pos = 0; pos < COMMAND_SIZE; pos++)
  {
    command[pos] = 0;
  }
}

void RobotBrain::synchronize()
{
  int chain_pos = 0;
  for (int iter = 0; iter < SYNC_ITERATIONS; iter++)
  {
    update();
  }

  for (int pos = 0; pos < last_chain_added; pos++)
  {
    chain_pos = chain_map_pos[pos];

    for (int module = 0; module < MAX_CHAIN; module++)
    {

      switch (chain_list[chain_pos]->getType(module))
      {
      case TYPE_SERVO:
        servo_list[last_servo_added] = chain_list[chain_pos]->getServo(module);
        servo_list[last_servo_added]->setLim(false);
        chain_list[chain_pos]->update();
        chain_list[chain_pos]->update();
        servo_list[last_servo_added]->setColor(SERVO_COLOR_BLUE);
        chain_list[chain_pos]->update();
        servo_list[last_servo_added]->setPosition(90);
        chain_list[chain_pos]->update();
        servo_list[last_servo_added]->setAutoUpdate(false);
        last_servo_added++;
        break;

      case TYPE_LED:
        smart_led = chain_list[chain_pos]->getLed(module);
        break;
      }
    }
  }
}

void RobotBrain::update()
{
  int chain_pos = 0;
  for (int pos = 0; pos < last_chain_added; pos++)
  {
    chain_pos = chain_map_pos[pos];
    chain_list[chain_pos]->update();
  }
  if (isWheelsRegistered())
  {
    stopWheels();
  }
}

int RobotBrain::registerChain(int chain_pos, int pwmPin)
{
  if (chain_pos >= MAX_NUMBER_CHAINS)
  {
    return -1;
  }
  if (chain_list[chain_pos] != NULL)
  {
    return -2;
  }
  chain_list[chain_pos] = new Chain(pwmPin);
  chain_map_pos[last_chain_added] = chain_pos;
  last_chain_added++;
  return last_chain_added;
}

int RobotBrain::registerWheels(int IN1, int IN2, int IN3, int IN4, int ENA, int ENB)
{
  in1_pin = IN1;
  in2_pin = IN2;
  in3_pin = IN3;
  in4_pin = IN4;
  enA_pin = ENA;
  enB_pin = ENB;

  pinMode(in1_pin, OUTPUT); //Motor-driven port configuration
  pinMode(in2_pin, OUTPUT);
  pinMode(in3_pin, OUTPUT);
  pinMode(in4_pin, OUTPUT);
  pinMode(enA_pin, OUTPUT);
  pinMode(enB_pin, OUTPUT);
  wheels_registered = true;
}

int RobotBrain::processCommand(byte *command)
{
  int pos = 0;
  byte cmd = command[pos++];
  switch (cmd)
  {
  case SERVO_SET_MIN:
    setServosMinCommand(command, pos);
    break;
  case SERVO_GET_MIN:
    break;
  case SERVO_SET_MAX:
    setServosMaxCommand(command, pos);
    break;
  case SERVO_GET_MAX:
    break;
  case SERVO_SET_MANUAL:
    setServosManual(command, pos);
    break;
  case SERVO_MOVE_DELTA_CMD:
    moveDeltaServosCommand(command, pos);
    break;
  case SERVO_MOVE_CMD:
    moveServosCommand(command, pos);
    break;
  case SERVO_COLOR_CMD:
    setServosColor(command, pos);
    break;
  
  case BASE_MOVE_CMD:

    if (isWheelsRegistered()){
      commandMovement(command[pos++], command[pos++], command[pos++], command[pos++]);
    }
    break;

  case LED_COLOR_CMD:
    break;
  }
}

/**
 * Set Min position in a Servo. 
 * If value provided is 0, skip servo 
 */
void RobotBrain::setServosMinCommand(byte *command, int &pos)
{
  int value=0; 
  for (int servo = 0; servo < last_servo_added; servo++)
  {
    value = command[pos++];
    if (value == 0){
      continue;
    }
    servo_list[servo]->setPositionMin(value);
  }
}

/**
 * Set Max position in a Servo. 
 * If value provided is 0, skip servo 
 */
void RobotBrain::setServosMaxCommand(byte *command, int &pos)
{

  int value=0; 
  for (int servo = 0; servo < last_servo_added; servo++)
  {
    value = command[pos++];
    if (value == 0){
      continue;
    }
    servo_list[servo]->setPositionMax(value);
  }
}

void RobotBrain::moveServosCommand(byte *command, int &pos)
{

  for (int servo = 0; servo < last_servo_added; servo++)
  {
    servo_list[servo]->setPosition(command[pos++]);
  }
}

void RobotBrain::moveDeltaServosCommand(byte *command, int &pos)
{
  int value = 0;

  for (int servo = 0; servo < last_servo_added; servo++)
  {
    value = servo_list[servo]->getPosition();
    value += int(char(command[pos++])); //Enable negative increment
    servo_list[servo]->setPosition(value);
  }
}

int RobotBrain::getServosPosition(byte *reply)
{
  int pos = 0;
  reply[pos++] = SERVO_GET_POS;
  reply[pos++] = last_servo_added;

  for (int servo = 0; servo < last_servo_added; servo++)
  {
    reply[pos++] = servo_list[servo]->getPosition();
  }

  for (; pos < COMMAND_SIZE; pos++)
  {
    reply[pos] = 0;
  }
  return pos;
}

/*
Return servo mode
*/
int RobotBrain::getServosMode(byte *reply)
{
  int pos = 0;
  reply[pos++] = SERVO_REQ_MOD;
  reply[pos++] = last_servo_added;

  for (int servo = 0; servo < last_servo_added; servo++)
  {
    reply[pos++] = servo_list[servo]->getLimStatus();
  }

  for (; pos < COMMAND_SIZE; pos++)
  {
    reply[pos] = 0;
  }
  return pos;
}

void RobotBrain::setServosManual(byte *command, int &pos)
{
  for (int servo = 0; servo < last_servo_added; servo++)
  {
    if (command[pos++] == 0)
    {
      servo_list[servo]->setLim(false);
    }
    else
    {
      servo_list[servo]->setLim(true);
    }
  }
}

void RobotBrain::setServosColor(byte *command, int &pos)
{
  for (int servo = 0; servo < last_servo_added; servo++)
  {
    servo_list[servo]->setColor(command[pos++]);
  }
}

int RobotBrain::getServosColor(byte *reply)
{
  int pos = 0;
  reply[pos++] = SERVO_COLOR_GET;
  reply[pos++] = last_servo_added;

  for (int servo = 0; servo < last_servo_added; servo++)
  {
    reply[pos++] = servo_list[servo]->getColor();
  }

  for (; pos < COMMAND_SIZE; pos++)
  {
    reply[pos] = 0;
  }
  return pos;
}

void RobotBrain::commandMovement(byte left_dir, byte right_dir, byte left_speed, byte right_speed)
{
  if (left_dir == 0x01)
  {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
  }
  else
  {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
  }
  if (right_dir == 0x01)
  {
    digitalWrite(in3_pin, HIGH);
    digitalWrite(in4_pin, LOW);
  }
  else
  {
    digitalWrite(in3_pin, LOW);
    digitalWrite(in4_pin, HIGH);
  }

  analogWrite(enA_pin, left_speed);
  analogWrite(enB_pin, right_speed);
}

void RobotBrain::stopWheels()
{
  digitalWrite(enA_pin, LOW); //Turn off the motor enable pin
  digitalWrite(enB_pin, LOW);
}

// int RobotBrain::processCommand(byte *command)
// {
//   int pos = 0;
//   byte cmd = command[pos++];
//   ack = command[pos++];

//   switch (cmd)
//   {
//   case HEADER_BRAIN_COMMAND:
//     reply.write(HEADER_BRAIN_COMMAND);
//     reply.write(HEADER_REPLY_COMMAND);
//     reply.write(ack);
//     reply.write(0);

//     processBrainCommand(command, pos);
//     break;

//   case HEADER_COMMAND_LEN:
//     return command[pos++];

//   default:
//     writeErrorInBufferReply(255, 255, cmd);
//   }
//   return error;
// }

// int RobotBrain::processBrainCommand(byte *command, int &pos)
// {
//   byte n_command = command[pos++];

//   //reply.enableConditionalWrite();
//   reply.enableCounter(reply.CONT_N_CC);

//   for (byte cmd_pos = 0; cmd_pos < n_command; cmd_pos++)
//   {
//     byte cmd = command[pos++];
//     switch (cmd)
//     {
//     case HEADER_CHAIN_CHANNEL_COMMAND:

//       reply.incrementCounter(reply.CONT_N_CC);
//       reply.write(HEADER_CHAIN_CHANNEL_COMMAND);
//       processChainCommand(command, pos);

//       break;

//     default:
//       writeErrorInBufferReply(255, cmd, 0);
//     }
//     if (error < 0)
//     {
//       return error;
//     }
//   }
//       //reply.disableConditionalWrite();
//   return error;
// }

// //ChainCommand  ChainID NumberMessages ModuleCommand ModulePosition  VALUES
// //CC            CI      NM             MC             MP
// int RobotBrain::processChainCommand(byte *command, int &pos)
// {
//   byte chain_pos = command[pos++];
//   byte n_messages = command[pos++];
//   reply.write(chain_pos);
//   reply.enableCounter(reply.CONT_N_COMMAND);
//   byte module_command;
//   for (byte message = 0; message < n_messages; message++)
//   {

//     module_command = command[pos++];
//     switch (module_command)
//     {
//     case SERVO_WRITE_POS ... SERVO_LAST_COMMAND:
//       processServoCommand(command, pos, chain_pos, module_command);
//       break;

//     case LED_COLOR_SET ... LED_LAST_COMMAND:
//       processLedCommand(command, pos, chain_pos, module_command);
//       break;

//     default:
//       writeErrorInBufferReply(255, 255, module_command);
//     }
//     if (error < 0)
//     {
//       return error;
//     }
//   }
//   return error;
// }
// void RobotBrain::processServoCommand(byte *command, int &pos, byte chain_pos, byte module_command)
// {
//   int position = 0;
//   byte module_pos = command[pos++];

//   switch (module_command)
//   {
//   case SERVO_WRITE_POS:
//     chain_list[chain_pos]->getServo(module_pos).setPositionSmart(command[pos++]);
//     break;

//   case SERVO_READ_POS:
//     position = chain_list[chain_pos]->getServo(module_pos).getPositionSmart();
//     reply.incrementCounter(reply.CONT_N_COMMAND);
//     reply.write(module_command);
//     reply.write(module_pos);
//     reply.write(position);
//     reply.commit();
//     break;

//   case SERVO_COLOR_SET:
//     chain_list[chain_pos]->getServo(module_pos).setColor(command[pos++], command[pos++], command[pos++]);
//     break;

//   default:
//     writeErrorInBufferReply(chain_pos, module_pos, SERVO_ERROR_COMMAND_UNKNOWN);
//   }
// }

// void RobotBrain::processLedCommand(byte *command, int &pos, byte chain_pos, byte module_command)
// {
//   byte module_pos = command[pos++];

//   switch (module_command)
//   {
//   case LED_COLOR_SET:
//     chain_list[chain_pos]->getLed(module_pos).setColor(command[pos++], command[pos++], command[pos++], command[pos++]);
//     break;

//   default:
//     writeErrorInBufferReply(chain_pos, module_pos, LED_ERROR_COMMAND_UNKNOWN);
//   }
// }

// void RobotBrain::writeErrorInBufferReply(byte chain_pos, byte module_pos, byte error_code)
// {
//   reply.reset();
//   error = -1;
//   reply.write(HEADER_BRAIN_COMMAND);
//   reply.write(HEADER_REPLY_COMMAND);
//   reply.write(ack);
//   reply.write(1); // Error code
//   reply.write(chain_pos); // Body part
//   reply.write(module_pos);
//   reply.write(error_code);
// }

/*  END  OF CODE */
