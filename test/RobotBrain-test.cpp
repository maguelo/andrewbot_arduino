#ifdef TESTING
#include "gtest/gtest.h"

#define protected public
#define private public
#include "RobotBrain.h"
#undef protected
#undef private

#include <iostream>
using namespace std;


// void printBuffer(Buffer buffer){
//     for (int pos =0; pos<buffer.len();pos++){
//         cout << pos << " " << int(buffer.get(pos)) << endl;
//     }
// }

TEST(register_chain, ok)
{
    RobotBrain robotbrain;
    int result = robotbrain.registerChain(0, 0);
    EXPECT_EQ(result, 1);
}

TEST(register_chain, error_max)
{
    RobotBrain robotbrain;
    int result = robotbrain.registerChain(10, 0);
    EXPECT_EQ(result, -1);
}

TEST(register_chain, error_position)
{
    RobotBrain robotbrain;
    int result = robotbrain.registerChain(1, 0);
    EXPECT_EQ(result, 1);
    result = robotbrain.registerChain(1, 0);
    EXPECT_EQ(result, -2);
}

TEST(synchronize, update_chain_empty)
{
    RobotBrain robotbrain;
    int result = robotbrain.registerChain(1, 0);
    
    robotbrain.synchronize();
    for (int servo =0; servo < robotbrain.last_servo_added; servo++){
        std::cout << robotbrain.servo_list[servo] << std::endl;
    }
}


TEST(getServosPosition, empty_value)
{
    RobotBrain robotbrain;
    int result = robotbrain.registerChain(1, 0);
    robotbrain.synchronize();
    byte reply[COMMAND_SIZE];
    int last_position = robotbrain.getServosPosition(reply);
    EXPECT_EQ(last_position, 2);
    EXPECT_EQ(reply[0], SERVO_GET_POS);
    EXPECT_EQ(reply[1], 0);
}

TEST(moveServosCommand, empty_servos)
{
    RobotBrain robotbrain;
    int result = robotbrain.registerChain(1, 0);
    int pos =0;
    robotbrain.synchronize();
    byte command[COMMAND_SIZE]={SERVO_MOVE_CMD, 90,90,90,90,90,90,90,90,90,90,90,90,90,0,0,0,0};
    robotbrain.moveServosCommand(command, pos);

    byte reply[COMMAND_SIZE];
    int last_position = robotbrain.getServosPosition(reply);
 
    EXPECT_EQ(last_position, 2);
    EXPECT_EQ(reply[0], SERVO_GET_POS);
    EXPECT_EQ(reply[1], 0);
}

TEST(process_servo_command, servo_write_pos){
    byte value = -1;
    char value2 = byte(-1);
    std::cout << int(char(0xF0)) << std::endl;
}



// TEST(process_servo_command, servo_write_pos)
// {
//     RobotBrain robotbrain;
//     int result = robotbrain.registerChain(1, 0);
//     EXPECT_EQ(result, 1);
//     byte command[4] = {0, 90};
//     int pos = 0;
//     robotbrain.processServoCommand(command, pos, 1, 0x10);
//     EXPECT_EQ(pos, 2);
//     EXPECT_EQ(robotbrain.reply.len(), 0);
//     EXPECT_EQ(robotbrain.error, 0);
// }

// TEST(process_servo_command, servo_read_pos)
// {
//     RobotBrain robotbrain;
//     int result = robotbrain.registerChain(1, 0);
//     EXPECT_EQ(result, 1);

//     byte command[4] = {0, 90};
//     int pos = 0;

//     robotbrain.processServoCommand(command, pos, 1, 0x11);

//     EXPECT_EQ(robotbrain.chain_list[1]->modules[0].lastInput, 0);
//     EXPECT_EQ(pos, 1);
//     EXPECT_EQ(robotbrain.reply.len(), 3);
//     EXPECT_EQ(robotbrain.reply.get(0), 0x11);
//     EXPECT_EQ(robotbrain.reply.get(1), 0);
//     EXPECT_EQ(robotbrain.reply.get(2), 0xEC);
//     EXPECT_EQ(robotbrain.error, 0);
// }

// TEST(process_servo_command, servo_color_set)
// {
//     RobotBrain robotbrain;
//     int result = robotbrain.registerChain(1, 0);
//     EXPECT_EQ(result, 1);

//     byte command[4] = {0, 1, 1, 1};
//     int pos = 0;

//     robotbrain.processServoCommand(command, pos, 1, 0x12);

//     EXPECT_EQ(robotbrain.chain_list[1]->modules[0].lastInput, 0);
//     EXPECT_EQ(pos, 4);
//     EXPECT_EQ(robotbrain.reply.len(), 0);
//     EXPECT_EQ(robotbrain.error, 0);
// }

// TEST(process_servo_command, unknown_command)
// {
//     RobotBrain robotbrain;
//     int result = robotbrain.registerChain(1, 0);
//     EXPECT_EQ(result, 1);

//     byte command[4] = {0, 1, 1, 1};
//     int pos = 0;

//     robotbrain.processServoCommand(command, pos, 1, 0x13);

//     EXPECT_EQ(pos, 1);
//     EXPECT_EQ(robotbrain.reply.len(), 7);
//     EXPECT_EQ(robotbrain.reply.get(0), 0x4A);
//     EXPECT_EQ(robotbrain.reply.get(1), 0xFC);
//     EXPECT_EQ(robotbrain.reply.get(2), robotbrain.ack);
//     EXPECT_EQ(robotbrain.reply.get(3), 1);
//     EXPECT_EQ(robotbrain.reply.get(4), 1);
//     EXPECT_EQ(robotbrain.reply.get(5), 0);
//     EXPECT_EQ(robotbrain.reply.get(6), 0x10);
//     EXPECT_EQ(robotbrain.error, -1);
// }

// TEST(process_led_command, set_color)
// {
//     RobotBrain robotbrain;
//     int result = robotbrain.registerChain(1, 0);
//     EXPECT_EQ(result, 1);

//     byte command[5] = {0, 1, 1, 1, 1};
//     int pos = 0;

//     robotbrain.processLedCommand(command, pos, 1, 0x20);

//     EXPECT_EQ(pos, 5);
//     EXPECT_EQ(robotbrain.reply.len(), 0);
//     EXPECT_EQ(robotbrain.chain_list[1]->modules[0].output1, 9);
//     EXPECT_EQ(robotbrain.chain_list[1]->modules[0].output2, 73);
//     EXPECT_EQ(robotbrain.error, 0);
// }

// TEST(process_led_command, unknown_command)
// {
//     RobotBrain robotbrain;
//     int result = robotbrain.registerChain(1, 0);
//     EXPECT_EQ(result, 1);

//     byte command[5] = {0, 1, 1, 1, 1};
//     int pos = 0;

//     robotbrain.processLedCommand(command, pos, 1, 0x21);

//     EXPECT_EQ(robotbrain.chain_list[1]->modules[0].lastInput, 0);
//     EXPECT_EQ(pos, 1);
//     EXPECT_EQ(robotbrain.reply.len(), 7);
//     EXPECT_EQ(robotbrain.reply.get(0), 0x4A);
//     EXPECT_EQ(robotbrain.reply.get(1), 0xFC);
//     EXPECT_EQ(robotbrain.reply.get(2), robotbrain.ack);
//     EXPECT_EQ(robotbrain.reply.get(3), 1);
//     EXPECT_EQ(robotbrain.reply.get(4), 1);
//     EXPECT_EQ(robotbrain.reply.get(5), 0);
//     EXPECT_EQ(robotbrain.reply.get(6), 0x20);
//     EXPECT_EQ(robotbrain.error, -1);
// }

// TEST(process_chain_command, one_module)
// {
//     RobotBrain robotbrain;
//     int result = robotbrain.registerChain(1, 0);
//     EXPECT_EQ(result, 1);

//     byte command[5] = {1, 1, 0x10, 0, 90};
//     int pos = 0;
//     robotbrain.processChainCommand(command, pos);
//     EXPECT_EQ(pos, 5);
//     EXPECT_EQ(robotbrain.error, 0);
// }

// TEST(process_chain_command, two_module)
// {
//     RobotBrain robotbrain;
//     int result = robotbrain.registerChain(1, 0);
//     EXPECT_EQ(result, 1);

//     byte command[12] = {1, 2, 0x10, 0, 90, 0x20, 1, 1, 1, 1, 1};
//     int pos = 0;
//     robotbrain.processChainCommand(command, pos);
//     EXPECT_EQ(pos, 11);
//     EXPECT_EQ(robotbrain.error, 0);
//     EXPECT_EQ(robotbrain.chain_list[1]->modules[1].output1, 9);
//     EXPECT_EQ(robotbrain.chain_list[1]->modules[1].output2, 73);
// }

// TEST(process_chain_command, two_module_first_error)
// {
//     RobotBrain robotbrain;
//     int result = robotbrain.registerChain(1, 0);
//     EXPECT_EQ(result, 1);

//     byte command[12] = {1, 2, 0x14, 0, 90, 0x20, 1, 1, 1, 1, 1};
//     int pos = 0;
//     robotbrain.processChainCommand(command, pos);
//     EXPECT_EQ(pos, 3);
//     EXPECT_EQ(robotbrain.reply.len(), 7);
//     EXPECT_EQ(robotbrain.reply.get(0), 0x4A);
//     EXPECT_EQ(robotbrain.reply.get(1), 0xFC);
//     EXPECT_EQ(robotbrain.reply.get(2), robotbrain.ack);
//     EXPECT_EQ(robotbrain.reply.get(3), 1);
//     EXPECT_EQ(robotbrain.reply.get(4), 255);
//     EXPECT_EQ(robotbrain.reply.get(5), 255);
//     EXPECT_EQ(robotbrain.reply.get(6), 0x14);
//     EXPECT_EQ(robotbrain.error, -1);
// }

// TEST(process_chain_command, two_module_second_error)
// {
//     RobotBrain robotbrain;
//     int result = robotbrain.registerChain(1, 0);
//     EXPECT_EQ(result, 1);

//     byte command[12] = {1, 2, 0x10, 0, 90, 0x23, 1, 1, 1, 1, 1};
//     int pos = 0;
//     result = robotbrain.processChainCommand(command, pos);
//     EXPECT_EQ(result, -1);

//     EXPECT_EQ(pos, 6);
//     EXPECT_EQ(robotbrain.reply.len(), 7);
//     EXPECT_EQ(robotbrain.reply.get(0), 0x4A);
//     EXPECT_EQ(robotbrain.reply.get(1), 0xFC);
//     EXPECT_EQ(robotbrain.reply.get(2), robotbrain.ack);
//     EXPECT_EQ(robotbrain.reply.get(3), 1);
//     EXPECT_EQ(robotbrain.reply.get(4), 255);
//     EXPECT_EQ(robotbrain.reply.get(5), 255);
//     EXPECT_EQ(robotbrain.reply.get(6), 0x23);
//     EXPECT_EQ(robotbrain.error, -1);
// }

// TEST(process_brain_command, write_position){
//     RobotBrain robotbrain;
//     int result = robotbrain.registerChain(1, 0);
//     EXPECT_EQ(result, 1);

//     byte command[20] = {1, 0xCC, 1, 1, 0x10, 0, 90};
//     int pos = 0;
//     result = robotbrain.processBrainCommand(command, pos);
//     EXPECT_EQ(result, 0);

//     EXPECT_EQ(pos, 7);
//     EXPECT_EQ(robotbrain.reply.len(), 4);
//     EXPECT_EQ(robotbrain.reply.get(0), 1);
//     EXPECT_EQ(robotbrain.reply.get(1), 0xCC);
//     EXPECT_EQ(robotbrain.reply.get(2), 1);
//     EXPECT_EQ(robotbrain.reply.get(3), 0);
//     EXPECT_EQ(robotbrain.error, 0);
// }

// TEST(process_brain_command, get_position){
//     RobotBrain robotbrain;
//     int result = robotbrain.registerChain(1, 0);
//     EXPECT_EQ(result, 1);

//     byte command[20] = {1, 0xCC, 1, 1, 0x11, 0};
//     int pos = 0;
//     result = robotbrain.processBrainCommand(command, pos);
//     EXPECT_EQ(result, 0);
    
//     EXPECT_EQ(pos, 6);
//     EXPECT_EQ(robotbrain.reply.len(), 7);
//     EXPECT_EQ(robotbrain.reply.get(0), 1);
//     EXPECT_EQ(robotbrain.reply.get(1), 0xCC);
//     EXPECT_EQ(robotbrain.reply.get(2), 1);
//     EXPECT_EQ(robotbrain.reply.get(3), 1);
//     EXPECT_EQ(robotbrain.reply.get(4), 17);
//     EXPECT_EQ(robotbrain.reply.get(5), 0);
//     EXPECT_EQ(robotbrain.reply.get(6), 236);
//     EXPECT_EQ(robotbrain.error, 0);
// }

// TEST(process_brain_command, write_get_position){
//     RobotBrain robotbrain;
//     int result = robotbrain.registerChain(1, 0);
//     EXPECT_EQ(result, 1);

//     byte command[20] = {1, 0xCC, 1, 2, 0x10, 0, 90, 0x11,1};
//     int pos = 0;
//     result = robotbrain.processBrainCommand(command, pos);
//     EXPECT_EQ(result, 0);
    
//     EXPECT_EQ(pos, 9);
//     EXPECT_EQ(robotbrain.reply.len(), 7);
//     EXPECT_EQ(robotbrain.reply.get(0), 1);
//     EXPECT_EQ(robotbrain.reply.get(1), 0xCC);
//     EXPECT_EQ(robotbrain.reply.get(2), 1);
//     EXPECT_EQ(robotbrain.reply.get(3), 1);
//     EXPECT_EQ(robotbrain.reply.get(4), 17);
//     EXPECT_EQ(robotbrain.reply.get(5), 1);
//     EXPECT_EQ(robotbrain.reply.get(6), 236);
//     EXPECT_EQ(robotbrain.error, 0);
    
// }

// TEST(process_brain_command, get_write_position){
//     RobotBrain robotbrain;
//     int result = robotbrain.registerChain(1, 0);
//     EXPECT_EQ(result, 1);

//     byte command[20] = {1, 0xCC, 1, 2, 0x11, 0, 0x10,1, 90};
//     int pos = 0;
//     result = robotbrain.processBrainCommand(command, pos);
//     EXPECT_EQ(result, 0);
    
//     EXPECT_EQ(pos, 9);
//     EXPECT_EQ(robotbrain.reply.len(), 7);
//     EXPECT_EQ(robotbrain.reply.get(0), 1);
//     EXPECT_EQ(robotbrain.reply.get(1), 0xCC);
//     EXPECT_EQ(robotbrain.reply.get(2), 1);
//     EXPECT_EQ(robotbrain.reply.get(3), 1);
//     EXPECT_EQ(robotbrain.reply.get(4), 17);
//     EXPECT_EQ(robotbrain.reply.get(5), 0);
//     EXPECT_EQ(robotbrain.reply.get(6), 236);
//     EXPECT_EQ(robotbrain.error, 0);
// }

// TEST(process_brain_command, multiple_chain){
//     RobotBrain robotbrain;
//     int result = robotbrain.registerChain(1, 0);
//     EXPECT_EQ(result, 1);

//     result = robotbrain.registerChain(2, 1);
//     EXPECT_EQ(result, 2);

//     byte command[20] = {2, 0xCC, 1, 2, 0x11, 0, 0x10,1, 90, 0xCC, 2, 2, 0x11, 0, 0x10,1, 90};
//     int pos = 0;
//     result = robotbrain.processBrainCommand(command, pos);
//     EXPECT_EQ(result, 0);
    
//     EXPECT_EQ(pos, 17);
//     EXPECT_EQ(robotbrain.reply.len(), 13);
//     EXPECT_EQ(robotbrain.reply.get(0), 2);
//     EXPECT_EQ(robotbrain.reply.get(1), 0xCC);
//     EXPECT_EQ(robotbrain.reply.get(2), 1);
//     EXPECT_EQ(robotbrain.reply.get(3), 1);
//     EXPECT_EQ(robotbrain.reply.get(4), 17);
//     EXPECT_EQ(robotbrain.reply.get(5), 0);
//     EXPECT_EQ(robotbrain.reply.get(6), 236);
//     EXPECT_EQ(robotbrain.reply.get(7), 0xCC);
//     EXPECT_EQ(robotbrain.reply.get(8), 2);
//     EXPECT_EQ(robotbrain.reply.get(9), 1);
//     EXPECT_EQ(robotbrain.reply.get(10), 17);
//     EXPECT_EQ(robotbrain.reply.get(11), 0);
//     EXPECT_EQ(robotbrain.reply.get(12), 236);
//     EXPECT_EQ(robotbrain.error, 0);
// }

// TEST(process_brain_command, error){
//     RobotBrain robotbrain;
//     int result = robotbrain.registerChain(1, 0);
//     EXPECT_EQ(result, 1);

//     byte command[20] = {1, 0xCD, 1, 2, 0x11, 0, 0x10,1, 90};
//     int pos = 0;
//     result = robotbrain.processBrainCommand(command, pos);
//     EXPECT_EQ(result, -1);
    
//     EXPECT_EQ(pos, 2);
//     EXPECT_EQ(robotbrain.reply.len(), 7);
//     EXPECT_EQ(robotbrain.reply.get(0), 74);
//     EXPECT_EQ(robotbrain.reply.get(1), 252);
//     EXPECT_EQ(robotbrain.reply.get(2), 0);
//     EXPECT_EQ(robotbrain.reply.get(3), 1);
//     EXPECT_EQ(robotbrain.reply.get(4), 255);
//     EXPECT_EQ(robotbrain.reply.get(5), 0xCD);
//     EXPECT_EQ(robotbrain.reply.get(6), 0);
//     EXPECT_EQ(robotbrain.error, -1);
    

// }

// TEST(process_command, multiple_chain){
//     RobotBrain robotbrain;
//     int result = robotbrain.registerChain(1, 0);
//     EXPECT_EQ(result, 1);

//     result = robotbrain.registerChain(2, 1);
//     EXPECT_EQ(result, 2);

//     byte command[20] = {0x4a, 0, 2, 0xCC, 1, 2, 0x11, 0, 0x10,1, 90, 0xCC, 2, 2, 0x11, 0, 0x10,1, 90};
    
//     result = robotbrain.processCommand(command);
//     EXPECT_EQ(result, 0);
    
//     EXPECT_EQ(robotbrain.reply.len(), 17);
//     EXPECT_EQ(robotbrain.reply.get(0), 74);
//     EXPECT_EQ(robotbrain.reply.get(1), 252);
//     EXPECT_EQ(robotbrain.reply.get(2), 0);
//     EXPECT_EQ(robotbrain.reply.get(3), 0);
//     EXPECT_EQ(robotbrain.reply.get(4), 2);
//     EXPECT_EQ(robotbrain.reply.get(5), 0xCC);
//     EXPECT_EQ(robotbrain.reply.get(6), 1);
//     EXPECT_EQ(robotbrain.reply.get(7), 1);
//     EXPECT_EQ(robotbrain.reply.get(8), 17);
//     EXPECT_EQ(robotbrain.reply.get(9), 0);
//     EXPECT_EQ(robotbrain.reply.get(10), 236);
//     EXPECT_EQ(robotbrain.reply.get(11), 0xCC);
//     EXPECT_EQ(robotbrain.reply.get(12), 2);
//     EXPECT_EQ(robotbrain.reply.get(13), 1);
//     EXPECT_EQ(robotbrain.reply.get(14), 17);
//     EXPECT_EQ(robotbrain.reply.get(15), 0);
//     EXPECT_EQ(robotbrain.reply.get(16), 236);
//     EXPECT_EQ(robotbrain.error, 0);
// }

// TEST(process_command, error_commnad){
//     RobotBrain robotbrain;
//     int result = robotbrain.registerChain(1, 0);
//     EXPECT_EQ(result, 1);

//     result = robotbrain.registerChain(2, 1);
//     EXPECT_EQ(result, 2);

//     byte command[20] = {0x4B, 0, 2, 0xCC, 1, 2, 0x11, 0, 0x10,1, 90, 0xCC, 2, 2, 0x11, 0, 0x10,1, 90};
    
//     result = robotbrain.processCommand(command);
//     EXPECT_EQ(result, -1);
    
//     EXPECT_EQ(robotbrain.reply.len(), 7);
//     EXPECT_EQ(robotbrain.reply.get(0), 74);
//     EXPECT_EQ(robotbrain.reply.get(1), 252);
//     EXPECT_EQ(robotbrain.reply.get(2), 0);
//     EXPECT_EQ(robotbrain.reply.get(3), 1);
//     EXPECT_EQ(robotbrain.reply.get(4), 255);
//     EXPECT_EQ(robotbrain.reply.get(5), 255);
//     EXPECT_EQ(robotbrain.reply.get(6), 0x4B);
//     EXPECT_EQ(robotbrain.error, -1);
// }
#endif