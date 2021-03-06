/*
 * The library produced by the company was very much not up to par with any standards, so
 * this is my adaptation of the original Meccano library found on their website.
 * Feel free to use it for whatever you want. (;
 *
 * @author Alex Frederiksen
 */

#ifndef Meccanoid_h
#define Meccanoid_h
#pragma once

/* for "byte" type and others */
#include "Arduino.h"

#define MAX_CHAIN 4

#define BIT_DELAY 417

#define TYPE_NONE  0x00
#define TYPE_SERVO 0x01
#define TYPE_LED   0x02

#define HEADER_BYTE  0xFF
#define NOMOD_BYTE   0xFE
#define NEWMOD_BYTE  0xFE
#define ERASE_BYTE   0xFD
#define REQUEST_BYTE 0xFC
#define NIL_BYTE     0xFB
#define LIM_BYTE     0xFA

#define SERVO_MIN 0x18
#define SERVO_MAX 0xE8

#define SERVO_COLOR_OFF 0x00
#define SERVO_COLOR_BLUE 0x01
#define SERVO_COLOR_GREEN 0x02
#define SERVO_COLOR_CYAN 0x03
#define SERVO_COLOR_RED 0x04
#define SERVO_COLOR_PURPLE 0x005
#define SERVO_COLOR_YELLOW 0x06
#define SERVO_COLOR_WHITE 0x07
#define SERVO_COLOR_KEEP 0x08


class ServoAdapter;
class LedAdapter;

/* convenience typedefs for users */
typedef ServoAdapter MeccanoServo;
typedef LedAdapter   MeccanoLed;

struct Module {
	int connected = 0;
	int justConnected = 0;

	byte lastInput = 0x00;
	byte type = TYPE_NONE;

	byte output1 = NIL_BYTE;
	byte output2 = NIL_BYTE;

	void onInput(byte input);
	byte getOutput();
	int isConnected();
	void connect(byte type);
	void disconnect();
};

class Chain {
	protected:
		int pwmPin;
		int moduleNum = 0;
		byte outputs[MAX_CHAIN];
		Module modules[MAX_CHAIN];

		byte calculateCheckSum();
		void sendByte(byte data);
		byte receiveByte();

	public:
		Chain(int pwmPin);
		void update();
		
		byte getType(int pos);
		MeccanoServo* getServo(int id);
		MeccanoLed* getLed(int id);
};

class ModuleAdapter {

	protected:
		Module & module;
		Chain & chain;
		bool autoUpdate = true;

	public:
		ModuleAdapter(Module & module, Chain & chain);

		void setAutoUpdate(bool enable);
		int isConnected();
		int justConnected();
		virtual int checkType() = 0;

};

class ServoAdapter : public ModuleAdapter {

	public:
		ServoAdapter(Module & module, Chain & chain);

		int getPosition();
		//int getPositionSmart();
		ServoAdapter & setPosition(int pos);
		//ServoAdapter & setPositionSmart(int pos);
		ServoAdapter & setLim(int enable);
		ServoAdapter & setColor(byte color);
		byte getColor();
		void setPositionMin(int pos);
		void setPositionMax(int pos);
		bool getLimStatus();
		int checkType();

	protected:
		ServoAdapter & setColor(byte r, byte g, byte b);

	protected:
		bool lim_status=false;
		byte min_pos= SERVO_MIN;
		byte max_pos= SERVO_MAX;
		byte position;
		byte color=SERVO_COLOR_OFF;
};

class LedAdapter : public ModuleAdapter {

	public:
		LedAdapter(Module & module, Chain & chain);

		LedAdapter & setColor(byte r, byte g, byte b, byte fadetime);

		int checkType();

};


#endif