// Visual Micro is in vMicro>General>Tutorial Mode
/*
    Name:       IR_receiver.ino
    Created:	26/04/2021 12:38:32
    Author:     DESKTOP-QP4J4VJ\bruno
*/

// Define User Types below here or use a .h file
#include <SoftwareSerial.h>
#include "IRremote.h"
#define IR_PIN 11
#define RX_PIN 7 //input pin
#define TX_PIN 8 //output pin

// new serial channel for communication 
SoftwareSerial toSTM = SoftwareSerial(RX_PIN, TX_PIN);
IRrecv irrecv(IR_PIN);     // create instance of 'irrecv'
decode_results results;      // create instance of 'decode_results'

uint8_t cmd;

//function which reads from IR sensor, send a command on STM32 serial channel 
void translateIR() // takes action based on IR code received
{
	switch (results.value) {
	case 0xFFA25D:
		Serial.println("ON/OFF");
		cmd = 100;
		break;
	case 0xFFE21D: 
		Serial.println("FUNC/STOP");
		cmd = 101;
		break;
	case 0xFF629D: 
		Serial.println("VOL+");
		cmd = 102;
		break;
	case 0xFF22DD: 
		Serial.println("FAST BACK");
		cmd = 103;
		break;
	case 0xFF02FD: 
		Serial.println("PAUSE");
		cmd = 104;
		break;
	case 0xFFC23D: 
		Serial.println("FAST FORWARD");
		cmd = 105;
		break;
	case 0xFFE01F: 
		Serial.println("DOWN");
		cmd = 106;
		break;
	case 0xFFA857: 
		Serial.println("VOL-");
		cmd = 107;
		break;
	case 0xFF906F:
		Serial.println("UP");
		cmd = 108;
		break;
	case 0xFF9867: 
		Serial.println("EQ");
		cmd = 109;
		break;
	case 0xFFB04F: 
		Serial.println("ST/REPT");
		cmd = 110;
		break;
	case 0xFF6897:
		Serial.println("0");
		cmd = 0;
		break;
	case 0xFF30CF:
		Serial.println("1");
		cmd = 1;
		break;
	case 0xFF18E7:
		Serial.println("2");
		cmd = 2;
		break;
	case 0xFF7A85:
		Serial.println("3");
		cmd = 3;
		break;
	case 0xFF10EF:
		Serial.println("4");
		cmd = 4;
		break;
	case 0xFF38C7:
		Serial.println("5");
		cmd = 5;
		break;
	case 0xFF5AA5: 
		Serial.println("6");   
		cmd = 6;
		break;
	case 0xFF42BD: 
		Serial.println("7"); 
		cmd = 7;
		break;
	case 0xFF4AB5: 
		Serial.println("8");
		cmd = 8;
		break;
	case 0xFF52AD: 
		Serial.println("9");   
		cmd = 9;
		break;
	default: 
		Serial.println("NO BUTTON");
	}
	toSTM.write(cmd);
	delay(300);
}

void setup()
{
	pinMode(RX_PIN, INPUT); 
	pinMode(TX_PIN, OUTPUT); 
	Serial.begin(9600);
	toSTM.begin(9600);
	irrecv.enableIRIn();
	cmd = 255; //default value
}

void loop()
{
	if (irrecv.decode(&results)) // have we received an IR signal?
	{
		translateIR();
		irrecv.resume();
	}

}
