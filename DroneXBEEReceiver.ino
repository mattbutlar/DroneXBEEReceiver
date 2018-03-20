#include <TimerOne.h> 
#include <SoftwareSerial.h>

#define DEBUG 0

#define THROTTLE 0
#define ROLL 1
#define PITCH 2
#define YAW 3
#define AUX1 4
#define AUX2 5
#define AUX3 6
#define AUX4 7

#define CHANNEL1_BREAK   0x50
#define CHANNEL2_BREAK   0x51
#define CHANNEL3_BREAK   0x52
#define CHANNEL4_BREAK   0x53
#define CHANNEL5_BREAK   0x54
#define CHANNEL6_BREAK   0x55
#define CHANNEL7_BREAK   0x56
#define CHANNEL8_BREAK   0x57

#define RC_CHANNEL_MIN 990
#define RC_CHANNEL_MAX 2010

#define SBUS_MIN_OFFSET 173
#define SBUS_MID_OFFSET 992
#define SBUS_MAX_OFFSET 1811
#define SBUS_CHANNEL_NUMBER 16
#define SBUS_PACKET_LENGTH 25
#define SBUS_FRAME_HEADER 0x0f
#define SBUS_FRAME_FOOTER 0x00
#define SBUS_FRAME_FOOTER_V2 0x04
#define SBUS_STATE_FAILSAFE 0x08
#define SBUS_STATE_SIGNALLOSS 0x04
#define SBUS_UPDATE_RATE 15 //ms

#define RSSI_PIN 5

#define FAILSAFE_TIMEOUT 2000

SoftwareSerial xbeeSerial(9, 8);

byte breaks[8] = {CHANNEL1_BREAK, CHANNEL2_BREAK, CHANNEL3_BREAK, CHANNEL4_BREAK, CHANNEL5_BREAK, CHANNEL6_BREAK, CHANNEL7_BREAK, CHANNEL8_BREAK};

byte byteBuffer[2] = {0};
uint16_t rc[SBUS_CHANNEL_NUMBER] = { 1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000 };

byte incomingByte;

char channel = 0;

uint8_t sbusPacket[SBUS_PACKET_LENGTH];
uint32_t sbusTime = 0;

uint32_t currentMillis = 0;
uint32_t failsafeMillis = 0;
boolean failsafe = false;

boolean validByte = false;

void sbusPreparePacket(uint8_t packet[], int channels[], bool isSignalLoss, bool isFailsafe){

    static int output[SBUS_CHANNEL_NUMBER] = {0};

    /*
     * Map 1000-2000 with middle at 1500 chanel values to
     * 173-1811 with middle at 992 S.BUS protocol requires
     */
    for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
        output[i] = map(channels[i], RC_CHANNEL_MIN, RC_CHANNEL_MAX, SBUS_MIN_OFFSET, SBUS_MAX_OFFSET);
    }

    uint8_t stateByte = 0x00;
    if (isSignalLoss) {
        stateByte |= SBUS_STATE_SIGNALLOSS;
    }
    if (isFailsafe) {
        stateByte |= SBUS_STATE_FAILSAFE;
    }
    packet[0] = SBUS_FRAME_HEADER; //Header

    packet[1] = (uint8_t) (output[0] & 0x07FF);
    packet[2] = (uint8_t) ((output[0] & 0x07FF)>>8 | (output[1] & 0x07FF)<<3);
    packet[3] = (uint8_t) ((output[1] & 0x07FF)>>5 | (output[2] & 0x07FF)<<6);
    packet[4] = (uint8_t) ((output[2] & 0x07FF)>>2);
    packet[5] = (uint8_t) ((output[2] & 0x07FF)>>10 | (output[3] & 0x07FF)<<1);
    packet[6] = (uint8_t) ((output[3] & 0x07FF)>>7 | (output[4] & 0x07FF)<<4);
    packet[7] = (uint8_t) ((output[4] & 0x07FF)>>4 | (output[5] & 0x07FF)<<7);
    packet[8] = (uint8_t) ((output[5] & 0x07FF)>>1);
    packet[9] = (uint8_t) ((output[5] & 0x07FF)>>9 | (output[6] & 0x07FF)<<2);
    packet[10] = (uint8_t) ((output[6] & 0x07FF)>>6 | (output[7] & 0x07FF)<<5);
    packet[11] = (uint8_t) ((output[7] & 0x07FF)>>3);
    packet[12] = (uint8_t) ((output[8] & 0x07FF));
    packet[13] = (uint8_t) ((output[8] & 0x07FF)>>8 | (output[9] & 0x07FF)<<3);
    packet[14] = (uint8_t) ((output[9] & 0x07FF)>>5 | (output[10] & 0x07FF)<<6);  
    packet[15] = (uint8_t) ((output[10] & 0x07FF)>>2);
    packet[16] = (uint8_t) ((output[10] & 0x07FF)>>10 | (output[11] & 0x07FF)<<1);
    packet[17] = (uint8_t) ((output[11] & 0x07FF)>>7 | (output[12] & 0x07FF)<<4);
    packet[18] = (uint8_t) ((output[12] & 0x07FF)>>4 | (output[13] & 0x07FF)<<7);
    packet[19] = (uint8_t) ((output[13] & 0x07FF)>>1);
    packet[20] = (uint8_t) ((output[13] & 0x07FF)>>9 | (output[14] & 0x07FF)<<2);
    packet[21] = (uint8_t) ((output[14] & 0x07FF)>>6 | (output[15] & 0x07FF)<<5);
    packet[22] = (uint8_t) ((output[15] & 0x07FF)>>3);

    packet[23] = stateByte; //Flags byte
    packet[24] = SBUS_FRAME_FOOTER; //Footer
}

void setup()
{
  pinMode(RSSI_PIN, INPUT);
  
  if (DEBUG) {
    Serial.begin(38400);
  }
  
  xbeeSerial.begin(38400);

  if (DEBUG) {
    Serial.print("Initialize");
  }

  Serial1.begin(100000, SERIAL_8E2);
}

void loop()
{
  currentMillis = millis();

  rc[15] = pulseIn(RSSI_PIN, LOW, 200);
  
	if (xbeeSerial.available() > 0) {
    failsafeMillis = currentMillis + FAILSAFE_TIMEOUT;
		incomingByte = xbeeSerial.read();

    channel = getChannel(incomingByte);

    if (channel > -1) {
      byteBuffer[0] = xbeeSerial.read();
      byteBuffer[1] = xbeeSerial.read();
      
      rc[channel] = getInt(byteBuffer[0], byteBuffer[1]);
      
      if (DEBUG) {
        Serial.print("Channel: ");
        Serial.print((channel + 1));
        Serial.print(" - ");
        Serial.print(rc[channel], DEC);
        Serial.println("");
        validByte = true;
      }
    } else {
      if (DEBUG && validByte) {
        Serial.println("Bad Start");
        validByte = false;
      }
    }
	}

  if (currentMillis > sbusTime) {
    if (currentMillis > failsafeMillis) {
      failsafe = true;
    } else {
      failsafe = false;
    }
    
    sbusPreparePacket(sbusPacket, rc, failsafe, failsafe);
    
    Serial1.write(sbusPacket, SBUS_PACKET_LENGTH);

    sbusTime = currentMillis + SBUS_UPDATE_RATE;
  }
}

char getChannel(byte incoming) {
  for (char i = 0; i < sizeof(breaks); i++) {
    if (incoming == breaks[i]) {
      return i;
    }
  }

  return -1;
}

byte getMSB(uint16_t value) {
  return (value >> 8);
}

byte getLSB(uint16_t value) {
  return (value & 0x00FF);
}

uint16_t getInt(byte MSB, byte LSB) {
  return (MSB << 8) | LSB;
}
