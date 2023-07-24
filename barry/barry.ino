#include <esp_now.h>
#include <Ps3Controller.h>
#include <WiFi.h>
#include <Wire.h>
#include <EEPROM.h>
#include "MPU6050_light.h"
#include "encoding.h"

#define WIFI_CHANNEL 4
//#define espnow

extern float P_Gyro;
extern float I_Gyro;
extern float D_Gyro;
extern float P_Angle;
extern float I_Angle;
extern float D_Angle;

#define ACCRESO 4096
#define CYCLETIME 10
#define MINTHROTTLE 1100
#define MIDRUD 1500
#define THRCORR 0

enum ang { ROLL,PITCH,YAW };

static int16_t gyroADC[3];
static int16_t accADC[3];
static int16_t gyroData[3];
static float angle[2]    = {0,0};  
extern int calibratingA;

long lastUpdate;
#define ROL 0
#define PIT 1
#define THR 2
#define RUD 3

static int16_t rcCommand[] = {0,0,0};
static int16_t rcValue[4];  // in us, center = 1500
#define GYRO     0
#define STABI    1
static int8_t flightmode;
static int8_t oldflightmode;

boolean armed = false;
uint8_t armct = 0;
int debugvalue = 0;
MPU6050 mpu(Wire);

void setup(){
  	Serial.begin(9600); Serial.println();
	delay(3000); // give it some time to stop shaking after battery plugin
	Wire.begin();
	byte status = mpu.begin();
	while(status!=0){
		Serial.println("MPU6050 Failed to start, trying again");
		delay(1000);
		status = mpu.begin();
	}
	Serial.println(F("Calculating offsets, do not move MPU6050"));
  	delay(1000);
  	mpu.calcOffsets(true,true); // gyro and accelero
	Serial.println("MPU6050 Success");

	#ifdef espnow 
		WiFi.mode(WIFI_STA);
  		Serial.println(WiFi.macAddress());
		esp_now_init();
		esp_now_register_recv_cb(receiveControllerData);
	#else
		Ps3.begin();
		Ps3.attach(update);
		Ps3.attachOnConnect(onConnect);
	#endif

  	//EEPROM.begin(64);
  	//if (EEPROM.read(62) != 0xAA) Serial.println("Need to check and write PID");
  	//else PID_Read(); // eeprom is initialized

	//Signal: 14,25,16,18 Pull down: 32,26,5
	pinMode(32, INPUT_PULLDOWN);  
	pinMode(26, INPUT_PULLDOWN);  
	pinMode(5, INPUT_PULLDOWN);  
  	delay(500); 
  	initServo();
}

uint32_t rxt; // receive time, used for falisave

void loop(){
  	uint32_t now,mnow,diff; 
  	now = millis(); // actual time
  	if (debugvalue == 5) mnow = micros();
  	if (millis()-lastUpdate<100){
    	buf_to_rc();
    	if (debugvalue == 4) Serial.printf("%4d %4d %4d %4d \n", rcValue[0], rcValue[1], rcValue[2], rcValue[3]); 
    	if (armed) {
      		rcValue[THR]    -= THRCORR;
      		rcCommand[ROLL]  = rcValue[ROL] - MIDRUD;
      		rcCommand[PITCH] = rcValue[PIT] - MIDRUD;
      		rcCommand[YAW]   = rcValue[RUD] - MIDRUD;
    	}  
    	else{  
      		if (rcValue[THR] < MINTHROTTLE) armct++;
      		if (armct >= 25)armed = true;
    	}
    	rxt = millis();
	}

  	pid();
	mix();
	mpu.update();
  	writeServo();
		//printOutputs();	
  	// Failsave part
  	if (now > rxt+90){
		//future self land emergency fuction
    	rcValue[THR] = MINTHROTTLE*1.2;
    	rxt = now;
  	}
  	delay(CYCLETIME-1);  

  	if (debugvalue == 5){
    	diff = micros() - mnow;
    	Serial.println(diff); 
  	}
}

int readsernum(){
  int num;
  char numStr[3]; 
  numStr[0] = Serial.read();
  numStr[1] = Serial.read();
  return atol(numStr);
}
