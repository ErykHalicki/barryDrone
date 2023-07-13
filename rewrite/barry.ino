#include<esp_now.h>
#include <Ps3Controller.h>
#include <WiFi.h>
#include <Wire.h>
//Signal: 14,25,16,18 Pull down: 32,26,5

extern int16_t accZero[3];
extern float yawRate;
extern float rollPitchRate;
extern float P_PID;
extern float I_PID;
extern float D_PID;
extern float P_Level_PID;
extern float I_Level_PID;
extern float D_Level_PID;


float PID[3]={0,0,0};
enum ang { ROLL,PITCH,YAW };
enum channels { THR,PIT,RUD,ROL };
static int16_t gyroADC[3];
static int16_t accADC[3];
static int16_t gyroData[3];
static float angle[2] = {0,0};  
extern int calibratingA;
static int rcValue[8];
static int16_t rcCommand[] = {0,0,0};

#define ACCRESO 4096
#define CALSTEPS 256 // gyro and acc calibration steps
#define MINTHROTTLE 1100
#define MIDRUD 1500
uint8_t destAddress[6];

volatile boolean recv;
static uint16_t servo[4];
const int MotPin0 = 25;  //BR
const int MotPin1 = 14;  //FR
const int MotPin2 = 18;  //BL
const int MotPin3 = 16;  //FL
const int MotChannel0 = 2;
const int MotChannel1 = 3;   
const int MotChannel2 = 4;
const int MotChannel3 = 5;
const int safety = 1300;

const bool using_espnow=false;
struct ControllerData {
  bool calibrating,cross;
  bool tri;
  bool sqr;
  bool circ;
  bool leftTrigger;
  bool rightTrigger;
  bool leftShoulder;
  bool rightShoulder;
  int leftStickX;
  int leftStickY;
  int rightStickX;
  int rightStickY;
  float gyroXOffset;
  float gyroYOffset;
  float gyroX;
  float gyroY;
 };

ControllerData cData;

void writeServo() 
{
  ledcWrite(MotChannel0, uint16_t(min(safety,int(servo[0]))));
  ledcWrite(MotChannel1, uint16_t(min(safety,int(servo[1]))));
  ledcWrite(MotChannel2, uint16_t(min(safety,int(servo[2]))));
  ledcWrite(MotChannel3, uint16_t(min(safety,int(servo[3]))));
}

void initServo() 
{
  ledcSetup(MotChannel0, 500, 11); // 500 hz PWM, 11-bit resolution
  ledcSetup(MotChannel1, 500, 11); // 500 hz PWM, 11-bit resolution
  ledcSetup(MotChannel2, 500, 11); // 500 hz PWM, 11-bit resolution
  ledcSetup(MotChannel3, 500, 11); // 500 hz PWM, 11-bit resolution
  ledcAttachPin(MotPin0, MotChannel0); 
  ledcAttachPin(MotPin1, MotChannel1); 
  ledcAttachPin(MotPin2, MotChannel2); 
  ledcAttachPin(MotPin3, MotChannel3); 
}

void mix(){
  if (rcValue[THR] > MINTHROTTLE){
    servo[0] = constrain(rcValue[THR] - PID[ROLL] + PID[PITCH] - PID[YAW],1000,2000);
	servo[1] = constrain(rcValue[THR] - PID[ROLL] - PID[PITCH] + PID[YAW],1000,2000);
    servo[2] = constrain(rcValue[THR] + PID[ROLL] + PID[PITCH] + PID[YAW],1000,2000);
    servo[3] = constrain(rcValue[THR] + PID[ROLL] - PID[PITCH] - PID[YAW],1000,2000);
  }
  else 
  { 
    PID[0] = 0; PID[1] = 0; PID[2] = 0;
    servo[0] = 1000; servo[1] = 1000; servo[2] = 1000; servo[3] = 1000;
  }
}
void setup(){
	Serial.begin(9600);
	Serial.println("BARRY Startup...");
	if(!using_espnow){
		Ps3.begin();
		Ps3.attach(update);
		Ps3.attachOnConnect(onConnect);
	}
	else{
		WiFi.mode(WIFI_MODE_STA);
  		Serial.println(WiFi.macAddress());
		esp_now_init();
		esp_now_register_recv_cb(receiveControllerData);
	}
	delay(3000); // give it some time to stop shaking after battery plugin
  	MPU6050_init();
  	MPU6050_readId(); // must be 0x68, 104dec
	pinMode(32, INPUT_PULLDOWN);  
	pinMode(26, INPUT_PULLDOWN);  
	pinMode(5, INPUT_PULLDOWN);  
  	delay(500); 
  	initServo();
}
void loop(){
	pid();
	rcValue[THR]=map(max(0,cData.leftStickY),0,127,1000,1800);
	rcValue[RUD]=map(cData.leftStickX,-127,127,1000,2000);
	rcValue[ROL]=map(cData.rightStickX,-127,127,1000,2000);
	rcValue[PIT]=map(cData.rightStickY,-127,127,1000,2000);

	Serial.print(angle[0]);
	Serial.print(" ");
	Serial.println(angle[1]);
}
