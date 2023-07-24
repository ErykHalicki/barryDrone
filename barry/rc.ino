static uint16_t servo[4];

void buf_to_rc()
{
	if(cross)rcValue[THR]=1110;
	else if(sqr)rcValue[THR]=1250;
	else if(tri)rcValue[THR]=1300;
	else rcValue[THR]=map(max(0,leftStickY),0,127,1000,1800);
	rcValue[RUD]=map(leftStickX,-127,127,1000,2000);
	rcValue[ROL]=map(rightStickX,-127,127,1000,2000);
	rcValue[PIT]=map(rightStickY,-127,127,1000,2000);
}

void mix()
{
  if (armed & (rcValue[THR] > MINTHROTTLE))
  {
    servo[0] = constrain(rcValue[THR] - axisPID[ROLL] + axisPID[PITCH] - axisPID[YAW],1000,2000);
	servo[1] = constrain(rcValue[THR] - axisPID[ROLL] - axisPID[PITCH] + axisPID[YAW],1000,2000);
    servo[2] = constrain(rcValue[THR] + axisPID[ROLL] + axisPID[PITCH] + axisPID[YAW],1000,2000);
    servo[3] = constrain(rcValue[THR] + axisPID[ROLL] - axisPID[PITCH] - axisPID[YAW],1000,2000);
  }
  else 
  { 
    axisPID[0] = 0; axisPID[1] = 0; axisPID[2] = 0;
    servo[0] = 1000; servo[1] = 1000; servo[2] = 1000; servo[3] = 1000;
  }
 }

void printOutputs(){
	Serial.println();
	Serial.print(servo[0]);
	Serial.print(" ");
	Serial.print(servo[1]);
	Serial.print(" ");
	Serial.print(servo[2]);
	Serial.print(" ");
	Serial.print(servo[3]);
	Serial.println();

}
//Signal: 14,25,16,18 Pull down: 32,26,5
const int MotPin0 = 25;  //BR
const int MotPin1 = 14;  //FR
const int MotPin2 = 18;  //BL
const int MotPin3 = 16;  //FL
const int MotChannel0 = 2;
const int MotChannel1 = 3;   
const int MotChannel2 = 4;
const int MotChannel3 = 5;
const int safety = 1400;//maxiumium motor output

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
