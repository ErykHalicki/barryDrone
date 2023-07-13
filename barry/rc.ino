void buf_to_rc()
{
  uint8_t seq;
	if(cross)rcValue[THR]=1200;
	else rcValue[THR]=map(max(0,leftStickY),0,127,1000,1800);
	rcValue[RUD]=map(leftStickX,-127,127,1000,2000);
	rcValue[ROL]=map(rightStickX,-127,127,1000,2000);
	rcValue[PIT]=map(rightStickY,-127,127,1000,2000);

/*
	Serial.print(leftStickY);
	Serial.print(" ");
	Serial.print(leftStickX);
	Serial.print(" ");
	Serial.print(rightStickX);
	Serial.print(" ");
	Serial.println(rightStickY);

	Serial.print(rcValue[THR]);
	Serial.print(" ");
	Serial.print(rcValue[RUD]);
	Serial.print(" ");
	Serial.print(rcValue[ROL]);
	Serial.print(" ");
	Serial.println(rcValue[PIT]);

  rcValue[0] = RCdata.chans.Ch1;
  rcValue[1] = RCdata.chans.Ch2;
  rcValue[2] = RCdata.chans.Ch3;
  rcValue[3] = RCdata.chans.Ch4;
  rcValue[4] = RCdata.chans.Ch5;
  rcValue[5] = RCdata.chans.Ch6;
  rcValue[6] = RCdata.chans.Ch7;
  rcValue[7] = RCdata.chans.Ch8;
  seqno = RCdata.chans.spare;
*/
}

static uint16_t servo[4];

void mix()
{
  if (armed & (rcValue[THR] > MINTHROTTLE))
  {
    servo[0] = constrain(rcValue[THR] - axisPID[ROLL] + axisPID[PITCH] - axisPID[YAW],1000,2000);
	servo[1] = constrain(rcValue[THR] - axisPID[ROLL] - axisPID[PITCH] + axisPID[YAW],1000,2000);
    servo[2] = constrain(rcValue[THR] + axisPID[ROLL] + axisPID[PITCH] + axisPID[YAW],1000,2000);
    servo[3] = constrain(rcValue[THR] + axisPID[ROLL] - axisPID[PITCH] - axisPID[YAW],1000,2000);
/*
	Serial.println();
	Serial.print(servo[0]);
	Serial.print(" ");
	Serial.print(servo[1]);
	Serial.print(" ");
	Serial.print(servo[2]);
	Serial.print(" ");
	Serial.print(servo[3]);
	Serial.println();
*/
  }
  else 
  { 
    axisPID[0] = 0; axisPID[1] = 0; axisPID[2] = 0;
    servo[0] = 1000; servo[1] = 1000; servo[2] = 1000; servo[3] = 1000;
  }
  Serial.print(servo[0]); Serial.print("  ");
  Serial.print(servo[1]); Serial.print("  ");
  Serial.print(servo[2]); Serial.print("  ");
  Serial.print(servo[3]); Serial.println();
}


#if defined PWMOUT //----------------------------------------------

//Signal: 14,25,16,18 Pull down: 32,26,5
const int MotPin0 = 25;  //BR
const int MotPin1 = 14;  //FR
const int MotPin2 = 18;  //BL
const int MotPin3 = 16;  //FL
const int MotChannel0 = 2;
const int MotChannel1 = 3;   
const int MotChannel2 = 4;
const int MotChannel3 = 5;
const int safety = 1400;
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

#else //----------------------------------------------

uint8_t outmsg[5];

void writeServo()
{
  outmsg[0] = 0xF5;
  outmsg[1] = constrain((servo[0]-1000)>>2,0,0xF4);
  outmsg[2] = constrain((servo[1]-1000)>>2,0,0xF4);
  outmsg[3] = constrain((servo[2]-1000)>>2,0,0xF4);
  outmsg[4] = constrain((servo[3]-1000)>>2,0,0xF4);
  Serial1.write(outmsg,5);
}

void initServo()
{
  Serial1.begin(128000);
}

#endif //----------------------------------------------
