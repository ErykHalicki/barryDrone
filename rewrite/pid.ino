float P_PID=0.34, I_PID=0.001, D_PID=0.08;
float yawRate = 5.0;
int lastError[3]={0,0,0};
float errorAngleI[3] = {0,0,0};
int32_t deltasum[3];
long lastTime=0;
static float errorGyroI[3] = {0,0,0};

void pid(){
	uint8_t axis;
  	float errorAngle;
  	float AngleRateTmp, RateError;
  	float PTerm,ITerm,DTerm;
  	int16_t delta;
  
      //----------PID controller----------
	Serial.print(angle[0]);
	Serial.print(" ");
	Serial.print(angle[1]);
	Serial.print(" ");
	Serial.println(angle[2]);
	for(axis=0;axis<3;axis++) {
		if (axis == 2){ //YAW is always gyro-controlled 
          	AngleRateTmp = yawRate * rcCommand[YAW];
          	RateError = AngleRateTmp - gyroData[axis];
          	PTerm = RateError * P_PID;
          
          	delta           = RateError - lastError[axis];  
          	lastError[axis] = RateError;
          	deltasum[axis] += delta;
          	DTerm = deltasum[axis] * D_PID;
         
          	ITerm = 0.0;
        }
            // calculate error and limit the angle to 45 degrees max inclination
		else{
          	errorAngle = constrain(rcCommand[axis],-400,+400) - angle[axis]; //16 bits is ok here           
			// TODO check the angle readings, is is the scale from -1000 to +1000?
            PTerm = errorAngle * P_PID;
			DTerm = (errorAngle-lastError[axis])/max((long)1,(long)(millis()-lastTime)) * D_PI; 
            errorAngleI[axis]  += errorAngle * I_PID;
            ITerm = errorAngleI[axis];
			lastError[axis] = errorAngle;//update last error
		}
        PID[axis] =  PTerm + ITerm + DTerm;
      }
	lastTime=millis();
}

void zeroGyroAccI()
{
  for(int axis=0;axis<3;axis++) 
  {
    errorAngleI[axis] = 0.0;
    errorGyroI[axis] = 0.0;
  } 
}
