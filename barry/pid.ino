float yawRate = 5.0;
float rollPitchRate = 5.0;

float P_PID = 0.14;    // P8
float I_PID = 0.00;    // I8
float D_PID = 0.08;    // D8

//0.8 0.01 0.5 a little shaky, on the edge
//0.8 0.01 0.9 a little shaky, good to fly

float P_Level_PID = 0.40;   // P8
float I_Level_PID = 0.01;   // I8
float D_Level_PID = 0.05;   // D8

static int16_t axisPID[3];
static int16_t lastError[3] = {0,0,0};
static float errorGyroI[3] = {0,0,0};
static float errorAngleI[3] = {0,0,0};

//----------PID controller----------
    
#define GYRO_I_MAX 10000.0
#define ANGLE_I_MAX 6000.0

int plotct;
int16_t deltab[6][3];
int8_t  deltabpt = 0;
int32_t deltasum[3];
long lastTime=0;
void pid()
{
  uint8_t axis;
  float errorAngle;
  float AngleRateTmp, RateError;
  float PTerm,ITerm,DTerm;
  int16_t delta;
  
      //----------PID controller----------
      for(axis=0;axis<3;axis++) 
      {
        if (axis == 2) 
        { //YAW is always gyro-controlled 
          AngleRateTmp = yawRate * rcCommand[YAW];
          RateError = AngleRateTmp - gyroData[axis];
          PTerm = RateError * P_PID;
          
          delta           = RateError - lastError[axis];  
          lastError[axis] = RateError;
          deltasum[axis] += delta;
          deltasum[axis] -= deltab[deltabpt][axis];
          deltab[deltabpt][axis] = delta;  
          DTerm = deltasum[axis] * D_PID;
         
          ITerm = 0.0;
          
          deltabpt++;
          if (deltabpt >= 6) deltabpt = 0;
        } 
        else 
        {//not sure what the purpose of gyro mode is
          if (flightmode == GYRO) // GYRO mode 
          { 
            //control is GYRO based (ACRO - direct sticks control is applied to rate PID
            AngleRateTmp = rollPitchRate * rcCommand[axis];//intended output * coefficent
            RateError = AngleRateTmp - gyroData[axis];//prev line - gyroOutput
            //-----calculate P-term
            PTerm = RateError * P_PID;//error*coeffcient
            //-----calculate D-term
            delta           = RateError - lastError[axis];//diff between last error and current error 
            lastError[axis] = RateError;//update last error

            deltasum[axis] += delta;//adding up differences,
            deltasum[axis] -= deltab[deltabpt][axis];//not sure what deltab is array of past delta values, deltabpt is the point in the array that is being read?
			// something isnt making sense here
            deltab[deltabpt][axis] = delta;
            
            DTerm = deltasum[axis] * D_PID;//sum of deltas like an integral? why is it mulitplied by the derivative coefficient
            //-----calculate I-term
            ITerm = 0.0;// no integral because gyro data is instantaneous i think
          }
          else // STABI mode
          {
            // calculate error and limit the angle to 45 degrees max inclination
            errorAngle = constrain(rcCommand[axis],-400,+400) - angle[axis]; //16 bits is ok here           
			
			// TODO check the angle readings, is is the scale from -1000 to +1000?

            //it's the ANGLE mode - control is angle based, so control loop is needed
            //-----calculate P-term
            PTerm = errorAngle * P_Level_PID;
            //-----calculate D-term
            delta = (errorAngle-lastError[axis])/(millis()-lastTime); // why is the derivative set to the negative gyroscope value? shouldnt it be something like:
			lastError[axis] = errorAngle;//update last error
//			derivative = (new value - old value)/time diff 

			//TODO check pid loop pseudocode on wikipedia
			//test derivative with time division and no time division
			//adjust d coefficient, could be very different depending on time/no time

            DTerm = delta * D_Level_PID; 
            //-----calculate I-term
            errorAngleI[axis]  += errorAngle * I_Level_PID;
            errorAngleI[axis]  = constrain(errorAngleI[axis], -ANGLE_I_MAX, +ANGLE_I_MAX);
            ITerm = errorAngleI[axis] * 0.01;
          } 
        }
         
        //-----calculate total PID output
        axisPID[axis] =  PTerm + ITerm + DTerm;

        /*
        if (axis==2)
        {
          Serial.print(AngleRateTmp); Serial.print("  ");
          Serial.print(RateError); Serial.print("  ");
          Serial.print(PTerm); Serial.print("  ");
          Serial.println();
        }
        */
        /*
        if (axis==0)
        {
          Serial.print(PTerm); Serial.print("  ");
          Serial.print(ITerm); Serial.print("  ");
          Serial.print(DTerm); Serial.print("  ");
          if      (plotct == 0) Serial.print(-2000); 
          else if (plotct == 1) Serial.print( 2000); 
          else                  Serial.print( 0);
          if (plotct == 300) plotct = 0; else plotct++; 
          Serial.println();
        }
        */     
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
