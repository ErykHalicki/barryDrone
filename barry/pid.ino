static int16_t axisPID[3];
float lastError[3] = {0, 0, 0};
float P_Gyro = 0.9;    // P8
float I_Gyro = 0.00;   // I8
float D_Gyro = 0.05;   // D8

float P_Angle = 0.5;  // P8
float I_Angle = 0.001; // I8
float D_Angle = 0.05;   // D8
float overallScale = 0.4; // PID output multiplier
float errorSum[3] = {0, 0, 0};
#define maxYawRate 180
#define maxTiltAngle 35
#define IMU_FILTER_SIZE 15 // Size of the filter window
float angleX=0,angleY=0,gyroZ=0;
float imuReadings[IMU_FILTER_SIZE][3];

void pid() {
  int16_t P, I, D;
  float error;

  // Update IMU readings and apply KZ filter
  for (int i = IMU_FILTER_SIZE - 1; i > 0; i--) {
    for (int j = 0; j < 3; j++) {
      imuReadings[i][j] = imuReadings[i - 1][j];
    }
  }
  imuReadings[0][0] = mpu.getAngleY();
  imuReadings[0][1] = mpu.getAngleX();
  imuReadings[0][2] = mpu.getGyroZ();

  // Apply KZ filter
  for (int j = 0; j < 3; j++) {
    float filteredValue = 0;
    for (int i = 0; i < IMU_FILTER_SIZE; i++) {
      filteredValue += imuReadings[i][j];
    }
    filteredValue /= IMU_FILTER_SIZE;
    // Use the filtered value for angle and gyro
    if (j==0) angleX = angleX*0.95+filteredValue*0.05;
    if (j==1) angleY = angleY*0.95+filteredValue*0.05;
    if (j==2) gyroZ = gyroZ*0.95+filteredValue*0.05;
  }

	//angleY-= max(0,(int)map(rcValue[THR],1300,1400,15,25));	

  // Roll control
  error = map(angleX, -maxTiltAngle, maxTiltAngle, -300, 300) - rcCommand[ROLL];
  errorSum[0] += error;
  P = error * P_Angle;
  I = errorSum[0] * I_Angle;
  D = (error - lastError[0]) * D_Angle;
  axisPID[0] = (P + I + D) * overallScale;
  lastError[0] = error;

  // Pitch control
  error = map(-angleY, -maxTiltAngle, maxTiltAngle, -300, 300) - rcCommand[PITCH];
  errorSum[1] += error;
  P = error * P_Angle;
  I = errorSum[1] * I_Angle;
  D = (error - lastError[1]) * D_Angle;
  axisPID[1] = (P + I + D) * overallScale;
  lastError[1] = error;

  // Yaw control
  error = map(-gyroZ, -maxYawRate, maxYawRate, -300,300) - rcCommand[YAW];
  P = error * P_Gyro;
  I = errorSum[2] * I_Gyro; // Note: There's no I term for Yaw in the original code
  D = (error - lastError[2]) * D_Gyro;
  axisPID[2] = (P + I + D) * overallScale;
  lastError[2] = error;

  // Output PID values for debugging
  Serial.print(axisPID[0]);
  Serial.print(" ");
  Serial.print(axisPID[1]);
  Serial.print(" ");
  Serial.println(axisPID[2]);
/*
	Serial.print(angleX);
  Serial.print(" ");
  Serial.print(angleY);
  Serial.print(" ");
  Serial.println(gyroZ);

*/
}
/*
static int16_t axisPID[3];
float lastError[]={0,0,0};
float P_Gyro = 0.9;    // P8
float I_Gyro = 0.00;    // I8
float D_Gyro = 0.05;    // D8

//0.8 0.01 0.5 a little shaky, on the edge
//0.8 0.01 0.9 a little shaky, good to fly

float P_Angle = 0.40;   // P8
float I_Angle = 0.001;  // I8
float D_Angle = 0.3;   // D8
float overallScale = 0.2; // PID output multiplier
float errorSum[]={0,0,0};
#define maxYawRate 180
#define maxTiltAngle 35
#define IMAX 10000
float angleY=0,angleX=0,gyroZ=0;
//pid tuning?
//check if vibration is still a big issue
//print covers for the proppellors
//get esp now data reporting working <--- very important
void pid(){
	int16_t P,I,D;
	float error;
	//roll=y pitch=x yaw=z
	angleY=0.95*angleY+mpu.getAngleY()*0.05;
	error=map(angleY,-maxTiltAngle,maxTiltAngle,-500,500)-rcCommand[ROLL];//adjust
	errorSum[ROLL]+=error;
	P=error*P_Angle;
	I=max(IMAX,errorSum[ROLL])*I_Angle;
	D=(error-lastError[ROLL])*D_Angle;	
	axisPID[ROLL]=(P+I+D)*overallScale;
	lastError[ROLL]=error;

	angleX=0.95*angleX+mpu.getAngleX()*0.05;
	error=map(-angleX,-maxTiltAngle,maxTiltAngle,-500,500)-rcCommand[PITCH];//adjust
	errorSum[PITCH]+=error;
	P=error*P_Angle;
	I=max(IMAX,errorSum[PITCH])*I_Angle;
	D=(error-lastError[PITCH])*D_Angle;	
	axisPID[PITCH]=(P+I+D)*overallScale;
	lastError[PITCH]=error;
	
	gyroZ=gyroZ*0.95+mpu.getGyroZ()*0.05;
	error=map(-gyroZ,-maxYawRate,maxYawRate,-500,500)-rcCommand[YAW];//adjust
	//errorSum[YAW]+=error;
	P=error*P_Gyro;
	I=max(IMAX,errorSum[YAW])*I_Gyro;//0 value for now
	D=(error-lastError[YAW])*D_Gyro;	
	axisPID[YAW]=(P+I+D)*overallScale;
	lastError[YAW]=error;
	Serial.print(axisPID[ROLL]);
	Serial.print(" ");
	Serial.print(axisPID[PITCH]);
	Serial.print(" ");
	Serial.print(axisPID[YAW]);
	Serial.print(" ");
	Serial.print(errorSum[ROLL]);
	Serial.print(" ");
	Serial.print(errorSum[PITCH]);
	Serial.print(" ");
	Serial.println(errorSum[YAW]);
}
*/
