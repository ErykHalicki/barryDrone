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
struct DiagnosticData{
	int PID[3];
	int servo[4];
	float gyroData[6];//roll, pitch, yaw(gyro), acc xyz
	bool landing;
};
uint8_t destAddress[]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

